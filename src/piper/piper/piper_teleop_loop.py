#!/usr/bin/env python3
# -*-coding:utf8-*-
"""Teleop loop node for Piper arms.

This node connects only to the local teacher arm via CAN.  The follower arm
lives on a separate workstation and is driven by a piper_ctrl_single_node (or
equivalent) that subscribes to the joint_ctrl topic published here — identical
to the mechanism used in piper_broadcast_master_v2.

Teach-mode exit sequence (from playTrajectory_new_en.py):
  - EmergencyStop(0x01) to halt the arm
  - Wait until joints 2, 3, 5 are within safe limits
  - EmergencyStop(0x02) to restore
  - Poll ModeCtrl until ctrl_mode == 1 (CAN mode)
  - EnablePiper() loop + GripperCtrl + ModeCtrl to re-enable

Workflow (loops indefinitely):
  1. Prompt operator to disable teacher mode; run exit sequence above.
  2. Publish the preset joint position on joint_ctrl so the remote follower
     moves to home.
  3. Prompt operator to enable teacher mode and press Enter when ready.
  4. Confirm ctrl_mode == 2.
  5. Call data_collector_begin() [placeholder].
  6. Read teacher joints at ~200 Hz and publish on joint_ctrl until Enter is
     pressed again.
  7. Call data_collector_end() [placeholder].
  8. Go to step 1.
"""

import os
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from piper_sdk import C_PiperInterface_V2
import numpy as np


def _wait_enter(prompt: str) -> None:
    """Print prompt and wait for Enter, reading from /dev/tty so that the
    call works correctly whether the process is started via ros2 run or
    ros2 launch (which redirects stdin away from the terminal)."""
    print(prompt, flush=True)
    try:
        with open('/dev/tty', 'r') as tty:
            tty.readline()
    except OSError:
        input()


# Unit conversion constants matching piper_broadcast_master_v2
RAW_TO_RAD = 0.017444       # (pi / 180) / 1000  — millidegrees → radians

# Safe joint limits used by the e-stop restore check (from playTrajectory_new_en.py)
# joints are 0-indexed: joint2=idx1, joint3=idx2, joint5=idx4
_SAFE_J2_ABS  = 0.1745   # rad  (~10°)
_SAFE_J3_ABS  = 0.1745   # rad  (~10°)
_SAFE_J5_MAX  = 0.7854   # rad  (~45°)
_SAFE_J5_MIN  = 0.2094   # rad  (~12°)


class TeleopLoopNode(Node):
    """ROS2 node that runs a teacher-arm teleoperation loop."""

    def __init__(self) -> None:
        super().__init__('piper_teleop_loop')

        self.declare_parameter('can_port', 'can0')
        self.declare_parameter('gripper_exist', True)
        self.declare_parameter('move_speed', 0xAD) # 1-100, passed to MotionCtrl_2 for follower movement, will be high-follow mode if 0xAD
        # Preset joint positions in radians (joints 1-6) + gripper in metres.
        # Published on joint_ctrl when resetting the remote follower to home.
        self.declare_parameter('preset_joints', [-0.02886982, 1.0887149280000001, -1.17476618, -0.040696852000000006, 1.194704672, -0.038743124000000004, 0.0])
        # Timeout for CAN mode confirmation after exiting teach mode
        self.declare_parameter('can_mode_timeout', 5.0)
        # Speed used when resetting the follower to preset/zero (slower for safety)
        self.declare_parameter('preset_speed', 30)

        self.can_port = self.get_parameter('can_port').get_parameter_value().string_value
        self.gripper_exist = self.get_parameter('gripper_exist').get_parameter_value().bool_value
        self.move_speed = self.get_parameter('move_speed').get_parameter_value().integer_value
        self.preset_speed = self.get_parameter('preset_speed').get_parameter_value().integer_value
        self.preset_joints = list(
            self.get_parameter('preset_joints').get_parameter_value().double_array_value
        )

        self.can_mode_timeout = (
            self.get_parameter('can_mode_timeout').get_parameter_value().double_value
        )
        while len(self.preset_joints) < 7:
            self.preset_joints.append(0.0)

        self.get_logger().info(f"can_port: {self.can_port}")
        self.get_logger().info(f"gripper_exist: {self.gripper_exist}")
        self.get_logger().info(f"move_speed: {self.move_speed}")
        self.get_logger().info(f"preset_speed: {self.preset_speed}")
        self.get_logger().info(f"preset_joints: {self.preset_joints}")

        # Connect to the teacher arm only
        self.piper = C_PiperInterface_V2(can_name=self.can_port)
        self.piper.ConnectPort()
        time.sleep(0.1)

        # Publisher: mirrors the topic published by piper_broadcast_master_v2.
        # The remote follower node subscribes to this.
        self.joint_ctrl_pub = self.create_publisher(JointState, 'joint_states', 1)
        self.data_collect_pub = self.create_publisher(Bool, 'data_collect', 1)

        # Forwarding active flag — controlled by the operator loop thread
        self._forwarding = False
        self._forwarding_lock = threading.Lock()

        # Operator interaction loop runs in a background thread so rclpy.spin()
        # is not blocked by input() calls.
        self._loop_thread = threading.Thread(target=self._operator_loop, daemon=True)
        self._loop_thread.start()

        # High-frequency publish thread
        self._fwd_thread = threading.Thread(target=self._forward_thread, daemon=True)
        self._fwd_thread.start()

    # ── Joint position helper ─────────────────────────────────────────────────

    def _get_pos(self):
        """Return current joint angles (rad) as a tuple, with gripper (m) appended."""
        js = self.piper.GetArmJointMsgs().joint_state
        joints = tuple(getattr(js, f'joint_{i+1}') / 1e3 * 0.0174533 for i in range(6))
        if self.gripper_exist:
            return joints + (self.piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1e6,)
        return joints

    # ── Teach-mode exit sequence (mirrors playTrajectory_new_en.py) ───────────

    def _exit_teach_mode(self) -> None:
        """Run the mandatory e-stop + restore + enable sequence after teach mode.

        Mirrors the stop() and enable() functions in playTrajectory_new_en.py.
        Must be called whenever the arm transitions out of teach mode before
        CAN joint control is used.
        """
        self.get_logger().info("Running teach-mode exit sequence (e-stop + restore)…")

        # E-stop
        self.piper.EmergencyStop(0x01)
        time.sleep(1.0)

        # Wait until joints 2, 3, 5 are within the safe range for restore
        pos = self._get_pos()
        while not (
            abs(pos[1]) < _SAFE_J2_ABS
            and abs(pos[2]) < _SAFE_J3_ABS
            and pos[4] < _SAFE_J5_MAX
            and pos[4] > _SAFE_J5_MIN
        ):
            time.sleep(0.01)
            pos = self._get_pos()

        # Restore arm from e-stop
        self.piper.EmergencyStop(0x02)
        time.sleep(1.0)

        # Poll until ctrl_mode == 1 (CAN mode), sending ModeCtrl to nudge it
        self.get_logger().info("Waiting for CAN mode (ctrl_mode == 1)…")
        over_time = time.time() + self.can_mode_timeout
        while self.piper.GetArmStatus().arm_status.ctrl_mode != 1:
            if time.time() > over_time:
                self.get_logger().error(
                    "CAN mode switch timed out. Confirm teach mode is fully exited."
                )
                raise RuntimeError("CAN mode switch failed")
            self.piper.ModeCtrl(0x01, 0x01, self.preset_speed, 0x00)
            time.sleep(0.01)

        # Enable arm
        self.get_logger().info("Enabling teacher arm…")
        while not self.piper.EnablePiper():
            time.sleep(0.01)
        if self.gripper_exist:
            time.sleep(0.01)
            self.piper.GripperCtrl(0, 1000, 0x01, 0x00)
        self.piper.ModeCtrl(0x01, 0x01, self.preset_speed, 0x00)
        self.get_logger().info("Teacher arm enabled in CAN mode.")

    def _wait_teach_mode_on(self) -> None:
        """Block until teacher arm reports ctrl_mode == 2 (teach mode active)."""
        self.get_logger().info("Waiting for teacher arm to enter teach mode…")
        while True:
            try:
                if self.piper.GetArmStatus().arm_status.ctrl_mode == 2:
                    break
            except Exception:
                pass
            time.sleep(0.1)
        self.get_logger().info("Teacher arm is in teach mode.")

    # ── Preset-position reset ─────────────────────────────────────────────────

    def _move_and_stream(self, target_joints_rad: list, label: str,
                         settle_timeout: float = 15.0,
                         speed: int | None = None) -> None:
        """Command the teacher arm to a target position (radians) and stream
        its actual joint positions to the follower on joint_ctrl at 200 Hz
        until it arrives or the timeout expires.

        ``speed`` controls velocity[6] sent to the follower (1-100).  Defaults
        to ``self.move_speed`` when None.
        """
        RAD_TO_RAW = 57324.840764   # millidegrees per radian
        arrival_threshold = 0.05    # rad per-joint tolerance
        dt = 1.0 / 200.0

        if speed is None:
            speed = self.move_speed

        target_raw = [round(j * RAD_TO_RAW) for j in target_joints_rad]

        self.get_logger().info(f"Moving teacher arm to {label} (speed={speed})…")
        self.piper.MotionCtrl_2(0x01, 0x01, speed)

        for i in range(6):
            self.piper.JointCtrl(*target_raw)
            time.sleep(0.01)

        deadline = time.time() + settle_timeout
        while time.time() < deadline:
            self._publish_teacher_joints(speed=speed)
            pos = self._get_pos()
            if all(abs(pos[i] - target_joints_rad[i]) < arrival_threshold for i in range(6)):
                break
            time.sleep(dt)

        # Extra streaming to let the follower fully settle
        settle_extra = time.time() + 0.5
        while time.time() < settle_extra:
            self._publish_teacher_joints(speed=speed)
            time.sleep(dt)

        self.get_logger().info(f"Teacher arm reached {label}.")

    def _reset_to_preset(self) -> None:
        """Move the teacher arm safely to the preset position.

        Stage 1: move to all-zeros (safe neutral) and wait to arrive, streaming
                 the teacher's live position to the follower throughout.
        Stage 2: move to the configured preset and wait to arrive, again
                 streaming live position so the follower tracks continuously.
        """
        zero_joints = [0.0] * 6
        preset_joints_rad = self.preset_joints[:6]

        preset_joints_np = np.array(preset_joints_rad)
        preset_joints_np = preset_joints_np + np.random.normal(0, 0.1, 6)
        preset_joints_rad = preset_joints_np.tolist()

        # Stage 1: safe neutral position first
        if self.gripper_exist:
            self.piper.GripperCtrl(0, 1000, 0x01, 0x00)
        self._move_and_stream(zero_joints, "zero position", speed=self.preset_speed)

        # Stage 2: move to preset
        self._move_and_stream(preset_joints_rad, "preset position", speed=self.preset_speed)

        # Apply preset gripper after arm reaches preset
        if self.gripper_exist:
            gripper_raw = round(self.preset_joints[6] * 1e6)
            gripper_raw = max(0, min(gripper_raw, 80000))
            self.piper.GripperCtrl(gripper_raw, 1000, 0x01, 0x00)

        self.get_logger().info("Reset to preset complete.")

    # ── Placeholder data-collector hooks ─────────────────────────────────────

    def _data_collector_begin(self) -> None:
        self.get_logger().info("DATA COLLECTOR BEGIN")
        msg = Bool()
        msg.data = True
        self.data_collect_pub.publish(msg)

    def _data_collector_end(self) -> None:
        self.get_logger().info("DATA COLLECTOR END")
        msg = Bool()
        msg.data = False
        self.data_collect_pub.publish(msg)

    # ── Forwarding thread ─────────────────────────────────────────────────────

    def _forward_thread(self) -> None:
        """Publish teacher joint positions at 200 Hz while forwarding is active."""
        rate = self.create_rate(200)
        while rclpy.ok():
            with self._forwarding_lock:
                active = self._forwarding
            if active:
                self._publish_teacher_joints()
            rate.sleep()

    def _publish_teacher_joints(self, speed: int | None = None) -> None:
        """Read teacher arm joints and publish as JointState on joint_ctrl.

        ``speed`` is placed in velocity[6] and tells the follower's
        piper_ctrl_single_node what speed to pass to MotionCtrl_2.
        Defaults to ``self.move_speed`` when None.
        """
        if speed is None:
            speed = self.move_speed
        try:
            js = self.piper.GetArmJointMsgs().joint_state
            # millidegrees → radians (same conversion as piper_broadcast_master_v2)
            positions = [
                getattr(js, f'joint_{i+1}') / 1000 * RAW_TO_RAD for i in range(6)
            ]
            if self.gripper_exist:
                gripper_m = self.piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1e6
                positions.append(gripper_m)
            else:
                positions.append(0.0)

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
            msg.position = positions
            msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(speed)]
            msg.effort = [0.0] * 7
            self.joint_ctrl_pub.publish(msg)

        except Exception as exc:
            self.get_logger().warn(f"Publish error: {exc}")

    # ── Operator loop ─────────────────────────────────────────────────────────

    def _operator_loop(self) -> None:
        """Main operator interaction loop — runs in a background thread."""
        while rclpy.ok():
            # ── Step 1: operator disables teach mode; run e-stop exit sequence ─
            _wait_enter(
                "\n" + "=" * 60 + "\n"
                "Step 1: Move the teacher arm to zero, and disable teacher mode.\n"
                "        Press Enter when teacher mode is disabled."
            )
            try:
                self._exit_teach_mode()
            except RuntimeError as exc:
                print(f"\nERROR: {exc}  — retrying from step 1.")
                continue

            # ── Step 2: move teacher arm to preset; follower tracks in real time ─
            self._reset_to_preset()

            # ── Step 3: ask operator to enable teach mode ─────────────────────
            _wait_enter(
                "\n" + "=" * 60 + "\n"
                "Step 2: Enable teacher mode on the teacher arm.\n"
                "        Press Enter when ready to start recording."
            )
            self._wait_teach_mode_on()

            # ── Step 4: begin data collection + forwarding ────────────────────
            with self._forwarding_lock:
                self._forwarding = True

            time.sleep(2)  # let the forwarding thread publish a few messages before data collection starts
            print("\nStarting data collection...")

            self._data_collector_begin()

            # ── Step 5: wait for stop signal ──────────────────────────────────
            _wait_enter("\nRecording started. Press Enter to stop.")

            # ── Step 6: end data collection + stop forwarding ─────────────────
            with self._forwarding_lock:
                self._forwarding = False
            self._data_collector_end()
            print("Recording ended.")

            # Loop back to step 1


def main(args=None):
    rclpy.init(args=args)
    node = TeleopLoopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
