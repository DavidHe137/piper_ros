#!/usr/bin/env python3
# -*-coding:utf8-*-
import select
import threading
import time
import math

import rclpy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

from piper.piper_broadcast_master import PiperRosNode as BroadcastMasterNode


class _GatedPublisher:
    def __init__(self, publisher, enabled_fn):
        self._publisher = publisher
        self._enabled_fn = enabled_fn

    def publish(self, msg):
        if self._enabled_fn():
            self._publisher.publish(msg)


class PiperBroadcastTeleopNode(BroadcastMasterNode):
    def __init__(self) -> None:
        super().__init__()

        self._broadcast_enabled = True
        self._continue_event = threading.Event()
        self._joint_states_pub_raw = self.joint_states_pub
        self.joint_states_pub = _GatedPublisher(self._joint_states_pub_raw, self._is_broadcast_enabled)
        self.create_subscription(Bool, 'continue_broadcast', self._continue_callback, 1)

        self.declare_parameter('reset_joint1', 0.0)
        self.declare_parameter('reset_joint2', 0.0)
        self.declare_parameter('reset_joint3', 0.0)
        self.declare_parameter('reset_joint4', 0.0)
        self.declare_parameter('reset_joint5', 0.0)
        self.declare_parameter('reset_joint6', 0.0)
        self.declare_parameter('reset_gripper', 0.0)
        self.declare_parameter('reset_motion_speed', 30)
        self.declare_parameter('reset_gripper_effort', 1.0)
        self.declare_parameter('reset_hold_seconds', 4.0)
        self.declare_parameter('reset_command_hz', 10.0)
        self.declare_parameter('wait_for_enable_timeout', 10.0)
        self.declare_parameter('startup_delay_seconds', 0.5)
        self.declare_parameter('prompt_to_continue', True)

        self._reset_positions = [
            self._read_float_param('reset_joint1', 0.0),
            self._read_float_param('reset_joint2', 0.0),
            self._read_float_param('reset_joint3', 0.0),
            self._read_float_param('reset_joint4', 0.0),
            self._read_float_param('reset_joint5', 0.0),
            self._read_float_param('reset_joint6', 0.0),
            self._read_float_param('reset_gripper', 0.0),
        ]
        self._reset_motion_speed = int(max(1, min(100, round(self._read_float_param('reset_motion_speed', 30.0)))))
        self._reset_gripper_effort = self._read_float_param('reset_gripper_effort', 1.0)
        self._reset_hold_seconds = max(0.1, self._read_float_param('reset_hold_seconds', 4.0))
        self._reset_command_hz = max(1.0, self._read_float_param('reset_command_hz', 10.0))
        self._wait_for_enable_timeout = self._read_float_param('wait_for_enable_timeout', 10.0)
        self._startup_delay_seconds = max(0.0, self._read_float_param('startup_delay_seconds', 0.5))
        self._prompt_to_continue = self._read_bool_param('prompt_to_continue', True)

        # self._manual_reset()
        # self._reset_thread = threading.Thread(target=self._manual_reset, daemon=True)
        # self._reset_thread.start()
        # self.reset_timer = self.create_timer(0.05, self._manual_reset)
        # self._startup_thread = threading.Thread(target=self._run_startup_sequence, daemon=True)
        # self._startup_thread.start()
        self.print_initial_status = False
        self.disabled_arm = False
        self._startup_mode_switch_done = False
        self._startup_mode_thread = threading.Thread(target=self.set_ctrl_mode2can, daemon=True)
        self._startup_mode_thread.start()


    def set_ctrl_mode2can(self):
        """Wait for enable, force teach-off, then drive CAN+joint commands for a short startup window."""
        self.get_logger().info("Attempting to switch to CAN control mode on startup.")
        if not self._wait_for_enable():
            self.get_logger().warning("Startup mode switch skipped: arm was not enabled in time.")
            return

        rad_to_mdeg = 180000.0 / math.pi
        joints = [round(v * rad_to_mdeg) for v in self._reset_positions[:6]]
        # Force a tiny delta in joint2 to avoid "no-op" command filtering while switching mode.
        joints[1] += 3000

        for _ in range(20):
            if not rclpy.ok():
                return
            self.piper.MotionCtrl_1(emergency_stop=0x00, track_ctrl=0x00, grag_teach_ctrl=0x02)
            self.piper.MotionCtrl_2(0x01, 0x01, self._reset_motion_speed)
            self.piper.JointCtrl(*joints)
            time.sleep(0.05)

        status = self.piper.GetArmStatus().arm_status
        self.get_logger().info(f"Startup mode check: ctrl={status.ctrl_mode}, teach={status.teach_status}")
        self._startup_mode_switch_done = True


    
    def _manual_reset(self) -> bool:
        if self._startup_mode_switch_done:
            return True
        status = self.piper.GetArmStatus()
        if not self.print_initial_status:
            self.get_logger().info(f"mode is : {status.arm_status.ctrl_mode} \n" +   
            f"arm_status is : {status.arm_status.arm_status}\n" +
            f"teach_status is : {status.arm_status.teach_status}\n" +
            f"motion_status is : {status.arm_status.motion_status}\n" + 
            f"trajectory_num is : {status.arm_status.trajectory_num}\n" +
            f"err_code is : {status.arm_status.err_code}\n")
            self.print_initial_status = True

        if self.GetEnableFlag():
            self.get_logger().info("Master arm is enabled, skipping manual reset.")
            status = self.piper.GetArmStatus()
            self.get_logger().info(f"mode is : {status.arm_status.ctrl_mode} \n" +   
            f"arm_status is : {status.arm_status.arm_status}\n" +
            f"teach_status is : {status.arm_status.teach_status}\n" +
            f"motion_status is : {status.arm_status.motion_status}\n" + 
            f"trajectory_num is : {status.arm_status.trajectory_num}\n" +
            f"err_code is : {status.arm_status.err_code}\n")

       
            if status.arm_status.ctrl_mode == 0x02:
                for _ in range(10):
                    self.piper.MotionCtrl_1(emergency_stop=0x00, track_ctrl=0x00, grag_teach_ctrl=0x02)
                    time.sleep(0.05)

                for _ in range(5):
                    self.piper.MotionCtrl_2(ctrl_mode=0x00, move_mode=0x01, move_spd_rate_ctrl=30)
                    time.sleep(0.05)
                self.piper.MotionCtrl_2(ctrl_mode=0x00, move_mode=0x01, move_spd_rate_ctrl=self._reset_motion_speed)
                time.sleep(0.1)
                new_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
                self.get_logger().info(f"Switched to mode {new_mode}, expected 0x00.")
            else:
                return True
            # else:
            #     # 2) Switch to CAN control mode
            #     # already switched to CAN
            #     self.get_logger().info("switching to CAN control mode")
            #     self.piper.MotionCtrl_2(ctrl_mode=0x01, move_mode=0x01, move_spd_rate_ctrl=self._reset_motion_speed)
            #     time.sleep(0.1)
            #     new_mode = self.piper.GetArmStatus().arm_status.ctrl_mode
            #     self.get_logger().info(f"Switched to mode {new_mode}, expected 0x01.")
            

                



            
    def _is_broadcast_enabled(self) -> bool:
        return self._broadcast_enabled

    def _read_float_param(self, name: str, default: float) -> float:
        value = self.get_parameter(name).value
        try:
            return float(value)
        except (TypeError, ValueError):
            self.get_logger().warning(f"Invalid value for {name}: {value}. Using {default}.")
            return default

    def _read_bool_param(self, name: str, default: bool) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ('1', 'true', 'yes', 'y', 'on')
        if isinstance(value, (int, float)):
            return value != 0
        self.get_logger().warning(f"Invalid value for {name}: {value}. Using {default}.")
        return default

    def _wait_for_enable(self) -> bool:
        start_time = time.time()
        gate_sync_attempted = False
        while rclpy.ok():
            if self.GetEnableFlag():
                return True
            drivers_enabled = self._drivers_enabled()
            if drivers_enabled and not gate_sync_attempted:
                gate_sync_attempted = True
                # Keep master reset aligned with normal callback logic, which requires GetEnableFlag().
                self.enable_callback(Bool(data=True))
                if self.GetEnableFlag():
                    return True
            self.piper.EnableArm(7)
            if self.gripper_exist:
                self.piper.GripperCtrl(0, 1000, 0x01, 0)
            if self._wait_for_enable_timeout > 0 and (time.time() - start_time) > self._wait_for_enable_timeout:
                return False
            time.sleep(0.2)
        return False

    def _drivers_enabled(self) -> bool:
        try:
            low_spd = self.piper.GetArmLowSpdInfoMsgs()
            return (
                low_spd.motor_1.foc_status.driver_enable_status and
                low_spd.motor_2.foc_status.driver_enable_status and
                low_spd.motor_3.foc_status.driver_enable_status and
                low_spd.motor_4.foc_status.driver_enable_status and
                low_spd.motor_5.foc_status.driver_enable_status and
                low_spd.motor_6.foc_status.driver_enable_status
            )
        except Exception:
            return False

    def _continue_callback(self, msg: Bool) -> None:
        if msg.data:
            self._continue_event.set()
            self.get_logger().info("Continue signal received on /continue_broadcast.")

    def _wait_for_continue_signal(self) -> None:
        self._continue_event.clear()
        self.get_logger().info(
            "Broadcast paused. Press Enter in this terminal, or run "
            "`ros2 topic pub --once /continue_broadcast std_msgs/msg/Bool \"{data: true}\"`."
        )
        try:
            with open('/dev/tty', 'r', encoding='utf-8') as tty:
                while rclpy.ok() and not self._continue_event.is_set():
                    ready, _, _ = select.select([tty], [], [], 0.2)
                    if ready:
                        tty.readline()
                        self._continue_event.set()
                        self.get_logger().info("Continue signal received from terminal Enter.")
                        break
        except OSError as exc:
            self.get_logger().warning(f"/dev/tty unavailable ({exc}); waiting on /continue_broadcast only.")
        while rclpy.ok() and not self._continue_event.wait(timeout=0.2):
            pass

    def _run_startup_sequence(self) -> None:
        time.sleep(self._startup_delay_seconds)
        self.get_logger().info("Teleop startup: waiting for master arm enable state.")

        if not self._wait_for_enable():
            self.get_logger().warning(
                "Master arm was not enabled before timeout. Startup reset skipped."
            )
            return

        self._send_reset_pose()

        if not self._prompt_to_continue:
            return

        self._broadcast_enabled = False
        self._wait_for_continue_signal()
        self._broadcast_enabled = True
        self.get_logger().info("Master broadcast resumed.")

    def _send_reset_pose(self) -> None:
        command_period = 1.0 / self._reset_command_hz
        repeat_count = max(1, round(self._reset_hold_seconds * self._reset_command_hz))
        reset_msg = JointState()
        reset_msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
        reset_msg.position = [float(v) for v in self._reset_positions]
        reset_msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(self._reset_motion_speed)]
        reset_msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(self._reset_gripper_effort)]

        self.get_logger().info(
            f"Sending startup reset via joint_callback (rad): {self._reset_positions}, speed: {self._reset_motion_speed}"
        )
        for _ in range(repeat_count):
            if not rclpy.ok():
                return
            if not self.GetEnableFlag():
                if not self._wait_for_enable():
                    self.get_logger().warning("Reset aborted: master arm is not enabled.")
                    return
            # breakpoint()
            # self.joint_callback(reset_msg)
            time.sleep(command_period)
        self.get_logger().info("Startup reset sequence complete.")


def main(args=None):
    rclpy.init(args=args)
    piper_teleop_node = PiperBroadcastTeleopNode()
    try:
        rclpy.spin(piper_teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        piper_teleop_node.destroy_node()
        rclpy.shutdown()
