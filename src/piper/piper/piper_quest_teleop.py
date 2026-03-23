#!/usr/bin/env python3
# -*-coding:utf8-*-
"""Quest controller teleoperation node for the Piper arm.

Replaces the physical master arm.  Publishes sensor_msgs/JointState in the
same format expected by piper_ctrl_single_node on the follower arm.

Button mapping (right controller):
  RJ          – calibration: point controller along robot +X, press to lock
  RTr (index) – hold to teleoperate; release to pause (arm holds position)
  rightGrip   – proportional gripper (0=open, 1=closed)
  A           – return arm to home position
"""

import time
import threading

import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

import tf2_ros


from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics
from oculus_reader import OculusReader


# ── Constants ────────────────────────────────────────────────────────────────

JOINT_LIMITS_MIN = np.array([-2.618,  0.0,   -2.967, -1.745, -1.22,  -2.094])
JOINT_LIMITS_MAX = np.array([ 2.168,  3.14,   0.0,    1.745,  1.22,   2.094])

GRIPPER_MAX_MM = 70.0

# Quest OpenXR convention: X=right, Y=up, Z=back (toward user)
# Piper robot convention:  X=forward, Y=left, Z=up
# det=+1 (proper rotation)
R_QUEST_TO_ROBOT = np.array([
    [ 0,  0, -1],   # robot X  = -quest Z
    [-1,  0,  0],   # robot Y  = -quest X
    [ 0,  1,  0],   # robot Z  =  quest Y
], dtype=float)


# ── IK module ────────────────────────────────────────────────────────────────

class PiperIK:
    """Damped-least-squares Jacobian IK using the Piper FK as the kinematic model.

    Position error is in metres, orientation error in radians.
    Orientation is deliberately down-weighted (ori_weight << 1) because the
    Piper Jacobian has orientation columns ~5× larger than position columns,
    which would otherwise make the IK ignore translation entirely.

    ori_weight=0   → position only (stable but no wrist tracking)
    ori_weight=0.05 → position-prioritised with soft wrist tracking (default)
    """

    def __init__(self, dh_is_offset: int = 0x01, ori_weight: float = 0.05,
                 n_iter: int = 200, alpha: float = 0.5, lam: float = 0.05):
        self._fk = C_PiperForwardKinematics(dh_is_offset=dh_is_offset)
        self.ori_weight = ori_weight
        self.n_iter = n_iter
        self.alpha = alpha
        self.lam = lam

    def fk(self, joints: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Forward kinematics.  Returns (xyz_m [3], R [3×3])."""
        raw = self._fk.CalFK(joints)[-1]
        xyz_m = np.array(raw[:3]) / 1000.0
        R = Rotation.from_euler('xyz', raw[3:], degrees=True).as_matrix()
        return xyz_m, R

    def solve(self, target_xyz_m: np.ndarray, target_R: np.ndarray,
              q0: np.ndarray) -> np.ndarray:
        """Solve IK from seed q0.  Returns new joint angles (radians)."""
        q = q0.copy()
        w = self.ori_weight
        eps = 1e-5

        for _ in range(self.n_iter):
            xyz, R = self.fk(q)
            ep = target_xyz_m - xyz                                 # pos err (m)
            er = Rotation.from_matrix(target_R @ R.T).as_rotvec()  # ori err (rad)

            if np.linalg.norm(ep) < 5e-4:   # 0.5 mm
                break

            # 6×6 numerical Jacobian (pos rows in m/rad, ori rows in rad/rad)
            J = np.zeros((6, 6))
            for i in range(6):
                qd = q.copy(); qd[i] += eps
                xd, Rd = self.fk(qd)
                J[:3, i] = (xd - xyz) / eps
                J[3:, i] = Rotation.from_matrix(Rd @ R.T).as_rotvec() / eps

            # Weight orientation rows so pos and ori contribute equally
            # despite the ~5× Jacobian magnitude difference
            W = np.diag([1, 1, 1, w, w, w])
            err_w = np.concatenate([ep, er * w])
            Jw = W @ J

            # Damped pseudoinverse step
            dq = Jw.T @ np.linalg.solve(Jw @ Jw.T + self.lam**2 * np.eye(6), err_w)
            q = np.clip(q + self.alpha * dq, JOINT_LIMITS_MIN, JOINT_LIMITS_MAX)

        return q


# ── ROS node ─────────────────────────────────────────────────────────────────

class PiperQuestTeleopNode(Node):

    def __init__(self) -> None:
        super().__init__('piper_quest_teleop_node')
        self._declare_params()
        self._read_params()

        self.get_logger().info(f"reset joints (rad): {self.reset_joints_rad.tolist()}")
        self.get_logger().info(
            f"pos_scale={self.pos_scale}  ori_weight={self.ori_weight}  "
            f"speed_pct={self.speed_pct}  reset_speed_pct={self.reset_speed_pct}  "
            f"control_hz={self.control_hz}"
        )

        self._pub = self.create_publisher(JointState, 'joint_states', 1)
        self._pub_cartesian_target = self.create_publisher(PoseStamped, 'ee_target_pose', 1)
        self._ik = PiperIK(ori_weight=self.ori_weight)

        # Live state
        self._joints = self.reset_joints_rad.copy()
        self._gripper_mm = self.reset_gripper_mm
        self._current_speed = self.speed_pct

        # Calibration: inverse of the raw Quest rotation at the moment of locking.
        # While _reset_orientation=True this is updated every loop tick (live tracking).
        # It is locked (frozen) when the user presses RJ or holds the trigger.
        self._reset_orientation: bool = True
        self._vr_to_global: np.ndarray = np.eye(3)  # inv(raw_rot) in Quest frame

        # Teleoperation snapshots (updated on trigger rising edge)
        self._q0_pos = np.zeros(3)    # controller pos at trigger press (calibrated frame, m)
        self._q0_rot = np.eye(3)      # controller rot at trigger press (calibrated frame)
        self._arm0_xyz = np.zeros(3)  # arm EE xyz at trigger press (m)
        self._arm0_rot = np.eye(3)    # arm EE rot at trigger press

        # Button edge-detection
        self._prev: dict = {'A': False, 'RJ': False, 'RTr': False}

        # Startup: send home pose so follower arm moves to known state
        self._publish_joints_for(
            self.reset_joints_rad, self.reset_gripper_mm,
            speed=self.reset_speed_pct, duration_s=3.0, rate_hz=20.0
        )

        # Connect Quest
        self.get_logger().info("Connecting to Quest via ADB …")
        self._oculus = OculusReader()
        self.get_logger().info("Quest connected.  Press RJ to calibrate.")

        # set the initial target pose
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer, self)

        target_frame = 'link6'
        source_frame = 'base_link'
        trans = None
    

        if not trans:
            self.get_logger().error(f"Transform not available:")
            self.target_xyz_m = np.zeros(3, dtype=float)
            self.target_rot = np.eye(3, dtype=float)
        else:
            self.target_xyz_m = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
            self.target_rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]).as_matrix()

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ── Parameters ───────────────────────────────────────────────────────────

    def _declare_params(self):
        self.declare_parameter('gripper_exist',   True)
        for i in range(1, 7):
            self.declare_parameter(f'reset_joint_{i}', 0.0)
        self.declare_parameter('reset_gripper',   0.0)
        self.declare_parameter('reset_speed_pct', 80)
        self.declare_parameter('pos_scale',       1.0)
        self.declare_parameter('ori_weight',      0.05)
        self.declare_parameter('speed_pct',       30)
        self.declare_parameter('control_hz',      30.0)

    def _read_params(self):
        g = self.get_parameter
        self.gripper_exist     = g('gripper_exist').get_parameter_value().bool_value
        self.reset_joints_rad  = np.array([
            g(f'reset_joint_{i}').get_parameter_value().double_value for i in range(1, 7)
        ])
        self.reset_gripper_mm  = g('reset_gripper').get_parameter_value().double_value
        self.reset_speed_pct   = g('reset_speed_pct').get_parameter_value().integer_value
        self.pos_scale         = g('pos_scale').get_parameter_value().double_value
        self.ori_weight        = g('ori_weight').get_parameter_value().double_value
        self.speed_pct         = g('speed_pct').get_parameter_value().integer_value
        self.control_hz        = g('control_hz').get_parameter_value().double_value

    # ── Publishing helpers ────────────────────────────────────────────────────

    def _make_msg(self, joints: np.ndarray, gripper_mm: float, speed: int) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = ['joint1','joint2','joint3','joint4','joint5','joint6','gripper']
        msg.position = joints.tolist() + [gripper_mm / 1000.0]
        msg.velocity = [0.0] * 6 + [float(speed)]   # velocity[6] = speed % for follower
        msg.effort   = [0.0] * 7
        return msg

    def _make_cartesian_target_msg(self, xyz_m: np.ndarray, rpy_rad: np.ndarray) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.pose.position.x = xyz_m[0]
        msg.pose.position.y = xyz_m[1]
        msg.pose.position.z = xyz_m[2]
        quat = Rotation.from_matrix(rpy_rad).as_quat()
        self.get_logger().info(f"quat: {quat}")
        self.get_logger().info(f"rpy_rad: {rpy_rad}")
        self.get_logger().info(f"xyz_m: {xyz_m}")

        
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        return msg

    def _publish_joints_for(self, joints: np.ndarray, gripper_mm: float,
                            speed: int, duration_s: float, rate_hz: float):
        self.get_logger().info(
            f"Publishing reset pose at {speed}% speed for {duration_s:.0f}s …"
        )
        msg = self._make_msg(joints, gripper_mm, speed)
        t_end = time.time() + duration_s
        dt = 1.0 / rate_hz
        while time.time() < t_end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self._pub.publish(msg)
            time.sleep(dt)
        self.get_logger().info("Reset done.")

    # def _get_transform(self, target_frame: str, source_frame: str) -> TransformStamped:
    #     try:
    #         trans = self._tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
    #     except Exception as e:
    #         self.get_logger().error(f"Transform not available: {e}")
    #         trans = None
    #     if trans:
    #         self.target_xyz_m = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
    #         self.target_rot = Rotation.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]).as_matrix()
 

    # ── Main control loop ─────────────────────────────────────────────────────

    def _loop(self):
        dt = 1.0 / self.control_hz
        trigger_active = False

        while not self._stop.is_set():
            t0 = time.time()

            transforms, buttons = self._oculus.get_transformations_and_buttons()

            # Read buttons regardless of whether transforms arrived
            cur = {
                'A':   bool((buttons or {}).get('A',   False)),
                'RJ':  bool((buttons or {}).get('RJ',  False)),
                'RTr': bool((buttons or {}).get('RTr', False)),
            }

            if not transforms or 'r' not in transforms:
                self._prev = cur
                time.sleep(dt)
                continue

            # ── Extract raw Quest pose ────────────────────────────────────────
            T = transforms['r']           # 4×4, metres, OpenXR
            raw_rot = T[:3, :3]
            raw_pos = T[:3, 3]

            # ── Live calibration tracking ─────────────────────────────────────
            # While _reset_orientation is True we continuously update the inverse
            # of the raw controller rotation.  The calibration locks when the user
            # presses RJ or activates the trigger (matching quest_teleop_droid.py).
            if self._reset_orientation:
                try:
                    self._vr_to_global = np.linalg.inv(raw_rot)
                except np.linalg.LinAlgError:
                    self._vr_to_global = np.eye(3)

            # Apply full transform chain:
            #   R_QUEST_TO_ROBOT  – axis reorder (Quest → robot convention)
            #   _vr_to_global     – locked calibration inverse
            #   raw_rot / raw_pos – current Quest measurement
            q_pos = R_QUEST_TO_ROBOT @ (self._vr_to_global @ raw_pos)
            q_rot = R_QUEST_TO_ROBOT @ (self._vr_to_global @ raw_rot) @ R_QUEST_TO_ROBOT.T

            # ── A: return to home ─────────────────────────────────────────────
            if cur['A'] and not self._prev['A']:
                self.get_logger().info("A: returning to home")
                self._joints      = self.reset_joints_rad.copy()
                self._gripper_mm  = self.reset_gripper_mm
                self._current_speed = self.reset_speed_pct
                trigger_active    = False
            elif not cur['A'] and self._prev['A']:
                self._current_speed = self.speed_pct

            # ── RJ: calibration lock / unlock ─────────────────────────────────
            # Point the controller along the robot's +X axis, then press the
            # joystick to lock that orientation as "forward".  Press again to
            # re-arm (resume live tracking so you can re-calibrate).
            if cur['RJ'] and not self._prev['RJ']:
                if not self._reset_orientation:
                    # Already locked — toggle back to live tracking
                    self._reset_orientation = True
                    self.get_logger().info("Calibration re-armed (live tracking)")
                else:
                    # Lock the current inverse rotation as calibration
                    self._reset_orientation = False
                    trigger_active = False   # force re-snapshot on next trigger press
                    self.get_logger().info(
                        f"Calibrated.  vr_to_global:\n{self._vr_to_global.round(3)}"
                    )

            # ── RTr: teleoperation ────────────────────────────────────────────
            # Trigger active → lock calibration so it doesn't drift mid-motion.
            if cur['RTr']:
                self._reset_orientation = False
            else:
                # Trigger released → re-arm live tracking so the user can
                # re-calibrate by just moving the controller and pressing RJ.
                if trigger_active:
                    self._reset_orientation = True
                trigger_active = False

            rising_edge = cur['RTr'] and not self._prev['RTr']
            if rising_edge:
                # Snapshot controller and arm pose at the moment of press
                self._q0_pos,  self._q0_rot  = q_pos.copy(), q_rot.copy()
                self._arm0_xyz, self._arm0_rot = self._ik.fk(self._joints)
                trigger_active = True
                self.get_logger().info("Trigger: teleoperation engaged")

            if trigger_active:
                # Calibration is already baked into q_pos / q_rot, so deltas are
                # computed directly in the calibrated robot frame.
                delta_pos_m = (q_pos - self._q0_pos) * self.pos_scale
                target_xyz_m = self._arm0_xyz + delta_pos_m
                self.target_xyz_m = target_xyz_m

                delta_rot = q_rot @ self._q0_rot.T
                target_rot = delta_rot @ self._arm0_rot
                self.target_rot = target_rot
                self._joints = self._ik.solve(target_xyz_m, target_rot, self._joints)

            # ── Gripper ───────────────────────────────────────────────────────
            if self.gripper_exist and buttons:
                grip = buttons.get('rightGrip', 0.0)
                if isinstance(grip, (tuple, list)):
                    grip = grip[0]
                self._gripper_mm = float(grip) * GRIPPER_MAX_MM

            # ── Publish ───────────────────────────────────────────────────────
            self._pub.publish(
                self._make_msg(self._joints, self._gripper_mm, self._current_speed)
            )

            self._pub_cartesian_target.publish(
                self._make_cartesian_target_msg(self.target_xyz_m, self.target_rot)
            )

            self._prev = cur

            elapsed = time.time() - t0
            if elapsed < dt:
                time.sleep(dt - elapsed)

    def destroy_node(self):
        self._stop.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PiperQuestTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
