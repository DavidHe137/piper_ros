"""Axis-aligned workspace safety box in metres (base frame)."""

from __future__ import annotations

from typing import List, Tuple

MICRON_TO_M = 1e-6


def trunc_end_pose_m(end_pose) -> Tuple[float, float, float]:
    """Convert SDK end_pose to metres with the same rounding as scripts/safety_box.py."""
    return (
        round(end_pose.X_axis * MICRON_TO_M, 3),
        round(end_pose.Y_axis * MICRON_TO_M, 3),
        round(end_pose.Z_axis * MICRON_TO_M, 3),
    )


def box_from_corners(
    corner_a: List[float], corner_b: List[float]
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    ax, ay, az = corner_a
    bx, by, bz = corner_b
    return (
        (min(ax, bx), min(ay, by), min(az, bz)),
        (max(ax, bx), max(ay, by), max(az, bz)),
    )


def is_outside_box(
    xyz: Tuple[float, float, float],
    box_min: Tuple[float, float, float],
    box_max: Tuple[float, float, float],
) -> bool:
    x, y, z = xyz
    return (
        x < box_min[0] or x > box_max[0]
        or y < box_min[1] or y > box_max[1]
        or z < box_min[2] or z > box_max[2]
    )


# Same scale as piper_ctrl_single_node joint_callback (rad ↔ SDK joint ctrl units).
_ARM_JOINT_RAD_PER_SDK = 0.017444
_ARM_JOINT_CMD_FACTOR = 57324.840764


def joint_feedback_cmd_ints(piper) -> dict:
    """Current arm joint commands as integers for JointCtrl, from hardware feedback."""
    s = piper.GetArmJointMsgs().joint_state
    f = _ARM_JOINT_CMD_FACTOR

    def enc(raw: float) -> int:
        rad = (raw / 1000.0) * _ARM_JOINT_RAD_PER_SDK
        return int(round(rad * f))

    return {
        'joint1': enc(s.joint_1),
        'joint2': enc(s.joint_2),
        'joint3': enc(s.joint_3),
        'joint4': enc(s.joint_4),
        'joint5': enc(s.joint_5),
        'joint6': enc(s.joint_6),
    }


def clamp_joint_cmd_if_ee_outside_box(
    piper,
    joint_positions: dict,
    safety_enable: bool,
    box_min: Tuple[float, float, float],
    box_max: Tuple[float, float, float],
) -> Tuple[dict, bool]:
    """If EE is outside the box, replace joint1..6 commands with feedback (hold)."""
    if not safety_enable:
        return joint_positions, False
    xyz = trunc_end_pose_m(piper.GetArmEndPoseMsgs().end_pose)
    if not is_outside_box(xyz, box_min, box_max):
        return joint_positions, False
    fb = joint_feedback_cmd_ints(piper)
    out = dict(joint_positions)
    for k in ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'):
        out[k] = fb[k]
    return out, True
