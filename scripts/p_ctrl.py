#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import subprocess
import sys
import time

import numpy as np
from piper_sdk import *

INIT_POSITION = [
    -0.02886982,
    1.0887149280000001,
    -1.17476618,
    -0.040696852000000006,
    1.194704672,
    -0.038743124000000004,
    0.0,
]

RESET_POSITION = [
    0.0,
    0.0,
    0.0,
    0.0,
    0.46671422,
    0.0,
    0.0,
]


def _joints_rad_from_feedback(piper):
    js = piper.GetArmJointMsgs().joint_state
    return np.array(
        [getattr(js, f"joint_{i+1}") / 1e3 * 0.0174533 for i in range(6)],
        dtype=np.float64,
    )


def _gripper_m_from_feedback(piper):
    return float(piper.GetArmGripperMsgs().gripper_state.grippers_angle) / 1e6


def _run_shell_command(cmd, script):
    result = subprocess.run([cmd, script])


def do_enable():
    _run_shell_command("bash", "/piper_sdk/piper_sdk/can_activate.sh can0 1000000")
    _run_shell_command("python3", "/piper_sdk/piper_sdk/demo/V2/piper_ctrl_enable.py")
    print("Piper enable sequence complete.")


def do_disable():
    # _run_shell_command("python3", "/piper_sdk/piper_sdk/demo/V2/piper_ctrl_reset.py")
    _run_shell_command("python3", "/piper_ros/scripts/piper_ctrl_disable.py")
    print("Piper disable sequence complete.")


def do_goto(mode: str):
    piper = C_PiperInterface("can0")
    piper.ConnectPort()

    while not piper.EnablePiper():
        time.sleep(0.01)

    factor = 57295.7795  # 1000 * 180 / pi

    if mode == "init":
        target_joints = np.array(INIT_POSITION[:6]) + np.random.normal(0, 0.05, 6)
        target_grip_m = float(INIT_POSITION[6])
    elif mode == "reset":
        target_joints = np.array(RESET_POSITION[:6], dtype=np.float64)
        target_grip_m = float(RESET_POSITION[6])
    elif mode == "zero":
        target_joints = np.zeros(6, dtype=np.float64)
        target_grip_m = 0.0
    else:
        raise ValueError(f"Unknown goto mode: {mode}")

    start_joints = _joints_rad_from_feedback(piper)
    start_grip_m = _gripper_m_from_feedback(piper)

    duration_sec = 1.0
    ctrl_hz = 50.0
    n_steps = max(1, int(round(duration_sec * ctrl_hz)))
    dt = duration_sec / n_steps

    piper.ModeCtrl(0x01, 0x01, 0, 0xAD)

    for k in range(n_steps + 1):
        alpha = k / n_steps
        joints = (1.0 - alpha) * start_joints + alpha * target_joints
        grip_m = (1.0 - alpha) * start_grip_m + alpha * target_grip_m

        joint_0 = round(float(joints[0]) * factor)
        joint_1 = round(float(joints[1]) * factor)
        joint_2 = round(float(joints[2]) * factor)
        joint_3 = round(float(joints[3]) * factor)
        joint_4 = round(float(joints[4]) * factor)
        joint_5 = round(float(joints[5]) * factor)
        joint_6 = round(grip_m * 1e6)

        piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        piper.GripperCtrl(abs(joint_6), 1000, 0x01, 0)

        if k < n_steps:
            time.sleep(dt)

    print(f"Go to {mode} complete.")


def build_parser():
    parser = argparse.ArgumentParser(description="Piper control helper.")

    subparsers = parser.add_subparsers(dest="command", required=True)

    goto_parser = subparsers.add_parser("goto", help="Move robot to a named pose.")
    goto_parser.add_argument("target", choices=("init", "zero", "reset"))

    subparsers.add_parser("enable", help="Run enable sequence.")
    subparsers.add_parser("disable", help="Run disable sequence.")

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()

    try:
        if args.command == "goto":
            do_goto(args.target)
        elif args.command == "enable":
            do_enable()
        elif args.command == "disable":
            do_disable()
        else:
            parser.error("Unknown command.")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()