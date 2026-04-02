#!/usr/bin/env python3
"""Interactive sim-validation testing node.

Workflow per iteration:
  1. Collect a complete observation (images + joint state).
  2. Query the policy server for one action chunk.
  3. Print chunk info, then wait for Enter to roll out in MuJoCo sim.
  4. After sim rollout, optionally roll out on the real robot.

Usage:
  python test_client_node.py [--host HOST] [--port PORT]
                             [--control-hz HZ] [--prompt PROMPT]
"""

import argparse
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

from openpi_client.schemas import LiberoObservation
from openpi_client.websocket_client_policy import WebsocketClientPolicy

TARGET_SIZE = (224, 224)
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


# ── image transforms (identical to db3_to_lerobot.py / client_node.py) ────────

def _transform_top(img: np.ndarray) -> np.ndarray:
    h = img.shape[0]
    img = img[:, :h, :]          # left-side square crop
    return cv2.resize(img, TARGET_SIZE)


def _transform_wrist(img: np.ndarray) -> np.ndarray:
    h, w = img.shape[:2]
    start = (w - h) // 2
    img = img[:, start:start + h, :]  # center square crop
    return cv2.resize(img, TARGET_SIZE)


# ── ROS node ───────────────────────────────────────────────────────────────────

class TestClientNode(Node):
    def __init__(self, prompt: str, top_topic: str, wrist_topic: str) -> None:
        super().__init__("test_client_node")
        self.observation = LiberoObservation(
            state=None, step=0, image=None, wrist_image=None, prompt=prompt
        )
        self._obs_lock = threading.Lock()

        # publishers
        self.sim_pub = self.create_publisher(JointState, "/joint_states_mujoco", 1)
        self.real_pub = self.create_publisher(JointState, "/joint_states", 1)

        # subscribers
        self.create_subscription(JointState, "joint_states_single", self._cb_joints, 1)
        self.create_subscription(Image, top_topic, self._cb_top, 10)
        self.create_subscription(Image, wrist_topic, self._cb_wrist, 10)

        self.get_logger().info("TestClientNode ready.")

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _cb_joints(self, msg: JointState) -> None:
        with self._obs_lock:
            self.observation.state = np.array(msg.position.tolist(), dtype=np.float32)

    def _cb_top(self, msg: Image) -> None:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img = _transform_top(img)
        with self._obs_lock:
            self.observation.image = img

    def _cb_wrist(self, msg: Image) -> None:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        img = _transform_wrist(img)
        with self._obs_lock:
            self.observation.wrist_image = img

    # ── helpers ────────────────────────────────────────────────────────────────

    def wait_for_observation(self) -> LiberoObservation:
        """Block until all observation fields are populated."""
        print("Waiting for complete observation...", end="", flush=True)
        while True:
            with self._obs_lock:
                missing = [k for k, v in self.observation.__dict__.items() if v is None]
                if not missing:
                    obs = LiberoObservation(
                        state=self.observation.state.copy(),
                        step=self.observation.step,
                        image=self.observation.image.copy(),
                        wrist_image=self.observation.wrist_image.copy(),
                        prompt=self.observation.prompt,
                    )
                    print(" done.")
                    return obs
            print(".", end="", flush=True)
            time.sleep(0.1)

    def current_state(self) -> np.ndarray:
        with self._obs_lock:
            return self.observation.state.copy()

    def _make_joint_msg(self, position: list[float]) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(x) for x in position]
        msg.velocity = [0.0] * len(JOINT_NAMES)
        msg.effort = [0.0] * len(JOINT_NAMES)
        return msg

    def rollout(self, actions: np.ndarray, publisher, label: str, control_hz: float) -> None:
        """Publish each action step in the chunk at control_hz, tracking running position."""
        dt = 1.0 / control_hz
        current_pos = self.current_state()
        print(f"  Rolling out {len(actions)} steps on {label} at {control_hz} Hz...")
        for i, action in enumerate(actions):
            new_joints = list(current_pos[:6] + action[:6])   # deltas for joints 1-6
            new_joints += list(action[6:])                     # absolute for gripper+
            publisher.publish(self._make_joint_msg(new_joints))
            time.sleep(dt)
            current_pos = self.current_state()                 # refresh from /joint_states_single
            print(f"    step {i+1}/{len(actions)}: {[f'{v:.3f}' for v in new_joints]}")
        print(f"  {label} rollout complete.")


# ── main ───────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Sim-validation testing client")
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--control-hz", type=float, default=10.0)
    parser.add_argument("--prompt", default="pick up the legos and sort them into the correct bins.")
    parser.add_argument(
        "--top-topic",
        default="/camera/intel_realsense_d435i_top/color/image_raw",
    )
    parser.add_argument(
        "--wrist-topic",
        default="/camera/intel_realsense_d435i_wrist/color/image_raw",
    )
    args = parser.parse_args()

    rclpy.init()
    node = TestClientNode(
        prompt=args.prompt,
        top_topic=args.top_topic,
        wrist_topic=args.wrist_topic,
    )

    # spin ROS in a background thread so main thread stays free for input()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(f"Connecting to policy server at {args.host}:{args.port} ...")
    policy = WebsocketClientPolicy(robot_id="robot_0", host=args.host, port=args.port)
    print("Connected.")

    try:
        while True:
            obs = node.wait_for_observation()

            print("Querying server for action chunk...")
            result = policy.infer(obs)
            actions = result["actions"]  # (action_horizon, action_dim)

            print(f"\nChunk received: shape={actions.shape}")
            print(f"  first action: {[f'{v:.3f}' for v in actions[0]]}")
            print(f"  last  action: {[f'{v:.3f}' for v in actions[-1]]}")

            ans = input('\nType "yes" to roll out in SIM, anything else aborts: ').strip().lower()
            if ans != "yes":
                print("Sim rollout aborted.")
                continue
            node.rollout(actions, node.sim_pub, "SIM", args.control_hz)

            ans = input('\nType "yes" to roll out on REAL robot, anything else skips: ').strip().lower()
            if ans == "yes":
                node.rollout(actions, node.real_pub, "REAL", args.control_hz)
            else:
                print("Real rollout skipped.")

            ans = input('\nType "yes" to query another chunk, anything else exits: ').strip().lower()
            if ans != "yes":
                break

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
