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
import re
import socket
import tempfile
import threading
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

from openpi_client.schemas import LiberoObservation
from openpi_client.websocket_client_policy import WebsocketClientPolicy

TARGET_SIZE = (224, 224)
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]


# ── image transforms (identical to db3_to_lerobot.py / client_node.py) ────────

def _resize(img: np.ndarray, size: tuple[int, int]) -> np.ndarray:
    from PIL import Image as PILImage
    return np.array(PILImage.fromarray(img).resize(size, PILImage.BILINEAR))


def _transform_top(img: np.ndarray) -> np.ndarray:
    h = img.shape[0]
    img = img[:, :h, :]          # left-side square crop
    return _resize(img, TARGET_SIZE)


def _transform_wrist(img: np.ndarray) -> np.ndarray:
    h, w = img.shape[:2]
    start = (w - h) // 2
    img = img[:, start:start + h, :]  # center square crop
    return _resize(img, TARGET_SIZE)

def get_station_number():
    hostname = socket.gethostname()
    match = re.search(r'robotics-education-lab(\d+)(?:\..*)?$', hostname)
    if match:
        return int(match.group(1))
    return None


def get_station_namespace():
    station_number = get_station_number()
    if station_number is not None:
        return f'station{station_number}'
    return 'station'


def default_rs_color_topic(camera_node_name: str) -> str:
    """Absolute color topic for realsense2_camera_node under PushRosNamespace(station).

    Matches ``ros2 topic list`` on lab stations, e.g.
    ``/station11/intel_realsense_d435i_top/color/image_raw`` (no extra ``camera/`` segment).
    """
    ns = get_station_namespace()
    return f"/{ns}/{camera_node_name}/color/image_raw"


# ── ROS node ───────────────────────────────────────────────────────────────────

class TestClientNode(Node):
    def __init__(self, prompt: str, top_topic: str, wrist_topic: str) -> None:
        super().__init__("test_client_node")
        self.observation = LiberoObservation(
            state=None, step=0, image=None, wrist_image=None, prompt=prompt
        )
        self._obs_lock = threading.Lock()

        ns = get_station_namespace()
        # Leading / so topics are absolute (not resolved under this node's name).
        self.sim_pub = self.create_publisher(JointState, f"/{ns}/joint_states_mujoco", 1)
        self.real_pub = self.create_publisher(JointState, f"/{ns}/joint_states", 1)

        # subscribers
        self.create_subscription(JointState, f"/{ns}/joint_states_single", self._cb_joints, 1)
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

    def show_obs_images(self, obs: LiberoObservation) -> None:
        """Save top and wrist images to a temp file and open with xdg-open."""
        fig, axes = plt.subplots(1, 2, figsize=(8, 4))
        axes[0].imshow(obs.image)
        axes[0].set_title("Top")
        axes[0].axis("off")
        axes[1].imshow(obs.wrist_image)
        axes[1].set_title("Wrist")
        axes[1].axis("off")
        fig.tight_layout()
        tmp = tempfile.NamedTemporaryFile(suffix=".png", delete=False)
        fig.savefig(tmp.name)
        plt.close(fig)
        print(f"  Observation image saved to: {tmp.name}")

    def wait_for_observation(self) -> LiberoObservation:
        """Block until all observation fields are populated."""
        print("Waiting for complete observation...", end="", flush=True)
        required = ("state", "image", "wrist_image")
        while True:
            with self._obs_lock:
                missing = [k for k in required if getattr(self.observation, k) is None]
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
            if missing:
                print(f"\rWaiting for complete observation... missing: {', '.join(missing)}   ", end="", flush=True)
            time.sleep(0.1)

    def current_state(self) -> np.ndarray:
        with self._obs_lock:
            return self.observation.state.copy()

    def _make_joint_msg(self, position: list[float]) -> JointState:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(x) for x in position]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(0xAD)]
        msg.effort = [0.0] * len(JOINT_NAMES)
        return msg

    def rollout(self, actions: np.ndarray, publisher, label: str, control_hz: float) -> None:
        """Publish each action step in the chunk at control_hz.

        Actions from the policy are absolute joint positions (the dataset stores
        next-frame absolute positions as actions, not deltas), so they are sent
        directly without adding a current-state offset.
        """
        dt = 1.0 / control_hz
        print(f"  Rolling out {len(actions)} steps on {label} at {control_hz} Hz...")
        for i, action in enumerate(actions):
            publisher.publish(self._make_joint_msg(list(action)))
            time.sleep(dt)
            print(f"    step {i+1}/{len(actions)}: {[f'{v:.3f}' for v in action]}")
        print(f"  {label} rollout complete.")


# ── main ───────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Sim-validation testing client")
    parser.add_argument("--host", default="https://rohan-bansal--openpi-serve-modalpolicyserver-stable--98f7b0-dev.modal.run")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--control-hz", type=float, default=30.0)
    parser.add_argument("--prompt", default="pick up the legos and sort them into the correct bins.")
    parser.add_argument(
        "--top-topic",
        default=default_rs_color_topic("intel_realsense_d435i_top"),
    )
    parser.add_argument(
        "--wrist-topic",
        default=default_rs_color_topic("intel_realsense_d435i_wrist"),
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
    policy = WebsocketClientPolicy(robot_id=f"robot_{get_station_number()}", host=args.host, port=args.port, control_hz=args.control_hz)
    print("Connected.")

    try:
        while True:
            obs = node.wait_for_observation()

            # node.show_obs_images(obs)
            # input("\nObservation images displayed. Press Enter to query the server...")

            print("Querying server for action chunk...")
            result = policy.infer(obs)
            actions = result["actions"]  # (action_horizon, action_dim)

            print(f"\nChunk received: shape={actions.shape}")
            print(f"  first action: {[f'{v:.3f}' for v in actions[0]]}")
            print(f"  last  action: {[f'{v:.3f}' for v in actions[-1]]}")

            # input("Enter to rollout")

            # ans = input('\nType "yes" to roll out in SIM, anything else aborts: ').strip().lower()
            # if ans != "yes":
            #     print("Sim rollout aborted.")
            #     continue
            # node.rollout(actions, node.sim_pub, "SIM", args.control_hz)

            # ans = input('\nType "yes" to roll out on REAL robot, anything else skips: ').strip().lower()
            # if ans == "yes":
            print("control_hz: ", args.control_hz)
            node.rollout(actions, node.real_pub, "REAL", args.control_hz)
            # else:
            #     print("Real rollout skipped.")

            # ans = input('\nType "yes" to query another chunk, anything else exits: ').strip().lower()
            # if ans != "yes":
            #     break

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
