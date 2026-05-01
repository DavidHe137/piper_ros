import argparse
import re
import socket
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from openpi_client.action_chunkers.rtc import InferenceTimeRTCBroker as RTCBroker
from openpi_client.client import BidirectionalWebsocket
from openpi_client.schemas import Action, LiberoObservation, Observation
from openpi_client.websocket_client_policy import WebsocketClientPolicy

import lerobot.datasets.lerobot_dataset as lerobot_dataset
import numpy as np
import sys

CONTROL_HZ = 20
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']

# Per-workstation joint offsets (indexed by station_number - 1).
# Subtracted from every policy action before publishing, matching test_client_node.py.
offsets = [
    [0.02321829,  0.00910173, -0.03996214, -0.03830964,  0.10155467, -0.03065713, 0.0],
    [0.01820654, -0.00352656, -0.00632112,  0.00353667,  0.04040816, -0.03062013, 0.0],
    [0.01551772,  0.00531139, -0.0099189,  -0.08857924,  0.03289094, -0.04199907, 0.0],
    [0.0214633,   0.00162367,  0.00224904, -0.00127418,  0.07886142, -0.02211866, 0.0],
    [0.0251082,  -0.00635497, -0.02101643, -0.0789244,   0.09792906, -0.09720207, 0.0],
    [0.01112712,  0.03264597, -0.02738706, -0.01611043,  0.10601071, -0.01252022, 0.0],
    [0.02140098, -0.00576347,  0.00880733, -0.00593615,  0.02133216,  0.00560446, 0.0],
    [0.00730359,  0.00740132, -0.04213898, -0.00291651,  0.10787516, -0.02172383, 0.0],
    [0.01102888, -0.00241522, -0.03314344, -0.01546467,  0.1276725,  -0.03068104, 0.0],
    [0.03264093,  0.02947261, -0.02512171, -0.02812324,  0.05315014, -0.02938591, 0.0],
    [0.0,         0.0,         0.0,         0.0,          0.0,         0.0,        0.0],
    [-0.0171232, -0.002284,   0.00153009, -0.09052934,  0.07490137,  0.08260109,  0.0],
    [0.01038045,  0.00931645, -0.08858023, -0.02868523,  0.2464043,  -0.00241596, 0.0],
    [0.01498489, -0.01462671, -0.02245972, -0.04720494,  0.06598705, -0.01857491, 0.0],
    [0.0,         0.0,         0.0,         0.0,          0.0,         0.0,        0.0],
    [0.0,         0.0,         0.0,         0.0,          0.0,         0.0,        0.0],
    [0.0,         0.0,         0.0,         0.0,          0.0,         0.0,        0.0],
]


def get_station_number() -> int | None:
    hostname = socket.gethostname()
    match = re.search(r'robotics-education-lab(\d+)(?:\..*)?$', hostname)
    if match:
        return int(match.group(1))
    return None


def _station_offset() -> np.ndarray:
    """Return the offset array for this workstation, or zeros if unknown."""
    station = get_station_number()
    if station is not None and 1 <= station <= len(offsets):
        return np.array(offsets[station - 1], dtype=np.float64)
    return np.zeros(7, dtype=np.float64)


def load_episode(
    repo_id: str,
    episode_index: int,
) -> list[Observation]:
    """Load all frames from a single LeRobot episode."""
    ds = lerobot_dataset.LeRobotDataset(repo_id, episodes=[episode_index])
    return [
        LiberoObservation(
            step=i,
            state=sample["state"].numpy().astype(np.float32),
            image=sample["image"].numpy().astype(np.float32),
            wrist_image=sample["wrist_image"].numpy().astype(np.float32),
            prompt=sample["task"],
        )
        for i, sample in enumerate(ds)
    ]


class TestDataNode(Node):
    """ROS2 node that replays a LeRobot dataset episode through a policy server.

    Two modes (selected via the 'use_rtc' ROS parameter):
      rtc  (default) – uses RTCBroker; one action published per timer tick.
      non-rtc        – uses WebsocketClientPolicy; queries the server for a
                       full chunk (execution_horizon actions) and rolls them
                       out at control_hz before querying again.
    """

    def __init__(self) -> None:
        super().__init__('test_data_node')

        self.declare_parameter('robot_id', "robot_0")
        self.declare_parameter('host', "https://rohan-bansal--openpi-serve-modalpolicyserver-stable--98f7b0-dev.modal.run")
        self.declare_parameter('port', 8080)
        self.declare_parameter('control_hz', CONTROL_HZ)
        self.declare_parameter('execution_horizon', 20)
        self.declare_parameter('lerobot_repo_id', "solace222/sort-the-legos-into-the-correct-bins-20260427")
        self.declare_parameter('episode_idx', 0)
        self.declare_parameter('use_rtc', False)

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        robot_id = self.get_parameter('robot_id').value
        control_hz = self.get_parameter('control_hz').value
        execution_horizon = self.get_parameter('execution_horizon').value
        self.use_rtc: bool = self.get_parameter('use_rtc').value

        self.observations = load_episode(
            repo_id=self.get_parameter('lerobot_repo_id').value,
            episode_index=self.get_parameter('episode_idx').value,
        )

        self.joint_pub = self.create_publisher(JointState, '/station11/joint_states', 1)
        self.step = 0

        station = get_station_number()
        offset = _station_offset()
        self.get_logger().info(
            f"Station number: {station}  "
            f"joint offset: {[f'{v:.4f}' for v in offset]}"
        )

        if self.use_rtc:
            self.ws_client = BidirectionalWebsocket(
                robot_id=robot_id,
                host=host,
                port=port,
                api_key=None,
                control_hz=control_hz,
            )
            self.broker = RTCBroker(
                ws_client=self.ws_client,
                control_hz=control_hz,
                execution_horizon=execution_horizon,
            )
            self.create_timer(1.0 / control_hz, self._rtc_tick)
            self.get_logger().info("TestDataNode started in RTC mode.")
        else:
            self.policy = WebsocketClientPolicy(
                robot_id=robot_id,
                host=host,
                port=port,
                control_hz=control_hz,
            )
            self._control_hz = control_hz
            self._execution_horizon = execution_horizon
            self._non_rtc_thread = threading.Thread(
                target=self._non_rtc_loop, daemon=True
            )
            self._non_rtc_thread.start()
            self.get_logger().info("TestDataNode started in non-RTC mode.")

    # ── RTC mode ───────────────────────────────────────────────────────────────

    def _rtc_tick(self) -> None:
        try:
            obs = self.observations[self.step]
            if all(v is not None for v in obs.__dict__.values()):
                action = self.broker.infer(obs)
                self._publish_action(action.action)
                self.step += 1
            else:
                self.get_logger().info("Waiting for complete observation...")
        except IndexError:
            self.get_logger().info("Episode replay done.")
            sys.exit(0)
        except Exception as e:
            self.get_logger().error(f"RTC tick error: {e}")

    # ── non-RTC mode ───────────────────────────────────────────────────────────

    def _non_rtc_loop(self) -> None:
        dt = 1.0 / self._control_hz
        while self.step < len(self.observations):
            obs = self.observations[self.step]
            self.get_logger().info(
                f"Querying server at step {self.step}/{len(self.observations)}..."
            )
            try:
                result = self.policy.infer(obs)
            except Exception as e:
                self.get_logger().error(f"Policy inference failed: {e}")
                break

            actions = result["actions"]  # (execution_horizon, action_dim)
            self.get_logger().info(
                f"Chunk received: {len(actions)} actions. Rolling out..."
            )
            input("rollout? ")

            for action in actions:
                if self.step >= len(self.observations):
                    break
                self._publish_action(action)
                self.step += 1
                time.sleep(dt)

        self.get_logger().info("Episode replay done.")
        sys.exit(0)

    # ── shared helper ──────────────────────────────────────────────────────────

    def _publish_action(self, action) -> None:
        corrected = np.asarray(action, dtype=np.float64) - _station_offset()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(x) for x in corrected]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(0xAD)]
        msg.effort = [0.0] * len(JOINT_NAMES)
        self.joint_pub.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description="Test data node")
    parser.add_argument('--no-rtc', action='store_true', help="Use non-RTC chunk rollout mode")
    parsed, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TestDataNode()

    # Allow overriding use_rtc from CLI without going through ROS param syntax
    if parsed.no_rtc and node.use_rtc:
        node.get_logger().warn(
            "--no-rtc flag ignored because 'use_rtc' ROS param is already set. "
            "Pass use_rtc:=false as a ROS parameter to use non-RTC mode."
        )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()