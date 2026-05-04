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

from piper.util.station_util import get_station_number, default_rs_color_topic, get_station_namespace

CONTROL_HZ = 20
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']


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
        self.get_logger().info(
            f"Station number: {station}  "
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
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(x) for x in action]
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