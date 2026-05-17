import pathlib
import sys
from copy import deepcopy

import lerobot.datasets.lerobot_dataset as lerobot_dataset
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from armory_client.action_chunkers.naive_async import NaiveAsyncBroker
from armory_client.client import BidirectionalWebsocket
from armory_client.runtime.real_saver import RealSaver
from armory_client.schemas import Action, LiberoObservation

from piper.util.station_util import get_station_namespace

CONTROL_HZ = 20
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "gripper"]


def load_episode(
    repo_id: str,
    episode_index: int,
) -> list[LiberoObservation]:
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
    """ROS2 node that replays a LeRobot episode through the policy server.

    Same control path as ``client_node_armory`` (BidirectionalWebsocket +
    NaiveAsyncBroker + timer ``publish_callback``), but observations are read
    from the dataset instead of live camera / joint topics.
    """

    def __init__(self) -> None:
        super().__init__("test_data_node")

        self.declare_parameter("robot_id", "robot_0")
        self.declare_parameter("host", "localhost")
        self.declare_parameter("port", 8080)
        self.declare_parameter("control_hz", CONTROL_HZ)
        self.declare_parameter("execution_horizon", 20)
        self.declare_parameter("lerobot_repo_id", "solace222/legos_turntable_30hz")
        self.declare_parameter("episode_idx", 93)

        self.declare_parameter("save_data", False)
        self.declare_parameter("save_video", False)
        self.declare_parameter("data_dir", "/datasets/armory_episodes")
        self.declare_parameter(
            "prompt",
            "pick up the legos and sort them into the correct bins.",
        )

        host = self.get_parameter("host").value
        port = self.get_parameter("port").value
        robot_id = self.get_parameter("robot_id").value
        control_hz = self.get_parameter("control_hz").value
        execution_horizon = self.get_parameter("execution_horizon").value

        self.observations = load_episode(
            repo_id=self.get_parameter("lerobot_repo_id").value,
            episode_index=self.get_parameter("episode_idx").value,
        )

        ns = get_station_namespace()
        self.joint_pub = self.create_publisher(JointState, f"/joint_states_mujoco", 1)
        self.create_timer(1.0 / control_hz, self.publish_callback)
        self.step = 0

        self.ws_client = BidirectionalWebsocket(
            robot_id=robot_id,
            host=host,
            port=port,
            api_key=None,
            control_hz=control_hz,
        )
        self.broker = NaiveAsyncBroker(
            ws_client=self.ws_client,
            control_hz=control_hz,
            execution_horizon=execution_horizon,
            real=True,
        )

        self._saver = None
        if bool(self.get_parameter("save_data").value):
            self._saver = RealSaver(
                out_dir=pathlib.Path(self.get_parameter("data_dir").value),
                robot_id=str(robot_id),
                prompt=str(self.get_parameter("prompt").value),
                control_hz=int(control_hz),
                action_chunk_broker=self.broker,
                save_video=bool(self.get_parameter("save_video").value),
            )
            self._saver.on_episode_start()
            self.get_logger().info(
                f"RealSaver enabled; writing to {self.get_parameter('data_dir').value}"
            )

        self.observation = LiberoObservation(
            state=None,
            step=0,
            image=None,
            wrist_image=None,
            prompt=self.get_parameter("prompt").value,
        )
        self.prev_observation = LiberoObservation(
            state=None,
            step=0,
            image=None,
            wrist_image=None,
            prompt=self.get_parameter("prompt").value,
        )

        self.get_logger().info(
            f"TestDataNode: {len(self.observations)} frames, naive async broker, "
            f"publishing /{ns}/joint_states @ {control_hz} Hz"
        )

    def publish_callback(self) -> None:
        if self.step >= len(self.observations):
            self.get_logger().info("Episode replay done.")
            sys.exit(0)

        src = self.observations[self.step]
        self.observation.state = src.state
        self.observation.image = src.image
        self.observation.wrist_image = src.wrist_image
        self.observation.prompt = src.prompt

        if not all(v is not None for v in self.observation.__dict__.values()):
            self.get_logger().info("Waiting for complete observation...")
            return

        self.observation.step = self.step
        action = self.broker.infer(self.observation)
        self.step += 1
        if self.prev_observation.state is not None:
            state_diff = self.observation.state - self.prev_observation.state
            # self.get_logger().info(f"State diff: {state_diff}")
        self.prev_observation = deepcopy(self.observation)

        self.get_logger().info(f"Action: {action}")
        if (
            all(float(x) == 0.0 for x in action.action)
            or action.action is None
            or len(action.action) != 7
        ):
            self.get_logger().info("Skipping all-zero or invalid action.")
            return

        self.publish_action(action)
        if self._saver is not None:
            try:
                self._saver.on_step(deepcopy(self.observation), action)
            except Exception as e:
                self.get_logger().warning(f"RealSaver.on_step failed: {e}")

    def publish_action(self, action: Action) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(x) for x in action.action]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(0xAD)]
        msg.effort = [0.0] * len(JOINT_NAMES)
        self.joint_pub.publish(msg)

    def destroy_node(self):
        if self._saver is not None:
            try:
                self._saver.on_episode_end()
                self._saver.close()
                self.get_logger().info("RealSaver flushed.")
            except Exception as e:
                self.get_logger().warning(f"RealSaver flush failed: {e}")
            self._saver = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TestDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
