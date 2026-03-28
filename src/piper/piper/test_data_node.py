
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

from openpi_client.action_chunkers.rtc import InferenceTimeRTCBroker as RTCBroker
from openpi_client.client import BidirectionalWebsocket
from openpi_client.schemas import Action, LiberoObservation, Observation

import lerobot.datasets.lerobot_dataset as lerobot_dataset
import numpy as np

CONTROL_HZ = 30

def load_episode(
    repo_id: str,
    episode_index: int,
) -> list[Observation]:
    """Load all frames from a single LeRobot episode.

    Returns:
        observations: List of observation objects in the format expected by the policy server.
    """
    ds = lerobot_dataset.LeRobotDataset(
        repo_id,
        episodes=[episode_index]
    )

    return [LiberoObservation(
            step=i,
            state=sample["state"].numpy().astype(np.float32),
            image=sample["image"].numpy().astype(np.float32),
            wrist_image=sample["wrist_image"].numpy().astype(np.float32),
            prompt=sample["task"],
        ) for i, sample in enumerate(ds)]


class TestDataNode(Node):
    """ROS2 node for the client"""

    def __init__(self) -> None:
        super().__init__('test_data_node')
        # ROS parameters
        self.declare_parameter('robot_id', "robot_0")
        self.declare_parameter('host', "localhost")
        self.declare_parameter('port', 8080)
        self.declare_parameter('control_hz', CONTROL_HZ)
        self.declare_parameter('execution_horizon', 10)
        self.declare_parameter('lerobot_repo_id', "davidhe137/pick-up-the-legos-and-sort-them-into-the-correct-bins-20260324")
        self.declare_parameter('top_image_topic', "/camera/intel_realsense_d435i_top/color/image_raw")
        self.declare_parameter('wrist_image_topic', "/camera/intel_realsense_d435i_wrist/color/image_raw")
        self.declare_parameter('episode_idx', 0)

        self.observations = load_episode(
            repo_id=self.get_parameter('lerobot_repo_id').value,
            episode_index=self.get_parameter('episode_idx').value,
        )

        self.joint_pub = self.create_publisher(JointState, 'joint_states', 1) 
        self.create_timer(1.0 / self.get_parameter('control_hz').value, self.publish_callback)
        self.step = 0

        self.ws_client = BidirectionalWebsocket(
            robot_id=self.get_parameter('robot_id').value,
            host=self.get_parameter('host').value,
            port=self.get_parameter('port').value,
            api_key=None,
            control_hz=self.get_parameter('control_hz').value,
        )

        self.broker = RTCBroker(
            ws_client=self.ws_client,
            control_hz=self.get_parameter('control_hz').value,
            execution_horizon=self.get_parameter('execution_horizon').value,
        )   

        # Joint
        self.joint_states = JointState()
        self.joint_states.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        self.joint_states.position = [0.0] * 7
        self.joint_states.velocity = [0.0] * 7
        self.joint_states.effort = [0.0] * 7

        self.get_logger().info("Client node initialized and publisher thread started.")
    
    def publish_callback(self):
        if all(v is not None for v in self.observations[self.step].__dict__.values()):  
            action = self.broker.infer(self.observations[self.step])
            self.publish_action(action)
            self.step+=1
            return
        self.get_logger().info("Waiting for complete observation...")

    def publish_action(self, action: Action) -> None:
        new_position = [float(pos) + float(act) for pos, act in zip(self.joint_states.position[:6], action.action[:6], strict=True)] + [float(act) for act in action.action[6:]]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        msg.position = new_position
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(0xAD)]
        msg.effort = [0.0] * 7
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    piper_single_node = TestDataNode()
    try:
        rclpy.spin(piper_single_node)
    except KeyboardInterrupt:
        pass
    finally:
        piper_single_node.destroy_node()
        rclpy.shutdown()