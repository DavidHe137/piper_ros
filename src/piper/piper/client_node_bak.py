import re
import socket
from copy import deepcopy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

from openpi_client.action_chunkers.rtc import InferenceTimeRTCBroker as RTCBroker
from openpi_client.client import BidirectionalWebsocket
from openpi_client.schemas import Action, LiberoObservation, Observation

import cv2
import numpy as np

TARGET_SIZE = (224, 224)

CONTROL_HZ = 20


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
    """Absolute color topic under PushRosNamespace(station), e.g. /station11/.../color/image_raw."""
    ns = get_station_namespace()
    return f"/{ns}/{camera_node_name}/color/image_raw"


class ClientNode(Node):
    """ROS2 node for the client"""

    def __init__(self) -> None:
        super().__init__('chunx_client')
        # ROS parameters
        self.declare_parameter('robot_id', f"robot_{get_station_number()}")
        self.declare_parameter('host', "localhost")
        self.declare_parameter('port', 8080)
        self.declare_parameter('control_hz', CONTROL_HZ)
        self.declare_parameter('execution_horizon', 20)
        self.declare_parameter(
            'top_image_topic', default_rs_color_topic('intel_realsense_d435i_top')
        )
        self.declare_parameter(
            'wrist_image_topic', default_rs_color_topic('intel_realsense_d435i_wrist')
        )
        self.declare_parameter('prompt', "pick up the legos and sort them into the correct bins.")

        ns = get_station_namespace()
        # Leading / so topics match data_collection_new.launch.py (PushRosNamespace + remap).
        self.joint_pub = self.create_publisher(JointState, f'/{ns}/joint_states', 1)
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

        self.observation = LiberoObservation(state=None, step=0, image=None, wrist_image=None, prompt=self.get_parameter('prompt').value)
        self.observation_step = 0
        self.action_step = 0 # TODO: use action step on server
        self.prev_observation = LiberoObservation(state=None, step=0, image=None, wrist_image=None, prompt=self.get_parameter('prompt').value)
        self.create_subscription(JointState, f'/{ns}/joint_states_single', self._update_joint_states, 1)
        self.create_subscription(Image, self.get_parameter('top_image_topic').value, self._update_top_image, 10)
        self.create_subscription(Image, self.get_parameter('wrist_image_topic').value, self._update_wrist_image, 10)

        self.get_logger().info("Client node initialized and publisher thread started.")

    def _update_joint_states(self, msg: JointState) -> None:
        self.observation.state = np.array(msg.position.tolist())

    def _update_top_image(self, image: Image) -> None:
        image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        h = image.shape[0]
        image = image[:, :h, :]  # left-side square crop
        image = cv2.resize(image, TARGET_SIZE)
        self.observation.image = image
        # self.get_logger().info(f"Received top image with shape: {image.shape}")

    def _update_wrist_image(self, image: Image) -> None:
        image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        h, w = image.shape[:2]
        start = (w - h) // 2
        image = image[:, start:start + h, :]  # center square crop
        image = cv2.resize(image, TARGET_SIZE)
        self.observation.wrist_image = image
        # self.get_logger().info(f"Received wrist image with shape: {image.shape}")
    
    def publish_callback(self):
        if not all(v is not None for v in self.observation.__dict__.values()):
            self.get_logger().info("Waiting for complete observation... missing: " + ", ".join([k for k, v in self.observation.__dict__.items() if v is None]))
            return

        # # Don't start executing until the broker has received at least one real chunk
        # # from the server, so we never act on the initial null (all-zero) action.
        # if not self.broker.current_action_chunk:
        #     self.get_logger().info("Waiting for first action chunk from server...")
        #     self.observation.step = self.step
        #     self.broker.infer(self.observation)
        #     self.step += 1
        #     return
        # TODO: maybe have two time
        self.observation.step = self.step
        action = self.broker.infer(self.observation)
        if self.prev_observation.state is not None:
            state_diff = self.observation.state - self.prev_observation.state
            self.get_logger().info(f"State diff: {state_diff}")
        self.prev_observation = deepcopy(self.observation)
        # action = self.broker.infer(self.observation)
        self.get_logger().info(f"Action: {action}")
        if all(float(x) == 0.0 for x in action.action) or action.action is None or len(action.action) != 7:
            self.get_logger().info("Skipping all-zero action.")
            return
        self.publish_action(action)

    def publish_action(self, action: Action) -> None:
        self.get_logger().info(f"Publishing action: {action}")
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        msg.position = [float(x) for x in action.action]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(0xAD)]
        msg.effort = [0.0] * 7
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    piper_single_node = ClientNode()
    try:
        rclpy.spin(piper_single_node)
    except KeyboardInterrupt:
        pass
    finally:
        piper_single_node.destroy_node()
        rclpy.shutdown()
