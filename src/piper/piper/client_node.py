import re
import socket
from copy import deepcopy

import cv2
import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, JointState

from openpi_client.action_chunkers.rtc import InferenceTimeRTCBroker as RTCBroker
from openpi_client.action_chunkers.naive_async import NaiveAsyncBroker
from openpi_client.client import BidirectionalWebsocket
from openpi_client.schemas import Action, LiberoObservation

from piper.util.station_util import get_station_number, get_station_namespace, default_rs_color_topic

TARGET_SIZE = (224, 224)

CONTROL_HZ = 20


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
        self.declare_parameter('sync_queue_size', 30)
        self.declare_parameter('sync_slop_sec', 0.1)

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

        self.broker = NaiveAsyncBroker(
            ws_client=self.ws_client,
            control_hz=self.get_parameter('control_hz').value,
            execution_horizon=self.get_parameter('execution_horizon').value,
        )

        self.observation = LiberoObservation(state=None, step=0, image=None, wrist_image=None, prompt=self.get_parameter('prompt').value)
        self.observation_step = 0
        self.action_step = 0 # TODO: use action step on server
        self.prev_observation = LiberoObservation(state=None, step=0, image=None, wrist_image=None, prompt=self.get_parameter('prompt').value)

        # Match default create_subscription QoS (reliable) for broad publisher compatibility.
        image_qos = QoSProfile(depth=1)
        joint_qos = QoSProfile(depth=1)
        self._sub_joint = Subscriber(
            self, JointState, f'/{ns}/joint_states_single', qos_profile=joint_qos
        )
        self._sub_top = Subscriber(
            self, Image, self.get_parameter('top_image_topic').value, qos_profile=image_qos
        )
        self._sub_wrist = Subscriber(
            self, Image, self.get_parameter('wrist_image_topic').value, qos_profile=image_qos
        )
        self._observation_sync = ApproximateTimeSynchronizer(
            [self._sub_joint, self._sub_top, self._sub_wrist],
            queue_size=int(self.get_parameter('sync_queue_size').value),
            slop=float(self.get_parameter('sync_slop_sec').value),
        )
        self._observation_sync.registerCallback(self._on_synchronized_observation)

        self.get_logger().info("Client node initialized with approximate-time observation sync.")

    def _process_top_image(self, image: Image) -> np.ndarray:
        arr = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        h = arr.shape[0]
        arr = arr[:, :h, :]  # left-side square crop
        return cv2.resize(arr, TARGET_SIZE)

    def _process_wrist_image(self, image: Image) -> np.ndarray:
        arr = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        h, w = arr.shape[:2]
        start = (w - h) // 2
        arr = arr[:, start : start + h, :]  # center square crop
        return cv2.resize(arr, TARGET_SIZE)

    def _on_synchronized_observation(
        self, joint_msg: JointState, top_image: Image, wrist_image: Image
    ) -> None:
        """Apply joint + both cameras together when headers fall within sync_slop_sec."""
        self.observation.state = np.array(joint_msg.position.tolist())
        self.observation.image = self._process_top_image(top_image)
        self.observation.wrist_image = self._process_wrist_image(wrist_image)
        self.observation_step += 1
    
    def publish_callback(self):
        if not all(v is not None for v in self.observation.__dict__.values()):
            self.get_logger().info("Waiting for complete observation... missing: " + ", ".join([k for k, v in self.observation.__dict__.items() if v is None]))
            return

        self.observation.step = self.step
        action = self.broker.infer(self.observation)
        self.step += 1
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
