import re
import socket
from copy import deepcopy

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image, JointState

from openpi_client.action_chunkers.rtc import InferenceTimeRTCBroker as RTCBroker
from openpi_client.client import BidirectionalWebsocket
from openpi_client.schemas import Action, LiberoObservation

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
        self.declare_parameter('host', "https://rohan-bansal--openpi-serve-modalpolicyserver-stable--98f7b0-dev.modal.run")
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

        self.broker = RTCBroker(
            ws_client=self.ws_client,
            control_hz=self.get_parameter('control_hz').value,
            execution_horizon=self.get_parameter('execution_horizon').value,
        )

        self.action_step = 0 # TODO: use action step on server
        self.prev_observation = LiberoObservation(state=None, step=0, image=None, wrist_image=None, prompt=self.get_parameter('prompt').value)

        # Latest raw messages cached per topic; synchronized on-demand in publish_callback.
        self._latest_joint: JointState | None = None
        self._latest_top: Image | None = None
        self._latest_wrist: Image | None = None

        image_qos = QoSProfile(depth=1)
        joint_qos = QoSProfile(depth=1)
        self.create_subscription(
            JointState, f'/{ns}/joint_states_single',
            lambda msg: setattr(self, '_latest_joint', msg), joint_qos
        )
        self.create_subscription(
            Image, self.get_parameter('top_image_topic').value,
            lambda msg: setattr(self, '_latest_top', msg), image_qos
        )
        self.create_subscription(
            Image, self.get_parameter('wrist_image_topic').value,
            lambda msg: setattr(self, '_latest_wrist', msg), image_qos
        )

        self.get_logger().info("Client node initialized (observation synchronized on-demand in publish_callback).")

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

    def _get_synchronized_observation(self) -> LiberoObservation | None:
        """Build a synchronized observation from the latest cached messages.

        Returns None (and logs which topics are missing) if any topic has not
        yet received a message, or if the timestamps of the cached messages
        diverge by more than sync_slop_sec.
        """
        joint_msg = self._latest_joint
        top_img   = self._latest_top
        wrist_img = self._latest_wrist

        missing = [name for name, msg in [('joint', joint_msg), ('top_image', top_img), ('wrist_image', wrist_img)] if msg is None]
        if missing:
            self.get_logger().info("Waiting for complete observation... missing: " + ", ".join(missing))
            return None

        slop = float(self.get_parameter('sync_slop_sec').value)
        stamps = [
            joint_msg.header.stamp.sec + joint_msg.header.stamp.nanosec * 1e-9,
            top_img.header.stamp.sec   + top_img.header.stamp.nanosec   * 1e-9,
            wrist_img.header.stamp.sec + wrist_img.header.stamp.nanosec * 1e-9,
        ]
        if max(stamps) - min(stamps) > slop:
            self.get_logger().info(
                f"Observation timestamps out of sync (spread {max(stamps)-min(stamps):.3f}s > slop {slop}s), skipping."
            )
            return None

        return LiberoObservation(
            state=np.array(joint_msg.position.tolist()),
            step=self.step,
            image=self._process_top_image(top_img),
            wrist_image=self._process_wrist_image(wrist_img),
            prompt=self.get_parameter('prompt').value,
        )

    def publish_callback(self):
        observation = self._get_synchronized_observation()
        if observation is None:
            return

        # TODO: maybe have two timers
        action = self.broker.infer(observation)
        self.step += 1
        if self.prev_observation.state is not None:
            state_diff = observation.state - self.prev_observation.state
            self.get_logger().info(f"State diff: {state_diff}")
        self.prev_observation = deepcopy(observation)
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
