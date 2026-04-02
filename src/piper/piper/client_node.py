import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState

from openpi_client.action_chunkers.rtc import InferenceTimeRTCBroker as RTCBroker
from openpi_client.client import BidirectionalWebsocket
from openpi_client.schemas import Action, LiberoObservation, Observation

import cv2
import numpy as np

TARGET_SIZE = (224, 224)

CONTROL_HZ = 1

class ClientNode(Node):
    """ROS2 node for the client"""

    def __init__(self) -> None:
        super().__init__('chunx_client')
        # ROS parameters
        self.declare_parameter('robot_id', "robot_0")
        self.declare_parameter('host', "localhost")
        self.declare_parameter('port', 8080)
        self.declare_parameter('control_hz', CONTROL_HZ)
        self.declare_parameter('execution_horizon', 10)
        self.declare_parameter('top_image_topic', "/camera/intel_realsense_d435i_top/color/image_raw")
        self.declare_parameter('wrist_image_topic', "/camera/intel_realsense_d435i_wrist/color/image_raw")
        self.declare_parameter('prompt', "pick up the legos and sort them into the correct bins.")

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

        self.observation = LiberoObservation(state=None, step=0, image=None, wrist_image=None, prompt=self.get_parameter('prompt').value)
        self.create_subscription(JointState, 'joint_states_single', self._update_joint_states, 1)
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
        self.get_logger().info(f"Received top image with shape: {image.shape}")

    def _update_wrist_image(self, image: Image) -> None:
        image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        h, w = image.shape[:2]
        start = (w - h) // 2
        image = image[:, start:start + h, :]  # center square crop
        image = cv2.resize(image, TARGET_SIZE)
        self.observation.wrist_image = image
        self.get_logger().info(f"Received wrist image with shape: {image.shape}")
    
    def publish_callback(self):
        if all(v is not None for v in self.observation.__dict__.values()):  
            # self.get_logger().info("Observation:" + ", ".join([f"{k}: {type(v)}" for k, v in self.observation.__dict__.items()]))
            action = self.broker.infer(self.observation)
            self.publish_action(action)
            self.step+=1
            return
        self.get_logger().info("Waiting for complete observation... missing: " + ", ".join([k for k, v in self.observation.__dict__.items() if v is None]))

    def publish_action(self, action: Action) -> None:
        self.get_logger().info(f"Publishing action: {action}")
        new_position = [float(pos) + float(act) for pos, act in zip(self.observation.state[:6], action.action[:6], strict=True)] + [float(act) for act in action.action[6:]]
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper']
        msg.position = list([float(x) for x in new_position])
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

