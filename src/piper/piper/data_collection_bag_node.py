#!/usr/bin/env python3

import os
import signal
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, JointState


class DataCollectionBagNode(Node):
    def __init__(self) -> None:
        super().__init__("data_collection_bag_node")

        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("record_topic", "/data_collect")
        self.declare_parameter("output_dir", str(Path.home() / "bags"))
        self.declare_parameter("episode_prefix", "episode")

        self._control_hz = self.get_parameter("control_hz").value
        self._record_topic = self.get_parameter("record_topic").value
        self._output_dir = Path(self.get_parameter("output_dir").value)
        self._episode_prefix = self.get_parameter("episode_prefix").value

        self._topics = [
            "/image_raw",
            "/camera/intel_realsense_d435/color/image_raw",
            "/camera/intel_realsense_d435/depth/image_rect_raw",
            "/joint_states_single",
        ]

        self._topic_to_timestamp = {
            self._topics[0]: -1,
            self._topics[1]: -1,
            self._topics[2]: -1,
            self._topics[3]: -1,
        }

        self._recording_requested = False
        self._is_recording = False
        self._bag_process: Optional[subprocess.Popen] = None
        self._current_bag_path: Optional[Path] = None
        self._episode_count = 0

        self.create_subscription(Bool, self._record_topic, self._record_callback, 10)

        self.create_subscription(Image, self._topics[0], self._check_image0, 10)
        self.create_subscription(Image, self._topics[1], self._check_image1, 10)
        self.create_subscription(Image, self._topics[2], self._check_image2, 10)
        self.create_subscription(JointState, self._topics[3], self._check_joint_state, 10)

        self.create_timer(1.0 / float(self._control_hz), self._control_loop)

        self.get_logger().info(f"Running control loop at {self._control_hz:.1f} Hz")
        self.get_logger().info(f"Listening for data collection command on {self._record_topic}")
        self.get_logger().info(f"Bag output directory: {self._output_dir}")

    def _check_image0(self, msg: Image) -> None:
        self._topic_to_timestamp[self._topics[0]] = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

    def _check_image1(self, msg: Image) -> None:
        self._topic_to_timestamp[self._topics[1]] = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

    def _check_image2(self, msg: Image) -> None:
        self._topic_to_timestamp[self._topics[2]] = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

    def _check_joint_state(self, msg: JointState) -> None:
        self._topic_to_timestamp[self._topics[3]] = msg.header.stamp.sec * 1e9 + msg.header.stamp.nanosec

    def _record_callback(self, msg: Bool) -> None:
        self.get_logger().info(f"Recording requested: {msg.data}")
        self._recording_requested = msg.data

    def _control_loop(self) -> None:
        if self._recording_requested and not self._is_recording:
            self.get_logger().info(f"Starting recording")
            self._start_recording()
        elif not self._recording_requested and self._is_recording:
            self.get_logger().info(f"Stopping recording")
            self._stop_recording()

        current_ros_time = self.get_clock().now().to_msg().sec * 1e9 + self.get_clock().now().to_msg().nanosec

        for topic, timestamp in self._topic_to_timestamp.items():
            if timestamp == -1:
                return
            if abs(timestamp - current_ros_time) > 1e9:
                self.get_logger().warning(f"Topic {topic} is too old: {(current_ros_time - timestamp) * 1e-9} s")
                raise ValueError(f"Topic {topic} is too old: {(current_ros_time - timestamp) * 1e-9} s")

    def _start_recording(self) -> None:
        self._output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"{self._episode_prefix}_{self._episode_count:04d}_{timestamp}"
        bag_path = self._output_dir / bag_name

        cmd = ["ros2", "bag", "record", "-o", str(bag_path), *self._topics]
        self.get_logger().info(f"Starting rosbag for episode {self._episode_count}: {bag_path}")

        try:
            self._bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            self._current_bag_path = bag_path
            self._is_recording = True
            self._episode_count += 1
        except Exception as exc:
            self._bag_process = None
            self._is_recording = False
            self.get_logger().error(f"Failed to start rosbag recording: {exc}")

    def _stop_recording(self) -> None:
        if self._bag_process is None:
            self._is_recording = False
            return

        self.get_logger().info("Stopping rosbag recording and saving episode")

        try:
            pgid = os.getpgid(self._bag_process.pid)
            os.killpg(pgid, signal.SIGINT)
            self._bag_process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning("rosbag did not exit after SIGINT, forcing termination")
            try:
                pgid = os.getpgid(self._bag_process.pid)
                os.killpg(pgid, signal.SIGTERM)
                self._bag_process.wait(timeout=5.0)
            except Exception:
                self.get_logger().warning("rosbag did not exit after SIGTERM, killing process")
                try:
                    pgid = os.getpgid(self._bag_process.pid)
                    os.killpg(pgid, signal.SIGKILL)
                except Exception:
                    pass
        except Exception as exc:
            self.get_logger().error(f"Error while stopping rosbag: {exc}")
        finally:
            bag_path = self._current_bag_path
            self._bag_process = None
            self._current_bag_path = None
            self._is_recording = False
            if bag_path is not None:
                self.get_logger().info(f"Saved episode to: {bag_path}")
                try:
                    subprocess.run(["chmod", "-R", "777", str(bag_path)], check=True)
                except Exception as exc:
                    self.get_logger().warning(f"Failed to set permissions on {bag_path}: {exc}")

    def destroy_node(self) -> bool:
        if self._is_recording:
            self._stop_recording()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DataCollectionBagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
