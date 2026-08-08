#!/usr/bin/env python3

import os
import shutil
import signal
import subprocess
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Pose


class DataCollectionBagNode(Node):
    def __init__(self) -> None:
        super().__init__("data_collection_bag_node")

        self.declare_parameter("record_topic", "/data_collect")
        self.declare_parameter("discard_topic", "/data_discard")
        self.declare_parameter("output_dir", str(Path.home() / "bags"))
        self.declare_parameter("episode_prefix", "episode")

        # Per-topic expected Hz: list of [topic, expected_hz] pairs.
        # Warn when measured Hz drops more than 2 Hz below the expected value.
        self.declare_parameter(
            "topics_hz",
            [
                "/camera/intel_realsense_d435i_top/color/image_raw", "60.0",
                "/camera/intel_realsense_d435i_wrist/color/image_raw", "60.0",
                "/follower_joint_states", "200.0",
                "/master_joint_states", "200.0",
                "/follower_eef_pose", "200.0",
                "/master_eef_pose", "200.0",
            ],
        )

        self._record_topic = self.get_parameter("record_topic").value
        self._discard_topic = self.get_parameter("discard_topic").value
        self._output_dir = Path(self.get_parameter("output_dir").value)
        self._episode_prefix = self.get_parameter("episode_prefix").value

        # Parse topics_hz: flat list alternating [topic, hz, topic, hz, ...]
        raw_topics_hz = list(self.get_parameter("topics_hz").value)
        if len(raw_topics_hz) % 2 != 0:
            self.get_logger().error(
                "topics_hz must have an even number of entries (topic, hz pairs). "
                "Ignoring last entry."
            )
            raw_topics_hz = raw_topics_hz[:-1]
        self._topic_expected_hz: dict[str, float] = {}
        for i in range(0, len(raw_topics_hz), 2):
            topic = raw_topics_hz[i]
            try:
                hz = float(raw_topics_hz[i + 1])
            except ValueError:
                self.get_logger().error(
                    f"Invalid Hz value '{raw_topics_hz[i + 1]}' for topic '{topic}', defaulting to 0."
                )
                hz = 0.0
            self._topic_expected_hz[topic] = hz

        self._topics: list[str] = list(self._topic_expected_hz.keys())

        # Per-topic: last arrival wall-clock time and recent-window message count
        # for measuring actual Hz over a 1-second sliding window.
        self._topic_last_recv: dict[str, float] = {t: -1.0 for t in self._topics}
        # Ring of (wall_time,) for each msg within the last 2 s used for Hz est.
        self._topic_recv_times: dict[str, list[float]] = {t: [] for t in self._topics}

        self._recording_requested = False
        self._is_recording = False
        self._discard_pending = False
        self._bag_process: Optional[subprocess.Popen] = None
        self._current_bag_path: Optional[Path] = None

        self.create_subscription(Bool, self._record_topic, self._record_callback, 10)
        self.create_subscription(Bool, self._discard_topic, self._discard_callback, 10)

        # One subscription per topic for Hz tracking
        _msg_type_map = {
            "/camera/intel_realsense_d435i_top/color/image_raw": (Image, "_on_image_0"),
            "/camera/intel_realsense_d435i_wrist/color/image_raw": (Image, "_on_image_1"),
            "/follower_joint_states": (JointState, "_on_joint_0"),
            "/master_joint_states": (JointState, "_on_joint_1"),
            "/follower_eef_pose": (Pose, "_on_eef_pose_0"),
            "/master_eef_pose": (Pose, "_on_eef_pose_1"),
        }
        for topic in self._topics:
            if topic in _msg_type_map:
                msg_type, cb_name = _msg_type_map[topic]
                cb = getattr(self, cb_name)
                self.create_subscription(msg_type, topic, cb, 10)
            else:
                self.get_logger().warning(
                    f"No subscription handler for topic '{topic}'; Hz monitoring disabled for it."
                )

        # Hz-check timer runs at 1 Hz
        self.create_timer(1.0, self._hz_check_loop)

        self.get_logger().info(f"Listening for record commands on '{self._record_topic}'")
        self.get_logger().info(f"Listening for discard commands on '{self._discard_topic}'")
        self.get_logger().info(f"Bag output directory: {self._output_dir}")
        for topic, hz in self._topic_expected_hz.items():
            self.get_logger().info(f"  {topic}  expected {hz:.0f} Hz  (warn if < {hz - 2:.0f} Hz)")

    # ── per-topic arrival callbacks ───────────────────────────────────────────

    def _record_arrival(self, topic: str) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        self._topic_last_recv[topic] = now
        self._topic_recv_times[topic].append(now)
        # Prune entries older than 2 s for the sliding window
        cutoff = now - 2.0
        self._topic_recv_times[topic] = [
            t for t in self._topic_recv_times[topic] if t >= cutoff
        ]

    def _on_image_0(self, _msg: Image) -> None:
        self._record_arrival(self._topics[0])

    def _on_image_1(self, _msg: Image) -> None:
        self._record_arrival(self._topics[1])

    def _on_joint_0(self, _msg: JointState) -> None:
        self._record_arrival(self._topics[2])

    def _on_joint_1(self, _msg: JointState) -> None:
        self._record_arrival(self._topics[3])

    def _on_eef_pose_0(self, _msg: Pose) -> None:
        self._record_arrival(self._topics[4])

    def _on_eef_pose_1(self, _msg: Pose) -> None:
        self._record_arrival(self._topics[5])

    # ── command callbacks ─────────────────────────────────────────────────────

    def _record_callback(self, msg: Bool) -> None:
        self.get_logger().info(f"Record signal received: {msg.data}")
        self._recording_requested = msg.data
        if msg.data:
            # Clear discard flag when a new recording starts
            self._discard_pending = False
        if self._recording_requested and not self._is_recording:
            self._start_recording()
        elif not self._recording_requested and self._is_recording:
            self._stop_recording(discard=self._discard_pending)
            self._discard_pending = False

    def _discard_callback(self, msg: Bool) -> None:
        if msg.data:
            self.get_logger().info(
                "Discard signal received — current episode will be deleted on stop."
            )
            self._discard_pending = True

    # ── Hz monitoring ─────────────────────────────────────────────────────────

    def _hz_check_loop(self) -> None:
        """Called at 1 Hz. Estimates each topic's rate over the last 2 s."""
        now = self.get_clock().now().nanoseconds * 1e-9
        for topic in self._topics:
            expected = self._topic_expected_hz.get(topic, 0.0)
            if expected <= 0.0:
                continue

            times = self._topic_recv_times[topic]
            cutoff = now - 2.0
            recent = [t for t in times if t >= cutoff]
            self._topic_recv_times[topic] = recent  # keep pruned

            if not recent:
                if self._topic_last_recv[topic] < 0.0:
                    self.get_logger().warning(
                        f"[Hz] {topic}: no messages received yet (expected {expected:.0f} Hz)"
                    )
                else:
                    self.get_logger().warning(
                        f"[Hz] {topic}: no messages in last 2 s (expected {expected:.0f} Hz)"
                    )
                continue

            window = now - recent[0]
            if window < 0.1:
                continue  # too short a window to estimate reliably
            measured_hz = (len(recent) - 1) / window if len(recent) > 1 else 0.0
            threshold = expected - 2.0
            if measured_hz < threshold:
                self.get_logger().warning(
                    f"[Hz] {topic}: {measured_hz:.1f} Hz "
                    f"(expected {expected:.0f} Hz, threshold {threshold:.0f} Hz)"
                )

    # ── bag management ────────────────────────────────────────────────────────

    def _start_recording(self) -> None:
        self._output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_name = f"{self._episode_prefix}_{timestamp}"
        bag_path = self._output_dir / bag_name

        cmd = ["ros2", "bag", "record", "-o", str(bag_path), *self._topics]
        self.get_logger().info(f"Starting rosbag: {bag_path}")

        try:
            self._bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            self._current_bag_path = bag_path
            self._is_recording = True
        except Exception as exc:
            self._bag_process = None
            self._is_recording = False
            self.get_logger().error(f"Failed to start rosbag: {exc}")

    def _stop_recording(self, discard: bool = False) -> None:
        if self._bag_process is None:
            self._is_recording = False
            return

        bag_path = self._current_bag_path
        action = "Discarding" if discard else "Saving"
        self.get_logger().info(f"{action} rosbag recording: {bag_path}")

        try:
            pgid = os.getpgid(self._bag_process.pid)
            os.killpg(pgid, signal.SIGINT)
            self._bag_process.wait(timeout=10.0)
        except subprocess.TimeoutExpired:
            self.get_logger().warning("rosbag did not exit after SIGINT, forcing SIGTERM")
            try:
                pgid = os.getpgid(self._bag_process.pid)
                os.killpg(pgid, signal.SIGTERM)
                self._bag_process.wait(timeout=5.0)
            except Exception:
                self.get_logger().warning("rosbag did not exit after SIGTERM, sending SIGKILL")
                try:
                    pgid = os.getpgid(self._bag_process.pid)
                    os.killpg(pgid, signal.SIGKILL)
                except Exception:
                    pass
        except Exception as exc:
            self.get_logger().error(f"Error stopping rosbag: {exc}")
        finally:
            self._bag_process = None
            self._current_bag_path = None
            self._is_recording = False

            if bag_path is not None:
                if discard:
                    try:
                        if bag_path.exists():
                            shutil.rmtree(str(bag_path))
                            self.get_logger().info(f"Deleted bag: {bag_path}")
                        else:
                            self.get_logger().warning(
                                f"Bag path not found for deletion: {bag_path}"
                            )
                    except Exception as exc:
                        self.get_logger().error(f"Failed to delete bag {bag_path}: {exc}")
                else:
                    self.get_logger().info(f"Saved episode to: {bag_path}")
                    try:
                        subprocess.run(
                            ["chmod", "-R", "777", str(bag_path)], check=True
                        )
                    except Exception as exc:
                        self.get_logger().warning(
                            f"Failed to set permissions on {bag_path}: {exc}"
                        )

    def destroy_node(self) -> bool:
        if self._is_recording:
            self._stop_recording(discard=False)
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
