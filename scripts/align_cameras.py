#!/usr/bin/env python3
"""
Blend the Nth top-camera frame from a rosbag2 recording with the live camera feed
so you can align the robot / camera to match a demo.

Requires a sourced ROS 2 workspace (rclpy, rosbag2_py, sensor_msgs).

Example:
  source /opt/ros/humble/setup.bash
  python3 align_cameras.py --bag episode_20260426_215711 --nth 10

Keys when a local OpenCV window works:
  +/- or [/]  adjust blend (more recorded vs more live)
  r           reset blend to 0.5
  q / ESC     quit

Headless (no GTK / opencv-python-headless): overlay is published as sensor_msgs/Image
(default topic /align_cameras/overlay). View with ``rqt_image_view`` from another machine.
Tune blend with: ros2 topic pub -r 1 /align_cameras/alpha std_msgs/msg/Float32 "{data: 0.35}"
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

TOPIC_DEFAULT = "/camera/intel_realsense_d435i_top/color/image_raw"


def image_msg_to_bgr(msg: Image) -> np.ndarray:
    """Decode sensor_msgs/Image to BGR uint8 (H, W, 3)."""
    h, w = int(msg.height), int(msg.width)
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    enc = (msg.encoding or "").lower()

    if enc in ("rgb8", "bgr8"):
        arr = raw.reshape((h, w, 3))
        if enc == "rgb8":
            return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        return arr.copy()
    if enc in ("rgba8", "bgra8"):
        arr = raw.reshape((h, w, 4))
        code = cv2.COLOR_RGBA2BGR if enc == "rgba8" else cv2.COLOR_BGRA2BGR
        return cv2.cvtColor(arr, code)
    if enc == "mono8":
        g = raw.reshape((h, w))
        return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    if enc == "16uc1":
        d = raw.reshape((h, w)).view(np.uint16)
        d = np.clip(d.astype(np.float32) / 1000.0 * 255.0 / 5.0, 0, 255).astype(np.uint8)
        return cv2.cvtColor(d, cv2.COLOR_GRAY2BGR)

    raise ValueError(f"Unsupported image encoding: {msg.encoding!r}")


def bgr_to_image_msg(bgr: np.ndarray, stamp, frame_id: str = "") -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    h, w = bgr.shape[:2]
    msg.height = int(h)
    msg.width = int(w)
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = int(w * 3)
    msg.data = bgr.tobytes()
    return msg


def opencv_highgui_available() -> bool:
    """False for opencv-python-headless or OpenCV built without GTK/Cocoa."""
    try:
        cv2.namedWindow("__align_cameras_probe__", cv2.WINDOW_NORMAL)
        cv2.destroyWindow("__align_cameras_probe__")
        return True
    except cv2.error:
        return False


def load_nth_image_from_bag(bag_dir: Path, topic: str, nth: int) -> np.ndarray:
    """Return the nth message on ``topic`` as BGR (1-based ``nth``)."""
    try:
        from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
        from rclpy.serialization import deserialize_message
    except ImportError as exc:
        raise SystemExit(
            "Missing rosbag2_py or rclpy. Source your ROS 2 distro first, e.g.\n"
            "  source /opt/ros/humble/setup.bash\n"
            f"Original error: {exc}"
        ) from exc

    if nth < 1:
        raise ValueError("nth must be >= 1 (1-based index into that topic's stream)")

    meta = bag_dir / "metadata.yaml"
    if not meta.is_file():
        raise FileNotFoundError(f"Not a rosbag2 directory (missing metadata.yaml): {bag_dir}")

    storage_options = StorageOptions(uri=str(bag_dir.resolve()), storage_id="sqlite3")
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    reader.set_filter(StorageFilter(topics=[topic]))

    count = 0
    while reader.has_next():
        row = reader.read_next()
        tname, data = row[0], row[1]
        if tname != topic:
            continue
        count += 1
        if count < nth:
            continue
        msg = deserialize_message(data, Image)
        return image_msg_to_bgr(msg)

    raise RuntimeError(
        f"Fewer than {nth} messages on {topic!r} in bag {bag_dir} (saw {count})."
    )


class AlignOverlayNode(Node):
    def __init__(
        self,
        topic: str,
        ref_bgr: np.ndarray,
        alpha: float,
        sensor_data_qos: bool,
        publish_topic: str | None,
        alpha_cmd_topic: str,
    ) -> None:
        super().__init__("align_cameras_overlay")
        self._ref_bgr = ref_bgr
        self._alpha = float(alpha)
        self._live_bgr: np.ndarray | None = None
        self._overlay_pub = None
        if sensor_data_qos:
            qos = qos_profile_sensor_data
        else:
            qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Image, topic, self._on_image, qos)
        if publish_topic:
            pub_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
            self._overlay_pub = self.create_publisher(Image, publish_topic, pub_qos)
            self.create_subscription(Float32, alpha_cmd_topic, self._on_alpha_msg, 10)
            self.get_logger().info(
                f"Publishing overlay to {publish_topic!r}; "
                f"set blend (0..1 recorded weight) on {alpha_cmd_topic!r}"
            )
        self.get_logger().info(
            f"Subscribing to {topic!r}; blend alpha={self._alpha:.2f} "
            "(recorded weight; live weight is 1-alpha)"
        )

    def publish_overlay(self, bgr: np.ndarray) -> None:
        if self._overlay_pub is None:
            return
        msg = bgr_to_image_msg(bgr, self.get_clock().now().to_msg())
        self._overlay_pub.publish(msg)

    def _on_alpha_msg(self, msg: Float32) -> None:
        self.set_alpha(float(msg.data))

    def set_alpha(self, a: float) -> None:
        self._alpha = float(np.clip(a, 0.0, 1.0))

    def alpha(self) -> float:
        return self._alpha

    def _on_image(self, msg: Image) -> None:
        try:
            self._live_bgr = image_msg_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().warning(str(exc))
            return

    def maybe_blend(self) -> np.ndarray | None:
        if self._live_bgr is None:
            return None
        live = self._live_bgr
        ref = self._ref_bgr
        if ref.shape[:2] != live.shape[:2]:
            ref = cv2.resize(ref, (live.shape[1], live.shape[0]), interpolation=cv2.INTER_LINEAR)
        a = self._alpha
        return cv2.addWeighted(ref, a, live, 1.0 - a, 0.0)


def parse_args() -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    default_bag = here / "episode_20260426_215711"

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "--bag",
        type=Path,
        default=default_bag,
        help=f"rosbag2 directory (metadata.yaml + .db3). Default: {default_bag}",
    )
    p.add_argument(
        "--topic",
        default=TOPIC_DEFAULT,
        help="Bag + live image topic",
    )
    p.add_argument(
        "--nth",
        type=int,
        default=10,
        help="1-based index: use the Nth image message on that topic from the bag (default: 10)",
    )
    p.add_argument(
        "--alpha",
        type=float,
        default=0.5,
        help="Initial blend weight on recorded frame (0=live only, 1=recorded only)",
    )
    p.add_argument(
        "--sensor-data",
        action="store_true",
        help="Subscribe with sensor_data QoS (best-effort); use if the live stream uses RealSense-style QoS",
    )
    p.add_argument(
        "--no-gui",
        action="store_true",
        help="Do not use cv2.imshow (publish overlay only)",
    )
    p.add_argument(
        "--publish-topic",
        default=None,
        metavar="TOPIC",
        help="Publish blended sensor_msgs/Image (bgr8). If OpenCV has no GUI, defaults to /align_cameras/overlay",
    )
    p.add_argument(
        "--alpha-cmd-topic",
        default="/align_cameras/alpha",
        help="std_msgs/Float32: data = recorded blend weight (0..1). Used when publishing overlay",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    bag_dir = args.bag.expanduser().resolve()
    if not bag_dir.is_dir():
        print(f"Bag path is not a directory: {bag_dir}", file=sys.stderr)
        sys.exit(1)
    if not list(bag_dir.glob("*.db3")):
        print(
            f"No .db3 files under {bag_dir}. Copy the bag database next to metadata.yaml.",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"Loading frame #{args.nth} from {bag_dir} …")
    ref_bgr = load_nth_image_from_bag(bag_dir, args.topic, args.nth)
    print(f"Reference frame: {ref_bgr.shape[1]}x{ref_bgr.shape[0]} BGR")

    use_gui = not args.no_gui and opencv_highgui_available()
    publish_topic = args.publish_topic
    if publish_topic is None and not use_gui:
        publish_topic = "/align_cameras/overlay"
        print(
            "OpenCV GUI unavailable (headless OpenCV or no display). "
            f"Publishing overlay to {publish_topic!r} — open rqt_image_view on this topic.",
            file=sys.stderr,
        )

    rclpy.init()
    node = AlignOverlayNode(
        args.topic,
        ref_bgr,
        args.alpha,
        args.sensor_data,
        publish_topic,
        args.alpha_cmd_topic,
    )
    win = "align_cameras: bag vs live (addWeighted) | +/- blend | q quit"
    if use_gui:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            blended = node.maybe_blend()
            if blended is None:
                canvas = np.zeros_like(ref_bgr)
                cv2.putText(
                    canvas,
                    "Waiting for live images…",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                show = canvas
            else:
                show = blended.copy()
                t = f"alpha(recorded)={node.alpha():.2f}"
                cv2.putText(
                    show,
                    t,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            node.publish_overlay(show)
            if use_gui:
                cv2.imshow(win, show)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key in (ord("+"), ord("=")):
                    node.set_alpha(node.alpha() + 0.05)
                elif key in (ord("-"), ord("_")):
                    node.set_alpha(node.alpha() - 0.05)
                elif key == ord("["):
                    node.set_alpha(node.alpha() - 0.02)
                elif key == ord("]"):
                    node.set_alpha(node.alpha() + 0.02)
                elif key == ord("r"):
                    node.set_alpha(0.5)
    finally:
        if use_gui:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
