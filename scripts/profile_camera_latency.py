#!/usr/bin/env python3
"""Compare consecutive hw_timestamp vs frame_timestamp deltas on RealSense color/metadata."""

from __future__ import annotations

import argparse
import json
import re
import socket
import sys
from typing import Any

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data

try:
    from realsense2_camera_msgs.msg import Metadata as RSMetadata
except ImportError:
    try:
        from realsense2_camera.msg import Metadata as RSMetadata
    except ImportError as exc:  # pragma: no cover - runtime ROS env
        raise SystemExit(
            "Could not import RealSense Metadata message. "
            "Install the realsense2_camera / realsense2_camera_msgs packages "
            "and source the ROS workspace, then retry."
        ) from exc


def get_station_number() -> int | None:
    hostname = socket.gethostname()
    match = re.search(r"robotics-education-lab(\d+)(?:\..*)?$", hostname)
    if match:
        return int(match.group(1))
    return None


def get_station_namespace() -> str:
    n = get_station_number()
    return f"station{n}" if n is not None else "station"


def default_color_metadata_topic(camera_node_name: str) -> str:
    ns = get_station_namespace()
    return f"/{ns}/{camera_node_name}/color/metadata"


def _summarize(name: str, x: np.ndarray) -> str:
    if x.size == 0:
        return f"{name}: (no data)"
    return (
        f"{name}: n={x.size}  mean={np.mean(x):.6g}  std={np.std(x):.6g}  "
        f"min={np.min(x):.6g}  p50={np.median(x):.6g}  max={np.max(x):.6g}"
    )


def _pick_qos(kind: str):
    if kind == "sensor_data":
        return qos_profile_sensor_data
    return QoSProfile(depth=100)


class CameraMetadataProfiler(Node):
    """Collects Δ(hw) and Δ(frame_timestamp) between consecutive metadata messages."""

    def __init__(
        self,
        topic: str,
        max_pairs: int,
        qos_kind: str,
        print_every: int,
    ) -> None:
        super().__init__("profile_camera_latency")
        self._topic = topic
        self._max_pairs = max_pairs
        self._print_every = print_every
        self._prev_hw: float | None = None
        self._prev_ft: float | None = None
        self._delta_hw: list[float] = []
        self._delta_ft: list[float] = []
        self._parse_errors = 0
        self._missing_ts = 0
        self._pairs = 0
        self.finished = False

        qos = _pick_qos(qos_kind)
        self.create_subscription(RSMetadata, topic, self._on_metadata, qos)
        self.get_logger().info(
            f"Listening on {topic!r} (qos={qos_kind}); need {max_pairs} inter-frame pairs "
            "(Ctrl+C to stop early)."
        )

    def _on_metadata(self, msg: Any) -> None:
        try:
            meta = json.loads(msg.json_data)
        except (json.JSONDecodeError, TypeError, AttributeError):
            self._parse_errors += 1
            return

        try:
            hw = float(meta["hw_timestamp"])
            ft = float(meta["frame_timestamp"])
        except (KeyError, TypeError, ValueError):
            self._missing_ts += 1
            return

        if self._prev_hw is None or self._prev_ft is None:
            self._prev_hw, self._prev_ft = hw, ft
            return

        d_hw = hw - self._prev_hw
        d_ft = ft - self._prev_ft
        self._prev_hw, self._prev_ft = hw, ft

        self._delta_hw.append(d_hw)
        self._delta_ft.append(d_ft)
        self._pairs += 1

        if self._print_every > 0 and self._pairs % self._print_every == 0:
            self.get_logger().info(f"Collected {self._pairs} pairs…")

        if self._pairs >= self._max_pairs:
            self.finished = True

    def report(self) -> int:
        if not self._delta_hw:
            print("No inter-frame pairs collected.", file=sys.stderr)
            if self._parse_errors:
                print(f"JSON parse errors: {self._parse_errors}", file=sys.stderr)
            if self._missing_ts:
                print(f"Messages missing hw/frame timestamps: {self._missing_ts}", file=sys.stderr)
            return 1

        d_hw = np.asarray(self._delta_hw, dtype=np.float64)
        d_ft = np.asarray(self._delta_ft, dtype=np.float64)
        residual = d_ft - d_hw

        print(f"Topic: {self._topic}")
        print(f"Inter-frame pairs: {d_hw.size}")
        if self._parse_errors or self._missing_ts:
            print(f"(parse_errors={self._parse_errors}, missing_ts={self._missing_ts})")
        print()
        print(_summarize("Δ hw_timestamp (consecutive)", d_hw))
        print(_summarize("Δ frame_timestamp (consecutive)", d_ft))
        print(_summarize("Δ frame_timestamp − Δ hw_timestamp", residual))
        print()
        mae = float(np.mean(np.abs(residual)))
        max_abs = float(np.max(np.abs(residual)))
        print(f"Mean |Δframe − Δhw|: {mae:.6g}")
        print(f"Max  |Δframe − Δhw|: {max_abs:.6g}")
        # Same clock / scaling → residuals near zero; large residuals suggest different units or domains.
        rel = np.divide(
            residual,
            np.where(np.abs(d_hw) > 1e-30, d_hw, np.nan),
        )
        rel = rel[np.isfinite(rel)]
        if rel.size:
            print(
                f"Ratio (Δframe/Δhw): median={float(np.median(rel)):.6g}  "
                f"mean={float(np.mean(rel)):.6g}"
            )
        return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Subscribe to RealSense color/metadata and compare consecutive differences "
            "of hw_timestamp vs frame_timestamp (from json_data)."
        )
    )
    parser.add_argument(
        "--topic",
        type=str,
        default=None,
        help="Full metadata topic (default: .../<camera>/color/metadata for this station).",
    )
    parser.add_argument(
        "--camera",
        type=str,
        default="intel_realsense_d435i_wrist",
        help="Camera node name under the station namespace (used only if --topic is omitted).",
    )
    parser.add_argument(
        "--pairs",
        type=int,
        default=500,
        help="Stop after this many consecutive inter-frame pairs (default: 500).",
    )
    parser.add_argument(
        "--qos",
        choices=("sensor_data", "default"),
        default="sensor_data",
        help="Subscriber QoS preset (default: sensor_data, typical for RealSense streams).",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=100,
        help="Log progress every N pairs (0 to disable).",
    )
    args = parser.parse_args(argv)

    topic = args.topic or default_color_metadata_topic(args.camera)
    max_pairs = max(1, args.pairs)

    rclpy.init()
    node = CameraMetadataProfiler(
        topic=topic,
        max_pairs=max_pairs,
        qos_kind=args.qos,
        print_every=max(0, args.print_every),
    )
    exit_code = 1
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
        exit_code = node.report()
    except KeyboardInterrupt:
        print("\nInterrupted.", file=sys.stderr)
        exit_code = node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
