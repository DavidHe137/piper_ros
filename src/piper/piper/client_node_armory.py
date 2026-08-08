import argparse
import pathlib
import re
import socket
import sys
import time
from copy import deepcopy

import cv2
import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image, JointState

from armory_client.action_chunkers.rtc import InferenceTimeRTCBroker as RTCBroker
from armory_client.action_chunkers.naive_async import NaiveAsyncBroker
from armory_client.client import BidirectionalWebsocket
from armory_client.runtime.real_saver import RealSaver
from armory_client.schemas import Action, LiberoObservation

from piper.util.station_util import (
    get_station_number,
    get_station_namespace,
    default_rs_color_topic,
)

TARGET_SIZE = (224, 224)


class ClientNode(Node):
    """ROS2 node for the client"""

    def __init__(self, *, parameter_overrides: list[Parameter] | None = None) -> None:
        init_kwargs: dict = {}
        if parameter_overrides is not None:
            init_kwargs["parameter_overrides"] = parameter_overrides
        super().__init__("chunx_client", **init_kwargs)
        # ROS parameters
        self.declare_parameter("robot_id", f"robot_{get_station_number()}")
        # self.declare_parameter("host", "https://vvla--armory-serve-modalpolicyserver-stable-endpoint-dev.modal.run/")
        self.declare_parameter("host", "localhost") # if using skynet
        self.declare_parameter("port", 8080)
        self.declare_parameter("control_hz", 25.0)
        self.declare_parameter("min_execution_horizon", 5)
        self.declare_parameter("max_execution_horizon", 20)
        self.declare_parameter(
            "top_image_topic", default_rs_color_topic("intel_realsense_d435i_top")
        )
        self.declare_parameter(
            "wrist_image_topic", default_rs_color_topic("intel_realsense_d435i_wrist")
        )
        self.declare_parameter(
            "prompt", "put the red legos in the rotating red mug."
        )
        self.declare_parameter("sync_queue_size", 30)
        self.declare_parameter("sync_slop_sec", 0.1)

        ns = get_station_namespace()
        # Leading / so topics match data_collection_new.launch.py (PushRosNamespace + remap).
        self.joint_pub = self.create_publisher(JointState, f"/{ns}/joint_states", 1)
        self.create_timer(
            1.0 / self.get_parameter("control_hz").value, self.publish_callback
        )
        self.step = 0

        self.get_logger().info(
            f"Prompt: {self.get_parameter('prompt').value}, "
            f"Control Hz: {self.get_parameter('control_hz').value}, "
            f"Min Execution Horizon: {self.get_parameter('min_execution_horizon').value}, "
            f"Max Execution Horizon: {self.get_parameter('max_execution_horizon').value}"
        )

        self.ws_client = BidirectionalWebsocket(
            robot_id=self.get_parameter("robot_id").value,
            host=self.get_parameter("host").value,
            port=self.get_parameter("port").value,
            api_key=None,
            control_hz=self.get_parameter("control_hz").value,
        )

        self.broker = NaiveAsyncBroker(
            ws_client=self.ws_client,
            control_hz=self.get_parameter("control_hz").value,
            min_execution_horizon=self.get_parameter("min_execution_horizon").value,
            max_execution_horizon=self.get_parameter("max_execution_horizon").value,
            real=True,
        )

        # RealSaver: per-step trajectory capture so the orchestrator can
        # SFTP the data back and run calculate_metrics on it.
        self.declare_parameter("save_data", True)
        self.declare_parameter("save_video", False)
        self.declare_parameter("data_dir", "/datasets/armory_episodes")

        self._saver = None
        if bool(self.get_parameter("save_data").value):
            self._saver = RealSaver(
                out_dir=pathlib.Path(self.get_parameter("data_dir").value),
                robot_id=str(self.get_parameter("robot_id").value),
                prompt=str(self.get_parameter("prompt").value),
                control_hz=int(self.get_parameter("control_hz").value),
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

        # Match default create_subscription QoS (reliable) for broad publisher compatibility.
        image_qos = QoSProfile(depth=1)
        joint_qos = QoSProfile(depth=1)
        self._sub_joint = Subscriber(
            self, JointState, f"/{ns}/joint_states_single", qos_profile=joint_qos
        )
        self._sub_top = Subscriber(
            self,
            Image,
            self.get_parameter("top_image_topic").value,
            qos_profile=image_qos,
        )
        self._sub_wrist = Subscriber(
            self,
            Image,
            self.get_parameter("wrist_image_topic").value,
            qos_profile=image_qos,
        )
        self._observation_sync = ApproximateTimeSynchronizer(
            [self._sub_joint, self._sub_top, self._sub_wrist],
            queue_size=int(self.get_parameter("sync_queue_size").value),
            slop=float(self.get_parameter("sync_slop_sec").value),
        )
        self._observation_sync.registerCallback(self._on_synchronized_observation)

        self._sync_obs_count_window = 0
        self.create_timer(2.0, self._print_sync_observation_hz)

        self.get_logger().info(
            "Client node initialized with approximate-time observation sync."
        )

    def _process_top_image(self, image: Image) -> np.ndarray:
        arr = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1
        )
        h = arr.shape[0]
        arr = arr[:, :h, :]  # left-side square crop
        return cv2.resize(arr, TARGET_SIZE)

    def _process_wrist_image(self, image: Image) -> np.ndarray:
        arr = np.frombuffer(image.data, dtype=np.uint8).reshape(
            image.height, image.width, -1
        )
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
        self._sync_obs_count_window += 1

    def _print_sync_observation_hz(self) -> None:
        hz = self._sync_obs_count_window / 2.0
        self.get_logger().info(
            f"_on_synchronized_observation update rate: {hz:.2f} Hz (over last 2 s)"
        )
        self._sync_obs_count_window = 0

    def publish_callback(self):
        if not all(v is not None for v in self.observation.__dict__.values()):
            self.get_logger().info(
                "Waiting for complete observation... missing: "
                + ", ".join(
                    [k for k, v in self.observation.__dict__.items() if v is None]
                )
            )
            return

        self.observation.step = self.step
        action = self.broker.infer(self.observation)
        self.step += 1
        if self.prev_observation.state is not None:
            state_diff = self.observation.state - self.prev_observation.state
            # self.get_logger().info(f"State diff: {state_diff}")
        self.prev_observation = deepcopy(self.observation)
        # action = self.broker.infer(self.observation)
        self.get_logger().info(f"Action: {action}")
        if (
            all(float(x) == 0.0 for x in action.action)
            or action.action is None
            or len(action.action) != 7
        ):
            self.get_logger().info("Skipping all-zero action.")
            return
        self.publish_action(action)
        if self._saver is not None:
            try:
                self._saver.on_step(deepcopy(self.observation), action)
            except Exception as e:
                self.get_logger().warning(f"RealSaver.on_step failed: {e}")

    def publish_action(self, action: Action) -> None:
        # self.get_logger().info(f"Publishing action: {action}")
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "gripper",
        ]
        msg.position = [float(x) for x in action.action]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float(0xAD)]
        msg.effort = [0.0] * 7
        self.joint_pub.publish(msg)

    def destroy_node(self):
        # Flush RealSaver before super tears down the rclpy machinery so the
        # logger is still usable while we wait for the background executor.
        if self._saver is not None:
            try:
                self._saver.on_episode_end()
                self._saver.close()
                self.get_logger().info("RealSaver flushed.")
            except Exception as e:
                self.get_logger().warning(f"RealSaver flush failed: {e}")
            self._saver = None
        super().destroy_node()


def main(args=None) -> None:
    argv = sys.argv if args is None else args

    parser = argparse.ArgumentParser(
        prog="piper_client_armory",
        description=(
            "Piper Armory client. Control rate: use --control-hz, or "
            "e.g. --ros-args -p control_hz:=30"
        ),
    )
    parser.add_argument(
        "--control-hz",
        type=float,
        # MUST stay None when the flag is absent: any non-None value below
        # builds a parameter_overrides list that outranks --ros-args -p,
        # silently masking external overrides.
        default=None,
        metavar="HZ",
        help="Control loop frequency in Hz. Overrides any ros-args param if set.",
    )
    parser.add_argument(
        "--prompt",
        type=str,
        # MUST stay None when the flag is absent: any non-None value below
        # builds a parameter_overrides list that outranks --ros-args -p,
        # silently masking external overrides.
        default=None,
        metavar="PROMPT",
        help="Prompt for the client. Overrides any ros-args param if set.",
    )
    parser.add_argument(
        "--min-execution-horizon",
        type=int,
        # MUST stay None when the flag is absent: matches --control-hz so we
        # only build a Parameter override when explicitly given.
        default=None,
        metavar="N",
        help="Min action-chunk execution horizon (int). Overrides any ros-args param if set.",
    )
    parser.add_argument(
        "--max-execution-horizon",
        type=int,
        # MUST stay None when the flag is absent: matches --control-hz so we
        # only build a Parameter override when explicitly given.
        default=None,
        metavar="N",
        help="Max action-chunk execution horizon (int). Overrides any ros-args param if set.",
    )
    parser.add_argument(
        "--barrier",
        action="store_true",
        help=(
            "Block after node init until /tmp/armory_go.flag exists. Touch "
            "/tmp/armory_ready.flag to signal readiness. Used by run_real.py "
            "to start all robots' control loops at the same wall-clock moment."
        ),
    )
    parser.add_argument(
        "--barrier-timeout",
        type=float,
        default=60.0,
        metavar="SEC",
        help="Max seconds to wait for the go signal before proceeding anyway.",
    )
    filtered = remove_ros_args(argv)
    parsed, _unknown = parser.parse_known_args(filtered[1:])
    if parsed.control_hz is not None and parsed.control_hz <= 0:
        parser.error("--control-hz must be positive")
    if parsed.min_execution_horizon is not None and parsed.min_execution_horizon <= 0:
        parser.error("--min-execution-horizon must be positive")
    if parsed.max_execution_horizon is not None and parsed.max_execution_horizon <= 0:
        parser.error("--max-execution-horizon must be positive")
    if (
        parsed.min_execution_horizon is not None
        and parsed.max_execution_horizon is not None
        and parsed.min_execution_horizon > parsed.max_execution_horizon
    ):
        parser.error(
            "--min-execution-horizon must be <= --max-execution-horizon"
        )

    rclpy.init(args=argv)

    overrides: list[Parameter] | None = None
    # Build parameter overrides for any CLI flags that were actually set.
    override_params = []
    if parsed.control_hz is not None:
        override_params.append(
            Parameter(
                "control_hz",
                Parameter.Type.DOUBLE,
                float(parsed.control_hz),
            )
        )
    if parsed.prompt is not None:
        override_params.append(
            Parameter(
                "prompt",
                Parameter.Type.STRING,
                str(parsed.prompt),
            )
        )
    if parsed.min_execution_horizon is not None:
        override_params.append(
            Parameter(
                "min_execution_horizon",
                Parameter.Type.INTEGER,
                int(parsed.min_execution_horizon),
            )
        )
    if parsed.max_execution_horizon is not None:
        override_params.append(
            Parameter(
                "max_execution_horizon",
                Parameter.Type.INTEGER,
                int(parsed.max_execution_horizon),
            )
        )
    if override_params:
        overrides = override_params

    piper_single_node = ClientNode(parameter_overrides=overrides)
    if parsed.barrier:
        _await_startup_barrier(piper_single_node, parsed.barrier_timeout)
    try:
        rclpy.spin(piper_single_node)
    except KeyboardInterrupt:
        pass
    finally:
        piper_single_node.destroy_node()
        rclpy.shutdown()


def _await_startup_barrier(node, timeout_sec: float) -> None:
    """Block until the dispatcher writes /tmp/armory_go.flag, signaling ready first.

    Mirrors run_libero.py's multiprocessing.Barrier so every robot's control
    loop begins on the same wall clock tick rather than whenever each node
    happens to finish its warmup/handshake. Falls through after ``timeout_sec``
    so a missing dispatcher doesn't wedge the client indefinitely.
    """
    import pathlib
    ready_flag = pathlib.Path("/tmp/armory_ready.flag")
    go_flag = pathlib.Path("/tmp/armory_go.flag")
    # Stale go.flag from a previous trial would let us race past the barrier
    # before the dispatcher even sees us as ready.
    go_flag.unlink(missing_ok=True)
    ready_flag.touch()
    node.get_logger().info(
        f"barrier: ready, waiting for {go_flag} (timeout={timeout_sec:.1f}s)"
    )
    deadline = time.time() + timeout_sec
    while not go_flag.exists():
        if time.time() > deadline:
            node.get_logger().warning(
                "barrier: timed out without go signal; starting anyway"
            )
            break
        time.sleep(0.05)
    else:
        node.get_logger().info("barrier: go signal received; starting spin")
    # Don't leave our own readiness flag behind to confuse the next trial.
    ready_flag.unlink(missing_ok=True)