#!/usr/bin/env python3
"""Standalone heading-hold example using only generated Protobuf and Zenoh."""

from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import threading
import time
import uuid
from collections.abc import Sequence

import zenoh
from google.protobuf.message import DecodeError
from robot.v1 import common_pb2, control_pb2, navigation_pb2

QUATERNION_NORM_TOLERANCE = 1.0e-6
COVARIANCE_SYMMETRY_TOLERANCE = 1.0e-9
COMMAND_VALIDITY_NS = 100_000_000


class InputError(ValueError):
    """An input payload violated the example's canonical subset."""


def parse_positive_int(value: str) -> int:
    parsed = int(value, 10)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def parse_positive_finite(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("value must be finite and positive")
    return parsed


def parse_boot_id(value: str) -> bytes:
    try:
        parsed = bytes.fromhex(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("boot ID must be hexadecimal") from error
    if len(parsed) != 16 or not any(parsed):
        raise argparse.ArgumentTypeError("boot ID must contain 16 nonzero bytes")
    return parsed


def parse_lease_id(value: str) -> bytes:
    try:
        parsed = bytes.fromhex(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError("lease ID must be hexadecimal") from error
    if len(parsed) != 16 or not any(parsed):
        raise argparse.ArgumentTypeError("lease ID must contain 16 nonzero bytes")
    return parsed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Consume robot.v1 odometry and publish a bounded heading-hold intent."
    )
    parser.add_argument("--connect", required=True, help="Zenoh TCP endpoint")
    parser.add_argument("--input-key", required=True)
    parser.add_argument("--output-key", required=True)
    parser.add_argument("--source-id", default="python_heading_hold")
    parser.add_argument("--resource-id", default="chassis")
    parser.add_argument("--owner-id", required=True)
    parser.add_argument("--lease-id", required=True, type=parse_lease_id)
    parser.add_argument("--session-epoch", required=True, type=parse_positive_int)
    parser.add_argument("--requested-ttl-ns", required=True, type=parse_positive_int)
    parser.add_argument("--heading-gain", default=1.0, type=parse_positive_finite)
    parser.add_argument("--maximum-yaw-rate-radps", default=0.5, type=parse_positive_finite)
    parser.add_argument(
        "--boot-id",
        type=parse_boot_id,
        help="Deterministic qualification identity; normal starts generate a new UUID.",
    )
    return parser


def validate_identifier(value: str, field: str) -> None:
    if not value or len(value) > 64 or not value.isascii():
        raise InputError(f"{field} must be nonempty portable ASCII")
    if any(character in value for character in ("/", "*", "$", "#", "?")):
        raise InputError(f"{field} contains a reserved character")


def validate_vector(values: Sequence[float], field: str) -> None:
    if not all(math.isfinite(value) for value in values):
        raise InputError(f"{field} contains a non-finite value")


def validate_covariance(values: Sequence[float], field: str) -> None:
    if len(values) != 36:
        raise InputError(f"{field} must contain exactly 36 values")
    validate_vector(values, field)
    for row in range(6):
        if values[row * 6 + row] < 0.0:
            raise InputError(f"{field} contains a negative diagonal")
        for column in range(row + 1, 6):
            left = values[row * 6 + column]
            right = values[column * 6 + row]
            if abs(left - right) > COVARIANCE_SYMMETRY_TOLERANCE:
                raise InputError(f"{field} is asymmetric")


def decode_odometry(payload: bytes) -> navigation_pb2.OdometryState:
    value = navigation_pb2.OdometryState()
    try:
        value.ParseFromString(payload)
    except DecodeError as error:
        raise InputError("invalid OdometryState Protobuf") from error

    if not value.HasField("header"):
        raise InputError("odometry header is missing")
    if not value.HasField("pose") or not value.HasField("twist"):
        raise InputError("odometry pose or twist is missing")
    if not value.pose.HasField("position_m") or not value.pose.HasField("orientation"):
        raise InputError("odometry pose members are missing")
    if not value.twist.HasField("linear_mps") or not value.twist.HasField("angular_radps"):
        raise InputError("odometry twist members are missing")
    if value.header.valid_for_ns == 0:
        raise InputError("odometry validity must be nonzero")
    if len(value.header.boot_id) != 16 or not any(value.header.boot_id):
        raise InputError("odometry boot ID must contain 16 nonzero bytes")
    validate_identifier(value.header.source_id, "header.source_id")
    validate_identifier(value.parent_frame_id, "parent_frame_id")
    validate_identifier(value.child_frame_id, "child_frame_id")
    if value.header.frame_id != value.parent_frame_id:
        raise InputError("header frame must match odometry parent frame")
    if value.header.quality not in (
        common_pb2.DATA_QUALITY_VALID,
        common_pb2.DATA_QUALITY_DEGRADED,
    ):
        raise InputError("odometry quality is not usable")

    position = value.pose.position_m
    orientation = value.pose.orientation
    linear = value.twist.linear_mps
    angular = value.twist.angular_radps
    validate_vector((position.x, position.y, position.z), "pose.position_m")
    validate_vector(
        (orientation.x, orientation.y, orientation.z, orientation.w),
        "pose.orientation",
    )
    validate_vector((linear.x, linear.y, linear.z), "twist.linear_mps")
    validate_vector((angular.x, angular.y, angular.z), "twist.angular_radps")
    norm_squared = (
        orientation.x * orientation.x
        + orientation.y * orientation.y
        + orientation.z * orientation.z
        + orientation.w * orientation.w
    )
    if abs(norm_squared - 1.0) > QUATERNION_NORM_TOLERANCE:
        raise InputError("odometry orientation is not a unit quaternion")
    validate_covariance(value.pose_covariance, "pose_covariance")
    validate_covariance(value.twist_covariance, "twist_covariance")
    return value


def planar_yaw(orientation: common_pb2.Quaternion) -> float:
    sin_yaw = 2.0 * (
        orientation.w * orientation.z + orientation.x * orientation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        orientation.y * orientation.y + orientation.z * orientation.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class HeadingHold:
    """Stateful boot-local sequence and command-intent transformer."""

    def __init__(self, arguments: argparse.Namespace) -> None:
        self._arguments = arguments
        self._boot_id = arguments.boot_id or uuid.uuid4().bytes
        self._sequence = 0
        self._lock = threading.Lock()

    @property
    def boot_id(self) -> bytes:
        return self._boot_id

    def transform(self, payload: bytes) -> bytes:
        odometry = decode_odometry(payload)
        yaw = planar_yaw(odometry.pose.orientation)
        yaw_rate = max(
            -self._arguments.maximum_yaw_rate_radps,
            min(
                self._arguments.maximum_yaw_rate_radps,
                -self._arguments.heading_gain * yaw,
            ),
        )
        now_monotonic_ns = time.monotonic_ns()
        with self._lock:
            self._sequence += 1
            sequence = self._sequence

        header = common_pb2.Header(
            source_id=self._arguments.source_id,
            boot_id=self._boot_id,
            sequence=sequence,
            source_monotonic_ns=now_monotonic_ns,
            publish_unix_ns=time.time_ns(),
            valid_for_ns=min(odometry.header.valid_for_ns, COMMAND_VALIDITY_NS),
            clock_status=common_pb2.ClockStatus(
                state=common_pb2.CLOCK_STATE_LOCAL,
                uncertainty_ns=0,
            ),
            quality=common_pb2.DATA_QUALITY_VALID,
        )
        intent = control_pb2.ChassisSetpointIntent(
            intent=control_pb2.CommandIntent(
                header=header,
                resource_id=self._arguments.resource_id,
                owner_id=self._arguments.owner_id,
                lease_id=self._arguments.lease_id,
                session_epoch=self._arguments.session_epoch,
                command_id=uuid.uuid4().bytes,
                requested_ttl_ns=self._arguments.requested_ttl_ns,
            ),
            target=common_pb2.Twist2(
                vx_mps=0.0,
                vy_mps=0.0,
                yaw_rate_radps=yaw_rate,
            ),
        )
        return intent.SerializeToString()


def zenoh_config(endpoint: str) -> zenoh.Config:
    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", json.dumps([endpoint]))
    config.insert_json5("scouting/multicast/enabled", "false")
    config.insert_json5("scouting/gossip/enabled", "false")
    return config


def emit(record: dict[str, object], *, stream: object = sys.stdout) -> None:
    print(json.dumps(record, sort_keys=True), file=stream, flush=True)


def run(arguments: argparse.Namespace) -> int:
    for field in ("source_id", "resource_id", "owner_id"):
        validate_identifier(getattr(arguments, field), field)
    transformer = HeadingHold(arguments)
    stopped = threading.Event()

    def request_stop(_signal_number: int, _frame: object) -> None:
        stopped.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    with zenoh.open(zenoh_config(arguments.connect)) as session:
        publisher = session.declare_publisher(arguments.output_key)

        def on_sample(sample: zenoh.Sample) -> None:
            try:
                result = transformer.transform(bytes(sample.payload))
                publisher.put(
                    result,
                    encoding=zenoh.Encoding.APPLICATION_PROTOBUF,
                )
            except (InputError, DecodeError, ValueError) as error:
                emit(
                    {"event": "rejected", "reason": str(error)},
                    stream=sys.stderr,
                )

        subscriber = session.declare_subscriber(arguments.input_key, on_sample)
        emit(
            {
                "boot_id": transformer.boot_id.hex(),
                "event": "ready",
                "input_key": arguments.input_key,
                "output_key": arguments.output_key,
            }
        )
        stopped.wait()
        subscriber.undeclare()
        publisher.undeclare()
    return 0


def main() -> int:
    try:
        return run(build_parser().parse_args())
    except (InputError, RuntimeError, ValueError) as error:
        print(f"python-zenoh-algorithm: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
