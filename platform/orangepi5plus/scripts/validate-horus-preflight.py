#!/usr/bin/env python3
"""Validate the reviewed Horus adapter before target evidence is created."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any


PAYLOADS = [64, 256, 1024]
KINDS = ["latest", "ordered"]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--source-commit", required=True)
    parser.add_argument("--capacity", type=int, required=True)
    parser.add_argument("--expected-arch", required=True)
    return parser.parse_args()


def read_object(path: Path) -> dict[str, Any]:
    with path.open("rb") as source:
        value = json.load(source)
    if not isinstance(value, dict):
        raise ValueError("root must be an object")
    return value


def main() -> int:
    args = parse_args()
    expected = {
        "schema_version": 1,
        "candidate": "horus",
        "source_commit": args.source_commit,
        "target_os": "linux",
        "target_arch": args.expected_arch,
        "capacity": args.capacity,
        "payload_bytes": PAYLOADS,
        "kinds": KINDS,
        "latency_clock": "CLOCK_MONOTONIC",
        "operation_semantics": "try_publish_call",
        "memory_policy": "lock-required",
        "memory_lock_mechanism": "mlockall_current_future",
        "memory_lock_verified": True,
        "payload_contract": "repr_c_bytemuck_pod",
        "horus_generic_pod_detection": "needs_drop_heuristic_not_payload_proof",
        "hot_path_api": "try_send_try_recv",
        "fault_suite_ready": True,
    }
    if args.capacity < 16 or args.capacity > 1024 or args.capacity & (args.capacity - 1):
        print("error: capacity must be a power of two in 16..=1024", file=sys.stderr)
        return 2
    try:
        value = read_object(args.input)
        for key, expected_value in expected.items():
            actual = value.get(key)
            if type(actual) is not type(expected_value) or actual != expected_value:
                raise ValueError(f"{key} mismatch")
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
        print(f"error: invalid Horus preflight: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
