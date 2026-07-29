#!/usr/bin/env python3
"""Strictly evaluate rt-tests cyclictest v2.10 JSON without mutating it."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, NoReturn


def emit(status: str, reason: str, threshold_ns: int, max_ns: int = -1, cycles: int = 0) -> None:
    print(f"CYCLICTEST_STATUS={status}")
    print(f"CYCLICTEST_REASON={reason}")
    print(f"THRESHOLD_NS={threshold_ns}")
    print(f"MAX_LATENCY_NS={max_ns}")
    print(f"TOTAL_CYCLES={cycles}")


def incomplete(reason: str, threshold_ns: int) -> NoReturn:
    emit("incomplete", reason, threshold_ns)
    raise SystemExit(2)


def strict_int(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("json_file", type=Path)
    parser.add_argument("--threshold-ns", type=int, default=100_000)
    parser.add_argument("--expected-cpu", type=int)
    parser.add_argument("--minimum-cycles", type=int, default=1)
    args = parser.parse_args()

    if args.threshold_ns <= 0:
        parser.error("--threshold-ns must be greater than zero")
    if args.expected_cpu is not None and args.expected_cpu < 0:
        parser.error("--expected-cpu must be zero or greater")
    if args.minimum_cycles <= 0:
        parser.error("--minimum-cycles must be greater than zero")

    try:
        with args.json_file.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, UnicodeError, json.JSONDecodeError):
        incomplete("invalid_json", args.threshold_ns)

    if not isinstance(data, dict):
        incomplete("root_not_object", args.threshold_ns)
    return_code = data.get("return_code")
    if not strict_int(return_code) or return_code != 0:
        incomplete("tool_return_code", args.threshold_ns)
    resolution_in_ns = data.get("resolution_in_ns")
    if not strict_int(resolution_in_ns) or resolution_in_ns != 1:
        incomplete("unit_not_nanoseconds", args.threshold_ns)

    threads = data.get("thread")
    if not isinstance(threads, dict) or not threads:
        incomplete("missing_threads", args.threshold_ns)
    num_threads = data.get("num_threads")
    if not strict_int(num_threads) or num_threads != len(threads):
        incomplete("thread_count_mismatch", args.threshold_ns)

    maximum = 0
    total_cycles = 0
    for thread_id, stats in threads.items():
        if not isinstance(thread_id, str) or not isinstance(stats, dict):
            incomplete("invalid_thread_record", args.threshold_ns)
        cycles = stats.get("cycles")
        latency_max = stats.get("max")
        cpu = stats.get("cpu")
        if not strict_int(cycles) or cycles <= 0:
            incomplete("zero_or_invalid_cycles", args.threshold_ns)
        if not strict_int(latency_max) or latency_max < 0:
            incomplete("invalid_max_latency", args.threshold_ns)
        if not strict_int(cpu) or cpu < 0:
            incomplete("invalid_cpu", args.threshold_ns)
        if args.expected_cpu is not None and cpu != args.expected_cpu:
            incomplete("unexpected_cpu", args.threshold_ns)
        total_cycles += cycles
        maximum = max(maximum, latency_max)

    if total_cycles < args.minimum_cycles:
        incomplete("insufficient_cycles", args.threshold_ns)

    if maximum <= args.threshold_ns:
        emit("pass", "within_threshold", args.threshold_ns, maximum, total_cycles)
        return 0

    emit("fail", "max_latency_exceeded", args.threshold_ns, maximum, total_cycles)
    return 1


if __name__ == "__main__":
    sys.exit(main())
