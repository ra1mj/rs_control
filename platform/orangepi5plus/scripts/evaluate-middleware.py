#!/usr/bin/env python3
"""Validate a complete target middleware matrix without average-only ranking."""

from __future__ import annotations

import argparse
import json
import sys
from decimal import Decimal, InvalidOperation
from pathlib import Path
from typing import Any


PROJECT_CANDIDATES = (
    ("local", "thread"),
    ("shm", "thread"),
    ("shm", "process"),
)
KINDS = ("latest", "ordered")
PAYLOADS = (64, 256, 1024)
PERF_EVENTS = (
    "cycles",
    "instructions",
    "task-clock",
    "context-switches",
    "cpu-migrations",
    "page-faults",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--capacity", type=int, required=True)
    parser.add_argument("--warmup", type=int, required=True)
    parser.add_argument("--samples", type=int, required=True)
    parser.add_argument("--expected-arch", required=True)
    parser.add_argument("--horus-commit", required=True)
    return parser.parse_args()


def read_json(path: Path) -> dict[str, Any]:
    with path.open("rb") as source:
        value = json.load(source)
    if not isinstance(value, dict):
        raise ValueError("root must be an object")
    return value


def exact_int(value: Any, name: str, minimum: int = 0) -> int:
    if type(value) is not int or value < minimum:
        raise ValueError(f"{name} must be an integer >= {minimum}")
    return value


def validate_histogram(value: Any, samples: int, name: str) -> list[int]:
    if not isinstance(value, list) or len(value) != 64:
        raise ValueError(f"{name} must contain exactly 64 buckets")
    buckets = [exact_int(item, "histogram bucket") for item in value]
    if sum(buckets) != samples:
        raise ValueError(f"{name} sample total mismatch")
    return buckets


def histogram_bucket(value: int) -> int:
    return 0 if value == 0 else min(value.bit_length(), 63)


def validate_histogram_max(buckets: list[int], maximum: int, name: str) -> None:
    if buckets[histogram_bucket(maximum)] == 0:
        raise ValueError(f"{name} does not belong to a populated histogram bucket")


def validate_counters(value: Any, total: int) -> dict[str, int]:
    if not isinstance(value, dict):
        raise ValueError("counters must be an object")
    counters = {
        key: exact_int(value.get(key), f"counter {key}")
        for key in ("published", "consumed", "overwritten", "full", "disconnected")
    }
    if counters["published"] != total or counters["consumed"] != total:
        raise ValueError("paired-run published/consumed counter mismatch")
    if any(counters[key] != 0 for key in ("overwritten", "full", "disconnected")):
        raise ValueError("paired-run loss/backpressure counter is nonzero")
    return counters


def validate_perf(path: Path) -> dict[str, str]:
    observations: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        fields = line.split(";")
        if len(fields) < 3:
            raise ValueError("invalid perf stat field count")
        raw_value = fields[0].strip()
        event = fields[2].strip()
        if event not in PERF_EVENTS:
            continue
        if event in observations:
            raise ValueError(f"duplicate perf event: {event}")
        try:
            numeric = Decimal(raw_value)
        except InvalidOperation as error:
            raise ValueError(f"perf event {event} was not counted") from error
        if not numeric.is_finite() or numeric < 0:
            raise ValueError(f"invalid perf event value: {event}")
        if event in ("cycles", "instructions") and numeric <= 0:
            raise ValueError(f"perf event {event} must be positive")
        if event != "task-clock" and numeric != numeric.to_integral_value():
            raise ValueError(f"perf event {event} must be an integer")
        observations[event] = raw_value
    missing = sorted(set(PERF_EVENTS) - observations.keys())
    if missing:
        raise ValueError(f"missing perf events: {','.join(missing)}")
    return {event: observations[event] for event in PERF_EVENTS}


def validate_common(
    value: dict[str, Any],
    *,
    kind: str,
    payload: int,
    capacity: int,
    warmup: int,
    samples: int,
    perf_path: Path,
) -> dict[str, Any]:
    if value.get("schema_version") != 2:
        raise ValueError("unsupported schema_version")
    expected = {
        "kind": kind,
        "payload_bytes": payload,
        "capacity": capacity,
        "warmup": warmup,
        "samples": samples,
        "latency_semantics": "one_way_receive",
        "latency_clock": "CLOCK_MONOTONIC",
        "latency_jitter_semantics": "max_minus_min",
        "operation_semantics": "try_publish_call",
        "operation_clock": "CLOCK_MONOTONIC",
    }
    for key, expected_value in expected.items():
        if value.get(key) != expected_value:
            raise ValueError(f"{key} mismatch")
    latency_buckets = validate_histogram(
        value.get("latency_log2_ns"), samples, "latency_log2_ns"
    )
    operation_buckets = validate_histogram(
        value.get("operation_log2_ns"), samples, "operation_log2_ns"
    )
    latency_min = exact_int(value.get("latency_min_ns"), "latency_min_ns")
    latency_max = exact_int(value.get("latency_max_ns"), "latency_max_ns")
    if latency_min > latency_max:
        raise ValueError("latency minimum exceeds maximum")
    if exact_int(value.get("latency_jitter_span_ns"), "latency_jitter_span_ns") != (
        latency_max - latency_min
    ):
        raise ValueError("latency jitter span is not max minus min")
    operation_max = exact_int(value.get("operation_max_ns"), "operation_max_ns")
    validate_histogram_max(latency_buckets, latency_max, "latency maximum")
    validate_histogram_max(operation_buckets, operation_max, "operation maximum")
    wall_time = exact_int(value.get("wall_time_ns"), "wall_time_ns", 1)
    cpu_time = exact_int(value.get("cpu_time_ns"), "cpu_time_ns", 1)
    peer_cpu_time = exact_int(value.get("peer_cpu_time_ns"), "peer_cpu_time_ns")
    if value.get("cpu_time_source") != "CLOCK_PROCESS_CPUTIME_ID":
        raise ValueError("CPU time source mismatch")
    peak_resident = exact_int(
        value.get("peak_resident_bytes"), "peak_resident_bytes", 1
    )
    peer_peak_resident = exact_int(
        value.get("peer_peak_resident_bytes"), "peer_peak_resident_bytes"
    )
    if value.get("memory_source") != "proc_self_status_VmHWM":
        raise ValueError("memory source mismatch")
    allocations = value.get("allocations")
    if not isinstance(allocations, dict):
        raise ValueError("allocations must be an object")
    if value.get("allocation_scope") != "publisher_and_consumer":
        raise ValueError("allocation scope mismatch")
    for key in (
        "count",
        "bytes",
        "current_count",
        "current_bytes",
        "peak_count",
        "peak_bytes",
    ):
        if exact_int(allocations.get(key), f"allocation {key}") != 0:
            raise ValueError(f"hot allocation {key} is nonzero")
    counters = validate_counters(value.get("counters"), warmup + samples)
    return {
        "latency_log2_ns": latency_buckets,
        "latency_min_ns": latency_min,
        "latency_max_ns": latency_max,
        "latency_jitter_span_ns": latency_max - latency_min,
        "operation_log2_ns": operation_buckets,
        "operation_max_ns": operation_max,
        "wall_time_ns": wall_time,
        "cpu_time_ns": cpu_time,
        "peer_cpu_time_ns": peer_cpu_time,
        "peak_resident_bytes": peak_resident,
        "peer_peak_resident_bytes": peer_peak_resident,
        "hot_allocation_count": 0,
        "counters": counters,
        "perf_stat": validate_perf(perf_path),
    }


def validate_project(
    value: dict[str, Any],
    *,
    backend: str,
    peer: str,
    kind: str,
    payload: int,
    perf_path: Path,
    args: argparse.Namespace,
) -> dict[str, Any]:
    if value.get("qualification") != "host_research":
        raise ValueError("standalone runner qualification label changed")
    if value.get("target_os") != "linux" or value.get("target_arch") != args.expected_arch:
        raise ValueError("project target identity mismatch")
    if value.get("backend") != backend or value.get("peer") != peer:
        raise ValueError("project candidate identity mismatch")
    expected_execution = (
        "separate_process_paired" if peer == "process" else "separate_thread_paired"
    )
    if value.get("peer_execution") != expected_execution:
        raise ValueError("project peer execution mismatch")
    if value.get("requested_memory_policy") != "lock-required":
        raise ValueError("project lock-required policy was not requested")
    if value.get("memory_policy_scope") != "shm_mappings":
        raise ValueError("project memory-policy scope mismatch")
    expected_effective = "not-applicable" if backend == "local" else "lock-required"
    if value.get("effective_memory_policy") != expected_effective:
        raise ValueError("project effective memory policy mismatch")
    peer_cpu = exact_int(value.get("peer_cpu_time_ns"), "peer_cpu_time_ns")
    if peer == "process" and peer_cpu == 0:
        raise ValueError("project process peer CPU time is missing")
    if peer == "thread" and peer_cpu != 0:
        raise ValueError("thread candidate unexpectedly reports peer process CPU time")
    peer_peak = exact_int(
        value.get("peer_peak_resident_bytes"), "peer_peak_resident_bytes"
    )
    if peer == "process" and peer_peak == 0:
        raise ValueError("project process peer peak resident memory is missing")
    if peer == "thread" and peer_peak != 0:
        raise ValueError("thread candidate unexpectedly reports peer process memory")
    return validate_common(
        value,
        kind=kind,
        payload=payload,
        capacity=args.capacity,
        warmup=args.warmup,
        samples=args.samples,
        perf_path=perf_path,
    )


def validate_horus(
    value: dict[str, Any], *, kind: str, payload: int, perf_path: Path, args: argparse.Namespace
) -> dict[str, Any]:
    if value.get("candidate") != "horus" or value.get("source_commit") != args.horus_commit:
        raise ValueError("Horus source identity mismatch")
    if value.get("target_os") != "linux" or value.get("target_arch") != args.expected_arch:
        raise ValueError("Horus target identity mismatch")
    if value.get("effective_memory_policy") != "lock-required":
        raise ValueError("Horus effective memory policy mismatch")
    if value.get("peer_execution") != "separate_process_paired":
        raise ValueError("Horus peer execution mismatch")
    return validate_common(
        value,
        kind=kind,
        payload=payload,
        capacity=args.capacity,
        warmup=args.warmup,
        samples=args.samples,
        perf_path=perf_path,
    )


def validate_faults(
    results: Path, expected_arch: str, horus_commit: str
) -> tuple[list[str], list[str]]:
    failures: list[str] = []
    incomplete: list[str] = []
    project_status = results / "project-faults.env"
    if not project_status.is_file():
        incomplete.append("project_fault_status_missing")
    elif project_status.read_text(encoding="ascii") != "EXIT_CODE=0\n":
        failures.append("project_fault_suite_failed")
    horus_path = results / "horus-faults.json"
    if not horus_path.is_file():
        incomplete.append("horus_fault_evidence_missing")
    else:
        try:
            value = read_json(horus_path)
            if (
                value.get("schema_version") != 1
                or value.get("candidate") != "horus"
                or value.get("source_commit") != horus_commit
                or value.get("target_arch") != expected_arch
            ):
                raise ValueError("invalid Horus fault schema")
            for key in (
                "producer_bounded",
                "no_torn_values",
                "crash_reclaim",
                "losses_accounted",
                "slow_consumer_accounted",
                "stopped_consumer_bounded",
                "full_queue_explicit",
                "clean_exit_reclaim",
                "killed_peer_reclaim",
            ):
                if value.get(key) is not True:
                    failures.append(f"horus_fault_{key}_failed")
        except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
            failures.append(f"horus_fault_evidence_invalid:{error}")
    return failures, incomplete


def evaluate(args: argparse.Namespace) -> tuple[int, dict[str, Any]]:
    failures, incomplete = validate_faults(
        args.results, args.expected_arch, args.horus_commit
    )
    observations: dict[str, dict[str, Any]] = {}

    for backend, peer in PROJECT_CANDIDATES:
        for kind in KINDS:
            for payload in PAYLOADS:
                candidate = f"{backend}-{peer}"
                path = args.results / f"project-{backend}-{peer}-{kind}-{payload}.json"
                perf_path = path.with_suffix(".perf")
                key = f"{candidate}/{kind}/{payload}"
                if not path.is_file():
                    incomplete.append(f"missing:{path.name}")
                    continue
                if not perf_path.is_file():
                    incomplete.append(f"missing:{perf_path.name}")
                    continue
                try:
                    observations[key] = validate_project(
                        read_json(path),
                        backend=backend,
                        peer=peer,
                        kind=kind,
                        payload=payload,
                        perf_path=perf_path,
                        args=args,
                    )
                except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
                    failures.append(f"invalid:{path.name}:{error}")

    for kind in KINDS:
        for payload in PAYLOADS:
            path = args.results / f"horus-{kind}-{payload}.json"
            perf_path = path.with_suffix(".perf")
            key = f"horus/{kind}/{payload}"
            if not path.is_file():
                incomplete.append(f"missing:{path.name}")
                continue
            if not perf_path.is_file():
                incomplete.append(f"missing:{perf_path.name}")
                continue
            try:
                observations[key] = validate_horus(
                    read_json(path),
                    kind=kind,
                    payload=payload,
                    perf_path=perf_path,
                    args=args,
                )
            except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
                failures.append(f"invalid:{path.name}:{error}")

    status = "pass"
    exit_code = 0
    if failures:
        status = "fail"
        exit_code = 1
    elif incomplete:
        status = "incomplete"
        exit_code = 2

    report = {
        "schema_version": 1,
        "qualification_scope": "middleware_candidate_matrix",
        "status": status,
        "selection_policy": "safety_gates_then_worst_case_maxima_no_average_ranking",
        "semantic_differences": {
            "local-thread": "separate in-process SPSC consumer thread; shared-memory locking is not applicable",
            "shm-thread": "separate in-process SPSC consumer thread over a project fixed-layout mapping with lock-required pages",
            "shm-process": "project fixed-layout cross-process SPSC with separately reported peer CPU time",
            "horus": "pinned external adapter; its native ownership/reclamation behavior is retained",
        },
        "config": {
            "capacity": args.capacity,
            "warmup": args.warmup,
            "samples": args.samples,
            "expected_arch": args.expected_arch,
            "horus_commit": args.horus_commit,
        },
        "failures": sorted(failures),
        "incomplete_reasons": sorted(incomplete),
        "observations": {key: observations[key] for key in sorted(observations)},
    }
    return exit_code, report


def main() -> int:
    args = parse_args()
    if args.output.exists():
        print(f"error: output already exists: {args.output}", file=sys.stderr)
        return 2
    if args.capacity < 3 or args.warmup < 0 or args.samples < 1:
        print("error: invalid bounded matrix settings", file=sys.stderr)
        return 2
    try:
        exit_code, report = evaluate(args)
        encoded = json.dumps(report, sort_keys=True, separators=(",", ":")) + "\n"
        with args.output.open("x", encoding="utf-8") as destination:
            destination.write(encoded)
        return exit_code
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
