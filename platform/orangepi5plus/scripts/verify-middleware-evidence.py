#!/usr/bin/env python3
"""Independently verify a sealed RK3588 middleware evidence bundle."""

from __future__ import annotations

import argparse
import hashlib
import re
import subprocess
import sys
import tempfile
from pathlib import Path

from middleware_evidence import (
    PINNED_HORUS_COMMIT,
    is_comparison_capacity,
    read_data_file,
    read_json_object,
    require_exact,
)


SCRIPT_DIR = Path(__file__).resolve().parent
SHA256_LINE = re.compile(r"^([0-9a-f]{64})  (\./[^\r\n]+)$")
HEX_40 = re.compile(r"^[0-9a-f]{40}$")
HEX_64 = re.compile(r"^[0-9a-f]{64}$")
DATA_TOKEN = re.compile(r"^[A-Za-z0-9._:+-]+$")
INPUT_KEYS = frozenset(
    (
        "FORMAT_VERSION",
        "PROFILE",
        "MODE",
        "DURATION",
        "APP_COMMIT",
        "APP_BUILD_ID",
        "MEASUREMENT_CPU",
        "RT_PRIORITY",
        "CAPACITY",
        "WARMUP",
        "SAMPLES",
        "HORUS_COMMIT",
        "RUNNER_SHA256",
        "HORUS_RUNNER_SHA256",
        "SOURCES_LOCK_SHA256",
        "RT_PROFILE_SHA256",
    )
)
VERDICT_KEYS = frozenset(
    (
        "QUALIFICATION_STATUS",
        "QUALIFICATION_REASON",
        "QUALIFICATION_SCOPE",
        "PLATFORM_STATUS",
        "PLATFORM_EXIT_CODE",
        "EVALUATOR_EXIT_CODE",
        "MATRIX_EXECUTION_FAILED",
        "PLATFORM_ENVELOPE_LOST",
        "PROFILE_MUTATED",
    )
)
HARDWARE_KEYS = frozenset(
    (
        "HARDWARE_EVIDENCE_FORMAT_VERSION",
        "BOARD_MODEL",
        "BOARD_REVISION",
        "POWER_SUPPLY",
        "ACTIVE_COOLER",
        "CAN_TRANSCEIVER",
        "CAN_ISOLATION_RATING",
        "CAN_TERMINATION",
        "SERIAL_CONSOLE_VERIFIED",
        "VENDOR_KERNEL_FALLBACK",
    )
)
STATUS_EXIT = {"pass": 0, "fail": 1, "incomplete": 2}


class EvidenceError(ValueError):
    """A sealed bundle is malformed, inconsistent or has changed."""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--evidence", type=Path, required=True)
    parser.add_argument("--expected-app-commit", required=True)
    parser.add_argument("--expected-sha256")
    return parser.parse_args()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def bundle_files(base: Path, manifest: Path) -> list[str]:
    files: list[str] = []
    for path in base.rglob("*"):
        if path.is_symlink():
            raise EvidenceError(f"symlink is forbidden in evidence: {path}")
        if path.is_dir():
            continue
        if not path.is_file():
            raise EvidenceError(f"special file is forbidden in evidence: {path}")
        if path != manifest:
            files.append(f"./{path.relative_to(base).as_posix()}")
    return sorted(files)


def verify_manifest(base: Path) -> int:
    manifest = base / "SHA256SUMS"
    if not manifest.is_file() or manifest.is_symlink():
        raise EvidenceError(f"checksum manifest missing: {manifest}")
    entries: list[tuple[str, str]] = []
    for line_number, line in enumerate(
        manifest.read_text(encoding="ascii").splitlines(), 1
    ):
        match = SHA256_LINE.fullmatch(line)
        if match is None:
            raise EvidenceError(f"invalid checksum line {line_number}: {manifest}")
        entries.append((match.group(2), match.group(1)))
    paths = [path for path, _digest in entries]
    expected_paths = bundle_files(base, manifest)
    if paths != expected_paths:
        raise EvidenceError(f"checksum file set/order mismatch: {manifest}")
    for relative, expected_digest in entries:
        actual_digest = sha256_file(base / relative.removeprefix("./"))
        if actual_digest != expected_digest:
            raise EvidenceError(f"checksum mismatch: {base / relative}")
    return len(entries)


def exact_uint(value: str, name: str, *, minimum: int = 0) -> int:
    if not value.isascii() or not value.isdecimal():
        raise EvidenceError(f"{name} must be an unsigned decimal integer")
    parsed = int(value)
    if parsed < minimum or str(parsed) != value:
        raise EvidenceError(f"{name} must be canonical and >= {minimum}")
    return parsed


def duration_seconds(value: str) -> int:
    match = re.fullmatch(r"([1-9][0-9]*)([smhd]?)", value)
    if match is None:
        raise EvidenceError("DURATION is invalid")
    multiplier = {"": 1, "s": 1, "m": 60, "h": 3600, "d": 86400}[match.group(2)]
    return int(match.group(1)) * multiplier


def run_checked(command: list[str], name: str, expected_codes: set[int]) -> int:
    completed = subprocess.run(command, check=False, capture_output=True, text=True)
    if completed.returncode not in expected_codes:
        detail = completed.stderr.strip() or completed.stdout.strip() or "no diagnostic"
        raise EvidenceError(f"{name} failed: {detail}")
    return completed.returncode


def expected_wrapper_status(
    *,
    matrix_failed: int,
    evaluator_exit: int,
    platform_status: str,
    platform_exit: int,
    platform_lost: int,
) -> str:
    if matrix_failed != 0 or evaluator_exit == 1 or platform_status == "fail":
        return "fail"
    if (
        evaluator_exit != 0
        or platform_exit != 0
        or platform_status != "pass"
        or platform_lost != 0
    ):
        return "incomplete"
    return "pass"


def verify(evidence: Path, expected_app_commit: str) -> tuple[str, str, int]:
    if not evidence.is_absolute() or not evidence.is_dir() or evidence.is_symlink():
        raise EvidenceError("--evidence must be an absolute, real directory")
    manifests = sorted(
        evidence.rglob("SHA256SUMS"),
        key=lambda path: len(path.relative_to(evidence).parts),
        reverse=True,
    )
    if evidence / "SHA256SUMS" not in manifests:
        raise EvidenceError("top-level checksum manifest is missing")
    file_count = 0
    for manifest in manifests:
        verified_count = verify_manifest(manifest.parent)
        if manifest.parent == evidence:
            file_count = verified_count
    platform_root = evidence / "platform"

    inputs = read_data_file(evidence / "inputs.env", expected_keys=INPUT_KEYS)
    verdict = read_data_file(evidence / "verdict.env", expected_keys=VERDICT_KEYS)
    sources = read_data_file(evidence / "sources.lock")
    reviewed_sources_path = SCRIPT_DIR.parent / "sources.lock"
    reviewed_sources = read_data_file(reviewed_sources_path)
    hardware = read_data_file(
        evidence / "hardware-evidence.env", expected_keys=HARDWARE_KEYS
    )
    if inputs["FORMAT_VERSION"] != "1":
        raise EvidenceError("unsupported inputs format")
    if inputs["PROFILE"] not in ("1khz", "500hz"):
        raise EvidenceError("invalid profile")
    if inputs["MODE"] not in ("baseline", "concurrent"):
        raise EvidenceError("invalid mode")
    if hardware["HARDWARE_EVIDENCE_FORMAT_VERSION"] != "1":
        raise EvidenceError("unsupported hardware evidence format")
    if hardware["BOARD_MODEL"] != "OrangePi5Plus":
        raise EvidenceError("hardware evidence board mismatch")
    if hardware["SERIAL_CONSOLE_VERIFIED"] != "1":
        raise EvidenceError("serial console recovery was not verified")
    if "unrecorded" in hardware.values():
        raise EvidenceError("hardware evidence contains unrecorded values")
    seconds = duration_seconds(inputs["DURATION"])
    if HEX_40.fullmatch(inputs["APP_COMMIT"]) is None:
        raise EvidenceError("invalid app commit")
    if inputs["APP_COMMIT"] != expected_app_commit:
        raise EvidenceError("caller-owned app commit mismatch")
    if DATA_TOKEN.fullmatch(inputs["APP_BUILD_ID"]) is None:
        raise EvidenceError("invalid app build ID")
    exact_uint(inputs["MEASUREMENT_CPU"], "MEASUREMENT_CPU")
    priority = exact_uint(inputs["RT_PRIORITY"], "RT_PRIORITY", minimum=1)
    if priority > 89:
        raise EvidenceError("RT_PRIORITY exceeds 89")
    capacity = exact_uint(inputs["CAPACITY"], "CAPACITY")
    warmup = exact_uint(inputs["WARMUP"], "WARMUP")
    samples = exact_uint(inputs["SAMPLES"], "SAMPLES", minimum=1)
    if not is_comparison_capacity(capacity):
        raise EvidenceError("invalid comparison capacity")
    if warmup > 1_000_000 or samples > 10_000_000:
        raise EvidenceError("bounded matrix setting exceeds limit")
    for key in (
        "RUNNER_SHA256",
        "HORUS_RUNNER_SHA256",
        "SOURCES_LOCK_SHA256",
        "RT_PROFILE_SHA256",
    ):
        if HEX_64.fullmatch(inputs[key]) is None:
            raise EvidenceError(f"invalid digest: {key}")
    if sources.get("LOCK_FORMAT_VERSION") != "1":
        raise EvidenceError("unsupported source lock format")
    if sources.get("HORUS_COMMIT") != inputs["HORUS_COMMIT"]:
        raise EvidenceError("Horus source lock mismatch")
    if inputs["HORUS_COMMIT"] != PINNED_HORUS_COMMIT:
        raise EvidenceError("Horus commit is not the pinned comparison source")
    if reviewed_sources.get("HORUS_COMMIT") != PINNED_HORUS_COMMIT:
        raise EvidenceError("reviewed verifier source lock has unexpected Horus commit")
    evidence_sources_digest = sha256_file(evidence / "sources.lock")
    if evidence_sources_digest != inputs["SOURCES_LOCK_SHA256"]:
        raise EvidenceError("sources.lock identity mismatch")
    if evidence_sources_digest != sha256_file(reviewed_sources_path):
        raise EvidenceError("bundle source lock differs from reviewed verifier checkout")
    if sha256_file(evidence / "rt-profile.env") != inputs["RT_PROFILE_SHA256"]:
        raise EvidenceError("rt-profile.env identity mismatch")

    run_checked(
        [
            sys.executable,
            "-B",
            str(SCRIPT_DIR / "validate-horus-preflight.py"),
            "--input",
            str(evidence / "horus-preflight.json"),
            "--source-commit",
            inputs["HORUS_COMMIT"],
            "--capacity",
            str(capacity),
            "--expected-arch",
            "aarch64",
        ],
        "Horus preflight validation",
        {0},
    )

    comparison_path = evidence / "comparison.json"
    comparison = read_json_object(comparison_path)
    require_exact(comparison.get("schema_version"), 1, "comparison schema_version")
    comparison_status = comparison.get("status")
    if not isinstance(comparison_status, str) or comparison_status not in STATUS_EXIT:
        raise EvidenceError("comparison status is invalid")
    config = comparison.get("config")
    if not isinstance(config, dict):
        raise EvidenceError("comparison config must be an object")
    expected_config = {
        "capacity": capacity,
        "warmup": warmup,
        "samples": samples,
        "expected_arch": "aarch64",
        "horus_commit": inputs["HORUS_COMMIT"],
    }
    for key, expected in expected_config.items():
        require_exact(config.get(key), expected, f"comparison config {key}")

    with tempfile.TemporaryDirectory(prefix="rs-control-middleware-verify.") as temp:
        regenerated = Path(temp) / "comparison.json"
        evaluator_exit = run_checked(
            [
                sys.executable,
                "-B",
                str(SCRIPT_DIR / "evaluate-middleware.py"),
                "--results",
                str(evidence / "results"),
                "--output",
                str(regenerated),
                "--capacity",
                str(capacity),
                "--warmup",
                str(warmup),
                "--samples",
                str(samples),
                "--expected-arch",
                "aarch64",
                "--horus-commit",
                inputs["HORUS_COMMIT"],
            ],
            "middleware matrix re-evaluation",
            {0, 1, 2},
        )
        if regenerated.read_bytes() != comparison_path.read_bytes():
            raise EvidenceError("comparison.json differs from raw-result re-evaluation")

    if evaluator_exit != STATUS_EXIT[comparison_status]:
        raise EvidenceError("comparison status/exit mismatch")
    if (
        exact_uint(verdict["EVALUATOR_EXIT_CODE"], "EVALUATOR_EXIT_CODE")
        != evaluator_exit
    ):
        raise EvidenceError("retained evaluator exit mismatch")
    matrix_failed = exact_uint(
        verdict["MATRIX_EXECUTION_FAILED"], "MATRIX_EXECUTION_FAILED"
    )
    platform_lost = exact_uint(
        verdict["PLATFORM_ENVELOPE_LOST"], "PLATFORM_ENVELOPE_LOST"
    )
    if matrix_failed not in (0, 1) or platform_lost not in (0, 1):
        raise EvidenceError("wrapper boolean token is invalid")
    platform_exit = exact_uint(verdict["PLATFORM_EXIT_CODE"], "PLATFORM_EXIT_CODE")
    platform_status = verdict["PLATFORM_STATUS"]
    if platform_status not in STATUS_EXIT:
        raise EvidenceError("platform status is invalid")
    platform_verdict_path = platform_root / "verdict.env"
    if platform_verdict_path.is_file():
        platform_verdict = read_data_file(platform_verdict_path)
        required_platform = {
            "QUALIFICATION_STATUS": platform_status,
            "QUALIFICATION_SCOPE": "kernel_platform",
            "PROFILE": inputs["PROFILE"],
            "MODE": inputs["MODE"],
            "DURATION_SECONDS": str(seconds),
        }
        for key, expected in required_platform.items():
            if platform_verdict.get(key) != expected:
                raise EvidenceError(f"platform verdict {key} mismatch")
    elif platform_status != "incomplete":
        raise EvidenceError("non-incomplete platform verdict is missing")

    status = verdict["QUALIFICATION_STATUS"]
    expected_status = expected_wrapper_status(
        matrix_failed=matrix_failed,
        evaluator_exit=evaluator_exit,
        platform_status=platform_status,
        platform_exit=platform_exit,
        platform_lost=platform_lost,
    )
    if status != expected_status:
        raise EvidenceError("wrapper qualification status mismatch")
    expected_reason = {
        "pass": "all_safety_gates_passed",
        "fail": "safety_or_execution_gate_failed",
        "incomplete": "platform_or_candidate_evidence_incomplete",
    }[status]
    if verdict["QUALIFICATION_REASON"] != expected_reason:
        raise EvidenceError("wrapper qualification reason mismatch")
    if verdict["QUALIFICATION_SCOPE"] != "rk3588_middleware_candidates":
        raise EvidenceError("wrapper qualification scope mismatch")
    if verdict["PROFILE_MUTATED"] != "0":
        raise EvidenceError("qualification claims profile mutation")

    verify_manifest(evidence)
    evidence_digest = sha256_file(evidence / "SHA256SUMS")
    return status, evidence_digest, file_count


def main() -> int:
    args = parse_args()
    try:
        status, digest, file_count = verify(args.evidence, args.expected_app_commit)
        if args.expected_sha256 is not None:
            if HEX_64.fullmatch(args.expected_sha256) is None:
                raise EvidenceError("--expected-sha256 must be 64 lowercase hex digits")
            if args.expected_sha256 != digest:
                raise EvidenceError("caller-owned evidence digest mismatch")
    except (EvidenceError, OSError, UnicodeError, ValueError) as error:
        print(f"error: invalid middleware evidence: {error}", file=sys.stderr)
        return 1
    print(
        f"verified qualification_status={status} "
        f"evidence_sha256={digest} files={file_count}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
