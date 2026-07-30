"""Shared constants and strict readers for middleware qualification evidence."""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any, Collection


KINDS = ("latest", "ordered")
PAYLOADS = (64, 256, 1024)
CAPACITIES = frozenset((16, 32, 64, 128, 256, 512, 1024))
PINNED_HORUS_COMMIT = "fac3cfb8704e275bde9daec6d554a17c59dd8532"
DATA_KEY = re.compile(r"^[A-Z][A-Z0-9_]*$")
DATA_VALUE = re.compile(r"^[A-Za-z0-9._,:/@+-]+$")


def is_comparison_capacity(value: int) -> bool:
    return type(value) is int and value in CAPACITIES


def read_json_object(path: Path) -> dict[str, Any]:
    with path.open("rb") as source:
        value = json.load(source)
    if not isinstance(value, dict):
        raise ValueError("root must be an object")
    return value


def require_exact(value: Any, expected: Any, name: str) -> None:
    if type(value) is not type(expected) or value != expected:
        raise ValueError(f"{name} mismatch")


def read_data_file(
    path: Path, *, expected_keys: Collection[str] | None = None
) -> dict[str, str]:
    values: dict[str, str] = {}
    for line_number, line in enumerate(path.read_text(encoding="ascii").splitlines(), 1):
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if line != line.strip() or line.count("=") != 1:
            raise ValueError(f"invalid data line {line_number}: {path}")
        key, value = line.split("=", 1)
        if (
            not DATA_KEY.fullmatch(key)
            or DATA_VALUE.fullmatch(value) is None
        ):
            raise ValueError(f"invalid data token on line {line_number}: {path}")
        if key in values:
            raise ValueError(f"duplicate data key {key}: {path}")
        values[key] = value
    if expected_keys is not None:
        expected = set(expected_keys)
        if values.keys() != expected:
            missing = sorted(expected - values.keys())
            extra = sorted(values.keys() - expected)
            raise ValueError(
                f"data keys mismatch in {path}: missing={missing}, extra={extra}"
            )
    return values
