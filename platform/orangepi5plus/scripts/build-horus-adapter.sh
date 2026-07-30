#!/bin/sh
set -eu

adapter_script_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
adapter_platform_root=$(CDPATH= cd -- "$adapter_script_dir/.." && pwd)
adapter_source_root=$adapter_platform_root/horus-adapter
. "$adapter_platform_root/lib/common.sh"

adapter_usage() {
    printf '%s\n' \
        'Usage: build-horus-adapter.sh --horus-source ABSOLUTE_DIR --output ABSOLUTE_FILE' \
        '       build-horus-adapter.sh --verify-only' \
        '' \
        'Builds only from the clean, pinned Horus checkout with Cargo offline/locked.'
}

adapter_horus_source=
adapter_output=
adapter_verify_only=0
while [ "$#" -gt 0 ]; do
    case "$1" in
        --horus-source|--output)
            [ "$#" -ge 2 ] || platform_die "$1 requires a value"
            adapter_option=$1
            adapter_value=$2
            shift
            case "$adapter_option" in
                --horus-source) adapter_horus_source=$adapter_value ;;
                --output) adapter_output=$adapter_value ;;
            esac
            ;;
        --verify-only) adapter_verify_only=1 ;;
        -h|--help) adapter_usage; exit 0 ;;
        *) platform_die "unknown argument: $1" ;;
    esac
    shift
done

for adapter_file in Cargo.toml.in Cargo.lock src/main.rs; do
    [ -r "$adapter_source_root/$adapter_file" ] || platform_die "missing adapter source: $adapter_file"
done
adapter_commit=$(awk -F= '$1 == "HORUS_COMMIT" { value=$2; count++ } END { if (count != 1 || length(value) != 40 || value !~ /^[0-9a-f]+$/) exit 1; print value }' "$adapter_platform_root/sources.lock") ||
    platform_die 'invalid HORUS_COMMIT source lock'

if [ "$adapter_verify_only" -eq 1 ]; then
    [ -z "$adapter_horus_source" ] || platform_die '--verify-only does not accept --horus-source'
    [ -z "$adapter_output" ] || platform_die '--verify-only does not accept --output'
    platform_require_command rustfmt
    [ "$(grep -F -c 'path = "@HORUS_CORE_PATH@"' "$adapter_source_root/Cargo.toml.in")" -eq 1 ] ||
        platform_die 'adapter manifest must contain one external Horus path placeholder'
    grep -F 'name = "rs-control-horus-qualification-adapter"' "$adapter_source_root/Cargo.lock" >/dev/null ||
        platform_die 'adapter lock is missing the adapter package'
    HORUS_SOURCE_COMMIT=$adapter_commit rustfmt --edition 2021 --check "$adapter_source_root/src/main.rs"
    printf 'adapter_source=verified\nhorus_commit=%s\nbuild_mode=offline-locked-external\n' "$adapter_commit"
    exit 0
fi

[ -n "$adapter_horus_source" ] || platform_die '--horus-source is required'
[ -n "$adapter_output" ] || platform_die '--output is required'
platform_require_absolute_path '--horus-source' "$adapter_horus_source"
platform_require_absolute_path '--output' "$adapter_output"
[ -d "$adapter_horus_source" ] || platform_die 'Horus source is not a directory'
[ ! -e "$adapter_output" ] || platform_die "adapter output must be absent: $adapter_output"
adapter_output_parent=$(dirname -- "$adapter_output")
[ -d "$adapter_output_parent" ] || platform_die 'adapter output parent is not a directory'
for adapter_command in cargo git ln mktemp python3; do platform_require_command "$adapter_command"; done

git -C "$adapter_horus_source" rev-parse --is-inside-work-tree >/dev/null 2>&1 ||
    platform_die 'Horus source is not a git worktree'
adapter_actual_commit=$(git -C "$adapter_horus_source" rev-parse HEAD)
[ "$adapter_actual_commit" = "$adapter_commit" ] ||
    platform_die "Horus commit mismatch: $adapter_actual_commit"
[ -z "$(git -C "$adapter_horus_source" status --porcelain)" ] ||
    platform_die 'Horus checkout must be clean'
[ -r "$adapter_horus_source/horus_core/Cargo.toml" ] ||
    platform_die 'pinned Horus checkout is missing horus_core'

adapter_build_root=$(mktemp -d "$adapter_output_parent/.rs-control-horus-adapter-build.XXXXXX")
adapter_cleanup() {
    case "$adapter_build_root" in
        "$adapter_output_parent"/.rs-control-horus-adapter-build.*) rm -rf -- "$adapter_build_root" ;;
        *) platform_die "refusing to remove unexpected build path: $adapter_build_root" ;;
    esac
}
trap adapter_cleanup EXIT INT TERM HUP

mkdir "$adapter_build_root/src"
cp "$adapter_source_root/Cargo.lock" "$adapter_build_root/Cargo.lock"
cp "$adapter_source_root/src/main.rs" "$adapter_build_root/src/main.rs"
python3 -B - "$adapter_source_root/Cargo.toml.in" "$adapter_build_root/Cargo.toml" \
    "$adapter_horus_source/horus_core" <<'PY'
import json
import pathlib
import sys

source = pathlib.Path(sys.argv[1]).read_text(encoding="utf-8")
placeholder = '"@HORUS_CORE_PATH@"'
if source.count(placeholder) != 1:
    raise SystemExit("adapter manifest must contain exactly one Horus path placeholder")
rendered = source.replace(placeholder, json.dumps(sys.argv[3]))
pathlib.Path(sys.argv[2]).write_text(rendered, encoding="utf-8")
PY

CARGO_TARGET_DIR=$adapter_build_root/target \
HORUS_SOURCE_COMMIT=$adapter_commit \
    cargo build --offline --locked --release --manifest-path "$adapter_build_root/Cargo.toml"
adapter_binary=$adapter_build_root/target/release/rs-control-horus-qualification-adapter
[ -x "$adapter_binary" ] || platform_die 'Cargo did not produce the adapter binary'
ln "$adapter_binary" "$adapter_output" || platform_die 'adapter output appeared during build'
printf 'adapter=%s\nhorus_commit=%s\nbuild_mode=offline-locked-external\n' \
    "$adapter_output" "$adapter_commit"
