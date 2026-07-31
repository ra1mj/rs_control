#!/usr/bin/env bash
set -euo pipefail

tool_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "${tool_dir}/../.." && pwd)"

cd "${repo_dir}"
cargo build --locked --release -p hsm-spike --bins

direct_bin="${repo_dir}/target/release/direct-enum"
statig_bin="${repo_dir}/target/release/statig"

printf 'measurement_scope=host_research\n'
rustc -Vv
printf 'direct_checksum=%s\n' "$("${direct_bin}")"
printf 'statig_checksum=%s\n' "$("${statig_bin}")"
printf 'direct_file_bytes=%s\n' "$(wc -c < "${direct_bin}")"
printf 'statig_file_bytes=%s\n' "$(wc -c < "${statig_bin}")"
printf 'direct_source_lines=%s\n' "$(wc -l < "${tool_dir}/src/direct.rs")"
printf 'statig_source_lines=%s\n' "$(wc -l < "${tool_dir}/src/statig_candidate.rs")"
size "${direct_bin}" "${statig_bin}"
cargo tree --locked -p hsm-spike -e features
