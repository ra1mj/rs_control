#!/bin/sh
set -eu

ci_repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$ci_repo_root"

platform/orangepi5plus/tests/host-test.sh
platform/orangepi5plus/tests/middleware-host-test.sh

cargo fmt --all -- --check
cargo clippy --locked --workspace --all-targets --all-features -- -D warnings
cargo test --locked --workspace --all-features
cargo run --locked -p architecture-check -- check
cargo run --locked -p proto-compat-check

tools/bufw format --diff --exit-code
tools/bufw lint
tools/bufw breaking --against proto/golden/robot-v1.binpb

cargo deny check

for ci_config in config/robots/*.yaml; do
    cargo run --locked -p robotctl -- config validate "$ci_config"
done
