#!/bin/sh
set -eu

arm_sim_repo_root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$arm_sim_repo_root"

arm_sim_evidence="evidence/arm-adaptive-gravity-simulation.json"

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    echo "usage: tools/run-arm-simulation.sh {qualify [output]|verify|test}" >&2
    exit 2
fi

case "$1" in
    qualify)
        arm_sim_output=${2:-$arm_sim_evidence}
        if [ -e "$arm_sim_output" ]; then
            # Existing evidence is immutable from the operator wrapper. A
            # qualification against the checked-in path becomes a read-only
            # verification instead of truncating or replacing the file.
            cargo run --locked --release -p rs-arm-simulation -- \
                verify "$arm_sim_output"
        else
            cargo run --locked --release -p rs-arm-simulation -- \
                qualify --output "$arm_sim_output"
            echo "wrote $arm_sim_output"
        fi
        ;;
    verify)
        if [ "$#" -ne 1 ]; then
            echo "usage: tools/run-arm-simulation.sh {qualify [output]|verify|test}" >&2
            exit 2
        fi
        cargo run --locked --release -p rs-arm-simulation -- \
            verify "$arm_sim_evidence"
        ;;
    test)
        if [ "$#" -ne 1 ]; then
            echo "usage: tools/run-arm-simulation.sh {qualify [output]|verify|test}" >&2
            exit 2
        fi
        cargo test --locked -p arm-algorithms --test dynamics
        cargo test --locked -p arm-algorithms --test adaptive_gravity
        cargo test --locked -p arm-algorithms --test gravity_prepared
        cargo test --locked -p arm-algorithms --test kinematics
        cargo test --locked -p arm-control --test control
        cargo test --locked -p rs-arm-simulation --test cli -- --test-threads=1
        cargo test --locked -p rs-arm-simulation --test qualification -- --test-threads=1
        ;;
    *)
        echo "unknown command: $1" >&2
        echo "usage: tools/run-arm-simulation.sh {qualify [output]|verify|test}" >&2
        exit 2
        ;;
esac
