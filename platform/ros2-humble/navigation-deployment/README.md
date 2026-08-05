# Motor-disabled navigation deployment

This directory is a checked host-software profile for the crash-isolated
`chassis_nav` integration. It never enables motors or CAN. Install the three
executables and the external ROS packages first, copy `navigation.env.example`
to `/etc/rs-control/navigation.env`, replace only installation prefixes, and
keep the checked manifest digest unchanged.

The target starts the fake-output Rust harness, then the isolated bridge so
odometry/TF are available while the minimal Nav2 graph activates. The weak-host
service uses the Rust manifest validator once, then a small shell supervisor
executes the five native Nav2 binaries directly. It does not retain the Python
`ros2 launch` parent; the checked launch file remains available for development.
The bridge
retains at most one queued canonical goal behind a supervisor-owned readiness
file. A single short-lived readiness process queries the lifecycle manager's
aggregate state and action discovery; Nav2 writes the file only after
`/navigate_to_pose` is discoverable and all four managed lifecycle nodes are
active. The bridge then sends the goal without repeated per-node CLI probes.
A critical service loss stops the target and therefore all three services.
Restarts generate a new numeric process epoch, remove the old readiness file,
and let the SHM source guards retire old publisher boots.

Run `verify.sh` before installation. For a non-overwriting measurement-only
host snapshot, run:

```bash
collect-host-evidence.sh /absolute/new/evidence/directory
```

On a developer host without permission to create the systemd slice, pass `-`
and the supervisor/launch PIDs. Descendants are discovered before sampling:

```bash
collect-host-evidence.sh /absolute/new/evidence/directory - PID [PID ...]
```

The collector records cgroup memory/CPU/I/O counters plus per-process status,
scheduler, context-switch, fault and PSS/RSS observations when the kernel
exports them. It takes start/end process and cgroup snapshots one second apart
by default so rates can be derived; set
`RS_NAVIGATION_EVIDENCE_SAMPLE_SECONDS` to an integer from 1 through 60 for a
longer interval. It deliberately leaves AArch64 timing, physical motion,
production SLAM and whole-robot qualification pending.
