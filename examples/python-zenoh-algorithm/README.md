# Python Zenoh heading-hold example

This is an intentionally independent, non-realtime algorithm process. It
consumes generated `robot.v1.OdometryState` values over Zenoh and publishes
generated `robot.v1.ChassisSetpointIntent` values. It has no ROS, driver or
Rust binding dependency.

The output is only a remote command intent. A local Rust receiver must still
authorize the publisher boot, reject duplicate/reordered samples, match the
asserted authority tuple against an existing lease, and construct a bounded
local-monotonic command deadline.

## Prepare

```bash
uv sync --project examples/python-zenoh-algorithm --frozen
```

Generate Python clients into a disposable directory using the repository's
pinned protoc. The `cross-language-contract` test does this automatically and
sets `PYTHONPATH` to that directory.

## Run

With a Zenoh router listening on `tcp/127.0.0.1:7447` and generated modules on
`PYTHONPATH`:

```bash
uv run --project examples/python-zenoh-algorithm --frozen \
  examples/python-zenoh-algorithm/heading_hold.py \
  --connect tcp/127.0.0.1:7447 \
  --input-key robot/demo/v1/navigation/odometry/state \
  --output-key robot/demo/v1/control/chassis/command_intent \
  --owner-id example_operator \
  --lease-id 07070707070707070707070707070707 \
  --session-epoch 1 \
  --requested-ttl-ns 20000000
```

The normal executable creates a new boot ID at every start. `--boot-id` exists
only so qualification tests can pre-authorize deterministic process identities.

This example uses no camera, Orange Pi or physical hardware. Its loopback test
is host software evidence and does not qualify target realtime behavior.
