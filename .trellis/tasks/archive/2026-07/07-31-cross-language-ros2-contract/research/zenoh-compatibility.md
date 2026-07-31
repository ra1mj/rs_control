# Zenoh compatibility evidence

Research date: 2026-07-31

## Decision

Keep the repository's existing exact `1.9.0` family for:

- Rust `zenoh`;
- Python `eclipse-zenoh`;
- optional standalone `zenoh-bridge-ros2dds`.

Use the standalone ros2dds executable, not the dynamic plugin. Keep
`rmw_cyclonedds_cpp` as the Humble profile RMW and prevent a parallel direct
DDS path between DDS islands connected by ros2dds.

## Primary-source evidence

- Eclipse Zenoh's official 1.x Python tutorial uses
  `with zenoh.open(zenoh.Config())`, declared publishers/subscribers and the
  `eclipse-zenoh` package:
  <https://zenoh.io/docs/getting-started/first-app/>
- The official Zenoh repository lists 1.9.0 as the current release and
  identifies `zenoh-python` as the Python binding:
  <https://github.com/eclipse-zenoh/zenoh>
- The official ros2dds repository lists 1.9.0 as the current release, provides
  a standalone `zenoh-bridge-ros2dds` artifact, recommends CycloneDDS, and
  warns that direct DDS communication alongside the bridge can create
  duplicates or loops:
  <https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds>

## Repository evidence

- Workspace Rust Zenoh pin: `Cargo.toml`, `zenoh = "=1.9.0"`.
- Existing Humble ros2dds pin:
  `config/protocol/ros2-humble/routes.toml`,
  `ros2dds_version = "1.9.0"`.
- Vendored protoc is `libprotoc 31.1`; generated Python modules therefore
  require a compatible modern Protobuf runtime.

This is a compatibility pin, not permission to silently upgrade one component.
A future version changes all affected pins and reruns cross-language plus ROS
profile gates together.
