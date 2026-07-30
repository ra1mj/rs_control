# Zenoh and ROS 2 Protocol Contract — Technical Design

## 1. Outcome and boundary

This task creates a backend-neutral contract that a later `ros2-adapter`
process can consume. The deliverable compiles and tests on the existing host
without ROS 2, DDS or Zenoh development packages.

```text
robot.v1 Protobuf bytes
        |
        v
protocol_module codec -> validated semantic values
        |
        v
integration/ros2-mapping -> validated route and projection contract
        |
        v                         later task only
generated ROS values <-> rcl client <-> rmw_cyclonedds_cpp
```

The integration crate may depend on `protocol_module`; the reverse dependency
is forbidden. It owns no hardware handle, local authority service, middleware
executor or realtime queue. `zenoh-bridge-ros2dds` remains an optional,
standalone DDS-island transport and never decodes the native Protobuf payload.

## 2. Compatibility profile

The first checked-in profile is fixed to:

| Component | Contract value |
|---|---|
| Board architecture | RK3588 / arm64 |
| OS | Ubuntu 22.04 |
| ROS distribution | Humble |
| ROS RMW | `rmw_cyclonedds_cpp` |
| Native payload | `robot.v1` Protobuf, `application/protobuf` |
| Optional DDS-island transport | standalone `zenoh-bridge-ros2dds` 1.9.0 |

The manifest carries these identities. A different ROS distribution or RMW is
a separate profile with its own interface sources, hashes and validation.
`rmw_zenoh` is not an alternate switch inside this profile: its native Zenoh
keys, attachments and discovery conventions are private to that RMW and are
incompatible with ros2dds wire keys.

## 3. Repository shape

```text
proto/robot/v1/
  common.proto                    # acquisition clock/domain, 3D primitives
  device.proto                    # IMU and time-aligned joint aggregation
  navigation.proto                # odometry and stamped transforms
  control.proto                   # complete action lifecycle and typed payloads
  health.proto                    # bounded health snapshot query DTOs

crates/protocol_module/
  message-model/                  # validated backend-neutral values
  contract-codec/                 # fallible Protobuf DTO conversion
  proto-generated/                # generated code and descriptor set

crates/integration/ros2-mapping/
  src/lib.rs                      # manifest model, validator, fingerprints

config/protocol/ros2-humble/
  routes.toml                     # allowlisted route/deployment profile
  interfaces/...                  # reviewed ROS source snapshots used for hashes
```

`config/robots/*.yaml` remains unchanged. Protocol routes and ROS deployment
policy do not belong in the minimal robot topology schema.

## 4. Canonical semantic amendments

### 4.1 Header time model

`Header` keeps its existing field numbers and gains an optional acquisition
timestamp at a new field number. The timestamp contains nanoseconds, an
explicit `UNIX` or `ROS` domain, and uncertainty. `source_monotonic_ns` remains
mandatory for local ordering/freshness. `publish_unix_ns` remains optional
correlation metadata.

The semantic model rejects an unspecified acquisition domain and never
converts monotonic time into a ROS header stamp. A mapping policy chooses one
of:

- require a trusted acquisition time within a maximum uncertainty;
- drop when no trusted acquisition time exists;
- use bridge reception time and mark the projection degraded.

ROS clock jumps can therefore change observation metadata but cannot extend a
lease or command deadline.

### 4.2 Numeric, covariance and frame values

The semantic layer owns checked finite scalars, vectors, unit quaternions,
3-by-3 and 6-by-6 covariances, and portable frame IDs. Validation rejects:

- NaN and infinity;
- quaternions outside the documented normalization tolerance;
- wrong covariance lengths, non-finite entries, asymmetric matrices or
  invalid diagonal values;
- empty, leading-slash, double-slash or nonportable frame IDs.

IMU orientation, angular velocity and linear acceleration are independently
optional. Their presence and covariance status map explicitly to ROS IMU
sentinels. Odometry contains parent/child frames, pose, twist and two 6-by-6
covariances. Transform samples distinguish static and dynamic authority. The
mapping validator rejects self edges, duplicate child ownership and cycles.

### 4.3 Action lifecycle

Existing v1 messages and enum values remain source/wire compatible. New
messages add registered fully-qualified goal, feedback and result payload type
identities, cancel request/decision and terminal result delivery.

The concrete first action is
`rs_control_interfaces/action/ExecuteChassisTrajectory`. Its bounded canonical
goal is a sequence of time-ordered chassis setpoints; feedback reports the
active point/progress; the terminal result reports completed points and a typed
outcome. The 16-byte canonical command ID is copied byte-for-byte to the ROS
goal UUID.

The pure lifecycle permits:

```text
submitted -> accepted -> executing -> terminal
        \-> rejected
executing -> cancel_requested -> canceling -> canceled
                               \-> executing (cancel rejected)
```

Terminal results have a nonzero bounded retention time and cache depth in the
manifest. Cancel acceptance is never treated as terminal completion.

### 4.4 Command authority

ROS ingress supplies only a requested command and observation stamp. The
adapter contract must capture local monotonic reception time, use an existing
bridge-owned lease and construct a locally bounded `CommandEnvelope` deadline.
No mapping API can mint, renew or extend a lease from a ROS timestamp. Pure
tests cover expired lease, excessive TTL, stale stamp, duplicate and prior-boot
rejection.

## 5. Route manifest

The TOML manifest is deny-by-default and parsed with unknown-field rejection.
It contains:

- schema/profile identity and pinned compatibility matrix;
- the SHA-256 fingerprint of the generated Protobuf descriptor set;
- native and ros2dds key prefixes plus DDS-island deployment ownership;
- a frame alias table and TF authority declarations;
- one entry per allowlisted direction and interaction;
- exact bounds, identity, QoS, time/frame and authorization policies.

Each route declares its ID, native key, direction, interaction class,
fully-qualified Protobuf type, maximum encoded bytes/rate/queue, freshness,
ROS name/type/distribution, interface source path and SHA-256, optional native
ROS type hash, explicit QoS, clock/frame policy and authorization class.

The first profile declares these projections:

| Direction | Native contract | ROS projection | Interaction |
|---|---|---|---|
| egress | `robot.v1.ImuState` | `sensor_msgs/msg/Imu` | sensor stream |
| egress | `robot.v1.JointStateAggregate` | `sensor_msgs/msg/JointState` | sensor stream |
| egress | `robot.v1.OdometryState` | `nav_msgs/msg/Odometry` | state stream |
| egress | `robot.v1.TransformBatch` | `/tf`, `/tf_static` | dynamic/static stream |
| ingress | `robot.v1.ChassisSetpoint` | `geometry_msgs/msg/TwistStamped` | guarded command |
| request/reply | `robot.v1.HealthSnapshotRequest/Response` | `rs_control_interfaces/srv/GetHealthSnapshot` | bounded query |
| bidirectional lifecycle | typed trajectory payloads | `rs_control_interfaces/action/ExecuteChassisTrajectory` | action |

A nominally bidirectional action is modeled as a fixed lifecycle contract, not
as an arbitrary bidirectional topic. Ordinary bidirectional stream mappings
still require two separately named routes and loop-suppression identities.

## 6. Validation model

Manifest loading performs all checks before returning a usable `RouteRegistry`:

1. schema/profile and descriptor fingerprint are supported;
2. route IDs, native keys and ROS endpoint ownership are unique;
3. prefixes do not overlap and direct DDS hosts do not also use ros2dds;
4. every route is bounded and has explicit direction, class, QoS and policy;
5. interaction class, direction, QoS and authorization are compatible;
6. Protobuf types exist in the descriptor catalog;
7. ROS source paths remain under the profile root and their hashes match;
8. frame aliases are portable, TF ownership is unique and acyclic;
9. command/action routes require local authority and monotonic deadline policy;
10. lossy standard mappings declare degraded/companion metadata behavior.

The registry has a deterministic canonical serialization and SHA-256
fingerprint for diagnostics and recordings. It exposes descriptors by route ID,
native key and ROS name but never auto-discovers additional endpoints.

## 7. Error and compatibility behavior

Errors are structured enums with the route/policy field that failed. Startup
collects deterministic validation failures where practical; payload conversion
fails at the first invalid semantic value. Unknown Protobuf enums and missing
required semantic fields are errors rather than default values.

All Protobuf edits are additive in `robot.v1`, formatted/linted by Buf and
checked against the committed descriptor baseline. The baseline is updated
only after the old schema is proven nonbreaking. ROS interface source changes
must update their digest and manifest in one reviewable change.

## 8. Runtime handoff and rollback

The later runtime adapter will consume `RouteRegistry` and semantic conversion
APIs. It must keep bounded worker queues between the native and ROS executors,
publish counters, and remain outside the realtime process.

This task is rollback-safe: removing the new integration crate/profile leaves
the native protocol path unchanged. If an additive field proves unsuitable
before release, reserve it and add a replacement field; never reinterpret an
already published v1 field. A runtime adapter must fail closed when its
manifest, descriptor or interface digests do not match.
