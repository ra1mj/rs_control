//! Backend-neutral, deny-by-default ROS 2 route contract.
//!
//! This crate validates checked-in mapping metadata. It intentionally has no
//! ROS, DDS or Zenoh runtime dependency.

mod projection;
mod runtime;

pub use projection::*;
pub use runtime::*;

use std::{
    collections::{BTreeMap, BTreeSet},
    fmt::Write,
    fs, io,
    path::{Component, Path, PathBuf},
};

use message_model::AsciiId;
use prost::Message;
use prost_types::{DescriptorProto, FileDescriptorSet};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use thiserror::Error;

/// Supported route-manifest schema version.
pub const MANIFEST_SCHEMA_VERSION: u32 = 1;
/// Native sample encoding required by the canonical protocol.
pub const PROTOBUF_ENCODING: &str = "application/protobuf";
/// Exact Zenoh family qualified by the first compatibility profile.
pub const SUPPORTED_ZENOH_VERSION: &str = "1.9.0";
/// Exact standalone ros2dds release qualified by the first profile.
pub const SUPPORTED_ROS2DDS_VERSION: &str = "1.9.0";
/// Expected standalone executable build identity.
pub const SUPPORTED_ROS2DDS_BUILD_ID: &str = "eclipse-zenoh/zenoh-bridge-ros2dds@1.9.0";

/// Complete checked-in mapping manifest.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Manifest {
    /// Manifest schema version.
    pub schema_version: u32,
    /// Pinned compatibility identity.
    pub profile: Profile,
    /// Network path ownership and key prefixes.
    pub deployment: Deployment,
    /// Portable canonical-to-ROS frame aliases.
    #[serde(default)]
    pub frame_alias: Vec<FrameAlias>,
    /// Explicit TF child authorities.
    #[serde(default)]
    pub tf_authority: Vec<TfAuthority>,
    /// Allowlisted routes.
    #[serde(default)]
    pub route: Vec<Route>,
}

/// Pinned compatibility matrix for one mapping profile.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Profile {
    /// Stable profile identifier.
    pub id: String,
    /// Target CPU architecture.
    pub architecture: String,
    /// Target operating system.
    pub operating_system: String,
    /// ROS distribution.
    pub ros_distribution: String,
    /// Selected ROS middleware implementation.
    pub rmw: String,
    /// Native sample encoding.
    pub native_encoding: String,
    /// SHA-256 of the complete robot.v1 descriptor set.
    pub protobuf_descriptor_sha256: String,
    /// Reviewed ROS interface source revision.
    pub ros_interface_revision: String,
    /// Zenoh compatibility family shared by native clients and the bridge.
    pub zenoh_version: String,
    /// Optional standalone ros2dds executable version.
    pub ros2dds_version: String,
    /// Exact standalone ros2dds build identity.
    pub ros2dds_build_id: String,
    /// Standalone/plugin deployment selection.
    pub ros2dds_mode: Ros2DdsMode,
    /// Adapter executor is outside the qualified realtime process.
    pub adapter_outside_realtime: bool,
    /// Must remain false: the adapter cannot mint authority.
    pub adapter_can_issue_leases: bool,
}

/// Supported ros2dds deployment shape.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Ros2DdsMode {
    /// Crash-isolated standalone executable.
    Standalone,
    /// Dynamic plugin loaded into a Zenoh router; rejected by this profile.
    DynamicPlugin,
}

/// Deployment ownership needed to prevent loops and duplicate DDS paths.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Deployment {
    /// Prefix owned by native Protobuf routes.
    pub native_key_prefix: String,
    /// Disjoint prefix reserved for ros2dds transport internals.
    pub ros2dds_key_prefix: String,
    /// DDS islands reached directly.
    #[serde(default)]
    pub direct_dds_hosts: Vec<String>,
    /// DDS islands reached through standalone ros2dds.
    #[serde(default)]
    pub ros2dds_hosts: Vec<String>,
}

/// One validated boundary-only frame alias.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct FrameAlias {
    /// Slash-free canonical frame.
    pub canonical: String,
    /// Explicit ROS namespace/frame value without a leading slash.
    pub ros: String,
}

/// Static/dynamic transform authority.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct TfAuthority {
    /// Parent canonical frame.
    pub parent: String,
    /// Child canonical frame with exactly one owner.
    pub child: String,
    /// Static or dynamic authority.
    pub kind: TfKind,
    /// Route ID that owns publication.
    pub route_id: String,
}

/// TF authority class.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TfKind {
    /// Volatile changing transform.
    Dynamic,
    /// Transient-local fixed transform.
    Static,
}

/// One allowlisted mapping route.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Route {
    /// Stable route identity.
    pub id: String,
    /// Exact native Zenoh key, never inferred from ROS name.
    pub native_key: String,
    /// One explicit direction or fixed action lifecycle.
    pub direction: Direction,
    /// Semantic interaction class.
    pub interaction: Interaction,
    /// Native payload encoding.
    pub native_encoding: String,
    /// Fully-qualified primary Protobuf message type.
    pub protobuf_type: String,
    /// Optional response Protobuf type for a query.
    pub response_protobuf_type: Option<String>,
    /// Per-route copy of the descriptor identity.
    pub protobuf_descriptor_sha256: String,
    /// Pre-decode payload bound.
    pub maximum_encoded_bytes: usize,
    /// Maximum accepted/publication rate.
    pub maximum_rate_hz: f64,
    /// Bounded adapter queue depth.
    pub queue_depth: usize,
    /// Application freshness/timeout bound.
    pub freshness_ns: u64,
    /// Exact ROS graph endpoint.
    pub ros_name: String,
    /// Fully-qualified ROS interface type.
    pub ros_type: String,
    /// ROS distribution for this type identity.
    pub ros_distribution: String,
    /// Source path relative to the profile root.
    pub ros_interface_source: String,
    /// SHA-256 of the exact source snapshot.
    pub ros_interface_sha256: String,
    /// Native ROS type hash when that distribution exposes one.
    pub ros_type_hash: Option<String>,
    /// Explicit middleware policy.
    pub qos: QosPolicy,
    /// Timestamp conversion policy.
    pub clock: ClockPolicy,
    /// Frame projection policy.
    pub frame: FramePolicy,
    /// Read/authority classification.
    pub authorization: Authorization,
    /// Standard ROS projections lose canonical metadata.
    pub lossy_projection: bool,
    /// Companion metadata endpoint; empty only for lossless custom interfaces.
    pub metadata_ros_name: String,
    /// Fixed origin identity used by loop suppression.
    pub origin_id: String,
    /// Service-only policy.
    pub service: Option<ServicePolicy>,
    /// Action-only policy.
    pub action: Option<ActionPolicy>,
}

/// Route direction.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Direction {
    /// Native protocol to ROS.
    Egress,
    /// ROS to native protocol.
    Ingress,
    /// Bounded idempotent request/reply.
    RequestReply,
    /// Fixed ROS three-service/two-topic action lifecycle.
    ActionLifecycle,
}

/// Semantic interaction class.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Interaction {
    /// High-rate sensor stream.
    SensorStream,
    /// State stream.
    StateStream,
    /// Dynamic transform stream.
    TransformDynamic,
    /// Static transform stream.
    TransformStatic,
    /// Guarded motion command.
    Command,
    /// Bounded idempotent service.
    Service,
    /// Complete action lifecycle.
    Action,
}

/// Explicit bounded ROS QoS policy.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct QosPolicy {
    /// Delivery reliability.
    pub reliability: Reliability,
    /// Sample persistence.
    pub durability: Durability,
    /// Only bounded keep-last is supported by this schema.
    pub history: History,
    /// Keep-last depth.
    pub depth: usize,
    /// Requested deadline; zero explicitly disables it.
    pub deadline_ns: u64,
    /// Requested lifespan; zero explicitly disables it.
    pub lifespan_ns: u64,
    /// Liveliness mode.
    pub liveliness: Liveliness,
    /// Requested lease; zero explicitly disables it.
    pub liveliness_lease_ns: u64,
}

/// ROS reliability policy.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Reliability {
    /// Latest/drop sensor delivery.
    BestEffort,
    /// Reliable bounded delivery.
    Reliable,
}

/// ROS durability policy.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Durability {
    /// No replay after publisher restart.
    Volatile,
    /// Late joiners receive retained values.
    TransientLocal,
}

/// ROS history policy; KEEP_ALL is intentionally unrepresentable.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum History {
    /// Bounded keep-last storage.
    KeepLast,
}

/// ROS liveliness policy.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Liveliness {
    /// Middleware-managed automatic liveliness.
    Automatic,
    /// Publisher process explicitly asserts liveliness.
    ManualByTopic,
}

/// Clock conversion policy.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ClockPolicy {
    /// Mapping behavior.
    pub mode: ClockMode,
    /// Maximum trusted acquisition uncertainty; zero if not applicable.
    pub maximum_uncertainty_ns: u64,
    /// Maximum ROS command age; zero for egress.
    pub command_maximum_age_ns: u64,
    /// Commands must construct a local monotonic deadline.
    pub construct_local_deadline: bool,
}

/// Supported timestamp projection mode.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ClockMode {
    /// Drop without a trusted acquisition timestamp.
    RequireAcquisition,
    /// Use bridge reception time and mark degradation when acquisition is absent.
    ReceptionTimeDegraded,
    /// Treat ROS time only as ingress observation metadata.
    ObserveRosIngress,
}

/// Frame projection policy.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct FramePolicy {
    /// Mapping behavior.
    pub mode: FrameMode,
    /// Apply the checked frame alias table.
    pub use_alias_table: bool,
}

/// Supported frame projection mode.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum FrameMode {
    /// Preserve the sample frame after optional aliasing.
    Preserve,
    /// Publish an explicitly owned dynamic transform.
    TfDynamic,
    /// Publish an explicitly owned static transform.
    TfStatic,
    /// Interpret command values in the configured body frame.
    BodyCommand,
    /// Interface carries no coordinate frame.
    None,
}

/// Route authorization class reserved for later ACL enforcement.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Authorization {
    /// Read-only publication/query.
    ReadOnly,
    /// Ingress requires an existing local lease and bounded local deadline.
    LocalLeaseRequired,
}

/// Service-specific bounded behavior.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ServicePolicy {
    /// Request timeout.
    pub timeout_ns: u64,
    /// Requests must be safe to retry by correlation ID.
    pub idempotent: bool,
}

/// Action-specific generated lifecycle behavior.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ActionPolicy {
    /// Registered canonical goal payload.
    pub goal_protobuf_type: String,
    /// Registered canonical feedback payload.
    pub feedback_protobuf_type: String,
    /// Registered canonical result payload.
    pub result_protobuf_type: String,
    /// Maximum retained terminal results.
    pub result_cache_depth: usize,
    /// Terminal result retention time.
    pub result_retention_ns: u64,
    /// Bounded feedback queue depth.
    pub feedback_depth: usize,
    /// Status is retained for action late joiners.
    pub status_transient_local: bool,
}

/// Validated route registry and deterministic fingerprint.
#[derive(Debug)]
pub struct RouteRegistry {
    manifest: Manifest,
    fingerprint: String,
    route_by_id: BTreeMap<String, usize>,
    route_by_native_key: BTreeMap<String, usize>,
    route_by_ros_name: BTreeMap<String, usize>,
}

impl RouteRegistry {
    /// Loads `routes.toml` and validates it against the generated descriptor set.
    pub fn load(profile_root: impl AsRef<Path>) -> Result<Self, RegistryError> {
        Self::load_with_descriptor(profile_root, proto_generated::FILE_DESCRIPTOR_SET)
    }

    /// Loads a profile against explicit descriptor bytes (used by compatibility tests).
    pub fn load_with_descriptor(
        profile_root: impl AsRef<Path>,
        descriptor_bytes: &[u8],
    ) -> Result<Self, RegistryError> {
        let profile_root = profile_root.as_ref();
        let manifest_path = profile_root.join("routes.toml");
        let source = fs::read_to_string(&manifest_path).map_err(|source| RegistryError::Read {
            path: manifest_path.clone(),
            source,
        })?;
        let manifest: Manifest = toml::from_str(&source)?;
        validate_manifest(&manifest, profile_root, descriptor_bytes)?;

        let canonical = toml::to_string(&manifest)?;
        let fingerprint = sha256_hex(canonical.as_bytes());
        let mut route_by_id = BTreeMap::new();
        let mut route_by_native_key = BTreeMap::new();
        let mut route_by_ros_name = BTreeMap::new();
        for (index, route) in manifest.route.iter().enumerate() {
            route_by_id.insert(route.id.clone(), index);
            route_by_native_key.insert(route.native_key.clone(), index);
            route_by_ros_name.insert(route.ros_name.clone(), index);
        }
        Ok(Self {
            manifest,
            fingerprint,
            route_by_id,
            route_by_native_key,
            route_by_ros_name,
        })
    }

    /// Returns the validated manifest.
    #[must_use]
    pub const fn manifest(&self) -> &Manifest {
        &self.manifest
    }

    /// Returns deterministic SHA-256 over canonical manifest serialization.
    #[must_use]
    pub fn fingerprint(&self) -> &str {
        &self.fingerprint
    }

    /// Finds a route by stable ID.
    #[must_use]
    pub fn route_by_id(&self, id: &str) -> Option<&Route> {
        self.route_by_id
            .get(id)
            .map(|index| &self.manifest.route[*index])
    }

    /// Finds a route by exact native key.
    #[must_use]
    pub fn route_by_native_key(&self, key: &str) -> Option<&Route> {
        self.route_by_native_key
            .get(key)
            .map(|index| &self.manifest.route[*index])
    }

    /// Finds a route by explicit ROS endpoint.
    #[must_use]
    pub fn route_by_ros_name(&self, name: &str) -> Option<&Route> {
        self.route_by_ros_name
            .get(name)
            .map(|index| &self.manifest.route[*index])
    }
}

fn validate_manifest(
    manifest: &Manifest,
    profile_root: &Path,
    descriptor_bytes: &[u8],
) -> Result<(), RegistryError> {
    require(
        manifest.schema_version == MANIFEST_SCHEMA_VERSION,
        "schema_version",
        "unsupported manifest schema",
    )?;
    require(
        !manifest.profile.id.is_empty(),
        "profile.id",
        "cannot be empty",
    )?;
    for (field, value) in [
        ("profile.architecture", &manifest.profile.architecture),
        (
            "profile.operating_system",
            &manifest.profile.operating_system,
        ),
        (
            "profile.ros_distribution",
            &manifest.profile.ros_distribution,
        ),
        ("profile.rmw", &manifest.profile.rmw),
        (
            "profile.ros_interface_revision",
            &manifest.profile.ros_interface_revision,
        ),
        ("profile.zenoh_version", &manifest.profile.zenoh_version),
        ("profile.ros2dds_version", &manifest.profile.ros2dds_version),
        (
            "profile.ros2dds_build_id",
            &manifest.profile.ros2dds_build_id,
        ),
    ] {
        require(!value.is_empty(), field, "cannot be empty")?;
    }
    require(
        manifest.profile.native_encoding == PROTOBUF_ENCODING,
        "profile.native_encoding",
        "must be application/protobuf",
    )?;
    require(
        manifest.profile.adapter_outside_realtime,
        "profile.adapter_outside_realtime",
        "adapter must remain outside realtime",
    )?;
    require(
        !manifest.profile.adapter_can_issue_leases,
        "profile.adapter_can_issue_leases",
        "adapter cannot issue authority leases",
    )?;
    require(
        manifest.profile.zenoh_version == SUPPORTED_ZENOH_VERSION,
        "profile.zenoh_version",
        "unsupported Zenoh compatibility version",
    )?;
    require(
        manifest.profile.ros2dds_version == SUPPORTED_ROS2DDS_VERSION,
        "profile.ros2dds_version",
        "ros2dds must match the supported Zenoh compatibility version",
    )?;
    require(
        manifest.profile.ros2dds_build_id == SUPPORTED_ROS2DDS_BUILD_ID,
        "profile.ros2dds_build_id",
        "unexpected standalone ros2dds build identity",
    )?;
    require(
        manifest.profile.ros2dds_mode == Ros2DdsMode::Standalone,
        "profile.ros2dds_mode",
        "ros2dds must run as a standalone executable",
    )?;
    validate_sha256(
        &manifest.profile.protobuf_descriptor_sha256,
        "profile.protobuf_descriptor_sha256",
    )?;
    require(
        sha256_hex(descriptor_bytes) == manifest.profile.protobuf_descriptor_sha256,
        "profile.protobuf_descriptor_sha256",
        "does not match generated descriptor set",
    )?;
    require(
        !prefixes_overlap(
            &manifest.deployment.native_key_prefix,
            &manifest.deployment.ros2dds_key_prefix,
        ),
        "deployment key prefixes",
        "native and ros2dds prefixes overlap",
    )?;
    let direct_hosts = unique_nonempty(&manifest.deployment.direct_dds_hosts, "direct_dds_hosts")?;
    let bridged_hosts = unique_nonempty(&manifest.deployment.ros2dds_hosts, "ros2dds_hosts")?;
    require(
        direct_hosts.is_disjoint(&bridged_hosts),
        "deployment DDS hosts",
        "a host cannot have direct DDS and ros2dds paths",
    )?;

    let descriptor_types = descriptor_types(descriptor_bytes)?;
    validate_aliases(&manifest.frame_alias)?;

    let mut ids = BTreeSet::new();
    let mut native_keys = BTreeSet::new();
    let mut ros_names = BTreeSet::new();
    for route in &manifest.route {
        require(
            ids.insert(route.id.as_str()),
            "route.id",
            "duplicate route ID",
        )?;
        require(
            native_keys.insert(route.native_key.as_str()),
            "route.native_key",
            "duplicate native key ownership",
        )?;
        require(
            ros_names.insert(route.ros_name.as_str()),
            "route.ros_name",
            "duplicate ROS endpoint ownership",
        )?;
        validate_route(route, manifest, profile_root, &descriptor_types)?;
    }
    require(
        !manifest.route.is_empty(),
        "route",
        "at least one route is required",
    )?;
    validate_tf_authorities(manifest, &ids)?;
    Ok(())
}

fn validate_route(
    route: &Route,
    manifest: &Manifest,
    profile_root: &Path,
    descriptor_types: &BTreeSet<String>,
) -> Result<(), RegistryError> {
    require(!route.id.is_empty(), "route.id", "cannot be empty")?;
    let prefix = manifest.deployment.native_key_prefix.trim_end_matches('/');
    require(
        route.native_key.starts_with(&format!("{prefix}/")),
        "route.native_key",
        "must be below the native key prefix",
    )?;
    require(
        route.native_encoding == PROTOBUF_ENCODING,
        "route.native_encoding",
        "must be application/protobuf",
    )?;
    require(
        route.protobuf_descriptor_sha256 == manifest.profile.protobuf_descriptor_sha256,
        "route.protobuf_descriptor_sha256",
        "must match the profile descriptor",
    )?;
    require(
        descriptor_types.contains(&route.protobuf_type),
        "route.protobuf_type",
        "type is absent from descriptor set",
    )?;
    if let Some(response) = &route.response_protobuf_type {
        require(
            descriptor_types.contains(response),
            "route.response_protobuf_type",
            "type is absent from descriptor set",
        )?;
    }
    require(
        route.maximum_encoded_bytes > 0,
        "route.maximum_encoded_bytes",
        "must be bounded and nonzero",
    )?;
    require(
        route.maximum_rate_hz.is_finite() && route.maximum_rate_hz > 0.0,
        "route.maximum_rate_hz",
        "must be finite and positive",
    )?;
    require(
        route.queue_depth > 0,
        "route.queue_depth",
        "must be bounded and nonzero",
    )?;
    require(
        route.freshness_ns > 0,
        "route.freshness_ns",
        "must be bounded and nonzero",
    )?;
    require(
        route.ros_name.starts_with('/') && !route.ros_name.contains("//"),
        "route.ros_name",
        "must be an explicit absolute ROS name",
    )?;
    require(
        route.ros_distribution == manifest.profile.ros_distribution,
        "route.ros_distribution",
        "must match the profile distribution",
    )?;
    require(route.qos.depth > 0, "route.qos.depth", "must be nonzero")?;
    validate_sha256(&route.ros_interface_sha256, "route.ros_interface_sha256")?;
    if let Some(type_hash) = &route.ros_type_hash {
        validate_sha256(type_hash, "route.ros_type_hash")?;
    }
    validate_source_digest(route, profile_root)?;
    validate_route_class(route, descriptor_types)?;

    let is_custom = route.ros_type.starts_with("rs_control_interfaces/");
    if !is_custom {
        require(
            route.lossy_projection,
            "route.lossy_projection",
            "standard ROS projections must declare metadata loss",
        )?;
    }
    if route.lossy_projection {
        require(
            route.metadata_ros_name.starts_with('/'),
            "route.metadata_ros_name",
            "lossy projection requires a companion metadata endpoint",
        )?;
    }
    require(
        !route.origin_id.is_empty(),
        "route.origin_id",
        "loop-suppression origin identity is required",
    )?;
    Ok(())
}

fn validate_route_class(
    route: &Route,
    descriptor_types: &BTreeSet<String>,
) -> Result<(), RegistryError> {
    match route.interaction {
        Interaction::SensorStream | Interaction::StateStream => {
            require(
                route.direction == Direction::Egress,
                "route.direction",
                "stream must be egress",
            )?;
            require(
                route.qos.reliability == Reliability::BestEffort
                    && route.qos.durability == Durability::Volatile,
                "route.qos",
                "sensor/state stream must be best_effort and volatile",
            )?;
            require(
                matches!(
                    route.clock.mode,
                    ClockMode::RequireAcquisition | ClockMode::ReceptionTimeDegraded
                ),
                "route.clock.mode",
                "egress stream requires an acquisition/fallback policy",
            )?;
        }
        Interaction::TransformDynamic => {
            require(
                route.direction == Direction::Egress,
                "route.direction",
                "TF must be egress",
            )?;
            require(
                route.qos.durability == Durability::Volatile,
                "route.qos.durability",
                "dynamic TF must be volatile",
            )?;
            require(
                route.frame.mode == FrameMode::TfDynamic,
                "route.frame.mode",
                "dynamic TF frame policy required",
            )?;
        }
        Interaction::TransformStatic => {
            require(
                route.direction == Direction::Egress,
                "route.direction",
                "TF must be egress",
            )?;
            require(
                route.qos.durability == Durability::TransientLocal,
                "route.qos.durability",
                "static TF must be transient_local",
            )?;
            require(
                route.frame.mode == FrameMode::TfStatic,
                "route.frame.mode",
                "static TF frame policy required",
            )?;
        }
        Interaction::Command => {
            require(
                route.direction == Direction::Ingress,
                "route.direction",
                "command must be ingress",
            )?;
            require(
                route.qos.reliability == Reliability::Reliable
                    && route.qos.durability == Durability::Volatile,
                "route.qos",
                "command must be reliable and volatile",
            )?;
            require(
                route.authorization == Authorization::LocalLeaseRequired,
                "route.authorization",
                "command requires local lease",
            )?;
            require(
                route.clock.mode == ClockMode::ObserveRosIngress
                    && route.clock.construct_local_deadline
                    && route.clock.command_maximum_age_ns > 0,
                "route.clock",
                "command requires observation-age and local deadline policy",
            )?;
            require(
                route.frame.mode == FrameMode::BodyCommand,
                "route.frame.mode",
                "body command frame policy required",
            )?;
        }
        Interaction::Service => {
            require(
                route.direction == Direction::RequestReply,
                "route.direction",
                "service must be request_reply",
            )?;
            require(
                route.qos.reliability == Reliability::Reliable
                    && route.qos.durability == Durability::Volatile,
                "route.qos",
                "service must be reliable and volatile",
            )?;
            let service = route
                .service
                .as_ref()
                .ok_or_else(|| invalid("route.service", "service policy is required"))?;
            require(
                service.timeout_ns > 0 && service.idempotent,
                "route.service",
                "service must be bounded and idempotent",
            )?;
            require(
                route.response_protobuf_type.is_some(),
                "route.response_protobuf_type",
                "service response type is required",
            )?;
        }
        Interaction::Action => {
            require(
                route.direction == Direction::ActionLifecycle,
                "route.direction",
                "action lifecycle direction required",
            )?;
            require(
                route.authorization == Authorization::LocalLeaseRequired,
                "route.authorization",
                "action requires local lease",
            )?;
            require(
                route.clock.construct_local_deadline,
                "route.clock.construct_local_deadline",
                "action requires local deadline",
            )?;
            let action = route
                .action
                .as_ref()
                .ok_or_else(|| invalid("route.action", "action policy is required"))?;
            for (field, type_name) in [
                (
                    "route.action.goal_protobuf_type",
                    &action.goal_protobuf_type,
                ),
                (
                    "route.action.feedback_protobuf_type",
                    &action.feedback_protobuf_type,
                ),
                (
                    "route.action.result_protobuf_type",
                    &action.result_protobuf_type,
                ),
            ] {
                require(
                    descriptor_types.contains(type_name),
                    field,
                    "type is absent from descriptor set",
                )?;
            }
            require(
                action.result_cache_depth > 0
                    && action.result_retention_ns > 0
                    && action.feedback_depth > 0,
                "route.action",
                "action storage and retention must be bounded and nonzero",
            )?;
            require(
                action.status_transient_local,
                "route.action.status_transient_local",
                "late-joiner action status must be retained",
            )?;
        }
    }
    require(
        route.service.is_some() == (route.interaction == Interaction::Service),
        "route.service",
        "service policy is only valid for service routes",
    )?;
    require(
        route.action.is_some() == (route.interaction == Interaction::Action),
        "route.action",
        "action policy is only valid for action routes",
    )?;
    Ok(())
}

fn validate_source_digest(route: &Route, profile_root: &Path) -> Result<(), RegistryError> {
    let relative = Path::new(&route.ros_interface_source);
    require(
        !relative.as_os_str().is_empty()
            && relative
                .components()
                .all(|part| matches!(part, Component::Normal(_))),
        "route.ros_interface_source",
        "must be a relative traversal-free path",
    )?;
    let path = profile_root.join(relative);
    let source = fs::read(&path).map_err(|source| RegistryError::Read {
        path: path.clone(),
        source,
    })?;
    require(
        sha256_hex(&source) == route.ros_interface_sha256,
        "route.ros_interface_sha256",
        "does not match checked-in interface source",
    )
}

fn validate_aliases(aliases: &[FrameAlias]) -> Result<(), RegistryError> {
    let mut canonical = BTreeSet::new();
    let mut ros = BTreeSet::new();
    for alias in aliases {
        AsciiId::<64>::parse(&alias.canonical)
            .map_err(|_| invalid("frame_alias.canonical", "must be a portable slash-free ID"))?;
        validate_ros_frame(&alias.ros)?;
        require(
            canonical.insert(alias.canonical.as_str()),
            "frame_alias.canonical",
            "duplicate canonical frame",
        )?;
        require(
            ros.insert(alias.ros.as_str()),
            "frame_alias.ros",
            "duplicate ROS frame",
        )?;
    }
    Ok(())
}

fn validate_tf_authorities(
    manifest: &Manifest,
    route_ids: &BTreeSet<&str>,
) -> Result<(), RegistryError> {
    let mut children = BTreeSet::new();
    let mut parents = BTreeMap::<&str, &str>::new();
    for authority in &manifest.tf_authority {
        AsciiId::<64>::parse(&authority.parent)
            .map_err(|_| invalid("tf_authority.parent", "invalid canonical frame"))?;
        AsciiId::<64>::parse(&authority.child)
            .map_err(|_| invalid("tf_authority.child", "invalid canonical frame"))?;
        require(
            authority.parent != authority.child,
            "tf_authority",
            "self edge is forbidden",
        )?;
        require(
            children.insert(authority.child.as_str()),
            "tf_authority.child",
            "child has multiple authorities",
        )?;
        require(
            route_ids.contains(authority.route_id.as_str()),
            "tf_authority.route_id",
            "unknown route",
        )?;
        let route = manifest
            .route
            .iter()
            .find(|route| route.id == authority.route_id)
            .expect("validated route ID exists");
        let expected = match authority.kind {
            TfKind::Dynamic => Interaction::TransformDynamic,
            TfKind::Static => Interaction::TransformStatic,
        };
        require(
            route.interaction == expected,
            "tf_authority.route_id",
            "route interaction does not match TF kind",
        )?;
        parents.insert(authority.child.as_str(), authority.parent.as_str());
    }
    for child in parents.keys() {
        let mut seen = BTreeSet::new();
        let mut cursor = *child;
        while let Some(parent) = parents.get(cursor) {
            require(
                seen.insert(cursor),
                "tf_authority",
                "frame graph contains a cycle",
            )?;
            cursor = parent;
        }
    }
    Ok(())
}

fn validate_ros_frame(value: &str) -> Result<(), RegistryError> {
    require(
        !value.is_empty()
            && !value.starts_with('/')
            && !value.ends_with('/')
            && !value.contains("//"),
        "frame_alias.ros",
        "leading, trailing or double slash is forbidden",
    )?;
    for segment in value.split('/') {
        AsciiId::<64>::parse(segment)
            .map_err(|_| invalid("frame_alias.ros", "contains a nonportable segment"))?;
    }
    Ok(())
}

fn descriptor_types(bytes: &[u8]) -> Result<BTreeSet<String>, RegistryError> {
    let set = FileDescriptorSet::decode(bytes)?;
    let mut types = BTreeSet::new();
    for file in set.file {
        let package = file.package.unwrap_or_default();
        for message in file.message_type {
            collect_message_types(&package, &message, &mut types);
        }
    }
    Ok(types)
}

fn collect_message_types(prefix: &str, message: &DescriptorProto, out: &mut BTreeSet<String>) {
    let Some(name) = message.name.as_deref() else {
        return;
    };
    let full_name = if prefix.is_empty() {
        name.to_owned()
    } else {
        format!("{prefix}.{name}")
    };
    out.insert(full_name.clone());
    for nested in &message.nested_type {
        collect_message_types(&full_name, nested, out);
    }
}

fn unique_nonempty<'a>(
    values: &'a [String],
    field: &'static str,
) -> Result<BTreeSet<&'a str>, RegistryError> {
    let mut unique = BTreeSet::new();
    for value in values {
        require(!value.is_empty(), field, "entries cannot be empty")?;
        require(unique.insert(value.as_str()), field, "duplicate entry")?;
    }
    Ok(unique)
}

fn prefixes_overlap(left: &str, right: &str) -> bool {
    let left = left.trim_end_matches('/');
    let right = right.trim_end_matches('/');
    left == right
        || left.starts_with(&format!("{right}/"))
        || right.starts_with(&format!("{left}/"))
}

fn validate_sha256(value: &str, field: &'static str) -> Result<(), RegistryError> {
    require(
        value.len() == 64
            && value
                .bytes()
                .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase()),
        field,
        "must be 64 lowercase hexadecimal characters",
    )
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        write!(&mut output, "{byte:02x}").expect("writing to String cannot fail");
    }
    output
}

fn require(
    condition: bool,
    field: &'static str,
    reason: &'static str,
) -> Result<(), RegistryError> {
    if condition {
        Ok(())
    } else {
        Err(invalid(field, reason))
    }
}

fn invalid(field: &'static str, reason: &'static str) -> RegistryError {
    RegistryError::Invalid { field, reason }
}

/// Route-registry load or validation failure.
#[derive(Debug, Error)]
pub enum RegistryError {
    /// Profile or interface source could not be read.
    #[error("failed to read {path}: {source}")]
    Read {
        /// Failed path.
        path: PathBuf,
        /// Filesystem error.
        #[source]
        source: io::Error,
    },
    /// TOML parse failure.
    #[error("invalid route manifest TOML: {0}")]
    Parse(#[from] toml::de::Error),
    /// Canonical TOML serialization failure.
    #[error("failed to canonicalize route manifest: {0}")]
    Serialize(#[from] toml::ser::Error),
    /// Protobuf descriptor set was malformed.
    #[error("invalid Protobuf descriptor set: {0}")]
    Descriptor(#[from] prost::DecodeError),
    /// Typed validation failure.
    #[error("invalid {field}: {reason}")]
    Invalid {
        /// Rejected manifest field.
        field: &'static str,
        /// Stable diagnostic reason.
        reason: &'static str,
    },
}

#[cfg(test)]
mod tests {
    use std::{fs, path::PathBuf};

    use super::{
        Direction, Ros2DdsMode, TfAuthority, TfKind, prefixes_overlap, validate_manifest,
        validate_ros_frame,
    };

    fn profile_root() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../config/protocol/ros2-humble")
    }

    fn manifest() -> super::Manifest {
        toml::from_str(&fs::read_to_string(profile_root().join("routes.toml")).unwrap()).unwrap()
    }

    #[test]
    fn key_prefix_overlap_is_segment_aware() {
        assert!(prefixes_overlap("rs/robot", "rs/robot/ros2dds"));
        assert!(!prefixes_overlap("rs/robot/v1", "ros2dds/robot"));
    }

    #[test]
    fn ros_frames_reject_implicit_slash_normalization() {
        assert!(validate_ros_frame("robot_1/base_link").is_ok());
        assert!(validate_ros_frame("/robot_1/base_link").is_err());
        assert!(validate_ros_frame("robot_1//base_link").is_err());
    }

    #[test]
    fn manifest_rejects_duplicate_ownership_and_unbounded_routes() {
        let mut duplicate = manifest();
        duplicate.route[1].id = duplicate.route[0].id.clone();
        assert!(
            validate_manifest(
                &duplicate,
                &profile_root(),
                proto_generated::FILE_DESCRIPTOR_SET
            )
            .is_err()
        );

        let mut unbounded = manifest();
        unbounded.route[0].queue_depth = 0;
        assert!(
            validate_manifest(
                &unbounded,
                &profile_root(),
                proto_generated::FILE_DESCRIPTOR_SET
            )
            .is_err()
        );
    }

    #[test]
    fn manifest_rejects_qos_direction_and_authority_mismatch() {
        let mut wrong_direction = manifest();
        wrong_direction.route[5].direction = Direction::Egress;
        assert!(
            validate_manifest(
                &wrong_direction,
                &profile_root(),
                proto_generated::FILE_DESCRIPTOR_SET
            )
            .is_err()
        );

        let mut wrong_qos = manifest();
        wrong_qos.route[0].qos.reliability = super::Reliability::Reliable;
        assert!(
            validate_manifest(
                &wrong_qos,
                &profile_root(),
                proto_generated::FILE_DESCRIPTOR_SET
            )
            .is_err()
        );
    }

    #[test]
    fn manifest_rejects_duplicate_network_paths_and_key_collisions() {
        let mut duplicate_path = manifest();
        duplicate_path
            .deployment
            .direct_dds_hosts
            .push("operator-station".to_owned());
        assert!(
            validate_manifest(
                &duplicate_path,
                &profile_root(),
                proto_generated::FILE_DESCRIPTOR_SET
            )
            .is_err()
        );

        let mut collision = manifest();
        collision.deployment.ros2dds_key_prefix = "rs/robot/v1/ros2dds".to_owned();
        assert!(
            validate_manifest(
                &collision,
                &profile_root(),
                proto_generated::FILE_DESCRIPTOR_SET
            )
            .is_err()
        );
    }

    #[test]
    fn manifest_rejects_incompatible_or_embedded_ros2dds() {
        let mut wrong_zenoh = manifest();
        wrong_zenoh.profile.zenoh_version = "1.8.0".to_owned();
        assert!(
            validate_manifest(
                &wrong_zenoh,
                &profile_root(),
                proto_generated::FILE_DESCRIPTOR_SET
            )
            .is_err()
        );

        let mut wrong_bridge = manifest();
        wrong_bridge.profile.ros2dds_version = "1.8.0".to_owned();
        assert!(
            validate_manifest(
                &wrong_bridge,
                &profile_root(),
                proto_generated::FILE_DESCRIPTOR_SET
            )
            .is_err()
        );

        let mut plugin = manifest();
        plugin.profile.ros2dds_mode = Ros2DdsMode::DynamicPlugin;
        assert!(
            validate_manifest(
                &plugin,
                &profile_root(),
                proto_generated::FILE_DESCRIPTOR_SET
            )
            .is_err()
        );
    }

    #[test]
    fn manifest_rejects_tf_cycles_and_source_traversal() {
        let mut cycle = manifest();
        cycle.tf_authority.push(TfAuthority {
            parent: "base_link".to_owned(),
            child: "map".to_owned(),
            kind: TfKind::Dynamic,
            route_id: "tf_dynamic".to_owned(),
        });
        assert!(
            validate_manifest(
                &cycle,
                &profile_root(),
                proto_generated::FILE_DESCRIPTOR_SET
            )
            .is_err()
        );

        let mut traversal = manifest();
        traversal.route[0].ros_interface_source = "../routes.toml".to_owned();
        assert!(
            validate_manifest(
                &traversal,
                &profile_root(),
                proto_generated::FILE_DESCRIPTOR_SET
            )
            .is_err()
        );
    }
}
