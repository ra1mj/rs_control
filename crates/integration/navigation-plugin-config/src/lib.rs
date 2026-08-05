//! Strict startup contract for the crash-isolated chassis navigation plugin.

use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    fmt, fs,
    path::{Path, PathBuf},
};
use thiserror::Error;

/// Supported navigation-plugin manifest schema.
pub const SCHEMA_VERSION: u32 = 1;
/// Maximum accepted manifest size.
pub const MAX_MANIFEST_BYTES: usize = 64 * 1024;
/// Canonical plugin identity.
pub const PLUGIN_ID: &str = "chassis_nav.rs_control.navigation";
/// Motor-disabled host profile identity.
pub const PROFILE_ID: &str = "motor_disabled_host_v1";

const EXPECTED_CHANNELS: [(ChannelRole, ChannelMode, u32, u64); 7] = [
    (ChannelRole::Authority, ChannelMode::Ordered, 8, 100),
    (ChannelRole::ActionRequest, ChannelMode::Ordered, 8, 1_000),
    (ChannelRole::ActionResponse, ChannelMode::Ordered, 8, 1_000),
    (ChannelRole::ControllerProposal, ChannelMode::Latest, 3, 100),
    (ChannelRole::Odometry, ChannelMode::Latest, 3, 100),
    (ChannelRole::DynamicTf, ChannelMode::Latest, 3, 100),
    (ChannelRole::Health, ChannelMode::Latest, 3, 500),
];

/// One immutable, validated plugin configuration.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NavigationPluginManifest {
    raw: RawManifest,
    fingerprint: Sha256Digest,
}

impl NavigationPluginManifest {
    /// Parses and validates one YAML document and binds its exact byte identity.
    pub fn parse(yaml: &str) -> Result<Self, ManifestError> {
        if yaml.len() > MAX_MANIFEST_BYTES {
            return Err(ManifestError::TooLarge {
                actual: yaml.len(),
                maximum: MAX_MANIFEST_BYTES,
            });
        }
        let raw: RawManifest = serde_saphyr::from_str(yaml)
            .map_err(|error| ManifestError::Syntax(error.to_string()))?;
        validate(&raw)?;
        Ok(Self {
            raw,
            fingerprint: Sha256Digest::of(yaml.as_bytes()),
        })
    }

    /// Reads a bounded regular file and validates it.
    pub fn load(path: &Path) -> Result<Self, ManifestError> {
        let bytes = read_bounded_regular_file(path, MAX_MANIFEST_BYTES)?;
        let yaml = std::str::from_utf8(&bytes).map_err(|_| ManifestError::NotUtf8)?;
        Self::parse(yaml)
    }

    /// Returns the checked schema version.
    #[must_use]
    pub const fn schema_version(&self) -> u32 {
        self.raw.schema_version
    }

    /// Returns the checked plugin ID.
    #[must_use]
    pub fn plugin_id(&self) -> &str {
        &self.raw.plugin_id
    }

    /// Returns the checked profile ID.
    #[must_use]
    pub fn profile_id(&self) -> &str {
        &self.raw.profile_id
    }

    /// Returns SHA-256 over the exact manifest bytes.
    #[must_use]
    pub const fn fingerprint(&self) -> Sha256Digest {
        self.fingerprint
    }

    /// Returns the configured shared-memory namespace.
    #[must_use]
    pub fn ipc_namespace(&self) -> &str {
        &self.raw.ipc.namespace
    }

    /// Returns the exact map identity admitted by this profile.
    #[must_use]
    pub fn map_id(&self) -> &str {
        &self.raw.map.id
    }

    /// Returns the exact map version admitted by this profile.
    #[must_use]
    pub fn map_version(&self) -> &str {
        &self.raw.map.version
    }

    /// Returns the canonical frame of the admitted map.
    #[must_use]
    pub fn map_frame(&self) -> &str {
        &self.raw.frames.map
    }

    /// Returns the approved map-manifest digest as typed bytes.
    #[must_use]
    pub fn map_manifest_digest(&self) -> Sha256Digest {
        decode_validated_sha256(&self.raw.map.manifest_sha256)
    }

    /// Returns one fixed channel policy.
    #[must_use]
    pub fn channel(&self, role: ChannelRole) -> ChannelContract<'_> {
        let raw = self.raw.ipc.channels.get(role);
        ChannelContract {
            layout_id: &raw.layout_id,
            mode: raw.mode,
            capacity: raw.capacity,
            freshness_ms: raw.freshness_ms,
        }
    }

    /// Returns the expected digest for one external asset.
    #[must_use]
    pub fn asset_digest(&self, role: AssetRole) -> &str {
        match role {
            AssetRole::MapManifest => &self.raw.map.manifest_sha256,
            AssetRole::OccupancyYaml => &self.raw.map.occupancy_yaml_sha256,
            AssetRole::OccupancyImage => &self.raw.map.occupancy_image_sha256,
            AssetRole::BaseParameters => &self.raw.nav2_assets.base_parameters_sha256,
            AssetRole::TuningParameters => &self.raw.nav2_assets.tuning_parameters_sha256,
            AssetRole::IntegrationParameters => &self.raw.nav2_assets.integration_parameters_sha256,
            AssetRole::NavigateToPoseBt => &self.raw.nav2_assets.navigate_to_pose_bt_sha256,
            AssetRole::NavigateThroughPosesBt => {
                &self.raw.nav2_assets.navigate_through_poses_bt_sha256
            }
            AssetRole::NavigateToPoseInterface => {
                &self.raw.interfaces.navigate_to_pose.source_sha256
            }
            AssetRole::ControllerProposalInterface => {
                &self.raw.interfaces.controller_proposal.source_sha256
            }
        }
    }

    /// Verifies the complete exact asset set before runtime eligibility.
    pub fn verify_assets(
        &self,
        assets: &BTreeMap<AssetRole, PathBuf>,
    ) -> Result<(), ManifestError> {
        let actual: BTreeSet<_> = assets.keys().copied().collect();
        let expected: BTreeSet<_> = AssetRole::ALL.into_iter().collect();
        if actual != expected {
            let missing = expected.difference(&actual).copied().collect();
            let unexpected = actual.difference(&expected).copied().collect();
            return Err(ManifestError::AssetSet {
                missing,
                unexpected,
            });
        }
        for (role, path) in assets {
            if !path.is_absolute() {
                return Err(ManifestError::RelativeAssetPath {
                    role: *role,
                    path: path.clone(),
                });
            }
            let bytes = read_bounded_regular_file(path, usize::MAX)?;
            let actual = Sha256Digest::of(&bytes);
            let expected = self.asset_digest(*role);
            if actual.to_string() != expected {
                return Err(ManifestError::AssetDigest {
                    role: *role,
                    expected: expected.to_owned(),
                    actual,
                });
            }
        }
        Ok(())
    }

    /// Proves the other repository contains a byte-identical manifest copy.
    pub fn verify_peer_copy(local: &Path, peer: &Path) -> Result<Sha256Digest, ManifestError> {
        let local_bytes = read_bounded_regular_file(local, MAX_MANIFEST_BYTES)?;
        let peer_bytes = read_bounded_regular_file(peer, MAX_MANIFEST_BYTES)?;
        if local_bytes != peer_bytes {
            return Err(ManifestError::PeerMismatch);
        }
        Self::parse(std::str::from_utf8(&local_bytes).map_err(|_| ManifestError::NotUtf8)?)?;
        Ok(Sha256Digest::of(&local_bytes))
    }
}

/// Stable role for one checked external asset.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum AssetRole {
    /// Approved-map manifest fixture.
    MapManifest,
    /// Occupancy-grid YAML.
    OccupancyYaml,
    /// Occupancy-grid image.
    OccupancyImage,
    /// Base Nav2 parameters.
    BaseParameters,
    /// Low-resource tuning overlay.
    TuningParameters,
    /// rs_control-specific Nav2 overlay.
    IntegrationParameters,
    /// NavigateToPose behavior tree.
    NavigateToPoseBt,
    /// Disabled NavigateThroughPoses behavior tree.
    NavigateThroughPosesBt,
    /// Installed/snapshotted NavigateToPose action source.
    NavigateToPoseInterface,
    /// Installed/snapshotted Twist message source.
    ControllerProposalInterface,
}

impl AssetRole {
    /// Complete required asset set.
    pub const ALL: [Self; 10] = [
        Self::MapManifest,
        Self::OccupancyYaml,
        Self::OccupancyImage,
        Self::BaseParameters,
        Self::TuningParameters,
        Self::IntegrationParameters,
        Self::NavigateToPoseBt,
        Self::NavigateThroughPosesBt,
        Self::NavigateToPoseInterface,
        Self::ControllerProposalInterface,
    ];

    /// Stable CLI spelling.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::MapManifest => "map_manifest",
            Self::OccupancyYaml => "occupancy_yaml",
            Self::OccupancyImage => "occupancy_image",
            Self::BaseParameters => "base_parameters",
            Self::TuningParameters => "tuning_parameters",
            Self::IntegrationParameters => "integration_parameters",
            Self::NavigateToPoseBt => "navigate_to_pose_bt",
            Self::NavigateThroughPosesBt => "navigate_through_poses_bt",
            Self::NavigateToPoseInterface => "navigate_to_pose_interface",
            Self::ControllerProposalInterface => "controller_proposal_interface",
        }
    }
}

impl std::str::FromStr for AssetRole {
    type Err = ManifestError;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        Self::ALL
            .into_iter()
            .find(|role| role.as_str() == value)
            .ok_or_else(|| ManifestError::UnknownAssetRole(value.to_owned()))
    }
}

impl fmt::Display for AssetRole {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        formatter.write_str(self.as_str())
    }
}

/// Fixed shared-memory interaction class.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ChannelMode {
    /// Lossless bounded event stream.
    Ordered,
    /// Depth-one replaceable state/proposal stream.
    Latest,
}

/// Stable role of one navigation shared-memory channel.
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum ChannelRole {
    /// Current test/mission authority snapshot.
    Authority,
    /// Goal/cancel request stream.
    ActionRequest,
    /// Feedback/cancel/result stream.
    ActionResponse,
    /// Raw Nav2 body-twist proposal.
    ControllerProposal,
    /// Simulated odometry state.
    Odometry,
    /// Simulated dynamic TF.
    DynamicTf,
    /// Bridge health state.
    Health,
}

/// Borrowed immutable channel configuration.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ChannelContract<'a> {
    /// Exact stable 16-byte layout identity.
    pub layout_id: &'a str,
    /// Ordered/latest policy.
    pub mode: ChannelMode,
    /// Fixed channel capacity.
    pub capacity: u32,
    /// Exclusive local reception-age limit.
    pub freshness_ms: u64,
}

/// Typed SHA-256 identity.
#[derive(Clone, Copy, Eq, Hash, PartialEq)]
pub struct Sha256Digest([u8; 32]);

impl Sha256Digest {
    fn of(bytes: &[u8]) -> Self {
        let digest = Sha256::digest(bytes);
        let mut value = [0; 32];
        value.copy_from_slice(&digest);
        Self(value)
    }

    /// Returns digest bytes.
    #[must_use]
    pub const fn as_bytes(self) -> [u8; 32] {
        self.0
    }
}

impl fmt::Debug for Sha256Digest {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self, formatter)
    }
}

impl fmt::Display for Sha256Digest {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        for byte in self.0 {
            write!(formatter, "{byte:02x}")?;
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct RawManifest {
    schema_version: u32,
    plugin_id: String,
    profile_id: String,
    compatibility: Compatibility,
    interfaces: Interfaces,
    endpoints: Endpoints,
    frames: Frames,
    map: MapIdentity,
    nav2_assets: Nav2Assets,
    ipc: Ipc,
    safety: Safety,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct Compatibility {
    ros_distribution: String,
    rmw_implementation: String,
    rclrs_version: String,
    nav2_msgs_version: String,
    geometry_msgs_version: String,
    nav_msgs_version: String,
    tf2_msgs_version: String,
    protobuf_descriptor_sha256: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct Interfaces {
    navigate_to_pose: InterfaceWithSource,
    controller_proposal: InterfaceWithSource,
    odometry: Interface,
    dynamic_tf: Interface,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct InterfaceWithSource {
    ros_type: String,
    source_sha256: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct Interface {
    ros_type: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct Endpoints {
    navigate_to_pose_action: String,
    controller_proposal_topic: String,
    odometry_topic: String,
    dynamic_tf_topic: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct Frames {
    map: String,
    odom: String,
    base: String,
    tf_authorities: Vec<TfAuthority>,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct TfAuthority {
    parent: String,
    child: String,
    writer: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct MapIdentity {
    id: String,
    version: String,
    manifest_sha256: String,
    occupancy_yaml_sha256: String,
    occupancy_image_sha256: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct Nav2Assets {
    base_parameters_sha256: String,
    tuning_parameters_sha256: String,
    integration_parameters_sha256: String,
    navigate_to_pose_bt_sha256: String,
    navigate_through_poses_bt_sha256: String,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct Ipc {
    namespace: String,
    channels: Channels,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct Channels {
    authority: RawChannel,
    action_request: RawChannel,
    action_response: RawChannel,
    controller_proposal: RawChannel,
    odometry: RawChannel,
    dynamic_tf: RawChannel,
    health: RawChannel,
}

impl Channels {
    fn get(&self, role: ChannelRole) -> &RawChannel {
        match role {
            ChannelRole::Authority => &self.authority,
            ChannelRole::ActionRequest => &self.action_request,
            ChannelRole::ActionResponse => &self.action_response,
            ChannelRole::ControllerProposal => &self.controller_proposal,
            ChannelRole::Odometry => &self.odometry,
            ChannelRole::DynamicTf => &self.dynamic_tf,
            ChannelRole::Health => &self.health,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct RawChannel {
    layout_id: String,
    mode: ChannelMode,
    capacity: u32,
    freshness_ms: u64,
}

#[derive(Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(deny_unknown_fields)]
struct Safety {
    final_constraint_owner: String,
    adapter_can_issue_leases: bool,
    motor_output: String,
    can_transmit: String,
    simulation_only: bool,
    approved_for_production: bool,
    first_resource_run: String,
}

fn validate(raw: &RawManifest) -> Result<(), ManifestError> {
    exact("schema_version", raw.schema_version, SCHEMA_VERSION)?;
    exact_str("plugin_id", &raw.plugin_id, PLUGIN_ID)?;
    exact_str("profile_id", &raw.profile_id, PROFILE_ID)?;
    for (field, actual, expected) in [
        (
            "compatibility.ros_distribution",
            raw.compatibility.ros_distribution.as_str(),
            "humble",
        ),
        (
            "compatibility.rmw_implementation",
            raw.compatibility.rmw_implementation.as_str(),
            "rmw_fastrtps_cpp",
        ),
        (
            "compatibility.rclrs_version",
            raw.compatibility.rclrs_version.as_str(),
            "0.7.0",
        ),
        (
            "compatibility.nav2_msgs_version",
            raw.compatibility.nav2_msgs_version.as_str(),
            "1.1.20",
        ),
        (
            "compatibility.geometry_msgs_version",
            raw.compatibility.geometry_msgs_version.as_str(),
            "4.9.1",
        ),
        (
            "compatibility.nav_msgs_version",
            raw.compatibility.nav_msgs_version.as_str(),
            "4.9.1",
        ),
        (
            "compatibility.tf2_msgs_version",
            raw.compatibility.tf2_msgs_version.as_str(),
            "0.25.20",
        ),
    ] {
        exact_str(field, actual, expected)?;
    }
    validate_sha256(
        "compatibility.protobuf_descriptor_sha256",
        &raw.compatibility.protobuf_descriptor_sha256,
    )?;
    let descriptor = Sha256Digest::of(proto_generated::FILE_DESCRIPTOR_SET).to_string();
    exact_str(
        "compatibility.protobuf_descriptor_sha256",
        &raw.compatibility.protobuf_descriptor_sha256,
        &descriptor,
    )?;

    for (field, actual, expected) in [
        (
            "interfaces.navigate_to_pose.ros_type",
            raw.interfaces.navigate_to_pose.ros_type.as_str(),
            "nav2_msgs/action/NavigateToPose",
        ),
        (
            "interfaces.controller_proposal.ros_type",
            raw.interfaces.controller_proposal.ros_type.as_str(),
            "geometry_msgs/msg/Twist",
        ),
        (
            "interfaces.odometry.ros_type",
            raw.interfaces.odometry.ros_type.as_str(),
            "nav_msgs/msg/Odometry",
        ),
        (
            "interfaces.dynamic_tf.ros_type",
            raw.interfaces.dynamic_tf.ros_type.as_str(),
            "tf2_msgs/msg/TFMessage",
        ),
        (
            "endpoints.navigate_to_pose_action",
            raw.endpoints.navigate_to_pose_action.as_str(),
            "/navigate_to_pose",
        ),
        (
            "endpoints.controller_proposal_topic",
            raw.endpoints.controller_proposal_topic.as_str(),
            "/rs_control/navigation/controller_proposal",
        ),
        (
            "endpoints.odometry_topic",
            raw.endpoints.odometry_topic.as_str(),
            "/rs_control/navigation/odometry",
        ),
        (
            "endpoints.dynamic_tf_topic",
            raw.endpoints.dynamic_tf_topic.as_str(),
            "/tf",
        ),
    ] {
        exact_str(field, actual, expected)?;
    }
    validate_sha256(
        "interfaces.navigate_to_pose.source_sha256",
        &raw.interfaces.navigate_to_pose.source_sha256,
    )?;
    validate_sha256(
        "interfaces.controller_proposal.source_sha256",
        &raw.interfaces.controller_proposal.source_sha256,
    )?;

    for (field, value) in [
        ("map.manifest_sha256", &raw.map.manifest_sha256),
        ("map.occupancy_yaml_sha256", &raw.map.occupancy_yaml_sha256),
        (
            "map.occupancy_image_sha256",
            &raw.map.occupancy_image_sha256,
        ),
        (
            "nav2_assets.base_parameters_sha256",
            &raw.nav2_assets.base_parameters_sha256,
        ),
        (
            "nav2_assets.tuning_parameters_sha256",
            &raw.nav2_assets.tuning_parameters_sha256,
        ),
        (
            "nav2_assets.integration_parameters_sha256",
            &raw.nav2_assets.integration_parameters_sha256,
        ),
        (
            "nav2_assets.navigate_to_pose_bt_sha256",
            &raw.nav2_assets.navigate_to_pose_bt_sha256,
        ),
        (
            "nav2_assets.navigate_through_poses_bt_sha256",
            &raw.nav2_assets.navigate_through_poses_bt_sha256,
        ),
    ] {
        validate_sha256(field, value)?;
    }
    exact_str("map.id", &raw.map.id, "warehouse_sim_map")?;
    exact_str("map.version", &raw.map.version, "0.1.0")?;

    exact_str("frames.map", &raw.frames.map, "map")?;
    exact_str("frames.odom", &raw.frames.odom, "odom")?;
    exact_str("frames.base", &raw.frames.base, "base_link")?;
    let expected_edges = [
        ("map", "odom", "rs-navigation-e2e"),
        ("odom", "base_link", "rs-navigation-e2e"),
    ];
    if raw.frames.tf_authorities.len() != expected_edges.len()
        || raw
            .frames
            .tf_authorities
            .iter()
            .zip(expected_edges)
            .any(|(actual, expected)| {
                (
                    actual.parent.as_str(),
                    actual.child.as_str(),
                    actual.writer.as_str(),
                ) != expected
            })
    {
        return Err(ManifestError::InvalidField {
            field: "frames.tf_authorities",
            detail: "expected exact map->odom->base_link ownership".to_owned(),
        });
    }

    exact_str(
        "ipc.namespace",
        &raw.ipc.namespace,
        "rs_control.navigation.motor_disabled.v1",
    )?;
    let mut layouts = BTreeSet::new();
    for (role, mode, capacity, freshness_ms) in EXPECTED_CHANNELS {
        let channel = raw.ipc.channels.get(role);
        if channel.mode != mode
            || channel.capacity != capacity
            || channel.freshness_ms != freshness_ms
        {
            return Err(ManifestError::InvalidField {
                field: "ipc.channels",
                detail: format!("unsafe policy for {role:?}"),
            });
        }
        if channel.layout_id.len() != 16
            || !channel
                .layout_id
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'_'))
        {
            return Err(ManifestError::InvalidField {
                field: "ipc.channels.layout_id",
                detail: "layout ID must be exactly 16 portable ASCII bytes".to_owned(),
            });
        }
        if !layouts.insert(channel.layout_id.as_str()) {
            return Err(ManifestError::InvalidField {
                field: "ipc.channels.layout_id",
                detail: "layout IDs must be unique".to_owned(),
            });
        }
    }

    exact_str(
        "safety.final_constraint_owner",
        &raw.safety.final_constraint_owner,
        "rs_control",
    )?;
    if raw.safety.adapter_can_issue_leases
        || raw.safety.motor_output != "forbidden"
        || raw.safety.can_transmit != "forbidden"
        || !raw.safety.simulation_only
        || raw.safety.approved_for_production
        || raw.safety.first_resource_run != "measurement_only"
    {
        return Err(ManifestError::InvalidField {
            field: "safety",
            detail: "motor-disabled safety policy changed".to_owned(),
        });
    }
    Ok(())
}

fn exact<T: Eq + fmt::Display>(
    field: &'static str,
    actual: T,
    expected: T,
) -> Result<(), ManifestError> {
    if actual == expected {
        Ok(())
    } else {
        Err(ManifestError::InvalidField {
            field,
            detail: format!("expected {expected}, got {actual}"),
        })
    }
}

fn exact_str(field: &'static str, actual: &str, expected: &str) -> Result<(), ManifestError> {
    if actual == expected {
        Ok(())
    } else {
        Err(ManifestError::InvalidField {
            field,
            detail: format!("expected {expected}, got {actual}"),
        })
    }
}

fn validate_sha256(field: &'static str, value: &str) -> Result<(), ManifestError> {
    if value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
    {
        Ok(())
    } else {
        Err(ManifestError::InvalidField {
            field,
            detail: "expected lowercase SHA-256".to_owned(),
        })
    }
}

fn decode_validated_sha256(value: &str) -> Sha256Digest {
    debug_assert!(validate_sha256("sha256", value).is_ok());
    let mut decoded = [0_u8; 32];
    for (index, pair) in value.as_bytes().chunks_exact(2).enumerate() {
        decoded[index] = (hex_nibble(pair[0]) << 4) | hex_nibble(pair[1]);
    }
    Sha256Digest(decoded)
}

const fn hex_nibble(value: u8) -> u8 {
    match value {
        b'0'..=b'9' => value - b'0',
        b'a'..=b'f' => value - b'a' + 10,
        _ => 0,
    }
}

fn read_bounded_regular_file(path: &Path, maximum: usize) -> Result<Vec<u8>, ManifestError> {
    let resolved = path.canonicalize().map_err(|source| ManifestError::Read {
        path: path.to_path_buf(),
        source,
    })?;
    let metadata = fs::metadata(&resolved).map_err(|source| ManifestError::Read {
        path: resolved.clone(),
        source,
    })?;
    if !metadata.is_file() {
        return Err(ManifestError::NotRegularFile(resolved));
    }
    let actual = usize::try_from(metadata.len()).unwrap_or(usize::MAX);
    if actual > maximum {
        return Err(ManifestError::TooLarge { actual, maximum });
    }
    fs::read(&resolved).map_err(|source| ManifestError::Read {
        path: resolved,
        source,
    })
}

/// Typed startup rejection for a navigation plugin manifest.
#[derive(Debug, Error)]
pub enum ManifestError {
    /// YAML exceeds the explicit startup bound.
    #[error("navigation manifest is {actual} bytes, maximum is {maximum}")]
    TooLarge {
        /// Observed bytes.
        actual: usize,
        /// Accepted bytes.
        maximum: usize,
    },
    /// File content is not UTF-8 YAML.
    #[error("navigation manifest is not UTF-8")]
    NotUtf8,
    /// YAML syntax or shape is invalid.
    #[error("invalid navigation manifest YAML: {0}")]
    Syntax(String),
    /// A fixed compatibility/safety field differs.
    #[error("invalid {field}: {detail}")]
    InvalidField {
        /// Rejected field.
        field: &'static str,
        /// Stable diagnostic detail.
        detail: String,
    },
    /// File cannot be resolved/read.
    #[error("failed to read {path}: {source}")]
    Read {
        /// Input path.
        path: PathBuf,
        /// Filesystem failure.
        #[source]
        source: std::io::Error,
    },
    /// Resolved input is not a regular file.
    #[error("not a regular file: {0}")]
    NotRegularFile(PathBuf),
    /// Asset role is unknown.
    #[error("unknown navigation asset role {0}")]
    UnknownAssetRole(String),
    /// Asset list is incomplete or contains an impossible role.
    #[error("navigation asset set mismatch; missing={missing:?}, unexpected={unexpected:?}")]
    AssetSet {
        /// Missing required roles.
        missing: Vec<AssetRole>,
        /// Unexpected roles.
        unexpected: Vec<AssetRole>,
    },
    /// Startup asset paths must be explicit absolute paths.
    #[error("navigation asset {role} path must be absolute: {path}")]
    RelativeAssetPath {
        /// Asset role.
        role: AssetRole,
        /// Rejected path.
        path: PathBuf,
    },
    /// Asset digest differs from the manifest.
    #[error("navigation asset {role} digest mismatch; expected {expected}, got {actual}")]
    AssetDigest {
        /// Asset role.
        role: AssetRole,
        /// Expected lowercase SHA-256.
        expected: String,
        /// Observed SHA-256.
        actual: Sha256Digest,
    },
    /// Both repositories do not contain the same manifest bytes.
    #[error("navigation plugin peer manifest is not byte-identical")]
    PeerMismatch,
}

#[cfg(test)]
mod tests {
    use super::*;

    const MANIFEST: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../../config/navigation/rs_control_navigation_plugin.yaml"
    ));

    #[test]
    fn checked_in_manifest_has_exact_contract() {
        let manifest = NavigationPluginManifest::parse(MANIFEST).unwrap();
        assert_eq!(manifest.schema_version(), 1);
        assert_eq!(manifest.plugin_id(), PLUGIN_ID);
        assert_eq!(manifest.profile_id(), PROFILE_ID);
        assert_eq!(manifest.map_id(), "warehouse_sim_map");
        assert_eq!(manifest.map_version(), "0.1.0");
        assert_eq!(manifest.map_frame(), "map");
        assert_eq!(
            manifest.map_manifest_digest().to_string(),
            "3d0ad97aeb90b265a62218294683b8cfd122f178044319c38b9c880f9e39428e"
        );
        assert_eq!(
            manifest.channel(ChannelRole::ControllerProposal),
            ChannelContract {
                layout_id: "rs-nav-prop-v001",
                mode: ChannelMode::Latest,
                // `middleware-shm` latest-v1 uses three physical seqlock
                // slots while exposing one logical newest value.
                capacity: 3,
                freshness_ms: 100,
            }
        );
    }

    #[test]
    fn unknown_and_unsafe_fields_fail_closed() {
        let unknown = MANIFEST.replace("safety:\n", "safety:\n  allow_hardware: true\n");
        assert!(matches!(
            NavigationPluginManifest::parse(&unknown),
            Err(ManifestError::Syntax(_))
        ));
        let unsafe_manifest = MANIFEST.replace("motor_output: forbidden", "motor_output: allowed");
        assert!(matches!(
            NavigationPluginManifest::parse(&unsafe_manifest),
            Err(ManifestError::InvalidField {
                field: "safety",
                ..
            })
        ));
    }

    #[test]
    fn descriptor_and_channel_identity_are_enforced() {
        let bad_descriptor = MANIFEST.replace(
            "757f3637560ad9e1e5c5ad51fdf519fab09ae00ff6a38bb2e3e96d27ab4fc44e",
            &"0".repeat(64),
        );
        assert!(matches!(
            NavigationPluginManifest::parse(&bad_descriptor),
            Err(ManifestError::InvalidField {
                field: "compatibility.protobuf_descriptor_sha256",
                ..
            })
        ));
        let duplicate_layout = MANIFEST.replace("rs-nav-health001", "rs-nav-prop-v001");
        assert!(NavigationPluginManifest::parse(&duplicate_layout).is_err());
    }

    #[test]
    fn peer_copy_and_complete_asset_set_are_required() {
        let directory = tempfile::tempdir().unwrap();
        let local = directory.path().join("local.yaml");
        let peer = directory.path().join("peer.yaml");
        fs::write(&local, MANIFEST).unwrap();
        fs::write(&peer, MANIFEST).unwrap();
        assert_eq!(
            NavigationPluginManifest::verify_peer_copy(&local, &peer).unwrap(),
            Sha256Digest::of(MANIFEST.as_bytes())
        );
        fs::write(
            &peer,
            MANIFEST.replace("profile_id:", "profile_id: changed #"),
        )
        .unwrap();
        assert!(matches!(
            NavigationPluginManifest::verify_peer_copy(&local, &peer),
            Err(ManifestError::PeerMismatch)
        ));

        let manifest = NavigationPluginManifest::parse(MANIFEST).unwrap();
        assert!(matches!(
            manifest.verify_assets(&BTreeMap::new()),
            Err(ManifestError::AssetSet { .. })
        ));
    }
}
