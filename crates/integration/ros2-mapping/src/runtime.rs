//! Pure runtime-evidence gate for an external ROS 2 adapter and ros2dds.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{Authorization, Ros2DdsMode, Route, RouteRegistry, SUPPORTED_ROS2DDS_BUILD_ID};

/// Observed lifecycle of one external runtime component or link.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ComponentState {
    /// Process/link is known absent.
    Stopped,
    /// A new process epoch is starting and has not proven readiness.
    Starting,
    /// The component is alive and its owning readiness check passed.
    Ready,
    /// The component or link reported a failure.
    Failed,
}

/// Exact runtime identities reported by a ready adapter.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RuntimeIdentity {
    /// Checked-in profile ID.
    pub profile_id: String,
    /// Runtime Protobuf descriptor identity.
    pub protobuf_descriptor_sha256: String,
    /// Canonical route-manifest identity.
    pub manifest_fingerprint: String,
    /// Native/Python Zenoh compatibility version.
    pub zenoh_version: String,
    /// Standalone ros2dds release version.
    pub ros2dds_version: String,
    /// Exact ros2dds executable build identity.
    pub ros2dds_build_id: String,
    /// Standalone/plugin deployment mode.
    pub ros2dds_mode: Ros2DdsMode,
}

/// One non-realtime observation of the external ROS integration path.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RuntimeEvidence {
    /// Supervisor-owned, strictly increasing adapter process epoch.
    pub epoch: u64,
    /// Adapter process lifecycle.
    pub adapter: ComponentState,
    /// Identity evidence, required only when the adapter reports ready.
    pub identity: Option<RuntimeIdentity>,
    /// Native Protobuf/Zenoh side connectivity.
    pub native_zenoh: ComponentState,
    /// Local ROS graph/DDS side availability.
    pub ros_graph: ComponentState,
    /// Optional remote DDS-island ros2dds path availability.
    pub ros2dds: ComponentState,
}

/// Fail-closed reason retained in a readiness report.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RuntimeFailure {
    /// No adapter process has completed startup.
    AdapterUnavailable,
    /// Native Zenoh connectivity is absent or failed.
    NativeZenohUnavailable,
    /// The local ROS graph/DDS path is absent or failed.
    RosGraphUnavailable,
    /// A configured remote DDS island lost its standalone ros2dds path.
    Ros2DdsUnavailable,
    /// A report from a prior process epoch was presented after restart.
    StaleEvidence,
}

/// Aggregate runtime state after identity and path validation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RuntimeStatus {
    /// Every required external path is ready.
    Ready,
    /// At least one path is closed.
    Unavailable(RuntimeFailure),
}

/// Admission result for one route in one validated process epoch.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RouteAdmission {
    /// Read-only stream/query path is externally available.
    Available,
    /// External transport is ready, but local lease/deadline validation remains mandatory.
    LocalValidationRequired,
    /// The external path is closed.
    Closed(RuntimeFailure),
}

/// Epoch-bound report. Fields stay private so admission goes through [`RuntimeGate`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RuntimeReport {
    epoch: u64,
    status: RuntimeStatus,
}

impl RuntimeReport {
    /// Returns the aggregate path status for diagnostics.
    #[must_use]
    pub const fn status(self) -> RuntimeStatus {
        self.status
    }

    /// Returns the process epoch bound to this report.
    #[must_use]
    pub const fn epoch(self) -> u64 {
        self.epoch
    }
}

/// Supervisor-owned guard that invalidates prior readiness on every restart.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RuntimeGate {
    current_epoch: Option<u64>,
}

impl RuntimeGate {
    /// Creates a gate with no accepted process epoch.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            current_epoch: None,
        }
    }

    /// Begins a fresh adapter epoch and immediately invalidates older reports.
    pub fn begin_epoch(&mut self, epoch: u64) -> Result<(), RuntimeError> {
        if epoch == 0 {
            return Err(RuntimeError::ZeroEpoch);
        }
        if self.current_epoch.is_some_and(|current| epoch <= current) {
            return Err(RuntimeError::NonIncreasingEpoch);
        }
        self.current_epoch = Some(epoch);
        Ok(())
    }

    /// Validates current-epoch identity and computes a fail-closed path report.
    pub fn evaluate(
        &self,
        registry: &RouteRegistry,
        evidence: &RuntimeEvidence,
    ) -> Result<RuntimeReport, RuntimeError> {
        let epoch = self.current_epoch.ok_or(RuntimeError::NoCurrentEpoch)?;
        if evidence.epoch != epoch {
            return Err(RuntimeError::EpochMismatch);
        }
        if evidence.adapter != ComponentState::Ready {
            return Ok(RuntimeReport {
                epoch,
                status: RuntimeStatus::Unavailable(RuntimeFailure::AdapterUnavailable),
            });
        }
        let identity = evidence
            .identity
            .as_ref()
            .ok_or(RuntimeError::MissingIdentity)?;
        validate_identity(registry, identity)?;
        let status = if evidence.native_zenoh != ComponentState::Ready {
            RuntimeStatus::Unavailable(RuntimeFailure::NativeZenohUnavailable)
        } else if evidence.ros_graph != ComponentState::Ready {
            RuntimeStatus::Unavailable(RuntimeFailure::RosGraphUnavailable)
        } else if !registry.manifest().deployment.ros2dds_hosts.is_empty()
            && evidence.ros2dds != ComponentState::Ready
        {
            RuntimeStatus::Unavailable(RuntimeFailure::Ros2DdsUnavailable)
        } else {
            RuntimeStatus::Ready
        };
        Ok(RuntimeReport { epoch, status })
    }

    /// Evaluates one route while rejecting reports invalidated by restart.
    #[must_use]
    pub fn admission(&self, report: RuntimeReport, route: &Route) -> RouteAdmission {
        if self.current_epoch != Some(report.epoch) {
            return RouteAdmission::Closed(RuntimeFailure::StaleEvidence);
        }
        match report.status {
            RuntimeStatus::Unavailable(reason) => RouteAdmission::Closed(reason),
            RuntimeStatus::Ready if route.authorization == Authorization::LocalLeaseRequired => {
                RouteAdmission::LocalValidationRequired
            }
            RuntimeStatus::Ready => RouteAdmission::Available,
        }
    }
}

fn validate_identity(
    registry: &RouteRegistry,
    identity: &RuntimeIdentity,
) -> Result<(), RuntimeError> {
    let profile = &registry.manifest().profile;
    for (field, matches) in [
        ("profile_id", identity.profile_id == profile.id),
        (
            "protobuf_descriptor_sha256",
            identity.protobuf_descriptor_sha256 == profile.protobuf_descriptor_sha256,
        ),
        (
            "manifest_fingerprint",
            identity.manifest_fingerprint == registry.fingerprint(),
        ),
        (
            "zenoh_version",
            identity.zenoh_version == profile.zenoh_version,
        ),
        (
            "ros2dds_version",
            identity.ros2dds_version == profile.ros2dds_version,
        ),
        (
            "ros2dds_build_id",
            identity.ros2dds_build_id == profile.ros2dds_build_id
                && identity.ros2dds_build_id == SUPPORTED_ROS2DDS_BUILD_ID,
        ),
        (
            "ros2dds_mode",
            identity.ros2dds_mode == profile.ros2dds_mode,
        ),
    ] {
        if !matches {
            return Err(RuntimeError::IdentityMismatch { field });
        }
    }
    Ok(())
}

/// Runtime evidence or epoch validation failure.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum RuntimeError {
    /// Epoch zero is reserved as unspecified.
    #[error("runtime epoch must be nonzero")]
    ZeroEpoch,
    /// Restart epochs must strictly increase.
    #[error("runtime epoch must strictly increase")]
    NonIncreasingEpoch,
    /// Evaluation was attempted before a supervisor epoch started.
    #[error("no runtime epoch has started")]
    NoCurrentEpoch,
    /// Evidence belongs to a different process epoch.
    #[error("runtime evidence epoch does not match the current process")]
    EpochMismatch,
    /// A ready adapter omitted build/profile identity evidence.
    #[error("ready adapter omitted runtime identity")]
    MissingIdentity,
    /// One identity field does not match the checked-in profile.
    #[error("runtime identity field {field} does not match the checked-in profile")]
    IdentityMismatch {
        /// Stable rejected field name.
        field: &'static str,
    },
}
