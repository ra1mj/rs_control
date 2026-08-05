//! Backend-neutral navigation plugin values.

use arrayvec::ArrayVec;

use crate::{
    AsciiId, FrameId, Header, Id128, Pose3, ResourceLease, SemanticError, numeric::require_finite,
};

/// Maximum typed navigation fault codes in one health snapshot.
pub const MAX_NAVIGATION_FAULTS: usize = 16;
/// Stable map and behavior-tree identifier length.
pub const MAX_NAVIGATION_ID_LEN: usize = 64;

/// Stable navigation map identity.
pub type NavigationMapId = AsciiId<MAX_NAVIGATION_ID_LEN>;
/// Stable navigation map version.
pub type NavigationMapVersion = AsciiId<MAX_NAVIGATION_ID_LEN>;
/// Allowlisted navigation behavior-tree identity.
pub type NavigationBehaviorTreeId = AsciiId<MAX_NAVIGATION_ID_LEN>;

/// Validated nonzero SHA-256 identity.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct Sha256Identity([u8; 32]);

impl Sha256Identity {
    /// Constructs a digest while rejecting the all-zero unspecified sentinel.
    pub fn new(value: [u8; 32], field: &'static str) -> Result<Self, SemanticError> {
        if value == [0; 32] {
            return Err(SemanticError::AllZeroIdentity { field });
        }
        Ok(Self(value))
    }

    /// Parses exact wire bytes.
    pub fn from_slice(value: &[u8], field: &'static str) -> Result<Self, SemanticError> {
        let value: [u8; 32] = value
            .try_into()
            .map_err(|_| SemanticError::DimensionMismatch {
                field,
                expected: 32,
                actual: value.len(),
            })?;
        Self::new(value, field)
    }

    /// Returns exact digest bytes.
    #[must_use]
    pub const fn as_bytes(self) -> [u8; 32] {
        self.0
    }
}

/// Exact map artifact and coordinate-frame identity.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NavigationMapIdentity {
    map_id: NavigationMapId,
    version: NavigationMapVersion,
    artifact_sha256: Sha256Identity,
    frame_id: FrameId,
}

impl NavigationMapIdentity {
    /// Constructs one immutable map identity.
    #[must_use]
    pub const fn new(
        map_id: NavigationMapId,
        version: NavigationMapVersion,
        artifact_sha256: Sha256Identity,
        frame_id: FrameId,
    ) -> Self {
        Self {
            map_id,
            version,
            artifact_sha256,
            frame_id,
        }
    }

    /// Returns the stable map ID.
    #[must_use]
    pub const fn map_id(self) -> NavigationMapId {
        self.map_id
    }

    /// Returns the stable map version.
    #[must_use]
    pub const fn version(self) -> NavigationMapVersion {
        self.version
    }

    /// Returns the bound artifact digest.
    #[must_use]
    pub const fn artifact_sha256(self) -> Sha256Identity {
        self.artifact_sha256
    }

    /// Returns the goal/map coordinate frame.
    #[must_use]
    pub const fn frame_id(self) -> FrameId {
        self.frame_id
    }
}

/// Concrete `navigate_to_pose_v1` goal payload.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavigateToPoseGoal {
    /// Exact active map identity.
    pub map: NavigationMapIdentity,
    /// Target pose expressed in `map.frame_id`.
    pub target: Pose3,
    /// Allowlisted behavior-tree identity, not an arbitrary path.
    pub behavior_tree_id: NavigationBehaviorTreeId,
}

/// Concrete `navigate_to_pose_v1` feedback payload.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavigateToPoseFeedback {
    /// Current pose expressed in the goal map frame.
    pub current_pose: Pose3,
    /// Elapsed navigation time.
    pub navigation_time_ns: u64,
    /// Estimated remaining time when Nav2 supplies it.
    pub estimated_time_remaining_ns: Option<u64>,
    /// Recovery count observed for this goal.
    pub recoveries: u32,
    /// Nonnegative remaining distance in metres.
    pub distance_remaining_m: f64,
}

impl NavigateToPoseFeedback {
    /// Validates finite, nonnegative remaining distance.
    pub fn validate(self) -> Result<Self, SemanticError> {
        require_finite(
            self.distance_remaining_m,
            "navigate_to_pose.feedback.distance_remaining_m",
        )?;
        if self.distance_remaining_m < 0.0 {
            return Err(SemanticError::OutsideRange {
                field: "navigate_to_pose.feedback.distance_remaining_m",
            });
        }
        Ok(self)
    }
}

/// Concrete `navigate_to_pose_v1` terminal result payload.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NavigateToPoseResult {
    /// Exact map used for the completed action.
    pub map: NavigationMapIdentity,
    /// Last correlated pose evidence.
    pub final_pose: Pose3,
    /// Total navigation time.
    pub navigation_time_ns: u64,
    /// Total recovery count.
    pub recoveries: u32,
}

/// Why the bridge currently has no command authority assertion.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NavigationAuthorityUnavailable {
    /// No chassis lease exists.
    NoLease,
    /// The previously observed lease expired.
    LeaseExpired,
    /// The mission has no active correlated navigation action.
    MissionInactive,
    /// Plugin/IPC readiness is unavailable.
    PluginUnavailable,
}

/// Explicit bridge-visible authority snapshot; it never grants authority.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NavigationAuthority {
    /// No proposal may be emitted.
    Unavailable(NavigationAuthorityUnavailable),
    /// Existing local authority may be asserted for one optional active goal.
    Available {
        /// Lease created by the local authority owner.
        lease: ResourceLease,
        /// Correlated active navigation goal, if one is executing.
        active_command_id: Option<Id128>,
    },
}

/// Source/boot ordered authority state sent to the isolated bridge.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NavigationAuthorityState {
    /// Authority publisher metadata.
    pub header: Header,
    /// Explicit available/unavailable state.
    pub authority: NavigationAuthority,
}

impl NavigationAuthorityState {
    /// Checks an available lease at the local shared monotonic time.
    pub fn validate_at(self, now_monotonic_ns: u64) -> Result<Self, SemanticError> {
        if let NavigationAuthority::Available { lease, .. } = self.authority {
            lease.validate_at(now_monotonic_ns)?;
        }
        Ok(self)
    }
}

/// Lifecycle of the isolated ROS navigation bridge.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NavigationComponentState {
    /// Process is absent/stopped.
    Stopped,
    /// Process is starting and not eligible.
    Starting,
    /// Process and required links are ready.
    Ready,
    /// Process or a required link failed.
    Failed,
}

/// Exact eligibility gates observed by the bridge.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct NavigationReadiness {
    /// Manifest and assets match.
    pub manifest: bool,
    /// Nav2 action server is ready.
    pub action_server: bool,
    /// Required lifecycle nodes are active.
    pub nav2_lifecycle: bool,
    /// Current exact map is loaded.
    pub map: bool,
    /// Localization and TF ownership are ready.
    pub localization: bool,
    /// Shared-memory peers are attached.
    pub ipc: bool,
}

impl NavigationReadiness {
    /// Returns true only when every non-authority runtime gate is ready.
    #[must_use]
    pub const fn all_ready(self) -> bool {
        self.manifest
            && self.action_server
            && self.nav2_lifecycle
            && self.map
            && self.localization
            && self.ipc
    }
}

/// Saturating counters reported by the navigation bridge.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct NavigationCounters {
    /// Accepted controller proposals.
    pub accepted_proposals: u64,
    /// Proposals rejected for authority.
    pub authority_rejections: u64,
    /// Proposals rejected for freshness/order.
    pub freshness_rejections: u64,
    /// Proposals dropped because the latest slot was overwritten.
    pub overwritten_proposals: u64,
    /// Accepted action goals.
    pub accepted_goals: u64,
    /// Terminal action results.
    pub terminal_results: u64,
}

/// Bounded bridge readiness and diagnostic snapshot.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NavigationHealth {
    /// Health publisher metadata.
    pub header: Header,
    /// Supervisor-owned nonzero adapter process epoch.
    pub process_epoch: u64,
    /// Exact shared manifest identity.
    pub manifest_sha256: Sha256Identity,
    /// Process/link lifecycle.
    pub component_state: NavigationComponentState,
    /// Detailed readiness gates.
    pub readiness: NavigationReadiness,
    /// Machine-readable typed fault codes.
    pub fault_codes: ArrayVec<u32, MAX_NAVIGATION_FAULTS>,
    /// Saturating bridge counters.
    pub counters: NavigationCounters,
}

impl NavigationHealth {
    /// Constructs a health snapshot with nonzero process epoch and fault codes.
    pub fn new(
        header: Header,
        process_epoch: u64,
        manifest_sha256: Sha256Identity,
        component_state: NavigationComponentState,
        readiness: NavigationReadiness,
        fault_codes: ArrayVec<u32, MAX_NAVIGATION_FAULTS>,
        counters: NavigationCounters,
    ) -> Result<Self, SemanticError> {
        if process_epoch == 0 {
            return Err(SemanticError::ZeroCount {
                field: "navigation_health.process_epoch",
            });
        }
        if fault_codes.contains(&0) {
            return Err(SemanticError::ZeroCount {
                field: "navigation_health.fault_codes",
            });
        }
        Ok(Self {
            header,
            process_epoch,
            manifest_sha256,
            component_state,
            readiness,
            fault_codes,
            counters,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        ClockState, ClockStatus, DataQuality, HeaderFields, ResourceLeaseFields, UnitQuaternion,
        Vector3,
    };

    fn header() -> Header {
        Header::new(HeaderFields {
            source_id: AsciiId::parse("rs-navigation-e2e").unwrap(),
            boot_id: Id128::new([1; 16]).unwrap(),
            sequence: 1,
            source_monotonic_ns: 100,
            publish_unix_ns: None,
            acquisition_timestamp: None,
            valid_for_ns: 100,
            frame_id: None,
            clock_status: ClockStatus {
                state: ClockState::Local,
                uncertainty_ns: 0,
            },
            quality: DataQuality::Valid,
        })
        .unwrap()
    }

    fn pose() -> Pose3 {
        Pose3 {
            position_m: Vector3::new(1.0, 2.0, 0.0).unwrap(),
            orientation: UnitQuaternion::new(0.0, 0.0, 0.0, 1.0).unwrap(),
        }
    }

    #[test]
    fn map_digest_and_feedback_bounds_are_explicit() {
        assert!(Sha256Identity::new([0; 32], "map").is_err());
        let feedback = NavigateToPoseFeedback {
            current_pose: pose(),
            navigation_time_ns: 10,
            estimated_time_remaining_ns: None,
            recoveries: 0,
            distance_remaining_m: 1.0,
        };
        assert!(feedback.validate().is_ok());
        assert!(
            NavigateToPoseFeedback {
                distance_remaining_m: -1.0,
                ..feedback
            }
            .validate()
            .is_err()
        );
    }

    #[test]
    fn authority_never_refreshes_an_expired_lease() {
        let lease = ResourceLease::new(ResourceLeaseFields {
            resource_id: AsciiId::parse("chassis").unwrap(),
            owner_id: AsciiId::parse("mission").unwrap(),
            lease_id: Id128::new([2; 16]).unwrap(),
            session_epoch: 1,
            expires_monotonic_ns: 200,
        })
        .unwrap();
        let state = NavigationAuthorityState {
            header: header(),
            authority: NavigationAuthority::Available {
                lease,
                active_command_id: Some(Id128::new([3; 16]).unwrap()),
            },
        };
        assert!(state.validate_at(199).is_ok());
        assert!(state.validate_at(200).is_err());
    }

    #[test]
    fn health_requires_epoch_and_nonzero_fault_codes() {
        let digest = Sha256Identity::new([4; 32], "manifest").unwrap();
        assert!(
            NavigationHealth::new(
                header(),
                0,
                digest,
                NavigationComponentState::Starting,
                NavigationReadiness::default(),
                ArrayVec::new(),
                NavigationCounters::default(),
            )
            .is_err()
        );
        let mut faults = ArrayVec::new();
        faults.push(0);
        assert!(
            NavigationHealth::new(
                header(),
                1,
                digest,
                NavigationComponentState::Failed,
                NavigationReadiness::default(),
                faults,
                NavigationCounters::default(),
            )
            .is_err()
        );
    }
}
