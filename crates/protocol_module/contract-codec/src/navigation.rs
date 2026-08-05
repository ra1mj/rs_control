//! Navigation plugin semantic/Protobuf conversions.

use arrayvec::ArrayVec;
use message_model::{
    AsciiId, Id128, MAX_NAVIGATION_FAULTS, NavigateToPoseFeedback, NavigateToPoseGoal,
    NavigateToPoseResult, NavigationAuthority, NavigationAuthorityState,
    NavigationAuthorityUnavailable, NavigationComponentState, NavigationCounters, NavigationHealth,
    NavigationMapIdentity, NavigationReadiness, Sha256Identity,
};
use prost::Message;
use proto_generated::robot::v1 as wire;

use crate::{
    CodecError, header_from_wire, header_to_wire, pose3_from_wire, pose3_to_wire,
    resource_lease_from_wire, resource_lease_to_wire,
};

/// Encodes one exact map identity.
#[must_use]
pub fn encode_navigation_map_identity(value: NavigationMapIdentity) -> Vec<u8> {
    navigation_map_identity_to_wire(value).encode_to_vec()
}

/// Decodes and validates one exact map identity.
pub fn decode_navigation_map_identity(bytes: &[u8]) -> Result<NavigationMapIdentity, CodecError> {
    navigation_map_identity_from_wire(wire::NavigationMapIdentity::decode(bytes)?)
}

/// Converts semantic map identity to wire.
#[must_use]
pub fn navigation_map_identity_to_wire(
    value: NavigationMapIdentity,
) -> wire::NavigationMapIdentity {
    wire::NavigationMapIdentity {
        map_id: value.map_id().as_str().to_owned(),
        version: value.version().as_str().to_owned(),
        artifact_sha256: value.artifact_sha256().as_bytes().to_vec(),
        frame_id: value.frame_id().as_str().to_owned(),
    }
}

/// Converts and validates wire map identity.
pub fn navigation_map_identity_from_wire(
    value: wire::NavigationMapIdentity,
) -> Result<NavigationMapIdentity, CodecError> {
    Ok(NavigationMapIdentity::new(
        AsciiId::parse(&value.map_id)?,
        AsciiId::parse(&value.version)?,
        Sha256Identity::from_slice(&value.artifact_sha256, "navigation_map.artifact_sha256")?,
        AsciiId::parse(&value.frame_id)?,
    ))
}

/// Encodes a concrete navigate-to-pose goal payload.
#[must_use]
pub fn encode_navigate_to_pose_goal(value: NavigateToPoseGoal) -> Vec<u8> {
    navigate_to_pose_goal_to_wire(value).encode_to_vec()
}

/// Decodes and validates a concrete navigate-to-pose goal payload.
pub fn decode_navigate_to_pose_goal(bytes: &[u8]) -> Result<NavigateToPoseGoal, CodecError> {
    navigate_to_pose_goal_from_wire(wire::NavigateToPoseGoal::decode(bytes)?)
}

/// Converts semantic navigate-to-pose goal to wire.
#[must_use]
pub fn navigate_to_pose_goal_to_wire(value: NavigateToPoseGoal) -> wire::NavigateToPoseGoal {
    wire::NavigateToPoseGoal {
        map: Some(navigation_map_identity_to_wire(value.map)),
        target: Some(pose3_to_wire(value.target)),
        behavior_tree_id: value.behavior_tree_id.as_str().to_owned(),
    }
}

/// Converts and validates wire navigate-to-pose goal.
pub fn navigate_to_pose_goal_from_wire(
    value: wire::NavigateToPoseGoal,
) -> Result<NavigateToPoseGoal, CodecError> {
    Ok(NavigateToPoseGoal {
        map: navigation_map_identity_from_wire(
            value
                .map
                .ok_or(CodecError::MissingField("navigation_goal.map"))?,
        )?,
        target: pose3_from_wire(
            value
                .target
                .ok_or(CodecError::MissingField("navigation_goal.target"))?,
        )?,
        behavior_tree_id: AsciiId::parse(&value.behavior_tree_id)?,
    })
}

/// Encodes validated navigate-to-pose feedback.
pub fn encode_navigate_to_pose_feedback(
    value: NavigateToPoseFeedback,
) -> Result<Vec<u8>, CodecError> {
    Ok(navigate_to_pose_feedback_to_wire(value.validate()?).encode_to_vec())
}

/// Decodes and validates navigate-to-pose feedback.
pub fn decode_navigate_to_pose_feedback(
    bytes: &[u8],
) -> Result<NavigateToPoseFeedback, CodecError> {
    navigate_to_pose_feedback_from_wire(wire::NavigateToPoseFeedback::decode(bytes)?)
}

/// Converts semantic navigate-to-pose feedback to wire.
#[must_use]
pub fn navigate_to_pose_feedback_to_wire(
    value: NavigateToPoseFeedback,
) -> wire::NavigateToPoseFeedback {
    wire::NavigateToPoseFeedback {
        current_pose: Some(pose3_to_wire(value.current_pose)),
        navigation_time_ns: value.navigation_time_ns,
        estimated_time_remaining_ns: value.estimated_time_remaining_ns,
        recoveries: value.recoveries,
        distance_remaining_m: value.distance_remaining_m,
    }
}

/// Converts and validates wire navigate-to-pose feedback.
pub fn navigate_to_pose_feedback_from_wire(
    value: wire::NavigateToPoseFeedback,
) -> Result<NavigateToPoseFeedback, CodecError> {
    NavigateToPoseFeedback {
        current_pose: pose3_from_wire(
            value
                .current_pose
                .ok_or(CodecError::MissingField("navigation_feedback.current_pose"))?,
        )?,
        navigation_time_ns: value.navigation_time_ns,
        estimated_time_remaining_ns: value.estimated_time_remaining_ns,
        recoveries: value.recoveries,
        distance_remaining_m: value.distance_remaining_m,
    }
    .validate()
    .map_err(CodecError::from)
}

/// Encodes a concrete navigate-to-pose terminal result.
#[must_use]
pub fn encode_navigate_to_pose_result(value: NavigateToPoseResult) -> Vec<u8> {
    navigate_to_pose_result_to_wire(value).encode_to_vec()
}

/// Decodes and validates a concrete navigate-to-pose terminal result.
pub fn decode_navigate_to_pose_result(bytes: &[u8]) -> Result<NavigateToPoseResult, CodecError> {
    navigate_to_pose_result_from_wire(wire::NavigateToPoseResult::decode(bytes)?)
}

/// Converts semantic navigate-to-pose result to wire.
#[must_use]
pub fn navigate_to_pose_result_to_wire(value: NavigateToPoseResult) -> wire::NavigateToPoseResult {
    wire::NavigateToPoseResult {
        map: Some(navigation_map_identity_to_wire(value.map)),
        final_pose: Some(pose3_to_wire(value.final_pose)),
        navigation_time_ns: value.navigation_time_ns,
        recoveries: value.recoveries,
    }
}

/// Converts and validates wire navigate-to-pose result.
pub fn navigate_to_pose_result_from_wire(
    value: wire::NavigateToPoseResult,
) -> Result<NavigateToPoseResult, CodecError> {
    Ok(NavigateToPoseResult {
        map: navigation_map_identity_from_wire(
            value
                .map
                .ok_or(CodecError::MissingField("navigation_result.map"))?,
        )?,
        final_pose: pose3_from_wire(
            value
                .final_pose
                .ok_or(CodecError::MissingField("navigation_result.final_pose"))?,
        )?,
        navigation_time_ns: value.navigation_time_ns,
        recoveries: value.recoveries,
    })
}

/// Encodes an authority snapshot only while an available lease remains valid.
pub fn encode_navigation_authority_state(
    value: NavigationAuthorityState,
    now_monotonic_ns: u64,
) -> Result<Vec<u8>, CodecError> {
    Ok(navigation_authority_state_to_wire(value.validate_at(now_monotonic_ns)?).encode_to_vec())
}

/// Decodes an authority snapshot and validates a present lease locally.
pub fn decode_navigation_authority_state(
    bytes: &[u8],
    now_monotonic_ns: u64,
) -> Result<NavigationAuthorityState, CodecError> {
    navigation_authority_state_from_wire(
        wire::NavigationAuthorityState::decode(bytes)?,
        now_monotonic_ns,
    )
}

/// Converts a semantic authority snapshot to wire.
#[must_use]
pub fn navigation_authority_state_to_wire(
    value: NavigationAuthorityState,
) -> wire::NavigationAuthorityState {
    let (lease, active_command_id, unavailable) = match value.authority {
        NavigationAuthority::Unavailable(reason) => (
            None,
            Vec::new(),
            navigation_authority_unavailable_to_wire(reason) as i32,
        ),
        NavigationAuthority::Available {
            lease,
            active_command_id,
        } => (
            Some(resource_lease_to_wire(&lease)),
            active_command_id.map_or_else(Vec::new, |id| id.as_bytes().to_vec()),
            0,
        ),
    };
    wire::NavigationAuthorityState {
        header: Some(header_to_wire(&value.header)),
        lease,
        active_command_id,
        unavailable,
    }
}

/// Converts and validates a wire authority snapshot.
pub fn navigation_authority_state_from_wire(
    value: wire::NavigationAuthorityState,
    now_monotonic_ns: u64,
) -> Result<NavigationAuthorityState, CodecError> {
    let authority = match value.lease {
        Some(lease) => {
            if value.unavailable != 0 {
                return Err(CodecError::InvalidField {
                    field: "navigation_authority.unavailable",
                    reason: "available state cannot also carry an unavailable reason",
                });
            }
            NavigationAuthority::Available {
                lease: resource_lease_from_wire(lease, now_monotonic_ns)?,
                active_command_id: if value.active_command_id.is_empty() {
                    None
                } else {
                    Some(Id128::from_slice(&value.active_command_id)?)
                },
            }
        }
        None => {
            if !value.active_command_id.is_empty() {
                return Err(CodecError::InvalidField {
                    field: "navigation_authority.active_command_id",
                    reason: "unavailable state cannot carry an active command",
                });
            }
            NavigationAuthority::Unavailable(navigation_authority_unavailable_from_wire(
                value.unavailable,
            )?)
        }
    };
    Ok(NavigationAuthorityState {
        header: header_from_wire(
            value
                .header
                .ok_or(CodecError::MissingField("navigation_authority.header"))?,
        )?,
        authority,
    })
}

/// Encodes one navigation health snapshot.
#[must_use]
pub fn encode_navigation_health(value: &NavigationHealth) -> Vec<u8> {
    navigation_health_to_wire(value).encode_to_vec()
}

/// Decodes and validates one bounded navigation health snapshot.
pub fn decode_navigation_health(bytes: &[u8]) -> Result<NavigationHealth, CodecError> {
    navigation_health_from_wire(wire::NavigationHealth::decode(bytes)?)
}

/// Converts semantic navigation health to wire.
#[must_use]
pub fn navigation_health_to_wire(value: &NavigationHealth) -> wire::NavigationHealth {
    wire::NavigationHealth {
        header: Some(header_to_wire(&value.header)),
        process_epoch: value.process_epoch,
        manifest_sha256: value.manifest_sha256.as_bytes().to_vec(),
        component_state: navigation_component_state_to_wire(value.component_state) as i32,
        readiness: Some(navigation_readiness_to_wire(value.readiness)),
        fault_codes: value.fault_codes.to_vec(),
        counters: Some(navigation_counters_to_wire(value.counters)),
    }
}

/// Converts and validates wire navigation health.
pub fn navigation_health_from_wire(
    value: wire::NavigationHealth,
) -> Result<NavigationHealth, CodecError> {
    if value.fault_codes.len() > MAX_NAVIGATION_FAULTS {
        return Err(CodecError::TooMany {
            field: "navigation_health.fault_codes",
            maximum: MAX_NAVIGATION_FAULTS,
            actual: value.fault_codes.len(),
        });
    }
    let fault_codes = value
        .fault_codes
        .into_iter()
        .collect::<ArrayVec<_, MAX_NAVIGATION_FAULTS>>();
    NavigationHealth::new(
        header_from_wire(
            value
                .header
                .ok_or(CodecError::MissingField("navigation_health.header"))?,
        )?,
        value.process_epoch,
        Sha256Identity::from_slice(&value.manifest_sha256, "navigation_health.manifest_sha256")?,
        navigation_component_state_from_wire(value.component_state)?,
        navigation_readiness_from_wire(
            value
                .readiness
                .ok_or(CodecError::MissingField("navigation_health.readiness"))?,
        ),
        fault_codes,
        navigation_counters_from_wire(
            value
                .counters
                .ok_or(CodecError::MissingField("navigation_health.counters"))?,
        ),
    )
    .map_err(CodecError::from)
}

fn navigation_authority_unavailable_to_wire(
    value: NavigationAuthorityUnavailable,
) -> wire::NavigationAuthorityUnavailable {
    match value {
        NavigationAuthorityUnavailable::NoLease => wire::NavigationAuthorityUnavailable::NoLease,
        NavigationAuthorityUnavailable::LeaseExpired => {
            wire::NavigationAuthorityUnavailable::LeaseExpired
        }
        NavigationAuthorityUnavailable::MissionInactive => {
            wire::NavigationAuthorityUnavailable::MissionInactive
        }
        NavigationAuthorityUnavailable::PluginUnavailable => {
            wire::NavigationAuthorityUnavailable::PluginUnavailable
        }
    }
}

fn navigation_authority_unavailable_from_wire(
    value: i32,
) -> Result<NavigationAuthorityUnavailable, CodecError> {
    match wire::NavigationAuthorityUnavailable::try_from(value) {
        Ok(wire::NavigationAuthorityUnavailable::NoLease) => {
            Ok(NavigationAuthorityUnavailable::NoLease)
        }
        Ok(wire::NavigationAuthorityUnavailable::LeaseExpired) => {
            Ok(NavigationAuthorityUnavailable::LeaseExpired)
        }
        Ok(wire::NavigationAuthorityUnavailable::MissionInactive) => {
            Ok(NavigationAuthorityUnavailable::MissionInactive)
        }
        Ok(wire::NavigationAuthorityUnavailable::PluginUnavailable) => {
            Ok(NavigationAuthorityUnavailable::PluginUnavailable)
        }
        Ok(wire::NavigationAuthorityUnavailable::Unspecified) | Err(_) => {
            Err(CodecError::UnknownEnum {
                field: "navigation_authority.unavailable",
                value,
            })
        }
    }
}

fn navigation_component_state_to_wire(
    value: NavigationComponentState,
) -> wire::NavigationComponentState {
    match value {
        NavigationComponentState::Stopped => wire::NavigationComponentState::Stopped,
        NavigationComponentState::Starting => wire::NavigationComponentState::Starting,
        NavigationComponentState::Ready => wire::NavigationComponentState::Ready,
        NavigationComponentState::Failed => wire::NavigationComponentState::Failed,
    }
}

fn navigation_component_state_from_wire(
    value: i32,
) -> Result<NavigationComponentState, CodecError> {
    match wire::NavigationComponentState::try_from(value) {
        Ok(wire::NavigationComponentState::Stopped) => Ok(NavigationComponentState::Stopped),
        Ok(wire::NavigationComponentState::Starting) => Ok(NavigationComponentState::Starting),
        Ok(wire::NavigationComponentState::Ready) => Ok(NavigationComponentState::Ready),
        Ok(wire::NavigationComponentState::Failed) => Ok(NavigationComponentState::Failed),
        Ok(wire::NavigationComponentState::Unspecified) | Err(_) => Err(CodecError::UnknownEnum {
            field: "navigation_health.component_state",
            value,
        }),
    }
}

fn navigation_readiness_to_wire(value: NavigationReadiness) -> wire::NavigationReadiness {
    wire::NavigationReadiness {
        manifest: value.manifest,
        action_server: value.action_server,
        nav2_lifecycle: value.nav2_lifecycle,
        map: value.map,
        localization: value.localization,
        ipc: value.ipc,
    }
}

fn navigation_readiness_from_wire(value: wire::NavigationReadiness) -> NavigationReadiness {
    NavigationReadiness {
        manifest: value.manifest,
        action_server: value.action_server,
        nav2_lifecycle: value.nav2_lifecycle,
        map: value.map,
        localization: value.localization,
        ipc: value.ipc,
    }
}

fn navigation_counters_to_wire(value: NavigationCounters) -> wire::NavigationCounters {
    wire::NavigationCounters {
        accepted_proposals: value.accepted_proposals,
        authority_rejections: value.authority_rejections,
        freshness_rejections: value.freshness_rejections,
        overwritten_proposals: value.overwritten_proposals,
        accepted_goals: value.accepted_goals,
        terminal_results: value.terminal_results,
    }
}

fn navigation_counters_from_wire(value: wire::NavigationCounters) -> NavigationCounters {
    NavigationCounters {
        accepted_proposals: value.accepted_proposals,
        authority_rejections: value.authority_rejections,
        freshness_rejections: value.freshness_rejections,
        overwritten_proposals: value.overwritten_proposals,
        accepted_goals: value.accepted_goals,
        terminal_results: value.terminal_results,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use message_model::{
        ClockState, ClockStatus, DataQuality, Header, HeaderFields, Pose3, ResourceLease,
        ResourceLeaseFields, UnitQuaternion, Vector3,
    };

    fn header(frame: Option<&str>) -> Header {
        Header::new(HeaderFields {
            source_id: AsciiId::parse("rs-navigation-bridge").unwrap(),
            boot_id: Id128::new([1; 16]).unwrap(),
            sequence: 7,
            source_monotonic_ns: 100,
            publish_unix_ns: None,
            acquisition_timestamp: None,
            valid_for_ns: 100,
            frame_id: frame.map(|value| AsciiId::parse(value).unwrap()),
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

    fn map() -> NavigationMapIdentity {
        NavigationMapIdentity::new(
            AsciiId::parse("warehouse_sim_map").unwrap(),
            AsciiId::parse("0.1.0").unwrap(),
            Sha256Identity::new([2; 32], "map").unwrap(),
            AsciiId::parse("map").unwrap(),
        )
    }

    #[test]
    fn navigation_payloads_round_trip_and_reject_bad_values() {
        let goal = NavigateToPoseGoal {
            map: map(),
            target: pose(),
            behavior_tree_id: AsciiId::parse("navigate_to_pose_fail_closed_v1").unwrap(),
        };
        assert_eq!(
            decode_navigate_to_pose_goal(&encode_navigate_to_pose_goal(goal)).unwrap(),
            goal
        );

        let feedback = NavigateToPoseFeedback {
            current_pose: pose(),
            navigation_time_ns: 10,
            estimated_time_remaining_ns: Some(20),
            recoveries: 1,
            distance_remaining_m: 3.0,
        };
        assert_eq!(
            decode_navigate_to_pose_feedback(&encode_navigate_to_pose_feedback(feedback).unwrap())
                .unwrap(),
            feedback
        );
        let bad = wire::NavigateToPoseFeedback {
            current_pose: Some(pose3_to_wire(pose())),
            distance_remaining_m: f64::NAN,
            ..Default::default()
        };
        assert!(navigate_to_pose_feedback_from_wire(bad).is_err());

        let result = NavigateToPoseResult {
            map: map(),
            final_pose: pose(),
            navigation_time_ns: 30,
            recoveries: 2,
        };
        assert_eq!(
            decode_navigate_to_pose_result(&encode_navigate_to_pose_result(result)).unwrap(),
            result
        );
    }

    #[test]
    fn authority_round_trip_requires_exact_current_lease() {
        let lease = ResourceLease::new(ResourceLeaseFields {
            resource_id: AsciiId::parse("chassis").unwrap(),
            owner_id: AsciiId::parse("mission").unwrap(),
            lease_id: Id128::new([3; 16]).unwrap(),
            session_epoch: 2,
            expires_monotonic_ns: 500,
        })
        .unwrap();
        let state = NavigationAuthorityState {
            header: header(None),
            authority: NavigationAuthority::Available {
                lease,
                active_command_id: Some(Id128::new([4; 16]).unwrap()),
            },
        };
        let bytes = encode_navigation_authority_state(state, 499).unwrap();
        assert_eq!(
            decode_navigation_authority_state(&bytes, 499).unwrap(),
            state
        );
        assert!(decode_navigation_authority_state(&bytes, 500).is_err());

        let invalid = wire::NavigationAuthorityState {
            header: Some(header_to_wire(&header(None))),
            lease: None,
            active_command_id: vec![5; 16],
            unavailable: wire::NavigationAuthorityUnavailable::NoLease as i32,
        };
        assert!(navigation_authority_state_from_wire(invalid, 1).is_err());
    }

    #[test]
    fn health_round_trip_is_bounded_and_rejects_unknown_enum() {
        let health = NavigationHealth::new(
            header(None),
            2,
            Sha256Identity::new([6; 32], "manifest").unwrap(),
            NavigationComponentState::Ready,
            NavigationReadiness {
                manifest: true,
                action_server: true,
                nav2_lifecycle: true,
                map: true,
                localization: true,
                ipc: true,
            },
            ArrayVec::new(),
            NavigationCounters::default(),
        )
        .unwrap();
        assert_eq!(
            decode_navigation_health(&encode_navigation_health(&health)).unwrap(),
            health
        );
        let mut wire = navigation_health_to_wire(&health);
        wire.component_state = 99;
        assert!(navigation_health_from_wire(wire).is_err());
    }
}
