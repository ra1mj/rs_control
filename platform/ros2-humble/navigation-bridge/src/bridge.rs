use contract_codec::{encode_navigate_to_pose_feedback, encode_navigate_to_pose_result};
use geometry_msgs::msg::{Pose, PoseStamped, Quaternion, Twist};
use message_model::{
    ActionCancelDecision, ActionCancelResponse, ActionFeedback, ActionResult, ActionStatus,
    AsciiId, ChassisSetpointIntent, ClockState, ClockStatus, CommandIntent, CommandIntentFields,
    DataQuality, Header, HeaderFields, Id128, NavigateToPoseFeedback, NavigateToPoseGoal,
    NavigateToPoseResult, NavigationAuthority, NavigationAuthorityState, NavigationMapIdentity,
    Pose3, RegisteredPayload, SemanticError, Twist2, UnitQuaternion, Vector3,
};
use nav2_msgs::action::{NavigateToPose_Feedback, NavigateToPose_Goal};
use navigation_ipc::{
    NavigationActionResponse, NavigationIpcError, NAVIGATE_TO_POSE_FEEDBACK_TYPE,
    NAVIGATE_TO_POSE_RESULT_TYPE,
};
use thiserror::Error;

pub const PROPOSAL_TTL_NS: u64 = 100_000_000;
pub const RESPONSE_TTL_NS: u64 = 1_000_000_000;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AdmissionError {
    AuthorityUnavailable,
    NoActiveGoal,
    ExpiredAuthority,
    NonFiniteTwist,
}

impl std::fmt::Display for AdmissionError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "{self:?}")
    }
}

impl std::error::Error for AdmissionError {}

#[derive(Debug, Error)]
pub enum MappingError {
    #[error(transparent)]
    Semantic(#[from] SemanticError),
    #[error("ROS duration is negative or out of range")]
    Duration,
    #[error("ROS recovery count is negative")]
    Recoveries,
}

#[derive(Debug, Error)]
pub enum CorrelationError {
    #[error("another navigation goal is already active")]
    Busy,
    #[error("the canonical command ID has no active ROS goal")]
    UnknownCommand,
    #[error("the ROS goal UUID does not match the canonical command ID binding")]
    WrongRosGoal,
    #[error(transparent)]
    Mapping(#[from] MappingError),
    #[error(transparent)]
    Semantic(#[from] SemanticError),
    #[error(transparent)]
    Identifier(#[from] message_model::IdError),
    #[error(transparent)]
    Codec(#[from] contract_codec::CodecError),
    #[error(transparent)]
    Ipc(#[from] NavigationIpcError),
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct GoalBinding {
    pub command_id: Id128,
    pub ros_goal_uuid: [u8; 16],
}

#[derive(Clone, Copy, Debug)]
struct ActiveGoal {
    binding: GoalBinding,
    map: NavigationMapIdentity,
    last_feedback: NavigateToPoseFeedback,
    canceling: bool,
}

/// Bounded one-goal correlation state between canonical IPC IDs and rclrs UUIDs.
///
/// rclrs 0.7 creates the ROS UUID internally, so the adapter retains an exact
/// one-to-one mapping instead of pretending the two identifiers are identical.
pub struct ActionCorrelation {
    source_id: AsciiId<64>,
    boot_id: Id128,
    sequence: u64,
    active: Option<ActiveGoal>,
}

impl ActionCorrelation {
    pub fn new(boot_id: Id128) -> Self {
        Self {
            source_id: AsciiId::parse("navigation-bridge").expect("static bridge source"),
            boot_id,
            sequence: 0,
            active: None,
        }
    }

    pub fn bind(
        &mut self,
        command_id: Id128,
        ros_goal_uuid: [u8; 16],
        goal: NavigateToPoseGoal,
    ) -> Result<GoalBinding, CorrelationError> {
        if self.active.is_some() {
            return Err(CorrelationError::Busy);
        }
        let binding = GoalBinding {
            command_id,
            ros_goal_uuid,
        };
        self.active = Some(ActiveGoal {
            binding,
            map: goal.map,
            last_feedback: NavigateToPoseFeedback {
                current_pose: goal.target,
                navigation_time_ns: 0,
                estimated_time_remaining_ns: None,
                recoveries: 0,
                distance_remaining_m: 0.0,
            },
            canceling: false,
        });
        Ok(binding)
    }

    pub const fn is_idle(&self) -> bool {
        self.active.is_none()
    }

    pub fn binding(&self, command_id: Id128) -> Result<GoalBinding, CorrelationError> {
        let active = self.active.ok_or(CorrelationError::UnknownCommand)?;
        if active.binding.command_id != command_id {
            return Err(CorrelationError::UnknownCommand);
        }
        Ok(active.binding)
    }

    pub fn feedback(
        &mut self,
        binding: GoalBinding,
        value: NavigateToPose_Feedback,
        now_monotonic_ns: u64,
    ) -> Result<NavigationActionResponse, CorrelationError> {
        let semantic = feedback_from_ros(value)?;
        let active = self.active_mut(binding)?;
        active.last_feedback = semantic;
        let status = if active.canceling {
            ActionStatus::Canceling
        } else {
            ActionStatus::Executing
        };
        let payload = RegisteredPayload::new(
            AsciiId::parse(NAVIGATE_TO_POSE_FEEDBACK_TYPE)?,
            encode_navigate_to_pose_feedback(semantic)?,
        )?;
        let value = ActionFeedback {
            header: self.header(now_monotonic_ns)?,
            command_id: binding.command_id,
            status,
            feedback: payload,
        };
        Ok(NavigationActionResponse::feedback(value)?)
    }

    pub fn cancellation(
        &mut self,
        binding: GoalBinding,
        accepted: bool,
        result_code: u32,
        detail: String,
        now_monotonic_ns: u64,
    ) -> Result<NavigationActionResponse, CorrelationError> {
        if accepted {
            self.active_mut(binding)?.canceling = true;
        } else {
            let _ = self.active_mut(binding)?;
        }
        let value = ActionCancelResponse {
            header: self.header(now_monotonic_ns)?,
            command_id: binding.command_id,
            decision: if accepted {
                ActionCancelDecision::Accepted
            } else {
                ActionCancelDecision::Rejected
            },
            result_code,
            detail,
        };
        Ok(NavigationActionResponse::cancel(value)?)
    }

    pub fn finish(
        &mut self,
        binding: GoalBinding,
        status: ActionStatus,
        result_code: u32,
        detail: String,
        now_monotonic_ns: u64,
    ) -> Result<NavigationActionResponse, CorrelationError> {
        let active = *self.active_ref(binding)?;
        let semantic = NavigateToPoseResult {
            map: active.map,
            final_pose: active.last_feedback.current_pose,
            navigation_time_ns: active.last_feedback.navigation_time_ns,
            recoveries: active.last_feedback.recoveries,
        };
        let payload = RegisteredPayload::new(
            AsciiId::parse(NAVIGATE_TO_POSE_RESULT_TYPE)?,
            encode_navigate_to_pose_result(semantic),
        )?;
        let value = ActionResult {
            header: self.header(now_monotonic_ns)?,
            command_id: binding.command_id,
            status,
            result_code,
            detail,
            result: payload,
        };
        let response = NavigationActionResponse::result(value)?;
        self.active = None;
        Ok(response)
    }

    fn active_ref(&self, binding: GoalBinding) -> Result<&ActiveGoal, CorrelationError> {
        let active = self
            .active
            .as_ref()
            .ok_or(CorrelationError::UnknownCommand)?;
        if active.binding.command_id != binding.command_id {
            return Err(CorrelationError::UnknownCommand);
        }
        if active.binding.ros_goal_uuid != binding.ros_goal_uuid {
            return Err(CorrelationError::WrongRosGoal);
        }
        Ok(active)
    }

    fn active_mut(&mut self, binding: GoalBinding) -> Result<&mut ActiveGoal, CorrelationError> {
        let active = self
            .active
            .as_mut()
            .ok_or(CorrelationError::UnknownCommand)?;
        if active.binding.command_id != binding.command_id {
            return Err(CorrelationError::UnknownCommand);
        }
        if active.binding.ros_goal_uuid != binding.ros_goal_uuid {
            return Err(CorrelationError::WrongRosGoal);
        }
        Ok(active)
    }

    fn header(&mut self, now_monotonic_ns: u64) -> Result<Header, SemanticError> {
        self.sequence = self.sequence.saturating_add(1);
        Header::new(HeaderFields {
            source_id: self.source_id,
            boot_id: self.boot_id,
            sequence: self.sequence,
            source_monotonic_ns: now_monotonic_ns,
            publish_unix_ns: None,
            acquisition_timestamp: None,
            valid_for_ns: RESPONSE_TTL_NS,
            frame_id: None,
            clock_status: ClockStatus {
                state: ClockState::Local,
                uncertainty_ns: 0,
            },
            quality: DataQuality::Valid,
        })
    }
}

pub struct BridgeAdmission {
    source_id: AsciiId<64>,
    boot_id: Id128,
    sequence: u64,
    authority: Option<NavigationAuthorityState>,
}

impl BridgeAdmission {
    pub fn new(boot_id: Id128) -> Self {
        Self {
            source_id: AsciiId::parse("navigation-bridge").expect("static bridge source"),
            boot_id,
            sequence: 0,
            authority: None,
        }
    }

    pub fn update_authority(
        &mut self,
        value: NavigationAuthorityState,
        now_monotonic_ns: u64,
    ) -> Result<(), AdmissionError> {
        value
            .validate_at(now_monotonic_ns)
            .map_err(|_| AdmissionError::ExpiredAuthority)?;
        self.authority = Some(value);
        Ok(())
    }

    pub fn clear_authority(&mut self) {
        self.authority = None;
    }

    pub fn accept_twist(
        &mut self,
        value: Twist,
        received_monotonic_ns: u64,
    ) -> Result<ChassisSetpointIntent, AdmissionError> {
        let authority = self.authority.ok_or(AdmissionError::AuthorityUnavailable)?;
        authority
            .validate_at(received_monotonic_ns)
            .map_err(|_| AdmissionError::ExpiredAuthority)?;
        let NavigationAuthority::Available {
            lease,
            active_command_id: Some(command_id),
        } = authority.authority
        else {
            return Err(AdmissionError::NoActiveGoal);
        };
        let target = Twist2::new(value.linear.x, value.linear.y, value.angular.z)
            .map_err(|_| AdmissionError::NonFiniteTwist)?;
        self.sequence = self.sequence.saturating_add(1);
        let header = Header::new(HeaderFields {
            source_id: self.source_id,
            boot_id: self.boot_id,
            sequence: self.sequence,
            source_monotonic_ns: received_monotonic_ns,
            publish_unix_ns: None,
            acquisition_timestamp: None,
            valid_for_ns: PROPOSAL_TTL_NS,
            frame_id: None,
            clock_status: ClockStatus {
                state: ClockState::Local,
                uncertainty_ns: 0,
            },
            quality: DataQuality::Valid,
        })
        .map_err(|_| AdmissionError::NonFiniteTwist)?;
        let intent = CommandIntent::new(CommandIntentFields {
            header,
            resource_id: lease.resource_id(),
            owner_id: lease.owner_id(),
            lease_id: lease.lease_id(),
            session_epoch: lease.session_epoch(),
            command_id,
            requested_ttl_ns: PROPOSAL_TTL_NS,
        })
        .map_err(|_| AdmissionError::AuthorityUnavailable)?;
        Ok(ChassisSetpointIntent::new(intent, target))
    }
}

pub fn goal_to_ros(value: NavigateToPoseGoal) -> NavigateToPose_Goal {
    NavigateToPose_Goal {
        pose: PoseStamped {
            header: std_msgs::msg::Header {
                stamp: builtin_interfaces::msg::Time::default(),
                frame_id: value.map.frame_id().as_str().to_owned(),
            },
            pose: pose_to_ros(value.target),
        },
        // The semantic contract carries an allowlisted identity, not a path.
        // An empty ROS field selects the exact launch-bound default BT file.
        behavior_tree: String::new(),
    }
}

pub fn feedback_from_ros(
    value: NavigateToPose_Feedback,
) -> Result<NavigateToPoseFeedback, MappingError> {
    let estimated_ns = duration_ns(value.estimated_time_remaining)?;
    NavigateToPoseFeedback {
        current_pose: pose_from_ros(value.current_pose.pose)?,
        navigation_time_ns: duration_ns(value.navigation_time)?.unwrap_or(0),
        estimated_time_remaining_ns: estimated_ns,
        recoveries: u32::try_from(value.number_of_recoveries)
            .map_err(|_| MappingError::Recoveries)?,
        distance_remaining_m: f64::from(value.distance_remaining),
    }
    .validate()
    .map_err(Into::into)
}

fn duration_ns(value: builtin_interfaces::msg::Duration) -> Result<Option<u64>, MappingError> {
    let seconds = u64::try_from(value.sec).map_err(|_| MappingError::Duration)?;
    if value.nanosec >= 1_000_000_000 {
        return Err(MappingError::Duration);
    }
    let result = seconds
        .checked_mul(1_000_000_000)
        .and_then(|base| base.checked_add(u64::from(value.nanosec)))
        .ok_or(MappingError::Duration)?;
    Ok((result != 0).then_some(result))
}

fn pose_to_ros(value: Pose3) -> Pose {
    let [x, y, z, w] = value.orientation.components();
    Pose {
        position: geometry_msgs::msg::Point {
            x: value.position_m.x(),
            y: value.position_m.y(),
            z: value.position_m.z(),
        },
        orientation: Quaternion { x, y, z, w },
    }
}

fn pose_from_ros(value: Pose) -> Result<Pose3, SemanticError> {
    Ok(Pose3 {
        position_m: Vector3::new(value.position.x, value.position.y, value.position.z)?,
        orientation: UnitQuaternion::new(
            value.orientation.x,
            value.orientation.y,
            value.orientation.z,
            value.orientation.w,
        )?,
    })
}
