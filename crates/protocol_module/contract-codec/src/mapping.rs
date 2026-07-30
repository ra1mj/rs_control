//! Codec coverage for the first backend-neutral ROS mapping profile.

use arrayvec::ArrayVec;
use message_model::{
    ActionCancelDecision, ActionCancelRequest, ActionCancelResponse, ActionFeedback, ActionGoal,
    ActionName, ActionResult, ActionStatus, AsciiId, ChassisSetpoint, ChassisTrajectoryPoint,
    ComponentHealth, Covariance3, Covariance6, CovarianceStatus, ExecuteChassisTrajectoryFeedback,
    ExecuteChassisTrajectoryGoal, ExecuteChassisTrajectoryResult, Fault, FaultRecoverability,
    FaultSeverity, HealthSnapshotRequest, HealthSnapshotResponse, Id128, ImuMeasurement, ImuState,
    JointState, JointStateAggregate, LifecycleState, MAX_COMPONENT_FAULTS, MAX_HEALTH_COMPONENTS,
    MAX_JOINTS, MAX_TRAJECTORY_POINTS, MAX_TRANSFORMS, OdometryState, OdometryStateFields, Pose3,
    ProtobufTypeName, RegisteredPayload, StampedTransform, StampedTransformFields, TransformBatch,
    Twist2, Twist3, UnitQuaternion, Vector3,
};
use prost::Message;
use proto_generated::robot::v1 as wire;

use crate::{
    CodecError, command_envelope_from_wire, command_envelope_to_wire, header_from_wire,
    header_to_wire,
};

/// Encodes a validated IMU state.
pub fn encode_imu_state(value: &ImuState) -> Vec<u8> {
    imu_state_to_wire(value).encode_to_vec()
}

/// Decodes and validates an IMU state.
pub fn decode_imu_state(bytes: &[u8]) -> Result<ImuState, CodecError> {
    imu_state_from_wire(wire::ImuState::decode(bytes)?)
}

/// Converts a semantic IMU state to its wire DTO.
#[must_use]
pub fn imu_state_to_wire(value: &ImuState) -> wire::ImuState {
    let (orientation, orientation_covariance) =
        orientation_measurement_to_wire(value.orientation());
    let (angular_velocity_radps, angular_velocity_covariance) =
        vector_measurement_to_wire(value.angular_velocity());
    let (linear_acceleration_mps2, linear_acceleration_covariance) =
        vector_measurement_to_wire(value.linear_acceleration());
    wire::ImuState {
        header: Some(header_to_wire(&value.header())),
        orientation,
        angular_velocity_radps,
        linear_acceleration_mps2,
        orientation_covariance,
        angular_velocity_covariance,
        linear_acceleration_covariance,
    }
}

/// Converts and validates an IMU wire DTO.
pub fn imu_state_from_wire(value: wire::ImuState) -> Result<ImuState, CodecError> {
    ImuState::new(
        header_from_wire(value.header.ok_or(CodecError::MissingField("header"))?)?,
        orientation_measurement_from_wire(value.orientation, value.orientation_covariance)?,
        vector_measurement_from_wire(
            value.angular_velocity_radps,
            value.angular_velocity_covariance,
            "angular_velocity_covariance",
        )?,
        vector_measurement_from_wire(
            value.linear_acceleration_mps2,
            value.linear_acceleration_covariance,
            "linear_acceleration_covariance",
        )?,
    )
    .map_err(CodecError::from)
}

/// Encodes a validated joint aggregation.
pub fn encode_joint_state_aggregate(value: &JointStateAggregate) -> Vec<u8> {
    joint_state_aggregate_to_wire(value).encode_to_vec()
}

/// Decodes and validates a joint aggregation.
pub fn decode_joint_state_aggregate(bytes: &[u8]) -> Result<JointStateAggregate, CodecError> {
    joint_state_aggregate_from_wire(wire::JointStateAggregate::decode(bytes)?)
}

/// Converts a semantic joint aggregation to its wire DTO.
#[must_use]
pub fn joint_state_aggregate_to_wire(value: &JointStateAggregate) -> wire::JointStateAggregate {
    wire::JointStateAggregate {
        header: Some(header_to_wire(&value.header())),
        joints: value
            .joints()
            .iter()
            .map(|joint| wire::JointState {
                resource_id: joint.resource_id().as_str().to_owned(),
                position_rad: joint.position_rad(),
                velocity_radps: joint.velocity_radps(),
                effort_nm: joint.effort_nm(),
            })
            .collect(),
    }
}

/// Converts and validates a joint aggregation wire DTO.
pub fn joint_state_aggregate_from_wire(
    value: wire::JointStateAggregate,
) -> Result<JointStateAggregate, CodecError> {
    let mut joints = ArrayVec::<JointState, MAX_JOINTS>::new();
    if value.joints.len() > MAX_JOINTS {
        return Err(CodecError::TooMany {
            field: "joints",
            maximum: MAX_JOINTS,
            actual: value.joints.len(),
        });
    }
    for joint in value.joints {
        joints.push(JointState::new(
            AsciiId::parse(&joint.resource_id)?,
            joint.position_rad,
            joint.velocity_radps,
            joint.effort_nm,
        )?);
    }
    JointStateAggregate::new(
        header_from_wire(value.header.ok_or(CodecError::MissingField("header"))?)?,
        joints,
    )
    .map_err(CodecError::from)
}

/// Encodes validated odometry.
pub fn encode_odometry_state(value: &OdometryState) -> Vec<u8> {
    odometry_state_to_wire(value).encode_to_vec()
}

/// Decodes and validates odometry.
pub fn decode_odometry_state(bytes: &[u8]) -> Result<OdometryState, CodecError> {
    odometry_state_from_wire(wire::OdometryState::decode(bytes)?)
}

/// Converts semantic odometry to its wire DTO.
#[must_use]
pub fn odometry_state_to_wire(value: &OdometryState) -> wire::OdometryState {
    wire::OdometryState {
        header: Some(header_to_wire(&value.header())),
        parent_frame_id: value.parent_frame_id().as_str().to_owned(),
        child_frame_id: value.child_frame_id().as_str().to_owned(),
        pose: Some(pose3_to_wire(value.pose())),
        twist: Some(twist3_to_wire(value.twist())),
        pose_covariance: value.pose_covariance().values().to_vec(),
        twist_covariance: value.twist_covariance().values().to_vec(),
    }
}

/// Converts and validates an odometry wire DTO.
pub fn odometry_state_from_wire(value: wire::OdometryState) -> Result<OdometryState, CodecError> {
    OdometryState::new(OdometryStateFields {
        header: header_from_wire(value.header.ok_or(CodecError::MissingField("header"))?)?,
        parent_frame_id: AsciiId::parse(&value.parent_frame_id)?,
        child_frame_id: AsciiId::parse(&value.child_frame_id)?,
        pose: pose3_from_wire(value.pose.ok_or(CodecError::MissingField("pose"))?)?,
        twist: twist3_from_wire(value.twist.ok_or(CodecError::MissingField("twist"))?)?,
        pose_covariance: covariance6_from_vec(value.pose_covariance, "pose_covariance")?,
        twist_covariance: covariance6_from_vec(value.twist_covariance, "twist_covariance")?,
    })
    .map_err(CodecError::from)
}

/// Encodes a validated transform batch.
pub fn encode_transform_batch(value: &TransformBatch) -> Vec<u8> {
    transform_batch_to_wire(value).encode_to_vec()
}

/// Decodes and validates a transform batch.
pub fn decode_transform_batch(bytes: &[u8]) -> Result<TransformBatch, CodecError> {
    transform_batch_from_wire(wire::TransformBatch::decode(bytes)?)
}

/// Converts a semantic transform batch to its wire DTO.
#[must_use]
pub fn transform_batch_to_wire(value: &TransformBatch) -> wire::TransformBatch {
    wire::TransformBatch {
        transforms: value
            .transforms()
            .iter()
            .map(|transform| wire::StampedTransform {
                header: Some(header_to_wire(&transform.header())),
                parent_frame_id: transform.parent_frame_id().as_str().to_owned(),
                child_frame_id: transform.child_frame_id().as_str().to_owned(),
                translation_m: Some(vector3_to_wire(transform.translation_m())),
                rotation: Some(quaternion_to_wire(transform.rotation())),
                is_static: transform.is_static(),
            })
            .collect(),
    }
}

/// Converts and validates a transform batch wire DTO.
pub fn transform_batch_from_wire(
    value: wire::TransformBatch,
) -> Result<TransformBatch, CodecError> {
    if value.transforms.len() > MAX_TRANSFORMS {
        return Err(CodecError::TooMany {
            field: "transforms",
            maximum: MAX_TRANSFORMS,
            actual: value.transforms.len(),
        });
    }
    let mut transforms = ArrayVec::<StampedTransform, MAX_TRANSFORMS>::new();
    for transform in value.transforms {
        transforms.push(StampedTransform::new(StampedTransformFields {
            header: header_from_wire(
                transform
                    .header
                    .ok_or(CodecError::MissingField("transforms.header"))?,
            )?,
            parent_frame_id: AsciiId::parse(&transform.parent_frame_id)?,
            child_frame_id: AsciiId::parse(&transform.child_frame_id)?,
            translation_m: vector3_from_wire(
                transform
                    .translation_m
                    .ok_or(CodecError::MissingField("transforms.translation_m"))?,
            )?,
            rotation: quaternion_from_wire(
                transform
                    .rotation
                    .ok_or(CodecError::MissingField("transforms.rotation"))?,
            )?,
            is_static: transform.is_static,
        })?);
    }
    TransformBatch::new(transforms).map_err(CodecError::from)
}

/// Encodes a validated chassis setpoint after checking its local deadline.
pub fn encode_chassis_setpoint(
    value: &ChassisSetpoint,
    now_monotonic_ns: u64,
) -> Result<Vec<u8>, CodecError> {
    value.envelope.validate_at(now_monotonic_ns)?;
    Ok(chassis_setpoint_to_wire(value).encode_to_vec())
}

/// Decodes a chassis setpoint and checks its local deadline.
pub fn decode_chassis_setpoint(
    bytes: &[u8],
    now_monotonic_ns: u64,
) -> Result<ChassisSetpoint, CodecError> {
    chassis_setpoint_from_wire(wire::ChassisSetpoint::decode(bytes)?, now_monotonic_ns)
}

/// Converts a validated chassis setpoint to its wire DTO.
#[must_use]
pub fn chassis_setpoint_to_wire(value: &ChassisSetpoint) -> wire::ChassisSetpoint {
    wire::ChassisSetpoint {
        envelope: Some(command_envelope_to_wire(&value.envelope)),
        target: Some(twist2_to_wire(value.target)),
    }
}

/// Converts and validates a chassis setpoint wire DTO.
pub fn chassis_setpoint_from_wire(
    value: wire::ChassisSetpoint,
    now_monotonic_ns: u64,
) -> Result<ChassisSetpoint, CodecError> {
    Ok(ChassisSetpoint {
        envelope: command_envelope_from_wire(
            value.envelope.ok_or(CodecError::MissingField("envelope"))?,
            now_monotonic_ns,
        )?,
        target: twist2_from_wire(value.target.ok_or(CodecError::MissingField("target"))?)?,
    })
}

fn orientation_measurement_to_wire(
    value: ImuMeasurement<UnitQuaternion>,
) -> (Option<wire::Quaternion>, Vec<f64>) {
    match value {
        ImuMeasurement::Unavailable => (None, Vec::new()),
        ImuMeasurement::Present { value, covariance } => (
            Some(quaternion_to_wire(value)),
            covariance_to_wire(covariance),
        ),
    }
}

fn vector_measurement_to_wire(value: ImuMeasurement<Vector3>) -> (Option<wire::Vector3>, Vec<f64>) {
    match value {
        ImuMeasurement::Unavailable => (None, Vec::new()),
        ImuMeasurement::Present { value, covariance } => {
            (Some(vector3_to_wire(value)), covariance_to_wire(covariance))
        }
    }
}

fn orientation_measurement_from_wire(
    value: Option<wire::Quaternion>,
    covariance: Vec<f64>,
) -> Result<ImuMeasurement<UnitQuaternion>, CodecError> {
    let Some(value) = value else {
        if !covariance.is_empty() {
            return Err(CodecError::InvalidField {
                field: "orientation_covariance",
                reason: "covariance is present for an unavailable measurement",
            });
        }
        return Ok(ImuMeasurement::Unavailable);
    };
    Ok(ImuMeasurement::Present {
        value: quaternion_from_wire(value)?,
        covariance: covariance_status_from_vec(covariance, "orientation_covariance")?,
    })
}

fn vector_measurement_from_wire(
    value: Option<wire::Vector3>,
    covariance: Vec<f64>,
    field: &'static str,
) -> Result<ImuMeasurement<Vector3>, CodecError> {
    let Some(value) = value else {
        if !covariance.is_empty() {
            return Err(CodecError::InvalidField {
                field,
                reason: "covariance is present for an unavailable measurement",
            });
        }
        return Ok(ImuMeasurement::Unavailable);
    };
    Ok(ImuMeasurement::Present {
        value: vector3_from_wire(value)?,
        covariance: covariance_status_from_vec(covariance, field)?,
    })
}

fn covariance_to_wire(value: CovarianceStatus) -> Vec<f64> {
    match value {
        CovarianceStatus::Unknown => vec![0.0; 9],
        CovarianceStatus::Known(value) => value.values().to_vec(),
    }
}

fn covariance_status_from_vec(
    values: Vec<f64>,
    field: &'static str,
) -> Result<CovarianceStatus, CodecError> {
    if values.is_empty() || values.iter().all(|value| *value == 0.0) {
        return Ok(CovarianceStatus::Unknown);
    }
    Ok(CovarianceStatus::Known(Covariance3::new(exact_array(
        values, field,
    )?)?))
}

fn covariance6_from_vec(values: Vec<f64>, field: &'static str) -> Result<Covariance6, CodecError> {
    Ok(Covariance6::new(exact_array(values, field)?)?)
}

fn exact_array<const N: usize>(
    values: Vec<f64>,
    field: &'static str,
) -> Result<[f64; N], CodecError> {
    let actual = values.len();
    values.try_into().map_err(|_| CodecError::InvalidLength {
        field,
        expected: N,
        actual,
    })
}

fn vector3_to_wire(value: Vector3) -> wire::Vector3 {
    wire::Vector3 {
        x: value.x(),
        y: value.y(),
        z: value.z(),
    }
}

fn vector3_from_wire(value: wire::Vector3) -> Result<Vector3, CodecError> {
    Vector3::new(value.x, value.y, value.z).map_err(CodecError::from)
}

fn quaternion_to_wire(value: UnitQuaternion) -> wire::Quaternion {
    let [x, y, z, w] = value.components();
    wire::Quaternion { x, y, z, w }
}

fn quaternion_from_wire(value: wire::Quaternion) -> Result<UnitQuaternion, CodecError> {
    UnitQuaternion::new(value.x, value.y, value.z, value.w).map_err(CodecError::from)
}

fn pose3_to_wire(value: Pose3) -> wire::Pose3 {
    wire::Pose3 {
        position_m: Some(vector3_to_wire(value.position_m)),
        orientation: Some(quaternion_to_wire(value.orientation)),
    }
}

fn pose3_from_wire(value: wire::Pose3) -> Result<Pose3, CodecError> {
    Ok(Pose3 {
        position_m: vector3_from_wire(
            value
                .position_m
                .ok_or(CodecError::MissingField("pose.position_m"))?,
        )?,
        orientation: quaternion_from_wire(
            value
                .orientation
                .ok_or(CodecError::MissingField("pose.orientation"))?,
        )?,
    })
}

fn twist3_to_wire(value: Twist3) -> wire::Twist3 {
    wire::Twist3 {
        linear_mps: Some(vector3_to_wire(value.linear_mps)),
        angular_radps: Some(vector3_to_wire(value.angular_radps)),
    }
}

fn twist3_from_wire(value: wire::Twist3) -> Result<Twist3, CodecError> {
    Ok(Twist3 {
        linear_mps: vector3_from_wire(
            value
                .linear_mps
                .ok_or(CodecError::MissingField("twist.linear_mps"))?,
        )?,
        angular_radps: vector3_from_wire(
            value
                .angular_radps
                .ok_or(CodecError::MissingField("twist.angular_radps"))?,
        )?,
    })
}

fn twist2_to_wire(value: Twist2) -> wire::Twist2 {
    let [vx_mps, vy_mps, yaw_rate_radps] = value.components();
    wire::Twist2 {
        vx_mps,
        vy_mps,
        yaw_rate_radps,
    }
}

fn twist2_from_wire(value: wire::Twist2) -> Result<Twist2, CodecError> {
    Twist2::new(value.vx_mps, value.vy_mps, value.yaw_rate_radps).map_err(CodecError::from)
}

/// Encodes a validated health snapshot request.
pub fn encode_health_snapshot_request(value: &HealthSnapshotRequest) -> Vec<u8> {
    health_snapshot_request_to_wire(value).encode_to_vec()
}

/// Decodes and validates a health snapshot request.
pub fn decode_health_snapshot_request(bytes: &[u8]) -> Result<HealthSnapshotRequest, CodecError> {
    health_snapshot_request_from_wire(wire::HealthSnapshotRequest::decode(bytes)?)
}

/// Converts a semantic health snapshot request to its wire DTO.
#[must_use]
pub fn health_snapshot_request_to_wire(
    value: &HealthSnapshotRequest,
) -> wire::HealthSnapshotRequest {
    wire::HealthSnapshotRequest {
        header: Some(header_to_wire(&value.header)),
        component_ids: value
            .component_ids
            .iter()
            .map(|id| id.as_str().to_owned())
            .collect(),
        max_components: u32::try_from(value.max_components)
            .expect("semantic health bound fits u32"),
    }
}

/// Converts and validates a health snapshot request wire DTO.
pub fn health_snapshot_request_from_wire(
    value: wire::HealthSnapshotRequest,
) -> Result<HealthSnapshotRequest, CodecError> {
    if value.component_ids.len() > MAX_HEALTH_COMPONENTS {
        return Err(CodecError::TooMany {
            field: "component_ids",
            maximum: MAX_HEALTH_COMPONENTS,
            actual: value.component_ids.len(),
        });
    }
    let mut component_ids = ArrayVec::new();
    for id in value.component_ids {
        component_ids.push(AsciiId::parse(&id)?);
    }
    HealthSnapshotRequest {
        header: header_from_wire(value.header.ok_or(CodecError::MissingField("header"))?)?,
        component_ids,
        max_components: value.max_components as usize,
    }
    .validate()
    .map_err(CodecError::from)
}

/// Encodes a validated health snapshot response.
pub fn encode_health_snapshot_response(value: &HealthSnapshotResponse) -> Vec<u8> {
    health_snapshot_response_to_wire(value).encode_to_vec()
}

/// Decodes and validates a health snapshot response.
pub fn decode_health_snapshot_response(bytes: &[u8]) -> Result<HealthSnapshotResponse, CodecError> {
    health_snapshot_response_from_wire(wire::HealthSnapshotResponse::decode(bytes)?)
}

/// Converts a semantic health snapshot response to its wire DTO.
#[must_use]
pub fn health_snapshot_response_to_wire(
    value: &HealthSnapshotResponse,
) -> wire::HealthSnapshotResponse {
    wire::HealthSnapshotResponse {
        header: Some(header_to_wire(&value.header)),
        components: value
            .components
            .iter()
            .map(component_health_to_wire)
            .collect(),
        truncated: value.truncated,
    }
}

/// Converts and validates a health snapshot response wire DTO.
pub fn health_snapshot_response_from_wire(
    value: wire::HealthSnapshotResponse,
) -> Result<HealthSnapshotResponse, CodecError> {
    if value.components.len() > MAX_HEALTH_COMPONENTS {
        return Err(CodecError::TooMany {
            field: "components",
            maximum: MAX_HEALTH_COMPONENTS,
            actual: value.components.len(),
        });
    }
    let mut components = ArrayVec::new();
    for component in value.components {
        components.push(component_health_from_wire(component)?);
    }
    Ok(HealthSnapshotResponse {
        header: header_from_wire(value.header.ok_or(CodecError::MissingField("header"))?)?,
        components,
        truncated: value.truncated,
    })
}

fn component_health_to_wire(value: &ComponentHealth) -> wire::ComponentHealth {
    wire::ComponentHealth {
        header: Some(header_to_wire(&value.header)),
        component_id: value.component_id.as_str().to_owned(),
        lifecycle: lifecycle_to_wire(value.lifecycle) as i32,
        faults: value.faults.iter().map(fault_to_wire).collect(),
        last_success_monotonic_ns: value.last_success_monotonic_ns,
    }
}

fn component_health_from_wire(value: wire::ComponentHealth) -> Result<ComponentHealth, CodecError> {
    if value.faults.len() > MAX_COMPONENT_FAULTS {
        return Err(CodecError::TooMany {
            field: "component.faults",
            maximum: MAX_COMPONENT_FAULTS,
            actual: value.faults.len(),
        });
    }
    let mut faults = ArrayVec::new();
    for fault in value.faults {
        faults.push(fault_from_wire(fault)?);
    }
    Ok(ComponentHealth {
        header: header_from_wire(
            value
                .header
                .ok_or(CodecError::MissingField("component.header"))?,
        )?,
        component_id: AsciiId::parse(&value.component_id)?,
        lifecycle: lifecycle_from_wire(value.lifecycle)?,
        faults,
        last_success_monotonic_ns: value.last_success_monotonic_ns,
    })
}

fn fault_to_wire(value: &Fault) -> wire::Fault {
    wire::Fault {
        code: value.code,
        severity: fault_severity_to_wire(value.severity) as i32,
        latched: value.latched,
        recoverability: fault_recoverability_to_wire(value.recoverability) as i32,
        first_seen_monotonic_ns: value.first_seen_monotonic_ns,
        last_seen_monotonic_ns: value.last_seen_monotonic_ns,
        count: value.count,
        source: value.fault_source.as_str().to_owned(),
        related_command_id: value
            .related_command_id
            .map_or_else(Vec::new, |id| id.as_bytes().to_vec()),
        recommended_action: value.recommended_action,
    }
}

fn fault_from_wire(value: wire::Fault) -> Result<Fault, CodecError> {
    Fault {
        code: value.code,
        severity: fault_severity_from_wire(value.severity)?,
        latched: value.latched,
        recoverability: fault_recoverability_from_wire(value.recoverability)?,
        first_seen_monotonic_ns: value.first_seen_monotonic_ns,
        last_seen_monotonic_ns: value.last_seen_monotonic_ns,
        count: value.count,
        fault_source: AsciiId::parse(&value.source)?,
        related_command_id: if value.related_command_id.is_empty() {
            None
        } else {
            Some(Id128::from_slice(&value.related_command_id)?)
        },
        recommended_action: value.recommended_action,
    }
    .validate()
    .map_err(CodecError::from)
}

fn lifecycle_to_wire(value: LifecycleState) -> wire::LifecycleState {
    match value {
        LifecycleState::Starting => wire::LifecycleState::Starting,
        LifecycleState::Ready => wire::LifecycleState::Ready,
        LifecycleState::Active => wire::LifecycleState::Active,
        LifecycleState::Degraded => wire::LifecycleState::Degraded,
        LifecycleState::Faulted => wire::LifecycleState::Faulted,
        LifecycleState::Stopping => wire::LifecycleState::Stopping,
    }
}

fn lifecycle_from_wire(value: i32) -> Result<LifecycleState, CodecError> {
    match wire::LifecycleState::try_from(value) {
        Ok(wire::LifecycleState::Starting) => Ok(LifecycleState::Starting),
        Ok(wire::LifecycleState::Ready) => Ok(LifecycleState::Ready),
        Ok(wire::LifecycleState::Active) => Ok(LifecycleState::Active),
        Ok(wire::LifecycleState::Degraded) => Ok(LifecycleState::Degraded),
        Ok(wire::LifecycleState::Faulted) => Ok(LifecycleState::Faulted),
        Ok(wire::LifecycleState::Stopping) => Ok(LifecycleState::Stopping),
        Ok(wire::LifecycleState::Unspecified) | Err(_) => Err(CodecError::UnknownEnum {
            field: "component.lifecycle",
            value,
        }),
    }
}

fn fault_severity_to_wire(value: FaultSeverity) -> wire::FaultSeverity {
    match value {
        FaultSeverity::Info => wire::FaultSeverity::Info,
        FaultSeverity::Warning => wire::FaultSeverity::Warning,
        FaultSeverity::Error => wire::FaultSeverity::Error,
        FaultSeverity::Critical => wire::FaultSeverity::Critical,
    }
}

fn fault_severity_from_wire(value: i32) -> Result<FaultSeverity, CodecError> {
    match wire::FaultSeverity::try_from(value) {
        Ok(wire::FaultSeverity::Info) => Ok(FaultSeverity::Info),
        Ok(wire::FaultSeverity::Warning) => Ok(FaultSeverity::Warning),
        Ok(wire::FaultSeverity::Error) => Ok(FaultSeverity::Error),
        Ok(wire::FaultSeverity::Critical) => Ok(FaultSeverity::Critical),
        Ok(wire::FaultSeverity::Unspecified) | Err(_) => Err(CodecError::UnknownEnum {
            field: "fault.severity",
            value,
        }),
    }
}

fn fault_recoverability_to_wire(value: FaultRecoverability) -> wire::FaultRecoverability {
    match value {
        FaultRecoverability::Automatic => wire::FaultRecoverability::Automatic,
        FaultRecoverability::Operator => wire::FaultRecoverability::Operator,
        FaultRecoverability::PowerCycle => wire::FaultRecoverability::PowerCycle,
        FaultRecoverability::NotRecoverable => wire::FaultRecoverability::NotRecoverable,
    }
}

fn fault_recoverability_from_wire(value: i32) -> Result<FaultRecoverability, CodecError> {
    match wire::FaultRecoverability::try_from(value) {
        Ok(wire::FaultRecoverability::Automatic) => Ok(FaultRecoverability::Automatic),
        Ok(wire::FaultRecoverability::Operator) => Ok(FaultRecoverability::Operator),
        Ok(wire::FaultRecoverability::PowerCycle) => Ok(FaultRecoverability::PowerCycle),
        Ok(wire::FaultRecoverability::NotRecoverable) => Ok(FaultRecoverability::NotRecoverable),
        Ok(wire::FaultRecoverability::Unspecified) | Err(_) => Err(CodecError::UnknownEnum {
            field: "fault.recoverability",
            value,
        }),
    }
}

/// Encodes an action goal after checking its local deadline.
pub fn encode_action_goal(
    value: &ActionGoal,
    now_monotonic_ns: u64,
) -> Result<Vec<u8>, CodecError> {
    value.envelope.validate_at(now_monotonic_ns)?;
    Ok(action_goal_to_wire(value).encode_to_vec())
}

/// Decodes an action goal and checks its local deadline.
pub fn decode_action_goal(bytes: &[u8], now_monotonic_ns: u64) -> Result<ActionGoal, CodecError> {
    action_goal_from_wire(wire::ActionGoal::decode(bytes)?, now_monotonic_ns)
}

/// Converts a semantic action goal to its wire DTO.
#[must_use]
pub fn action_goal_to_wire(value: &ActionGoal) -> wire::ActionGoal {
    wire::ActionGoal {
        envelope: Some(command_envelope_to_wire(&value.envelope)),
        action_name: value.action_name.as_str().to_owned(),
        goal: Some(payload_to_wire(&value.goal)),
    }
}

/// Converts and validates an action goal wire DTO.
pub fn action_goal_from_wire(
    value: wire::ActionGoal,
    now_monotonic_ns: u64,
) -> Result<ActionGoal, CodecError> {
    Ok(ActionGoal {
        envelope: command_envelope_from_wire(
            value.envelope.ok_or(CodecError::MissingField("envelope"))?,
            now_monotonic_ns,
        )?,
        action_name: ActionName::parse(&value.action_name)?,
        goal: payload_from_wire(value.goal.ok_or(CodecError::MissingField("goal"))?)?,
    })
}

/// Converts a semantic action feedback to its wire DTO.
pub fn encode_action_feedback(value: &ActionFeedback) -> Result<Vec<u8>, CodecError> {
    value.clone().validate()?;
    Ok(action_feedback_to_wire(value).encode_to_vec())
}

/// Decodes and validates action feedback.
pub fn decode_action_feedback(bytes: &[u8]) -> Result<ActionFeedback, CodecError> {
    action_feedback_from_wire(wire::ActionFeedback::decode(bytes)?)
}

/// Converts a semantic action feedback to its wire DTO.
#[must_use]
pub fn action_feedback_to_wire(value: &ActionFeedback) -> wire::ActionFeedback {
    wire::ActionFeedback {
        header: Some(header_to_wire(&value.header)),
        command_id: value.command_id.as_bytes().to_vec(),
        status: action_status_to_wire(value.status) as i32,
        feedback: Some(payload_to_wire(&value.feedback)),
    }
}

/// Converts and validates an action feedback wire DTO.
pub fn action_feedback_from_wire(
    value: wire::ActionFeedback,
) -> Result<ActionFeedback, CodecError> {
    ActionFeedback {
        header: header_from_wire(value.header.ok_or(CodecError::MissingField("header"))?)?,
        command_id: Id128::from_slice(&value.command_id)?,
        status: action_status_from_wire(value.status)?,
        feedback: payload_from_wire(value.feedback.ok_or(CodecError::MissingField("feedback"))?)?,
    }
    .validate()
    .map_err(CodecError::from)
}

/// Converts a cancellation request to its wire DTO.
pub fn encode_action_cancel_request(value: &ActionCancelRequest) -> Vec<u8> {
    action_cancel_request_to_wire(value).encode_to_vec()
}

/// Decodes and validates a cancellation request.
pub fn decode_action_cancel_request(bytes: &[u8]) -> Result<ActionCancelRequest, CodecError> {
    action_cancel_request_from_wire(wire::ActionCancelRequest::decode(bytes)?)
}

/// Converts a cancellation request to its wire DTO.
#[must_use]
pub fn action_cancel_request_to_wire(value: &ActionCancelRequest) -> wire::ActionCancelRequest {
    wire::ActionCancelRequest {
        header: Some(header_to_wire(&value.header)),
        command_id: value.command_id.as_bytes().to_vec(),
    }
}

/// Converts and validates a cancellation request wire DTO.
pub fn action_cancel_request_from_wire(
    value: wire::ActionCancelRequest,
) -> Result<ActionCancelRequest, CodecError> {
    Ok(ActionCancelRequest {
        header: header_from_wire(value.header.ok_or(CodecError::MissingField("header"))?)?,
        command_id: Id128::from_slice(&value.command_id)?,
    })
}

/// Converts a cancellation response to its wire DTO.
pub fn encode_action_cancel_response(value: &ActionCancelResponse) -> Vec<u8> {
    action_cancel_response_to_wire(value).encode_to_vec()
}

/// Decodes and validates a cancellation response.
pub fn decode_action_cancel_response(bytes: &[u8]) -> Result<ActionCancelResponse, CodecError> {
    action_cancel_response_from_wire(wire::ActionCancelResponse::decode(bytes)?)
}

/// Converts a cancellation response to its wire DTO.
#[must_use]
pub fn action_cancel_response_to_wire(value: &ActionCancelResponse) -> wire::ActionCancelResponse {
    wire::ActionCancelResponse {
        header: Some(header_to_wire(&value.header)),
        command_id: value.command_id.as_bytes().to_vec(),
        decision: match value.decision {
            ActionCancelDecision::Accepted => wire::ActionCancelDecision::Accepted,
            ActionCancelDecision::Rejected => wire::ActionCancelDecision::Rejected,
        } as i32,
        result_code: value.result_code,
        detail: value.detail.clone(),
    }
}

/// Converts and validates a cancellation response wire DTO.
pub fn action_cancel_response_from_wire(
    value: wire::ActionCancelResponse,
) -> Result<ActionCancelResponse, CodecError> {
    let decision = match wire::ActionCancelDecision::try_from(value.decision) {
        Ok(wire::ActionCancelDecision::Accepted) => ActionCancelDecision::Accepted,
        Ok(wire::ActionCancelDecision::Rejected) => ActionCancelDecision::Rejected,
        Ok(wire::ActionCancelDecision::Unspecified) | Err(_) => {
            return Err(CodecError::UnknownEnum {
                field: "cancel.decision",
                value: value.decision,
            });
        }
    };
    Ok(ActionCancelResponse {
        header: header_from_wire(value.header.ok_or(CodecError::MissingField("header"))?)?,
        command_id: Id128::from_slice(&value.command_id)?,
        decision,
        result_code: value.result_code,
        detail: value.detail,
    })
}

/// Converts a terminal action result to its wire DTO.
pub fn encode_action_result(value: &ActionResult) -> Result<Vec<u8>, CodecError> {
    value.clone().validate()?;
    Ok(action_result_to_wire(value).encode_to_vec())
}

/// Decodes and validates a terminal action result.
pub fn decode_action_result(bytes: &[u8]) -> Result<ActionResult, CodecError> {
    action_result_from_wire(wire::ActionResult::decode(bytes)?)
}

/// Converts a terminal action result to its wire DTO.
#[must_use]
pub fn action_result_to_wire(value: &ActionResult) -> wire::ActionResult {
    wire::ActionResult {
        header: Some(header_to_wire(&value.header)),
        command_id: value.command_id.as_bytes().to_vec(),
        status: action_status_to_wire(value.status) as i32,
        result_code: value.result_code,
        detail: value.detail.clone(),
        result: Some(payload_to_wire(&value.result)),
    }
}

/// Converts and validates a terminal action result wire DTO.
pub fn action_result_from_wire(value: wire::ActionResult) -> Result<ActionResult, CodecError> {
    ActionResult {
        header: header_from_wire(value.header.ok_or(CodecError::MissingField("header"))?)?,
        command_id: Id128::from_slice(&value.command_id)?,
        status: action_status_from_wire(value.status)?,
        result_code: value.result_code,
        detail: value.detail,
        result: payload_from_wire(value.result.ok_or(CodecError::MissingField("result"))?)?,
    }
    .validate()
    .map_err(CodecError::from)
}

/// Converts a concrete chassis trajectory goal to its wire DTO.
pub fn encode_trajectory_goal(value: &ExecuteChassisTrajectoryGoal) -> Result<Vec<u8>, CodecError> {
    value.clone().validate()?;
    Ok(trajectory_goal_to_wire(value).encode_to_vec())
}

/// Decodes and validates a concrete chassis trajectory goal.
pub fn decode_trajectory_goal(bytes: &[u8]) -> Result<ExecuteChassisTrajectoryGoal, CodecError> {
    trajectory_goal_from_wire(wire::ExecuteChassisTrajectoryGoal::decode(bytes)?)
}

/// Converts a concrete chassis trajectory goal to its wire DTO.
#[must_use]
pub fn trajectory_goal_to_wire(
    value: &ExecuteChassisTrajectoryGoal,
) -> wire::ExecuteChassisTrajectoryGoal {
    wire::ExecuteChassisTrajectoryGoal {
        points: value
            .points
            .iter()
            .map(|point| wire::ChassisTrajectoryPoint {
                time_from_start_ns: point.time_from_start_ns,
                target: Some(twist2_to_wire(point.target)),
            })
            .collect(),
    }
}

/// Converts and validates a concrete chassis trajectory goal wire DTO.
pub fn trajectory_goal_from_wire(
    value: wire::ExecuteChassisTrajectoryGoal,
) -> Result<ExecuteChassisTrajectoryGoal, CodecError> {
    if value.points.len() > MAX_TRAJECTORY_POINTS {
        return Err(CodecError::TooMany {
            field: "trajectory.points",
            maximum: MAX_TRAJECTORY_POINTS,
            actual: value.points.len(),
        });
    }
    let mut points = ArrayVec::new();
    for point in value.points {
        points.push(ChassisTrajectoryPoint {
            time_from_start_ns: point.time_from_start_ns,
            target: twist2_from_wire(
                point
                    .target
                    .ok_or(CodecError::MissingField("trajectory.points.target"))?,
            )?,
        });
    }
    ExecuteChassisTrajectoryGoal { points }
        .validate()
        .map_err(CodecError::from)
}

/// Converts concrete action feedback to its wire DTO.
pub fn encode_trajectory_feedback(
    value: ExecuteChassisTrajectoryFeedback,
) -> Result<Vec<u8>, CodecError> {
    value.validate()?;
    Ok(trajectory_feedback_to_wire(value).encode_to_vec())
}

/// Decodes and validates concrete action feedback.
pub fn decode_trajectory_feedback(
    bytes: &[u8],
) -> Result<ExecuteChassisTrajectoryFeedback, CodecError> {
    trajectory_feedback_from_wire(wire::ExecuteChassisTrajectoryFeedback::decode(bytes)?)
}

/// Converts concrete action feedback to its wire DTO.
#[must_use]
pub fn trajectory_feedback_to_wire(
    value: ExecuteChassisTrajectoryFeedback,
) -> wire::ExecuteChassisTrajectoryFeedback {
    wire::ExecuteChassisTrajectoryFeedback {
        active_point: value.active_point,
        progress: value.progress,
    }
}

/// Converts and validates concrete action feedback wire DTO.
pub fn trajectory_feedback_from_wire(
    value: wire::ExecuteChassisTrajectoryFeedback,
) -> Result<ExecuteChassisTrajectoryFeedback, CodecError> {
    ExecuteChassisTrajectoryFeedback {
        active_point: value.active_point,
        progress: value.progress,
    }
    .validate()
    .map_err(CodecError::from)
}

/// Converts concrete action result to its wire DTO.
pub fn encode_trajectory_result(value: ExecuteChassisTrajectoryResult) -> Vec<u8> {
    trajectory_result_to_wire(value).encode_to_vec()
}

/// Decodes concrete action result.
pub fn decode_trajectory_result(
    bytes: &[u8],
) -> Result<ExecuteChassisTrajectoryResult, CodecError> {
    Ok(trajectory_result_from_wire(
        wire::ExecuteChassisTrajectoryResult::decode(bytes)?,
    ))
}

/// Converts concrete action result to its wire DTO.
#[must_use]
pub const fn trajectory_result_to_wire(
    value: ExecuteChassisTrajectoryResult,
) -> wire::ExecuteChassisTrajectoryResult {
    wire::ExecuteChassisTrajectoryResult {
        completed_points: value.completed_points,
    }
}

/// Converts concrete action result wire DTO.
#[must_use]
pub const fn trajectory_result_from_wire(
    value: wire::ExecuteChassisTrajectoryResult,
) -> ExecuteChassisTrajectoryResult {
    ExecuteChassisTrajectoryResult {
        completed_points: value.completed_points,
    }
}

fn payload_to_wire(value: &RegisteredPayload) -> wire::ActionPayload {
    wire::ActionPayload {
        protobuf_type: value.protobuf_type().as_str().to_owned(),
        value: value.value().to_vec(),
    }
}

fn payload_from_wire(value: wire::ActionPayload) -> Result<RegisteredPayload, CodecError> {
    RegisteredPayload::new(ProtobufTypeName::parse(&value.protobuf_type)?, value.value)
        .map_err(CodecError::from)
}

fn action_status_to_wire(value: ActionStatus) -> wire::ActionStatus {
    match value {
        ActionStatus::Accepted => wire::ActionStatus::Accepted,
        ActionStatus::Executing => wire::ActionStatus::Running,
        ActionStatus::Canceling => wire::ActionStatus::Canceling,
        ActionStatus::Succeeded => wire::ActionStatus::Succeeded,
        ActionStatus::Failed => wire::ActionStatus::Failed,
        ActionStatus::Cancelled => wire::ActionStatus::Cancelled,
        ActionStatus::TimedOut => wire::ActionStatus::TimedOut,
        ActionStatus::Rejected => wire::ActionStatus::Rejected,
    }
}

fn action_status_from_wire(value: i32) -> Result<ActionStatus, CodecError> {
    match wire::ActionStatus::try_from(value) {
        Ok(wire::ActionStatus::Accepted) => Ok(ActionStatus::Accepted),
        Ok(wire::ActionStatus::Running) => Ok(ActionStatus::Executing),
        Ok(wire::ActionStatus::Canceling) => Ok(ActionStatus::Canceling),
        Ok(wire::ActionStatus::Succeeded) => Ok(ActionStatus::Succeeded),
        Ok(wire::ActionStatus::Failed) => Ok(ActionStatus::Failed),
        Ok(wire::ActionStatus::Cancelled) => Ok(ActionStatus::Cancelled),
        Ok(wire::ActionStatus::TimedOut) => Ok(ActionStatus::TimedOut),
        Ok(wire::ActionStatus::Rejected) => Ok(ActionStatus::Rejected),
        Ok(wire::ActionStatus::Unspecified) | Err(_) => Err(CodecError::UnknownEnum {
            field: "action.status",
            value,
        }),
    }
}

#[cfg(test)]
mod tests {
    use message_model::{
        AcquisitionTimestamp, ActionCancelDecision, ActionCancelResponse, ActionGoal, AsciiId,
        ClockState, ClockStatus, CommandEnvelope, CommandEnvelopeFields, Covariance3, Covariance6,
        CovarianceStatus, DataQuality, Header, HeaderFields, Id128, ImuMeasurement, ImuState,
        OdometryState, OdometryStateFields, Pose3, RegisteredPayload, TimestampDomain, Twist3,
        UnitQuaternion, Vector3,
    };
    use prost::Message;
    use proto_generated::robot::v1 as wire;

    use super::{
        action_goal_from_wire, action_goal_to_wire, decode_action_cancel_response,
        decode_imu_state, decode_odometry_state, encode_action_cancel_response, encode_imu_state,
        encode_odometry_state, imu_state_to_wire, trajectory_goal_from_wire,
    };

    fn header(frame: Option<&str>) -> Header {
        Header::new(HeaderFields {
            source_id: AsciiId::parse("bridge-fixture").unwrap(),
            boot_id: Id128::new([1; 16]).unwrap(),
            sequence: 9,
            source_monotonic_ns: 1_000,
            publish_unix_ns: Some(2_000),
            acquisition_timestamp: Some(AcquisitionTimestamp {
                timestamp_ns: 1_500,
                domain: TimestampDomain::Unix,
                uncertainty_ns: 25,
            }),
            valid_for_ns: 10_000,
            frame_id: frame.map(|value| AsciiId::parse(value).unwrap()),
            clock_status: ClockStatus {
                state: ClockState::Synchronized,
                uncertainty_ns: 25,
            },
            quality: DataQuality::Valid,
        })
        .unwrap()
    }

    fn covariance3() -> Covariance3 {
        let mut values = [0.0; 9];
        values[0] = 0.1;
        values[4] = 0.2;
        values[8] = 0.3;
        Covariance3::new(values).unwrap()
    }

    fn covariance6() -> Covariance6 {
        let mut values = [0.0; 36];
        for index in 0..6 {
            values[index * 6 + index] = 0.1;
        }
        Covariance6::new(values).unwrap()
    }

    #[test]
    fn imu_round_trip_preserves_unavailable_and_unknown_measurements() {
        let value = ImuState::new(
            header(Some("imu_link")),
            ImuMeasurement::Present {
                value: UnitQuaternion::new(0.0, 0.0, 0.0, 1.0).unwrap(),
                covariance: CovarianceStatus::Unknown,
            },
            ImuMeasurement::Present {
                value: Vector3::new(0.1, 0.2, 0.3).unwrap(),
                covariance: CovarianceStatus::Known(covariance3()),
            },
            ImuMeasurement::Unavailable,
        )
        .unwrap();

        assert_eq!(decode_imu_state(&encode_imu_state(&value)).unwrap(), value);
        let wire = imu_state_to_wire(&value);
        assert_eq!(wire.orientation_covariance, vec![0.0; 9]);
        assert!(wire.linear_acceleration_mps2.is_none());
        assert!(wire.linear_acceleration_covariance.is_empty());
    }

    #[test]
    fn imu_rejects_nonfinite_and_malformed_covariance() {
        let value = ImuState::new(
            header(Some("imu_link")),
            ImuMeasurement::Unavailable,
            ImuMeasurement::Unavailable,
            ImuMeasurement::Unavailable,
        )
        .unwrap();
        let mut wire = imu_state_to_wire(&value);
        wire.angular_velocity_radps = Some(wire::Vector3 {
            x: f64::NAN,
            y: 0.0,
            z: 0.0,
        });
        assert!(decode_imu_state(&wire.encode_to_vec()).is_err());

        wire.angular_velocity_radps = Some(wire::Vector3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        });
        wire.angular_velocity_covariance = vec![1.0; 8];
        assert!(decode_imu_state(&wire.encode_to_vec()).is_err());
    }

    #[test]
    fn odometry_round_trip_preserves_explicit_frames_and_covariance() {
        let value = OdometryState::new(OdometryStateFields {
            header: header(Some("odom")),
            parent_frame_id: AsciiId::parse("odom").unwrap(),
            child_frame_id: AsciiId::parse("base_link").unwrap(),
            pose: Pose3 {
                position_m: Vector3::new(1.0, 2.0, 0.0).unwrap(),
                orientation: UnitQuaternion::new(0.0, 0.0, 0.0, 1.0).unwrap(),
            },
            twist: Twist3 {
                linear_mps: Vector3::new(0.2, 0.0, 0.0).unwrap(),
                angular_radps: Vector3::new(0.0, 0.0, 0.1).unwrap(),
            },
            pose_covariance: covariance6(),
            twist_covariance: covariance6(),
        })
        .unwrap();
        assert_eq!(
            decode_odometry_state(&encode_odometry_state(&value)).unwrap(),
            value
        );
    }

    #[test]
    fn action_goal_preserves_command_id_as_uuid_bytes() {
        let command_id = Id128::new([7; 16]).unwrap();
        let envelope = CommandEnvelope::new(CommandEnvelopeFields {
            header: header(Some("base_link")),
            resource_id: AsciiId::parse("chassis").unwrap(),
            owner_id: AsciiId::parse("ros2-adapter").unwrap(),
            lease_id: Id128::new([8; 16]).unwrap(),
            session_epoch: 2,
            command_id,
            deadline_monotonic_ns: 5_000,
        })
        .unwrap();
        let goal = ActionGoal {
            envelope,
            action_name: AsciiId::parse("robot.execute_chassis_trajectory.v1").unwrap(),
            goal: RegisteredPayload::new(
                AsciiId::parse("robot.v1.ExecuteChassisTrajectoryGoal").unwrap(),
                vec![1, 2, 3],
            )
            .unwrap(),
        };
        let wire = action_goal_to_wire(&goal);
        assert_eq!(wire.envelope.as_ref().unwrap().command_id, [7; 16]);
        assert_eq!(action_goal_from_wire(wire, 2_000).unwrap(), goal);
    }

    #[test]
    fn trajectory_goal_rejects_duplicate_or_zero_offsets() {
        let mut points = Vec::new();
        for _ in 0..2 {
            points.push(wire::ChassisTrajectoryPoint {
                time_from_start_ns: 0,
                target: Some(wire::Twist2 {
                    vx_mps: 0.0,
                    vy_mps: 0.0,
                    yaw_rate_radps: 0.0,
                }),
            });
        }
        assert!(trajectory_goal_from_wire(wire::ExecuteChassisTrajectoryGoal { points }).is_err());
    }

    #[test]
    fn cancel_acceptance_round_trip_is_nonterminal_decision() {
        let value = ActionCancelResponse {
            header: header(None),
            command_id: Id128::new([4; 16]).unwrap(),
            decision: ActionCancelDecision::Accepted,
            result_code: 0,
            detail: "canceling".to_owned(),
        };
        assert_eq!(
            decode_action_cancel_response(&encode_action_cancel_response(&value)).unwrap(),
            value
        );
    }
}
