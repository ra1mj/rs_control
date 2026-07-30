//! Pure value projections used by a later generated ROS adapter.

use message_model::{
    AcquisitionTimestamp, ChassisSetpoint, CommandEnvelope, CovarianceStatus, Header, Id128,
    ImuMeasurement, ImuState, OdometryState, ResourceLease, SemanticError, StampedTransform,
    TimestampDomain, Twist2,
};
use thiserror::Error;

use crate::{ClockMode, FrameAlias, FrameMode, Interaction, Route};

/// ROS-header-shaped output without depending on generated ROS types.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RosHeaderProjection {
    /// Observation timestamp in the selected external clock domain.
    pub stamp_ns: u64,
    /// Explicit ROS frame after aliasing.
    pub frame_id: String,
    /// True when bridge reception time replaced acquisition time.
    pub degraded: bool,
}

/// `sensor_msgs/msg/Imu`-shaped pure value.
#[derive(Clone, Debug, PartialEq)]
pub struct RosImuProjection {
    /// Projected standard header.
    pub header: RosHeaderProjection,
    /// Quaternion in x, y, z, w order.
    pub orientation: [f64; 4],
    /// ROS 3-by-3 covariance/sentinel.
    pub orientation_covariance: [f64; 9],
    /// Angular velocity in radians per second.
    pub angular_velocity: [f64; 3],
    /// ROS 3-by-3 covariance/sentinel.
    pub angular_velocity_covariance: [f64; 9],
    /// Linear acceleration in metres per second squared.
    pub linear_acceleration: [f64; 3],
    /// ROS 3-by-3 covariance/sentinel.
    pub linear_acceleration_covariance: [f64; 9],
}

/// `nav_msgs/msg/Odometry`-shaped pure value.
#[derive(Clone, Debug, PartialEq)]
pub struct RosOdometryProjection {
    /// Projected standard header.
    pub header: RosHeaderProjection,
    /// Explicit child frame after aliasing.
    pub child_frame_id: String,
    /// Position in metres.
    pub position_m: [f64; 3],
    /// Quaternion in x, y, z, w order.
    pub orientation: [f64; 4],
    /// Row-major ROS 6-by-6 pose covariance.
    pub pose_covariance: [f64; 36],
    /// Linear velocity in metres per second.
    pub linear_mps: [f64; 3],
    /// Angular velocity in radians per second.
    pub angular_radps: [f64; 3],
    /// Row-major ROS 6-by-6 twist covariance.
    pub twist_covariance: [f64; 36],
}

/// `geometry_msgs/msg/TransformStamped`-shaped pure value.
#[derive(Clone, Debug, PartialEq)]
pub struct RosTransformProjection {
    /// Projected parent-frame header.
    pub header: RosHeaderProjection,
    /// Explicit child frame after aliasing.
    pub child_frame_id: String,
    /// Translation in metres.
    pub translation_m: [f64; 3],
    /// Quaternion in x, y, z, w order.
    pub rotation: [f64; 4],
    /// Static/dynamic route class.
    pub is_static: bool,
}

/// Pure `geometry_msgs/msg/TwistStamped` ingress value.
#[derive(Clone, Debug, PartialEq)]
pub struct RosTwistStampedInput {
    /// ROS observation timestamp; never used directly as a local deadline.
    pub stamp_ns: u64,
    /// Explicit ROS body frame.
    pub frame_id: String,
    /// Forward velocity in metres per second.
    pub vx_mps: f64,
    /// Left velocity in metres per second.
    pub vy_mps: f64,
    /// Yaw rate in radians per second.
    pub yaw_rate_radps: f64,
}

/// Local authority and clock context for one ROS command ingress conversion.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TwistIngressContext {
    /// Adapter-owned header whose acquisition time matches the ROS stamp.
    pub adapter_header: Header,
    /// Existing locally granted authority; the adapter cannot mint it.
    pub lease: ResourceLease,
    /// Canonical 16-byte command identity.
    pub command_id: Id128,
    /// Current ROS clock used only to evaluate observation age.
    pub current_ros_time_ns: u64,
    /// Local monotonic receipt used to construct the deadline.
    pub received_monotonic_ns: u64,
    /// Requested local validity window.
    pub requested_ttl_ns: u64,
}

/// Projects a validated IMU sample using the route's clock/frame policy.
pub fn project_imu(
    value: ImuState,
    route: &Route,
    aliases: &[FrameAlias],
    bridge_reception_time_ns: u64,
) -> Result<RosImuProjection, ProjectionError> {
    require_route(route, Interaction::SensorStream, "sensor_msgs/msg/Imu")?;
    let header = project_header(value.header(), route, aliases, bridge_reception_time_ns)?;
    let (orientation, orientation_covariance) = project_orientation(value.orientation());
    let (angular_velocity, angular_velocity_covariance) = project_vector(value.angular_velocity());
    let (linear_acceleration, linear_acceleration_covariance) =
        project_vector(value.linear_acceleration());
    Ok(RosImuProjection {
        header,
        orientation,
        orientation_covariance,
        angular_velocity,
        angular_velocity_covariance,
        linear_acceleration,
        linear_acceleration_covariance,
    })
}

/// Projects canonical odometry without inventing values for any covariance axis.
pub fn project_odometry(
    value: OdometryState,
    route: &Route,
    aliases: &[FrameAlias],
    bridge_reception_time_ns: u64,
) -> Result<RosOdometryProjection, ProjectionError> {
    require_route(route, Interaction::StateStream, "nav_msgs/msg/Odometry")?;
    Ok(RosOdometryProjection {
        header: project_header(value.header(), route, aliases, bridge_reception_time_ns)?,
        child_frame_id: project_frame(
            value.child_frame_id().as_str(),
            route.frame.use_alias_table,
            aliases,
        )?,
        position_m: vector_components(value.pose().position_m),
        orientation: value.pose().orientation.components(),
        pose_covariance: *value.pose_covariance().values(),
        linear_mps: vector_components(value.twist().linear_mps),
        angular_radps: vector_components(value.twist().angular_radps),
        twist_covariance: *value.twist_covariance().values(),
    })
}

/// Projects one explicitly owned static or dynamic transform.
pub fn project_transform(
    value: StampedTransform,
    route: &Route,
    aliases: &[FrameAlias],
    bridge_reception_time_ns: u64,
) -> Result<RosTransformProjection, ProjectionError> {
    let expected = if value.is_static() {
        Interaction::TransformStatic
    } else {
        Interaction::TransformDynamic
    };
    require_route(route, expected, "tf2_msgs/msg/TFMessage")?;
    Ok(RosTransformProjection {
        header: project_header(value.header(), route, aliases, bridge_reception_time_ns)?,
        child_frame_id: project_frame(
            value.child_frame_id().as_str(),
            route.frame.use_alias_table,
            aliases,
        )?,
        translation_m: vector_components(value.translation_m()),
        rotation: value.rotation().components(),
        is_static: value.is_static(),
    })
}

/// Converts a ROS TwistStamped observation into an authorized local command.
///
/// The caller must supply an existing local lease. ROS time is checked only for
/// observation age; the command deadline derives from local monotonic receipt.
pub fn project_twist_ingress(
    input: &RosTwistStampedInput,
    route: &Route,
    aliases: &[FrameAlias],
    context: TwistIngressContext,
) -> Result<ChassisSetpoint, ProjectionError> {
    require_route(
        route,
        Interaction::Command,
        "geometry_msgs/msg/TwistStamped",
    )?;
    if route.clock.mode != ClockMode::ObserveRosIngress || !route.clock.construct_local_deadline {
        return Err(ProjectionError::RoutePolicy(
            "command route lacks local deadline policy",
        ));
    }
    if input.stamp_ns == 0 {
        return Err(ProjectionError::UninitializedRosTime);
    }
    let age_ns = context
        .current_ros_time_ns
        .checked_sub(input.stamp_ns)
        .ok_or(ProjectionError::FutureRosTimestamp)?;
    if age_ns > route.clock.command_maximum_age_ns {
        return Err(ProjectionError::StaleRosTimestamp);
    }
    let canonical_frame = context
        .adapter_header
        .frame_id()
        .ok_or(ProjectionError::MissingFrame)?;
    let expected_frame = project_frame(
        canonical_frame.as_str(),
        route.frame.use_alias_table,
        aliases,
    )?;
    if input.frame_id != expected_frame {
        return Err(ProjectionError::FrameMismatch);
    }
    if context.adapter_header.acquisition_timestamp()
        != Some(AcquisitionTimestamp {
            timestamp_ns: input.stamp_ns,
            domain: TimestampDomain::Ros,
            uncertainty_ns: 0,
        })
    {
        return Err(ProjectionError::HeaderStampMismatch);
    }

    let envelope = CommandEnvelope::from_bridge_ingress(
        context.adapter_header,
        context.lease,
        context.command_id,
        context.received_monotonic_ns,
        context.requested_ttl_ns,
        route.freshness_ns,
    )?;
    Ok(ChassisSetpoint {
        envelope,
        target: Twist2::new(input.vx_mps, input.vy_mps, input.yaw_rate_radps)?,
    })
}

fn project_header(
    header: Header,
    route: &Route,
    aliases: &[FrameAlias],
    bridge_reception_time_ns: u64,
) -> Result<RosHeaderProjection, ProjectionError> {
    let (stamp_ns, degraded) = project_stamp(
        header.acquisition_timestamp(),
        route,
        bridge_reception_time_ns,
    )?;
    let frame_id = match route.frame.mode {
        FrameMode::None => String::new(),
        _ => {
            let frame = header.frame_id().ok_or(ProjectionError::MissingFrame)?;
            project_frame(frame.as_str(), route.frame.use_alias_table, aliases)?
        }
    };
    Ok(RosHeaderProjection {
        stamp_ns,
        frame_id,
        degraded,
    })
}

fn project_stamp(
    acquisition: Option<AcquisitionTimestamp>,
    route: &Route,
    bridge_reception_time_ns: u64,
) -> Result<(u64, bool), ProjectionError> {
    let trusted = acquisition.filter(|timestamp| {
        timestamp.timestamp_ns != 0
            && timestamp.uncertainty_ns <= route.clock.maximum_uncertainty_ns
    });
    match (route.clock.mode, trusted) {
        (ClockMode::RequireAcquisition, Some(timestamp)) => Ok((timestamp.timestamp_ns, false)),
        (ClockMode::RequireAcquisition, None) => Err(ProjectionError::UntrustedAcquisitionTime),
        (ClockMode::ReceptionTimeDegraded, Some(timestamp)) => Ok((timestamp.timestamp_ns, false)),
        (ClockMode::ReceptionTimeDegraded, None) if bridge_reception_time_ns != 0 => {
            Ok((bridge_reception_time_ns, true))
        }
        (ClockMode::ReceptionTimeDegraded, None) => Err(ProjectionError::UninitializedRosTime),
        (ClockMode::ObserveRosIngress, _) => Err(ProjectionError::RoutePolicy(
            "ingress clock policy cannot project an egress header",
        )),
    }
}

fn project_frame(
    canonical: &str,
    use_alias_table: bool,
    aliases: &[FrameAlias],
) -> Result<String, ProjectionError> {
    if !use_alias_table {
        return Ok(canonical.to_owned());
    }
    aliases
        .iter()
        .find(|alias| alias.canonical == canonical)
        .map(|alias| alias.ros.clone())
        .ok_or_else(|| ProjectionError::MissingFrameAlias(canonical.to_owned()))
}

fn project_orientation(
    value: ImuMeasurement<message_model::UnitQuaternion>,
) -> ([f64; 4], [f64; 9]) {
    match value {
        ImuMeasurement::Unavailable => ([0.0, 0.0, 0.0, 1.0], unavailable_covariance()),
        ImuMeasurement::Present { value, covariance } => {
            (value.components(), project_covariance(covariance))
        }
    }
}

fn project_vector(value: ImuMeasurement<message_model::Vector3>) -> ([f64; 3], [f64; 9]) {
    match value {
        ImuMeasurement::Unavailable => ([0.0; 3], unavailable_covariance()),
        ImuMeasurement::Present { value, covariance } => {
            (vector_components(value), project_covariance(covariance))
        }
    }
}

fn project_covariance(value: CovarianceStatus) -> [f64; 9] {
    match value {
        CovarianceStatus::Unknown => [0.0; 9],
        CovarianceStatus::Known(value) => *value.values(),
    }
}

fn unavailable_covariance() -> [f64; 9] {
    let mut covariance = [0.0; 9];
    covariance[0] = -1.0;
    covariance
}

fn vector_components(value: message_model::Vector3) -> [f64; 3] {
    [value.x(), value.y(), value.z()]
}

fn require_route(
    route: &Route,
    interaction: Interaction,
    ros_type: &'static str,
) -> Result<(), ProjectionError> {
    if route.interaction != interaction || route.ros_type != ros_type {
        Err(ProjectionError::RoutePolicy(
            "route interaction or ROS type does not match projection",
        ))
    } else {
        Ok(())
    }
}

/// Pure mapping or command-ingress failure.
#[derive(Debug, Error, PartialEq)]
pub enum ProjectionError {
    /// Semantic constructor rejected the value.
    #[error(transparent)]
    Semantic(#[from] SemanticError),
    /// Route policy does not support the attempted projection.
    #[error("route policy mismatch: {0}")]
    RoutePolicy(&'static str),
    /// Required source acquisition time was absent or too uncertain.
    #[error("source acquisition time is absent or outside uncertainty policy")]
    UntrustedAcquisitionTime,
    /// ROS/reception time was zero and therefore uninitialized.
    #[error("ROS time is uninitialized")]
    UninitializedRosTime,
    /// ROS input stamp is in the future relative to the supplied ROS clock.
    #[error("ROS input timestamp is in the future")]
    FutureRosTimestamp,
    /// ROS input exceeded the route age limit.
    #[error("ROS input timestamp is stale")]
    StaleRosTimestamp,
    /// A frame-bearing value omitted its frame.
    #[error("required frame is missing")]
    MissingFrame,
    /// ROS command frame did not match the configured body frame.
    #[error("ROS command frame does not match the configured body frame")]
    FrameMismatch,
    /// Adapter header acquisition time did not match the ROS input stamp.
    #[error("adapter header acquisition timestamp does not match ROS input")]
    HeaderStampMismatch,
    /// No explicit boundary alias exists for a canonical frame.
    #[error("no ROS frame alias is configured for {0}")]
    MissingFrameAlias(String),
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use message_model::{
        AcquisitionTimestamp, AsciiId, ClockState, ClockStatus, CovarianceStatus, DataQuality,
        Header, HeaderFields, Id128, ImuMeasurement, ImuState, ResourceLease, ResourceLeaseFields,
        TimestampDomain, UnitQuaternion,
    };

    use super::{
        ProjectionError, RosTwistStampedInput, TwistIngressContext, project_imu,
        project_twist_ingress,
    };
    use crate::RouteRegistry;

    fn registry() -> RouteRegistry {
        RouteRegistry::load(
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../config/protocol/ros2-humble"),
        )
        .unwrap()
    }

    fn header(frame: &str, acquisition_timestamp: Option<AcquisitionTimestamp>) -> Header {
        Header::new(HeaderFields {
            source_id: AsciiId::parse("ros2-adapter").unwrap(),
            boot_id: Id128::new([1; 16]).unwrap(),
            sequence: 1,
            source_monotonic_ns: 1_000,
            publish_unix_ns: None,
            acquisition_timestamp,
            valid_for_ns: 100_000_000,
            frame_id: Some(AsciiId::parse(frame).unwrap()),
            clock_status: ClockStatus {
                state: ClockState::Local,
                uncertainty_ns: 0,
            },
            quality: DataQuality::Valid,
        })
        .unwrap()
    }

    #[test]
    fn imu_projection_uses_exact_ros_covariance_sentinels() {
        let registry = registry();
        let route = registry.route_by_id("imu_state").unwrap();
        let value = ImuState::new(
            header(
                "imu_link",
                Some(AcquisitionTimestamp {
                    timestamp_ns: 50_000,
                    domain: TimestampDomain::Unix,
                    uncertainty_ns: 10,
                }),
            ),
            ImuMeasurement::Present {
                value: UnitQuaternion::new(0.0, 0.0, 0.0, 1.0).unwrap(),
                covariance: CovarianceStatus::Unknown,
            },
            ImuMeasurement::Unavailable,
            ImuMeasurement::Unavailable,
        )
        .unwrap();
        let projection =
            project_imu(value, route, &registry.manifest().frame_alias, 60_000).unwrap();
        assert_eq!(projection.header.stamp_ns, 50_000);
        assert_eq!(projection.header.frame_id, "robot_1/imu_link");
        assert_eq!(projection.orientation_covariance, [0.0; 9]);
        assert_eq!(projection.angular_velocity_covariance[0], -1.0);
        assert_eq!(projection.linear_acceleration_covariance[0], -1.0);
        assert!(!projection.header.degraded);
    }

    #[test]
    fn required_acquisition_does_not_silently_use_reception_time() {
        let registry = registry();
        let route = registry.route_by_id("imu_state").unwrap();
        let value = ImuState::new(
            header("imu_link", None),
            ImuMeasurement::Unavailable,
            ImuMeasurement::Unavailable,
            ImuMeasurement::Unavailable,
        )
        .unwrap();
        assert_eq!(
            project_imu(value, route, &registry.manifest().frame_alias, 60_000),
            Err(ProjectionError::UntrustedAcquisitionTime)
        );
    }

    #[test]
    fn twist_ingress_uses_ros_time_only_for_age_and_local_time_for_deadline() {
        let registry = registry();
        let route = registry.route_by_id("chassis_twist_command").unwrap();
        let ros_stamp = 5_000_000_000;
        let adapter_header = header(
            "base_link",
            Some(AcquisitionTimestamp {
                timestamp_ns: ros_stamp,
                domain: TimestampDomain::Ros,
                uncertainty_ns: 0,
            }),
        );
        let lease = ResourceLease::new(ResourceLeaseFields {
            resource_id: AsciiId::parse("chassis").unwrap(),
            owner_id: AsciiId::parse("ros2-adapter").unwrap(),
            lease_id: Id128::new([2; 16]).unwrap(),
            session_epoch: 7,
            expires_monotonic_ns: 1_050,
        })
        .unwrap();
        let input = RosTwistStampedInput {
            stamp_ns: ros_stamp,
            frame_id: "robot_1/base_link".to_owned(),
            vx_mps: 0.2,
            vy_mps: 0.0,
            yaw_rate_radps: 0.1,
        };
        let command = project_twist_ingress(
            &input,
            route,
            &registry.manifest().frame_alias,
            TwistIngressContext {
                adapter_header,
                lease,
                command_id: Id128::new([3; 16]).unwrap(),
                current_ros_time_ns: ros_stamp + 10,
                received_monotonic_ns: 1_000,
                requested_ttl_ns: 100,
            },
        )
        .unwrap();
        assert_eq!(command.envelope.deadline_monotonic_ns(), 1_050);
        assert_eq!(command.envelope.lease_id(), lease.lease_id());

        assert_eq!(
            project_twist_ingress(
                &input,
                route,
                &registry.manifest().frame_alias,
                TwistIngressContext {
                    adapter_header,
                    lease,
                    command_id: Id128::new([3; 16]).unwrap(),
                    current_ros_time_ns: ros_stamp - 1,
                    received_monotonic_ns: 1_000,
                    requested_ttl_ns: 100,
                },
            ),
            Err(ProjectionError::FutureRosTimestamp)
        );
    }
}
