use std::path::PathBuf;

use ros2_mapping::{
    Authorization, ClockMode, Direction, Durability, FrameMode, History, Interaction, Reliability,
    Ros2DdsMode, RouteRegistry, SUPPORTED_ROS2DDS_BUILD_ID, SUPPORTED_ROS2DDS_VERSION,
    SUPPORTED_ZENOH_VERSION,
};

fn profile_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../config/protocol/ros2-humble")
}

#[test]
fn checked_in_humble_profile_is_valid_and_queryable() {
    let registry = RouteRegistry::load(profile_root()).unwrap();
    assert_eq!(registry.manifest().route.len(), 8);
    assert_eq!(registry.fingerprint().len(), 64);
    let profile = &registry.manifest().profile;
    assert_eq!(profile.zenoh_version, SUPPORTED_ZENOH_VERSION);
    assert_eq!(profile.ros2dds_version, SUPPORTED_ROS2DDS_VERSION);
    assert_eq!(profile.ros2dds_build_id, SUPPORTED_ROS2DDS_BUILD_ID);
    assert_eq!(profile.ros2dds_mode, Ros2DdsMode::Standalone);

    let sensor = registry.route_by_id("imu_state").unwrap();
    assert_eq!(sensor.direction, Direction::Egress);
    assert_eq!(sensor.interaction, Interaction::SensorStream);
    assert_eq!(sensor.ros_type, "sensor_msgs/msg/Imu");
    assert_eq!(sensor.qos.reliability, Reliability::BestEffort);
    assert_eq!(sensor.qos.durability, Durability::Volatile);
    assert_eq!(sensor.qos.history, History::KeepLast);
    assert_eq!(sensor.qos.depth, 5);
    assert_eq!(sensor.clock.mode, ClockMode::RequireAcquisition);
    assert_eq!(sensor.frame.mode, FrameMode::Preserve);
    assert!(sensor.frame.use_alias_table);

    let command = registry
        .route_by_ros_name("/robot_1/cmd_vel_stamped")
        .unwrap();
    assert_eq!(command.direction, Direction::Ingress);
    assert_eq!(command.interaction, Interaction::Command);
    assert_eq!(command.ros_type, "geometry_msgs/msg/TwistStamped");
    assert_eq!(command.authorization, Authorization::LocalLeaseRequired);
    assert_eq!(command.qos.reliability, Reliability::Reliable);
    assert_eq!(command.qos.durability, Durability::Volatile);
    assert_eq!(command.qos.depth, 1);
    assert_eq!(command.frame.mode, FrameMode::BodyCommand);
    assert_eq!(command.clock.mode, ClockMode::ObserveRosIngress);
    assert!(command.clock.command_maximum_age_ns > 0);
    assert!(command.clock.construct_local_deadline);

    let service = registry.route_by_id("health_snapshot").unwrap();
    assert_eq!(service.direction, Direction::RequestReply);
    assert_eq!(service.interaction, Interaction::Service);
    assert_eq!(
        service.response_protobuf_type.as_deref(),
        Some("robot.v1.HealthSnapshotResponse")
    );
    assert_eq!(service.qos.reliability, Reliability::Reliable);
    assert_eq!(service.qos.durability, Durability::Volatile);
    assert!(service.service.as_ref().unwrap().idempotent);
    assert!(service.service.as_ref().unwrap().timeout_ns > 0);

    let action = registry.route_by_id("execute_chassis_trajectory").unwrap();
    assert_eq!(action.direction, Direction::ActionLifecycle);
    assert_eq!(action.interaction, Interaction::Action);
    assert_eq!(action.authorization, Authorization::LocalLeaseRequired);
    let action_policy = action.action.as_ref().unwrap();
    assert_eq!(
        action_policy.goal_protobuf_type,
        "robot.v1.ExecuteChassisTrajectoryGoal"
    );
    assert_eq!(
        action_policy.feedback_protobuf_type,
        "robot.v1.ExecuteChassisTrajectoryFeedback"
    );
    assert_eq!(
        action_policy.result_protobuf_type,
        "robot.v1.ExecuteChassisTrajectoryResult"
    );
    assert!(action_policy.feedback_depth > 0);
    assert!(action_policy.result_cache_depth > 0);
    assert!(action_policy.result_retention_ns > 0);
    assert!(action_policy.status_transient_local);
    assert!(
        registry
            .route_by_native_key("rs/robot/v1/device/imu/state")
            .is_some()
    );
}

#[test]
fn manifest_fingerprint_is_deterministic() {
    let first = RouteRegistry::load(profile_root()).unwrap();
    let second = RouteRegistry::load(profile_root()).unwrap();
    assert_eq!(first.fingerprint(), second.fingerprint());
    assert_eq!(
        first.fingerprint(),
        "02b1cd6b6bd4d2d7a9ae7e1f7958710c3ca2980311a97dd031525199ecce557b"
    );
}
