use std::path::PathBuf;

use ros2_mapping::{Direction, Interaction, RouteRegistry};

fn profile_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../config/protocol/ros2-humble")
}

#[test]
fn checked_in_humble_profile_is_valid_and_queryable() {
    let registry = RouteRegistry::load(profile_root()).unwrap();
    assert_eq!(registry.manifest().route.len(), 8);
    assert_eq!(registry.fingerprint().len(), 64);

    let command = registry
        .route_by_ros_name("/robot_1/cmd_vel_stamped")
        .unwrap();
    assert_eq!(command.direction, Direction::Ingress);
    assert_eq!(command.interaction, Interaction::Command);
    assert!(command.clock.construct_local_deadline);

    let action = registry.route_by_id("execute_chassis_trajectory").unwrap();
    assert!(action.action.as_ref().unwrap().status_transient_local);
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
        "7cab3c3efdcbbe0cf97a3135b7d210f64daf661a25802d0b3316b679dcb49976"
    );
}
