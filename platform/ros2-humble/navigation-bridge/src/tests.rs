use geometry_msgs::msg::{PoseStamped, Twist, Vector3 as RosVector3};
use message_model::{
    ActionStatus, AsciiId, ClockState, ClockStatus, DataQuality, Header, HeaderFields, Id128,
    NavigateToPoseGoal, NavigationAuthority, NavigationAuthorityState, NavigationMapIdentity,
    Pose3, ResourceLease, ResourceLeaseFields, Sha256Identity, UnitQuaternion, Vector3,
};
use nav2_msgs::action::NavigateToPose_Feedback;
use navigation_ipc::NavigationActionResponseKind;

use crate::bridge::{
    feedback_from_ros, goal_to_ros, ActionCorrelation, AdmissionError, BridgeAdmission,
    CorrelationError, GoalBinding,
};

fn id(value: &str) -> AsciiId<64> {
    AsciiId::parse(value).unwrap()
}

fn token(value: u8) -> Id128 {
    Id128::new([value; 16]).unwrap()
}

fn header(now: u64) -> Header {
    Header::new(HeaderFields {
        source_id: id("rs-navigation-e2e"),
        boot_id: token(1),
        sequence: 1,
        source_monotonic_ns: now,
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

fn authority(now: u64, active: bool) -> NavigationAuthorityState {
    NavigationAuthorityState {
        header: header(now),
        authority: NavigationAuthority::Available {
            lease: ResourceLease::new(ResourceLeaseFields {
                resource_id: id("chassis"),
                owner_id: id("navigation-mission"),
                lease_id: token(2),
                session_epoch: 1,
                expires_monotonic_ns: now + 100,
            })
            .unwrap(),
            active_command_id: active.then_some(token(3)),
        },
    }
}

#[test]
fn twist_requires_current_authority_and_exact_active_goal() {
    let mut bridge = BridgeAdmission::new(token(8));
    assert_eq!(
        bridge.accept_twist(Twist::default(), 10),
        Err(AdmissionError::AuthorityUnavailable)
    );
    bridge.update_authority(authority(10, false), 10).unwrap();
    assert_eq!(
        bridge.accept_twist(Twist::default(), 10),
        Err(AdmissionError::NoActiveGoal)
    );
    bridge.update_authority(authority(10, true), 10).unwrap();
    let intent = bridge
        .accept_twist(
            Twist {
                linear: RosVector3 {
                    x: 0.2,
                    y: -0.1,
                    z: 0.0,
                },
                angular: RosVector3 {
                    x: 0.0,
                    y: 0.0,
                    z: 0.3,
                },
            },
            10,
        )
        .unwrap();
    assert_eq!(intent.intent().command_id(), token(3));
    assert_eq!(intent.target().components(), [0.2, -0.1, 0.3]);
    assert_eq!(
        bridge.accept_twist(Twist::default(), 110),
        Err(AdmissionError::ExpiredAuthority)
    );
}

#[test]
fn goal_and_feedback_mapping_preserve_frames_pose_and_units() {
    let pose = Pose3 {
        position_m: Vector3::new(1.0, -2.0, 0.0).unwrap(),
        orientation: UnitQuaternion::new(0.0, 0.0, 0.0, 1.0).unwrap(),
    };
    let goal = NavigateToPoseGoal {
        map: NavigationMapIdentity::new(
            id("warehouse_sim_map"),
            id("0.1.0"),
            Sha256Identity::new([4; 32], "map").unwrap(),
            id("map"),
        ),
        target: pose,
        behavior_tree_id: id("navigate_to_pose_fail_closed"),
    };
    let ros_goal = goal_to_ros(goal);
    assert_eq!(ros_goal.pose.header.frame_id, "map");
    assert_eq!(ros_goal.pose.pose.position.x, 1.0);
    assert!(ros_goal.behavior_tree.is_empty());

    let feedback = feedback_from_ros(NavigateToPose_Feedback {
        current_pose: PoseStamped {
            header: ros_goal.pose.header,
            pose: ros_goal.pose.pose,
        },
        navigation_time: builtin_interfaces::msg::Duration { sec: 2, nanosec: 3 },
        estimated_time_remaining: builtin_interfaces::msg::Duration { sec: 4, nanosec: 5 },
        number_of_recoveries: 2,
        distance_remaining: 1.25,
    })
    .unwrap();
    assert_eq!(feedback.navigation_time_ns, 2_000_000_003);
    assert_eq!(feedback.estimated_time_remaining_ns, Some(4_000_000_005));
    assert_eq!(feedback.recoveries, 2);
    assert_eq!(feedback.distance_remaining_m, 1.25);
}

#[test]
fn action_correlation_is_bounded_and_preserves_canonical_id() {
    let pose = Pose3 {
        position_m: Vector3::new(1.0, -2.0, 0.0).unwrap(),
        orientation: UnitQuaternion::new(0.0, 0.0, 0.0, 1.0).unwrap(),
    };
    let goal = NavigateToPoseGoal {
        map: NavigationMapIdentity::new(
            id("warehouse_sim_map"),
            id("0.1.0"),
            Sha256Identity::new([4; 32], "map").unwrap(),
            id("map"),
        ),
        target: pose,
        behavior_tree_id: id("navigate_to_pose_fail_closed"),
    };
    let mut correlation = ActionCorrelation::new(token(8));
    let binding = correlation.bind(token(3), [9; 16], goal).unwrap();
    assert_eq!(correlation.binding(token(3)).unwrap(), binding);
    assert!(matches!(
        correlation.bind(token(4), [8; 16], goal),
        Err(CorrelationError::Busy)
    ));
    assert!(matches!(
        correlation.feedback(
            GoalBinding {
                command_id: token(3),
                ros_goal_uuid: [7; 16],
            },
            NavigateToPose_Feedback::default(),
            10,
        ),
        Err(CorrelationError::WrongRosGoal)
    ));

    let feedback = correlation
        .feedback(
            binding,
            NavigateToPose_Feedback {
                current_pose: PoseStamped {
                    header: std_msgs::msg::Header::default(),
                    pose: geometry_msgs::msg::Pose {
                        position: geometry_msgs::msg::Point {
                            x: 2.0,
                            y: 3.0,
                            z: 0.0,
                        },
                        orientation: geometry_msgs::msg::Quaternion {
                            x: 0.0,
                            y: 0.0,
                            z: 0.0,
                            w: 1.0,
                        },
                    },
                },
                navigation_time: builtin_interfaces::msg::Duration { sec: 1, nanosec: 0 },
                estimated_time_remaining: builtin_interfaces::msg::Duration::default(),
                number_of_recoveries: 0,
                distance_remaining: 0.5,
            },
            10,
        )
        .unwrap();
    let NavigationActionResponseKind::Feedback(feedback) = feedback.kind() else {
        panic!("expected action feedback");
    };
    assert_eq!(feedback.command_id, token(3));
    assert_eq!(feedback.status, ActionStatus::Executing);

    let cancellation = correlation
        .cancellation(binding, true, 0, "accepted".to_owned(), 11)
        .unwrap();
    assert!(matches!(
        cancellation.kind(),
        NavigationActionResponseKind::Cancel(_)
    ));
    let result = correlation
        .finish(
            binding,
            ActionStatus::Cancelled,
            0,
            "cancelled".to_owned(),
            12,
        )
        .unwrap();
    let NavigationActionResponseKind::Result(result) = result.kind() else {
        panic!("expected terminal result");
    };
    assert_eq!(result.command_id, token(3));
    assert_eq!(result.status, ActionStatus::Cancelled);
    assert!(correlation.is_idle());
}
