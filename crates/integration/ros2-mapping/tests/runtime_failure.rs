use std::path::PathBuf;

use message_model::{
    AcquisitionTimestamp, ActionEvent, ActionLifecycle, ActionStatus, AsciiId, ClockState,
    ClockStatus, DataQuality, Header, HeaderFields, Id128, ResourceLease, ResourceLeaseFields,
    TimestampDomain,
};
use ros2_mapping::{
    ComponentState, ProjectionError, Ros2DdsMode, RosTwistStampedInput, RouteAdmission,
    RouteRegistry, RuntimeError, RuntimeEvidence, RuntimeFailure, RuntimeGate, RuntimeIdentity,
    RuntimeStatus, TwistIngressContext, project_twist_ingress,
};

fn registry() -> RouteRegistry {
    RouteRegistry::load(
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../../config/protocol/ros2-humble"),
    )
    .unwrap()
}

fn identity(registry: &RouteRegistry) -> RuntimeIdentity {
    let profile = &registry.manifest().profile;
    RuntimeIdentity {
        profile_id: profile.id.clone(),
        protobuf_descriptor_sha256: profile.protobuf_descriptor_sha256.clone(),
        manifest_fingerprint: registry.fingerprint().to_owned(),
        zenoh_version: profile.zenoh_version.clone(),
        ros2dds_version: profile.ros2dds_version.clone(),
        ros2dds_build_id: profile.ros2dds_build_id.clone(),
        ros2dds_mode: profile.ros2dds_mode,
    }
}

fn ready_evidence(registry: &RouteRegistry, epoch: u64) -> RuntimeEvidence {
    RuntimeEvidence {
        epoch,
        adapter: ComponentState::Ready,
        identity: Some(identity(registry)),
        native_zenoh: ComponentState::Ready,
        ros_graph: ComponentState::Ready,
        ros2dds: ComponentState::Ready,
    }
}

#[test]
fn readiness_is_identity_checked_and_invalidated_on_restart() {
    let registry = registry();
    let mut gate = RuntimeGate::new();
    assert_eq!(gate.begin_epoch(0), Err(RuntimeError::ZeroEpoch));
    gate.begin_epoch(1).unwrap();

    let evidence = ready_evidence(&registry, 1);
    let report = gate.evaluate(&registry, &evidence).unwrap();
    assert_eq!(report.status(), RuntimeStatus::Ready);
    assert_eq!(
        gate.admission(report, registry.route_by_id("imu_state").unwrap()),
        RouteAdmission::Available
    );
    assert_eq!(
        gate.admission(
            report,
            registry.route_by_id("chassis_twist_command").unwrap()
        ),
        RouteAdmission::LocalValidationRequired
    );
    assert_eq!(
        gate.admission(
            report,
            registry.route_by_id("execute_chassis_trajectory").unwrap()
        ),
        RouteAdmission::LocalValidationRequired
    );

    gate.begin_epoch(2).unwrap();
    assert_eq!(
        gate.admission(report, registry.route_by_id("imu_state").unwrap()),
        RouteAdmission::Closed(RuntimeFailure::StaleEvidence)
    );
    assert_eq!(
        gate.evaluate(&registry, &evidence),
        Err(RuntimeError::EpochMismatch)
    );
    assert_eq!(gate.begin_epoch(2), Err(RuntimeError::NonIncreasingEpoch));

    let starting = RuntimeEvidence {
        epoch: 2,
        adapter: ComponentState::Starting,
        identity: None,
        native_zenoh: ComponentState::Stopped,
        ros_graph: ComponentState::Stopped,
        ros2dds: ComponentState::Stopped,
    };
    let starting_report = gate.evaluate(&registry, &starting).unwrap();
    assert_eq!(
        starting_report.status(),
        RuntimeStatus::Unavailable(RuntimeFailure::AdapterUnavailable)
    );

    let restarted = ready_evidence(&registry, 2);
    assert_eq!(
        gate.evaluate(&registry, &restarted).unwrap().status(),
        RuntimeStatus::Ready
    );
}

#[test]
fn identity_mismatch_and_every_external_failure_close_ingress() {
    let registry = registry();
    let mut gate = RuntimeGate::new();
    gate.begin_epoch(7).unwrap();

    let expected = identity(&registry);
    let mut wrong_profile = expected.clone();
    wrong_profile.profile_id = "other-profile".to_owned();
    let mut wrong_descriptor = expected.clone();
    wrong_descriptor.protobuf_descriptor_sha256 = "0".repeat(64);
    let mut wrong_manifest = expected.clone();
    wrong_manifest.manifest_fingerprint = "0".repeat(64);
    let mut wrong_zenoh = expected.clone();
    wrong_zenoh.zenoh_version = "1.8.0".to_owned();
    let mut wrong_ros2dds_version = expected.clone();
    wrong_ros2dds_version.ros2dds_version = "1.8.0".to_owned();
    let mut wrong_bridge = expected.clone();
    wrong_bridge.ros2dds_build_id = "unknown-build".to_owned();
    let mut wrong_ros2dds_mode = expected;
    wrong_ros2dds_mode.ros2dds_mode = Ros2DdsMode::DynamicPlugin;
    for (field, wrong_identity) in [
        ("profile_id", wrong_profile),
        ("protobuf_descriptor_sha256", wrong_descriptor),
        ("manifest_fingerprint", wrong_manifest),
        ("zenoh_version", wrong_zenoh),
        ("ros2dds_version", wrong_ros2dds_version),
        ("ros2dds_build_id", wrong_bridge),
        ("ros2dds_mode", wrong_ros2dds_mode),
    ] {
        let wrong = RuntimeEvidence {
            identity: Some(wrong_identity),
            ..ready_evidence(&registry, 7)
        };
        assert_eq!(
            gate.evaluate(&registry, &wrong),
            Err(RuntimeError::IdentityMismatch { field })
        );
    }
    let missing_identity = RuntimeEvidence {
        identity: None,
        ..ready_evidence(&registry, 7)
    };
    assert_eq!(
        gate.evaluate(&registry, &missing_identity),
        Err(RuntimeError::MissingIdentity)
    );

    let command = registry.route_by_id("chassis_twist_command").unwrap();
    for (evidence, reason) in [
        (
            RuntimeEvidence {
                adapter: ComponentState::Failed,
                ..ready_evidence(&registry, 7)
            },
            RuntimeFailure::AdapterUnavailable,
        ),
        (
            RuntimeEvidence {
                native_zenoh: ComponentState::Failed,
                ..ready_evidence(&registry, 7)
            },
            RuntimeFailure::NativeZenohUnavailable,
        ),
        (
            RuntimeEvidence {
                ros_graph: ComponentState::Failed,
                ..ready_evidence(&registry, 7)
            },
            RuntimeFailure::RosGraphUnavailable,
        ),
        (
            RuntimeEvidence {
                ros2dds: ComponentState::Stopped,
                ..ready_evidence(&registry, 7)
            },
            RuntimeFailure::Ros2DdsUnavailable,
        ),
    ] {
        let report = gate.evaluate(&registry, &evidence).unwrap();
        assert_eq!(report.status(), RuntimeStatus::Unavailable(reason));
        assert_eq!(
            gate.admission(report, command),
            RouteAdmission::Closed(reason)
        );
    }
}

#[test]
fn mapped_action_keeps_cancel_acceptance_distinct_from_terminal_result() {
    let registry = registry();
    let route = registry.route_by_id("execute_chassis_trajectory").unwrap();
    let policy = route.action.as_ref().unwrap();
    assert!(policy.feedback_depth > 0);
    assert!(policy.result_cache_depth > 0);
    assert!(policy.result_retention_ns > 0);

    let lifecycle = ActionLifecycle::Submitted
        .apply(ActionEvent::Accept)
        .unwrap()
        .apply(ActionEvent::Start)
        .unwrap()
        .apply(ActionEvent::RequestCancel)
        .unwrap()
        .apply(ActionEvent::AcceptCancel)
        .unwrap();
    assert_eq!(lifecycle, ActionLifecycle::Canceling);
    assert_eq!(
        lifecycle
            .apply(ActionEvent::Complete(ActionStatus::Cancelled))
            .unwrap(),
        ActionLifecycle::Terminal(ActionStatus::Cancelled)
    );
}

fn adapter_header(ros_stamp: u64) -> Header {
    Header::new(HeaderFields {
        source_id: AsciiId::parse("ros2-adapter").unwrap(),
        boot_id: Id128::new([1; 16]).unwrap(),
        sequence: 1,
        source_monotonic_ns: 1_000,
        publish_unix_ns: None,
        acquisition_timestamp: Some(AcquisitionTimestamp {
            timestamp_ns: ros_stamp,
            domain: TimestampDomain::Ros,
            uncertainty_ns: 0,
        }),
        valid_for_ns: 100_000_000,
        frame_id: Some(AsciiId::parse("base_link").unwrap()),
        clock_status: ClockStatus {
            state: ClockState::Local,
            uncertainty_ns: 0,
        },
        quality: DataQuality::Valid,
    })
    .unwrap()
}

#[test]
fn dds_failure_cannot_extend_a_locally_deadlined_command() {
    let registry = registry();
    let command_route = registry.route_by_id("chassis_twist_command").unwrap();
    let mut gate = RuntimeGate::new();
    gate.begin_epoch(9).unwrap();
    let ready = gate
        .evaluate(&registry, &ready_evidence(&registry, 9))
        .unwrap();
    assert_eq!(
        gate.admission(ready, command_route),
        RouteAdmission::LocalValidationRequired
    );

    let ros_stamp = 5_000_000_000;
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
        command_route,
        &registry.manifest().frame_alias,
        TwistIngressContext {
            adapter_header: adapter_header(ros_stamp),
            lease,
            command_id: Id128::new([3; 16]).unwrap(),
            current_ros_time_ns: ros_stamp + 10,
            received_monotonic_ns: 1_000,
            requested_ttl_ns: 100,
        },
    )
    .unwrap();
    assert!(command.envelope.validate_at(1_049).is_ok());

    let failed_evidence = RuntimeEvidence {
        ros_graph: ComponentState::Failed,
        ..ready_evidence(&registry, 9)
    };
    let failed = gate.evaluate(&registry, &failed_evidence).unwrap();
    assert_eq!(
        gate.admission(failed, command_route),
        RouteAdmission::Closed(RuntimeFailure::RosGraphUnavailable)
    );
    assert!(command.envelope.validate_at(1_050).is_err());

    let stale_input = RosTwistStampedInput {
        stamp_ns: ros_stamp - command_route.clock.command_maximum_age_ns - 1,
        ..input
    };
    assert_eq!(
        project_twist_ingress(
            &stale_input,
            command_route,
            &registry.manifest().frame_alias,
            TwistIngressContext {
                adapter_header: adapter_header(stale_input.stamp_ns),
                lease,
                command_id: Id128::new([4; 16]).unwrap(),
                current_ros_time_ns: ros_stamp,
                received_monotonic_ns: 1_000,
                requested_ttl_ns: 100,
            },
        ),
        Err(ProjectionError::StaleRosTimestamp)
    );
}
