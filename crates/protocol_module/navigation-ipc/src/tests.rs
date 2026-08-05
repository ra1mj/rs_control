use allocation_counter::measure;
use arrayvec::ArrayVec;
use message_model::{
    AsciiId, ChassisSetpointIntent, ClockState, ClockStatus, CommandIntent, CommandIntentFields,
    Covariance6, DataQuality, Header, HeaderFields, Id128, NavigationAuthority,
    NavigationAuthorityState, NavigationComponentState, NavigationCounters, NavigationHealth,
    NavigationReadiness, OdometryState, OdometryStateFields, Pose3, ResourceLease,
    ResourceLeaseFields, Sha256Identity, StampedTransform, StampedTransformFields, TransformBatch,
    Twist2, Twist3, UnitQuaternion, Vector3,
};
use middleware_api::{
    EventPublishError, EventPublisher, LatestPublish, LatestPublisher, LatestSubscriber,
};
use middleware_shm::{
    ChannelName, MemoryPolicy, ShmChannelConfig, ShmCodec, ShmError, ShmNamespace,
};
use tempfile::TempDir;

use crate::{
    NavigationAuthorityCodec, NavigationGuardError, NavigationIpcNamespace,
    NavigationOdometryCodec, NavigationProposalCodec, NavigationSourceGuard, NavigationTfBatch,
    NavigationTfCodec, ODOMETRY_CHANNEL, PROPOSAL_CHANNEL, wire::monotonic_now_ns,
};

const TEST_NOW: u64 = 500;

fn fixed_now() -> u64 {
    TEST_NOW
}

fn id(value: &str) -> AsciiId<64> {
    AsciiId::parse(value).unwrap()
}

fn token(value: u8) -> Id128 {
    Id128::new([value; 16]).unwrap()
}

fn header(source: &str, boot: u8, sequence: u64, now: u64, frame: Option<&str>) -> Header {
    Header::new(HeaderFields {
        source_id: id(source),
        boot_id: token(boot),
        sequence,
        source_monotonic_ns: now,
        publish_unix_ns: None,
        acquisition_timestamp: None,
        valid_for_ns: 1_000_000_000,
        frame_id: frame.map(id),
        clock_status: ClockStatus {
            state: ClockState::Local,
            uncertainty_ns: 0,
        },
        quality: DataQuality::Valid,
    })
    .unwrap()
}

fn lease(expires: u64) -> ResourceLease {
    ResourceLease::new(ResourceLeaseFields {
        resource_id: id("chassis"),
        owner_id: id("navigation-mission"),
        lease_id: token(3),
        session_epoch: 7,
        expires_monotonic_ns: expires,
    })
    .unwrap()
}

fn proposal(sequence: u64, now: u64) -> ChassisSetpointIntent {
    ChassisSetpointIntent::new(
        CommandIntent::new(CommandIntentFields {
            header: header("navigation-bridge", 2, sequence, now, None),
            resource_id: id("chassis"),
            owner_id: id("navigation-mission"),
            lease_id: token(3),
            session_epoch: 7,
            command_id: token(9),
            requested_ttl_ns: 100_000_000,
        })
        .unwrap(),
        Twist2::new(0.2, -0.1, 0.3).unwrap(),
    )
}

fn pose(x: f64, y: f64) -> Pose3 {
    Pose3 {
        position_m: Vector3::new(x, y, 0.0).unwrap(),
        orientation: UnitQuaternion::new(0.0, 0.0, 0.0, 1.0).unwrap(),
    }
}

fn odometry(sequence: u64, now: u64) -> OdometryState {
    OdometryState::new(OdometryStateFields {
        header: header("rs-navigation-e2e", 4, sequence, now, Some("odom")),
        parent_frame_id: id("odom"),
        child_frame_id: id("base_link"),
        pose: pose(1.0, -2.0),
        twist: Twist3 {
            linear_mps: Vector3::new(0.2, -0.1, 0.0).unwrap(),
            angular_radps: Vector3::new(0.0, 0.0, 0.3).unwrap(),
        },
        pose_covariance: Covariance6::new([0.0; 36]).unwrap(),
        twist_covariance: Covariance6::new([0.0; 36]).unwrap(),
    })
    .unwrap()
}

fn transform(sequence: u64, now: u64, parent: &str, child: &str) -> StampedTransform {
    StampedTransform::new(StampedTransformFields {
        header: header("rs-navigation-e2e", 4, sequence, now, Some(parent)),
        parent_frame_id: id(parent),
        child_frame_id: id(child),
        translation_m: Vector3::new(0.0, 0.0, 0.0).unwrap(),
        rotation: UnitQuaternion::new(0.0, 0.0, 0.0, 1.0).unwrap(),
        is_static: false,
    })
    .unwrap()
}

#[test]
fn authority_proposal_odometry_and_tf_round_trip_with_local_expiry() {
    let authority = NavigationAuthorityState {
        header: header("rs-navigation-e2e", 4, 1, 100, None),
        authority: NavigationAuthority::Available {
            lease: lease(1_000),
            active_command_id: Some(token(9)),
        },
    };
    let mut authority_codec = NavigationAuthorityCodec::new(fixed_now);
    let authority_repr = authority_codec.encode(&authority);
    assert_eq!(authority_codec.decode(authority_repr).unwrap(), authority);

    let proposal = proposal(1, 100);
    let mut proposal_codec = NavigationProposalCodec::new(fixed_now);
    let proposal_repr = proposal_codec.encode(&proposal);
    assert_eq!(proposal_codec.decode(proposal_repr).unwrap(), proposal);

    let odometry = odometry(1, 100);
    let mut odometry_codec = NavigationOdometryCodec::new(fixed_now);
    let odometry_repr = odometry_codec.encode(&odometry);
    assert_eq!(odometry_codec.decode(odometry_repr).unwrap(), odometry);

    let mut transforms = ArrayVec::new();
    transforms.push(transform(1, 100, "map", "odom"));
    transforms.push(transform(1, 100, "odom", "base_link"));
    let batch = NavigationTfBatch::new(TransformBatch::new(transforms).unwrap()).unwrap();
    let mut tf_codec = NavigationTfCodec::new(fixed_now);
    let tf_repr = tf_codec.encode(&batch);
    assert_eq!(tf_codec.decode(tf_repr).unwrap(), batch);

    let mut expired_codec = NavigationProposalCodec::new(|| 1_000_000_100);
    assert!(expired_codec.decode(proposal_repr).is_err());
}

#[test]
fn proposal_and_odometry_codec_hot_paths_allocate_nothing() {
    let proposal = proposal(1, 100);
    let odometry = odometry(1, 100);
    let mut proposal_codec = NavigationProposalCodec::new(fixed_now);
    let mut odometry_codec = NavigationOdometryCodec::new(fixed_now);
    let proposal_repr = proposal_codec.encode(&proposal);
    let odometry_repr = odometry_codec.encode(&odometry);
    let allocations = measure(|| {
        let encoded_proposal = proposal_codec.encode(&proposal);
        assert_eq!(proposal_codec.decode(encoded_proposal).unwrap(), proposal);
        let encoded_odometry = odometry_codec.encode(&odometry);
        assert_eq!(odometry_codec.decode(encoded_odometry).unwrap(), odometry);
        assert_eq!(proposal_codec.decode(proposal_repr).unwrap(), proposal);
        assert_eq!(odometry_codec.decode(odometry_repr).unwrap(), odometry);
    });
    assert_eq!(allocations.count_total, 0);
}

#[test]
fn source_guard_rejects_duplicate_reorder_prior_boot_and_disconnect_replay() {
    let mut guard = NavigationSourceGuard::<4>::new(id("navigation-bridge"));
    assert_eq!(
        guard.accept(header("navigation-bridge", 1, 2, 100, None), TEST_NOW),
        Ok(())
    );
    assert_eq!(
        guard.accept(header("navigation-bridge", 1, 2, 100, None), TEST_NOW),
        Err(NavigationGuardError::Duplicate)
    );
    assert_eq!(
        guard.accept(header("navigation-bridge", 1, 1, 100, None), TEST_NOW),
        Err(NavigationGuardError::Reordered)
    );
    assert_eq!(
        guard.accept(header("navigation-bridge", 2, 1, 100, None), TEST_NOW),
        Ok(())
    );
    assert_eq!(
        guard.accept(header("navigation-bridge", 1, 3, 100, None), TEST_NOW),
        Err(NavigationGuardError::PriorBoot)
    );
    guard.disconnect();
    assert_eq!(
        guard.accept(header("navigation-bridge", 2, 2, 100, None), TEST_NOW),
        Err(NavigationGuardError::PriorBoot)
    );
}

#[test]
fn exact_layouts_attach_latest_overwrite_and_reject_wrong_schema() {
    let temp = TempDir::new().unwrap();
    let root = temp.path().join("navigation-ipc");
    let namespace = NavigationIpcNamespace::new(&root).unwrap();
    namespace.create_layouts().unwrap();

    let mut publisher = namespace.attach_proposal_publisher().unwrap();
    let mut subscriber = namespace.attach_proposal_subscriber().unwrap();
    let now = monotonic_now_ns();
    assert!(matches!(
        publisher.try_publish(proposal(1, now)),
        LatestPublish::Published {
            overwritten: false,
            ..
        }
    ));
    assert!(matches!(
        publisher.try_publish(proposal(2, now)),
        LatestPublish::Published {
            overwritten: true,
            ..
        }
    ));
    let sample = subscriber.try_read_latest().unwrap();
    assert_eq!(sample.value.intent().header().sequence(), 2);
    assert_eq!(publisher.counters().overwritten, 1);

    let raw = ShmNamespace::new(&root).unwrap();
    let wrong = raw.attach_latest_publisher::<ChassisSetpointIntent, _>(
        ChannelName::new(ODOMETRY_CHANNEL).unwrap(),
        ShmChannelConfig::new(3, MemoryPolicy::Prefault).unwrap(),
        NavigationProposalCodec::default(),
    );
    assert!(matches!(
        wrong,
        Err(ShmError::Layout(_) | ShmError::FileLength { .. })
    ));
}

#[test]
fn ordered_authority_queue_is_bounded_and_health_is_fixed() {
    let temp = TempDir::new().unwrap();
    let root = temp.path().join("navigation-ipc");
    let namespace = NavigationIpcNamespace::new(&root).unwrap();
    namespace.create_layouts().unwrap();
    let mut publisher = namespace.attach_authority_publisher().unwrap();
    let _subscriber = namespace.attach_authority_subscriber().unwrap();
    let now = monotonic_now_ns();
    let authority = NavigationAuthorityState {
        header: header("rs-navigation-e2e", 4, 1, now, None),
        authority: NavigationAuthority::Available {
            lease: lease(now.saturating_add(1_000_000_000)),
            active_command_id: None,
        },
    };
    for _ in 0..8 {
        assert!(publisher.try_publish(authority).is_ok());
    }
    assert_eq!(
        publisher.try_publish(authority),
        Err(EventPublishError::Full(authority))
    );

    let mut faults = ArrayVec::new();
    faults.push(101_u32);
    faults.push(102_u32);
    let health = NavigationHealth::new(
        header("navigation-bridge", 5, 1, now, None),
        1,
        Sha256Identity::new([7; 32], "manifest").unwrap(),
        NavigationComponentState::Ready,
        NavigationReadiness {
            manifest: true,
            action_server: true,
            nav2_lifecycle: true,
            map: true,
            localization: true,
            ipc: true,
        },
        faults,
        NavigationCounters::default(),
    )
    .unwrap();
    let mut codec = crate::NavigationHealthCodec::new(fixed_now);
    let repr = codec.encode(&NavigationHealth {
        header: header("navigation-bridge", 5, 1, 100, None),
        ..health
    });
    assert!(codec.decode(repr).unwrap().readiness.all_ready());

    assert_eq!(PROPOSAL_CHANNEL, "navigation.proposal.v1");
}
