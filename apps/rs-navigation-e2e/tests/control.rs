use allocation_counter::measure;
use message_model::{
    AsciiId, ChassisSetpointIntent, ClockState, ClockStatus, CommandIntent, CommandIntentFields,
    DataQuality, Header, HeaderFields, Id128, ResourceLease, Twist2,
};
use navigation_ipc::NavigationGuardError;
use rs_navigation_e2e::{HarnessError, MAX_PROPOSAL_TTL_NS, MotorDisabledHarness, StopReason};

fn id(value: &str) -> AsciiId<64> {
    AsciiId::parse(value).unwrap()
}

fn token(value: u8) -> Id128 {
    Id128::new([value; 16]).unwrap()
}

fn harness(now: u64, lease_duration: u64) -> MotorDisabledHarness {
    MotorDisabledHarness::new(now, lease_duration, token(1), id("navigation-bridge")).unwrap()
}

struct ProposalSpec<'a> {
    lease: ResourceLease,
    owner: &'a str,
    boot: u8,
    sequence: u64,
    source_time: u64,
    valid_for: u64,
    target: [f64; 3],
}

fn proposal(spec: ProposalSpec<'_>) -> ChassisSetpointIntent {
    ChassisSetpointIntent::new(
        CommandIntent::new(CommandIntentFields {
            header: Header::new(HeaderFields {
                source_id: id("navigation-bridge"),
                boot_id: token(spec.boot),
                sequence: spec.sequence,
                source_monotonic_ns: spec.source_time,
                publish_unix_ns: None,
                acquisition_timestamp: None,
                valid_for_ns: spec.valid_for,
                frame_id: None,
                clock_status: ClockStatus {
                    state: ClockState::Local,
                    uncertainty_ns: 0,
                },
                quality: DataQuality::Valid,
            })
            .unwrap(),
            resource_id: spec.lease.resource_id(),
            owner_id: id(spec.owner),
            lease_id: spec.lease.lease_id(),
            session_epoch: spec.lease.session_epoch(),
            command_id: token(8),
            requested_ttl_ns: MAX_PROPOSAL_TTL_NS,
        })
        .unwrap(),
        Twist2::new(spec.target[0], spec.target[1], spec.target[2]).unwrap(),
    )
}

#[test]
fn nominal_command_uses_final_limiter_mecanum_fake_sink_and_feedback() {
    let mut harness = harness(100, 2_000);
    let lease = harness.active_lease(100).unwrap();
    let output = harness
        .accept_proposal(
            proposal(ProposalSpec {
                lease,
                owner: "navigation-mission",
                boot: 2,
                sequence: 1,
                source_time: 100,
                valid_for: MAX_PROPOSAL_TTL_NS,
                target: [5.0, -5.0, 4.0],
            }),
            100,
            0.02,
        )
        .unwrap();
    assert!(output.body_constraints.is_limited());
    assert!(output.wheel_radps.iter().all(|value| value.abs() <= 20.0));
    assert_eq!(harness.sink().latest().unwrap().command_id, Some(token(8)));
    let (odometry, tf) = harness.feedback(1, 101).unwrap();
    assert_eq!(odometry.parent_frame_id().as_str(), "odom");
    assert_eq!(odometry.child_frame_id().as_str(), "base_link");
    assert_eq!(tf.batch().transforms().len(), 2);
    assert_eq!(tf.batch().transforms()[0].parent_frame_id().as_str(), "map");
}

#[test]
fn wrong_owner_expiry_stale_duplicate_reorder_and_prior_boot_never_reach_sink() {
    let mut wrong_owner = harness(100, 2_000);
    let lease = wrong_owner.active_lease(100).unwrap();
    assert!(matches!(
        wrong_owner.accept_proposal(
            proposal(ProposalSpec {
                lease,
                owner: "foreign-owner",
                boot: 2,
                sequence: 1,
                source_time: 100,
                valid_for: 100,
                target: [0.1, 0.0, 0.0],
            }),
            100,
            0.02,
        ),
        Err(HarnessError::Semantic(_))
    ));
    assert!(wrong_owner.sink().samples().is_empty());

    let mut expired = harness(100, 50);
    let expired_lease = expired.active_lease(100).unwrap();
    assert!(matches!(
        expired.accept_proposal(
            proposal(ProposalSpec {
                lease: expired_lease,
                owner: "navigation-mission",
                boot: 2,
                sequence: 1,
                source_time: 200,
                valid_for: 100,
                target: [0.1, 0.0, 0.0],
            }),
            200,
            0.02,
        ),
        Err(HarnessError::NoActiveLease)
    ));

    let mut ordered = harness(100, 2_000);
    let lease = ordered.active_lease(100).unwrap();
    let make = |boot, sequence, source_time, valid_for| {
        proposal(ProposalSpec {
            lease,
            owner: "navigation-mission",
            boot,
            sequence,
            source_time,
            valid_for,
            target: [0.1, 0.0, 0.0],
        })
    };
    assert!(
        ordered
            .accept_proposal(make(2, 1, 100, 100), 100, 0.02)
            .is_ok()
    );
    assert!(matches!(
        ordered.accept_proposal(make(2, 1, 100, 100), 100, 0.02),
        Err(HarnessError::Source(NavigationGuardError::Duplicate))
    ));
    assert!(matches!(
        ordered.accept_proposal(make(2, 0, 100, 100), 100, 0.02),
        Err(HarnessError::Source(NavigationGuardError::Reordered))
    ));
    assert!(
        ordered
            .accept_proposal(make(3, 1, 100, 100), 100, 0.02)
            .is_ok()
    );
    assert!(matches!(
        ordered.accept_proposal(make(2, 2, 100, 100), 100, 0.02),
        Err(HarnessError::Source(NavigationGuardError::PriorBoot))
    ));
    assert!(matches!(
        ordered.accept_proposal(make(3, 2, 0, 100), 100, 0.02),
        Err(HarnessError::Source(NavigationGuardError::Freshness))
    ));
    assert_eq!(ordered.sink().samples().len(), 2);
}

#[test]
fn proposal_timeout_and_peer_loss_emit_bounded_zero_decisions() {
    let mut harness = harness(100, 2_000_000_000);
    let lease = harness.active_lease(100).unwrap();
    harness
        .accept_proposal(
            proposal(ProposalSpec {
                lease,
                owner: "navigation-mission",
                boot: 2,
                sequence: 1,
                source_time: 100,
                valid_for: MAX_PROPOSAL_TTL_NS,
                target: [0.2, 0.0, 0.0],
            }),
            100,
            0.02,
        )
        .unwrap();
    assert_eq!(
        harness.tick(100 + MAX_PROPOSAL_TTL_NS).unwrap(),
        Some(StopReason::ProposalExpired)
    );
    let stopped = harness.sink().latest().unwrap();
    assert_eq!(stopped.body.components(), [0.0; 3]);
    assert_eq!(stopped.stop_reason, Some(StopReason::ProposalExpired));
    harness.peer_lost(200 + MAX_PROPOSAL_TTL_NS).unwrap();
    assert_eq!(
        harness.sink().latest().unwrap().stop_reason,
        Some(StopReason::PeerUnavailable)
    );
}

#[test]
fn admitted_control_and_fake_sink_hot_path_allocates_nothing() {
    let mut harness = harness(100, 2_000_000_000);
    let lease = harness.active_lease(100).unwrap();
    let proposal = proposal(ProposalSpec {
        lease,
        owner: "navigation-mission",
        boot: 2,
        sequence: 1,
        source_time: 100,
        valid_for: MAX_PROPOSAL_TTL_NS,
        target: [0.2, -0.1, 0.1],
    });
    let allocations = measure(|| {
        let _ = harness.accept_proposal(proposal, 100, 0.02).unwrap();
    });
    assert_eq!(allocations.count_total, 0);
}
