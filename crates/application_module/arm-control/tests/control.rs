use arm_control::{
    ActuatorControlLimits, ArmControlAdmission, ArmControlConfig, ArmControlError,
    ArmControlIntent, LogExecutor, SimExecutor, direct_effort_proposal, mit_feed_forward_proposal,
};
use arm_model::{
    ARM_PROFILE_SCHEMA_VERSION, ArmModel, ArmProfileClass, ArmProfileSpec,
    JointAccelerationLimitSpec, KinematicGroupSpec, RotaryTransmissionSpec,
};
use arrayvec::ArrayVec;
use control_core::{LeaseEnforcementCache, LeaseEvent, LifecycleState};
use message_model::{
    ActionStatus, ArmActuatorVector, ArmControlMode, ArmControlProposal, ArmControlValue,
    ArmEffort, ArmJointKind, ArmJointObservation, ArmJointSample, ArmJointVector, ArmPosition,
    ArmProposalContext, ArmVelocity, AsciiId, ClockState, ClockStatus, CommandEnvelope,
    CommandEnvelopeFields, DataQuality, Header, HeaderFields, Id128, MAX_JOINTS, ResourceLease,
    ResourceLeaseFields, RotaryMitSetpoint, RotaryTorqueNm,
};

const SYNTHETIC_6DOF: &str = include_str!("../../arm-model/tests/fixtures/synthetic_6dof.urdf");

fn id(value: &str) -> message_model::ArmJointId {
    AsciiId::parse(value).unwrap()
}

fn token(value: u8) -> Id128 {
    Id128::new([value; 16]).unwrap()
}

fn model() -> ArmModel {
    let joints = (1..=6)
        .map(|index| id(&format!("joint_{index}")))
        .collect::<Vec<_>>();
    ArmModel::compile_urdf(
        SYNTHETIC_6DOF,
        &ArmProfileSpec {
            schema_version: ARM_PROFILE_SCHEMA_VERSION,
            name: id("synthetic_6dof"),
            class: ArmProfileClass::HostFixture,
            groups: vec![KinematicGroupSpec {
                id: id("arm"),
                base_link: id("base_link"),
                tip_link: id("tool0"),
                active_joint_order: joints.clone(),
            }],
            acceleration_limits: joints
                .iter()
                .copied()
                .map(|joint_id| JointAccelerationLimitSpec {
                    joint_id,
                    maximum: 4.0,
                })
                .collect(),
            transmissions: joints
                .iter()
                .enumerate()
                .map(|(index, joint_id)| RotaryTransmissionSpec {
                    joint_id: *joint_id,
                    actuator_id: id(&format!("motor_{}", index + 1)),
                    scale_rad_per_joint_unit: 100.0,
                    offset_rad: 0.0,
                })
                .collect(),
        },
    )
    .unwrap()
}

fn config(model: &ArmModel) -> ArmControlConfig {
    ArmControlConfig::new(
        model,
        id("arm"),
        7,
        id("motor_state"),
        token(10),
        model
            .transmissions()
            .iter()
            .map(|transmission| {
                ActuatorControlLimits::new(
                    transmission.binding(),
                    400.0,
                    400.0,
                    10.0,
                    20.0,
                    1.0,
                    1_000_000.0,
                    1_000_000.0,
                    1_000_000.0,
                )
                .unwrap()
            })
            .collect(),
    )
    .unwrap()
}

fn header(sequence: u64, observed_ns: u64) -> Header {
    Header::new(HeaderFields {
        source_id: id("motor_state"),
        boot_id: token(10),
        sequence,
        source_monotonic_ns: observed_ns,
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

fn observation(model: &ArmModel, sequence: u64, observed_ns: u64) -> ArmJointObservation {
    let group = model.group(id("arm")).unwrap();
    let mut samples = ArrayVec::new();
    for descriptor in group.joint_order().joints() {
        let position = match descriptor.kind() {
            ArmJointKind::Revolute | ArmJointKind::Continuous => {
                message_model::ArmPosition::angular_rad(0.0).unwrap()
            }
            ArmJointKind::Prismatic => message_model::ArmPosition::linear_m(0.0).unwrap(),
        };
        let velocity = match descriptor.kind() {
            ArmJointKind::Revolute | ArmJointKind::Continuous => {
                message_model::ArmVelocity::angular_radps(0.0).unwrap()
            }
            ArmJointKind::Prismatic => message_model::ArmVelocity::linear_mps(0.0).unwrap(),
        };
        let effort = match descriptor.kind() {
            ArmJointKind::Revolute | ArmJointKind::Continuous => ArmEffort::torque_nm(0.0).unwrap(),
            ArmJointKind::Prismatic => ArmEffort::force_n(0.0).unwrap(),
        };
        samples.push(ArmJointSample::new(*descriptor, position, velocity, effort).unwrap());
    }
    ArmJointObservation::new(
        header(sequence, observed_ns),
        model.profile_id(),
        id("arm"),
        samples,
    )
    .unwrap()
}

fn authority(now: u64) -> (LeaseEnforcementCache<1>, CommandEnvelope) {
    let lease = ResourceLease::new(ResourceLeaseFields {
        resource_id: id("arm"),
        owner_id: id("planner"),
        lease_id: token(20),
        session_epoch: 1,
        expires_monotonic_ns: now + 1_000,
    })
    .unwrap();
    let mut cache = LeaseEnforcementCache::new();
    cache
        .apply(LeaseEvent::StartSession { session_epoch: 1 }, now)
        .unwrap();
    cache.apply(LeaseEvent::Grant(lease), now).unwrap();
    let envelope = CommandEnvelope::new(CommandEnvelopeFields {
        header: header(1, now),
        resource_id: lease.resource_id(),
        owner_id: lease.owner_id(),
        lease_id: lease.lease_id(),
        session_epoch: lease.session_epoch(),
        command_id: token(21),
        deadline_monotonic_ns: now + 500,
    })
    .unwrap();
    (cache, envelope)
}

fn torque_proposal(config: &ArmControlConfig, torque_nm: f64) -> ArmControlProposal {
    let mut values = ArrayVec::<RotaryTorqueNm, MAX_JOINTS>::new();
    for _ in config.actuator_order().bindings() {
        values.push(RotaryTorqueNm::new(torque_nm).unwrap());
    }
    ArmControlProposal::direct_rotary_torque(
        ArmProposalContext::new(config.profile_id(), config.group_id(), config.generation())
            .unwrap(),
        ArmActuatorVector::new(config.actuator_order().clone(), values).unwrap(),
    )
}

#[test]
fn feed_forward_builders_preserve_power_and_fixed_gains() {
    let model = model();
    let config = config(&model);
    let order = config.joint_order().clone();
    let mut positions = ArrayVec::<ArmPosition, MAX_JOINTS>::new();
    let mut velocities = ArrayVec::<ArmVelocity, MAX_JOINTS>::new();
    let mut efforts = ArrayVec::<ArmEffort, MAX_JOINTS>::new();
    for descriptor in order.joints() {
        positions.push(
            match descriptor.kind() {
                ArmJointKind::Revolute | ArmJointKind::Continuous => ArmPosition::angular_rad(0.1),
                ArmJointKind::Prismatic => ArmPosition::linear_m(0.1),
            }
            .unwrap(),
        );
        velocities.push(
            match descriptor.kind() {
                ArmJointKind::Revolute | ArmJointKind::Continuous => {
                    ArmVelocity::angular_radps(0.2)
                }
                ArmJointKind::Prismatic => ArmVelocity::linear_mps(0.2),
            }
            .unwrap(),
        );
        efforts.push(
            match descriptor.kind() {
                ArmJointKind::Revolute | ArmJointKind::Continuous => ArmEffort::torque_nm(100.0),
                ArmJointKind::Prismatic => ArmEffort::force_n(100.0),
            }
            .unwrap(),
        );
    }
    let positions = ArmJointVector::new(order.clone(), positions).unwrap();
    let velocities = ArmJointVector::new(order.clone(), velocities).unwrap();
    let efforts = ArmJointVector::new(order, efforts).unwrap();

    let direct = direct_effort_proposal(&config, &efforts).unwrap();
    assert_eq!(direct.mode(), ArmControlMode::DirectRotaryTorque);
    assert!(direct.values().iter().all(|value| matches!(
        value,
        ArmControlValue::RotaryTorque(torque) if torque.get() == 1.0
    )));

    let mit = mit_feed_forward_proposal(&config, &positions, &velocities, &efforts).unwrap();
    assert_eq!(mit.mode(), ArmControlMode::Mit);
    assert!(mit.values().iter().all(|value| matches!(
        value,
        ArmControlValue::Mit(setpoint)
            if setpoint.position_rad() == 10.0
                && setpoint.velocity_radps() == 20.0
                && setpoint.kp() == 20.0
                && setpoint.kd() == 1.0
                && setpoint.feed_forward_torque_nm() == 1.0
    )));
}

#[test]
fn direct_torque_uses_fresh_position_and_zero_mit_gains() {
    let model = model();
    let config = config(&model);
    let proposal = torque_proposal(&config, 1.0);
    let now = 1_000;
    let (cache, envelope) = authority(now);
    let intent = ArmControlIntent::collision_unchecked(envelope, proposal);
    let mut admission = ArmControlAdmission::new(config);

    let admitted = admission
        .admit_collision_unchecked(
            &intent,
            &observation(&model, 1, now),
            &cache,
            LifecycleState::Running,
            ActionStatus::Executing,
            now,
        )
        .unwrap();
    assert!(admitted.actuator_commands().iter().all(|command| {
        let setpoint = command.setpoint();
        setpoint.position_rad() == 0.0
            && setpoint.velocity_radps() == 0.0
            && setpoint.kp() == 0.0
            && setpoint.kd() == 0.0
            && setpoint.feed_forward_torque_nm() == 1.0
    }));

    let mut simulation = SimExecutor::default();
    assert_eq!(simulation.execute(&admitted).len(), 6);
    assert_eq!(simulation.generation(), 1);
    let mut log = LogExecutor::<1>::new();
    log.execute(&admitted).unwrap();
    assert!(matches!(
        log.execute(&admitted),
        Err(ArmControlError::Capacity { capacity: 1 })
    ));
}

#[test]
fn mit_requires_exact_immutable_gains_and_open_lifecycles() {
    let model = model();
    let config = config(&model);
    let mut values = ArrayVec::<RotaryMitSetpoint, MAX_JOINTS>::new();
    for _ in config.actuator_order().bindings() {
        values.push(RotaryMitSetpoint::new(0.1, 0.2, 20.0, 1.0, 0.5).unwrap());
    }
    let proposal = ArmControlProposal::mit(
        ArmProposalContext::new(config.profile_id(), config.group_id(), config.generation())
            .unwrap(),
        ArmActuatorVector::new(config.actuator_order().clone(), values).unwrap(),
    );
    let now = 2_000;
    let (cache, envelope) = authority(now);
    let intent = ArmControlIntent::collision_unchecked(envelope, proposal);
    let mut admission = ArmControlAdmission::new(config);
    assert!(
        admission
            .admit_collision_unchecked(
                &intent,
                &observation(&model, 1, now),
                &cache,
                LifecycleState::Running,
                ActionStatus::Canceling,
                now,
            )
            .is_err()
    );
    let mut wrong_gains = ArrayVec::<RotaryMitSetpoint, MAX_JOINTS>::new();
    for _ in admission.config().actuator_order().bindings() {
        wrong_gains.push(RotaryMitSetpoint::new(0.1, 0.2, 21.0, 1.0, 0.5).unwrap());
    }
    let wrong_proposal = ArmControlProposal::mit(
        ArmProposalContext::new(
            admission.config().profile_id(),
            admission.config().group_id(),
            admission.config().generation(),
        )
        .unwrap(),
        ArmActuatorVector::new(admission.config().actuator_order().clone(), wrong_gains).unwrap(),
    );
    let wrong_intent = ArmControlIntent::collision_unchecked(envelope, wrong_proposal);
    assert!(matches!(
        admission.admit_collision_unchecked(
            &wrong_intent,
            &observation(&model, 1, now),
            &cache,
            LifecycleState::Running,
            ActionStatus::Executing,
            now,
        ),
        Err(ArmControlError::IdentityMismatch {
            field: "proposal.fixed_mit_gains"
        })
    ));
    let admitted = admission
        .admit_collision_unchecked(
            &intent,
            &observation(&model, 1, now),
            &cache,
            LifecycleState::Running,
            ActionStatus::Executing,
            now,
        )
        .unwrap();
    assert_eq!(admitted.actuator_commands()[0].setpoint().kp(), 20.0);
}

#[test]
fn stale_identity_sequence_limits_and_authority_fail_closed() {
    let model = model();
    let config = config(&model);
    let now = 3_000;
    let (cache, envelope) = authority(now);
    let intent = ArmControlIntent::collision_unchecked(envelope, torque_proposal(&config, 1.0));
    let mut admission = ArmControlAdmission::new(config.clone());
    admission
        .admit_collision_unchecked(
            &intent,
            &observation(&model, 1, now),
            &cache,
            LifecycleState::Running,
            ActionStatus::Executing,
            now,
        )
        .unwrap();
    assert!(matches!(
        admission.admit_collision_unchecked(
            &intent,
            &observation(&model, 1, now + 1),
            &cache,
            LifecycleState::Running,
            ActionStatus::Executing,
            now + 1,
        ),
        Err(ArmControlError::StaleSequence { .. })
    ));

    let over_limit =
        ArmControlIntent::collision_unchecked(envelope, torque_proposal(&config, 11.0));
    let mut fresh_admission = ArmControlAdmission::new(config);
    assert!(matches!(
        fresh_admission.admit_collision_unchecked(
            &over_limit,
            &observation(&model, 2, now),
            &cache,
            LifecycleState::Running,
            ActionStatus::Executing,
            now,
        ),
        Err(ArmControlError::LimitExceeded {
            field: "feed_forward_torque_nm",
            ..
        })
    ));

    let empty_cache = LeaseEnforcementCache::<1>::new();
    assert!(matches!(
        fresh_admission.admit_collision_unchecked(
            &intent,
            &observation(&model, 2, now),
            &empty_cache,
            LifecycleState::Running,
            ActionStatus::Executing,
            now,
        ),
        Err(ArmControlError::Lease(_))
    ));
}
