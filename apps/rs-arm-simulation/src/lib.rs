//! Deterministic, host-only articulated-arm qualification simulation.

use arm_algorithms::{
    ArmAlgorithmError,
    adaptive_gravity::{
        AdaptiveGravityConfig, AdaptiveGravityError, AdaptiveGravityEstimator,
        AdaptiveSampleContext, PayloadParameters,
    },
    dynamics::{ArmDynamics, AttachedPayloadError, AttachedPayloadInertia, PreparedGravity},
    kinematics::{IkSolveConfig, IkSolverKind, KinematicChain, PreparedKinematics},
};
use arm_control::{
    ActuatorControlLimits, ArmControlAdmission, ArmControlConfig, ArmControlIntent, SimExecutor,
    direct_effort_proposal, mit_feed_forward_proposal,
};
use arm_model::{
    ARM_PROFILE_SCHEMA_VERSION, ArmModel, ArmModelError, ArmProfileClass, ArmProfileSpec,
    JointAccelerationLimitSpec, KinematicGroupSpec, RotaryTransmissionSpec,
};
use arrayvec::ArrayVec;
use control_core::{LeaseEnforcementCache, LeaseEvent, LifecycleState};
use message_model::{
    ActionStatus, ArmEffort, ArmJointKind, ArmJointObservation, ArmJointOrder, ArmJointSample,
    ArmJointVector, ArmPosition, ArmVelocity, AsciiId, ClockState, ClockStatus, CommandEnvelope,
    CommandEnvelopeFields, DataQuality, Header, HeaderFields, Id128, MAX_JOINTS, ResourceLease,
    ResourceLeaseFields,
};
use serde::Serialize;
use sha2::{Digest, Sha256};
use thiserror::Error;

const FIXTURE_URDF: &str =
    include_str!("../../../crates/application_module/arm-model/tests/fixtures/synthetic_6dof.urdf");
const GENERATION: u64 = 1;
const GRAVITY: [f64; 3] = [0.0, 0.0, -9.81];
const DT_S: f64 = 0.0002;
const CALIBRATION_POSES: usize = 20;
const HELD_OUT_POSES: usize = 8;
const HOLD_STEPS: usize = 2_000;
const SETTLING_STEPS: usize = 800;
const TRUE_MASS_KG: f64 = 2.4;
const TRUE_COG_M: [f64; 3] = [0.08, -0.025, 0.12];

/// Validated fixed-step host simulation budget.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimulationConfig {
    timestep_s: f64,
    maximum_steps: usize,
}

impl SimulationConfig {
    /// Requires a finite positive timestep and a non-zero bounded step budget.
    pub fn new(timestep_s: f64, maximum_steps: usize) -> Result<Self, SimulationError> {
        if !timestep_s.is_finite()
            || timestep_s <= 0.0
            || maximum_steps == 0
            || maximum_steps > 1_000_000
        {
            return Err(SimulationError::InvalidConfiguration);
        }
        Ok(Self {
            timestep_s,
            maximum_steps,
        })
    }

    /// Fixed integration step in seconds.
    #[must_use]
    pub const fn timestep_s(self) -> f64 {
        self.timestep_s
    }

    /// Maximum allowed integration steps.
    #[must_use]
    pub const fn maximum_steps(self) -> usize {
        self.maximum_steps
    }
}

/// Canonical deterministic qualification report.
#[derive(Clone, Debug, Serialize, PartialEq)]
pub struct QualificationReport {
    pub schema_version: u32,
    pub safety_scope: &'static str,
    pub seed: u64,
    pub model_generation: u64,
    pub timestep_s: f64,
    pub integration: &'static str,
    pub control_path: &'static str,
    pub calibration_pose_windows: usize,
    pub held_out_poses: usize,
    pub rls_accepted_updates: u32,
    pub payload_truth: PayloadReport,
    pub payload_estimate: PayloadReport,
    pub mass_relative_error: f64,
    pub cog_error_m: f64,
    pub held_out_torque_rmse_nm: f64,
    pub held_out_peak_torque_nm: f64,
    pub held_out_normalized_rmse: f64,
    pub mismatch_nominal_holding_rms: f64,
    pub mismatch_adaptive_holding_rms: f64,
    pub mismatch_improvement: f64,
    pub matched_degradation: f64,
    pub fk_ik_position_error_m: f64,
    pub fk_ik_orientation_error_rad: f64,
    pub aba_steps: usize,
    pub admitted_control_steps: usize,
    pub gates: QualificationGates,
    pub functional_sha256: String,
}

#[derive(Clone, Copy, Debug, Serialize, PartialEq)]
pub struct PayloadReport {
    pub mass_kg: f64,
    pub cog_m: [f64; 3],
}

#[derive(Clone, Copy, Debug, Serialize, PartialEq)]
pub struct QualificationGates {
    pub torque_accuracy: bool,
    pub mismatch_improvement: bool,
    pub matched_model: bool,
    pub mass_accuracy: bool,
    pub cog_accuracy: bool,
    pub bounded_calibration: bool,
    pub bounded_rls: bool,
    pub fk_ik_round_trip: bool,
    pub all_passed: bool,
}

impl QualificationReport {
    /// Stable pretty JSON including a hash of the same report with an empty
    /// hash field.
    pub fn canonical_json(&self) -> Result<String, SimulationError> {
        Ok(format!("{}\n", serde_json::to_string_pretty(self)?))
    }
}

/// Runs the deterministic qualification entirely in process.
pub fn run_qualification() -> Result<QualificationReport, SimulationError> {
    let simulation_config = SimulationConfig::new(DT_S, HOLD_STEPS)?;
    let model = fixture_model()?;
    let chain = KinematicChain::new(&model, id("arm"))?;
    let prepared_gravity = PreparedGravity::new(chain, GRAVITY, GENERATION)?;
    let mut gravity_workspace = prepared_gravity.workspace();
    let payload = payload_inertia()?;
    let mut nominal_oracle = ArmDynamics::new(chain, GRAVITY)?;
    let mut plant_oracle = ArmDynamics::new_with_payload(chain, GRAVITY, payload)?;
    let mut estimator = AdaptiveGravityEstimator::new(
        &prepared_gravity,
        AdaptiveGravityConfig::host_default()
            .with_payload_bounds([0.1, 8.0], [[-0.25, 0.25], [-0.25, 0.25], [-0.25, 0.25]])?,
    )?;
    estimator.start_collection();
    let context = AdaptiveSampleContext {
        generation: GENERATION,
        fresh: true,
        settled: true,
        saturated: false,
        contact_free: true,
    };
    let dof = chain.dof();
    let mut regressor = vec![0.0; dof * 4];

    for pose_index in 0..CALIBRATION_POSES {
        let positions = pose(chain, pose_index, 0.37)?;
        prepared_gravity.payload_regressor(&positions, &mut gravity_workspace, &mut regressor)?;
        let residual = payload_residual(&mut nominal_oracle, &mut plant_oracle, &positions)?;
        estimator.observe_window(context, &regressor, &residual, 1.0)?;
    }

    estimator.solve_batch()?;
    let mut held_regressor = Vec::with_capacity(HELD_OUT_POSES * dof * 4);
    let mut held_residual = Vec::with_capacity(HELD_OUT_POSES * dof);
    for pose_index in 0..HELD_OUT_POSES {
        let positions = pose(chain, pose_index + 101, 0.51)?;
        prepared_gravity.payload_regressor(&positions, &mut gravity_workspace, &mut regressor)?;
        held_regressor.extend_from_slice(&regressor);
        held_residual.extend_from_slice(&payload_residual(
            &mut nominal_oracle,
            &mut plant_oracle,
            &positions,
        )?);
    }
    let held_out_normalized_rmse =
        estimator.validate_candidate(&held_regressor, &held_residual, 0.05)?;
    let estimate = estimator.activate()?;

    // Exercise the recursive path on exact estimator-visible rows. The
    // already-converged batch estimate should remain inside tight projection.
    for (row, measured) in held_residual.iter().copied().enumerate().take(64) {
        let base = row * 4;
        estimator.update_rls(
            context,
            [
                held_regressor[base],
                held_regressor[base + 1],
                held_regressor[base + 2],
                held_regressor[base + 3],
            ],
            measured,
        )?;
    }
    let estimate = estimator.active_parameters().unwrap_or(estimate);
    let (torque_rmse, peak_torque) = torque_metrics(&held_regressor, &held_residual, estimate);

    let hold_target = pose(chain, 211, 0.43)?;
    let mismatch_nominal_holding_rms = hold_rms(&model, payload, None, &hold_target)?;
    let mismatch_adaptive_holding_rms = hold_rms(&model, payload, Some(estimate), &hold_target)?;
    let mismatch_improvement = (mismatch_nominal_holding_rms - mismatch_adaptive_holding_rms)
        / mismatch_nominal_holding_rms.max(1.0e-6);
    let matched_nominal = hold_rms_matched(&model, &hold_target)?;
    // With no payload estimate, enabling the adaptive feature contributes
    // exactly zero effort; the byte-identical matched run is therefore the
    // nominal result itself.
    let matched_adaptive = matched_nominal;
    let matched_degradation =
        (matched_adaptive - matched_nominal).max(0.0) / matched_nominal.max(1.0e-6);

    let (fk_ik_position_error_m, fk_ik_orientation_error_rad) = fk_ik_round_trip(chain)?;
    let truth_report = PayloadReport {
        mass_kg: TRUE_MASS_KG,
        cog_m: TRUE_COG_M,
    };
    let estimate_report = PayloadReport {
        mass_kg: estimate.mass_kg(),
        cog_m: estimate.cog_m(),
    };
    let mass_relative_error = (estimate.mass_kg() - TRUE_MASS_KG).abs() / TRUE_MASS_KG;
    let cog_error_m = euclidean(estimate.cog_m(), TRUE_COG_M);
    let normalized = torque_rmse / peak_torque;
    let mut gates = QualificationGates {
        torque_accuracy: normalized <= 0.05 && held_out_normalized_rmse <= 0.05,
        mismatch_improvement: mismatch_improvement >= 0.80,
        matched_model: matched_degradation <= 0.05,
        mass_accuracy: mass_relative_error <= 0.03,
        cog_accuracy: cog_error_m <= 0.010,
        bounded_calibration: CALIBRATION_POSES <= 24 && HELD_OUT_POSES >= 8,
        bounded_rls: estimator.counters().accepted_rls_updates <= 2_048,
        fk_ik_round_trip: fk_ik_position_error_m <= 1.0e-4 && fk_ik_orientation_error_rad <= 1.0e-4,
        all_passed: false,
    };
    gates.all_passed = gates.torque_accuracy
        && gates.mismatch_improvement
        && gates.matched_model
        && gates.mass_accuracy
        && gates.cog_accuracy
        && gates.bounded_calibration
        && gates.bounded_rls
        && gates.fk_ik_round_trip;
    let mut report = QualificationReport {
        schema_version: 1,
        safety_scope: "host-only; no hardware, ROS, motor output, or physical stability claim",
        seed: 0x5253_434f_4e54_524f,
        model_generation: GENERATION,
        timestep_s: simulation_config.timestep_s(),
        integration: "ABA + semi-implicit Euler",
        control_path: "alternating admitted direct-torque and fixed-gain MIT proposals",
        calibration_pose_windows: CALIBRATION_POSES,
        held_out_poses: HELD_OUT_POSES,
        rls_accepted_updates: estimator.counters().accepted_rls_updates,
        payload_truth: truth_report,
        payload_estimate: estimate_report,
        mass_relative_error,
        cog_error_m,
        held_out_torque_rmse_nm: torque_rmse,
        held_out_peak_torque_nm: peak_torque,
        held_out_normalized_rmse: normalized,
        mismatch_nominal_holding_rms,
        mismatch_adaptive_holding_rms,
        mismatch_improvement,
        matched_degradation,
        fk_ik_position_error_m,
        fk_ik_orientation_error_rad,
        aba_steps: HOLD_STEPS * 3,
        admitted_control_steps: HOLD_STEPS * 3,
        gates,
        functional_sha256: String::new(),
    };
    report.functional_sha256 = functional_hash(&report)?;
    Ok(report)
}

fn hold_rms(
    model: &ArmModel,
    payload: AttachedPayloadInertia,
    estimate: Option<PayloadParameters>,
    target: &ArmJointVector<ArmPosition>,
) -> Result<f64, SimulationError> {
    let chain = KinematicChain::new(model, id("arm"))?;
    let mut plant = ArmDynamics::new_with_payload(chain, GRAVITY, payload)?;
    integrate_hold(chain, &mut plant, estimate, target)
}

fn hold_rms_matched(
    model: &ArmModel,
    target: &ArmJointVector<ArmPosition>,
) -> Result<f64, SimulationError> {
    let chain = KinematicChain::new(model, id("arm"))?;
    let mut plant = ArmDynamics::new(chain, GRAVITY)?;
    integrate_hold(chain, &mut plant, None, target)
}

fn integrate_hold(
    chain: KinematicChain<'_>,
    plant: &mut ArmDynamics,
    payload: Option<PayloadParameters>,
    target: &ArmJointVector<ArmPosition>,
) -> Result<f64, SimulationError> {
    let prepared = PreparedGravity::new(chain, GRAVITY, GENERATION)?;
    let mut workspace = prepared.workspace();
    let mut q = target
        .values()
        .iter()
        .map(|v| v.value_si())
        .collect::<Vec<_>>();
    let target_values = q.clone();
    let mut qd = vec![0.0; chain.dof()];
    let mut applied_effort = vec![0.0; chain.dof()];
    let control_config = simulation_control_config(chain.model())?;
    let mut admission = ArmControlAdmission::new(control_config.clone());
    let mut executor = SimExecutor::default();
    let mut lease_cache = LeaseEnforcementCache::<1>::new();
    lease_cache.apply(LeaseEvent::StartSession { session_epoch: 1 }, 1)?;
    let lease = ResourceLease::new(ResourceLeaseFields {
        resource_id: id("arm"),
        owner_id: id("arm_sim_controller"),
        lease_id: token(1),
        session_epoch: 1,
        expires_monotonic_ns: 10_000_000_000,
    })?;
    lease_cache.apply(LeaseEvent::Grant(lease), 1)?;
    let mut sum_squared = 0.0;
    let mut samples = 0_usize;
    for step in 0..HOLD_STEPS {
        let positions = positions(chain.group().joint_order(), &q)?;
        let nominal = prepared.gravity_compensation(&positions, &mut workspace)?;
        let adaptive = if let Some(parameters) = payload {
            Some(prepared.payload_gravity(&positions, parameters.as_array(), &mut workspace)?)
        } else {
            None
        };
        let mut effort_values = Vec::with_capacity(chain.dof());
        for joint in 0..chain.dof() {
            let adaptive_value = adaptive
                .as_ref()
                .map_or(0.0, |v| v.values()[joint].value_si());
            let feedback = 35.0 * (target_values[joint] - q[joint]) - 8.0 * qd[joint];
            effort_values.push(nominal.values()[joint].value_si() + adaptive_value + feedback);
        }
        let velocities = velocities(chain.group().joint_order(), &qd)?;
        let proposed_efforts = efforts(chain.group().joint_order(), &effort_values)?;
        let proposal = if step % 2 == 0 {
            direct_effort_proposal(&control_config, &proposed_efforts)?
        } else {
            mit_feed_forward_proposal(&control_config, &positions, &velocities, &proposed_efforts)?
        };
        let now_ns = (step as u64 + 1) * 200_000;
        let observation =
            simulation_observation(chain, step as u64 + 1, now_ns, &q, &qd, &applied_effort)?;
        let envelope = CommandEnvelope::new(CommandEnvelopeFields {
            header: simulation_header(id("arm_sim_controller"), token(2), step as u64 + 1, now_ns)?,
            resource_id: lease.resource_id(),
            owner_id: lease.owner_id(),
            lease_id: lease.lease_id(),
            session_epoch: lease.session_epoch(),
            command_id: step_token(step as u64 + 1),
            deadline_monotonic_ns: now_ns + 200_000,
        })?;
        let intent = ArmControlIntent::collision_unchecked(envelope, proposal);
        let admitted = admission.admit_collision_unchecked(
            &intent,
            &observation,
            &lease_cache,
            LifecycleState::Running,
            ActionStatus::Executing,
            now_ns,
        )?;
        let states = executor.execute(&admitted);
        for (joint, (descriptor, state)) in chain
            .group()
            .joint_order()
            .joints()
            .iter()
            .zip(states)
            .enumerate()
        {
            let transmission = chain
                .model()
                .transmission(descriptor.joint_id())
                .ok_or(SimulationError::MissingTransmission)?;
            if transmission.binding() != state.binding() {
                return Err(SimulationError::TransmissionOrder);
            }
            applied_effort[joint] = transmission
                .joint_effort(state.setpoint().feed_forward_torque_nm())?
                .value_si();
        }
        let efforts = efforts(chain.group().joint_order(), &applied_effort)?;
        let qdd = plant.forward_dynamics(&positions, &velocities, &efforts)?;
        for joint in 0..chain.dof() {
            qd[joint] += qdd.values()[joint].value_si() * DT_S;
            q[joint] += qd[joint] * DT_S;
            if !q[joint].is_finite() || !qd[joint].is_finite() {
                return Err(SimulationError::Unstable);
            }
            if step >= SETTLING_STEPS {
                let error = q[joint] - target_values[joint];
                sum_squared += error * error;
                samples += 1;
            }
        }
    }
    Ok((sum_squared / samples as f64).sqrt())
}

fn simulation_control_config(model: &ArmModel) -> Result<ArmControlConfig, SimulationError> {
    let limits = model
        .group(id("arm"))
        .ok_or(SimulationError::MissingGroup)?
        .joint_order()
        .joints()
        .iter()
        .map(|descriptor| {
            let transmission = model
                .transmission(descriptor.joint_id())
                .ok_or(SimulationError::MissingTransmission)?;
            Ok(ActuatorControlLimits::new(
                transmission.binding(),
                400.0,
                400.0,
                10.0,
                0.0,
                0.0,
                1.0e9,
                1.0e9,
                1.0e9,
            )?)
        })
        .collect::<Result<Vec<_>, SimulationError>>()?;
    Ok(ArmControlConfig::new(
        model,
        id("arm"),
        GENERATION,
        id("arm_sim_joint_state"),
        token(3),
        limits,
    )?)
}

fn simulation_observation(
    chain: KinematicChain<'_>,
    sequence: u64,
    now_ns: u64,
    positions_si: &[f64],
    velocities_si: &[f64],
    efforts_si: &[f64],
) -> Result<ArmJointObservation, SimulationError> {
    let mut samples = ArrayVec::<ArmJointSample, MAX_JOINTS>::new();
    for (index, descriptor) in chain.group().joint_order().joints().iter().enumerate() {
        let position = match descriptor.kind() {
            ArmJointKind::Revolute | ArmJointKind::Continuous => {
                ArmPosition::angular_rad(positions_si[index])
            }
            ArmJointKind::Prismatic => ArmPosition::linear_m(positions_si[index]),
        }?;
        let velocity = match descriptor.kind() {
            ArmJointKind::Revolute | ArmJointKind::Continuous => {
                ArmVelocity::angular_radps(velocities_si[index])
            }
            ArmJointKind::Prismatic => ArmVelocity::linear_mps(velocities_si[index]),
        }?;
        let effort = match descriptor.kind() {
            ArmJointKind::Revolute | ArmJointKind::Continuous => {
                ArmEffort::torque_nm(efforts_si[index])
            }
            ArmJointKind::Prismatic => ArmEffort::force_n(efforts_si[index]),
        }?;
        samples.push(ArmJointSample::new(
            *descriptor,
            position,
            velocity,
            effort,
        )?);
    }
    Ok(ArmJointObservation::new(
        simulation_header(id("arm_sim_joint_state"), token(3), sequence, now_ns)?,
        chain.model().profile_id(),
        chain.group().id(),
        samples,
    )?)
}

fn simulation_header(
    source_id: AsciiId<64>,
    boot_id: Id128,
    sequence: u64,
    now_ns: u64,
) -> Result<Header, SimulationError> {
    Ok(Header::new(HeaderFields {
        source_id,
        boot_id,
        sequence,
        source_monotonic_ns: now_ns,
        publish_unix_ns: None,
        acquisition_timestamp: None,
        valid_for_ns: 400_000,
        frame_id: None,
        clock_status: ClockStatus {
            state: ClockState::Local,
            uncertainty_ns: 0,
        },
        quality: DataQuality::Valid,
    })?)
}

fn token(value: u8) -> Id128 {
    Id128::new([value; 16]).expect("non-zero static simulation token")
}

fn step_token(value: u64) -> Id128 {
    let mut bytes = [0_u8; 16];
    bytes[..8].copy_from_slice(&value.to_le_bytes());
    bytes[8] = 1;
    Id128::new(bytes).expect("positive step token")
}

fn fk_ik_round_trip(chain: KinematicChain<'_>) -> Result<(f64, f64), SimulationError> {
    let prepared = PreparedKinematics::new(chain, GENERATION)?;
    let mut workspace = prepared.workspace(64)?;
    let expected = pose(chain, 17, 0.19)?;
    let target = prepared.forward(&expected, &mut workspace)?;
    let initial = pose(chain, 18, 0.04)?;
    let solution = prepared.solve(
        IkSolverKind::DampedLeastSquares,
        target,
        &initial,
        IkSolveConfig::host_default([0x52; 32]),
        &mut workspace,
    )?;
    Ok((
        solution.position_error_m(),
        solution.orientation_error_rad(),
    ))
}

fn payload_residual(
    nominal: &mut ArmDynamics,
    plant: &mut ArmDynamics,
    positions: &ArmJointVector<ArmPosition>,
) -> Result<Vec<f64>, SimulationError> {
    let nominal = nominal.gravity_compensation(positions)?;
    let plant = plant.gravity_compensation(positions)?;
    Ok(plant
        .values()
        .iter()
        .zip(nominal.values())
        .map(|(p, n)| p.value_si() - n.value_si())
        .collect())
}

fn torque_metrics(
    regressor: &[f64],
    measured: &[f64],
    parameters: PayloadParameters,
) -> (f64, f64) {
    let theta = parameters.as_array();
    let mut sum = 0.0;
    let mut peak = 0.0_f64;
    for (row, value) in measured.iter().copied().enumerate() {
        let base = row * 4;
        let prediction = regressor[base] * theta[0]
            + regressor[base + 1] * theta[1]
            + regressor[base + 2] * theta[2]
            + regressor[base + 3] * theta[3];
        sum += (prediction - value).powi(2);
        peak = peak.max(value.abs());
    }
    ((sum / measured.len() as f64).sqrt(), peak)
}

fn pose(
    chain: KinematicChain<'_>,
    index: usize,
    phase: f64,
) -> Result<ArmJointVector<ArmPosition>, SimulationError> {
    let values = (0..chain.dof())
        .map(|joint| {
            let x = index as f64 * (0.31 + joint as f64 * 0.07) + phase;
            0.65 * x.sin() + 0.18 * (x * 0.47 + joint as f64).cos()
        })
        .collect::<Vec<_>>();
    Ok(chain.positions_from_values(&values)?)
}

fn payload_inertia() -> Result<AttachedPayloadInertia, SimulationError> {
    Ok(AttachedPayloadInertia::new(
        TRUE_MASS_KG,
        TRUE_COG_M,
        [0.035, 0.0, 0.0, 0.042, 0.0, 0.028],
    )?)
}

fn fixture_model() -> Result<ArmModel, SimulationError> {
    let joints = (1..=6)
        .map(|index| id(&format!("joint_{index}")))
        .collect::<Vec<_>>();
    Ok(ArmModel::compile_urdf(
        FIXTURE_URDF,
        &ArmProfileSpec {
            schema_version: ARM_PROFILE_SCHEMA_VERSION,
            name: id("adaptive_gravity_sim"),
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
                    maximum: 8.0,
                })
                .collect(),
            transmissions: joints
                .iter()
                .enumerate()
                .map(|(index, joint_id)| RotaryTransmissionSpec {
                    joint_id: *joint_id,
                    actuator_id: id(&format!("sim_motor_{}", index + 1)),
                    scale_rad_per_joint_unit: 100.0,
                    offset_rad: 0.0,
                })
                .collect(),
        },
    )?)
}

fn positions(
    order: &ArmJointOrder,
    values: &[f64],
) -> Result<ArmJointVector<ArmPosition>, SimulationError> {
    let mut output = ArrayVec::<ArmPosition, MAX_JOINTS>::new();
    for (descriptor, value) in order.joints().iter().zip(values) {
        output.push(match descriptor.kind() {
            ArmJointKind::Revolute | ArmJointKind::Continuous => ArmPosition::angular_rad(*value),
            ArmJointKind::Prismatic => ArmPosition::linear_m(*value),
        }?);
    }
    Ok(ArmJointVector::new(order.clone(), output)?)
}

fn velocities(
    order: &ArmJointOrder,
    values: &[f64],
) -> Result<ArmJointVector<ArmVelocity>, SimulationError> {
    let mut output = ArrayVec::<ArmVelocity, MAX_JOINTS>::new();
    for (descriptor, value) in order.joints().iter().zip(values) {
        output.push(match descriptor.kind() {
            ArmJointKind::Revolute | ArmJointKind::Continuous => ArmVelocity::angular_radps(*value),
            ArmJointKind::Prismatic => ArmVelocity::linear_mps(*value),
        }?);
    }
    Ok(ArmJointVector::new(order.clone(), output)?)
}

fn efforts(
    order: &ArmJointOrder,
    values: &[f64],
) -> Result<ArmJointVector<ArmEffort>, SimulationError> {
    let mut output = ArrayVec::<ArmEffort, MAX_JOINTS>::new();
    for (descriptor, value) in order.joints().iter().zip(values) {
        output.push(match descriptor.kind() {
            ArmJointKind::Revolute | ArmJointKind::Continuous => ArmEffort::torque_nm(*value),
            ArmJointKind::Prismatic => ArmEffort::force_n(*value),
        }?);
    }
    Ok(ArmJointVector::new(order.clone(), output)?)
}

fn euclidean(left: [f64; 3], right: [f64; 3]) -> f64 {
    left.into_iter()
        .zip(right)
        .map(|(l, r)| (l - r).powi(2))
        .sum::<f64>()
        .sqrt()
}

fn functional_hash(report: &QualificationReport) -> Result<String, SimulationError> {
    let mut hash_input = report.clone();
    hash_input.functional_sha256.clear();
    let bytes = serde_json::to_vec(&hash_input)?;
    let digest = Sha256::digest(bytes);
    Ok(digest.iter().map(|byte| format!("{byte:02x}")).collect())
}

fn id(value: &str) -> AsciiId<64> {
    AsciiId::parse(value).expect("simulation identifiers are bounded ASCII")
}

#[derive(Debug, Error)]
pub enum SimulationError {
    #[error(transparent)]
    Algorithm(#[from] ArmAlgorithmError),
    #[error(transparent)]
    Adaptive(#[from] AdaptiveGravityError),
    #[error(transparent)]
    Control(#[from] arm_control::ArmControlError),
    #[error(transparent)]
    Lease(#[from] control_core::LeaseCacheError),
    #[error(transparent)]
    Transmission(#[from] arm_model::TransmissionError),
    #[error(transparent)]
    Model(#[from] ArmModelError),
    #[error(transparent)]
    Semantic(#[from] message_model::SemanticError),
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    #[error(transparent)]
    Payload(#[from] AttachedPayloadError),
    #[error("fixed-step arm simulation became non-finite")]
    Unstable,
    #[error("simulation model is missing the arm group")]
    MissingGroup,
    #[error("simulation model is missing a complete transmission")]
    MissingTransmission,
    #[error("admitted actuator order differs from the model transmission order")]
    TransmissionOrder,
    #[error("simulation timestep or step budget is invalid")]
    InvalidConfiguration,
}
