//! Motor-disabled authority, final-limiter and deterministic feedback harness.

use arrayvec::ArrayVec;
use chassis_mecanum::Mecanum4;
use chassis_model::{
    BodyCommandLimits, BodyConstraintEvidence, BodyPose2, BodyTwist, Feasibility, KinematicsError,
    constrain_body_command,
};
use control_core::{LeaseCacheError, LeaseEnforcementCache, LeaseEvent};
use message_model::{
    AsciiId, ChassisSetpointIntent, ClockState, ClockStatus, Covariance6, DataQuality, FrameId,
    Header, HeaderFields, Id128, NavigationAuthority, NavigationAuthorityState, OdometryState,
    OdometryStateFields, Pose3, ResourceLease, ResourceLeaseFields, SemanticError,
    StampedTransform, StampedTransformFields, TransformBatch, Twist2, Twist3, UnitQuaternion,
    Vector3,
};
use navigation_ipc::{NavigationGuardError, NavigationSourceGuard, NavigationTfBatch};
use thiserror::Error;

/// Hard maximum reception-relative proposal lifetime.
pub const MAX_PROPOSAL_TTL_NS: u64 = 100_000_000;
/// Fake sink history is fixed and never grows at runtime.
pub const FAKE_SINK_CAPACITY: usize = 64;
/// This binary has no build mode that permits physical output.
pub const MOTOR_OUTPUT_COMPILED_OUT: bool = true;
/// Stable synthetic authority session for one process boot.
pub const SYNTHETIC_SESSION_EPOCH: u64 = 1;

/// Why the motor-disabled sink received a zero command.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StopReason {
    /// No current proposal remained usable.
    ProposalExpired,
    /// Authority became absent or expired.
    AuthorityUnavailable,
    /// An IPC peer disappeared.
    PeerUnavailable,
}

/// One bounded fake-output observation; it can never reach a device.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FakeSinkSample {
    /// Local enforcement timestamp.
    pub monotonic_ns: u64,
    /// Correlated command when this is executable proposal evidence.
    pub command_id: Option<Id128>,
    /// Final body command after the sole Rust limiter.
    pub body: BodyTwist,
    /// Projected FL/FR/RL/RR wheel velocities.
    pub wheel_radps: [f64; 4],
    /// Constraint evidence from the body limiter.
    pub body_constraints: BodyConstraintEvidence,
    /// Uniform mecanum wheel saturation evidence.
    pub wheel_feasibility: Feasibility,
    /// Typed zero-output decision, if any.
    pub stop_reason: Option<StopReason>,
}

/// Fixed-capacity overwrite-oldest fake sink.
#[derive(Clone, Debug, Default)]
pub struct FakeSink {
    samples: ArrayVec<FakeSinkSample, FAKE_SINK_CAPACITY>,
    overwritten: u64,
}

impl FakeSink {
    /// Returns retained samples in oldest-to-newest order.
    #[must_use]
    pub fn samples(&self) -> &[FakeSinkSample] {
        &self.samples
    }

    /// Returns the newest retained decision.
    #[must_use]
    pub fn latest(&self) -> Option<FakeSinkSample> {
        self.samples.last().copied()
    }

    /// Returns how many old observations were overwritten.
    #[must_use]
    pub const fn overwritten(&self) -> u64 {
        self.overwritten
    }

    fn push(&mut self, sample: FakeSinkSample) {
        if self.samples.is_full() {
            let _ = self.samples.remove(0);
            self.overwritten = self.overwritten.saturating_add(1);
        }
        self.samples.push(sample);
    }
}

/// Successful proposal admission and final projection evidence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HarnessOutput {
    /// Correlated command identity.
    pub command_id: Id128,
    /// Final constrained body command.
    pub body: BodyTwist,
    /// Final projected wheel speeds in stable mecanum order.
    pub wheel_radps: [f64; 4],
    /// Body constraint evidence.
    pub body_constraints: BodyConstraintEvidence,
    /// Wheel projection feasibility.
    pub wheel_feasibility: Feasibility,
    /// Updated deterministic simulated pose.
    pub pose: BodyPose2,
}

/// Typed fail-closed proposal rejection.
#[derive(Debug, Error)]
pub enum HarnessError {
    /// Proposal source/boot/sequence/freshness was unusable.
    #[error(transparent)]
    Source(#[from] NavigationGuardError),
    /// Semantic authority materialization failed.
    #[error(transparent)]
    Semantic(#[from] SemanticError),
    /// Local lease cache rejected the exact envelope.
    #[error(transparent)]
    Lease(#[from] LeaseCacheError),
    /// Final limiting, projection or integration failed.
    #[error(transparent)]
    Kinematics(#[from] KinematicsError),
    /// No current chassis authority exists.
    #[error("no active chassis lease")]
    NoActiveLease,
    /// Fixed navigation TF ownership was violated.
    #[error(transparent)]
    NavigationIpc(#[from] navigation_ipc::NavigationIpcError),
}

/// Pure motor-disabled control owner used by the E2E process and tests.
pub struct MotorDisabledHarness {
    lease_cache: LeaseEnforcementCache<1>,
    chassis_resource: AsciiId<64>,
    authority_source: AsciiId<64>,
    authority_boot: Id128,
    authority_sequence: u64,
    proposal_guard: NavigationSourceGuard<4>,
    model: Mecanum4,
    limits: BodyCommandLimits,
    previous_body: BodyTwist,
    pose: BodyPose2,
    active_command_id: Option<Id128>,
    last_proposal_deadline_ns: Option<u64>,
    sink: FakeSink,
}

impl MotorDisabledHarness {
    /// Constructs the exact synthetic authority and reviewed mecanum profile.
    pub fn new(
        now_monotonic_ns: u64,
        lease_duration_ns: u64,
        authority_boot: Id128,
        proposal_source: AsciiId<64>,
    ) -> Result<Self, HarnessError> {
        Self::new_at_pose(
            now_monotonic_ns,
            lease_duration_ns,
            authority_boot,
            proposal_source,
            BodyPose2::new(0.0, 0.0, 0.0)?,
        )
    }

    /// Constructs the harness at an explicit deterministic map pose.
    pub fn new_at_pose(
        now_monotonic_ns: u64,
        lease_duration_ns: u64,
        authority_boot: Id128,
        proposal_source: AsciiId<64>,
        initial_pose: BodyPose2,
    ) -> Result<Self, HarnessError> {
        let chassis_resource = AsciiId::parse("chassis").expect("static chassis ID");
        let authority_source = AsciiId::parse("rs-navigation-e2e").expect("static source ID");
        let owner = AsciiId::parse("navigation-mission").expect("static owner ID");
        let lease_id = Id128::new([0x4c; 16]).expect("static lease identity");
        let expires_monotonic_ns = now_monotonic_ns.checked_add(lease_duration_ns).ok_or(
            SemanticError::InvalidTimeOrder {
                field: "synthetic_lease",
            },
        )?;
        let lease = ResourceLease::new(ResourceLeaseFields {
            resource_id: chassis_resource,
            owner_id: owner,
            lease_id,
            session_epoch: SYNTHETIC_SESSION_EPOCH,
            expires_monotonic_ns,
        })?;
        let mut lease_cache = LeaseEnforcementCache::new();
        let _ = lease_cache.apply(
            LeaseEvent::StartSession {
                session_epoch: SYNTHETIC_SESSION_EPOCH,
            },
            now_monotonic_ns,
        )?;
        let _ = lease_cache.apply(LeaseEvent::Grant(lease), now_monotonic_ns)?;
        Ok(Self {
            lease_cache,
            chassis_resource,
            authority_source,
            authority_boot,
            authority_sequence: 0,
            proposal_guard: NavigationSourceGuard::new(proposal_source),
            model: Mecanum4::new(0.1, 0.3, 0.25, 20.0)?,
            limits: BodyCommandLimits::new(1.0, 1.5, 2.0, 3.0)?,
            previous_body: Twist2::new(0.0, 0.0, 0.0)?,
            pose: initial_pose,
            active_command_id: None,
            last_proposal_deadline_ns: None,
            sink: FakeSink::default(),
        })
    }

    /// Publishes an assertion about existing authority; it does not grant it.
    pub fn authority_state(
        &mut self,
        now_monotonic_ns: u64,
        active_command_id: Option<Id128>,
    ) -> Result<NavigationAuthorityState, HarnessError> {
        self.authority_sequence = self.authority_sequence.saturating_add(1);
        let authority = self
            .lease_cache
            .active_lease(self.chassis_resource, now_monotonic_ns)
            .map_or(
                NavigationAuthority::Unavailable(
                    message_model::NavigationAuthorityUnavailable::LeaseExpired,
                ),
                |lease| NavigationAuthority::Available {
                    lease,
                    active_command_id,
                },
            );
        Ok(NavigationAuthorityState {
            header: local_header(
                self.authority_source,
                self.authority_boot,
                self.authority_sequence,
                now_monotonic_ns,
                MAX_PROPOSAL_TTL_NS,
                None,
            )?,
            authority,
        })
    }

    /// Validates authority twice, applies the sole final limiter and fake sink.
    pub fn accept_proposal(
        &mut self,
        proposal: ChassisSetpointIntent,
        received_monotonic_ns: u64,
        dt_s: f64,
    ) -> Result<HarnessOutput, HarnessError> {
        self.proposal_guard
            .accept(proposal.intent().header(), received_monotonic_ns)?;
        let lease = self
            .lease_cache
            .active_lease(self.chassis_resource, received_monotonic_ns)
            .ok_or(HarnessError::NoActiveLease)?;
        let command = proposal.materialize(lease, received_monotonic_ns, MAX_PROPOSAL_TTL_NS)?;
        let _ = self
            .lease_cache
            .authorize(&command.envelope, received_monotonic_ns)?;
        let constrained =
            constrain_body_command(command.target, self.previous_body, self.limits, dt_s)?;
        let wheel_solution = self.model.inverse(constrained.command())?;
        let wheel_feasibility = wheel_solution.feasibility();
        let wheel_radps = wheel_solution.value().into_values();
        let achieved = self.model.forward(*wheel_solution.value())?;
        self.pose = self.pose.integrate_body(achieved, dt_s)?;
        self.previous_body = achieved;
        self.active_command_id = Some(command.envelope.command_id());
        self.last_proposal_deadline_ns = Some(command.envelope.deadline_monotonic_ns());
        let sample = FakeSinkSample {
            monotonic_ns: received_monotonic_ns,
            command_id: self.active_command_id,
            body: achieved,
            wheel_radps,
            body_constraints: constrained.evidence(),
            wheel_feasibility,
            stop_reason: None,
        };
        self.sink.push(sample);
        Ok(HarnessOutput {
            command_id: command.envelope.command_id(),
            body: achieved,
            wheel_radps,
            body_constraints: constrained.evidence(),
            wheel_feasibility,
            pose: self.pose,
        })
    }

    /// Expires authority/proposals and emits an explicit zero fake-sink decision.
    pub fn tick(&mut self, now_monotonic_ns: u64) -> Result<Option<StopReason>, HarnessError> {
        let lease_was_active = self
            .lease_cache
            .active_lease(self.chassis_resource, now_monotonic_ns)
            .is_some();
        let _ = self.lease_cache.apply(LeaseEvent::Tick, now_monotonic_ns)?;
        let authority_active = self
            .lease_cache
            .active_lease(self.chassis_resource, now_monotonic_ns)
            .is_some();
        let reason = if lease_was_active && !authority_active {
            Some(StopReason::AuthorityUnavailable)
        } else if self
            .last_proposal_deadline_ns
            .is_some_and(|deadline| now_monotonic_ns >= deadline)
        {
            Some(StopReason::ProposalExpired)
        } else {
            None
        };
        if let Some(reason) = reason {
            self.emit_stop(now_monotonic_ns, reason)?;
        }
        Ok(reason)
    }

    /// Retires the proposal publisher boot and emits a zero decision immediately.
    pub fn peer_lost(&mut self, now_monotonic_ns: u64) -> Result<(), HarnessError> {
        self.proposal_guard.disconnect();
        self.emit_stop(now_monotonic_ns, StopReason::PeerUnavailable)
    }

    /// Produces exact semantic odometry plus owned map/odom/base dynamic TF.
    pub fn feedback(
        &self,
        sequence: u64,
        now_monotonic_ns: u64,
    ) -> Result<(OdometryState, NavigationTfBatch), HarnessError> {
        let [x, y, yaw] = self.pose.components();
        let half = yaw * 0.5;
        let orientation = UnitQuaternion::new(0.0, 0.0, half.sin(), half.cos())?;
        let odom = AsciiId::parse("odom").expect("static frame");
        let base = AsciiId::parse("base_link").expect("static frame");
        let map = AsciiId::parse("map").expect("static frame");
        let header = local_header(
            self.authority_source,
            self.authority_boot,
            sequence,
            now_monotonic_ns,
            MAX_PROPOSAL_TTL_NS,
            Some(odom),
        )?;
        let pose = Pose3 {
            position_m: Vector3::new(x, y, 0.0)?,
            orientation,
        };
        let components = self.previous_body.components();
        let twist = Twist3 {
            linear_mps: Vector3::new(components[0], components[1], 0.0)?,
            angular_radps: Vector3::new(0.0, 0.0, components[2])?,
        };
        let covariance = Covariance6::new([0.0; 36])?;
        let odometry = OdometryState::new(OdometryStateFields {
            header,
            parent_frame_id: odom,
            child_frame_id: base,
            pose,
            twist,
            pose_covariance: covariance,
            twist_covariance: covariance,
        })?;
        let mut transforms = ArrayVec::new();
        transforms.push(StampedTransform::new(StampedTransformFields {
            header: local_header(
                self.authority_source,
                self.authority_boot,
                sequence,
                now_monotonic_ns,
                MAX_PROPOSAL_TTL_NS,
                Some(map),
            )?,
            parent_frame_id: map,
            child_frame_id: odom,
            translation_m: Vector3::new(0.0, 0.0, 0.0)?,
            rotation: UnitQuaternion::new(0.0, 0.0, 0.0, 1.0)?,
            is_static: false,
        })?);
        transforms.push(StampedTransform::new(StampedTransformFields {
            header,
            parent_frame_id: odom,
            child_frame_id: base,
            translation_m: pose.position_m,
            rotation: pose.orientation,
            is_static: false,
        })?);
        let tf = NavigationTfBatch::new(TransformBatch::new(transforms)?)?;
        Ok((odometry, tf))
    }

    /// Returns the bounded fake sink.
    #[must_use]
    pub const fn sink(&self) -> &FakeSink {
        &self.sink
    }

    /// Returns the current deterministic simulated pose.
    #[must_use]
    pub const fn pose(&self) -> BodyPose2 {
        self.pose
    }

    /// Returns the exact current synthetic lease when still active.
    #[must_use]
    pub fn active_lease(&self, now_monotonic_ns: u64) -> Option<ResourceLease> {
        self.lease_cache
            .active_lease(self.chassis_resource, now_monotonic_ns)
    }

    fn emit_stop(&mut self, now_monotonic_ns: u64, reason: StopReason) -> Result<(), HarnessError> {
        let zero = Twist2::new(0.0, 0.0, 0.0)?;
        self.previous_body = zero;
        self.active_command_id = None;
        self.last_proposal_deadline_ns = None;
        self.sink.push(FakeSinkSample {
            monotonic_ns: now_monotonic_ns,
            command_id: None,
            body: zero,
            wheel_radps: [0.0; 4],
            body_constraints: BodyConstraintEvidence::default(),
            wheel_feasibility: Feasibility::Feasible,
            stop_reason: Some(reason),
        });
        Ok(())
    }
}

fn local_header(
    source_id: AsciiId<64>,
    boot_id: Id128,
    sequence: u64,
    now_monotonic_ns: u64,
    valid_for_ns: u64,
    frame_id: Option<FrameId>,
) -> Result<Header, SemanticError> {
    Header::new(HeaderFields {
        source_id,
        boot_id,
        sequence,
        source_monotonic_ns: now_monotonic_ns,
        publish_unix_ns: None,
        acquisition_timestamp: None,
        valid_for_ns,
        frame_id,
        clock_status: ClockStatus {
            state: ClockState::Local,
            uncertainty_ns: 0,
        },
        quality: DataQuality::Valid,
    })
}
