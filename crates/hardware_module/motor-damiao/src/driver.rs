use crate::{
    codec::{
        DamiaoCodecError, DamiaoEscId, DamiaoFeedback, DamiaoStatus, MitCommand, RuntimeLimits,
        clear_error_frame, decode_feedback, disable_frame, enable_frame, encode_mit_command,
    },
    register::{Register, RegisterReply, RegisterTransaction, RegisterTransactionError},
};
use hardware_module_core::{
    ClassicCanFrame, DeviceDriver, DeviceError, FrameBatch, Lifecycle, LifecycleState, MonoTime,
    StandardCanId,
};
use thiserror::Error;

const MIT_CONTROL_MODE: u32 = 1;

/// Typed deployment safety policy applied to discovered motor values.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DamiaoSafetyPolicy {
    maximum_limits: RuntimeLimits,
    maximum_motor_timeout_ticks: u32,
    feedback_max_age_ns: u64,
    transition_timeout_ns: u64,
    register_reply_timeout_ns: u64,
}

impl DamiaoSafetyPolicy {
    /// Creates a finite, nonzero policy. Runtime values may be equal to but not
    /// exceed these bounds.
    pub fn new(
        maximum_limits: RuntimeLimits,
        maximum_motor_timeout_ticks: u32,
        feedback_max_age_ns: u64,
        transition_timeout_ns: u64,
        register_reply_timeout_ns: u64,
    ) -> Result<Self, SafetyPolicyError> {
        if maximum_motor_timeout_ticks == 0 {
            return Err(SafetyPolicyError::ZeroMaximumMotorTimeout);
        }
        if feedback_max_age_ns == 0 {
            return Err(SafetyPolicyError::ZeroDeadline {
                field: "feedback_max_age_ns",
            });
        }
        if transition_timeout_ns == 0 {
            return Err(SafetyPolicyError::ZeroDeadline {
                field: "transition_timeout_ns",
            });
        }
        if register_reply_timeout_ns == 0 {
            return Err(SafetyPolicyError::ZeroDeadline {
                field: "register_reply_timeout_ns",
            });
        }
        Ok(Self {
            maximum_limits,
            maximum_motor_timeout_ticks,
            feedback_max_age_ns,
            transition_timeout_ns,
            register_reply_timeout_ns,
        })
    }

    /// Maximum accepted discovered scaling identity.
    #[must_use]
    pub const fn maximum_limits(self) -> RuntimeLimits {
        self.maximum_limits
    }

    /// Maximum accepted motor timeout register value.
    #[must_use]
    pub const fn maximum_motor_timeout_ticks(self) -> u32 {
        self.maximum_motor_timeout_ticks
    }

    /// Maximum age of feedback used for command acceptance.
    #[must_use]
    pub const fn feedback_max_age_ns(self) -> u64 {
        self.feedback_max_age_ns
    }
}

/// Invalid safety-policy construction.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum SafetyPolicyError {
    /// A zero maximum would forbid every safe nonzero motor watchdog.
    #[error("maximum motor timeout ticks must be nonzero")]
    ZeroMaximumMotorTimeout,
    /// A bounded deadline must be positive.
    #[error("Damiao safety policy {field} must be nonzero")]
    ZeroDeadline {
        /// Rejected field.
        field: &'static str,
    },
}

/// Read-only motor identity and scaling proven by the configuration handshake.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerifiedConfiguration {
    esc_id: DamiaoEscId,
    mst_id: StandardCanId,
    timeout_ticks: u32,
    limits: RuntimeLimits,
    generation: u64,
}

impl VerifiedConfiguration {
    /// Expected and observed command ID.
    #[must_use]
    pub const fn esc_id(self) -> DamiaoEscId {
        self.esc_id
    }

    /// Discovered feedback/reply ID.
    #[must_use]
    pub const fn mst_id(self) -> StandardCanId {
        self.mst_id
    }

    /// Verified nonzero motor timeout ticks.
    #[must_use]
    pub const fn timeout_ticks(self) -> u32 {
        self.timeout_ticks
    }

    /// Verified physical scaling identity.
    #[must_use]
    pub const fn limits(self) -> RuntimeLimits {
        self.limits
    }

    /// Driver-local configuration generation.
    #[must_use]
    pub const fn generation(self) -> u64 {
        self.generation
    }

    /// Converts this proof into one bus-wide validation input.
    #[must_use]
    pub const fn bus_device(self) -> DamiaoBusDevice {
        DamiaoBusDevice {
            esc_id: self.esc_id,
            mst_id: self.mst_id,
            configuration_generation: self.generation,
        }
    }
}

/// One verified identity supplied to bus-wide collision validation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DamiaoBusDevice {
    esc_id: DamiaoEscId,
    mst_id: StandardCanId,
    configuration_generation: u64,
}

impl DamiaoBusDevice {
    /// Verified command identifier.
    #[must_use]
    pub const fn esc_id(self) -> DamiaoEscId {
        self.esc_id
    }

    /// Verified reply identifier.
    #[must_use]
    pub const fn mst_id(self) -> StandardCanId {
        self.mst_id
    }

    /// Configuration generation from the owning driver.
    #[must_use]
    pub const fn configuration_generation(self) -> u64 {
        self.configuration_generation
    }
}

/// Bus-wide identity validation failure.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum DamiaoBusSetError {
    /// An empty set cannot authorize any motor to arm.
    #[error("Damiao bus set must not be empty")]
    Empty,
    /// Two motors use the same command ID.
    #[error("duplicate Damiao ESC_ID {id}")]
    DuplicateEscId {
        /// Colliding ID.
        id: u8,
    },
    /// Two motors use the same reply ID.
    #[error("duplicate Damiao MST_ID {id:#x}")]
    DuplicateMstId {
        /// Colliding ID.
        id: u16,
    },
    /// A reply ID is zero or the reserved register request ID.
    #[error("reserved Damiao MST_ID {id:#x}")]
    ReservedMstId {
        /// Rejected ID.
        id: u16,
    },
    /// A reply ID aliases any motor command ID.
    #[error("Damiao MST_ID {mst_id:#x} collides with ESC_ID {esc_id}")]
    MstEscCollision {
        /// Reply ID.
        mst_id: u16,
        /// Command ID.
        esc_id: u8,
    },
    /// The validating driver's current generation is absent from the set.
    #[error("current Damiao configuration is absent from the validated bus set")]
    CurrentDeviceMissing,
}

/// Validates every ESC/MST/reserved-ID collision without allocation.
pub fn validate_bus_set(devices: &[DamiaoBusDevice]) -> Result<(), DamiaoBusSetError> {
    if devices.is_empty() {
        return Err(DamiaoBusSetError::Empty);
    }
    for (index, device) in devices.iter().enumerate() {
        let mst = device.mst_id.as_raw();
        if mst == 0 || mst == crate::REGISTER_REQUEST_ID {
            return Err(DamiaoBusSetError::ReservedMstId { id: mst });
        }
        for prior in &devices[..index] {
            if prior.esc_id == device.esc_id {
                return Err(DamiaoBusSetError::DuplicateEscId {
                    id: device.esc_id.as_raw(),
                });
            }
            if prior.mst_id == device.mst_id {
                return Err(DamiaoBusSetError::DuplicateMstId { id: mst });
            }
        }
        for command_owner in devices {
            if mst == u16::from(command_owner.esc_id.as_raw()) {
                return Err(DamiaoBusSetError::MstEscCollision {
                    mst_id: mst,
                    esc_id: command_owner.esc_id.as_raw(),
                });
            }
        }
    }
    Ok(())
}

/// Protocol, discovery, lifecycle, or transport fault that requires reconfiguration.
#[derive(Debug, Error)]
pub enum DamiaoFault {
    /// Wire/physical-value codec rejected input.
    #[error(transparent)]
    Codec(#[from] DamiaoCodecError),
    /// Register transaction failed correlation or its deadline.
    #[error(transparent)]
    Register(#[from] RegisterTransactionError),
    /// Discovered value violates the arming policy.
    #[error("Damiao configuration rejected: {0}")]
    Configuration(&'static str),
    /// Feedback reported a known or unknown fault status.
    #[error("Damiao feedback reported fault status {0:?}")]
    Status(DamiaoStatus),
    /// A bounded transition/query deadline expired.
    #[error("Damiao {0} deadline expired")]
    Deadline(&'static str),
    /// Local clock or deadline arithmetic was inconsistent.
    #[error("Damiao monotonic deadline arithmetic failed")]
    DeadlineArithmetic,
    /// Frame did not belong to the outstanding query/transition.
    #[error("Damiao feedback is not correlated with the outstanding operation")]
    MisCorrelatedFeedback,
    /// Sample sequence could no longer advance monotonically.
    #[error("Damiao feedback sequence overflowed")]
    SequenceOverflow,
    /// Lifecycle transition violated an internal invariant.
    #[error("Damiao lifecycle invariant failed")]
    LifecycleInvariant,
    /// Bus-wide identities were not validated for this configuration generation.
    #[error("Damiao bus set is not validated for the current configuration")]
    BusNotValidated,
    /// Coordinator reported a transport fault or partial/fatal transmit.
    #[error("Damiao transport fault")]
    Transport,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum ConfigurationStep {
    MstId,
    EscId,
    Timeout,
    ControlMode,
    PositionMaximum,
    VelocityMaximum,
    TorqueMaximum,
}

impl ConfigurationStep {
    const fn register(self) -> Register {
        match self {
            Self::MstId => Register::MstId,
            Self::EscId => Register::EscId,
            Self::Timeout => Register::Timeout,
            Self::ControlMode => Register::ControlMode,
            Self::PositionMaximum => Register::PositionMaximum,
            Self::VelocityMaximum => Register::VelocityMaximum,
            Self::TorqueMaximum => Register::TorqueMaximum,
        }
    }

    const fn next(self) -> Option<Self> {
        match self {
            Self::MstId => Some(Self::EscId),
            Self::EscId => Some(Self::Timeout),
            Self::Timeout => Some(Self::ControlMode),
            Self::ControlMode => Some(Self::PositionMaximum),
            Self::PositionMaximum => Some(Self::VelocityMaximum),
            Self::VelocityMaximum => Some(Self::TorqueMaximum),
            Self::TorqueMaximum => None,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct ConfigurationValues {
    mst_id: Option<StandardCanId>,
    timeout_ticks: Option<u32>,
    position_maximum: Option<f32>,
    velocity_maximum: Option<f32>,
    torque_maximum: Option<f32>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum FeedbackPurpose {
    Enable,
    Command,
    Disable { invalidate_configuration: bool },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PendingFeedback {
    purpose: FeedbackPurpose,
    deadline: MonoTime,
}

/// Bounded, allocation-free Damiao MIT-mode device state machine.
#[derive(Debug)]
pub struct DamiaoDriver {
    esc_id: DamiaoEscId,
    policy: DamiaoSafetyPolicy,
    lifecycle: Lifecycle,
    transaction: RegisterTransaction,
    configuration_step: Option<ConfigurationStep>,
    values: ConfigurationValues,
    configuration: Option<VerifiedConfiguration>,
    bus_validated_generation: Option<u64>,
    pending_feedback: Option<PendingFeedback>,
    last_feedback_at: Option<MonoTime>,
    sample_sequence: u64,
    next_generation: u64,
}

impl DamiaoDriver {
    /// Creates an unconfigured, disarmed driver. No frame is emitted.
    #[must_use]
    pub const fn new(esc_id: DamiaoEscId, policy: DamiaoSafetyPolicy) -> Self {
        Self {
            esc_id,
            policy,
            lifecycle: Lifecycle::new(),
            transaction: RegisterTransaction::new(),
            configuration_step: None,
            values: ConfigurationValues {
                mst_id: None,
                timeout_ticks: None,
                position_maximum: None,
                velocity_maximum: None,
                torque_maximum: None,
            },
            configuration: None,
            bus_validated_generation: None,
            pending_feedback: None,
            last_feedback_at: None,
            sample_sequence: 0,
            next_generation: 0,
        }
    }

    /// Current lifecycle state without selecting a frame-batch capacity.
    #[must_use]
    pub const fn state(&self) -> LifecycleState {
        self.lifecycle.state()
    }

    /// Current read-only verified configuration, if its generation remains valid.
    #[must_use]
    pub const fn verified_configuration(&self) -> Option<VerifiedConfiguration> {
        self.configuration
    }

    /// Applies bus-wide collision validation and authorizes only this exact generation.
    pub fn authorize_validated_bus_set(
        &mut self,
        devices: &[DamiaoBusDevice],
    ) -> Result<(), DamiaoBusSetError> {
        validate_bus_set(devices)?;
        let configuration = self
            .configuration
            .ok_or(DamiaoBusSetError::CurrentDeviceMissing)?;
        let current = configuration.bus_device();
        if !devices.contains(&current) {
            return Err(DamiaoBusSetError::CurrentDeviceMissing);
        }
        self.bus_validated_generation = Some(configuration.generation);
        Ok(())
    }

    /// Builds an explicit disarmed clear-error request without changing state.
    ///
    /// It never enables, retries, or creates an outstanding command window.
    pub fn clear_error_request(&self) -> Result<ClassicCanFrame, DeviceError<DamiaoFault>> {
        if !matches!(
            self.state(),
            LifecycleState::Disabled | LifecycleState::Faulted
        ) {
            return Err(DeviceError::InvalidLifecycle {
                required: LifecycleState::Disabled,
                actual: self.state(),
            });
        }
        Ok(clear_error_frame(self.esc_id))
    }

    fn invalidate_configuration(&mut self) {
        self.configuration = None;
        self.bus_validated_generation = None;
        self.last_feedback_at = None;
    }

    fn fault_error(&mut self, fault: DamiaoFault) -> DeviceError<DamiaoFault> {
        self.transaction.clear();
        self.configuration_step = None;
        self.pending_feedback = None;
        self.invalidate_configuration();
        if self.state() != LifecycleState::Faulted {
            let _ = self.lifecycle.transition(LifecycleState::Faulted);
        }
        DeviceError::Protocol(fault)
    }

    fn deadline(now: MonoTime, duration_ns: u64) -> Result<MonoTime, DamiaoFault> {
        now.as_nanos()
            .checked_add(duration_ns)
            .map(MonoTime::from_nanos)
            .ok_or(DamiaoFault::DeadlineArithmetic)
    }

    fn append<const N: usize>(
        output: &mut FrameBatch<ClassicCanFrame, N>,
        frame: ClassicCanFrame,
    ) -> Result<(), DeviceError<DamiaoFault>> {
        output
            .push(frame)
            .map_err(|_| DeviceError::OutputBatchFull { capacity: N })
    }

    fn begin_configuration_read<const N: usize>(
        &mut self,
        step: ConfigurationStep,
        expected_reply_id: Option<StandardCanId>,
        now: MonoTime,
        output: &mut FrameBatch<ClassicCanFrame, N>,
    ) -> Result<(), DeviceError<DamiaoFault>> {
        if output.len() == N {
            return Err(DeviceError::OutputBatchFull { capacity: N });
        }
        let frame = self
            .transaction
            .begin_read(
                self.esc_id,
                step.register(),
                expected_reply_id,
                now,
                self.policy.register_reply_timeout_ns,
            )
            .map_err(DamiaoFault::from)
            .map_err(DeviceError::Protocol)?;
        if let Err(error) = Self::append(output, frame) {
            self.transaction.clear();
            return Err(error);
        }
        self.configuration_step = Some(step);
        Ok(())
    }

    fn apply_configuration_reply(
        &mut self,
        step: ConfigurationStep,
        reply: RegisterReply,
    ) -> Result<(), DamiaoFault> {
        match step {
            ConfigurationStep::MstId => {
                let raw = reply.value_u32();
                let raw = u16::try_from(raw)
                    .map_err(|_| DamiaoFault::Configuration("MST_ID exceeds standard CAN"))?;
                let mst_id = StandardCanId::new(raw)
                    .map_err(|_| DamiaoFault::Configuration("MST_ID exceeds standard CAN"))?;
                if raw == 0 || raw == crate::REGISTER_REQUEST_ID {
                    return Err(DamiaoFault::Configuration("MST_ID is reserved"));
                }
                if reply.source_id != mst_id {
                    return Err(DamiaoFault::Configuration(
                        "MST_ID value does not match reply arbitration ID",
                    ));
                }
                self.values.mst_id = Some(mst_id);
            }
            ConfigurationStep::EscId => {
                if reply.value_u32() != u32::from(self.esc_id.as_raw()) {
                    return Err(DamiaoFault::Configuration(
                        "observed ESC_ID does not match expected motor ID",
                    ));
                }
            }
            ConfigurationStep::Timeout => {
                let timeout = reply.value_u32();
                if timeout == 0 {
                    return Err(DamiaoFault::Configuration("TIMEOUT is zero"));
                }
                if timeout > self.policy.maximum_motor_timeout_ticks {
                    return Err(DamiaoFault::Configuration("TIMEOUT exceeds safety policy"));
                }
                self.values.timeout_ticks = Some(timeout);
            }
            ConfigurationStep::ControlMode => {
                if reply.value_u32() != MIT_CONTROL_MODE {
                    return Err(DamiaoFault::Configuration("CTRL_MODE is not current MIT"));
                }
            }
            ConfigurationStep::PositionMaximum => {
                self.values.position_maximum = Some(self.accept_limit(
                    "PMAX",
                    reply.value_f32(),
                    self.policy.maximum_limits.position(),
                )?);
            }
            ConfigurationStep::VelocityMaximum => {
                self.values.velocity_maximum = Some(self.accept_limit(
                    "VMAX",
                    reply.value_f32(),
                    self.policy.maximum_limits.velocity(),
                )?);
            }
            ConfigurationStep::TorqueMaximum => {
                self.values.torque_maximum = Some(self.accept_limit(
                    "TMAX",
                    reply.value_f32(),
                    self.policy.maximum_limits.torque(),
                )?);
            }
        }
        Ok(())
    }

    fn accept_limit(
        &self,
        name: &'static str,
        observed: f32,
        maximum: f32,
    ) -> Result<f32, DamiaoFault> {
        if !observed.is_finite() || observed <= 0.0 {
            return Err(DamiaoFault::Configuration(match name {
                "PMAX" => "PMAX is not finite and positive",
                "VMAX" => "VMAX is not finite and positive",
                _ => "TMAX is not finite and positive",
            }));
        }
        if observed > maximum {
            return Err(DamiaoFault::Configuration(match name {
                "PMAX" => "PMAX exceeds safety policy",
                "VMAX" => "VMAX exceeds safety policy",
                _ => "TMAX exceeds safety policy",
            }));
        }
        Ok(observed)
    }

    fn complete_configuration(&mut self) -> Result<(), DamiaoFault> {
        let mst_id = self
            .values
            .mst_id
            .ok_or(DamiaoFault::Configuration("MST_ID was not read"))?;
        let timeout_ticks = self
            .values
            .timeout_ticks
            .ok_or(DamiaoFault::Configuration("TIMEOUT was not read"))?;
        let limits = RuntimeLimits::new(
            self.values
                .position_maximum
                .ok_or(DamiaoFault::Configuration("PMAX was not read"))?,
            self.values
                .velocity_maximum
                .ok_or(DamiaoFault::Configuration("VMAX was not read"))?,
            self.values
                .torque_maximum
                .ok_or(DamiaoFault::Configuration("TMAX was not read"))?,
        )?;
        self.next_generation =
            self.next_generation
                .checked_add(1)
                .ok_or(DamiaoFault::Configuration(
                    "configuration generation overflowed",
                ))?;
        self.configuration = Some(VerifiedConfiguration {
            esc_id: self.esc_id,
            mst_id,
            timeout_ticks,
            limits,
            generation: self.next_generation,
        });
        self.configuration_step = None;
        self.lifecycle
            .transition(LifecycleState::Disabled)
            .map_err(|_| DamiaoFault::LifecycleInvariant)
    }

    fn enter_safe_stop<const N: usize>(
        &mut self,
        now: MonoTime,
        output: &mut FrameBatch<ClassicCanFrame, N>,
    ) -> Result<(), DeviceError<DamiaoFault>> {
        let deadline = match Self::deadline(now, self.policy.transition_timeout_ns) {
            Ok(deadline) => deadline,
            Err(fault) => return Err(self.fault_error(fault)),
        };
        if let Err(error) = Self::append(output, disable_frame(self.esc_id)) {
            let _ = self.fault_error(DamiaoFault::Transport);
            return Err(error);
        }
        self.lifecycle
            .transition(LifecycleState::SafeStop)
            .map_err(|_| DeviceError::Protocol(DamiaoFault::LifecycleInvariant))?;
        self.pending_feedback = Some(PendingFeedback {
            purpose: FeedbackPurpose::Disable {
                invalidate_configuration: true,
            },
            deadline,
        });
        Ok(())
    }

    fn feedback_age(&self, now: MonoTime, observed: MonoTime) -> Result<u64, DamiaoFault> {
        now.checked_elapsed_since(observed)
            .ok_or(DamiaoFault::DeadlineArithmetic)
    }

    fn process_feedback<const N: usize>(
        &mut self,
        frame: ClassicCanFrame,
        received_at: MonoTime,
        now: MonoTime,
        output: &mut FrameBatch<ClassicCanFrame, N>,
    ) -> Result<Option<DamiaoFeedback>, DeviceError<DamiaoFault>> {
        let Some(pending) = self.pending_feedback else {
            if matches!(
                self.state(),
                LifecycleState::Disabled | LifecycleState::Faulted
            ) {
                return Ok(None);
            }
            return Err(self.fault_error(DamiaoFault::MisCorrelatedFeedback));
        };
        let Some(configuration) = self.configuration else {
            return Err(self.fault_error(DamiaoFault::Configuration(
                "feedback arrived without verified scaling",
            )));
        };
        let age = match self.feedback_age(now, received_at) {
            Ok(age) => age,
            Err(fault) => return Err(self.fault_error(fault)),
        };
        if received_at > pending.deadline || age > self.policy.feedback_max_age_ns {
            if self.state() == LifecycleState::Enabled {
                self.enter_safe_stop(now, output)?;
                return Err(DeviceError::StaleFeedback {
                    age_ns: age,
                    limit_ns: self.policy.feedback_max_age_ns,
                });
            }
            return Err(self.fault_error(DamiaoFault::Deadline("feedback")));
        }
        let sequence = match self.sample_sequence.checked_add(1) {
            Some(sequence) => sequence,
            None => return Err(self.fault_error(DamiaoFault::SequenceOverflow)),
        };
        let feedback = match decode_feedback(
            frame,
            configuration.mst_id,
            self.esc_id,
            configuration.limits,
            received_at,
            sequence,
        ) {
            Ok(feedback) => feedback,
            Err(error) => return Err(self.fault_error(DamiaoFault::Codec(error))),
        };
        if feedback.status.is_fault() {
            return Err(self.fault_error(DamiaoFault::Status(feedback.status)));
        }
        self.sample_sequence = sequence;
        self.last_feedback_at = Some(received_at);

        match (self.state(), pending.purpose, feedback.status) {
            (LifecycleState::Enabling, FeedbackPurpose::Enable, DamiaoStatus::Enabled) => {
                self.lifecycle
                    .transition(LifecycleState::Enabled)
                    .map_err(|_| DeviceError::Protocol(DamiaoFault::LifecycleInvariant))?;
                self.pending_feedback = None;
            }
            (LifecycleState::Enabling, FeedbackPurpose::Enable, DamiaoStatus::Disabled) => {}
            (LifecycleState::Enabled, FeedbackPurpose::Command, DamiaoStatus::Enabled) => {
                self.pending_feedback = None;
            }
            (LifecycleState::Enabled, FeedbackPurpose::Command, DamiaoStatus::Disabled) => {
                return Err(self.fault_error(DamiaoFault::MisCorrelatedFeedback));
            }
            (
                LifecycleState::Disabling | LifecycleState::SafeStop,
                FeedbackPurpose::Disable {
                    invalidate_configuration,
                },
                DamiaoStatus::Disabled,
            ) => {
                if self.state() == LifecycleState::SafeStop {
                    self.lifecycle
                        .transition(LifecycleState::Disabling)
                        .map_err(|_| DeviceError::Protocol(DamiaoFault::LifecycleInvariant))?;
                }
                self.lifecycle
                    .transition(LifecycleState::Disabled)
                    .map_err(|_| DeviceError::Protocol(DamiaoFault::LifecycleInvariant))?;
                self.pending_feedback = None;
                if invalidate_configuration {
                    self.invalidate_configuration();
                }
            }
            (
                LifecycleState::Disabling | LifecycleState::SafeStop,
                FeedbackPurpose::Disable { .. },
                DamiaoStatus::Enabled,
            ) => {}
            _ => return Err(self.fault_error(DamiaoFault::MisCorrelatedFeedback)),
        }
        Ok(Some(feedback))
    }
}

impl<const N: usize> DeviceDriver<ClassicCanFrame, N> for DamiaoDriver {
    type Command = MitCommand;
    type Sample = DamiaoFeedback;
    type Fault = DamiaoFault;

    fn lifecycle(&self) -> LifecycleState {
        self.state()
    }

    fn start_configure(
        &mut self,
        now: MonoTime,
        output: &mut FrameBatch<ClassicCanFrame, N>,
    ) -> Result<(), DeviceError<Self::Fault>> {
        if !matches!(
            self.state(),
            LifecycleState::Unconfigured | LifecycleState::Disabled | LifecycleState::Faulted
        ) {
            return Err(DeviceError::InvalidLifecycle {
                required: LifecycleState::Disabled,
                actual: self.state(),
            });
        }
        self.transaction.clear();
        self.configuration_step = None;
        self.values = ConfigurationValues::default();
        self.pending_feedback = None;
        self.invalidate_configuration();
        self.begin_configuration_read(ConfigurationStep::MstId, None, now, output)?;
        if self
            .lifecycle
            .transition(LifecycleState::Configuring)
            .is_err()
        {
            self.transaction.clear();
            return Err(self.fault_error(DamiaoFault::LifecycleInvariant));
        }
        Ok(())
    }

    fn start_enable(
        &mut self,
        now: MonoTime,
        output: &mut FrameBatch<ClassicCanFrame, N>,
    ) -> Result<(), DeviceError<Self::Fault>> {
        if self.state() != LifecycleState::Disabled {
            return Err(DeviceError::InvalidLifecycle {
                required: LifecycleState::Disabled,
                actual: self.state(),
            });
        }
        let configuration =
            self.configuration
                .ok_or(DeviceError::Protocol(DamiaoFault::Configuration(
                    "enable requires verified configuration",
                )))?;
        if self.bus_validated_generation != Some(configuration.generation) {
            return Err(DeviceError::Protocol(DamiaoFault::BusNotValidated));
        }
        let deadline = match Self::deadline(now, self.policy.transition_timeout_ns) {
            Ok(deadline) => deadline,
            Err(fault) => return Err(self.fault_error(fault)),
        };
        Self::append(output, enable_frame(self.esc_id))?;
        self.lifecycle
            .transition(LifecycleState::Enabling)
            .map_err(|_| DeviceError::Protocol(DamiaoFault::LifecycleInvariant))?;
        self.pending_feedback = Some(PendingFeedback {
            purpose: FeedbackPurpose::Enable,
            deadline,
        });
        Ok(())
    }

    fn start_disable(
        &mut self,
        now: MonoTime,
        output: &mut FrameBatch<ClassicCanFrame, N>,
    ) -> Result<(), DeviceError<Self::Fault>> {
        let invalidate_configuration = match self.state() {
            LifecycleState::Enabled => false,
            LifecycleState::Enabling | LifecycleState::SafeStop => true,
            actual => {
                return Err(DeviceError::InvalidLifecycle {
                    required: LifecycleState::Enabled,
                    actual,
                });
            }
        };
        let deadline = match Self::deadline(now, self.policy.transition_timeout_ns) {
            Ok(deadline) => deadline,
            Err(fault) => return Err(self.fault_error(fault)),
        };
        if let Err(error) = Self::append(output, disable_frame(self.esc_id)) {
            let _ = self.fault_error(DamiaoFault::Transport);
            return Err(error);
        }
        if self.state() == LifecycleState::Enabling {
            self.lifecycle
                .transition(LifecycleState::SafeStop)
                .map_err(|_| DeviceError::Protocol(DamiaoFault::LifecycleInvariant))?;
        }
        self.lifecycle
            .transition(LifecycleState::Disabling)
            .map_err(|_| DeviceError::Protocol(DamiaoFault::LifecycleInvariant))?;
        self.pending_feedback = Some(PendingFeedback {
            purpose: FeedbackPurpose::Disable {
                invalidate_configuration,
            },
            deadline,
        });
        Ok(())
    }

    fn handle_input(
        &mut self,
        frame: ClassicCanFrame,
        received_at: MonoTime,
        now: MonoTime,
        output: &mut FrameBatch<ClassicCanFrame, N>,
    ) -> Result<Option<Self::Sample>, DeviceError<Self::Fault>> {
        if self.state() == LifecycleState::Configuring {
            let step = self
                .configuration_step
                .ok_or(DeviceError::Protocol(DamiaoFault::LifecycleInvariant))?;
            let reply = match self.transaction.accept_reply(frame, received_at) {
                Ok(reply) => reply,
                Err(error) => return Err(self.fault_error(DamiaoFault::Register(error))),
            };
            if let Err(fault) = self.apply_configuration_reply(step, reply) {
                return Err(self.fault_error(fault));
            }
            if let Some(next) = step.next() {
                let expected_reply_id = self.values.mst_id;
                if let Err(error) =
                    self.begin_configuration_read(next, expected_reply_id, now, output)
                {
                    if matches!(error, DeviceError::OutputBatchFull { .. }) {
                        let _ = self.fault_error(DamiaoFault::LifecycleInvariant);
                    }
                    return Err(error);
                }
            } else if let Err(fault) = self.complete_configuration() {
                return Err(self.fault_error(fault));
            }
            return Ok(None);
        }
        self.process_feedback(frame, received_at, now, output)
    }

    fn poll(
        &mut self,
        now: MonoTime,
        output: &mut FrameBatch<ClassicCanFrame, N>,
    ) -> Result<(), DeviceError<Self::Fault>> {
        match self.state() {
            LifecycleState::Configuring => {
                if self
                    .transaction
                    .deadline()
                    .is_some_and(|deadline| now > deadline)
                {
                    return Err(self.fault_error(DamiaoFault::Deadline("register reply")));
                }
            }
            LifecycleState::Enabling | LifecycleState::Disabling => {
                if self
                    .pending_feedback
                    .is_some_and(|pending| now > pending.deadline)
                {
                    return Err(self.fault_error(DamiaoFault::Deadline("transition")));
                }
            }
            LifecycleState::SafeStop => {
                let Some(pending) = self.pending_feedback else {
                    return Err(self.fault_error(DamiaoFault::LifecycleInvariant));
                };
                if now > pending.deadline {
                    return Err(self.fault_error(DamiaoFault::Deadline("safe-stop disable")));
                }
                self.lifecycle
                    .transition(LifecycleState::Disabling)
                    .map_err(|_| DeviceError::Protocol(DamiaoFault::LifecycleInvariant))?;
            }
            LifecycleState::Enabled => {
                let pending_expired = self
                    .pending_feedback
                    .is_some_and(|pending| now > pending.deadline);
                let feedback_stale = match self.last_feedback_at {
                    Some(last) => self
                        .feedback_age(now, last)
                        .map(|age| age > self.policy.feedback_max_age_ns)
                        .unwrap_or(true),
                    None => true,
                };
                if pending_expired || feedback_stale {
                    self.enter_safe_stop(now, output)?;
                }
            }
            LifecycleState::Unconfigured | LifecycleState::Disabled | LifecycleState::Faulted => {}
        }
        Ok(())
    }

    fn encode_command(
        &mut self,
        command: &Self::Command,
        now: MonoTime,
        output: &mut FrameBatch<ClassicCanFrame, N>,
    ) -> Result<(), DeviceError<Self::Fault>> {
        if self.state() != LifecycleState::Enabled {
            return Err(DeviceError::InvalidLifecycle {
                required: LifecycleState::Enabled,
                actual: self.state(),
            });
        }
        if self.pending_feedback.is_some() {
            return Err(DeviceError::InvalidCommand(
                "previous Damiao feedback is still pending",
            ));
        }
        let configuration =
            self.configuration
                .ok_or(DeviceError::Protocol(DamiaoFault::Configuration(
                    "command requires verified configuration",
                )))?;
        if self.bus_validated_generation != Some(configuration.generation) {
            return Err(DeviceError::Protocol(DamiaoFault::BusNotValidated));
        }
        let last = self
            .last_feedback_at
            .ok_or(DeviceError::Protocol(DamiaoFault::MisCorrelatedFeedback))?;
        let age = self
            .feedback_age(now, last)
            .map_err(DeviceError::Protocol)?;
        if age > self.policy.feedback_max_age_ns {
            self.enter_safe_stop(now, output)?;
            return Err(DeviceError::StaleFeedback {
                age_ns: age,
                limit_ns: self.policy.feedback_max_age_ns,
            });
        }
        let frame = encode_mit_command(self.esc_id, configuration.limits, *command)
            .map_err(DamiaoFault::Codec)
            .map_err(DeviceError::Protocol)?;
        let deadline = match Self::deadline(now, self.policy.feedback_max_age_ns) {
            Ok(deadline) => deadline,
            Err(fault) => return Err(self.fault_error(fault)),
        };
        Self::append(output, frame)?;
        self.pending_feedback = Some(PendingFeedback {
            purpose: FeedbackPurpose::Command,
            deadline,
        });
        Ok(())
    }

    fn report_transport_fault(&mut self, _now: MonoTime) -> Result<(), DeviceError<Self::Fault>> {
        Err(self.fault_error(DamiaoFault::Transport))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hardware_module_core::DeviceDriver;

    const BATCH: usize = 2;

    fn esc() -> DamiaoEscId {
        DamiaoEscId::new(1).unwrap()
    }

    fn mst() -> StandardCanId {
        StandardCanId::new(0x11).unwrap()
    }

    fn policy() -> DamiaoSafetyPolicy {
        DamiaoSafetyPolicy::new(
            RuntimeLimits::new(20.0, 60.0, 20.0).unwrap(),
            1_000,
            100,
            100,
            100,
        )
        .unwrap()
    }

    fn reply(register: Register, value: [u8; 4]) -> ClassicCanFrame {
        ClassicCanFrame::new(
            mst(),
            &[
                1,
                0,
                0x33,
                register.address(),
                value[0],
                value[1],
                value[2],
                value[3],
            ],
        )
        .unwrap()
    }

    fn feedback(status: u8) -> ClassicCanFrame {
        ClassicCanFrame::new(
            mst(),
            &[(status << 4) | 1, 0x7f, 0xff, 0x7f, 0xf7, 0xff, 20, 21],
        )
        .unwrap()
    }

    fn configure(driver: &mut DamiaoDriver) -> VerifiedConfiguration {
        let mut output = FrameBatch::<ClassicCanFrame, BATCH>::new();
        driver
            .start_configure(MonoTime::from_nanos(0), &mut output)
            .unwrap();
        assert_eq!(driver.state(), LifecycleState::Configuring);
        assert_eq!(output.as_slice()[0].data()[2], 0x33);

        let replies = [
            (Register::MstId, u32::from(mst().as_raw()).to_le_bytes()),
            (Register::EscId, 1_u32.to_le_bytes()),
            (Register::Timeout, 20_u32.to_le_bytes()),
            (Register::ControlMode, MIT_CONTROL_MODE.to_le_bytes()),
            (Register::PositionMaximum, 12.5_f32.to_le_bytes()),
            (Register::VelocityMaximum, 30.0_f32.to_le_bytes()),
            (Register::TorqueMaximum, 10.0_f32.to_le_bytes()),
        ];
        for (index, (register, value)) in replies.into_iter().enumerate() {
            output.clear();
            let now = MonoTime::from_nanos((index + 1) as u64 * 10);
            driver
                .handle_input(reply(register, value), now, now, &mut output)
                .unwrap();
            if index < 6 {
                assert_eq!(output.len(), 1);
                assert_eq!(output.as_slice()[0].data()[2], 0x33);
            }
        }
        assert_eq!(driver.state(), LifecycleState::Disabled);
        let configuration = driver.verified_configuration().unwrap();
        driver
            .authorize_validated_bus_set(&[configuration.bus_device()])
            .unwrap();
        configuration
    }

    fn enable(driver: &mut DamiaoDriver) {
        let mut output = FrameBatch::<ClassicCanFrame, BATCH>::new();
        driver
            .start_enable(MonoTime::from_nanos(80), &mut output)
            .unwrap();
        assert_eq!(driver.state(), LifecycleState::Enabling);
        assert_eq!(output.as_slice()[0].data(), crate::ENABLE_DATA);
        output.clear();
        let sample = driver
            .handle_input(
                feedback(1),
                MonoTime::from_nanos(90),
                MonoTime::from_nanos(90),
                &mut output,
            )
            .unwrap()
            .unwrap();
        assert_eq!(sample.status, DamiaoStatus::Enabled);
        assert_eq!(driver.state(), LifecycleState::Enabled);
    }

    #[test]
    fn lifecycle_configuration_is_read_only_and_required_before_enable() {
        let mut driver = DamiaoDriver::new(esc(), policy());
        let mut output = FrameBatch::<ClassicCanFrame, BATCH>::new();
        assert!(
            driver
                .start_enable(MonoTime::from_nanos(0), &mut output)
                .is_err()
        );
        let configuration = configure(&mut driver);
        assert_eq!(configuration.mst_id(), mst());
        assert_eq!(configuration.timeout_ticks(), 20);
        assert_eq!(
            configuration.limits(),
            RuntimeLimits::new(12.5, 30.0, 10.0).unwrap()
        );
    }

    #[test]
    fn lifecycle_enable_command_and_feedback_are_correlated() {
        let mut driver = DamiaoDriver::new(esc(), policy());
        configure(&mut driver);
        enable(&mut driver);
        let command = MitCommand {
            position: 0.0,
            velocity: 0.0,
            kp: 0.0,
            kd: 0.0,
            feed_forward_torque: 0.0,
        };
        let mut output = FrameBatch::<ClassicCanFrame, BATCH>::new();
        driver
            .encode_command(&command, MonoTime::from_nanos(100), &mut output)
            .unwrap();
        assert_eq!(output.len(), 1);
        assert!(
            driver
                .encode_command(&command, MonoTime::from_nanos(101), &mut output)
                .is_err()
        );
        output.clear();
        driver
            .handle_input(
                feedback(1),
                MonoTime::from_nanos(110),
                MonoTime::from_nanos(110),
                &mut output,
            )
            .unwrap();
        assert_eq!(driver.state(), LifecycleState::Enabled);
    }

    #[test]
    fn lifecycle_stale_feedback_uses_bounded_safe_stop_then_invalidates_configuration() {
        let mut driver = DamiaoDriver::new(esc(), policy());
        configure(&mut driver);
        enable(&mut driver);
        let mut output = FrameBatch::<ClassicCanFrame, BATCH>::new();
        driver.poll(MonoTime::from_nanos(191), &mut output).unwrap();
        assert_eq!(driver.state(), LifecycleState::SafeStop);
        assert_eq!(output.as_slice()[0].data(), crate::DISABLE_DATA);
        output.clear();
        driver.poll(MonoTime::from_nanos(192), &mut output).unwrap();
        assert_eq!(driver.state(), LifecycleState::Disabling);
        driver
            .handle_input(
                feedback(0),
                MonoTime::from_nanos(193),
                MonoTime::from_nanos(193),
                &mut output,
            )
            .unwrap();
        assert_eq!(driver.state(), LifecycleState::Disabled);
        assert!(driver.verified_configuration().is_none());
        assert!(
            driver
                .start_enable(MonoTime::from_nanos(194), &mut output)
                .is_err()
        );
    }

    #[test]
    fn lifecycle_transition_timeout_and_fault_status_fault_closed() {
        let mut driver = DamiaoDriver::new(esc(), policy());
        configure(&mut driver);
        let mut output = FrameBatch::<ClassicCanFrame, BATCH>::new();
        driver
            .start_enable(MonoTime::from_nanos(80), &mut output)
            .unwrap();
        output.clear();
        assert!(driver.poll(MonoTime::from_nanos(181), &mut output).is_err());
        assert_eq!(driver.state(), LifecycleState::Faulted);
        assert!(driver.verified_configuration().is_none());

        driver
            .start_configure(MonoTime::from_nanos(200), &mut output)
            .unwrap();
        assert_eq!(driver.state(), LifecycleState::Configuring);

        let mut other = DamiaoDriver::new(esc(), policy());
        configure(&mut other);
        enable(&mut other);
        assert!(
            other
                .handle_input(
                    feedback(0x0f),
                    MonoTime::from_nanos(100),
                    MonoTime::from_nanos(100),
                    &mut output,
                )
                .is_err()
        );
        assert_eq!(other.state(), LifecycleState::Faulted);
    }

    #[test]
    fn lifecycle_wrong_register_reply_faults_and_requires_fresh_configuration() {
        let mut driver = DamiaoDriver::new(esc(), policy());
        let mut output = FrameBatch::<ClassicCanFrame, BATCH>::new();
        driver
            .start_configure(MonoTime::from_nanos(0), &mut output)
            .unwrap();
        output.clear();
        assert!(
            driver
                .handle_input(
                    reply(Register::EscId, 1_u32.to_le_bytes()),
                    MonoTime::from_nanos(1),
                    MonoTime::from_nanos(1),
                    &mut output,
                )
                .is_err()
        );
        assert_eq!(driver.state(), LifecycleState::Faulted);
        assert!(driver.verified_configuration().is_none());
    }

    #[test]
    fn lifecycle_transport_fault_and_clear_error_never_auto_enable() {
        let mut driver = DamiaoDriver::new(esc(), policy());
        configure(&mut driver);
        let before = driver.state();
        assert_eq!(
            driver.clear_error_request().unwrap().data(),
            crate::CLEAR_ERROR_DATA
        );
        assert_eq!(driver.state(), before);
        assert!(
            <DamiaoDriver as DeviceDriver<ClassicCanFrame, BATCH>>::report_transport_fault(
                &mut driver,
                MonoTime::from_nanos(0)
            )
            .is_err()
        );
        assert_eq!(driver.state(), LifecycleState::Faulted);
        assert_eq!(
            driver.clear_error_request().unwrap().data(),
            crate::CLEAR_ERROR_DATA
        );
        assert_eq!(driver.state(), LifecycleState::Faulted);
    }

    #[test]
    fn lifecycle_full_safe_stop_output_faults_instead_of_remaining_enabled() {
        let mut driver = DamiaoDriver::new(esc(), policy());
        configure(&mut driver);
        enable(&mut driver);
        let mut output = FrameBatch::<ClassicCanFrame, 0>::new();
        assert!(
            <DamiaoDriver as DeviceDriver<ClassicCanFrame, 0>>::poll(
                &mut driver,
                MonoTime::from_nanos(191),
                &mut output,
            )
            .is_err()
        );
        assert_eq!(driver.state(), LifecycleState::Faulted);
    }

    #[test]
    fn lifecycle_normal_disable_confirmation_retains_verified_configuration() {
        let mut driver = DamiaoDriver::new(esc(), policy());
        let configuration = configure(&mut driver);
        enable(&mut driver);
        let mut output = FrameBatch::<ClassicCanFrame, BATCH>::new();
        driver
            .start_disable(MonoTime::from_nanos(100), &mut output)
            .unwrap();
        output.clear();
        driver
            .handle_input(
                feedback(0),
                MonoTime::from_nanos(110),
                MonoTime::from_nanos(110),
                &mut output,
            )
            .unwrap();
        assert_eq!(driver.state(), LifecycleState::Disabled);
        assert_eq!(driver.verified_configuration(), Some(configuration));
    }

    #[test]
    fn lifecycle_configuration_policy_rejects_mode_timeout_and_limits() {
        let invalid_values = [
            (Register::Timeout, 0_u32.to_le_bytes()),
            (Register::ControlMode, 2_u32.to_le_bytes()),
            (Register::PositionMaximum, f32::NAN.to_le_bytes()),
            (Register::PositionMaximum, 21.0_f32.to_le_bytes()),
        ];
        for (target_register, bad_value) in invalid_values {
            let mut driver = DamiaoDriver::new(esc(), policy());
            let mut output = FrameBatch::<ClassicCanFrame, BATCH>::new();
            driver
                .start_configure(MonoTime::from_nanos(0), &mut output)
                .unwrap();
            let prefix = [
                (Register::MstId, u32::from(mst().as_raw()).to_le_bytes()),
                (Register::EscId, 1_u32.to_le_bytes()),
                (Register::Timeout, 20_u32.to_le_bytes()),
                (Register::ControlMode, 1_u32.to_le_bytes()),
            ];
            for (index, (register, value)) in prefix.into_iter().enumerate() {
                if register == target_register {
                    output.clear();
                    assert!(
                        driver
                            .handle_input(
                                reply(register, bad_value),
                                MonoTime::from_nanos((index + 1) as u64),
                                MonoTime::from_nanos((index + 1) as u64),
                                &mut output,
                            )
                            .is_err()
                    );
                    break;
                }
                output.clear();
                driver
                    .handle_input(
                        reply(register, value),
                        MonoTime::from_nanos((index + 1) as u64),
                        MonoTime::from_nanos((index + 1) as u64),
                        &mut output,
                    )
                    .unwrap();
            }
            if target_register == Register::PositionMaximum {
                output.clear();
                assert!(
                    driver
                        .handle_input(
                            reply(Register::PositionMaximum, bad_value),
                            MonoTime::from_nanos(5),
                            MonoTime::from_nanos(5),
                            &mut output,
                        )
                        .is_err()
                );
            }
            assert_eq!(driver.state(), LifecycleState::Faulted);
        }
    }

    #[test]
    fn bus_set_validation_rejects_all_collision_classes() {
        let a = DamiaoBusDevice {
            esc_id: DamiaoEscId::new(1).unwrap(),
            mst_id: StandardCanId::new(0x11).unwrap(),
            configuration_generation: 1,
        };
        let duplicate_esc = DamiaoBusDevice {
            esc_id: a.esc_id,
            mst_id: StandardCanId::new(0x12).unwrap(),
            configuration_generation: 1,
        };
        assert!(matches!(
            validate_bus_set(&[a, duplicate_esc]),
            Err(DamiaoBusSetError::DuplicateEscId { .. })
        ));
        let duplicate_mst = DamiaoBusDevice {
            esc_id: DamiaoEscId::new(2).unwrap(),
            mst_id: a.mst_id,
            configuration_generation: 1,
        };
        assert!(matches!(
            validate_bus_set(&[a, duplicate_mst]),
            Err(DamiaoBusSetError::DuplicateMstId { .. })
        ));
        let reserved = DamiaoBusDevice {
            esc_id: DamiaoEscId::new(2).unwrap(),
            mst_id: StandardCanId::new(crate::REGISTER_REQUEST_ID).unwrap(),
            configuration_generation: 1,
        };
        assert!(matches!(
            validate_bus_set(&[reserved]),
            Err(DamiaoBusSetError::ReservedMstId { .. })
        ));
        let collision = DamiaoBusDevice {
            esc_id: DamiaoEscId::new(2).unwrap(),
            mst_id: a.esc_id.standard_id(),
            configuration_generation: 1,
        };
        assert!(matches!(
            validate_bus_set(&[a, collision]),
            Err(DamiaoBusSetError::MstEscCollision { .. })
        ));
    }
}
