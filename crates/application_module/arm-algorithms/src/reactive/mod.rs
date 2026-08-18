//! Reactive servo, filtering and collision-independent RMP policies.

use arrayvec::ArrayVec;
use message_model::{
    ArmAcceleration, ArmJointId, ArmJointKind, ArmJointVector, ArmPosition, ArmVelocity, MAX_JOINTS,
};
use nalgebra::{DMatrix, DVector};

use crate::{
    ArmAlgorithmError,
    kinematics::{ArmPose, IkWorkspace, KinematicChain, PreparedKinematics},
};

/// Finite base-frame Cartesian twist.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmTwist {
    linear_mps: [f64; 3],
    angular_radps: [f64; 3],
}

impl ArmTwist {
    /// Validates a finite twist.
    pub fn new(linear_mps: [f64; 3], angular_radps: [f64; 3]) -> Result<Self, ArmAlgorithmError> {
        if linear_mps
            .iter()
            .chain(&angular_radps)
            .any(|value| !value.is_finite())
        {
            return Err(ArmAlgorithmError::NonFinite {
                field: "reactive.twist",
            });
        }
        Ok(Self {
            linear_mps,
            angular_radps,
        })
    }

    /// Returns base-frame linear velocity.
    #[must_use]
    pub const fn linear_mps(self) -> [f64; 3] {
        self.linear_mps
    }

    /// Returns base-frame angular velocity.
    #[must_use]
    pub const fn angular_radps(self) -> [f64; 3] {
        self.angular_radps
    }
}

/// Finite servo gain, damping and output scaling.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ServoConfig {
    position_gain: f64,
    orientation_gain: f64,
    damping: f64,
    maximum_velocity_scale: f64,
}

impl ServoConfig {
    /// Validates positive gains/damping and scale in `(0,1]`.
    pub fn new(
        position_gain: f64,
        orientation_gain: f64,
        damping: f64,
        maximum_velocity_scale: f64,
    ) -> Result<Self, ArmAlgorithmError> {
        if !position_gain.is_finite()
            || position_gain <= 0.0
            || !orientation_gain.is_finite()
            || orientation_gain <= 0.0
            || !damping.is_finite()
            || damping <= 0.0
            || !maximum_velocity_scale.is_finite()
            || !(0.0..=1.0).contains(&maximum_velocity_scale)
            || maximum_velocity_scale == 0.0
        {
            return Err(ArmAlgorithmError::NonFinite {
                field: "reactive.servo_config",
            });
        }
        Ok(Self {
            position_gain,
            orientation_gain,
            damping,
            maximum_velocity_scale,
        })
    }

    /// Returns conservative host defaults.
    #[must_use]
    pub const fn host_default() -> Self {
        Self {
            position_gain: 1.0,
            orientation_gain: 1.0,
            damping: 1.0e-3,
            maximum_velocity_scale: 0.5,
        }
    }
}

/// In-place joint-velocity filter used after validated servo projection.
pub trait JointVelocityFilter {
    /// Filters one exact-DOF finite vector.
    fn filter(&mut self, values_si: &mut [f64]) -> Result<(), ArmAlgorithmError>;
}

/// Identity filter.
#[derive(Clone, Copy, Debug, Default)]
pub struct NoFilter;

impl JointVelocityFilter for NoFilter {
    fn filter(&mut self, _values_si: &mut [f64]) -> Result<(), ArmAlgorithmError> {
        Ok(())
    }
}

/// Per-joint exponential moving-average filter.
#[derive(Clone, Debug, PartialEq)]
pub struct EmaFilter {
    alpha: f64,
    state: Vec<f64>,
    initialized: bool,
}

impl EmaFilter {
    /// Creates an exact-DOF EMA with `alpha` in `(0,1]`.
    pub fn new(dof: usize, alpha: f64) -> Result<Self, ArmAlgorithmError> {
        if dof == 0 || dof > MAX_JOINTS {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "reactive.filter_dof",
            });
        }
        if !alpha.is_finite() || alpha <= 0.0 || alpha > 1.0 {
            return Err(ArmAlgorithmError::NonFinite {
                field: "reactive.ema_alpha",
            });
        }
        Ok(Self {
            alpha,
            state: vec![0.0; dof],
            initialized: false,
        })
    }
}

impl JointVelocityFilter for EmaFilter {
    fn filter(&mut self, values_si: &mut [f64]) -> Result<(), ArmAlgorithmError> {
        if values_si.len() != self.state.len() {
            return Err(ArmAlgorithmError::DimensionMismatch {
                field: "reactive.ema",
                expected: self.state.len(),
                actual: values_si.len(),
            });
        }
        if values_si.iter().any(|value| !value.is_finite()) {
            return Err(ArmAlgorithmError::NonFinite {
                field: "reactive.ema",
            });
        }
        if !self.initialized {
            self.state.copy_from_slice(values_si);
            self.initialized = true;
        } else {
            for (state, value) in self.state.iter_mut().zip(values_si.iter().copied()) {
                *state = (value - *state).mul_add(self.alpha, *state);
            }
        }
        values_si.copy_from_slice(&self.state);
        Ok(())
    }
}

/// Per-joint second-order Butterworth low-pass biquad.
#[derive(Clone, Debug, PartialEq)]
pub struct ButterworthFilter {
    b: [f64; 3],
    a: [f64; 2],
    x1: Vec<f64>,
    x2: Vec<f64>,
    y1: Vec<f64>,
    y2: Vec<f64>,
}

impl ButterworthFilter {
    /// Designs a bilinear-transform low-pass filter below Nyquist.
    pub fn new(
        dof: usize,
        sample_frequency_hz: f64,
        cutoff_frequency_hz: f64,
    ) -> Result<Self, ArmAlgorithmError> {
        if dof == 0 || dof > MAX_JOINTS {
            return Err(ArmAlgorithmError::InvalidBudget {
                field: "reactive.filter_dof",
            });
        }
        if !sample_frequency_hz.is_finite()
            || sample_frequency_hz <= 0.0
            || !cutoff_frequency_hz.is_finite()
            || cutoff_frequency_hz <= 0.0
            || cutoff_frequency_hz >= sample_frequency_hz * 0.5
        {
            return Err(ArmAlgorithmError::NonFinite {
                field: "reactive.butterworth_frequency",
            });
        }
        let k = (std::f64::consts::PI * cutoff_frequency_hz / sample_frequency_hz).tan();
        let normalization = 1.0 / (1.0 + std::f64::consts::SQRT_2 * k + k * k);
        let b0 = k * k * normalization;
        let b = [b0, 2.0 * b0, b0];
        let a = [
            2.0 * (k * k - 1.0) * normalization,
            (1.0 - std::f64::consts::SQRT_2 * k + k * k) * normalization,
        ];
        Ok(Self {
            b,
            a,
            x1: vec![0.0; dof],
            x2: vec![0.0; dof],
            y1: vec![0.0; dof],
            y2: vec![0.0; dof],
        })
    }
}

impl JointVelocityFilter for ButterworthFilter {
    fn filter(&mut self, values_si: &mut [f64]) -> Result<(), ArmAlgorithmError> {
        if values_si.len() != self.x1.len() {
            return Err(ArmAlgorithmError::DimensionMismatch {
                field: "reactive.butterworth",
                expected: self.x1.len(),
                actual: values_si.len(),
            });
        }
        for (index, value) in values_si.iter_mut().enumerate() {
            if !value.is_finite() {
                return Err(ArmAlgorithmError::NonFinite {
                    field: "reactive.butterworth",
                });
            }
            let output =
                self.b[0] * *value + self.b[1] * self.x1[index] + self.b[2] * self.x2[index]
                    - self.a[0] * self.y1[index]
                    - self.a[1] * self.y2[index];
            self.x2[index] = self.x1[index];
            self.x1[index] = *value;
            self.y2[index] = self.y1[index];
            self.y1[index] = output;
            *value = output;
        }
        Ok(())
    }
}

impl KinematicChain<'_> {
    /// Projects a base-frame Cartesian twist through a damped pseudoinverse.
    pub fn twist_servo(
        self,
        positions: &ArmJointVector<ArmPosition>,
        twist: ArmTwist,
        config: ServoConfig,
        filter: &mut impl JointVelocityFilter,
    ) -> Result<ArmJointVector<ArmVelocity>, ArmAlgorithmError> {
        let jacobian = self.jacobian(positions)?.as_matrix();
        let target =
            DVector::from_iterator(6, twist.linear_mps.into_iter().chain(twist.angular_radps));
        let regularized =
            &jacobian * jacobian.transpose() + DMatrix::identity(6, 6) * config.damping.powi(2);
        let solved = regularized
            .lu()
            .solve(&target)
            .ok_or(ArmAlgorithmError::Singular {
                rank: 0,
                required_rank: 6.min(self.dof()),
            })?;
        let mut velocities = (jacobian.transpose() * solved).as_slice().to_vec();
        clamp_velocities(self, &mut velocities, config.maximum_velocity_scale)?;
        filter.filter(&mut velocities)?;
        velocity_vector(self, &velocities)
    }

    /// Tracks one Cartesian pose by proportional twist feedback.
    pub fn pose_tracking_servo(
        self,
        positions: &ArmJointVector<ArmPosition>,
        target: ArmPose,
        config: ServoConfig,
        filter: &mut impl JointVelocityFilter,
    ) -> Result<ArmJointVector<ArmVelocity>, ArmAlgorithmError> {
        let current = self.forward(positions)?.tool_pose().to_isometry();
        let target = target.to_isometry();
        let translation = target.translation.vector - current.translation.vector;
        let orientation = (target.rotation * current.rotation.inverse()).scaled_axis();
        self.twist_servo(
            positions,
            ArmTwist::new(
                [
                    translation.x * config.position_gain,
                    translation.y * config.position_gain,
                    translation.z * config.position_gain,
                ],
                [
                    orientation.x * config.orientation_gain,
                    orientation.y * config.orientation_gain,
                    orientation.z * config.orientation_gain,
                ],
            )?,
            config,
            filter,
        )
    }

    /// Validates, scales and filters a direct stable-order joint jog.
    pub fn joint_jog(
        self,
        requested: &ArmJointVector<ArmVelocity>,
        maximum_velocity_scale: f64,
        filter: &mut impl JointVelocityFilter,
    ) -> Result<ArmJointVector<ArmVelocity>, ArmAlgorithmError> {
        if requested.order() != self.group().joint_order() {
            return Err(ArmAlgorithmError::JointOrderMismatch {
                group: self.group().id(),
            });
        }
        if !maximum_velocity_scale.is_finite()
            || maximum_velocity_scale <= 0.0
            || maximum_velocity_scale > 1.0
        {
            return Err(ArmAlgorithmError::NonFinite {
                field: "reactive.velocity_scale",
            });
        }
        let mut values = requested
            .values()
            .iter()
            .zip(self.group().joint_order().joints())
            .map(|(value, descriptor)| {
                if value.kind() != descriptor.kind().coordinate_kind() {
                    Err(ArmAlgorithmError::UnitMismatch {
                        joint: descriptor.joint_id(),
                    })
                } else {
                    Ok(value.value_si())
                }
            })
            .collect::<Result<Vec<_>, _>>()?;
        clamp_velocities(self, &mut values, maximum_velocity_scale)?;
        filter.filter(&mut values)?;
        velocity_vector(self, &values)
    }
}

/// Collision-independent RMP acceleration policy weights.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RmpConfig {
    reach_gain: f64,
    damping_gain: f64,
    joint_limit_gain: f64,
    joint_limit_margin: f64,
    singularity_gain: f64,
    singularity_threshold: f64,
}

impl RmpConfig {
    /// Validates finite nonnegative gains and positive margin.
    pub fn new(
        reach_gain: f64,
        damping_gain: f64,
        joint_limit_gain: f64,
        joint_limit_margin: f64,
    ) -> Result<Self, ArmAlgorithmError> {
        if [reach_gain, damping_gain, joint_limit_gain]
            .iter()
            .any(|value| !value.is_finite() || *value < 0.0)
            || !joint_limit_margin.is_finite()
            || joint_limit_margin <= 0.0
        {
            return Err(ArmAlgorithmError::NonFinite {
                field: "reactive.rmp_config",
            });
        }
        Ok(Self {
            reach_gain,
            damping_gain,
            joint_limit_gain,
            joint_limit_margin,
            singularity_gain: 0.0,
            singularity_threshold: 1.0,
        })
    }

    /// Adds bounded velocity damping as the prepared Jacobian approaches a
    /// singular configuration.
    pub fn with_singularity(
        mut self,
        singularity_gain: f64,
        singularity_threshold: f64,
    ) -> Result<Self, ArmAlgorithmError> {
        if !singularity_gain.is_finite()
            || singularity_gain < 0.0
            || !singularity_threshold.is_finite()
            || singularity_threshold <= 0.0
        {
            return Err(ArmAlgorithmError::NonFinite {
                field: "reactive.rmp_singularity",
            });
        }
        self.singularity_gain = singularity_gain;
        self.singularity_threshold = singularity_threshold;
        Ok(self)
    }

    /// Returns balanced host defaults.
    #[must_use]
    pub const fn host_default() -> Self {
        Self {
            reach_gain: 4.0,
            damping_gain: 1.5,
            joint_limit_gain: 0.2,
            joint_limit_margin: 0.2,
            singularity_gain: 2.0,
            singularity_threshold: 0.05,
        }
    }
}

/// Immutable prepared RMP policy bound to one arm model generation.
pub struct PreparedRmp<'prepared, 'model> {
    kinematics: &'prepared PreparedKinematics<'model>,
    config: RmpConfig,
}

impl<'prepared, 'model> PreparedRmp<'prepared, 'model> {
    /// Binds the RMP policy to one prepared kinematics identity.
    #[must_use]
    pub const fn new(kinematics: &'prepared PreparedKinematics<'model>, config: RmpConfig) -> Self {
        Self { kinematics, config }
    }

    /// Allocates all Jacobian and kinematics scratch before warm calls.
    pub fn workspace(&self) -> Result<RmpWorkspace, ArmAlgorithmError> {
        RmpWorkspace::new(self.kinematics)
    }

    /// Evaluates bounded reach, damping, joint-limit and singularity RMP
    /// components without allocating after workspace preparation.
    pub fn accelerate(
        &self,
        positions: &ArmJointVector<ArmPosition>,
        velocities: &ArmJointVector<ArmVelocity>,
        target: &ArmJointVector<ArmPosition>,
        workspace: &mut RmpWorkspace,
    ) -> Result<PreparedRmpOutput, ArmAlgorithmError> {
        workspace.require(self.kinematics)?;
        self.kinematics
            .copy_positions(positions, &mut workspace.current)?;
        self.kinematics
            .copy_positions(target, &mut workspace.target)?;
        self.kinematics.jacobian_into(
            positions,
            &mut workspace.kinematics_workspace,
            &mut workspace.jacobian,
        )?;
        if velocities.order() != self.kinematics.order() {
            return Err(ArmAlgorithmError::JointOrderMismatch {
                group: self.kinematics.group_id(),
            });
        }

        let (smallest_singular_value, singularity_rotations) =
            smallest_singular_value(&workspace.jacobian, self.kinematics.dof());
        let singularity_scale = if smallest_singular_value < self.config.singularity_threshold {
            1.0 - smallest_singular_value / self.config.singularity_threshold
        } else {
            0.0
        };
        let chain = self.kinematics.chain();
        let mut reach = ArrayVec::<ArmAcceleration, MAX_JOINTS>::new();
        let mut damping = ArrayVec::<ArmAcceleration, MAX_JOINTS>::new();
        let mut joint_limit = ArrayVec::<ArmAcceleration, MAX_JOINTS>::new();
        let mut singularity = ArrayVec::<ArmAcceleration, MAX_JOINTS>::new();
        let mut total = ArrayVec::<ArmAcceleration, MAX_JOINTS>::new();

        for (index, descriptor) in self.kinematics.order().joints().iter().enumerate() {
            let velocity = velocities.values()[index];
            if velocity.kind() != descriptor.kind().coordinate_kind() {
                return Err(ArmAlgorithmError::UnitMismatch {
                    joint: descriptor.joint_id(),
                });
            }
            let limits = chain
                .model()
                .joint(descriptor.joint_id())
                .and_then(|joint| joint.limits())
                .ok_or(ArmAlgorithmError::InvalidModel {
                    reason: "prepared RMP active joint has no limits",
                })?;
            let maximum = limits.max_acceleration();
            let reach_value = clamp_component(
                self.config.reach_gain * (workspace.target[index] - workspace.current[index]),
                maximum,
            );
            let damping_value =
                clamp_component(-self.config.damping_gain * velocity.value_si(), maximum);
            let limit_value = clamp_component(
                joint_limit_repulsion(
                    chain,
                    descriptor.joint_id(),
                    workspace.current[index],
                    self.config,
                )?,
                maximum,
            );
            let singularity_value = clamp_component(
                -self.config.singularity_gain * singularity_scale * velocity.value_si(),
                maximum,
            );
            let total_value = clamp_component(
                reach_value + damping_value + limit_value + singularity_value,
                maximum,
            );
            reach.push(acceleration(descriptor.kind(), reach_value)?);
            damping.push(acceleration(descriptor.kind(), damping_value)?);
            joint_limit.push(acceleration(descriptor.kind(), limit_value)?);
            singularity.push(acceleration(descriptor.kind(), singularity_value)?);
            total.push(acceleration(descriptor.kind(), total_value)?);
        }
        let order = self.kinematics.order().clone();
        Ok(PreparedRmpOutput {
            reach: ArmJointVector::new(order.clone(), reach).map_err(rmp_output_error)?,
            damping: ArmJointVector::new(order.clone(), damping).map_err(rmp_output_error)?,
            joint_limit: ArmJointVector::new(order.clone(), joint_limit)
                .map_err(rmp_output_error)?,
            singularity: ArmJointVector::new(order.clone(), singularity)
                .map_err(rmp_output_error)?,
            acceleration: ArmJointVector::new(order, total).map_err(rmp_output_error)?,
            smallest_singular_value,
            singularity_rotations,
        })
    }
}

/// Reusable workspace for one prepared RMP identity.
pub struct RmpWorkspace {
    profile_id: message_model::ArmProfileId,
    group_id: message_model::KinematicGroupId,
    generation: u64,
    dof: usize,
    kinematics_workspace: IkWorkspace,
    jacobian: Vec<f64>,
    current: [f64; MAX_JOINTS],
    target: [f64; MAX_JOINTS],
}

impl RmpWorkspace {
    fn new(kinematics: &PreparedKinematics<'_>) -> Result<Self, ArmAlgorithmError> {
        Ok(Self {
            profile_id: kinematics.profile_id(),
            group_id: kinematics.group_id(),
            generation: kinematics.generation(),
            dof: kinematics.dof(),
            kinematics_workspace: kinematics.workspace(2)?,
            jacobian: vec![0.0; 6 * kinematics.dof()],
            current: [0.0; MAX_JOINTS],
            target: [0.0; MAX_JOINTS],
        })
    }

    /// Returns all storage reserved during preparation.
    #[must_use]
    pub fn allocated_bytes(&self) -> usize {
        self.kinematics_workspace.allocated_bytes()
            + self.jacobian.capacity() * std::mem::size_of::<f64>()
            + std::mem::size_of::<[f64; MAX_JOINTS]>() * 2
    }

    fn require(&self, kinematics: &PreparedKinematics<'_>) -> Result<(), ArmAlgorithmError> {
        if self.generation != kinematics.generation() {
            return Err(ArmAlgorithmError::PreparedGenerationMismatch {
                expected: kinematics.generation(),
                actual: self.generation,
            });
        }
        if self.profile_id != kinematics.profile_id() || self.group_id != kinematics.group_id() {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "RMP profile or group changed",
            });
        }
        if self.dof != kinematics.dof() {
            return Err(ArmAlgorithmError::PreparedWorkspaceMismatch {
                reason: "RMP degree of freedom changed",
            });
        }
        Ok(())
    }
}

/// Bounded RMP components and their combined acceleration.
#[derive(Clone, Debug, PartialEq)]
pub struct PreparedRmpOutput {
    reach: ArmJointVector<ArmAcceleration>,
    damping: ArmJointVector<ArmAcceleration>,
    joint_limit: ArmJointVector<ArmAcceleration>,
    singularity: ArmJointVector<ArmAcceleration>,
    acceleration: ArmJointVector<ArmAcceleration>,
    smallest_singular_value: f64,
    singularity_rotations: u32,
}

impl PreparedRmpOutput {
    /// Returns the bounded reach component.
    #[must_use]
    pub const fn reach(&self) -> &ArmJointVector<ArmAcceleration> {
        &self.reach
    }

    /// Returns the bounded velocity damping component.
    #[must_use]
    pub const fn damping(&self) -> &ArmJointVector<ArmAcceleration> {
        &self.damping
    }

    /// Returns the bounded joint-limit repulsion component.
    #[must_use]
    pub const fn joint_limit(&self) -> &ArmJointVector<ArmAcceleration> {
        &self.joint_limit
    }

    /// Returns the bounded singularity damping component.
    #[must_use]
    pub const fn singularity(&self) -> &ArmJointVector<ArmAcceleration> {
        &self.singularity
    }

    /// Returns the bounded total acceleration.
    #[must_use]
    pub const fn acceleration(&self) -> &ArmJointVector<ArmAcceleration> {
        &self.acceleration
    }

    /// Returns the smallest prepared-Jacobian singular value estimate.
    #[must_use]
    pub const fn smallest_singular_value(&self) -> f64 {
        self.smallest_singular_value
    }

    /// Returns the fixed-storage Jacobi rotations used for singularity evidence.
    #[must_use]
    pub const fn singularity_rotations(&self) -> u32 {
        self.singularity_rotations
    }
}

impl KinematicChain<'_> {
    /// Combines joint-space reach, damping and finite-limit repulsion policies.
    pub fn rmp_joint_acceleration(
        self,
        positions: &ArmJointVector<ArmPosition>,
        velocities: &ArmJointVector<ArmVelocity>,
        target: &ArmJointVector<ArmPosition>,
        config: RmpConfig,
    ) -> Result<ArmJointVector<ArmAcceleration>, ArmAlgorithmError> {
        let current = self.values(positions)?;
        let target = self.values(target)?;
        if velocities.order() != self.group().joint_order() {
            return Err(ArmAlgorithmError::JointOrderMismatch {
                group: self.group().id(),
            });
        }
        let mut output = ArrayVec::<ArmAcceleration, MAX_JOINTS>::new();
        for (index, descriptor) in self.group().joint_order().joints().iter().enumerate() {
            if velocities.values()[index].kind() != descriptor.kind().coordinate_kind() {
                return Err(ArmAlgorithmError::UnitMismatch {
                    joint: descriptor.joint_id(),
                });
            }
            let mut acceleration = config.reach_gain * (target[index] - current[index])
                - config.damping_gain * velocities.values()[index].value_si();
            let joint = self.model().joint(descriptor.joint_id()).ok_or(
                ArmAlgorithmError::InvalidModel {
                    reason: "RMP group references missing joint",
                },
            )?;
            if let Some([lower, upper]) = joint.limits().and_then(|limits| limits.position()) {
                let lower_distance = current[index] - lower;
                let upper_distance = upper - current[index];
                if lower_distance < config.joint_limit_margin {
                    acceleration += config.joint_limit_gain
                        * (1.0 / lower_distance.max(1.0e-6) - 1.0 / config.joint_limit_margin);
                }
                if upper_distance < config.joint_limit_margin {
                    acceleration -= config.joint_limit_gain
                        * (1.0 / upper_distance.max(1.0e-6) - 1.0 / config.joint_limit_margin);
                }
            }
            let limits = joint.limits().ok_or(ArmAlgorithmError::InvalidModel {
                reason: "RMP active joint has no limits",
            })?;
            acceleration =
                acceleration.clamp(-limits.max_acceleration(), limits.max_acceleration());
            output.push(
                match descriptor.kind() {
                    ArmJointKind::Revolute | ArmJointKind::Continuous => {
                        ArmAcceleration::angular_radps2(acceleration)
                    }
                    ArmJointKind::Prismatic => ArmAcceleration::linear_mps2(acceleration),
                }
                .map_err(|_| ArmAlgorithmError::NonFinite {
                    field: "reactive.rmp_acceleration",
                })?,
            );
        }
        ArmJointVector::new(self.group().joint_order().clone(), output).map_err(|_| {
            ArmAlgorithmError::InvalidModel {
                reason: "RMP output dimension changed",
            }
        })
    }

    /// Explicitly rejects obstacle/self-collision RMP without a provider.
    pub fn require_collision_rmp(self) -> Result<(), ArmAlgorithmError> {
        let _ = self;
        Err(ArmAlgorithmError::CollisionCapabilityUnavailable {
            surface: "reactive_rmp",
        })
    }
}

fn joint_limit_repulsion(
    chain: KinematicChain<'_>,
    joint_id: ArmJointId,
    position: f64,
    config: RmpConfig,
) -> Result<f64, ArmAlgorithmError> {
    let joint = chain
        .model()
        .joint(joint_id)
        .ok_or(ArmAlgorithmError::InvalidModel {
            reason: "prepared RMP group references missing joint",
        })?;
    let Some([lower, upper]) = joint.limits().and_then(|limits| limits.position()) else {
        return Ok(0.0);
    };
    let lower_distance = position - lower;
    let upper_distance = upper - position;
    let mut repulsion = 0.0;
    if lower_distance < config.joint_limit_margin {
        repulsion += config.joint_limit_gain
            * (1.0 / lower_distance.max(1.0e-6) - 1.0 / config.joint_limit_margin);
    }
    if upper_distance < config.joint_limit_margin {
        repulsion -= config.joint_limit_gain
            * (1.0 / upper_distance.max(1.0e-6) - 1.0 / config.joint_limit_margin);
    }
    Ok(repulsion)
}

fn clamp_component(value: f64, maximum: f64) -> f64 {
    value.clamp(-maximum, maximum)
}

fn acceleration(kind: ArmJointKind, value: f64) -> Result<ArmAcceleration, ArmAlgorithmError> {
    match kind {
        ArmJointKind::Revolute | ArmJointKind::Continuous => ArmAcceleration::angular_radps2(value),
        ArmJointKind::Prismatic => ArmAcceleration::linear_mps2(value),
    }
    .map_err(|_| ArmAlgorithmError::NonFinite {
        field: "reactive.prepared_rmp_acceleration",
    })
}

fn rmp_output_error<T>(_error: T) -> ArmAlgorithmError {
    ArmAlgorithmError::InvalidModel {
        reason: "prepared RMP output dimension changed",
    }
}

fn smallest_singular_value(jacobian: &[f64], dof: usize) -> (f64, u32) {
    let mut gram = [0.0_f64; MAX_JOINTS * MAX_JOINTS];
    let mut rotations = 0_u32;
    for row in 0..dof {
        for column in 0..=row {
            let value = (0..6)
                .map(|axis| jacobian[axis * dof + row] * jacobian[axis * dof + column])
                .sum::<f64>();
            gram[row * MAX_JOINTS + column] = value;
            gram[column * MAX_JOINTS + row] = value;
        }
    }

    // Jacobi rotations on the fixed maximum joint matrix avoid an allocation
    // or a dynamic SVD on the warm reactive-control path.
    for _ in 0..(MAX_JOINTS * MAX_JOINTS * 4) {
        let mut pivot = None;
        let mut magnitude = 0.0_f64;
        for row in 0..dof {
            for column in (row + 1)..dof {
                let candidate = gram[row * MAX_JOINTS + column].abs();
                if candidate > magnitude {
                    magnitude = candidate;
                    pivot = Some((row, column));
                }
            }
        }
        let Some((left, right)) = pivot else {
            break;
        };
        if magnitude <= 1.0e-12 {
            break;
        }
        let left_value = gram[left * MAX_JOINTS + left];
        let right_value = gram[right * MAX_JOINTS + right];
        let angle = 0.5 * (2.0 * gram[left * MAX_JOINTS + right]).atan2(right_value - left_value);
        let cosine = angle.cos();
        let sine = angle.sin();
        for index in 0..dof {
            if index == left || index == right {
                continue;
            }
            let left_entry = gram[index * MAX_JOINTS + left];
            let right_entry = gram[index * MAX_JOINTS + right];
            let rotated_left = cosine.mul_add(left_entry, -sine * right_entry);
            let rotated_right = sine.mul_add(left_entry, cosine * right_entry);
            gram[index * MAX_JOINTS + left] = rotated_left;
            gram[left * MAX_JOINTS + index] = rotated_left;
            gram[index * MAX_JOINTS + right] = rotated_right;
            gram[right * MAX_JOINTS + index] = rotated_right;
        }
        let diagonal_left = cosine * cosine * left_value
            - 2.0 * sine * cosine * gram[left * MAX_JOINTS + right]
            + sine * sine * right_value;
        let diagonal_right = sine * sine * left_value
            + 2.0 * sine * cosine * gram[left * MAX_JOINTS + right]
            + cosine * cosine * right_value;
        gram[left * MAX_JOINTS + left] = diagonal_left;
        gram[right * MAX_JOINTS + right] = diagonal_right;
        gram[left * MAX_JOINTS + right] = 0.0;
        gram[right * MAX_JOINTS + left] = 0.0;
        rotations = rotations.saturating_add(1);
    }
    (
        (0..dof)
            .map(|index| gram[index * MAX_JOINTS + index].max(0.0).sqrt())
            .fold(f64::INFINITY, f64::min),
        rotations,
    )
}

fn clamp_velocities(
    chain: KinematicChain<'_>,
    values: &mut [f64],
    scale: f64,
) -> Result<(), ArmAlgorithmError> {
    if values.len() != chain.dof() {
        return Err(ArmAlgorithmError::DimensionMismatch {
            field: "reactive.velocity",
            expected: chain.dof(),
            actual: values.len(),
        });
    }
    for (value, descriptor) in values.iter_mut().zip(chain.group().joint_order().joints()) {
        let limit = chain
            .model()
            .joint(descriptor.joint_id())
            .and_then(|joint| joint.limits())
            .ok_or(ArmAlgorithmError::InvalidModel {
                reason: "reactive joint has no limits",
            })?
            .max_velocity()
            * scale;
        *value = value.clamp(-limit, limit);
    }
    Ok(())
}

fn velocity_vector(
    chain: KinematicChain<'_>,
    values: &[f64],
) -> Result<ArmJointVector<ArmVelocity>, ArmAlgorithmError> {
    let mut output = ArrayVec::<ArmVelocity, MAX_JOINTS>::new();
    for (descriptor, value) in chain.group().joint_order().joints().iter().zip(values) {
        output.push(
            match descriptor.kind() {
                ArmJointKind::Revolute | ArmJointKind::Continuous => {
                    ArmVelocity::angular_radps(*value)
                }
                ArmJointKind::Prismatic => ArmVelocity::linear_mps(*value),
            }
            .map_err(|_| ArmAlgorithmError::NonFinite {
                field: "reactive.velocity",
            })?,
        );
    }
    ArmJointVector::new(chain.group().joint_order().clone(), output).map_err(|_| {
        ArmAlgorithmError::InvalidModel {
            reason: "reactive velocity output dimension changed",
        }
    })
}
