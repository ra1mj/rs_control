//! Reactive servo, filtering and collision-independent RMP policies.

use arrayvec::ArrayVec;
use message_model::{
    ArmAcceleration, ArmJointKind, ArmJointVector, ArmPosition, ArmVelocity, MAX_JOINTS,
};
use nalgebra::{DMatrix, DVector};

use crate::{
    ArmAlgorithmError,
    kinematics::{ArmPose, KinematicChain},
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
        })
    }

    /// Returns balanced host defaults.
    #[must_use]
    pub const fn host_default() -> Self {
        Self {
            reach_gain: 4.0,
            damping_gain: 1.5,
            joint_limit_gain: 0.2,
            joint_limit_margin: 0.2,
        }
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
