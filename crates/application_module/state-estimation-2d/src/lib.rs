//! Deterministic local-monotonic wheel/IMU planar state estimation.

use chassis_model::{
    BodyPose2, BodyTwist, DataQuality, KinematicsError, TimedSample, body_twist,
    require_closed_unit, require_finite,
};
use thiserror::Error;

/// Finite IMU yaw-rate observation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct YawRate(f64);

impl YawRate {
    /// Constructs a finite yaw rate.
    pub fn new(radps: f64) -> Result<Self, KinematicsError> {
        Ok(Self(require_finite(radps, "imu.yaw_rate_radps")?))
    }

    /// Returns radians per second.
    #[must_use]
    pub const fn radians_per_second(self) -> f64 {
        self.0
    }
}

/// Behavior when IMU input cannot be used.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ImuLossPolicy {
    /// Continue with wheel yaw rate and mark the estimate degraded.
    WheelOnly,
    /// Refuse to advance the estimator.
    Stop,
}

/// Whether external pose is advisory or required for nominal quality.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ExternalPosePolicy {
    /// Missing correction is reported but does not degrade wheel/IMU odometry.
    Optional,
    /// Missing correction marks an otherwise usable estimate degraded.
    Required,
}

/// Validated estimator timing, fusion and degradation policy.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EstimatorConfig {
    max_wheel_age_ns: u64,
    max_imu_age_ns: u64,
    max_external_age_ns: u64,
    max_wheel_imu_skew_ns: u64,
    max_step_ns: u64,
    imu_yaw_weight: f64,
    external_correction_gain: f64,
    imu_loss_policy: ImuLossPolicy,
    external_pose_policy: ExternalPosePolicy,
}

impl EstimatorConfig {
    /// Constructs a policy with nonzero time bounds and closed-unit gains.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        max_wheel_age_ns: u64,
        max_imu_age_ns: u64,
        max_external_age_ns: u64,
        max_wheel_imu_skew_ns: u64,
        max_step_ns: u64,
        imu_yaw_weight: f64,
        external_correction_gain: f64,
        imu_loss_policy: ImuLossPolicy,
        external_pose_policy: ExternalPosePolicy,
    ) -> Result<Self, EstimatorError> {
        if [
            max_wheel_age_ns,
            max_imu_age_ns,
            max_external_age_ns,
            max_step_ns,
        ]
        .contains(&0)
        {
            return Err(EstimatorError::ZeroTimeBound);
        }
        Ok(Self {
            max_wheel_age_ns,
            max_imu_age_ns,
            max_external_age_ns,
            max_wheel_imu_skew_ns,
            max_step_ns,
            imu_yaw_weight: require_closed_unit(imu_yaw_weight, "imu_yaw_weight")?,
            external_correction_gain: require_closed_unit(
                external_correction_gain,
                "external_correction_gain",
            )?,
            imu_loss_policy,
            external_pose_policy,
        })
    }
}

/// Samples offered to one serialized estimator step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EstimatorInputs {
    /// Required body velocity derived from wheel kinematics.
    pub wheel_body: Option<TimedSample<BodyTwist>>,
    /// IMU yaw rate selected by policy.
    pub imu_yaw_rate: Option<TimedSample<YawRate>>,
    /// Optional/required external pose correction.
    pub external_pose: Option<TimedSample<BodyPose2>>,
}

/// Input gate failure retained as estimator evidence.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InputRejection {
    /// No sample was supplied.
    Missing,
    /// Timestamp is ahead of the caller's local monotonic time.
    Future,
    /// Age exceeded the configured maximum.
    Stale,
    /// Timestamp predates the last accepted sample for that input.
    OutOfOrder,
    /// Quality requires rejection by the estimator.
    UnusableQuality,
    /// Wheel and IMU timestamps were too far apart.
    ExcessiveSkew,
}

/// Why an estimator step could not produce a control-usable state.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EstimateUnavailable {
    /// Required wheel velocity was unavailable.
    Wheel(InputRejection),
    /// IMU policy requires stopping.
    Imu(InputRejection),
}

/// Detailed input/fallback evidence for a completed estimator call.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct EstimatorEvidence {
    /// At least one consumed input declared degraded quality.
    pub degraded_input: bool,
    /// Wheel yaw rate replaced an unavailable IMU.
    pub wheel_only_yaw: bool,
    /// A usable external correction was applied.
    pub external_pose_used: bool,
    /// External pose was absent or rejected.
    pub external_pose_unavailable: bool,
}

/// Quality of an estimate that advanced.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EstimateQuality {
    /// All required inputs and policies were nominal.
    Nominal,
    /// State advanced under an explicit degraded policy.
    Degraded,
}

/// Current 2D estimate.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EstimatedState {
    /// Pose in the estimator's world/odometry frame.
    pub pose: BodyPose2,
    /// Fused current body velocity.
    pub body_velocity: BodyTwist,
    /// Local monotonic time represented by the estimate.
    pub monotonic_ns: u64,
}

/// Result of one estimator step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EstimatorStep {
    /// State advanced and is usable under the returned quality.
    Advanced {
        /// Updated state.
        state: EstimatedState,
        /// Nominal or degraded state quality.
        quality: EstimateQuality,
        /// Exact input/fallback evidence.
        evidence: EstimatorEvidence,
    },
    /// State did not advance.
    Unavailable {
        /// Required unavailable source.
        reason: EstimateUnavailable,
        /// Optional/fallback evidence observed before refusal.
        evidence: EstimatorEvidence,
    },
}

/// Estimator configuration or caller-time failure.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum EstimatorError {
    /// A configured freshness or step bound was zero.
    #[error("estimator time bounds must be nonzero")]
    ZeroTimeBound,
    /// Caller step time did not advance.
    #[error("estimator step time must increase monotonically")]
    NonMonotonicStep,
    /// Caller attempted a step larger than the configured integration bound.
    #[error("estimator step exceeds maximum")]
    StepTooLarge,
    /// Shared numeric validation failed.
    #[error(transparent)]
    Kinematics(#[from] KinematicsError),
}

/// Serialized deterministic 2D estimator.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Estimator2d {
    config: EstimatorConfig,
    state: EstimatedState,
    last_wheel_ns: Option<u64>,
    last_imu_ns: Option<u64>,
    last_external_ns: Option<u64>,
}

impl Estimator2d {
    /// Starts from an explicit pose and local monotonic time.
    pub fn new(
        config: EstimatorConfig,
        initial_pose: BodyPose2,
        monotonic_ns: u64,
    ) -> Result<Self, EstimatorError> {
        Ok(Self {
            config,
            state: EstimatedState {
                pose: initial_pose,
                body_velocity: body_twist([0.0, 0.0, 0.0])?,
                monotonic_ns,
            },
            last_wheel_ns: None,
            last_imu_ns: None,
            last_external_ns: None,
        })
    }

    /// Returns the last successfully advanced state.
    #[must_use]
    pub const fn state(self) -> EstimatedState {
        self.state
    }

    /// Gates inputs, fuses yaw rate, integrates pose and applies correction.
    pub fn step(
        &mut self,
        now_monotonic_ns: u64,
        inputs: EstimatorInputs,
    ) -> Result<EstimatorStep, EstimatorError> {
        let Some(step_ns) = now_monotonic_ns.checked_sub(self.state.monotonic_ns) else {
            return Err(EstimatorError::NonMonotonicStep);
        };
        if step_ns == 0 {
            return Err(EstimatorError::NonMonotonicStep);
        }
        if step_ns > self.config.max_step_ns {
            return Err(EstimatorError::StepTooLarge);
        }

        let mut evidence = EstimatorEvidence::default();
        let wheel = match gate(
            inputs.wheel_body,
            now_monotonic_ns,
            self.config.max_wheel_age_ns,
            self.last_wheel_ns,
        ) {
            Ok(sample) => sample,
            Err(reason) => {
                return Ok(EstimatorStep::Unavailable {
                    reason: EstimateUnavailable::Wheel(reason),
                    evidence,
                });
            }
        };
        evidence.degraded_input |= wheel.quality() == DataQuality::Degraded;

        let imu_gate = gate(
            inputs.imu_yaw_rate,
            now_monotonic_ns,
            self.config.max_imu_age_ns,
            self.last_imu_ns,
        )
        .and_then(|sample| {
            let skew = wheel
                .sample_monotonic_ns()
                .abs_diff(sample.sample_monotonic_ns());
            (skew <= self.config.max_wheel_imu_skew_ns)
                .then_some(sample)
                .ok_or(InputRejection::ExcessiveSkew)
        });
        let mut accepted_imu_ns = None;
        let yaw_rate = match imu_gate {
            Ok(imu) => {
                evidence.degraded_input |= imu.quality() == DataQuality::Degraded;
                accepted_imu_ns = Some(imu.sample_monotonic_ns());
                let wheel_yaw = wheel.value().components()[2];
                (imu.value().radians_per_second() - wheel_yaw)
                    .mul_add(self.config.imu_yaw_weight, wheel_yaw)
            }
            Err(reason) => match self.config.imu_loss_policy {
                ImuLossPolicy::WheelOnly => {
                    evidence.wheel_only_yaw = true;
                    wheel.value().components()[2]
                }
                ImuLossPolicy::Stop => {
                    return Ok(EstimatorStep::Unavailable {
                        reason: EstimateUnavailable::Imu(reason),
                        evidence,
                    });
                }
            },
        };

        let [vx, vy, _] = wheel.value().components();
        let fused_body = body_twist([vx, vy, yaw_rate])?;
        let dt_s = step_ns as f64 * 1.0e-9;
        let mut pose = self.state.pose.integrate_body(fused_body, dt_s)?;

        let external_floor = self
            .last_external_ns
            .map_or(self.state.monotonic_ns, |last| {
                last.max(self.state.monotonic_ns)
            });
        match gate(
            inputs.external_pose,
            now_monotonic_ns,
            self.config.max_external_age_ns,
            Some(external_floor),
        ) {
            Ok(external) => {
                evidence.degraded_input |= external.quality() == DataQuality::Degraded;
                pose = pose.interpolate(*external.value(), self.config.external_correction_gain)?;
                evidence.external_pose_used = true;
                self.last_external_ns = Some(external.sample_monotonic_ns());
            }
            Err(_) => evidence.external_pose_unavailable = true,
        }

        let required_external_missing = evidence.external_pose_unavailable
            && self.config.external_pose_policy == ExternalPosePolicy::Required;
        let quality =
            if evidence.degraded_input || evidence.wheel_only_yaw || required_external_missing {
                EstimateQuality::Degraded
            } else {
                EstimateQuality::Nominal
            };
        self.last_wheel_ns = Some(wheel.sample_monotonic_ns());
        if accepted_imu_ns.is_some() {
            self.last_imu_ns = accepted_imu_ns;
        }
        self.state = EstimatedState {
            pose,
            body_velocity: fused_body,
            monotonic_ns: now_monotonic_ns,
        };
        Ok(EstimatorStep::Advanced {
            state: self.state,
            quality,
            evidence,
        })
    }
}

fn gate<T: Copy>(
    sample: Option<TimedSample<T>>,
    now_ns: u64,
    max_age_ns: u64,
    last_accepted_ns: Option<u64>,
) -> Result<TimedSample<T>, InputRejection> {
    let sample = sample.ok_or(InputRejection::Missing)?;
    if sample.sample_monotonic_ns() > now_ns {
        return Err(InputRejection::Future);
    }
    if now_ns - sample.sample_monotonic_ns() > max_age_ns {
        return Err(InputRejection::Stale);
    }
    if last_accepted_ns.is_some_and(|last| sample.sample_monotonic_ns() <= last) {
        return Err(InputRejection::OutOfOrder);
    }
    if !matches!(sample.quality(), DataQuality::Valid | DataQuality::Degraded) {
        return Err(InputRejection::UnusableQuality);
    }
    Ok(sample)
}

#[cfg(test)]
mod tests {
    use allocation_counter::measure;
    use chassis_model::{BodyPose2, DataQuality, TimedSample, body_twist};

    use super::*;

    fn config(imu: ImuLossPolicy, external: ExternalPosePolicy) -> EstimatorConfig {
        EstimatorConfig::new(
            200_000_000,
            200_000_000,
            500_000_000,
            20_000_000,
            1_000_000_000,
            1.0,
            0.5,
            imu,
            external,
        )
        .unwrap()
    }

    fn inputs(at: u64) -> EstimatorInputs {
        EstimatorInputs {
            wheel_body: Some(TimedSample::new(
                body_twist([1.0, 0.0, 0.2]).unwrap(),
                at,
                DataQuality::Valid,
            )),
            imu_yaw_rate: Some(TimedSample::new(
                YawRate::new(0.4).unwrap(),
                at,
                DataQuality::Valid,
            )),
            external_pose: None,
        }
    }

    #[test]
    fn nominal_fusion_integrates_wheel_translation_and_imu_yaw() {
        let mut estimator = Estimator2d::new(
            config(ImuLossPolicy::Stop, ExternalPosePolicy::Optional),
            BodyPose2::new(0.0, 0.0, 0.0).unwrap(),
            0,
        )
        .unwrap();
        let EstimatorStep::Advanced { state, quality, .. } =
            estimator.step(1_000_000_000, inputs(990_000_000)).unwrap()
        else {
            panic!("must advance");
        };
        assert_eq!(quality, EstimateQuality::Nominal);
        assert!((state.body_velocity.components()[2] - 0.4).abs() < 1.0e-12);
        assert!(state.pose.components()[0] > 0.9);
    }

    #[test]
    fn stale_future_skew_and_reorder_never_advance_state() {
        for (now, offered, expected) in [
            (1_000_000_000, inputs(700_000_000), InputRejection::Stale),
            (1_000, inputs(1_001), InputRejection::Future),
        ] {
            let mut estimator = Estimator2d::new(
                config(ImuLossPolicy::Stop, ExternalPosePolicy::Optional),
                BodyPose2::new(0.0, 0.0, 0.0).unwrap(),
                0,
            )
            .unwrap();
            assert!(matches!(
                estimator.step(now, offered).unwrap(),
                EstimatorStep::Unavailable { reason: EstimateUnavailable::Wheel(reason), .. } if reason == expected
            ));
            assert_eq!(estimator.state().monotonic_ns, 0);
        }

        let mut estimator = Estimator2d::new(
            config(ImuLossPolicy::Stop, ExternalPosePolicy::Optional),
            BodyPose2::new(0.0, 0.0, 0.0).unwrap(),
            0,
        )
        .unwrap();
        let mut skewed = inputs(900_000_000);
        skewed.imu_yaw_rate = Some(TimedSample::new(
            YawRate::new(0.1).unwrap(),
            950_000_001,
            DataQuality::Valid,
        ));
        assert!(matches!(
            estimator.step(1_000_000_000, skewed).unwrap(),
            EstimatorStep::Unavailable {
                reason: EstimateUnavailable::Imu(InputRejection::ExcessiveSkew),
                ..
            }
        ));

        let mut estimator = Estimator2d::new(
            config(ImuLossPolicy::Stop, ExternalPosePolicy::Optional),
            BodyPose2::new(0.0, 0.0, 0.0).unwrap(),
            0,
        )
        .unwrap();
        estimator.step(500_000_000, inputs(490_000_000)).unwrap();
        assert!(matches!(
            estimator.step(600_000_000, inputs(490_000_000)).unwrap(),
            EstimatorStep::Unavailable {
                reason: EstimateUnavailable::Wheel(InputRejection::OutOfOrder),
                ..
            }
        ));
    }

    #[test]
    fn imu_and_external_loss_follow_explicit_policies() {
        let mut wheel_only = Estimator2d::new(
            config(ImuLossPolicy::WheelOnly, ExternalPosePolicy::Required),
            BodyPose2::new(0.0, 0.0, 0.0).unwrap(),
            0,
        )
        .unwrap();
        let mut offered = inputs(900_000_000);
        offered.imu_yaw_rate = None;
        let EstimatorStep::Advanced {
            quality, evidence, ..
        } = wheel_only.step(1_000_000_000, offered).unwrap()
        else {
            panic!("wheel-only policy advances");
        };
        assert_eq!(quality, EstimateQuality::Degraded);
        assert!(evidence.wheel_only_yaw && evidence.external_pose_unavailable);

        let mut stop = Estimator2d::new(
            config(ImuLossPolicy::Stop, ExternalPosePolicy::Optional),
            BodyPose2::new(0.0, 0.0, 0.0).unwrap(),
            0,
        )
        .unwrap();
        assert!(matches!(
            stop.step(1_000_000_000, offered).unwrap(),
            EstimatorStep::Unavailable {
                reason: EstimateUnavailable::Imu(InputRejection::Missing),
                ..
            }
        ));
    }

    #[test]
    fn external_pose_correction_is_bounded_and_wrap_safe() {
        let mut estimator = Estimator2d::new(
            config(ImuLossPolicy::Stop, ExternalPosePolicy::Optional),
            BodyPose2::new(0.0, 0.0, 3.0).unwrap(),
            0,
        )
        .unwrap();
        let mut offered = inputs(900_000_000);
        offered.external_pose = Some(TimedSample::new(
            BodyPose2::new(2.0, 0.0, -3.0).unwrap(),
            900_000_000,
            DataQuality::Valid,
        ));
        let EstimatorStep::Advanced {
            state, evidence, ..
        } = estimator.step(1_000_000_000, offered).unwrap()
        else {
            panic!("must advance");
        };
        assert!(evidence.external_pose_used);
        assert!(state.pose.components()[0] > 0.4 && state.pose.components()[0] < 0.6);
        assert!(state.pose.components()[2].abs() > 2.5);
    }

    #[test]
    fn estimator_hot_path_allocates_nothing() {
        let mut estimator = Estimator2d::new(
            config(ImuLossPolicy::Stop, ExternalPosePolicy::Optional),
            BodyPose2::new(0.0, 0.0, 0.0).unwrap(),
            0,
        )
        .unwrap();
        let allocations = measure(|| {
            let _ = estimator.step(1_000_000_000, inputs(900_000_000)).unwrap();
        });
        assert_eq!(allocations.count_total, 0);
    }
}
