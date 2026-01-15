//! Bounded joint-space trajectory generation, validation and sampling.

mod monitor;

use arrayvec::ArrayVec;
use message_model::{
    ArmAcceleration, ArmJointKind, ArmJointOrder, ArmJointVector, ArmPosition, ArmProfileId,
    ArmTrajectoryPoint as SemanticTrajectoryPoint, ArmVelocity, ExecuteArmTrajectoryGoal,
    KinematicGroupId, MAX_JOINTS, MAX_TRAJECTORY_POINTS,
};

use crate::{ArmAlgorithmError, TrajectoryViolationKind, kinematics::KinematicChain};

pub use monitor::*;

/// Maximum represented trajectory duration: one hour.
pub const MAX_TRAJECTORY_DURATION_NS: u64 = 3_600_000_000_000;
const MIN_TRAJECTORY_DURATION_NS: u64 = 1_000;
const MIN_SAMPLE_COUNT: usize = 2;
const QUINTIC_MAX_VELOCITY_FACTOR: f64 = 1.875;
const QUINTIC_MAX_ACCELERATION_FACTOR: f64 = 5.773_502_691_896_258;
const QUINTIC_MAX_JERK_FACTOR: f64 = 60.0;

/// Bounded generator output density.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TrajectorySampling {
    sample_count: usize,
}

impl TrajectorySampling {
    /// Requires at least a start/end pair and at most the protocol bound.
    pub fn new(sample_count: usize) -> Result<Self, ArmAlgorithmError> {
        if !(MIN_SAMPLE_COUNT..=MAX_TRAJECTORY_POINTS).contains(&sample_count) {
            return Err(ArmAlgorithmError::Capacity {
                field: "trajectory.sample_count",
                actual: sample_count,
                maximum: MAX_TRAJECTORY_POINTS,
            });
        }
        Ok(Self { sample_count })
    }

    /// Returns the exact number of generated points.
    #[must_use]
    pub const fn count(self) -> usize {
        self.sample_count
    }
}

/// One heap-backed point whose scalar units are owned by trajectory order.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmTrajectorySample {
    time_from_start_ns: u64,
    positions_si: Vec<f64>,
    velocities_si: Vec<f64>,
    accelerations_si: Vec<f64>,
    jerks_si: Vec<f64>,
}

impl ArmTrajectorySample {
    /// Returns the strictly positive offset from trajectory start.
    #[must_use]
    pub const fn time_from_start_ns(&self) -> u64 {
        self.time_from_start_ns
    }

    /// Returns joint positions in trajectory order (rad or m).
    #[must_use]
    pub fn positions_si(&self) -> &[f64] {
        &self.positions_si
    }

    /// Returns joint velocities in trajectory order (rad/s or m/s).
    #[must_use]
    pub fn velocities_si(&self) -> &[f64] {
        &self.velocities_si
    }

    /// Returns joint accelerations in trajectory order (rad/s² or m/s²).
    #[must_use]
    pub fn accelerations_si(&self) -> &[f64] {
        &self.accelerations_si
    }

    /// Returns joint jerks in trajectory order (rad/s³ or m/s³).
    #[must_use]
    pub fn jerks_si(&self) -> &[f64] {
        &self.jerks_si
    }
}

/// Validated, bounded, profile/group/order-bound joint trajectory.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmTrajectory {
    profile_id: ArmProfileId,
    group_id: KinematicGroupId,
    order: ArmJointOrder,
    points: Vec<ArmTrajectorySample>,
}

impl ArmTrajectory {
    /// Generates a synchronized per-joint triangular/trapezoidal trajectory.
    pub fn trapezoidal(
        chain: KinematicChain<'_>,
        start: &ArmJointVector<ArmPosition>,
        goal: &ArmJointVector<ArmPosition>,
        sampling: TrajectorySampling,
    ) -> Result<Self, ArmAlgorithmError> {
        let start_values = chain.values(start)?;
        let goal_values = chain.values(goal)?;
        let limits = motion_limits(chain, None)?;
        let axes = start_values
            .iter()
            .zip(&goal_values)
            .zip(&limits)
            .map(|((&start, &goal), limits)| {
                TrapezoidAxis::new(start, goal, limits.max_velocity, limits.max_acceleration)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let duration_s = axes
            .iter()
            .map(|axis| axis.duration_s)
            .fold(0.0_f64, f64::max);
        let duration_ns = bounded_duration_ns(duration_s, sampling.count())?;
        let duration_s = duration_ns as f64 * 1.0e-9;
        let mut points = Vec::with_capacity(sampling.count());

        for index in 0..sampling.count() {
            let progress = normalized_sample(index, sampling.count());
            let evaluation_time = duration_s * progress;
            let mut positions = Vec::with_capacity(chain.dof());
            let mut velocities = Vec::with_capacity(chain.dof());
            let mut accelerations = Vec::with_capacity(chain.dof());
            let mut jerks = Vec::with_capacity(chain.dof());
            for axis in &axes {
                let [position, velocity, acceleration] =
                    axis.sample_synchronized(evaluation_time, duration_s);
                positions.push(position);
                velocities.push(velocity);
                accelerations.push(acceleration);
                jerks.push(0.0);
            }
            points.push(ArmTrajectorySample {
                time_from_start_ns: sample_time_ns(index, sampling.count(), duration_ns),
                positions_si: positions,
                velocities_si: velocities,
                accelerations_si: accelerations,
                jerks_si: jerks,
            });
        }
        Self::validated(chain, points, None)
    }

    /// Generates a quintic jerk-bounded S-profile with zero endpoint rates.
    pub fn jerk_limited(
        chain: KinematicChain<'_>,
        start: &ArmJointVector<ArmPosition>,
        goal: &ArmJointVector<ArmPosition>,
        maximum_jerk_si: &[f64],
        sampling: TrajectorySampling,
    ) -> Result<Self, ArmAlgorithmError> {
        let start_values = chain.values(start)?;
        let goal_values = chain.values(goal)?;
        let limits = motion_limits(chain, Some(maximum_jerk_si))?;
        let mut duration_s = 0.0_f64;
        for ((start, goal), limits) in start_values.iter().zip(&goal_values).zip(&limits) {
            let distance = (goal - start).abs();
            duration_s = duration_s.max(
                (QUINTIC_MAX_VELOCITY_FACTOR * distance / limits.max_velocity)
                    .max(
                        (QUINTIC_MAX_ACCELERATION_FACTOR * distance / limits.max_acceleration)
                            .sqrt(),
                    )
                    .max((QUINTIC_MAX_JERK_FACTOR * distance / limits.max_jerk).cbrt()),
            );
        }
        let duration_ns = bounded_duration_ns(duration_s, sampling.count())?;
        let duration_s = duration_ns as f64 * 1.0e-9;
        let mut points = Vec::with_capacity(sampling.count());

        for index in 0..sampling.count() {
            let s = normalized_sample(index, sampling.count());
            let [shape, d1, d2, d3] = quintic_shape(s);
            let mut positions = Vec::with_capacity(chain.dof());
            let mut velocities = Vec::with_capacity(chain.dof());
            let mut accelerations = Vec::with_capacity(chain.dof());
            let mut jerks = Vec::with_capacity(chain.dof());
            for (&start, &goal) in start_values.iter().zip(&goal_values) {
                let delta = goal - start;
                positions.push(delta.mul_add(shape, start));
                velocities.push(delta * d1 / duration_s);
                accelerations.push(delta * d2 / duration_s.powi(2));
                jerks.push(delta * d3 / duration_s.powi(3));
            }
            points.push(ArmTrajectorySample {
                time_from_start_ns: sample_time_ns(index, sampling.count(), duration_ns),
                positions_si: positions,
                velocities_si: velocities,
                accelerations_si: accelerations,
                jerks_si: jerks,
            });
        }
        Self::validated(chain, points, Some(maximum_jerk_si))
    }

    /// Generates one clamped cubic-Hermite segment.
    pub fn cubic_clamped(
        chain: KinematicChain<'_>,
        start: &ArmJointVector<ArmPosition>,
        goal: &ArmJointVector<ArmPosition>,
        start_velocity_si: &[f64],
        goal_velocity_si: &[f64],
        duration_ns: u64,
        sampling: TrajectorySampling,
    ) -> Result<Self, ArmAlgorithmError> {
        let start_values = chain.values(start)?;
        let goal_values = chain.values(goal)?;
        validate_dimension(
            chain.dof(),
            start_velocity_si.len(),
            "trajectory.start_velocity",
        )?;
        validate_dimension(
            chain.dof(),
            goal_velocity_si.len(),
            "trajectory.goal_velocity",
        )?;
        if !(MIN_TRAJECTORY_DURATION_NS..=MAX_TRAJECTORY_DURATION_NS).contains(&duration_ns)
            || duration_ns < sampling.count() as u64
        {
            return Err(ArmAlgorithmError::InvalidTime {
                field: "trajectory.duration_ns",
            });
        }
        if start_velocity_si
            .iter()
            .chain(goal_velocity_si)
            .any(|value| !value.is_finite())
        {
            return Err(ArmAlgorithmError::NonFinite {
                field: "trajectory.endpoint_velocity",
            });
        }
        let duration_s = duration_ns as f64 * 1.0e-9;
        let mut points = Vec::with_capacity(sampling.count());
        for index in 0..sampling.count() {
            let s = normalized_sample(index, sampling.count());
            let mut positions = Vec::with_capacity(chain.dof());
            let mut velocities = Vec::with_capacity(chain.dof());
            let mut accelerations = Vec::with_capacity(chain.dof());
            let mut jerks = Vec::with_capacity(chain.dof());
            for (((&p0, &p1), &v0), &v1) in start_values
                .iter()
                .zip(&goal_values)
                .zip(start_velocity_si)
                .zip(goal_velocity_si)
            {
                let coefficients = CubicAxis::new(p0, p1, v0, v1, duration_s);
                let [position, velocity, acceleration, jerk] = coefficients.sample(s * duration_s);
                positions.push(position);
                velocities.push(velocity);
                accelerations.push(acceleration);
                jerks.push(jerk);
            }
            points.push(ArmTrajectorySample {
                time_from_start_ns: sample_time_ns(index, sampling.count(), duration_ns),
                positions_si: positions,
                velocities_si: velocities,
                accelerations_si: accelerations,
                jerks_si: jerks,
            });
        }
        Self::validated(chain, points, None)
    }

    /// Time-parameterizes a bounded waypoint path as synchronized trapezoidal segments.
    pub fn parameterize_path(
        chain: KinematicChain<'_>,
        waypoints: &[ArmJointVector<ArmPosition>],
        samples_per_segment: TrajectorySampling,
    ) -> Result<Self, ArmAlgorithmError> {
        if waypoints.len() < 2 || waypoints.len() > MAX_TRAJECTORY_POINTS {
            return Err(ArmAlgorithmError::Capacity {
                field: "trajectory.waypoints",
                actual: waypoints.len(),
                maximum: MAX_TRAJECTORY_POINTS,
            });
        }
        let point_count = 1 + (waypoints.len() - 1) * (samples_per_segment.count() - 1);
        if point_count > MAX_TRAJECTORY_POINTS {
            return Err(ArmAlgorithmError::Capacity {
                field: "trajectory.parameterized_points",
                actual: point_count,
                maximum: MAX_TRAJECTORY_POINTS,
            });
        }
        let mut points = Vec::with_capacity(point_count);
        let mut time_offset_ns = 0_u64;
        for (segment_index, pair) in waypoints.windows(2).enumerate() {
            let segment = Self::trapezoidal(chain, &pair[0], &pair[1], samples_per_segment)?;
            for (point_index, point) in segment.points.into_iter().enumerate() {
                if segment_index > 0 && point_index == 0 {
                    continue;
                }
                let local_time = if segment_index == 0 {
                    point.time_from_start_ns
                } else {
                    point.time_from_start_ns.saturating_sub(1)
                };
                let shifted = time_offset_ns.checked_add(local_time).ok_or(
                    ArmAlgorithmError::InvalidTime {
                        field: "trajectory.path_duration",
                    },
                )?;
                points.push(ArmTrajectorySample {
                    time_from_start_ns: shifted,
                    ..point
                });
            }
            time_offset_ns = points
                .last()
                .ok_or(ArmAlgorithmError::InvalidModel {
                    reason: "parameterized segment produced no points",
                })?
                .time_from_start_ns;
            if time_offset_ns > MAX_TRAJECTORY_DURATION_NS {
                return Err(ArmAlgorithmError::InvalidTime {
                    field: "trajectory.path_duration",
                });
            }
        }
        Self::validated(chain, points, None)
    }

    /// Blends two compatible trajectories with one clamped cubic seam.
    pub fn blend_sequence(
        chain: KinematicChain<'_>,
        first: &Self,
        second: &Self,
        blend_duration_ns: u64,
        blend_sampling: TrajectorySampling,
    ) -> Result<Self, ArmAlgorithmError> {
        first.ensure_context(chain)?;
        second.ensure_context(chain)?;
        let first_end = first.points.last().ok_or(ArmAlgorithmError::InvalidModel {
            reason: "first blend trajectory is empty",
        })?;
        let second_start = second
            .points
            .first()
            .ok_or(ArmAlgorithmError::InvalidModel {
                reason: "second blend trajectory is empty",
            })?;
        let blend_start = chain.positions_from_values(&first_end.positions_si)?;
        let blend_goal = chain.positions_from_values(&second_start.positions_si)?;
        let blend = Self::cubic_clamped(
            chain,
            &blend_start,
            &blend_goal,
            &first_end.velocities_si,
            &second_start.velocities_si,
            blend_duration_ns,
            blend_sampling,
        )?;
        let combined_count = first.points.len() - 1 + blend.points.len() + second.points.len() - 1;
        if combined_count > MAX_TRAJECTORY_POINTS {
            return Err(ArmAlgorithmError::Capacity {
                field: "trajectory.blended_points",
                actual: combined_count,
                maximum: MAX_TRAJECTORY_POINTS,
            });
        }
        let first_blend_offset = first.duration_ns().saturating_sub(1);
        let second_offset = first_blend_offset.checked_add(blend.duration_ns()).ok_or(
            ArmAlgorithmError::InvalidTime {
                field: "trajectory.blended_duration",
            },
        )?;
        let mut points = Vec::with_capacity(combined_count);
        points.extend(first.points.iter().take(first.points.len() - 1).cloned());
        for point in &blend.points {
            points.push(shifted_point(point, first_blend_offset)?);
        }
        for point in second.points.iter().skip(1) {
            points.push(shifted_point(point, second_offset.saturating_sub(1))?);
        }
        Self::validated(chain, points, None)
    }

    fn validated(
        chain: KinematicChain<'_>,
        points: Vec<ArmTrajectorySample>,
        jerk_limits: Option<&[f64]>,
    ) -> Result<Self, ArmAlgorithmError> {
        validate_points(chain, &points, jerk_limits)?;
        Ok(Self {
            profile_id: chain.model().profile_id(),
            group_id: chain.group().id(),
            order: chain.group().joint_order().clone(),
            points,
        })
    }

    fn ensure_context(&self, chain: KinematicChain<'_>) -> Result<(), ArmAlgorithmError> {
        if self.profile_id != chain.model().profile_id()
            || self.group_id != chain.group().id()
            || &self.order != chain.group().joint_order()
        {
            return Err(ArmAlgorithmError::JointOrderMismatch {
                group: chain.group().id(),
            });
        }
        Ok(())
    }

    /// Returns the immutable compiled profile identity.
    #[must_use]
    pub const fn profile_id(&self) -> ArmProfileId {
        self.profile_id
    }

    /// Returns the exact group identity.
    #[must_use]
    pub const fn group_id(&self) -> KinematicGroupId {
        self.group_id
    }

    /// Returns the exact stable joint order.
    #[must_use]
    pub const fn order(&self) -> &ArmJointOrder {
        &self.order
    }

    /// Returns validated points in strictly increasing time order.
    #[must_use]
    pub fn points(&self) -> &[ArmTrajectorySample] {
        &self.points
    }

    /// Returns total duration.
    #[must_use]
    pub fn duration_ns(&self) -> u64 {
        self.points
            .last()
            .map_or(0, ArmTrajectorySample::time_from_start_ns)
    }

    /// Interpolates into caller-owned fixed-capacity semantic storage.
    pub fn sample_into(
        &self,
        time_from_start_ns: u64,
        output: &mut TrajectorySampleBuffer,
    ) -> Result<(), ArmAlgorithmError> {
        if time_from_start_ns == 0 || time_from_start_ns > self.duration_ns() {
            return Err(ArmAlgorithmError::InvalidTime {
                field: "trajectory.sample_time",
            });
        }
        let upper = self
            .points
            .partition_point(|point| point.time_from_start_ns < time_from_start_ns);
        let (left, right, alpha) = if upper == 0 {
            (&self.points[0], &self.points[0], 0.0)
        } else if upper >= self.points.len() {
            let last = &self.points[self.points.len() - 1];
            (last, last, 0.0)
        } else {
            let left = &self.points[upper - 1];
            let right = &self.points[upper];
            let numerator = time_from_start_ns - left.time_from_start_ns;
            let denominator = right.time_from_start_ns - left.time_from_start_ns;
            (left, right, numerator as f64 / denominator as f64)
        };
        output.fill(&self.order, left, right, alpha)
    }

    /// Converts the validated algorithm trajectory to the protocol action goal.
    pub fn to_action_goal(&self) -> Result<ExecuteArmTrajectoryGoal, ArmAlgorithmError> {
        let mut semantic_points = Vec::with_capacity(self.points.len());
        for point in &self.points {
            let mut positions = ArrayVec::<ArmPosition, MAX_JOINTS>::new();
            let mut velocities = ArrayVec::<ArmVelocity, MAX_JOINTS>::new();
            let mut accelerations = ArrayVec::<ArmAcceleration, MAX_JOINTS>::new();
            for (index, descriptor) in self.order.joints().iter().enumerate() {
                positions.push(position_value(
                    descriptor.kind(),
                    point.positions_si[index],
                )?);
                velocities.push(velocity_value(
                    descriptor.kind(),
                    point.velocities_si[index],
                )?);
                accelerations.push(acceleration_value(
                    descriptor.kind(),
                    point.accelerations_si[index],
                )?);
            }
            semantic_points.push(
                SemanticTrajectoryPoint::new(
                    point.time_from_start_ns,
                    &self.order,
                    positions,
                    velocities,
                    accelerations,
                )
                .map_err(|_| ArmAlgorithmError::InvalidModel {
                    reason: "validated trajectory could not form semantic point",
                })?,
            );
        }
        ExecuteArmTrajectoryGoal::new(
            self.profile_id,
            self.group_id,
            self.order.clone(),
            semantic_points,
        )
        .map_err(|_| ArmAlgorithmError::InvalidModel {
            reason: "validated trajectory could not form action goal",
        })
    }
}

/// Caller-owned fixed-capacity trajectory sample.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct TrajectorySampleBuffer {
    positions: ArrayVec<ArmPosition, MAX_JOINTS>,
    velocities: ArrayVec<ArmVelocity, MAX_JOINTS>,
    accelerations: ArrayVec<ArmAcceleration, MAX_JOINTS>,
    jerks_si: ArrayVec<f64, MAX_JOINTS>,
}

impl TrajectorySampleBuffer {
    /// Creates empty reusable output storage.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            positions: ArrayVec::new_const(),
            velocities: ArrayVec::new_const(),
            accelerations: ArrayVec::new_const(),
            jerks_si: ArrayVec::new_const(),
        }
    }

    /// Returns sampled positions.
    #[must_use]
    pub fn positions(&self) -> &[ArmPosition] {
        &self.positions
    }

    /// Returns sampled velocities.
    #[must_use]
    pub fn velocities(&self) -> &[ArmVelocity] {
        &self.velocities
    }

    /// Returns sampled accelerations.
    #[must_use]
    pub fn accelerations(&self) -> &[ArmAcceleration] {
        &self.accelerations
    }

    /// Returns sampled jerk in the order-owned SI unit.
    #[must_use]
    pub fn jerks_si(&self) -> &[f64] {
        &self.jerks_si
    }

    fn fill(
        &mut self,
        order: &ArmJointOrder,
        left: &ArmTrajectorySample,
        right: &ArmTrajectorySample,
        alpha: f64,
    ) -> Result<(), ArmAlgorithmError> {
        self.positions.clear();
        self.velocities.clear();
        self.accelerations.clear();
        self.jerks_si.clear();
        for (index, descriptor) in order.joints().iter().enumerate() {
            self.positions.push(position_value(
                descriptor.kind(),
                interpolate(left.positions_si[index], right.positions_si[index], alpha),
            )?);
            self.velocities.push(velocity_value(
                descriptor.kind(),
                interpolate(left.velocities_si[index], right.velocities_si[index], alpha),
            )?);
            self.accelerations.push(acceleration_value(
                descriptor.kind(),
                interpolate(
                    left.accelerations_si[index],
                    right.accelerations_si[index],
                    alpha,
                ),
            )?);
            self.jerks_si.push(interpolate(
                left.jerks_si[index],
                right.jerks_si[index],
                alpha,
            ));
        }
        Ok(())
    }
}

#[derive(Clone, Copy)]
struct AxisLimits {
    max_velocity: f64,
    max_acceleration: f64,
    max_jerk: f64,
}

#[derive(Clone, Copy)]
struct TrapezoidAxis {
    start: f64,
    direction: f64,
    distance: f64,
    acceleration: f64,
    acceleration_time_s: f64,
    cruise_time_s: f64,
    duration_s: f64,
}

impl TrapezoidAxis {
    fn new(
        start: f64,
        goal: f64,
        maximum_velocity: f64,
        maximum_acceleration: f64,
    ) -> Result<Self, ArmAlgorithmError> {
        let delta = goal - start;
        if !delta.is_finite() {
            return Err(ArmAlgorithmError::NonFinite {
                field: "trajectory.delta",
            });
        }
        let distance = delta.abs();
        let direction = delta.signum();
        if distance == 0.0 {
            return Ok(Self {
                start,
                direction: 0.0,
                distance: 0.0,
                acceleration: maximum_acceleration,
                acceleration_time_s: 0.0,
                cruise_time_s: 0.0,
                duration_s: 0.0,
            });
        }
        let velocity_limited_acceleration_time = maximum_velocity / maximum_acceleration;
        let velocity_limited_distance =
            maximum_acceleration * velocity_limited_acceleration_time.powi(2);
        let (acceleration_time_s, cruise_time_s) = if distance <= velocity_limited_distance {
            ((distance / maximum_acceleration).sqrt(), 0.0)
        } else {
            (
                velocity_limited_acceleration_time,
                (distance - velocity_limited_distance) / maximum_velocity,
            )
        };
        Ok(Self {
            start,
            direction,
            distance,
            acceleration: maximum_acceleration,
            acceleration_time_s,
            cruise_time_s,
            duration_s: 2.0 * acceleration_time_s + cruise_time_s,
        })
    }

    fn sample_synchronized(self, global_time_s: f64, global_duration_s: f64) -> [f64; 3] {
        if self.distance == 0.0 || self.duration_s == 0.0 {
            return [self.start, 0.0, 0.0];
        }
        let scale = global_duration_s / self.duration_s;
        let local_time = (global_time_s / scale).clamp(0.0, self.duration_s);
        let acceleration_time = self.acceleration_time_s;
        let cruise_end = acceleration_time + self.cruise_time_s;
        let peak_velocity = self.acceleration * acceleration_time;
        let (distance, velocity, acceleration) = if local_time < acceleration_time {
            (
                0.5 * self.acceleration * local_time.powi(2),
                self.acceleration * local_time,
                self.acceleration,
            )
        } else if local_time < cruise_end {
            (
                0.5 * self.acceleration * acceleration_time.powi(2)
                    + peak_velocity * (local_time - acceleration_time),
                peak_velocity,
                0.0,
            )
        } else if local_time < self.duration_s {
            let remaining = self.duration_s - local_time;
            (
                self.distance - 0.5 * self.acceleration * remaining.powi(2),
                self.acceleration * remaining,
                -self.acceleration,
            )
        } else {
            (self.distance, 0.0, 0.0)
        };
        [
            self.direction.mul_add(distance, self.start),
            self.direction * velocity / scale,
            self.direction * acceleration / scale.powi(2),
        ]
    }
}

#[derive(Clone, Copy)]
struct CubicAxis {
    a0: f64,
    a1: f64,
    a2: f64,
    a3: f64,
}

impl CubicAxis {
    fn new(
        start: f64,
        goal: f64,
        start_velocity: f64,
        goal_velocity: f64,
        duration_s: f64,
    ) -> Self {
        let delta = goal - start;
        Self {
            a0: start,
            a1: start_velocity,
            a2: (3.0 * delta / duration_s.powi(2))
                - (2.0 * start_velocity + goal_velocity) / duration_s,
            a3: (-2.0 * delta / duration_s.powi(3))
                + (start_velocity + goal_velocity) / duration_s.powi(2),
        }
    }

    fn sample(self, time_s: f64) -> [f64; 4] {
        [
            self.a3.mul_add(
                time_s.powi(3),
                self.a2
                    .mul_add(time_s.powi(2), self.a1.mul_add(time_s, self.a0)),
            ),
            (3.0 * self.a3).mul_add(time_s.powi(2), (2.0 * self.a2).mul_add(time_s, self.a1)),
            (6.0 * self.a3).mul_add(time_s, 2.0 * self.a2),
            6.0 * self.a3,
        ]
    }
}

fn motion_limits(
    chain: KinematicChain<'_>,
    jerk_limits: Option<&[f64]>,
) -> Result<Vec<AxisLimits>, ArmAlgorithmError> {
    if let Some(jerk) = jerk_limits {
        validate_dimension(chain.dof(), jerk.len(), "trajectory.jerk_limits")?;
    }
    chain
        .group()
        .joint_order()
        .joints()
        .iter()
        .enumerate()
        .map(|(index, descriptor)| {
            let limits = chain
                .model()
                .joint(descriptor.joint_id())
                .and_then(|joint| joint.limits())
                .ok_or(ArmAlgorithmError::InvalidModel {
                    reason: "active group joint has no compiled limits",
                })?;
            let max_jerk = jerk_limits.map_or(f64::MAX, |values| values[index]);
            if !max_jerk.is_finite() || max_jerk <= 0.0 {
                return Err(ArmAlgorithmError::NonFinite {
                    field: "trajectory.jerk_limit",
                });
            }
            Ok(AxisLimits {
                max_velocity: limits.max_velocity(),
                max_acceleration: limits.max_acceleration(),
                max_jerk,
            })
        })
        .collect()
}

fn validate_points(
    chain: KinematicChain<'_>,
    points: &[ArmTrajectorySample],
    jerk_limits: Option<&[f64]>,
) -> Result<(), ArmAlgorithmError> {
    if points.len() < MIN_SAMPLE_COUNT || points.len() > MAX_TRAJECTORY_POINTS {
        return Err(ArmAlgorithmError::Capacity {
            field: "trajectory.points",
            actual: points.len(),
            maximum: MAX_TRAJECTORY_POINTS,
        });
    }
    let limits = motion_limits(chain, jerk_limits)?;
    let mut previous_time = 0;
    for (point_index, point) in points.iter().enumerate() {
        if point.time_from_start_ns <= previous_time
            || point.time_from_start_ns > MAX_TRAJECTORY_DURATION_NS
        {
            return Err(ArmAlgorithmError::InvalidTime {
                field: "trajectory.points",
            });
        }
        previous_time = point.time_from_start_ns;
        for (field, actual) in [
            ("trajectory.positions", point.positions_si.len()),
            ("trajectory.velocities", point.velocities_si.len()),
            ("trajectory.accelerations", point.accelerations_si.len()),
            ("trajectory.jerks", point.jerks_si.len()),
        ] {
            validate_dimension(chain.dof(), actual, field)?;
        }
        for (joint_index, descriptor) in chain.group().joint_order().joints().iter().enumerate() {
            let joint = chain.model().joint(descriptor.joint_id()).ok_or(
                ArmAlgorithmError::InvalidModel {
                    reason: "trajectory order references missing joint",
                },
            )?;
            let position = point.positions_si[joint_index];
            let velocity = point.velocities_si[joint_index];
            let acceleration = point.accelerations_si[joint_index];
            let jerk = point.jerks_si[joint_index];
            if [position, velocity, acceleration, jerk]
                .iter()
                .any(|value| !value.is_finite())
            {
                return Err(ArmAlgorithmError::NonFinite {
                    field: "trajectory.point",
                });
            }
            if let Some([lower, upper]) = joint.limits().and_then(|value| value.position())
                && !(lower..=upper).contains(&position)
            {
                let nearest = if position < lower { lower } else { upper };
                return Err(trajectory_violation(
                    TrajectoryViolationKind::Position,
                    point_index,
                    descriptor.joint_id(),
                    position,
                    nearest,
                ));
            }
            let axis_limits = limits[joint_index];
            for (kind, value, limit) in [
                (
                    TrajectoryViolationKind::Velocity,
                    velocity,
                    axis_limits.max_velocity,
                ),
                (
                    TrajectoryViolationKind::Acceleration,
                    acceleration,
                    axis_limits.max_acceleration,
                ),
                (TrajectoryViolationKind::Jerk, jerk, axis_limits.max_jerk),
            ] {
                if value.abs() > limit * (1.0 + 1.0e-10) {
                    return Err(trajectory_violation(
                        kind,
                        point_index,
                        descriptor.joint_id(),
                        value,
                        limit,
                    ));
                }
            }
        }
    }
    Ok(())
}

fn trajectory_violation(
    kind: TrajectoryViolationKind,
    point_index: usize,
    joint: message_model::ArmJointId,
    value: f64,
    limit: f64,
) -> ArmAlgorithmError {
    ArmAlgorithmError::TrajectoryViolation {
        kind,
        point_index,
        joint: Box::new(joint),
        value,
        limit,
    }
}

fn shifted_point(
    point: &ArmTrajectorySample,
    offset_ns: u64,
) -> Result<ArmTrajectorySample, ArmAlgorithmError> {
    let time_from_start_ns =
        point
            .time_from_start_ns
            .checked_add(offset_ns)
            .ok_or(ArmAlgorithmError::InvalidTime {
                field: "trajectory.shifted_time",
            })?;
    if time_from_start_ns > MAX_TRAJECTORY_DURATION_NS {
        return Err(ArmAlgorithmError::InvalidTime {
            field: "trajectory.shifted_time",
        });
    }
    Ok(ArmTrajectorySample {
        time_from_start_ns,
        positions_si: point.positions_si.clone(),
        velocities_si: point.velocities_si.clone(),
        accelerations_si: point.accelerations_si.clone(),
        jerks_si: point.jerks_si.clone(),
    })
}

fn validate_dimension(
    expected: usize,
    actual: usize,
    field: &'static str,
) -> Result<(), ArmAlgorithmError> {
    if expected == actual {
        Ok(())
    } else {
        Err(ArmAlgorithmError::DimensionMismatch {
            field,
            expected,
            actual,
        })
    }
}

fn bounded_duration_ns(duration_s: f64, sample_count: usize) -> Result<u64, ArmAlgorithmError> {
    if !duration_s.is_finite() || duration_s < 0.0 {
        return Err(ArmAlgorithmError::NonFinite {
            field: "trajectory.duration",
        });
    }
    let duration_ns = (duration_s * 1.0e9)
        .ceil()
        .max(MIN_TRAJECTORY_DURATION_NS.max(sample_count as u64) as f64);
    if duration_ns > MAX_TRAJECTORY_DURATION_NS as f64 {
        return Err(ArmAlgorithmError::InvalidTime {
            field: "trajectory.duration",
        });
    }
    Ok(duration_ns as u64)
}

fn normalized_sample(index: usize, sample_count: usize) -> f64 {
    index as f64 / (sample_count - 1) as f64
}

fn sample_time_ns(index: usize, sample_count: usize, duration_ns: u64) -> u64 {
    if index == 0 {
        1
    } else if index + 1 == sample_count {
        duration_ns
    } else {
        ((duration_ns as u128 * index as u128) / (sample_count - 1) as u128) as u64
    }
}

fn quintic_shape(s: f64) -> [f64; 4] {
    let s2 = s * s;
    let s3 = s2 * s;
    let s4 = s3 * s;
    let s5 = s4 * s;
    [
        10.0 * s3 - 15.0 * s4 + 6.0 * s5,
        30.0 * s2 - 60.0 * s3 + 30.0 * s4,
        60.0 * s - 180.0 * s2 + 120.0 * s3,
        60.0 - 360.0 * s + 360.0 * s2,
    ]
}

fn interpolate(left: f64, right: f64, alpha: f64) -> f64 {
    (right - left).mul_add(alpha, left)
}

fn position_value(kind: ArmJointKind, value: f64) -> Result<ArmPosition, ArmAlgorithmError> {
    match kind {
        ArmJointKind::Revolute | ArmJointKind::Continuous => ArmPosition::angular_rad(value),
        ArmJointKind::Prismatic => ArmPosition::linear_m(value),
    }
    .map_err(|_| ArmAlgorithmError::NonFinite {
        field: "trajectory.position",
    })
}

fn velocity_value(kind: ArmJointKind, value: f64) -> Result<ArmVelocity, ArmAlgorithmError> {
    match kind {
        ArmJointKind::Revolute | ArmJointKind::Continuous => ArmVelocity::angular_radps(value),
        ArmJointKind::Prismatic => ArmVelocity::linear_mps(value),
    }
    .map_err(|_| ArmAlgorithmError::NonFinite {
        field: "trajectory.velocity",
    })
}

fn acceleration_value(
    kind: ArmJointKind,
    value: f64,
) -> Result<ArmAcceleration, ArmAlgorithmError> {
    match kind {
        ArmJointKind::Revolute | ArmJointKind::Continuous => ArmAcceleration::angular_radps2(value),
        ArmJointKind::Prismatic => ArmAcceleration::linear_mps2(value),
    }
    .map_err(|_| ArmAlgorithmError::NonFinite {
        field: "trajectory.acceleration",
    })
}
