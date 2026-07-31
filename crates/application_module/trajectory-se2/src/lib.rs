//! Fixed-capacity SE(2) trajectories and bounded chassis controllers.

use arrayvec::ArrayVec;
use chassis_model::{
    BodyCommandLimits, BodyPose2, BodyTwist, ConstrainedBodyCommand, KinematicsError, body_twist,
    constrain_body_command, require_finite,
};
use thiserror::Error;

/// Maximum points in one local SE(2) trajectory.
pub const MAX_SE2_TRAJECTORY_POINTS: usize = 256;

/// One strictly timed pose/feed-forward point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Se2TrajectoryPoint {
    /// Nonzero offset from trajectory acceptance.
    pub time_from_start_ns: u64,
    /// Desired world-frame pose.
    pub pose: BodyPose2,
    /// Desired body-frame velocity.
    pub feedforward: BodyTwist,
}

/// Validated immutable fixed-capacity trajectory.
#[derive(Clone, Debug, PartialEq)]
pub struct Se2Trajectory {
    points: ArrayVec<Se2TrajectoryPoint, MAX_SE2_TRAJECTORY_POINTS>,
}

impl Se2Trajectory {
    /// Copies and validates a nonempty, strictly ordered point slice.
    pub fn try_from_slice(points: &[Se2TrajectoryPoint]) -> Result<Self, TrajectoryError> {
        if points.is_empty() {
            return Err(TrajectoryError::Empty);
        }
        let mut stored = ArrayVec::new();
        let mut previous = 0;
        for point in points {
            if point.time_from_start_ns == 0 || point.time_from_start_ns <= previous {
                return Err(TrajectoryError::InvalidTimeOrder);
            }
            stored
                .try_push(*point)
                .map_err(|_| TrajectoryError::CapacityExceeded)?;
            previous = point.time_from_start_ns;
        }
        Ok(Self { points: stored })
    }

    /// Samples with shortest-yaw interpolation and deterministic completion.
    #[must_use]
    pub fn sample(&self, elapsed_ns: u64) -> TrajectorySample {
        if elapsed_ns >= self.points[self.points.len() - 1].time_from_start_ns {
            return TrajectorySample::Complete(self.points[self.points.len() - 1]);
        }
        if elapsed_ns <= self.points[0].time_from_start_ns {
            return TrajectorySample::Active {
                segment: 0,
                point: self.points[0],
            };
        }
        let upper = self
            .points
            .iter()
            .position(|point| point.time_from_start_ns >= elapsed_ns)
            .expect("elapsed is before final validated point");
        let lower = upper - 1;
        let start = self.points[lower];
        let end = self.points[upper];
        let span = end.time_from_start_ns - start.time_from_start_ns;
        let fraction = (elapsed_ns - start.time_from_start_ns) as f64 / span as f64;
        let start_twist = start.feedforward.components();
        let end_twist = end.feedforward.components();
        let feedforward = body_twist(std::array::from_fn(|index| {
            (end_twist[index] - start_twist[index]).mul_add(fraction, start_twist[index])
        }))
        .expect("finite interpolation of validated endpoints");
        TrajectorySample::Active {
            segment: lower,
            point: Se2TrajectoryPoint {
                time_from_start_ns: elapsed_ns,
                pose: start
                    .pose
                    .interpolate(end.pose, fraction)
                    .expect("fraction is in the closed unit interval"),
                feedforward,
            },
        }
    }

    /// Returns the validated ordered points.
    #[must_use]
    pub fn points(&self) -> &[Se2TrajectoryPoint] {
        &self.points
    }
}

/// Sampled trajectory state.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TrajectorySample {
    /// Active interpolated point and its lower segment index.
    Active {
        /// Zero-based lower segment.
        segment: usize,
        /// Desired state at the requested elapsed time.
        point: Se2TrajectoryPoint,
    },
    /// Final state at or after trajectory duration.
    Complete(Se2TrajectoryPoint),
}

/// Trajectory construction failure.
#[derive(Clone, Copy, Debug, Error, Eq, PartialEq)]
pub enum TrajectoryError {
    /// A trajectory contained no points.
    #[error("SE(2) trajectory cannot be empty")]
    Empty,
    /// Point storage exceeded its fixed maximum.
    #[error("SE(2) trajectory exceeds fixed capacity")]
    CapacityExceeded,
    /// Point offsets were zero, duplicate or decreasing.
    #[error("SE(2) trajectory times must be nonzero and strictly increasing")]
    InvalidTimeOrder,
}

/// Finite nonnegative proportional gains for x, y and yaw.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlanarGains([f64; 3]);

impl PlanarGains {
    /// Constructs nonnegative gains.
    pub fn new(x: f64, y: f64, yaw: f64) -> Result<Self, KinematicsError> {
        for (field, value) in [("gain.x", x), ("gain.y", y), ("gain.yaw", yaw)] {
            require_finite(value, field)?;
            if value < 0.0 {
                return Err(KinematicsError::OutsideRange { field });
            }
        }
        Ok(Self([x, y, yaw]))
    }
}

/// Stateful body-velocity controller with one shared limiter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BodyVelocityController {
    gains: PlanarGains,
    limits: BodyCommandLimits,
    previous_command: BodyTwist,
}

impl BodyVelocityController {
    /// Starts from an explicit prior command.
    #[must_use]
    pub const fn new(
        gains: PlanarGains,
        limits: BodyCommandLimits,
        previous_command: BodyTwist,
    ) -> Self {
        Self {
            gains,
            limits,
            previous_command,
        }
    }

    /// Applies proportional velocity error and configured command/rate limits.
    pub fn step(
        &mut self,
        target: BodyTwist,
        measured: BodyTwist,
        dt_s: f64,
    ) -> Result<ConstrainedBodyCommand, KinematicsError> {
        let target_components = target.components();
        let measured_components = measured.components();
        let desired = body_twist(std::array::from_fn(|index| {
            (target_components[index] - measured_components[index])
                .mul_add(self.gains.0[index], target_components[index])
        }))?;
        let output = constrain_body_command(desired, self.previous_command, self.limits, dt_s)?;
        self.previous_command = output.command();
        Ok(output)
    }
}

/// Stateful heading-hold controller retaining target translation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeadingController {
    yaw_gain: f64,
    limits: BodyCommandLimits,
    previous_command: BodyTwist,
}

impl HeadingController {
    /// Constructs a finite nonnegative yaw controller.
    pub fn new(
        yaw_gain: f64,
        limits: BodyCommandLimits,
        previous_command: BodyTwist,
    ) -> Result<Self, KinematicsError> {
        require_finite(yaw_gain, "heading.yaw_gain")?;
        if yaw_gain < 0.0 {
            return Err(KinematicsError::OutsideRange {
                field: "heading.yaw_gain",
            });
        }
        Ok(Self {
            yaw_gain,
            limits,
            previous_command,
        })
    }

    /// Applies shortest-angle heading error and command/rate limits.
    pub fn step(
        &mut self,
        translation_target: BodyTwist,
        measured_pose: BodyPose2,
        target_yaw_rad: f64,
        feedforward_yaw_rate: f64,
        dt_s: f64,
    ) -> Result<ConstrainedBodyCommand, KinematicsError> {
        let target_pose = BodyPose2::new(0.0, 0.0, target_yaw_rad)?;
        let yaw_error = measured_pose.body_error_to(target_pose)[2];
        require_finite(feedforward_yaw_rate, "heading.feedforward_yaw_rate")?;
        let [vx, vy, _] = translation_target.components();
        let desired = body_twist([
            vx,
            vy,
            yaw_error.mul_add(self.yaw_gain, feedforward_yaw_rate),
        ])?;
        let output = constrain_body_command(desired, self.previous_command, self.limits, dt_s)?;
        self.previous_command = output.command();
        Ok(output)
    }
}

/// Body-frame SE(2) pose-error tracker with velocity feedback.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Se2Tracker {
    pose_gains: PlanarGains,
    velocity_gains: PlanarGains,
    limits: BodyCommandLimits,
    previous_command: BodyTwist,
}

impl Se2Tracker {
    /// Constructs a tracker from validated gains, limits and prior output.
    #[must_use]
    pub const fn new(
        pose_gains: PlanarGains,
        velocity_gains: PlanarGains,
        limits: BodyCommandLimits,
        previous_command: BodyTwist,
    ) -> Self {
        Self {
            pose_gains,
            velocity_gains,
            limits,
            previous_command,
        }
    }

    /// Tracks a sampled point using body-frame pose and velocity error.
    pub fn step(
        &mut self,
        target: Se2TrajectoryPoint,
        measured_pose: BodyPose2,
        measured_velocity: BodyTwist,
        dt_s: f64,
    ) -> Result<ConstrainedBodyCommand, KinematicsError> {
        let pose_error = measured_pose.body_error_to(target.pose);
        let feedforward = target.feedforward.components();
        let measured = measured_velocity.components();
        let desired = body_twist(std::array::from_fn(|index| {
            self.pose_gains.0[index].mul_add(
                pose_error[index],
                self.velocity_gains.0[index]
                    .mul_add(feedforward[index] - measured[index], feedforward[index]),
            )
        }))?;
        let output = constrain_body_command(desired, self.previous_command, self.limits, dt_s)?;
        self.previous_command = output.command();
        Ok(output)
    }
}

#[cfg(test)]
mod tests {
    use allocation_counter::measure;
    use chassis_model::{BodyCommandLimits, BodyPose2, body_twist};

    use super::*;

    fn point(time: u64, x: f64, yaw: f64, vx: f64) -> Se2TrajectoryPoint {
        Se2TrajectoryPoint {
            time_from_start_ns: time,
            pose: BodyPose2::new(x, 0.0, yaw).unwrap(),
            feedforward: body_twist([vx, 0.0, 0.0]).unwrap(),
        }
    }

    fn limits() -> BodyCommandLimits {
        BodyCommandLimits::new(2.0, 1.0, 10.0, 10.0).unwrap()
    }

    #[test]
    fn trajectory_rejects_empty_ordering_and_capacity() {
        assert_eq!(
            Se2Trajectory::try_from_slice(&[]),
            Err(TrajectoryError::Empty)
        );
        assert_eq!(
            Se2Trajectory::try_from_slice(&[point(1, 0.0, 0.0, 0.0), point(1, 1.0, 0.0, 0.0)]),
            Err(TrajectoryError::InvalidTimeOrder)
        );
        let oversized = (1..=MAX_SE2_TRAJECTORY_POINTS + 1)
            .map(|index| point(index as u64, 0.0, 0.0, 0.0))
            .collect::<Vec<_>>();
        assert_eq!(
            Se2Trajectory::try_from_slice(&oversized),
            Err(TrajectoryError::CapacityExceeded)
        );
    }

    #[test]
    fn trajectory_interpolates_shortest_yaw_and_completes() {
        let trajectory = Se2Trajectory::try_from_slice(&[
            point(1_000, 0.0, 3.0, 0.0),
            point(2_000, 2.0, -3.0, 2.0),
        ])
        .unwrap();
        let TrajectorySample::Active { segment, point } = trajectory.sample(1_500) else {
            panic!("midpoint must be active");
        };
        assert_eq!(segment, 0);
        assert!((point.pose.components()[0] - 1.0).abs() < 1.0e-12);
        assert!(point.pose.components()[2].abs() > 3.0);
        assert_eq!(point.feedforward.components(), [1.0, 0.0, 0.0]);
        assert!(matches!(
            trajectory.sample(2_000),
            TrajectorySample::Complete(_)
        ));
    }

    #[test]
    fn velocity_and_heading_controllers_apply_feedback_and_limits() {
        let zero = body_twist([0.0, 0.0, 0.0]).unwrap();
        let mut velocity =
            BodyVelocityController::new(PlanarGains::new(1.0, 1.0, 1.0).unwrap(), limits(), zero);
        let output = velocity
            .step(body_twist([1.0, 0.0, 0.0]).unwrap(), zero, 0.1)
            .unwrap();
        assert!(output.command().components()[0] > 0.9);

        let mut heading = HeadingController::new(2.0, limits(), zero).unwrap();
        let output = heading
            .step(zero, BodyPose2::new(0.0, 0.0, 3.0).unwrap(), -3.0, 0.0, 0.1)
            .unwrap();
        assert!(output.command().components()[2] > 0.0);
    }

    #[test]
    fn tracker_rotates_world_error_into_body_frame() {
        let zero = body_twist([0.0, 0.0, 0.0]).unwrap();
        let mut tracker = Se2Tracker::new(
            PlanarGains::new(1.0, 1.0, 1.0).unwrap(),
            PlanarGains::new(0.0, 0.0, 0.0).unwrap(),
            limits(),
            zero,
        );
        let target = Se2TrajectoryPoint {
            time_from_start_ns: 1,
            pose: BodyPose2::new(0.0, 1.0, std::f64::consts::FRAC_PI_2).unwrap(),
            feedforward: zero,
        };
        let output = tracker
            .step(
                target,
                BodyPose2::new(0.0, 0.0, std::f64::consts::FRAC_PI_2).unwrap(),
                zero,
                0.1,
            )
            .unwrap();
        assert!(output.command().components()[0] > 0.9);
        assert!(output.command().components()[1].abs() < 1.0e-12);
    }

    #[test]
    fn controller_and_sampling_hot_paths_allocate_nothing() {
        let trajectory =
            Se2Trajectory::try_from_slice(&[point(1, 0.0, 0.0, 0.0), point(2, 1.0, 0.0, 1.0)])
                .unwrap();
        let zero = body_twist([0.0, 0.0, 0.0]).unwrap();
        let mut controller =
            BodyVelocityController::new(PlanarGains::new(1.0, 1.0, 1.0).unwrap(), limits(), zero);
        let allocations = measure(|| {
            let _ = trajectory.sample(1);
            let _ = controller
                .step(body_twist([0.5, 0.0, 0.0]).unwrap(), zero, 0.1)
                .unwrap();
        });
        assert_eq!(allocations.count_total, 0);
    }
}
