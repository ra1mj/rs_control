//! Swerve-4 forward/inverse kinematics and steering optimization.

use std::f64::consts::{FRAC_PI_2, PI, TAU};

use chassis_model::{
    BodyTwist, Feasibility, KinematicsError, PlanarLeastSquares, WheelAngularSpeeds, WheelPosition,
    WheelRadius, WheelSpeedLimit, desaturate_angular_speeds, require_effective_length,
    require_finite, require_positive,
};

/// Front-left module index.
pub const FRONT_LEFT: usize = 0;
/// Front-right module index.
pub const FRONT_RIGHT: usize = 1;
/// Rear-left module index.
pub const REAR_LEFT: usize = 2;
/// Rear-right module index.
pub const REAR_RIGHT: usize = 3;

const ZERO_SPEED_EPSILON_MPS: f64 = 1.0e-12;
const ANGLE_EPSILON_RAD: f64 = 1.0e-12;

/// One validated swerve module command/observation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SwerveModuleState {
    drive_radps: f64,
    steering_angle_rad: f64,
}

impl SwerveModuleState {
    /// Constructs a finite state and normalizes its angle to `[-pi, pi)`.
    pub fn new(drive_radps: f64, steering_angle_rad: f64) -> Result<Self, KinematicsError> {
        require_finite(drive_radps, "swerve.drive_radps")?;
        require_finite(steering_angle_rad, "swerve.steering_angle_rad")?;
        Ok(Self {
            drive_radps,
            steering_angle_rad: normalize_angle(steering_angle_rad),
        })
    }

    /// Returns signed drive angular velocity.
    #[must_use]
    pub const fn drive_radps(self) -> f64 {
        self.drive_radps
    }

    /// Returns normalized steering angle.
    #[must_use]
    pub const fn steering_angle_rad(self) -> f64 {
        self.steering_angle_rad
    }
}

/// Constrained swerve inverse result and achieved-body evidence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SwerveInverseSolution {
    modules: [SwerveModuleState; 4],
    achieved_body: BodyTwist,
    drive_feasibility: Feasibility,
    steering_limited: bool,
}

impl SwerveInverseSolution {
    /// Returns module states in FL, FR, RL, RR order.
    #[must_use]
    pub const fn modules(&self) -> &[SwerveModuleState; 4] {
        &self.modules
    }

    /// Returns the least-squares body motion produced by reachable modules.
    #[must_use]
    pub const fn achieved_body(&self) -> BodyTwist {
        self.achieved_body
    }

    /// Returns uniform drive saturation evidence.
    #[must_use]
    pub const fn drive_feasibility(&self) -> Feasibility {
        self.drive_feasibility
    }

    /// Returns whether at least one module hit its steering-rate bound.
    #[must_use]
    pub const fn steering_limited(&self) -> bool {
        self.steering_limited
    }
}

/// Validated four-module swerve map.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Swerve4 {
    wheel_radius: WheelRadius,
    drive_speed_limit: WheelSpeedLimit,
    max_steer_rate_radps: f64,
    positions: [WheelPosition; 4],
    forward_solver: PlanarLeastSquares<8>,
}

impl Swerve4 {
    /// Constructs a custom four-module map.
    pub fn new(
        wheel_radius_m: f64,
        max_drive_radps: f64,
        max_steer_rate_radps: f64,
        positions: [WheelPosition; 4],
    ) -> Result<Self, KinematicsError> {
        let rows = std::array::from_fn(|row| {
            let [x, y] = positions[row / 2].components();
            if row % 2 == 0 {
                [1.0, 0.0, -y]
            } else {
                [0.0, 1.0, x]
            }
        });
        Ok(Self {
            wheel_radius: WheelRadius::new(wheel_radius_m)?,
            drive_speed_limit: WheelSpeedLimit::new(max_drive_radps)?,
            max_steer_rate_radps: require_positive(max_steer_rate_radps, "max_steer_rate_radps")?,
            positions,
            forward_solver: PlanarLeastSquares::new(rows)?,
        })
    }

    /// Constructs a rectangular FL, FR, RL, RR layout.
    pub fn rectangular(
        wheel_radius_m: f64,
        half_length_m: f64,
        half_width_m: f64,
        max_drive_radps: f64,
        max_steer_rate_radps: f64,
    ) -> Result<Self, KinematicsError> {
        let half_length_m = require_effective_length(half_length_m, "half_length_m")?;
        let half_width_m = require_effective_length(half_width_m, "half_width_m")?;
        let positions = [
            WheelPosition::new(half_length_m, half_width_m).expect("validated finite dimensions"),
            WheelPosition::new(half_length_m, -half_width_m).expect("validated finite dimensions"),
            WheelPosition::new(-half_length_m, half_width_m).expect("validated finite dimensions"),
            WheelPosition::new(-half_length_m, -half_width_m).expect("validated finite dimensions"),
        ];
        Self::new(
            wheel_radius_m,
            max_drive_radps,
            max_steer_rate_radps,
            positions,
        )
    }

    /// Converts a body target into reachable module states for one explicit step.
    pub fn inverse(
        &self,
        body: BodyTwist,
        previous: [SwerveModuleState; 4],
        dt_s: f64,
    ) -> Result<SwerveInverseSolution, KinematicsError> {
        let dt_s = require_positive(dt_s, "swerve.dt_s")?;
        let maximum_angle_step = self.max_steer_rate_radps * dt_s;
        if !maximum_angle_step.is_finite() {
            return Err(KinematicsError::NumericalOverflow);
        }

        let [vx, vy, yaw_rate] = body.components();
        let mut raw_drive_radps = [0.0; 4];
        let mut steering_angles = [0.0; 4];
        let mut steering_limited = false;
        for index in 0..4 {
            let [x, y] = self.positions[index].components();
            let desired_x = vx - yaw_rate * y;
            let desired_y = vy + yaw_rate * x;
            if !desired_x.is_finite() || !desired_y.is_finite() {
                return Err(KinematicsError::NumericalOverflow);
            }
            let magnitude = desired_x.hypot(desired_y);
            if magnitude <= ZERO_SPEED_EPSILON_MPS {
                steering_angles[index] = previous[index].steering_angle_rad();
                raw_drive_radps[index] = 0.0;
                continue;
            }

            let raw_angle = desired_y.atan2(desired_x);
            let current_angle = previous[index].steering_angle_rad();
            let mut target_angle = raw_angle;
            let mut delta = shortest_angular_distance(current_angle, target_angle);
            if delta.abs() > FRAC_PI_2 {
                target_angle = normalize_angle(target_angle + PI);
                delta = shortest_angular_distance(current_angle, target_angle);
            }
            let limited_delta = delta.clamp(-maximum_angle_step, maximum_angle_step);
            steering_limited |= (delta - limited_delta).abs() > ANGLE_EPSILON_RAD;
            let reachable_angle = normalize_angle(current_angle + limited_delta);
            steering_angles[index] = reachable_angle;

            let reachable_linear_speed =
                desired_x.mul_add(reachable_angle.cos(), desired_y * reachable_angle.sin());
            raw_drive_radps[index] = reachable_linear_speed / self.wheel_radius.metres();
        }

        let unconstrained = WheelAngularSpeeds::new(raw_drive_radps)
            .map_err(|_| KinematicsError::NumericalOverflow)?;
        let constrained = desaturate_angular_speeds(unconstrained, self.drive_speed_limit);
        let drive_feasibility = constrained.feasibility();
        let drive_values = constrained.into_value().into_values();
        let modules = std::array::from_fn(|index| SwerveModuleState {
            drive_radps: drive_values[index],
            steering_angle_rad: steering_angles[index],
        });
        let achieved_body = self.forward(modules)?;
        Ok(SwerveInverseSolution {
            modules,
            achieved_body,
            drive_feasibility,
            steering_limited,
        })
    }

    /// Recovers the least-squares body twist from four module vectors.
    pub fn forward(&self, modules: [SwerveModuleState; 4]) -> Result<BodyTwist, KinematicsError> {
        let radius = self.wheel_radius.metres();
        let mut observations = [0.0; 8];
        for (index, module) in modules.into_iter().enumerate() {
            let linear_speed = module.drive_radps() * radius;
            observations[index * 2] = linear_speed * module.steering_angle_rad().cos();
            observations[index * 2 + 1] = linear_speed * module.steering_angle_rad().sin();
        }
        self.forward_solver.solve(observations)
    }

    /// Returns module positions in FL, FR, RL, RR order.
    #[must_use]
    pub const fn positions(&self) -> &[WheelPosition; 4] {
        &self.positions
    }
}

/// Normalizes an angle to `[-pi, pi)`.
#[must_use]
fn normalize_angle(angle_rad: f64) -> f64 {
    (angle_rad + PI).rem_euclid(TAU) - PI
}

/// Returns the signed shortest rotation from `from` to `to`.
#[must_use]
fn shortest_angular_distance(from_rad: f64, to_rad: f64) -> f64 {
    normalize_angle(to_rad - from_rad)
}

#[cfg(test)]
mod tests {
    use allocation_counter::measure;
    use chassis_model::{COMMON_BODY_FIXTURES, MIN_EFFECTIVE_LENGTH_M, body_twist};

    use super::*;

    const EPSILON: f64 = 1.0e-9;

    fn model(max_drive: f64, max_steer_rate: f64) -> Swerve4 {
        Swerve4::rectangular(0.05, 0.3, 0.25, max_drive, max_steer_rate).unwrap()
    }

    fn module(drive_radps: f64, steering_angle_rad: f64) -> SwerveModuleState {
        SwerveModuleState::new(drive_radps, steering_angle_rad).unwrap()
    }

    fn assert_twist_close(actual: BodyTwist, expected: BodyTwist) {
        for (actual, expected) in actual.components().into_iter().zip(expected.components()) {
            assert!((actual - expected).abs() < EPSILON);
        }
    }

    fn aligned_previous(model: &Swerve4, body: BodyTwist) -> [SwerveModuleState; 4] {
        let [vx, vy, yaw_rate] = body.components();
        std::array::from_fn(|index| {
            let [x, y] = model.positions()[index].components();
            let module_x = vx - yaw_rate * y;
            let module_y = vy + yaw_rate * x;
            let angle = if module_x.hypot(module_y) <= ZERO_SPEED_EPSILON_MPS {
                0.0
            } else {
                module_y.atan2(module_x)
            };
            module(0.0, angle)
        })
    }

    #[test]
    fn common_body_fixtures_round_trip_when_steering_is_reachable() {
        let model = model(1_000.0, 100.0);
        for fixture in COMMON_BODY_FIXTURES {
            let expected = body_twist(fixture).unwrap();
            let solution = model
                .inverse(expected, aligned_previous(&model, expected), 0.1)
                .unwrap();
            assert!(!solution.steering_limited());
            assert_eq!(solution.drive_feasibility(), Feasibility::Feasible);
            assert_twist_close(solution.achieved_body(), expected);
            assert_twist_close(model.forward(*solution.modules()).unwrap(), expected);
        }
    }

    #[test]
    fn reversal_selects_equivalent_target_within_ninety_degrees() {
        let model = model(100.0, 100.0);
        let previous = [module(0.0, 0.0); 4];
        let solution = model
            .inverse(body_twist([-1.0, 0.0, 0.0]).unwrap(), previous, 0.1)
            .unwrap();
        for module in solution.modules() {
            assert!(module.steering_angle_rad().abs() < EPSILON);
            assert!(module.drive_radps() < 0.0);
        }
        assert_twist_close(
            solution.achieved_body(),
            body_twist([-1.0, 0.0, 0.0]).unwrap(),
        );
    }

    #[test]
    fn steering_rate_limit_reports_degraded_achieved_motion() {
        let model = model(100.0, 1.0);
        let previous = [module(0.0, 0.0); 4];
        let solution = model
            .inverse(body_twist([0.0, 1.0, 0.0]).unwrap(), previous, 0.1)
            .unwrap();
        assert!(solution.steering_limited());
        for module in solution.modules() {
            assert!((module.steering_angle_rad() - 0.1).abs() < EPSILON);
        }
        assert!(solution.achieved_body().components()[1] < 1.0);
    }

    #[test]
    fn zero_speed_holds_prior_angles_exactly() {
        let model = model(100.0, 1.0);
        let previous = [
            module(0.0, 0.2),
            module(0.0, -0.4),
            module(0.0, 1.2),
            module(0.0, -2.0),
        ];
        let solution = model
            .inverse(body_twist([0.0, 0.0, 0.0]).unwrap(), previous, 0.01)
            .unwrap();
        assert!(!solution.steering_limited());
        for (actual, previous) in solution.modules().iter().zip(previous) {
            assert_eq!(actual.steering_angle_rad(), previous.steering_angle_rad());
            assert_eq!(actual.drive_radps(), 0.0);
        }
    }

    #[test]
    fn drive_saturation_is_uniform_and_explicit() {
        let model = model(2.0, 100.0);
        let target = body_twist([1.0, 0.5, 0.4]).unwrap();
        let solution = model
            .inverse(target, aligned_previous(&model, target), 0.1)
            .unwrap();
        let Feasibility::Saturated(saturation) = solution.drive_feasibility() else {
            panic!("fixture must saturate");
        };
        let scale = saturation.scale();
        assert!(scale > 0.0 && scale < 1.0);
        assert!(
            solution
                .modules()
                .iter()
                .all(|module| module.drive_radps().abs() <= 2.0 + EPSILON)
        );
        let expected = target.components().map(|component| component * scale);
        assert_twist_close(solution.achieved_body(), body_twist(expected).unwrap());
    }

    #[test]
    fn singular_near_singular_and_invalid_updates_are_rejected() {
        let origin = WheelPosition::new(0.0, 0.0).unwrap();
        assert_eq!(
            Swerve4::new(0.05, 10.0, 10.0, [origin; 4]),
            Err(KinematicsError::SingularGeometry)
        );
        let tiny = MIN_EFFECTIVE_LENGTH_M / 10.0;
        let positions = [
            WheelPosition::new(tiny, tiny).unwrap(),
            WheelPosition::new(tiny, -tiny).unwrap(),
            WheelPosition::new(-tiny, tiny).unwrap(),
            WheelPosition::new(-tiny, -tiny).unwrap(),
        ];
        assert_eq!(
            Swerve4::new(0.05, 10.0, 10.0, positions),
            Err(KinematicsError::NearSingularGeometry)
        );
        assert!(Swerve4::rectangular(0.05, 0.3, 0.2, 0.0, 10.0).is_err());
        assert!(
            model(10.0, 10.0)
                .inverse(
                    body_twist([0.0, 0.0, 0.0]).unwrap(),
                    [module(0.0, 0.0); 4],
                    0.0
                )
                .is_err()
        );
    }

    #[test]
    fn forward_inverse_hot_paths_allocate_nothing() {
        let model = model(1_000.0, 100.0);
        let target = body_twist([0.4, -0.3, 0.2]).unwrap();
        let previous = aligned_previous(&model, target);
        let allocations = measure(|| {
            let solution = model.inverse(target, previous, 0.1).unwrap();
            let _ = model.forward(*solution.modules()).unwrap();
        });
        assert_eq!(allocations.count_total, 0);
    }
}
