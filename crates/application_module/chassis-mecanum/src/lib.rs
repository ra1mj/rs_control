//! Mecanum-4 forward and inverse kinematics.

use chassis_model::{
    BodyTwist, KinematicSolution, KinematicsError, WheelAngularSpeeds, WheelRadius,
    WheelSpeedLimit, body_twist, desaturate_angular_speeds, require_effective_length,
};

/// Front-left wheel index.
pub const FRONT_LEFT: usize = 0;
/// Front-right wheel index.
pub const FRONT_RIGHT: usize = 1;
/// Rear-left wheel index.
pub const REAR_LEFT: usize = 2;
/// Rear-right wheel index.
pub const REAR_RIGHT: usize = 3;

/// Stable mecanum-4 wheel order: FL, FR, RL, RR.
pub type MecanumWheelSpeeds = WheelAngularSpeeds<4>;

/// Validated rectangular mecanum-4 map.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Mecanum4 {
    wheel_radius: WheelRadius,
    half_length_m: f64,
    half_width_m: f64,
    wheel_speed_limit: WheelSpeedLimit,
}

impl Mecanum4 {
    /// Constructs a conventional X-roller mecanum layout.
    pub fn new(
        wheel_radius_m: f64,
        half_length_m: f64,
        half_width_m: f64,
        max_wheel_radps: f64,
    ) -> Result<Self, KinematicsError> {
        Ok(Self {
            wheel_radius: WheelRadius::new(wheel_radius_m)?,
            half_length_m: require_effective_length(half_length_m, "half_length_m")?,
            half_width_m: require_effective_length(half_width_m, "half_width_m")?,
            wheel_speed_limit: WheelSpeedLimit::new(max_wheel_radps)?,
        })
    }

    /// Converts a body-frame target into uniformly constrained wheel speeds.
    pub fn inverse(
        &self,
        body: BodyTwist,
    ) -> Result<KinematicSolution<MecanumWheelSpeeds>, KinematicsError> {
        let [vx, vy, yaw_rate] = body.components();
        let lever = self.half_length_m + self.half_width_m;
        let radius = self.wheel_radius.metres();
        let raw = [
            (vx - vy - lever * yaw_rate) / radius,
            (vx + vy + lever * yaw_rate) / radius,
            (vx + vy - lever * yaw_rate) / radius,
            (vx - vy + lever * yaw_rate) / radius,
        ];
        let speeds =
            WheelAngularSpeeds::new(raw).map_err(|_| KinematicsError::NumericalOverflow)?;
        Ok(desaturate_angular_speeds(speeds, self.wheel_speed_limit))
    }

    /// Recovers the least-squares body twist from wheel angular velocities.
    pub fn forward(&self, wheels: MecanumWheelSpeeds) -> Result<BodyTwist, KinematicsError> {
        let values = wheels.values();
        let radius_quarter = self.wheel_radius.metres() * 0.25;
        let lever = self.half_length_m + self.half_width_m;
        body_twist([
            radius_quarter * (values[0] + values[1] + values[2] + values[3]),
            radius_quarter * (-values[0] + values[1] + values[2] - values[3]),
            radius_quarter * (-values[0] + values[1] - values[2] + values[3]) / lever,
        ])
    }

    /// Returns the validated geometry in metres.
    #[must_use]
    pub const fn dimensions_m(&self) -> [f64; 3] {
        [
            self.wheel_radius.metres(),
            self.half_length_m,
            self.half_width_m,
        ]
    }
}

#[cfg(test)]
mod tests {
    use allocation_counter::measure;
    use chassis_model::{COMMON_BODY_FIXTURES, Feasibility, MIN_EFFECTIVE_LENGTH_M, body_twist};

    use super::*;

    const EPSILON: f64 = 1.0e-10;

    fn model(limit: f64) -> Mecanum4 {
        Mecanum4::new(0.1, 0.3, 0.25, limit).unwrap()
    }

    fn assert_twist_close(actual: BodyTwist, expected: BodyTwist) {
        for (actual, expected) in actual.components().into_iter().zip(expected.components()) {
            assert!((actual - expected).abs() < EPSILON);
        }
    }

    #[test]
    fn common_body_fixtures_round_trip() {
        let model = model(1_000.0);
        for fixture in COMMON_BODY_FIXTURES {
            let expected = body_twist(fixture).unwrap();
            let inverse = model.inverse(expected).unwrap();
            assert_eq!(inverse.feasibility(), Feasibility::Feasible);
            assert_twist_close(model.forward(*inverse.value()).unwrap(), expected);
        }
    }

    #[test]
    fn wheel_order_and_signs_cover_pure_body_axes() {
        let model = model(1_000.0);
        let forward = model
            .inverse(body_twist([1.0, 0.0, 0.0]).unwrap())
            .unwrap()
            .into_value()
            .into_values();
        assert!(forward.iter().all(|speed| *speed > 0.0));

        let left = model
            .inverse(body_twist([0.0, 1.0, 0.0]).unwrap())
            .unwrap()
            .into_value()
            .into_values();
        assert!(left[FRONT_LEFT] < 0.0);
        assert!(left[FRONT_RIGHT] > 0.0);
        assert!(left[REAR_LEFT] > 0.0);
        assert!(left[REAR_RIGHT] < 0.0);

        let yaw = model
            .inverse(body_twist([0.0, 0.0, 1.0]).unwrap())
            .unwrap()
            .into_value()
            .into_values();
        assert!(yaw[FRONT_LEFT] < 0.0);
        assert!(yaw[FRONT_RIGHT] > 0.0);
        assert!(yaw[REAR_LEFT] < 0.0);
        assert!(yaw[REAR_RIGHT] > 0.0);
    }

    #[test]
    fn saturation_is_uniform_and_reversal_is_linear() {
        let model = model(2.0);
        let positive = model
            .inverse(body_twist([1.0, -0.5, 0.25]).unwrap())
            .unwrap();
        let negative = model
            .inverse(body_twist([-1.0, 0.5, -0.25]).unwrap())
            .unwrap();
        let Feasibility::Saturated(saturation) = positive.feasibility() else {
            panic!("fixture must saturate");
        };
        let scale = saturation.scale();
        assert!(scale > 0.0 && scale < 1.0);
        for (positive, negative) in positive
            .value()
            .values()
            .iter()
            .zip(negative.value().values())
        {
            assert!((*positive + *negative).abs() < EPSILON);
            assert!(positive.abs() <= 2.0 + EPSILON);
        }
    }

    #[test]
    fn invalid_and_near_singular_geometry_is_rejected() {
        assert!(matches!(
            Mecanum4::new(0.0, 0.3, 0.2, 10.0),
            Err(KinematicsError::NonPositive { .. })
        ));
        assert_eq!(
            Mecanum4::new(0.1, MIN_EFFECTIVE_LENGTH_M / 2.0, 0.2, 10.0),
            Err(KinematicsError::NearSingularGeometry)
        );
        assert!(Mecanum4::new(0.1, 0.3, 0.2, f64::NAN).is_err());
    }

    #[test]
    fn forward_inverse_hot_paths_allocate_nothing() {
        let model = model(1_000.0);
        let target = body_twist([0.4, -0.3, 0.2]).unwrap();
        let allocations = measure(|| {
            let wheels = model.inverse(target).unwrap().into_value();
            let _ = model.forward(wheels).unwrap();
        });
        assert_eq!(allocations.count_total, 0);
    }
}
