//! Generic omni-wheel forward and inverse kinematics.

use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, TAU};

use chassis_model::{
    BodyTwist, KinematicSolution, KinematicsError, PlanarLeastSquares, WheelAngularSpeeds,
    WheelPosition, WheelRadius, WheelSpeedLimit, desaturate_angular_speeds,
    require_effective_length, require_finite,
};

/// Three-wheel omni angular velocities in declaration order.
pub type Omni3WheelSpeeds = WheelAngularSpeeds<3>;
/// Four-wheel omni angular velocities in declaration order.
pub type Omni4WheelSpeeds = WheelAngularSpeeds<4>;

/// Validated arbitrary omni-wheel planar map.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OmniKinematics<const N: usize> {
    wheel_radius: WheelRadius,
    wheel_speed_limit: WheelSpeedLimit,
    positions: [WheelPosition; N],
    drive_angles_rad: [f64; N],
    rows: [[f64; 3]; N],
    forward_solver: PlanarLeastSquares<N>,
}

impl<const N: usize> OmniKinematics<N> {
    /// Constructs a map from body positions and positive drive-tangent angles.
    pub fn new(
        wheel_radius_m: f64,
        max_wheel_radps: f64,
        positions: [WheelPosition; N],
        drive_angles_rad: [f64; N],
    ) -> Result<Self, KinematicsError> {
        for angle in drive_angles_rad {
            require_finite(angle, "omni.drive_angle_rad")?;
        }
        let rows = std::array::from_fn(|index| {
            let [x, y] = positions[index].components();
            let cosine = drive_angles_rad[index].cos();
            let sine = drive_angles_rad[index].sin();
            [cosine, sine, -cosine * y + sine * x]
        });
        let forward_solver = PlanarLeastSquares::new(rows)?;
        Ok(Self {
            wheel_radius: WheelRadius::new(wheel_radius_m)?,
            wheel_speed_limit: WheelSpeedLimit::new(max_wheel_radps)?,
            positions,
            drive_angles_rad,
            rows,
            forward_solver,
        })
    }

    /// Converts a body target into uniformly constrained wheel speeds.
    pub fn inverse(
        &self,
        body: BodyTwist,
    ) -> Result<KinematicSolution<WheelAngularSpeeds<N>>, KinematicsError> {
        let components = body.components();
        let radius = self.wheel_radius.metres();
        let raw = self.rows.map(|row| {
            row[0].mul_add(
                components[0],
                row[1].mul_add(components[1], row[2] * components[2]),
            ) / radius
        });
        let speeds =
            WheelAngularSpeeds::new(raw).map_err(|_| KinematicsError::NumericalOverflow)?;
        Ok(desaturate_angular_speeds(speeds, self.wheel_speed_limit))
    }

    /// Recovers a least-squares body twist from wheel angular velocities.
    pub fn forward(&self, wheels: WheelAngularSpeeds<N>) -> Result<BodyTwist, KinematicsError> {
        let radius = self.wheel_radius.metres();
        let observations = wheels.into_values().map(|speed| speed * radius);
        self.forward_solver.solve(observations)
    }

    /// Returns module positions in declaration order.
    #[must_use]
    pub const fn positions(&self) -> &[WheelPosition; N] {
        &self.positions
    }

    /// Returns positive drive-tangent angles in declaration order.
    #[must_use]
    pub const fn drive_angles_rad(&self) -> &[f64; N] {
        &self.drive_angles_rad
    }
}

impl OmniKinematics<3> {
    /// Constructs a symmetric three-wheel layout, first module on body +x.
    pub fn symmetric(
        wheel_radius_m: f64,
        chassis_radius_m: f64,
        max_wheel_radps: f64,
    ) -> Result<Self, KinematicsError> {
        symmetric_layout::<3>(wheel_radius_m, chassis_radius_m, max_wheel_radps, 0.0)
    }
}

impl OmniKinematics<4> {
    /// Constructs a symmetric four-wheel layout at body-frame diagonals.
    pub fn symmetric(
        wheel_radius_m: f64,
        chassis_radius_m: f64,
        max_wheel_radps: f64,
    ) -> Result<Self, KinematicsError> {
        symmetric_layout::<4>(wheel_radius_m, chassis_radius_m, max_wheel_radps, FRAC_PI_4)
    }
}

fn symmetric_layout<const N: usize>(
    wheel_radius_m: f64,
    chassis_radius_m: f64,
    max_wheel_radps: f64,
    phase_rad: f64,
) -> Result<OmniKinematics<N>, KinematicsError> {
    let chassis_radius_m = require_effective_length(chassis_radius_m, "chassis_radius_m")?;
    let positions = std::array::from_fn(|index| {
        let position_angle = phase_rad + TAU * index as f64 / N as f64;
        WheelPosition::new(
            chassis_radius_m * position_angle.cos(),
            chassis_radius_m * position_angle.sin(),
        )
        .expect("validated radius and finite trigonometry")
    });
    let drive_angles_rad =
        std::array::from_fn(|index| phase_rad + TAU * index as f64 / N as f64 + FRAC_PI_2);
    OmniKinematics::new(wheel_radius_m, max_wheel_radps, positions, drive_angles_rad)
}

#[cfg(test)]
mod tests {
    use allocation_counter::measure;
    use chassis_model::{COMMON_BODY_FIXTURES, Feasibility, MIN_EFFECTIVE_LENGTH_M, body_twist};

    use super::*;

    const EPSILON: f64 = 1.0e-9;

    fn assert_twist_close(actual: BodyTwist, expected: BodyTwist) {
        for (actual, expected) in actual.components().into_iter().zip(expected.components()) {
            assert!((actual - expected).abs() < EPSILON);
        }
    }

    #[test]
    fn symmetric_omni_three_and_four_round_trip_common_fixtures() {
        let omni_three = OmniKinematics::<3>::symmetric(0.05, 0.3, 1_000.0).unwrap();
        let omni_four = OmniKinematics::<4>::symmetric(0.05, 0.3, 1_000.0).unwrap();
        for fixture in COMMON_BODY_FIXTURES {
            let expected = body_twist(fixture).unwrap();
            for actual in [
                omni_three
                    .forward(omni_three.inverse(expected).unwrap().into_value())
                    .unwrap(),
                omni_four
                    .forward(omni_four.inverse(expected).unwrap().into_value())
                    .unwrap(),
            ] {
                assert_twist_close(actual, expected);
            }
        }
    }

    #[test]
    fn saturation_is_uniform_and_reversal_changes_every_sign() {
        let model = OmniKinematics::<4>::symmetric(0.05, 0.3, 3.0).unwrap();
        let positive = model
            .inverse(body_twist([1.0, -0.4, 0.5]).unwrap())
            .unwrap();
        let negative = model
            .inverse(body_twist([-1.0, 0.4, -0.5]).unwrap())
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
            assert!(positive.abs() <= 3.0 + EPSILON);
        }
    }

    #[test]
    fn zero_input_produces_zero_wheels() {
        let model = OmniKinematics::<3>::symmetric(0.05, 0.3, 10.0).unwrap();
        let wheels = model
            .inverse(body_twist([0.0, 0.0, 0.0]).unwrap())
            .unwrap()
            .into_value();
        assert_eq!(wheels.values(), &[0.0; 3]);
    }

    #[test]
    fn singular_and_near_singular_layouts_are_rejected() {
        let origin = WheelPosition::new(0.0, 0.0).unwrap();
        assert_eq!(
            OmniKinematics::<3>::new(0.05, 10.0, [origin; 3], [0.0; 3]),
            Err(KinematicsError::SingularGeometry)
        );
        assert_eq!(
            OmniKinematics::<3>::symmetric(0.05, MIN_EFFECTIVE_LENGTH_M / 2.0, 10.0),
            Err(KinematicsError::NearSingularGeometry)
        );
        assert!(OmniKinematics::<4>::symmetric(f64::NAN, 0.3, 10.0).is_err());
    }

    #[test]
    fn forward_inverse_hot_paths_allocate_nothing() {
        let model = OmniKinematics::<4>::symmetric(0.05, 0.3, 1_000.0).unwrap();
        let target = body_twist([0.4, -0.3, 0.2]).unwrap();
        let allocations = measure(|| {
            let wheels = model.inverse(target).unwrap().into_value();
            let _ = model.forward(wheels).unwrap();
        });
        assert_eq!(allocations.count_total, 0);
    }
}
