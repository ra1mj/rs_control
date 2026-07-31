//! Shared validated SI models and fixed-size planar kinematics utilities.

use thiserror::Error;

pub use message_model::Twist2 as BodyTwist;

/// Smallest geometry length accepted as numerically useful.
pub const MIN_EFFECTIVE_LENGTH_M: f64 = 1.0e-6;
/// Normalized Gram determinant below this value is considered near singular.
pub const NEAR_SINGULAR_DETERMINANT: f64 = 1.0e-8;
const SINGULAR_DETERMINANT: f64 = 1.0e-14;

/// Common body-frame fixtures in `[vx_mps, vy_mps, yaw_rate_radps]` order.
pub const COMMON_BODY_FIXTURES: [[f64; 3]; 8] = [
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [-0.75, 0.0, 0.0],
    [0.0, -0.5, 0.0],
    [0.4, -0.3, 0.2],
    [-0.2, 0.35, -0.6],
];

/// Typed failure at a chassis geometry or calculation boundary.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum KinematicsError {
    /// A caller supplied a non-finite numeric value.
    #[error("{field} must be finite")]
    NonFinite {
        /// Stable field identity.
        field: &'static str,
    },
    /// A dimension, limit or time step was not positive.
    #[error("{field} must be positive")]
    NonPositive {
        /// Stable field identity.
        field: &'static str,
    },
    /// Geometry cannot observe or command all three planar degrees of freedom.
    #[error("chassis geometry is singular")]
    SingularGeometry,
    /// Geometry is technically nonzero but too poorly conditioned to use.
    #[error("chassis geometry is near singular")]
    NearSingularGeometry,
    /// Finite inputs overflowed during the requested calculation.
    #[error("kinematics calculation exceeded finite numeric range")]
    NumericalOverflow,
}

/// Validated wheel/module position in the body frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelPosition {
    x_m: f64,
    y_m: f64,
}

impl WheelPosition {
    /// Constructs a finite body-frame position.
    pub fn new(x_m: f64, y_m: f64) -> Result<Self, KinematicsError> {
        require_finite(x_m, "wheel_position.x_m")?;
        require_finite(y_m, "wheel_position.y_m")?;
        Ok(Self { x_m, y_m })
    }

    /// Returns `[x_m, y_m]`.
    #[must_use]
    pub const fn components(self) -> [f64; 2] {
        [self.x_m, self.y_m]
    }
}

/// Validated effective wheel radius.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelRadius(f64);

impl WheelRadius {
    /// Constructs a positive, numerically useful wheel radius.
    pub fn new(radius_m: f64) -> Result<Self, KinematicsError> {
        let radius_m = require_effective_length(radius_m, "wheel_radius_m")?;
        Ok(Self(radius_m))
    }

    /// Returns the radius in metres.
    #[must_use]
    pub const fn metres(self) -> f64 {
        self.0
    }
}

/// Validated symmetric wheel angular-speed limit.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelSpeedLimit(f64);

impl WheelSpeedLimit {
    /// Constructs a finite positive magnitude limit.
    pub fn new(max_abs_radps: f64) -> Result<Self, KinematicsError> {
        require_positive(max_abs_radps, "max_wheel_radps")?;
        Ok(Self(max_abs_radps))
    }

    /// Returns the magnitude limit in radians per second.
    #[must_use]
    pub const fn radians_per_second(self) -> f64 {
        self.0
    }
}

/// Validated fixed-size wheel angular velocities.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct WheelAngularSpeeds<const N: usize>([f64; N]);

impl<const N: usize> WheelAngularSpeeds<N> {
    /// Constructs wheel speeds after rejecting non-finite values.
    pub fn new(values_radps: [f64; N]) -> Result<Self, KinematicsError> {
        for value in values_radps {
            require_finite(value, "wheel_speed_radps")?;
        }
        Ok(Self(values_radps))
    }

    /// Returns the wheel speeds in stable morphology order.
    #[must_use]
    pub const fn values(&self) -> &[f64; N] {
        &self.0
    }

    /// Returns the owned fixed-size array.
    #[must_use]
    pub const fn into_values(self) -> [f64; N] {
        self.0
    }
}

/// Whether a requested body velocity fit the configured wheel-speed envelope.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Feasibility {
    /// The request required no modification.
    Feasible,
    /// Every wheel speed was multiplied by the reported uniform scale.
    Saturated(Saturation),
}

/// Read-only uniform saturation evidence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Saturation {
    scale: f64,
}

impl Saturation {
    /// Returns the applied scale in `(0, 1)`.
    #[must_use]
    pub const fn scale(self) -> f64 {
        self.scale
    }
}

impl Feasibility {
    /// Returns the scale applied to the unconstrained wheel solution.
    #[must_use]
    pub const fn scale(self) -> f64 {
        match self {
            Self::Feasible => 1.0,
            Self::Saturated(saturation) => saturation.scale(),
        }
    }

    /// Returns whether uniform wheel-speed saturation was required.
    #[must_use]
    pub const fn is_saturated(self) -> bool {
        matches!(self, Self::Saturated(_))
    }
}

/// A calculated value plus explicit constraint evidence.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct KinematicSolution<T> {
    value: T,
    feasibility: Feasibility,
}

impl<T> KinematicSolution<T> {
    const fn new(value: T, feasibility: Feasibility) -> Self {
        Self { value, feasibility }
    }

    /// Borrows the calculated value.
    #[must_use]
    pub const fn value(&self) -> &T {
        &self.value
    }

    /// Returns constraint evidence.
    #[must_use]
    pub const fn feasibility(&self) -> Feasibility {
        self.feasibility
    }

    /// Consumes the solution and returns its value.
    #[must_use]
    pub fn into_value(self) -> T {
        self.value
    }
}

/// Uniformly scales validated wheel speeds into a symmetric limit.
#[must_use]
pub fn desaturate_angular_speeds<const N: usize>(
    speeds: WheelAngularSpeeds<N>,
    limit: WheelSpeedLimit,
) -> KinematicSolution<WheelAngularSpeeds<N>> {
    let maximum = speeds
        .values()
        .iter()
        .fold(0.0_f64, |current, value| current.max(value.abs()));
    if maximum <= limit.radians_per_second() {
        return KinematicSolution::new(speeds, Feasibility::Feasible);
    }

    let scale = limit.radians_per_second() / maximum;
    let values = speeds.into_values().map(|value| value * scale);
    KinematicSolution::new(
        WheelAngularSpeeds(values),
        Feasibility::Saturated(Saturation { scale }),
    )
}

/// Validated fixed-size least-squares map from observations to a body twist.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PlanarLeastSquares<const ROWS: usize> {
    normalized_rows: [[f64; 3]; ROWS],
    inverse_gram: [[f64; 3]; 3],
    column_scales: [f64; 3],
}

impl<const ROWS: usize> PlanarLeastSquares<ROWS> {
    /// Validates a design matrix and precomputes its normalized normal inverse.
    pub fn new(rows: [[f64; 3]; ROWS]) -> Result<Self, KinematicsError> {
        if ROWS < 3 {
            return Err(KinematicsError::SingularGeometry);
        }
        for row in rows {
            for value in row {
                require_finite(value, "kinematics_matrix")?;
            }
        }

        let mut column_scales = [0.0_f64; 3];
        for (column, scale) in column_scales.iter_mut().enumerate() {
            *scale = rows
                .iter()
                .map(|row| row[column] * row[column])
                .sum::<f64>()
                .sqrt();
            if *scale <= f64::EPSILON {
                return Err(KinematicsError::SingularGeometry);
            }
            if *scale < MIN_EFFECTIVE_LENGTH_M {
                return Err(KinematicsError::NearSingularGeometry);
            }
        }

        let normalized_rows =
            rows.map(|row| std::array::from_fn(|column| row[column] / column_scales[column]));
        let mut gram = [[0.0_f64; 3]; 3];
        for (left, gram_row) in gram.iter_mut().enumerate() {
            for (right, value) in gram_row.iter_mut().enumerate() {
                *value = normalized_rows.iter().fold(0.0, |sum, candidate| {
                    candidate[left].mul_add(candidate[right], sum)
                });
            }
        }
        let (inverse_gram, determinant) = inverse_3x3(gram)?;
        if determinant.abs() < NEAR_SINGULAR_DETERMINANT {
            return Err(KinematicsError::NearSingularGeometry);
        }

        Ok(Self {
            normalized_rows,
            inverse_gram,
            column_scales,
        })
    }

    /// Solves the validated least-squares system without allocating.
    pub fn solve(&self, observations: [f64; ROWS]) -> Result<BodyTwist, KinematicsError> {
        for value in observations {
            require_finite(value, "kinematics_observation")?;
        }

        let mut right_hand_side = [0.0_f64; 3];
        for (row, observation) in self.normalized_rows.iter().zip(observations) {
            for column in 0..3 {
                right_hand_side[column] = row[column].mul_add(observation, right_hand_side[column]);
            }
        }
        let normalized_solution = self.inverse_gram.map(|row| dot3(row, right_hand_side));
        let solution =
            std::array::from_fn(|index| normalized_solution[index] / self.column_scales[index]);
        body_twist(solution)
    }
}

/// Validates a finite positive scalar.
pub fn require_positive(value: f64, field: &'static str) -> Result<f64, KinematicsError> {
    require_finite(value, field)?;
    if value <= 0.0 {
        Err(KinematicsError::NonPositive { field })
    } else {
        Ok(value)
    }
}

/// Validates a positive geometry length with a numerical floor.
pub fn require_effective_length(value: f64, field: &'static str) -> Result<f64, KinematicsError> {
    require_positive(value, field)?;
    if value < MIN_EFFECTIVE_LENGTH_M {
        Err(KinematicsError::NearSingularGeometry)
    } else {
        Ok(value)
    }
}

/// Rejects a non-finite scalar.
pub fn require_finite(value: f64, field: &'static str) -> Result<f64, KinematicsError> {
    if value.is_finite() {
        Ok(value)
    } else {
        Err(KinematicsError::NonFinite { field })
    }
}

/// Converts finite components into the canonical body-frame type.
pub fn body_twist(components: [f64; 3]) -> Result<BodyTwist, KinematicsError> {
    if components.iter().any(|value| !value.is_finite()) {
        return Err(KinematicsError::NumericalOverflow);
    }
    BodyTwist::new(components[0], components[1], components[2])
        .map_err(|_| KinematicsError::NumericalOverflow)
}

fn dot3(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0].mul_add(right[0], left[1].mul_add(right[1], left[2] * right[2]))
}

fn inverse_3x3(matrix: [[f64; 3]; 3]) -> Result<([[f64; 3]; 3], f64), KinematicsError> {
    let determinant = matrix[0][0].mul_add(
        matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1],
        -matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
            + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]),
    );
    if !determinant.is_finite() {
        return Err(KinematicsError::NumericalOverflow);
    }
    if determinant.abs() <= SINGULAR_DETERMINANT {
        return Err(KinematicsError::SingularGeometry);
    }
    let reciprocal = determinant.recip();
    let inverse = [
        [
            (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1]) * reciprocal,
            (matrix[0][2] * matrix[2][1] - matrix[0][1] * matrix[2][2]) * reciprocal,
            (matrix[0][1] * matrix[1][2] - matrix[0][2] * matrix[1][1]) * reciprocal,
        ],
        [
            (matrix[1][2] * matrix[2][0] - matrix[1][0] * matrix[2][2]) * reciprocal,
            (matrix[0][0] * matrix[2][2] - matrix[0][2] * matrix[2][0]) * reciprocal,
            (matrix[0][2] * matrix[1][0] - matrix[0][0] * matrix[1][2]) * reciprocal,
        ],
        [
            (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]) * reciprocal,
            (matrix[0][1] * matrix[2][0] - matrix[0][0] * matrix[2][1]) * reciprocal,
            (matrix[0][0] * matrix[1][1] - matrix[0][1] * matrix[1][0]) * reciprocal,
        ],
    ];
    Ok((inverse, determinant))
}

#[cfg(test)]
mod tests {
    use allocation_counter::measure;

    use super::*;

    const EPSILON: f64 = 1.0e-10;

    #[test]
    fn shared_si_types_reject_invalid_values() {
        assert_eq!(
            WheelPosition::new(f64::NAN, 0.0),
            Err(KinematicsError::NonFinite {
                field: "wheel_position.x_m"
            })
        );
        assert_eq!(
            WheelRadius::new(0.0),
            Err(KinematicsError::NonPositive {
                field: "wheel_radius_m"
            })
        );
        assert_eq!(
            WheelRadius::new(MIN_EFFECTIVE_LENGTH_M / 2.0),
            Err(KinematicsError::NearSingularGeometry)
        );
        assert!(WheelSpeedLimit::new(f64::INFINITY).is_err());
        assert!(WheelAngularSpeeds::<4>::new([0.0, f64::NAN, 0.0, 0.0]).is_err());
    }

    #[test]
    fn common_fixtures_are_finite_body_twists() {
        for fixture in COMMON_BODY_FIXTURES {
            assert_eq!(body_twist(fixture).unwrap().components(), fixture);
        }
    }

    #[test]
    fn desaturation_is_uniform_and_explicit() {
        let result = desaturate_angular_speeds(
            WheelAngularSpeeds::new([2.0, -4.0, 1.0, 0.0]).unwrap(),
            WheelSpeedLimit::new(2.0).unwrap(),
        );
        assert_eq!(
            result.feasibility(),
            Feasibility::Saturated(Saturation { scale: 0.5 })
        );
        assert_eq!(result.value().values(), &[1.0, -2.0, 0.5, 0.0]);

        let exact = desaturate_angular_speeds(
            WheelAngularSpeeds::new([1.0, -2.0]).unwrap(),
            WheelSpeedLimit::new(2.0).unwrap(),
        );
        assert_eq!(exact.feasibility(), Feasibility::Feasible);
    }

    #[test]
    fn least_squares_rejects_singular_and_near_singular_geometry() {
        assert_eq!(
            PlanarLeastSquares::new([[1.0, 0.0, 0.0]; 3]),
            Err(KinematicsError::SingularGeometry)
        );
        assert_eq!(
            PlanarLeastSquares::new([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0e-8]]),
            Err(KinematicsError::NearSingularGeometry)
        );
    }

    #[test]
    fn least_squares_solves_overdetermined_body_system() {
        let solver = PlanarLeastSquares::new([
            [1.0, 0.0, -0.2],
            [0.0, 1.0, 0.3],
            [1.0, 0.0, 0.2],
            [0.0, 1.0, -0.3],
        ])
        .unwrap();
        let expected = [0.4, -0.2, 0.5];
        let observations = [
            expected[0] - 0.2 * expected[2],
            expected[1] + 0.3 * expected[2],
            expected[0] + 0.2 * expected[2],
            expected[1] - 0.3 * expected[2],
        ];
        let actual = solver.solve(observations).unwrap().components();
        for (actual, expected) in actual.into_iter().zip(expected) {
            assert!((actual - expected).abs() < EPSILON);
        }
    }

    #[test]
    fn shared_hot_paths_allocate_nothing() {
        let solver =
            PlanarLeastSquares::new([[1.0, 0.0, 0.1], [0.0, 1.0, 0.1], [-1.0, -1.0, 0.1]]).unwrap();
        let speeds = WheelAngularSpeeds::new([1.0, 2.0, 3.0]).unwrap();
        let limit = WheelSpeedLimit::new(2.0).unwrap();
        let allocations = measure(|| {
            let _ = solver.solve([1.0, 2.0, 3.0]).unwrap();
            let _ = desaturate_angular_speeds(speeds, limit);
        });
        assert_eq!(allocations.count_total, 0);
    }
}
