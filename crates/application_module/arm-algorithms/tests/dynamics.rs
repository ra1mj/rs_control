mod common;

use arm_algorithms::{
    ArmAlgorithmError,
    dynamics::{ArmDynamics, AttachedPayloadInertia, PreparedGravity},
    kinematics::KinematicChain,
    trajectory::{ArmTrajectory, TrajectorySampling},
};
use arm_model::{
    ARM_PROFILE_SCHEMA_VERSION, ArmModel, ArmProfileClass, ArmProfileSpec,
    JointAccelerationLimitSpec, KinematicGroupSpec,
};
use arrayvec::ArrayVec;
use message_model::{
    ArmAcceleration, ArmEffort, ArmJointKind, ArmJointOrder, ArmJointVector, ArmVelocity,
    MAX_JOINTS,
};

use common::{id, mixed_model, synthetic_model};

const PENDULUM: &str = r#"
<robot name="pendulum">
  <link name="base">
    <inertial><mass value="1"/><inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/></inertial>
  </link>
  <link name="link">
    <inertial>
      <origin xyz="0.5 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>
  <joint name="joint" type="revolute">
    <parent link="base"/><child link="link"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" velocity="4" effort="20"/>
  </joint>
</robot>
"#;

fn pendulum_model() -> ArmModel {
    ArmModel::compile_urdf(
        PENDULUM,
        &ArmProfileSpec {
            schema_version: ARM_PROFILE_SCHEMA_VERSION,
            name: id("pendulum"),
            class: ArmProfileClass::HostFixture,
            groups: vec![KinematicGroupSpec {
                id: id("arm"),
                base_link: id("base"),
                tip_link: id("link"),
                active_joint_order: vec![id("joint")],
            }],
            acceleration_limits: vec![JointAccelerationLimitSpec {
                joint_id: id("joint"),
                maximum: 10.0,
            }],
            transmissions: Vec::new(),
        },
    )
    .unwrap()
}

fn velocities(order: &ArmJointOrder, values: &[f64]) -> ArmJointVector<ArmVelocity> {
    let mut output = ArrayVec::<ArmVelocity, MAX_JOINTS>::new();
    for (descriptor, value) in order.joints().iter().zip(values) {
        output.push(
            match descriptor.kind() {
                ArmJointKind::Revolute | ArmJointKind::Continuous => {
                    ArmVelocity::angular_radps(*value)
                }
                ArmJointKind::Prismatic => ArmVelocity::linear_mps(*value),
            }
            .unwrap(),
        );
    }
    ArmJointVector::new(order.clone(), output).unwrap()
}

fn accelerations(order: &ArmJointOrder, values: &[f64]) -> ArmJointVector<ArmAcceleration> {
    let mut output = ArrayVec::<ArmAcceleration, MAX_JOINTS>::new();
    for (descriptor, value) in order.joints().iter().zip(values) {
        output.push(
            match descriptor.kind() {
                ArmJointKind::Revolute | ArmJointKind::Continuous => {
                    ArmAcceleration::angular_radps2(*value)
                }
                ArmJointKind::Prismatic => ArmAcceleration::linear_mps2(*value),
            }
            .unwrap(),
        );
    }
    ArmJointVector::new(order.clone(), output).unwrap()
}

fn assert_near(actual: f64, expected: f64, tolerance: f64) {
    assert!(
        (actual - expected).abs() <= tolerance,
        "{actual} is not within {tolerance} of {expected}"
    );
}

#[test]
fn pendulum_gravity_matches_analytic_golden() {
    let model = pendulum_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let positions = chain.positions_from_values(&[0.0]).unwrap();
    let mut dynamics = ArmDynamics::new(chain, [0.0, -9.81, 0.0]).unwrap();
    let gravity = dynamics.gravity_compensation(&positions).unwrap();
    assert_near(gravity.values()[0].value_si(), 4.905, 1.0e-3);
    let prepared = PreparedGravity::new(chain, [0.0, -9.81, 0.0], 1).unwrap();
    let mut workspace = prepared.workspace();
    let fast = prepared
        .gravity_compensation(&positions, &mut workspace)
        .unwrap();
    assert_near(fast.values()[0].value_si(), 4.905, 1.0e-12);

    let payload =
        AttachedPayloadInertia::new(2.0, [0.25, 0.0, 0.0], [0.01, 0.0, 0.0, 0.01, 0.0, 0.01])
            .unwrap();
    let mut payload_dynamics =
        ArmDynamics::new_with_payload(chain, [0.0, -9.81, 0.0], payload).unwrap();
    let payload_total = payload_dynamics.gravity_compensation(&positions).unwrap();
    let predicted = prepared
        .payload_gravity(&positions, [2.0, 0.5, 0.0, 0.0], &mut workspace)
        .unwrap();
    assert_near(
        payload_total.values()[0].value_si() - gravity.values()[0].value_si(),
        predicted.values()[0].value_si(),
        2.0e-3,
    );
    let mass = dynamics.mass_matrix(&positions).unwrap();
    assert_eq!(mass.dof(), 1);
    assert!(mass.get(0, 0).unwrap() > 0.0);
}

#[test]
fn rnea_aba_round_trip_and_mass_matrix_properties_hold() {
    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let order = chain.group().joint_order();
    let q = chain
        .positions_from_values(&[0.2, -0.3, 0.1, 0.15, -0.2, 0.25])
        .unwrap();
    let qd = velocities(order, &[0.1, -0.2, 0.15, 0.05, -0.1, 0.2]);
    let qdd = accelerations(order, &[0.3, -0.2, 0.1, -0.1, 0.2, -0.15]);
    let mut dynamics = ArmDynamics::new(chain, [0.0, 0.0, -9.81]).unwrap();
    let backend_gravity = dynamics.gravity_compensation(&q).unwrap();
    let prepared = PreparedGravity::new(chain, [0.0, 0.0, -9.81], 7).unwrap();
    let mut gravity_workspace = prepared.workspace();
    let analytical_gravity = prepared
        .gravity_compensation(&q, &mut gravity_workspace)
        .unwrap();
    for (actual, expected) in analytical_gravity
        .values()
        .iter()
        .zip(backend_gravity.values())
    {
        assert_near(actual.value_si(), expected.value_si(), 3.0e-3);
    }
    for cog in [[0.0, 0.2, 0.0], [0.0, 0.0, 0.2]] {
        let payload =
            AttachedPayloadInertia::new(1.0, cog, [0.01, 0.0, 0.0, 0.01, 0.0, 0.01]).unwrap();
        let mut with_payload =
            ArmDynamics::new_with_payload(chain, [0.0, 0.0, -9.81], payload).unwrap();
        let total = with_payload.gravity_compensation(&q).unwrap();
        let mut theta = [1.0, 0.0, 0.0, 0.0];
        theta[1] = cog[0];
        theta[2] = cog[1];
        theta[3] = cog[2];
        let predicted = prepared
            .payload_gravity(&q, theta, &mut gravity_workspace)
            .unwrap();
        for ((actual, nominal), expected) in total
            .values()
            .iter()
            .zip(backend_gravity.values())
            .zip(predicted.values())
        {
            assert_near(
                actual.value_si() - nominal.value_si(),
                expected.value_si(),
                3.0e-3,
            );
        }
    }
    let effort = dynamics.inverse_dynamics(&q, &qd, &qdd).unwrap();
    let recovered = dynamics.forward_dynamics(&q, &qd, &effort).unwrap();
    for (actual, expected) in recovered
        .values()
        .iter()
        .zip(qdd.values())
        .map(|(actual, expected)| (actual.value_si(), expected.value_si()))
    {
        assert_near(actual, expected, 2.0e-3);
    }
    let mass = dynamics.mass_matrix(&q).unwrap();
    for row in 0..mass.dof() {
        assert!(mass.get(row, row).unwrap() > 0.0);
        for column in 0..mass.dof() {
            assert_near(
                mass.get(row, column).unwrap(),
                mass.get(column, row).unwrap(),
                1.0e-5,
            );
        }
    }
}

#[test]
fn missing_inertia_fails_closed_and_timed_effort_is_checked() {
    let missing = mixed_model();
    let missing_chain = KinematicChain::new(&missing, id("left_arm")).unwrap();
    assert!(matches!(
        ArmDynamics::new(missing_chain, [0.0, 0.0, -9.81]),
        Err(ArmAlgorithmError::DynamicsUnavailable { .. })
    ));

    let model = synthetic_model();
    let chain = KinematicChain::new(&model, id("arm")).unwrap();
    let start = chain.positions_from_values(&[0.0; 6]).unwrap();
    let goal = chain
        .positions_from_values(&[0.1, -0.1, 0.1, 0.05, -0.05, 0.1])
        .unwrap();
    let trajectory =
        ArmTrajectory::trapezoidal(chain, &start, &goal, TrajectorySampling::new(17).unwrap())
            .unwrap();
    let mut dynamics = ArmDynamics::new(chain, [0.0, 0.0, -9.81]).unwrap();
    let feasibility = dynamics.check_trajectory_effort(&trajectory).unwrap();
    assert_eq!(feasibility.evaluated_points(), 17);
    assert!(feasibility.maximum_utilization().is_finite());

    let zero_effort = {
        let mut values = ArrayVec::<ArmEffort, MAX_JOINTS>::new();
        for descriptor in chain.group().joint_order().joints() {
            values.push(
                match descriptor.kind() {
                    ArmJointKind::Revolute | ArmJointKind::Continuous => ArmEffort::torque_nm(0.0),
                    ArmJointKind::Prismatic => ArmEffort::force_n(0.0),
                }
                .unwrap(),
            );
        }
        ArmJointVector::new(chain.group().joint_order().clone(), values).unwrap()
    };
    let acceleration = dynamics
        .forward_dynamics(
            &start,
            &velocities(chain.group().joint_order(), &[0.0; 6]),
            &zero_effort,
        )
        .unwrap();
    assert!(
        acceleration
            .values()
            .iter()
            .all(|value| value.value_si().is_finite())
    );
}

#[test]
fn attached_payload_inertia_rejects_nonfinite_and_non_positive_tensors() {
    assert!(
        AttachedPayloadInertia::new(0.0, [0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 1.0, 0.0, 1.0],).is_err()
    );
    assert!(
        AttachedPayloadInertia::new(1.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0, 0.0, 1.0],).is_err()
    );
    assert!(
        AttachedPayloadInertia::new(1.0, [f64::NAN, 0.0, 0.0], [1.0, 0.0, 0.0, 1.0, 0.0, 1.0],)
            .is_err()
    );
}
