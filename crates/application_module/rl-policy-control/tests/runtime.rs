mod common;

use std::fs;
use std::path::Path;
use std::time::{Duration, Instant};

use message_model::{ArmControlMode, ArmControlValue};
use rl_policy_control::{
    PolicyError, PolicyFallback, PolicyRuntime, PolicyTerminalReason, ReadyPolicy,
};
use sha2::{Digest, Sha256};

use common::{copy_fixture, fixture_control, fixture_model, observation};

#[test]
fn checked_fixture_loads_and_runs_fixed_gain_mit() {
    let model = fixture_model();
    let control = fixture_control(&model);
    let ready = ReadyPolicy::load(&common::checked_fixture(), &control).unwrap();
    assert_ne!(ready.identity().as_bytes(), [0; 32]);
    assert!(ready.generation().get() > 0);

    let mut runtime = PolicyRuntime::spawn(ready).unwrap();
    let submitted = Instant::now();
    let request = runtime
        .try_submit(observation(&model, 1, 1_000, 1_000_000), 1_100, submitted)
        .unwrap();
    let decision = loop {
        match runtime.try_receive(2_000, Instant::now()) {
            Ok(decision) => break decision,
            Err(PolicyError::ResultUnavailable) => std::thread::yield_now(),
            Err(error) => panic!("unexpected policy result: {error}"),
        }
    };
    assert_eq!(decision.request_sequence(), request);
    assert_eq!(decision.proposal().mode(), ArmControlMode::Mit);
    let expected = [(0.1, 0.2, 0.3), (-0.1, -0.2, -0.3)];
    for (value, expected) in decision.proposal().values().iter().zip(expected) {
        let ArmControlValue::Mit(value) = value else {
            panic!("expected MIT output");
        };
        assert!((value.position_rad() - expected.0).abs() < 1e-6);
        assert!((value.velocity_radps() - expected.1).abs() < 1e-6);
        assert_eq!(value.kp(), 20.0);
        assert_eq!(value.kd(), 0.5);
        assert!((value.feed_forward_torque_nm() - expected.2).abs() < 1e-6);
    }
    assert!(!decision.saturated());
}

#[test]
fn artifact_hash_extra_file_and_unknown_manifest_field_fail_closed() {
    let model = fixture_model();
    let control = fixture_control(&model);

    let hash_dir = tempfile::tempdir().unwrap();
    copy_fixture(hash_dir.path());
    let mut bytes = fs::read(hash_dir.path().join("model.onnx")).unwrap();
    bytes[0] ^= 1;
    fs::write(hash_dir.path().join("model.onnx"), bytes).unwrap();
    let error = ReadyPolicy::load(hash_dir.path(), &control).err().unwrap();
    assert_eq!(error.fallback(), PolicyFallback::WithdrawProposal);
    assert!(matches!(error, PolicyError::Artifact(_)));

    let extra_dir = tempfile::tempdir().unwrap();
    copy_fixture(extra_dir.path());
    fs::write(extra_dir.path().join("weights.bin"), b"forbidden").unwrap();
    assert!(matches!(
        ReadyPolicy::load(extra_dir.path(), &control).err().unwrap(),
        PolicyError::Artifact(_)
    ));

    let manifest_dir = tempfile::tempdir().unwrap();
    copy_fixture(manifest_dir.path());
    let manifest = fs::read_to_string(manifest_dir.path().join("manifest.toml")).unwrap();
    fs::write(
        manifest_dir.path().join("manifest.toml"),
        format!("{manifest}\nunknown_runtime_switch = true\n"),
    )
    .unwrap();
    assert!(matches!(
        ReadyPolicy::load(manifest_dir.path(), &control)
            .err()
            .unwrap(),
        PolicyError::Manifest(_)
    ));
}

#[test]
fn stale_and_replayed_observations_never_produce_a_proposal() {
    let model = fixture_model();
    let control = fixture_control(&model);
    let ready = ReadyPolicy::load(&common::checked_fixture(), &control).unwrap();
    let mut runtime = PolicyRuntime::spawn(ready).unwrap();
    let now = Instant::now();

    let stale = runtime
        .try_submit(observation(&model, 1, 100, 10), 110, now)
        .unwrap_err();
    assert!(matches!(stale, PolicyError::Observation(_)));
    assert_eq!(stale.fallback(), PolicyFallback::WithdrawProposal);

    runtime
        .try_submit(
            observation(&model, 2, 1_000, 1_000_000),
            1_100,
            now + Duration::from_millis(100),
        )
        .unwrap();
    let replay = runtime
        .try_submit(
            observation(&model, 2, 1_100, 1_000_000),
            1_200,
            now + Duration::from_millis(200),
        )
        .unwrap_err();
    assert!(matches!(replay, PolicyError::Observation(_)));
}

#[test]
fn late_result_is_withdrawn_and_reload_changes_generation() {
    let model = fixture_model();
    let control = fixture_control(&model);
    let first = ReadyPolicy::load(&common::checked_fixture(), &control).unwrap();
    let first_generation = first.generation();
    let second = ReadyPolicy::load(&common::checked_fixture(), &control).unwrap();
    let second_generation = second.generation();
    assert_ne!(first_generation, second_generation);

    let mut runtime = PolicyRuntime::spawn(first).unwrap();
    let submitted = Instant::now();
    runtime
        .try_submit(observation(&model, 1, 1_000, 1_000_000), 1_100, submitted)
        .unwrap();
    loop {
        match runtime.try_receive(100_000_000, submitted + Duration::from_millis(90)) {
            Err(PolicyError::ResultUnavailable) => std::thread::yield_now(),
            Err(PolicyError::DeadlineExpired) => break,
            other => panic!("expected exclusive deadline failure, got {other:?}"),
        }
    }
    assert_eq!(runtime.counters().deadline_expired, 1);
    let runtime = runtime.reload(second).unwrap();
    assert!(runtime.is_ready());
}

#[test]
fn explicit_worker_shutdown_is_typed_and_non_authorizing() {
    let model = fixture_model();
    let control = fixture_control(&model);
    let ready = ReadyPolicy::load(&common::checked_fixture(), &control).unwrap();
    let mut runtime = PolicyRuntime::spawn(ready).unwrap();
    runtime.shutdown();
    assert!(!runtime.is_ready());
    assert_eq!(
        runtime.terminal_reason(),
        Some(PolicyTerminalReason::CleanShutdown)
    );
    let error = runtime
        .try_submit(
            observation(&model, 1, 1_000, 1_000_000),
            1_100,
            Instant::now(),
        )
        .unwrap_err();
    assert!(matches!(error, PolicyError::WorkerUnavailable));
    assert_eq!(error.fallback(), PolicyFallback::WithdrawProposal);

    let ready = ReadyPolicy::load(&common::checked_fixture(), &control).unwrap();
    let mut supervised = PolicyRuntime::spawn(ready).unwrap();
    supervised.mark_supervised_process_lost();
    assert_eq!(
        supervised.terminal_reason(),
        Some(PolicyTerminalReason::SupervisedProcessLost)
    );
    assert!(matches!(
        supervised.try_receive(2_000, Instant::now()).unwrap_err(),
        PolicyError::WorkerUnavailable
    ));
}

#[test]
fn all_non_mit_action_semantics_produce_typed_proposals() {
    let cases = [
        ("absolute_position", ArmControlMode::AbsolutePosition),
        ("position_residual", ArmControlMode::PositionResidual),
        ("joint_velocity", ArmControlMode::Velocity),
        ("direct_rotary_torque", ArmControlMode::DirectRotaryTorque),
    ];
    for (manifest_kind, expected_mode) in cases {
        let model = fixture_model();
        let control = fixture_control(&model);
        let directory = tempfile::tempdir().unwrap();
        write_joint_artifact(directory.path(), manifest_kind, None);
        let ready = ReadyPolicy::load(directory.path(), &control).unwrap();
        let mut runtime = PolicyRuntime::spawn(ready).unwrap();
        runtime
            .try_submit(
                observation(&model, 1, 1_000, 1_000_000),
                1_100,
                Instant::now(),
            )
            .unwrap();
        let decision = loop {
            match runtime.try_receive(2_000, Instant::now()) {
                Ok(decision) => break decision,
                Err(PolicyError::ResultUnavailable) => std::thread::yield_now(),
                Err(error) => panic!("{manifest_kind} failed: {error}"),
            }
        };
        assert_eq!(decision.proposal().mode(), expected_mode);
        assert_eq!(decision.proposal().values().len(), 2);
        match (expected_mode, &decision.proposal().values()[0]) {
            (
                ArmControlMode::AbsolutePosition | ArmControlMode::PositionResidual,
                ArmControlValue::Position(value),
            ) => assert!((value.value_si() - 0.7).abs() < 1e-6),
            (ArmControlMode::Velocity, ArmControlValue::Velocity(value)) => {
                assert!((value.value_si() - 0.7).abs() < 1e-6);
            }
            (ArmControlMode::DirectRotaryTorque, ArmControlValue::RotaryTorque(value)) => {
                assert!((value.get() - 0.7).abs() < 1e-6);
            }
            _ => panic!("wrong typed payload for {manifest_kind}"),
        }
    }
}

#[test]
fn dynamic_or_non_f32_onnx_inputs_are_rejected_before_readiness() {
    let model = fixture_model();
    let control = fixture_control(&model);
    let original = fs::read(
        common::checked_fixture()
            .parent()
            .unwrap()
            .join("joint_affine")
            .join("model.onnx"),
    )
    .unwrap();

    let mut dynamic = original.clone();
    let shape = find_subslice(
        &dynamic,
        &[0x12, 0x08, 0x0a, 0x02, 0x08, 0x01, 0x0a, 0x02, 0x08, 0x02],
    );
    dynamic[shape + 8] = 0x12;
    dynamic[shape + 9] = 0;
    let dynamic_dir = tempfile::tempdir().unwrap();
    write_joint_artifact(dynamic_dir.path(), "absolute_position", Some(&dynamic));
    assert!(matches!(
        ReadyPolicy::load(dynamic_dir.path(), &control)
            .err()
            .unwrap(),
        PolicyError::Graph(_)
    ));

    let mut int32 = original;
    let tensor_type = find_subslice(&int32, &[0x0a, 0x0c, 0x08, 0x01, 0x12, 0x08]);
    int32[tensor_type + 3] = 6;
    let dtype_dir = tempfile::tempdir().unwrap();
    write_joint_artifact(dtype_dir.path(), "absolute_position", Some(&int32));
    assert!(matches!(
        ReadyPolicy::load(dtype_dir.path(), &control).err().unwrap(),
        PolicyError::Graph(_)
    ));

    let mut non_finite = fs::read(
        common::checked_fixture()
            .parent()
            .unwrap()
            .join("joint_affine")
            .join("model.onnx"),
    )
    .unwrap();
    let constant = find_subslice(&non_finite, &[0x25, 0, 0, 0, 0x40]);
    non_finite[constant + 1..constant + 5].copy_from_slice(&f32::NAN.to_le_bytes());
    let non_finite_dir = tempfile::tempdir().unwrap();
    write_joint_artifact(
        non_finite_dir.path(),
        "absolute_position",
        Some(&non_finite),
    );
    assert!(matches!(
        ReadyPolicy::load(non_finite_dir.path(), &control)
            .err()
            .unwrap(),
        PolicyError::Graph(_)
    ));

    let external = fs::read(
        common::checked_fixture()
            .parent()
            .unwrap()
            .join("external_data")
            .join("model.onnx"),
    )
    .unwrap();
    let external_dir = tempfile::tempdir().unwrap();
    write_joint_artifact(external_dir.path(), "absolute_position", Some(&external));
    assert!(matches!(
        ReadyPolicy::load(external_dir.path(), &control)
            .err()
            .unwrap(),
        PolicyError::Graph(_)
    ));
}

#[cfg(unix)]
#[test]
fn symlinked_artifact_files_are_rejected() {
    use std::os::unix::fs::symlink;

    let model = fixture_model();
    let control = fixture_control(&model);
    let directory = tempfile::tempdir().unwrap();
    fs::copy(
        common::checked_fixture().join("model.onnx"),
        directory.path().join("model.onnx"),
    )
    .unwrap();
    symlink(
        common::checked_fixture().join("manifest.toml"),
        directory.path().join("manifest.toml"),
    )
    .unwrap();
    assert!(matches!(
        ReadyPolicy::load(directory.path(), &control).err().unwrap(),
        PolicyError::Artifact(_)
    ));
}

#[test]
fn action_projection_is_evidenced_and_hard_overflow_withdraws() {
    let model = fixture_model();
    let control = fixture_control(&model);

    let projected_dir = tempfile::tempdir().unwrap();
    write_joint_artifact(projected_dir.path(), "absolute_position", None);
    mutate_manifest(
        projected_dir.path(),
        "scale = [1.0, 1.0]",
        "scale = [2.0, 2.0]",
    );
    let ready = ReadyPolicy::load(projected_dir.path(), &control).unwrap();
    let mut runtime = PolicyRuntime::spawn(ready).unwrap();
    runtime
        .try_submit(
            observation(&model, 1, 1_000, 1_000_000),
            1_100,
            Instant::now(),
        )
        .unwrap();
    let decision = receive_at(&mut runtime, 2_000, Instant::now());
    assert!(decision.saturated());
    assert_position_values(&decision, [1.0, -1.0]);

    let hard_dir = tempfile::tempdir().unwrap();
    write_joint_artifact(hard_dir.path(), "absolute_position", None);
    mutate_manifest(
        hard_dir.path(),
        "scale = [1.0, 1.0]",
        "scale = [20.0, 20.0]",
    );
    let ready = ReadyPolicy::load(hard_dir.path(), &control).unwrap();
    let mut runtime = PolicyRuntime::spawn(ready).unwrap();
    runtime
        .try_submit(
            observation(&model, 1, 1_000, 1_000_000),
            1_100,
            Instant::now(),
        )
        .unwrap();
    loop {
        match runtime.try_receive(2_000, Instant::now()) {
            Err(PolicyError::ResultUnavailable) => std::thread::yield_now(),
            Err(PolicyError::Inference(_)) => break,
            other => panic!("hard overflow did not withdraw: {other:?}"),
        }
    }
}

#[test]
fn mit_gain_profile_and_order_mismatch_fail_at_startup() {
    let model = fixture_model();
    let control = fixture_control(&model);

    for (from, to) in [
        ("fixed_kp = [20.0, 20.0]", "fixed_kp = [19.0, 20.0]"),
        (
            "joint_order = [\"joint_1\", \"joint_2\"]",
            "joint_order = [\"joint_2\", \"joint_1\"]",
        ),
        (
            "arm_profile_sha256 = \"af192ffe53a66335160e0da5462e712a7f531c858cb286138329de6ddb04adcc\"",
            "arm_profile_sha256 = \"bf192ffe53a66335160e0da5462e712a7f531c858cb286138329de6ddb04adcc\"",
        ),
    ] {
        let directory = tempfile::tempdir().unwrap();
        copy_fixture(directory.path());
        mutate_manifest(directory.path(), from, to);
        assert!(matches!(
            ReadyPolicy::load(directory.path(), &control).err().unwrap(),
            PolicyError::Binding(_)
        ));
    }
}

#[test]
fn recurrent_state_advances_only_after_acceptance_and_reset_discards_it() {
    let model = fixture_model();
    let control = fixture_control(&model);
    let recurrent = common::checked_fixture()
        .parent()
        .unwrap()
        .join("recurrent");
    let ready = ReadyPolicy::load(&recurrent, &control).unwrap();
    let mut runtime = PolicyRuntime::spawn(ready).unwrap();
    let base = Instant::now();

    runtime
        .try_submit(observation(&model, 1, 1_000, 1_000_000), 1_100, base)
        .unwrap();
    let first = receive_at(&mut runtime, 1_000_000_000, base + Duration::from_millis(1));
    assert_position_values(&first, [0.1, -0.1]);

    runtime
        .try_submit(
            observation(&model, 2, 2_000, 1_000_000),
            2_100,
            base + Duration::from_millis(100),
        )
        .unwrap();
    let second = receive_at(
        &mut runtime,
        1_100_000_000,
        base + Duration::from_millis(101),
    );
    assert_position_values(&second, [0.2, -0.2]);

    runtime.reset();
    runtime
        .try_submit(
            observation(&model, 3, 3_000, 1_000_000),
            3_100,
            base + Duration::from_millis(200),
        )
        .unwrap();
    let after_reset = receive_at(
        &mut runtime,
        1_200_000_000,
        base + Duration::from_millis(201),
    );
    assert_position_values(&after_reset, [0.1, -0.1]);
}

#[test]
fn recurrent_queue_discontinuity_withdraws_inflight_result() {
    let model = fixture_model();
    let control = fixture_control(&model);
    let recurrent = common::checked_fixture()
        .parent()
        .unwrap()
        .join("recurrent");
    let ready = ReadyPolicy::load(&recurrent, &control).unwrap();
    let mut runtime = PolicyRuntime::spawn(ready).unwrap();
    let now = Instant::now();
    runtime
        .try_submit(observation(&model, 1, 1_000, 1_000_000), 1_100, now)
        .unwrap();
    let error = runtime
        .try_submit(observation(&model, 2, 1_100, 1_000_000), 1_200, now)
        .unwrap_err();
    assert!(matches!(error, PolicyError::QueueFull));
    assert_eq!(error.fallback(), PolicyFallback::WithdrawProposal);
    assert_eq!(runtime.counters().request_full, 1);

    loop {
        match runtime.try_receive(2_000, now + Duration::from_millis(1)) {
            Err(PolicyError::ResultUnavailable) => std::thread::yield_now(),
            Err(PolicyError::StaleResult) => break,
            other => panic!("inflight recurrent result was not withdrawn: {other:?}"),
        }
    }
}

fn receive_at(
    runtime: &mut PolicyRuntime,
    accepted_time_ns: u64,
    now: Instant,
) -> rl_policy_control::PolicyDecision {
    loop {
        match runtime.try_receive(accepted_time_ns, now) {
            Ok(decision) => return decision,
            Err(PolicyError::ResultUnavailable) => std::thread::yield_now(),
            Err(error) => panic!("unexpected recurrent result: {error}"),
        }
    }
}

fn assert_position_values(decision: &rl_policy_control::PolicyDecision, expected: [f64; 2]) {
    for (value, expected) in decision.proposal().values().iter().zip(expected) {
        let ArmControlValue::Position(value) = value else {
            panic!("expected position output");
        };
        assert!(
            (value.value_si() - expected).abs() < 1e-6,
            "actual={} expected={expected}",
            value.value_si()
        );
    }
}

fn write_joint_artifact(directory: &Path, action_kind: &str, model_override: Option<&[u8]>) {
    let fixture = common::checked_fixture()
        .parent()
        .unwrap()
        .join("joint_affine")
        .join("model.onnx");
    let model = model_override
        .map(<[u8]>::to_vec)
        .unwrap_or_else(|| fs::read(fixture).unwrap());
    let digest: [u8; 32] = Sha256::digest(&model).into();
    let digest = digest
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect::<String>();
    fs::write(directory.join("model.onnx"), model).unwrap();
    fs::write(
        directory.join("manifest.toml"),
        format!(
            r#"
schema_version = 1
runtime = "rten"
runtime_version = "0.24.0"
model_format = "onnx"
model_sha256 = "{digest}"
max_model_bytes = 1024
arm_profile_sha256 = "af192ffe53a66335160e0da5462e712a7f531c858cb286138329de6ddb04adcc"
kinematic_group_id = "arm"
joint_order = ["joint_1", "joint_2"]
actuator_order = ["act_1", "act_2"]
inference_period_ns = 100000000
inference_deadline_ns = 90000000
queue_capacity = 2
thread_count = 1
fallback = "withdraw_proposal"

[[inputs]]
name = "observation"
dtype = "f32"
shape = [1, 2]

[[outputs]]
name = "action"
dtype = "f32"
shape = [1, 2]

[[observations]]
joint = "joint_1"
field = "position"
unit = "rad"
mean = 0.0
inverse_std = 1.0
minimum = -2.0
maximum = 2.0

[[observations]]
joint = "joint_2"
field = "position"
unit = "rad"
mean = 0.0
inverse_std = 1.0
minimum = -2.0
maximum = 2.0

[action]
kind = "{action_kind}"
output = "action"
scale = [1.0, 1.0]
offset = [0.0, 0.0]
hard_minimum = [-10.0, -10.0]
hard_maximum = [10.0, 10.0]
command_minimum = [-1.0, -1.0]
command_maximum = [1.0, 1.0]
slew_per_second = [5.0, 5.0]
"#
        ),
    )
    .unwrap();
}

fn find_subslice(haystack: &[u8], needle: &[u8]) -> usize {
    haystack
        .windows(needle.len())
        .position(|window| window == needle)
        .expect("generated ONNX fixture pattern")
}

fn mutate_manifest(directory: &Path, from: &str, to: &str) {
    let path = directory.join("manifest.toml");
    let source = fs::read_to_string(&path).unwrap();
    assert!(source.contains(from));
    fs::write(path, source.replacen(from, to, 1)).unwrap();
}
