use rs_arm_simulation::{SimulationConfig, run_qualification};

#[test]
fn qualification_is_deterministic_and_passes_all_gates() {
    let first = run_qualification().unwrap();
    let second = run_qualification().unwrap();
    assert_eq!(
        first.canonical_json().unwrap(),
        second.canonical_json().unwrap()
    );
    assert!(first.gates.all_passed, "{first:#?}");
    assert!(first.calibration_pose_windows <= 24);
    assert!(first.held_out_poses >= 8);
    assert!(first.rls_accepted_updates <= 2_048);
}

#[test]
fn invalid_timestep_and_step_budget_fail_closed() {
    assert!(SimulationConfig::new(0.0, 1).is_err());
    assert!(SimulationConfig::new(f64::NAN, 1).is_err());
    assert!(SimulationConfig::new(0.001, 0).is_err());
    assert!(SimulationConfig::new(0.001, 1_000_001).is_err());
}
