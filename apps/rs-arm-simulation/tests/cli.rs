use std::process::Command;

#[test]
fn qualify_has_stable_machine_output_and_failures_use_stderr() {
    let binary = env!("CARGO_BIN_EXE_rs-arm-simulation");
    let qualified = Command::new(binary).arg("qualify").output().unwrap();
    assert!(qualified.status.success());
    assert!(qualified.stderr.is_empty());
    let stdout = String::from_utf8(qualified.stdout).unwrap();
    assert!(stdout.starts_with("{\n  \"schema_version\": 1,"));
    assert!(stdout.contains("\"all_passed\": true"));
    assert!(stdout.ends_with("\n"));

    let missing = Command::new(binary)
        .args(["verify", "/definitely/missing/arm-report.json"])
        .output()
        .unwrap();
    assert!(!missing.status.success());
    assert!(missing.stdout.is_empty());
    assert!(
        String::from_utf8(missing.stderr)
            .unwrap()
            .starts_with("rs-arm-simulation: ")
    );
}

#[test]
fn qualify_can_write_and_verify_a_canonical_report() {
    let binary = env!("CARGO_BIN_EXE_rs-arm-simulation");
    let directory = tempfile::tempdir().unwrap();
    let report = directory.path().join("report.json");
    let qualified = Command::new(binary)
        .args(["qualify", "--output"])
        .arg(&report)
        .output()
        .unwrap();
    assert!(qualified.status.success());
    assert!(qualified.stdout.is_empty());
    assert!(qualified.stderr.is_empty());

    let verified = Command::new(binary)
        .arg("verify")
        .arg(&report)
        .output()
        .unwrap();
    assert!(verified.status.success());
    assert!(verified.stderr.is_empty());
    assert!(
        String::from_utf8(verified.stdout)
            .unwrap()
            .starts_with("verified ")
    );

    let refused = Command::new(binary)
        .args(["qualify", "--output"])
        .arg(&report)
        .output()
        .unwrap();
    assert!(!refused.status.success());
    assert!(refused.stdout.is_empty());
    assert!(
        String::from_utf8(refused.stderr)
            .unwrap()
            .starts_with("rs-arm-simulation: ")
    );
}
