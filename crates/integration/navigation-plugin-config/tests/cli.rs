use std::{fs, process::Command};

#[test]
fn fingerprint_is_stable_and_failure_uses_stderr() {
    let manifest = format!(
        "{}/../../../config/navigation/rs_control_navigation_plugin.yaml",
        env!("CARGO_MANIFEST_DIR")
    );
    let output = Command::new(env!("CARGO_BIN_EXE_rs-navigation-plugin"))
        .args(["fingerprint", &manifest])
        .output()
        .unwrap();
    assert!(output.status.success());
    assert_eq!(output.stderr, b"");
    assert_eq!(String::from_utf8(output.stdout).unwrap().trim().len(), 64);

    let directory = tempfile::tempdir().unwrap();
    let invalid = directory.path().join("invalid.yaml");
    fs::write(&invalid, "schema_version: 1\nunknown: true\n").unwrap();
    let output = Command::new(env!("CARGO_BIN_EXE_rs-navigation-plugin"))
        .args(["fingerprint", invalid.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    assert!(
        String::from_utf8(output.stderr)
            .unwrap()
            .starts_with("rs-navigation-plugin: invalid navigation manifest YAML:")
    );
}
