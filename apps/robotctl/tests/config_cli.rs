use std::{path::PathBuf, process::Command};

fn example_config() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../config/robots/example.yaml")
}

fn robotctl() -> Command {
    Command::new(env!("CARGO_BIN_EXE_robotctl"))
}

#[test]
fn validate_reports_stable_configuration_identity() {
    let output = robotctl()
        .args(["config", "validate"])
        .arg(example_config())
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8(output.stdout).unwrap();
    assert_eq!(
        stdout,
        "valid schema_version=1 sha256=6f2f323e46ee4f3fbcb080d9e8ed1c08411763a15f1af39d16d48262d1748b10\n"
    );
}

#[test]
fn normalized_output_is_byte_stable_across_runs() {
    let run = || {
        robotctl()
            .args(["config", "print-normalized"])
            .arg(example_config())
            .output()
            .unwrap()
    };
    let first = run();
    let second = run();
    assert!(first.status.success());
    assert!(second.status.success());
    assert_eq!(first.stdout, second.stdout);
    assert!(first.stderr.is_empty());
    assert!(second.stderr.is_empty());
}
