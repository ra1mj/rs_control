use std::process::Command;

fn command() -> Command {
    Command::new(env!("CARGO_BIN_EXE_middleware-qualification"))
}

#[test]
fn local_run_emits_versioned_host_evidence() {
    let output = command()
        .args([
            "--backend=local",
            "--kind=latest",
            "--warmup=2",
            "--samples=4",
        ])
        .output()
        .unwrap();
    assert!(output.status.success());
    let stdout = String::from_utf8(output.stdout).unwrap();
    assert!(stdout.starts_with("{\"schema_version\":1,\"qualification\":\"host_research\""));
    assert!(stdout.contains("\"backend\":\"local\""));
    assert!(stdout.contains("\"samples\":4"));
    assert!(output.stderr.is_empty());
}

#[test]
fn shm_process_run_completes_bounded_round_trips() {
    for kind in ["latest", "ordered"] {
        let output = command()
            .args([
                "--backend=shm",
                &format!("--kind={kind}"),
                "--peer=process",
                "--warmup=2",
                "--samples=4",
            ])
            .output()
            .unwrap();
        assert!(
            output.status.success(),
            "{}",
            String::from_utf8_lossy(&output.stderr)
        );
        let stdout = String::from_utf8(output.stdout).unwrap();
        assert!(stdout.contains("\"backend\":\"shm\""));
        assert!(stdout.contains("\"peer\":\"process\""));
        assert!(stdout.contains(&format!("\"kind\":\"{kind}\"")));
    }
}

#[test]
fn existing_output_is_not_overwritten() {
    let temp = tempfile::TempDir::new().unwrap();
    let output_path = temp.path().join("evidence.json");
    std::fs::write(&output_path, b"preserve").unwrap();
    let output = command()
        .args(["--samples=1", "--output"])
        .arg(&output_path)
        .output()
        .unwrap();
    assert!(!output.status.success());
    assert_eq!(std::fs::read(output_path).unwrap(), b"preserve");
}

#[test]
fn invalid_unbounded_settings_are_rejected() {
    let output = command().args(["--samples=0"]).output().unwrap();
    assert!(!output.status.success());
    assert!(
        String::from_utf8(output.stderr)
            .unwrap()
            .contains("samples must be in 1..=10000000")
    );
}
