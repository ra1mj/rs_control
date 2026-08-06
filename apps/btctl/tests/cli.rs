use std::process::Command;

fn btctl() -> Command {
    Command::new(env!("CARGO_BIN_EXE_btctl"))
}

#[test]
fn example_is_byte_stable_and_does_not_report_a_hardware_boundary() {
    let run = || btctl().arg("example").output().expect("btctl runs");
    let first = run();
    let second = run();
    assert!(first.status.success());
    assert!(second.status.success());
    assert_eq!(first.stdout, second.stdout);
    assert!(first.stderr.is_empty());
    assert!(second.stderr.is_empty());

    let stdout = String::from_utf8(first.stdout).expect("CLI writes UTF-8");
    assert!(stdout.starts_with("BehaviorTree root=0\n"));
    assert!(stdout.contains("event=start\n"));
    assert!(stdout.contains("event=guard_resolved value=true\n"));
    assert!(stdout.contains("effect=dispatch_action"));
    assert!(stdout.contains("BehaviorTreeSnapshot state=running\n"));
    assert!(!stdout.contains("hardware"));
}

#[test]
fn unknown_subcommand_returns_a_nonzero_status() {
    let output = btctl().arg("unknown").output().expect("btctl runs");
    assert!(!output.status.success());
    assert!(output.stdout.is_empty());
    let stderr = String::from_utf8(output.stderr).expect("CLI writes UTF-8");
    assert!(stderr.contains("unrecognized subcommand 'unknown'"));
}
