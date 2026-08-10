use std::{path::Path, process::Command};

use arrayvec::ArrayVec;
use message_model::{AsciiId, ClockState, ClockStatus, DataQuality, Header, HeaderFields, Id128};
use middleware_api::LatestPublisher;
use navigation_ipc::{
    NavigationIpcNamespace, NavigationParticipant, NavigationParticipantReason,
    NavigationParticipantState, NavigationParticipationSnapshot,
};
use rustix::time::{ClockId, clock_gettime};

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

#[test]
fn inspect_reads_a_fresh_snapshot_without_consuming_it() {
    let temporary = tempfile::tempdir().expect("temporary namespace");
    let root = temporary.path().join("navigation-ipc");
    let namespace = NavigationIpcNamespace::new(&root).expect("namespace");
    namespace.create_layouts().expect("layouts");
    let mut publisher = namespace
        .attach_participation_publisher()
        .expect("publisher");
    let now = monotonic_ns();
    let mut participants = ArrayVec::new();
    participants.push(NavigationParticipant::new(
        AsciiId::parse("tree.navigate_to_pose").expect("static ID"),
        NavigationParticipantState::Active,
        NavigationParticipantReason::Executing,
    ));
    participants.push(NavigationParticipant::new(
        AsciiId::parse("output.fake_sink").expect("static ID"),
        NavigationParticipantState::Active,
        NavigationParticipantReason::FakeOutputApplied,
    ));
    let snapshot = NavigationParticipationSnapshot::new(
        header(now),
        7,
        Some(Id128::new([9; 16]).expect("nonzero command")),
        participants,
    )
    .expect("snapshot");
    let _ = publisher.try_publish_without_subscriber(snapshot);
    let before = publisher.counters();

    let output = btctl()
        .arg("inspect")
        .arg("--ipc-root")
        .arg(&root)
        .output()
        .expect("btctl runs");
    assert!(output.status.success());
    assert!(output.stderr.is_empty());
    let stdout = String::from_utf8(output.stdout).expect("CLI writes UTF-8");
    assert!(stdout.starts_with("snapshot generation=1 source=rs-navigation-e2e"));
    assert!(stdout.contains("process_epoch=7"));
    assert!(stdout.contains("participant id=tree.navigate_to_pose state=active reason=executing"));
    assert!(
        stdout.contains("participant id=output.fake_sink state=active reason=fake_output_applied")
    );
    assert_eq!(publisher.counters(), before);
}

#[test]
fn inspect_rejects_relative_or_missing_namespaces() {
    let relative = btctl()
        .arg("inspect")
        .arg("--ipc-root")
        .arg("relative-ipc")
        .output()
        .expect("btctl runs");
    assert!(!relative.status.success());
    assert!(relative.stdout.is_empty());
    assert!(
        String::from_utf8(relative.stderr)
            .expect("CLI writes UTF-8")
            .contains("--ipc-root must be absolute")
    );

    let temporary = tempfile::tempdir().expect("temporary directory");
    let missing = temporary.path().join("missing-ipc");
    let absent = btctl()
        .arg("inspect")
        .arg("--ipc-root")
        .arg(&missing)
        .output()
        .expect("btctl runs");
    assert!(!absent.status.success());
    assert!(absent.stdout.is_empty());
    assert!(
        String::from_utf8(absent.stderr)
            .expect("CLI writes UTF-8")
            .contains("navigation IPC namespace is missing")
    );
    assert!(!Path::new(&missing).exists());
}

#[test]
fn inspect_rejects_stale_or_incompatible_participation_data() {
    let temporary = tempfile::tempdir().expect("temporary namespace");
    let stale_root = temporary.path().join("stale-ipc");
    let stale = NavigationIpcNamespace::new(&stale_root).expect("namespace");
    stale.create_layouts().expect("layouts");
    let mut publisher = stale.attach_participation_publisher().expect("publisher");
    let mut participants = ArrayVec::new();
    participants.push(NavigationParticipant::new(
        AsciiId::parse("tree.navigate_to_pose").expect("static ID"),
        NavigationParticipantState::Pending,
        NavigationParticipantReason::WaitingAcknowledgement,
    ));
    let stale_snapshot = NavigationParticipationSnapshot::new(
        header(1),
        8,
        Some(Id128::new([7; 16]).expect("nonzero command")),
        participants,
    )
    .expect("snapshot");
    let _ = publisher.try_publish_without_subscriber(stale_snapshot);
    let stale_output = btctl()
        .arg("inspect")
        .arg("--ipc-root")
        .arg(&stale_root)
        .output()
        .expect("btctl runs");
    assert!(!stale_output.status.success());
    assert!(stale_output.stdout.is_empty());
    assert!(
        String::from_utf8(stale_output.stderr)
            .expect("CLI writes UTF-8")
            .contains("missing, stale, or invalid")
    );

    let incompatible_root = temporary.path().join("incompatible-ipc");
    let incompatible = NavigationIpcNamespace::new(&incompatible_root).expect("namespace");
    incompatible.create_layouts().expect("layouts");
    std::fs::OpenOptions::new()
        .write(true)
        .open(incompatible_root.join("navigation.participation.v1.region"))
        .expect("region")
        .set_len(1)
        .expect("truncate temporary region");
    let incompatible_output = btctl()
        .arg("inspect")
        .arg("--ipc-root")
        .arg(&incompatible_root)
        .output()
        .expect("btctl runs");
    assert!(!incompatible_output.status.success());
    assert!(incompatible_output.stdout.is_empty());
    assert!(
        String::from_utf8(incompatible_output.stderr)
            .expect("CLI writes UTF-8")
            .contains("region length mismatch")
    );
}

fn header(now: u64) -> Header {
    Header::new(HeaderFields {
        source_id: AsciiId::parse("rs-navigation-e2e").expect("static source"),
        boot_id: Id128::new([8; 16]).expect("nonzero boot"),
        sequence: 1,
        source_monotonic_ns: now,
        publish_unix_ns: None,
        acquisition_timestamp: None,
        valid_for_ns: 250_000_000,
        frame_id: None,
        clock_status: ClockStatus {
            state: ClockState::Local,
            uncertainty_ns: 0,
        },
        quality: DataQuality::Valid,
    })
    .expect("valid header")
}

fn monotonic_ns() -> u64 {
    let time = clock_gettime(ClockId::Monotonic);
    u64::try_from(time.tv_sec)
        .unwrap_or(u64::MAX)
        .saturating_mul(1_000_000_000)
        .saturating_add(u64::try_from(time.tv_nsec).unwrap_or(0))
}
