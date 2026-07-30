use std::{
    env, fs,
    path::{Path, PathBuf},
    process::{Child, Command, ExitStatus, Stdio},
    thread,
    time::{Duration, Instant},
};

use bytemuck::{Pod, Zeroable};
use middleware_api::{
    EventPublishError, EventPublisher, EventSubscriber, LatestPublisher, LatestSubscriber,
};
use middleware_shm::{
    ChannelName, EndpointRole, IdentityCodec, MemoryPolicy, ShmChannelConfig, ShmError,
    ShmNamespace, ShmPayload,
};
use tempfile::TempDir;

const TIMEOUT: Duration = Duration::from_secs(5);

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Pod, Zeroable)]
struct Payload {
    first: u64,
    second: u64,
}

impl ShmPayload for Payload {
    const SCHEMA_ID: [u8; 16] = *b"process-fault001";
    const SCHEMA_VERSION: u32 = 1;
}

fn config(capacity: u32) -> ShmChannelConfig {
    ShmChannelConfig::new(capacity, MemoryPolicy::Prefault).unwrap()
}

fn spawn_peer(root: &Path, name: &str, mode: &str, ready: &Path, result: &Path) -> Child {
    Command::new(env::current_exe().unwrap())
        .args(["--exact", "process_peer", "--nocapture"])
        .env("RS_SHM_TEST_ROOT", root)
        .env("RS_SHM_TEST_NAME", name)
        .env("RS_SHM_TEST_MODE", mode)
        .env("RS_SHM_TEST_READY", ready)
        .env("RS_SHM_TEST_RESULT", result)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .spawn()
        .unwrap()
}

fn wait_for_file(path: &Path) {
    let deadline = Instant::now() + TIMEOUT;
    while !path.exists() {
        assert!(
            Instant::now() < deadline,
            "peer readiness timeout: {path:?}"
        );
        thread::sleep(Duration::from_millis(10));
    }
}

fn wait_for_exit(child: &mut Child) -> ExitStatus {
    let deadline = Instant::now() + TIMEOUT;
    loop {
        if let Some(status) = child.try_wait().unwrap() {
            return status;
        }
        if Instant::now() >= deadline {
            child.kill().unwrap();
            let _ = child.wait();
            panic!("peer exit timeout");
        }
        thread::sleep(Duration::from_millis(10));
    }
}

#[test]
fn process_peer() {
    let Ok(root) = env::var("RS_SHM_TEST_ROOT") else {
        return;
    };
    let name = ChannelName::new(&env::var("RS_SHM_TEST_NAME").unwrap()).unwrap();
    let mode = env::var("RS_SHM_TEST_MODE").unwrap();
    let ready = PathBuf::from(env::var_os("RS_SHM_TEST_READY").unwrap());
    let result = PathBuf::from(env::var_os("RS_SHM_TEST_RESULT").unwrap());
    let namespace = ShmNamespace::new(root).unwrap();

    match mode.as_str() {
        "receive-event" => {
            let mut subscriber = namespace
                .attach_event_subscriber(name, config(4), IdentityCodec::<Payload>::new())
                .unwrap();
            fs::write(&ready, b"ready").unwrap();
            let deadline = Instant::now() + TIMEOUT;
            loop {
                if let Some(sample) = subscriber.try_receive() {
                    fs::write(result, sample.value.first.to_string()).unwrap();
                    return;
                }
                assert!(Instant::now() < deadline, "event receive timeout");
                thread::yield_now();
            }
        }
        "hold-subscriber" => {
            let _subscriber = namespace
                .attach_event_subscriber(name, config(2), IdentityCodec::<Payload>::new())
                .unwrap();
            fs::write(&ready, b"ready").unwrap();
            loop {
                thread::park();
            }
        }
        #[cfg(feature = "fault-injection")]
        "odd-publisher" => {
            let mut publisher = namespace
                .attach_latest_publisher(name, config(3), IdentityCodec::<Payload>::new())
                .unwrap();
            publisher.fault_set_diagnostic_pid_for_test(1);
            publisher.fault_set_diagnostic_epoch_for_test(u64::MAX);
            let generation = publisher.fault_publish_incomplete_for_test(&Payload {
                first: 0xaaaa_aaaa_aaaa_aaaa,
                second: 0xbbbb_bbbb_bbbb_bbbb,
            });
            assert_eq!(generation, 1);
            fs::write(&ready, b"ready").unwrap();
            loop {
                thread::park();
            }
        }
        other => panic!("unknown peer mode: {other}"),
    }
}

#[test]
fn normal_event_crosses_a_real_process_boundary() {
    let temp = TempDir::new().unwrap();
    let root = temp.path().join("shm");
    let namespace = ShmNamespace::new(&root).unwrap();
    let name = ChannelName::new("process-event").unwrap();
    namespace
        .create_ordered::<Payload>(name, config(4))
        .unwrap();
    let mut publisher = namespace
        .attach_event_publisher(name, config(4), IdentityCodec::<Payload>::new())
        .unwrap();
    let ready = temp.path().join("ready");
    let result = temp.path().join("result");
    let mut child = spawn_peer(&root, name.as_str(), "receive-event", &ready, &result);
    wait_for_file(&ready);
    assert!(
        publisher
            .try_publish(Payload {
                first: 42,
                second: 24
            })
            .is_ok()
    );
    assert!(wait_for_exit(&mut child).success());
    assert_eq!(fs::read_to_string(result).unwrap(), "42");
    let replacement = namespace
        .attach_event_subscriber(name, config(4), IdentityCodec::<Payload>::new())
        .unwrap();
    assert_eq!(replacement.diagnostics().reclaimed_endpoints, 0);
}

#[test]
fn killed_consumer_never_blocks_producer_and_replacement_skips_backlog() {
    let temp = TempDir::new().unwrap();
    let root = temp.path().join("shm");
    let namespace = ShmNamespace::new(&root).unwrap();
    let name = ChannelName::new("killed-consumer").unwrap();
    namespace
        .create_ordered::<Payload>(name, config(2))
        .unwrap();
    let mut publisher = namespace
        .attach_event_publisher(name, config(2), IdentityCodec::<Payload>::new())
        .unwrap();
    let ready = temp.path().join("ready");
    let result = temp.path().join("unused");
    let mut child = spawn_peer(&root, name.as_str(), "hold-subscriber", &ready, &result);
    wait_for_file(&ready);
    assert!(
        publisher
            .try_publish(Payload {
                first: 1,
                second: 1
            })
            .is_ok()
    );
    assert!(
        publisher
            .try_publish(Payload {
                first: 2,
                second: 2
            })
            .is_ok()
    );
    let started = Instant::now();
    for value in 0..10_000 {
        let payload = Payload {
            first: value,
            second: value,
        };
        assert_eq!(
            publisher.try_publish(payload),
            Err(EventPublishError::Full(payload))
        );
    }
    assert!(started.elapsed() < TIMEOUT);
    child.kill().unwrap();
    assert!(!wait_for_exit(&mut child).success());
    assert!(!publisher.refresh_peer_liveness().unwrap());
    let disconnected = Payload {
        first: 3,
        second: 3,
    };
    assert_eq!(
        publisher.try_publish(disconnected),
        Err(EventPublishError::Disconnected(disconnected))
    );
    let mut replacement = namespace
        .attach_event_subscriber(name, config(2), IdentityCodec::<Payload>::new())
        .unwrap();
    assert_eq!(replacement.diagnostics().reclaimed_endpoints, 1);
    assert_eq!(replacement.diagnostics().skipped_backlog, 2);
    assert!(
        publisher
            .try_publish(Payload {
                first: 4,
                second: 4
            })
            .is_ok()
    );
    assert_eq!(replacement.try_receive().unwrap().value.first, 4);
}

#[cfg(feature = "fault-injection")]
#[test]
fn sigkill_reclaims_corrupt_pid_writer_and_odd_slot_is_never_exposed() {
    let temp = TempDir::new().unwrap();
    let root = temp.path().join("shm");
    let namespace = ShmNamespace::new(&root).unwrap();
    let name = ChannelName::new("odd-writer").unwrap();
    namespace.create_latest::<Payload>(name, config(3)).unwrap();
    let mut subscriber = namespace
        .attach_latest_subscriber(name, config(3), IdentityCodec::<Payload>::new())
        .unwrap();
    let ready = temp.path().join("ready");
    let result = temp.path().join("unused");
    let mut child = spawn_peer(&root, name.as_str(), "odd-publisher", &ready, &result);
    wait_for_file(&ready);
    assert!(matches!(
        namespace.attach_latest_publisher::<Payload, _>(
            name,
            config(3),
            IdentityCodec::<Payload>::new()
        ),
        Err(ShmError::EndpointOccupied {
            role: EndpointRole::Publisher,
            ..
        })
    ));
    assert!(subscriber.try_read_latest().is_none());
    child.kill().unwrap();
    assert!(!wait_for_exit(&mut child).success());
    assert!(subscriber.try_read_latest().is_none());
    let mut replacement = namespace
        .attach_latest_publisher(name, config(3), IdentityCodec::<Payload>::new())
        .unwrap();
    assert_eq!(replacement.diagnostics().reclaimed_endpoints, 1);
    assert_eq!(
        replacement.try_publish(Payload {
            first: 7,
            second: 7
        }),
        middleware_api::LatestPublish::Published {
            generation: 1,
            overwritten: false,
        }
    );
    assert_eq!(
        subscriber.try_read_latest().unwrap().value,
        Payload {
            first: 7,
            second: 7
        }
    );
}
