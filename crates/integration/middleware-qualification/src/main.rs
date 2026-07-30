//! Bounded host/target evidence runner for local and shared-memory middleware.

use std::{
    fmt::Write as _,
    fs::{self, OpenOptions},
    io::{self, Write as _},
    path::{Path, PathBuf},
    process::{Child, Command, ExitCode, Stdio},
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
    thread,
    time::{Duration, Instant},
};

use allocation_counter::{AllocationInfo, measure};
use bytemuck::{Pod, Zeroable};
use clap::{Parser, ValueEnum};
use middleware_api::{
    ChannelCounters, EventPublishError, EventPublisher, EventSubscriber, LatestPublish,
    LatestPublisher, LatestSubscriber,
};
use middleware_local::{latest_channel, ordered_channel};
use middleware_shm::{
    ChannelName, IdentityCodec, MemoryPolicy, ShmChannelConfig, ShmNamespace, ShmPayload,
};
use rustix::time::{ClockId, clock_gettime};
use tempfile::TempDir;

const MAX_WARMUP: u64 = 1_000_000;
const MAX_SAMPLES: u64 = 10_000_000;
const PEER_TIMEOUT: Duration = Duration::from_secs(10);

#[derive(Clone, Copy, Debug, ValueEnum)]
enum Backend {
    Local,
    Shm,
}

#[derive(Clone, Copy, Debug, ValueEnum)]
enum DeliveryKind {
    Latest,
    Ordered,
}

#[derive(Clone, Copy, Debug, ValueEnum)]
enum PeerMode {
    Thread,
    Process,
}

#[derive(Clone, Copy, Debug, ValueEnum)]
enum MemorySetting {
    Prefault,
    LockRequired,
}

#[derive(Debug, Parser)]
#[command(about = "Emit bounded middleware qualification evidence as JSON")]
struct Args {
    #[arg(long, value_enum, default_value = "local")]
    backend: Backend,
    #[arg(long, value_enum, default_value = "latest")]
    kind: DeliveryKind,
    #[arg(long, value_enum, default_value = "thread")]
    peer: PeerMode,
    #[arg(long, default_value_t = 64)]
    payload_bytes: u16,
    #[arg(long, default_value_t = 64)]
    capacity: u32,
    #[arg(long, value_enum, default_value = "prefault")]
    memory_policy: MemorySetting,
    #[arg(long, default_value_t = 1_000)]
    warmup: u64,
    #[arg(long, default_value_t = 10_000)]
    samples: u64,
    #[arg(long)]
    output: Option<PathBuf>,
    #[arg(long, hide = true)]
    worker: bool,
    #[arg(long, hide = true)]
    namespace: Option<PathBuf>,
    #[arg(long, hide = true)]
    channel: Option<String>,
    #[arg(long, hide = true)]
    ready: Option<PathBuf>,
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Payload64([u64; 8]);

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Payload256([u64; 32]);

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Payload1024([u64; 128]);

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct AckPayload([u64; 7]);

macro_rules! payload_contract {
    ($type:ty, $schema:literal) => {
        impl ShmPayload for $type {
            const SCHEMA_ID: [u8; 16] = *$schema;
            const SCHEMA_VERSION: u32 = 1;
        }
    };
}

payload_contract!(Payload64, b"qual-payload-064");
payload_contract!(Payload256, b"qual-payload-256");
payload_contract!(Payload1024, b"qual-payld-v1024");

impl ShmPayload for AckPayload {
    const SCHEMA_ID: [u8; 16] = *b"qual-ack-v000001";
    const SCHEMA_VERSION: u32 = 3;
}

trait BenchPayload: ShmPayload + Copy {
    fn sequence(self) -> u64;
    fn sent_mono_ns(self) -> u64;
    fn with_timing(sequence: u64, sent_mono_ns: u64) -> Self;
}

macro_rules! bench_payload {
    ($type:ty) => {
        impl BenchPayload for $type {
            fn sequence(self) -> u64 {
                self.0[0]
            }

            fn sent_mono_ns(self) -> u64 {
                self.0[1]
            }

            fn with_timing(sequence: u64, sent_mono_ns: u64) -> Self {
                let mut value = Self::zeroed();
                value.0[0] = sequence;
                value.0[1] = sent_mono_ns;
                value
            }
        }
    };
}

bench_payload!(Payload64);
bench_payload!(Payload256);
bench_payload!(Payload1024);

struct Histogram {
    buckets: [u64; 64],
    min_ns: u64,
    max_ns: u64,
}

impl Default for Histogram {
    fn default() -> Self {
        Self {
            buckets: [0; 64],
            min_ns: u64::MAX,
            max_ns: 0,
        }
    }
}

impl Histogram {
    fn record(&mut self, elapsed: Duration) {
        let nanos = elapsed.as_nanos().min(u128::from(u64::MAX)) as u64;
        let bucket = if nanos == 0 {
            0
        } else {
            (u64::BITS - nanos.leading_zeros()) as usize
        };
        self.buckets[bucket.min(self.buckets.len() - 1)] += 1;
        self.min_ns = self.min_ns.min(nanos);
        self.max_ns = self.max_ns.max(nanos);
    }
}

struct Evidence {
    latency: Histogram,
    operation: Histogram,
    allocations: AllocationInfo,
    counters: ChannelCounters,
    wall_ns: u64,
    cpu_ns: u64,
    peer_cpu_ns: u64,
    peak_resident_bytes: u64,
    peer_peak_resident_bytes: u64,
}

fn main() -> ExitCode {
    match run(Args::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("middleware-qualification: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run(args: Args) -> Result<(), String> {
    validate(&args)?;
    if args.worker {
        return dispatch_worker(&args);
    }
    let cpu_before = process_cpu_ns();
    let wall_before = Instant::now();
    let evidence = dispatch_run(&args)?;
    let wall_ns = nanos(wall_before.elapsed());
    let cpu_ns = process_cpu_ns()
        .checked_sub(cpu_before)
        .expect("CLOCK_PROCESS_CPUTIME_ID moved backwards");
    let evidence = Evidence {
        wall_ns,
        cpu_ns,
        peak_resident_bytes: peak_resident_bytes()?,
        ..evidence
    };
    let json = render_json(&args, &evidence);
    write_output(args.output.as_deref(), json.as_bytes())
}

fn validate(args: &Args) -> Result<(), String> {
    if args.capacity == 0 || args.capacity > 65_536 {
        return Err("capacity must be in 1..=65536".into());
    }
    if matches!(args.kind, DeliveryKind::Latest) && args.capacity < 3 {
        return Err("latest SHM-compatible capacity must be at least 3".into());
    }
    if args.warmup > MAX_WARMUP {
        return Err(format!("warmup exceeds {MAX_WARMUP}"));
    }
    if args.samples == 0 || args.samples > MAX_SAMPLES {
        return Err(format!("samples must be in 1..={MAX_SAMPLES}"));
    }
    if !matches!(args.payload_bytes, 64 | 256 | 1024) {
        return Err("payload-bytes must be one of 64, 256, 1024".into());
    }
    if matches!(args.backend, Backend::Local) && matches!(args.peer, PeerMode::Process) {
        return Err("local backend does not cross a process boundary".into());
    }
    Ok(())
}

fn dispatch_run(args: &Args) -> Result<Evidence, String> {
    match args.payload_bytes {
        64 => run_payload::<Payload64>(args),
        256 => run_payload::<Payload256>(args),
        1024 => run_payload::<Payload1024>(args),
        _ => unreachable!("validated payload size"),
    }
}

fn run_payload<P: BenchPayload>(args: &Args) -> Result<Evidence, String> {
    match (args.backend, args.peer, args.kind) {
        (Backend::Local, PeerMode::Thread, DeliveryKind::Latest) => run_local_latest::<P>(args),
        (Backend::Local, PeerMode::Thread, DeliveryKind::Ordered) => run_local_ordered::<P>(args),
        (Backend::Shm, PeerMode::Thread, DeliveryKind::Latest) => run_shm_thread::<P>(args, true),
        (Backend::Shm, PeerMode::Thread, DeliveryKind::Ordered) => run_shm_thread::<P>(args, false),
        (Backend::Shm, PeerMode::Process, _) => run_shm_process::<P>(args),
        (Backend::Local, PeerMode::Process, _) => unreachable!("validated peer mode"),
    }
}

fn run_local_latest<P: BenchPayload>(args: &Args) -> Result<Evidence, String> {
    let (mut publisher, subscriber) = latest_channel::<P>();
    let (latency, operation, allocations) = paired_thread_measure(
        args,
        subscriber,
        |value| match publisher.try_publish(value) {
            LatestPublish::Published { .. } => Ok(()),
            LatestPublish::Disconnected => Err("local latest thread peer disconnected"),
        },
        |subscriber| subscriber.try_read_latest().map(|sample| sample.value),
    )?;
    Ok(measured(
        latency,
        operation,
        allocations,
        publisher.counters(),
    ))
}

fn run_local_ordered<P: BenchPayload>(args: &Args) -> Result<Evidence, String> {
    let (mut publisher, subscriber) = ordered_channel::<P>(args.capacity as usize)
        .map_err(|error| format!("create local ordered channel: {error}"))?;
    let (latency, operation, allocations) = paired_thread_measure(
        args,
        subscriber,
        |value| {
            publisher
                .try_publish(value)
                .map(|_| ())
                .map_err(|_| "local ordered thread peer publish failed")
        },
        |subscriber| subscriber.try_receive().map(|sample| sample.value),
    )?;
    Ok(measured(
        latency,
        operation,
        allocations,
        publisher.counters(),
    ))
}

fn run_shm_thread<P: BenchPayload>(args: &Args, latest: bool) -> Result<Evidence, String> {
    let temp = TempDir::new().map_err(|error| format!("create temp namespace: {error}"))?;
    let namespace = ShmNamespace::new(temp.path().join("shm")).map_err(string_error)?;
    let name = ChannelName::new("qualification").map_err(string_error)?;
    let config = shm_config(args)?;
    if latest {
        namespace
            .create_latest::<P>(name, config)
            .map_err(string_error)?;
        let mut publisher = namespace
            .attach_latest_publisher(name, config, IdentityCodec::<P>::new())
            .map_err(string_error)?;
        let subscriber = namespace
            .attach_latest_subscriber(name, config, IdentityCodec::<P>::new())
            .map_err(string_error)?;
        let (latency, operation, allocations) = paired_thread_measure(
            args,
            subscriber,
            |value| match publisher.try_publish(value) {
                LatestPublish::Published { .. } => Ok(()),
                LatestPublish::Disconnected => Err("SHM latest thread peer disconnected"),
            },
            |subscriber| subscriber.try_read_latest().map(|sample| sample.value),
        )?;
        Ok(measured(
            latency,
            operation,
            allocations,
            publisher.counters(),
        ))
    } else {
        namespace
            .create_ordered::<P>(name, config)
            .map_err(string_error)?;
        let mut publisher = namespace
            .attach_event_publisher(name, config, IdentityCodec::<P>::new())
            .map_err(string_error)?;
        let subscriber = namespace
            .attach_event_subscriber(name, config, IdentityCodec::<P>::new())
            .map_err(string_error)?;
        let (latency, operation, allocations) = paired_thread_measure(
            args,
            subscriber,
            |value| {
                publisher
                    .try_publish(value)
                    .map(|_| ())
                    .map_err(|_| "SHM ordered thread peer publish failed")
            },
            |subscriber| subscriber.try_receive().map(|sample| sample.value),
        )?;
        Ok(measured(
            latency,
            operation,
            allocations,
            publisher.counters(),
        ))
    }
}

fn paired_thread_measure<P, S>(
    args: &Args,
    subscriber: S,
    mut publish: impl FnMut(P) -> Result<(), &'static str>,
    mut receive: impl FnMut(&mut S) -> Option<P> + Send,
) -> Result<(Histogram, Histogram, AllocationInfo), String>
where
    P: BenchPayload,
    S: Send,
{
    let progress = AtomicU64::new(0);
    let aborted = AtomicBool::new(false);
    let total = args.warmup.saturating_add(args.samples);
    thread::scope(|scope| {
        let progress_ref = &progress;
        let aborted_ref = &aborted;
        let consumer = scope.spawn(move || {
            let mut subscriber = subscriber;
            let mut latency = Histogram::default();
            let mut consume_one = |expected: u64, record_latency: bool| {
                let deadline = Instant::now() + PEER_TIMEOUT;
                let value = loop {
                    if aborted_ref.load(Ordering::Acquire) {
                        return Err("thread peer aborted");
                    }
                    if let Some(value) = receive(&mut subscriber) {
                        break value;
                    }
                    if Instant::now() >= deadline {
                        aborted_ref.store(true, Ordering::Release);
                        return Err("thread peer receive timeout");
                    }
                    thread::yield_now();
                };
                if value.sequence() != expected {
                    aborted_ref.store(true, Ordering::Release);
                    return Err("thread peer sequence mismatch");
                }
                if record_latency {
                    latency.record(Duration::from_nanos(one_way_ns(value)));
                }
                progress_ref.store(expected + 1, Ordering::Release);
                Ok(())
            };
            for expected in 0..args.warmup {
                consume_one(expected, false)?;
            }
            let mut measurement_error = None;
            let allocations = measure(|| {
                for expected in args.warmup..total {
                    if let Err(error) = consume_one(expected, true) {
                        measurement_error = Some(error);
                        break;
                    }
                }
            });
            if let Some(error) = measurement_error {
                return Err(error);
            }
            Ok((latency, allocations))
        });

        for sequence in 0..args.warmup {
            publish(P::with_timing(sequence, monotonic_ns())).map_err(str::to_owned)?;
            wait_thread_progress(&progress, &aborted, sequence + 1).map_err(str::to_owned)?;
        }

        let mut operation = Histogram::default();
        let mut measurement_error = None;
        let allocations = measure(|| {
            for sequence in args.warmup..total {
                let value = P::with_timing(sequence, monotonic_ns());
                let operation_start = Instant::now();
                if let Err(error) = publish(value) {
                    aborted.store(true, Ordering::Release);
                    measurement_error = Some(error);
                    break;
                }
                operation.record(operation_start.elapsed());
                if let Err(error) = wait_thread_progress(&progress, &aborted, sequence + 1) {
                    measurement_error = Some(error);
                    break;
                }
            }
        });
        if let Some(error) = measurement_error {
            return Err(error.to_owned());
        }
        let (latency, consumer_allocations) = consumer
            .join()
            .map_err(|_| "thread peer panicked".to_owned())?
            .map_err(str::to_owned)?;
        let mut allocations = allocations;
        allocations += consumer_allocations;
        Ok((latency, operation, allocations))
    })
}

fn wait_thread_progress(
    progress: &AtomicU64,
    aborted: &AtomicBool,
    expected: u64,
) -> Result<(), &'static str> {
    let deadline = Instant::now() + PEER_TIMEOUT;
    loop {
        if progress.load(Ordering::Acquire) == expected {
            return Ok(());
        }
        if aborted.load(Ordering::Acquire) {
            return Err("thread peer failed");
        }
        if Instant::now() >= deadline {
            aborted.store(true, Ordering::Release);
            return Err("thread peer acknowledgement timeout");
        }
        thread::yield_now();
    }
}

fn run_shm_process<P: BenchPayload>(args: &Args) -> Result<Evidence, String> {
    let temp = TempDir::new().map_err(|error| format!("create temp namespace: {error}"))?;
    let root = temp.path().join("shm");
    let ready = temp.path().join("ready");
    let namespace = ShmNamespace::new(&root).map_err(string_error)?;
    let name = ChannelName::new("qualification").map_err(string_error)?;
    let ack_name = ChannelName::new("qualification-ack").map_err(string_error)?;
    let config = shm_config(args)?;
    let ack_config = ShmChannelConfig::new(2, shm_memory_policy(args)).map_err(string_error)?;
    match args.kind {
        DeliveryKind::Latest => namespace
            .create_latest::<P>(name, config)
            .map_err(string_error)?,
        DeliveryKind::Ordered => namespace
            .create_ordered::<P>(name, config)
            .map_err(string_error)?,
    }
    namespace
        .create_ordered::<AckPayload>(ack_name, ack_config)
        .map_err(string_error)?;
    let mut ack_subscriber = namespace
        .attach_event_subscriber(ack_name, ack_config, IdentityCodec::<AckPayload>::new())
        .map_err(string_error)?;
    let mut child = spawn_worker::<P>(args, &root, name, &ready)?;
    wait_file(&ready, &mut child)?;

    let mut result = match args.kind {
        DeliveryKind::Latest => {
            let mut publisher = namespace
                .attach_latest_publisher(name, config, IdentityCodec::<P>::new())
                .map_err(string_error)?;
            let mut latency = Histogram::default();
            let mut operation = Histogram::default();
            for sequence in 0..args.warmup {
                publish_latest_and_wait(&mut publisher, &mut ack_subscriber, sequence)?;
            }
            let allocations = measure(|| {
                for sequence in 0..args.samples {
                    let (operation_elapsed, one_way_ns) =
                        publish_latest_and_wait(&mut publisher, &mut ack_subscriber, sequence)
                            .expect("live process peer");
                    operation.record(operation_elapsed);
                    latency.record(Duration::from_nanos(one_way_ns));
                }
            });
            measured(latency, operation, allocations, publisher.counters())
        }
        DeliveryKind::Ordered => {
            let mut publisher = namespace
                .attach_event_publisher(name, config, IdentityCodec::<P>::new())
                .map_err(string_error)?;
            let mut latency = Histogram::default();
            let mut operation = Histogram::default();
            for sequence in 0..args.warmup {
                publish_ordered_and_wait(&mut publisher, &mut ack_subscriber, sequence)?;
            }
            let allocations = measure(|| {
                for sequence in 0..args.samples {
                    let (operation_elapsed, one_way_ns) =
                        publish_ordered_and_wait(&mut publisher, &mut ack_subscriber, sequence)
                            .expect("live process peer");
                    operation.record(operation_elapsed);
                    latency.record(Duration::from_nanos(one_way_ns));
                }
            });
            measured(latency, operation, allocations, publisher.counters())
        }
    };
    let peer_summary = wait_ack_payload(&mut ack_subscriber, u64::MAX)?;
    result.peer_cpu_ns = peer_summary.0[1];
    result.peer_peak_resident_bytes = peer_summary.0[2];
    result.allocations += AllocationInfo {
        count_total: peer_summary.0[3],
        count_current: 0,
        count_max: peer_summary.0[5],
        bytes_total: peer_summary.0[4],
        bytes_current: 0,
        bytes_max: peer_summary.0[6],
    };
    wait_child(&mut child)?;
    Ok(result)
}

fn publish_latest_and_wait<P: BenchPayload>(
    publisher: &mut impl LatestPublisher<P>,
    ack: &mut impl EventSubscriber<AckPayload>,
    sequence: u64,
) -> Result<(Duration, u64), String> {
    let value = P::with_timing(sequence, monotonic_ns());
    let operation_start = Instant::now();
    if !matches!(
        publisher.try_publish(value),
        LatestPublish::Published { .. }
    ) {
        return Err("latest process peer disconnected".into());
    }
    let operation = operation_start.elapsed();
    let one_way_ns = wait_ack(ack, sequence)?;
    Ok((operation, one_way_ns))
}

fn publish_ordered_and_wait<P: BenchPayload>(
    publisher: &mut impl EventPublisher<P>,
    ack: &mut impl EventSubscriber<AckPayload>,
    sequence: u64,
) -> Result<(Duration, u64), String> {
    let value = P::with_timing(sequence, monotonic_ns());
    let operation_start = Instant::now();
    publisher.try_publish(value).map_err(|error| match error {
        EventPublishError::Full(_) => "ordered process channel unexpectedly full".to_owned(),
        EventPublishError::Disconnected(_) => "ordered process peer disconnected".to_owned(),
    })?;
    let operation = operation_start.elapsed();
    let one_way_ns = wait_ack(ack, sequence)?;
    Ok((operation, one_way_ns))
}

fn wait_ack(
    subscriber: &mut impl EventSubscriber<AckPayload>,
    sequence: u64,
) -> Result<u64, String> {
    Ok(wait_ack_payload(subscriber, sequence)?.0[1])
}

fn wait_ack_payload(
    subscriber: &mut impl EventSubscriber<AckPayload>,
    sequence: u64,
) -> Result<AckPayload, String> {
    let deadline = Instant::now() + PEER_TIMEOUT;
    loop {
        if let Some(sample) = subscriber.try_receive() {
            if sample.value.0[0] != sequence {
                return Err("process peer acknowledgement sequence mismatch".into());
            }
            return Ok(sample.value);
        }
        if Instant::now() >= deadline {
            return Err("process peer acknowledgement timeout".into());
        }
        thread::yield_now();
    }
}

fn dispatch_worker(args: &Args) -> Result<(), String> {
    match args.payload_bytes {
        64 => run_worker::<Payload64>(args),
        256 => run_worker::<Payload256>(args),
        1024 => run_worker::<Payload1024>(args),
        _ => unreachable!("validated payload size"),
    }
}

fn run_worker<P: BenchPayload>(args: &Args) -> Result<(), String> {
    let root = args.namespace.as_ref().ok_or("worker namespace missing")?;
    let name = ChannelName::new(args.channel.as_deref().ok_or("worker channel missing")?)
        .map_err(string_error)?;
    let ready = args.ready.as_ref().ok_or("worker ready path missing")?;
    let namespace = ShmNamespace::new(root).map_err(string_error)?;
    let config = shm_config(args)?;
    let ack_config = ShmChannelConfig::new(2, shm_memory_policy(args)).map_err(string_error)?;
    let ack_name = ChannelName::new("qualification-ack").map_err(string_error)?;
    let mut ack = namespace
        .attach_event_publisher(ack_name, ack_config, IdentityCodec::<AckPayload>::new())
        .map_err(string_error)?;
    match args.kind {
        DeliveryKind::Latest => {
            let mut subscriber = namespace
                .attach_latest_subscriber(name, config, IdentityCodec::<P>::new())
                .map_err(string_error)?;
            fs::write(ready, b"ready").map_err(|error| format!("write ready: {error}"))?;
            worker_loop(
                args,
                || subscriber.try_read_latest().map(|sample| sample.value),
                &mut ack,
            )
        }
        DeliveryKind::Ordered => {
            let mut subscriber = namespace
                .attach_event_subscriber(name, config, IdentityCodec::<P>::new())
                .map_err(string_error)?;
            fs::write(ready, b"ready").map_err(|error| format!("write ready: {error}"))?;
            worker_loop(
                args,
                || subscriber.try_receive().map(|sample| sample.value),
                &mut ack,
            )
        }
    }
}

fn worker_loop<P: BenchPayload>(
    args: &Args,
    mut receive: impl FnMut() -> Option<P>,
    ack: &mut impl EventPublisher<AckPayload>,
) -> Result<(), String> {
    let cpu_before = process_cpu_ns();
    for _ in 0..args.warmup {
        worker_receive_and_ack(&mut receive, ack)?;
    }
    let mut measurement_error = None;
    let allocations = measure(|| {
        for _ in 0..args.samples {
            if let Err(error) = worker_receive_and_ack(&mut receive, ack) {
                measurement_error = Some(error);
                break;
            }
        }
    });
    if let Some(error) = measurement_error {
        return Err(error);
    }
    if allocations.count_current != 0 || allocations.bytes_current != 0 {
        return Err("worker hot path retained an allocation".into());
    }
    let cpu_ns = process_cpu_ns()
        .checked_sub(cpu_before)
        .expect("CLOCK_PROCESS_CPUTIME_ID moved backwards");
    ack.try_publish(AckPayload([
        u64::MAX,
        cpu_ns,
        peak_resident_bytes()?,
        allocations.count_total,
        allocations.bytes_total,
        allocations.count_max,
        allocations.bytes_max,
    ]))
    .map_err(|_| "worker CPU acknowledgement backpressure")?;
    Ok(())
}

fn worker_receive_and_ack<P: BenchPayload>(
    receive: &mut impl FnMut() -> Option<P>,
    ack: &mut impl EventPublisher<AckPayload>,
) -> Result<(), String> {
    let deadline = Instant::now() + PEER_TIMEOUT;
    let value = loop {
        if let Some(value) = receive() {
            break value;
        }
        if Instant::now() >= deadline {
            return Err("worker receive timeout".into());
        }
        thread::yield_now();
    };
    ack.try_publish(AckPayload([
        value.sequence(),
        one_way_ns(value),
        0,
        0,
        0,
        0,
        0,
    ]))
    .map(|_| ())
    .map_err(|_| "worker acknowledgement backpressure".into())
}

fn spawn_worker<P: BenchPayload>(
    args: &Args,
    root: &Path,
    name: ChannelName,
    ready: &Path,
) -> Result<Child, String> {
    let executable =
        std::env::current_exe().map_err(|error| format!("locate executable: {error}"))?;
    Command::new(executable)
        .arg("--worker")
        .arg("--backend=shm")
        .arg(format!("--kind={}", kind_name(args.kind)))
        .arg("--peer=process")
        .arg(format!("--payload-bytes={}", std::mem::size_of::<P>()))
        .arg(format!("--capacity={}", args.capacity))
        .arg(format!(
            "--memory-policy={}",
            memory_policy_name(args.memory_policy)
        ))
        .arg(format!("--warmup={}", args.warmup))
        .arg(format!("--samples={}", args.samples))
        .arg("--namespace")
        .arg(root)
        .arg("--channel")
        .arg(name.as_str())
        .arg("--ready")
        .arg(ready)
        .stdin(Stdio::null())
        .stdout(Stdio::null())
        .stderr(Stdio::inherit())
        .spawn()
        .map_err(|error| format!("spawn worker: {error}"))
}

fn wait_file(path: &Path, child: &mut Child) -> Result<(), String> {
    let deadline = Instant::now() + PEER_TIMEOUT;
    loop {
        if path.exists() {
            return Ok(());
        }
        if let Some(status) = child
            .try_wait()
            .map_err(|error| format!("poll worker: {error}"))?
        {
            return Err(format!("worker exited before ready: {status}"));
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let _ = child.wait();
            return Err("worker readiness timeout".into());
        }
        thread::sleep(Duration::from_millis(10));
    }
}

fn wait_child(child: &mut Child) -> Result<(), String> {
    let deadline = Instant::now() + PEER_TIMEOUT;
    loop {
        if let Some(status) = child
            .try_wait()
            .map_err(|error| format!("poll worker: {error}"))?
        {
            return if status.success() {
                Ok(())
            } else {
                Err(format!("worker failed: {status}"))
            };
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let _ = child.wait();
            return Err("worker exit timeout".into());
        }
        thread::sleep(Duration::from_millis(10));
    }
}

fn measured(
    latency: Histogram,
    operation: Histogram,
    allocations: AllocationInfo,
    counters: ChannelCounters,
) -> Evidence {
    Evidence {
        latency,
        operation,
        allocations,
        counters,
        wall_ns: 0,
        cpu_ns: 0,
        peer_cpu_ns: 0,
        peak_resident_bytes: 0,
        peer_peak_resident_bytes: 0,
    }
}

fn process_cpu_ns() -> u64 {
    timespec_ns(
        clock_gettime(ClockId::ProcessCPUTime),
        "CLOCK_PROCESS_CPUTIME_ID",
    )
}

fn peak_resident_bytes() -> Result<u64, String> {
    let status = fs::read_to_string("/proc/self/status")
        .map_err(|error| format!("read /proc/self/status: {error}"))?;
    let high_water_kib = status
        .lines()
        .find_map(|line| line.strip_prefix("VmHWM:"))
        .ok_or("missing VmHWM in /proc/self/status")?;
    let mut fields = high_water_kib.split_whitespace();
    let value: u64 = fields
        .next()
        .ok_or("missing VmHWM value")?
        .parse()
        .map_err(|error| format!("parse VmHWM: {error}"))?;
    if fields.next() != Some("kB") || fields.next().is_some() {
        return Err("invalid VmHWM unit".into());
    }
    value
        .checked_mul(1024)
        .ok_or_else(|| "VmHWM byte count overflow".into())
}

fn render_json(args: &Args, evidence: &Evidence) -> String {
    let latency_buckets = render_buckets(&evidence.latency);
    let operation_buckets = render_buckets(&evidence.operation);
    format!(
        concat!(
            "{{\"schema_version\":2,\"qualification\":\"host_research\",",
            "\"runner_version\":\"{}\",\"target_os\":\"{}\",\"target_arch\":\"{}\",",
            "\"backend\":\"{}\",\"kind\":\"{}\",\"peer\":\"{}\",\"peer_execution\":\"{}\",",
            "\"payload_bytes\":{},\"capacity\":{},",
            "\"requested_memory_policy\":\"{}\",\"effective_memory_policy\":\"{}\",",
            "\"memory_policy_scope\":\"shm_mappings\",",
            "\"warmup\":{},\"samples\":{},",
            "\"latency_semantics\":\"one_way_receive\",\"latency_clock\":\"CLOCK_MONOTONIC\",",
            "\"latency_log2_ns\":[{}],\"latency_min_ns\":{},\"latency_max_ns\":{},",
            "\"latency_jitter_semantics\":\"max_minus_min\",\"latency_jitter_span_ns\":{},",
            "\"operation_semantics\":\"try_publish_call\",\"operation_clock\":\"CLOCK_MONOTONIC\",",
            "\"operation_log2_ns\":[{}],\"operation_max_ns\":{},",
            "\"wall_time_ns\":{},\"cpu_time_ns\":{},\"peer_cpu_time_ns\":{},",
            "\"cpu_time_source\":\"CLOCK_PROCESS_CPUTIME_ID\",",
            "\"peak_resident_bytes\":{},\"peer_peak_resident_bytes\":{},",
            "\"memory_source\":\"proc_self_status_VmHWM\",",
            "\"allocation_scope\":\"publisher_and_consumer\",",
            "\"allocations\":{{\"count\":{},\"bytes\":{},\"current_count\":{},",
            "\"current_bytes\":{},\"peak_count\":{},\"peak_bytes\":{}}},",
            "\"counters\":{{\"published\":{},\"consumed\":{},\"overwritten\":{},",
            "\"full\":{},\"disconnected\":{}}}}}\n"
        ),
        env!("CARGO_PKG_VERSION"),
        std::env::consts::OS,
        std::env::consts::ARCH,
        backend_name(args.backend),
        kind_name(args.kind),
        peer_name(args.peer),
        peer_execution_name(args.peer),
        args.payload_bytes,
        args.capacity,
        memory_policy_name(args.memory_policy),
        effective_memory_policy_name(args),
        args.warmup,
        args.samples,
        latency_buckets,
        evidence.latency.min_ns,
        evidence.latency.max_ns,
        evidence
            .latency
            .max_ns
            .saturating_sub(evidence.latency.min_ns),
        operation_buckets,
        evidence.operation.max_ns,
        evidence.wall_ns,
        evidence.cpu_ns,
        evidence.peer_cpu_ns,
        evidence.peak_resident_bytes,
        evidence.peer_peak_resident_bytes,
        evidence.allocations.count_total,
        evidence.allocations.bytes_total,
        evidence.allocations.count_current,
        evidence.allocations.bytes_current,
        evidence.allocations.count_max,
        evidence.allocations.bytes_max,
        evidence.counters.published,
        evidence.counters.consumed,
        evidence.counters.overwritten,
        evidence.counters.full,
        evidence.counters.disconnected,
    )
}

fn render_buckets(histogram: &Histogram) -> String {
    let mut buckets = String::new();
    for (index, count) in histogram.buckets.iter().enumerate() {
        if index != 0 {
            buckets.push(',');
        }
        write!(buckets, "{count}").expect("String writes cannot fail");
    }
    buckets
}

fn write_output(path: Option<&Path>, bytes: &[u8]) -> Result<(), String> {
    if let Some(path) = path {
        let mut file = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .map_err(|error| format!("create output exclusively: {error}"))?;
        file.write_all(bytes)
            .map_err(|error| format!("write output: {error}"))
    } else {
        io::stdout()
            .lock()
            .write_all(bytes)
            .map_err(|error| format!("write stdout: {error}"))
    }
}

fn backend_name(value: Backend) -> &'static str {
    match value {
        Backend::Local => "local",
        Backend::Shm => "shm",
    }
}

fn kind_name(value: DeliveryKind) -> &'static str {
    match value {
        DeliveryKind::Latest => "latest",
        DeliveryKind::Ordered => "ordered",
    }
}

fn peer_name(value: PeerMode) -> &'static str {
    match value {
        PeerMode::Thread => "thread",
        PeerMode::Process => "process",
    }
}

fn peer_execution_name(value: PeerMode) -> &'static str {
    match value {
        PeerMode::Thread => "separate_thread_paired",
        PeerMode::Process => "separate_process_paired",
    }
}

fn memory_policy_name(value: MemorySetting) -> &'static str {
    match value {
        MemorySetting::Prefault => "prefault",
        MemorySetting::LockRequired => "lock-required",
    }
}

fn effective_memory_policy_name(args: &Args) -> &'static str {
    match args.backend {
        Backend::Local => "not-applicable",
        Backend::Shm => memory_policy_name(args.memory_policy),
    }
}

fn shm_config(args: &Args) -> Result<ShmChannelConfig, String> {
    ShmChannelConfig::new(args.capacity, shm_memory_policy(args)).map_err(string_error)
}

fn shm_memory_policy(args: &Args) -> MemoryPolicy {
    match args.memory_policy {
        MemorySetting::Prefault => MemoryPolicy::Prefault,
        MemorySetting::LockRequired => MemoryPolicy::LockRequired,
    }
}

fn nanos(duration: Duration) -> u64 {
    duration.as_nanos().min(u128::from(u64::MAX)) as u64
}

fn monotonic_ns() -> u64 {
    timespec_ns(clock_gettime(ClockId::Monotonic), "CLOCK_MONOTONIC")
}

fn timespec_ns(now: rustix::time::Timespec, name: &str) -> u64 {
    let seconds =
        u64::try_from(now.tv_sec).unwrap_or_else(|_| panic!("{name} returned negative seconds"));
    let nanoseconds = u64::try_from(now.tv_nsec).expect("tv_nsec is nonnegative");
    seconds
        .checked_mul(1_000_000_000)
        .and_then(|value| value.checked_add(nanoseconds))
        .unwrap_or_else(|| panic!("{name} nanoseconds overflow u64"))
}

fn one_way_ns<P: BenchPayload>(value: P) -> u64 {
    monotonic_ns()
        .checked_sub(value.sent_mono_ns())
        .expect("CLOCK_MONOTONIC moved backwards")
}

fn string_error(error: impl std::fmt::Display) -> String {
    error.to_string()
}
