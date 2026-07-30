//! Bounded host/target evidence runner for local and shared-memory middleware.

use std::{
    fmt::Write as _,
    fs::{self, OpenOptions},
    io::{self, Write as _},
    path::{Path, PathBuf},
    process::{Child, Command, ExitCode, Stdio},
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
struct AckPayload([u64; 2]);

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
payload_contract!(AckPayload, b"qual-ack-v000001");

trait BenchPayload: ShmPayload + Copy {
    fn sequence(self) -> u64;
    fn with_sequence(sequence: u64) -> Self;
}

macro_rules! bench_payload {
    ($type:ty) => {
        impl BenchPayload for $type {
            fn sequence(self) -> u64 {
                self.0[0]
            }

            fn with_sequence(sequence: u64) -> Self {
                let mut value = Self::zeroed();
                value.0[0] = sequence;
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
    max_ns: u64,
}

impl Default for Histogram {
    fn default() -> Self {
        Self {
            buckets: [0; 64],
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
        self.max_ns = self.max_ns.max(nanos);
    }
}

struct Evidence {
    histogram: Histogram,
    operation_max_ns: u64,
    allocations: AllocationInfo,
    counters: ChannelCounters,
    wall_ns: u64,
    cpu_ns: u64,
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
    let cpu_before = process_cpu_ns()?;
    let wall_before = Instant::now();
    let evidence = dispatch_run(&args)?;
    let wall_ns = nanos(wall_before.elapsed());
    let cpu_ns = process_cpu_ns()?.saturating_sub(cpu_before);
    let evidence = Evidence {
        wall_ns,
        cpu_ns,
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
    let (mut publisher, mut subscriber) = latest_channel();
    for sequence in 0..args.warmup {
        let _ = publisher.try_publish(P::with_sequence(sequence));
        let _ = subscriber.try_read_latest();
    }
    let mut histogram = Histogram::default();
    let mut operation_max_ns = 0;
    let allocations = measure(|| {
        for sequence in 0..args.samples {
            let start = Instant::now();
            let operation_start = Instant::now();
            assert!(matches!(
                publisher.try_publish(P::with_sequence(sequence)),
                LatestPublish::Published { .. }
            ));
            operation_max_ns = operation_max_ns.max(nanos(operation_start.elapsed()));
            let sample = subscriber.try_read_latest().expect("paired latest sample");
            assert_eq!(sample.value.sequence(), sequence);
            histogram.record(start.elapsed());
        }
    });
    Ok(measured(
        histogram,
        operation_max_ns,
        allocations,
        publisher.counters(),
    ))
}

fn run_local_ordered<P: BenchPayload>(args: &Args) -> Result<Evidence, String> {
    let (mut publisher, mut subscriber) = ordered_channel(args.capacity as usize)
        .map_err(|error| format!("create local ordered channel: {error}"))?;
    for sequence in 0..args.warmup {
        publisher
            .try_publish(P::with_sequence(sequence))
            .map_err(|_| "unexpected local warmup backpressure")?;
        let _ = subscriber.try_receive();
    }
    let mut histogram = Histogram::default();
    let mut operation_max_ns = 0;
    let allocations = measure(|| {
        for sequence in 0..args.samples {
            let start = Instant::now();
            let operation_start = Instant::now();
            assert!(publisher.try_publish(P::with_sequence(sequence)).is_ok());
            operation_max_ns = operation_max_ns.max(nanos(operation_start.elapsed()));
            let sample = subscriber.try_receive().expect("paired ordered sample");
            assert_eq!(sample.value.sequence(), sequence);
            histogram.record(start.elapsed());
        }
    });
    Ok(measured(
        histogram,
        operation_max_ns,
        allocations,
        publisher.counters(),
    ))
}

fn run_shm_thread<P: BenchPayload>(args: &Args, latest: bool) -> Result<Evidence, String> {
    let temp = TempDir::new().map_err(|error| format!("create temp namespace: {error}"))?;
    let namespace = ShmNamespace::new(temp.path().join("shm")).map_err(string_error)?;
    let name = ChannelName::new("qualification").map_err(string_error)?;
    let config =
        ShmChannelConfig::new(args.capacity, MemoryPolicy::Prefault).map_err(string_error)?;
    if latest {
        namespace
            .create_latest::<P>(name, config)
            .map_err(string_error)?;
        let mut publisher = namespace
            .attach_latest_publisher(name, config, IdentityCodec::<P>::new())
            .map_err(string_error)?;
        let mut subscriber = namespace
            .attach_latest_subscriber(name, config, IdentityCodec::<P>::new())
            .map_err(string_error)?;
        for sequence in 0..args.warmup {
            let _ = publisher.try_publish(P::with_sequence(sequence));
            let _ = subscriber.try_read_latest();
        }
        let mut histogram = Histogram::default();
        let mut operation_max_ns = 0;
        let allocations = measure(|| {
            for sequence in 0..args.samples {
                let start = Instant::now();
                let operation_start = Instant::now();
                assert!(matches!(
                    publisher.try_publish(P::with_sequence(sequence)),
                    LatestPublish::Published { .. }
                ));
                operation_max_ns = operation_max_ns.max(nanos(operation_start.elapsed()));
                let sample = subscriber.try_read_latest().expect("paired latest sample");
                assert_eq!(sample.value.sequence(), sequence);
                histogram.record(start.elapsed());
            }
        });
        Ok(measured(
            histogram,
            operation_max_ns,
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
        let mut subscriber = namespace
            .attach_event_subscriber(name, config, IdentityCodec::<P>::new())
            .map_err(string_error)?;
        for sequence in 0..args.warmup {
            publisher
                .try_publish(P::with_sequence(sequence))
                .map_err(|_| "unexpected SHM warmup backpressure")?;
            let _ = subscriber.try_receive();
        }
        let mut histogram = Histogram::default();
        let mut operation_max_ns = 0;
        let allocations = measure(|| {
            for sequence in 0..args.samples {
                let start = Instant::now();
                let operation_start = Instant::now();
                assert!(publisher.try_publish(P::with_sequence(sequence)).is_ok());
                operation_max_ns = operation_max_ns.max(nanos(operation_start.elapsed()));
                let sample = subscriber.try_receive().expect("paired ordered sample");
                assert_eq!(sample.value.sequence(), sequence);
                histogram.record(start.elapsed());
            }
        });
        Ok(measured(
            histogram,
            operation_max_ns,
            allocations,
            publisher.counters(),
        ))
    }
}

fn run_shm_process<P: BenchPayload>(args: &Args) -> Result<Evidence, String> {
    let temp = TempDir::new().map_err(|error| format!("create temp namespace: {error}"))?;
    let root = temp.path().join("shm");
    let ready = temp.path().join("ready");
    let namespace = ShmNamespace::new(&root).map_err(string_error)?;
    let name = ChannelName::new("qualification").map_err(string_error)?;
    let ack_name = ChannelName::new("qualification-ack").map_err(string_error)?;
    let config =
        ShmChannelConfig::new(args.capacity, MemoryPolicy::Prefault).map_err(string_error)?;
    let ack_config = ShmChannelConfig::new(1, MemoryPolicy::Prefault).map_err(string_error)?;
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

    let result = match args.kind {
        DeliveryKind::Latest => {
            let mut publisher = namespace
                .attach_latest_publisher(name, config, IdentityCodec::<P>::new())
                .map_err(string_error)?;
            let mut histogram = Histogram::default();
            let mut operation_max_ns = 0;
            for sequence in 0..args.warmup {
                publish_latest_and_wait(&mut publisher, &mut ack_subscriber, sequence)?;
            }
            let allocations = measure(|| {
                for sequence in 0..args.samples {
                    let start = Instant::now();
                    let operation =
                        publish_latest_and_wait(&mut publisher, &mut ack_subscriber, sequence)
                            .expect("live process peer");
                    operation_max_ns = operation_max_ns.max(nanos(operation));
                    histogram.record(start.elapsed());
                }
            });
            measured(
                histogram,
                operation_max_ns,
                allocations,
                publisher.counters(),
            )
        }
        DeliveryKind::Ordered => {
            let mut publisher = namespace
                .attach_event_publisher(name, config, IdentityCodec::<P>::new())
                .map_err(string_error)?;
            let mut histogram = Histogram::default();
            let mut operation_max_ns = 0;
            for sequence in 0..args.warmup {
                publish_ordered_and_wait(&mut publisher, &mut ack_subscriber, sequence)?;
            }
            let allocations = measure(|| {
                for sequence in 0..args.samples {
                    let start = Instant::now();
                    let operation =
                        publish_ordered_and_wait(&mut publisher, &mut ack_subscriber, sequence)
                            .expect("live process peer");
                    operation_max_ns = operation_max_ns.max(nanos(operation));
                    histogram.record(start.elapsed());
                }
            });
            measured(
                histogram,
                operation_max_ns,
                allocations,
                publisher.counters(),
            )
        }
    };
    wait_child(&mut child)?;
    Ok(result)
}

fn publish_latest_and_wait<P: BenchPayload>(
    publisher: &mut impl LatestPublisher<P>,
    ack: &mut impl EventSubscriber<AckPayload>,
    sequence: u64,
) -> Result<Duration, String> {
    let operation_start = Instant::now();
    if !matches!(
        publisher.try_publish(P::with_sequence(sequence)),
        LatestPublish::Published { .. }
    ) {
        return Err("latest process peer disconnected".into());
    }
    let operation = operation_start.elapsed();
    wait_ack(ack, sequence)?;
    Ok(operation)
}

fn publish_ordered_and_wait<P: BenchPayload>(
    publisher: &mut impl EventPublisher<P>,
    ack: &mut impl EventSubscriber<AckPayload>,
    sequence: u64,
) -> Result<Duration, String> {
    let operation_start = Instant::now();
    publisher
        .try_publish(P::with_sequence(sequence))
        .map_err(|error| match error {
            EventPublishError::Full(_) => "ordered process channel unexpectedly full".to_owned(),
            EventPublishError::Disconnected(_) => "ordered process peer disconnected".to_owned(),
        })?;
    let operation = operation_start.elapsed();
    wait_ack(ack, sequence)?;
    Ok(operation)
}

fn wait_ack(
    subscriber: &mut impl EventSubscriber<AckPayload>,
    sequence: u64,
) -> Result<(), String> {
    let deadline = Instant::now() + PEER_TIMEOUT;
    loop {
        if let Some(sample) = subscriber.try_receive() {
            if sample.value.0[0] != sequence {
                return Err("process peer acknowledgement sequence mismatch".into());
            }
            return Ok(());
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
    let config =
        ShmChannelConfig::new(args.capacity, MemoryPolicy::Prefault).map_err(string_error)?;
    let ack_config = ShmChannelConfig::new(1, MemoryPolicy::Prefault).map_err(string_error)?;
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
    let total = args.warmup.saturating_add(args.samples);
    for _ in 0..total {
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
        ack.try_publish(AckPayload([value.sequence(), 0]))
            .map_err(|_| "worker acknowledgement backpressure")?;
    }
    Ok(())
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
    histogram: Histogram,
    operation_max_ns: u64,
    allocations: AllocationInfo,
    counters: ChannelCounters,
) -> Evidence {
    Evidence {
        histogram,
        operation_max_ns,
        allocations,
        counters,
        wall_ns: 0,
        cpu_ns: 0,
    }
}

fn process_cpu_ns() -> Result<u64, String> {
    let stat = fs::read_to_string("/proc/self/stat")
        .map_err(|error| format!("read /proc/self/stat: {error}"))?;
    let fields = stat
        .rsplit_once(')')
        .ok_or("invalid /proc/self/stat comm field")?
        .1
        .split_whitespace()
        .collect::<Vec<_>>();
    let user: u64 = fields
        .get(11)
        .ok_or("missing process user ticks")?
        .parse()
        .map_err(|error| format!("parse process user ticks: {error}"))?;
    let system: u64 = fields
        .get(12)
        .ok_or("missing process system ticks")?
        .parse()
        .map_err(|error| format!("parse process system ticks: {error}"))?;
    let output = Command::new("getconf")
        .arg("CLK_TCK")
        .output()
        .map_err(|error| format!("run getconf CLK_TCK: {error}"))?;
    if !output.status.success() {
        return Err("getconf CLK_TCK failed".into());
    }
    let ticks_per_second: u64 = String::from_utf8(output.stdout)
        .map_err(|error| format!("decode CLK_TCK: {error}"))?
        .trim()
        .parse()
        .map_err(|error| format!("parse CLK_TCK: {error}"))?;
    Ok(user.saturating_add(system).saturating_mul(1_000_000_000) / ticks_per_second)
}

fn render_json(args: &Args, evidence: &Evidence) -> String {
    let mut buckets = String::new();
    for (index, count) in evidence.histogram.buckets.iter().enumerate() {
        if index != 0 {
            buckets.push(',');
        }
        write!(buckets, "{count}").expect("String writes cannot fail");
    }
    format!(
        concat!(
            "{{\"schema_version\":1,\"qualification\":\"host_research\",",
            "\"runner_version\":\"{}\",\"target_os\":\"{}\",\"target_arch\":\"{}\",",
            "\"backend\":\"{}\",\"kind\":\"{}\",\"peer\":\"{}\",",
            "\"payload_bytes\":{},\"capacity\":{},\"warmup\":{},\"samples\":{},",
            "\"latency_log2_ns\":[{}],\"latency_max_ns\":{},\"operation_max_ns\":{},",
            "\"wall_time_ns\":{},\"cpu_time_ns\":{},\"cpu_time_source\":\"proc_self_stat\",",
            "\"allocations\":{{\"count\":{},\"bytes\":{},\"peak_count\":{},\"peak_bytes\":{}}},",
            "\"counters\":{{\"published\":{},\"consumed\":{},\"overwritten\":{},",
            "\"full\":{},\"disconnected\":{}}}}}\n"
        ),
        env!("CARGO_PKG_VERSION"),
        std::env::consts::OS,
        std::env::consts::ARCH,
        backend_name(args.backend),
        kind_name(args.kind),
        peer_name(args.peer),
        args.payload_bytes,
        args.capacity,
        args.warmup,
        args.samples,
        buckets,
        evidence.histogram.max_ns,
        evidence.operation_max_ns,
        evidence.wall_ns,
        evidence.cpu_ns,
        evidence.allocations.count_total,
        evidence.allocations.bytes_total,
        evidence.allocations.count_max,
        evidence.allocations.bytes_max,
        evidence.counters.published,
        evidence.counters.consumed,
        evidence.counters.overwritten,
        evidence.counters.full,
        evidence.counters.disconnected,
    )
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

fn nanos(duration: Duration) -> u64 {
    duration.as_nanos().min(u128::from(u64::MAX)) as u64
}

fn string_error(error: impl std::fmt::Display) -> String {
    error.to_string()
}
