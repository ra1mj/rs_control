//! Target-only qualification adapter for the pinned external Horus checkout.

use std::{
    alloc::{GlobalAlloc, Layout, System},
    collections::BTreeMap,
    env,
    fmt::Write as _,
    fs::{self, OpenOptions},
    io::{Read, Write as _},
    path::{Path, PathBuf},
    process::{Child, Command, ExitCode, Stdio},
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
    thread,
    time::{Duration, Instant},
};

use bytemuck::{Pod, Zeroable};
use horus_core::communication::Topic;
use serde::{Deserialize, Serialize};

const BUILT_HORUS_COMMIT: &str = env!("HORUS_SOURCE_COMMIT");
const MAX_WARMUP: u64 = 1_000_000;
const MAX_SAMPLES: u64 = 10_000_000;
const PROCESS_TIMEOUT: Duration = Duration::from_secs(10);
const TRY_SEND_BOUND_NS: u64 = 100_000_000;
const READY_ACK: u64 = u64::MAX;
const PRIME_SEQUENCE: u64 = u64::MAX - 1;

struct CountingAllocator;

static ALLOCATION_ARMED: AtomicBool = AtomicBool::new(false);
static ALLOCATION_COUNT: AtomicU64 = AtomicU64::new(0);
static ALLOCATION_BYTES: AtomicU64 = AtomicU64::new(0);

unsafe impl GlobalAlloc for CountingAllocator {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if ALLOCATION_ARMED.load(Ordering::Relaxed) {
            ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
            ALLOCATION_BYTES.fetch_add(layout.size() as u64, Ordering::Relaxed);
        }
        // SAFETY: forwarding the original layout to the system allocator.
        unsafe { System.alloc(layout) }
    }

    unsafe fn alloc_zeroed(&self, layout: Layout) -> *mut u8 {
        if ALLOCATION_ARMED.load(Ordering::Relaxed) {
            ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
            ALLOCATION_BYTES.fetch_add(layout.size() as u64, Ordering::Relaxed);
        }
        // SAFETY: forwarding the original layout to the system allocator.
        unsafe { System.alloc_zeroed(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        // SAFETY: forwarding the allocation and its original layout.
        unsafe { System.dealloc(ptr, layout) }
    }

    unsafe fn realloc(&self, ptr: *mut u8, layout: Layout, new_size: usize) -> *mut u8 {
        if ALLOCATION_ARMED.load(Ordering::Relaxed) {
            ALLOCATION_COUNT.fetch_add(1, Ordering::Relaxed);
            ALLOCATION_BYTES.fetch_add(new_size as u64, Ordering::Relaxed);
        }
        // SAFETY: forwarding the allocation, original layout and requested size.
        unsafe { System.realloc(ptr, layout, new_size) }
    }
}

#[global_allocator]
static GLOBAL_ALLOCATOR: CountingAllocator = CountingAllocator;

#[derive(Clone, Copy, Debug, Default)]
struct AllocationSnapshot {
    count: u64,
    bytes: u64,
}

fn arm_allocations() {
    ALLOCATION_COUNT.store(0, Ordering::Relaxed);
    ALLOCATION_BYTES.store(0, Ordering::Relaxed);
    ALLOCATION_ARMED.store(true, Ordering::SeqCst);
}

fn disarm_allocations() -> AllocationSnapshot {
    ALLOCATION_ARMED.store(false, Ordering::SeqCst);
    AllocationSnapshot {
        count: ALLOCATION_COUNT.load(Ordering::Relaxed),
        bytes: ALLOCATION_BYTES.load(Ordering::Relaxed),
    }
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable, Serialize, Deserialize)]
struct Payload64 {
    words: [u64; 8],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable, Serialize, Deserialize)]
struct Payload256 {
    words: [u64; 32],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable, Serialize, Deserialize)]
struct Payload1024 {
    words0: [u64; 32],
    words1: [u64; 32],
    words2: [u64; 32],
    words3: [u64; 32],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable, Serialize, Deserialize)]
struct Ack {
    words: [u64; 8],
}

trait BenchPayload:
    Pod + Zeroable + Copy + Clone + Send + Sync + Serialize + for<'de> Deserialize<'de> + 'static
{
    const BYTES: usize;

    fn words(&self) -> &[u64] {
        bytemuck::cast_slice(bytemuck::bytes_of(self))
    }

    fn words_mut(&mut self) -> &mut [u64] {
        bytemuck::cast_slice_mut(bytemuck::bytes_of_mut(self))
    }

    fn new(sequence: u64) -> Self {
        let mut value = Self::zeroed();
        let words = value.words_mut();
        words[0] = sequence;
        words[1] = 0;
        for (index, word) in words.iter_mut().enumerate().skip(2) {
            *word = pattern_word(sequence, index as u64);
        }
        value
    }

    fn set_sent_ns(&mut self, value: u64) {
        self.words_mut()[1] = value;
    }

    fn sequence(&self) -> u64 {
        self.words()[0]
    }

    fn sent_ns(&self) -> u64 {
        self.words()[1]
    }

    fn is_intact(&self) -> bool {
        let sequence = self.sequence();
        self.words()
            .iter()
            .enumerate()
            .skip(2)
            .all(|(index, word)| *word == pattern_word(sequence, index as u64))
    }
}

impl BenchPayload for Payload64 {
    const BYTES: usize = 64;
}
impl BenchPayload for Payload256 {
    const BYTES: usize = 256;
}
impl BenchPayload for Payload1024 {
    const BYTES: usize = 1024;
}

fn pattern_word(sequence: u64, index: u64) -> u64 {
    sequence.rotate_left((index & 63) as u32)
        ^ index.wrapping_mul(0x9e37_79b9_7f4a_7c15)
        ^ 0xa5a5_5a5a_d3c3_b4b4
}

struct Histogram {
    buckets: [u64; 64],
    min_ns: u64,
    max_ns: u64,
    initialized: bool,
}

impl Default for Histogram {
    fn default() -> Self {
        Self {
            buckets: [0; 64],
            min_ns: 0,
            max_ns: 0,
            initialized: false,
        }
    }
}

impl Histogram {
    fn record(&mut self, value: u64) {
        let bucket = if value == 0 {
            0
        } else {
            (u64::BITS - value.leading_zeros()) as usize
        };
        self.buckets[bucket.min(63)] += 1;
        if !self.initialized {
            self.min_ns = value;
            self.max_ns = value;
            self.initialized = true;
        } else {
            self.min_ns = self.min_ns.min(value);
            self.max_ns = self.max_ns.max(value);
        }
    }

    fn total(&self) -> u64 {
        self.buckets.iter().sum()
    }
}

#[derive(Debug)]
struct Flags(BTreeMap<String, String>);

impl Flags {
    fn parse(values: impl Iterator<Item = String>) -> Result<Self, String> {
        let mut values = values.peekable();
        let mut flags = BTreeMap::new();
        while let Some(flag) = values.next() {
            if !flag.starts_with("--") || flag == "--" {
                return Err(format!("unexpected positional argument: {flag}"));
            }
            let value = values
                .next()
                .ok_or_else(|| format!("{flag} requires a value"))?;
            if value.starts_with("--") {
                return Err(format!("{flag} requires a value"));
            }
            if flags.insert(flag.clone(), value).is_some() {
                return Err(format!("duplicate argument: {flag}"));
            }
        }
        Ok(Self(flags))
    }

    fn take(&mut self, name: &str) -> Result<String, String> {
        self.0
            .remove(name)
            .ok_or_else(|| format!("missing required argument: {name}"))
    }

    fn finish(self) -> Result<(), String> {
        if let Some(name) = self.0.keys().next() {
            Err(format!("unknown argument: {name}"))
        } else {
            Ok(())
        }
    }
}

struct CommonPublic {
    source_commit: String,
    capacity: u32,
    output: PathBuf,
}

struct BenchmarkArgs {
    common: CommonPublic,
    kind: String,
    payload_bytes: usize,
    warmup: u64,
    samples: u64,
    self_test: bool,
}

struct Evidence {
    latency: Histogram,
    operation: Histogram,
    allocations: AllocationSnapshot,
    published: u64,
    consumed: u64,
    wall_ns: u64,
    cpu_ns: u64,
    peer_cpu_ns: u64,
    peak_resident_bytes: u64,
    peer_peak_resident_bytes: u64,
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("horus-qualification-adapter: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> Result<(), String> {
    let mut args = env::args().skip(1);
    let command = args.next().ok_or_else(usage)?;
    let flags = Flags::parse(args)?;
    match command.as_str() {
        "preflight" => preflight(flags),
        "benchmark" => benchmark(flags),
        "fault-suite" => fault_suite(flags),
        "self-test" => self_test(flags),
        "__paired-child" => paired_child(flags),
        "__slow-child" => slow_child(flags),
        "__hold-child" => hold_child(flags),
        "__once-child" => once_child(flags),
        "help" | "--help" | "-h" => {
            println!("{}", usage());
            Ok(())
        }
        _ => Err(format!("unknown command: {command}\n{}", usage())),
    }
}

fn usage() -> String {
    "Usage: rs-control-horus-qualification-adapter preflight|benchmark|fault-suite|self-test [options]".to_string()
}

fn parse_common(flags: &mut Flags) -> Result<CommonPublic, String> {
    let source_commit = flags.take("--source-commit")?;
    if source_commit != BUILT_HORUS_COMMIT {
        return Err(format!(
            "source commit mismatch: binary={}, requested={source_commit}",
            BUILT_HORUS_COMMIT
        ));
    }
    let capacity = parse_u32(&flags.take("--capacity")?, "capacity")?;
    validate_capacity(capacity)?;
    let output = PathBuf::from(flags.take("--output")?);
    validate_output(&output)?;
    Ok(CommonPublic {
        source_commit,
        capacity,
        output,
    })
}

fn validate_contract_flags(flags: &mut Flags) -> Result<(), String> {
    require_value(flags, "--memory-policy", "lock-required")?;
    require_value(flags, "--payload-contract", "repr_c_bytemuck_pod")?;
    Ok(())
}

fn require_value(flags: &mut Flags, name: &str, expected: &str) -> Result<(), String> {
    let actual = flags.take(name)?;
    if actual == expected {
        Ok(())
    } else {
        Err(format!("{name} must be {expected}"))
    }
}

fn parse_u32(value: &str, name: &str) -> Result<u32, String> {
    value
        .parse::<u32>()
        .map_err(|_| format!("{name} must be a decimal integer"))
}

fn parse_u64(value: &str, name: &str) -> Result<u64, String> {
    value
        .parse::<u64>()
        .map_err(|_| format!("{name} must be a decimal integer"))
}

fn validate_capacity(capacity: u32) -> Result<(), String> {
    if (16..=1024).contains(&capacity) && capacity.is_power_of_two() {
        Ok(())
    } else {
        Err("capacity must be a power of two in 16..=1024".to_string())
    }
}

fn validate_output(path: &Path) -> Result<(), String> {
    if !path.is_absolute() {
        return Err("output must be an absolute path".to_string());
    }
    if path.exists() {
        return Err(format!("output already exists: {}", path.display()));
    }
    match path.parent() {
        Some(parent) if parent.is_dir() => Ok(()),
        _ => Err("output parent must be an existing directory".to_string()),
    }
}

fn require_target() -> Result<(), String> {
    if env::consts::OS != "linux" || env::consts::ARCH != "aarch64" {
        return Err(format!(
            "target evidence requires linux/aarch64, found {}/{}",
            env::consts::OS,
            env::consts::ARCH
        ));
    }
    Ok(())
}

struct NamespaceGuard {
    name: String,
}

impl std::ops::Deref for NamespaceGuard {
    type Target = str;

    fn deref(&self) -> &Self::Target {
        &self.name
    }
}

impl Drop for NamespaceGuard {
    fn drop(&mut self) {
        if env::consts::OS != "linux"
            || !self.name.starts_with("rsq_")
            || !self
                .name
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || byte == b'_')
        {
            return;
        }
        let path = Path::new("/dev/shm").join(format!("horus_{}", self.name));
        if path.parent() == Some(Path::new("/dev/shm")) {
            let _ = fs::remove_dir_all(path);
        }
    }
}

fn install_namespace(tag: &str) -> Result<NamespaceGuard, String> {
    let namespace = format!("rsq_{}_{}_{}", tag, std::process::id(), monotonic_ns()?);
    env::set_var("HORUS_NAMESPACE", &namespace);
    Ok(NamespaceGuard { name: namespace })
}

fn lock_memory() -> Result<(), String> {
    // SAFETY: mlockall has no pointer arguments. The flags request all current and
    // future mappings, and the return code is checked before any evidence is emitted.
    let result = unsafe { libc::mlockall(libc::MCL_CURRENT | libc::MCL_FUTURE) };
    if result != 0 {
        return Err(format!(
            "mlockall(MCL_CURRENT|MCL_FUTURE) failed: {}",
            std::io::Error::last_os_error()
        ));
    }
    let status = fs::read_to_string("/proc/self/status")
        .map_err(|error| format!("read /proc/self/status after mlockall: {error}"))?;
    let locked_kib = status
        .lines()
        .find_map(|line| line.strip_prefix("VmLck:"))
        .and_then(|value| value.split_ascii_whitespace().next())
        .and_then(|value| value.parse::<u64>().ok())
        .ok_or_else(|| "VmLck is missing after mlockall".to_string())?;
    if locked_kib == 0 {
        return Err("mlockall succeeded but VmLck is zero".to_string());
    }
    Ok(())
}

fn child_lock_mode(self_test: bool) -> &'static str {
    if self_test {
        "self-test-unlocked"
    } else {
        "required"
    }
}

fn prepare_child_lock(mode: &str) -> Result<(), String> {
    match mode {
        "required" => lock_memory(),
        "self-test-unlocked" if env::consts::ARCH != "aarch64" => Ok(()),
        "self-test-unlocked" => {
            Err("unlocked adapter self-test is forbidden on aarch64".to_string())
        }
        _ => Err("invalid child lock mode".to_string()),
    }
}

fn monotonic_ns() -> Result<u64, String> {
    clock_ns(libc::CLOCK_MONOTONIC, "CLOCK_MONOTONIC")
}

fn process_cpu_ns() -> Result<u64, String> {
    clock_ns(libc::CLOCK_PROCESS_CPUTIME_ID, "CLOCK_PROCESS_CPUTIME_ID")
}

fn clock_ns(clock: libc::clockid_t, name: &str) -> Result<u64, String> {
    let mut value = libc::timespec {
        tv_sec: 0,
        tv_nsec: 0,
    };
    // SAFETY: value points to writable timespec storage and clock is a Linux clock ID.
    let result = unsafe { libc::clock_gettime(clock, &mut value) };
    if result != 0 || value.tv_sec < 0 || !(0..1_000_000_000).contains(&value.tv_nsec) {
        return Err(format!(
            "{name} failed: {}",
            std::io::Error::last_os_error()
        ));
    }
    (value.tv_sec as u64)
        .checked_mul(1_000_000_000)
        .and_then(|seconds| seconds.checked_add(value.tv_nsec as u64))
        .ok_or_else(|| format!("{name} overflow"))
}

fn peak_resident_bytes() -> Result<u64, String> {
    let status = fs::read_to_string("/proc/self/status")
        .map_err(|error| format!("read /proc/self/status: {error}"))?;
    let kib = status
        .lines()
        .find_map(|line| line.strip_prefix("VmHWM:"))
        .and_then(|value| value.split_ascii_whitespace().next())
        .and_then(|value| value.parse::<u64>().ok())
        .ok_or_else(|| "VmHWM is missing from /proc/self/status".to_string())?;
    kib.checked_mul(1024)
        .ok_or_else(|| "VmHWM byte conversion overflow".to_string())
}

fn write_new(path: &Path, contents: &str) -> Result<(), String> {
    let result = (|| {
        let mut output = OpenOptions::new()
            .write(true)
            .create_new(true)
            .open(path)
            .map_err(|error| format!("create output {}: {error}", path.display()))?;
        output
            .write_all(contents.as_bytes())
            .and_then(|()| output.sync_all())
            .map_err(|error| format!("write output {}: {error}", path.display()))
    })();
    if result.is_err() {
        let _ = fs::remove_file(path);
    }
    result
}

fn preflight(mut flags: Flags) -> Result<(), String> {
    let common = parse_common(&mut flags)?;
    require_value(&mut flags, "--payload-bytes", "64,256,1024")?;
    require_value(&mut flags, "--kinds", "latest,ordered")?;
    require_value(&mut flags, "--latency-clock", "CLOCK_MONOTONIC")?;
    validate_contract_flags(&mut flags)?;
    flags.finish()?;
    require_target()?;
    let _namespace = install_namespace("preflight")?;
    lock_memory()?;
    prove_payload::<Payload64>(common.capacity)?;
    prove_payload::<Payload256>(common.capacity)?;
    prove_payload::<Payload1024>(common.capacity)?;
    let json = format!(
        concat!(
            "{{\"schema_version\":1,\"candidate\":\"horus\",",
            "\"source_commit\":\"{}\",\"target_os\":\"linux\",",
            "\"target_arch\":\"aarch64\",\"capacity\":{},",
            "\"payload_bytes\":[64,256,1024],\"kinds\":[\"latest\",\"ordered\"],",
            "\"latency_clock\":\"CLOCK_MONOTONIC\",",
            "\"operation_semantics\":\"try_publish_call\",",
            "\"memory_policy\":\"lock-required\",",
            "\"memory_lock_mechanism\":\"mlockall_current_future\",",
            "\"memory_lock_verified\":true,",
            "\"payload_contract\":\"repr_c_bytemuck_pod\",",
            "\"horus_generic_pod_detection\":\"needs_drop_heuristic_not_payload_proof\",",
            "\"hot_path_api\":\"try_send_try_recv\",",
            "\"fault_suite_ready\":true}}\n"
        ),
        common.source_commit, common.capacity
    );
    write_new(&common.output, &json)
}

fn prove_payload<P: BenchPayload>(capacity: u32) -> Result<(), String> {
    if std::mem::size_of::<P>() != P::BYTES {
        return Err(format!("payload {} has wrong layout", P::BYTES));
    }
    let topic_name = unique_topic("preflight", P::BYTES)?;
    let topic = Topic::<P>::with_capacity(&topic_name, capacity, None)
        .map_err(|error| format!("create Horus preflight topic: {error}"))?;
    let mut value = P::new(7);
    value.set_sent_ns(monotonic_ns()?);
    topic
        .try_send(value)
        .map_err(|_| "Horus preflight try_send failed".to_string())?;
    let received = topic
        .try_recv()
        .ok_or_else(|| "Horus preflight try_recv failed".to_string())?;
    if received.sequence() != 7 || !received.is_intact() {
        return Err("Horus preflight payload integrity failed".to_string());
    }
    Ok(())
}

fn unique_topic(tag: &str, payload: usize) -> Result<String, String> {
    Ok(format!(
        "rsq_{}_{}_{}_{}",
        tag,
        payload,
        std::process::id(),
        monotonic_ns()?
    ))
}

fn benchmark(mut flags: Flags) -> Result<(), String> {
    let common = parse_common(&mut flags)?;
    let kind = flags.take("--kind")?;
    if kind != "latest" && kind != "ordered" {
        return Err("kind must be latest or ordered".to_string());
    }
    let payload_bytes = flags
        .take("--payload-bytes")?
        .parse::<usize>()
        .map_err(|_| "payload-bytes must be 64, 256 or 1024".to_string())?;
    if !matches!(payload_bytes, 64 | 256 | 1024) {
        return Err("payload-bytes must be 64, 256 or 1024".to_string());
    }
    let warmup = parse_u64(&flags.take("--warmup")?, "warmup")?;
    let samples = parse_u64(&flags.take("--samples")?, "samples")?;
    if warmup > MAX_WARMUP {
        return Err(format!("warmup exceeds {MAX_WARMUP}"));
    }
    if samples == 0 || samples > MAX_SAMPLES {
        return Err(format!("samples must be in 1..={MAX_SAMPLES}"));
    }
    require_value(&mut flags, "--latency-clock", "CLOCK_MONOTONIC")?;
    validate_contract_flags(&mut flags)?;
    flags.finish()?;
    require_target()?;
    let namespace = install_namespace("benchmark")?;
    lock_memory()?;
    let args = BenchmarkArgs {
        common,
        kind,
        payload_bytes,
        warmup,
        samples,
        self_test: false,
    };
    let evidence = match payload_bytes {
        64 => run_pair::<Payload64>(&args, &namespace)?,
        256 => run_pair::<Payload256>(&args, &namespace)?,
        1024 => run_pair::<Payload1024>(&args, &namespace)?,
        _ => unreachable!("validated payload size"),
    };
    if evidence.allocations.count != 0 || evidence.allocations.bytes != 0 {
        return Err(format!(
            "hot path allocated: count={}, bytes={}",
            evidence.allocations.count, evidence.allocations.bytes
        ));
    }
    let json = render_benchmark(&args, &evidence);
    write_new(&args.common.output, &json)
}

struct ReadyMarker {
    path: PathBuf,
}

impl ReadyMarker {
    fn new(output: &Path, tag: &str) -> Result<Self, String> {
        let parent = output
            .parent()
            .ok_or_else(|| "output parent is missing".to_string())?;
        for attempt in 0..32u32 {
            let path = parent.join(format!(
                ".rs-control-horus-adapter-{}-{}-{}.ready",
                std::process::id(),
                tag,
                attempt
            ));
            if !path.exists() {
                return Ok(Self { path });
            }
        }
        Err("could not allocate a unique child readiness marker".to_string())
    }

    fn wait(&self, child: &mut Child) -> Result<(), String> {
        let deadline = Instant::now() + PROCESS_TIMEOUT;
        loop {
            if self.path.is_file() {
                return Ok(());
            }
            if let Some(status) = child
                .try_wait()
                .map_err(|error| format!("poll child: {error}"))?
            {
                return Err(format!("child exited before ready: {status}"));
            }
            if Instant::now() >= deadline {
                terminate_child(child);
                return Err("child readiness timed out".to_string());
            }
            thread::sleep(Duration::from_millis(1));
        }
    }
}

impl Drop for ReadyMarker {
    fn drop(&mut self) {
        let _ = fs::remove_file(&self.path);
    }
}

fn mark_ready(path: &Path) -> Result<(), String> {
    OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(path)
        .and_then(|mut marker| marker.write_all(b"ready\n"))
        .map_err(|error| format!("create readiness marker {}: {error}", path.display()))
}

struct ChildGuard {
    child: Option<Child>,
}

impl ChildGuard {
    fn process(&mut self) -> &mut Child {
        self.child.as_mut().expect("child guard is populated")
    }

    fn take(&mut self) -> Child {
        self.child.take().expect("child guard is populated")
    }

    fn terminate(&mut self) {
        if let Some(child) = self.child.as_mut() {
            terminate_child(child);
        }
        self.child.take();
    }
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        self.terminate();
    }
}

fn spawn_child(
    command: &str,
    args: &[(&str, String)],
    namespace: &str,
) -> Result<ChildGuard, String> {
    let executable = env::current_exe().map_err(|error| format!("resolve current exe: {error}"))?;
    let mut child = Command::new(executable);
    child.arg(command).env("HORUS_NAMESPACE", namespace);
    for (name, value) in args {
        child.arg(name).arg(value);
    }
    child
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::inherit())
        .spawn()
        .map(|child| ChildGuard { child: Some(child) })
        .map_err(|error| format!("spawn {command}: {error}"))
}

fn terminate_child(child: &mut Child) {
    let _ = child.kill();
    let _ = child.wait();
}

fn wait_child_output(mut guard: ChildGuard) -> Result<String, String> {
    let mut child = guard.take();
    let deadline = Instant::now() + PROCESS_TIMEOUT;
    let status = loop {
        if let Some(status) = child
            .try_wait()
            .map_err(|error| format!("poll child: {error}"))?
        {
            break status;
        }
        if Instant::now() >= deadline {
            terminate_child(&mut child);
            return Err("child execution timed out".to_string());
        }
        thread::sleep(Duration::from_millis(1));
    };
    let mut output = String::new();
    child
        .stdout
        .take()
        .ok_or_else(|| "child stdout was not captured".to_string())?
        .read_to_string(&mut output)
        .map_err(|error| format!("read child output: {error}"))?;
    if !status.success() {
        return Err(format!("child failed with {status}: {}", output.trim()));
    }
    Ok(output)
}

fn wait_for_ack(topic: &Topic<Ack>, sequence: u64) -> Result<(), String> {
    let deadline = monotonic_ns()?
        .checked_add(PROCESS_TIMEOUT.as_nanos() as u64)
        .ok_or_else(|| "ack deadline overflow".to_string())?;
    loop {
        if let Some(ack) = topic.try_recv() {
            if ack.words[0] != sequence {
                return Err(format!(
                    "unexpected acknowledgement: expected {sequence}, got {}",
                    ack.words[0]
                ));
            }
            return Ok(());
        }
        if monotonic_ns()? >= deadline {
            return Err(format!("acknowledgement {sequence} timed out"));
        }
        thread::yield_now();
    }
}

fn send_ack(topic: &Topic<Ack>, sequence: u64) -> Result<(), String> {
    let deadline = monotonic_ns()?
        .checked_add(PROCESS_TIMEOUT.as_nanos() as u64)
        .ok_or_else(|| "ack deadline overflow".to_string())?;
    let ack = Ack {
        words: [sequence, 0, 0, 0, 0, 0, 0, 0],
    };
    loop {
        if topic.try_send(ack).is_ok() {
            return Ok(());
        }
        if monotonic_ns()? >= deadline {
            return Err(format!("send acknowledgement {sequence} timed out"));
        }
        thread::yield_now();
    }
}

fn run_pair<P: BenchPayload>(args: &BenchmarkArgs, namespace: &str) -> Result<Evidence, String> {
    let payload_topic_name = unique_topic("payload", P::BYTES)?;
    let ack_topic_name = unique_topic("ack", P::BYTES)?;
    let payload_topic = Topic::<P>::with_capacity(&payload_topic_name, args.common.capacity, None)
        .map_err(|error| format!("create Horus payload topic: {error}"))?;
    let ack_topic = Topic::<Ack>::with_capacity(&ack_topic_name, args.common.capacity, None)
        .map_err(|error| format!("create Horus ack topic: {error}"))?;
    let marker = ReadyMarker::new(&args.common.output, "paired")?;
    let mut child = spawn_child(
        "__paired-child",
        &[
            ("--payload-topic", payload_topic_name),
            ("--ack-topic", ack_topic_name),
            ("--payload-bytes", P::BYTES.to_string()),
            ("--capacity", args.common.capacity.to_string()),
            ("--warmup", args.warmup.to_string()),
            ("--samples", args.samples.to_string()),
            ("--ready", marker.path.display().to_string()),
            ("--lock-mode", child_lock_mode(args.self_test).to_string()),
        ],
        namespace,
    )?;
    marker.wait(child.process())?;
    wait_for_ack(&ack_topic, READY_ACK)?;
    let mut prime = P::new(PRIME_SEQUENCE);
    prime.set_sent_ns(monotonic_ns()?);
    payload_topic
        .try_send(prime)
        .map_err(|_| "Horus prime try_send failed".to_string())?;
    wait_for_ack(&ack_topic, PRIME_SEQUENCE)?;
    payload_topic.check_migration_now();
    ack_topic.check_migration_now();

    let cpu_before = process_cpu_ns()?;
    let wall_before = monotonic_ns()?;
    let mut operation = Histogram::default();
    let total = args
        .warmup
        .checked_add(args.samples)
        .ok_or_else(|| "warmup + samples overflow".to_string())?;
    arm_allocations();
    let measured: Result<(), String> = (|| {
        for sequence in 0..total {
            let mut payload = P::new(sequence);
            payload.set_sent_ns(monotonic_ns()?);
            let operation_start = monotonic_ns()?;
            let send_result = payload_topic.try_send(payload);
            let operation_end = monotonic_ns()?;
            if send_result.is_err() {
                return Err(format!(
                    "paired Horus try_send failed at sequence {sequence}"
                ));
            }
            if sequence >= args.warmup {
                operation.record(operation_end.checked_sub(operation_start).ok_or_else(|| {
                    "CLOCK_MONOTONIC moved backwards during try_send".to_string()
                })?);
            }
            wait_for_ack(&ack_topic, sequence)?;
        }
        Ok(())
    })();
    let parent_allocations = disarm_allocations();
    measured?;
    let wall_ns = monotonic_ns()?
        .checked_sub(wall_before)
        .ok_or_else(|| "CLOCK_MONOTONIC moved backwards".to_string())?;
    let cpu_ns = process_cpu_ns()?
        .checked_sub(cpu_before)
        .ok_or_else(|| "CLOCK_PROCESS_CPUTIME_ID moved backwards".to_string())?;
    let peak_resident_bytes = peak_resident_bytes()?;
    let output = wait_child_output(child)?;
    let summary = ChildSummary::parse(&output)?;
    if summary.latency.total() != args.samples || operation.total() != args.samples {
        return Err("paired histogram sample count mismatch".to_string());
    }
    if summary.consumed != total || !summary.intact {
        return Err("paired child consumption/integrity mismatch".to_string());
    }
    Ok(Evidence {
        latency: summary.latency,
        operation,
        allocations: AllocationSnapshot {
            count: parent_allocations
                .count
                .saturating_add(summary.allocations.count),
            bytes: parent_allocations
                .bytes
                .saturating_add(summary.allocations.bytes),
        },
        published: total,
        consumed: summary.consumed,
        wall_ns,
        cpu_ns,
        peer_cpu_ns: summary.cpu_ns,
        peak_resident_bytes,
        peer_peak_resident_bytes: summary.peak_resident_bytes,
    })
}

struct ChildSummary {
    latency: Histogram,
    allocations: AllocationSnapshot,
    consumed: u64,
    cpu_ns: u64,
    peak_resident_bytes: u64,
    intact: bool,
}

impl ChildSummary {
    fn parse(output: &str) -> Result<Self, String> {
        let fields: Vec<&str> = output.split_ascii_whitespace().collect();
        if fields.len() != 9 || fields[0] != "PAIRED" {
            return Err(format!("invalid paired child summary: {}", output.trim()));
        }
        let mut latency = Histogram::default();
        let buckets: Vec<&str> = fields[1].split(',').collect();
        if buckets.len() != 64 {
            return Err("paired child histogram must have 64 buckets".to_string());
        }
        for (destination, value) in latency.buckets.iter_mut().zip(buckets) {
            *destination = parse_u64(value, "latency bucket")?;
        }
        latency.min_ns = parse_u64(fields[2], "latency min")?;
        latency.max_ns = parse_u64(fields[3], "latency max")?;
        latency.initialized = latency.total() != 0;
        Ok(Self {
            latency,
            cpu_ns: parse_u64(fields[4], "peer CPU time")?,
            peak_resident_bytes: parse_u64(fields[5], "peer peak resident bytes")?,
            allocations: AllocationSnapshot {
                count: parse_u64(fields[6], "peer allocation count")?,
                bytes: parse_u64(fields[7], "peer allocation bytes")?,
            },
            consumed: parse_u64(fields[8].split(':').next().unwrap_or(""), "consumed")?,
            intact: fields[8].split(':').nth(1) == Some("true"),
        })
    }
}

fn paired_child(mut flags: Flags) -> Result<(), String> {
    let payload_topic = flags.take("--payload-topic")?;
    let ack_topic = flags.take("--ack-topic")?;
    let payload_bytes = flags
        .take("--payload-bytes")?
        .parse::<usize>()
        .map_err(|_| "invalid child payload size".to_string())?;
    let capacity = parse_u32(&flags.take("--capacity")?, "capacity")?;
    let warmup = parse_u64(&flags.take("--warmup")?, "warmup")?;
    let samples = parse_u64(&flags.take("--samples")?, "samples")?;
    let ready = PathBuf::from(flags.take("--ready")?);
    let lock_mode = flags.take("--lock-mode")?;
    flags.finish()?;
    prepare_child_lock(&lock_mode)?;
    match payload_bytes {
        64 => paired_child_payload::<Payload64>(
            &payload_topic,
            &ack_topic,
            capacity,
            warmup,
            samples,
            &ready,
        ),
        256 => paired_child_payload::<Payload256>(
            &payload_topic,
            &ack_topic,
            capacity,
            warmup,
            samples,
            &ready,
        ),
        1024 => paired_child_payload::<Payload1024>(
            &payload_topic,
            &ack_topic,
            capacity,
            warmup,
            samples,
            &ready,
        ),
        _ => Err("invalid child payload size".to_string()),
    }
}

fn paired_child_payload<P: BenchPayload>(
    payload_name: &str,
    ack_name: &str,
    capacity: u32,
    warmup: u64,
    samples: u64,
    ready: &Path,
) -> Result<(), String> {
    let payload_topic = Topic::<P>::with_capacity(payload_name, capacity, None)
        .map_err(|error| format!("child create payload topic: {error}"))?;
    let ack_topic = Topic::<Ack>::with_capacity(ack_name, capacity, None)
        .map_err(|error| format!("child create ack topic: {error}"))?;
    let _ = payload_topic.try_recv();
    send_ack(&ack_topic, READY_ACK)?;
    mark_ready(ready)?;
    let prime = receive_with_timeout(&payload_topic)?;
    if prime.sequence() != PRIME_SEQUENCE || !prime.is_intact() {
        return Err("paired prime payload was invalid".to_string());
    }
    send_ack(&ack_topic, PRIME_SEQUENCE)?;
    payload_topic.check_migration_now();
    ack_topic.check_migration_now();

    let total = warmup
        .checked_add(samples)
        .ok_or_else(|| "child warmup + samples overflow".to_string())?;
    let cpu_before = process_cpu_ns()?;
    let mut latency = Histogram::default();
    let mut consumed = 0u64;
    let mut intact = true;
    arm_allocations();
    let measured: Result<(), String> = (|| {
        for sequence in 0..total {
            let payload = receive_with_timeout(&payload_topic)?;
            let received_ns = monotonic_ns()?;
            intact &= payload.sequence() == sequence && payload.is_intact();
            if sequence >= warmup {
                latency.record(received_ns.checked_sub(payload.sent_ns()).ok_or_else(|| {
                    "CLOCK_MONOTONIC receive time predates send time".to_string()
                })?);
            }
            consumed += 1;
            send_ack(&ack_topic, sequence)?;
        }
        Ok(())
    })();
    let allocations = disarm_allocations();
    measured?;
    let cpu_ns = process_cpu_ns()?
        .checked_sub(cpu_before)
        .ok_or_else(|| "child CPU clock moved backwards".to_string())?;
    let peak = peak_resident_bytes()?;
    let mut buckets = String::new();
    render_buckets(&mut buckets, &latency.buckets);
    println!(
        "PAIRED {} {} {} {} {} {} {} {}:{}",
        buckets,
        latency.min_ns,
        latency.max_ns,
        cpu_ns,
        peak,
        allocations.count,
        allocations.bytes,
        consumed,
        intact
    );
    Ok(())
}

fn receive_with_timeout<P: BenchPayload>(topic: &Topic<P>) -> Result<P, String> {
    let deadline = monotonic_ns()?
        .checked_add(PROCESS_TIMEOUT.as_nanos() as u64)
        .ok_or_else(|| "receive deadline overflow".to_string())?;
    loop {
        if let Some(value) = topic.try_recv() {
            return Ok(value);
        }
        if monotonic_ns()? >= deadline {
            return Err("Horus try_recv timed out".to_string());
        }
        thread::yield_now();
    }
}

fn render_buckets(destination: &mut String, buckets: &[u64; 64]) {
    for (index, value) in buckets.iter().enumerate() {
        if index != 0 {
            destination.push(',');
        }
        write!(destination, "{value}").expect("write to String");
    }
}

fn render_benchmark(args: &BenchmarkArgs, evidence: &Evidence) -> String {
    let mut latency = String::new();
    let mut operation = String::new();
    render_buckets(&mut latency, &evidence.latency.buckets);
    render_buckets(&mut operation, &evidence.operation.buckets);
    format!(
        concat!(
            "{{\"schema_version\":2,\"candidate\":\"horus\",",
            "\"source_commit\":\"{}\",\"target_os\":\"linux\",",
            "\"target_arch\":\"aarch64\",\"peer_execution\":\"separate_process_paired\",",
            "\"requested_memory_policy\":\"lock-required\",",
            "\"effective_memory_policy\":\"lock-required\",",
            "\"memory_policy_scope\":\"process_current_and_future_mappings\",",
            "\"memory_lock_mechanism\":\"mlockall_current_future\",",
            "\"memory_lock_verified\":true,",
            "\"payload_contract\":\"repr_c_bytemuck_pod\",",
            "\"horus_generic_pod_detection\":\"needs_drop_heuristic_not_payload_proof\",",
            "\"hot_path_api\":\"try_send_try_recv\",",
            "\"candidate_semantics\":\"universal_topic_same_nonblocking_api_for_declared_row\",",
            "\"kind\":\"{}\",\"payload_bytes\":{},\"capacity\":{},",
            "\"warmup\":{},\"samples\":{},",
            "\"latency_semantics\":\"one_way_receive\",",
            "\"latency_clock\":\"CLOCK_MONOTONIC\",",
            "\"latency_jitter_semantics\":\"max_minus_min\",",
            "\"latency_log2_ns\":[{}],\"latency_min_ns\":{},",
            "\"latency_max_ns\":{},\"latency_jitter_span_ns\":{},",
            "\"operation_semantics\":\"try_publish_call\",",
            "\"operation_clock\":\"CLOCK_MONOTONIC\",",
            "\"operation_log2_ns\":[{}],\"operation_max_ns\":{},",
            "\"wall_time_ns\":{},\"cpu_time_ns\":{},\"peer_cpu_time_ns\":{},",
            "\"cpu_time_source\":\"CLOCK_PROCESS_CPUTIME_ID\",",
            "\"peak_resident_bytes\":{},\"peer_peak_resident_bytes\":{},",
            "\"memory_source\":\"proc_self_status_VmHWM\",",
            "\"allocation_scope\":\"publisher_and_consumer\",",
            "\"allocations\":{{\"count\":{},\"bytes\":{},",
            "\"current_count\":0,\"current_bytes\":0,\"peak_count\":{},\"peak_bytes\":{}}},",
            "\"counters\":{{\"published\":{},\"consumed\":{},",
            "\"overwritten\":0,\"full\":0,\"disconnected\":0}}}}\n"
        ),
        args.common.source_commit,
        args.kind,
        args.payload_bytes,
        args.common.capacity,
        args.warmup,
        args.samples,
        latency,
        evidence.latency.min_ns,
        evidence.latency.max_ns,
        evidence
            .latency
            .max_ns
            .saturating_sub(evidence.latency.min_ns),
        operation,
        evidence.operation.max_ns,
        evidence.wall_ns,
        evidence.cpu_ns,
        evidence.peer_cpu_ns,
        evidence.peak_resident_bytes,
        evidence.peer_peak_resident_bytes,
        evidence.allocations.count,
        evidence.allocations.bytes,
        evidence.allocations.count,
        evidence.allocations.bytes,
        evidence.published,
        evidence.consumed
    )
}

#[derive(Default)]
struct SlowObservation {
    attempted: u64,
    published: u64,
    consumed: u64,
    full: u64,
    gaps: u64,
    max_try_send_ns: u64,
    intact: bool,
}

#[derive(Default)]
struct StoppedObservation {
    attempted: u64,
    published: u64,
    full: u64,
    max_try_send_ns: u64,
}

fn self_test(mut flags: Flags) -> Result<(), String> {
    let source_commit = flags.take("--source-commit")?;
    flags.finish()?;
    if source_commit != BUILT_HORUS_COMMIT {
        return Err(format!(
            "source commit mismatch: binary={}, requested={source_commit}",
            BUILT_HORUS_COMMIT
        ));
    }
    if env::consts::ARCH == "aarch64" {
        return Err(
            "unlocked self-test is host-only; use the lock-required qualification commands on aarch64"
                .to_string(),
        );
    }
    let output = env::temp_dir().join(format!(
        "rs-control-horus-adapter-self-test-{}-{}.json",
        std::process::id(),
        monotonic_ns()?
    ));
    validate_output(&output)?;
    let common = CommonPublic {
        source_commit,
        capacity: 16,
        output,
    };
    let namespace = install_namespace("self_test")?;
    let pair_args = BenchmarkArgs {
        common: CommonPublic {
            source_commit: common.source_commit.clone(),
            capacity: common.capacity,
            output: common.output.clone(),
        },
        kind: "ordered".to_string(),
        payload_bytes: 64,
        warmup: 4,
        samples: 64,
        self_test: true,
    };
    let pairs = [
        run_pair::<Payload64>(&pair_args, &namespace)?,
        run_pair::<Payload256>(&pair_args, &namespace)?,
        run_pair::<Payload1024>(&pair_args, &namespace)?,
    ];
    if pairs.iter().any(|evidence| {
        evidence.published != 68
            || evidence.consumed != 68
            || evidence.allocations.count != 0
            || evidence.allocations.bytes != 0
    }) {
        return Err("paired host self-test counters or allocation gate failed".to_string());
    }
    let slow = run_slow_consumer::<Payload64>(&common, &namespace, true)?;
    let stopped = run_stopped_consumer::<Payload64>(&common, &namespace, true)?;
    let clean = run_clean_reclaim::<Payload64>(&common, &namespace, true)?;
    let killed = run_killed_reclaim::<Payload64>(&common, &namespace, true)?;
    if slow.attempted != slow.published + slow.full
        || slow.consumed != slow.published + 1
        || slow.gaps != slow.full
        || slow.full == 0
        || !slow.intact
        || slow.max_try_send_ns > TRY_SEND_BOUND_NS
        || stopped.attempted != stopped.published + stopped.full
        || stopped.full == 0
        || stopped.max_try_send_ns > TRY_SEND_BOUND_NS
        || !clean
        || !killed
    {
        return Err("host process-fault self-test gate failed".to_string());
    }
    println!(
        "self_test=pass target_qualification=false arch={} horus_commit={}",
        env::consts::ARCH,
        BUILT_HORUS_COMMIT
    );
    Ok(())
}

fn fault_suite(mut flags: Flags) -> Result<(), String> {
    let common = parse_common(&mut flags)?;
    require_value(&mut flags, "--payload-bytes", "64,256,1024")?;
    validate_contract_flags(&mut flags)?;
    flags.finish()?;
    require_target()?;
    let namespace = install_namespace("faults")?;
    lock_memory()?;

    let pair_args = BenchmarkArgs {
        common: CommonPublic {
            source_commit: common.source_commit.clone(),
            capacity: common.capacity,
            output: common.output.clone(),
        },
        kind: "ordered".to_string(),
        payload_bytes: 64,
        warmup: 4,
        samples: 64,
        self_test: false,
    };
    let pair64 = run_pair::<Payload64>(&pair_args, &namespace)?;
    let pair256 = run_pair::<Payload256>(&pair_args, &namespace)?;
    let pair1024 = run_pair::<Payload1024>(&pair_args, &namespace)?;
    let no_torn_values = [pair64, pair256, pair1024]
        .iter()
        .all(|evidence| evidence.published == 68 && evidence.consumed == 68);

    let slow = run_slow_consumer::<Payload64>(&common, &namespace, false)?;
    let stopped = run_stopped_consumer::<Payload64>(&common, &namespace, false)?;
    let clean_exit_reclaim = run_clean_reclaim::<Payload64>(&common, &namespace, false)?;
    let killed_peer_reclaim = run_killed_reclaim::<Payload64>(&common, &namespace, false)?;
    let producer_bounded =
        slow.max_try_send_ns <= TRY_SEND_BOUND_NS && stopped.max_try_send_ns <= TRY_SEND_BOUND_NS;
    let losses_accounted = slow.attempted == slow.published + slow.full
        && slow.consumed == slow.published + 1
        && slow.gaps == slow.full;
    let slow_consumer_accounted = losses_accounted && slow.full > 0 && slow.intact;
    let stopped_consumer_bounded = stopped.max_try_send_ns <= TRY_SEND_BOUND_NS;
    let full_queue_explicit = stopped.full > 0
        && stopped.attempted == stopped.published + stopped.full
        && stopped.published <= u64::from(common.capacity);
    let crash_reclaim = clean_exit_reclaim && killed_peer_reclaim;
    let passed = producer_bounded
        && no_torn_values
        && crash_reclaim
        && losses_accounted
        && slow_consumer_accounted
        && stopped_consumer_bounded
        && full_queue_explicit;
    let json = format!(
        concat!(
            "{{\"schema_version\":1,\"candidate\":\"horus\",",
            "\"source_commit\":\"{}\",\"target_arch\":\"aarch64\",",
            "\"capacity\":{},\"payload_bytes\":[64,256,1024],",
            "\"producer_bounded\":{},\"no_torn_values\":{},",
            "\"crash_reclaim\":{},\"losses_accounted\":{},",
            "\"slow_consumer_accounted\":{},\"stopped_consumer_bounded\":{},",
            "\"full_queue_explicit\":{},\"clean_exit_reclaim\":{},",
            "\"killed_peer_reclaim\":{},\"memory_lock_verified\":true,",
            "\"capacity_contract\":\"power_of_two_16_through_1024\",",
            "\"payload_contract\":\"repr_c_bytemuck_pod\",",
            "\"memory_lock_mechanism\":\"mlockall_current_future\",",
            "\"hot_path_api\":\"try_send_try_recv\",",
            "\"observations\":{{\"slow_attempted\":{},\"slow_published\":{},",
            "\"slow_consumed\":{},\"slow_full\":{},\"slow_gaps\":{},",
            "\"slow_max_try_send_ns\":{},\"stopped_attempted\":{},",
            "\"stopped_published\":{},\"stopped_full\":{},",
            "\"stopped_max_try_send_ns\":{}}}}}\n"
        ),
        common.source_commit,
        common.capacity,
        producer_bounded,
        no_torn_values,
        crash_reclaim,
        losses_accounted,
        slow_consumer_accounted,
        stopped_consumer_bounded,
        full_queue_explicit,
        clean_exit_reclaim,
        killed_peer_reclaim,
        slow.attempted,
        slow.published,
        slow.consumed,
        slow.full,
        slow.gaps,
        slow.max_try_send_ns,
        stopped.attempted,
        stopped.published,
        stopped.full,
        stopped.max_try_send_ns
    );
    write_new(&common.output, &json)?;
    if passed {
        Ok(())
    } else {
        Err("one or more Horus process fault gates failed".to_string())
    }
}

fn run_slow_consumer<P: BenchPayload>(
    common: &CommonPublic,
    namespace: &str,
    self_test: bool,
) -> Result<SlowObservation, String> {
    let topic_name = unique_topic("slow", P::BYTES)?;
    let topic = Topic::<P>::with_capacity(&topic_name, common.capacity, None)
        .map_err(|error| format!("create slow-consumer topic: {error}"))?;
    let marker = ReadyMarker::new(&common.output, "slow")?;
    let attempts = u64::from(common.capacity) * 8;
    let stop_sequence = attempts + 1;
    let mut child = spawn_child(
        "__slow-child",
        &[
            ("--topic", topic_name),
            ("--payload-bytes", P::BYTES.to_string()),
            ("--capacity", common.capacity.to_string()),
            ("--delay-us", "1000".to_string()),
            ("--stop-sequence", stop_sequence.to_string()),
            ("--ready", marker.path.display().to_string()),
            ("--lock-mode", child_lock_mode(self_test).to_string()),
        ],
        namespace,
    )?;
    marker.wait(child.process())?;
    topic.check_migration_now();
    send_payload_with_timeout(&topic, 0)?;
    thread::sleep(Duration::from_millis(5));
    let mut observation = SlowObservation {
        attempted: attempts,
        intact: false,
        ..SlowObservation::default()
    };
    for sequence in 1..=attempts {
        let mut payload = P::new(sequence);
        payload.set_sent_ns(monotonic_ns()?);
        let before = monotonic_ns()?;
        let result = topic.try_send(payload);
        let elapsed = monotonic_ns()?
            .checked_sub(before)
            .ok_or_else(|| "slow-consumer try_send clock moved backwards".to_string())?;
        observation.max_try_send_ns = observation.max_try_send_ns.max(elapsed);
        if result.is_ok() {
            observation.published += 1;
        } else {
            observation.full += 1;
        }
    }
    send_payload_with_timeout(&topic, stop_sequence)?;
    let output = wait_child_output(child)?;
    let fields: Vec<&str> = output.split_ascii_whitespace().collect();
    if fields.len() != 4 || fields[0] != "SLOW" {
        return Err(format!("invalid slow child summary: {}", output.trim()));
    }
    observation.consumed = parse_u64(fields[1], "slow consumed")?;
    observation.gaps = parse_u64(fields[2], "slow gaps")?;
    observation.intact = fields[3] == "true";
    Ok(observation)
}

fn run_stopped_consumer<P: BenchPayload>(
    common: &CommonPublic,
    namespace: &str,
    self_test: bool,
) -> Result<StoppedObservation, String> {
    let topic_name = unique_topic("stopped", P::BYTES)?;
    let topic = Topic::<P>::with_capacity(&topic_name, common.capacity, None)
        .map_err(|error| format!("create stopped-consumer topic: {error}"))?;
    let marker = ReadyMarker::new(&common.output, "stopped")?;
    let mut child = spawn_child(
        "__hold-child",
        &[
            ("--topic", topic_name),
            ("--payload-bytes", P::BYTES.to_string()),
            ("--capacity", common.capacity.to_string()),
            ("--hold-ms", "10000".to_string()),
            ("--ready", marker.path.display().to_string()),
            ("--lock-mode", child_lock_mode(self_test).to_string()),
        ],
        namespace,
    )?;
    marker.wait(child.process())?;
    topic.check_migration_now();
    let attempted = u64::from(common.capacity) * 4;
    let mut observation = StoppedObservation {
        attempted,
        ..StoppedObservation::default()
    };
    for sequence in 0..attempted {
        let mut payload = P::new(sequence);
        payload.set_sent_ns(monotonic_ns()?);
        let before = monotonic_ns()?;
        let result = topic.try_send(payload);
        let elapsed = monotonic_ns()?
            .checked_sub(before)
            .ok_or_else(|| "stopped-consumer try_send clock moved backwards".to_string())?;
        observation.max_try_send_ns = observation.max_try_send_ns.max(elapsed);
        if result.is_ok() {
            observation.published += 1;
        } else {
            observation.full += 1;
        }
    }
    child.terminate();
    Ok(observation)
}

fn send_payload_with_timeout<P: BenchPayload>(
    topic: &Topic<P>,
    sequence: u64,
) -> Result<(), String> {
    let deadline = monotonic_ns()?
        .checked_add(PROCESS_TIMEOUT.as_nanos() as u64)
        .ok_or_else(|| "send deadline overflow".to_string())?;
    loop {
        let mut payload = P::new(sequence);
        payload.set_sent_ns(monotonic_ns()?);
        if topic.try_send(payload).is_ok() {
            return Ok(());
        }
        if monotonic_ns()? >= deadline {
            return Err(format!("send sequence {sequence} timed out"));
        }
        thread::yield_now();
    }
}

fn slow_child(mut flags: Flags) -> Result<(), String> {
    let topic = flags.take("--topic")?;
    let payload_bytes = flags
        .take("--payload-bytes")?
        .parse::<usize>()
        .map_err(|_| "invalid slow child payload size".to_string())?;
    let capacity = parse_u32(&flags.take("--capacity")?, "capacity")?;
    let delay_us = parse_u64(&flags.take("--delay-us")?, "delay-us")?;
    let stop_sequence = parse_u64(&flags.take("--stop-sequence")?, "stop-sequence")?;
    let ready = PathBuf::from(flags.take("--ready")?);
    let lock_mode = flags.take("--lock-mode")?;
    flags.finish()?;
    prepare_child_lock(&lock_mode)?;
    match payload_bytes {
        64 => slow_child_payload::<Payload64>(&topic, capacity, delay_us, stop_sequence, &ready),
        256 => slow_child_payload::<Payload256>(&topic, capacity, delay_us, stop_sequence, &ready),
        1024 => {
            slow_child_payload::<Payload1024>(&topic, capacity, delay_us, stop_sequence, &ready)
        }
        _ => Err("invalid slow child payload size".to_string()),
    }
}

fn slow_child_payload<P: BenchPayload>(
    topic_name: &str,
    capacity: u32,
    delay_us: u64,
    stop_sequence: u64,
    ready: &Path,
) -> Result<(), String> {
    let topic = Topic::<P>::with_capacity(topic_name, capacity, None)
        .map_err(|error| format!("slow child create topic: {error}"))?;
    let _ = topic.try_recv();
    mark_ready(ready)?;
    let deadline = monotonic_ns()?
        .checked_add(PROCESS_TIMEOUT.as_nanos() as u64)
        .ok_or_else(|| "slow child deadline overflow".to_string())?;
    let mut consumed = 0u64;
    let mut gaps = 0u64;
    let mut last = None;
    let mut intact = true;
    loop {
        if let Some(payload) = topic.try_recv() {
            let sequence = payload.sequence();
            intact &= payload.is_intact();
            if let Some(previous) = last {
                if sequence <= previous {
                    intact = false;
                } else {
                    gaps = gaps.saturating_add(sequence - previous - 1);
                }
            }
            last = Some(sequence);
            if sequence == stop_sequence {
                break;
            }
            consumed += 1;
            thread::sleep(Duration::from_micros(delay_us));
        } else {
            if monotonic_ns()? >= deadline {
                return Err("slow child timed out".to_string());
            }
            thread::yield_now();
        }
    }
    println!("SLOW {consumed} {gaps} {intact}");
    Ok(())
}

fn hold_child(mut flags: Flags) -> Result<(), String> {
    let topic = flags.take("--topic")?;
    let payload_bytes = flags
        .take("--payload-bytes")?
        .parse::<usize>()
        .map_err(|_| "invalid hold child payload size".to_string())?;
    let capacity = parse_u32(&flags.take("--capacity")?, "capacity")?;
    let hold_ms = parse_u64(&flags.take("--hold-ms")?, "hold-ms")?;
    let ready = PathBuf::from(flags.take("--ready")?);
    let lock_mode = flags.take("--lock-mode")?;
    flags.finish()?;
    prepare_child_lock(&lock_mode)?;
    match payload_bytes {
        64 => hold_child_payload::<Payload64>(&topic, capacity, hold_ms, &ready),
        256 => hold_child_payload::<Payload256>(&topic, capacity, hold_ms, &ready),
        1024 => hold_child_payload::<Payload1024>(&topic, capacity, hold_ms, &ready),
        _ => Err("invalid hold child payload size".to_string()),
    }
}

fn hold_child_payload<P: BenchPayload>(
    topic_name: &str,
    capacity: u32,
    hold_ms: u64,
    ready: &Path,
) -> Result<(), String> {
    let topic = Topic::<P>::with_capacity(topic_name, capacity, None)
        .map_err(|error| format!("hold child create topic: {error}"))?;
    let _ = topic.try_recv();
    mark_ready(ready)?;
    thread::sleep(Duration::from_millis(hold_ms));
    Ok(())
}

fn run_clean_reclaim<P: BenchPayload>(
    common: &CommonPublic,
    namespace: &str,
    self_test: bool,
) -> Result<bool, String> {
    let topic_name = unique_topic("clean_reclaim", P::BYTES)?;
    let topic = Topic::<P>::with_capacity(&topic_name, common.capacity, None)
        .map_err(|error| format!("create clean-reclaim topic: {error}"))?;
    run_once_consumer(&topic, &topic_name, common, namespace, 10, self_test)?;
    run_once_consumer(&topic, &topic_name, common, namespace, 11, self_test)?;
    Ok(true)
}

fn run_killed_reclaim<P: BenchPayload>(
    common: &CommonPublic,
    namespace: &str,
    self_test: bool,
) -> Result<bool, String> {
    let topic_name = unique_topic("killed_reclaim", P::BYTES)?;
    let topic = Topic::<P>::with_capacity(&topic_name, common.capacity, None)
        .map_err(|error| format!("create killed-reclaim topic: {error}"))?;
    let marker = ReadyMarker::new(&common.output, "killed")?;
    let mut child = spawn_child(
        "__hold-child",
        &[
            ("--topic", topic_name.clone()),
            ("--payload-bytes", P::BYTES.to_string()),
            ("--capacity", common.capacity.to_string()),
            ("--hold-ms", "10000".to_string()),
            ("--ready", marker.path.display().to_string()),
            ("--lock-mode", child_lock_mode(self_test).to_string()),
        ],
        namespace,
    )?;
    marker.wait(child.process())?;
    child.terminate();
    run_once_consumer(&topic, &topic_name, common, namespace, 12, self_test)?;
    Ok(true)
}

fn run_once_consumer<P: BenchPayload>(
    topic: &Topic<P>,
    topic_name: &str,
    common: &CommonPublic,
    namespace: &str,
    sequence: u64,
    self_test: bool,
) -> Result<(), String> {
    let marker = ReadyMarker::new(&common.output, "once")?;
    let mut child = spawn_child(
        "__once-child",
        &[
            ("--topic", topic_name.to_string()),
            ("--payload-bytes", P::BYTES.to_string()),
            ("--capacity", common.capacity.to_string()),
            ("--sequence", sequence.to_string()),
            ("--ready", marker.path.display().to_string()),
            ("--lock-mode", child_lock_mode(self_test).to_string()),
        ],
        namespace,
    )?;
    marker.wait(child.process())?;
    topic.check_migration_now();
    send_payload_with_timeout(topic, sequence)?;
    let output = wait_child_output(child)?;
    if output.trim() != format!("ONCE {sequence} true") {
        return Err(format!("invalid once child summary: {}", output.trim()));
    }
    Ok(())
}

fn once_child(mut flags: Flags) -> Result<(), String> {
    let topic = flags.take("--topic")?;
    let payload_bytes = flags
        .take("--payload-bytes")?
        .parse::<usize>()
        .map_err(|_| "invalid once child payload size".to_string())?;
    let capacity = parse_u32(&flags.take("--capacity")?, "capacity")?;
    let sequence = parse_u64(&flags.take("--sequence")?, "sequence")?;
    let ready = PathBuf::from(flags.take("--ready")?);
    let lock_mode = flags.take("--lock-mode")?;
    flags.finish()?;
    prepare_child_lock(&lock_mode)?;
    match payload_bytes {
        64 => once_child_payload::<Payload64>(&topic, capacity, sequence, &ready),
        256 => once_child_payload::<Payload256>(&topic, capacity, sequence, &ready),
        1024 => once_child_payload::<Payload1024>(&topic, capacity, sequence, &ready),
        _ => Err("invalid once child payload size".to_string()),
    }
}

fn once_child_payload<P: BenchPayload>(
    topic_name: &str,
    capacity: u32,
    sequence: u64,
    ready: &Path,
) -> Result<(), String> {
    let topic = Topic::<P>::with_capacity(topic_name, capacity, None)
        .map_err(|error| format!("once child create topic: {error}"))?;
    let _ = topic.try_recv();
    mark_ready(ready)?;
    let payload = receive_with_timeout(&topic)?;
    let intact = payload.sequence() == sequence && payload.is_intact();
    println!("ONCE {} {}", payload.sequence(), intact);
    if intact {
        Ok(())
    } else {
        Err("once child payload integrity failed".to_string())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn payload_contracts_have_exact_sizes_and_detect_corruption() {
        fn check<P: BenchPayload>() {
            assert_eq!(std::mem::size_of::<P>(), P::BYTES);
            let mut payload = P::new(42);
            payload.set_sent_ns(99);
            assert_eq!(payload.sequence(), 42);
            assert_eq!(payload.sent_ns(), 99);
            assert!(payload.is_intact());
            payload.words_mut()[2] ^= 1;
            assert!(!payload.is_intact());
        }
        check::<Payload64>();
        check::<Payload256>();
        check::<Payload1024>();
    }

    #[test]
    fn histogram_uses_the_evaluator_log2_bucket_contract() {
        let mut histogram = Histogram::default();
        for value in [0, 1, 2, 3, 4, u64::MAX] {
            histogram.record(value);
        }
        assert_eq!(histogram.total(), 6);
        assert_eq!(histogram.buckets[0], 1);
        assert_eq!(histogram.buckets[1], 1);
        assert_eq!(histogram.buckets[2], 2);
        assert_eq!(histogram.buckets[3], 1);
        assert_eq!(histogram.buckets[63], 1);
        assert_eq!(histogram.min_ns, 0);
        assert_eq!(histogram.max_ns, u64::MAX);
    }

    #[test]
    fn benchmark_json_has_the_strict_evaluator_fields() {
        let mut latency = Histogram::default();
        let mut operation = Histogram::default();
        for _ in 0..4 {
            latency.record(8);
            operation.record(4);
        }
        let args = BenchmarkArgs {
            common: CommonPublic {
                source_commit: BUILT_HORUS_COMMIT.to_string(),
                capacity: 64,
                output: PathBuf::from("/tmp/not-written.json"),
            },
            kind: "latest".to_string(),
            payload_bytes: 64,
            warmup: 2,
            samples: 4,
            self_test: true,
        };
        let evidence = Evidence {
            latency,
            operation,
            allocations: AllocationSnapshot::default(),
            published: 6,
            consumed: 6,
            wall_ns: 10,
            cpu_ns: 9,
            peer_cpu_ns: 8,
            peak_resident_bytes: 7,
            peer_peak_resident_bytes: 6,
        };
        let value: serde_json::Value =
            serde_json::from_str(&render_benchmark(&args, &evidence)).unwrap();
        assert_eq!(value["schema_version"], 2);
        assert_eq!(value["candidate"], "horus");
        assert_eq!(value["source_commit"], BUILT_HORUS_COMMIT);
        assert_eq!(value["peer_execution"], "separate_process_paired");
        assert_eq!(value["memory_lock_verified"], true);
        assert_eq!(value["kind"], "latest");
        assert_eq!(value["latency_log2_ns"].as_array().unwrap().len(), 64);
        assert_eq!(value["operation_log2_ns"].as_array().unwrap().len(), 64);
        assert_eq!(value["allocations"]["count"], 0);
        assert_eq!(value["counters"]["published"], 6);
        assert_eq!(value["counters"]["consumed"], 6);
    }

    #[test]
    fn paired_child_summary_parser_rejects_shape_drift() {
        let buckets = std::iter::repeat_n("0", 64).collect::<Vec<_>>().join(",");
        let summary =
            ChildSummary::parse(&format!("PAIRED {buckets} 0 0 1 2 0 0 4:true\n")).unwrap();
        assert_eq!(summary.latency.total(), 0);
        assert_eq!(summary.consumed, 4);
        assert!(summary.intact);
        assert!(ChildSummary::parse("PAIRED incomplete").is_err());
    }

    #[test]
    fn public_flag_parser_rejects_duplicates_and_positionals() {
        assert!(Flags::parse(["--capacity".to_string(), "64".to_string()].into_iter()).is_ok());
        assert!(Flags::parse(
            [
                "--capacity".to_string(),
                "64".to_string(),
                "--capacity".to_string(),
                "32".to_string(),
            ]
            .into_iter()
        )
        .is_err());
        assert!(Flags::parse(["capacity".to_string()].into_iter()).is_err());
    }
}
