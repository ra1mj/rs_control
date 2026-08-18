use std::{env, process::Command, sync::OnceLock};

use allocation_counter::AllocationInfo;

/// Isolated Release-test report for one warmed host-only kernel.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct HostPerformanceReport {
    p50_ns: u128,
    p95_ns: u128,
    p99_ns: u128,
    throughput_per_second: u128,
}

impl HostPerformanceReport {
    /// Returns the median warm-call latency in nanoseconds.
    #[must_use]
    pub const fn p50_ns(self) -> u128 {
        self.p50_ns
    }

    /// Returns the P95 warm-call latency in nanoseconds.
    #[must_use]
    pub const fn p95_ns(self) -> u128 {
        self.p95_ns
    }
}

/// Emits a stable host-only Release report and applies an optional same-host
/// median baseline. Set both `ARM_ALGORITHMS_BASELINE_HOST` and
/// `ARM_ALGORITHMS_BASELINE_<KERNEL>_P50_NS` to enable the 15% regression gate.
pub fn report(
    kernel: &str,
    samples_ns: &mut [u128],
    allocations: AllocationInfo,
    prepared_bytes: usize,
    work_counter: u64,
) -> HostPerformanceReport {
    assert!(
        !samples_ns.is_empty(),
        "performance samples cannot be empty"
    );
    samples_ns.sort_unstable();
    let report = HostPerformanceReport {
        p50_ns: percentile(samples_ns, 50),
        p95_ns: percentile(samples_ns, 95),
        p99_ns: percentile(samples_ns, 99),
        throughput_per_second: samples_ns
            .iter()
            .copied()
            .sum::<u128>()
            .checked_div(samples_ns.len() as u128)
            .filter(|average_ns| *average_ns > 0)
            .map_or(0, |average_ns| 1_000_000_000_u128 / average_ns),
    };
    eprintln!(
        "arm-algorithms host-release kernel={kernel} p50_ns={} p95_ns={} p99_ns={} throughput_per_s={} allocations_total={} allocation_bytes_total={} prepared_bytes={prepared_bytes} work_counter={work_counter} host={} qualification=host-only",
        report.p50_ns,
        report.p95_ns,
        report.p99_ns,
        report.throughput_per_second,
        allocations.count_total,
        allocations.bytes_total,
        host_fingerprint(),
    );
    assert_same_host_baseline(kernel, report.p50_ns);
    report
}

fn percentile(samples_ns: &[u128], percentile: usize) -> u128 {
    samples_ns[(samples_ns.len() * percentile).div_ceil(100) - 1]
}

fn assert_same_host_baseline(kernel: &str, p50_ns: u128) {
    let baseline_key = format!(
        "ARM_ALGORITHMS_BASELINE_{}_P50_NS",
        kernel
            .bytes()
            .map(|byte| match byte {
                b'a'..=b'z' => (byte - b'a' + b'A') as char,
                b'A'..=b'Z' | b'0'..=b'9' => byte as char,
                _ => '_',
            })
            .collect::<String>()
    );
    let Ok(raw_baseline) = env::var(&baseline_key) else {
        return;
    };
    let baseline_ns = raw_baseline
        .parse::<u128>()
        .unwrap_or_else(|_| panic!("{baseline_key} must be a positive nanosecond integer"));
    assert!(baseline_ns > 0, "{baseline_key} must be positive");
    let expected_host = env::var("ARM_ALGORITHMS_BASELINE_HOST")
        .unwrap_or_else(|_| panic!("ARM_ALGORITHMS_BASELINE_HOST is required with {baseline_key}"));
    assert_eq!(
        expected_host,
        host_fingerprint(),
        "baseline host differs; compare Release medians only on the same host/profile"
    );
    let permitted_ns = baseline_ns
        .checked_mul(115)
        .and_then(|value| value.checked_div(100))
        .expect("baseline regression threshold overflow");
    assert!(
        p50_ns <= permitted_ns,
        "{kernel} median regressed by more than 15%: baseline={baseline_ns} ns current={p50_ns} ns"
    );
}

fn host_fingerprint() -> &'static str {
    static FINGERPRINT: OnceLock<String> = OnceLock::new();
    FINGERPRINT
        .get_or_init(|| {
            let uname = Command::new("uname")
                .args(["-srmo"])
                .output()
                .ok()
                .filter(|output| output.status.success())
                .map(|output| String::from_utf8_lossy(&output.stdout).trim().to_owned())
                .unwrap_or_else(|| "uname-unavailable".to_owned());
            let parallelism = std::thread::available_parallelism()
                .map(|value| value.get())
                .unwrap_or(0);
            format!(
                "os={} arch={} uname={} cpus={} rust_msrv={}",
                env::consts::OS,
                env::consts::ARCH,
                uname,
                parallelism,
                env!("CARGO_PKG_RUST_VERSION"),
            )
        })
        .as_str()
}
