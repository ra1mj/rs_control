//! Bounded, non-realtime MCAP recording and replay of already-encoded evidence.
//!
//! This crate has no command, transport, or device capability. Callers use the
//! fixed-capacity ingress from a producer and let a non-realtime owner drain it
//! into [`TelemetryRecorder`].

use std::{
    collections::{BTreeMap, BTreeSet},
    fs::{self, File, OpenOptions},
    io::{BufWriter, Write},
    path::{Path, PathBuf},
};

use arrayvec::{ArrayString, ArrayVec};
use mcap::{
    WriteOptions, Writer,
    read::ChunkFlattener,
    records::{MessageHeader, Metadata, Record},
};
use message_model::{Id128, SourceId};
use proto_generated::FILE_DESCRIPTOR_SET;
use sha2::{Digest, Sha256};
use thiserror::Error;

/// Maximum bytes in one topic name.
pub const MAX_TOPIC_BYTES: usize = 96;
/// Maximum original Protobuf payload preserved by one record.
pub const MAX_PAYLOAD_BYTES: usize = 4_096;
/// Maximum items retained for each priority class before a drain occurs.
pub const MAX_CRITICAL_ITEMS: usize = 16;
/// Maximum control-state items retained before a drain occurs.
pub const MAX_CONTROL_ITEMS: usize = 16;
/// Maximum bulk-input items retained before a drain occurs.
pub const MAX_BULK_ITEMS: usize = 16;
/// Maximum distinct topic/source/boot channels in one bounded output file.
pub const MAX_RECORDING_CHANNELS: usize = 64;
/// Maximum MCAP chunk size admitted by the non-realtime writer.
pub const MAX_CHUNK_SIZE_BYTES: u64 = 1_048_576;
const METADATA_BATCH_ITEMS: usize = 16;
const GAP_RECORD_RESERVE_BYTES: u64 = 512;

/// SHA-256 identity of one descriptor set.
pub type DescriptorDigest = [u8; 32];

/// Returns the descriptor identity recorded by this crate.
#[must_use]
pub fn descriptor_digest() -> DescriptorDigest {
    Sha256::digest(FILE_DESCRIPTOR_SET).into()
}

/// Producer priority. Capacity is reserved independently for every class.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TelemetryPriority {
    /// Fault, lifecycle, and command evidence.
    Critical,
    /// Control-state evidence.
    Control,
    /// Bulk sensor or auxiliary input evidence.
    Bulk,
}

impl TelemetryPriority {
    const fn label(self) -> &'static str {
        match self {
            Self::Critical => "critical",
            Self::Control => "control",
            Self::Bulk => "bulk",
        }
    }
}

/// Fixed, typed identity and timestamp metadata for an encoded record.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TelemetryMetadata {
    /// Sequence in the source boot identity. It is preserved as a full `u64`.
    pub sequence: u64,
    /// Stable semantic publisher identity.
    pub source_id: SourceId,
    /// Publisher boot identity.
    pub boot_id: Id128,
    /// Timestamp in the source monotonic clock domain.
    pub source_monotonic_ns: u64,
    /// Optional Unix correlation timestamp.
    pub publish_unix_ns: Option<u64>,
    /// Timestamp supplied by the non-realtime recording owner.
    pub record_monotonic_ns: u64,
}

/// Original protocol bytes plus bounded recording metadata.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TelemetryRecord {
    topic: ArrayString<MAX_TOPIC_BYTES>,
    payload: ArrayVec<u8, MAX_PAYLOAD_BYTES>,
    priority: TelemetryPriority,
    metadata: TelemetryMetadata,
}

impl TelemetryRecord {
    /// Copies bounded original Protobuf bytes without decoding or transforming them.
    pub fn new(
        topic: &str,
        payload: &[u8],
        priority: TelemetryPriority,
        metadata: TelemetryMetadata,
    ) -> Result<Self, RecordError> {
        if topic.is_empty() {
            return Err(RecordError::EmptyTopic);
        }
        if !topic.bytes().all(|byte| byte.is_ascii_graphic()) {
            return Err(RecordError::InvalidTopic);
        }
        let mut bounded_topic = ArrayString::new();
        bounded_topic
            .try_push_str(topic)
            .map_err(|_| RecordError::TopicTooLong {
                maximum: MAX_TOPIC_BYTES,
            })?;
        let payload = ArrayVec::try_from(payload).map_err(|_| RecordError::PayloadTooLarge {
            maximum: MAX_PAYLOAD_BYTES,
        })?;
        if metadata.sequence == 0 {
            return Err(RecordError::ZeroSequence);
        }
        Ok(Self {
            topic: bounded_topic,
            payload,
            priority,
            metadata,
        })
    }

    /// Returns the canonical topic.
    #[must_use]
    pub fn topic(&self) -> &str {
        self.topic.as_str()
    }

    /// Returns the unmodified encoded bytes.
    #[must_use]
    pub fn payload(&self) -> &[u8] {
        self.payload.as_slice()
    }

    /// Returns the bounded pressure class.
    #[must_use]
    pub const fn priority(&self) -> TelemetryPriority {
        self.priority
    }

    /// Returns the typed record metadata.
    #[must_use]
    pub const fn metadata(&self) -> TelemetryMetadata {
        self.metadata
    }
}

/// Invalid producer-provided record data.
#[derive(Clone, Copy, Debug, Error, Eq, PartialEq)]
pub enum RecordError {
    /// Topic names must not be empty.
    #[error("telemetry topic cannot be empty")]
    EmptyTopic,
    /// Topic names use printable ASCII to keep MCAP channel keys deterministic.
    #[error("telemetry topic contains a non-printable byte")]
    InvalidTopic,
    /// The topic exceeded fixed storage.
    #[error("telemetry topic exceeds {maximum} bytes")]
    TopicTooLong {
        /// Supported topic length.
        maximum: usize,
    },
    /// The original encoded message exceeded fixed producer storage.
    #[error("telemetry payload exceeds {maximum} bytes")]
    PayloadTooLarge {
        /// Supported payload length.
        maximum: usize,
    },
    /// Zero is reserved as an unspecified source sequence.
    #[error("telemetry sequence must be nonzero")]
    ZeroSequence,
}

/// Result of a producer's nonblocking ingress attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IngressOutcome {
    /// The record was retained for the non-realtime recorder.
    Queued,
    /// The priority class was full and a coalesced gap was incremented.
    Dropped {
        /// The independently saturated class.
        priority: TelemetryPriority,
    },
}

/// Bounded producer-side queues. It has no filesystem, clock, or waiting API.
#[derive(Debug, Default)]
pub struct TelemetryIngress {
    critical: ArrayVec<TelemetryRecord, MAX_CRITICAL_ITEMS>,
    control: ArrayVec<TelemetryRecord, MAX_CONTROL_ITEMS>,
    bulk: ArrayVec<TelemetryRecord, MAX_BULK_ITEMS>,
    critical_gaps: u64,
    control_gaps: u64,
    bulk_gaps: u64,
}

impl TelemetryIngress {
    /// Attempts to retain one record without waiting for storage or a consumer.
    pub fn try_push(&mut self, record: TelemetryRecord) -> IngressOutcome {
        let (queue, gaps, priority) = match record.priority() {
            TelemetryPriority::Critical => (
                &mut self.critical,
                &mut self.critical_gaps,
                TelemetryPriority::Critical,
            ),
            TelemetryPriority::Control => (
                &mut self.control,
                &mut self.control_gaps,
                TelemetryPriority::Control,
            ),
            TelemetryPriority::Bulk => {
                (&mut self.bulk, &mut self.bulk_gaps, TelemetryPriority::Bulk)
            }
        };
        if queue.try_push(record).is_ok() {
            IngressOutcome::Queued
        } else {
            *gaps = gaps.saturating_add(1);
            IngressOutcome::Dropped { priority }
        }
    }

    /// Drains bounded records in priority order on a non-realtime owner.
    pub fn drain_into(&mut self, recorder: &mut TelemetryRecorder) -> DrainReport {
        let mut report = DrainReport::default();
        for (priority, dropped) in [
            (TelemetryPriority::Critical, &mut self.critical_gaps),
            (TelemetryPriority::Control, &mut self.control_gaps),
            (TelemetryPriority::Bulk, &mut self.bulk_gaps),
        ] {
            if *dropped != 0 {
                recorder.record_gap(priority, *dropped);
                report.gaps_recorded = report.gaps_recorded.saturating_add(*dropped);
                *dropped = 0;
            }
        }
        drain_queue(&mut self.critical, recorder, &mut report);
        drain_queue(&mut self.control, recorder, &mut report);
        drain_queue(&mut self.bulk, recorder, &mut report);
        report
    }
}

fn drain_queue<const CAPACITY: usize>(
    queue: &mut ArrayVec<TelemetryRecord, CAPACITY>,
    recorder: &mut TelemetryRecorder,
    report: &mut DrainReport,
) {
    while !queue.is_empty() {
        let record = queue.remove(0);
        match recorder.append(&record) {
            AppendOutcome::Recorded => report.recorded = report.recorded.saturating_add(1),
            AppendOutcome::Dropped(_) => report.dropped = report.dropped.saturating_add(1),
        }
    }
}

/// Outcome observed by the recorder drain owner.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct DrainReport {
    /// Number of records written to an active staging output.
    pub recorded: u64,
    /// Number of records discarded because storage was unavailable.
    pub dropped: u64,
    /// Number of producer losses represented by explicit gap evidence.
    pub gaps_recorded: u64,
}

/// Immutable non-realtime file bounds for an output set.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RecorderConfig {
    output_directory: PathBuf,
    file_prefix: String,
    maximum_file_bytes: u64,
    quota_bytes: u64,
    chunk_size_bytes: u64,
}

impl RecorderConfig {
    /// Constructs bounded recording configuration for an existing absolute directory.
    pub fn new(
        output_directory: PathBuf,
        file_prefix: &str,
        maximum_file_bytes: u64,
        quota_bytes: u64,
        chunk_size_bytes: u64,
    ) -> Result<Self, RecorderError> {
        if !output_directory.is_absolute() {
            return Err(RecorderError::InvalidConfiguration {
                field: "output_directory",
            });
        }
        if file_prefix.is_empty()
            || !file_prefix
                .bytes()
                .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-'))
        {
            return Err(RecorderError::InvalidConfiguration {
                field: "file_prefix",
            });
        }
        if maximum_file_bytes == 0
            || quota_bytes < maximum_file_bytes
            || chunk_size_bytes == 0
            || chunk_size_bytes > MAX_CHUNK_SIZE_BYTES
        {
            return Err(RecorderError::InvalidConfiguration {
                field: "file_or_quota_bound",
            });
        }
        Ok(Self {
            output_directory,
            file_prefix: file_prefix.into(),
            maximum_file_bytes,
            quota_bytes,
            chunk_size_bytes,
        })
    }

    /// Returns the caller-owned output directory.
    #[must_use]
    pub fn output_directory(&self) -> &Path {
        &self.output_directory
    }
}

/// Typed storage degradation that remains outside producer authority.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RecorderFailure {
    /// The configured quota was reached before a record was accepted.
    Quota,
    /// A configured bounded record/channel limit was reached.
    Bound,
    /// MCAP serialization or storage failed.
    Storage,
}

/// Observable recorder availability.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RecorderHealth {
    /// A staging output is writable.
    Available,
    /// The recorder has failed closed for the rest of this instance.
    Degraded(RecorderFailure),
}

/// Result of one non-realtime append attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AppendOutcome {
    /// The record became part of the staging MCAP.
    Recorded,
    /// Storage could not accept the record; no producer is asked to retry.
    Dropped(RecorderFailure),
}

/// Startup staging-file recovery evidence.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RecoveryReport {
    /// Complete staged MCAPs sealed without replacing an existing output.
    pub sealed: u64,
    /// Invalid or conflicting staged files retained as failed evidence.
    pub retained_failed: u64,
}

/// Recorder setup, I/O, and replay-boundary failures.
#[derive(Debug, Error)]
pub enum RecorderError {
    /// A caller supplied an invalid non-realtime recording bound.
    #[error("invalid recorder configuration: {field}")]
    InvalidConfiguration {
        /// Invalid setting name.
        field: &'static str,
    },
    /// The output root was unavailable or did not have the required shape.
    #[error("recording output directory is unavailable")]
    OutputDirectory {
        #[source]
        source: std::io::Error,
    },
    /// An owned recording file operation failed.
    #[error("failed to {operation}")]
    Io {
        /// Stable operation classification.
        operation: &'static str,
        #[source]
        source: std::io::Error,
    },
    /// MCAP creation or serialization failed.
    #[error("MCAP operation failed")]
    Mcap(#[from] mcap::McapError),
    /// A recording attempted to create too many distinct MCAP channels.
    #[error("recording channel bound is exhausted")]
    ChannelBound,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ChannelKey {
    topic: ArrayString<MAX_TOPIC_BYTES>,
    source_id: SourceId,
    boot_id: Id128,
}

impl ChannelKey {
    const fn from_record(record: &TelemetryRecord) -> Self {
        Self {
            topic: record.topic,
            source_id: record.metadata.source_id,
            boot_id: record.metadata.boot_id,
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ChannelState {
    key: ChannelKey,
    channel_id: u16,
    next_sequence: u32,
}

struct ActiveOutput {
    staging_path: PathBuf,
    final_path: PathBuf,
    manifest_path: PathBuf,
    writer: Writer<BufWriter<File>>,
    schema_id: u16,
    estimated_bytes: u64,
    messages: u64,
    channels: ArrayVec<ChannelState, MAX_RECORDING_CHANNELS>,
    pending_metadata: BTreeMap<String, String>,
}

/// A non-realtime writer that creates, seals, rotates, and recovers MCAP files.
pub struct TelemetryRecorder {
    config: RecorderConfig,
    active: Option<ActiveOutput>,
    occupied_bytes: u64,
    next_file_number: u64,
    health: RecorderHealth,
}

impl TelemetryRecorder {
    /// Recovers only this config's staging names, then opens a new create-only output.
    pub fn open(config: RecorderConfig) -> Result<(Self, RecoveryReport), RecorderError> {
        let metadata = fs::metadata(config.output_directory())
            .map_err(|source| RecorderError::OutputDirectory { source })?;
        if !metadata.is_dir() {
            return Err(RecorderError::InvalidConfiguration {
                field: "output_directory",
            });
        }
        let recovery = recover_staging(&config)?;
        let occupied_bytes = owned_bytes(&config)?;
        let mut recorder = Self {
            config,
            active: None,
            occupied_bytes,
            next_file_number: 1,
            health: RecorderHealth::Available,
        };
        recorder.open_next_output()?;
        Ok((recorder, recovery))
    }

    /// Returns typed availability without formatting or command authority.
    #[must_use]
    pub const fn health(&self) -> RecorderHealth {
        self.health
    }

    /// Records one explicit priority-class gap on the non-realtime drain path.
    pub fn record_gap(&mut self, priority: TelemetryPriority, dropped: u64) {
        if dropped == 0 || !matches!(self.health, RecorderHealth::Available) {
            return;
        }
        if self.occupied_bytes.saturating_add(GAP_RECORD_RESERVE_BYTES) > self.config.quota_bytes {
            self.health = RecorderHealth::Degraded(RecorderFailure::Quota);
            return;
        }
        let needs_rotation = self.active.as_ref().is_some_and(|active| {
            active.messages != 0
                && active
                    .estimated_bytes
                    .saturating_add(GAP_RECORD_RESERVE_BYTES)
                    > self.config.maximum_file_bytes
        });
        let result = if needs_rotation {
            self.seal_active().and_then(|_| self.open_next_output())
        } else {
            Ok(())
        }
        .and_then(|_| self.record_gap_inner(priority, dropped));
        match result {
            Ok(()) => {
                let active = self
                    .active
                    .as_mut()
                    .expect("active output remains after gap write");
                active.estimated_bytes = active
                    .estimated_bytes
                    .saturating_add(GAP_RECORD_RESERVE_BYTES);
                self.occupied_bytes = self.occupied_bytes.saturating_add(GAP_RECORD_RESERVE_BYTES);
            }
            Err(_) => self.health = RecorderHealth::Degraded(RecorderFailure::Storage),
        }
    }

    /// Appends an already-encoded message without requiring producer retry.
    pub fn append(&mut self, record: &TelemetryRecord) -> AppendOutcome {
        if let RecorderHealth::Degraded(failure) = self.health {
            return AppendOutcome::Dropped(failure);
        }
        let reserve = estimated_record_bytes(record);
        if self.occupied_bytes.saturating_add(reserve) > self.config.quota_bytes {
            self.health = RecorderHealth::Degraded(RecorderFailure::Quota);
            return AppendOutcome::Dropped(RecorderFailure::Quota);
        }
        let result = self.append_inner(record, reserve);
        match result {
            Ok(()) => AppendOutcome::Recorded,
            Err(error) => {
                let failure = if matches!(error, RecorderError::ChannelBound) {
                    RecorderFailure::Bound
                } else {
                    RecorderFailure::Storage
                };
                self.health = RecorderHealth::Degraded(failure);
                AppendOutcome::Dropped(failure)
            }
        }
    }

    /// Flushes record metadata and seals the current staging file atomically.
    pub fn seal(&mut self) -> Result<PathBuf, RecorderError> {
        self.seal_active()
    }

    fn append_inner(
        &mut self,
        record: &TelemetryRecord,
        reserve: u64,
    ) -> Result<(), RecorderError> {
        let needs_rotation = self.active.as_ref().is_some_and(|active| {
            active.messages != 0
                && active.estimated_bytes.saturating_add(reserve) > self.config.maximum_file_bytes
        });
        if needs_rotation {
            self.seal_active()?;
            self.open_next_output()?;
        }

        let active = self
            .active
            .as_mut()
            .ok_or(RecorderError::InvalidConfiguration {
                field: "active_output",
            })?;
        let key = ChannelKey::from_record(record);
        let (channel_id, channel_sequence) = if let Some(index) = active
            .channels
            .iter()
            .position(|channel| channel.key == key)
        {
            let channel = &mut active.channels[index];
            channel.next_sequence = channel.next_sequence.checked_add(1).ok_or(
                RecorderError::InvalidConfiguration {
                    field: "mcap_channel_sequence",
                },
            )?;
            (channel.channel_id, channel.next_sequence)
        } else {
            if active.channels.is_full() {
                return Err(RecorderError::ChannelBound);
            }
            let mut channel_metadata = BTreeMap::new();
            channel_metadata.insert("robot.boot_id".into(), hex_id(record.metadata.boot_id));
            channel_metadata.insert(
                "robot.schema_sha256".into(),
                hex_digest(&descriptor_digest()),
            );
            channel_metadata.insert(
                "robot.source_id".into(),
                record.metadata.source_id.as_str().into(),
            );
            let channel_id = active.writer.add_channel(
                active.schema_id,
                record.topic(),
                "protobuf",
                &channel_metadata,
            )?;
            active
                .channels
                .try_push(ChannelState {
                    key,
                    channel_id,
                    next_sequence: 1,
                })
                .expect("checked fixed channel capacity");
            (channel_id, 1)
        };
        active.writer.write_to_known_channel(
            &MessageHeader {
                channel_id,
                sequence: channel_sequence,
                log_time: record.metadata.record_monotonic_ns,
                publish_time: record.metadata.publish_unix_ns.unwrap_or(0),
            },
            record.payload(),
        )?;
        let key = format!("record.{channel_id}.{channel_sequence}");
        active.pending_metadata.insert(
            format!("{key}.sequence"),
            record.metadata.sequence.to_string(),
        );
        active.pending_metadata.insert(
            format!("{key}.source_monotonic_ns"),
            record.metadata.source_monotonic_ns.to_string(),
        );
        active.pending_metadata.insert(
            format!("{key}.record_monotonic_ns"),
            record.metadata.record_monotonic_ns.to_string(),
        );
        active.pending_metadata.insert(
            format!("{key}.publish_unix_ns"),
            record
                .metadata
                .publish_unix_ns
                .map_or_else(|| "none".into(), |timestamp| timestamp.to_string()),
        );
        active.estimated_bytes = active.estimated_bytes.saturating_add(reserve);
        active.messages = active.messages.saturating_add(1);
        self.occupied_bytes = self.occupied_bytes.saturating_add(reserve);
        if active.pending_metadata.len() >= METADATA_BATCH_ITEMS * 4 {
            flush_metadata(active)?;
        }
        Ok(())
    }

    fn record_gap_inner(
        &mut self,
        priority: TelemetryPriority,
        dropped: u64,
    ) -> Result<(), RecorderError> {
        let active = self
            .active
            .as_mut()
            .ok_or(RecorderError::InvalidConfiguration {
                field: "active_output",
            })?;
        let mut metadata = BTreeMap::new();
        metadata.insert("priority".into(), priority.label().into());
        metadata.insert("dropped".into(), dropped.to_string());
        active.writer.write_metadata(&Metadata {
            name: "robot.telemetry-gap-v1".into(),
            metadata,
        })?;
        Ok(())
    }

    fn open_next_output(&mut self) -> Result<(), RecorderError> {
        loop {
            let file_number = self.next_file_number;
            self.next_file_number = self.next_file_number.saturating_add(1);
            let base_name = format!("{}-{file_number:020}.mcap", self.config.file_prefix);
            let final_path = self.config.output_directory.join(&base_name);
            let staging_path = self
                .config
                .output_directory
                .join(format!("{base_name}.staging"));
            let manifest_path = self
                .config
                .output_directory
                .join(format!("{base_name}.sha256"));
            if final_path.exists() || staging_path.exists() || manifest_path.exists() {
                continue;
            }
            let file = OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&staging_path)
                .map_err(|source| RecorderError::Io {
                    operation: "create staging MCAP",
                    source,
                })?;
            let writer = WriteOptions::new()
                .use_chunks(true)
                .chunk_size(Some(self.config.chunk_size_bytes))
                .create(BufWriter::new(file))?;
            let mut writer = writer;
            let schema_id = writer.add_schema("robot.v1", "protobuf", FILE_DESCRIPTOR_SET)?;
            self.active = Some(ActiveOutput {
                staging_path,
                final_path,
                manifest_path,
                writer,
                schema_id,
                estimated_bytes: 0,
                messages: 0,
                channels: ArrayVec::new(),
                pending_metadata: BTreeMap::new(),
            });
            return Ok(());
        }
    }

    fn seal_active(&mut self) -> Result<PathBuf, RecorderError> {
        let mut active = self
            .active
            .take()
            .ok_or(RecorderError::InvalidConfiguration {
                field: "active_output",
            })?;
        flush_metadata(&mut active)?;
        active.writer.finish()?;
        let buffer = active.writer.into_inner();
        let file = buffer.into_inner().map_err(|error| RecorderError::Io {
            operation: "flush staging MCAP",
            source: error.into_error(),
        })?;
        file.sync_all().map_err(|source| RecorderError::Io {
            operation: "sync staging MCAP",
            source,
        })?;
        fs::rename(&active.staging_path, &active.final_path).map_err(|source| {
            RecorderError::Io {
                operation: "seal staging MCAP",
                source,
            }
        })?;
        write_manifest(&active.final_path, &active.manifest_path)?;
        self.occupied_bytes = owned_bytes(&self.config)?;
        Ok(active.final_path)
    }
}

fn flush_metadata(active: &mut ActiveOutput) -> Result<(), RecorderError> {
    if active.pending_metadata.is_empty() {
        return Ok(());
    }
    let metadata = std::mem::take(&mut active.pending_metadata);
    active.writer.write_metadata(&Metadata {
        name: "robot.telemetry-records-v1".into(),
        metadata,
    })?;
    Ok(())
}

/// Recovers complete task-owned staging files and retains malformed ones as evidence.
pub fn recover_staging(config: &RecorderConfig) -> Result<RecoveryReport, RecorderError> {
    let mut report = RecoveryReport::default();
    for staging_path in owned_staging_paths(config)? {
        let valid = fs::read(&staging_path)
            .map_err(|source| RecorderError::Io {
                operation: "read staging MCAP",
                source,
            })
            .and_then(|bytes| validate_complete_mcap(&bytes));
        let suffix = ".staging";
        let final_path = PathBuf::from(staging_path.to_string_lossy().strip_suffix(suffix).ok_or(
            RecorderError::InvalidConfiguration {
                field: "staging_filename",
            },
        )?);
        if valid.is_ok() && !final_path.exists() {
            let manifest_path = PathBuf::from(format!("{}.sha256", final_path.to_string_lossy()));
            if manifest_path.exists() {
                retain_failed_staging(&staging_path)?;
                report.retained_failed = report.retained_failed.saturating_add(1);
                continue;
            }
            fs::rename(&staging_path, &final_path).map_err(|source| RecorderError::Io {
                operation: "seal recovered staging MCAP",
                source,
            })?;
            write_manifest(&final_path, &manifest_path)?;
            report.sealed = report.sealed.saturating_add(1);
        } else {
            retain_failed_staging(&staging_path)?;
            report.retained_failed = report.retained_failed.saturating_add(1);
        }
    }
    Ok(report)
}

fn validate_complete_mcap(bytes: &[u8]) -> Result<(), RecorderError> {
    let reader = ChunkFlattener::new(bytes)?;
    for record in reader {
        record?;
    }
    Ok(())
}

fn retain_failed_staging(staging_path: &Path) -> Result<(), RecorderError> {
    let failed_path = PathBuf::from(format!("{}.failed", staging_path.to_string_lossy()));
    if failed_path.exists() {
        return Err(RecorderError::InvalidConfiguration {
            field: "failed_evidence_collision",
        });
    }
    fs::rename(staging_path, failed_path).map_err(|source| RecorderError::Io {
        operation: "retain failed staging MCAP",
        source,
    })
}

fn owned_staging_paths(config: &RecorderConfig) -> Result<Vec<PathBuf>, RecorderError> {
    let mut paths = Vec::new();
    for entry in fs::read_dir(config.output_directory()).map_err(|source| RecorderError::Io {
        operation: "scan recording directory",
        source,
    })? {
        let entry = entry.map_err(|source| RecorderError::Io {
            operation: "read recording directory entry",
            source,
        })?;
        let file_name = entry.file_name();
        let file_name = file_name.to_string_lossy();
        if file_name.starts_with(&config.file_prefix)
            && file_name.ends_with(".mcap.staging")
            && entry
                .file_type()
                .map_err(|source| RecorderError::Io {
                    operation: "inspect recording directory entry",
                    source,
                })?
                .is_file()
        {
            paths.push(entry.path());
        }
    }
    paths.sort();
    Ok(paths)
}

fn owned_bytes(config: &RecorderConfig) -> Result<u64, RecorderError> {
    let mut total = 0_u64;
    for entry in fs::read_dir(config.output_directory()).map_err(|source| RecorderError::Io {
        operation: "scan recording directory",
        source,
    })? {
        let entry = entry.map_err(|source| RecorderError::Io {
            operation: "read recording directory entry",
            source,
        })?;
        let file_name = entry.file_name();
        let file_name = file_name.to_string_lossy();
        if file_name.starts_with(&config.file_prefix)
            && (file_name.ends_with(".mcap")
                || file_name.ends_with(".mcap.staging")
                || file_name.ends_with(".mcap.sha256")
                || file_name.ends_with(".mcap.staging.failed"))
        {
            total = total.saturating_add(
                entry
                    .metadata()
                    .map_err(|source| RecorderError::Io {
                        operation: "read recording file metadata",
                        source,
                    })?
                    .len(),
            );
        }
    }
    Ok(total)
}

fn write_manifest(final_path: &Path, manifest_path: &Path) -> Result<(), RecorderError> {
    let digest = sha256_file(final_path)?;
    let file_name = final_path
        .file_name()
        .and_then(|name| name.to_str())
        .ok_or(RecorderError::InvalidConfiguration {
            field: "final_filename",
        })?;
    let mut manifest = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(manifest_path)
        .map_err(|source| RecorderError::Io {
            operation: "create recording manifest",
            source,
        })?;
    writeln!(manifest, "{}  {file_name}", hex_digest(&digest)).map_err(|source| {
        RecorderError::Io {
            operation: "write recording manifest",
            source,
        }
    })?;
    manifest.sync_all().map_err(|source| RecorderError::Io {
        operation: "sync recording manifest",
        source,
    })
}

fn sha256_file(path: &Path) -> Result<DescriptorDigest, RecorderError> {
    let bytes = fs::read(path).map_err(|source| RecorderError::Io {
        operation: "hash recording output",
        source,
    })?;
    Ok(Sha256::digest(bytes).into())
}

fn estimated_record_bytes(record: &TelemetryRecord) -> u64 {
    u64::try_from(record.payload().len())
        .unwrap_or(u64::MAX)
        .saturating_add(1_024)
}

fn hex_digest(digest: &DescriptorDigest) -> String {
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn hex_id(identity: Id128) -> String {
    identity
        .as_bytes()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

/// A replay-only bounded input. It has no command, driver, or sink capability.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ReplayInput {
    /// Topic admitted by the caller's immutable allowlist.
    pub topic: ArrayString<MAX_TOPIC_BYTES>,
    /// Unmodified original Protobuf bytes.
    pub payload: ArrayVec<u8, MAX_PAYLOAD_BYTES>,
    /// Preserved source and recording metadata.
    pub metadata: TelemetryMetadata,
}

/// Immutable replay validation settings.
#[derive(Clone, Debug)]
pub struct ReplayPolicy {
    expected_descriptor_digest: DescriptorDigest,
    allowed_topics: Vec<String>,
    maximum_inputs: usize,
}

impl ReplayPolicy {
    /// Requires a nonempty immutable topic allowlist and an explicit item bound.
    pub fn new(
        expected_descriptor_digest: DescriptorDigest,
        allowed_topics: &[&str],
        maximum_inputs: usize,
    ) -> Result<Self, ReplayError> {
        if allowed_topics.is_empty() || maximum_inputs == 0 {
            return Err(ReplayError::InvalidPolicy);
        }
        let mut topics = Vec::with_capacity(allowed_topics.len());
        for topic in allowed_topics {
            if topic.is_empty() || !topic.bytes().all(|byte| byte.is_ascii_graphic()) {
                return Err(ReplayError::InvalidPolicy);
            }
            topics.push((*topic).into());
        }
        topics.sort();
        topics.dedup();
        Ok(Self {
            expected_descriptor_digest,
            allowed_topics: topics,
            maximum_inputs,
        })
    }
}

/// Strict replay validation failure.
#[derive(Debug, Error)]
pub enum ReplayError {
    /// Replay only reads sealed files.
    #[error("replay requires a sealed .mcap output")]
    UnsealedPath,
    /// The replay policy is missing a bound or topic authority.
    #[error("replay policy is invalid")]
    InvalidPolicy,
    /// Input/output failed at the non-realtime replay boundary.
    #[error("replay I/O failed")]
    Io {
        #[source]
        source: std::io::Error,
    },
    /// MCAP structure, magic, or CRC validation failed.
    #[error("MCAP validation failed")]
    Mcap(#[from] mcap::McapError),
    /// The supplied descriptor identity does not match the compiled contract.
    #[error("replay descriptor identity is incompatible")]
    DescriptorMismatch,
    /// The MCAP schema/channel/metadata contract is malformed.
    #[error("replay metadata is incompatible: {field}")]
    Incompatible {
        /// Stable failure location.
        field: &'static str,
    },
    /// A record exceeded a public replay bound.
    #[error("replay input exceeded a configured bound")]
    BoundExceeded,
}

#[derive(Clone)]
struct ReplayChannel {
    topic: ArrayString<MAX_TOPIC_BYTES>,
    source_id: SourceId,
    boot_id: Id128,
}

struct PendingMessage {
    channel_id: u16,
    mcap_sequence: u32,
    record_time_ns: u64,
    publish_time_ns: u64,
    payload: ArrayVec<u8, MAX_PAYLOAD_BYTES>,
}

/// Reads validated replay-only inputs in file order. It cannot construct output commands.
pub fn replay_file(path: &Path, policy: &ReplayPolicy) -> Result<Vec<ReplayInput>, ReplayError> {
    if path.extension().and_then(|extension| extension.to_str()) != Some("mcap") {
        return Err(ReplayError::UnsealedPath);
    }
    if policy.expected_descriptor_digest != descriptor_digest() {
        return Err(ReplayError::DescriptorMismatch);
    }
    let bytes = fs::read(path).map_err(|source| ReplayError::Io { source })?;
    let reader = ChunkFlattener::new(&bytes)?;
    let mut schema_seen = false;
    let mut schema_ids = BTreeSet::new();
    let mut channels = BTreeMap::new();
    let mut records = Vec::new();
    let mut metadata = BTreeMap::new();
    for record in reader {
        match record? {
            Record::Schema { header, data } => {
                if header.name != "robot.v1"
                    || header.encoding != "protobuf"
                    || data.as_ref() != FILE_DESCRIPTOR_SET
                {
                    return Err(ReplayError::DescriptorMismatch);
                }
                schema_seen = true;
                schema_ids.insert(header.id);
            }
            Record::Channel(channel) => {
                if !schema_ids.contains(&channel.schema_id) {
                    return Err(ReplayError::DescriptorMismatch);
                }
                let parsed = parse_replay_channel(&channel, policy)?;
                if let Some(existing) = channels.insert(channel.id, parsed.clone())
                    && (existing.topic != parsed.topic
                        || existing.source_id != parsed.source_id
                        || existing.boot_id != parsed.boot_id)
                {
                    return Err(ReplayError::Incompatible { field: "channel" });
                }
            }
            Record::Message { header, data } => {
                if records.len() >= policy.maximum_inputs {
                    return Err(ReplayError::BoundExceeded);
                }
                let payload =
                    ArrayVec::try_from(data.as_ref()).map_err(|_| ReplayError::BoundExceeded)?;
                records.push(PendingMessage {
                    channel_id: header.channel_id,
                    mcap_sequence: header.sequence,
                    record_time_ns: header.log_time,
                    publish_time_ns: header.publish_time,
                    payload,
                });
            }
            Record::Metadata(entry) if entry.name == "robot.telemetry-records-v1" => {
                for (key, value) in entry.metadata {
                    if metadata.insert(key, value).is_some() {
                        return Err(ReplayError::Incompatible {
                            field: "record_metadata_duplicate",
                        });
                    }
                }
            }
            Record::Metadata(entry) if entry.name == "robot.telemetry-gap-v1" => {
                if !entry.metadata.contains_key("priority")
                    || !entry.metadata.contains_key("dropped")
                {
                    return Err(ReplayError::Incompatible {
                        field: "gap_metadata",
                    });
                }
            }
            Record::Unknown { .. } => return Err(ReplayError::Incompatible { field: "record" }),
            _ => {}
        }
    }
    if !schema_seen {
        return Err(ReplayError::DescriptorMismatch);
    }
    let mut replayed = Vec::with_capacity(records.len());
    let mut previous_record_time = 0_u64;
    let mut channel_sequences = BTreeMap::<u16, u64>::new();
    for record in records {
        let channel = channels
            .get(&record.channel_id)
            .ok_or(ReplayError::Incompatible {
                field: "message_channel",
            })?;
        if policy
            .allowed_topics
            .binary_search_by(|topic| topic.as_str().cmp(channel.topic.as_str()))
            .is_err()
        {
            return Err(ReplayError::Incompatible {
                field: "topic_allowlist",
            });
        }
        let key = format!("record.{}.{}", record.channel_id, record.mcap_sequence);
        let sequence = parse_metadata_u64(&metadata, &format!("{key}.sequence"))?;
        let source_monotonic_ns =
            parse_metadata_u64(&metadata, &format!("{key}.source_monotonic_ns"))?;
        let record_monotonic_ns =
            parse_metadata_u64(&metadata, &format!("{key}.record_monotonic_ns"))?;
        let publish_unix_ns =
            parse_optional_metadata_u64(&metadata, &format!("{key}.publish_unix_ns"))?;
        if record.record_time_ns != record_monotonic_ns
            || record.publish_time_ns != publish_unix_ns.unwrap_or(0)
            || record_monotonic_ns < previous_record_time
        {
            return Err(ReplayError::Incompatible {
                field: "record_time",
            });
        }
        previous_record_time = record_monotonic_ns;
        let previous_sequence = channel_sequences.entry(record.channel_id).or_insert(0);
        if sequence <= *previous_sequence {
            return Err(ReplayError::Incompatible { field: "sequence" });
        }
        *previous_sequence = sequence;
        replayed.push(ReplayInput {
            topic: channel.topic,
            payload: record.payload,
            metadata: TelemetryMetadata {
                sequence,
                source_id: channel.source_id,
                boot_id: channel.boot_id,
                source_monotonic_ns,
                publish_unix_ns,
                record_monotonic_ns,
            },
        });
    }
    Ok(replayed)
}

fn parse_replay_channel(
    channel: &mcap::records::Channel,
    policy: &ReplayPolicy,
) -> Result<ReplayChannel, ReplayError> {
    if channel.message_encoding != "protobuf"
        || channel.metadata.get("robot.schema_sha256")
            != Some(&hex_digest(&policy.expected_descriptor_digest))
    {
        return Err(ReplayError::DescriptorMismatch);
    }
    let mut topic = ArrayString::new();
    topic
        .try_push_str(&channel.topic)
        .map_err(|_| ReplayError::BoundExceeded)?;
    let source = channel
        .metadata
        .get("robot.source_id")
        .ok_or(ReplayError::Incompatible {
            field: "channel_source",
        })?;
    let source_id = SourceId::parse(source).map_err(|_| ReplayError::Incompatible {
        field: "channel_source",
    })?;
    let boot = channel
        .metadata
        .get("robot.boot_id")
        .ok_or(ReplayError::Incompatible {
            field: "channel_boot",
        })?;
    let boot_id = parse_hex_id(boot)?;
    Ok(ReplayChannel {
        topic,
        source_id,
        boot_id,
    })
}

fn parse_metadata_u64(metadata: &BTreeMap<String, String>, key: &str) -> Result<u64, ReplayError> {
    metadata
        .get(key)
        .ok_or(ReplayError::Incompatible {
            field: "record_metadata",
        })?
        .parse()
        .map_err(|_| ReplayError::Incompatible {
            field: "record_metadata",
        })
}

fn parse_optional_metadata_u64(
    metadata: &BTreeMap<String, String>,
    key: &str,
) -> Result<Option<u64>, ReplayError> {
    let value = metadata.get(key).ok_or(ReplayError::Incompatible {
        field: "record_metadata",
    })?;
    if value == "none" {
        Ok(None)
    } else {
        value
            .parse()
            .map(Some)
            .map_err(|_| ReplayError::Incompatible {
                field: "record_metadata",
            })
    }
}

fn parse_hex_id(value: &str) -> Result<Id128, ReplayError> {
    if value.len() != 32 {
        return Err(ReplayError::Incompatible {
            field: "channel_boot",
        });
    }
    let mut bytes = [0_u8; 16];
    for (index, pair) in value.as_bytes().chunks_exact(2).enumerate() {
        let high = hex_nibble(pair[0]).ok_or(ReplayError::Incompatible {
            field: "channel_boot",
        })?;
        let low = hex_nibble(pair[1]).ok_or(ReplayError::Incompatible {
            field: "channel_boot",
        })?;
        bytes[index] = (high << 4) | low;
    }
    Id128::new(bytes).map_err(|_| ReplayError::Incompatible {
        field: "channel_boot",
    })
}

const fn hex_nibble(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use std::fs;

    use message_model::AsciiId;
    use tempfile::tempdir;

    use super::*;

    fn metadata(sequence: u64, record_monotonic_ns: u64) -> TelemetryMetadata {
        TelemetryMetadata {
            sequence,
            source_id: AsciiId::parse("synthetic-source").unwrap(),
            boot_id: Id128::new([7; 16]).unwrap(),
            source_monotonic_ns: record_monotonic_ns - 1,
            publish_unix_ns: Some(1_700_000_000_000_000_000),
            record_monotonic_ns,
        }
    }

    fn config(directory: &Path) -> RecorderConfig {
        RecorderConfig::new(
            directory.to_path_buf(),
            "host-motion",
            32_768,
            131_072,
            1_024,
        )
        .unwrap()
    }

    #[test]
    fn writes_seals_and_replays_original_protocol_bytes() {
        let directory = tempdir().unwrap();
        let (mut recorder, recovery) = TelemetryRecorder::open(config(directory.path())).unwrap();
        assert_eq!(recovery, RecoveryReport::default());
        let first = TelemetryRecord::new(
            "/robot/state",
            &[0x08, 0x01],
            TelemetryPriority::Control,
            metadata(7, 100),
        )
        .unwrap();
        let second = TelemetryRecord::new(
            "/robot/state",
            &[0x08, 0x02],
            TelemetryPriority::Control,
            metadata(8, 101),
        )
        .unwrap();
        assert_eq!(recorder.append(&first), AppendOutcome::Recorded);
        assert_eq!(recorder.append(&second), AppendOutcome::Recorded);
        let sealed = recorder.seal().unwrap();
        assert!(sealed.exists());
        assert!(sealed.with_extension("mcap.sha256").exists());

        let policy = ReplayPolicy::new(descriptor_digest(), &["/robot/state"], 4).unwrap();
        let replayed = replay_file(&sealed, &policy).unwrap();
        assert_eq!(replayed.len(), 2);
        assert_eq!(replayed[0].payload.as_slice(), &[0x08, 0x01]);
        assert_eq!(replayed[1].metadata.sequence, 8);
        assert_eq!(replayed[1].metadata.record_monotonic_ns, 101);
        let wrong_policy = ReplayPolicy::new([0; 32], &["/robot/state"], 4).unwrap();
        assert!(matches!(
            replay_file(&sealed, &wrong_policy),
            Err(ReplayError::DescriptorMismatch)
        ));
    }

    #[test]
    fn ingress_coalesces_pressure_and_recorder_degrades_without_producer_retry() {
        let directory = tempdir().unwrap();
        let small_config = RecorderConfig::new(
            directory.path().to_path_buf(),
            "host-motion",
            2_048,
            2_048,
            1_024,
        )
        .unwrap();
        let (mut recorder, _) = TelemetryRecorder::open(small_config).unwrap();
        let mut ingress = TelemetryIngress::default();
        for sequence in 1..=u64::try_from(MAX_BULK_ITEMS + 1).unwrap() {
            let record = TelemetryRecord::new(
                "/robot/bulk",
                &[1],
                TelemetryPriority::Bulk,
                metadata(sequence, sequence),
            )
            .unwrap();
            let _ = ingress.try_push(record);
        }
        let report = ingress.drain_into(&mut recorder);
        assert!(report.gaps_recorded >= 1);
        assert!(report.dropped >= 1);
        assert_eq!(
            recorder.health(),
            RecorderHealth::Degraded(RecorderFailure::Quota)
        );
    }

    #[test]
    fn recording_bounds_channel_cardinality_and_chunk_configuration() {
        let directory = tempdir().unwrap();
        assert!(
            RecorderConfig::new(
                directory.path().to_path_buf(),
                "host-motion",
                2_000_000,
                2_000_000,
                MAX_CHUNK_SIZE_BYTES + 1,
            )
            .is_err()
        );
        let config = RecorderConfig::new(
            directory.path().to_path_buf(),
            "host-motion",
            2_000_000,
            2_000_000,
            1_024,
        )
        .unwrap();
        let (mut recorder, _) = TelemetryRecorder::open(config).unwrap();
        for sequence in 1..=u64::try_from(MAX_RECORDING_CHANNELS).unwrap() {
            let topic = format!("/robot/channel-{sequence}");
            let record = TelemetryRecord::new(
                &topic,
                &[1],
                TelemetryPriority::Bulk,
                metadata(sequence, sequence),
            )
            .unwrap();
            assert_eq!(recorder.append(&record), AppendOutcome::Recorded);
        }
        let over_bound = TelemetryRecord::new(
            "/robot/channel-over",
            &[1],
            TelemetryPriority::Bulk,
            metadata(65, 65),
        )
        .unwrap();
        assert_eq!(
            recorder.append(&over_bound),
            AppendOutcome::Dropped(RecorderFailure::Bound)
        );
        assert_eq!(
            recorder.health(),
            RecorderHealth::Degraded(RecorderFailure::Bound)
        );
    }

    #[test]
    fn complete_staging_is_recovered_but_malformed_staging_is_retained() {
        let directory = tempdir().unwrap();
        let (mut recorder, _) = TelemetryRecorder::open(config(directory.path())).unwrap();
        let record = TelemetryRecord::new(
            "/robot/state",
            &[1, 2, 3],
            TelemetryPriority::Critical,
            metadata(1, 10),
        )
        .unwrap();
        assert_eq!(recorder.append(&record), AppendOutcome::Recorded);
        let sealed = recorder.seal().unwrap();
        let staging = PathBuf::from(format!("{}.staging", sealed.to_string_lossy()));
        fs::rename(&sealed, &staging).unwrap();
        fs::remove_file(sealed.with_extension("mcap.sha256")).unwrap();
        let recovered = recover_staging(&config(directory.path())).unwrap();
        assert_eq!(recovered.sealed, 1);
        assert!(sealed.exists());

        let broken = directory
            .path()
            .join("host-motion-99999999999999999999.mcap.staging");
        fs::write(&broken, b"not an mcap").unwrap();
        let failed = recover_staging(&config(directory.path())).unwrap();
        assert_eq!(failed.retained_failed, 1);
        assert!(PathBuf::from(format!("{}.failed", broken.to_string_lossy())).exists());
    }

    #[test]
    fn replay_rejects_unsealed_or_incompatible_input() {
        let directory = tempdir().unwrap();
        let invalid = directory.path().join("invalid.mcap");
        fs::write(&invalid, b"not an mcap").unwrap();
        let policy = ReplayPolicy::new(descriptor_digest(), &["/robot/state"], 1).unwrap();
        assert!(matches!(
            replay_file(&invalid, &policy),
            Err(ReplayError::Mcap(_))
        ));
        let staging = directory.path().join("invalid.mcap.staging");
        assert!(matches!(
            replay_file(&staging, &policy),
            Err(ReplayError::UnsealedPath)
        ));
        assert!(ReplayPolicy::new([0; 32], &["/robot/state"], 1).is_ok());
    }
}
