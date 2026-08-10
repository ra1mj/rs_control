//! The complete mmap/raw-pointer exception for layout v1.
#![allow(unsafe_code)]
#![deny(unsafe_op_in_unsafe_fn)]

use std::{
    fs::File,
    hint::black_box,
    marker::PhantomData,
    mem::{MaybeUninit, size_of},
    sync::atomic::{AtomicU8, AtomicU32, AtomicU64, Ordering, fence},
};

use memmap2::{Mmap, MmapMut, MmapOptions};
use middleware_api::ChannelCounters;

use crate::{
    MemoryPolicy, ShmError, ShmPayload,
    layout::{HEADER_BYTES, LayoutDescriptor, LayoutSnapshot},
};

const PAGE_BYTES: usize = 4_096;
pub(crate) const READ_ATTEMPTS: usize = 16;

#[repr(C, align(64))]
struct IdentityLine {
    ready_magic: AtomicU64,
    version: u32,
    endian: u32,
    atomic_width: u32,
    channel_kind: u32,
    capacity: u32,
    schema_version: u32,
    payload_size: u32,
    payload_alignment: u32,
    slot_stride: u32,
    schema_id: [u8; 16],
    _padding: [u8; 4],
}

#[repr(C, align(64))]
struct ProducerLine {
    head: AtomicU64,
    published: AtomicU64,
    overwritten: AtomicU64,
    full: AtomicU64,
    disconnected: AtomicU64,
    epoch: AtomicU64,
    pid: AtomicU32,
    active: AtomicU32,
    _padding: [u8; 8],
}

#[repr(C, align(64))]
struct ConsumerLine {
    tail: AtomicU64,
    consumed: AtomicU64,
    last_consumed: AtomicU64,
    read_races: AtomicU64,
    decode_rejected: AtomicU64,
    skipped_backlog: AtomicU64,
    epoch: AtomicU64,
    pid: AtomicU32,
    active: AtomicU32,
}

#[repr(C, align(64))]
struct MetadataLine {
    slots_offset: u64,
    versions_offset: u64,
    total_length: u64,
    reclaimed_endpoints: AtomicU64,
    attach_rejected: AtomicU64,
    _padding: [u8; 24],
}

#[repr(C, align(64))]
struct Header {
    identity: IdentityLine,
    producer: ProducerLine,
    consumer: ConsumerLine,
    metadata: MetadataLine,
}

const _: () = assert!(size_of::<IdentityLine>() == 64);
const _: () = assert!(size_of::<ProducerLine>() == 64);
const _: () = assert!(size_of::<ConsumerLine>() == 64);
const _: () = assert!(size_of::<MetadataLine>() == 64);
const _: () = assert!(size_of::<Header>() == HEADER_BYTES);

impl Header {
    fn new(layout: LayoutSnapshot) -> Self {
        Self {
            identity: IdentityLine {
                ready_magic: AtomicU64::new(0),
                version: layout.version,
                endian: layout.endian,
                atomic_width: layout.atomic_width,
                channel_kind: layout.channel_kind,
                capacity: layout.capacity,
                schema_version: layout.schema_version,
                payload_size: layout.payload_size,
                payload_alignment: layout.payload_alignment,
                slot_stride: layout.slot_stride,
                schema_id: layout.schema_id,
                _padding: [0; 4],
            },
            producer: ProducerLine {
                head: AtomicU64::new(0),
                published: AtomicU64::new(0),
                overwritten: AtomicU64::new(0),
                full: AtomicU64::new(0),
                disconnected: AtomicU64::new(0),
                epoch: AtomicU64::new(0),
                pid: AtomicU32::new(0),
                active: AtomicU32::new(0),
                _padding: [0; 8],
            },
            consumer: ConsumerLine {
                tail: AtomicU64::new(0),
                consumed: AtomicU64::new(0),
                last_consumed: AtomicU64::new(0),
                read_races: AtomicU64::new(0),
                decode_rejected: AtomicU64::new(0),
                skipped_backlog: AtomicU64::new(0),
                epoch: AtomicU64::new(0),
                pid: AtomicU32::new(0),
                active: AtomicU32::new(0),
            },
            metadata: MetadataLine {
                slots_offset: layout.slots_offset,
                versions_offset: layout.versions_offset,
                total_length: layout.total_length,
                reclaimed_endpoints: AtomicU64::new(0),
                attach_rejected: AtomicU64::new(0),
                _padding: [0; 24],
            },
        }
    }
}

/// Validated mapping that owns both the file and its shared lifetime lock.
pub(crate) struct MappedRegion {
    map: MmapMut,
    layout: LayoutDescriptor,
    _region_file: File,
    _region_lock: File,
}

/// Read-only validated mapping used by unprivileged diagnostic observers.
///
/// Unlike a subscriber endpoint, this mapping never claims a consumer lease,
/// updates consumption metadata, or records decode diagnostics in shared
/// memory. It is intentionally limited to latest-value observation.
pub(crate) struct ReadOnlyMappedRegion {
    map: Mmap,
    layout: LayoutDescriptor,
    _region_file: File,
    _region_lock: File,
}

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub(crate) struct RawDiagnostics {
    pub(crate) attach_rejected: u64,
    pub(crate) read_races: u64,
    pub(crate) decode_rejected: u64,
    pub(crate) reclaimed_endpoints: u64,
    pub(crate) skipped_backlog: u64,
}

pub(crate) enum SlotRead<R> {
    Empty,
    Raced,
    Value { generation: u64, value: R },
}

impl MappedRegion {
    pub(crate) fn create(
        region_file: File,
        region_lock: File,
        layout: LayoutDescriptor,
        policy: MemoryPolicy,
    ) -> Result<Self, ShmError> {
        let map = map_file(&region_file, layout, policy)?;
        let mut region = Self {
            map,
            layout,
            _region_file: region_file,
            _region_lock: region_lock,
        };
        region.initialize();
        region.map.flush().map_err(|source| ShmError::Io {
            operation: "flush initialization",
            source,
        })?;
        region
            .header()
            .identity
            .ready_magic
            .store(layout.snapshot().magic, Ordering::Release);
        region.map.flush().map_err(|source| ShmError::Io {
            operation: "flush ready state",
            source,
        })?;
        Ok(region)
    }

    pub(crate) fn attach(
        region_file: File,
        region_lock: File,
        layout: LayoutDescriptor,
        policy: MemoryPolicy,
    ) -> Result<Self, ShmError> {
        let region = Self {
            map: map_file(&region_file, layout, policy)?,
            layout,
            _region_file: region_file,
            _region_lock: region_lock,
        };
        layout.validate_observed(region.observed_layout())?;
        Ok(region)
    }

    fn initialize(&mut self) {
        let snapshot = self.layout.snapshot();
        // SAFETY: map_file checked an exact length >= Header, mmap bases are
        // page-aligned, and initialization runs under the exclusive region
        // lock before ready is published or an attaching process may proceed.
        unsafe { self.header_mut_ptr().write(Header::new(snapshot)) };

        let slot_bytes = snapshot.versions_offset as usize - snapshot.slots_offset as usize;
        for offset in 0..slot_bytes {
            // SAFETY: offsets were constructed with checked arithmetic and the
            // complete slot area lies inside this exclusive, not-yet-ready map.
            unsafe { self.slot_atomic_ptr(offset).write(AtomicU8::new(0)) };
        }
        for index in 0..snapshot.capacity as usize {
            // SAFETY: the aligned version array length was included in the
            // checked exact mapping length and no other process can observe it.
            unsafe { self.version_ptr(index).write(AtomicU64::new(0)) };
        }
    }

    fn observed_layout(&self) -> LayoutSnapshot {
        let header = self.header();
        let magic = header.identity.ready_magic.load(Ordering::Acquire);
        let mut schema_id = [0; 16];
        schema_id.copy_from_slice(&header.identity.schema_id);
        LayoutSnapshot {
            magic,
            version: header.identity.version,
            endian: header.identity.endian,
            atomic_width: header.identity.atomic_width,
            channel_kind: header.identity.channel_kind,
            capacity: header.identity.capacity,
            schema_id,
            schema_version: header.identity.schema_version,
            payload_size: header.identity.payload_size,
            payload_alignment: header.identity.payload_alignment,
            slot_stride: header.identity.slot_stride,
            slots_offset: header.metadata.slots_offset,
            versions_offset: header.metadata.versions_offset,
            total_length: header.metadata.total_length,
        }
    }

    pub(crate) fn claim_publisher(&self) {
        let line = &self.header().producer;
        if line.active.load(Ordering::Acquire) != 0 {
            increment_shared(&self.header().metadata.reclaimed_endpoints);
        }
        line.epoch.store(
            line.epoch.load(Ordering::Relaxed).saturating_add(1),
            Ordering::Relaxed,
        );
        line.pid.store(std::process::id(), Ordering::Relaxed);
        line.active.store(1, Ordering::Release);
    }

    pub(crate) fn claim_subscriber(&self) {
        let header = self.header();
        let line = &header.consumer;
        if line.active.load(Ordering::Acquire) != 0 {
            increment_shared(&header.metadata.reclaimed_endpoints);
        }
        line.epoch.store(
            line.epoch.load(Ordering::Relaxed).saturating_add(1),
            Ordering::Relaxed,
        );
        line.pid.store(std::process::id(), Ordering::Relaxed);
        let head = header.producer.head.load(Ordering::Acquire);
        let old_tail = line.tail.load(Ordering::Relaxed);
        add_saturating(&line.skipped_backlog, head.saturating_sub(old_tail));
        line.tail.store(head, Ordering::Release);
        line.last_consumed.store(head, Ordering::Release);
        line.active.store(1, Ordering::Release);
    }

    pub(crate) fn release_publisher(&self) {
        let line = &self.header().producer;
        line.pid.store(0, Ordering::Relaxed);
        line.active.store(0, Ordering::Release);
    }

    pub(crate) fn release_subscriber(&self) {
        let line = &self.header().consumer;
        line.pid.store(0, Ordering::Relaxed);
        line.active.store(0, Ordering::Release);
    }

    pub(crate) fn subscriber_active(&self) -> bool {
        self.header().consumer.active.load(Ordering::Acquire) != 0
    }

    pub(crate) fn repair_publisher_inactive(&self) {
        let header = self.header();
        header.producer.pid.store(0, Ordering::Relaxed);
        if header.producer.active.swap(0, Ordering::AcqRel) != 0 {
            increment_shared(&header.metadata.reclaimed_endpoints);
        }
    }

    pub(crate) fn repair_subscriber_inactive(&self) {
        let header = self.header();
        header.consumer.pid.store(0, Ordering::Relaxed);
        if header.consumer.active.swap(0, Ordering::AcqRel) != 0 {
            increment_shared(&header.metadata.reclaimed_endpoints);
        }
    }

    pub(crate) fn record_attach_rejected(&self) {
        increment_shared(&self.header().metadata.attach_rejected);
    }

    pub(crate) fn record_disconnected(&self) {
        increment(&self.header().producer.disconnected);
    }

    pub(crate) fn record_full(&self) {
        increment(&self.header().producer.full);
    }

    pub(crate) fn record_decode_rejected(&self) {
        increment(&self.header().consumer.decode_rejected);
    }

    #[cfg(feature = "fault-injection")]
    pub(crate) fn fault_set_publisher_pid(&self, pid: u32) {
        self.header().producer.pid.store(pid, Ordering::Relaxed);
    }

    #[cfg(feature = "fault-injection")]
    pub(crate) fn fault_set_publisher_epoch(&self, epoch: u64) {
        self.header().producer.epoch.store(epoch, Ordering::Relaxed);
    }

    #[cfg(feature = "fault-injection")]
    pub(crate) fn fault_force_latest_head(&self, generation: u64) {
        self.header()
            .producer
            .head
            .store(generation, Ordering::Release);
    }

    #[cfg(feature = "fault-injection")]
    pub(crate) fn fault_publish_incomplete<R: ShmPayload>(&self, value: &R) -> u64 {
        let header = self.header();
        let generation = header
            .producer
            .head
            .load(Ordering::Relaxed)
            .saturating_add(1);
        let snapshot = self.layout.snapshot();
        let index = ((generation - 1) % u64::from(snapshot.capacity)) as usize;
        self.version(index)
            .store(generation.wrapping_mul(2) | 1, Ordering::Relaxed);
        fence(Ordering::Release);
        for (offset, byte) in bytemuck::bytes_of(value)
            .iter()
            .copied()
            .take(size_of::<R>() / 2)
            .enumerate()
        {
            self.slot_byte(index, offset).store(byte, Ordering::Relaxed);
        }
        generation
    }

    pub(crate) fn counters(&self) -> ChannelCounters {
        let header = self.header();
        ChannelCounters {
            published: header.producer.published.load(Ordering::Relaxed),
            consumed: header.consumer.consumed.load(Ordering::Relaxed),
            overwritten: header.producer.overwritten.load(Ordering::Relaxed),
            full: header.producer.full.load(Ordering::Relaxed),
            disconnected: header.producer.disconnected.load(Ordering::Relaxed),
        }
    }

    pub(crate) fn diagnostics(&self) -> RawDiagnostics {
        let header = self.header();
        RawDiagnostics {
            attach_rejected: header.metadata.attach_rejected.load(Ordering::Relaxed),
            read_races: header.consumer.read_races.load(Ordering::Relaxed),
            decode_rejected: header.consumer.decode_rejected.load(Ordering::Relaxed),
            reclaimed_endpoints: header.metadata.reclaimed_endpoints.load(Ordering::Relaxed),
            skipped_backlog: header.consumer.skipped_backlog.load(Ordering::Relaxed),
        }
    }

    pub(crate) fn publish_latest<R: ShmPayload>(&self, value: &R) -> (u64, bool) {
        let header = self.header();
        let current = header.producer.head.load(Ordering::Relaxed);
        let generation = current.saturating_add(1);
        let overwritten = current > header.consumer.last_consumed.load(Ordering::Acquire);
        self.write_slot(generation, value);
        header.producer.head.store(generation, Ordering::Release);
        increment(&header.producer.published);
        if overwritten {
            increment(&header.producer.overwritten);
        }
        (generation, overwritten)
    }

    pub(crate) fn read_latest<R: ShmPayload>(&self) -> SlotRead<R> {
        let header = self.header();
        let last = header.consumer.last_consumed.load(Ordering::Relaxed);
        for _ in 0..READ_ATTEMPTS {
            let generation = header.producer.head.load(Ordering::Acquire);
            if generation == 0 || generation == last {
                return SlotRead::Empty;
            }
            if let Some(value) = self.read_slot(generation) {
                header
                    .consumer
                    .last_consumed
                    .store(generation, Ordering::Release);
                return SlotRead::Value { generation, value };
            }
        }
        increment(&header.consumer.read_races);
        SlotRead::Raced
    }

    pub(crate) fn mark_latest_consumed(&self) {
        increment(&self.header().consumer.consumed);
    }

    pub(crate) fn publish_ordered<R: ShmPayload>(&self, value: &R) -> Option<u64> {
        let header = self.header();
        let head = header.producer.head.load(Ordering::Relaxed);
        let tail = header.consumer.tail.load(Ordering::Acquire);
        if head.saturating_sub(tail) >= u64::from(self.layout.snapshot().capacity) {
            return None;
        }
        let generation = head.saturating_add(1);
        self.write_slot(generation, value);
        header.producer.head.store(generation, Ordering::Release);
        increment(&header.producer.published);
        Some(generation)
    }

    pub(crate) fn read_ordered<R: ShmPayload>(&self) -> SlotRead<R> {
        let header = self.header();
        let tail = header.consumer.tail.load(Ordering::Relaxed);
        let head = header.producer.head.load(Ordering::Acquire);
        if tail == head {
            return SlotRead::Empty;
        }
        let generation = tail.saturating_add(1);
        let Some(value) = self.read_slot(generation) else {
            increment(&header.consumer.read_races);
            return SlotRead::Raced;
        };
        header.consumer.tail.store(generation, Ordering::Release);
        header
            .consumer
            .last_consumed
            .store(generation, Ordering::Release);
        SlotRead::Value { generation, value }
    }

    pub(crate) fn mark_ordered_consumed(&self) {
        increment(&self.header().consumer.consumed);
    }

    fn write_slot<R: ShmPayload>(&self, generation: u64, value: &R) {
        let snapshot = self.layout.snapshot();
        let index = ((generation - 1) % u64::from(snapshot.capacity)) as usize;
        let version = self.version(index);
        let complete = generation.wrapping_mul(2);
        version.store(complete | 1, Ordering::Relaxed);
        fence(Ordering::Release);
        for (offset, byte) in bytemuck::bytes_of(value).iter().copied().enumerate() {
            self.slot_byte(index, offset).store(byte, Ordering::Relaxed);
        }
        version.store(complete, Ordering::Release);
    }

    fn read_slot<R: ShmPayload>(&self, generation: u64) -> Option<R> {
        let snapshot = self.layout.snapshot();
        let index = ((generation - 1) % u64::from(snapshot.capacity)) as usize;
        let expected = generation.wrapping_mul(2);
        let version = self.version(index);
        if version.load(Ordering::Acquire) != expected {
            return None;
        }
        let mut value = MaybeUninit::<R>::uninit();
        let destination = value.as_mut_ptr().cast::<u8>();
        for offset in 0..size_of::<R>() {
            let byte = self.slot_byte(index, offset).load(Ordering::Relaxed);
            // SAFETY: destination points to size_of::<R>() writable bytes and
            // offset stays in that range. R: Pod permits every byte pattern.
            unsafe { destination.add(offset).write(byte) };
        }
        fence(Ordering::Acquire);
        if version.load(Ordering::Acquire) != expected {
            return None;
        }
        // SAFETY: every byte was initialized and R: Pod accepts all patterns.
        Some(unsafe { value.assume_init() })
    }

    fn header(&self) -> &Header {
        // SAFETY: map_file/initialize guarantee a page-aligned base, exact
        // validated length, and a Header initialized before any attach.
        unsafe { &*self.map.as_ptr().cast::<Header>() }
    }

    fn header_mut_ptr(&mut self) -> *mut Header {
        self.map.as_mut_ptr().cast::<Header>()
    }

    fn slot_atomic_ptr(&self, byte_offset: usize) -> *mut AtomicU8 {
        let base = self.layout.snapshot().slots_offset as usize;
        // SAFETY: callers pass checked slot-area offsets and map lives longer
        // than the returned pointer's immediate initialization/access.
        unsafe { self.map.as_ptr().add(base + byte_offset).cast_mut().cast() }
    }

    fn slot_byte(&self, index: usize, payload_offset: usize) -> &AtomicU8 {
        let stride = self.layout.snapshot().slot_stride as usize;
        // SAFETY: index < capacity and payload_offset < payload size in callers;
        // checked layout arithmetic places this initialized AtomicU8 in-map.
        unsafe { &*self.slot_atomic_ptr(index * stride + payload_offset) }
    }

    fn version_ptr(&self, index: usize) -> *mut AtomicU64 {
        let base = self.layout.snapshot().versions_offset as usize;
        // SAFETY: version offset is 64-byte aligned and index < capacity in all
        // callers; exact layout length includes capacity AtomicU64 entries.
        unsafe {
            self.map
                .as_ptr()
                .add(base)
                .cast::<AtomicU64>()
                .add(index)
                .cast_mut()
        }
    }

    fn version(&self, index: usize) -> &AtomicU64 {
        // SAFETY: version_ptr documents alignment/bounds and map ownership.
        unsafe { &*self.version_ptr(index) }
    }
}

impl ReadOnlyMappedRegion {
    pub(crate) fn attach(
        region_file: File,
        region_lock: File,
        layout: LayoutDescriptor,
        policy: MemoryPolicy,
    ) -> Result<Self, ShmError> {
        let region = Self {
            map: map_file_read_only(&region_file, layout, policy)?,
            layout,
            _region_file: region_file,
            _region_lock: region_lock,
        };
        region.layout.validate_observed(region.observed_layout())?;
        Ok(region)
    }

    /// Reads the current complete latest generation without updating shared
    /// consumer state. The caller owns any local decode error handling.
    pub(crate) fn observe_latest<R: ShmPayload>(&self) -> SlotRead<R> {
        let generation = self.header().producer.head.load(Ordering::Acquire);
        if generation == 0 {
            return SlotRead::Empty;
        }
        for _ in 0..READ_ATTEMPTS {
            if let Some(value) = self.read_slot(generation) {
                return SlotRead::Value { generation, value };
            }
        }
        SlotRead::Raced
    }

    fn observed_layout(&self) -> LayoutSnapshot {
        let header = self.header();
        let magic = header.identity.ready_magic.load(Ordering::Acquire);
        let mut schema_id = [0; 16];
        schema_id.copy_from_slice(&header.identity.schema_id);
        LayoutSnapshot {
            magic,
            version: header.identity.version,
            endian: header.identity.endian,
            atomic_width: header.identity.atomic_width,
            channel_kind: header.identity.channel_kind,
            capacity: header.identity.capacity,
            schema_id,
            schema_version: header.identity.schema_version,
            payload_size: header.identity.payload_size,
            payload_alignment: header.identity.payload_alignment,
            slot_stride: header.identity.slot_stride,
            slots_offset: header.metadata.slots_offset,
            versions_offset: header.metadata.versions_offset,
            total_length: header.metadata.total_length,
        }
    }

    fn read_slot<R: ShmPayload>(&self, generation: u64) -> Option<R> {
        let snapshot = self.layout.snapshot();
        let index = ((generation - 1) % u64::from(snapshot.capacity)) as usize;
        let expected = generation.wrapping_mul(2);
        let version = self.version(index);
        if version.load(Ordering::Acquire) != expected {
            return None;
        }
        let mut value = MaybeUninit::<R>::uninit();
        let destination = value.as_mut_ptr().cast::<u8>();
        for offset in 0..size_of::<R>() {
            let byte = self.slot_byte(index, offset).load(Ordering::Relaxed);
            // SAFETY: destination points to size_of::<R>() writable bytes and
            // offset stays in that range. R: Pod permits every byte pattern.
            unsafe { destination.add(offset).write(byte) };
        }
        fence(Ordering::Acquire);
        if version.load(Ordering::Acquire) != expected {
            return None;
        }
        // SAFETY: every byte was initialized and R: Pod accepts all patterns.
        Some(unsafe { value.assume_init() })
    }

    fn header(&self) -> &Header {
        // SAFETY: map_file_read_only checked the exact length, shared region
        // lock prevents removal, and attach validates the initialized layout
        // before typed reads.
        unsafe { &*self.map.as_ptr().cast::<Header>() }
    }

    fn slot_byte(&self, index: usize, payload_offset: usize) -> &AtomicU8 {
        let stride = self.layout.snapshot().slot_stride as usize;
        let base = self.layout.snapshot().slots_offset as usize;
        // SAFETY: index < capacity and payload_offset < payload size in
        // callers; the checked layout locates initialized atomics in-map.
        unsafe {
            &*self
                .map
                .as_ptr()
                .add(base + index * stride + payload_offset)
                .cast()
        }
    }

    fn version(&self, index: usize) -> &AtomicU64 {
        let base = self.layout.snapshot().versions_offset as usize;
        // SAFETY: layout construction guarantees AtomicU64 alignment and
        // bounds for every capacity-bounded index.
        unsafe { &*self.map.as_ptr().add(base).cast::<AtomicU64>().add(index) }
    }
}

fn map_file(
    file: &File,
    layout: LayoutDescriptor,
    policy: MemoryPolicy,
) -> Result<MmapMut, ShmError> {
    let length = layout.snapshot().total_length as usize;
    let mut options = MmapOptions::new();
    options.len(length).populate();
    // SAFETY: the private namespace, exact fixed file length, held region lock,
    // absence of resize/remap APIs, and stored File constrain cooperating
    // processes for the full mapping lifetime. Header/slots are validated
    // before typed access. Arbitrary same-UID mutation is outside this boundary.
    let map = unsafe { options.map_mut(file) }.map_err(|source| ShmError::Io {
        operation: "map region",
        source,
    })?;
    for offset in (0..length).step_by(PAGE_BYTES) {
        black_box(map[offset]);
    }
    if length != 0 {
        black_box(map[length - 1]);
    }
    if policy == MemoryPolicy::LockRequired {
        map.lock()
            .map_err(|source| ShmError::MemoryLock { source })?;
    }
    Ok(map)
}

fn map_file_read_only(
    file: &File,
    layout: LayoutDescriptor,
    policy: MemoryPolicy,
) -> Result<Mmap, ShmError> {
    let length = layout.snapshot().total_length as usize;
    let mut options = MmapOptions::new();
    options.len(length).populate();
    // SAFETY: this observer owns a shared region-lifetime lock and maps the
    // exact immutable file extent. It performs atomic loads only and exposes
    // no pointer outside this module.
    let map = unsafe { options.map(file) }.map_err(|source| ShmError::Io {
        operation: "map read-only region",
        source,
    })?;
    for offset in (0..length).step_by(PAGE_BYTES) {
        black_box(map[offset]);
    }
    if length != 0 {
        black_box(map[length - 1]);
    }
    if policy == MemoryPolicy::LockRequired {
        map.lock()
            .map_err(|source| ShmError::MemoryLock { source })?;
    }
    Ok(map)
}

fn increment(counter: &AtomicU64) {
    add_saturating(counter, 1);
}

fn add_saturating(counter: &AtomicU64, amount: u64) {
    let current = counter.load(Ordering::Relaxed);
    counter.store(current.saturating_add(amount), Ordering::Relaxed);
}

fn increment_shared(counter: &AtomicU64) {
    let _ = counter.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |current| {
        Some(current.saturating_add(1))
    });
}

// Keep the generic ownership marker available to endpoint modules without raw
// pointers or unsafe auto-trait implementations.
pub(crate) type ValueMarker<T> = PhantomData<fn(T)>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn counter_helpers_saturate() {
        let counter = AtomicU64::new(u64::MAX);
        increment(&counter);
        add_saturating(&counter, u64::MAX);
        assert_eq!(counter.load(Ordering::Relaxed), u64::MAX);
    }
}
