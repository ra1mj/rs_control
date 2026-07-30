//! Safe in-process implementations of the backend-neutral middleware ports.

use std::sync::{
    Arc,
    atomic::{AtomicBool, AtomicU64, Ordering},
};

use middleware_api::{
    ChannelCounters, EventPublish, EventPublishError, EventPublisher, EventSubscriber,
    LatestPublish, LatestPublisher, LatestSample, LatestSubscriber, OrderedSample,
};
use rtrb::{Consumer, Producer, PushError, RingBuffer};
use thiserror::Error;
use waitfree_sync::triple_buffer;

#[derive(Debug)]
struct SharedState {
    reader_alive: AtomicBool,
    writer_alive: AtomicBool,
    published: AtomicU64,
    consumed: AtomicU64,
    overwritten: AtomicU64,
    full: AtomicU64,
    disconnected: AtomicU64,
    last_consumed_generation: AtomicU64,
}

impl SharedState {
    fn new() -> Self {
        Self {
            reader_alive: AtomicBool::new(true),
            writer_alive: AtomicBool::new(true),
            published: AtomicU64::new(0),
            consumed: AtomicU64::new(0),
            overwritten: AtomicU64::new(0),
            full: AtomicU64::new(0),
            disconnected: AtomicU64::new(0),
            last_consumed_generation: AtomicU64::new(0),
        }
    }

    fn snapshot(&self) -> ChannelCounters {
        ChannelCounters {
            published: self.published.load(Ordering::Relaxed),
            consumed: self.consumed.load(Ordering::Relaxed),
            overwritten: self.overwritten.load(Ordering::Relaxed),
            full: self.full.load(Ordering::Relaxed),
            disconnected: self.disconnected.load(Ordering::Relaxed),
        }
    }
}

fn increment_saturating(counter: &AtomicU64) {
    let mut current = counter.load(Ordering::Relaxed);
    while current != u64::MAX {
        match counter.compare_exchange_weak(
            current,
            current + 1,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => return,
            Err(observed) => current = observed,
        }
    }
}

/// Wait-free latest-value writer for fixed-layout `Copy` values.
#[derive(Debug)]
pub struct LocalLatestPublisher<T> {
    inner: triple_buffer::Writer<(u64, T)>,
    shared: Arc<SharedState>,
    generation: u64,
    last_published_generation: u64,
}

/// Wait-free latest-value reader for fixed-layout `Copy` values.
#[derive(Debug)]
pub struct LocalLatestSubscriber<T> {
    inner: triple_buffer::Reader<(u64, T)>,
    shared: Arc<SharedState>,
    last_read_generation: u64,
}

/// Creates a preallocated single-writer/single-reader latest-value channel.
#[must_use]
pub fn latest_channel<T: Copy + Send + 'static>()
-> (LocalLatestPublisher<T>, LocalLatestSubscriber<T>) {
    let (writer, reader) = triple_buffer::triple_buffer();
    let shared = Arc::new(SharedState::new());
    (
        LocalLatestPublisher {
            inner: writer,
            shared: Arc::clone(&shared),
            generation: 0,
            last_published_generation: 0,
        },
        LocalLatestSubscriber {
            inner: reader,
            shared,
            last_read_generation: 0,
        },
    )
}

impl<T: Copy + Send + 'static> LatestPublisher<T> for LocalLatestPublisher<T> {
    fn try_publish(&mut self, value: T) -> LatestPublish {
        if !self.shared.reader_alive.load(Ordering::Acquire) {
            increment_saturating(&self.shared.disconnected);
            return LatestPublish::Disconnected;
        }

        let overwritten = self.last_published_generation != 0
            && self.shared.last_consumed_generation.load(Ordering::Acquire)
                < self.last_published_generation;
        self.generation = self.generation.saturating_add(1);
        self.inner.write((self.generation, value));
        self.last_published_generation = self.generation;
        increment_saturating(&self.shared.published);
        if overwritten {
            increment_saturating(&self.shared.overwritten);
        }
        LatestPublish::Published {
            generation: self.generation,
            overwritten,
        }
    }

    fn counters(&self) -> ChannelCounters {
        self.shared.snapshot()
    }
}

impl<T> Drop for LocalLatestPublisher<T> {
    fn drop(&mut self) {
        self.shared.writer_alive.store(false, Ordering::Release);
    }
}

impl<T: Copy + Send + 'static> LatestSubscriber<T> for LocalLatestSubscriber<T> {
    fn try_read_latest(&mut self) -> Option<LatestSample<T>> {
        let (generation, value) = self.inner.try_read()?;
        if generation == self.last_read_generation {
            return None;
        }
        self.last_read_generation = generation;
        self.shared
            .last_consumed_generation
            .store(generation, Ordering::Release);
        increment_saturating(&self.shared.consumed);
        Some(LatestSample { generation, value })
    }

    fn counters(&self) -> ChannelCounters {
        self.shared.snapshot()
    }
}

impl<T> Drop for LocalLatestSubscriber<T> {
    fn drop(&mut self) {
        self.shared.reader_alive.store(false, Ordering::Release);
    }
}

/// Ordered-channel construction failure.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum ChannelError {
    /// A bounded channel must have at least one slot.
    #[error("ordered channel capacity must be nonzero")]
    ZeroCapacity,
}

/// Wait-free bounded ordered-event writer.
#[derive(Debug)]
pub struct LocalEventPublisher<T> {
    inner: Producer<(u64, T)>,
    shared: Arc<SharedState>,
    generation: u64,
}

/// Wait-free bounded ordered-event reader.
#[derive(Debug)]
pub struct LocalEventSubscriber<T> {
    inner: Consumer<(u64, T)>,
    shared: Arc<SharedState>,
}

/// Creates a preallocated single-writer/single-reader ordered channel.
pub fn ordered_channel<T: Send + 'static>(
    capacity: usize,
) -> Result<(LocalEventPublisher<T>, LocalEventSubscriber<T>), ChannelError> {
    if capacity == 0 {
        return Err(ChannelError::ZeroCapacity);
    }
    let (producer, consumer) = RingBuffer::new(capacity);
    let shared = Arc::new(SharedState::new());
    Ok((
        LocalEventPublisher {
            inner: producer,
            shared: Arc::clone(&shared),
            generation: 0,
        },
        LocalEventSubscriber {
            inner: consumer,
            shared,
        },
    ))
}

impl<T: Send + 'static> EventPublisher<T> for LocalEventPublisher<T> {
    fn try_publish(&mut self, value: T) -> Result<EventPublish, EventPublishError<T>> {
        if !self.shared.reader_alive.load(Ordering::Acquire) || self.inner.is_abandoned() {
            increment_saturating(&self.shared.disconnected);
            return Err(EventPublishError::Disconnected(value));
        }

        let generation = self.generation.saturating_add(1);
        match self.inner.push((generation, value)) {
            Ok(()) => {
                self.generation = generation;
                increment_saturating(&self.shared.published);
                Ok(EventPublish { generation })
            }
            Err(PushError::Full((_, value))) => {
                increment_saturating(&self.shared.full);
                Err(EventPublishError::Full(value))
            }
        }
    }

    fn counters(&self) -> ChannelCounters {
        self.shared.snapshot()
    }
}

impl<T> Drop for LocalEventPublisher<T> {
    fn drop(&mut self) {
        self.shared.writer_alive.store(false, Ordering::Release);
    }
}

impl<T: Send + 'static> EventSubscriber<T> for LocalEventSubscriber<T> {
    fn try_receive(&mut self) -> Option<OrderedSample<T>> {
        let (generation, value) = self.inner.pop().ok()?;
        self.shared
            .last_consumed_generation
            .store(generation, Ordering::Release);
        increment_saturating(&self.shared.consumed);
        Some(OrderedSample { generation, value })
    }

    fn counters(&self) -> ChannelCounters {
        self.shared.snapshot()
    }
}

impl<T> Drop for LocalEventSubscriber<T> {
    fn drop(&mut self) {
        self.shared.reader_alive.store(false, Ordering::Release);
    }
}

#[cfg(test)]
mod tests {
    use allocation_counter::measure;

    use super::*;

    #[test]
    fn latest_reports_overwrite_and_distinct_reads() {
        let (mut writer, mut reader) = latest_channel();
        assert_eq!(
            writer.try_publish(10_u64),
            LatestPublish::Published {
                generation: 1,
                overwritten: false
            }
        );
        assert_eq!(
            writer.try_publish(20),
            LatestPublish::Published {
                generation: 2,
                overwritten: true
            }
        );
        assert_eq!(
            reader.try_read_latest(),
            Some(LatestSample {
                generation: 2,
                value: 20
            })
        );
        assert_eq!(reader.try_read_latest(), None);
        assert_eq!(
            writer.counters(),
            ChannelCounters {
                published: 2,
                consumed: 1,
                overwritten: 1,
                full: 0,
                disconnected: 0,
            }
        );
    }

    #[test]
    fn ordered_returns_full_value_and_disconnects() {
        let (mut writer, mut reader) = ordered_channel(1).expect("capacity is valid");
        assert_eq!(writer.try_publish(10), Ok(EventPublish { generation: 1 }));
        assert_eq!(writer.try_publish(20), Err(EventPublishError::Full(20)));
        assert_eq!(
            reader.try_receive(),
            Some(OrderedSample {
                generation: 1,
                value: 10
            })
        );
        drop(reader);
        assert_eq!(
            writer.try_publish(30),
            Err(EventPublishError::Disconnected(30))
        );
        assert_eq!(writer.counters().full, 1);
        assert_eq!(writer.counters().disconnected, 1);
    }

    #[test]
    fn zero_capacity_is_rejected_without_panic() {
        assert!(matches!(
            ordered_channel::<u64>(0),
            Err(ChannelError::ZeroCapacity)
        ));
    }

    #[test]
    fn steady_state_fixed_layout_operations_do_not_allocate() {
        let (mut latest_writer, mut latest_reader) = latest_channel();
        let (mut event_writer, mut event_reader) = ordered_channel(4).expect("capacity is valid");
        latest_writer.try_publish(1_u64);
        latest_reader.try_read_latest();
        event_writer.try_publish(1_u64).expect("queue has space");
        event_reader.try_receive();

        let allocations = measure(|| {
            latest_writer.try_publish(2);
            assert!(latest_reader.try_read_latest().is_some());
            event_writer.try_publish(2).expect("queue has space");
            assert!(event_reader.try_receive().is_some());
        });
        assert_eq!(allocations.count_total, 0);
    }
}
