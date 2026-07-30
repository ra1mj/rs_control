/// Saturating channel observations.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ChannelCounters {
    /// Values/events accepted by the backend.
    pub published: u64,
    /// Distinct generations consumed.
    pub consumed: u64,
    /// Latest generations replaced before consumption.
    pub overwritten: u64,
    /// Ordered events rejected because capacity was exhausted.
    pub full: u64,
    /// Producer calls rejected because the consumer was gone.
    pub disconnected: u64,
}

/// Result of a latest-value publish attempt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LatestPublish {
    /// The value became the newest generation.
    Published {
        /// Channel-local generation.
        generation: u64,
        /// Whether an unread generation was replaced.
        overwritten: bool,
    },
    /// No consumer remains.
    Disconnected,
}

/// One distinct latest-value generation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct LatestSample<T> {
    /// Channel-local generation.
    pub generation: u64,
    /// Typed semantic value.
    pub value: T,
}

/// Successful ordered-event publish.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EventPublish {
    /// Channel-local generation.
    pub generation: u64,
}

/// One ordered event and its channel generation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct OrderedSample<T> {
    /// Channel-local generation.
    pub generation: u64,
    /// Typed event value.
    pub value: T,
}

/// Failed ordered publish that returns ownership to the caller.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum EventPublishError<T> {
    /// The bounded queue was full.
    Full(T),
    /// The consumer endpoint was destroyed.
    Disconnected(T),
}

/// Producer capability for latest-value delivery.
pub trait LatestPublisher<T> {
    /// Publishes without waiting for the consumer.
    fn try_publish(&mut self, value: T) -> LatestPublish;
    /// Returns a nonblocking counter snapshot.
    fn counters(&self) -> ChannelCounters;
}

/// Consumer capability for latest-value delivery.
pub trait LatestSubscriber<T> {
    /// Returns the newest generation once, or `None` when unchanged/empty.
    fn try_read_latest(&mut self) -> Option<LatestSample<T>>;
    /// Returns a nonblocking counter snapshot.
    fn counters(&self) -> ChannelCounters;
}

/// Producer capability for ordered bounded delivery.
pub trait EventPublisher<T> {
    /// Publishes without waiting and returns ownership on rejection.
    fn try_publish(&mut self, value: T) -> Result<EventPublish, EventPublishError<T>>;
    /// Returns a nonblocking counter snapshot.
    fn counters(&self) -> ChannelCounters;
}

/// Consumer capability for ordered bounded delivery.
pub trait EventSubscriber<T> {
    /// Returns the oldest queued event, or `None` when empty.
    fn try_receive(&mut self) -> Option<OrderedSample<T>>;
    /// Returns a nonblocking counter snapshot.
    fn counters(&self) -> ChannelCounters;
}
