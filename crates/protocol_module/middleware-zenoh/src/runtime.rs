use std::{
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicU64, Ordering},
        mpsc::{self, Receiver, SyncSender, TryRecvError, TrySendError},
    },
    thread::{self, JoinHandle},
    time::{Duration, Instant},
};

use middleware_api::{
    EndpointDescriptor, EndpointDirection, EndpointError, EndpointId, InteractionClass,
    validate_route_set,
};
use thiserror::Error;
use zenoh::{Wait, handlers::RingChannel, query::Query, sample::SampleKind};

use crate::{QueryError, QueryReply, resolve_single_reply, zenoh_qos};

const DEFAULT_COMMAND_CAPACITY: usize = 8;
const WORKER_POLL_INTERVAL: Duration = Duration::from_millis(1);
const QUERY_RESPONSE_GRACE: Duration = Duration::from_millis(250);

type OutboundPoll =
    Box<dyn FnMut() -> Result<Option<Vec<u8>>, RouteHandlingError> + Send + 'static>;
type InboundHandler = Box<dyn FnMut(Vec<u8>) -> Result<(), RouteHandlingError> + Send + 'static>;
type QueryHandler = Box<dyn FnMut(Vec<u8>) -> Result<Vec<u8>, Vec<u8>> + Send + 'static>;
type LivelinessHandler = Box<dyn FnMut(bool) + Send + 'static>;

/// Zenoh configuration hidden behind the adapter boundary.
#[derive(Clone, Debug)]
pub struct ZenohBridgeConfig {
    inner: zenoh::Config,
    command_capacity: usize,
}

impl ZenohBridgeConfig {
    /// Parses a Zenoh JSON5 configuration without exposing a session to callers.
    pub fn from_json5(value: &str) -> Result<Self, BridgeError> {
        let inner = zenoh::Config::from_json5(value)
            .map_err(|error| BridgeError::Configuration(error.to_string()))?;
        Ok(Self {
            inner,
            command_capacity: DEFAULT_COMMAND_CAPACITY,
        })
    }

    /// Uses Zenoh defaults. Intended for non-realtime composition roots only.
    #[must_use]
    pub fn defaults() -> Self {
        Self {
            inner: zenoh::Config::default(),
            command_capacity: DEFAULT_COMMAND_CAPACITY,
        }
    }

    /// Sets the bounded non-realtime bridge command capacity.
    pub fn with_command_capacity(mut self, capacity: usize) -> Result<Self, BridgeError> {
        if capacity == 0 {
            return Err(BridgeError::ZeroCommandCapacity);
        }
        self.command_capacity = capacity;
        Ok(self)
    }
}

/// Route callback failure classified without leaking codec or Zenoh types.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum RouteHandlingError {
    /// Bytes could not be decoded into the declared semantic type.
    #[error("route payload is malformed")]
    Malformed,
    /// A decoded value failed source, freshness, authority or policy checks.
    #[error("route payload was rejected by semantic policy")]
    Rejected,
}

/// Non-realtime publisher route polled by the bridge worker.
pub struct OutboundRoute {
    descriptor: EndpointDescriptor,
    poll: OutboundPoll,
}

impl OutboundRoute {
    /// Creates a publisher route. Encoding happens inside `poll` on the worker.
    pub fn new<F>(descriptor: EndpointDescriptor, poll: F) -> Result<Self, BridgeError>
    where
        F: FnMut() -> Result<Option<Vec<u8>>, RouteHandlingError> + Send + 'static,
    {
        require_direction(descriptor, EndpointDirection::Publish)?;
        Ok(Self {
            descriptor,
            poll: Box::new(poll),
        })
    }
}

/// Non-realtime subscriber route drained by the bridge worker.
pub struct InboundRoute {
    descriptor: EndpointDescriptor,
    handler: InboundHandler,
}

impl InboundRoute {
    /// Creates a subscriber route. Decode and semantic validation happen in `handler`.
    pub fn new<F>(descriptor: EndpointDescriptor, handler: F) -> Result<Self, BridgeError>
    where
        F: FnMut(Vec<u8>) -> Result<(), RouteHandlingError> + Send + 'static,
    {
        require_direction(descriptor, EndpointDirection::Subscribe)?;
        Ok(Self {
            descriptor,
            handler: Box::new(handler),
        })
    }
}

/// Bounded non-realtime queryable route.
pub struct QueryableRoute {
    descriptor: EndpointDescriptor,
    handler: QueryHandler,
}

impl QueryableRoute {
    /// Creates a queryable route whose error bytes become a Zenoh error reply.
    pub fn new<F>(descriptor: EndpointDescriptor, handler: F) -> Result<Self, BridgeError>
    where
        F: FnMut(Vec<u8>) -> Result<Vec<u8>, Vec<u8>> + Send + 'static,
    {
        require_direction(descriptor, EndpointDirection::Queryable)?;
        Ok(Self {
            descriptor,
            handler: Box::new(handler),
        })
    }
}

/// Bounded liveliness observer route.
pub struct LivelinessObserverRoute {
    descriptor: EndpointDescriptor,
    handler: LivelinessHandler,
}

impl LivelinessObserverRoute {
    /// Creates an observer receiving `true` for put and `false` for delete.
    pub fn new<F>(descriptor: EndpointDescriptor, handler: F) -> Result<Self, BridgeError>
    where
        F: FnMut(bool) + Send + 'static,
    {
        require_direction(descriptor, EndpointDirection::ObserveLiveliness)?;
        Ok(Self {
            descriptor,
            handler: Box::new(handler),
        })
    }
}

/// Complete declaration set owned by one Zenoh worker.
#[derive(Default)]
pub struct BridgeRoutes {
    outbound: Vec<OutboundRoute>,
    inbound: Vec<InboundRoute>,
    queries: Vec<EndpointDescriptor>,
    queryables: Vec<QueryableRoute>,
    announcements: Vec<EndpointDescriptor>,
    observers: Vec<LivelinessObserverRoute>,
}

impl BridgeRoutes {
    /// Creates an empty route set.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            outbound: Vec::new(),
            inbound: Vec::new(),
            queries: Vec::new(),
            queryables: Vec::new(),
            announcements: Vec::new(),
            observers: Vec::new(),
        }
    }

    /// Adds an egress publication route.
    pub fn add_outbound(&mut self, route: OutboundRoute) {
        self.outbound.push(route);
    }

    /// Adds an ingress subscription route.
    pub fn add_inbound(&mut self, route: InboundRoute) {
        self.inbound.push(route);
    }

    /// Registers a bounded query endpoint callable through [`ZenohBridge::query`].
    pub fn add_query(&mut self, descriptor: EndpointDescriptor) -> Result<(), BridgeError> {
        require_direction(descriptor, EndpointDirection::Query)?;
        self.queries.push(descriptor);
        Ok(())
    }

    /// Adds a bounded queryable declaration.
    pub fn add_queryable(&mut self, route: QueryableRoute) {
        self.queryables.push(route);
    }

    /// Adds a liveliness token declaration.
    pub fn add_liveliness_announcement(
        &mut self,
        descriptor: EndpointDescriptor,
    ) -> Result<(), BridgeError> {
        require_direction(descriptor, EndpointDirection::AnnounceLiveliness)?;
        self.announcements.push(descriptor);
        Ok(())
    }

    /// Adds a liveliness observer declaration.
    pub fn add_liveliness_observer(&mut self, route: LivelinessObserverRoute) {
        self.observers.push(route);
    }

    fn descriptors(&self) -> Vec<EndpointDescriptor> {
        self.outbound
            .iter()
            .map(|route| route.descriptor)
            .chain(self.inbound.iter().map(|route| route.descriptor))
            .chain(self.queries.iter().copied())
            .chain(self.queryables.iter().map(|route| route.descriptor))
            .chain(self.announcements.iter().copied())
            .chain(self.observers.iter().map(|route| route.descriptor))
            .collect()
    }
}

/// Aggregate bridge observations. Every field saturates.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct BridgeCounters {
    /// Successful Zenoh publications.
    pub published: u64,
    /// Samples/events delivered to route handlers.
    pub received: u64,
    /// Callback inputs discarded because a bounded inbox was full.
    pub callback_dropped: u64,
    /// Payloads rejected before decode because they exceeded route bounds.
    pub oversized: u64,
    /// Route codec failures.
    pub malformed: u64,
    /// Route semantic-policy rejections.
    pub rejected: u64,
    /// Zenoh open/declaration/publish/reply/close failures after startup.
    pub transport_errors: u64,
    /// Query calls whose bounded completion window elapsed.
    pub query_timeouts: u64,
}

#[derive(Debug, Default)]
struct AtomicBridgeCounters {
    published: AtomicU64,
    received: AtomicU64,
    callback_dropped: AtomicU64,
    oversized: AtomicU64,
    malformed: AtomicU64,
    rejected: AtomicU64,
    transport_errors: AtomicU64,
    query_timeouts: AtomicU64,
}

impl AtomicBridgeCounters {
    fn snapshot(&self) -> BridgeCounters {
        BridgeCounters {
            published: self.published.load(Ordering::Relaxed),
            received: self.received.load(Ordering::Relaxed),
            callback_dropped: self.callback_dropped.load(Ordering::Relaxed),
            oversized: self.oversized.load(Ordering::Relaxed),
            malformed: self.malformed.load(Ordering::Relaxed),
            rejected: self.rejected.load(Ordering::Relaxed),
            transport_errors: self.transport_errors.load(Ordering::Relaxed),
            query_timeouts: self.query_timeouts.load(Ordering::Relaxed),
        }
    }
}

fn increment(counter: &AtomicU64) {
    let _ = counter.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |value| {
        value.checked_add(1)
    });
}

/// Lifecycle and bounded query handle for a dedicated non-realtime Zenoh worker.
pub struct ZenohBridge {
    alive: Arc<AtomicBool>,
    shutdown: Arc<AtomicBool>,
    counters: Arc<AtomicBridgeCounters>,
    commands: SyncSender<QueryCommand>,
    query_routes: Vec<EndpointDescriptor>,
    worker: Option<JoinHandle<()>>,
}

impl ZenohBridge {
    /// Opens declarations on a named worker thread and returns only after readiness.
    pub fn spawn(config: ZenohBridgeConfig, routes: BridgeRoutes) -> Result<Self, BridgeError> {
        validate_route_set(&routes.descriptors())?;
        let query_routes = routes.queries.clone();
        let alive = Arc::new(AtomicBool::new(false));
        let shutdown = Arc::new(AtomicBool::new(false));
        let counters = Arc::new(AtomicBridgeCounters::default());
        let (commands, command_rx) = mpsc::sync_channel(config.command_capacity);
        let (ready_tx, ready_rx) = mpsc::sync_channel(1);

        let thread_alive = Arc::clone(&alive);
        let thread_shutdown = Arc::clone(&shutdown);
        let thread_counters = Arc::clone(&counters);
        let worker = thread::Builder::new()
            .name("zenoh-bridge".to_owned())
            .spawn(move || {
                run_worker(
                    config.inner,
                    routes,
                    command_rx,
                    ready_tx,
                    thread_alive,
                    thread_shutdown,
                    thread_counters,
                );
            })
            .map_err(BridgeError::ThreadSpawn)?;

        match ready_rx.recv() {
            Ok(Ok(())) => Ok(Self {
                alive,
                shutdown,
                counters,
                commands,
                query_routes,
                worker: Some(worker),
            }),
            Ok(Err(error)) => {
                let _ = worker.join();
                Err(BridgeError::Startup(error))
            }
            Err(_) => {
                let _ = worker.join();
                Err(BridgeError::WorkerPanicked)
            }
        }
    }

    /// Returns whether the non-realtime worker is still available.
    #[must_use]
    pub fn is_running(&self) -> bool {
        self.alive.load(Ordering::Acquire)
    }

    /// Returns a nonblocking aggregate counter snapshot.
    #[must_use]
    pub fn counters(&self) -> BridgeCounters {
        self.counters.snapshot()
    }

    /// Performs one bounded non-realtime query through a predeclared route.
    pub fn query(&self, endpoint: EndpointId, payload: Vec<u8>) -> Result<Vec<u8>, BridgeError> {
        if !self.is_running() {
            return Err(BridgeError::WorkerUnavailable);
        }
        let descriptor = self
            .query_routes
            .iter()
            .copied()
            .find(|route| route.fields().id == endpoint)
            .ok_or(BridgeError::UnknownQueryEndpoint)?;
        let fields = descriptor.fields();
        if payload.len() > fields.bounds.max_payload_bytes {
            return Err(BridgeError::OversizedPayload {
                actual: payload.len(),
                maximum: fields.bounds.max_payload_bytes,
            });
        }
        let timeout = Duration::from_nanos(
            fields
                .bounds
                .timeout_ns
                .ok_or(BridgeError::MissingQueryTimeout)?,
        );
        let (response_tx, response_rx) = mpsc::sync_channel(1);
        let command = QueryCommand {
            descriptor,
            payload,
            response: response_tx,
        };
        self.commands
            .try_send(command)
            .map_err(|error| match error {
                TrySendError::Full(_) => BridgeError::CommandQueueFull,
                TrySendError::Disconnected(_) => BridgeError::WorkerUnavailable,
            })?;
        if let Some(worker) = &self.worker {
            worker.thread().unpark();
        }
        response_rx
            .recv_timeout(timeout.saturating_add(QUERY_RESPONSE_GRACE))
            .map_err(|_| {
                increment(&self.counters.query_timeouts);
                BridgeError::Query(QueryError::Timeout)
            })?
    }

    /// Cooperatively closes declarations/session and joins the worker.
    pub fn stop(mut self) -> Result<BridgeCounters, BridgeError> {
        self.stop_inner()?;
        Ok(self.counters.snapshot())
    }

    fn stop_inner(&mut self) -> Result<(), BridgeError> {
        self.shutdown.store(true, Ordering::Release);
        if let Some(worker) = self.worker.take() {
            worker.thread().unpark();
            worker.join().map_err(|_| BridgeError::WorkerPanicked)?;
        }
        Ok(())
    }
}

impl Drop for ZenohBridge {
    fn drop(&mut self) {
        let _ = self.stop_inner();
    }
}

struct QueryCommand {
    descriptor: EndpointDescriptor,
    payload: Vec<u8>,
    response: SyncSender<Result<Vec<u8>, BridgeError>>,
}

struct InboundPump {
    handler: InboundHandler,
    receiver: Receiver<Vec<u8>>,
}

struct QueryablePump {
    descriptor: EndpointDescriptor,
    handler: QueryHandler,
    receiver: Receiver<Query>,
}

struct LivelinessPump {
    handler: LivelinessHandler,
    receiver: Receiver<bool>,
}

fn run_worker(
    config: zenoh::Config,
    routes: BridgeRoutes,
    commands: Receiver<QueryCommand>,
    ready: SyncSender<Result<(), String>>,
    alive: Arc<AtomicBool>,
    shutdown: Arc<AtomicBool>,
    counters: Arc<AtomicBridgeCounters>,
) {
    let mut state = WorkerState {
        ready: &ready,
        ready_sent: false,
        alive: &alive,
        shutdown: &shutdown,
        counters: &counters,
    };
    let result = run_worker_inner(config, routes, commands, &mut state);
    alive.store(false, Ordering::Release);
    if let Err(error) = result {
        if !state.ready_sent {
            let _ = ready.send(Err(error));
        } else {
            increment(&counters.transport_errors);
        }
    }
}

struct WorkerState<'a> {
    ready: &'a SyncSender<Result<(), String>>,
    ready_sent: bool,
    alive: &'a AtomicBool,
    shutdown: &'a AtomicBool,
    counters: &'a Arc<AtomicBridgeCounters>,
}

fn run_worker_inner(
    config: zenoh::Config,
    mut routes: BridgeRoutes,
    commands: Receiver<QueryCommand>,
    state: &mut WorkerState<'_>,
) -> Result<(), String> {
    let session = zenoh::open(config)
        .wait()
        .map_err(|error| error.to_string())?;
    let mut inbound_pumps = Vec::with_capacity(routes.inbound.len());
    for route in routes.inbound.drain(..) {
        let fields = route.descriptor.fields();
        let (sender, receiver) = mpsc::sync_channel(fields.bounds.queue_capacity);
        let callback_counters = Arc::clone(state.counters);
        let maximum = fields.bounds.max_payload_bytes;
        session
            .declare_subscriber(fields.key.to_string())
            .callback(move |sample| {
                if sample.payload().len() > maximum {
                    increment(&callback_counters.oversized);
                    return;
                }
                if sender
                    .try_send(sample.payload().to_bytes().into_owned())
                    .is_err()
                {
                    increment(&callback_counters.callback_dropped);
                }
            })
            .background()
            .wait()
            .map_err(|error| error.to_string())?;
        inbound_pumps.push(InboundPump {
            handler: route.handler,
            receiver,
        });
    }

    let mut queryable_pumps = Vec::with_capacity(routes.queryables.len());
    for route in routes.queryables.drain(..) {
        let fields = route.descriptor.fields();
        let (sender, receiver) = mpsc::sync_channel(fields.bounds.queue_capacity);
        let callback_counters = Arc::clone(state.counters);
        session
            .declare_queryable(fields.key.to_string())
            .callback(move |query| {
                if sender.try_send(query).is_err() {
                    increment(&callback_counters.callback_dropped);
                }
            })
            .background()
            .wait()
            .map_err(|error| error.to_string())?;
        queryable_pumps.push(QueryablePump {
            descriptor: route.descriptor,
            handler: route.handler,
            receiver,
        });
    }

    let mut liveliness_pumps = Vec::with_capacity(routes.observers.len());
    for route in routes.observers.drain(..) {
        let fields = route.descriptor.fields();
        let (sender, receiver) = mpsc::sync_channel(fields.bounds.queue_capacity);
        let callback_counters = Arc::clone(state.counters);
        session
            .liveliness()
            .declare_subscriber(fields.key.to_string())
            .history(true)
            .callback(move |sample| {
                let available = sample.kind() == SampleKind::Put;
                if sender.try_send(available).is_err() {
                    increment(&callback_counters.callback_dropped);
                }
            })
            .background()
            .wait()
            .map_err(|error| error.to_string())?;
        liveliness_pumps.push(LivelinessPump {
            handler: route.handler,
            receiver,
        });
    }

    let mut liveliness_tokens = Vec::with_capacity(routes.announcements.len());
    for descriptor in routes.announcements.drain(..) {
        let token = session
            .liveliness()
            .declare_token(descriptor.fields().key.to_string())
            .wait()
            .map_err(|error| error.to_string())?;
        liveliness_tokens.push(token);
    }

    state.alive.store(true, Ordering::Release);
    state
        .ready
        .send(Ok(()))
        .map_err(|error| error.to_string())?;
    state.ready_sent = true;

    while !state.shutdown.load(Ordering::Acquire) {
        pump_outbound(&session, &mut routes.outbound, state.counters);
        pump_inbound(&mut inbound_pumps, state.counters);
        pump_queryables(&mut queryable_pumps, state.counters);
        pump_liveliness(&mut liveliness_pumps, state.counters);
        match commands.try_recv() {
            Ok(command) => handle_query(&session, command, state.counters),
            Err(TryRecvError::Empty) => {}
            Err(TryRecvError::Disconnected) => break,
        }
        thread::park_timeout(WORKER_POLL_INTERVAL);
    }

    drop(liveliness_tokens);
    session.close().wait().map_err(|error| error.to_string())
}

fn pump_outbound(
    session: &zenoh::Session,
    routes: &mut [OutboundRoute],
    counters: &AtomicBridgeCounters,
) {
    for route in routes {
        let payload = match (route.poll)() {
            Ok(Some(payload)) => payload,
            Ok(None) => continue,
            Err(RouteHandlingError::Malformed) => {
                increment(&counters.malformed);
                continue;
            }
            Err(RouteHandlingError::Rejected) => {
                increment(&counters.rejected);
                continue;
            }
        };
        let fields = route.descriptor.fields();
        if payload.len() > fields.bounds.max_payload_bytes {
            increment(&counters.oversized);
            continue;
        }
        let qos = zenoh_qos(fields.qos);
        if session
            .put(fields.key.to_string(), payload)
            .priority(qos.priority)
            .congestion_control(qos.congestion)
            .wait()
            .is_ok()
        {
            increment(&counters.published);
        } else {
            increment(&counters.transport_errors);
        }
    }
}

fn pump_inbound(pumps: &mut [InboundPump], counters: &AtomicBridgeCounters) {
    for pump in pumps {
        while let Ok(payload) = pump.receiver.try_recv() {
            increment(&counters.received);
            match (pump.handler)(payload) {
                Ok(()) => {}
                Err(RouteHandlingError::Malformed) => increment(&counters.malformed),
                Err(RouteHandlingError::Rejected) => increment(&counters.rejected),
            }
        }
    }
}

fn pump_queryables(pumps: &mut [QueryablePump], counters: &AtomicBridgeCounters) {
    for pump in pumps {
        while let Ok(query) = pump.receiver.try_recv() {
            increment(&counters.received);
            let fields = pump.descriptor.fields();
            let payload = query
                .payload()
                .map_or_else(Vec::new, |payload| payload.to_bytes().into_owned());
            let reply = if payload.len() > fields.bounds.max_payload_bytes {
                increment(&counters.oversized);
                query
                    .reply_err(b"request exceeds route bound".to_vec())
                    .wait()
            } else {
                match (pump.handler)(payload) {
                    Ok(reply) if reply.len() <= fields.bounds.max_payload_bytes => {
                        query.reply(fields.key.to_string(), reply).wait()
                    }
                    Ok(_) => {
                        increment(&counters.oversized);
                        query
                            .reply_err(b"reply exceeds route bound".to_vec())
                            .wait()
                    }
                    Err(error) => query.reply_err(error).wait(),
                }
            };
            if reply.is_err() {
                increment(&counters.transport_errors);
            }
        }
    }
}

fn pump_liveliness(pumps: &mut [LivelinessPump], counters: &AtomicBridgeCounters) {
    for pump in pumps {
        while let Ok(available) = pump.receiver.try_recv() {
            increment(&counters.received);
            (pump.handler)(available);
        }
    }
}

fn handle_query(session: &zenoh::Session, command: QueryCommand, counters: &AtomicBridgeCounters) {
    let fields = command.descriptor.fields();
    let timeout = Duration::from_nanos(fields.bounds.timeout_ns.unwrap_or(1));
    let qos = zenoh_qos(fields.qos);
    let receiver = match session
        .get(fields.key.to_string())
        .payload(command.payload)
        .priority(qos.priority)
        .congestion_control(qos.congestion)
        .timeout(timeout)
        .with(RingChannel::new(fields.bounds.queue_capacity.max(2)))
        .wait()
    {
        Ok(receiver) => receiver,
        Err(error) => {
            increment(&counters.transport_errors);
            let _ = command
                .response
                .send(Err(BridgeError::Transport(error.to_string())));
            return;
        }
    };
    let deadline = Instant::now() + timeout;
    let mut replies = Vec::with_capacity(2);
    loop {
        match receiver.recv_deadline(deadline) {
            Ok(Some(reply)) => match reply.into_result() {
                Ok(sample) => {
                    replies.push(QueryReply::Data(sample.payload().to_bytes().into_owned()))
                }
                Err(error) => replies.push(QueryReply::RemoteError(
                    error.payload().to_bytes().into_owned(),
                )),
            },
            Ok(None) => {
                increment(&counters.query_timeouts);
                let _ = command
                    .response
                    .send(Err(BridgeError::Query(QueryError::Timeout)));
                return;
            }
            Err(_) => break,
        }
    }
    let result =
        resolve_single_reply(replies, fields.bounds.max_payload_bytes).map_err(BridgeError::Query);
    let _ = command.response.send(result);
}

fn require_direction(
    descriptor: EndpointDescriptor,
    expected: EndpointDirection,
) -> Result<(), BridgeError> {
    let fields = descriptor.fields();
    if fields.direction != expected {
        return Err(BridgeError::WrongRouteDirection {
            expected,
            actual: fields.direction,
        });
    }
    if matches!(
        expected,
        EndpointDirection::Query | EndpointDirection::Queryable
    ) && !matches!(
        fields.interaction,
        InteractionClass::Query | InteractionClass::Queryable
    ) {
        return Err(BridgeError::WrongRouteInteraction);
    }
    Ok(())
}

/// Zenoh bridge construction, lifecycle and query failure.
#[derive(Debug, Error)]
pub enum BridgeError {
    /// JSON5 or Zenoh configuration was invalid.
    #[error("invalid Zenoh bridge configuration: {0}")]
    Configuration(String),
    /// The bridge command channel must be bounded and nonempty.
    #[error("bridge command capacity must be nonzero")]
    ZeroCommandCapacity,
    /// A validated route was attached to the wrong adapter primitive.
    #[error("route direction mismatch: expected {expected:?}, got {actual:?}")]
    WrongRouteDirection {
        /// Required direction.
        expected: EndpointDirection,
        /// Descriptor direction.
        actual: EndpointDirection,
    },
    /// A query primitive was paired with a non-query interaction.
    #[error("route interaction is incompatible with the adapter primitive")]
    WrongRouteInteraction,
    /// Endpoint route-set validation failed.
    #[error(transparent)]
    Endpoint(#[from] EndpointError),
    /// Worker OS-thread creation failed.
    #[error("failed to spawn Zenoh worker: {0}")]
    ThreadSpawn(#[source] std::io::Error),
    /// Opening the session or declaring routes failed before readiness.
    #[error("Zenoh bridge startup failed: {0}")]
    Startup(String),
    /// The worker panicked during startup or shutdown.
    #[error("Zenoh bridge worker panicked")]
    WorkerPanicked,
    /// The worker has stopped or its command receiver is gone.
    #[error("Zenoh bridge worker is unavailable")]
    WorkerUnavailable,
    /// The bounded worker command queue is full.
    #[error("Zenoh bridge command queue is full")]
    CommandQueueFull,
    /// No registered query route owns the requested endpoint identity.
    #[error("unknown Zenoh query endpoint")]
    UnknownQueryEndpoint,
    /// A query route unexpectedly omitted its required timeout.
    #[error("Zenoh query route has no timeout")]
    MissingQueryTimeout,
    /// A payload exceeded the declared pre-decode bound.
    #[error("payload size {actual} exceeds maximum {maximum}")]
    OversizedPayload {
        /// Actual byte length.
        actual: usize,
        /// Route maximum.
        maximum: usize,
    },
    /// A bounded query failed project-level reply resolution.
    #[error(transparent)]
    Query(#[from] QueryError),
    /// Zenoh returned an operation failure after startup.
    #[error("Zenoh transport operation failed: {0}")]
    Transport(String),
}

#[cfg(test)]
mod tests {
    use message_model::{AsciiId, MAX_TYPE_NAME_LEN};
    use middleware_api::{
        ConcreteKey, EndpointBounds, EndpointFields, FallbackBehavior, KeyDomain, SemanticQos,
    };

    use super::*;

    fn descriptor(
        direction: EndpointDirection,
        interaction: InteractionClass,
    ) -> EndpointDescriptor {
        let qos = match interaction {
            InteractionClass::Query | InteractionClass::Queryable => SemanticQos::RequestReply,
            InteractionClass::Liveliness => SemanticQos::Presence,
            _ => SemanticQos::StateLatest,
        };
        EndpointDescriptor::new(EndpointFields {
            id: AsciiId::parse("route").unwrap(),
            key: ConcreteKey::new(
                AsciiId::parse("r1").unwrap(),
                KeyDomain::Device,
                AsciiId::parse("imu").unwrap(),
                AsciiId::parse("state").unwrap(),
            ),
            protobuf_type: AsciiId::<MAX_TYPE_NAME_LEN>::parse("robot.v1.ImuState").unwrap(),
            direction,
            interaction,
            qos,
            bounds: EndpointBounds {
                max_payload_bytes: 128,
                queue_capacity: 2,
                max_age_ns: if direction == EndpointDirection::Subscribe {
                    Some(1_000)
                } else {
                    None
                },
                timeout_ns: if matches!(
                    interaction,
                    InteractionClass::Query | InteractionClass::Queryable
                ) {
                    Some(1_000)
                } else {
                    None
                },
            },
            allowed_source: if direction == EndpointDirection::Subscribe {
                Some(AsciiId::parse("imu_driver").unwrap())
            } else {
                None
            },
            fallback: FallbackBehavior::Unavailable,
        })
        .unwrap()
    }

    #[test]
    fn route_constructors_reject_wrong_primitives() {
        let publish = descriptor(EndpointDirection::Publish, InteractionClass::LatestStream);
        assert!(InboundRoute::new(publish, |_| Ok(())).is_err());

        let query = descriptor(EndpointDirection::Query, InteractionClass::Query);
        assert!(OutboundRoute::new(query, || Ok(None)).is_err());
    }

    #[test]
    fn bridge_config_rejects_zero_command_capacity() {
        assert!(matches!(
            ZenohBridgeConfig::defaults().with_command_capacity(0),
            Err(BridgeError::ZeroCommandCapacity)
        ));
    }
}
