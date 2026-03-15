//! Named non-realtime worker with bounded nonblocking SPSC queues.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use message_model::ArmJointObservation;
use rtrb::{Consumer, Producer, RingBuffer};

use crate::{
    ActionDecoder, EncodedPolicyObservation, MAX_POLICY_RECURRENT_ELEMENTS, PolicyDecision,
    PolicyEngine, PolicyError, PolicyGeneration, PolicyIdentity, PolicyObservationEncoder,
    ReadyPolicy,
};

struct InferenceRequest {
    sequence: u64,
    enqueued_at: Instant,
    observation: EncodedPolicyObservation,
    recurrent_state: arrayvec::ArrayVec<f32, MAX_POLICY_RECURRENT_ELEMENTS>,
}

struct InferenceResult {
    sequence: u64,
    enqueued_at: Instant,
    started_at: Instant,
    finished_at: Instant,
    observation: EncodedPolicyObservation,
    result: Result<crate::EngineOutput, PolicyError>,
}

#[derive(Default)]
struct SharedCounters {
    submitted: AtomicU64,
    request_full: AtomicU64,
    result_full: AtomicU64,
    inference_error: AtomicU64,
    deadline_expired: AtomicU64,
    stale_result: AtomicU64,
}

/// Saturating runtime evidence snapshot.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PolicyCounterSnapshot {
    /// Accepted requests.
    pub submitted: u64,
    /// Request queue saturation.
    pub request_full: u64,
    /// Result queue saturation.
    pub result_full: u64,
    /// Model/decode errors.
    pub inference_error: u64,
    /// Exclusive deadline failures.
    pub deadline_expired: u64,
    /// Superseded identity/sequence results.
    pub stale_result: u64,
}

/// Observable terminal worker status.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PolicyTerminalReason {
    /// Caller requested a clean shutdown or disarmed reload.
    CleanShutdown,
    /// An external supervisor reported loss of an isolated policy process.
    SupervisedProcessLost,
}

/// Control-facing policy runtime. Calls are nonblocking; only the worker owns
/// RTen and allocated model tensors.
pub struct PolicyRuntime {
    request_tx: Option<Producer<InferenceRequest>>,
    result_rx: Consumer<InferenceResult>,
    encoder: PolicyObservationEncoder,
    decoder: ActionDecoder,
    identity: PolicyIdentity,
    generation: PolicyGeneration,
    deadline: Duration,
    period: Duration,
    last_submit: Option<Instant>,
    latest_sequence: u64,
    next_sequence: u64,
    alive: Arc<AtomicBool>,
    stop: Arc<AtomicBool>,
    counters: Arc<SharedCounters>,
    worker: Option<JoinHandle<()>>,
    terminal: Arc<AtomicU64>,
    recurrent_reset: arrayvec::ArrayVec<f32, MAX_POLICY_RECURRENT_ELEMENTS>,
    recurrent_state: arrayvec::ArrayVec<f32, MAX_POLICY_RECURRENT_ELEMENTS>,
    recurrent_pending: bool,
}

impl PolicyRuntime {
    /// Spawn the dedicated worker after all artifact, graph and morphology
    /// validation has completed.
    pub fn spawn(policy: ReadyPolicy) -> Result<Self, PolicyError> {
        let manifest = policy.manifest().clone();
        let identity = policy.identity();
        let generation = policy.generation();
        let control = policy.control().clone();
        let encoder = PolicyObservationEncoder::new(
            manifest.clone(),
            identity,
            generation,
            control.profile_id(),
            control.group_id(),
            control.observation_source(),
            control.observation_boot(),
        );
        let decoder = ActionDecoder::new(manifest.action.clone(), control, identity, generation);
        let mut recurrent_reset = arrayvec::ArrayVec::new();
        for pair in &manifest.recurrent {
            recurrent_reset
                .try_extend_from_slice(&pair.reset)
                .map_err(|_| PolicyError::Manifest("recurrent reset capacity exceeded".into()))?;
        }
        let recurrent_state = recurrent_reset.clone();
        let engine = PolicyEngine::new(policy)?;
        let (request_tx, request_rx) = RingBuffer::new(manifest.queue_capacity);
        let (result_tx, result_rx) = RingBuffer::new(manifest.queue_capacity);
        let alive = Arc::new(AtomicBool::new(false));
        let stop = Arc::new(AtomicBool::new(false));
        let counters = Arc::new(SharedCounters::default());
        let terminal = Arc::new(AtomicU64::new(0));
        let worker_alive = Arc::clone(&alive);
        let worker_stop = Arc::clone(&stop);
        let worker_counters = Arc::clone(&counters);
        let worker_terminal = Arc::clone(&terminal);
        let worker = std::thread::Builder::new()
            .name("rs-rl-policy".into())
            .spawn(move || {
                run_worker(
                    engine,
                    request_rx,
                    result_tx,
                    &worker_alive,
                    &worker_stop,
                    &worker_counters,
                    &worker_terminal,
                );
            })
            .map_err(|_| PolicyError::WorkerUnavailable)?;
        alive.store(true, Ordering::Release);
        Ok(Self {
            request_tx: Some(request_tx),
            result_rx,
            encoder,
            decoder,
            identity,
            generation,
            deadline: Duration::from_nanos(manifest.inference_deadline_ns),
            period: Duration::from_nanos(manifest.inference_period_ns),
            last_submit: None,
            latest_sequence: 0,
            next_sequence: 1,
            alive,
            stop,
            counters,
            worker: Some(worker),
            terminal,
            recurrent_reset,
            recurrent_state,
            recurrent_pending: false,
        })
    }

    /// Validate and nonblockingly submit one semantic observation.
    pub fn try_submit(
        &mut self,
        observation: ArmJointObservation,
        now_monotonic_ns: u64,
        now: Instant,
    ) -> Result<u64, PolicyError> {
        if !self.alive.load(Ordering::Acquire) {
            self.decoder.reset();
            self.reset_recurrent();
            return Err(PolicyError::WorkerUnavailable);
        }
        if !self.recurrent_state.is_empty() && self.recurrent_pending {
            saturating_increment(&self.counters.request_full);
            self.latest_sequence = 0;
            self.decoder.reset();
            self.reset_recurrent();
            return Err(PolicyError::QueueFull);
        }
        if self
            .last_submit
            .is_some_and(|previous| now.saturating_duration_since(previous) < self.period)
        {
            return Err(PolicyError::Observation(
                "inference period has not elapsed".into(),
            ));
        }
        let encoded = self.encoder.encode(observation, now_monotonic_ns)?;
        let sequence = self.next_sequence;
        let next_sequence = sequence
            .checked_add(1)
            .ok_or_else(|| PolicyError::Inference("request sequence exhausted".into()))?;
        let request = InferenceRequest {
            sequence,
            enqueued_at: now,
            observation: encoded,
            recurrent_state: self.recurrent_state.clone(),
        };
        let push = self
            .request_tx
            .as_mut()
            .ok_or(PolicyError::WorkerUnavailable)?
            .push(request);
        if push.is_err() {
            saturating_increment(&self.counters.request_full);
            self.decoder.reset();
            self.reset_recurrent();
            return Err(PolicyError::QueueFull);
        }
        self.latest_sequence = sequence;
        self.next_sequence = next_sequence;
        self.last_submit = Some(now);
        self.recurrent_pending = !self.recurrent_state.is_empty();
        saturating_increment(&self.counters.submitted);
        Ok(sequence)
    }

    /// Nonblockingly receive and admit only the latest, on-time result.
    pub fn try_receive(
        &mut self,
        accepted_time_ns: u64,
        now: Instant,
    ) -> Result<PolicyDecision, PolicyError> {
        if !self.alive.load(Ordering::Acquire) {
            self.decoder.reset();
            self.reset_recurrent();
            return Err(PolicyError::WorkerUnavailable);
        }
        let result = self
            .result_rx
            .pop()
            .map_err(|_| PolicyError::ResultUnavailable)?;
        self.recurrent_pending = false;
        let age = now.saturating_duration_since(result.enqueued_at);
        let inference = result
            .finished_at
            .saturating_duration_since(result.started_at);
        if age >= self.deadline || inference >= self.deadline {
            saturating_increment(&self.counters.deadline_expired);
            self.decoder.reset();
            self.reset_recurrent();
            return Err(PolicyError::DeadlineExpired);
        }
        if result.sequence != self.latest_sequence
            || result.observation.identity() != self.identity
            || result.observation.generation() != self.generation
        {
            saturating_increment(&self.counters.stale_result);
            self.decoder.reset();
            self.reset_recurrent();
            return Err(PolicyError::StaleResult);
        }
        let output = match result.result {
            Ok(output) => output,
            Err(error) => {
                saturating_increment(&self.counters.inference_error);
                self.decoder.reset();
                self.reset_recurrent();
                return Err(error);
            }
        };
        let decision = self.decoder.decode(
            result.sequence,
            &output.values,
            result.observation.observation(),
            accepted_time_ns,
        );
        match decision {
            Ok(decision) => {
                self.recurrent_state = output.recurrent_state;
                Ok(decision)
            }
            Err(error) => {
                self.reset_recurrent();
                Err(error)
            }
        }
    }

    /// Returns worker availability.
    #[must_use]
    pub fn is_ready(&self) -> bool {
        self.alive.load(Ordering::Acquire)
    }

    /// Return a terminal reason after the worker has stopped.
    #[must_use]
    pub fn terminal_reason(&self) -> Option<PolicyTerminalReason> {
        match self.terminal.load(Ordering::Acquire) {
            1 => Some(PolicyTerminalReason::CleanShutdown),
            2 => Some(PolicyTerminalReason::SupervisedProcessLost),
            _ => None,
        }
    }

    /// Returns saturating evidence without resetting counters.
    #[must_use]
    pub fn counters(&self) -> PolicyCounterSnapshot {
        PolicyCounterSnapshot {
            submitted: self.counters.submitted.load(Ordering::Relaxed),
            request_full: self.counters.request_full.load(Ordering::Relaxed),
            result_full: self.counters.result_full.load(Ordering::Relaxed),
            inference_error: self.counters.inference_error.load(Ordering::Relaxed),
            deadline_expired: self.counters.deadline_expired.load(Ordering::Relaxed),
            stale_result: self.counters.stale_result.load(Ordering::Relaxed),
        }
    }

    /// Explicitly invalidate control-side rate and replay state.
    pub fn reset(&mut self) {
        self.decoder.reset();
        self.encoder.reset();
        self.last_submit = None;
        self.latest_sequence = 0;
        self.reset_recurrent();
    }

    /// Stop the named worker and invalidate all pending state.
    pub fn shutdown(&mut self) {
        self.stop_with_reason(1);
    }

    /// Fail closed when an optional external process supervisor reports loss.
    pub fn mark_supervised_process_lost(&mut self) {
        self.stop_with_reason(2);
    }

    fn stop_with_reason(&mut self, reason: u64) {
        self.stop.store(true, Ordering::Release);
        self.request_tx.take();
        if let Some(worker) = self.worker.take() {
            let _ = worker.join();
        }
        self.terminal.store(reason, Ordering::Release);
        self.decoder.reset();
        self.reset_recurrent();
        self.latest_sequence = 0;
    }

    /// Perform a disarmed reload. Consuming the old runtime guarantees its
    /// queues, state and worker are destroyed before the new generation starts.
    pub fn reload(mut self, policy: ReadyPolicy) -> Result<Self, PolicyError> {
        self.shutdown();
        drop(self);
        Self::spawn(policy)
    }

    fn reset_recurrent(&mut self) {
        self.recurrent_state.clone_from(&self.recurrent_reset);
        self.recurrent_pending = false;
    }
}

impl Drop for PolicyRuntime {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn run_worker(
    engine: PolicyEngine,
    mut request_rx: Consumer<InferenceRequest>,
    mut result_tx: Producer<InferenceResult>,
    alive: &AtomicBool,
    stop: &AtomicBool,
    counters: &SharedCounters,
    terminal: &AtomicU64,
) {
    alive.store(true, Ordering::Release);
    while !stop.load(Ordering::Acquire) {
        let Ok(request) = request_rx.pop() else {
            std::thread::yield_now();
            continue;
        };
        let started_at = Instant::now();
        let result = engine.infer(&request.observation, &request.recurrent_state);
        let finished_at = Instant::now();
        let result = InferenceResult {
            sequence: request.sequence,
            enqueued_at: request.enqueued_at,
            started_at,
            finished_at,
            observation: request.observation,
            result,
        };
        if result_tx.push(result).is_err() {
            saturating_increment(&counters.result_full);
        }
    }
    alive.store(false, Ordering::Release);
    let _ = terminal.compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire);
}

fn saturating_increment(counter: &AtomicU64) {
    let _ = counter.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |value| {
        Some(value.saturating_add(1))
    });
}
