use arrayvec::ArrayVec;
use message_model::{ActionStatus, Id128};
use thiserror::Error;

/// Bounded action command correlation independent of Zenoh delivery retries.
#[derive(Clone, Debug)]
pub struct ActionCorrelations<const CAPACITY: usize> {
    active: ArrayVec<Id128, CAPACITY>,
}

/// Bounded non-realtime retry policy for correlated action commands.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ActionRetryPolicy {
    retry_interval_ns: u64,
    timeout_ns: u64,
    maximum_attempts: u32,
}

impl ActionRetryPolicy {
    /// Creates a policy whose attempt count includes the initial send.
    pub fn new(
        retry_interval_ns: u64,
        timeout_ns: u64,
        maximum_attempts: u32,
    ) -> Result<Self, CorrelationError> {
        if retry_interval_ns == 0 || timeout_ns == 0 || maximum_attempts == 0 {
            return Err(CorrelationError::InvalidRetryPolicy);
        }
        Ok(Self {
            retry_interval_ns,
            timeout_ns,
            maximum_attempts,
        })
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PendingRetry {
    command_id: Id128,
    policy: ActionRetryPolicy,
    attempts_sent: u32,
    next_retry_ns: u64,
    expires_ns: u64,
}

/// Result of polling one action's bounded retry state.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RetryDecision {
    /// No retry is due yet.
    NotDue,
    /// The caller should resend the same command identity outside RT.
    Retry {
        /// One-based attempt number including the initial send.
        attempt: u32,
    },
    /// Timeout or maximum attempts removed the correlation.
    Exhausted,
}

/// Fixed-capacity action retry/correlation state owned by a non-RT bridge.
#[derive(Clone, Debug)]
pub struct ActionRetryState<const CAPACITY: usize> {
    active: ArrayVec<PendingRetry, CAPACITY>,
}

impl<const CAPACITY: usize> Default for ActionRetryState<CAPACITY> {
    fn default() -> Self {
        Self {
            active: ArrayVec::new(),
        }
    }
}

impl<const CAPACITY: usize> ActionRetryState<CAPACITY> {
    /// Records an already-sent initial action command.
    pub fn register(
        &mut self,
        command_id: Id128,
        now_ns: u64,
        policy: ActionRetryPolicy,
    ) -> Result<(), CorrelationError> {
        if self
            .active
            .iter()
            .any(|pending| pending.command_id == command_id)
        {
            return Err(CorrelationError::DuplicateCommand);
        }
        let next_retry_ns = now_ns
            .checked_add(policy.retry_interval_ns)
            .ok_or(CorrelationError::TimeOverflow)?;
        let expires_ns = now_ns
            .checked_add(policy.timeout_ns)
            .ok_or(CorrelationError::TimeOverflow)?;
        self.active
            .try_push(PendingRetry {
                command_id,
                policy,
                attempts_sent: 1,
                next_retry_ns,
                expires_ns,
            })
            .map_err(|_| CorrelationError::CapacityExhausted)
    }

    /// Polls whether the same command identity should be retried now.
    pub fn poll(
        &mut self,
        command_id: Id128,
        now_ns: u64,
    ) -> Result<RetryDecision, CorrelationError> {
        let index = self
            .active
            .iter()
            .position(|pending| pending.command_id == command_id)
            .ok_or(CorrelationError::UnknownCommand)?;
        let pending = &mut self.active[index];
        if now_ns >= pending.expires_ns || pending.attempts_sent >= pending.policy.maximum_attempts
        {
            self.active.remove(index);
            return Ok(RetryDecision::Exhausted);
        }
        if now_ns < pending.next_retry_ns {
            return Ok(RetryDecision::NotDue);
        }
        pending.attempts_sent += 1;
        pending.next_retry_ns = now_ns
            .checked_add(pending.policy.retry_interval_ns)
            .ok_or(CorrelationError::TimeOverflow)?;
        Ok(RetryDecision::Retry {
            attempt: pending.attempts_sent,
        })
    }

    /// Applies an acknowledgement and removes terminal correlations.
    pub fn acknowledge(
        &mut self,
        command_id: Id128,
        status: ActionStatus,
    ) -> Result<bool, CorrelationError> {
        let index = self
            .active
            .iter()
            .position(|pending| pending.command_id == command_id)
            .ok_or(CorrelationError::UnknownCommand)?;
        if status.is_terminal() {
            self.active.remove(index);
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Returns the bounded active correlation count.
    #[must_use]
    pub fn len(&self) -> usize {
        self.active.len()
    }

    /// Returns whether the retry state is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.active.is_empty()
    }
}

impl<const CAPACITY: usize> Default for ActionCorrelations<CAPACITY> {
    fn default() -> Self {
        Self {
            active: ArrayVec::new(),
        }
    }
}

impl<const CAPACITY: usize> ActionCorrelations<CAPACITY> {
    /// Registers a new action identity.
    pub fn register(&mut self, command_id: Id128) -> Result<(), CorrelationError> {
        if self.active.contains(&command_id) {
            return Err(CorrelationError::DuplicateCommand);
        }
        self.active
            .try_push(command_id)
            .map_err(|_| CorrelationError::CapacityExhausted)
    }

    /// Applies an acknowledgement and removes terminal actions.
    pub fn acknowledge(
        &mut self,
        command_id: Id128,
        status: ActionStatus,
    ) -> Result<bool, CorrelationError> {
        let index = self
            .active
            .iter()
            .position(|active| *active == command_id)
            .ok_or(CorrelationError::UnknownCommand)?;
        if status.is_terminal() {
            self.active.remove(index);
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Returns the number of active bounded correlations.
    #[must_use]
    pub fn len(&self) -> usize {
        self.active.len()
    }

    /// Returns whether no action is active.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.active.is_empty()
    }
}

/// One non-RT query reply before project-level resolution.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum QueryReply {
    /// Successful reply payload.
    Data(Vec<u8>),
    /// Remote queryable returned a typed error payload.
    RemoteError(Vec<u8>),
}

/// Resolves Zenoh's zero-or-more replies to the project's exactly-one contract.
pub fn resolve_single_reply(
    replies: impl IntoIterator<Item = QueryReply>,
    maximum_payload_bytes: usize,
) -> Result<Vec<u8>, QueryError> {
    if maximum_payload_bytes == 0 {
        return Err(QueryError::ZeroPayloadBound);
    }
    let mut replies = replies.into_iter();
    let first = replies.next().ok_or(QueryError::NoReply)?;
    if replies.next().is_some() {
        return Err(QueryError::MultipleReplies);
    }
    match first {
        QueryReply::Data(payload) if payload.len() <= maximum_payload_bytes => Ok(payload),
        QueryReply::Data(payload) => Err(QueryError::OversizedReply {
            actual: payload.len(),
            maximum: maximum_payload_bytes,
        }),
        QueryReply::RemoteError(payload) => Err(QueryError::RemoteError(payload)),
    }
}

/// Action-correlation failure.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum CorrelationError {
    /// Command identity is already active.
    #[error("action command identity is already active")]
    DuplicateCommand,
    /// Fixed correlation capacity is exhausted.
    #[error("action correlation capacity is exhausted")]
    CapacityExhausted,
    /// Ack/result referenced no active command.
    #[error("action acknowledgement references an unknown command")]
    UnknownCommand,
    /// Retry durations and attempt count must all be nonzero.
    #[error("action retry policy must use nonzero bounds")]
    InvalidRetryPolicy,
    /// Retry schedule arithmetic exceeded the monotonic representation.
    #[error("action retry schedule overflowed monotonic time")]
    TimeOverflow,
}

/// Bounded query resolution failure.
#[derive(Clone, Debug, Eq, Error, PartialEq)]
pub enum QueryError {
    /// Query bound must be explicit.
    #[error("query payload bound must be nonzero")]
    ZeroPayloadBound,
    /// No queryable replied before the caller's timeout.
    #[error("query returned no reply")]
    NoReply,
    /// The bounded request window elapsed before completion.
    #[error("query timed out")]
    Timeout,
    /// More than one owner replied to an exactly-one route.
    #[error("query returned multiple replies")]
    MultipleReplies,
    /// Reply exceeded the pre-decode bound.
    #[error("query reply size {actual} exceeds maximum {maximum}")]
    OversizedReply {
        /// Observed payload size.
        actual: usize,
        /// Configured maximum.
        maximum: usize,
    },
    /// Remote queryable rejected the request.
    #[error("remote query error")]
    RemoteError(Vec<u8>),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn action_correlation_requires_known_id_and_terminal_removes_it() {
        let id = Id128::new([1; 16]).unwrap();
        let mut tracker = ActionCorrelations::<1>::default();
        tracker.register(id).unwrap();
        assert_eq!(
            tracker.register(id),
            Err(CorrelationError::DuplicateCommand)
        );
        assert_eq!(tracker.acknowledge(id, ActionStatus::Executing), Ok(false));
        assert_eq!(tracker.acknowledge(id, ActionStatus::Succeeded), Ok(true));
        assert!(tracker.is_empty());
        assert_eq!(
            tracker.acknowledge(id, ActionStatus::Succeeded),
            Err(CorrelationError::UnknownCommand)
        );
    }

    #[test]
    fn action_retry_is_bounded_by_attempts_timeout_and_terminal_ack() {
        let first = Id128::new([1; 16]).unwrap();
        let second = Id128::new([2; 16]).unwrap();
        let policy = ActionRetryPolicy::new(10, 100, 3).unwrap();
        let mut retries = ActionRetryState::<2>::default();
        retries.register(first, 1_000, policy).unwrap();
        assert_eq!(retries.poll(first, 1_009), Ok(RetryDecision::NotDue));
        assert_eq!(
            retries.poll(first, 1_010),
            Ok(RetryDecision::Retry { attempt: 2 })
        );
        assert_eq!(
            retries.poll(first, 1_020),
            Ok(RetryDecision::Retry { attempt: 3 })
        );
        assert_eq!(retries.poll(first, 1_021), Ok(RetryDecision::Exhausted));
        assert!(retries.is_empty());

        retries.register(second, 2_000, policy).unwrap();
        assert_eq!(
            retries.acknowledge(second, ActionStatus::Executing),
            Ok(false)
        );
        assert_eq!(
            retries.acknowledge(second, ActionStatus::Succeeded),
            Ok(true)
        );
        assert!(retries.is_empty());
        assert_eq!(
            ActionRetryPolicy::new(0, 100, 3),
            Err(CorrelationError::InvalidRetryPolicy)
        );
    }

    #[test]
    fn query_resolution_rejects_zero_multiple_error_and_oversize() {
        assert_eq!(
            resolve_single_reply(Vec::new(), 8),
            Err(QueryError::NoReply)
        );
        assert_eq!(
            resolve_single_reply([QueryReply::Data(vec![1]), QueryReply::Data(vec![2])], 8),
            Err(QueryError::MultipleReplies)
        );
        assert!(matches!(
            resolve_single_reply([QueryReply::Data(vec![0; 9])], 8),
            Err(QueryError::OversizedReply { .. })
        ));
        assert_eq!(
            resolve_single_reply([QueryReply::RemoteError(vec![4])], 8),
            Err(QueryError::RemoteError(vec![4]))
        );
    }
}
