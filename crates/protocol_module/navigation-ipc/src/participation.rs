//! Fixed local inspection contract for motion-participating navigation stages.
//!
//! This channel is diagnostic only.  It carries no authority and cannot alter
//! action dispatch, proposal admission, or the motor-disabled output gate.

use arrayvec::ArrayVec;
use bytemuck::{Pod, Zeroable};
use message_model::{AsciiId, Header, Id128, MAX_ID_LEN};
use middleware_shm::{ShmCodec, ShmPayload};

use crate::{
    NavigationIpcError,
    wire::{NowFn, WireAsciiId, WireHeader, monotonic_now_ns, validate_header_at},
};

/// Maximum stage records retained in one navigation participation snapshot.
pub const MAX_NAVIGATION_PARTICIPANTS: usize = 16;

/// Stable status of one stage participating in a navigation command.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NavigationParticipantState {
    /// The stage has no current work for this command.
    Inactive,
    /// The stage is waiting on a bounded predecessor or acknowledgement.
    Pending,
    /// The stage currently contributes to command execution.
    Active,
    /// The stage reached a terminal result for the command.
    Terminal,
    /// The stage observed a typed fault or unusable dependency.
    Faulted,
}

impl NavigationParticipantState {
    /// Stable lowercase identifier for operator tools.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Inactive => "inactive",
            Self::Pending => "pending",
            Self::Active => "active",
            Self::Terminal => "terminal",
            Self::Faulted => "faulted",
        }
    }
}

/// Typed explanation for one participant state.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum NavigationParticipantReason {
    /// No additional explanation is applicable.
    None,
    /// The behavior-tree leaf is waiting to dispatch its command.
    WaitingDispatch,
    /// The behavior-tree leaf is waiting for the bridge acknowledgement.
    WaitingAcknowledgement,
    /// The action server accepted the correlated goal.
    GoalAccepted,
    /// The action server rejected the correlated goal.
    GoalRejected,
    /// The action server is executing the correlated goal.
    Executing,
    /// The behavior tree requested goal cancellation.
    CancelRequested,
    /// The action server accepted the cancellation request.
    CancelAccepted,
    /// The action completed successfully.
    Succeeded,
    /// The action completed with failure.
    Failed,
    /// The action completed cancellation.
    Cancelled,
    /// The behavior-tree action deadline elapsed.
    TimedOut,
    /// Current chassis authority is available for the command.
    AuthorityAvailable,
    /// Current chassis authority is unavailable or expired.
    AuthorityUnavailable,
    /// A correlated raw controller proposal was accepted by the final owner.
    ProposalAccepted,
    /// No usable controller proposal is available.
    ProposalUnavailable,
    /// The final limiter produced a bounded command.
    FinalLimited,
    /// The fake output sink received a nonzero command.
    FakeOutputApplied,
    /// The fake output sink received a typed zero-output decision.
    OutputStopped,
    /// The proposal-producing peer disappeared.
    PeerUnavailable,
}

impl NavigationParticipantReason {
    /// Stable lowercase identifier for operator tools.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::None => "none",
            Self::WaitingDispatch => "waiting_dispatch",
            Self::WaitingAcknowledgement => "waiting_acknowledgement",
            Self::GoalAccepted => "goal_accepted",
            Self::GoalRejected => "goal_rejected",
            Self::Executing => "executing",
            Self::CancelRequested => "cancel_requested",
            Self::CancelAccepted => "cancel_accepted",
            Self::Succeeded => "succeeded",
            Self::Failed => "failed",
            Self::Cancelled => "cancelled",
            Self::TimedOut => "timed_out",
            Self::AuthorityAvailable => "authority_available",
            Self::AuthorityUnavailable => "authority_unavailable",
            Self::ProposalAccepted => "proposal_accepted",
            Self::ProposalUnavailable => "proposal_unavailable",
            Self::FinalLimited => "final_limited",
            Self::FakeOutputApplied => "fake_output_applied",
            Self::OutputStopped => "output_stopped",
            Self::PeerUnavailable => "peer_unavailable",
        }
    }
}

/// One stable participant identity with typed current state and reason.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NavigationParticipant {
    id: AsciiId<MAX_ID_LEN>,
    state: NavigationParticipantState,
    reason: NavigationParticipantReason,
}

impl NavigationParticipant {
    /// Creates one participant record.
    #[must_use]
    pub const fn new(
        id: AsciiId<MAX_ID_LEN>,
        state: NavigationParticipantState,
        reason: NavigationParticipantReason,
    ) -> Self {
        Self { id, state, reason }
    }

    /// Returns the stable participant identifier.
    #[must_use]
    pub const fn id(self) -> AsciiId<MAX_ID_LEN> {
        self.id
    }

    /// Returns the current participant state.
    #[must_use]
    pub const fn state(self) -> NavigationParticipantState {
        self.state
    }

    /// Returns the typed explanation for the state.
    #[must_use]
    pub const fn reason(self) -> NavigationParticipantReason {
        self.reason
    }
}

/// Fixed-capacity, command-correlated movement participation snapshot.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NavigationParticipationSnapshot {
    header: Header,
    process_epoch: u64,
    command_id: Option<Id128>,
    participants: ArrayVec<NavigationParticipant, MAX_NAVIGATION_PARTICIPANTS>,
}

impl NavigationParticipationSnapshot {
    /// Constructs a snapshot after checking process identity and unique stage IDs.
    pub fn new(
        header: Header,
        process_epoch: u64,
        command_id: Option<Id128>,
        participants: ArrayVec<NavigationParticipant, MAX_NAVIGATION_PARTICIPANTS>,
    ) -> Result<Self, NavigationIpcError> {
        if process_epoch == 0 {
            return Err(NavigationIpcError::Participation("zero process epoch"));
        }
        if participants.is_empty() {
            return Err(NavigationIpcError::Participation("empty participant set"));
        }
        for (index, participant) in participants.iter().enumerate() {
            if participants[..index]
                .iter()
                .any(|previous| previous.id == participant.id)
            {
                return Err(NavigationIpcError::Participation(
                    "duplicate participant ID",
                ));
            }
        }
        Ok(Self {
            header,
            process_epoch,
            command_id,
            participants,
        })
    }

    /// Returns the source/boot/freshness metadata for this snapshot.
    #[must_use]
    pub const fn header(&self) -> Header {
        self.header
    }

    /// Returns the supervisor-owned process epoch.
    #[must_use]
    pub const fn process_epoch(&self) -> u64 {
        self.process_epoch
    }

    /// Returns the canonical command identity when one is active.
    #[must_use]
    pub const fn command_id(&self) -> Option<Id128> {
        self.command_id
    }

    /// Returns participant records in deterministic publisher order.
    #[must_use]
    pub fn participants(&self) -> &[NavigationParticipant] {
        self.participants.as_slice()
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq, Pod, Zeroable)]
struct NavigationParticipantRepr {
    id: WireAsciiId,
    state: u8,
    reason: u8,
    reserved: [u8; 6],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct NavigationParticipationRepr {
    header: WireHeader,
    process_epoch: u64,
    command_id: [u8; 16],
    participants: [NavigationParticipantRepr; MAX_NAVIGATION_PARTICIPANTS],
    participant_count: u8,
    command_present: u8,
    reserved: [u8; 6],
}

impl ShmPayload for NavigationParticipationRepr {
    const SCHEMA_ID: [u8; 16] = *b"rs-nav-part-v001";
    const SCHEMA_VERSION: u32 = 1;
}

/// Participation codec using receive-time freshness validation.
#[derive(Clone, Copy)]
pub struct NavigationParticipationCodec {
    now: NowFn,
}

impl NavigationParticipationCodec {
    /// Uses a caller-supplied local monotonic clock function.
    #[must_use]
    pub const fn new(now: NowFn) -> Self {
        Self { now }
    }
}

impl Default for NavigationParticipationCodec {
    fn default() -> Self {
        Self::new(monotonic_now_ns)
    }
}

impl ShmCodec<NavigationParticipationSnapshot> for NavigationParticipationCodec {
    type Repr = NavigationParticipationRepr;
    type Error = NavigationIpcError;

    fn encode(&mut self, value: &NavigationParticipationSnapshot) -> Self::Repr {
        let mut participants = [NavigationParticipantRepr::zeroed(); MAX_NAVIGATION_PARTICIPANTS];
        for (index, participant) in value.participants().iter().enumerate() {
            participants[index] = NavigationParticipantRepr {
                id: WireAsciiId::encode(participant.id()),
                state: encode_state(participant.state()),
                reason: encode_reason(participant.reason()),
                reserved: [0; 6],
            };
        }
        NavigationParticipationRepr {
            header: WireHeader::encode(value.header()),
            process_epoch: value.process_epoch(),
            command_id: value.command_id().map_or([0; 16], |id| *id.as_bytes()),
            participants,
            participant_count: value.participants().len() as u8,
            command_present: u8::from(value.command_id().is_some()),
            reserved: [0; 6],
        }
    }

    fn decode(
        &mut self,
        value: Self::Repr,
    ) -> Result<NavigationParticipationSnapshot, Self::Error> {
        let participant_count = usize::from(value.participant_count);
        if value.reserved != [0; 6]
            || value.command_present > 1
            || participant_count == 0
            || participant_count > MAX_NAVIGATION_PARTICIPANTS
            || value.participants[participant_count..]
                .iter()
                .any(|participant| *participant != NavigationParticipantRepr::zeroed())
        {
            return Err(NavigationIpcError::Participation(
                "participant count, flags, or padding",
            ));
        }
        let command_id = if value.command_present == 1 {
            Some(Id128::new(value.command_id)?)
        } else {
            if value.command_id != [0; 16] {
                return Err(NavigationIpcError::Participation(
                    "absent command contains identity",
                ));
            }
            None
        };
        let mut participants = ArrayVec::new();
        for participant in &value.participants[..participant_count] {
            if participant.reserved != [0; 6] {
                return Err(NavigationIpcError::Participation("participant padding"));
            }
            participants
                .try_push(NavigationParticipant::new(
                    participant.id.decode::<MAX_ID_LEN>()?,
                    decode_state(participant.state)?,
                    decode_reason(participant.reason)?,
                ))
                .map_err(|_| NavigationIpcError::Participation("participant capacity"))?;
        }
        NavigationParticipationSnapshot::new(
            validate_header_at(value.header.decode()?, (self.now)())?,
            value.process_epoch,
            command_id,
            participants,
        )
    }
}

const fn encode_state(value: NavigationParticipantState) -> u8 {
    match value {
        NavigationParticipantState::Inactive => 1,
        NavigationParticipantState::Pending => 2,
        NavigationParticipantState::Active => 3,
        NavigationParticipantState::Terminal => 4,
        NavigationParticipantState::Faulted => 5,
    }
}

fn decode_state(value: u8) -> Result<NavigationParticipantState, NavigationIpcError> {
    match value {
        1 => Ok(NavigationParticipantState::Inactive),
        2 => Ok(NavigationParticipantState::Pending),
        3 => Ok(NavigationParticipantState::Active),
        4 => Ok(NavigationParticipantState::Terminal),
        5 => Ok(NavigationParticipantState::Faulted),
        _ => Err(NavigationIpcError::Participation("participant state")),
    }
}

const fn encode_reason(value: NavigationParticipantReason) -> u8 {
    match value {
        NavigationParticipantReason::None => 0,
        NavigationParticipantReason::WaitingDispatch => 1,
        NavigationParticipantReason::WaitingAcknowledgement => 2,
        NavigationParticipantReason::GoalAccepted => 3,
        NavigationParticipantReason::GoalRejected => 4,
        NavigationParticipantReason::Executing => 5,
        NavigationParticipantReason::CancelRequested => 6,
        NavigationParticipantReason::CancelAccepted => 7,
        NavigationParticipantReason::Succeeded => 8,
        NavigationParticipantReason::Failed => 9,
        NavigationParticipantReason::Cancelled => 10,
        NavigationParticipantReason::TimedOut => 11,
        NavigationParticipantReason::AuthorityAvailable => 12,
        NavigationParticipantReason::AuthorityUnavailable => 13,
        NavigationParticipantReason::ProposalAccepted => 14,
        NavigationParticipantReason::ProposalUnavailable => 15,
        NavigationParticipantReason::FinalLimited => 16,
        NavigationParticipantReason::FakeOutputApplied => 17,
        NavigationParticipantReason::OutputStopped => 18,
        NavigationParticipantReason::PeerUnavailable => 19,
    }
}

fn decode_reason(value: u8) -> Result<NavigationParticipantReason, NavigationIpcError> {
    match value {
        0 => Ok(NavigationParticipantReason::None),
        1 => Ok(NavigationParticipantReason::WaitingDispatch),
        2 => Ok(NavigationParticipantReason::WaitingAcknowledgement),
        3 => Ok(NavigationParticipantReason::GoalAccepted),
        4 => Ok(NavigationParticipantReason::GoalRejected),
        5 => Ok(NavigationParticipantReason::Executing),
        6 => Ok(NavigationParticipantReason::CancelRequested),
        7 => Ok(NavigationParticipantReason::CancelAccepted),
        8 => Ok(NavigationParticipantReason::Succeeded),
        9 => Ok(NavigationParticipantReason::Failed),
        10 => Ok(NavigationParticipantReason::Cancelled),
        11 => Ok(NavigationParticipantReason::TimedOut),
        12 => Ok(NavigationParticipantReason::AuthorityAvailable),
        13 => Ok(NavigationParticipantReason::AuthorityUnavailable),
        14 => Ok(NavigationParticipantReason::ProposalAccepted),
        15 => Ok(NavigationParticipantReason::ProposalUnavailable),
        16 => Ok(NavigationParticipantReason::FinalLimited),
        17 => Ok(NavigationParticipantReason::FakeOutputApplied),
        18 => Ok(NavigationParticipantReason::OutputStopped),
        19 => Ok(NavigationParticipantReason::PeerUnavailable),
        _ => Err(NavigationIpcError::Participation("participant reason")),
    }
}
