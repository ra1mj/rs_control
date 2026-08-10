use bytemuck::{Pod, Zeroable};
use contract_codec::{
    decode_action_ack, decode_action_cancel_request, decode_action_cancel_response,
    decode_action_feedback, decode_action_goal_intent, decode_action_result,
    decode_navigate_to_pose_feedback, decode_navigate_to_pose_goal, decode_navigate_to_pose_result,
    encode_action_ack, encode_action_cancel_request, encode_action_cancel_response,
    encode_action_feedback, encode_action_goal_intent, encode_action_result,
};
use message_model::{
    ActionAck, ActionCancelRequest, ActionCancelResponse, ActionFeedback, ActionGoalIntent,
    ActionResult, Header,
};
use middleware_shm::{ShmCodec, ShmPayload};

use crate::{
    NavigationIpcError,
    wire::{NowFn, monotonic_now_ns, validate_header_at},
};

/// Exact local action identifier accepted by the navigation bridge.
pub const NAVIGATE_TO_POSE_ACTION: &str = "navigate_to_pose_v1";
/// Exact registered goal payload type.
pub const NAVIGATE_TO_POSE_GOAL_TYPE: &str = "robot.v1.NavigateToPoseGoal";
/// Exact registered feedback payload type.
pub const NAVIGATE_TO_POSE_FEEDBACK_TYPE: &str = "robot.v1.NavigateToPoseFeedback";
/// Exact registered result payload type.
pub const NAVIGATE_TO_POSE_RESULT_TYPE: &str = "robot.v1.NavigateToPoseResult";

const ACTION_PACKET_BYTES: usize = 16_376;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct ActionRequestRepr {
    payload: [u8; ACTION_PACKET_BYTES],
    len: u32,
    tag: u8,
    reserved: [u8; 3],
}

impl ShmPayload for ActionRequestRepr {
    const SCHEMA_ID: [u8; 16] = *b"rs-nav-act-req01";
    const SCHEMA_VERSION: u32 = 1;
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub struct ActionResponseRepr {
    payload: [u8; ACTION_PACKET_BYTES],
    len: u32,
    tag: u8,
    reserved: [u8; 3],
}

impl ShmPayload for ActionResponseRepr {
    // Goal acknowledgement adds a distinct response variant. Bump both the
    // fixed identity and schema version so an older bridge never consumes a
    // layout whose action lifecycle it cannot represent.
    const SCHEMA_ID: [u8; 16] = *b"rs-nav-act-rsp02";
    const SCHEMA_VERSION: u32 = 2;
}

/// Ordered request delivered from the Rust authority owner to the ROS bridge.
// Keeping the low-rate action value inline avoids another fallible heap step
// after its registered payload was already bounded and validated.
#[expect(
    clippy::large_enum_variant,
    reason = "fixed-capacity action payloads avoid allocation at the IPC boundary"
)]
#[derive(Clone, Debug, PartialEq)]
pub enum NavigationActionRequestKind {
    /// Submit one exact NavigateToPose goal intent.
    Goal(ActionGoalIntent),
    /// Request cancellation of the exact correlated goal.
    Cancel(ActionCancelRequest),
}

/// Validated, bounded request plus its allocation-free publish representation.
#[derive(Clone)]
pub struct NavigationActionRequest {
    kind: NavigationActionRequestKind,
    repr: ActionRequestRepr,
}

impl NavigationActionRequest {
    /// Validates the action/type identity and concrete goal before admission.
    pub fn goal(value: ActionGoalIntent) -> Result<Self, NavigationIpcError> {
        if value.action_name().as_str() != NAVIGATE_TO_POSE_ACTION
            || value.goal().protobuf_type().as_str() != NAVIGATE_TO_POSE_GOAL_TYPE
        {
            return Err(NavigationIpcError::ActionIdentity("goal"));
        }
        let _ = decode_navigate_to_pose_goal(value.goal().value())?;
        let bytes = encode_action_goal_intent(&value);
        Ok(Self {
            repr: request_repr(1, &bytes)?,
            kind: NavigationActionRequestKind::Goal(value),
        })
    }

    /// Admits a bounded cancellation request.
    pub fn cancel(value: ActionCancelRequest) -> Result<Self, NavigationIpcError> {
        let bytes = encode_action_cancel_request(&value);
        Ok(Self {
            repr: request_repr(2, &bytes)?,
            kind: NavigationActionRequestKind::Cancel(value),
        })
    }

    /// Returns the typed request.
    #[must_use]
    pub const fn kind(&self) -> &NavigationActionRequestKind {
        &self.kind
    }

    fn header(&self) -> Header {
        match &self.kind {
            NavigationActionRequestKind::Goal(value) => value.intent().header(),
            NavigationActionRequestKind::Cancel(value) => value.header,
        }
    }
}

impl std::fmt::Debug for NavigationActionRequest {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("NavigationActionRequest")
            .field("kind", &self.kind)
            .finish_non_exhaustive()
    }
}

impl PartialEq for NavigationActionRequest {
    fn eq(&self, other: &Self) -> bool {
        self.kind == other.kind
    }
}

/// Ordered response delivered from the ROS bridge to the Rust owner.
#[derive(Clone, Debug, PartialEq)]
pub enum NavigationActionResponseKind {
    /// Exact action-server goal admission decision; feedback follows only
    /// after an accepted acknowledgement.
    Acknowledgement(ActionAck),
    /// Nonterminal executing/canceling feedback.
    Feedback(ActionFeedback),
    /// Cancellation acceptance/rejection; not terminal evidence.
    Cancel(ActionCancelResponse),
    /// Exact terminal action result.
    Result(ActionResult),
}

/// Validated, bounded response plus its allocation-free publish representation.
#[derive(Clone)]
pub struct NavigationActionResponse {
    kind: NavigationActionResponseKind,
    repr: ActionResponseRepr,
}

impl NavigationActionResponse {
    /// Admits the exact goal acceptance or rejection decision.
    pub fn acknowledgement(value: ActionAck) -> Result<Self, NavigationIpcError> {
        let value = value.validate()?;
        let bytes = encode_action_ack(&value)?;
        Ok(Self {
            repr: response_repr(4, &bytes)?,
            kind: NavigationActionResponseKind::Acknowledgement(value),
        })
    }

    /// Validates status, payload identity and concrete navigation feedback.
    pub fn feedback(value: ActionFeedback) -> Result<Self, NavigationIpcError> {
        let value = value.validate()?;
        if value.feedback.protobuf_type().as_str() != NAVIGATE_TO_POSE_FEEDBACK_TYPE {
            return Err(NavigationIpcError::ActionIdentity("feedback"));
        }
        let _ = decode_navigate_to_pose_feedback(value.feedback.value())?;
        let bytes = encode_action_feedback(&value)?;
        Ok(Self {
            repr: response_repr(1, &bytes)?,
            kind: NavigationActionResponseKind::Feedback(value),
        })
    }

    /// Admits a bounded cancellation decision.
    pub fn cancel(value: ActionCancelResponse) -> Result<Self, NavigationIpcError> {
        let bytes = encode_action_cancel_response(&value);
        Ok(Self {
            repr: response_repr(2, &bytes)?,
            kind: NavigationActionResponseKind::Cancel(value),
        })
    }

    /// Validates terminal status, payload identity and concrete result.
    pub fn result(value: ActionResult) -> Result<Self, NavigationIpcError> {
        let value = value.validate()?;
        if value.result.protobuf_type().as_str() != NAVIGATE_TO_POSE_RESULT_TYPE {
            return Err(NavigationIpcError::ActionIdentity("result"));
        }
        let _ = decode_navigate_to_pose_result(value.result.value())?;
        let bytes = encode_action_result(&value)?;
        Ok(Self {
            repr: response_repr(3, &bytes)?,
            kind: NavigationActionResponseKind::Result(value),
        })
    }

    /// Returns the typed response.
    #[must_use]
    pub const fn kind(&self) -> &NavigationActionResponseKind {
        &self.kind
    }

    fn header(&self) -> Header {
        match &self.kind {
            NavigationActionResponseKind::Acknowledgement(value) => value.header,
            NavigationActionResponseKind::Feedback(value) => value.header,
            NavigationActionResponseKind::Cancel(value) => value.header,
            NavigationActionResponseKind::Result(value) => value.header,
        }
    }
}

impl std::fmt::Debug for NavigationActionResponse {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter
            .debug_struct("NavigationActionResponse")
            .field("kind", &self.kind)
            .finish_non_exhaustive()
    }
}

impl PartialEq for NavigationActionResponse {
    fn eq(&self, other: &Self) -> bool {
        self.kind == other.kind
    }
}

/// Fixed action-request codec with receive-time expiry validation.
#[derive(Clone, Copy)]
pub struct NavigationActionRequestCodec {
    now: NowFn,
}

impl NavigationActionRequestCodec {
    /// Uses the supplied local monotonic clock function.
    #[must_use]
    pub const fn new(now: NowFn) -> Self {
        Self { now }
    }
}

impl Default for NavigationActionRequestCodec {
    fn default() -> Self {
        Self::new(monotonic_now_ns)
    }
}

impl ShmCodec<NavigationActionRequest> for NavigationActionRequestCodec {
    type Repr = ActionRequestRepr;
    type Error = NavigationIpcError;

    fn encode(&mut self, value: &NavigationActionRequest) -> Self::Repr {
        value.repr
    }

    fn decode(&mut self, repr: Self::Repr) -> Result<NavigationActionRequest, Self::Error> {
        let bytes = packet_bytes(repr.len, repr.reserved, &repr.payload)?;
        let mut value = match repr.tag {
            1 => NavigationActionRequest::goal(decode_action_goal_intent(bytes)?)?,
            2 => NavigationActionRequest::cancel(decode_action_cancel_request(bytes)?)?,
            _ => {
                return Err(NavigationIpcError::InvalidRepresentation(
                    "action request tag",
                ));
            }
        };
        let _ = validate_header_at(value.header(), (self.now)())?;
        value.repr = repr;
        Ok(value)
    }
}

/// Fixed action-response codec with receive-time expiry validation.
#[derive(Clone, Copy)]
pub struct NavigationActionResponseCodec {
    now: NowFn,
}

impl NavigationActionResponseCodec {
    /// Uses the supplied local monotonic clock function.
    #[must_use]
    pub const fn new(now: NowFn) -> Self {
        Self { now }
    }
}

impl Default for NavigationActionResponseCodec {
    fn default() -> Self {
        Self::new(monotonic_now_ns)
    }
}

impl ShmCodec<NavigationActionResponse> for NavigationActionResponseCodec {
    type Repr = ActionResponseRepr;
    type Error = NavigationIpcError;

    fn encode(&mut self, value: &NavigationActionResponse) -> Self::Repr {
        value.repr
    }

    fn decode(&mut self, repr: Self::Repr) -> Result<NavigationActionResponse, Self::Error> {
        let bytes = packet_bytes(repr.len, repr.reserved, &repr.payload)?;
        let mut value = match repr.tag {
            1 => NavigationActionResponse::feedback(decode_action_feedback(bytes)?)?,
            2 => NavigationActionResponse::cancel(decode_action_cancel_response(bytes)?)?,
            3 => NavigationActionResponse::result(decode_action_result(bytes)?)?,
            4 => NavigationActionResponse::acknowledgement(decode_action_ack(bytes)?)?,
            _ => {
                return Err(NavigationIpcError::InvalidRepresentation(
                    "action response tag",
                ));
            }
        };
        let _ = validate_header_at(value.header(), (self.now)())?;
        value.repr = repr;
        Ok(value)
    }
}

fn request_repr(tag: u8, bytes: &[u8]) -> Result<ActionRequestRepr, NavigationIpcError> {
    let mut payload = [0; ACTION_PACKET_BYTES];
    copy_packet(bytes, &mut payload)?;
    Ok(ActionRequestRepr {
        payload,
        len: bytes.len() as u32,
        tag,
        reserved: [0; 3],
    })
}

fn response_repr(tag: u8, bytes: &[u8]) -> Result<ActionResponseRepr, NavigationIpcError> {
    let mut payload = [0; ACTION_PACKET_BYTES];
    copy_packet(bytes, &mut payload)?;
    Ok(ActionResponseRepr {
        payload,
        len: bytes.len() as u32,
        tag,
        reserved: [0; 3],
    })
}

fn copy_packet(bytes: &[u8], target: &mut [u8]) -> Result<(), NavigationIpcError> {
    if bytes.len() > target.len() {
        return Err(NavigationIpcError::ActionPacketTooLarge {
            actual: bytes.len(),
            maximum: target.len(),
        });
    }
    target[..bytes.len()].copy_from_slice(bytes);
    Ok(())
}

fn packet_bytes(
    len: u32,
    reserved: [u8; 3],
    payload: &[u8; ACTION_PACKET_BYTES],
) -> Result<&[u8], NavigationIpcError> {
    let len = usize::try_from(len).map_err(|_| {
        NavigationIpcError::InvalidRepresentation("action packet length conversion")
    })?;
    if reserved != [0; 3] || len > payload.len() || payload[len..].iter().any(|byte| *byte != 0) {
        return Err(NavigationIpcError::InvalidRepresentation(
            "action packet length or padding",
        ));
    }
    Ok(&payload[..len])
}
