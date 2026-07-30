use crate::codec::DamiaoEscId;
use hardware_module_core::{ClassicCanFrame, MonoTime, StandardCanId};
use thiserror::Error;

/// Standard arbitration identifier for all Damiao register requests.
pub const REGISTER_REQUEST_ID: u16 = 0x07ff;
/// Duration of one motor communication-timeout tick.
pub const TIMEOUT_TICK_NANOS: u64 = 50_000;

const READ_OPCODE: u8 = 0x33;
const WRITE_OPCODE: u8 = 0x55;
const STORE_OPCODE: u8 = 0xaa;
const STORE_REGISTER: u8 = 0x01;

/// Registers required by the read-only arming handshake.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
#[repr(u8)]
pub enum Register {
    /// Feedback/reply arbitration ID.
    MstId = 0x07,
    /// Command arbitration ID.
    EscId = 0x08,
    /// Enabled communication timeout in 50-microsecond ticks.
    Timeout = 0x09,
    /// Control mode; current MIT mode is `1`.
    ControlMode = 0x0a,
    /// Symmetric position scaling limit.
    PositionMaximum = 0x15,
    /// Symmetric velocity scaling limit.
    VelocityMaximum = 0x16,
    /// Symmetric torque scaling limit.
    TorqueMaximum = 0x17,
}

impl Register {
    /// Returns the on-wire register address.
    #[must_use]
    pub const fn address(self) -> u8 {
        self as u8
    }
}

/// Damiao register operation opcode.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum RegisterOperation {
    /// Read a register without changing motor configuration.
    Read,
    /// Write one runtime RAM value; maintenance-only.
    Write,
    /// Store all parameters to flash; destructive maintenance-only.
    Store,
}

impl RegisterOperation {
    const fn opcode(self) -> u8 {
        match self {
            Self::Read => READ_OPCODE,
            Self::Write => WRITE_OPCODE,
            Self::Store => STORE_OPCODE,
        }
    }
}

/// Correlated eight-byte register response.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RegisterReply {
    /// Reply arbitration identifier.
    pub source_id: StandardCanId,
    /// Matched operation.
    pub operation: RegisterOperation,
    /// Raw register address (`0x01` for store acknowledgement).
    pub register_address: u8,
    /// Raw little-endian 32-bit payload bytes.
    pub raw_value: [u8; 4],
}

impl RegisterReply {
    /// Interprets the register data as little-endian `u32`.
    #[must_use]
    pub const fn value_u32(self) -> u32 {
        u32::from_le_bytes(self.raw_value)
    }

    /// Interprets the register data as little-endian IEEE-754 `f32`.
    #[must_use]
    pub fn value_f32(self) -> f32 {
        f32::from_le_bytes(self.raw_value)
    }
}

/// Register frame or timeout conversion failure.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum RegisterCodecError {
    /// Register reply payload is not exactly eight bytes.
    #[error("Damiao register reply DLC {actual} is not 8")]
    WrongDlc {
        /// Received DLC.
        actual: u8,
    },
    /// Reply arbitration ID does not match the expected MST ID.
    #[error("Damiao register reply ID {actual:#x} does not match {expected:#x}")]
    WrongReplyId {
        /// Expected ID.
        expected: u16,
        /// Received ID.
        actual: u16,
    },
    /// Embedded target does not match the request ESC ID.
    #[error("Damiao register target {actual} does not match ESC_ID {expected}")]
    WrongTarget {
        /// Expected ESC ID.
        expected: u16,
        /// Received embedded target.
        actual: u16,
    },
    /// Reply opcode does not match the outstanding operation.
    #[error("Damiao register opcode {actual:#x} does not match {expected:#x}")]
    WrongOpcode {
        /// Expected opcode.
        expected: u8,
        /// Received opcode.
        actual: u8,
    },
    /// Reply register does not match the outstanding request.
    #[error("Damiao register address {actual:#x} does not match {expected:#x}")]
    WrongRegister {
        /// Expected register.
        expected: u8,
        /// Received register.
        actual: u8,
    },
    /// Reply arrived after the bounded deadline.
    #[error("Damiao register reply expired at {deadline_ns} ns, received at {received_ns} ns")]
    Expired {
        /// Transaction deadline.
        deadline_ns: u64,
        /// Local monotonic receive time.
        received_ns: u64,
    },
    /// Deadline addition overflowed the monotonic representation.
    #[error("Damiao register deadline overflowed")]
    DeadlineOverflow,
    /// Timeout duration is zero or not an exact positive 50-microsecond tick count.
    #[error("Damiao timeout duration {nanos} ns is not an exact positive 50 us tick count")]
    InvalidTimeoutDuration {
        /// Rejected duration.
        nanos: u64,
    },
    /// Timeout duration requires more than `u32::MAX` ticks.
    #[error("Damiao timeout duration {nanos} ns exceeds the register range")]
    TimeoutOverflow {
        /// Rejected duration.
        nanos: u64,
    },
    /// Fixed register frame construction unexpectedly failed.
    #[error("internal Damiao register frame construction failed")]
    FrameConstruction,
}

fn register_request_id() -> StandardCanId {
    match StandardCanId::new(REGISTER_REQUEST_ID) {
        Ok(id) => id,
        Err(_) => unreachable!("0x7ff is a valid standard CAN ID"),
    }
}

fn register_frame(
    esc_id: DamiaoEscId,
    operation: RegisterOperation,
    register_address: u8,
    raw_value: [u8; 4],
) -> Result<ClassicCanFrame, RegisterCodecError> {
    let target = u16::from(esc_id.as_raw()).to_le_bytes();
    ClassicCanFrame::new(
        register_request_id(),
        &[
            target[0],
            target[1],
            operation.opcode(),
            register_address,
            raw_value[0],
            raw_value[1],
            raw_value[2],
            raw_value[3],
        ],
    )
    .map_err(|_| RegisterCodecError::FrameConstruction)
}

/// Builds one deterministic DLC-8 read request on standard ID `0x7ff`.
pub fn register_read_frame(
    esc_id: DamiaoEscId,
    register: Register,
) -> Result<ClassicCanFrame, RegisterCodecError> {
    register_frame(esc_id, RegisterOperation::Read, register.address(), [0; 4])
}

/// Builds a maintenance-only runtime `u32` register write.
pub fn register_write_u32_frame(
    esc_id: DamiaoEscId,
    register: Register,
    value: u32,
) -> Result<ClassicCanFrame, RegisterCodecError> {
    register_frame(
        esc_id,
        RegisterOperation::Write,
        register.address(),
        value.to_le_bytes(),
    )
}

/// Builds a maintenance-only runtime IEEE-754 `f32` register write.
pub fn register_write_f32_frame(
    esc_id: DamiaoEscId,
    register: Register,
    value: f32,
) -> Result<ClassicCanFrame, RegisterCodecError> {
    register_frame(
        esc_id,
        RegisterOperation::Write,
        register.address(),
        value.to_le_bytes(),
    )
}

/// Builds the maintenance-only flash-store request.
pub fn register_store_frame(esc_id: DamiaoEscId) -> Result<ClassicCanFrame, RegisterCodecError> {
    register_frame(esc_id, RegisterOperation::Store, STORE_REGISTER, [0; 4])
}

/// Converts the motor's 50-microsecond timeout ticks to nanoseconds.
#[must_use]
pub const fn timeout_ticks_to_nanos(ticks: u32) -> u64 {
    (ticks as u64) * TIMEOUT_TICK_NANOS
}

/// Converts a positive exact nanosecond duration to 50-microsecond ticks.
pub fn timeout_nanos_to_ticks_exact(nanos: u64) -> Result<u32, RegisterCodecError> {
    if nanos == 0 || !nanos.is_multiple_of(TIMEOUT_TICK_NANOS) {
        return Err(RegisterCodecError::InvalidTimeoutDuration { nanos });
    }
    let ticks = nanos / TIMEOUT_TICK_NANOS;
    u32::try_from(ticks).map_err(|_| RegisterCodecError::TimeoutOverflow { nanos })
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct PendingRegister {
    esc_id: DamiaoEscId,
    expected_reply_id: Option<StandardCanId>,
    operation: RegisterOperation,
    register_address: u8,
    deadline: MonoTime,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct RegisterRequest {
    operation: RegisterOperation,
    register_address: u8,
    raw_value: [u8; 4],
}

/// One serialized register transaction slot.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct RegisterTransaction {
    pending: Option<PendingRegister>,
}

/// Serialized register transaction state/correlation failure.
#[derive(Clone, Copy, Debug, Eq, Error, PartialEq)]
pub enum RegisterTransactionError {
    /// A second transaction was attempted while one remains pending.
    #[error("a Damiao register transaction is already pending")]
    Busy,
    /// A reply was supplied without an outstanding transaction.
    #[error("no Damiao register transaction is pending")]
    NoPending,
    /// Frame/deadline validation failed.
    #[error(transparent)]
    Codec(#[from] RegisterCodecError),
}

impl RegisterTransaction {
    /// Creates an empty transaction slot.
    #[must_use]
    pub const fn new() -> Self {
        Self { pending: None }
    }

    /// Returns whether a request is awaiting its correlated reply.
    #[must_use]
    pub const fn is_pending(self) -> bool {
        self.pending.is_some()
    }

    /// Returns the pending deadline, if any.
    #[must_use]
    pub const fn deadline(self) -> Option<MonoTime> {
        match self.pending {
            Some(pending) => Some(pending.deadline),
            None => None,
        }
    }

    /// Cancels any pending transaction without producing traffic.
    pub fn clear(&mut self) {
        self.pending = None;
    }

    fn begin(
        &mut self,
        esc_id: DamiaoEscId,
        expected_reply_id: Option<StandardCanId>,
        request: RegisterRequest,
        now: MonoTime,
        timeout_ns: u64,
    ) -> Result<ClassicCanFrame, RegisterTransactionError> {
        if self.pending.is_some() {
            return Err(RegisterTransactionError::Busy);
        }
        let deadline_ns = now
            .as_nanos()
            .checked_add(timeout_ns)
            .ok_or(RegisterCodecError::DeadlineOverflow)?;
        let frame = register_frame(
            esc_id,
            request.operation,
            request.register_address,
            request.raw_value,
        )?;
        self.pending = Some(PendingRegister {
            esc_id,
            expected_reply_id,
            operation: request.operation,
            register_address: request.register_address,
            deadline: MonoTime::from_nanos(deadline_ns),
        });
        Ok(frame)
    }

    /// Starts one bounded read transaction.
    pub fn begin_read(
        &mut self,
        esc_id: DamiaoEscId,
        register: Register,
        expected_reply_id: Option<StandardCanId>,
        now: MonoTime,
        timeout_ns: u64,
    ) -> Result<ClassicCanFrame, RegisterTransactionError> {
        self.begin(
            esc_id,
            expected_reply_id,
            RegisterRequest {
                operation: RegisterOperation::Read,
                register_address: register.address(),
                raw_value: [0; 4],
            },
            now,
            timeout_ns,
        )
    }

    /// Starts one bounded maintenance-only runtime write transaction.
    pub fn begin_write_u32(
        &mut self,
        esc_id: DamiaoEscId,
        register: Register,
        value: u32,
        expected_reply_id: StandardCanId,
        now: MonoTime,
        timeout_ns: u64,
    ) -> Result<ClassicCanFrame, RegisterTransactionError> {
        self.begin(
            esc_id,
            Some(expected_reply_id),
            RegisterRequest {
                operation: RegisterOperation::Write,
                register_address: register.address(),
                raw_value: value.to_le_bytes(),
            },
            now,
            timeout_ns,
        )
    }

    /// Starts one bounded maintenance-only flash-store transaction.
    pub fn begin_store(
        &mut self,
        esc_id: DamiaoEscId,
        expected_reply_id: StandardCanId,
        now: MonoTime,
        timeout_ns: u64,
    ) -> Result<ClassicCanFrame, RegisterTransactionError> {
        self.begin(
            esc_id,
            Some(expected_reply_id),
            RegisterRequest {
                operation: RegisterOperation::Store,
                register_address: STORE_REGISTER,
                raw_value: [0; 4],
            },
            now,
            timeout_ns,
        )
    }

    /// Correlates one reply by ID, target, opcode, register and deadline.
    pub fn accept_reply(
        &mut self,
        frame: ClassicCanFrame,
        received_at: MonoTime,
    ) -> Result<RegisterReply, RegisterTransactionError> {
        let pending = self.pending.ok_or(RegisterTransactionError::NoPending)?;
        let reply = decode_register_reply(frame, received_at, pending)?;
        self.pending = None;
        Ok(reply)
    }
}

fn decode_register_reply(
    frame: ClassicCanFrame,
    received_at: MonoTime,
    pending: PendingRegister,
) -> Result<RegisterReply, RegisterCodecError> {
    if received_at > pending.deadline {
        return Err(RegisterCodecError::Expired {
            deadline_ns: pending.deadline.as_nanos(),
            received_ns: received_at.as_nanos(),
        });
    }
    if let Some(expected) = pending.expected_reply_id
        && frame.id() != expected
    {
        return Err(RegisterCodecError::WrongReplyId {
            expected: expected.as_raw(),
            actual: frame.id().as_raw(),
        });
    }
    if frame.dlc() != 8 {
        return Err(RegisterCodecError::WrongDlc {
            actual: frame.dlc(),
        });
    }
    let data = frame.data();
    let target = u16::from_le_bytes([data[0], data[1]]);
    let expected_target = u16::from(pending.esc_id.as_raw());
    if target != expected_target {
        return Err(RegisterCodecError::WrongTarget {
            expected: expected_target,
            actual: target,
        });
    }
    let expected_opcode = pending.operation.opcode();
    if data[2] != expected_opcode {
        return Err(RegisterCodecError::WrongOpcode {
            expected: expected_opcode,
            actual: data[2],
        });
    }
    if data[3] != pending.register_address {
        return Err(RegisterCodecError::WrongRegister {
            expected: pending.register_address,
            actual: data[3],
        });
    }
    Ok(RegisterReply {
        source_id: frame.id(),
        operation: pending.operation,
        register_address: pending.register_address,
        raw_value: [data[4], data[5], data[6], data[7]],
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn esc() -> DamiaoEscId {
        DamiaoEscId::new(1).unwrap()
    }

    fn mst() -> StandardCanId {
        StandardCanId::new(0x11).unwrap()
    }

    fn reply(
        source: StandardCanId,
        target: u16,
        opcode: u8,
        register: u8,
        value: [u8; 4],
    ) -> ClassicCanFrame {
        let target = target.to_le_bytes();
        ClassicCanFrame::new(
            source,
            &[
                target[0], target[1], opcode, register, value[0], value[1], value[2], value[3],
            ],
        )
        .unwrap()
    }

    #[test]
    fn register_read_write_and_store_bytes_are_exact_little_endian() {
        assert_eq!(
            register_read_frame(esc(), Register::MstId).unwrap().data(),
            [0x01, 0x00, 0x33, 0x07, 0, 0, 0, 0]
        );
        assert_eq!(
            register_write_u32_frame(esc(), Register::Timeout, 0x1234_5678)
                .unwrap()
                .data(),
            [0x01, 0x00, 0x55, 0x09, 0x78, 0x56, 0x34, 0x12]
        );
        assert_eq!(
            register_write_f32_frame(esc(), Register::PositionMaximum, 12.5)
                .unwrap()
                .data(),
            [0x01, 0x00, 0x55, 0x15, 0x00, 0x00, 0x48, 0x41]
        );
        assert_eq!(
            register_store_frame(esc()).unwrap().data(),
            [0x01, 0x00, 0xaa, 0x01, 0, 0, 0, 0]
        );
    }

    #[test]
    fn register_transaction_serializes_and_correlates_read() {
        let mut transaction = RegisterTransaction::new();
        let request = transaction
            .begin_read(esc(), Register::MstId, None, MonoTime::from_nanos(10), 20)
            .unwrap();
        assert_eq!(request.id().as_raw(), REGISTER_REQUEST_ID);
        assert!(matches!(
            transaction.begin_read(esc(), Register::EscId, None, MonoTime::from_nanos(10), 20),
            Err(RegisterTransactionError::Busy)
        ));
        let reply = transaction
            .accept_reply(
                reply(
                    mst(),
                    1,
                    READ_OPCODE,
                    Register::MstId.address(),
                    0x11_u32.to_le_bytes(),
                ),
                MonoTime::from_nanos(30),
            )
            .unwrap();
        assert_eq!(reply.source_id, mst());
        assert_eq!(reply.value_u32(), 0x11);
        assert!(!transaction.is_pending());
    }

    #[test]
    fn register_transaction_rejects_each_correlation_dimension_and_deadline() {
        let cases = [
            (
                reply(
                    StandardCanId::new(0x12).unwrap(),
                    1,
                    READ_OPCODE,
                    Register::EscId.address(),
                    [0; 4],
                ),
                MonoTime::from_nanos(11),
            ),
            (
                reply(mst(), 2, READ_OPCODE, Register::EscId.address(), [0; 4]),
                MonoTime::from_nanos(11),
            ),
            (
                reply(mst(), 1, WRITE_OPCODE, Register::EscId.address(), [0; 4]),
                MonoTime::from_nanos(11),
            ),
            (
                reply(mst(), 1, READ_OPCODE, Register::Timeout.address(), [0; 4]),
                MonoTime::from_nanos(11),
            ),
            (
                reply(mst(), 1, READ_OPCODE, Register::EscId.address(), [0; 4]),
                MonoTime::from_nanos(31),
            ),
        ];
        for (frame, received_at) in cases {
            let mut transaction = RegisterTransaction::new();
            transaction
                .begin_read(
                    esc(),
                    Register::EscId,
                    Some(mst()),
                    MonoTime::from_nanos(10),
                    20,
                )
                .unwrap();
            assert!(transaction.accept_reply(frame, received_at).is_err());
        }

        let short = ClassicCanFrame::new(mst(), &[1, 0, READ_OPCODE, 8]).unwrap();
        let mut transaction = RegisterTransaction::new();
        transaction
            .begin_read(
                esc(),
                Register::EscId,
                Some(mst()),
                MonoTime::from_nanos(10),
                20,
            )
            .unwrap();
        assert!(
            transaction
                .accept_reply(short, MonoTime::from_nanos(11))
                .is_err()
        );
    }

    #[test]
    fn register_write_and_store_transactions_match_their_own_opcodes() {
        let mut write = RegisterTransaction::new();
        write
            .begin_write_u32(
                esc(),
                Register::Timeout,
                20,
                mst(),
                MonoTime::from_nanos(0),
                10,
            )
            .unwrap();
        assert!(
            write
                .accept_reply(
                    reply(
                        mst(),
                        1,
                        WRITE_OPCODE,
                        Register::Timeout.address(),
                        20_u32.to_le_bytes()
                    ),
                    MonoTime::from_nanos(1)
                )
                .is_ok()
        );

        let mut store = RegisterTransaction::new();
        store
            .begin_store(esc(), mst(), MonoTime::from_nanos(0), 10)
            .unwrap();
        assert!(
            store
                .accept_reply(
                    reply(mst(), 1, STORE_OPCODE, STORE_REGISTER, [0; 4]),
                    MonoTime::from_nanos(1)
                )
                .is_ok()
        );
    }

    #[test]
    fn register_timeout_conversion_is_exact_and_checked() {
        assert_eq!(timeout_ticks_to_nanos(2), 100_000);
        assert_eq!(timeout_nanos_to_ticks_exact(100_000), Ok(2));
        assert!(matches!(
            timeout_nanos_to_ticks_exact(1),
            Err(RegisterCodecError::InvalidTimeoutDuration { .. })
        ));
        let overflow = (u64::from(u32::MAX) + 1) * TIMEOUT_TICK_NANOS;
        assert!(matches!(
            timeout_nanos_to_ticks_exact(overflow),
            Err(RegisterCodecError::TimeoutOverflow { .. })
        ));
    }
}
