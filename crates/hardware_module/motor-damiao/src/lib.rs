//! Safe host-side contracts for a current-firmware Damiao MIT-mode motor.
//!
//! This crate implements classic-CAN wire facts, read-only register discovery,
//! bus identity validation, and a bounded lifecycle state machine. It performs
//! no I/O and has no dependency on Linux SocketCAN or robot configuration.
//! Physical compatibility remains unqualified until separately approved target
//! tests confirm the installed firmware and restrained-motor behavior.

mod codec;
mod driver;
mod register;

pub use codec::{
    CLEAR_ERROR_DATA, DISABLE_DATA, DamiaoCodecError, DamiaoEscId, DamiaoFeedback, DamiaoStatus,
    ENABLE_DATA, MitCommand, RuntimeLimits, ZERO_POSITION_DATA, clear_error_frame, decode_feedback,
    disable_frame, enable_frame, encode_mit_command,
};
pub use driver::{
    DamiaoBusDevice, DamiaoBusSetError, DamiaoDriver, DamiaoFault, DamiaoSafetyPolicy,
    SafetyPolicyError, VerifiedConfiguration, validate_bus_set,
};
pub use register::{
    REGISTER_REQUEST_ID, Register, RegisterCodecError, RegisterOperation, RegisterReply,
    RegisterTransaction, RegisterTransactionError, register_read_frame,
};

/// Explicitly disarmed, potentially destructive codec operations.
///
/// Nothing in the ordinary driver calls these functions. A future maintenance
/// executable must add its own authorization, physical-safety, and persistence
/// policy before transmitting any returned frame.
pub mod maintenance {
    pub use crate::codec::zero_position_frame;
    pub use crate::register::{
        register_store_frame, register_write_f32_frame, register_write_u32_frame,
        timeout_nanos_to_ticks_exact, timeout_ticks_to_nanos,
    };
}
