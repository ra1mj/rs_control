use hardware_module_core::{ClassicCanFrame, MonoTime, StandardCanId};
use thiserror::Error;

const KP_MAX: f32 = 500.0;
const KD_MAX: f32 = 5.0;

/// Exact MIT-mode enable payload.
pub const ENABLE_DATA: [u8; 8] = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc];
/// Exact MIT-mode disable payload.
pub const DISABLE_DATA: [u8; 8] = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd];
/// Exact MIT-mode save-current-position-as-zero payload.
pub const ZERO_POSITION_DATA: [u8; 8] = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe];
/// Exact MIT-mode clear-error payload.
pub const CLEAR_ERROR_DATA: [u8; 8] = [0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfb];

/// Framework-safe Damiao command identifier.
///
/// Current feedback embeds only the low nibble, so this boundary deliberately
/// accepts `1..=15` even though the vendor-neutral CAN type accepts all SFF IDs.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct DamiaoEscId(u8);

impl DamiaoEscId {
    /// Validates the safe low-nibble motor identifier policy.
    pub const fn new(raw: u16) -> Result<Self, DamiaoCodecError> {
        if raw >= 1 && raw <= 15 {
            Ok(Self(raw as u8))
        } else {
            Err(DamiaoCodecError::InvalidEscId { raw })
        }
    }

    /// Returns the raw configured ESC identifier.
    #[must_use]
    pub const fn as_raw(self) -> u8 {
        self.0
    }

    /// Returns this ID as a validated standard CAN identifier.
    #[must_use]
    pub fn standard_id(self) -> StandardCanId {
        match StandardCanId::new(u16::from(self.0)) {
            Ok(id) => id,
            Err(_) => unreachable!("DamiaoEscId is always a valid standard CAN ID"),
        }
    }
}

impl TryFrom<u16> for DamiaoEscId {
    type Error = DamiaoCodecError;

    fn try_from(value: u16) -> Result<Self, Self::Error> {
        Self::new(value)
    }
}

/// Runtime scaling identity read from the motor before arming.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RuntimeLimits {
    position: f32,
    velocity: f32,
    torque: f32,
}

impl RuntimeLimits {
    /// Validates finite, positive symmetric physical limits.
    pub fn new(position: f32, velocity: f32, torque: f32) -> Result<Self, DamiaoCodecError> {
        validate_positive_limit("PMAX", position)?;
        validate_positive_limit("VMAX", velocity)?;
        validate_positive_limit("TMAX", torque)?;
        Ok(Self {
            position,
            velocity,
            torque,
        })
    }

    /// Symmetric position magnitude in radians.
    #[must_use]
    pub const fn position(self) -> f32 {
        self.position
    }

    /// Symmetric velocity magnitude in radians per second.
    #[must_use]
    pub const fn velocity(self) -> f32 {
        self.velocity
    }

    /// Symmetric feed-forward torque magnitude in newton-metres.
    #[must_use]
    pub const fn torque(self) -> f32 {
        self.torque
    }
}

fn validate_positive_limit(name: &'static str, value: f32) -> Result<(), DamiaoCodecError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(DamiaoCodecError::InvalidLimit { name, value });
    }
    Ok(())
}

/// Physical MIT command values.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MitCommand {
    /// Desired position in radians.
    pub position: f32,
    /// Desired velocity in radians per second.
    pub velocity: f32,
    /// Position gain in `[0, 500]`.
    pub kp: f32,
    /// Velocity gain in `[0, 5]`.
    pub kd: f32,
    /// Feed-forward torque in newton-metres.
    pub feed_forward_torque: f32,
}

/// Decoded motor status nibble.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub enum DamiaoStatus {
    /// Motor output is disabled.
    Disabled,
    /// Motor output is enabled.
    Enabled,
    /// Output-shaft calibration fault.
    OutputShaftCalibrationFault,
    /// Sensor output fault.
    SensorOutputFault,
    /// Motor encoder calibration fault.
    MotorEncoderCalibrationFault,
    /// DC bus over-voltage.
    OverVoltage,
    /// DC bus under-voltage.
    UnderVoltage,
    /// Phase/output over-current.
    OverCurrent,
    /// MOSFET over-temperature.
    MosOverTemperature,
    /// Rotor/winding over-temperature.
    WindingOverTemperature,
    /// Communication timeout/loss.
    CommunicationLost,
    /// Mechanical/electrical overload.
    Overload,
    /// Unspecified status; callers must fault closed.
    Unknown(u8),
}

impl DamiaoStatus {
    /// Decodes and preserves every possible status nibble.
    #[must_use]
    pub const fn from_raw(raw: u8) -> Self {
        match raw & 0x0f {
            0x0 => Self::Disabled,
            0x1 => Self::Enabled,
            0x3 => Self::OutputShaftCalibrationFault,
            0x4 => Self::SensorOutputFault,
            0x5 => Self::MotorEncoderCalibrationFault,
            0x8 => Self::OverVoltage,
            0x9 => Self::UnderVoltage,
            0xa => Self::OverCurrent,
            0xb => Self::MosOverTemperature,
            0xc => Self::WindingOverTemperature,
            0xd => Self::CommunicationLost,
            0xe => Self::Overload,
            other => Self::Unknown(other),
        }
    }

    /// Returns the original status nibble.
    #[must_use]
    pub const fn as_raw(self) -> u8 {
        match self {
            Self::Disabled => 0x0,
            Self::Enabled => 0x1,
            Self::OutputShaftCalibrationFault => 0x3,
            Self::SensorOutputFault => 0x4,
            Self::MotorEncoderCalibrationFault => 0x5,
            Self::OverVoltage => 0x8,
            Self::UnderVoltage => 0x9,
            Self::OverCurrent => 0xa,
            Self::MosOverTemperature => 0xb,
            Self::WindingOverTemperature => 0xc,
            Self::CommunicationLost => 0xd,
            Self::Overload => 0xe,
            Self::Unknown(raw) => raw & 0x0f,
        }
    }

    /// Returns whether this status must fault the driver closed.
    #[must_use]
    pub const fn is_fault(self) -> bool {
        !matches!(self, Self::Disabled | Self::Enabled)
    }
}

/// One validated, scaled motor feedback sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DamiaoFeedback {
    /// Feedback arbitration identifier (`MST_ID`).
    pub source_id: StandardCanId,
    /// Embedded configured motor identity.
    pub esc_id: DamiaoEscId,
    /// Decoded status, including unknown values.
    pub status: DamiaoStatus,
    /// Position in radians.
    pub position: f32,
    /// Velocity in radians per second.
    pub velocity: f32,
    /// Estimated output torque in newton-metres.
    pub torque: f32,
    /// Raw MOS temperature byte; signedness/sentinels remain unqualified.
    pub mos_temperature_raw: u8,
    /// Raw rotor/winding temperature byte.
    pub winding_temperature_raw: u8,
    /// Local monotonic observation time.
    pub observed_monotonic: MonoTime,
    /// Monotonic per-driver sample sequence.
    pub sequence: u64,
}

/// Typed Damiao wire/physical-value validation failure.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum DamiaoCodecError {
    /// ESC ID violates the safe low-nibble policy.
    #[error("Damiao ESC_ID {raw} is outside 1..=15")]
    InvalidEscId {
        /// Rejected identifier.
        raw: u16,
    },
    /// Runtime scaling limit is not finite and positive.
    #[error("Damiao runtime limit {name}={value} must be finite and positive")]
    InvalidLimit {
        /// Register/limit name.
        name: &'static str,
        /// Rejected value.
        value: f32,
    },
    /// Command field is non-finite or outside its selected physical range.
    #[error("Damiao command field {field}={value} is outside [{minimum}, {maximum}]")]
    CommandOutOfRange {
        /// Physical field name.
        field: &'static str,
        /// Rejected value.
        value: f32,
        /// Inclusive lower limit.
        minimum: f32,
        /// Inclusive upper limit.
        maximum: f32,
    },
    /// Feedback frame has the wrong arbitration ID.
    #[error("Damiao feedback ID {actual:#x} does not match MST_ID {expected:#x}")]
    WrongFeedbackId {
        /// Required identifier.
        expected: u16,
        /// Received identifier.
        actual: u16,
    },
    /// Feedback payload is not exactly eight bytes.
    #[error("Damiao feedback DLC {actual} is not 8")]
    WrongFeedbackDlc {
        /// Received DLC.
        actual: u8,
    },
    /// Embedded feedback motor identity does not match the configured ESC ID.
    #[error("Damiao feedback embedded ESC_ID {actual} does not match {expected}")]
    WrongEmbeddedEscId {
        /// Required low-nibble identity.
        expected: u8,
        /// Received low-nibble identity.
        actual: u8,
    },
    /// A validated project frame unexpectedly could not be constructed.
    #[error("internal classic-CAN frame construction failed")]
    FrameConstruction,
}

fn encode_unsigned(
    field: &'static str,
    value: f32,
    minimum: f32,
    maximum: f32,
    bits: u8,
) -> Result<u16, DamiaoCodecError> {
    if !value.is_finite() || value < minimum || value > maximum {
        return Err(DamiaoCodecError::CommandOutOfRange {
            field,
            value,
            minimum,
            maximum,
        });
    }
    let integer_max = (1_u32 << bits) - 1;
    let scaled = ((value - minimum) / (maximum - minimum)) * integer_max as f32;
    Ok(scaled.trunc() as u16)
}

fn decode_unsigned(raw: u16, minimum: f32, maximum: f32, bits: u8) -> f32 {
    let integer_max = (1_u32 << bits) - 1;
    minimum + (f32::from(raw) / integer_max as f32) * (maximum - minimum)
}

/// Encodes one exact eight-byte current-firmware MIT command.
pub fn encode_mit_command(
    esc_id: DamiaoEscId,
    limits: RuntimeLimits,
    command: MitCommand,
) -> Result<ClassicCanFrame, DamiaoCodecError> {
    let position = encode_unsigned(
        "position",
        command.position,
        -limits.position,
        limits.position,
        16,
    )?;
    let velocity = encode_unsigned(
        "velocity",
        command.velocity,
        -limits.velocity,
        limits.velocity,
        12,
    )?;
    let kp = encode_unsigned("kp", command.kp, 0.0, KP_MAX, 12)?;
    let kd = encode_unsigned("kd", command.kd, 0.0, KD_MAX, 12)?;
    let torque = encode_unsigned(
        "feed_forward_torque",
        command.feed_forward_torque,
        -limits.torque,
        limits.torque,
        12,
    )?;

    let data = [
        (position >> 8) as u8,
        position as u8,
        (velocity >> 4) as u8,
        ((velocity & 0x0f) << 4) as u8 | ((kp >> 8) & 0x0f) as u8,
        kp as u8,
        (kd >> 4) as u8,
        ((kd & 0x0f) << 4) as u8 | ((torque >> 8) & 0x0f) as u8,
        torque as u8,
    ];
    ClassicCanFrame::new(esc_id.standard_id(), &data)
        .map_err(|_| DamiaoCodecError::FrameConstruction)
}

/// Decodes one exact feedback data frame after routing/correlation selects it.
pub fn decode_feedback(
    frame: ClassicCanFrame,
    mst_id: StandardCanId,
    esc_id: DamiaoEscId,
    limits: RuntimeLimits,
    observed_monotonic: MonoTime,
    sequence: u64,
) -> Result<DamiaoFeedback, DamiaoCodecError> {
    if frame.id() != mst_id {
        return Err(DamiaoCodecError::WrongFeedbackId {
            expected: mst_id.as_raw(),
            actual: frame.id().as_raw(),
        });
    }
    if frame.dlc() != 8 {
        return Err(DamiaoCodecError::WrongFeedbackDlc {
            actual: frame.dlc(),
        });
    }
    let data = frame.data();
    let embedded_id = data[0] & 0x0f;
    if embedded_id != esc_id.as_raw() {
        return Err(DamiaoCodecError::WrongEmbeddedEscId {
            expected: esc_id.as_raw(),
            actual: embedded_id,
        });
    }
    let position_raw = u16::from_be_bytes([data[1], data[2]]);
    let velocity_raw = (u16::from(data[3]) << 4) | u16::from(data[4] >> 4);
    let torque_raw = (u16::from(data[4] & 0x0f) << 8) | u16::from(data[5]);
    Ok(DamiaoFeedback {
        source_id: mst_id,
        esc_id,
        status: DamiaoStatus::from_raw(data[0] >> 4),
        position: decode_unsigned(position_raw, -limits.position, limits.position, 16),
        velocity: decode_unsigned(velocity_raw, -limits.velocity, limits.velocity, 12),
        torque: decode_unsigned(torque_raw, -limits.torque, limits.torque, 12),
        mos_temperature_raw: data[6],
        winding_temperature_raw: data[7],
        observed_monotonic,
        sequence,
    })
}

fn system_frame(esc_id: DamiaoEscId, data: &[u8; 8]) -> ClassicCanFrame {
    match ClassicCanFrame::new(esc_id.standard_id(), data) {
        Ok(frame) => frame,
        Err(_) => unreachable!("fixed Damiao system payload is always a classic CAN frame"),
    }
}

/// Builds the exact MIT enable frame without transmitting it.
#[must_use]
pub fn enable_frame(esc_id: DamiaoEscId) -> ClassicCanFrame {
    system_frame(esc_id, &ENABLE_DATA)
}

/// Builds the exact MIT disable frame without transmitting it.
#[must_use]
pub fn disable_frame(esc_id: DamiaoEscId) -> ClassicCanFrame {
    system_frame(esc_id, &DISABLE_DATA)
}

/// Builds the explicit disarmed clear-error frame without transmitting it.
#[must_use]
pub fn clear_error_frame(esc_id: DamiaoEscId) -> ClassicCanFrame {
    system_frame(esc_id, &CLEAR_ERROR_DATA)
}

/// Builds the destructive save-current-position-as-zero frame.
#[must_use]
pub fn zero_position_frame(esc_id: DamiaoEscId) -> ClassicCanFrame {
    system_frame(esc_id, &ZERO_POSITION_DATA)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn limits() -> RuntimeLimits {
        RuntimeLimits::new(12.5, 30.0, 10.0).unwrap()
    }

    fn esc() -> DamiaoEscId {
        DamiaoEscId::new(1).unwrap()
    }

    #[test]
    fn codec_mit_matches_pinned_golden_vectors() {
        let vectors = [
            (
                MitCommand {
                    position: 0.0,
                    velocity: 0.0,
                    kp: 0.0,
                    kd: 0.0,
                    feed_forward_torque: 0.0,
                },
                [0x7f, 0xff, 0x7f, 0xf0, 0x00, 0x00, 0x07, 0xff],
            ),
            (
                MitCommand {
                    position: 1.0,
                    velocity: -2.0,
                    kp: 50.0,
                    kd: 0.5,
                    feed_forward_torque: 3.0,
                },
                [0x8a, 0x3c, 0x77, 0x71, 0x99, 0x19, 0x9a, 0x65],
            ),
            (
                MitCommand {
                    position: -12.5,
                    velocity: -30.0,
                    kp: 0.0,
                    kd: 0.0,
                    feed_forward_torque: -10.0,
                },
                [0; 8],
            ),
            (
                MitCommand {
                    position: 12.5,
                    velocity: 30.0,
                    kp: 500.0,
                    kd: 5.0,
                    feed_forward_torque: 10.0,
                },
                [0xff; 8],
            ),
        ];
        for (command, expected) in vectors {
            assert_eq!(
                encode_mit_command(esc(), limits(), command).unwrap().data(),
                expected
            );
        }
    }

    #[test]
    fn codec_rejects_non_finite_and_out_of_range_values() {
        let base = MitCommand {
            position: 0.0,
            velocity: 0.0,
            kp: 0.0,
            kd: 0.0,
            feed_forward_torque: 0.0,
        };
        for bad in [f32::NAN, f32::INFINITY, f32::NEG_INFINITY, 12.500_001] {
            assert!(
                encode_mit_command(
                    esc(),
                    limits(),
                    MitCommand {
                        position: bad,
                        ..base
                    }
                )
                .is_err()
            );
        }
        assert!(encode_mit_command(esc(), limits(), MitCommand { kp: -0.1, ..base }).is_err());
        assert!(RuntimeLimits::new(0.0, 1.0, 1.0).is_err());
    }

    #[test]
    fn codec_feedback_matches_golden_vector_and_temperatures() {
        let mst = StandardCanId::new(0x11).unwrap();
        let frame =
            ClassicCanFrame::new(mst, &[0x11, 0x8a, 0x3c, 0x77, 0x7a, 0x65, 0x2a, 0x2b]).unwrap();
        let feedback =
            decode_feedback(frame, mst, esc(), limits(), MonoTime::from_nanos(77), 9).unwrap();
        assert_eq!(feedback.status, DamiaoStatus::Enabled);
        assert!((feedback.position - 0.999_656_7).abs() < 0.000_01);
        assert!((feedback.velocity - -2.0).abs() < 0.000_01);
        assert!((feedback.torque - 2.996_337).abs() < 0.000_01);
        assert_eq!(feedback.mos_temperature_raw, 42);
        assert_eq!(feedback.winding_temperature_raw, 43);
        assert_eq!(feedback.sequence, 9);
    }

    #[test]
    fn codec_status_preserves_every_nibble() {
        let expected_unknown = [2, 6, 7, 15];
        for raw in 0_u8..=15 {
            let status = DamiaoStatus::from_raw(raw);
            assert_eq!(status.as_raw(), raw);
            assert_eq!(
                matches!(status, DamiaoStatus::Unknown(_)),
                expected_unknown.contains(&raw)
            );
            assert_eq!(status.is_fault(), raw > 1);
        }
    }

    #[test]
    fn codec_raw_decoders_are_total_over_their_full_domains() {
        let limits = limits();
        let mut previous = f32::NEG_INFINITY;
        for raw in 0_u16..=u16::MAX {
            let value = decode_unsigned(raw, -limits.position, limits.position, 16);
            assert!(value.is_finite());
            assert!(value >= previous);
            previous = value;
        }
        previous = f32::NEG_INFINITY;
        for raw in 0_u16..=0x0fff {
            let value = decode_unsigned(raw, -limits.velocity, limits.velocity, 12);
            assert!(value.is_finite());
            assert!(value >= previous);
            previous = value;
        }
    }

    #[test]
    fn codec_feedback_rejects_wrong_id_identity_and_every_invalid_dlc() {
        let mst = StandardCanId::new(0x11).unwrap();
        let valid = [0x01, 0, 0, 0, 0, 0, 0, 0];
        let wrong_id = ClassicCanFrame::new(StandardCanId::new(0x12).unwrap(), &valid).unwrap();
        assert!(
            decode_feedback(wrong_id, mst, esc(), limits(), MonoTime::from_nanos(0), 0).is_err()
        );

        let mut wrong_identity = valid;
        wrong_identity[0] = 0x02;
        let wrong_identity = ClassicCanFrame::new(mst, &wrong_identity).unwrap();
        assert!(
            decode_feedback(
                wrong_identity,
                mst,
                esc(),
                limits(),
                MonoTime::from_nanos(0),
                0
            )
            .is_err()
        );

        for dlc in 0..8 {
            let frame = ClassicCanFrame::new(mst, &valid[..dlc]).unwrap();
            assert!(
                decode_feedback(frame, mst, esc(), limits(), MonoTime::from_nanos(0), 0).is_err()
            );
        }
    }

    #[test]
    fn codec_feedback_decoder_is_total_across_raw_field_domains() {
        let mst = StandardCanId::new(0x11).unwrap();
        for first in 0_u8..=u8::MAX {
            let frame = ClassicCanFrame::new(mst, &[first, 0, 0, 0, 0, 0, 0, 0]).unwrap();
            let _ = decode_feedback(frame, mst, esc(), limits(), MonoTime::from_nanos(0), 0);
        }
        for raw in 0_u16..=u16::MAX {
            let bytes = raw.to_be_bytes();
            let frame = ClassicCanFrame::new(mst, &[1, bytes[0], bytes[1], 0, 0, 0, 0, 0]).unwrap();
            assert!(
                decode_feedback(frame, mst, esc(), limits(), MonoTime::from_nanos(0), 0,).is_ok()
            );
        }
        for raw in 0_u16..=0x0fff {
            let frame = ClassicCanFrame::new(
                mst,
                &[
                    1,
                    0,
                    0,
                    (raw >> 4) as u8,
                    ((raw & 0x0f) << 4) as u8 | ((raw >> 8) & 0x0f) as u8,
                    raw as u8,
                    0,
                    0,
                ],
            )
            .unwrap();
            assert!(
                decode_feedback(frame, mst, esc(), limits(), MonoTime::from_nanos(0), 0,).is_ok()
            );
        }
    }

    #[test]
    fn codec_system_frames_are_exact_and_side_effect_free() {
        assert_eq!(enable_frame(esc()).data(), ENABLE_DATA);
        assert_eq!(disable_frame(esc()).data(), DISABLE_DATA);
        assert_eq!(clear_error_frame(esc()).data(), CLEAR_ERROR_DATA);
        assert_eq!(zero_position_frame(esc()).data(), ZERO_POSITION_DATA);
    }

    #[test]
    fn codec_esc_id_enforces_low_nibble_policy() {
        assert!(DamiaoEscId::new(0).is_err());
        assert_eq!(DamiaoEscId::new(1).unwrap().as_raw(), 1);
        assert_eq!(DamiaoEscId::new(15).unwrap().as_raw(), 15);
        assert!(DamiaoEscId::new(16).is_err());
    }
}
