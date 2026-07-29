//! Strict, deployment-only robot topology configuration.
//!
//! This crate intentionally does not depend on a production layer. A process
//! composition root parses the full document and later converts it into narrow,
//! layer-owned immutable startup settings.

use serde::Deserialize;
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, BTreeSet},
    fmt::{self, Write as _},
};
use thiserror::Error;

/// Supported robot YAML schema version.
pub const SCHEMA_VERSION: u32 = 1;
/// Maximum accepted YAML byte length.
pub const MAX_CONFIG_BYTES: usize = 64 * 1024;
const MIN_BAUD_RATE: u32 = 1_200;
const MAX_BAUD_RATE: u32 = 4_000_000;
const MIN_MOTOR_ID: u16 = 1;
const MAX_STANDARD_CAN_ID: u16 = 0x07ff;

/// Validated immutable robot topology.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct RobotConfig {
    schema_version: u32,
    chassis: ChassisConfig,
    arm: ArmConfig,
    imu: ImuConfig,
}

impl RobotConfig {
    /// Parses and semantically validates one YAML document.
    pub fn parse(yaml: &str) -> Result<Self, ConfigError> {
        if yaml.len() > MAX_CONFIG_BYTES {
            return Err(ConfigError::TooLarge {
                actual: yaml.len(),
                maximum: MAX_CONFIG_BYTES,
            });
        }
        let raw: RawRobotConfig =
            serde_saphyr::from_str(yaml).map_err(|error| ConfigError::Syntax(error.to_string()))?;
        Self::try_from(raw)
    }

    /// Returns the schema version.
    #[must_use]
    pub const fn schema_version(&self) -> u32 {
        self.schema_version
    }

    /// Returns the validated chassis selection.
    #[must_use]
    pub const fn chassis(&self) -> &ChassisConfig {
        &self.chassis
    }

    /// Returns the validated arm selection.
    #[must_use]
    pub const fn arm(&self) -> &ArmConfig {
        &self.arm
    }

    /// Returns the validated IMU transport selection.
    #[must_use]
    pub const fn imu(&self) -> ImuConfig {
        self.imu
    }

    /// Produces a deterministic minimal YAML representation.
    #[must_use]
    pub fn normalized_yaml(&self) -> String {
        let mut output = String::new();
        writeln!(output, "schema_version: {}", self.schema_version)
            .expect("writing to String is infallible");
        writeln!(output, "chassis:").expect("writing to String is infallible");
        writeln!(output, "  morphology: {}", self.chassis.morphology.as_str())
            .expect("writing to String is infallible");
        write_motor_ids(&mut output, &self.chassis.motor_ids, 2);
        writeln!(output, "arm:").expect("writing to String is infallible");
        writeln!(output, "  morphology: {}", self.arm.morphology)
            .expect("writing to String is infallible");
        write_motor_ids(&mut output, &self.arm.motor_ids, 2);
        writeln!(output, "imu:").expect("writing to String is infallible");
        writeln!(output, "  transport: {}", self.imu.transport.as_str())
            .expect("writing to String is infallible");
        writeln!(output, "  baud_rate: {}", self.imu.baud_rate)
            .expect("writing to String is infallible");
        output
    }

    /// Returns a SHA-256 digest of the deterministic normalized YAML.
    #[must_use]
    pub fn normalized_hash(&self) -> ConfigHash {
        let digest = Sha256::digest(self.normalized_yaml().as_bytes());
        let mut bytes = [0; 32];
        bytes.copy_from_slice(&digest);
        ConfigHash(bytes)
    }
}

impl TryFrom<RawRobotConfig> for RobotConfig {
    type Error = ConfigError;

    fn try_from(raw: RawRobotConfig) -> Result<Self, Self::Error> {
        if raw.schema_version != SCHEMA_VERSION {
            return Err(ConfigError::UnsupportedSchemaVersion {
                actual: raw.schema_version,
                supported: SCHEMA_VERSION,
            });
        }

        validate_exact_roles(
            "chassis",
            &raw.chassis.motor_ids,
            raw.chassis.morphology.required_roles(),
        )?;
        validate_motor_ids("chassis", &raw.chassis.motor_ids)?;
        validate_identifier("arm.morphology", &raw.arm.morphology)?;
        for role in raw.arm.motor_ids.keys() {
            validate_identifier("arm.motor_ids role", role)?;
        }
        if raw.arm.morphology == "none" && !raw.arm.motor_ids.is_empty() {
            return Err(ConfigError::ArmNoneHasMotors);
        }
        if raw.arm.morphology != "none" && raw.arm.motor_ids.is_empty() {
            return Err(ConfigError::ArmProfileHasNoMotors {
                morphology: raw.arm.morphology,
            });
        }
        validate_motor_ids("arm", &raw.arm.motor_ids)?;
        if !(MIN_BAUD_RATE..=MAX_BAUD_RATE).contains(&raw.imu.baud_rate) {
            return Err(ConfigError::InvalidBaudRate {
                actual: raw.imu.baud_rate,
                minimum: MIN_BAUD_RATE,
                maximum: MAX_BAUD_RATE,
            });
        }

        Ok(Self {
            schema_version: raw.schema_version,
            chassis: ChassisConfig {
                morphology: raw.chassis.morphology,
                motor_ids: raw.chassis.motor_ids,
            },
            arm: ArmConfig {
                morphology: raw.arm.morphology,
                motor_ids: raw.arm.motor_ids,
            },
            imu: ImuConfig {
                transport: raw.imu.transport,
                baud_rate: raw.imu.baud_rate,
            },
        })
    }
}

/// Validated chassis selection and logical motor bindings.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ChassisConfig {
    morphology: ChassisMorphology,
    motor_ids: BTreeMap<String, u16>,
}

impl ChassisConfig {
    /// Returns the chassis morphology.
    #[must_use]
    pub const fn morphology(&self) -> ChassisMorphology {
        self.morphology
    }

    /// Returns sorted logical-role to motor-ID bindings.
    #[must_use]
    pub const fn motor_ids(&self) -> &BTreeMap<String, u16> {
        &self.motor_ids
    }
}

/// Chassis morphology with a compiled set of required motor roles.
#[derive(Clone, Copy, Debug, Deserialize, Eq, Hash, PartialEq)]
pub enum ChassisMorphology {
    /// Four-wheel mecanum chassis.
    #[serde(rename = "mecanum_4")]
    Mecanum4,
    /// Three-wheel omni chassis.
    #[serde(rename = "omni_3")]
    Omni3,
    /// Four-wheel omni chassis.
    #[serde(rename = "omni_4")]
    Omni4,
    /// Four-module swerve chassis with drive and steering motors.
    #[serde(rename = "swerve_4")]
    Swerve4,
}

impl ChassisMorphology {
    /// Returns the stable YAML spelling.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Mecanum4 => "mecanum_4",
            Self::Omni3 => "omni_3",
            Self::Omni4 => "omni_4",
            Self::Swerve4 => "swerve_4",
        }
    }

    /// Returns logical motor roles required by this compiled morphology.
    #[must_use]
    pub const fn required_roles(self) -> &'static [&'static str] {
        match self {
            Self::Mecanum4 | Self::Omni4 => {
                &["front_left", "front_right", "rear_left", "rear_right"]
            }
            Self::Omni3 => &["front", "rear_left", "rear_right"],
            Self::Swerve4 => &[
                "front_left_drive",
                "front_left_steer",
                "front_right_drive",
                "front_right_steer",
                "rear_left_drive",
                "rear_left_steer",
                "rear_right_drive",
                "rear_right_steer",
            ],
        }
    }
}

/// Validated arm profile name and logical motor bindings.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ArmConfig {
    morphology: String,
    motor_ids: BTreeMap<String, u16>,
}

impl ArmConfig {
    /// Returns the compiled-profile name selected by deployment.
    #[must_use]
    pub fn morphology(&self) -> &str {
        &self.morphology
    }

    /// Returns sorted logical-joint to motor-ID bindings.
    #[must_use]
    pub const fn motor_ids(&self) -> &BTreeMap<String, u16> {
        &self.motor_ids
    }
}

/// Validated IMU transport selection.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ImuConfig {
    transport: ImuTransport,
    baud_rate: u32,
}

impl ImuConfig {
    /// Returns the selected physical transport family.
    #[must_use]
    pub const fn transport(self) -> ImuTransport {
        self.transport
    }

    /// Returns the selected baud rate.
    #[must_use]
    pub const fn baud_rate(self) -> u32 {
        self.baud_rate
    }
}

/// IMU connection type kept independent of a concrete protocol driver.
#[derive(Clone, Copy, Debug, Deserialize, Eq, Hash, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum ImuTransport {
    /// Native UART device.
    Uart,
    /// USB or USB-serial device.
    Usb,
}

impl ImuTransport {
    /// Returns the stable YAML spelling.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Uart => "uart",
            Self::Usb => "usb",
        }
    }
}

/// Deterministic SHA-256 configuration identity.
#[derive(Clone, Copy, Eq, Hash, PartialEq)]
pub struct ConfigHash([u8; 32]);

impl ConfigHash {
    /// Returns the digest bytes.
    #[must_use]
    pub const fn as_bytes(&self) -> &[u8; 32] {
        &self.0
    }
}

impl fmt::Debug for ConfigHash {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        fmt::Display::fmt(self, formatter)
    }
}

impl fmt::Display for ConfigHash {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        for byte in self.0 {
            write!(formatter, "{byte:02x}")?;
        }
        Ok(())
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawRobotConfig {
    schema_version: u32,
    chassis: RawChassisConfig,
    arm: RawArmConfig,
    imu: RawImuConfig,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawChassisConfig {
    morphology: ChassisMorphology,
    motor_ids: BTreeMap<String, u16>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawArmConfig {
    morphology: String,
    motor_ids: BTreeMap<String, u16>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct RawImuConfig {
    transport: ImuTransport,
    baud_rate: u32,
}

fn validate_exact_roles(
    scope: &'static str,
    actual: &BTreeMap<String, u16>,
    required: &[&str],
) -> Result<(), ConfigError> {
    let actual: BTreeSet<&str> = actual.keys().map(String::as_str).collect();
    let required: BTreeSet<&str> = required.iter().copied().collect();
    if actual == required {
        return Ok(());
    }
    let missing = required
        .difference(&actual)
        .copied()
        .map(str::to_owned)
        .collect();
    let unexpected = actual
        .difference(&required)
        .copied()
        .map(str::to_owned)
        .collect();
    Err(ConfigError::RoleMismatch {
        scope,
        missing,
        unexpected,
    })
}

fn validate_motor_ids(
    scope: &'static str,
    motor_ids: &BTreeMap<String, u16>,
) -> Result<(), ConfigError> {
    let mut seen = BTreeMap::<u16, &str>::new();
    for (role, id) in motor_ids {
        if !(MIN_MOTOR_ID..=MAX_STANDARD_CAN_ID).contains(id) {
            return Err(ConfigError::MotorIdOutOfRange {
                scope,
                role: role.clone(),
                id: *id,
                minimum: MIN_MOTOR_ID,
                maximum: MAX_STANDARD_CAN_ID,
            });
        }
        if let Some(first_role) = seen.insert(*id, role) {
            return Err(ConfigError::DuplicateMotorId {
                scope,
                id: *id,
                first_role: first_role.to_owned(),
                second_role: role.clone(),
            });
        }
    }
    Ok(())
}

fn validate_identifier(field: &'static str, value: &str) -> Result<(), ConfigError> {
    let valid_len = !value.is_empty() && value.len() <= 64;
    let valid_chars = value.bytes().enumerate().all(|(index, byte)| {
        if index == 0 {
            byte.is_ascii_lowercase()
        } else {
            byte.is_ascii_lowercase() || byte.is_ascii_digit() || matches!(byte, b'_' | b'-')
        }
    });
    if valid_len && valid_chars {
        Ok(())
    } else {
        Err(ConfigError::InvalidIdentifier {
            field,
            value: value.to_owned(),
        })
    }
}

fn write_motor_ids(output: &mut String, ids: &BTreeMap<String, u16>, indent: usize) {
    let prefix = " ".repeat(indent);
    if ids.is_empty() {
        writeln!(output, "{prefix}motor_ids: {{}}").expect("writing to String is infallible");
        return;
    }
    writeln!(output, "{prefix}motor_ids:").expect("writing to String is infallible");
    for (role, id) in ids {
        writeln!(output, "{prefix}  {role}: {id}").expect("writing to String is infallible");
    }
}

/// Robot YAML syntax or semantic validation failure.
#[derive(Debug, Error, PartialEq)]
pub enum ConfigError {
    /// YAML parser rejected the typed document.
    #[error("invalid robot YAML: {0}")]
    Syntax(String),
    /// Input exceeded the bounded configuration size.
    #[error("robot YAML size {actual} bytes exceeds maximum {maximum} bytes")]
    TooLarge {
        /// Observed input size.
        actual: usize,
        /// Configured size ceiling.
        maximum: usize,
    },
    /// Schema version is not implemented.
    #[error("unsupported schema_version {actual}; this build supports {supported}")]
    UnsupportedSchemaVersion {
        /// Requested schema version.
        actual: u32,
        /// Supported schema version.
        supported: u32,
    },
    /// Logical roles do not exactly match the compiled morphology.
    #[error("{scope} motor roles mismatch; missing={missing:?}, unexpected={unexpected:?}")]
    RoleMismatch {
        /// Configuration section.
        scope: &'static str,
        /// Required roles not supplied.
        missing: Vec<String>,
        /// Supplied roles not recognized by the morphology.
        unexpected: Vec<String>,
    },
    /// Two roles in one device group bind the same motor ID.
    #[error("{scope} motor ID {id} is assigned to both {first_role} and {second_role}")]
    DuplicateMotorId {
        /// Configuration section.
        scope: &'static str,
        /// Duplicated ID.
        id: u16,
        /// First logical role.
        first_role: String,
        /// Second logical role.
        second_role: String,
    },
    /// Motor ID is outside the v1 standard-CAN binding range.
    #[error("{scope}.{role} motor ID {id} is outside {minimum}..={maximum}")]
    MotorIdOutOfRange {
        /// Configuration section.
        scope: &'static str,
        /// Logical role.
        role: String,
        /// Rejected ID.
        id: u16,
        /// Minimum ID.
        minimum: u16,
        /// Maximum ID.
        maximum: u16,
    },
    /// Arm profile or joint role used a non-portable identifier.
    #[error("{field} value {value:?} must match [a-z][a-z0-9_-]{{0,63}}")]
    InvalidIdentifier {
        /// Field description.
        field: &'static str,
        /// Rejected value.
        value: String,
    },
    /// The `none` arm profile cannot bind motors.
    #[error("arm morphology 'none' requires an empty motor_ids map")]
    ArmNoneHasMotors,
    /// A named arm profile must bind at least one motor.
    #[error("arm morphology {morphology:?} requires at least one motor binding")]
    ArmProfileHasNoMotors {
        /// Selected profile.
        morphology: String,
    },
    /// Baud rate is outside the deployment-supported range.
    #[error("IMU baud_rate {actual} is outside {minimum}..={maximum}")]
    InvalidBaudRate {
        /// Rejected value.
        actual: u32,
        /// Supported minimum.
        minimum: u32,
        /// Supported maximum.
        maximum: u32,
    },
}

#[cfg(test)]
mod tests {
    use super::{ConfigError, MAX_CONFIG_BYTES, RobotConfig};

    const MECANUM: &str = include_str!("../../../../config/robots/example.yaml");
    const OMNI_3: &str = include_str!("../../../../config/robots/omni_3.yaml");
    const OMNI_4: &str = include_str!("../../../../config/robots/omni_4.yaml");
    const SWERVE_4: &str = include_str!("../../../../config/robots/swerve_4.yaml");

    #[test]
    fn all_v1_chassis_examples_validate() {
        for yaml in [MECANUM, OMNI_3, OMNI_4, SWERVE_4] {
            RobotConfig::parse(yaml).unwrap();
        }
    }

    #[test]
    fn normalization_and_hash_are_deterministic() {
        let config = RobotConfig::parse(MECANUM).unwrap();
        let normalized = config.normalized_yaml();
        let reparsed = RobotConfig::parse(&normalized).unwrap();
        assert_eq!(reparsed, config);
        assert_eq!(reparsed.normalized_hash(), config.normalized_hash());
    }

    #[test]
    fn unknown_fields_are_rejected() {
        let yaml = MECANUM.replace(
            "schema_version: 1",
            "schema_version: 1\ncan_bitrate: 1000000",
        );
        assert!(matches!(
            RobotConfig::parse(&yaml),
            Err(ConfigError::Syntax(_))
        ));
    }

    #[test]
    fn unsupported_version_is_rejected() {
        let yaml = MECANUM.replace("schema_version: 1", "schema_version: 2");
        assert!(matches!(
            RobotConfig::parse(&yaml),
            Err(ConfigError::UnsupportedSchemaVersion { actual: 2, .. })
        ));
    }

    #[test]
    fn role_mismatch_is_rejected() {
        let yaml = MECANUM.replace("    rear_right: 4\n", "");
        assert!(matches!(
            RobotConfig::parse(&yaml),
            Err(ConfigError::RoleMismatch { .. })
        ));
    }

    #[test]
    fn duplicate_motor_ids_are_rejected() {
        let yaml = MECANUM.replace("    front_right: 2", "    front_right: 1");
        assert!(matches!(
            RobotConfig::parse(&yaml),
            Err(ConfigError::DuplicateMotorId { id: 1, .. })
        ));
    }

    #[test]
    fn out_of_range_motor_id_is_rejected() {
        let yaml = MECANUM.replace("    front_left: 1", "    front_left: 0");
        assert!(matches!(
            RobotConfig::parse(&yaml),
            Err(ConfigError::MotorIdOutOfRange { id: 0, .. })
        ));
    }

    #[test]
    fn invalid_arm_profile_and_none_bindings_are_rejected() {
        let invalid_profile = MECANUM.replace("morphology: none", "morphology: Serial/6");
        assert!(matches!(
            RobotConfig::parse(&invalid_profile),
            Err(ConfigError::InvalidIdentifier { .. })
        ));

        let none_with_motor = MECANUM.replace(
            "  motor_ids: {}\nimu:",
            "  motor_ids:\n    joint_1: 10\nimu:",
        );
        assert!(matches!(
            RobotConfig::parse(&none_with_motor),
            Err(ConfigError::ArmNoneHasMotors)
        ));
    }

    #[test]
    fn invalid_transport_and_baud_are_rejected() {
        let invalid_transport = MECANUM.replace("transport: uart", "transport: spi");
        assert!(matches!(
            RobotConfig::parse(&invalid_transport),
            Err(ConfigError::Syntax(_))
        ));

        let invalid_baud = MECANUM.replace("baud_rate: 921600", "baud_rate: 0");
        assert!(matches!(
            RobotConfig::parse(&invalid_baud),
            Err(ConfigError::InvalidBaudRate { actual: 0, .. })
        ));
    }

    #[test]
    fn oversized_input_is_rejected_before_yaml_parsing() {
        let input = "x".repeat(MAX_CONFIG_BYTES + 1);
        assert!(matches!(
            RobotConfig::parse(&input),
            Err(ConfigError::TooLarge { .. })
        ));
    }
}
