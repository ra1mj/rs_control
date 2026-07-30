use std::{fmt, str::FromStr};

use message_model::{AsciiId, IdError, MAX_ID_LEN};
use thiserror::Error;

/// Stable robot identifier used in the canonical key space.
pub type RobotId = AsciiId<MAX_ID_LEN>;
/// Stable resource segment used in the canonical key space.
pub type KeyResource = AsciiId<MAX_ID_LEN>;
/// Stable channel segment used in the canonical key space.
pub type KeyChannel = AsciiId<MAX_ID_LEN>;

/// Canonical protocol domain segment.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub enum KeyDomain {
    /// Device observations and single-device commands.
    Device,
    /// Robot control commands and state.
    Control,
    /// Component and system health.
    Health,
    /// System/mission lifecycle.
    Fsm,
    /// Perception observations and results.
    Perception,
}

impl KeyDomain {
    /// Returns the canonical wire spelling.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Device => "device",
            Self::Control => "control",
            Self::Health => "health",
            Self::Fsm => "fsm",
            Self::Perception => "perception",
        }
    }
}

impl FromStr for KeyDomain {
    type Err = KeyError;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        match value {
            "device" => Ok(Self::Device),
            "control" => Ok(Self::Control),
            "health" => Ok(Self::Health),
            "fsm" => Ok(Self::Fsm),
            "perception" => Ok(Self::Perception),
            _ => Err(KeyError::UnknownDomain(value.to_owned())),
        }
    }
}

/// Validated concrete `robot/<id>/v1/<domain>/<resource>/<channel>` key.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct ConcreteKey {
    robot_id: RobotId,
    domain: KeyDomain,
    resource: KeyResource,
    channel: KeyChannel,
}

impl ConcreteKey {
    /// Constructs a concrete key from already validated segments.
    #[must_use]
    pub const fn new(
        robot_id: RobotId,
        domain: KeyDomain,
        resource: KeyResource,
        channel: KeyChannel,
    ) -> Self {
        Self {
            robot_id,
            domain,
            resource,
            channel,
        }
    }

    /// Returns the robot segment.
    #[must_use]
    pub const fn robot_id(self) -> RobotId {
        self.robot_id
    }

    /// Returns the domain segment.
    #[must_use]
    pub const fn domain(self) -> KeyDomain {
        self.domain
    }

    /// Returns the resource segment.
    #[must_use]
    pub const fn resource(self) -> KeyResource {
        self.resource
    }

    /// Returns the channel segment.
    #[must_use]
    pub const fn channel(self) -> KeyChannel {
        self.channel
    }
}

impl fmt::Display for ConcreteKey {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "robot/{}/v1/{}/{}/{}",
            self.robot_id,
            self.domain.as_str(),
            self.resource,
            self.channel
        )
    }
}

impl FromStr for ConcreteKey {
    type Err = KeyError;

    fn from_str(value: &str) -> Result<Self, Self::Err> {
        let segments = value.split('/').collect::<Vec<_>>();
        if segments.len() != 6 {
            return Err(KeyError::SegmentCount {
                actual: segments.len(),
            });
        }
        if segments[0] != "robot" {
            return Err(KeyError::InvalidRoot(segments[0].to_owned()));
        }
        if segments[2] != "v1" {
            return Err(KeyError::UnsupportedVersion(segments[2].to_owned()));
        }
        Ok(Self::new(
            parse_segment(segments[1], "robot_id")?,
            segments[3].parse()?,
            parse_segment(segments[4], "resource")?,
            parse_segment(segments[5], "channel")?,
        ))
    }
}

fn parse_segment(value: &str, field: &'static str) -> Result<AsciiId<MAX_ID_LEN>, KeyError> {
    AsciiId::parse(value).map_err(|error| KeyError::InvalidIdentifier { field, error })
}

/// A selector created only through a named wildcard scope.
#[derive(Clone, Debug, Eq, Hash, PartialEq)]
pub struct KeySelector(String);

impl KeySelector {
    /// Selects every v1 key belonging to one robot.
    #[must_use]
    pub fn for_robot(robot_id: RobotId) -> Self {
        Self(format!("robot/{robot_id}/v1/**"))
    }

    /// Selects every v1 key in one robot/domain pair.
    #[must_use]
    pub fn for_domain(robot_id: RobotId, domain: KeyDomain) -> Self {
        Self(format!("robot/{robot_id}/v1/{}/**", domain.as_str()))
    }

    /// Returns the validated selector spelling.
    #[must_use]
    pub fn as_str(&self) -> &str {
        &self.0
    }
}

/// Canonical-key validation failure.
#[derive(Clone, Debug, Eq, Error, PartialEq)]
pub enum KeyError {
    /// A key did not contain the six required segments.
    #[error("canonical key must contain 6 segments, got {actual}")]
    SegmentCount {
        /// Observed segment count.
        actual: usize,
    },
    /// The root segment was not `robot`.
    #[error("canonical key root must be robot, got {0}")]
    InvalidRoot(String),
    /// Only the v1 key space is accepted.
    #[error("unsupported key-space version {0}")]
    UnsupportedVersion(String),
    /// A domain segment was not registered.
    #[error("unknown key domain {0}")]
    UnknownDomain(String),
    /// An identity segment failed its owning validator.
    #[error("invalid {field} key segment: {error}")]
    InvalidIdentifier {
        /// Segment role.
        field: &'static str,
        /// Owning identifier error.
        #[source]
        error: IdError,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id(value: &str) -> AsciiId<MAX_ID_LEN> {
        AsciiId::parse(value).expect("test identifier is valid")
    }

    #[test]
    fn concrete_key_round_trips_every_domain() {
        for domain in [
            KeyDomain::Device,
            KeyDomain::Control,
            KeyDomain::Health,
            KeyDomain::Fsm,
            KeyDomain::Perception,
        ] {
            let key = ConcreteKey::new(id("r1"), domain, id("front_left"), id("state"));
            assert_eq!(key.to_string().parse(), Ok(key));
        }
    }

    #[test]
    fn concrete_key_rejects_wildcards_versions_and_extra_segments() {
        assert!(matches!(
            "robot/r1/v2/device/motor/state".parse::<ConcreteKey>(),
            Err(KeyError::UnsupportedVersion(_))
        ));
        assert!(matches!(
            "robot/r1/v1/device/motor/*".parse::<ConcreteKey>(),
            Err(KeyError::InvalidIdentifier {
                field: "channel",
                ..
            })
        ));
        assert!(matches!(
            "robot/r1/v1/device/motor/state/extra".parse::<ConcreteKey>(),
            Err(KeyError::SegmentCount { actual: 7 })
        ));
    }

    #[test]
    fn selectors_are_created_only_for_named_scopes() {
        assert_eq!(KeySelector::for_robot(id("r1")).as_str(), "robot/r1/v1/**");
        assert_eq!(
            KeySelector::for_domain(id("r1"), KeyDomain::Health).as_str(),
            "robot/r1/v1/health/**"
        );
    }
}
