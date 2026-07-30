use middleware_api::SemanticQos;
use zenoh::qos::{CongestionControl, Priority};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct ZenohQos {
    pub(crate) priority: Priority,
    pub(crate) congestion: CongestionControl,
}

pub(crate) const fn zenoh_qos(value: SemanticQos) -> ZenohQos {
    match value {
        SemanticQos::ControlLatest => ZenohQos {
            priority: Priority::RealTime,
            congestion: CongestionControl::Drop,
        },
        SemanticQos::StateLatest => ZenohQos {
            priority: Priority::DataHigh,
            congestion: CongestionControl::Drop,
        },
        SemanticQos::CorrelatedEvent | SemanticQos::RequestReply | SemanticQos::Presence => {
            ZenohQos {
                priority: Priority::InteractiveHigh,
                congestion: CongestionControl::Drop,
            }
        }
        SemanticQos::BulkLossy => ZenohQos {
            priority: Priority::DataLow,
            congestion: CongestionControl::Drop,
        },
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_semantic_class_maps_to_nonblocking_congestion() {
        for qos in [
            SemanticQos::ControlLatest,
            SemanticQos::StateLatest,
            SemanticQos::CorrelatedEvent,
            SemanticQos::RequestReply,
            SemanticQos::Presence,
            SemanticQos::BulkLossy,
        ] {
            assert_eq!(zenoh_qos(qos).congestion, CongestionControl::Drop);
        }
    }
}
