use message_model::{AsciiId, ClockState, ClockStatus, DataQuality, Header, HeaderFields, Id128};
use middleware_zenoh::{
    FreshnessEvidence, FreshnessPolicy, IngressError, IngressPolicy, SourceGuard,
};

fn header(source: &str, boot: u8, sequence: u64, quality: DataQuality) -> Header {
    Header::new(HeaderFields {
        source_id: AsciiId::parse(source).unwrap(),
        boot_id: Id128::new([boot; 16]).unwrap(),
        sequence,
        source_monotonic_ns: 1_000,
        publish_unix_ns: None,
        acquisition_timestamp: None,
        valid_for_ns: 100,
        frame_id: None,
        clock_status: ClockStatus {
            state: ClockState::Synchronized,
            uncertainty_ns: 5,
        },
        quality,
    })
    .unwrap()
}

fn reception_policy() -> IngressPolicy {
    IngressPolicy {
        max_age_ns: 100,
        freshness: FreshnessPolicy::ReceptionBounded,
        allow_degraded: false,
    }
}

#[test]
fn scripted_loss_delay_duplicate_reorder_restart_and_reconnect_fail_closed() {
    let boot_one = Id128::new([1; 16]).unwrap();
    let boot_two = Id128::new([2; 16]).unwrap();
    let mut guard = SourceGuard::<4>::new(AsciiId::parse("publisher").unwrap());
    guard.authorize_boot(boot_one).unwrap();

    guard
        .accept(
            header("publisher", 1, 1, DataQuality::Valid),
            1_u8,
            10_000,
            reception_policy(),
            FreshnessEvidence::ReceptionOnly,
        )
        .unwrap();
    guard
        .accept(
            header("publisher", 1, 4, DataQuality::Valid),
            4_u8,
            10_010,
            reception_policy(),
            FreshnessEvidence::ReceptionOnly,
        )
        .unwrap();

    assert_eq!(
        guard.accept(
            header("publisher", 1, 4, DataQuality::Valid),
            4_u8,
            10_020,
            reception_policy(),
            FreshnessEvidence::ReceptionOnly,
        ),
        Err(IngressError::DuplicateSequence)
    );
    assert_eq!(
        guard.accept(
            header("publisher", 1, 3, DataQuality::Valid),
            3_u8,
            10_030,
            reception_policy(),
            FreshnessEvidence::ReceptionOnly,
        ),
        Err(IngressError::ReorderedSequence)
    );
    assert_eq!(
        guard.accept(
            header("publisher", 1, 5, DataQuality::Valid),
            5_u8,
            10_040,
            IngressPolicy {
                max_age_ns: 100,
                freshness: FreshnessPolicy::SynchronizedAgeRequired {
                    max_uncertainty_ns: 10,
                },
                allow_degraded: false,
            },
            FreshnessEvidence::TranslatedAge {
                age_ns: 100,
                uncertainty_ns: 5,
            },
        ),
        Err(IngressError::Expired)
    );
    assert_eq!(
        guard.accept(
            header("publisher", 1, 5, DataQuality::Degraded),
            5_u8,
            10_050,
            reception_policy(),
            FreshnessEvidence::ReceptionOnly,
        ),
        Err(IngressError::DisallowedQuality(DataQuality::Degraded))
    );
    assert_eq!(
        guard.accept(
            header("intruder", 1, 5, DataQuality::Valid),
            5_u8,
            10_060,
            reception_policy(),
            FreshnessEvidence::ReceptionOnly,
        ),
        Err(IngressError::UnexpectedSource)
    );
    assert_eq!(
        guard.accept(
            header("publisher", 2, 1, DataQuality::Valid),
            1_u8,
            10_070,
            reception_policy(),
            FreshnessEvidence::ReceptionOnly,
        ),
        Err(IngressError::UnapprovedBoot)
    );

    guard.authorize_boot(boot_two).unwrap();
    guard
        .accept(
            header("publisher", 2, 1, DataQuality::Valid),
            1_u8,
            10_080,
            reception_policy(),
            FreshnessEvidence::ReceptionOnly,
        )
        .unwrap();
    assert_eq!(
        guard.accept(
            header("publisher", 1, 5, DataQuality::Valid),
            5_u8,
            10_090,
            reception_policy(),
            FreshnessEvidence::ReceptionOnly,
        ),
        Err(IngressError::RetiredBoot)
    );

    guard.mark_disconnected();
    assert_eq!(
        guard.accept(
            header("publisher", 2, 2, DataQuality::Valid),
            2_u8,
            10_100,
            reception_policy(),
            FreshnessEvidence::ReceptionOnly,
        ),
        Err(IngressError::Unavailable)
    );
    guard.authorize_boot(boot_two).unwrap();
    guard
        .accept(
            header("publisher", 2, 2, DataQuality::Valid),
            2_u8,
            10_110,
            reception_policy(),
            FreshnessEvidence::ReceptionOnly,
        )
        .unwrap();

    let counters = guard.counters();
    assert_eq!(counters.accepted, 4);
    assert_eq!(counters.lost_sequences, 2);
    assert_eq!(counters.duplicate, 1);
    assert_eq!(counters.reordered, 1);
    assert_eq!(counters.freshness, 1);
    assert_eq!(counters.quality, 1);
    assert_eq!(counters.unexpected_source, 1);
    assert_eq!(counters.boot, 2);
    assert_eq!(counters.unavailable, 1);
}
