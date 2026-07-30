use middleware_api::{
    EventPublishError, EventPublisher, LatestPublish, LatestPublisher, LatestSubscriber,
};
use middleware_local::{latest_channel, ordered_channel};

#[test]
fn stopped_or_stalled_consumers_never_wait_or_hide_accounting() {
    let (mut latest_writer, latest_reader) = latest_channel();
    for value in 0_u64..10_000 {
        assert!(matches!(
            latest_writer.try_publish(value),
            LatestPublish::Published { .. }
        ));
    }
    let latest = latest_writer.counters();
    assert_eq!(latest.published, 10_000);
    assert_eq!(latest.overwritten, 9_999);

    let (mut event_writer, event_reader) = ordered_channel(8).unwrap();
    for value in 0_u64..8 {
        event_writer.try_publish(value).unwrap();
    }
    for value in 8_u64..10_000 {
        assert_eq!(
            event_writer.try_publish(value),
            Err(EventPublishError::Full(value))
        );
    }
    let ordered = event_writer.counters();
    assert_eq!(ordered.published, 8);
    assert_eq!(ordered.full, 9_992);

    drop(latest_reader);
    drop(event_reader);
    assert_eq!(
        latest_writer.try_publish(10_000),
        LatestPublish::Disconnected
    );
    assert_eq!(
        event_writer.try_publish(10_000),
        Err(EventPublishError::Disconnected(10_000))
    );
    assert_eq!(latest_writer.counters().disconnected, 1);
    assert_eq!(event_writer.counters().disconnected, 1);
}

#[test]
fn latest_writer_and_reader_progress_on_independent_threads() {
    let (mut writer, mut reader) = latest_channel();
    let consumer = std::thread::spawn(move || {
        loop {
            if let Some(sample) = reader.try_read_latest()
                && sample.generation == 10_000
            {
                return (sample, reader.counters());
            }
            std::thread::yield_now();
        }
    });

    for value in 0_u64..10_000 {
        writer.try_publish(value);
        if value % 128 == 0 {
            std::thread::yield_now();
        }
    }

    let (last, counters) = consumer.join().unwrap();
    assert_eq!(last.value, 9_999);
    assert_eq!(counters.published, 10_000);
    assert!(counters.consumed > 0);
}
