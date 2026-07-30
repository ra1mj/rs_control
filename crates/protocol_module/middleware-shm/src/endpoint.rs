//! Typed middleware ports over a validated private mapping.

use std::{fs::File, path::PathBuf};

use middleware_api::{
    ChannelCounters, EventPublish, EventPublishError, EventPublisher, EventSubscriber,
    LatestPublish, LatestPublisher, LatestSample, LatestSubscriber, OrderedSample,
};

use crate::{
    ChannelName, EndpointRole, ShmChannelConfig, ShmCodec, ShmError, ShmNamespace,
    layout::ChannelKind,
    mapping::{MappedRegion, RawDiagnostics, SlotRead, ValueMarker},
    namespace::{AttachedRegion, probe_endpoint},
};

/// Extra bounded observations that do not reinterpret common port counters.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ShmDiagnostics {
    /// Attach attempts rejected by a live endpoint lock.
    pub attach_rejected: u64,
    /// Reads abandoned after bounded seqlock validation failed.
    pub read_races: u64,
    /// Complete fixed representations rejected by the semantic codec.
    pub decode_rejected: u64,
    /// Stale active metadata repaired only after kernel-lock acquisition.
    pub reclaimed_endpoints: u64,
    /// Ordered generations abandoned when a subscriber lease was claimed.
    pub skipped_backlog: u64,
}

impl From<RawDiagnostics> for ShmDiagnostics {
    fn from(raw: RawDiagnostics) -> Self {
        Self {
            attach_rejected: raw.attach_rejected,
            read_races: raw.read_races,
            decode_rejected: raw.decode_rejected,
            reclaimed_endpoints: raw.reclaimed_endpoints,
            skipped_backlog: raw.skipped_backlog,
        }
    }
}

/// Single-producer latest-value endpoint.
pub struct ShmLatestPublisher<T, C: ShmCodec<T>> {
    region: MappedRegion,
    _endpoint_lock: File,
    peer_lock_path: PathBuf,
    codec: C,
    _value: ValueMarker<T>,
}

/// Single-consumer latest-value endpoint.
pub struct ShmLatestSubscriber<T, C: ShmCodec<T>> {
    region: MappedRegion,
    _endpoint_lock: File,
    peer_lock_path: PathBuf,
    codec: C,
    _value: ValueMarker<T>,
}

/// Single-producer ordered-event endpoint.
pub struct ShmEventPublisher<T, C: ShmCodec<T>> {
    region: MappedRegion,
    _endpoint_lock: File,
    peer_lock_path: PathBuf,
    codec: C,
    _value: ValueMarker<T>,
}

/// Single-consumer ordered-event endpoint.
pub struct ShmEventSubscriber<T, C: ShmCodec<T>> {
    region: MappedRegion,
    _endpoint_lock: File,
    peer_lock_path: PathBuf,
    codec: C,
    _value: ValueMarker<T>,
}

impl ShmNamespace {
    /// Validates the region and claims the sole latest-value publisher lease.
    pub fn attach_latest_publisher<T, C: ShmCodec<T>>(
        &self,
        name: ChannelName,
        config: ShmChannelConfig,
        codec: C,
    ) -> Result<ShmLatestPublisher<T, C>, ShmError> {
        let attached =
            self.attach::<C::Repr>(ChannelKind::Latest, name, config, EndpointRole::Publisher)?;
        Ok(ShmLatestPublisher::new(attached, codec))
    }

    /// Validates the region and claims the sole latest-value subscriber lease.
    pub fn attach_latest_subscriber<T, C: ShmCodec<T>>(
        &self,
        name: ChannelName,
        config: ShmChannelConfig,
        codec: C,
    ) -> Result<ShmLatestSubscriber<T, C>, ShmError> {
        let attached =
            self.attach::<C::Repr>(ChannelKind::Latest, name, config, EndpointRole::Subscriber)?;
        Ok(ShmLatestSubscriber::new(attached, codec))
    }

    /// Validates the region and claims the sole ordered-event publisher lease.
    pub fn attach_event_publisher<T, C: ShmCodec<T>>(
        &self,
        name: ChannelName,
        config: ShmChannelConfig,
        codec: C,
    ) -> Result<ShmEventPublisher<T, C>, ShmError> {
        let attached =
            self.attach::<C::Repr>(ChannelKind::Ordered, name, config, EndpointRole::Publisher)?;
        Ok(ShmEventPublisher::new(attached, codec))
    }

    /// Validates the region and claims the sole ordered-event subscriber lease.
    pub fn attach_event_subscriber<T, C: ShmCodec<T>>(
        &self,
        name: ChannelName,
        config: ShmChannelConfig,
        codec: C,
    ) -> Result<ShmEventSubscriber<T, C>, ShmError> {
        let attached =
            self.attach::<C::Repr>(ChannelKind::Ordered, name, config, EndpointRole::Subscriber)?;
        Ok(ShmEventSubscriber::new(attached, codec))
    }
}

macro_rules! endpoint_common {
    ($type:ident, $repair_peer:ident) => {
        impl<T, C: ShmCodec<T>> $type<T, C> {
            fn new(attached: AttachedRegion, codec: C) -> Self {
                Self {
                    region: attached.region,
                    _endpoint_lock: attached.endpoint_lock,
                    peer_lock_path: attached.peer_lock_path,
                    codec,
                    _value: ValueMarker::default(),
                }
            }

            /// Reads common and SHM-only atomic observations without I/O.
            #[must_use]
            pub fn diagnostics(&self) -> ShmDiagnostics {
                self.region.diagnostics().into()
            }

            /// Probes the peer lock outside the RT path and repairs stale metadata.
            pub fn refresh_peer_liveness(&self) -> Result<bool, ShmError> {
                let Some(_vacant_lock) = probe_endpoint(&self.peer_lock_path)? else {
                    return Ok(true);
                };
                self.region.$repair_peer();
                Ok(false)
            }
        }
    };
}

endpoint_common!(ShmLatestPublisher, repair_subscriber_inactive);
endpoint_common!(ShmLatestSubscriber, repair_publisher_inactive);
endpoint_common!(ShmEventPublisher, repair_subscriber_inactive);
endpoint_common!(ShmEventSubscriber, repair_publisher_inactive);

impl<T, C: ShmCodec<T>> LatestPublisher<T> for ShmLatestPublisher<T, C> {
    fn try_publish(&mut self, value: T) -> LatestPublish {
        if !self.region.subscriber_active() {
            self.region.record_disconnected();
            return LatestPublish::Disconnected;
        }
        let repr = self.codec.encode(&value);
        let (generation, overwritten) = self.region.publish_latest(&repr);
        LatestPublish::Published {
            generation,
            overwritten,
        }
    }

    fn counters(&self) -> ChannelCounters {
        self.region.counters()
    }
}

#[cfg(feature = "fault-injection")]
impl<T, C: ShmCodec<T>> ShmLatestPublisher<T, C> {
    /// Test-only: leaves a half-written slot under its odd version stamp.
    #[doc(hidden)]
    pub fn fault_publish_incomplete_for_test(&mut self, value: &T) -> u64 {
        let repr = self.codec.encode(value);
        self.region.fault_publish_incomplete(&repr)
    }

    /// Test-only: corrupts derived PID metadata without changing lock authority.
    #[doc(hidden)]
    pub fn fault_set_diagnostic_pid_for_test(&self, pid: u32) {
        self.region.fault_set_publisher_pid(pid);
    }

    /// Test-only: corrupts derived epoch metadata without changing lock authority.
    #[doc(hidden)]
    pub fn fault_set_diagnostic_epoch_for_test(&self, epoch: u64) {
        self.region.fault_set_publisher_epoch(epoch);
    }
}

#[cfg(feature = "fault-injection")]
impl<T, C: ShmCodec<T>> ShmLatestSubscriber<T, C> {
    /// Test-only: publishes a head with no matching complete slot.
    #[doc(hidden)]
    pub fn fault_force_unreadable_head_for_test(&self, generation: u64) {
        self.region.fault_force_latest_head(generation);
    }
}

impl<T, C: ShmCodec<T>> LatestSubscriber<T> for ShmLatestSubscriber<T, C> {
    fn try_read_latest(&mut self) -> Option<LatestSample<T>> {
        let SlotRead::Value { generation, value } = self.region.read_latest::<C::Repr>() else {
            return None;
        };
        match self.codec.decode(value) {
            Ok(value) => {
                self.region.mark_latest_consumed();
                Some(LatestSample { generation, value })
            }
            Err(_) => {
                self.region.record_decode_rejected();
                None
            }
        }
    }

    fn counters(&self) -> ChannelCounters {
        self.region.counters()
    }
}

impl<T, C: ShmCodec<T>> EventPublisher<T> for ShmEventPublisher<T, C> {
    fn try_publish(&mut self, value: T) -> Result<EventPublish, EventPublishError<T>> {
        if !self.region.subscriber_active() {
            self.region.record_disconnected();
            return Err(EventPublishError::Disconnected(value));
        }
        let repr = self.codec.encode(&value);
        let Some(generation) = self.region.publish_ordered(&repr) else {
            self.region.record_full();
            return Err(EventPublishError::Full(value));
        };
        Ok(EventPublish { generation })
    }

    fn counters(&self) -> ChannelCounters {
        self.region.counters()
    }
}

impl<T, C: ShmCodec<T>> EventSubscriber<T> for ShmEventSubscriber<T, C> {
    fn try_receive(&mut self) -> Option<OrderedSample<T>> {
        let SlotRead::Value { generation, value } = self.region.read_ordered::<C::Repr>() else {
            return None;
        };
        match self.codec.decode(value) {
            Ok(value) => {
                self.region.mark_ordered_consumed();
                Some(OrderedSample { generation, value })
            }
            Err(_) => {
                self.region.record_decode_rejected();
                None
            }
        }
    }

    fn counters(&self) -> ChannelCounters {
        self.region.counters()
    }
}

impl<T, C: ShmCodec<T>> Drop for ShmLatestPublisher<T, C> {
    fn drop(&mut self) {
        self.region.release_publisher();
    }
}

impl<T, C: ShmCodec<T>> Drop for ShmLatestSubscriber<T, C> {
    fn drop(&mut self) {
        self.region.release_subscriber();
    }
}

impl<T, C: ShmCodec<T>> Drop for ShmEventPublisher<T, C> {
    fn drop(&mut self) {
        self.region.release_publisher();
    }
}

impl<T, C: ShmCodec<T>> Drop for ShmEventSubscriber<T, C> {
    fn drop(&mut self) {
        self.region.release_subscriber();
    }
}

#[cfg(test)]
mod tests {
    use std::{fs, os::unix::fs::PermissionsExt};

    use allocation_counter::measure;
    use bytemuck::{Pod, Zeroable};
    use middleware_api::{EventPublisher, EventSubscriber, LatestPublisher, LatestSubscriber};
    use tempfile::TempDir;

    use super::*;
    use crate::{IdentityCodec, LayoutError, LayoutField, MemoryPolicy, ShmPayload};

    #[repr(C)]
    #[derive(Clone, Copy, Debug, Eq, PartialEq, Pod, Zeroable)]
    struct Payload {
        generation: u64,
        value: u64,
    }

    impl ShmPayload for Payload {
        const SCHEMA_ID: [u8; 16] = *b"endpoint-test001";
        const SCHEMA_VERSION: u32 = 1;
    }

    #[repr(C)]
    #[derive(Clone, Copy, Pod, Zeroable)]
    struct OtherPayload {
        first: u64,
        second: u64,
    }

    impl ShmPayload for OtherPayload {
        const SCHEMA_ID: [u8; 16] = *b"other-schema-v01";
        const SCHEMA_VERSION: u32 = 1;
    }

    struct RejectOdd;

    impl ShmCodec<u64> for RejectOdd {
        type Repr = Payload;
        type Error = ();

        fn encode(&mut self, value: &u64) -> Self::Repr {
            Payload {
                generation: *value,
                value: !*value,
            }
        }

        fn decode(&mut self, value: Self::Repr) -> Result<u64, Self::Error> {
            if value.generation.is_multiple_of(2) {
                Ok(value.generation)
            } else {
                Err(())
            }
        }
    }

    fn setup() -> (TempDir, ShmNamespace, ShmChannelConfig) {
        let temp = TempDir::new().unwrap();
        let namespace = ShmNamespace::new(temp.path().join("shm")).unwrap();
        let config = ShmChannelConfig::new(3, MemoryPolicy::Prefault).unwrap();
        (temp, namespace, config)
    }

    #[test]
    fn latest_reports_disconnect_delivery_and_exact_overwrite() {
        let (_temp, namespace, config) = setup();
        let name = ChannelName::new("latest").unwrap();
        namespace.create_latest::<Payload>(name, config).unwrap();
        let mut publisher = namespace
            .attach_latest_publisher(name, config, IdentityCodec::<Payload>::new())
            .unwrap();
        assert_eq!(
            publisher.try_publish(Payload {
                generation: 0,
                value: 1
            }),
            LatestPublish::Disconnected
        );
        let mut subscriber = namespace
            .attach_latest_subscriber(name, config, IdentityCodec::<Payload>::new())
            .unwrap();
        assert_eq!(
            publisher.try_publish(Payload {
                generation: 0,
                value: 2
            }),
            LatestPublish::Published {
                generation: 1,
                overwritten: false
            }
        );
        assert_eq!(
            publisher.try_publish(Payload {
                generation: 0,
                value: 3
            }),
            LatestPublish::Published {
                generation: 2,
                overwritten: true
            }
        );
        assert_eq!(subscriber.try_read_latest().unwrap().value.value, 3);
        assert!(subscriber.try_read_latest().is_none());
        assert_eq!(
            publisher.counters(),
            ChannelCounters {
                published: 2,
                consumed: 1,
                overwritten: 1,
                full: 0,
                disconnected: 1
            }
        );
        for expected_generation in 3..=10 {
            assert!(matches!(
                publisher.try_publish(Payload {
                    generation: expected_generation,
                    value: expected_generation,
                }),
                LatestPublish::Published {
                    generation,
                    overwritten: false
                } if generation == expected_generation
            ));
            assert_eq!(
                subscriber.try_read_latest().unwrap().generation,
                expected_generation
            );
        }
    }

    #[test]
    fn ordered_is_fifo_bounded_and_returns_ownership() {
        let (_temp, namespace, _) = setup();
        let config = ShmChannelConfig::new(2, MemoryPolicy::Prefault).unwrap();
        let name = ChannelName::new("events").unwrap();
        namespace.create_ordered::<Payload>(name, config).unwrap();
        let mut publisher = namespace
            .attach_event_publisher(name, config, IdentityCodec::<Payload>::new())
            .unwrap();
        let mut subscriber = namespace
            .attach_event_subscriber(name, config, IdentityCodec::<Payload>::new())
            .unwrap();
        assert_eq!(
            publisher.try_publish(Payload {
                generation: 0,
                value: 1
            }),
            Ok(EventPublish { generation: 1 })
        );
        assert_eq!(
            publisher.try_publish(Payload {
                generation: 0,
                value: 2
            }),
            Ok(EventPublish { generation: 2 })
        );
        let rejected = Payload {
            generation: 7,
            value: 3,
        };
        assert_eq!(
            publisher.try_publish(rejected),
            Err(EventPublishError::Full(rejected))
        );
        assert_eq!(subscriber.try_receive().unwrap().value.value, 1);
        assert_eq!(subscriber.try_receive().unwrap().value.value, 2);
        assert!(subscriber.try_receive().is_none());
        assert_eq!(
            publisher.counters(),
            ChannelCounters {
                published: 2,
                consumed: 2,
                overwritten: 0,
                full: 1,
                disconnected: 0
            }
        );
    }

    #[test]
    fn live_lease_refuses_duplicate_and_explicit_remove_refuses_mapping() {
        let (_temp, namespace, config) = setup();
        let name = ChannelName::new("leases").unwrap();
        namespace.create_latest::<Payload>(name, config).unwrap();
        let publisher = namespace
            .attach_latest_publisher(name, config, IdentityCodec::<Payload>::new())
            .unwrap();
        let subscriber = namespace
            .attach_latest_subscriber(name, config, IdentityCodec::<Payload>::new())
            .unwrap();
        assert!(matches!(
            namespace.attach_latest_publisher::<Payload, _>(name, config, IdentityCodec::new()),
            Err(ShmError::EndpointOccupied {
                role: EndpointRole::Publisher,
                ..
            })
        ));
        assert!(matches!(
            namespace.attach_latest_subscriber::<Payload, _>(
                name,
                config,
                IdentityCodec::<Payload>::new()
            ),
            Err(ShmError::EndpointOccupied {
                role: EndpointRole::Subscriber,
                ..
            })
        ));
        assert!(matches!(
            namespace.remove_if_unused(name),
            Err(ShmError::RegionBusy { .. })
        ));
        drop(publisher);
        drop(subscriber);
        namespace.remove_if_unused(name).unwrap();
        assert!(matches!(
            namespace.remove_if_unused(name),
            Err(ShmError::NotFound { .. })
        ));
    }

    #[test]
    fn attach_rejects_schema_and_exact_file_length_mismatches() {
        let (temp, namespace, config) = setup();
        let name = ChannelName::new("compatibility").unwrap();
        namespace.create_latest::<Payload>(name, config).unwrap();
        assert!(matches!(
            namespace.attach_latest_publisher::<OtherPayload, _>(
                name,
                config,
                IdentityCodec::<OtherPayload>::new()
            ),
            Err(ShmError::Layout(LayoutError::Incompatible {
                field: LayoutField::SchemaId
            }))
        ));
        let region_path = temp.path().join("shm/compatibility.region");
        let file = fs::OpenOptions::new()
            .write(true)
            .open(region_path)
            .unwrap();
        file.set_len(256).unwrap();
        assert!(matches!(
            namespace.attach_latest_publisher::<Payload, _>(
                name,
                config,
                IdentityCodec::<Payload>::new()
            ),
            Err(ShmError::FileLength { .. })
        ));
    }

    #[test]
    fn namespace_requires_exact_private_directory_permissions() {
        let temp = TempDir::new().unwrap();
        let root = temp.path().join("wide");
        fs::create_dir(&root).unwrap();
        fs::set_permissions(&root, fs::Permissions::from_mode(0o750)).unwrap();
        assert!(matches!(
            ShmNamespace::new(root),
            Err(ShmError::NamespacePermissions { mode: 0o750 })
        ));
    }

    #[test]
    fn attach_refuses_an_exclusive_initialization_or_admin_lock() {
        let (temp, namespace, config) = setup();
        let name = ChannelName::new("initializing").unwrap();
        namespace.create_latest::<Payload>(name, config).unwrap();
        let lock = fs::OpenOptions::new()
            .read(true)
            .write(true)
            .open(temp.path().join("shm/initializing.region.lock"))
            .unwrap();
        lock.try_lock().unwrap();
        assert!(matches!(
            namespace.attach_latest_publisher::<Payload, _>(
                name,
                config,
                IdentityCodec::<Payload>::new()
            ),
            Err(ShmError::RegionBusy { .. })
        ));
        drop(lock);
        assert!(
            namespace
                .attach_latest_publisher::<Payload, _>(
                    name,
                    config,
                    IdentityCodec::<Payload>::new()
                )
                .is_ok()
        );
    }

    #[test]
    fn codec_rejection_advances_transport_without_counting_consumption() {
        let (_temp, namespace, _) = setup();
        let config = ShmChannelConfig::new(2, MemoryPolicy::Prefault).unwrap();
        let name = ChannelName::new("decode").unwrap();
        namespace.create_ordered::<Payload>(name, config).unwrap();
        let mut publisher = namespace
            .attach_event_publisher::<u64, _>(name, config, RejectOdd)
            .unwrap();
        let mut subscriber = namespace
            .attach_event_subscriber::<u64, _>(name, config, RejectOdd)
            .unwrap();
        assert!(publisher.try_publish(1).is_ok());
        assert!(subscriber.try_receive().is_none());
        assert!(publisher.try_publish(2).is_ok());
        assert_eq!(subscriber.try_receive().unwrap().value, 2);
        assert_eq!(subscriber.diagnostics().decode_rejected, 1);
        assert_eq!(subscriber.counters().published, 2);
        assert_eq!(subscriber.counters().consumed, 1);
    }

    #[test]
    fn steady_state_latest_and_ordered_operations_allocate_nothing() {
        let (_temp, namespace, config) = setup();
        let latest = ChannelName::new("allocation-latest").unwrap();
        let ordered = ChannelName::new("allocation-ordered").unwrap();
        namespace.create_latest::<Payload>(latest, config).unwrap();
        namespace
            .create_ordered::<Payload>(ordered, config)
            .unwrap();
        let mut latest_publisher = namespace
            .attach_latest_publisher(latest, config, IdentityCodec::<Payload>::new())
            .unwrap();
        let mut latest_subscriber = namespace
            .attach_latest_subscriber(latest, config, IdentityCodec::<Payload>::new())
            .unwrap();
        let mut event_publisher = namespace
            .attach_event_publisher(ordered, config, IdentityCodec::<Payload>::new())
            .unwrap();
        let mut event_subscriber = namespace
            .attach_event_subscriber(ordered, config, IdentityCodec::<Payload>::new())
            .unwrap();
        let warm = Payload {
            generation: 0,
            value: 0,
        };
        let _ = latest_publisher.try_publish(warm);
        let _ = latest_subscriber.try_read_latest();
        let _ = event_publisher.try_publish(warm);
        let _ = event_subscriber.try_receive();

        let allocations = measure(|| {
            assert!(matches!(
                latest_publisher.try_publish(warm),
                LatestPublish::Published { .. }
            ));
            assert!(latest_subscriber.try_read_latest().is_some());
            assert!(event_publisher.try_publish(warm).is_ok());
            assert!(event_subscriber.try_receive().is_some());
        });
        assert_eq!(allocations.count_total, 0);
    }

    #[cfg(feature = "fault-injection")]
    #[test]
    fn latest_read_exhaustion_is_bounded_and_counted() {
        let (_temp, namespace, config) = setup();
        let name = ChannelName::new("read-race").unwrap();
        namespace.create_latest::<Payload>(name, config).unwrap();
        let _publisher = namespace
            .attach_latest_publisher(name, config, IdentityCodec::<Payload>::new())
            .unwrap();
        let mut subscriber = namespace
            .attach_latest_subscriber(name, config, IdentityCodec::<Payload>::new())
            .unwrap();
        subscriber.fault_force_unreadable_head_for_test(99);
        assert!(subscriber.try_read_latest().is_none());
        assert_eq!(subscriber.diagnostics().read_races, 1);
    }
}
