use std::path::Path;

use message_model::{
    ChassisSetpointIntent, NavigationAuthorityState, NavigationHealth, OdometryState,
};
use middleware_shm::{
    ChannelName, MemoryPolicy, ShmChannelConfig, ShmEventPublisher, ShmEventSubscriber,
    ShmLatestObserver, ShmLatestPublisher, ShmLatestSubscriber, ShmNamespace,
};

use crate::{
    ActionRequestRepr, ActionResponseRepr, NavigationActionRequest, NavigationActionRequestCodec,
    NavigationActionResponse, NavigationActionResponseCodec, NavigationAuthorityCodec,
    NavigationAuthorityRepr, NavigationHealthCodec, NavigationHealthRepr, NavigationIpcError,
    NavigationOdometryCodec, NavigationOdometryRepr, NavigationParticipationCodec,
    NavigationParticipationRepr, NavigationParticipationSnapshot, NavigationProposalCodec,
    NavigationProposalRepr, NavigationTfBatch, NavigationTfCodec, NavigationTfRepr,
    NavigationTransportDiagnosticsCodec, NavigationTransportDiagnosticsRepr,
    NavigationTransportDiagnosticsSnapshot,
};

/// Stable local file name for authority events.
pub const AUTHORITY_CHANNEL: &str = "navigation.authority.v1";
/// Stable local file name for action requests.
pub const ACTION_REQUEST_CHANNEL: &str = "navigation.action-request.v1";
/// Stable local file name for action responses.
pub const ACTION_RESPONSE_CHANNEL: &str = "navigation.action-response.v1";
/// Stable local file name for raw controller proposals.
pub const PROPOSAL_CHANNEL: &str = "navigation.proposal.v1";
/// Stable local file name for semantic odometry.
pub const ODOMETRY_CHANNEL: &str = "navigation.odometry.v1";
/// Stable local file name for the exact dynamic TF batch.
pub const DYNAMIC_TF_CHANNEL: &str = "navigation.dynamic-tf.v1";
/// Stable local file name for bridge health.
pub const HEALTH_CHANNEL: &str = "navigation.health.v1";
/// Stable local file name for the latest command-participation snapshot.
pub const PARTICIPATION_CHANNEL: &str = "navigation.participation.v1";
/// Stable local file name for the latest transport diagnostic snapshot.
pub const TRANSPORT_DIAGNOSTICS_CHANNEL: &str = "navigation.transport-diagnostics.v1";

/// Authority ordered publisher endpoint.
pub type AuthorityPublisher = ShmEventPublisher<NavigationAuthorityState, NavigationAuthorityCodec>;
/// Authority ordered subscriber endpoint.
pub type AuthoritySubscriber =
    ShmEventSubscriber<NavigationAuthorityState, NavigationAuthorityCodec>;
/// Action-request ordered publisher endpoint.
pub type ActionRequestPublisher =
    ShmEventPublisher<NavigationActionRequest, NavigationActionRequestCodec>;
/// Action-request ordered subscriber endpoint.
pub type ActionRequestSubscriber =
    ShmEventSubscriber<NavigationActionRequest, NavigationActionRequestCodec>;
/// Action-response ordered publisher endpoint.
pub type ActionResponsePublisher =
    ShmEventPublisher<NavigationActionResponse, NavigationActionResponseCodec>;
/// Action-response ordered subscriber endpoint.
pub type ActionResponseSubscriber =
    ShmEventSubscriber<NavigationActionResponse, NavigationActionResponseCodec>;
/// Latest controller-proposal publisher endpoint.
pub type ProposalPublisher = ShmLatestPublisher<ChassisSetpointIntent, NavigationProposalCodec>;
/// Latest controller-proposal subscriber endpoint.
pub type ProposalSubscriber = ShmLatestSubscriber<ChassisSetpointIntent, NavigationProposalCodec>;
/// Latest odometry publisher endpoint.
pub type OdometryPublisher = ShmLatestPublisher<OdometryState, NavigationOdometryCodec>;
/// Latest odometry subscriber endpoint.
pub type OdometrySubscriber = ShmLatestSubscriber<OdometryState, NavigationOdometryCodec>;
/// Latest dynamic-TF publisher endpoint.
pub type DynamicTfPublisher = ShmLatestPublisher<NavigationTfBatch, NavigationTfCodec>;
/// Latest dynamic-TF subscriber endpoint.
pub type DynamicTfSubscriber = ShmLatestSubscriber<NavigationTfBatch, NavigationTfCodec>;
/// Latest bridge-health publisher endpoint.
pub type HealthPublisher = ShmLatestPublisher<NavigationHealth, NavigationHealthCodec>;
/// Latest bridge-health subscriber endpoint.
pub type HealthSubscriber = ShmLatestSubscriber<NavigationHealth, NavigationHealthCodec>;
/// Latest movement-participation publisher endpoint.
pub type ParticipationPublisher =
    ShmLatestPublisher<NavigationParticipationSnapshot, NavigationParticipationCodec>;
/// Latest movement-participation subscriber endpoint.
pub type ParticipationSubscriber =
    ShmLatestSubscriber<NavigationParticipationSnapshot, NavigationParticipationCodec>;
/// Read-only diagnostic observer that never claims a subscriber lease.
pub type ParticipationObserver =
    ShmLatestObserver<NavigationParticipationSnapshot, NavigationParticipationCodec>;
/// Latest transport diagnostics publisher endpoint.
pub type TransportDiagnosticsPublisher =
    ShmLatestPublisher<NavigationTransportDiagnosticsSnapshot, NavigationTransportDiagnosticsCodec>;
/// Read-only transport diagnostics observer that never claims a subscriber lease.
pub type TransportDiagnosticsObserver =
    ShmLatestObserver<NavigationTransportDiagnosticsSnapshot, NavigationTransportDiagnosticsCodec>;

/// Exact motor-disabled navigation shared-memory namespace.
#[derive(Clone, Debug)]
pub struct NavigationIpcNamespace {
    inner: ShmNamespace,
}

impl NavigationIpcNamespace {
    /// Creates or validates an owner-only absolute namespace directory.
    pub fn new(root: impl AsRef<Path>) -> Result<Self, NavigationIpcError> {
        Ok(Self {
            inner: ShmNamespace::new(root)?,
        })
    }

    /// Creates every manifest-declared layout without claiming endpoints.
    pub fn create_layouts(&self) -> Result<(), NavigationIpcError> {
        self.inner.create_ordered::<NavigationAuthorityRepr>(
            channel(AUTHORITY_CHANNEL)?,
            ordered_config()?,
        )?;
        self.inner.create_ordered::<ActionRequestRepr>(
            channel(ACTION_REQUEST_CHANNEL)?,
            ordered_config()?,
        )?;
        self.inner.create_ordered::<ActionResponseRepr>(
            channel(ACTION_RESPONSE_CHANNEL)?,
            ordered_config()?,
        )?;
        self.inner.create_latest::<NavigationProposalRepr>(
            channel(PROPOSAL_CHANNEL)?,
            latest_config()?,
        )?;
        self.inner.create_latest::<NavigationOdometryRepr>(
            channel(ODOMETRY_CHANNEL)?,
            latest_config()?,
        )?;
        self.inner
            .create_latest::<NavigationTfRepr>(channel(DYNAMIC_TF_CHANNEL)?, latest_config()?)?;
        self.inner
            .create_latest::<NavigationHealthRepr>(channel(HEALTH_CHANNEL)?, latest_config()?)?;
        self.inner.create_latest::<NavigationParticipationRepr>(
            channel(PARTICIPATION_CHANNEL)?,
            latest_config()?,
        )?;
        self.inner
            .create_latest::<NavigationTransportDiagnosticsRepr>(
                channel(TRANSPORT_DIAGNOSTICS_CHANNEL)?,
                latest_config()?,
            )?;
        Ok(())
    }

    /// Claims the sole Rust authority publisher endpoint.
    pub fn attach_authority_publisher(&self) -> Result<AuthorityPublisher, NavigationIpcError> {
        Ok(self.inner.attach_event_publisher(
            channel(AUTHORITY_CHANNEL)?,
            ordered_config()?,
            NavigationAuthorityCodec::default(),
        )?)
    }

    /// Claims the sole bridge authority subscriber endpoint.
    pub fn attach_authority_subscriber(&self) -> Result<AuthoritySubscriber, NavigationIpcError> {
        Ok(self.inner.attach_event_subscriber(
            channel(AUTHORITY_CHANNEL)?,
            ordered_config()?,
            NavigationAuthorityCodec::default(),
        )?)
    }

    /// Claims the sole Rust action-request publisher endpoint.
    pub fn attach_action_request_publisher(
        &self,
    ) -> Result<ActionRequestPublisher, NavigationIpcError> {
        Ok(self.inner.attach_event_publisher(
            channel(ACTION_REQUEST_CHANNEL)?,
            ordered_config()?,
            NavigationActionRequestCodec::default(),
        )?)
    }

    /// Claims the sole bridge action-request subscriber endpoint.
    pub fn attach_action_request_subscriber(
        &self,
    ) -> Result<ActionRequestSubscriber, NavigationIpcError> {
        Ok(self.inner.attach_event_subscriber(
            channel(ACTION_REQUEST_CHANNEL)?,
            ordered_config()?,
            NavigationActionRequestCodec::default(),
        )?)
    }

    /// Claims the sole bridge action-response publisher endpoint.
    pub fn attach_action_response_publisher(
        &self,
    ) -> Result<ActionResponsePublisher, NavigationIpcError> {
        Ok(self.inner.attach_event_publisher(
            channel(ACTION_RESPONSE_CHANNEL)?,
            ordered_config()?,
            NavigationActionResponseCodec::default(),
        )?)
    }

    /// Claims the sole Rust action-response subscriber endpoint.
    pub fn attach_action_response_subscriber(
        &self,
    ) -> Result<ActionResponseSubscriber, NavigationIpcError> {
        Ok(self.inner.attach_event_subscriber(
            channel(ACTION_RESPONSE_CHANNEL)?,
            ordered_config()?,
            NavigationActionResponseCodec::default(),
        )?)
    }

    /// Claims the sole bridge proposal publisher endpoint.
    pub fn attach_proposal_publisher(&self) -> Result<ProposalPublisher, NavigationIpcError> {
        Ok(self.inner.attach_latest_publisher(
            channel(PROPOSAL_CHANNEL)?,
            latest_config()?,
            NavigationProposalCodec::default(),
        )?)
    }

    /// Claims the sole Rust proposal subscriber endpoint.
    pub fn attach_proposal_subscriber(&self) -> Result<ProposalSubscriber, NavigationIpcError> {
        Ok(self.inner.attach_latest_subscriber(
            channel(PROPOSAL_CHANNEL)?,
            latest_config()?,
            NavigationProposalCodec::default(),
        )?)
    }

    /// Claims the sole Rust odometry publisher endpoint.
    pub fn attach_odometry_publisher(&self) -> Result<OdometryPublisher, NavigationIpcError> {
        Ok(self.inner.attach_latest_publisher(
            channel(ODOMETRY_CHANNEL)?,
            latest_config()?,
            NavigationOdometryCodec::default(),
        )?)
    }

    /// Claims the sole bridge odometry subscriber endpoint.
    pub fn attach_odometry_subscriber(&self) -> Result<OdometrySubscriber, NavigationIpcError> {
        Ok(self.inner.attach_latest_subscriber(
            channel(ODOMETRY_CHANNEL)?,
            latest_config()?,
            NavigationOdometryCodec::default(),
        )?)
    }

    /// Claims the sole Rust dynamic-TF publisher endpoint.
    pub fn attach_dynamic_tf_publisher(&self) -> Result<DynamicTfPublisher, NavigationIpcError> {
        Ok(self.inner.attach_latest_publisher(
            channel(DYNAMIC_TF_CHANNEL)?,
            latest_config()?,
            NavigationTfCodec::default(),
        )?)
    }

    /// Claims the sole bridge dynamic-TF subscriber endpoint.
    pub fn attach_dynamic_tf_subscriber(&self) -> Result<DynamicTfSubscriber, NavigationIpcError> {
        Ok(self.inner.attach_latest_subscriber(
            channel(DYNAMIC_TF_CHANNEL)?,
            latest_config()?,
            NavigationTfCodec::default(),
        )?)
    }

    /// Claims the sole bridge health publisher endpoint.
    pub fn attach_health_publisher(&self) -> Result<HealthPublisher, NavigationIpcError> {
        Ok(self.inner.attach_latest_publisher(
            channel(HEALTH_CHANNEL)?,
            latest_config()?,
            NavigationHealthCodec::default(),
        )?)
    }

    /// Claims the sole Rust health subscriber endpoint.
    pub fn attach_health_subscriber(&self) -> Result<HealthSubscriber, NavigationIpcError> {
        Ok(self.inner.attach_latest_subscriber(
            channel(HEALTH_CHANNEL)?,
            latest_config()?,
            NavigationHealthCodec::default(),
        )?)
    }

    /// Claims the sole authority-owner participation publisher endpoint.
    pub fn attach_participation_publisher(
        &self,
    ) -> Result<ParticipationPublisher, NavigationIpcError> {
        Ok(self.inner.attach_latest_publisher(
            channel(PARTICIPATION_CHANNEL)?,
            latest_config()?,
            NavigationParticipationCodec::default(),
        )?)
    }

    /// Attaches the sole diagnostic participation subscriber endpoint.
    pub fn attach_participation_subscriber(
        &self,
    ) -> Result<ParticipationSubscriber, NavigationIpcError> {
        Ok(self.inner.attach_latest_subscriber(
            channel(PARTICIPATION_CHANNEL)?,
            latest_config()?,
            NavigationParticipationCodec::default(),
        )?)
    }

    /// Attaches a read-only observer without changing shared endpoint state.
    pub fn attach_participation_observer(
        &self,
    ) -> Result<ParticipationObserver, NavigationIpcError> {
        Ok(self.inner.attach_latest_observer(
            channel(PARTICIPATION_CHANNEL)?,
            latest_config()?,
            NavigationParticipationCodec::default(),
        )?)
    }

    /// Claims the sole transport diagnostic publisher endpoint.
    pub fn attach_transport_diagnostics_publisher(
        &self,
    ) -> Result<TransportDiagnosticsPublisher, NavigationIpcError> {
        Ok(self.inner.attach_latest_publisher(
            channel(TRANSPORT_DIAGNOSTICS_CHANNEL)?,
            latest_config()?,
            NavigationTransportDiagnosticsCodec::default(),
        )?)
    }

    /// Attaches a read-only transport observer without changing endpoint state.
    pub fn attach_transport_diagnostics_observer(
        &self,
    ) -> Result<TransportDiagnosticsObserver, NavigationIpcError> {
        Ok(self.inner.attach_latest_observer(
            channel(TRANSPORT_DIAGNOSTICS_CHANNEL)?,
            latest_config()?,
            NavigationTransportDiagnosticsCodec::default(),
        )?)
    }
}

fn channel(value: &str) -> Result<ChannelName, NavigationIpcError> {
    Ok(ChannelName::new(value)?)
}

fn ordered_config() -> Result<ShmChannelConfig, NavigationIpcError> {
    Ok(ShmChannelConfig::new(8, MemoryPolicy::Prefault)?)
}

fn latest_config() -> Result<ShmChannelConfig, NavigationIpcError> {
    // Latest delivery remains one logical value. Layout v1 uses a three-slot
    // seqlock window so a reader never consumes a slot being overwritten.
    Ok(ShmChannelConfig::new(3, MemoryPolicy::Prefault)?)
}
