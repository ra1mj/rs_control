//! Motor-disabled navigation integration composition root.

use std::{error::Error, path::PathBuf, process::ExitCode, thread, time::Duration};

use arrayvec::ArrayVec;
use chassis_model::BodyPose2;
use clap::{Parser, Subcommand};
use contract_codec::encode_navigate_to_pose_goal;
use control_core::{
    BehaviorTree, BehaviorTreeActivationId, BehaviorTreeActivationState, BehaviorTreeBuilder,
    BehaviorTreeEffect, BehaviorTreeEvent, BehaviorTreeNodeId, BehaviorTreeNodeName,
    BehaviorTreeOutcome, BehaviorTreeRegistry, BehaviorTreeResources, BehaviorTreeRuntime,
    BehaviorTreeRuntimeState, TypedBehaviorTreeAction,
};
use message_model::{
    ActionCancelDecision, ActionCancelRequest, ActionGoalIntent, ActionName, ActionStatus, AsciiId,
    ChassisSetpointIntent, ClockState, ClockStatus, CommandIntent, CommandIntentFields,
    DataQuality, Header, HeaderFields, Id128, NavigateToPoseGoal, NavigationMapIdentity, Pose3,
    RegisteredPayload, ResourceId, Sha256Identity, Twist2, UnitQuaternion, Vector3,
};
use middleware_api::{
    ChannelCounters, EventPublishError, EventPublisher, EventSubscriber, LatestPublisher,
    LatestSubscriber,
};
use navigation_ipc::{
    ACTION_REQUEST_CHANNEL, ACTION_RESPONSE_CHANNEL, AUTHORITY_CHANNEL, DYNAMIC_TF_CHANNEL,
    HEALTH_CHANNEL, NAVIGATE_TO_POSE_ACTION, NAVIGATE_TO_POSE_GOAL_TYPE, NavigationActionRequest,
    NavigationActionResponseKind, NavigationIpcNamespace, NavigationParticipant,
    NavigationParticipantReason, NavigationParticipantState, NavigationParticipationSnapshot,
    NavigationShmDiagnostics, NavigationTransportChannelSnapshot,
    NavigationTransportDiagnosticsSnapshot, ODOMETRY_CHANNEL, PARTICIPATION_CHANNEL,
    PROPOSAL_CHANNEL, ParticipationPublisher, TransportDiagnosticsPublisher,
};
use navigation_plugin_config::NavigationPluginManifest;
use rs_navigation_e2e::{MAX_PROPOSAL_TTL_NS, MotorDisabledHarness, StopReason};
use rustix::time::{ClockId, clock_gettime};

const MOTOR_DISABLED_TICK: Duration = Duration::from_millis(5);
const NAV2_CONTROLLER_PERIOD_S: f64 = 0.05;
const MIN_PROPOSAL_STEP_NS: u64 = 1_000_000;
const NAVIGATE_TO_POSE_TIMEOUT_NS: u64 = 30_000_000_000;
const PARTICIPATION_VALID_FOR_NS: u64 = 250_000_000;
const PARTICIPATION_PERIOD_NS: u64 = 50_000_000;
const LOCALIZATION_LOSS_RESULT_CODE: u32 = 6;

#[derive(Clone, Debug)]
struct PendingGoal {
    activation: BehaviorTreeActivationId,
    request: NavigationActionRequest,
}

#[derive(Clone, Copy, Debug)]
struct ArmedDeadline {
    activation: BehaviorTreeActivationId,
    expires_monotonic_ns: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
struct ParticipantStage {
    state: NavigationParticipantState,
    reason: NavigationParticipantReason,
}

impl ParticipantStage {
    const fn new(state: NavigationParticipantState, reason: NavigationParticipantReason) -> Self {
        Self { state, reason }
    }
}

struct NavigationParticipation {
    source_id: AsciiId<64>,
    boot_id: Id128,
    process_epoch: u64,
    command_id: Id128,
    sequence: u64,
    tree_root: ParticipantStage,
    tree_navigate_to_pose: ParticipantStage,
    action: ParticipantStage,
    bridge: ParticipantStage,
    authority: ParticipantStage,
    proposal: ParticipantStage,
    final_limiter: ParticipantStage,
    mecanum: ParticipantStage,
    fake_sink: ParticipantStage,
}

impl NavigationParticipation {
    fn new(process_epoch: u64, boot_id: Id128, command_id: Id128) -> Self {
        let inactive = ParticipantStage::new(
            NavigationParticipantState::Inactive,
            NavigationParticipantReason::None,
        );
        Self {
            source_id: AsciiId::parse("rs-navigation-e2e").expect("static inspection source"),
            boot_id,
            process_epoch,
            command_id,
            sequence: 0,
            tree_root: inactive,
            tree_navigate_to_pose: inactive,
            action: inactive,
            bridge: inactive,
            authority: ParticipantStage::new(
                NavigationParticipantState::Inactive,
                NavigationParticipantReason::AuthorityUnavailable,
            ),
            proposal: ParticipantStage::new(
                NavigationParticipantState::Inactive,
                NavigationParticipantReason::ProposalUnavailable,
            ),
            final_limiter: ParticipantStage::new(
                NavigationParticipantState::Inactive,
                NavigationParticipantReason::OutputStopped,
            ),
            mecanum: ParticipantStage::new(
                NavigationParticipantState::Inactive,
                NavigationParticipantReason::OutputStopped,
            ),
            fake_sink: ParticipantStage::new(
                NavigationParticipantState::Inactive,
                NavigationParticipantReason::OutputStopped,
            ),
        }
    }

    fn update_tree(&mut self, runtime: &BehaviorTreeRuntime<'_>) -> bool {
        let snapshot = runtime.snapshot();
        let stage = snapshot
            .activations()
            .iter()
            .find(|activation| activation.name().as_str() == "navigate_to_pose")
            .map_or_else(
                || stage_from_runtime(snapshot.state()),
                |activation| stage_from_activation(activation.state()),
            );
        replace_stage(&mut self.tree_root, stage)
            | replace_stage(&mut self.tree_navigate_to_pose, stage)
    }

    fn dispatched(&mut self) -> bool {
        replace_stage(
            &mut self.action,
            ParticipantStage::new(
                NavigationParticipantState::Pending,
                NavigationParticipantReason::WaitingAcknowledgement,
            ),
        ) | replace_stage(
            &mut self.bridge,
            ParticipantStage::new(
                NavigationParticipantState::Pending,
                NavigationParticipantReason::WaitingAcknowledgement,
            ),
        )
    }

    fn acknowledgement(&mut self, accepted: bool) -> bool {
        let stage = if accepted {
            ParticipantStage::new(
                NavigationParticipantState::Active,
                NavigationParticipantReason::GoalAccepted,
            )
        } else {
            ParticipantStage::new(
                NavigationParticipantState::Terminal,
                NavigationParticipantReason::GoalRejected,
            )
        };
        replace_stage(&mut self.action, stage) | replace_stage(&mut self.bridge, stage)
    }

    fn executing(&mut self) -> bool {
        let stage = ParticipantStage::new(
            NavigationParticipantState::Active,
            NavigationParticipantReason::Executing,
        );
        replace_stage(&mut self.action, stage) | replace_stage(&mut self.bridge, stage)
    }

    fn cancellation_requested(&mut self) -> bool {
        let stage = ParticipantStage::new(
            NavigationParticipantState::Pending,
            NavigationParticipantReason::CancelRequested,
        );
        replace_stage(&mut self.action, stage) | replace_stage(&mut self.bridge, stage)
    }

    fn cancellation_decision(&mut self, accepted: bool) -> bool {
        let stage = if accepted {
            ParticipantStage::new(
                NavigationParticipantState::Active,
                NavigationParticipantReason::CancelAccepted,
            )
        } else {
            ParticipantStage::new(
                NavigationParticipantState::Active,
                NavigationParticipantReason::Executing,
            )
        };
        replace_stage(&mut self.action, stage) | replace_stage(&mut self.bridge, stage)
    }

    fn deadline_elapsed(&mut self) -> bool {
        let stage = ParticipantStage::new(
            NavigationParticipantState::Pending,
            NavigationParticipantReason::TimedOut,
        );
        replace_stage(&mut self.action, stage) | replace_stage(&mut self.bridge, stage)
    }

    fn terminal(&mut self, status: ActionStatus) -> bool {
        let stage = ParticipantStage::new(
            NavigationParticipantState::Terminal,
            reason_from_status(status),
        );
        replace_stage(&mut self.action, stage) | replace_stage(&mut self.bridge, stage)
    }

    fn bridge_lost(&mut self) -> bool {
        replace_stage(
            &mut self.bridge,
            ParticipantStage::new(
                NavigationParticipantState::Faulted,
                NavigationParticipantReason::PeerUnavailable,
            ),
        )
    }

    fn stopped(&mut self, reason: StopReason) -> bool {
        let mut changed = false;
        match reason {
            StopReason::ProposalExpired => {
                changed |= replace_stage(
                    &mut self.proposal,
                    ParticipantStage::new(
                        NavigationParticipantState::Inactive,
                        NavigationParticipantReason::ProposalUnavailable,
                    ),
                );
            }
            StopReason::AuthorityUnavailable => {
                changed |= replace_stage(
                    &mut self.authority,
                    ParticipantStage::new(
                        NavigationParticipantState::Faulted,
                        NavigationParticipantReason::AuthorityUnavailable,
                    ),
                );
                changed |= replace_stage(
                    &mut self.proposal,
                    ParticipantStage::new(
                        NavigationParticipantState::Inactive,
                        NavigationParticipantReason::ProposalUnavailable,
                    ),
                );
            }
            StopReason::PeerUnavailable => {
                changed |= replace_stage(
                    &mut self.proposal,
                    ParticipantStage::new(
                        NavigationParticipantState::Faulted,
                        NavigationParticipantReason::PeerUnavailable,
                    ),
                );
            }
        }
        let stopped = ParticipantStage::new(
            NavigationParticipantState::Inactive,
            NavigationParticipantReason::OutputStopped,
        );
        changed |= replace_stage(&mut self.final_limiter, stopped);
        changed |= replace_stage(&mut self.mecanum, stopped);
        changed |= replace_stage(&mut self.fake_sink, stopped);
        changed
    }

    fn publish_best_effort(&mut self, publisher: &mut ParticipationPublisher, now: u64) {
        self.sequence = self.sequence.saturating_add(1);
        let Ok(header) = local_header(
            self.source_id,
            self.boot_id,
            self.sequence,
            now,
            PARTICIPATION_VALID_FOR_NS,
        ) else {
            return;
        };
        let mut participants = ArrayVec::new();
        for (id, stage) in [
            ("tree.root", self.tree_root),
            ("tree.navigate_to_pose", self.tree_navigate_to_pose),
            ("navigation.action", self.action),
            ("navigation.bridge", self.bridge),
            ("control.authority", self.authority),
            ("control.proposal", self.proposal),
            ("control.final_limiter", self.final_limiter),
            ("control.mecanum", self.mecanum),
            ("output.fake_sink", self.fake_sink),
        ] {
            participants
                .try_push(NavigationParticipant::new(
                    AsciiId::parse(id).expect("static participation ID"),
                    stage.state,
                    stage.reason,
                ))
                .expect("fixed participant set fits its bounded snapshot");
        }
        let Ok(snapshot) = NavigationParticipationSnapshot::new(
            header,
            self.process_epoch,
            Some(self.command_id),
            participants,
        ) else {
            return;
        };
        // Inspection is deliberately best effort; this result has no effect on
        // action admission, authority, limiting, or the motor-disabled sink.
        let _ = publisher.try_publish_without_subscriber(snapshot);
    }
}

fn publish_transport_diagnostics(
    publisher: &mut TransportDiagnosticsPublisher,
    now: u64,
    process_epoch: u64,
    boot_id: Id128,
    command_id: Id128,
    sequence: &mut u64,
    observations: [(&'static str, ChannelCounters, NavigationShmDiagnostics); 8],
) {
    *sequence = sequence.saturating_add(1);
    let Ok(header) = local_header(
        AsciiId::parse("rs-navigation-e2e").expect("static diagnostic source"),
        boot_id,
        *sequence,
        now,
        PARTICIPATION_VALID_FOR_NS,
    ) else {
        return;
    };
    let mut channels = ArrayVec::new();
    for (channel_id, counters, diagnostics) in observations {
        channels
            .try_push(NavigationTransportChannelSnapshot::new(
                AsciiId::parse(channel_id).expect("static navigation channel ID"),
                counters,
                diagnostics,
            ))
            .expect("fixed navigation channel set fits its bounded snapshot");
    }
    let Ok(snapshot) = NavigationTransportDiagnosticsSnapshot::new(
        header,
        process_epoch,
        Some(command_id),
        channels,
    ) else {
        return;
    };
    // Transport observation is best effort and cannot affect navigation work.
    let _ = publisher.try_publish_without_subscriber(snapshot);
}

fn replace_stage(slot: &mut ParticipantStage, next: ParticipantStage) -> bool {
    let changed = *slot != next;
    *slot = next;
    changed
}

fn stage_from_activation(state: BehaviorTreeActivationState) -> ParticipantStage {
    match state {
        BehaviorTreeActivationState::Entering | BehaviorTreeActivationState::WaitingDispatch => {
            ParticipantStage::new(
                NavigationParticipantState::Pending,
                NavigationParticipantReason::WaitingDispatch,
            )
        }
        BehaviorTreeActivationState::WaitingAcknowledgement => ParticipantStage::new(
            NavigationParticipantState::Pending,
            NavigationParticipantReason::WaitingAcknowledgement,
        ),
        BehaviorTreeActivationState::Running => ParticipantStage::new(
            NavigationParticipantState::Active,
            NavigationParticipantReason::Executing,
        ),
        BehaviorTreeActivationState::CancelRequested => ParticipantStage::new(
            NavigationParticipantState::Pending,
            NavigationParticipantReason::CancelRequested,
        ),
        BehaviorTreeActivationState::Canceling => ParticipantStage::new(
            NavigationParticipantState::Active,
            NavigationParticipantReason::CancelAccepted,
        ),
        BehaviorTreeActivationState::WaitingCondition
        | BehaviorTreeActivationState::WaitingChildren => ParticipantStage::new(
            NavigationParticipantState::Pending,
            NavigationParticipantReason::None,
        ),
    }
}

fn stage_from_runtime(state: BehaviorTreeRuntimeState) -> ParticipantStage {
    match state {
        BehaviorTreeRuntimeState::Idle => ParticipantStage::new(
            NavigationParticipantState::Inactive,
            NavigationParticipantReason::None,
        ),
        BehaviorTreeRuntimeState::Running => ParticipantStage::new(
            NavigationParticipantState::Pending,
            NavigationParticipantReason::WaitingDispatch,
        ),
        BehaviorTreeRuntimeState::Terminal(outcome) => ParticipantStage::new(
            NavigationParticipantState::Terminal,
            reason_from_outcome(outcome),
        ),
        BehaviorTreeRuntimeState::Faulted(_) => ParticipantStage::new(
            NavigationParticipantState::Faulted,
            NavigationParticipantReason::Failed,
        ),
    }
}

const fn reason_from_status(status: ActionStatus) -> NavigationParticipantReason {
    match status {
        ActionStatus::Accepted | ActionStatus::Executing | ActionStatus::Canceling => {
            NavigationParticipantReason::Executing
        }
        ActionStatus::Succeeded => NavigationParticipantReason::Succeeded,
        ActionStatus::Failed => NavigationParticipantReason::Failed,
        ActionStatus::Cancelled => NavigationParticipantReason::Cancelled,
        ActionStatus::TimedOut => NavigationParticipantReason::TimedOut,
        ActionStatus::Rejected => NavigationParticipantReason::GoalRejected,
    }
}

const fn reason_from_outcome(outcome: BehaviorTreeOutcome) -> NavigationParticipantReason {
    match outcome {
        BehaviorTreeOutcome::Succeeded => NavigationParticipantReason::Succeeded,
        BehaviorTreeOutcome::Failed => NavigationParticipantReason::Failed,
        BehaviorTreeOutcome::TimedOut => NavigationParticipantReason::TimedOut,
        BehaviorTreeOutcome::Cancelled => NavigationParticipantReason::Cancelled,
        BehaviorTreeOutcome::Rejected => NavigationParticipantReason::GoalRejected,
    }
}

#[derive(Debug, Parser)]
#[command(
    name = "rs-navigation-e2e",
    version,
    about = "Motor-disabled navigation authority/fake-output harness"
)]
struct Cli {
    /// Exact plugin manifest path.
    #[arg(long)]
    manifest: PathBuf,
    /// Expected complete manifest SHA-256.
    #[arg(long)]
    expected_sha256: String,
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Validate the hard output gate and run one deterministic proposal/stop loop.
    SelfTest,
    /// Create every fixed shared-memory channel without attaching peers.
    InitIpc {
        /// Absolute owner-only runtime namespace.
        #[arg(long)]
        ipc_root: PathBuf,
    },
    /// Run the complete motor-disabled IPC authority/control/feedback loop.
    Run {
        /// Absolute owner-only runtime namespace.
        #[arg(long)]
        ipc_root: PathBuf,
        /// Supervisor-owned nonzero process epoch regenerated on restart.
        #[arg(long)]
        process_epoch: u64,
        /// Request cancellation after this many milliseconds; omitted runs to result.
        #[arg(long)]
        cancel_after_ms: Option<u64>,
        /// Exit after the first correlated terminal result (test orchestration only).
        #[arg(long, default_value_t = false)]
        exit_on_terminal: bool,
        /// Withhold odometry/TF after the first proposal (fault injection only).
        #[arg(long)]
        stop_feedback_after_first_proposal_ms: Option<u64>,
    },
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("rs-navigation-e2e: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run(cli: Cli) -> Result<(), Box<dyn Error>> {
    let manifest = NavigationPluginManifest::load(&cli.manifest)?;
    if manifest.fingerprint().to_string() != cli.expected_sha256 {
        return Err("manifest SHA-256 does not match --expected-sha256".into());
    }
    match cli.command {
        Command::SelfTest => self_test()?,
        Command::InitIpc { ipc_root } => {
            NavigationIpcNamespace::new(ipc_root)?.create_layouts()?;
            println!(
                "motor_output=forbidden can_transmit=forbidden ipc_namespace={}",
                manifest.ipc_namespace()
            );
        }
        Command::Run {
            ipc_root,
            process_epoch,
            cancel_after_ms,
            exit_on_terminal,
            stop_feedback_after_first_proposal_ms,
        } => run_ipc(
            &manifest,
            ipc_root,
            process_epoch,
            cancel_after_ms,
            exit_on_terminal,
            stop_feedback_after_first_proposal_ms,
        )?,
    }
    Ok(())
}

fn self_test() -> Result<(), Box<dyn Error>> {
    let now = monotonic_ns();
    let bridge_source = AsciiId::parse("navigation-bridge")?;
    let bridge_boot = boot_id(now, 0x42)?;
    let mut harness =
        MotorDisabledHarness::new(now, 1_000_000_000, boot_id(now, 0x24)?, bridge_source)?;
    let lease = harness
        .active_lease(now)
        .ok_or("synthetic authority unavailable")?;
    let command_id = boot_id(now, 0x66)?;
    let proposal = ChassisSetpointIntent::new(
        CommandIntent::new(CommandIntentFields {
            header: Header::new(HeaderFields {
                source_id: bridge_source,
                boot_id: bridge_boot,
                sequence: 1,
                source_monotonic_ns: now,
                publish_unix_ns: None,
                acquisition_timestamp: None,
                valid_for_ns: MAX_PROPOSAL_TTL_NS,
                frame_id: None,
                clock_status: ClockStatus {
                    state: ClockState::Local,
                    uncertainty_ns: 0,
                },
                quality: DataQuality::Valid,
            })?,
            resource_id: lease.resource_id(),
            owner_id: lease.owner_id(),
            lease_id: lease.lease_id(),
            session_epoch: lease.session_epoch(),
            command_id,
            requested_ttl_ns: MAX_PROPOSAL_TTL_NS,
        })?,
        Twist2::new(0.4, -0.1, 0.2)?,
    );
    let output = harness.accept_proposal(proposal, now, 0.02)?;
    let _ = harness.feedback(1, now)?;
    let stop_at = now.saturating_add(MAX_PROPOSAL_TTL_NS);
    let reason = harness.tick(stop_at)?.ok_or("proposal did not expire")?;
    println!(
        "motor_output=forbidden command_id={:02x?} body={:?} stop={reason:?}",
        output.command_id.as_bytes(),
        output.body.components()
    );
    Ok(())
}

fn run_ipc(
    manifest: &NavigationPluginManifest,
    ipc_root: PathBuf,
    process_epoch: u64,
    cancel_after_ms: Option<u64>,
    exit_on_terminal: bool,
    stop_feedback_after_first_proposal_ms: Option<u64>,
) -> Result<(), Box<dyn Error>> {
    if process_epoch == 0 {
        return Err("--process-epoch must be nonzero".into());
    }
    let start_ns = monotonic_ns();
    let authority_boot = boot_id(process_epoch, 0x24)?;
    let command_id = boot_id(process_epoch, 0x66)?;
    let mut harness = MotorDisabledHarness::new_at_pose(
        start_ns,
        86_400_000_000_000,
        authority_boot,
        AsciiId::parse("navigation-bridge")?,
        BodyPose2::new(1.5, -1.5, 0.0)?,
    )?;
    let lease = harness
        .active_lease(start_ns)
        .ok_or("synthetic authority unavailable")?;

    let ipc = NavigationIpcNamespace::new(ipc_root)?;
    ipc.create_layouts()?;
    let mut authority_publisher = ipc.attach_authority_publisher()?;
    let mut action_request_publisher = ipc.attach_action_request_publisher()?;
    let mut action_response_subscriber = ipc.attach_action_response_subscriber()?;
    let mut proposal_subscriber = ipc.attach_proposal_subscriber()?;
    let mut odometry_publisher = ipc.attach_odometry_publisher()?;
    let mut tf_publisher = ipc.attach_dynamic_tf_publisher()?;
    let mut health_subscriber = ipc.attach_health_subscriber()?;
    let mut participation_publisher = ipc.attach_participation_publisher()?;
    let mut transport_diagnostics_publisher = ipc.attach_transport_diagnostics_publisher()?;

    let goal = canonical_goal(manifest)?;
    let goal_payload = RegisteredPayload::new(
        AsciiId::parse(NAVIGATE_TO_POSE_GOAL_TYPE)?,
        encode_navigate_to_pose_goal(goal),
    )?;
    let goal_intent = ActionGoalIntent::new(
        CommandIntent::new(CommandIntentFields {
            header: local_header(
                AsciiId::parse("rs-navigation-e2e")?,
                authority_boot,
                1,
                start_ns,
                1_000_000_000,
            )?,
            resource_id: lease.resource_id(),
            owner_id: lease.owner_id(),
            lease_id: lease.lease_id(),
            session_epoch: lease.session_epoch(),
            command_id,
            requested_ttl_ns: 1_000_000_000,
        })?,
        AsciiId::parse(NAVIGATE_TO_POSE_ACTION)?,
        goal_payload,
    );
    let tree = navigation_behavior_tree()?;
    let mut runtime = BehaviorTreeRuntime::new(&tree);
    let mut participation = NavigationParticipation::new(process_epoch, authority_boot, command_id);
    let source_id = AsciiId::parse("rs-navigation-e2e")?;
    let mut request_sequence = 1_u64;
    let mut pending_goal = None;
    let mut pending_cancel = None;
    let mut deadline = None;
    let mut cancel_all_requested = false;
    let mut goal_sent_at_ns = None;
    let mut terminal = false;
    let mut feedback_sequence = 0_u64;
    let mut proposal_peer_was_alive = false;
    let mut first_proposal_at_ns = None;
    let mut last_proposal_at_ns = None;
    let mut proposal_accepted_logged = false;
    let mut feedback_unavailable_logged = false;
    let mut participation_dirty = apply_tree_effects(
        runtime.advance(BehaviorTreeEvent::Start).effects(),
        start_ns,
        command_id,
        &goal_intent,
        source_id,
        authority_boot,
        &mut request_sequence,
        &mut pending_goal,
        &mut pending_cancel,
        &mut deadline,
        &mut participation,
    )?;
    participation_dirty |= participation.update_tree(&runtime);
    let mut last_participation_publish_ns = 0_u64;
    let mut transport_diagnostics_sequence = 0_u64;

    println!(
        "motor_output=forbidden can_transmit=forbidden process_epoch={process_epoch} command_id={:02x?}",
        command_id.as_bytes()
    );
    loop {
        let now = monotonic_ns();
        if let Some(pending) = pending_goal.take() {
            match action_request_publisher.try_publish(pending.request) {
                Ok(_) => {
                    goal_sent_at_ns = Some(now);
                    participation_dirty |= participation.dispatched();
                    let step = runtime.advance(BehaviorTreeEvent::ActionDispatched {
                        activation: pending.activation,
                        command_id,
                    });
                    participation_dirty |= apply_tree_effects(
                        step.effects(),
                        now,
                        command_id,
                        &goal_intent,
                        source_id,
                        authority_boot,
                        &mut request_sequence,
                        &mut pending_goal,
                        &mut pending_cancel,
                        &mut deadline,
                        &mut participation,
                    )?;
                }
                Err(
                    EventPublishError::Full(request) | EventPublishError::Disconnected(request),
                ) => {
                    pending_goal = Some(PendingGoal {
                        activation: pending.activation,
                        request,
                    });
                }
            }
        }

        if !cancel_all_requested
            && !terminal
            && cancel_after_ms.is_some_and(|delay_ms| {
                goal_sent_at_ns.is_some_and(|sent_ns| {
                    now.saturating_sub(sent_ns) >= delay_ms.saturating_mul(1_000_000)
                })
            })
        {
            cancel_all_requested = true;
            let step = runtime.advance(BehaviorTreeEvent::CancelAll);
            participation_dirty |= apply_tree_effects(
                step.effects(),
                now,
                command_id,
                &goal_intent,
                source_id,
                authority_boot,
                &mut request_sequence,
                &mut pending_goal,
                &mut pending_cancel,
                &mut deadline,
                &mut participation,
            )?;
        }
        if let Some(request) = pending_cancel.take() {
            match action_request_publisher.try_publish(request) {
                Ok(_) => {
                    participation_dirty |= participation.cancellation_requested();
                }
                Err(
                    EventPublishError::Full(request) | EventPublishError::Disconnected(request),
                ) => {
                    pending_cancel = Some(request);
                }
            }
        }

        if deadline.is_some_and(|armed: ArmedDeadline| now >= armed.expires_monotonic_ns) {
            let armed = deadline.take().expect("checked deadline remains present");
            participation_dirty |= participation.deadline_elapsed();
            let step = runtime.advance(BehaviorTreeEvent::DeadlineElapsed {
                activation: armed.activation,
            });
            participation_dirty |= apply_tree_effects(
                step.effects(),
                now,
                command_id,
                &goal_intent,
                source_id,
                authority_boot,
                &mut request_sequence,
                &mut pending_goal,
                &mut pending_cancel,
                &mut deadline,
                &mut participation,
            )?;
        }

        while let Some(sample) = action_response_subscriber.try_receive() {
            match sample.value.kind() {
                NavigationActionResponseKind::Acknowledgement(acknowledgement) => {
                    if acknowledgement.command_id == command_id {
                        let accepted = acknowledgement.accepted();
                        participation_dirty |= participation.acknowledgement(accepted);
                        if !accepted {
                            terminal = true;
                            deadline = None;
                            println!(
                                "terminal command_id={:02x?} status={:?} code={}",
                                command_id.as_bytes(),
                                ActionStatus::Rejected,
                                acknowledgement.result_code
                            );
                        }
                        let step = runtime.advance(BehaviorTreeEvent::ActionAcknowledged {
                            command_id,
                            accepted,
                        });
                        participation_dirty |= apply_tree_effects(
                            step.effects(),
                            now,
                            command_id,
                            &goal_intent,
                            source_id,
                            authority_boot,
                            &mut request_sequence,
                            &mut pending_goal,
                            &mut pending_cancel,
                            &mut deadline,
                            &mut participation,
                        )?;
                    }
                }
                NavigationActionResponseKind::Feedback(feedback) => {
                    if feedback.command_id == command_id && !terminal {
                        participation_dirty |= participation.executing();
                    }
                }
                NavigationActionResponseKind::Cancel(response) => {
                    if response.command_id == command_id && !terminal {
                        println!(
                            "cancel command_id={:02x?} decision={:?} code={}",
                            command_id.as_bytes(),
                            response.decision,
                            response.result_code
                        );
                        participation_dirty |= participation.cancellation_decision(
                            response.decision == ActionCancelDecision::Accepted,
                        );
                        let step = runtime.advance(BehaviorTreeEvent::CancelDecision {
                            command_id,
                            decision: response.decision,
                        });
                        participation_dirty |= apply_tree_effects(
                            step.effects(),
                            now,
                            command_id,
                            &goal_intent,
                            source_id,
                            authority_boot,
                            &mut request_sequence,
                            &mut pending_goal,
                            &mut pending_cancel,
                            &mut deadline,
                            &mut participation,
                        )?;
                    }
                }
                NavigationActionResponseKind::Result(result) => {
                    if result.command_id == command_id && !terminal {
                        terminal = true;
                        deadline = None;
                        participation_dirty |= participation.terminal(result.status);
                        println!(
                            "terminal command_id={:02x?} status={:?} code={}",
                            command_id.as_bytes(),
                            result.status,
                            result.result_code
                        );
                        let step = runtime.advance(BehaviorTreeEvent::ActionTerminal {
                            command_id,
                            status: result.status,
                        });
                        participation_dirty |= apply_tree_effects(
                            step.effects(),
                            now,
                            command_id,
                            &goal_intent,
                            source_id,
                            authority_boot,
                            &mut request_sequence,
                            &mut pending_goal,
                            &mut pending_cancel,
                            &mut deadline,
                            &mut participation,
                        )?;
                    }
                }
            }
        }

        let active_command_id = executing_tree_command(&runtime);
        let authority = harness.authority_state(now, active_command_id)?;
        let _ = authority_publisher.try_publish(authority);
        participation_dirty |= replace_stage(
            &mut participation.authority,
            if active_command_id.is_some() && harness.active_lease(now).is_some() {
                ParticipantStage::new(
                    NavigationParticipantState::Active,
                    NavigationParticipantReason::AuthorityAvailable,
                )
            } else {
                ParticipantStage::new(
                    NavigationParticipantState::Inactive,
                    NavigationParticipantReason::AuthorityUnavailable,
                )
            },
        );

        if let Some(sample) = proposal_subscriber.try_read_latest() {
            let dt_s = proposal_dt_s(last_proposal_at_ns, now);
            if harness.accept_proposal(sample.value, now, dt_s).is_ok() {
                let _ = first_proposal_at_ns.get_or_insert(now);
                last_proposal_at_ns = Some(now);
                if !proposal_accepted_logged {
                    println!("proposal=Accepted motor_output=forbidden");
                    proposal_accepted_logged = true;
                }
                participation_dirty |= replace_stage(
                    &mut participation.proposal,
                    ParticipantStage::new(
                        NavigationParticipantState::Active,
                        NavigationParticipantReason::ProposalAccepted,
                    ),
                );
                participation_dirty |= replace_stage(
                    &mut participation.final_limiter,
                    ParticipantStage::new(
                        NavigationParticipantState::Active,
                        NavigationParticipantReason::FinalLimited,
                    ),
                );
                participation_dirty |= replace_stage(
                    &mut participation.mecanum,
                    ParticipantStage::new(
                        NavigationParticipantState::Active,
                        NavigationParticipantReason::FinalLimited,
                    ),
                );
                participation_dirty |= replace_stage(
                    &mut participation.fake_sink,
                    ParticipantStage::new(
                        NavigationParticipantState::Active,
                        NavigationParticipantReason::FakeOutputApplied,
                    ),
                );
            }
        }
        let proposal_peer_alive = proposal_subscriber.refresh_peer_liveness().unwrap_or(false);
        if proposal_peer_was_alive && !proposal_peer_alive {
            harness.peer_lost(now)?;
            println!("stop=PeerUnavailable motor_output=forbidden");
            participation_dirty |= participation.bridge_lost();
            participation_dirty |= participation.stopped(StopReason::PeerUnavailable);
        }
        proposal_peer_was_alive = proposal_peer_alive;
        if let Some(reason) = harness.tick(now)? {
            if reason == StopReason::ProposalExpired {
                let proposal_age_us = last_proposal_at_ns
                    .map_or(0, |accepted_ns| now.saturating_sub(accepted_ns) / 1_000);
                println!(
                    "stop={reason:?} motor_output=forbidden proposal_age_us={proposal_age_us}"
                );
            } else {
                println!("stop={reason:?} motor_output=forbidden");
            }
            participation_dirty |= participation.stopped(reason);
        }

        let feedback_available = !stop_feedback_after_first_proposal_ms.is_some_and(|delay_ms| {
            first_proposal_at_ns.is_some_and(|started_ns| {
                now.saturating_sub(started_ns) >= delay_ms.saturating_mul(1_000_000)
            })
        });
        if feedback_available {
            feedback_sequence = feedback_sequence.saturating_add(1);
            let (odometry, tf) = harness.feedback(feedback_sequence, now)?;
            let _ = odometry_publisher.try_publish(odometry);
            let _ = tf_publisher.try_publish(tf);
        } else if !feedback_unavailable_logged {
            println!("feedback=Unavailable reason=InjectedLocalizationLoss");
            feedback_unavailable_logged = true;
            if !terminal {
                // Localized motion feedback is a prerequisite for the project
                // action, independent of Nav2's eventual cancellation response.
                // Stop output first, request remote cleanup, then publish an exact
                // local terminal failure so the outer tree cannot remain active.
                queue_action_cancel(
                    now,
                    command_id,
                    source_id,
                    authority_boot,
                    &mut request_sequence,
                    &mut pending_cancel,
                )?;
                terminal = true;
                deadline = None;
                participation_dirty |= participation.terminal(ActionStatus::Failed);
                println!(
                    "terminal command_id={:02x?} status={:?} code={LOCALIZATION_LOSS_RESULT_CODE}",
                    command_id.as_bytes(),
                    ActionStatus::Failed,
                );
                let step = runtime.advance(BehaviorTreeEvent::ActionTerminal {
                    command_id,
                    status: ActionStatus::Failed,
                });
                participation_dirty |= apply_tree_effects(
                    step.effects(),
                    now,
                    command_id,
                    &goal_intent,
                    source_id,
                    authority_boot,
                    &mut request_sequence,
                    &mut pending_goal,
                    &mut pending_cancel,
                    &mut deadline,
                    &mut participation,
                )?;
            }
        }
        let _ = health_subscriber.try_read_latest();

        participation_dirty |= participation.update_tree(&runtime);
        if participation_dirty
            || now.saturating_sub(last_participation_publish_ns) >= PARTICIPATION_PERIOD_NS
        {
            participation.publish_best_effort(&mut participation_publisher, now);
            publish_transport_diagnostics(
                &mut transport_diagnostics_publisher,
                now,
                process_epoch,
                authority_boot,
                command_id,
                &mut transport_diagnostics_sequence,
                [
                    (
                        AUTHORITY_CHANNEL,
                        authority_publisher.counters(),
                        authority_publisher.diagnostics().into(),
                    ),
                    (
                        ACTION_REQUEST_CHANNEL,
                        action_request_publisher.counters(),
                        action_request_publisher.diagnostics().into(),
                    ),
                    (
                        ACTION_RESPONSE_CHANNEL,
                        action_response_subscriber.counters(),
                        action_response_subscriber.diagnostics().into(),
                    ),
                    (
                        PROPOSAL_CHANNEL,
                        proposal_subscriber.counters(),
                        proposal_subscriber.diagnostics().into(),
                    ),
                    (
                        ODOMETRY_CHANNEL,
                        odometry_publisher.counters(),
                        odometry_publisher.diagnostics().into(),
                    ),
                    (
                        DYNAMIC_TF_CHANNEL,
                        tf_publisher.counters(),
                        tf_publisher.diagnostics().into(),
                    ),
                    (
                        HEALTH_CHANNEL,
                        health_subscriber.counters(),
                        health_subscriber.diagnostics().into(),
                    ),
                    (
                        PARTICIPATION_CHANNEL,
                        participation_publisher.counters(),
                        participation_publisher.diagnostics().into(),
                    ),
                ],
            );
            last_participation_publish_ns = now;
            participation_dirty = false;
        }

        if terminal && exit_on_terminal {
            return Ok(());
        }
        thread::sleep(MOTOR_DISABLED_TICK);
    }
}

fn navigation_behavior_tree() -> Result<BehaviorTree, Box<dyn Error>> {
    let action_name: ActionName = ActionName::parse(NAVIGATE_TO_POSE_ACTION)?;
    let mut registry = BehaviorTreeRegistry::new();
    registry.register_action(action_name)?;
    let resources = BehaviorTreeResources::try_from_slice(&[ResourceId::parse("chassis")?])?;
    let root = BehaviorTreeNodeId::new(0);
    let mut builder = BehaviorTreeBuilder::new(root);
    builder.add_node(
        TypedBehaviorTreeAction::<NavigateToPoseGoal, ()>::new(
            action_name,
            resources,
            NAVIGATE_TO_POSE_TIMEOUT_NS,
        )
        .into_node(root, BehaviorTreeNodeName::parse("navigate_to_pose")?),
    )?;
    Ok(builder.validate(&registry)?)
}

#[allow(clippy::too_many_arguments)]
fn apply_tree_effects(
    effects: &[BehaviorTreeEffect<'_>],
    now: u64,
    command_id: Id128,
    goal_intent: &ActionGoalIntent,
    source_id: AsciiId<64>,
    boot_id: Id128,
    request_sequence: &mut u64,
    pending_goal: &mut Option<PendingGoal>,
    pending_cancel: &mut Option<NavigationActionRequest>,
    deadline: &mut Option<ArmedDeadline>,
    participation: &mut NavigationParticipation,
) -> Result<bool, Box<dyn Error>> {
    let mut changed = false;
    for effect in effects {
        match effect {
            BehaviorTreeEffect::DispatchAction {
                activation, action, ..
            } => {
                if action.action_name().as_str() != NAVIGATE_TO_POSE_ACTION {
                    return Err("navigation tree dispatched an unexpected action".into());
                }
                if pending_goal.is_some() {
                    return Err("navigation tree requested a duplicate goal dispatch".into());
                }
                *pending_goal = Some(PendingGoal {
                    activation: *activation,
                    request: NavigationActionRequest::goal(goal_intent.clone())?,
                });
                changed |= replace_stage(
                    &mut participation.action,
                    ParticipantStage::new(
                        NavigationParticipantState::Pending,
                        NavigationParticipantReason::WaitingDispatch,
                    ),
                );
            }
            BehaviorTreeEffect::ArmDeadline {
                activation,
                timeout_ns,
            } => {
                *deadline = Some(ArmedDeadline {
                    activation: *activation,
                    expires_monotonic_ns: now.saturating_add(*timeout_ns),
                });
            }
            BehaviorTreeEffect::RequestActionCancel {
                command_id: effect_command_id,
                ..
            } => {
                if *effect_command_id != command_id {
                    return Err("navigation tree requested cancellation for another command".into());
                }
                queue_action_cancel(
                    now,
                    command_id,
                    source_id,
                    boot_id,
                    request_sequence,
                    pending_cancel,
                )?;
                changed |= participation.cancellation_requested();
            }
            BehaviorTreeEffect::ReleaseActionResources {
                command_id: effect_command_id,
                ..
            } if *effect_command_id == command_id => {
                *deadline = None;
            }
            BehaviorTreeEffect::RuntimeFault(_) => {
                changed |= replace_stage(
                    &mut participation.action,
                    ParticipantStage::new(
                        NavigationParticipantState::Faulted,
                        NavigationParticipantReason::Failed,
                    ),
                );
                changed |= replace_stage(
                    &mut participation.bridge,
                    ParticipantStage::new(
                        NavigationParticipantState::Faulted,
                        NavigationParticipantReason::Failed,
                    ),
                );
            }
            BehaviorTreeEffect::EvaluateGuard { .. }
            | BehaviorTreeEffect::CancellationRejected { .. }
            | BehaviorTreeEffect::ReleaseActionResources { .. }
            | BehaviorTreeEffect::MissionCompleted(_)
            | BehaviorTreeEffect::EventRejected(_) => {}
        }
    }
    Ok(changed)
}

fn queue_action_cancel(
    now: u64,
    command_id: Id128,
    source_id: AsciiId<64>,
    boot_id: Id128,
    request_sequence: &mut u64,
    pending_cancel: &mut Option<NavigationActionRequest>,
) -> Result<(), Box<dyn Error>> {
    if pending_cancel.is_some() {
        return Ok(());
    }
    *request_sequence = request_sequence.saturating_add(1);
    *pending_cancel = Some(NavigationActionRequest::cancel(ActionCancelRequest {
        header: local_header(source_id, boot_id, *request_sequence, now, 1_000_000_000)?,
        command_id,
    })?);
    Ok(())
}

fn executing_tree_command(runtime: &BehaviorTreeRuntime<'_>) -> Option<Id128> {
    runtime
        .snapshot()
        .activations()
        .iter()
        .find_map(|activation| {
            matches!(
                activation.state(),
                BehaviorTreeActivationState::Running
                    | BehaviorTreeActivationState::CancelRequested
                    | BehaviorTreeActivationState::Canceling
            )
            .then(|| activation.command_id())
            .flatten()
        })
}

fn canonical_goal(
    manifest: &NavigationPluginManifest,
) -> Result<NavigateToPoseGoal, Box<dyn Error>> {
    Ok(NavigateToPoseGoal {
        map: NavigationMapIdentity::new(
            AsciiId::parse(manifest.map_id())?,
            AsciiId::parse(manifest.map_version())?,
            Sha256Identity::new(
                manifest.map_manifest_digest().as_bytes(),
                "navigation map manifest digest",
            )?,
            AsciiId::parse(manifest.map_frame())?,
        ),
        target: Pose3 {
            position_m: Vector3::new(1.5, -3.0, 0.0).expect("static goal position"),
            orientation: UnitQuaternion::new(0.0, 0.0, 0.0, 1.0).expect("static goal orientation"),
        },
        behavior_tree_id: AsciiId::parse("navigate_to_pose_fail_closed")
            .expect("static behavior tree ID"),
    })
}

fn local_header(
    source_id: AsciiId<64>,
    boot_id: Id128,
    sequence: u64,
    now: u64,
    valid_for_ns: u64,
) -> Result<Header, message_model::SemanticError> {
    Header::new(HeaderFields {
        source_id,
        boot_id,
        sequence,
        source_monotonic_ns: now,
        publish_unix_ns: None,
        acquisition_timestamp: None,
        valid_for_ns,
        frame_id: None,
        clock_status: ClockStatus {
            state: ClockState::Local,
            uncertainty_ns: 0,
        },
        quality: DataQuality::Valid,
    })
}

fn monotonic_ns() -> u64 {
    let time = clock_gettime(ClockId::Monotonic);
    u64::try_from(time.tv_sec)
        .unwrap_or(u64::MAX)
        .saturating_mul(1_000_000_000)
        .saturating_add(u64::try_from(time.tv_nsec).unwrap_or(0))
}

fn proposal_dt_s(previous_accepted_ns: Option<u64>, now_ns: u64) -> f64 {
    previous_accepted_ns.map_or(NAV2_CONTROLLER_PERIOD_S, |previous_ns| {
        now_ns
            .saturating_sub(previous_ns)
            .clamp(MIN_PROPOSAL_STEP_NS, MAX_PROPOSAL_TTL_NS) as f64
            * 1.0e-9
    })
}

fn boot_id(now: u64, marker: u8) -> Result<Id128, message_model::Id128Error> {
    let mut bytes = [marker; 16];
    bytes[..8].copy_from_slice(&now.to_le_bytes());
    bytes[8..12].copy_from_slice(&std::process::id().to_le_bytes());
    Id128::new(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn proposal_step_tracks_publish_period_and_is_bounded() {
        assert_eq!(proposal_dt_s(None, 10), NAV2_CONTROLLER_PERIOD_S);
        assert!((proposal_dt_s(Some(1_000_000_000), 1_050_000_000) - 0.05).abs() < 1.0e-12);
        assert!((proposal_dt_s(Some(100), 100) - 0.001).abs() < 1.0e-12);
        assert!((proposal_dt_s(Some(1), 500_000_001) - 0.1).abs() < 1.0e-12);
    }

    #[test]
    fn navigation_leaf_waits_for_exact_ack_and_defers_pre_ack_cancellation() {
        let tree = navigation_behavior_tree().unwrap();
        let mut runtime = BehaviorTreeRuntime::new(&tree);
        let start = runtime.advance(BehaviorTreeEvent::Start);
        let activation = start
            .effects()
            .iter()
            .find_map(|effect| match effect {
                BehaviorTreeEffect::DispatchAction {
                    activation, action, ..
                } if action.action_name().as_str() == NAVIGATE_TO_POSE_ACTION => Some(*activation),
                _ => None,
            })
            .expect("one NavigateToPose dispatch");
        let command_id = Id128::new([3; 16]).unwrap();
        let _ = runtime.advance(BehaviorTreeEvent::ActionDispatched {
            activation,
            command_id,
        });
        assert!(matches!(
            runtime.snapshot().activations()[0].state(),
            BehaviorTreeActivationState::WaitingAcknowledgement
        ));

        let pre_ack_cancel = runtime.advance(BehaviorTreeEvent::CancelAll);
        assert!(
            !pre_ack_cancel
                .effects()
                .iter()
                .any(|effect| matches!(effect, BehaviorTreeEffect::RequestActionCancel { .. }))
        );
        let acknowledged = runtime.advance(BehaviorTreeEvent::ActionAcknowledged {
            command_id,
            accepted: true,
        });
        assert!(acknowledged.effects().iter().any(|effect| matches!(
            effect,
            BehaviorTreeEffect::RequestActionCancel {
                command_id: observed,
                ..
            } if *observed == command_id
        )));
        assert!(matches!(
            runtime.snapshot().activations()[0].state(),
            BehaviorTreeActivationState::CancelRequested
        ));
    }

    #[test]
    fn rejected_navigation_acknowledgement_is_terminal_without_feedback() {
        let tree = navigation_behavior_tree().unwrap();
        let mut runtime = BehaviorTreeRuntime::new(&tree);
        let start = runtime.advance(BehaviorTreeEvent::Start);
        let activation = start
            .effects()
            .iter()
            .find_map(|effect| match effect {
                BehaviorTreeEffect::DispatchAction { activation, .. } => Some(*activation),
                _ => None,
            })
            .expect("one dispatch");
        let command_id = Id128::new([4; 16]).unwrap();
        let _ = runtime.advance(BehaviorTreeEvent::ActionDispatched {
            activation,
            command_id,
        });
        let rejected = runtime.advance(BehaviorTreeEvent::ActionAcknowledged {
            command_id,
            accepted: false,
        });
        assert!(rejected.effects().iter().any(|effect| matches!(
            effect,
            BehaviorTreeEffect::ReleaseActionResources {
                command_id: observed,
                outcome: BehaviorTreeOutcome::Rejected,
                ..
            } if *observed == command_id
        )));
        assert_eq!(
            runtime.snapshot().state(),
            BehaviorTreeRuntimeState::Terminal(BehaviorTreeOutcome::Rejected)
        );
    }

    #[test]
    fn localized_feedback_loss_terminates_an_acknowledged_navigation_leaf() {
        let tree = navigation_behavior_tree().unwrap();
        let mut runtime = BehaviorTreeRuntime::new(&tree);
        let start = runtime.advance(BehaviorTreeEvent::Start);
        let activation = start
            .effects()
            .iter()
            .find_map(|effect| match effect {
                BehaviorTreeEffect::DispatchAction { activation, .. } => Some(*activation),
                _ => None,
            })
            .expect("one dispatch");
        let command_id = Id128::new([5; 16]).unwrap();
        let _ = runtime.advance(BehaviorTreeEvent::ActionDispatched {
            activation,
            command_id,
        });
        let _ = runtime.advance(BehaviorTreeEvent::ActionAcknowledged {
            command_id,
            accepted: true,
        });

        let failed = runtime.advance(BehaviorTreeEvent::ActionTerminal {
            command_id,
            status: ActionStatus::Failed,
        });
        assert!(failed.effects().iter().any(|effect| matches!(
            effect,
            BehaviorTreeEffect::ReleaseActionResources {
                command_id: observed,
                outcome: BehaviorTreeOutcome::Failed,
                ..
            } if *observed == command_id
        )));
        assert_eq!(
            runtime.snapshot().state(),
            BehaviorTreeRuntimeState::Terminal(BehaviorTreeOutcome::Failed)
        );
    }
}
