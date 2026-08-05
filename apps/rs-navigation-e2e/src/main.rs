//! Motor-disabled navigation integration composition root.

use std::{error::Error, path::PathBuf, process::ExitCode, thread, time::Duration};

use chassis_model::BodyPose2;
use clap::{Parser, Subcommand};
use contract_codec::encode_navigate_to_pose_goal;
use message_model::{
    ActionCancelRequest, ActionGoalIntent, AsciiId, ChassisSetpointIntent, ClockState, ClockStatus,
    CommandIntent, CommandIntentFields, DataQuality, Header, HeaderFields, Id128,
    NavigateToPoseGoal, NavigationMapIdentity, Pose3, RegisteredPayload, Sha256Identity, Twist2,
    UnitQuaternion, Vector3,
};
use middleware_api::{
    EventPublishError, EventPublisher, EventSubscriber, LatestPublisher, LatestSubscriber,
};
use navigation_ipc::{
    NAVIGATE_TO_POSE_ACTION, NAVIGATE_TO_POSE_GOAL_TYPE, NavigationActionRequest,
    NavigationActionResponseKind, NavigationIpcNamespace,
};
use navigation_plugin_config::NavigationPluginManifest;
use rs_navigation_e2e::{MAX_PROPOSAL_TTL_NS, MotorDisabledHarness, StopReason};
use rustix::time::{ClockId, clock_gettime};

const MOTOR_DISABLED_TICK: Duration = Duration::from_millis(5);
const NAV2_CONTROLLER_PERIOD_S: f64 = 0.05;
const MIN_PROPOSAL_STEP_NS: u64 = 1_000_000;

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
    let mut pending_goal = Some(NavigationActionRequest::goal(goal_intent)?);
    let mut pending_cancel = None;
    let mut cancel_sent = false;
    let mut goal_sent_at_ns = None;
    let mut active_goal = true;
    let mut terminal = false;
    let mut feedback_sequence = 0_u64;
    let mut proposal_peer_was_alive = false;
    let mut first_proposal_at_ns = None;
    let mut last_proposal_at_ns = None;
    let mut proposal_accepted_logged = false;
    let mut feedback_unavailable_logged = false;

    println!(
        "motor_output=forbidden can_transmit=forbidden process_epoch={process_epoch} command_id={:02x?}",
        command_id.as_bytes()
    );
    loop {
        let now = monotonic_ns();
        if let Some(request) = pending_goal.take() {
            match action_request_publisher.try_publish(request) {
                Ok(_) => goal_sent_at_ns = Some(now),
                Err(
                    EventPublishError::Full(request) | EventPublishError::Disconnected(request),
                ) => {
                    pending_goal = Some(request);
                }
            }
        }

        if !cancel_sent
            && pending_cancel.is_none()
            && !terminal
            && cancel_after_ms.is_some_and(|delay_ms| {
                goal_sent_at_ns.is_some_and(|sent_ns| {
                    now.saturating_sub(sent_ns) >= delay_ms.saturating_mul(1_000_000)
                })
            })
        {
            pending_cancel = Some(NavigationActionRequest::cancel(ActionCancelRequest {
                header: local_header(
                    AsciiId::parse("rs-navigation-e2e")?,
                    authority_boot,
                    2,
                    now,
                    1_000_000_000,
                )?,
                command_id,
            })?);
        }
        if let Some(request) = pending_cancel.take() {
            match action_request_publisher.try_publish(request) {
                Ok(_) => cancel_sent = true,
                Err(
                    EventPublishError::Full(request) | EventPublishError::Disconnected(request),
                ) => {
                    pending_cancel = Some(request);
                }
            }
        }

        while let Some(sample) = action_response_subscriber.try_receive() {
            match sample.value.kind() {
                NavigationActionResponseKind::Feedback(feedback) => {
                    if feedback.command_id == command_id {
                        active_goal = true;
                    }
                }
                NavigationActionResponseKind::Cancel(response) => {
                    if response.command_id == command_id {
                        println!(
                            "cancel command_id={:02x?} decision={:?} code={}",
                            command_id.as_bytes(),
                            response.decision,
                            response.result_code
                        );
                    }
                }
                NavigationActionResponseKind::Result(result) => {
                    if result.command_id == command_id {
                        active_goal = false;
                        terminal = true;
                        println!(
                            "terminal command_id={:02x?} status={:?} code={}",
                            command_id.as_bytes(),
                            result.status,
                            result.result_code
                        );
                    }
                }
            }
        }

        let authority = harness.authority_state(
            now,
            (active_goal && goal_sent_at_ns.is_some()).then_some(command_id),
        )?;
        let _ = authority_publisher.try_publish(authority);

        if let Some(sample) = proposal_subscriber.try_read_latest() {
            let dt_s = proposal_dt_s(last_proposal_at_ns, now);
            if harness.accept_proposal(sample.value, now, dt_s).is_ok() {
                let _ = first_proposal_at_ns.get_or_insert(now);
                last_proposal_at_ns = Some(now);
                if !proposal_accepted_logged {
                    println!("proposal=Accepted motor_output=forbidden");
                    proposal_accepted_logged = true;
                }
            }
        }
        let proposal_peer_alive = proposal_subscriber.refresh_peer_liveness().unwrap_or(false);
        if proposal_peer_was_alive && !proposal_peer_alive {
            harness.peer_lost(now)?;
            println!("stop=PeerUnavailable motor_output=forbidden");
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
        }
        let _ = health_subscriber.try_read_latest();

        if terminal && exit_on_terminal {
            return Ok(());
        }
        thread::sleep(MOTOR_DISABLED_TICK);
    }
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
}
