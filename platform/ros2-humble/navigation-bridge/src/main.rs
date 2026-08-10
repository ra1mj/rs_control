mod bridge;
#[cfg(test)]
mod tests;

use std::{
    error::Error,
    path::{Path, PathBuf},
    process::ExitCode,
    sync::{Arc, Mutex},
    time::Duration,
};

use bridge::{goal_to_ros, ActionCorrelation, BridgeAdmission, GoalBinding};
use clap::Parser;
use contract_codec::decode_navigate_to_pose_goal;
use futures::StreamExt;
use geometry_msgs::msg::Twist;
use message_model::{
    ActionStatus, AsciiId, ClockState, ClockStatus, DataQuality, Header, HeaderFields, Id128,
    NavigationComponentState, NavigationCounters, NavigationHealth, NavigationReadiness,
    OdometryState, Sha256Identity, TransformBatch,
};
use middleware_api::{EventPublisher, EventSubscriber, LatestPublisher, LatestSubscriber};
use nav2_msgs::action::NavigateToPose;
use navigation_ipc::{
    ActionResponsePublisher, NavigationActionRequest, NavigationActionRequestKind,
    NavigationActionResponse, NavigationIpcNamespace, ProposalPublisher,
};
use navigation_plugin_config::NavigationPluginManifest;
use rclrs::*;
use rustix::time::{clock_gettime, ClockId};

// rclrs sends an action goal request even if the client has not yet discovered
// a late-starting server. That request then has no response to correlate. Hold
// the already-admitted local request briefly after client/server readiness so
// the bridge's own DDS endpoint has completed discovery before its one exact
// goal request is sent.
const ACTION_CLIENT_DISCOVERY_SETTLE_NS: u64 = 500_000_000;

#[derive(Debug, Parser)]
#[command(name = "rs-navigation-bridge", version)]
struct Cli {
    #[arg(long)]
    manifest: PathBuf,
    #[arg(long)]
    expected_sha256: String,
    #[arg(long)]
    ipc_root: PathBuf,
    #[arg(long)]
    process_epoch: u64,
    /// Optional supervisor-owned readiness file gating action requests.
    #[arg(long)]
    nav2_ready_file: Option<PathBuf>,
}

struct ProposalState {
    admission: BridgeAdmission,
    publisher: ProposalPublisher,
}

struct ActionState {
    correlation: ActionCorrelation,
    publisher: ActionResponsePublisher,
}

#[derive(Default)]
struct BridgeMetrics {
    counters: NavigationCounters,
    action_server: bool,
    nav2_lifecycle: bool,
    odometry: bool,
    tf: bool,
}

impl ActionState {
    fn publish(&mut self, value: NavigationActionResponse) {
        let _ = self.publisher.try_publish(value);
    }
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("rs-navigation-bridge: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run(cli: Cli) -> Result<(), Box<dyn Error>> {
    if cli.process_epoch == 0 {
        return Err("--process-epoch must be nonzero".into());
    }
    if cli
        .nav2_ready_file
        .as_ref()
        .is_some_and(|path| !path.is_absolute() || path == Path::new("/"))
    {
        return Err("--nav2-ready-file must be a bounded absolute path".into());
    }
    let manifest = NavigationPluginManifest::load(&cli.manifest)?;
    if manifest.fingerprint().to_string() != cli.expected_sha256 {
        return Err("manifest SHA-256 does not match --expected-sha256".into());
    }
    if std::env::var("ROS_DISTRO").as_deref() != Ok("humble") {
        return Err("ROS_DISTRO must be humble".into());
    }
    if std::env::var("RMW_IMPLEMENTATION").as_deref() != Ok("rmw_fastrtps_cpp") {
        return Err("RMW_IMPLEMENTATION must be rmw_fastrtps_cpp".into());
    }

    let ipc = NavigationIpcNamespace::new(&cli.ipc_root)?;
    let mut authority_subscriber = ipc.attach_authority_subscriber()?;
    let proposal_publisher = ipc.attach_proposal_publisher()?;
    let mut action_request_subscriber = ipc.attach_action_request_subscriber()?;
    let action_response_publisher = ipc.attach_action_response_publisher()?;
    let mut odometry_subscriber = ipc.attach_odometry_subscriber()?;
    let mut tf_subscriber = ipc.attach_dynamic_tf_subscriber()?;
    let mut health_publisher = ipc.attach_health_publisher()?;

    let context = Context::default();
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("rs_navigation_bridge")?;
    let action_client = node.create_action_client::<NavigateToPose>("/navigate_to_pose")?;
    let action_client_created_ns = monotonic_ns();
    let ros_clock = node.get_clock();
    let odometry_publisher = node.create_publisher::<nav_msgs::msg::Odometry>(
        "/rs_control/navigation/odometry".keep_last(1),
    )?;
    let tf_publisher = node.create_publisher::<tf2_msgs::msg::TFMessage>("/tf".keep_last(1))?;

    let boot_id = process_boot_id(cli.process_epoch)?;
    let shared = Arc::new(Mutex::new(ProposalState {
        admission: BridgeAdmission::new(boot_id),
        publisher: proposal_publisher,
    }));
    let action_state = Arc::new(Mutex::new(ActionState {
        correlation: ActionCorrelation::new(boot_id),
        publisher: action_response_publisher,
    }));
    let metrics = Arc::new(Mutex::new(BridgeMetrics::default()));
    let proposal_shared = Arc::clone(&shared);
    let proposal_metrics = Arc::clone(&metrics);
    let _proposal_subscription = node.create_subscription::<Twist, _>(
        "/rs_control/navigation/controller_proposal".keep_last(1),
        move |twist: Twist| {
            let now = monotonic_ns();
            let Ok(mut state) = proposal_shared.lock() else {
                return;
            };
            match state.admission.accept_twist(twist, now) {
                Ok(proposal) => {
                    let _ = state.publisher.try_publish(proposal);
                    if let Ok(mut metrics) = proposal_metrics.lock() {
                        metrics.counters.accepted_proposals =
                            metrics.counters.accepted_proposals.saturating_add(1);
                    }
                }
                Err(_) => {
                    if let Ok(mut metrics) = proposal_metrics.lock() {
                        metrics.counters.authority_rejections =
                            metrics.counters.authority_rejections.saturating_add(1);
                    }
                }
            }
        },
    )?;

    let authority_shared = Arc::clone(&shared);
    let _authority_timer = node.create_timer_repeating(Duration::from_millis(10), move || {
        while let Some(sample) = authority_subscriber.try_receive() {
            if let Ok(mut state) = authority_shared.lock() {
                let _ = state
                    .admission
                    .update_authority(sample.value, monotonic_ns());
            }
        }
        if !authority_subscriber
            .refresh_peer_liveness()
            .unwrap_or(false)
        {
            if let Ok(mut state) = authority_shared.lock() {
                state.admission.clear_authority();
            }
        }
    })?;

    let action_commands = Arc::clone(executor.commands());
    let action_timer_state = Arc::clone(&action_state);
    let action_timer_client = Arc::clone(&action_client);
    let action_timer_metrics = Arc::clone(&metrics);
    let nav2_ready_file = cli.nav2_ready_file.clone();
    let mut pending_goal: Option<NavigationActionRequest> = None;
    let mut nav2_ready_since_ns = None;
    let _action_timer = node.create_timer_repeating(Duration::from_millis(10), move || {
        while let Some(sample) = action_request_subscriber.try_receive() {
            match sample.value.kind() {
                NavigationActionRequestKind::Goal(_) if pending_goal.is_none() => {
                    pending_goal = Some(sample.value.clone());
                }
                NavigationActionRequestKind::Goal(_) => {}
                NavigationActionRequestKind::Cancel(cancel) => {
                    let Ok(state) = action_timer_state.lock() else {
                        continue;
                    };
                    let Ok(binding) = state.correlation.binding(cancel.command_id) else {
                        continue;
                    };
                    let Ok(cancellation) =
                        action_timer_client.try_cancel_goal(GoalUuid(binding.ros_goal_uuid))
                    else {
                        continue;
                    };
                    drop(state);

                    let response_state = Arc::clone(&action_timer_state);
                    drop(action_commands.run(async move {
                        let response = cancellation.await;
                        if let Ok(mut state) = response_state.lock() {
                            let semantic = state.correlation.cancellation(
                                binding,
                                response.is_accepted(),
                                response.code as u32,
                                format!("Nav2 cancel response: {:?}", response.code),
                                monotonic_ns(),
                            );
                            if let Ok(semantic) = semantic {
                                state.publish(semantic);
                            }
                        }
                    }));
                }
            }
        }

        let Some(request) = pending_goal.take() else {
            return;
        };
        let now = monotonic_ns();
        let ready_since_ns = match nav2_ready_file.as_ref() {
            Some(path) if !path.is_file() => {
                pending_goal = Some(request);
                return;
            }
            Some(_) => *nav2_ready_since_ns.get_or_insert(now),
            None => action_client_created_ns,
        };
        if !goal_dispatch_ready(now, ready_since_ns) {
            pending_goal = Some(request);
            return;
        }
        let NavigationActionRequestKind::Goal(intent) = request.kind() else {
            return;
        };
        let command_id = intent.intent().command_id();
        let Ok(goal) = decode_navigate_to_pose_goal(intent.goal().value()) else {
            reject_action(
                &action_timer_state,
                command_id,
                10,
                "invalid NavigateToPose goal payload",
            );
            return;
        };
        if goal.behavior_tree_id.as_str() != "navigate_to_pose_fail_closed" {
            reject_action(
                &action_timer_state,
                command_id,
                11,
                "unexpected NavigateToPose behavior tree identifier",
            );
            return;
        }
        let Ok(mut state) = action_timer_state.lock() else {
            pending_goal = Some(request);
            return;
        };
        if !state.correlation.is_idle() {
            pending_goal = Some(request);
            return;
        }
        let Ok(requested) = action_timer_client.try_request_goal(goal_to_ros(goal)) else {
            pending_goal = Some(request);
            return;
        };
        let ros_goal_uuid = requested.goal_id().0;
        let Ok(binding) = state.correlation.bind(command_id, ros_goal_uuid, goal) else {
            return;
        };
        drop(state);

        let response_state = Arc::clone(&action_timer_state);
        let response_metrics = Arc::clone(&action_timer_metrics);
        drop(action_commands.run(async move {
            let Some(goal_client) = requested.await else {
                acknowledge_action(
                    &response_state,
                    &response_metrics,
                    binding,
                    false,
                    1,
                    "Nav2 rejected the goal",
                );
                return;
            };
            acknowledge_action(
                &response_state,
                &response_metrics,
                binding,
                true,
                0,
                "Nav2 accepted the goal",
            );
            let mut events = goal_client.stream();
            while let Some(event) = events.next().await {
                match event {
                    GoalEvent::Feedback(feedback) => {
                        if let Ok(mut state) = response_state.lock() {
                            let response =
                                state
                                    .correlation
                                    .feedback(binding, feedback, monotonic_ns());
                            if let Ok(response) = response {
                                state.publish(response);
                            }
                        }
                    }
                    GoalEvent::Status(_) => {}
                    GoalEvent::Result((status, _)) => {
                        let (status, code, detail) = terminal_status(status);
                        finish_action(
                            &response_state,
                            &response_metrics,
                            binding,
                            status,
                            code,
                            detail,
                        );
                        return;
                    }
                }
            }
            finish_action(
                &response_state,
                &response_metrics,
                binding,
                ActionStatus::Failed,
                2,
                "Nav2 action stream ended without a terminal result",
            );
        }));
    })?;

    let feedback_metrics = Arc::clone(&metrics);
    let feedback_clock = ros_clock.clone();
    let _feedback_timer = node.create_timer_repeating(Duration::from_millis(20), move || {
        // The IPC timestamps stay in their local monotonic clock domain.  ROS
        // TF consumers instead require a timestamp from the bridge's ROS clock
        // so it can be compared to sensor messages in the ROS graph.
        let Ok(rclrs_stamp) = feedback_clock.now().to_ros_msg() else {
            return;
        };
        let ros_stamp = builtin_interfaces::msg::Time {
            sec: rclrs_stamp.sec,
            nanosec: rclrs_stamp.nanosec,
        };
        if let Some(sample) = odometry_subscriber.try_read_latest() {
            let _ = odometry_publisher.publish(odometry_to_ros(sample.value, ros_stamp.clone()));
            if let Ok(mut metrics) = feedback_metrics.lock() {
                metrics.odometry = true;
            }
        }
        if let Some(sample) = tf_subscriber.try_read_latest() {
            let _ = tf_publisher.publish(tf_to_ros(sample.value.into_batch(), ros_stamp));
            if let Ok(mut metrics) = feedback_metrics.lock() {
                metrics.tf = true;
            }
        }
    })?;

    let health_metrics = Arc::clone(&metrics);
    let manifest_identity = Sha256Identity::new(manifest.fingerprint().as_bytes(), "manifest")?;
    let process_epoch = cli.process_epoch;
    let mut health_sequence = 0_u64;
    let _health_timer = node.create_timer_repeating(Duration::from_millis(100), move || {
        let Ok(metrics) = health_metrics.lock() else {
            return;
        };
        health_sequence = health_sequence.saturating_add(1);
        let readiness = NavigationReadiness {
            manifest: true,
            action_server: metrics.action_server,
            nav2_lifecycle: metrics.nav2_lifecycle,
            map: true,
            localization: metrics.odometry && metrics.tf,
            ipc: true,
        };
        let component_state = if readiness.all_ready() {
            NavigationComponentState::Ready
        } else {
            NavigationComponentState::Starting
        };
        let Ok(header) = bridge_header(boot_id, health_sequence, monotonic_ns()) else {
            return;
        };
        let health = NavigationHealth::new(
            header,
            process_epoch,
            manifest_identity,
            component_state,
            readiness,
            Default::default(),
            metrics.counters,
        );
        if let Ok(health) = health {
            let _ = health_publisher.try_publish(health);
        }
    })?;

    let errors = executor.spin(SpinOptions::default());
    if !errors.is_empty() {
        return Err(format!("executor stopped with {} errors", errors.len()).into());
    }
    Ok(())
}

fn goal_dispatch_ready(now_monotonic_ns: u64, ready_since_monotonic_ns: u64) -> bool {
    now_monotonic_ns.saturating_sub(ready_since_monotonic_ns) >= ACTION_CLIENT_DISCOVERY_SETTLE_NS
}

fn acknowledge_action(
    shared: &Arc<Mutex<ActionState>>,
    metrics: &Arc<Mutex<BridgeMetrics>>,
    binding: GoalBinding,
    accepted: bool,
    result_code: u32,
    detail: &str,
) {
    if let Ok(mut state) = shared.lock() {
        let response = state.correlation.acknowledgement(
            binding,
            accepted,
            result_code,
            detail.to_owned(),
            monotonic_ns(),
        );
        if let Ok(response) = response {
            state.publish(response);
            if accepted {
                if let Ok(mut metrics) = metrics.lock() {
                    metrics.action_server = true;
                    metrics.nav2_lifecycle = true;
                    metrics.counters.accepted_goals =
                        metrics.counters.accepted_goals.saturating_add(1);
                }
            }
        }
    }
}

fn reject_action(
    shared: &Arc<Mutex<ActionState>>,
    command_id: Id128,
    result_code: u32,
    detail: &str,
) {
    if let Ok(mut state) = shared.lock() {
        let response =
            state
                .correlation
                .rejection(command_id, result_code, detail.to_owned(), monotonic_ns());
        if let Ok(response) = response {
            state.publish(response);
        }
    }
}

fn finish_action(
    shared: &Arc<Mutex<ActionState>>,
    metrics: &Arc<Mutex<BridgeMetrics>>,
    binding: GoalBinding,
    status: ActionStatus,
    result_code: u32,
    detail: &str,
) {
    if let Ok(mut state) = shared.lock() {
        let response = state.correlation.finish(
            binding,
            status,
            result_code,
            detail.to_owned(),
            monotonic_ns(),
        );
        if let Ok(response) = response {
            state.publish(response);
            if let Ok(mut metrics) = metrics.lock() {
                metrics.counters.terminal_results =
                    metrics.counters.terminal_results.saturating_add(1);
            }
        }
    }
}

fn terminal_status(value: GoalStatusCode) -> (ActionStatus, u32, &'static str) {
    match value {
        GoalStatusCode::Succeeded => (ActionStatus::Succeeded, 0, "Nav2 goal succeeded"),
        GoalStatusCode::Cancelled => (ActionStatus::Cancelled, 0, "Nav2 goal cancelled"),
        GoalStatusCode::Aborted => (ActionStatus::Failed, 3, "Nav2 goal aborted"),
        GoalStatusCode::Unknown
        | GoalStatusCode::Accepted
        | GoalStatusCode::Executing
        | GoalStatusCode::Cancelling => (
            ActionStatus::Failed,
            4,
            "Nav2 returned a nonterminal status with its result",
        ),
    }
}

fn bridge_header(
    boot_id: Id128,
    sequence: u64,
    now_monotonic_ns: u64,
) -> Result<Header, message_model::SemanticError> {
    Header::new(HeaderFields {
        source_id: AsciiId::parse("navigation-bridge").expect("static bridge source"),
        boot_id,
        sequence,
        source_monotonic_ns: now_monotonic_ns,
        publish_unix_ns: None,
        acquisition_timestamp: None,
        valid_for_ns: 500_000_000,
        frame_id: None,
        clock_status: ClockStatus {
            state: ClockState::Local,
            uncertainty_ns: 0,
        },
        quality: DataQuality::Valid,
    })
}

fn odometry_to_ros(
    value: OdometryState,
    stamp: builtin_interfaces::msg::Time,
) -> nav_msgs::msg::Odometry {
    let pose = value.pose();
    let [qx, qy, qz, qw] = pose.orientation.components();
    let twist = value.twist();
    nav_msgs::msg::Odometry {
        header: ros_header(value.parent_frame_id().as_str(), stamp),
        child_frame_id: value.child_frame_id().as_str().to_owned(),
        pose: geometry_msgs::msg::PoseWithCovariance {
            pose: geometry_msgs::msg::Pose {
                position: geometry_msgs::msg::Point {
                    x: pose.position_m.x(),
                    y: pose.position_m.y(),
                    z: pose.position_m.z(),
                },
                orientation: geometry_msgs::msg::Quaternion {
                    x: qx,
                    y: qy,
                    z: qz,
                    w: qw,
                },
            },
            covariance: *value.pose_covariance().values(),
        },
        twist: geometry_msgs::msg::TwistWithCovariance {
            twist: Twist {
                linear: geometry_msgs::msg::Vector3 {
                    x: twist.linear_mps.x(),
                    y: twist.linear_mps.y(),
                    z: twist.linear_mps.z(),
                },
                angular: geometry_msgs::msg::Vector3 {
                    x: twist.angular_radps.x(),
                    y: twist.angular_radps.y(),
                    z: twist.angular_radps.z(),
                },
            },
            covariance: *value.twist_covariance().values(),
        },
    }
}

fn tf_to_ros(
    value: TransformBatch,
    stamp: builtin_interfaces::msg::Time,
) -> tf2_msgs::msg::TFMessage {
    tf2_msgs::msg::TFMessage {
        transforms: value
            .transforms()
            .iter()
            .map(|transform| {
                let [x, y, z, w] = transform.rotation().components();
                geometry_msgs::msg::TransformStamped {
                    header: ros_header(transform.parent_frame_id().as_str(), stamp.clone()),
                    child_frame_id: transform.child_frame_id().as_str().to_owned(),
                    transform: geometry_msgs::msg::Transform {
                        translation: geometry_msgs::msg::Vector3 {
                            x: transform.translation_m().x(),
                            y: transform.translation_m().y(),
                            z: transform.translation_m().z(),
                        },
                        rotation: geometry_msgs::msg::Quaternion { x, y, z, w },
                    },
                }
            })
            .collect(),
    }
}

fn ros_header(frame_id: &str, stamp: builtin_interfaces::msg::Time) -> std_msgs::msg::Header {
    std_msgs::msg::Header {
        stamp,
        frame_id: frame_id.to_owned(),
    }
}

fn monotonic_ns() -> u64 {
    let time = clock_gettime(ClockId::Monotonic);
    u64::try_from(time.tv_sec)
        .unwrap_or(u64::MAX)
        .saturating_mul(1_000_000_000)
        .saturating_add(u64::try_from(time.tv_nsec).unwrap_or(0))
}

fn process_boot_id(epoch: u64) -> Result<Id128, message_model::Id128Error> {
    let mut bytes = [0x52; 16];
    bytes[..8].copy_from_slice(&epoch.to_le_bytes());
    bytes[8..12].copy_from_slice(&std::process::id().to_le_bytes());
    Id128::new(bytes)
}
