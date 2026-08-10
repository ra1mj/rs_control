//! Operator-facing behavior-tree inspection utility with no hardware access.

use std::{error::Error, fmt::Write as _, fs, io, path::PathBuf, process::ExitCode};

use clap::{Parser, Subcommand};
use control_core::{
    BehaviorTreeActivationId, BehaviorTreeCommandId, BehaviorTreeEffect, BehaviorTreeEvent,
    BehaviorTreeRuntime, representative_automatic_project,
};
use navigation_ipc::NavigationIpcNamespace;

#[derive(Debug, Parser)]
#[command(
    name = "btctl",
    version,
    about = "Inspect the project-native behavior-tree runtime"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Render the checked example tree and replay deterministic startup events.
    Example,
    /// Read the latest navigation participation snapshot without mutating IPC.
    Inspect {
        /// Existing absolute navigation IPC namespace.
        #[arg(long)]
        ipc_root: PathBuf,
    },
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("btctl: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run(cli: Cli) -> Result<(), Box<dyn Error>> {
    match cli.command {
        Command::Example => run_example(),
        Command::Inspect { ipc_root } => run_inspect(ipc_root),
    }
}

fn run_inspect(ipc_root: PathBuf) -> Result<(), Box<dyn Error>> {
    if !ipc_root.is_absolute() {
        return Err("--ipc-root must be absolute".into());
    }
    let metadata = fs::metadata(&ipc_root).map_err(|error| {
        io::Error::new(
            error.kind(),
            format!("navigation IPC namespace is missing: {ipc_root:?}"),
        )
    })?;
    if !metadata.is_dir() {
        return Err("--ipc-root must name an existing directory".into());
    }
    let ipc = NavigationIpcNamespace::new(&ipc_root)?;
    let mut observer = ipc.attach_participation_observer()?;
    let sample = observer.try_observe_latest().ok_or_else(|| {
        io::Error::other(
            "no current navigation participation snapshot (missing, stale, or invalid)",
        )
    })?;
    let snapshot = sample.value;
    let header = snapshot.header();
    println!(
        "snapshot generation={} source={} boot={} sequence={} process_epoch={} command={}",
        sample.generation,
        header.source_id(),
        id128_text(header.boot_id()),
        header.sequence(),
        snapshot.process_epoch(),
        snapshot
            .command_id()
            .map_or_else(|| "-".to_owned(), id128_text),
    );
    for participant in snapshot.participants() {
        println!(
            "participant id={} state={} reason={}",
            participant.id(),
            participant.state().as_str(),
            participant.reason().as_str(),
        );
    }
    Ok(())
}

fn run_example() -> Result<(), Box<dyn Error>> {
    let tree = representative_automatic_project()?;
    print!("{}", tree.render_tree());

    let mut runtime = BehaviorTreeRuntime::new(&tree);
    println!("event=start");
    let start = runtime.advance(BehaviorTreeEvent::Start);
    print_effects(start.effects());
    print!("{}", runtime.snapshot().render());

    let guard = start
        .effects()
        .iter()
        .find_map(|effect| match effect {
            BehaviorTreeEffect::EvaluateGuard { activation, .. } => Some(*activation),
            _ => None,
        })
        .ok_or_else(|| io::Error::other("example did not request route guard evaluation"))?;
    println!("event=guard_resolved value=true");
    let routed = runtime.advance(BehaviorTreeEvent::GuardResolved {
        activation: guard,
        value: true,
    });
    print_effects(routed.effects());
    print!("{}", runtime.snapshot().render());
    Ok(())
}

fn print_effects(effects: &[BehaviorTreeEffect<'_>]) {
    for effect in effects {
        match effect {
            BehaviorTreeEffect::DispatchAction {
                activation,
                node_id,
                action,
            } => println!(
                "effect=dispatch_action activation={} node={} action={}",
                activation_text(*activation),
                node_id.get(),
                action.action_name(),
            ),
            BehaviorTreeEffect::ArmDeadline {
                activation,
                timeout_ns,
            } => println!(
                "effect=arm_deadline activation={} timeout_ns={timeout_ns}",
                activation_text(*activation),
            ),
            BehaviorTreeEffect::EvaluateGuard {
                activation,
                guard_id,
            } => println!(
                "effect=evaluate_guard activation={} guard={guard_id}",
                activation_text(*activation),
            ),
            BehaviorTreeEffect::RequestActionCancel { command_id, reason } => println!(
                "effect=request_action_cancel command={} reason={reason:?}",
                id128_text(*command_id),
            ),
            BehaviorTreeEffect::CancellationRejected { command_id } => println!(
                "effect=cancellation_rejected command={}",
                id128_text(*command_id),
            ),
            BehaviorTreeEffect::ReleaseActionResources {
                activation,
                node_id,
                command_id,
                outcome,
            } => println!(
                "effect=release_action_resources activation={} node={} command={} outcome={outcome:?}",
                activation_text(*activation),
                node_id.get(),
                id128_text(*command_id),
            ),
            BehaviorTreeEffect::MissionCompleted(outcome) => {
                println!("effect=mission_completed outcome={outcome:?}");
            }
            BehaviorTreeEffect::EventRejected(rejection) => {
                println!("effect=event_rejected reason={rejection:?}");
            }
            BehaviorTreeEffect::RuntimeFault(fault) => {
                println!("effect=runtime_fault reason={fault:?}");
            }
        }
    }
}

fn activation_text(activation: BehaviorTreeActivationId) -> String {
    format!("{}:{}", activation.slot(), activation.generation())
}

fn id128_text(value: BehaviorTreeCommandId) -> String {
    let mut rendered = String::with_capacity(32);
    for byte in value.as_bytes() {
        write!(rendered, "{byte:02x}").expect("writing to String cannot fail");
    }
    rendered
}
