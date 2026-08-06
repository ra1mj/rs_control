//! Operator-facing behavior-tree inspection utility with no hardware access.

use std::{error::Error, fmt::Write as _, io, process::ExitCode};

use clap::{Parser, Subcommand};
use control_core::{
    BehaviorTreeActivationId, BehaviorTreeCommandId, BehaviorTreeEffect, BehaviorTreeEvent,
    BehaviorTreeRuntime, representative_automatic_project,
};

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
    }
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
