//! Operator-facing behavior-tree inspection utility with no hardware access.

mod dashboard;

use std::{
    error::Error,
    fmt::Write as _,
    fs, io,
    io::{IsTerminal as _, Write as _},
    path::{Path, PathBuf},
    process::ExitCode,
    thread,
    time::Duration,
};

use clap::{Args, Parser, Subcommand, ValueEnum};
use control_core::{
    BehaviorTreeActivationId, BehaviorTreeCommandId, BehaviorTreeEffect, BehaviorTreeEvent,
    BehaviorTreeRuntime, representative_automatic_project,
};
use navigation_ipc::{
    NavigationIpcNamespace, NavigationParticipationSnapshot, NavigationTransportDiagnosticsSnapshot,
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
    /// Read the latest navigation participation snapshot without mutating IPC.
    Inspect {
        /// Existing absolute navigation IPC namespace.
        #[arg(long)]
        ipc_root: PathBuf,
    },
    /// Observe live tree, node and transport diagnostics in one view.
    Observe {
        #[command(flatten)]
        options: ObserveOptions,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, ValueEnum)]
enum ObserveSection {
    /// Show all three sections in one sample.
    All,
    /// Show only the behavior-tree hierarchy.
    Tree,
    /// Show only node participation state.
    Nodes,
    /// Show only transport delivery diagnostics.
    Transport,
}

impl ObserveSection {
    const fn includes_tree(self) -> bool {
        matches!(self, Self::All | Self::Tree)
    }

    const fn includes_nodes(self) -> bool {
        matches!(self, Self::All | Self::Nodes)
    }

    const fn includes_participation(self) -> bool {
        self.includes_tree() || self.includes_nodes()
    }

    const fn includes_transport(self) -> bool {
        matches!(self, Self::All | Self::Transport)
    }
}

#[derive(Clone, Debug, Args)]
struct ObserveOptions {
    /// Existing absolute navigation IPC namespace.
    #[arg(long)]
    ipc_root: PathBuf,
    /// Limit output to one section; the default shows all three.
    #[arg(long, value_enum, default_value = "all")]
    section: ObserveSection,
    /// Delay between samples; zero is one-shot in plain mode and 250 ms in the dashboard.
    #[arg(long, default_value_t = 0)]
    interval_ms: u64,
    /// Stop after this many samples; omitted with an interval watches indefinitely.
    #[arg(long)]
    samples: Option<u64>,
    /// Disable the interactive dashboard and emit stable key=value records.
    #[arg(long)]
    plain: bool,
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
        Command::Observe { options } => run_observe(options),
    }
}

fn run_inspect(ipc_root: PathBuf) -> Result<(), Box<dyn Error>> {
    let ipc = open_ipc(&ipc_root)?;
    let mut observer = ipc.attach_participation_observer()?;
    let sample = observer.try_observe_latest().ok_or_else(|| {
        io::Error::other(
            "no current navigation participation snapshot (missing, stale, or invalid)",
        )
    })?;
    print_participation_snapshot(sample.generation, &sample.value);
    Ok(())
}

fn open_ipc(ipc_root: &Path) -> Result<NavigationIpcNamespace, Box<dyn Error>> {
    if !ipc_root.is_absolute() {
        return Err("--ipc-root must be absolute".into());
    }
    let metadata = fs::metadata(ipc_root).map_err(|error| {
        io::Error::new(
            error.kind(),
            format!("navigation IPC namespace is missing: {ipc_root:?}"),
        )
    })?;
    if !metadata.is_dir() {
        return Err("--ipc-root must name an existing directory".into());
    }
    Ok(NavigationIpcNamespace::new(ipc_root)?)
}

fn print_participation_snapshot(generation: u64, snapshot: &NavigationParticipationSnapshot) {
    let header = snapshot.header();
    println!(
        "snapshot generation={} source={} boot={} sequence={} process_epoch={} command={}",
        generation,
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
}

fn run_observe(options: ObserveOptions) -> Result<(), Box<dyn Error>> {
    validate_observe_options(&options)?;
    let ipc = open_ipc(&options.ipc_root)?;
    let section = options.section;
    let mut participation_observer = section
        .includes_participation()
        .then(|| ipc.attach_participation_observer())
        .transpose()?;
    let mut transport_observer = section
        .includes_transport()
        .then(|| ipc.attach_transport_diagnostics_observer())
        .transpose()?;
    if !options.plain && io::stdout().is_terminal() {
        return dashboard::run(
            dashboard::DashboardConfig {
                show_tree: section.includes_tree(),
                show_nodes: section.includes_nodes(),
                show_transport: section.includes_transport(),
                interval: Duration::from_millis(if options.interval_ms == 0 {
                    dashboard::DEFAULT_REFRESH_MS
                } else {
                    options.interval_ms
                }),
                sample_limit: options.samples,
            },
            participation_observer,
            transport_observer,
        );
    }
    run_observation(options, |sample_index| {
        let participation_sample = participation_observer
            .as_mut()
            .map(|observer| {
                observer.try_observe_latest().ok_or_else(|| {
                    io::Error::other(
                        "no current navigation participation snapshot (missing, stale, or invalid)",
                    )
                })
            })
            .transpose()?;
        let transport_sample = transport_observer
            .as_mut()
            .map(|observer| {
                observer.try_observe_latest().ok_or_else(|| {
                    io::Error::other(
                        "no current navigation transport diagnostics snapshot (missing, stale, or invalid)",
                    )
                })
            })
            .transpose()?;

        println!("sample={sample_index}");
        if section.includes_tree() {
            let sample = participation_sample
                .as_ref()
                .expect("tree section attached its participation observer");
            print_tree_section(sample.generation, &sample.value)?;
        }
        if section.includes_nodes() {
            let sample = participation_sample
                .as_ref()
                .expect("node section attached its participation observer");
            println!("section=nodes");
            print_participation_snapshot(sample.generation, &sample.value);
        }
        if section.includes_transport() {
            let sample = transport_sample
                .as_ref()
                .expect("transport section attached its diagnostic observer");
            print_transport_section(sample.generation, &sample.value);
        }
        Ok(())
    })
}

fn run_observation<F>(options: ObserveOptions, mut observe: F) -> Result<(), Box<dyn Error>>
where
    F: FnMut(u64) -> Result<(), Box<dyn Error>>,
{
    let sample_limit = if options.interval_ms == 0 {
        Some(1)
    } else {
        options.samples
    };
    let interval = Duration::from_millis(options.interval_ms);
    let mut sample_index = 1_u64;
    loop {
        observe(sample_index)?;
        io::stdout().flush()?;
        if sample_limit.is_some_and(|limit| sample_index >= limit) {
            return Ok(());
        }
        thread::sleep(interval);
        sample_index = sample_index.saturating_add(1);
    }
}

fn validate_observe_options(options: &ObserveOptions) -> Result<(), Box<dyn Error>> {
    if options.samples == Some(0) {
        return Err("--samples must be nonzero".into());
    }
    if (options.plain || !io::stdout().is_terminal())
        && options.interval_ms == 0
        && options.samples.is_some_and(|samples| samples > 1)
    {
        return Err("--samples greater than 1 requires nonzero --interval-ms".into());
    }
    Ok(())
}

fn print_tree_section(
    generation: u64,
    snapshot: &NavigationParticipationSnapshot,
) -> Result<(), Box<dyn Error>> {
    let root = snapshot
        .participants()
        .iter()
        .find(|participant| participant.id().as_str() == "tree.root")
        .ok_or_else(|| io::Error::other("participation snapshot has no tree.root participant"))?;
    let header = snapshot.header();
    println!("section=tree");
    println!(
        "tree generation={} source={} boot={} sequence={} process_epoch={} command={} state={} reason={}",
        generation,
        header.source_id(),
        id128_text(header.boot_id()),
        header.sequence(),
        snapshot.process_epoch(),
        snapshot
            .command_id()
            .map_or_else(|| "-".to_owned(), id128_text),
        root.state().as_str(),
        root.reason().as_str(),
    );
    for participant in snapshot
        .participants()
        .iter()
        .filter(|participant| participant.id().as_str().starts_with("tree."))
    {
        let depth = u8::from(participant.id().as_str() != "tree.root");
        println!(
            "tree_node depth={depth} id={} state={} reason={}",
            participant.id(),
            participant.state().as_str(),
            participant.reason().as_str(),
        );
    }
    Ok(())
}

fn print_transport_section(generation: u64, snapshot: &NavigationTransportDiagnosticsSnapshot) {
    let header = snapshot.header();
    println!("section=transport");
    println!(
        "transport generation={} source={} boot={} sequence={} process_epoch={} command={}",
        generation,
        header.source_id(),
        id128_text(header.boot_id()),
        header.sequence(),
        snapshot.process_epoch(),
        snapshot
            .command_id()
            .map_or_else(|| "-".to_owned(), id128_text),
    );
    for channel in snapshot.channels() {
        let counters = channel.counters();
        let diagnostics = channel.shm_diagnostics();
        println!(
            "channel id={} published={} consumed={} overwritten={} full={} disconnected={} attach_rejected={} read_races={} decode_rejected={} reclaimed_endpoints={} skipped_backlog={}",
            channel.channel_id(),
            counters.published,
            counters.consumed,
            counters.overwritten,
            counters.full,
            counters.disconnected,
            diagnostics.attach_rejected,
            diagnostics.read_races,
            diagnostics.decode_rejected,
            diagnostics.reclaimed_endpoints,
            diagnostics.skipped_backlog,
        );
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
