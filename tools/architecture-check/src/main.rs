use architecture_check::{graph_from_cargo, load_policy, validate};
use clap::{Parser, Subcommand};
use std::{error::Error, path::PathBuf, process::ExitCode};

#[derive(Debug, Parser)]
#[command(
    name = "architecture-check",
    version,
    about = "Validate robot crate dependency boundaries"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Validate the current Cargo workspace against its architecture policy.
    Check {
        /// Workspace root containing Cargo.toml.
        #[arg(long, default_value = ".")]
        workspace_root: PathBuf,
        /// Architecture policy path, relative to the workspace root by default.
        #[arg(long, default_value = "architecture.toml")]
        policy: PathBuf,
    },
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("architecture-check: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run(cli: Cli) -> Result<(), Box<dyn Error>> {
    match cli.command {
        Command::Check {
            workspace_root,
            policy,
        } => {
            let policy_path = if policy.is_absolute() {
                policy
            } else {
                workspace_root.join(policy)
            };
            let policy = load_policy(&policy_path)?;
            let graph = graph_from_cargo(&workspace_root)?;
            let violations = validate(&policy, &graph);
            if violations.is_empty() {
                println!(
                    "architecture check passed: {} packages, {} internal dependencies",
                    graph.packages.len(),
                    graph.dependencies.len()
                );
                return Ok(());
            }
            let rendered = violations
                .iter()
                .map(|violation| format!("- {violation}"))
                .collect::<Vec<_>>()
                .join("\n");
            Err(format!("{} violation(s):\n{rendered}", violations.len()).into())
        }
    }
}
