//! Configuration-only operator utility. This binary performs no hardware access.

use clap::{Parser, Subcommand};
use config_model::{MAX_CONFIG_BYTES, RobotConfig};
use std::{
    error::Error,
    fs::File,
    io::Read as _,
    path::{Path, PathBuf},
    process::ExitCode,
};

#[derive(Debug, Parser)]
#[command(
    name = "robotctl",
    version,
    about = "Robot configuration and maintenance utility"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Validate or normalize the minimal robot YAML.
    Config {
        #[command(subcommand)]
        command: ConfigCommand,
    },
}

#[derive(Debug, Subcommand)]
enum ConfigCommand {
    /// Validate a robot YAML and print its stable SHA-256 identity.
    Validate {
        /// Robot YAML path.
        path: PathBuf,
    },
    /// Print deterministic normalized YAML.
    PrintNormalized {
        /// Robot YAML path.
        path: PathBuf,
    },
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("robotctl: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run(cli: Cli) -> Result<(), Box<dyn Error>> {
    match cli.command {
        Command::Config { command } => match command {
            ConfigCommand::Validate { path } => {
                let config = load_config(&path)?;
                println!(
                    "valid schema_version={} sha256={}",
                    config.schema_version(),
                    config.normalized_hash()
                );
            }
            ConfigCommand::PrintNormalized { path } => {
                let config = load_config(&path)?;
                print!("{}", config.normalized_yaml());
            }
        },
    }
    Ok(())
}

fn load_config(path: &Path) -> Result<RobotConfig, Box<dyn Error>> {
    let file = File::open(path)?;
    let mut yaml = String::new();
    file.take((MAX_CONFIG_BYTES + 1) as u64)
        .read_to_string(&mut yaml)?;
    RobotConfig::parse(&yaml).map_err(|error| Box::new(error) as Box<dyn Error>)
}
