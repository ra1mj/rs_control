//! Command-line entry point for the host-only arm simulation.

use std::{fs, io::Write, path::PathBuf, process::ExitCode};

use clap::{Parser, Subcommand};
use rs_arm_simulation::run_qualification;

#[derive(Parser)]
#[command(
    name = "rs-arm-simulation",
    about = "Host-only adaptive arm-gravity qualification"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Run the deterministic qualification and print or write its report.
    Qualify {
        #[arg(long)]
        output: Option<PathBuf>,
    },
    /// Re-run and byte-compare with a checked-in canonical report.
    Verify {
        #[arg(value_name = "REPORT")]
        report: PathBuf,
    },
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("rs-arm-simulation: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run(cli: Cli) -> Result<(), Box<dyn std::error::Error>> {
    match cli.command {
        Command::Qualify { output } => {
            let report = run_qualification()?;
            let json = report.canonical_json()?;
            if let Some(path) = output {
                let mut file = fs::OpenOptions::new()
                    .write(true)
                    .create_new(true)
                    .open(path)?;
                file.write_all(json.as_bytes())?;
            } else {
                print!("{json}");
            }
            if !report.gates.all_passed {
                return Err("one or more qualification gates failed".into());
            }
        }
        Command::Verify { report } => {
            let expected = fs::read_to_string(&report)?;
            let actual_report = run_qualification()?;
            let actual = actual_report.canonical_json()?;
            if expected != actual {
                return Err(format!(
                    "{} differs from a fresh deterministic run",
                    report.display()
                )
                .into());
            }
            if !actual_report.gates.all_passed {
                return Err("checked-in report contains a failed gate".into());
            }
            println!(
                "verified {} ({})",
                report.display(),
                actual_report.functional_sha256
            );
        }
    }
    Ok(())
}
