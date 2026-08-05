use clap::{Parser, Subcommand};
use navigation_plugin_config::{AssetRole, NavigationPluginManifest};
use std::{
    collections::BTreeMap, error::Error, path::PathBuf, process::ExitCode, str::FromStr as _,
};

#[derive(Debug, Parser)]
#[command(
    name = "rs-navigation-plugin",
    version,
    about = "Validate the motor-disabled chassis navigation plugin contract"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Validate both manifest copies and every bound external asset.
    Validate {
        /// Canonical local manifest.
        manifest: PathBuf,
        /// Byte-identical manifest in the chassis_nav repository/install.
        #[arg(long)]
        peer_manifest: PathBuf,
        /// Required ROLE=ABSOLUTE_PATH asset binding; specify every role once.
        #[arg(long = "asset")]
        assets: Vec<String>,
    },
    /// Print the exact-byte SHA-256 of a valid manifest.
    Fingerprint {
        /// Manifest path.
        manifest: PathBuf,
    },
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("rs-navigation-plugin: {error}");
            ExitCode::FAILURE
        }
    }
}

fn run(cli: Cli) -> Result<(), Box<dyn Error>> {
    match cli.command {
        Command::Validate {
            manifest,
            peer_manifest,
            assets,
        } => {
            let parsed = NavigationPluginManifest::load(&manifest)?;
            let peer_fingerprint =
                NavigationPluginManifest::verify_peer_copy(&manifest, &peer_manifest)?;
            if parsed.fingerprint() != peer_fingerprint {
                return Err("peer fingerprint changed between checks".into());
            }
            let assets = parse_assets(assets)?;
            parsed.verify_assets(&assets)?;
            println!(
                "valid schema_version={} plugin_id={} profile_id={} sha256={}",
                parsed.schema_version(),
                parsed.plugin_id(),
                parsed.profile_id(),
                parsed.fingerprint()
            );
        }
        Command::Fingerprint { manifest } => {
            println!(
                "{}",
                NavigationPluginManifest::load(&manifest)?.fingerprint()
            );
        }
    }
    Ok(())
}

fn parse_assets(values: Vec<String>) -> Result<BTreeMap<AssetRole, PathBuf>, Box<dyn Error>> {
    let mut result = BTreeMap::new();
    for value in values {
        let (role, path) = value
            .split_once('=')
            .ok_or("asset must use ROLE=ABSOLUTE_PATH")?;
        let role = AssetRole::from_str(role)?;
        if result.insert(role, PathBuf::from(path)).is_some() {
            return Err(format!("duplicate asset role {role}").into());
        }
    }
    Ok(result)
}
