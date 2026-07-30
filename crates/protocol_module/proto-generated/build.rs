use std::{env, error::Error, ffi::OsStr, fs, path::PathBuf};

fn main() -> Result<(), Box<dyn Error>> {
    let crate_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR")?);
    let workspace_root = crate_dir.join("../../..");
    let proto_root = workspace_root.join("proto");
    let schema_dir = proto_root.join("robot/v1");
    let entries = fs::read_dir(&schema_dir)?
        .map(|entry| entry.map(|value| value.path()))
        .collect::<Result<Vec<_>, _>>()?;
    let mut protos = entries
        .into_iter()
        .filter(|path| path.extension() == Some(OsStr::new("proto")))
        .collect::<Vec<_>>();
    protos.sort();
    if protos.is_empty() {
        return Err(format!("no .proto files found in {}", schema_dir.display()).into());
    }

    for proto in &protos {
        println!("cargo:rerun-if-changed={}", proto.display());
    }
    println!("cargo:rerun-if-changed={}", proto_root.display());

    let descriptor_path = PathBuf::from(env::var("OUT_DIR")?).join("robot-v1.binpb");
    let mut config = prost_build::Config::new();
    config.protoc_executable(protoc_bin_vendored::protoc_bin_path()?);
    config.file_descriptor_set_path(descriptor_path);
    config.compile_protos(&protos, &[proto_root])?;
    Ok(())
}
