//! Smoke-test standard C++ and Python generation from the canonical schemas.

use std::{
    error::Error,
    ffi::OsStr,
    fs, io,
    path::{Path, PathBuf},
    process::Command,
};

fn main() -> Result<(), Box<dyn Error>> {
    let workspace_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let proto_root = workspace_root.join("proto");
    let proto_files = collect_proto_files(&proto_root.join("robot/v1"))?;
    let output_root = tempfile::tempdir()?;
    let cpp_root = output_root.path().join("cpp");
    let python_root = output_root.path().join("python");
    fs::create_dir_all(&cpp_root)?;
    fs::create_dir_all(&python_root)?;

    run_protoc(&proto_root, &proto_files, "cpp_out", &cpp_root)?;
    run_protoc(&proto_root, &proto_files, "python_out", &python_root)?;

    for proto in &proto_files {
        let stem = proto
            .file_stem()
            .and_then(OsStr::to_str)
            .ok_or_else(|| io::Error::new(io::ErrorKind::InvalidData, "invalid proto filename"))?;
        require_nonempty(&cpp_root.join(format!("robot/v1/{stem}.pb.h")))?;
        require_nonempty(&cpp_root.join(format!("robot/v1/{stem}.pb.cc")))?;
        require_nonempty(&python_root.join(format!("robot/v1/{stem}_pb2.py")))?;
    }

    println!(
        "protobuf compatibility check passed: {} schemas generated for C++ and Python",
        proto_files.len()
    );
    Ok(())
}

fn collect_proto_files(directory: &Path) -> Result<Vec<PathBuf>, Box<dyn Error>> {
    let entries = fs::read_dir(directory)?
        .map(|entry| entry.map(|value| value.path()))
        .collect::<Result<Vec<_>, _>>()?;
    let mut files = entries
        .into_iter()
        .filter(|path| path.extension() == Some(OsStr::new("proto")))
        .collect::<Vec<_>>();
    files.sort();
    if files.is_empty() {
        return Err(io::Error::new(io::ErrorKind::NotFound, "no robot v1 schemas found").into());
    }
    Ok(files)
}

fn run_protoc(
    proto_root: &Path,
    proto_files: &[PathBuf],
    output_flag: &str,
    output_root: &Path,
) -> Result<(), Box<dyn Error>> {
    let output = Command::new(protoc_bin_vendored::protoc_bin_path()?)
        .arg(format!("--proto_path={}", proto_root.display()))
        .arg(format!("--{output_flag}={}", output_root.display()))
        .args(proto_files)
        .output()?;
    if output.status.success() {
        return Ok(());
    }
    Err(io::Error::other(format!(
        "protoc {output_flag} failed: {}",
        String::from_utf8_lossy(&output.stderr)
    ))
    .into())
}

fn require_nonempty(path: &Path) -> Result<(), Box<dyn Error>> {
    let metadata = fs::metadata(path)?;
    if metadata.len() == 0 {
        return Err(io::Error::new(
            io::ErrorKind::InvalidData,
            format!("generated file is empty: {}", path.display()),
        )
        .into());
    }
    Ok(())
}
