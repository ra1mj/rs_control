use std::process::Command;

const EXPECTED_STDOUT: &[u8] = b"8439712902066387584\n";

#[test]
fn size_candidates_emit_the_same_stable_checksum() {
    for binary in [
        env!("CARGO_BIN_EXE_direct-enum"),
        env!("CARGO_BIN_EXE_statig"),
    ] {
        let output = Command::new(binary).output().unwrap();
        assert!(output.status.success());
        assert_eq!(output.stdout, EXPECTED_STDOUT);
        assert!(output.stderr.is_empty());
    }
}
