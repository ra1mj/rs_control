//! Generated Protocol Buffers types for the canonical external contract.
//!
//! The generated Rust is build output and must never be edited manually.

/// Robot v1 wire contract.
pub mod robot {
    /// Version 1 messages.
    pub mod v1 {
        include!(concat!(env!("OUT_DIR"), "/robot.v1.rs"));
    }
}

/// Encoded `google.protobuf.FileDescriptorSet` for every robot v1 schema.
pub const FILE_DESCRIPTOR_SET: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/robot-v1.binpb"));

#[cfg(test)]
mod tests {
    use super::FILE_DESCRIPTOR_SET;

    #[test]
    fn descriptor_set_is_generated() {
        assert!(!FILE_DESCRIPTOR_SET.is_empty());
    }
}
