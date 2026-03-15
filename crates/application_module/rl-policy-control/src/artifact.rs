//! Bounded immutable policy artifact loading and morphology binding.

use std::collections::BTreeSet;
use std::fs;
use std::num::NonZeroU64;
use std::path::Path;
use std::sync::atomic::{AtomicU64, Ordering};

use arm_control::ArmControlConfig;
use message_model::ArmProfileId;
use sha2::{Digest, Sha256};

use crate::graph::{ValidatedPolicyGraph, validate_policy_graph};
use crate::{MAX_POLICY_MANIFEST_BYTES, PolicyActionKind, PolicyError, PolicyManifest};

const MANIFEST_FILE: &str = "manifest.toml";
const MODEL_FILE: &str = "model.onnx";

static NEXT_POLICY_GENERATION: AtomicU64 = AtomicU64::new(1);

/// SHA-256 identity over the exact manifest bytes and model digest.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct PolicyIdentity([u8; 32]);

impl PolicyIdentity {
    /// Returns canonical digest bytes.
    #[must_use]
    pub const fn as_bytes(self) -> [u8; 32] {
        self.0
    }
}

/// Nonzero process-local load generation.
#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
pub struct PolicyGeneration(NonZeroU64);

impl PolicyGeneration {
    /// Returns the process-local generation number.
    #[must_use]
    pub const fn get(self) -> u64 {
        self.0.get()
    }
}

/// Fully validated policy ready to be moved into an inference engine.
pub struct ReadyPolicy {
    manifest: PolicyManifest,
    identity: PolicyIdentity,
    generation: PolicyGeneration,
    graph: ValidatedPolicyGraph,
    control: ArmControlConfig,
}

impl ReadyPolicy {
    /// Load exactly `manifest.toml` and `model.onnx`, then bind the graph to a
    /// compiled arm-control configuration.
    pub fn load(directory: &Path, control: &ArmControlConfig) -> Result<Self, PolicyError> {
        let (manifest_bytes, model_bytes) = load_artifact_files(directory)?;
        let manifest = PolicyManifest::parse(&manifest_bytes)?;
        if model_bytes.len() as u64 > manifest.max_model_bytes {
            return Err(PolicyError::Artifact(
                "model exceeds manifest max_model_bytes".into(),
            ));
        }
        let model_digest: [u8; 32] = Sha256::digest(&model_bytes).into();
        if encode_hex(&model_digest) != manifest.model_sha256 {
            return Err(PolicyError::Artifact("model SHA-256 mismatch".into()));
        }
        validate_binding(&manifest, control)?;
        let graph = validate_policy_graph(&model_bytes, &manifest)?;

        let mut identity_hasher = Sha256::new();
        identity_hasher.update(b"rs-control-policy-v1\0");
        identity_hasher.update(&manifest_bytes);
        identity_hasher.update(model_digest);
        let identity = PolicyIdentity(identity_hasher.finalize().into());
        let generation = next_generation()?;

        Ok(Self {
            manifest,
            identity,
            generation,
            graph,
            control: control.clone(),
        })
    }

    /// Returns the strict manifest.
    #[must_use]
    pub const fn manifest(&self) -> &PolicyManifest {
        &self.manifest
    }

    /// Returns the immutable artifact identity.
    #[must_use]
    pub const fn identity(&self) -> PolicyIdentity {
        self.identity
    }

    /// Returns the process-local load generation.
    #[must_use]
    pub const fn generation(&self) -> PolicyGeneration {
        self.generation
    }

    /// Returns the bound control contract.
    #[must_use]
    pub const fn control(&self) -> &ArmControlConfig {
        &self.control
    }

    pub(crate) fn into_parts(
        self,
    ) -> (
        PolicyManifest,
        PolicyIdentity,
        PolicyGeneration,
        ValidatedPolicyGraph,
        ArmControlConfig,
    ) {
        (
            self.manifest,
            self.identity,
            self.generation,
            self.graph,
            self.control,
        )
    }
}

fn load_artifact_files(directory: &Path) -> Result<(Vec<u8>, Vec<u8>), PolicyError> {
    let metadata = fs::symlink_metadata(directory)
        .map_err(|error| PolicyError::Artifact(error.to_string()))?;
    if !metadata.file_type().is_dir() || metadata.file_type().is_symlink() {
        return Err(PolicyError::Artifact(
            "artifact path must be a real directory".into(),
        ));
    }
    let mut names = BTreeSet::new();
    for entry in
        fs::read_dir(directory).map_err(|error| PolicyError::Artifact(error.to_string()))?
    {
        let entry = entry.map_err(|error| PolicyError::Artifact(error.to_string()))?;
        let name = entry
            .file_name()
            .into_string()
            .map_err(|_| PolicyError::Artifact("non-UTF-8 artifact filename".into()))?;
        if !names.insert(name) {
            return Err(PolicyError::Artifact(
                "duplicate artifact directory entry".into(),
            ));
        }
    }
    let expected = BTreeSet::from([MANIFEST_FILE.to_string(), MODEL_FILE.to_string()]);
    if names != expected {
        return Err(PolicyError::Artifact(
            "artifact directory must contain exactly manifest.toml and model.onnx".into(),
        ));
    }
    let manifest = read_bounded_regular(
        &directory.join(MANIFEST_FILE),
        MAX_POLICY_MANIFEST_BYTES,
        "manifest",
    )?;
    // The global model bound is checked here; the narrower manifest bound is
    // checked immediately after strict parsing.
    let model = read_bounded_regular(
        &directory.join(MODEL_FILE),
        crate::MAX_POLICY_MODEL_BYTES,
        "model",
    )?;
    Ok((manifest, model))
}

fn read_bounded_regular(path: &Path, maximum: u64, field: &str) -> Result<Vec<u8>, PolicyError> {
    let metadata =
        fs::symlink_metadata(path).map_err(|error| PolicyError::Artifact(error.to_string()))?;
    if !metadata.file_type().is_file() || metadata.file_type().is_symlink() {
        return Err(PolicyError::Artifact(format!(
            "{field} must be a regular non-symlink file"
        )));
    }
    if metadata.len() == 0 || metadata.len() > maximum {
        return Err(PolicyError::Artifact(format!(
            "{field} size is outside its bound"
        )));
    }
    let bytes = fs::read(path).map_err(|error| PolicyError::Artifact(error.to_string()))?;
    if bytes.len() as u64 != metadata.len() {
        return Err(PolicyError::Artifact(format!(
            "{field} changed while loading"
        )));
    }
    Ok(bytes)
}

fn validate_binding(
    manifest: &PolicyManifest,
    control: &ArmControlConfig,
) -> Result<(), PolicyError> {
    let profile_bytes = decode_hex_digest(&manifest.arm_profile_sha256)?;
    let profile = ArmProfileId::new(profile_bytes)
        .map_err(|error| PolicyError::Binding(error.to_string()))?;
    if profile != control.profile_id()
        || manifest.kinematic_group_id != control.group_id().as_str()
        || manifest.joint_order.len() != control.joint_order().len()
        || manifest.actuator_order.len() != control.actuator_order().len()
    {
        return Err(PolicyError::Binding(
            "profile, group or degree of freedom differs".into(),
        ));
    }
    for ((joint, actuator), (descriptor, binding)) in manifest
        .joint_order
        .iter()
        .zip(&manifest.actuator_order)
        .zip(
            control
                .joint_order()
                .joints()
                .iter()
                .zip(control.actuator_order().bindings()),
        )
    {
        if joint != descriptor.joint_id().as_str()
            || actuator != binding.actuator_id().as_str()
            || descriptor.joint_id() != binding.joint_id()
        {
            return Err(PolicyError::Binding(
                "joint or actuator order differs from compiled binding".into(),
            ));
        }
    }
    validate_action_caps(manifest, control)
}

fn validate_action_caps(
    manifest: &PolicyManifest,
    control: &ArmControlConfig,
) -> Result<(), PolicyError> {
    let action = &manifest.action;
    if action.kind == PolicyActionKind::Mit
        && control
            .joint_order()
            .joints()
            .iter()
            .any(|joint| joint.kind() == message_model::ArmJointKind::Prismatic)
    {
        return Err(PolicyError::Binding(
            "MIT requires a complete rotary kinematic group".into(),
        ));
    }
    for (index, limits) in control.limits().iter().copied().enumerate() {
        let fields: &[(&str, usize, f64, f64)] = match action.kind {
            PolicyActionKind::Mit => &[
                (
                    "position",
                    index * 3,
                    limits.maximum_position_rad(),
                    limits.maximum_position_slew_radps(),
                ),
                (
                    "velocity",
                    index * 3 + 1,
                    limits.maximum_velocity_radps(),
                    limits.maximum_velocity_slew_radps2(),
                ),
                (
                    "torque",
                    index * 3 + 2,
                    limits.maximum_torque_nm(),
                    limits.maximum_torque_slew_nmps(),
                ),
            ],
            PolicyActionKind::DirectRotaryTorque => &[(
                "torque",
                index,
                limits.maximum_torque_nm(),
                limits.maximum_torque_slew_nmps(),
            )],
            _ => continue,
        };
        for &(field, output_index, maximum, maximum_slew) in fields {
            if action.command_minimum[output_index] < -maximum
                || action.command_maximum[output_index] > maximum
                || action.slew_per_second[output_index] > maximum_slew
            {
                return Err(PolicyError::Binding(format!(
                    "{field} command or slew bound exceeds arm-control cap"
                )));
            }
        }
        if action.kind == PolicyActionKind::Mit
            && (action.fixed_kp[index] != limits.fixed_kp()
                || action.fixed_kd[index] != limits.fixed_kd())
        {
            return Err(PolicyError::Binding(
                "MIT fixed gains differ from arm-control configuration".into(),
            ));
        }
    }
    Ok(())
}

fn next_generation() -> Result<PolicyGeneration, PolicyError> {
    let generation = NEXT_POLICY_GENERATION
        .fetch_update(Ordering::Relaxed, Ordering::Relaxed, |generation| {
            generation.checked_add(1)
        })
        .map_err(|_| PolicyError::Artifact("policy generation exhausted".into()))?;
    NonZeroU64::new(generation)
        .map(PolicyGeneration)
        .ok_or_else(|| PolicyError::Artifact("policy generation exhausted".into()))
}

pub(crate) fn decode_hex_digest(value: &str) -> Result<[u8; 32], PolicyError> {
    let mut output = [0u8; 32];
    if value.len() != 64 {
        return Err(PolicyError::Manifest("invalid SHA-256 length".into()));
    }
    for (index, byte) in output.iter_mut().enumerate() {
        let pair = &value[index * 2..index * 2 + 2];
        *byte = u8::from_str_radix(pair, 16)
            .map_err(|_| PolicyError::Manifest("invalid SHA-256 encoding".into()))?;
    }
    Ok(output)
}

pub(crate) fn encode_hex(value: &[u8; 32]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut encoded = String::with_capacity(64);
    for byte in value {
        encoded.push(char::from(HEX[usize::from(byte >> 4)]));
        encoded.push(char::from(HEX[usize::from(byte & 0x0f)]));
    }
    encoded
}

#[cfg(test)]
mod tests {
    use arm_control::{ActuatorControlLimits, ArmControlConfig};
    use arm_model::{
        ARM_PROFILE_SCHEMA_VERSION, ArmModel, ArmProfileClass, ArmProfileSpec,
        JointAccelerationLimitSpec, KinematicGroupSpec, RotaryTransmissionSpec,
    };
    use message_model::{AsciiId, Id128};

    use super::validate_action_caps;
    use crate::PolicyManifest;

    #[test]
    fn mit_rejects_prismatic_group_before_inference() {
        let joint = AsciiId::parse("slide").unwrap();
        let model = ArmModel::compile_urdf(
            r#"
<robot name="linear">
  <link name="base"/>
  <link name="tool"/>
  <joint name="slide" type="prismatic">
    <parent link="base"/>
    <child link="tool"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1" upper="1" effort="20" velocity="2"/>
  </joint>
</robot>
"#,
            &ArmProfileSpec {
                schema_version: ARM_PROFILE_SCHEMA_VERSION,
                name: AsciiId::parse("linear").unwrap(),
                class: ArmProfileClass::HostFixture,
                groups: vec![KinematicGroupSpec {
                    id: AsciiId::parse("arm").unwrap(),
                    base_link: AsciiId::parse("base").unwrap(),
                    tip_link: AsciiId::parse("tool").unwrap(),
                    active_joint_order: vec![joint],
                }],
                acceleration_limits: vec![JointAccelerationLimitSpec {
                    joint_id: joint,
                    maximum: 4.0,
                }],
                transmissions: vec![RotaryTransmissionSpec {
                    joint_id: joint,
                    actuator_id: AsciiId::parse("linear_motor").unwrap(),
                    scale_rad_per_joint_unit: 2.0,
                    offset_rad: 0.0,
                }],
            },
        )
        .unwrap();
        let transmission = model.transmissions()[0];
        let control = ArmControlConfig::new(
            &model,
            AsciiId::parse("arm").unwrap(),
            1,
            AsciiId::parse("state").unwrap(),
            Id128::new([1; 16]).unwrap(),
            vec![
                ActuatorControlLimits::new(
                    transmission.binding(),
                    2.0,
                    4.0,
                    10.0,
                    20.0,
                    0.5,
                    10.0,
                    20.0,
                    50.0,
                )
                .unwrap(),
            ],
        )
        .unwrap();
        let manifest = PolicyManifest::parse(include_bytes!(
            "../tests/fixtures/mit_identity/manifest.toml"
        ))
        .unwrap();
        assert!(validate_action_caps(&manifest, &control).is_err());
    }
}
