//! Strict immutable TOML policy contract.

use std::collections::BTreeSet;

use serde::Deserialize;

use crate::{
    MAX_POLICY_ELEMENTS, MAX_POLICY_MODEL_BYTES, MAX_POLICY_QUEUE_CAPACITY,
    MAX_POLICY_RECURRENT_ELEMENTS, MAX_POLICY_TENSORS, MAX_POLICY_THREADS, PolicyError,
};

/// Supported manifest schema.
pub const POLICY_MANIFEST_SCHEMA_VERSION: u32 = 1;
/// Exact production inference backend.
pub const POLICY_RUNTIME: &str = "rten";
/// Exact admitted RTen version.
pub const POLICY_RUNTIME_VERSION: &str = "0.24.0";

/// Exact static `f32` tensor contract.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct TensorContract {
    /// Unique graph node name.
    pub name: String,
    /// Must be `f32`.
    pub dtype: String,
    /// Fully static nonzero shape.
    pub shape: Vec<usize>,
}

impl TensorContract {
    /// Returns checked element count.
    pub fn element_count(&self) -> Result<usize, PolicyError> {
        if self.dtype != "f32" || self.name.is_empty() || self.shape.is_empty() {
            return Err(PolicyError::Manifest(
                "tensor requires a name, dtype=f32 and a nonempty shape".into(),
            ));
        }
        let count = self.shape.iter().try_fold(1usize, |count, &dimension| {
            if dimension == 0 {
                None
            } else {
                count.checked_mul(dimension)
            }
        });
        match count {
            Some(count) if count <= MAX_POLICY_ELEMENTS => Ok(count),
            _ => Err(PolicyError::Manifest(
                "tensor shape is empty, overflows or exceeds policy capacity".into(),
            )),
        }
    }
}

/// One semantic scalar read from every joint in stable order.
#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd)]
#[serde(rename_all = "snake_case")]
pub enum ObservationField {
    /// Joint coordinate in radians/metres.
    Position,
    /// Joint velocity in radians/s or metres/s.
    Velocity,
    /// Joint torque/force in N·m/N.
    Effort,
}

/// One ordered, normalized observation scalar.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ObservationElement {
    /// Stable joint identifier.
    pub joint: String,
    /// Semantic source field.
    pub field: ObservationField,
    /// Explicit SI unit name.
    pub unit: String,
    /// Normalization mean.
    pub mean: f64,
    /// Reciprocal standard deviation, strictly positive.
    pub inverse_std: f64,
    /// Inclusive hard source lower bound.
    pub minimum: f64,
    /// Inclusive hard source upper bound.
    pub maximum: f64,
}

/// Closed policy action semantics.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum PolicyActionKind {
    /// Absolute joint position.
    AbsolutePosition,
    /// Joint-position residual.
    PositionResidual,
    /// Joint velocity.
    JointVelocity,
    /// Direct actuator torque.
    DirectRotaryTorque,
    /// Fixed-gain MIT impedance tuple.
    Mit,
}

/// Strict action scaling and safety contract.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ActionManifest {
    /// Selected semantic.
    pub kind: PolicyActionKind,
    /// Exact output tensor name.
    pub output: String,
    /// Per-output affine scale.
    pub scale: Vec<f64>,
    /// Per-output affine offset.
    pub offset: Vec<f64>,
    /// Hard reject lower bounds.
    pub hard_minimum: Vec<f64>,
    /// Hard reject upper bounds.
    pub hard_maximum: Vec<f64>,
    /// Narrower projected command lower bounds.
    pub command_minimum: Vec<f64>,
    /// Narrower projected command upper bounds.
    pub command_maximum: Vec<f64>,
    /// Maximum accepted change per second.
    pub slew_per_second: Vec<f64>,
    /// Immutable per-actuator MIT proportional gains.
    #[serde(default)]
    pub fixed_kp: Vec<f64>,
    /// Immutable per-actuator MIT derivative gains.
    #[serde(default)]
    pub fixed_kd: Vec<f64>,
}

/// Paired private recurrent tensors with an explicit reset vector.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct RecurrentTensorPair {
    /// Exact recurrent input tensor name.
    pub input: String,
    /// Exact recurrent output tensor name.
    pub output: String,
    /// Finite flattened reset state.
    pub reset: Vec<f32>,
}

/// Only accepted failure behavior.
#[derive(Clone, Copy, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum FallbackManifest {
    /// Publish no proposal.
    WithdrawProposal,
}

/// Complete versioned policy manifest.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct PolicyManifest {
    /// Must be one.
    pub schema_version: u32,
    /// Must be `rten`.
    pub runtime: String,
    /// Must be `0.24.0`.
    pub runtime_version: String,
    /// Must be `onnx`.
    pub model_format: String,
    /// Lowercase hexadecimal model SHA-256.
    pub model_sha256: String,
    /// Artifact-specific model bound.
    pub max_model_bytes: u64,
    /// Lowercase hexadecimal compiled arm-profile SHA-256.
    pub arm_profile_sha256: String,
    /// Exact named kinematic group.
    pub kinematic_group_id: String,
    /// Stable compiled joint identifiers.
    pub joint_order: Vec<String>,
    /// Stable compiled actuator identifiers.
    pub actuator_order: Vec<String>,
    /// Exact model inputs.
    pub inputs: Vec<TensorContract>,
    /// Exact model outputs.
    pub outputs: Vec<TensorContract>,
    /// Exact scalar encoding order.
    pub observations: Vec<ObservationElement>,
    /// Action decode contract.
    pub action: ActionManifest,
    /// Optional private recurrent tensor pairs.
    #[serde(default)]
    pub recurrent: Vec<RecurrentTensorPair>,
    /// Minimum request cadence.
    pub inference_period_ns: u64,
    /// Exclusive request/result age bound.
    pub inference_deadline_ns: u64,
    /// Both SPSC queue capacities.
    pub queue_capacity: usize,
    /// Dedicated RTen pool size.
    pub thread_count: usize,
    /// Failure behavior.
    pub fallback: FallbackManifest,
}

impl PolicyManifest {
    /// Strictly parse and validate backend-independent invariants.
    pub fn parse(bytes: &[u8]) -> Result<Self, PolicyError> {
        let source =
            std::str::from_utf8(bytes).map_err(|error| PolicyError::Manifest(error.to_string()))?;
        let manifest: Self =
            toml::from_str(source).map_err(|error| PolicyError::Manifest(error.to_string()))?;
        manifest.validate()?;
        Ok(manifest)
    }

    /// Validate closed enums, bounds and exact vector dimensions.
    pub fn validate(&self) -> Result<(), PolicyError> {
        if self.schema_version != POLICY_MANIFEST_SCHEMA_VERSION
            || self.runtime != POLICY_RUNTIME
            || self.runtime_version != POLICY_RUNTIME_VERSION
            || self.model_format != "onnx"
            || self.fallback != FallbackManifest::WithdrawProposal
        {
            return Err(PolicyError::Manifest(
                "unsupported schema, runtime, model format or fallback".into(),
            ));
        }
        if self.max_model_bytes == 0 || self.max_model_bytes > MAX_POLICY_MODEL_BYTES {
            return Err(PolicyError::Manifest(
                "max_model_bytes is outside the production bound".into(),
            ));
        }
        validate_hex_digest("model_sha256", &self.model_sha256)?;
        validate_hex_digest("arm_profile_sha256", &self.arm_profile_sha256)?;
        if self.kinematic_group_id.is_empty()
            || self.joint_order.is_empty()
            || self.joint_order.len() > message_model::MAX_JOINTS
            || self.actuator_order.len() != self.joint_order.len()
            || has_duplicates(&self.joint_order)
            || has_duplicates(&self.actuator_order)
        {
            return Err(PolicyError::Manifest(
                "profile group orders must be nonempty, bounded and duplicate-free".into(),
            ));
        }
        if self.inputs.is_empty()
            || self.outputs.is_empty()
            || self.inputs.len() > MAX_POLICY_TENSORS
            || self.outputs.len() > MAX_POLICY_TENSORS
        {
            return Err(PolicyError::Manifest(
                "tensor counts are empty or exceed the v1 bound".into(),
            ));
        }
        let input_count = self.inputs[0].element_count()?;
        let action_output = self
            .outputs
            .iter()
            .find(|contract| contract.name == self.action.output)
            .ok_or_else(|| PolicyError::Manifest("action output tensor is absent".into()))?;
        let output_count = action_output.element_count()?;
        if input_count != self.observations.len() {
            return Err(PolicyError::Manifest(
                "observation/action layout differs from tensor element count".into(),
            ));
        }
        validate_tensor_names(&self.inputs, "input")?;
        validate_tensor_names(&self.outputs, "output")?;
        let expected_output = match self.action.kind {
            PolicyActionKind::Mit => self.actuator_order.len() * 3,
            PolicyActionKind::DirectRotaryTorque => self.actuator_order.len(),
            _ => self.joint_order.len(),
        };
        if output_count != expected_output {
            return Err(PolicyError::Manifest(
                "action output count differs from selected semantic".into(),
            ));
        }
        for element in &self.observations {
            if !self.joint_order.contains(&element.joint)
                || element.unit.is_empty()
                || !all_finite(&[
                    element.mean,
                    element.inverse_std,
                    element.minimum,
                    element.maximum,
                ])
                || element.inverse_std <= 0.0
                || element.minimum > element.maximum
            {
                return Err(PolicyError::Manifest(
                    "observation normalization or range is invalid".into(),
                ));
            }
        }
        let mut observation_sources = BTreeSet::new();
        if self
            .observations
            .iter()
            .any(|element| !observation_sources.insert((&element.joint, element.field)))
        {
            return Err(PolicyError::Manifest(
                "observation scalar sources must be unique".into(),
            ));
        }
        validate_action(&self.action, output_count, self.actuator_order.len())?;
        validate_recurrent(self)?;
        if self.inference_period_ns == 0
            || self.inference_deadline_ns == 0
            || self.inference_deadline_ns > self.inference_period_ns
            || !(1..=MAX_POLICY_QUEUE_CAPACITY).contains(&self.queue_capacity)
            || !(1..=MAX_POLICY_THREADS).contains(&self.thread_count)
        {
            return Err(PolicyError::Manifest(
                "period, deadline, queue capacity or thread count is invalid".into(),
            ));
        }
        Ok(())
    }
}

fn validate_tensor_names(tensors: &[TensorContract], field: &str) -> Result<(), PolicyError> {
    let mut names = BTreeSet::new();
    for tensor in tensors {
        tensor.element_count()?;
        if !names.insert(&tensor.name) {
            return Err(PolicyError::Manifest(format!(
                "{field} tensor names must be unique"
            )));
        }
    }
    Ok(())
}

fn validate_recurrent(manifest: &PolicyManifest) -> Result<(), PolicyError> {
    let mut input_names = BTreeSet::new();
    let mut output_names = BTreeSet::new();
    let mut total = 0usize;
    for pair in &manifest.recurrent {
        let input = manifest
            .inputs
            .iter()
            .find(|tensor| tensor.name == pair.input)
            .ok_or_else(|| PolicyError::Manifest("recurrent input tensor is absent".into()))?;
        let output = manifest
            .outputs
            .iter()
            .find(|tensor| tensor.name == pair.output)
            .ok_or_else(|| PolicyError::Manifest("recurrent output tensor is absent".into()))?;
        let count = input.element_count()?;
        if pair.input == manifest.inputs[0].name
            || pair.output == manifest.action.output
            || !input_names.insert(&pair.input)
            || !output_names.insert(&pair.output)
            || input.shape != output.shape
            || pair.reset.len() != count
            || pair.reset.iter().any(|value| !value.is_finite())
        {
            return Err(PolicyError::Manifest(
                "recurrent pair names, shapes or reset values are invalid".into(),
            ));
        }
        total = total
            .checked_add(count)
            .ok_or_else(|| PolicyError::Manifest("recurrent state size overflow".into()))?;
    }
    if total > MAX_POLICY_RECURRENT_ELEMENTS
        || manifest.inputs.len() != 1 + manifest.recurrent.len()
        || manifest.outputs.len() != 1 + manifest.recurrent.len()
        || manifest
            .inputs
            .iter()
            .skip(1)
            .any(|tensor| !input_names.contains(&tensor.name))
        || manifest
            .outputs
            .iter()
            .filter(|tensor| tensor.name != manifest.action.output)
            .any(|tensor| !output_names.contains(&tensor.name))
    {
        return Err(PolicyError::Manifest(
            "recurrent pairs do not exactly cover auxiliary tensors".into(),
        ));
    }
    Ok(())
}

fn validate_action(
    action: &ActionManifest,
    count: usize,
    actuator_count: usize,
) -> Result<(), PolicyError> {
    let vectors = [
        &action.scale,
        &action.offset,
        &action.hard_minimum,
        &action.hard_maximum,
        &action.command_minimum,
        &action.command_maximum,
        &action.slew_per_second,
    ];
    if vectors
        .iter()
        .any(|values| values.len() != count || !all_finite(values))
    {
        return Err(PolicyError::Manifest(
            "action vectors must be finite and match output size".into(),
        ));
    }
    for index in 0..count {
        if action.scale[index] == 0.0
            || action.hard_minimum[index] > action.command_minimum[index]
            || action.command_minimum[index] > action.command_maximum[index]
            || action.command_maximum[index] > action.hard_maximum[index]
            || action.slew_per_second[index] <= 0.0
        {
            return Err(PolicyError::Manifest(
                "action scale, hard, projected or slew bounds are invalid".into(),
            ));
        }
    }
    match action.kind {
        PolicyActionKind::Mit => {
            if action.fixed_kp.len() != actuator_count
                || action.fixed_kd.len() != actuator_count
                || !all_finite(&action.fixed_kp)
                || !all_finite(&action.fixed_kd)
                || action
                    .fixed_kp
                    .iter()
                    .any(|gain| !(0.0..=500.0).contains(gain))
                || action
                    .fixed_kd
                    .iter()
                    .any(|gain| !(0.0..=5.0).contains(gain))
            {
                return Err(PolicyError::Manifest(
                    "MIT fixed gains are incomplete or outside verified bounds".into(),
                ));
            }
        }
        _ if !action.fixed_kp.is_empty() || !action.fixed_kd.is_empty() => {
            return Err(PolicyError::Manifest(
                "fixed MIT gains are forbidden for non-MIT actions".into(),
            ));
        }
        _ => {}
    }
    Ok(())
}

fn validate_hex_digest(field: &str, value: &str) -> Result<(), PolicyError> {
    if value.len() != 64
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_hexdigit() && !byte.is_ascii_uppercase())
        || value.bytes().all(|byte| byte == b'0')
    {
        return Err(PolicyError::Manifest(format!(
            "{field} must be a nonzero lowercase SHA-256 digest"
        )));
    }
    Ok(())
}

fn has_duplicates(values: &[String]) -> bool {
    let mut seen = BTreeSet::new();
    values
        .iter()
        .any(|value| value.is_empty() || !seen.insert(value))
}

fn all_finite(values: &[f64]) -> bool {
    values.iter().all(|value| value.is_finite())
}

#[cfg(test)]
mod tests {
    use super::PolicyManifest;

    const GOOD: &str = include_str!("../tests/fixtures/mit_identity/manifest.toml");

    #[test]
    fn checked_manifest_is_strict_and_valid() {
        PolicyManifest::parse(GOOD.as_bytes()).unwrap();
    }

    #[test]
    fn rejects_unknown_versions_fields_timing_shapes_and_dynamic_gains() {
        let mutations = [
            ("schema_version = 1", "schema_version = 2", "schema version"),
            (
                "runtime_version = \"0.24.0\"",
                "runtime_version = \"0.25.0\"",
                "runtime version",
            ),
            (
                "model_format = \"onnx\"",
                "model_format = \"rten\"",
                "model format",
            ),
            (
                "inference_deadline_ns = 90000000",
                "inference_deadline_ns = 100000001",
                "deadline",
            ),
            ("queue_capacity = 2", "queue_capacity = 0", "queue"),
            ("thread_count = 1", "thread_count = 0", "threads"),
            ("fixed_kp = [20.0, 20.0]", "fixed_kp = [501.0, 20.0]", "kp"),
            ("shape = [1, 6]", "shape = [0, 6]", "zero dimension"),
        ];
        for (from, to, field) in mutations {
            let source = GOOD.replacen(from, to, 1);
            assert!(
                PolicyManifest::parse(source.as_bytes()).is_err(),
                "{field} mutation was accepted"
            );
        }
        let unknown = format!("{GOOD}\nunknown_field = true\n");
        assert!(PolicyManifest::parse(unknown.as_bytes()).is_err());
    }
}
