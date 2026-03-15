//! ONNX structure inspection and exact RTen graph validation.

use std::collections::BTreeSet;

use rten::{DataType, Dimension, Model, NodeId, ValueType};
use rten_onnx::onnx::{
    AttributeProto, DataLocation, DataType as OnnxDataType, GraphProto, ModelProto, TensorProto,
    ValueInfoProto,
};

use crate::{PolicyError, PolicyManifest, TensorContract};

/// RTen model plus pre-resolved single input/output IDs.
pub(crate) struct ValidatedPolicyGraph {
    pub model: Model,
    pub input_ids: Vec<NodeId>,
    pub output_ids: Vec<NodeId>,
}

/// Validate the ONNX protobuf before RTen readiness, then cross-check RTen's
/// optimized graph metadata.
pub(crate) fn validate_policy_graph(
    bytes: &[u8],
    manifest: &PolicyManifest,
) -> Result<ValidatedPolicyGraph, PolicyError> {
    let onnx =
        ModelProto::parse_buf(bytes).map_err(|error| PolicyError::Graph(error.to_string()))?;
    let graph = onnx
        .graph
        .as_ref()
        .ok_or_else(|| PolicyError::Graph("ONNX model has no graph".into()))?;
    validate_onnx_graph(graph, &manifest.inputs, &manifest.outputs)?;

    let model = Model::load(bytes.to_vec())
        .map_err(|error| PolicyError::Graph(format!("RTen load failed: {error}")))?;
    if model.input_ids().len() != manifest.inputs.len()
        || model.output_ids().len() != manifest.outputs.len()
    {
        return Err(PolicyError::Graph(
            "RTen input/output count differs from manifest".into(),
        ));
    }
    for (&id, contract) in model.input_ids().iter().zip(&manifest.inputs) {
        validate_rten_node(&model, id, contract)?;
    }
    for (&id, contract) in model.output_ids().iter().zip(&manifest.outputs) {
        validate_rten_node(&model, id, contract)?;
    }
    Ok(ValidatedPolicyGraph {
        input_ids: model.input_ids().to_vec(),
        output_ids: model.output_ids().to_vec(),
        model,
    })
}

fn validate_onnx_graph(
    graph: &GraphProto,
    inputs: &[TensorContract],
    outputs: &[TensorContract],
) -> Result<(), PolicyError> {
    validate_value_infos("input", &graph.input, inputs)?;
    validate_value_infos("output", &graph.output, outputs)?;
    validate_constants(graph)?;
    Ok(())
}

fn validate_value_infos(
    field: &str,
    values: &[ValueInfoProto],
    expected: &[TensorContract],
) -> Result<(), PolicyError> {
    if values.len() != expected.len() {
        return Err(PolicyError::Graph(format!(
            "ONNX {field} count differs from manifest"
        )));
    }
    let mut names = BTreeSet::new();
    for (value, contract) in values.iter().zip(expected) {
        let name = value
            .name
            .as_deref()
            .ok_or_else(|| PolicyError::Graph(format!("ONNX {field} has no name")))?;
        if !names.insert(name) || name != contract.name {
            return Err(PolicyError::Graph(format!(
                "ONNX {field} name is duplicate or differs from manifest"
            )));
        }
        let tensor = value
            .r#type
            .as_ref()
            .and_then(|value_type| value_type.tensor_type.as_ref())
            .ok_or_else(|| PolicyError::Graph(format!("ONNX {field} is not a tensor")))?;
        if tensor.elem_type != Some(OnnxDataType::FLOAT) {
            return Err(PolicyError::Graph(format!("ONNX {field} is not f32")));
        }
        let dimensions = tensor
            .shape
            .as_ref()
            .ok_or_else(|| PolicyError::Graph(format!("ONNX {field} shape is absent")))?
            .dim
            .iter()
            .map(
                |dimension| match (dimension.dim_value, dimension.dim_param.as_deref()) {
                    (Some(value), None) if value > 0 => usize::try_from(value).ok(),
                    _ => None,
                },
            )
            .collect::<Option<Vec<_>>>()
            .ok_or_else(|| PolicyError::Graph(format!("ONNX {field} shape is dynamic")))?;
        if dimensions != contract.shape {
            return Err(PolicyError::Graph(format!(
                "ONNX {field} shape differs from manifest"
            )));
        }
    }
    Ok(())
}

fn validate_constants(graph: &GraphProto) -> Result<(), PolicyError> {
    for tensor in &graph.initializer {
        validate_tensor(tensor)?;
    }
    for node in &graph.node {
        for attribute in &node.attribute {
            validate_attribute(attribute)?;
        }
    }
    Ok(())
}

fn validate_attribute(attribute: &AttributeProto) -> Result<(), PolicyError> {
    if attribute.f.is_some_and(|value| !value.is_finite())
        || attribute.floats.iter().any(|value| !value.is_finite())
    {
        return Err(PolicyError::Graph(
            "ONNX attribute contains a non-finite constant".into(),
        ));
    }
    if let Some(tensor) = &attribute.t {
        validate_tensor(tensor)?;
    }
    if let Some(graph) = &attribute.g {
        validate_constants(graph)?;
    }
    Ok(())
}

fn validate_tensor(tensor: &TensorProto) -> Result<(), PolicyError> {
    if !tensor.external_data.is_empty() || tensor.data_location == Some(DataLocation::EXTERNAL) {
        return Err(PolicyError::Graph(
            "ONNX external tensor data is forbidden".into(),
        ));
    }
    if tensor.float_data.iter().any(|value| !value.is_finite())
        || tensor.double_data.iter().any(|value| !value.is_finite())
    {
        return Err(PolicyError::Graph(
            "ONNX initializer contains a non-finite value".into(),
        ));
    }
    if let Some(raw) = &tensor.raw_data {
        let raw = raw.borrow();
        match tensor.data_type {
            Some(OnnxDataType::FLOAT)
                if raw.len() % 4 != 0
                    || raw
                        .chunks_exact(4)
                        .map(|bytes| f32::from_le_bytes(bytes.try_into().expect("chunk size")))
                        .any(|value| !value.is_finite()) =>
            {
                return Err(PolicyError::Graph(
                    "ONNX f32 initializer bytes are malformed or non-finite".into(),
                ));
            }
            Some(OnnxDataType::DOUBLE)
                if raw.len() % 8 != 0
                    || raw
                        .chunks_exact(8)
                        .map(|bytes| f64::from_le_bytes(bytes.try_into().expect("chunk size")))
                        .any(|value| !value.is_finite()) =>
            {
                return Err(PolicyError::Graph(
                    "ONNX f64 initializer bytes are malformed or non-finite".into(),
                ));
            }
            _ => {}
        }
    }
    Ok(())
}

fn validate_rten_node(
    model: &Model,
    id: NodeId,
    contract: &TensorContract,
) -> Result<(), PolicyError> {
    let info = model
        .node_info(id)
        .ok_or_else(|| PolicyError::Graph("RTen node metadata is absent".into()))?;
    if info.name() != Some(contract.name.as_str())
        || info.dtype() != Some(ValueType::Tensor(DataType::Float))
    {
        return Err(PolicyError::Graph(
            "RTen node name or dtype differs from manifest".into(),
        ));
    }
    let shape = info
        .shape()
        .ok_or_else(|| PolicyError::Graph("RTen node shape is absent".into()))?
        .into_iter()
        .map(|dimension| match dimension {
            Dimension::Fixed(value) if value > 0 => Some(value),
            Dimension::Fixed(_) | Dimension::Symbolic(_) => None,
        })
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| PolicyError::Graph("RTen node shape is dynamic".into()))?;
    if shape != contract.shape {
        return Err(PolicyError::Graph(
            "RTen node shape differs from manifest".into(),
        ));
    }
    Ok(())
}
