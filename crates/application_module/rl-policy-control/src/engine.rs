//! Synchronous RTen engine owned exclusively by the non-realtime worker.

use std::sync::Arc;

use arrayvec::ArrayVec;
use rten::{RunOptions, ThreadPool, Value};
use rten_tensor::prelude::*;

use crate::{
    EncodedPolicyObservation, MAX_POLICY_ELEMENTS, MAX_POLICY_RECURRENT_ELEMENTS, PolicyError,
    PolicyGeneration, PolicyIdentity, ReadyPolicy,
};

/// Raw finite output bound to one request and model generation.
pub(crate) struct EngineOutput {
    pub values: ArrayVec<f32, MAX_POLICY_ELEMENTS>,
    pub recurrent_state: ArrayVec<f32, MAX_POLICY_RECURRENT_ELEMENTS>,
}

struct RecurrentSlot {
    input_id: rten::NodeId,
    output_id: rten::NodeId,
    shape: Vec<usize>,
    offset: usize,
    len: usize,
}

/// Single-owner synchronous model executor.
pub struct PolicyEngine {
    model: rten::Model,
    input_id: rten::NodeId,
    output_id: rten::NodeId,
    input_shape: Vec<usize>,
    output_shape: Vec<usize>,
    identity: PolicyIdentity,
    generation: PolicyGeneration,
    run_options: RunOptions,
    recurrent: Vec<RecurrentSlot>,
}

impl PolicyEngine {
    /// Construct an engine with an explicit caller-sized RTen pool. The global
    /// RTen pool is never used.
    pub fn new(policy: ReadyPolicy) -> Result<Self, PolicyError> {
        let (manifest, identity, generation, graph, _) = policy.into_parts();
        let warm_input = vec![0.0; manifest.inputs[0].element_count()?];
        let warm_recurrent = manifest
            .recurrent
            .iter()
            .flat_map(|pair| pair.reset.iter().copied())
            .collect::<Vec<_>>();
        let pool = Arc::new(ThreadPool::with_num_threads(manifest.thread_count));
        let action_output_index = manifest
            .outputs
            .iter()
            .position(|tensor| tensor.name == manifest.action.output)
            .ok_or_else(|| {
                PolicyError::Graph("action output disappeared after validation".into())
            })?;
        let mut recurrent = Vec::with_capacity(manifest.recurrent.len());
        let mut offset = 0usize;
        for pair in &manifest.recurrent {
            let input_index = manifest
                .inputs
                .iter()
                .position(|tensor| tensor.name == pair.input)
                .ok_or_else(|| PolicyError::Graph("recurrent input disappeared".into()))?;
            let output_index = manifest
                .outputs
                .iter()
                .position(|tensor| tensor.name == pair.output)
                .ok_or_else(|| PolicyError::Graph("recurrent output disappeared".into()))?;
            let len = pair.reset.len();
            recurrent.push(RecurrentSlot {
                input_id: graph.input_ids[input_index],
                output_id: graph.output_ids[output_index],
                shape: manifest.inputs[input_index].shape.clone(),
                offset,
                len,
            });
            offset += len;
        }
        let engine = Self {
            model: graph.model,
            input_id: graph.input_ids[0],
            output_id: graph.output_ids[action_output_index],
            input_shape: manifest.inputs[0].shape.clone(),
            output_shape: manifest.outputs[action_output_index].shape.clone(),
            identity,
            generation,
            run_options: RunOptions::default().with_thread_pool(Some(pool)),
            recurrent,
        };
        // Startup-only synthetic tensors allocate and qualify RTen execution
        // before readiness. The discarded result can never become a proposal.
        engine.run_values(&warm_input, &warm_recurrent)?;
        Ok(engine)
    }

    pub(crate) fn infer(
        &self,
        observation: &EncodedPolicyObservation,
        recurrent_state: &[f32],
    ) -> Result<EngineOutput, PolicyError> {
        if observation.identity() != self.identity || observation.generation() != self.generation {
            return Err(PolicyError::StaleResult);
        }
        self.run_values(observation.values(), recurrent_state)
    }

    fn run_values(
        &self,
        observation: &[f32],
        recurrent_state: &[f32],
    ) -> Result<EngineOutput, PolicyError> {
        let input = Value::from_shape(&self.input_shape, observation.to_vec())
            .map_err(|error| PolicyError::Inference(error.to_string()))?;
        let expected_recurrent = self.recurrent.iter().map(|slot| slot.len).sum::<usize>();
        if recurrent_state.len() != expected_recurrent {
            return Err(PolicyError::StaleResult);
        }
        let mut inputs = Vec::with_capacity(1 + self.recurrent.len());
        inputs.push((self.input_id, input.into()));
        for slot in &self.recurrent {
            let end = slot.offset + slot.len;
            let input = Value::from_shape(&slot.shape, recurrent_state[slot.offset..end].to_vec())
                .map_err(|error| PolicyError::Inference(error.to_string()))?;
            inputs.push((slot.input_id, input.into()));
        }
        let mut output_ids = Vec::with_capacity(1 + self.recurrent.len());
        output_ids.push(self.output_id);
        output_ids.extend(self.recurrent.iter().map(|slot| slot.output_id));
        let mut outputs = self
            .model
            .run(inputs, &output_ids, Some(self.run_options.clone()))
            .map_err(|error| PolicyError::Inference(error.to_string()))?;
        let action_output = outputs.remove(0);
        let values = checked_output::<MAX_POLICY_ELEMENTS>(action_output, &self.output_shape)?;
        let mut next_recurrent = ArrayVec::new();
        for (output, slot) in outputs.into_iter().zip(&self.recurrent) {
            let values = checked_output::<MAX_POLICY_RECURRENT_ELEMENTS>(output, &slot.shape)?;
            next_recurrent
                .try_extend_from_slice(&values)
                .map_err(|_| PolicyError::Inference("recurrent state capacity exceeded".into()))?;
        }
        Ok(EngineOutput {
            values,
            recurrent_state: next_recurrent,
        })
    }
}

fn checked_output<const CAPACITY: usize>(
    output: Value,
    expected_shape: &[usize],
) -> Result<ArrayVec<f32, CAPACITY>, PolicyError> {
    let tensor = output
        .into_tensor::<f32>()
        .ok_or_else(|| PolicyError::Inference("output tensor is not f32".into()))?;
    if tensor.shape() != expected_shape {
        return Err(PolicyError::Inference(
            "runtime output shape differs from manifest".into(),
        ));
    }
    let mut values = ArrayVec::new();
    for value in tensor.into_data() {
        if !value.is_finite() {
            return Err(PolicyError::Inference(
                "runtime output contains NaN or infinity".into(),
            ));
        }
        values
            .try_push(value)
            .map_err(|_| PolicyError::Inference("output capacity exceeded".into()))?;
    }
    Ok(values)
}
