//! Deterministically generate the tiny ONNX fixture used by host tests.
//!
//! This development-only generator writes a single Identity graph with one
//! static `[1, 6]` f32 input and output. It intentionally has no production
//! dependency on Python, ONNX Runtime or an online trainer.

use std::fs;
use std::path::PathBuf;

use arm_model::{
    ARM_PROFILE_SCHEMA_VERSION, ArmModel, ArmProfileClass, ArmProfileSpec,
    JointAccelerationLimitSpec, KinematicGroupSpec, RotaryTransmissionSpec,
};
use message_model::{ArmActuatorId, ArmJointId, AsciiId, KinematicGroupId};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    for (name, model) in [
        ("joint_affine", affine_model_proto(false)),
        ("mit_identity", model_proto(6)),
    ] {
        let output = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests")
            .join("fixtures")
            .join(name)
            .join("model.onnx");
        if let Some(parent) = output.parent() {
            fs::create_dir_all(parent)?;
        }
        fs::write(&output, model)?;
        println!("{}", output.display());
    }
    let recurrent_output = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("recurrent")
        .join("model.onnx");
    fs::create_dir_all(recurrent_output.parent().unwrap())?;
    fs::write(&recurrent_output, recurrent_model_proto())?;
    println!("{}", recurrent_output.display());
    let external_output = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("tests")
        .join("fixtures")
        .join("external_data")
        .join("model.onnx");
    fs::create_dir_all(external_output.parent().unwrap())?;
    fs::write(&external_output, affine_model_proto(true))?;
    println!("{}", external_output.display());
    println!("arm_profile_sha256={}", fixture_profile_sha256()?);
    Ok(())
}

fn affine_model_proto(external_scale: bool) -> Vec<u8> {
    let multiply = message(&[
        string_field(1, "observation"),
        string_field(1, "scale"),
        string_field(2, "scaled"),
        string_field(4, "Mul"),
    ]);
    let add = message(&[
        string_field(1, "scaled"),
        string_field(1, "bias"),
        string_field(2, "action"),
        string_field(4, "Add"),
    ]);
    let input = value_info("observation", &[1, 2]);
    let output = value_info("action", &[1, 2]);
    let scale = if external_scale {
        external_float_tensor("scale", &[1, 2])
    } else {
        float_tensor("scale", &[1, 2], &[2.0, 2.0])
    };
    let bias = float_tensor("bias", &[1, 2], &[0.5, -0.5]);
    let graph = message(&[
        bytes_field(1, &multiply),
        bytes_field(1, &add),
        bytes_field(5, &scale),
        bytes_field(5, &bias),
        bytes_field(11, &input),
        bytes_field(12, &output),
    ]);
    message(&[varint_field(1, 8), bytes_field(7, &graph)])
}

fn external_float_tensor(name: &str, shape: &[u64]) -> Vec<u8> {
    let mut fields = shape
        .iter()
        .map(|&dimension| varint_field(1, dimension))
        .collect::<Vec<_>>();
    fields.push(varint_field(2, 1));
    let location = message(&[string_field(1, "location"), string_field(2, "weights.bin")]);
    fields.push(bytes_field(13, &location));
    fields.push(varint_field(14, 1));
    fields.push(string_field(8, name));
    message(&fields)
}

fn float_tensor(name: &str, shape: &[u64], values: &[f32]) -> Vec<u8> {
    let mut fields = shape
        .iter()
        .map(|&dimension| varint_field(1, dimension))
        .collect::<Vec<_>>();
    fields.push(varint_field(2, 1));
    fields.extend(values.iter().map(|value| fixed32_field(4, value.to_bits())));
    fields.push(string_field(8, name));
    message(&fields)
}

fn recurrent_model_proto() -> Vec<u8> {
    let add = message(&[
        string_field(1, "observation"),
        string_field(1, "state_in"),
        string_field(2, "sum"),
        string_field(4, "Add"),
    ]);
    let action = message(&[
        string_field(1, "sum"),
        string_field(2, "action"),
        string_field(4, "Identity"),
    ]);
    let state = message(&[
        string_field(1, "sum"),
        string_field(2, "state_out"),
        string_field(4, "Identity"),
    ]);
    let observation = value_info("observation", &[1, 2]);
    let state_in = value_info("state_in", &[1, 2]);
    let action_output = value_info("action", &[1, 2]);
    let state_out = value_info("state_out", &[1, 2]);
    let graph = message(&[
        bytes_field(1, &add),
        bytes_field(1, &action),
        bytes_field(1, &state),
        bytes_field(11, &observation),
        bytes_field(11, &state_in),
        bytes_field(12, &action_output),
        bytes_field(12, &state_out),
    ]);
    message(&[varint_field(1, 8), bytes_field(7, &graph)])
}

fn fixture_profile_sha256() -> Result<String, Box<dyn std::error::Error>> {
    let joint_1 = ArmJointId::parse("joint_1")?;
    let joint_2 = ArmJointId::parse("joint_2")?;
    let model = ArmModel::compile_urdf(
        FIXTURE_URDF,
        &ArmProfileSpec {
            schema_version: ARM_PROFILE_SCHEMA_VERSION,
            name: AsciiId::parse("rl_fixture")?,
            class: ArmProfileClass::HostFixture,
            groups: vec![KinematicGroupSpec {
                id: KinematicGroupId::parse("arm")?,
                base_link: AsciiId::parse("base")?,
                tip_link: AsciiId::parse("tool")?,
                active_joint_order: vec![joint_1, joint_2],
            }],
            acceleration_limits: vec![
                JointAccelerationLimitSpec {
                    joint_id: joint_1,
                    maximum: 8.0,
                },
                JointAccelerationLimitSpec {
                    joint_id: joint_2,
                    maximum: 8.0,
                },
            ],
            transmissions: vec![
                RotaryTransmissionSpec {
                    joint_id: joint_1,
                    actuator_id: ArmActuatorId::parse("act_1")?,
                    scale_rad_per_joint_unit: 1.0,
                    offset_rad: 0.0,
                },
                RotaryTransmissionSpec {
                    joint_id: joint_2,
                    actuator_id: ArmActuatorId::parse("act_2")?,
                    scale_rad_per_joint_unit: 1.0,
                    offset_rad: 0.0,
                },
            ],
        },
    )?;
    Ok(model
        .profile_id()
        .as_bytes()
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect())
}

fn model_proto(width: u64) -> Vec<u8> {
    let node = message(&[
        string_field(1, "observation"),
        string_field(2, "action"),
        string_field(4, "Identity"),
    ]);
    let input = value_info("observation", &[1, width]);
    let output = value_info("action", &[1, width]);
    let graph = message(&[
        bytes_field(1, &node),
        bytes_field(11, &input),
        bytes_field(12, &output),
    ]);
    message(&[varint_field(1, 8), bytes_field(7, &graph)])
}

fn value_info(name: &str, shape: &[u64]) -> Vec<u8> {
    let dimensions = shape
        .iter()
        .map(|&value| bytes_field(1, &message(&[varint_field(1, value)])))
        .collect::<Vec<_>>();
    let tensor_shape = message(&dimensions);
    let tensor_type = message(&[varint_field(1, 1), bytes_field(2, &tensor_shape)]);
    let value_type = message(&[bytes_field(1, &tensor_type)]);
    message(&[string_field(1, name), bytes_field(2, &value_type)])
}

fn message(fields: &[Vec<u8>]) -> Vec<u8> {
    fields.iter().flatten().copied().collect()
}

fn string_field(number: u64, value: &str) -> Vec<u8> {
    bytes_field(number, value.as_bytes())
}

fn bytes_field(number: u64, value: &[u8]) -> Vec<u8> {
    let mut output = varint((number << 3) | 2);
    output.extend(varint(value.len() as u64));
    output.extend(value);
    output
}

fn varint_field(number: u64, value: u64) -> Vec<u8> {
    let mut output = varint(number << 3);
    output.extend(varint(value));
    output
}

fn fixed32_field(number: u64, value: u32) -> Vec<u8> {
    let mut output = varint((number << 3) | 5);
    output.extend(value.to_le_bytes());
    output
}

fn varint(mut value: u64) -> Vec<u8> {
    let mut output = Vec::new();
    loop {
        let mut byte = (value & 0x7f) as u8;
        value >>= 7;
        if value != 0 {
            byte |= 0x80;
        }
        output.push(byte);
        if value == 0 {
            return output;
        }
    }
}

const FIXTURE_URDF: &str = r#"
<robot name="rl_fixture">
  <link name="base"/>
  <link name="link_1"/>
  <link name="tool"/>
  <joint name="joint_1" type="revolute">
    <parent link="base"/>
    <child link="link_1"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-2" upper="2" effort="20" velocity="4"/>
  </joint>
  <joint name="joint_2" type="revolute">
    <parent link="link_1"/>
    <child link="tool"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" effort="20" velocity="4"/>
  </joint>
</robot>
"#;
