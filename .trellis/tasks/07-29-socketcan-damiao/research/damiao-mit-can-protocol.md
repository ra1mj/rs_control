# Research: DM-J4310-2EC MIT CAN protocol and safe host-test boundary

- Query: Determine the DM-J4310-2EC classic-CAN/MIT wire contract, lifecycle and register operations from manufacturer-controlled sources, distinguish community assumptions, and define what can be tested without motor hardware.
- Scope: mixed (repository-local architecture plus external manufacturer and community sources)
- Date: 2026-07-29

## Findings

### Confidence labels

- **Verified — manufacturer manual**: stated by the pinned DM-J4310-2EC V1.2 manual published through the current DAMIAO public-materials repository.
- **Verified — manufacturer SDK**: encoded by the pinned SDK that the current DAMIAO public-materials repository selects as a submodule. This is useful for byte-level details but does not override the product manual.
- **Corroborated — community**: agrees with the manufacturer sources but is not authoritative for firmware behavior.
- **Unverified**: absent, ambiguous, or contradictory in the pinned manufacturer sources; requires a firmware identity and target capture.

### Files found in this repository

| Path | Relevance |
|---|---|
| `crates/hardware_module/hardware-module-core/src/lib.rs` | Current path of the synchronous, caller-buffered transport and device lifecycle traits owned by `hardware_module`. |
| `crates/integration/config-model/src/lib.rs` | Generic robot-YAML motor-ID validation; currently accepts standard CAN IDs `1..=0x7ff`. |
| `platform/orangepi5plus/config/can.env` | Fixes the first platform to classic CAN at 1,000,000 bit/s, error reporting on, and automatic restart off. |
| `platform/orangepi5plus/scripts/can-link.sh` | Applies and reads back the platform CAN-link contract without transmitting motor frames. |
| `.trellis/spec/backend/robot-contracts.md` | Requires motor wire protocols and lifecycle to remain in `hardware_module` and keeps protocol details out of robot YAML. |
| `.trellis/spec/backend/error-handling.md` | Requires typed device faults and forbids turning missing/stale feedback into numeric zero. |
| `.trellis/spec/backend/realtime-platform.md` | Separates safe host tests from target qualification and forbids platform tooling from operating a motor. |

The task PRD is still a placeholder and `design.md` / `implement.md` do not yet exist as of this research pass.

### Source provenance and license status

#### Manufacturer-controlled publication chain

1. DAMIAO public-materials entry point

   - Permanent tree: <https://gitee.com/kit-miao/damiao/tree/0aceca1e0e8afd704917a80ced840c3ddf93020f>
   - Commit: `0aceca1e0e8afd704917a80ced840c3ddf93020f`
   - `README.md` SHA-256: `99a3d37cd58d2acfbf7f2aa029023da0f41b258a095f62258d75093330cc29db`
   - Evidence of control: the repository describes itself as the DAMIAO external-materials entry point and pins the product and SDK repositories as submodules.
   - License: no repository-level license declared. Treat material outside an explicitly licensed subdirectory as all-rights-reserved reference material.

2. DM-J4310-2EC product materials

   - Permanent tree: <https://gitee.com/kit-miao/DM-J4310-2EC/tree/9db7863cba1c386ad9b8dedad3286465011ca6b2>
   - Commit: `9db7863cba1c386ad9b8dedad3286465011ca6b2` (the public-materials entry point pins this exact submodule commit)
   - Repository `README.md` SHA-256: `62c10ed86fe5d7b10cd4860889116db7a946aaf4ece1018d0d753135e2a5407d`
   - Primary manual: `说明书/DM-J4310-2EC V1.2减速电机说明书V1.2定稿.pdf`
   - Permanent manual URL: <https://gitee.com/kit-miao/DM-J4310-2EC/blob/9db7863cba1c386ad9b8dedad3286465011ca6b2/%E8%AF%B4%E6%98%8E%E4%B9%A6/DM-J4310-2EC%20V1.2%E5%87%8F%E9%80%9F%E7%94%B5%E6%9C%BA%E8%AF%B4%E6%98%8E%E4%B9%A6V1.2%E5%AE%9A%E7%A8%BF.pdf>
   - Git blob: `3800012c1555ca3fd6fa233e25ab1d09f17a81e7`
   - PDF SHA-256: `89da6e8f30b384df2ac1b2d4ae741b611828b2d97d2656e6fb026b31c66553ee`
   - Manual identity: document V1.2, dated 2025-11-21, 45 pages.
   - License: the manual says it is copyrighted by Shenzhen DAMIAO Technology and forbids reproduction without permission. Facts may be independently implemented; do not copy its prose, figures, or tables into source code or redistribute the PDF.

3. DAMIAO motor SDK selected by the public-materials repository

   - Permanent tree: <https://gitee.com/kit-miao/motor-sdk/tree/ae360bde6a5af373e5a68440f80708bea321689d>
   - Commit: `ae360bde6a5af373e5a68440f80708bea321689d`
   - Python example: `Python例程/u2can/DM_CAN.py`, Git blob `e948af2b50344ad1ea5cbc261442b3f2eb7ededb`, SHA-256 `6e9f4b6fb639ff2d5ebeb426d61b0af70c9146d47a05c0f57d034b94cc522e1d`.
   - Python example license: MIT in the same directory, Git blob `9afae278e2459d2f844d1abafd948524a0963210`, SHA-256 `b12746718fc87bc01d5fc9d9e612eea30584fdc78d4eb54021b3f1d82b620fd7`, copyright 2024 cmjang.
   - C++ USB2CANFD header: `C++例程/u2canfd/include/protocol/damiao.h`, SHA-256 `9506bc5f6ff24416b47168a80bf616e893cc600103107f3a12e496bd90d1d911`.
   - C++ USB2CANFD implementation: `C++例程/u2canfd/src/protocol/damiao.cpp`, SHA-256 `4959b466dbd8921ccebd550f82cf83315b08c3f2ca1bd6db49970e948f755e5f`.
   - License caveat: there is no root license and no license next to the C++ USB2CANFD files. Use those files to corroborate protocol facts only; do not copy their code. Only the specifically licensed Python/Matlab example subtrees can be treated as MIT.

4. Manufacturer website product page (identity cross-check, not a reproducible source pin)

   - <https://www.damiaokeji.com/index.php?c=show&id=125>
   - Confirms the current product name `DM-J4310-2EC V1.2` and CAN feedback of speed, position, torque, and temperature.
   - The page is mutable and has no source commit, so it is secondary to the pinned manual above.

#### Community implementations (not firmware authorities)

1. `cmjang/DM_Control_Python`

   - Permanent tree: <https://github.com/cmjang/DM_Control_Python/tree/7da93877ba844d9587149d6f3a6385453aa8379f>
   - Commit: `7da93877ba844d9587149d6f3a6385453aa8379f`
   - `DM_CAN.py` SHA-256: `3286b682a38a7df97ccef680680ec1f689b9fec1f948e3b0bdc47a466bf40d8f`
   - License: MIT, LICENSE SHA-256 `b12746718fc87bc01d5fc9d9e612eea30584fdc78d4eb54021b3f1d82b620fd7`.
   - Use: useful corroboration and apparent upstream of some licensed SDK examples, but it wraps a serial USB2CAN protocol and must not be copied as a SocketCAN transport design.

2. `jia-xie/python-damiao-driver`

   - Permanent tree: <https://github.com/jia-xie/python-damiao-driver/tree/3c6773aa303b5eb9d2613b83fc0b8a5274b79b94>
   - Commit: `3c6773aa303b5eb9d2613b83fc0b8a5274b79b94`
   - Protocol document SHA-256: `162ff6c1452bfbd67861e48bb4ad5dce8ba63441dfaca1eec9e85551983f0538`
   - License: `pyproject.toml` declares MIT, but GitHub reports no detected repository license and no root LICENSE was present in the inspected tree. Treat reuse status as less certain than the explicit cmjang LICENSE.
   - Important contradiction: its protocol page says motors continuously transmit feedback. The pinned V1.2 manufacturer manual says feedback is query/response after a matching command. The implementation must follow the manufacturer statement and preserve a target capture as the final proof.

### Authoritative bus and arbitration contract

For this task, `motor_id` means the configured receive ID (`ESC_ID`), not the feedback ID.

| Purpose | Standard arbitration ID | Source/confidence |
|---|---:|---|
| MIT command | `ESC_ID` | Verified — manufacturer manual, CAN communication/MIT frame sections |
| MIT enable/disable/zero/error-clear | `ESC_ID` | Verified for MIT: lifecycle frames use the active mode's control-frame ID, and MIT has offset zero |
| Feedback and register reply | `MST_ID` | Verified — manufacturer manual |
| Register read/write/store request | `0x7ff` | Verified — manufacturer manual |
| Position-velocity command (out of current task scope) | `0x100 + ESC_ID` | Verified — manufacturer manual |
| Velocity command (out of current task scope) | `0x200 + ESC_ID` | Verified — manufacturer manual |
| Force-position command (out of current task scope) | `0x300 + ESC_ID` | Verified — manufacturer manual |

Additional rules:

- Frames are 11-bit standard (`STD`) CAN data frames, not extended frames and not RTR frames.
- The current platform contract is classic CAN at exactly 1,000,000 bit/s. The V1.2 motor defaults to 1 Mbit/s. Do not exercise the motor's bitrate register in this task.
- The V1.2 manual says feedback is **query-style**: after a received frame matches the configured CAN ID, the drive sends current state. Do not design the driver around unsolicited periodic feedback.
- The manual says the receive match checks the low eight identifier bits and ignores the high three. Those high bits are also used by the non-MIT mode offsets. This task is MIT-only, so use `ESC_ID` with no offset.
- Register ranges allow `ESC_ID` and `MST_ID` in `0..=0x7ff`, but the same manual recommends setting `CAN_ID < 16` to avoid conflict with the error/status representation. The feedback payload can carry only the low nibble of the motor ID. Therefore the safe framework profile is `ESC_ID in 1..=15`, unique on the bus. This should be a Damiao-specific startup/newtype check, not a global narrowing of the generic robot YAML contract.
- A production bus should also require a unique, nonzero `MST_ID` that does not collide with any command ID or `0x7ff`. Robot YAML can stay minimal: list only the `ESC_ID`; read `MST_ID` from register `0x07` during disarmed configuration and refuse arming on collision/mismatch.

### MIT command encoding

The command is exactly eight bytes:

| Byte | Bits |
|---:|---|
| `D[0]` | `position_u[15:8]` |
| `D[1]` | `position_u[7:0]` |
| `D[2]` | `velocity_u[11:4]` |
| `D[3]` | `velocity_u[3:0] << 4 | kp_u[11:8]` |
| `D[4]` | `kp_u[7:0]` |
| `D[5]` | `kd_u[11:4]` |
| `D[6]` | `kd_u[3:0] << 4 | torque_u[11:8]` |
| `D[7]` | `torque_u[7:0]` |

For an `N`-bit field:

```text
u = trunc((x - x_min) / (x_max - x_min) * (2^N - 1))
x = x_min + u / (2^N - 1) * (x_max - x_min)
```

The mathematical map is in the manufacturer manual. Truncation rather than rounding is corroborated by the pinned manufacturer SDK examples. A safe Rust boundary should reject non-finite and out-of-range values with a typed error instead of silently clamping them.

Ranges:

| Field | Bits | Range source |
|---|---:|---|
| position | 16 | `[-PMAX, +PMAX]`, runtime register `0x15` |
| velocity | 12 | `[-VMAX, +VMAX]`, runtime register `0x16` |
| feed-forward torque | 12 | `[-TMAX, +TMAX]`, runtime register `0x17` |
| `kp` | 12 | `[0, 500]` fixed by manual |
| `kd` | 12 | `[0, 5]` fixed by manual |

Do not equate product peak torque with `TMAX`. The pinned SDK's preset for a 24 V `DM4310` is `PMAX=12.5 rad`, `VMAX=30 rad/s`, `TMAX=10 N·m`; the 48 V preset uses `VMAX=50 rad/s`. These are useful golden-fixture values, not a substitute for reading registers `0x15..0x17` and matching the actual motor/firmware before enable.

Golden host vectors using the 24 V SDK preset and truncation:

| Physical input `(p,v,kp,kd,t)` | Encoded data |
|---|---|
| `(0, 0, 0, 0, 0)` | `7F FF 7F F0 00 00 07 FF` |
| `(1, -2, 50, 0.5, 3)` | `8A 3C 77 71 99 19 9A 65` |
| all lower bounds | `00 00 00 00 00 00 00 00` |
| all upper bounds | `FF FF FF FF FF FF FF FF` |

The second vector maps to integer fields `(35388, 1911, 409, 409, 2661)`.

### Feedback decoding, status, and temperature

Feedback is exactly eight bytes on arbitration ID `MST_ID`:

| Byte | Bits/meaning |
|---:|---|
| `D[0]` | `status[3:0] << 4 | (ESC_ID & 0x0f)` |
| `D[1]` | `position_u[15:8]` |
| `D[2]` | `position_u[7:0]` |
| `D[3]` | `velocity_u[11:4]` |
| `D[4]` | `velocity_u[3:0] << 4 | torque_u[11:8]` |
| `D[5]` | `torque_u[7:0]` |
| `D[6]` | MOS average temperature, degrees Celsius |
| `D[7]` | rotor/winding average temperature, degrees Celsius |

Decode position, velocity, and torque with the same runtime `PMAX`, `VMAX`, and `TMAX` used by the drive. Example feedback using the second golden vector, enabled status, motor ID 1, and temperatures 42/43 °C:

```text
11 8A 3C 77 7A 65 2A 2B
```

It decodes to approximately `p=0.999656672 rad`, `v=-2.0 rad/s`, and `t=2.996336996 N·m`.

Status nibble:

| Code | Meaning | Confidence |
|---:|---|---|
| `0x0` | disabled | Verified — feedback table |
| `0x1` | enabled | Verified — feedback table |
| `0x3` | output-shaft calibration fault | Verified in the same manual's fault list; the feedback table omits it, but the manual says faults are observable through feedback |
| `0x4` | sensor-output fault | Same caveat as `0x3` |
| `0x5` | motor-encoder calibration fault | Same caveat as `0x3` |
| `0x8` | over-voltage | Verified — feedback table |
| `0x9` | under-voltage | Verified — feedback table |
| `0xa` | over-current | Verified — feedback table |
| `0xb` | MOS over-temperature | Verified — feedback table |
| `0xc` | winding/rotor over-temperature | Verified — feedback table |
| `0xd` | communication lost/timeout | Verified — feedback table |
| `0xe` | overload | Verified — feedback table |

Treat all other nibble values as `Unknown(code)` and fault the device; never convert an unknown status to disabled or healthy. Preserve the raw status and both raw temperature bytes in diagnostics. The manual states Celsius units but does not define signedness, invalid sentinels, accuracy, or valid temperature range, so a codec may store the on-wire byte as `u8` while target qualification determines any higher-level plausibility policy.

Accept feedback only when all of these hold:

1. standard data frame, DLC 8;
2. arbitration ID equals the configured/discovered `MST_ID`;
3. `D[0] & 0x0f == ESC_ID & 0x0f`;
4. the runtime scaling identity is known and current;
5. the feedback belongs to the outstanding query/command window and is fresh.

### Enable, disable, zero, and fault reset

All four are eight-byte standard data frames. In MIT mode their arbitration ID is `ESC_ID`.

| Operation | Data |
|---|---|
| enable | `FF FF FF FF FF FF FF FC` |
| disable | `FF FF FF FF FF FF FF FD` |
| save current position as zero | `FF FF FF FF FF FF FF FE` |
| clear error | `FF FF FF FF FF FF FF FB` |

`0xFB` is explicitly present in the pinned V1.2 manufacturer manual even though the inspected manufacturer Python/C++ APIs do not expose a dedicated clear-error wrapper.

Safety consequences:

- Sending a frame successfully is not a completed lifecycle transition. Enable completes only after bounded fresh feedback reports status `0x1`; disable completes only after bounded fresh feedback reports `0x0`.
- A fault status transitions the driver to `Faulted`. Clear-error may be sent only through an explicit disarmed maintenance action after the physical cause is removed. It does not imply enable and must not auto-retry.
- The zero command changes the motor's reference and is operationally destructive. It must never run during ordinary startup, CI, discovery, or automatic fault recovery.
- Host tests may verify construction of these bytes and lifecycle decisions with a fake transport, but must never transmit them on a physical interface.
- The manufacturer SDK contains a separate legacy-firmware enable-ID calculation. It is irrelevant to an identified current MIT firmware and must not become silent fallback behavior. Unknown/legacy firmware must keep the device disarmed until its exact protocol is qualified.

### Register access: ESC_ID, MST_ID, mode, timeout, and scaling

All register requests are standard frames on arbitration ID `0x7ff`. The target `ESC_ID` is little-endian in `D[0:2]`; 32-bit register data is little-endian in `D[4:8]`.

| Operation | Request bytes | Reply arbitration/data |
|---|---|---|
| read | `[ESC_L, ESC_H, 0x33, RID, 0, 0, 0, 0]` | `MST_ID`; `[ESC_L, ESC_H, 0x33, RID, data0..data3]` |
| write runtime RAM | `[ESC_L, ESC_H, 0x55, RID, data0..data3]` | `MST_ID`; same opcode/RID/data form |
| store all parameters | `[ESC_L, ESC_H, 0xAA, 0x01, 0, 0, 0, 0]` | `MST_ID`; header begins `[ESC_L, ESC_H, 0xAA, 0x01]` |

The manual prints only four request bytes for read/store, while the manufacturer SDK pads them to DLC 8. Use DLC 8 for deterministic compatibility; a decoder can accept a documented short store acknowledgement only after target capture establishes its DLC.

Required registers:

| RID | Name | Type/range | Required handling |
|---:|---|---|---|
| `0x07` | `MST_ID` | `uint32`, `0..=0x7ff` | Read during disarmed configuration; require framework-safe uniqueness/nonzero before arming. |
| `0x08` | `ESC_ID` | `uint32`, `0..=0x7ff` | Must match YAML motor ID and framework policy `1..=15`; mismatch refuses arming. |
| `0x09` | `TIMEOUT` | `uint32`, `0..=2^32-1` | One tick is 50 µs and enforcement is active only while enabled. Read and record. Zero semantics are not documented, so do not rewrite automatically. |
| `0x0a` | `CTRL_MODE` | `uint32`; documented modes `1..=4` | Require `1` (MIT) for this task before enable. Value `0` appears in the table range but has no documented mode meaning. |
| `0x15` | `PMAX` | float, `(0, fmax]` | Read and validate finite/positive; establishes position codec identity. |
| `0x16` | `VMAX` | float, `(0, fmax]` | Read and validate finite/positive; establishes velocity codec identity. |
| `0x17` | `TMAX` | float, `(0, fmax]` | Read and validate finite/positive; establishes torque codec identity. |

Mode write values are `1=MIT`, `2=position-velocity`, `3=velocity`, and `4=force-position`. A mode write clears position, velocity, torque feed-forward, `kp`, and `kd` commands. It is runtime-only until a store command. For this task, a mode mismatch should be reported and remain disarmed rather than silently writing motor configuration.

Store has unusual safety/performance properties: it works only while disabled, stores all parameters at once, may take up to 30 ms, and flash endurance is approximately 10,000 writes. It is never an RT-path operation and should not be in normal robot startup.

Register transactions should be serialized by the single `hardware_module` CAN-I/O owner. Correlate a reply by expected `MST_ID`, embedded target ID, opcode, RID, and a bounded deadline. Do not parse an arbitrary state frame as a register reply merely because its third byte happens to equal `0x33` or `0x55`. No register read/write/store should run concurrently with armed motion traffic for the same device.

### Architecture and implementation consequences

- Damiao wire codec, SocketCAN adapter, bus-I/O ownership, register discovery, feedback freshness, and device lifecycle all belong to `hardware_module`.
- `protocol_module` receives validated semantic samples/health and exposes node-to-node communication; it does not parse CAN frames.
- `application_module` sends semantic commands through `protocol_module`; it does not open SocketCAN or invoke the Damiao driver directly.
- Keep robot YAML unchanged and minimal. Its motor IDs are the expected Damiao `ESC_ID`s. Discover/verify `MST_ID`, `CTRL_MODE`, `TIMEOUT`, `PMAX`, `VMAX`, and `TMAX` during disarmed driver configuration rather than adding them to YAML.
- The current generic config validator accepts `1..=0x7ff`; retain that vendor-neutral range. Constructing a Damiao-specific ID/config should apply the stricter `1..=15` policy and refuse ambiguous IDs before any transmit.
- The RT command/feedback path must be non-blocking, allocation-free, and caller-buffered. Register/configuration operations are bounded non-RT startup work and must finish before the motion loop is armed.
- A single bus-I/O owner should install exact standard-frame filters after discovery: known `MST_ID`s plus CAN error frames as required by the SocketCAN adapter. Do not rely on per-algorithm direct socket access.
- A missing reply is not a zero sample. It advances a missed/stale counter and eventually enters safe stop/fault according to the driver deadline.

### Code patterns found

Repository-local patterns:

- `crates/hardware_module/hardware-module-core/src/lib.rs:228-246` defines non-blocking, caller-buffered `FrameTransport::{receive, transmit, health}` and explicitly forbids allocation on RT implementations.
- `crates/hardware_module/hardware-module-core/src/lib.rs:249-319` provides typed command/protocol/stale-feedback errors and lifecycle operations suitable for a Damiao driver.
- `crates/integration/config-model/src/lib.rs:21-22,378-402` validates generic motor IDs as unique standard CAN IDs `1..=0x7ff`; Damiao's stricter low-nibble identity rule must be applied later at the vendor boundary.
- `platform/orangepi5plus/config/can.env:1-6` fixes `can0`, 1 Mbit/s, bus-error reporting, and no automatic restart.
- `platform/orangepi5plus/scripts/can-link.sh:64-69,112-128` validates and reads back that link contract without sending frames.

Pinned external implementation patterns (corroboration, not code to copy):

- Manufacturer SDK MIT packing and range presets: <https://gitee.com/kit-miao/motor-sdk/blob/ae360bde6a5af373e5a68440f80708bea321689d/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py#L86-L149>
- Manufacturer SDK lifecycle byte construction and register requests: <https://gitee.com/kit-miao/motor-sdk/blob/ae360bde6a5af373e5a68440f80708bea321689d/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py#L280-L318> and <https://gitee.com/kit-miao/motor-sdk/blob/ae360bde6a5af373e5a68440f80708bea321689d/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py#L426-L475>
- Manufacturer SDK feedback unpacking: <https://gitee.com/kit-miao/motor-sdk/blob/ae360bde6a5af373e5a68440f80708bea321689d/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py#L340-L370>
- Manufacturer SDK mode offsets and mode-aware lifecycle IDs: <https://gitee.com/kit-miao/motor-sdk/blob/ae360bde6a5af373e5a68440f80708bea321689d/C%2B%2B%E4%BE%8B%E7%A8%8B/u2canfd/include/protocol/damiao.h#L47-L64> and <https://gitee.com/kit-miao/motor-sdk/blob/ae360bde6a5af373e5a68440f80708bea321689d/C%2B%2B%E4%BE%8B%E7%A8%8B/u2canfd/src/protocol/damiao.cpp#L155-L220>

### Safe host-test matrix

All default CI tests must use pure codecs, an in-memory fake transport, or a caller-provisioned nonphysical `vcan` interface. They must never open the configured physical `can0` and never call a motor-transmit path.

1. MIT codec unit tests
   - Exact four golden vectors above.
   - Encode/decode every boundary and one-value-beyond each boundary.
   - Reject NaN, positive/negative infinity, invalid/nonpositive runtime limits, and non-MIT mode.
   - Verify truncation, bit ordering, and quantization tolerances.

2. Feedback codec unit tests
   - Decode the golden feedback frame.
   - Cover every known status plus every unknown nibble.
   - Preserve both temperature bytes.
   - Reject extended, RTR, error-frame-as-data, wrong arbitration ID, wrong low-nibble identity, and DLC other than 8.

3. System/register frame tests
   - Exact bytes for enable/disable/zero/clear-error without transmitting.
   - Exact read/write/store frames and little-endian `uint32`/IEEE-754 `float` values.
   - Correlation failures: wrong `MST_ID`, target ID, opcode, RID, DLC, or expired deadline.
   - TIMEOUT duration-to-tick conversion overflow and nonintegral-policy cases without writing a motor.

4. Driver lifecycle tests with a fake transport
   - Configure reads and validates `MST_ID`, `ESC_ID`, `CTRL_MODE`, `TIMEOUT`, `PMAX`, `VMAX`, `TMAX` before enable.
   - Enable/disable remain transitional until matching fresh status feedback arrives.
   - Fault, unknown status, stale feedback, TX partial/failure, wrong reply, and deadline expiry enter the required safe state.
   - Clear-error never auto-enables; zero/store are unavailable to ordinary command paths.
   - No allocation or blocking in the declared RT command/feedback functions.

5. SocketCAN adapter tests
   - Pure conversion between the project's frame type and Linux classic-CAN structure, including CAN flag masking.
   - Deterministic filter construction and transport-health counter behavior.
   - Optional `vcan` process-level round trip only when the operator/CI fixture has already provided an isolated interface. Test setup must not create or delete kernel interfaces by default.

Host tests can prove codec and state-machine correctness. They cannot prove motor firmware compatibility, physical disable, watchdog behavior, feedback timing, temperature accuracy, or a 1 kHz RK3588 deadline.

### Required target validation before claiming hardware support

- Record exact motor hardware revision, voltage variant, `hw_ver`, `sw_ver`, `sub_ver`, boot version if available, and the SHA-256 of the manual/driver build.
- With output mechanically safe and motion disarmed, capture `candump` plus an analyzer trace for register discovery and one state query. Confirm standard IDs, DLC, reply ordering, and whether any unsolicited feedback occurs.
- Read and record `ESC_ID`, `MST_ID`, `CTRL_MODE`, `TIMEOUT`, `PMAX`, `VMAX`, and `TMAX`; do not write/store them during initial qualification.
- Verify enable and disable only under an explicit hardware fixture, physical restraint, emergency-stop procedure, and operator approval. Require observed status transitions, not merely successful writes.
- Qualify timeout/fault/clear behavior separately. Never simulate communication loss on an unsupported or unrestrained mechanism.
- Do not exercise zero-position, ID changes, bitrate changes, mode persistence, or flash store in normal validation; each needs its own destructive maintenance procedure and rollback evidence.
- Measure command-to-feedback latency, loss, stale handling, and bus utilization at the intended motor count and 1 kHz schedule on RK3588. Host/vcan timing is not target evidence.

### Related specs

- `.trellis/spec/backend/directory-structure.md`: layer dependency and composition-root ownership. Apply the newly agreed names `hardware_module`, `protocol_module`, and `application_module` while preserving the same dependency direction.
- `.trellis/spec/backend/robot-contracts.md:19-41,55-83`: device contracts, minimal YAML, typed validation, and required tests.
- `.trellis/spec/backend/error-handling.md:3-32`: typed boundary errors, stale data, and numeric fault handling.
- `.trellis/spec/backend/quality-guidelines.md`: host gate and prohibition on unapproved hardware-changing tests.
- `.trellis/spec/backend/realtime-platform.md:45-78`: exact 1 Mbit/s link contract, explicit recovery, and separation of host from target qualification.

## Caveats / Not Found

- The exact DM-J4310-2EC firmware installed on the user's motors is unknown. Current-manual behavior must not be silently applied to legacy firmware. Read version registers and preserve target captures.
- The V1.2 manual does not define the semantic meaning of `TIMEOUT=0`, temperature signedness/sentinels/accuracy, feedback latency, retry counts, or a guaranteed maximum response time.
- The register table says `ESC_ID` may be `0..=0x7ff`, while the narrative recommends `CAN_ID < 16` and feedback carries only four embedded ID bits. This research recommends `1..=15` as a framework safety policy, but that policy is an engineering decision based on the documented ambiguity, not a manufacturer-declared hard limit.
- The feedback status table omits codes `0x3..0x5`, although the same manual lists them as faults and says faults can be viewed via feedback. Preserve unknown codes and confirm these three with a safe diagnostic fixture before depending on their names.
- The manual's read/store request tables show only four data bytes, while SDK examples pad to DLC 8. The recommended transmitter uses DLC 8; target capture must establish reply DLC variants.
- Manufacturer SDK examples disagree about lifecycle arbitration IDs outside MIT mode, and one legacy helper uses a different enable-ID calculation. This task is MIT-only, where all relevant sources resolve to `ESC_ID`; do not generalize the driver to other modes without separate research and target tests.
- Manufacturer SDK code is mainly written for DAMIAO USB adapters, not Linux SocketCAN. Its serial/USB framing, sleeps, receive loops, hash maps, process exits, and implicit clamping are not suitable patterns for this Rust RT path.
- The manufacturer SDK repository has no root license and the inspected C++ files have no adjacent license. No C++ code should be copied. The Python example subtree is explicitly MIT, but an independent Rust implementation from the documented wire facts is preferable.
- The community `jia-xie` documentation's claim of continuous feedback contradicts the current product manual's query-response statement. Treat community behavior as implementation-specific polling, not motor firmware truth.
- No manufacturer-supplied SocketCAN reference, conformance vector set, formal protocol version negotiation, checksum/CRC field, or error-reset acknowledgement contract was found.
