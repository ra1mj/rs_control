# rs_control 使用文档

[English version](usage-guide.md)

本文是 `rs_control` 的操作入口文档，覆盖当前支持的开发与运维流程，并将目标板相关工作引导至对应的权威运行手册。本文不会提供电机使能流程，也不会把主机结果描述为 Orange Pi、CAN 链路、电机或实时 profile 的资格证明。

## 1. 范围与安全等级

执行命令前先确认所属的安全等级。

| 等级 | 可执行内容 | 能够证明的范围 |
|---|---|---|
| 主机安全/只读 | 构建、测试、校验配置、运行机械臂仿真、渲染行为树、检查已有诊断 IPC，或检查平台计划。 | 仅证明当前主机上的软件与契约行为。 |
| 仅目标板且电机失能 | 构建/安装锁定内核、读取板卡状态、准备候选 profile、配置仅分析仪使用的 CAN 链路，以及采集资格证据。 | 只有证据包及其独立审核结论才能为该精确目标配置提供资格证明。 |
| 物理安全门 | 使用真实电机、运动电源、物理停止路径或安全夹具。 | 本仓库中的任何单个流程目前都不能独立关闭此门。必须遵循其所属的验收程序。 |

已提交的 Orange Pi profile 是**候选项**。`PROFILE_QUALIFIED=0` 与 `QUALIFICATION_EVIDENCE_SHA256=unqualified` 表示生产准备服务不能使用它。失败的 1 kHz 结果必须保留为证据，不能静默切换到 500 Hz。

## 2. 仓库结构与前置条件

以下命令均假定运行在 Linux 开发环境中，且代码检出干净。先进入仓库根目录，确保相对路径按本文所述解析：

```bash
cd "$(git rev-parse --show-toplevel)"
```

工作区使用已提交的 `rust-toolchain.toml` 与 `Cargo.lock`。执行 Cargo 前，请按项目的 `rustup` 规范安装 Rust。完整主机质量门还会使用 `uv` 运行已提交的 Python 示例。Docker 仅用于可选的 Orange Pi 内核可复现构建。主机快速开始中的命令都不需要 `sudo`，不会修改 `/sys` 或 `/proc`，不会配置网络链路，也不会发送 CAN 流量。

依赖方向是单向的：

```text
application_module algorithms, control, FSM
                  |
                  v
protocol_module    messages, local ports, Zenoh, Protobuf
                  |
                  v
hardware_module    transports and device protocols
```

`apps/` 中的组合根可以连接已批准的层。应用逻辑不得绕过 protocol 边界直接打开设备后端。

## 3. 主机快速开始

### 3.1 校验机器人配置

`robotctl` 只处理配置。它不会打开设备、配置 CAN 链路或启动运动。

```bash
cargo run --locked -p robotctl -- \
  config validate config/robots/example.yaml

cargo run --locked -p robotctl -- \
  config print-normalized config/robots/example.yaml
```

第一个命令输出稳定的 SHA-256 标识；第二个命令输出规范化 YAML。校验失败是预期的保护行为，应修正输入，而不是编辑生成输出或放宽 schema。

### 3.2 检查层级边界

```bash
cargo run --locked -p architecture-check -- check
```

此命令校验 Cargo 依赖策略。它是主机安全的静态检查，不能证明调度、I/O、CAN 或电机行为。

### 3.3 验证机械臂仿真

```bash
tools/run-arm-simulation.sh verify
```

该命令重新运行确定性的关节机械臂仿真，并将结果与已提交的证据报告逐字节比较。若需生成独立报告，请提供一个不存在的路径；已有报告会被验证而不会被覆盖：

```bash
tools/run-arm-simulation.sh qualify /absolute/path/new-arm-report.json
```

仿真覆盖预计算运动学、动力学、重力前馈与有界负载辨识。它不打开硬件或 ROS 端点，不能证明物理机械臂、控制器增益、实时性或负载限制。验收与回滚规则见 [Arm Payload Identification and Adaptive Gravity Compensation](arm-payload-identification.md)。

### 3.4 渲染行为树示例

```bash
cargo run --locked -p btctl -- example
```

该命令构造已检查的行为树示例，并回放确定性的启动事件。它不会打开 IPC、ROS、执行器或硬件边界。运行时是事件驱动的：嵌入方提供 deadline 与 I/O 结果；行为树本身不会形成控制循环，也不会授予 authority。

### 3.5 运行完整主机质量门

```bash
tools/ci.sh
```

这是仓库完整的主机侧质量门。它运行平台夹具测试、格式化检查、将警告视为错误的 Clippy、工作区测试、架构与 Protobuf 兼容性检查、仿真证据验证、Buf 检查、依赖策略检查，以及所有已提交机器人 YAML 的校验。

它刻意**不会**获取、构建或安装内核，不会写入目标内核状态、配置真实网络链路、发送 CAN 帧、使能电机，也不会建立 ARM/实时资格证明。

## 4. Robot YAML

Robot YAML 只用于选择拓扑。最小麦克纳姆示例如下：

```yaml
schema_version: 1
chassis:
  morphology: mecanum_4
  motor_ids:
    front_left: 1
    front_right: 2
    rear_left: 3
    rear_right: 4
arm:
  morphology: none
  motor_ids: {}
imu:
  transport: uart
  baud_rate: 921600
```

已支持的示例位于 `config/robots/`：`example.yaml`、`omni_3.yaml`、`omni_4.yaml` 与 `swerve_4.yaml`。解析器会拒绝未知字段、无效构型角色、重复或越界的 ID、不支持的 schema 版本、无效 transport 以及无效 baud rate。

不要将 CAN 帧布局、字节序、校验和、增益、设备路径、Zenoh 设置、实时调参或可执行表达式写入该文件。这些内容属于经过审查的代码和平台配置，而非机器人拓扑。

## 5. 行为树与导航观测

### 5.1 观测已有导航 IPC

电机失能的导航组合会发布行为树参与状态与传输计数的 latest 诊断快照。`btctl` 仅用于观测；它不能取得 lease、刷新命令 deadline 或创建输出。

```bash
cargo run --locked -p btctl -- inspect \
  --ipc-root /absolute/path/to/navigation-ipc

cargo run --locked -p btctl -- observe \
  --ipc-root /absolute/path/to/navigation-ipc \
  --plain --samples 1
```

`--ipc-root` 必须是已经存在的绝对目录。缺失、过期、格式错误或不兼容的快照会被报告为 unavailable，绝不会被显示为健康的零值状态。需要有界、机器可读的观测流时，加入 `--interval-ms 100 --samples 20`。全部观测选项和运行时语义见 [Project-native behavior tree](behavior-tree.md)。

### 5.2 运行电机失能导航自检

已提交的导航 manifest 明确声明 `motor_output: forbidden`、`can_transmit: forbidden`、`simulation_only: true` 与 `approved_for_production: false`。应从精确文件导出所需 digest，而不是手工复制：

```bash
nav_manifest=config/navigation/rs_control_navigation_plugin.yaml
nav_digest=$(sha256sum "$nav_manifest" | awk '{print $1}')

cargo run --locked -p rs-navigation-e2e -- \
  --manifest "$nav_manifest" \
  --expected-sha256 "$nav_digest" \
  self-test
```

自检会针对 fake sink 运行一条合成的 authority/proposal/stop 路径。预期输出包含 `motor_output=forbidden`。它不是 ROS 2 部署，也不是物理导航测试。

隔离 bridge、Nav2 profile、service unit、主机证据采集及故障模式请遵循 [Motor-disabled chassis navigation plugin](navigation-plugin.md) 与 [the deployment runbook](../platform/ros2-humble/navigation-deployment/README.md)。这些流程保持电机失能边界。

## 6. Motion Runtime 与遥测记录

`apps/motion-rt` 当前提供主机专用的组合库和空的可执行入口。没有可供记录或执行的生产 `motion-rt` 命令行操作。不要从主机 systemd 示例推断出运维流程。

`HostMotionRuntime` 使用固定容量本地通道、注入的控制 tick、fake motor-disabled sink、类型化 safe-stop 决策以及非实时诊断 drain。预期的 bridge、control worker、recorder 或诊断消费者丢失时，主机测试只会保留 fail-closed 的 fake 输出路径。`apps/motion-rt/systemd/motion-rt-host.service` 中的示例固定 `RS_CONTROL_MOTOR_MODE=disabled`；它是最小权限的主机示例，不是目标板部署 unit。

`telemetry-recorder` crate 同样是库边界，而不是 CLI。只能从非实时 owner 集成：

1. 从原始已编码的 Protobuf bytes 创建有界 `TelemetryRecord`。topic 必须是可打印 ASCII 且不超过 96 bytes；payload 不超过 4,096 bytes；source sequence 不得为零。
2. 在 producer 中调用 `TelemetryIngress::try_push`。它为 `Critical`、`Control` 和 `Bulk` 分别提供固定容量，并将丢失记录为 gap，而不是等待 consumer。
3. 在非实时 drain worker 中，使用已有绝对输出目录和不可变 byte/quota/chunk 限制打开 `TelemetryRecorder`；在记录边界调用 `drain_into`，然后调用 `seal`。
4. 仅通过 `ReplayPolicy` 与 `replay_file` 回放已 seal 的 `.mcap`，并提供已编译 descriptor digest、显式 topic allowlist 与输入上限。回放返回已校验输入，不能构造输出命令。

recorder 使用 create-new staging 输出、有界 MCAP 文件轮转、manifest sealing、完整 staging 恢复，并在 storage 或 quota 压力下显式降级。它保留原始 payload bytes 并记录 gap metadata。recorder 故障绝不能阻塞 realtime producer 或创建 motion authority。

## 7. Orange Pi/RK3588 目标板交接

### 7.1 主机侧安全平台检查

以下命令校验已提交的输入与夹具行为，不会修改板卡或创建资格证据：

```bash
platform/orangepi5plus/kernel/build-kernel.sh --verify-only
platform/orangepi5plus/kernel/build-kernel.sh --print-plan
platform/orangepi5plus/scripts/rt-affinity.sh plan
platform/orangepi5plus/scripts/can-link.sh plan
platform/orangepi5plus/tests/host-test.sh
platform/orangepi5plus/tests/middleware-host-test.sh
platform/orangepi5plus/scripts/build-horus-adapter.sh --verify-only
```

### 7.2 仅目标板的前置条件

执行任何有特权的目标板操作前，准备真实 Orange Pi 5 Plus，并记录 board revision、稳定电源、active cooling、serial-console recovery、保留的 vendor-kernel boot path、隔离式 3.3 V CAN transceiver、正确的两端终端电阻，以及 CAN analyzer 或 non-motor loopback peer。平台 bring-up 阶段必须移除运动电源并断开电机。

板卡的 TX/RX header pin 是逻辑信号，不是 CANH/CANL。不得直接将它们接入 CAN 总线。候选 `0-5 / 6 / 7` CPU role map 不是将不同主机 pin 到 CPU 的许可，也不允许使用“第一个能工作的核”作为 fallback。

目标板流程被刻意分阶段执行：

1. 按 [Build and install the pinned kernel](../platform/orangepi5plus/docs/build-and-install.md) 构建和安装锁定内核，并保留 serial recovery。
2. 按 [Bring up the board and analyzer-only CAN link](../platform/orangepi5plus/docs/bring-up.md) 完成板卡与仅分析仪 CAN 链路 bring-up。`rt-affinity.sh apply` 和 disruptive CAN 操作需要显式 `--confirm-disarmed`；生产准备额外需要 `--require-qualified`。
3. 使用唯一且不存在的输出目录以及已填写的 hardware manifest，执行 [realtime qualification procedure](../platform/orangepi5plus/docs/realtime-qualification.md)。
4. 任何 profile promotion 前，独立审核并保留 raw output、嵌套 manifest 和顶层 SHA-256。调参、CAN 或 boot 失败后使用 [rollback procedure](../platform/orangepi5plus/docs/rollback.md)。

完整的 concurrent 结果需要精确的目标板标识、锁定的 source 与 build ID、CPU/IRQ/governor readback、memory lock policy、CPU/memory/storage、真实 Ethernet、accelerator、thermal/frequency 观测以及至少八小时的持续时间。1 kHz 通过要求在约定 soak 中最大唤醒抖动不超过 100 microseconds，且没有 application deadline miss。缺少输入或短时 research run 的结论是 `incomplete`，而不是通过。

middleware 选择遵循相同标准。它在 latest/ordered 语义、payload size 以及 normal/slow/stopped/killed peer 条件下比较 local、SHM 与锁定的外部 Horus adapter。`qualify-middleware.sh` 会拒绝错误 architecture、board、clean-source identity、locking/FIFO/perf preflight 或已存在输出。内部完整性有效的 sealed `fail` 或 `incomplete` bundle 仍可通过验证，但只有 `qualification_status=pass` 才有资格用于 profile 决策。

## 8. 物理门、故障响应与相关文档

本文中的任何命令都不会使能 torque、发送 motor frame、证明硬件 watchdog 行为或验证物理停止。真实 SocketCAN 与 DM-J4310-2EC 工作仍属于独立拥有的受约束流程，必须具备 safety fixture、current limit、显式授权和不可变目标板证据。

目标板调参、CAN、thermal 或 boot 失败时：保持运动物理失能，保留证据目录，不要将通过 digest 复用于发生变化的 board/kernel/device tree/profile/build，并执行目标板 [rollback and evidence retention](../platform/orangepi5plus/docs/rollback.md) 流程。不得将 `fail` 或 `incomplete` 输出变成 candidate profile。

下表列出各领域详细流程的权威文档：

| 主题 | 权威文档 |
|---|---|
| 平台概览与主机安全检查 | [Orange Pi 5 Plus / RK3588 platform baseline](../platform/orangepi5plus/README.md) |
| 内核构建与受控安装 | [Pinned kernel build and guarded installation](../platform/orangepi5plus/docs/build-and-install.md) |
| 板卡、overlay 与 CAN bring-up | [Orange Pi 5 Plus bring-up](../platform/orangepi5plus/docs/bring-up.md) |
| 时序与 middleware 证据 | [Realtime qualification](../platform/orangepi5plus/docs/realtime-qualification.md) |
| 目标板故障恢复 | [Rollback and evidence retention](../platform/orangepi5plus/docs/rollback.md) |
| 行为树语义 | [Project-native behavior tree](behavior-tree.md) |
| 电机失能导航集成 | [Motor-disabled chassis navigation plugin](navigation-plugin.md) |
| 机械臂负载辨识 | [Arm Payload Identification and Adaptive Gravity Compensation](arm-payload-identification.md) |

需要精确 CLI synopsis 时，请使用已提交的 executable 或 script help，不要猜测 flag：

```bash
cargo run --locked -p robotctl -- --help
cargo run --locked -p btctl -- --help
cargo run --locked -p rs-arm-simulation -- --help
cargo run --locked -p rs-navigation-e2e -- --help
platform/orangepi5plus/scripts/rt-affinity.sh plan --help
platform/orangepi5plus/scripts/can-link.sh plan --help
```
