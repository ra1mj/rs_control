# Orange Pi 5 Plus realtime platform

## Goal

为 Orange Pi 5 Plus（RK3588）建立一个可重建、可回滚、可量化验收的 Ubuntu 22.04 Server + PREEMPT_RT 平台基线，为后续 Rust SocketCAN 驱动、1 kHz 控制线程和 Zenoh 中间层提供可信运行环境。

本任务既交付可在普通开发机验证的源码锁定、构建脚本、平台配置和验收工具，也定义必须在目标板上补齐的硬件证据。没有目标板证据时可以继续开发后续软件切片，但不得把本平台标记为“已通过实时/硬件验收”。

## Scope

### In scope

- 固定 Orange Pi 5 Plus 的官方 BSP、RT 分支、构建基线、交叉工具链、设备树 overlay、`rt-tests` 和构建容器来源。
- 提供从固定源码生成内核配置与 Debian 内核包的可重复构建入口；普通 CI 只执行快速、无网络、无大规模内核编译的静态验证。
- 保留 Orange Pi 官方 RT 分支的真实来源链，包括它把 `5.10.180-rt89` 补丁集成到标称 `5.10.160` BSP 的版本偏差，不伪装成严格同版本补丁。
- 以项目配置片段强制启用 PREEMPT_RT、1 kHz tick、高精度定时器、CPU 隔离能力、SocketCAN、Rockchip CAN、USB/MIPI/V4L2 和内核配置自描述能力。
- 覆盖官方 RT 配置中关闭 CPU 频率热控的设置；项目基线必须保留热保护，温度降频或证据缺失均不得得到“已通过”结果。
- 提供 CAN0/CAN1 overlay、3.3 V 逻辑侧隔离 CAN 收发器、CANH/CANL 和两端 120 Ω 终端的接线/检查说明。
- 提供 classic CAN 1 Mbps、错误帧开启、自动 bus-off 重启关闭、显式失能后恢复的幂等 SocketCAN 管理入口。
- 提供 RK3588 核心拓扑发现、候选 CPU/IRQ 亲和性配置、应用前验证和应用后回读；不得把逻辑 CPU 列表误实现成“任选一个核心”。
- 提供 `cyclictest` 基线与压力验收包装器，记录 CPU、内存、存储、网络和加速器并发压力所需证据。
- 记录板卡、供电、散热、内核、设备树、CPU governor、温度、CPU/IRQ 亲和性、CAN 状态、`rt-tests` 和应用构建身份。
- 参考 Horus 的核心绑定、共享内存和进程间队列实现；本任务只落地平台核心绑定与测量方法，共享内存/队列实现约束回写到父任务的中间层阶段。
- 提供 systemd 示例与最小权限说明；平台准备单元不得承担机器人业务逻辑。

### Out of scope

- 达妙电机协议、使能、转矩命令或任何会产生运动的硬件测试。
- 中间层共享内存、进程队列、Zenoh、Protobuf 数据面的具体实现。
- `motion-rt` 控制循环、底盘闭环、机械臂、视觉、FSM 或逻辑编排器实现。
- 自动修改引导分区、覆盖当前可启动内核或在没有串口恢复路径时写入设备树配置。
- 把 Horus 引入为运行时依赖，或复用其未经本项目安全审计的 `unsafe` 共享内存代码。
- 宣称普通以太网、NPU/GPU 或具体 CAN 收发器型号已通过硬件验收，除非证据包中有真实目标板结果。

## Requirements

### Reproducible platform baseline

- PB-R1：目标基线必须是 Orange Pi 5 Plus / RK3588、Ubuntu 22.04 Server、Orange Pi/Rockchip 5.10 BSP 和 PREEMPT_RT。
- PB-R2：所有外部源码必须记录官方 URL、不可变提交或标签、必要的 SHA-256 和许可证；构建不得依赖可漂移的分支头或 `latest` 镜像。
- PB-R3：内核构建必须从干净目录运行，使用显式工作目录/输出目录，不修改工作区源码，并产生内核配置、源码身份和产物校验和。
- PB-R4：构建入口必须支持仅打印计划和仅验证锁文件，使普通 CI 不下载工具链或编译内核。
- PB-R5：Orange Pi RT 分支的版本偏差、Rockchip 修补提交和上游 RT 补丁哈希必须进入证据；任何 BSP/RT/DT 升级都触发完整板级回归。
- PB-R6：项目内核片段必须强制 `CONFIG_CPU_FREQ_THERMAL=y`；热保护不得为了更好看的延迟数字被关闭。
- PB-R7：已知可启动的供应商内核、串口恢复方法和回滚步骤必须在安装新内核前可用；脚本不得自动删除旧内核。

### Board and CAN bring-up

- PB-R8：必须明确 CAN0/CAN1 的 overlay 名称、40-pin 排针引脚、收发器 3.3 V 逻辑电平、隔离要求和终端电阻规则。
- PB-R9：SocketCAN 管理必须固定 classic CAN 1 Mbps，开启 bus-error reporting，设置 `restart-ms 0`，并在配置后回读验证。
- PB-R10：运行中的 CAN 链路不得被无条件 down/up；任何显式恢复都必须确认机器人已失能。本阶段不得连接或使能达妙电机。
- PB-R11：CAN 回环或总线分析仪收发、错误帧和 bus-off 行为必须先于后续电机任务验证。

### Realtime and affinity

- PB-R12：核心选择必须先读取目标板在线 CPU、capacity/频率拓扑和当前 cpuset，再使用 Linux 逻辑 CPU ID；候选默认值不是验收结论。
- PB-R13：控制 CPU、CAN I/O CPU、housekeeping CPU 和 CAN IRQ CPU 的角色必须显式、可回读且冲突受检。CAN IRQ 不得被“搬离所有 RT 核心”的通用循环误放到慢核或不可用 CPU。
- PB-R14：合格生产 profile 中，缺少 PREEMPT_RT、`SCHED_FIFO`、`mlockall`、所需 CPU 亲和性或指定 IRQ 亲和性时必须拒绝启动；仅开发 profile 可以明确标记为非合格后继续。
- PB-R15：不默认关闭 RT throttling，不盲目复制 `isolcpus/nohz_full` 参数；候选的 affinity/cpuset/隔离方案必须在 RK3588 上比较最大抖动和热行为后选择。
- PB-R16：`cyclictest` 必须使用绝对定时、纳秒输出、内存锁、固定测量 CPU 和独立 main-thread CPU；结果解析必须验证工具返回码和单位。
- PB-R17：1 kHz profile 的间隔为 1 ms，验收阈值为最大唤醒延迟不超过 100 us；压力浸泡期间不得出现应用截止期限丢失。
- PB-R18：目标板正式资格测试采用至少 8 小时并发压力浸泡。若 1 kHz 失败，必须保留失败证据并显式选择、重新完整验收 500 Hz；运行时不得静默换频。
- PB-R19：压力矩阵必须包含 CPU、内存、目标存储、实际以太网和 NPU/GPU 工作负载。缺少任何必需压力源时，结果只能是 `incomplete`，不能是 `pass`。
- PB-R20：发生温控降频、监测中断、证据文件缺失、核心迁移或测试进程异常时，资格结果必须失败或不完整，不能只依据平均值通过。

### Horus-informed communication constraints

- PB-R21：Horus 仅作为有版本的研究输入，必须固定所审阅的提交并区分“可借鉴模式”和“不可复制风险”。
- PB-R22：平台核心绑定实现必须使用精确 CPU 集合并在设置后回读；不得复制 Horus `pin_to_cores` 的“列表中首个成功核心”语义。
- PB-R23：后续中间层本地 IPC 必须评估 Horus 的缓存行隔离 SPSC 矩阵、每槽版本戳、崩溃后 `flock` 回收和 latest/drop 策略，但不得以 `needs_drop::<T>() == false` 自动认定共享内存 POD 安全。
- PB-R24：后续共享内存的 `unsafe` 必须隔离在中间层私有边界，要求显式 `Pod`/等价安全契约、布局版本、端序/ABI 策略、溢出计数和崩溃/撕裂/慢消费者测试；上层仍只依赖中间层接口。

### Evidence and operations

- PB-R25：每次资格测试必须产生不可覆盖的证据目录，包含源码锁摘要、板卡/内核/DT/启动参数、CPU/IRQ/governor/温度、CAN、cyclictest 原始 JSON、压力进程状态、判定结果和文件 SHA-256。
- PB-R26：所有平台脚本必须有 `--help` 或清晰子命令、默认安全、严格参数校验和 dry-run/plan 能力；普通主机运行测试不得需要 root 或真实硬件。
- PB-R27：systemd 示例必须把有权限的平台准备与无特权业务进程分开；核心绑定、IRQ、CAN 配置失败必须可观察。

## Acceptance Criteria

### Host-verifiable gate

- [x] 源码锁固定官方 Orange Pi RT 内核提交、官方构建提交、工具链和容器哈希、`rt-tests` v2.10 提交以及所审阅 Horus 提交。
- [x] 内核构建的 `--verify-only` 和 `--print-plan` 在干净主机上通过，不执行下载或构建；配置检查证明热保护和必需 RT/CAN/media 选项未被供应商片段覆盖。
- [x] 平台主机测试覆盖危险参数拒绝、CPU 列表解析/冲突、CAN 1 Mbps/restart 禁用计划、cyclictest 1 kHz/500 Hz 命令和结果判定边界。
- [x] `tools/ci.sh` 运行现有 Rust/Protobuf/架构门禁，并加入快速平台检查，不触发目标板变更或大规模内核编译。
- [x] 文档给出构建、安装、CAN overlay/接线、实时调优、8 小时资格测试、证据解释和回滚流程。
- [x] 父任务明确保留 Horus 启发的中间层共享内存/进程队列验收项，同时禁止架构层级绕过和不安全 POD 自动检测。

### Target-board evidence gate

- [ ] 从固定来源构建出 Debian 内核包，记录产物 SHA-256，并在保留旧内核的情况下启动 Orange Pi 5 Plus。
- [ ] 目标板报告预期 PREEMPT_RT、热保护、USB、MIPI/V4L2、网络、存储和 CAN 内核能力。
- [ ] 通过外接 3.3 V 逻辑侧隔离收发器和分析仪/回环在 can0 或 can1 完成 1 Mbps 收发、错误帧与显式 bus-off 恢复；没有电机使能或转矩动作。
- [ ] affinity/IRQ 配置应用后回读与期望一致，测试线程不迁移，CAN IRQ 位于指定 I/O CPU，housekeeping 保留至少一个 CPU。
- [ ] 在 active cooling、固定供电和完整 CPU/内存/存储/以太网/NPU/GPU 压力下完成 8 小时 1 kHz 资格测试，最大唤醒延迟 `<= 100000 ns`，无热降频、工具错误和应用丢期。
- [ ] 若 1 kHz 未通过，失败证据仍被保存；500 Hz 只有在同一完整矩阵单独通过后才能成为显式资格 profile。
- [ ] 回滚演练能够从串口/引导菜单进入保留的供应商内核，且不删除用户数据或证据包。

## Completion Rule

主机侧实现和检查完成后，本任务保持 `in_progress`，直到上述目标板证据门禁全部满足。后续软件任务可以并行推进，但任何依赖本平台的硬实时结论均标记为“待 RK3588 目标板验证”。

## Notes

- 用户已明确批准进入后续阶段并要求常规工作持续推进；该授权覆盖本任务规划完成后的激活、实现、检查、提交和推送，不取消物理急停、失能确认、凭据或不可逆硬件操作的安全门禁。
- 机器人拓扑 YAML 仍只包含机械臂构形/电机 ID、IMU 通讯方式/波特率、底盘构形/电机 ID；平台 affinity、CAN link 和内核资格配置存放于 `platform/orangepi5plus/`，不得扩张机器人 YAML。
