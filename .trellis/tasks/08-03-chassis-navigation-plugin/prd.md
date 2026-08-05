# Integrate chassis navigation as an rs_control plugin

## Goal

Integrate `/home/mj/workspaces/qi/chassis_nav` as a crash-isolated navigation
plugin for `rs_control` on a resource-constrained robot computer, while Rust
retains local command authority, freshness enforcement, motion constraints and
the only path to hardware output.

## Confirmed background

- The `rs_control` parent architecture requires ROS 2 compatibility to remain
  in a standalone bridge process. Nav2 is an upper-layer client and must not
  enter `motion-rt` or access a driver directly (parent requirements R65,
  R67-R70 and R83).
- `rs_control` already defines a lease-required ROS command ingress at
  `/robot_1/cmd_vel_stamped`, with a 100 ms maximum age, local-monotonic
  deadline construction and a queue depth of one.
- `control-core`, `trajectory-se2` and the chassis morphology crates already
  own local speed/acceleration limiting and body-to-wheel projection.
- `chassis_nav` has an extracted headless Nav2 launch and synthetic x86
  validation, but its installed robot/map parameters are explicitly
  simulation-only. The current Nav2 output is an unstamped `Twist`, and the
  standalone profile still contains `nav2_velocity_smoother`.
- The robot must run SLAM/localization and other motion-control workloads at
  the same time. A Nav2-only RSS result is component evidence, not a host
  deployment budget.
- The user selected a motor-disabled end-to-end goal, command, feedback and
  fault loop. The host harness may issue synthetic authority, but the ROS
  adapter itself may not grant or extend that authority.
- The `rs_control` root manifests are currently clean, while the task tree and
  Trellis bootstrap files contain unrelated work. The `chassis_nav` worktree
  has extensive pre-existing modified and untracked navigation/localization
  changes. Implementation must preserve both repositories' unrelated edits
  and make minimal additive changes to shared manifests and launch packages.

## Requirements

- **NPI-01 — Isolation:** Nav2, GLIM and ROS-facing adaptation run outside the
  realtime motion process. Their crash, restart, blocking work or memory
  pressure cannot terminate `motion-rt`, extend a lease or bypass a local safe
  stop.
- **NPI-02 — Rust authority:** The plugin cannot mint a resource lease. Every
  command reaching the Rust control path must match the active `chassis`
  lease, current session epoch, configured owner and local freshness bound.
- **NPI-03 — One final constraint owner:** The integrated profile must not
  cascade two independent velocity/acceleration smoothers. Rust owns final
  command limiting, morphology projection and hardware output; the Nav2 side
  emits a bounded controller proposal.
- **NPI-04 — Explicit interfaces:** Map identity, localization state,
  odometry, TF, obstacle observations, navigation goals, action feedback,
  cancellation, controller proposals and plugin health use explicit,
  versioned interfaces. Topic names, local shared-memory channels, frame
  aliases, units, QoS, bounds and single-writer ownership are checked rather
  than inferred.
- **NPI-05 — Action lifecycle:** A mission-owned navigation action has a
  stable command ID and reports accepted/running/succeeded/failed/canceling/
  cancelled outcomes. Publishing a goal or accepting cancellation is not a
  terminal result.
- **NPI-06 — Fail-closed startup:** Navigation defaults inactive. It becomes
  eligible only after the exact approved map and localization readiness gate
  pass and the mission owns the chassis lease. A launch argument alone is not
  safety authority.
- **NPI-07 — Supervision boundary:** systemd/deployment owns starting and
  restarting ROS/plugin processes. The Rust FSM consumes health and controls
  mission/lease state; it does not become a general process supervisor.
- **NPI-08 — Bounded failure behavior:** Missing/expired authority, stale or
  malformed commands, duplicate publishers, localization loss, ROS/DDS/local
  IPC loss, Nav2 exit, adapter exit, cancellation and restart all produce a
  typed rejection or bounded safe stop. No old command is replayed after a
  restart.
- **NPI-09 — Low-resource evidence:** Qualification measures the complete
  concurrent deployment slice: navigation, localization/SLAM profile, Rust
  motion-control workload, bridge and supervision. Record CPU, cgroup memory,
  threads, context switches, deadlines and fault behavior; retain per-process
  attribution so ROS overhead is visible.
- **NPI-10 — Provenance and compatibility:** Both repositories bind the same
  plugin manifest/interface identity, source or artifact digests, ROS Humble
  compatibility and `rs_control` Protobuf/route schema. Mismatch fails before
  motion eligibility.
- **NPI-11 — Evidence honesty:** Synthetic x86 testing may validate contracts,
  actions and faults but cannot approve a production map, physical sensors,
  traction, calibration, AArch64 timing or whole-robot deployment.
- **NPI-12 — Motor-disabled end-to-end scope:** This delivery runs the complete
  mission goal, Nav2 planning/control, Rust authority/constraint and simulated
  chassis feedback loop while a hard output gate prevents motor enable and CAN
  command transmission. A command reaching a fake/simulation sink is evidence
  of software integration only.
- **NPI-13 — Minimum local runtime:** The co-located profile uses the existing
  fixed-capacity shared-memory protocol path for high-rate Rust/ROS-adapter
  exchange. It does not start `zenohd`, `zenoh-bridge-ros2dds` or a second
  command smoother. The isolated Rust adapter is the only new DDS participant.
- **NPI-14 — Measurement-only first resource run:** Because no commissioned
  target budget or target hardware is available, the first concurrent host run
  records measurements and regressions without declaring target suitability.
  Numeric pass/fail budgets and AArch64 qualification remain a later product
  and hardware decision.

## Acceptance criteria

- [x] Architecture checks prove no ROS, Nav2 or GLIM dependency is linked into
      `motion-rt`, hardware crates or the Rust realtime hot path.
- [x] A checked-in, versioned plugin manifest is validated by both repositories
      and rejects a mismatched launch/config/interface identity before startup.
- [x] The integrated headless profile starts no Gazebo, RViz, scenario runner,
      online mapping worker, `zenohd`, `zenoh-bridge-ros2dds` or redundant ROS
      command smoother.
- [x] With motors disabled and synthetic inputs, a mission navigation goal can
      be accepted, executed and completed/cancelled through the full action
      lifecycle, and Nav2 proposals reach the Rust command boundary only under
      the exact active chassis lease.
- [x] Missing/expired/wrong-owner leases and stale, duplicate, reordered or
      prior-boot proposals produce no executable chassis command.
- [x] Nav2, bridge, localization and local-IPC fault injection each causes the
      declared bounded stop/cancel behavior while the Rust control process
      remains alive.
- [x] Publisher/TF ownership tests prove one final command owner and one owner
      for each dynamic transform edge.
- [x] Host CI, architecture policy, formatting, Clippy, Rust tests, ROS
      contracts and cross-repository compatibility checks pass without
      overwriting unrelated arm-stack work.
- [x] A concurrent resource report separates the Nav2 slice from the Rust
      control/bridge/SLAM slices and leaves real AArch64 whole-robot
      qualification explicitly pending until target evidence exists.

## Out of scope

- Approving the current synthetic map or parameters for production.
- Physical motor enable, ground motion, traction or sensor calibration.
- Claiming AArch64 realtime or memory qualification from x86 host evidence.
- Embedding ROS, C++ plugins or a Python interpreter into `motion-rt`.
- Replacing Nav2/GLIM algorithms with new Rust planners or SLAM in this task.
