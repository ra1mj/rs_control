# Realtime process and configuration research

## Scope

Evidence for the robot configuration model, PREEMPT_RT deployment, process isolation, low-copy camera transport, Zenoh transport, and lightweight supervision/FSM design.

## Findings

### rm-controls configuration model

- The referenced project is assumed to be `rm-controls/rm_control`.
- Its `rm_hw.yaml` example separates bus selection and actuator instances. Each actuator binds a logical name to `bus`, `id`, and `type`.
- It also exposes `loop_frequency`, cycle-time error threshold, and realtime thread priority.
- This project will retain the useful logical-name-to-device binding pattern, but the user has explicitly limited the initial robot YAML to arm morphology/motor IDs, IMU transport/baud rate, and chassis morphology/motor IDs.
- Controller tuning, protocol frame descriptions, and arbitrary expressions do not belong in the initial robot YAML.

Source: https://rm-control-docs.netlify.app/quick_start/rm-controls_101

### PREEMPT_RT and CPU isolation

- PREEMPT_RT makes most kernel execution preemptible and moves interrupt handling into schedulable threads. `SCHED_FIFO` priority relationships therefore need to include relevant IRQ threads, not just application threads.
- CPU isolation can reduce unrelated load balancing and unbound-workqueue interference, but CPU placement must be validated on each SoC rather than copied between RK3588 and RDK X5.
- cgroup v2 has limitations around realtime bandwidth control under `CONFIG_RT_GROUP_SCHED`; deployment must verify the target kernel configuration before relying on cgroup CPU controls.

Sources:

- https://docs.kernel.org/core-api/real-time/theory.html
- https://docs.kernel.org/admin-guide/cgroup-v2.html

### Zenoh and in-process realtime paths

- Zenoh supports pub/sub, query, configurable deployment topology, batching, priorities, and shared-memory transport.
- Shared-memory support reduces payload copies across processes, but it does not remove executor scheduling, queueing, backpressure, or lifecycle concerns.
- The deterministic realtime loop should therefore access a middle-layer-owned preallocated mailbox/ring buffer. The same middle layer mirrors selected data to Zenoh/Protobuf for cross-process and cross-node consumers.
- Candidate mechanisms must be benchmarked on both target boards before final selection.

Sources:

- https://zenoh.io/docs/getting-started/deployment/
- https://zenoh.io/blog/2025-12-11-zenoh-jiaolong/

### Camera buffers

- Linux V4L2 supports DMABUF exchange using file descriptors and buffer metadata rather than copying image planes.
- USB and MIPI camera backends should converge on a buffer-handle abstraction where platform support permits, while Protobuf carries metadata or remote compressed payloads rather than raw local frames in the realtime path.

Source: https://docs.kernel.org/userspace-api/media/v4l/dmabuf.html

## Preliminary direction

- Keep compile-time layer dependencies strictly `upper -> middle -> lower`.
- Build deployment units as vertical runtime cells when co-location is beneficial; process boundaries do not need to mirror compile-time layers.
- Isolate device I/O, realtime control, middleware/network, vision inference, logging, and FSM work by thread or process according to blocking and failure behavior.
- Use system service supervision for process lifecycle and watchdog integration; keep the robot FSM responsible for mission/safety state rather than duplicating the operating system's process manager.
- Treat control-loop frequency and process placement as measured target-board properties with explicit deadlines and overload tests.
