# Orange Pi 5 Plus bring-up

## Electrical and recovery prerequisites

- active cooling and a recorded stable power supply;
- verified serial-console access and a retained vendor-kernel boot entry;
- a 3.3 V logic-compatible **isolated** CAN transceiver;
- 120 ohm termination at exactly the two physical CAN-bus endpoints;
- a CAN analyzer or non-motor loopback peer for this phase;
- robot motion power removed and no motor connected.

The RK3588 header signals are controller-side TX/RX logic. They are not CANH and
CANL and must never be wired directly to a CAN bus.

## Device-tree choice

The supported official overlay choices are:

| Interface | Overlay | Header TX | Header RX |
|---|---|---:|---:|
| CAN0 | `rk3588-can0-m0` | physical pin 5 | physical pin 3 |
| CAN1 | `rk3588-can1-m0` | physical pin 18 | physical pin 16 |

The checked-in CAN contract selects CAN0. Enable exactly the matching overlay by
the vendor boot-environment mechanism, retain the prior boot entry, reboot and
verify the active device tree and interface. Do not guess pinmux from interface
names.

## Read-only discovery

```bash
ip -details -statistics link show can0
platform/orangepi5plus/scripts/rt-affinity.sh snapshot
platform/orangepi5plus/scripts/rt-affinity.sh plan
platform/orangepi5plus/scripts/can-link.sh plan
```

Confirm the target reports eight expected logical CPUs, CPU capacity/frequency
topology, PREEMPT_RT, a resolvable CAN IRQ and a writable affinity file. The
checked-in `0-5 / 6 / 7` role map is only a candidate until these observations
and timing evidence agree.

## Candidate-only runtime preparation

Save the snapshot first. While motion is physically disarmed, a target operator
may apply the candidate profile manually:

```bash
sudo platform/orangepi5plus/scripts/rt-affinity.sh apply --confirm-disarmed
platform/orangepi5plus/scripts/rt-affinity.sh check
```

The apply path records its original scheduler, governor and CAN IRQ values and
attempts to roll them back if any write or readback fails. Keep motion disarmed
if rollback reports an error. Production startup additionally passes
`--require-qualified` and rejects this candidate profile.

## Classic SocketCAN link

The fixed contract is classic CAN at 1,000,000 bit/s, bus-error reporting on and
`restart-ms 0`:

```bash
sudo platform/orangepi5plus/scripts/can-link.sh up
platform/orangepi5plus/scripts/can-link.sh status
platform/orangepi5plus/scripts/can-link.sh check
```

`up` is idempotent only when the existing link already matches. Reconfiguring an
active mismatched link, cycling bus-off state or taking the link down requires an
explicit disarmed acknowledgement:

```bash
sudo platform/orangepi5plus/scripts/can-link.sh recover --confirm-disarmed
sudo platform/orangepi5plus/scripts/can-link.sh down --confirm-disarmed
```

Automatic bus-off recovery remains disabled. This phase validates traffic only
against the analyzer/loopback peer; the platform tools contain no CAN transmit
or motor operation.

## Privileged service boundary

Install the files under `systemd/` only after a profile has real evidence and is
marked qualified. `rs-control-platform-ready.target` requires both privileged
oneshot services to succeed. Application and FSM units depend on that target but
do not receive `CAP_SYS_ADMIN` or `CAP_NET_ADMIN`.

The example motion unit illustrates limits and ordering only. The future Rust
runtime must still set and read back `mlockall`, `SCHED_FIFO` and exact per-thread
affinity before reporting ready; systemd `CPUAffinity` alone is not sufficient.
