# Rollback and evidence retention

## Runtime tuning failure

Keep motion physically disarmed. `rt-affinity.sh apply` attempts an immediate
transactional rollback to the scheduler budget, governors and selected CAN IRQ
affinity captured before its first write. If it reports an incomplete rollback,
restore those values from the pre-apply `snapshot` through the serial console,
then run `snapshot` and `check` again. Do not start application services merely
because some individual values look correct.

The qualification wrapper stops only the monitor and pressure processes whose
PIDs it created. It does not kill unrelated processes or delete the evidence
directory.

## CAN failure or bus-off

No automatic restart is configured. Revoke all future actuator authority, keep
the current phase's analyzer-only bus disconnected as needed, inspect link/error
counters and wiring, then perform explicit recovery only while disarmed:

```bash
sudo platform/orangepi5plus/scripts/can-link.sh recover --confirm-disarmed
```

## Kernel boot failure

Use the previously verified serial console and select the retained vendor kernel
and device tree. Do not purge the failed packages or overwrite the only working
boot entry during diagnosis. After booting the vendor kernel:

1. record `uname -a`, the selected boot entry and device-tree identity;
2. preserve the failed build's `SHA256SUMS` and build metadata;
3. preserve every partial/failed qualification directory;
4. restore the previous runtime/CAN configuration bundle;
5. verify CAN remains down or analyzer-only before enabling any later service.

Rollback never reuses a passing evidence hash for a different kernel, DT,
profile, board revision or build. Any such change requires a new unique evidence
directory and a complete qualification run.
