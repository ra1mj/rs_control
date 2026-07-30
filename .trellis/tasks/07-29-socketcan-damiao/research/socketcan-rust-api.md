# Research: safe Rust SocketCAN API for the RK3588 realtime path

- Query: Select the lowest-overhead safe Rust SocketCAN boundary that supports
  classic CAN, non-blocking I/O, filters, error frames and RX timestamps without
  weakening the workspace `unsafe_code = "forbid"` policy.
- Scope: external Linux/Rust API research plus repository-local constraints
- Date: 2026-07-29

## Recommendation

Use the synchronous API from exactly pinned `socketcan` 3.6.2 with default
features disabled:

```toml
socketcan = { version = "=3.6.2", default-features = false }
```

The selected release is tag/commit
`ac9070e1d408c4152ea2cef43305e8e00bbfc614`, is MIT licensed, declares Rust
1.75 as its minimum version and has crates.io checksum
`ea24328be148eea05f36bc3991b1c174c586853e04d1480b92429b2df6aea865`.
The workspace Rust 1.97 toolchain satisfies that requirement. Cargo's exact
version plus lockfile checksum is the dependency pin; implementation source must
not be copied into this repository.

The crate keeps Linux ABI `unsafe` inside the external dependency while the
project adapter remains safe Rust. Direct `libc` would require local unsafe
socket, `sockaddr_can`, `setsockopt`, `recvmsg`, cmsg alignment and file
descriptor ownership code. `nix` 0.31.3 does not expose a public `SockaddrCan`
or ready-made `CAN_RAW_FILTER` / `CAN_RAW_ERR_FILTER` options, so a direct nix
adapter still falls back to libc and local unsafe.

## Realtime findings

- `read_frame_with_timestamps()` performs one `recvmsg`, uses a stack-resident
  aligned 256-byte control-message buffer and skips unknown cmsgs. Its normal RX
  path has no explicit heap allocation.
- `write_frame()` has no explicit normal-path heap allocation.
- `set_filters()` constructs a temporary `Vec`; call it only during disarmed
  initialization, never in the realtime loop.
- Do not use `write_frame_insist()`: it retries indefinitely and violates the
  non-blocking transport contract.
- Disable async/Tokio features. One fixed CAN-I/O owner uses one socket; multiple
  concurrent readers would split frames nondeterministically.
- RX drains at most a configured fixed number of frames per invocation.
  `WouldBlock` means the non-blocking queue is empty, not a link failure.
- TX makes one attempt. `WouldBlock`, `ENOBUFS`, partial acceptance and other
  errors become explicit typed backpressure/fault outcomes and counters.

## Initialization contract

The unprivileged adapter opens a link already configured by the platform
service. During the disarmed startup phase it:

1. opens the explicitly selected interface;
2. enables close-on-exec and non-blocking operation;
3. installs discovery-stage standard-frame filters, the bounded error mask and
   timestamp options;
4. drains frames that may have arrived between bind and filter installation;
5. performs serialized device discovery;
6. installs final exact feedback-ID filters before handing the socket to the
   fixed CAN-I/O thread.

The adapter never changes bitrate, restarts a controller or brings the link
up/down. Those operations remain in the privileged Orange Pi platform service.

For one exact standard data-frame ID, the filter mask includes
`CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG`; this excludes extended and RTR
frames with matching low identifier bits and enables the Linux single-ID
receive-list optimization.

## Error-frame contract

Raw CAN sockets do not receive error frames by default. Subscribe only to the
safety-relevant set on the control socket:

```text
TX_TIMEOUT | CRTL | TRX | ACK | BUSOFF | RESTARTED
```

Do not subscribe the realtime control socket to the full `CAN_ERR_MASK` because
the Linux UAPI warns that `CAN_ERR_BUSERROR` can flood the receive path. Full
physical diagnostics, if later required, use a separate rate-limited socket on
a housekeeping CPU.

Linux error-class values are a bitmask. `socketcan::CanError::from()` matches
single values and can turn a multi-bit frame into `Unknown`; the safety path
must preserve `error_bits()` and test flags such as `CAN_ERR_BUSOFF` with a bit
operation. A BUS-OFF bit combined with any other bit still immediately faults
the hardware path. With platform `restart-ms=0`, recovery stays explicit and no
driver automatically re-enables a motor.

## Timestamp contract

Kernel RX timestamps are diagnostic clock-domain data. The adapter also records
the local process monotonic time immediately after each successful receive;
only that compatible monotonic domain participates in feedback freshness.

- `SO_TIMESTAMPNS` is wall-clock based.
- `SO_TIMESTAMPING_RX_SOFTWARE | SOFTWARE | OPT_CMSG` can expose an earlier
  kernel software timestamp.
- Hardware timestamps require driver/adapter support plus a measured conversion
  from the PHC/raw clock domain and cannot be inferred from an API capability.
- The selected crate does not provide the complete Linux TX-error-queue
  timestamp flow; TX timestamps are not required for this motor-control slice.

## Host-test boundary

Default tests exercise pure frame conversion, filter construction, error-mask
classification, health counters, bounded drain/TX behavior and bus budgeting
through a fake backend. They never open `can0` or create/delete an interface.

An optional process test may use a caller-provided `vcan` interface only when an
explicit environment variable names a nonphysical interface. The test remains
ignored by default and does not configure kernel networking.

## Sources

- Linux v5.10 SocketCAN documentation:
  <https://github.com/torvalds/linux/blob/v5.10/Documentation/networking/can.rst>
- Pinned Orange Pi kernel SocketCAN documentation:
  <https://github.com/orangepi-xunlong/linux-orangepi/blob/9f9e9d18574d0914c0d192a90c3babfe1fd63c95/Documentation/networking/can.rst>
- Linux v5.10 raw-CAN UAPI:
  <https://github.com/torvalds/linux/blob/v5.10/include/uapi/linux/can/raw.h>
- Linux v5.10 CAN error UAPI:
  <https://github.com/torvalds/linux/blob/v5.10/include/uapi/linux/can/error.h>
- Linux timestamping documentation:
  <https://www.kernel.org/doc/html/latest/networking/timestamping.html>
- `socketcan` 3.6.2 release documentation:
  <https://docs.rs/crate/socketcan/3.6.2>
- Pinned socket implementation:
  <https://github.com/socketcan-rs/socketcan-rs/blob/v3.6.2/src/socket.rs>
- Pinned error decoder:
  <https://github.com/socketcan-rs/socketcan-rs/blob/v3.6.2/src/errors.rs>
- `nix` 0.31.3 socket implementation used for comparison:
  <https://github.com/nix-rust/nix/blob/v0.31.3/src/sys/socket/mod.rs>

## Caveats

- Actual allocation behavior, syscall duration, receive overrun metadata and
  timestamp support must still be measured on the pinned RK3588 kernel.
- Hardware error-frame ordering and bus-off recovery require a CAN analyzer and
  physically safe target fixture; host tests cannot qualify them.
- External dependency review must be repeated before upgrading the exact
  `socketcan` version.
