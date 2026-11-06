# Experimental CAN-only clock synchronization

This optional feature maps independently sampled IMU data onto one node's MCU
clock. It does not trigger simultaneous MEMS conversions or provide UTC. No
additional board connections are required: it uses CAN FD and the existing
ICM INT1-to-PB0 connection. Hardware timing accuracy has not been validated.

## Build and activation

The CMake build introduced in PR #2 is a prerequisite for the commands below:

```sh
cmake -S . -B build/sync -G Ninja -DCMAKE_BUILD_TYPE=Release -DOPENIMU_CAN_SYNC=ON -DOPENIMU_NODE_ID=1
cmake --build build/sync
```

`OPENIMU_CAN_SYNC` defaults to OFF. Assign unique node IDs 1..5 when using multiple
boards. For STM32CubeIDE, add `OPENIMU_CAN_SYNC=1` and `OPENIMU_NODE_ID=1` to the C
compiler preprocessor symbols of each configuration you intend to build; omit
the sync symbol or set it to 0 for the original firmware. Use numeric 1/0 in
CubeIDE, not CMake's ON/OFF strings. The new C sources are under the existing
Core source entry. CubeIDE itself has not been exercised for this feature.

New IRQ code and optional filter allocation live in existing USER CODE blocks.
Filter allocation reinitializes FDCAN while it is still stopped. Existing upstream
CubeMX regeneration caveats for SPI/CAN settings still apply; this feature does
not regenerate or repair those unrelated settings.

An enabled build initially sends the original 16-byte samples. Only a valid
START activates the 24-byte extension. A matching STOP or 1000 ms without a fresh
valid control returns to 16 bytes. Already queued frames can remain in flight.
Timers and capture monitoring run in the background in enabled builds, including
before START. A disabled build removes this machinery.

## Host usage

Install `python-can` and configure a CAN FD adapter for 1 Mbit/s nominal and
5 Mbit/s data. The example transport is SocketCAN:

```sh
python3 tools/clock_sync.py --interface socketcan --channel can0 --reference 1
```

The CLI sends controls every 100 ms and requests STOP on Ctrl-C. JSON Lines retain
raw values and distinguish valid common time from unavailable time (`null` with a
reason). The transport-independent `ClockMapper` can be reused by another bridge.
For a separate control sender, use `--listen-only --session 0x1234`; that sender
must implement the control protocol below. C5/MQTT and ROS integration are outside
this patch. The existing dashboards still select ID 0x01 by default; this feature
does not turn them into multi-node dashboards.

## Wire protocol version 1

All messages are standard-ID CAN FD data frames with BRS. Integers are big-endian.

| Message | CAN ID | Bytes | Fields in order |
|---|---|---:|---|
| Control | 0x700 | 16 | `"OSYN"[4], version:u8, opcode:u8, reserved:u16, session:u32, sequence:u32` |
| Sample | node ID 1..5 | 16 or 24 | Official 16 bytes; when active, append `source_time_low32:u32, epoch:u32` |
| Anchor | 0x710 + node ID | 48 | `version:u8, flags:u8, node:u8, reserved:u8, session:u32, sequence:u32, epoch:u32, sync_local_us:u64, sample_local_us:u64, sample_source_us:u64, capture_overrun:u32, invalid_samples:u32` |

Version is 1 and reserved fields are zero. Session is nonzero. Opcodes are START=1,
TICK=2, STOP=3. Active sessions reject a different session. Sequences must advance
within the forward half of the modulo-2^32 space; duplicates do not renew leases.
Only START activates an inactive node. Periodic fresh STARTs allow late joining.

Anchor flags: bit 0 means the CAN event is valid; bit 1 means the source/capture
pair is valid and requires bit 0. A flags=0 anchor revokes the identified group.
Observed conflicting retransmissions or RX overflow revoke uncertain events and
advance the source epoch. Revocation retries are bounded by the session lease;
queued transmission is not proof of bus delivery. A new DATA epoch also invalidates
old mappings. Revocation cannot retract timestamps already consumed downstream.

The first 16 sample bytes retain their official meaning and CAN ID. The supplied
legacy decoders accept at least 16 bytes and ignore the extension. Other receivers
that require exactly 16 bytes must be checked separately. Local source time
remains meaningful before a common-time fit is available; it is never replaced
by a fabricated common time of zero.

## Time mapping and boundaries

TIM3 runs at 1 MHz. FDCAN captures its low 16 bits at RX SOF; TIM3_CH3 captures PB0.
The host fits source-to-local and local-to-reference affine mappings. It requires
at least eight suitable points and a minimum time span before publishing common
time. Models expire, reject implausible points and preserve per-epoch monotonicity.
Reference epoch changes start a new common timeline. No automatic reference
selection or on-device clock steering is provided.

FIFO backlog, failed reads, reused/overrun/old captures, unknown rollover and
ambiguous CAN events invalidate associated timing. Sample data is enqueued before
its optional anchor, matching the controller's TX FIFO ordering. Existing anchors
already in flight can still delay later data; this is not a hard latency guarantee.
The upstream disabled-auto-retransmission policy remains in force, so lossless
multi-node delivery is not claimed.

DWT cross-checks timer service delays. Clock faults disable synchronization until
reboot. Dynamic clock changes, debugger halts and unbounded interrupt masking are
unsupported during measurement. A stalled sensor also stops the acquisition-task
polling used for controls; it cannot remain an independent reference service.

See [validation and outstanding hardware acceptance](testing-can-clock-sync.md).
