# EXAMPLE_CODE — fixes from the 2026-04-26 pipeline review

This document tracks the resolutions applied to the code-review findings in
[`AI NOTES/CODE REVIEWS/2026-04-26_v25_controller_pipeline_example_code_review.md`](../../../AI%20NOTES/CODE%20REVIEWS/2026-04-26_v25_controller_pipeline_example_code_review.md).

All thirteen findings were verified against the actual files and confirmed
valid. The fixes below are now in-tree. The code is still **draft for review
only** — none of it has been compiled or flashed — but the pipeline is now
internally consistent and matches the canonical specs in
[`TRACTOR_NODE.md`](../../TRACTOR_NODE.md) and
[`LORA_PROTOCOL.md`](../../LORA_PROTOCOL.md).

---

## Critical

### 1. `ControlFrame` is now genuinely 16 bytes

**Was:** the packed C struct totalled 15 bytes while every doc/comment/airtime
calculation said 16.

**Fix:** added an explicit `uint8_t reserved` field between `heartbeat_ctr`
and `crc16` in [`lora_proto/lora_proto.h`](lora_proto/lora_proto.h). The
field is documented as "MUST be 0 on TX, ignored on RX" so we can repurpose
it later without bumping `LIFETRAC_PROTO_VERSION`. The C packer
([`lora_proto/lora_proto.c`](lora_proto/lora_proto.c)) and the Python packer
([`base_station/web_ui.py`](base_station/web_ui.py) → `pack_control`) both
stamp it to zero. Layout matches doc table in
[`LORA_PROTOCOL.md § ControlFrame`](../../LORA_PROTOCOL.md#controlframe-16-bytes-total-5-header--11-payload).

### 2. Handheld nonce is no longer one sequence ahead of the header

**Was:** `lp_make_control(..., g_seq++, ...)` then `build_nonce(nonce, g_seq)`
read the post-incremented value, so the nonce was bound to a sequence one
ahead of the one inside the encrypted frame. AEAD verification would always
fail at the receiver.

**Fix:** [`handheld_mkr/handheld.ino`](handheld_mkr/handheld.ino) `loop()`
captures the sequence number into a local (`cf_seq`, `hb_seq`) and threads
it into both `lp_make_control` and `send_frame`. `send_frame` now takes the
sequence as an explicit parameter — comment on the function spells out why.

### 3. M7 now feeds the M4 watchdog

**Was:** `tractor_m4.cpp` reads `SHARED->alive_tick_ms` and trips the PSR
relay if it hasn't moved in 200 ms. `tractor_m7.ino` never wrote it, so the
watchdog would fire immediately after the 500 ms grace period.

**Fix:** [`tractor_h7/tractor_m7.ino`](tractor_h7/tractor_m7.ino) declares
the `SharedM7M4` struct with the same layout as the M4 side and stamps
`SHARED->alive_tick_ms = millis()` as the **first** statement in `loop()`.

### 4. Opta watchdog is no longer reset by ordinary Modbus polling

**Was:** the slave's main loop unconditionally called
`on_holding_write(REG_LAST_ERROR, ModbusRTUServer.holdingRegisterRead(...))`
on every `poll()`. Since the register defaulted to 0, that call took the
"clear faults" branch and updated `g_last_alive_change_ms` to "now" — meaning
any Modbus traffic kept the watchdog alive even when the master had stopped
ticking the alive register.

**Fix:** [`opta_modbus_slave/opta_modbus_slave.ino`](opta_modbus_slave/opta_modbus_slave.ino)
now keeps a `s_last_seen[]` snapshot of every writable register and only
invokes side effects when a register **transitions**. The fault-clear path
was deleted entirely; clearing latched faults is now an out-of-band action
(future: `CMD_CLEAR_ESTOP` from the base over the air). The watchdog's
freshness check lives only in `check_safety()`, which is independent of the
Modbus poll loop.

---

## High

### 5. Modbus register map realigned to `TRACTOR_NODE.md`

**Was:** the example used a fictional map (slave id `0x10`, individual valve
registers `0x0010..0x0017`, alive tick at `0x0000`). The spec
([`TRACTOR_NODE.md`](../../TRACTOR_NODE.md) §"Modbus RTU register map") is
slave id `0x01`, a `valve_coils` bitfield at `0x0000`, watchdog at `0x0004`,
and an input-register telemetry block at `0x0100..0x010B`.

**Fix:** the spec is the source of truth. Both
[`tractor_h7/tractor_m7.ino`](tractor_h7/tractor_m7.ino) and
[`opta_modbus_slave/opta_modbus_slave.ino`](opta_modbus_slave/opta_modbus_slave.ino)
now use:

| Type | Address | Field |
|---|---|---|
| Holding | `0x0000` | `valve_coils` (8-bit bitfield) |
| Holding | `0x0001` | `flow_setpoint_1` |
| Holding | `0x0002` | `flow_setpoint_2` |
| Holding | `0x0003` | `aux_outputs` |
| Holding | `0x0004` | `watchdog_counter` |
| Holding | `0x0005` | `command_source` |
| Holding | `0x0006` | `arm_engine_kill` |
| Input   | `0x0100..0x010B` | safety_state, digital_inputs, analog block, faults, uptime, fw_version |

Master writes the full 7-register holding block in a single
`WriteMultipleRegisters` call from `apply_control()`. Telemetry is now read
via `INPUT_REGISTERS` (function 0x04) — `emit_telemetry()` was updated.

### 6. Stale ControlFrames no longer drive valves

**Was:** `pick_active_source()` only checked `last_heartbeat_ms`. A source
whose heartbeat was fresh but whose ControlFrame was stale (e.g. one TX path
broken at 50 Hz and the other at 20 Hz) would keep replaying the last good
joystick position. Worse, `process_air_frame` `memcpy`'d the new frame into
`g_src[idx].latest` before checking CRC, so a corrupted frame could overwrite
the last good one.

**Fix:** in [`tractor_h7/tractor_m7.ino`](tractor_h7/tractor_m7.ino):
- `pick_active_source()` now requires both heartbeat freshness
  (`HEARTBEAT_TIMEOUT_MS = 500`) **and** control freshness
  (`CONTROL_TIMEOUT_MS = 200`).
- `process_air_frame()` validates CRC against a stack-local copy and only
  swaps it into `g_src[idx].latest` if CRC passes.

### 7. `CMD_ESTOP` is wired up

**Was:** the `case FT_COMMAND` in `process_air_frame` was a TODO comment.

**Fix:** [`tractor_h7/tractor_m7.ino`](tractor_h7/tractor_m7.ino) defines a
`CmdOp` enum and handles `CMD_ESTOP`, `CMD_CLEAR_ESTOP`, and `CMD_REKEY`.
E-stop sets a global `g_estop_latched`, immediately calls `apply_control(-1)`,
and forces `REG_ARM_ESTOP=1` on every subsequent Modbus write until cleared.
Only `SRC_BASE` may clear an E-stop (`CMD_CLEAR_ESTOP`). The bridge
([`base_station/lora_bridge.py`](base_station/lora_bridge.py)) now builds a
properly-CRC'd FT_COMMAND payload for the `lifetrac/v25/cmd/estop` MQTT
topic.

### 8. `all_coils_off()` actually drops everything

**Was:** the function only handled the four onboard relays. The D1608S
expansion (relays R5–R8 = boom + bucket) and the A0602 0–10 V outputs were
TODO comments — i.e. an E-stop or watchdog trip would leave the implement
moving.

**Fix:** [`opta_modbus_slave/opta_modbus_slave.ino`](opta_modbus_slave/opta_modbus_slave.ino)
introduces `d1608s_set()` and `a0602_write_mv()` shims (clearly marked as
expansion-bus placeholders to swap for real `OptaController` calls during
hardware bring-up) and `all_coils_off()` now drops all 8 valve coils and
both flow set-points. `apply_valve_coils()` writes the full bitfield into
all 8 channels.

---

## Medium

### 9. Telemetry nonce is no longer mostly zeros

**Was:** `nonce[12] = { SRC_TRACTOR, 0,0, 0,0,0,0, 0,0,0,0,0 }`, then bytes
[1..2] overwritten with the sequence. The remaining 9 bytes were
deterministic — a sequence rollover or reboot would reuse a nonce, which
breaks AES-GCM confidentiality.

**Fix:** [`tractor_h7/tractor_m7.ino`](tractor_h7/tractor_m7.ino) gains a
`build_nonce(out, source_id, seq)` helper of the same shape as the handheld:
`[ source_id | seq(LE) | t_seconds(LE) | rand(5) ]`.

### 10. Bridge now validates frame length and CRC before publishing

**Was:** [`base_station/lora_bridge.py`](base_station/lora_bridge.py)
`_handle_air()` sliced telemetry by `payload_len` without bounds checks and
never verified the on-air CRC. `_on_mqtt_message()` accepted arbitrary-length
payloads from the LAN side and TX'd them as-is.

**Fix:**
- Added `crc16_ccitt()` and `verify_crc()` helpers (matching
  [`lora_proto.c`](lora_proto/lora_proto.c)).
- `_handle_air()` checks frame length per type, then CRC, then publishes.
- `_on_mqtt_message()` requires LAN-side `cmd/control` payloads to be
  exactly `CTRL_FRAME_LEN` bytes with a valid CRC.
- `web_ui.py` `pack_control` clamps every joystick axis to int8 and masks
  buttons/flags to their declared widths.

### 11. Gamepad START button is edge-detected

**Was:** the 50 Hz gamepad poll fired `fetch('/api/estop')` every 20 ms while
the operator held START — a low-grade DoS on the local FastAPI server.

**Fix:** [`base_station/web/app.js`](base_station/web/app.js) maintains a
`prevButtons` map and a `pressed(gp, idx)` helper that returns `true` only
on the rising edge. E-stop fires once per press. Axis values are clamped to
`[-127, +127]` to match the int8 wire format.

### 12. Crypto stub is now opt-in via build guard

**Was:** [`crypto_stub.c`](lora_proto/crypto_stub.c) silently produced a
no-op AES that copied plaintext through. The Python bridge uses real
AES-GCM. A naive build would link the stub on the C side and produce a
"working" link with zero confidentiality — and would fail AEAD verification
against the real Python peer in subtle, intermittent ways.

**Fix:** added `#error` guard requiring the developer to define
`LIFETRAC_ALLOW_STUB_CRYPTO` to acknowledge the insecure build. Real
production builds must link MbedTLS (Portenta) or ArduinoBearSSL (MKR).

---

## Low

### 13. Handheld deadband + button debounce; gamepad axis clamp

**Was:** raw ADC reads on the handheld passed through to the wire — neutral
was rarely truly zero — and digital inputs had no debounce. The web gamepad
path could in principle emit out-of-range axis values.

**Fix:**
- [`handheld_mkr/handheld.ino`](handheld_mkr/handheld.ino) adds a 16-count
  per-axis deadband with stretched live-region scaling, and a 15 ms
  shift-register button debounce.
- [`base_station/web/app.js`](base_station/web/app.js) clamps each axis
  with `Math.max(-127, Math.min(127, ...))`.

---

## Not addressed in this pass

- Per-coil overcurrent monitoring (was `read_current()` placeholder; left
  out of the canonical `TRACTOR_NODE.md` register map; can land later as an
  optional input-register block).
- Boot self-test for the D1608S/A0602 expansion modules — the onboard
  relays still get the test pulse, but the expansion shims are placeholders
  so there's nothing to actuate yet.
- `CMD_REKEY` handling — out of scope for v25; key provisioning stays USB.
- Differential-turn drive math (`tractor_m7.ino` `apply_control` still drives
  both sides identically from `axis_lh_y`). Carry-over from
  [`RESEARCH-CONTROLLER/arduino_opta_controller/`](../arduino_opta_controller/)
  remains TODO.

---

## Follow-up additions (post-review, same session)

These aren't review findings — they're new code added on top of the reviewed
pipeline as the hardware design solidified. Listed here so the pipeline doc
and the code stay in step.

### A. Tractor X8-side IMU service (`tractor_x8/imu_service.py`)

Adds a Linux-side service that reads the **SparkFun BNO086 Qwiic** IMU
through an **Adafruit MCP2221A USB→I²C bridge** (Adafruit 4471) and forwards
18-byte fused samples (Q14 quaternion + linear-accel mg + heading×10) to the
M7 over the X8↔H747 UART at 5 Hz. The M7 will wrap each sample in a
`TelemetryFrame` with `topic_id = 0x07`. Rationale and the native-I²C
fallback are documented in
[`HARDWARE_BOM.md` § Notes on substitutions — IMU path](../../HARDWARE_BOM.md#notes-on-substitutions)
and in the file's module docstring.

### A2. Tractor X8-side GPS service (`tractor_x8/gps_service.py`)

Companion service that reads the **SparkFun NEO-M9N SMA Qwiic** receiver on
the *same* Qwiic bus (chained off the IMU through one MCP2221A) and forwards
21-byte fixed-point samples (lat/lon/alt e7+cm, speed cm/s, heading×10,
fix-type, sats, HDOP) to the M7 at **1 Hz active / 0.2 Hz when parked**. The
M7 emits each one as a `TelemetryFrame` with `topic_id = 0x01`. 1 Hz is the
sweet spot for a tractor doing ≤5 mph: position uncertainty (~2.2 m
between fixes) is comparable to the receiver's standalone accuracy.

### A3. Common X8→M7 UART pump (`tractor_h7/tractor_m7.ino`)

The formerly-`emit_telemetry`-only path has been split. `emit_topic(topic_id,
payload, len)` is now the canonical encryptor + LoRa transmitter for any
(topic, payload) tuple; `emit_telemetry()` calls it with the Opta hydraulics
block at 1 Hz. A new `poll_x8_uart()` loop runs every iteration of `loop()`,
KISS-decodes packets coming in from `gps_service.py` / `imu_service.py` on
`Serial1` (X8↔H747 UART, 921600 8N1), peels the leading topic byte, and
hands the rest to `emit_topic`. Standalone-H7 builds simply see no bytes on
`Serial1` and skip the pump — the rest of the firmware is unchanged.

### B. Two new MQTT topic IDs in the bridge

[`base_station/lora_bridge.py`](base_station/lora_bridge.py) `TOPIC_BY_ID`
gained:

- `0x07 → lifetrac/v25/telemetry/imu` (BNO086 quaternion + accel)
- `0x08 → lifetrac/v25/telemetry/sensor_faults` (sensor disconnect / CRC bitmap)

Unknown topic IDs continue to fall back to `lifetrac/v25/raw/<id>` so adding
more is non-breaking.

