# Code Review: v25 Controller EXAMPLE_CODE Pipeline

Date: 2026-04-26  
Reviewer: GitHub Copilot  
Scope: `LifeTrac-v25/DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/`

This review covers the first-draft example pipeline:

- `lora_proto/` shared frame structs, CRC, KISS framing, and crypto stub
- `handheld_mkr/handheld.ino`
- `tractor_h7/tractor_m7.ino` and `tractor_h7/tractor_m4.cpp`
- `opta_modbus_slave/opta_modbus_slave.ino`
- `base_station/lora_bridge.py`, `web_ui.py`, and `web/`

No hardware compile, Arduino CLI compile, or Python runtime test was completed. VS Code diagnostics reported no editor-visible errors in the files checked. The Python environment tool could not find a base Python interpreter in this workspace, so the Python files were reviewed statically.

## Findings

### Critical: ControlFrame size is documented as 16 bytes but implemented as 15

Locations:

- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/lora_proto/lora_proto.h`, line 76
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/web_ui.py`, lines 63-73
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/lora_bridge.py`, line 180

The protocol and comments repeatedly say `ControlFrame` is 16 bytes, but the packed fields add up to 15 bytes:

- Header: 5 bytes
- Four int8 axes: 4 bytes
- Buttons: 2 bytes
- Flags: 1 byte
- Heartbeat counter: 1 byte
- CRC16: 2 bytes
- Total: 15 bytes

Python mirrors the same 15-byte format with `struct.pack("<BBBHbbbbHBB", ...)` plus a 2-byte CRC. This is internally consistent between C and Python, but inconsistent with the spec, README language, and comments. That will cause bad assumptions in airtime estimates, validation checks, test fixtures, and any future decoder that hard-codes 16.

Recommendation: decide whether the frame is really 15 or 16 bytes. If 15 is acceptable, update `LORA_PROTOCOL.md`, the README, comments, and bridge checks. If 16 is desired, add an explicit reserved byte before the CRC and update both C and Python packers.

### Critical: Handheld AES-GCM nonce sequence does not match the frame sequence

Locations:

- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/handheld_mkr/handheld.ino`, lines 73-75
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/handheld_mkr/handheld.ino`, lines 137-143

`lp_make_control()` and `lp_make_heartbeat()` receive `g_seq++`, then `send_frame()` calls `build_nonce(nonce, g_seq)`. That means the AES-GCM nonce sequence is one count ahead of the sequence embedded in the encrypted frame header.

This weakens replay/audit assumptions and will make cross-node debugging painful once the tractor starts checking nonce/header consistency. It also breaks the documented nonce shape: `[source_id | sequence_num | timestamp | random]`.

Recommendation: reserve the sequence in a local variable, put the same value in the header and nonce, then increment once after transmission. For example: `uint16_t seq = g_seq++; lp_make_control(..., seq, ...); send_frame(..., seq);`.

### Critical: M4 watchdog will trip because the M7 never writes the shared alive tick

Locations:

- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_h7/tractor_m4.cpp`, lines 19-26 and 43-47
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_h7/tractor_m7.ino`, lines 133-135 and 275-280

The M4 expects `SHARED->alive_tick_ms` to be updated by the M7 and latches the safety GPIO low if it is older than 200 ms. The M7 code only ticks the Opta Modbus register and never writes the shared SRAM field.

On real hardware, the M4 would likely drop the PSR safety relay shortly after boot, even while M7 is healthy. The M4 also does not currently detect a wedged Modbus link; it only detects the absence of the M7 shared-memory heartbeat.

Recommendation: add the matching shared-memory struct to M7 and write `millis()` every loop or arbitration tick. Also decide whether M4 should monitor only M7 liveness or a richer state such as last successful Modbus write.

### Critical: Opta watchdog can be accidentally refreshed by the error-clear scan

Locations:

- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/opta_modbus_slave/opta_modbus_slave.ino`, lines 113-139
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/opta_modbus_slave/opta_modbus_slave.ino`, lines 181-187 and 203

The loop scans `REG_LAST_ERROR` on every successful Modbus poll and calls `on_holding_write(REG_LAST_ERROR, current_value)`. When that value is `0`, `on_holding_write()` clears errors and updates `g_last_alive_change_ms = millis()`.

As a result, ordinary Modbus traffic can keep the watchdog fresh even if `REG_ALIVE_TICK` is not changing. A master that is still polling or writing unrelated registers could prevent the watchdog from tripping, which defeats the intended safety behavior.

Recommendation: only update `g_last_alive_change_ms` when `REG_ALIVE_TICK` changes. Treat `REG_LAST_ERROR=0` as a real write-to-clear command, not as a scanned side effect. Prefer explicit Modbus write callbacks or track previous register values so unchanged registers do not retrigger side effects.

### High: The example Modbus register map is out of sync with `TRACTOR_NODE.md` and `TODO.md`

Locations:

- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_h7/tractor_m7.ino`, lines 32-40
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/opta_modbus_slave/opta_modbus_slave.ino`, lines 23-34
- `DESIGN-CONTROLLER/TRACTOR_NODE.md`, lines 130-146 and 162-168
- `DESIGN-CONTROLLER/TODO.md`, lines 194-199

The example uses slave ID `0x10`, `REG_ALIVE_TICK = 0x0000`, and eight separate valve registers at `0x0010..0x0017`. The tractor node design says the Opta slave is address `0x01`, register `0x0000` is a `valve_coils` bitfield, register `0x0004` is `watchdog_counter`, and telemetry should be input registers at `0x0100..0x010B`.

This is an integration blocker because the master, slave, tests, and documentation will not agree on the first bench bring-up.

Recommendation: choose one canonical register map now. The newer `TRACTOR_NODE.md`/`TODO.md` map looks better for bus efficiency because it writes the complete control block at 50 Hz, so the example code should probably be updated to that map.

### High: Tractor can continue applying stale or invalid control frames

Locations:

- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_h7/tractor_m7.ino`, lines 78-95
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_h7/tractor_m7.ino`, lines 154-170
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_h7/tractor_m7.ino`, line 275

`pick_active_source()` only checks heartbeat freshness. It does not require a fresh valid `ControlFrame`. If heartbeats continue but control frames stop, the tractor applies the last stored command indefinitely. Also, the code copies the incoming `ControlFrame` into `latest` before CRC validation and only clears `last_control_ms` on CRC failure; `apply_control()` does not check `last_control_ms`.

Recommendation: verify CRC before copying into `latest`, update `last_control_ms` only after a valid frame, and make `apply_control()` neutralize if the selected source has no valid control frame newer than a short timeout, for example 150-250 ms. Consider separate heartbeat and control sequence windows.

### High: E-stop is not wired through the tractor control path yet

Locations:

- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/web/app.js`, lines 138 and 171-172
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/web_ui.py`, lines 141-144
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/lora_bridge.py`, lines 180-185
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_h7/tractor_m7.ino`, lines 185-187

The UI and bridge send an E-stop command, but the tractor M7 command handler is still a TODO. In the current draft, a web or gamepad E-stop would be transmitted and then ignored by the tractor logic.

Recommendation: before any bench test that presents an E-stop button, implement `CMD_ESTOP` handling as a latched fault that immediately writes neutral/off to Opta, stops watchdog progression if that is the desired safety behavior, and requires an explicit reset path.

### High: Safety-off paths do not actually zero all outputs yet

Locations:

- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/opta_modbus_slave/opta_modbus_slave.ino`, lines 59-66
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/opta_modbus_slave/opta_modbus_slave.ino`, lines 68-78

`all_coils_off()` clears only the four onboard relay pins. The D1608S outputs and A0602 analog outputs are TODOs. `apply_valve_register()` also only implements channels 0-3. That means the draft does not yet satisfy its own safety comment: watchdog or E-stop should drop all eight coils and zero both flow outputs.

Recommendation: either implement all eight relay outputs and both analog outputs before bench movement tests, or explicitly label the current Opta sketch as a four-relay placeholder that must only be used with LEDs on the onboard relays.

### Medium: Telemetry nonce generation is deterministic after sequence rollover or reboot

Location:

- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_h7/tractor_m7.ino`, lines 233-241

The telemetry nonce is initialized as `{ SRC_TRACTOR, seq, 0,0,0,0, 0,0,0,0,0 }`. For AES-GCM, nonce reuse with the same key is dangerous. This will repeat after 16-bit sequence rollover and after device reboot if the same key is retained.

Recommendation: reuse the documented nonce builder shape for tractor telemetry too: source ID, frame sequence, timestamp or boot counter, and random bytes from a hardware RNG or a properly seeded source.

### Medium: Base station and bridge do not validate frame lengths, CRC, or payload ranges

Locations:

- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/web_ui.py`, lines 124-132
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/lora_bridge.py`, lines 166-168

`web_ui.py` casts browser-provided values to `int` but does not clamp axes to int8 range, clamp button/flag bitmasks, or handle `struct.pack()` failures. `lora_bridge.py` slices telemetry payloads using `payload_len` without checking that the frame actually contains `7 + payload_len + crc` bytes, and it does not verify telemetry CRC before publishing.

Recommendation: clamp and validate all browser inputs server-side. In the bridge, require exact or minimum frame lengths by frame type, verify CRC before publishing, and publish malformed-frame counters for diagnostics.

### Medium: Gamepad START can spam E-stop requests at 50 Hz

Location:

- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/web/app.js`, lines 160-172

`pollGamepad()` runs every 20 ms. If START is held, it sends a fetch to `/api/estop` every poll. That can queue many HTTP requests during the exact moment the operator needs predictable behavior.

Recommendation: edge-detect START, rate-limit E-stop posts, and make the E-stop command idempotent on the server/tractor side.

### Medium: Placeholder all-zero keys and crypto stubs must be isolated from anything field-like

Locations:

- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/lora_proto/crypto_stub.c`, lines 1-34
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/handheld_mkr/handheld.ino`, line 34
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_h7/tractor_m7.ino`, line 48
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/lora_bridge.py`, line 40

The draft correctly labels the C crypto as a stub, but the mixed state is risky: Python uses real AES-GCM with an all-zero key while Arduino-side C copies plaintext and appends a zero tag. These nodes cannot interoperate securely as written, and the all-zero key should not survive into any demo branch used near hardware.

Recommendation: add a build-time guard such as `#error Replace crypto_stub.c before non-sim builds`, load keys from a provisioning source, and provide a simulator-only mode that is unmistakably separate from hardware builds.

### Low: Button and joystick input need debouncing/calibration before operator trials

Locations:

- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/handheld_mkr/handheld.ino`, lines 45-56 and 127-133
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/web/app.js`, lines 122-134

The handheld reads raw ADC values with no deadband or calibration, and the physical/web buttons have no debounce or stuck-button handling. This is acceptable for a sketch, but it will produce drift and accidental toggles with real controls.

Recommendation: add per-axis center calibration, deadband, optional inversion, and button debounce. Keep the calibrated values in the same int8 wire format.

## Strengths

- The example folder is organized well for review: each tier is short, named clearly, and maps to the intended handheld -> tractor -> Opta -> base-station chain.
- KISS framing and CRC are straightforward and readable. The C and Python KISS implementations are close enough that shared tests can be written quickly.
- The split between radio/arbitration on the H747 and industrial I/O on Opta is sound and consistent with the architecture goals.
- The web UI/Gamepad API path is a useful operator-console prototype and should make bench testing easier once the command path and safety semantics are finished.
- The README is honest that this is study code, not compiled firmware. That framing should remain visible until the critical safety items above are resolved.

## Suggested Next Steps

1. Freeze one wire contract: exact frame sizes, byte order, CRC coverage, nonce contents, and Modbus register map.
2. Add host-side unit tests for `lora_proto`: CRC known vectors, KISS escaping, ControlFrame size, heartbeat size, telemetry size, and Python/C packing equivalence.
3. Replace `crypto_stub.c` with real AES-GCM or create a clearly named simulator mode that never ships in hardware builds.
4. Make the tractor safety path complete before adding features: fresh-control timeout, E-stop latch, M4 shared-memory heartbeat, Opta watchdog semantics, all eight coils, and both flow outputs.
5. Add a bench simulator for the Opta Modbus register map so the M7 master can be tested without hydraulic hardware.
6. Add Arduino CLI or PlatformIO compile coverage for the example sketches once the target board packages and libraries are chosen.

## Verification Notes

- VS Code diagnostics: no errors reported for the reviewed Python, JavaScript, C/C++, and Arduino files.
- Python runtime syntax check: not run because the Python environment tool could not locate a base interpreter.
- Arduino compile: not run. These sketches are marked as draft and include hardware/library placeholders.
- Hardware behavior: not tested.
