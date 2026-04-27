# LifeTrac v25 Controller Master Plan Review - Copilot v1.0

Date: 2026-04-27  
Reviewer: GitHub Copilot  
Scope: [`MASTER_PLAN.md`](../DESIGN-CONTROLLER/MASTER_PLAN.md), adjacent canonical controller docs, active draft firmware/services, and prior 2026-04-26 readiness reviews.

## Executive verdict

The master plan is a strong canonical anchor. The major architecture choices are directionally good: LoRa-only tractor control, Opta-as-industrial-I/O over RS-485, base-station web UI on local LAN, X8 as a Linux sidecar rather than a realtime controller, USB-only firmware updates for v25, and a real field-test gate.

However, the plan is not yet clean enough to be treated as the single build sheet for procurement, firmware, or bench bring-up. The largest risks are not small wording issues; they are contract mismatches between the plan, active code, and support docs. The most important ones are:

- The LoRa rate plan is oversubscribed once 20 Hz control, 20 Hz heartbeat, telemetry, and thumbnails are combined at the pinned SF7 / BW 125 kHz control PHY.
- The base-station radio path is contradictory: the master plan says Linux drives SX1276 directly over SPI, while active `lora_bridge.py` and some docs still assume a `base.ino` H747 UART modem.
- The D1608S routing decision is not fully propagated; the active Opta sketch still drives four directional coils from onboard EMRs.
- The base E-stop command can be rejected as replay after normal base control traffic because it is built with header sequence `0`.
- The M7/M4 realtime split in the plan does not match the active firmware: M7 currently owns arbitration, control mapping, and Modbus; M4 is only an alive GPIO watchdog.

My recommendation is to treat the plan as **architecturally promising but still RED/YELLOW for implementation readiness**. Do one deliberate cleanup pass before procurement or hardware energizing: freeze the radio budget, base-radio ownership, valve-output routing, sensor parts, E-stop clear flow, and CI/build targets.

---

## What the master plan gets right

These choices should be preserved:

1. **LoRa-only tractor RF scope.** Removing WiFi/MQTT-to-tractor, BLE/DroidPad, cellular fallback, and ROS2 from the active control path dramatically reduces the safety surface.
2. **Opta as isolated industrial I/O.** Keeping the hydraulic outputs behind a Modbus slave is a good safety partition. A LoRa-stack bug should not directly energize a coil.
3. **Base station as LAN appliance.** Plain HTTP plus shared PIN is acceptable only because the plan explicitly keeps the base on a trusted LAN for v25.
4. **X8 as sidecar.** The hard rule that H747 control never blocks on Linux is exactly right. Keep that rule prominent.
5. **USB-C flashing only.** No OTA firmware for v25 is the conservative choice.
6. **Field-test gate.** The 500 m line-of-sight, packet-loss, and E-stop-latency gate is concrete enough to drive real test cards.
7. **D1608S SSR correction.** Moving all latency-sensitive directional coils to SSR outputs is the right hardware decision, if the rest of the repo and code are brought into alignment.

---

## Critical findings

### 1. The LoRa airtime budget is oversubscribed

The master plan pins control + heartbeat to SF7 / BW 125 kHz / CR 4/5 and estimates about 30 ms airtime for a protected 16-byte `ControlFrame`. It also pins both `ControlFrame` and active-source `HeartbeatFrame` at 20 Hz.

That combination cannot hold as written:

- 20 Hz control at about 30 ms per frame consumes roughly 600 ms of airtime per second before CSMA gaps, commands, or collisions.
- A separate 20 Hz heartbeat stream adds more P0 airtime. Even if the heartbeat is shorter than control, the combined P0 stream is at or above practical channel saturation.
- SF8 and SF9 fallback make the problem worse. The plan lists about 55 ms at SF8 and about 100 ms at SF9 for a control frame. A 50 ms cadence cannot fit a 100 ms packet.
- The telemetry table in [`LORA_PROTOCOL.md`](../DESIGN-CONTROLLER/LORA_PROTOCOL.md) still includes GPS 5 Hz, IMU 5 Hz, hydraulics 2 Hz, plus video thumbnails. That is not credible on the same half-duplex SX1276 link unless most of it is gated, aggregated, or dropped.

Impact:

- Control latency, E-stop latency, and source arbitration can degrade exactly when the link is weak.
- The adaptive SF ladder can become self-defeating: stepping to SF9 improves demodulation margin but destroys the nominal 20 Hz cadence.
- Video thumbnails and high-rate telemetry can starve control unless a strict queue and airtime budget are implemented.

Recommendation:

- Make `ControlFrame` carry liveness. Send dedicated `HeartbeatFrame` only when no fresh control frame is being sent, or at a much lower rate such as 2-4 Hz for priority claim/status.
- Add an explicit airtime budget table to the master plan. Count protected on-air bytes, LoRa PHY airtime, cadence, and worst-case head-of-line blocking for P0/P1/P2/P3.
- Define degraded-link behavior honestly: at SF8/SF9 either reduce control cadence, merge heartbeat, disable all non-P0 traffic, or declare the link unsafe and fail neutral.
- Promote the QoS design from [`2026-04-26_LoRa_QoS_Bandwidth_Management.md`](2026-04-26_LoRa_QoS_Bandwidth_Management.md) into [`LORA_PROTOCOL.md`](../DESIGN-CONTROLLER/LORA_PROTOCOL.md) before implementing video or high-rate sensor telemetry.
- Keep LoRa thumbnails out of the hardware-test critical path. They are P3 opportunistic, not a requirement for first hydraulic motion.

### 2. Base-station LoRa ownership is contradictory

The master plan says the base station runs no Arduino firmware and that Linux on the X8 drives the SX1276 directly over SPI from `base_station/lora_bridge.py`. That is clean and reduces a firmware target.

But active and adjacent files still disagree:

- [`base_station/lora_bridge.py`](../DESIGN-CONTROLLER/base_station/lora_bridge.py) says the X8 H747 co-MCU runs `base.ino` and pipes KISS frames over UART.
- [`BASE_STATION.md`](../DESIGN-CONTROLLER/BASE_STATION.md) still shows `firmware/base_h7/base.ino` in the stack and source layout.
- [`LORA_PROTOCOL.md`](../DESIGN-CONTROLLER/LORA_PROTOCOL.md) still lists `firmware/base_h7/base.ino` in the planned layout.
- No active SPI SX1276 driver path is present in `base_station/` yet.

Impact:

- The current active bridge will not satisfy the master plan without a major rewrite.
- CI and bring-up cannot know whether to compile/flash a base H747 target.
- Hardware debugging will fork immediately: direct Linux SPI problems are very different from UART-modem problems.

Recommendation:

- Decide one base radio path now.
- If direct Linux SPI remains canonical, rewrite `lora_bridge.py` around `spidev`/GPIO IRQ control and remove all `base.ino`/`base_h7` references from canonical docs.
- If an H747 UART modem is the near-term practical path, amend the master plan and add a minimal `firmware/base_h7` target with CI. Do not leave this as an implicit fallback.

### 3. D1608S coil routing is not fully propagated

The master plan correctly pins all eight directional valve coils to D1608S SSR outputs. The four onboard Opta EMRs are reserved for engine kill, beacon/horn, parking brake release, and spare.

The repo still contains mixed routing:

- [`TRACTOR_NODE.md`](../DESIGN-CONTROLLER/TRACTOR_NODE.md) has a corrected wiring block showing SSR1-SSR8 for all directional coils, but its hardware overview still says D1608S carries "the other 4" directional coils.
- [`HARDWARE_BOM.md`](../DESIGN-CONTROLLER/HARDWARE_BOM.md) says D1608S adds the remaining four directional-valve drivers plus four spare SSR outputs.
- [`firmware/tractor_opta/opta_modbus_slave.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino) drives bits 0-3 through onboard relays R1-R4 and bits 4-7 through D1608S placeholder shims.

Impact:

- The active Opta firmware does not implement the low-latency routing promised by the master plan.
- Bench LED tests could pass with onboard EMRs and still fail the intended latency budget.
- Wiring four drive coils to EMRs defeats the entire section 8.18 correction.

Recommendation:

- Make D1608S channels 0-7 the only directional-coil outputs in both wiring and code.
- Move onboard EMR handling to `aux_outputs`/engine-kill paths only.
- Replace `d1608s_set()` placeholders with actual `OptaController` calls before any hydraulic test.
- Update BOM, tractor doc overview, and Opta comments so they all say the same thing.

### 4. Base E-stop can be rejected as replay

The master plan says `CMD_ESTOP` from any source preempts everything, always. Active code does not guarantee that.

In [`base_station/lora_bridge.py`](../DESIGN-CONTROLLER/base_station/lora_bridge.py), `/cmd/estop` builds a command frame whose header sequence is always `0`, then `_tx()` encrypts it with the bridge's current `tx_seq`. In [`firmware/tractor_h7/tractor_m7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino), replay protection uses the plaintext header sequence. Once the tractor has seen any normal base control frame with a sequence greater than zero, a later E-stop header sequence of `0` can be dropped as stale before command handling.

Impact:

- The web UI E-stop may work after boot and then silently fail after ordinary base control traffic.
- This is a hard blocker for presenting the browser E-stop as a safety control.

Recommendation:

- Make the bridge own the `SRC_BASE` sequence number for every outbound base frame, including control, camera commands, clear E-stop, and E-stop.
- Stamp the same sequence in the plaintext header and AES-GCM nonce.
- Add a regression test: send base control frames with sequence 1..N, then send E-stop, and verify the tractor latches E-stop.
- Consider accepting authenticated `CMD_ESTOP` idempotently even if its sequence is stale, but only after confirming that this does not create a replay hazard for non-E-stop commands.

### 5. M7/M4 responsibilities are not pinned consistently

The master plan says the H747 hot path includes the 100 Hz M4 control loop, 50 Hz Modbus master, 50 ms arbitration, E-stop latch, and LoRa frame verification. It also says Linux must never block those paths.

The active firmware is different:

- [`firmware/tractor_h7/tractor_m7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino) owns LoRa, replay checks, arbitration, joystick-to-coil mapping, E-stop latch, and the Modbus write block.
- [`firmware/tractor_h7/tractor_m4.cpp`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m4.cpp) only watches `SHARED->alive_tick_ms` and pulls a PSR-alive GPIO low if M7 stops ticking.

Impact:

- The plan overstates what the M4 currently protects.
- If M7 is alive but wedged in a bad control-mapping or Modbus logic path, the M4 watchdog may not catch it.
- Test plans based on "M4 owns control" will not test the firmware that actually moves the Opta registers.

Recommendation:

- Freeze the intended split in the master plan and code.
- Conservative option: M7 owns radio + Modbus, M4 only supervises M7/PSR. If so, edit the master plan to stop claiming a 100 Hz M4 control loop.
- More robust option: M7 verifies radio and writes selected source state to shared memory; M4 owns final control mapping, rate limiting, E-stop latch mirror, and Modbus writes. If so, refactor the active firmware before hardware testing.
- Either way, add a shared `last_successful_modbus_write_ms` or equivalent if the M4 is expected to detect Modbus wedge conditions.

### 6. E-stop clear/recovery flow is incomplete and inconsistent

The master plan and [`LORA_PROTOCOL.md`](../DESIGN-CONTROLLER/LORA_PROTOCOL.md) describe a clear E-stop flow with confirmation semantics. Active code currently lets `SRC_BASE` clear `g_estop_latched` with `CMD_CLEAR_ESTOP`, but the Opta safe state is latched and `on_holding_change()` refuses non-watchdog writes while faulted. That means a Modbus write setting `REG_ARM_ESTOP` back to zero may be ignored after the Opta enters `SAFETY_ESTOP_LATCHED`.

Impact:

- Tractor M7 and Opta can disagree about whether E-stop is cleared.
- Engine-kill output may remain latched until Opta reset, or the UI may show a cleared state while the I/O layer remains faulted.
- Without a physical acknowledgement/reset policy, clearing an E-stop from the browser can become too easy.

Recommendation:

- Define a single E-stop state machine covering base UI, LoRa command, M7 latch, Opta latch, PSR relay, engine-kill output, and recovery.
- Require local physical reset or a documented two-step clear token, not just any base command.
- Add input-register status bits so the base UI can distinguish `M7_ESTOP_LATCHED`, `OPTA_ESTOP_LATCHED`, `PSR_OPEN`, and `ENGINE_KILL_ACTIVE`.
- Bench-test clear/rearm with LEDs and a dummy engine-kill load before any hydraulics.

---

## High findings

### 7. Sensor hardware in the master plan disagrees with BOM and code

The master plan names:

- Adafruit BNO085 IMU.
- Adafruit FT232H USB-to-I2C/Qwiic adapter.

The active BOM, TODO, firmware, and Python requirements use:

- SparkFun BNO086 Qwiic IMU.
- Adafruit MCP2221A breakout (4471) with Linux `hid-mcp2221`/Blinka support.

Impact:

- Procurement can order the wrong bridge/IMU combination.
- Driver and service assumptions differ. FT232H and MCP2221A are not interchangeable at the software layer.
- Time-sync language mentions GPS PPS through the USB-to-Qwiic path. I2C/NMEA over MCP2221A does not carry PPS; PPS needs a separate GPIO-capable path if it is a real requirement.

Recommendation:

- Pick one canonical sensor stack. The current implementation clearly prefers MCP2221A + SparkFun BNO086 + SparkFun NEO-M9N SMA Qwiic.
- Amend the master plan if that is the intended path.
- If GPS PPS is required, add a separate PPS wire to an X8 GPIO or explicitly demote time sync to NMEA-derived for v25.

### 8. Command opcode tables are split and conflicting

The master plan and [`LORA_PROTOCOL.md`](../DESIGN-CONTROLLER/LORA_PROTOCOL.md) define opcodes such as `CMD_CLEAR_ESTOP = 0x02`, `CMD_CAMERA_SELECT = 0x03`, and `CMD_LINK_TUNE = 0x21`.

The active shared header still defines `CMD_REQ_CONTROL = 0x02` and `CMD_REKEY = 0x03`. Tractor firmware locally defines `CMD_REKEY = 0x10`. The bridge sends camera select as `0x03`, but the tractor command handler does not implement camera select.

Impact:

- Components can compile while disagreeing about command meaning.
- Camera selection appears wired in the web UI/bridge but is currently ignored by the tractor.
- Future link-tune work can collide with existing local enums.

Recommendation:

- Move every opcode into one shared source of truth and mirror it into Python from generated constants or a test-verified table.
- Add command decode tests for E-stop, clear E-stop, camera select, and link tune.
- Remove `CMD_REQ_CONTROL` or redefine it in the protocol if it is still needed.

### 9. Key management is specified but not implemented

The master plan says production uses `firmware/common/lora_proto/key.h`, gitignored, with a committed `key.h.example`. Active C code still uses `static const uint8_t kFleetKey[16] = {0};`; Python uses `FLEET_KEY = bytes(16)`.

The C crypto stub is now guarded by `LIFETRAC_ALLOW_STUB_CRYPTO`, which is good, but real AES-GCM is not implemented on the embedded side.

Impact:

- Production firmware cannot compile securely yet.
- C nodes and Python bridge cannot interoperate unless the embedded side gets real AES-GCM or Python is deliberately put into simulator mode.
- AES-GCM nonce uniqueness across reboot is still not implemented as described. Current sequence counters are volatile and 16-bit.

Recommendation:

- Add `key.h.example`, `.gitignore` entry, and C include path.
- Load Python key from the same provisioning artifact, not a literal zero key.
- Add real MbedTLS/ArduinoBearSSL AES-GCM before field-like radio tests.
- Persist per-source sequence/boot counter or widen the nonce strategy so reboot does not risk nonce reuse with the same key.

### 10. Web UI authentication is planned but absent

The master plan pins a shared PIN, session cookie, idle timeout, and wrong-PIN rate limiting. Active [`base_station/web_ui.py`](../DESIGN-CONTROLLER/base_station/web_ui.py) has no authentication layer yet; control and E-stop endpoints are exposed to anyone who can reach the service.

Impact:

- The current UI is fine as a local draft but not consistent with the v25 safety model.
- A LAN peer could send joystick frames or E-stop requests.

Recommendation:

- Add the PIN/session gate before any multi-device LAN test.
- Keep E-stop always available but authenticated command release/clear should require the session plus explicit confirmation.
- Bind Mosquitto to loopback and document firewall rules in the active compose/service files.

### 11. CI still targets archived/non-canonical paths

The GitHub Actions workflow and [`ARDUINO_CI.md`](../DESIGN-CONTROLLER/ARDUINO_CI.md) still reference `DESIGN-CONTROLLER/arduino_opta_controller` and `DESIGN-CONTROLLER/esp32_remote_control`. The active tree is under `firmware/handheld_mkr`, `firmware/tractor_h7`, `firmware/tractor_opta`, and `base_station/`.

Impact:

- The canonical firmware can change without compile validation.
- The CI docs reinforce the archived WiFi/BLE/MQTT path.

Recommendation:

- Rewrite CI for MKR WAN 1310, Portenta X8/H747 target(s), and Opta Modbus slave.
- Add Python syntax/tests for `base_station` and `firmware/tractor_x8`.
- Pin Arduino and Python library versions.
- Remove automatic issues that assign archived-path failures to the active build.

### 12. Cellular references are too numerous for safe procurement

Many canonical files include banners saying cellular is archived, but still list SIMs, cellular antennas, SARA power budget, cellular fallback tests, and cellular E-stop paths.

Impact:

- Procurement could still order SIMs and antennas from TODO/BOM line items.
- Testers may expect cellular fallback behavior that the master plan explicitly rejects.
- `LORA_PROTOCOL.md` still says `CMD_ESTOP` is sent over both LoRa and cellular, contradicting the LoRa-only plan.

Recommendation:

- Move cellular rows out of active BOM/TODO tables into an archived appendix or remove them from procurement lists entirely.
- Delete cellular fallback tests from active bring-up phases.
- Replace `cellular_fallback` control flag with a reserved bit unless cellular is re-promoted by a master-plan amendment.

### 13. Coil voltage and power rail are not pinned cleanly

The master plan repeatedly frames the D1608S outputs as 24 VDC / 2 A and describes a 24 V valve rail. [`HARDWARE_BOM.md`](../DESIGN-CONTROLLER/HARDWARE_BOM.md) notes that the directional valves are 12 V, not 24 V. Tractor electrical is also described as a 12 V system.

Impact:

- Wrong coil-voltage assumptions can lead to wrong PSR rail, fusing, flyback, and SSR thermal calculations.
- D1608S suitability should be checked against actual coil voltage/current, inrush, inductive load handling, leakage current, and off-state behavior.

Recommendation:

- Add a single canonical coil voltage/current table: nominal voltage, pull current, hold current, measured coil resistance, fuse size, diode/TVS choice, and D1608S channel assignment.
- State whether the valve rail is tractor 12 V, converted 24 V, or selectable by coil SKU.

### 14. Camera scope is too entangled with the first hydraulic test

The master plan says cameras are optional for LED stand-in bring-up but asks for at least the front Kurokesu before the single-valve hydraulic test. It also reserves LoRa thumbnail topics and discusses selected-camera transmission.

This is useful eventually, but it risks pulling video into the safety-critical path too early.

Impact:

- Video can consume engineering time before the control/safety chain is proven.
- LoRa thumbnails are not viable without QoS and fragmentation discipline.
- BOM and TODO still disagree between Kurokesu C2-290C, Kurokesu C1 PRO, Logitech, and ELP.

Recommendation:

- For first hydraulic test, require direct line-of-sight or a local wired/WiFi camera outside the LoRa control channel, not LoRa thumbnails.
- Keep Kurokesu C2-290C as the preferred production camera only after the sensor/camera BOM is reconciled.
- Treat LoRa thumbnails as P3 post-QoS work, not a gate for single-valve hydraulic testing.

### 15. X8-to-H747 IPC contract is underspecified

The master plan says X8 services publish pose/time/parameters through non-blocking IPC and the H747 reads last-known values with stale flags. Active services send KISS-framed topic payloads over `Serial1` to the M7.

Impact:

- The plan's safety guarantee depends on stale-flag semantics that are not documented at byte level.
- Sensor disconnects, UART overflow, stale GPS, and stale IMU behavior are not fully specified.
- `params_service.py`, `logger_service.py`, `event_recorder.py`, and `time_service.py` are pinned for v25 but not present in the active tree.

Recommendation:

- Define the actual X8-H747 IPC: transport, framing, message types, stale timeout, buffer limits, and failure behavior.
- Decide whether pinned X8 services are required before bench bring-up or before field testing. GPS/IMU/logging are useful, but not all are blockers for LED or single-valve tests.
- Add a "minimum X8 services for bench" subsection to the master plan.

### 16. `CMD_LINK_TUNE` adaptive SF is fragile as specified

The plan requires the tractor to send `CMD_LINK_TUNE` at both current and new SF/BW, then both ends switch and revert if no heartbeat arrives within 500 ms.

Impact:

- If heartbeat cadence is reduced or congested, the 500 ms revert window may create oscillation.
- Half-duplex timing and independent base/handheld sources make simultaneous retune tricky.
- Telemetry remains on an independent SF, but the same radio cannot truly listen on two SF/BW settings at once without a scheduling discipline.

Recommendation:

- Defer adaptive SF until the fixed SF7/BW125 or SF7/BW500 link is proven.
- First implement link-state telemetry and manual/test-only SF switching.
- Add a state diagram for retune, including what happens when handheld and base are both present.

---

## Medium findings and cleanup items

1. **`LORA_PROTOCOL.md` still contains old benchmark rows.** Bench criteria still mention SF7/BW500 and long-range SF9/BW250 expectations. Update these to match the final PHY decision.
2. **Telemetry max size is confusing.** The spec says payload 0..120 and frame 9-128 bytes, while active code limits payload to `sizeof(payload)-2` because CRC is stored after payload in the same array. Freeze one max and add boundary tests.
3. **Base MQTT exposure needs enforcement.** The plan says loopback-only. Some docs still mention Mosquitto LAN-only or port 1883. Make loopback binding part of active config.
4. **Operator source-disable behavior is not implemented.** The plan says base controls are disabled when handheld is active. The web UI currently updates the banner but still sends control frames. It should drop joystick transmissions or set read-only mode when source is not `BASE`.
5. **Master plan reserves autonomy opcodes but says waypoint transport is not on LoRa.** The QoS note later proposes parked-mode plan transfer over LoRa. Decide whether that is v25-out-of-scope research or a future amendment candidate.
6. **Use of `random()` for nonce bytes is not proven seeded.** Embedded code should use hardware RNG or a persisted boot counter; otherwise the random nonce tail may be predictable/repeated.
7. **The active code is still marked draft and uncompiled.** The master plan should include an implementation status box so readers do not mistake docs for tested firmware.
8. **Arduino libraries are unpinned and legacy-oriented.** [`arduino_libraries.txt`](../DESIGN-CONTROLLER/arduino_libraries.txt) still lists WiFi/BLE/MQTT libraries for archived designs rather than RadioLib, ArduinoRS485, ArduinoModbus, OptaController, and crypto dependencies for the active tree.
9. **Physical E-stop topology needs one drawing.** The plan mentions handheld, base, tractor-mounted, PSR, Opta alive, and M4 alive. Collapse this into one ladder diagram with normally-open/normally-closed contact states.
10. **Logger/event-recorder storage should be constrained.** Rotating SQLite is fine, but specify retention, write frequency, power-loss behavior, and how logs are recovered.

---

## Suggested master-plan amendments

### A. Add a `Build Contract` section

State these values in one table and require every implementation/doc to match:

| Contract | Recommendation |
|---|---|
| Base radio path | Pick direct Linux SPI or H747 UART modem; do not leave both active. |
| Control PHY | Re-evaluate SF7/BW125 against actual airtime. If kept, merge/reduce heartbeats. |
| P0 cadence | Control 20 Hz; heartbeat only if needed, not another 20 Hz stream. |
| Telemetry cadence | Aggregate to a budgeted P1 frame set; disable high-rate topics until QoS exists. |
| Valve outputs | D1608S SSR1-SSR8 for all directional coils. Opta EMR only aux/engine-kill. |
| Sensor stack | MCP2221A+BNO086 or FT232H+BNO085, one only. |
| Key path | `key.h`/Python env secret, no zero literals. |
| E-stop clear | Physical reset/token/state machine pinned before field. |

### B. Add an `Implementation Status` box

Suggested wording:

> Active firmware and Python services under `DESIGN-CONTROLLER/firmware/` and `DESIGN-CONTROLLER/base_station/` are draft, not compiled, not flashed, and not hardware-tested. They are the intended canonical tree, but they do not yet satisfy this master plan.

That prevents a future reader from treating design completeness as build readiness.

### C. Replace the current wire-rate table

The current section 8.12 table is too optimistic without airtime. Add columns for protected bytes, estimated airtime, max acceptable jitter, and degradation behavior.

Example shape:

| Class | Frame | Normal cadence | Degraded cadence | Drop policy |
|---|---|---:|---:|---|
| P0 | ControlFrame | 20 Hz | 10-20 Hz depending SF | newest only; no retry |
| P0 | Heartbeat | 2-4 Hz or control-carried | 1-2 Hz | source-liveness only |
| P0 | E-stop | event | event | authenticated, idempotent |
| P1 | Aggregated state | 1-2 Hz | 0.2-1 Hz | drop old |
| P3 | Thumbnail/log bulk | opportunistic | off | never blocks P0 |

### D. Move archived cellular out of active procurement

Do not rely on banners. Put archived cellular parts in a separate appendix or `RESEARCH-CONTROLLER/` note. Active BOM/TODO should not say "order SIM" if the master plan says no SIMs.

### E. Split bring-up gates into `LED`, `single-valve`, and `field`

Recommended gates:

1. **LED bench gate:** build-validated firmware, all 8 D1608S channels mapped, both AO outputs zero/scale tested, E-stop latch/clear tested with LED/buzzer loads.
2. **LoRa bench gate:** base/handheld/tractor interop with real AES or explicit stub-sim mode, replay tests, E-stop after prior control traffic, source-priority tests.
3. **Single-valve hydraulic gate:** written test card, low pressure, one valve only, PSR measured, operator line-of-sight or independent camera, no LoRa thumbnails required.
4. **Field gate:** use the existing 500 m field-test criteria.

---

## High-value next actions

1. Fix base E-stop sequencing in `lora_bridge.py` and add a replay regression test.
2. Decide base radio ownership: direct SPI or H747 UART modem.
3. Rework the LoRa rate plan: merge/reduce heartbeat, add QoS queue, and revise SF8/SF9 degraded behavior.
4. Align D1608S routing across master plan, BOM, tractor doc, and Opta firmware.
5. Update CI to compile the active firmware tree and run Python checks.
6. Replace all-zero key literals with the `key.h`/secret-file flow described by the master plan.
7. Amend the master plan sensor rows to match MCP2221A+BNO086 or change the BOM/code to FT232H+BNO085.
8. Implement PIN/session auth before any LAN control demo.
9. Remove active cellular procurement/test steps from TODO/BOM.
10. Write the E-stop state machine and bench test card.

## Bottom line

The core architecture is worth keeping. The plan is at the point where the next improvement is not more scope; it is contraction and alignment. Freeze the radio budget, base-radio path, D1608S wiring, M7/M4 split, key provisioning, and E-stop recovery. Then make the code and CI prove those contracts before any live hydraulic test.
