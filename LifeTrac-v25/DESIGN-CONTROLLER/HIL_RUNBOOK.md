# HIL Runbook — Wave 4 Gates W4-01 … W4-10

**Scope.** Step-by-step bench procedures for the ten Wave-4 hardware-in-the-loop
gates currently open in [`TODO.md`](TODO.md). Each section is structured the
same way so the operator can execute without re-deriving the plan and so the
artifacts collected become the test-evidence package for the §A CI compile-gate
flip and the SAFETY_CASE update.

This is a **runbook**, not a design document. Read [`SAFETY_CASE.md`](SAFETY_CASE.md),
[`HARDWARE_BOM.md`](HARDWARE_BOM.md), and [`OPERATIONS_MANUAL.md`](OPERATIONS_MANUAL.md)
first; this file assumes you already know the architecture.

---

## 0. Common bench setup

Before running any W4 gate, complete the following once per session.

### 0.1 Hardware

- [ ] Tractor crate powered from 12 V battery emulator (set 13.0 V, 30 A limit).
- [ ] Handheld powered from charged Li-ion (≥ 3.9 V).
- [ ] All 8 valve coils replaced with **dummy loads** (12 V, 5 W incandescent
      bulbs or 24 Ω resistors) so coil energization is visible without moving
      hydraulics. Real solenoids only after every gate passes.
- [ ] PSR-alive line broken out to oscilloscope CH1 (M4 GPIO PIN_PSR_ALIVE,
      see [`SAFETY_CASE.md`](SAFETY_CASE.md) §3).
- [ ] One relay coil terminal (any of the eight) broken out to scope CH2 — used
      as a proxy for "valves de-energized".
- [ ] Logic analyzer on:
  - X8↔H747 UART (Serial1, 115200 8N1) — channels 0/1
  - M7↔M4 GPIO PIN_PSR_ALIVE — channel 2
  - H747↔Opta RS-485 A/B differential — channels 3/4 (use differential probe
    or RS-485-to-TTL breakout)
  - SX1276 DIO0 IRQ line — channel 5 (used by W4-09)

### 0.2 Software

- [ ] Base-station container running, MQTT broker reachable on `localhost:1883`.
- [ ] `journalctl -u lifetrac-base -f` open in one terminal.
- [ ] `mosquitto_sub -t 'lifetrac/#' -v` open in another.
- [ ] `tail -f /var/log/lifetrac/audit.jsonl` open in a third.
- [ ] Web UI open at `http://localhost:8000`, logged in (PIN session active).
- [ ] All firmware built **without** `LIFETRAC_ALLOW_UNCONFIGURED_KEY` —
      i.e., the production gate from §D Round 9 is exercised. Real
      `lp_keys_secret.h` provisioned per [`PAIRING_PROCEDURE.md`](PAIRING_PROCEDURE.md).

### 0.3 Capture conventions

- All scope captures saved as `W4-XX_run-NN_YYYY-MM-DD.csv` with channel headers
  preserved. Store under `bench-evidence/W4-XX/`.
- All audit-log captures saved as `W4-XX_run-NN_audit.jsonl` (filtered with
  `jq 'select(.ts >= "<start>")'`).
- All MQTT captures saved as `W4-XX_run-NN_mqtt.txt`.
- Each run gets a one-paragraph note in `bench-evidence/W4-XX/NOTES.md`
  recording: date, operator, fw git SHAs (handheld + M7 + M4 + Opta + base),
  battery voltage, ambient temperature, link-budget config (TX power, SF),
  result (PASS / FAIL / RETRY), and any deviation from the procedure.

### 0.4 Abort criteria (any gate)

Stop immediately and document if any of the following occur:

- Battery emulator current exceeds 25 A (short upstream of fuse).
- Smoke, smell, or temperature rise above 60 °C on any board.
- A valve coil energizes when no operator input is present (uncommanded motion;
  this is itself a finding — capture audit log, do not retry until root-caused).
- The audit log contains an `unexpected_panic` or `crc_mismatch_burst` event
  unrelated to the test stimulus.

---

## W4-01 — Handheld E-stop latch latency

**Gate.** Mushroom-button press → PSR-alive drops → all 8 valve coils
de-energize within **< 100 ms**, measured at the relay terminals. Repeat
**100×** across SF7/SF8/SF9 PHY rungs. Capture worst-case + histogram.

**Why.** Closes the safety-case timing claim in
[`SAFETY_CASE.md`](SAFETY_CASE.md) §4 (E-stop latency budget). The Round-8 SIL
test [`test_m4_safety_sil.py`](base_station/tests/test_m4_safety_sil.py) already
proves the M4 supervisor honours the 200 ms watchdog logically; W4-01 proves
the end-to-end air-link + PSR latch hits the tighter 100 ms operator budget.

**Pre-conditions.**
- §0 setup complete.
- Scope CH1 = PSR-alive, CH2 = relay coil 1 terminal.
- Trigger: CH1 falling edge, threshold 1.5 V, holdoff 500 ms.
- Time base 20 ms/div, 200 ms total window pre-trigger 50 ms.
- Handheld driving full-throttle stick (left & right tracks) so all four track
  coils are energized at trigger time. Arm axes cycle 1 Hz so all eight coils
  see at least one energization per second.
- Confirm `mosquitto_sub` shows `lifetrac/handheld/state` at ≥ 8 Hz.

**Procedure (one rung).**
1. Set SF via debug command `mosquitto_pub -t lifetrac/cmd/link_tune -m "SF7"`
   and confirm `lifetrac/m7/phy_state` reports `sf=7` within 1 s.
2. Drive sticks to full deflection; verify all eight relay coils show ≥ 11.5 V.
3. Press mushroom button **firmly and fully** (do not feather). Operator
   keeps hand on button until scope captures.
4. Capture: PSR-alive falling edge → CH2 falling edge (coil de-energize).
   Record Δt as `latency_ms`.
5. Save scope CSV per §0.3.
6. Reset handheld E-stop (twist-release). Wait 5 s for re-pair.
7. Repeat from step 2 until **100** captures are stored for this SF.

**Per-rung pass criteria.**
- 100/100 captures with `latency_ms < 100 ms`.
- Worst case (max `latency_ms`) recorded; should be < 80 ms with 20 ms margin.
- No "missed E-stop" event — i.e., no run where coils stayed energized > 100 ms.

**Cross-rung pass criteria.**
- All three SF rungs (7, 8, 9) pass per-rung criteria.
- Histogram shows monotonic increase in median latency from SF7 → SF9 (sanity
  check that the higher SF actually slows the air link).

**Artifacts.**
- 300 scope CSVs (3 rungs × 100 runs).
- 3 histogram plots (matplotlib script in `tools/`, future work).
- One summary row appended to `bench-evidence/W4-01/SUMMARY.csv`:
  `date, operator, sf, n, p50_ms, p95_ms, p99_ms, max_ms, pass`.

**Failure modes & next steps.**
- `latency_ms > 100 ms` on any single run → STOP, capture surrounding 5
  packets via `mosquitto_sub`, check for `crc_mismatch` near the trigger.
  Likely root cause: missed handheld TX (re-check antenna), or M4 watchdog
  threshold too lax (verify `LIFETRAC_M4_WATCHDOG_MS = 200`).
- Coils don't de-energize at all → PSR latch stuck closed; check 24 V coil
  on PSR + read [`SAFETY_CASE.md`](SAFETY_CASE.md) §6 (latch-failure mode).
  Do **not** continue testing until PSR is fixed.

---

## W4-02 — Link-tune walk-down

**Gate.** Force RSSI degradation (RF attenuator) and verify the M7 walks
SF7 → SF8 → SF9 and back via `CMD_LINK_TUNE` with **< 1 % packet loss**
during each transition. Verify `try_step_ladder()` revert deadline (500 ms)
fires correctly on a synthetic missing-HB.

**Why.** Validates the adaptive PHY ladder added in IP-201; protects the
operator from a silent walk-up that leaves them on a slow rung when the
link recovers.

**Pre-conditions.**
- §0 setup complete.
- Variable RF attenuator (0–60 dB, 0.5 dB step) inline between handheld
  and tractor antenna pair. SMA pigtails per [`HARDWARE_BOM.md`](HARDWARE_BOM.md).
- Spectrum analyzer or RTL-SDR sniffing 915 MHz to confirm TX bursts.
- `mosquitto_sub -t 'lifetrac/m7/phy_state' -v` capture running, output
  piped to `W4-02_phy_state.log`.
- Sticks driving sinusoidal full-deflection 0.5 Hz so packet loss is visible
  as motion stutter on the dummy-coil scope.

**Procedure.**
1. **Walk-down.** Start at 0 dB attenuation. Confirm SF7 reported.
2. Increase attenuation 3 dB/min until M7 reports SF8. Record attenuation
   at transition (`atten_to_sf8_db`).
3. Continue 3 dB/min until SF9 reported (`atten_to_sf9_db`).
4. Hold at SF9 for 60 s; capture `mosquitto_sub` to count `seq_gap` events.
5. **Walk-up.** Decrease attenuation 3 dB/min until SF7 returns.
6. **Revert deadline.** Set SF7. Inject a synthetic missed-HB by issuing
   `mosquitto_pub -t lifetrac/debug/drop_hb -m "1"` (debug-build feature;
   see [`LORA_PROTOCOL.md`](LORA_PROTOCOL.md) §7). Verify `try_step_ladder()`
   reverts within 500 ms (capture in `phy_state.log`).

**Pass criteria.**
- Walk-down hits SF8 and SF9 monotonically, no skipping.
- Walk-up returns to SF7 within 30 s of attenuation < `atten_to_sf8_db - 3 dB`.
- During each transition, packet loss < 1 % (counted from `seq_gap` over a
  10 s window centred on the transition).
- Revert-deadline test: `phy_state.log` shows SF↑ within `500 ± 50 ms`.

**Artifacts.**
- `W4-02_phy_state.log` covering the full 30+ minute run.
- `W4-02_atten_log.csv`: `t, atten_db, sf, rssi_dbm, packet_loss_pct`.
- Spectrogram screenshot at each SF transition.

**Failure modes.**
- SF skips a rung (7→9 directly) → bug in ladder logic; check
  `try_step_ladder()` thresholds in `firmware/tractor_h7/tractor_h7.ino`.
- Walk-up never returns to SF7 → hysteresis too wide; tune `LINK_TUNE_*`
  constants in [`firmware/common/lora_proto/lora_proto.h`](firmware/common/lora_proto/lora_proto.h).
- Revert deadline > 550 ms → seqlock contention; profile with
  [`base_station/tests/test_m4_safety_sil.py`](base_station/tests/test_m4_safety_sil.py)
  to confirm the SIL still passes the same scenario.

---

## W4-03 — M7↔M4 watchdog trip

**Gate.** Halt the M7 with a debugger breakpoint or a deliberate `while(1)`
injected via a debug build; verify the M4 trips the PSR within **200 ms** of
the last `alive_tick_ms` and that `estop_request = 0xA5A5A5A5` is observed
on the SRAM4 capture (IP-105/106 seqlock holds).

**Why.** End-to-end verification of the SIL model in
[`test_m4_safety_sil.py`](base_station/tests/test_m4_safety_sil.py).

**Pre-conditions.**
- §0 setup complete.
- Debug-build firmware on M7 with the `while(1)` halt path enabled by
  defining `LIFETRAC_DEBUG_HALT_TRIGGER` (see firmware sketch comment block).
- J-Link or equivalent SWD probe on the H747 debug header.
- Logic analyzer triggered on PSR-alive falling edge.
- SRAM4 capture: J-Link script `tools/sram4_capture.jlink` (future) or
  manual `monitor mem32 0x38000000 0x100` reads pre/post halt.

**Procedure.**
1. Confirm baseline: M7 stamping `alive_tick_ms` every loop, M4 reporting
   `loop_counter` incrementing in MQTT telemetry.
2. Trigger debug halt:
   - Method A (preferred): `mosquitto_pub -t lifetrac/debug/halt_m7 -m "1"`
     causes M7 to enter `while(1)` after the next loop iteration.
   - Method B: J-Link `halt` command on the M7 core only (M4 keeps running).
3. Note `t_halt` (wall clock when halt issued).
4. Capture scope: PSR-alive falling edge → record Δt = `t_trip - t_halt`.
5. Read SRAM4 via J-Link: confirm `estop_request == 0xA5A5A5A5`.
6. Confirm all 8 dummy coils de-energized.
7. Reset M7 (J-Link `reset` or power cycle); verify PSR-alive returns high
   only after fresh `alive_tick_ms` stream.

**Pass criteria.**
- 10/10 runs with `t_trip - t_halt < 200 ms` (200 ms is the watchdog
  threshold; expect ≈ 210–220 ms total including detection slack — verify
  this matches the SIL model's prediction).
- SRAM4 `estop_request == 0xA5A5A5A5` in 10/10 runs.
- After reset, no auto-recovery: operator must twist-reset the handheld
  E-stop **and** re-pair before motion is re-enabled.

**Artifacts.**
- 10 scope CSVs.
- 10 J-Link SRAM4 dumps (200 bytes each).
- One `W4-03_SUMMARY.csv` row.

**Failure modes.**
- Trip > 200 ms → re-check `LIFETRAC_M4_WATCHDOG_MS` and seqlock retry
  count; cross-check against `M4Supervisor` SIL model.
- `estop_request` not set → IP-105/106 seqlock regression; halt the gate,
  re-run [`test_m4_safety_sil.py`](base_station/tests/test_m4_safety_sil.py),
  bisect firmware.
- M4 auto-recovers without operator action → safety-case violation; STOP.

---

## W4-04 — Modbus failure → E-stop

**Gate.** Pull the RS-485 cable mid-run; verify IP-205 counter ticks,
`apply_control(-1)` fires after 10 consecutive failures, valves go neutral,
and the audit log shows the disconnect within 1 s.

**Pre-conditions.**
- §0 setup complete.
- RS-485 cable terminated in a quick-disconnect (Phoenix screw terminal
  or XLR-style locking).
- Driving slow ramp on track sticks so coil state is observable.
- `mosquitto_sub -t 'lifetrac/m7/modbus_stats' -v` capture running.

**Procedure.**
1. Baseline: confirm `modbus_stats.failures == 0`, `modbus_stats.ok` ticking.
2. Disconnect RS-485 cable. Note `t_disconnect`.
3. Watch `modbus_stats.failures` increment. Record time at first failure
   (`t_first_fail`) and at 10th consecutive failure (`t_apply_-1`).
4. Verify all 8 dummy coils de-energize at or before `t_apply_-1`.
5. Verify audit log contains `modbus_disconnect` event with `ts` between
   `t_disconnect` and `t_apply_-1 + 100 ms`.
6. Reconnect cable; verify M7 resumes after 5 consecutive successes
   (`modbus_stats.ok` resumes ticking) **and** operator must release
   sticks to neutral first (no motion until both conditions met).

**Pass criteria.**
- `t_apply_-1 - t_disconnect < 1.0 s` in 10/10 runs.
- Audit log entry present, schema valid.
- No motion resumes without stick-release.

**Artifacts.**
- `W4-04_modbus_stats.log` × 10 runs.
- 10 audit-log filtered captures.

**Failure modes.**
- `apply_control(-1)` doesn't fire → check IP-205 counter logic in
  [`firmware/tractor_h7/tractor_h7.ino`](firmware/tractor_h7/tractor_h7.ino);
  may indicate the failure-count threshold is read from a stale register.
- Audit log missing entry → AuditLog singleton (Round 9 §E) silently
  failing; check `_get_audit_log()` returns non-None.

---

## W4-05 — Proportional valve ramp-out

**Gate.** Hold full-speed track for 3 s, release joystick; verify
`REG_FLOW_SP_*` ramps from 10000 mV to 0 over **2 s** (track ladder) on
the A0602 output, valve coil stays energized through the ramp, drops at
t = 2 s. Repeat for arm axes (1 s ramp).

**Pre-conditions.**
- §0 setup complete.
- Multimeter or scope on Opta A0602 analog output (0–10 V) for the
  axis under test.
- `mosquitto_sub -t 'lifetrac/opta/flow_sp' -v` capture running.

**Procedure (per axis).**
1. Drive full deflection for 3 s. Confirm `flow_sp == 10000` mV on MQTT
   and ≈ 10 V on scope.
2. Release stick to neutral. Note `t_release`.
3. Capture analog output ramp; record `t_zero` when output crosses 100 mV.
4. Confirm coil stays energized (CH2 high) until `t_zero ± 50 ms`.
5. Repeat 5× per axis. Track axes: target 2 s ramp. Arm axes: target 1 s.

**Pass criteria.**
- Track axes: `t_zero - t_release == 2.00 ± 0.10 s`.
- Arm axes: `t_zero - t_release == 1.00 ± 0.05 s`.
- Linear ramp (R² > 0.98) — use the saved scope CSV with a Python fit.
- Coil never drops before `t_zero`.

**Artifacts.**
- 6 axes × 5 runs = 30 scope CSVs.
- One `W4-05_SUMMARY.csv` row per axis.

---

## W4-06 — Mixed-mode skip

**Gate.** Drive both axes; release one; verify released axis stops
*immediately* (no ramp) because sibling is active. Critical for coordinated
dig→drive transitions.

**Pre-conditions.**
- §0 setup complete.
- Scope on two coils (one per axis pair).

**Procedure.**
1. Drive left track full + right track full. Confirm both coils energized.
2. Release left stick only. Note `t_release_left`.
3. Capture: left coil should drop within **50 ms** (no 2 s ramp, since
   right track still active triggers the "mixed mode" skip).
4. Release right stick. Right coil should ramp normally (W4-05 behaviour)
   because left is already down.
5. Repeat for arm-axis pair (lift + tilt).

**Pass criteria.**
- Released-while-sibling-active axis drops within 50 ms in 10/10 runs.
- Released-as-only-active axis ramps per W4-05.

**Failure modes.**
- Released axis ramps anyway → mixed-mode detection bug in
  `apply_control()` ramp logic; check the `sibling_active` predicate.

---

## W4-07 — Boot-PHY first-frame decode

**Gate.** Cold-boot both ends; first encrypted frame decodes on the receiver
with no `bad_header` events in the audit log.

**Pre-conditions.**
- §0 setup complete.
- Both handheld and tractor fully powered down for ≥ 30 s.
- Audit log tail running.
- `mosquitto_sub -t 'lifetrac/audit/bad_header' -v` running.

**Procedure.**
1. Power on tractor first. Wait for `lifetrac/m7/state == ready`.
2. Power on handheld. Note `t_handheld_on`.
3. Capture first 10 s of audit log + MQTT.
4. Confirm first `lifetrac/handheld/state` MQTT message arrives within
   3 s of `t_handheld_on`.
5. Confirm zero `bad_header` events in that 10 s window.
6. Repeat 20× alternating power-on order (handheld first then tractor for
   half the runs).

**Pass criteria.**
- 20/20 runs with first frame decoded successfully.
- 0 `bad_header` events.

**Failure modes.**
- `bad_header` events → boot-PHY mismatch; verify both ends start at SF7
  and that `radio.begin()` completes before TX. See IP-006 in
  [`LORA_PROTOCOL.md`](LORA_PROTOCOL.md).

---

## W4-08 — Camera back-channel round-trip

**Gate.** Operator presses "Force keyframe" on web UI → `CMD_REQ_KEYFRAME`
over LoRa → M7 forwards on Serial1 → `camera_service.py` emits I-frame
within **< 200 ms** end-to-end. Verify with frame-flag capture from the
X8↔H747 UART.

**Pre-conditions.**
- §0 setup complete.
- Coral X8 attached, camera streaming, `lifetrac/image/frame` MQTT topic
  publishing at ≥ 5 fps.
- Logic analyzer on X8↔H747 UART decoding ASCII text.

**Procedure.**
1. Capture baseline: P-frames flowing, keyframe interval ≈ 60 frames.
2. Click "Force keyframe" in web UI. Note browser timestamp `t_click`
   (use browser devtools).
3. Capture UART: look for `KEYFRAME` ASCII command from H747 to X8.
   Record `t_uart_cmd`.
4. Capture next MQTT frame with `is_keyframe == true`. Record `t_keyframe`.
5. Compute `t_keyframe - t_click` end-to-end latency.
6. Repeat 30 times.

**Pass criteria.**
- `t_keyframe - t_click < 200 ms` in 30/30 runs.
- `t_uart_cmd - t_click < 100 ms` (LoRa leg).

**Artifacts.**
- 30 logic-analyzer captures (UART decode).
- `W4-08_latency.csv` with all timestamps.

**Failure modes.**
- > 200 ms end-to-end but UART leg < 100 ms → X8 / camera_service.py is
  the bottleneck; profile [`base_station/.../camera_service.py`](base_station).
- UART leg > 100 ms → LoRa back-channel queue stuck behind a TX; this is
  expected to improve once W4-09 lands the async TX state machine.

---

## W4-09 — Async M7 TX state machine

**Gate.** Bench-validate `radio.startTransmit()` + `isTransmitDone()` IRQ
timing on real H747 + SX1276 wiring. Once green, replace the
`refresh_m4_alive_before_tx()` watchdog-refresh hack with the proper
non-blocking queue. Removes the worst-case time-on-air margin assumption.

**Pre-conditions.**
- §0 setup complete.
- Logic analyzer CH5 on SX1276 DIO0 (TxDone IRQ).
- Debug-build firmware exposing IP-107 instrumentation: timestamps for
  `startTransmit()` call, DIO0 IRQ assert, `isTransmitDone()` true return.

**Procedure (Phase 1 — measure).**
1. Run normal traffic at SF7. Capture 100 TX cycles.
2. Record `t_dio0 - t_start` (time-on-air, should match LoRa calculator).
3. Record `t_isdone - t_dio0` (IRQ → flag latency, should be < 1 ms).
4. Repeat at SF8, SF9.

**Phase 1 pass criteria.**
- Measured time-on-air within 5 % of theoretical for each SF.
- IRQ → flag latency < 2 ms p99.

**Procedure (Phase 2 — replace hack).**
5. Land the non-blocking TX queue (separate PR; replace
   `refresh_m4_alive_before_tx()` calls with queue submit).
6. Re-run W4-03 and W4-09 Phase 1. Both must still pass.
7. Verify `alive_tick_ms` keeps ticking within 5 ms even during a TX cycle
   (this is the whole point of removing the refresh hack).

**Phase 2 pass criteria.**
- W4-03 still passes (200 ms watchdog gate held).
- `alive_tick_ms` jitter < 5 ms p99 across a 1-minute TX-heavy burst.

---

## W4-10 — Fleet-key provisioning sanity

**Gate.** Flash a fresh image with `lp_keys_secret.h` deleted; confirm the
M7 halts in `setup()` with the OLED "FLEET KEY NOT PROVISIONED" message
and the bridge container exits non-zero.

**Why.** Verifies §D Round 9 — the `#ifndef LIFETRAC_ALLOW_UNCONFIGURED_KEY`
gate is now opt-in, so this test must demonstrate that a developer who
forgets to provision **cannot** silently ship.

**Pre-conditions.**
- §0 setup complete.
- Spare H747 + handheld for the destructive flash.
- Build environment ready to delete `firmware/common/lp_keys_secret.h`.

**Procedure.**
1. **Negative test (handheld).** Delete `lp_keys_secret.h`. Build the
   handheld sketch **without** `-DLIFETRAC_ALLOW_UNCONFIGURED_KEY`.
   Build must fail at compile time (missing header) — that's a pass for
   the build-system half.
2. **Negative test (firmware path).** Restore `lp_keys_secret.h` but with
   all-zero key bytes. Build without the bypass flag. Flash to handheld.
   Power on. Verify OLED shows "FLEET KEY NOT PROVISIONED" within 2 s
   of boot. Verify no LoRa TX occurs (spectrum analyzer shows no 915 MHz
   bursts for 30 s).
3. Repeat step 2 for tractor M7.
4. **Bypass test.** Build the same all-zero image **with**
   `-DLIFETRAC_ALLOW_UNCONFIGURED_KEY`. Confirm boot proceeds (this is
   the CI / dev-loop path — must still work).
5. **Bridge test.** Start base-station container with bridge config
   pointing to a non-existent fleet-key file. Confirm container exits
   with non-zero status code within 5 s and audit log records
   `fleet_key_missing`.

**Pass criteria.**
- Step 1: compile fails.
- Steps 2–3: OLED message present, no LoRa TX, halt holds for 30 s.
- Step 4: boot completes (dev path still works).
- Step 5: container exit code ≠ 0, audit log entry present.

**Artifacts.**
- Photo of OLED for steps 2 and 3.
- Spectrum analyzer screenshot showing no TX.
- `docker logs` capture for step 5.
- `journalctl -u lifetrac-base` capture showing the audit event.

**Failure modes.**
- Boot proceeds in step 2 → §D regression; the
  `#ifndef LIFETRAC_ALLOW_UNCONFIGURED_KEY` gate is not reaching the
  `fleet_key_is_zero()` check. Re-read the firmware sketch around the
  documented Round 9 block and the
  [`.github/workflows/arduino-ci.yml`](../../.github/workflows/arduino-ci.yml)
  compile flags.
- LoRa TX observed in step 2 → halt path is firing too late; confirm
  the check runs **before** `radio.begin()` returns success.

---

## Sign-off

A Wave-4 gate is **closed** when:

1. All numbered runs in its procedure are stored under
   `bench-evidence/W4-XX/`.
2. The `SUMMARY.csv` row is filled out with `pass = true`.
3. Two operators have countersigned `bench-evidence/W4-XX/NOTES.md`.
4. The corresponding checkbox in [`TODO.md`](TODO.md) is flipped to `[x]`
   and the commit message references the evidence directory.

Once W4-01 … W4-10 are all closed:

- §A (CI compile-gate flip from `continue-on-error: true` to blocking) can
  land in [`.github/workflows/arduino-ci.yml`](../../.github/workflows/arduino-ci.yml)
  with a one-line YAML change.
- [`SAFETY_CASE.md`](SAFETY_CASE.md) gets a "Wave 4 evidence package" appendix
  pointing to `bench-evidence/`.
- The Wave-4 status in
  [`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md`](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md)
  flips from "open" to "closed".
