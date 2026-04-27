# LifeTrac v25 — Implementation Readiness Review for Hardware Testing

Date: 2026-04-26
Reviewer: GitHub Copilot
Scope: Everything under `LifeTrac-v25/DESIGN-CONTROLLER/` that is intended to run on hardware, plus integration glue.
Companion document: [2026-04-26_v25_controller_pipeline_example_code_review.md](2026-04-26_v25_controller_pipeline_example_code_review.md) — file-level review of the new example pipeline. This note is the higher-level readiness picture: what exists, what is missing, what would block a bench bring-up.

> Bottom line: **not ready for powered-hydraulics testing**. We are close to a *bench bring-up of the legacy Opta + ESP32 path with LED stand-ins*, and roughly half-built on the new three-tier (handheld → tractor H7/X8 → Opta-as-Modbus-slave → base station) pipeline. The new pipeline has critical safety gaps documented in the companion review, plus several integration blockers and a missing test/CI rig that would make a real powered test risky.

---

## 1. What "ready for hardware testing" means here

Two distinct test campaigns are visible in [TODO.md](../../DESIGN-CONTROLLER/TODO.md):

1. **Legacy v25 path** — single Arduino Opta running [`arduino_opta_controller/lifetrac_v25_controller.ino`](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino), driven by the [ESP32 dual-Qwiic-joystick remote](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino), [Pi web UI](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/raspberry_pi_web_controller/app.py), or DroidPad over BLE. Wire format is JSON-over-MQTT or BLE float vectors.
2. **New three-tier path** — MKR WAN 1310 handheld → Portenta H7/X8 (M7+M4) on the tractor → Opta as Modbus-RTU slave → Portenta X8 base station. Wire format is the binary `lora_proto` frames in [`EXAMPLE_CODE/`](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/).

Both paths are partially implemented and **the two paths use incompatible firmware on the Opta**. We need to decide which path bench tests target before any hardware is energized.

---

## 2. Readiness summary by subsystem

| Subsystem | Path | Code present | Compiles* | Bench-test ready | Hardware-test ready |
|---|---|---|---|---|---|
| Opta controller — JSON/MQTT/BLE | Legacy | Yes (~700 LOC) | Untested in CI | LED stand-ins yes | **No** — see §4 |
| Opta — Modbus slave | New | Yes, draft (`opta_modbus_slave.ino`) | Untested | Pseudocode shims | **No** — see §4 |
| ESP32 remote (Qwiic joysticks) | Legacy | Yes (~365 LOC) | Untested in CI | Likely | Conditional |
| Pi web controller (Flask + libcamera) | Legacy | Yes (`app.py` + templates) | Untested | Likely | Conditional |
| ROS2 bridge (`lifetrac_mqtt_bridge`) | Legacy | Yes, package skeleton | Untested | Likely | Optional, not on critical path |
| Handheld MKR WAN 1310 firmware | New | Draft (`handheld.ino`) | Untested | No — uncalibrated | **No** |
| Tractor M7 firmware | New | Draft (`tractor_m7.ino`) | Untested | No | **No** — see §4 |
| Tractor M4 firmware | New | Draft (`tractor_m4.cpp`) | Untested | No — watchdog will trip | **No** |
| Base station LoRa↔MQTT bridge | New | Draft (`lora_bridge.py`) | Static only | Partial | No |
| Base station web UI + gamepad | New | Draft (`web_ui.py`, `web/`) | Static only | Partial | No |
| `lora_proto` shared library | New | Draft (C + Python) | No build system | No | **No** — crypto stub |
| AES-128-GCM | All | C is no-op stub; Python real | n/a | No interop | **No** |
| Tests / CI | All | Only `mqtt_test.py` and `test_system.sh` | n/a | None for `lora_proto` | **No** |

\* "Compiles" means there is no automation in this repo that proves the firmware builds. Arduino-CLI CI is documented in [ARDUINO_CI.md](../../DESIGN-CONTROLLER/ARDUINO_CI.md) but no GitHub Actions workflow or pre-commit hook is wired up to it.

---

## 3. Critical blockers (must fix before any powered test)

These are the items that would either prevent the test from running or actively endanger hardware/people. Several were found in the companion file review and are repeated here for completeness; see that document for line numbers and exact recommendations.

1. **Two Opta firmwares, one tractor.** [`arduino_opta_controller/lifetrac_v25_controller.ino`](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino) is a JSON-over-MQTT/BLE consumer. [`EXAMPLE_CODE/opta_modbus_slave/opta_modbus_slave.ino`](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/opta_modbus_slave/opta_modbus_slave.ino) is a Modbus slave with a *different* register map than the master in `tractor_m7.ino` and *different* than the canonical map in [TRACTOR_NODE.md](../../DESIGN-CONTROLLER/TRACTOR_NODE.md). The first bench bring-up will silently disagree on the wire. **Pick one canonical Opta firmware and one register map before flashing the Opta on a hydraulic rig.**
2. **Tractor M4 watchdog will latch the safety GPIO low.** M4 expects `SHARED->alive_tick_ms` from M7; M7 never writes it. The PSR safety relay would drop on every boot. Companion review §"Critical: M4 watchdog will trip…".
3. **Opta watchdog can be defeated by ordinary Modbus traffic.** The `opta_modbus_slave.ino` loop scans `REG_LAST_ERROR` on every poll and side-effects `g_last_alive_change_ms`, so a master polling unrelated registers keeps the watchdog fresh even if the alive tick stops. Companion review §"Critical: Opta watchdog can be accidentally refreshed…".
4. **Stale-control acceptance.** `tractor_m7.ino`'s `pick_active_source()` only checks heartbeats and `apply_control()` does not require a fresh valid `ControlFrame`. Lose the control stream while heartbeats continue and the tractor will keep applying the last commanded vector. Companion review §"High: Tractor can continue applying stale or invalid control frames".
5. **E-stop is unwired end-to-end.** The web UI and gamepad path send an E-stop, the bridge forwards it, and the tractor handler is a `// TODO`. Both legacy and new pipelines need a real latched E-stop *before* energizing hydraulics. Companion review §"High: E-stop is not wired through…".
6. **`all_coils_off()` does not actually turn off all coils.** It clears four onboard relays only; D1608S relays 4–7 and the A0602 0–10 V flow setpoints are TODOs. A "safe state" today still energizes the boom/bucket coils and leaves flow at whatever was last commanded. Companion review §"High: Safety-off paths do not actually zero all outputs yet".
7. **Crypto is intentionally broken for interop.** Python uses real AES-128-GCM with an all-zero key; the C side links a no-op stub that copies plaintext through. Even if `LIFETRAC_ALLOW_STUB_CRYPTO` is set for sim builds, a tractor firmware built with the stub cannot decrypt frames from the Python bridge, and an all-zero key must never reach a hardware build.
8. **`ControlFrame` size mismatch (15 vs 16 bytes documented).** Internally consistent between C and Python today, but documents/comments/airtime budgets all say 16. Pick one and update everything before tests. Companion review §"Critical: ControlFrame size…".
9. **Handheld nonce/sequence mismatch.** AES-GCM nonce is one ahead of the embedded sequence number, breaking the documented nonce shape and any future replay/audit checks. Companion review §"Critical: Handheld AES-GCM nonce sequence…".

---

## 4. Major missing pieces

These are not necessarily safety bugs, but they would stop a hardware test from being meaningful or repeatable.

### 4.1 Build / toolchain

- **No build system for `lora_proto/`.** Just a header, a stub `.c`, and a Python module. There is no `library.properties`, no PlatformIO `library.json`, no CMake or `Makefile`, no Arduino-CLI invocation in CI. The example `.ino` files include `lora_proto.h` but nothing tells the Arduino build where to find it. Add a `library.properties` (Arduino) **or** restructure as a PlatformIO library, and add an Arduino-CLI compile job per board target (MKR WAN 1310, Portenta H7 M7, Portenta H7 M4, Opta).
- **No CI workflow.** [ARDUINO_CI.md](../../DESIGN-CONTROLLER/ARDUINO_CI.md) describes the intended setup, but there is no `.github/workflows/*.yml` invoking `arduino-cli compile` for any sketch. Until that exists, every "draft compiles" claim in IMPLEMENTATION_SUMMARY-style docs is unverified.
- **No host-side unit tests for `lora_proto`.** CRC, KISS escaping, frame sizes, and Python/C packing equivalence are all unguarded. These tests can run with no hardware in seconds and would have caught the 15-vs-16-byte mismatch.
- **`arduino_libraries.txt` is unpinned.** Library *names* only; no versions. The MbedTLS / ArduinoBLE / OptaController / RadioLib APIs all have moved between minor versions. Pin every library used by both the legacy and new pipeline before bench tests so two operators get the same binary.
- **Python deps similarly unpinned.** [`base_station/requirements.txt`](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/requirements.txt) and [`raspberry_pi_web_controller/requirements.txt`](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/raspberry_pi_web_controller/requirements.txt) — confirm versions are pinned with `==`, not loose.

### 4.2 Configuration that ships with placeholders

- WiFi SSID/password literally `"YOUR_WIFI_SSID"` / `"YOUR_WIFI_PASSWORD"` in both [`lifetrac_v25_controller.ino`](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino) and [`lifetrac_v25_remote.ino`](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino). MQTT user `lifetrac` / password `lifetrac_pass` shipped in source. MQTT broker hardcoded to `192.168.1.100`. Move secrets to a separate, gitignored `secrets.h` (or `.env` for Python) and fail loudly at boot if the file is missing.
- Pre-shared LoRa key is `0x00 × 16` in `handheld.ino`, `tractor_m7.ino`, and `lora_bridge.py`. Key provisioning utility (`provision.py`) is a TODO in [TODO.md Phase 2](../../DESIGN-CONTROLLER/TODO.md).
- Pin maps in [`opta_modbus_slave.ino`](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/opta_modbus_slave/opta_modbus_slave.ino) are commented "placeholder — verify against actual Opta + expansion pinout". Cross-check against the wiring diagram and your real Opta + D1608S + A0602 silkscreen before the first energize.
- Legacy `FLOW_CONTROL_PIN_1 = 2` / `_PIN_2 = 3` collide numerically with the digital valve pins `D2` / `D3` in the same sketch. The Opta library distinguishes these by API (`OptaController.analogWriteMode` vs `digitalWrite`), but the duplicate integer literals are confusing — verify with a multimeter that O2/O3 on the A0602 are actually being driven and not D2/D3 on the relays.

### 4.3 Safety-critical functionality not yet implemented

In addition to the eight critical items in §3:

- **Soft-start / ramp on flow setpoints.** Both legacy and new code write 4-20 mA / 0-10 V step changes directly. Step changes on hydraulic flow can shock the system. Add a slew-rate limiter on `REG_FLOW_SP_*` writes.
- **Joystick calibration / deadband on the new handheld.** Handheld currently reads raw ADC. A center-zero calibration pass at boot, plus per-axis deadband, is required before hands-on testing.
- **Stuck-button / stuck-stick detection.** Neither path detects a held control that exceeds plausible duration (a wedged START or full-forward stick).
- **Watchdog on the M7 itself.** TODO.md Phase 4 calls for an internal watchdog "M7 watchdog hits → reset". `tractor_m7.ino` does not arm `IWDG_HandleTypeDef` or its Mbed equivalent.
- **Engine-kill arming policy.** `opta_modbus_slave.ino` has a `REG_ARM_ESTOP` write that latches engine kill, but no policy doc says when the operator can clear it. Define and document the clear-from-fault flow.
- **Take-control arbitration.** Spec'd in [LORA_PROTOCOL.md] and TODO.md Phase 3, not implemented in `handheld.ino`.

### 4.4 Missing tooling / fixtures

- **No bench Modbus simulator** to exercise `tractor_m7.ino` without a real Opta. Recommended: a small Python script using `pymodbus` + a USB-RS485 dongle that pretends to be the Opta slave with the canonical register map.
- **No LoRa loopback fixture.** Two MKRs or two Portentas talking on a bench would catch the crypto stub interop break, the 15-vs-16-byte issue, and the nonce skew immediately.
- **No latency / failsafe test plan.** [TODO.md] Phase 1 lists "verify three nodes can hear each other" but not the timing-budget validation in [LORA_PROTOCOL.md] (200 ms control RTT, 500 ms failsafe).
- **No emergency-stop bench rig.** Need a 12 V solenoid stand-in (LED + buzzer is fine) wired through the PSR contact so we can prove the safety drop-out is actually happening before any hydraulic test.
- **No FCC EIRP measurement plan.** The hardware BOM lists a spectrum analyzer rental but no procedure or pass/fail thresholds.

### 4.5 Documentation drift

- The legacy [`CODE_REVIEW.md`](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/CODE_REVIEW.md) and [`CODE_REVIEW_FIXES.md`](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/CODE_REVIEW_FIXES.md) cover the JSON path; the companion review covers the new binary path. We do not yet have a single "as-built v25 firmware status" page operators can read.
- [`MODE_SWITCH_WIRING.md`](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/MODE_SWITCH_WIRING.md) and [`IMPLEMENTATION_SUMMARY.md`](../../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/IMPLEMENTATION_SUMMARY.md) describe the legacy MQTT/OFF/BLE switch on `D9`/`D10`. The new `opta_modbus_slave.ino` reuses `D5`/`D6` for `MODE_SW_A`/`B` for a different purpose. Reconcile the pin map before wiring.
- [TODO.md Phase 0](../../DESIGN-CONTROLLER/TODO.md) is a procurement list, not a readiness gate. Mark which Phase 0 items are *blockers* for the next bench session vs nice-to-have.

---

## 5. Likely pitfalls during a real bench session

Things that the code will not stop you from doing wrong:

1. **Flashing M7 without M4** (or vice versa) — the dual-core build artifacts are easy to mismatch on the H7/X8. Document the exact `arduino-cli compile` invocation for both cores and which order to flash.
2. **Powering the Opta from the same 12 V rail as the valve coils.** Solenoid coils kick on de-energize; without a flyback diode and a separate analog 12 V for the Opta, the MCU will reset under load. The HARDWARE_BOM calls out fuses and EMI ferrites; verify those are installed on the bench rig too, not just deferred to the field harness.
3. **RS-485 bus without termination / biasing.** [TODO.md Phase 0] orders "120 Ω termination resistors (×2)" but the bring-up steps don't say to install them. A floating idle bus produces phantom Modbus frames.
4. **BLE security off in DroidPad mode.** `CODE_REVIEW.md` correctly notes BLE is unauthenticated. Anyone in range can drive the tractor. Either pair, or only run BLE in a Faraday cage / tethered bench.
5. **MQTT broker accepting anonymous in dev.** [`config/mosquitto.conf`](../../DESIGN-CONTROLLER/config/mosquitto.conf) — verify `allow_anonymous false` is set before the broker is reachable from anything other than localhost.
6. **Camera process leak.** `app.py`'s `generate_camera_frames` spawns `libcamera-vid` as a subprocess; if the Flask request handler dies the subprocess can be orphaned. Add a `try/finally` that kills the process on shutdown and on disconnect.
7. **PubSubClient default packet size (128 B).** The legacy controller's MQTT JSON status messages exceed this once you add fields. Call `client.setBufferSize(...)` early, or status publishes will silently drop.
8. **`DynamicJsonDocument` deprecated.** ArduinoJson v7+ replaced it with `JsonDocument`. If you pin to v7, the legacy sketches stop compiling. This is another reason to pin library versions before bench tests.
9. **Browser gamepad polling at 50 Hz hitting `/api/estop` on every poll.** Companion review §"Medium: Gamepad START can spam E-stop…". The first time an operator holds START during a real test you will queue dozens of HTTP requests at exactly the worst moment.

---

## 6. What would unblock a first powered bench test

The minimum viable list — in order — to get from "draft code" to "we can energize one hydraulic valve on a bench manifold":

1. **Decide the path.** Legacy JSON Opta firmware *or* new Modbus-slave firmware. Park the other.
2. **Fix the chosen Opta firmware's safety paths**: complete `all_coils_off()`, complete `apply_valve_register()` for all eight channels (or all four if legacy), implement E-stop latch, fix the watchdog refresh logic if Modbus, add slew-rate limiting on flow setpoints.
3. **Pin all library versions** (Arduino + Python) and move secrets/keys out of source.
4. **Stand up Arduino-CLI CI** for at least the chosen Opta firmware and one master/remote firmware. No compile-on-laptop-only runs into a real test.
5. **Add a Modbus or BLE/MQTT bench fixture** that can drive the Opta with synthetic inputs and verify expected outputs with LEDs only.
6. **Wire and prove the PSR drop-out** with LEDs first, then with one solenoid valve open-loop on a low-pressure bench manifold.
7. **Write and follow a written test card** — power-up checklist, step-by-step input/expected-output, abort criteria. This is missing today.

For the *new* three-tier path specifically, additionally:

8. Fix the nine companion-review criticals (frame size, nonce skew, M4 alive-tick, watchdog refresh, register map sync, stale-control acceptance, E-stop wiring, all-off completeness, telemetry nonce determinism).
9. Add `lora_proto` host unit tests and a two-node loopback test before Opta is connected.
10. Replace the crypto stub or build with `LIFETRAC_ALLOW_STUB_CRYPTO` only on bench radios that are physically isolated.

---

## 7. Strengths worth preserving

- The architecture documents (`ARCHITECTURE.md`, `LORA_PROTOCOL.md`, `TRACTOR_NODE.md`, `HARDWARE_BOM.md`) are unusually detailed and internally cross-linked. Most of the gaps above are implementation drift away from those documents, not missing design.
- Both pipelines default to safe modes when their hardware is missing (BLE on missing mode switch; safety-state latched on watchdog miss, in principle).
- The repo separates RESEARCH-CONTROLLER from production-intent firmware, which makes it easy to keep the new pipeline visible as "draft" until ready.
- The Pi web controller and ROS2 bridge are scoped narrowly and are not on the critical path for a first powered test, so they can stay un-touched while the Opta + handheld + tractor path is hardened.

---

## 8. Recommended next concrete actions (one sprint)

- [ ] Decide and document: legacy JSON path vs new Modbus path for first bench test.
- [ ] Resolve the nine companion-review criticals on the chosen path.
- [ ] Add `library.properties` to `lora_proto/` + Arduino-CLI CI workflow that compiles the chosen Opta sketch and the chosen master sketch.
- [ ] Add host unit tests for `lora_proto` (CRC, KISS, frame sizes, C/Python equivalence) — pure Python, runs in CI.
- [ ] Move secrets out of source; pin all library versions.
- [ ] Build a Modbus (or MQTT) bench fixture and run the chosen Opta sketch against LEDs end-to-end.
- [ ] Wire and verify PSR safety-relay drop-out with LED loads.
- [ ] Write a one-page test card for the first hydraulic-energize session.
- [ ] Only then connect a single bench manifold, low-pressure, single valve at a time.
