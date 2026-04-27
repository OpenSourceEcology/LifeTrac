# Controller Hardware Testing Readiness Review — LifeTrac v25
Date: 2026-04-26
Reviewer: GitHub Copilot

## Executive Verdict
Overall status: RED (not ready for hardware testing yet).

Readiness estimate:
- End-to-end hydraulic hardware test readiness: ~10%
- Bench software/protocol validation readiness: ~25%

Reason: the current primary path is still design/docs + draft prototype code. There is no active, build-validated implementation tree in the locations defined by the main controller roadmap.

## Findings (Ordered by Severity)

### Critical 1: Primary implementation tree is missing from active controller path
Evidence:
- `DESIGN-CONTROLLER/TODO.md` defines expected active implementation locations such as `firmware/handheld_mkr`, `firmware/tractor_h7`, `firmware/tractor_opta`, `firmware/base_h7`, and `base_station` (line 108).
- Workspace search currently returns no files under `LifeTrac-v25/DESIGN-CONTROLLER/firmware/**` and no files under `LifeTrac-v25/DESIGN-CONTROLLER/base_station/**`.

Impact:
- There is no canonical active firmware/web implementation to compile, flash, and test as the "official" v25 controller stack.
- Hardware testing cannot be treated as implementation-ready while core target directories are absent.

### Critical 2: Current code with most implementation detail is explicitly archived/superseded
Evidence:
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/README.md` says this folder is "superseded" (line 3), "Nothing here is the active design" (line 5), and archived/superseded by the main design docs (line 55).
- `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/EXAMPLE_CODE/README.md` states the code is "study and review only" and "None of this code has been compiled, flashed, or tested" (line 3).

Impact:
- The code that appears closest to implementation is intentionally non-production and non-validated.
- Treating it as hardware-test-ready would bypass the project's own architecture boundary.

### Critical 3: Controller roadmap shows no completed execution tasks in the main TODO
Evidence:
- `DESIGN-CONTROLLER/TODO.md` contains many unchecked items (`- [ ]`) beginning in Phase 0 onward (e.g., lines 13-302 in sampled results).
- Search found no checked items (`- [x]` or `- [X]`) in `DESIGN-CONTROLLER/TODO.md`.

Impact:
- Bench bring-up, protocol implementation, safety wiring/tests, and integration verification are still formally incomplete.
- Hardware test risk remains high due to unclosed prerequisites.

### Critical 4: CI currently points to non-existent sketch paths
Evidence:
- `.github/workflows/arduino-ci.yml` references:
  - `LifeTrac-v25/DESIGN-CONTROLLER/arduino_opta_controller/**` (lines 6, 12, 44, 74)
  - `LifeTrac-v25/DESIGN-CONTROLLER/esp32_remote_control/**` (lines 7, 13, 112, 142)
- Those directories do not exist in current workspace layout; actual sketches are under `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/...`.
- `DESIGN-CONTROLLER/ARDUINO_CI.md` repeats the same outdated locations (lines 8-9, 17, 22, 86).

Impact:
- Automated compile validation is effectively disconnected from actual source files.
- Regressions can slip in without CI signal, which is a major hardware-test blocker.

### High 5: Crypto path remains draft/stub-keyed and not field-ready
Evidence:
- `RESEARCH-CONTROLLER/EXAMPLE_CODE/lora_proto/crypto_stub.c` is a placeholder AES-GCM implementation (line 1), marked draft (line 3), explicitly says replace before field use (line 5), and uses a stub plaintext copy/tag behavior (line 34).
- All-zero fleet keys are still present in examples:
  - `handheld_mkr/handheld.ino` line 33
  - `tractor_h7/tractor_m7.ino` line 91
  - `base_station/lora_bridge.py` line 40

Impact:
- Security posture is not acceptable for field operation.
- Even for bench tests, security behavior may not represent final runtime behavior.

### High 6: Remaining known TODOs in the example control pipeline affect control quality and completeness
Evidence:
- `RESEARCH-CONTROLLER/EXAMPLE_CODE/tractor_h7/tractor_m7.ino` keeps TODO for deadband/ramp/dual-flow split in core control mapping (line 173).
- `RESEARCH-CONTROLLER/EXAMPLE_CODE/opta_modbus_slave/opta_modbus_slave.ino` still contains placeholder pin map and expansion-bus shims (lines 69, 96), plus TODO for aux relays/engine-kill arm bit handling (line 185).
- `RESEARCH-CONTROLLER/EXAMPLE_CODE/CODE_REVIEW_FIXES.md` explicitly lists unresolved items under "Not addressed in this pass" (line 217), including placeholder expansion shims and differential-turn drive math still TODO (lines 223, 226, 229).

Impact:
- Command shaping and final I/O mapping remain incomplete for realistic hydraulic behavior.
- Bench results could be misleading versus expected production behavior.

### High 7: Legacy Raspberry Pi web controller contains a stale-command hazard in keyboard mode
Evidence:
- `RESEARCH-CONTROLLER/raspberry_pi_web_controller/static/js/controller.js` updates `currentControl` when keys are active (lines 169-173), but when key set becomes empty (line 177), it does not zero `currentControl`.
- Same file transmits commands continuously at 20 Hz (`COMMAND_INTERVAL = 50`, line 33; send loop lines 322-324).

Impact:
- If keyboard keys are released after movement input, last non-zero command can continue being sent.
- This is a potential runaway-movement behavior if this archived stack is reused for bench/hardware control.

### Medium 8: Test coverage remains largely manual for archived stack
Evidence:
- `RESEARCH-CONTROLLER/raspberry_pi_web_controller/IMPLEMENTATION_SUMMARY.md` marks Unit Tests as not implemented (line 271) and Integration Tests as not implemented (line 277).

Impact:
- Less confidence in edge-case behavior before connecting to hardware.

## Readiness Gap Summary
The project has strong architecture/specification documentation, but implementation readiness is currently blocked by:
1. Missing canonical active code tree under the main design path.
2. CI not validating actual sketch locations.
3. Example pipeline still draft and partially placeholder-based.
4. Safety/completion gaps in fallback archived web-controller path.

## Second-Pass Code-Level Findings

This addendum reviews the current `RESEARCH-CONTROLLER/EXAMPLE_CODE` implementation after the previous pipeline review fixes. Several earlier critical issues were corrected in-tree, but the controller is still not ready to connect to live hydraulic hardware.

### Critical 9: Base station radio co-MCU firmware is missing
Evidence:
- `RESEARCH-CONTROLLER/EXAMPLE_CODE/base_station/lora_bridge.py` says the X8 H747 co-MCU runs `base.ino` as the LoRa UART wrapper.
- Workspace search found no `base.ino`, no `base_h7` firmware folder, and no equivalent H747 base-station sketch.

Impact:
- The Python bridge has no actual radio-side peer to talk to on the base station.
- Base-station web control cannot be hardware-tested end-to-end until the H747 LoRa bridge firmware exists and is build-validated.

Recommendation:
- Add `firmware/base_h7` or an equivalent active base-station sketch that moves KISS-framed bytes between the X8 UART and the SX1276 radio.
- Include it in PlatformIO/Arduino CLI CI before any base-station control test.

### Critical 10: Base-station E-stop can be rejected as a replayed command
Evidence:
- `base_station/lora_bridge.py` builds `/cmd/estop` with a plaintext command header sequence number of `0` every time.
- `tractor_h7/tractor_m7.ino` rejects frames from a source when `hdr.sequence_num` does not advance.
- Normal base control frames use the `web_ui.py` websocket sequence. After any base control frame advances the tractor's `SRC_BASE` sequence state, a later E-stop with header sequence `0` can be dropped before the command handler runs.

Impact:
- The base-station E-stop button may work only as the first command after boot, then fail silently after prior base control traffic.
- This is a hard blocker for any hardware test that presents the web UI E-stop as a safety control.

Recommendation:
- Make one component own the base source sequence. The bridge should stamp the final sequence into every `SRC_BASE` `ControlFrame` and `CommandFrame`, then use the same sequence in the AES nonce.
- Add an explicit E-stop replay exception only if it is still authenticated and idempotent, but do not rely on a constant sequence number.

### Critical 11: Tractor LoRa receive path lacks a ciphertext/plaintext length cap before decrypt
Evidence:
- `tractor_h7/tractor_m7.ino` allocates `uint8_t pt[160]` in `process_air_frame()`.
- Incoming KISS frames can be up to the decoder buffer size, and `lp_decrypt()` is called before checking whether decrypted plaintext fits in `pt`.
- The current C crypto stub copies `ct_len - 16` bytes into the supplied plaintext buffer.

Impact:
- An oversized or malformed radio frame can overflow the stack buffer in the current bench/sim crypto build.
- Even with real AES-GCM, the receive path should reject over-sized ciphertext before passing a fixed output buffer to the decryptor.

Recommendation:
- Add a guard such as `if (len > 12 + sizeof(pt) + 16) return;` before `lp_decrypt()`.
- Make the real `lp_decrypt()` API take an output buffer length and return the actual plaintext length.

### Critical 12: Handheld wireless E-stop path is not implemented
Evidence:
- `HANDHELD_REMOTE.md` describes a handheld E-stop input that sets `FLAG_ESTOP_ARMED` or sends an E-stop condition.
- `handheld_mkr/handheld.ino` has no `PIN_ESTOP`, never sets `FLAG_ESTOP_ARMED`, and does not send a `CMD_ESTOP` command.
- `tractor_h7/tractor_m7.ino` handles `CMD_ESTOP`, but does not interpret `FLAG_ESTOP_ARMED` in `ControlFrame` or `HeartbeatFrame` as a safety latch.

Impact:
- The handheld cannot currently command an explicit wireless E-stop as documented.
- Link-loss failsafe still exists, and the Opta has an external E-stop loop input, but the operator handheld E-stop behavior is incomplete.

Recommendation:
- Decide the canonical wireless E-stop semantics: command frame, control flag, heartbeat flag, or all of the above.
- Implement the handheld input, tractor latch, and confirmation/clear flow, then bench-test it before any hydraulic motion test.

### High 13: Command opcode definitions are split and inconsistent
Evidence:
- `lora_proto.h` defines `CMD_REQ_CONTROL = 0x02` and `CMD_REKEY = 0x03`.
- `LORA_PROTOCOL.md` defines `CMD_CLEAR_ESTOP = 0x02`, `CMD_CAMERA_SELECT = 0x03`, `CMD_CAMERA_QUALITY = 0x04`, `CMD_PLAN_COMMIT = 0x10`, and `CMD_LINK_HINT = 0x20`.
- `tractor_m7.ino` locally defines `CMD_CLEAR_ESTOP = 0x02` and `CMD_REKEY = 0x10`, while `lora_bridge.py` sends camera select as opcode `0x03`.

Impact:
- Different components can compile against different meanings for the same opcode.
- Camera select is sent by the UI/bridge but ignored by the tractor M7 command handler.
- Clear E-stop lacks the confirmation token required by the protocol document.

Recommendation:
- Move all command opcodes into the shared protocol header and mirror the same table in Python from one generated or tested source.
- Add command-frame decode tests for E-stop, clear E-stop, and camera select before adding more commands.

### High 14: Opta expansion outputs and analog I/O are still placeholders
Evidence:
- `opta_modbus_slave.ino` has placeholder `d1608s_set()`, `a0602_write_mv()`, and `a0602_read()` shims that do nothing.
- The pin map is marked placeholder and the boot self-test only cycles onboard relays R1-R4.
- `REG_AUX_OUTPUTS` still has a TODO for aux relays and engine-kill arm handling.

Impact:
- Boom, bucket, dual flow-valve outputs, and telemetry analog inputs will not operate on real Opta expansion hardware yet.
- A test using this sketch would only exercise the four onboard relays, not the full hydraulic control surface.

Recommendation:
- Replace the shims with actual `OptaController` expansion calls and verify D1608S/A0602 enumeration.
- Build a relay/LED and multimeter bench test that proves all 8 coil outputs and both 0-10 V outputs move to commanded and safe states.

### High 15: Core drive math is still simplified and incomplete
Evidence:
- `tractor_m7.ino` maps only `axis_lh_y` to both left and right drive directions.
- The code still marks deadband, ramping, differential turn, and dual-flow split as TODO.
- `CODE_REVIEW_FIXES.md` explicitly lists differential-turn drive math as not addressed.

Impact:
- Bench motion behavior will not match the expected tank-steering/differential-turn design.
- Flow-valve behavior will not match the dual-flow hydraulic configuration.

Recommendation:
- Port the proven deadband/ramp/flow-split logic from the archived Opta controller into the active M7/Opta split, with host tests for the mapping table.

### Medium 16: M4 safety watchdog comments overstate what the code detects
Evidence:
- `tractor_m4.cpp` claims the M4 watchdog catches a wedged Modbus link.
- The actual M4 code only watches `SHARED->alive_tick_ms` from M7 and pulls `PIN_SAFETY_ALIVE` low if M7 stops ticking.
- M7 stamps `alive_tick_ms` before polling radio, UART, or Modbus, so Modbus failures do not trip the M4.

Impact:
- The overall design still has the Opta watchdog for Modbus write loss, but the M4 code does not provide the stated Modbus-link coverage.
- Misunderstanding this could lead to incomplete safety tests.

Recommendation:
- Update comments and test plans, or add a shared `last_successful_modbus_write_ms` signal that the M4 can monitor if independent Modbus supervision is required.

### Medium 17: Telemetry frame size limits are inconsistent
Evidence:
- `LORA_PROTOCOL.md` says telemetry payload length is `0..120` and frame size is `9-128` bytes, which is internally inconsistent because `5 + 2 + 120 + 2 = 129`.
- `lora_bridge.py` allows `TELEM_MAX_PAYLOAD = 128`.
- `tractor_m7.ino` limits payloads to `sizeof(TelemetryFrame.payload) - 2`, effectively 118 bytes, because it stores the CRC inside the fixed payload array.

Impact:
- Small GPS/IMU/hydraulic packets are fine, but larger telemetry/video-thumbnail packets can disagree across implementations.
- This will cause hard-to-debug drops when larger payload classes are tested.

Recommendation:
- Freeze the true max payload and frame length, then add shared tests for boundary sizes.
- Prefer an explicit encode buffer for variable telemetry frames instead of storing CRC beyond or inside a fixed payload field.

### Medium 18: Local verification is currently blocked by missing tooling
Evidence:
- Python environment selection failed in this workspace, so Python syntax/runtime checks were not run.
- Local checks found no `arduino-cli`, `pio`, or `gcc` on PATH.
- VS Code diagnostics reported no visible errors for the workflow file checked, but folder-wide firmware diagnostics were not available.

Impact:
- This review remains static. It should not be treated as a compile or runtime validation pass.

Recommendation:
- Install/select the project Python environment and firmware build tooling.
- Add `py_compile`/pytest for Python services, host-side C protocol tests, and Arduino/PlatformIO compile checks for every firmware target.

## Minimum Exit Criteria Before Hardware Test

### Phase A: Make Build Path Real (required)
- Create/populate canonical active implementation directories under `DESIGN-CONTROLLER` (`firmware/*`, `base_station/*`) or officially re-designate `RESEARCH-CONTROLLER/EXAMPLE_CODE` as active.
- Fix `.github/workflows/arduino-ci.yml` and `DESIGN-CONTROLLER/ARDUINO_CI.md` to point to real paths.
- Enforce successful CI compile for all firmware targets.

### Phase B: Safety-Critical Control Readiness (required)
- Resolve remaining control-mapping TODOs (deadband/ramp/dual-flow split).
- Replace Opta expansion placeholders with actual OptaController calls and validate all coil/AO outputs.
- Close stale-command hazard in keyboard path if legacy stack will be used.

### Phase C: Security/Protocol Readiness (required for field; recommended for bench)
- Replace crypto stub with real AES-GCM on embedded side, or explicitly gate a bench-only plaintext mode across all nodes.
- Remove all-zero keys and implement provisioning path used in actual tests.

### Phase D: Bench Validation Gates (required before live hydraulics)
- Deterministic bench tests for watchdog drop-to-safe behavior.
- E-stop latch/clear verification.
- End-to-end protocol interoperability tests (control, heartbeat, telemetry).
- Soak test and reconnect/fault-injection tests.

## Bottom Line
You are not close to safe live hardware testing yet on the primary v25 controller stack. The quickest path forward is to first establish a single active implementation tree + functioning CI, then close the remaining safety/control TODOs before any powered hydraulic test.