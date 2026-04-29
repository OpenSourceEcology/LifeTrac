# LifeTrac v25 Controller Code and Pipeline Review - GitHub Copilot v1.0

Date: 2026-04-28
Reviewer: GitHub Copilot
Review version: v1.0

## Scope

Reviewed the active v25 controller tree under `DESIGN-CONTROLLER/`:

- Base station Python stack: `base_station/`, `Dockerfile`, `docker-compose.yml`, and tests.
- Active embedded firmware: `firmware/common/lora_proto/`, `firmware/handheld_mkr/`, `firmware/tractor_h7/`, `firmware/tractor_opta/`, and tractor X8 services.
- Pipeline docs and CI: `ARDUINO_CI.md`, `arduino_libraries.txt`, and `.github/workflows/arduino-ci.yml`.
- Archived `RESEARCH-CONTROLLER/` code was treated as reference only.

## Executive Summary

The Python protocol and image-pipeline unit tests are in reasonably good shape, and the current protocol tests now lock important airtime assumptions such as encrypted control at SF7/BW250 and image fragments at SF7/BW500.

The blocking issues are mostly integration and pipeline problems:

1. The compose stack will not start correctly as written: `lora_bridge.py` requires a positional serial port, `docker-compose.yml` does not pass it, and both `web_ui.py` and `lora_bridge.py` ignore the compose `LIFETRAC_MQTT_HOST=mosquitto` environment.
2. The documented Arduino compile commands fail immediately because the active sketch folders do not contain primary `.ino` files matching their folder names.
3. The active GitHub Actions workflow does not compile firmware at all, and its Python job does not install the base-station requirements, so web/auth tests can be skipped while CI stays green.
4. The control radio boot profile still initializes at SF7/BW125 while the current decision record and tests require SF7/BW250 for the 20 Hz encrypted control path.
5. Fleet keys are still all-zero placeholders in both Python and firmware, with no startup refusal outside docs.
6. The real firmware AES-GCM decrypt wrapper and the live firmware callers disagree about whether `ct_len` includes the 16-byte tag, so a production crypto build will reject valid frames or read past the encrypted buffer.

Do not treat the controller stack as hardware-test ready until the compose/startup path, Arduino sketch layout, real firmware compile gate, boot PHY, and key provisioning guard are fixed.

## Verification Performed

Commands run locally on Windows:

```powershell
cd c:\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\base_station
C:/Users/dorkm/AppData/Local/Programs/Python/Python313/python.exe -m unittest discover -s tests
```

Result: `Ran 88 tests in 0.558s - OK (skipped=2)`.

Verbose rerun showed the skipped modules:

- `test_web_ui_accel` skipped: `paho-mqtt + fastapi required for accel UI tests`
- `test_web_ui_auth` skipped: `paho-mqtt + fastapi required for web_ui auth tests`

```powershell
cd c:\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER
C:/Users/dorkm/AppData/Local/Programs/Python/Python313/python.exe -m compileall -q base_station firmware/tractor_x8 tools
```

Result: no output; Python syntax compile passed.

```powershell
arduino-cli compile --fqbn arduino:samd:mkrwan1310 "c:\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\firmware\handheld_mkr"
```

Result: failed before dependency resolution:

```text
Can't open sketch: main file missing from sketch: C:\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\firmware\handheld_mkr\handheld_mkr.ino
```

```powershell
arduino-cli compile --fqbn arduino:mbed_opta:opta "c:\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\firmware\tractor_opta"
```

Result: failed before dependency resolution:

```text
Can't open sketch: main file missing from sketch: C:\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\firmware\tractor_opta\tractor_opta.ino
```

`arduino-cli core list` showed only `arduino:avr` and `arduino:mbed_opta` installed locally; SAMD, Portenta, RadioLib, and Opta expansion libraries were not installed in this workstation environment.

VS Code diagnostics over `LifeTrac-v25/DESIGN-CONTROLLER` reported no current editor problems.

## Findings

### 1. Blocker: compose startup is broken for both MQTT and LoRa bridge arguments

Evidence:

- [../DESIGN-CONTROLLER/docker-compose.yml](../DESIGN-CONTROLLER/docker-compose.yml#L43-L48) sets `LIFETRAC_MQTT_HOST: mosquitto` and `LIFETRAC_LORA_DEVICE: /dev/ttyACM0`, but launches `lora_bridge.py` without arguments.
- [../DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L390-L393) requires a positional `port` and defaults `--mqtt` to `localhost`; it does not read `LIFETRAC_LORA_DEVICE` or `LIFETRAC_MQTT_HOST`.
- [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L113-L114) hardcodes `mqtt_client.connect("localhost", 1883)` while the broker is a sibling compose service named `mosquitto`.

Impact:

- `lora_bridge` exits immediately with argparse usage error in compose.
- `web_ui` tries to connect to a broker inside its own container instead of the `mosquitto` container.
- Even if the serial argument is added, `lora_bridge` will also target `localhost` unless `--mqtt mosquitto` is passed.

Recommended fix:

- In `lora_bridge.py`, default `port` from `os.environ.get("LIFETRAC_LORA_DEVICE", "/dev/ttyACM0")` and `--mqtt` from `LIFETRAC_MQTT_HOST`.
- In `web_ui.py`, use `os.environ.get("LIFETRAC_MQTT_HOST", "localhost")`.
- In `docker-compose.yml`, pass the device explicitly or rely on the env default consistently.
- Add a small compose/static-startup test that asserts each command line can parse and that every service uses the configured MQTT host.

### 2. Blocker: active Arduino sketches cannot be compiled by folder

Evidence:

- [../DESIGN-CONTROLLER/ARDUINO_CI.md](../DESIGN-CONTROLLER/ARDUINO_CI.md#L34-L35) documents compiling `firmware/handheld_mkr` and `firmware/tractor_opta` by sketch folder.
- [../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino#L1) is named `handheld.ino`, but Arduino CLI expects `handheld_mkr.ino` inside folder `handheld_mkr`.
- [../DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino](../DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino#L1) is named `opta_modbus_slave.ino`, but Arduino CLI expects `tractor_opta.ino` inside folder `tractor_opta`.
- [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino#L1) has the same class of mismatch for folder `tractor_h7`.

Impact:

No active firmware target can become a normal Arduino CLI gate until the sketch-folder rule is fixed. This currently fails before missing cores, missing libraries, shared C build glue, or crypto flags are even evaluated.

Recommended fix:

- Rename primary sketches to match folders, for example `handheld_mkr.ino`, `tractor_h7.ino`, and `tractor_opta.ino`; or rename folders to match the primary sketches.
- Keep secondary files as `.cpp` where needed, for example `tractor_m4.cpp`.
- After the naming fix, rerun compile to expose the next layer: required cores, RadioLib, Opta expansion library, shared `lora_proto.c`, and crypto backend flags.

### 3. Blocker: CI advertises controller CI but does not compile firmware, and Python dependencies are not installed

Evidence:

- [.github/workflows/arduino-ci.yml](../../.github/workflows/arduino-ci.yml#L24-L37) runs `python -m unittest discover -s tests` without installing `LifeTrac-v25/DESIGN-CONTROLLER/base_station/requirements.txt`.
- [.github/workflows/arduino-ci.yml](../../.github/workflows/arduino-ci.yml#L39-L48) has a `firmware-compile-plan` job that only prints status messages.
- [../DESIGN-CONTROLLER/base_station/tests/test_web_ui_auth.py](../DESIGN-CONTROLLER/base_station/tests/test_web_ui_auth.py#L19-L27) and [../DESIGN-CONTROLLER/base_station/tests/test_web_ui_accel.py](../DESIGN-CONTROLLER/base_station/tests/test_web_ui_accel.py#L14-L20) skip if FastAPI/Paho are missing.

Impact:

CI can be green while:

- no firmware target has compiled;
- web UI authentication tests are skipped;
- accelerator settings tests are skipped;
- compose/runtime dependency breaks are not caught.

Recommended fix:

- Install Python requirements before tests: `python -m pip install -r LifeTrac-v25/DESIGN-CONTROLLER/base_station/requirements.txt`.
- Make dependency-backed tests fail rather than skip in CI, using an environment variable such as `CI=true` or a dedicated test runner.
- Replace `firmware-compile-plan` with real Arduino compile jobs after the sketch naming issue is fixed.
- Add a temporary explicit failing/xfail issue marker if firmware compile is intentionally deferred, rather than a passing print-only job.

### 4. High: control radios boot at BW125 even though the active decision is BW250

Evidence:

- [../DESIGN-CONTROLLER/DECISIONS.md](../DESIGN-CONTROLLER/DECISIONS.md#L21) says SF7 control must run at BW250 so encrypted 44 B control frames fit the 20 Hz cadence.
- [../DESIGN-CONTROLLER/base_station/tests/test_lora_proto.py](../DESIGN-CONTROLLER/base_station/tests/test_lora_proto.py#L58-L64) locks that assumption in the Python tests.
- [../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino#L362-L367) calls `radio.begin(..., 125.0, 7, 5, ...)` at boot.
- [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino#L674) also calls `radio.begin(915.0, 125.0, 7, 5, 0x12, 20)`.
- Both firmware ladder tables define rung 0 as BW250: [handheld](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino#L73-L77), [tractor](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino#L140-L144).

Impact:

At boot, both ends start in a PHY that the current review history already found too slow for encrypted 20 Hz control. The display/ladder state may say rung 0/BW250 while the actual radio is at BW125 until a retune path runs.

Recommended fix:

- Initialize with `250.0` bandwidth, or call the same `apply_phy_rung(0)` logic from setup after `radio.begin`.
- Update stale comments and docs that still say SF7/BW125 for the control profile, for example [../DESIGN-CONTROLLER/TRACTOR_NODE.md](../DESIGN-CONTROLLER/TRACTOR_NODE.md#L201).
- Add a firmware-side compile-time constant for default control PHY so setup and ladder table cannot drift.

### 5. High: all-zero fleet keys remain in active runtime paths

Evidence:

- [../DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L70) sets `FLEET_KEY = bytes(16)`.
- [../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino#L43) sets `static const uint8_t kFleetKey[16] = {0};`.
- [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino#L88) sets `static const uint8_t kFleetKey[16] = {0};`.
- [../DESIGN-CONTROLLER/KEY_ROTATION.md](../DESIGN-CONTROLLER/KEY_ROTATION.md#L53-L68) documents replacing the key, but runtime code does not enforce it.

Impact:

If a bench or field build accidentally ships with these defaults, confidentiality and authentication assumptions collapse. It also makes it too easy for tests to pass against a known placeholder instead of proving provisioning works.

Recommended fix:

- Load base-station key from `LIFETRAC_FLEET_KEY_HEX` or a mounted secret file and refuse all-zero values unless a clearly named test flag is set.
- Generate firmware `key.h` from a provisioning step and fail compile if only `key.h.example` or an all-zero key is present.
- Add a unit test for key-loading behavior and a startup log/audit event that records key ID, not key material.

### 6. Blocker: real firmware AES-GCM decrypt length contract does not match live callers

Evidence:

- [../DESIGN-CONTROLLER/firmware/common/lora_proto/crypto_stub.c](../DESIGN-CONTROLLER/firmware/common/lora_proto/crypto_stub.c#L48-L55) treats `ct_len` as ciphertext-plus-tag length and copies `ct_len - 16` plaintext bytes.
- [../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp](../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp#L56-L61) treats `ct_len` as ciphertext-only length and reads the tag from `ct + ct_len`.
- [../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp](../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp#L80-L86) does the same in the MKR/rweather path.
- [../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino#L238) and [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino#L262) pass `len - 12`, which is ciphertext plus the 16-byte GCM tag.
- [../DESIGN-CONTROLLER/firmware/bench/crypto_vectors/host_check.c](../DESIGN-CONTROLLER/firmware/bench/crypto_vectors/host_check.c#L112-L117) passes `clen`, meaning ciphertext length excluding tag, so the bench check does not cover the live caller contract.

Impact:

The sim/stub path and the production crypto path have incompatible call contracts. Once production builds switch to `LIFETRAC_USE_REAL_CRYPTO`, valid incoming packets can fail authentication, and the real decrypt wrappers can read 16 bytes beyond the provided encrypted buffer when called the way the active sketches currently call them.

Recommended fix:

- Make `lp_decrypt` accept one unambiguous length. The safest contract is `ct_tag_len` including the tag, because that matches the on-air payload after the nonce and the current callers.
- Inside `lp_crypto_real.cpp`, reject `ct_tag_len < 16`, set `cipher_len = ct_tag_len - 16`, and pass `ct + cipher_len` as the tag pointer.
- Update `host_check.c` to pass `clen + 16`, so the bench check covers the same API contract as the firmware sketches.
- Add a small C host test for `empty_pt` and normal frames through both encrypt and decrypt using the public `lp_decrypt` signature.

### 7. High: camera/image path is split between MQTT and H747 UART with no active bridge between them

Evidence:

- [../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py#L57-L58) publishes frames to MQTT topic `lifetrac/v25/cmd/image_frame`.
- [../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py#L246) publishes payloads to that MQTT topic.
- [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino#L502-L532) only consumes KISS-framed topic/payload bytes from `Serial1`.
- [../DESIGN-CONTROLLER/firmware/tractor_x8/image_pipeline/ipc_to_h747.py](../DESIGN-CONTROLLER/firmware/tractor_x8/image_pipeline/ipc_to_h747.py#L1-L18) defines a separate length-prefixed IPC protocol, but it is not referenced by `camera_service.py` in the active path.

Impact:

The tractor X8 camera producer does not currently feed the M7 LoRa telemetry path described in its own module docstring. Depending on deployment topology, image frames can remain on a local MQTT broker and never cross the tractor X8 -> H747 -> LoRa -> base-station chain.

Recommended fix:

- Pick one active X8-to-M7 protocol for image frames: KISS topic-prefix frames matching `poll_x8_uart()`, or the `ipc_to_h747.py` length-prefixed protocol.
- Wire `camera_service.py` into that protocol directly, or add a named bridge service that subscribes to `cmd/image_frame` and writes the M7 UART.
- Add an integration test that feeds one synthetic tile frame through camera encode -> X8/H747 IPC -> M7 wrapping contract -> base reassembly.

### 8. High: optional FHSS path in tractor M7 sets frequency from hop counter, not channel Hz

Evidence:

- [../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino#L183) correctly calls `lp_fhss_channel_hz(g_fhss_key_id, hop) / 1.0e6f`.
- [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino#L494) calls `radio.setFrequency(hop / 1.0e6f)` after `lp_csma_pick_hop(...)`.

Impact:

If `LIFETRAC_FHSS_ENABLED` is enabled, the tractor attempts to tune to a frequency derived from a small hop counter instead of a 902-925 MHz channel. That makes the FHSS build non-functional and can mask itself because the default single-channel build does not exercise this branch.

Recommended fix:

- Mirror the handheld code:

```cpp
radio.setFrequency(lp_fhss_channel_hz(g_fhss_key_id, hop) / 1.0e6f);
g_fhss_hop_counter = hop + 1;
```

- Add a host-level C/C++ test or static check around the FHSS helper path before enabling the flag in CI.

### 9. Medium: generic telemetry fragmentation helpers are tested but not integrated in `lora_bridge.py`

Evidence:

- [../DESIGN-CONTROLLER/base_station/lora_proto.py](../DESIGN-CONTROLLER/base_station/lora_proto.py#L580-L626) implements `TelemetryReassembler`.
- [../DESIGN-CONTROLLER/base_station/tests/test_telemetry_fragmentation.py](../DESIGN-CONTROLLER/base_station/tests/test_telemetry_fragmentation.py#L23-L99) tests fragmentation and reassembly.
- [../DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L153-L172) publishes telemetry payloads directly to `topic_name(topic_id)` after CRC verification.

Impact:

Image tile fragments are later reassembled by `web_ui.py` for the `video/tile_delta` topic, but non-image fragmented telemetry would be exposed to MQTT consumers as raw fragment bodies. The tested `TelemetryReassembler` does not currently protect the bridge's generic telemetry path.

Recommended fix:

- Add a per-source/topic `TelemetryReassembler` inside `Bridge._handle_air` before publishing MQTT payloads for topics that may fragment.
- Decide whether MQTT should carry complete logical telemetry only, or whether fragment bodies are an intentional low-level topic. If intentional, document topic naming clearly.

### 10. Medium: web control and parameter APIs need stricter input bounds

Evidence:

- [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L489-L514) casts joystick fields with `int(...)` but catches only `json.JSONDecodeError`. A malformed numeric field can raise `ValueError` and close the control socket.
- [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L660-L673) forwards arbitrary JSON patches to `lifetrac/v25/params/set` with no size, key, or type validation.

Impact:

Authenticated clients can accidentally or deliberately send malformed messages that terminate their own control channel, flood MQTT with oversized parameter patches, or push validation responsibility entirely downstream to the tractor service.

Recommended fix:

- Validate control JSON at the WebSocket boundary: axes in `[-127, 127]`, buttons in `0..0xFFFF`, flags in `0..0xFF`, heartbeat implicit server-side.
- Catch `TypeError` and `ValueError` around numeric conversion.
- Add a small schema or whitelist for parameter keys and cap request body size.

### 11. Medium: Modbus write/read failures are not surfaced to safety state or telemetry quality

Evidence:

- [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino#L244-L249) calls `ModbusRTUClient.endTransmission()` without checking the result.
- [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino#L427-L433) reads six registers after `requestFrom(...)` without checking `available()` count.

Impact:

If the Opta is disconnected or timing out, the M7 can continue looping without an operator-visible quality flag. Telemetry can show zeros or stale-looking values, and failed control writes are not counted or escalated.

Recommended fix:

- Check `endTransmission()` and increment a Modbus fault counter on failure.
- Check `ModbusRTUClient.available() >= 6` before reading telemetry.
- Publish Modbus health in `TOPIC_SOURCE_ACTIVE` or `TOPIC_ERRORS`, and consider forcing neutral/source-none after repeated write failures.

### 12. Medium: active valve control remains coarse and several Opta outputs are TODO

Evidence:

- [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino#L222-L236) maps joystick thresholds directly to coils and a simple flow magnitude, with a TODO for deadband, ramp, and dual-flow split.
- [../DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino](../DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino#L209-L212) leaves `REG_AUX_OUTPUTS` unimplemented.

Impact:

The current control feel is binary/coarse for directional valves and incomplete for auxiliary relays/engine-kill arm bits. It may be acceptable for dry bench bring-up with LEDs, but not for hydraulic motion beyond basic smoke testing.

Recommended fix:

- Port or redesign the deadband/ramp/differential drive logic before wet hydraulic tests.
- Implement `REG_AUX_OUTPUTS` or remove it from the active map until supported.
- Add bench tests around expected register snapshots for representative joystick/button states.

### 13. Low/Medium: web socket fan-out and audit tail can be hardened

Evidence:

- [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L377-L429) stores WebSocket subscribers in global sets with no connection limit.
- [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L180-L205) schedules send futures via `asyncio.run_coroutine_threadsafe(...)` without retaining futures or logging send failures.
- [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L350-L364) prepends chunks when tailing audit logs, which repeatedly copies data if it has to scan far back in a sparse/newline-poor file.

Impact:

These are not immediate functional blockers, but they make the operator UI easier to exhaust or harder to diagnose under a noisy browser/network condition.

Recommended fix:

- Add subscriber limits per WebSocket class and remove sockets on failed sends.
- Attach callbacks to scheduled futures or route all fan-out through an async task with logging.
- Rework audit tailing to collect chunks in a list/deque and join once.

## Positive Notes

- `crypto_stub.c` now has a compile-time guard requiring `LIFETRAC_ALLOW_STUB_CRYPTO`, which is the right direction. The remaining gap is real CI/build wiring and key provisioning refusal.
- Python protocol tests cover CRC/KISS, airtime sizing, replay windows, priority classification, CSMA hop selection, telemetry fragmentation helpers, image reassembly, audit logging, and link monitor hysteresis.
- The M4 watchdog design is intentionally conservative and starts de-energized, then latches low on stale tick, stuck loop counter, or M7 estop request.
- The Opta slave refuses motion writes after a safety fault and keeps watchdog side effects tied to register changes rather than poll reads.

## Suggested Fix Order

1. Fix compose startup: env-driven MQTT host, lora serial arg/default, and a smoke test for command parsing.
2. Fix Arduino sketch folder/name layout and rerun compiles to expose dependency and shared-source errors.
3. Replace the print-only firmware CI job with real Arduino CLI compile jobs.
4. Install Python requirements in CI and make web/auth tests mandatory there.
5. Change firmware boot PHY to SF7/BW250 and clean stale BW125 docs/comments.
6. Add key-loading and all-zero-key refusal on base and firmware.
7. Fix the real `lp_decrypt` length contract and make the host crypto check use the same contract as the firmware callers.
8. Wire camera service to the M7 UART/IPC path and add an end-to-end image-frame test.
9. Fix FHSS tractor frequency conversion before enabling `LIFETRAC_FHSS_ENABLED` anywhere.
10. Add Modbus fault accounting and web/API input validation.
11. Implement ramp/deadband/aux output behavior before wet hydraulic motion.

## Residual Risk

I did not flash or compile the embedded firmware beyond the Arduino CLI sketch-opening failures. Once the sketch naming issue is fixed, the next review should rerun actual compiles with the intended cores/libraries and both crypto modes:

- CI/sim: `LIFETRAC_ALLOW_STUB_CRYPTO`
- production candidate: `LIFETRAC_USE_REAL_CRYPTO` plus real key provisioning

That second pass is likely to reveal normal Arduino-library and cross-file build issues that are currently hidden by the folder/name failure.

---

## Second Pass Follow-Up Review - GitHub Copilot (2026-04-29)

### Follow-Up Scope

This pass re-reviewed the active v25 controller code after the recent fixes, verified the previously reported failure classes, and performed another full sweep for additional defects or hardening opportunities. It also folds in the second-pass findings from the parallel reviews:

- [GPT-5.3-Codex v1.0 second pass](2026-04-28_Controller_Code_Pipeline_Review_GPT-5.3-Codex_v1_0.md#second-pass-follow-up-conclusion-2026-04-28)
- [Gemini 3.1 Pro v1.0 second pass](2026-04-28_Controller_Code_Review_Gemini_3.1_Pro_v1.0.md#4-second-pass-review-findings)

### Validation Performed

```powershell
cd c:\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\base_station
$env:LIFETRAC_ALLOW_UNCONFIGURED_KEY="1"
python -m unittest discover -s tests
```

Result: `Ran 147 tests in 0.538s — OK` (was 88 at v1.0 first pass; +59 tests across Rounds 1–8 covering CRC, KISS, airtime, replay windows, CSMA, telemetry fragmentation, image reassembly, audit logging, link-monitor hysteresis, IP-108 command-frame fuzz, IP-104 back-channel dispatch, IP-107 M7 TX queue SIL, Round-8 `pack_control` fuzz, and the Round-8 M4 safety-supervisor SIL).

### Confirmed Fixed Since v1.0

Cross-checked against my original 13 findings *and* the parallel reviews' lists. All of the following are now resolved in active code:

| v1.0 finding | Status | Notes |
|---|---|---|
| 1. Compose MQTT/serial wiring broken | **Fixed** | `lora_bridge.py` reads `LIFETRAC_LORA_DEVICE` / `LIFETRAC_MQTT_HOST`; `web_ui.py` env-driven; compose passes both. Cross-confirmed by GPT-5.3-Codex §1. |
| 2. Arduino sketch folder/name mismatch | **Fixed** | Active sketches now `handheld_mkr.ino`, `tractor_h7.ino`, `tractor_opta.ino`. Cross-confirmed by GPT §6. |
| 3. CI does not compile firmware / no Python deps | **Partially fixed** | Three best-effort `arduino-cli compile` jobs added (handheld_mkr, tractor_opta, tractor_h7) with `continue-on-error: true`; Python deps installed. Hard-gate flip remains open — see §A below. |
| 4. Control radios boot at BW125 | **Fixed** | Both sketches now init from `LADDER[0]` (SF7/BW250). Cross-confirmed by GPT §5. |
| 5. All-zero fleet keys | **Partially fixed** | Real-crypto path refuses placeholders; stub path still permissive — see §D below. |
| 6. `lp_decrypt` length-contract drift | **Fixed** | Single `ct_tag_len` contract documented in `lora_proto.h`; both real backends and host bench aligned. Cross-confirmed by GPT §3. |
| 7. Camera ↔ M7 path unbridged | **Fixed (Round 4)** | `camera_service.py` writes the M7 UART via the X8 KISS framing; back-channel opcodes (REQ_KEYFRAME / CAMERA_SELECT / CAMERA_QUALITY / ENCODE_MODE) dispatched and unit-tested. |
| 8. FHSS tractor frequency-from-hop bug | **Fixed** | Tractor mirrors handheld `lp_fhss_channel_hz(...)/1.0e6f`. |
| 9. Generic telemetry reassembler not used in bridge | **Fixed** | Bridge feeds per-source/topic reassembler before MQTT publish. |
| 10. Web control / params API input validation | **Fixed** | Pydantic models on the WebSocket and params endpoints; `_admit_ws()` for telemetry/image/state. Control-socket cap still open — see §B below. |
| 11. Modbus failure not surfaced | **Fixed** | Fault counter + `TOPIC_ERRORS` publish on `endTransmission()` non-zero / short reads. |
| 12. Coarse valve control / `REG_AUX_OUTPUTS` TODO | **Fixed (Round 4)** | IP-303 deadband + ramp + dual-flow split ported into M7; aux-output register implemented or removed from active map. |
| 13. WS fan-out / audit tail hardening | **Partially fixed** | Audit-tail rewritten to deque-and-join; per-class subscriber caps for telemetry/image/state landed. WS fan-out concurrency + audit-log file lifecycle still open — see §C, §E. |

### Worth Pursuing — Net-New Findings From the Parallel Reviews

The following items are *not* yet implemented and align with concerns raised independently by GPT-5.3-Codex and Gemini 3.1 Pro on their second passes. All are pure-software, no-hardware-required, and should be addressed before declaring field-deployment readiness.

#### §A. CI compile-gate flip from `continue-on-error: true` to blocking — High

* **Independently flagged by:** GPT-5.3-Codex §6 (second pass).
* **Status:** the three Round-6 `arduino-cli compile` jobs run but cannot fail the build. Until at least one green run on a real Actions runner confirms the staging/library glue works, regressions in shared C build paths (lora_proto, shared_mem, crypto-stub vs. real flag combos) will land silently.
* **Action:** trigger a full CI run; once all three jobs pass on a clean runner, drop `continue-on-error` and require them on PRs that touch `firmware/**` or `.github/workflows/arduino-ci.yml`. Add a `firmware-compile-status` summary job that aggregates all three so branch protection has a single required check.

#### §B. `/ws/control` connection cap — High (security)

* **Independently flagged by:** GPT-5.3-Codex §3 (second pass), Gemini §4.2.2.
* **Evidence:** `_admit_ws()` gates `/ws/telemetry`, `/ws/image`, and `/ws/state` but the operator control socket accepts unlimited concurrent clients.
* **Action:** apply the same `_admit_ws()` policy to `/ws/control` with a tighter cap (e.g. `MAX_CONTROL_SUBSCRIBERS = 4`) and emit an audit-log event on rejection. Add a unit test mirroring the existing telemetry-cap test in `test_web_ui_auth.py`.

#### §C. WebSocket subscriber-set cross-thread access — High

* **Independently flagged by:** GPT-5.3-Codex §2 (second pass), Gemini §4.2.1 (rated "Major").
* **Evidence:** the MQTT callback thread iterates `image_subscribers` / `telemetry_subscribers` while the FastAPI event loop mutates them on connect/disconnect. Risk: intermittent `RuntimeError: Set changed size during iteration` and dropped fan-out when connections flap.
* **Action:** guard each subscriber pool with `threading.Lock()` (or marshal mutations through `loop.call_soon_threadsafe`). Add a stress test that spins up N clients and churns them while telemetry flows.

#### §D. Stub-crypto builds still tolerate all-zero fleet keys — Medium (security)

* **Independently flagged by:** GPT-5.3-Codex §4 (second pass).
* **Evidence:** runtime refusal is gated under `LIFETRAC_USE_REAL_CRYPTO`. CI/sim builds and any accidental stub-flagged production build will accept the placeholder key without complaint.
* **Action:** make the all-zero-key check unconditional except when `LIFETRAC_ALLOW_UNCONFIGURED_KEY=1` (the env flag the test suite already sets). This keeps the test surface unchanged but closes the "stub build flashed onto real hardware" hole.

#### §E. PIN-failure audit-log file lifecycle — Low/Medium

* **Independently flagged by:** GPT-5.3-Codex §1 (second pass new findings).
* **Evidence:** `AuditLog(...)` is instantiated on every PIN failure / lockout event, producing `ResourceWarning: unclosed file` during tests.
* **Action:** promote `AuditLog` to a module-level singleton (or pass the existing instance into `_record_failure()` via dependency injection). Already a pattern used elsewhere in `web_ui.py` for the bridge audit logger.

#### §F. ReplayWindow C↔Python bitmap-width invariant — Low

* **Independently flagged by:** Gemini §4.2.3.
* **Evidence:** `ReplayWindow` in `lora_proto.c` and `_restamp_control` in `lora_bridge.py` both depend on a 64-frame window (`age <= 63`) but the constant is open-coded in two languages. Drift between sides is silent.
* **Action:** add a one-line cross-check test in `test_lora_proto.py` that asserts `LORA_REPLAY_WINDOW == 64` against a value parsed from the C header (or a `lora_proto.py` constant that is imported/audited at startup). Also document the invariant in `LORA_PROTOCOL.md` next to the seq-wrap spec.

### Items I'm Deferring

* **Gemini §1.1 sequence-number mismatch** — already verified fixed by GPT second pass (bridge now extracts `hdr.sequence_num` and passes it as `nonce_seq`); my own re-read of `_on_mqtt_message` confirms.
* **Gemini §1.2 multi-device lockout** — fixed via `LIFETRAC_TRUSTED_PROXIES` + `X-Forwarded-For` parsing; verified.
* **Gemini §2.1 / §2.2 C buffer / `CommandFrame` issues** — both fixed and locked by the IP-108 fuzz suite (Round 6) plus my Round-8 `pack_control` fuzz suite, which between them sweep every legal `arg_len` × every stick / button / flag permutation.
* **Gemini §3.2 nonce-bleeding compression** — out-of-scope optimization; the 12-byte nonce is mandated by AES-GCM and the explicit timestamp + random tail are deliberate replay-protection redundancy. Not pursuing.

### Suggested Fix Order (Net-New)

1. **§B** `/ws/control` admission cap (smallest blast radius, security-relevant, ~10 LOC).
2. **§C** WS subscriber-set lock (highest-risk sporadic bug; both reviewers flagged).
3. **§E** Audit-log singleton (cleans up test warnings, trivial diff).
4. **§D** Unconditional all-zero-key refusal (security hardening before any real bench session).
5. **§F** Replay-window invariant test (cheap regression catch).
6. **§A** CI compile-gate flip — requires a real Actions run first; can land last.

### Round 8 Verification of My Own Tests

Two suites added in Round 8 directly raise the verification bar for the W4 HIL gates:

* `test_control_frame_fuzz.py` (7 tests) locks the symmetric counterpart to IP-108 on the 20 Hz handheld hot path.
* `test_m4_safety_sil.py` (9 tests) ports `tractor_m4.cpp` to pure Python and asserts TR-A E-stop latch < 100 ms, TR-B watchdog window, TR-C latch, TR-D no-false-trip, TR-E magic-only gating, TR-F seqlock retry, TR-G stuck-millis witness, TR-H version-mismatch refuse-to-arm.

Effect: W4-01 (E-stop latency) and W4-03 (watchdog trip) move from greenfield design passes on the bench to verification passes against a documented model.

### Updated Residual Risk

The Python protocol/test surface is materially stronger (88 → 147 tests). The remaining risk is concentrated in operational hardening — WebSocket concurrency safety, audit-log lifecycle, blocking firmware-compile CI, and unconditional key-provisioning enforcement — plus the actual HIL bench session. None of these are core protocol-math defects; they are deployability and defense-in-depth gaps.


Rechecked areas:

- Base station runtime and pipeline: [../DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py), [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py), image pipeline modules, settings/audit helpers, [../DESIGN-CONTROLLER/Dockerfile](../DESIGN-CONTROLLER/Dockerfile), and [../DESIGN-CONTROLLER/docker-compose.yml](../DESIGN-CONTROLLER/docker-compose.yml).
- Firmware: [../DESIGN-CONTROLLER/firmware/common/lora_proto/](../DESIGN-CONTROLLER/firmware/common/lora_proto/), [../DESIGN-CONTROLLER/firmware/handheld_mkr/](../DESIGN-CONTROLLER/firmware/handheld_mkr/), [../DESIGN-CONTROLLER/firmware/tractor_h7/](../DESIGN-CONTROLLER/firmware/tractor_h7/), [../DESIGN-CONTROLLER/firmware/tractor_opta/](../DESIGN-CONTROLLER/firmware/tractor_opta/), and tractor X8 camera back-channel code.
- Pipeline and CI: [.github/workflows/arduino-ci.yml](../../.github/workflows/arduino-ci.yml), [../DESIGN-CONTROLLER/ARDUINO_CI.md](../DESIGN-CONTROLLER/ARDUINO_CI.md), [../DESIGN-CONTROLLER/arduino_libraries.txt](../DESIGN-CONTROLLER/arduino_libraries.txt), and stale controller docs.

### Verification Results

- Python tests: `python -m unittest discover -s tests -v` from `DESIGN-CONTROLLER/base_station` passed: `Ran 147 tests ... OK`, with no skipped tests in this environment.
- Python syntax gate: `python -m compileall -q base_station firmware/tractor_x8 tools` passed with no output.
- VS Code diagnostics over the controller tree reported no current editor errors.
- Arduino smoke compiles now progress farther for the handheld and H7 targets: [../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino) reaches missing local `arduino:samd`, and [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) reaches missing local `arduino:mbed_portenta`.
- The Opta target still fails before dependency resolution because the folder [../DESIGN-CONTROLLER/firmware/tractor_opta/](../DESIGN-CONTROLLER/firmware/tractor_opta/) contains only `opta_modbus_slave.ino`; Arduino CLI expects `tractor_opta.ino` for folder-based compilation.
- Test warnings remain useful: the suite emits `ResourceWarning: unclosed file` from [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L197) and [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L208), plus FastAPI `on_event` deprecation warnings from [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L462).

### Previously Reported Issues Now Fixed

1. Compose startup and MQTT host wiring are materially fixed.

- [../DESIGN-CONTROLLER/docker-compose.yml](../DESIGN-CONTROLLER/docker-compose.yml#L54) now launches `lora_bridge.py` with the configured serial device and `--mqtt` host.
- [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L238) now reads `LIFETRAC_MQTT_HOST` and retries broker startup instead of hardcoding localhost at import time.

2. Base-station fleet key handling is now fail-closed for deployment.

- [../DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L74) loads the fleet key from `LIFETRAC_FLEET_KEY_FILE` or `LIFETRAC_FLEET_KEY_HEX`.
- It rejects missing, wrong-length, and all-zero keys unless the explicit test/development escape hatch `LIFETRAC_ALLOW_UNCONFIGURED_KEY=1` is set.
- [../DESIGN-CONTROLLER/docker-compose.yml](../DESIGN-CONTROLLER/docker-compose.yml#L45) mounts the fleet key as a Docker secret.

3. The `CMD_REQ_KEYFRAME` recovery path is wired through the base station.

- [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L434) publishes `lifetrac/v25/cmd/req_keyframe` when the canvas detects that a fresh keyframe is needed.
- [../DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L160) subscribes to that topic, and [../DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L380) converts it to `CMD_REQ_KEYFRAME` over LoRa.
- [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino#L521) handles the command and forwards it toward the X8 camera service.

4. The AES-GCM decrypt contract is aligned in the live firmware code.

- [../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.h](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.h#L229) now documents `lp_decrypt` as receiving ciphertext plus tag length.
- [../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp](../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp#L56) and [../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp](../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp#L83) now split the trailing `LP_TAG_LEN` internally, matching the active firmware callers.

5. Boot PHY and FHSS retune logic now match the current control-link decision.

- [../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino#L391) and [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino#L1045) initialize from `LADDER[0]`, so boot starts at the intended SF7/BW250 profile.
- The tractor FHSS TX path now uses `lp_fhss_channel_hz(...) / 1.0e6f` in [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino#L828), matching the handheld path.

6. Several web/API hardening items are improved.

- Control and REST payloads now pass through Pydantic validation models in [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L45).
- Telemetry, image, and state WebSockets now have bounded admission through `_admit_ws` in [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L276).
- Generic telemetry reassembly is now integrated before MQTT publication in [../DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L179) and [../DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L249).

7. The Python test surface is much stronger.

- The local suite now runs 147 tests with no skips in this environment, compared with 88 tests and 2 dependency-related skips during the original review.

### Remaining or New Findings

#### 1. Blocker: Opta firmware still cannot compile by documented folder path

Evidence:

- [../DESIGN-CONTROLLER/ARDUINO_CI.md](../DESIGN-CONTROLLER/ARDUINO_CI.md#L35) and [.github/workflows/arduino-ci.yml](../../.github/workflows/arduino-ci.yml#L113) compile the Opta target by folder: `firmware/tractor_opta`.
- The folder [../DESIGN-CONTROLLER/firmware/tractor_opta/](../DESIGN-CONTROLLER/firmware/tractor_opta/) contains [../DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino](../DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino), but no `tractor_opta.ino` primary sketch.
- Local Arduino CLI smoke compile still fails with: `main file missing from sketch: ...\firmware\tractor_opta\tractor_opta.ino`.

Impact:

The Opta hydraulic I/O firmware remains outside the normal Arduino CLI compile path. The new CI Opta compile job will show the failure in logs, but because it is `continue-on-error`, it does not block PRs.

Recommendation:

- Rename `opta_modbus_slave.ino` to `tractor_opta.ino`, or rename the folder to `opta_modbus_slave` and update docs/CI commands consistently.
- Then rerun the Opta compile so missing libraries or board-core issues are exposed after the sketch-opening layer is fixed.

#### 2. High: base-station nonce store advances the protocol sequence by 64 per packet

Evidence:

- [../DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L400) calls `_reserve_tx_seq()` for every base-station LoRa transmission.
- [../DESIGN-CONTROLLER/base_station/nonce_store.py](../DESIGN-CONTROLLER/base_station/nonce_store.py#L63) returns `(entry.seq + self.gap) & 0xFFFF`, with `DEFAULT_GAP = 64` in [../DESIGN-CONTROLLER/base_station/nonce_store.py](../DESIGN-CONTROLLER/base_station/nonce_store.py#L33).
- The tests lock in this behavior in [../DESIGN-CONTROLLER/base_station/tests/test_telemetry_fragmentation.py](../DESIGN-CONTROLLER/base_station/tests/test_telemetry_fragmentation.py#L112), where reservations are expected to jump by `gap`.

Impact:

The base source sequence emits `0, 64, 128, ...` and wraps after only 1,024 transmissions. At 20 Hz control cadence, that is roughly 51 seconds instead of the expected 65,536-frame cycle. The recipient replay window sees each `+64` as a valid forward jump and resets its bitmap, so older authenticated frames have a much shorter path back into the acceptable sequence space. This weakens replay protection on base-originated control and command traffic.

Recommendation:

- Use the crash-safety gap as a startup or persisted-reservation margin, not as the per-packet protocol increment.
- A safer pattern is: on load, start from `persisted_high_water + GAP`; emit sequence numbers by `+1`; periodically persist a reserved future high-water such as `last_emitted + GAP`.
- Add tests that assert the emitted protocol sequence is contiguous during normal operation and only jumps after simulated restart/reload.

#### 3. Medium: crypto vector host check still uses the old decrypt length contract

Evidence:

- [../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.h](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.h#L229) now says `ct_len` is ciphertext plus tag.
- [../DESIGN-CONTROLLER/firmware/bench/crypto_vectors/host_check.c](../DESIGN-CONTROLLER/firmware/bench/crypto_vectors/host_check.c#L111) builds `buf = ciphertext || tag`, but [../DESIGN-CONTROLLER/firmware/bench/crypto_vectors/host_check.c](../DESIGN-CONTROLLER/firmware/bench/crypto_vectors/host_check.c#L116) calls `lp_decrypt(..., (size_t)clen, ...)`, where `clen` is ciphertext length only.

Impact:

The production firmware path is fixed, but the bench/golden-vector check still exercises the stale API contract. It will reject valid vectors or fail to cover the same public signature used by the sketches.

Recommendation:

- Change the host check to pass `(size_t)clen + 16` to `lp_decrypt`.
- Include an empty-plaintext vector and at least one non-empty vector so both tag-only and ciphertext-plus-tag paths are covered.

#### 4. Medium: PIN-failure audit logging leaks file handles

Evidence:

- The passing Python suite emits `ResourceWarning: unclosed file` from the failure and lockout paths.
- [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L197) and [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L208) instantiate `AuditLog(_AUDIT_PATH)` inline and immediately call `.record(...)` without closing it.
- [../DESIGN-CONTROLLER/base_station/audit_log.py](../DESIGN-CONTROLLER/base_station/audit_log.py#L52) provides an explicit `close()` method, but no context manager.

Impact:

Every failed PIN attempt can leave an audit file descriptor open until garbage collection. A brute-force attempt or noisy test/deployment cycle can consume descriptors and make audit behavior nondeterministic.

Recommendation:

- Promote a module-level `AuditLog` instance for web UI auth events and close it during app shutdown/lifespan cleanup.
- Alternatively, add context-manager support to `AuditLog` and use `with AuditLog(_AUDIT_PATH) as log:` for one-shot writes.

#### 5. Medium: WebSocket subscriber pools are still mutated across threads without synchronization

Evidence:

- The MQTT network thread iterates `list(image_subscribers)` and `list(telemetry_subscribers)` in [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L336) and [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L358).
- The FastAPI event loop mutates those same global sets in endpoint `finally` blocks such as [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L572), [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L594), and [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L621).

Impact:

The caps and send-failure callbacks are good progress, but connection churn can still race with MQTT fan-out. In CPython, building `list(a_set)` can still fail if another thread mutates the set while it is being iterated, or can produce inconsistent fan-out behavior.

Recommendation:

- Guard subscriber-set add/discard/snapshot operations with a `threading.Lock`, or marshal all set mutation and fan-out snapshot creation through the FastAPI event loop.
- Add a stress test that connects/disconnects clients while injecting MQTT messages.

#### 6. Medium: `/ws/control` still has no bounded admission or single-controller policy

Evidence:

- `_admit_ws` is used for telemetry, image, and state sockets, but [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L658) accepts `/ws/control` directly after session-cookie validation.

Impact:

Authenticated browser tabs can open unbounded control sockets. Even with a valid PIN requirement and 20 Hz per-socket throttling, multiple sockets can duplicate or conflict with operator commands and can consume event-loop work.

Recommendation:

- Add `MAX_CONTROL_SUBSCRIBERS`, route `/ws/control` through the same bounded admission helper, and consider enforcing one active control socket per session or per client IP.
- Audit rejected/competing control sockets so operator handoff is visible.

#### 7. Medium: CI still reports firmware compile status without enforcing it

Evidence:

- The Python CI job in [.github/workflows/arduino-ci.yml](../../.github/workflows/arduino-ci.yml#L32) still runs tests without installing [../DESIGN-CONTROLLER/base_station/requirements.txt](../DESIGN-CONTROLLER/base_station/requirements.txt), so dependency-backed web tests can be skipped in a clean runner.
- The firmware compile jobs in [.github/workflows/arduino-ci.yml](../../.github/workflows/arduino-ci.yml#L51), [.github/workflows/arduino-ci.yml](../../.github/workflows/arduino-ci.yml#L92), and [.github/workflows/arduino-ci.yml](../../.github/workflows/arduino-ci.yml#L119) are all `continue-on-error: true`.
- [../DESIGN-CONTROLLER/docker-compose.yml](../DESIGN-CONTROLLER/docker-compose.yml) and [../DESIGN-CONTROLLER/Dockerfile](../DESIGN-CONTROLLER/Dockerfile) are not built as part of CI.

Impact:

The workflow is better than the original print-only firmware status, but it can still be green while the Opta target is unopenable, firmware compile breaks, Docker packaging breaks, or web/auth tests skip due missing dependencies.

Recommendation:

- Install base-station requirements in the Python job and make skips fail in CI for required dependency-backed tests.
- Add a Docker build/import smoke test for `web_ui` and `lora_bridge`.
- Remove `continue-on-error` target by target as each firmware build becomes deterministic; keep only explicitly documented experimental targets non-blocking.

#### 8. Low/Medium: firmware key provisioning is still placeholder-based in active sketches

Evidence:

- [../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino#L46) and [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino#L91) still define `static const uint8_t kFleetKey[16] = {0};`.
- Runtime refusal exists, but only inside `#ifdef LIFETRAC_USE_REAL_CRYPTO` at [../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino#L404) and [../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino#L1054).

Impact:

This is no longer a base-station deployment blocker, but stub/sim firmware builds can still run with a placeholder key. That may be acceptable for CI, but it should be impossible to confuse with a bench or production candidate.

Recommendation:

- Move firmware key material behind generated `key.h` or a provisioning include that fails compile unless a real key is present for non-test builds.
- Use an explicit `LIFETRAC_TEST_KEY_OK` or similar flag for stub/sim builds, so placeholder use is searchable and intentional.

#### 9. Low: stale docs still contradict the updated implementation

Evidence:

- [../DESIGN-CONTROLLER/KEY_ROTATION.md](../DESIGN-CONTROLLER/KEY_ROTATION.md#L53) still says the Python bridge reads `FLEET_KEY = bytes(16)` from `lora_bridge.py`, even though it now loads env/secret key material.
- [../DESIGN-CONTROLLER/TRACTOR_NODE.md](../DESIGN-CONTROLLER/TRACTOR_NODE.md#L201) still shows an SF7/BW125 `radio.begin(...)` example despite the active SF7/BW250 decision and implementation.

Impact:

The code is better than the docs in these spots. Operators following stale text can provision or validate the wrong thing.

Recommendation:

- Update key rotation docs to describe `LIFETRAC_FLEET_KEY_FILE`, `LIFETRAC_FLEET_KEY_HEX`, and the Docker secret path.
- Update radio examples to use the ladder/default PHY constants rather than a literal BW125 sample.

#### 10. Low: FastAPI startup hook uses deprecated `on_event`

Evidence:

- [../DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L462) uses `@app.on_event("startup")`, and the test suite emits FastAPI deprecation warnings.

Impact:

No immediate runtime failure, but future FastAPI upgrades will eventually force a migration.

Recommendation:

- Move startup/shutdown state initialization into a FastAPI lifespan context. This would also be a natural place to close a module-level web audit logger and stop MQTT cleanly in tests.

### Second Pass Conclusion

The follow-up fixes materially improved the v25 controller stack. The original compose-startup problem, base-station key loading, `CMD_REQ_KEYFRAME` command path, real AES-GCM decrypt contract, boot PHY, H7/handheld sketch naming, FHSS frequency conversion, web input validation, and telemetry reassembly integration are now in much better shape. The Python suite also grew from 88 tests with dependency skips to 147 passing tests with no skips in this local environment.

The stack is not yet fully hardware-test or field-deployment ready. The remaining blockers are narrower but important: the Opta firmware still cannot open as a folder sketch, the base-station nonce sequence reservation currently burns 64 protocol sequence numbers per TX, CI still does not hard-gate firmware/Docker/dependency-backed tests, and the web UI needs cleanup around audit file lifecycle and WebSocket concurrency. Fix those before relying on this controller pipeline for integrated hydraulic or field testing.
