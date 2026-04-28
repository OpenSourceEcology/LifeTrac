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
