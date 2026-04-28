# LifeTrac v25 Controller Code and Pipeline Review - GPT-5.3-Codex v1.0

Date: 2026-04-28
Reviewer: GitHub Copilot (model: GPT-5.3-Codex)
Review version: v1.0

## Scope

Reviewed active v25 controller code and delivery pipeline under:

- [DESIGN-CONTROLLER](../DESIGN-CONTROLLER)
- [DESIGN-CONTROLLER/base_station](../DESIGN-CONTROLLER/base_station)
- [DESIGN-CONTROLLER/firmware](../DESIGN-CONTROLLER/firmware)
- [DESIGN-CONTROLLER/docker-compose.yml](../DESIGN-CONTROLLER/docker-compose.yml)
- [DESIGN-CONTROLLER/Dockerfile](../DESIGN-CONTROLLER/Dockerfile)
- [.github/workflows/arduino-ci.yml](../../.github/workflows/arduino-ci.yml)

Excluded from blocking findings unless directly referenced: archived prototype code in RESEARCH-CONTROLLER.

## Validation Performed

Executed local protocol/image test sweep from base_station:

- Command: python -m unittest discover -s tests -p "test_*.py" -v
- Result: 88 passed, 2 skipped (web_ui auth/accel suites skipped due missing FastAPI/Paho in this local environment)

## Executive Summary

Core protocol logic and image-pipeline unit tests are generally healthy, but there are multiple integration breakpoints that will prevent reliable end-to-end operation.

Top blockers are:

1. Compose runtime wiring is broken (broker host and serial bridge entrypoint mismatch).
2. Keyframe recovery command path is incomplete (published by web_ui, not consumed by lora_bridge).
3. Firmware decrypt API contract is inconsistent between stub and real crypto paths versus active call sites.
4. Fleet-key provisioning remains placeholder-only in active runtime paths.

Do not treat the controller stack as deployment-ready until these are resolved.

## Findings (Ordered by Severity)

### 1. Critical: Compose stack wiring is internally inconsistent and lora_bridge command is invalid

Evidence:

- lora_bridge container command omits required serial port argument in [DESIGN-CONTROLLER/docker-compose.yml](../DESIGN-CONTROLLER/docker-compose.yml#L48).
- lora_bridge requires positional port in [DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L390).
- lora_bridge defaults MQTT to localhost in [DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L391), while compose injects mosquitto host in [DESIGN-CONTROLLER/docker-compose.yml](../DESIGN-CONTROLLER/docker-compose.yml#L43).
- web_ui also hardcodes localhost MQTT in [DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L114), ignoring compose host injection in [DESIGN-CONTROLLER/docker-compose.yml](../DESIGN-CONTROLLER/docker-compose.yml#L57).

Impact:

- lora_bridge fails at startup under compose unless manually patched.
- web_ui may fail broker connection in containerized deployment.

Recommendation:

- Read MQTT host and serial device from environment in both services.
- Make lora_bridge port optional when LIFETRAC_LORA_DEVICE is provided.
- Add a pipeline smoke test that validates command parsing and broker connectivity for compose services.

### 2. Critical: Keyframe request control path is incomplete

Evidence:

- web_ui publishes keyframe requests on MQTT in [DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L251).
- lora_bridge subscribes only to control, estop, camera_select in [DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L84), [DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L85), [DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L86).
- No handler path sends CMD_REQ_KEYFRAME from MQTT to LoRa in lora_bridge.

Impact:

- Base station can detect stale canvas conditions but cannot reliably request a new I-frame over air.
- Loss recovery behavior described in design docs becomes partial/non-functional.

Recommendation:

- Add subscription and handler for lifetrac/v25/cmd/req_keyframe.
- Pack and transmit CMD_REQ_KEYFRAME command frame (priority P1).
- Add integration test for forced keyframe request round-trip.

### 3. Critical: Firmware decrypt length contract is inconsistent across implementations and call sites

Evidence:

- Stub decrypt treats ct_len as ciphertext-plus-tag in [DESIGN-CONTROLLER/firmware/common/lora_proto/crypto_stub.c](../DESIGN-CONTROLLER/firmware/common/lora_proto/crypto_stub.c#L52).
- Real decrypt paths treat ct_len as ciphertext-only and read tag at ct + ct_len in [DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp](../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp#L85) and [DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp](../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp#L86).
- Active receivers pass len - 12 (ciphertext + tag) in [DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino#L238) and [DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino#L262).

Impact:

- Real crypto builds can reject valid packets or read tag at wrong offset.
- Stub-path testing can mask production crypto failures.

Recommendation:

- Standardize lp_decrypt API to a single ct_tag_len convention.
- Update both stub and real implementations to that convention.
- Add host-level crypto regression tests that use the same call signature as embedded call sites.

### 4. High: Fleet key handling is still placeholder-based in active runtime code

Evidence:

- Base bridge uses all-zero key in [DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py#L70).
- Handheld firmware uses all-zero key in [DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino#L43).
- Tractor firmware uses all-zero key in [DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino#L88).

Impact:

- Security posture collapses if placeholder keys are ever deployed.
- Provisioning process is not enforced by runtime startup/compile guards.

Recommendation:

- Fail startup/build when key is zero unless explicit insecure test flag is set.
- Wire runtime key loading to environment secret or generated key header path consistently across Python and firmware.
- Add key ID telemetry/audit on startup (never key material).

### 5. High: Control PHY boot settings conflict with current ladder assumptions

Evidence:

- Ladder rung 0 expects SF7/BW250 in [DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino#L74) and [DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino#L141).
- Boot radio init uses BW125 in handheld [DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld.ino#L363) and tractor [DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m7.ino#L674).

Impact:

- Runtime starts in a different PHY than ladder policy and airtime assumptions.
- Increases risk of cadence mismatch or misleading diagnostics at startup.

Recommendation:

- Initialize boot PHY through the same ladder/rung application path used later.
- Consolidate PHY defaults in one source of truth constant.

### 6. Medium: CI pipeline has blind spots for real controller readiness

Evidence:

- CI runs unit tests without dependency install in [.github/workflows/arduino-ci.yml](../../.github/workflows/arduino-ci.yml#L37).
- Firmware job is status echo only in [.github/workflows/arduino-ci.yml](../../.github/workflows/arduino-ci.yml#L39) and [.github/workflows/arduino-ci.yml](../../.github/workflows/arduino-ci.yml#L43).
- Auth/accel tests are explicitly skippable if dependencies absent in [DESIGN-CONTROLLER/base_station/tests/test_web_ui_auth.py](../DESIGN-CONTROLLER/base_station/tests/test_web_ui_auth.py#L19), [DESIGN-CONTROLLER/base_station/tests/test_web_ui_auth.py](../DESIGN-CONTROLLER/base_station/tests/test_web_ui_auth.py#L27), and [DESIGN-CONTROLLER/base_station/tests/test_web_ui_accel.py](../DESIGN-CONTROLLER/base_station/tests/test_web_ui_accel.py#L20).

Impact:

- Green CI does not guarantee firmware compiles or full web/auth tests ran.

Recommendation:

- Install base_station requirements in CI before tests.
- Convert skipped critical suites into required CI suites.
- Add real Arduino compile jobs (or explicit failing placeholder until enabled).

### 7. Medium: Settings environment variable mismatch can confuse deployment behavior

Evidence:

- Compose sets LIFETRAC_SETTINGS_PATH in [DESIGN-CONTROLLER/docker-compose.yml](../DESIGN-CONTROLLER/docker-compose.yml#L59).
- Settings store reads LIFETRAC_BASE_SETTINGS in [DESIGN-CONTROLLER/base_station/settings_store.py](../DESIGN-CONTROLLER/base_station/settings_store.py#L24).

Impact:

- Operators may think one settings path is active while code uses another.
- Troubleshooting and persistence expectations can drift.

Recommendation:

- Standardize on one environment variable name.
- Update compose, docs, and code to the same variable.

### 8. Medium: camera_service has a config edge-case crash risk and a thread-race on keyframe trigger

Evidence:

- WEBP_QUALITY is unbounded input in [DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py#L53).
- Encode loop is guarded by quality >= 5 in [DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py#L131), but function returns blob after loop in [DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py#L140).
- Keyframe request flag is shared unsafely between MQTT callback and main loop in [DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py#L232), [DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py#L242), [DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py#L243).

Impact:

- Invalid quality setting can lead to runtime exceptions.
- Keyframe requests can be dropped under timing races.

Recommendation:

- Clamp WEBP_QUALITY to a valid minimum/maximum at startup.
- Initialize fallback blob path explicitly.
- Replace shared dict flag with threading.Event or locked state.

## Optimization and Hardening Opportunities

1. Camera tile diff/encode performance

- Current implementation uses nested Python loops and per-tile slice comparisons in [DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py#L170) and [DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py#L175).
- Improvement: use numpy-backed frame arrays with vectorized tile-change masks; expected CPU reduction and better frame-time stability.

2. WebSocket fan-out backpressure

- Fan-out pushes from MQTT thread directly into event loop for each socket in [DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L178) and [DESIGN-CONTROLLER/base_station/web_ui.py](../DESIGN-CONTROLLER/base_station/web_ui.py#L203).
- Improvement: per-subscriber bounded queues with drop/coalesce strategy to prevent slow consumers from increasing scheduling overhead.

3. CI reproducibility

- Requirements are loosely pinned in [DESIGN-CONTROLLER/base_station/requirements.txt](../DESIGN-CONTROLLER/base_station/requirements.txt).
- Improvement: add a lockfile or constraints file for deterministic CI/runtime behavior.

## Suggested Fix Order

1. Fix compose wiring (MQTT host and lora_bridge args).
2. Wire CMD_REQ_KEYFRAME through lora_bridge.
3. Unify lp_decrypt length contract and re-test real crypto path.
4. Enforce non-placeholder key provisioning.
5. Align boot PHY with ladder defaults.
6. Upgrade CI to install dependencies and run real firmware compile gates.

## Residual Risk Statement

Even with current passing protocol tests, deployment risk remains high because the largest defects are cross-service integration and runtime wiring issues rather than isolated unit logic.
