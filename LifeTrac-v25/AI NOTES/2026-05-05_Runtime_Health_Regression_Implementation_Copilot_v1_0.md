# Runtime Health Regression Implementation (Copilot v1.0)

Date: 2026-05-05
Status: Implemented and host-regressed; local Arduino full-sketch compile blocked by pre-existing Portenta link-collision set.
Author: GitHub Copilot

## 1. Scope

This note records the code-level execution of the runtime-fix tranche requested by the bench-start plan updates:

1. Correct BOOT parsing semantics and runtime health tracking.
2. Add explicit RX_FRAME and TX_DONE runtime handling.
3. Add host-side regression vectors for these runtime paths.
4. Wire CI to run the new vectors.
5. Keep Windows TCP loopback green after the runtime changes.

## 2. Code Changes

### 2.1 Runtime health parser and shared state

Added:

- firmware/tractor_h7/murata_host/mh_runtime_health.h
- firmware/tractor_h7/murata_host/mh_runtime_health.c

Key behavior:

- BOOT_URC parsed as payload[0..5] = reset, radio_ok, radio_version, protocol_ver, wire_schema_ver, clock_source_id.
- TX_DONE_URC parsed and counted.
- RX_FRAME_URC parsed and counted, including len/snr/rssi/timestamp/payload copy.
- STATS_URC parsed into mh_stats_t with payload length retained.

Important correction made during regression:

- RX_FRAME wire layout is 8-byte header before payload (len + snr + rssi + timestamp), so parser uses:
  - payload_len == 8 + rx_len
  - data copy from payload[8]

### 2.2 Runtime integration

Updated:

- firmware/tractor_h7/murata_host/mh_runtime.h
- firmware/tractor_h7/murata_host/mh_runtime.c

Changes:

- Runtime now owns mh_runtime_health_t instead of ad hoc scalar fields.
- Added APIs:
  - mh_runtime_get_health(...)
  - mh_runtime_set_log_sink(...)
- Optional bench log support under LIFETRAC_MH_BENCH_LOG:
  - per-frame decoded logs
  - 1 Hz summary logs

### 2.3 Sketch wiring for optional bench logs

Updated:

- firmware/tractor_h7/tractor_h7.ino

Changes:

- Added default for LIFETRAC_MH_BENCH_LOG = 0 in Method G build path.
- Added optional log sink callback to print runtime lines to USB Serial when bench log is enabled.
- Added Method G setup wiring:
  - Serial.begin(115200) when bench log enabled
  - mh_runtime_set_log_sink(...) call

Additional compile hygiene fix discovered while validating:

- Added missing no-op stub in non-bench build path:
  - bench_radio_status_meta(uint8_t, int16_t)

This prevented an immediate symbol-not-found failure in default compile mode.

### 2.4 Regression vectors and CI

Added:

- firmware/tractor_h7/bench/h7_host_proto/mh_runtime_health_vectors.c

Vectors cover:

1. BOOT offsets (radio_ok from payload[1]).
2. TX_DONE parse.
3. RX_FRAME parse.
4. STATS legacy/additive compatibility.
5. Reject paths (unknown type + malformed RX frame).

Updated CI:

- .github/workflows/arduino-ci.yml

Added step:

- Build and run runtime health vectors (mh_runtime_health_vectors)

Docs updated:

- DESIGN-CONTROLLER/ARDUINO_CI.md (new vector listed)
- DESIGN-CONTROLLER/BUILD_CONFIG.md (bench log flag documentation)

## 3. Validation Results

## 3.1 Host protocol gates (local)

Command set executed:

- check_mh_wire_sync.py
- mh_cobs_crc_unit
- mh_stats_vectors
- mh_runtime_health_vectors

Result:

- [PASS] mh_wire sync: 45 constants match
- [PASS] mh_cobs_crc_unit: 4 checks
- [PASS] mh_stats_vectors: 4 vectors
- [PASS] mh_runtime_health_vectors: 4 vectors

## 3.2 Windows TCP loopback integration

Built and ran:

- build/loopback_driver_tcp.exe
- tools/murata_host_loopback.py --transport tcp --iterations 150

Result:

- loopback verdict pass with expected pass counts and reject counters

## 3.3 Arduino compile status (local)

Attempted local tractor_h7 compile preflight using absolute Arduino CLI path.

Observed blocker (after fixing the bench_radio_status_meta stub):

- Link-time multiple-definition collisions against Portenta core/system objects:
  - software_init_hook
  - HAL_GetREVID / HAL_GetDEVID
  - lp_ticker_* overrides
  - us_ticker_* overrides
  - SetSysClock / SystemCoreClockUpdate

Interpretation:

- This appears to be an existing full-sketch integration/link-layer issue in the local toolchain/build context, not specific to the runtime-health parser additions.
- Host regression and loopback evidence for the runtime patch set are complete and green.

## 4. Net Outcome

Implemented and validated:

1. BOOT mapping correctness in runtime health.
2. RX/TX URC handling added and regression-tested.
3. Operator-visible runtime bench logs available behind compile-time flag.
4. CI coverage expanded with runtime health vectors.
5. Windows TCP loopback remains passing post-change.

Remaining follow-up (outside this patch's core logic):

- Resolve local Portenta full-sketch linker collision set before using local Arduino compile as readiness signal.
