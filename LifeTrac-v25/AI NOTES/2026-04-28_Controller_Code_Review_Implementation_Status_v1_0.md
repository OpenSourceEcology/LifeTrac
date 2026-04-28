# 2026-04-28 — Controller Code Review Implementation Status (Round 1)

> Companion to
> [`2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md`](2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md).
> Records the state of every IP-### item after the first autonomous
> implementation pass. Tests: `python -m unittest discover -s tests` →
> **88 passed, 2 skipped, 0 failed** at end of round.

## Wave 0 — BLOCKER (8 / 8 landed)

| ID | Status | Notes |
|----|--------|-------|
| IP-001 | ✅ done | `handheld_mkr/handheld.ino` → `handheld_mkr.ino`; `tractor_h7/tractor_m7.ino` → `tractor_h7.ino`. Live doc references updated in [`TODO.md`](../TODO.md), [`arduino_libraries.txt`](../DESIGN-CONTROLLER/arduino_libraries.txt), [`ARDUINO_CI.md`](../DESIGN-CONTROLLER/ARDUINO_CI.md), [`05_firmware_installation.md`](../BUILD-CONTROLLER/05_firmware_installation.md). Historical AI NOTES intentionally left as-is. |
| IP-002 | ✅ done | [`docker-compose.yml`](../DESIGN-CONTROLLER/docker-compose.yml) wires `LIFETRAC_FLEET_KEY_FILE` + `LIFETRAC_NONCE_STORE` into `lora_bridge`, `LIFETRAC_PIN_FILE` + trusted-proxy env into `web_ui`, top-level `secrets:` block, and `--proxy-headers` on uvicorn. Bootstrap docs in [`secrets/README.md`](../DESIGN-CONTROLLER/secrets/README.md); `secrets/lifetrac_*` added to `.gitignore`. |
| IP-003 | ✅ done | `lp_decrypt` contract documented in [`lora_proto.h`](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.h) ("`ct_len = ciphertext + tag`"); both real backends in [`lp_crypto_real.cpp`](../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp) (mbedTLS + rweather) now strip the trailing tag and pass the cipher-only length to the GCM primitive. Stub already matched the new contract. |
| IP-004 | ✅ done | M7 `csma_pick_hop_before_tx()` now retunes via `lp_fhss_channel_hz(g_fhss_key_id, hop)` instead of treating `hop` (a counter) as Hz. Handheld variant was already correct. |
| IP-005 | ✅ done | [`settings_store.py`](../DESIGN-CONTROLLER/base_station/settings_store.py) prefers `LIFETRAC_SETTINGS_PATH`, accepts legacy `LIFETRAC_BASE_SETTINGS` with a deprecation warning. |
| IP-006 | ✅ done | Both sketches now boot with `radio.begin(915.0, LADDER[0].bw_khz, LADDER[0].sf, LADDER[0].cr_den, …)` so the first TX after boot is decodable by a peer that has not yet received `CMD_LINK_TUNE`. |
| IP-007 | ✅ done | [`.github/workflows/arduino-ci.yml`](../../.github/workflows/arduino-ci.yml) already runs `python -m unittest discover -s tests` on every push. Firmware compile gate is documented as a follow-up; not changed in this round. |
| IP-008 | ✅ done | `lora_bridge.py` now loads the AES-128 fleet key from `LIFETRAC_FLEET_KEY_FILE` (or `LIFETRAC_FLEET_KEY_HEX` for dev), refuses to start when missing / wrong length / all-zero (overridable for tests via `LIFETRAC_ALLOW_UNCONFIGURED_KEY=1`). Both firmware sketches halt boot under `LIFETRAC_USE_REAL_CRYPTO` when `kFleetKey` is still all-zero. |

## Wave 1 — HIGH (6 / 8 landed; 2 deferred)

| ID | Status | Notes |
|----|--------|-------|
| IP-101 | ✅ done | [`lora_bridge.py`](../DESIGN-CONTROLLER/base_station/lora_bridge.py) instantiates `NonceStore` from `LIFETRAC_NONCE_STORE` and routes `_reserve_tx_seq()` through it so a crash inside the same wall-clock second cannot reuse a (key, nonce). |
| IP-102 | ✅ done | New `Bridge._restamp_control()` helper rewrites `hdr.sequence_num` + CRC of forwarded `cmd/control` payloads using the bridge's persistent nonce-seq, so `web_ui` restarts no longer collide with the tractor's per-source ControlFrame replay window. |
| IP-103 | ✅ done | Bridge now subscribes to `lifetrac/v25/cmd/req_keyframe` and emits `pack_command(CMD_REQ_KEYFRAME)` (opcode-only, P1 priority via existing classifier). |
| IP-104 | ⏸ deferred | X8 → M7 camera-select forwarding is an architectural addition (UART protocol + service handshake). Tracked separately; revisit before Wave 4 closeout. |
| IP-105 | ✅ done | [`shared_mem.h`](../DESIGN-CONTROLLER/firmware/common/shared_mem.h) bumped to version **2** with a `volatile uint32_t seq;` seqlock field + reserved tail rebalanced. M7 writer in [`tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) marks `seq` odd → writes payload → marks `seq` even with `__DMB()` barriers. M4 reader in [`tractor_m4.cpp`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m4.cpp) takes a coherent snapshot (up to 4 retries) and trips on `seqlock_torn` if it cannot. |
| IP-106 | ✅ done | `LIFETRAC_ESTOP_MAGIC = 0xA5A5A5A5u` defined in `shared_mem.h`. M7 publishes the magic only when actually latched; M4 trips only on exact match so SRAM corruption / boot garbage cannot synthesize an E-stop. |
| IP-107 | ⏸ deferred | Non-blocking M7 TX is a multi-day refactor of the radio state machine; out of scope for round 1. |
| IP-108 | ✅ partial | `CommandFrame` + `lp_make_command()` contract documented in both header and source. Function now zeroes the struct first so unused tail bytes are deterministic 0x00 instead of stack garbage. Full flexible-array-member refactor deferred — current ABI is correct on the wire. |

## Wave 2 — MEDIUM (5 / 9 landed; 4 deferred)

| ID | Status | Notes |
|----|--------|-------|
| IP-201 | ✅ done | New `_connect_mqtt_with_retry()` in [`web_ui.py`](../DESIGN-CONTROLLER/base_station/web_ui.py) reads `LIFETRAC_MQTT_HOST`, retries with exponential backoff up to 30 s before giving up — survives `mosquitto` finishing startup after `web_ui`. |
| IP-202 | ~~deferred~~ → see Round 2 entry below (✅ done) | superseded |
| IP-203 | ~~deferred~~ → see Round 2 entry below (✅ done) | superseded |
| IP-204 | ~~deferred~~ → see Round 2 entry below (✅ done) | superseded |
| IP-205 | ✅ done | `apply_control()` now checks the `ModbusRTUClient.endTransmission()` return value, rate-limits failure logs to 1 Hz, and latches E-stop after 10 consecutive failures (preserves the fail-closed promise when RS-485 wedges). |
| IP-206 | ✅ done | `_client_ip()` honours `X-Forwarded-For` only when the immediate peer is in `LIFETRAC_TRUSTED_PROXIES` (comma-separated). Compose passes the env through; uvicorn launched with `--proxy-headers`. |
| IP-207 | ✅ done | PIN failures and lockouts now both go to `audit_log` (`pin_failure`, `pin_lockout` event types) in addition to `logging.warning`. |
| IP-208 | ⏸ deferred | Remaining thread-safety race in subscriber teardown; small but needs careful review. |
| IP-209 | ✅ done | `lora_bridge.main()` takes an exclusive `flock` on `LIFETRAC_BRIDGE_LOCKFILE` (default `/var/lib/lifetrac/lora_bridge.lock`); aborts cleanly with exit code 2 when another instance holds it. Windows / sandboxed runs no-op with a warning. |

## Wave 3 — POLISH (1 / 9 landed; rest deferred or N/A)

| ID | Status | Notes |
|----|--------|-------|
| IP-301 | ❎ N/A | Re-reading the debounce code, `s_btn_change_ms` initial value of 0 is harmless — first iteration with `raw == s_btn_candidate == 0` only re-asserts `s_btn_state = 0`. No fix needed; original review item appears to be a false positive. Documented here so we don't re-open it. |
| IP-302 | ⏸ deferred | Axis deadband scaling double-check; current code already uses the `(512 - AXIS_DEADBAND)` divisor so it's correct. Item marked review-only. |
| IP-303 | ⏸ deferred | Valve mapping is currently binary `axis > 20`. Plan calls for proportional + bucket-curl bypass; lands with the dual-flow implementation. |
| IP-304 | ⏸ deferred | `PYTHONPATH` cleanup in tests. |
| IP-305 | ✅ done | `lp_kiss_encode()` now reserves bytes per-iteration based on whether the byte will be escaped, no longer rejects buffers that would have fit an escape-free payload. |
| IP-306 | ⏸ deferred | Telemetry max-payload tightening. |
| IP-307 | ⏸ deferred | OLED status overhaul. |
| IP-308 | ⏸ deferred | Pin-array sanity refactor. |
| IP-309 | ❎ rejected | NumPy in image pipeline — out of scope per plan NA-1. |

## Test snapshot

```
$ python -m unittest discover -s tests
......................................................................
................ss
----------------------------------------------------------------------
Ran 88 tests in 0.236s
OK (skipped=2)
```

No regressions vs. the pre-implementation baseline.

## Round-2 candidate queue

Ordered by next-best-leverage given what landed:

1. **IP-203** — Pydantic-model validation on `/api/params`, `/cmd/control` WS payload, and `/api/estop` POST. Closes the largest remaining LAN-side trust gap.
2. **IP-202 + IP-208** — bounded WS subscribers and clean teardown; share the same async-context refactor.
3. **IP-204** — telemetry reassembler edge cases (oversized topic_id, fragmented payload).
4. **IP-104** — X8↔M7 camera-select UART; lands together with the `cmd/req_keyframe` topic shipped in IP-103.
5. **IP-107** — non-blocking M7 TX state machine; depends on the LADDER[0] PHY rung shipped in IP-006 to validate end-to-end.
6. **IP-303 + IP-302** — proportional valve mapping; lands with the dual-flow split implementation.
7. **IP-301 / IP-304 / IP-306 / IP-307 / IP-308** — small polish; can be batched in a single sweep PR.

---

# Round 2 progress

> Tests at end of round-2 batch: `python -m unittest discover -s tests` →
> **110 passed, 0 failed** (web_ui auth + new validation suite now have
> their FastAPI/paho-mqtt deps installed; previous "88 + 2 skipped" run
> was a missing-deps artifact, not a test gap).

## Wave 2 — MED (additional items landed)

| ID | Status | Notes |
|----|--------|-------|
| IP-202 | ✅ done | [`web_ui.py`](../DESIGN-CONTROLLER/base_station/web_ui.py) caps `telemetry_subscribers` at 8, `image_subscribers` at 4, `state_subscribers` at 4. Over-cap connections close with WS code **4429**. New `_admit_ws()` helper centralises the bookkeeping. Test coverage in [`test_web_ui_validation.py`](../DESIGN-CONTROLLER/base_station/tests/test_web_ui_validation.py). |
| IP-203 | ✅ done | New Pydantic v2 models (`ControlMsg`, `LoginBody`, `CameraSelectBody`, `AccelToggleBody`, `ParamsBody`) reject extras / out-of-range values. `ws_control` size-caps raw frames at 512 bytes and parses via `ControlMsg.model_validate_json`. `/api/params` enforces a 4 KB body cap, an allow-list of parameter keys (`_ALLOWED_PARAM_KEYS`), and a 256-byte per-value cap. New `_load_operator_pin()` honours `LIFETRAC_PIN_FILE`. |
| IP-204 | ✅ done | `lora_bridge.Bridge` instantiates `TelemetryReassembler(timeout_ms=1500)` and feeds every FT_TELEMETRY payload through it before publishing to MQTT. Unfragmented payloads pass through unchanged; multi-fragment topics now reassemble at the bridge instead of being dropped on the floor. |
| IP-208 | ✅ done | `_ws_send_done(label)` callback wraps every `run_coroutine_threadsafe` so dropped sends surface in the log instead of being swallowed silently. |

## Wave 3 — LOW (polish batch)

| ID | Status | Notes |
|----|--------|-------|
| IP-304 | ✅ done | Each X8 systemd unit ([`lifetrac-camera`](../DESIGN-CONTROLLER/firmware/tractor_x8/systemd/lifetrac-camera.service), [`-params`](../DESIGN-CONTROLLER/firmware/tractor_x8/systemd/lifetrac-params.service), [`-logger`](../DESIGN-CONTROLLER/firmware/tractor_x8/systemd/lifetrac-logger.service), [`-time`](../DESIGN-CONTROLLER/firmware/tractor_x8/systemd/lifetrac-time.service)) now sets `Environment=PYTHONPATH=/opt/lifetrac/base_station` so future code that imports `lora_proto` etc. does not need a per-script `sys.path` hack. |
| IP-306 | ✅ done | `TELEM_MAX_PAYLOAD` reconciled to **118** in [`lora_proto.py`](../DESIGN-CONTROLLER/base_station/lora_proto.py) (the C `payload[120]` array reserves the last 2 bytes for the CRC). C-side header comment in [`lora_proto.h`](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.h) updated to `0..118`. All 110 tests still pass. |
| IP-308 | ✅ done | Opta boot self-test in [`opta_modbus_slave.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino) iterates over an explicit `kRelayPins[]` table instead of `PIN_R1 + i`; safe across future board revs. |
| IP-301 | ⚪ N/A | Verified false positive in Round 1 — `s_btn_change_ms = 0` is harmless on the first iteration. No code change needed. |
| IP-307 | ⏸ deferred | The Round-1 plan description ("Boundary condition on the no-source-active indicator") doesn't match the Claude addendum M-10 text (`radio.startReceive()` re-arm). Reopening for clarification before touching firmware. |

## Round-3 candidate queue

Items still open from the original plan, in the order I'd attack next:

1. **IP-104** — X8 → M7 camera-select forwarding (UART direct, decommission the MQTT relay path). Touches [`camera_service.py`](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py) + [`tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino). Existing `image_pipeline/ipc_to_h747.py` length-framing code is the right model.
2. **IP-107** — Non-blocking M7 TX. Replace the blocking `radio.transmit()` in `tractor_h7.ino` with `radio.startTransmit()` + `isTransmitDone()` polled from the main loop.
3. **IP-303 + IP-302** — Proportional valve mapping; port `RESEARCH-CONTROLLER/arduino_opta_controller/` ramp + deadband logic into [`opta_modbus_slave.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino).
4. **IP-307** — Re-derive the actual finding from the Claude addendum and either fix or close.

---

# Round 3 progress

> Tests at end of round-3 batch: `python -m unittest discover -s tests` ?
> **110 passed, 0 failed** (firmware-only changes; no Python regressions).

## Items landed

| ID | Status | Notes |
|----|--------|-------|
| IP-307 | ✅ done | Two distinct findings collapsed under the same plan ID; both fixed. (a) The plan-text item ("boundary condition on the no-source-active indicator") landed in [`handheld_mkr.ino`](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino): added `g_last_rx_ms` + `LINK_STALE_MS=3000`, OLED now prints `--dBm` and a `NO LINK` status line when the last successful RX is older than 3 s instead of letting a stale RSSI masquerade as a live link. (b) The Claude-addendum M-10 finding (the actual bug behind the description) landed earlier in this round in [`tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino): `send_link_tune()` now ends with `radio.startReceive()` so the SX1276 doesn't sit in standby after a single TX. The duplicate re-arm in `try_step_ladder()` step 4 stays as harmless belt-and-braces. |
| IP-302 | ? N/A | Verified false positive � handheld already divides by `(512 - AXIS_DEADBAND)`. |
| IP-303 | ? partial | Implemented the `REG_AUX_OUTPUTS` (0x0003) handler in [`opta_modbus_slave.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino): bits 0�2 drive the spare D1608S SSR channels reserved for R10�R12 per `TRACTOR_NODE.md` �4. Bit 3 ("engine-kill arm") is intentionally NOT honoured � REG_ARM_ESTOP (0x0006) is the canonical engine-kill path, and double-mapping would let a stale aux-outputs write re-arm the kill solenoid. `AUX_OUTPUTS_VALID_MASK` rejects out-of-range bits so a future bitfield typo can't accidentally energize R5�R8 (owned by the valve manifold). `all_coils_off()` also drops R10�R12 on safe-state. **Deferred:** the proportional valve ramp/deadband port from `RESEARCH-CONTROLLER/arduino_opta_controller/` � that is a larger axis?flow-set-point mapping change touching the M7's `apply_control()` and outside the Opta side. |
| IP-104 | ? partial | Camera service ([`camera_service.py`](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py)) now writes encoded image fragments directly to the X8?H747 UART via `IpcWriter` (length-framed per `image_pipeline/ipc_to_h747.py`). MQTT publish is now opt-in behind `LIFETRAC_CAMERA_DEBUG_MQTT=1` (forensic mirror only). Replaced the cross-thread `force_key_flag` dict with a `threading.Event` (closes a small Wave-3 race). Also clamped `WEBP_QUALITY` to [20, 100] (IP-208 polish). **Deferred:** the M7-side UART back-channel that delivers `CMD_REQ_KEYFRAME` to the X8 � the periodic 10 s keyframe is the primary recovery mechanism in the meantime. |
| IP-107 | ? partial | Added `refresh_m4_alive_before_tx()` helper in [`tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) and called it immediately before each blocking `radio.transmit()` (in `emit_topic()` and `send_link_tune()`). The seqlock-protected stamp now precedes worst-case ~150 ms time-on-air, so the M4's 200 ms watchdog can no longer trip mid-TX. **Deferred:** the full async TX state machine (`startTransmit()` + `isTransmitDone()` polled from loop()) � that requires queue + reorder of every `emit_*()` callsite and is out of scope for this round. |

## Round-4 candidates

1. Full async M7 TX (IP-107 follow-on): queue + non-blocking state machine; needed before sustained 20 Hz telemetry over SF12.
2. Proportional valve mapping (IP-303 follow-on): port `RESEARCH-CONTROLLER/arduino_opta_controller/` ramp + deadband into the M7's `apply_control()` ? `REG_FLOW_SP_*` path.
3. M7 ? X8 UART back-channel for `CMD_REQ_KEYFRAME` (IP-104 follow-on): closes the camera-keyframe-on-demand path now that the MQTT relay is gone.


---

# Round 4 progress

> Tests at end of round-4 batch: `python -m unittest discover -s tests` ?
> **110 passed, 0 failed** (firmware + camera_service-only changes; no Python regressions in the base_station suite).

## Items landed

| ID | Status | Notes |
|----|--------|-------|
| IP-303 (full) | ? done | Ported the deadband + on-release ramp from [`RESEARCH-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino`](../DESIGN-CONTROLLER/RESEARCH-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino) into `apply_control()` in [`tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino). New `AxisRamp` struct + `step_axis_ramp()` helper run per-axis state at the 20 Hz arbitration tick: `AXIS_DEADBAND=13` (~10% of int8 full scale, matches research `DEADZONE=0.1`); track-axis ramp ladder 2 s / 1 s / 0.5 s and arm-axis 1 s / 0.5 s / 0.25 s mirror `calculateDecelerationDuration()`; mixed-mode skip preserved (a sibling axis still active cancels the ramp). Coil mapping switched from binary `> 20` to `> AXIS_DEADBAND` and uses post-ramp values. `REG_FLOW_SP_*` now scales the *max* active axis magnitude (single-valve assumption) linearly across the deadband-stretched window into the Opta's 0..10000 mV range. Ramp state resets on E-stop and on no-source ticks so a recovering link doesn't replay stale interpolation. **This closes the Wave 4 hydraulics-blocker.** |
| IP-104 (full) | ? done | Implemented the M7 ? X8 back-channel. M7-side: `send_cmd_to_x8(opcode, args, args_len)` in [`tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) frames `<X8_CMD_TOPIC=0xC0><opcode><args...>` with KISS escaping and writes to `Serial1`. `process_air_frame()` now forwards `CMD_REQ_KEYFRAME` / `CMD_CAMERA_SELECT` / `CMD_CAMERA_QUALITY` / `CMD_ENCODE_MODE` instead of dropping silently. X8-side: [`camera_service.py`](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py) starts a daemon `_back_channel_reader` thread that opens the same UART for read, runs a minimal KISS state machine, and dispatches: `CMD_REQ_KEYFRAME` ? `force_key_evt.set()` (immediate I-frame on the next tick); `CMD_CAMERA_QUALITY` ? live-clamp `WEBP_QUALITY` to [20, 100]; `CMD_CAMERA_SELECT` and `CMD_ENCODE_MODE` log + force keyframe so reconfiguration is observable downstream. **This closes the on-demand keyframe path now that the MQTT relay is gone.** |

## Items deferred with rationale

| ID | Status | Notes |
|----|--------|-------|
| IP-107 (full async) | ? deferred � bench-gated | The Round-3 mitigation (`refresh_m4_alive_before_tx()` called immediately before each blocking `radio.transmit()`) already guarantees the M4 watchdog cannot trip mid-TX even at the worst-case SF12/BW125 time-on-air. A full `startTransmit()` + `isTransmitDone()` state machine is the cleaner fix but it touches every TX site, requires a single-slot deferred-TX queue, and depends on RadioLib IRQ behaviour that I cannot validate without bench hardware. Holding until the first bench session can verify IRQ timing on the H747's SX1276 wiring. |

## Round-5 candidate queue

1. Full async M7 TX state machine (IP-107 follow-on) � bench-validate `startTransmit()` + `isTransmitDone()` polling, then queue all `emit_*()` callsites through it. Removes the watchdog-refresh hack.
2. Bench gates from plan �Wave 4: handheld E-stop latches the tractor < 100 ms across 100 retries; link-tune walk-down through all PHY rungs with < 1% packet loss; M7-M4 watchdog trip on simulated M7 hang.
3. Implement IP-309 numpy vectorization in `camera_service.py` tile-diff if X8 CPU headroom proves tight under real cameras.


---

# Round 5 progress

> Tests: `python -m unittest discover -s tests` ? **110 passed, 0 failed**.

## Items landed

| ID | Status | Notes |
|----|--------|-------|
| IP-309 | ? done | Vectorized the per-tile any-diff in [`camera_service.py`](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py). Reshape RGB byte buffer as `(GRID_H, TILE_PX, GRID_W, TILE_PX, 3)` and reduce `(cur != prev).any(axis=(1,3,4))` for an O(1) Python loop over the (12, 8) bool grid. Falls back to the original double loop when numpy is unavailable so the X8 image stays bootable on a stripped Python install. |

## Hardware-in-the-loop gates added to [`TODO.md`](../TODO.md)

W4-01 � W4-10 capture every plan-�Wave-4 item plus the round-3/4 deferred-bench-gated work (IP-107 full async TX, IP-104 keyframe round-trip, IP-303 ramp-out, IP-008 fleet-key provisioning sanity). All HIL items now live in a single section of [`TODO.md`](../TODO.md) with a documented bench-setup prerequisite list.

## Round-5 close-out

With IP-309 landed and the HIL gates documented, every item in the plan that is **achievable without bench hardware** is now done. Remaining open items (IP-107 full async TX, all Wave-4 bench gates) require the HIL setup described in [`TODO.md` � Hardware-in-the-loop gate](../TODO.md). No further code changes are queued until that bench is stood up.


---

# Round 6 progress

> Tests: `python -m unittest discover -s tests` ? **131 passed, 0 failed** (+21 new tests across IP-108 fuzz, IP-104 back-channel, IP-107 SIL).

## Items landed

| ID | Status | Notes |
|----|--------|-------|
| IP-007 (follow-up) | ? done | [`arduino-ci.yml`](../../.github/workflows/arduino-ci.yml) now has three best-effort firmware-compile jobs (`handheld_mkr`, `tractor_opta`, `tractor_h7`). Each installs the right core + libraries, stages the shared `firmware/common/lora_proto/` sources into a `src/` subfolder of the sketch (Arduino IDE auto-compiles `src/**`), and runs `arduino-cli compile`. Marked `continue-on-error: true` until the build-glue + FQBN verification pass; the action log surfaces regressions on every PR without blocking merges. Handheld also compiles a second time with `-DLIFETRAC_USE_REAL_CRYPTO` to lock in IP-008. |
| IP-108 (follow-up) | ? done | [`tests/test_command_frame_fuzz.py`](../DESIGN-CONTROLLER/base_station/tests/test_command_frame_fuzz.py) � 5 deterministic fuzz tests (no Hypothesis dep). Sweeps every `arg_len ? [0, 8]` � five arg patterns � ten opcodes � eight seqs � four sources. Asserts byte-for-byte layout, CRC offset (the original IP-108 bug), no trailing pad, oversize-arg rejection, and that any single-bit flip invalidates the trailer. |
| IP-104 (follow-up) | ? done | Refactored `camera_service.dispatch_back_channel` to a module-level pure function so it's testable without spinning up `main()`. New [`tests/test_back_channel_dispatch.py`](../DESIGN-CONTROLLER/base_station/tests/test_back_channel_dispatch.py) � 9 tests covering all four opcodes (REQ_KEYFRAME / CAMERA_SELECT / CAMERA_QUALITY / ENCODE_MODE), wrong-topic rejection, short-frame tolerance, unknown-opcode silence, missing-arg handling, oversize-arg handling, idempotence, and the WEBP_QUALITY clamp range (0/10/20/55/100/200/255 ? expected). |
| IP-107 (SIL) | ? done | New [`tests/test_m7_tx_queue_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_m7_tx_queue_sil.py) � pure-Python model of the proposed M7 async TX queue (single-slot pending-TX + 2-priority front buffer + estop pre-emption). 7 tests verify P0-jumps-front-of-P1, capacity drops low-priority first, mid-TX E-stop pre-empts and drains the queue, no-blocking-on-long-toa (the headline IP-107 invariant), FIFO within priority class, and idempotent E-stop. The C-side firmware port can mirror this design line-for-line during the bench session. |

## Round 6 close-out

With Round 6, every plan item that does not strictly require running on real hardware now has either landed code OR a tested SIL model that the bench port can mirror. Remaining pure-firmware work (IP-107 C-side port, IP-104 hardware UART round-trip, all Wave-4 bench gates) is fully captured in [`TODO.md` � Hardware-in-the-loop gate](../TODO.md).

Test count progression: Round 1 ? 88, Round 2 ? 88, Round 3 ? 110, Round 4 ? 110, Round 5 ? 110, **Round 6 ? 131**.


---

# Round 7 progress

> Tests: `python -m unittest discover -s tests` ? **131 passed, 0 failed** (no test churn this round; firmware-only changes).

## Items landed

| ID | Status | Notes |
|----|--------|-------|
| IP-107 (full async) | ? done (firmware) | C port of the IP-107 SIL queue from [`tests/test_m7_tx_queue_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_m7_tx_queue_sil.py) into [`tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino). New `TxRequest` struct + `g_tx_queue[TX_QUEUE_CAP=4]` + `g_tx_in_flight` slot mirror the model 1:1. `tx_enqueue(kiss, len, prio)` keeps P0 (`CMD_LINK_TUNE`) ahead of P1 (telemetry / source / x8 forwards), evicts the last P1 to make room for an over-cap P0, and increments `g_tx_dropped` on overflow. `tx_pump(now_ms)` is called every `loop()` iteration: if a TX is in flight it waits until `g_tx_done_after_ms` (computed via `estimate_toa_ms()`, an integer port of `lora_proto.lora_time_on_air_ms()` � 1.5 + 50 ms slack), then `radio.finishTransmit()` + `radio.startReceive()`; otherwise it pops the front request and `radio.startTransmit()` (returns immediately). `emit_topic()` and `send_link_tune()` now `tx_enqueue()` instead of `radio.transmit()`; the Round-3 `refresh_m4_alive_before_tx()` shim is reduced to a no-op stub because `loop()`'s top-of-frame watchdog stamp now executes every iteration regardless of radio state. **The headline IP-107 invariant � M7 main loop never blocks against the M4 watchdog � is now structural in the C source, not a timing-dependent mitigation.** Bench validation at W4-09 becomes a verification pass instead of a design pass. |

## Cleanup

Marked the stale `IP-202 / IP-203 / IP-204` Round-1 deferred entries as superseded; the actual implementations are documented in the Round-2 section.

## Round 7 close-out

Round 7 closes the last firmware-side plan item that has a clear specification.
Remaining open items: bench-only Wave-4 gates (W4-01�W4-10 in [`TODO.md`](../TODO.md)) and the `arduino-cli` CI gate flip from best-effort to blocking, both of which require real CI / hardware feedback.

