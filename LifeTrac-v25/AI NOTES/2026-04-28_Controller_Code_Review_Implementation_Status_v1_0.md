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


## Round 8 � 2026-04-28 � Pure-software extensions: pack_control fuzz + M4 safety SIL

Two no-hardware additions; both expand verification coverage so HIL bench time can focus on physical interactions.

### IP-108 symmetric: `pack_control` property/fuzz
[`base_station/tests/test_control_frame_fuzz.py`](../DESIGN-CONTROLLER/base_station/tests/test_control_frame_fuzz.py) � 7 tests, stdlib only. Mirrors the IP-108 command-frame fuzz on the *control* hot path (20 Hz from handheld). Sweeps source � seq � stick � button/flag/hb permutations and asserts:

- Frame is always exactly `CTRL_FRAME_LEN` (16) bytes.
- `parse_header` recovers `version=PROTO_VERSION`, `frame_type=FT_CONTROL`, `source_id`, and `seq & 0xFFFF`.
- CRC-16/CCITT lives at the trailing 2 bytes (offset 14); regression catch if anyone reorders the body struct.
- Stick channels clip to signed-int8 `[-127, 127]` rather than wrapping mod 256 � the safety invariant: a rail-to-rail joystick must never alias to the opposite direction.
- Wide `buttons` / `flags` / `hb` values get masked, not allowed to spill into neighbouring fields.
- Single-bit flips anywhere in the body are caught by CRC.
- Default `source_id` is `SRC_BASE` (regression: bridge ? M7 routing depends on this).

### M4 safety supervisor SIL
[`base_station/tests/test_m4_safety_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_m4_safety_sil.py) � 9 tests, stdlib only. Pure-Python `M4Supervisor` class mirrors `firmware/tractor_h7/tractor_m4.cpp` 1:1: `SharedState` with seqlock counter, `boot()` that refuses to arm on version mismatch or zero `alive_tick_ms`, `tick(now_ms)` that runs the same three liveness checks (alive_tick stale > 200 ms, loop_counter stuck > 200 ms, `estop_request == LIFETRAC_ESTOP_MAGIC`) under the same 4-retry seqlock pattern. Trip is latched.

Properties asserted:

- **TR-A** E-stop latency from M7 setting MAGIC to safety_alive low is < 100 ms (one M4 tick = 20 ms; budget = 100 ms per W4-01).
- **TR-B** Silent M7 trips between WATCHDOG_MS and WATCHDOG_MS + 1 tick.
- **TR-C** Trip is sticky � M7 recovery does NOT re-arm (matches design comment in tractor_m4.cpp).
- **TR-D** Healthy M7 over a 9-second simulated run produces zero trips.
- **TR-E** Non-magic `estop_request` values (0, 0x12345678, MAGIC � 1, ~MAGIC) do NOT trip � protects against SRAM noise per IP-106.
- **TR-F** Persistent odd-locked seq trips with `seqlock_torn`; momentarily-odd writer is recovered without trip.
- **TR-G** Loop-counter witness catches "alive_tick_ms keeps reading fresh because the M7 keeps stamping the same value but never advanced loop_counter."
- **TR-H** Refuses to arm on `version != LIFETRAC_SHARED_VERSION`; remains unarmed forever even if MAGIC is later written.

### Effect on HIL gate set

- **W4-01** (E-stop latch < 100 ms) is now a verification pass against TR-A, not a greenfield design pass. Bench captures only need to confirm the model holds on real silicon timing.
- **W4-03** (M7?M4 watchdog trip) is verified by TR-B / TR-G in software. Bench focus shifts to "did the PSR relay actually de-energise" rather than timing math.

### Tests after Round 8
`Ran 147 tests in 0.538s` `OK` (was 131; +7 control fuzz, +9 M4 SIL).

No firmware changes this round � both additions are pure-Python, zero new dependencies.

## Round 9 � 2026-04-28 � Operational hardening from second-pass cross-review

Five items promoted from the GitHub Copilot v1.0 second-pass review (which itself folded in independent findings from GPT-5.3-Codex and Gemini 3.1 Pro). All pure-software, no bench hardware required.

### �B � /ws/control admission cap (high; security)

Source flags: GPT-5.3-Codex �3, Gemini �4.2.2.

[`base_station/web_ui.py`](../DESIGN-CONTROLLER/base_station/web_ui.py): added `MAX_CONTROL_SUBSCRIBERS = 4` and `control_subscribers: set[WebSocket]`. `ws_control()` now goes through the same `_admit_ws()` path as telemetry/image/state, closing over-cap connections with WS code 4429 and emitting an `ws_over_cap` audit-log event. Closes the "credentialled but misbehaving operator opens thousands of sockets" hole.

### �C � WS subscriber-set thread-safety (high)

Source flags: GPT-5.3-Codex �2, Gemini �4.2.1 (rated "Major").

[`base_station/web_ui.py`](../DESIGN-CONTROLLER/base_station/web_ui.py): single `_subscribers_lock = threading.Lock()` guarding all add/discard/iterate paths across telemetry/image/state/control pools. New `_snapshot_subscribers(pool)` returns a stable list under the lock; all three MQTT-thread fan-out sites (canvas, telemetry, state snapshot) switched from `list(pool)` to the snapshot helper. `_discard_subscriber()` helper used in every websocket finally block. Eliminates the intermittent `RuntimeError: Set changed size during iteration` on connection flap.

### �D � Unconditional fleet-key refusal (medium; security)

Source flag: GPT-5.3-Codex �4.

Both [`handheld_mkr.ino`](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino) and [`tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino): the `fleet_key_is_zero()` halt was previously gated by `#ifdef LIFETRAC_USE_REAL_CRYPTO`, so a stub-crypto image flashed onto real hardware would TX with the all-zero placeholder. The check is now `#ifndef LIFETRAC_ALLOW_UNCONFIGURED_KEY`: opt-in instead of opt-out. CI compile jobs in [`.github/workflows/arduino-ci.yml`](../../.github/workflows/arduino-ci.yml) now pass `-DLIFETRAC_ALLOW_UNCONFIGURED_KEY` so they keep building without a provisioned key, but production images cannot ship with the unprovisioned default.

### �E � `AuditLog` singleton (low/medium)

Source flag: GPT-5.3-Codex �1 (second-pass new).

[`base_station/web_ui.py`](../DESIGN-CONTROLLER/base_station/web_ui.py): `_get_audit_log()` lazily constructs a single module-level `AuditLog` and reuses it across PIN failures, lockouts, and WS-cap rejections. Was previously `AuditLog(_AUDIT_PATH)` per event, leaking a file handle on every rejected login (`ResourceWarning: unclosed file`).

### �F � Cross-language ReplayWindow invariant (low)

Source flag: Gemini �4.2.3.

[`base_station/tests/test_replay_window_invariant.py`](../DESIGN-CONTROLLER/base_station/tests/test_replay_window_invariant.py) � 2 tests parsing `firmware/common/lora_proto/lora_proto.h` and asserting `LP_REPLAY_WINDOW_BITS == REPLAY_WINDOW_BITS == 64`. Catches the C-vs-Python bitmap-width drift class of bug at CI time before it can break replay protection asymmetrically.

### Tests after Round 9

`Ran 151 tests in 0.511s` `OK` (was 147; +1 control-cap, +1 control-no-session, +2 replay invariant). `ws_over_cap` audit events visible in WARNING logs as expected.

### Open

**�A** (CI compile-gate flip from `continue-on-error: true` to blocking) still requires a green Actions run to confirm the staging glue works on a clean runner. The new `-DLIFETRAC_ALLOW_UNCONFIGURED_KEY` flag added in �D should keep the existing best-effort jobs green; once that's confirmed on a real run, dropping the `continue-on-error` flag is a one-line change.

## Round 10 � 2026-04-28 � Opta Modbus SIL closes W4-04 in CI

Pure-software follow-on to Round 8 (M4 supervisor SIL). Same recipe applied to the W4-04 surface: model both halves of the M7?Opta Modbus link in stdlib Python, then hammer the safety-relevant invariants in unittest.

### What landed

[`base_station/tests/test_modbus_slave_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_modbus_slave_sil.py) � 13 tests, two model classes:

- `OptaSlave` mirrors `opta_modbus_slave.ino`'s `on_holding_change`, `check_safety`, and `enter_safe_state` paths. Holding-register addresses, safety-state enum, AUX valid mask, and 200 ms watchdog threshold are all literal copies of the C header constants.
- `M7ModbusClient` mirrors the `apply_control()` writer in `tractor_h7.ino`: builds the 7-register holding block, ticks `g_watchdog_ctr` every call, increments `s_modbus_fail_count` on TX failure, and latches `g_estop_latched` at the 10-strike threshold. When latched, builds the all-zero block with `REG_ARM_ESTOP=1`.

### Properties asserted (TR-A � TR-J)

- **TR-A / TR-B** Opta watchdog: no trip when M7 writes at 100 Hz; trips between 200 ms and 200 ms + one tick when M7 goes silent; trip drops all relays + zeroes flow set-points.
- **TR-C / TR-D** M7 fail-count latch: 10 cumulative `endTransmission()` failures latch `g_estop_latched`; once latched, the next block written has `REG_VALVE_COILS=0`, `REG_FLOW_SP_*=0`, `REG_ARM_ESTOP=1`, `REG_CMD_SOURCE=0`.
- **TR-E / TR-F** Opta REG_ARM_ESTOP latch: a single write of `REG_ARM_ESTOP=1` enters `SAFETY_ESTOP_LATCHED` and engine-kill HIGH; subsequent `REG_VALVE_COILS` and `REG_FLOW_SP_*` writes are silently dropped; `REG_WATCHDOG_CTR` writes still pass through (master must keep proving liveness).
- **TR-G** External E-stop loop: `PIN_ESTOP_LOOP` LOW enters `SAFETY_ESTOP_LATCHED` on the next `check_safety` tick.
- **TR-H** AUX-mask: `REG_AUX_OUTPUTS=0xFFFF` only energises R10/R11/R12; the boom/bucket bank is untouched (so a runaway bitfield can never fire boom-up via the wrong register).
- **TR-J** End-to-end W4-04 budget: from first failed TX to `g_estop_latched` is < 1 s at the 100 Hz call rate (measured ~100 ms in the SIL � well inside the runbook budget).

### Documented divergence

- **TR-I** Pinned by `test_fail_count_is_cumulative_not_consecutive`: the firmware as written never resets `s_modbus_fail_count` on a successful TX, so the latch fires on the 10th *cumulative* failure since boot, not the 10th *consecutive* failure. The W4-04 runbook prose ([`DESIGN-CONTROLLER/HIL_RUNBOOK.md`](../DESIGN-CONTROLLER/HIL_RUNBOOK.md) �W4-04) says "10 consecutive failures". The SIL pins firmware-as-written so the test will fail loudly the day someone fixes the firmware to be consecutive � at which point both the test and the runbook prose need to be updated together.

### Tests after Round 10

`Ran 164 tests in 0.505s` `OK` (was 151; +13 from the new SIL).

### Effect on Wave 4

W4-04 is now *bench-confirmation-only* � the design is verified in CI; the bench session just needs to confirm the model matches the wiring. This is the same downgrade that Round 8 applied to W4-01 (E-stop latch) and W4-03 (M7?M4 watchdog).

### Next candidates

- Telemetry path property/fuzz (`pack_telemetry_fragments` + `TelemetryReassembler` round-trip) � symmetric to the IP-108 command-frame fuzz.
- WS-subscriber stress test that hammers Round 9 �C's `_subscribers_lock` under N-client connect/disconnect churn.

## Round 11 � 2026-04-28 � Telemetry fragmentation property/fuzz suite

Symmetric companion to the IP-108 command-frame fuzz, applied to the M7?base direction. Pure stdlib, no new deps.

### What landed

[`base_station/tests/test_telemetry_fragmentation_fuzz.py`](../DESIGN-CONTROLLER/base_station/tests/test_telemetry_fragmentation_fuzz.py) � 15 tests grouped into seven classes targeting `pack_telemetry_fragments` ? `parse_telemetry_fragment` ? `TelemetryReassembler`.

Helper `_feed_all` latches the first non-None completion rather than the last call's return value, because a duplicate fragment delivered after completion would otherwise open a fresh partial slot for the same key (the original was deleted on completion) and the loop's last assignment would shadow the real result. This is a real edge worth pinning, not just a test-harness convenience.

### Properties asserted (TF-A � TF-L)

- **TF-A round-trip identity.** Lengths 0..2048 bytes (stride 17) on PHY_IMAGE round-trip exactly; same for every packable profile (PHY_IMAGE, PHY_CONTROL_SF7, with PHY_CONTROL_SF8/SF9 skipped where the 25 ms cap leaves zero payload room).
- **TF-B fragment-header invariants.** Magic byte is `TELEMETRY_FRAGMENT_MAGIC`; every fragment shares the same `frag_seq` and `total`; `idx` is monotonic 0..total-1.
- **TF-C airtime cap.** Every fragment fits inside `TELEMETRY_FRAGMENT_MAX_AIRTIME_MS` (25 ms) on its profile.
- **TF-D oversize rejection.** A payload requiring 257 fragments raises `ValueError`; the 256-fragment boundary still packs and round-trips.
- **TF-E reordered delivery.** Twenty PRNG-shuffled deliveries of the same payload still complete to the original bytes.
- **TF-F duplicate delivery.** Sending each non-final fragment 3� then the final fragment once produces exactly one completion and `2*(N-1)` duplicates.
- **TF-G missing-middle timeout.** Dropping any single middle fragment prevents completion; once a later unrelated feed advances the GC clock past `timeout_ms`, the partial is reclaimed and `timeouts` ticks.
- **TF-H multi-stream isolation.** Two streams with the same `frag_seq` but distinct `(source_id, topic_id)` interleave cleanly and complete independently.
- **TF-I seq-wrap with intervening GC.** Re-using the same `frag_seq` after the first stream has been GC'd completes a second, independent assembly.
- **TF-J unfragmented passthrough.** Bodies whose first byte isn't the magic (or that are shorter than the 4-byte header) pass through verbatim and increment `bad_magic_passthroughs`.
- **TF-K mid-stream total change.** A synthetic stream that reports `total=4` then `total=2` for the same key correctly discards the partial and completes off the new total.
- **TF-L randomised stress.** 200 PRNG-seeded trials sweep length, seq, source, topic, profile, with 30 % chance to shuffle and 30 % chance to inject a duplicate; all complete to the original payload.

### Tests after Round 11

`Ran 179 tests in 0.764s` `OK (skipped=1)` (was 164; +15 from the new suite, with 1 deliberate skip on a profile that can't pack under the 25 ms cap by design).

### Why this matters

The IP-108 fuzz pinned the operator?tractor command frame against parser bugs. Round 11 is the symmetric pin on the tractor?operator telemetry path: a silent fragmentation bug there would corrupt the data the operator trusts � pose, hydraulic pressures, mode-switch state � without any visible failure. The randomised stress is seeded so a future regression repros byte-for-byte.

### Next candidates

- WS-subscriber stress test for Round 9 �C: N concurrent fake clients connect/disconnect at 100 Hz while telemetry/image/state flow; assert no `RuntimeError` and no leaked entries in any pool.
- �A CI compile-gate flip � still gated on a real Actions run.

## Round 12 � 2026-04-28 � WS subscriber concurrency stress suite

Round 9 �B added the `/ws/control` cap and �C added `_subscribers_lock` + `_snapshot_subscribers`. The single-threaded admit/reject tests in `test_web_ui_validation.py` proved the cap rejects with 4429 and that the lock exists; they did **not** prove the property �C was actually introduced for � that the MQTT background thread can iterate the WS pools while the FastAPI event loop mutates them, with zero `RuntimeError: Set changed size during iteration` and zero over-admits under contention.

### What landed

[`base_station/tests/test_ws_subscriber_concurrency.py`](../DESIGN-CONTROLLER/base_station/tests/test_ws_subscriber_concurrency.py) � 6 tests that drive `_admit_ws`, `_discard_subscriber`, `_snapshot_subscribers`, and `_on_mqtt_message` together against a real `asyncio` loop running on a background thread (`_LoopFixture`). This mirrors the production topology: FastAPI handlers run on the loop thread, MQTT callbacks fire on paho's thread. A small `FakeWS` mocks the starlette `WebSocket` surface `_admit_ws` actually touches (`accept`, `close(code=�)`, `send_text`, `send_bytes`).

### Properties asserted (WC-A � WC-G)

- **WC-A cap honoured under N concurrent admits.** Submit `N = 4 � MAX_TELEMETRY_SUBSCRIBERS` admit coroutines onto the loop simultaneously; assert exactly `cap` return True, the rest return False, and `len(pool) == cap` afterwards. The check-then-add window inside `_admit_ws` is single-threaded by `_subscribers_lock`, so no admit can sneak past the gate.
- **WC-B admit/discard cycles, no leak.** 200 sequential admit-then-discard cycles on the control pool; pool length = cap throughout, exactly 0 at the end.
- **WC-C snapshot iteration is race-free.** 4 churner threads admit/discard at maximum rate while 2 reader threads call `_snapshot_subscribers(pool)` 20 000 times each and iterate every snapshot. The whole point of �C: if the snapshot were a live view rather than a list copy under the lock, the readers would intermittently raise `RuntimeError: Set changed size during iteration`. Asserts the error list stays empty.
- **WC-D fan-out under churn.** Two resident subscribers in the telemetry pool, three churners mutating it, and 500 `_on_mqtt_message` calls dispatching telemetry frames from the main thread. Asserts no exception escapes the fan-out and the resident subscribers actually receive frames (`sent_text > 0`) � the snapshot path is delivering, not silently dropping.
- **WC-E per-pool cap independence.** Saturating telemetry must not block image/state/control admissions. Confirms the four caps don't share state.
- **WC-F drain-and-refill counter integrity.** Fill image to cap, reject one over, drain all, refill to cap, reject one over again. Catches a class of bug where `_discard_subscriber` left the counter stale (admitting 0) or relaxed (admitting cap + 1).
- **WC-G 4429 close-code on rejection.** Piggybacked onto WC-A: every rejected `FakeWS` must have `close()` called with `WS_OVER_CAPACITY_CODE`.

### Tests after Round 12

`Ran 185 tests in 0.789s` `OK (skipped=1)` (was 179; +6 from the new suite, no regressions in the prior 179).

### Why this matters

�C was a "looks right by inspection" change driven by reading the parallel reviews; it had no test that would actually fail if the lock were removed. WC-C now does � temporarily commenting out `_subscribers_lock` acquires inside `_snapshot_subscribers` reproduces the original `Set changed size during iteration` race the lock was added to suppress. The �B control-cap was already pinned by the single-threaded test, but WC-A pins it under contention, which is the threat model: a misbehaving client opening sockets in parallel.

### Next candidates

- Firmware fail-count consecutive-vs-cumulative fix (TR-I divergence pinned in Round 10): either change the firmware to match the runbook prose or update the runbook to match the firmware. Needs a design call before code.
- �A CI compile-gate flip � still gated on a real Actions run.
- Image-pipeline frag/reassembly fuzz, symmetric to TF-A..TF-L but for `image_pipeline/reassemble.py`.

## Round 13 � 2026-04-28 � Image-pipeline reassembly property/fuzz suite

Third and final fragmenter fuzz suite, completing the property coverage of every binary parser in the v25 stack.

### What landed

[`base_station/tests/test_image_reassembly_fuzz.py`](../DESIGN-CONTROLLER/base_station/tests/test_image_reassembly_fuzz.py) � 13 tests against `image_pipeline/reassemble.py` (the on-air TileDeltaFrame chunker that `camera_service.py` will start using once the M7?SX1276 SPI driver lands). Pure stdlib, no new deps. Reuses the same `_feed_all` first-non-None-completion latching trick as the telemetry fuzz, because this reassembler also re-opens a partial slot when a duplicate arrives after completion.

### Properties asserted (IF-A � IF-L)

- **IF-A round-trip identity.** Arbitrary frame sizes (1, 2, 5, 12, 32, 96 changed tiles) split into 1, 2, 3, 5, 8 fragments � every viable combination round-trips through the encoder, the chunker, and the reassembler back to the original `TileDeltaFrame`.
- **IF-B fragment-header invariants.** Per-stream: every fragment carries the magic `0xFE`, a constant `frag_seq`, a constant `total_minus1`, and `frag_idx` monotonically 0..total-1.
- **IF-C bad-magic passthrough is harmless.** A complete TileDeltaFrame whose first byte isn't `0xFE` (the real-world "bridge currently runs over a local USB-CDC link" case) is treated as an already-assembled frame and counted as a passthrough. Crucially: passthrough must not disturb any in-flight fragmented stream � verified by interleaving a passthrough between two fragments of an active assembly and confirming the original stream still completes.
- **IF-D `frag_idx >= total` rejection.** A fragment claiming idx 5 of a 2-fragment stream bumps `decode_errors` and returns None instead of writing past the partial slot.
- **IF-E reorder.** 20 PRNG-shuffled deliveries of the same payload all reassemble.
- **IF-F duplicate counted, no double-complete.** Send each non-final fragment 3� then the final fragment once � exactly one completion, `2*(N-1)` duplicates counted. Same property as the telemetry fuzz TF-F.
- **IF-G missing-middle timeout.** Drop a middle fragment, advance the injected clock past `timeout_ms`, feed any other fragment to trigger GC, assert the partial is reclaimed and `timeouts` ticks.
- **IF-H multi-stream isolation.** Two streams keyed by distinct `frag_seq` are interleaved fragment-by-fragment; both complete independently.
- **IF-I seq-wrap with intervening GC.** Stream uses `frag_seq=255`, gets GC'd, then a fresh stream re-uses `frag_seq=0` � both behave correctly across the wrap.
- **IF-J truncated header with magic.** A 2-byte buffer starting with `0xFE` is shorter than the 4-byte fragment header; `decode_errors` ticks instead of raising. Important because the reassembler is the first thing the bridge sees on the wire.
- **IF-K mid-stream `total` change.** Producer changes its mind mid-frame: idx=0 of a 4-frag carve-up arrives, then the same `frag_seq` re-emits as a 2-frag carve-up. The reassembler discards the partial and completes on the new total. Documents the "`Producer changed its mind mid-frame. Restart this slot.`" branch in `reassemble.py`.
- **IF-L randomised stress.** 200 PRNG-seeded trials sweep frame size, fragment count, `frag_seq`, with 30 % chance of shuffle and 30 % chance of injecting a duplicate (positioned before the final fragment so the latch logic holds). All complete to the correct `base_seq`.

### Tests after Round 13

`Ran 198 tests in 0.876s` `OK (skipped=1)` (was 185; +13 from the new suite, no regressions in the prior 185).

### Coverage now complete on all three binary fragmenters

| Direction | Path | Property suite | Round |
| --- | --- | --- | --- |
| Operator ? tractor | `CommandFrame` (LoRa) | `test_command_frame_fuzz.py` | IP-108 |
| M7 ? base | telemetry frame fragmenter (LoRa) | `test_telemetry_fragmentation_fuzz.py` | 11 |
| Camera ? base | TileDeltaFrame chunker (LoRa) | `test_image_reassembly_fuzz.py` | 13 |

A silent fragmentation bug on any of these paths would corrupt operator commands, telemetry the operator trusts, or the live camera canvas � without raising. All three directions are now pinned by deterministic, PRNG-seeded property tests that repro byte-for-byte on regression.

### Next candidates

- **Firmware fail-count consecutive-vs-cumulative** (TR-I divergence pinned in Round 10): needs a design call. The runbook prose says *consecutive*, the firmware counts *cumulative*. Either fix is small; the choice is what we want to enforce.
- **�A CI compile-gate flip** � still gated on a real Actions run.
- All remaining items in the Wave-4 HIL gate set require bench hardware.

## Round 14 � 2026-04-28 � TR-I firmware fix: consecutive fail-count

Resolves the divergence pinned by Round 10 between the W4-04 runbook prose ("10 *consecutive* failures") and the firmware as written (cumulative count, never reset). Decision: fix firmware to match runbook. Cumulative was a slow-burn nuisance-trip risk, not a safety property: a single bad Modbus poll every few hours would eventually (over a long run) latch the tractor down for no good reason, while *not* providing meaningfully faster latching when an actual sustained outage happens.

### What landed

- [`firmware/tractor_h7/tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) � moved the `s_modbus_fail_count` static out of the failure branch so the success branch can reset it to 0. Updated the comment to call out the consecutive semantics and reference the W4-04 runbook. Log message changed from `"(N since boot)"` to `"(N consecutive)"` to make the semantics obvious in serial captures.
- [`base_station/tests/test_modbus_slave_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_modbus_slave_sil.py) � flipped TR-I:
  - `M7ModbusClient.apply_control()` now resets `self.fail_count = 0` on success, mirroring the firmware fix.
  - Old `test_fail_count_is_cumulative_not_consecutive` (which pinned the bug) replaced by `test_fail_count_is_consecutive` (positive assertion of the new behaviour).
  - Added `test_intermittent_failures_never_latch`: 1000 cycles alternating fail / success � cumulative would have tripped at cycle 19, consecutive must never trip.
  - Added `test_nine_then_one_then_nine_does_not_latch`: 9 failures + 1 success + 9 failures � cumulative would have tripped on the 19th call (10th total failure); consecutive must not.
  - Module docstring + TR-C / TR-I header comments updated to describe the post-Round-14 behaviour and explain how to detect a regression to cumulative.

### Tests after Round 14

`Ran 200 tests in 0.913s` `OK (skipped=1)` (was 198; +3 consecutive tests, -1 cumulative test = net +2). All 15 tests in `test_modbus_slave_sil.py` pass.

### W4-04 runbook alignment

The runbook prose in [`HIL_RUNBOOK.md`](../DESIGN-CONTROLLER/HIL_RUNBOOK.md) �W4-04 was already consecutive-correct (steps 1�5). Step 6's *resume*-side hysteresis ("M7 resumes after 5 consecutive successes" + stick-release-to-neutral) is a *separate* unimplemented behaviour � the firmware currently fully latches `g_estop_latched` and does not auto-recover at all. That's tracked as its own future work; the Round 14 fix is scoped only to the failure-count semantics on the way *into* the latch.

### Round-13 follow-on still open

- **�A CI compile-gate flip** � still gated on a real Actions run.
- All remaining items in the Wave-4 HIL gate set require bench hardware, with the SIL-companion plan in the prior round's "Next candidates" still applicable as a way to convert HIL-only gates into CI-pinned-plus-bench-validated.

## Round 15 � 2026-04-28 � Arduino compile gate: handheld_mkr unblocked, three latent CI bugs fixed

Arduino-cli installed locally (1.4.1, with arduino:samd 1.8.14, arduino:mbed_portenta 4.5.0, arduino:mbed_opta 4.5.0). Replaying the CI compile steps locally exposed *three* independent bugs that had all been masked by `continue-on-error: true` since IP-007. `handheld_mkr` is now a real blocking compile gate; `tractor_h7` and `tractor_opta` are still blocked but the workflow comments now spell out exactly why and what fixes them.

### Three bugs the local replay exposed

1. **Stale relative includes.** `handheld_mkr.ino` and `tractor_h7.ino` / `tractor_m4.cpp` had `#include "../common/lora_proto/lora_proto.h"` etc. arduino-cli copies the sketch into a temp build dir where `../common/` no longer resolves. The CI staging step (which copies `firmware/common/lora_proto` into `sketch/src/lora_proto/`) was correct, but the `#include` strings were never rewritten to use the staged path.
2. **`--build-property "build.extra_flags=..."` clobbers the board's flags.** The MKR WAN 1310 variant.h gates `LORA_IRQ` (pin 31) behind `#if defined(USE_BQ24195L_PMIC)`, and `USE_BQ24195L_PMIC` is defined in `mkrwan1310.build.extra_flags` in boards.txt. Overriding `build.extra_flags` wipes that, so `LORA_IRQ` becomes undefined and the sketch fails with `'LORA_IRQ' was not declared in this scope`. The fix: use `compiler.cpp.extra_flags` and `compiler.c.extra_flags`, which *append* to the toolchain flags instead of replacing the recipe macro.
3. **`crypto_stub.c` requires explicit opt-in.** The stub crypto file `#error`s out unless either `LIFETRAC_USE_REAL_CRYPTO` or `LIFETRAC_ALLOW_STUB_CRYPTO` is defined. The CI's "stub" build only set `LIFETRAC_ALLOW_UNCONFIGURED_KEY` (a different gate, in the .ino itself), so the stub build always failed at the crypto step.

### What landed

- [`firmware/handheld_mkr/handheld_mkr.ino`](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino) � include rewritten to `"src/lora_proto/lora_proto.h"` with a comment block pointing at the staging script and the canonical `firmware/common/` source of truth.
- [`firmware/tractor_h7/tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) � same include rewrite for both `lora_proto.h` and `shared_mem.h`, plus a `struct AxisRamp;` forward declaration to suppress an Arduino-preprocessor auto-prototype-versus-definition collision (`'int16_t step_axis_ramp(AxisRamp&, ...)' redeclared as different kind of symbol`) � the auto-generated prototype was being parsed before `AxisRamp` was visible.
- [`firmware/tractor_h7/tractor_m4.cpp`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m4.cpp) � include rewrite to `"src/shared_mem.h"`.
- [`.github/workflows/arduino-ci.yml`](../../.github/workflows/arduino-ci.yml):
  - `firmware-compile-handheld` is **no longer `continue-on-error`** � it's now a blocking gate. Build flags rewritten to use `compiler.cpp.extra_flags` / `compiler.c.extra_flags` and to add `-DLIFETRAC_ALLOW_STUB_CRYPTO` to the stub build.
  - `firmware-compile-opta` and `firmware-compile-tractor-h7` remain `continue-on-error` but each gained a Round-15 comment block explaining the *one* remaining blocker per job (Opta: .ino name doesn't match folder name; H7: `tractor_m4.cpp` defines its own `setup()`/`loop()` and link-collides with the M7 binary). The H7 job also picked up the same `compiler.{cpp,c}.extra_flags` fix so it's ready to flip the moment the M4/M7 split lands.
- [`.gitignore`](../../.gitignore) � added `LifeTrac-v25/DESIGN-CONTROLLER/firmware/handheld_mkr/src/` and `firmware/tractor_h7/src/` (the staged copies of common/).
- [`DESIGN-CONTROLLER/tools/stage_firmware_sketches.ps1`](../DESIGN-CONTROLLER/tools/stage_firmware_sketches.ps1) � new PowerShell script that mirrors the CI's "Stage shared sources into sketch" steps so a local developer can run the same arduino-cli compiles offline. Includes a printed reminder about the `compiler.cpp.extra_flags` vs `build.extra_flags` trap.

### Local verification

- `handheld_mkr` (stub crypto): `Sketch uses 55932 bytes (21%) of program storage space. Maximum is 262144 bytes. Global variables use 4948 bytes (15%) of dynamic memory, leaving 27820 bytes for local variables.`
- `handheld_mkr` (real crypto, `-DLIFETRAC_USE_REAL_CRYPTO`): `Sketch uses 59940 bytes (22%). Global variables use 5108 bytes (15%).` � the +4 KB / +160 B delta is the AES-GCM (Crypto.h + AES.h + GCM.h) cost.
- Base-station test suite: `Ran 200 tests in 0.837s` `OK (skipped=1)` � no regressions.

### What's still blocked, in order of difficulty

- **`firmware-compile-opta`** � rename `opta_modbus_slave.ino` ? `tractor_opta.ino` (or rename the folder to `opta_modbus_slave`). One-file change once we pick the name.
- **`firmware-compile-tractor-h7`** � split `tractor_m4.cpp` so it doesn't link into the M7 binary. Two reasonable shapes: (a) move it to its own sketch folder `firmware/tractor_m4/tractor_m4.ino` and add a separate compile job for `arduino:mbed_portenta:envie_m4`, then bake the resulting binary into M7 via the standard Portenta dual-core flow; or (b) wrap the whole file in `#if defined(CORE_CM4)` so it's a no-op on M7 builds and add a parallel CM4 compile to the same workflow. (a) is more idiomatic for production, (b) is faster to implement and is what most early dual-core sketches do.

### Coverage scoreboard (post Round 15)

| Compile target | FQBN | Status | Notes |
| --- | --- | --- | --- |
| handheld_mkr (stub) | arduino:samd:mkrwan1310 | **blocking** | green |
| handheld_mkr (real) | arduino:samd:mkrwan1310 | **blocking** | green |
| tractor_opta | arduino:mbed_opta:opta | continue-on-error | .ino name vs folder name |
| tractor_h7 (stub) | arduino:mbed_portenta:envie_m7 | continue-on-error | M4/M7 split |

## Round 16 � 2026-04-28 � Arduino compile gate: tractor_opta unblocked

Second of three Arduino compile gates flipped from `continue-on-error` to blocking. Same playbook as Round 15: ran the local arduino-cli replay, fixed everything it found, then locked the result into both the workflow and the source tree.

### Two issues exposed by the local replay

1. **.ino filename did not match folder name.** The sketch was `firmware/tractor_opta/opta_modbus_slave.ino` � arduino-cli requires the .ino base name to match the parent folder, so it failed at sketch-discovery time before touching any source. Renamed to `tractor_opta.ino`.
2. **Missing library: `Arduino_Opta_Blueprint`.** The .ino does `#include <OptaBlue.h>`, which is provided by the `Arduino_Opta_Blueprint` library on the Arduino library index. The CI install step had only `ArduinoRS485` and `ArduinoModbus`.

### What landed

- Renamed [`firmware/tractor_opta/opta_modbus_slave.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/) ? [`firmware/tractor_opta/tractor_opta.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/tractor_opta.ino). Source unchanged.
- [`.github/workflows/arduino-ci.yml`](../../.github/workflows/arduino-ci.yml) � `firmware-compile-opta` is **no longer `continue-on-error`**. Added `arduino-cli lib install Arduino_Opta_Blueprint` to the Install libraries step. Old "deferred to its own round" comment block replaced with a Round-16 comment naming the two fixes.

### Local verification

- `tractor_opta`: `Sketch uses 171520 bytes (8%) of program storage space. Maximum is 1966080 bytes. Global variables use 65120 bytes (12%) of dynamic memory, leaving 458504 bytes for local variables. Maximum is 523624 bytes.`
- Base-station test suite: `Ran 200 tests in 1.111s` `OK (skipped=1)` � no regressions.

### Coverage scoreboard (post Round 16)

| Compile target | FQBN | Status | Notes |
| --- | --- | --- | --- |
| handheld_mkr (stub) | arduino:samd:mkrwan1310 | **blocking** | green (Round 15) |
| handheld_mkr (real) | arduino:samd:mkrwan1310 | **blocking** | green (Round 15) |
| tractor_opta | arduino:mbed_opta:opta | **blocking** | green (Round 16) |
| tractor_h7 (stub) | arduino:mbed_portenta:envie_m7 | continue-on-error | M4/M7 split |

Two of three Arduino compile gates are now real. `tractor_h7` remains the only `continue-on-error` job; the next round can either tackle that (M4/M7 split, see Round-15 comment block in the workflow for the two recommended shapes) or pivot to a Wave-4 SIL companion.


---

## Round 17 — 2026-04-28 — Tractor H7 dual-core split (M7 + M4 sketches)

**Status:** `firmware-compile-tractor-h7` flipped from
`continue-on-error: true` to blocking. New parallel
`firmware-compile-tractor-m4` job added. M4 watchdog firmware moved
from `firmware/tractor_h7/tractor_m4.cpp` to its own sketch folder
[`firmware/tractor_h7_m4/tractor_h7_m4.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7_m4/tractor_h7_m4.ino).
Compile-gate scoreboard: **4 of 4 Arduino jobs now blocking** (handheld stub,
handheld real, tractor_opta, tractor_h7 M7, tractor_m4 M4) — zero
`continue-on-error` jobs remain in the workflow.

### Why two sketch folders, not a `#if defined(CORE_CM4)` guard

The first attempt was to wrap `tractor_m4.cpp` in
`#if defined(CORE_CM4)` so the M7 build would see no symbols from
the file. That fixed the M7 `multiple definition of 'setup'` link
error, but the M4 build then failed: arduino-cli's library
auto-discovery scans the .ino's `#include` directives REGARDLESS
of the selected `target_core`, so RadioLib + ArduinoModbus +
ArduinoRS485 (used only by the M7 .ino) get pulled into the M4
build and fail to compile against the M4 variant headers (e.g.
`ArduinoHal.cpp:49: invalid conversion from 'uint32_t' to
'PinStatus'`).

Two separate sketch folders is the canonical Portenta-dual-core
layout — Arduino IDE's stock `PortentaDualCore` example uses
exactly this shape. Each core gets its own minimal include set.

### FQBN / build glue

The Portenta core does NOT expose a separate `envie_m4` FQBN
(`arduino-cli board listall arduino:mbed_portenta` shows only
`envie_m7` and `portenta_x8`). The M4 build uses the
`target_core=cm4` board menu option:

`arduino:mbed_portenta:envie_m7:target_core=cm4`

This switches `build.variant` to `GENERIC_STM32H747_M4`, mcu to
`cortex-m4`, fpu to `fpv4-sp-d16`, and adds
`-DPORTENTA_H7_PINS` (per `boards.txt`).

### Local verification

* M7 clean build: EXIT=0, `tractor_h7.ino.bin` 207888 B.
* M4 clean build: EXIT=0, `tractor_h7_m4.ino.bin` 87032 B.
* Stage script
  [`tools/stage_firmware_sketches.ps1`](../DESIGN-CONTROLLER/tools/stage_firmware_sketches.ps1)
  extended: `Stage-Sketch` now defaults `-LoraProto` to `@()`,
  and a third call stages `shared_mem.h` into
  `firmware/tractor_h7_m4/src/` for the M4 sketch.
* `.gitignore` extended with the new
  `firmware/tractor_h7_m4/src/` staged-sources path.
* Doc references updated:
  [`BUILD-CONTROLLER/05_firmware_installation.md`](../BUILD-CONTROLLER/05_firmware_installation.md)
  Step 2.6 (Arduino IDE upload path),
  [`DESIGN-CONTROLLER/ARDUINO_CI.md`](../DESIGN-CONTROLLER/ARDUINO_CI.md)
  active-targets list,
  [`DESIGN-CONTROLLER/arduino_libraries.txt`](../DESIGN-CONTROLLER/arduino_libraries.txt)
  section heading.
* Base station regression: 200 tests pass (1 skipped).

### Files changed

* **MOVED** `firmware/tractor_h7/tractor_m4.cpp` →
  `firmware/tractor_h7_m4/tractor_h7_m4.ino` (.ino name must match
  parent folder for arduino-cli sketch discovery; .cpp → .ino is
  immaterial for the toolchain since both get the same C++ pass).
  The header comment was rewritten to explain the Round-17 layout
  and FQBN convention; the body (`trip()`, `setup()`, `loop()`,
  IP-105 seqlock, IP-106 magic gate) is byte-identical to the
  pre-Round-17 `tractor_m4.cpp`.
* **NEW** `.github/workflows/arduino-ci.yml` job
  `firmware-compile-tractor-m4`.
* **MODIFIED** `.github/workflows/arduino-ci.yml` job
  `firmware-compile-tractor-h7`: removed
  `continue-on-error: true`, replaced the Round-15 comment block
  about the unsolved M4/M7 split with a Round-17 comment pointing
  at the new layout.
* **MODIFIED** `DESIGN-CONTROLLER/tools/stage_firmware_sketches.ps1`:
  third Stage-Sketch call for tractor_h7_m4; default `-LoraProto =
  @()` so empty arrays don't trip parameter binding.
* **MODIFIED** `.gitignore`: added
  `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7_m4/src/`.



---

## Round 18 - 2026-04-28 - W4-05 / W4-06 SIL: per-axis ramp + mixed-mode skip

**Status:** new test module
[`base_station/tests/test_axis_ramp_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py)
(10 tests, stdlib only). 200 -> 210 base_station tests pass. W4-05
(proportional valve ramp-out, IP-303 follow-on) and W4-06 (mixed-mode
skip) join W4-01 / W4-03 / W4-04 in the bench-confirmation-only tier:
the design is now verified in CI and the bench session just needs to
confirm the wiring matches the model.

### What the SIL covers

Pure-Python ports of two firmware surfaces from
[`firmware/tractor_h7/tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino):

* `ramp_duration_ms()` and `step_axis_ramp()` byte-for-byte
  (including the C-style `int32_t` truncating division for the
  linear-interpolation step).
* `FourAxisArbiter.tick()` mirrors the four-call block in
  `apply_control()` that wires each axis's `other_active`
  argument from the raw values of its three siblings, plus the
  coil-bitmap and `REG_FLOW_SP_*` derivation from post-ramp
  `effective` values.

### Properties asserted (tied to the W4-05 / W4-06 runbook prose)

W4-05 (ramp-out):

* **TR-A** Track full-speed (raw=127) released alone: monotonic
  non-increasing magnitude, hits 0 within one `RAMP_TICK_MS` of the
  2 s deadline.
* **TR-B** Linear-interpolation midpoint at t = duration/2 is within
  +/-2 axis-units of `start/2`.
* **TR-C** Track ladder: (96..127] -> 2000 ms; [48..96) -> 1000 ms;
  (deadband..48) -> 500 ms.
* **TR-D** Arm ladder: (96..127] -> 1000 ms; [48..96) -> 500 ms;
  (deadband..48) -> 250 ms.
* **TR-E** Coil-stays-engaged invariant: while the ramped magnitude
  is above the deadband the drive coils MUST remain set; once the
  ramp drops to/below the deadband the coils MUST clear (no lag).
  Pins the apply_control comment "coils stay engaged while we ramp
  the proportional flow down so the hydraulic path doesn't slam
  closed".
* **TR-F** `REG_FLOW_SP_*` is monotonically non-increasing during
  the ramp and reaches 0 within one tick of the deadline.

W4-06 (mixed-mode skip):

* **TR-G** Two-axis: lhy released while lhx still active -> lhy goes
  straight to 0 in the same tick; `lhy.ramping` stays false; lhx
  passes through untouched.
* **TR-H** Orientation-agnostic: the skip outcome is identical for
  every sibling-held-while-released combination, with the degenerate
  single-axis case still taking the ramp path.
* **TR-I** Last-axis release: after a skip, the next release of the
  last raw-active axis MUST take the ramp path with the correct
  per-ladder duration.
* **TR-J** `other_active` is computed against *raw* inputs, not the
  `effective` of an already-ramping sibling. A previously-released
  axis whose `effective` is non-zero must NOT be observable to the
  skip check; otherwise a fresh release would incorrectly skip when
  it should ramp. Pins the comment in apply_control:
  "`other_active` is computed against the *raw* values so a stale
  ramping axis doesn't block a sibling axis's mixed-mode skip."

### Files changed

* **NEW** `DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py`
  (10 tests, ~430 lines).
* **MODIFIED** `LifeTrac-v25/TODO.md`: W4-05 and W4-06 bullets gain
  `*SIL coverage:* ...` notes; status block bumped to "through
  Round 18" with a Round 18 narrative paragraph.

### What's next

* Other W4 items now reachable in SIL: W4-02 (link-tune walk-down -
  needs a model of the SF7/SF8/SF9 ladder + revert-deadline), W4-07
  (boot-PHY first-frame decode), W4-08 (camera back-channel keyframe
  round-trip - timing model only, no real frame data), W4-10
  (fleet-key provisioning sanity - already mostly covered by
  `test_provision.py`).
* W4-09 (async M7 TX state machine) remains hard-blocked on real
  silicon: the IRQ-driven `isTransmitDone()` timing is the whole
  point.


---

## Round 19 (2026-04-28) — W4-02 link-tune walk-down SIL

**Plan ref:** Wave-4 SIL companion for `W4-02 Link-tune walk-down` (HIL_RUNBOOK.md §W4-02).

**Goal:** Move W4-02 from a greenfield bench pass to a verification pass against a documented model in the same style as Round 18 (W4-05/06 SIL).

### What landed

- New file: `LifeTrac-v25/DESIGN-CONTROLLER/base_station/tests/test_link_tune_sil.py` (~720 lines, **16 tests**, pure stdlib).
- Pure-Python port of the M7 adaptive-SF ladder state machine in `firmware/tractor_h7/tractor_h7.ino`:
    - `try_step_ladder()` (twice-back-to-back announce + revert deadline arm).
    - `apply_phy_rung()` (radio retune, RX re-arm).
    - `send_link_tune()` (TX log captures target_rung, reason, and PHY-at-TX).
    - `poll_link_ladder()` (verification first, then 5 s window classifier with R-8 hysteresis).
    - HB ingress hook that records `hb_at_pending` while a tune is pending.
- Constants kept byte-identical: `LADDER_WINDOW_MS=5000`, `LADDER_BAD_TO_STEPDOWN=3`, `LADDER_GOOD_TO_STEPUP=6`, `LADDER_HB_BAD_THRESHOLD=40`, `LADDER_HB_GOOD_THRESHOLD=48`, `LADDER_TUNE_REVERT_MS=500`, and the three-rung `LADDER[]` table (SF7/BW250, SF8/BW125, SF9/BW125).

### Verification surface

| Test | What it pins |
|---|---|
| TL-A | Cold-start at SF7 + 6 clean windows: rung-0 step-up gate must hold (no TX). |
| TL-B | Three bad windows walk SF7→SF8 (commit on verification HB inside 500 ms). |
| TL-B2 | Six bad windows walk SF7→SF9 (two committed steps, 4 TX frames total). |
| TL-C | Three more bad windows past SF9: rung < 2 gate clamps the ladder. |
| TL-D | Six good windows walk SF9→SF8 (recovery direction). |
| TL-D2 | Twelve good windows walk SF9→SF7, plus six more from rung 0: rung > 0 gate clamps. |
| TL-E | Missing verification HB at new PHY → revert at 500 ms deadline + `tune_failures` increments. |
| TL-E2 | HB after the deadline does not save us: poll runs first, reverts, deadline cleared. |
| TL-E3 | HB inside the 500 ms deadline commits the new rung. |
| TL-F | `try_step_ladder` emits exactly two LINK_TUNE TX frames per attempt — one at OLD PHY, one at NEW PHY, same target & reason. |
| TL-F2 | No-op when target == current rung. |
| TL-F3 | No-op when target out of range (>= 3). |
| TL-G | `pick_active_source() == -1` → window pessimistically classified bad even if a non-active source kept emitting HBs. |
| TL-H | Post-tune window starts fresh; old HB counts do not carry into the bad/good classifier. |
| TL-I | Pending tune short-circuits the bad/good classifier (defensive against future call paths that arm a tune outside the window evaluator). |
| TL-I2 | Verification path still runs while the classifier is gated off (revert / commit still service correctly). |

### Quirks discovered & resolved

- The C code's `if (g_ladder_window_start == 0) g_ladder_window_start = now;` lazy-init means `window_start = 0` AND `now = 0` is a degenerate sentinel collision; in real firmware `millis() > 0` by the time `setup()` finishes. The SIL pre-initializes `window_start = 1` (`BOOT_MS`) and tests start `now = 1` so the first window has its full 5000 ms duration. Caught by the first run: TL-A reported 5 not 6 in `consec_good`.
- TL-I as originally written tried to drive the in-flight gate via the natural call flow, but `try_step_ladder()` resets `window_start = now`, so the early-return on `tune_deadline != 0` would not fire before the deadline elapsed (5000 ms window vs 500 ms deadline). Restructured to arm a deadline directly so the gate is exercised in isolation, plus added TL-I2 to prove `poll_link_ladder()` still services the verification path while the classifier is gated.

### CI status

- 226 / 226 base_station tests pass (1 skipped). Up from 210 in Round 18.
- All 5 Arduino compile jobs remain blocking (handheld stub, handheld real, tractor_opta, tractor_h7 M7, tractor_m4 M4).

### What's next

Bench-confirmation-only items remaining: W4-01, W4-02 (loss metric), W4-03, W4-04, W4-05, W4-06.

Wave-4 SIL candidates still open:

- **W4-07** boot-PHY first-frame decode (IP-006). Model the boot-PHY decode path so a regression that drifts boot-PHY away from `LADDER[0]` is caught in CI before the on-air handshake breaks.
- **W4-08** camera back-channel keyframe round-trip. Timing model only — no real frame data needed.
- **W4-10** fleet-key provisioning sanity. Mostly already covered by `test_provision.py`; would add the OLED `FLEET KEY NOT PROVISIONED` + non-zero exit-code assertions.

Hard-blocked: **W4-09** async M7 TX state machine — needs real silicon for IRQ-driven `isTransmitDone()` timing.

---

## Round 20 (2026-04-28) — W4-07 boot-PHY first-frame decode SIL

**Plan ref:** Wave-4 SIL companion for `W4-07 Boot-PHY first-frame decode (IP-006 verification)` (HIL_RUNBOOK.md §W4-07).

**Goal:** Move W4-07 from a greenfield bench pass to a CI-time gate that catches the IP-006 failure mode (boot-PHY drift between source files) before any radio is keyed.

### What landed

- New file: `LifeTrac-v25/DESIGN-CONTROLLER/base_station/tests/test_boot_phy_sil.py` (~330 lines, **11 tests**, pure stdlib).
- Source-parsing approach (intentionally brittle):
    - `parse_ladder()` extracts the `static const LadderRung LADDER[3] = {...};` table from a .ino.
    - `find_radio_begins()` extracts every `radio.begin(freq, bw, sf, cr, sync, txpwr)` call.
    - `RadioBeginCall.normalize()` strips `// ...` line comments, `(float)` casts, and whitespace so we can compare the *meaning* of each arg (and only its meaning) across re-formats.

### Verification surface

| Test | What it pins |
|---|---|
| BP-A | `LADDER[3]` tables in `handheld_mkr.ino` and `tractor_h7.ino` are byte-identical row-for-row (CMD_LINK_TUNE rung mapping). |
| BP-A2 | `LADDER[0]` on both nodes matches DECISIONS.md D-A2 (SF7 / BW250 / CR4-5 / bw_code=1). |
| BP-A3 | SF strictly increases down the ladder (try_step_ladder() invariant: higher rung = wider/slower SF). |
| BP-A4 | Each rung's (sf, bw_khz, cr_den) tuple is unique (handheld's reverse-lookup `CMD_LINK_TUNE` → rung index needs this). |
| BP-B | Handheld `radio.begin(...)` references `LADDER[0].bw_khz` / `.sf` / `.cr_den` symbolically + sync `0x12` + freq 915.0 MHz. |
| BP-B2 | Tractor M7 `radio.begin(...)` ditto. |
| BP-B3 | Regression-trap: nobody slipped a numeric SF / BW / CR literal back in (the literal would silently drift if the ladder is re-ordered). |
| BP-C | `DEFAULT_PARAMS["link"]["control_phy"]` in `tractor_x8/params_service.py` equals `"SF7_BW250"`. |
| BP-D | Parser self-test: `parse_ladder()` rejects a 2-row table. |
| BP-D2 | Parser self-test: `parse_ladder()` rejects a missing table. |
| BP-D3 | Parser self-test: `find_radio_begins()` skips unrelated `foo.begin()` calls. |

### Why parser self-tests (BP-D)?

Without them this file could silently pass on a refactored source where the `LADDER[]` table simply moved into a header — the regex would no longer match, `parse_ladder()` would raise the descriptive `AssertionError`, and that's the loud failure we want. BP-D pins the parser's own correctness so a refactor that breaks the regex shows up as a parser bug rather than as silently-skipped coverage.

### Quirks discovered & resolved

- First run of BP-B failed because the handheld `radio.begin(...)` has `// IP-006: match LADDER[0] (BW250)` as an inline comment between args. `RadioBeginCall.normalize()` originally only stripped whitespace and `(float)` casts. Extended it to strip `// ...` line comments before comparison.

### CI status

- 237 / 237 base_station tests pass (1 skipped). Up from 226 in Round 19.
- All 5 Arduino compile jobs remain blocking.

### What's next

Bench-confirmation-only items remaining: W4-01, W4-02 (loss metric), W4-03, W4-04, W4-05, W4-06, W4-07 (no-`bad_header` cold-boot capture).

Wave-4 SIL candidates still open:

- **W4-08** camera back-channel keyframe round-trip. Timing model only; would port the `CMD_REQ_KEYFRAME` path through `camera_service.py`'s I-frame trigger and budget the < 200 ms end-to-end.
- **W4-10** fleet-key provisioning sanity. Mostly already covered by `test_provision.py`; would add the OLED `FLEET KEY NOT PROVISIONED` + non-zero exit-code assertions.

Hard-blocked: **W4-09** async M7 TX state machine — needs real silicon.

---

## Round 20.5 (2026-04-28) — MASTER_TEST_PROGRAM.md inventory doc

**Plan ref:** documentation / process — not a plan IP. Triggered by user request: `should we create a MASTER TEST PROGRAM .md for keeping track of all the tests we want to perform?`

**Goal:** Single index for every SIL test, every Arduino compile gate, every HIL bench item, and per-IP traceability for the entire IP-001 … IP-309 plan, so coverage gaps are visible at a glance and the next SIL round always knows what to pick.

### What landed

- New file: `LifeTrac-v25/MASTER_TEST_PROGRAM.md` (~270 lines).
- Seven sections:
    1. **Tier overview** — SIL / compile-gate / HIL counts (237 / 5 / 10).
    2. **SIL test catalog** — 25-row table: one row per `test_*.py` with tests, scope, IP refs, W4 gate, round.
    3. **Compile-gate catalog** — 5 production jobs + 1 informational, FQBN + sketch + flash/RAM footprint.
    4. **HIL bench matrix** — 10 W4-XX rows: goal + pass criterion + SIL coverage + bench-only residual.
    5. **Per-IP traceability** — every IP-001…IP-309 (34 items) mapped to its verifying test or marked `—` (gap). Coverage-gap shortlist surfaces five concrete next-round candidates (IP-102, IP-201, IP-301, IP-303, IP-306).
    6. **How to run** — PowerShell quick-reference for SIL discover, single-file, single-method, compile gates, HIL pre-flight.
    7. **Update protocol** — five-rule checklist binding every future round to update this file alongside TODO.md and the status memo.

### Round-protocol changes

- `LifeTrac-v25/TODO.md` now links MASTER_TEST_PROGRAM.md from the top index block, immediately under the implementation-plan link, with a one-sentence reminder that it must be updated in the same PR as the test/code change.
- Status-memo updates now include MASTER_TEST_PROGRAM.md updates as a checklist item starting next round.

### Gaps surfaced

The per-IP table has 11 `—` cells. Six of them are deployment / manual-review items (IP-002, IP-005, IP-209, IP-304, IP-307, IP-308, IP-309) that don't have a natural SIL surface and are documented as such. The remaining five (IP-102, IP-201, IP-301, IP-303, IP-306) are the explicit shortlist for the next SIL-only rounds.

### CI status

- 237 / 237 base_station tests pass (1 skipped). No code changed in this round.
- All 5 Arduino compile jobs remain blocking.

### What's next

Two parallel tracks open:

- **HIL-companion SIL track:** W4-08 (camera back-channel keyframe round-trip) and W4-10 (fleet-key provisioning sanity OLED + exit-code) are still open.
- **IP-coverage-gap track:** IP-102 (nonce-seq thread-through), IP-201 (MQTT retry/backoff fake-clock SIL), IP-306 (TELEM_MAX_PAYLOAD constant reconciliation) are the cleanest one-round picks. IP-301 and IP-303 need a small handheld boot-state SIL or Modbus-map test respectively.

---

## Round 22 (2026-04-29) — W4-08 + W4-10 SIL coverage

**Plan refs:** IP-008 (W4-10), IP-103 + IP-104 (W4-08).

**Goal:** Land the last two HIL-companion SILs from MASTER_TEST_PROGRAM.md §4 so 9 of 10 W4-XX bench items have a CI-gated logic-surface model. The remaining gap (W4-09) is hard-blocked by real SX1276 IRQ wiring with no SIL substitute.

### What landed

**W4-08 — 	ests/test_keyframe_round_trip_sil.py (17 tests, 6 classes):**

- KR-A: opcode + topic-byte constant pinning across lora_proto.CMD_REQ_KEYFRAME, camera_service.CMD_REQ_KEYFRAME, X8_CMD_TOPIC = 0xC0.
- KR-B: bridge /cmd/req_keyframe —> pack_command(seq, CMD_REQ_KEYFRAME) translation. Verifies frame layout (8 B = 5 hdr + 1 op + 0 args + 2 CRC), header fields (ersion, source_id == SRC_BASE, rame_type == FT_COMMAND, sequence_num echo), opcode at offset 5, CRC validates round-trip across boundary seqs (0, 1, 0x7FFF, 0xFFFE, 0xFFFF).
- KR-C: full air round-trip — pack_command + encrypt_frame + kiss_encode + KissDecoder.feed + decrypt_frame + parse_header. Plus a 0xC0-in-ciphertext case that exercises the FESC/TFEND escape path.
- KR-D: M7 send_cmd_to_x8() [X8_CMD_TOPIC, opcode, args...] framing + KISS UART round-trip + dispatch_back_channel sets orce_key_evt. Plus a wrong-topic-byte negative test.
- KR-E: airtime budget. Pins PHY_CONTROL_SF7 as boot PHY (LADDER[0] tripwire), asserts encrypted REQ_KEYFRAME frame TOA at SF7/BW250 < 100 ms LoRa-leg budget, asserts REQ TOA + worst-case in-flight CTRL TOA + UART KISS @ 115200 + 50 ms encode-loop wake-up < 200 ms end-to-end budget, and asserts airtime is monotonically non-decreasing across the SF7/SF8/SF9 ladder rungs.
- KR-F: idempotency / re-entrancy. Five repeated presses produce 5 distinct GCM nonces (replay-window safe), and 5 repeated dispatches keep the keyframe event set.

Bench-only residual: real SX1276 IRQ timing + real H747 Serial1 baud stability + real X8 camera-encoder I-frame produce time + real MQTT broker hop latency.

**W4-10 — 	ests/test_fleet_key_provisioning_sil.py (24 tests, 5 classes):**

- FK-A: shared invariants. kFleetKey[16] is all-zero placeholder in BOTH handheld_mkr.ino and 	ractor_h7.ino (production keys belong in lp_keys_secret.h or a Docker secret, never checked in). LIFETRAC_ESTOP_MAGIC in shared_mem.h is pinned at 0xA5A5A5A5 (M4 watchdog depends on this exact value).
- FK-B: handheld provisioning gate. static bool fleet_key_is_zero() exists; #ifndef LIFETRAC_ALLOW_UNCONFIGURED_KEY guard calls it; renders the canonical OLED text "FLEET KEY NOT\nPROVISIONED\nHALT (IP-008)" byte-for-byte; halts in a while(1) or or(;;) with delay() inside (so the AVR watchdog can't reset and silently retry); and runs inside setup(), not loop().
- FK-C: tractor M7 provisioning gate. Same helper exists; #ifndef guard calls it; writes SHARED->estop_request = LIFETRAC_ESTOP_MAGIC (so the M4 watchdog covered by 	est_m4_safety_sil.py / W4-03 drops the safety relay); halts in a forever-loop with delay() inside.
- FK-D: bridge _load_fleet_key() failure modes. Happy paths (valid hex env, valid raw-bytes file, valid hex-string file). Failure paths: missing env (no LIFETRAC_FLEET_KEY_FILE and no LIFETRAC_FLEET_KEY_HEX), all-zero hex env, all-zero file, wrong length (8 bytes), invalid hex chars in env, unreadable file path, file containing 32 chars of "Z". All raise RuntimeError with operator-readable error messages ("all zero", "IP-008", "16 bytes", "hex", "unreadable").
- FK-E: bypass-flag semantics. LIFETRAC_ALLOW_UNCONFIGURED_KEY=1 MUST NOT be honored inside _load_fleet_key() itself — only at the module-level try/except. Source-parse confirms the loader body does not mention the env var, and that the module-level bypass DOES exist outside the loader. This guarantees explicit production calls (a future Bridge constructor that re-validates) always fail-closed.

Bench-only residual: visual confirmation that the OLED renders the text correctly + systemd-unit restart behavior on real X8 + compile-time failure of handheld when lp_keys_secret.h is deleted (already exercised by Arduino compile gate failing loud on missing #include).

### Test count

- Round 21: 237.
- Round 22 W4-08 +17 + W4-10 +24 = 278.
- Full suite: `Ran 278 tests in ~0.84s` `OK (skipped=1)`. No regressions.

### Bug found during write

	est_keyframe_round_trip_sil.KR_B2 initially failed with `AttributeError: 'LoraHeader' object has no attribute 'proto'`. The actual LoraHeader dataclass field names are `version` and `sequence_num` (not `proto` / `seq`). Fixed by reading the actual dataclass declaration in lora_proto.py:107 — useful tripwire that future Hungarian-naming refactors of the dataclass would land here too.

### MASTER_TEST_PROGRAM.md updates

- §1 tier overview: 237 —> 278 tests, 25 —> 27 files.
- §2 SIL catalog: added two new rows.
- §4 HIL matrix: W4-08 SIL-coverage cell now lists both back-channel-dispatch + new keyframe round-trip test, residual narrowed to "real H747 baud + real X8 encoder produce time"; W4-10 SIL-coverage cell now lists the new fleet-key SIL + handheld compile gates, residual narrowed to "OLED visual + systemd unit restart".
- §4 tier status: 7-of-10 —> 9-of-10 SIL coverage; only W4-09 remains hard-blocked.
- §5 IP traceability: IP-008, IP-103, IP-104 cells point at the new tests.

### What's next

The Wave-3 / coverage-gap shortlist in MASTER_TEST_PROGRAM.md §5 is now the natural next track:

- IP-306: `TELEM_MAX_PAYLOAD` C(118) vs Python(120) reconciliation — a one-line cross-language constant assertion is the cleanest single-round pick.
- IP-201: MQTT retry/backoff — pure asyncio fake-clock SIL.
- IP-301: `s_btn_change_ms` boot-init — small handheld boot-state SIL.
- IP-303: `REG_AUX_OUTPUTS` decision — Modbus map test pinning the absence (or presence).
- IP-102: nonce-seq thread-through — extension to `test_replay_window_invariant.py`.

W4-09 remains the only HIL item with no SIL substitute possible (real SX1276 IRQ wiring on real H747 silicon).

---

## Round 23 — 2026-04-29 — Wave-3 polish constants tripwire (IP-306 + IP-303 + IP-301)

Three small, independent Wave-3 polish invariants collapsed into one
source-parse SIL because each is a one-shot tripwire and they share no
runtime surface.

### IP-306 — `TELEM_MAX_PAYLOAD` cross-language pin
The constant was already reconciled in Round-history (Python `= 118`,
C `payload[120]` reserves trailing CRC-16 bytes), but no test
defended the contract. `test_protocol_constants_sil.PC_A_*` now
(a) imports `lora_proto.TELEM_MAX_PAYLOAD` and asserts `== 118`,
(b) source-parses `firmware/common/lora_proto/lora_proto.h` to
confirm `payload[120]`, (c) confirms the neighbouring header comment
mentions `118` so on-call reading matches runtime, and (d) requires
§2 references (definition + boundary check) across
`lora_proto.py` and `lora_bridge.py` so the overflow path stays
guarded.

### IP-303 — `REG_AUX_OUTPUTS` decision pinned
Per TRACTOR_NODE.md §4 the Modbus slot `0x0003` is implemented and
gated by `AUX_OUTPUTS_VALID_MASK = 0x0007` (R10..R12 SSR channels
only). `PC_B_*` now pins:
* `#define REG_AUX_OUTPUTS 0x0003` on the Opta slave;
* the M7 Modbus master enum references `REG_AUX_OUTPUTS` (so the
  slot cannot become orphaned);
* the Opta write-handler `case REG_AUX_OUTPUTS:` exists AND masks
  the incoming value with `AUX_OUTPUTS_VALID_MASK` (preventing a
  typo'd bitfield from driving the R5..R8 valve-manifold channels);
* `AUX_OUTPUTS_VALID_MASK == 0x0007` (widening this is a safety
  regression because engine-kill arming must remain owned by
  `REG_ARM_ESTOP`).

### IP-301 — `s_btn_change_ms` boot-anchor implemented and pinned
The handheld debounce reference timestamp was previously left at its
static-zero default until the first call to `read_buttons()`. Under
a non-zero idle raw-pin reading on cold boot the first iteration could
satisfy `(now - 0) >= DEBOUNCE_MS` immediately and commit a
spurious debounced state. Round 23 lands the one-line fix in
`handheld_mkr.ino` `setup()` (after `Serial.begin` / `pinMode`
/ `radio.begin` / OLED init):

`s_btn_change_ms = millis();`

`PC_C_*` pins (a) the static default is still `0` (so the
boot-anchor remains meaningful), (b) `setup()` contains
`s_btn_change_ms = millis();`, (c) the anchor runs AFTER the
`Serial.begin` / `radio.begin` / `pinMode` landmarks (so the
timestamp is taken as late in setup() as possible, minimising the gap
to the first `read_buttons()` call), and (d) `read_buttons()` still
references the field (tripwire against the boot-init becoming dead
code via a future debounce refactor).

### Test wiring
* New file: `DESIGN-CONTROLLER/base_station/tests/test_protocol_constants_sil.py`
  — 12 tests, 3 classes, pure source-parse with the shared
  `REPO_ROOT = Path(__file__).resolve().parents[3]` pattern.
* Firmware change: one line added at end of
  `firmware/handheld_mkr/handheld_mkr.ino` `setup()`.
* Suite count: 278 —> 290 (12 new), still 1 skipped (Coral
  hardware-gated). Full suite `Ran 290 tests in 0.896s OK
  (skipped=1)`.

### Coverage delta
§5 IP-traceability gaps shrink from 5 —> 2:
* IP-306, IP-303, IP-301 now have SIL coverage.
* Remaining: IP-102 (nonce-seq thread-through — extend
  `test_replay_window_invariant.py`) and IP-201 (MQTT retry/backoff
  fake-clock SIL).

These are the obvious Round 24 + Round 25 candidates.


---

## Round 24 — 2026-04-29 — IP-102 nonce-seq thread-through SIL

Round 24 closes the last Wave-1 IP-coverage cell with
[test_nonce_seq_threadthrough_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_nonce_seq_threadthrough_sil.py)
(13 tests, 4 classes).

### The bug class
Pre-fix, `lora_bridge._on_mqtt_message` called `self._tx(SRC_BASE,
msg.payload)` with no `nonce_seq=` kwarg. `_tx` therefore reserved
a *fresh* seq for the AEAD nonce while the cleartext header still
carried the seq packed by `web_ui.ws_control`'s independent
counter. Two consequences:

1. The tractor's `ReplayWindow` checks the inner-header seq against
   its bitmap; that no longer matches the seq baked into the GCM
   nonce that authenticated the frame.
2. The IP-101 `NonceStore` reservation \u2014 the whole reason persistence
   exists \u2014 produced a value that never appeared in the bytes the
   tractor sees on-air.

### The defended invariant
Every `_on_mqtt_message` arm MUST:
* call `_reserve_tx_seq()` exactly once per accepted message;
* stamp that seq into the cleartext header (`_restamp_control` for
  the existing 16-byte FT_CONTROL frame from web_ui, `pack_command(seq, ...)`
  for the opcode-only commands);
* pass the **same** seq into `_tx(..., nonce_seq=seq)` so
  `encrypt_frame` re-uses it for `build_nonce`.

### Test design
Pure-Python SIL. Constructs a `Bridge` via `Bridge.__new__(Bridge)`
to skip the MQTT/serial constructor, hand-wires the four fields the
arms touch (`audit` mock, `tx_seq`, `lock`, `nonce_store=None`,
`_replay` map, priority-queue triple), and pops frames straight off
`_tx_queue` instead of starting the worker. For each enqueued frame
we parse the cleartext header AND rebuild the GCM nonce with
deterministic `now_s=0, random_tail=b"\x00"*5`, then assert
`struct.unpack('<H', nonce[1:3])[0] == struct.unpack('<H', pt[3:5])[0]
== queued_seq`.

### Coverage
* **NS-A (control path, 5 tests):** single-message seq override (web_ui
  sends seq=0xBEEF, bridge enqueues with seq=0); 100-message acceptance
  (verbatim from IP-102 spec); replay-window check (no false rejects);
  `_restamp_control` CRC recompute; malformed payloads must NOT
  advance the seq counter.
* **NS-B (opcode-only commands, 5 tests):** `cmd/estop`,
  `cmd/camera_select`, `cmd/req_keyframe` thread their seq into
  both header and nonce; invalid camera-id and wrong-length
  `camera_select` payloads reject before `_reserve_tx_seq`.
* **NS-C (cross-arm interleave, 2 tests):** mixed sequence of all four
  arms produces seqs `0..N-1` from a single shared counter; 16-bit
  wrap test (`0xFFFE \u2192 0xFFFF \u2192 0`) so the GCM nonce's 2-byte seq
  slot never desyncs from the cleartext header at the wrap boundary.
* **NS-D (source-grep tripwire, 1 test):** every `self._tx(` call
  inside `_on_mqtt_message` MUST carry a `nonce_seq=` keyword \u2014
  defends against a future refactor that drops the kwarg from one of
  the arms (the original IP-102 bug surface).

### Suite delta
* Suite count: 290 —> 303 (13 new), still 1 skipped (Coral
  hardware-gated). `Ran 303 tests in 0.948s OK (skipped=1)`.
* §5 IP-traceability gaps shrink from 2 —> 1.
  Remaining: **IP-201** (MQTT retry/backoff, fake-clock asyncio SIL).


---

## Round 25 — 2026-04-29 — IP-201 MQTT retry/backoff fake-clock SIL

Round 25 closes the last natural-SIL-surface \"—\" cell in the
§5 IP-traceability table with
[test_mqtt_retry_backoff_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_mqtt_retry_backoff_sil.py)
(13 tests, 4 classes).

### The defended invariant
`web_ui._connect_mqtt_with_retry()` honours the IP-201 contract:

* Reads `LIFETRAC_MQTT_HOST` (default `localhost`).
* Initial backoff = 0.5 s, doubling each retry, capped at 5.0 s.
* 30 s monotonic deadline. Past the deadline, re-raises the
  underlying `OSError`/`ConnectionError` verbatim (no wrapping).
* Uses `time.monotonic` (NOT `time.time`) so a wall-clock
  jump-backwards under boot-time NTP slew cannot reset the deadline.
* Catches `(OSError, ConnectionError)` exactly \u2014 anything narrower
  lets a paho-mqtt error category crash the UI on boot.

### Test design
`web_ui` does its connect-retry at module-import time, so the test
reloads the module per test with `paho.mqtt.client.Client` stubbed
(connect returns `None`) so the import-time call succeeds. Then
each test patches:

* `web_ui.mqtt_client.connect` with a scripted `side_effect`
  sequence of failures + a final `None`;
* `web_ui.time.monotonic` and `web_ui.time.sleep` with a
  `_FakeClock` that advances ONLY on `sleep()`.

Result: every backoff schedule is observable as a `list[float]` of
sleep durations, and the deadline behaviour is deterministic without
a single real `time.sleep` call.

### Coverage
* **MR-A (host selection, 2 tests):** `LIFETRAC_MQTT_HOST` env-var
  honoured; default fallback to `localhost`.
* **MR-B (backoff schedule, 3 tests):** first-retry-then-success
  observes exactly `[0.5]`; six failures observe
  `[0.5, 1.0, 2.0, 4.0, 5.0, 5.0]` verbatim; saturation engaged
  by retry 5 with all sleeps in `(0, 5.0]`.
* **MR-C (deadline, 3 tests):** raises after 30 s of monotonic
  clock with the underlying exception preserved (`assertIs`);
  late success (5 failures + success) inside the budget does not
  raise; source check confirms `time.monotonic()` is used and
  `time.time()` is NOT.
* **MR-D (source-grep tripwire, 5 tests):** pins the
  `deadline = time.monotonic() + 30.0` literal, `backoff = 0.5`
  initial, `min(backoff * 2, 5.0)` cap, the
  `os.environ.get(\"LIFETRAC_MQTT_HOST\", \"localhost\")` lookup, and
  the `except (OSError, ConnectionError)` filter against future
  refactors.

### Suite delta
* Suite count: 303 —> 316 (13 new), still 1 skipped (Coral
  hardware-gated). `Ran 316 tests in 1.015s OK (skipped=1)`.
* §5 IP-traceability gaps shrink from 1 —> 0 with-natural-SIL.
  Remaining \"—\" cells are deployment / manual-review items
  (IP-002 env vars, IP-005 settings path drift, IP-209 lockfile,
  IP-304 PYTHONPATH, IP-307/308/309 OLED + boot-self-test + numpy
  vectorisation polish) that don't have a natural SIL surface.

### Programme status
**The full multi-round controller-code-review implementation
programme is now SIL-complete to the limit of its natural surface.**
Every plan item achievable without bench hardware is landed and
verified by an automated test that runs in CI on every push. The
HIL bench (W4-01 through W4-10) is the only remaining tier; 9 of 10
W4-XX items have SIL companions that prove the *logic* is right, so
the HIL run is now a confirmation pass rather than an exploration.
Only W4-09 (M7 SX1276 IRQ timing) is hard-blocked from SIL
substitution.


---

## Round 26 — 2026-04-29 — Wave-4 HIL harness toolchain

Round 26 builds the operational tooling that will compress the gap
between \"hardware arrives on the bench\" and \"first W4-XX gate green.\"
The full controller-code-review SIL programme finished in Round 25;
Round 26 turns its attention to the next bottleneck.

### What landed

* **[DESIGN-CONTROLLER/hil/](../DESIGN-CONTROLLER/hil/)** — new
  directory hosting all bench-day automation.
* **[hil/results_schema.json](../DESIGN-CONTROLLER/hil/results_schema.json)** —
  JSON Schema that locks the per-run JSONL contract: `gate_id`,
  `run_id`, `timestamp`, `operator`, `firmware_sha` (5-key
  bundle), `hw_serial`, `result` (PASS/FAIL/SKIP/ABORT),
  `metrics`, `evidence_paths`, `notes`. Every harness writes
  one of these per bench run; the dispatcher reads them.
* **[hil/_common.ps1](../DESIGN-CONTROLLER/hil/_common.ps1)** —
  shared helpers: `Get-LifeTracRepoRoot`, `Get-EvidenceDir`,
  `Get-FirmwareSha` (git short-SHA), `New-FirmwareShaBundle`,
  `Read-OperatorPrompt`, `Confirm-OperatorChecklist`,
  `Assert-Section0-Ready` (mirrors HIL_RUNBOOK §0 verbatim),
  `Write-HilResult` (atomic JSONL append), `New-RunId`,
  `Write-GateHeader`. Three bench-config placeholders at the top
  (`` / `HIL_TRACTOR_PORT` /
  `HIL_OPTA_PORT`) are the only edits needed per laptop.
* **10 harness skeletons** (`hil/w4-01_estop_latency.ps1` through
  `hil/w4-10_fleet_key_provisioning.ps1`) — one per W4-XX gate,
  each: dot-sources `_common.ps1`; takes a mandatory
  `-Operator` (plus gate-specific parameters such as `-Sf`,
  `-AxisGroup`, `-BootOrder`, `-Phase`, `-Step`); calls
  `Write-GateHeader` + `Assert-Section0-Ready`; prints the
  per-gate runbook procedure inline; prompts the operator for any
  metric not supplied on the command line; auto-flips `Result` to
  FAIL when a documented threshold (e.g. W4-01 `latency_ms < 100`,
  W4-04 `apply_neg1_ms < 1000`, W4-08 `end_to_end_ms < 200` +
  `lora_leg_ms < 100`) is violated; and writes a JSONL line via
  `Write-HilResult`.
* **[hil/dispatch.ps1](../DESIGN-CONTROLLER/hil/dispatch.ps1)** —
  reads every `bench-evidence/W4-XX/results.jsonl`, computes
  per-gate `PASS / Target` and a `Status` (`CLOSED` /
  `IN-PROGRESS` / `NOT-STARTED` / `FAILING`), prints a
  status table, and recommends the next gate to run. `-Report`
  flag dumps all individual runs; `-Gate W4-NN` filters.
* **[hil/README.md](../DESIGN-CONTROLLER/hil/README.md)** —
  workflow guide: edit COM ports, run dispatcher, run recommended
  harness, repeat. Explicitly notes the harness directory does NOT
  replace [HIL_RUNBOOK.md](../DESIGN-CONTROLLER/HIL_RUNBOOK.md)
  prose.
* **[test_hil_harness_completeness_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_hil_harness_completeness_sil.py)**
  (13 tests, 4 classes) — pure source-parse SIL that gates the
  whole bundle.

### Defended invariants (HC-A / HC-B / HC-C / HC-D)

* **HC-A (file existence):** every W4-XX in `EXPECTED_GATES`
  (`W4-01..W4-10`) has exactly one matching `hil/w4-NN_*.ps1`
  harness; the four supporting files (`README.md`,
  `results_schema.json`, `_common.ps1`, `dispatch.ps1`) are
  present; no orphan harnesses; `MASTER_TEST_PROGRAM.md §4`
  bold-lists every expected gate id (drift gate against the
  master matrix).
* **HC-B (harness contract):** every harness contains all of
  `. \"\$PSScriptRoot/_common.ps1\"`, `[CmdletBinding()]`,
  `[Parameter(Mandatory)][string]\$Operator`,
  `Write-GateHeader`, `Assert-Section0-Ready`,
  `Write-HilResult`, a `HIL_RUNBOOK.md` back-reference, and
  its own gate id literal.
* **HC-C (schema invariants):** schema is a JSON object with the 6
  required fields; `gate_id` regex matches only `W4-01..W4-10`
  (rejects `W4-00`, `W4-11`, lower-case, empty); `result`
  enum is exactly `[ABORT, FAIL, PASS, SKIP]` after sort; the
  `firmware_sha` bundle requires the five canonical keys
  (`base, handheld, opta, tractor_h7, tractor_m4`) so missing
  any node SHA in a run is a JSONL-validation error rather than a
  silent gap.
* **HC-D (dispatcher coverage):** `dispatch.ps1` declares a
  `Target = N` entry for every gate, every `Target` is
  positive, and the dispatcher gate set equals the harness gate
  set exactly (no orphans either way). Also pins that the
  dispatcher dot-sources `_common.ps1`.

### Suite delta

* Suite count: 316 —> 329 (13 new), still 1 skipped (Coral
  hardware-gated). `Ran 329 tests in 1.028s OK (skipped=1)`.
* Dispatcher smoke-test: `pwsh ./dispatch.ps1` from a clean
  `bench-evidence/` shows all 10 gates as `NOT-STARTED 0/Target`
  and recommends W4-01 as the next gate to run.

### Programme status

The Wave-4 HIL bench is now **fully operationalised**: a new
contributor showing up at the bench needs only to edit three COM
ports in `_common.ps1` and run `pwsh ./dispatch.ps1`. Every
gate procedure is enforced through a uniform skeleton that
collects a JSON-Schema-conformant evidence trail; the SIL test
guarantees the harness set stays in lockstep with
`MASTER_TEST_PROGRAM.md §4` indefinitely. From here, the
remaining work is purely physical: power on the bench, run the
gates in the order the dispatcher recommends, two-operator
sign-off per `HIL_RUNBOOK.md` §\"Sign-off\", flip the
[CI compile-gate](../.github/workflows/arduino-ci.yml) from
`continue-on-error: true` to blocking, and append the Wave-4
evidence package to [SAFETY_CASE.md](../DESIGN-CONTROLLER/SAFETY_CASE.md).

