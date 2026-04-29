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


## Round 27 — BC-01 + BC-02: capability inventory + build-config loader (2026-04-29)

**Outcome:** the BC-XX initiative is open. v25 now has a single
source of truth for *which* optional capabilities are present on a
given unit and *what* the per-unit parameters are, with a
strict-by-default loader that the rest of the codebase can consume in
later rounds. Suite went 329 —> 349 (+20).

**Why this round:**
[Round 26](#round-26-—-wave-4-hil-harness-toolchain-2026-04-29) closed the
last natural surface for hardware-readiness tooling. The next biggest
unaddressed risk was **build heterogeneity** — not every operator
will run with a second camera, an IMU/GPS pair, the Coral TPU, dual
arm axes, or a dual-channel PSR. Today those assumptions are
hard-coded in five places (web_ui camera tile, Modbus error spam for
absent inputs, IMU/GPS publishers, GPS-zero audit poisoning, HIL gates
asking for absent hardware). BC-XX exists to make that
config-as-data; BC-01 catalogues, BC-02 ships the schema and loader.

**What landed (BC-01):**

* [DESIGN-CONTROLLER/CAPABILITY_INVENTORY.md](DESIGN-CONTROLLER/CAPABILITY_INVENTORY.md)
  — markdown table per section (identity / hydraulic / safety /
  cameras / sensors / comm / ui / net) listing capability id, type,
  default, range/enum, and consumer modules. Captures the realistic
  field range for every optional or parameterised line in
  [HARDWARE_BOM.md](DESIGN-CONTROLLER/HARDWARE_BOM.md): axis count
  (track 1—2, arm 0—2), proportional-flow yes/no, three E-stop
  topologies (psr_monitored_dual / psr_monitored_single /
  hardwired_only), camera count 0—4 with per-position presence
  flags, IMU model (BNO086 / BNO055 / ICM-20948), GPS model
  (NEO-M9N / NEO-M9N+F9P RTK), LoRa region, Coral TPU bus
  (mini-PCIe / USB / none), MQTT host+port. Section §
  *How a capability ends up here* documents the eligibility rule;
  section § *Drift gate* points at the BC_E inventory-parity
  test.

**What landed (BC-02):**

* [ase_station/config/build_config.schema.json](DESIGN-CONTROLLER/base_station/config/build_config.schema.json)
  — draft-07 JSON Schema. Top level + every nested section is
  dditionalProperties: false, so unknown keys fail validation
  loudly. Range / enum / pattern constraints encode the inventory
  ranges. § The schema is human-curated, not machine-generated;
  the BC_E parity test guarantees it stays in sync with the doc.
* [ase_station/config/build.default.toml](DESIGN-CONTROLLER/base_station/config/build.default.toml)
  — canonical-BOM build (lifetrac-001, dual track + dual arm,
  proportional flow, dual-channel PSR, 1 front camera + Coral mini-PCIe,
  BNO086 IMU + NEO-M9N GPS, US915, MQTT localhost:1883).
* [ase_station/build_config.py](DESIGN-CONTROLLER/base_station/build_config.py)
  — stdlib-only loader. Public API:
  load(unit_id: str | None = None) -> BuildConfig. Resolution
  order: LIFETRAC_BUILD_CONFIG_PATH env var (explicit absolute
  path) —> config/build.<unit_id>.toml if unit_id given and
  the file exists —> config/build.default.toml (the floor;
  must always exist). BuildConfig is a frozen dataclass with one
  frozen sub-dataclass per section + source_path + 
aw (the
  validated dict) + config_sha256 (SHA-256 over canonical JSON,
  sorted keys, no whitespace — this is what BC-04 will write to
  the audit log on every boot). The JSON-Schema validator is
  hand-rolled over the small draft-07 subset we use (object /
  required / properties / additionalProperties:false / enum /
  minimum / maximum / pattern / minLength / maxLength / type
  integer-rejects-bool) so we add zero third-party deps.

**What landed (SIL gate):**

* [ase_station/tests/test_build_config_loader_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_loader_sil.py)
  — 20 tests across 5 classes, all passing first run.
  * **BC_A_SchemaWellFormedness (3 tests)** — schema top level is
    a strict object with 
equired + properties; every nested
    section is itself a strict object; the canonical default TOML
    satisfies the schema's top-level 
equired.
  * **BC_B_LoaderFallbackChain (4 tests)** — default load returns
    the canonical (source_path == build.default.toml,
    unit_id == "lifetrac-001", 	rack_axis_count == 2); per-unit
    override beats default (writes a temp uild.lifetrac-test-unit.toml
    with cameras.count = 0 and confirms the loader reads it);
    LIFETRAC_BUILD_CONFIG_PATH env beats per-unit beats default
    (synthesises uild.env-target.toml and confirms env wins);
    env path pointing at a missing file raises BuildConfigError.
  * **BC_C_StrictValidation (7 tests)** — unknown top-level key
    rejected with key name in message; unknown nested key rejected;
    out-of-range integer rejected; wrong type rejected (bool vs
    string); bad enum token rejected with enum in message;
    pattern-violating unit_id ("Has Spaces!") rejected with
    pattern in message; missing required section ([comm] stripped)
    rejected with section name in message.
  * **BC_D_DeterministicSha256 (3 tests)** — config_sha256
    matches the explicit canonical-JSON formula
    sha256(json.dumps(raw, sort_keys=True, separators=(",", ":")));
    invariant under TOML whitespace + section re-ordering ([net]
    swapped before [ui]); changes when any leaf changes.
  * **BC_E_InventoryParity (3 tests)** — the inventory doc exists;
    every property the schema declares appears as a backticked id
    token in the inventory; no orphan inventory rows without a
    matching schema property. **This is the third-place drift
    detector.** Adding a capability is now a three-place edit (doc,
    schema, default TOML) and the SIL test refuses to let any two of
    those drift apart.

**Test growth:**
329 —> 349 (+20). Full base_station suite still runs in ~1.6 s.

**Wrap-up artifacts:**

* [TODO.md](TODO.md) status block bumped to "through Round 27" and
  prepended with the BC-01 + BC-02 narrative.
* [MASTER_TEST_PROGRAM.md](MASTER_TEST_PROGRAM.md) § 1 totals
  —> 32 files / 349 tests; § 2 SIL catalog row inserted for
  	est_build_config_loader_sil.py; § 5 IP-traceability gains a
  new "Wave 4 — bench-bring-up tooling and build-configuration"
  block carrying HIL-01 + BC-01 + BC-02 rows; coverage-gap shortlist
  re-anchored to "as of Round 27."
* This memo.

**Defers (queued for later rounds, not in this one):**

* BC-03: pre-build script that lowers the BuildConfig into a
  irmware/common/lifetrac_build_config.h with #define
  LIFETRAC_HAS_* flags + parameter literals so sketches can
  #if LIFETRAC_HAS_*-guard hardware that may be absent. Adds three
  reduced-BOM compile gates to CI.
* BC-04: web_ui.py and lora_bridge.py actually consume the
  loaded BuildConfig instead of the current hard-coded constants;
  audit-log every boot's config_sha256.
* BC-05: PIN-gated /config admin route in web_ui.py with a
  schema-driven form, atomic .toml.next —> rename —> audit
  —> restart prompt.
* BC-06: extend hil/dispatch.ps1 to read the active config and
  flag inapplicable gates as N/A instead of failing them.
* BC-07: variant-matrix SIL exercising 4 configurations
  (canonical, no-camera, no-IMU/GPS, single-axis) end-to-end through
  web_ui + lora_bridge with LIFETRAC_BUILD_CONFIG_PATH pointed
  at fixture TOMLs.
* BC-08: onboarding doc at
  DESIGN-CONTROLLER/BUILD_CONFIG.md (how to author a per-unit
  TOML, common patterns, troubleshooting).
* BC-09: cross-reference between HARDWARE_BOM.md optional rows and
  capability ids, with an orphan SIL gate.

**Known limitations of BC-02 as shipped:**

* Loader is **read-only**. Mutation (BC-05 admin form) is a separate
  initiative.
* No fleet-wide config delivery over LoRa / cellular — a
  per-unit TOML lands on the unit by SSH or USB stick. OTA delivery
  is explicitly out-of-scope per the BC-XX scope statement.
* BuildConfig.raw carries only the validated dict — loader does
  not retain TOML comments. If we ever want to round-trip TOML for
  the admin form, we'll switch to 	omli_w or 	omlkit in BC-05.


## Round 28 — BC-04: web_ui + lora_bridge consume BuildConfig (2026-04-29)

**Outcome:** the BC-XX initiative crosses from "config sits on disk"
to "config changes runtime behaviour." Both base-station Python
daemons now load the active build configuration at boot, audit-log
its identity, and use selected fields to drive UI surface area.
Suite went 349 —> 360 (+11).

**What landed (web_ui):**

* Module-level BUILD = build_config.load(os.environ.get("LIFETRAC_UNIT_ID"))
  with a try/except fallback to BUILD = None so dev checkouts
  without a config file still boot a degraded console rather than
  crash-loop.
* _audit_config_loaded() helper called once at module-import
  time, writes a config_loaded audit-log line carrying
  component="web_ui", unit_id, source_path, and
  config_sha256. No-ops cleanly when BUILD is None or the
  audit log is unavailable.
* _CAMERA_IDS is now _filter_cameras(_CAMERA_IDS_FULL). The
  filter inspects BUILD.cameras.{front,rear,implement,crop_health}_present
  and drops absent positions; cameras.count == 0 collapses the
  table entirely (the /api/camera/select API then returns HTTP
  400 \"unknown camera\" for every request). Default canonical
  build = {auto, front}. Full-loadout variant = all five.
  Loader-failure fallback = full hard-coded set (preserves dev-
  checkout behaviour).
* MAX_CONTROL_SUBSCRIBERS = BUILD.ui.max_control_subscribers if
  BUILD is not None else 4 — the historical hard-coded value
  becomes the dev-checkout default; production reads it from the
  schema-validated config.

**What landed (lora_bridge):**

* Bridge.__init__ gained a 7-line block right after the existing
  ridge_start audit record: load BuildConfig, write
  config_loaded with the same field set the web_ui emits but
  component="lora_bridge". Best-effort, same try/except pattern.
  No other consumer surface yet — the bridge's hot paths
  (Modbus failure latch count, IMU/GPS telemetry filtering, ramp
  seconds) are deferred to Round 30+ once BC-10 hot-reload is in
  place, so the bridge doesn't have to restart on every config
  change.

**What landed (SIL gate):**

* [ase_station/tests/test_build_config_consumption_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_consumption_sil.py)
  — 11 tests across 5 classes, all passing first run.
  * **BC4_A_AuditBootRecord (3 tests)** — web_ui's
    _audit_config_loaded() writes one config_loaded line with
    the expected field set; no-ops when BUILD is None; the
    inlined lora_bridge audit-block produces the same field set
    with component="lora_bridge".
  * **BC4_B_CameraGating (3 tests)** — canonical build (front
    only) advertises {auto, front}; a synthesised full-loadout
    variant (count=4 + all four position flags = true) advertises
    all five; a no-camera variant (count=0, front=false,
    coral=none) collapses the table to {}.
  * **BC4_C_ParameterSubstitution (2 tests)** — a variant with
    max_control_subscribers = 12 is read through to
    web_ui.MAX_CONTROL_SUBSCRIBERS == 12; a missing config file
    falls through to the hard-coded 4.
  * **BC4_D_GracefulDegradation (1 test)** — pointing
    LIFETRAC_BUILD_CONFIG_PATH at a non-existent file leaves
    web_ui.BUILD is None but the module still imports and the
    full hard-coded camera table is in effect.
  * **BC4_E_SourceTripwire (2 tests)** — both web_ui.py and
    lora_bridge.py contain greppable BC-04 + config_loaded
    references so future refactors can't silently delete the
    audit call.

  Implementation notes that were non-obvious:
  * The test reloads web_ui per case via importlib.import_module
    after sys.modules.pop(...) so the module-level BUILD and
    _CAMERA_IDS re-evaluate against the test's
    LIFETRAC_BUILD_CONFIG_PATH fixture. paho's Client is
    patched at the class level so _connect_mqtt_with_retry()
    returns instantly without a real TCP connect.
  * lora_bridge.Bridge.__init__ opens a serial port + a real
    MQTT client; rather than mocking the entire constructor the
    test exercises the audit-block code path inline against a
    fake AuditLog and pins the source-text to ensure the call is
    actually present. Source-grep is the contract; behaviour is
    end-to-end-tested at HIL bring-up.

**Deliberate non-changes this round:**

* Did NOT change lora_bridge's Modbus latch count, telemetry
  field filtering for absent IMU/GPS, or any axis/ramp constant.
  Those need BC-10's hot-reload contract first so we can change
  values without restarting the daemon mid-mission.
* Did NOT touch the firmware sketches (BC-03's territory).
* Did NOT add a /config admin route (BC-05).
* MQTT host: LIFETRAC_MQTT_HOST env var still wins over the
  config's 
et.mqtt_host. This preserves the IP-201 retry-
  backoff contract pinned by Round 25 and keeps the docker-
  compose deployment path working unchanged.

**Test growth:** 349 —> 360 (+11). Suite still ~1.25 s.

**Wrap-up artifacts:**

* [TODO.md](TODO.md) status block bumped to "through Round 28"
  and prepended with the BC-04 narrative.
* [MASTER_TEST_PROGRAM.md](MASTER_TEST_PROGRAM.md) § 1
  totals —> 33 files / 360 tests; § 2 SIL catalog row
  inserted for 	est_build_config_consumption_sil.py; § 5
  Wave-4 IP-traceability gains a BC-04 row; coverage-gap
  shortlist re-anchored to "as of Round 28."
* This memo.

**Next up (per the round-ordering note added to BC-XX in Round 27):**

* **Round 29 = BC-10** — config delivery + hot-reload contract
  (laptop USB-cable CLI + base-UI-generated installer bundle,
  quiescence gate, live vs 
estart_required reload-class
  annotations on every schema property).
* Round 30 = BC-05 (web admin form, thin on top of BC-10).
* Round 31 = BC-03 (firmware codegen + reduced-BOM compile gates).


## Round 29 — BC-10 (schema + library half): hot-reload contract (2026-04-29)

**Outcome:** the schema/library half of BC-10 lands. Every leaf in
the build-config schema is now annotated with how a change to it is
meant to land on a running unit, and the loader exposes the two
helpers (`diff_reload_classes` + `evaluate_quiescence`) that
future delivery code will call. Suite went 360 —> 379 (+19).
The actual `lifetrac-config push` CLI, base-UI installer-bundle
download route, and X8-side installer daemon are deferred to **Round
29b** so this round stays a clean schema/library PR; the contract
pinned here is what 29b builds against.

**Why this round before BC-05 (web admin form):** BC-05 is a thin web
UI on top of BC-10's contract. Without `diff_reload_classes` the
web form has no idea whether to apply a save in place, queue a
restart-required reload, or refuse a firmware-required edit; without
`evaluate_quiescence` it can't decide whether right now is a safe
moment to commit. So BC-10 schema + library lands first, then BC-29b
puts an installer in front of it, then BC-05 puts the web form on
top.

**What landed (schema):**

* Every leaf in [uild_config.schema.json](DESIGN-CONTROLLER/base_station/config/build_config.schema.json)
  carries a `reload_class` keyword. The set is exactly
  `{"live", "restart_required", "firmware_required"}` — encoded
  as `RELOAD_CLASSES` in the loader, pinned by SIL.
* `firmware_required` is **deliberately small**: only
  `schema_version` (a schema bump is by definition a breaking
  change requiring new code on both ends) and `safety.m4_watchdog_ms`
  (the M4-side watchdog literal is compiled into the M4 sketch).
  BC10_A pins this exact set so a future schema edit can't silently
  add or drop a member.
* `restart_required` covers things that need driver/radio re-init or
  a graceful daemon restart: `unit_id`, axis counts,
  `proportional_flow`, `estop_topology`, `estop_latency_ms_max`,
  `cameras.coral_tpu`, `imu_model`, `gps_model`, `lora_region`,
  `cellular_backup_present`, `ui.web_ui_enabled`.
* Everything else is `live` (UI strings, presence flags, ramp
  seconds, MQTT host/port via reconnect, `max_control_subscribers`,
  `hyd_pressure_sensor_count`, `modbus_fail_latch_count`).

**What landed (loader helpers in [uild_config.py](DESIGN-CONTROLLER/base_station/build_config.py)):**

* `RELOAD_CLASSES` constant tuple + private `_RELOAD_CLASS_RANK`
  for strictness ordering.
* `iter_reload_classes(schema=None) -> dict[dotted_path, reload_class]`
  walks the schema's leaves and returns the classification map.
  Missing or out-of-enum annotations raise `BuildConfigError` —
  the schema is the contract; an un-annotated leaf is a bug, not an
  implicit "default to live."
* `ReloadDiff` frozen dataclass: `changed` tuple of dotted paths,
  `classes` mapping per-path, `worst` strictest class present
  (or `None` when nothing changed). Convenience properties
  `is_empty`, `restart_required`, `firmware_required`.
* `diff_reload_classes(old, new, schema=None)` flattens both raw
  dicts and walks the union of keys. A leaf that's in the data but
  not the schema raises (signalling schema/loader drift) rather
  than defaulting to `live` — silent defaults are how config
  systems stop being trustworthy.
* `QuiescenceState` frozen dataclass capturing the four runtime
  inputs the live-reload predicate depends on: `parked_seconds`,
  `active_control_subscribers`, `m7_tx_queue_depth`,
  `engine_idle_or_off`. Pure data; no I/O.
* `QuiescenceResult` frozen dataclass: `ok` bool plus an
  operator-facing `reason` string, plus `__bool__` so callers
  can write `if evaluate_quiescence(state):`.
* `evaluate_quiescence(state, *, parked_seconds_min=30.0)` checks
  the four conditions in order and returns the first failure
  reason (so the OLED line / web banner can show one coherent
  message rather than a bullet list). Default 30 s parked floor;
  threshold is a kwarg so the SIL test and future BC-05 admin
  form can override it.

**What landed (SIL gate):**

* [ase_station/tests/test_build_config_delivery_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_delivery_sil.py)
  — 19 tests across 5 classes, all green first run.
  * **BC10_A_ReloadClassAnnotations (5 tests)** — every leaf has
    a `reload_class`; `RELOAD_CLASSES` enum is exact;
    `firmware_required` set equals `{schema_version,
    safety.m4_watchdog_ms}` exactly; missing annotation raises;
    unknown annotation raises.
  * **BC10_B_ReloadClassDiff (5 tests)** — identical configs
    yield empty diff (`is_empty`, `worst is None`,
    `restart_required` False); a single `live` change classifies
    correctly; a single `restart_required` change does too; a
    mixed `live + restart_required + firmware_required` change
    surfaces all three paths and `worst == "firmware_required"`;
    a leaf that exists in the data but not the schema raises
    instead of being silently ignored.
  * **BC10_C_Quiescence (6 tests)** — the all-OK state passes;
    each of the four preconditions blocks independently (parked
    seconds < threshold; control-subscribers > 0; M7 TX queue > 0;
    engine under load) and produces a reason string mentioning the
    blocked condition by name; the parked-seconds threshold is a
    kwarg and lowering it lets a previously-blocked state pass.
  * **BC10_D_LeafCoverage (1 test)** — the canonical
    `build.default.toml` flattened leaves equal the schema's
    leaves (no drift in either direction).
  * **BC10_E_SourceTripwire (2 tests)** — the loader file
    contains the `BC-10` marker plus all helper names
    (`iter_reload_classes`, `diff_reload_classes`,
    `evaluate_quiescence`, `QuiescenceState`, `ReloadDiff`);
    the schema file uses the `reload_class` keyword and every
    enum value appears.

**Test-helper note:** the BC10_B tests need to construct a
`BuildConfig` from a mutated raw dict. Round 27's loader doesn't
expose a public "rebuild from raw" entry point (deliberately; the
load path is the only sanctioned construction route). The test
file's private `_rebuild` helper imports the dataclasses + the
private `_validate` and reassembles a `BuildConfig` for the
diff to chew on. `validate=False` is used exclusively for the
single negative-path test that injects a leaf the schema doesn't
declare. This keeps the production loader's invariants intact while
giving the SIL gate enough surface to exercise the diff classifier.

**Deliberate non-changes this round:**

* No CLI. `tools/lifetrac-config push` is Round 29b. Without it
  the contract still has a tester (this SIL gate) and a downstream
  consumer (29b). Splitting keeps both reviewable in isolation.
* No installer daemon. The X8 side that watches for
  `lifetrac-config-*.toml` USB-stick files, validates filename +
  unit-id + embedded SHA, and writes back `lifetrac-config-result.json`
  is Round 29b.
* No web download route. `/config/download` emitting the named
  installer bundle is part of Round 30 (BC-05 web admin form).
* No daemon hot-apply path. `web_ui` and `lora_bridge` don't
  call `evaluate_quiescence` yet — they only audit-log boot
  identity (BC-04). Once 29b lands the installer, the daemons gain
  a watch-and-reload loop that consults both helpers; until then,
  config changes still take a daemon restart and that's fine
  because every operator-facing surface to change them is offline.

**Test growth:** 360 —> 379 (+19). Suite still ~1.30 s.

**Wrap-up artifacts:**

* [TODO.md](TODO.md) status block bumped to "through Round 29"
  and prepended with the BC-10 schema/library narrative.
* [MASTER_TEST_PROGRAM.md](MASTER_TEST_PROGRAM.md) § 1
  totals —> 34 files / 379 tests; § 2 SIL catalog row
  inserted for `test_build_config_delivery_sil.py`; § 5
  Wave-4 IP-traceability gains a BC-10 row pointing at the same
  test file; coverage-gap shortlist re-anchored to "as of Round 29."
* This memo.

**Next up:**

* **Round 29b** — the delivery half of BC-10. Laptop-side
  `tools/lifetrac-config` Python CLI for paths 1 (USB-cable) +
  5 (SSH); X8-side `lifetrac-config-installer` daemon for paths
  2 (base-UI installer bundle on USB-stick) + 3 (hand-authored
  TOML on USB-stick); base-side `/config/download` route emitting
  the named installer bundle; daemon watch-and-reload loop that
  calls `diff_reload_classes` + `evaluate_quiescence` from this
  round. Adds ~15 SIL tests covering filename + unit-id-match
  enforcement, embedded-SHA verification, base-side bundle
  generation, atomic-rename invariant, deferred-reload pickup,
  result-file schema.
* Round 30 = BC-05 (web admin form, thin on top of 29 + 29b).
* Round 31 = BC-03 (firmware codegen + reduced-BOM compile gates).


## Round 29b-alpha — BC-10 delivery (base-side half) (2026-04-29)

**Outcome:** the base-side half of BC-10's delivery surface lands.
An operator can now click a button on the base UI and walk away with
a single self-verifying file on a USB stick, or build the same file
from any laptop with the new `lifetrac-config` CLI. The X8-side
installer that consumes that file is Round 29b-beta. Suite went
379 —> 403 (+24).

**Why split alpha / beta:** the alpha pieces (envelope, CLI, web
download) are all base-side or laptop-side; they're useful the
moment they land (operators can already audit, validate, and
hand-author bundles). The beta pieces (X8 installer daemon, OLED +
LED feedback, daemon watch-and-reload loop) all touch the running
controller and need their own quiescence-and-rollback story. Two
separate PRs review more cleanly than one big one.

**What landed (envelope, [config_bundle.py](DESIGN-CONTROLLER/base_station/config_bundle.py)):**

* Bundle is plain UTF-8 text. Header block: `# bundle_version`,
  `# unit_id`, `# sha256`, `# generator`, `# created`.
  Sentinel: `# --- body ---`. Body: validated TOML byte-for-byte.
* SHA-256 covers the **body bytes only** — no chicken-and-egg
  with the header that carries it; same TOML always produces the
  same SHA regardless of who built the header.
* Filename: `lifetrac-config-<unit_id>-<sha8>.toml`. The
  `unit_id` and the first 8 hex chars of the SHA both appear in
  the filename so a USB stick with several bundles is sortable and
  self-disambiguating without opening anything.
* `verify_filename_matches` cross-checks the filename's
  `unit_id` + `sha8` against the parsed header. A renamed
  bundle is refused — two layers of "is this for this tractor?"
  defence cost essentially nothing.
* Public API: `BUNDLE_VERSION`, `BODY_SENTINEL`, `FILENAME_RE`,
  `Bundle` (frozen dataclass), `BundleError`, `body_sha256`,
  `make_bundle`, `parse`, `parse_filename`, `serialise`,
  `verify_filename_matches`. Pure stdlib; no third-party deps.

**What landed (CLI, [	ools/lifetrac_config.py](tools/lifetrac_config.py)):**

* `validate <path>` — schema-validates a TOML or a bundle.
  Exit 0 on pass, exit 2 on validation failure. Prints `unit_id`
  + `schema_version` + `config_sha256` on success so CI logs
  show the SHA of the artefact that just passed.
* `bundle <toml> -o <dir>` — schema-validates the input,
  writes `<dir>/lifetrac-config-<unit>-<sha8>.toml`. Optional
  `--unit-id` override is allowed only when it agrees with the
  TOML body's `unit_id` (preventing accidental mis-targeting at
  bundle time).
* `verify <bundle>` — re-parses + re-hashes a bundle on
  disk, cross-checks the filename, then schema-validates the body.
  This is the post-USB-stick-copy sanity check.
* `diff <old> --against <new>` — shows which leaves changed
  and what reload class each requires, finishing with
  `--> reload required: <worst-class>`. Built on the BC-10
  `diff_reload_classes` helper from Round 29.
* Exit codes: 0 ok, 2 user-facing failure, 1 reserved for
  unexpected exceptions — CI gates can distinguish "config
  bad" from "tool bad."

**What landed (base web UI, [web_ui.py](DESIGN-CONTROLLER/base_station/web_ui.py)):**

* `GET /config/download` — PIN-gated (same `_require_session`
  dep as `/api/camera/select`). Reads the active `BUILD`'s
  source TOML, builds a bundle via `config_bundle.make_bundle`,
  serialises it, returns `PlainTextResponse` with
  `Content-Disposition: attachment; filename="<bundle.filename>"`
  and `Cache-Control: no-store`.
* Emits a `config_download` audit-log entry carrying `unit_id`
  and the full `config_sha256` so post-mortem can reconstruct
  who downloaded what.
* Returns 503 if `BUILD is None` (dev-checkout fallback) — we
  refuse to serve a half-loaded config rather than emit a bundle
  the X8 installer would just reject.

**What landed (SIL gate):**

* [ase_station/tests/test_build_config_installer_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_installer_sil.py)
  — 24 tests across 4 classes, all green.
  * **BC10b_A_BundleEnvelope (8 tests)** — round-trip preserves
    every field; filename matches the documented pattern;
    body-tamper breaks the SHA check; unsupported
    `bundle_version` is refused; bad `unit_id` pattern is
    refused at construction time; filename `unit_id` mismatch
    refused; filename SHA mismatch refused; missing sentinel
    line refused.
  * **BC10b_B_CLI (8 tests)** — `validate` passes the
    canonical TOML and rejects an out-of-range one with exit 2;
    `bundle` writes a file whose embedded SHA matches the body
    bytes and whose filename agrees with the header; `verify`
    accepts bundler output; `verify` rejects a body-tampered
    bundle (exit 2); `diff` reports identical configs as
    no-diff; `diff` classifies a `ui.max_control_subscribers`
    change as `live`; `diff` classifies a `comm.lora_region`
    change as `restart_required`.
  * **BC10b_C_DownloadRoute (3 tests)** — unauthenticated
    GET returns 401; authenticated GET returns 200 with a
    parseable bundle and the documented Content-Disposition
    filename; the route emits a `config_download` audit entry
    carrying `unit_id` and a 64-char SHA.
  * **BC10b_D_SourceTripwire (4 tests)** — CLI script exists;
    contains all four `cmd_*` handlers + `BC-10` marker;
    `web_ui.py` carries `"/config/download"` + `config_download`
    + `config_bundle` + `BC-10`; `config_bundle.py` exports
    the documented surface.

**Implementation note:** the `test_validate_out_of_range_rejected`
and `test_diff_*` cases rely on string substitution against the
canonical `build.default.toml`. Each substitution now asserts
`body != original` first — a silent no-op replace would have
made the test pass against an unchanged file (it bit us once during
authoring; the assertion is the tripwire that caught it on the
re-run).

**Deliberate non-changes this round:**

* No X8-side installer daemon. No USB-stick auto-discovery, no
  atomic-rename apply path, no `lifetrac-config-result.json`
  write-back. All Round 29b-beta.
* No OLED + LED feedback. The status-line / flash-pattern table
  is pinned in 29b-beta when the installer that drives them lands.
* No `push` subcommand on the CLI. Paths 1 (USB-cable g_ether)
  and 5 (SSH) are 29b-beta because they need the X8 installer on
  the receiving end.
* No daemon watch-and-reload loop in `web_ui` / `lora_bridge`.
  Configs still take a daemon restart to apply — which is fine
  because there's no operator-facing surface to change them between
  restarts yet either. 29b-beta wires both ends in a single PR.
* No web admin form. `/config` (BC-05) is Round 30; `/config/download`
  is the only new web surface this round.

**Test growth:** 379 —> 403 (+24). Suite ~2.3 s.

**Wrap-up artifacts:**

* [TODO.md](TODO.md) status block bumped to "through Round
  29b-alpha" and prepended with the delivery-half narrative.
* [MASTER_TEST_PROGRAM.md](MASTER_TEST_PROGRAM.md) § 1
  totals —> 35 files / 403 tests; § 2 SIL catalog row
  inserted for `test_build_config_installer_sil.py`; § 5
  Wave-4 IP-traceability BC-10 row updated to point at both SIL
  gates and to enumerate the new alpha deliverables;
  coverage-gap shortlist re-anchored to "as of Round 29b-alpha."
* This memo.

**Next up:**

* **Round 29b-beta** — the X8-side half. `lifetrac-config-installer`
  daemon (USB-stick mount watcher + filename + `unit_id` +
  embedded-SHA verify + atomic-rename apply +
  `lifetrac-config-result.json` write-back); OLED status line
  + LED flash convention (3 long = applied, 3 short = rejected,
  alternating = deferred); daemon watch-and-reload loop in
  `web_ui` + `lora_bridge` calling `diff_reload_classes` +
  `evaluate_quiescence` (Round 29 helpers); `push` CLI
  subcommand for paths 1 (USB-cable g_ether) + 5 (SSH). New
  `test_build_config_installer_daemon_sil.py` (~12 tests).
* Round 30 = BC-05 (web admin form, thin on top of 29 + 29b).
* Round 31 = BC-03 (firmware codegen + reduced-BOM compile gates).


## Round 29b-beta — BC-10 delivery (X8-side half) (2026-04-29)

**Outcome:** the tractor-side half of BC-10's delivery surface lands;
the loop opened by 29b-alpha (operator can build + download a bundle)
is closed (X8 installer verifies + atomically applies + writes a
result file the operator can read off the stick; daemons watch +
reload + classify + gate on quiescence). 403 —> 433 (+30).

**Why this round:** 29b-alpha was a producer (build + download a
bundle, but tractor still ignored it). 29b-beta is the consumer +
the operator-feedback contract + the daemon hot-reload story. With
both halves landed, BC-10's hot-reload contract is end-to-end.

**What landed (X8 installer, [installer_daemon.py](DESIGN-CONTROLLER/base_station/installer_daemon.py)):**

* `apply_bundle(bundle_path, *, target_path, unit_id, current_cfg=None)`
  — verifies envelope + filename + body schema; cross-checks the
  bundle's `unit_id` against this X8's identity at three layers
  (filename, header, body); classifies the diff against
  `current_cfg` via Round 29's `diff_reload_classes`; rejects
  `firmware_required` outright (re-flashes are a bench operation);
  for live + restart_required diffs writes the new TOML via
  `os.replace` so a partial write can never leave a half-applied
  config behind.
* Status tokens: `applied` / `rejected` / `noop` /
  `deferred`. Stable; the feedback table + log queries + result
  files all key off these strings.
* `InstallResult` dataclass carries `status`, `reason`,
  `unit_id`, `applied_sha`, `prior_sha`, `reload_class`,
  `applied_at` (RFC 3339 UTC), `schema_version`,
  `result_version` (currently 1; bumped if the JSON shape grows).
* `write_result_file(result, dst_dir)` — drops
  `lifetrac-config-result.json` next to the bundle on the USB
  stick. Atomic via temp + replace, so a yanked stick can't leave a
  truncated file behind.
* `discover_bundles(mount_dir, unit_id)` — globs candidates,
  filters foreign filenames + bundles for other units. An empty
  match is **not** an error (that's "stick has no bundle for me",
  the normal idle case).
* `process_mount` — convenience polling-loop body: discover
  + apply each bundle + write result file. The X8 systemd unit (a
  bench operation, deferred) is a 20-line wrapper that calls this.

**What landed (operator feedback, [eedback.py](DESIGN-CONTROLLER/base_station/feedback.py)):**

* OLED budget: 21 chars. `OLED_LINE_MAX` constant + truncation
  with ellipsis when over budget.
* Pinned LED patterns (steps + repeat flag locked verbatim in the
  SIL gate):
  * `applied`  : 3 long green flashes (700/300) then steady green
    for 5 s.
  * `rejected` : 3 short red flashes (150/150) then steady red
    for 5 s.
  * `deferred` : amber slow blink (500/500), repeats.
  * `noop`     : single short green pulse (200 ms).
  * `idle`     : LED off, repeats.
* OLED prefix per status (`cfg ok` / `cfg REJECTED` /
  `cfg same` / `cfg deferred`) + sha8 suffix when known. The
  operator standing next to the tractor reads the prefix at a
  glance and the sha8 to correlate with the result.json on the
  stick.
* `feedback_for(InstallResult)` + `feedback_for_status(str, *,
  sha8=None)`. The watcher loop uses the latter to surface a
  `deferred` banner before any install attempt.

**What landed (watch-and-reload, [config_watcher.py](DESIGN-CONTROLLER/base_station/config_watcher.py)):**

* `ConfigWatcher` polls `cfg.source_path` (mtime + size
  fingerprint), reloads on change, runs `diff_reload_classes` +
  `evaluate_quiescence`, and emits one of six `WatchEvent`
  kinds. Caller does no decision-making of its own; the watcher
  swaps in the new `BuildConfig` only when `EVENT_APPLIED`
  fires.
* Event vocabulary: `noop` / `applied` / `deferred` /
  `restart_pending` / `firmware_required` / `rejected`.
* `restart_pending` is sticky — once a restart-required
  change is on disk the flag stays set across subsequent polls so
  the UI banner doesn't flicker. Cleared only when the daemon
  restarts (and that's the whole point).
* No I/O, no threading; the caller drives a simple
  `while True: poll(); sleep(N)` loop. That keeps the SIL gate
  purely synchronous.

**What landed (web UI integration, [web_ui.py](DESIGN-CONTROLLER/base_station/web_ui.py)):**

* `GET /api/build_config/state` — PIN-gated. Polls the
  module-level `ConfigWatcher` and returns
  `{unit_id, schema_version, sha256, source_path,
  restart_pending, last_event, last_event_reason, changed_leaves,
  worst_reload_class}`. Emits a `config_watch_event` audit
  entry on every non-noop poll so post-mortem can reconstruct
  which reload happened when.
* Base-side `ConfigWatcher` quiescence is trivially-true (base
  UI runs no hydraulics) so live changes apply on the next poll.
  Tractor-side `lora_bridge` will pass a real quiescence
  callback reading M4/M7 telemetry; that wire-up lands in the
  next round (it touches the lora_bridge main loop, kept out of
  29b-beta to keep the diff reviewable).

**What landed (CLI, [	ools/lifetrac_config.py](tools/lifetrac_config.py)):**

* `push <bundle> --via local --dest <dir>` — copies the
  bundle into `<dir>` (typically a USB-stick mount the X8 also
  watches, or the local inbox path). Exit 0 on success.
* `push ... --apply --target <active.toml>` — after copying,
  invokes `installer_daemon.process_mount` directly and prints
  per-bundle `status` + `unit_id` + `sha8` + `reload_class`
  + `reason`. This is the SIL path used by CI.
* `push ... --via ssh --host <h> [--user <u>] [--dest <d>]` —
  builds and prints the planned scp + systemd-trigger commands.
  Default is dry-run; `--execute` actually shells out (kept
  opt-in because shelling out from CI is a bench operation, not
  a unit-test one).

**What landed (SIL gate):**

* [ase_station/tests/test_build_config_installer_daemon_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_installer_daemon_sil.py)
  — 30 tests across 4 classes, all green.
  * **BC10c_A_DaemonApply (8 tests)** — fresh apply writes the
    target TOML and returns `applied` with the right SHA; same
    SHA returns `noop` without touching the target; filename
    `unit_id` mismatch rejected; body `unit_id` mismatch
    rejected even when filename + header agree; schema violation
    rejected; `firmware_required` diff rejected with
    `reload_class == "firmware_required"` and target untouched;
    `discover_bundles` filters by unit_id and ignores foreign
    files; `write_result_file` round-trips JSON with the
    documented field set.
  * **BC10c_B_FeedbackContract (7 tests)** — `STATUS_TABLE`
    covers every install status; `LED_APPLIED` /
    `LED_REJECTED` step tuples pinned verbatim; `LED_DEFERRED`
    repeats; OLED line stays under 21 chars for every status;
    `feedback_for` raises on unknown status; `feedback_for_status`
    helper produces the right LED + sha8-suffixed line.
  * **BC10c_C_WatchReload (7 tests)** — `poll` returns
    `noop` when nothing changed; `applied` when a live change
    is on disk and quiescence holds (cfg actually swapped);
    `deferred` when quiescence does not hold (cfg **not**
    swapped); `restart_pending` for restart_required diffs (cfg
    not swapped, flag set, flag survives subsequent `noop`
    polls); `rejected` when the on-disk file fails schema;
    `/api/build_config/state` returns 401 without session;
    `/api/build_config/state` with session returns the
    documented JSON envelope.
  * **BC10c_D_PushAndTripwires (8 tests)** — `push --via
    local` copies the bundle and exits 0; `push --via local
    --apply --target` invokes the daemon, prints `applied`,
    writes the target, drops `lifetrac-config-result.json` on
    the dest dir; `push --via ssh` without `--execute` only
    prints the plan; module-export tripwires for
    `installer_daemon` (12 names), `feedback` (10 names),
    `config_watcher` (9 names); `web_ui.py` carries
    `/api/build_config/state` + `config_watch_event` +
    `Round 29b-beta`; CLI carries `cmd_push` + the four push
    flags + `Round 29b-beta`.

**Implementation note:** same alignment-bug class as 29b-alpha
caught three of the SIL fixtures on first run (`track_axis_count`
in the canonical TOML uses 4-space alignment, `unit_id` uses
8-space alignment). Each `.replace()` substitution now asserts
`new != original` first — the assertion is the tripwire that
caught it on the re-run. Pattern is now uniformly applied across
both 29b SIL gates.

**Deliberate non-changes this round:**

* No production X8 systemd unit + udev rule. The module function
  `installer_daemon.process_mount` is the polling-loop body; the
  20-line shell wrapper around it that udev / systemd invokes is
  a bench operation queued for the next on-tractor checkout. SIL
  has no udev to mock.
* No real LED + OLED driver. `feedback.LedPattern` is hardware-
  agnostic on purpose (`green` / `red` / `amber` / `off`
  + milliseconds). The HAL shim that translates to GPIO writes +
  the SSD1306 driver call is bench work.
* No tractor-side `lora_bridge` integration of `ConfigWatcher`.
  The watcher is wired into `web_ui` (which proves the API
  works) but `lora_bridge` still re-loads at restart. Kept out
  of this round because it touches the main control loop and
  needs a real quiescence callback reading M4/M7 telemetry; lands
  in the next round.
* No web admin form for editing the config in-browser — that's
  BC-05 (Round 30), thin on top of 29 + 29b.

**Test growth:** 403 —> 433 (+30). Suite ~3.2 s.

**Wrap-up artifacts:**

* [TODO.md](TODO.md) status block bumped to "through Round
  29b-beta" and prepended with the consumer-side narrative.
* [MASTER_TEST_PROGRAM.md](MASTER_TEST_PROGRAM.md) § 1
  totals —> 36 files / 433 tests; § 2 catalog row
  inserted for `test_build_config_installer_daemon_sil.py` (30
  tests); § 5 BC-10 IP-traceability row updated to enumerate
  the new beta deliverables and to point at all three SIL gates;
  coverage-gap shortlist re-anchored to "as of Round 29b-beta."
* This memo.

**Next up:**

* **Round 30 = BC-05 web admin form.** Now that delivery is end-to-
  end, BC-05 is a thin React-free HTML form on top of the same
  TOML the watcher serves. Operator edits in-browser, POSTs to a
  new `/config/upload` route (PIN-gated), which schema-validates,
  saves to the canonical path, surfaces the would-be reload class
  via the same `diff_reload_classes` helper, and lets the
  watcher pick it up on next poll.
* **Round 31 = BC-03 firmware codegen + reduced-BOM compile gates.**
  Generate `build_config.h` / `build_config_pins.h` from the
  same schema; add `arduino-cli compile` jobs that exercise the
  M4/M7 firmware against representative BOMs.
* **Round 32+ = BC-06 / BC-07 / BC-08 / BC-09 backlog.**


## Round 30 — BC-05 web admin form (2026-04-29)

**IP:** BC-05 (operator-facing build-config admin form on the base UI).

**Code landed:**
- `DESIGN-CONTROLLER/base_station/web/config.html` — new page; thin client over four sibling routes (state / source / preview-diff / upload) with editor textarea, Preview Diff button, Save button, watcher pill, and SHA / unit_id / source-path metadata strip.
- `DESIGN-CONTROLLER/base_station/web_ui.py` — adds `GET /config` (PIN-gated HTML), `GET /api/build_config/source` (verbatim TOML, `Cache-Control: no-store`), `POST /api/build_config/preview-diff` (no-side-effects schema validation + `diff_reload_classes` classification), `POST /api/build_config/upload` (schema-validate — refuse `firmware_required` — atomic write via `installer_daemon._atomic_write` — audit `config_upload`). Body validation re-uses the BC-10 loader path (write to NamedTemporaryFile, env-override `LIFETRAC_BUILD_CONFIG_PATH`, call `build_config.load`) for byte-identity with the X8 installer.

**Contract details:**
- `firmware_required` diffs are rejected at the upload endpoint (re-flash via bench), matching the X8 installer's policy. `restart_required` diffs are accepted — they land on disk and the watcher then sets the sticky `restart_pending` flag on its next poll, surfaced through `/api/build_config/state` (and the page banner).
- `unit_id` mismatches are rejected at the upload endpoint with an explicit error string. The candidate body is schema-validated via `build_config.load` before any disk write so a malformed body cannot leave a partial file behind.
- All four endpoints are PIN-gated via the existing `_require_session` dependency.

**Tests landed:** `DESIGN-CONTROLLER/base_station/tests/test_build_config_admin_sil.py` (16 tests across 5 classes):
- `BC05_A_PageRouting` (2): unauth — 303 to /login; authed — widget IDs present.
- `BC05_B_SourceEndpoint` (2): 401 without session; verbatim body + `no-store` header.
- `BC05_C_PreviewDiff` (4): identical — no leaves; live change — `live`; restart-required change — `restart_required`; schema violation — `ok:false` + target untouched.
- `BC05_D_Upload` (6): identical no-op; live atomic-write rewrites target byte-for-byte; schema violation rejected (target untouched); unit_id mismatch rejected; firmware-required rejected (target untouched); 401 without session.
- `BC05_E_SourceTripwire` (2): web_ui carries the four routes + `config_upload` + `BC-05` marker; HTML references the four sibling routes + `/config/download`.

**Wrap-up artifacts:**
- `TODO.md` — status block bumped to "through Round 30" + Round 30 narrative prepended.
- `MASTER_TEST_PROGRAM.md` — Last-updated — Round 30;  totals 433 — 449 / 36 — 37 files;  catalog row added;  BC-05 row added; coverage-gap re-anchor.

**Suite:** 449 tests / 1 skipped / 37 files (433 — 449 from this round).

**Next:** BC-03 (firmware codegen — emit a compile-time C header from the canonical TOML so the M4/M7 firmware can drop preprocessor branches against the same single source of truth) + reduced-BOM compile gates is the natural Round 31 candidate.

## Round 31 — BC-03 firmware codegen (2026-04-29)

**IP:** BC-03 (canonical C header for the firmware build, single source of truth for LIFETRAC_* #defines).

**Code landed:**
- `DESIGN-CONTROLLER/base_station/build_config_codegen.py` — new module; `emit_header(cfg)` walks the validated `BuildConfig` and produces a deterministic C header (ASCII, LF-terminated, no trailing whitespace, no timestamps in the body). Type-safe formatting: `int` — bare literal, `bool` — `1`/`0`, `float` — `2.0f`-suffixed (avoids gcc -Wdouble-promotion), `str` — double-quoted. Enum-typed strings get an extra `LIFETRAC_<SECTION>_<NAME>_<VALUE>` side macro (active=1, others=0) so sketches can `#if` branch against the canonical truth. `write_header(cfg, dest)` uses the same tempfile + `os.replace` atomic pattern as `installer_daemon._atomic_write` so a partial write can never leave a half-rewritten header.
- `tools/lifetrac_config.py` — adds `codegen` subcommand: `lifetrac-config codegen <toml> --out <header>` writes; `--check` mode reads the on-disk file and exits non-zero on drift (CI gate). Module docstring updated for Round 31.
- `DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h` — new generated file (74 lines, ~30 macros across hydraulic / safety / cameras / sensors / comm / ui / net + identity macros + legacy aliases). `LIFETRAC_M4_WATCHDOG_MS` is preserved as a legacy alias of `LIFETRAC_SAFETY_M4_WATCHDOG_MS` so the existing firmware can `#include` the new header without a flag-day rename of every callsite (in `shared_mem.h` / `tractor_m4.cpp` / `tractor_h7_m4.ino`).

**Contract details:**
- The header carries `LIFETRAC_UNIT_ID` + `LIFETRAC_SCHEMA_VERSION` + `LIFETRAC_CONFIG_SHA256_HEX` (and `_HEX_SHORT`) so firmware can publish a heartbeat with the SHA the X8 booted with — the base `config_loaded` audit entry then cross-checks the firmware reported SHA against its own.
- `Source:` line in the header carries the TOML basename only (no directory prefix) so emissions are byte-identical across working directories: a CLI run from the repo root and a Python call from anywhere produce the same bytes.
- `--check` exit codes match the BC-10 CLI convention: 0 = clean, 2 = user-facing failure (stale or missing).

**Tests landed:** `DESIGN-CONTROLLER/base_station/tests/test_build_config_codegen_sil.py` (22 tests across 7 classes):
- `BC03_A_HeaderShape` (4): guard / identity macros / ASCII+LF+trailing newline / no trailing whitespace per line.
- `BC03_B_LeafParity` (1): every JSON-Schema scalar leaf appears as a `#define` — a future schema addition that the codegen forgets fails this gate.
- `BC03_C_TypeFormatting` (6): int bare; bool 1/0; float decimal+`f`; string double-quoted; enum side-macros active=1 others=0; enum value-string macro present.
- `BC03_D_Determinism` (2): two emissions byte-identical; on-disk firmware/common/lifetrac_build_config.h matches a fresh emission against build.default.toml (CI gate).
- `BC03_E_LegacyAliases` (1): every alias emitted; alias target defined earlier in the file (preprocessor-resolvable).
- `BC03_F_CliCodegen` (4): `--out` byte-identical to direct `emit_header`; `--check` clean / drift / canonical-firmware-header.
- `BC03_G_SourceTripwire` (4): codegen module exports + Round 31 / BC-03 marker; CLI `cmd_codegen` + `--check` + BC-03 marker; on-disk header carries `AUTOGENERATED` + `Round 31` + `BC-03` + `DO NOT EDIT`.

**Wrap-up artifacts:**
- `TODO.md` — status block bumped to "through Round 31" + Round 31 narrative prepended.
- `MASTER_TEST_PROGRAM.md` — Last-updated — Round 31;  totals 449 — 471 / 37 — 38 files;  catalog row added;  BC-03 row added; coverage-gap re-anchor.

**Suite:** 471 tests / 1 skipped / 38 files (449 — 471 from this round).

**Next:** the M4/M7 firmware sketches still `#define LIFETRAC_M4_WATCHDOG_MS 200u` directly in `shared_mem.h` (3 copies). Migrating those sketches to `#include "lifetrac_build_config.h"` and dropping the local `#define`s is a follow-on task that needs the Arduino compile gates to verify — queue as Round 32 candidate alongside BC-06 / BC-07 / BC-08 / BC-09.

## Round 32 — BC-07 variant-matrix SIL (2026-04-29)

**IP:** BC-07 (variant-matrix SIL exercising 4 representative fleet shapes end-to-end through loader / web_ui / codegen / BC-10 diff).

**Code landed:**
- `DESIGN-CONTROLLER/base_station/tests/test_build_config_variant_matrix_sil.py` — new SIL gate; four variants synthesised inline from the canonical default TOML via tracked find/replace pairs (each pair guarded by a no-op tripwire). Variants: `canonical` (baseline byte-identical to default), `no_camera` (count=0 + every present=false), `no_imu_gps` (imu_present=false + gps_present=false + hyd_pressure_sensor_count=0), `single_axis` (track_axis=1 + arm_axis=0 + proportional_flow=false + track_ramp=0.5).

**Contract details:**
- Each variant gets a per-test tempdir so the four fixtures are independent; `LIFETRAC_BUILD_CONFIG_PATH` pinning per variant.
- `no_imu_gps` keeps the model strings on purpose (re-enabling later just flips `present` back to true; the schema requires the model field regardless of presence so this matches the loader contract).
- `single_axis` exercises the schema minimum on `track_axis_count` (1) and `arm_axis_count` (0) so a future schema tightening that breaks the Microtrac shape gets caught.
- The four `config_sha256` values must be pairwise distinct so a single `config_loaded` audit-log line carrying the SHA disambiguates the fleet shape that booted.

**Tests landed:** 14 tests across 5 classes:
- `BC07_A_LoaderAcceptsEveryVariant` (4): no-camera / no-imu-gps / single-axis loader round-trip + canonical fixture byte-identical to the on-disk default.
- `BC07_B_WebUiConsumesVariant` (3): no-camera collapses `_CAMERA_IDS` (no front/rear/implement/crop entries); canonical keeps the front entry only; single-axis keeps `MAX_CONTROL_SUBSCRIBERS` reading from `BUILD.ui` (catches a hard-coded constant).
- `BC07_C_CodegenForEveryVariant` (3): no-camera header zeros `LIFETRAC_CAMERAS_*_PRESENT`; no-imu-gps zeros `LIFETRAC_SENSORS_*_PRESENT` but persists `LIFETRAC_SENSORS_IMU_MODEL "bno086"`; single-axis emits `LIFETRAC_HYDRAULIC_TRACK_AXIS_COUNT 1` + `LIFETRAC_HYDRAULIC_TRACK_RAMP_SECONDS 0.5f`.
- `BC07_D_ReloadClassDiffPerVariant` (3): canonical self-diff is empty; no-camera diff stays inside `cameras.*` and bubbles to `live` + `firmware_required = False`; single-axis diff bubbles to `restart_required` (axis counts carry that reload class per BC-10).
- `BC07_E_DistinctShasAcrossVariants` (1): all four variant SHAs pairwise distinct + canonical SHA equals a fresh load of `build.default.toml`.

**Wrap-up artifacts:**
- `TODO.md` — status block bumped to "through Round 32" + Round 32 narrative prepended.
- `MASTER_TEST_PROGRAM.md` — Last-updated — Round 32;  totals 471 — 485 / 38 — 39 files;  catalog row added;  BC-07 row added; coverage-gap re-anchor.

**Suite:** 485 tests / 1 skipped / 39 files (471 — 485 from this round).

**Remaining BC backlog:** BC-06 (extend hil/dispatch.ps1 to read the active config and flag inapplicable gates as N/A; pure-PowerShell + JSON, doable without bench hardware), BC-08 (onboarding doc DESIGN-CONTROLLER/BUILD_CONFIG.md), BC-09 (HARDWARE_BOM.md — capability ID cross-reference + orphan SIL gate). BC-06 and BC-09 are the next high-confidence pure-Python/PowerShell rounds; BC-08 is a doc round so easily packaged alongside either.

## Round 33 — BC-11 first-class auxiliary attachment ports (2026-04-29)

**IP:** BC-11 (new schema section `[aux]` for auxiliary hydraulic attachment ports; user-driven design question about how the tractor knows its own hardware config surfaced this gap \u2014 today aux ports are conflated into `arm_axis_count`).

**Code landed:**
- `DESIGN-CONTROLLER/base_station/config/build_config.schema.json` — `aux` added to top-level `required`; new `aux` object with `port_count` (int 0..2, restart_required), `coupler_type` (enum `iso_5675` / `flat_face` / `none`, restart_required), `case_drain_present` (bool, restart_required). All three are restart_required because aux ports are a hardware capability \u2014 changing any of them requires the M4 to re-init the PWM driver tables and the attachment-permit gate, same precedent as `track_axis_count`.
- `DESIGN-CONTROLLER/base_station/config/build.default.toml` — `[aux]` block added (placed adjacent to `[hydraulic]` so the existing BC_D rearrange-and-rehash test that anchored on `[net]` being last still holds). Default is the conservative shape: `port_count = 0`, `coupler_type = "none"`, `case_drain_present = false` so existing v25 builds without aux plumbing keep getting the safe answer.
- `DESIGN-CONTROLLER/base_station/build_config.py` — new `AuxConfig` dataclass; `BuildConfig.aux` field; `load()` populates from `data["aux"]`.
- `DESIGN-CONTROLLER/base_station/build_config_codegen.py` — `"aux"` appended to `_SECTIONS` so the codegen walks it (everything else is automatic \u2014 leaf macros, enum side-macros, header guard).
- `DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h` — **regenerated**. New SHA `f961d9fd924b52ec46074c6cd26e6d1efcac035f8993e676f6003beb84b1c280` (was `63a15fcb...` before this round). New macros: `LIFETRAC_AUX_PORT_COUNT 0`, `LIFETRAC_AUX_COUPLER_TYPE "none"`, `LIFETRAC_AUX_COUPLER_TYPE_NONE 1`, `LIFETRAC_AUX_COUPLER_TYPE_ISO_5675 0`, `LIFETRAC_AUX_COUPLER_TYPE_FLAT_FACE 0`, `LIFETRAC_AUX_CASE_DRAIN_PRESENT 0`. M4 firmware can now use `#if LIFETRAC_AUX_PORT_COUNT > 0` to conditionally compile aux-PWM init.
- `DESIGN-CONTROLLER/CAPABILITY_INVENTORY.md` — new "Auxiliary attachment ports (Round 33 / BC-11)" section so the schema-vs-doc parity gate (`BC_E_InventoryParity.test_every_schema_property_has_inventory_row`) stays green; also added `"aux"` to the inventory-scraper's section tuple.
- `DESIGN-CONTROLLER/base_station/tests/test_build_config_loader_sil.py` — `_inventory_ids` sections tuple includes `"aux"`.
- `DESIGN-CONTROLLER/base_station/tests/test_build_config_delivery_sil.py` — `_rebuild` helper imports + populates `AuxConfig` so the BC10 hot-reload tests can construct a synthetic BuildConfig with the new field.

**Contract details:**
- Default `port_count = 0` keeps existing single-aux-channel-or-fewer builds backward-compatible \u2014 no per-unit override needed for the canonical Open Source Ecology v25 tractor.
- `coupler_type` enum: `iso_5675` (legacy ag QD), `flat_face` (skid-steer / construction), `none` (no aux ports installed). `none` is the safe answer when `port_count == 0`.
- `case_drain_present` matters for motor-type attachments (auger, post-pounder, mower spindle) which leak internally; cylinder-type attachments (grapple, thumb) don't need it. The M4 attachment-permit gate will (in a future BC) refuse to drive an aux PWM channel when `case_drain_present == false` and the operator has commanded a motor-type attachment.
- Cross-section invariant deferred: not enforcing `coupler_type == "none" iff port_count == 0` in the schema today \u2014 that belongs in a future BC-12 boot-time hardware-vs-config self-test, where the firmware can verify the actual coupler hardware against the declaration.

**Tests landed:** 13 tests across 5 classes:
- `BC11_A_SchemaDeclaresAux` (4): aux in top-level required; section is object with three required leaves; every leaf carries `reload_class = restart_required`; coupler_type enum includes `none`/`iso_5675`/`flat_face`.
- `BC11_B_LoaderExposesAuxConfig` (3): `AuxConfig` has the three expected dataclass fields; canonical default is the conservative shape; loader rejects a TOML missing the aux section (with a strip-aux fixture + tripwire).
- `BC11_C_CodegenEmitsAuxMacros` (3): `aux` is in `_SECTIONS`; every aux leaf emits a macro with the expected literal; coupler_type emits enum side-macros (active=1, others=0).
- `BC11_D_AuxLeafChangesAreRestartRequired` (2): `iter_reload_classes` covers every aux leaf; `diff_reload_classes` against a BuildConfig with all three aux leaves changed returns `restart_required` as worst class.
- `BC11_E_FirmwareHeaderInSyncWithAux` (1): committed on-disk `firmware/common/lifetrac_build_config.h` contains the aux section + macros \u2014 the drift gate that fails when an aux schema edit forgets the regen step.

**Wrap-up artifacts:**
- `TODO.md` — status block bumped to "through Round 33" + Round 33 narrative prepended.
- `MASTER_TEST_PROGRAM.md` — Last-updated — Round 33;  totals 485 — 498 / 39 — 40 files;  catalog row added;  BC-11 row added; coverage-gap re-anchor.

**Suite:** 498 tests / 1 skipped / 40 files (485 — 498 from this round).

**Remaining BC backlog:** BC-06 (extend hil/dispatch.ps1 to read the active config and flag inapplicable gates as N/A), BC-08 (onboarding doc DESIGN-CONTROLLER/BUILD_CONFIG.md), BC-09 (HARDWARE_BOM.md — capability ID cross-reference + orphan SIL gate). New: BC-12 proposed (boot-time config-vs-hardware verification self-test \u2014 SIL half doable today against mocked m4_io, HIL half waits for bench rig).

## Round 34 — BC-06 hil/dispatch.ps1 build-config awareness (2026-04-29)

**IP:** BC-06 (per-build HIL harness gating: dispatch.ps1 reads the active build config and flags inapplicable Wave-4 gates as N/A instead of FAIL/NOT-STARTED, so a build without a handheld doesn't get blamed for the absence of W4-01 PASS rows).

**Code landed:**
- `DESIGN-CONTROLLER/hil/gate_applicability.json` — new rules file. Single source of truth for per-gate predicates. Op vocabulary: `eq` / `gt` / `gte` / `truthy`. Conjunctive-only (each gate's `applies_when` array is AND-ed); empty array means always applicable. Every gate carries a `rationale` string for the next reader. Coverage: W4-01 + W4-02 require `comm.handheld_present == true`; W4-05 requires `hydraulic.proportional_flow == true`; W4-06 requires `hydraulic.arm_axis_count > 0`; W4-08 requires `cameras.count > 0`; W4-03 / W4-04 / W4-07 / W4-09 / W4-10 always apply.
- `DESIGN-CONTROLLER/hil/dispatch.ps1` — new `-ConfigPath` and `-NoApplicability` parameters. New helpers: `Resolve-LifeTracConfigPath`, `Get-LifeTracConfigJson` (shells out to `python tools/lifetrac_config.py dump-json`), `Get-LifeTracGateApplicabilityRules`, `Resolve-DottedPath`, `Test-GateApplicable`. `Get-GateStatus` accepts an `-Applicable` switch and returns `Status = 'N/A'` when false. The recommended-next-gate filter excludes both `CLOSED` and `N/A`. Every failure path warns and falls back to the legacy "everything applies" behavior so a missing python or a malformed TOML never breaks the existing dispatcher.
- `tools/lifetrac_config.py` — new `dump-json` subcommand (`cmd_dump_json`). Returns `{unit_id, schema_version, config_sha256, config}` as canonical JSON (`sort_keys=True, separators=(',', ':')`). Pure stdout; suitable for `ConvertFrom-Json` in PowerShell or `json.loads` anywhere else. Module docstring updated for Round 34.

**Contract details:**
- The applicability evaluator in `dispatch.ps1` is a thin mechanical mirror of the four ops the rules file declares; the SIL gate exercises the rules in pure Python (no `powershell.exe` subprocess in CI), so the rules contract is the integration boundary.
- Conjunctive-only predicates were chosen deliberately: every real-world inapplicability rule we have so far is "feature X is present", which is naturally AND-able. If a gate ever needs OR semantics (e.g. "applies when EITHER cellular OR LoRa is present") add an `any_of` field next to `applies_when` rather than overloading the existing op set.
- `dump-json` payload deliberately includes `config_sha256` so the dispatcher's "Applicability source" header line can disambiguate which fleet shape was used to compute the N/A flags (matches the Round 31 `config_loaded` audit-log identity).
- BOM caveat: `Set-Content -Encoding UTF8` writes a BOM that Python's `tomllib` rejects. Existing TOML files in the repo are LF + no-BOM; harness operators editing a per-unit TOML on the bench laptop should use Notepad++ "UTF-8 without BOM" or `[System.IO.File]::WriteAllText($path, $body, [System.Text.UTF8Encoding]::new($false))`. This was caught during Round 34 smoke-testing; documented here.

**Tests landed:** 13 tests across 7 classes:
- `BC06_A_RulesFileShapeAndCoverage` (4): rules file gate set equals dispatcher's `\` (parity prevents drift); every predicate uses a known op; every dotted path resolves against the canonical default config (catches typos like `hyrdaulic.`); every gate carries a non-trivial rationale string.
- `BC06_B_CanonicalDefaultIsFullyApplicable` (1): every Wave-4 gate applies to the canonical default fleet shape.
- `BC06_C_NoHandheldMarksRadioGatesNA` (3): handheld-off fixture marks W4-01 + W4-02 N/A; W4-03..W4-10 stay applicable.
- `BC06_D_NoCamerasMarksW408NA` (1): zero-cameras fixture marks W4-08 N/A; spot-checks unrelated gates stay applicable.
- `BC06_E_BangBangMarksW405NA` (1): bang-bang fixture marks W4-05 N/A; W4-06 stays applicable (arm axes still present).
- `BC06_F_NoArmAxisMarksW406NA` (1): zero-arm-axis fixture marks W4-06 N/A; W4-05 stays applicable.
- `BC06_G_DumpJsonSubcommandStable` (2): `dump-json` byte-identical across two invocations; payload carries the four documented top-level keys + the eight build_config sections.

**Wrap-up artifacts:**
- `TODO.md` — status block bumped to "through Round 34" + Round 34 narrative prepended.
- `MASTER_TEST_PROGRAM.md` — Last-updated — Round 34;  totals 498 — 511 / 40 — 41 files;  catalog row added;  BC-06 row added; coverage-gap re-anchor.

**Suite:** 511 tests / 1 skipped / 41 files (498 — 511 from this round).

**Remaining BC backlog:** BC-08 (onboarding doc DESIGN-CONTROLLER/BUILD_CONFIG.md), BC-09 (HARDWARE_BOM.md — capability ID cross-reference + orphan SIL gate), proposed BC-12 (boot-time config-vs-hardware verification self-test \u2014 SIL half doable today against mocked m4_io, HIL half waits for bench rig). BC-08 is the highest-confidence next round (pure documentation; ample new content from Round 30/31/32/33/34 to tie together).

## Round 35 — BC-08 BUILD_CONFIG.md operator/integrator onboarding (2026-04-29)

**IP:** BC-08 (single operator-facing onboarding doc that ties every piece of the per-unit build-config system together for the next person who is handed a LifeTrac v25 to bring up; pinned to schema + CLI + reload-class + link-target reality by a SIL drift gate so it cannot quietly diverge).

**Code landed:**
- `DESIGN-CONTROLLER/BUILD_CONFIG.md` — new operator/integrator onboarding doc. Nine sections: (1) what a build-config is + what it explicitly is NOT (no secrets, no runtime settings, no operator preferences \u2014 facts about the hardware); (2) where the files live + override resolution order (env var — per-unit TOML — default); (3) the eight required sections with one-line rationale + common-override-reason for each (hydraulic / safety / cameras / sensors / comm / ui / net / aux); (4) the three reload classes with operator action for each; (5) the seven CLI subcommands as a synopsis table; (6) the deterministic `config_sha256` identity + the four places it appears (bundle filename, audit log, dispatcher header, `dump-json` payload); (7) the `config_loaded` audit trail; (8) four common pitfalls (UTF-8 BOM, section ordering for tests, four-edit checklist for new schema leaves, `firmware_required` — reflash); (9) where-to-read-next pointer table.

- `DESIGN-CONTROLLER/base_station/tests/test_build_config_doc_sil.py` — new 5-test SIL drift gate, 5 classes BC08_A..BC08_E:
  * `BC08_A_DocCoversEverySchemaSection` (1): every required object section in the schema is named in the doc (backticked or linked). Catches a doc that forgets to mention a newly added section.
  * `BC08_B_DocCoversEveryCLISubcommand` (1): every `sub.add_parser(...)` registered subcommand is named in the doc, AND no orphan subcommand-shaped tokens appear in the synopsis table (flag tokens like `--check` are filtered out by the leading-dash rule). Bidirectional parity.
  * `BC08_C_DocCoversEveryReloadClass` (1): the schema's `reload_class` vocabulary is exactly `{live, restart_required, firmware_required}` AND every value is named in the doc. Catches a fourth class added without a doc edit.
  * `BC08_D_DocLinksResolve` (1): every relative-path Markdown link in the doc resolves to an existing file. Catches stale links from rounds that move files.
  * `BC08_E_DocNamesEveryEstopTopology` (1): every `estop_topology` enum value in the schema appears verbatim in the doc \u2014 representative drift gate covering a safety-critical enum the doc explicitly enumerates.

**Contract details:**
- The drift gate is mechanical — it does not parse the doc semantically, only checks for backtick / link presence of names that already exist as ground truth elsewhere (schema, CLI module). This means the doc author still has to write *correct* prose, but cannot quietly forget to mention a section, subcommand, or reload class.
- BC08_E is deliberately a *spot* check, not a *general* enum gate. Generalising it to every enum in the schema would force the doc to enumerate values it doesn't logically need to (e.g. every IMU model). The pattern is: if the doc explicitly mentions an enum's set of values in prose, add a BC08-E-style gate for that enum specifically. Today only `estop_topology` qualifies.
- Hit one tooling lesson during build: the BC08_B reverse-orphan check originally tripped on `--check` because the synopsis cell for `codegen` mentions the flag. Fixed by filtering tokens with a leading dash from the synopsis-token set — flags are not subcommands by definition.

**Tests landed:** 5 tests across 5 classes (full breakdown above).

**Wrap-up artifacts:**
- `TODO.md` — status block bumped to "through Round 35" + Round 35 narrative prepended.
- `MASTER_TEST_PROGRAM.md` — Last-updated — Round 35;  totals 511 — 516 / 41 — 42 files;  catalog row added;  BC-08 row added; coverage-gap re-anchor.

**Suite:** 516 tests / 1 skipped / 42 files (511 — 516 from this round).

**Remaining BC backlog:**
- **BC-09** (HARDWARE_BOM.md — capability ID cross-reference + orphan SIL gate): walk every line item in HARDWARE_BOM.md and pin which BC-* capability ID it provides; SIL gate flags any HARDWARE_BOM line that has no capability-ID anchor or any capability ID without a HARDWARE_BOM anchor. Pure documentation + test, no bench hardware. Highest-confidence next round.
- **BC-12** (proposed — boot-time config-vs-hardware verification self-test): SIL half is doable today against mocked m4_io; HIL half waits for bench rig. Lower priority than BC-09 because it requires firmware-side work too.
- **BC-13** (proposed — CYBERSECURITY_CASE.md cross-reference for build-config attack surface): doc-only, low priority.

## Round 36 — BC-09 HARDWARE_BOM.md capability cross-reference (2026-04-29)

**IP:** BC-09 (every physical-shape build-config leaf is now anchored to the specific BOM section + part(s) that physically realise it; bidirectional SIL drift gate prevents the BOM and the schema from quietly diverging on which capabilities have hardware backing).

**Code landed:**
- `DESIGN-CONTROLLER/HARDWARE_BOM.md` — new `## Capability cross-reference (Round 36 / BC-09)` appendix. 15-row table mapping every physical-shape build-config leaf to its BOM evidence (Tier number + bolded part name(s)) + per-row notes covering `false` / non-canonical enum behaviour. Includes a `How to keep this in sync` paragraph that documents the new five-place-edit rule (schema — default TOML — loader dataclass — codegen `_SECTIONS` — this table). The three Round-33 `aux.*` leaves are present with `planned` markers because no current BOM line exists for aux PWM coil drivers / couplers yet.

- `DESIGN-CONTROLLER/base_station/tests/test_hardware_bom_xref_sil.py` — new 6-test SIL drift gate, 5 classes BC09_A..BC09_E:
  * `BC09_A_NoOrphanIDsInAppendix` (1): every capability id in the appendix table is a real schema property. Catches typos and stale rows for renamed capabilities.
  * `BC09_B_EveryPhysicalCapabilityHasBOMRow` (2): every entry in the curated `_PHYSICAL_CAPABILITIES` set appears in the appendix; AND every entry in the curated set itself is a real schema property. Bidirectional. The curated set is the test file's source of truth for "this capability needs hardware" — adding a new physical capability is a single-line edit there plus the matching appendix row.
  * `BC09_C_RowEvidenceIsAnchored` (1): every row's evidence cell carries a `Tier 1` / `Tier 2` / `Tier 3` token OR a `planned` marker. Catches a row that points at no part section AND distinguishes deliberately-unrealised capabilities (aux today) from accidentally-empty cells.
  * `BC09_D_AppendixLinksResolve` (1): every relative link in the appendix resolves on disk.
  * `BC09_E_AppendixIsDiscoverable` (1): the appendix heading is present AND self-references its enforcing test file by filename so the next reader can find the contract that pins it.

**Contract details:**
- The curated `_PHYSICAL_CAPABILITIES` set is the test file's deliberate source of truth, not derived from the schema. Reasoning: the schema has 30+ leaves but only 15 of them have a 1:1 hardware part (the rest are pure software thresholds). Auto-deriving "which leaves are physical" from the schema would either be wrong (forcing pointless rows for `track_ramp_seconds`) or require an annotation in the schema itself. The curated approach is one-line-per-capability and self-documenting.
- BC09_C accepts `planned` as a valid marker so an unrealised capability (aux today) can be documented honestly without forcing a fake BOM row. When aux hardware lands the row gets a `Tier 1` token and the `planned` marker is dropped.
- Tooling lesson during build: PowerShell here-strings consume backticks as escape characters even in double-quoted `@"..."@` form. The first attempt at appending the appendix via PowerShell stripped every backtick from the prose (turning `\unit_id\` into `unit_id` and `\
et.*\` into a literal newline plus `net.*\`). Workaround: write the appendix via a tiny scratch `_append_xref.py` Python script using a triple-quoted string, then delete the script. The `test_xref_sil.py` BC09_C row check would have caught the backtick-strip eventually because the surviving prose still parsed, but the row-regex parity check (BC09_A) caught it immediately because the row capability-id cells lost their backticks.

**Tests landed:** 6 tests across 5 classes (full breakdown above).

**Wrap-up artifacts:**
- `TODO.md` — status block bumped to "through Round 36" + Round 36 narrative prepended.
- `MASTER_TEST_PROGRAM.md` — Last-updated — Round 36;  totals 516 — 522 / 42 — 43 files;  catalog row added;  BC-09 row added; coverage-gap re-anchor.

**Suite:** 522 tests / 1 skipped / 43 files (516 — 522 from this round).

**Remaining BC backlog:**
- **BC-12** (proposed — boot-time config-vs-hardware verification self-test): SIL half is doable today against mocked m4_io; HIL half waits for bench rig.
- **BC-13** (proposed — CYBERSECURITY_CASE.md cross-reference for build-config attack surface): doc-only; pin the threat model to the same physical-shape capability list landed in this round.
- **BC-14** (proposed — fleet-wide config inventory tool): aggregate `config_loaded` audit lines across the fleet into a single CSV / table; pure offline tooling.
- The original BC-* delivery list is now closed end-to-end. Remaining items are agent-proposed extensions rather than gaps in the original review.

## Round 37 — BC-14 fleet-wide config inventory tool (2026-04-29)

**IP:** BC-14 (new `lifetrac-config inventory` subcommand aggregates `config_loaded` audit-log events across one or more `audit.jsonl` files into a fleet inventory CSV / Markdown table; depot operator can answer "which fleet vehicles are running which build right now?" without writing ad-hoc jq).

**Code landed:**
- `tools/lifetrac_config.py` — new `inventory` subcommand. Public functions for SIL importability (no subprocess in CI): `iter_config_loaded_events(paths)` parses JSONL streams and yields only `config_loaded` records with the canonical field set (silently skips non-JSON lines, non-dict values, records missing `ts` / `unit_id` / `config_sha256`, and missing files); `aggregate_inventory(records)` folds them into one row per `(unit_id, config_sha256)` pair tracking first_seen / last_seen / boot_count / components / source_paths; `render_inventory_csv(rows)` and `render_inventory_markdown(rows)` produce byte-stable output with ISO-8601 UTC timestamps; `cmd_inventory(args)` glues it together with directory-glob support (`audit*.jsonl*` so log-rotation siblings auto-expand). Module docstring updated for Round 37.

- `DESIGN-CONTROLLER/BUILD_CONFIG.md` — CLI synopsis bumped from seven to eight subcommands (the `inventory` row added). The BC-08 doc gate auto-validates this — `BC08_B_DocCoversEveryCLISubcommand` would have failed if the row had been forgotten.

- `DESIGN-CONTROLLER/base_station/tests/test_config_inventory_sil.py` — new 14-test SIL gate, 5 classes BC14_A..BC14_E:
  * `BC14_A_ParserSkipsNonInventoryAndMalformed` (4): only `config_loaded` records pass; malformed JSON lines / non-dict values / records missing required fields are skipped silently; missing files don't raise.
  * `BC14_B_AggregationDedupesByUnitAndSha` (3): one unit + one SHA = one row with boot_count summed and components unioned; one unit + two SHAs = two rows (reflash visible); two units = two rows sorted by unit_id then last_seen descending.
  * `BC14_C_RenderingIsStable` (3): CSV header == `INVENTORY_FIELDS`; CSV byte-identical across two runs (no nondeterminism from set iteration); Markdown table has header + separator with the right field names.
  * `BC14_D_DirectoryGlobAndCommandIntegration` (2): directory argument picks up rotated `audit*.jsonl*` siblings AND ignores stray non-audit files; `--format markdown` switches the renderer (signal: pipe-delimited header).
  * `BC14_E_TimestampFormatIsISO8601UTC` (2): known epoch (1700000000 = 2023-11-14T22:13:20Z) renders to known ISO; zero / negative renders empty.

**Contract details:**
- Aggregation key is **both** unit_id and SHA, never just unit_id. A reflashed vehicle is a distinct fact; collapsing it would hide the transition the operator is investigating.
- Sort order is `(unit_id, -last_seen)` so the same unit's most-recent build is at the top of its group — the depot operator's first question is always "what's it running NOW?".
- `boot_count` counts every `config_loaded` event, not distinct boots. Both `web_ui` and `lora_bridge` emit a record per process start, so a single tractor boot increments by 2 on the canonical layout. The `components` set lets the operator see which subsystem(s) emitted records — useful for spotting half-booted units (e.g. `components={lora_bridge}` only = web_ui never came up).
- Multi-valued cells (`components`, `source_paths`) join with `;` because comma is the CSV delimiter and `;` is the conventional secondary delimiter; values are sorted before joining for byte-stability.
- SHA truncated to 16 chars in the table for readability — BC-08 still pins that the full 64-char SHA is recoverable from the canonical `config_loaded` JSON record itself (this tool is a viewer, not an authoritative store).
- Public functions (`iter_config_loaded_events`, `aggregate_inventory`, `render_inventory_csv`, `render_inventory_markdown`, `cmd_inventory`) are deliberately importable so the SIL gate exercises them in-process; no `subprocess.run` in CI.

**Tests landed:** 14 tests across 5 classes (full breakdown above).

**Wrap-up artifacts:**
- `TODO.md` — status block bumped to "through Round 37" + Round 37 narrative prepended.
- `MASTER_TEST_PROGRAM.md` — Last-updated — Round 37;  totals 522 — 536 / 43 — 44 files;  catalog row added;  BC-14 row added; coverage-gap re-anchor.
- `DESIGN-CONTROLLER/BUILD_CONFIG.md` — CLI synopsis row added (auto-pinned by BC-08 gate).

**Suite:** 536 tests / 1 skipped / 44 files (522 — 536 from this round).

**Remaining BC backlog:**
- **BC-13** (proposed — CYBERSECURITY_CASE.md cross-reference for build-config attack surface): doc-only; pin the threat model to the Round-36 physical-shape capability list. Highest-confidence next round.
- **BC-12** (proposed — boot-time config-vs-hardware verification self-test): SIL half is doable today against mocked m4_io; HIL half waits for bench rig.
- The original BC-* delivery list remains closed; BC-14 was an agent-proposed extension that landed cleanly.

## Round 38 — BC-13 build-config attack surface in CYBERSECURITY_CASE.md (2026-04-29)

**IP:** BC-13 (treat the build-config subsystem as a first-class IEC-62443 asset class with its own threat model, so the SL-1 cybersecurity case explicitly accounts for "what does an attacker who can mint a TOML get?").

**Code landed:**
- `DESIGN-CONTROLLER/CYBERSECURITY_CASE.md` — new §9 with four sub-sections: 9.1 Z-CONFIG zone (5 named assets); 9.2 STRIDE per build-config asset (9 rows mapping threats on TOML / bundle / push / active config / config_loaded events / generated header to existing mitigations); 9.3 capability-altering leaves are safety-significant (defence-in-depth narrative tying together schema validation, BC-09 BOM cross-reference, `codegen --check`, BC-14 inventory visibility, and the three-watchdog safe-state); 9.4 self-reference to the enforcing drift gate.
- `DESIGN-CONTROLLER/base_station/tests/test_cybersecurity_buildconfig_xref_sil.py` — new 5-test SIL gate, 5 classes BC13_A..BC13_E:
  * `BC13_A_SectionStructure`: §9 has the four expected sub-headings.
  * `BC13_B_NamedCLISubcommandsExist`: every `lifetrac-config <subcommand>` named in §9 resolves through the live `_build_parser()` (no subprocess; uses argparse subparsers introspection so it stays in lockstep with the actual CLI). Catches typos and stale subcommand names without hand-curated lists.
  * `BC13_C_RelativeLinksResolve`: every relative Markdown link target in §9 exists on disk (skips `http(s)://` and `mailto:`).
  * `BC13_D_StrideCoversEveryZConfigAsset`: the §9.2 STRIDE table contains a row whose leftmost cell mentions each curated Z-CONFIG asset token (build.toml / Bundle on USB stick / lifetrac-config push / Active config / config_loaded / Generated firmware header). Curated rather than parsed-from-§9.1 because the wording in the two tables is deliberately different (assets row says "Active config under /etc/lifetrac/", STRIDE row says "Active config on tractor" — the test's job is to check coverage, not surface-form identity).
  * `BC13_E_SelfReferencesEnforcingTest`: §9 contains the literal path `base_station/tests/test_cybersecurity_buildconfig_xref_sil.py` so renaming the test file fails the gate by design.

**Contract details:**
- §9 is doc-only — no firmware or runtime code changed. BC-13 is the missing prose layer that makes the existing technical mitigations (BC-09 BOM gate, `codegen --check`, BC-14 inventory) attributable to specific STRIDE threats in the SL-1 case.
- The curated `_ASSET_TOKENS` tuple in BC13_D is intentional: the assets table and the STRIDE table use deliberately different prose (one is location-oriented, the other action-oriented); a parsed-comparison test would force them into surface-form identity and lose the readability we get from differentiated wording.
- BC13_B uses the live argparse parser, not a hand-curated subcommand list. The same gate philosophy as BC-08 — parity by introspection, never by enumeration.
- One PowerShell-here-string lesson recurred: `\\subcommand\\` (Markdown double-backtick) silently collapses to single backticks under PowerShell's `@"..."@` even with the doubled-up backtick escape. Mitigated in BC13_B by changing the regex from `\\lifetrac-config (\\w[\\w-]*)\\` to `\+lifetrac-config (\\w[\\w-]*)\+` so it tolerates either rendering. Memo added.

**Tests landed:** 5 tests across 5 classes (full breakdown above).

**Wrap-up artifacts:**
- `TODO.md` — status block bumped to "through Round 38" + Round 38 narrative prepended.
- `MASTER_TEST_PROGRAM.md` — Last-updated — Round 38; §1 totals 536 — 541 / 44 — 45 files; §2 catalog row added; §5 BC-13 row added; coverage-gap re-anchor.

**Suite:** 541 tests / 1 skipped / 45 files (536 — 541 from this round).

**Remaining BC backlog:**
- **BC-12** (proposed — boot-time config-vs-hardware verification self-test): SIL half is doable today against mocked m4_io; HIL half waits for bench rig. Now the highest-confidence next round.
- The original BC-* delivery list remains closed; BC-13 and BC-14 were agent-proposed extensions that landed cleanly.

## Round 39 — BC-12 SIL half (boot-time config-vs-hardware self-test) (2026-04-29)

**IP:** BC-12 SIL half (proposed BC-12 was the boot-time config-vs-hardware verification self-test; the SIL half is doable today against an injected `HardwareInventory`; the HIL half — real M4 Modbus probe + tractor-side wiring — remains deferred until bench rig).

**Code landed:**
- `DESIGN-CONTROLLER/base_station/boot_self_test.py` — new module. Public surface: `HardwareInventory` dataclass (what the M4 reports: track/arm axis counts, proportional_capable, hyd pressure sensor count, IMU/GPS presence flags, camera count, LoRa region, aux port count + coupler type + case-drain presence); `SelfTestFinding` dataclass (severity / code / message / expected / observed); `SelfTestReport` dataclass with `errors()` / `warnings()` helpers; `run_self_test(config, hardware, *, now=time.time)` returns the report; `emit_audit(audit, report, *, component)` writes one `boot_self_test` JSONL event. `CODES` tuple is the public catalogue of 12 stable identifiers.
- Severity model: error = safety-significant (axis counts, proportional flow when the config commands ramped flow but hardware can't deliver, pressure-sensor count, IMU/GPS presence, LoRa region — transmitting on the wrong jurisdiction is illegal, aux port_count / coupler_type / case_drain_present); warning = cosmetic (camera shortage / surplus). Errors set `ok=False`; warnings keep `ok=True`.
- Proportional-flow asymmetry is one-sided on purpose: hardware-can't-deliver-what-config-commands fails; hardware-capable-but-config-bang-bang passes (you're just leaving capability on the table).

- `DESIGN-CONTROLLER/base_station/tests/test_boot_self_test_sil.py` — new 15-test SIL gate, 8 classes BC12_A..BC12_H:
  * `BC12_A_MatchingInventoryPasses` (1): matching inventory yields ok=True, empty findings, correct unit_id + config_sha256, finished_ts > started_ts.
  * `BC12_B_AxisCountMismatchFailsBoot` (2): track axis short and arm axis extra each yield error finding + ok=False.
  * `BC12_C_ProportionalFlowAsymmetry` (2): one-sided rule pinned both ways (hardware-deficient fails, config-deficient passes via `dataclasses.replace` on the frozen BuildConfig).
  * `BC12_D_PresenceAndSensorMismatches` (3): IMU presence / GPS presence / pressure-sensor count mismatches each fail boot.
  * `BC12_E_CameraAsymmetryIsWarning` (2): camera shortage AND camera surplus both yield `warning` findings; `ok` stays True.
  * `BC12_F_LoraAndAuxMismatchesAreErrors` (3): LoRa region + aux port count + aux coupler type mismatches each fail boot.
  * `BC12_G_AuditEventShape` (1): `emit_audit` writes exactly one JSONL line; event=`boot_self_test`; flat fields `{component, unit_id, config_sha256, ok, error_count, warning_count, findings[], started_ts, finished_ts}`; each finding is a flat dict with exactly five keys.
  * `BC12_H_FindingCodeCatalogueIsStable` (1): `CODES` equals the 12-tuple in the expected order. Catches silent renames before dashboards / runbooks break.

**Contract details:**
- The HardwareInventory is *injected*, not probed. The SIL gate exercises the comparator + audit emitter; the HIL gate (deferred) will substitute a real Modbus read against the M4. This split is identical in spirit to how BC-06 / Round 34 `hil/dispatch.ps1` reads `lifetrac-config dump-json` instead of probing hardware.
- Finding codes are stable identifiers (`AXIS_COUNT_TRACK` not `axis_count_track`, no spaces, no version suffixes). Dashboards pivot on these strings.
- The audit event is a single JSONL line per boot regardless of finding count — flat-findings format means `jq '.event=="boot_self_test" and .ok==false'` and `jq '.findings[].code'` both work without nested-object handling.
- Composes with BC-14 (Round 37) inventory: a fleet-wide query "which units booted with non-OK self-test in the last week?" is a one-line jq pipeline today.

**Tests landed:** 15 tests across 8 classes (full breakdown above). Suite went 541 — 556 / 45 — 46 files.

**Wrap-up artifacts:**
- `TODO.md` — status block bumped to "through Round 39" + Round 39 narrative prepended.
- `MASTER_TEST_PROGRAM.md` — Last-updated — Round 39; §1 totals 541 — 556 / 45 — 46 files; §2 catalog row added; §5 BC-12 (SIL half) row added; coverage-gap re-anchor.

**Suite:** 556 tests / 1 skipped / 46 files (541 — 556 from this round).

**Remaining BC backlog:**
- **BC-12 HIL half** (deferred): real M4 Modbus probe of axis counts / proportional capability / sensor presence / aux plumbing; integration into `web_ui` + `lora_bridge` boot path so the `boot_self_test` event actually fires per process start. Waits for bench rig.
- The original BC-* delivery list and BC-13 / BC-14 are closed. BC-12 SIL half lands the highest-confidence SIL surface that exists today; the HIL half is the next natural unit of work once a tractor M4 is on the bench.

## Round 40 — BC-14B (config_loaded schema_version emitter follow-up) (2026-04-29)

**IP:** BC-14B (close the BC-14 follow-up gap noted in the Round 37 memo: `web_ui._audit_config_loaded` and `lora_bridge.Bridge.__init__` did NOT include `schema_version` in the `config_loaded` event, so the BC-14 inventory column always rendered empty).

**Code landed:**
- `DESIGN-CONTROLLER/base_station/web_ui.py` — `_audit_config_loaded()` now passes `schema_version=BUILD.schema_version` to `audit.record(...)`.
- `DESIGN-CONTROLLER/base_station/lora_bridge.py` — `Bridge.__init__` boot block now passes `schema_version=cfg.schema_version` likewise.
- `DESIGN-CONTROLLER/base_station/tests/test_config_loaded_schema_version_sil.py` — new 5-test SIL gate, 4 classes BC14B_A..BC14B_D:
  * `BC14B_A_WebUIEmitsSchemaVersion` (1): web_ui's `_audit_config_loaded` writes `schema_version` as `int` from `BUILD.schema_version`. Reuses the BC4_A paho-mock import pattern so module import doesn't TCP-connect.
  * `BC14B_B_LoraBridgeBlockEmitsSchemaVersion` (1): lora_bridge inline boot block writes the same field. Mirrors the BC4_A in-process exercise pattern (drives the 8-line block under `_FakeAudit` rather than instantiating `Bridge`).
  * `BC14B_C_InventorySurfacesSchemaVersion` (1): end-to-end against a synthetic `audit.jsonl` — new lines populate the schema_version CSV column; legacy lines (rotated logs from before Round 40, no schema_version field) still aggregate cleanly with empty cells. Back-compat is part of the contract.
  * `BC14B_D_EmitterSourceContainsSchemaVersion` (2): source greps both call-sites for the literal `schema_version=BUILD.schema_version` and `schema_version=cfg.schema_version` strings. Catches a refactor that drops the field even if the in-process tests happen to mock around it.

**Contract details:**
- The aggregator behaviour around schema_version was always defensive (Round 37 stored `None` for missing field, took the "latest non-None observed" if any). Round 40 makes the field actually populated for new boots while preserving the rotated-log back-compat that BC14B_C pins explicitly.
- Existing BC4_A audit tests (`test_build_config_consumption_sil.py`) check field-by-field with assertEqual, not no-extra-fields, so adding `schema_version` is a pure superset and didn't require updates.
- Lesson: when a test imports web_ui, the module-level `_connect_mqtt_with_retry()` will TCP-connect to localhost:1883 unless paho.mqtt.client.Client is patched. Reuse the BC4_A `mock.patch("paho.mqtt.client.Client")` block (caught this on first run; fix was switching from naive importlib.reload to the BC4_A pattern).

**Tests landed:** 5 tests across 4 classes (full breakdown above). Suite went 556 — 561 / 46 — 47 files.

**Wrap-up artifacts:**
- `TODO.md` — status block bumped to "through Round 40" + Round 40 narrative prepended.
- `MASTER_TEST_PROGRAM.md` — Last-updated — Round 40; §1 totals 556 — 561 / 46 — 47 files; §2 catalog row added; §5 BC-14B row added; coverage-gap re-anchor.

**Suite:** 561 tests / 1 skipped / 47 files (556 — 561 from this round).

**Remaining BC backlog:**
- **BC-12 HIL half** (deferred): real M4 Modbus probe of axis counts / proportional capability / sensor presence / aux plumbing; integration into `web_ui` + `lora_bridge` boot path so the `boot_self_test` event actually fires per process start. Waits for bench rig.
- The original BC-* delivery list, BC-13, BC-14, BC-14B, and BC-12 SIL half are all closed.

## Round 41 — BC-12C (lifetrac-config self-test CLI subcommand) (2026-04-29)

**IP:** BC-12C — the bench-side CLI half of BC-12. Round 39 landed the SIL `boot_self_test.run_self_test` comparator + audit emitter; the HIL half (real M4 Modbus probe) waits for bench rig. Round 41 fills the operator gap in between: a depot tech with a captured `HardwareInventory` JSON file and a candidate TOML can dry-run the exact same self-test from a laptop, before flashing.

**Code landed:**
- `tools/lifetrac_config.py` — new `cmd_self_test(args)` and `self-test` subparser (eight — nine subcommands). Loads TOML via the existing `_load_validated` helper, parses the inventory JSON, instantiates `HardwareInventory(**raw)`, calls `run_self_test`, prints text or JSON. Exit code is `EXIT_OK` if `report.ok` else `EXIT_USAGE`; same code on malformed-input errors so CI gates can `\0 -ne 0` uniformly.
- `DESIGN-CONTROLLER/BUILD_CONFIG.md` — synopsis row + subcommand table row added; "the eight subcommands are" — "the nine subcommands are"; §2 capability row appended `(... / dump-json / inventory / self-test)`.
- `DESIGN-CONTROLLER/base_station/tests/test_config_self_test_cli_sil.py` — new 10-test SIL gate, 5 classes BC12C_A..BC12C_E:
  * `BC12C_A_MatchingInventoryExitsZero` (2): text format prints `ok : True` with no findings section; JSON format returns `{ok:true, error_count:0, warning_count:0, findings:[], unit_id, config_sha256}`.
  * `BC12C_B_ErrorMismatchExitsNonZero` (2): twiddling `track_axis_count` or `arm_axis_count` exits `EXIT_USAGE`; text output names the stable code (`AXIS_COUNT_TRACK`); JSON output carries `severity=error` + `expected`/`observed` per finding.
  * `BC12C_C_WarningOnlyKeepsExitZero` (1): twiddling `camera_count` (warning-only per Round 39 contract) keeps exit 0 with `warning_count >= 1`, `error_count == 0`, and a `CAMERA_COUNT_*` warning surfaced.
  * `BC12C_D_BadInputsRejected` (4): missing inventory file, top-level JSON list (not an object), missing dataclass fields (`HardwareInventory(**raw)` raises `TypeError`), unparseable JSON — all four exit `EXIT_USAGE` with diagnostic stderr messages (`inventory file not found`, `top-level object`, `inventory shape mismatch`, `cannot parse inventory JSON`).
  * `BC12C_E_SourceContainsSubcommand` (1): tripwire greps the CLI source for `\"self-test\"`, `def cmd_self_test`, and `set_defaults(func=cmd_self_test)` so a refactor that drops the registration fails the gate even if mocking happens to hide it.

**Contract details:**
- Tests invoke `lifetrac_config.main(argv)` in-process (with `redirect_stdout` / `redirect_stderr` capture) rather than subprocess; matches the BC-14 `test_config_inventory_sil.py` pattern and keeps the suite fast.
- `HardwareInventory` JSON shape is the dataclass field names verbatim. Operators capturing inventories from M4 Modbus dumps (or the future HIL probe) write the same JSON the SIL fixtures use.
- Composition with BC-08 doc gate: adding a ninth subcommand without doc'ing it fails `test_doc_lists_every_subcommand_and_no_orphans`. Caught this on first suite run; first remediation also tripped the doc gate's "doc names a subcommand the CLI doesn't register" assertion because `--format` choices `text`/`json` got backticked in the table cell. Fixed by un-backticking those tokens (the gate only scans backticked tokens). Lesson: in BUILD_CONFIG.md §4 synopsis cells, do not backtick anything that isn't a real subcommand name.

**Tests landed:** 10 tests across 5 classes (full breakdown above). Suite went 561 — 571 / 47 — 48 files.

**Wrap-up artifacts:**
- `TODO.md` — status block bumped "through Round 41" + Round 41 narrative prepended.
- `MASTER_TEST_PROGRAM.md` — Last-updated — Round 41; §1 totals 561 — 571 / 47 — 48 files; §2 catalog row added; §5 BC-12C row added; coverage-gap re-anchor.
- `DESIGN-CONTROLLER/BUILD_CONFIG.md` — synopsis bumped eight — nine subcommands.

**Suite:** 571 tests / 1 skipped / 48 files (561 — 571 from this round).

**Remaining BC backlog:**
- **BC-12 HIL half** (deferred, unchanged): real M4 Modbus probe + integration into `web_ui` / `lora_bridge` boot path so `boot_self_test` event fires per process start. Now strictly waits for bench rig — the laptop-side dry-run path is closed.
- The original BC-* delivery list, BC-13, BC-14, BC-14B, BC-12 SIL half, and BC-12C are all closed.
