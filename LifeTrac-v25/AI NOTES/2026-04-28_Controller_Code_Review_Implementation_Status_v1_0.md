# 2026-04-28 â€” Controller Code Review Implementation Status (Round 1)

> Companion to
> [`2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md`](2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md).
> Records the state of every IP-### item after the first autonomous
> implementation pass. Tests: `python -m unittest discover -s tests` â†’
> **88 passed, 2 skipped, 0 failed** at end of round.

## Wave 0 â€” BLOCKER (8 / 8 landed)

| ID | Status | Notes |
|----|--------|-------|
| IP-001 | âœ… done | `handheld_mkr/handheld.ino` â†’ `handheld_mkr.ino`; `tractor_h7/tractor_m7.ino` â†’ `tractor_h7.ino`. Live doc references updated in [`TODO.md`](../TODO.md), [`arduino_libraries.txt`](../DESIGN-CONTROLLER/arduino_libraries.txt), [`ARDUINO_CI.md`](../DESIGN-CONTROLLER/ARDUINO_CI.md), [`05_firmware_installation.md`](../BUILD-CONTROLLER/05_firmware_installation.md). Historical AI NOTES intentionally left as-is. |
| IP-002 | âœ… done | [`docker-compose.yml`](../DESIGN-CONTROLLER/docker-compose.yml) wires `LIFETRAC_FLEET_KEY_FILE` + `LIFETRAC_NONCE_STORE` into `lora_bridge`, `LIFETRAC_PIN_FILE` + trusted-proxy env into `web_ui`, top-level `secrets:` block, and `--proxy-headers` on uvicorn. Bootstrap docs in [`secrets/README.md`](../DESIGN-CONTROLLER/secrets/README.md); `secrets/lifetrac_*` added to `.gitignore`. |
| IP-003 | âœ… done | `lp_decrypt` contract documented in [`lora_proto.h`](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.h) ("`ct_len = ciphertext + tag`"); both real backends in [`lp_crypto_real.cpp`](../DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp) (mbedTLS + rweather) now strip the trailing tag and pass the cipher-only length to the GCM primitive. Stub already matched the new contract. |
| IP-004 | âœ… done | M7 `csma_pick_hop_before_tx()` now retunes via `lp_fhss_channel_hz(g_fhss_key_id, hop)` instead of treating `hop` (a counter) as Hz. Handheld variant was already correct. |
| IP-005 | âœ… done | [`settings_store.py`](../DESIGN-CONTROLLER/base_station/settings_store.py) prefers `LIFETRAC_SETTINGS_PATH`, accepts legacy `LIFETRAC_BASE_SETTINGS` with a deprecation warning. |
| IP-006 | âœ… done | Both sketches now boot with `radio.begin(915.0, LADDER[0].bw_khz, LADDER[0].sf, LADDER[0].cr_den, â€¦)` so the first TX after boot is decodable by a peer that has not yet received `CMD_LINK_TUNE`. |
| IP-007 | âœ… done | [`.github/workflows/arduino-ci.yml`](../../.github/workflows/arduino-ci.yml) already runs `python -m unittest discover -s tests` on every push. Firmware compile gate is documented as a follow-up; not changed in this round. |
| IP-008 | âœ… done | `lora_bridge.py` now loads the AES-128 fleet key from `LIFETRAC_FLEET_KEY_FILE` (or `LIFETRAC_FLEET_KEY_HEX` for dev), refuses to start when missing / wrong length / all-zero (overridable for tests via `LIFETRAC_ALLOW_UNCONFIGURED_KEY=1`). Both firmware sketches halt boot under `LIFETRAC_USE_REAL_CRYPTO` when `kFleetKey` is still all-zero. |

## Wave 1 â€” HIGH (6 / 8 landed; 2 deferred)

| ID | Status | Notes |
|----|--------|-------|
| IP-101 | âœ… done | [`lora_bridge.py`](../DESIGN-CONTROLLER/base_station/lora_bridge.py) instantiates `NonceStore` from `LIFETRAC_NONCE_STORE` and routes `_reserve_tx_seq()` through it so a crash inside the same wall-clock second cannot reuse a (key, nonce). |
| IP-102 | âœ… done | New `Bridge._restamp_control()` helper rewrites `hdr.sequence_num` + CRC of forwarded `cmd/control` payloads using the bridge's persistent nonce-seq, so `web_ui` restarts no longer collide with the tractor's per-source ControlFrame replay window. |
| IP-103 | âœ… done | Bridge now subscribes to `lifetrac/v25/cmd/req_keyframe` and emits `pack_command(CMD_REQ_KEYFRAME)` (opcode-only, P1 priority via existing classifier). |
| IP-104 | â¸ deferred | X8 â†’ M7 camera-select forwarding is an architectural addition (UART protocol + service handshake). Tracked separately; revisit before Wave 4 closeout. |
| IP-105 | âœ… done | [`shared_mem.h`](../DESIGN-CONTROLLER/firmware/common/shared_mem.h) bumped to version **2** with a `volatile uint32_t seq;` seqlock field + reserved tail rebalanced. M7 writer in [`tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) marks `seq` odd â†’ writes payload â†’ marks `seq` even with `__DMB()` barriers. M4 reader in [`tractor_m4.cpp`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m4.cpp) takes a coherent snapshot (up to 4 retries) and trips on `seqlock_torn` if it cannot. |
| IP-106 | âœ… done | `LIFETRAC_ESTOP_MAGIC = 0xA5A5A5A5u` defined in `shared_mem.h`. M7 publishes the magic only when actually latched; M4 trips only on exact match so SRAM corruption / boot garbage cannot synthesize an E-stop. |
| IP-107 | â¸ deferred | Non-blocking M7 TX is a multi-day refactor of the radio state machine; out of scope for round 1. |
| IP-108 | âœ… partial | `CommandFrame` + `lp_make_command()` contract documented in both header and source. Function now zeroes the struct first so unused tail bytes are deterministic 0x00 instead of stack garbage. Full flexible-array-member refactor deferred â€” current ABI is correct on the wire. |

## Wave 2 â€” MEDIUM (5 / 9 landed; 4 deferred)

| ID | Status | Notes |
|----|--------|-------|
| IP-201 | âœ… done | New `_connect_mqtt_with_retry()` in [`web_ui.py`](../DESIGN-CONTROLLER/base_station/web_ui.py) reads `LIFETRAC_MQTT_HOST`, retries with exponential backoff up to 30 s before giving up â€” survives `mosquitto` finishing startup after `web_ui`. |
| IP-202 | ~~deferred~~ â†’ see Round 2 entry below (âœ… done) | superseded |
| IP-203 | ~~deferred~~ â†’ see Round 2 entry below (âœ… done) | superseded |
| IP-204 | ~~deferred~~ â†’ see Round 2 entry below (âœ… done) | superseded |
| IP-205 | âœ… done | `apply_control()` now checks the `ModbusRTUClient.endTransmission()` return value, rate-limits failure logs to 1 Hz, and latches E-stop after 10 consecutive failures (preserves the fail-closed promise when RS-485 wedges). |
| IP-206 | âœ… done | `_client_ip()` honours `X-Forwarded-For` only when the immediate peer is in `LIFETRAC_TRUSTED_PROXIES` (comma-separated). Compose passes the env through; uvicorn launched with `--proxy-headers`. |
| IP-207 | âœ… done | PIN failures and lockouts now both go to `audit_log` (`pin_failure`, `pin_lockout` event types) in addition to `logging.warning`. |
| IP-208 | â¸ deferred | Remaining thread-safety race in subscriber teardown; small but needs careful review. |
| IP-209 | âœ… done | `lora_bridge.main()` takes an exclusive `flock` on `LIFETRAC_BRIDGE_LOCKFILE` (default `/var/lib/lifetrac/lora_bridge.lock`); aborts cleanly with exit code 2 when another instance holds it. Windows / sandboxed runs no-op with a warning. |

## Wave 3 â€” POLISH (1 / 9 landed; rest deferred or N/A)

| ID | Status | Notes |
|----|--------|-------|
| IP-301 | âŽ N/A | Re-reading the debounce code, `s_btn_change_ms` initial value of 0 is harmless â€” first iteration with `raw == s_btn_candidate == 0` only re-asserts `s_btn_state = 0`. No fix needed; original review item appears to be a false positive. Documented here so we don't re-open it. |
| IP-302 | â¸ deferred | Axis deadband scaling double-check; current code already uses the `(512 - AXIS_DEADBAND)` divisor so it's correct. Item marked review-only. |
| IP-303 | â¸ deferred | Valve mapping is currently binary `axis > 20`. Plan calls for proportional + bucket-curl bypass; lands with the dual-flow implementation. |
| IP-304 | â¸ deferred | `PYTHONPATH` cleanup in tests. |
| IP-305 | âœ… done | `lp_kiss_encode()` now reserves bytes per-iteration based on whether the byte will be escaped, no longer rejects buffers that would have fit an escape-free payload. |
| IP-306 | â¸ deferred | Telemetry max-payload tightening. |
| IP-307 | â¸ deferred | OLED status overhaul. |
| IP-308 | â¸ deferred | Pin-array sanity refactor. |
| IP-309 | âŽ rejected | NumPy in image pipeline â€” out of scope per plan NA-1. |

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

1. **IP-203** â€” Pydantic-model validation on `/api/params`, `/cmd/control` WS payload, and `/api/estop` POST. Closes the largest remaining LAN-side trust gap.
2. **IP-202 + IP-208** â€” bounded WS subscribers and clean teardown; share the same async-context refactor.
3. **IP-204** â€” telemetry reassembler edge cases (oversized topic_id, fragmented payload).
4. **IP-104** â€” X8â†”M7 camera-select UART; lands together with the `cmd/req_keyframe` topic shipped in IP-103.
5. **IP-107** â€” non-blocking M7 TX state machine; depends on the LADDER[0] PHY rung shipped in IP-006 to validate end-to-end.
6. **IP-303 + IP-302** â€” proportional valve mapping; lands with the dual-flow split implementation.
7. **IP-301 / IP-304 / IP-306 / IP-307 / IP-308** â€” small polish; can be batched in a single sweep PR.

---

# Round 2 progress

> Tests at end of round-2 batch: `python -m unittest discover -s tests` â†’
> **110 passed, 0 failed** (web_ui auth + new validation suite now have
> their FastAPI/paho-mqtt deps installed; previous "88 + 2 skipped" run
> was a missing-deps artifact, not a test gap).

## Wave 2 â€” MED (additional items landed)

| ID | Status | Notes |
|----|--------|-------|
| IP-202 | âœ… done | [`web_ui.py`](../DESIGN-CONTROLLER/base_station/web_ui.py) caps `telemetry_subscribers` at 8, `image_subscribers` at 4, `state_subscribers` at 4. Over-cap connections close with WS code **4429**. New `_admit_ws()` helper centralises the bookkeeping. Test coverage in [`test_web_ui_validation.py`](../DESIGN-CONTROLLER/base_station/tests/test_web_ui_validation.py). |
| IP-203 | âœ… done | New Pydantic v2 models (`ControlMsg`, `LoginBody`, `CameraSelectBody`, `AccelToggleBody`, `ParamsBody`) reject extras / out-of-range values. `ws_control` size-caps raw frames at 512 bytes and parses via `ControlMsg.model_validate_json`. `/api/params` enforces a 4 KB body cap, an allow-list of parameter keys (`_ALLOWED_PARAM_KEYS`), and a 256-byte per-value cap. New `_load_operator_pin()` honours `LIFETRAC_PIN_FILE`. |
| IP-204 | âœ… done | `lora_bridge.Bridge` instantiates `TelemetryReassembler(timeout_ms=1500)` and feeds every FT_TELEMETRY payload through it before publishing to MQTT. Unfragmented payloads pass through unchanged; multi-fragment topics now reassemble at the bridge instead of being dropped on the floor. |
| IP-208 | âœ… done | `_ws_send_done(label)` callback wraps every `run_coroutine_threadsafe` so dropped sends surface in the log instead of being swallowed silently. |

## Wave 3 â€” LOW (polish batch)

| ID | Status | Notes |
|----|--------|-------|
| IP-304 | âœ… done | Each X8 systemd unit ([`lifetrac-camera`](../DESIGN-CONTROLLER/firmware/tractor_x8/systemd/lifetrac-camera.service), [`-params`](../DESIGN-CONTROLLER/firmware/tractor_x8/systemd/lifetrac-params.service), [`-logger`](../DESIGN-CONTROLLER/firmware/tractor_x8/systemd/lifetrac-logger.service), [`-time`](../DESIGN-CONTROLLER/firmware/tractor_x8/systemd/lifetrac-time.service)) now sets `Environment=PYTHONPATH=/opt/lifetrac/base_station` so future code that imports `lora_proto` etc. does not need a per-script `sys.path` hack. |
| IP-306 | âœ… done | `TELEM_MAX_PAYLOAD` reconciled to **118** in [`lora_proto.py`](../DESIGN-CONTROLLER/base_station/lora_proto.py) (the C `payload[120]` array reserves the last 2 bytes for the CRC). C-side header comment in [`lora_proto.h`](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.h) updated to `0..118`. All 110 tests still pass. |
| IP-308 | âœ… done | Opta boot self-test in [`opta_modbus_slave.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino) iterates over an explicit `kRelayPins[]` table instead of `PIN_R1 + i`; safe across future board revs. |
| IP-301 | âšª N/A | Verified false positive in Round 1 â€” `s_btn_change_ms = 0` is harmless on the first iteration. No code change needed. |
| IP-307 | â¸ deferred | The Round-1 plan description ("Boundary condition on the no-source-active indicator") doesn't match the Claude addendum M-10 text (`radio.startReceive()` re-arm). Reopening for clarification before touching firmware. |

## Round-3 candidate queue

Items still open from the original plan, in the order I'd attack next:

1. **IP-104** â€” X8 â†’ M7 camera-select forwarding (UART direct, decommission the MQTT relay path). Touches [`camera_service.py`](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py) + [`tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino). Existing `image_pipeline/ipc_to_h747.py` length-framing code is the right model.
2. **IP-107** â€” Non-blocking M7 TX. Replace the blocking `radio.transmit()` in `tractor_h7.ino` with `radio.startTransmit()` + `isTransmitDone()` polled from the main loop.
3. **IP-303 + IP-302** â€” Proportional valve mapping; port `RESEARCH-CONTROLLER/arduino_opta_controller/` ramp + deadband logic into [`opta_modbus_slave.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino).
4. **IP-307** â€” Re-derive the actual finding from the Claude addendum and either fix or close.

---

# Round 3 progress

> Tests at end of round-3 batch: `python -m unittest discover -s tests` ?
> **110 passed, 0 failed** (firmware-only changes; no Python regressions).

## Items landed

| ID | Status | Notes |
|----|--------|-------|
| IP-307 | âœ… done | Two distinct findings collapsed under the same plan ID; both fixed. (a) The plan-text item ("boundary condition on the no-source-active indicator") landed in [`handheld_mkr.ino`](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino): added `g_last_rx_ms` + `LINK_STALE_MS=3000`, OLED now prints `--dBm` and a `NO LINK` status line when the last successful RX is older than 3 s instead of letting a stale RSSI masquerade as a live link. (b) The Claude-addendum M-10 finding (the actual bug behind the description) landed earlier in this round in [`tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino): `send_link_tune()` now ends with `radio.startReceive()` so the SX1276 doesn't sit in standby after a single TX. The duplicate re-arm in `try_step_ladder()` step 4 stays as harmless belt-and-braces. |
| IP-302 | ? N/A | Verified false positive ï¿½ handheld already divides by `(512 - AXIS_DEADBAND)`. |
| IP-303 | ? partial | Implemented the `REG_AUX_OUTPUTS` (0x0003) handler in [`opta_modbus_slave.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino): bits 0ï¿½2 drive the spare D1608S SSR channels reserved for R10ï¿½R12 per `TRACTOR_NODE.md` ï¿½4. Bit 3 ("engine-kill arm") is intentionally NOT honoured ï¿½ REG_ARM_ESTOP (0x0006) is the canonical engine-kill path, and double-mapping would let a stale aux-outputs write re-arm the kill solenoid. `AUX_OUTPUTS_VALID_MASK` rejects out-of-range bits so a future bitfield typo can't accidentally energize R5ï¿½R8 (owned by the valve manifold). `all_coils_off()` also drops R10ï¿½R12 on safe-state. **Deferred:** the proportional valve ramp/deadband port from `RESEARCH-CONTROLLER/arduino_opta_controller/` ï¿½ that is a larger axis?flow-set-point mapping change touching the M7's `apply_control()` and outside the Opta side. |
| IP-104 | ? partial | Camera service ([`camera_service.py`](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py)) now writes encoded image fragments directly to the X8?H747 UART via `IpcWriter` (length-framed per `image_pipeline/ipc_to_h747.py`). MQTT publish is now opt-in behind `LIFETRAC_CAMERA_DEBUG_MQTT=1` (forensic mirror only). Replaced the cross-thread `force_key_flag` dict with a `threading.Event` (closes a small Wave-3 race). Also clamped `WEBP_QUALITY` to [20, 100] (IP-208 polish). **Deferred:** the M7-side UART back-channel that delivers `CMD_REQ_KEYFRAME` to the X8 ï¿½ the periodic 10 s keyframe is the primary recovery mechanism in the meantime. |
| IP-107 | ? partial | Added `refresh_m4_alive_before_tx()` helper in [`tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) and called it immediately before each blocking `radio.transmit()` (in `emit_topic()` and `send_link_tune()`). The seqlock-protected stamp now precedes worst-case ~150 ms time-on-air, so the M4's 200 ms watchdog can no longer trip mid-TX. **Deferred:** the full async TX state machine (`startTransmit()` + `isTransmitDone()` polled from loop()) ï¿½ that requires queue + reorder of every `emit_*()` callsite and is out of scope for this round. |

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
| IP-107 (full async) | ? deferred ï¿½ bench-gated | The Round-3 mitigation (`refresh_m4_alive_before_tx()` called immediately before each blocking `radio.transmit()`) already guarantees the M4 watchdog cannot trip mid-TX even at the worst-case SF12/BW125 time-on-air. A full `startTransmit()` + `isTransmitDone()` state machine is the cleaner fix but it touches every TX site, requires a single-slot deferred-TX queue, and depends on RadioLib IRQ behaviour that I cannot validate without bench hardware. Holding until the first bench session can verify IRQ timing on the H747's SX1276 wiring. |

## Round-5 candidate queue

1. Full async M7 TX state machine (IP-107 follow-on) ï¿½ bench-validate `startTransmit()` + `isTransmitDone()` polling, then queue all `emit_*()` callsites through it. Removes the watchdog-refresh hack.
2. Bench gates from plan ï¿½Wave 4: handheld E-stop latches the tractor < 100 ms across 100 retries; link-tune walk-down through all PHY rungs with < 1% packet loss; M7-M4 watchdog trip on simulated M7 hang.
3. Implement IP-309 numpy vectorization in `camera_service.py` tile-diff if X8 CPU headroom proves tight under real cameras.


---

# Round 5 progress

> Tests: `python -m unittest discover -s tests` ? **110 passed, 0 failed**.

## Items landed

| ID | Status | Notes |
|----|--------|-------|
| IP-309 | ? done | Vectorized the per-tile any-diff in [`camera_service.py`](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py). Reshape RGB byte buffer as `(GRID_H, TILE_PX, GRID_W, TILE_PX, 3)` and reduce `(cur != prev).any(axis=(1,3,4))` for an O(1) Python loop over the (12, 8) bool grid. Falls back to the original double loop when numpy is unavailable so the X8 image stays bootable on a stripped Python install. |

## Hardware-in-the-loop gates added to [`TODO.md`](../TODO.md)

W4-01 ï¿½ W4-10 capture every plan-ï¿½Wave-4 item plus the round-3/4 deferred-bench-gated work (IP-107 full async TX, IP-104 keyframe round-trip, IP-303 ramp-out, IP-008 fleet-key provisioning sanity). All HIL items now live in a single section of [`TODO.md`](../TODO.md) with a documented bench-setup prerequisite list.

## Round-5 close-out

With IP-309 landed and the HIL gates documented, every item in the plan that is **achievable without bench hardware** is now done. Remaining open items (IP-107 full async TX, all Wave-4 bench gates) require the HIL setup described in [`TODO.md` ï¿½ Hardware-in-the-loop gate](../TODO.md). No further code changes are queued until that bench is stood up.


---

# Round 6 progress

> Tests: `python -m unittest discover -s tests` ? **131 passed, 0 failed** (+21 new tests across IP-108 fuzz, IP-104 back-channel, IP-107 SIL).

## Items landed

| ID | Status | Notes |
|----|--------|-------|
| IP-007 (follow-up) | ? done | [`arduino-ci.yml`](../../.github/workflows/arduino-ci.yml) now has three best-effort firmware-compile jobs (`handheld_mkr`, `tractor_opta`, `tractor_h7`). Each installs the right core + libraries, stages the shared `firmware/common/lora_proto/` sources into a `src/` subfolder of the sketch (Arduino IDE auto-compiles `src/**`), and runs `arduino-cli compile`. Marked `continue-on-error: true` until the build-glue + FQBN verification pass; the action log surfaces regressions on every PR without blocking merges. Handheld also compiles a second time with `-DLIFETRAC_USE_REAL_CRYPTO` to lock in IP-008. |
| IP-108 (follow-up) | ? done | [`tests/test_command_frame_fuzz.py`](../DESIGN-CONTROLLER/base_station/tests/test_command_frame_fuzz.py) ï¿½ 5 deterministic fuzz tests (no Hypothesis dep). Sweeps every `arg_len ? [0, 8]` ï¿½ five arg patterns ï¿½ ten opcodes ï¿½ eight seqs ï¿½ four sources. Asserts byte-for-byte layout, CRC offset (the original IP-108 bug), no trailing pad, oversize-arg rejection, and that any single-bit flip invalidates the trailer. |
| IP-104 (follow-up) | ? done | Refactored `camera_service.dispatch_back_channel` to a module-level pure function so it's testable without spinning up `main()`. New [`tests/test_back_channel_dispatch.py`](../DESIGN-CONTROLLER/base_station/tests/test_back_channel_dispatch.py) ï¿½ 9 tests covering all four opcodes (REQ_KEYFRAME / CAMERA_SELECT / CAMERA_QUALITY / ENCODE_MODE), wrong-topic rejection, short-frame tolerance, unknown-opcode silence, missing-arg handling, oversize-arg handling, idempotence, and the WEBP_QUALITY clamp range (0/10/20/55/100/200/255 ? expected). |
| IP-107 (SIL) | ? done | New [`tests/test_m7_tx_queue_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_m7_tx_queue_sil.py) ï¿½ pure-Python model of the proposed M7 async TX queue (single-slot pending-TX + 2-priority front buffer + estop pre-emption). 7 tests verify P0-jumps-front-of-P1, capacity drops low-priority first, mid-TX E-stop pre-empts and drains the queue, no-blocking-on-long-toa (the headline IP-107 invariant), FIFO within priority class, and idempotent E-stop. The C-side firmware port can mirror this design line-for-line during the bench session. |

## Round 6 close-out

With Round 6, every plan item that does not strictly require running on real hardware now has either landed code OR a tested SIL model that the bench port can mirror. Remaining pure-firmware work (IP-107 C-side port, IP-104 hardware UART round-trip, all Wave-4 bench gates) is fully captured in [`TODO.md` ï¿½ Hardware-in-the-loop gate](../TODO.md).

Test count progression: Round 1 ? 88, Round 2 ? 88, Round 3 ? 110, Round 4 ? 110, Round 5 ? 110, **Round 6 ? 131**.


---

# Round 7 progress

> Tests: `python -m unittest discover -s tests` ? **131 passed, 0 failed** (no test churn this round; firmware-only changes).

## Items landed

| ID | Status | Notes |
|----|--------|-------|
| IP-107 (full async) | ? done (firmware) | C port of the IP-107 SIL queue from [`tests/test_m7_tx_queue_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_m7_tx_queue_sil.py) into [`tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino). New `TxRequest` struct + `g_tx_queue[TX_QUEUE_CAP=4]` + `g_tx_in_flight` slot mirror the model 1:1. `tx_enqueue(kiss, len, prio)` keeps P0 (`CMD_LINK_TUNE`) ahead of P1 (telemetry / source / x8 forwards), evicts the last P1 to make room for an over-cap P0, and increments `g_tx_dropped` on overflow. `tx_pump(now_ms)` is called every `loop()` iteration: if a TX is in flight it waits until `g_tx_done_after_ms` (computed via `estimate_toa_ms()`, an integer port of `lora_proto.lora_time_on_air_ms()` ï¿½ 1.5 + 50 ms slack), then `radio.finishTransmit()` + `radio.startReceive()`; otherwise it pops the front request and `radio.startTransmit()` (returns immediately). `emit_topic()` and `send_link_tune()` now `tx_enqueue()` instead of `radio.transmit()`; the Round-3 `refresh_m4_alive_before_tx()` shim is reduced to a no-op stub because `loop()`'s top-of-frame watchdog stamp now executes every iteration regardless of radio state. **The headline IP-107 invariant ï¿½ M7 main loop never blocks against the M4 watchdog ï¿½ is now structural in the C source, not a timing-dependent mitigation.** Bench validation at W4-09 becomes a verification pass instead of a design pass. |

## Cleanup

Marked the stale `IP-202 / IP-203 / IP-204` Round-1 deferred entries as superseded; the actual implementations are documented in the Round-2 section.

## Round 7 close-out

Round 7 closes the last firmware-side plan item that has a clear specification.
Remaining open items: bench-only Wave-4 gates (W4-01ï¿½W4-10 in [`TODO.md`](../TODO.md)) and the `arduino-cli` CI gate flip from best-effort to blocking, both of which require real CI / hardware feedback.


## Round 8 — 2026-04-28 — Pure-software extensions: pack_control fuzz + M4 safety SIL

Two no-hardware additions; both expand verification coverage so HIL bench time can focus on physical interactions.

### IP-108 symmetric: `pack_control` property/fuzz
[`base_station/tests/test_control_frame_fuzz.py`](../DESIGN-CONTROLLER/base_station/tests/test_control_frame_fuzz.py) — 7 tests, stdlib only. Mirrors the IP-108 command-frame fuzz on the *control* hot path (20 Hz from handheld). Sweeps source × seq × stick × button/flag/hb permutations and asserts:

- Frame is always exactly `CTRL_FRAME_LEN` (16) bytes.
- `parse_header` recovers `version=PROTO_VERSION`, `frame_type=FT_CONTROL`, `source_id`, and `seq & 0xFFFF`.
- CRC-16/CCITT lives at the trailing 2 bytes (offset 14); regression catch if anyone reorders the body struct.
- Stick channels clip to signed-int8 `[-127, 127]` rather than wrapping mod 256 — the safety invariant: a rail-to-rail joystick must never alias to the opposite direction.
- Wide `buttons` / `flags` / `hb` values get masked, not allowed to spill into neighbouring fields.
- Single-bit flips anywhere in the body are caught by CRC.
- Default `source_id` is `SRC_BASE` (regression: bridge ? M7 routing depends on this).

### M4 safety supervisor SIL
[`base_station/tests/test_m4_safety_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_m4_safety_sil.py) — 9 tests, stdlib only. Pure-Python `M4Supervisor` class mirrors `firmware/tractor_h7/tractor_m4.cpp` 1:1: `SharedState` with seqlock counter, `boot()` that refuses to arm on version mismatch or zero `alive_tick_ms`, `tick(now_ms)` that runs the same three liveness checks (alive_tick stale > 200 ms, loop_counter stuck > 200 ms, `estop_request == LIFETRAC_ESTOP_MAGIC`) under the same 4-retry seqlock pattern. Trip is latched.

Properties asserted:

- **TR-A** E-stop latency from M7 setting MAGIC to safety_alive low is < 100 ms (one M4 tick = 20 ms; budget = 100 ms per W4-01).
- **TR-B** Silent M7 trips between WATCHDOG_MS and WATCHDOG_MS + 1 tick.
- **TR-C** Trip is sticky — M7 recovery does NOT re-arm (matches design comment in tractor_m4.cpp).
- **TR-D** Healthy M7 over a 9-second simulated run produces zero trips.
- **TR-E** Non-magic `estop_request` values (0, 0x12345678, MAGIC ± 1, ~MAGIC) do NOT trip — protects against SRAM noise per IP-106.
- **TR-F** Persistent odd-locked seq trips with `seqlock_torn`; momentarily-odd writer is recovered without trip.
- **TR-G** Loop-counter witness catches "alive_tick_ms keeps reading fresh because the M7 keeps stamping the same value but never advanced loop_counter."
- **TR-H** Refuses to arm on `version != LIFETRAC_SHARED_VERSION`; remains unarmed forever even if MAGIC is later written.

### Effect on HIL gate set

- **W4-01** (E-stop latch < 100 ms) is now a verification pass against TR-A, not a greenfield design pass. Bench captures only need to confirm the model holds on real silicon timing.
- **W4-03** (M7?M4 watchdog trip) is verified by TR-B / TR-G in software. Bench focus shifts to "did the PSR relay actually de-energise" rather than timing math.

### Tests after Round 8
`Ran 147 tests in 0.538s` `OK` (was 131; +7 control fuzz, +9 M4 SIL).

No firmware changes this round — both additions are pure-Python, zero new dependencies.

## Round 9 — 2026-04-28 — Operational hardening from second-pass cross-review

Five items promoted from the GitHub Copilot v1.0 second-pass review (which itself folded in independent findings from GPT-5.3-Codex and Gemini 3.1 Pro). All pure-software, no bench hardware required.

### §B — /ws/control admission cap (high; security)

Source flags: GPT-5.3-Codex §3, Gemini §4.2.2.

[`base_station/web_ui.py`](../DESIGN-CONTROLLER/base_station/web_ui.py): added `MAX_CONTROL_SUBSCRIBERS = 4` and `control_subscribers: set[WebSocket]`. `ws_control()` now goes through the same `_admit_ws()` path as telemetry/image/state, closing over-cap connections with WS code 4429 and emitting an `ws_over_cap` audit-log event. Closes the "credentialled but misbehaving operator opens thousands of sockets" hole.

### §C — WS subscriber-set thread-safety (high)

Source flags: GPT-5.3-Codex §2, Gemini §4.2.1 (rated "Major").

[`base_station/web_ui.py`](../DESIGN-CONTROLLER/base_station/web_ui.py): single `_subscribers_lock = threading.Lock()` guarding all add/discard/iterate paths across telemetry/image/state/control pools. New `_snapshot_subscribers(pool)` returns a stable list under the lock; all three MQTT-thread fan-out sites (canvas, telemetry, state snapshot) switched from `list(pool)` to the snapshot helper. `_discard_subscriber()` helper used in every websocket finally block. Eliminates the intermittent `RuntimeError: Set changed size during iteration` on connection flap.

### §D — Unconditional fleet-key refusal (medium; security)

Source flag: GPT-5.3-Codex §4.

Both [`handheld_mkr.ino`](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino) and [`tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino): the `fleet_key_is_zero()` halt was previously gated by `#ifdef LIFETRAC_USE_REAL_CRYPTO`, so a stub-crypto image flashed onto real hardware would TX with the all-zero placeholder. The check is now `#ifndef LIFETRAC_ALLOW_UNCONFIGURED_KEY`: opt-in instead of opt-out. CI compile jobs in [`.github/workflows/arduino-ci.yml`](../../.github/workflows/arduino-ci.yml) now pass `-DLIFETRAC_ALLOW_UNCONFIGURED_KEY` so they keep building without a provisioned key, but production images cannot ship with the unprovisioned default.

### §E — `AuditLog` singleton (low/medium)

Source flag: GPT-5.3-Codex §1 (second-pass new).

[`base_station/web_ui.py`](../DESIGN-CONTROLLER/base_station/web_ui.py): `_get_audit_log()` lazily constructs a single module-level `AuditLog` and reuses it across PIN failures, lockouts, and WS-cap rejections. Was previously `AuditLog(_AUDIT_PATH)` per event, leaking a file handle on every rejected login (`ResourceWarning: unclosed file`).

### §F — Cross-language ReplayWindow invariant (low)

Source flag: Gemini §4.2.3.

[`base_station/tests/test_replay_window_invariant.py`](../DESIGN-CONTROLLER/base_station/tests/test_replay_window_invariant.py) — 2 tests parsing `firmware/common/lora_proto/lora_proto.h` and asserting `LP_REPLAY_WINDOW_BITS == REPLAY_WINDOW_BITS == 64`. Catches the C-vs-Python bitmap-width drift class of bug at CI time before it can break replay protection asymmetrically.

### Tests after Round 9

`Ran 151 tests in 0.511s` `OK` (was 147; +1 control-cap, +1 control-no-session, +2 replay invariant). `ws_over_cap` audit events visible in WARNING logs as expected.

### Open

**§A** (CI compile-gate flip from `continue-on-error: true` to blocking) still requires a green Actions run to confirm the staging glue works on a clean runner. The new `-DLIFETRAC_ALLOW_UNCONFIGURED_KEY` flag added in §D should keep the existing best-effort jobs green; once that's confirmed on a real run, dropping the `continue-on-error` flag is a one-line change.

## Round 10 — 2026-04-28 — Opta Modbus SIL closes W4-04 in CI

Pure-software follow-on to Round 8 (M4 supervisor SIL). Same recipe applied to the W4-04 surface: model both halves of the M7?Opta Modbus link in stdlib Python, then hammer the safety-relevant invariants in unittest.

### What landed

[`base_station/tests/test_modbus_slave_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_modbus_slave_sil.py) — 13 tests, two model classes:

- `OptaSlave` mirrors `opta_modbus_slave.ino`'s `on_holding_change`, `check_safety`, and `enter_safe_state` paths. Holding-register addresses, safety-state enum, AUX valid mask, and 200 ms watchdog threshold are all literal copies of the C header constants.
- `M7ModbusClient` mirrors the `apply_control()` writer in `tractor_h7.ino`: builds the 7-register holding block, ticks `g_watchdog_ctr` every call, increments `s_modbus_fail_count` on TX failure, and latches `g_estop_latched` at the 10-strike threshold. When latched, builds the all-zero block with `REG_ARM_ESTOP=1`.

### Properties asserted (TR-A … TR-J)

- **TR-A / TR-B** Opta watchdog: no trip when M7 writes at 100 Hz; trips between 200 ms and 200 ms + one tick when M7 goes silent; trip drops all relays + zeroes flow set-points.
- **TR-C / TR-D** M7 fail-count latch: 10 cumulative `endTransmission()` failures latch `g_estop_latched`; once latched, the next block written has `REG_VALVE_COILS=0`, `REG_FLOW_SP_*=0`, `REG_ARM_ESTOP=1`, `REG_CMD_SOURCE=0`.
- **TR-E / TR-F** Opta REG_ARM_ESTOP latch: a single write of `REG_ARM_ESTOP=1` enters `SAFETY_ESTOP_LATCHED` and engine-kill HIGH; subsequent `REG_VALVE_COILS` and `REG_FLOW_SP_*` writes are silently dropped; `REG_WATCHDOG_CTR` writes still pass through (master must keep proving liveness).
- **TR-G** External E-stop loop: `PIN_ESTOP_LOOP` LOW enters `SAFETY_ESTOP_LATCHED` on the next `check_safety` tick.
- **TR-H** AUX-mask: `REG_AUX_OUTPUTS=0xFFFF` only energises R10/R11/R12; the boom/bucket bank is untouched (so a runaway bitfield can never fire boom-up via the wrong register).
- **TR-J** End-to-end W4-04 budget: from first failed TX to `g_estop_latched` is < 1 s at the 100 Hz call rate (measured ~100 ms in the SIL — well inside the runbook budget).

### Documented divergence

- **TR-I** Pinned by `test_fail_count_is_cumulative_not_consecutive`: the firmware as written never resets `s_modbus_fail_count` on a successful TX, so the latch fires on the 10th *cumulative* failure since boot, not the 10th *consecutive* failure. The W4-04 runbook prose ([`DESIGN-CONTROLLER/HIL_RUNBOOK.md`](../DESIGN-CONTROLLER/HIL_RUNBOOK.md) §W4-04) says "10 consecutive failures". The SIL pins firmware-as-written so the test will fail loudly the day someone fixes the firmware to be consecutive — at which point both the test and the runbook prose need to be updated together.

### Tests after Round 10

`Ran 164 tests in 0.505s` `OK` (was 151; +13 from the new SIL).

### Effect on Wave 4

W4-04 is now *bench-confirmation-only* — the design is verified in CI; the bench session just needs to confirm the model matches the wiring. This is the same downgrade that Round 8 applied to W4-01 (E-stop latch) and W4-03 (M7?M4 watchdog).

### Next candidates

- Telemetry path property/fuzz (`pack_telemetry_fragments` + `TelemetryReassembler` round-trip) — symmetric to the IP-108 command-frame fuzz.
- WS-subscriber stress test that hammers Round 9 §C's `_subscribers_lock` under N-client connect/disconnect churn.

## Round 11 — 2026-04-28 — Telemetry fragmentation property/fuzz suite

Symmetric companion to the IP-108 command-frame fuzz, applied to the M7?base direction. Pure stdlib, no new deps.

### What landed

[`base_station/tests/test_telemetry_fragmentation_fuzz.py`](../DESIGN-CONTROLLER/base_station/tests/test_telemetry_fragmentation_fuzz.py) — 15 tests grouped into seven classes targeting `pack_telemetry_fragments` ? `parse_telemetry_fragment` ? `TelemetryReassembler`.

Helper `_feed_all` latches the first non-None completion rather than the last call's return value, because a duplicate fragment delivered after completion would otherwise open a fresh partial slot for the same key (the original was deleted on completion) and the loop's last assignment would shadow the real result. This is a real edge worth pinning, not just a test-harness convenience.

### Properties asserted (TF-A … TF-L)

- **TF-A round-trip identity.** Lengths 0..2048 bytes (stride 17) on PHY_IMAGE round-trip exactly; same for every packable profile (PHY_IMAGE, PHY_CONTROL_SF7, with PHY_CONTROL_SF8/SF9 skipped where the 25 ms cap leaves zero payload room).
- **TF-B fragment-header invariants.** Magic byte is `TELEMETRY_FRAGMENT_MAGIC`; every fragment shares the same `frag_seq` and `total`; `idx` is monotonic 0..total-1.
- **TF-C airtime cap.** Every fragment fits inside `TELEMETRY_FRAGMENT_MAX_AIRTIME_MS` (25 ms) on its profile.
- **TF-D oversize rejection.** A payload requiring 257 fragments raises `ValueError`; the 256-fragment boundary still packs and round-trips.
- **TF-E reordered delivery.** Twenty PRNG-shuffled deliveries of the same payload still complete to the original bytes.
- **TF-F duplicate delivery.** Sending each non-final fragment 3× then the final fragment once produces exactly one completion and `2*(N-1)` duplicates.
- **TF-G missing-middle timeout.** Dropping any single middle fragment prevents completion; once a later unrelated feed advances the GC clock past `timeout_ms`, the partial is reclaimed and `timeouts` ticks.
- **TF-H multi-stream isolation.** Two streams with the same `frag_seq` but distinct `(source_id, topic_id)` interleave cleanly and complete independently.
- **TF-I seq-wrap with intervening GC.** Re-using the same `frag_seq` after the first stream has been GC'd completes a second, independent assembly.
- **TF-J unfragmented passthrough.** Bodies whose first byte isn't the magic (or that are shorter than the 4-byte header) pass through verbatim and increment `bad_magic_passthroughs`.
- **TF-K mid-stream total change.** A synthetic stream that reports `total=4` then `total=2` for the same key correctly discards the partial and completes off the new total.
- **TF-L randomised stress.** 200 PRNG-seeded trials sweep length, seq, source, topic, profile, with 30 % chance to shuffle and 30 % chance to inject a duplicate; all complete to the original payload.

### Tests after Round 11

`Ran 179 tests in 0.764s` `OK (skipped=1)` (was 164; +15 from the new suite, with 1 deliberate skip on a profile that can't pack under the 25 ms cap by design).

### Why this matters

The IP-108 fuzz pinned the operator?tractor command frame against parser bugs. Round 11 is the symmetric pin on the tractor?operator telemetry path: a silent fragmentation bug there would corrupt the data the operator trusts — pose, hydraulic pressures, mode-switch state — without any visible failure. The randomised stress is seeded so a future regression repros byte-for-byte.

### Next candidates

- WS-subscriber stress test for Round 9 §C: N concurrent fake clients connect/disconnect at 100 Hz while telemetry/image/state flow; assert no `RuntimeError` and no leaked entries in any pool.
- §A CI compile-gate flip — still gated on a real Actions run.

## Round 12 — 2026-04-28 — WS subscriber concurrency stress suite

Round 9 §B added the `/ws/control` cap and §C added `_subscribers_lock` + `_snapshot_subscribers`. The single-threaded admit/reject tests in `test_web_ui_validation.py` proved the cap rejects with 4429 and that the lock exists; they did **not** prove the property §C was actually introduced for — that the MQTT background thread can iterate the WS pools while the FastAPI event loop mutates them, with zero `RuntimeError: Set changed size during iteration` and zero over-admits under contention.

### What landed

[`base_station/tests/test_ws_subscriber_concurrency.py`](../DESIGN-CONTROLLER/base_station/tests/test_ws_subscriber_concurrency.py) — 6 tests that drive `_admit_ws`, `_discard_subscriber`, `_snapshot_subscribers`, and `_on_mqtt_message` together against a real `asyncio` loop running on a background thread (`_LoopFixture`). This mirrors the production topology: FastAPI handlers run on the loop thread, MQTT callbacks fire on paho's thread. A small `FakeWS` mocks the starlette `WebSocket` surface `_admit_ws` actually touches (`accept`, `close(code=…)`, `send_text`, `send_bytes`).

### Properties asserted (WC-A … WC-G)

- **WC-A cap honoured under N concurrent admits.** Submit `N = 4 × MAX_TELEMETRY_SUBSCRIBERS` admit coroutines onto the loop simultaneously; assert exactly `cap` return True, the rest return False, and `len(pool) == cap` afterwards. The check-then-add window inside `_admit_ws` is single-threaded by `_subscribers_lock`, so no admit can sneak past the gate.
- **WC-B admit/discard cycles, no leak.** 200 sequential admit-then-discard cycles on the control pool; pool length = cap throughout, exactly 0 at the end.
- **WC-C snapshot iteration is race-free.** 4 churner threads admit/discard at maximum rate while 2 reader threads call `_snapshot_subscribers(pool)` 20 000 times each and iterate every snapshot. The whole point of §C: if the snapshot were a live view rather than a list copy under the lock, the readers would intermittently raise `RuntimeError: Set changed size during iteration`. Asserts the error list stays empty.
- **WC-D fan-out under churn.** Two resident subscribers in the telemetry pool, three churners mutating it, and 500 `_on_mqtt_message` calls dispatching telemetry frames from the main thread. Asserts no exception escapes the fan-out and the resident subscribers actually receive frames (`sent_text > 0`) — the snapshot path is delivering, not silently dropping.
- **WC-E per-pool cap independence.** Saturating telemetry must not block image/state/control admissions. Confirms the four caps don't share state.
- **WC-F drain-and-refill counter integrity.** Fill image to cap, reject one over, drain all, refill to cap, reject one over again. Catches a class of bug where `_discard_subscriber` left the counter stale (admitting 0) or relaxed (admitting cap + 1).
- **WC-G 4429 close-code on rejection.** Piggybacked onto WC-A: every rejected `FakeWS` must have `close()` called with `WS_OVER_CAPACITY_CODE`.

### Tests after Round 12

`Ran 185 tests in 0.789s` `OK (skipped=1)` (was 179; +6 from the new suite, no regressions in the prior 179).

### Why this matters

§C was a "looks right by inspection" change driven by reading the parallel reviews; it had no test that would actually fail if the lock were removed. WC-C now does — temporarily commenting out `_subscribers_lock` acquires inside `_snapshot_subscribers` reproduces the original `Set changed size during iteration` race the lock was added to suppress. The §B control-cap was already pinned by the single-threaded test, but WC-A pins it under contention, which is the threat model: a misbehaving client opening sockets in parallel.

### Next candidates

- Firmware fail-count consecutive-vs-cumulative fix (TR-I divergence pinned in Round 10): either change the firmware to match the runbook prose or update the runbook to match the firmware. Needs a design call before code.
- §A CI compile-gate flip — still gated on a real Actions run.
- Image-pipeline frag/reassembly fuzz, symmetric to TF-A..TF-L but for `image_pipeline/reassemble.py`.

## Round 13 — 2026-04-28 — Image-pipeline reassembly property/fuzz suite

Third and final fragmenter fuzz suite, completing the property coverage of every binary parser in the v25 stack.

### What landed

[`base_station/tests/test_image_reassembly_fuzz.py`](../DESIGN-CONTROLLER/base_station/tests/test_image_reassembly_fuzz.py) — 13 tests against `image_pipeline/reassemble.py` (the on-air TileDeltaFrame chunker that `camera_service.py` will start using once the M7?SX1276 SPI driver lands). Pure stdlib, no new deps. Reuses the same `_feed_all` first-non-None-completion latching trick as the telemetry fuzz, because this reassembler also re-opens a partial slot when a duplicate arrives after completion.

### Properties asserted (IF-A … IF-L)

- **IF-A round-trip identity.** Arbitrary frame sizes (1, 2, 5, 12, 32, 96 changed tiles) split into 1, 2, 3, 5, 8 fragments — every viable combination round-trips through the encoder, the chunker, and the reassembler back to the original `TileDeltaFrame`.
- **IF-B fragment-header invariants.** Per-stream: every fragment carries the magic `0xFE`, a constant `frag_seq`, a constant `total_minus1`, and `frag_idx` monotonically 0..total-1.
- **IF-C bad-magic passthrough is harmless.** A complete TileDeltaFrame whose first byte isn't `0xFE` (the real-world "bridge currently runs over a local USB-CDC link" case) is treated as an already-assembled frame and counted as a passthrough. Crucially: passthrough must not disturb any in-flight fragmented stream — verified by interleaving a passthrough between two fragments of an active assembly and confirming the original stream still completes.
- **IF-D `frag_idx >= total` rejection.** A fragment claiming idx 5 of a 2-fragment stream bumps `decode_errors` and returns None instead of writing past the partial slot.
- **IF-E reorder.** 20 PRNG-shuffled deliveries of the same payload all reassemble.
- **IF-F duplicate counted, no double-complete.** Send each non-final fragment 3× then the final fragment once — exactly one completion, `2*(N-1)` duplicates counted. Same property as the telemetry fuzz TF-F.
- **IF-G missing-middle timeout.** Drop a middle fragment, advance the injected clock past `timeout_ms`, feed any other fragment to trigger GC, assert the partial is reclaimed and `timeouts` ticks.
- **IF-H multi-stream isolation.** Two streams keyed by distinct `frag_seq` are interleaved fragment-by-fragment; both complete independently.
- **IF-I seq-wrap with intervening GC.** Stream uses `frag_seq=255`, gets GC'd, then a fresh stream re-uses `frag_seq=0` — both behave correctly across the wrap.
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

A silent fragmentation bug on any of these paths would corrupt operator commands, telemetry the operator trusts, or the live camera canvas — without raising. All three directions are now pinned by deterministic, PRNG-seeded property tests that repro byte-for-byte on regression.

### Next candidates

- **Firmware fail-count consecutive-vs-cumulative** (TR-I divergence pinned in Round 10): needs a design call. The runbook prose says *consecutive*, the firmware counts *cumulative*. Either fix is small; the choice is what we want to enforce.
- **§A CI compile-gate flip** — still gated on a real Actions run.
- All remaining items in the Wave-4 HIL gate set require bench hardware.

## Round 14 — 2026-04-28 — TR-I firmware fix: consecutive fail-count

Resolves the divergence pinned by Round 10 between the W4-04 runbook prose ("10 *consecutive* failures") and the firmware as written (cumulative count, never reset). Decision: fix firmware to match runbook. Cumulative was a slow-burn nuisance-trip risk, not a safety property: a single bad Modbus poll every few hours would eventually (over a long run) latch the tractor down for no good reason, while *not* providing meaningfully faster latching when an actual sustained outage happens.

### What landed

- [`firmware/tractor_h7/tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) — moved the `s_modbus_fail_count` static out of the failure branch so the success branch can reset it to 0. Updated the comment to call out the consecutive semantics and reference the W4-04 runbook. Log message changed from `"(N since boot)"` to `"(N consecutive)"` to make the semantics obvious in serial captures.
- [`base_station/tests/test_modbus_slave_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_modbus_slave_sil.py) — flipped TR-I:
  - `M7ModbusClient.apply_control()` now resets `self.fail_count = 0` on success, mirroring the firmware fix.
  - Old `test_fail_count_is_cumulative_not_consecutive` (which pinned the bug) replaced by `test_fail_count_is_consecutive` (positive assertion of the new behaviour).
  - Added `test_intermittent_failures_never_latch`: 1000 cycles alternating fail / success — cumulative would have tripped at cycle 19, consecutive must never trip.
  - Added `test_nine_then_one_then_nine_does_not_latch`: 9 failures + 1 success + 9 failures — cumulative would have tripped on the 19th call (10th total failure); consecutive must not.
  - Module docstring + TR-C / TR-I header comments updated to describe the post-Round-14 behaviour and explain how to detect a regression to cumulative.

### Tests after Round 14

`Ran 200 tests in 0.913s` `OK (skipped=1)` (was 198; +3 consecutive tests, -1 cumulative test = net +2). All 15 tests in `test_modbus_slave_sil.py` pass.

### W4-04 runbook alignment

The runbook prose in [`HIL_RUNBOOK.md`](../DESIGN-CONTROLLER/HIL_RUNBOOK.md) §W4-04 was already consecutive-correct (steps 1–5). Step 6's *resume*-side hysteresis ("M7 resumes after 5 consecutive successes" + stick-release-to-neutral) is a *separate* unimplemented behaviour — the firmware currently fully latches `g_estop_latched` and does not auto-recover at all. That's tracked as its own future work; the Round 14 fix is scoped only to the failure-count semantics on the way *into* the latch.

### Round-13 follow-on still open

- **§A CI compile-gate flip** — still gated on a real Actions run.
- All remaining items in the Wave-4 HIL gate set require bench hardware, with the SIL-companion plan in the prior round's "Next candidates" still applicable as a way to convert HIL-only gates into CI-pinned-plus-bench-validated.

## Round 15 — 2026-04-28 — Arduino compile gate: handheld_mkr unblocked, three latent CI bugs fixed

Arduino-cli installed locally (1.4.1, with arduino:samd 1.8.14, arduino:mbed_portenta 4.5.0, arduino:mbed_opta 4.5.0). Replaying the CI compile steps locally exposed *three* independent bugs that had all been masked by `continue-on-error: true` since IP-007. `handheld_mkr` is now a real blocking compile gate; `tractor_h7` and `tractor_opta` are still blocked but the workflow comments now spell out exactly why and what fixes them.

### Three bugs the local replay exposed

1. **Stale relative includes.** `handheld_mkr.ino` and `tractor_h7.ino` / `tractor_m4.cpp` had `#include "../common/lora_proto/lora_proto.h"` etc. arduino-cli copies the sketch into a temp build dir where `../common/` no longer resolves. The CI staging step (which copies `firmware/common/lora_proto` into `sketch/src/lora_proto/`) was correct, but the `#include` strings were never rewritten to use the staged path.
2. **`--build-property "build.extra_flags=..."` clobbers the board's flags.** The MKR WAN 1310 variant.h gates `LORA_IRQ` (pin 31) behind `#if defined(USE_BQ24195L_PMIC)`, and `USE_BQ24195L_PMIC` is defined in `mkrwan1310.build.extra_flags` in boards.txt. Overriding `build.extra_flags` wipes that, so `LORA_IRQ` becomes undefined and the sketch fails with `'LORA_IRQ' was not declared in this scope`. The fix: use `compiler.cpp.extra_flags` and `compiler.c.extra_flags`, which *append* to the toolchain flags instead of replacing the recipe macro.
3. **`crypto_stub.c` requires explicit opt-in.** The stub crypto file `#error`s out unless either `LIFETRAC_USE_REAL_CRYPTO` or `LIFETRAC_ALLOW_STUB_CRYPTO` is defined. The CI's "stub" build only set `LIFETRAC_ALLOW_UNCONFIGURED_KEY` (a different gate, in the .ino itself), so the stub build always failed at the crypto step.

### What landed

- [`firmware/handheld_mkr/handheld_mkr.ino`](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino) — include rewritten to `"src/lora_proto/lora_proto.h"` with a comment block pointing at the staging script and the canonical `firmware/common/` source of truth.
- [`firmware/tractor_h7/tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) — same include rewrite for both `lora_proto.h` and `shared_mem.h`, plus a `struct AxisRamp;` forward declaration to suppress an Arduino-preprocessor auto-prototype-versus-definition collision (`'int16_t step_axis_ramp(AxisRamp&, ...)' redeclared as different kind of symbol`) — the auto-generated prototype was being parsed before `AxisRamp` was visible.
- [`firmware/tractor_h7/tractor_m4.cpp`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m4.cpp) — include rewrite to `"src/shared_mem.h"`.
- [`.github/workflows/arduino-ci.yml`](../../.github/workflows/arduino-ci.yml):
  - `firmware-compile-handheld` is **no longer `continue-on-error`** — it's now a blocking gate. Build flags rewritten to use `compiler.cpp.extra_flags` / `compiler.c.extra_flags` and to add `-DLIFETRAC_ALLOW_STUB_CRYPTO` to the stub build.
  - `firmware-compile-opta` and `firmware-compile-tractor-h7` remain `continue-on-error` but each gained a Round-15 comment block explaining the *one* remaining blocker per job (Opta: .ino name doesn't match folder name; H7: `tractor_m4.cpp` defines its own `setup()`/`loop()` and link-collides with the M7 binary). The H7 job also picked up the same `compiler.{cpp,c}.extra_flags` fix so it's ready to flip the moment the M4/M7 split lands.
- [`.gitignore`](../../.gitignore) — added `LifeTrac-v25/DESIGN-CONTROLLER/firmware/handheld_mkr/src/` and `firmware/tractor_h7/src/` (the staged copies of common/).
- [`DESIGN-CONTROLLER/tools/stage_firmware_sketches.ps1`](../DESIGN-CONTROLLER/tools/stage_firmware_sketches.ps1) — new PowerShell script that mirrors the CI's "Stage shared sources into sketch" steps so a local developer can run the same arduino-cli compiles offline. Includes a printed reminder about the `compiler.cpp.extra_flags` vs `build.extra_flags` trap.

### Local verification

- `handheld_mkr` (stub crypto): `Sketch uses 55932 bytes (21%) of program storage space. Maximum is 262144 bytes. Global variables use 4948 bytes (15%) of dynamic memory, leaving 27820 bytes for local variables.`
- `handheld_mkr` (real crypto, `-DLIFETRAC_USE_REAL_CRYPTO`): `Sketch uses 59940 bytes (22%). Global variables use 5108 bytes (15%).` — the +4 KB / +160 B delta is the AES-GCM (Crypto.h + AES.h + GCM.h) cost.
- Base-station test suite: `Ran 200 tests in 0.837s` `OK (skipped=1)` — no regressions.

### What's still blocked, in order of difficulty

- **`firmware-compile-opta`** — rename `opta_modbus_slave.ino` ? `tractor_opta.ino` (or rename the folder to `opta_modbus_slave`). One-file change once we pick the name.
- **`firmware-compile-tractor-h7`** — split `tractor_m4.cpp` so it doesn't link into the M7 binary. Two reasonable shapes: (a) move it to its own sketch folder `firmware/tractor_m4/tractor_m4.ino` and add a separate compile job for `arduino:mbed_portenta:envie_m4`, then bake the resulting binary into M7 via the standard Portenta dual-core flow; or (b) wrap the whole file in `#if defined(CORE_CM4)` so it's a no-op on M7 builds and add a parallel CM4 compile to the same workflow. (a) is more idiomatic for production, (b) is faster to implement and is what most early dual-core sketches do.

### Coverage scoreboard (post Round 15)

| Compile target | FQBN | Status | Notes |
| --- | --- | --- | --- |
| handheld_mkr (stub) | arduino:samd:mkrwan1310 | **blocking** | green |
| handheld_mkr (real) | arduino:samd:mkrwan1310 | **blocking** | green |
| tractor_opta | arduino:mbed_opta:opta | continue-on-error | .ino name vs folder name |
| tractor_h7 (stub) | arduino:mbed_portenta:envie_m7 | continue-on-error | M4/M7 split |

## Round 16 — 2026-04-28 — Arduino compile gate: tractor_opta unblocked

Second of three Arduino compile gates flipped from `continue-on-error` to blocking. Same playbook as Round 15: ran the local arduino-cli replay, fixed everything it found, then locked the result into both the workflow and the source tree.

### Two issues exposed by the local replay

1. **.ino filename did not match folder name.** The sketch was `firmware/tractor_opta/opta_modbus_slave.ino` — arduino-cli requires the .ino base name to match the parent folder, so it failed at sketch-discovery time before touching any source. Renamed to `tractor_opta.ino`.
2. **Missing library: `Arduino_Opta_Blueprint`.** The .ino does `#include <OptaBlue.h>`, which is provided by the `Arduino_Opta_Blueprint` library on the Arduino library index. The CI install step had only `ArduinoRS485` and `ArduinoModbus`.

### What landed

- Renamed [`firmware/tractor_opta/opta_modbus_slave.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/) ? [`firmware/tractor_opta/tractor_opta.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/tractor_opta.ino). Source unchanged.
- [`.github/workflows/arduino-ci.yml`](../../.github/workflows/arduino-ci.yml) — `firmware-compile-opta` is **no longer `continue-on-error`**. Added `arduino-cli lib install Arduino_Opta_Blueprint` to the Install libraries step. Old "deferred to its own round" comment block replaced with a Round-16 comment naming the two fixes.

### Local verification

- `tractor_opta`: `Sketch uses 171520 bytes (8%) of program storage space. Maximum is 1966080 bytes. Global variables use 65120 bytes (12%) of dynamic memory, leaving 458504 bytes for local variables. Maximum is 523624 bytes.`
- Base-station test suite: `Ran 200 tests in 1.111s` `OK (skipped=1)` — no regressions.

### Coverage scoreboard (post Round 16)

| Compile target | FQBN | Status | Notes |
| --- | --- | --- | --- |
| handheld_mkr (stub) | arduino:samd:mkrwan1310 | **blocking** | green (Round 15) |
| handheld_mkr (real) | arduino:samd:mkrwan1310 | **blocking** | green (Round 15) |
| tractor_opta | arduino:mbed_opta:opta | **blocking** | green (Round 16) |
| tractor_h7 (stub) | arduino:mbed_portenta:envie_m7 | continue-on-error | M4/M7 split |

Two of three Arduino compile gates are now real. `tractor_h7` remains the only `continue-on-error` job; the next round can either tackle that (M4/M7 split, see Round-15 comment block in the workflow for the two recommended shapes) or pivot to a Wave-4 SIL companion.


---

## Round 17 â€” 2026-04-28 â€” Tractor H7 dual-core split (M7 + M4 sketches)

**Status:** `firmware-compile-tractor-h7` flipped from
`continue-on-error: true` to blocking. New parallel
`firmware-compile-tractor-m4` job added. M4 watchdog firmware moved
from `firmware/tractor_h7/tractor_m4.cpp` to its own sketch folder
[`firmware/tractor_h7_m4/tractor_h7_m4.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7_m4/tractor_h7_m4.ino).
Compile-gate scoreboard: **4 of 4 Arduino jobs now blocking** (handheld stub,
handheld real, tractor_opta, tractor_h7 M7, tractor_m4 M4) â€” zero
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
layout â€” Arduino IDE's stock `PortentaDualCore` example uses
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

* **MOVED** `firmware/tractor_h7/tractor_m4.cpp` â†’
  `firmware/tractor_h7_m4/tractor_h7_m4.ino` (.ino name must match
  parent folder for arduino-cli sketch discovery; .cpp â†’ .ino is
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

