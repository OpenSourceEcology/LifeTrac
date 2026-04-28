# LifeTrac v25 — TODO

> **⚡ Controller architecture update (2026):** The primary controller design
> is now the three-tier Portenta Max Carrier + MKR WAN 1310 system documented
> in [DESIGN-CONTROLLER/ARCHITECTURE.md](DESIGN-CONTROLLER/ARCHITECTURE.md).
> All hardware-purchase, firmware, and bring-up tasks for that design live in
> [DESIGN-CONTROLLER/TODO.md](DESIGN-CONTROLLER/TODO.md).
>
> The Opta / ESP32 / Raspberry Pi prototype code referenced below has been
> moved to
> [DESIGN-CONTROLLER/RESEARCH-CONTROLLER/](DESIGN-CONTROLLER/RESEARCH-CONTROLLER/)
> (see its [README](DESIGN-CONTROLLER/RESEARCH-CONTROLLER/README.md)).
> The safety bug-fixes listed in this file are still worth completing on
> the prototype code if it gets used for any further bench tests, but the
> equivalent safety logic must be implemented from the start in the new
> Portenta firmware (tracked in [DESIGN-CONTROLLER/TODO.md § Phase 4](DESIGN-CONTROLLER/TODO.md#phase-4--tractor-firmware)).
> Path links below refer to the historical locations; prepend
> `DESIGN-CONTROLLER/RESEARCH-CONTROLLER/` for the current location of any
> Opta / ESP32 / `raspberry_pi_web_controller` files.

## 2026-04-28 controller code-review implementation plan

Four code reviews of the DESIGN-CONTROLLER stack (Claude Opus 4.7, GitHub
Copilot v1.0, GPT-5.3-Codex v1.0, Gemini 3.1 Pro) have been merged into a
single actionable plan with stable IDs (`IP-001` … `IP-309`) and severity
tags. **All work tracked there:**

➡️ **[AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md)**

**Implementation status (2026-04-28, through Round 7):**
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md`](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md)
— every plan item achievable without bench hardware is now landed
(Wave 0 8/8, Wave 1 8/8, Wave 2 9/9, Wave 3 9/9). 131 base_station
tests pass. **Round 7** ported the IP-107 SIL queue into M7 firmware:
`emit_topic()` / `send_link_tune()` now enqueue into a 4-deep priority
queue; `tx_pump()` runs every `loop()` iteration via
`startTransmit()` + estimated-TOA polling. The headline IP-107 invariant
— M7 never blocks against the M4 watchdog — is now structural in
`tractor_h7.ino` instead of a timing-dependent mitigation. CI also runs
best-effort `arduino-cli compile` for handheld_mkr / tractor_opta /
tractor_h7. Remaining work is the HIL gate set below — bench validation
of W4-01…W4-10 and the CI compile-gate flip from best-effort to blocking.

Source reviews:

- [Claude Opus 4.7 (primary, with cross-review addendum)](AI%20NOTES/2026-04-28_Controller_Code_Pipeline_Review_ClaudeOpus4_7_v1_0.md)
- [GitHub Copilot v1.0](AI%20NOTES/2026-04-28_Controller_Code_Pipeline_Review_GitHub_Copilot_v1_0.md)
- [GPT-5.3-Codex v1.0](AI%20NOTES/2026-04-28_Controller_Code_Pipeline_Review_GPT-5.3-Codex_v1_0.md)
- [Gemini 3.1 Pro v1.0](AI%20NOTES/2026-04-28_Controller_Code_Review_Gemini_3.1_Pro_v1.0.md)

Wave 0 (IP-001 … IP-008) is the BLOCKER set — nothing else is testable
until those land. Wave 4 lists the gates that must pass before any wet
hydraulic test.

## Current sprint status — DESIGN-CONTROLLER LoRa stack (2026-04-27)

Tracked in detail in [DESIGN-CONTROLLER/TODO.md](DESIGN-CONTROLLER/TODO.md) and
[DESIGN-CONTROLLER/DECISIONS.md](DESIGN-CONTROLLER/DECISIONS.md). Snapshot:

### Done
- [x] Handheld firmware: full RX, KISS, per-source replay, `CMD_LINK_TUNE` /
  `CMD_ESTOP` / `CMD_CLEAR_ESTOP` ingest, OLED status, latching mushroom E-stop
  ([firmware/handheld_mkr/handheld_mkr.ino](DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino)).
- [x] Base-station audit log
  ([base_station/audit_log.py](DESIGN-CONTROLLER/base_station/audit_log.py))
  + bridge wiring (rx/tx/tx_error/gcm_tag_reject/bad_header/replay_reject/
  link_tune/encode_mode_change/airtime_alarm) + per-source ReplayWindow.
- [x] DECISIONS.md option tables for D-A2 / D-A3 / D-C1 / D-C2 / D-C6 / D-E1.
- [x] **D-A2 control PHY → SF7/BW250** (~46 ms encrypted ControlFrame, fits
  20 Hz cadence). Updated in C, Python, both LADDER tables, LORA_PROTOCOL.md.
- [x] **D-A3 image PHY → SF7/BW500** (~18 ms 32 B fragment, fits 25 ms cap).
- [x] **D-C1/C2 real-crypto scaffold**:
  [lp_crypto_real.cpp](DESIGN-CONTROLLER/firmware/common/lora_proto/lp_crypto_real.cpp)
  with MbedTLS (Portenta H7) + rweather/Crypto (MKR) backends, gated on
  `-DLIFETRAC_USE_REAL_CRYPTO`; `crypto_stub.c` re-guarded so it skips when
  real crypto wins.
- [x] **D-C6 CSMA scanChannel** wired into handheld TX under
  `#ifdef LIFETRAC_FHSS_ENABLED`.
- [x] **D-E1 web UI PIN auth**: `LIFETRAC_PIN`, HttpOnly+SameSite=strict
  cookie, 30 min idle TTL, 5-fail/60 s IP lockout, WS cookie-check, new
  `/api/login` `/api/logout` `/api/session` routes
  ([base_station/web_ui.py](DESIGN-CONTROLLER/base_station/web_ui.py),
  [tests/test_web_ui_auth.py](DESIGN-CONTROLLER/base_station/tests/test_web_ui_auth.py)).
- [x] Tractor X8 service scaffolds:
  [time_service.py](DESIGN-CONTROLLER/firmware/tractor_x8/time_service.py),
  [params_service.py](DESIGN-CONTROLLER/firmware/tractor_x8/params_service.py),
  [logger_service.py](DESIGN-CONTROLLER/firmware/tractor_x8/logger_service.py).
- [x] [DESIGN-CONTROLLER/KEY_ROTATION.md](DESIGN-CONTROLLER/KEY_ROTATION.md)
  operator procedure.
- [x] arduino_libraries.txt: + Adafruit SSD1306/GFX + rweather Crypto.
- [x] All 26 base_station unit tests pass (1 env-skip when fastapi/paho
  missing).
- [x] **Mirror handheld CSMA #ifdef into the tractor M7 TX path** —
  shared `csma_pick_hop_before_tx()` helper called from both
  `emit_topic()` and `send_link_tune()` under `LIFETRAC_FHSS_ENABLED`
  ([tractor_h7.ino](DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)).
- [x] **`g_fhss_hop_counter` / `g_fhss_key_id` globals** defined on M7 and
  handheld (file-scope, not static, so a future shared header can re-extern
  them). Initialised to 0; epoch-rotated by `time_service.py` once the
  X8↔H747 UART tick lands.
- [x] **D6 source-active telemetry enrichment**: `emit_source_active()`
  payload extended from 8 → 20 bytes, now carries per-source RSSI (int16)
  + SNR×10 (int16) for `[HANDHELD, BASE, AUTONOMY]` alongside the
  existing source/rung/estop/pending/tune-failures fields. SourceState
  gained `snr_db_x10`; `process_air_frame` captures `radio.getSNR()` per RX.
- [x] **Web operator HTML/JS** PIN-entry page +
  [`/login`](DESIGN-CONTROLLER/base_station/web/login.html) route on
  `web_ui.py`; `/` now redirects to `/login` when no valid session.
  The operator console (joysticks, telemetry sidebar, camera selector,
  E-stop, gamepad support) was already in place.
- [x] **Real-crypto golden vectors**:
  [vectors.json](DESIGN-CONTROLLER/firmware/bench/crypto_vectors/vectors.json)
  + Python regression
  [test_crypto_vectors.py](DESIGN-CONTROLLER/base_station/tests/test_crypto_vectors.py)
  + host-build cross-check
  [host_check.c + Makefile](DESIGN-CONTROLLER/firmware/bench/crypto_vectors/)
  (`make crypto-check`, needs `libmbedtls-dev`).
- [x] **Tractor X8 service follow-through**: real
  [time_service.py](DESIGN-CONTROLLER/firmware/tractor_x8/time_service.py)
  (PPS_FETCH ioctl + UART tick + wall-clock fallback),
  [params_service.py](DESIGN-CONTROLLER/firmware/tractor_x8/params_service.py)
  (atomic JSON store + FastAPI sub-app + MQTT publish-on-change), and
  [logger_service.py](DESIGN-CONTROLLER/firmware/tractor_x8/logger_service.py)
  (paho subscriber + queued SQLite writer + JSONL audit).
- [x] **`tools/provision.py`** gained a `--write-port` USB-CDC mode
  implementing the wire protocol from KEY_ROTATION.md (prologue → KEY:
  → COMMIT). Header-generation mode unchanged.
- [x] All 39 base_station unit tests pass (2 env-skips: fastapi + paho
  optional; cryptography optional for the golden-vector test).
- [x] **Phase 5 Docker / compose stack**:
  [Dockerfile](DESIGN-CONTROLLER/Dockerfile),
  [docker-compose.yml](DESIGN-CONTROLLER/docker-compose.yml),
  [base_station/mosquitto.conf](DESIGN-CONTROLLER/base_station/mosquitto.conf),
  [.env.example](DESIGN-CONTROLLER/.env.example), and
  [systemd units](DESIGN-CONTROLLER/base_station/systemd/) for the
  base + tractor X8 services. `audit_log.py` gained a `--tail-mqtt`
  CLI mode used by the `audit_tail` compose service.
- [x] **Tractor M4 core hardened**:
  [firmware/common/shared_mem.h](DESIGN-CONTROLLER/firmware/common/shared_mem.h)
  formalises the M7↔M4 SRAM4 layout (version + alive_tick + loop_counter
  + estop_request); rewrote
  [tractor_m4.cpp](DESIGN-CONTROLLER/firmware/tractor_h7/tractor_m4.cpp)
  with three independent trip conditions (stale tick, stuck loop counter,
  M7 e-stop request) + heartbeat LED. M7 stamps version/loop_counter/
  estop_request every iteration.
- [x] **Opta expansion shims replaced with real `OptaBlue` library
  calls** (digitalWrite via `DigitalExpansion`, analog out/in via
  `AnalogExpansion`); `OptaController.begin()`/`update()` wired into
  `setup()`/`loop()`.
- [x] **Web UI tractor-params proxy** —
  `web_ui` subscribes to `lifetrac/v25/params/changed`, exposes
  GET/POST `/api/params`, publishes patches on `lifetrac/v25/params/set`;
  `params_service` listens on the same topic and applies. Settings page
  gained a JSON params editor.
- [x] **Tractor X8 image pipeline encoder** —
  [camera_service.py](DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py)
  with libcamera + synthetic backends, 12×8 tile diff, WebP per tile
  with quality back-off, I/P frames + CMD_REQ_KEYFRAME subscription;
  publishes ready-to-fragment payloads on `lifetrac/v25/cmd/image_frame`.
- [x] **Web UI image canvas + audit viewer** — `<canvas id="image-canvas">`
  in `index.html`; `/ws/image` binary WS in `web_ui.py` forwarding the
  `lifetrac/v25/video/canvas` topic; new
  [`web/audit.html`](DESIGN-CONTROLLER/base_station/web/audit.html) page
  + `/api/audit` route reading the rotating JSONL tail with filter and
  auto-refresh.
- [x] **Cross-cutting docs**:
  [SAFETY_CASE.md](DESIGN-CONTROLLER/SAFETY_CASE.md) (ISO 25119 hazard
  table + AgPL targets), [CYBERSECURITY_CASE.md](DESIGN-CONTROLLER/CYBERSECURITY_CASE.md)
  (IEC 62443 SL-1 sketch + STRIDE per zone),
  [OTA_STRATEGY.md](DESIGN-CONTROLLER/OTA_STRATEGY.md) (signed image
  flow + A/B + 60 s health-check rollback),
  [PAIRING_PROCEDURE.md](DESIGN-CONTROLLER/PAIRING_PROCEDURE.md)
  (initial provision, field re-pair, decommission).
- [x] **Tooling**: [tools/keygen.py](DESIGN-CONTROLLER/tools/keygen.py)
  (offline 16/32 B fleet-key generator) and
  [tools/replay_pcap.py](DESIGN-CONTROLLER/tools/replay_pcap.py)
  (offline GCM + replay-window validator over a hex capture file).

### Remaining — code (no hardware needed)

The 2026-04-27 backlog sweep against
[DESIGN-CONTROLLER/TODO.md](DESIGN-CONTROLLER/TODO.md) Phase 5A/5A.B/5B/5C +
Cross-cutting + Tractor image pipeline identified six pure-code buckets
(A-F). **All six are now code-complete; G remains intentionally skipped.**
71/71 base_station unit tests pass (`python -m unittest discover tests`).

**A — Base-station image pipeline (Python, `base_station/image_pipeline/`):** ✅ Done

- [x] `reassemble.py` — collect TileDeltaFrame (topic `0x25`) fragments,
  time out missing fragments, surface stale-tile bitmap.
- [x] `canvas.py` — persistent tile canvas; on `base_seq` mismatch publish
  `CMD_REQ_KEYFRAME` (opcode `0x62`); attach badge enum to every published
  tile.
- [x] `bg_cache.py` — rolling per-tile median; fills missed tiles with
  `Cached` badge + age.
- [x] `state_publisher.py` — authoritative WS publisher (canvas tiles +
  per-tile age + badge + detections + safety verdicts + accel status).
- [x] `fallback_render.py` — server-side 1 fps composite for HDMI console
  + headless QA.
- [x] `recolourise.py` — Y-only luma + 30 s colour reference (scheme Z),
  sets `Recolourised` badge.
- [x] `motion_replay.py` — applies `0x28` motion vectors to canvas, sets
  `Predicted` badge (Q degraded mode).
- [x] `wireframe_render.py` — renders `0x29` PiDiNet wireframe overlay,
  sets `Wireframe` badge (P extreme degraded mode).
- [x] `link_monitor.py` `LinkMonitor` orchestrator with on_air()/tick(),
  publish_command/publish_status callbacks, and 3-window hysteresis ladder.

**B — Browser image-tier modules (`base_station/web/img/`):** ✅ Done

- [x] `canvas_renderer.js` — WS subscriber; per-tile blits via OffscreenCanvas
  worker pattern; dispatches `lifetrac-state` + `lifetrac-tile-painted` events.
- [x] `fade_shader.js` — 3-frame alpha pulse on `#image-fade` overlay.
- [x] `staleness_overlay.js` — yellow tint scaling with server-supplied
  `age_ms` (1 s threshold, max at 5 s).
- [x] `badge_renderer.js` — fail-closed badge enforcement; refusals POST
  to `/api/health/refusal`.
- [x] `detection_overlay.js` — bbox rendering by class colour; toggles
  `#detector-disagree` banner.
- [x] `accel_status.js` — fixed-position pill with 4 status colours.
- [x] `raw_mode_toggle.js` — body.raw-mode CSS class toggle, persisted in
  localStorage; choice POSTs to `/api/audit/view_mode`.
- [x] `web_ui.py` wired with `/ws/state` + `/api/health/refusal` +
  `/api/audit/view_mode`; tile_delta MQTT topics dispatched into
  Canvas + StatePublisher singletons.

**C — Tractor X8 image pipeline (split out of monolithic `camera_service.py`):** ✅ Done

- [x] `firmware/tractor_x8/image_pipeline/register.py` — phase-correlation
  pre-diff (NumPy + optional OpenCV NEON path).
- [x] `firmware/tractor_x8/image_pipeline/roi.py` — ROI mask from valve
  activity + `CMD_ROI_HINT` (opcode `0x61`) honour.
- [x] `firmware/tractor_x8/image_pipeline/encode_motion.py` — block-match
  optical-flow microframe encoder for topic `0x28`.
- [x] `firmware/tractor_x8/image_pipeline/encode_wireframe.py` — packed-bitmap
  edge encoder for topic `0x29`.
- [x] `firmware/tractor_x8/image_pipeline/ipc_to_h747.py` — UART ring-buffer
  hand-off with CRC-8/SMBUS-framed envelope.

**D — AI detector CPU scaffolds (model weights out of band, scaffolding pure
code):** ✅ Done

- [x] `base_station/image_pipeline/detect_yolo.py` — base-side independent
  safety detector with NanoDet-Plus default (Apache-2.0) and YOLOv8 path
  guarded by `LIFETRAC_DETECTOR=yolov8`; `cross_check()` IoU verdict +
  `DetectorWorker` thread.
- [x] `base_station/image_pipeline/superres_cpu.py` — ncnn Real-ESRGAN-x4v3
  façade with passthrough fallback.
- [x] `firmware/tractor_x8/image_pipeline/detect_nanodet.py` — NanoDet-Plus
  scaffold + topic-`0x26` `pack_detection_frame` wire format.

**E — Tooling + cross-cutting code:** ✅ Done

- [x] `tools/pair_handheld.py` — handheld provisioning over USB-CDC with
  signed-handshake + audit-log append.
- [x] LoRa R-6 fragment scheme extended to `TelemetryFrame` —
  `pack_telemetry_fragments()` / `parse_telemetry_fragment()` /
  `TelemetryReassembler` in `lora_proto.py`; obeys 25 ms-per-fragment cap.
- [x] Persistent AES-GCM nonce counter — `base_station/nonce_store.py`
  (file-backed, fsync'd, gap-bumped on every reserve, restore on boot).
- [x] Tests: `tests/test_telemetry_fragmentation.py` covers R-6 single +
  multi-fragment round-trips, dedup, timeouts, and per-source `NonceStore`
  persistence across instances.

**F — Documentation:** ✅ Done

- [x] [DESIGN-CONTROLLER/NON_ARDUINO_BOM.md](DESIGN-CONTROLLER/NON_ARDUINO_BOM.md)
  — DigiKey/Mouser/L-com/Phoenix/Bürkert/McMaster consolidated order list.
- [x] [DESIGN-CONTROLLER/CALIBRATION.md](DESIGN-CONTROLLER/CALIBRATION.md)
  — joystick deadband, flow-valve curve, pressure zero, GPS offset, IMU bias.
- [x] [DESIGN-CONTROLLER/FIELD_SERVICE.md](DESIGN-CONTROLLER/FIELD_SERVICE.md)
  — spare-parts kit, fuse map, diagnostic flowcharts, escalation.
- [x] [DESIGN-CONTROLLER/OPERATIONS_MANUAL.md](DESIGN-CONTROLLER/OPERATIONS_MANUAL.md)
  — operator-facing power-on, pairing, take-control, E-stop, charging.
- [x] [DESIGN-CONTROLLER/FIRMWARE_UPDATES.md](DESIGN-CONTROLLER/FIRMWARE_UPDATES.md)
  — per-node update path (X8 OCI vs H747 USB-CDC vs handheld USB-CDC),
  signing, fleet sequencing.
- [x] [DESIGN-CONTROLLER/BASE_STATION.md](DESIGN-CONTROLLER/BASE_STATION.md)
  trust-boundary table per IMAGE_PIPELINE §6.1 (server-only vs browser surface).

**H — 2026-04-27 follow-up sweep (residual no-hardware items):** ✅ Done

- [x] [firmware/tractor_x8/image_pipeline/tile_diff.py](DESIGN-CONTROLLER/firmware/tractor_x8/image_pipeline/tile_diff.py)
  — 64-bit pHash per 32 px tile + Hamming-distance differ.
- [x] [firmware/tractor_x8/image_pipeline/fragment.py](DESIGN-CONTROLLER/firmware/tractor_x8/image_pipeline/fragment.py)
  — image-PHY fragmenter that reuses the R-6 magic so base-station
  reassembly stays identical for telemetry + image.
- [x] [firmware/tractor_x8/image_pipeline/capture.py](DESIGN-CONTROLLER/firmware/tractor_x8/image_pipeline/capture.py)
  — `CaptureRing` newest-wins ring buffer with `MockCaptureBackend`
  (unit-testable on Windows / CI) and a `V4l2CaptureBackend` shim
  whose embedded body is filled in by the vendor patch.
- [x] [tools/lora_rtt.py](DESIGN-CONTROLLER/tools/lora_rtt.py)
  — RTT harness with JSONL logging, percentile summary, and an
  `--echo-loopback` CI mode that needs no real radios.
- [x] [base_station/person_alert.py](DESIGN-CONTROLLER/base_station/person_alert.py)
  — `CMD_PERSON_APPEARED` (opcode `0x60`) packer + `PersonAlertEmitter`
  with confidence filter, debounce, and audit-log hook; ready to be
  passed as `DetectorWorker(on_result=emitter.feed)`.
- [x] Named `audit_log.py` event helpers (`log_sf_step`, `log_encode_mode_change`,
  `log_fhss_skip`, `log_replay_reject`, `log_gcm_reject`, `log_source_transition`,
  `log_person_appeared`); `lora_bridge.py` and `link_monitor.py` switched
  to the named helpers so the JSONL stays queryable.
- [x] [base_station/web/diagnostics.html](DESIGN-CONTROLLER/base_station/web/diagnostics.html)
  + [diagnostics.js](DESIGN-CONTROLLER/base_station/web/diagnostics.js)
  — airtime utilization graph (30 % WARN / 60 % ALARM bands), SF rung
  history with hysteresis marker, FHSS 8×60 s heatmap, link-loss
  timeline; subscribes to `/ws/state` only.
- [x] [base_station/web/map.html](DESIGN-CONTROLLER/base_station/web/map.html)
  + [map.js](DESIGN-CONTROLLER/base_station/web/map.js)
  — Leaflet with offline-first tile pyramid (`/tiles/{z}/{x}/{y}.png`),
  OSM fallback when online, live tractor marker + breadcrumb track,
  range-estimate sidebar from the FSPL model in IMAGE_PIPELINE Appendix B.

88/88 base_station unit tests pass (was 71; +17 new tests for the helpers,
emitter, and `lora_rtt._percentile`).

**G — Skip (explicitly out of scope this pass):**

- [ ] Legacy-prototype safety bugs (rest of this file) — only worth fixing
  if the prototype gets used for any further bench tests.
- Anything Coral-only (`superres_coral.py`, `interp_rife.py`, `inpaint_lama.py`).
- Anything in Phase 6+ (mast install, integration, field test, FCC verification).
- Phase 10+ stretch goals (SVT-AV1, neural-inflate, NDVI, ROS 2 bridge, etc.).

### Remaining — needs hardware

- [ ] **Phase A1 R-7 retune bench** —
  [firmware/bench/lora_retune_bench/](DESIGN-CONTROLLER/firmware/bench/lora_retune_bench/)
  is ready; needs two LoRa boards on a desk to measure actual retune cost.
- [ ] **Phase B Opta Modbus slave** — replace the `OptaController` /
  D1608S / A0602 placeholder shims in
  [firmware/tractor_opta/opta_modbus_slave.ino](DESIGN-CONTROLLER/firmware/tractor_opta/opta_modbus_slave.ino)
  with real library calls; needs an Opta + expansions on the bench.
- [ ] **Compile `lp_crypto_real.cpp` on-target** (Portenta H7 + MKR WAN
  1310) and verify with the golden-vector test from above.
- [ ] **Tractor M7 ↔ SX1276 SPI driver** (bridge currently talks
  KISS-over-serial; on-tractor needs the real radio path).
- [ ] **Bench-validate E-stop end-to-end** per the procedure in
  [KEY_ROTATION.md](DESIGN-CONTROLLER/KEY_ROTATION.md) §7.

---

## Hardware-in-the-loop (HIL) gate — required before any wet hydraulic test

These are the items the Round 1–4 implementation plan explicitly deferred
because they cannot be validated without the actual bench setup (handheld
+ tractor H747 + Opta + expansions + SX1276 modems on both ends). Each
must pass before the system is allowed to drive real hydraulics. Tracked
against the Wave-4 gates in
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md` § Wave 4](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md).

**HIL bench setup prerequisites:**

- [ ] Two SX1276 LoRa modems wired (handheld MKR WAN 1310 + tractor H747).
- [ ] Tractor H747 + Opta connected over RS-485 with both expansions
  (D1608S + A0602) chained.
- [ ] Solenoid-valve bank powered through the PSR safety relay with the
  hydraulic supply valved off (dry test only, no fluid pressure).
- [ ] Logic analyzer or scope on the X8↔H747 UART, the M7↔M4 GPIO, and
  the PSR-alive line. CRC-protected `.csv` capture for each test.
- [ ] Battery emulator on the 12 V rail so a brownout can be simulated
  without yanking the keyswitch.

**HIL tests (Wave 4 gates):**

- [ ] **W4-01 Handheld E-stop latch latency.** Mushroom-button press →
  PSR-alive drops → all 8 valve coils de-energize within **< 100 ms**
  measured at the relay terminals. Repeat 100× across the SF7/SF8/SF9
  PHY rungs. Capture worst-case + histogram.
- [ ] **W4-02 Link-tune walk-down.** Force RSSI degradation
  (RF attenuator) and verify the M7 walks SF7 → SF8 → SF9 and back via
  `CMD_LINK_TUNE` with **< 1% packet loss** during each transition.
  Verify `try_step_ladder()` revert deadline (500 ms) fires correctly
  on a synthetic missing-HB.
- [ ] **W4-03 M7↔M4 watchdog trip.** Halt the M7 with a debugger
  breakpoint or a deliberate `while(1)` injected via a debug build;
  verify the M4 trips the PSR within **200 ms** of the last
  `alive_tick_ms` and that `estop_request = 0xA5A5A5A5` is observed
  on the SRAM4 capture (IP-105/106 seqlock holds).
- [ ] **W4-04 Modbus failure → E-stop.** Pull the RS-485 cable mid-run;
  verify IP-205 counter ticks, `apply_control(-1)` fires after 10
  consecutive failures, valves go neutral, and the audit log shows
  the disconnect within 1 s.
- [ ] **W4-05 Proportional valve ramp-out (IP-303 Round-4 follow-on).**
  Hold full-speed track for 3 s, release joystick; verify
  `REG_FLOW_SP_*` ramps from 10000 mV to 0 over **2 s** (track ladder)
  on the A0602 output, valve coil stays energized through the ramp,
  drops at t=2 s. Repeat for arm axes (1 s ramp).
- [ ] **W4-06 Mixed-mode skip.** Drive both axes; release one; verify
  released axis stops *immediately* (no ramp) because sibling is
  active. Critical for coordinated dig→drive transitions.
- [ ] **W4-07 Boot-PHY first-frame decode (IP-006 verification).**
  Cold-boot both ends; first encrypted frame decodes on the receiver
  with no `bad_header` events in the audit log.
- [ ] **W4-08 Camera back-channel round-trip (IP-104 Round-4 follow-on).**
  Operator presses "Force keyframe" on web UI → `CMD_REQ_KEYFRAME`
  over LoRa → M7 forwards on Serial1 → `camera_service.py` emits
  I-frame within **< 200 ms** end-to-end. Verify with frame-flag
  capture from the X8↔H747 UART.
- [ ] **W4-09 Async M7 TX state machine (IP-107 follow-on).** Bench-
  validate `radio.startTransmit()` + `isTransmitDone()` IRQ timing
  on real H747 + SX1276 wiring. Once green, replace the
  `refresh_m4_alive_before_tx()` watchdog-refresh hack with the
  proper non-blocking queue. Removes the worst-case time-on-air
  margin assumption.
- [ ] **W4-10 Fleet-key provisioning sanity (IP-008 verification).**
  Flash a fresh image with `lp_keys_secret.h` deleted; confirm the
  M7 halts in `setup()` with the OLED "FLEET KEY NOT PROVISIONED"
  message and the bridge container exits non-zero.

**Status memo for ongoing HIL work:**
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md`](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md)
— Round 4 deferred items + Round 5 candidate queue.

---

## Mechanical / UTU integration

- [ ] Verify the loader-arm hydraulic (lift) cylinders do not collide with
  the upper UTU drive shaft / motor across the full arm travel range
  (`ARM_MIN_ANGLE` … `ARM_MAX_ANGLE`). The drive shaft was lowered to leave
  only 0.25" clearance above the rear cross frame tube, so cylinder swing
  geometry should be re-checked at all lift angles.

## Software / firmware safety (from
[2026-04-25 code review](AI%20NOTES/CODE%20REVIEWS/2026-04-25_Review_ClaudeOpus4_7.md))

The first four items also appear in the
[2026-04-16 review](AI%20NOTES/CODE%20REVIEWS/2026-04-16_Review_GPT5_4.md)
and are still unfixed. They are the highest-priority work on the software side.

### Stale-command / fail-safe (must fix before further deployment)

- [ ] **Opta MQTT proportional control is broken.** Replace
  `doc["..."] | 0` with `doc["..."] | 0.0f` in
  [DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L329-L332)
  so MQTT joystick values are parsed as floats, not coerced to int. Also
  clamp to `[-1.0, 1.0]` and reject NaN, mirroring the BLE path's
  `validateAndClampJoystickValue()`.
- [ ] **Opta MQTT reconnect blocks the safety timeout.** Make
  `reconnectMQTT()` non-blocking (one attempt per loop, gated by `millis()`),
  call `stopAllMovement()` immediately on detected disconnect, and run the
  `SAFETY_TIMEOUT` check on every loop iteration regardless of broker
  state. See
  [DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L221-L300).
- [ ] **Browser keyboard control latches motion after key release.** In
  [DESIGN-CONTROLLER/raspberry_pi_web_controller/static/js/controller.js](DESIGN-CONTROLLER/raspberry_pi_web_controller/static/js/controller.js#L177-L181)
  the `activeKeys.size === 0` branch must zero `currentControl.{left_x,
  left_y, right_x, right_y}` — currently it only refreshes the display.
  Also fix the wholesale-overwrite of `currentControl` in the active branch
  (lines 174–186) so simultaneous joystick + keyboard input does not stomp
  each other; use the existing-but-unused `keyboardControl` accumulator.
- [ ] **ESP32 remote keeps publishing stale axes when a Qwiic joystick
  drops off the I²C bus.** Zero the corresponding axes when
  `joystick.connected()` is false in
  [DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino](DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino#L177-L201).
- [ ] **Mode switch and flow-valve jumper are sampled only at boot.**
  Either poll them in `loop()` (calling `stopAllMovement()` on change) or
  update `MODE_SWITCH_WIRING.md` and the README to make clear that switch
  changes require a reboot. See
  [DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L207-L217).

### Boot-time blocking

- [ ] Time-box `setupWiFi()` (e.g. 30 s) in both
  [DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L268-L276)
  and
  [DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino](DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino#L141-L147).
  On the Opta, fall back to BLE if MQTT mode WiFi fails to associate so the
  unit does not hang in `setup()`.

### Authentication / authorization

- [ ] **Web controller has no auth.** Add a login (Flask-Login or HTTP
  basic over TLS), restrict `cors_allowed_origins` from `"*"` to a known
  origin list, and load `SECRET_KEY` from environment / `config.yaml`.
  See [DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py](DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py#L40-L41).
- [ ] **BLE characteristics accept writes from any unpaired device.**
  Require encryption/pairing on the joystick characteristics in
  [DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L621-L646),
  and add a disconnect hook that calls `stopAllMovement()` explicitly.
- [ ] **Hardcoded MQTT credentials and broker IP** in
  [DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L48-L53)
  and
  [DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino](DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino#L26-L30).
  Move to a `secrets.h` / NVS-stored config and stop documenting the literal
  password as the default.

### Configuration / docs vs code

- [ ] **`config/config.yaml` is documented but not consumed** by
  [DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py](DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py#L40-L54).
  Add a YAML loader that overrides the module-level constants (broker, port,
  credentials, camera resolution, secret key), or remove the docs that
  reference it.
- [ ] Default `MQTT_BROKER` in
  [DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py](DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py#L42)
  should be `127.0.0.1` (broker runs on the Pi itself per the install
  guide), not the literal `192.168.1.100`.
- [ ] Doc-vs-code audit pass: README, INSTALLATION_GUIDE, and
  MODE_SWITCH_WIRING describe runtime behaviors (live mode switching, YAML
  config, etc.) that the firmware does not implement.

### Robustness / quality

- [ ] Normalize tank-steering output before clamping in `computeTrackSpeeds()`
  ([DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L342-L355))
  so high `baseSpeed + turnRate` does not produce a "dead zone" near full
  forward where additional turn input has no effect.
- [ ] Update `previousInput` only after deceleration completes, so a new
  input arriving mid-ramp does not cancel the active→zero transition
  detection.
- [ ] Pin `paho-mqtt < 2.0` in
  [DESIGN-CONTROLLER/raspberry_pi_web_controller/requirements.txt](DESIGN-CONTROLLER/raspberry_pi_web_controller/requirements.txt)
  *or* migrate to `mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, ...)` and
  update callbacks.
- [ ] Register `cleanup()` with `atexit` and install a `SIGTERM` handler in
  [DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py](DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py#L385-L399)
  so the camera process is terminated under systemd shutdown.
- [ ] Restart `libcamera-vid` automatically on stream loss in
  [DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py](DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py#L142-L196)
  and notify the browser via SocketIO.
- [ ] Rate-limit / debounce `control_command` events server-side in
  [DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py](DESIGN-CONTROLLER/raspberry_pi_web_controller/app.py#L243-L271).
- [ ] After an `emergency_stop`, server should publish a sticky zero
  `control_command` every ~100 ms for ~1 s to be robust to packet loss
  (mitigates browser `setInterval` throttling on hidden tabs).
- [ ] `serialEvent()` on ESP32 ([DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino](DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino#L353-L380))
  is dead code; the documented "type `stop`" e-stop does not work. Poll
  `Serial.available()` from `loop()` instead.
- [ ] Either remove the placeholder `readBatteryVoltage()` in
  [DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino](DESIGN-CONTROLLER/esp32_remote_control/lifetrac_v25_remote.ino#L286-L293)
  or document and calibrate the divider.
- [ ] Detect the placeholder literals `"YOUR_WIFI_SSID"` /
  `"YOUR_WIFI_PASSWORD"` at runtime and refuse to enter MQTT mode (fall
  back to BLE) instead of looping forever in `setupWiFi()`.
- [ ] Migrate from `DynamicJsonDocument` to `StaticJsonDocument` (or
  ArduinoJson 7's `JsonDocument`) for the small fixed-size payloads on both
  firmwares.
- [ ] De-duplicate `controlValve()` and `controlTrack()` in
  [DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino](DESIGN-CONTROLLER/arduino_opta_controller/lifetrac_v25_controller.ino#L498-L527).
- [ ] Factor the joystick wire format into a shared `lifetrac_protocol.h`
  consumed by both firmwares so the Opta and ESP32 cannot drift on
  deadzone / clamp / type expectations.

### Test coverage

- [ ] Extend [DESIGN-CONTROLLER/test_scripts/mqtt_test.py](DESIGN-CONTROLLER/test_scripts/mqtt_test.py) (or add a
  pytest) that publishes representative float payloads and asserts they
  round-trip through the parsing logic the Opta firmware uses. This would
  catch regressions of the int-coercion class.
- [ ] Wire the MQTT contract test into `ARDUINO_CI` so it runs on PRs.
