# LifeTrac v25 — Master Test Program

**Status:** living document, updated every implementation round.
**Last updated:** 2026-04-29 (Round 26).
**Owner:** every PR that adds, removes, or re-scopes a test MUST update
the relevant section of this file in the same commit. See
[§7 Update Protocol](#7-update-protocol).

This page is the **single index** for every test the LifeTrac v25
controller subsystem runs. It does NOT duplicate the implementation
plan, the HIL runbook, or the round-by-round status memo — it
links into them and answers three questions they don't:

1. **What tests exist today?** (catalog tables — §2, §3, §4)
2. **Which Implementation-Plan item (IP-XXX) does each test cover?** (§5)
3. **What is the gap between SIL coverage and the HIL bench?** (§4)

---

## Table of Contents

1. [Tier overview](#1-tier-overview)
2. [SIL test catalog](#2-sil-test-catalog)
3. [Compile-gate catalog](#3-compile-gate-catalog)
4. [HIL bench matrix](#4-hil-bench-matrix)
5. [Per-IP traceability](#5-per-ip-traceability)
6. [How to run](#6-how-to-run)
7. [Update protocol](#7-update-protocol)

---

## 1. Tier overview

The v25 controller is verified at three tiers. Each tier catches a
different class of regression — none of the three is sufficient on its
own.

| Tier | Where it runs | What it catches | Count (Round 20) |
|---|---|---|---|
| **SIL** (software-in-the-loop) | CI on every push, pure Python stdlib + project deps | Logic, protocol, state-machine regressions; cross-language byte-identity (C ↔ Python). **No hardware required.** | **329 tests** across [31 files](#2-sil-test-catalog) |
| **Compile gates** | CI on every push, `arduino-cli compile` against pinned cores | Toolchain drift, missing libraries, link errors, flash/RAM regressions, FQBN / sketch-folder convention breaks. | **5 jobs** (4 sketches × 1, plus stub/real-crypto on handheld) — [§3](#3-compile-gate-catalog) |
| **HIL** (hardware-in-the-loop) | Bench, manually, per [HIL_RUNBOOK.md](DESIGN-CONTROLLER/HIL_RUNBOOK.md) | Real-radio timing, valve-coil energise/release latency, RF link budget, IRQ jitter, on-air decode after cold-boot. | **10 procedures** — [§4](#4-hil-bench-matrix) |

**Coverage rule:** every HIL bench item should have a SIL companion that
verifies the *logic* it depends on (state machine, threshold, ramp
shape, ladder transitions). The HIL run is then a *confirmation* that
the on-air / valve-coil reality matches the model — much faster to
diagnose when something fails. SIL coverage of a HIL item is documented
in [§4](#4-hil-bench-matrix).

---

## 2. SIL test catalog

All paths relative to [`LifeTrac-v25/DESIGN-CONTROLLER/base_station/tests/`](DESIGN-CONTROLLER/base_station/tests/).

| File | Tests | Scope | IP refs | W4 gate | Round |
|---|---|---|---|---|---|
| [test_accel_select.py](DESIGN-CONTROLLER/base_station/tests/test_accel_select.py) | 10 | Coral accelerator detection and state management. | — | — | — |
| [test_audit_log.py](DESIGN-CONTROLLER/base_station/tests/test_audit_log.py) | 5 | JSONL append-only audit log rotation and thread-safety. | — | — | — |
| [test_axis_ramp_sil.py](DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py) | 10 | M7 proportional valve ramp timing + mixed-mode skip arbitration. | — | W4-05, W4-06 | 18 |
| [test_back_channel_dispatch.py](DESIGN-CONTROLLER/base_station/tests/test_back_channel_dispatch.py) | 12 | X8 back-channel dispatcher for camera control opcodes. | IP-104 | (W4-08 — partial) | — |
| [test_boot_phy_sil.py](DESIGN-CONTROLLER/base_station/tests/test_boot_phy_sil.py) | 11 | Boot-PHY first-frame decode: LADDER[0] + radio.begin() consistency across all nodes. | IP-006 | W4-07 | 20 |
| [test_fleet_key_provisioning_sil.py](DESIGN-CONTROLLER/base_station/tests/test_fleet_key_provisioning_sil.py) | 24 | Fleet-key sanity: bridge `_load_fleet_key()` failure modes; firmware `#ifndef LIFETRAC_ALLOW_UNCONFIGURED_KEY` guard, OLED text, ESTOP magic write. | IP-008 | W4-10 | 22 |
| [test_hil_harness_completeness_sil.py](DESIGN-CONTROLLER/base_station/tests/test_hil_harness_completeness_sil.py) | 13 | W4-XX HIL harness contract: file existence per gate, harness honours `_common`/Write-GateHeader/Assert-Section0/Write-HilResult/HIL_RUNBOOK back-ref, JSONL schema invariants (W4-01..W4-10 pattern + PASS/FAIL/SKIP/ABORT enum + 5-key firmware-SHA bundle), dispatcher gate-set parity. | HIL-01 | (operationalises all W4-XX) | 26 |
| [test_keyframe_round_trip_sil.py](DESIGN-CONTROLLER/base_station/tests/test_keyframe_round_trip_sil.py) | 17 | Camera back-channel keyframe round-trip: bridge translation, encrypt/KISS air round-trip, M7 wrap → X8 dispatcher, SF7/BW250 < 100 ms LoRa-leg airtime model. | IP-103, IP-104 | W4-08 | 22 |
| [test_command_frame_fuzz.py](DESIGN-CONTROLLER/base_station/tests/test_command_frame_fuzz.py) | 5 | FT_COMMAND wire format CRC and layout invariants. | IP-108 | — | — |
| [test_control_frame_fuzz.py](DESIGN-CONTROLLER/base_station/tests/test_control_frame_fuzz.py) | 7 | FT_CONTROL packet structure, stick clipping, CRC validation. | IP-108 | — | 8 |
| [test_crypto_vectors.py](DESIGN-CONTROLLER/base_station/tests/test_crypto_vectors.py) | 2 | AES-GCM encryption vectors (Python vs. firmware C). | IP-003 | — | — |
| [test_followup_sweep.py](DESIGN-CONTROLLER/base_station/tests/test_followup_sweep.py) | 15 | Image-pipeline Coral inference logging and debounce. | — | — | — |
| [test_image_pipeline.py](DESIGN-CONTROLLER/base_station/tests/test_image_pipeline.py) | 18 | Canvas tile reassembly, delta encoding, grid mismatches. | — | — | — |
| [test_image_reassembly_fuzz.py](DESIGN-CONTROLLER/base_station/tests/test_image_reassembly_fuzz.py) | 12 | Image fragment reordering, timeouts, interleaved streams. | — | — | 13 |
| [test_link_monitor.py](DESIGN-CONTROLLER/base_station/tests/test_link_monitor.py) | 3 | LoRa heartbeat window classification for link quality. | — | — | — |
| [test_link_monitor_orchestrator.py](DESIGN-CONTROLLER/base_station/tests/test_link_monitor_orchestrator.py) | 2 | Orchestrator state machine for image encode-mode transitions. | — | — | — |
| [test_link_tune_sil.py](DESIGN-CONTROLLER/base_station/tests/test_link_tune_sil.py) | 16 | M7 adaptive SF ladder walk-down/up + 500 ms revert deadline + twice-back-to-back announce. | — | W4-02 | 19 |
| [test_lora_proto.py](DESIGN-CONTROLLER/base_station/tests/test_lora_proto.py) | 19 | CRC16, KISS encoding, replay window, CSMA, PHY routing. | IP-004 | — | — |
| [test_m4_safety_sil.py](DESIGN-CONTROLLER/base_station/tests/test_m4_safety_sil.py) | 11 | M4 watchdog trip, seqlock safety, E-stop latency < 100 ms. | IP-105, IP-106 | W4-01, W4-03 | 8 |
| [test_m7_tx_queue_sil.py](DESIGN-CONTROLLER/base_station/tests/test_m7_tx_queue_sil.py) | 6 | M7 non-blocking TX queue with priority preemption. | IP-107 | (W4-09 — partial) | — |
| [test_modbus_slave_sil.py](DESIGN-CONTROLLER/base_station/tests/test_modbus_slave_sil.py) | 17 | Opta Modbus watchdog, fail-count latch, ramp arbitration. | IP-205 | W4-04 | 10, 14 |
| [test_mqtt_retry_backoff_sil.py](DESIGN-CONTROLLER/base_station/tests/test_mqtt_retry_backoff_sil.py) | 13 | Web-UI MQTT connect: fake-clock retry/backoff schedule (`0.5→1→2→4→5→5`), 30 s monotonic deadline, env-var host, source-grep tripwire on contract literals. | IP-201 | — | 25 |
| [test_nonce_seq_threadthrough_sil.py](DESIGN-CONTROLLER/base_station/tests/test_nonce_seq_threadthrough_sil.py) | 13 | Bridge `_on_mqtt_message` reserves seq, stamps cleartext header, threads same seq into `_tx(nonce_seq=)` so GCM nonce seq == header seq across all four command arms; 100-message acceptance + replay window check + 16-bit wrap. | IP-102 | — | 24 |
| [test_protocol_constants_sil.py](DESIGN-CONTROLLER/base_station/tests/test_protocol_constants_sil.py) | 12 | Wave-3 polish constants tripwire: `TELEM_MAX_PAYLOAD` C↔Python pin, `REG_AUX_OUTPUTS` triplet (slave define + master enum + write-handler mask), handheld `s_btn_change_ms` boot-anchor in `setup()`. | IP-306, IP-303, IP-301 | — | 23 |
| [test_replay_window_invariant.py](DESIGN-CONTROLLER/base_station/tests/test_replay_window_invariant.py) | 2 | Cross-language ReplayWindow C/Python bit-for-bit match. | IP-101 | — | 9 |
| [test_telemetry_fragmentation.py](DESIGN-CONTROLLER/base_station/tests/test_telemetry_fragmentation.py) | 7 | Telemetry packet fragmentation and reassembly. | IP-204 | — | — |
| [test_telemetry_fragmentation_fuzz.py](DESIGN-CONTROLLER/base_station/tests/test_telemetry_fragmentation_fuzz.py) | 15 | Multi-stream telemetry reassembly, wrap-around, GC. | IP-204 | — | 11 |
| [test_web_ui_accel.py](DESIGN-CONTROLLER/base_station/tests/test_web_ui_accel.py) | 5 | Web UI Coral toggle and settings persistence. | — | — | — |
| [test_web_ui_auth.py](DESIGN-CONTROLLER/base_station/tests/test_web_ui_auth.py) | 8 | PIN auth, session cookies, lockout, IP rate-limiting. | IP-206, IP-207 | — | — |
| [test_web_ui_validation.py](DESIGN-CONTROLLER/base_station/tests/test_web_ui_validation.py) | 15 | Pydantic input validation, subscriber admission bounds. | IP-202, IP-203 | — | — |
| [test_ws_subscriber_concurrency.py](DESIGN-CONTROLLER/base_station/tests/test_ws_subscriber_concurrency.py) | 4 | WebSocket pool cap, concurrent admit/discard, MQTT fan-out. | IP-202, IP-208 | — | 12 |

**Totals:** 31 files, 329 tests, 1 skipped.

> **Test counts:** the "Tests" column counts `def test_*` methods, not
> parametrized cases. The skipped test in `test_accel_select.py` is
> Coral-hardware-gated and is expected to skip in CI.

---

## 3. Compile-gate catalog

CI workflow: [`.github/workflows/arduino-ci.yml`](.github/workflows/arduino-ci.yml).
All paths relative to [`LifeTrac-v25/DESIGN-CONTROLLER/firmware/`](DESIGN-CONTROLLER/firmware/).

| Job | FQBN | Sketch | Blocking? | Notes |
|---|---|---|---|---|
| `python-protocol-tests` | n/a | n/a | **Yes** | Runs the Python 3.11 unittest suite from §2. Must be green before any compile job is dispatched. |
| `firmware-compile-handheld` (stub) | `arduino:samd:mkrwan1310` | [`handheld_mkr/`](DESIGN-CONTROLLER/firmware/handheld_mkr/) | **Yes** | Stub-crypto build — 55 932 B / 21 % flash. (Round 15) |
| `firmware-compile-handheld` (real) | `arduino:samd:mkrwan1310` | [`handheld_mkr/`](DESIGN-CONTROLLER/firmware/handheld_mkr/) | **Yes** | Real-crypto build (`-DLIFETRAC_USE_REAL_CRYPTO`) — 59 940 B / 22 % flash. (Round 15) |
| `firmware-compile-opta` | `arduino:mbed_opta:opta` | [`tractor_opta/`](DESIGN-CONTROLLER/firmware/tractor_opta/) | **Yes** | 171 520 B / 8 % flash, 65 120 B / 12 % RAM. (Round 16) Requires `Arduino_Opta_Blueprint` lib install. |
| `firmware-compile-tractor-h7` | `arduino:mbed_portenta:envie_m7` | [`tractor_h7/`](DESIGN-CONTROLLER/firmware/tractor_h7/) | **Yes** | M7 build — 207 888 B / 9 % flash. Stages `lora_proto` + `shared_mem.h` to `src/`. (Round 17) |
| `firmware-compile-tractor-m4` | `arduino:mbed_portenta:envie_m7:target_core=cm4` | [`tractor_h7_m4/`](DESIGN-CONTROLLER/firmware/tractor_h7_m4/) | **Yes** | M4 watchdog — 87 032 B. Separate sketch folder per Portenta dual-core convention. (Round 17) |
| `firmware-compile-plan` | n/a | n/a | informational | Resolves the ESP32 / extra hardware sketches for inventory only — not gating. |

**5 of 5 production-firmware jobs are blocking** (no `continue-on-error`).
The Round 17 dual-core split documented why a single sketch with
`#if defined(CORE_CM4)` cannot work: arduino-cli scans `.ino` `#include`
directives regardless of `target_core`, so RadioLib + Modbus get pulled
into the M4 build. The two-folder layout matches Arduino IDE's stock
PortentaDualCore example.

---

## 4. HIL bench matrix

Procedures: [`DESIGN-CONTROLLER/HIL_RUNBOOK.md`](DESIGN-CONTROLLER/HIL_RUNBOOK.md).
"Bench-only residual" = the part of the procedure that no SIL test
can replace (timing on real silicon, RF link budget, valve-coil energise
latency, …).

| W4-XX | Goal | Pass criterion | SIL coverage | Bench-only residual |
|---|---|---|---|---|
| **W4-01** | Handheld E-stop latch < 100 ms across SF7/8/9 | 100/100 captures `latency_ms < 100`; p99 < 80 ms | [test_m4_safety_sil.py](DESIGN-CONTROLLER/base_station/tests/test_m4_safety_sil.py) | Real PSR-alive falling edge → relay terminal de-energise time |
| **W4-02** | Link-tune walk-down SF7→8→9 with < 1 % packet loss; revert deadline 500 ms | full ladder walk both directions; histogram monotonic; zero missed E-stop | [test_link_tune_sil.py](DESIGN-CONTROLLER/base_station/tests/test_link_tune_sil.py) | RF attenuator + < 1 % loss measurement |
| **W4-03** | M7↔M4 watchdog trip < 200 ms; `estop_request = 0xA5A5A5A5` magic on SRAM4 | 10/10 runs `t_trip - t_halt < 200 ms`; magic confirmed | [test_m4_safety_sil.py](DESIGN-CONTROLLER/base_station/tests/test_m4_safety_sil.py) | SRAM4 capture probe + JTAG halt of M7 |
| **W4-04** | Modbus failure → E-stop within 1 s; IP-205 counter + 10-failure latch | `t_apply_-1 - t_disconnect < 1.0 s`; audit log entry; no stuck motion | [test_modbus_slave_sil.py](DESIGN-CONTROLLER/base_station/tests/test_modbus_slave_sil.py) | Physical RS-485 cable disconnect mid-run |
| **W4-05** | Proportional valve ramp-out: track 2 s, arm 1 s; coil stays engaged | track 2.00 ± 0.10 s; arm 1.00 ± 0.05 s; linear R² > 0.98 | [test_axis_ramp_sil.py](DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py) | A0602 output probe + valve-coil voltage capture |
| **W4-06** | Mixed-mode skip: sibling-active release drops coil < 50 ms (no ramp) | 10/10 runs drop < 50 ms | [test_axis_ramp_sil.py](DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py) | Same probe rig as W4-05 |
| **W4-07** | Cold-boot first-frame decode; no `bad_header` events | 20/20 alternate power-on orders; zero `bad_header` in 10 s window | [test_boot_phy_sil.py](DESIGN-CONTROLLER/base_station/tests/test_boot_phy_sil.py) | Audit-log capture of first 10 s after each cold boot |
| **W4-08** | Camera back-channel `CMD_REQ_KEYFRAME` < 200 ms end-to-end | 30/30 runs < 200 ms; LoRa leg < 100 ms | [test_back_channel_dispatch.py](DESIGN-CONTROLLER/base_station/tests/test_back_channel_dispatch.py) (X8 dispatcher) + [test_keyframe_round_trip_sil.py](DESIGN-CONTROLLER/base_station/tests/test_keyframe_round_trip_sil.py) (bridge translation, air round-trip, M7 wrap, airtime budget model) | Real H747 Serial1 baud stability + camera-encoder I-frame produce time on real X8 silicon |
| **W4-09** | Async M7 TX state machine: `startTransmit()` + `isTransmitDone()` IRQ timing | Phase 1 time-on-air within 5 % of theory + IRQ→flag p99 < 2 ms; Phase 2 W4-03 + alive_tick jitter p99 < 5 ms | [test_m7_tx_queue_sil.py](DESIGN-CONTROLLER/base_station/tests/test_m7_tx_queue_sil.py) (queue logic only) | Real SX1276 IRQ wiring on real H747 silicon — **hard-blocked, no SIL substitute possible** |
| **W4-10** | Fleet-key provisioning sanity: missing key halts M7 + bridge at startup | compile fail (missing header); OLED `FLEET KEY NOT PROVISIONED`; container exit ≠ 0 | [test_fleet_key_provisioning_sil.py](DESIGN-CONTROLLER/base_station/tests/test_fleet_key_provisioning_sil.py) (bridge loader failure modes; source-parse of handheld + tractor `#ifndef` guard, OLED string, ESTOP magic write) + handheld stub/real compile gates | OLED visual confirmation on real bench; systemd-unit restart behavior on real X8 |

**Tier status (Round 22):**
- 9 of 10 (W4-01, W4-02, W4-03, W4-04, W4-05, W4-06, W4-07, W4-08,
  W4-10) have SIL coverage of the *logic surface* and need only a
  bench-confirmation pass.
- 1 of 10 (W4-09) has *partial* SIL — the M7 TX queue logic is
  covered, but the IRQ-driven async startTransmit / isTransmitDone
  on real SX1276 silicon is hard-blocked from SIL substitution.

---

## 5. Per-IP traceability

Every Implementation-Plan item (IP-001 … IP-309) from
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md`](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md)
mapped to its verifying test. **"—" in the test column means the IP is
landed but has no automated verification** — those are the cells that
should drive future SIL rounds.

### Wave 0 — blockers / repo hygiene

| IP | Severity | Title | Verified by |
|---|---|---|---|
| IP-001 | BLOCKER | Rename sketch folders / `.ino` files | All compile gates (§3) |
| IP-002 | BLOCKER | Wire `LIFETRAC_PIN`, `LIFETRAC_MQTT_HOST`, `LIFETRAC_LORA_DEVICE` env vars | (env-var contract — manual) |
| IP-003 | BLOCKER | Fix `lp_decrypt(ct_len)` contract mismatch | [test_crypto_vectors.py](DESIGN-CONTROLLER/base_station/tests/test_crypto_vectors.py) |
| IP-004 | BLOCKER | Fix M7 CSMA frequency bug (hop index vs Hz) | [test_lora_proto.py](DESIGN-CONTROLLER/base_station/tests/test_lora_proto.py) |
| IP-005 | BLOCKER | Resolve `LIFETRAC_SETTINGS_PATH` env-var drift | (env-var contract — manual) |
| IP-006 | BLOCKER | Match boot PHY to `LADDER[0]` (BW250) | [test_boot_phy_sil.py](DESIGN-CONTROLLER/base_station/tests/test_boot_phy_sil.py) + W4-07 |
| IP-007 | BLOCKER | CI must build firmware and run all suites | [arduino-ci.yml](.github/workflows/arduino-ci.yml) (§3) |
| IP-008 | BLOCKER | Provisioned-or-fail fleet-key loading | [test_fleet_key_provisioning_sil.py](DESIGN-CONTROLLER/base_station/tests/test_fleet_key_provisioning_sil.py) + handheld compile gates + W4-10 |

### Wave 1 — protocol / pipeline

| IP | Severity | Title | Verified by |
|---|---|---|---|
| IP-101 | HIGH | Wire `NonceStore` into the LoRa bridge | [test_replay_window_invariant.py](DESIGN-CONTROLLER/base_station/tests/test_replay_window_invariant.py) |
| IP-102 | HIGH | Thread `nonce_seq` through bridge | [test_nonce_seq_threadthrough_sil.py](DESIGN-CONTROLLER/base_station/tests/test_nonce_seq_threadthrough_sil.py) |
| IP-103 | HIGH | Subscribe and forward `cmd/req_keyframe` | [test_back_channel_dispatch.py](DESIGN-CONTROLLER/base_station/tests/test_back_channel_dispatch.py) + [test_keyframe_round_trip_sil.py](DESIGN-CONTROLLER/base_station/tests/test_keyframe_round_trip_sil.py) |
| IP-104 | HIGH | Wire X8 `camera_service` to M7 UART | [test_back_channel_dispatch.py](DESIGN-CONTROLLER/base_station/tests/test_back_channel_dispatch.py) + [test_keyframe_round_trip_sil.py](DESIGN-CONTROLLER/base_station/tests/test_keyframe_round_trip_sil.py) + W4-08 |
| IP-105 | HIGH | M4 read of SRAM4 must be seqlock-protected | [test_m4_safety_sil.py](DESIGN-CONTROLLER/base_station/tests/test_m4_safety_sil.py) |
| IP-106 | HIGH | Watchdog wrap on M4: require `0xA5A5A5A5` magic | [test_m4_safety_sil.py](DESIGN-CONTROLLER/base_station/tests/test_m4_safety_sil.py) |
| IP-107 | HIGH | M7 LoRa TX async (un-block M4 watchdog) | [test_m7_tx_queue_sil.py](DESIGN-CONTROLLER/base_station/tests/test_m7_tx_queue_sil.py) + W4-09 |
| IP-108 | HIGH | Fix CRC asymmetry on `FT_COMMAND` | [test_command_frame_fuzz.py](DESIGN-CONTROLLER/base_station/tests/test_command_frame_fuzz.py), [test_control_frame_fuzz.py](DESIGN-CONTROLLER/base_station/tests/test_control_frame_fuzz.py) |

### Wave 2 — base-station hardening

| IP | Severity | Title | Verified by |
|---|---|---|---|
| IP-201 | MED | MQTT connect retry/backoff | [test_mqtt_retry_backoff_sil.py](DESIGN-CONTROLLER/base_station/tests/test_mqtt_retry_backoff_sil.py) |
| IP-202 | MED | WebSocket fan-out admission cap | [test_web_ui_validation.py](DESIGN-CONTROLLER/base_station/tests/test_web_ui_validation.py), [test_ws_subscriber_concurrency.py](DESIGN-CONTROLLER/base_station/tests/test_ws_subscriber_concurrency.py) |
| IP-203 | MED | Pydantic input validation | [test_web_ui_validation.py](DESIGN-CONTROLLER/base_station/tests/test_web_ui_validation.py) |
| IP-204 | MED | Decode telemetry through `TelemetryReassembler` | [test_telemetry_fragmentation.py](DESIGN-CONTROLLER/base_station/tests/test_telemetry_fragmentation.py), [test_telemetry_fragmentation_fuzz.py](DESIGN-CONTROLLER/base_station/tests/test_telemetry_fragmentation_fuzz.py) |
| IP-205 | MED | Surface Modbus failures (counter + 10-fail latch) | [test_modbus_slave_sil.py](DESIGN-CONTROLLER/base_station/tests/test_modbus_slave_sil.py) + W4-04 |
| IP-206 | MED | Honor `X-Forwarded-For` for PIN lockout | [test_web_ui_auth.py](DESIGN-CONTROLLER/base_station/tests/test_web_ui_auth.py) |
| IP-207 | MED | Forward auth failures to `AuditLog` | [test_web_ui_auth.py](DESIGN-CONTROLLER/base_station/tests/test_web_ui_auth.py) |
| IP-208 | MED | `camera_service.py` thread race + clamp WEBP_QUALITY | [test_ws_subscriber_concurrency.py](DESIGN-CONTROLLER/base_station/tests/test_ws_subscriber_concurrency.py) |
| IP-209 | MED | Lockfile Python deps (`requirements.lock`) | (build-time — manual review of lockfile) |

### Wave 3 — firmware polish

| IP | Severity | Title | Verified by |
|---|---|---|---|
| IP-301 | MED | Initialize `s_btn_change_ms` at boot | [test_protocol_constants_sil.py](DESIGN-CONTROLLER/base_station/tests/test_protocol_constants_sil.py) |
| IP-302 | MED | Fix axis dead-band scaling | [test_axis_ramp_sil.py](DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py) (deadband constant pinned) |
| IP-303 | MED | Implement `REG_AUX_OUTPUTS` or remove it | [test_protocol_constants_sil.py](DESIGN-CONTROLLER/base_station/tests/test_protocol_constants_sil.py) |
| IP-304 | MED | Stable PYTHONPATH for X8 services | (deployment — manual) |
| IP-305 | LOW | `lp_kiss_encode` over-conservative bounds | [test_lora_proto.py](DESIGN-CONTROLLER/base_station/tests/test_lora_proto.py) |
| IP-306 | LOW | Reconcile `TELEM_MAX_PAYLOAD` C(118) vs Python(120) | [test_protocol_constants_sil.py](DESIGN-CONTROLLER/base_station/tests/test_protocol_constants_sil.py) |
| IP-307 | LOW | Per-source-active OLED status threshold | — |
| IP-308 | LOW | Boot self-test PIN ordering | — |
| IP-309 | LOW | Camera tile-diff numpy vectorization | — |

**Coverage gaps (drive future SIL rounds):**

As of Round 25 every "—" cell that has a natural SIL surface is
closed. The remaining "—" cells (IP-002, IP-005, IP-209, IP-304,
IP-307, IP-308, IP-309) are deployment / manual-review items that
don't have a natural SIL surface. Document them in the relevant
deployment runbook
instead of forcing test coverage.

---

## 6. How to run

### Run all SIL tests (CI-equivalent)

```powershell
cd c:\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\base_station
$env:LIFETRAC_ALLOW_UNCONFIGURED_KEY="1"
python -m unittest discover -s tests
```

Expected: `Ran 278 tests in ~0.9s` `OK (skipped=1)`.

### Run a single SIL file

```powershell
python -m unittest tests.test_link_tune_sil -v
```

### Run a single test method

```powershell
python -m unittest tests.test_axis_ramp_sil.W4_05_RampOutTests.test_TR_A_track_full_speed_two_second_ramp -v
```

### Compile gates locally (per-sketch)

```powershell
# Stage shared sources (one-time per build):
cd c:\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER
.\tools\stage_firmware_sketches.ps1 -LoraProto common\lora_proto\

# Then compile, e.g. handheld:
arduino-cli compile --fqbn arduino:samd:mkrwan1310 firmware\handheld_mkr
```

For the M4 build, use `arduino:mbed_portenta:envie_m7:target_core=cm4`
on the [`firmware/tractor_h7_m4/`](DESIGN-CONTROLLER/firmware/tractor_h7_m4/)
sketch (NOT a separate `envie_m4` FQBN — that doesn't exist).

### HIL pre-flight

Per [HIL_RUNBOOK.md](DESIGN-CONTROLLER/HIL_RUNBOOK.md) §0:
1. Confirm SIL suite + all compile gates green on the commit under test.
2. Confirm fleet key provisioned on every node (no `0x00…` placeholder).
3. Confirm the [BUILD-CONTROLLER first-power-up checklist](BUILD-CONTROLLER/06_bringup_and_testing.md)
   has passed for the bench rig.

---

## 7. Update protocol

Every implementation round MUST update this file in the same PR as the
test/code change. Specifically:

1. **New SIL test file** → add a row to [§2](#2-sil-test-catalog),
   update the totals at the bottom of §2, update the test count in
   the [§1 tier overview](#1-tier-overview), and tick the relevant IP
   row(s) in [§5](#5-per-ip-traceability).
2. **New SIL coverage of a HIL item** → fill in the "SIL coverage"
   cell in [§4](#4-hil-bench-matrix) and link both ways.
3. **New / changed compile-gate job** → update [§3](#3-compile-gate-catalog)
   AND the count in [§1](#1-tier-overview).
4. **New IP added to the implementation plan** → add a row to the
   relevant Wave subsection of [§5](#5-per-ip-traceability) with `—`
   in the test column; that's the trigger for the next SIL round.
5. **Bump the "Last updated" line at the top of this file** to the
   round number that landed the change.

If you need to deviate (e.g. a deferred-coverage SIL is intentional),
document the rationale in the relevant cell with a footnote rather
than leaving the cell ambiguous.
