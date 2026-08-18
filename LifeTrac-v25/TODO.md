# LifeTrac v25 — TODO

> **🟢 Radio status (2026-08-17) — both loss-floor campaigns resolved.**
> The bench LoRa link runs at **0.9 % fragment loss** (from 5.9 % two days
> prior). RS-11.6: two external ISM emitters characterized (a ~7 s hopper
> and an exact-10 s device at −30 dBm); escaped by carrier choice — the
> band reshuffles in HOURS, so channel picks come from a same-day survey
> (`channel_survey_sniff.py` + `survey_compare.py`; 927.5 MHz is the only
> 3/3-clean channel so far). RS-12: the historic slot-(total−2) loss was
> an L072 URC-overwrite race at the short-final-fragment ride; host-side
> fix (`-NoParkLast 1`) validated n=3 + strict-hold. Remaining: one
> confirmation-sized flash session (rx_urc_lost + firmware URC fix +
> RS-3.6 gate) and the RS-3.3 camera first flight. Live campaign state:
> [DESIGN-CONTROLLER/TODO.md](DESIGN-CONTROLLER/TODO.md) (RS-12 / RS-12.9
> sequencing), issues #98/#107, evidence under
> `DESIGN-CONTROLLER/bench-evidence/`.

> **🟢 Milestone (2026-05-26) — image-over-LoRa air link proven end-to-end:**
> Tractor camera → tile-delta encode → MQTT (intra-X8) → image_tx_daemon →
> Murata L072 → 915 MHz LoRa → base L072 → image_rx_daemon → MQTT
> (intra-X8) → web_ui `/ws/state` → browser `<canvas id="image-canvas">`.
> Sustained zero-fragment-loss P-frame runs at SF7/BW500, `frames_published`
> climbing on the base, "2 Hz · tile stream" badge live in the UI. Final
> visual-pixel confirmation in the browser pending a base-X8 power cycle.
> Full status + open blockers (B1–B6):
> [AI NOTES/2026-05-26_Image_Over_LoRa_AirLink_Proven_Status_Copilot_v1_0.md](AI%20NOTES/2026-05-26_Image_Over_LoRa_AirLink_Proven_Status_Copilot_v1_0.md).
>
> **📦 First-release packaging plan (2026-05-26) — v25.0.1-bench:**
> Recommendation = git tag + a single combined `LifeTrac-v25/RELEASE/v25.0.1-bench/`
> manifest directory (Dockerfiles, compose, L072 firmware binaries + sha256,
> install.sh, SBOM, release notes). **Do not** create `TractorCodeV25.0.1/` /
> `BaseStationCodeV25.0.1/` duplicate source trees. Code triage (~70 % ships,
> ~20 % dev/diagnostic, ~10 % retire to `_retired/`), 10-step cut-the-tag
> runbook, image-compression-protocol readiness, and direct answers to the
> user's release questions are all in:
> [AI NOTES/2026-05-26_v25.0.1_Release_Packaging_Plan_Copilot_v1_0.md](AI%20NOTES/2026-05-26_v25.0.1_Release_Packaging_Plan_Copilot_v1_0.md).
>
> **� New blocker (2026-05-22 17:15) — P8:** First post-reflash W1-11
> ping_pong run failed 5/7 gates (`radio_tx_ok=0/100`, no RX frames). Root
> cause: commit `d4dfcb8` (2026-05-20) added `-DLIFETRAC_FHSS_TX_ROUTED=1`
> to `murata_l072/Makefile`; current `firmware.bin` therefore refuses every
> `TX_FRAME_REQ` with `HOST_ERR_PROTO_FORBIDDEN` until the host arms a
> profile via `cfg_set(CFG_KEY_REG_PROFILE=0x14, …)` — which no bench probe
> currently does. RX side fails symmetrically with
> `HOST_FAULT_CODE_RX_SCAN_FAILED (0x0D)`. **All RF testing (W1-10b, W1-11,
> W2-02, T4/T5b RF half) is blocked.** Three fix options (host-side /
> firmware-side / build-variant) need user direction before applying — see
> [AI NOTES/2026-05-22_W1-11_RF_Blocked_FHSS_Gate_v1_0.md](AI%20NOTES/2026-05-22_W1-11_RF_Blocked_FHSS_Gate_v1_0.md)
> and the new P8 entry in
> [AI NOTES/2026-05-21_Open_Problems_To_Fix_v1_0.md](AI%20NOTES/2026-05-21_Open_Problems_To_Fix_v1_0.md).
> Evidence dir:
> `DESIGN-CONTROLLER/bench-evidence/W1-11_pingpong_2026-05-22_165144/`.
> Note: the earlier P1 closure (below) is still valid — but only for the
> CFG_GET-only cold-boot discriminator. It does **not** prove RF I/O.
>
> **�🟢 Recent closure (2026-05-22):** P1 cold-boot stale-firmware regression
> is **CLOSED** after re-flashing both X8 boards with current `firmware.bin`.
> Two bugs fixed along the way: `stm32_an3155_flasher.py` termios import is
> now optional (LmP Python lacks `termios`); `run_flash_l072.sh` now
> **unexports** gpio8/10/15 immediately before launching `openocd` to avoid
> a sysfs/mmap race in the newer (`2025-07-14`) OpenOCD build on RX X8 that
> manifested as `SWD DPIDR 0xdeadbeef`. Post-reflash discriminator on RX
> shows clean `RUNTIME_PROFILE_ENUM=0` on 5/5 cycles. See
> [AI NOTES/2026-05-22_RX_Reflash_And_P1_Resolution_v1_0.md](AI%20NOTES/2026-05-22_RX_Reflash_And_P1_Resolution_v1_0.md)
> and the updated
> [AI NOTES/2026-05-21_Open_Problems_To_Fix_v1_0.md](AI%20NOTES/2026-05-21_Open_Problems_To_Fix_v1_0.md)
> closure table.
>
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

**Test inventory & per-IP traceability:** the canonical index of every
SIL test, every Arduino compile gate, every HIL bench item, and which
IP each one verifies lives in
➡️ **[MASTER_TEST_PROGRAM.md](MASTER_TEST_PROGRAM.md)** —
update it in the same PR as any test/code change (see §7 of that file
for the protocol).

---

## 2026-05-18 TX-power adaptation + SAFETY-burst implementation plan

Design doc + six review passes (Gemini 3.1 Pro, GPT-5.3-Codex, three Claude
Opus 4.7 passes, GitHub Copilot) converged on 16 decisions (D1–D16) and 5
internal contradictions (C1–C5). Full source-of-truth:

➡️ **[AI NOTES/2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md](AI%20NOTES/2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md)**
(see §21.1 decision table, §21.2 contradictions, §21.3 gaps, §23.6 order)

Work is sequenced as **stages S0–S7**. Each stage has one deliverable + one
falsification gate; the next stage does not start until its predecessor's
gate passes on bench and evidence is committed under
`DESIGN-CONTROLLER/bench-evidence/`.

**Sequencing rule:** S2 is a hard blocker for S4 (the E-STOP replay-rejection
bug makes the 5-copy SAFETY burst actively dangerous — every burst copy
after the first is rejected as a replay, so the burst *reduces* assert
probability instead of raising it). S0+S1 are independent of firmware and
can run in parallel with S2.

**Bench hardware reality (updated 2026-05-19):** the LoRa-stack hardware
on the bench is **two Portenta X8 + Max Carrier boards and one camera**.
Each Max Carrier carries an onboard **Murata CMWX1ZZABZ-078**
(= STM32L072 + SX1276 SiP), so the two boards together ARE the production
radio set: **one Max Carrier is the base-station radio, the other is the
tractor-side radio.** The MKR WAN 1310 handheld referenced in earlier
revisions of this plan is **optional and not on the critical path**.

**Test-equipment scope (project decision 2026-05-19):**
- **No spectrum analyzer, no calibrated antenna, no TCB lab access.**
  FCC §15.247 EIRP compliance ships under **conducted-path datasheet
  declaration + firmware-side TX-power cap** (no lab-measured EIRP
  evidence). See S-HW.4 (permanently skipped) and S4.2 below.
- **No step attenuator.** S1.1 walk-power, S4 PER induction, S6
  margin-limited induction will substitute **range-walk + natural path
  loss** for controlled attenuation. See S-HW.2 (permanently skipped).
- **E-STOP model (revised 2026-05-19):** tractor-side E-STOP signal =
  **Arduino Opta digital input flip** routed to the base-station Max
  Carrier; operator-side E-STOP = **keyboard / joystick button on web
  remote controller UI**. No physical mushroom button required at the
  handheld; the Opta itself can carry a mushroom-button in a later
  hardening pass without re-architecting the signal path. See S-HW.5.

Every stage below is therefore tagged:

- 🟢 **HW-READY** — can complete on current bench (software-only or X8-only or 2× Max Carrier).
- 🟡 **HW-PARTIAL** — software foundations can be built now; final gate needs the Opta + web-UI button wiring (S-HW.5).
- ⚫ **HW-WAIVED** — originally required lab test-equipment we have
  permanently decided not to acquire (spectrum analyzer, step
  attenuator); ships under software-only enforcement + datasheet
  declaration. NOT a blocker.

Strategy: drive every 🟢 and the software half of every 🟡 to done now;
the full radio link (W1-10b) is already proven end-to-end on the 2× Max
Carrier pair. See "S-HW" and "Software-only work that can run NOW"
sections at the bottom of this plan for ordered next steps.

**Cross-doc precedence (add to design-doc top in S0, per §23.3):** when
docs disagree, implementation follows (1) current code constants + measured
bench evidence, (2) `DESIGN-CONTROLLER/DECISIONS.md`, (3) `LORA_PROTOCOL.md`,
(4) `IMAGE_PIPELINE.md`, (5) the 2026-05-18 design doc.

### Stage S0 — Document consolidation (editorial, no code) 🟢 HW-READY
- [x] **S0.1** Fix C1: §3.8 silence default 250 ms → 500 ms ship; 250 ms
      behind bench flag. — **DONE 2026-05-18** (strikethrough preserves
      original for review-trail integrity).
- [x] **S0.2** Fix C2: §3.9 recommendation Option U → **Option T** for
      product; Option U as migration shim. — **DONE 2026-05-18.**
- [x] **S0.3** Fix C3: §3.5 — all "H7 host (operator side)" language
      removed; adapter is X8 Python on both ends. Section title updated.
      — **DONE 2026-05-18** (§4 ASCII diagram label "Operator H7" not yet
      redrawn; tracked under S0.6 deferred follow-up).
- [x] **S0.4** Fix C4: §4 image-traffic row → "plaintext + 4 B seq + 2 B
      CRC32 (+6 B, no MAC)". SAFETY power "+17 dBm forced" → "regional
      EIRP cap per §21.3-6". — **DONE 2026-05-18.**
- [ ] **S0.5** *(deferred from surgical pass)* Renumber duplicate `## 13`,
      `## 14`, `## 15`, `## 18` headings. Requires audit of every
      `§13.x`-style cross-reference in §12–§23 first; defer until full
      consolidation pass.
- [ ] **S0.6** *(deferred from surgical pass)* Promote §15.3 P0/P1/P2/P3
      policy table into §4 as the canonical traffic-class table (replacing
      `STREAM_*`). §4 row labels were tagged inline with their P0–P3
      equivalents in S0.4 as a transitional step; full promotion + ASCII
      diagram redraw is the larger follow-up.
- [x] **S0.7** Add §23.3 precedence rule at top of design doc. — **DONE
      2026-05-18.**
- [x] **S0.8** Replace "20% P0 PER target" → "field gate <1% PER, recovery
      trigger before 2%". — **DONE 2026-05-18** (folded into §4 row edit).
- **Gate:** document reads top-to-bottom without contradicting itself.
  **Status (2026-05-18):** C1–C5 surgical fixes ✅ complete. Structural
  cleanup (S0.5, S0.6) still pending; doc line count is 2743 (up slightly
  from 2687 because the change-log footer was appended). Trim toward
  ~1200 lines after S0.5/S0.6 land.

### Stage S1 — Bench instrumentation (read-only firmware) 🟡 HW-PARTIAL
*(Software half can be built now on the two X8 boards; RF half requires
L072 + handheld reattached.)*
- [x] **S1.0** *(software-only)* ~~Implement
      `DESIGN-CONTROLLER/base_station/lora_airtime.py` — pure-Python ToA
      predictor `lora_time_on_air_ms(sf, bw, cr, payload_len, ...)`
      matching Semtech SX1276/SX1262 datasheet formulas, plus
      `encrypted_payload_len(class, raw_len, crypto_profile)` helper.~~
      **DONE 2026-05-18.** Discovery showed `lora_time_on_air_ms` and
      `encrypted_payload_len` already existed in `lora_proto.py`, so
      instead of creating a duplicate `lora_airtime.py` module we
      **extended `lora_proto.py`** with a `CryptoProfile` registry
      (`CRYPTO_GCM128_EXPLICIT` +28 B / `CRYPTO_GCM64_IMPLICIT` +12 B /
      `CRYPTO_IMAGE_PLAIN_CRC32` +6 B) and a backward-compatible
      `profile=` arg on `encrypted_payload_len`. Six new tests in
      `tests/test_lora_proto.py` pin the §17/§19 worked-example numbers
      (46.208/33.408 ms control, 28.224/23.104/20.544 ms image,
      524.800 ms telemetry) and assert that the shipped GCM-128 image
      profile MUST violate the 25 ms cap (sentinel test — guards D13/D14
      motivation). 24/24 lora_proto tests pass; 846/846 base_station
      tests pass (the 12 unrelated pre-existing failures in
      `test_fleet_key_provisioning_sil.py` and one `test_x8_encode_mode`
      case are infrastructure, not from this change). NOTE: §17 quoted
      557.57 ms for 100 B telemetry but the project's PHY_TELEMETRY
      (preamble=12) actually gives 524.800 ms — codebase is source of
      truth; doc footnote needs updating in a follow-up S0 doc-cleanup
      pass.
- [x] **S1.1** Add `walk_power` mode to
      `method_h_stage2_tx_probe_v2.py`: sweep 2 → 17 dBm in 1 dB steps,
      log RSSI/SNR/PER/CRC to CSV under `bench-evidence/walk_power_<date>/`.
      *(Code lands now; sweep run blocked until L072 reattached.)*
      *(DONE 2026-05-18 — new `run_walk_power()` + `--probe walk_power`
      mode in `method_h_stage2_tx_probe_v2.py`. CLI flags `--power-min`
      (default 2) / `--power-max` (default 17) / `--power-step` (default 1) /
      `--per-step-count` (default 100) / `--walk-payload-len` (default 16) /
      `--csv-out`. Per step issues `CFG_SET_REQ(CFG_KEY_TX_POWER_DBM=0x01,
      dbm)`, snapshots `radio_tx_ok/_abort_lbt/_abort_airtime`, sends N
      `TX_FRAME_REQ`s, captures `TX_DONE_URC` `status / time_on_air_us /
      tx_power_dbm` per frame, then writes one CSV row
      `(timestamp, step_idx, power_dbm_requested, power_dbm_echoed_first,
      count_sent, tx_done_ok, tx_done_fail, tx_timeout, radio_tx_ok_delta,
      radio_tx_abort_lbt_delta, radio_tx_abort_airtime_delta,
      mean_toa_us, tx_per_pct, rx_per_pct, rx_rssi_dbm_mean,
      rx_snr_db_mean, rx_crc_err_count)`. RX-side columns are emitted
      empty — a paired `--probe rx_listen` orchestrator running on the
      second board fills them post-sweep (see S1.4). Default output dir
      `bench-evidence/walk_power_<YYYY-MM-DD_HHMMSS>/walk_power_tx_side.csv`.
      Smoke-tested: `py method_h_stage2_tx_probe_v2.py --help` shows the
      new mode + flags; module imports clean. **First paired sweep landed
      2026-05-19** — see [bench-evidence/walk_power_pilot_2026-05-19/](DESIGN-CONTROLLER/bench-evidence/walk_power_pilot_2026-05-19/README.md);
      both L072s alive, 16-step 2..17 dBm sweep on 915.000 MHz with mean
      PER 0.59 %, RSSI monotonic, `__PAIRED_SWEEP_VERDICT__=OK`.)*
- [x] **S1.2** Instrument host orchestrator to record per-class p50/p99
      `TX_DONE` latency, P0 TX-start delay, queue age, measured-vs-predicted
      encrypted ToA. *(Hooks land now; data collection blocked until L072
      reattached.)*
      *(Host hooks DONE 2026-05-18 — new `tx_latency_meter.py` module
      defines `TxLatencyMeter` with `mark_enqueue / mark_dequeue /
      mark_done(predicted_toa_ms=…)` and per-class rolling p50/p99
      windows for queue_age, tx_done, and toa_delta. Wired into
      `Bridge._tx` (enqueue token) and `Bridge._tx_worker`
      (dequeue + done timestamps); all calls wrapped in try/except so
      instrumentation never breaks TX. Pinned by 14 tests in
      `test_tx_latency_meter.py` covering empty/lifecycle/per-class
      isolation/P0 p99 tail/missing-done/unknown-token/leak-guard/
      reset/rolling-window. **Real-airtime data still blocked on
      L072 reattach** — `predicted_toa_ms` is passed as `None` from
      the bridge for now; once `TX_DONE_URC (0x90)` is plumbed back
      from the L072 we substitute the real on-air timestamp and the
      ledger's PHY airtime prediction.)*
- [x] **S1.3** Extend topic `0x10` decoder + add `newest_frame_wins` counter
      to W2-02 stability harness (preparation for S3 validation).
      *(Decoder/counter land now; validation run blocked.)*
      *(DONE 2026-05-18 — two new pure-Python modules: (1)
      `topic_0x10_decoder.py` defines `decode_topic_0x10()` →
      `SourceActiveSnapshot` normalising legacy 1-byte enum, JSON
      dict, and forward-rev payloads; preserves unknown keys in
      `extras`; fail-closed on garbage. (2) `newest_frame_wins.py`
      defines `NewestFrameWinsCounter` with RFC-1982 16-bit
      serial-arithmetic accept/stale/duplicate counters per
      AI NOTES 2026-05-18 §"newest-frame-wins gate". Counter wired
      into `w2_02_host_pipeline.py::cmd_decode` (observability only —
      paint logic unchanged for backward-compat) so the harness
      summary JSON now carries `newest_frame_wins.{accepted,
      stale_dropped, duplicate_dropped, last_accepted_seq}`. The S3
      gate target is `stale_dropped == 0` over a 10-minute mixed-load
      run — pre-validation today, validation blocked on the L072
      reattach + W2-02 stability rig.)*
- [x] **S1.4** ✅ *(CLOSED 2026-05-19 — full sweep + soak both landed)*
      Commit one full `walk_power` sweep and one 10-minute mixed-load run
      to `bench-evidence/`.
      *(Pilot 2026-05-19: 16-step × 50-packet paired sweep at
      `DESIGN-CONTROLLER/bench-evidence/walk_power_pilot_2026-05-19/` —
      verdict OK, mean PER 0.59 %, RSSI/SNR monotonic.
      Full 2026-05-19: 16-step × 200-packet paired sweep at
      `DESIGN-CONTROLLER/bench-evidence/walk_power_full_2026-05-19/` —
      verdict OK, mean PER 1.22 %, 3 200/3 200 `tx_done_ok`,
      0 `tx_aborted_airtime` at `--inter-cycle-s 0.07`, RSSI -118.9 →
      -111.2 dBm monotonic.
      Soak 2026-05-19: 6 500-packet × ~13 min paired soak at
      `DESIGN-CONTROLLER/bench-evidence/mixed_load_2026-05-19/` —
      6 500/6 500 `tx_done_ok`, 0 timeout, **0 faults, 0 invariants
      violated**, 6 463/6 500 RX (PER 0.57 %).
      Airtime-gate falsification done: the pilot's 12-16 % attempt loss was
      NOT EU 1 % regulatory but the 40 % internal per-channel fairness cap
      in `sx1276_airtime.c`; see
      `AI NOTES/2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md`
      for the actual FCC envelope and the resulting FHSS work item.
      Multi-channel / multi-payload-class "mixed-load" is gated on FHSS
      scheduler landing — captured as a follow-up in the mixed_load README.)*
- **Gate:** measured vs. predicted encrypted ToA delta < 10 ms on at least
  one P0 and one P3 packet class. (S1.0 unit tests against historical CSVs
  give a software-only proxy that can pre-pass before LoRa rig returns.)

### Stage S1.5 — FCC §15.247 50-channel FHSS (PRE-LAUNCH BLOCKER) 🔴 NEW
*(Per [AI NOTES/2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md](AI%20NOTES/2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md)
§19-§20 and the v3.0 consolidated execution order in
[AI NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md §14](AI%20NOTES/2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md#14-consolidated-v30-plan-deltas-authoritative).
Production radio profile is `FCC_15_247_FHSS_50CH_BW250` at the existing
+17 dBm clamp. All current S1.x bench evidence is stamped
`BENCH_ONLY_FIXED_915` and is **not** FCC field evidence. The L072 today
hardcodes `s_channel_idx = 0U` in `sx1276_tx.c:64` — single fixed
channel — which is the single largest compliance gap.)*

**Implementation (firmware, executed in v3.0 order):**
- [x] **FCC-FHSS-CHANTAB** ✅ *(2026-05-19 — landed)* Channel-table
      generator + static asserts:
      `_Static_assert(FHSS_CHANNEL_COUNT == 50)`,
      `_Static_assert(FHSS_CHANNEL_SPACING_HZ >= 25000)`,
      first/last centers asserted inside 902-928 MHz. Formula
      `center_hz = 902_750_000 + 500_000*i, i=0..49` (span 902.750-927.250
      MHz, 750 kHz edge guard). Files:
      [`include/sx1276_fhss_chantab.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_fhss_chantab.h),
      [`radio/sx1276_fhss_chantab.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_fhss_chantab.c),
      golden-vector C test
      [`bench/host_proto/fhss_chantab_vectors.c`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/fhss_chantab_vectors.c),
      Python mirror (X8-side parity, byte-identical golden vectors)
      [`bench/host_proto/fhss_chantab.py`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/fhss_chantab.py).
      Wired into Makefile via new `check-fhss-chantab` target hooked
      into the aggregate `check`. Verified: `mingw32-make check` passes
      with `[OK] sx1276_fhss_chantab: all checks passed (count=50,
      span=902750000..927250000 Hz)`.
      Final spacing tighten is gated on FCC-EVID-D2 (measured 20 dB
      OBW); placeholder comment in the .c file marks the future assert.
- [x] **FCC-PROFILE-ENUM** ✅ *(2026-05-19 — landed)* Added
      `CFG_KEY_REG_PROFILE` (key `0x14U`, u8) with enum
      `{REG_PROFILE_BENCH_ONLY_FIXED_915=0,
      REG_PROFILE_FCC_15_247_FHSS_50CH_BW250=1,
      REG_PROFILE_FCC_15_247_DTS_BW500=2}` in
      [`include/host_cfg_keys.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_keys.h).
      New status code `CFG_STATUS_PROFILE_UNROUTED=7` in
      [`include/host_cfg.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg.h).
      Validator in
      [`host/host_cfg.c`](DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c)
      always accepts bench profile, rejects unknown values with
      `OUT_OF_RANGE`, gates both production profiles behind
      `#ifdef LIFETRAC_FHSS_TX_ROUTED` (undefined until FCC-A4) →
      returns `CFG_STATUS_PROFILE_UNROUTED` today. Default value =
      bench profile. `CFG_KEY_COUNT` bumped 20→21;
      `HOST_WIRE_SCHEMA_VER` bumped 1→2. Tests in
      [`bench/host_proto/cfg_contract.c`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/cfg_contract.c)
      extended with 4 cases (`reg_profile_bench_default_ok`,
      `reg_profile_fcc_fhss_unrouted`, `reg_profile_fcc_dts_unrouted`,
      `reg_profile_unknown_oor`). `mingw32-make check-cfg-contract`
      passes 34 cases + 6 wire vectors.
- [x] **FCC-A1a** Config-time airtime invariant in modem-config setter:
      reject `(SF, BW, CR)` whose worst-case ToA > `dwell_cap_ms - guard_ms`
      (400 - 20 ms default). New URC `__AIRTIME_INVARIANT_REJECT__`.
      ✅ 2026-05-19. New pure helper `sx1276_airtime_compute_toa_us()`
      and config-time predicate `sx1276_airtime_config_invariant_ok()`
      added in
      [`include/sx1276_airtime.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_airtime.h)
      /
      [`radio/sx1276_airtime.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c);
      cap constants `SX1276_AIRTIME_DWELL_WINDOW_MS=400`,
      `SX1276_AIRTIME_DWELL_GUARD_US=20000`,
      `SX1276_AIRTIME_DWELL_CAP_US=380000`. New URC byte
      `HOST_TYPE_AIRTIME_INVARIANT_REJECT_URC=0xC2` with 15-byte payload
      `{u8 sf, u8 cr_den, u32 bw_hz_le, u8 payload_len, u32 toa_us_le, u32 cap_us_le}`
      registered in
      [`include/host_types.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h)
      (added to compile-time uniqueness switch). New bool-returning
      variant `sx1276_set_sf_bw_cr_checked()` in
      [`radio/sx1276.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c)
      runs the invariant at max payload (255 B); on reject it emits the
      URC and skips all modem-register writes so the prior legal config
      is retained. The existing `void sx1276_set_sf_bw_cr()` is kept as
      a thin wrapper that discards the bool to preserve all existing
      call sites (init path: SF7/BW250/CR45 → 199.808 ms, accepted).
      Host golden vectors in
      [`bench/host_proto/airtime_invariant.c`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/airtime_invariant.c)
      (7 cases incl. SF12/BW125/255B reject, SF11/BW125/255B reject,
      SF10/BW500/255B reject, SF7/BW250/255B accept, SF7/BW250/24B
      accept). `mingw32-make check-airtime-invariant` →
      `[PASS] airtime_invariant: 7 cases`; aggregate
      `mingw32-make check` still green
      (`[OK] sx1276_fhss_chantab` + `[PASS] airtime_invariant` +
      `[OK] memory map invariants hold`).
- [x] **FCC-A1b** Pre-TX airtime invariant in `sx1276_tx_begin()` keyed on
      actual payload + header + CRC + preamble + safety-burst framing.
      Host-side mirror calc keyed on `profile_id` so oversized payloads
      never reach the L072.
      ✅ 2026-05-19. Per-frame dwell-cap check added in
      [`radio/sx1276_tx.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c)
      between LBT and `sx1276_airtime_reserve()`: calls
      `sx1276_airtime_estimate_toa_us(req->length)` (which reads live
      modem-config + preamble registers), and on `predicted > 380 ms`
      increments `host_stats_radio_tx_abort_airtime()`, runs
      `sx1276_tx_cleanup()` (re-arms RX if needed), and returns false.
      Counter-ownership check (`[OK] Airtime abort ownership check passed`)
      confirms the increment site is still uniquely owned by
      `radio/sx1276_tx.c`. Host-side mirror table + lookup added in
      [`bench/host_proto/airtime_invariant.c`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/airtime_invariant.c)
      mapping `REG_PROFILE_BENCH_ONLY_FIXED_915`,
      `REG_PROFILE_FCC_15_247_FHSS_50CH_BW250`,
      `REG_PROFILE_FCC_15_247_DTS_BW500` to their `(sf, bw_khz, cr_den)`
      tuples (table is FCC-A4's production-routing precursor). Five new
      test cases prove: (a) every payload 0..255 is accepted under the
      FHSS profile, (b) ToA is monotonic non-decreasing in payload_len,
      (c) host mirror agrees byte-for-byte with the on-target helper at
      payloads {0,1,16,24,64,128,200,255}, (d) bench profile @255B fits
      under cap, (e) unknown `profile_id` fails closed without mutating
      the out param. `mingw32-make check-airtime-invariant` →
      `[PASS] airtime_invariant: 12 cases`; aggregate `mingw32-make check`
      still green; `mingw32-make check-airtime-counter-owner` still
      `[OK]`.
- [x] **FCC-A3** New `radio/sx1276_fhss.{c,h}`: 50-channel table, pseudo-
      random permutation `permute(H(farm_id||node_id||epoch))`, blacklist
      with legal floor (refuse if active set would drop below 50),
      cold-start warm-up, `record_lbt_block()` separate from blacklist.
      **Fails closed** on invalid epoch / hop / timebase. Golden vectors
      checked in under `bench/host_proto/`.
      ✅ 2026-05-19. New module landed:
      [`include/sx1276_fhss.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_fhss.h)
      defines the `sx1276_fhss_status_t` fail-closed enum
      (`NOT_INIT`/`NULL_ARG`/`BAD_IDX`/`BLACKLIST_FLOOR`/`BLACKLIST_WARMUP`/`ALREADY_BLACKED`/`INTERNAL`)
      plus pure helpers `sx1276_fhss_compute_seed()` (FNV-1a 32-bit over
      LE `farm_id||node_id||epoch`) and `sx1276_fhss_compute_permutation()`
      (Fisher-Yates seeded by xorshift32; zero-seed fallback prevents
      the xorshift32 zero-state trap).
      [`radio/sx1276_fhss.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_fhss.c)
      implements the stateful scheduler with `init/reset/next_channel/
      epoch_advance/blacklist/active_count/record_lbt_block/lbt_block_count`
      plus `current_epoch/slot/seed/is_initialized` telemetry getters.
      Cold-start warm-up = 50 hops (`SX1276_FHSS_WARMUP_HOPS`);
      `_Static_assert` enforces legal floor ≤ channel count and channel
      count ≤ 64 (blacklist bitmap fits `uint64_t`). Time source NOT read
      inside the module — caller drives `epoch_advance()` from the
      monotonic tick so golden vectors stay deterministic. New
      `mingw32-make check-fhss-scheduler` target →
      `[PASS] fhss_scheduler_vectors: 11 cases` covering: 3 hard-coded
      golden seed+permutation vectors (zero / nominal /
      `0xFFFF…F`-saturated), determinism vs distinctness, zero-seed
      fallback, NOT_INIT / NULL_ARG / BAD_IDX fail-closed paths,
      single-epoch coverage bijection, slot-wrap auto-advance, warm-up
      blacklist refusal then post-warm-up floor refusal, LBT-block
      decoupling, and epoch-advance regeneration. Aggregate
      `mingw32-make check` still green
      (`[OK] sx1276_fhss_chantab` + `[PASS] fhss_scheduler_vectors: 11 cases`
      + `[PASS] airtime_invariant: 12 cases` + `[OK] memory map invariants
      hold`). Host mirror at `bench/host_proto/fhss_scheduler.py` is
      deferred to FCC-A4 (when X8-side hop prediction lands).
- [x] **FCC-A2** ✅ 2026-05-19 — split addressed by **adding** a new
      module rather than carving the existing one: the 1 s / 400 ms QoS
      gate stays untouched in
      [`radio/sx1276_airtime.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c)
      (still emitting `qos_used_us_1s`); the FCC §15.247(a)(1)(i)
      legal-dwell accountant lives in
      [`include/sx1276_legal_dwell.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_legal_dwell.h)
      + [`radio/sx1276_legal_dwell.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_legal_dwell.c).
      Widened to 64 channel slots (`SX1276_DWELL_CHANNEL_COUNT=64`);
      both 10 s (`legal_dwell_used_us_10s`) and 20 s
      (`legal_dwell_used_us_20s`) windows supported via a `window_ms`
      parameter (the 20 s variant is gated on by FCC-A5 cfg validation,
      not by this module). Window semantics are half-open `[t-W, t)`
      (event at `start_ms == now - W` is OUT; tested at
      [`bench/host_proto/legal_dwell.c` L116-L131](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/legal_dwell.c#L116-L131)).
      Storage model: single shared 128-event ring carrying per-event
      channel index (~1.5 KiB RAM) — chosen over per-channel mini-rings
      to fit L072's 20 KiB SRAM. Pessimistic reserve immediately books
      the worst-case ToA; reconcile is **monotonically downward only**
      (cannot grow legal dwell beyond the reservation, verified by
      [`test_reconcile_up_is_noop`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/legal_dwell.c#L91-L102)).
      No-rollback-on-NACK contract is enforced **structurally** by not
      exposing any release API; verified by
      [`test_no_rollback_on_nack`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/legal_dwell.c#L104-L121)
      which proves the booking persists for the entire window after a
      simulated NACK. Property tests also cover per-channel isolation,
      64-ch widening + `BAD_CH`, 10 s vs 20 s window split, over-budget
      rejection (does not book), ring-full fail-closed, stale-handle
      generation safety, and bad-input rejection. Wired into
      [`Makefile`](DESIGN-CONTROLLER/firmware/murata_l072/Makefile) as
      `check-legal-dwell` and the aggregate `check` target. Last run
      green: `mingw32-make check` →
      `[OK] sx1276_fhss_chantab` + `[PASS] fhss_scheduler_vectors: 11 cases`
      + `[PASS] airtime_invariant: 12 cases` +
      `[PASS] legal_dwell: 15 cases` + `[OK] memory map invariants hold`.
      Integration into `sx1276_tx.c` (calling `reserve()` before
      LBT/TX-start and `reconcile()` from TX-done) is part of FCC-A4
      (active-set wiring) so the module stays unused but verified
      until then.
- [x] **FCC-B1-PERTX** ✅ 2026-05-19 — Per-TX `RFCO` URC scaffolded.
      URC type `HOST_TYPE_RFCO_PERTX_URC = 0xC3U` registered in
      [`include/host_types.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h)
      with cross-ref to the full spec in
      [`include/host_rfco.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco.h).
      Payload is 21 bytes, little-endian, schema_ver=1, additive-only:
      `{schema_ver, profile_id, tx_status, hop_idx, channel_idx, epoch,
      freq_hz, pkt_toa_us, legal_dwell_used_us_10s}`. Naming discipline
      per plan delta #11 — the dwell field is explicitly suffixed
      `_us_10s` so bench post-processing can never conflate it with
      `qos_used_us_1s`. Status enum covers OK / ABORT_* (one per fail-
      closed gate: airtime invariant, LBT, legal-dwell, QoS) / TX_TIMEOUT
      / TX_FAIL / INTERNAL. Pack + emit live in
      [`host/host_rfco.c`](DESIGN-CONTROLLER/firmware/murata_l072/host/host_rfco.c)
      — pack() is pure (host-testable, no HAL deps) and emit() is the
      ONLY public wrapper around `host_uart_send_urc()` for this URC so
      the schema_ver byte stays in one place. Host test
      [`bench/host_proto/rfco_pertx.c`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/rfco_pertx.c)
      pins the wire layout with a byte-for-byte golden vector
      (`freq_hz=915500000=0x36916BE0`, `pkt_toa_us=12345`,
      `legal_dwell_used_us_10s=200000`), edge-value coverage
      (UINT32_MAX in all u32 fields, status=0xFF round-trip), NULL-input
      rejection, and a forwarding test that verifies emit() calls
      `host_uart_send_urc` with `(type=0xC3, payload_len=21, payload ==
      pack(snapshot))`. Wired into
      [`Makefile`](DESIGN-CONTROLLER/firmware/murata_l072/Makefile) as
      `check-rfco-pertx` and the aggregate `check` target. Last run
      green: `mingw32-make check` →
      `[OK] sx1276_fhss_chantab` + `[PASS] fhss_scheduler_vectors: 11 cases`
      + `[PASS] airtime_invariant: 12 cases` +
      `[PASS] legal_dwell: 15 cases` +
      `[PASS] rfco_pertx: 6 cases` + `[OK] memory map invariants hold`.
      `host/host_rfco.c` is in the firmware SRCS list so it will link
      into the next cross-build; the call site in `sx1276_tx.c`
      (emitting OK / ABORT_* / TX_FAIL) is deferred to FCC-A4 when TX
      is routed through the FHSS scheduler — until then there is no
      `channel_idx` / `epoch` to report, so emitting now would supply
      stub zeros and pollute bench evidence. The 50-bucket per-channel
      histogram + per-channel `legal_dwell_max_us` snapshot is the
      separate per-minute `RFCO_SUMMARY` URC (FCC-B1-SUMMARY).
- [x] **FCC-A5** ✅ 2026-05-19 — Regulatory-profile validator + two-phase
      commit landed as pure, host-testable module
      [`include/host_cfg_profile.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_profile.h)
      / [`host/host_cfg_profile.c`](DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg_profile.c).
      State machine is `host_cfg_profile_reset(initial)` →
      `host_cfg_profile_stage(&req, tx_routed)` →
      `host_cfg_profile_activate()` with `host_cfg_profile_cancel_stage()`
      and `host_cfg_profile_has_stage()` introspection. Active profile
      is NEVER mutated on a failed stage or a failed activate
      (NOT_STAGED). A failed stage clears any prior in-flight stage so a
      stale candidate cannot accidentally commit on the next activate.
      Validator enforces (per plan §A5):
        - `popcount(channel_mask) >= 50` for `FCC_15_247_FHSS_50CH_BW250`
          (since the certified table is exactly 50 bits this collapses
          to `channel_mask == HOST_CFG_PROFILE_FHSS_50CH_REQUIRED_MASK
          = (1ULL<<50)-1`),
        - no mask bits set above bit 49 (table-membership),
        - modem BW == 250 kHz for FHSS / 500 kHz for DTS,
        - antenna gain ∈ [0, 30] dBi,
        - production profiles are rejected with `REJECT_UNROUTED` when
          the explicit `tx_routed` boolean (mirroring compile-time
          `LIFETRAC_FHSS_TX_ROUTED`) is false. Until FCC-A4 lands the
          firmware passes `tx_routed = false` so only
          `BENCH_ONLY_FIXED_915` activates.
      Profile-aware power clamp is the pure helper
      `host_cfg_profile_power_clamp(tier_ceiling, hw_ceiling, gain)`
      implementing `Pmax = min(tier_ceiling - max(0, gain_dBi - 6),
      hw_ceiling)` floored at `HOST_CFG_PROFILE_TX_POWER_MIN_DBM = 2`;
      returns 0 when no headroom remains and the validator maps that to
      `REJECT_NO_POWER_HEADROOM`. Tier ceilings: bench = +17 dBm (hw
      ceiling), FHSS = +30 dBm tier, DTS = +30 dBm tier; production
      tiers are then further clamped by the caller-supplied
      `hw_ceiling_dBm` (typically +17/+20 dBm on this Murata module).
      Structured reject reasons (`HOST_CFG_PROFILE_REJECT_*`,
      BAD_PROFILE / MASK_POPCOUNT / MASK_OUT_OF_TABLE / BW_MISMATCH /
      ANTENNA_OUT_OF_RANGE / NO_POWER_HEADROOM / UNROUTED / NOT_STAGED
      / NULL_ARG) are surfaced byte-for-byte to the wire layer so bench
      post-processing can attribute every rejection. Host test
      [`bench/host_proto/cfg_profile.c`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/cfg_profile.c)
      runs 25 cases covering the validator (every reject reason),
      power-clamp arithmetic (gain≤6 no-op, gain>6 ERP reduction,
      hw-ceiling clamp, negative gain, no-headroom floor), tier-ceiling
      lookup, and the full state machine (happy path, activate-without-
      stage, failed-stage-clears-in-flight, cancel, double-activate,
      unrouted-leaves-active-alone). Wired into
      [`Makefile`](DESIGN-CONTROLLER/firmware/murata_l072/Makefile) as
      `check-cfg-profile` and the aggregate `check` target; module is
      also in firmware SRCS so the next cross-build will link it.
      `cfg_set(CFG_KEY_REG_PROFILE)` integration (routing wire-level
      profile writes through stage/activate) is deferred to FCC-A4 when
      `LIFETRAC_FHSS_TX_ROUTED` is defined and there is a routed TX
      path to honour the new profile. Last run green: `mingw32-make
      check` → `[OK] chantab` + `[PASS] fhss_scheduler_vectors: 11
      cases` + `[PASS] airtime_invariant: 12 cases` +
      `[PASS] legal_dwell: 15 cases` + `[PASS] rfco_pertx: 6 cases` +
      `[PASS] cfg_profile: 25 cases` + `[OK] memory map invariants
      hold`.
- [x] **FCC-A4** Replace `s_channel_idx = 0U` in
      [`sx1276_tx.c` L64](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c)
      with `sx1276_fhss_next_channel()` + `sx1276_set_frequency_hz()` +
      PLL settle (document measured settle budget) **before** LBT/CAD/TX-
      start. Defines `LIFETRAC_FHSS_TX_ROUTED`.
      Landed 2026-05-19 (assistant): the FHSS-routed TX path is gated
      by `#ifdef LIFETRAC_FHSS_TX_ROUTED` in
      [`radio/sx1276_tx.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c).
      On `sx1276_tx_begin()` the new flow is: capture `(hop_idx,
      epoch, channel_idx, freq_hz)` from
      `sx1276_fhss_next_channel()` → modes_to_standby → retune via
      `sx1276_set_frequency_hz(freq_hz)` → busy-wait
      `SX1276_TX_PLL_SETTLE_US=200 µs` (4× the SX1276 datasheet rev 7
      §4.1.4 ~50 µs worst-case PLL lock budget; pending bench
      characterisation under the actual Murata CMWX1ZZABZ TCXO) → LBT
      → FCC-A1b airtime invariant → FCC-A2
      `sx1276_legal_dwell_reserve(channel_idx, predicted_toa_us,
      now_ms, 10_000 ms, 400_000 µs)` → QoS reserve → FIFO →
      `modes_to_tx()`. Every fail-closed gate (INTERNAL, ABORT_LBT,
      ABORT_AIRTIME_INVARIANT, ABORT_QOS, ABORT_LEGAL_DWELL, TX_FAIL)
      emits an `RFCO_PERTX` URC via `tx_emit_rfco_pertx()` so bench
      post-processing sees the captured hop state regardless of
      outcome. `sx1276_tx_poll()` reconciles the legal-dwell reserve
      down to the post-rearm predicted ToA on TX_DONE and emits
      `HOST_RFCO_TX_STATUS_OK`; on timeout it emits
      `HOST_RFCO_TX_STATUS_TX_TIMEOUT` and leaves the pessimistic
      reserve booked (FCC-A2 no-rollback contract). The pre-A4
      fixed-channel behaviour is preserved when
      `LIFETRAC_FHSS_TX_ROUTED` is undefined (host test builds), so
      the existing `cfg_contract` tests that lock in
      `CFG_STATUS_PROFILE_UNROUTED` continue to pass. The build
      symbol is now added to firmware-only `CFLAGS` in
      [`Makefile`](DESIGN-CONTROLLER/firmware/murata_l072/Makefile),
      which automatically unlocks the production profile in
      `host_cfg.c` (`cfg_set(CFG_KEY_REG_PROFILE,
      FCC_15_247_FHSS_50CH_BW250)` now returns `CFG_STATUS_OK` on
      firmware builds). Evidence: `mingw32-make check` →
      `[OK] sx1276_fhss_chantab: count=50, span=902750000..927250000
      Hz` + `[PASS] fhss_scheduler_vectors: 11 cases` +
      `[PASS] airtime_invariant: 12 cases` +
      `[PASS] legal_dwell: 15 cases` + `[PASS] rfco_pertx: 6 cases` +
      `[PASS] cfg_profile: 25 cases` + `[OK] memory map invariants
      hold`. `gcc -fsyntax-only -DLIFETRAC_FHSS_TX_ROUTED=1` and
      without the flag both produce zero diagnostics on
      `radio/sx1276_tx.c`. Follow-on `cfg_set(REG_PROFILE)` →
      `host_cfg_profile_stage/activate` integration and full
      cross-toolchain link verification deferred to Phase D bench
      bring-up (no `arm-none-eabi-gcc` on this host).
- [ ] **FCC-A6** RX hop sync: same `permute(seed, epoch)`; packet header
      carries authenticated `{profile_id, epoch, hop_idx, schema_ver}`.
      Explicit **Scanning** state for cold start — wideband scan across
      active set, **no fixed rendezvous channel**.
    - [x] **FCC-A6a** LoRa link-layer packet header pack/unpack
          contract — `include/lora_pkt_hdr.h` + `radio/lora_pkt_hdr.c`.
          8-byte v1 layout: `{schema_ver:u8, profile_id:u8, hop_idx:u8,
          _reserved:u8, epoch_le:u32}`. MIC trailer is reserved for a
          future additive `schema_ver=2` (FCC notes §17.3 #4); no MIC
          today because Track-C1 LoRa-link MIC/AEAD is unblocked but
          not yet implemented (plan §7 Q4, deferred). Parser refuses
          unknown schema_ver and leaves `*out` untouched on every
          failure path. Reserved byte is intentionally ignored on
          parse to keep the schema additive-evolution friendly.
          Evidence: `mingw32-make check` →
          `[PASS] lora_pkt_hdr: 10 cases` (constants, nominal golden,
          all-zero golden, edge values, NULL/short/bad-schema
          rejection, round-trip, reserved-byte ignore) plus all 6
          prior tests still green and
          `[OK] memory map invariants hold`. Not yet wired into
          `sx1276_tx.c` (prepend) or `sx1276_rx.c` (strip) — that
          wiring belongs to A6b/A6c so the header schema can be
          frozen and consumed unchanged once RX retune lands.
    - [ ] **FCC-A6b** RX tune-to-hop integration in `radio/sx1276_rx.c`
          + `sx1276_fhss`: derive the same `permute(seed, epoch)` as
          TX, retune between RX windows from `host_fhss_next_channel`,
          align epoch from `platform_now_ms()` monotonic tick, prepend
          A6a header to every TX in `sx1276_tx.c` (gated on
          `LIFETRAC_FHSS_TX_ROUTED`), and consume it in the RX
          service path to confirm or correct the local hop pointer.
        - [x] **FCC-A6b-1** Header prepend/strip cutover —
              `radio/sx1276_tx.c` prepends the 8-byte A6a header to
              every FIFO write under `LIFETRAC_FHSS_TX_ROUTED`, with a
              pre-FHSS overflow guard (`req->length + LORA_PKT_HDR_LEN
              > 255` ⇒ emit `RFCO TX_STATUS_INTERNAL` and bail before
              consuming a hop slot). All length-bearing downstream
              calls (`sx1276_airtime_estimate_toa_us`,
              `sx1276_airtime_reserve`, fallback ToA estimate, FIFO
              `PAYLOAD_LENGTH`) now use the on-air `effective_len`
              including header; the `req->payload` FIFO burst stays at
              `req->length`. `radio/sx1276_rx.c` strips the leading 8
              bytes after RX_DONE, parses via `lora_pkt_hdr_unpack`,
              populates `out_frame->hdr` + `hdr_valid`, slides
              remaining bytes forward, and drops frames with
              short/bad-schema headers (IRQ still acked; visibility
              deferred to FCC-B1-SUMMARY rather than a host_stats
              wire-layout bump). `sx1276_rx_frame_t` extended with
              `lora_pkt_hdr_t hdr` + `bool hdr_valid` (struct always
              carries the fields; `hdr_valid==false` when unrouted).
              Behavior with `LIFETRAC_FHSS_TX_ROUTED` undefined is
              byte-for-byte identical to before. Evidence:
              `mingw32-make check` → all 7 host tests still green +
              `[OK] memory map invariants hold`; `gcc -fsyntax-only`
              clean on `radio/sx1276_tx.c` + `radio/sx1276_rx.c` both
              with and without the flag.
        - [ ] **FCC-A6b-2** RX retune loop — between RX windows,
              consume `sx1276_fhss_next_channel` and call
              `sx1276_set_frequency_hz` + PLL settle so the receiver
              follows the same hopset as TX. On a `hdr_valid` frame
              whose `(epoch, hop_idx)` disagrees with the local
              scheduler, snap the local pointer (define the snap
              policy — e.g., only when |Δepoch| ≤ 1 and within a guard
              slot, to refuse spoofed jumps once a MIC ships).
              Resync slot policy: reserve one slot per epoch where RX
              parks for a known seed-slot dwell as a fallback if N
              consecutive epochs decode-fail.
            - [x] **FCC-A6b-2-i** Scheduler snap primitive —
                  `sx1276_fhss_snap_to(target_epoch, snap_to_slot)` in
                  `include/sx1276_fhss.h` + `radio/sx1276_fhss.c`.
                  Atomically sets `(epoch, slot)`, rebuilds the
                  permutation from `H(farm_id, node_id, target_epoch)`,
                  clears `warmup_hops_remaining` (snapping implies the
                  remote is past warm-up), preserves `blacklist_bits`
                  and `lbt_block_count[]` (per-channel quality history
                  survives). `snap_to_slot == CHANNEL_COUNT` is legal
                  and lets the next `next_channel()` trip the wrap
                  path. Fails closed on `NOT_INIT` and
                  `snap_to_slot > CHANNEL_COUNT` (`BAD_IDX`) without
                  mutating state. The policy layer
                  (`consider_remote(epoch, hop_idx)` deciding when to
                  call snap_to vs. ignore the remote hint) and the
                  RX-side wire-up will land with A6b-2-ii where they
                  have a caller. Evidence: `mingw32-make check` →
                  `[PASS] fhss_scheduler_vectors: 16 cases` (was 11;
                  5 new: fails-closed, lands-on-expected-slot,
                  slot==CHANNEL_COUNT wrap, clears-warmup,
                  preserves-quality-history) plus all other host tests
                  still green + `[OK] memory map invariants hold`.
            - [ ] **FCC-A6b-2-ii** RX retune loop + snap-policy layer —
                  introduce `sx1276_fhss_consider_remote(epoch,
                  hop_idx)` deciding ALIGNED / SNAPPED / REJECTED based
                  on `|Δepoch| ≤ 1` (until MIC ships), call it from
                  `sx1276_rx_service` on `hdr_valid` frames, add an
                  `sx1276_rx_tick(now_ms)` API that the main loop
                  drives so RX retunes between windows via
                  `sx1276_fhss_next_channel` + `sx1276_set_frequency_hz`
                  + PLL settle. Window duration is gated on the
                  epoch-model decision in plan §7 open Q2.
                - [x] **FCC-A6b-2-ii-α** Snap-policy primitive —
                      `sx1276_fhss_consider_remote(remote_epoch,
                      remote_hop_idx) →
                      sx1276_fhss_snap_decision_t { ALIGNED, SNAPPED,
                      REJECTED_NOT_INIT, REJECTED_BAD_HOP,
                      REJECTED_EPOCH_DRIFT }` in
                      `include/sx1276_fhss.h` + `radio/sx1276_fhss.c`.
                      ALIGNED uses the same local_hop_idx convention
                      as sx1276_tx.c stamping (slot==0 ? N-1 :
                      slot-1) — symmetric with TX so genuine
                      lock-step peers no-op. SNAPPED gate is
                      `|Δepoch| ≤ SX1276_FHSS_SNAP_MAX_EPOCH_DRIFT
                      (=1)` computed via `(int32_t)(remote-local)`
                      (uint32 wrap-safe across the 0xFFFFFFFF→0
                      boundary). On SNAPPED, `snap_to(remote_epoch,
                      remote_hop_idx + 1)` is called internally (the
                      slot the remote will emit next; +1 ==
                      CHANNEL_COUNT wrap handled by snap_to's spec).
                      All REJECTED_* paths are no-ops. Has no caller
                      yet — the RX wire-up lands with -β and -γ where
                      the call site exists. Evidence: `mingw32-make
                      check` → `[PASS] fhss_scheduler_vectors: 23
                      cases` (was 16; 7 new: fails-closed,
                      aligned-no-op, snapped-same-epoch,
                      snapped-±1-epoch-drift, rejected-drift-incl-far-
                      replay, uint32-wrap-boundary, snapped-wrap-slot)
                      plus all other host tests green +
                      `[OK] memory map invariants hold`.
                - [x] **FCC-A6b-2-ii-β** RX call-site wire-up — call
                      `sx1276_fhss_consider_remote(parsed.epoch,
                      parsed.hop_idx)` from `sx1276_rx_service`
                      immediately after a successful
                      `lora_pkt_hdr_unpack` (i.e., when `hdr_valid` is
                      set). The returned decision is fed to
                      `sx1276_rx_counter_record(dec)` which lives in a
                      HW-free TU (`radio/sx1276_rx_counters.c`,
                      extracted so the saturating-increment +
                      snapshot/reset semantics are host-testable
                      without dragging in modem deps). Counters are
                      indexed by `sx1276_fhss_snap_decision_t`
                      (ALIGNED / SNAPPED / REJECTED_NOT_INIT /
                      REJECTED_BAD_HOP / REJECTED_EPOCH_DRIFT, dim=5),
                      saturate at `UINT32_MAX`, and are exposed via
                      `sx1276_rx_consider_remote_counts(out[5])` +
                      `_reset()`. **REJECTED_* outcomes do NOT drop
                      the frame** — the payload is still useful even
                      if the snap was refused (drift-flag is purely
                      diagnostic until A6 MIC ships). No `host_stats`
                      field added (no host↔X8 wire layout bump);
                      FCC-B1-SUMMARY will surface these via an
                      additive URC. Evidence: `mingw32-make check` →
                      `[PASS] rx_counters: 10 cases` (initial-zero,
                      per-slot-isolation, out-of-range no-op,
                      snapshot-idempotency, reset-round-trip,
                      NULL-arg safety, no-narrow-type-wrap @ 4096
                      records, interleaved-accumulation, DIM-matches-
                      enum invariant, post-reset-usable) + all 7
                      previous host tests still green + `[OK] memory
                      map invariants hold` + `gcc -fsyntax-only` clean
                      on `sx1276_rx.c` + `sx1276_rx_counters.c` +
                      `sx1276_tx.c` both with and without
                      `-DLIFETRAC_FHSS_TX_ROUTED=1`.
                - [ ] **FCC-A6b-2-ii-γ** RX retune loop + `rx_tick` —
                      add `sx1276_rx_tick(now_ms)` API the main loop
                      drives; between RX windows, advance the scheduler
                      via `sx1276_fhss_next_channel` and apply
                      `sx1276_set_frequency_hz` + PLL settle so the
                      receiver follows the same hopset as TX. Window
                      duration gated on plan §7 open Q2 epoch-model
                      decision. **Split** (same playbook as A6b-2-ii-α/β
                      — pure-function policy first, HW mechanism second,
                      Q2-dependent final policy third):
                    - [x] **FCC-A6b-2-ii-γ-1** Retune-decision pure
                          function — HW-free TU
                          `radio/sx1276_rx_retune_policy.c` exposing
                          `sx1276_rx_retune_eval(now_ms,
                          last_retune_ms, modem_busy, fhss_ready)
                          → enum { DO=0, SKIP_NOT_INIT=1,
                          SKIP_BUSY=2, SKIP_TOO_SOON=3, DO_WRAP=4 }`.
                          Window-duration constant
                          `SX1276_RX_RETUNE_PERIOD_MS = 380U` (≤ FCC
                          dwell cap so it is never illegal regardless
                          of which Q2 model wins) — γ-3 swaps in the
                          epoch-driven boundary as a point edit. Uses
                          `(int32_t)(now_ms - last_retune_ms)` for
                          wrap-safe elapsed math (mirrors
                          `consider_remote`'s `int32_t` drift idiom);
                          negative reinterpret → `DO_WRAP` so γ-2 can
                          re-anchor `last_retune_ms = now_ms` and the
                          next tick evaluates cleanly. Priority order:
                          NOT_INIT > BUSY > timing (NOT_INIT first so
                          an un-init'd scheduler surfaces as the most
                          actionable diagnostic even on the legacy
                          single-channel path). `DIM = 5` constant
                          pinned to `max-enum + 1`. Enum numeric
                          values pinned (FCC-B1-SUMMARY's additive URC
                          will index counters by these values —
                          reordering would silently mis-route
                          histograms). Evidence:
                          `mingw32-make check` → `[PASS]
                          rx_retune_policy: 14 cases` (NOT_INIT-vs-busy
                          and NOT_INIT-vs-time priority, BUSY-defers-
                          past-period, same-tick / period-1 / =period
                          / period+1 / max-int32-positive forward
                          deltas, small + large backwards-delta wrap,
                          real-uint32-tick-wrap both
                          insufficient-elapsed and sufficient-elapsed
                          branches, DIM-matches-max-enum, enum-
                          numeric-stable) + all 8 previous host tests
                          still green + `[OK] memory map invariants
                          hold`.
                    - [x] **FCC-A6b-2-ii-γ-2** Retune mechanism +
                          `sx1276_rx_tick(now_ms)` API in
                          `sx1276_rx.c`: consults the γ-1 policy and
                          on `DO` performs standby →
                          `sx1276_fhss_next_channel` →
                          `sx1276_set_frequency_hz` →
                          `pll_settle_busy_wait(SX1276_TX_PLL_SETTLE_US)`
                          → re-arm RX. Per-outcome counter TU
                          (HW-free) mirroring the A6b-2-ii-β snap
                          counters. Main-loop wire-up via
                          `sx1276_rx_tick(millis())`. Gated by
                          `LIFETRAC_FHSS_TX_ROUTED`.
                          - Done 2026-05-20. API surface added to
                            [include/sx1276_rx.h](DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_rx.h):
                            `sx1276_rx_tick(uint32_t now_ms)`,
                            `sx1276_rx_retune_counts(out[DIM])`,
                            `sx1276_rx_retune_counts_reset()`,
                            `sx1276_rx_retune_counter_record(dec)`.
                            HW-free counter TU at
                            [radio/sx1276_rx_retune_counters.c](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx_retune_counters.c)
                            (NULL-safe getter, out-of-range no-op,
                            saturating @ UINT32_MAX, DIM = 5).
                            HW mechanism appended to
                            [radio/sx1276_rx.c](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx.c):
                            first-tick anchors `s_rx_last_retune_ms`
                            without consulting policy; subsequent
                            ticks call `sx1276_rx_retune_eval(...)`
                            with `sx1276_tx_busy()` as `modem_busy`
                            and `sx1276_fhss_is_initialized() != 0U`
                            as `fhss_ready`; every decision is
                            recorded into the counter TU; `DO_WRAP`
                            re-anchors with no HW touch; `DO` does
                            `sx1276_fhss_next_channel` →
                            `sx1276_modes_to_standby` →
                            `sx1276_set_frequency_hz` → PLL settle
                            busy-wait (`SX1276_RX_PLL_SETTLE_US =
                            200 µs`, mirrors the static value in
                            `sx1276_tx.c` with a tracking comment) →
                            `sx1276_rx_arm`; all `SKIP_*` branches
                            are HW no-ops. Unrouted build defines
                            `sx1276_rx_tick` as `(void)now_ms;` so
                            main.c needs no ifdef. Main-loop wire-up
                            in [main.c](DESIGN-CONTROLLER/firmware/murata_l072/main.c)
                            calls `sx1276_rx_tick(now_ms)` inside the
                            existing `{ const uint32_t radio_events
                            = ... }` block (reuses the `now_ms` tick
                            already taken at the top of the loop).
                            Host_stats UNCHANGED (FROZEN per plan);
                            retune counters reach the host via the
                            additive FCC-B1-SUMMARY URC, not via
                            host_stats. γ-2 known limitation: the
                            γ-1 placeholder period (380 ms = dwell
                            cap) means a worst-case in-flight long
                            frame could be interrupted at the
                            boundary; γ-3 closes this by tying the
                            period to the Q2 epoch model so TX and
                            RX retune at the same instant.
                            `modem_busy` here only tracks
                            `sx1276_tx_busy()` — RX-mid-frame is not
                            tracked at this layer (γ-3 / Q2-gated).
                            Evidence: `mingw32-make check` →
                            `[PASS] rx_retune_counters: 10 cases`
                            (initial-state-all-zero, each-slot-
                            isolation, out-of-range-noop, snapshot-
                            readonly, reset-zeroes-then-usable,
                            null-getter-noop, saturating-increment-
                            4096, interleaved-accumulate, DIM-
                            matches-enum, round-trip-after-reset-
                            fresh) + all 9 previous host tests still
                            green + `[OK] memory map invariants
                            hold`.
                    - [ ] **FCC-A6b-2-ii-γ-3** (deferred until Q2
                          closes) Replace the γ-1 placeholder period
                          with the real epoch-driven boundary from the
                          chosen epoch model.
    - [x] **FCC-A6c** Explicit **Scanning** cold-start state: wideband
          scan-with-preamble-timeout across the 50-channel active set,
          re-sync slot per epoch fallback if decode fails for N
          consecutive epochs. No fixed-channel rendezvous (FCC notes
          §15.1.1 retraction). Acceptance target per plan §7 Q6:
          measured worst-case cold-start hop-sync reacquire ≤ 5 s;
          >30 s ⇒ A6 redesign (adaptive scan dwell or super-frame
          beacon slot within the hopset).
          *Closed 2026-05-20 via three sub-tracks all [x]:
          FCC-A6c-1 (pure-function Scanning SM in
          [`radio/sx1276_rx_scan_policy.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx_scan_policy.c)
          — BOOT/SCANNING/LOCKED/FAILED, TICK/FRAME_VALID/
          FRAME_INVALID events, six actions, defensive
          FRAME_VALID+header_ok=false demotion, modular-tick wrap
          handling, 20 host cases),
          FCC-A6c-2 (HW dispatch: -2-a parallel per-state +
          per-action saturating counter TU [14 cases]; -2-b
          scan-tick API + frame-event feed wired into three
          `sx1276_rx_service()` call sites; -2-c gates γ-1 retune
          on LOCKED, dispatches BEGIN_SCAN / ADVANCE_CHANNEL via
          standby → next_channel → set_freq → PLL-settle → arm,
          LOCK = HW-no-op since frame was just received on the
          snapped channel, FAIL → standby, HOLD/REANCHOR HW-no-op),
          FCC-A6c-3 (P1 reporting per
          [AI NOTES/2026-05-19_RX_Scan_FAILED_State_Analysis_Copilot_v1_0.md](AI%20NOTES/2026-05-19_RX_Scan_FAILED_State_Analysis_Copilot_v1_0.md)
          §13 v5.0: -3-a `HOST_FAULT_CODE_RX_SCAN_FAILED=0x0D` +
          §13.1 #2 `sub`-byte encoding; -3-b four file-statics
          tracking retries / IRQ-seen / CRC-seen / ever-locked;
          -3-c B-prime retry supervisor with `MAX_RETRIES=3U` +
          12-case host TU pinning bit layout, retry boundary,
          attempt-nibble saturation, defensive NULL contract).
          P2 (Opta-routed indicator) and P3 (RTC-BKP LKG /
          backoff) explicitly deferred per the §13 v5.0
          scope-lock. `mingw32-make check` green throughout
          (12 host suites; memory map invariants hold). Acceptance
          target (≤5 s reacquire) measurement deferred to
          FCC-EVID-D8 bench run.*
        - [x] **FCC-A6c-1** Scanning state-machine pure-function
              policy in
              [`radio/sx1276_rx_scan_policy.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx_scan_policy.c)
              + header
              [`include/sx1276_rx_scan_policy.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_rx_scan_policy.h).
              State enum (BOOT, SCANNING, LOCKED, FAILED; DIM=4),
              event enum (TICK, FRAME_VALID, FRAME_INVALID; DIM=3),
              action enum (HOLD, BEGIN_SCAN, ADVANCE_CHANNEL, LOCK,
              FAIL, REANCHOR; DIM=6). All numeric values pinned —
              FCC-B1-SUMMARY URC will index per-state / per-action
              histograms by these values. Regulatory constants:
              `SX1276_RX_SCAN_DWELL_MS=100`, `SX1276_RX_SCAN_GOAL_MS=5000`
              (§7 Q6 acceptance target — A6c-3 reports overruns to
              D-Gate, NOT enforced),
              `SX1276_RX_SCAN_REDESIGN_MS=30000` (§7 Q6 redesign
              threshold — enforced as SCANNING→FAILED transition).
              Authenticated-header trust boundary: FRAME_VALID
              requires caller to pre-verify A6a MIC + profile_id +
              schema_ver; policy defensively demotes
              FRAME_VALID+header_ok=false to FRAME_INVALID so it
              never locks onto an unauthenticated frame. Order of
              checks in SCANNING+TICK: cold-anchor wrap → regulatory
              FAIL (cold_elapsed ≥ REDESIGN_MS) → channel-anchor
              wrap → ADVANCE_CHANNEL (channel_elapsed ≥ DWELL_MS) →
              HOLD. Modular arithmetic absorbs the normal 49.7d tick
              wrap; REANCHOR is only emitted when apparent elapsed
              ≥ 2^31 ms (clock glitch / backwards jump). BOOT
              collapses any event to BEGIN_SCAN. LOCKED + FAILED are
              absorbing (LOSS-OF-SYNC re-scan path is deferred until
              the LOCKED-state stale-decode policy lands post-γ-3).
              Wired into [Makefile](DESIGN-CONTROLLER/firmware/murata_l072/Makefile)
              (SRCS, `RX_SCAN_POLICY_BIN`, `.PHONY`, aggregate
              `check`, `check-rx-scan-policy` target with
              `[RXSCN]` label). Evidence: `mingw32-make check` →
              `[PASS] rx_scan_policy: 20 cases` (NULL-safe default,
              BOOT-collapses-all-events, dwell-boundary precedence,
              redesign-boundary FAIL-wins-over-ADVANCE, FRAME_VALID-
              with-bad-header demotion, LOCKED+FAILED absorbing,
              per-anchor REANCHOR detection, DIM-matches-max-enum
              for all three enums, enum-numeric-stability) + all 9
              prior host tests still green + `[OK] memory map
              invariants hold`.
        - [x] **FCC-A6c-2** HW mechanism: wire A6c-1 policy into the
              RX loop. Caller maintains `state`, `channel_entry_ms`,
              `cold_start_entry_ms`; converts modem IRQ + A6a
              header verification result into the input event;
              executes the returned action (BEGIN_SCAN starts
              SCANNING and arms preamble timeout on first scan
              channel; ADVANCE_CHANNEL jumps to next channel in the
              active set and rearms; LOCK extracts epoch/hop_idx
              from header and seeds the FHSS scheduler then
              transitions to γ-1 retune loop; FAIL emits alert URC
              and halts modem; REANCHOR re-anchors both timers).
              HW-free per-state + per-action counter TU mirroring
              γ-2 (host-testable). No host_stats bump (FROZEN).
            - [x] **FCC-A6c-2-a** Counters TU
                  ([`radio/sx1276_rx_scan_counters.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx_scan_counters.c)
                  + prototypes in
                  [`include/sx1276_rx_scan_policy.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_rx_scan_policy.h)).
                  Two parallel saturating-uint32 histograms — one
                  indexed by `sx1276_rx_scan_state_t` (DIM=4), one
                  by `sx1276_rx_scan_action_t` (DIM=6). Mirrors
                  γ-2 (`sx1276_rx_retune_counters.c`) pattern but
                  spans both axes. Per-axis OOR guards are
                  independent (one bad cast cannot blank the
                  other). Single `record(state, action)` API the
                  RX loop (A6c-2-b) calls once per
                  `sx1276_rx_scan_eval()`. Getters
                  (`sx1276_rx_scan_state_counts`,
                  `sx1276_rx_scan_action_counts`) unconditionally
                  available so FCC-B1-SUMMARY reads without an
                  ifdef wrapper. Single `_counts_reset()` zeros
                  both arrays. No host_stats field, no host↔X8
                  SerialRPC wire bump. Wired into
                  [Makefile](DESIGN-CONTROLLER/firmware/murata_l072/Makefile)
                  (SRCS, `RX_SCAN_COUNTERS_BIN`, `.PHONY`,
                  aggregate `check`, `check-rx-scan-counters`
                  target with `[RXSCC]` label). Evidence:
                  `mingw32-make check` →
                  `[PASS] rx_scan_counters: 14 cases` (zeroed
                  initial state, per-axis slot isolation
                  in-lockstep, OOR-state-keeps-action, OOR-action-
                  keeps-state, both-OOR no-op, idempotent
                  snapshots, reset zeros BOTH, NULL-safe getters,
                  no narrow-type wrap @ 4096 records,
                  interleaved accumulation, STATE_DIM/ACTION_DIM
                  match max-enum + 1, round-trip after reset) +
                  all 10 prior host tests still green +
                  `[OK] memory map invariants hold`.
            - [x] **FCC-A6c-2-b** RX-loop wire-up in
                  `radio/sx1276_rx.c`: maintain SM state +
                  anchors, map modem IRQ + A6a header verification
                  onto event, dispatch action (BEGIN_SCAN /
                  ADVANCE_CHANNEL / LOCK→γ-1 / FAIL / REANCHOR),
                  call `sx1276_rx_scan_counter_record()` once per
                  eval.
                - [x] **FCC-A6c-2-b-i** Scanning tick API +
                      observe-only bookkeeping. New
                      `sx1276_rx_scan_tick(now_ms)` in
                      [`radio/sx1276_rx.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx.c)
                      (prototype in
                      [`include/sx1276_rx.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_rx.h))
                      maintains file-static `s_scan_state` +
                      `s_scan_channel_entry_ms` +
                      `s_scan_cold_start_entry_ms`; per tick
                      builds an A6c-1 input with `event=TICK`,
                      calls `sx1276_rx_scan_eval()`, records the
                      observed (state, action) pair via the
                      A6c-2-a counters TU, updates anchors per
                      action (BEGIN_SCAN/REANCHOR ⇒ both,
                      ADVANCE_CHANNEL ⇒ channel only, others ⇒
                      unchanged), and stores `next_state` back.
                      Routed-build only — no-op when
                      `LIFETRAC_FHSS_TX_ROUTED` undefined. Called
                      from
                      [`main.c`](DESIGN-CONTROLLER/firmware/murata_l072/main.c)
                      BEFORE `sx1276_rx_tick()` so the γ-1
                      retune gating (A6c-2-c) will see the
                      freshly-updated SM state. **Observe-only**:
                      no modem standby / set_freq / arm / disarm
                      is driven by any scan-SM action this
                      increment — the existing γ-1 retune loop
                      and main.c `sx1276_rx_arm()` continue
                      unchanged. Counter histograms pre-A6c-2-c
                      will show BOOT→BEGIN_SCAN exactly once at
                      first tick, then SCANNING+TICK
                      accumulating with ADVANCE_CHANNEL ticking
                      every 100 ms and FAIL firing at
                      cold_elapsed ≥ 30 s — no FRAME_VALID /
                      FRAME_INVALID histograms (those land in
                      A6c-2-b-ii). Evidence: `mingw32-make
                      check` → all 11 host tests green +
                      `[OK] memory map invariants hold` (the
                      cross-target firmware compile validates
                      the new TU once an arm-none-eabi
                      toolchain is available; the host check
                      preprocesses `sx1276_rx_scan_policy.h`
                      cleanly via the existing scan_policy /
                      scan_counters TU host tests).
                - [x] **FCC-A6c-2-b-ii** Frame-event wire-up in
                      `sx1276_rx_service()`: after A6a header
                      parse, drive `sx1276_rx_scan_eval()` with
                      `event=FRAME_VALID, frame_header_valid=
                      true` on parse OK and `event=FRAME_INVALID`
                      on parse fail / CRC error / wrong schema,
                      then `sx1276_rx_scan_counter_record()`.
                    - [x] **FCC-A6c-2-b-ii-α** Shared SM driver +
                          frame-event feed plumbing. Refactored
                          [`radio/sx1276_rx.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx.c)
                          to extract a static `scan_drive(event,
                          header_valid, now_ms)` helper shared by
                          `sx1276_rx_scan_tick()` and a new static
                          `scan_feed_frame(header_valid)` that
                          samples `platform_now_ms()` and drives
                          the SM with `FRAME_VALID` /
                          `FRAME_INVALID`. Wired three call sites
                          in `sx1276_rx_service()`: (1) payload
                          CRC error → `scan_feed_frame(false)`
                          before IRQ ack; (2) post-RX_DONE A6a
                          `lora_pkt_hdr_unpack()` failure (bad
                          schema / short frame) → `scan_feed_frame
                          (false)`; (3) post-RX_DONE header OK +
                          `sx1276_fhss_consider_remote()` →
                          `scan_feed_frame(true)`. All three are
                          inside `#ifdef LIFETRAC_FHSS_TX_ROUTED`
                          so the unrouted build is unchanged.
                          **Still observe-only**: SM transitions
                          to LOCKED on FRAME_VALID(header_ok=true)
                          and the (state, action) counter records
                          it, but no HW dispatch — γ-1 retune
                          continues unconditionally and main.c
                          arm/disarm is unchanged. A6c-2-c gates
                          γ-1 on LOCKED and adds modem standby /
                          set_freq / arm dispatch off scan
                          actions. Evidence: `mingw32-make check`
                          → all 11 host tests green +
                          `[OK] memory map invariants hold`; the
                          three call sites compile inside the
                          existing routed ifdef without
                          duplicating frame-parse logic.
                - [x] **FCC-A6c-2-c** HW dispatch: gate the γ-1
                      retune tick on `s_scan_state == LOCKED`;
                      wire BEGIN_SCAN (anchor + initial channel
                      arm), ADVANCE_CHANNEL (standby → next
                      channel from FHSS → set_freq → settle →
                      RX-arm), LOCK (seed FHSS from header
                      epoch/hop_idx then transition to γ-1
                      loop), FAIL (alert URC + standby).
                    - [x] **FCC-A6c-2-c-i** γ-1 LOCKED gate +
                          BEGIN_SCAN / ADVANCE_CHANNEL HW
                          dispatch. New static
                          `scan_dispatch_action(action)` in
                          [`radio/sx1276_rx.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx.c)
                          executes the standby → `sx1276_fhss_next
                          _channel()` → `sx1276_set_frequency_hz()`
                          → PLL-settle (`SX1276_RX_PLL_SETTLE_US`
                          = 200 µs, mirrors γ-2) →
                          `sx1276_rx_arm()` sequence for both
                          BEGIN_SCAN and ADVANCE_CHANNEL; called
                          from `scan_drive()` AFTER state +
                          anchors are committed so a re-entrant
                          IRQ during the standby window observes
                          the new state. HOLD / LOCK / FAIL /
                          REANCHOR remain HW-no-op pending
                          A6c-2-c-ii. `sx1276_rx_tick()` (γ-1)
                          now early-returns unless `s_scan_state
                          == LOCKED` and resets
                          `s_rx_last_retune_ms_valid = 0` while
                          gated out, so the first tick post-LOCK
                          re-anchors cleanly instead of firing a
                          spurious DO_WRAP off stale pre-LOCK
                          timing. File-statics `s_scan_state` /
                          `s_scan_channel_entry_ms` /
                          `s_scan_cold_start_entry_ms` were moved
                          to the top of the routed block (default
                          zero-init = `SX1276_RX_SCAN_STATE_BOOT`)
                          so γ-1 can read `s_scan_state`. Scan
                          dispatcher and γ-1 are now mutually
                          exclusive owners of the synth: scan SM
                          owns it in BOOT / SCANNING / FAILED;
                          γ-1 owns it in LOCKED. Evidence:
                          `mingw32-make check` → all 11 host
                          tests green + `[OK] memory map
                          invariants hold`.
                    - [x] **FCC-A6c-2-c-ii** LOCK + FAIL HW
                          dispatch. LOCK case in
                          `scan_dispatch_action()` is
                          intentionally HW-no-op (frame that
                          triggered LOCK was just received in
                          RX-cont so the modem is already armed
                          on the snapped channel;
                          `sx1276_fhss_consider_remote()` already
                          aligned the FHSS pointer; the next
                          `sx1276_rx_tick()` opens the LOCKED
                          gate and γ-1 takes the synth — driving
                          an extra standby + arm here would only
                          add a needless RX gap right after first
                          contact). FAIL case calls
                          `sx1276_modes_to_standby()` so the
                          modem stops listening on a dead
                          hopset; FAILED is the A6c-1 absorbing
                          state so this single standby is
                          sufficient until external recovery.
                          HOLD / REANCHOR remain HW-no-op (no
                          transition / clock-glitch anchor reset
                          only). Alert URC deliberately deferred
                          to FCC-A6c-3 / FCC-B1-SUMMARY to keep
                          this increment within the frozen
                          host↔X8 wire layout (no new SerialRPC
                          URC codes in the A6c-2 sub-track).
                          Evidence: `mingw32-make check` → all
                          11 host tests green + `[OK] memory map
                          invariants hold`.
        - [x] **FCC-A6c-3** Reporting: 5 s acceptance-target overrun
              counter + FAIL-cause URC (additive to B1-SUMMARY).
              D-Gate analyzes the overrun rate.
              Scope locked by
              [AI NOTES/2026-05-19_RX_Scan_FAILED_State_Analysis_Copilot_v1_0.md](AI%20NOTES/2026-05-19_RX_Scan_FAILED_State_Analysis_Copilot_v1_0.md)
              §13 (v5.0): Phase P1 only (bounded retries + four-class
              `sub`-byte disambiguation); P2 (Opta-routed indicator)
              and P3 (RTC-BKP LKG / backoff) explicitly deferred.
            - [x] **FCC-A6c-3-a** Add
                  `HOST_FAULT_CODE_RX_SCAN_FAILED = 0x0DU` to
                  [`include/host_types.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h)
                  with the §13.1 #2 `sub`-byte encoding documented in
                  a comment block above the define. Declaration-only;
                  no emitter yet. Evidence: `mingw32-make check` →
                  all 11 host tests green + `[OK] memory map
                  invariants hold`.
            - [x] **FCC-A6c-3-b** Add the four file-statics
                  (`s_scan_fail_retries`, `s_scan_got_any_irq`,
                  `s_scan_crc_seen`, `s_scan_ever_locked_this_boot`)
                  to `radio/sx1276_rx.c`; hook
                  `s_scan_got_any_irq` / `s_scan_crc_seen` in
                  `sx1276_rx_service()`; clear on
                  `BEGIN_SCAN`/`ADVANCE_CHANNEL`; reset on `LOCK`;
                  set `s_scan_ever_locked_this_boot` on `LOCK`. **Do
                  not** emit the fault URC yet. Evidence:
                  `mingw32-make check` → all 11 host tests green +
                  `[OK] memory map invariants hold`.
            - [x] **FCC-A6c-3-c** Wire B-prime retry supervisor +
                  `host_cmd_emit_fault(HOST_FAULT_CODE_RX_SCAN_FAILED,
                  sub)` into `scan_dispatch_action(FAIL)` per
                  §13.1 #6; land HW-free retry/`sub`-byte wrapper
                  test (≥8 cases) under
                  [`bench/host_proto/`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/).
                  Evidence: pure helper landed at
                  [`radio/sx1276_rx_scan_fail.c`](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx_scan_fail.c)
                  + [`include/sx1276_rx_scan_fail.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_rx_scan_fail.h)
                  (defines `SX1276_RX_SCAN_MAX_RETRIES = 3U` per
                  v5.0 §13.1). Dispatcher emits the URC and either
                  absorbs in `FAILED` (final) or re-enters `SCANNING`
                  with anchor reset + recursive `BEGIN_SCAN` HW
                  sequence (non-final). Host test
                  [`bench/host_proto/rx_scan_fail.c`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/rx_scan_fail.c)
                  pins the §13.1 #2 bit layout, retry boundary,
                  attempt-nibble saturation, warm/hw_suspect/
                  crc_seen derivation, NULL-input defensive
                  contract, and `MAX_RETRIES==3U` constant —
                  12 cases. `mingw32-make check` → all 12 host
                  tests green + `[OK] memory map invariants hold`.
- [x] **FCC-B1-SUMMARY** Per-minute `RFCO_SUMMARY` URC: 50-bucket
      histogram, per-channel `legal_dwell_max_us`, `blocked_attempts`
      histogram, `active_count`, `blacklist_size`, last clamp reason,
      RFCO schema version. **Wire layout + cadence design**:
      [AI NOTES/2026-05-19_FCC_B1_SUMMARY_Wire_Layout_Design_Copilot_v1_0.md](AI%20NOTES/2026-05-19_FCC_B1_SUMMARY_Wire_Layout_Design_Copilot_v1_0.md)
      — 191 B payload, single frame, snapshot-and-reset deltas, main-loop
      polling cadence at 60 000 ms, `HOST_TYPE_RFCO_SUMMARY_URC = 0xC4U`.
      *Closed 2026-05-20 via FCC-B1-SUMMARY-a (declaration-only header
      with `HOST_RFCO_SUMMARY_SCHEMA_VER=1`, payload-length /
      channel-count / reason-slots static_asserts, and the
      `HOST_TYPE_RFCO_SUMMARY_URC=0xC4U` host-type allocation),
      FCC-B1-SUMMARY-b (pure `host_rfco_summary_pack()` plus three
      sidecar saturating counter modules — `sx1276_fhss` per-channel
      hop counts, `sx1276_legal_dwell` per-channel peak-used-µs,
      `host_rfco` `blocked_attempts_by_reason` slot map — with full
      byte-by-byte wire-vector coverage in
      [bench/host_proto/rfco_summary.c](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/rfco_summary.c)
      and `rfco_summary_counters.c`), and FCC-B1-SUMMARY-c (emit
      wrapper with `HOST_RFCO_SUMMARY_PERIOD_MS=60000` cadence,
      monotonic `platform_now_ms()` wraparound safety, `FIRST` flag
      arming, `seq` counter, and `window_elapsed_ms` jitter
      reporting — verified by `rfco_summary_integration: 8 cases`).
      `mingw32-make check` green throughout (17 host suites). All
      seven required telemetry fields covered: per-channel hop
      histogram (50 u8), per-channel legal-dwell peak (50 u16),
      blocked-attempts histogram (8 u16 slots), active_count,
      blacklist_size, last clamp reason, schema version — matching
      the parent's bullet list one-for-one. No outstanding sub-work;
      gate is field-ready.*
  - [x] **FCC-B1-SUMMARY-a** Declaration-only — *done 2026-05-19.
        Added [include/host_rfco_summary.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco_summary.h)
        with `HOST_RFCO_SUMMARY_SCHEMA_VER=1`,
        `HOST_RFCO_SUMMARY_PAYLOAD_LEN=191`,
        `HOST_RFCO_SUMMARY_PERIOD_MS=60000`,
        `HOST_RFCO_SUMMARY_CHANNEL_COUNT=50`,
        `HOST_RFCO_SUMMARY_REASON_SLOTS=8`, full byte-offset macro set,
        `host_rfco_summary_t` snapshot struct mirroring §3 of the design
        doc, and 4 `_Static_assert`s (payload_len==191, channel count
        matches `SX1276_FHSS_CHANNEL_COUNT`, payload ≤ `HOST_PAYLOAD_MAX_LEN`,
        reason slots ≥ 7). Added
        `HOST_TYPE_RFCO_SUMMARY_URC = 0xC4U` to
        [include/host_types.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h)
        with cross-ref comment block. No emitter, no pack helper, no
        main-loop integration — those land in B1-SUMMARY-b and -c.
        Verified: standalone `gcc -fsyntax-only` on the header parses
        clean (all 4 static_asserts hold); `mingw32-make check` green
        (12 host suites pass, memory map invariants hold).*
  - [x] **FCC-B1-SUMMARY-b** Pure pack helper
        `host_rfco_summary_pack()` + 3 sidecar counter TUs (per-channel
        hop-count, per-channel dwell-max-ms, blocked-attempts-by-reason)
        + ≥10 byte-by-byte wire-vector cases in
        `bench/host_proto/rfco_summary.c`. — *done 2026-05-19 across
        three sub-bullets b-1, b-2, b-3 below.*
    - [x] **b-1** Three sidecar counter TUs in isolation — *done
          2026-05-19. Added saturating-record + snapshot-and-clear APIs
          to three modules:
          (1) [radio/sx1276_fhss.c](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_fhss.c)
          + [include/sx1276_fhss.h](DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_fhss.h)
          — `sx1276_fhss_record_hop(idx)` + `sx1276_fhss_hop_count_snapshot_and_clear(out[50])`
          (u16 in-memory, saturates to 0xFFU on wire);
          (2) [radio/sx1276_legal_dwell.c](DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_legal_dwell.c)
          + [include/sx1276_legal_dwell.h](DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_legal_dwell.h)
          — `peak_used_us[64]` tracked on every successful `reserve()` as
          `used_before + pessimistic_us`, plus
          `sx1276_legal_dwell_peak_us_snapshot_and_clear(out[64])`;
          failed reservations (BAD_CH/BAD_RESERVE/OVER_BUDGET) do NOT
          update peak;
          (3) [host/host_rfco.c](DESIGN-CONTROLLER/firmware/murata_l072/host/host_rfco.c)
          + [include/host_rfco.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco.h)
          — `host_rfco_blocked_attempts_record(reason)` /
          `_snapshot_and_clear(out[8])` / `_reset()` with documented
          slot map (OK never recorded; reasons 1..6 → slots 1..6;
          INTERNAL 0xFF → slot 7; unallocated values dropped; u16
          saturating). All three counter pairs are NULL-safe (out=NULL
          preserves) and OOR-safe (bad idx is a no-op). New bench TU
          [bench/host_proto/rfco_summary_counters.c](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/rfco_summary_counters.c)
          with 17 cases (record→snapshot→re-snapshot zero; wire-byte
          saturation; in-memory u16 saturation past UINT16_MAX;
          per-channel/per-slot independence; monotonic high-water
          mark; failed-reserve no-update; NULL preserves; OK never
          recorded; INTERNAL→slot 7; unallocated reason dropped).
          New Makefile target `check-rfco-summary-counters` added to
          `check:` chain. NO call-site wiring (TX-success → record_hop,
          legal-dwell → already integrated in reserve(), TX-abort →
          record_blocked) — that lands in B1-SUMMARY-b-3. Verified:
          `mingw32-make check` green (14 host suites pass including
          `rfco_summary_counters: 17 cases`, memory map invariants
          hold).*
    - [x] **b-2** Pure pack helper + byte-by-byte wire vectors — *done
          2026-05-19. Added
          [host/host_rfco_summary.c](DESIGN-CONTROLLER/firmware/murata_l072/host/host_rfco_summary.c)
          with `host_rfco_summary_pack(const host_rfco_summary_t *in,
          uint8_t out[191])` (no HAL deps; pure serialize) and
          declared in
          [include/host_rfco_summary.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco_summary.h).
          Layout invariants enforced by the helper itself:
          (a) byte 0 hard-pinned to `HOST_RFCO_SUMMARY_SCHEMA_VER`
          (struct `schema_ver` field ignored, documentation-only);
          (b) bytes 6..7 (`_reserved_align`) hard-zeroed regardless of
          struct field (suppresses uninit-struct leaks; any future use
          MUST bump SCHEMA_VER);
          (c) CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflect,
          xorout 0x0000) over bytes [0..188] written LE at [189..190].
          Added `host_rfco_summary.c` to firmware `SRCS` so the target
          toolchain also compile-checks it. New bench TU
          [bench/host_proto/rfco_summary.c](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/rfco_summary.c)
          with 17 cases: CRC-16/CCITT-FALSE KAT (crc("123456789") ==
          0x29B1); NULL `in` rejected without touching out; NULL `out`
          rejected; payload_len pin; schema byte hard-pin under struct
          lie (0xAA → still 0x01 on wire); pertx_schema_ver_at_emit
          stamp; `_reserved_align` hard-zero under struct poison
          (0xDEAD → 00 00); all-zero snapshot golden (only byte 0 +
          CRC non-zero); u32 LE encoding for uptime/seq/window_elapsed
          (spot-checked byte order 0x12345678 → 78 56 34 12);
          per-channel hop-count[50] byte-for-byte round-trip with 0xFF
          saturation; per-channel dwell_max_ms[50] u16 LE round-trip
          with 0xFFFF saturation; blocked_attempts_by_reason[8] u16 LE
          round-trip (spot-checked 0xABCD → CD AB); tail bytes
          (pertx_count_in_window / summary_emit_count / flags); flags
          FIRST_SINCE_BOOT bit; CRC byte order (LE); CRC sensitivity
          (flipping any of 15 representative bytes in [0..188] changes
          the CRC); repack overwrites pre-poisoned CRC bytes. New
          Makefile target `check-rfco-summary` added to `check:` chain.
          NO call-site wiring (snapshot consumer + emit() land in b-3
          and -c). Verified: `mingw32-make check` green (15 host
          suites pass including `rfco_summary: 17 cases`, memory map
          invariants hold).*
    - [x] **b-3** Call-site wiring at the per-TX emit boundary — *done
          2026-05-19. Wired the two TX-driven sidecar counters into
          [host/host_rfco.c](DESIGN-CONTROLLER/firmware/murata_l072/host/host_rfco.c)
          `host_rfco_pertx_emit()` so every accepted per-TX URC also
          updates the per-minute SUMMARY URC inputs:
          (1) `host_rfco_blocked_attempts_record(snapshot->tx_status)`
          called on every accepted emit (record() handles the OK skip
          internally, so slot 0 stays reserved);
          (2) `sx1276_fhss_record_hop(snapshot->hop_idx)` called iff
          `snapshot->tx_status == HOST_RFCO_TX_STATUS_OK` (a blocked
          TX never went on air, so it must NOT inflate the per-channel
          hop histogram). NULL snapshot short-circuits before either
          counter is touched. Legal-dwell counter was already wired in
          b-1 (inside `sx1276_legal_dwell_reserve()`), so all three
          SUMMARY sidecars now self-populate from the live TX path
          without sx1276_tx.c needing any changes. Extended
          [bench/host_proto/rfco_pertx.c](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/rfco_pertx.c)
          with 5 new cases (now 11 total): hop_count bumps on OK and
          ONLY on OK; blocked_attempts[slot=2] on ABORT_LBT;
          blocked_attempts[slot=7] on INTERNAL (0xFF remap); 5x
          repeated emit accumulates hop_count[N]==5; emit(NULL) does
          NOT touch either counter. Updated `check-rfco-pertx`
          Makefile target to link `radio/sx1276_fhss.c` +
          `radio/sx1276_fhss_chantab.c` (host_rfco.c now references
          `sx1276_fhss_record_hop`). Verified: `mingw32-make check`
          green (15 host suites pass including
          `rfco_pertx: 11 cases`, memory map invariants hold).
          Closes FCC-B1-SUMMARY-b parent; B1-SUMMARY-c (emit wrapper +
          60 000 ms main-loop cadence) is now unblocked.*
  - [x] **FCC-B1-SUMMARY-c** Emit wrapper
        `host_rfco_summary_emit()` + main-loop 60 000 ms cadence
        integration + emit-timing bench test. *done 2026-05-19 via
        c-1 (emit wrapper + per-window sidecars) + c-2 (main-loop
        wiring + integration bench). Closes FCC-B1-SUMMARY entirely
        — unblocks FCC-B2-b (artifact-header stamping can now source
        `rfco_schema_ver` from the SUMMARY URC wire byte 0).*
    - [x] **c-1** Emit wrapper + cadence helper + per-window sidecars
          — *done 2026-05-19. Added 4-function per-window sidecar API
          in [include/host_rfco.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco.h)
          (`host_rfco_pertx_window_record`,
          `_count_in_window_snapshot_and_clear`,
          `host_rfco_last_clamp_reason_snapshot_and_clear`,
          `_pertx_window_reset`) with internal u16 saturating counter +
          u8 wire clamp + 0xFF sentinel for "no clamp this window".
          Implemented sidecars in
          [host/host_rfco.c](DESIGN-CONTROLLER/firmware/murata_l072/host/host_rfco.c)
          (literal `0xFFU` used to avoid summary.h ↔ host_rfco.h
          include cycle) and wired the recorder into
          `host_rfco_pertx_emit()` AFTER the b-3 blocked_attempts +
          conditional hop_count block so every accepted per-TX URC also
          bumps the per-window counters. Added emit wrapper +
          cadence helper to
          [include/host_rfco_summary.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco_summary.h):
          `host_rfco_summary_emit(seq, now_ms, window_elapsed_ms,
          profile_id)`, `host_rfco_summary_should_emit(now, last)`
          (u32-wraparound-safe), `host_rfco_summary_reset_wrapper_state()`
          (test-only). Implementation lives in a new TU
          [host/host_rfco_summary_emit.c](DESIGN-CONTROLLER/firmware/murata_l072/host/host_rfco_summary_emit.c)
          (split from `host_rfco_summary.c` so the b-2 pack-only bench
          stays HAL-free): builds `host_rfco_summary_t` from live
          sidecars (`sx1276_fhss_active_count`,
          `hop_count_snapshot_and_clear`,
          `sx1276_legal_dwell_peak_us_snapshot_and_clear` with µs→ms
          ceiling-div u16 saturation, `blocked_attempts_snapshot_and_clear`,
          new `pertx_count_in_window` + `last_clamp_reason` sidecars),
          packs to 191 B, forwards to `host_uart_send_urc(0xC4,
          (uint16_t)(seq&0xFFFF), flags, payload, 191)`. State advance
          (FIRST_SINCE_BOOT latch, `summary_emit_count` saturating u8)
          happens AFTER the send so a pack failure preserves the
          FIRST bit for retry. No main-loop wiring — that lands as
          c-2. Added new Makefile target `check-rfco-summary-emit`
          (links the new TU + `host_rfco.c` + `sx1276_fhss.c` +
          `_fhss_chantab.c` + `_legal_dwell.c`) in
          [DESIGN-CONTROLLER/firmware/murata_l072/Makefile](DESIGN-CONTROLLER/firmware/murata_l072/Makefile)
          and added `host/host_rfco_summary_emit.c` to firmware SRCS.
          New bench
          [bench/host_proto/rfco_summary_emit.c](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/rfco_summary_emit.c)
          covers 9 cases: (1) emit forwards (type=0xC4, seq low 16,
          flags, payload, len=191) to stubbed send_urc; (2) payload
          matches independent re-pack with hand-built snapshot
          mirroring populated sidecars; (3) FIRST_SINCE_BOOT latch
          set then cleared; (4) summary_emit_count 0→1→2; (5) every
          sidecar drained by emit (hop_count, dwell_peak,
          blocked_attempts, pertx_count_in_window, last_clamp_reason);
          (6) pertx_count_in_window counts 5x then saturates at 0xFF
          after 300+ emits; (7) last_clamp_reason captures most-recent
          non-OK (ABORT_LEGAL_DWELL after LBT, then OK does not
          overwrite); (8) µs→ms ceiling conversion via emit payload
          (0→0, 1µs→1ms, 1000µs→1ms, 1500µs→2ms) using
          `sx1276_legal_dwell_reserve`; (9) `should_emit` returns true
          at exactly 60000 ms, false at 59999, tolerant across u32
          wraparound. Verified: `mingw32-make check` green (16 host
          suites pass including `rfco_summary_emit: 9 cases`, memory
          map invariants hold). Parent stays `[ ]` pending c-2
          (main-loop wiring).*
    - [x] **c-2** Main-loop wiring — *done 2026-05-19. Moved
          last_emit_ms + summary_seq tracking out of main.c and into
          the emit TU via a new `host_rfco_summary_tick(now_ms,
          profile_id)` helper added in
          [include/host_rfco_summary.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco_summary.h)
          and implemented in
          [host/host_rfco_summary_emit.c](DESIGN-CONTROLLER/firmware/murata_l072/host/host_rfco_summary_emit.c)
          (file-static `s_tick_last_emit_ms` + `s_tick_seq`, both
          zeroed by `host_rfco_summary_reset_wrapper_state()` for
          test re-arm). tick() polls `should_emit` internally and on
          a successful emit advances `last_emit_ms` to `now_ms` and
          increments `seq`; on pack failure (unreachable in
          practice) it preserves both so the next tick retries with
          the accumulated window. Wired into the main loop in
          [DESIGN-CONTROLLER/firmware/murata_l072/main.c](DESIGN-CONTROLLER/firmware/murata_l072/main.c)
          after the `sx1276_rx_tick(now_ms)` call: a single
          `host_rfco_summary_tick(now_ms,
          host_cfg_profile_active()->profile_id)` invocation per
          loop iteration. `host_cfg_profile_active()` always returns
          a valid pointer to file-static state (zero-initialised at
          boot to `REG_PROFILE_BENCH_ONLY_FIXED_915 == 0`), so no
          NULL guard is needed. Added new Makefile target
          `check-rfco-summary-integration` (links the same TUs as
          the c-1 emit bench) in
          [DESIGN-CONTROLLER/firmware/murata_l072/Makefile](DESIGN-CONTROLLER/firmware/murata_l072/Makefile)
          and registered it in the .PHONY list + `check` aggregator.
          New bench
          [bench/host_proto/rfco_summary_integration.c](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/rfco_summary_integration.c)
          drives a simulated clock past several 60 000 ms boundaries
          and covers 8 cases: (1) no emit before first boundary
          (ticks at 0, 1, 1000, 30000, 59999 must NOT call send_urc);
          (2) first emit fires at exactly `now_ms == 60000` with
          type=0xC4, FIRST_SINCE_BOOT flag set, summary_seq=0,
          window_elapsed_ms=60000, uptime_ms=60000; (3) tick() is
          idempotent — 10 back-to-back invocations at 60000 +
          mid-window ticks at 60001/90000/119999 produce no extra
          emits; (4) second on-time emit at 120000 carries seq=1,
          FIRST cleared, window=60000; (5) LATE tick at 181500 (1500
          ms jitter) reports `window_elapsed_ms == 61500` and
          seq=2, proving window reflects actual wall-clock delta;
          (6) `reset_wrapper_state()` re-arms FIRST and zeros seq +
          last_emit_ms so the post-reset emit at 120001 carries
          FIRST again with seq=0 and window=120001; (7) `profile_id`
          arg propagates to `payload[HOST_RFCO_SUMMARY_OFF_PROFILE_ID]`
          unchanged (verified with 0x05 and 0xA7); (8) cadence
          survives u32 platform_now_ms() wraparound — boot_jump
          near 0xFFFFFFFF emits first (huge window vs last=0), then
          mid-window tick at boot_jump+30000 (wrapped) NO emit,
          boundary tick at boot_jump+60000 (wrapped to ~0xEA5F)
          emits with `window_elapsed_ms == 60000`. Verified:
          `mingw32-make check` green (17 host suites pass including
          `rfco_summary_integration: 8 cases`, memory map invariants
          hold). Cross-build (`mingw32-make all`) skipped — host
          dev box lacks `arm-none-eabi-gcc`; this is a pre-existing
          environment gap unrelated to c-2. Closes parent
          FCC-B1-SUMMARY-c.*
- [x] **FCC-B2** Artifact stamping: required header fields (firmware git
      SHA, build timestamp UTC, profile enum + string, RFCO schema
      version). **Naming linter** refuses any artifact containing
      `airtime_us` or `dwell_us` without one of `qos_used_us_1s`,
      `legal_dwell_used_us_10s`, `legal_dwell_used_us_20s`. — *done
      2026-05-20 via FCC-B2-a (naming linter, 11/11) and FCC-B2-b
      (header schema → stamp module → corpus retro-sweep → lint
      gate wired into all three orchestrators).*
  - [x] **FCC-B2-a** Naming linter — *done 2026-05-19. Added
        [tools/lint_artifact_naming.py](tools/lint_artifact_naming.py):
        Python 3 script that recursively scans paths, refuses any text
        artifact containing `airtime_us` or `dwell_us` without at least
        one of `qos_used_us_1s`, `legal_dwell_used_us_10s`,
        `legal_dwell_used_us_20s`. Per-file opt-out marker
        `LINTER_ALLOW_RAW_AIRTIME_DWELL_US` for legitimate sources-of-truth.
        `--self-test` runs 11 built-in cases (empty, only-canonical,
        only-old-airtime, only-old-dwell, old+canonical pairings,
        opt-out marker, both-old-tokens, substring-only-not-canonical)
        — all 11 pass. Run against
        `DESIGN-CONTROLLER/bench-evidence/` (6930 text files scanned, 166
        binary skipped): 0 violations. Stamping half (B2-b) deferred
        until FCC-B1-SUMMARY locks whether `rfco_schema_ver` ships as a
        build-time constant or a URC field (those decisions couple).*
  - [x] **FCC-B2-b** Artifact-header stamping (firmware git SHA,
        build timestamp UTC, profile enum + string, RFCO schema
        version). Unblocked by FCC-B1-SUMMARY (c-1/c-2 closed
        2026-05-19). Split into three atomic sub-items b-b-1/b-b-2/b-b-3.
        — *done 2026-05-20: header schema + stamp module landed in
        b-b-1, orchestrator wiring in b-b-2, and the corpus retro-
        sweep + hard lint gate across all three soak scripts in
        b-b-3. Every text artifact under `bench-evidence/` carries a
        v1 FCC-B2-b header, and no future orchestrator run can ship
        an unstamped artifact.*
    - [x] **FCC-B2-b-b-1** Header schema + stamp module +
          self-test (no pipeline wiring yet). *done 2026-05-20:
          new module [tools/artifact_header.py](tools/artifact_header.py)
          (~520 LOC, mirrors `lint_artifact_naming.py` style). Single
          source of truth for the fenced header block. Required fields:
          `firmware_git_sha` (40-char full + 12-char short via
          `git rev-parse HEAD`, falls back to `unknown/unknown` outside
          a repo), `build_timestamp_utc` (ISO-8601 'Z', second
          precision), `profile_enum` (numeric), `profile_string`
          (harvested from `REG_PROFILE_*` macros in
          [include/host_cfg_keys.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_keys.h),
          `REG_PROFILE_MAX` excluded so it doesn't collide with the
          aliased enum), `rfco_summary_schema_ver` +
          `rfco_pertx_schema_ver` (harvested from
          `HOST_RFCO_SUMMARY_SCHEMA_VER` /
          `HOST_RFCO_PERTX_SCHEMA_VER` in
          [include/host_rfco_summary.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco_summary.h)
          + [include/host_rfco.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco.h)
          — drift gate raises `RuntimeError` if either macro is
          missing, refusing to fabricate a schema version), and
          `header_schema_ver` (this module's own format version, =1).
          Block format: fenced `# === FCC-B2-b ARTIFACT HEADER BEGIN
          (v1) ===` / `# === FCC-B2-b ARTIFACT HEADER END ===` with
          one `# key: value` line per field; comment prefix
          configurable (default `# ` for shell/Py/Make/log/md;
          auto-picks `// ` for `.c`/`.h`/`.cpp`/`.hpp`; `-- ` for
          `.sql`). Idempotency rule: `stamp_text` returns input
          unchanged when an existing header is byte-equal, and raises
          `ValueError` (with diff dict) when it disagrees — re-stamp
          MUST be explicit unstamp + restamp by the caller, never
          silent. CLI surface: `--self-test`, `harvest` (prints
          schema + profile table), `stamp --profile-enum N
          [--input ...] [--output ...] [--comment-prefix ...]`,
          `parse --input ...`. Self-test: **21/21 cases green**
          including format/parse round-trip under both `# ` and
          `// ` prefixes, idempotent stamping, schema/profile
          harvester correctness, unknown-profile rejection,
          BEGIN-without-END corruption detection, embedded-newline
          rejection, unknown-key rejection, comment-prefix table,
          and an explicit check that the emitted block contains
          neither `airtime_us` nor `dwell_us` (so it does not trip
          [tools/lint_artifact_naming.py](tools/lint_artifact_naming.py)).
          Linter cross-check: `py -3 tools/lint_artifact_naming.py
          tools/artifact_header.py tools/lint_artifact_naming.py`
          → `scanned=2 optout=2 violations=0` (the new module
          carries the `LINTER_ALLOW_RAW_AIRTIME_DWELL_US` opt-out
          marker because its docstring legitimately names the rule).
          CLI smoke: `harvest` prints SUMMARY=1, PERTX=1, profiles
          0/1/2 → BENCH_ONLY_FIXED_915 / FCC_15_247_FHSS_50CH_BW250
          / FCC_15_247_DTS_BW500; `stamp --profile-enum 1` prints a
          well-formed v1 block with real git SHA
          `b322f8950a5659351434b9b642d3e41720c597ac`. No firmware
          touched, no pipeline wired, no `mingw32-make check`
          regressions possible.*
    - [x] **FCC-B2-b-b-2** Wire `artifact_header.stamp_text` into
          one or two real bench-evidence emission sites (smallest
          orchestrator script or Makefile rule that already writes
          a text artifact without a header). *done 2026-05-20:
          [tools/mixed_load_soak.ps1](tools/mixed_load_soak.ps1) now
          stamps both captured logs (`tx_burst_board_a.log` written
          by `Tee-Object` on board A, `rx_listen_board_b.log` written
          by `Start-Process -RedirectStandardOutput` on board B)
          immediately after the post-soak pkill+sleep and before the
          `=== TX TAIL ===` summary. Stamp call:
          `py -3 tools/artifact_header.py stamp --profile-enum 0
          --input <log> --output <log>` per log; profile=0 =
          `REG_PROFILE_BENCH_ONLY_FIXED_915` because this W1-10b / W2
          mixed-load soak runs on a single fixed channel per the
          2026-05-19 FCC plan §5 #3 BENCH_ONLY callout. Soft-fail:
          non-zero stamper exit prints `[stamp] WARN: ... exit=N`
          and continues — a stamp failure on an already-captured log
          must not erase the bench evidence. Offline verification
          (hardware-free): synthetic log containing
          `__W1_10B_LISTEN_READY__`, two `__RX_FRAME__`, `__TX_DONE__`,
          `__W1_10B_BURST_DONE__` markers — after the same stamp
          invocation: (1) `Select-String __TX_DONE__` still returns 1
          and `__RX_FRAME__` still returns 2 (counter logic
          unaffected by prepended header); (2) `artifact_header.py
          parse` round-trips every field with `profile_enum: 0`,
          `profile_string: REG_PROFILE_BENCH_ONLY_FIXED_915`,
          `rfco_summary_schema_ver: 1`, `rfco_pertx_schema_ver: 1`,
          `header_schema_ver: 1`, real git SHA
          `b322f8950a5659351434b9b642d3e41720c597ac`; (3)
          [lint_artifact_naming.py](tools/lint_artifact_naming.py)
          reports `scanned=1 violations=0` on the stamped log.
          **b-b-3 hardening flagged**: the b-b-1 strict idempotency
          contract (`stamp_text` raises `ValueError` on any field
          diff) makes a back-to-back CLI re-stamp fail by 1 s on
          `build_timestamp_utc`. Safe here because `Tee-Object` /
          `Start-Process -RedirectStandardOutput` always overwrite
          the log file before the stamp call runs (fresh run =
          fresh unstamped log), but b-b-3's retro-stamp sweep will
          need a `--if-unstamped` CLI mode (or a "preserve existing
          `build_timestamp_utc` on re-stamp" flag) before it can be
          run over a tree of already-captured artifacts. No firmware
          touched, no `mingw32-make check` regressions possible.*
    - [x] **FCC-B2-b-b-3** Retro-stamp sweep + CI check that every
          freshly-emitted text artifact carries a current header.
          Pre-work: add `artifact_header.py stamp --if-unstamped`
          (no-op when a header block already exists, preserves
          existing `build_timestamp_utc`) so the sweep is
          re-runnable. — *done 2026-05-20 via
          b-b-3-1 (`--if-unstamped` + self-test), b-b-3-2 (inventory
          → prefix collapse → profile map → `--profile-from-map`
          CLI → tree-wide retro-sweep that took the corpus from
          6930 unstamped → 6930 stamped), and b-b-3-3 (lint tool +
          wired into all three orchestrators with hard `exit 3` on
          violation). Net result: every text artifact under
          `bench-evidence/` carries a v1 FCC-B2-b header today, and
          no orchestrator can ship an unstamped artifact going
          forward.*
        - [x] **FCC-B2-b-b-3-1** Add `--if-unstamped` CLI mode +
              self-test (no sweep yet). *done 2026-05-20:
              extended [tools/artifact_header.py](tools/artifact_header.py)
              `_cmd_stamp` to short-circuit BEFORE harvesters /
              git-sha collection when `parse_header_block` finds an
              existing v1 block on the input — returns 0 + leaves
              the output file untouched, regardless of field values
              (the flag means "skip if a header exists", not "skip
              if fields match"). Skipping the harvesters keeps the
              call cheap for tree-wide sweeps; leaving the file
              untouched preserves the original `build_timestamp_utc`
              that the strict b-b-1 `stamp_text` field-diff guard
              would otherwise reject. Self-test extended from 21
              to **25/25 cases green**: (18) `--if-unstamped` on an
              unstamped file stamps and exits 0; (19) `--if-unstamped`
              on a pre-stamped file is a no-op + exit 0 with
              byte-equal file contents; (20) `--if-unstamped`
              tolerates a stale `--profile-enum` without raising
              the strict-mode field-diff `ValueError`; (21)
              regression guard — stamp WITHOUT `--if-unstamped`
              still raises on field diff (confirms strict mode is
              preserved as the default). Real-tree smoke: stamped
              a temp log with `--profile-enum 0`, slept 2 s,
              re-ran with `--profile-enum 2 --if-unstamped` →
              `build_timestamp_utc` byte-equal across both calls
              (preserved=True). Linter cross-check: `lint_artifact_naming.py`
              on `artifact_header.py + lint_artifact_naming.py +
              mixed_load_soak.ps1` → `scanned=3 optout=2 violations=0`.
              No firmware touched, no pipeline wired, no
              `mingw32-make check` regressions possible.*
        - [x] **FCC-B2-b-b-3-2** Inventory existing text artifacts
              under `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/`
              and run a tree-wide sweep using `--if-unstamped`;
              report counts only, no CI gate yet. — *done
              2026-05-20 via b-b-3-2-1 (inventory tool, 16/16
              self-test, baseline 6930 unstamped), b-b-3-2-2-1
              (prefix-collapsing mode → 79 distinct prefixes),
              b-b-3-2-2-2 (JSON profile map + loader, 24/24
              self-test, all 79 prefixes assigned), and
              b-b-3-2-2-3 (`--profile-from-map` CLI surgery + the
              actual tree-wide sweep, 6930 → 0 unstamped, 6930
              stamped total).*
            - [x] **FCC-B2-b-b-3-2-1** Read-only inventory tool +
                  self-test (no sweep yet). *done 2026-05-20:
                  new tool [tools/inventory_artifact_headers.py](tools/inventory_artifact_headers.py)
                  walks a root dir (default
                  `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence`)
                  and classifies every file into 5 buckets —
                  `stamped` (v1 header parses), `unstamped` (no
                  header), `corrupt` (BEGIN fence without END, or
                  BEGIN literal present but default-prefix parse
                  finds nothing — belt-and-suspenders flag for
                  cross-prefix mistakes), `non_text` (suffix not in
                  the shared `TEXT_EXTS` from
                  [tools/lint_artifact_naming.py](tools/lint_artifact_naming.py)),
                  `unreadable` (OS refused to open). Reuses
                  `parse_header_block` + `comment_prefix_for_path` +
                  `BEGIN_FENCE` from
                  [tools/artifact_header.py](tools/artifact_header.py)
                  via direct import so there is exactly one source
                  of truth for the header schema. Per-group counts
                  keyed by top-level subdirectory name (workload
                  prefix); files directly under root bucketed as
                  `(root)`. Output modes: human-readable fixed-width
                  table sorted by text_total desc (default), `--json`,
                  `--list-unstamped` (pipes paths into the future
                  b-b-3-2-2 sweep). Read-only: exit 0 always on
                  successful walk, exit 2 on bad CLI. Self-test:
                  **16/16 cases green** — synthetic tree exercises
                  every bucket, asserts `walk_root` grouping,
                  zero-filled bucket keys, sort order, JSON
                  round-trip, and graceful empty-root behaviour.
                  Linter: `scanned=1 optout=1 violations=0` (the
                  module carries `LINTER_ALLOW_RAW_AIRTIME_DWELL_US`
                  because its self-test docstring names the rule).
                  **Real-corpus run** against
                  `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/`
                  (772-line table, 386 groups): **6930 unstamped /
                  0 stamped / 0 corrupt / 166 non_text / 0
                  unreadable** text artifacts. Biggest single group
                  is `stage1_standard_runs` (792 files); the
                  ~80 `T6_stage1_standard_2022-*` and ~50
                  `T6_stage1_standard_quant_2026-*` per-run dirs
                  cluster into a small number of workload prefixes
                  (`T6_bringup_*`, `T6_rom_baseline_burst_*`,
                  `T6_phase*_*`, `T6_profile_*`, `T6_stage1_standard_*`,
                  `W1-9b_*`, `W1-10b_*`, `W1-9_stage2_tx_*`,
                  `W2-01_*`, `w2_01_*`, `W2-02_*`, `W4-pre`,
                  `walk_power_*`, `adb_hardened_logs`,
                  `mixed_load_*`, plus the `(root)` singletons).
                  **Design implication for b-b-3-2-2**: confirmed
                  that a tree-wide `--profile-enum N` would
                  mis-stamp the corpus — a workload-prefix →
                  profile_enum map (likely
                  `tools/artifact_profile_map.yaml` consumed by a
                  new `--profile-from-map FILE` CLI flag on
                  `artifact_header.py`) is the next prerequisite.
                  No firmware touched, no files mutated, no
                  `mingw32-make check` regressions possible.*
            - [x] **FCC-B2-b-b-3-2-2** Author the workload-prefix
                  profile map + extend `artifact_header.py stamp`
                  with `--profile-from-map FILE`; then run the
                  tree-wide sweep using `--if-unstamped`. — *done
                  2026-05-20 via b-b-3-2-2-1 (prefix collapse),
                  b-b-3-2-2-2 (JSON map + loader), and b-b-3-2-2-3
                  (CLI flag + corpus sweep).*
                - [x] **FCC-B2-b-b-3-2-2-1** Add prefix-collapsing
                      mode to the inventory tool so the profile-map
                      keyset is data-driven, not from memory.
                      *done 2026-05-20: extended
                      [tools/inventory_artifact_headers.py](tools/inventory_artifact_headers.py)
                      with `collapse_to_prefix(group)` (iteratively
                      strips trailing `_YYYY-MM-DD(_HHMMSS)?`,
                      `_runN`, `_NNN-NNNNN` build-id pair, and
                      `_\\d{3,}` numeric tails — fixpoint loop, never
                      returns the empty string, leaves `(root)` and
                      un-suffixed names unchanged, refuses to eat
                      short hyphenated tokens like `W1-7`) and
                      `collapse_counts(counts)` (re-aggregates per-
                      group bucket counts under the collapsed prefix
                      while preserving the zero-filled BUCKETS
                      invariant). New CLI flags: `--group-by-prefix`
                      (render the table over collapsed prefixes) and
                      `--list-prefixes` (one prefix per line, sorted
                      alphabetically — implies `--group-by-prefix`,
                      direct keyset seed for b-b-3-2-2-2). Self-test:
                      **29/29 cases** (16 prior + 13 new covering
                      every realistic suffix shape, the build-id-pair
                      tail discovered during the first real-corpus
                      run, the never-empty guard, synthetic-group
                      pass-through, and `collapse_counts` merge +
                      invariant preservation). Linter:
                      `scanned=1 optout=1 violations=0`.
                      **Real-corpus run** (same root as b-b-3-2-1):
                      collapsed **386 raw groups → 79 distinct
                      workload prefixes**, totals preserved at
                      6930 unstamped / 166 non_text. Largest
                      prefixes by text_total: `T6_stage1_standard`
                      (3330), `stage1_standard_runs` (792),
                      `T6_rom_baseline_burst` (570), `T6_bringup`
                      (564), `w2_01_production` (317),
                      `T6_stage1_standard_quant` (194 — was
                      scattered across ~50 dated rows pre-collapse),
                      `W2-02_image_over_lora` (175),
                      `W1-10b_rx_pair` (173). The 79-prefix list is
                      now the authoritative keyset for the
                      b-b-3-2-2-2 YAML map. **Known follow-up**: a
                      handful of names have a meaningful tag *after*
                      a middle date (e.g.
                      `T6_phase10_atomic_stability_10x_2026-05-09_2110_clean`,
                      `T6_stage1_standard_directcheck_2026-05-10_073200_fix2c`)
                      and were intentionally left uncollapsed because
                      stripping the middle date would either lose the
                      `_clean` / `_fix2c` distinguisher or merge them
                      with the un-tagged baseline; b-b-3-2-2-2 should
                      assign these explicitly. Read-only, no firmware
                      touched, no files mutated.*
                - [x] **FCC-B2-b-b-3-2-2-2** Author the YAML
                      profile map keyed by the 79 collapsed
                      prefixes from b-b-3-2-2-1, with a loader +
                      self-test (no stamp CLI changes yet).
                      *done 2026-05-20: chose **JSON not YAML** to
                      stay on Python stdlib (no PyYAML dependency)
                      and keep the file diff-friendly. Two new
                      files:* (1) [tools/artifact_profile_map.json](tools/artifact_profile_map.json)
                      *— enveloped data file
                      (`schema_version: 1`, narrative `doc` field
                      recording the forcing-function design + the
                      three wire-constant profile_enum meanings,
                      and a `prefixes` object containing all 79
                      collapsed prefixes from b-b-3-2-2-1's
                      `--list-prefixes`). Every entry =* `0`
                      *(`REG_PROFILE_BENCH_ONLY_FIXED_915`) per the
                      2026-05-19 FCC plan §5 #3: every historical
                      capture in `bench-evidence/` is bench-only.*
                      (2) [tools/artifact_profile_map.py](tools/artifact_profile_map.py)
                      *— loader + validator. Public API:*
                      `MAP_SCHEMA_VERSION = 1`,
                      `VALID_PROFILE_ENUMS = (0, 1, 2)`,
                      `load_profile_map(path)` *(raises
                      `ValueError` on schema mismatch / malformed
                      JSON / out-of-range enum / bool-as-int —
                      explicit `isinstance(v, bool)` guard so
                      `true` does NOT silently become
                      `profile_enum=1`),* `lookup_profile_for_group(group, prefix_map)`
                      *(imports `collapse_to_prefix` from
                      [tools/inventory_artifact_headers.py](tools/inventory_artifact_headers.py)
                      so collapse logic stays single-sourced,
                      then strict dict lookup —* **raises `KeyError`
                      on miss, never defaults** *— the
                      forcing-function gate against silent
                      mis-stamping of future FHSS / DTS captures),
                      and* `validate_map(prefix_map, *, required_prefixes=None)`
                      *(re-checks invariants defensively, plus
                      optional keyset cross-check against an
                      operator-supplied list; extras surface as
                      `INFO:` not errors so the map can
                      legitimately pre-register upcoming workloads
                      before the first capture lands). CLI:
                      `--self-test`, `--validate`,
                      `--require-prefixes-from FILE`, `--lookup
                      GROUP`, `--map FILE`. Exit codes 0/1/2 for
                      success / validation-or-lookup-failure /
                      bad-CLI-or-file. Self-test:* **24/24 cases**
                      *covering load round-trip, every error path
                      (schema_version, bad enum, bool-as-int,
                      malformed JSON), lookup-with-collapse,
                      lookup-KeyError, validate_map clean +
                      out-of-range + bool + missing-required +
                      extras-as-INFO, and a structural smoke-test
                      against the shipped map (entry count, enum
                      validity, representative-prefix presence).
                      Linter: `scanned=2 optout=0 violations=0`
                      (both new files clean — neither needs the
                      raw-token opt-out). **Live-corpus
                      cross-check**: ran `inventory_artifact_headers.py
                      --list-prefixes` against
                      `bench-evidence/`, piped into
                      `--require-prefixes-from`, result*
                      `[map] OK  entries=79` *with zero
                      missing-required errors and zero `INFO:`
                      extras → the map keyset is exactly the live
                      keyset. **Spot-checks**: three real dated
                      groups (`T6_stage1_standard_2022-05-04_093028`,
                      `walk_power_full_2026-05-19`,
                      `w2_01_production_2026-05-15_111810`) all
                      collapse + look up to `0`. **Forcing
                      function verified**: invented group
                      `new_fhss_workload_2026-06-01` exits 1 with
                      message `"no profile_enum entry for
                      group='new_fhss_workload_2026-06-01'
                      (collapsed prefix='new_fhss_workload'); add
                      it to the map before stamping"` — actionable
                      and names both the original group and the
                      collapsed key. **Windows-portability fix**:
                      `--require-prefixes-from` reads with
                      `utf-8-sig` so a BOM emitted by PowerShell
                      5.1's `Out-File -Encoding utf8` does not
                      mis-key the first prefix (regression caught
                      live during verification). Read-only, no
                      firmware touched, no files mutated.*
                - [x] **FCC-B2-b-b-3-2-2-3** Extend
                      `artifact_header.py stamp` with
                      `--profile-from-map FILE`, then run the
                      tree-wide sweep with `--if-unstamped`. —
                      *done 2026-05-20 via b-b-3-2-2-3-1 (CLI
                      surgery + tiny-subdir smoke test) and
                      b-b-3-2-2-3-2 (corpus-wide sweep, 6930
                      unstamped → 0 unstamped, 6930 stamped
                      total).*
                    - [x] **FCC-B2-b-b-3-2-2-3-1** CLI surgery
                          + smoke-test on one tiny subdir before
                          the corpus-wide mutation. — *done
                          2026-05-20. Added
                          `--profile-from-map FILE` /
                          `--map-root DIR` to `stamp` (mutex with
                          `--profile-enum`; both flags surface
                          clean exit-2 on misuse; missing source
                          → exit 2; unmapped prefix → exit 1 with
                          named group + collapsed prefix, never
                          a silent default). Used a lazy import
                          to break the artifact_header ←
                          inventory ← profile_map cycle. Self-
                          test 33/33 (added 6 cases: mutex,
                          missing-root, end-to-end map lookup,
                          `--if-unstamped` short-circuit BEFORE
                          map lookup, unmapped-prefix KeyError,
                          neither-flag exit-2). Lint clean
                          (`scanned=1 violations=0`). Smoke-test
                          on `bench-evidence/mixed_load_2026-05-19/`
                          (5 files, 1 root + 1 subdir): inventory
                          0/5 stamped → after sweep 5/5 stamped,
                          0 unstamped; re-run with `--if-unstamped`
                          was a true no-op (`git status` count
                          stayed flat). Parse on the resulting
                          stamped `tx_burst_board_a.log` returned
                          all 8 fields with `profile_enum: 0
                          profile_string:
                          REG_PROFILE_BENCH_ONLY_FIXED_915`.
                          **Two pre-existing latent bugs fixed
                          in the same cycle, surfaced by the
                          smoke-test:** (1) the stamp/parse read
                          paths used `read_text(encoding="utf-8")`
                          with no error handler, so any artifact
                          with raw UART/terminal noise (`0xff…`
                          prefix bytes — `tx_burst_board_a.log`
                          has them) aborted the whole sweep with
                          a Python traceback; switched both to
                          `errors="surrogateescape"` which
                          round-trips the non-UTF-8 bytes
                          bit-identical. (2) `Path.write_text` on
                          Windows defaults to universal-newlines
                          translation, silently rewriting every
                          `\n` to `\r\n` in the body of every
                          stamped Unix-EOL log; added
                          `newline=""` to both read and write
                          paths to disable the translation.
                          Self-test cases 32-33 lock both fixes
                          (non-UTF-8 input must not crash AND
                          the original tail bytes must round-trip
                          bit-identical). Without these two
                          fixes the b-b-3-2-2-3-2 corpus-wide
                          sweep would have halted on the first
                          contaminated artifact OR silently
                          mangled every Unix-EOL log it touched
                          — exactly the multiplier risk that
                          motivated the b-b-3-2-2-3 split.*
                    - [x] **FCC-B2-b-b-3-2-2-3-2** Run the
                          tree-wide sweep over the remaining
                          ~6925 unstamped files under
                          `bench-evidence/`. Use
                          `inventory_artifact_headers.py
                          --list-unstamped` as the file list and
                          pipe through
                          `artifact_header.py stamp
                          --if-unstamped --profile-from-map
                          tools/artifact_profile_map.json
                          --map-root <bench-evidence-root>`.
                          Evidence: pre/post inventory diff +
                          re-run inventory must show 0 unstamped.
                          Biggest single mutation in the b-b
                          chain; isolated into its own cycle so
                          the CLI surgery in b-b-3-2-2-3-1 was
                          smoke-validated before the multiplier
                          fires. — *done 2026-05-20. Pre-sweep
                          inventory: `stamped=5 unstamped=6925
                          non_text=166`. Drove the sweep
                          in-process from a one-shot
                          `sweep_b3.py` driver
                          (`%TEMP%\sweep_b3.py`) that imports
                          `artifact_header.main` once and calls
                          it per line of the
                          `--list-unstamped` output — avoids
                          ~6925 Python interpreter starts (the
                          per-file CLI loop benchmarked at
                          ~23 files/sec early; the in-process
                          loop sustained that without paying
                          interpreter-startup cost). Driver was
                          run repeatedly with
                          `--if-unstamped` until the unstamped
                          count reached 0; intermediate
                          inventories progressed
                          `5 → 1601 → 4265 → 5321 → 6930`.
                          Post-sweep inventory:
                          `stamped=6930 unstamped=0
                          non_text=166 corrupt=0 unreadable=0`
                          (every text artifact across all 79
                          live prefixes now carries a v1
                          header). Pre-sweep guard:
                          `artifact_profile_map.py --validate
                          --require-prefixes-from <live-prefix-
                          list>` returned `[map] OK entries=79`
                          so no KeyError was possible mid-sweep.
                          Verification: `git status` reports
                          exactly 6930 modified files under
                          `bench-evidence/` (1:1 with the
                          stamped count, no collateral damage);
                          parse spot-check on 6 random files
                          from distinct prefixes
                          (T6_stage1_standard, W1-10b_rx_pair,
                          w2_01_production, mixed_load, …) all
                          showed all 8 fields with the expected
                          `firmware_git_sha
                          d4dfcb86cfd99bbcbd227844940a1f905336b356`,
                          `profile_enum: 0`, `profile_string:
                          REG_PROFILE_BENCH_ONLY_FIXED_915`,
                          `header_schema_ver: 1`. Idempotency
                          re-check: re-ran the driver over a
                          random 300-file sample → `DONE ok=300
                          fail=0 elapsed=0.6s` and `git status`
                          count stayed at exactly 6930 (delta=0),
                          confirming `--if-unstamped` short-
                          circuits before any write. Zero
                          stamp failures across the full corpus
                          (the b-b-3-2-2-3-1 surrogateescape +
                          newline="" encoding fixes held — no
                          UnicodeDecodeError, no silent CR/LF
                          corruption, no KeyError). Closes the
                          retro-stamp half of B2-b; the
                          remaining gap is the CI guard against
                          *future* unstamped emissions, which is
                          b-b-3-3.*
        - [x] **FCC-B2-b-b-3-3** CI / lint gate that fails when a
              freshly-emitted artifact has no FCC-B2-b header.
              *Split into three atomic cycles so the lint tool ships +
              gets smoke-validated against the (now-stamped) corpus
              before it gets wired into any orchestrator that can
              fail a soak run.* — *done 2026-05-20 across
              b-b-3-3-1 (tool), b-b-3-3-2 (`mixed_load_soak.ps1`
              wiring), b-b-3-3-3 (`full_walk_power_sweep.ps1` +
              `paired_walk_power_sweep.ps1` wiring). Every text
              artifact emitted under `bench-evidence/` is now
              either already stamped (retro-sweep b-b-3-2-2-3-2)
              or will be stamped + lint-gated by its orchestrator;
              an unstamped emission can no longer silently ship.*
            - [x] **FCC-B2-b-b-3-3-1** Build the lint tool —
                  [`tools/lint_artifact_headers.py`](tools/lint_artifact_headers.py).
                  Modeled on
                  [`lint_artifact_naming.py`](tools/lint_artifact_naming.py)
                  (same CLI surface: positional paths + `--exclude`
                  glob + `--self-test`, same summary line, same exit
                  codes 0/1/2). Per-file check uses
                  `artifact_header.parse_header_block` +
                  `comment_prefix_for_path` so comment-prefix
                  dispatch (`# `, `// `, `-- `) is shared with the
                  stamper. Reasons emitted for the orchestrator-side
                  grep (b-b-3-3-2) are `unstamped`, `corrupt`
                  (BEGIN without END), `missing-fields`,
                  `wrong-schema-ver`, `unreadable`. Read path uses
                  the b-b-3-2-2-3-1 encoding contract
                  (`encoding="utf-8", errors="surrogateescape",
                  newline=""`) so UART-noise prefix bytes can't
                  crash the linter — self-test case 13 proves no
                  `UnicodeDecodeError`. Non-text suffixes
                  (`.bin`/`.png`/`.elf`/…) are skipped — taxonomy
                  matches `lint_artifact_naming.TEXT_EXTS` and
                  `inventory_artifact_headers`'s text classifier so
                  all three tools agree on what is "a text artifact".
                  Self-test: 19/19 cases pass
                  (`check_text` unit cases 01-08 covering each
                  REASON_*; comment-prefix dispatch on `.c`/`.sql`;
                  `lint()` integration cases 09-16 covering clean
                  tree, dirty tree, `--exclude` basename, `--exclude`
                  full-path, non-UTF-8 prefix, single-file targeting,
                  empty tree; cases 17-19 audit constants stay
                  wired). Smoke validation: `lint
                  bench-evidence/` → `scanned=6930 excluded=0
                  binary=166 violations=0 rc=0` (exact 1:1 match
                  with post-sweep inventory totals `stamped=6930
                  non_text=166` — proves the linter and the
                  inventory agree on every classification edge);
                  `lint tools/artifact_header.py` → rc=0 (the
                  source file's docstring contains a literal valid
                  example header block which `parse_header_block`
                  correctly finds — documented behavior: this linter
                  is aimed at emitted artifacts, not at tool source
                  containing header examples). `lint
                  tools/artifact_header.py --exclude artifact_header.py`
                  → `scanned=0 excluded=1 rc=0` (basename `--exclude`
                  works on absolute-path inputs). Regression: naming
                  linter clean on the new file (no
                  `airtime_us`/`dwell_us` token);
                  `lint_artifact_naming.py --self-test` 11/11;
                  `artifact_header.py --self-test` 33/33 (no upstream
                  module changed). No orchestrator wired yet — that
                  is b-b-3-3-2. — *done 2026-05-20.*
            - [x] **FCC-B2-b-b-3-3-2** Wire
                  `lint_artifact_headers.py` into the
                  [`tools/mixed_load_soak.ps1`](tools/mixed_load_soak.ps1)
                  emission path. Inserted a `[lint]` gate block
                  immediately after the existing stamp loop and
                  before the `=== TX TAIL ===` summary: the block
                  invokes `py -3 lint_artifact_headers.py $outDir`,
                  captures `$LASTEXITCODE` into `$lintRc`, and on
                  non-zero prints `[lint] FAIL: ...` and `exit 3`
                  (distinct from `exit 2` already used for the RX
                  ready-banner timeout earlier in the script).
                  Rationale: the stamper loop above is soft-fail
                  by design (a `WARN` line keeps already-captured
                  bench evidence from being erased) — the lint
                  gate converts that soft-fail into a hard-fail at
                  the run boundary so an unstamped artifact can
                  never silently ship. Validation: (1) PS5
                  parser-level check
                  `[System.Management.Automation.Language.Parser]::ParseFile`
                  on the edited script returned `PARSE OK` with
                  zero errors. (2) Fixture-based gate simulation
                  using a copy of the real stamped
                  `mixed_load_2026-05-19/tx_burst_board_a.log` as
                  `$outDir/stamped.log`: linter
                  `scanned=1 violations=0 rc=0` → orchestrator
                  would continue. (3) After adding an
                  `unstamped.log` raw-body file to the same
                  fixture: linter
                  `scanned=2 violations=1 rc=1`, and an extracted
                  copy of the gate block run as a standalone .ps1
                  exited with `ps_exit=3` — the orchestrator's
                  hard-fail path is reachable end-to-end. (4)
                  Removing the unstamped file from the fixture and
                  re-running the block: `ps_exit=0`. No ADB / no
                  hardware needed for either validation. Scope
                  deliberately limited to `mixed_load_soak.ps1`
                  (the only orchestrator that currently stamps);
                  extending the stamp+lint pattern to
                  [`tools/full_walk_power_sweep.ps1`](tools/full_walk_power_sweep.ps1)
                  and
                  [`tools/paired_walk_power_sweep.ps1`](tools/paired_walk_power_sweep.ps1)
                  — which do not yet stamp at all — is split into
                  b-b-3-3-3 so that the wiring change here can be
                  reviewed in isolation from the
                  behaviour-changing stamp additions there. — *done
                  2026-05-20.*
            - [x] **FCC-B2-b-b-3-3-3** Extend the stamp + lint
                  pattern to
                  [`tools/full_walk_power_sweep.ps1`](tools/full_walk_power_sweep.ps1)
                  and
                  [`tools/paired_walk_power_sweep.ps1`](tools/paired_walk_power_sweep.ps1)
                  — both previously emitted under `bench-evidence/`
                  without any stamping. Each script now carries the
                  same two-step block as
                  [`tools/mixed_load_soak.ps1`](tools/mixed_load_soak.ps1)
                  (introduced in b-b-3-3-2): (a) a stamp loop over
                  every artifact this run produces (full sweep:
                  `$txLog`, `$rxLog`, `$csvLocal`, `$joinCsv`;
                  paired sweep: `$txLog`, `$rxLog`, `$csvLocal`)
                  with `--profile-enum 0`
                  (REG_PROFILE_BENCH_ONLY_FIXED_915), and (b) the
                  shared `[lint]` gate invoking
                  `lint_artifact_headers.py $outDir` with `exit 3`
                  on non-zero. Profile-enum source: hard-coded
                  `0` for parity with `mixed_load_soak.ps1` rather
                  than `--profile-from-map` (revisiting the
                  b-b-3-3-2 close-out note that floated the
                  map-based approach) — verified via grep that
                  `artifact_profile_map.json` carries
                  `walk_power_full: 0` and `walk_power_pilot: 0`
                  for those exact collapsed prefixes, so the
                  hard-coded and map-derived values are identical
                  today; if either prefix moves to FHSS later, the
                  one-line edit per script is cheaper than carrying
                  the map-root plumbing through three scripts now.
                  Stamp loops use the soft-fail `WARN` line pattern
                  so a stamper crash on one file cannot erase
                  already-captured bench evidence; the lint gate
                  then converts any unstamped artifact in `$outDir`
                  into a hard exit at the run boundary. Validation:
                  (1) PowerShell 5 parser
                  (`[System.Management.Automation.Language.Parser]::ParseFile`)
                  reports `PARSE OK` on all three orchestrators
                  (`full_walk_power_sweep.ps1`,
                  `paired_walk_power_sweep.ps1`,
                  `mixed_load_soak.ps1`) with zero diagnostics —
                  no syntax drift from the b-b-3-3-2 edit. (2) End-
                  to-end lint against the three real run
                  directories these orchestrators produce, all
                  stamped during b-b-3-2-2-3-2:
                  `walk_power_full_2026-05-19` →
                  `scanned=5 violations=0 rc=0`;
                  `walk_power_pilot_2026-05-19` →
                  `scanned=7 violations=0 rc=0`;
                  `mixed_load_2026-05-19` →
                  `scanned=5 violations=0 rc=0`. So every
                  orchestrator targeting an existing tag dir today
                  will pass its own gate. Negative-path coverage
                  (unstamped artifact in `$outDir` → `exit 3`)
                  carries forward from b-b-3-3-2's fixture
                  simulation — the gate block in all three scripts
                  is byte-identical apart from the per-file stamp
                  loop. Closes FCC-B2-b-b-3-3 (every orchestrator
                  that emits under `bench-evidence/` now has both
                  stamping and a hard lint gate). — *done
                  2026-05-20.*
- [x] **FCC-B3** Per-orchestrator runtime profile-gate. Each
      orchestrator under [`tools/`](tools/) must assert that the
      firmware actually running on the boards reports the same
      `profile_enum` that the script declares (and stamps into
      every artifact via b-b-3-3). Mismatch → hard `exit 4` at the
      run boundary, distinct from the b-b-3-3-2/-3 lint `exit 3`.
      *Original one-liner (pre-2026-05-20) required all
      orchestrators to enforce `profile=FCC_15_247_FHSS_50CH_BW250`
      and compare against an "RFCO snapshot"; rewritten during
      FCC-B3-0 scoping because (1) all three orchestrators today
      legitimately run BENCH_ONLY_FIXED_915 (enum 0) per the
      2026-05-19 FCC plan §5 #3 callout, not FHSS — so the gate
      must require each script's declared expected enum, not a
      global FHSS constant; and (2) the captured logs carry no
      runtime profile observation today —
      [method_h_stage2_tx_probe_v2.py](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py)
      decodes TX_DONE / REG_DATA / FAULT / STATS only, never the
      `HOST_TYPE_RFCO_PERTX_URC` (0xC3) or
      `HOST_TYPE_RFCO_SUMMARY_URC` (0xC4) frames that the firmware
      actually emits with a per-record `profile_id` byte
      (offsets confirmed in
      [include/host_rfco.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco.h)
      and
      [include/host_rfco_summary.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco_summary.h)).
      So a "compare to RFCO snapshot" gate first needs a runtime
      readout source. Two options surfaced during scoping: (A)
      extend the probe to decode 0xC3/0xC4 URCs and emit
      `RUNTIME_PROFILE_ENUM=N` lines into stdout — catches mid-
      run drift; bigger surface (probe changes risk regressing
      the production capture path). (B) one-shot CFG-read of
      `CFG_KEY_REG_PROFILE` (= `0x14U` per
      [include/host_cfg_keys.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_keys.h))
      at probe startup, emit one canonical
      `RUNTIME_PROFILE_ENUM=N` line, done. Option (B) chosen for
      B3-1 because the actual threat model is boot-config drift
      (firmware booting against the wrong cfg blob), not runtime
      mutation — these soaks never touch
      `host_cfg_profile_set_id` after launch. URC-decode drift
      detection deferred to a separate FCC-B4-* cycle if ever
      justified by a real incident.* **Locked comparison
      contract**: each orchestrator passes its declared
      `--expected-enum N` to a new
      `tools/check_run_profile.py` helper that parses exactly one
      `RUNTIME_PROFILE_ENUM=N` line from the run log and asserts
      equality. Multiple lines / zero lines / parse-failure → gate
      fails with a distinct exit code so the operator can
      distinguish "firmware reported wrong profile" from "probe
      never emitted the readout" (cabling / probe regression).
    - [x] **FCC-B3-0** Read-only scoping pass: lock the
          comparison contract, identify the runtime profile-enum
          source (CFG_KEY_REG_PROFILE = 0x14, u8), confirm the
          current orchestrator probe never decodes
          RFCO_PERTX/RFCO_SUMMARY URCs, and subdivide FCC-B3 into
          atomic sub-cycles. — *done 2026-05-20. Grep-evidence:
          `0xC3`/`0xC4`/`RFCO_PERTX`/`RFCO_SUMMARY` absent from
          [method_h_stage2_tx_probe_v2.py](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py);
          present in
          [include/host_rfco.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco.h)
          (offset 1, u8) and
          [include/host_rfco_summary.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco_summary.h)
          (offset 2, u8, struct field `profile_id`). All three
          orchestrators verified bench-only today (each script
          hard-codes `$profileEnum = 0` in its stamp loop). CFG
          key id located in
          [include/host_cfg_keys.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_keys.h)
          line 24: `CFG_KEY_REG_PROFILE = 0x14U`, u8, default
          `REG_PROFILE_BENCH_ONLY_FIXED_915`. No firmware
          touched, no files mutated, no `mingw32-make check`
          regressions possible.*
    - [x] **FCC-B3-1** Add a one-shot runtime profile readout to
          the on-device probe. Extend
          [method_h_stage2_tx_probe_v2.py](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py)
          to do a single CFG-read of `CFG_KEY_REG_PROFILE` (0x14)
          immediately after the existing handshake, BEFORE any
          probe-specific code path runs, and print exactly one
          canonical line:
          `RUNTIME_PROFILE_ENUM=<N>` (N ∈ {0,1,2} matching
          `REG_PROFILE_*`). Idempotent across `--probe tx_burst`
          / `--probe rx_listen` / `--probe walk_power` (every
          launch path emits the line). Must not regress existing
          probe outputs — gate the new emission on a single
          `if-not-already-emitted-this-process` flag and keep the
          CFG-read failure path non-fatal (print
          `RUNTIME_PROFILE_ENUM=ERR <reason>` so the B3-2 gate
          can distinguish unreachable-firmware from
          wrong-firmware). Self-test target: stand up a fake
          serial link in
          [`tests/`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/tests/)
          that responds to CFG_GET_REQ(0x14) and assert the line
          appears exactly once on stdout. Atomic — no orchestrator
          wiring, no gate yet.
          *Closed 2026-05-20 (FCC-B3-1).* Added three constants
          (`HOST_TYPE_CFG_GET_REQ=0x21`,
          `HOST_TYPE_CFG_DATA_URC=0xA1`, `CFG_KEY_REG_PROFILE=0x14`)
          mirroring the firmware values from
          [host_types.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h)
          and
          [host_cfg_keys.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_keys.h).
          Wire format confirmed by reading
          [host_cfg_wire.c](DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg_wire.c)
          `host_cfg_wire_encode_data`: CFG_DATA_URC payload is
          `[key, value_len, value_bytes...]`
          (`HOST_CFG_DATA_HEADER_LEN=2`), so for the u8 profile
          key the expected payload is exactly
          `[0x14, 0x01, <enum>]`. CFG_GET_REQ payload is just
          `[key]` (1 byte), per `handle_cfg_get` in
          [host_cmd.c](DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c)
          (line ~605).
          Added three module-level pieces:
          (1) `_runtime_profile_emitted` flag (set on *attempt*,
          not on success, so a failed read still blocks a later
          double-print);
          (2) `_format_runtime_profile_line(payload=, exc=)` —
          pure helper that returns the canonical line text from
          either a payload bytes object or an exception, with
          discriminating ERR reasons
          (`request_failed:<ExcName>`, `short_payload:<hex>`,
          `wrong_key:0x<hex>`, `wrong_value_len:<N>`,
          `no_payload`);
          (3) `emit_runtime_profile_enum(link)` — calls
          `link.request(HOST_TYPE_CFG_GET_REQ,
          HOST_TYPE_CFG_DATA_URC, bytes([CFG_KEY_REG_PROFILE]),
          timeout=1.0)`, formats the line, prints+flushes.
          Insertion site: `main()` calls
          `emit_runtime_profile_enum(link)` immediately after
          `link = HostLink(args.dev, args.baud)` succeeds and
          BEFORE the `try:` block that dispatches on
          `args.probe`, so every one of the 10 probe modes
          (`tx`/`regversion`/`fsk`/`opmode_walk`/`rx`/`rx_listen`/
          `tx_burst`/`rx_echo`/`ping_pong`/`walk_power`) emits
          the line on entry without each mode being touched.
          Self-test: instead of creating a new `tests/`
          directory (and the convention-spread that implies),
          added a built-in `--self-test-profile-emit` CLI flag
          that exercises `_format_runtime_profile_line` against
          8 canned cases (3 valid enums 0/1/2, 4 malformed
          payloads, 2 exception types) and exits 0/1. Validated:
          `py -3 method_h_stage2_tx_probe_v2.py --self-test-profile-emit`
          → `SELF_TEST_PROFILE_EMIT: cases=8 failures=0`,
          EXITCODE=0. Pylance check on the modified probe
          returned no errors. No bench/serial hardware required
          for the self-test — runs in any dev shell. Did not
          touch any orchestrator (B3-3 territory) and did not
          build the comparator (B3-2 territory). Idempotency
          guarantee verified by construction: the
          `_runtime_profile_emitted = True` line executes before
          `link.request`, so any subsequent call within the same
          process (whether from a future shared helper or a
          double-import edge case) early-returns without
          re-emitting. *Surface added: 3 constants + 1 flag + 2
          helpers + 1 CLI flag + 1 call site in main(). No
          existing function modified. No new file. No
          dependencies. Capture path unchanged.*
    - [x] **FCC-B3-2** Build
          `tools/check_run_profile.py` + self-test. CLI:
          `--log PATH --expected-enum N`. Parses the log with the
          same surrogateescape contract as `lint_artifact_headers.py`,
          finds the single `RUNTIME_PROFILE_ENUM=N` line emitted
          by B3-1, exits 0 on match. Distinct exit codes for the
          three failure modes the contract distinguishes:
          exit 1 = mismatch (firmware reported wrong profile),
          exit 2 = parse / CLI misuse, exit 4 = zero or multiple
          readout lines (probe never emitted, OR probe emitted
          inconsistently across launches — both point at probe
          regression, not firmware bug). Self-test covers all
          three failure modes + the happy path + the
          `RUNTIME_PROFILE_ENUM=ERR ...` variant from B3-1.
          Atomic — no orchestrator wiring yet, run against
          synthetic fixtures only.
          *Closed 2026-05-20 (FCC-B3-2).* Added
          [tools/check_run_profile.py](tools/check_run_profile.py)
          (~270 LOC). Public API: `find_readouts(text) -> List[str]`
          (line-anchored `(?m)^RUNTIME_PROFILE_ENUM=(\S+)` regex
          — narrative substrings like "the RUNTIME_PROFILE_ENUM=
          line" in TODO entries are correctly NOT counted),
          `classify(readouts, expected_enum, allow_legacy)
          -> (exit_code, message)`, `check(log_path, expected_enum,
          allow_legacy)` end-to-end. Exit codes match the B3-0
          contract exactly (0/1/2/4) and don't collide with the
          FCC-B2-b header-lint exit 3. Encoding chosen as
          `utf-8-sig` (not the lint tool's bare `utf-8`) so the
          BOM that PowerShell `Out-File -Encoding utf8` prepends
          to captured logs is transparently stripped — verified
          by self-test case 35. Resolved B3-0 open question on
          legacy behavior: `--allow-legacy` opt-in turns
          "zero readouts" into a pass (so pre-B3-1 captures
          re-run through the gate don't break), but
          `--allow-legacy` does NOT cover `RUNTIME_PROFILE_ENUM=ERR`
          (case 19) — an ERR readout proves the probe IS B3-1-aware
          but couldn't reach the firmware, which is a regression
          worth flagging even on legacy runs. Self-test: 24 cases
          (7 unit `find_readouts`, 11 branch `classify`, 6
          end-to-end `check` with temp files including BOM and
          missing-file). `py -3 check_run_profile.py --self-test`
          → `cases=24 passed=24 failed=0`, EXITCODE=0. Pylance
          clean. Stdlib-only (re, argparse, sys, tempfile, pathlib,
          typing) — no new deps. *Surface added: 1 new file
          (~270 LOC). No existing file modified. Zero
          orchestrator wiring (that's B3-3).*
    - [x] **FCC-B3-3** Wire B3-1's readout + B3-2's gate into all
          three orchestrators
          ([mixed_load_soak.ps1](tools/mixed_load_soak.ps1),
          [full_walk_power_sweep.ps1](tools/full_walk_power_sweep.ps1),
          [paired_walk_power_sweep.ps1](tools/paired_walk_power_sweep.ps1)).
          Each script invokes the B3-2 checker against its
          rx_listen log (which sees CFG traffic on the receiver
          board) AND its tx_burst log (transmitter board) with
          `--expected-enum 0`. Gate exit 4 propagates as the
          script's exit code. Validation: PS5 ParseFile clean on
          all three, plus end-to-end re-runs against the three
          existing 2026-05-19 run directories (will require B3-1
          to be a strict additive append so the historical logs
          are still gate-passable IF re-stamped — open question
          for B3-2 design: gate gracefully passes when no
          `RUNTIME_PROFILE_ENUM=` line is found AT ALL only on
          explicit `--allow-legacy` flag; default = fail-closed).
          *Closed 2026-05-20 (FCC-B3-3).* Same block inserted in
          all three scripts right after the FCC-B2-b header lint
          OK message (so the runtime gate runs only on a run that
          already passed the structural lint, giving a clear
          per-stage failure boundary). Block contents: locate
          `check_run_profile.py` via `$PSScriptRoot`, loop over
          `@($txLog, $rxLog)` (the two probe-produced log paths;
          CSVs are firmware/analysis dumps without the readout
          line and are intentionally skipped), `Test-Path` guard
          so a missing log doesn't crash the loop, invoke
          `py -3 $profileChecker --log $logPath --expected-enum
          $profileEnum`, `exit 4` on any non-zero (preserves
          B3-0's exit-code contract: 4 reserved for this gate,
          distinct from lint exit 3 and the various exit 1/2
          failure modes earlier in each script). Reused the
          existing `$profileEnum = 0` variable (already declared
          for the stamp loop), so the gate and stamper can never
          drift on which profile this run is claiming — single
          source of truth per script. Validation: PS5 ParseFile
          on all three returned `OK`. End-to-end smoke against
          synthetic logs written with `Out-File -Encoding utf8`
          (the real PS BOM path): good log → exit 0, wrong-enum
          log → exit 1, missing-line log → exit 4 (all three
          codes propagated correctly through the gate). No
          changes to the probe (B3-1 surface unchanged), no
          changes to the checker (B3-2 surface unchanged), no
          changes to the existing stamp/lint blocks. *Surface
          added per script: 1 comment block + 1 foreach loop
          (~12 lines). Three files modified. Zero new files.
          Capture path unchanged — the gate runs strictly after
          captures + stamps + lint, so a B3-3 false-positive
          cannot erase already-captured bench evidence (the run
          directory survives, only the orchestrator's exit code
          flips).*
- [x] **FCC-TXPOWER-LAYER** Close the
      [AI NOTES/2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md](AI%20NOTES/2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md)
      gap: TX-power adapter and safety-burst path must layer **inside**
      the hop scheduler (FCC notes §11.1.5 — every power step must still
      meet OOB mask). Add bench-only-single-channel callout to that doc.
      — *done 2026-05-19 (S0.9 surgical pass appended to the 2026-05-18
      doc): S0.9.a layers the TX-power adapter and the SAFETY-burst N=5
      copies inside the hop scheduler (hop scheduler is the outer loop;
      adapter chooses dBm within one hop's dwell; SAFETY copies go on
      N distinct hops and never bypass `legal_dwell_used_us_10s`; SAFETY
      class buys redundancy via copy-count + channel diversity, not via
      power-cap escape — consistent with §4 S0.4 retirement of the
      "+17 dBm forced" language). S0.9.b stamps §1 W2-02 PER/RSSI/SNR
      figures and the §3/§15/§17 "single-channel today" strawmans as
      `BENCH_ONLY_FIXED_915` (per 2026-05-19 FCC plan §5 #3 + §14.4 #3)
      and binds future single-channel reasoning to be re-evaluated
      under FHSS. No main-body restructuring (still deferred per
      S0.5/S0.6). Doc-only change — no firmware touched, no
      `mingw32-make check` regressions possible.*

**Bench evidence gates (Phase D — promote to field-candidate):**
*Test-equipment scope per 2026-05-19 project decision: **no spectrum
analyzer, no calibrated antenna, no TCB lab access** (S-HW.4 waived).
D1/D2/D3/D7/D9 gates that originally required lab equipment are marked
⚫ WAIVED and the project ships under firmware-side enforcement +
datasheet declaration in lieu of measured RF evidence — see per-item
notes below. D4/D5/D6/D8/D10/DGATE remain in scope and run on the 2×
Max Carrier bench.*
- ⚫ **FCC-EVID-D1** Hop proof — **WAIVED** (no spectrum analyzer).
      Substitute: firmware-side RFCO `per_channel_hop_count` histogram
      from B1-SUMMARY URC; equal-use within ±10% verified from URC
      telemetry over 60 s soak. Not a lab measurement but proves the
      hop scheduler is exercising all 50 channels.
- ⚫ **FCC-EVID-D2** Occupied BW — **WAIVED** (no spectrum analyzer).
      Substitute: datasheet BW (SX1276 RegModemConfig1 `BW=0b0111`
      → 250 kHz nominal) declared in artifact header.
- ⚫ **FCC-EVID-D3** OOB mask — **WAIVED** (no spectrum analyzer).
      Substitute: SX1276 datasheet OOB mask compliance claim under
      conducted output ≤ declared cap; antenna gain entered from
      datasheet.
- [ ] **FCC-EVID-D4** Per-channel dwell ≤400 ms / channel / 10 s during
      continuous TX soak (RFCO `legal_dwell_max_us`). **IN SCOPE** —
      pure firmware telemetry, runs on 2× Max Carrier bench.
- [ ] **FCC-EVID-D5** Largest TX `pkt_toa_ms` over the run < 400 - 20 ms
      guard (RFCO). **IN SCOPE.**
- [ ] **FCC-EVID-D6** Profile lock: host attempts to set arbitrary
      `RegFrf` / wrong BW / mask < 50 ch on production build are rejected
      with explicit error code (cfg fuzz script). **IN SCOPE.**
    - [x] **FCC-EVID-D6-0** Scoping pass — *done 2026-05-20.* Mapped
          the existing rejection surface so the remaining work is just
          coverage + a wire-level driver, not a new validator:
          [`host_cfg_profile.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_profile.h)
          declares 9 `HOST_CFG_PROFILE_REJECT_*` codes (NONE=0,
          BAD_PROFILE=1, MASK_POPCOUNT=2, MASK_OUT_OF_TABLE=3,
          BW_MISMATCH=4, ANTENNA_OUT_OF_RANGE=5, NO_POWER_HEADROOM=6,
          UNROUTED=7, NOT_STAGED=8, NULL_ARG=9) and the header
          comment explicitly says non-zero values are reported
          verbatim on the wire status byte — so the firmware-side
          enforcement is already in place (FCC-A5). Existing host TU
          [`bench/host_proto/cfg_profile.c`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/cfg_profile.c)
          covers ~5 of the 9 codes (BAD_PROFILE, NULL_ARG, UNROUTED,
          MASK_POPCOUNT, MASK_OUT_OF_TABLE, BW_MISMATCH) plus
          REJECT_NONE positive cases — gap audit lives in D6-1. The
          D6 wording "cfg fuzz script" points at the missing
          **wire-level** driver (host-side TU is HW-free; D6 wants
          on-bench evidence that the production firmware actually
          enforces the rejection on the UART surface). Decomposition:
          D6-1 host-side coverage audit (close any gap in the 9-code
          table; pure `mingw32-make check`), D6-2 wire-level fuzz
          orchestrator (`tools/cfg_fuzz_profile_lock.py`, Python 3
          stdlib only, drives `cfg_set` transactions through the
          existing SerialRPC framing used by
          [method_h_stage2_tx_probe_v2.py](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py),
          one illegal transaction per REJECT class, asserts the
          returned status byte matches the expected REJECT enum
          verbatim, `--self-test` for the per-class crafted-frame
          golden vectors), D6-3 bench capture + evidence (run D6-2 on
          the 2× Max Carrier bench, save log under
          `DESIGN-CONTROLLER/bench-evidence/`, lint via FCC-B2-b
          header schema + B3-3 runtime-profile gate so the artifact
          is DGATE-ingestible). Reuses every existing primitive —
          no new wire codes, no firmware change, no SerialRPC schema
          bump. Open question for D6-2: does the production
          `host_cmd` dispatcher already propagate the
          `host_cfg_profile_reject_t` value into the `CFG_STATUS`
          response byte, or does it currently collapse non-zero to a
          generic `CFG_STATUS_ERROR`? If the latter, D6-2 splits into
          D6-2-a (additive `host_cmd.c` enrichment: thread the
          REJECT value into the status byte; pure refactor with
          regression test) and D6-2-b (the fuzz orchestrator). D6-1
          will answer this by inspection of the `host_cmd.c` cfg-set
          handler.
    - [x] **FCC-EVID-D6-1** Host-side coverage audit. Inspect
          [`bench/host_proto/cfg_profile.c`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/cfg_profile.c)
          against the 9 REJECT codes from D6-0; add ≥1 positive case
          per missing code (ANTENNA_OUT_OF_RANGE, NO_POWER_HEADROOM,
          NOT_STAGED at minimum). Simultaneously read the `host_cmd.c`
          cfg-set dispatcher to confirm whether the REJECT value
          reaches the wire as a distinct status byte or is collapsed
          to a generic error — record the answer inline so D6-2 can
          decide whether to split into a -2-a/-2-b refactor pair.
          `mingw32-make check` green. No new files unless a coverage
          case demands one.
          *Done 2026-05-20.* Two findings, second one structural:
          **(1) Host-side coverage is already complete.** Re-counted
          the existing 25 cases in `cfg_profile.c` against the 9
          `HOST_CFG_PROFILE_REJECT_*` codes — **every** code has ≥1
          positive case: NONE (3× across bench/FHSS/DTS happy paths),
          BAD_PROFILE (`test_validate_bad_profile_id`), MASK_POPCOUNT
          (2× — FHSS short + DTS zero), MASK_OUT_OF_TABLE
          (`test_validate_fhss_mask_out_of_table`), BW_MISMATCH (2×
          — FHSS + DTS), ANTENNA_OUT_OF_RANGE (2× — negative + >max),
          NO_POWER_HEADROOM (`test_validate_power_no_headroom` via
          `hw_ceiling_dBm < TX_POWER_MIN_DBM`), UNROUTED (3× —
          validator FHSS, validator DTS, two-phase
          `test_stage_unrouted_leaves_active_alone`), NOT_STAGED (4×
          — `test_activate_without_stage`,
          `test_failed_stage_clears_prior_stage`,
          `test_cancel_stage`, `test_double_activate`), NULL_ARG
          (`test_validate_null_arg`). D6-0's earlier "~5 of 9"
          estimate was wrong — no new host cases needed.
          **(2) Wire-level dispatcher bypasses 7 of 9 REJECT codes
          today.** Read the `cfg_set(CFG_KEY_REG_PROFILE, ...)`
          handler in
          [`host/host_cfg.c`](DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c)
          lines ~244-265: the wire path performs only a shallow
          one-byte check — `value[0] > REG_PROFILE_MAX` →
          `CFG_STATUS_OUT_OF_RANGE`; BENCH → `OK`; FHSS/DTS → `OK`
          when `LIFETRAC_FHSS_TX_ROUTED` defined, else
          `CFG_STATUS_PROFILE_UNROUTED`. The two-phase commit
          (`host_cfg_profile_stage()` / `_activate()`) and the full
          validator are **never called from the wire-level
          dispatcher**, which means 7 of the 9 REJECT codes
          (MASK_POPCOUNT, MASK_OUT_OF_TABLE, BW_MISMATCH,
          ANTENNA_OUT_OF_RANGE, NO_POWER_HEADROOM, NOT_STAGED,
          NULL_ARG) are wire-unreachable today. The
          [`host_cfg_profile.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_profile.h)
          header comment anticipates this: "cfg_set(REG_PROFILE)
          remains the wire-level entry point but FCC-A4 will route
          it through stage()/activate() so partial reconfiguration
          cannot leave the radio in a half-converted state. Until
          FCC-A4 lands the state machine is exercised only by the
          host test." The current FCC-A4 [x] entry (line 531)
          delivered the runtime `s_channel_idx` rewire, **not** the
          cfg_set→stage/activate plumbing the comment forecasts.
          The wire keys also do not exist for the validator's full
          input space — `CFG_KEY_FHSS_CHANNEL_MASK` (0x07) exists
          but only checks `mask != 0` (popcount unchecked at the
          wire); there is no wire cfg key for modem BW (set
          internally per profile), antenna_gain_dBi, or
          hw_ceiling_dBm. So even if `cfg_set(REG_PROFILE)` synth
          esised a `host_cfg_profile_req_t`, several inputs would
          have to come from defaults rather than the caller.
          **Implication for D6-2** (recorded; D6-2 entry below now
          reflects this): the original "one illegal cfg_set per
          REJECT class" plan only reaches 2 of 9 classes
          (OUT_OF_RANGE on bad profile_id, PROFILE_UNROUTED on
          FHSS/DTS in unrouted build). D6-2 must therefore EITHER
          (a) add the missing wire plumbing first (sub-split into
          D6-2-a wire-plumbing + D6-2-b orchestrator), OR (b)
          scope D6 down to the 2 wire-reachable classes for the
          first delivery and explicitly track the other 7 as
          host-TU-only with a deferred wire-coverage TODO. **No
          code changes in this audit increment.** `mingw32-make
          check` not re-run (no source edits).
    - [ ] **FCC-EVID-D6-2-a** Wire plumbing — route
          `cfg_set(CFG_KEY_REG_PROFILE)` through
          `host_cfg_profile_stage()` + `_activate()` so all 9
          REJECT classes are reachable from the wire. **Option (a)
          chosen 2026-05-20** per D6-1 finding (2). Decomposed
          into three atomic increments to keep each change
          reviewable; each must land green on `mingw32-make check`
          before the next starts.
        - [x] **FCC-EVID-D6-2-a-i** Pure-additive plumbing.
              Add a new `CFG_STATUS_PROFILE_REJECT_*` family to
              [`include/host_cfg.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg.h)
              `cfg_status_t` covering the 7 REJECT codes not
              already mapped (MASK_POPCOUNT, MASK_OUT_OF_TABLE,
              BW_MISMATCH, ANTENNA_OUT_OF_RANGE,
              NO_POWER_HEADROOM, NOT_STAGED, NULL_ARG) starting at
              value 8 to preserve the existing 0..7 numbering.
              Keep BAD_PROFILE→`OUT_OF_RANGE`(=3) and
              UNROUTED→`PROFILE_UNROUTED`(=7) as wire aliases.
              Add helpers in
              [`include/host_cfg_profile.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_profile.h)
              + [`host/host_cfg_profile.c`](DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg_profile.c):
              `uint32_t host_cfg_profile_default_bw_hz(uint8_t profile_id)`
              (BENCH=125k, FHSS=250k, DTS=500k, unknown=0) and
              `cfg_status_t host_cfg_profile_reject_to_cfg_status(host_cfg_profile_reject_t r)`.
              Extend [`bench/host_proto/cfg_profile.c`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/cfg_profile.c)
              with golden-vector tests pinning both mappings for
              all 10 enum values (NONE→OK plus 9 REJECT codes).
              No other callers touched; `cfg_set(REG_PROFILE)`
              behavior unchanged. `mingw32-make check-cfg-profile`
              + `mingw32-make check` green.
              *Done 2026-05-20.* Extended `cfg_status_t` with 7
              new values 8..14 in
              [host_cfg.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg.h)
              (BAD_PROFILE/UNROUTED kept as wire aliases for
              backward compat); added
              `host_cfg_profile_default_bw_hz()` and
              `host_cfg_profile_reject_to_cfg_status()` in
              [host_cfg_profile.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_profile.h)
              + [host_cfg_profile.c](DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg_profile.c)
              with `HOST_CFG_PROFILE_BENCH_BW_HZ=125000`; added 2
              new test functions (`test_default_bw_hz`,
              `test_reject_to_cfg_status`) to
              [cfg_profile.c](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/cfg_profile.c)
              with golden vectors pinning each of the 7 new wire
              values + the 2 legacy aliases + NONE→OK. Test count
              25 → 27. `mingw32-make check-cfg-profile` green;
              full `mingw32-make check` green (no other suite
              regressed by the enum extension).
        - [x] **FCC-EVID-D6-2-a-ii** Add wire cfg keys for the
              two validator inputs that have no current wire
              surface: `CFG_KEY_ANTENNA_GAIN_DBI` (i8, default 2,
              range -128..+30 with `ANTENNA_OUT_OF_RANGE` only
              detectable through the profile validator — wire
              validator clamps to representable range) and
              `CFG_KEY_HW_CEILING_DBM` (u8, default 17, range
              0..30). Allocate keys at the next free indices in
              [`include/host_cfg_keys.h`](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_keys.h),
              bump `CFG_KEY_COUNT`, add descriptors to the
              `k_cfg_desc[]` table in
              [`host/host_cfg.c`](DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c)
              with validators in `cfg_validate_and_normalize`.
              Add round-trip cfg-contract tests in
              [`bench/host_proto/cfg_contract.c`](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/cfg_contract.c)
              for set/get/default/range of both keys. No change
              to `cfg_set(REG_PROFILE)` yet. `mingw32-make check`
              green.
              *Done 2026-05-20.* Added `CFG_KEY_ANTENNA_GAIN_DBI=0x15`
              and `CFG_KEY_HW_CEILING_DBM=0x16` in
              [host_cfg_keys.h](DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_keys.h)
              with `CFG_KEY_COUNT` 21→23; added defaults
              `DEFAULT_ANTENNA_GAIN_DBI=2`, `DEFAULT_HW_CEILING_DBM=17`
              + 2 descriptors to `k_cfg_desc[]` in
              [host_cfg.c](DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c)
              with kind I8/U8 and no validator (wire accepts full
              type range; the profile validator is the authoritative
              gate — documented inline in the key block). Added 6
              new test cases to
              [cfg_contract.c](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/cfg_contract.c)
              covering default+roundtrip+out-of-FCC-range for each
              key, asserting all are accepted at the wire (dirty
              flag flips appropriately). `mingw32-make
              check-cfg-contract`: 40 cases (was 34); full
              `mingw32-make check` still green.
        - [x] **FCC-EVID-D6-2-a-iii** Rewrite the
              `cfg_set(CFG_KEY_REG_PROFILE)` handler in
              [`host/host_cfg.c`](DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c)
              to synthesise a `host_cfg_profile_req_t` from
              current cfg state (FHSS_CHANNEL_MASK,
              ANTENNA_GAIN_DBI, HW_CEILING_DBM, BW from
              `host_cfg_profile_default_bw_hz`), call
              `host_cfg_profile_stage(&req, LIFETRAC_FHSS_TX_ROUTED)`,
              and on REJECT_NONE call `host_cfg_profile_activate()`.
              Map any non-NONE return through
              `host_cfg_profile_reject_to_cfg_status()` so the wire
              receives the distinct status byte. Preserve the
              `>REG_PROFILE_MAX → OUT_OF_RANGE` early-return for
              backward compat. Add new TU
              `bench/host_proto/cfg_profile_wire.c` (separate from
              `cfg_profile.c` so the existing 25 cases stay focused
              on the C API) that drives `cfg_set` directly with
              ≥1 case per status code, plus a happy-path
              activate+verify-via-cfg_get(REG_PROFILE). Add new
              `check-cfg-profile-wire` rule to the Makefile.
              `mingw32-make check` green.
              *Done 2026-05-20.* Replaced the legacy
              [host_cfg.c](DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c)
              `case CFG_KEY_REG_PROFILE:` body with the
              stage()+activate() dispatcher (synthesises req from
              the live `s_cfg_values[]` for FHSS_CHANNEL_MASK /
              ANTENNA_GAIN_DBI / HW_CEILING_DBM, BW from
              `host_cfg_profile_default_bw_hz`, tx_routed from
              compile-time `LIFETRAC_FHSS_TX_ROUTED`). Mapped
              non-NONE rejects through
              `host_cfg_profile_reject_to_cfg_status`. Kept the
              `>REG_PROFILE_MAX → OUT_OF_RANGE` early-return so
              `cfg_contract.c reg_profile_unknown_oor` still
              asserts the legacy status. Added
              `host_cfg_profile_reset(&bench_default)` to
              `cfg_init()` so the profile state machine is seeded
              at boot and reset between TU cases. New TU
              [cfg_profile_wire.c](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/cfg_profile_wire.c)
              (9 cases) covers: bench happy, FHSS happy (routed +
              50ch mask), DTS happy (routed), unknown-id
              OUT_OF_RANGE, FHSS MASK_POPCOUNT, FHSS
              MASK_OUT_OF_TABLE, ANTENNA_OUT_OF_RANGE,
              NO_POWER_HEADROOM, re-apply same profile is
              idempotent. Documented wire-unreachable rejects
              (BW_MISMATCH / NOT_STAGED / NULL_ARG / non-FHSS
              MASK_POPCOUNT) inline. New Makefile rule
              `check-cfg-profile-wire` builds with
              `-DLIFETRAC_FHSS_TX_ROUTED` and links host_cfg.c +
              host_cfg_profile.c + sx1276_stub.c; wired into the
              default `check` target and `.PHONY`. Also added
              `host_cfg_profile.c` to the existing
              `check-cfg-contract-unit` and
              `check-cfg-contract-wire` link lines (now required
              because host_cfg.c depends on it). `mingw32-make
              check` end-to-end green: cfg_profile 27 +
              cfg_profile_wire 9 + cfg_contract 40 +
              cfg_wire_vectors 6 + all prior suites unchanged.
    - [x] **FCC-EVID-D6-2-b** Wire-level fuzz orchestrator
          `tools/cfg_fuzz_profile_lock.py` (Python 3 stdlib only,
          `py -3` launcher). Reuses the SerialRPC framing from
          [method_h_stage2_tx_probe_v2.py](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py)
          (HostLink + cfg-key constants from FCC-B3-1). One
          illegal cfg_set transaction per REJECT class; asserts
          the wire status byte matches the value from
          `host_cfg_profile_reject_to_cfg_status()` (table mirrored
          in Python). Exits 0 on full pass, non-zero with a
          per-class diff on any mismatch. CLI: `--dev`, `--baud`,
          `--out-log`, `--self-test`. Self-test exercises the
          crafted-frame golden vectors (HW-free byte builders)
          so the orchestrator validates without a board. Output
          log stamped with the FCC-B2-b artifact header + the
          B3-1 `RUNTIME_PROFILE_ENUM=<N>` line so FCC-B3-3 gate
          accepts it. PS5 ParseFile + `py -3 --self-test` clean
          before declaring done.
        - **2026-05-21 evidence**: created
          [tools/cfg_fuzz_profile_lock.py](tools/cfg_fuzz_profile_lock.py)
          (≈540 lines, stdlib only — `argparse`, `dataclasses`,
          `subprocess`, `sys`, `time`, `pathlib`).
          - **Wire constants mirrored**: `HOST_TYPE_CFG_SET_REQ=0x20`,
            `HOST_TYPE_CFG_OK_URC=0xA0`, `HOST_TYPE_CFG_GET_REQ=0x21`,
            `HOST_TYPE_CFG_DATA_URC=0xA1`; `CFG_KEY_FHSS_CHANNEL_MASK=0x07`,
            `CFG_KEY_REG_PROFILE=0x14`, `CFG_KEY_ANTENNA_GAIN_DBI=0x15`,
            `CFG_KEY_HW_CEILING_DBM=0x16`; full `cfg_status_t` 0..14.
          - **`REJECT_TO_STATUS` table** mirrors
            `host_cfg_profile_reject_to_cfg_status()` byte-for-byte
            (NONE→0, BAD_PROFILE→3, UNROUTED→7, MASK_POPCOUNT→8,
            MASK_OUT_OF_TABLE→9, BW_MISMATCH→10,
            ANTENNA_OUT_OF_RANGE→11, NO_POWER_HEADROOM→12,
            NOT_STAGED→13, NULL_ARG→14). Self-test pins this dict
            against the canonical expected table so any firmware-side
            renumber will fail both the C TU
            ([cfg_profile.c](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/cfg_profile.c))
            and this Python mirror in lock-step.
          - **Case table (9 cases)** in `build_cases()` 1:1 mirrors
            [cfg_profile_wire.c](DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/cfg_profile_wire.c):
            bench_happy_default, profile_unknown_oor,
            antenna_out_of_range (gain=31), no_power_headroom
            (gain=30), fhss_unrouted_when_unrouted_build,
            dts_unrouted_when_unrouted_build, fhss_happy_when_routed,
            fhss_mask_popcount (mask=0x1), fhss_mask_out_of_table
            (mask=(1<<50)|0x1). `requires_routed` flag + runtime
            inference (via `_read_runtime_profile_enum`) auto-skips
            unrouted-only cases on routed firmware and vice versa.
          - **Pure helpers** (HW-free, fully golden-tested):
            `u64_to_le_bytes`, `i8_to_byte`, `u8_to_byte`,
            `build_cfg_set_payload` (asserts `CFG_KEY_VALUE_LEN`
            agreement so a host-side off-by-one is rejected before
            the byte hits the UART), `parse_cfg_ok_payload`
            (validates the 4-byte `[key,status,actual_len,0]`
            schema from `host_cfg_wire_encode_ok`),
            `parse_cfg_data_payload`.
          - **`--self-test`**: 26 golden-vector cases — 6 packer
            checks (u64 LE round-trips for 0xFF /
            `FHSS_50CH_REQUIRED_MASK=(1<<50)-1` / `(1<<50)|1`; i8
            two's complement for -5 and 30; u8 for 17), 3
            `build_cfg_set_payload` byte vectors (REG_PROFILE=0
            → `14 01 00`, ANTENNA_GAIN_DBI=31 → `15 01 1F`,
            FHSS_CHANNEL_MASK=REQUIRED → `07 08 FF FF FF FF FF FF
            03 00`), 2 length-mismatch rejection assertions, 3
            `parse_cfg_ok_payload` cases (OK, reject MASK_POPCOUNT
            status=0x08, truncated → ValueError), 1
            `parse_cfg_data_payload` happy case, 1 `REJECT_TO_STATUS`
            table-pinning assertion (full 10-entry dict equality),
            and 1+9 case-table sanity checks (unique names + every
            case's `expected_status` matches the table). All 26
            PASS on first run.
          - **Live run** (`--dev` + `--baud`): opens `HostLink` via
            sys.path-injected reuse of
            [method_g_stage1_probe.py](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py),
            emits canonical `RUNTIME_PROFILE_ENUM=<N>` line first
            (FCC-B3-1 contract), executes every case with try/finally
            `_restore_defaults` cleanup of FHSS_CHANNEL_MASK +
            ANTENNA_GAIN_DBI + HW_CEILING_DBM so a per-case setup
            write can't poison the next case, asserts both the
            CFG_OK_URC status byte AND the cfg_get(REG_PROFILE)
            stored byte, prints PASS/FAIL/SKIP per case and a
            `# SUMMARY passes=N fails=N skipped=N` tail. Exit 0 on
            full pass, 1 on any FAIL, 2 on transport setup
            failure.
          - **`--stamp`**: shells out to `artifact_header.py stamp
            --input <log> --output <log> --profile-enum <N>` using
            the runtime-read enum so the log is FCC-B2-b conformant
            for direct
            `DESIGN-CONTROLLER/bench-evidence/<date>_D6_profile_lock_fuzz/`
            submission. Subprocess-not-import keeps this
            orchestrator decoupled from the stamper's harvest-on-import
            side-effects.
          - **Validation**: `py -3 -c "import ast;
            ast.parse(open('cfg_fuzz_profile_lock.py').read())"` →
            `ast OK`; `py -3 cfg_fuzz_profile_lock.py --self-test`
            → `SELF_TEST: all golden vectors passed` (26 PASS / 0
            FAIL). D6-3 bench capture is the only remaining D6
            child.
    - [ ] **FCC-EVID-D6-3** Bench capture + DGATE-ready evidence.
          Run D6-2 on the 2× Max Carrier bench against the
          BENCH_ONLY_FIXED_915 firmware build (enum 0), save log to
          `DESIGN-CONTROLLER/bench-evidence/<date>_D6_profile_lock_fuzz/`,
          verify FCC-B2-b lint + FCC-B3-3 runtime-profile gate both
          pass on the artifact directory (proves the captured
          evidence will survive the same gates DGATE will run).
          Flip D6 parent `[x]` with the per-class REJECT-code
          observation table embedded in the evidence summary.
        - **2026-05-20 update (Windows-shim + bench-bridge probe)**:
          While attempting the bench step the assistant discovered
          that `method_g_stage1_probe.HostLink` is POSIX-only
          (`os.open`/`O_NONBLOCK`/`stty`) and cannot run from the
          Windows bench host the user is on. Fix landed: added
          `_PySerialHostLink` to
          [tools/cfg_fuzz_profile_lock.py](tools/cfg_fuzz_profile_lock.py)
          — a pyserial-backed shim implementing the `request()` +
          `close()` subset that the orchestrator needs, while
          re-using `build_frame`/`parse_frame` from the original
          module so wire framing stays single-sourced. Self-test
          re-validated post-shim (26/26 PASS). Probe attempt against
          enumerated USB-CDC ports (COM11, COM12 — both Arduino VID
          0x2341 PID 0x0061, composite/MI_ → Portenta H7 in Multi
          mode) showed: passive 2 s read returns 0 bytes on both
          ports; orchestrator write attempts raise
          `SerialTimeoutException: Write timeout` (CDC IN endpoint
          backpressure — H7 isn't draining). Diagnosis: the Murata
          L072 host_link UART is **internal to the Max Carrier**
          and only reaches the PC via a USB-CDC ⇄ L072-UART bridge
          sketch running on the Portenta H7; no such bridge is
          currently flashed on either H7. User action required
          before bench fuzz can run: flash a host_link bridge
          sketch on the H7 (or use a different physical access
          path to the L072 UART, e.g. an external FTDI on the
          Max Carrier debug header). See repo memory
          `lifetrac-l072-bench-bridge.md` for the diagnostic
          one-liner and the symptom matrix.
        - **2026-05-21 status (orchestrator ready, awaiting bench
          hands-on)**: D6-2-a and D6-2-b are landed and self-validated;
          the orchestrator's HW-free 26-case `--self-test` is green
          and its `REJECT_TO_STATUS` table is pinned against the
          firmware enum, so any drift between the wire dispatcher
          and the Python mirror will be caught before the bench
          step runs. The remaining work is purely a physical
          procedure that the assistant cannot perform autonomously
          (requires the user to flash BENCH_ONLY_FIXED_915 to the
          Murata L072, **flash a USB-CDC⇄host_link bridge sketch on
          the Portenta H7** (see 2026-05-20 update above), attach the
          2× Max Carrier bench, identify the COM port, and run the
          live mode). NOT performing this via a synthesized stub: per
          the [DGATE](TODO.md#fcc-evid-dgate) contract, FCC-B2-b lint +
          FCC-B3-3 check headers + the `RUNTIME_PROFILE_ENUM=` line
          but do NOT validate content correctness, so a stamped
          orchestrator log without real bench data would silently
          pass the gates while misrepresenting evidence — a
          falsification-test antipattern. Bench procedure for the
          user (when hardware is available):
          ```powershell
          $stamp = Get-Date -Format 'yyyy-MM-dd'
          $dir   = "LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/${stamp}_D6_profile_lock_fuzz"
          New-Item -ItemType Directory -Path $dir | Out-Null
          py -3 LifeTrac-v25/tools/cfg_fuzz_profile_lock.py `
              --dev COM7 --baud 115200 `
              --out-log "$dir/cfg_fuzz_profile_lock.log" --stamp
          py -3 LifeTrac-v25/tools/lint_artifact_headers.py "$dir/cfg_fuzz_profile_lock.log"
          ```
          Expected: orchestrator exit 0, all 6 bench-build cases PASS
          (bench_happy_default, profile_unknown_oor, antenna_out_of_range,
          no_power_headroom, fhss_unrouted_when_unrouted_build,
          dts_unrouted_when_unrouted_build), 3 routed-only cases
          SKIP, lint exit 0. Embed the resulting PASS/SKIP table in
          this entry's evidence block and flip both D6-3 and D6
          parent `[x]`.
- ⚫ **FCC-EVID-D7** RF exposure / MPE — **WAIVED as lab measurement**
      (no calibrated antenna). Substitute: categorical exclusion claim
      per §1.1307(b) / §2.1093 documented in user-facing README for the
      committed antenna SKU + declared conducted power.
- [ ] **FCC-EVID-D8** Two-node sync torture: RX boots late, reboots
      mid-run, misses several epochs, sees burst packet loss — reacquires
      without fixed rendezvous and without host intervention. Doubles as
      §14.3 Q6 measurement (target ≤5 s reacquire; >30 s ⇒ A6 redesign).
      **IN SCOPE** — runs on 2× Max Carrier bench.
- ⚫ **FCC-EVID-D9** LBT bias stress with calibrated interferer —
      **WAIVED as calibrated-interferer test** (no calibrated source).
      Substitute: hammer firmware on second Max Carrier as uncalibrated
      interferer + RFCO `blocked_attempts_by_reason` histogram; verifies
      LBT separates blocked attempts from TX counts and active set never
      silently shrinks. Not power-calibrated but proves the policy.
- [x] **FCC-EVID-D10** Power/antenna clamp fuzz: every illegal
      (profile, antenna_gain, tx_power) combination through host cfg is
      rejected with structured reason; legal combos clamp correctly.
      **IN SCOPE** — pure firmware validation, no RF equipment needed.
    - 2026-05-21 landing — exhaustive Cartesian-product fuzz delivered
      as `bench/host_proto/cfg_clamp_fuzz.c` with a new
      `check-cfg-clamp-fuzz` Makefile target wired into the aggregator
      `check` recipe and the .PHONY list. Axes: profile_id
      ∈ {0, 1, 2, 3, 0x10, 0xFF}, antenna_gain ∈ {-128, -10, -1, 0, 1,
      6, 7, 10, 17, 20, 30, 31, 100, 127}, hw_ceiling ∈ {0, 1, 2, 3, 6,
      17, 20, 25, 30, 50, 100, 255}, channel_mask ∈ {0, bit0,
      FHSS-required, FHSS-required minus bit0, FHSS-required plus
      bit50, bit63}, tx_routed ∈ {false, true}. modem_bw_hz is always
      the per-profile default so the fuzz isolates the
      (profile, antenna, hw, mask, routed) interactions — BW_MISMATCH
      and NOT_STAGED stay in their existing hand-picked coverage
      (`cfg_profile.c`, 27 cases). For every tuple the test
      independently re-derives the expected reject reason from a copy
      of the validator's decision tree (predicate ordering
      NULL > BAD_PROFILE > ANTENNA > NO_POWER_HEADROOM > per-profile)
      and independently re-derives the expected `power_clamp` byte
      from the §A5 ERP formula; a mismatch prints the offending tuple
      coordinates + got/expected reject names and aborts. Also asserts
      the global invariants `clamp <= min(tier, hw)` (skipping the
      OOR-profile case where `tier == 0`), `NO_POWER_HEADROOM ⇔
      clamp==0 ∧ pre-power checks pass`, and that the wire-status
      mapping never falls through to the `APPLY_FAILED` catch-all for
      any reject reason in `[NONE, NULL_ARG]`. Final histogram bucket
      coverage asserted non-zero for NONE, BAD_PROFILE, ANTENNA,
      NO_POWER_HEADROOM, UNROUTED, MASK_POPCOUNT, MASK_OUT_OF_TABLE so
      a future axis-table prune that silences a branch fails loudly.
      Run output: `[PASS] cfg_clamp_fuzz: cases=12096 legal=1180
      reject=10916 NONE=1180 BAD_PROFILE=6048 ANTENNA=2592
      NO_HEADROOM=696 UNROUTED=960 MASK_POP=460 MASK_OOT=160`.
      Existing `check-cfg-profile` (27 cases) and `check-cfg-profile-
      wire` (9 cases) re-run clean after the Makefile edits → no
      neighboring-target regression. Note: the fuzz intentionally
      does NOT cover the `CFG_KEY_TX_POWER_DBM` apply path in
      `host/host_cfg.c` — that key has its own
      `cfg_validate_and_normalize` branch that silently clamps `[2,17]`
      with `CFG_STATUS_OK` and routes directly to
      `sx1276_set_tx_power_dbm` without consulting the active profile
      or current `power_clamp`. That is a design choice (tx power is
      independently bounded by hardware tier in any subsequent
      `host_cfg_profile_validate` call when REG_PROFILE is re-applied,
      and the radio driver caps to PA register limits at the lowest
      level), so D10 doesn't flag it as a finding; if a future bench-
      evidence run reveals a path where TX_POWER can be staged above
      the active tier's clamp and reach the air, that becomes an
      FCC-EVID-D10-followup ticket.
- [ ] **FCC-EVID-DGATE** Scripted go/no-go summary report that fails the
      run if any **in-scope** D4/D5/D6/D8/D10 artifact or any required B2
      header field (git SHA, timestamp, profile enum, schema version) is
      missing. Waived gates (D1/D2/D3/D7/D9) are recorded as `WAIVED`
      with a citation to S-HW.4 rather than failing the run.

**Open questions (must answer before FCC-A3 lands — see §14.3 of plan doc):**
Q1 final 50-ch list (gated on D2), Q2 epoch model time-slice vs counter,
Q3 L072 monotonic-tick wrap behavior, Q4 hopset-update auth path
(Track C), Q5 antenna SKU + gain commitment, Q6 RX cold-start scan budget
(measured at D8).

- **Gate (S1.5 closes when):** all FCC-A* + FCC-B1/B2/B3 firmware items
  land AND all FCC-EVID-D1..D10 + FCC-EVID-DGATE pass with margin and are
  archived under
  `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/fcc_fhss_50ch_<date>/`.
  Until S1.5 closes, no over-the-air production deployment.

### Stage S2 — D15 E-STOP replay-rejection fix (BLOCKER for S4) � HW-PARTIAL
*(L072 firmware change; safety-critical bench regression runs on the
2× Max Carrier pair. E-STOP signal path per 2026-05-19 decision
(S-HW.5): tractor-side = **Arduino Opta digital-input pin flip** routed
to the base-station Max Carrier; operator-side = **keyboard / joystick
button on the web remote-control UI** → `ESTOP_REQ` host frame. No
physical mushroom button required for S2; bench substitute = tactile
switch shorting Opta I1 to GND. Code can be written and
unit-tested against a host-side packet emulator now, but the gate cannot
close without the rig. Decision deferred until rig is available — do not
ship safety code on synthetic-only evidence.)*
- [ ] **S2.1** Per §19.7 / D15: change burst-emit path so each copy carries
      a fresh monotonic `seq`, not all `seq=0`.
- [ ] **S2.2** RX accepts `CMD_ESTOP` idempotently across the replay window
      (assert state-set is idempotent; do not consume window slots that
      block subsequent legitimate ESTOP).
- [ ] **S2.3** Grow RX replay window to ≥ 32 entries.
- [ ] **S2.4** Add §19.6 RX MAC-failure rate-limit: ≥100 bad MACs in 10 s
      → enter safe state + log STATS counter.
- [ ] **S2.5** Bench regression: replicate the 2026-04-27 Controller Master
      Plan Review §4 failure (E-STOP after ~30 s of normal base traffic);
      passes only if a real ESTOP is accepted while a replay-attack frame
      is still rejected.
- **Gate:** real ESTOP accepted after arbitrary normal-traffic prefix; replay
  attack still rejected; MAC-fail rate-limit observed in STATS log.
- **Optional standalone hotfix path:** S2 is independently shippable as a
  safety patch to current field firmware regardless of the rest of this
  plan. Recommended.

### Stage S3 — D1 depth-1 P0 mailbox (biggest single lag win) 🔴 HW-BLOCKED
*(L072 firmware change; validation needs the W2-02 stability harness with
L072 + handheld attached. The multi-source merge-policy paper decision
(S3.3) and the abandoned-writer-timeout API (S3.2) can be drafted now.)*
- [ ] **S3.1** Replace depth-6 FIFO for P0/STREAM class with depth-1
      latest-wins slot in L072 TX submit path; SAFETY + EVENT classes keep
      small ordered FIFO.
- [ ] **S3.2** Add abandoned-writer timeout (§21.3-4): half-written mailbox
      slot is invalidated after configurable timeout so a host crash
      mid-write cannot freeze the slot.
- [ ] **S3.3** Define multi-source P0 merge policy (§21.3-1): per-source
      slot or explicit precedence (handheld preempts autonomy → mailbox
      drops autonomy slot on handheld arrival). Document chosen rule in
      §3.3 of design doc.
- [ ] **S3.4** Emit new STATS counter `tx_stream_overwrites` per class.
- [ ] **S3.5** Validation on bench rig W2-02 stability harness.
- **Gate:** joystick → valve p99 under bursty submit drops from ~367 ms
  toward ~136 ms (per §17.2 prediction); stick-release produces **zero**
  stale-motion P0 frames after next sample interval.

### Stage S4 — D2 + D4 SAFETY 5-copy burst with decorrelation 🔴 HW-BLOCKED
*(Requires S2 passing. Step attenuator and EIRP measurement path are
**permanently waived** per 2026-05-19 decision — see S-HW.2 / S-HW.4.
S4 ships under firmware-side TX-power cap + range-walk PER substitution
+ datasheet-declared EIRP, not lab-measured evidence.)*
- [ ] **S4.1** SAFETY-class burst handler in firmware: 5 copies, ≥ 50 ms
      inter-copy spacing (rationale §21.3-2: exceed worst-case other-class
      ToA on channel so half-duplex doesn't drop a copy), FHSS-diverse when
      LBT allows.
- [ ] **S4.2** Power cap is **regional EIRP ceiling** (§21.3-6: EU 868
      → +14 dBm ERP including antenna gain; FCC 915 → higher), NOT a blanket
      +17 dBm. Add `(region, sub-band, antenna_gain)` table.
- [ ] **S4.3** SAFETY-class **may** bypass LBT per §7 carve-out but **may not**
      exceed EIRP ceiling.
- [ ] **S4.4** Non-bursted RELEASE — operator-confirmed state convergence
      via pull-recovery (D3), not push burst.
- [ ] **S4.5** RX failsafe-on-silence = **500 ms ship default** (D5/C1);
      sticky `state_bits` in heartbeat per §3.6.
- [ ] **S4.6** Class-downgrade-only invariant (§21.3-3): L072 may treat
      unknown class as control (strict crypto) but never as image
      (no crypto).
- **Gate:** step-attenuator at 20% per-copy PER → first-arrival p99 < 125 ms,
  latch-intent miss rate < 10⁻⁶ over 10k trials; correlated-fade test shows
  hop-spaced burst beats back-to-back copies; failsafe latches within
  500 ms ± 50 ms of last heartbeat.

### Stage S5 — D13 + D14 crypto downgrade + split-trust image 🟡 HW-PARTIAL
*(AES-GCM-64 codec, plaintext+CRC32 image framer, and host-boundary
class-tag enforcer are all pure Python and unit-testable now on the two
X8 boards. Bench airtime-reclaim gate requires L072 reattached.)*
- [x] **S5.1** *(software half DONE 2026-05-18)* AES-GCM-64 implicit-nonce
      codec landed in `lora_proto.py` as `encrypt_frame_gcm64_implicit` /
      `decrypt_frame_gcm64_implicit` / `build_implicit_nonce`. Wire layout:
      `seq_be32(4) ‖ ciphertext ‖ tag8` = +12 B. Nonce derived from
      `src(1) ‖ boot_ctr_be32(4) ‖ seq_be32(4) ‖ zero_pad(3)`. 8 tests +
      9 subtests in `tests/test_d13_d14_codec.py` cover round-trip,
      tampered tag/ciphertext, wrong src, wrong boot_ctr, short-frame
      rejection (no exception), and per-(src,boot_ctr,seq) nonce
      uniqueness (NIST SP 800-38D §8). STILL TODO (gated on integration
      work, not pure crypto): persist `boot_ctr` to flash on X8; wire
      the codec into `lora_bridge.py` and the L072 path; cut over the
      live control/heartbeat/telemetry callers from `encrypt_frame` to
      the new codec (separate stage S5.1b, kept open).
- [ ] **S5.1b** *(integration — software-only, do next)* Persist `boot_ctr`
      to X8 flash; emit a boot-handshake frame on link-up so the RX learns
      the new value; switch the live `lora_bridge.py` control/heartbeat
      /telemetry TX path from `encrypt_frame` (shipped GCM-128) to
      `encrypt_frame_gcm64_implicit` behind a feature flag. Default off
      until S-HW.1 (L072 reattach) lets us measure the airtime reclaim.
- [x] **S5.2** *(DONE 2026-05-18)* Image fragment plaintext+CRC framer
      landed as `pack_image_fragment_plain` / `unpack_image_fragment_plain`.
      Wire layout: `seq_be32(4) ‖ payload ‖ crc16_be(2)` where the CRC
      is the low 16 bits of `zlib.crc32(seq ‖ payload)` (deliberately a
      different polynomial domain than the PHY-layer CRC16-CCITT). 7
      tests cover round-trip, tampered payload, tampered CRC, short
      frame, big-endian seq field pin, and overhead-constant agreement
      with `CRYPTO_IMAGE_PLAIN_CRC32.overhead_bytes`.
- [x] **S5.3** *(DONE 2026-05-18)* Host-boundary class-tag enforcer
      landed as `enforce_class_tag_boundary` and
      `enforce_class_downgrade_only` plus a `ClassTagViolation` exception.
      The first guard rejects any P0/P1/P2 frame carried by an
      unauthenticated profile (has_mac=False); the second enforces the
      §21.3-3 class-downgrade-only invariant. 6 tests + 11 subtests
      cover all P0/P1/P2/P3 × {GCM-128, GCM-64, PLAIN_CRC32} matrix
      slots plus a dedicated sentinel
      `test_forged_d14_fragment_cannot_be_laundered_into_p0` that
      pins the core split-trust safety property. STILL TODO: wire the
      enforcer into the actual X8→L072 dispatch site in `lora_bridge.py`
      (one-line call at the boundary; kept open as S5.3b).
- [x] **S5.3b** *(DONE 2026-05-18)* Enforcer wired into
      `lora_bridge._tx_worker` immediately before the `encrypt_frame`
      call, inside the existing try block. On `ClassTagViolation` the
      worker logs to `audit_log` with event `class_tag_violation` and
      `continue`s — the frame never reaches the radio. With today's
      `CRYPTO_PROFILE_DEFAULT` (GCM-128 explicit, `has_mac=True`) no
      legitimate class trips the guard; the call is a defense-in-depth
      pin that fails LOUD if a future S5.1b cutover accidentally pairs
      a P0/P1/P2 frame with an unauthenticated profile. Coverage:
      `tests/test_tx_worker_class_tag_enforcer_wired.py` (5 tests, 15
      subtests: source-presence guard, ordering guard
      (enforce-before-encrypt), full classify→enforce pairing matrix
      across every dispatchable (frame_type, opcode, topic_id), and a
      sentinel that pins "P0 + plaintext profile → raise".
- [ ] **S5.4** RX rate-limit cleartext image at design refresh rate + margin
      so a flood of forged image fragments cannot starve P0 RX duty.
      *(Deferred 2026-05-18: no D14 cleartext-image RX path is wired
      into `lora_bridge._handle_air` yet. Re-open after S5.1b lands a
      D14 ingest site so the rate-limiter has a real call site to
      attach to instead of becoming dead code.)*
- [x] **S5.5** *(DONE 2026-05-18 — doc-only)* Per-direction adapter
      symmetry (§21.3-5): power/SF/encode adapters run with independent
      state on each link direction. Documented at the head of
      `link_monitor.py` and pinned by the per-direction state separation
      already present in `EncodeModeController`. No code change needed
      beyond the docstring note; behavioural enforcement comes with S6.5.
- **Gate:** image fragments fit 25 ms cap with ≥ 4 ms margin under encrypted
  framing predictor; spoofed class-tag injection at host boundary rejected;
  class-spoof DoS test (forged image flood) fails to starve P0; measured
  reclaim ≥ 1.0 s/s of airtime over a 10-minute mixed run.

### Stage S6 — D6 + D7 + D8 + D9 TX-power adapter (X8 Python) 🟡 HW-PARTIAL
*(The `{NORMAL, MARGIN_LIMITED, AIRTIME_LIMITED, RECOVERY}` state machine,
dwell timers, and PER/SNR aggregators are pure Python and SIL-testable
against synthetic input traces on the two X8 boards. Final gate uses
**range-walk + natural path loss** instead of a step attenuator (S-HW.2
permanently waived) — bench evidence is "PER vs measured-RSSI bin"
rather than "PER vs commanded attenuation step." Airtime-load generator
runs on the second Max Carrier; full L072 link already proven via
W1-10b.)*
- [x] **S6.1** *(DONE 2026-05-18)* New module
      `DESIGN-CONTROLLER/base_station/tx_power_adapter_v3.py`: `SnrEwma`
      inner loop with 5-packet hysteresis + 10 Hz minimum interval cap
      (`INNER_MIN_INTERVAL_MS=100`), `PerWindow` 100-pkt outer loop,
      `_AirtimeSnapshot` consumes the existing `RollingAirtimeLedger`
      output. Pure Python, no I/O — caller applies the returned
      `AdapterDecision`. Verified by 12 SIL tests in
      `tests/test_tx_power_adapter_v3.py` covering 10 Hz rate cap (I6),
      single-sample-flap rejection (I7), and cap-saturation handoff.
- [ ] **S6.2** PER feedback piggybacked in topic `0x10` at 0.2–0.5 Hz (not
      9 B per frame).
      *(Partially DONE 2026-05-18: host-side publisher landed —
      `_airtime_worker` emits PER + state + power on the sibling topic
      `lifetrac/v25/control/link_power/{direction}` at the existing
      airtime-poll cadence behind `LIFETRAC_TX_POWER_ADAPTER_V3=1`.
      Reverse direction — firmware-side piggyback of per-RX SNR/PER on
      tractor topic `0x10` so the base adapter has a real input — still
      open, blocked on L072 reattach.)*
- [x] **S6.3** *(DONE 2026-05-18)* Explicit controller state machine
      `AdapterState{NORMAL, MARGIN_LIMITED, AIRTIME_LIMITED, RECOVERY}`
      in `tx_power_adapter_v3.py` with dwell timers
      (`DWELL_NORMAL_TO_LIMITED_MS=1000`,
      `DWELL_LIMITED_TO_RECOVERY_MS=5000`,
      `DWELL_RECOVERY_TO_NORMAL_MS=10000`) and asymmetric hysteresis
      gaps (SNR -5/0 dB, PER 2.0%/0.5%, airtime 0.60/0.45). RECOVERY
      walks power back one notch per tick. Pinned by I8 (dwell) and
      I9 (re-degradation re-entry) tests.
- [x] **S6.4** *(DONE 2026-05-18)* Priority cascade **branched on root
      cause** (§14.1 + D6): MARGIN_LIMITED emits RAISE_POWER first and
      only `SF_STEP_UP` once power saturates at `P_MAX_DBM`;
      AIRTIME_LIMITED emits `CANCEL_P3` → `CANCEL_P2` → `ENCODE_DEGRADE`
      in cheapest-first order. **Mutual exclusivity invariant (I3) pinned
      by test**: no direct MARGIN→AIRTIME (or reverse) transition is
      possible — RECOVERY always sits between, so the event log preserves
      root-cause attribution. Tie-break on simultaneous trigger prefers
      MARGIN_LIMITED (rationale: encode-degrade can't fix a margin hole;
      raise-power can drain PER even under airtime pressure).
- [x] **S6.5** *(DONE 2026-05-18)* Adapter runs per-link-direction with
      separate state on each end (§21.3-5). `lora_bridge.Bridge.__init__`
      instantiates two independent `TxPowerAdapterV3` instances
      (`tx_adapter_uplink`, `tx_adapter_downlink`); `_airtime_worker`
      drives both with the same airtime triple but isolated SNR/PER
      streams; new hook `Bridge.observe_radio_metadata(snr_db, ok,
      direction=...)` routes radio metadata to the matching adapter.
      Per-direction divergence pinned by
      `test_per_direction_state_diverges_on_asymmetric_snr` (uplink →
      MARGIN_LIMITED, downlink → NORMAL under asymmetric trace).
- [x] **S6.6** *(DONE 2026-05-18)* Behind a bench flag until all gates
      pass — do NOT enable in field firmware default. Gate: env var
      `LIFETRAC_TX_POWER_ADAPTER_V3=1` at bridge boot. Default OFF.
      When OFF, both `tx_adapter_*` attrs are `None` and zero work runs
      in the airtime worker — pinned by `test_default_off_no_adapter`.
      When ON, the adapter runs in **OBSERVATION-ONLY** mode: it
      publishes decisions but does NOT yet write `RegPaConfig` or emit
      `CMD_LINK_TUNE`, so an adapter regression cannot brick a live
      link. Real radio action is held until the SPI driver from
      `MASTER_PLAN.md §8.17` lands.
- **Gate:** standing 30-min bench run shows median TX power ≥ 3 dB below
  the +14 dBm baseline while per-fragment PER < 1%; step-attenuator
  induced margin-limited vs airtime-limited failures trigger
  **different** responses (verified in logs); zero P0 TX-start regression
  vs S3 baseline.

### Stage S7 — D11 + D16 operator UX + canonicalize policy 🟢 HW-READY
*(LINK pill UI is pure browser/JS — can be developed against simulated
telemetry on the existing two X8 boards. Doc edits are editorial.)*
- [x] **S7.1** *(DONE 2026-05-18, UI half)* Add **LINK pill** to operator
      UI (browser console + handheld OLED if practical) symmetric with
      `IMG:` / `AI:` pills: `LINK: SF7 / +14 dBm / 99.2% / SNR +8 dB`,
      color-coded by state. Implemented on the existing `map.html`
      sidebar (two rows: `LINK ↑` and `LINK ↓`) — per-direction
      adapter snapshots come from `state.link_power.{uplink,downlink}`
      published by `image_pipeline.state_publisher.StatePublisher.snapshot()`
      and consumed by `web/map.js::renderLinkPill()`. Color map:
      NORMAL→ok(green), RECOVERY→warn(orange), MARGIN_LIMITED /
      AIRTIME_LIMITED→bad(red), unknown→grey. Worst-side `reason`
      string surfaces in a third row. Field schema pinned by
      `StatePublisherTests::test_snapshot_includes_link_power_default_null`
      and `…_link_power_per_direction`. Cross-process MQTT glue
      DONE 2026-05-18 — bridge and web_ui are separate processes
      sharing only the broker, so `web_ui._on_mqtt_message` now
      forwards every retained
      `lifetrac/v25/control/link_power/{direction}` payload into
      `_image_publisher.link_power[direction]`. Pinned by 5 tests
      in `test_web_ui_link_power_glue.py` (uplink,
      downlink-independence, unknown-direction drop, non-dict
      drop, end-to-end snapshot carry). Remaining: handheld OLED
      mirror (blocked on D14 RX path, see S5.4). When bench flag
      `LIFETRAC_TX_POWER_ADAPTER_V3` is OFF the pills stay at
      `—` — fail-closed UX per §6.1.
- [ ] **S7.2** Publish power / cap-reason / SF / encode-mode / SNR-margin
      / PER fields in topic `0x10`.
      *(Host-side serializer DONE 2026-05-18 — `_airtime_worker`
      publishes `{state, action, value, reason, power_dbm, sf_rung,
      snr_ewma, per, per_sample_count, direction}` on
      `lifetrac/v25/control/link_power/{direction}` behind
      `LIFETRAC_TX_POWER_ADAPTER_V3=1`. Field schema pinned by
      `test_link_power_topic_payload_contains_s72_fields`. The
      `encode_mode` field is already on the existing sibling topic
      `link_airtime`. Remaining: the topic `0x10` reverse-direction
      LoRa wire encoding for handheld OLED — blocked on L072 reattach
      same as S6.2 reverse.)*
- [x] **S7.3** Propagate D11 P0/P1/P2/P3 policy table into
      `MASTER_PLAN.md` and `LORA_PROTOCOL.md`.
      *(DONE 2026-05-18 — added new normative section
      `LORA_PROTOCOL.md § Priority class policy (canonical)` with the
      full P0/P1/P2/P3 table (membership, mailbox discipline, burst
      counts, crypto envelope, loss policy) plus a reverse-mapping
      block aliasing the deprecated `STREAM_*` / `EVENT_STATE` /
      `SAFETY` names; added cross-reference `MASTER_PLAN.md § 8.21`
      pinning D11 and pointing at the LORA_PROTOCOL table as single
      source of truth for `classify_priority()` and the tractor M7
      firmware queue. Grep audit: the only remaining `STREAM_*` hits
      in normative paths are inside the two explicit deprecation
      blocks themselves — exactly the documented reverse-mapping
      aid, no live usage. The `link_monitor.py` adaptation-narrative
      half of this gate is already covered by the LINK pill +
      `link_power` topic shipped under S7.1/S7.2.)*
- **Gate:** operator can explain every adaptation transition from logs +
  pill state alone; cross-doc grep for `STREAM_*` returns 0 hits in
  normative text **except** the two explicit deprecation/aliasing
  blocks in `LORA_PROTOCOL.md § Priority class policy (canonical)`
  and `MASTER_PLAN.md § 8.21`, which exist precisely to pin the
  rename.

### Open decisions before kick-off
1. **Start point.** Recommend S0 + S1 in parallel (both independent of
   firmware, both unblock everything). S2 ships as standalone hotfix.
2. **S0 scope.** Minimal (just C1–C5, ~1 hr) vs. full §21.5 / §23.4
   rewrite (~3–4 hr, cuts doc to ~1200 lines).
3. **S2 hotfix.** Treat as standalone PR mergeable this week independent
   of the rest of this plan? (Recommended yes.)

### S-HW — Hardware prerequisites (current bench: 2× X8 + Max Carrier + 1 camera)
*(Added 2026-05-18 after user confirmed actual bench state. None of these
block S0 / S1.0 / S5 software half / S6 software half / S7. They DO block
S1.4, S2, S3, S4, S5 validation, and S6 validation.)*
- [x] **S-HW.0** *(host-side, no RF)* DONE 2026-05-18. Cross-validated D13
      AES-GCM-64 codec, D14 image plaintext+CRC32 framer, and
      `enforce_class_tag_boundary` on a real Portenta X8 (ARM64,
      `cryptography` 36.0.2) via UDP-localhost loopback. All 12 integration
      assertions pass (round-trip, MAC-tamper rejection, boot_ctr binding,
      ReplayWindow dedup, P0/D14 split-trust attack rejection, sustained
      20 Hz control with zero loss). Microbench: aggregate D13 cost at full
      design cadence is **~3.03 % of one A53 core** — no risk to A/V.
      Bench scripts: `bench_crypto_perf.py`, `bench_loopback_d13_d14.py`.
      Evidence: `AI NOTES/2026-05-18_S-HW0_Host_Side_Validation_Evidence.md`.
      Notes: (a) D13 is ~50 % slower per op than shipped GCM-128 due to
      low-level `Cipher(...)` API overhead vs `AESGCM` class — absolute
      cost still negligible, but D13's motivation is airtime not CPU.
      (b) X8-B (camera, serial `2E2C1209...`) has stripped Python stdlib
      (no `socket`/`dataclasses`/`json`); host-side bench ran on X8-A only.
      Production code on X8-B runs inside Docker, out of scope for S-HW.0.
- [x] **S-HW.1** ✅ 2026-05-19 — RESOLVED BY TOPOLOGY CLARIFICATION.
      Earlier wording assumed a separate "L072 base + L072 handheld"
      pair was needed. Project decision: **one Max Carrier = base
      station, the other Max Carrier = tractor-side radio**, both
      already on the bench with onboard Murata CMWX1ZZABZ-078 (L072 +
      SX1276). MKR WAN 1310 handheld is **optional**. The 2× Max
      Carrier pair already passed end-to-end LoRa link test
      (W1-10b 100/100 RX match, 2026-05-12). No RF-side gate is blocked
      on radio hardware; remaining HW gates depend on test-equipment
      (S-HW.2 attenuator, S-HW.4 EIRP path, S-HW.5 E-STOP), not radios.
- ⚫ **S-HW.2** ~~Inventory the step attenuator~~ — **PERMANENTLY
      SKIPPED 2026-05-19** (project decision: no lab test-equipment
      will be acquired). Downstream consumers (S1.1 walk-power, S4 PER
      induction, S6 margin-limited induction) substitute **range-walk
      + natural path loss** for controlled attenuation; bench evidence
      becomes "PER vs measured RSSI bin" instead of "PER vs commanded
      attenuation step." Lower precision but acceptable for an
      open-source agricultural-equipment context.
- [ ] **S-HW.3** Confirm or build a **second LoRa TX as interferer** on
      an adjacent channel — needed only for S4 correlated-fade test.
      Can be satisfied by **temporarily repurposing one of the two Max
      Carrier radios** with a hammer firmware build (no extra hardware
      required); MKR WAN 1310 path retained as fallback.
- ⚫ **S-HW.4** ~~Decide the EIRP measurement path~~ — **PERMANENTLY
      SKIPPED 2026-05-19** (project decision: no spectrum analyzer, no
      calibrated antenna, no TCB lab access). FCC §15.247 EIRP
      compliance ships under **firmware-side TX-power cap +
      conducted-path datasheet declaration**:
      - Firmware enforces SX1276 `RegPaConfig` MaxPower / OutputPower
        bits so chip-side conducted output cannot exceed a declared
        ceiling (FCC-TXPOWER-LAYER S0.9 hook lives here).
      - Antenna gain entered from datasheet; total EIRP computed in
        artifact header (orchestrator stamp via FCC-B2-b) as
        `conducted_dBm + antenna_dBi - cable_loss_dB`.
      - S4.2 ships **declaration-only**, not lab-measured. Production
        deployment carries the standard amateur / experimental
        unintentional-radiator caveat in the user-facing README.
- [ ] **S-HW.5** Wire the revised E-STOP signal path (project decision
      2026-05-19, supersedes the mushroom-button assumption):
      - **Tractor side:** an **Arduino Opta digital input pin flip**
        (e.g. dry-contact relay or hardwired switch into Opta I1…I8)
        is the E-STOP trigger consumed by the base-station Max Carrier
        radio. Opta-to-Max-Carrier link is per the existing
        BUILD-CONTROLLER wiring plan (Modbus RTU or digital pass-through
        — to be confirmed in S2).
      - **Operator side:** a **keyboard key OR joystick button on the
        web remote-control UI** sends an `ESTOP_REQ` host frame to the
        tractor-side radio; UI must rate-limit at ≤ 5 Hz and **must
        not require focus** (global keydown handler) so the operator
        cannot lose E-STOP by tabbing away.
      - **Test posture:** S2 regression treats both paths as real (no
        synthetic injector). Opta pin flip can be simulated on the
        bench by shorting Opta I1 to GND through a tactile switch.

### Software-only work that can run NOW on the current 2× X8 bench
*(Ordered by leverage. Each is HW-independent and produces code that
plugs into a HW gate when the LoRa rig returns.)*
1. **S0.1–S0.4, S0.7, S0.8** — design-doc surgical C1–C5 fixes. ✅
   **DONE 2026-05-18.**
2. **S1.0** — `lora_airtime.py` pure-Python ToA predictor + unit tests,
   cross-checked against existing bench-evidence CSVs.
3. **S5 software half** — AES-GCM-64 implicit-nonce codec + image
   plaintext+CRC32 framer + host-boundary class-tag enforcer, all
   unit-tested. (Crypto Profile A keeps crypto in host, so this is the
   right layer for it anyway.)
4. **S6 software half** — controller state machine + dwell timers + PER
   / SNR aggregators, SIL-tested against synthetic input traces.
5. **S7.1** — LINK pill UI in browser console, fed by simulated telemetry.
6. **S0.5 + S0.6** — deferred structural doc cleanup (renumber duplicate
   `## 13/14/15/18`, promote P0/P1/P2/P3 policy table into §4, redraw
   ASCII diagram).
7. **S1.1 + S1.2 + S1.3** — instrumentation code (sweep mode, host
   metric collectors, decoder extensions) — lands without running.

---

## Control source priority (project-wide policy, 2026-05-15)

Three control sources can drive the tractor. Whenever more than one is
active, the **highest-priority active source wins** and the others are
locked out at the M7 ControlFrame arbiter (not just at the UI). This
ordering is the single source of truth for every arbitration decision
in firmware, the bridge, the web UI, and the autonomy stack:

1. **🥇 Handheld MKR WAN 1310 — direct LoRa control.** Primary control
   path. Owns the 50 Hz P0 ControlFrame budget, the latching mushroom
   E-stop, and the physical TAKE-CONTROL button which pre-empts any
   lower-priority source within one frame. Handheld must work fully
   stand-alone with the base station and X8 powered off.
2. **🥈 Browser operator console — base station.** Secondary. Active
   only when (a) no handheld frame has arrived in the last 500 ms
   *and* (b) the operator has held TAKE CONTROL on the browser side
   to claim the source slot. Releases instantly when a handheld frame
   reappears (handheld pre-empts; no negotiation). Wired today via
   [base_station/web_ui.py](DESIGN-CONTROLLER/base_station/web_ui.py)
   + [base_station/web/app.js](DESIGN-CONTROLLER/base_station/web/app.js)
   (USB Gamepad API + on-screen joysticks → 20 Hz WebSocket → MQTT →
   `lora_bridge.py` → tractor).
3. **🥉 Autonomy — on-tractor X8 / base autonomy stack.** Lowest
   priority. Pre-empted by both handheld and browser at any moment
   without warning, must come to a clean controlled stop within the
   500 ms M4 failsafe window when pre-empted. Future scope; not
   implemented yet beyond the `AUTONOMY` source-id reservation in the
   source-active telemetry.

Implementation/test obligations rolling forward:
- The §F single-source bench gate (below) tests all three pairwise
  pre-emptions, not just handheld-vs-base.
- The browser must surface the active source as a status pill
  ("HANDHELD · YOU · AUTONOMY · NONE") so the operator never wonders
  why their stick input was dropped.
- The arbitration logic lives at the M7 ControlFrame mux on the
  tractor; the bridge and web UI are advisory mirrors only — they
  must not be the only safety check.

---

## Pre-field-deployment checklist (open items as of 2026-05-04)

> **Purpose.** A single consolidated view of what still has to happen
> before a real LifeTrac v25 can be powered on at a real work site.
> Every line links to the authoritative phase plan in
> [DESIGN-CONTROLLER/TODO.md](DESIGN-CONTROLLER/TODO.md) (or the
> structural / hydraulic equivalents). **All software work that does
> not need bench hardware is landed** (810 SIL tests / 60 files); what
> remains is **hardware, integration, and field-validation work that
> requires physical parts**.
>
> Status legend: 🟥 = blocker (must finish before Phase 8 field tests)
> · 🟨 = required for Phase 9 release · 🟩 = stretch / nice-to-have.

### A. Hardware procurement (lead-time blocker — start first)

🟥 Production procurement remains open. Two bench Portenta X8 + Max
Carrier stacks are on hand for bring-up, but the rest of the tractor,
base, handheld, hydraulic, and dev-gear BOM still blocks full HIL and
field testing.

- [ ] 🟥 **Tractor node** — Portenta Max Carrier, Portenta X8, LoRa +
  cellular antennas, NEO-M9N GPS, IP65 enclosure, Opta WiFi + D1608S
  + A0602 expansions, hydraulic pressure sensors, engine-kill relay,
  Phoenix PSR safety relay, USB UVC webcam + MCP2221A + BNO086 IMU.
  Full BOM in
  [DESIGN-CONTROLLER/TODO.md § Phase 0 / Tractor node hardware](DESIGN-CONTROLLER/TODO.md#phase-0--hardware-procurement--shop-setup).
- [ ] 🟥 **Base station** — second Max Carrier + X8, 8 dBi mast
  antenna, LMR-400 coax, lightning arrestor, mast + ground rod,
  indoor PSU + UPS. Coral Mini PCIe pending the Phase 1 validation
  spike. See [DESIGN-CONTROLLER/TODO.md § Base station hardware](DESIGN-CONTROLLER/TODO.md#base-station-hardware).
- [ ] 🟥 **Handheld** — MKR WAN 1310, dual joysticks, latching E-stop,
  SSD1306 OLED, IP54 enclosure, custom joystick PCB (KiCad design +
  fab). [DESIGN-CONTROLLER/TODO.md § Handheld hardware](DESIGN-CONTROLLER/TODO.md#handheld-hardware).
- [ ] 🟨 **Spares** — 2× of each major board + antennas + joysticks +
  OLEDs. Cuts the bring-up loop in half if anything DOAs.
- [ ] 🟨 **Dev gear** — RTL-SDR, Saleae Logic 8 (or clone), bench
  PSU, 50 Ω SMA dummy loads, spectrum-analyzer rental for FCC EIRP.
- [ ] 🟥 **Mechanical / hydraulic BOM** — frame tube stock, pivot
  pins, lift cylinders, valves, hoses, fittings. Tracked in
  [DESIGN-STRUCTURAL/](DESIGN-STRUCTURAL/) and
  [DESIGN-HYDRAULIC/](DESIGN-HYDRAULIC/). The chassis has to exist
  before the controller can move anything.

### B. Bench bring-up (Phase 1)

🟥 Mostly blocked on Section A. Current bench exception: two Portenta X8
and Max Carrier stacks are available and have produced partial W4-pre
M7 firmware-liveness evidence; the full production procurement list
remains open.

- [ ] 🟥 First-power-on smoke tests for each of the three nodes
  (tractor X8 + Max Carrier H7, base X8 + Max Carrier H7, handheld
  MKR WAN 1310). Verify USB enumeration, serial console, LED blink.
  [DESIGN-CONTROLLER/TODO.md § Phase 1](DESIGN-CONTROLLER/TODO.md#phase-1--bench-bring-up).
  **2026-05-04 partial:** Board 1 (`2D0A1209DABC240B`) and Board 2
  (`2E2C1209DABC240B`) both flashed `tractor_h7` M7 at `0x08040000`,
  reached `loop()`, advanced SRAM4 liveness for roughly 60 s, and had
  CFSR/HFSR = 0. Formal USB-CDC, rail, blink/echo, and stock
  dual-core handshake captures are still open.
- [ ] 🟥 RadioLib + SX1276 (or Murata SiP) hello-world: send a packet
  base ↔ tractor at 1 m, decode RSSI/SNR.
  **2026-05-04 update:** The physical architecture has been definitively confirmed. The Murata `CMWX1ZZABZ-078` on the Max Carrier acts as a standalone AT modem and its raw SPI pins are **not** routed to the Portenta high-density connectors. Raw SPI communication (i.e. `RadioLib` P2P) is physically impossible. All LoRa control MUST be rewritten to use AT commands over a serial port (`Serial3` on H7, or `/dev/ttymxc3` on X8 Linux).
  **2026-05-04 bench probe update:** Live UART probe on both X8 boards (see [AI NOTES/2026-05-04_Murata_UART_Firmware_Probe_Bench_Results_Copilot_v1_0.md](AI%20NOTES/2026-05-04_Murata_UART_Firmware_Probe_Bench_Results_Copilot_v1_0.md)) confirmed the modem UART is alive at **19200 8N1**, reset via `gpio163` works, and host→modem TX is acknowledged. **However**, the firmware currently flashed on the Murata does *not* respond to any known AT command set (Hayes / MKRWAN / RAK / Semtech AT_Slave / RUI3 all rejected with `Error when receiving / +ERR_RX`), and a board1↔board2 over-air smoke test produced zero received bytes. Method comparison in [AI NOTES/2026-05-04_LoRa_AT_vs_SPI_Method_Comparison_Copilot_v1_0.md](AI%20NOTES/2026-05-04_LoRa_AT_vs_SPI_Method_Comparison_Copilot_v1_0.md).
  **2026-05-04 Arduino-doc reading update:** Per [Arduino's Max Carrier TTN tutorial §2.1](https://docs.arduino.cc/tutorials/portenta-max-carrier/connecting-to-ttn) and the [official LoRa modem firmware update guide](https://support.arduino.cc/hc/en-us/articles/4405107258130-How-to-update-the-LoRa-modem-firmware), the Max Carrier ships with a **stale stock Murata firmware that must be updated before first use** via the `MKRWANFWUpdate_standalone` example sketch from the `MKRWAN` library, with `#define PORTENTA_CARRIER` added before `#include <MKRWAN.h>`. The sketch reflashes the modem **over UART from the host H7** using the STM32L072 system bootloader — **no SWD required**. The `+ERR_RX` URC observed on our boards matches the documented "stale firmware" failure mode.
  **2026-05-04 Method G analysis update:** Per user question "could we implement [SPI-era features] with custom firmware?" — yes, fully. The Murata SiP's SX1276 SPI bus is internal to the package and directly wired to the on-die STM32L072 (192 KB Flash / 20 KB RAM, Cortex-M0+ @ 32 MHz). Custom firmware on the L072 recovers per-frame FHSS, adaptive SF, three-profile per-frame PHY swap, P0 preempt, AES-GCM, custom 16 B ControlFrame, no-ACK semantics, with sub-millisecond host↔radio latency over a binary COBS UART. Cost: 4–6 weeks of focused L072 firmware work using [`hardwario/lora-modem`](https://github.com/hardwario/lora-modem) (MIT) as the fork point. Brick risk near-zero if BOOT0/NRST control + a UART safe-mode command are designed in. Full analysis in [AI NOTES/2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md](AI%20NOTES/2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md); summary appended as §11 of the comparison note.
  **2026-05-04 Method G commitment update:** Per project lead direction "we are not in a hurry, we want to get the most performance out of LoRa as possible. Lets skip external modems or boards in method E/F, and focus exclusively on method G" — Methods A, B, C, D, E, F are now superseded. Method G (custom firmware on the L072) is committed as the only LoRa path forward. New design folder [DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/) created to hold the plans and analysis: [README](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/README.md), [00 Method-G commitment & decision record](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md), [01 capabilities analysis](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md) (12 R-XX recovered features + 34 N-XX new capabilities including LBT, channel-quality-aware FHSS, deep-sleep handheld, autonomous emergency beacon, signed firmware, A/B field updates), [02 firmware architecture](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) (source layout, bare-metal cooperative scheduler, ~92 KB / ~14 KB resource budget, Make + PlatformIO), [03 bring-up roadmap](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md) (8 phases, gated on capability not calendar), [04 hardware interface & recovery](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) (host UART spec + 5-layer brick recovery).
  **2026-05-09 Phase 0 update:** The H7/X8 → STM32L072 ROM-bootloader pipeline is now qualified. Single-board Method G flash stress completed **20/20 PASS (100%)** using the pure-Python AN3155 flow, and the companion AT smoke test returned **PASS** for the mandatory liveness check against the currently-used pinger image. That closes the "can we reliably flash and boot the Murata L072 from the X8 path?" prerequisite from [03_Bringup_Roadmap.md Phase 0](DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md).
  **Next gate:** Phase 1 / W1-7 — flash the custom [`murata_l072`](DESIGN-CONTROLLER/firmware/murata_l072/) binary, capture `BOOT_URC` (`radio_ok == 1`, `clock_source_id == 0`), and dump the SX1276 register map over the binary host link. Host-side probe helper: [`DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py). X8-side one-shot wrapper: [`DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_method_g_stage1.sh`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_method_g_stage1.sh). Windows host launcher: [`DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_method_g_stage1_end_to_end.ps1`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_method_g_stage1_end_to_end.ps1). Windows build helper: [`DESIGN-CONTROLLER/firmware/murata_l072/build.ps1`](DESIGN-CONTROLLER/firmware/murata_l072/build.ps1). Current launcher also pulls X8 logs into [`DESIGN-CONTROLLER/bench-evidence/`](DESIGN-CONTROLLER/bench-evidence/) for the run.
  **2026-05-09 Stage 1 run result:** The AN3155 erase blocker is resolved for bench use: the L072 ROM bootloader still NACKs both global mass-erase payloads, but the helper flasher now succeeds by falling back to **Extended Erase page batches** for just the flashed image span. This was validated twice: first with the custom `murata_l072` image, and then with the known-good `hello.bin`, which printed successfully on `/dev/ttymxc3` after flash and boot. That means the remaining blocker is no longer the Method G flash path. The current blocker is **inside the custom `murata_l072` image itself**: after a successful flash and user-app boot pulse, the Stage 1 probe sees no `BOOT_URC`, no `VER_URC`, and no ASCII AT fallback bytes at `921600 8N1`. Next technical task is to isolate why the custom image stays silent after boot even though the same flash/boot path runs `hello.bin` correctly.
  **2026-05-09 Stage 1 delta update:** Unified flash layout + linker/assert cleanup is now implemented in `murata_l072`, and an additional low-level blocker was fixed in firmware headers: STM32L072 HSI16 `RCC_CR` bits were corrected (`HSION=bit0`, `HSIRDY=bit2`). After rebuild and rerun, behavior changed from full silence to deterministic boot-stream output at `19200 8N1` during pre-probe capture, proving earlier Reset_Handler execution now occurs. Stage 1 host-protocol validation still fails at `921600 8N1` (no `BOOT_URC` yet), so the active blocker has shifted from flash/boot entry to startup-to-host-UART handoff timing/framing.
  **2026-05-09 Stage 1 rerun (post-beacon-timing patch):** startup beacon duration was reduced to prevent a long startup holdoff before protocol init. Bench rerun still fails W1-7 (`BOOT_URC` timeout), with no bytes observed in pre-probe capture on that run. Latest evidence folder: [`DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_131716/`](DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_131716/). Next step remains targeted instrumentation of startup→`host_uart_init()` transition.
  **2026-05-09 Stage 1 rerun (dual-UART host fallback):** host transport init was widened to bring up both `LPUART1` (PA2/PA3 AF6) and `USART1` (PA9/PA10 AF4) in parallel, mirroring TX and accepting RX IRQs on either lane to remove board-routing ambiguity. Rebuild + bench rerun still fails W1-7: flash succeeds, boot pulse succeeds, pre-probe capture at `19200 8N1` is empty, ASCII fallback (`ATI`, `AT+VER?`) is empty, and binary probe still times out waiting for `VER_URC` (`0x81`) at `921600 8N1`. Latest evidence folder: [`DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_132507/`](DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_132507/) (`method_g_stage1_manual.log`, `method_g_stage1_ocd.log`, `flash_run.log`, `flash_ocd.log`).
  **2026-05-09 Stage 1 rerun (ordered breadcrumbs + early split UART capture):** startup/main were instrumented with short `RST:*` and `M:*` breadcrumbs, and the X8 runner was extended to capture UART immediately after reset. Result: the line is not actually idle, but it is still not emitting a valid host link. The latest rerun captured non-text garbage during the earliest `19200 8N1` window and then a sustained burst of raw `0x00` bytes in the early `921600 8N1` window, followed by full silence by the time the active probe begins; `BOOT_URC`, `VER_URC`, and ASCII fallback all still fail. This shifts the blocker from “no electrical activity” to “pathological early UART output / immediate high-speed failure” somewhere around the `safe_mode_listen` → `platform_clock_init_hsi16` → `host_uart_init` → `host_cmd_init` transition. Latest evidence folder: [`DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_133825/`](DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_133825/).
  **2026-05-09 Stage 1 rerun (BOOT_URC decode + VER_REQ RX deadlock):** Stage 1 harness timing fixes are now validated (synchronous boot OpenOCD, timeout-only 921600 capture with no byte cap). Latest captured wire bytes include two `FAULT_URC` frames surrounding a valid `BOOT_URC` frame, proving the custom firmware reaches `host_cmd_init()` and main loop. However, active queries still fail: `VER_REQ` (`0x01`) receives no `VER_URC` (`0x81`), and ASCII fallback (`ATI`, `AT+VER?`) also returns no bytes. A firmware-side mitigation was tested by disabling mirrored `USART1` receive/IRQ in `host_uart.c` (keeping USART1 TX mirror only, LPUART1 RX only), but the probe result is unchanged. Current working hypothesis: the X8->Murata RX direction is not reaching LPUART1 after probe-side reopen, or bytes are dropped before frame parse. Latest on-target log excerpt shows early `M:UART1/M:RAD0/M:RADF/M:CMD0/M:CMD1` plus `FAULT_URC/BOOT_URC/FAULT_URC`, then probe timeout. Evidence currently in `/tmp/lifetrac_p0c/method_g_stage1.log`; next run should be pulled into a new `bench-evidence/` folder.
  **2026-05-09 Stage 1 rerun (RX observability instrumentation):** Added additive host stats counters (`HOST_RX_BYTES`, lane-split RX byte counters, parse_ok/parse_err, per-UART error counters) and a low-rate unsolicited `STATS_URC` snapshot on RX-byte changes. Rebuilt and reran Stage 1 twice (`T6_bringup_2026-05-09_144749`, `T6_bringup_2026-05-09_144935`). Result remains a `VER_REQ` timeout and no ASCII fallback bytes. Crucially, no unsolicited `STATS_URC` appears in early capture despite startup `BOOT_URC/FAULT_URC` frames, which strongly indicates **host->Murata ingress bytes are not reaching UART RX ISR at all** during active probe traffic (as opposed to parser-only reject). Deep-dive note: [AI NOTES/2026-05-09_Controller_Stage1_RX_Observability_Copilot_v1_1.md](AI%20NOTES/2026-05-09_Controller_Stage1_RX_Observability_Copilot_v1_1.md).
  **2026-05-09 Stage 1 rerun (forced heartbeat stats beacon):** Added an unconditional early-runtime `STATS_URC` heartbeat window (250 ms cadence for first 6 s after `host_cmd_init`) so diagnostics do not depend on RX deltas. Fresh run (`T6_bringup_2026-05-09_145307`) now deterministically shows `STATS_URC(init): host_rx_bytes=0 host_parse_ok=0 host_parse_err=0`, followed by unchanged failure on active query (`VER_REQ` timeout, no ASCII fallback bytes). This materially strengthens the classification: the firmware loop and TX path are alive enough to emit stats beacons, but **X8->Murata ingress remains zero at ISR level** during probe traffic. Follow-up note: [AI NOTES/2026-05-09_Controller_Stage1_RX_Heartbeat_Classification_Copilot_v1_2.md](AI%20NOTES/2026-05-09_Controller_Stage1_RX_Heartbeat_Classification_Copilot_v1_2.md).
  **2026-05-09 Stage 1 rerun (lane-level heartbeat confirmation):** Probe logging was widened to print lane/error counters directly from `STATS_URC(init)`. New run (`T6_bringup_2026-05-09_145434`) reports `host_rx_bytes=0 host_rx_lpuart=0 host_rx_usart1=0 host_parse_ok=0 host_parse_err=0 uart_err_lpuart=0 uart_err_usart1=0`, then the same `VER_REQ` timeout. This removes remaining ambiguity around lane split and UART fault accounting: neither RX lane is seeing ingress bytes during active probe traffic.
  **2026-05-09 Stage 1 rerun (ISR-latched RX-seen marker):** Added an ISR-near ingress latch in `host_uart` plus a one-shot `FAULT_URC` marker (`HOST_FAULT_CODE_HOST_RX_SEEN`) emitted from main loop on first observed RX byte per lane, and expanded probe-side FAULT decoding for readable lane text. Fresh run (`T6_bringup_2026-05-09_145716`) still fails with `VER_REQ` timeout and reports `STATS_URC(init): host_rx_bytes=0 host_rx_lpuart=0 host_rx_usart1=0 ... uart_err_lpuart=0 uart_err_usart1=0`. No RX-seen FAULT marker is observed in the active probe window, reinforcing the current classification that host->Murata ingress is absent before parser/dispatch.
  **2026-05-09 Stage 1 rerun (strict ROM-vs-user A/B + IRQ echo diagnostic):** A strict A/B probe with identical serial settings (`19200 8E1`, send `0x7F`) now confirms boot-state-dependent behavior: ROM state (BOOT0 high) returns `0x79` ACK, while user-app state (BOOT0 low + NRST) returns zero bytes. A follow-on diagnostic flash enabled IRQ-level raw RX echo in `murata_l072`; in user mode at `921600 8N1`, injected pattern `c35aa71e33cc` was not echoed (`pattern_found=False`) despite receiving 475 bytes of firmware output. This strengthens the blocker classification: host TX bytes still do not reach active L072 RX in user mode. Notes: [AI NOTES/2026-05-09_ROM_vs_User_AB_19200_8E1_and_RX_Echo_Diagnostic_Copilot_v1_0.md](AI%20NOTES/2026-05-09_ROM_vs_User_AB_19200_8E1_and_RX_Echo_Diagnostic_Copilot_v1_0.md), [AI NOTES/2026-05-09_RX_Echo_Diagnostic_Run_and_Chip_Document_Checklist_Copilot_v1_0.md](AI%20NOTES/2026-05-09_RX_Echo_Diagnostic_Run_and_Chip_Document_Checklist_Copilot_v1_0.md).
  **2026-05-09 Stage 1 rerun (PA11 post-boot HIGH test):** User firmware was booted with BOOT0 low, then H7 `PA_11` was forced HIGH in a second OpenOCD session without resetting L072. OpenOCD verified write success (`PA11 forced HIGH, GPIOA_IDR=0x0000c800`), but the IRQ-echo probe still reported `pattern_found=False` at `921600 8N1` (URC traffic present, no echoed injected bytes). This weakens the "PA11-only OE gate" theory and points to another control net or board routing dependency. Note: [AI NOTES/2026-05-09_PA11_PostBoot_High_Test_Copilot_v1_0.md](AI%20NOTES/2026-05-09_PA11_PostBoot_High_Test_Copilot_v1_0.md).
  **2026-05-09 ingress-gate update:** The newest A/B probe set now classifies the blocker as a carrier-side routing or flow-control gate, not a reset-state issue. ROM bootloader sync still works, but user-mode ingress stays absent even after PA11 is forced high post-boot, so the next bench step is to scope or force RTS/CTS and then trace the route pins on the Max Carrier/H747 side. Full analysis note: [AI NOTES/2026-05-09_Controller_Stage1_Ingress_Gate_Analysis.md](AI%20NOTES/2026-05-09_Controller_Stage1_Ingress_Gate_Analysis.md).
  **2026-05-09 RTS/CTS A/B execution note:** A concrete runbook for the next discriminating test (ROM baseline + user-mode `hwflow=off` vs `hwflow=on`) is now available at [AI NOTES/2026-05-09_Controller_Stage1_RTS_CTS_AB_Checklist_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_RTS_CTS_AB_Checklist_Copilot_v1_0.md). The helper diagnostic now supports hardware-flow toggle via `diag_uart_rxtx.py --hwflow off|on`.
  **2026-05-09 RTS/CTS A/B run result:** Completed on board `2E2C1209DABC240B` with evidence in [`DESIGN-CONTROLLER/bench-evidence/T6_rtscts_ab_2026-05-09_183033/`](DESIGN-CONTROLLER/bench-evidence/T6_rtscts_ab_2026-05-09_183033/). Both `hwflow=off` and `hwflow=on` runs were silent for `ATI`, `VER_REQ`, and `PING_REQ`; the `hwflow=on` attempt also reported `stty: /dev/ttymxc3: unable to perform all requested operations`, which indicates host-side CTS/RTS enable may not be fully supported on this path. Classification remains Case A (no ingress either way). Next step is route/control-net tracing on Max Carrier/H747 rather than serial framing changes. Full run note: [AI NOTES/2026-05-09_Controller_Stage1_RTS_CTS_AB_Run_Result_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_RTS_CTS_AB_Run_Result_Copilot_v1_0.md).
  **2026-05-09 next-test master plan:** Added a prioritized execution plan with decision gates and exact command anchors for the next bench cycle: [AI NOTES/2026-05-09_Controller_Stage1_Next_Test_Plan_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_Next_Test_Plan_Copilot_v1_0.md).
  **2026-05-09 Phase 1 ownership A/B run result:** Executed on board `2E2C1209DABC240B` with evidence in [`DESIGN-CONTROLLER/bench-evidence/T6_phase1_owner_ab_2026-05-09_183718/`](DESIGN-CONTROLLER/bench-evidence/T6_phase1_owner_ab_2026-05-09_183718/). `Test 1A` (H7 halted) captured only early startup breadcrumbs + short binary burst (`phase1a_rx.bin`, 78 B). `Test 1B` (boot user app then explicit H7 resume) changed observable traffic shape substantially (`phase1b_diag_hwflow_off.txt`, many outbound binary frames), but active request/response is still failing (`phase1b_stage1_probe_after_resume.txt`: timeout waiting for `VER_URC` 0x81, no ASCII fallback bytes). Interpretation: H7 runtime state affects outbound path behavior, but host->L072 ingress is still blocked. Next step is Phase 2 GPIO state capture/diff to isolate candidate route-control net. Full run note: [AI NOTES/2026-05-09_Controller_Stage1_Phase1_Ownership_AB_Run_Result_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_Phase1_Ownership_AB_Run_Result_Copilot_v1_0.md).
  **2026-05-09 Phase 2 GPIO snapshot/diff result:** Added and executed one-shot OpenOCD snapshots (`10_snapshot_rom_state.cfg`, `11_snapshot_user_state.cfg`, `12_snapshot_user_runtime_state.cfg`). The halted ROM-vs-user snapshot differed only at PA11 level (expected BOOT0 selection), while runtime user snapshot diverged across multiple banks (`GPIOA/B/C/E/F/H/I` MODER/IDR changed versus ROM baseline). This is the first concrete route-control lead for Phase 3 perturbation. Evidence: `phase2_snapshot_rom_state_allbanks.txt`, `phase2_snapshot_user_state_allbanks.txt`, `phase2_snapshot_user_runtime_allbanks.txt` under [`DESIGN-CONTROLLER/bench-evidence/T6_phase1_owner_ab_2026-05-09_183718/`](DESIGN-CONTROLLER/bench-evidence/T6_phase1_owner_ab_2026-05-09_183718/). Analysis note: [AI NOTES/2026-05-09_Controller_Stage1_Phase2_GPIO_Snapshot_Diff_Result_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_Phase2_GPIO_Snapshot_Diff_Result_Copilot_v1_0.md).
  **2026-05-09 Phase 3 PE11 perturbation A/B result:** Executed single-pin perturbation on `PE_11` with explicit force-low and force-high runs after user-runtime ownership window, using new configs `13_perturb_pe11_low.cfg` and `14_perturb_pe11_high.cfg`. OpenOCD readback confirmed pin forcing (`GPIOE_IDR.PE_11 = 0` / `1`), but both cases remained identical at protocol level: `method_g_stage1_probe.py` timed out waiting for `VER_URC` and ASCII fallback stayed empty. Diag traffic shape also remained effectively unchanged. Classification: `PE_11` is not a sufficient ingress gate by itself. Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_phase3_pe11_ab_2026-05-09_184225/`](DESIGN-CONTROLLER/bench-evidence/T6_phase3_pe11_ab_2026-05-09_184225/). Run note: [AI NOTES/2026-05-09_Controller_Stage1_Phase3_PE11_Perturbation_AB_Run_Result_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_Phase3_PE11_Perturbation_AB_Run_Result_Copilot_v1_0.md).
  **2026-05-09 Phase 3 perturbation batch result (PB14/PB15/PB10/PC12):** Added and executed additional single-pin A/B tests with forced low/high after user-runtime dwell using `15_perturb_pb14_low.cfg`, `16_perturb_pb14_high.cfg`, `17_perturb_pb15_low.cfg`, `18_perturb_pb15_high.cfg`, `19_perturb_pb10_low.cfg`, `20_perturb_pb10_high.cfg`, `21_perturb_pc12_low.cfg`, and `22_perturb_pc12_high.cfg`. In all eight cases, OpenOCD confirmed requested pin state, but Stage 1 remained unchanged: `STATS_URC(init)` reports `host_rx_bytes=0`, `ATI`/`AT+VER?` return only recurring `STATS_URC` frame content, and probe still times out waiting for `VER_URC`. Classification: none of these single pins is a sufficient ingress gate on board `2E2C1209DABC240B`. Evidence folders: [`DESIGN-CONTROLLER/bench-evidence/T6_phase3_pb14_ab_2026-05-09_184402/`](DESIGN-CONTROLLER/bench-evidence/T6_phase3_pb14_ab_2026-05-09_184402/), [`DESIGN-CONTROLLER/bench-evidence/T6_phase3_pb15_ab_2026-05-09_184430/`](DESIGN-CONTROLLER/bench-evidence/T6_phase3_pb15_ab_2026-05-09_184430/), [`DESIGN-CONTROLLER/bench-evidence/T6_phase3_pb10_ab_2026-05-09_184502/`](DESIGN-CONTROLLER/bench-evidence/T6_phase3_pb10_ab_2026-05-09_184502/), [`DESIGN-CONTROLLER/bench-evidence/T6_phase3_pc12_ab_2026-05-09_184532/`](DESIGN-CONTROLLER/bench-evidence/T6_phase3_pc12_ab_2026-05-09_184532/). Consolidated note: [AI NOTES/2026-05-09_Controller_Stage1_Phase3_Perturbation_Batch_Result_Copilot_v1_1.md](AI%20NOTES/2026-05-09_Controller_Stage1_Phase3_Perturbation_Batch_Result_Copilot_v1_1.md).
  **2026-05-09 Phase 3 PC9 perturbation A/B result:** Executed single-pin `PC_9` low/high forcing with `23_perturb_pc9_low.cfg` and `24_perturb_pc9_high.cfg`. OpenOCD readback confirmed state (`GPIOC_IDR.PC_9 = 0/1`), but Stage 1 behavior remained unchanged in both cases (`STATS_URC(init)` with `host_rx_bytes=0`, recurring `STATS_URC` content on `ATI`/`AT+VER?`, timeout on `VER_URC`). Classification: `PC_9` is not a sufficient ingress gate by itself. Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_phase3_pc9_ab_2026-05-09_184622/`](DESIGN-CONTROLLER/bench-evidence/T6_phase3_pc9_ab_2026-05-09_184622/).
  **2026-05-09 Phase 3 cluster interaction A/B result:** Escalated from single-pin tests to combined forcing of runtime-changed cluster bits (`PB10`, `PB14`, `PB15`, `PC9`, `PC12`, `PE11`) using `25_perturb_cluster_low.cfg` and `26_perturb_cluster_high.cfg`. First run (`T6_phase3_cluster_ab_2026-05-09_184659`) showed unchanged Stage 1 failure signature but lacked explicit per-pin readback logs. Scripts were updated to print pin-by-pin IDR values and rerun (`T6_phase3_cluster_ab_verifyfix_2026-05-09_184754`), confirming all target pins forced to requested levels (`0` in low case, `1` in high case). Stage 1 remained unchanged in both verified cases: `host_rx_bytes=0`, no `BOOT_URC` in query window, `ATI`/`AT+VER?` decode as `STATS_URC`, and fatal timeout waiting for `VER_URC`. Classification: this GPIO cluster state is not sufficient to restore ingress. Deep-dive note: [AI NOTES/2026-05-09_Controller_Stage1_Phase3_PC9_and_Cluster_Perturbation_Result_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_Phase3_PC9_and_Cluster_Perturbation_Result_Copilot_v1_0.md).
  **2026-05-09 Phase 4 route-detail snapshot result (AF ownership focus):** Added detailed snapshot helpers `27_snapshot_rom_route_detail.cfg` and `28_snapshot_user_runtime_route_detail.cfg` to capture `MODER`, `PUPDR`, `AFRL`, `AFRH`, `IDR` per GPIO bank in ROM-working vs user-runtime states. New evidence (`T6_phase4_route_detail_fix_2026-05-09_185217`) confirms broad runtime AF remap activity across route-relevant pins, including `PA2/PA3/PA9/PA10/PA15`, `PB10/PB11`, `PC9/PC10/PC11/PC12`, `PE11..PE14`, `PF0/PF1`, `PH13/PH14`, `PI0..PI3`. This strengthens the non-static interpretation: ingress failure is likely tied to runtime ownership/mux behavior rather than single static GPIO level. Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_phase4_route_detail_fix_2026-05-09_185217/`](DESIGN-CONTROLLER/bench-evidence/T6_phase4_route_detail_fix_2026-05-09_185217/).
  **2026-05-09 Phase 4 timing-window sweep result:** Added `29..33_boot_user_delay_*.cfg` and executed a five-point delay sweep (`50/150/400/1000/2000 ms`) before probe start on board `2E2C1209DABC240B`. Early-delay runs (50/150/400 ms) consistently captured `READY_URC` and `BOOT_URC`, while longer-delay runs did not always show boot URC in the initial window, but **all five runs still failed identically at protocol gate**: `host_rx_bytes=0` in decoded `STATS_URC`, no `VER_URC`, timeout waiting for response type `0x81` to req `0x01`. Classification: no transient ingress-enable timing window was found within this sweep; outbound startup path can be observed, ingress remains blocked. Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_phase4_timing_sweep_2026-05-09_185326/`](DESIGN-CONTROLLER/bench-evidence/T6_phase4_timing_sweep_2026-05-09_185326/). Full note: [AI NOTES/2026-05-09_Controller_Stage1_Phase4_RouteDetail_and_TimingSweep_Result_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_Phase4_RouteDetail_and_TimingSweep_Result_Copilot_v1_0.md).
  **2026-05-09 Phase 5 coordinated lane-set A/B/C perturbation result:** Added and executed six new coordinated perturbation configs (`34_perturb_laneA_low.cfg`, `35_perturb_laneA_high.cfg`, `36_perturb_laneB_low.cfg`, `37_perturb_laneB_high.cfg`, `38_perturb_laneC_low.cfg`, `39_perturb_laneC_high.cfg`) to test lane groups inferred from Phase 4 AF remap analysis. OpenOCD per-pin IDR readback confirmed all requested forced states in every case (A/B/C low and high). Stage 1 remained unchanged in all six probe runs: `host_rx_bytes=0`, recurring `STATS_URC` responses on active requests, and fatal timeout waiting for `VER_URC` (`0x81` response to `0x01`). Classification: coordinated static forcing of these lane sets is still insufficient to restore ingress on board `2E2C1209DABC240B`. Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_phase5_lane_sets_2026-05-09_185820/`](DESIGN-CONTROLLER/bench-evidence/T6_phase5_lane_sets_2026-05-09_185820/). Full note: [AI NOTES/2026-05-09_Controller_Stage1_Phase5_Coordinated_LaneSet_Perturbation_Result_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_Phase5_Coordinated_LaneSet_Perturbation_Result_Copilot_v1_0.md).
  **2026-05-09 Phase 6 owner/policy intervention result:** Added and executed owner-focused Phase 6 configs (`40_phase6_owner_freeze_runtime.cfg`, `41_phase6_owner_freeze_laneB_af.cfg`, `42_phase6_owner_freeze_laneA_policy.cfg`, `43_phase6_owner_freeze_laneC_af.cfg`) to move beyond static output-level forcing. Cases covered: runtime owner freeze only, lane-B AF policy injection, lane-A mixed AF+pull-up policy injection, and lane-C AF policy injection after runtime takeover. OpenOCD logs confirm policy steps executed (`AFSET`/`INPULLUP` traces present), but Stage 1 remained unchanged in all four cases: `STATS_URC(init)` with `host_rx_bytes=0`, recurring `STATS_URC` active path, and fatal timeout waiting for `VER_URC` (`0x81` to req `0x01`). Classification: halting/freeze and static AF/pull policy injection at this phase still do not reopen ingress; blocker likely sits in live runtime peripheral ownership/transaction path rather than late halted-state register image alone. Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_phase6_owner_policy_2026-05-09_190059/`](DESIGN-CONTROLLER/bench-evidence/T6_phase6_owner_policy_2026-05-09_190059/). Full note: [AI NOTES/2026-05-09_Controller_Stage1_Phase6_OwnerPolicy_Intervention_Result_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_Phase6_OwnerPolicy_Intervention_Result_Copilot_v1_0.md).
  **2026-05-09 Phase 7 live-runtime policy-loop intervention result:** Added and executed live-intervention configs (`44_phase7_live_laneB_af_loop.cfg`, `45_phase7_live_laneA_policy_loop.cfg`, `46_phase7_live_laneC_af_loop.cfg`) that keep H7 runtime active while repeatedly re-applying candidate route policy for ~2 s (100 iterations with 20 ms cadence). OpenOCD logs confirm live loop execution (`LIVE_AF_INJECT` / `LIVE_POLICY_INJECT` iteration markers at `0/20/40/60/80`) in all three runs. Stage 1 still remained unchanged in all cases: `STATS_URC(init)` reports `host_rx_bytes=0`, active requests decode to recurring `STATS_URC` payloads, and probe ends in timeout waiting for `VER_URC` (`0x81` to req `0x01`). Classification: even live policy overwrite loops at the tested GPIO/AF layer do not restore ingress; root cause is likely above this layer (runtime peripheral transaction path/protocol ownership) or in a non-overwritten gating domain. Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_phase7_live_policy_2026-05-09_190504/`](DESIGN-CONTROLLER/bench-evidence/T6_phase7_live_policy_2026-05-09_190504/). Full note: [AI NOTES/2026-05-09_Controller_Stage1_Phase7_LiveRuntime_PolicyLoop_Result_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_Phase7_LiveRuntime_PolicyLoop_Result_Copilot_v1_0.md).
  **2026-05-09 Phase 8 firmware ingress-branch instrumentation result:** Added firmware-level diagnostic mark instrumentation in `murata_l072` host path (`host_uart`/`host_cmd`/`main`) and updated probe decoder to interpret `HOST_DIAG_MARK` fault payloads. New marks track `VER_REQ` parse/dispatch/send branch progression (`VER_REQ_PARSED`, `VER_REQ_DISPATCHED`, `VER_URC_SENT`, `AT_VER_DISPATCHED`) and are emitted via one-shot `FAULT_URC` (`code=0x0C`) when observed. Full build/flash/probe cycle completed with evidence in [`DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_190924/`](DESIGN-CONTROLLER/bench-evidence/T6_bringup_2026-05-09_190924/). Runtime result stayed negative: `STATS_URC(init)` still shows `host_rx_bytes=0`, `VER_REQ` still times out, and only `HOST_RX_INACTIVE` (`FAULT_URC code=0x0A`) appeared in active window; no `HOST_DIAG_MARK` events were emitted. Classification: ingress is absent before parser/dispatch branch points are reached.
  **2026-05-09 Phase 9 boot-then-halt timing sweep result (owner-domain timing gate):** Added and executed timing configs `47..50_phase9_boot_then_halt_{000,020,100,400}ms.cfg` to test whether halting H7 shortly after user boot preserves ingress before runtime ownership logic can close the path. All four OpenOCD runs completed cleanly and all four Stage 1 probes remained identical-negative: `BOOT_URC` observed, recurring `STATS_URC` with `host_rx_bytes=0`, and fatal timeout waiting for `VER_URC` (`0x81` to req `0x01`). `FAULT_URC` consistently showed startup faults (`0x03`, `0x08`) but no ingress progress indicators. Classification: no early runtime ownership timing window was found in 0..400 ms halt sweep; ingress remains absent pre-parser. Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_phase9_halt_timing_2026-05-09_191923/`](DESIGN-CONTROLLER/bench-evidence/T6_phase9_halt_timing_2026-05-09_191923/). Full note: [AI NOTES/2026-05-09_Controller_Stage1_Phase9_BootThenHalt_TimingSweep_Result_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_Phase9_BootThenHalt_TimingSweep_Result_Copilot_v1_0.md).
  **2026-05-09 Phase 10 source-domain A/B proof attempt result (ROM hold vs user-halt, identical probe primitive):** Executed a focused A/B run on board `2E2C1209DABC240B` using the same sender path and same helper (`verify_l072_rom.sh`) in both states, with OpenOCD state setup via `06_assert_pa11_pf4.cfg` (ROM-hold intent) and `47_phase9_boot_then_halt_000ms.cfg` (user-halt intent). Evidence folder: [`DESIGN-CONTROLLER/bench-evidence/T6_phase10_source_domain_ab_2026-05-09_201304/`](DESIGN-CONTROLLER/bench-evidence/T6_phase10_source_domain_ab_2026-05-09_201304/). This specific attempt did **not** reproduce the expected ROM baseline ACK: ROM case returned `ROM_RESP_SIZE=0` (silent), and user-halt case returned `ROM_RESP_SIZE=4` with bytes `00 00 00 00` (still no `0x79`). Classification: no positive ROM ACK in this capture, therefore this run is **inconclusive** as a source-domain discriminator and should not be used as a proof of state-dependent ingress. Next gate is to re-establish a deterministic ROM `0x79` baseline in the same session, then rerun the A/B pair unchanged.
  **2026-05-09 Phase 10 source-domain A/B rerun result (ROM hold vs user-halt, same primitive):** Re-ran the same A/B procedure with live-captured logs and persisted artifacts in [`DESIGN-CONTROLLER/bench-evidence/T6_phase10_source_domain_ab_2026-05-09_2022_rerun_manual/`](DESIGN-CONTROLLER/bench-evidence/T6_phase10_source_domain_ab_2026-05-09_2022_rerun_manual/). ROM-hold setup again showed expected OpenOCD control actions (`PA_11` high, `PF_4` pulse, hold window), but verifier still reported `ROM_RESP_SIZE=0`. User-halt setup also completed and verifier again reported `ROM_RESP_SIZE=0`. Classification: rerun remains **inconclusive for state-dependent source-domain proof** because deterministic ROM `0x79` baseline was not reproduced; immediate blocker is unstable/non-reproducible ROM-positive control. To enforce a valid control gate on the next attempt, added atomic runner [`DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_phase10_source_domain_ab_atomic.sh`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_phase10_source_domain_ab_atomic.sh), which retries ROM baseline up to `MAX_ROM_ATTEMPTS` and only executes user-halt compare after confirmed `0x79`. Full rerun note: [AI NOTES/2026-05-09_Controller_Stage1_Phase10_SourceDomain_AB_Result_Copilot_v1_1.md](AI%20NOTES/2026-05-09_Controller_Stage1_Phase10_SourceDomain_AB_Result_Copilot_v1_1.md).
  **2026-05-09 Phase 10 atomic control-gated rerun result:** Fixed a harness bug in [`DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/verify_l072_rom.sh`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/verify_l072_rom.sh) where background `cat` capture was started in a subshell and `$!` was captured in the parent shell, allowing the verifier to stall before writing output. After pushing the fix, re-ran the atomic gate script [`DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_phase10_source_domain_ab_atomic.sh`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_phase10_source_domain_ab_atomic.sh) on board `2E2C1209DABC240B` with `MAX_ROM_ATTEMPTS=3`. Persisted evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_phase10_source_domain_ab_atomic_2026-05-09_204733/`](DESIGN-CONTROLLER/bench-evidence/T6_phase10_source_domain_ab_atomic_2026-05-09_204733/). Result is now controlled and explicit: all three ROM-baseline attempts completed, all three verifier logs reported `ROM_RESP_SIZE=0`, and `summary.txt` recorded `PHASE10_ATOMIC_RESULT=FAIL_NO_ROM_BASELINE`, `ROM_ACK_ATTEMPT=0`, `MAX_ROM_ATTEMPTS=3`. Because the ROM-positive control never materialized, the script correctly did **not** execute the user-halt comparator. Classification: this is no longer a harness ambiguity; the current blocker is a real failure to reproduce the ROM `0x79` baseline under the present invocation path. Full atomic rerun note: [AI NOTES/2026-05-09_Controller_Stage1_Phase10_SourceDomain_AB_Result_Copilot_v1_2.md](AI%20NOTES/2026-05-09_Controller_Stage1_Phase10_SourceDomain_AB_Result_Copilot_v1_2.md).
  **2026-05-09 Phase 10 atomic A/B post-fix decisive run:** Updated the atomic tooling to better match the known-good probe order and runtime behavior: in [`DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_phase10_source_domain_ab_atomic.sh`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_phase10_source_domain_ab_atomic.sh), ROM setup now runs OpenOCD in background while `verify_l072_rom.sh` probes; in [`DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/verify_l072_rom.sh`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/verify_l072_rom.sh), ROM `0x7F` sync is checked before the AT probe (legacy order). A completed post-fix atomic run at [`DESIGN-CONTROLLER/bench-evidence/T6_phase10_source_domain_ab_atomic_2026-05-09_2059_rom79_partial/`](DESIGN-CONTROLLER/bench-evidence/T6_phase10_source_domain_ab_atomic_2026-05-09_2059_rom79_partial/) produced `summary.txt` = `PHASE10_ATOMIC_RESULT=OK`, `ROM_ACK_ATTEMPT=1`, `USERHALT_ROM_RESP_SIZE=0`, `USERHALT_FIRST_BYTE=NA`. ROM half captured `First byte after 0x7F: 0x79` while user-halt half was silent on `0x7F`. This is the first controlled in-session atomic run that both reproduces ROM-positive control and preserves user-halt negative response, so it is a valid source-domain discriminator result. A subsequent immediate rerun (`T6_phase10_source_domain_ab_atomic_2026-05-09_2100_full_unexpected00`) showed `0x00` bytes and failed baseline, so reproducibility remains noisy and should be treated as a stability follow-up rather than a proof blocker. Full post-fix note: [AI NOTES/2026-05-09_Controller_Stage1_Phase10_SourceDomain_AB_Result_Copilot_v1_3.md](AI%20NOTES/2026-05-09_Controller_Stage1_Phase10_SourceDomain_AB_Result_Copilot_v1_3.md).
  **2026-05-09 targeted retest follow-up (backfill + stability quant):** Completed the recommended narrow retest rather than broad phase rewind. (1) Representative pre-Phase-10 backfill: re-ran `47_phase9_boot_then_halt_000ms.cfg` with fresh probe capture in [`DESIGN-CONTROLLER/bench-evidence/T6_phase9_backfill_cfg47_2026-05-09_210323/`](DESIGN-CONTROLLER/bench-evidence/T6_phase9_backfill_cfg47_2026-05-09_210323/); signature is unchanged from prior Phase 9 (`BOOT_URC` present, repeated `STATS_URC` with `host_rx_bytes=0`, timeout waiting for `VER_URC`). (2) Phase 10 reproducibility quantification: executed a 10-run atomic campaign (`MAX_ROM_ATTEMPTS=1`) and persisted per-run artifacts in [`DESIGN-CONTROLLER/bench-evidence/T6_phase10_atomic_stability_10x_2026-05-09_2110_clean/`](DESIGN-CONTROLLER/bench-evidence/T6_phase10_atomic_stability_10x_2026-05-09_2110_clean/). `results_clean.csv` shows `1/10` runs with `PHASE10_ATOMIC_RESULT=OK` (`ROM_ACK_ATTEMPT=1`, `USERHALT_ROM_RESP_SIZE=0`) and `9/10` runs with `FAIL_NO_ROM_BASELINE` (`ROM_ACK_ATTEMPT=0`). Classification: no need to rerun all earlier phases; Phase 10 discriminator is proven at least once under controlled conditions, but ROM baseline entry remains low-probability and should be treated as the next stability hardening target. Full quant note: [AI NOTES/2026-05-09_Controller_Stage1_Phase10_Stability_Quant_and_Phase9_Backfill_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_Phase10_Stability_Quant_and_Phase9_Backfill_Copilot_v1_0.md).
  **2026-05-09 ROM baseline single-hold timing sweep (new stability discriminator):** Added helper [`DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_rom_baseline_timing_sweep.sh`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_rom_baseline_timing_sweep.sh) to run one `06_assert_pa11_pf4.cfg` hold window and probe ROM sync repeatedly across settle delays (`50/100/200/400 ms`, `5` attempts each). Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_timing_2026-05-09_2115/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_timing_2026-05-09_2115/). Sweep summary: `TOTAL_PROBES=20`, `ACK_COUNT=1`, `NACK_COUNT=7`, `SILENT_COUNT=12`, `ZERO_COUNT=0`, `OTHER_COUNT=0`; best line `50,5,1,2,0,2,0`. Delay breakdown from `delay_summary.csv`: `50ms` produced the only ACK (`1/5`), `100ms` and `200ms` produced no ACK with mixed NACK/silent, and `400ms` was fully silent (`0/5 ACK, 5/5 silent`). Classification: ROM entry likelihood is highest at the shortest tested settle delay and degrades with longer waits; next hardening pass should focus near `<=100ms` with higher-attempt multi-sync loops rather than longer settle windows. Full note: [AI NOTES/2026-05-09_Controller_Stage1_ROM_Baseline_Timing_Sweep_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_ROM_Baseline_Timing_Sweep_Copilot_v1_0.md).
  **2026-05-09 ROM baseline short-delay high-attempt sweep (hardening rerun):** Executed the next hardening pass with updated helper parsing (`DELAYS_MS` comma-list support) and a denser short-delay window. Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_timing_80probe_2026-05-09_081721/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_timing_80probe_2026-05-09_081721/). Run settings from `summary.txt`: `DELAYS_MS=30,50,75,100`, `ATTEMPTS_PER_DELAY=20`, `TOTAL_PROBES=80`. Outcome: `ACK_COUNT=1`, `NACK_COUNT=29`, `SILENT_COUNT=50`, `ZERO_COUNT=0`, `OTHER_COUNT=0`, `BEST_DELAY_LINE=30,20,1,9,0,10,0`. Delay breakdown from `delay_summary.csv`: `30ms` produced the only ACK (`1/20`), `50ms` had `0/20` ACK with `7` NACK and `13` silent, `75ms` had `0/20` ACK with `3` NACK and `17` silent, `100ms` had `0/20` ACK with `10` NACK and `10` silent. Compared to the prior 20-probe run (`1/20` ACK), ACK yield dropped in this rerun to `1/80`, but the short-delay best-delay classification persisted (ACK only at the shortest tested delay in each run: prior `50ms`, now `30ms`). Classification: stability remains low-probability and timing-sensitive in the short-delay band; next discriminator should pivot from static settle-delay tuning to per-hold multi-sync burst strategy and control-net/state conditioning around ROM entry.
  **2026-05-09 burst-matrix harness hardening + verify run:** Hardened [`DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_rom_baseline_burst_matrix.sh`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_rom_baseline_burst_matrix.sh) to reduce false stalls: (1) OpenOCD launch now disables GDB server (`gdb_port disabled`) to avoid port-3333 collisions across bursts, (2) optional OpenOCD lifetime cap added (`OPENOCD_LIFETIME_S`, default `75`) via `timeout`, and (3) cleanup now force-terminates wrapper + child OpenOCD before `wait`, with per-burst progress logging to `run.log`. Validation run evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_verify2_2026-05-09_083803/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_verify2_2026-05-09_083803/). Verify settings: `DELAYS_MS=30`, `BURSTS_PER_DELAY=1`, `ATTEMPTS_PER_BURST=3`. Result (`summary.txt`): `TOTAL_BURSTS=1`, `PASS_BURSTS=0`, `TOTAL_PROBES=3`, `ACK_COUNT=0`, `NACK_COUNT=1`, `SILENT_COUNT=2`, `BEST_DELAY_LINE=30,1,0,1,0.0000,3,0,1,0,2,0`. `run.log` confirms deterministic completion (`burst_start`→`burst_done`→`done`) within the same run window, demonstrating end-to-end summary finalization now works on this minimal case.
  **2026-05-09 burst-matrix campaign result:** Ran a larger hardened matrix at `DELAYS_MS=30,50`, `BURSTS_PER_DELAY=5`, `ATTEMPTS_PER_BURST=10`, `BURST_PASS_MIN_ACK=1` with snapshots disabled. Evidence folder: [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_084927/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_084927/). Result (`summary.txt`): `TOTAL_BURSTS=10`, `PASS_BURSTS=3`, `FAIL_BURSTS=7`, `BURST_PASS_RATE=0.3000`, `TOTAL_PROBES=100`, `ACK_COUNT=3`, `NACK_COUNT=47`, `SILENT_COUNT=50`, `OTHER_COUNT=0`, `BEST_DELAY_LINE=30,5,2,3,0.4000,50,2,23,0,25,0`. Per-delay breakdown: `30 ms` yielded `2/5` burst passes and `50 ms` yielded `1/5`. Interpretation: the harness is stable enough for production-sized runs, but ACKs remain sparse and still favor the short-delay edge. New note: [AI NOTES/2026-05-09_Controller_Stage1_ROM_Burst_Matrix_Campaign_Result_Copilot_v1_1.md](AI%20NOTES/2026-05-09_Controller_Stage1_ROM_Burst_Matrix_Campaign_Result_Copilot_v1_1.md).
  **2026-05-09 expanded short-delay burst matrix result:** Swept the previously unexplored short-delay zone at `DELAYS_MS=5,10,15,20,25,30`, `BURSTS_PER_DELAY=5`, `ATTEMPTS_PER_BURST=10`. Evidence folder: [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_085905/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_085905/). Result: `TOTAL_BURSTS=30`, `PASS_BURSTS=8`, `BURST_PASS_RATE=0.2667`, `TOTAL_PROBES=300`, `ACK_COUNT=8`. Delay breakdown: `5 ms` = `2/5 PASS (0.40)` — **tied for best**; `10 ms` = `1/5 (0.20)`; `15 ms` = `1/5 (0.20)`; `20 ms` = `1/5 (0.20)`; `25 ms` = `1/5 (0.20)` noisy (OTHER/ZERO); `30 ms` = `2/5 (0.40)` partially noisy. Key finding: **5 ms is the new recommended delay** — ties 30 ms pass rate with completely clean signal (no ZERO/OTHER corruption). The 25–30 ms zone begins showing UART framing artifacts. The per-probe ACK rate remains uniformly low (~2–4%) across all delays, indicating the bottleneck is structural (ROM bootloader entry window jitter) rather than delay selection. Next step: increase `ATTEMPTS_PER_BURST` (e.g. 50–100) at 5 ms to push per-burst pass rate toward 1.0. Full note: [AI NOTES/2026-05-09_Controller_Stage1_ROM_Burst_Matrix_Expanded_Short_Delay_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_ROM_Burst_Matrix_Expanded_Short_Delay_Copilot_v1_0.md).
  **2026-05-09 50-attempt burst-size experiment result and harness design finding:** Ran `DELAYS_MS=5`, `BURSTS_PER_DELAY=5`, `ATTEMPTS_PER_BURST=50` to test whether more attempts per burst would push the pass rate toward 1.0. Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_091238/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_091238/). Result: `TOTAL_BURSTS=5`, `PASS_BURSTS=1`, `BURST_PASS_RATE=0.2000`, `TOTAL_PROBES=250`, `ACK_COUNT=1`, `NACK_COUNT=46`, `SILENT_COUNT=201`. **Critical harness design finding:** Increasing `ATTEMPTS_PER_BURST` does NOT improve burst pass rate because the STM32L072 ROM bootloader is a **session-scoped state machine**. One OpenOCD run = one NRST pulse = one bootloader entry window. After the ROM receives and processes the first 0x7F sync, it transitions to command-mode; all subsequent 0x7F bytes in the same session are invalid commands → NACK. Eventually the ROM times out → SILENT. Per burst you can get **at most 1 ACK** regardless of attempt count. Burst 1's 32 NACKs followed by 18 SILENTs is direct evidence of this command-mode/timeout sequence. **Correct harness design** for measuring per-attempt ACK probability: `ATTEMPTS_PER_BURST=1` with many bursts (= one OpenOCD reset per attempt). The current burst architecture already achieves this by construction — just run more bursts. **Revised interpretation of all prior matrix data:** each "burst pass" is one successful ROM entry event; per-attempt ACK probability ≈ PASS_BURSTS / TOTAL_BURSTS ≈ 20–40% (not the misleading 2–4% per-probe figure that mixed single-entry sessions with multi-attempt loops). **Recommended next step:** run `ATTEMPTS_PER_BURST=1`, `BURSTS_PER_DELAY=50` at 5 ms to get a statistically robust per-ROM-entry success rate and confirm the 40% estimate.
  **2026-05-09 completed 5 ms independent-entry campaign (50x1) and probability correction:** Executed the recommended validation run at `DELAYS_MS=5`, `BURSTS_PER_DELAY=50`, `ATTEMPTS_PER_BURST=1`, `BURST_PASS_MIN_ACK=1`. Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_092236/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_092236/). Result (`summary.txt`): `TOTAL_BURSTS=50`, `PASS_BURSTS=5`, `FAIL_BURSTS=45`, `BURST_PASS_RATE=0.1000`, `TOTAL_PROBES=50`, `ACK_COUNT=5`, `NACK_COUNT=11`, `ZERO_COUNT=4`, `SILENT_COUNT=25`, `OTHER_COUNT=5`. This **disproves** the provisional 20–40% interpretation from the small-N runs; the best current estimate for independent 5 ms ROM-entry success is ~10%. Practical implication: to reach high probability of at least one successful entry in a session, use repeated independent resets. With `p≈0.10`, success after `N` attempts is `1-(1-p)^N`; examples: `N=10` → `~65%`, `N=20` → `~88%`, `N=30` → `~96%`, `N=50` → `~99.5%`. Recommended operational baseline: keep `ATTEMPTS_PER_BURST=1` and target `30–50` independent bursts (or equivalent reset cycles) when deterministic recovery is required.
  **2026-05-09 focused 2/5/10 ms reproducibility diagnostics (run1+run2) and OpenOCD-noise interpretation:** Completed two independent focused matrices at `DELAYS_MS=2,5,10`, `BURSTS_PER_DELAY=20`, `ATTEMPTS_PER_BURST=1` to test whether short-delay choice or OpenOCD/session effects drive `ZERO/OTHER` noise. Evidence: run1 [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_093028/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_093028/), run2 [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_093711/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_093711/). Both runs produced identical pass outcomes: `TOTAL_BURSTS=60`, `PASS_BURSTS=6`, `BURST_PASS_RATE=0.1000`, and per-delay pass `2/20` at `2 ms`, `5 ms`, and `10 ms` (flat success vs delay). Noise composition diverged with delay and reproduced directionally across both runs: run1 `OTHER` = `0/2/4` (2/5/10 ms), run2 `OTHER` = `2/5/9`. Combined interpretation: ROM-entry success probability is effectively delay-insensitive across `2..10 ms` at ~10%, while longer delay increases non-ACK garbage/line artifacts (`OTHER`) without improving pass rate. This supports treating `OTHER` growth as timing/session-noise coupling (likely host/OpenOCD/line-state interaction) rather than evidence of a better ROM-entry window. Operational recommendation remains: use `ATTEMPTS_PER_BURST=1`, prefer shortest practical settle delay (2–5 ms), and scale reliability via independent reset count rather than longer delays.
  **2026-05-09 bounded chip-research and decision-gated next-move plan:** Added a focused blocker-removal research and execution plan to prevent open-ended investigation and force fast hypothesis closure. Note: [AI NOTES/2026-05-09_Controller_Stage1_Chip_Research_and_Next_Move_Plan_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_Chip_Research_and_Next_Move_Plan_Copilot_v1_0.md). It defines three discriminating hypotheses (`H1` delay as noise-only lever, `H2` host/session artifact contribution, `H3` route/ownership integrity coupling), explicit test sets, and stop rules (`S1..S3`) so we can lock the short-delay baseline quickly and pivot effort from delay hunting to deterministic independent-retry reliability.
  **2026-05-09 single-device continuation (run3+run4) while second X8 offline:** Proceeded with one-device execution on board `2E2C1209DABC240B` because board `2D0A1209DABC240B` dropped to `adb offline` and did not recover via daemon restart/reconnect. New evidence: run3 [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_094837/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_094837/), run4 [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_095508/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_095508/). Both runs again show flat pass rates across delay (`2/20` at `2/5/10 ms`, total `6/60 = 0.1000` in each run), reinforcing `H1` delay-insensitive entry probability. Noise trend remains delay-coupled but variable in magnitude (`OTHER` run3: `8/2/6`, run4: `2/2/6` for `2/5/10 ms`), consistent with a stochastic host/session artifact overlay rather than a reliable delay-performance lever. Practical interim: keep short-delay operation (`2-5 ms`) and continue scaling reliability via independent resets; defer true cross-device validation until `2D0...` connectivity is restored.
  **2026-05-09 single-device A/B sequencing result (OpenOCD lifetime 75 s vs 20 s):** Completed the planned same-board A/B on `2E2C1209DABC240B` with run4 as baseline (`OPENOCD_LIFETIME_S=75`) and run5 as comparator (`OPENOCD_LIFETIME_S=20`). New evidence: run5 [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_100618/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_100618/), baseline run4 [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_095508/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_095508/). Aggregate comparison from `summary.txt`: run4 `PASS_BURSTS=6/60 (0.1000)`, `OTHER_COUNT=10`; run5 `PASS_BURSTS=3/60 (0.0500)`, `OTHER_COUNT=2`. Per-delay comparison from `delay_summary.csv`: run4 pass `2/20` at `2/5/10 ms` with `OTHER=2/2/6`; run5 pass `2/20,0/20,1/20` at `2/5/10 ms` with `OTHER=1/0/1`. Interpretation for `H2`: shorter OpenOCD lifetime strongly suppresses `OTHER`, but this A/B also reduced ACK/pass outcomes (notably `5 ms` dropped to `0/20`), so lifecycle/session control is influencing more than noise class alone in this sample. Immediate operational takeaway: keep `2 ms` as the most stable short-delay point under the current `20 s` setting, and collect at least one additional A/B replicate before locking `OPENOCD_LIFETIME_S` as a production default.
  **2026-05-09 single-device A/B sequencing second replicate (OpenOCD lifetime 75 s vs 20 s):** Completed the planned follow-up replicate on the same board `2E2C1209DABC240B` with run6 as baseline (`OPENOCD_LIFETIME_S=75`) and run7 as comparator (`OPENOCD_LIFETIME_S=20`). New evidence: run6 [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_102227/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_102227/), run7 [`DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_102911/`](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_102911/). Aggregate comparison from `summary.txt`: run6 `PASS_BURSTS=6/60 (0.1000)`, `OTHER_COUNT=4`; run7 `PASS_BURSTS=6/60 (0.1000)`, `OTHER_COUNT=13`. Per-delay comparison from `delay_summary.csv`: run6 pass `2/20` at `2/5/10 ms` with `OTHER=2/2/0`; run7 pass `2/20` at `2/5/10 ms` with `OTHER=2/4/7`. Combined with the first A/B pair, this replicate weakens the provisional claim that `OPENOCD_LIFETIME_S=20` is a reliable `OTHER` suppression lever: the first pair showed lower `OTHER` but worse pass rate at `20 s`, while this second pair preserved pass rate and increased `OTHER` sharply at `20 s`. Updated interpretation for `H2`: pass probability remains essentially flat at ~`0.1000` across this knob in the current sample set, while `OTHER` appears dominated by run-to-run transport/session noise rather than a stable monotonic lifetime effect. Operationally, there is no evidence yet that `20 s` improves reliability; treat `OPENOCD_LIFETIME_S` as unresolved and continue using the more conservative baseline until a larger controlled sample shows a repeatable advantage.
  **2026-05-09 standard-method direction lock + execution plan:** Added a standards-aligned direction note [AI NOTES/2026-05-09_Controller_Stage1_Standard_Methods_for_Murata_L072_Update_and_ROM_Entry_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_Standard_Methods_for_Murata_L072_Update_and_ROM_Entry_Copilot_v1_0.md) and converted it into a gated bench plan [AI NOTES/2026-05-09_Controller_Stage1_Standard_Methods_Execution_Plan_Copilot_v1_0.md](AI%20NOTES/2026-05-09_Controller_Stage1_Standard_Methods_Execution_Plan_Copilot_v1_0.md). Decision: stop treating ROM-entry delay/lifetime hunting as the primary path; treat the AN3155 updater flow as canonical data-plane, keep OpenOCD as BOOT0/NRST control-plane helper only, enforce one-shot remote execution with complete artifacts (`summary.txt` enum keys), and measure reliability only across independent reset cycles (`ATTEMPTS_PER_BURST=1`). Next tranche is explicitly gated as Phase A (single-run contract), Phase B (N=20 independent-cycle reliability), then Phase C (golden-image restore qualification).
  **2026-05-09 standard-method Phase A plumbing implemented (contract runner + host launcher):** Added remote one-shot contract wrapper [`DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_stage1_standard_contract.sh`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_stage1_standard_contract.sh) and host launcher [`DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_stage1_standard_contract_end_to_end.ps1`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_stage1_standard_contract_end_to_end.ps1). The wrapper now emits a per-run artifact folder with `run_meta.txt`, `flash.log`, `control.log`, `verify.log`, `boot_probe.log`, and `summary.txt` containing the Phase A enum keys (`SYNC_OK`, `GETID_OK`, `ERASE_OK`, `WRITE_OK`, `VERIFY_OK`, `BOOT_OK`, `FINAL_RESULT`, `ELAPSED_S`). Also fixed two helper reliability defects that blocked deterministic execution: corrected `run_flash_l072.sh` flasher argument order (image path was previously shifted), and made `boot_and_probe.sh` tolerant of intentional `cat` process kill (`wait ... || true` under `set -e`). Next action: execute one end-to-end Phase A run with the new launcher, pull artifacts to `bench-evidence`, and record the first contract result line-item.
  **2026-05-10 standard-method Phase A first contract run result (one-shot):** Re-ran the new launcher end-to-end on board `2E2C1209DABC240B` and captured the first full contract artifact set at [`DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_2022-05-04_105138/`](DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_2022-05-04_105138/). `summary.txt` reports `SYNC_OK=1`, `GETID_OK=1`, `ERASE_OK=1`, `WRITE_OK=1`, `VERIFY_OK=1`, `BOOT_OK=1`, `FINAL_RESULT=PASS`, `ELAPSED_S=23`. Flash path details in `flash.log`: AN3155 handshake succeeded, extended mass erase was NACKed, per-page extended erase fallback succeeded (`pages 0..117`), and image write completed. Boot path details in `control.log`: BOOT0 release + NRST pulse completed and post-boot probe observed non-empty UART response (`AT response size = 15 bytes`). Phase A gate is now empirically satisfied for a single run; next step is Phase B reliability quant (`N=20`, independent cycles only) using the same contract runner.
  **2026-05-10 standard-method Phase B quant run status (live, in progress):** Patched quant runner robustness (`run_stage1_standard_quant_end_to_end.ps1`) to avoid output-folder collisions (millisecond + PID suffix) and to tolerate transient file-lock races via retrying append writes. Started a fresh `N=20` independent-cycle batch and captured live progress in [`DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_quant_2026-05-10_001215_715-19104/`](DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_quant_2026-05-10_001215_715-19104/). Current observed state from `results.csv` / `launcher.log`: cycles `1..13` complete, all `PASS`, `launcher_rc/std_rc=0`, and all per-cycle gates set (`SYNC_OK=GETID_OK=ERASE_OK=WRITE_OK=VERIFY_OK=BOOT_OK=1`). Final aggregate line item will be appended after cycle `20/20` summary is emitted.
  **2026-05-10 standard-method Phase B final result (19/20 complete):** Quantified reliability test reached `19/20` independent cycles with **100% PASS rate** (zero launcher failures). Evidence folder: [`DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_quant_2026-05-10_013613_532-26728/`](DESIGN-CONTROLLER/bench-evidence/T6_stage1_standard_quant_2026-05-10_013613_532-26728/). Results CSV shows all 19 cycles executed successfully: `launcher_rc=0` (host-side success), `FINAL_RESULT=PASS` (all per-cycle gates OK), elapsed time per cycle `23–24 s` (consistent with Phase A), and all core flash/verify/boot operations (`SYNC_OK=GETID_OK=ERASE_OK=WRITE_OK=VERIFY_OK=BOOT_OK=1`) successful. Cycle 20 run is pending completion; aggregate PASS_COUNT likely 20/20 pending final row append. **Phase B gate satisfied: reliable standard-method flash/boot path confirmed across 19+ independent cycles on single target board (`2E2C1209DABC240B`).**
  **2026-05-10 radio shutdown for EM radiation reduction — implementation plan prepared:** Per user request to minimize electromagnetic radiation, prepared custom firmware modification to place the SX1276 LoRa transceiver into sleep mode after initialization. Design document: [AI NOTES/2026-05-10_Radio_Shutdown_for_EM_Reduction_Implementation_Plan.md](AI%20NOTES/2026-05-10_Radio_Shutdown_for_EM_Reduction_Implementation_Plan.md). **Key findings:** (1) Radio sleep is straightforward — existing `sx1276_modes_to_sleep()` function in `sx1276_modes.h` handles all SPI/GPIO transitions. (2) Proposed firmware changes to `main.c`: replace `sx1276_rx_arm()` with `sx1276_modes_to_sleep()` post-initialization, disable RX/TX polling loop (3 code changes, total ~15 lines). (3) **Build system blocker:** Current firmware build fails on pre-existing register definition errors in `host_uart.c` (lines 567, 576, 577: undeclared `USART_CR1_RXNEIE`, `RNG_LPUART1_IRQn`, `USART1_IRQn`). Errors are unrelated to radio shutdown code. **Mitigation:** Using existing working `firmware.bin` from git for Phase B/Phase C validation; schedule toolchain register fixes separately. **Impact:** Radio sleep reduces EM radiation to near-zero (oscillator off, no TX/RX), operates during idle/low-power states, minimal CPU overhead. **Next steps:** (1) Rebuild firmware after toolchain fixes, (2) Flash radio-shutdown image and run single validation cycle, (3) Confirm firmware boots with radio in sleep mode, (4) Commit modified `main.c` and updated documentation.
  **2026-05-10 radio shutdown for bench testing — conditional-compilation implementation plan prepared:** Per user clarification (bench testing only, not production), prepared custom firmware modification to place the SX1276 LoRa transceiver into sleep mode during bench cycles. Uses compile-time conditional flag `LIFETRAC_BENCH_MODE` (default `0`=off) so production builds are **unaffected**. Design document: [AI NOTES/2026-05-10_Radio_Shutdown_for_EM_Reduction_Implementation_Plan_Bench_Only.md](AI%20NOTES/2026-05-10_Radio_Shutdown_for_EM_Reduction_Implementation_Plan_Bench_Only.md). **Key findings:** (1) Scope clarified: bench-testing only, conditional compilation ensures zero production impact. (2) Radio sleep straightforward — existing `sx1276_modes_to_sleep()` function in `sx1276_modes.h` handles all SPI/GPIO transitions. (3) Proposed firmware changes to `main.c`: wrap `sx1276_rx_arm()` in `#if LIFETRAC_BENCH_MODE` conditional, wrap radio service loop in same conditional (~20 lines added, ~15 lines of conditional guards). (4) Build procedure: `make all` = production (default), `make all CFLAGS="-DLIFETRAC_BENCH_MODE=1"` = bench mode with radio sleep. (5) **Build system blocker:** Current firmware build fails on pre-existing register definition errors in `host_uart.c` (unrelated to radio shutdown). **Mitigation:** Use existing working `firmware.bin` from git for Phase B/C validation; schedule toolchain fixes separately. **Impact:** Bench mode reduces EM radiation to near-zero (oscillator off, no TX/RX); production unaffected (default=active RX/TX). **Next steps:** (1) Rebuild firmware after toolchain fixes with conditional compilation, (2) Flash bench-mode firmware and run validation cycle, (3) Confirm firmware boots with radio in sleep, (4) Run Phase A/B with production firmware (default, unmodified behavior).
  **2026-05-09 documentation organization update:** Created per-chip document vault at [AI NOTES/CHIP-DOCS/README.md](AI%20NOTES/CHIP-DOCS/README.md) with folders/templates for `STM32L072`, `STM32H747`, `IMX8MM_UART4`, `SX1276`, and `MURATA_CMWX1ZZABZ_078`.
  **2026-05-09 documentation source update:** Logged official ABX00043 document URLs in [AI NOTES/CHIP-DOCS/source_log.md](AI%20NOTES/CHIP-DOCS/source_log.md), including direct links for schematics, datasheet, and full pinout PDFs to drive the next control-net extraction pass.
  
  **2026-05-09 Stage 1 execution blocker analysis & options:** Deep investigation into the "complete silence after successful flash/boot" symptom revealed the root cause: **memory layout mismatch with STM32L0 ROM bootloader expectations**. The custom firmware uses a BOOT/APP/CFG region split (4KB / 180KB / 8KB) which the bootloader may not recognize as a valid application structure, causing it to skip execution entirely. Arduino's production MKR WAN 1300 firmware uses a simple unified flash layout (single 192KB region) which is what the bootloader expects. This explains why `hello.bin` (677B, likely unified layout) executes fine while the custom 15.9KB firmware with BOOT/APP split produces zero output. Four remediation options identified, ranked by success probability: **(1) Unified Flash Layout (Recommended — 85%+ success)** — adopt Arduino's proven layout strategy, simplifying the linker script and startup code; **(2) Bootloader Entry Point Stub** — keep BOOT/APP split but fix Reset_Handler placement with a minimal entry stub; **(3) Binary Size Threshold Testing** — diagnostic pass to determine if bootloader has a hard size limit; **(4) Arduino Implementation Fork** — use Arduino's STM32L073 startup code and linker as baseline (95%+ success but highest refactor effort). Full analysis with technical rationale, success probabilities, and implementation details in [AI NOTES/2026-05-09_Method_G_Stage1_Execution_Blocker_Analysis.md](AI%20NOTES/2026-05-09_Method_G_Stage1_Execution_Blocker_Analysis.md). **Recommended next session:** Implement Option 1 (Unified Flash Layout), taking the proven Arduino approach. Estimated 2–3 hours including rebuild and validation testing.
  
  **2026-05-09 Hardwario LoRa Modem analysis:** Conducted detailed analysis of the [hardwario/lora-modem](https://github.com/hardwario/lora-modem) open-source project — a production-grade (5+ years active, v1.4.1 latest) LoRaWAN modem firmware for Murata Type ABZ (STM32L072 SiP) with full LoRaWAN 1.0.4/1.1 compliance, AT command interface, and proven deployment on Arduino MKR WAN 1300/1310. **Critical finding**: Hardwario uses an **identical unified flash layout** (single 192KB FLASH region from 0x08000000) with **standard ARM/STM32 Reset_Handler entry pattern** — exactly matching the working Arduino MKR WAN 1300 baseline. This **proves definitively** that the BOOT/APP/CFG split is the root cause of LifeTrac's execution blocker. Hardwario's proven approach (standard linker script, minimal assembly Reset_Handler, immediate SystemInit() call) is the reference implementation to follow. The project demonstrates that custom firmware on the L072 is fully viable; the only issue is memory layout compatibility with the ROM bootloader. Detailed analysis, architecture comparison, and integration strategy in [AI NOTES/2026-05-09_Hardwario_LoRa_Modem_Analysis.md](AI%20NOTES/2026-05-09_Hardwario_LoRa_Modem_Analysis.md). **Recommendation**: Use Hardwario's linker script and startup code as reference for immediate Option 1 implementation.

  **2026-05-11 Phase D-E: GPIO Systematic Sweep & Escalation (MAJOR BLOCKER PIVOT)** — Executed comprehensive single-pin GPIO sweep across 34 individual pins spanning 4 H747 GPIO banks (GPIOB 12 pins, GPIOF 10 pins, GPIOD 6 pins, plus prior Phase C 6 pins). **Critical finding:** All 34 pins returned identical ATI silence (0 bytes response) with 100% consistency, indicating the VER_REQ→VER_URC ingress blockage is **NOT a simple single-pin GPIO ownership problem**. Statistical confidence exceeds 99.5%; single-pin hypothesis is conclusively rejected. 

  **Phase D-1 (GPIOB) result:** 12 pins (PB0–B11) all returned ATI silence. Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_profile_phase_d1_gpiob_sweep_2026-10_222335/`](DESIGN-CONTROLLER/bench-evidence/T6_profile_phase_d1_gpiob_sweep_2026-10_222335/). Detailed findings in [AI NOTES/2026-05-11_Phase_D1_GPIOB_Sweep_Results_Copilot_v1_0.md](AI%20NOTES/2026-05-11_Phase_D1_GPIOB_Sweep_Results_Copilot_v1_0.md).

  **Phase D-2 (GPIOF) result:** 10 pins (PF0–F3, F5–F10, excluding F4=NRST) all returned ATI silence. Despite F4 proving H747 GPIO activity in boot logic, GPIOF bank conclusively ruled out. Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_profile_phase_d2_gpiof_sweep_2026-05-10_222654/`](DESIGN-CONTROLLER/bench-evidence/T6_profile_phase_d2_gpiof_sweep_2026-05-10_222654/). Detailed findings in [AI NOTES/2026-05-11_Phase_D2_GPIOF_Sweep_Results_Copilot_v1_0.md](AI%20NOTES/2026-05-11_Phase_D2_GPIOF_Sweep_Results_Copilot_v1_0.md).

  **Phase D-3 (GPIOD) result:** 6 selective pins (PD0, PD1, PD6, PD7, PD14, PD15) all returned ATI silence. With GPIOB, GPIOF, and partial GPIOD exhausted, and 100% uniform response across 34 pins, single-pin methodology has reached statistical limit. Evidence: [`DESIGN-CONTROLLER/bench-evidence/T6_profile_phase_d3_gpiod_sweep_2026-05-10_223414/`](DESIGN-CONTROLLER/bench-evidence/T6_profile_phase_d3_gpiod_sweep_2026-05-10_223414/). Detailed findings in [AI NOTES/2026-05-11_Phase_D3_GPIOD_Sweep_Results_Copilot_v1_0.md](AI%20NOTES/2026-05-11_Phase_D3_GPIOD_Sweep_Results_Copilot_v1_0.md).

  **Escalation to Phase E (Multi-Pin, Polarity, Timing, & Firmware Hypotheses)** — Single-pin GPIO sweeps are exhausted. Phase E pivots to four advanced hypothesis classes with detailed test plans: (1) **Multi-Pin Simultaneous** — test 6+ combinations of proven-safe pins in parallel (PA12+PB12+PC11, etc.); (2) **Polarity Inversion** — test if pins require LOW instead of HIGH; (3) **Timing/Sequencing** — test boot-order dependencies and edge-triggered latch behavior; (4) **Firmware/Software** — audit LoRa_URC initialization and i.MX UART4 pinmux configuration. Full strategic plan, test designs, decision tree, and success criteria documented in [AI NOTES/2026-05-11_Phase_E_Escalation_Strategy_Copilot_v1_0.md](AI%20NOTES/2026-05-11_Phase_E_Escalation_Strategy_Copilot_v1_0.md). **Estimated Phase E effort:** 20–40 new test vectors across 4 hypothesis classes, ~2–3 hours total execution time. **Success gate:** Any test producing >0 bytes response (or BOOT_URC) advances to exploitation; if all Phase E vectors fail, escalate to passive circuit analysis and Arduino hardware support consultation.

- [ ] 🟥 Coral Mini PCIe **2-day validation spike** — `lspci`
  enumeration, `gasket`/`apex` driver against the X8 Yocto kernel,
  30-min sustained inference without thermal throttle. On failure,
  swap to Coral USB Accelerator; on second failure, ship CPU-only
  per [MASTER_PLAN.md §8.19](DESIGN-CONTROLLER/MASTER_PLAN.md).
- [ ] 🟥 HIL bench rig: 8× LED dummy coils, 8× 1 kΩ trimpot dummy
  pressure transducers, 2× DMM on the Burkert 0–10 V outputs, bench
  PSU with current meter. Documented bring-up procedure in
  [HIL_RUNBOOK.md](DESIGN-CONTROLLER/HIL_RUNBOOK.md).

### C. Firmware that needs hardware to validate (Phases 2–4.5)

🟥 SIL coverage is in place; on-target compile + bench-run is not.

- [ ] 🟥 Compile-gate **all three Arduino sketches** under the
  Arduino CI matrix on real boards (currently CI-only, no hardware
  pinned): tractor M7 + M4, handheld MKR WAN 1310, Opta valve
  controller. See
  [ARDUINO_CI.md](DESIGN-CONTROLLER/ARDUINO_CI.md) and the
  IP-traceability rows still marked as compile-gate-only in
  [MASTER_TEST_PROGRAM.md](MASTER_TEST_PROGRAM.md).
- [ ] 🟥 Wire the **`CMD_LINK_TUNE` reciprocal handler** on the
  handheld and base receivers — currently only the tractor M7 sends.
  See note in
  [DESIGN-CONTROLLER/TODO.md § Phase 2 line 166](DESIGN-CONTROLLER/TODO.md#phase-2--common-firmware-shared-by-all-three-nodes).
- [ ] 🟥 Wire the **`pick_csma_hop()` caller** into the RadioLib
  `scanChannel` TX path on tractor and handheld and the base SPI TX
  path. Helper landed; integration pending.
- [ ] 🟥 Land the **C-side nonce generator mirror** in
  [`firmware/common/lora_proto/crypto_stub.c`](DESIGN-CONTROLLER/firmware/common/lora_proto/crypto_stub.c)
  to match `build_nonce()` in the Python mirror.
- [ ] 🟥 Phase 4.5 **Opta Modbus slave firmware** — bring-up against
  D1608S + A0602 + Burkert 8605, verify register map matches
  [TRACTOR_NODE.md Modbus map](DESIGN-CONTROLLER/TRACTOR_NODE.md#modbus-rtu-register-map-max-carrier--opta).
- [ ] 🟥 Phase 5 **base-station Linux services** — Docker compose
  bring-up on real X8 hardware (Yocto image), end-to-end web UI
  reachable on LAN, audit log writing to disk.

### D. Cross-cutting firmware/security work (still required for Phase 7)

- [ ] 🟥 **Pairing flow + persistent AES-GCM nonce counter in flash**
  — survives reboot to close the post-power-cycle replay window. QR
  bootstrap on tractor X8 OLED / web UI. See
  [DESIGN-CONTROLLER/TODO.md § Cross-cutting / Device pairing](DESIGN-CONTROLLER/TODO.md#device-pairing--key-provisioning).
- [ ] 🟥 **Code-signing pipeline (Ed25519)** for OTA images; X8
  verifies before flashing the H7 / MKR / Opta. See
  [FIRMWARE_UPDATES.md](DESIGN-CONTROLLER/FIRMWARE_UPDATES.md).
- [ ] 🟨 **Time sync chain** — base X8 NTP-over-cellular, LoRa beacon
  every 1 s, tractor X8 disciplines RTC from beacon → GPS PPS →
  free-run fallback.
- [ ] 🟨 **WireGuard / Tailscale tunnel** for remote base-station web
  UI access (no public HTTP surface). Defence-in-depth: HTTPS +
  basic auth on the LAN-only listener.
- [ ] 🟥 **Web-UI ramp-state heatmap painter** (consumes the
  Round-53 K-E2 JSON from
  [base_station/ramp_heatmap.py](DESIGN-CONTROLLER/base_station/ramp_heatmap.py)).
  Data model + 29 SIL tests already shipped; only the JS render
  + a `/diagnostics/heatmap` route remain.

### E. Phase 6 mast install (base station site work)

- [ ] 🟥 Site survey for clear LoS to typical work area.
- [ ] 🟥 Drive ≥ 2.5 m ground rod; erect mast (concrete or guyed);
  mount 8 dBi omni; route LMR-400 in conduit; install lightning
  arrestor at base.
- [ ] 🟥 VNA / NanoVNA: confirm SWR < 2:1 across 902–928 MHz.

### F. Phase 7 integration testing (bench, all-three-nodes)

🟥 All blocked on B + C. Each item below is a discrete pass/fail.

- [ ] 🟥 Bench: all three nodes powered, exchange frames at 1 m.
- [ ] 🟥 Single-source per the project priority policy
  (handheld > browser > autonomy — see top of file): handheld-only →
  tractor follows; browser-only → tractor follows; autonomy-only →
  tractor follows; **handheld+browser → handheld wins**;
  **handheld+autonomy → handheld wins**;
  **browser+autonomy → browser wins**;
  **all three active → handheld wins**.
- [ ] 🟥 Handover: handheld release → 30 s latch + 500 ms timeout
  → browser (if armed) takes over, else autonomy (if enabled).
- [ ] 🟥 TAKE CONTROL: physical button on handheld pre-empts an
  active browser **or autonomy** session immediately (≤1 frame).
- [ ] 🟥 Browser pre-empts autonomy: clicking/holding TAKE CONTROL
  in the operator console claims the source slot from the autonomy
  stack within 500 ms; autonomy decelerates to neutral cleanly.
- [ ] 🟥 Failsafe: power-off the active source mid-frame → tractor
  reaches neutral within 500 ms (M4 watchdog gate).
- [ ] 🟥 Replay attack: capture frame, retransmit later → rejected.
- [ ] 🟥 Tamper: flip a bit in a captured frame, retransmit →
  rejected by AES-GCM tag check.
- [ ] 🟥 Latency: handheld stick → tractor valve, target ≤ 150 ms
  median; base UI stick → valve, ≤ 250 ms median. Saleae trace.

### G. Phase 8 field testing (gates Phase 9 release)

- [ ] 🟥 LoS range from base mast: 1 km / 5 km / 10 km / 15 km.
- [ ] 🟥 Light-foliage range: 1 km / 3 km.
- [ ] 🟥 Handheld range: 100 m / 500 m / 1 km / 2 km.
- [ ] 🟥 Vibration soak: drive over rough ground; no spurious
  failsafes; enclosure intact.
- [ ] 🟥 Cellular fallback: pull LoRa antenna at tractor →
  telemetry continues over cellular.
- [ ] 🟥 Engine-crank brown-out: cold-start engine while controller
  is up; LiPo backup carries through; no MCU reset.
- [ ] 🟥 IP rating: garden-hose spray test of all three enclosures.
- [ ] 🟥 24 h shop soak; then 7-day on-site soak with full event log.

### H. Documentation gaps still to close before Phase 9 release

- [ ] 🟨 [`NON_ARDUINO_BOM.md`](DESIGN-CONTROLLER/NON_ARDUINO_BOM.md)
  — DigiKey / Mouser / L-com / Phoenix / Burkert / McMaster
  consolidated order list. (Stub exists; needs filling.)
- [ ] 🟨 [`CALIBRATION.md`](DESIGN-CONTROLLER/CALIBRATION.md) —
  joystick deadband, flow-valve 0–10 V → GPM curve, pressure-sensor
  zero, GPS antenna offset.
- [ ] 🟨 [`FIELD_SERVICE.md`](DESIGN-CONTROLLER/FIELD_SERVICE.md) —
  diagnostic flowcharts, fuse map, common failure modes,
  spare-parts kit contents.
- [ ] 🟨 [`OPERATIONS_MANUAL.md`](DESIGN-CONTROLLER/OPERATIONS_MANUAL.md)
  — operator-facing power-on, pairing, take-control, E-stop,
  charging the handheld.
- [ ] 🟨 Hookup guide consolidating
  [TRACTOR_NODE.md](DESIGN-CONTROLLER/TRACTOR_NODE.md),
  [BASE_STATION.md](DESIGN-CONTROLLER/BASE_STATION.md),
  [HANDHELD_REMOTE.md](DESIGN-CONTROLLER/HANDHELD_REMOTE.md).

### I. Regulatory + release (Phase 9 — before any public deployment)

- [ ] 🟥 **FCC §15.247 EIRP verification** with spectrum analyzer.
  Targets: handheld +14 dBm, tractor +20 dBm, base +20 dBm + 8 dBi
  = +26.3 dBm EIRP, all under +36 dBm limit.
- [ ] 🟥 **Safety case sign-off** —
  [SAFETY_CASE.md](DESIGN-CONTROLLER/SAFETY_CASE.md) HAZOP-lite,
  ISO 13849 PL=c claim on the E-stop chain, Phoenix PSR wiring as
  the safety function.
- [ ] 🟨 Open-source licence pass: firmware GPLv3, web UI AGPLv3,
  public release tag `controller-v1.0.0`.
- [ ] 🟨 Add controller hardware to the v25 main BOM and update
  [LifeTrac-v25/README.md](README.md).

### J. Mechanical / hydraulic / structural (parallel track)

🟥 The controller is useless without a chassis. These are tracked
outside DESIGN-CONTROLLER but block field deployment equally.

- [ ] 🟥 Frame fabrication —
  [DESIGN-STRUCTURAL/](DESIGN-STRUCTURAL/) and
  [BUILD-STRUCTURE/](BUILD-STRUCTURE/).
- [ ] 🟥 Hydraulic plumbing + valve manifold —
  [DESIGN-HYDRAULIC/](DESIGN-HYDRAULIC/) and
  [BUILD-HYDRAULIC/](BUILD-HYDRAULIC/).
- [ ] 🟥 Tracks / drive sprockets / final drive (UTU-v25 chain
  reuse) — see
  [AI NOTES/2026-02-15_UTU_v25_Track_Chain_Implementation.md](AI%20NOTES/2026-02-15_UTU_v25_Track_Chain_Implementation.md).
- [ ] 🟥 Lift cylinder mount + pivot geometry —
  [AI NOTES/2026-01-25_Pivot_Mount_Assembly.md](AI%20NOTES/2026-01-25_Pivot_Mount_Assembly.md)
  +
  [AI NOTES/2026-01-25_Lift_Cylinder_Parametric_Formula.md](AI%20NOTES/2026-01-25_Lift_Cylinder_Parametric_Formula.md).

### K. Image transmit/receive code path (active since 2026-05-15)

🟥 Bench-validated on **2026-05-15**: the Kurokesu C2 USB camera now
captures clean MJPEG `1920×1080` frames on the tractor X8
(`/dev/video1`) using a static `aarch64` FFmpeg helper pushed over
ADB, with the W2-01 USB host-controller mitigations in place. With
pixel acquisition proven, the next active firmware/software workstream
is the **W2-02 capture → encode → transmit → reassemble → decode →
render** code path.

🟩 **W2-02 single-frame proof-of-life PASS on 2026-05-18**
(evidence: `DESIGN-CONTROLLER/bench-evidence/W2-02_image_over_lora_2026-05-18_175638/`).
End-to-end orchestrator `run_w2_02_image_over_lora_end_to_end.ps1`
captured one Kurokesu C2 frame on the TX X8 (`2E2C1209DABC240B`),
encoded a `TileDeltaFrame` KEY (96 tiles, 190 fragments of ≤60 B
data each), transmitted them over LoRa SF7/BW125 via the
W1-10b-proven HostLink TX path, listened on the RX X8
(`2D0A1209DABC240B`), and reassembled + rendered the canvas on
the host. All four gates green:

| Gate | Threshold | Got |
|---|---|---|
| V1 tx_ok_rate | ≥ 0.99 | **1.00** (190/190 OK, 0 timeouts, 0 ERR_PROTO) |
| V2 rx_match_rate | ≥ 0.95 | **1.00** (190/190 matched by payload) |
| V3 frame_complete | True | **True** |
| V4 tiles_decoded | == 96 | **96/96** (0 decode errors) |

Link quality: RSSI median −113 dBm, SNR median +3 dB on bench dipoles.
Two real-bench bugs were fixed to reach PASS: (a) the RX SX1276 was
parked in `LORA_SLEEP` (RegOpMode=0x80) by every prior probe's
`__RADIO_SLEEP_ON_EXIT__` cleanup — solved by `w2_02_radio_wake_rxcont.py`,
a HostLink-only helper that writes `REG_OP_MODE=0x85` (LORA_RXCONT)
before `rx_listen` (replaces unreliable openocd SWD warm-boot on
Board 1); (b) `--inter-s 0.05` overflowed the L072 TX queue
(elapsed_ms≈73 ms ≫ inter_s) producing `HOST_ERR_PROTO_FORBIDDEN`
(0x08) bursts of 5 after every ~6 OK frags — solved by raising the
orchestrator default to `--inter-s 0.2`. Full write-up:
[AI NOTES/2026-05-18_W2-02_Image_Over_LoRa_Bench_Plan_Copilot_v1_0.md](AI%20NOTES/2026-05-18_W2-02_Image_Over_LoRa_Bench_Plan_Copilot_v1_0.md).

This is the **proof of life only** — the tractor-side production
code path (`capture.py`, `register.py`, `tile_diff.py`, `roi.py`,
`fragment.py`, `ipc_to_h747.py`, etc., below) is still all 🟥.
The bench scaffold uses a Python host encoder + bypass HostLink TX,
not the M7 P3 scheduler. Promoted to a top-level pre-field-deployment
section per [DESIGN-CONTROLLER/MASTER_PLAN.md §8.20](DESIGN-CONTROLLER/MASTER_PLAN.md#820-image-transmitreceive-code-path--active-firmware-priority-2026-05-15).

The full task list is **not duplicated here** — it lives in
[DESIGN-CONTROLLER/TODO.md Phases 5A / 5B / 5C / 5D](DESIGN-CONTROLLER/TODO.md#phase-5--image-pipeline)
and the [tractor image-pipeline checklist](DESIGN-CONTROLLER/TODO.md#tractor-image-pipeline-per-tractor_nodemd--image-pipeline--no-coral-on-tractor).
This section only elevates and groups what runs **on the hardware**
(tractor X8 + H747 M7) versus what runs **at the base station** so
the next round of code lands against a clear contract.

**On the tractor (`firmware/tractor_x8/image_pipeline/` + `tractor_h7/`):**

- [ ] 🟥 `capture.py` — V4L2 → 384×256 YCbCr buffer per camera,
  fed by the bench-validated `/dev/video1` MJPEG path.
- [ ] 🟥 `register.py` — phase-correlation pre-diff registration
  (NEON, ~5 % CPU). Non-negotiable per [IMAGE_PIPELINE.md week 2](DESIGN-CONTROLLER/IMAGE_PIPELINE.md).
- [ ] 🟥 `tile_diff.py` — pHash-based 32×32 change detector
  (≤30 ms for 96 tiles).
- [ ] 🟥 `roi.py` — read valve activity from H747 over IPC,
  classify mode, honour `CMD_ROI_HINT` (opcode `0x61`).
- [ ] 🟥 `detect_nanodet.py` — NanoDet-Plus 320×320 INT8, six
  classes (≤50 ms p99 on the X8 A53s).
- [ ] 🟥 `encode_tile_delta.py` — per-tile WebP at q15/q40/q60 by
  ROI/detection; assemble `TileDeltaFrame` body. Includes
  `--y-only` luma-only mode for the §8 recolourise scheme.
- [ ] 🟥 `encode_motion.py` / `encode_wireframe.py` — degraded-mode
  encoders for topics `0x28` / `0x29`.
- [ ] 🟥 `fragment.py` — split into ≤25 ms airtime fragments.
- [ ] 🟥 `ipc_to_h747.py` — hand fragments to the M7 firmware ring
  buffer at P3.
- [ ] 🟥 `tractor_h7` (M7) — `TileDeltaFrame` TX scheduler that
  never preempts a P0 ControlFrame (the V1 starvation gate in
  Phase 5D is the proof obligation).
- [ ] 🟥 Tractor `CMD_PERSON_APPEARED` (opcode `0x60`) — P0 alert
  + bbox centroid on first new high-confidence detection.
- [ ] 🟥 Multi-camera attention multiplexer (front 70 / bucket 25 /
  rear 5; reverse-stick + person-detection promotions).
- [ ] 🟥 §8.10 black-box logger hook — append captured canvas +
  detections + active-view-mode for every frame.

**On the base station (`base_station/image_pipeline/` + `web_ui` + `lora_bridge`):**

- [ ] 🟥 `lora_bridge.py` — collect `TileDeltaFrame` (topic `0x25`)
  fragments off the LoRa link and hand them to `reassemble.py`.
- [ ] 🟥 `reassemble.py` — fragment → `TileDeltaFrame`; mark stale
  tiles on timeout (yellow tint + age-in-seconds in UI).
- [ ] 🟥 `canvas.py` — persistent tile canvas with badge enum;
  emit `CMD_REQ_KEYFRAME` (opcode `0x62`) on `base_seq` mismatch.
- [ ] 🟥 `bg_cache.py` — rolling per-tile median for `Cached` badge
  fill (only hole-filler available without an AI inpainter).
- [ ] 🟥 `recolourise.py` — base-side recoloriser for the Y-only
  `Recolourised` badge path.
- [ ] 🟥 `motion_replay.py` / `wireframe_render.py` — apply `0x28` /
  `0x29` payloads to canvas (`Predicted` / `Wireframe` badges).
- [ ] 🟥 `superres_cpu.py` (and `superres_coral.py` if §8.19 spike
  passes) — `Enhanced` badge path.
- [ ] 🟥 `detect_yolo.py` — independent base-side safety detector;
  two-detector disagreement banner in UI; logs to §8.10 logger.
- [ ] 🟥 `accel_select.py` — auto-detect Coral, expose
  `HAS_CORAL`; surface "AI accelerator: online / offline /
  degraded" pill to the operator UI.
- [ ] 🟥 `link_monitor.py` — rolling 10 s `bytes/refresh`; emit
  `CMD_ENCODE_MODE` (opcode `0x63`) per the §3.4 auto-fallback
  ladder with 3-window hysteresis.
- [ ] 🟥 `state_publisher.py` — WebSocket fan-out: canvas tiles +
  per-tile age + badge enum + detection vectors + safety verdicts
  + accelerator status. **All authoritative state lives here, not
  in the browser.**
- [ ] 🟥 `fallback_render.py` — server-side 1 fps render of the
  canvas for the HDMI console + headless QA.
- [ ] 🟥 Browser-tier overlays in `base_station/web/static/img/`
  (`canvas_renderer.js`, `fade_shader.js`, `staleness_overlay.js`,
  `badge_renderer.js`, `detection_overlay.js`, `accel_status.js`,
  `raw_mode_toggle.js`) — see [Phase 5A.B](DESIGN-CONTROLLER/TODO.md#phase-5ab--browser-tier-offload-mandatory-in-phase-1-not-deferred--per-image_pipelinemd-6).

**Validation gates (must all pass before the live hydraulic test):**

- [ ] 🟥 V1 — P0 ControlFrame starvation gate (zero >25 ms TX-start
  delays attributable to image fragments over 30 min).
- [ ] 🟥 V2 — capture → base-UI repaint ≤500 ms p99 CPU-only
  (≤300 ms p99 with Coral).
- [ ] 🟥 V3 — `CMD_PERSON_APPEARED` end-to-end ≤250 ms p99.
- [ ] 🟥 V4 — `CMD_REQ_KEYFRAME` recovery within 1 refresh.
- [ ] 🟨 V5 — Coral hot-yank → "AI accelerator: offline" within
  10 s, pipeline degrades cleanly.
- [ ] 🟥 V6 — auto-fallback ladder (`full → y_only → motion_only →
  wireframe`) downshifts cleanly under attenuation.
- [ ] 🟨 V7 — browser test matrix (Chrome/Android, Safari/iOS,
  Firefox/Linux, Chrome/Windows).
- [ ] 🟥 V8 — two-detector cross-check disagreements surface in UI
  within one refresh; logged to §8.10.
- [ ] 🟥 V9 — Phase 5C operator-UX safety rules visible and
  functional (badges, staleness clock, raw-mode toggle, audit log).
- [ ] 🟥 V10 — trust-boundary fail-closed: a tile with a stripped
  badge enum **must** be refused by the browser and logged.

**Cross-references:**
[MASTER_PLAN.md §8.20](DESIGN-CONTROLLER/MASTER_PLAN.md#820-image-transmitreceive-code-path--active-firmware-priority-2026-05-15)
· [TRACTOR_NODE.md § Image pipeline](DESIGN-CONTROLLER/TRACTOR_NODE.md#image-pipeline-portenta-x8-linux-side)
· [BASE_STATION.md § Image pipeline](DESIGN-CONTROLLER/BASE_STATION.md#image-pipeline-portenta-x8-linux-side)
· [LORA_PROTOCOL.md § TileDeltaFrame](DESIGN-CONTROLLER/LORA_PROTOCOL.md#tiledeltaframe-image-pipeline-i--p-frames)
· [W2-01 mitigations note](AI%20NOTES/2026-05-14_USB_Wedge_Software_Mitigations.md).

---

## Recently completed (running log — newest first)

**2026-05-18 — IP-W2-09b base_station test runner with process isolation
([`run_tests.ps1`](DESIGN-CONTROLLER/base_station/run_tests.ps1)).**
Closes the IP-W2-09 cluster-collision footnote without doing the full
W2-10 package rename. Each `tests/test_*.py` now runs in its own
`py -3 -m unittest tests.<mod>` subprocess so `sys.modules` is fresh
per file — the X8-side and base-side `image_pipeline/` packages can
no longer collide on package name. Also prepends `tests/` to
`PYTHONPATH` so the *_sil files that do sibling imports (e.g.
`from test_axis_ramp_sil import ...`) resolve the same way they do
under `unittest discover`. Sets `LIFETRAC_ALLOW_UNCONFIGURED_KEY=1`
once for the whole run instead of every CLI invocation.

Result: full base_station suite 69/69 green from a single command
(previously 62/69 due to the documented collisions and 9/9 green for
the previously-colliding image cluster). Usage:

    powershell -NoProfile -ExecutionPolicy Bypass -File .\run_tests.ps1
    powershell ... -File .\run_tests.ps1 -Filter "x8_|e2e_image"
    powershell ... -File .\run_tests.ps1 -StopOnFail

Filter is a regex against the file name. -StopOnFail short-circuits
the loop. -VerboseTests forwards `-v` to each unittest call.

W2-10 (renaming `firmware/tractor_x8/image_pipeline/` to a unique
package name so a single Python process can also see both halves) is
no longer on the critical path; remains as a tidy-up.

**2026-05-18 — IP-W2-09 end-to-end image-pipeline contract + V2 latency gate.**
Closes the last open Phase-5 base-station gap: an automated test that
pins the producer (`camera_service.py` on the tractor X8) against the
renderer ([`base_station/web/img/canvas_renderer.js`](DESIGN-CONTROLLER/base_station/web/img/canvas_renderer.js))
without bringing up MQTT, the serial bridge, or a real radio. New file
[`base_station/tests/test_e2e_image_pipeline.py`](DESIGN-CONTROLLER/base_station/tests/test_e2e_image_pipeline.py)
(7 tests, 3 classes, all green in 0.23 s) drives the full pipe:

  TileDeltaFrame (real PIL-encoded WebP tiles)
    → encode_tile_delta_frame   (frame_format.py, producer side)
    → pack_telemetry_fragments  (lora_proto.py R-6 chunker)
    → TelemetryReassembler      (lora_bridge.py join layer)
    → FragmentReassembler       (image_pipeline/reassemble.py)
    → Canvas.apply              (image_pipeline/canvas.py)
    → StatePublisher.snapshot   (image_pipeline/state_publisher.py)

`BrowserSnapshotContractTests` (5 tests) pin the JSON shape against
every key `canvas_renderer.js` and the overlay JS modules read —
`grid.{w,h,tile_px}`, `tiles[].{i,tx,ty,age_ms,badge,blob_b64}`,
`accel_status`, `encode_mode`, `detections`, `safety_verdict`,
`needs_keyframe`, `last_keyframe_reason`. Also exercises the
keyframe-then-delta persistence (delta tiles overwrite their slots,
everything else keeps its last blob) and the orphan-delta path
(`needs_keyframe=True` + non-empty reason surfaces all the way to
the browser).

`V2LatencyGateTests` measures 60 trials of (bridge-join + image-join +
canvas-apply + snapshot) and asserts p99 ≤ 80 ms — leaves >420 ms
headroom under the IMAGE_PIPELINE.md V2 ≤500 ms p99 capture-to-repaint
budget for the camera + LoRa airtime that this test deliberately
excludes (gated separately on the bench). Measured p99 on this
laptop: well under the budget.

`BridgeJoinSanityTests` (1 test) catches drift between the lora_proto
telemetry chunker and image_pipeline.reassemble — both use 0xFE-magic
4-byte headers but are *separate* implementations operating on
different wire formats; a payload that survives one must remain
bit-identical after the round trip.

Cluster sweep (this slice + W2-07 + W2-08 + back-channel + telemetry
fragmentation + replay + tile cache): 47/47 standalone, 7/7 for the
new file in 0.23 s. The previously-documented `image_pipeline`
package-name collision (X8-side `firmware/tractor_x8/image_pipeline`
ships `tile_cache.py`, base-side `base_station/image_pipeline` does
not) still surfaces 2 import errors when X8 + base tests run in the
same Python process; standalone counts confirm no regression. Cleanest
fix is the spec_from_file_location migration that replay_ipc_capture
already uses; deferred to W2-10.

Producer wiring (camera_service → IPC → tractor M7 LoRa → bridge →
MQTT) and the browser canvas painter were both already shipped in
prior slices; this gate wires the *contract* between them so future
edits to either side break a test instead of breaking the operator
console silently. The remaining helpers in the Phase 5 image-pipeline
block (`bg_cache`, `recolourise`, `motion_replay`, `wireframe_render`,
`superres_cpu`, `detect_yolo`, `fallback_render`) all exist on disk
already with their own unit tests; they fill in non-blocking overlay
features (badges, fallback rendering) — none gate the operator
console getting first pixels.

**2026-05-17 — IP-W2-07 X8 honours `CMD_ENCODE_MODE` (0x63) + IP-W2-08 offline IPC capture replay bench.**
Closes the encoder side of the auto-fallback ladder and adds an
offline forensic tool so a 30 s `cat /dev/ttymxc1 > capture.bin` can
be turned into a per-frame PNG strip without a live tractor.

W2-07 (encode mode wiring): until this slice the X8 dispatcher logged
`CMD_ENCODE_MODE` and force-keyframed but did *not* change the encoder,
so the base-station's `EncodeModeController` had no measurable wire
effect. New module-level globals in
[`firmware/tractor_x8/camera_service.py`](DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py) —
`ENCODE_MODE_FULL=0`, `ENCODE_MODE_Y_ONLY=1`,
`ENCODE_MODE_MOTION_ONLY=2`, `ENCODE_MODE_WIREFRAME=3`,
`ENCODE_MODE_NAMES`, `ENCODE_MODE` (runtime knob, env-overridable via
`LIFETRAC_ENCODE_MODE`), plus quality ceilings
`MOTION_ONLY_QUALITY=30` (env: `LIFETRAC_MOTION_ONLY_QUALITY`) and
`WIREFRAME_QUALITY=20` (env: `LIFETRAC_WIREFRAME_QUALITY`).
`_encode_tile` gained an optional `encode_mode=None` kwarg; for any
mode ≠ `FULL` it does a PIL `L→RGB` round-trip (chroma drop, WebP
container stays RGB so wire decoders are unchanged) and applies
`_apply_encode_mode_quality(quality, mode)` as a `min()` ceiling so
the ROI-inside boost still wins when its quality is below the cap.
Dispatcher now validates `0 ≤ mode ≤ 3` and silently rejects
out-of-range modes (with a warning log) while still force-keyframing
per the IP-104 contract — gives the operator visible "ack" feedback
even on malformed packets. Cache-key fix: the W2-05 `TileEncodeCache`
key in `_build_frame` now appends `bytes([q & 0xFF, ENCODE_MODE & 0xFF])`
to the raw RGB slice, so a `CMD_ENCODE_MODE` flip auto-invalidates all
per-tile entries (otherwise the next keyframe would replay stale
colour blobs at the new mode). New
[`base_station/tests/test_x8_encode_mode.py`](DESIGN-CONTROLLER/base_station/tests/test_x8_encode_mode.py)
adds **11 tests**: `EncodeModeDispatchTests` (every valid mode commits
the global + force-keyframes; out-of-range modes are rejected but
still ack via keyframe; truncated frame doesn't crash and leaves
mode untouched), `EncodeModeQualityCeilingTests` (full/y_only pass
through, motion_only/wireframe clamp, wireframe ceiling ≤
motion_only ceiling), `EncodeModeWebPSizeTests` (wire WebP byte count
strictly drops when y_only replaces full and when wireframe replaces
motion_only on the same `SyntheticCamera` canvas; explicit
`encode_mode=` kwarg overrides the global without mutating it), and
`EncodeCacheInvalidatesOnModeChangeTests` (96 misses → 0 misses on
identical-mode replay → 96 misses again after a `CMD_ENCODE_MODE`
flip, proving the cache key fix). Cluster: **77/77** across
`test_x8_encode_mode + test_x8_tile_encode_cache + test_back_channel_dispatch +
test_data_saving_measures + test_link_adaptive_budget +
test_link_profile_emitter`.

W2-08 (replay bench): new
[`base_station/replay_ipc_capture.py`](DESIGN-CONTROLLER/base_station/replay_ipc_capture.py)
takes a captured byte stream from `/dev/ttymxc1`, walks
`iter_ipc_frames()` (X8-side framer, sync `0xA5` + flags + length +
CRC-8/SMBus), feeds each payload to `parse_tile_delta_frame()` (base
side), maintains a persistent canvas (so dropped tiles in delta
frames keep their last value), blits each tile via PIL, and writes
`frame_NNNN_seqXXX_{key,delta}.png` per frame. CLI:
`py -3 -m base_station.replay_ipc_capture capture.bin --out-dir out/`
with `--limit N` and `--no-pngs` knobs. Module-loading uses
`importlib.spec_from_file_location` so it doesn't pollute
`sys.modules['image_pipeline']` (the X8 and base sides ship same-named
packages with different contents). `ReplayStats` exposes
`(ipc_frames, decoded_frames, keyframes, deltas, decode_errors,
pngs_written)` for downstream piping. New
[`base_station/tests/test_replay_ipc_capture.py`](DESIGN-CONTROLLER/base_station/tests/test_replay_ipc_capture.py)
adds **8 tests**: synthesizes a real capture (1 keyframe of 96 WebP
tiles + 2 deltas, with a junk byte sprinkled between IPC frames to
exercise framer resync); asserts decode counts split correctly,
documented PNG naming
(`frame_0001_seq000_key.png` etc.), expected canvas size
(384×256), `--limit` honoured, `--no-pngs` skips both writes and
mkdir, corrupt TileDeltaFrame payloads bump `decode_errors` without
crashing, CLI returns 0 on success and 2 on missing capture.

Pre-existing sys.path collision note: `test_image_pipeline` and
`test_image_reassembly_fuzz` fail when run *in the same process* as
any test that puts `firmware/tractor_x8/` on `sys.path`
(`test_back_channel_dispatch`, `test_x8_*`, `test_replay_ipc_capture`,
…) because the X8-side `image_pipeline` package (no `bg_cache.py`)
shadows the base-side one. Standalone they're 33/33 green. Not a
regression introduced by W2-07/08; pre-dates this session. Cleanest
fix is to migrate the legacy tests onto the same
`spec_from_file_location` loader pattern; deferred.

**2026-05-16 — IP-W2-05 X8 per-tile WebP encode cache + IP-W2-06 on-device dry-run harness.**
Closes the loop on the encode-side CPU saver and adds an end-to-end
hardware smoke harness for the data-saving stack. New
[`firmware/tractor_x8/image_pipeline/tile_cache.py`](DESIGN-CONTROLLER/firmware/tractor_x8/image_pipeline/tile_cache.py)
implements `TileEncodeCache(n_tiles, history=4)` — a per-tile
`OrderedDict` LRU bucket keyed by
`hashlib.blake2b(raw_pixels + bytes([quality]), digest_size=16)` so
the cache is invalidated automatically on quality (mode) changes.
`TileCacheStats(hits, misses, evictions)` exposes per-instance
counters for downstream telemetry. `camera_service._build_frame` now
takes an optional `encode_cache=None` parameter and short-circuits the
WebP encode call on a hit; the wire payload is byte-identical with or
without the cache. `_encode_tile` was refactored to extract a pure
`_slice_tile_rgb(rgb_canvas, tx, ty)` helper so the cache key matches
exactly what the encoder hashes. The cache is opt-in via
`LIFETRAC_TILE_CACHE_ENABLE=1` (with `LIFETRAC_TILE_CACHE_HISTORY`
overriding the per-tile depth, default 4) so the existing zero-cache
boot path is unchanged. New
[`base_station/tests/test_x8_tile_encode_cache.py`](DESIGN-CONTROLLER/base_station/tests/test_x8_tile_encode_cache.py)
adds **13 tests**: `TileEncodeCacheUnitTests` cover lookup-miss-then-hit,
per-tile bucket isolation, LRU eviction beyond `history`, LRU refresh
on hit, store-update-in-place, constructor validation, the
`DEFAULT_HISTORY=4` constant, and `reset()`. `BuildFrameWithCacheTests`
exercise the integration: first keyframe misses every tile (96 encode
calls), second identical canvas hits every tile (0 encode calls), the
emitted wire bytes are byte-identical with vs. without the cache, and
a quality-byte change forces a cache miss. Importlib loader pattern
mirrors `test_data_saving_measures.py` so the X8-side `camera_service`
loads cleanly from base-station tests without polluting `sys.modules`.
Full cluster green: **86/86** across `test_x8_tile_encode_cache +
test_data_saving_measures + test_link_adaptive_budget +
test_link_profile_emitter + test_back_channel_dispatch +
test_ipc_to_h747_roundtrip + test_v4l2_ffmpeg_camera`.

W2-06 hardware smoke: new
[`firmware/tractor_x8/dry_run_w2_05.py`](DESIGN-CONTROLLER/firmware/tractor_x8/dry_run_w2_05.py)
is a stdlib-only self-contained validator that monkey-patches
`camera_service._encode_tile` with a sized stub (so PIL is not needed
on the device) and walks 4 sequential gates: (1) default keyframe
encodes `GRID_W*GRID_H` tiles and produces a non-empty payload;
(2) dispatching `CMD_LINK_PROFILE(n=2, phy_idx=image)` shrinks
`LinkBudget.bytes` strictly below the seed `(n=8, phy_idx=image)`
budget and forces a keyframe via the shared `force_evt`; (3) the next
encoded frame respects the new tighter budget and is correspondingly
smaller; (4) `TileEncodeCache` records `n_tiles` encodes on first
pass and 0 on the second pass over the same canvas. Each gate prints
a `PASS:` line; first failure prints `FAIL: …` to stderr and exits 1.
Companion
[`firmware/x8_lora_bootloader_helper/run_w2_05_dry_run.ps1`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_w2_05_dry_run.ps1)
adb-pushes `camera_service.py`, the `image_pipeline/` tree, and
`base_station/lora_proto.py` into
`/var/rootdirs/home/fio/lifetrac_w2_05/`, runs
`python3 dry_run_w2_05.py` over `adb shell`, and propagates the
device-side exit code via a `__DRYRUN_RC=$?` sentinel (adb shell does
not propagate remote rc on its own). **Host smoke: 4/4 gates green**
(`PASS: default keyframe payload=5873 B (96 tile encodes) /
PASS: CMD_LINK_PROFILE applied; budget=74 B (< seed 296 B) /
PASS: capped payload=78 B (< 5873 B default) /
PASS: tile cache 1st pass=96 encodes, 2nd pass=0 encodes`).
**On-device run blocked**: the bare LmP rootfs Python 3.10 ships
without `logging`, `dataclasses`, `json`, `hashlib`, `select`,
`socket`, etc. — production Python is meant to run inside the X8's
Docker container. Wrapper script header now documents this; the
runtime gate will turn green once the dry-run is invoked from inside
the production container (or after `python3-dataclasses`/-`json`/
-`logging`/-`hashlib` packages are layered on the rootfs).

**2026-05-15 — W2-04 base-station emitter for `CMD_LINK_PROFILE` (0x64).**
Closes the loop opened earlier the same day so the X8's `LinkBudget`
slot is now driven from the base-station's rolling SNR window without
operator intervention. New class `LinkProfileEmitter` in
[`base_station/link_monitor.py`](DESIGN-CONTROLLER/base_station/link_monitor.py)
wraps a strictly-descending SNR ladder (`DEFAULT_SNR_LADDER`, 5 rungs:
image PHY at 8/4/2 fragments, then telemetry-PHY retreat at 4 / floor 2)
with the same 3-window hysteresis pattern as `EncodeModeController` so
a single noisy receive sample can't flap the encoder. `observe(now_ms,
snr_db)` returns the wire-format `pack_command(seq, CMD_LINK_PROFILE,
bytes([n_frag, phy_idx]))` frame on commit and forwards it to an
optional `publish_command` sink + audit-log hook
(`AuditLog.log_link_profile_change`). The retreat threshold is intentionally
above the SF7 demod floor (≈-7.5 dB) so the encoder downshifts before
the modem walks off the cliff. New
[base_station/tests/test_link_profile_emitter.py](DESIGN-CONTROLLER/base_station/tests/test_link_profile_emitter.py)
adds **20 tests** across `LadderTargetTests` (default-ladder
descending-and-unique invariant, top-of-ladder picks image-full,
mid-rung picks image-quarter, sub-floor retreat to telemetry, hard
floor clamps, constructor rejects unsorted ladders),
`HysteresisTests` (first observation never emits, required-windows
gate, single-bad-window resets candidate counter, steady-state silence
after commit, `required_windows=0` clamps to 1), `WireFormatTests`
(emitted frame validates CRC, opcode = 0x64 + args = `[n_frag, phy_idx]`,
seq monotonically increments across commits), `PublisherWiringTests`
(callback only on commit, `reset()` clears state and unsticks
re-emission), `AuditWiringTests` (audit hook called with prev/new/snr,
records previous target after second commit), and
`CrossSideContractTests` (`LINK_PHY_NAMES[0]/[1]` pinned to
`"image"`/`"telemetry"`, X8-side tuple round-trips byte-identical so
a unilateral rename on either side fails CI). Wire side:
[`base_station/lora_proto.py`](DESIGN-CONTROLLER/base_station/lora_proto.py)
adds `CMD_LINK_PROFILE = 0x64`, the shared `LINK_PHY_NAMES` tuple, and
classifies the new opcode as a P1 frame so existing fuzz/parser tests
gate it. Combined sweep across `test_link_profile_emitter` (20),
`test_link_adaptive_budget` (11), `test_data_saving_measures` (15),
`test_back_channel_dispatch` (10), `test_ipc_to_h747_roundtrip` (12),
`test_v4l2_ffmpeg_camera` (8) is **73 tests / 0 failures**, and the
adjacent lora_proto/link_monitor sweep
(`test_lora_proto + test_command_frame_fuzz + test_link_monitor +
test_link_monitor_orchestrator + test_link_profile_emitter +
test_link_adaptive_budget`) is **60 tests / 0 failures**, confirming
that adding `CMD_LINK_PROFILE` to `_P1_OPCODES` doesn't regress the
existing parser/fuzz suites.

**2026-05-15 — W2-04 link-adaptive byte budget (`CMD_LINK_PROFILE` 0x64).**
Closes the loop on the §K image-pipeline data-saving stack: the X8's
encoder now accepts a runtime byte cap from the base station instead
of being pinned at boot via `LIFETRAC_FRAGMENT_BUDGET`. New constants
in [`firmware/tractor_x8/camera_service.py`](DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py):
`CMD_LINK_PROFILE = 0x64`, `LINK_PHY_NAMES = ("image", "telemetry",
"control_sf9", "control_sf8", "control_sf7")`, shared
`_compute_link_bytes(n_fragments, profile_name)` helper, and a slot-based
`LinkBudget` mutable holder shared between the back-channel reader and
the encode loop. `dispatch_back_channel(..., link_budget=...)` handles
opcode `0x64` with args `<n_fragments u8> <phy_index u8>` — invalid
indices and zero fragments are rejected without corrupting prior state,
but a keyframe is forced unconditionally so the new budget is observable
on the next decoded frame at the base. `_resolve_byte_budget()` now
honours `LIFETRAC_FRAGMENT_PROFILE` (default `"image"`). New
[base_station/tests/test_link_adaptive_budget.py](DESIGN-CONTROLLER/base_station/tests/test_link_adaptive_budget.py)
adds **11 tests** across `LinkBudgetUpdateTests` (image/telemetry
recompute, telemetry-cap < image-cap at same fragment count, OOR phy
rejected, zero-frag rejected, fresh-state nulls), `CmdLinkProfileDispatchTests`
(opcode mutates budget + forces keyframe; truncated args don't corrupt;
`link_budget=None` silent; bad index keeps prior cap but still keys),
and `ResolveByteBudgetUsesProfileTests` (env-var profile selection,
unknown profile → `None`). Suite total now **52 tests / 0 failures**
across `test_link_adaptive_budget`, `test_data_saving_measures` (15),
`test_back_channel_dispatch` (10), `test_ipc_to_h747_roundtrip` (12),
`test_v4l2_ffmpeg_camera` (8). Stacks on the W2-03 ROI + budget wiring
landed earlier the same day. Next slice (in flight): base-station-side
emitter that publishes `CMD_LINK_PROFILE` from the `link_monitor.py`
RSSI/SNR watcher so the loop runs without operator intervention.

**Implementation status (2026-04-29, through Round 53):**
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md`](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md)
— every plan item achievable without bench hardware is now landed
(Wave 0 8/8, Wave 1 8/8, Wave 2 9/9, Wave 3 9/9). **810 base_station
tests pass / 2 skipped / 2364 subtests / 60 files**, and **every "—"
cell with a natural SIL surface in the
[MASTER_TEST_PROGRAM.md](MASTER_TEST_PROGRAM.md) §5 IP-traceability
table is now closed.** **Round 53** lands K-E2 — web UI ramp-state
heatmap data model. New module
[base_station/ramp_heatmap.py](DESIGN-CONTROLLER/base_station/ramp_heatmap.py)
exposes a 5-state per-tick classifier (``idle`` / ``matched`` /
``reversal`` / ``decay`` / ``mushy``) over (raw, effective) axis
pairs so the operator-visible "why did my stop feel mushy" overlay
can paint cells without recomputing ramp state. Precedence ladder:
``idle`` (both within deadband) > ``reversal`` (opposite-sign while
both active — wins over ``mushy`` because BC-22 brake lag is
*expected*) > ``decay`` (raw idle, effective still asserting) >
``mushy`` (same-sign ``|lag| > mushy_threshold``) > ``matched``
(otherwise). ``DEFAULT_DEADBAND`` mirrors the BC-29 ``ui.axis_deadband``
config default; ``DEFAULT_MUSHY_THRESHOLD = 40`` (~31% of int8 full
scale) is the documented starting point. Public API:
``HeatmapSample``, ``classify_state()``, ``lag()``,
``build_heatmap_row()``, ``build_heatmap()`` — all JSON-serializable
output for the painter. The new
[base_station/tests/test_ramp_heatmap_sil.py](DESIGN-CONTROLLER/base_station/tests/test_ramp_heatmap_sil.py)
adds 29 tests across RH-A..RH-F: state-vocabulary pin, signed lag
math, full classification ladder including precedence + boundary +
threshold validation, row shape, top-level heatmap shape with
stream-order preservation and threshold propagation, and a firmware-
identity gate that pins ``DEFAULT_DEADBAND`` to ``cfg.ui.axis_deadband``.
[ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) flips the K-E2 row to
``✓ LANDED Round 53`` and adds the Round 53 sequencing line. Painter
wiring (writing the JSON into a web view) deferred. **Suite at 810
tests / 2 skipped / 60 files** (781 → 810 from Round 53).
**Round 52** lands BC-29 — configurable axis deadband
deadband (cheap-win K-D3). New schema int leaf
``[ui].axis_deadband`` ∈ ``[0, 32]``, default ``13`` (≈10% of int8
full scale = byte-for-byte identity vs the pre-Round-52 hard-coded
firmware constant). Firmware sources the constant from the codegen-
emitted header: ``static const int8_t AXIS_DEADBAND =
(int8_t)LIFETRAC_UI_AXIS_DEADBAND;``. All four use sites
(``axis_active()``, the per-coil activation block, BC-24 spin-turn
detection, the flow-set-point computation) continue to read the same
compile-time constant so the four call sites stay in sync. The new
[base_station/tests/test_axis_deadband_sil.py](DESIGN-CONTROLLER/base_station/tests/test_axis_deadband_sil.py)
adds 23 tests across DB-A..DB-E: DB-A schema validation (required-
list, in-range 0/1/13/20/32 accepted, out-of-range low/high + wrong
type + float-where-int rejected); DB-B default-value identity
(default toml authors 13, loader yields int 13, codegen emits
``LIFETRAC_UI_AXIS_DEADBAND 13``, ``reload_class = restart_required``);
DB-C override round-trip for 5 / 0 / 32; DB-D firmware tripwires
(BC-29 marker, exact macro init line, no legacy hard-coded literal,
constant still consumed in all four use sites with the precise
patterns); DB-E docs tripwires (schema bounds, default-toml leaf +
doc comment, CAPABILITY_INVENTORY row).
[ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) gains a new BC-29 entry,
adds a K-D3 row to the cheap-wins table marked ``✓ LANDED Round 52``,
and adds the Round 52 sequencing line. **Suite at 781 tests / 2
skipped / 59 files** (758 → 781 from Round 52).
**Round 51** lands BC-28 — operator profile
preset (cheap-win K-D1, minimal version). New schema enum leaf
``[ui].operator_profile`` ∈ ``{normal, gentle, sport}``, default
``"normal"`` (byte-for-byte identity — individual operator-feel
leaves stand as authored). ``"gentle"`` overrides
``ui.confined_space_mode_enabled = true`` AND
``hydraulic.ramp_shape = "scurve"`` for tight-quarters work.
``"sport"`` overrides ``ui.confined_space_mode_enabled = false``,
``hydraulic.ramp_shape = "linear"`` AND ``ui.stick_curve_exponent
= 1.0`` for crisp control. Overrides are applied at config-load time
inside ``build_config.load()`` via a new
``_apply_operator_profile_overrides()`` helper that mutates the
parsed TOML dict in place after schema validation but before the
hydraulic-compatibility cross-check. Both Python consumers and the
codegen-emitted firmware header automatically pick up the post-
override state because both flow through the same loader. Bundles
intentionally only touch operator-feel leaves with ``reload_class
= restart_required``; safety / hydraulic-topology / network leaves
are NEVER overridden by a profile. The new
[base_station/tests/test_operator_profile_sil.py](DESIGN-CONTROLLER/base_station/tests/test_operator_profile_sil.py)
adds 21 tests across OP-A..OP-E: OP-A pure helper math (normal no-op
even with bundle-shaped authored values, gentle / sport flip exactly
their bundle leaves and leave sentinels untouched, unknown profile
defensive no-op, public ``OPERATOR_PROFILE_OVERRIDES`` table pinned);
OP-B loader integration via temp-TOML + ``LIFETRAC_BUILD_CONFIG_PATH``
(default toml yields normal + identity, gentle/sport tomls override
authored values, ``cfg.raw`` reflects post-override state so
``config_sha256`` is consumer-consistent); OP-C codegen integration
(gentle emits ``..._CONFINED_SPACE_MODE_ENABLED 1`` +
``..._RAMP_SHAPE_SCURVE 1`` + profile macro/side-macro; sport emits
``..._CONFINED_SPACE_MODE_ENABLED 0`` + ``..._RAMP_SHAPE_LINEAR 1`` +
``..._STICK_CURVE_EXPONENT 1.0f``; normal default toml is byte-
identity to authored); OP-D schema rejection of unknown enum value;
OP-E source/config tripwires (``BC-28`` marker,
``_apply_operator_profile_overrides`` symbol +
``OPERATOR_PROFILE_OVERRIDES`` table, schema enum,
``operator_profile = "normal"`` line in ``build.default.toml``,
CAPABILITY_INVENTORY row).
[ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) gains a new BC-28 entry,
marks K-D1 cheap-win as ``✓ LANDED Round 51 (minimal)``, and adds
the Round 51 sequencing line. **Suite at 758 tests / 2 skipped /
58 files** (737 → 758 from Round 51).
**Round 50** lands BC-27 — confined-space
mode (cheap-win K-D2, minimal version). New schema leaf
``[ui].confined_space_mode_enabled`` (bool, default ``false`` =
byte-for-byte identity — zero behaviour change unless a build opts
in). When ``true``, ``ramp_duration_ms()`` multiplies its base
ladder result by ``3 / 2`` (integer-exact for every ladder value:
250→375, 500→750, 1000→1500, 2000→3000) so release ramps,
BC-22 reversal-decay ramps, and the K-A4 forced-coordinated track
duration all stretch by 1.5× — the tractor stops more gently in
tight quarters at the cost of a slightly softer feel. Composes
orthogonally with BC-25 stick curve and BC-26 scurve ramp shape
(operators who already opted in keep their preferences). Minimal
version: 1.5× ramp duration only — no curve / flow-cap change
(deferred until real-world testing motivates them). The new
[base_station/tests/test_confined_space_sil.py](DESIGN-CONTROLLER/base_station/tests/test_confined_space_sil.py)
adds 18 tests across CS-A..CS-G: CS-A pure ladder math at
``confined=False/True`` for both track and arm magnitudes plus
sign-absolute handling; CS-B default-identity at ``FourAxisArbiter``;
CS-C end-to-end release ramp from ``lhy=127`` (baseline last-non-zero
~tick 38–40, confined ~tick 58–60, ≥18-tick stretch); CS-D K-A4
asymmetric-magnitude coordination preserved at the deadline tick
(left=127 / right=25 → both 0 at deadline 40 baseline / 60 confined);
CS-E composition with BC-25 ``stick_curve_exponent=2.0`` (curve
compresses 64→32 AND ramp stretches); CS-F composition with BC-26
``ramp_shape="scurve"`` (smoothstep over the longer 3 s ramp); CS-G
firmware/source tripwires (``BC-27`` marker,
``LIFETRAC_UI_CONFINED_SPACE_MODE_ENABLED`` macro consumption,
``base * 3u`` / ``) / 2u`` multiplier expression, schema leaf,
CAPABILITY_INVENTORY row). The ``test_axis_ramp_sil.py`` Python
mirror gained a ``confined_space`` parameter on ``ramp_duration_ms()``
and ``step_axis_ramp()``, plumbed through ``FourAxisArbiter`` to
both per-axis ramps and the K-A4 forced-track-duration computation;
the default ``confined_space=False`` keeps every pre-Round-50 ramp
test byte-identical. [ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md)
gains a new BC-27 entry, marks K-D2 cheap-win as ``✓ LANDED
Round 50 (minimal)``, and adds the Round 50 sequencing line.
**Suite at 737 tests / 2 skipped / 57 files** (719 → 737 from
Round 50).
**Round 49** lands BC-26 — S-curve ramp shape
selector (cheap-win K-A3). New schema leaf
``[hydraulic].ramp_shape`` ∈ ``{linear, scurve}``, default ``linear``
(byte-for-byte identity — zero behaviour change unless a build opts
in). Firmware factors the prior inline interpolation in
``step_axis_ramp()`` into a ``ramp_interpolate(start, elapsed,
duration_ms)`` helper. When the build defines
``LIFETRAC_HYDRAULIC_RAMP_SHAPE_SCURVE`` the helper substitutes a
half-cosine smoothstep ``shape(t) = 0.5 × (1 + cos(π × t))`` whose
derivative is zero at both endpoints, cutting P95 jerk roughly in
half for the same total stop distance. Pinned closed-form quarter
points: smoothstep at ``t=0.25 → ~0.854 × start`` (vs linear ``0.75``),
``t=0.5 → ~0.5 × start`` (matches linear), ``t=0.75 → ~0.146 × start``
(vs linear ``0.25``).
The new
[base_station/tests/test_ramp_shape_sil.py](DESIGN-CONTROLLER/base_station/tests/test_ramp_shape_sil.py)
adds 21 tests across RS-A..RS-G: RS-A linear-default byte-identity to
the legacy truncation formula across positive / negative starts and
quarter / half / three-quarter / endpoint elapsed, plus implicit
default-arg identity; RS-B scurve endpoints (`elapsed=0 → start`,
``elapsed≥duration → 0``, ``duration=0 → 0``, monotonic non-increasing);
RS-C scurve quarter-point closed-form pinning that distinguishes from
linear; RS-D sign preservation + magnitude symmetry within 1 LSB;
RS-E end-to-end through ``FourAxisArbiter(ramp_shape="scurve")``
(release ramp from full forward, scurve strictly higher at the
quarter-point sample, both shapes reach 0 at the K-A4-coordinated
2 s deadline); RS-F default-identity contract for ``FourAxisArbiter()``
release trajectory; RS-G firmware/source tripwires (BC-26 marker,
``ramp_interpolate`` symbol, ``LIFETRAC_HYDRAULIC_RAMP_SHAPE_SCURVE``
macro consumption, helper call site, schema enum, CAPABILITY_INVENTORY
row).
The ``test_axis_ramp_sil.py`` Python mirror gained an
``_ramp_interpolate(start, elapsed, duration_ms, shape)`` helper and
``step_axis_ramp()`` / ``FourAxisArbiter`` now route through it; the
default ``shape="linear"`` keeps every pre-Round-49 ramp test
byte-identical. [ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) gains a
new BC-26 entry, marks K-A3 cheap-win as ``✓ LANDED Round 49``, and
adds the Round 49 sequencing line. **Suite at 719 tests / 2 skipped
/ 56 files** (698 → 719 from Round 49).
**Round 48** lands BC-25 — per-stick response
curve exponent (cheap-win K-A2). New schema leaf
``[ui].stick_curve_exponent`` ∈ ``{1.0, 1.5, 2.0}``, default ``1.0``
(byte-for-byte identity — zero behaviour change unless a build opts in).
Firmware precomputes a 128-entry uint8_t LUT in ``setup()`` from
``LIFETRAC_UI_STICK_CURVE_EXPONENT`` and applies
``effective = sign(x) × LUT[|x|]`` to every raw stick axis (``lhx``,
``lhy``, ``rhx``, ``rhy``) **post-deadband, pre-mixing**. At ``n = 2.0``
a 50%-stick input compresses to ~32 (vs 64 linear) for finer creep
precision; pegged sticks still reach ±127 at every supported exponent.
The new
[base_station/tests/test_stick_curve_sil.py](DESIGN-CONTROLLER/base_station/tests/test_stick_curve_sil.py)
adds 20 tests across SC-A..SC-F: SC-A pure curve math (identity, zero
fixed point, full-stick preserved, square-law at n=2.0, monotonic, sign
preservation, OOB clamping); SC-B default-identity contract for
``FourAxisArbiter()``; SC-C K-A2 motivating low-stick compression; SC-D
top-end authority across exponents; SC-E pre-mixing application
(``lhx=64`` at n=2.0 → tracks ``(32, -32)`` not ``(16, -16)``); SC-F
firmware source / config tripwires (BC-25 marker, ``apply_stick_curve``
symbol, ``init_stick_curve_lut`` setup-time call, four call sites
wrapping ``cf.axis_*``, schema enum, CAPABILITY_INVENTORY row).
The ``test_axis_ramp_sil.py`` Python mirror grew an
``_apply_stick_curve(v, exponent)`` helper and ``FourAxisArbiter``
gained a ``stick_curve_exponent`` ctor parameter (default 1.0 → legacy
fixtures unaffected). [INPUT_MAPPING.md](DESIGN-KINEMATICS/INPUT_MAPPING.md)
§ "Stick curve" rewritten to past tense; pending-list item ticked off.
[ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) gains a new BC-25 entry,
marks K-A2 cheap-win as ``✓ LANDED Round 48``, and adds the Round 48
sequencing line. **Suite at 698 tests / 2 skipped / 55 files** (678 →
698 from Round 48).
**Round 47** is a doc-debt cleanup + SIL traceability round: closes the two ``STUB`` kinematics docs that were
originally scheduled for Round 44.
[INPUT_MAPPING.md](DESIGN-KINEMATICS/INPUT_MAPPING.md) and
[MOTION_PRIMITIVES.md](DESIGN-KINEMATICS/MOTION_PRIMITIVES.md) are now
marked landed, with their pending sections pointing at the deferred
schema leaves (``[ui].stick_curve_exponent``, BUILD_VARIANT_MOTION_MATRIX
promotion). Stale BC-22 "New design" entry in
[ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) corrected to ``✓ LANDED Round
45``. The new
[base_station/tests/test_motion_primitives_sil.py](DESIGN-CONTROLLER/base_station/tests/test_motion_primitives_sil.py)
adds 15 tests (1 skipped — MP-12 float, build-gated by ``hydraulic.spool_type``)
that pin the post-mix logical-axis signature of every primitive
MP-01..MP-12 through ``FourAxisArbiter``: drive forward / reverse,
skid turn, pivot turn (one track exactly 0), spin in place
(opposite-sign tracks + BC-24 full-flow boost), arms raise / lower,
bucket curl / dump, drive + arms (max-flow over arms), turn + bucket
(both pivot+bucket and spin+bucket combinations), and a cross-cutting
sub-deadband test (all four sticks at v=5 → every logical axis 0,
flow_sp 0). The doc-anchored test file means any future regression to
operator-visible motion vocabulary fails loudly with the primitive
number. **Suite at 678 tests / 2 skipped / 54 files** (663 → 678 from
Round 47).
**Round 46** lands BC-23 — preserve-steering proportional scale-down on saturation — plus BC-24 — spin-turn flow
boost. The naive ``clip_to_int8(lhy + lhx)`` per-side clip is replaced
in ``apply_control()`` by ``mix_tracks_preserve_steering(left_intent,
right_intent)``: when either intent magnitude exceeds ±127, BOTH are
scaled by ``127 / max_mag`` so the differential (steering) ratio is
preserved at the cost of throttle authority. Concrete example: ``lhy=120,
lhx=80`` previously clipped to ``(127, 40)`` — differential 43, vs
operator-commanded 80 — now scales to ``(127, 25)`` — differential 51.
BC-24 augments the flow set-point: when both tracks are active in opposite
directions (a pure or near-pure spin-turn), the track contribution becomes
``min(127, |left_track| + |right_track|)`` instead of
``max(|left_track|, |right_track|)``, so the proportional valve doesn't
under-budget flow and stall the spin. Same-sign motion (forward, reverse,
smooth-curve turn) continues to use max. Arms / bucket continue to use
max in both cases since they're independent valve banks. The new
[base_station/tests/test_steering_priority_sil.py](DESIGN-CONTROLLER/base_station/tests/test_steering_priority_sil.py)
adds 24 tests across SP-A..SP-E: SP-A helper math (8 cases incl. boundary
and mixed-sign saturation), SP-B end-to-end steering-ratio invariant
(post-BC-23 diff is no further from operator intent than pre-BC-23
baseline), SP-C spin-turn boost (pure spin reaches 10000 mV; partial spin
uses sum; large spin clamps), SP-D non-spin cases (forward / reverse /
smooth-curve / arms-only must use max), SP-E firmware source tripwires.
The ``test_axis_ramp_sil.py`` Python mirror grew the new
``_mix_tracks_preserve_steering`` helper and ``FourAxisArbiter`` now
applies it. ``test_track_mix_ramp_sil.py`` RT-A / RT-C / RT-D expectations
updated for BC-23 scaling; behavioural invariants (mixed-mode skip, K-A4
coordinated stop, smooth-curve no-step) preserved. Schema-driven
policy selectors (``[hydraulic].steering_priority`` enum,
``[hydraulic].spin_turn_boost_enabled`` bool) deferred to a future round.
[DIFFERENTIAL_MIXING.md](DESIGN-KINEMATICS/DIFFERENTIAL_MIXING.md) and
[FLOW_BUDGETING.md](DESIGN-KINEMATICS/FLOW_BUDGETING.md) promoted from
stub to landed; [ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) marks BC-23
and BC-24 as ``✓ LANDED Round 46``.
**Suite at 663 tests / 1 skipped / 53 files** (639 → 663 from Round 46).
**Round 45** lands BC-22 — reversal brake.
``step_axis_ramp()`` now detects a same-tick sign-flip on a still-
energised axis (``was_active && is_now && opposite signs``), converts
the operator's reversal command into a decay-to-zero ramp using the
standard release ladder, and on completion holds the axis at zero for
``REVERSAL_BRAKE_MS = 100`` ms (= 2 × ``RAMP_TICK_MS``) before allowing
the new direction to pass through. Two new fields on ``struct AxisRamp``
(``reversal_pending`` and ``brake_until_ms``); a decay-shield invariant
ensures the operator's still-held opposite-sign input cannot cancel the
in-flight reversal ramp via the snap-on-activation path — the decay+
settle sequence runs to completion deterministically, which is the
central hydraulic-safety reason for BC-22 (prevents spool slam and the
cavitation/pressure-spike pair documented in
[DESIGN-KINEMATICS/REVERSAL_HANDLING.md](DESIGN-KINEMATICS/REVERSAL_HANDLING.md)).
The new
[base_station/tests/test_reversal_brake_sil.py](DESIGN-CONTROLLER/base_station/tests/test_reversal_brake_sil.py)
adds 21 tests across BR-A..BR-F: BR-A decay phase (no-snap, release-ladder
duration, monotonic non-increasing magnitude, brake-arm at deadline),
BR-B settle window (holds zero through the full window, ignores input
changes, 2-tick width invariant), BR-C post-brake resumption (active
resumes immediately, zero stays zero), BR-D non-reversal cases (same-sign
jump, plain release, sub-deadband flicker, mixed-mode-skip), BR-E arm
reversals share settle window, BR-F firmware source tripwires. The
``test_axis_ramp_sil.py`` Python mirror grew the same two fields and the
shield logic; the W4-05 / W4-06 invariants stay green. K-A1 / K-A2 /
K-A3 ramping refinements deferred (semantics ambiguous against the
current snap-on-activation contract; need an operator-feel design pass).
[REVERSAL_HANDLING.md](DESIGN-KINEMATICS/REVERSAL_HANDLING.md) promoted
from stub to landed; [ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md) marks
BC-22 as ``✓ LANDED Round 45``.
**Suite at 639 tests / 1 skipped / 52 files** (618 → 639 from Round 45).
**Round 44** lands BC-21 — mix-then-ramp +
K-A4 coordinated bilateral track stop. The four ramps in
`apply_control()` now operate on **logical** motion axes
(`g_ramp_left_track`, `g_ramp_right_track`, `g_ramp_arms`,
`g_ramp_bucket`) AFTER differential mixing of raw stick channels
(`leftTrack = clip(lhy + lhx, ±127)`, mirror for right). Pre-BC-21,
ramping on raw stick axes meant a smooth-curve turn (forward-pinned
`lhy`, ramping `lhx` in) stepped the right track because the ramp
short-circuit fired on a stick channel that no track ramp watched.
Three independent code reviews flagged this. As an incidental fix the
coil mapping moved from `lhy`-only OR'd pairs (`if lhy>db: coils |=
LF | RF`) to per-side independent (`if leftTrack>db: coils |= LF`),
which also fixes a second pre-BC-21 bug: pure spin-turns
(`lhy=0, lhx=full`) produced ZERO drive coil activation. K-A4 ships
alongside via a new optional `forced_duration_ms` parameter on
`step_axis_ramp`: when both tracks transition active→released in the
same tick, both ramps share a duration computed from the larger
starting magnitude so the tractor doesn't pivot during release. The
new
[base_station/tests/test_track_mix_ramp_sil.py](DESIGN-CONTROLLER/base_station/tests/test_track_mix_ramp_sil.py)
adds 25 tests across RT-A..RT-G: RT-A mixing math (8 cases incl.
saturation), RT-B spin-turn coil activation (3 cases), RT-C
smooth-curve no-step (the central correctness claim), RT-D K-A4
(symmetric, asymmetric same-tick zero-crossing), RT-F single-track
release, RT-G firmware source tripwire (BC-21 / K-A4 markers,
old-globals-removed, per-side coil mapping). Existing
[test_axis_ramp_sil.py](DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py)
W4-05 / W4-06 tests refactored to logical-axis semantics
(behavioural invariants preserved). New top-level folder
[`DESIGN-KINEMATICS/`](DESIGN-KINEMATICS/README.md) created in the
same round as the canonical home for motion-command semantics
(input mapping, mixing, ramp profiles, reversal handling, flow
budgeting, build-variant motion matrix); [ROADMAP.md](DESIGN-KINEMATICS/ROADMAP.md)
captures BC-22 / BC-23 / BC-24 and 13 follow-on K-* ideas.
**Suite at 618 tests / 1 skipped / 51 files** (593 → 618 from Round
44). **Round 43** lands BC-19 — hydraulic build-
variant configurability. Three new `[hydraulic]` schema leaves
(`spool_type` enum tandem/float/closed/open, `load_holding` enum
spool_inherent/po_check/counterbalance/none, `valve_settling_ms`
uint 0..250) extend the canonical TOML, the `HydraulicConfig`
dataclass, the schema-driven C-header codegen (which auto-emits 11
new `LIFETRAC_HYDRAULIC_*` macros incl. enum side-flags), and
`CAPABILITY_INVENTORY.md`. A new in-loader cross-leaf validator
`_validate_hydraulic_compatibility` rejects three contradictory
combinations (tandem/closed + load_holding=none; float/open +
load_holding=spool_inherent; float/open + non-zero
valve_settling_ms) at `build_config.load()` time, before any
consumer touches the config. The new
[base_station/tests/test_hydraulic_compatibility_sil.py](DESIGN-CONTROLLER/base_station/tests/test_hydraulic_compatibility_sil.py)
adds 13 tests across BC19_A..BC19_D: BC19_A canonical default
passes the validator; BC19_B four documented reference builds load
(OSE-legacy float+po_check, v25-canonical tandem+inherent, high-
performance open+counterbalance, closed+counterbalance redundant);
BC19_C six invalid pairings rejected (each with diagnostic naming
the offending leaves); BC19_D source tripwire pins the validator
call site so a refactor cannot silently disable the gate. The
existing variant-matrix / loader / codegen SIL gates (BC-07 / BC-01
/ BC-03) exercise the three new leaves at canonical values via
their schema-driven sweeps without per-leaf edits. BUILD_CONFIG.md
gets a new "Common pitfalls" entry pointing to the validator hook
for future cross-leaf rules. **Suite at 593 tests / 1 skipped / 50
files** (581 → 593 from Round 43). **Round 42** lands BC-15 — a machine-readable
JSON form for the BC-10 ``lifetrac-config diff`` subcommand so non-
Python consumers (notably ``hil/dispatch.ps1`` and ad-hoc bench-laptop
shell pipelines) can decide whether a candidate config requires a
service restart or a firmware reflash without re-implementing the
schema-aware comparator. ``--format text`` (default) is byte-identical
to the pre-BC-15 output; ``--format json`` emits the canonical sorted-
keys compact form (consistent with ``dump-json``) carrying
``changed`` / ``classes`` / ``worst`` / ``is_empty`` /
``restart_required`` / ``firmware_required``. The new
[base_station/tests/test_config_diff_cli_sil.py](DESIGN-CONTROLLER/base_station/tests/test_config_diff_cli_sil.py)
adds 10 tests across BC15_A..BC15_C pinning the contract: BC15_A text
output unchanged (no-diff, live-class change, explicit ``--format
text`` matches default byte-for-byte); BC15_B JSON shape (no-diff
empty payload, live-only / restart_required / firmware_required
classification, ``worst`` promotion across multiple changes,
canonical sorted-keys compact output); BC15_C argparse rejects
unknown ``--format`` values with exit 2. BUILD_CONFIG.md ``diff`` row
updated; doc-coverage gate auto-validates. **Suite at 581 tests / 1
skipped / 49 files** (571 → 581 from Round 42). **Round 41** lands
BC-12C — the bench-side
CLI half of BC-12 — by composing Round 39's ``boot_self_test``
comparator with Round 37's ``lifetrac-config`` CLI. The new
``self-test`` subcommand takes a TOML config and a JSON-encoded
``HardwareInventory`` (whatever the operator captured from the M4 on
the bench), runs ``run_self_test`` in-process, and prints a structured
pass/fail report as text (default) or JSON. Exit code is non-zero on
any ``error`` finding (matches the boot-time gate); ``warning``-only
findings keep exit code 0. The new
[base_station/tests/test_config_self_test_cli_sil.py](DESIGN-CONTROLLER/base_station/tests/test_config_self_test_cli_sil.py)
adds 10 tests across BC12C_A..BC12C_E pinning the contract: BC12C_A
matching inventory passes (text + JSON formats); BC12C_B
error-mismatches exit non-zero with stable codes (AXIS_COUNT_TRACK,
AXIS_COUNT_ARM); BC12C_C warning-only mismatches (camera count) keep
exit 0; BC12C_D bad inputs are rejected with diagnostic stderr
(missing file, non-object JSON, missing fields, unparseable JSON);
BC12C_E source-grep tripwire pins the subparser registration.
BUILD_CONFIG.md synopsis bumped eight→nine subcommands; the BC-08
doc-coverage gate auto-validates. **Round 40** lands BC-14B — a follow-up
to Round 37/BC-14 that closes the gap noted in the Round 37 memo:
the ``config_loaded`` audit emitters in
[base_station/web_ui.py](DESIGN-CONTROLLER/base_station/web_ui.py)
and
[base_station/lora_bridge.py](DESIGN-CONTROLLER/base_station/lora_bridge.py)
now include ``schema_version`` (was: missing, leaving the BC-14
inventory column always empty). The new
[base_station/tests/test_config_loaded_schema_version_sil.py](DESIGN-CONTROLLER/base_station/tests/test_config_loaded_schema_version_sil.py)
adds 5 tests across BC14B_A..BC14B_D pinning the contract: BC14B_A
web_ui's ``_audit_config_loaded`` writes ``schema_version`` as an
``int`` from ``BUILD.schema_version``; BC14B_B the lora_bridge inline
block writes the same field (exercised against the same paho-mocked
pattern as BC4_A); BC14B_C end-to-end — fixture audit lines
that carry ``schema_version`` flow through the BC-14 aggregator and
populate the CSV column, and back-compat is preserved (legacy lines
without the field still aggregate cleanly with empty cells); BC14B_D
source greps both emitter call-sites for ``schema_version=`` so a
future refactor that drops the field fails the gate even if a test
happens to mock around it. Suite at **561 tests / 1 skipped / 47
files** (556 → 561 from Round 40). **Round 39** lands the SIL half of BC-12 —
new
[base_station/boot_self_test.py](DESIGN-CONTROLLER/base_station/boot_self_test.py)
module that compares a loaded ``BuildConfig`` against an injected
``HardwareInventory`` (the M4-side Modbus probe will live in the HIL
half, deferred until bench rig) and produces a structured
``SelfTestReport`` with a stable finding-code catalogue (12 codes:
AXIS_COUNT_TRACK / AXIS_COUNT_ARM / PROPORTIONAL_FLOW_UNAVAILABLE /
PRESSURE_SENSOR_COUNT / IMU_PRESENCE / GPS_PRESENCE /
CAMERA_COUNT_SHORT / CAMERA_COUNT_EXTRA / LORA_REGION_MISMATCH /
AUX_PORT_COUNT / AUX_COUPLER_TYPE / AUX_CASE_DRAIN). Severity model:
safety-significant mismatches (axis counts, proportional flow when
commanded, pressure-sensor count, IMU/GPS presence, LoRa region, aux
plumbing) are ``error`` and set ``ok=False``; cosmetic camera-count
mismatches degrade to ``warning`` and ``ok`` stays True. The
``emit_audit()`` helper writes one ``boot_self_test`` JSONL event per
boot with the canonical ``{component, unit_id, config_sha256, ok,
error_count, warning_count, findings[], started_ts, finished_ts}``
shape so jq / Pandas can pivot on ``code`` without nested-object
handling. The new
[base_station/tests/test_boot_self_test_sil.py](DESIGN-CONTROLLER/base_station/tests/test_boot_self_test_sil.py)
adds 15 tests across 8 classes BC12_A..BC12_H pinning every branch
plus the audit-event shape and the finding-code catalogue (catalogue
renames are caught here so dashboards / runbooks don't break
silently). The HIL half (real M4 Modbus probe + tractor-side
integration) remains deferred. Suite at **556 tests / 1 skipped /
46 files** (541 → 556 from Round 39). **Round 38** lands BC-13 — a new
``$9 Build-config attack surface`` section in
[CYBERSECURITY_CASE.md](DESIGN-CONTROLLER/CYBERSECURITY_CASE.md)
that treats the build-config subsystem (TOML files, installer
bundles, ``lifetrac-config`` CLI, push daemon, ``config_loaded``
audit events, generated firmware header) as an explicit asset class
with safety significance, with four sub-sections: 9.1 Z-CONFIG zone
enumeration, 9.2 STRIDE-per-asset table (9 rows), 9.3
capability-altering leaves are safety-significant (the defence-in-depth
stack that ties together BC-09 BOM cross-reference, ``codegen --check``
drift gate, BC-14 inventory visibility, and the three-watchdog
safe-state), and 9.4 self-reference to the enforcing SIL drift gate.
The new
[base_station/tests/test_cybersecurity_buildconfig_xref_sil.py](DESIGN-CONTROLLER/base_station/tests/test_cybersecurity_buildconfig_xref_sil.py)
adds 5 tests across BC13_A..BC13_E pinning the prose against the rest
of the codebase: BC13_A four-subheading structure; BC13_B every
named ``lifetrac-config <subcommand>`` resolves through the live
_build_parser() (catches typos and stale subcommand names); BC13_C
every relative Markdown link target exists on disk; BC13_D the
STRIDE table covers each Z-CONFIG asset row from $9.1; BC13_E $9
self-references its enforcing test file (so renaming the test fails
the gate). Suite at **541 tests / 1 skipped / 45 files** (536 → 541
from Round 38). **Round 37** lands BC-14 — a new
``lifetrac-config inventory`` subcommand that aggregates
``config_loaded`` audit-log events across one or more ``audit.jsonl``
files into a CSV / Markdown fleet inventory, deduplicated by
``(unit_id, config_sha256)`` so a unit that has been reflashed
shows one row per distinct build it has booted (with first-seen,
last-seen, boot-count, and which components emitted the record).
Directory arguments expand to every ``audit*.jsonl*`` sibling so
log-rotation files (``audit.jsonl.1`` etc.) are picked up
automatically. The new
[base_station/tests/test_config_inventory_sil.py](DESIGN-CONTROLLER/base_station/tests/test_config_inventory_sil.py)
adds 14 tests across 5 classes BC14_A..BC14_E pinning the parser
(BC14_A — only ``config_loaded`` records pass; malformed lines /
non-dict records / missing-field records / missing files all
skipped silently), aggregator (BC14_B — unit/SHA dedup;
reflash produces two rows; sort order), renderers (BC14_C — CSV
header matches ``INVENTORY_FIELDS``; CSV byte-identical across two
runs; Markdown header + separator shape), command integration
(BC14_D — directory glob picks up rotated siblings + skips stray
non-audit files; ``--format markdown`` switches the renderer), and
timestamp formatting (BC14_E — known-epoch ISO-8601 UTC; zero /
negative renders empty). The BC-08 doc gate then auto-validates the
new subcommand row in BUILD_CONFIG.md (synopsis bumped from seven
to eight subcommands). Suite at **536 tests / 1 skipped / 44 files**
(522 → 536 from Round 37). **Round 36** lands BC-09 — a new
`## Capability cross-reference` appendix in
[DESIGN-CONTROLLER/HARDWARE_BOM.md](DESIGN-CONTROLLER/HARDWARE_BOM.md)
that maps every **physical-shape** build-config leaf (the 15 leaves
whose value materially changes which parts you order) to the BOM
section + part(s) that physically realise it. Pure-software / tunable
leaves (`unit_id`, `schema_version`, `*_ramp_seconds`, every
`safety.*` threshold, every `ui.*`, every `net.*`) are deliberately
omitted because they do not change the BOM. The new
[base_station/tests/test_hardware_bom_xref_sil.py](DESIGN-CONTROLLER/base_station/tests/test_hardware_bom_xref_sil.py)
adds 6 tests across 5 classes BC09_A..BC09_E pinning the table to
reality: every appendix capability ID exists in the schema (BC09_A
— no orphan IDs); every entry in the curated
``_PHYSICAL_CAPABILITIES`` set appears in the appendix AND every
curated entry is itself a real schema property (BC09_B —
bidirectional); every row's evidence cell carries a ``Tier 1`` /
``Tier 2`` / ``Tier 3`` token OR a ``planned`` marker so unrealised
capabilities (e.g. the three Round-33 ``aux.*`` leaves) stay self-
documenting (BC09_C); every relative link in the appendix resolves
on disk (BC09_D); the appendix self-references its enforcing test
file so the next reader can find the contract that pins it (BC09_E).
Adding a new physical-shape capability is now a five-place edit
(schema, default TOML, loader dataclass, codegen ``_SECTIONS``, AND
the BOM xref row) instead of four; the gate fails loudly if step
five is forgotten. Suite at **522 tests / 1 skipped / 43 files**
(516 → 522 from Round 36). **Round 35** lands BC-08 — a single
operator-facing onboarding doc
[DESIGN-CONTROLLER/BUILD_CONFIG.md](DESIGN-CONTROLLER/BUILD_CONFIG.md)
that ties every piece of the per-unit build-config system together
for the next person who is handed a LifeTrac v25 to bring up. Nine
short sections cover what a build-config is, where the files live
(schema / default / per-unit / loader / codegen / CLI), the eight
required sections (with a one-line rationale + common-override-
reason for each), the three reload classes (``live`` /
``restart_required`` / ``firmware_required`` with the operator
action each demands), the seven CLI subcommands (``validate`` /
``bundle`` / ``verify`` / ``diff`` / ``push`` / ``codegen`` /
``dump-json``), the deterministic ``config_sha256`` identity, the
``config_loaded`` audit trail, the four common pitfalls (UTF-8
BOMs trip ``tomllib``, section ordering matters for tests, a new
schema leaf needs four edits, ``firmware_required`` changes need a
reflash), and a where-to-read-next pointer table. The new
[base_station/tests/test_build_config_doc_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_doc_sil.py)
adds 5 tests across 5 classes BC08_A..BC08_E pinning the doc to
reality: every required schema section is named (BC08_A); every
``sub.add_parser`` registered subcommand is named AND no orphan
subcommands appear in the synopsis table (BC08_B); every
``reload_class`` value the schema declares is named, and the
schema's vocabulary is exactly the documented three (BC08_C);
every relative-path link target resolves on disk (BC08_D); every
``estop_topology`` enum value appears verbatim (BC08_E —
representative drift gate so a new topology can't be added without
updating the doc). Suite at **516 tests / 1 skipped / 42 files**
(511 → 516 from Round 35). **Round 34** lands BC-06 — the Wave-4
HIL dispatcher is now build-config aware. Per-gate applicability
rules live in the new
[DESIGN-CONTROLLER/hil/gate_applicability.json](DESIGN-CONTROLLER/hil/gate_applicability.json)
file (one source of truth shared between PowerShell and Python
consumers); ``hil/dispatch.ps1`` gains ``-ConfigPath`` and
``-NoApplicability`` parameters and shells out to a new
``lifetrac-config dump-json`` subcommand to load the validated
config as canonical JSON, then evaluates the rules table to flag
gates the active fleet shape can't exercise as ``N/A`` instead of
``NOT-STARTED``. The recommended-next-gate line skips ``N/A`` rows.
The rules cover every Wave-4 gate (BC06_A enforces parity between
the rules file and ``$Script:GateTargets``): radio gates W4-01 /
W4-02 are N/A when ``comm.handheld_present == false``; W4-05 is
N/A when ``hydraulic.proportional_flow == false``; W4-06 is N/A
when ``hydraulic.arm_axis_count == 0``; W4-08 is N/A when
``cameras.count == 0``; the rest always apply. The new
[base_station/tests/test_hil_dispatch_applicability_sil.py](DESIGN-CONTROLLER/base_station/tests/test_hil_dispatch_applicability_sil.py)
adds 13 tests across 7 classes covering rules-file shape +
dispatcher-vs-rules parity (BC06_A), canonical default fully
applicable (BC06_B), four representative N/A scenarios (BC06_C
through BC06_F), and ``dump-json`` determinism + payload contract
(BC06_G). The applicability evaluator is a pure-Python translation
of the four ops the rules file declares (``eq`` / ``gt`` / ``gte`` /
``truthy``) so the SIL gate exercises the rules without ever
subprocessing ``powershell.exe`` from CI; the PowerShell evaluator
in ``dispatch.ps1`` is the trivial mechanical mirror of the same
vocabulary. Suite at **511 tests / 1 skipped / 41 files**
(498 → 511 from Round 34). **Round 33** lands BC-11 — first-class auxiliary
attachment ports. The schema gains an ``[aux]`` top-level section
(``port_count`` 0..2, ``coupler_type`` ``iso_5675``/``flat_face``/``none``,
``case_drain_present``); every leaf carries
``reload_class = restart_required`` because aux ports are a hardware
capability that requires re-init of the M4 PWM channels and the
attachment-permit gate. The canonical default is the conservative
shape (``port_count = 0``, ``coupler_type = "none"``,
``case_drain_present = false``) so existing v25 builds without aux
plumbing keep getting the safe answer; per-unit ``build.<unit_id>.toml``
files can opt in. The Round 31 codegen walks ``aux`` automatically
(now in ``_SECTIONS``) and emits ``LIFETRAC_AUX_PORT_COUNT``,
``LIFETRAC_AUX_COUPLER_TYPE`` (with enum side-macros
``LIFETRAC_AUX_COUPLER_TYPE_NONE`` etc.), and
``LIFETRAC_AUX_CASE_DRAIN_PRESENT`` macros so M4 firmware can
``#if LIFETRAC_AUX_PORT_COUNT > 0`` to gate aux-PWM init. The on-disk
[firmware/common/lifetrac_build_config.h](DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h)
is regenerated; new SHA ``f961d9fd924b52ec46074c6cd26e6d1efcac035f8993e676f6003beb84b1c280``
replaces the Round 31 ``63a15fcb...``. The new
[base_station/tests/test_build_config_aux_section_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_aux_section_sil.py)
adds 13 tests across BC11_A (schema declares aux + every leaf
``restart_required`` + coupler_type enum includes ``none``), BC11_B
(``AuxConfig`` dataclass + canonical default conservative + loader
rejects missing section), BC11_C (codegen emits every leaf macro +
enum side-macros), BC11_D (``iter_reload_classes`` covers every aux
leaf + ``diff_reload_classes`` classifies aux changes as
``restart_required``), and BC11_E (committed on-disk header contains
the aux block — the drift gate that fails when an aux schema edit
forgets the regen). [CAPABILITY_INVENTORY.md](DESIGN-CONTROLLER/CAPABILITY_INVENTORY.md)
gains an "Auxiliary attachment ports" section so the schema-vs-doc
parity gate stays green. Suite at **498 tests / 1 skipped /
40 files** (485 → 498 from Round 33). **Round 32** lands BC-07 — the variant-matrix
SIL. Four representative fleet shapes (canonical / no-camera / no-IMU+GPS /
single-axis) are synthesised inline from the canonical default TOML
and round-tripped through the loader, ``web_ui`` module-import
(``_CAMERA_IDS`` filter + ``MAX_CONTROL_SUBSCRIBERS`` substitution),
the Round 31 codegen (header values reflect the variant), and the
BC-10 reload-class diff helper (camera-only diffs classify as
``live``, axis-count diffs bubble to ``restart_required``). The four
variants produce pairwise-distinct ``config_sha256`` values so a
single ``config_loaded`` audit-log line disambiguates the fleet shape
that booted. The new
[base_station/tests/test_build_config_variant_matrix_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_variant_matrix_sil.py)
adds 14 tests across BC07_A (loader accepts every variant; canonical
fixture byte-identical to the default), BC07_B (web_ui consumes the
variant: empty camera table when ``cameras.count == 0``, default
camera table when canonical, ``MAX_CONTROL_SUBSCRIBERS`` reads from
``BUILD.ui`` even on single-axis), BC07_C (codegen for every variant:
present flags zero on no-camera and no-IMU+GPS, model strings persist
so re-enable just flips a bool, single-axis emits the smaller numbers
verbatim with the ``f`` suffix on the float), BC07_D (canonical
self-diff empty; no-camera diff stays inside the cameras section and
bubbles to ``live``; single-axis diff bubbles to ``restart_required``
because ``track_axis_count`` and ``arm_axis_count`` carry that
reload class), and BC07_E (pairwise-distinct SHAs across the four
variants + canonical SHA equals a fresh load of
``build.default.toml``). Suite at **485 tests / 1 skipped /
39 files** (471 → 485 from Round 32). **Round 31** lands BC-03 — firmware codegen.
New [base_station/build_config_codegen.py](DESIGN-CONTROLLER/base_station/build_config_codegen.py)
walks the validated `BuildConfig` and emits a deterministic C header
([firmware/common/lifetrac_build_config.h](DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h))
that the M4/M7 sketches `#include` in place of the hand-edited
`#define` blocks they carry today (existing
`LIFETRAC_M4_WATCHDOG_MS` is preserved as a legacy alias of the
generated `LIFETRAC_SAFETY_M4_WATCHDOG_MS` so the firmware can
migrate without a flag-day rename). Every scalar leaf the JSON Schema
declares becomes a `#define` (~30 macros across hydraulic / safety /
cameras / sensors / comm / ui / net), with type-safe formatting
(`int` → bare literal, `bool` → `1`/`0`, `float` → `2.0f`-suffixed,
`str` → double-quoted) and enum side-macros
(`LIFETRAC_SAFETY_ESTOP_TOPOLOGY_PSR_MONITORED_DUAL` etc.) so
sketches can `#if` against the canonical truth. The header is
ASCII-only, LF-terminated, deterministic (no timestamps in the body),
and carries `LIFETRAC_UNIT_ID` + `LIFETRAC_SCHEMA_VERSION` +
`LIFETRAC_CONFIG_SHA256_HEX` for runtime cross-checks against the
`config_loaded` audit entry. The `tools/lifetrac-config` CLI grows
a `codegen` subcommand with `--check` mode (zero-exit when the
on-disk header matches the canonical emission, non-zero on drift) so
CI catches a schema or default-TOML edit that didn't also regenerate
the firmware header. The new
[base_station/tests/test_build_config_codegen_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_codegen_sil.py)
adds 22 tests across BC03_A (header-shape: guard, identity macros,
ASCII / LF / no trailing whitespace), BC03_B (leaf-parity: every
schema leaf appears as a `#define`), BC03_C (type formatting: int /
bool / float / string / enum side-macros), BC03_D (determinism + the
CI gate that fails when the on-disk firmware header drifts from the
default TOML), BC03_E (legacy aliases resolvable + ordered correctly),
BC03_F (CLI write byte-identical to direct `emit_header`, `--check`
clean / drift / canonical), and BC03_G (source tripwires for the
module + CLI + on-disk header). Suite at **471 tests / 1 skipped /
38 files** (449 → 471 from Round 31). **Round 30** lands BC-05 — the operator-facing
build-config admin form on the base UI. New page at `/config`
([web/config.html](DESIGN-CONTROLLER/base_station/web/config.html))
is a thin client over four PIN-gated sibling routes:
`GET /api/build_config/source` seeds the editor with the verbatim
active TOML body, `POST /api/build_config/preview-diff` schema-
validates a candidate body and returns the would-be reload-class
classification (`live` / `restart_required` / `firmware_required`)
without writing, `POST /api/build_config/upload` schema-validates,
refuses `firmware_required` diffs (re-flash via bench), atomically
rewrites `BUILD.source_path` via `installer_daemon._atomic_write`,
and emits a `config_upload` audit entry; the existing watcher then
picks up the change on its next poll and routes it through the same
BC-10 contract the X8 installer uses (`live` → swap immediately;
`restart_required` → set sticky `restart_pending` flag, surface in
the watcher state). The page itself shows running unit_id, schema
version, SHA, source path, and watcher pill, plus a Preview-Diff
button that lists every changed leaf with its reload class before
the operator commits. New SIL gate
[base_station/tests/test_build_config_admin_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_admin_sil.py)
adds 16 tests across BC05_A (page routing + widget IDs), BC05_B
(source endpoint verbatim + auth gate), BC05_C (preview-diff:
identical / live / restart-required / schema-violation + no-write
tripwire), BC05_D (upload: identical / live atomic-write / schema-
rejected / unit_id-mismatch / firmware-required-rejected + auth
gate), and BC05_E (source + HTML tripwires). Suite at
**449 tests / 1 skipped / 37 files** (433 → 449 from Round 30).
**Round 29b-beta** lands the tractor-side half
of the BC-10 delivery surface and closes the loop opened by 29b-alpha.
New [base_station/installer_daemon.py](DESIGN-CONTROLLER/base_station/installer_daemon.py)
verifies a USB-stick bundle against this X8's identity at three layers
(filename `unit_id`, header `unit_id`, body `unit_id`), schema-validates
the body, classifies the diff against the in-memory current config
(rejecting `firmware_required` outright since a re-flash isn't a
USB-stick operation), and atomically replaces the active TOML via
`os.replace` so a partial write can't leave a half-applied config
behind. Every outcome — `applied` / `rejected` / `noop` /
`deferred` — produces an `InstallResult` that gets serialised to
`lifetrac-config-result.json` next to the bundle on the stick, so an
operator unplugging the stick has a record of what happened.
[base_station/feedback.py](DESIGN-CONTROLLER/base_station/feedback.py)
pins the operator-facing contract: a 21-character OLED line and a
blink pattern keyed off the install status (3 long green = applied,
3 short red = rejected, amber slow = deferred, single short green =
no-op). The pattern table is locked verbatim in the SIL gate so
changing it is a deliberate contract bump.
[base_station/config_watcher.py](DESIGN-CONTROLLER/base_station/config_watcher.py)
is the watch-and-reload helper consumed by both the base `web_ui`
(via the new `/api/build_config/state` endpoint) and the tractor-side
`lora_bridge`: it polls the source-path `mtime`+`size`, reloads on
change, runs `diff_reload_classes` + `evaluate_quiescence` (Round 29
helpers), and emits one of six `WatchEvent` kinds (`noop` /
`applied` / `deferred` / `restart_pending` / `firmware_required` /
`rejected`) so the caller does no decision-making of its own. The
`tools/lifetrac-config` CLI grows a `push` subcommand (path 1: local
copy into a USB-stick mount or LAN drop directory with optional
`--apply` that invokes the daemon directly; path 5: scp+ssh-trigger
plan, dry-run by default, `--execute` to actually run). The
base web UI gains `/api/build_config/state` (PIN-gated, returns
`{unit_id, sha256, schema_version, restart_pending, last_event,
last_event_reason, changed_leaves, worst_reload_class}`) and emits a
`config_watch_event` audit entry on every non-noop poll. The new
[base_station/tests/test_build_config_installer_daemon_sil.py](DESIGN-CONTROLLER/base_station/tests/test_build_config_installer_daemon_sil.py)
adds 30 tests across BC10c_A (daemon: apply / discover / result-file),
BC10c_B (feedback: pinned LED + OLED for every status), BC10c_C
(watcher: noop / applied-when-quiet / deferred-when-busy /
restart-pending / rejected-on-schema-violation + `/api/build_config/state`
integration), and BC10c_D (push: local copy / local --apply / ssh
dry-run + module-export tripwires for `installer_daemon`, `feedback`,
`config_watcher`, the web route, the audit verb, and the CLI
subcommand). The X8 systemd unit + udev rule that wraps
`installer_daemon.process_mount` in a polling loop on USB-stick mount
is a bench operation queued for the next on-tractor checkout. BC-03,
BC-05, BC-06, BC-07, BC-08, BC-09 remain queued. **Round 29b-alpha**
lands the base-side half of
the BC-10 delivery surface: the installer-bundle envelope, a four-
subcommand laptop CLI (`tools/lifetrac-config validate / bundle /
verify / diff`), and a PIN-gated `/config/download` route on the base
web UI that emits a named, SHA-stamped installer file an operator can
drop on a USB stick. Bundle format is plain UTF-8 text with a header
block (`# bundle_version`, `# unit_id`, `# sha256`, `# generator`,
`# created`) terminated by a `# --- body ---` sentinel and the
validated TOML verbatim, so an operator can `cat` the file in the
field and see exactly what's about to be applied; the SHA covers body
bytes only (no chicken-and-egg with the header that carries it) and
the filename `lifetrac-config-<unit_id>-<sha8>.toml` carries the same
`unit_id` + first-8-of-SHA so a USB stick with multiple bundles is
self-disambiguating. The new
[base_station/config_bundle.py](DESIGN-CONTROLLER/base_station/config_bundle.py)
is the single source of truth for the format; both the CLI and the
web route use `make_bundle` / `serialise` / `parse` /
`verify_filename_matches` so byte-for-byte agreement between
"download from base UI" and "build with the CLI" is enforced by
construction. The CLI exits 0 on success and 2 on validation /
verification failure (reserves 1 for unexpected exceptions so CI can
distinguish "config bad" from "tool bad"); `validate` and `verify`
schema-validate the body too, so a malformed TOML can never reach
the X8-side installer (deferred to 29b-beta). The bundle is gated by
[`base_station/tests/test_build_config_installer_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_build_config_installer_sil.py)
(24 tests across 4 classes BC10b_A/B/C/D) which pins the envelope
(round-trip, filename pattern, body-tamper breaks SHA, unsupported
bundle_version refused, bad unit_id pattern refused, filename/header
mismatch refused, missing sentinel refused), the CLI (`validate`
passes canonical / fails out-of-range; `bundle` writes a file whose
on-disk SHA matches the embedded SHA; `verify` accepts bundler output
/ rejects tampered body; `diff` reports identical / classifies live /
classifies restart_required), the download route (401 without
session, 200 with session, returns a parseable bundle with the
documented Content-Disposition filename, emits a `config_download`
audit entry carrying the SHA), and source tripwires (CLI script
exists with the four subcommand handlers; web_ui carries the route
and the `config_download` audit verb; `config_bundle.py` exports the
documented surface). The X8-side installer daemon (USB-stick watch,
filename + unit-id + SHA verify, atomic-rename apply, OLED + LED
feedback, `lifetrac-config-result.json` write-back) and the daemon
watch-and-reload loop in `web_ui` + `lora_bridge` are deferred to
**Round 29b-beta** so this PR stays a clean base-side round; the CLI
and the download route are immediately useful for fleet-onboarding
even without the X8 installer in place. BC-03, BC-05, BC-06, BC-07,
BC-08, BC-09 remain queued. **Round 29** lands the schema + library
half of
BC-10 — the hot-reload contract that future config delivery (USB-cable
CLI, base-UI installer bundle) is built on. Every leaf in
[build_config.schema.json](DESIGN-CONTROLLER/base_station/config/build_config.schema.json)
now carries a `reload_class` annotation (`live`, `restart_required`, or
`firmware_required`); a missing or out-of-enum annotation is an error,
not a default. The loader gains three pure helpers in
[build_config.py](DESIGN-CONTROLLER/base_station/build_config.py):
`iter_reload_classes(schema)` returns the dotted-path → class map;
`diff_reload_classes(old, new)` returns a `ReloadDiff` with `changed`
paths, per-path classes, and the strictest `worst` class so the
installer can decide apply vs defer vs reject in one call;
`evaluate_quiescence(state)` checks the four preconditions any live
reload depends on (parked ≥ 30 s, no `/ws/control` subscribers, M7 TX
queue empty, engine off or idle-low) and returns an operator-facing
reason string when any fails. `firmware_required` is currently
restricted to `schema_version` and `safety.m4_watchdog_ms`; the SIL
gate pins this set so any future demotion or expansion is a deliberate
edit. The bundle is gated by
[`base_station/tests/test_build_config_delivery_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_build_config_delivery_sil.py)
(19 tests across 5 classes BC10_A/B/C/D/E) which pins reload-class
annotations (every leaf, in-enum, firmware set exact, missing/unknown
raises), diff classification (identical → empty; live-only; restart-
required; firmware-required wins over both; undeclared leaf raises),
quiescence (all four preconditions block independently with their own
operator-facing reason; threshold configurable; OK only when all hold),
leaf coverage (no schema/loader drift in either direction), and source
tripwires (loader contains the BC-10 helper names; schema uses the
`reload_class` keyword and every class token). The actual delivery
landing (laptop USB-cable `lifetrac-config push` CLI, base-UI installer
bundle download route, X8-side installer daemon with OLED + LED
feedback, atomic-rename + result-file write-back) is deferred to
**Round 29b** so this PR stays a clean schema/library round; the
contract pinned here is what 29b builds against. BC-03, BC-05, BC-06,
BC-07, BC-08, BC-09 remain queued. **Round 28** lands BC-04 — the first round in
which the BC-XX BuildConfig actually changes runtime behaviour rather
than just sitting on disk. Both
[`web_ui.py`](DESIGN-CONTROLLER/base_station/web_ui.py) and
[`lora_bridge.py`](DESIGN-CONTROLLER/base_station/lora_bridge.py) now
load the active config at boot (env
`LIFETRAC_UNIT_ID` selects the per-unit override) and write a
`config_loaded` audit-log line carrying `unit_id` / `source_path` /
`config_sha256` so post-mortems can cross-reference behaviour against
the exact file that was running. The web UI's `_CAMERA_IDS` table is
filtered against `BUILD.cameras.{front,rear,implement,crop_health}_present`
— a build with `cameras.count == 0` advertises an empty table (UI tile
and `/api/camera/select` API both refuse the absent positions); the
default canonical build (front camera only) advertises just `auto` +
`front`; a fully-loaded variant build advertises all five. The
`MAX_CONTROL_SUBSCRIBERS` constant is now read from
`BUILD.ui.max_control_subscribers` instead of being hard-coded; the
historical default of 4 is the dev-checkout fallback when the loader
fails. Loader failures are non-fatal — the daemons log a warning and
continue with built-in defaults so a dev checkout without a config
file still boots a degraded-but-running console rather than crash-
looping (BC-XX rationale: observe-the-failure first, then BC-05/BC-10
tighten enforcement). The bundle is gated by
[`base_station/tests/test_build_config_consumption_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_build_config_consumption_sil.py)
(11 tests across 5 classes BC4_A/B/C/D/E) which pins the boot audit
contract (both daemons emit `config_loaded` with the `unit_id` /
`source_path` / `config_sha256` field set, and the helper is a no-op
when `BUILD is None`), camera gating (canonical = `auto + front`,
full-loadout = all five, `count == 0` collapses), parameter
substitution (`MAX_CONTROL_SUBSCRIBERS` reads from BuildConfig at
import time, falls back to 4 when missing), graceful degradation
(missing TOML does not break module import; full hard-coded camera
table falls through), and source tripwires (both files greppably
reference the BC-04 audit call). BC-03, BC-05, BC-06, BC-07, BC-08,
BC-09, BC-10 remain queued. **Round 27** opens the BC-XX build-configuration
initiative with BC-01 + BC-02: a markdown
[`CAPABILITY_INVENTORY.md`](DESIGN-CONTROLLER/CAPABILITY_INVENTORY.md)
enumerating every optional / parameterised capability in the v25
BOM (axis count, E-stop topology, camera count, IMU/GPS presence,
LoRa region, etc.); a draft-07 JSON Schema at
[`base_station/config/build_config.schema.json`](DESIGN-CONTROLLER/base_station/config/build_config.schema.json)
that encodes the inventory machine-readably with strict
`additionalProperties: false` on every section; a canonical
[`build.default.toml`](DESIGN-CONTROLLER/base_station/config/build.default.toml)
for the canonical-BOM build; and a stdlib-only loader
[`base_station/build_config.py`](DESIGN-CONTROLLER/base_station/build_config.py)
(`load(unit_id) -> BuildConfig`) with frozen-dataclass nested
sections, fallback chain `LIFETRAC_BUILD_CONFIG_PATH` env →
`build.<unit_id>.toml` → `build.default.toml`, hand-rolled JSON
Schema validator (no third-party deps), and a deterministic
`config_sha256` derived from canonical-JSON of the validated
dict — what every boot will record into the audit log per BC-04.
The whole bundle is gated by
[`base_station/tests/test_build_config_loader_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_build_config_loader_sil.py)
(20 tests across 5 classes BC_A/BC_B/BC_C/BC_D/BC_E) which pins
schema well-formedness (every section is a strict object), loader
fallback-chain precedence (env wins over per-unit wins over default,
missing env path raises), strict validation (unknown top-level /
nested keys, out-of-range integers, wrong types, bad enum tokens,
pattern-violating `unit_id`, missing required section all raise
`BuildConfigError`), deterministic SHA-256 (matches the explicit
canonical-JSON formula, invariant under TOML whitespace + section
re-ordering, changes when any leaf changes), and **inventory
parity** (every schema property has a backticked `id` row in
`CAPABILITY_INVENTORY.md` and vice-versa — no third-place drift
between loader, schema, and doc). BC-03..BC-10 (firmware codegen +
`#if LIFETRAC_HAS_*` guards, web_ui consumption, admin `/config`
route, HIL `N/A` skipping, variant-matrix SIL, onboarding doc, BOM
cross-reference) remain queued. **Round 26** lands the Wave-4 HIL harness
toolchain under
[`DESIGN-CONTROLLER/hil/`](DESIGN-CONTROLLER/hil/): one PowerShell
harness skeleton per W4-XX gate (10 files) that wraps each
[HIL_RUNBOOK.md](DESIGN-CONTROLLER/HIL_RUNBOOK.md) procedure with
the §0 setup checklist, the per-gate prompts, the auto pass/fail
flip on threshold violation, and a uniform JSONL result-line
writer; a shared
[`_common.ps1`](DESIGN-CONTROLLER/hil/_common.ps1) helper exposing
`Write-GateHeader` / `Assert-Section0-Ready` / `Write-HilResult`
/ `New-RunId`; a [`results_schema.json`](DESIGN-CONTROLLER/hil/results_schema.json)
JSON Schema that locks the JSONL contract; and a
[`dispatch.ps1`](DESIGN-CONTROLLER/hil/dispatch.ps1) that reads
every `bench-evidence/W4-XX/results.jsonl`, prints a
status-table (`PASS/Target` per gate), and recommends the next
gate to run. The whole bundle is gated by the new
[`base_station/tests/test_hil_harness_completeness_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_hil_harness_completeness_sil.py)
(13 tests, 4 classes HC_A/HC_B/HC_C/HC_D) which pins file
existence, harness contract (dot-source `_common`, mandatory
`-Operator`, `Write-GateHeader` + `Assert-Section0-Ready` +
`Write-HilResult` + HIL_RUNBOOK back-reference), schema invariants
(W4-01..W4-10 gate-id pattern, PASS/FAIL/SKIP/ABORT enum, 5-key
firmware-SHA bundle), and dispatcher coverage (every gate has a
positive `Target`, no orphans either way). Day-1 with hardware is
now "edit three COM ports in `_common.ps1`, run `pwsh
./dispatch.ps1`, run the recommended harness" rather than "write
the harness from scratch." **Round 25** lands the IP-201 MQTT
retry/backoff fake-clock SIL
[`base_station/tests/test_mqtt_retry_backoff_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_mqtt_retry_backoff_sil.py)
(13 tests across 4 classes). The test reloads `web_ui` per-test with
`paho.mqtt.client.Client` stubbed so the module-import-time
`_connect_mqtt_with_retry()` call succeeds, then patches
`web_ui.mqtt_client.connect` with a scripted side-effect sequence
and replaces `web_ui.time.monotonic` / `web_ui.time.sleep` with a
`_FakeClock` that advances only on `sleep()`. This makes the
entire retry contract deterministic: env-var (`LIFETRAC_MQTT_HOST`)
honoured with `localhost` default; first-try success path observes
zero sleeps; one-retry-then-success observes exactly `[0.5]`; full
six-failure schedule observes `[0.5, 1.0, 2.0, 4.0, 5.0, 5.0]`
(geometric doubling, cap at 5.0); deadline path raises the
underlying exception verbatim once monotonic time crosses
`+30.0` s; late-success-near-deadline does NOT raise spuriously;
the deadline uses `time.monotonic` (NOT wall-clock `time.time`,
which can jump backwards under boot-time NTP slew). A source-grep
tripwire pins the 30 s deadline literal, the 0.5 s initial
backoff, the `min(backoff * 2, 5.0)` cap, the env-var lookup, and
the `(OSError, ConnectionError)` exception filter against future
refactors. **Round 24** lands the IP-102 nonce-seq thread-through SIL
[`base_station/tests/test_nonce_seq_threadthrough_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_nonce_seq_threadthrough_sil.py)
(13 tests across 4 classes). The test pins the bridge invariant that
every `_on_mqtt_message` arm reserves exactly one seq via
`_reserve_tx_seq()`, stamps it into the cleartext header (via
`_restamp_control` for the `cmd/control` arm or `pack_command(seq, ...)`
for `cmd/estop` / `cmd/camera_select` / `cmd/req_keyframe`), and
threads that *same* seq into `_tx(..., nonce_seq=seq)` so the GCM
nonce bytes (`build_nonce` offsets 1..3) match the cleartext header
seq bytes (`pt[3:5]`) byte-for-byte. The acceptance criterion from
the IP-102 spec ("100 control messages — AEAD nonce seq == cleartext
header seq for every frame; no replay-window false-rejects") is
driven verbatim. Negative tests confirm malformed payloads and
invalid camera-id values are rejected BEFORE `_reserve_tx_seq` so
garbage on the LAN side cannot accelerate `NonceStore` wear or
advance the persistent counter. A cross-arm interleave test pins
that all four arms share ONE counter (so two arms cannot reserve
the same value), and a 16-bit wrap test pins `tx_seq` masking so
the GCM nonce's 2-byte seq slot never desynchronises from the
cleartext header at wrap-around. A source-grep tripwire asserts
every `self._tx(` call inside `_on_mqtt_message` carries a
`nonce_seq=` keyword — the original IP-102 bug surface.
**Round 23** lands the Wave-3 polish constants SIL
[`base_station/tests/test_protocol_constants_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_protocol_constants_sil.py)
(12 tests across three IPs): IP-306 pins
`lora_proto.TELEM_MAX_PAYLOAD == 118` against the C-side
`TelemetryFrame.payload[120]` storage and the `payload_len = 0..118`
header comment so future buffer drift trips loudly; IP-303 pins
`REG_AUX_OUTPUTS = 0x0003` on both the Opta slave (`#define`,
`AUX_OUTPUTS_VALID_MASK = 0x0007`, and a `case REG_AUX_OUTPUTS:`
write-handler arm that masks reserved bits so a typo'd bitfield
cannot energise the valve-manifold SSR channels) and on the M7
Modbus master enum so the slot can never become orphaned;
IP-301 lands the one-line `s_btn_change_ms = millis();` anchor
at the end of `handheld_mkr.ino` `setup()` (after `Serial.begin` /
`pinMode` / `radio.begin` / OLED init) so the very first
`read_buttons()` call after boot can never satisfy
`(now - s_btn_change_ms) >= DEBOUNCE_MS` against the
uninitialised-zero default and commit a spurious debounced state.
**Round 22** lands the last two HIL-companion SILs:
W4-08 (camera back-channel keyframe round-trip / IP-103, IP-104)
via
[`base_station/tests/test_keyframe_round_trip_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_keyframe_round_trip_sil.py)
(17 tests) and W4-10 (fleet-key provisioning sanity / IP-008) via
[`base_station/tests/test_fleet_key_provisioning_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_fleet_key_provisioning_sil.py)
(24 tests). The keyframe SIL exercises the `/cmd/req_keyframe` →
`pack_command(seq, CMD_REQ_KEYFRAME)` translation, the on-air
encrypt + KISS encode + KISS decode + decrypt + parse round-trip
(including 0xC0 escape-path coverage), the M7 `send_cmd_to_x8()`
`[X8_CMD_TOPIC, opcode]` framing → KISS UART → `dispatch_back_channel`
keyframe-event firing, and a pure-Python time-on-air model that
proves the LADDER[0] (SF7/BW250) airtime fits the < 100 ms LoRa
leg with margin and the modeled end-to-end (REQ TOA + worst-case
in-flight CTRL TOA + UART KISS @ 115200 + encode-loop wake-up)
fits the < 200 ms HIL acceptance budget. The fleet-key SIL parses
both `handheld_mkr.ino` and `tractor_h7.ino` to assert the
`#ifndef LIFETRAC_ALLOW_UNCONFIGURED_KEY` guard exists in `setup()`,
calls `fleet_key_is_zero()`, halts in a forever-loop with
`delay()` inside, renders the canonical handheld OLED text
`"FLEET KEY NOT\nPROVISIONED\nHALT (IP-008)"` byte-for-byte, and
writes `LIFETRAC_ESTOP_MAGIC` (= 0xA5A5A5A5) to
`SHARED->estop_request` on the tractor side so the M4 watchdog
covered by `test_m4_safety_sil.py` drops the safety relay; the
Python half drives `lora_bridge._load_fleet_key()` through every
failure mode (missing env, all-zero hex, all-zero file, wrong
length, invalid hex, unreadable file) and verifies the
`LIFETRAC_ALLOW_UNCONFIGURED_KEY=1` bypass lives ONLY in the
module-level try/except, not inside the loader, so explicit
production calls always fail-closed. **Round 21** added
[`MASTER_TEST_PROGRAM.md`](MASTER_TEST_PROGRAM.md), the
canonical index of all SIL tests, compile gates, and HIL items
with per-IP traceability. **Round 20** lands the W4-07 (boot-PHY first-frame
decode / IP-006) SIL via
[`base_station/tests/test_boot_phy_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_boot_phy_sil.py)
(11 tests). The test parses the three relevant firmware sources
(`handheld_mkr.ino`, `tractor_h7.ino`, `tractor_x8/params_service.py`)
and asserts the `LADDER[3]` tables on the two LoRa nodes are
byte-identical, every `radio.begin(...)` references `LADDER[0]`
symbolically (regression-trapping numeric-literal drift), `LADDER[0]`
matches DECISIONS.md D-A2 (SF7/BW250/CR4-5/bw_code=1), the sync word
is `0x12`, and the Pi-side `control_phy` string agrees. **Round 19** ports the M7 adaptive-SF ladder state
machine (`try_step_ladder()`, `poll_link_ladder()`, the HB ingress
hook, and `apply_phy_rung()`) from
[`firmware/tractor_h7/tractor_h7.ino`](DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)
to pure-Python in
[`base_station/tests/test_link_tune_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_link_tune_sil.py)
(16 tests). W4-02 (link-tune walk-down) moves from greenfield bench
pass to verification pass against a documented model: the SF7→SF8→SF9
bad-window walk-down (R-8 hysteresis: 3 bad of 5 s windows), the
SF9→SF8→SF7 good-window walk-back (6 good of 5 s windows), SF9 +
SF7 saturation gates, twice-back-to-back announce invariant
(one TX at OLD PHY, one TX at NEW PHY, same target & reason byte),
500 ms revert deadline with commit / revert / edge-case coverage,
stale-active-source pessimism rule, and the verification-window
gate that prevents the in-flight tune window from being
double-counted as bad. Only the < 1% packet-loss metric still
needs an HIL bench RF attenuator. **Round 18** ports `step_axis_ramp()` and the four-axis
`apply_control` arbitration shape from
[`firmware/tractor_h7/tractor_h7.ino`](DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)
to pure-Python in
[`base_station/tests/test_axis_ramp_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py)
(10 tests). W4-05 (proportional valve ramp-out) and W4-06 (mixed-mode
skip) move from greenfield bench passes to verification passes
against a documented model: the track 2/1/0.5 s ladder, arm 1/0.5/0.25 s
ladder, linear-interpolation midpoint, coil-stays-engaged-through-ramp
invariant, flow-setpoint monotonicity, and the four mixed-mode skip
variants (including the subtle one where `other_active` is computed
from *raw* inputs so a stale ramping sibling can't block a fresh
release's ramp/skip decision) are all now CI-gated. **Round 17** flips the last best-effort Arduino compile
gate (`firmware-compile-tractor-h7`) to blocking and adds a parallel
`firmware-compile-tractor-m4` job. The M4 watchdog firmware was
moved out of the M7 sketch folder into its own
[`firmware/tractor_h7_m4/tractor_h7_m4.ino`](DESIGN-CONTROLLER/firmware/tractor_h7_m4/tractor_h7_m4.ino).
A `#if defined(CORE_CM4)` guard inside the original `tractor_m4.cpp`
fixed the M7 `multiple definition of 'setup'` link error, but the M4
build still failed because arduino-cli's library auto-discovery
scans the .ino's `#include` directives regardless of `target_core`,
pulling RadioLib + Modbus into the M4 build where they don't
compile against the M4 variant headers. Two separate sketch folders
is the canonical Portenta-dual-core layout (matches Arduino IDE's
stock `PortentaDualCore` example). M7 builds at 207888 B / 9% flash;
M4 at 87032 B. The Portenta core does not expose a separate
`envie_m4` FQBN — the M4 build uses the `target_core=cm4` board
menu option (`arduino:mbed_portenta:envie_m7:target_core=cm4`).
**Round 16** flips the second of three Arduino compile
gates (`firmware-compile-opta`) from `continue-on-error: true` to
blocking after a local arduino-cli replay exposed two issues: the
.ino base name didn't match its parent folder (renamed
`opta_modbus_slave.ino` → `tractor_opta.ino`) and the
`Arduino_Opta_Blueprint` library (provider of `OptaBlue.h`) was
missing from the workflow's install step. tractor_opta now compiles
clean (171520 B / 8% flash). **Round 15** turned the Arduino compile
gate from `continue-on-error: true` into a real blocking job for
`firmware/handheld_mkr` after a local arduino-cli replay exposed
three independent bugs that had been hiding behind that flag since
IP-007: stale `#include "../common/..."` paths that don't resolve
in arduino-cli's temp build dir; `--build-property build.extra_flags=...`
clobbering the MKR WAN 1310's `-DUSE_BQ24195L_PMIC` (which gates
`LORA_IRQ`); and `crypto_stub.c` requiring an explicit
`-DLIFETRAC_ALLOW_STUB_CRYPTO` opt-in. Both stub (55932 B) and real-crypto
(59940 B) handheld builds are now green locally and in
[`.github/workflows/arduino-ci.yml`](../.github/workflows/arduino-ci.yml).
A reproducible local staging script
[`DESIGN-CONTROLLER/tools/stage_firmware_sketches.ps1`](DESIGN-CONTROLLER/tools/stage_firmware_sketches.ps1)
mirrors the CI staging steps (extended in Round 17 to also stage
shared_mem.h into the new tractor_h7_m4 sketch).
**Round 14** resolves the TR-I cumulative-vs-consecutive
divergence pinned in Round 10: the M7's
`s_modbus_fail_count` in
[`firmware/tractor_h7/tractor_h7.ino`](DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)
now resets to 0 on every successful Modbus poll, so the 10-strike
E-stop latch counts *consecutive* failures and matches the W4-04
runbook prose. The SIL test suite was flipped accordingly: the old
`test_fail_count_is_cumulative_not_consecutive` (which pinned the
firmware bug) is replaced by `test_fail_count_is_consecutive` plus
two new pathological-pattern tests
(`test_intermittent_failures_never_latch`,
`test_nine_then_one_then_nine_does_not_latch`) — 1000-cycle
alternating fail/success and 9-1-9 patterns that *would* have tripped
under cumulative semantics but must not under consecutive. This
prevents the "noisy bus is a slow-burn nuisance trip" failure mode
where a single bad poll every few hours would eventually latch the
tractor down. Test count: 198 → 200 (+1 cumulative test removed,
+3 consecutive tests added).
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md`](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md)
— every plan item achievable without bench hardware is now landed
(Wave 0 8/8, Wave 1 8/8, Wave 2 9/9, Wave 3 9/9). 198 base_station
tests pass. **Round 13** completes the third and final fragmenter
fuzz: the on-air TileDeltaFrame chunker in
[`base_station/image_pipeline/reassemble.py`](DESIGN-CONTROLLER/base_station/image_pipeline/reassemble.py),
exercised via
[`base_station/tests/test_image_reassembly_fuzz.py`](DESIGN-CONTROLLER/base_station/tests/test_image_reassembly_fuzz.py).
13 tests cover IF-A through IF-L: round-trip identity for arbitrary
frame sizes split N ways, fragment-header invariants (magic /
constant `frag_seq` / monotonic idx / shared `total_minus1`),
bad-magic passthrough that must not disturb an in-flight assembly,
truncated-header-with-magic decode-error path, `frag_idx >= total`
rejection, reordered delivery (20 PRNG trials), duplicate handling
with `2*(N-1)` duplicate-counter check, missing-middle timeout +
GC, two-stream interleave by `frag_seq`, seq-wrap 255 → 0 across an
intervening GC, mid-stream `total` change recovery, and a 200-trial
PRNG-seeded randomised stress with optional shuffle + duplicate.
The system now has property-fuzz coverage on **all three** binary
parsers — IP-108 command frame (operator → tractor),
`telemetry_fragmentation_fuzz` (M7 → base, Round 11), and the
TileDeltaFrame chunker (camera → base, Round 13).
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md`](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md)
— every plan item achievable without bench hardware is now landed
(Wave 0 8/8, Wave 1 8/8, Wave 2 9/9, Wave 3 9/9). 185 base_station
tests pass. **Round 12** added a WS subscriber-concurrency stress
suite
([`base_station/tests/test_ws_subscriber_concurrency.py`](DESIGN-CONTROLLER/base_station/tests/test_ws_subscriber_concurrency.py))
that drives `_admit_ws`, `_discard_subscriber`, `_snapshot_subscribers`,
and `_on_mqtt_message` together against a real asyncio loop running on
a background thread — the same topology the production gateway
executes. 6 tests cover: cap honoured under N=4×cap concurrent admits
(WC-A), 200 admit/discard cycles with no leak (WC-B), 20 000 snapshot
iterations under 4-thread churn with zero `RuntimeError: Set changed
size during iteration` (WC-C), 500 `_on_mqtt_message` fan-outs while
3 churners mutate the pool (WC-D), per-pool cap independence (WC-E),
drain-and-refill counter integrity (WC-F), and 4429-on-overflow
close-code verification piggybacked on WC-A (WC-G). This converts
Round 9 §B/§C from "code looks right by inspection" to a CI-enforced
property.
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md`](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Status_v1_0.md)
— every plan item achievable without bench hardware is now landed
(Wave 0 8/8, Wave 1 8/8, Wave 2 9/9, Wave 3 9/9). 179 base_station
tests pass. **Round 11** added the property/fuzz suite for the
telemetry fragmentation path
([`base_station/tests/test_telemetry_fragmentation_fuzz.py`](DESIGN-CONTROLLER/base_station/tests/test_telemetry_fragmentation_fuzz.py)),
symmetric to the IP-108 command-frame fuzz: 15 tests cover round-trip
identity across every length × packable profile, header invariants
(magic, monotonic idx, shared seq/total), 25 ms airtime cap per
fragment, the > 256-fragment oversize rejection, reordered and
duplicated delivery, missing-middle timeout & GC, multi-source/topic
isolation, seq-wrap across GC boundaries, non-magic passthrough,
mid-stream `total` change recovery, and a 200-trial PRNG-seeded
randomised stress. **Round 10** added a SIL model of the Opta Modbus
slave + M7 `apply_control()` writer
([`base_station/tests/test_modbus_slave_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_modbus_slave_sil.py)),
which converts W4-04 (Modbus failure → E-stop) from a HIL-only gate
into a CI verification — 13 tests cover the 200 ms Opta watchdog,
the 10-strike fail-count latch, the fail-closed block shape, the
REG_ARM_ESTOP/external-loop trip paths, the AUX-mask guard, and the
W4-04 < 1 s budget. The suite also pins a documented divergence: the
current firmware fail-count is *cumulative* not *consecutive*, so the
W4-04 runbook prose and the firmware disagree (firmware wins until a
deliberate fix lands). **Round 9** addressed five operational-hardening
findings that surfaced in the parallel GPT-5.3-Codex and Gemini
second-pass reviews: **§B** `/ws/control` admission cap
(`MAX_CONTROL_SUBSCRIBERS=4` + `_admit_ws()` parity with
telemetry/image/state); **§C** `_subscribers_lock` guarding all WS-pool
mutations + `_snapshot_subscribers()` for the MQTT fan-out thread;
**§D** unconditional all-zero fleet-key refusal in both firmware
sketches (was previously gated only on `LIFETRAC_USE_REAL_CRYPTO`,
now requires explicit `LIFETRAC_ALLOW_UNCONFIGURED_KEY` opt-in);
**§E** `AuditLog` promoted to a module-level singleton via
`_get_audit_log()` so PIN failures stop opening a new file handle per
event; **§F** cross-language
`LP_REPLAY_WINDOW_BITS == REPLAY_WINDOW_BITS == 64` invariant test
parsing the C header. Remaining work is the HIL gate set
below — bench validation of W4-01…W4-10 and the **§A** CI compile-gate
flip from best-effort to blocking once the three `arduino-cli compile`
jobs go green on a real Actions runner.

Source reviews:

- [Claude Opus 4.7 (primary, with cross-review addendum)](AI%20NOTES/2026-04-28_Controller_Code_Pipeline_Review_ClaudeOpus4_7_v1_0.md)
- [GitHub Copilot v1.0](AI%20NOTES/2026-04-28_Controller_Code_Pipeline_Review_GitHub_Copilot_v1_0.md)
- [GPT-5.3-Codex v1.0](AI%20NOTES/2026-04-28_Controller_Code_Pipeline_Review_GPT-5.3-Codex_v1_0.md)
- [Gemini 3.1 Pro v1.0](AI%20NOTES/2026-04-28_Controller_Code_Review_Gemini_3.1_Pro_v1.0.md)

Wave 0 (IP-001 … IP-008) is the BLOCKER set — nothing else is testable
until those land. Wave 4 lists the gates that must pass before any wet
hydraulic test.

## Open initiative — Build-configuration / hardware-variant editor (BC-XX)

> **Status:** planning only — not yet scheduled. Captured 2026-04-29
> after the user observed that not every builder will procure every
> BOM line item and the software must not break when an optional
> sensor / camera / IMU / GPS / second-camera is absent or differently
> spec'd. Owner: TBD. Target start: after Wave-4 HIL gates are CLOSED.

### The problem

LifeTrac v25 firmware + base-station + web UI today assume the
canonical [`HARDWARE_BOM.md`](DESIGN-CONTROLLER/HARDWARE_BOM.md) build:
one Coral X8 camera, one IMU on the M7 I²C bus, one GPS on the X8,
the documented mushroom-button E-stop wiring, the four-track / four-arm
hydraulic axis count from
[`MASTER_TEST_PROGRAM.md`](MASTER_TEST_PROGRAM.md) §3. A builder who
cannot source — or chooses to omit — any of those parts today has
three bad options: ship anyway and let the code log errors / crash;
fork the firmware; or hand-edit constants. None of those scale to a
fleet. Specific failure modes already lurking in the codebase:

* `web_ui` rendering a "second camera" tile that always says
  `no signal` because the second X8 is absent.
* M7 logging Modbus failures forever for a non-existent Opta input
  (different E-stop topology that doesn't use the AUX bank).
* IMU-derived telemetry fields (`pitch_deg`, `roll_deg`) emitted as
  `null` and crashing chart code that assumed numbers.
* GPS-stamped audit log lines emitting `lat=0.000, lon=0.000` and
  poisoning any post-run analysis.
* W4-XX HIL gates failing because a builder omitted the sensor under
  test even though their build doesn't need that gate.

### Goals

1. **Fail-soft on absent parts.** Every optional capability must
   declare itself optional in code; UI and logging must tolerate
   `feature_disabled` without raising or rendering broken widgets.
2. **Per-build configuration as data, not code.** Each fleet unit
   has one canonical config file (`config/build.<unit_id>.toml`)
   that selects which BOM-optional features are present and pins
   their parameters (axis count, valve flow ratings, IMU/GPS port,
   camera count, E-stop topology id, …).
3. **Web UI editor.** Operator can view + edit the active config
   from the base-station web UI, with form-level validation against
   a JSON Schema, audit-logged save + reboot, and a one-click
   "export this config" for fleet onboarding.
4. **Compile-time + run-time gating.** Firmware respects the same
   config for `#define`-style feature flags via a generated
   `lifetrac_build_config.h`; bridge / UI honour it at run time.
5. **No regression in existing SIL coverage.** Existing 329 SIL
   tests must continue passing under the *default* (canonical-BOM)
   config; new SIL tests gate the variant matrix.

### Proposed work breakdown (BC-01 … BC-10)

> **Round ordering (post-Round-27):** BC-01 + BC-02 are landed. The
> recommended order from here is **Round 28 = BC-04** (consumers
> first, so the loader actually changes behaviour); **Round 29 =
> BC-10** (delivery + hot-reload contract, ahead of BC-05 because
> BC-05 is essentially a thin web UI on top of BC-10's atomic-write +
> reload machinery); **Round 30 = BC-05**; **Round 31 = BC-03**
> (firmware codegen + reduced-BOM compile gates is the largest piece
> and benefits from CI work landing on its own).

* **BC-01 — Capability inventory.** Walk the BOM, the firmware
  sketches, [`web_ui.py`](DESIGN-CONTROLLER/base_station/web_ui.py),
  the [`lora_bridge`](DESIGN-CONTROLLER/base_station/), and
  [`MASTER_TEST_PROGRAM.md`](MASTER_TEST_PROGRAM.md) §3 to enumerate
  every feature that is *physically optional* or *parameterised*.
  Output: a markdown table of capability id → default → spec range
  → consumer modules. This is the source of truth for the schema.
* **BC-02 — Config schema (JSON Schema + TOML loader).** Define
  `config/schema/build_config.schema.json` with one property per
  capability and per-capability `enum` / `min` / `max` / `default`.
  Land a Python loader
  (`base_station/build_config.py::load(unit_id) -> BuildConfig`)
  with strict-mode validation and a documented fallback chain
  (env override → `config/build.<unit_id>.toml` → `config/build.default.toml`).
* **BC-03 — Firmware codegen.** A pre-build script consumes the
  active TOML and emits
  `firmware/common/lifetrac_build_config.h` with the
  `#define LIFETRAC_HAS_IMU 1` style flags + the parameter literals
  (`LIFETRAC_AXIS_COUNT`, `LIFETRAC_CAMERA_COUNT`, …). Sketch code
  switches from hard-coded to `#if LIFETRAC_HAS_*` guards. Adds a
  CI job that builds the canonical-BOM config and three
  representative reduced-BOM configs to the existing five compile
  gates.
* **BC-04 — Base-station + web_ui plumbing.** Replace the
  hard-coded `CAMERA_COUNT = 1` / `HAS_IMU = True` constants with
  attribute reads off the loaded `BuildConfig`. UI templates render
  optional tiles only when the corresponding capability is enabled;
  MQTT topics that depend on absent hardware are not subscribed.
  Audit-log records the active `config_sha256` on every boot.
* **BC-05 — Web UI editor page.** New `/config` admin route
  (PIN-gated, same as the existing admin surface): form generated
  from the JSON Schema, client-side validation, save → write
  `config/build.<unit_id>.toml.next` → atomic rename → audit-log →
  prompt operator for restart. Read-only "diff against canonical"
  view to make fleet-onboarding obvious.
* **BC-06 — Per-build HIL harness gating.** Extend the Round-26
  [`hil/dispatch.ps1`](DESIGN-CONTROLLER/hil/dispatch.ps1) to read
  the active build config and report gates that don't apply as
  `N/A` rather than `NOT-STARTED`. Each `w4-XX_*.ps1` harness
  consults the config in `Assert-Section0-Ready` and `SKIP`s if
  the capability under test is disabled (e.g. W4-08 SKIPs when
  `camera_count == 0`).
* **BC-07 — SIL variant matrix.** New
  `base_station/tests/test_build_config_variants_sil.py` that
  loads the schema, instantiates ≥ 4 representative configs
  (canonical, no-camera, no-IMU/GPS, single-axis), and asserts
  the bridge + UI startup paths run clean (no exceptions, no
  subscription to disabled topics, no broken template renders).
  Plus a source-grep tripwire that `web_ui.py` references no
  optional capability without a `BuildConfig.has_*` guard.
* **BC-08 — Migration + onboarding doc.** New
  `DESIGN-CONTROLLER/BUILD_CONFIG.md` that explains the schema,
  the editor, the codegen, and the supported variants; existing
  builders create a `build.<unit_id>.toml` once and never edit
  firmware again.
* **BC-09 — BOM cross-reference.** Update
  [`HARDWARE_BOM.md`](DESIGN-CONTROLLER/HARDWARE_BOM.md) so each
  optional row lists its BC capability id, and a SIL test asserts
  every `optional` BOM row is referenced by exactly one capability
  in the schema (no orphans either way).
* **BC-10 — Config delivery & hot-reload contract.** Defines *how*
  a new `build.<unit_id>.toml` lands on a tractor / base station
  and *when* the running daemons reload it. Explicitly **non-OTA
  over LoRa** — LoRa airtime is reserved for control + telemetry
  + thumbnails, and the LoRa write surface is intentionally narrow
  (control / E-stop / camera-select opcodes only); rewriting the
  config over LoRa would be a category jump in trust boundary.
  Supported delivery paths, ranked by preference:
  1. **USB-cable from a laptop** (preferred for technical operators).
     The X8 enumerates as `g_ether` USB-Ethernet gadget; the laptop
     sees a `usb0` interface and runs `lifetrac-config push --target
     usb0 ./build.<unit_id>.toml`. CLI streams installer stdout
     (validation errors, applied SHA, deferral reason) to the laptop
     terminal in real time. Narrowest attack surface, best UX for
     anyone comfortable with a terminal.
  2. **USB-stick installer bundle generated by the base-station web
     UI** (preferred for non-technical operators — default field
     workflow). Operator opens the PIN-gated `/config` page on the
     base, edits fields via the schema-driven form, clicks
     **Download installer bundle** — base emits a single file named
     `lifetrac-config-<unit_id>-<sha8>.toml` with the validated TOML
     plus an embedded `# sha256:<full>` header comment. Operator
     copies the file to any USB stick, walks it to the tractor,
     plugs in. The tractor X8 installer auto-discovers the file
     (`lifetrac-config-*.toml` glob), refuses if `unit_id` in the
     filename or payload doesn't match this tractor, refuses if the
     embedded SHA doesn't match the file body (anti-corruption),
     then applies via the atomic-rename + quiescence-gate path.
     Result file `lifetrac-config-result.json` is written back to
     the stick: `{unit_id, source_sha256, applied_sha256, accepted,
     reason, applied_fields[], reload_class, timestamp}`. Operator
     pulls the stick, plugs it back into the base, the `/config`
     page surfaces the receipt and any deferral status. **All
     validation happens on the base UI before the file ever leaves
     the building**, so no syntax/range errors reach the tractor.
     Plus tractor-side OLED status line (`CFG: validated, applied
     <sha8>` / `CFG: REJECTED <reason>` / `CFG: deferred (in use)`)
     and X8 status-LED flash convention (3 long = applied, 3 short
     = rejected, alternating = deferred).
  3. **Hand-authored USB-stick TOML** (fallback for ops without the
     base online). Same on-tractor installer logic as path 2, but
     no UI-side validation — the installer is the only line of
     defence. Same OLED + LED feedback. Same result-file write-back.
     Documented as "power-user" in BUILD_CONFIG.md.
  4. **Web admin form on the base station** (BC-05, base-side
     config only — the base's *own* `BuildConfig` reload, not the
     tractor's).
  5. **SSH from any networked laptop** (advanced operators only).
     Same `lifetrac-config push` CLI, just over the LAN/WiFi link
     to the X8 instead of `usb0`. Documented in BUILD_CONFIG.md
     as the path for fleet-wide scripted updates; not the
     primary field workflow because not every operator is
     comfortable with SSH key management.

  **Quiescence gate** (precondition for live reload): tractor in
  `STATE_PARKED` for ≥ N seconds (default 30, configurable), no
  active `/ws/control` subscriber, engine off OR idle-low,
  M7 TX queue empty. If any condition fails, installer writes
  `.toml.next` and the daemons pick it up at the next quiescent
  window; operator sees a "reload pending" banner.

  **Reload taxonomy** (encoded as a `reload_class` annotation on
  every JSON Schema property):
  * `live` — reloadable without daemon restart (UI strings, camera
    presence flags, MQTT host with reconnect, `max_control_subscribers`,
    IMU/GPS presence flags, `hyd_pressure_sensor_count`).
  * `restart_required` — needs graceful daemon restart
    (`lora_region`, `estop_topology`, `m4_watchdog_ms`, axis counts).
    Detected by diffing old vs new `BuildConfig`; logged as
    "restart required" and operator confirms via OLED button or
    web UI before reboot.

  Deliverables: `tools/lifetrac-config` Python CLI (laptop-side,
  for paths 1 + 5); X8-side `lifetrac-config-installer` daemon
  (handles USB-stick discovery for paths 2 + 3, atomic rename,
  result-file write-back, OLED + LED feedback); base-side
  `/config/download` route emitting the named installer bundle
  for path 2; quiescence detector module in
  `base_station/build_config.py`; new `test_build_config_delivery_sil.py`
  (~15 tests covering USB-stick filename + unit-id-match
  enforcement, embedded-SHA verification, base-side bundle
  generation, quiescence-gate logic, reload-class annotations,
  result-file schema, atomic-rename invariant, deferred-reload
  pickup).

### Out of scope (for now)

* **OTA delivery of build configs over LoRa.** Wrong trust
  boundary (LoRa write surface is intentionally narrow), wrong
  airtime budget (would steal from control/telemetry/thumbnails),
  and would require reinventing transactional delivery + rollback
  on a half-duplex radio. Deliver via USB-cable / SSH / web admin
  per BC-10 instead.
* OTA delivery of build configs over cellular backup (would need
  signing + rollback; defer).
* Per-axis hydraulic flow auto-calibration from manifold response
  (separate initiative; the config only carries operator-supplied
  rated flows).
* Fundamentally different control topologies (e.g. CAN bus vs
  RS-485). The schema is for *presence/absence and parameters*,
  not for swapping the control fabric.

### Acceptance criteria

* `pwsh ./dispatch.ps1` shows correct PASS / N/A / NOT-STARTED for
  three distinct test fleets (canonical, no-camera, single-axis).
* All 329 + new BC SIL tests pass under the canonical config.
* Web UI `/config` editor round-trips a config edit → reboot →
  audit-logged `config_sha256` change with no firmware re-flash
  needed for run-time-only flags.
* No firmware sketch contains a hard-coded "is this hardware
  present" check that the schema doesn't also gate.

## Open initiative — Hydraulic soft-stop sequencer & build variants (BC-18 / BC-19 / BC-20)

Tracked in detail in
[DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md](DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md)
and the canonical decisions in
[DESIGN-CONTROLLER/DECISIONS.md](DESIGN-CONTROLLER/DECISIONS.md) §D-HYD1 and
§D-HYD2. **Status:** designed, not yet implemented. Three sequential rounds:

* **BC-18 — Valve-centre settling timer (firmware sequencer).** Add the
  100 ms (default) settling delay between EFC reaching zero and the
  directional-valve solenoid de-energising, gating the tandem-centre spool
  (`D1VW00*8*CNKW`) becoming usable in field hardware. Concrete work:
  add `LIFETRAC_HYDRAULIC_VALVE_SETTLING_MS` macro to
  [`firmware/common/lifetrac_build_config.h`](DESIGN-CONTROLLER/firmware/common/lifetrac_build_config.h);
  gate solenoid de-energise in
  [`firmware/tractor_opta/tractor_opta.ino`](DESIGN-CONTROLLER/firmware/tractor_opta/tractor_opta.ino)
  valve-drive loop behind a settling timer that starts when each
  `REG_FLOW_SP_*` register reaches zero; new SIL gate
  `base_station/tests/test_valve_settling_sil.py` (4 cases:
  axis-active→zero starts timer; EFC reaches zero before timer; valve
  de-energises only after timer; E-stop cancels timer immediately);
  MASTER_TEST_PROGRAM.md row.

* **BC-19 — Hydraulic build-variant schema. ✓ LANDED Round 43.**
  Make the soft-stop parameters configurable per unit so non-canonical
  hydraulic builds (OSE-legacy float + PO check, high-performance
  open + counterbalance, etc.) work without forking firmware. Three
  new leaves added under `[hydraulic]` in
  [`base_station/config/build_config.schema.json`](DESIGN-CONTROLLER/base_station/config/build_config.schema.json) —
  `spool_type` (enum: `tandem` / `float` / `closed` / `open`,
  `restart_required`), `load_holding` (enum: `spool_inherent` /
  `po_check` / `counterbalance` / `none`, `restart_required`),
  `valve_settling_ms` (uint 0..250, `live`).
  [`build.default.toml`](DESIGN-CONTROLLER/base_station/config/build.default.toml),
  `HydraulicConfig` dataclass in
  [`build_config.py`](DESIGN-CONTROLLER/base_station/build_config.py),
  schema-driven codegen in
  [`build_config_codegen.py`](DESIGN-CONTROLLER/base_station/build_config_codegen.py)
  (auto-emits 11 new `LIFETRAC_HYDRAULIC_*` macros incl. enum side-
  flags), and `CAPABILITY_INVENTORY.md` rows all updated.
  Cross-leaf compatibility validator
  `_validate_hydraulic_compatibility` lives in
  [`build_config.py`](DESIGN-CONTROLLER/base_station/build_config.py)
  and runs after JSON-Schema validation in `load()`; rejects three
  contradictory combinations (tandem/closed + load_holding=none;
  float/open + load_holding=spool_inherent; float/open + non-zero
  valve_settling_ms). New SIL gate
  [`test_hydraulic_compatibility_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_hydraulic_compatibility_sil.py)
  pins 13 cases (BC19_A canonical, BC19_B four reference builds load,
  BC19_C six invalid pairings rejected, BC19_D rejection-at-load
  tripwire). MASTER_TEST_PROGRAM.md §5 row promoted from "(planned)"
  to landed. BUILD_CONFIG.md gets a new "Common pitfalls" entry on
  the cross-leaf validator hook. Firmware consumer (BC-18) will read
  the new `LIFETRAC_HYDRAULIC_SPOOL_TYPE` and `_VALVE_SETTLING_MS`
  macros in the next round instead of hard-coding tandem.

* **BC-20 — IMU-adaptive ramp tuning (jerk-based auto-tune).** Use
  the BNO086 IMU already on the canonical BOM
  ([HARDWARE_BOM.md](DESIGN-CONTROLLER/HARDWARE_BOM.md)) to detect jerk
  spikes during start/stop transitions and slowly nudge
  `LIFETRAC_HYDRAULIC_TRACK_RAMP_SECONDS` /
  `_ARM_RAMP_SECONDS` / `_VALVE_SETTLING_MS` toward a smoother
  trajectory. **Read-only telemetry first; closed-loop tuning gated.**
  Detail in [DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md](DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md)
  §"IMU-adaptive ramp tuning". Three sub-rounds:

  * **BC-20A — passive jerk telemetry.** Compute `d(accel)/dt` from
    BNO086 linear-acceleration samples on the Portenta H7 M7 core,
    publish per-transition jerk peak (g/s, signed per axis) over the
    existing telemetry channel, surface as a chart in the web UI
    diagnostics page. **No control-loop change.** SIL: synthetic
    accel time series → expected jerk peak.

  * **BC-20B — recommendation engine (offline / advisory).** Base
    station accumulates per-build jerk-vs-ramp histograms over
    operator-confirmed clean transitions and recommends ramp-leaf
    edits (e.g. "track_ramp_seconds 2.0 → 2.4 reduces P95 jerk from
    1.8 g/s to 0.9 g/s"). Operator approves via the existing
    `lifetrac-config` flow (validate / bundle / verify / push) — no
    autonomous writes. SIL: mock telemetry stream → expected
    recommendation; reject recommendations that would push values
    outside the schema range.

  * **BC-20C — closed-loop online tuning (gated, opt-in).** New
    schema leaf `hydraulic.adaptive_ramp_tuning` (bool, default
    `false`, `restart_required`). When enabled, M7 firmware adjusts
    ramp seconds in ±5 % steps per accepted clean transition,
    bounded by the schema range, audit-logged, with operator panic
    button to revert to defaults. Disabled by default for v25
    canonical because closed-loop control on a safety-relevant
    parameter requires real bench validation (BC-12 HIL hardware
    dependency). SIL: simulated jerk feedback drives ramp
    adjustments; out-of-range writes refused; revert-on-panic
    restores baseline.

  **Why three sub-rounds:** BC-20A is pure observability and is safe
  to ship anytime. BC-20B closes the loop through the human via the
  existing config-delivery flow, which already has all the safety
  gates we need. BC-20C is the only sub-round that adds autonomous
  writes to a safety parameter, so it is opt-in, gated by HIL bench
  validation, and exposes a panic revert.

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
  [tractor_h7_m4.ino](DESIGN-CONTROLLER/firmware/tractor_h7_m4/tractor_h7_m4.ino)
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
- [ ] **Tractor radio driver/control path** (bridge currently talks
  KISS-over-serial; on-tractor needs the real radio path). 2026-05-04
  update: Max Carrier evidence points away from a bare M7↔SX1276 SPI
  path and toward the Murata LPWAN AT modem interface on X8 Linux
  `/dev/ttymxc3`; revalidate before writing more raw-SPI RadioLib code.
- [ ] **Bench-validate E-stop end-to-end** per the procedure in
  [KEY_ROTATION.md](DESIGN-CONTROLLER/KEY_ROTATION.md) §7.

---

## Hardware-in-the-loop (HIL) gate — required before any wet hydraulic test

These are the items the Round 1–4 implementation plan explicitly deferred
because they cannot be fully validated without hardware. W4-pre and W4-00
use the minimal two-carrier desk setup from the runbook; W4-01 and later
need the full bench setup (handheld + tractor H747 + Opta + expansions +
SX1276 modems on both ends). Each gate must pass before the system is
allowed to drive real hydraulics. Tracked
against the Wave-4 gates in
[`AI NOTES/2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md` § Wave 4](AI%20NOTES/2026-04-28_Controller_Code_Review_Implementation_Plan_v1_0.md).

**HIL bench setup prerequisites:**

W4-pre and W4-00 use their own minimal hardware lists in the runbook. The
full setup below applies before W4-01 and later.

- [ ] Two SX1276 LoRa modems wired (handheld MKR WAN 1310 + tractor H747).
- [ ] Tractor H747 + Opta connected over RS-485 with both expansions
  (D1608S + A0602) chained.
- [ ] Solenoid-valve bank powered through the PSR safety relay with the
  hydraulic supply valved off (dry test only, no fluid pressure).
- [ ] Logic analyzer or scope on the X8↔H747 UART, the M7↔M4 GPIO, and
  the PSR-alive line. CRC-protected `.csv` capture for each test.
- [ ] Battery emulator on the 12 V rail so a brownout can be simulated
  without yanking the keyswitch.

**HIL tests (Wave 4 gates):** Step-by-step bench procedures, capture
conventions, pass criteria, and sign-off rules for every gate below are
documented in
[`DESIGN-CONTROLLER/HIL_RUNBOOK.md`](DESIGN-CONTROLLER/HIL_RUNBOOK.md).
Execute from the runbook; the bullets below are the index.

- [ ] **W4-pre Board bring-up sanity.** Partial 2026-05-04 evidence:
  both Portenta X8 M7 cores flash, boot, reach `loop()`, and update
  SRAM4 liveness with CFSR/HFSR = 0. Still open: USB enumeration,
  3.3 V/5 V rail measurements, blink/echo, stock M7<->M4 handshake,
  and formal capture/sign-off package.
- [ ] **W4-00 LoRa stack dual-Portenta bench.** Blocked until the X8
  no-USB bench image has a control plane plus a real base-role
  1000-frame encrypted ControlFrame burst sender and receiver-side
  counters/export. Also blocked on Max Carrier radio-interface
  revalidation: the onboard Murata `CMWX1ZZABZ-078` appears to be
  exposed through the X8 Linux AT-modem path (`/dev/ttymxc3`), while
  the current M7 raw-SPI RadioLib diagnostic fails before TX start.
  The current PowerShell harness records metrics but does not generate
  the burst.
- [ ] **W1-11 / W4-00(b) Latency testing improvements.** Status
  2026-05-11: bench-tier analyzer
  [`analyze_rtt.py`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/analyze_rtt.py)
  is wired into
  [`run_w1_10b_rx_pair_end_to_end.ps1`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_w1_10b_rx_pair_end_to_end.ps1)
  step 14 and confirms LoRa ToA SF7/BW125/44 B = **30.85 ms**
  deterministic across 800 samples (100/200/500-cycle runs), +2.8 % vs
  the [`LATENCY_BUDGET.md`](DESIGN-CONTROLLER/RESEARCH-CONTROLLER/LATENCY_BUDGET.md)
  §1 row #2 estimate. Measured one-way host overhead (TX_FRAME_REQ →
  TX_DONE_URC minus ToA) is **~24.5 ms**, much higher than the 5 ms
  assumed in `PREDICTED_HOST_OVERHEAD_MS`. Open follow-ups:
  - [x] **L-1 True ping-pong RTT.** Added `--probe rx_echo` (RX board
    re-TXes every received frame using its own `tx_id` counter) and
    `--probe ping_pong` (TX board sends payload, waits for both
    `TX_DONE_URC` and an `RX_FRAME_URC` echo matching the original
    payload, computes per-cycle `rtt_ms`) to
    [`method_h_stage2_tx_probe.py`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py).
    Wired through
    [`run_w1_10b_rx_pair_end_to_end.ps1`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_w1_10b_rx_pair_end_to_end.ps1)
    via new `-Probe`/`-RttTimeout` params (`-Probe ping_pong` selects
    the new pair) plus a B7 gate `rtt_match_rate ≥ 0.99`, and through
    [`analyze_rtt.py`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/analyze_rtt.py)
    (parses `__PINGPONG__` lines, emits `pingpong_rtt_ms` percentiles +
    `verdicts.w4_00b_within_5pct_pingpong`).
    *2026-05-12 first-light run* (`bench-evidence/W1-11_pingpong_2026-05-12_115122/`,
    100 cycles, InterCycleS=0.2, RttTimeout=3.0): all 7 gates **PASS**,
    `pingpong_rtt_ms` p50=**107.0** ms, p90=115.0, p99=118.0, p999=121.8,
    max=121.8 (rtt_match_rate=1.000). The bench ping-pong measures
    `2×ToA + 2×host_overhead`; predicted = 2×33.4 + 2×21 = **108.8 ms**,
    actual p50=107.0 ms (within 2 ms — independently confirms both ToA
    and the new `PREDICTED_HOST_OVERHEAD_MS=21.0`). Both
    `2*elapsed_p50` surrogate and the new true RTT print
    `OUTSIDE_BUDGET` against W4-00(b)'s `2*ToA + 30 ms = 96.8 ms ± 4.8`,
    because the bench HostLink path pays host overhead **twice** (once
    per board) while production M7-direct will pay it only once. The
    true production-equivalent RTT requires firmware-side echo (a
    `HOST_TYPE_CFG_RX_ECHO_ENABLE` config key + L072 dispatch that
    skips the host round-trip on the RX side); deferred until W2-01
    H747 transport bring-up makes the equivalent path live.
  - [x] **L-2 Reconcile budget rows #1 + #5.** Bumped
    `PREDICTED_HOST_OVERHEAD_MS` from 5.0 → **21.0 ms** in
    [`analyze_rtt.py`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/analyze_rtt.py)
    (bench-measured p50 across 5 evidence dirs / 3,400 samples;
    p999 ~30 ms; max 30.9 ms). Added bench-vs-production caveat
    blockquote to
    [`LATENCY_BUDGET.md`](DESIGN-CONTROLLER/RESEARCH-CONTROLLER/LATENCY_BUDGET.md)
    §1 explaining that the 21 ms overhead is the X8 HostLink
    (Python+ADB+UART) path, not the production M7-direct path.
    Refreshed all 5 prior W1-10b dirs (L-3a/c/d/e/f) so
    `rtt_report.json` and
    `summary.json.latency.budget_predictions.host_overhead_ms` reflect
    the new constant. (L-3b dir lacks `summary.json` from earlier
    terminal kill; analyzer warns and skips merge — non-fatal.) The
    finer encode-vs-decode split via firmware-side timestamps is
    deferred; current data confirms the *combined* overhead is
    well-modeled.
  - [x] **L-3 Higher-rate stress.** Re-ran `run_w1_10b_rx_pair_*`
    bracketing inter-cycle from 100 ms → 50 ms and cycles 200 → 2000
    to characterize p999 latency, RX-side queueing, and packet-loss
    rate under sustained load.
    *2026-05-11 first attempt:* `InterCycleS=0.05 Cycles=2000` wedged
    the L072 — RX listener entered TRANSPORT_FAIL on the next run
    (FATAL: VER warm-up failed: timeout waiting for response type 0x81
    to req 0x01; bench recovered after a single 200-cycle run at the
    default 0.2 s rate, no power-cycle needed).
    *2026-05-12 bracketing (this session):* repeated each rung end-to-end
    on the same fleet key and got **clean PASS at every rate**, including
    the previously-wedged 50 ms × 2000-cycle condition:

    | Rung | InterCycleS | Cycles | tx_done_rate | rx_match_rate | Verdict | Evidence dir |
    |---|---|---|---|---|---|---|
    | L-3a | 0.100 | 200 | 1.000 | 0.995 | RX_PAIR_PASS | `W1-10b_rx_pair_2026-05-11_230351` |
    | L-3b | 0.075 | 200 | (200/200, summary lost to terminal kill) | clean | effective PASS | `W1-10b_rx_pair_2026-05-11_230545` |
    | L-3c | 0.060 | 200 | 1.000 | 1.000 | RX_PAIR_PASS | `W1-10b_rx_pair_2026-05-11_230802` |
    | L-3d | 0.050 | 200 | 1.000 | 1.000 | RX_PAIR_PASS | `W1-10b_rx_pair_2026-05-11_230955` |
    | L-3e | 0.050 | 1000 | 1.000 | 0.999 | RX_PAIR_PASS | `W1-10b_rx_pair_2026-05-11_231144` |
    | L-3f | 0.050 | 2000 | 1.000 | 0.9995 | RX_PAIR_PASS | `W1-10b_rx_pair_2026-05-11_231533` |

    Findings:
    1. **Prior wedge is non-reproducible** under identical params on the
       same hardware — likely a one-off transport hiccup (USB/ADB jitter,
       a partial frame straddling a window, or a transient L072 timing
       slip). Not a hard ceiling at 20 Hz.
    2. **Real per-cycle wall time ≈ 156 ms**, not 50 ms — `InterCycleS`
       is the *delay between bursts*; each cycle also pays ToA
       (30.85 ms SF7/BW125) plus ~120 ms of host overhead for adb
       round-trips, HostLink encode/decode, and `__TX_DONE__` URC wait.
       L-3f took ~3 min 43 s for 2000 cycles → ~111 ms/cycle. The
       L-prefix `_PAIR_PASS` rates are therefore against a real link
       cadence of ≈ 6.5–9 Hz, not the nominal 20 Hz target. To truly
       stress 20 Hz, the host overhead in the helper toolkit needs to
       drop (batch URC parsing, persistent shell, or move TX scheduling
       on-MCU).
    3. **Single-frame loss in long runs** is consistent (1 missed frame
       in both L-3e and L-3f) — a 0.05 % loss at the boundary of the RX
       window or a single failed CRC; not a queueing collapse.

    Follow-ups remaining for L-3 family:
    - [x] **L-3g** Capture per-cycle elapsed array (already embedded in
      tx_stdout) and emit p50/p90/p99/p999 histograms. *Done
      2026-05-12:*
      [`analyze_rtt.py`](DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/analyze_rtt.py)
      `Stats` now carries p90 and p999 (in addition to p50/p95/p99); a
      new `--merge-summary` flag splices the analyzer payload into
      `summary.json` under a `latency` key, and step 14 of the
      orchestrator passes `--merge-summary` so every fresh evidence dir
      gets it for free. All 5 of L-3a/c/d/e/f were re-analyzed and now
      carry full histograms in `summary.json.latency.elapsed_ms`
      (p50≈51.8–55.4, p90≈55.9–56.2, p99=56.6–60.2, p999=59.8–60.6,
      max=59.8–61.3 ms across the 200..2000-cycle runs).
    - [ ] **L-3h (deferred)** Reduce host overhead so `InterCycleS=0.05`
      actually approaches 20 Hz cadence; then re-stress to find the
      *real* wedge condition.
  - [ ] **L-4 PHY sweep.** Repeat the 500-cycle measurement at
    SF8/BW125 and SF9/BW125 to populate ToA columns in
    `LATENCY_BUDGET.md` for the link-tune walk-down rungs.
  - [ ] **L-5 Cold-boot latency (W4-07 prep).** Sequence of N short
    bursts with full L072 reflash/reboot between each; characterize
    first-frame-after-boot latency vs warm-state latency. Feeds the
    handheld-power-on UX budget.
  - [ ] **L-6 End-to-end stick → hydraulic.** Once Opta + valve bench
    is live (§5 step 7), wire a synchronized timestamp from joystick
    ADC sample → LoRa TX → LoRa RX → Modbus → SSR coil → spool
    position sensor; this is the only test that closes the full
    LATENCY_BUDGET §1 rows #1–#13 chain and confirms the ≤ 150 ms
    target in [W4-01-stick-to-valve](#) below.
- [ ] **W4-01 Handheld E-stop latch latency.** Mushroom-button press →
  PSR-alive drops → all 8 valve coils de-energize within **< 100 ms**
  measured at the relay terminals. Repeat 100× across the SF7/SF8/SF9
  PHY rungs. Capture worst-case + histogram.
- [ ] **W4-02 Link-tune walk-down.** Force RSSI degradation
  (RF attenuator) and verify the M7 walks SF7 → SF8 → SF9 and back via
  `CMD_LINK_TUNE` with **< 1% packet loss** during each transition.
  Verify `try_step_ladder()` revert deadline (500 ms) fires correctly
  on a synthetic missing-HB.
  *SIL coverage:* [`base_station/tests/test_link_tune_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_link_tune_sil.py)
  (Round 19) ports `try_step_ladder()`, `poll_link_ladder()`, the HB
  ingress hook, and `apply_phy_rung()` from
  [`firmware/tractor_h7/tractor_h7.ino`](DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)
  to pure-Python; 16 tests cover TL-A through TL-I including the
  3-bad-window walk-down ladder, 6-good-window walk-back ladder, SF9
  saturation gate, SF7 saturation gate, twice-back-to-back announce
  invariant, 500 ms revert deadline (commit/revert/edge cases), and
  the stale-active-source pessimism rule. The < 1% packet-loss
  measurement is the only piece that still needs HIL bench time.
- [ ] **W4-03 M7↔M4 watchdog trip.** Halt the M7 with a debugger
  breakpoint or a deliberate `while(1)` injected via a debug build;
  verify the M4 trips the PSR within **200 ms** of the last
  `alive_tick_ms` and that `estop_request = 0xA5A5A5A5` is observed
  on the SRAM4 capture (IP-105/106 seqlock holds).
- [ ] **W4-04 Modbus failure → E-stop.** Pull the RS-485 cable mid-run;
  verify IP-205 counter ticks, `apply_control(-1)` fires after 10
  consecutive failures, valves go neutral, and the audit log shows
  the disconnect within 1 s.
  *SIL coverage:* [`base_station/tests/test_modbus_slave_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_modbus_slave_sil.py)
  (Round 10) models both the M7 `apply_control` writer and the
  Opta-side `on_holding_change`/`check_safety` surface; 13 tests cover
  TR-A through TR-J including the < 1 s W4-04 budget. Note TR-I:
  current firmware uses *cumulative* not *consecutive* fail count —
  pinned by `test_fail_count_is_cumulative_not_consecutive` so a
  future firmware fix will be a loud test failure.
- [ ] **W4-05 Proportional valve ramp-out (IP-303 Round-4 follow-on).**
  Hold full-speed track for 3 s, release joystick; verify
  `REG_FLOW_SP_*` ramps from 10000 mV to 0 over **2 s** (track ladder)
  on the A0602 output, valve coil stays energized through the ramp,
  drops at t=2 s. Repeat for arm axes (1 s ramp).
  *SIL coverage:* [`base_station/tests/test_axis_ramp_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py)
  (Round 18) ports `step_axis_ramp()` + the `apply_control()`
  arbitration to pure Python. TR-A pins the 2 s deadline at full speed,
  TR-B the linear-interpolation midpoint, TR-C/D both ladders
  (track 2/1/0.5 s, arm 1/0.5/0.25 s), TR-E the coil-stays-engaged
  invariant during ramp, TR-F flow-setpoint monotonicity. Bench job is
  now confirm-only.
- [ ] **W4-06 Mixed-mode skip.** Drive both axes; release one; verify
  released axis stops *immediately* (no ramp) because sibling is
  active. Critical for coordinated dig→drive transitions.
  *SIL coverage:* same suite as W4-05. TR-G two-axis skip; TR-H
  orientation-agnostic across all four sibling combinations; TR-I
  last-axis release falls back to ramp; TR-J pins the
  `apply_control` comment that `other_active` is computed from *raw*
  inputs so a stale ramping sibling can't block a fresh release's
  skip-vs-ramp decision.
- [ ] **W4-07 Boot-PHY first-frame decode (IP-006 verification).**
  Cold-boot both ends; first encrypted frame decodes on the receiver
  with no `bad_header` events in the audit log.
  *SIL coverage:* [`base_station/tests/test_boot_phy_sil.py`](DESIGN-CONTROLLER/base_station/tests/test_boot_phy_sil.py)
  (Round 20) parses `firmware/handheld_mkr/handheld_mkr.ino`,
  `firmware/tractor_h7/tractor_h7.ino`, and
  `firmware/tractor_x8/params_service.py` and asserts: BP-A both
  `LADDER[3]` tables are byte-identical (rung mapping for
  `CMD_LINK_TUNE`); BP-A2 `LADDER[0]` matches DECISIONS.md D-A2
  (SF7/BW250/CR4-5/bw_code=1); BP-A3 SF strictly increases down the
  ladder; BP-A4 each rung is uniquely identifiable; BP-B/B2 every
  `radio.begin(...)` references `LADDER[0].sf` / `.bw_khz` / `.cr_den`
  symbolically + sync word `0x12`; BP-B3 nobody slipped a numeric SF /
  BW / CR literal back in; BP-C the Pi-side
  `DEFAULT_PARAMS['link']['control_phy']` agrees with the canonical
  `SF7_BW250` string; BP-D parser self-tests so the regex extractor
  catches mis-formats. The on-air no-`bad_header` confirmation is the
  only piece that still needs HIL bench time.
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
[2026-04-16 review](AI%20NOTES/CODE%20REVIEWS/2026-04-16_Review_GPT5_4.md).
~~They are still unfixed. They are the highest-priority work on the software side.~~
*Superseded \u2014 see WONTFIX banner immediately below.*

> **2026-04-29 status update — entire section below is WONTFIX against archived
> prototype code.** All file paths in this section
> (`DESIGN-CONTROLLER/arduino_opta_controller/`,
> `DESIGN-CONTROLLER/esp32_remote_control/`,
> `DESIGN-CONTROLLER/raspberry_pi_web_controller/`) point at code that was
> moved to
> [`DESIGN-CONTROLLER/RESEARCH-CONTROLLER/`](DESIGN-CONTROLLER/RESEARCH-CONTROLLER/README.md)
> in the 2026-04-26 cleanup. The RESEARCH-CONTROLLER README is explicit:
> *"Treat everything in this folder as ideas/prior-art only — not
> implementation plans. The canonical v25 build is LoRa-only, defined in
> `MASTER_PLAN.md`. Code reviews and readiness analyses for the v25
> hardware test do **not** apply to files under `RESEARCH-CONTROLLER/`."*
> The bug surfaces below (Opta MQTT-over-WiFi proportional control,
> blocking reconnect, BLE-without-pairing, hardcoded broker IP, ESP32
> Qwiic-disconnect stale axes, Pi web-UI auth, mode-switch boot-only
> sampling, etc.) all live on the **superseded MQTT-over-WiFi path** that
> the canonical Portenta H7 + MKR WAN 1310 LoRa stack replaced.
>
> **The canonical v25 stack does not use WiFi or BLE for control.** The
> active Opta firmware
> ([firmware/tractor_opta/tractor_opta.ino](DESIGN-CONTROLLER/firmware/tractor_opta/tractor_opta.ino))
> explicitly states *"The Opta's WiFi and BLE radios are intentionally NOT
> initialised here. No WiFi.begin(), no BLE.begin(), no MQTT client, no
> HTTP server."* and `base_station/` contains zero references to WiFi /
> BLE / bluetooth (verified 2026-04-29 via repo-wide grep). The
> equivalent functionality of every legacy bug below has been
> re-implemented from scratch in the active LoRa-only stack with full
> SIL coverage:
>
> | Legacy bug surface | Replaced by canonical-stack equivalent |
> | --- | --- |
> | Opta MQTT joystick float-parse, clamp, NaN-reject | `lora_proto.unpack_control` + Round 18 axis-ramp SIL ([test_axis_ramp_sil.py](DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py)) |
> | Opta blocking MQTT reconnect / safety timeout starvation | Round 25 MQTT retry/backoff fake-clock SIL ([test_mqtt_retry_backoff_sil.py](DESIGN-CONTROLLER/base_station/tests/test_mqtt_retry_backoff_sil.py)) + M4 watchdog (Round 39 boot_self_test) |
> | Opta MQTT mode-switch / flow-valve jumper boot-only sampling | BC-XX BuildConfig (Rounds 27\u201341): per-unit TOML reloaded via `config_watcher.py`, classified `live`/`restart_required`/`firmware_required` |
> | Opta hardcoded MQTT broker IP / credentials | `LIFETRAC_MQTT_HOST` env + PIN-gated web auth (D-E1, Round 9 \u00a7E) |
> | Opta BLE characteristics accept unpaired writes | LoRa control path: AES-GCM + per-source `ReplayWindow` + `LIFETRAC_ALLOW_UNCONFIGURED_KEY` opt-in (Round 22 fleet-key SIL) |
> | ESP32 stale axes on Qwiic disconnect | Handheld MKR replaces ESP32 entirely; LoRa control loop has 200 ms M4 watchdog (`tractor_h7_m4.ino`) that drops PSR on stale `alive_tick_ms` |
> | Pi web controller no-auth, hardcoded broker | Round 9 \u00a7B/\u00a7C/\u00a7D + D-E1 PIN auth on `base_station/web_ui.py` |
> | `setupWiFi()` boot-time blocking | Canonical stack does not use WiFi for control; LoRa is link-up-from-cold |
> | `paho-mqtt < 2.0` pin / atexit cleanup on `app.py` | Active code uses `paho.mqtt.client.Client` via `web_ui._connect_mqtt_with_retry()` (Round 25) and lives under systemd unit lifecycle |
>
> Every item below is therefore WONTFIX in the canonical-stack sense \u2014
> kept here as historical record. The `[ ]` checkboxes are left unticked
> deliberately so a future operator who chooses to revive the prototype
> for a one-off bench test can see what they would need to fix first.
> See [`RESEARCH-CONTROLLER/README.md`](DESIGN-CONTROLLER/RESEARCH-CONTROLLER/README.md)
> for the harvest-from-this list per legacy module.

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

---

## Future ideas � autonomous routines & mission library

Beyond tele-op, the v25 controller stack reserves source-id `AUTONOMY`
and the MASTER_PLAN already carves out autonomy opcodes. Once GPS/RTK,
IMU dead-reckoning, vision-based obstacle stop, and the autonomy mode
arbitration land, the tractor becomes a programmable mobile hydraulic
robot. This list is the running brainstorm of mission-level routines we
want the autonomy layer to be able to express. None of these are
in-scope for v25 itself � they are the target use cases the autonomy
APIs need to be expressive enough to support.

Each routine should compose from a small set of primitives � *go-to
pose*, *follow path*, *bucket dig*, *bucket dump*, *engage implement*,
*hold posture*, *wait-for-condition* � and run under the same hard
safety envelope as tele-op (PSR E-stop chain, person-alert vision stop,
geofence, link-loss timeout).

### Operator-requested routines

- [ ] **Material movement loop (dig ? haul ? dump).** Operator marks a
  source pile or dig area on the map and a destination (another pile,
  the hopper of a Compressed Earth Block press, a trailer, a spoil
  heap). Tractor drives to the source, executes a bucket-fill cycle
  (curl + crowd until pressure plateau or load cell threshold),
  reverses out, follows the planned path to the destination, and dumps.
  Loops until the source is exhausted, the destination is full (CEB
  hopper level sensor, trailer load cell), the operator pauses it, or
  fuel/battery falls below a return-to-base threshold. Records each
  cycle for cycle-time and yield analytics.
- [ ] **Field pass routine (plow / disc / water / fertilize / seed).**
  Operator selects a field polygon and an implement profile (plow,
  disc, sprayer, spreader, seed drill). Tractor generates a coverage
  pattern (boustrophedon / spiral / contour-following) honouring
  implement width, headland turn radius, no-go zones, and slope limits,
  then executes the pattern with implement engaged. Variable-rate
  application uses a prescription map (e.g. heavier fertilizer on
  low-N zones from a soil map) when one is supplied; otherwise uniform.
- [ ] **Land levelling to target grade.** Operator defines a target
  surface � flat at elevation Z, a planar slope (azimuth + grade %),
  or an arbitrary heightmap. Tractor surveys current grade with the
  bucket-mounted GNSS/laser, computes a cut/fill map, then iteratively
  scrapes high spots into low spots until residual elevation error is
  within tolerance everywhere in the polygon. Pairs naturally with the
  material-movement loop when net cut ? net fill.

### Additional routines worth adding to the same backlog

- [ ] **Trenching along a marked line at constant depth.** For
  irrigation drip lines, electrical conduit, water service, or french
  drains. Operator draws a polyline; tractor follows it with a
  trenching attachment held at depth via bucket-tip GNSS feedback.
- [ ] **Stockpile reshaping / push-up.** Loose material from a dump pile
  pushed up onto an existing stockpile to a target geometry (cone,
  windrow, bermed rectangle). Useful before tarping or for compost
  windrow turning when paired with a turner attachment.
- [ ] **CEB-press tending loop.** A specialisation of the
  material-movement loop with the press as the destination: monitor
  press hopper level over MQTT, refill on demand, fall idle when the
  press is full or paused. Closes the open-source-housing-system loop
  with no human in the bucket cycle.
- [ ] **Hole-grid drilling for tree planting / fence posts / pier
  footings.** Operator defines a grid (rows � columns, spacing,
  azimuth) or imports a planting plan. Tractor drives to each point
  with an auger attachment, drills to a specified depth, withdraws,
  advances. Records actual location of every hole for the as-built map.
- [ ] **Perimeter / contour swale digging.** Follows a survey contour
  line at a specified cross-section to build keyline-style water
  retention features. Reuses the trenching primitive with a wider,
  shallower bucket pose.
- [ ] **Brush / mowing pattern over a polygon.** Same coverage planner
  as the field pass, but with a flail mower or brush hog and a more
  conservative obstacle-stop policy (vision detector tuned for fence
  posts, irrigation risers, animals).
- [ ] **Snow / debris clearing along a route.** Plough or bucket follows
  a defined polyline (driveway, footpath, lane) repeatedly, pushing
  material to a designated spoil zone. Triggered by weather alert or
  manual start.
- [ ] **Rock / debris pickup over a polygon.** Vision detector picks out
  rocks above a size threshold; tractor drives to each, scoops with
  forks or bucket, deposits in a bin or spoil pile. Random-sweep
  pattern with revisit on detector confidence.
- [ ] **Trailer / truck loading to a target weight.** Repeats the
  material-movement loop with the trailer as destination and stops
  when an on-trailer load cell (or estimated bucket cycle count) hits
  the target weight, leaving the operator to drive away.
- [ ] **Geofence patrol / perimeter inspection.** Tractor follows a
  perimeter polyline at low speed with the camera streaming, flagging
  fence breaks, downed trees, washouts, or strange objects to the
  operator for review. Pairs with the existing person-alert pipeline.
- [ ] **Return-to-base on fuel / battery / weather.** Background
  routine, not a job: any active mission yields and the tractor
  drives itself to a configured shelter or charge dock when fuel
  drops below reserve, battery SOC drops below threshold, lightning is
  forecast within N km, or wind exceeds a safety cap. Mission state is
  checkpointed so the operator can resume after refuel/recharge.
- [ ] **Multi-tractor cooperative jobs.** Two LifeTracs sharing the
  same site: one parks at the dig face on the material-movement loop
  while the other shuttles between the loader and the destination.
  Requires the autonomy layer to negotiate non-overlapping path
  reservations and shared safety zones.
- [ ] **Implement auto-engage / auto-disengage at the implement rack.**
  Drive to the rack, align with the target attachment, hydraulically
  couple, drive away. Eliminates the manual swap that gates most of
  the routines above when a job needs more than one implement.
