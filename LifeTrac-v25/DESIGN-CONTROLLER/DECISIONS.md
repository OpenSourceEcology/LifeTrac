# LifeTrac v25 Controller — Open Decisions & Picks

**Date:** 2026-04-27
**Status:** Source of truth for the Phase A blocker decisions and the Phase C crypto-library choice. When a pick is recorded here, the corresponding code change is made and `LORA_PROTOCOL.md` / `MASTER_PLAN.md §8.17` is updated to match.

This document discharges the **six bring-up blockers** flagged in the v25 implementation-readiness audit. Each section enumerates the realistic options, scores them, and records the chosen path. Anything labelled **PICK** has been implemented in this commit; anything labelled **DEFERRED** still needs user sign-off or bench measurement.

---

## D-A2 — Control-cadence airtime (was `CONTROL_CADENCE_BLOCKER`)

**Problem.** Encrypted `ControlFrame` = 16 cleartext + 12 nonce + 16 GCM tag = **44 B on air**. At SF7 / BW125 / CR 4-5 that is **~92 ms**. Target control cadence is 20 Hz (50 ms window). At any duty above ~50 % we fall behind every other frame and joystick latency jitters.

| Option | Air time | Range vs SF7/BW125 | Cadence fits? | Risk | Cost |
|---|---|---|---|---|---|
| A. **Control PHY → SF7/BW250** | ~46 ms | -3 dB (≈30 % range loss) | ✅ 20 Hz | Lower link margin in trees / behind buildings | One constant change, zero code. |
| B. Drop control cadence to 10 Hz | ~92 ms (unchanged) | unchanged | ✅ 10 Hz | Joystick feel degrades; operators report "mushy" | Operator-facing UX cost. |
| C. Trim header overhead (drop nonce to 8 B + reuse) | ~80 ms | unchanged | ❌ still > 50 ms | Breaks AES-GCM nonce uniqueness; security regression | Big spec change. |
| D. Two-radio split (control vs telemetry on separate SX1276) | ~46 ms (parallel) | unchanged | ✅ 20 Hz | Two more antennas, two more SPI buses, BOM creep | Hardware change. |

**PICK: Option A — switch `LP_PHY_CONTROL_SF7` to BW 250 kHz.** BW125 stays as the SF8/SF9 fallback rungs in the SF ladder so we still degrade gracefully when the ~3 dB margin matters; in clean line-of-sight the 20 Hz path runs at SF7/BW250 with ~4 ms of headroom per slot. This is one constant in `firmware/common/lora_proto/lora_proto.c` plus the same constant in `base_station/lora_proto.py`, plus the SF ladder rung 0 in `firmware/tractor_h7/tractor_m7.ino` and `firmware/handheld_mkr/handheld.ino`.

**Verification.** Unit test `test_encrypted_control_airtime_fits_20hz_cadence` asserts the encrypted control airtime is ≤ 50 ms at the new profile. R-7 retune-cost bench (separate task) must still pass for the SF7→SF8 ladder transitions.

---

## D-A3 — Image fragment airtime cap (was `IMAGE_FRAGMENT_BLOCKER`)

**Problem.** A 32 B image fragment at SF7 / BW250 estimates at **~36 ms**, but `LORA_IMPLEMENTATION.md §4` caps any single P2/P3 fragment at **25 ms** so a fragment never preempts a P0 control window.

| Option | Air time | Range vs SF7/BW250 | Throughput per slot | Cost |
|---|---|---|---|---|
| A. Shrink fragment to ~15 B | ~22 ms | unchanged | 60 B/s @ 4 Hz refresh | More fragments → more headers → worse goodput. |
| B. Raise the cap to 40 ms | ~36 ms | unchanged | 128 B/s @ 4 Hz | P0 frame can be delayed up to 40 ms by an in-flight P3. |
| C. **Image PHY → SF7/BW500** | ~18 ms (32 B) | -3 dB more | 128 B/s @ 4 Hz | Image-only PHY change; control PHY untouched. |

**PICK: Option C — switch `LP_PHY_IMAGE` to BW 500 kHz.** Image is best-effort, fragment loss already triggers `CMD_REQ_KEYFRAME`, and the image link auto-fallback ladder (encode-mode controller) handles degraded SNR by demoting `full → y_only → motion_only → wireframe`. Keeps the 25 ms cap intact, keeps the P0 preemption guarantee intact.

**Verification.** Unit test `test_image_fragment_fits_25ms_cap_at_bw500` asserts a 32 B payload fits the cap at the new profile.

---

## D-A1 — R-7 retune-cost benchmark **(DEFERRED — bench access required)**

Cannot run from a coding session. The bench sketch [firmware/bench/lora_retune_bench/lora_retune_bench.ino](LifeTrac-v25/DESIGN-CONTROLLER/firmware/bench/lora_retune_bench/lora_retune_bench.ino) is in tree and ready to flash. Acceptance: `setFrequency + setSpreadingFactor + setBandwidth + setCodingRate` round-trip ≤ 5 ms on the SX1276 via RadioLib. **Failure forces a burst-batching code path** in `tractor_m7.ino` so the radio retunes at most twice per refresh window. Track in `firmware/bench/lora_retune_bench/RESULTS.md` once a board is on the bench.

---

## D-C1/C2 — Real AES-128-GCM library choice

The v25 plan currently builds against `crypto_stub.c` (returns true, copies bytes). Production must use a real implementation. We have one library choice per Arduino target.

### Tractor H7 / Portenta X8 onboard co-MCU

| Option | Source | Vetted | Footprint | Notes |
|---|---|---|---|---|
| **MbedTLS `mbedtls/gcm.h`** | Already linked into Arduino mbed-os core | NIST CAVP | ~6 KB ROM | First-class API on Portenta. Hardware-accelerated AES on STM32H7 via mbed-os crypto plugin. |
| ArduinoBearSSL | Arduino-maintained | Yes | ~14 KB ROM | Heavier; designed for TLS, GCM via `br_aes_ct64_ctr` + manual GHASH. |
| rweather/Crypto | Arduino library | Independent | ~4 KB ROM | Header-only, easy. Slower on H7 (no hw accel). |

**PICK: MbedTLS** (`mbedtls/gcm.h`). Already linked, NIST-vetted, hardware-accelerated.

### Handheld MKR WAN 1310 (SAMD21)

| Option | Source | Vetted | Footprint | Notes |
|---|---|---|---|---|
| ArduinoBearSSL | Arduino-maintained | Yes | ~14 KB ROM | Big for SAMD21 (256 KB flash); GCM not in headline API. |
| **rweather/Crypto** (`AES128` + `GCM<AES128>`) | Arduino library | Independent | ~4 KB ROM | Targets SAMD21 specifically. Used widely. |
| Hand-port MbedTLS | Manual | NIST CAVP | ~6 KB ROM | Time sink to wire the build. |

**PICK: rweather/Crypto.** Compact, designed for AVR/SAMD-class MCUs, exactly the AES-GCM shape we need.

### Python base-station

`cryptography.hazmat.primitives.ciphers.aead.AESGCM` is **already in use** in `base_station/lora_proto.py`. No change.

### Implementation strategy

Add `firmware/common/lora_proto/lp_crypto_real.cpp` with `#ifdef` selection on `ARDUINO_ARCH_MBED` (Portenta) → MbedTLS, `ARDUINO_SAMD_MKRWAN1310` → rweather/Crypto. Existing `crypto_stub.c` stays in the tree but compiles only when `LIFETRAC_ALLOW_STUB_CRYPTO` is defined (CI / unit-test builds). Production firmware MUST set `-DLIFETRAC_USE_REAL_CRYPTO`.

**Status:** scaffolding + library declaration land in this commit; the actual MbedTLS / rweather hookup is one focused follow-up commit per platform with golden-vector cross-check vs `cryptography.AESGCM`. Tracked as **task C-real-crypto**.

---

## D-C5 — Reciprocal `CMD_LINK_TUNE` handlers

Already implemented this session — handheld's `apply_phy_rung()` retunes its SX1276 on inbound `CMD_LINK_TUNE`, and `lora_bridge.py` publishes the retune intent on `lifetrac/v25/control/link_tune` for the future SPI driver. **Closed.**

---

## D-C6 — CSMA `scanChannel` wiring

**Problem.** `lp_csma_pick_hop()` exists with passing tests, but no firmware caller invokes a real `RadioLib::scanChannel()` — the helper is wired only in unit-test mocks.

| Option | Pros | Cons |
|---|---|---|
| Wire it everywhere now | Future-proof | Adds latency to every TX (≈5–10 ms `scanChannel` per probe). Today the system is single-channel by default; no benefit until FHSS rollout. |
| **Wire it as an opt-in path gated by `LIFETRAC_FHSS_ENABLED`** | Code-complete; latency only paid when FHSS is on | Two TX paths to keep tested. |
| Defer to FHSS rollout | Less code now | Risk of forgetting; loses CSMA on the noisy 915 ISM. |

**PICK: opt-in via `#ifdef LIFETRAC_FHSS_ENABLED`.** Keeps the single-channel default fast and the FHSS variant correct. Lands as a thin adapter `radio_scan_channel_dbm()` in both `tractor_m7.ino` and `handheld.ino`.

---

## D-E1 — Web-UI auth model

Pinned in `MASTER_PLAN.md §8.5`: shared PIN, LAN-only, plain HTTP for v25. **No options remain.** Implementation lands in this commit.

---

## D-E5 — Image-pipeline foundation

Deferred until D-A3 lands and the image-link ladder is bench-verified end-to-end. The bridge already emits `CMD_ENCODE_MODE` on rung change.

---

## Summary of picks landed in this commit

| ID | Pick | Files touched |
|---|---|---|
| D-A2 | SF7/BW250 control | `lora_proto.{c,h,py}`, `tractor_m7.ino`, `handheld.ino`, tests |
| D-A3 | SF7/BW500 image | `lora_proto.{c,py}`, tests |
| D-C1 | MbedTLS on H7, rweather on MKR | `lp_crypto_real.cpp` scaffold + `arduino_libraries.txt` |
| D-C6 | `LIFETRAC_FHSS_ENABLED` opt-in CSMA | `tractor_m7.ino`, `handheld.ino` |
| D-E1 | PIN auth | `web_ui.py` |
| D-CORAL-1..6 | Optional Coral Edge TPU (canonical `accel_select.py`, three-level state, default-on, base-local JSON store, 30 s poll + manual rescan, single master toggle) | `image_pipeline/{accel_select,superres,detect}.py`, `settings_store.py`, `web_ui.py`, `web/settings.html`, `CORAL.md`, `IMAGE_PIPELINE.md §14` |

---

## D-CORAL — Optional Coral Edge TPU integration

**Context:** [MASTER_PLAN.md §8.19](MASTER_PLAN.md), [IMAGE_PIPELINE.md §5.1](IMAGE_PIPELINE.md), [HARDWARE_BOM.md Tier 2](HARDWARE_BOM.md). The Coral M.2 / USB Accelerator is on the BOM as an *optional* line item; the v25 image pipeline must validate CPU-only and treat the Coral as a strict additive upgrade.

### D-CORAL-1 — Where does detection live?

| Option | Pros | Cons |
|---|---|---|
| A. `base_station/image_pipeline/accel_select.py` (canonical per IMAGE_PIPELINE §5.1) | Already specified; co-located with the consumers (`superres.py` / `detect.py`) | None |
| B. `base_station/coral.py` | Slightly shorter import | Off-spec; would need a doc update |
| C. `firmware/tractor_x8/coral_service.py` | Mirrors the tractor service-scaffold layout | Wrong machine — Coral lives on the *base*, not the tractor |

**PICK: A.** Matches the canonical layout already documented in IMAGE_PIPELINE.md §5.1. Implemented in [`base_station/image_pipeline/accel_select.py`](base_station/image_pipeline/accel_select.py).

### D-CORAL-2 — How many state levels does the UI pill represent?

| Option | Pros | Cons |
|---|---|---|
| A. Two: on / off | Simplest | Cannot distinguish "no hardware" from "operator turned it off" from "driver missing" — operators can't troubleshoot |
| B. **Three: `present` / `usable` / `enabled`** | One-to-one with the four pill states (active / disabled / driver-missing / absent) operators actually need | One extra field |
| C. Five: + `warming_up` + `thermal_throttled` | Covers future thermal-throttle work | Premature; thermal handling is v26 (see CORAL.md §6) |

**PICK: B.** Implemented as `AcceleratorState{present, usable, enabled, kind, last_error, detected_at}` with `is_active() = present and usable and enabled`.

### D-CORAL-3 — Default value of the operator switch

| Option | Pros | Cons |
|---|---|---|
| A. `True` (auto-on when present) | Matches MASTER_PLAN §8.19 "automatic upgrade when present, zero functional dependency when absent" | Operator surprised if Coral is overheating on first boot |
| B. `False` (opt-in) | Conservative; operator must consciously enable | Most installs will silently leave the upgrade on the table |

**PICK: A.** Boot-time default also configurable via `LIFETRAC_CORAL_ENABLED` env var; operator override persists in [`base_settings.json`](base_station/settings_store.py).

### D-CORAL-4 — Where is the operator override persisted?

| Option | Pros | Cons |
|---|---|---|
| A. New base-local `base_station/settings_store.py` (JSON at `/var/lib/lifetrac/base_settings.json`) | ~30 LOC; no cross-machine coupling; matches the fact that Coral is base-only hardware | Yet another tiny store |
| B. Round-trip through tractor `params_service.py` over MQTT | Single settings system | Couples a base-local UI choice to the tractor; surprising failure modes if MQTT is down |
| C. Env var only | No persistence code | Lost on reboot; ops nightmare |

**PICK: A.** Implemented in [`base_station/settings_store.py`](base_station/settings_store.py).

### D-CORAL-5 — Detection cadence

| Option | Pros | Cons |
|---|---|---|
| A. udev hot-plug events | Instant | Linux-only; pulls in pyudev; doesn't run on the Windows dev box |
| B. **30 s background poll + manual "Re-scan" button** | Portable; cheap (lspci + lsusb are <50 ms); operator can force-refresh after hot-plug | 30 s worst-case latency before the pill flips after unplug |
| C. On-demand only (every UI request) | No background work | UI lag spikes; doesn't tell autonomy when accel is lost |

**PICK: B.** Per-call `is_active()` check in the dispatchers covers the inference-side hot-unplug case within one frame regardless.

### D-CORAL-6 — Per-feature toggles?

| Option | Pros | Cons |
|---|---|---|
| A. Single master switch | Simple; matches v25 single-Coral-consumer reality | Blunt instrument once super-res, detect, motion-replay all want Coral |
| B. Per-feature toggles (super-res, detect, ...) | Fine-grained | Premature — only one consumer ships in v25, and the feature hasn't been proven on Coral yet |

**PICK: A.** Per-feature is a v26 setting; tracked in CORAL.md §6.

---

## D-HYD1 - Actuator soft-stop strategy (EFC ramp + valve settling)

**Context:** [DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md](../DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md), [DESIGN-HYDRAULIC/HYDRAULIC_BOM.md](../DESIGN-HYDRAULIC/HYDRAULIC_BOM.md) Sec. 3, [firmware/tractor_h7/tractor_h7.ino](firmware/tractor_h7/tractor_h7.ino) IP-303 block, [firmware/common/lifetrac_build_config.h](firmware/common/lifetrac_build_config.h).

| Option | Pros | Cons |
|---|---|---|
| A. Float-centre spool (D1VW00**4**CNKW) + 2x PO check valves on cylinders | Track motors free-coast on release; OSE-legacy pattern | Pump full-flow bypass at idle through EFC EX port (heat); cylinders need extra hardware; no hill-hold on tracks |
| **B. Tandem-centre spool (D1VW00*8*CNKW) + EFC ramp + valve settling delay** | Natural cylinder load-hold (drop 2 PO checks); near-zero idle pressure (cool); track hill-hold; uses existing EFC + Burkert + IP-303 ramp infrastructure | Requires firmware sequencer (BC-18); no free track coast on release (decel follows ramp ladder) |
| C. Closed-centre spool + accumulator | True hold everywhere | Adds accumulator hardware; no benefit over B |

**PICK: B.** Adopted 2026-04-29. Detailed timing, safety properties, and tuning parameters in [DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md](../DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md). Firmware work tagged **BC-18** (settling-timer addition to `tractor_opta.ino` valve-drive loop + new `hydraulic.valve_settling_ms` BuildConfig leaf + SIL gate `test_valve_settling_sil.py`). Until BC-18 ships, populate field hardware with the float-centre interim bill (Option A) and migrate at next service interval.

## D-HYD2 - Hydraulic build-variant configurability

**Context:** Other LifeTrac builders won't all use the canonical v25 hydraulic stack. The firmware sequencer must be data-driven, not hard-coded. Same pattern as `safety.estop_topology` already established in [base_station/config/build_config.schema.json](base_station/config/build_config.schema.json).

| Option | Pros | Cons |
|---|---|---|
| A. Hard-code v25 canonical (tandem + 100ms settling) in firmware | Smallest code; matches v25 BOM exactly | Other builders must fork firmware; OSE-legacy float+PO-check builds break with current behaviour after BC-18 |
| **B. Three new BuildConfig leaves under `[hydraulic]`: `spool_type` (enum tandem/float/closed/open), `load_holding` (enum spool_inherent/po_check/counterbalance/none), `valve_settling_ms` (uint 0..250)** | Mirrors existing `estop_topology` pattern; ~30 lines of firmware branch + 3 schema leaves cover all four spool types; validator catches incompatible combinations | One more SIL matrix dimension; `lifetrac-config self-test` gains two new compatibility rules |
| C. Compile-time only via lifetrac_build_config.h (no schema/TOML/codegen) | Slightly less code | Loses the per-unit override + bundle/verify flow that all other config goes through; off-pattern |

**PICK: B.** Adopted 2026-04-29 alongside D-HYD1. Schema work tagged **BC-19** (separate from BC-18 firmware sequencer); BC-18 ships hard-coded with the canonical v25 defaults, BC-19 makes them configurable. Three reference builds (OSE-legacy, v25-canonical, high-performance) documented in [DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md → Hydraulic variants & per-build configuration](../DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md#hydraulic-variants--per-build-configuration). The two firmware branch points are listed there: `tractor_opta.ino` valve-drive loop + `lifetrac-config self-test` compatibility rules.

## D-HYD3 - IMU-adaptive ramp tuning (jerk feedback)

**Context:** The canonical ramp constants are starting points; optimal values vary with payload, slope, temperature, pump wear. The BNO086 IMU on [HARDWARE_BOM.md](HARDWARE_BOM.md) Tier-1 publishes 100 Hz linear acceleration; computing jerk (`d(accel)/dt`) gives an objective measure of ramp quality. Question: should we use it to auto-tune the soft-start code?

| Option | Pros | Cons |
|---|---|---|
| A. Ignore IMU; ramp values stay static (only manual operator tuning via `lifetrac-config push`) | Simplest; no new code | Operators have no objective signal; "feels rough" stays subjective; suboptimal ramps unnoticed |
| B. Passive jerk telemetry only (BC-20A) | Pure observability; safe to ship anytime; large diagnostic value with small implementation cost | Operator still has to interpret + manually edit the config |
| C. Passive telemetry + advisory recommendation engine (BC-20A + BC-20B), human-in-the-loop via existing `lifetrac-config validate/bundle/verify/push` flow | All the safety gates of static config delivery already exist (schema validate, audit log, reload-class, operator approval); recommendations refuse out-of-range writes pre-validation | Slightly more SIL surface |
| **D. Three-stage rollout: BC-20A passive telemetry, then BC-20B advisory, then BC-20C closed-loop online tuning gated by new `hydraulic.adaptive_ramp_tuning` bool (default false, opt-in, panic revert button, BC-12 HIL prerequisite)** | Captures the value of each stage independently; ships safely incrementally; closed-loop disabled by default matches IEC-62443 + ISO-25119 reviewer expectations for adaptive control on a safety parameter | Three rounds of work instead of one |
| E. Skip straight to closed-loop online tuning (no advisory stage) | One round; full automation | SIL cannot validate stability under all payload/slope/temperature combinations; needs HIL data we don't have yet; high blast radius on regression |

**PICK: D.** Adopted 2026-04-29. Three sub-rounds:

* **BC-20A passive jerk telemetry** - M7 publishes per-transition peak jerk over existing telemetry channel; web UI diagnostics surfaces; no control change. Safe to ship anytime.
* **BC-20B advisory recommendation engine** - base-station accumulates per-build histograms, recommends ramp-leaf edits via existing config-delivery flow; human approves; no autonomous writes.
* **BC-20C closed-loop online tuning** - new `hydraulic.adaptive_ramp_tuning` schema leaf (bool, default false, `restart_required`); +-5%% step adjustments per accepted clean transition, schema-bounded, audit-logged, panic revert. Disabled by default; requires BC-12 HIL bench validation before enabling.

Detailed rationale and SIL surface breakdown in [DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md](../DESIGN-HYDRAULIC/SOFT_STOP_STRATEGY.md) section "IMU-adaptive ramp tuning".
