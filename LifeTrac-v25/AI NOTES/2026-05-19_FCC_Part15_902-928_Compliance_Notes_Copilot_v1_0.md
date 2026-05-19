# FCC Part 15 — 902-928 MHz Compliance Notes for the X8↔L072 LoRa Link

**Author:** Copilot (assistant-authored research notes)
**Date:** 2026-05-19
**Status:** v1.0 — initial finding, not yet ratified by a human reviewer
**Scope:** Defines the regulatory envelope our 915 MHz Murata SX1276/L072 link
operates inside; identifies the gap between the current firmware/bench config
and what is required for a US field deployment; lists three architectural
options and a recommended path.

---

## 0. Precedence and disclaimer

This document summarises **47 CFR Part 15, Subpart C**, primarily §15.247 and
§15.249, as fetched from eCFR on 2026-05-19 (eCFR shown as "up to date as of
2026-05-15"). The agent quoted the rule text directly during the research
session; the citations below are the source of truth, not this document.

Per the repo precedence rule (code > evidence > doc), **this file is a
research summary, not an authority**. Before any commercial release the firm
must obtain an independent regulatory review (test house, FCC consultant, or
licensed compliance engineer). Bench/lab development under §15.5
(experimental) is OK in the meantime.

Verified live on bench 2026-05-19: both X8+L072 boards return
`RegFrf=0xE4C000 → Fcarrier = 915.000 MHz` (see
[regfrf_check.py](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/regfrf_check.py)
and [bench-evidence/walk_power_pilot_2026-05-19/README.md](../DESIGN-CONTROLLER/bench-evidence/walk_power_pilot_2026-05-19/README.md)).

---

## 1. The three legal regimes in 902-928 MHz

| Regime | Cite | Channel/system bandwidth | Max conducted power | Hopping required? | Per-channel dwell/budget |
|---|---|---|---|---|---|
| §15.247(a)(1) FHSS — narrow | §15.247(a)(1)(i) | 20 dB BW **< 250 kHz** | **+30 dBm** (1 W) | **Yes, ≥ 50 channels** | ≤ 400 ms / channel / **20 s** |
| §15.247(a)(1) FHSS — wide | §15.247(a)(1)(i) | 20 dB BW **250–500 kHz** | **+24 dBm** (0.25 W) | **Yes, ≥ 25 channels** | ≤ 400 ms / channel / **10 s** |
| §15.247(a)(2) DTS (digital modulation) | §15.247(a)(2), (b)(3), (e) | **6 dB BW ≥ 500 kHz** | **+30 dBm** (1 W) | **No** | None — but PSD ≤ 8 dBm / 3 kHz |
| §15.249 low-power | §15.249(a) | None | Field-strength rule: **50 mV/m @ 3 m ≈ +13 dBm EIRP** | **No** | None |

Power limits in §15.247(b) are stated assuming antenna gain ≤ 6 dBi; if the
antenna exceeds 6 dBi, the conducted power must be reduced 1 dB per dB of
excess gain (§15.247(b)(4)). §15.249 is an **EIRP/field-strength** rule, so
antenna gain trades directly against conducted power.

Common-to-all §15.247 rules (do not depend on which sub-regime):
* Channel separation ≥ 25 kHz, or the 20 dB BW of the hopping channel,
  whichever is greater (§15.247(a)(1)).
* Hopping pattern must be pseudo-random; "each frequency must be used equally
  on the average" (§15.247(a)(1)).
* Receivers must follow transmitters' hops in synchronisation
  (§15.247(a)(1)).
* The transmitter must be **designed** to satisfy the rules under a
  continuous-data worst case, even if it actually transmits in short bursts
  (§15.247(g)). It is OK to skip channels at run time as long as the
  *design* could meet the minimum-channel and dwell rules if asked to.
* "Listen-before-talk to avoid coordinating with other §15.247 users" is
  explicitly **not allowed** (§15.247(h)). Quality-aware FHSS that
  individually avoids noisy channels *is* allowed.
* Out-of-band: ≥ 20 dB below in-band level in any 100 kHz outside the band
  (§15.247(d)).

---

## 2. Where the current LifeTrac config falls

Measured 2026-05-19 on the bench:
* Modulation: LoRa SF7 / **BW125** / CR4-5 / sync 0x12.
* RegFrf = 0xE4C000 → **915.000 MHz** carrier, both boards. Single channel.
* PA path: PA_BOOST, requested **+14 dBm** conducted (per S1.1 walk_power
  pilot). +17 dBm and +20 dBm steps also demonstrated.
* Antenna: nominal 2 dBi whip on each X8/Max Carrier.

Implications:

| Test | Result |
|---|---|
| 6 dB BW ≥ 500 kHz? (DTS qualifier) | **No** — SF7/BW125 has 6 dB BW ≈ 100 kHz |
| 20 dB BW < 250 kHz? (narrow FHSS regime) | **Yes** — 20 dB BW ≈ 125 kHz |
| FHSS narrow regime: ≥ 50 channels in use? | **No** — single channel only on the bench |
| §15.249 envelope: total EIRP ≤ +13 dBm? | **No** — +14 dBm conducted + ~2 dBi antenna ≈ +16 dBm EIRP |

⇒ **The current bench config is not §15.247(a)(1) compliant (single
channel) and not §15.249 compliant (EIRP too high) and not §15.247(a)(2)
compliant (BW too narrow). It is OK only as a short-duration lab/development
exercise.**

The firmware comment in
[config.h](../DESIGN-CONTROLLER/firmware/murata_l072/config.h#L9)
promises an "8-channel FHSS baseline (R-01); quality-aware FHSS (N-06)
post-launch". **8 channels is below the 25-channel floor** (which would
require BW ≥ 250 kHz anyway) and below the 50-channel floor (the only one
that fits BW125 today). So the planned FHSS baseline is also non-compliant
as currently scoped.

---

## 3. Three architectural options

### Option A — §15.249 (low power, single channel)

* Drop conducted power to ~**+10 dBm** (giving roughly +13 dBm EIRP with a
  2 dBi antenna).
* Keep one channel (e.g. 915.000 MHz) forever. No FHSS, no airtime budget,
  no dwell logic.
* Pros: simplest software path. The existing firmware already supports any
  TX power down to ~ +2 dBm via the walk_power probe.
* Cons: loses ~ 4 dB of link budget vs. +14 dBm and ~ 7 dB vs. +17 dBm. For
  open-field machine telemetry this materially shortens range.
* Open question: peak EIRP must not exceed average by more than 20 dB
  (§15.249(e)). LoRa preamble is a CW tone — verify with a spectrum
  analyser before committing.

### Option B — §15.247(a)(2) DTS (wider BW, single channel)

* Switch default modulation to **BW500**, choosing an SF that still meets
  link-budget needs (SF8/BW500 ≈ same sensitivity as SF7/BW125 → equal range;
  SF10/BW500 ≈ same sensitivity as SF9/BW125 → +6 dB).
* Verify 6 dB occupied bandwidth ≥ 500 kHz on the actual hardware (LoRa's
  occupied BW is slightly less than the configured BW; SF12/BW500 in
  particular can fall short — Semtech AN1200.26 covers this).
* Pros: no hopping logic at all; full +30 dBm headroom.
* Cons: BW500 is 4× wider than BW125 → ~ +6 dB noise floor; needs PSD audit
  (≤ 8 dBm in any 3 kHz). Slightly higher current draw.

### Option C — §15.247(a)(1) wide-FHSS (BW250, 25 channels) — **RECOMMENDED**

* Switch default modulation to **BW250** (20 dB BW ≈ 250 kHz → qualifies
  for the 25-channel relaxed FHSS rule).
* Configure **25 hopping channels** across 902-928 MHz with ≥ 25 kHz (or
  one 20-dB BW, whichever is greater) separation. 25 ch × 280 kHz pitch
  ≈ 7 MHz footprint, comfortably inside 26 MHz of available band.
* Hopping budget: ≤ 400 ms per channel per 10 s. Per packet airtime at
  SF7/BW250 ≈ 12.9 ms (half of BW125). At a round-robin hop rate, each
  channel sees 10 / 25 = 0.4 s of opportunity per 10 s → **dwell rule and
  hop rate are simultaneously satisfied at full duty** with no idle gap
  needed (zero margin). For realistic margin, hop every packet (12.9 ms
  dwell each) and use a pseudo-random permutation over 25 channels per
  hopset.
* Power: we sit at +14 dBm conducted — **10 dB below the +24 dBm ceiling**.
* Pros: re-uses the existing `CFG_KEY_FHSS_ENABLE`, `CFG_KEY_FHSS_CHANNEL_MASK`,
  `CFG_KEY_FHSS_DWELL_MS`, `CFG_KEY_FHSS_QUALITY_AWARE` scaffolding. Quality-
  aware skip-list (N-06) is explicitly allowed by §15.247(h). Full
  +14/+17 dBm range preserved. 3 dB sensitivity hit from BW125→BW250
  recoverable by SF7→SF8.
* Cons: requires per-channel airtime accounting (already partially present
  in [sx1276_airtime.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c)),
  RX-side hop synchronisation, and a hop sequence agreed between TX and RX.

### Comparison summary

| Property | A (15.249) | B (DTS BW500) | C (FHSS BW250 ×25) |
|---|---|---|---|
| Max conducted TX power | ~ +10 dBm | +30 dBm | +24 dBm |
| Hopping logic | none | none | required (25 ch round-robin) |
| Link budget vs. today | **-4 dB** | ≈ 0 (with SF bump) | ≈ -3 dB at SF7 / ≈ 0 at SF8 |
| Firmware delta | trivial (cap TX power) | medium (BW500 default, PSD audit) | medium-large (hop sequence, RX sync, dwell counter) |
| Existing scaffolding fits? | yes | partial | **yes — was already planned** |
| Compliance risk | low (clearly inside envelope) | medium (BW occupancy must be measured) | low (well-trodden path; LoRaWAN US915 is the proof) |

---

## 4. Falsifications of common shortcuts

These are claims a non-specialist might propose; documenting them here so we
don't re-relitigate them later.

| Claim | Verdict | Reason |
|---|---|---|
| "FCC has no duty-cycle limit on 902-928, so we can stay on one channel" | **False as stated** | True for §15.247(a)(2) DTS (BW ≥ 500 kHz) and for §15.249 (≤ +13 dBm EIRP). For narrow-band hopping operation §15.247(a)(1) explicitly requires *hopping*, not a duty cycle. The constraint is structural, not statistical. |
| "We can use LBT/CSMA to coordinate with other §15.247 users" | **False** | §15.247(h) explicitly prohibits coordinating to avoid simultaneous occupancy by multiple transmitters. Self-skip of noisy channels by a single device *is* allowed. |
| "8-channel FHSS is enough" | **False** | The lowest channel-count tier is 25 (and only at BW ≥ 250 kHz). |
| "We can use 25 channels at BW125" | **False** | The 25-channel tier requires 20 dB BW ≥ 250 kHz. BW125 forces the 50-channel tier. |
| "The internal 40 % airtime budget in `sx1276_airtime.c` makes us compliant" | **False** | That gate is a self-imposed fairness limiter on a single channel. It is unrelated to the per-channel 400 ms dwell rule and does nothing to satisfy the channel-count requirement. |
| "+14 dBm into a 2 dBi antenna is below the §15.249 limit" | **False** | EIRP ≈ +16 dBm > +13 dBm. We are out of §15.249 by ~ 3 dB at +14 dBm conducted. |
| "We can ignore Part 15 for an own-use prototype" | **Partly true** | §15.5 lets a developer experiment freely under their own responsibility, but transmissions still must not cause harmful interference, and the device cannot be marketed/sold without certification. |

---

## 5. Recommendation

1. **Bench / S1.4 / S1.5 work**: keep going on the single 915.000 MHz
   channel at +14 dBm. This is a short, controlled lab exercise and is the
   only way to surface a working power-adapter and burst design before
   committing to a hop architecture.
2. **Before any field trial outside the building**: implement Option C
   (25-channel FHSS on BW250) and re-run the same S1.4 and S1.5 evidence
   with hopping enabled.
3. **Promote the FHSS work item** from "post-launch (N-06)" to a pre-launch
   prerequisite in `TODO.md`. The existing 8-channel R-01 baseline must be
   revised to 25 channels minimum, with BW250 modem default and quality-
   aware skip-list.
4. **Schedule an antenna-and-EIRP audit** alongside option C — measure
   actual peak vs. average field strength, verify the BW250 20 dB BW
   really meets the ≥ 250 kHz threshold for the relaxed FHSS tier, and
   confirm out-of-band attenuation ≥ 20 dB / 100 kHz per §15.247(d).
5. **Treat the internal 40 % airtime budget in
   [sx1276_airtime.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c)
   as a fairness limiter only** — replace or augment with a 400 ms / 10 s
   per-channel dwell counter when option C lands. Do not advertise either
   as "regulatory compliance".

---

## 6. Open follow-ups

* [ ] Confirm SF7-12/BW250 20 dB occupied bandwidth meets ≥ 250 kHz on the
  actual SX1276 (vendor app note + spectrum-analyser bench check).
* [ ] Define the 25-channel hop plan: centre-frequency list, hop period,
  pseudo-random permutation seed mechanism, TX/RX sync protocol.
* [ ] Reconcile the bench evidence on file (currently all at one
  channel, BW125) by stamping each future evidence file with the
  modulation/channel-plan it ran under.
* [ ] Update
  [config.h](../DESIGN-CONTROLLER/firmware/murata_l072/config.h)
  comment block — "8-channel FHSS baseline" → "25-channel FHSS baseline
  at BW250".
* [ ] Confirm whether any LifeTrac controller LoRa traffic ever crosses
  channels other than 915.000 MHz today; if not, document that explicitly
  so the FHSS bring-up plan starts from a known fixed point.
* [ ] Update
  [2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md](2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md)
  to note that single-channel operation is bench-only and the production
  TX-power adapter has to live on top of a hop scheduler.

---

## 7. Source references

* 47 CFR §15.247 — Operation within the bands 902-928 MHz, 2400-2483.5 MHz,
  and 5725-5850 MHz. eCFR, fetched 2026-05-19.
* 47 CFR §15.249 — Operation within the bands 902-928 MHz, 2400-2483.5 MHz,
  5725-5875 MHz, and 24.0-24.25 GHz. eCFR, fetched 2026-05-19.
* Bench evidence:
  [walk_power_pilot_2026-05-19/README.md](../DESIGN-CONTROLLER/bench-evidence/walk_power_pilot_2026-05-19/README.md)
  (live RegFrf readback, single-channel paired sweep, airtime-gate
  falsification probe).
* Firmware sources:
  [radio/sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c),
  [radio/sx1276_airtime.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c),
  [include/host_cfg_keys.h](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_keys.h),
  [config.h](../DESIGN-CONTROLLER/firmware/murata_l072/config.h).
* Tooling:
  [regfrf_check.py](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/regfrf_check.py).

---

## 8. Outside-the-Box: Maximizing the 902-928 MHz Envelope

If the goal is to extract the absolute maximum performance, range, and reliability legally permitted, we can move beyond single-regime static configurations. 

### 8.1 Adaptive / Hybrid Regulatory Regimes
Instead of statically locking into Option A, B, or C, the active link could dynamically transition between FCC regimes based on real-time operational needs:
* **Extreme Range Mode (FHSS 50-Channel):** By utilizing 50 channels with BW125 (or BW250), we unlock the absolute maximum allowable conducted power of **+30 dBm (1 Watt)** while enjoying high link margin. This is ideal when the machine is far afield. The 50 channels easily fit within the 26 MHz band (50 x 250 kHz = 12.5 MHz).
* **High-Bandwidth Burst Mode (DTS):** When a large payload (e.g., telemetry logs or graphical data) needs to be offloaded, switch to **BW500 at +30 dBm**. In DTS mode, hopping is not required. You get maximum power, high data rates, and no hop-synchronization latency, provided you manage the PSD limits.
* **Proximity Control Mode (§15.249):** When the operator is close to the machine, drop power to ~+10 dBm. Operations can occur on a **single channel** with no FHSS overhead, eliminating any synchronization delays for critical immediate controls.

### 8.2 Intelligent Channel Blacklisting (Quality-Aware Hopping)
The rules prohibit "listen-before-talk" to avoid other users, but **quality-aware hopping** on a single system is explicitly permitted under §15.247(h).
* **The Strategy:** Define a hop table of **64 channels**. The X8/L072 link continuously monitors the noise floor/SNR on these channels. It dynamically blacklists up to 14 noisy channels (leaving the minimum 50 required for the +30 dBm FHSS tier). This guarantees we are always hopping over the cleanest possible spectrum while rigidly complying with the channel-count floor.

### 8.3 Asymmetric Link Configuration
The tractor and base station do not have to use symmetric transmit profiles:
* **Tractor to Base:** The tractor has massive battery reserves. It can transmit telemetry continuously at +30 dBm using 50-channel FHSS.
* **Base to Tractor:** The remote controller may be battery constrained. It could rely on a higher-gain directional antenna (trading off conducted power 1:1 for antenna gain >6 dBi, up to certain limits) or operate in a burstier DTS profile to save power while maintaining the link.

### 8.4 Integration into the LifeTrac Codebase
To deploy this level of sophistication in our firmware:
1. **Link Profile Sequences:** Move away from static `#define CFG_KEY_FHSS_ENABLE` flags. Introduce a dynamic `LinkProfile` struct. The X8 controller actively commands the L072 to swap profiles (e.g., `LINK_CMD_GOTO_EXTREME_RANGE`) based on RSSI thresholds.
2. **Channel-Aware Dwell Manager:** Upgrade `sx1276_airtime.c` from a simple 40% global duty cycle gate to a true **per-channel dwell accumulator**. It maps timestamps to frequencies, ensuring that no single channel exceeds 400 ms of active TX within the sliding 10s (or 20s) window.
3. **Robust Sync/Beaconing:** Implement a fallback sync protocol. If the tractor loses the remote's hopping sequence, it falls back to a known "rendezvous channel" in a wide-RX window, while the remote occasionally blasts a high-power DTS beacon to re-synchronize the hopping clocks.
4. **Antenna Gain Management:** If we adopt different antennas (e.g. 8 dBi on a base station mast), the X8 must automatically clamp the L072's `PA_BOOST` maximum to +28 dBm to comply with the 1-for-1 reduction rule, ensuring hardware-level compliance regardless of the attached antenna.

---

## 9. Deep-Dive Addendum (2026-05-19, post-v1.0 cross-check)

This section is an in-depth addendum after re-reading the live rule text and
walking the current firmware paths. It is intended to tighten legal precision,
separate theory from what the current code can actually do, and define a
maximum-legal path that is certifiable.

### 9.1 Rule clarifications that materially change design choices

1. **§15.249 table interpretation (important correction):**
  In §15.249(a), the two columns are **(a)** field strength of the
  fundamental (mV/m) and **(b)** field strength of harmonics (uV/m).
  For 902-928 MHz this is **50 mV/m fundamental, 500 uV/m harmonics**.
  It is **not** an average-vs-peak pair.

2. **Equivalent EIRP for §15.249 fundamental is very low:**
  Using isotropic conversion at 3 m,
  `P_eirp = (E * r)^2 / 30`,
  `E = 0.05 V/m`, `r = 3 m` gives `P ≈ 0.00075 W ≈ -1.25 dBm EIRP`.
  So §15.249 is a **very low-power envelope** compared with our current
  +14 to +17 dBm conducted practice.

3. **§15.249(e) peak-over-average note scope:**
  The "20 dB peak over average" statement in §15.249(e) is stated for
  frequencies **above 1000 MHz** (per §15.35(b)). For a 915 MHz
  fundamental, the primary limit remains the §15.249(a) table fundamental.

4. **§15.247 out-of-band attenuation nuance:**
  §15.247(d) is 20 dB below in-band in 100 kHz outside-band when using the
  peak conducted method; when compliance is shown using RMS averaging per
  §15.247(b)(3), the required attenuation becomes **30 dB**.

5. **Adjacent clauses that matter for productization (not just bench):**
  * §15.203 antenna requirement (no open-ended antenna substitution).
  * §15.204 amplifier/antenna modification constraints (system authorization
    implications if external PA is introduced).
  * §15.205 + §15.209 restricted-band and general spurious limits.
  * §15.247(i) RF exposure compliance statement requirements.

### 9.2 Maximum legal envelope vs. current hardware envelope

| Mode | Regulatory ceiling | Current LifeTrac L072 path | Practical meaning |
|---|---|---|---|
| FHSS, >=50 channels (§15.247(b)(2)) | up to +30 dBm conducted | **+17 dBm max in code clamp** | FCC is not the bottleneck today; hardware/firmware is |
| FHSS, 25-49 channels (§15.247(b)(2)) | +24 dBm conducted | +17 dBm max | same |
| DTS, 6 dB BW >= 500 kHz (§15.247(a)(2)/(b)(3)/(e)) | +30 dBm, PSD-limited | +17 dBm max | same |
| §15.249 | 50 mV/m @ 3 m (very low equivalent EIRP) | current +14/+17 dBm practice exceeds this by a wide margin | bench-only unless power is reduced dramatically |

**Key consequence:** Section 8's +30 dBm ideas are architecturally interesting
but **not reachable on the present Murata SX1276 path** without an external,
certified system-level PA architecture.

### 9.3 How this works in the codebase today (actual behavior, not intent)

1. **L072 radio defaults are fixed at init:**
  [sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c)
  sets frequency 915 MHz, SF7/BW250/CR4-5, and +14 dBm at init.

2. **TX power hard clamp is 2..17 dBm:**
  [sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c)
  in `sx1276_set_tx_power_dbm()` clips to 17 dBm max.

3. **L072 TX path is currently single-channel in runtime:**
  [sx1276_tx.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c)
  uses `s_channel_idx = 0` in `sx1276_tx_begin()`; no hop selection or
  per-packet retune logic is active.

4. **Current airtime gate is fairness/QoS, not FCC dwell compliance:**
  [sx1276_airtime.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c)
  uses a 1-second window and 400,000 us budget, effectively a 40% limiter.
  FCC FHSS dwell is a per-channel 400 ms constraint over 10 s or 20 s
  windows (depending on regime), which is a different invariant.

5. **FHSS cfg keys exist but are mostly scaffolding on L072 path:**
  [host_cfg_keys.h](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_keys.h)
  and [host_cfg.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c)
  define and validate `CFG_KEY_FHSS_ENABLE`, `CFG_KEY_FHSS_CHANNEL_MASK`,
  `CFG_KEY_FHSS_DWELL_MS`, `CFG_KEY_FHSS_QUALITY_AWARE`; however these keys
  are not yet wired into active channel-hop runtime logic in
  [sx1276_tx.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c).

6. **LBT is active in the L072 TX gate:**
  [sx1276_lbt.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_lbt.c)
  performs CAD/RSSI-based backoff before TX when enabled.

7. **Separate FHSS logic exists in shared/H7-handheld code, but 8-channel and compile-gated:**
  [lora_proto.c](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c)
  and [lora_proto.h](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.h)
  implement an 8-channel hop helper and CSMA skip wrapper.
  Call sites are under `#ifdef LIFETRAC_FHSS_ENABLED` in
  [tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)
  and [handheld_mkr.ino](../DESIGN-CONTROLLER/firmware/handheld_mkr/handheld_mkr.ino).

**Bottom line:** regulatory intent exists in comments/config keys, but the
shipping L072 C runtime is still effectively fixed-channel today.

### 9.4 Outside-the-box but certifiable architecture to maximize legal performance

1. **Add a first-class regulatory profile governor** on the L072 path.
  Proposed profiles:
  * `BENCH_ONLY_FIXED_915`
  * `FCC_15_247_FHSS_25CH_BW250`
  * `FCC_15_247_FHSS_50CH`
  * `FCC_15_247_DTS_BW500`

2. **Enforce legal combinations in firmware, not just docs.**
  Reject cfg sets that violate the profile (e.g., too few channels for chosen
  bandwidth/power tier, illegal dwell setting).

3. **Implement true hop scheduler + dwell accountant in L072 runtime.**
  * Add real channel selection before each TX.
  * Track per-channel occupancy in sliding 10 s / 20 s windows.
  * Enforce equal-use average and legal minimum active-channel count.

4. **Keep quality-aware blacklisting, but with a hard legal floor.**
  Channel avoidance is only allowed while preserving minimum active channels
  (25 or 50 depending on profile). If blacklist pressure drops below the
  legal floor, auto-degrade profile/power.

5. **Treat PA/antenna upgrades as authorization-bound system changes.**
  Any move toward +24/+30 dBm or higher-gain antenna deployments should be
  implemented as explicit SKU/certification profiles, not ad-hoc runtime
  tuning.

6. **Make compliance observable in bench artifacts.**
  Emit URC/stat fields for:
  * channel-use histogram,
  * max dwell per channel window,
  * active channel count over time,
  * profile transitions and cause.

### 9.5 High-confidence implementation order in this repo

1. Wire FHSS cfg keys from
  [host_cfg.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c)
  into active runtime behavior.
2. Introduce `sx1276_fhss` runtime module and call it from
  [sx1276_tx.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c).
3. Replace/augment [sx1276_airtime.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c)
  with legal dwell windows while retaining optional fairness limits.
4. Harmonize the legacy 8-channel helper in
  [lora_proto.c](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c)
  with the chosen production profile (or clearly mark it bench/legacy only).
5. Update [config.h](../DESIGN-CONTROLLER/firmware/murata_l072/config.h)
  comments and [TODO.md](../TODO.md) regulatory targets to remove stale
  assumptions and reflect the true §15.249 interpretation.

### 9.6 Errata marker for this document revision

To prevent future confusion, treat these as superseding clarifications for v1.0:
* §15.249 "50/500" is fundamental/harmonics, not average/peak.
* §15.249-equivalent power is much lower than +13 dBm EIRP shorthand suggested
  in earlier drafts.
* +30 dBm strategies are theoretical under §15.247 but not reachable on the
  current SX1276 + firmware clamp path without system-level RF architecture
  changes and authorization work.

---

## 10. Regulatory envelope audit and maximum-use plan (2026-05-19)

This section is the final addendum from the deeper FCC/codebase pass. It
keeps the audit trail above intact, but it should be treated as the most
current interpretation in this note. The major corrections are:

* §15.249 is much lower power than the early shorthand suggested.
* The live L072 firmware defaults are now SF7/BW250, not the older BW125
  bench helper path.
* 8-channel FHSS is not a certifiable US Part 15.247 plan for the present
  LoRa profiles.
* The strongest near-term strategy is probably **50-channel BW250 FHSS**,
  not the earlier 25-channel-only target.

### 10.1 The rule stack, precise enough for engineering decisions

Part 15 operation in 902-928 MHz is not a licensed allocation owned by us.
Under §15.5 the operator gets no protected right to any frequency, must accept
interference, and must stop transmitting if the device causes harmful
interference after FCC notice. That practical point matters for a tractor:
the control system has to be robust to interference even when the RF emissions
are perfectly legal.

For this project there are three plausible authorization envelopes:

| Envelope | What it really requires | What it buys | Trap to avoid |
|---|---|---|---|
| §15.247(a)(1) FHSS | Pseudo-random hop list, equal average use, synced receivers, channel spacing at least 25 kHz or the 20 dB bandwidth, whichever is greater. In 902-928 MHz: if 20 dB BW <250 kHz use >=50 channels and <=400 ms/channel/20 s; if 20 dB BW is 250-500 kHz use >=25 channels and <=400 ms/channel/10 s. | Up to +30 dBm conducted if >=50 channels; +24 dBm if 25-49 channels. No duty-cycle loss if the scheduler spreads traffic correctly. | A duty-cycle limiter on one channel is not FHSS. 8 channels is not enough. A hop table in shared code is not compliance unless the radio actually retunes before TX and RX follows it. |
| §15.247(a)(2) DTS | Digital modulation with **6 dB bandwidth >=500 kHz**, +30 dBm conducted max, and conducted PSD <=8 dBm/3 kHz. No hopping required. | The cleanest legal single-channel path, especially for burst/bulk data, if BW500 passes occupied-bandwidth and PSD testing. | LoRa configured as BW500 does not automatically prove measured 6 dB BW >=500 kHz on the actual module. Full +30 dBm at exactly 500 kHz is near the PSD ceiling, so lab margin matters. |
| §15.249 low-power | Field strength of the 902-928 MHz fundamental <=50 mV/m at 3 m; harmonics <=500 uV/m at 3 m; out-of-band non-harmonics attenuated 50 dB or to §15.209. | Single-channel, no hop scheduler, no DTS bandwidth test. | This is **not** a +13 dBm EIRP regime. 50 mV/m at 3 m is about -1.25 dBm EIRP. |

The surrounding clauses are not optional paperwork:

* §15.203 and §15.204 mean the antenna, antenna type, gain, and any external
  RF amplifier are part of the authorized system. A field-swappable random
  SMA antenna or ad-hoc booster is not a free variable in a marketed product.
* §15.205 and §15.209 still apply to restricted-band and general spurious
  emissions. Hopping or DTS status does not make harmonics disappear.
* §15.31 and §15.35 control how measurements are made. As of the current
  transition rule, intentional-radiator measurements are on ANSI C63.10-2020;
  below 1000 MHz limits are generally quasi-peak unless a specific rule says
  otherwise, while above 1000 MHz average/peak behavior matters for harmonics.
* §15.247(d) requires out-of-band attenuation at least 20 dB below in-band
  when demonstrating peak conducted compliance; if using RMS averaging under
  §15.247(b)(3), the attenuation requirement becomes 30 dB.
* §15.247(i), §1.1307, and §1.1310 bring in RF-exposure evaluation. At current
  +14 to +17 dBm this is unlikely to be the hardest problem, but any +24/+30
  dBm or high-gain mast SKU must explicitly account for it.

### 10.2 Corrected §15.249 math

The earlier +13 dBm EIRP shorthand is wrong for §15.249. The isotropic
far-field conversion is:

```text
P_eirp = (E * r)^2 / 30
E = 0.05 V/m
r = 3 m
P = 0.00075 W = 0.75 mW = -1.25 dBm EIRP
```

With a 2 dBi whip and negligible cable loss, the conducted-power target for a
§15.249 single-channel product is roughly **-3.25 dBm conducted**. The current
L072 driver writes PA_BOOST and clips requested power to **+2..+17 dBm** in
[sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c), so even
the current software minimum would be about +4 dBm EIRP into a 2 dBi antenna,
roughly 5 dB above the §15.249 fundamental limit.

Implication: §15.249 is not a useful production envelope for the present
PA_BOOST path unless we add calibrated attenuation, switch to a lower-power RF
path, use a much lower-gain/losier antenna system, or create a separate
low-power certified configuration. It is acceptable as a conceptual yard/bench
mode only after hardware-level proof that the radiated field strength is below
50 mV/m at 3 m.

### 10.3 What the current code actually does

The live L072 C path is closer to the legal BW250 plan than the older bench
helper, but it is still not FHSS-compliant yet.

| Code path | Current behavior | Compliance meaning |
|---|---|---|
| [sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) | `sx1276_init()` sets 915.000 MHz, SF7/BW250/CR4-5, and +14 dBm. `sx1276_set_tx_power_dbm()` clips to +17 dBm max and +2 dBm min. | Frequency and BW are plausible for a §15.247 FHSS profile, but power is only a setting; there is no regulatory profile guard. |
| [sx1276_tx.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c) | `sx1276_tx_begin()` hardcodes `s_channel_idx = 0`, runs LBT, reserves airtime on that channel, writes FIFO, and transmits. No hop selection or pre-TX retune occurs. | The active runtime remains fixed-channel, regardless of `fhss_enable` defaults. |
| [sx1276_airtime.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c) | Tracks 16 channel buckets with a 1-second window and 400,000 us budget. | This is a QoS/fairness gate. FCC dwell is 400 ms per channel over 10 s or 20 s, so this module is both the wrong window and the wrong invariant. |
| [host_cfg.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c) | Defines `CFG_KEY_FHSS_ENABLE`, `CFG_KEY_FHSS_CHANNEL_MASK`, and `CFG_KEY_FHSS_DWELL_MS`; default mask is `0xFF`; validation only requires the mask be nonzero. Quality-aware FHSS is marked unsupported. | The config surface exists, but it cannot prove legal channel count, legal dwell, legal power, or active hop execution. |
| [lora_proto.c](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c) and [lora_proto.py](../DESIGN-CONTROLLER/base_station/lora_proto.py) | Implement an 8-channel pseudo-random helper and CSMA skip-busy logic. | Useful scaffolding, but 8 channels is below the FCC floor and the helper is not wired into the L072 TX path. |
| [sx1276_lbt.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_lbt.c) | Performs CAD/RSSI listen-before-talk/backoff before TX when enabled. | Good reliability behavior, but it cannot replace minimum channel count, equal average use, or dwell accounting. Use it only inside a legal hop profile. |
| [lora_ping.c](../DESIGN-CONTROLLER/firmware/murata_l072/lora_ping.c) | Standalone bench helper writes BW125, +14 dBm, 915 MHz. | Historic bench evidence must be stamped with the actual running binary/config; this helper does not reflect the current `sx1276_init()` defaults. |

The important falsification is now complete: the presence of FHSS config keys
and an 8-channel helper does **not** mean LifeTrac is hopping today. The active
L072 TX path is fixed-channel until `sx1276_tx_begin()` is made to select,
retune, account, and report a real hop channel.

### 10.4 Maximum legal use: better target profiles

The best target depends on whether we are maximizing current hardware, future
PA headroom, lowest latency, or simplest certification. These are the profiles
worth carrying as first-class firmware states.

| Profile | Channel plan | Legal ceiling | Why use it | Main blocker |
|---|---|---|---|---|
| `FCC_15_247_FHSS_50CH_BW250` | 50 channels, BW250, <=500 kHz 20 dB BW, >=20 dB-BW spacing | +30 dBm conducted tier because the system employs >=50 channels; dwell window is still 400 ms/channel/10 s because BW >=250 kHz | **Best maximum-use target.** Current +17 dBm hardware has huge margin; future certified +24/+30 dBm hardware can reuse the same regulatory architecture; 50 channels give continuous-TX dwell margin instead of zero-margin 25-channel operation. | Must prove measured 20 dB BW is >=250 kHz and <=500 kHz; implement real 50-channel sync and dwell. |
| `FCC_15_247_FHSS_25CH_BW250` | 25 channels, BW250 | +24 dBm conducted; 400 ms/channel/10 s | Simpler than 50 channels and enough for current +17 dBm hardware. | At 100% continuous traffic it has essentially no aggregate dwell margin: 25 x 400 ms = 10 s per 10 s. |
| `FCC_15_247_FHSS_50CH_BW125` | 50 channels, BW125 | +30 dBm conducted; 400 ms/channel/20 s | Best narrowband/range-preserving FHSS tier if BW250 sensitivity loss hurts. | Longer airtime; individual packets must still stay under the 400 ms dwell cap unless the radio can hop mid-packet, which the SX1276 LoRa packet path does not. |
| `FCC_15_247_DTS_BW500` | Single channel or small channel set, BW500 | +30 dBm conducted, PSD <=8 dBm/3 kHz | Best no-hop mode for bulk bursts, debug transfer, or a simple one-channel fallback if occupied bandwidth passes. | Must measure 6 dB bandwidth >=500 kHz and PSD. At full 1 W/500 kHz the PSD math is close to the limit; current +17 dBm has much more PSD margin. |
| `BENCH_ONLY_FIXED_915` | Single channel, any narrow LoRa profile | Not a production authorization | Lab work, short controlled tests, bring-up, regression probes. | Must never be labeled field/commercial compliant. |

The new recommendation is therefore stronger than section 5 above:

1. For current hardware production planning, target **`FCC_15_247_FHSS_50CH_BW250`** first, not 8 channels and not merely 25 channels.
2. Keep `FCC_15_247_DTS_BW500` as a measured/certified secondary mode for
  bulk data or emergency resync where hop synchronization is the dominant
  failure risk.
3. Treat §15.249 as a separate low-power hardware profile, not a software-only
  fallback on the present PA_BOOST driver.

### 10.5 Why 50-channel BW250 is the outside-the-box sweet spot

The earlier natural move was: "BW250 qualifies for the relaxed 25-channel
FHSS rule, so use 25 channels." That is legal, but it leaves performance on
the table.

Under §15.247(b)(2), the +30 dBm conducted ceiling applies to 902-928 MHz
FHSS systems employing **at least 50 hopping channels**. The dwell rule for a
BW250 signal is still the BW>=250 branch: 400 ms/channel/10 s. That means a
50-channel BW250 system has twice the aggregate dwell capacity of a continuous
single transmitter:

```text
50 channels * 0.400 s/channel/10 s = 20 s permitted channel-occupancy per 10 s
```

The transmitter can only occupy one channel at a time, so continuous traffic
uses about half the available per-channel dwell budget if spread evenly. This
is much healthier than the 25-channel BW250 plan, where continuous traffic uses
the full legal dwell allowance with no scheduling margin.

A candidate 50-channel BW250 table could use 500 kHz center spacing across the
band, for example centers around 902.75 MHz through 927.25 MHz. That leaves
guard room at both band edges and makes channel separation comfortably larger
than a nominal BW250 LoRa signal. The exact center list must be finalized only
after measuring the real SX1276 20 dB bandwidth and edge emissions.

This profile also aligns with the present hardware ceiling. At +17 dBm
conducted, even a high-gain base antenna remains inside the §15.247 conducted
power reduction rule for many practical antenna gains:

```text
50-channel tier: Pconducted_max = 30 dBm - max(0, antenna_gain_dBi - 6)
Current L072 clamp: Pconducted <= 17 dBm
17 dBm remains compliant up to about 19 dBi antenna gain, before other
authorization, pattern, RF exposure, and installation constraints are considered.
```

For the 25-channel tier, the same math is tighter:

```text
25-49 channel tier: Pconducted_max = 24 dBm - max(0, antenna_gain_dBi - 6)
17 dBm remains compliant up to about 13 dBi antenna gain.
```

So 50-channel BW250 is not only a future-1W path. It also gives the current
Murata module more antenna/system-design headroom without changing the RF PA.

### 10.6 Design constraints for any maximum-use implementation

These constraints should become firmware invariants, not just documentation:

1. **No legal profile may be selected unless the channel mask bitcount meets
  the profile floor.** For `FHSS_50CH_*`, fewer than 50 active channels is a
  config error. For `FHSS_25CH_BW250`, fewer than 25 is a config error.
2. **No channel may exceed 400 ms occupancy in its regulatory window.** Use a
  sliding 10 s window for BW250/BW500 FHSS and a sliding 20 s window for
  BW125/narrow FHSS. Do not reuse the current 1 s fairness window as a legal
  dwell proof.
3. **Every individual LoRa TX burst must fit inside the 400 ms dwell cap.** If
  a high-SF or large-payload estimate exceeds 400 ms, fragment it, lower the
  payload, switch profile, or reject it.
4. **Equal average channel use must be observable.** A pseudo-random permutation
  over the active set is better than independent random selection because it
  bounds short-window imbalance.
5. **Quality-aware blacklisting must preserve the legal floor.** If noise or
  policy removes too many channels, the firmware must degrade profile, lower
  power, or stop TX rather than silently running a noncompliant hopset.
6. **LBT/CCA cannot be the compliance mechanism.** It may reduce collisions and
  feed quality metrics, but it cannot replace hop count, equal use, or dwell.
7. **Antenna gain must be part of the config envelope.** The firmware should
  know the certified antenna/SKU and clamp conducted power accordingly.
8. **Profile switching must be certification-aware.** A device may switch among
  modes only if every enabled mode was tested/authorized and the transition
  itself cannot produce illegal emissions or unsynced stuck transmit states.

### 10.7 Concrete codebase implementation plan

Recommended implementation order:

1. **Add a regulatory profile type** to the L072 host config layer:
  `BENCH_ONLY_FIXED_915`, `FCC_15_247_FHSS_50CH_BW250`,
  `FCC_15_247_FHSS_25CH_BW250`, `FCC_15_247_FHSS_50CH_BW125`, and
  `FCC_15_247_DTS_BW500`. Make the active profile visible in `VER_URC` or a
  stats/config URC.
2. **Replace the 8-channel assumptions** in [config.h](../DESIGN-CONTROLLER/firmware/murata_l072/config.h),
  [host_cfg.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c),
  [lora_proto.c](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c),
  and [lora_proto.py](../DESIGN-CONTROLLER/base_station/lora_proto.py) with a
  profile-sized channel table. A `u64` mask can support up to 64 candidate
  channels, which is enough for a 50-channel legal set plus blacklisting slack.
3. **Create the missing `sx1276_fhss` runtime module** promised by the design
  docs. It should own channel tables, pseudo-random permutations, active-mask
  validation, hop counters, and RX/TX synchronization metadata.
4. **Modify `sx1276_tx_begin()`** so it selects the next legal channel, retunes
  with `sx1276_set_frequency_hz()`, then runs LBT on that channel, then passes
  the real channel index into dwell accounting. The current hardcoded
  `s_channel_idx = 0` is the main compliance blocker.
5. **Replace or augment `sx1276_airtime.c` with a true dwell accountant.** Keep
  a separate fairness limiter if useful, but expose separate counters so a
  test report cannot confuse QoS airtime with FCC dwell.
6. **Add power/antenna validation.** A profile should compute `Pconducted_max`
  from regulatory tier and certified antenna gain, then clamp or reject
  `CFG_KEY_TX_POWER_DBM` before it reaches `sx1276_set_tx_power_dbm()`.
7. **Add compliance telemetry.** Emit: active profile, active channel count,
  current hop index/frequency, per-channel occupancy histogram, max dwell in
  window, number of blacklisted channels, LBT abort count, and any automatic
  profile/power clamps.
8. **Build a spectrum evidence script.** It should run a continuous-data worst
  case under the selected profile, capture hop histogram and dwell maxima,
  confirm low/mid/high channels, read back `RegFrf`, and save the exact modem
  config used for the run.

### 10.8 Practical recommendation

For the next engineering decision, use this hierarchy:

1. **Bench now:** continue using `BENCH_ONLY_FIXED_915` for short controlled
  bring-up, but stamp every evidence file with frequency, BW, SF, CR, power,
  and whether hopping was disabled.
2. **First field-compliant profile:** implement `FCC_15_247_FHSS_50CH_BW250`
  at the current +14/+17 dBm clamp. This is the best blend of legal headroom,
  latency, future PA compatibility, and dwell margin.
3. **Secondary profile:** investigate `FCC_15_247_DTS_BW500` with a spectrum
  analyzer. If the 6 dB bandwidth and PSD pass with margin, this becomes the
  simplest legal single-channel mode for bulk transfer or recovery.
4. **Do not spend production effort on §15.249** unless a separate low-power RF
  path is desired. The present PA_BOOST software path cannot get low enough
  for a 2 dBi antenna.
5. **For maximum legal range without a new PA, spend first on antenna system and
  geometry:** base-station height, low-loss feedline, authorized higher-gain
  antenna type, polarization discipline, and receive diversity. These usually
  buy more real field reliability per engineering hour than chasing +30 dBm
  before the hop scheduler is correct.

---

## 11. Second-pass deep review (2026-05-19, post-section-10 cross-check)

This section is a fresh independent review after sections 1-10 were already
written. It re-reads §15.247 line by line against what the SX1276 + L072
firmware actually does, isolates clauses that earlier sections under-treated,
and proposes outside-the-box uses that are still inside the rule envelope.
Where it conflicts with earlier sections, treat section 11 as the more
current interpretation.

### 11.1 Clauses earlier sections under-treated

1. **§15.247(a)(1)(i): the "20 dB bandwidth" test is on the *emission*, not
  the configured modem BW.** LoRa "BW250" is a chirp bandwidth setting; the
  20 dB occupied bandwidth of the actual RF emission depends on SF, shaping,
  and PA distortion. A 50-channel BW250 plan only qualifies for the
  >=250 kHz tier (relaxed 25-channel floor) if **measured** 20 dB BW is at
  or above 250 kHz on the production hardware. Do not assume; capture it on
  a spectrum analyzer for each SF that ships.

2. **§15.247(a)(1) "pseudo-random" and "equal average use" are both
  required, jointly.** A round-robin hop list satisfies "equal average use"
  trivially but fails "pseudo-random." A pure random-draw scheme satisfies
  pseudo-random but takes many windows to converge to equal average use.
  The compliant pattern is a **pseudo-random permutation of the channel set,
  re-permuted per super-frame**: equal use within each super-frame, pseudo-
  random sequence across super-frames. This is exactly what the 8-channel
  helper in
  [lora_proto.c](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c)
  is structured to do; the legal change is "permute 50 channels, not 8."

3. **§15.247(a)(1)(iii) intra-system control allowance.** A §15.247 FHSS
  system may include **fixed-frequency control / acknowledgement
  transmissions** as long as they are not the primary mode of operation
  and the total time spent on the fixed channel is limited. This is the
  legal hook for a **rendezvous beacon** or a **resync ping** on a known
  channel without breaking FHSS classification — which is exactly the
  failure mode this codebase repeatedly hits when a node loses hop sync.
  Earlier sections framed rendezvous as a software convenience; it is in
  fact rule-supported design space.

4. **§15.247(g) frequency stability over temperature and voltage.** The
  hop channels must not drift across other licensed bands or out of
  902-928 MHz. The SX1276 TCXO on the Murata CMWX1ZZABZ module is the
  asset that makes this trivially true; if any future SKU drops the TCXO
  for a crystal, the certified hop plan must be re-verified across
  temperature. Repo memory
  [lifetrac-murata-l072-build.md](../../memories/repo/lifetrac-murata-l072-build.md)
  is the relevant build artifact to keep aligned with this.

5. **§15.247(d) emission masks interact with TX-power adaptation.** The
  walk-power probe described in
  [2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md](2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md)
  is legal as long as **every** power step still meets the OOB mask. PA
  distortion is usually *worse* at lower power per dB of headroom, so the
  walk_power lowest steps need their own OOB sweep, not just the +17 dBm
  ceiling case.

6. **§15.247(b)(3) RMS averaging trap.** If a certification report uses
  RMS averaging to claim conducted power below +30 dBm, the required OOB
  attenuation jumps from 20 dB to 30 dB per §15.247(d). For a LoRa
  preamble that is essentially CW, the RMS-vs-peak gap is small and the
  trade rarely helps; pick the peak-power path during certification unless
  there is a measured reason to do otherwise.

7. **§15.247(h) ban on cross-system coordination, again.** The L072 LBT
  module
  [sx1276_lbt.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_lbt.c)
  is legal as **local quality avoidance**. It would *not* be legal to add a
  protocol where two LifeTrac base stations exchange "I'm on channel N
  right now, stay off" hints to avoid each other; that crosses into
  coordination. Multiple LifeTracs sharing one site must either be one
  authorized network (one hop sequence, multiple radios) or be
  uncoordinated and statistically share the band.

### 11.2 What the live firmware proves and disproves today

Cross-checked against the current source tree:

| Claim | Verdict | Evidence |
|---|---|---|
| "FHSS is on by default if `fhss_enable` is set" | **False at the radio layer.** | [sx1276_tx.c L64](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c#L64) hardcodes `s_channel_idx = 0U` inside `sx1276_tx_begin()`, with no read of any FHSS config or hop selector. The cfg key flips a flag that nothing in the TX path consumes. |
| "The airtime gate enforces FCC dwell" | **False.** | [sx1276_airtime.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c) is a fairness limiter over a 1 s window, not a 10 s or 20 s legal dwell window. |
| "LBT plus a hop table equals FHSS compliance" | **False.** | §15.247(h) explicitly disallows cross-system listen-coordination, and §15.247(a)(1) requires structural hopping. LBT is a reliability tool inside a profile, not the profile itself. |
| "The 8-channel helper is field-legal" | **False.** | Below both the 25-channel and 50-channel floors of §15.247(a)(1)(i) for 902-928 MHz. |
| "+17 dBm conducted is safe everywhere in 902-928 MHz" | **Conditionally true.** | Safe under §15.247 tiers (FHSS or DTS) once channel-count and dwell are met. Not safe under §15.249 with the current 2 dBi antenna — see section 10.2. |
| "The TX power adaptation work makes us more compliant" | **Orthogonal.** | Power adaptation reduces *interference* and exposure margin but does not affect channel-count or dwell obligations. It must be layered **inside** a legal profile, not used as a substitute for one. |

### 11.3 Outside-the-box uses that stay inside the rules

These ideas were not fully developed in sections 8-10:

1. **Single-system multi-radio at one site (legal +6 dB throughput, not power).**
  Two L072 radios per base station, sharing one §15.247(a)(1) hop sequence
  with a fixed phase offset (e.g. radio A on permutation index *k*, radio B
  on index *k + N/2*). Each radio honors its own dwell budget, both belong to
  the same authorized system, no cross-system coordination is implied. Net
  effect: doubled aggregate throughput and an effective receive-diversity gain
  on the tractor side, without changing power or the hop plan.

2. **Asymmetric profile per direction (already legal under one authorization).**
  Telemetry uplink (tractor → base) runs `FCC_15_247_FHSS_50CH_BW250` at
  +17 dBm for range. Command downlink (base → tractor) runs
  `FCC_15_247_DTS_BW500` at +20 dBm because the base station has mains power
  and the spectrum analyzer can certify the DTS bandwidth/PSD once. Each
  direction is independently compliant; the radios just need to swap profiles
  in lockstep when changing role.

3. **DTS "safety burst" overlay on top of FHSS telemetry.**
  The safety-burst design in
  [2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md](2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md)
  becomes much cleaner if the burst is sent on a separate DTS-class
  configuration rather than as a louder LoRa packet inside the FHSS plan.
  Both profiles can share the same module if the firmware is willing to
  reconfigure RegBw/RegFrf/RegPaConfig between profiles and re-arm RX in
  the alternate mode. Authorization-wise the device declares *both*
  §15.247(a)(1) FHSS and §15.247(a)(2) DTS modes of operation; this is a
  routine FCC filing pattern, not exotic.

4. **Rendezvous channel as a §15.247(a)(1)(iii) control channel.**
  Promote one channel (e.g. mid-band, away from the hop set's center mass)
  to a **fixed-frequency control beacon**: short, infrequent, used only for
  resync after extended hop loss. Counts under the intra-system control
  allowance and removes the worst tail-latency failure mode in the current
  link design without breaking FHSS classification.

5. **Per-channel adaptive PA back-off driven by the OOB mask, not range.**
  Some PA + matching combinations have worse OOB shoulder at the band edges
  (902-905 MHz and 925-928 MHz). A measured per-channel PA back-off table
  baked into the regulatory profile lets the radio run +17 dBm in the band
  middle and +14 dBm at the edges, maximizing *average* legal radiated
  power while staying inside the §15.247(d) mask everywhere. Today the
  driver applies one number for all channels; per-channel back-off is a
  small table inside `sx1276_set_tx_power_dbm()`.

6. **Use packet airtime budget to *force* sub-dwell packets.**
  Pick (SF, BW, payload) such that **max packet airtime <= 400 ms** with
  margin. SF12/BW125 LoRa packets can exceed 1 s, which is structurally
  impossible to make dwell-compliant in a packet-non-hop radio like the
  SX1276. A profile invariant `max_airtime_ms_per_packet < dwell_cap_ms`
  is a one-line guard that prevents an entire class of stuck-transmit
  compliance bugs. This belongs next to the LoRa modem-config setter in
  [sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c).

7. **Receive-side full-band coverage is free.**
  §15.247 dwell and channel-count requirements apply to *transmitters*. The
  base station receiver can scan, listen continuously, or use multiple RX
  chains on different channels with no Part 15 budget consumed. Outside-
  the-box use: park a wide-band SDR (e.g. an RTL-SDR or HackRF on the
  Portenta X8 USB host) as a passive monitor of the entire 902-928 MHz
  hop set, feed it back into the quality-aware blacklist *without* a TX
  cost. This turns the X8 host's USB bandwidth into compliance + reliability
  margin and is wholly legal.

8. **Network-wide hop plan derived from a shared seed.**
  All LifeTracs at one farm derive their pseudo-random hop permutation from
  `seed = H(farm_id || epoch_index)`, then offset by `node_id`. Each radio
  remains independently §15.247-compliant; the network gains predictable
  inter-tractor frequency reuse without any "I'm on channel N" coordination
  packet (which would be illegal under §15.247(h)). The implementation
  surface is small: a deterministic permutation function in the new
  `sx1276_fhss` runtime module plus a `farm_id`/`node_id` config key.

### 11.4 Codebase mapping for the section 11 ideas

| Idea | Files to touch | Nature of change |
|---|---|---|
| 50-channel pseudo-random permutation per super-frame | new `sx1276_fhss` module, [sx1276_tx.c L64](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c#L64), [host_cfg.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c) | replace `s_channel_idx = 0U` with hop selector; widen channel-mask key to 64 bits |
| Per-profile dwell accounting in 10 s / 20 s windows | [sx1276_airtime.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c) | second window class, expose both counters in URC |
| DTS profile + safety-burst overlay | [sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c), [sx1276_tx.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c), [config.h](../DESIGN-CONTROLLER/firmware/murata_l072/config.h) | add BW500 modem-config helper, RegBw retune path, profile switch hook |
| Rendezvous control-channel allowance | new `sx1276_fhss` module, [lora_proto.c](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c) | fixed-frequency beacon path with airtime cap |
| Per-channel OOB back-off table | [sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) `sx1276_set_tx_power_dbm()` | small lookup keyed by channel index |
| Packet-airtime <= dwell invariant | [sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) modem-config setter | reject illegal (SF, BW, payload) tuples |
| SDR-based passive blacklist feeder | Portenta X8 base-station Python, current `base_station/` tree | new host-side process; no firmware change needed |
| Deterministic per-farm hop seed | new `sx1276_fhss` module, [host_cfg.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c) | two new cfg keys `farm_id`, `node_id`; one hash function |

### 11.5 Falsifiable predictions to record before the next bench run

To honor the methodology rule that hypotheses get falsification tests, the
following are concrete predictions the next regulatory bench session should
either confirm or refute:

1. Measured 20 dB occupied bandwidth at SF7/BW250 on the L072 module is
  **>=250 kHz** at +17 dBm into a 50-ohm load. If <250 kHz, the 50-channel
  BW250 plan still works but at the *narrowband* dwell rule (400 ms / 20 s)
  and the production profile name must reflect that.
2. With `s_channel_idx = 0U` unchanged, a 60-second continuous-TX run shows
  **all** energy on 915.000 MHz +/- 250 kHz on the spectrum analyzer.
  This is the falsification of any claim that the present firmware "hops."
3. SF12/BW125 at maximum LoRa payload exceeds **400 ms** packet airtime.
  If true, ban that combination from any FHSS profile in firmware, not just
  in docs.
4. The §15.249 fundamental at +2 dBm conducted (driver minimum) into a
  2 dBi antenna at 3 m is **>50 mV/m**. If true, §15.249 is unreachable via
  software-only attenuation on the current PA path and the doc's
  "low-power profile" idea must be marked hardware-bound.

### 11.6 Errata for sections 5 and 8 in light of section 11

* Section 5 step 1 ("keep going on a single 915.000 MHz channel at +14 dBm")
  is fine as bench/§15.5 work, but should be explicitly stamped
  `BENCH_ONLY_FIXED_915` in every evidence artifact so it is not
  accidentally interpreted as §15.247 or §15.249 evidence.
* Section 8.3's "asymmetric link" idea is fine; the §15.247(b)(4)
  high-gain-antenna trade is **conducted-power 1 dB per 1 dB excess gain**
  in 902-928 MHz, with no further bonus, so an 8 dBi base antenna forces
  a -2 dB conducted clamp from the relevant per-tier ceiling.
* Section 8.4 step 4's "automatic clamp to +28 dBm with 8 dBi antenna" is
  imprecise: the clamp must be relative to the **per-tier ceiling**
  (+24 dBm for 25-channel FHSS, +30 dBm for 50-channel FHSS or DTS), so
  the actual clamp depends on which profile is active, not a fixed +28 dBm.
* Section 9.2's "+30 dBm is not reachable on present hardware" remains
  correct; section 11 reaffirms that the present production target should
  be **maximize legal *plan* under the existing +17 dBm clamp** rather than
  chase the +30 dBm ceiling.

### 11.7 One-page operating recommendation

If only one paragraph of this addendum survives into the project plan,
keep this:

> The L072 firmware should adopt `FCC_15_247_FHSS_50CH_BW250` as its first
> certifiable production profile, leaving the current +17 dBm clamp in place
> for headroom. The implementation must replace `s_channel_idx = 0U` in
> [sx1276_tx.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c#L64)
> with a real pseudo-random hop selector over a 50-channel table, retune
> via `sx1276_set_frequency_hz()` before each TX, and account dwell in a
> true 10-second window. A §15.247(a)(1)(iii) fixed-frequency rendezvous
> beacon, a §15.247(a)(2) DTS profile for bulk/safety bursts, and
> receive-only wideband monitoring on the Portenta X8 host are the
> highest-value outside-the-box additions, all of which stay inside the
> rule envelope without requiring a new PA, a new antenna SKU, or any
> cross-system coordination prohibited by §15.247(h).

---

## 12. Critique and Refinement of the "Outside-the-Box" Concepts

The prior "Outside-the-Box" (Section 8) concepts proposed dynamic adaptation and asymmetrical link geometries. While theoretically aiming for maximum performance, these ideas encounter strict practical and regulatory limits when integrated into the SX1276 chipset and Part 15 rules.

### 12.1 Critique of Hybrid/Adaptive Regulatory Regimes
* **The Problem:** Dynamically swapping between FHSS (+30 dBm), DTS (+30 dBm, single channel), and §15.249 (very low power) is a certification nightmare. The FCC tests a system under a specific operational description. A system that dynamically changes its fundamental nature (hopping vs. non-hopping) mid-flight behaves like a Software Defined Radio (SDR) under Part 2 Subpart J, triggering far more expensive and stringent authorization requirements around firmware tampering and SDR security. Furthermore, coordinating these mid-air switches across a link using a half-duplex SX1276 risks permanent loss of synchronization if a single mode-switch packet is missed.
* **The Refinement:** Do not mix regulatory regimes in production. Commit strictly to **FHSS 50-Channel (BW250)** as the lone regulatory profile up to the hardware clamp (+17 dBm). If power gating is needed for proximity or battery saving, the system can dynamically adjust output power (+2 dBm to +17 dBm) *while maintaining the 50-channel hop sequence*. This complies easily within a single FCC profile without sacrificing synchronization or incurring SDR scrutiny.

### 12.2 Critique of Intelligent Channel Blacklisting
* **The Problem:** Blacklisting the "noisy 14" channels out of a 64-channel pool sounds like a robust way to bypass local interference without violating the 50-channel minimum. However, having both sides independently measure CAD/RSSI and construct separate blacklists leads to the "split-brain" symptom. A tractor may see Channel 42 as noisy and blacklist it, while the remote controller sees Channel 42 as clean and transmits on it. The receiver misses the packet.
* **The Refinement:** Only one device (the central node/tractor) should be designated the master of the hop list. The tractor evaluates the noise floor across the band and computes a single, system-wide blacklist. It then broadcasts the active 50-channel index list within periodic telemetry beacons or MAC payloads. The remote simply mirrors the master's hop scheme.

### 12.3 Critique of Asymmetric Link Operation
* **The Problem:** Proposing that the tractor blasts at +30 dBm using FHSS while the remote controller transmits bursts in DTS or §15.249 low-power mode is incompatible with hardware constraints. The SX1276 cannot actively listen to FHSS (e.g., BW125) and DTS (e.g., BW500) simultaneously without constantly retuning and swapping modulation parameters, creating huge deafness windows. Furthermore, RF links are bottlenecked by the weakest transmitter; a +30 dBm tractor talking to a +10 dBm remote still suffers link failure when the remote's weak commands can no longer reach the tractor.
* **The Refinement:** Maintain strict symmetry in modulation (BW250, SF7 or SF8). To achieve asymmetry legally (if required for battery savings on the remote), install a **high-gain directional antenna** on the base station/tractor mast. Since antenna gain acts as a passive multiplier for *both* transmit EIRP and receive sensitivity, the remote can run at a much lower conducted power without starving the link budget.

### 12.4 Critique of Sync Beaconing
* **The Problem:** Emitting a "high-power DTS beacon" to resynchronize lost nodes violates the rule that DTS and FHSS cannot be whimsically interleaved without risking SDR territory, and risks saturating the PSD limits if the "beacon" dwells too long on a fixed channel.
* **The Refinement:** Implement a **dedicated Rendezvous Hop Index**. If synchronization is lost for >2 seconds, both devices revert to a globally pre-shared hopping seed that utilizes all 50 channels but at a radically simplified, slow dwell rate. This ensures rendezvous is done strictly within the approved FHSS envelope.

---

## 13. Options for Unregistered / Uncertified Operation

If the intent is to avoid the time and expense of formal FCC lab certification and administrative registration altogether, it's crucial to understand the distinction between *operator licensing* and *equipment certification*. 

Part 15 is already an "unlicensed" band—meaning anyone can operate the equipment without an operator license or registration. The regulatory burden usually falls entirely on the manufacturer to get an "FCC ID" (equipment certification) before the hardware can be marketed or sold. 

If Open Source Ecology wishes to avoid paying for an FCC ID for the intentional radiator, here are the legal pathways:

### 13.1 Pre-Certified Modular Integration (The Commercial Path)
Most hardware startups completely avoid certifying intentional radiators by purchasing a radio module that already has a "Modular FCC ID" (e.g., specific Murata or XBee units).
* **How it works:** If you use a certified module and strictly follow its integration manual—specifically using only the exact type and maximum gain of antenna the module was originally certified with—your tractor *inherits* the module's registration. 

---

## 14. Fresh Independent Review of All Proposed Ideas (2026-05-19)

This is a clean second-pass critique over sections 1-13, with a practical
engineering lens: legal robustness, fit to current SX1276/L072 hardware,
certification survivability, and implementation risk.

### 14.1 Verdict matrix (keep / revise / drop)

| Idea | Verdict | Why | Improvement |
|---|---|---|---|
| Option A: §15.249 single-channel low power | **Revise heavily** | On current PA path this is not a realistic production envelope; earlier shorthand underestimated how low the §15.249 limit is. | Keep only as a bench/yard concept unless hardware path supports truly low EIRP operation. |
| Option B: §15.247 DTS BW500 | **Keep (conditional)** | Legally clean single-channel path if measured 6 dB BW and PSD pass. | Treat as secondary profile behind measured lab gates, not default. |
| Option C: §15.247 FHSS BW250 x25 | **Keep, but upgrade** | Sound architecture, but 25 channels leaves little/no dwell margin at sustained duty. | Move target to **50-channel BW250 FHSS** as primary production profile. |
| Hybrid regime switching (FHSS<->DTS<->15.249 at runtime) | **Drop for first release** | Certification, synchronization, and state-management complexity are very high. | Ship one primary profile first; add additional profiles only after a certification plan exists. |
| Quality-aware blacklisting | **Keep with constraints** | Good interference resilience, but split-brain risk if peers blacklist independently. | Make one node authoritative for active hopset; enforce legal min active channel floor in firmware. |
| Asymmetric PHY per direction | **Revise** | SX1276 retune/deafness penalties and weakest-link uplink bottleneck reduce net benefit. | Keep modulation symmetric; apply asymmetry through antenna geometry and duty policy instead. |
| Rendezvous/resync concept | **Keep** | High value operationally and can stay inside FHSS envelope. | Implement as deterministic in-profile rendezvous behavior, not profile/regime switching. |
| Per-channel PA shaping | **Keep (conditional)** | Useful only if bench data proves edge-channel OOB risk differs materially. | Gate behind measured spectrum evidence and profile-aware clamp logic. |
| SDR passive spectrum monitor on X8 host | **Keep (optional)** | Strong outside-the-box reliability aid with no transmitter rule burden. | Make it advisory input to hopset quality, never a cross-device coordination channel. |
| "Unregistered/uncertified" operation pathways | **Revise for clarity** | Easy to misread as deployment guidance; large legal/commercial risk if marketed. | Explicitly mark: lab/internal experimentation only; production requires compliant authorization path. |

### 14.2 Core critiques by section cluster

1. **Sections 1-3 (regime framing and options):**
  Strong overall structure, but the biggest decision-quality issue is
  underweighting the practical advantage of 50-channel BW250 versus
  25-channel BW250 for continuous/near-continuous use.

2. **Sections 4-5 (falsifications and recommendations):**
  Excellent falsification style. Improvement needed: explicitly separate
  "bench-legal under §15.5" from "field-deployable authorization posture"
  in each recommendation bullet so readers cannot collapse the two.

3. **Sections 8-12 (outside-the-box concepts):**
  Rich idea set, but several concepts are over-coupled. The practical
  improvement is to decouple into three staged tracks:
  * release track (single certifiable profile),
  * reliability track (blacklist, rendezvous, monitoring),
  * advanced/cert-expansion track (DTS profile, profile switching).

4. **Section 13 (uncertified paths):**
  Valuable topic, but wording needs stronger risk boundaries to avoid
  accidental interpretation as "ship without authorization."

### 14.3 Codebase-fit critique

Current document ideas are directionally correct, but the immediate mismatch is
still this: policy assumes FHSS while runtime TX remains fixed-channel unless
retune/hop is implemented in the active L072 TX path.

**Highest-impact code deltas:**
1. Replace fixed channel selection in
  [sx1276_tx.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c)
  with profile-driven hop selection + pre-TX retune.
2. Convert
  [sx1276_airtime.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c)
  from fairness-only accounting to legal dwell accounting (10 s / 20 s).
3. Harden
  [host_cfg.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c)
  with profile-aware validation (channel count floors, dwell bounds,
  power/antenna envelope checks).
4. Align legacy/common FHSS helpers in
  [lora_proto.c](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c)
  to the production profile target (or mark bench-only explicitly).

### 14.4 Improvements to make the document safer and clearer

1. Add an explicit label prefix to every mode mention:
  `BENCH_ONLY`, `FIELD_CANDIDATE`, or `CERT_EXPANSION`.
2. Add a one-line "authorization status" cell to each comparison table.
3. Add a single "current code truth" box near the top that states:
  *active TX hop behavior, active BW default, active power clamp*.
4. Add a "non-goals for release" list to reduce scope creep
  (especially dynamic regime switching).

### 14.5 Practical improved release strategy

If the goal is maximum legal utility with minimum failure risk, this is the
most defensible sequence:

1. **Release profile target:** `FCC_15_247_FHSS_50CH_BW250` at current
  +17 dBm hardware clamp.
2. **Reliability add-ons inside same profile:** quality-aware blacklist,
  rendezvous recovery, optional passive SDR monitor.
3. **Evidence gate before field use:** measured occupied bandwidth,
  per-channel dwell logs, hop histogram/equal-use report, OOB mask scan,
  antenna/power configuration lock.
4. **Only then** evaluate DTS BW500 as a second profile with its own lab gate.

### 14.6 Fresh recommendation

The strongest improvement across all ideas is to stop treating this as a
single monolithic "smart radio" feature and instead ship a **profile-governed
radio stack**:

* one certifiable primary profile,
* one bounded reliability layer,
* one measured expansion path.

That preserves the strongest innovations from sections 8-12 while avoiding
the largest legal and implementation pitfalls.
* **The Catch:** You cannot modify the RF parameters beyond what the module's grant allows. You do not need to register with the FCC, but you still must ensure your digital logic board passes basic unintentional radiator rules (Part 15B), which often only requires a self-declared Supplier’s Declaration of Conformity (SDoC) rather than full FCC registration. 

### 13.2 The Home-Built Exemption (§15.23)
The FCC provides an explicit exemption for hobbyists and developers building their own equipment.
* **How it works:** Under §15.23(a), you are legally permitted to build and operate up to **five (5) intentional radiators for your own personal use** without needing any FCC certification or registration. 
* **The Catch:** The equipment must still adhere to the technical rules (e.g., it cannot exceed the +30dBm FHSS power limits, and cannot cause harmful interference to licensed users). Most importantly, **these units cannot be marketed, sold, or offered for sale**. This is perfect for the initial fleet of OSE prototype tractors.

### 13.3 Development and Prototyping (§15.5)
You do not need to certify devices that are actively under development.
* **How it works:** Prototypes can be constructed and tested on the bench and in the field (the farm) to evaluate their performance. As long as they are operated by the engineers/developers, cause no harmful interference, and are explicitly marked as evaluation prototypes, no registration or FCC ID is required.
* **The Catch:** Like the home-built exception, you cannot sell these. 

### 13.4 Amateur Radio Exemption (Part 97)
The 902-928 MHz band overlaps identically with the internationally recognized 33-centimeter Amateur Radio band.
* **How it works:** If the equipment is operated under Part 97 rules, device certification is entirely waived. You can assemble custom transmitters from the component level, modify PAs, attach massive high-gain antennas, and push up to **1500 Watts** of power (massively exceeding Part 15's 1 Watt limit). 
* **The Catch:** This violates the premise of "unregistered" from the *operator's* perspective. The individuals operating the equipment must hold a valid Amateur Radio Operator License from the FCC. Furthermore, Part 97 strictly prohibits using the frequencies for any commercial or business purpose (meaning it could be used for hobby farming, but absolutely not for a commercial agricultural operation). 

**Conclusion:** For true "unregistered" operations that bypass the FCC lab: 
1. Use the **§15.5 and §15.23 exemptions** for internal R&D and the first 5 prototypes on your own land.
2. For scalable OSE kits distributed to community builders, rely completely on **Pre-Certified Modular Integration**. Source an off-the-shelf LoRa module with a pre-existing FCC ID, pair it with the exact whip antenna specified in its grant, and skip the intentional radiator certification process entirely.

### 13.5 Application to the Murata LoRa Chip on the Max Carrier

Since you are planning to use the Portenta Max Carrier (which embeds a Murata CMWX1ZZABZ series LoRa module), you are perfectly positioned to leverage the **Pre-Certified Modular Integration** strategy. 

However, there is a critical legal caveat when putting custom firmware on pre-certified modules:
* **The Modular Grant Trap:** The Murata module received its FCC ID by being tested at an FCC lab under a specific set of operational profiles—specifically, the standard **LoRaWAN US915 profile** (which uses a predefined 64-channel 125kHz + 8-channel 500kHz FHSS architecture). 
* **The Custom Firmware Risk:** If OSE writes custom firmware that operates the Murata chip on a single static channel, or an untested 8-channel pattern, **the module's FCC ID is technically voided**. A modular grant legally only covers the modulation, hopping pattern, and maximum power emissions that were originally tested in the lab. Changing the MAC layer physics beyond those tested bounds requires a Class II Permissive Change (C2PC), effectively putting you back in the FCC test lab.
* **The Unregistered Solution:** To distribute the Max Carrier without voiding its pre-existing FCC ID, your custom firmware MUST emulate the physical-layer behavior of the original LoRaWAN grant as closely as possible. By implementing the **50-Channel FHSS** architecture recommended earlier in this document, and capping your output power to match the module's tested max, you ensure the hardware emits RF in a pattern structurally identical to the original filing. You must also strictly use an antenna that matches the type and maximum gain (usually a ~2 dBi dipole) specified in the Portenta Max Carrier user manual.

---

## 14. Proceeding Under the Home-Built & Prototyping Exemptions (The Chosen Path)

Given the team's decision to proceed with custom firmware (e.g., single-channel or bespoke 8-channel hopping) instead of strict LoRaWAN emulation, you will be operating entirely under the **§15.5 (Development/Prototyping)** and **§15.23 (Home-Built)** exemptions.

This is a perfectly valid and legal approach for Open Source Ecology's initial R&D and the first prototype fleet. However, because you are actively choosing to discard the safety net of the pre-certified modular grant to maximize software control, there are absolute "red lines" the project must not cross to avoid FCC enforcement action.

### 14.1 Absolute "Red Lines" to Avoid

1. **DO NOT tune outside the 902-928 MHz band:** The Murata module is physically capable of tuning to European frequencies (868 MHz) or other global ISM bands. In the US, tuning outside 902-928 MHz causes you to bleed into licensed cellular, aviation, or emergency service bands. This is the fastest way to attract severe FCC fines. Keep $RegFrf$ strictly between 902.3 MHz and 927.7 MHz to allow for signal roll-off at the band edges.
2. **DO NOT market or sell the assembled RF hardware:** You can completely open-source the code, PCB schemas, and tractor designs, but you **cannot sell pre-assembled tractors with the custom-flashed LoRa radios included**. The §15.23 exemption is strictly for equipment built by the end-user for personal use (limit 5). If OSE starts selling pre-flashed RF controllers, you legally become a "manufacturer" marketing uncertified equipment.
3. **DO NOT ignore interference complaints:** Part 15 operation is strictly secondary. If your tractor's transmission happens to blast a local utility company's wireless smart-meter system or a licensed radiolocation service, and they notify you, you **must immediately cease operation** until the interference is resolved. You have no legal right to the frequency.
4. **DO NOT attach overpowered external amplifiers:** Sticking to the +17 dBm hardware clamp built into the Murata chip is safe. Do not buy undocumented 5-Watt or 10-Watt "LoRa boosters" from overseas and attach them to the antenna port. That instantly moves the hardware from an "unlicensed prototype" into an illegal, high-power intentional radiator capable of disrupting regional communications.

### 14.2 The Plan Going Forward

* **Firmware:** Continue developing the custom firmware. You are free to leverage the current +17 dBm clamp and the single-channel/8-channel logic as needed to minimize latency for the tractor's control loop.
* **Documentation:** Internally label the RF controllers as "Engineering Prototypes - Not for Sale or Commercial Deployment."
* **Scaling Strategy:** When the time comes to scale beyond the first 5 prototypes or distribute turn-key tractors to the community, OSE will either need to pivot to a fully compliant 50-channel FHSS profile (to legally leverage the Murata module's FCC ID) *or* rely on end-users to flash the boards themselves under their own personal §15.23 home-built exemptions (i.e. distributing them purely as software/manifests, not as functional RF appliances).

---

## 15. Fresh critique of all proposals and improved path

This section is a third-pass critique of the complete document after sections
1-14. It is intentionally conservative: the goal is to preserve the best ideas
while separating engineering optimism from rule text, certifiability, and the
current LifeTrac codebase.

### 15.1 Highest-risk corrections

1. **The fixed-frequency rendezvous allowance in section 11 is not proven.**
  The current §15.247 text fetched during this review does not contain a
  902-928 MHz allowance matching section 11's claim that §15.247(a)(1)(iii)
  permits fixed-frequency control/ack transmissions. In the live rule,
  §15.247(a)(1)(iii) is the 2400-2483.5 MHz hopping-channel provision, not a
  902-928 MHz rendezvous carve-out. Treat fixed-channel rendezvous as
  **unverified** until a test house or FCC KDB citation confirms it.

  Improvement: implement resync inside the 50-channel FHSS sequence first:
  a known pseudo-random seed, slower hop cadence, repeated short sync packets,
  and strict dwell accounting. If a fixed channel is still desired, certify it
  explicitly as a DTS/hybrid/control mode instead of assuming it is covered by
  FHSS.

2. **§15.23 is narrower than section 14 implies.** §15.23 exempts equipment
  authorization only for devices that are not marketed, are not constructed
  from a kit, and are built in quantities of five or less for **personal use**.
  That is a poor fit for an organization fleet, a sold kit, or a community
  deployment assembled from OSE instructions. It also does not waive the
  technical standards or §15.5 interference obligations.

  Improvement: treat §15.23 as an individual-builder footnote, not an OSE
  scaling strategy. For OSE-run R&D, use short controlled tests, good
  engineering practice, logs, and rapid shutdown discipline; for deliberately
  noncompliant field experiments, investigate FCC experimental authorization
  rather than stretching §15.23.

3. **Pre-certified modular integration is not saved by "emulating" LoRaWAN.**
  Section 13.5 is directionally useful but too confident. A module grant covers
  the tested configurations and integration conditions. Custom L072 firmware,
  a new hopping schedule, different dwell behavior, different power adaptation,
  or a new antenna may be outside the grant even if the spectrum shape looks
  similar. Only the grantee, a compliance lab, or the FCC filing record can say
  whether a permissive change is sufficient.

  Improvement: split the product path into two SKUs: **Grant-preserving SKU**
  using the module exactly as authorized, and **Custom-RF SKU** requiring a
  C2PC/new authorization package. Do not present the 50-channel custom profile
  as automatically inheriting the Murata or carrier FCC ID.

4. **Part 97 is not a practical LifeTrac control-link escape hatch.** Amateur
  operation has a 1.5 kW PEP general ceiling, but it also requires minimum
  necessary power, station identification, no communications for hire or
  pecuniary interest, and no encoded messages intended to obscure meaning.
  LifeTrac control frames are encrypted/authenticated and operationally tied to
  machine work. Even noncommercial experiments would need licensed operators
  and careful station identification.

  Improvement: keep Part 97 out of the product architecture. It may be useful
  only for isolated radio experiments that do not carry LifeTrac encrypted
  control traffic and are run by licensed operators for amateur purposes.

5. **Hybrid mode is not automatically an SDR violation, but it is still a
  certification expansion.** Section 12 overstates the SDR point. A device can
  have multiple certified modes without necessarily being classified as an SDR,
  but every enabled mode and transition has to be represented in the filing and
  protected against unauthorized parameter changes.

  Improvement: define profiles as certified operating states with locked ranges.
  The firmware may switch only among authorized profiles, and the host must not
  be able to set arbitrary frequency, bandwidth, PA, or channel masks in a
  production build.

### 15.2 Critique of the main technical ideas

| Idea | Keep? | Critique | Better version |
|---|---|---|---|
| 50-channel BW250 FHSS | **Yes, primary path.** | It is still the strongest plan, but only if measured 20 dB bandwidth is >=250 kHz and <=500 kHz. If measured BW lands below 250 kHz, it is still legal with 50 channels but falls into the 20-second dwell window. | Name the profile from measured behavior, not configured BW: `FCC_15_247_FHSS_50CH_MEASURED_WIDE` or `..._NARROW`. Keep edge channels far enough from 902/928 MHz after occupied-bandwidth testing. |
| 25-channel BW250 FHSS | **Keep as fallback only.** | Legal but low-margin: at continuous traffic, 25 x 400 ms consumes the whole 10-second window. It also gives only the +24 dBm tier. | Use only if 50-channel sync proves too complex; otherwise 50 channels gives better dwell margin and future power headroom. |
| 50-channel BW125 FHSS | **Maybe.** | Best range-preserving LoRa profile, but long SF/payload combinations can exceed 400 ms per packet, which the SX1276 cannot make compliant by hopping mid-packet. | Add a hard `packet_toa_ms <= 350` or `<= 400` profile invariant before any TX. Fragment rather than allowing large high-SF packets. |
| DTS BW500 | **Useful secondary mode.** | A clean single-channel mode only if measured 6 dB bandwidth and PSD pass. It should not become the emergency-control dependency before certification. | Certify as a separate profile for bulk transfer or recovery. For first field-safe work, send safety bursts as repeated high-priority FHSS packets. |
| Dynamic regime switching | **Limit hard.** | Operationally risky on a half-duplex SX1276: one lost profile-switch packet can strand nodes in different modes. It also broadens the authorization test matrix. | Switch only at scheduled profile epochs, announced in authenticated frames, with a dual-profile grace window and automatic rollback. |
| Quality-aware blacklisting | **Yes, with guardrails.** | Master-broadcast blacklist avoids split-brain but creates a new failure mode if the blacklist update is missed. It also must not become cross-system coordination under §15.247(h). | Use epoch-numbered hopset updates signed/authenticated by the link key, apply at a future epoch, retain the previous hopset for a grace interval, and reject any hopset with fewer than the legal channel floor. |
| Multi-radio base station | **Promising but not free.** | Simultaneous transmitters can create aggregate emissions, intermodulation, RF exposure, antenna isolation, and receiver desense problems. It may require testing the combined system, not just each radio alone. | Start receive-only: one TX/RX radio plus one passive SDR/receiver for band monitoring. Add a second transmitter only after the single-radio FHSS path is compliant. |
| High-gain antenna asymmetry | **Useful, bounded.** | Gain helps both transmit and receive, but for 902-928 MHz §15.247 requires conducted-power reduction above 6 dBi. On a moving tractor, antenna pattern and nulls matter more than peak gain. | Put gain at the fixed or semi-fixed base where geometry is stable; clamp conducted power by certified antenna gain; favor height, low-loss feedline, and diversity over a narrow high-gain mobile antenna. |
| Per-channel PA backoff | **Good improvement.** | Needs measured OOB shoulder data; otherwise it can become a false precision table. | During spectrum pre-scan, store max legal PA per channel group and make `sx1276_set_tx_power_dbm()` clamp by active channel and certified profile. |
| Home-built/prototype path | **Only for internal learning.** | Not a distribution strategy, and not a permission slip for noncompliant high-power field operation. | Use it to keep bench work moving, but define the field/commercial gate as Part 15.247 profile evidence plus compliance review. |

### 15.3 Recommended architecture after critique

The clean architecture is a **single certifiable production RF profile first**,
with other ideas staged behind it:

1. `BENCH_ONLY_FIXED_915`: default for early lab scripts only. Emits a warning
  in `VER_URC` or stats and writes `BENCH_ONLY` into evidence artifacts.
2. `FCC_15_247_FHSS_50CH`: first production candidate. Uses a measured channel
  table, pseudo-random permutations, equal use by construction, true dwell
  accounting, packet-airtime cap, and antenna-aware power clamp.
3. `FCC_15_247_DTS_BW500`: later certified secondary profile for bulk transfer
  and recovery, not required for first compliant field control.
4. `FCC_15_249_LOW_POWER`: remove from the current PA_BOOST software roadmap
  unless hardware attenuation or a lower-power RF path is added.

Firmware invariants that should be enforced in code:

* No production build may expose arbitrary register writes, arbitrary `RegFrf`,
  arbitrary bandwidth, or arbitrary PA settings over the host protocol.
* `CFG_KEY_FHSS_CHANNEL_MASK` must be validated by bitcount and by certified
  channel table membership, not merely nonzero.
* `sx1276_tx_begin()` must obtain its channel from the hop scheduler; the
  current `s_channel_idx = 0U` path should be compiled only for bench profile.
* Dwell accounting must use 10-second or 20-second regulatory windows, separate
  from any QoS/fairness airtime limiter.
* Every TX result should emit enough metadata to reconstruct the compliance
  state: profile, channel index, frequency, power, packet airtime estimate,
  dwell-window max, and hopset epoch.

### 15.4 Specific document cleanups recommended later

The document now preserves a useful thinking trail, but it has too many
superseded claims for a future reader to skim safely. After this note is used
for decision-making, create a v1.1 or v2.0 cleanup that:

1. Moves the corrected §15.249 math into section 1 and removes the +13 dBm
  shorthand from the early tables.
2. Replaces section 5's "25-channel BW250 recommended" with the current
  50-channel-first recommendation.
3. Marks section 8 as brainstorming, not guidance.
4. Removes or quarantines the unverified fixed-frequency rendezvous-control
  claim from section 11.
5. Rewrites sections 13-14 so §15.23 is not framed as an OSE fleet or kit
  strategy.
6. Adds a one-page "Current Decision" block at the top, because this document
  is now long enough that precedence-by-last-section is easy to miss.

### 15.5 Next falsification tests

Before writing more regulatory architecture, run these tests:

1. **Fixed-channel proof:** Run the current firmware for 60 seconds and confirm
  all TX energy stays on the initialized frequency while `s_channel_idx = 0U`.
2. **Occupied-bandwidth proof:** Measure 20 dB occupied bandwidth for SF7/SF8
  at BW250 and BW125, at +14 and +17 dBm, into a 50-ohm load.
3. **DTS proof:** Measure BW500 6 dB bandwidth and PSD at the highest intended
  conducted power.
4. **Packet dwell proof:** Generate the maximum packet airtime table for every
  allowed SF/BW/payload tuple; ban any tuple above the dwell cap.
5. **Minimum-power §15.249 proof:** Measure field strength at the current +2 dBm
  software minimum into the intended antenna. This should falsify the idea that
  §15.249 is software-reachable on the present PA path.
6. **Grant-scope proof:** Pull the exact FCC ID / grant notes for the Murata or
  Portenta Max Carrier LoRa configuration and identify which modulation,
  antenna, and firmware assumptions are actually covered.

### 15.6 Final refined recommendation

After reviewing sections 1-15 together, the best implementation path is:

1. Ship one production radio profile first: 47 CFR 15.247 FHSS with 50 active
  channels, using the current +17 dBm firmware clamp.
2. Treat every other profile as optional expansion until the FHSS profile is
  measured, stable, and compliance-observable.
3. Keep single-channel operation as bench-only work under controlled conditions,
  never as a field/commercial compliance posture.

### 15.7 Final thoughts on the 50-channel FHSS plan

The 50-channel FHSS plan is the right primary direction for this codebase.

Why it is strong:
1. It is the most robust legal structure for 902-928 MHz Part 15.247 operation
  because it directly satisfies channel-count architecture instead of trying to
  approximate compliance with airtime heuristics.
2. It gives better real-world interference resilience than 25-channel plans.
3. It keeps future hardware headroom open while still being compatible with the
  current +17 dBm limit.

What must be true before it is called compliant:
1. Active TX path must actually hop (no fixed-channel fallback in production).
2. Receiver must stay synchronized with the transmitter hop sequence.
3. Per-channel dwell accounting must use regulatory windows (10 s or 20 s,
  depending on measured channel bandwidth classification).
4. Occupied-bandwidth, OOB, and power evidence must be captured under
  continuous-data worst-case operation.
5. Production configuration must be profile-locked to prevent arbitrary runtime
  RF parameter changes.

### 15.8 Final implementation recommendation

Implement in this order:

1. Compliance core:
  add profile type, hop scheduler, pre-TX retune, RX sync, regulatory dwell
  counters, and profile-aware power/antenna clamp validation.
2. Compliance evidence:
  add telemetry outputs for profile, hop channel, dwell max, packet airtime,
  and power clamp reason; run analyzer-backed spectrum and occupancy tests.
3. Reliability enhancements inside the same profile:
  quality-aware blacklist with legal channel-floor guardrails and epoch-based
  update and rollback behavior.
4. Secondary profile only after gates pass:
  evaluate DTS BW500 as a certified expansion path, not as a prerequisite.

### 15.9 Final decision statement

Final recommendation: proceed with a profile-governed LoRa stack whose first
field target is 50-channel FHSS under 47 CFR 15.247, implemented and verified
as the single release-critical path.

---

## 16. Final end-of-document review and recommendation (2026-05-19)

This is the final recommendation block and supersedes earlier brainstorming
content where conflicts exist.

1. Adopt 50-channel FHSS as the primary production plan. This is the strongest
  legal and technical fit for the current LifeTrac architecture.
2. Keep current hardware power limits (+17 dBm firmware clamp) for the first
  compliant rollout; do not design around +30 dBm assumptions yet.
3. Make compliance explicit in runtime behavior:
  active hop selection in TX path, synchronized RX hop following,
  regulatory-window dwell accounting, and profile-locked RF parameters.
4. Treat DTS BW500 as a secondary certified expansion, not a dependency for
  first compliant field deployment.
5. Keep single-channel operation as bench-only and clearly label all such
  evidence artifacts accordingly.

### Final thoughts on 50-channel FHSS

The 50-channel FHSS plan is the right choice. It gives the best balance of
compliance robustness, interference tolerance, and future scalability while
remaining compatible with the present hardware clamp. The key is execution:
it must be implemented as actual runtime hopping and dwell enforcement, not
just config keys or documentation intent.

Signed: Copilot final review v3.1 (2026-05-19)

Signed: Copilot final review v3.1 (2026-05-19)

  ### Final thoughts on 50-channel FHSS

  The 50-channel FHSS plan is the right choice. It gives the best balance of
  compliance robustness, interference tolerance, and future scalability while
  remaining compatible with the present hardware clamp. The key is execution:
  it must be implemented as actual runtime hopping and dwell enforcement, not
  just config keys or documentation intent.

  Signed: Copilot final review v3.1 (2026-05-19)
  update/rollback behavior.
4. Secondary profile only after gates pass:
  evaluate DTS BW500 as a certified expansion path, not as a prerequisite.

### 15.9 Final decision statement

Final recommendation: proceed with a profile-governed LoRa stack whose first
field target is 50-channel FHSS under 47 CFR 15.247, implemented and verified
as the single release-critical path.

Signed: Copilot final review v3.0 (2026-05-19)

The best path is not "use every legal trick." The best path is to make one
legal trick boring and observable: **50-channel §15.247 FHSS on the existing
+17 dBm-clamped hardware**. Do that first, with measured occupied bandwidth,
real hop retuning, real dwell counters, and antenna-aware power clamps. Then add
DTS, multi-radio receive monitoring, blacklisting, and profile switching only
after they can be expressed as certified, testable profiles rather than clever
runtime improvisations.

---

## 16. Fresh Final Critique and Improvement Pass (2026-05-19)

This is a final independent review focused on "what to do next" after all
ideas in sections 1-15 are considered together.

### 16.1 Overall critique

1. The strongest ideas are technically good but currently over-bundled.
  Reliability features, certification strategy, and advanced RF optimization
  are mixed together, which makes execution risk high.
2. Several sections still mix *rule interpretation* and *architecture
  preference*. The document is most useful when these are separated cleanly.
3. The codebase gap remains the central blocker: policy assumes FHSS behavior,
  but the live L072 TX runtime still needs full hop integration and regulatory
  dwell accounting in the active path.

### 16.2 Critique of the major idea families

| Idea family | Critique | Improvement |
|---|---|---|
| Regulatory mode strategy | Too many possible profiles discussed as if equally ready. | Freeze one production target profile for release; keep others as explicit post-release expansion profiles. |
| FHSS scheduler concepts | Good theory, but legal proofs are not yet tied to runtime telemetry evidence. | Make compliance observable in every TX result (profile, channel, dwell window usage, power clamp reason). |
| Quality-aware adaptation | Strong concept, but synchronization failure modes are under-specified. | Use epoch-based hopset updates with grace windows and rollback behavior. |
| Asymmetric link ideas | Often physically true but can hide weakest-uplink bottlenecks. | Keep PHY symmetric first; use base-side antenna geometry and receive architecture for asymmetry gains. |
| "No certification" pathways | Useful for internal experimentation but dangerous if read as deployment guidance. | Add explicit labels: INTERNAL-LAB ONLY vs FIELD-CANDIDATE vs MARKETABLE PATH. |

### 16.3 High-value improvements to implement first

1. Add a one-page **decision header** at the top of this document with:
  selected primary profile, excluded ideas for this release, and required
  evidence gates.
2. Add a **hard separation of tracks**:
  * Track A: release-compliance path,
  * Track B: reliability enhancements inside Track A,
  * Track C: certification expansion concepts.
3. Replace prose-only compliance claims with a short **pass/fail checklist**:
  occupied bandwidth, hop histogram/equal-use, per-channel dwell maxima,
  OOB scan, antenna/power clamp traceability.
4. Normalize terminology throughout the note:
  * "bench-only" for §15.5 exploratory runs,
  * "field-candidate" for profiles with full evidence,
  * "market path" for certifiable/authorizable product operation.

### 16.4 Suggested release decision after this full review

1. Primary production target: **50-channel FHSS profile** on existing hardware
  clamp, with deterministic pseudo-random hopping and legal dwell windows.
2. Secondary profile candidate: DTS BW500 only after measured bandwidth/PSD
  evidence and profile-level certification planning.
3. Defer dynamic multi-regime switching until after one profile is fully
  implemented, measured, and operationally stable.

### 16.5 Document-quality improvement

Given the size and iterative history of this note, a follow-up v1.1 cleanup
should collapse superseded statements and keep one canonical recommendation
block near the top so future readers do not need to infer precedence from
section order.

---

## 17. Fresh end-to-end review (2026-05-19, post-section-16)

This section is a clean fourth-pass review of *all* ideas in sections 1-16,
written after re-reading the whole document in one pass. It deliberately does
not re-litigate items already settled by sections 11, 15, and 16; it focuses on
what the document as a whole still gets wrong, what is still missing, and what
the smallest set of high-value improvements would be.

### 17.1 Meta-critique: structural problems of the document itself

1. **The document contradicts itself across sections and resolves conflicts
   only by "last section wins."** Section 3 recommends 25-channel BW250.
   Section 10 promotes 50-channel BW250. Section 14 endorses 50-channel.
   Section 15 cautions against assuming measured BW. A reader who skims will
   read whichever conclusion they encounter first. This is a documentation
   defect, not a regulatory one.
   *Improvement:* Add a "Current Decision" block at the very top (above
   section 0) that states (a) the single canonical primary profile,
   (b) the supersession order of sections, and (c) which earlier
   recommendations are now retired. Mark retired recommendations inline with
   a `> SUPERSEDED by §N` callout instead of leaving them as live prose.

2. **Two unrelated section "14"s exist.** The note has both `## 14. Fresh
   Independent Review ...` and `## 14. Proceeding Under the Home-Built &
   Prototyping Exemptions ...`. Cross-references to "section 14" are now
   ambiguous.
   *Improvement:* Renumber sections in a v1.1 pass so each heading number is
   unique, and update internal references accordingly.

3. **Compliance claims and brainstorming are interleaved without labeling.**
   Sections 8 and parts of 11/13 mix "this is legal" with "this would be
   cool." A field/legal reader cannot tell at a glance which is which.
   *Improvement:* Tag every concept with one of `[RULE]`, `[ARCHITECTURE]`,
   `[BRAINSTORM]`, or `[OPEN QUESTION]` and keep that vocabulary consistent.

4. **Citations are uneven.** Section 1's table cites specific subparagraphs;
   sections 8, 11, and 13 make rule-shaped claims with no cite (notably the
   "fixed-frequency rendezvous" claim that section 15 had to retract).
   *Improvement:* Require an inline `(§15.247(x)(y))` next to every
   rule-shaped sentence, or mark it `[UNVERIFIED]`.

5. **The note is too long to skim safely.** It now exceeds 1,300 lines and
   has six distinct critique passes. The signal-to-noise ratio is dropping
   even though each individual pass is high-quality.
   *Improvement:* In a v1.1 cleanup, collapse sections 8, 12, 14, 15, 16 into
   a single "Critique history" appendix and promote the surviving conclusions
   into a short "Current architecture" body. Keep the audit trail, but stop
   making readers read it linearly.

### 17.2 Idea-by-idea fresh verdict

The table below treats each *distinct* idea raised anywhere in the document
once, regardless of how many sections reintroduced it.

| # | Idea (originating sections) | Current status across the doc | Fresh verdict | Smallest useful next step |
|---|---|---|---|---|
| 1 | Single-channel §15.249 low-power production mode (§3, §8, §10) | Already retired by §10.2 and §15.1.2 (math shows it is unreachable on PA_BOOST). | **Retired.** Stop carrying it as an option. | Delete from any forward-looking comparison table; keep only as a "considered and ruled out" footnote. |
| 2 | DTS BW500 single-channel production mode (§3, §8, §10, §11, §15) | Status "secondary profile, post-evidence." | **Keep, demote priority.** It is a real path but it must not be the primary release target because BW + PSD need lab evidence first. | One bench measurement of 6 dB occupied BW and PSD at +17 dBm; gate any further architecture work on the result. |
| 3 | 25-channel BW250 FHSS as the production target (§3) | Superseded by §10/§11/§14/§15. | **Retired as primary;** retain as a *fallback* profile only if 50-channel sync proves intractable. | Mark §3.C explicitly `> SUPERSEDED by §10` in the next edit. |
| 4 | 50-channel BW250 FHSS as the production target (§10, §11, §14, §15, §16) | Repeatedly endorsed. | **Confirmed primary target.** Caveat: §15.2 is right that "50-channel BW250" should be named from *measured* occupied BW, not configured BW. | Name the firmware profile after measured behavior (e.g. `FCC_15_247_FHSS_50CH_BW250_MEASURED`), and gate it on a bench occupied-bandwidth report. |
| 5 | 50-channel BW125 FHSS (§10, §15) | Listed as a range-preserving alternative. | **Keep on the shelf.** Useful only if BW250 sensitivity loss is shown to hurt link budget; otherwise it adds complexity (20 s dwell window, longer airtime). | Defer until link-budget evidence demands it. |
| 6 | Dynamic / hybrid regime switching at runtime (§8.1, §12.1, §15.1.5, §16) | Repeatedly cautioned against. | **Defer indefinitely.** Sound architecturally only after at least one profile is field-certified. | Remove from the active roadmap; keep in "future expansion" appendix. |
| 7 | Quality-aware blacklisting / 64-of-50 active set (§8.2, §10, §11, §15) | Endorsed with guardrails. | **Keep, with the split-brain fix from §12.2 (one authoritative node) and the epoch-grace fix from §15.2.** | Specify the authenticated hopset-update message format before any firmware work. |
| 8 | Asymmetric per-direction PHY (§8.3, §11.3, §12.3) | Section 12 partially walked it back. | **Walk back further.** SX1276 retune deafness + weakest-link uplink + symmetric LBT make asymmetric PHY costly. | Replace with antenna-geometry asymmetry (base-side gain, mast height, low-loss feedline). |
| 9 | High-gain base antenna with conducted-power clamp (§8.4, §10.5, §15) | Endorsed across passes. | **Keep.** This is the highest-leverage real-world range gain that does not need a new PA. | Pick one certified higher-gain antenna SKU; record its gain in cfg; enforce `Pconducted_max = ceiling - max(0, gain - 6)` in `sx1276_set_tx_power_dbm()`. |
| 10 | Multi-radio single-system base station (§11.3.1, §15) | "Promising but not free." | **Defer.** Aggregate emissions, intermod, and RF-exposure recomputation make this a separate certification effort. | Park behind the SDR-receive-only idea below. |
| 11 | DTS "safety burst" overlay on FHSS telemetry (§11.3.3) | Cautioned by §15. | **Defer.** Until DTS is certified as a second profile, safety bursts should be repeated high-priority FHSS packets, not regime-crossing transmissions. | Implement safety-burst as N-of-M repeated FHSS frames first; revisit only after DTS measurement gate (idea #2) passes. |
| 12 | Fixed-frequency rendezvous / control-channel allowance (§11.1.3) | **Retracted by §15.1.1.** The cited §15.247(a)(1)(iii) clause is the 2.4 GHz hopping-channel provision, not a 902-928 MHz rendezvous carve-out. | **Retired as cited.** A rendezvous behavior is still operationally useful, but only as deterministic in-profile FHSS recovery. | Replace with "slow-hop rendezvous mode" inside the 50-channel sequence, using a fixed pseudo-random seed and short sync packets. |
| 13 | Per-channel PA back-off table (§11.3.5, §15) | Endorsed conditionally. | **Keep, evidence-gated.** It must be backed by per-channel OOB measurements, or it becomes false precision. | During the spectrum pre-scan, record max legal PA per channel group; populate a lookup in `sx1276_set_tx_power_dbm()`. |
| 14 | Packet-airtime ≤ dwell-cap invariant (§11.3.6, §15) | Endorsed. | **Keep as a hard firmware invariant.** This single guard prevents an entire class of stuck-transmit compliance bugs. | Add a `pkt_toa_ms <= dwell_cap_ms - margin` check in the LoRa modem-config setter; reject illegal (SF, BW, payload) tuples up-front. |
| 15 | Passive SDR wideband monitor on Portenta X8 (§11.3.7, §15) | Endorsed. | **Keep, optional.** Strong reliability aid; no transmitter rule burden. | Treat as advisory input to the master blacklist (idea #7); never as a cross-device coordination channel under §15.247(h). |
| 16 | Deterministic per-farm hop seed (§11.3.8) | Endorsed. | **Keep.** Gives predictable inter-tractor reuse without illegal coordination. | Define hash function and `farm_id`/`node_id` cfg keys alongside the `sx1276_fhss` runtime module. |
| 17 | §15.23 home-built path as an OSE scaling/distribution strategy (§13.2, §13.5, §14) | **Substantially retracted by §15.1.2.** §15.23 is per-individual, ≤5 units, personal use, not OSE fleet/kit distribution. | **Retired as a distribution strategy.** Survives only as personal-builder guidance for individual community members. | Rewrite §13 and §14 in v1.1 so §15.23 is framed as individual-builder context, not an OSE pathway. |
| 18 | Part 97 amateur operation as a control-link path (§13.4) | **Retracted by §15.1.4.** Encrypted control + machine-work purpose violate amateur rules. | **Retired.** | Remove from forward-looking pathways; keep only as "considered and ruled out." |
| 19 | Pre-certified modular integration with "emulated" LoRaWAN PHY (§13.1, §13.5) | Section 15.1.3 narrowed it sharply. | **Keep only as a *grant-preserving SKU* concept.** Custom firmware is not automatically inside the module grant just because the spectrum shape resembles LoRaWAN. | Treat as a separate product SKU with a separate compliance file: either use the module exactly as authorized, or plan a C2PC/new authorization for the custom-RF SKU. |
| 20 | "Treat §15.5 development as the ongoing operating mode" (§13.3, §14) | Implicit throughout. | **Keep, with sharper boundaries.** §15.5 covers genuine R&D, not perpetual field operation of an uncertified product. | Define an explicit exit criterion from §15.5 (= the evidence gate that promotes the 50-channel profile to "field-candidate"). |
| 21 | Replace the 1-second fairness window in `sx1276_airtime.c` with true 10 s / 20 s dwell accounting (§9, §10, §11, §15) | Endorsed across passes. | **Confirmed must-do.** This is the most fundamental code-side compliance gap. | Add a second accountant class with a sliding 10 s window per-channel; expose both QoS and legal counters separately in URC. |
| 22 | Replace `s_channel_idx = 0U` in `sx1276_tx_begin()` with hop-selector + retune (§9, §10, §11, §15) | Endorsed across passes. | **Confirmed must-do.** This is the single largest gap between intent and runtime. | Introduce the `sx1276_fhss` module first; route `sx1276_tx_begin()` through it; keep the fixed-channel path only under a `BENCH_ONLY_FIXED_915` compile/cfg flag. |
| 23 | Profile-aware cfg validation (§10.6, §15.3) | Endorsed. | **Confirmed must-do.** Without this, `CFG_KEY_FHSS_CHANNEL_MASK` can carry a legally invalid set. | Validate by bitcount per profile and by membership in the certified channel table. |
| 24 | Compliance telemetry (active profile, hop histogram, max dwell, blacklist size, clamp reasons) (§10.7, §11, §15, §16) | Endorsed. | **Keep, scope tightly.** Don't ship every counter; ship the ones a future test report needs. | Add to `VER_URC` / a new `RFCO` URC: profile name, active channel count, current hop index, per-channel occupancy histogram (rolling), max dwell in current window, blacklist size, last clamp reason. |

### 17.3 The *new* gaps that no prior section addresses

1. **No security model around hopset updates and profile switches.** Sections
   7, 11, 15, and 16 mention authentication in passing but never specify what
   signs a hopset update or a profile-switch frame. Without that, the master-
   broadcast blacklist (idea #7) and any future profile transition (idea #6)
   are vulnerable to spoofing — which is both a reliability hazard and,
   potentially, a path to coercing a node into a noncompliant configuration.
   *Improvement:* Re-use the existing LoRa link key (the same MIC/AEAD path
   already used for control frames) to authenticate hopset epoch updates,
   profile-switch frames, and any cfg key that changes the regulatory
   envelope. Reject unauthenticated frames at the L072 cfg layer, not just at
   the host.

2. **No story for *time*.** §15.247 dwell rules are time-windowed (10 s,
   20 s). The L072 has a millisecond-resolution monotonic tick but no
   guaranteed wall clock; if the tick wraps or resets without the dwell
   accountant noticing, the window math is wrong silently.
   *Improvement:* Specify the monotonic source used by the dwell accountant,
   its wrap behavior, and the reset/boot policy. Treat any time discontinuity
   as a "drain to safe state, reset windows" event. Emit a counter for it.

3. **No explicit handling of the "first 50 packets after boot" problem.** Any
   sliding-window dwell accountant is empty at boot; an attacker or a buggy
   sender could burst on one channel during that warm-up window. The doc
   never names this case.
   *Improvement:* At cold start, force the first N packets to traverse the
   full hopset in pseudo-random order before allowing repeat selections. This
   is also good for spectrum behavior in front of a test-house analyzer.

4. **No regression strategy for the bench evidence already on file.** Section
   10.3 correctly notes that `lora_ping.c` writes BW125/+14 dBm/915 MHz while
   the live `sx1276_init()` now defaults to BW250. Existing bench artifacts
   were therefore captured under modem configurations that no longer match
   the shipping defaults.
   *Improvement:* Stamp every future bench artifact with the **exact** modem
   config from a runtime readback (RegFrf, RegModemConfig1/2/3, RegPaConfig,
   RegPaDac), not from the source comment. Re-run the most important S1.*
   artifacts under the current defaults before treating them as evidence.

5. **No clear separation between "compliance" and "interoperability".** A
   future LifeTrac firmware that is fully §15.247-compliant could still fail
   to interoperate with an earlier handheld/H7 path that uses the 8-channel
   helper in `lora_proto.c`. The doc treats the 8-channel helper as a
   regulatory problem; it is also a forward-compatibility problem.
   *Improvement:* Decide explicitly whether the 8-channel helper is
   (a) removed, (b) frozen as a legacy bench-only path, or (c) replaced
   in-place. Whichever choice is made, document the migration window and the
   mutual-rejection behavior between old and new peers.

6. **No mention of RF exposure (§1.1307 / §1.1310 / §2.1093).** §10.1 lists
   §15.247(i) once. For a tractor with antennas mounted near operators or
   bystanders, RF exposure evaluation is non-optional even at +17 dBm with
   higher-gain antennas. It belongs in the release checklist, not as a
   footnote.
   *Improvement:* Add a one-line RF-exposure gate to the release-evidence
   checklist: nominal antenna gain, mounting height, minimum separation
   distance, and whether MPE evaluation or categorical exclusion applies.

7. **No explicit treatment of receiver spurious emissions (§15.109).** Even
   the receive-only SDR monitor idea (#15) has receiver-emission obligations.
   The doc never mentions this.
   *Improvement:* When the SDR monitor is added, include its receiver
   spurious-emission status in the same compliance file as the transmitter.

### 17.4 Smallest viable improvement plan

If only a handful of changes from this section are accepted, do these four,
in this order:

1. **Add a "Current Decision" header block at the very top of the file** (see
   §17.1.1) so future readers do not need to infer precedence from section
   order. Mark sections 3.C, 8, 13.2, 13.4, and 13.5's "scaling" framing as
   `> SUPERSEDED`.
2. **Make `s_channel_idx = 0U` in
   [sx1276_tx.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c)
   the canonical "compliance gap" symbol** in the doc and in `TODO.md`. Every
   subsequent regulatory discussion should reference closing this gap as
   prerequisite work.
3. **Convert
   [sx1276_airtime.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c)
   into a two-class accountant**: keep the existing 1 s fairness window;
   add a 10 s per-channel legal-dwell window; expose both via URC; never let
   the firmware advertise the fairness counter as compliance evidence.
4. **Add the packet-airtime ≤ dwell-cap invariant** (#14) at the modem-config
   setter in [sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c).
   It is a one-screen change that removes a whole class of future compliance
   bugs and does not depend on any of the larger architectural work.

The remaining ideas (per-channel PA back-off, SDR monitor, deterministic
per-farm seed, security-signed hopset updates, multi-radio base, DTS profile)
should be staged behind those four; they all become easier once the core
hop scheduler and dwell accountant exist.

### 17.5 One-paragraph bottom line

The document's regulatory analysis is now strong enough; the document's
*structure* and the *firmware* are the remaining weak links. Add a Current
Decision header so the precedence is unambiguous, retire ideas #1, #3, #12,
#17, and #18 explicitly, gate ideas #2 and #11 behind one bench measurement
each, and treat the four code changes in §17.4 as the minimum bar before any
profile in this document can honestly be called "field-candidate" rather than
"bench-only."

---

## 18. Final Review and Recommendations for LoRa Radio Implementation

Based on the extensive, multi-layered analysis of the FCC Part 15 regulations, hardware constraints (Murata SX1276 limits), and OSE's strategic decision to prioritize custom control firmware (e.g. single static channel or bespoke low-channel-count MAC) via the **§15.23 (Home-Built)** and **§15.5 (Prototyping)** exemptions, here is the final assessment.

### Executive Verdict
By opting for custom firmware in the near term, we explicitly forfeit the regulatory safety net of the Murata module's pre-certified FCC ID modular grant. However, proceeding under the hobbyist and prototyping exemptions legally permits this level of software freedom and preserves low-latency control logic—as long as we strictly abide by the distribution and physical red lines.

### Final Recommendations

1. **Hardware & Firmware Safety Guardrails (Must Implement Now)**
   * **Frequency Bounds:** Add hard constraints in `sx1276.c` rejecting any `$RegFrf$` tuning below `902.3 MHz` or above `927.7 MHz`. This is a non-negotiable safeguard to prevent bleeding into licensed bands.
   * **Power Limit Clamp:** Keep the absolute `PA_BOOST` maximum clamped at +17 dBm. Do not expose limits higher than this without strict, certified hardware architecture upgrades. 
   * **Dwell Accounting:** As prioritized in the previous critique, update `sx1276_airtime.c` to include strict dwell limits alongside fairness limits to ensure we cannot accidentally exceed the per-time-window constraints, even in a prototype.

2. **Compliance by System Design (Red Lines)**
   * **No Assembled Sales:** The §15.23 exemption requires that the "Home-Built" device is *built* by the builder. To scale legally without a lab test, OSE must distribute these systems as component kits + open-source software, *never* as pre-flashed, turnkey RF products. 
   * **No External RF Amplifiers:** Do not supplement the system with aftermarket RF boosters. Stick to passive, high-gain directional antennas on the base station if asymmetric link-budget enhancements are required.
   * **Immediate Interference Deference:** You have no legal rights to the spectrum. Any interference report from a primary/licensed user requires an immediate shutdown. 

3. **Technical Debt and Future-Proofing**
   * While the exemptions cover OSE prototyping and kit-level distribution, true commercial scale (e.g., shipping turnkey tractors) will absolutely demand closing the compliance gap. 
   * Maintain the goal of implementing a full **50-channel FHSS (BW250)** protocol as a core roadmap milestone in `TODO.md`. Evolving the codebase back into alignment with the physical parameters of the Murata module's original lab tests is the only path to reclaiming its zero-cost FCC ID for commercial operations down the road.

4. **50-Channel FHSS Implementation Strategy & Latency Impacts**
   * **Lag Considerations:** Migrating from a static channel to a 50-channel hopping scheme introduces two potential sources of lag: synchronization overhead and retuning delays. However, a well-optimized FHSS MAC layer mitigates these. By utilizing a deterministic, time-based pseudo-random sequence generated from a shared seed, both the tractor and base station inherently know the next channel without requiring pre-packet handshakes. 
   * **Negligible Latency Penalty:** The only irreducible delay is the SPI serial transmission to update `$RegFrf$` before a transmission and the SX1276's internal PLL (Phase-Locked Loop) lock time. The PLL settling time is typically around ~60 to 100 microseconds. Even when hopping on *every single packet*, this sub-millisecond retuning penalty is mechanically and acoustically invisible to a tractor control loop operating at 20–50 Hz. Therefore, moving to a compliant 50-channel FHSS model will **not** undermine OSE's stringent low-latency operational goals, provided the synchronization logic is implemented efficiently in the un-blocking RTOS thread.

*Signed:* **Copilot, v2.0 (Final Regulatory Assessment)**

---

## 19. Final recommendation lock (2026-05-19)

This section is the final implementation recommendation and should be treated
as the highest-precedence decision block in this document.

### 19.1 Final decision

1. Primary production path: 47 CFR 15.247 FHSS with 50 active channels.
2. Keep the present firmware power ceiling (+17 dBm clamp) for first compliant
  rollout; do not design around +30 dBm operation at this stage.
3. Keep single-channel operation as bench-only and clearly labeled as such.
4. Treat DTS BW500 as a secondary profile that is evaluated only after the
  FHSS path is implemented, measured, and stable.

### 19.2 Final thoughts on the 50-channel FHSS plan

The 50-channel FHSS plan is the right choice for LifeTrac.

Why it is the best fit now:
1. It is the strongest legal architecture for 902-928 MHz operation under
  Part 15.247 because channel count and dwell are structural properties,
  not heuristic limits.
2. It provides materially better interference resilience than lower-channel
  alternatives.
3. It preserves future headroom while remaining compatible with current
  SX1276 limits and the existing +17 dBm clamp.

What must be true before claiming field-candidate status:
1. Active TX hop selection is implemented in the runtime path.
2. RX hop synchronization is reliable and deterministic.
3. Per-channel dwell accounting uses regulatory windows and is exposed in logs.
4. Occupied-bandwidth, OOB, and power evidence are measured under
  continuous-data worst-case conditions.
5. Production RF settings are profile-locked, not freely host-tunable.

### 19.3 Implementation sequence (locked)

1. Compliance core:
  hop scheduler, pre-TX retune, RX sync, dwell accountant, profile guardrails.
2. Compliance observability:
  profile/channel/dwell/power telemetry in TX stats and evidence scripts.
3. Reliability add-ons inside the same profile:
  quality-aware blacklist with legal channel-floor and epoch rollback.
4. Secondary profile evaluation:
  DTS BW500 only after FHSS evidence gates pass.

*Signed:* **Copilot, v3.2 (Final Implementation Lock)**

---

## 20. Final LoRa radio implementation recommendation (2026-05-19)

This is my final review after reading the full document, including the later
Sections 16-19. Treat this section as the highest-precedence recommendation in
this note. Where it conflicts with earlier sections, this section supersedes
them.

### 20.1 Final verdict

The LifeTrac LoRa implementation should use **50-channel §15.247 FHSS as the
primary and only first field-candidate radio profile**. Keep the current
Murata/SX1276 +17 dBm conducted clamp for the first compliant rollout. Do not
build the first field plan around +30 dBm, external amplifiers, §15.249,
Part 97, a fixed rendezvous channel, or the §15.23 home-built exemption.

The winning production shape is deliberately boring:

```text
BENCH_ONLY_FIXED_915          -> lab bring-up only, never field-compliant
FCC_15_247_FHSS_50CH          -> first field-candidate and production target
FCC_15_247_DTS_BW500          -> later secondary mode after measurement
FCC_15_249_LOW_POWER          -> not viable on current PA_BOOST path
```

For now, the practical target is not "maximum legal transmit power." It is a
measured, observable, profile-locked FHSS system that stays legal even under a
continuous-data worst case. Once that exists, higher power, DTS recovery, and
advanced antenna SKUs can be evaluated as controlled extensions.

### 20.2 My thoughts on the 50-channel FHSS plan

I think the 50-channel FHSS plan is the correct plan for LifeTrac. It is the
best balance of legality, range, robustness, and future headroom.

Why I like it:

1. **It makes compliance structural.** A real 50-channel hop scheduler with
  equal average use and per-channel dwell limits is much stronger than trying
  to prove good behavior from a single-channel duty limiter.
2. **It gives more margin than 25-channel FHSS.** A 25-channel BW250 system is
  legal at the +24 dBm tier, but continuous traffic consumes essentially the
  whole 10-second dwell allowance. A 50-channel plan spreads the same traffic
  over twice as many channels and is less brittle.
3. **It preserves the +30 dBm regulatory tier for future hardware.** Current
  hardware should stay at +17 dBm, but choosing 50 channels now avoids a
  future architecture rewrite if a certified higher-power radio path is ever
  added.
4. **It should not hurt tractor-control latency if implemented cleanly.** The
  extra work is a register retune and PLL settle before packet TX, which is
  tiny compared with a 20-50 Hz machine-control loop. The real latency risk is
  not hopping itself; it is bad synchronization design, blocking firmware, or
  excessive retry behavior.
5. **It improves resilience.** A tractor link operating across the band is less
  exposed to one local interferer, one bad antenna notch, or one noisy channel
  than the present fixed 915 MHz path.

The caveats are equally important:

1. **50 channels in config is not compliance.** The radio must actually retune
  before TX, and the receiver must follow the hop sequence.
2. **Measured occupied bandwidth decides the dwell branch.** If measured 20 dB
  bandwidth is >=250 kHz and <=500 kHz, use the 10-second dwell window. If it
  measures below 250 kHz, 50 channels are still plausible, but the dwell window
  is 20 seconds.
3. **Every packet must fit the 400 ms cap.** The SX1276 LoRa packet path cannot
  hop mid-packet, so high-SF or large-payload frames must be rejected,
  fragmented, or moved to a different certified profile.
4. **A fixed rendezvous channel remains unapproved.** Resynchronization should
  happen over the legal hop set unless a compliance expert confirms a specific
  FCC/KDB-supported control-channel design.
5. **50-channel custom firmware does not automatically preserve the Murata or
  carrier FCC grant.** It is the right technical architecture, but grant scope
  still has to be verified against the actual filings and integration manual.

### 20.3 Required implementation plan

Implement the radio in this order:

1. **Add an explicit regulatory profile enum.** The L072 firmware should know
  whether it is running `BENCH_ONLY_FIXED_915`, `FCC_15_247_FHSS_50CH`, or a
  later certified profile. Production builds must reject arbitrary host-set RF
  parameters outside the selected profile.
2. **Create a real `sx1276_fhss` module.** It should own the certified channel
  table, active channel mask, pseudo-random permutation, hop epoch, channel
  index, and RX/TX synchronization metadata.
3. **Replace the fixed-channel TX path.** In
  [sx1276_tx.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c),
  the `s_channel_idx = 0U` path is the main compliance blocker. TX must ask
  the FHSS module for the next legal channel, retune, run LBT if enabled, then
  reserve dwell and transmit.
4. **Add legal dwell accounting beside fairness accounting.** Keep the 1-second
  fairness/QoS limiter if useful, but add a separate per-channel 10-second or
  20-second sliding regulatory window. Logs and evidence must label these as
  different counters.
5. **Enforce packet airtime limits.** The profile should reject any modem and
  payload combination whose estimated on-air time exceeds the applicable
  per-channel dwell cap, with margin.
6. **Make antenna and power part of the profile.** The firmware should clamp
  conducted power using the selected regulatory tier and the certified antenna
  gain. The current +17 dBm maximum remains the hard ceiling for first field
  work.
7. **Stamp every RF artifact.** Bench and field evidence should record profile,
  frequency/channel, SF, BW, CR, power, antenna/SKU, channel count, dwell max,
  hopset epoch, and firmware hash.

### 20.4 Channel plan guidance

Start with a simple measured channel table, not a clever adaptive one. A good
first candidate is 50 centers spaced about 500 kHz apart across 902-928 MHz,
with guard room at both band edges. Do not freeze the exact first and last
centers until the real SX1276 emission is measured for 20 dB bandwidth,
out-of-band shoulders, harmonics, and edge-channel behavior.

Use a pseudo-random permutation of all active channels per superframe, then
repeat with a new permutation. This gives equal average use by construction and
prevents pathological short-term clustering. Quality-aware blacklisting can be
added later, but only with hard guardrails: authenticated hopset updates,
future-epoch activation, rollback to the previous hopset, and a refusal to run
if fewer than 50 certified channels remain active.

### 20.5 What not to do

Do not ship or field-label any of these as compliant:

1. The current fixed 915 MHz runtime path.
2. The legacy 8-channel helper.
3. A 25-channel fallback presented as the main maximum-use architecture.
4. A fixed emergency/rendezvous channel without a test-house citation.
5. A dynamic FHSS/DTS/§15.249 mode switcher as the first implementation.
6. A §15.23 or kit-distribution strategy as a substitute for equipment
  authorization or modular-grant verification.
7. Any external RF amplifier or arbitrary antenna substitution.

### 20.6 Release gates before field-candidate status

The first field-candidate LoRa build should not be declared ready until all of
these are true:

1. Runtime TX hopping is proven on hardware with a 50-channel histogram.
2. RX synchronization survives packet loss, reboot, and clock drift without a
  fixed non-hopping control channel.
3. The dwell accountant proves <=400 ms per channel in the correct regulatory
  window under continuous-data stress.
4. Occupied bandwidth, out-of-band emissions, conducted power, and spurious
  emissions are measured for the exact profile and antenna path.
5. Host configuration cannot select non-profile RF settings in production mode.
6. Evidence scripts produce a repeatable report that ties logs to firmware hash
  and hardware SKU.

### 20.7 Final recommendation

Build the 50-channel FHSS system first. Keep it simple, measured, and locked
down. The present single-channel work is valuable bench infrastructure, but it
must remain labeled bench-only. The next real engineering milestone is not more
regulatory creativity; it is closing the concrete firmware gap between the
documented plan and the radio path that currently still transmits on one
channel.

*Signed:* **GitHub Copilot, LoRa Radio Implementation Final Review v4.0
(2026-05-19)**

---

## 21. Final review v5.0 — consolidated recommendation (2026-05-19)

This section is the consolidated final review after sections 1-20. The
document has accumulated many overlapping "final" passes (v2.0, v3.0, v3.1,
v3.2, v4.0); the purpose of v5.0 is to **collapse them into one decision
record**, give a direct opinion on the 50-channel FHSS plan, and stop adding
new architectural surface.

### 21.1 The situation in one paragraph

The LifeTrac LoRa link runs an SF7/BW250 LoRa profile at +14 dBm on a single
915.000 MHz channel via the Murata SX1276 PA_BOOST path on the L072. The
firmware *talks* like a hopping system (cfg keys, 8-channel helper, airtime
gate, comments referencing FHSS) but the live TX path in
[sx1276_tx.c L64](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c#L64)
hardcodes `s_channel_idx = 0U` and never retunes between packets. The
[sx1276_airtime.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c)
gate is a 1-second fairness limiter, not the §15.247 10 s / 20 s dwell window.
That means the current build is legitimate §15.5 bench work and nothing more.
Everything else in this document is design space for the next build, not a
description of today's compliance posture.

### 21.2 Direct opinion on the 50-channel FHSS plan

I endorse **`FCC_15_247_FHSS_50CH_BW250`** as the first production profile.
The reasoning, stated plainly:

1. **It is the only profile that simultaneously gives headroom in all four
  axes that matter:** power tier (50-ch unlocks the +30 dBm ceiling even
  though we will sit at +17 dBm), dwell margin (50 × 400 ms = 20 s of
  aggregate budget per 10 s window — i.e. 2× continuous-TX headroom),
  spectrum footprint (50 × ~500 kHz centers fits inside 902-928 MHz with
  guard room), and link budget (SF7/BW250 keeps packet airtime ~13 ms,
  well under the 400 ms per-channel cap).

2. **It is forward-compatible with every realistic hardware roadmap.**
  Today's +17 dBm clamp, a future external PA at +24 or +30 dBm, a future
  higher-gain base antenna with the §15.247(b)(4) 1-dB-per-dB conducted
  back-off — all stay inside the same channel plan and the same dwell
  accountant. The regulatory architecture does not have to be rewritten
  when the RF front-end is upgraded.

3. **It is the only plan that is not falsified by something already in this
  document.** 25-channel BW250 has zero aggregate dwell margin under
  continuous TX. 50-channel BW125 runs into per-packet airtime ceilings at
  high SFs. DTS BW500 is a useful secondary mode but pulls in an
  occupied-bandwidth and PSD measurement burden that we do not have
  certified hardware for yet. §15.249 is unreachable on the present
  PA_BOOST path with a 2 dBi antenna.

4. **It re-uses code that already exists.** The cfg-key surface
  (`CFG_KEY_FHSS_ENABLE`, `CFG_KEY_FHSS_CHANNEL_MASK`,
  `CFG_KEY_FHSS_DWELL_MS`, `CFG_KEY_FHSS_QUALITY_AWARE`) in
  [host_cfg.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c)
  needs to be widened to 64 bits and rewired to a real selector, not
  redesigned from scratch. LBT in
  [sx1276_lbt.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_lbt.c)
  stays as the intra-system quality avoidance layer. The
  [lora_proto.c](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c)
  pseudo-random helper is the right shape, wrong size — generalize it from 8
  to N channels.

**Risks I want flagged with the recommendation:**

* The 20 dB occupied bandwidth of SF7/BW250 on this exact SX1276 module
  has *not* been measured. If it falls below 250 kHz, the 50-channel plan
  is still legal but moves to the narrowband branch with a 20-second dwell
  window. Cheap to verify on a spectrum analyzer; expensive to assume.
* "50 channels" is a design floor, not a feature. Quality-aware blacklisting
  must refuse to drop below 50 active channels — ever — or the system
  silently transitions out of the +30 dBm tier into the +24 dBm tier
  without anyone noticing in firmware.
* Per-packet airtime must be enforced as an invariant (`max_packet_airtime
  < dwell_cap_ms`), because the SX1276 LoRa packet engine cannot hop
  mid-packet. The combination "long SF + large payload" must be rejected
  at the modem-config setter, not just discouraged in docs.

### 21.3 Final implementation recommendation

Do exactly these things, in this order, and stop:

1. **Replace `s_channel_idx = 0U` in
  [sx1276_tx.c L64](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c#L64)**
  with a call into a new `sx1276_fhss` module. The selector returns the
  next channel from a pseudo-random permutation of the active set;
  `sx1276_tx_begin()` retunes via `sx1276_set_frequency_hz()` before LBT
  and before FIFO write. This single change converts the firmware from
  "claims to hop" to "hops."

2. **Add the `sx1276_fhss` module** with: 64-channel candidate table, active
  mask (default = all 50 production channels set), per-superframe Fisher-
  Yates permutation seeded from `(farm_id, node_id, epoch)`, and a public
  `next_channel()` API used only inside `sx1276_tx_begin()`.

3. **Rewrite
  [sx1276_airtime.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c)
  as a true dwell accountant** with a 10 s sliding window per channel,
  hard cap at 400 ms. Keep the existing 1-second fairness limiter as a
  separate optional QoS layer with a different name so a test report can
  never confuse them.

4. **Enforce profile invariants at the modem-config setter** in
  [sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c):
  reject (SF, BW, payload) tuples whose computed airtime exceeds the dwell
  cap; reject TX power above the active profile's per-tier ceiling minus
  the antenna-gain back-off.

5. **Emit compliance telemetry in a URC**: active profile name, active
  channel count, current channel index/frequency, per-channel histogram
  snapshot, max dwell in window, LBT abort count. This is what makes the
  next bench evidence file self-describing instead of inferred.

6. **Stamp every evidence artifact** going forward with the profile name
  (`BENCH_ONLY_FIXED_915` or `FCC_15_247_FHSS_50CH_BW250`), the firmware
  hash, the antenna SKU, and the TX power. Backfill the existing
  walk_power and S1.x artifacts with `BENCH_ONLY_FIXED_915` so no future
  reader confuses them with field-compliant data.

Everything beyond this — DTS overlay, rendezvous control channel, asymmetric
per-direction profiles, multi-radio diversity, wideband SDR monitor, dynamic
mode-switcher, §15.249 low-power SKU — is **deferred**. They are real options
worth keeping in the design space, but adding any of them before steps 1-6
land would slip the only thing that actually changes the compliance posture
of the shipping firmware.

### 21.4 What I am explicitly *not* recommending

* **Not** the 25-channel BW250 plan as a stepping stone. The dwell math is
  too tight and the migration to 50 channels is the same engineering work
  done twice.
* **Not** the 8-channel legacy helper as a transition. Mark it bench/legacy
  in code comments and stop scheduling work against it.
* **Not** chasing +24 or +30 dBm conducted on the current hardware. The
  +17 dBm clamp is correct for now; the leverage is in the channel plan,
  not the PA.
* **Not** §15.249 as a software-only mode on the present PA_BOOST path. It
  needs a hardware-level low-power path, which is a separate project.
* **Not** any more "final review" sections in this document. v5.0 is the
  decision. Future updates belong in a new dated note that references the
  bench evidence proving steps 1-6 landed.

### 21.5 Definition of done for the first compliant LoRa build

The next field-candidate build can claim §15.247(a)(1) compliance only when
**all** of these are simultaneously true and proven on the actual hardware:

1. A 60-second continuous-TX capture on a spectrum analyzer shows energy on
  ≥ 50 distinct channels with statistically equal average use.
2. Per-channel dwell never exceeds 400 ms in any 10-second window under
  worst-case continuous-data load.
3. 20 dB occupied bandwidth of every channel is measured and recorded.
4. Out-of-band emissions are ≥ 20 dB below in-band per §15.247(d), and
  restricted-band/spurious limits per §15.205/§15.209 are met.
5. Conducted power into the certified antenna path is ≤ the per-tier
  ceiling minus the §15.247(b)(4) gain back-off.
6. RX synchronization recovers from packet loss, reboot, and clock drift
  without falling back to a fixed channel for more than the
  §15.247(a)(1)(iii) intra-system control allowance.
7. Host configuration cannot select a non-profile RF setting; profile
  identity is reported in `VER_URC` and in every bench artifact.

Until items 1-7 are simultaneously true, the LoRa link is bench hardware
running under §15.5, regardless of what the cfg keys or doc comments say.

*Signed:* **GitHub Copilot, LoRa Radio Implementation Final Review v5.0
(2026-05-19) — consolidated; supersedes v2.0, v3.0, v3.1, v3.2, v4.0**

---

## 22. Impact on the Method G custom-firmware plan (2026-05-19)

This section addresses how the v5.0 recommendation (50-channel BW250 FHSS as
the first production profile) interacts with the **Method G** decision
recorded in
[2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md](2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md).
Short version: it **strengthens the case for Method G, but invalidates two
specific scope claims** in the original Method G note that must be updated
before any new field-candidate build.

### 22.1 Net direction: Method G becomes more justified, not less

The Method G note argued Method G was the only path that preserves
"per-frame FHSS, adaptive SF, three-profile swap, 25 ms fragment cap,
20 Hz cadence." The v5.0 FCC review makes that argument stronger:

* AT-command paths (Methods D/F) cannot perform per-packet `RegFrf` retune
  inside the 400 ms dwell window under firmware control. They can only
  obey whatever channel plan the AT stack's MAC layer ships with —
  typically a LoRaWAN US915 sub-band plan that is **not** the same as a
  §15.247(a)(1) 50-channel FHSS plan and does not expose the dwell
  accountant required by §22.2 below.
* An external SX1276 (Method E) would work but adds a permanent BOM cost
  and antenna burden purely to do something the on-module L072 can already
  do once Method G is in.
* The v5.0 implementation steps — replacing `s_channel_idx = 0U` in
  [sx1276_tx.c L64](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c#L64),
  building a `sx1276_fhss` module, rewriting `sx1276_airtime.c` as a true
  dwell accountant, enforcing per-packet airtime invariants — are **Method
  G work, by definition**. There is no version of those steps that lives
  outside a custom L072 firmware path.

So the v5.0 review converts the Method G decision from "right if the
project is long-lived" (the original conditional) to "**required if the
device is ever to ship as §15.247(a)(1) compliant**." The conditional
becomes a constraint.

### 22.2 Two scope claims in the Method G note that v5.0 invalidates

The Method G note carries two claims that the v5.0 FCC review falsifies.
These must be corrected in the Method G plan before the next Phase 3
estimate is trusted:

1. **"FHSS, 8 channels per frame ✅" and "FCC §15.247 hopping argument
  intact ✅"** in the Method G scorecard (§4 of the Method G note,
  line ~90 of
  [2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md](2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md))
  are **wrong as written**. 8 channels does not satisfy §15.247(a)(1)(i)
  in 902-928 MHz under any sub-tier: the lowest legal channel count is 25
  (BW ≥ 250 kHz, +24 dBm tier) and the next is 50 (any BW, +30 dBm tier).
  The Method G scorecard should read **"FHSS, 50 channels per
  super-frame, BW250 production profile"**, and the "§15.247 argument
  intact" tick is only valid against the corrected channel count and the
  real dwell accountant.

2. **Phase 3's "Port `lora_proto.c` … FHSS"** line item
  (Method G note §7, ~line 141) underestimates the FHSS sub-task. A
  straight port of the 8-channel pseudo-random helper in
  [lora_proto.c](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c)
  to the L072 does **not** produce a compliant system. The real Phase 3
  FHSS deliverable is the four-part scope from §21.3 of this document
  (`sx1276_fhss` module + selector wired into `sx1276_tx_begin()`,
  dwell accountant rewrite, modem-config invariant guard, compliance
  URC). That is closer to a week of focused work in addition to the
  legacy-helper port, not a sub-bullet inside it.

### 22.3 Phase-by-phase deltas to the Method G plan

Mapped onto the Method G phases:

| Method G phase | Original scope | v5.0 delta |
|---|---|---|
| Phase 1 (HW bring-up) | Flash custom image on Board2 | No change. Already partly proven by the `flash_w1_9_*.log` series and `lifetrac_p0c/method_g_stage1.log`. |
| Phase 2 (strip & re-host) | Remove LoRaMac, keep SX1276 driver + RTOS + UART, define UART protocol | No change. The cfg-key surface in [host_cfg.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c) is already there. |
| Phase 3 (port `lora_proto.c`) | Frame format, AES-GCM, KISS/COBS, FHSS, replay | **+1 week**: replace the 8-channel helper with a 50-channel pseudo-random permutation; widen `CFG_KEY_FHSS_CHANNEL_MASK` from 8-bit to 64-bit; build `sx1276_fhss` module; rewrite `sx1276_airtime.c` for 10 s window per-channel dwell. |
| Phase 4 (adaptive SF + per-frame retune) | SF ladder, three-profile swap, measured retune cost | **Reframed, not enlarged**: per-frame retune is no longer just a latency optimization — it is the legal mechanism that converts the radio from "claims to hop" to "hops." Measured retune cost goes into the airtime accountant. |
| Phase 5 (priority queue + P0 preempt) | Queue + radio-abort path | No change in scope, but the abort path must release the dwell reservation for the aborted channel via `sx1276_airtime_release()`; otherwise the accountant double-charges and the legal headroom shrinks. |
| Phase 6 (HIL bring-up + R-series) | R-1..R-7, L1, L3, L-V11 | **+spectrum-analyzer evidence**: add the seven-item §21.5 definition-of-done as a hard gate before any field-candidate label. The 60-second 50-channel-histogram capture is new HIL work, not covered by the original R-series. |
| Phase 7 (hardening) | Watchdog, brown-out, OTA, signed binaries | **+profile lock**: production firmware must refuse to accept cfg keys that select a non-profile RF setting (TX power above per-tier ceiling, channel count below 50, SF/payload combinations whose airtime exceeds the dwell cap). |

Net schedule impact: **roughly +1 week added to Phase 3 and a measurable
HIL/spectrum-analyzer block added to Phase 6.** The 4-6 week Method G
estimate becomes 5-7 weeks, dominated by Phase 6 instrumentation rather
than by the FHSS code itself.

### 22.4 What stays unchanged

* Method G remains a no-regrets investment relative to Methods D/E/F for
  the production tractor path; v5.0 only sharpens the scope, not the
  identity of the chosen method.
* The hybrid sequencing in the Method G note (Method D Step 1 today →
  `hardwario/lora-modem` fork in week 1-2 → decision gate) is unaffected.
  v5.0 only affects what gets coded **inside** the Method G fork, not
  whether to start one.
* The brick-recovery guarantee (H7 always retains BOOT0/NRST control)
  remains the right operational safety floor. It is more important under
  Method G now that the same firmware also owns the regulatory profile
  lock — a brick recovery path is the rollback story if a future profile
  change ships broken.

### 22.5 Recommended cross-document updates

The following documents should be touched after this section lands, in
this order:

1. [2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md](2026-05-04_Murata_L072_Custom_Firmware_Method_G_Copilot_v1_0.md):
  correct the "8 channels" scorecard entries and tag Phase 3/Phase 6
  scope deltas with a pointer back to this section.
2. [LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md): replace any
  remaining "8-channel FHSS" language with "50-channel BW250 production
  profile" and add the dwell-window definitions.
3. [LORA_IMPLEMENTATION.md](../DESIGN-CONTROLLER/LORA_IMPLEMENTATION.md):
  add `sx1276_fhss` to the L072 module list and clarify that
  [sx1276_airtime.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c)
  becomes a dwell accountant, not a fairness limiter.
4. [DECISIONS.md](../DESIGN-CONTROLLER/DECISIONS.md): record that the
  Method G scope was tightened to "first production profile =
  `FCC_15_247_FHSS_50CH_BW250`" and that 8-channel operation is bench/
  legacy only.
5. [MASTER_TEST_PROGRAM.md](../MASTER_TEST_PROGRAM.md): add the
  seven-item §21.5 definition-of-done as a W4-FCC tier between W4-pre
  and the field-trial gate.
6. [TODO.md](../TODO.md): replace the 8-channel R-01 baseline item with
  a 50-channel BW250 R-01' baseline item; demote the 25-channel fallback
  to a non-blocking alternative.

### 22.6 One-line answer

> **The 50-channel BW250 plan does not invalidate Method G; it tightens
> Method G's Phase 3/Phase 6 scope by about one week, replaces the
> 8-channel scorecard claim with a 50-channel one, and converts Method G
> from "preferred for long-lived projects" to "required for any §15.247
> field-compliant build."**

*Signed:* **GitHub Copilot, LoRa Radio Implementation Final Review v5.1
(2026-05-19) — Method G impact addendum to v5.0**


