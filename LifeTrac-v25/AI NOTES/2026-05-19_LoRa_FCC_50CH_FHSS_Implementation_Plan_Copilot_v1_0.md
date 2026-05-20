# LoRa FCC §15.247 50-channel FHSS — Implementation Plan

**Author:** Copilot (assistant-authored)
**Date:** 2026-05-19
**Status:** v3.0 — review consolidation (folds in v2.0/v2.1/v2.2/v2.3 addenda)
**Scope:** Translates the regulatory decision locked in
[2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md](2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md)
§19-§20 into a concrete, staged firmware + bench-evidence plan against the
current L072 source tree.
**Precedence:** code > evidence > doc. This doc is a plan, not authority.
**Section precedence inside this doc:** where §14 (consolidated v3.0
deltas) conflicts with §1-§9 (original v1.0), §14 wins. §10-§13 are the
historical v2.0-v2.3 review chain and are preserved unedited for trail.

---

## 0. Decision recap (from the FCC notes, locked)

| Item | Value | Source |
|---|---|---|
| Primary production profile | `FCC_15_247_FHSS_50CH_BW250` | FCC notes §19.1, §20.1 |
| TX power for first compliant rollout | current **+17 dBm** firmware clamp | FCC notes §19.1.2 |
| Secondary profile | `FCC_15_247_DTS_BW500` (post-measurement only) | FCC notes §19.1.4 |
| Bench-only profile | `BENCH_ONLY_FIXED_915` | FCC notes §19.1.3 |
| Retired | §15.249 production, Part 97, fixed-frequency rendezvous, dynamic regime switching, §15.23 as OSE scaling strategy | FCC notes §17.2 verdicts #1, #12, #17, #18, #6 |

The single-sentence north star (FCC notes §11.7): replace `s_channel_idx = 0U`
in [sx1276_tx.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c#L64)
with a real 50-channel hop selector + pre-TX retune, and account dwell in a
true 10-second window.

---

## 1. Current code truth (verified 2026-05-19)

| Path | Current behavior | Compliance role |
|---|---|---|
| [sx1276_tx.c L64](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c#L64) | `s_channel_idx = 0U;` hardcoded; no hop selection, no pre-TX retune | **Top compliance gap.** Runtime is fixed-channel regardless of cfg. |
| [sx1276_airtime.c L11-L13](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c#L11-L13) | `CHANNEL_COUNT=16`, `WINDOW_MS=1000`, `BUDGET_US=400000` — 1 s fairness gate | **Wrong invariant** for FCC dwell (10 s / 20 s, 400 ms per channel). Keep as QoS, add a second accountant. |
| [sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) `sx1276_set_tx_power_dbm()` | clamps 2..17 dBm; no per-channel back-off; no antenna-aware ceiling | Add profile-aware + antenna-aware clamp. |
| [host_cfg.c L90-L94](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c#L90-L94), [host_cfg_keys.h L10-L23](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_keys.h#L10-L23) | `CFG_KEY_FHSS_ENABLE`, `CFG_KEY_FHSS_CHANNEL_MASK` (u64), `CFG_KEY_FHSS_DWELL_MS`, `CFG_KEY_FHSS_QUALITY_AWARE` exist; quality-aware marked unsupported; mask validated only as nonzero | Scaffolding present, not consumed by TX runtime. Add profile-aware validation. |
| [lora_proto.c](../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c) | 8-channel hop helper (`#ifdef LIFETRAC_FHSS_ENABLED` on H7/handheld side) | Below 50-channel floor; freeze as legacy/bench, do not extend. |
| [sx1276_lbt.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_lbt.c) | CAD/RSSI backoff before TX | Reliability layer inside any profile; not the profile itself (§15.247(h)). |

Bench evidence on file ([walk_power_full_2026-05-19](../DESIGN-CONTROLLER/bench-evidence/walk_power_full_2026-05-19/README.md),
[mixed_load_2026-05-19](../DESIGN-CONTROLLER/bench-evidence/mixed_load_2026-05-19/README.md))
all captured under `BENCH_ONLY_FIXED_915`. None of it is FCC-field evidence.

---

## 2. Staged plan

Three tracks, executed in order. Track A is release-critical; B and C only
start when A is green.

### Track A — Compliance core (release-blocking)

Goal: every TX comes from a hop scheduler, every TX is dwell-accounted in a
regulatory window, every TX is bounded by a packet-airtime invariant.

**A1. Packet-airtime ≤ dwell-cap invariant** *(smallest change, lands first)*
- Where: [sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) modem-config setter.
- Add: compute `pkt_toa_ms` from (SF, BW, CR, payload, preamble, header,
  CRC) per Semtech AN1200.13; reject tuples where
  `pkt_toa_ms > dwell_cap_ms - guard_ms` (default `dwell_cap_ms=400`,
  `guard_ms=20`).
- Acceptance: SF12/BW125 at max payload is **rejected**; SF7/BW250 at 24 B
  passes. Unit test in `bench/host_proto/` lookup tables.
- Falsification target #3 in FCC notes §11.5.

**A2. Split `sx1276_airtime.c` into two accountants**
- Keep: existing 1 s / 400 ms QoS fairness gate (rename internal counter
  `qos_used_us`).
- Add: 10 s sliding per-channel **legal-dwell** accountant
  (`legal_used_us_10s`), with channel count widened to **64** to support
  50 active + blacklist slack.
- Add: 20 s variant gated by `profile_dwell_window_ms` (so a future
  measured-narrowband profile can switch to the 20 s rule per FCC
  §15.247(a)(1)(i)).
- Expose both counters separately in URC (see Track B); never let the QoS
  gate be reported as compliance evidence.
- Acceptance: synthetic load that intentionally over-spends one channel
  triggers `__AIRTIME_LEGAL_BLOCK__`, not just `__AIRTIME_QOS_BLOCK__`.

**A3. New `sx1276_fhss` runtime module**
- Files: new `radio/sx1276_fhss.c` + `radio/sx1276_fhss.h`.
- Owns:
  - 50-channel center-frequency table (interim: 500 kHz spacing across
    902.5–927.5 MHz; final list gated on bench occupied-BW measurement, see
    Phase D below).
  - Pseudo-random permutation per super-frame
    (`permute(seed = H(farm_id || node_id || epoch))`).
  - Equal-use-by-construction (one channel per slot, full set per epoch).
  - Quality-aware **blacklist with legal floor**: if active channel count
    would drop below 50, refuse to blacklist (FCC notes §10.6 #5).
  - Cold-start warm-up (force first 50 packets to traverse the hopset
    once, FCC notes §17.3 #3).
- Public API (sketch): `sx1276_fhss_next_channel(uint8_t *idx, uint32_t *hz)`,
  `sx1276_fhss_blacklist(uint8_t idx)`, `sx1276_fhss_active_count()`,
  `sx1276_fhss_epoch_advance()`.
- Time source: existing monotonic ms tick; record wrap/reset events as a
  counter, drain windows on discontinuity (FCC notes §17.3 #2).

**A4. Route TX through the hop scheduler**
- Where: [sx1276_tx.c L64](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c#L64).
- Replace `s_channel_idx = 0U` with:
  ```c
  uint8_t  ch_idx;
  uint32_t ch_hz;
  if (sx1276_fhss_next_channel(&ch_idx, &ch_hz) != 0) { /* fault */ }
  sx1276_set_frequency_hz(ch_hz);
  s_channel_idx = ch_idx;
  ```
- Then LBT, then `sx1276_airtime_reserve(ch_idx, est_us)` (now consulting
  both accountants).
- Keep the fixed-channel path under a compile/cfg flag tied to
  `profile == BENCH_ONLY_FIXED_915`.
- Acceptance: 60 s continuous TX run with new profile shows energy on
  ≥50 distinct centers in the spectrum capture (falsification target #2,
  FCC notes §11.5).

**A5. Profile type + cfg validation**
- Where: [host_cfg.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c),
  [host_cfg_keys.h](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_keys.h).
- Add `CFG_KEY_REG_PROFILE` (u8) with enum
  `{BENCH_ONLY_FIXED_915=0, FCC_15_247_FHSS_50CH_BW250=1,
   FCC_15_247_DTS_BW500=2}`.
- Reject sets that violate profile floors:
  - `FHSS_50CH` requires `popcount(CFG_KEY_FHSS_CHANNEL_MASK) >= 50` and
    membership in the certified channel table.
  - `FHSS_50CH` requires modem BW == 250 kHz at init.
  - Bench profile is the only one that accepts a single-channel mask.
- Profile-aware power clamp: `Pconducted_max = tier_ceiling -
  max(0, antenna_gain_dBi - 6)`; tier_ceiling is +30 dBm for 50-channel
  FHSS but is further clamped by the existing +17 dBm hardware ceiling.

**A6. RX hop sync**
- Where: `radio/sx1276_rx.c` + `sx1276_fhss`.
- RX derives the same `permute(seed, epoch)` and retunes between RX
  windows. Use existing monotonic tick for epoch alignment; reserve a
  resync slot per epoch where RX falls back to a known seed slot if
  decode fails for `N` consecutive epochs.
- **Do not** introduce a fixed-channel rendezvous (FCC notes §15.1.1
  retraction). Resync stays inside the 50-channel sequence.

### Track B — Compliance observability (release-blocking)

**B1. `RFCO` URC** — emit on every TX result and every minute:
- active profile name,
- current epoch index,
- current hop index + frequency,
- per-channel occupancy histogram (rolling 10 s),
- `legal_dwell_max_us` in current window,
- `qos_used_us` in current 1 s window (separate field),
- active channel count, blacklist size,
- last power-clamp reason (`profile`, `antenna`, `hw_ceiling`),
- packet-airtime estimate of the last TX.

**B2. Stamp every bench artifact** with a runtime readback of `RegFrf`,
`RegModemConfig1/2/3`, `RegPaConfig`, `RegPaDac`, the active profile name,
and the `RFCO` snapshot. Replaces the source-comment stamp in
[lora_ping.c](../DESIGN-CONTROLLER/firmware/murata_l072/lora_ping.c)
(FCC notes §17.3 #4).

**B3. Update orchestrator scripts** in
`LifeTrac-v25/tools/` (e.g., `mixed_load_soak.ps1`) to require
`profile=FCC_15_247_FHSS_50CH_BW250` in the run header and to abort if the
RFCO snapshot says otherwise.

### Track C — Reliability + cert expansion (post-release)

**C1.** Master-broadcast quality-aware blacklist (one authoritative node,
authenticated hopset epoch updates with grace window — FCC notes §12.2 +
§17.3 #1). Re-uses existing LoRa link MIC/AEAD path.

**C2.** Per-channel PA back-off table in `sx1276_set_tx_power_dbm()`,
gated on Phase D OOB measurements (FCC notes §11.3.5).

**C3.** Passive SDR wideband monitor on Portenta X8 base-station as
advisory blacklist input only, never cross-device coordination
(FCC notes §11.3.7).

**C4.** Deterministic per-farm hop seed via `farm_id`/`node_id` cfg keys
(FCC notes §11.3.8).

**C5.** DTS BW500 profile as a second certified mode (only after Phase D
6 dB BW + PSD measurement passes).

---

## 3. Bench evidence gates (Phase D — promote `FHSS_50CH` to field-candidate)

These map 1:1 to FCC notes §17.4 and §11.5 falsification targets. None
should be skipped before any field test.

| Gate | What is measured | Pass criterion | Tool |
|---|---|---|---|
| D1. Hop proof | TX energy distribution over 60 s continuous TX | ≥50 distinct centers, equal-use within ±10 % per epoch | spectrum analyzer, RFCO histogram |
| D2. Occupied BW | 20 dB occupied BW at SF7/SF8 × BW250 × +17 dBm into 50 Ω | ≥250 kHz and ≤500 kHz → 10 s window; <250 kHz → fallback profile name `..._NARROW` with 20 s window | spectrum analyzer |
| D3. OOB mask | Emissions ≥20 dB below in-band in any 100 kHz outside 902-928 MHz at every channel (especially band edges) | pass at +17 dBm at every channel | spectrum analyzer |
| D4. Per-channel dwell | Max per-channel occupancy in any 10 s window during continuous TX soak | ≤400 ms / channel / 10 s | RFCO `legal_dwell_max_us` |
| D5. Packet airtime cap | Largest TX `pkt_toa_ms` over the run | < 400 ms − 20 ms guard | RFCO |
| D6. Profile lock | Host attempts to set `RegFrf` / arbitrary BW / mask < 50 ch in production build | rejected with explicit error code | cfg fuzz script |
| D7. RF exposure | Antenna gain, mounting, separation distance | MPE evaluation or categorical exclusion documented (§1.1307 / §2.1093) | desktop calc + datasheet |

Soak target: re-run the mixed-load shape from
[mixed_load_2026-05-19](../DESIGN-CONTROLLER/bench-evidence/mixed_load_2026-05-19/README.md)
(6 500 packets, ~13 min) under `FCC_15_247_FHSS_50CH_BW250` with all
RFCO fields captured.

---

## 4. Out-of-scope for this plan (explicit)

The following are deliberately deferred or dropped, with rationale traceable
to FCC notes §17.2:

- **Dynamic regime switching at runtime.** Verdict: defer indefinitely
  until ≥1 profile is field-certified.
- **§15.249 low-power production profile.** Verdict: retired
  (-1.25 dBm EIRP target unreachable on PA_BOOST clamp ≥+2 dBm).
- **Part 97 amateur fallback.** Verdict: retired (encrypted control +
  pecuniary use disqualified).
- **Fixed-frequency rendezvous channel.** Verdict: retired (cited
  §15.247(a)(1)(iii) carve-out applies to 2.4 GHz, not 902-928 MHz).
- **§15.23 home-built path as an OSE scaling/distribution strategy.**
  Verdict: retired (per-individual, ≤5 units, personal use).
- **External RF amplifiers / >+17 dBm SKUs.** Verdict: defer to a separate
  authorized hardware SKU effort.
- **Asymmetric per-direction PHY.** Verdict: replaced by antenna-geometry
  asymmetry (base-side higher gain) under the same symmetric PHY.
- **Multi-radio single-system base station.** Verdict: defer behind
  passive-SDR receive-only monitor.

---

## 5. TODO.md edits this plan implies

Roll forward into [TODO.md](../TODO.md):

1. **Promote** N-06 (currently post-launch, 8-channel quality-aware FHSS)
   to a **pre-launch prerequisite** named `FCC-FHSS-50CH` with the Track A
   sub-tasks A1..A6 broken out.
2. **Add** `FCC-EVID-D1..D7` gate items linked to §3 above.
3. **Note** in the S1.x section that all existing bench evidence is
   stamped `BENCH_ONLY_FIXED_915` and is **not** FCC field evidence.
4. **Add a separate item** for closing the
   [2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md](2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md)
   gap: the TX-power adapter must layer **inside** the hop scheduler
   (FCC notes §11.1.5 — every power step must still meet OOB mask).

---

## 6. Order of execution (concrete)

1. **A1** (packet-airtime invariant) — single-file change, lowest risk,
   highest near-term safety value.
2. **A2** (split airtime accountant) — enables compliance telemetry to
   start collecting evidence even while A3/A4 are in flight.
3. **B1** (RFCO URC) — wire telemetry now so A3/A4 can be measured.
4. **A3** (`sx1276_fhss` module) — new file, no behavior change to TX path
   until A4 lands.
5. **A5** (cfg / profile validation) — locks the surface so A4 cannot be
   bypassed by a host-side override.
6. **A4** (route TX through hop scheduler) — the actual behavior change.
7. **A6** (RX hop sync) — required before paired bench evidence.
8. **B2 + B3** (artifact stamping + orchestrator updates).
9. **Phase D gates** (§3) — promotes profile to field-candidate.
10. **Track C** items as separate plans, each gated on its own measurement.

---

## 7. Open questions (need answers before A3 lands)

1. **Final 50-channel center list.** Interim plan: 500 kHz spacing across
   902.5-927.5 MHz. Final list is gated on D2 (occupied BW) — confirms
   spacing ≥ measured 20 dB BW and that edge channels (902.5, 927.5) still
   meet D3.
2. **Epoch length.** Default proposal: 50 slots × `inter_packet_ms` such
   that one full hopset per ~5 s under typical telemetry load. Needs
   verification against control-loop jitter budget.
3. **Time source resilience.** Confirm monotonic tick wrap behavior on
   L072 and define the "drain windows on discontinuity" action
   (FCC notes §17.3 #2).
4. **Authentication of hopset updates (Track C1).** Confirm the existing
   LoRa MIC/AEAD path can be re-used at the L072 cfg layer, not just at
   the host.
5. **Antenna SKU lock.** Track A5 power-clamp needs a certified antenna
   gain in cfg. What SKU + gain do we commit to for first field-candidate?

---

## 8. Sign-off conditions

This plan is "done enough to start coding" when:

- [ ] Open questions 1, 2, 3 in §7 have provisional answers (5 can be a
  config default, 4 is Track C and doesn't gate Track A).
- [ ] §5 TODO.md edits are applied.
- [ ] The first commit is A1 (packet-airtime invariant) with its unit
  test.

Field-candidate status is reached when:

- [ ] Track A complete (A1..A6).
- [ ] Track B1 + B2 emitting full RFCO + stamped artifacts.
- [ ] All Phase D gates (D1..D7) pass with margin and are archived under
  `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/fcc_fhss_50ch_<date>/`.

---

## 9. References

- [2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md](2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md)
  — regulatory analysis; this plan is its execution arm.
- [2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md](2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md)
  — TX-power adapter design (must layer on top of A3/A4).
- [walk_power_full_2026-05-19/README.md](../DESIGN-CONTROLLER/bench-evidence/walk_power_full_2026-05-19/README.md)
  — full S1.4 sweep evidence (bench-only baseline).
- [mixed_load_2026-05-19/README.md](../DESIGN-CONTROLLER/bench-evidence/mixed_load_2026-05-19/README.md)
  — 10-min soak evidence (bench-only baseline; shape to be re-run under
  the new profile).
- [sx1276_tx.c#L64](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c#L64)
  — the canonical compliance gap symbol.
- [sx1276_airtime.c#L11-L13](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c#L11-L13)
  — current QoS accountant constants.
- [host_cfg.c#L90-L94](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg.c#L90-L94)
  / [host_cfg_keys.h#L10-L23](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_cfg_keys.h#L10-L23)
  — existing FHSS scaffolding.

---

## 10. Copilot Review and Critique

This staged implementation plan serves as an excellent technical playbook for closing the compliance gap while preserving functional safety. By attacking the invariants first (Time-on-Air caps and dwell accumulation limits in Tracks A1/A2) *before* building the hop scheduler in A3/A4, you guarantee that even a malfunctioning hop sequencer cannot blast RF energy illegally on a single channel. 

### Key Strengths of the Plan
1. **Separation of Concerns:** Specifically separating the `QoS fairness gate` from the `FCC legal dwell accountant` is crucial. The FCC doesn't care if one channel is prioritized as long as none exceed the 400ms rolling ceiling and the pseudo-random average is maintained over the long term. 
2. **Defensive Config Validation:** Track A5 stops upstream bugs dead. If the central controller tries to negotiate an illegal 49-channel FHSS profile, the radio physically rejects the command. This protects the hardware configuration from logic errors up the stack.
3. **Evidence-Driven Promotion:** Forcing the firmware to explicitly emit `profile=FCC_15_247_FHSS_50CH_BW250` and stamping all artifacts ensures test logs are strictly tied to known firmware behaviors, avoiding future auditing ambiguities. 

### Suggestions for Improvement / Refinement
* **Epoch Length & Telemetry Payload (A3/A6):** The proposed ~5s epoch length (50 slots) runs the risk of colliding with maximum Time-on-Air (ToA) payload bursts if the `inter_packet_ms` is too tight. If the tractor is occasionally allowed to transmit large 255-byte bursts under SF7/BW250 (which take significant ms overhead), the scheduler must gracefully absorb or reject these to avoid corrupting the epoch alignment. A rigid time-slice scheduler might be simpler to sync than a pure packet-driven counter. 
* **PLL Lock Penalties in A4:** When rewriting `s_channel_idx`, inserting `sx1276_set_frequency_hz()` will invoke a PLL lock wait. Ensure this retune happens synchronously in the baseband cycle, well *before* the hardware initiates the CAD (Channel Activity Detection) or TX start-up sequence, to prevent clipping preamble symbols.
* **Boot/Sync State (A6):** Add an explicit "Scanning" state to Track A6. If the RX node boots *after* the TX node has begun its 50-channel sequence, it needs an efficient way to catch up. A wide-window scan or pausing on a single channel until a valid packet is intercepted (and extracting the epoch index from its header) will be necessary.

*Signed:* Copilot, v2.0 (Implementation Plan Review)

---

## 11. Copilot Final Addendum (Implementation Readiness)

This plan is strong and implementation-ready, but a few fail-closed details
will materially reduce both compliance and integration risk during Track A.

### Additional recommendations

1. **Fail closed on any FHSS state fault.**
  If hop index, epoch state, or timebase continuity is invalid, block TX with
  an explicit error code. Do not silently fall back to fixed-channel behavior
  in any production profile.

2. **Lock deterministic hop behavior with test vectors.**
  Add fixed-seed golden vectors for `sx1276_fhss` permutation output
  (first N channels per epoch) so firmware/toolchain changes cannot alter the
  hop sequence without a visible test failure.

3. **Define exact dwell-window edge semantics.**
  Specify whether the 10 s legal window is `[t-10000, t)` or `[t-9999, t]`
  and apply that same boundary rule in firmware, RFCO telemetry, and bench
  scripts. This removes off-by-one disagreements during evidence review.

4. **Add a two-phase config commit for profile transitions.**
  Stage, validate, then activate profile changes atomically. If any validation
  fails (mask size, BW mismatch, antenna constraints), keep the prior profile
  active and report the reject reason.

5. **Make evidence provenance machine-verifiable.**
  Include firmware git SHA, build timestamp, active profile enum, and RFCO
  schema version in each benchmark artifact header. This prevents ambiguous
  replay or stale-log attribution during audits.

6. **Add a pre-field go/no-go gate after D1-D7.**
  Require one scripted summary report that fails if any of the following are
  missing: hop histogram, legal dwell max, occupied-BW capture, OOB sweep,
  profile lock rejection logs, and exposure worksheet.

### Final assessment

The plan has the right architecture and sequencing. If the six controls above
are added, it will not only satisfy implementation needs but also create a
clear, auditable chain from runtime behavior to compliance evidence.

*Signed:* Copilot, v2.1 (Final Plan Addendum)

---

## 12. Copilot Review Addendum v2.2

This plan is ready to turn into implementation tickets. The architecture is
right: fail closed, make 50-channel FHSS a first-class profile, separate QoS
airtime from legal dwell, and make evidence generation part of the firmware
rather than an afterthought. My additional review is focused on details that
could otherwise create subtle compliance or sync failures during Track A.

### 12.1 Highest-value refinements before coding

1. **Make the 50-channel table exact and generated.** The interim phrase
  "500 kHz spacing across 902.5-927.5 MHz" is easy to misread as an inclusive
  list, which would produce 51 centers at 500 kHz spacing. Define the table by
  formula plus a static assert, for example `center_hz = 902750000 + 500000*i`
  for `i=0..49`, then finalize the start/end centers after occupied-BW and
  edge-emission measurements. The important thing is that the source code has
  exactly one `FHSS_CHANNEL_COUNT == 50` definition and the compiler proves it.

2. **Do A1 at both config time and TX time.** A modem-config setter can reject
  illegal SF/BW/CR combinations, but packet airtime also depends on payload
  length, header mode, CRC, preamble, and any future safety-burst framing. Add
  a pre-TX invariant in `sx1276_tx_begin()` as well: estimate the exact frame
  being sent, reject it if `toa_us > dwell_cap_us - guard_us`, then reserve
  dwell using that same estimate. This prevents a legal modem profile from
  carrying an illegal large packet.

3. **Define what happens when LBT blocks a hop.** If LBT repeatedly blocks the
  same channel and the scheduler simply advances, actual transmitted packets
  may become biased away from blocked channels. That may be acceptable as local
  intelligent avoidance only if it is reflected as an active-hopset decision
  with the legal channel floor preserved. Track A should therefore log blocked
  hop attempts separately from transmitted hop counts, and Track C blacklisting
  should be the only mechanism that removes a channel from the active set.

4. **Put profile/epoch metadata where resync can use it.** RX scanning will be
  much easier if every packet carries a small authenticated header containing
  profile id, hop epoch, hop index or slot, and RFCO schema version. It does
  not need to reveal sensitive command content, but the receiver needs enough
  metadata to recover timing after reboot or packet loss without relying on a
  fixed non-hopping rendezvous channel.

5. **Reserve dwell pessimistically and reconcile after TX completion.** A
  failed transmit attempt that starts RF emission still consumes dwell. The
  safest flow is: compute estimated airtime, reject if illegal, reserve the
  estimate before TX start, and on TX-done record actual or estimated-on-air
  time in telemetry. Do not roll back legal dwell merely because an ACK was not
  received.

6. **Add explicit channel-table and permutation test vectors to the host side.**
  The L072 and any X8/Python tooling should agree on the same channel list,
  permutation seed, epoch math, and dwell-window boundary semantics. Golden
  vectors should cover first channel, last channel, epoch rollover, mask
  rejection, and a deterministic known-seed permutation.

### 12.2 Extra evidence gates I would add

The D1-D7 gates are good. I would add three more before calling this
field-candidate:

| Gate | What is measured | Pass criterion |
|---|---|---|
| D8. Two-node sync torture | RX boots late, reboots mid-run, misses several epochs, and sees burst packet loss | RX reacquires without a fixed channel and without host intervention |
| D9. LBT bias stress | Inject interference on a subset of channels during continuous traffic | Legal dwell remains below cap; RFCO separates blocked attempts from actual TX counts; active set never drops below 50 without an explicit fault |
| D10. Power/antenna clamp fuzz | Try all profile, antenna-gain, and TX-power combinations through host cfg | Production profile clamps or rejects every illegal combination and reports the reason |

### 12.3 My recommended first coding slice

The first code slice should be small and testable, but it should establish the
shape of the final system:

1. Add the regulatory profile enum and readback string, even if only
  `BENCH_ONLY_FIXED_915` is active at first.
2. Add `sx1276_fhss` with the generated 50-channel table and golden-vector
  tests, but do not route TX through it yet.
3. Add pre-TX packet-airtime rejection using the exact payload length.
4. Add a stub RFCO snapshot that reports profile, configured channel count,
  selected channel, estimated airtime, and legal-dwell placeholder fields.

That slice gives the project immediate compile-time and log-time structure
without changing live RF behavior until the team is ready for the A4 retune
change. Then A4 becomes a narrow behavior switch instead of a giant first
landing.

### 12.4 Final assessment

The plan should move forward. The 50-channel FHSS approach is still the right
implementation target, and this document is now specific enough to prevent most
of the earlier regulatory ambiguity from leaking into code. The remaining risk
is not the FCC theory; it is ordinary embedded-system drift: one off-by-one in
the channel table, one fallback to fixed channel, one payload path that bypasses
the airtime cap, or one evidence script that labels QoS airtime as legal dwell.
If Track A treats those as first-order test cases, the implementation will be
auditable and much harder to accidentally misuse.

*Signed:* GitHub Copilot, Implementation Plan Review v2.2 (2026-05-19)

---

## 13. Copilot Review Addendum v2.3

After re-reading the full plan plus the prior v2.0/v2.1/v2.2 addenda, the
architecture is sound and most of the obvious foot-guns are already named.
This addendum focuses on a small number of things the prior reviews did not
fully cover and that I would not want to discover late in Track A.

### 13.1 Track-A intra-ordering hazard

The order in §6 is correct overall, but the seam between A5 (cfg + profile
enum) and A4 (route TX through hop scheduler) needs an explicit guard:
**A5 must NOT accept `CFG_KEY_REG_PROFILE == FCC_15_247_FHSS_50CH_BW250`
until A4 has landed.** Otherwise there is a real window in which the host
config layer happily reports "we are in the 50-channel FHSS profile" while
[sx1276_tx.c L64](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c#L64)
still hardcodes `s_channel_idx = 0U`. That state is worse than the current
single-channel bench mode because it produces evidence files stamped
`FCC_15_247_FHSS_50CH_BW250` from a radio that demonstrably does not hop.

Concrete fix: make A5 gate the production profile on a compile-time flag
(e.g. `LIFETRAC_FHSS_TX_ROUTED`) that is defined only by A4. Until A4
lands, A5 only accepts `BENCH_ONLY_FIXED_915`.

### 13.2 Channel-table generator is the safest single artifact to land

§7 question 1 lists "Final 50-channel center list" as an open question. The
v2.2 addendum already says generate-by-formula. I want to push that further:
**ship the generator and the static-assert before A3 starts**, even if the
exact start frequency is still TBD. Two reasons:

1. It locks the §15.247(a)(1) spacing rule into the build: a one-line
  `_Static_assert(FHSS_CHANNEL_SPACING_HZ >= 25000)` plus a later
  `_Static_assert(FHSS_CHANNEL_SPACING_HZ >= measured_20dB_BW_hz)`
  prevents a non-obvious illegal channel plan from compiling.
2. It gives the host side (X8 Python tooling, evidence scripts) a single
  source of truth to import, which matches the §12.1 #6 cross-side
  golden-vector recommendation but is cheaper to land.

Suggested interim: `center_hz = 902_750_000 + 500_000 * i, i = 0..49`,
range 902.750-927.250 MHz, ≥250 kHz from each band edge. Anything tighter
must wait on D2/D3 evidence.

### 13.3 RFCO bandwidth budget needs to be sized

Track B1 enumerates a generous set of fields. A naive implementation that
emits the full 50-bucket per-channel histogram every TX result will compete
with control telemetry over the X8↔L072 UART and through the LoRa link
itself. Two pragmatic shapes:

* **Per-TX RFCO:** minimal — `{profile_id, epoch, hop_idx, freq_hz,
  pkt_toa_us, dwell_used_us_for_this_channel}`. Fits in one line.
* **Per-minute RFCO_SUMMARY:** the heavy snapshot — full histogram,
  `legal_dwell_max_us` per channel, blacklist size, last clamp reason.

Defining this split now (a single sentence in §1 / B1) prevents the
"telemetry storm" anti-pattern from being implemented and then ripped out.

### 13.4 A6 (RX hop sync) is the biggest unmentioned schedule risk

A4 is small once A3 and A5 are in place. **A6 is not.** Recovering hop
sync without a fixed rendezvous channel — which the plan correctly
forbids — generally means one of: (a) every packet carries an
authenticated `(epoch, hop_idx)` header (v2.2 §12.1 #4, agreed), plus
(b) on cold start / extended loss, the RX performs a wideband scan
across the active set with a known preamble timeout per channel.

Worst-case scan time at SF7/BW250 with a typical preamble + sync timeout
on 50 channels is non-trivial; it should be measured, not estimated, as
part of D8 (v2.2). I would explicitly add to §7 open questions:

> **Q6. RX cold-start scan budget.** What is the measured worst-case time
> for an RX node to reacquire hop sync after a full reboot under a
> production load? Target ≤ 5 s; if measured > 30 s, A6 needs a redesign
> (e.g., adaptive scan dwell, super-frame beacon slot within the hopset),
> not just a bug fix.

### 13.5 A1 needs a host-side error contract, not just a reject

A1 (packet-airtime invariant) is described as "reject tuples where
`pkt_toa_ms + guard_ms > dwell_cap_ms`." That is correct for the radio
side, but the host runtime currently has no defined behavior for "the
L072 refused my TX." Without a contract, the host either drops the
packet silently (bad — lost commands) or retries forever (worse — DoS).

Minimum spec to add alongside A1:

* New URC/error code `__AIRTIME_INVARIANT_REJECT__` with the offending
  `(sf, bw, payload_bytes, computed_toa_us, cap_us)`.
* Host policy: log + drop, never auto-retry the same oversized payload.
* A regression: the H7 / X8 host stack must never construct a payload
  whose worst-case `toa_us` could exceed the cap under the active
  profile. Add a host-side mirror of the airtime calc keyed off the
  reported `profile_id`.

### 13.6 Track-B QoS-vs-legal field naming is a known landmine

The v2.2 closing paragraph already calls out "evidence script that labels
QoS airtime as legal dwell" as the dominant remaining failure mode. To
operationalize that, I'd require the RFCO schema to use unambiguous field
names that make it linter-checkable:

* `qos_used_us_1s` (fairness)
* `legal_dwell_used_us_10s` (FCC)
* `legal_dwell_used_us_20s` (FCC narrowband fallback profile)

…and have the evidence-stamping tool in B2 refuse to write any file that
contains a field named `airtime_us` or `dwell_us` without one of the three
qualifiers. This is a one-line guard that prevents an entire class of
auditing-time disasters.

### 13.7 Bench environment caveat (D1-D10)

D1 (50-distinct-center histogram) and D9 (LBT bias stress) will both be
sensitive to the **RF environment** they are captured in. In an open lab
with ambient 915 MHz traffic, LBT will skew the histogram even on a
correctly-implemented hop scheduler — which would look like a bug in
A3/A4. The plan should specify:

* D1 baseline runs in a shielded / quieted environment (conducted into
  a 50 Ω load with a directional coupler, or in an RF anechoic
  enclosure).
* D9 runs in the same environment with a calibrated interferer; ambient
  cannot substitute for a controlled jammer.

Without that note, the first bench session will likely produce ambiguous
D1 evidence that triggers an unnecessary code investigation.

### 13.8 What I would NOT change

* The Track A → B → D ordering. It is correct.
* The retirement list in §4. None of those should reopen now.
* The "compile-time profile lock" approach in A5. It is the right
  pattern even though §13.1 above wants the gate tightened.
* The choice to keep the existing 1 s QoS fairness gate alongside the
  new legal-dwell accountant. Removing it would mask QoS bugs as
  compliance bugs and vice versa.

### 13.9 Final assessment

This plan is ready to drive implementation tickets. The remaining open
points are tightening seams, not redesigning anything. Land the channel-
table generator and A1 first (per v2.2 §12.3), keep A5 from accepting the
production profile until A4 routes TX through `sx1276_fhss`, size the
RFCO budget before B1 lands, and treat A6 as the schedule pole.

If the plan ships approximately as written with the v2.0/v2.1/v2.2/v2.3
refinements folded in, the resulting build will be the first LifeTrac
firmware that can honestly carry the `FCC_15_247_FHSS_50CH_BW250` label.

*Signed:* **GitHub Copilot, Implementation Plan Review v2.3 (2026-05-19)
— incremental addendum to v2.0/v2.1/v2.2**

---

## 14. Consolidated v3.0 plan deltas (authoritative)

This section folds the v2.0/v2.1/v2.2/v2.3 review chain into a single
authoritative delta against §1-§9. Where §14 conflicts with §1-§9, §14
wins. §10-§13 remain unedited as the review trail.

### 14.1 Delta map (suggestion → plan edit)

| # | Source | Plan edit |
|---|---|---|
| 1 | §13.1 | **A5** compile-time gates `FCC_15_247_FHSS_50CH_BW250` on `LIFETRAC_FHSS_TX_ROUTED`, a flag defined only by A4. Until A4 lands, A5 accepts only `BENCH_ONLY_FIXED_915`. |
| 2 | §12.1 #2 | **A1 splits** into A1a (modem-config-time SF/BW/CR reject) and A1b (`sx1276_tx_begin()` pre-TX reject keyed on actual frame: payload, header mode, CRC, preamble, any safety-burst framing). A1b uses the exact same airtime calc as A2 reservation. |
| 3 | §12.1 #5 | **A2** reserves dwell pessimistically before TX-start using the A1b estimate, reconciles on TX-done; **never** rolls back legal dwell on ACK timeout or NACK. |
| 4 | §12.1 #3, §13.6 | **A3** API adds `sx1276_fhss_record_lbt_block(idx)`. Invariant: `active_count` only changes via the Track-C blacklist API, never via the LBT path. RFCO reports `blocked_attempts` separately from `tx_count` per channel. |
| 5 | §11 #1 | **A3** fails closed on any invalid epoch / hop / timebase state: returns error, TX caller must abort. The bench profile is the only path where fixed-channel TX is reachable. No silent fallback. |
| 6 | §12.1 #1, §13.2 | **New first-slice item: `sx1276_fhss_chantab`** — generator-based 50-channel table with `_Static_assert(FHSS_CHANNEL_COUNT == 50)`, `_Static_assert(FHSS_CHANNEL_SPACING_HZ >= 25000)`, and a stub assert slot for `>= measured_20dB_BW_hz` (filled after D2). Interim formula: `center_hz = 902_750_000 + 500_000 * i, i = 0..49`. Lands **before** A3. |
| 7 | §11 #2, §12.1 #6 | **A3** acceptance adds fixed-seed golden vectors (first-N-channels-per-epoch) checked in under `bench/host_proto/`; the same generator + permutation is imported by X8-side Python tooling so host and L072 agree by construction. |
| 8 | §11 #3 | **A2 + B1 + D4** all use the dwell-window convention `[t - WINDOW_MS, t)` (half-open, current sample excluded from prior window). Single sentence in A2, mirrored in RFCO schema and bench-script post-processing. |
| 9 | §11 #4 | **A5** adds two-phase commit: `cfg_profile_stage(p)` → validate (mask popcount, BW, antenna constraints) → `cfg_profile_activate()`. Failed activate keeps the prior profile live and reports a structured reject reason. |
| 10 | §13.5 | **A1** adds host-facing error contract: new URC `__AIRTIME_INVARIANT_REJECT__` carrying `(sf, bw, payload, computed_toa_us, cap_us)`. Host policy = log + drop, **no auto-retry**. Host stack carries a mirror calc keyed off `profile_id` so an oversized payload never reaches the L072 in the first place. |
| 11 | §13.6 | **B2** stamping tool refuses to write any artifact containing a field named `airtime_us` or `dwell_us` without one of three qualifiers: `qos_used_us_1s`, `legal_dwell_used_us_10s`, `legal_dwell_used_us_20s`. One-line linter, blocks an entire failure class. |
| 12 | §13.3 | **B1 splits** into two URCs:<br>• **`RFCO`** (per-TX, single UART line): `{profile_id, epoch, hop_idx, freq_hz, pkt_toa_us, legal_dwell_used_us_for_ch}`.<br>• **`RFCO_SUMMARY`** (per-minute): full 50-bucket histogram, per-channel `legal_dwell_max_us`, `blocked_attempts` histogram, `active_count`, `blacklist_size`, last clamp reason, RFCO schema version. |
| 13 | §12.1 #4, §13.4, §10 | **A6** packet header carries `{profile_id, epoch, hop_idx, schema_ver}` authenticated via existing MIC. Cold-start path = explicit **Scanning** state: wideband scan-with-preamble-timeout across the 50-channel active set; **no fixed rendezvous channel** (FCC notes §15.1.1). |
| 14 | §11 #5 | **B2** artifact header required fields: firmware git SHA, build timestamp (UTC), active profile enum + string, RFCO schema version. Stamping tool fails the run if any are missing. |
| 15 | §11 #6, §12.2, §13.7 | **§3** gains three gates: **D8** two-node sync torture (RX boots late, reboots, misses epochs — reacquires without host intervention); **D9** LBT bias stress (calibrated interferer; RFCO separates blocked attempts from TX; active set never silently shrinks); **D10** power/antenna clamp fuzz (every illegal profile×gain×power combination is rejected with reason). **D1 + D9 environment constraint:** captured conducted into 50 Ω with a directional coupler OR inside an RF-quiet enclosure — ambient open-lab traffic is not acceptable evidence. **D-Gate report:** one scripted summary that fails if any of D1-D10 artifacts or required header fields are missing. |
| 16 | §10 | **A4** code sketch annotated: `sx1276_set_frequency_hz()` must complete **and** PLL settle (document the settle budget in microseconds, measured) **before** LBT/CAD/TX-start, otherwise preamble symbols clip. |
| 17 | §10 | **§7 Q2** reworded: choose strict time-slice vs packet-driven epoch counter; specify large-payload absorption behavior at epoch boundary (large frame may not straddle a slot boundary; defer or reject). |
| 18 | §13.4 | **§7 Q6 (new):** measured worst-case RX cold-start hop-sync reacquire time under production load. Target ≤ 5 s. If measured > 30 s, A6 needs redesign (adaptive scan dwell or super-frame beacon slot within the hopset), not a bug fix. |

### 14.2 Replaces §6 — execution order (v3.0)

1. **`sx1276_fhss_chantab`** generator + static asserts. No behavior change.
2. **Profile enum + readback string** in cfg layer (only `BENCH_ONLY_FIXED_915`
   active; `LIFETRAC_FHSS_TX_ROUTED` undefined → A5 rejects production
   profile per delta #1).
3. **A1a** (config-time airtime invariant) + URC `__AIRTIME_INVARIANT_REJECT__`.
4. **A1b** (pre-TX, payload-aware) + host-side mirror calc.
5. **`sx1276_fhss` skeleton** with golden vectors (delta #7). No TX routing yet.
6. **A2** split accountant with `[t-WINDOW_MS, t)` semantics and pessimistic
   reserve / no-rollback contract.
7. **B1 per-TX `RFCO` URC** (placeholder for histogram fields).
8. **A5** cfg validation + two-phase commit. Still gated to bench profile.
9. **A4** routes TX through hop scheduler. Defines `LIFETRAC_FHSS_TX_ROUTED`.
   A5 now accepts `FCC_15_247_FHSS_50CH_BW250`.
10. **A6** RX hop sync + Scanning state + authenticated packet header.
11. **B1 `RFCO_SUMMARY`** + **B2** artifact stamping + naming linter + **B3**
    orchestrator profile gate.
12. **Phase D** D1-D10 in shielded / coupled environment, with the scripted
    **D-Gate** report.
13. **Track C** items as separate plans, each gated on its own measurement.

### 14.3 Replaces §7 open questions

1. **Final 50-channel center list** — interim per delta #6; final gated on D2.
2. **Epoch model** — strict time-slice vs packet-driven counter; large-frame
   straddling behavior (delta #17).
3. **L072 monotonic-tick wrap + discontinuity drain action** (FCC notes
   §17.3 #2).
4. **Hopset-update auth path (Track C1)** — re-use LoRa MIC/AEAD at L072 cfg
   layer? Does not gate Track A.
5. **Antenna SKU + gain commitment** for first field-candidate (A5 clamp).
6. **(NEW)** **RX cold-start scan budget** — target ≤ 5 s; >30 s ⇒ A6
   redesign (delta #18).

### 14.4 Replaces §5 — TODO.md edits this plan implies

1. **Promote** the 50-channel FHSS work to a **pre-launch prerequisite**
   stage `FCC-FHSS-50CH` with sub-tasks for the v3.0 execution order
   (delta #6 first, then A1a, A1b, A3 skeleton, A2, B1, A5, A4, A6).
2. **Add** `FCC-EVID-D1..D10` gate items linked to §3 (as extended by
   delta #15) plus the scripted **D-Gate** summary item.
3. **Note** in the S1.x section that all existing bench evidence is stamped
   `BENCH_ONLY_FIXED_915` and is **not** FCC field evidence.
4. **Add** an item closing the
   [2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md](2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md)
   gap: the TX-power adapter and safety-burst path must layer **inside**
   the hop scheduler (FCC notes §11.1.5).
5. **Add** `FCC-FHSS-CHANTAB` as the very first sub-task of `FCC-FHSS-50CH`
   (delta #6).
6. **Add** the **D-Gate** scripted summary as its own gate item, separate
   from D1-D10.

### 14.5 Sign-off conditions (replaces §8)

"Done enough to start coding" when:

- [ ] Q1, Q2, Q3 in §14.3 have provisional answers (Q4 is Track C; Q5 can
  default; Q6 is measured during D8, not before A6).
- [ ] §14.4 TODO.md edits applied.
- [ ] First commit is `sx1276_fhss_chantab` generator with static asserts.
- [ ] Second commit is A1a + the `__AIRTIME_INVARIANT_REJECT__` URC.

Field-candidate when:

- [ ] §14.2 steps 1-11 complete.
- [ ] D1-D10 all archived under
  `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/fcc_fhss_50ch_<date>/`
  with the **D-Gate** scripted summary passing.
- [ ] No bench artifact contains an unqualified `airtime_us` or `dwell_us`
  field (delta #11 linter).

*Signed:* **GitHub Copilot, Consolidated Plan v3.0 (2026-05-19)
— folds v2.0/v2.1/v2.2/v2.3 review chain into a single authoritative delta**



---

## 15. v3.1 Bench-hardware-reality + lab-equipment-waiver addendum (2026-05-19)

This addendum folds in four scope clarifications the user delivered after
v3.0 landed. Where §15 conflicts with §1-§14, **§15 wins** for the bench
hardware topology, the lab-equipment waivers, and the E-STOP signal
path. §10-§14 remain unedited as the review trail.

### 15.1 Bench hardware truth (replaces "no L072 hardware available" assumption)

The production radio module is **already on the bench, twice over.**

- 2× Portenta X8 + Max Carrier. Each Max Carrier integrates the Murata
  CMWX1ZZABZ-078 (STM32L072 + SX1276) as its onboard LoRa radio.
- Board 1 serial `2D0A1209DABC240B` = **tractor-side** radio.
- Board 2 serial `2E2C1209DABC240B` = **base-station-side** radio.
- 1× MKR WAN 1310 (Murata CMWX1ZZABZ-091) = optional bring-up spare /
  hammer-firmware host for the S-HW.3 interferer substitute.

**Implication for the plan:** there is no "L072 reattach" step
anywhere in Tracks A/B/C. The same firmware ships to both Max Carrier
units; profile selection at boot picks base-vs-tractor role. The
existing W1-10b 100/100 RX match (2026-05-12) is end-to-end production
hardware evidence, not a development-board proxy.

### 15.2 Permanent lab-equipment waivers (cascades to §3 gates)

The user has confirmed that the following equipment will **not** be
available for this product line — ever, not "later":

| Equipment | Waiver scope | Software substitute that ships in its place |
|---|---|---|
| Step attenuator | S-HW.2 PERMANENTLY SKIPPED | Range-walk + natural path loss. Bench artifact axis flips from "PER vs commanded attenuation step" to **"PER vs measured-RSSI bin"** — the existing PERTX URC's RSSI field is the discriminator. |
| Spectrum analyzer | S-HW.4 PERMANENTLY SKIPPED | FCC §15.247 conducted-emission evidence ships under (a) firmware-side `RegPaConfig` cap declared in the artifact header and (b) datasheet conducted-path table for the SX1276. |
| Calibrated antenna + anechoic enclosure | S-HW.4 PERMANENTLY SKIPPED | Antenna gain read from datasheet. EIRP computed at report-time as `EIRP_dBm = conducted_dBm + antenna_dBi − cable_loss_dB`; the three operands are stamped in every artifact header. |
| TCB / part-15 lab | S-HW.4 PERMANENTLY SKIPPED | Categorical exclusion under §1.1307(b) / §2.1093 documented in the artifact header (D7 substitute). |

**Cascade to §3 Phase-D evidence gates** — five of the seven gates
listed in §3 (and the three added by delta #15 in §14.1) lose their
direct measurement path. Each ships a software substitute and is
flagged **⚫ WAIVED** in TODO.md Phase-D block:

| §3 gate | New status | Substitute evidence (lands in firmware/orchestrator, not on a lab bench) |
|---|---|---|
| **D1** Hop proof | ⚫ WAIVED | `RFCO_SUMMARY.per_channel_hop_count[50]` (B1-SUMMARY-b). Equal-use within ±10 % per epoch is computed from the histogram, not from spectrum-analyzer integration. |
| **D2** Occupied BW | ⚫ WAIVED | SX1276 datasheet table for `BW250` (≥250 kHz inherent). Profile name stays `FCC_15_247_FHSS_50CH_BW250`. No `_NARROW` fallback path lands until a real SA run exists. |
| **D3** OOB mask | ⚫ WAIVED | SX1276 datasheet OOB mask declaration in artifact header. PA cap (`RegPaConfig` ≤ +17 dBm) enforced at runtime by S0.9 TX-power layer. |
| **D4** Per-channel dwell | ✅ IN SCOPE | `RFCO_SUMMARY.per_channel_dwell_max_ms[50]` already does this. No equipment needed. |
| **D5** Packet airtime cap | ✅ IN SCOPE | `RFCO_PERTX.pkt_toa_us_le` already does this. No equipment needed. |
| **D6** Profile lock | ✅ IN SCOPE | cfg-fuzz Python script; no equipment needed. |
| **D7** RF exposure | ⚫ WAIVED | Categorical exclusion §1.1307(b) / §2.1093 declared in artifact header. Antenna gain + separation distance numbers from datasheet + mechanical assembly. |
| **D8** Two-node sync torture | ✅ IN SCOPE | Two Max Carriers + W1-10b harness pattern. No equipment needed. |
| **D9** LBT bias stress | ⚫ WAIVED | Second Max Carrier flashed with hammer firmware = substitute interferer. `RFCO_SUMMARY.blocked_attempts_by_reason[8]` is the discriminator instead of "ambient cannot substitute for calibrated jammer." |
| **D10** Power/antenna clamp fuzz | ✅ IN SCOPE | cfg-fuzz Python script; no equipment needed. |
| **D-Gate** scripted summary | ✅ IN SCOPE | Aggregates the surviving D4/D5/D6/D8/D10 + waiver-declaration headers. |

**§13.7 bench-environment caveat is hereby superseded** for D1 + D9
specifically: since neither gate runs on a spectrum analyzer, the
"shielded enclosure / directional coupler / calibrated interferer"
constraint no longer applies. It still applies in principle to any
future SA-based re-measurement, but that re-measurement is not on the
critical path. §13.8's "do not change the §4 retirement list" still
holds — none of the retired profiles are reopened by this waiver.

### 15.3 E-STOP signal path (definitive)

S-HW.5 was previously written as if it required a physical mushroom
button. The user has redefined the path:

- **Tractor-side E-STOP:** Arduino Opta digital-input pin flip. Routed
  to the base-station Max Carrier as a host frame; on the L072 side
  this is an existing `host_uart` command, not a new URC.
- **Operator-side E-STOP:** keyboard / joystick button on the web
  remote-control UI. Generates an `ESTOP_REQ` host frame on the
  browser → backend → L072 path. Rate-limited ≤5 Hz. Global keydown
  handler (must fire even when the joystick widget does not have
  focus).
- **Bench substitute:** a tactile switch shorting Opta input I1 to GND.
  No mushroom hardware, no separate E-STOP cert path.

**Implication for the plan:** there is no new safety-burst PHY work
beyond what
[2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md](2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md)
already specifies. The "safety burst layered inside the hop scheduler"
note from §14.4 item 4 stands unchanged.

### 15.4 FCC-B1-SUMMARY landing trail (concretizes §14.2 step 11)

§14.2 step 11 lumps `RFCO_SUMMARY` + B2 + B3 into one bullet. The
implementation has decomposed it further (see TODO.md FCC track):

- **Wire-layout design doc:**
  [2026-05-19_FCC_B1_SUMMARY_Wire_Layout_Design_Copilot_v1_0.md](2026-05-19_FCC_B1_SUMMARY_Wire_Layout_Design_Copilot_v1_0.md)
  — 191 B single-frame payload, snapshot-and-reset deltas, main-loop
  polling cadence at 60 000 ms, `HOST_TYPE_RFCO_SUMMARY_URC = 0xC4U`.
- **FCC-B1-SUMMARY-a (landed 2026-05-19):** declaration-only header
  [include/host_rfco_summary.h](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_rfco_summary.h)
  + 0xC4 type-code in
  [include/host_types.h](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h).
  Four `_Static_assert`s pin the wire layout. No emitter, no pack
  helper, no main-loop integration. Reversible.
- **FCC-B1-SUMMARY-b (next):** pure `host_rfco_summary_pack()` + three
  sidecar counter TUs (per-channel hop count, per-channel dwell-max
  ms, blocked-attempts-by-reason) + ≥10 byte-by-byte wire-vector
  cases in `bench/host_proto/rfco_summary.c`.
- **FCC-B1-SUMMARY-c:** emit wrapper + main-loop 60 000 ms cadence +
  emit-timing bench test.
- **FCC-B2-b (unblocked by Q7 resolution):** the first SUMMARY URC's
  `schema_ver` byte at run start is stamped into the artifact header
  alongside firmware_git_sha, build_timestamp_utc, profile_id,
  profile_str.
- **FCC-B3:** orchestrator profile gate + RFCO snapshot-mismatch
  abort.

### 15.5 Sign-off conditions (incremental delta on §14.5)

The "Done enough to start coding" gate in §14.5 is unchanged — coding
has already started. Field-candidate gate is **amended** as follows:

- Strike "D1-D10 all archived" and replace with **"D4 + D5 + D6 + D8 +
  D10 + D-Gate all archived; D1 + D2 + D3 + D7 + D9 each have the
  software-substitute artifact and the §15.2 waiver declaration in
  the artifact header."**
- Add: "First `RFCO_SUMMARY` URC of every run is captured in the
  artifact and its `schema_ver` byte matches the stamped header
  field" (B2-b cross-check).
- Add: "E-STOP path tested end-to-end (Opta pin-flip → base-station
  Max Carrier → ESTOP host frame on host UART) under nominal LBT
  load. Web-UI keydown-handler path tested with browser-only stub
  (no Opta) under the same load."

*Signed:* **GitHub Copilot, Bench-Hardware-Reality + Lab-Equipment-Waiver Addendum v3.1 (2026-05-19) — incremental addendum to v3.0**
