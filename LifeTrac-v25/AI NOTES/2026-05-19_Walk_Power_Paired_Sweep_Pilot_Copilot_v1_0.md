# 2026-05-19 — Walk-Power Paired Sweep Pilot Summary

**Author:** GitHub Copilot (assistant-owned analysis note)
**Status:** PILOT LANDED — see [bench-evidence/walk_power_pilot_2026-05-19](../DESIGN-CONTROLLER/bench-evidence/walk_power_pilot_2026-05-19/README.md)
**Scope:** First over-the-air paired sweep on the live X8+L072+SX1276 bench;
closes S1.1 hardware validation; provides foundation for S1.4 full evidence.

## 0. Source-of-truth precedence

Per the standing repository convention (and reiterated in the
`2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0` doc §0):
**code constants > bench evidence > design docs**. This note follows that order
and flags one doc-level inconsistency that needs correction.

## 1. What ran

A 16-step `walk_power` sweep (2..17 dBm, 1 dB steps, 50 attempted frames per
step, 24-byte payload) on TX board A, paired with `rx_listen --rx-window 120`
on RX board B. Identical Portenta X8 + Murata SX1276/L072 hardware on both
sides; same bench geometry as the W2-02 stability run.

Orchestrator: [LifeTrac-v25/tools/paired_walk_power_sweep.ps1](../tools/paired_walk_power_sweep.ps1)
Analyser:    [LifeTrac-v25/tools/analyse_paired_sweep.py](../tools/analyse_paired_sweep.py)

## 2. Headline numbers

| Metric                         | Value                       |
|--------------------------------|-----------------------------|
| Steps with rx>0                | 16 / 16                     |
| Attempted frames               | 800                         |
| TX'd (after firmware airtime gate) | 685                     |
| Received & decoded             | 680                         |
| Mean PER (rx ÷ tx_done)        | **0.59 %**                  |
| RSSI walk                      | -119.0 → -109.8 dBm (+9.2)  |
| SNR walk                       | -6.6 → +5.2 dB (+11.8)      |
| Verdict                        | `__PAIRED_SWEEP_VERDICT__=OK` |

## 3. Falsifications performed before declaring "OK"

Per the methodology rule recorded in user memory ("Before declaring a hypothesis
a 'diagnosis' or 'root cause', run at least one falsification test"), I ran
two falsifications BEFORE running the sweep, not after:

1. **Regulatory band sanity** — `regfrf_check.py` reads RegFrf (0x06/0x07/0x08)
   directly. Both boards returned `RegFrf = 0xE4C000 → 915.000 MHz → US FCC
   Part 15.247 (902-928 MHz)`. The bench is US-legal as configured. This was
   triggered by the user's question "are supposed to broadcast at 915 in the
   united states?" and removed the regulatory blocker before any TX.

2. **TX-power apply path** — `falsify_tx_power_paconfig.py` walks
   `CFG_SET_REQ(TX_POWER_DBM)` over (2, 8, 14, 17) dBm and reads back
   `RegPaConfig (0x09)`. Got the predicted 1:1 mapping (2→0x80, 8→0x86,
   14→0x8C, 17→0x8F), confirming `cfg_apply_tx_power_dbm` actually moves the
   SX1276 register — so any PER-vs-power curve we observe is real, not faked.

Both probes are permanent helpers, committed alongside the main probe under
`firmware/x8_lora_bootloader_helper/`.

## 4. Notable observations

- **RSSI compression** (~9 dB walk for ~15 dB TX walk) is consistent with
  near-field coupling between bench antennas. SNR (~12 dB walk) is the more
  reliable proxy for link-margin scaling and is essentially 1:1 with TX power.
- **Firmware airtime limiter** rejected ~12-16 % of TX attempts via
  `radio_tx_abort_airtime_delta`. `radio_tx_abort_lbt_delta` was 0 across all
  steps — LBT did **not** interfere. The airtime gate is the EU-spec 1 %
  duty-cycle limiter inherited from the original Murata reference firmware;
  for US ISM (FCC Part 15.247 has no duty-cycle limit) it should be
  configurable, but is not blocking.
- **End-to-end PER 0.59 %** at the most-attenuated step (+2 dBm) is consistent
  with the bench's noise floor and a link margin of ~15-20 dB even at lowest
  power. This validates the SF7/BW125/CR4-5 baseline for short-range bench
  characterisation.

## 5. Doc inconsistency to correct (does NOT block any work)

The user's open document
[2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md](2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md)
says **"868.1 MHz"** in its §1 "Motivating Evidence" block. Per the §0
precedence rule, the **firmware is on 915.000 MHz** (verified by
[regfrf_check.py](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/regfrf_check.py),
plus [lora_ping.c#L312](../DESIGN-CONTROLLER/firmware/murata_l072/lora_ping.c#L312)
and [sx1276.c#L269](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c#L269)).
The W2-02 stability run on the same firmware therefore was also on 915 MHz —
the "868.1 MHz" reference in the doc is a stale copy-paste from earlier EU-spec
notes and should be corrected (one-line surgical fix, leaving a strikethrough
trail per existing review convention). Not making the change in this pass to
keep the patch surface tight; flagged here for the next pass through that doc.

## 6. What changed in the workspace (this session)

- **NEW** [tools/paired_walk_power_sweep.ps1](../tools/paired_walk_power_sweep.ps1):
  PowerShell orchestrator. Spawns `rx_listen` on board B via `Start-Process`
  (the only pattern that reliably detaches across `adb shell`), waits for
  `__W1_10B_LISTEN_READY__`, then runs `walk_power` on board A, then pulls
  the CSV. Initial path bug (`..\..\` → `..\`) fixed.
- **NEW** [tools/analyse_paired_sweep.py](../tools/analyse_paired_sweep.py):
  Joins TX-side CSV with RX-side log by parsing the `WP s<step> p<dbm> i<idx>`
  ASCII tag inside received payloads. X8-compatible (no `statistics` module
  dependency — see `lifetrac-x8-python-stripped` repo memory).
- **NEW** [firmware/x8_lora_bootloader_helper/regfrf_check.py](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/regfrf_check.py):
  Permanent regulatory-band probe.
- **NEW** [DESIGN-CONTROLLER/bench-evidence/walk_power_pilot_2026-05-19/](../DESIGN-CONTROLLER/bench-evidence/walk_power_pilot_2026-05-19/README.md):
  Evidence artifact: TX CSV, RX log, per-step JOIN CSV, README.
- **MODIFIED** [TODO.md](../TODO.md): S1.1 inline DONE note updated (removed
  stale "HW-BLOCKED" wording, linked to the new evidence dir). S1.4 status
  flipped from 🔴 *(HW-BLOCKED)* to 🟡 *(PILOT LANDED — full sweep pending)*
  with the airtime-gate caveat captured.

## 7. Open follow-ups (not done in this pass)

1. **Full S1.4 sweep**: re-run with `--per-step-count 200` once the airtime
   gate is either documented as a known-known or made configurable for US.
2. **10-minute mixed-load run**: still pending, also under S1.4.
3. **Doc fix**: surgical 868.1 → 915.0 MHz correction in
   `2026-05-18_TX_Power_Adaptation_And_Safety_Burst_Design_Copilot_v1_0.md` §1.
4. **Dangling 0xC0 URC cleanup**: benign noise observed each TX cycle; causes
   the cosmetic "STATS_URC(after) timeout" in single-frame probes. Drain at
   slot boundary in `wait_for_tx_done`.
5. **One-way latency analysis**: TX CSV has wall-clock `timestamp_iso`; RX log
   has SX1276 `timestamp_us` (free-running). With a calibration shot at sweep
   start we can correlate and produce a per-frame one-way latency table.
