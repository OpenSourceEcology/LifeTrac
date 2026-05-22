# Phase 2.1 verdict — 2026-05-22 16:03 T2

## Evidence cohorts
- `p1_cold_boot_2026-05-22_160005/` — N=2, PreProbeSleepS=0.05
- `p1_cold_boot_2026-05-22_160154/` — N=2, PreProbeSleepS=2.0
- `p1_cold_boot_2026-05-22_160255/` — N=3, PreProbeSleepS=5.0

(Also one earlier cycle in `p1_cold_boot_2026-05-22_155212/cycle_01_stdout.txt`
from the `adb reboot` draft — same failure mode.)

## Methodology change vs Open Problems v3.0 spec
The original spec (Open Problems v3.0 §1, lines 387-401) proposed a
Δ-clustering cohort analysis with the X8 reboot driving the cold boot.
Two empirical findings forced a revision:

1. `adb reboot` of the X8 did **not** reset the L072 — the probe always
   reported "BOOT_URC not observed during 1.0s settle" after an X8
   reboot, indicating the L072 was still mid-flight or had emitted its
   boot URC well before the probe opened the TTY.
2. The probe issues the `RUNTIME_PROFILE_ENUM` request immediately
   on open, BEFORE its boot-settle loop, so `t_boot_urc_ms` can never
   be measured from the probe's own stdout. The Δ = `t_profile - t_boot`
   metric is therefore not constructible.

Revised methodology:
- Reset the L072 directly via gpio163 NRST (SOFT_RESET_INDEX 3.1), the
  same line `revive_bridge.sh` toggles. ~150 ms total, no USB teardown,
  no wait-for-device. Empirically reproduces the P1 symptom on every
  cycle.
- Sweep the dwell between NRST release and probe launch
  (`-PreProbeSleepS`). If the failure is firmware-not-ready, the
  failure rate should drop as dwell grows. If the failure is
  host-race, the failure rate is dwell-invariant.

## Results

| dwell (s) | N | profile ok | profile err | drained (urcs in buffer at open) |
|----------:|--:|-----------:|------------:|---------------------------------:|
|      0.05 | 2 |          0 |           2 |                                2 |
|      2.00 | 2 |          0 |           2 |                                2 |
|      5.00 | 3 |          0 |           3 |                                2 |

100% failure rate across the full 0.05s..5.0s dwell range. All cycles
report `RUNTIME_PROFILE_ENUM=ERR request_failed:TimeoutError` and
`drained type=0xA0 seq=1` / `drained type=0xA0 seq=2` during the
subsequent boot-settle loop (the URCs are still in the kernel TTY
buffer at the moment the probe opens the port).

## Critical adjacent observation

In every cycle, the probe goes on to **succeed** at `T0a: VER warm-up OK`
*in the same boot* — AFTER its boot-settle loop drains the two queued
STATS URCs. So the L072 is unambiguously responsive to MKRWAN-style
requests; it just isn't being heard by the profile-enum path because
the host's request/response correlator is choking on the leading URCs.

## Verdict: HOST-RACE limb

Firmware-not-ready is falsified: a 5-second dwell with the L072 actively
emitting STATS URCs cannot be "not ready", and AT+VER works on the same
boot. The failure is in the host's request/response handling at the
moment of the first request after open.

## Phase 2.2 implementation (host-race limb)

Per Open Problems v3.0 §1 lines 395-400, the fix is:

1. **`drain_startup_until_quiet(timeout=N ms, quiet_window=Q ms)`** in
   `method_h_stage2_tx_probe_v2.py` (or whichever wrapper owns the
   first profile-enum request). Open port, then continuously read
   and discard frames until no traffic arrives for `quiet_window`
   ms (default 200 ms is plenty given the 2-URC-burst evidence).
2. **Bounded backoff** on the profile-enum request itself: 3 attempts
   with 100/250/500 ms gaps; each attempt drains again before sending
   so a late URC can't poison the correlator.
3. Probe should emit `__PROFILE_DRAINED__=<n>` and
   `__PROFILE_ATTEMPTS__=<k>` tokens so future runs of this same
   discriminator can verify the fix without regressing.

## Caveats / what we did not measure
- We only exercised the RX board (serial `2D0A1209DABC240B`). The TX
  board could in principle behave differently, but both run the same
  L072 image so the same fix applies.
- We did not measure absolute time-to-first-response (the probe's
  stdout has no per-line timestamps). The `PULSE_DONE_AT` token in
  `_p1_pulse_and_probe.sh` provides a reference t0 for a future
  revision that adds per-line wall-clock stamps to the probe.
- The cohort analysis in `summary.json` shows `completed=0` for all
  three runs because the original `completed` filter required
  `t_boot_urc_ms != null`, which is structurally impossible under
  the revised methodology. This is cosmetic; the per-cycle CSV and
  raw stdouts carry the actual evidence. A follow-up commit should
  loosen the filter and add a dwell-sweep summary mode.
