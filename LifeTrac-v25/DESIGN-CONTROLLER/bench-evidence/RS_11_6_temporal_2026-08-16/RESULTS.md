# RS-11.6 item 1 — temporal analysis of the corrupt captures

**Analysis only. No bench time, no radios, no boards touched.**
Tool: [`tools/crc_dump_temporal.py`](../../tools/crc_dump_temporal.py).
Raw transcript: [`analysis_output.txt`](analysis_output.txt).
Inputs: every committed archive containing `crc_dump` captures — five runs,
**429 captures**, all `reg_profile=2`, all 300 s.

| archive | started | n | healthy median | note |
|---|---|---:|---:|---|
| `radio_monitor_20260802_132817_30e1fbc6` | 18:23 | 126 | −97 dBm | pre-RF-switch-fix |
| `radio_monitor_20260802_171735_107634bb` | 22:12 | 72 | −62 dBm | post-fix |
| `radio_monitor_20260802_172500_03ce8044` | 22:19 | 78 | −75 dBm | post-fix |
| `radio_monitor_20260802_173023_03ce8044` | 22:25 | 67 | −60 dBm | post-fix |
| `radio_monitor_20260816_122735_9c891670` | 17:22 | 86 | −69 dBm | leg 1, Taoglas |

## 1. The headline: the interferer has a hard 7.085 s cadence

Separating the captures that carry **both halves** of the interference
signature — stronger than that run's own healthy median **and** SNR below
−5 dB — and fitting a period to their arrival times:

| archive | n | fitted period | R | residual RMS | verdict |
|---|---:|---:|---:|---:|---|
| `…0802_132817` (pre-fix) | 7 | **7.1447 s** | 0.962 | 0.068 cyc | quasi-periodic |
| `…0802_172500` | 8 | **7.0842 s** | 0.986 | 0.027 cyc | **hard grid** |
| `…0816_122735` | 11 | **7.0853 s** | 0.987 | 0.043 cyc | **hard grid** |

The two tightest fits agree to **0.0011 s — 16 parts per million** — from runs
**14 days apart**. Every event in those two runs lands within 0.076 of an
integer cycle. In the 2026-08-16 run the eleven events sit on cycles
0, 2, 4, 5, 7, 11, 15, 18, 26, 30, 31 of a single grid.

**This is a fixed-cadence emitter, and it has been sitting on the bench for the
entire campaign.** It is present in the pre-RF-switch-fix run as well, so it
predates and survives that fix — consistent with it explaining the residual
floor that the fix did not move.

The SNR signature is equally stable: median **−10.0 / −10.8 / −10.8 dB**
across the three runs.

## 2. It is not bursty — it is more regular than random

Worth stating because the question was framed as "periodic or bursty", and the
answer is neither of the usual two:

- Fano factor **0.68–1.04** across 2–30 s bins — at or **below** 1.
- Inter-arrival **CV = 0.789**, again below 1.

A bursty interferer gives F ≫ 1 and CV > 1. A memoryless one gives F = CV = 1.
This process is **under-dispersed** — more evenly spaced than chance. That is
what a duty-cycled emitter looks like, and it is consistent with the grid fit
above rather than an independent alternative to it.

## 3. Self-interference is ruled out, in all five runs

The base transmitted its commands inside the first 10 s window and then sent
**nothing for the remaining ~290 s** (`cmd_tx_ok` is constant at 17 thereafter),
while corrupt receptions continued at a steady rate throughout. Our own
transmissions cannot account for them.

This is a clean negative and it holds in every archive. The tractor's TX
schedule could not be cross-checked directly — see the clock caveat in §6 —
but the base is the receiver, so the base's own TX is the transmission that
would matter, and it is flat.

## 4. It is not locked to any cadence of ours

Rayleigh tests at each known system period, on the 2026-08-16 run:

| cadence | R | p |
|---|---:|---:|
| fragment ToA ~0.100 s | 0.172 | 0.078 |
| fragment slot ~0.117 s | 0.146 | 0.161 |
| synth frame 0.500 s | 0.074 | 0.625 |
| train boundary ~1.85 s | 0.053 | 0.786 |
| 10 s stats window | 0.120 | 0.293 |

Nothing concentrates. Combined with §3, the emitter is **external to and
asynchronous with our system**.

## 5. A methodological trap this analysis had to step around

The first pass found "best period 3.5458 s, p = 4×10⁻⁵, SIGNIFICANT" over the
whole capture set. **That result was an artifact and is not claimed here.**

A fold scan over a process with regular-ish spacing will always report high
phase concentration at roughly its mean inter-arrival, with no external clock
involved — and 3.5458 s sat right next to the 3.25 s mean gap. The control is
a null that preserves the inter-arrival *distribution* while destroying
absolute phase: shuffle the intervals and re-scan. That control is implemented
in `shuffled_null()` and is run on every archive.

The whole-population peak does survive its shuffled null, but it is not the
result worth quoting, because the whole population mixes three regimes (§6).
The 7.085 s grid in §1 is the finding, and it is supported by residuals rather
than by a scan p-value.

## 6. Corrections to the record

Two items in [`RS_11_6_antenna_swap_2026-08-16/RESULTS.md`](../RS_11_6_antenna_swap_2026-08-16/RESULTS.md)
need restating. Neither overturns that document's conclusion — the interference
finding stands and §1 strengthens it — but both would mislead if carried
forward.

**(a) "Corrupt frames at −65.8 dBm mean" pools three distinct regimes.**
The 86 captures in that run comprise: a **warm-up population** (the link's own
`rx_rf` median runs −84/−82/−84/−73 for the first four windows before settling
at −69 for the remaining 25, and the early corrupt captures track that curve
almost exactly); a **bulk population** at −64…−69; a **near-noise-floor tail**
at −113/−114; and the **interferer population** at −43…−47 with SNR ≈ −10.8.
The −65.8 mean is an average across all four and describes none of them. The
correct statement is that the interferer population arrives **~26 dB above the
healthy median**, not 3 dB.

**(b) The first ~60 s of any run is not comparable to the rest.** No prior
analysis excluded it. The tool now detects the settle point and reports the
settled population separately.

## 7. Caveats

- **Possible aliasing.** We only observe the interferer when it collides with
  one of our receptions, and our fragments arrive on a ~117 ms cadence. A
  faster true period could in principle alias to 7.085 s. The tight residuals
  argue against it, but a direct observation (§8) would settle it.
- **Absolute levels are not comparable across legs.** TX power and the RX path
  differ between the pre-fix and post-fix runs, so the interferer's apparent
  level moving −86 → −54 → −43 dBm across them should not be read as the
  emitter changing. The **period** is the robust cross-run quantity.
- **Small n per run** (7–11 events in the interferer population). The
  cross-run agreement is what carries the result, not any single fit.
- The 2026-08-02 22:12 and 22:25 runs yielded fewer than 4 events passing the
  cut, so no fit is reported for them. Absence of a fit there is not evidence
  of absence of the emitter.

## 8. What this buys the next bench session

The emitter now has a **fingerprint**: ~7.085 s, ~26 dB above our healthy
level, SNR ≈ −10.8 dB. That converts leg 2 from "try things and see if loss
moves" into a direct search, and it changes the recommended order:

1. **Switch things off one at a time and watch the 7.085 s line**, rather than
   watching the loss rate. Loss rate needs a 300 s run and has a ~0.5-point
   run-to-run spread; the periodic line is visible in far less and is
   unambiguous. Prime suspects are anything on a ~7 s duty cycle near the
   bench — the USB 3.0 hypothesis stays live, and the tractor's USB camera is
   worth isolating specifically.
2. **One idle capture** (no traffic, `rx_rf` + `crc_dump` running). If the
   7.085 s line persists with our radios silent, the emitter is fully external
   and the search narrows to the room. This is the single highest-value leg
   and it needs no traffic at all.
3. Only then the profile-0 A/B, which is confounded three ways (see issue #98)
   and is now a lower-value test than the two above.

**A note on mitigations.** CR 4/8 and RS-4.1 XOR parity were framed as
responses to a random corruption floor. Against a fixed-cadence emitter that
lands ~26 dB hot, FEC is unlikely to recover the affected symbols — the
interference is not marginal, it is overwhelming for the duration of the hit.
Parity across fragments is the better-matched mitigation of the two, since it
absorbs a whole lost fragment rather than trying to correct within one. Both
should wait until §8.1/§8.2 have run.
