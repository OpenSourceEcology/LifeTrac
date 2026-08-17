# RS-11.6 item 1 — temporal analysis of the corrupt captures

**Analysis only. No bench time, no radios, no boards touched.**
Tool: [`tools/crc_dump_temporal.py`](../../tools/crc_dump_temporal.py).
Raw transcript: [`analysis_output.txt`](analysis_output.txt).
Inputs: every committed archive containing `crc_dump` captures — five runs,
**429 captures**, all `reg_profile=2`, all 300 s.

| archive | started | n | RSSI reference† | note |
|---|---|---:|---:|---|
| `radio_monitor_20260802_132817_30e1fbc6` | 18:23 | 126 | −97 dBm* | pre-RF-switch-fix |
| `radio_monitor_20260802_171735_107634bb` | 22:12 | 72 | −62 dBm* | post-fix |
| `radio_monitor_20260802_172500_03ce8044` | 22:19 | 78 | −75 dBm* | post-fix |
| `radio_monitor_20260802_173023_03ce8044` | 22:25 | 67 | −60 dBm* | post-fix |
| `radio_monitor_20260816_122735_9c891670` | 17:22 | 86 | −69 dBm | leg 1, Taoglas |

† **Correction (review catch, PR #99): this column was first published as
"healthy median", which overstated what was measured.** Only the 2026-08-16
archive carries the `rx_rf` healthy-frame instrument (`healthy windows 29`);
the four earlier archives report `healthy windows 0`, and their reference
(\*) is the **median of the corrupt captures themselves** — the tool's
fallback, now labeled as such in its output. The −69 dBm leg-1 value is a
true healthy median.

## 1. The headline: the interferer has a hard 7.085 s cadence

Separating the captures that carry **both halves** of the interference
signature — stronger than that run's RSSI reference (healthy median where
measured, see table note) **and** SNR below −5 dB — and fitting a period to
their arrival times:

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

Scope note (review catch, PR #99): the correlation's bins start at the first
`air_gap` mark ~10 s in, so captures before that mark — the span where the
startup commands actually fly — sit outside it. Counted explicitly: **at most
1 of the 429 captures across all five archives falls in that span**, so the
blind spot is immaterial; the exoneration covers 428/429 by measurement and
the remaining 1 by absence of anything to correlate against.

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

## 7. The damage is LOCALISED, not total

We have no clean reference payload, so damaged bytes cannot be diffed
directly. Two reference-free integrity proxies survive:

- **`rx_len == 255`** — the LoRa explicit header demodulated correctly, so
  damage did not reach the header.
- **`RIFF` magic present** — a 4-byte known-plaintext landmark inside the
  payload survived.

Comparing those between populations, in each archive independently:

| archive | interferer: header / RIFF | bulk: header / RIFF |
|---|---|---|
| `…0802_132817` | 100 % / 85.7 % (n=7) | 87.4 % / 38.7 % (n=119) |
| `…0802_171735` | 100 % / 100 % (n=2) | 82.9 % / 18.6 % (n=70) |
| `…0802_172500` | 100 % / 100 % (n=8) | 84.3 % / 22.9 % (n=70) |
| `…0816_122735` | 100 % / 100 % (n=11) | 84.0 % / 20.0 % (n=75) |
| **pooled** | **100 % / 96.4 % (n=28)** | **85.0 % / 25.2 % (n=401)** |

**Every interferer-population capture kept its header, and all but one kept
the RIFF landmark.** The bulk population loses the header 15 % of the time
and the landmark 75 % of the time. Under a binomial null at the bulk's
25.2 % landmark-survival rate, 27 of 28 survivals is not a chance outcome.

So the strong periodic hit does **not** destroy the packet. It corrupts a
bounded region while leaving the header and the payload's early structure
demodulating cleanly — a short, hard hit inside an otherwise good reception.
The ordinary bulk corruption is the opposite: damage distributed widely
enough to take out the header and the landmark most of the time.

**This reverses the mitigation guidance given in §9 of this document's first
revision**, which argued FEC was unlikely to help because the interference
was "overwhelming for the duration of the hit". That was written before the
shape was measured and was wrong about the premise: the hit is bounded, and
bounded symbol damage inside an otherwise-clean packet is precisely the case
FEC exists for. What is still unmeasured is *how many* symbols are damaged,
which decides whether CR 4/8 has enough redundancy — see §8.

**The instrument that would settle it:** have the tractor log the fragment it
transmitted, so a true byte-level diff becomes possible. That is a host-side
change on the TX daemon, needs no firmware, and would turn every future
crc_dump into an exact damage map — position, run length, and burst count.
It is the natural successor to this analysis.

## 8. Caveats

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

- **The RIFF landmark is a proxy, not a damage map.** A fragment only
  contains `RIFF` if it spans a WebP frame start, so landmark *absence* is
  ambiguous between "corrupted" and "never present". Landmark *presence* is
  unambiguous, and it is the direction the interferer population sits in, so
  the comparison holds — but it caps how far this can be pushed without the
  TX-side reference described in §7.

## 9. What this buys the next bench session

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

**A note on mitigations (revised — see §7).** CR 4/8 and RS-4.1 XOR parity
were framed as responses to a random corruption floor. An earlier revision of
this document argued FEC would not help because a 26 dB-hot hit is
overwhelming; **§7 measured the shape and that reasoning does not hold** —
the interferer leaves the header and the payload's early structure intact and
damages a bounded region, which is the case FEC is designed for. Both
mitigations are therefore live:

- **CR 4/8** is now plausible rather than dismissed, at ~37 % airtime. Whether
  it has enough redundancy depends on the damaged-symbol count, which §7's
  proxies cannot measure — the TX-side reference logging described there is
  the prerequisite for sizing it honestly.
- **RS-4.1 XOR parity** remains well matched at ~25 % airtime per train,
  since it absorbs a whole lost fragment regardless of intra-packet damage
  extent, and single-fragment loss is the observed failure mode.

Neither should be built before §9.1/§9.2 run: removing the emitter is worth
more than paying 25–37 % airtime to tolerate it.

**Bench-configuration warning for the profile-0 leg.** Profile 0 is
SF7/**BW125** at the runtime (`host_cfg_profile_default_bw_hz()` returns
`HOST_CFG_PROFILE_BENCH_BW_HZ` = 125 000), not BW250 as this campaign's notes
previously recorded. At BW125 a **255 B fragment is 399 616 µs, which exceeds
the 380 000 µs dwell cap by 19.6 ms**; the largest payload that fits is
**243 B**. The standard leg sends 255 B fragments, so a profile-0 A/B at the
usual size would transmit over the cap and the airtime accountant may abort
those TXs — confounding exactly the comparison it was meant to make. Any
profile-0 leg must drop to ≤243 B fragments.

## 10. Addendum (2026-08-16 late): the bulk population vs train boundaries

A follow-up test of RS-11.4's boundary-event mechanism using the crc_dump
captures against `published frame_id` events (same log, one clock — tool
[`tools/bulk_loss_boundary.py`](../../tools/bulk_loss_boundary.py),
transcript [`bulk_boundary_output.txt`](bulk_boundary_output.txt)):

**0 of 359 bulk (non-interferer, settled) captures fall within ±100 ms of a
publish event, against a uniform-null expectation of ~7 % — in all five
archives independently.** Median |offset| ≈ 1.5 s ≈ one train period.
*(Refinement per PR #99 review: the documented boundary proxy is the next
train START ≈ publish + 213.9 ms, so the window was also tested centered
at +214 ms — 1.6–3.2 % across the five archives, still well below the ~7 %
null. The anti-correlation holds against the correctly-centered window.)*

This is NOT evidence against boundary clustering — it is a selection effect
with its own information: **a corrupt fragment usually kills its own train's
publish**, so the reference event is removed by the very corruption being
located, and the nearest surviving publish sits a full train away. The
instrument therefore cannot localise bulk corruption within the train; the
per-index `lost_frag_idx` attribution (RS-11.4) remains the right tool for
that. What this DOES independently confirm is the capture↔train-kill
linkage: captured bulk corruption lives almost exclusively in trains that
failed to publish, quantifying RS-11.4's 23–34 % timeout observation from a
second instrument.
