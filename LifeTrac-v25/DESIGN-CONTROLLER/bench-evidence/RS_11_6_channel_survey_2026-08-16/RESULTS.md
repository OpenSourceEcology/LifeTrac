# RS-11.6 leg 3 — band survey and the 902.5 MHz escape

SHA `9fee4460` (survey tool + harness knob on the same branch).
Survey transcript: [`survey_base.jsonl`](survey_base.jsonl).
Escape-leg archive: `radio_monitor_20260816_200712_9fee4460`.

## 1. The band survey: not one emitter — two

902.0–928.0 MHz inclusive at 500 kHz steps — 53 points × 30 s dwell (≥4
ticks of a 7 s-class beacon each), base radio, profile-2 modem settings,
radios otherwise silent.

**46 of the 49 channels from 903.0 to 927.0 MHz show hot samples at
−42…−48 dBm** (the three exceptions — 905.5, 907.0, 925.5 — are
consistent with the ~20 % per-dwell miss probability; 927.5 at the top
edge also read quiet at max −75; see §4).
Per-channel burst intervals split the activity into two families:

| device | period | dominant zone | peak |
|---|---|---|---|
| A (the RS-11.6 emitter) | ~7.0 s, drifting clock | 903–915, 925–927 | none found — flat −43 |
| **B (new)** | **10.000 s, exact** | 914–923.5 | **923.5 MHz, −30 dBm** |

Device B was invisible in all prior work: at 915.0 its collisions hid inside
the bulk-loss population. Its −30 dBm peak with sharp falloff (−53 by
924.0 MHz) localises its carrier near 923.5 MHz, and −30 dBm received means
it is genuinely close to the base antenna.

Device A's signature is stranger: dead-flat −43 across 13+ MHz with no
peak anywhere in the surveyed band. That is not a narrowband neighbor — it
is either a genuinely wideband emission (chirp/sweep) or **front-end
leakage from a strong carrier outside the surveyed grid**. Its true
carrier remains unlocalised.

**The only quiet spectrum at 500 kHz resolution: 902.0–902.5 MHz**
(max −86…−98 dBm; the probability of two adjacent channels going quiet by
sampling luck is ~4 %).

Corrections this forces onto the earlier record: leg 2's "effectively
fixed-frequency in/near our channel" inference was the wrong branch of an
either/or — the emitter reaches *every* channel, our fixed-channel sniffs
simply could not distinguish in-band from everywhere. The single-emitter
framing also falls: the 915.0 corruption was the *combined* work of two
periodic devices (~42 + ~30 ticks per 300 s), which retroactively explains
why the escape (below) bought more than the single-emitter ceiling
predicted.

## 2. The escape leg: 902.5 MHz, standard 300 s, direct A/B vs leg 1

Identical operating point to leg 1 except `-ForceFrfHz 902500000`:

| metric | leg 1 (915.0 MHz) | leg 3 (902.5 MHz) |
|---|---:|---:|
| **raw fragment loss** | 141 / 2405 = **5.9 %** | 80 / 2412 = **3.3 %** |
| hot corrupt captures (>−60 dBm) | 11–13 @ −43…−47 | **5** @ −45…−57 |
| corrupt captures total | 86 | 96 |
| reassembler timeouts | 82 | 87 |
| healthy link | −69 dBm / +5.0 dB | **−67 dBm** / +5.0 dB |
| goodput | — | 1969 B/s @ util 80 % |

**Fragment loss nearly halved.** The periodic hot population fell ~75 %
(and the residual five hits at −45…−57 confirm the emitters still reach
902.5 at reduced leakage level — quiet, not silent). The corrupt-capture
*count* rose slightly while loss fell: at 902.5 more corruption events are
marginal (demodulated, CRC-failed, dumped) rather than outright kills,
consistent with the interference arriving ~2+ dB weaker there. The healthy
link also gained ~2 dB — unexplained, possibly antenna/front-end response
at the band edge; noted, not claimed.

The −2.6 pp improvement exceeds the single-emitter ceiling (~1.4 pp) from
the corrected parity replay — as it should, now that the survey shows TWO
periodic emitters were hitting the 915.0 channel.

## 3. Verdict and standing recommendation

1. **Frequency escape works.** In-channel energy, not front-end desense,
   was the dominant corruption mechanism at 915.0.
2. **Bench legs should run `-ForceFrfHz 902500000`** until the emitters are
   physically removed. ⚠️ Comparability: this is a NEW operating point —
   results do not compare directly against the 915.0-series legs. Record
   `force_frf_hz` (now in params.txt) when citing any run.
3. **915.000 MHz — the band center and every cheap sensor's default — is
   the worst possible DTS parking spot**, on this bench and likely at any
   site. The field profile should not default to it; RS-11.7's site survey
   picks the center per installation.
4. **The remaining ~3.3 % floor is the clustered bulk process** — now
   cleanly separated from the interferer story and the next campaign.
5. The physical hunt gains a second, easier target: device B at ~923.5 MHz
   / −30 dBm / exact 10 s tick — `hunt_sniff.ps1` will see it at that
   carrier from arm's reach of the base antenna.

## 4. Caveats

- Single leg, n=1; the 915.0-series run-to-run spread was ~0.5 pp. The
  5.9→3.3 delta is ~5× that, so the direction is solid; the exact magnitude
  deserves the usual n=2 confirmation.
- The survey's per-channel dwell catches a 7 s beacon with ~80 %
  probability per channel; isolated quiet channels may be sampling luck,
  but the 902.0–902.5 pair passing together is ~96 % real.
- Survey timestamps span 24 min, over which device A's clock drifts;
  family assignment used per-channel intervals, not a global fit, for
  exactly that reason.
