# RS-11.8 — first chantab-grid survey (2026-08-22)

**Headline: the chantab grid's first survey is dominated by the band-wide
~7 s ticker — 49/50 channels "hot" (2–7 hits each, flat −46…−52 dBm),
and the survey therefore CANNOT discriminate channels at the standard
30 s dwell / −75 dBm cut while that ticker is active. The one zero-hot
channel (904.25) is a catch-probability fluke, NOT a valid pick.
Meanwhile 927.5 MHz (x.0/x.5 grid) stayed CLEAN in same-hour spot-checks
bracketing the survey — the top band edge remains protected.**

## Captures (this directory)

- `chantab_survey_20260822.jsonl` — 50 channels, 902.75–927.25 MHz,
  500 kHz steps, 30 s dwell, 0.05 s sampling, base radio (2D0A…),
  detached container `chantab_survey`.
- `devb_sniff_923p5_tractor_20260822.json` — simultaneous 600 s
  single-channel dwell at 923.5 MHz on the TRACTOR radio: 49 hot,
  **max −30 dBm**, hot-timestamp diffs cluster at ~7.0 s and multiples
  (14/21/28/35) with a few short-gap outliers suggesting a second
  interleaved ticker.

## Readings

1. **The ~7 s ticker hits every 30 s dwell wherever the receiver sits**
   (2–7 hits/channel across the whole band; the RS-11.6 leg-3 "flat −43
   on every channel" signature, reconfirmed on the x.25/x.75 grid). Its
   per-channel max tapers toward the top edge (−56 at 927.25 vs −46…−50
   mid-band).
2. **Same-hour control:** 927.5 MHz spot-checks (60 s each) read 0 hot /
   max −99 (morning) and 0 hot / max −96 (immediately post-survey) while
   the adjacent 927.25 chantab channel read hot=2 / max −56. Operating
   channel remained valid all session.
3. **The −30 dBm emitter is loud at BOTH radios** (tractor max −30 at
   923.5; historically −30 at the base too) — potent across the bench,
   not a base-proximity artifact. Hunt target confirmed alive today.
4. Per-channel n≈473–475 samples/dwell — instrument healthy; FRF
   readback OK at each step.

## Same-day synth control leg (post-survey)

`radio_monitor_20260822_183240_375974be` — standard 13-frag synth leg at
927.5, `-NoParkLast 1`, bracketed (`synth_*` files this directory):
**loss 35/2204 = 1.6 %, penultimate idx-11 = 1/35 (3 %, ≤ uniform 7.7 %)**,
crc closure 32=32, drops uniform across idx 1–10. The RS-12 fix holds on
a fresh day; the 0.9–1.6 % band is the known uniform residual awaiting
the flash session's `rx_urc_lost` counter. This leg also verified the
new `seq=` publish-line logging on air (git 375974be): TX↔RX train joins
now possible from standard logs. Against the same-day camera legs
(0.0–0.3 %, 1-frag trains) it localizes the residual floor to
multi-fragment train mechanics.

## Consequences for RS-11.8 (recorded in TODO)

- A single chantab survey cannot seed the hail-set constants. Options,
  in preference order: (a) accumulate the multi-survey stability history
  as planned and rank with `survey_compare.py --history` (the ticker's
  catches should decorrelate across surveys while a real squatter
  repeats); (b) add a ticker-aware metric (e.g. discount hits matching
  the ~7 s cadence, or raise the per-channel verdict to a hot-RATE
  threshold above the ticker's expected 2–7/30 s); (c) longer dwells.
- The band-edge taper + 927.5's persistent cleanliness strengthen the
  case for hail channels at the top edge — which the current chantab
  does not reach (ends 927.25). The table-extension question in
  POWER_MANAGEMENT.md gains weight.
