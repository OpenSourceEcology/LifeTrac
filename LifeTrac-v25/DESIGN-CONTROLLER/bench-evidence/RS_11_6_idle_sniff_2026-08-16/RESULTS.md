# RS-11.6 leg 2 — the interferer is EXTERNAL, confirmed in raw energy with all radios silent

Two back-to-back idle legs, 2026-08-16 evening, SHA `e1092868`.

| leg | instrument | archive |
|---|---|---|
| 2a | standard harness, `-SynthFps 0` (zero frames offered) | `radio_monitor_20260816_153155_e1092868` |
| 2b | `air_coupling_rssi_sniff.py`, raw `RegRssiValue` @ ~12.9 Hz, 300 s, base radio in RXCONT | [`rssi_sniff_idle.jsonl`](rssi_sniff_idle.jsonl) |

Analysis: [`tools/rssi_sniff_periodicity.py`](../../tools/rssi_sniff_periodicity.py),
transcript [`periodicity_analysis.txt`](periodicity_analysis.txt).

## 1. Leg 2a: the demodulator sees nothing in idle

One `crc_dump` in 300 s (≈12/h, the RS-11.5 idle-floor order), at
−108 dBm / −12.2 dB SNR, seconds after the base's 16-copy startup command
burst. Nothing at the −43 dBm interferer signature. This was the
pre-registered ambiguous null — resolved by leg 2b.

Useful narrowing even from the null: adb/USB debug streaming, both boards'
Linux stacks, MQTT, and the daemons were all running during this leg. The
always-on USB/adb path alone is therefore not the emitter.

## 2. Leg 2b: the 7.08 s emitter is there in raw energy, radios silent

Floor: median −114 dBm, p95 −106. Against that floor:

**13 samples at −43…−45 dBm — the exact leg-1 interferer signature — on a
hard grid: period 7.0733 s, R = 0.983, residual RMS 0.030 cycles** (cycle
indices 0, 3, 10, 12, 15, 18, 22, 23, 25, 27, 29, 35, 39). The same grid
measured at 7.0842/7.0853 s in the traffic runs, now observed with **zero
LoRa traffic in the air**.

The full-series phase fold at 7.0853 s independently shows periodic
elevation (max−min bin spread 3.54 dB vs circular-shift null 95th pct
3.24 dB, p ≈ 0.005).

### Burst duration ≈ 25 ms, and why the demodulator misses it

The sniffer samples every ~77 ms and caught the emitter in 13 of ~42 grid
cycles. Catch probability ≈ burst/77 ms → **burst ≈ 25 ms**. That resolves
leg 2a's null completely: a ~25 ms non-LoRa burst does not trigger LoRa
preamble detection on its own, so in idle the demodulator never engages. In
traffic runs the same burst lands inside one of our 100 ms fragments and
surfaces as a `crc_dump` — corrupting a bounded region of an
otherwise-clean reception, exactly the damage shape measured in the
temporal analysis (header intact 28/28, RIFF intact 27/28).

### Verdict

**The bench floor's periodic component is an external ~915 MHz emitter:
~25 ms bursts at a hard ~7.08 s cadence, arriving at −43 dBm** — ~26 dB
above our healthy signal level. It is not our TX chain (radios silent), not
the USB/adb path (running during the null leg), and not any daemon
behaviour. The burst profile — tens of ms, OOK-era duty cycle, rigid ~7 s
interval — matches the classic fingerprint of a **battery-powered 915 MHz
ISM telemetry sensor** (weather/temperature/humidity sensors, TPMS, utility
meters commonly beacon every ~7 s).

## 3. Second discovery: a separate 1.000 s emitter

174 weak excursions (median −102 dBm, p90 −93) sit on a **1.00003 s**
period (R = 0.462, jittery/partial — a duty-cycled digital ticker). It is
NOT a subharmonic of the 7.08 s line (fold at 7.0853/7 = 1.0122 s is null,
p = 0.13). Retrospectively, the traffic-run captures showed a marginal 1 s
concentration (R = 0.220, p = 0.016 uncorrected) that now has an
explanation. At ~24 dB below the healthy signal it is mostly harmless to
traffic, but it may account for part of the weak-tail corrupt population.

## 4. What this changes

1. **The hands-on hunt now has a fingerprint and a live detector.** Search
   for a battery ISM sensor near the bench. The sniffer is the geiger
   counter: the line shows within ~10 cycles (~70 s), so power off / remove
   suspects one at a time and re-sniff 120 s each. No 300 s loss runs
   needed.
2. **The RS-11.6 leg-2 candidate list is re-ordered**: near-field coupling
   from carriers/harnesses and USB 3.0 noise are both DEMOTED for the
   periodic component — neither predicts a rigid 7.08 s clock with our
   systems idle. (They remain candidates for the clustered bulk loss, which
   the parity replay showed owns ~⅔ of the floor.)
3. **The ~4 % post-removal floor estimate stands** (parity replay,
   corrected): removing this emitter buys back at most ~⅓ of the 5.9 %
   loss. The clustered bulk component is a separate hunt.

## 5. Leg 2c — twin 600 s sniffs localize the emitter to the base carrier

Simultaneous 600 s sniffs on both radios
([`rssi_sniff_idle_base_600s.jsonl`](rssi_sniff_idle_base_600s.jsonl),
[`rssi_sniff_idle_tractor_600s.jsonl`](rssi_sniff_idle_tractor_600s.jsonl)),
zero traffic, ~84 emitter cycles each:

| radio | hot level | period | R | excursions |
|---|---:|---:|---:|---:|
| base | **−42…−45 dBm** | 7.0797 s | 0.883 | 34 |
| tractor | −62…−63 dBm | 7.0805 s | 0.875 | 49 |

Same grid on both radios to within 0.8 ms — one emitter — and **20 dB
hotter at the base**. In free space 20 dB is a ~10× distance ratio, so the
source sits within roughly a tenth of the carrier-to-carrier separation
from the base antenna: **arm's reach of the base carrier**. The hunt zone
is the base station's immediate surroundings — shelf above/below, the wall
directly behind, anything battery-powered sitting on or next to it.

Period note: the 600 s fits (7.0797/7.0805) sit between the 300 s sniff
(7.0733) and the traffic-capture fits (7.0842/7.0853). Spread ≈ 0.07 %
across captures hours apart — consistent with a cheap drifting oscillator,
i.e. more evidence for a battery device, and a caution against using the
fourth decimal as an identity check between sessions.

### The 1 Hz-class tickers are board-local, not room emitters

The base sees a line at exactly **1.00000 s** (R = 0.438); the tractor
instead shows **0.93554 s** (R = 0.363). Different periods on different
radios means these are each board's own local electronics, not something in
the room. Both are weak (median ≈ −102 dBm, ~24 dB under healthy signal)
and are deprioritized — worth one line in the record, not a hunt.

## 6. Caveats

- The sniffer period (7.0733 s) sits ~0.17 % below the crc_dump-derived
  7.0842/7.0853 s. With 13 events over 39 cycles the fit uncertainty is
  ~0.005 s, so this is ~2σ — consistent with one emitter plus measurement
  error, or a slowly drifting cheap oscillator (which a battery sensor is).
- "External" here means external to our radio TX chain and the USB/adb
  path. The boards' own power electronics were live during both legs, so a
  board-local switching emitter is not fully excluded — but nothing in a
  board power stage plausibly runs on a rigid 7.08 s clock; that cadence is
  firmware-like, not regulator-like.
- The tractor's `tx_smoke` daemon container was still running (idle, RXCONT
  armed, zero TX) during leg 2b; the base's daemon was stopped and its
  radio driven by the sniffer.
