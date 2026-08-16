# RS-11.6 leg 1 — Taoglas antenna swap, and the first healthy-vs-corrupt comparison

**Archive:** `radio_monitor_20260816_122735_9c891670`
**Setup:** Taoglas TD.95.6H31 Blade (868/915 MHz dual-band omni dipole)
installed on BOTH Max Carriers, replacing the 868-only Würth
Hermippe-III. Antennas swapped IN PLACE — no geometry change. Operating
point identical to leg H (`-TxFeed local -RegProfile 2 -DurationS 300
-SynthFps 2 -SynthBudgetB 3000 -KfRequestDisable 1 -ProbeEcho 0`), so
the comparison is direct.

Both L072s verified before the run: `RS115-INSTRUMENTED-FIRMWARE=YES`,
`host_parse_err=0`, no ring overflow, nothing holding either UART.

## 1. The swap changed nothing

| metric | leg H (Hermippe-III, 868) | leg 1 (Taoglas, 868/915) |
|---|---:|---:|
| raw fragment loss | 131 / 2418 = **5.4 %** | 141 / 2405 = **5.9 %** |
| train timeouts | 74 | 82 |
| corrupt captures | 72 | 86 |
| corrupt-packet RSSI (mean) | −64.3 dBm | −65.8 dBm |

Within run-to-run spread on every axis. **The 868-vs-915 detuning was
not the limiting factor** — a hypothesis raised when the antennas were
first identified, now closed by measurement. Received power did not
improve at all, which also means the bench link is not
antenna-gain-limited at this geometry.

## 2. The finding: the residual floor is INTERFERENCE, not margin

The RS-11.6 healthy-frame instrument (`rx_rf`, shipped 2026-08-02)
produced its first data, letting the two populations be compared for the
first time:

| population | RSSI | SNR |
|---|---:|---:|
| **healthy** frames (29 windows) | **−69 dBm** (median of medians) | **+5.0 dB** |
| **corrupt** frames (n=86) | **−65.8 dBm** mean, −65 median | **+0.3 dB** mean, +1.1 median (range −12.0 .. +6.2) |

**Corrupt packets arrive ~3 dB STRONGER than healthy ones while carrying
~5 dB WORSE SNR.** Path loss cannot produce that — weak signal drives
RSSI and SNR down together. Higher total received power with degraded
signal quality is the signature of a second emitter adding energy into
the receiver during those receptions. The floor is an interference
process, not a link-margin process.

Extreme example captured in-run: `rssi=-43 snr=-10.8` — 26 dB stronger
than the healthy median, with SNR 15 dB worse.

This retroactively explains every null result in the RS-11.5 campaign:
transmit power did not move it (legs I/J), the RF-switch fix did not
beyond its initial +33 dB, and antenna tuning does not either (this leg).
None of those change an interferer.

## 3. Candidates and the tests that separate them

1. **Near-field coupling from the boards/cabling** — attacked directly by
   the leg-2 antenna-REPOSITION test (separate the antennas from the
   carriers and their harnesses). Prediction if true: corrupt-population
   SNR rises toward the healthy population's.
2. **USB 3.0 broadband noise** — a documented 900 MHz-band emitter, and
   both carriers sit on USB cables to the bench PC. Test: move both to
   USB 2.0 ports (or route cabling away from the antennas) and re-run the
   identical leg. Same prediction.
3. Ambient ISM-band emitter in the room — least likely given the idle
   floor measured only ~4 CRC events/hour (RS-11.5 §8), but a long idle
   capture with the new instrument would settle it.

## 4. Instrument note

`rx_rf` reported `n_rssi=n_snr=80` per 10 s window with a very tight
healthy spread (SNR min=med=max=5.0 in most windows, RSSI −70/−69/−68) —
the quantization is the L072's reporting granularity, not a bug. The
healthy population being this tight is itself useful: it makes the
corrupt population's −12 dB excursions unambiguous outliers rather than
a tail of the same distribution.

## 5. Housekeeping observations from the same session

- A tractor reboot brings back the production `tractor-camera`
  container, which maps `/dev/ttymxc3` and steals the radio UART. It
  mimics the break-storm ISR livelock exactly (L072 transmits
  `C:SEND_BOOT` + a COBS READY URC but never answers a request; SWD
  resets do not help). Harness now stops it at launch.
- Reboots wipe `/tmp` on both boards; the harness re-pushes
  `lifetrac_strict` itself, `lifetrac_p0c` (flash/reset tooling) must be
  re-pushed manually when needed.
- Base board USB enumeration was lost and recovered only after a power
  cycle + cable change; the board remained fully reachable on ethernet
  throughout (ping + sshd), confirming the fault was USB-only.
