# RS-12 fix confirmation — NoParkLast, n=3 interleaved: CONFIRMED

Issue #107. Six 300 s legs at 927.5 MHz (clean at both ends of the series:
0 hot, max −99/−102 dBm), interleaved ctrl/fix, every leg counter-bracketed
AND arrival-instrumented. Driver `run_rs12_noparklast_ab.ps1`; summary in
[`summary.txt`](summary.txt); legs mapped in [`legs.json`](legs.json).

## 1. Per-leg mechanism verification — the knob did the thing, every time

| leg | arm | last-pair demod gap | pairs <80 ms | loss | penult % |
|---|---|---:|---:|---:|---:|
| 01 | ctrl | 41.7 ms | 87 % | 3.4 % | 43 |
| 02 | fix | 114.7 ms | 26 % | 1.9 % | 11 |
| 03 | ctrl | 41.7 ms | 87 % | 3.2 % | 44 |
| 04 | fix | 114.7 ms | 27 % | 1.5 % | 11 |
| 05 | ctrl | 41.7 ms | 87 % | 3.4 % | 41 |
| 06 | fix | 114.7 ms | 25 % | 1.9 % | 13 |

No leg crosses arms on any metric.

## 2. Arm comparison

| metric | ctrl | fix | perm p |
|---|---:|---:|---:|
| **loss** | 3.3 % ±0.1 | **1.8 % ±0.2** | 0.10 (floor) |
| **penultimate lock** | 42.6 % ±1.7 | **11.6 % ±1.3** | 0.10 (floor) |
| goodput | 1977 ±15 | 1919 ±17 (**−2.9 %**) | 0.10 |
| fw-drop (ack-contaminated) | 63 ±16 | 50 ±9 | 0.30 |

The arms are the tightest of the campaign (loss sd 0.1–0.2 pp vs the 2×
swings of the search phase) — consistent with the mechanism being the
dominant variance source and now controlled.

## 3. Residuals, stated plainly

- Fix legs still show ~26 % of last pairs under 80 ms: the hold releases
  on TX_DONE receipt, which can beat the host pacer. The residual
  penultimate share (11.6 % vs 7.7 % uniform) tracks it. A stricter hold
  (TX_DONE + pacing remainder) would close it; the firmware fix
  (double-buffered URC path) removes the deadline entirely.
- The fw-drop metric stays ack-contaminated (~32/leg) and is reported for
  continuity, not inference.

## 4. Decision

**`-NoParkLast 1` is the standing bench operating point as of this
series** — 1.5 pp of loss for 2.9 % offered throughput, confirmed at the
design's significance floor with per-leg mechanism verification.

Whether to flip the daemon DEFAULT (production behaviour) is a maintainer
call, framed both ways in PR #108: for — the win is large, cheap, and
host-only; against — a workaround-as-default can mask the underlying
firmware bug, whose proper fix (double-buffer or minimum inter-fire
spacing) is specified and small. The flash session confirms with
`rx_urc_lost` either way.

## 5. Addendum — strict hold (leg K): the guarantee made unconditional

The plain hold races the pacer on TX_DONE receipt (~26 % of pairs still
rode). Leg K adds a timed gap: the final fragment submits no earlier than
`LIFETRAC_NO_PARK_LAST_GAP_MS` (default 80) after the penultimate's
TX_DONE. Archive `radio_monitor_20260817_190504_4b8c55be`, bracketed,
channel clean:

| metric | ctrl (n=3) | plain hold (n=3) | strict hold (n=1) |
|---|---:|---:|---:|
| last pairs <80 ms | 87 % | ~26 % | **0 % (min 130.9 ms)** |
| penultimate lock | 42.6 % | 11.6 % | **6 % (≤ uniform)** |
| loss | 3.3 % | 1.8 % | **1.5 %** |
| timeouts | ~69 | ~40 | **32** |
| frames published | ~90 | ~144 | 143 |
| offered | 2374 | 2317 (−2.4 %) | 2227 (−6.2 %) |

The stricter guarantee costs more offered throughput (the 80 ms gap plus
the wait-loop's 50 ms poll quantization — median realized gap 161 ms vs
the 117 ms target). Frame-level delivery matches the plain hold; loss and
timeouts improve further. Tuning headroom exists (a 40–50 ms gap or finer
poll would recover most of the offered cost) but correctness-first is the
right default while the firmware fix is pending.

**Final recommendation stands: `-NoParkLast 1` with the default 80 ms
gap.** The flash-session confirmation becomes maximally crisp: a
strict-hold leg predicts `rx_urc_lost == 0`; a control leg predicts
`rx_urc_lost ≈ timeouts`.
