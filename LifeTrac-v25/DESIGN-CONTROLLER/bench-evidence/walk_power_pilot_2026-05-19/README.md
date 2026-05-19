# Walk-Power Paired Sweep Pilot — 2026-05-19

First over-the-air paired sweep on the live X8+L072+SX1276 bench, exercising
`run_walk_power` (S1.1) end-to-end against a second board running
`--probe rx_listen`.

## Setup

- **TX board (A)**: Portenta X8 ADB `2D0A1209DABC240B`, L072 + SX1276 on Max Carrier.
- **RX board (B)**: Portenta X8 ADB `2E2C1209DABC240B`, identical hardware.
- **Frequency**: 915.000 MHz (US FCC Part 15.247) — verified live via
  `regfrf_check.py` (RegFrf=0xE4C000 on both boards before the run).
- **PHY**: SF7 / BW125 / CR4-5 / sync 0x12 / +14 dBm PA_BOOST baseline,
  walked 2..17 dBm in 1 dB steps.
- **Geometry**: same bench layout as the 2026-05-18 W2-02 stability run
  (per user confirmation).
- **Payload**: 24 bytes, ASCII tag `WP s<step> p<dbm> i<idx>` for join.
- **Per step**: 50 attempted TX frames, ~3 s per step (incl. CFG_SET + stats).
- **LBT**: firmware default (not explicitly disabled). `radio_tx_abort_lbt_delta`
  was 0 across every step — LBT did not interfere.

## Runner

`LifeTrac-v25/tools/paired_walk_power_sweep.ps1` — orchestrates RX-listen
launch on board B, then `walk_power` on board A, then pulls the CSV and
prints the verdict.

## Analyser

`LifeTrac-v25/tools/analyse_paired_sweep.py` — joins the TX-side
`walk_power_paired.csv` with the RX-side `rx_listen_board_b.log` by parsing
the `WP s<step>` tag inside the received payload hex.

## Result — `__PAIRED_SWEEP_VERDICT__=OK`

| step | dBm | tx_ok | rx | PER % | RSSI mean | SNR mean |
|-----:|----:|------:|---:|------:|----------:|---------:|
| 0    | 2   | 44    | 44 | 0.00  | -119.0    | -6.6     |
| 1    | 3   | 42    | 41 | 2.38  | -119.0    | -5.7     |
| 2    | 4   | 42    | 42 | 0.00  | -118.4    | -4.5     |
| 3    | 5   | 43    | 42 | 2.33  | -118.2    | -3.6     |
| 4    | 6   | 43    | 43 | 0.00  | -117.7    | -2.5     |
| 5    | 7   | 42    | 42 | 0.00  | -117.3    | -1.7     |
| 6    | 8   | 44    | 44 | 0.00  | -117.1    | -1.0     |
| 7    | 9   | 44    | 44 | 0.00  | -116.5    | -0.1     |
| 8    | 10  | 42    | 42 | 0.00  | -115.3    |  1.0     |
| 9    | 11  | 42    | 41 | 2.38  | -114.4    |  1.7     |
| 10   | 12  | 42    | 41 | 2.38  | -114.1    |  2.0     |
| 11   | 13  | 42    | 42 | 0.00  | -113.3    |  2.8     |
| 12   | 14  | 43    | 43 | 0.00  | -112.3    |  3.1     |
| 13   | 15  | 44    | 44 | 0.00  | -111.6    |  3.8     |
| 14   | 16  | 42    | 42 | 0.00  | -110.8    |  5.0     |
| 15   | 17  | 43    | 43 | 0.00  | -109.8    |  5.2     |

- **Mean PER over all steps: 0.59 %** (5 lost frames out of 685 transmitted)
- **RSSI**: monotonic, +9.2 dB walk for +15 dB TX walk (compression suggests
  near-field coupling between bench antennas — expected for the geometry).
- **SNR**: monotonic, +11.8 dB walk — close to 1:1 with TX power.
- **TX losses to firmware airtime limiter** (`radio_tx_abort_airtime_delta`):
  6-8 of 50 frames per step. This is the duty-cycle gate the L072
  firmware enforces (EU-spec heritage). Not an RF issue; flagged for a
  separate follow-up (loosen / make configurable for US).

## Falsification

- Verified RegFrf = 0xE4C000 (= 915.000 MHz, US ISM band) on **both** boards
  immediately before the run.
- Verified `RegPaConfig` (0x09) tracks `CFG_SET_REQ(TX_POWER_DBM)` 1:1 via
  `falsify_tx_power_paconfig.py` (BASELINE=0x8C, 2→0x80, 8→0x86, 14→0x8C,
  17→0x8F) on the TX board the same session.

## Files

- [walk_power_paired.csv](walk_power_paired.csv) — TX-side CSV emitted by
  `run_walk_power` (board A).
- [rx_listen_board_b.log](rx_listen_board_b.log) — RX-side raw log
  (`__RX_FRAME__` lines) from board B's `rx_listen --rx-window 120`.
- [walk_power_board_a.log](walk_power_board_a.log) — full TX-side stdout
  (CFG_SET acks, per-cycle TX_DONE_URC events, STATS_URC dumps).
- [paired_sweep_join.csv](paired_sweep_join.csv) — per-step JOIN produced by
  `analyse_paired_sweep.py`.

## Notes for the next run (S1.4 full evidence)

- Bump `--per-step-count` from 50 → 200 once the airtime limiter is
  characterised (or temporarily lifted) so we don't lose ~15 % of attempts
  to the duty-cycle gate.
- Capture RegFrf in the runner log automatically (one extra `read_reg(0x06..0x08)`
  in `run_walk_power` setup) so the evidence is self-contained.
- Add an explicit `--no-lbt` flag pass-through (firmware default may change).
- Time-correlate TX timestamps with RX `timestamp_us` to compute per-frame
  one-way latency; useful for jitter analysis.
