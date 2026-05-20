# === FCC-B2-b ARTIFACT HEADER BEGIN (v1) ===
# firmware_git_sha: d4dfcb86cfd99bbcbd227844940a1f905336b356
# firmware_git_sha_short: d4dfcb86cfd9
# build_timestamp_utc: 2026-05-20T07:50:34Z
# profile_enum: 0
# profile_string: REG_PROFILE_BENCH_ONLY_FIXED_915
# rfco_summary_schema_ver: 1
# rfco_pertx_schema_ver: 1
# header_schema_ver: 1
# === FCC-B2-b ARTIFACT HEADER END ===

# Walk-Power Paired Sweep — Full S1.4 Evidence — 2026-05-19

Full TX-power sweep across **+2 to +17 dBm in 1 dB steps × 200 packets per step**
on the live X8+L072+SX1276 bench. Successor to
[../walk_power_pilot_2026-05-19/README.md](../walk_power_pilot_2026-05-19/README.md).
Closes S1.4.

## Setup (delta vs. pilot)

Same hardware, geometry, modulation, and runner as the pilot. Changes:

| Knob | Pilot | This run | Why |
|---|---|---|---|
| `--per-step-count` | 50 | **200** | 4× evidence per step → narrower PER confidence interval |
| `--inter-cycle-s` | 0.02 | **0.07** | Pace TX below the 40 % internal airtime budget so the duty-cycle gate doesn't reject 12-16 % of attempts (falsified in pilot follow-up — see §"Airtime gate falsification" below) |
| `--rx-window` | 120 s | **480 s** | Cover the full 16-step × ~23 s/step TX wall-clock |
| Hardware otherwise | identical | identical | — |

- **TX board (A)**: Portenta X8 ADB `2D0A1209DABC240B`.
- **RX board (B)**: Portenta X8 ADB `2E2C1209DABC240B`.
- **Frequency**: 915.000 MHz (RegFrf=0xE4C000) — verified live on both boards via
  [regfrf_check.py](../../firmware/x8_lora_bootloader_helper/regfrf_check.py)
  earlier in the same session.
- **PHY**: SF7 / BW125 / CR4-5 / sync 0x12 / PA_BOOST.
- **Payload**: 24 bytes, ASCII tag `WP s<step> p<dbm> i<idx>` for join.

## Runner

[full_walk_power_sweep.ps1](../../../tools/full_walk_power_sweep.ps1) — same
shape as the pilot orchestrator with the three knob changes above and a
`pkill` pre-flight on board B to clear leftover python from the pilot.

## Analyser

[analyse_paired_sweep.py](../../../tools/analyse_paired_sweep.py) — unchanged
from pilot.

## Result — `__PAIRED_SWEEP_VERDICT__=OK`

| step | dBm | tx_ok | rx  | PER % | RSSI mean | SNR mean |
|-----:|----:|------:|----:|------:|----------:|---------:|
| 0    | 2   | 200   | 198 | 1.00  | -118.9    | -6.6     |
| 1    | 3   | 200   | 198 | 1.00  | -118.7    | -5.6     |
| 2    | 4   | 200   | 194 | 3.00  | -118.3    | -4.6     |
| 3    | 5   | 200   | 197 | 1.50  | -118.1    | -3.7     |
| 4    | 6   | 200   | 197 | 1.50  | -118.1    | -3.2     |
| 5    | 7   | 200   | 198 | 1.00  | -117.6    | -2.0     |
| 6    | 8   | 200   | 197 | 1.50  | -117.1    | -1.1     |
| 7    | 9   | 200   | 196 | 2.00  | -116.6    | -0.1     |
| 8    | 10  | 200   | 198 | 1.00  | -116.0    |  0.1     |
| 9    | 11  | 200   | 198 | 1.00  | -115.1    |  1.1     |
| 10   | 12  | 200   | 198 | 1.00  | -114.4    |  1.8     |
| 11   | 13  | 200   | 196 | 2.00  | -114.0    |  2.1     |
| 12   | 14  | 200   | 199 | 0.50  | -113.1    |  2.9     |
| 13   | 15  | 200   | 198 | 1.00  | -112.3    |  3.4     |
| 14   | 16  | 200   | 200 | 0.00  | -111.5    |  4.2     |
| 15   | 17  | 200   | 199 | 0.50  | -111.2    |  4.5     |

### Headline numbers

- **TX-side**: 3 200 / 3 200 attempted, **100 % `tx_done_ok`, 0 `tx_aborted_airtime`,
  0 `tx_timeout`** across all 16 steps. Confirms airtime gate is fully avoided
  at `--inter-cycle-s 0.07`.
- **RX-side**: 3 162 / 3 200 received → **mean PER 1.22 %**.
  Max per-step PER 3.0 % @ +4 dBm; min 0.0 % @ +16 dBm. 1 payload (of 3 162)
  failed the tag-parse — within noise.
- **RSSI walk**: -118.9 → -111.2 dBm, **monotonic across 16 steps** (Δ ≈ +7.7 dB
  for a +15 dB TX walk — compression consistent with near-field bench coupling).
- **SNR walk**: -6.6 → +4.5 dB, **monotonic** (Δ ≈ +11.1 dB, close to 1:1 with TX power).

## Airtime gate falsification (pilot follow-up)

The pilot lost 12-16 % of attempts to `radio_tx_abort_airtime_delta` at
`--inter-cycle-s 0.02`. Initial hypothesis (EU 1 % duty cycle) was wrong —
falsified by reading
[sx1276_airtime.c#L13](../../firmware/murata_l072/radio/sx1276_airtime.c#L13):

```c
#define SX1276_AIRTIME_CHANNEL_COUNT 16U
#define SX1276_AIRTIME_WINDOW_MS     1000U
#define SX1276_AIRTIME_BUDGET_US     400000U
```

→ **40 % internal duty-cycle limiter** (per channel, per 1-second window).
At SF7/BW125 with 24 B payload ≈ 25.7 ms ToA, the budget allows ~15.6 frames/s.
The pilot's 22 fps overran it. This run at 0.07 s inter-cycle ≈ 11.7 fps fits
comfortably under the budget.

Falsification probe artefact:
[../walk_power_pilot_2026-05-19/walk_power_airtime_falsify.csv](../walk_power_pilot_2026-05-19/walk_power_airtime_falsify.csv)
— 100/100 packets per step @ +14 and +17 dBm with `radio_tx_abort_airtime_delta = 0`.

This 40 % gate is a **self-imposed fairness limit, not regulatory**. See
[../../../AI NOTES/2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md](../../../AI%20NOTES/2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md)
for what the actual FCC envelope requires.

## Files

- [walk_power_paired.csv](walk_power_paired.csv) — TX-side per-step counters.
- [rx_listen_board_b.log](rx_listen_board_b.log) — 3 162 `__RX_FRAME__` lines
  from board B's `rx_listen --rx-window 480`.
- [walk_power_board_a.log](walk_power_board_a.log) — raw TX-side stdout.
- [paired_sweep_join.csv](paired_sweep_join.csv) — joined per-step PER and
  RSSI/SNR aggregates.

## Closes / opens

- ✅ **Closes** S1.4 acceptance: "walk_power produces monotonic RSSI/SNR with
  bounded PER on real hardware across the full +2 … +17 dBm range."
- ⏭️ **Next**: 10-minute mixed-load soak (S1.5) at a single power to surface
  state-machine drift / leaks that a 23 s burst won't catch.
- 📝 **Follow-up**: the 40 % internal airtime gate should be either
  (a) made channel-aware once FHSS lands per FCC notes doc, or
  (b) raised / disabled on US builds since it isn't regulatory.
