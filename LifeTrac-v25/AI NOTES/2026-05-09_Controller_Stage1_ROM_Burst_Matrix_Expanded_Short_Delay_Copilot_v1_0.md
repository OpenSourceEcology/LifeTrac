# Controller Stage 1 ROM Burst Matrix — Expanded Short-Delay Campaign (Copilot v1.0)

Date: 2026-05-09
Scope: Expand the burst matrix into the previously unexplored short-delay zone (5–30 ms) to find the optimal ROM bootloader sync timing for the L072 on the Portenta X8.

## Background

The prior campaign (`T6_rom_baseline_burst_2022-05-04_084927`) swept 30 ms and 50 ms:
- 30 ms: 2/5 burst passes (0.40), 50 ms: 1/5 burst passes (0.20)
- 30 ms was the best observed delay, but the short-delay side was completely unexplored.

This run sweeps the 5–30 ms short-delay zone at 5 ms granularity.

## Run configuration

Completed evidence folder:
- [DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_085905/](DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_2022-05-04_085905/)

Run settings from `summary.txt`:
- `DELAYS_MS=5,10,15,20,25,30`
- `BURSTS_PER_DELAY=5`
- `ATTEMPTS_PER_BURST=10`
- `BURST_PASS_MIN_ACK=1`
- `OPENOCD_LIFETIME_S=75`

Overall result:
- `TOTAL_BURSTS=30`
- `PASS_BURSTS=8`
- `FAIL_BURSTS=22`
- `BURST_PASS_RATE=0.2667`
- `TOTAL_PROBES=300`
- `ACK_COUNT=8`
- `NACK_COUNT=30`
- `ZERO_COUNT=7`
- `SILENT_COUNT=235`
- `OTHER_COUNT=20`

Best delay line (auto-selected by script): `5 ms` (0.40 pass rate)

## Delay-level results

| delay_ms | pass/5 | pass_rate | ack | nack | zero | silent | other | signal quality |
|----------|--------|-----------|-----|------|------|--------|-------|----------------|
| **5**    | **2/5**| **0.40**  | 2   | 3    | 0    | 45     | 0     | **clean**      |
| 10       | 1/5    | 0.20      | 1   | 4    | 0    | 45     | 0     | clean          |
| 15       | 1/5    | 0.20      | 1   | 5    | 0    | 44     | 0     | clean          |
| 20       | 1/5    | 0.20      | 1   | 9    | 0    | 40     | 0     | clean, 2 NACKs/burst |
| 25       | 1/5    | 0.20      | 1   | 1    | 5    | 28     | 15    | **noisy**      |
| **30**   | **2/5**| **0.40**  | 2   | 8    | 2    | 33     | 5     | mixed (noisy 3/5) |

## Per-burst detail

| delay | burst | ack | nack | zero | silent | other | pass |
|-------|-------|-----|------|------|--------|-------|------|
| 5     | 1     | 1   | 0    | 0    | 9      | 0     | PASS |
| 5     | 2     | 0   | 1    | 0    | 9      | 0     | FAIL |
| 5     | 3     | 0   | 1    | 0    | 9      | 0     | FAIL |
| 5     | 4     | 0   | 1    | 0    | 9      | 0     | FAIL |
| 5     | 5     | 1   | 0    | 0    | 9      | 0     | PASS |
| 10    | 1     | 0   | 1    | 0    | 9      | 0     | FAIL |
| 10    | 2     | 0   | 1    | 0    | 9      | 0     | FAIL |
| 10    | 3     | 0   | 1    | 0    | 9      | 0     | FAIL |
| 10    | 4     | 1   | 0    | 0    | 9      | 0     | PASS |
| 10    | 5     | 0   | 1    | 0    | 9      | 0     | FAIL |
| 15    | 1     | 0   | 1    | 0    | 9      | 0     | FAIL |
| 15    | 2     | 0   | 1    | 0    | 9      | 0     | FAIL |
| 15    | 3     | 1   | 0    | 0    | 9      | 0     | PASS |
| 15    | 4     | 0   | 1    | 0    | 9      | 0     | FAIL |
| 15    | 5     | 0   | 2    | 0    | 8      | 0     | FAIL |
| 20    | 1     | 0   | 2    | 0    | 8      | 0     | FAIL |
| 20    | 2     | 1   | 1    | 0    | 8      | 0     | PASS |
| 20    | 3     | 0   | 2    | 0    | 8      | 0     | FAIL |
| 20    | 4     | 0   | 2    | 0    | 8      | 0     | FAIL |
| 20    | 5     | 0   | 2    | 0    | 8      | 0     | FAIL |
| 25    | 1     | 1   | 1    | 0    | 8      | 0     | PASS |
| 25    | 2     | 0   | 0    | 0    | 1      | 9     | FAIL |
| 25    | 3     | 0   | 0    | 2    | 8      | 0     | FAIL |
| 25    | 4     | 0   | 0    | 0    | 4      | 6     | FAIL |
| 25    | 5     | 0   | 0    | 3    | 7      | 0     | FAIL |
| 30    | 1     | 1   | 4    | 0    | 5      | 0     | PASS |
| 30    | 2     | 0   | 0    | 0    | 5      | 5     | FAIL |
| 30    | 3     | 0   | 0    | 2    | 8      | 0     | FAIL |
| 30    | 4     | 0   | 0    | 0    | 10     | 0     | FAIL |
| 30    | 5     | 1   | 4    | 0    | 5      | 0     | PASS |

## Key findings

### 1. 5 ms ties 30 ms as the best delay (0.40 pass rate)

The short-delay zone was completely unexplored before this run. 5 ms matches the previous best (30 ms, from the prior campaign) and does so with a completely clean signal: zero ZERO bytes, zero OTHER bytes.

### 2. Two distinct signal zones

**Clean zone (5–20 ms):**
- Only ACK, NACK, and SILENT responses
- ROM sees ≤1 response per burst (9 of 10 probes silent)
- Increasing delay from 5→20 ms increases NACK count slightly (3→9) without improving pass rate

**Noisy zone (25–30 ms):**
- ZERO (0x00) and OTHER bytes appear, indicating UART framing artifacts or OpenOCD/IMX GPIO interference with the serial line
- At 25 ms: 15 OTHER + 5 ZERO across 5 bursts; only 1 ACK
- At 30 ms: 5 OTHER + 2 ZERO; 2 ACKs (both from bursts with 4 NACKs + 1 ACK — the ROM is partially processing the sync sequence)

### 3. NACK count increases with delay — but doesn't help

In the clean zone, each failed burst typically yields exactly 1 NACK (ROM sees exactly 1 of the 10 0x7F bytes and rejects it). At 20 ms, failed bursts show 2 NACKs/burst — the ROM is catching up to 2 bytes per window. At 30 ms, passing bursts show 4 NACKs + 1 ACK — the ROM is processing ~5 bytes in the window but only ACKing once.

This suggests the 30 ms window overlaps the ROM's internal retry/autobaud scan cycle, causing multiple bytes to land in the USART receiver. The ACK ultimately comes on the byte that aligns with the ROM's expected sync sequence.

### 4. Overall per-attempt ACK rate is uniformly low

Across all 300 probes: 8 ACKs total = **2.67% per-probe ACK rate**. This is nearly flat across delays:
- 5 ms: 2/50 = 4.0% per probe
- 10–15 ms: 1/50 = 2.0% per probe
- 20 ms: 1/50 = 2.0% per probe
- 25–30 ms: 2/50 = 4.0% per probe (but noisier)

The low and nearly-uniform ACK rate across all delays indicates the timing problem is structural rather than purely a delay selection issue. The ROM bootloader entry window is very narrow and varies run-to-run based on OpenOCD reset sequencing jitter.

### 5. Best delay recommendation: 5 ms

5 ms provides the highest burst pass rate (tied with 30 ms) with the cleanest signal. The 25–30 ms zone introduces UART corruption that could interfere with follow-on protocol exchanges. For production flash tooling, **5 ms is the recommended post-OpenOCD-reset delay** before beginning the 0x7F sync stream.

## Combined campaign pass-rate table (all data to date)

| delay_ms | campaign | pass/bursts | pass_rate |
|----------|----------|-------------|-----------|
| 5        | T7 (this)| 2/5         | 0.40      |
| 10       | T7 (this)| 1/5         | 0.20      |
| 15       | T7 (this)| 1/5         | 0.20      |
| 20       | T7 (this)| 1/5         | 0.20      |
| 25       | T7 (this)| 1/5         | 0.20      |
| 30       | T6 (prev)| 2/5         | 0.40      |
| 30       | T7 (this)| 2/5         | 0.40      |
| 50       | T6 (prev)| 1/5         | 0.20      |
| 75       | 80-probe | 0/20        | 0.00      |
| 100      | 80-probe | 0/20        | 0.00      |

## Next steps

The per-burst pass rate (0.40 best case) and the per-probe ACK rate (~2-4%) are too low for reliable single-pass flashing. Two paths forward:

1. **Increase sync attempts per burst**: Change `ATTEMPTS_PER_BURST` from 10 to 50+ and measure whether the higher attempt count raises burst pass rate toward 1.0. The data suggests ~1 ACK per 10-50 probes is achievable; a longer burst should close in on 100%.

2. **Hardware investigation**: Check BOOT0/BOOT1 pin routing and PA9/PA11 reset control wiring on the actual L072 module. The high silent rate (78%) and sparse ACK rate suggest the ROM bootloader may not be entering the sync-wait loop reliably after OpenOCD reset.
