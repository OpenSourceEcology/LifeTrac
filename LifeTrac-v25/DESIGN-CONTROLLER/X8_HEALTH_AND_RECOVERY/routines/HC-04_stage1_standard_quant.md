# HC-04 — Stage 1 standard quant (canonical pass)

**Purpose:** End-to-end "is this board fit for bench work?" gate. Drives the
full L072 flash + boot + AT probe pipeline once per cycle. This is the test we
run by default to confirm a board is healthy.

**When to run:** After HC-01..HC-03 PASS. The canonical "still good?" check.

## Procedure

```powershell
powershell -NoProfile -ExecutionPolicy Bypass `
  -File LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_stage1_standard_quant_end_to_end.ps1 `
  -AdbSerial 2E2C1209DABC240B `
  -Cycles 5
```

Vary `-AdbSerial` and `-Cycles` per intent:
- Quick sanity: `-Cycles 5`
- Confidence after a recovery: `-Cycles 20`
- Soak: `-Cycles 100+`

The launcher's preflight (line ~229) exports `gpio8/10/15` and asserts
`gpio10=1` (H7 NRST released) before each cycle.

## Pass criteria

| # | Check | Expected |
|---|---|---|
| 1 | Each cycle reports `READY` phase | yes |
| 2 | No `cannot read IDR` errors | zero |
| 3 | All cycles `PASS_SYNC` | `cycles_passed == cycles_total` |
| 4 | `__BENCH_RADIO_SLEEP_AUDIT__=PASS` per cycle | yes (see memory: SX1276 SLEEP on exit) |

## Common failure modes

| Symptom | Likely cause | Action |
|---|---|---|
| `Error connecting DP: cannot read IDR` every cycle | IOMUXC pad drift on gpio10 | HC-03 to confirm; escalate T2 / T3 |
| First cycle PASS, later cycles FAIL | Bridge degradation or watchdog reboot mid-run | Check `dmesg`; HC-02 between cycles |
| `READY` reached but L072 verify fails | Murata side issue (rare) | Re-run with `-Cycles 1`; check `/dev/ttymxc3` from HC-02 |
| `BENCH_RADIO_SLEEP_AUDIT=MISSING` | Helper bypass or `--no-sleep-on-exit` slipped in | Inspect probe stdout; not a board fault |

## Verdict matrix

| Date | Board | Cycles | Pass | Result | Notes |
|---|---|---|---|---|---|
| 2026-05-12 | Board 2 (`2E2C`) | 2 | 2 | PASS | Control known-good. |
| 2026-05-12 | Board 1 (`2D0A`) | 5 | 0 | FAIL_SYNC ×5 | `cannot read IDR` every cycle. |
| 2026-05-12 | Board 1 (`2D0A`) | 5 | 0 | FAIL_SYNC ×5 | After NRST pulse. No change. |
| 2026-05-12 | Board 1 (`2D0A`) | 1 | 0 | FAIL_SYNC | After `prep_bridge.sh` + watchdog auto-reboot. No change. |
| 2026-05-12 | Board 1 (`2D0A`) | 2 | 0 | FAIL_SYNC ×2 | Same. |
