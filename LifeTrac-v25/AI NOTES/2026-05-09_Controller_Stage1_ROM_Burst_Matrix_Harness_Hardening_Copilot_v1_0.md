# Controller Stage 1 ROM Burst Matrix Harness Hardening (Copilot v1.0)

Date: 2026-05-09  
Scope: Stabilize and validate `run_rom_baseline_burst_matrix.sh` so runs produce complete terminal artifacts (`summary.txt`, `delay_summary.csv`) instead of partial-only directories.

## 1) Problem statement

Recent burst-matrix runs repeatedly produced partial artifacts (`results.csv`, `burst_summary.csv`, per-burst OpenOCD logs) but often missed final summary outputs. This prevented reliable burst-pass rate comparisons across delays.

## 2) Root causes identified

1. OpenOCD gdb-port collisions across bursts/runs
- Symptom in per-burst logs: `couldn't bind gdb to socket on port 3333: Address already in use`.
- Effect: some bursts failed/aborted early and left partial state.

2. Wrapper/child process shutdown behavior causing apparent stalls
- With OpenOCD wrapped by `timeout`, cleanup could sit longer than expected around process teardown.
- Effect: runs appeared to stall after writing most probe rows.

3. Operator-side launcher pitfall (`pkill -f` self-match)
- A launch line containing `pkill -f run_rom_baseline_burst_matrix.sh` can match the launcher command itself and kill before execution.
- Effect: silent no-op launches that looked like harness failures.

## 3) Harness changes applied

File changed:
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_rom_baseline_burst_matrix.sh`

Changes:

1. Disable OpenOCD gdb server in burst runs
- Launch now uses `openocd -c "gdb_port disabled" ...`
- Purpose: remove port 3333 contention between nearby runs.

2. Add OpenOCD lifetime cap (configurable)
- New env knob: `OPENOCD_LIFETIME_S` (default `75`).
- When `timeout` exists, burst OpenOCD is launched under timeout.
- Purpose: bound stuck/long-lived OpenOCD lifespan.

3. Harden cleanup to force deterministic teardown
- Cleanup now:
  - sends `kill` to wrapper PID,
  - kills wrapper child OpenOCD via `pkill -P "$OCD_PID" openocd`,
  - escalates to `kill -9` on wrapper PID,
  - then `wait`s.
- Purpose: avoid hanging on lingering wrapper/child relationships.

4. Add run progress trace
- New `run.log` in run output directory with:
  - start metadata,
  - per-burst `burst_start` / `burst_done`,
  - terminal `done` marker.
- Purpose: distinguish active-run state from failed-finalization state.

## 4) Validation run

Pulled evidence folder:
- `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_rom_baseline_burst_verify2_2026-05-09_083803/`

On-target OUTDIR (from `summary.txt`):
- `/tmp/lifetrac_p0c/rom_baseline_burst_verify2/T6_rom_baseline_burst_2022-05-04_083803`

Run settings:
- `DELAYS_MS=30`
- `BURSTS_PER_DELAY=1`
- `ATTEMPTS_PER_BURST=3`
- `BURST_PASS_MIN_ACK=1`

Observed result (`summary.txt`):
- `TOTAL_BURSTS=1`
- `PASS_BURSTS=0`
- `FAIL_BURSTS=1`
- `TOTAL_PROBES=3`
- `ACK_COUNT=0`
- `NACK_COUNT=1`
- `SILENT_COUNT=2`
- `BEST_DELAY_LINE=30,1,0,1,0.0000,3,0,1,0,2,0`

Progress confirmation (`run.log`):
- `burst_start` and `burst_done` present
- final `done` marker present
- timestamps show same-run completion window (no partial-only termination)

## 5) Interpretation

- The harness now finalizes correctly on a minimal verification case and writes complete summary artifacts.
- This clears the immediate execution-path blocker for burst-matrix experiments.
- Performance result in this verify run remains negative (no ACK bursts), but this run's purpose was harness reliability, not timing optimization.

## 6) Recommended next bench step

Run a small but meaningful matrix with completed summaries guaranteed:
- `DELAYS_MS=30,50`
- `BURSTS_PER_DELAY=5`
- `ATTEMPTS_PER_BURST=10`
- `BURST_PASS_MIN_ACK=1`
- snapshots disabled for first pass (`PRE_BURST_SNAPSHOT_CFG=` and `POST_FAIL_SNAPSHOT_CFG=`)

Then compare:
- per-delay `pass_bursts` / `bursts`
- per-delay ACK/NACK/SILENT mix
- run.log timing for any residual long-tail teardown behavior

If stable, scale to `BURSTS_PER_DELAY=10` and re-enable targeted snapshots only for fail bursts.
