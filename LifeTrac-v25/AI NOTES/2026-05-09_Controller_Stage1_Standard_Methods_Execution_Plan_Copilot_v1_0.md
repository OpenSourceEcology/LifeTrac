# Controller Stage1 Standard Methods Execution Plan

**Date:** 2026-05-09  
**Author:** Copilot  
**Version:** v1.0  
**Purpose:** Convert the standard-method research into a concrete bench execution plan for the Murata L072 update path on Portenta X8.

---

## 1. Objective and success criteria

### Objective

Stabilize Stage1 around a deterministic, one-shot ROM bootloader update flow that matches normal embedded practice:

- control-plane pin choreography (BOOT0/NRST),
- data-plane AN3155 flash over UART,
- mandatory verify,
- deterministic user-app reboot and liveness check,
- artifacted logs per run.

### Exit criteria (all required)

1. A single command from host launches the full remote flow (no interactive polling loops).
2. Every run emits machine-parseable status: `sync/getid/erase/write/verify/boot`.
3. Verify failures and boot failures are distinguishable.
4. Reliability is measured only across independent reset cycles (`ATTEMPTS_PER_BURST=1`).
5. A known-good golden image restore path is callable with the same wrapper.

---

## 2. Scope boundaries

### In scope now

- hardening the existing AN3155 updater path,
- standardizing run orchestration and artifacts,
- replacing ADB-heavy polling with one-shot execution,
- standardizing reliability measurement methodology.

### Out of scope now

- broad route/ownership root-cause campaigns,
- OpenOCD lifetime tuning as a primary lever,
- multi-board cross-check campaign,
- application-level protocol debugging beyond post-boot liveness gate.

---

## 3. Canonical run contract

Every run should produce the following contract in one output folder:

- `run_meta.txt`
- `flash.log`
- `control.log`
- `verify.log`
- `boot_probe.log`
- `summary.txt`

`summary.txt` required keys:

- `RUN_ID`
- `BOARD_SERIAL`
- `IMAGE_NAME`
- `OPENOCD_CFG`
- `SYNC_OK`
- `GETID_OK`
- `ERASE_OK`
- `WRITE_OK`
- `VERIFY_OK`
- `BOOT_OK`
- `FINAL_RESULT`
- `ELAPSED_S`

`FINAL_RESULT` enum:

- `PASS`
- `FAIL_SYNC`
- `FAIL_ID`
- `FAIL_ERASE`
- `FAIL_WRITE`
- `FAIL_VERIFY`
- `FAIL_BOOT`
- `FAIL_CONTROL`

---

## 4. Execution phases

## Phase A - Wrapper hardening (single-run)

### Tasks

1. Build one wrapper entrypoint around existing helper scripts and flasher:
   - control path: BOOT0/NRST assert-release,
   - flash path: AN3155 updater,
   - verify path: readback verify,
   - boot path: release BOOT0 + NRST pulse + liveness probe.
2. Enforce one output folder per run with required artifacts.
3. Emit `summary.txt` with required enum and keys.

### Gate A

Pass when one manual run yields a complete artifact set and unambiguous `FINAL_RESULT`.

## Phase B - Reliability quant run (independent resets only)

### Tasks

1. Run `N=20` independent cycles with `ATTEMPTS_PER_BURST=1`.
2. Prohibit multi-sync-within-session attempts in this dataset.
3. Report:
   - pass count,
   - failure breakdown by enum,
   - mean and p95 elapsed time.

### Gate B

Pass when failure classes are stable and diagnosable (no `UNKNOWN` or mixed-state ambiguity).

## Phase C - Golden image restore qualification

### Tasks

1. Run same wrapper on the golden image payload.
2. Verify post-boot liveness signature expected for golden image.
3. Save one reference artifact set marked `GOLDEN_RESTORE`.

### Gate C

Pass when golden restore uses exactly the same orchestration path and returns deterministic `PASS` with matching liveness signature.

---

## 5. Measurement rules (mandatory)

1. Independent attempt unit = one bootloader-entry reset cycle.
2. Do not treat repeated `0x7F` bytes in one session as independent attempts.
3. Treat ADB as launch/transport only, not timing-critical data plane.
4. Keep OpenOCD as pin-control helper only; do not use lifetime as a default optimization variable.

---

## 6. Stop rules

Stop and escalate if any of these occurs:

1. `FAIL_CONTROL` exceeds 2/20 in Phase B.
2. `FAIL_VERIFY` appears after previously passing verify in same image/config.
3. `FAIL_BOOT` appears with clean verify for more than 2 consecutive runs.
4. ADB transport instability prevents artifact completeness for 3 consecutive runs.

Escalation target:

- move to SWD-assisted recovery diagnostics for control-plane confidence,
- do not continue ROM delay hunting as a substitute.

---

## 7. Immediate next run recipe

1. Run Phase A once with current known-good board `2E2C1209DABC240B`.
2. If Gate A passes, run Phase B (`N=20`) same board/config.
3. If Gate B passes, run Phase C golden restore once.
4. Append results to TODO with enum counts and one-line disposition.

---

## 8. Disposition template for TODO

Use this compact format in TODO for each tranche:

- `Tranche ID`:
- `Config`:
- `Runs`:
- `PASS`:
- `Failure enum counts`:
- `Median / p95 runtime`:
- `Decision`:

Example decision values:

- `PROMOTE_AS_CANONICAL`
- `REWORK_CONTROL_PATH`
- `REWORK_VERIFY_PATH`
- `ESCALATE_TO_SWD_DIAGNOSTICS`

---

## 9. References

- Standard-method research note: [2026-05-09_Controller_Stage1_Standard_Methods_for_Murata_L072_Update_and_ROM_Entry_Copilot_v1_0.md](2026-05-09_Controller_Stage1_Standard_Methods_for_Murata_L072_Update_and_ROM_Entry_Copilot_v1_0.md)
- Existing Stage1 chip plan: [2026-05-09_Controller_Stage1_Chip_Research_and_Next_Move_Plan_Copilot_v1_0.md](2026-05-09_Controller_Stage1_Chip_Research_and_Next_Move_Plan_Copilot_v1_0.md)