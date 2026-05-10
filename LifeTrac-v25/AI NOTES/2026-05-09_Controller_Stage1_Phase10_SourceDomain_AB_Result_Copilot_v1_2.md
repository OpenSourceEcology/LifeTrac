# Controller Stage 1 Phase 10 Source-Domain A/B Result (Copilot v1.2)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Scope

Validate the new Phase 10 atomic gate runner end-to-end and determine whether a deterministic ROM-positive control can be reproduced before any ROM-vs-user-halt A/B inference.

This run used:

- `run_phase10_source_domain_ab_atomic.sh`
- `MAX_ROM_ATTEMPTS=3`
- ROM-hold intent via `06_assert_pa11_pf4.cfg`
- verifier `verify_l072_rom.sh` on `/dev/ttymxc3`

## Harness Fix Applied First

Before this rerun, `verify_l072_rom.sh` had a control bug in its capture path:

- `cat` was backgrounded inside a subshell
- `$!` was captured by the parent shell instead of the actual reader process
- `kill`/`wait` therefore became unreliable and could leave the verifier stuck before writing its output file

The script was corrected to:

- start `cat` directly in the current shell background
- capture the real PID
- kill and `wait` that exact PID
- clear prior temp response files before each probe

This matters because the earlier atomic attempt stopped after `rom3_openocd_06.txt` without producing `rom3_verify_l072_rom.txt` or `summary.txt`, which was a harness failure, not yet a bench verdict.

## Evidence Folder

- `DESIGN-CONTROLLER/bench-evidence/T6_phase10_source_domain_ab_atomic_2026-05-09_204733/`

Captured artifacts:

- `rom1_openocd_06.txt`
- `rom1_verify_l072_rom.txt`
- `rom2_openocd_06.txt`
- `rom2_verify_l072_rom.txt`
- `rom3_openocd_06.txt`
- `rom3_verify_l072_rom.txt`
- `summary.txt`

## Results

### Atomic gate outcome

`summary.txt` records:

- `PHASE10_ATOMIC_RESULT=FAIL_NO_ROM_BASELINE`
- `ROM_ACK_ATTEMPT=0`
- `MAX_ROM_ATTEMPTS=3`

### Per-attempt ROM verifier outcome

All three verifier captures completed and each reported:

- `AT_RESP_SIZE=0`
- `ROM_RESP_SIZE=0`
- verdict `silent on 0x7F - BOOT0 not actually asserted on the L072 net.`

### OpenOCD state setup

The OpenOCD logs still show the intended control actions for ROM-hold intent:

- GPIO clocks enabled
- `PA_11` driven high
- `PF_4` pulsed low then high
- `PA_11` re-asserted high after reset

So the bench result is now:

- intended BOOT0/reset actions are logged on the H7 side
- UART-side ROM confirmation (`0x79`) still does not occur
- this failure persists across three gated attempts in one controlled run

## Interpretation

This run converts the previous ambiguity into a cleaner negative result.

What changed:

- the atomic runner now completes reliably
- the verifier no longer hangs mid-attempt
- the run writes a machine-readable summary

What did not change:

- no ROM `0x79` ACK was reproduced
- Phase 10 could not advance to the user-halt comparator because the ROM gate never opened

Therefore the primary blocker is now better defined:

- not a harness hang
- not a missing summary-path bug
- but failure to reproduce the ROM-positive control itself under the current invocation path

## Conclusion

Phase 10 remains unresolved, but the atomic gate is now functioning correctly and its result is decisive for this question:

- current state = `FAIL_NO_ROM_BASELINE`
- no valid A/B proof can be claimed until the ROM `0x79` baseline becomes reproducible again in-session

## Next Gate

Next work should focus on why the earlier known-good ROM entry condition is no longer reproducible, using the now-stable atomic harness as the acceptance test.