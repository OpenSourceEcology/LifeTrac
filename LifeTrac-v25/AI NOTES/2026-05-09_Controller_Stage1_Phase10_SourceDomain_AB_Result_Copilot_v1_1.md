# Controller Stage 1 Phase 10 Source-Domain A/B Result (Copilot v1.1)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Scope

Re-run Phase 10 source-domain A/B with identical probe primitive in both halves:

- ROM-hold intent via `06_assert_pa11_pf4.cfg`
- user-app-then-halt via `47_phase9_boot_then_halt_000ms.cfg`
- verifier: `verify_l072_rom.sh` on `/dev/ttymxc3` at `19200 8E1` sync check

## Evidence Folder

- `DESIGN-CONTROLLER/bench-evidence/T6_phase10_source_domain_ab_2026-05-09_2022_rerun_manual/`

Captured artifacts:

- `rom_openocd_06.txt`
- `rom_verify_l072_rom.txt`
- `userhalt_openocd_47.txt`
- `userhalt_verify_l072_rom.txt`

## Results

### ROM-hold half

- OpenOCD confirms intended ROM setup actions: `PA_11` high, `PF_4` reset pulse, hold window active.
- Verify result: `ROM_RESP_SIZE=0` and verdict `silent on 0x7F`.

### User-halt half

- OpenOCD confirms user-boot-then-halt state setup.
- Verify result: `ROM_RESP_SIZE=0` and verdict `silent on 0x7F`.

## Interpretation

Compared to the earlier Phase 10 capture (`T6_phase10_source_domain_ab_2026-05-09_201304`), this rerun did not reproduce the prior user-halt `00 00 00 00` pattern; both halves are now fully silent.

Common point across both captures:

- a deterministic ROM `0x79` ACK baseline was not reproduced in-session when executing the Phase 10 A/B procedure.

Therefore this rerun remains **inconclusive for state-dependent A/B proof** and instead strengthens the immediate operational blocker:

- the ROM baseline entry condition itself is not stable/reproducible under the current invocation path.

## Conclusion

Phase 10 source-domain A/B is still unresolved because the required ROM-positive control (`0x79` after `0x7F`) is not reproducible in the same session.

## Next Gate

Before any further A/B inference, add a dedicated precheck loop that repeatedly asserts ROM state and verifies `0x79` until success, then run the user-halt comparator immediately afterward using the same serial path and timestamps in one atomic script.
