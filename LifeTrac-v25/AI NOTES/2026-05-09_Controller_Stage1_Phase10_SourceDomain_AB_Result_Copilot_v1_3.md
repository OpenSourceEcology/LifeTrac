# Controller Stage 1 Phase 10 Source-Domain A/B Result (Copilot v1.3)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Scope

Close the Phase 10 atomic gate loop with post-fix tooling and determine whether a valid in-session source-domain discriminator can be produced.

## Tooling Changes Applied

Two local harness updates were made before this run:

1. `run_phase10_source_domain_ab_atomic.sh`
- ROM attempt now launches OpenOCD in background and runs verifier during active hold, matching the legacy successful pattern (`run_pa11_pf4_test.sh`).

2. `verify_l072_rom.sh`
- probe order changed to test ROM sync first (`0x7F` at `19200 8E1`), then AT probe (`19200 8N1`).
- this matches legacy ordering where ROM ACK had been observed.

## Evidence

Primary completed post-fix atomic run:

- `DESIGN-CONTROLLER/bench-evidence/T6_phase10_source_domain_ab_atomic_2026-05-09_2059_rom79_partial/`

Secondary immediate rerun (stability variance capture):

- `DESIGN-CONTROLLER/bench-evidence/T6_phase10_source_domain_ab_atomic_2026-05-09_2100_full_unexpected00/`

## Primary Run Result (Decisive A/B)

From `...2059_rom79_partial/summary.txt`:

- `PHASE10_ATOMIC_RESULT=OK`
- `ROM_ACK_ATTEMPT=1`
- `USERHALT_ROM_RESP_SIZE=0`
- `USERHALT_FIRST_BYTE=NA`

ROM half (`rom1_verify_l072_rom.txt`):

- `ROM_RESP_SIZE=1`
- first byte `0x79`
- verdict: ROM bootloader ACK present

User-halt half (`userhalt_verify_l072_rom.txt`):

- `ROM_RESP_SIZE=0`
- verdict: silent on `0x7F`

Interpretation:

- same session, same verifier primitive, different target state
- ROM state gives positive STM32 ROM ACK
- user-halt state does not

This satisfies the intended Phase 10 source-domain discriminator condition.

## Secondary Rerun Variance

From `...2100_full_unexpected00/summary.txt`:

- `PHASE10_ATOMIC_RESULT=FAIL_NO_ROM_BASELINE`
- `ROM_ACK_ATTEMPT=0`
- `MAX_ROM_ATTEMPTS=1`

`rom1_verify_l072_rom.txt` in this rerun shows `0x00` bytes rather than `0x79`.

Interpretation:

- atomic A/B proof is now demonstrated at least once under controlled conditions
- immediate rerun instability remains and should be treated as a ROM-entry stability issue, not as invalidation of the successful discriminator capture

## Conclusion

Phase 10 now has a valid controlled success capture:

- ROM-positive (`0x79`) and user-halt negative (`silent`) were observed in one atomic run using one harness.

Remaining work shifts from proof-of-difference to reproducibility hardening of ROM baseline entry.
