# Controller Stage 1 Phase 10 Source-Domain A/B Result (Copilot v1.0)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Scope

Run a source-domain discriminator with the same X8 sender/probe primitive in two target states:

- ROM-hold intent (`06_assert_pa11_pf4.cfg`)
- user-app then immediate halt (`47_phase9_boot_then_halt_000ms.cfg`)

Probe primitive in both halves:

- `verify_l072_rom.sh` on `/dev/ttymxc3`
- AT-silence check at `19200 8N1`
- ROM sync check at `19200 8E1` sending `0x7F` and expecting `0x79`

## Evidence Folder

- `DESIGN-CONTROLLER/bench-evidence/T6_phase10_source_domain_ab_2026-05-09_201304/`

Files captured:

- `rom_openocd_06.txt`
- `rom_verify_l072_rom.txt`
- `userhalt_openocd_47.txt`
- `userhalt_verify_l072_rom.txt`

## Integrity Checks

- Both OpenOCD invocations attached and halted the H7 target successfully.
- Both verify-script invocations completed and emitted verdict text.

## Observed Results

### A) ROM-hold intent (`06_assert_pa11_pf4.cfg`)

From `rom_verify_l072_rom.txt`:

- `AT_RESP_SIZE=0`
- `ROM_RESP_SIZE=0`
- verdict string: `silent on 0x7F - BOOT0 not actually asserted on the L072 net.`

No `0x79` ACK observed.

### B) User-halt intent (`47_phase9_boot_then_halt_000ms.cfg`)

From `userhalt_verify_l072_rom.txt`:

- `AT_RESP_SIZE=0`
- `ROM_RESP_SIZE=4`
- hex payload: `00 00 00 00`
- verdict string: `Unexpected response - review hex above.`

No `0x79` ACK observed.

## Interpretation

This specific Phase 10 run does not provide a clean A/B proof of source-domain state dependence because the ROM baseline requirement was not met in the same capture set.

What this run does establish:

- both halves remain negative for STM32 ROM ACK (`0x79` absent),
- user-halt returned non-ACK low bytes (`0x00` pattern), which is distinguishable from ROM-half silence but still not a successful bootloader sync.

Therefore, this run is classified as **inconclusive** for the intended discriminator.

## Conclusion

Phase 10 (this capture set) is inconclusive as a source-domain A/B proof because a deterministic ROM ACK baseline was not reproduced.

## Recommended Next Step

Re-run the exact same A/B procedure after first re-establishing a deterministic ROM baseline (`0x79` on `0x7F` at `19200 8E1`) in the same session. Only then compare against user-halt behavior for final source-domain classification.
