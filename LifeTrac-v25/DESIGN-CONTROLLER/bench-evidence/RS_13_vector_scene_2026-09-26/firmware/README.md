# L072 bench firmware kept with the RS-13.1 record

These are **bench builds** (`mingw32-make bench`, `HOST_ALLOW_REG_WRITE_DIAG=1`),
not the production binary. The production binary is the tracked
`firmware/murata_l072/build/firmware.bin` (md5 `589c1203…` on `main`), and it
refuses the host's register writes, the carrier pin included. Flash only with
[FLASH_RUNBOOK.md](../../../firmware/x8_lora_bootloader_helper/FLASH_RUNBOOK.md)
(`REVIVE_MODE=reboot`, so expect an X8 reboot and a `/tmp` re-push), and only on
an explicit GO.

| file | md5 | size | what it is |
|---|---|---|---|
| `firmware_bench_diag_0c1bb0a9.bin` | `0c1bb0a9573f813137f941dfa47177d0` | 24 860 B | **The firmware on both boards** (base `2D0A1209DABC240B`, tractor `2E2C1209DABC240B`), unchanged since 2026-09-15 and through all of RS-13.1. This is the RS-12.15 v2 bench build of the #125 head (merged into `main` `d3751286`). It was confirmed on air as "the SHIPPED build" in leg U (`bench-evidence/RS_12_15_clock_authority_2026-09-14/RESULTS.md`). Until now it existed only as an untracked file in the bench PC's build directory. Rebuilding it from source has not been re-verified byte-for-byte. |
| `firmware_bench_diag_43f0a74c_fhss_gap1500.bin` | `43f0a74cde4137dccf8e8943bf9c3acc` | 24 864 B | Branch `imp/fhss-authority` @ `2916601e` (on origin): `SX1276_FHSS_AUTHORITY_STREAK_GAP_MS` 1000 → 1500 ms so a 1 fps stream could chain into FHSS time authority (RS-13.1 A11). **Never flashed, never validated on air, and rejected.** A 1500 ms gap lets the base's commands, paced at ≥ 1.0 s, chain into authority, which the strict `< 1000 ms` discriminator exists to prevent (`sx1276_fhss_authority.h`). A11 was closed by flying at 2 fps instead. Kept for reference only. |

Both files were copied from the bench PC on 2026-10-04 before the session's
worktrees were removed.
