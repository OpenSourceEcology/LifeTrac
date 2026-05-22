# 2026-05-22 — RX X8 reflash + P1 cold-boot discriminator resolution (v1.0)

## TL;DR
- TX X8 (2E2C1209DABC240B) and RX X8 (2D0A1209DABC240B) both successfully
  re-flashed with current `firmware.bin` (21160 bytes) via the X8-resident
  pipeline (`run_stage1_standard_quant_end_to_end.ps1`).
- Phase 2.3 VERDICT.md complaint (stale RX firmware lacking
  `CFG_KEY_REG_PROFILE` descriptor entry, manifesting as
  `__PROFILE_TIMEOUT_FRAME__` lines) is RESOLVED. Post-reflash discriminator
  shows clean `RUNTIME_PROFILE_ENUM=0` on 5/5 cycles, zero timeouts.

## Two bugs fixed this session (both in the X8 flash path)

### Bug A — `stm32_an3155_flasher.py` `termios` import (TX symptom)
- LmP Python 3.10.4 ships a stripped stdlib that does not include `termios`.
  Flasher crashed at import → wrapper reported `SYNC_OK=0` indistinguishable
  from a real UART sync failure.
- Fix: `termios` import is now guarded by `try/except ImportError`. All
  `termios.*` references are conditional on `_HAVE_TERMIOS`. Serial port
  attributes are configured by `stty` in `run_flash_l072.sh` prior to flasher
  invocation, so the Python-side termios calls are pure belt-and-braces.

### Bug B — RX OpenOCD `SWD DPIDR 0xdeadbeef` (RX symptom)
- RX X8 has a newer OpenOCD build (`0.11.0-dirty 2025-07-14-18:35`) than TX
  (`0.11.0-dirty 2022-10-19-16:13`).
- With sysfs-export preflight pattern ACTIVE (gpio8/10/15 exported, gpio10
  driven high via sysfs) when openocd starts, the newer build's `imx_gpio`
  mmap driver collides with the sysfs kernel pinctrl owner. Result:
  `SWD DPIDR 0xdeadbeef` and never reaches `init`.
- Fix in `run_flash_l072.sh`:
  1. Export gpio8/10/15, drive gpio10=high via sysfs (releases H7 NRST).
  2. `sleep 1` (settle).
  3. **UNEXPORT** gpio8/10/15 immediately before `nohup openocd ... &`.
  4. Poll the openocd log for `READY: L072 in STM32 ROM bootloader` up to
     25 s (was 5 s — newer build takes 8–12 s on first SWD attach).
- Result: RX flash PASS in 35 s end-to-end (`SYNC_OK=1, ERASE_OK=1,
  WRITE_OK=1, VERIFY_OK=1, BOOT_OK=1`).

## Diagnostic discipline lesson (per user memory)
- The first three hypotheses (settle time, OCD slow READY, mmap-pinctrl
  conflict needing longer dwell) were each tested and falsified in one cycle
  by examining `flash_ocd.log` — DPIDR was still `0xdeadbeef`, not just
  "READY not printed yet". The eventual fix (unexport before openocd)
  mirrored the working manual probe (`_rx_swd_probe.sh`), which kept gpios
  exported but ran openocd FOREGROUND — both effectively avoid the
  background/race window the newer driver dislikes.
- See `/memories/methodology.md` and
  `/memories/repo/lifetrac-x8-l072-bootloader.md` (updated).

## P1 cold-boot discriminator result (RX, 5 cycles)
```
profileEmit@line=4  ok=1  drained=0  total=8.27s  text='0'     × 5
```
- No `__PROFILE_TIMEOUT_FRAME__` (no `bootURC@line`).
- All five cycles emit profile enum value `0` cleanly.
- Tool's correlation summary reads "INCONCLUSIVE" because there is no
  variance across cycles to correlate (delta_line / drained_count stats
  are null). **Behaviorally this is the cleanest possible result** —
  the stale-firmware symptom that motivated Phase 2.3 is gone.

## Files touched
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/stm32_an3155_flasher.py`
  (termios optional)
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_flash_l072.sh`
  (preflight + unexport + 25 s READY poll)
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_stage1_standard_quant_end_to_end.ps1`
  (1.5 s settle after preflight)
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx.c`,
  `sx1276_rx_scan_policy.c`, `sx1276_rx_scan_fail.c` (prior-turn changes
  carried into this firmware build)

## Evidence
- TX flash PASS: `bench-evidence/T6_stage1_standard_quant_2026-05-22_162539_412-26160/`
- RX flash PASS: `bench-evidence/T6_stage1_standard_quant_2026-05-22_163521_462-9128/`
- RX P1 discriminator: `bench-evidence/p1_cold_boot_2026-05-22_163619/`

## Recommended follow-up
- Update `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/p1_cold_boot_2026-05-22_161406/VERDICT.md`
  with a "Phase 2.3 resolved post-reflash" addendum, citing this note.
- Update `LifeTrac-v25/AI NOTES/2026-05-21_Open_Problems_To_Fix_v1_0.md` —
  cross out P1.
- Optional: backport `run_flash_l072.sh` unexport fix to TX so the script
  is identical across X8s (TX currently works without it but the fix is
  harmless on the older OCD).
