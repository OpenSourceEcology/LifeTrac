# L072 Hello-World Build + X8 Flash Attempt (Method G)

Date: 2026-05-08
Author: Copilot (GPT-5.3-Codex)

## Scope

Completed the pending task to build a minimal L072 hello-world image from the canonical Murata firmware Makefile, stage it onto the connected Portenta X8, and execute the existing flash pipeline.

## What Was Completed

1. Added explicit `hello` build target to the canonical Makefile:
   - File: `DESIGN-CONTROLLER/firmware/murata_l072/Makefile`
   - New variables:
     - `HELLO_TARGET := hello_world`
     - `HELLO_BUILD  := build_hello`
     - `HELLO_SRCS   := startup.c hal/platform.c hello_world.c include/static_asserts.c`
   - New target:
     - `make hello` delegates to `all` with those hello-world overrides.

2. Removed duplicate secondary build system:
   - Deleted `DESIGN-CONTROLLER/firmware/murata_l072/Makefile.hello`
   - Rationale: keep one canonical build entry point and avoid drift.

3. Confirmed ARM cross toolchain and make:
   - `arm-none-eabi-gcc` found at:
     - `C:/Users/dorkm/AppData/Local/Arduino15/packages/arduino/tools/arm-none-eabi-gcc/7-2017q4/bin/arm-none-eabi-gcc.exe`
   - make found at:
     - `C:/Users/dorkm/AppData/Local/Microsoft/WinGet/Packages/BrechtSanders.WinLibs.POSIX.UCRT_Microsoft.Winget.Source_8wekyb3d8bbwe/mingw64/bin/mingw32-make.exe`

4. Built hello-world successfully:
   - Command pattern used:
     - `mingw32-make -f Makefile CROSS=<toolchain-prefix> hello`
   - Output artifacts:
     - `DESIGN-CONTROLLER/firmware/murata_l072/build_hello/hello_world.elf`
     - `DESIGN-CONTROLLER/firmware/murata_l072/build_hello/hello_world.bin`
     - `DESIGN-CONTROLLER/firmware/murata_l072/build_hello/hello_world.hex`

5. Size output observed:
   - Total sections: `8914` bytes
   - `.cfg_reserved`: `8192` bytes (expected reserved region)
   - No compile or link errors after including `hal/platform.c` required by startup fault/reset hooks.

6. Staged image + scripts onto connected X8 over ADB:
   - Device detected: `2E2C1209DABC240B`
   - Pushed helper dir to `/tmp/lifetrac_p0c`
   - Pushed `tools/stm32_an3155_flasher.py` to `/tmp/lifetrac_p0c`
   - Staged image as:
     - `/tmp/lifetrac_p0c/mlm32l07x01.bin`

## Flash Attempt Result

Attempted:

```bash
cd /tmp/lifetrac_p0c
bash run_flash_l072.sh /tmp/lifetrac_p0c/mlm32l07x01.bin
```

Result:
- Script started and sanity checks passed.
- OpenOCD launch failed with:
  - `Error: open: Permission denied`
- `run_flash_l072.sh` aborted because READY phase was never reached.

Root cause:
- ADB shell user is `fio` (non-root).
- `adbd cannot run as root in production builds`.
- `sudo` exists but requires a password.

## Verified On-Device Prerequisites

On X8:
- `openocd` present: `/usr/bin/openocd`
- `python3` present: `/usr/bin/python3`
- UART node present: `/dev/ttymxc3`

## Exact Commands To Finish (requires sudo on X8)

Run these on X8 shell (SSH preferred):

```bash
cd /tmp/lifetrac_p0c
sudo bash run_flash_l072.sh /tmp/lifetrac_p0c/mlm32l07x01.bin
sudo bash boot_and_listen.sh 8
```

Expected capture log:
- `/tmp/lifetrac_p0c/boot_listen.log`

Optional quick check:

```bash
tail -120 /tmp/lifetrac_p0c/flash_run.log
tail -120 /tmp/lifetrac_p0c/boot_listen.log
```

## Notes

- The build task is complete and reproducible from the canonical Makefile.
- The remaining blocker is privilege elevation for OpenOCD on the X8 host.
- Once sudo execution is available, flash + UART capture should proceed directly with existing scripts.
