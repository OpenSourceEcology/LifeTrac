# `murata_l072/` — Custom firmware for the Murata `CMWX1ZZABZ-078` SiP

**Status:** Increment 1 complete. Startup, BOOT safe-mode listener, UART2 DMA-on-IDLE host transport, a compile-gated AT service shell, and a minimal SX1276 SPI register driver are now implemented.

**Design docs:** [../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/](../../DESIGN-LORAFIRMWARE/) (00–06)

**Target:** STM32L072CZ inside the Murata SiP — 192 KB Flash, 20 KB RAM, Cortex-M0+ @ 32 MHz.

---

## What is in this increment

| Path | Purpose |
|---|---|
| [include/memory_map.h](include/memory_map.h) | **Single source of truth** for every Flash/RAM region address & size. Consumed by both C and the linker. |
| [include/static_asserts.c](include/static_asserts.c) | Compile-time checks that the memory map closes on 192 KB exactly, that regions don't overlap, and that alignment matches L072 Flash erase granularity. |
| [include/stm32l072_regs.h](include/stm32l072_regs.h) | Register-level STM32L072 definitions used by startup, host transport, and radio driver code. |
| [ld/stm32l072cz_flash.ld](ld/stm32l072cz_flash.ld) | Preprocessor-fed linker script. **Includes `memory_map.h`** so addresses are never hand-mirrored. |
| [startup.c](startup.c) | Vector table, reset handler, `.data` copy, `.bss` clear, and default IRQ handlers in the BOOT region. |
| [boot/safe_mode.c](boot/safe_mode.c) | Implemented N-22 listener with compile-time magic sequence over baud sweep and ROM bootloader jump. |
| [hal/platform.c](hal/platform.c) | HSI16 clock switch, SysTick millisecond timebase, delay helper, IRQ enable helper, and software reset primitive. |
| [host/host_uart.c](host/host_uart.c) | UART2 DMA circular RX + IDLE ISR servicing, AT line recognition, COBS decode/encode, CRC16 checks, and parsed frame queue. |
| [radio/sx1276.c](radio/sx1276.c) | SPI1 register access, basic LoRa modem setup, DIO EXTI wiring, and IRQ event collection. |
| [main.c](main.c) | Bring-up sequence plus minimal host command handling (ping, version, SX1276 reg read/write). |
| [config.h](config.h) | Build-time feature flags. |
| [Makefile](Makefile) | Single canonical build entry point ([DESIGN-LORAFIRMWARE/02 §5](../../DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md), per Claude review §2.4). |

## Memory map (single-slot launch)

The reviews in [DESIGN-LORAFIRMWARE/05](../../DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md) §5.1 and §5.4 converged on **single-slot at launch, defer A/B (N-26) to Phase 6**. The layout below is sized so a future A/B split can land without changing the boot or config regions:

```
0x08000000  +-----------------------------------+
            | BOOT  (vectors + safe-mode)       |   4 KB   resident, never overwritten
0x08001000  +-----------------------------------+
            | APP   (single launch slot)        | 180 KB
0x0802E000  +-----------------------------------+
            | CFG   (calibration + flags)       |   8 KB   2× 4 KB logical sectors
0x08030000  +-----------------------------------+   end of Flash @ 192 KB

Future A/B layout (Phase 6, N-26) splits APP:
  Slot A: 0x08001000  88 KB
  Slot B: 0x08017000  88 KB
  CFG region & BOOT region unchanged.
```

**This map is enforced mechanically.** Any change to one region size that doesn't add up to 192 KB will fail the build via `_Static_assert` in [include/static_asserts.c](include/static_asserts.c). Any change to an address that misaligns against the L072's 128-byte page will also fail. The linker script reads the same constants via the C preprocessor — there are no hand-mirrored numbers in `.ld`.

## RAM map

```
0x20000000  +-----------------------------------+
            | .data + .bss                      |
            | DMA buffers                       |
            | static pools                      |
            | (fills upward)                    |
            +-----------------------------------+
            | (free)                            |
            +-----------------------------------+
            | stack (fills downward)            |
0x20005000  +-----------------------------------+   end of RAM @ 20 KB
```

Stack is reserved at the top (size set in [include/memory_map.h](include/memory_map.h) as `MM_STACK_SIZE`, default 2.5 KB per Claude review §2.3 recommendation).

## Building

### Toolchain (byte-identical builds need exactly this)

| tool | version | notes |
|---|---|---|
| cross compiler | **Arm GNU Toolchain 12.2.MPACBTI-Rel1**: `arm-none-eabi-gcc --version` prints `arm-none-eabi-gcc (Arm GNU Toolchain 12.2.MPACBTI-Rel1 (Build arm-12-mpacbti.34)) 12.2.1 20230214` | From developer.arm.com, *Arm GNU Toolchain Downloads*, release 12.2.MPACBTI-Rel1, "AArch32 bare-metal target (arm-none-eabi)". Bench PC install: `C:\Program Files (x86)\Arm GNU Toolchain arm-none-eabi\12.2 mpacbti-rel1\bin` on `PATH`. Linux x86_64 tarball: `https://developer.arm.com/-/media/Files/downloads/gnu/12.2.mpacbti-rel1/binrel/arm-gnu-toolchain-12.2.mpacbti-rel1-x86_64-arm-none-eabi.tar.xz` (170 644 784 B, sha256 `17455a06c816031cc2c66243c117cba48463cd6a3a3fdfac7275b4e9c40eb314`, as published in Arm's `.sha256asc`). |
| make | Windows: WinLibs `mingw32-make` (winget `BrechtSanders.WinLibs.POSIX.UCRT`), **run from PowerShell only**. Linux: GNU make. | On Windows the Makefile uses `cmd` syntax for `mkdir`/`rmdir`, so it fails from Git Bash. |
| host compiler (`make check` only) | any gcc/cc (WinLibs `gcc` on Windows) | `HOST_CC` overrides. |

Other compilers build **working firmware with different bytes**:
- Arduino's bundled `arm-none-eabi-gcc` 7-2017q4. [`build.ps1`](build.ps1)
  picks it first when it is installed, so do not use `build.ps1` for a build
  you will compare.
- Ubuntu's `gcc-arm-none-eabi`. The `L072 cross-compile` job in
  `arduino-ci.yml` uses it for its link + size-budget gate.

A matching md5 therefore proves both the source and the toolchain.

### Targets

```
mingw32-make            # = all: build/firmware.elf/.bin/.hex + region sizes   (Linux: make)
mingw32-make bench      # build_bench/ with -DHOST_ALLOW_REG_WRITE_DIAG=1, copied to build/firmware_bench_diag.bin
mingw32-make check      # host-compiler unit vectors + memory-map static asserts (no cross toolchain)
mingw32-make size       # region usage against the budget
mingw32-make clean      # removes build/ (including the committed production bin; restore it with `mingw32-make`)
python3 tools/check_size_budget.py build/firmware.elf
```

- `all` makes the **production** image `build/firmware.bin`, which is
  committed (`HOST_ALLOW_REG_WRITE_DIAG=0`). It refuses the bench's
  diagnostic register writes, such as the DTS carrier pin `-ForceFrfHz`.
  Rebuild and commit it with every firmware PR.
- `bench` makes `build/firmware_bench_diag.bin`, the image **both bench
  boards** run. It is untracked: add firmware files to commits by path, never
  with `git add <dir>`.
- `bench` writes into `build/` but does not create it. After a `clean`, run
  `mingw32-make` (or create `build/`) before `mingw32-make bench`.
- Run `check` before flashing anything. Flashing is covered in
  [`../x8_lora_bootloader_helper/FLASH_RUNBOOK.md`](../x8_lora_bootloader_helper/FLASH_RUNBOOK.md).

### Expected md5s

At `main` `bb4a2071` (re-checked 2026-10-04; `firmware/murata_l072` unchanged
since), built with the toolchain above and the default flags:

| image | md5 | size |
|---|---|---|
| `build/firmware_bench_diag.bin` (bench, on both boards since 2026-09-15) | `0c1bb0a9573f813137f941dfa47177d0` | 24 860 B |
| `build/firmware.bin` (production, committed) | `589c120323c2d5e7ef9f459d7a4ba42d` | 24 860 B |

Check with `certutil -hashfile build\firmware_bench_diag.bin MD5`
(PowerShell) or `md5sum build/firmware_bench_diag.bin`.
- Any source change moves both md5s. Record the new pair in the PR.
- Older flashed builds and their md5s are listed in
  [`bench-evidence/RS_13_vector_scene_2026-09-26/firmware/README.md`](../../bench-evidence/RS_13_vector_scene_2026-09-26/firmware/README.md).

### CI

- `arduino-ci.yml` → `L072 cross-compile` (blocking): links with Ubuntu's
  `gcc-arm-none-eabi`. It publishes the ELF/BIN/HEX/MAP artifacts and
  enforces the APP/RAM budgets through
  [tools/check_size_budget.py](tools/check_size_budget.py), using constants
  from [include/memory_map.h](include/memory_map.h). Its bytes differ from
  the bench build by design.
- `l072-bench-md5.yml` → `L072 bench md5 (Arm 12.2.MPACBTI-Rel1)`
  (**non-blocking**): downloads the exact Arm release above for Linux, runs
  `make bench` and `make`, and prints both md5s against this table and
  against the committed `build/firmware.bin`, with a warning on mismatch.
  The Windows bench PC builds are the reference. Whether a Linux build of
  the same release is byte-identical is what this job tests; it has not been
  checked by hand.

## Not yet present

Per the bring-up roadmap [DESIGN-LORAFIRMWARE/03](../../DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md), the following remain for later increments:

- v25 protocol state machine (`proto/`) and high-level application behavior (`app/`).
- Full radio mode sequencing (CAD/LBT scheduling, TX/RX transaction control, retries, and dwell-time enforcement).
- CFG persistence manager and watchdog policy integration.
- A/B slot select code (`boot/slot_select.c`) — Phase 6 only; do not stub now (per Claude review §2.1).
- Golden-jump helper (`boot/golden_jump.c`) — Phase 6.

The current code establishes the bare-metal transport and radio control substrate so protocol and policy layers can be added incrementally.

## Bench bring-up

- Runbook: [BRINGUP_MAX_CARRIER.md](BRINGUP_MAX_CARRIER.md)
- OpenOCD configs: [openocd/stlink.cfg](openocd/stlink.cfg), [openocd/stm32l0_swd.cfg](openocd/stm32l0_swd.cfg)
