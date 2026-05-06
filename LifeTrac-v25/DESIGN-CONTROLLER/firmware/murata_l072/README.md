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

## Build

```
make            # build firmware.elf, firmware.bin, firmware.hex
make size       # show region usage against the budget
python3 tools/check_size_budget.py build/firmware.elf
make clean
```

A real toolchain (`arm-none-eabi-gcc`) is required to link. CI now runs a full cross-compile gate on pinned `ubuntu-24.04` with Ubuntu's `gcc-arm-none-eabi` package, publishes the ELF/BIN/HEX/MAP artifacts, and enforces APP/RAM budgets via [tools/check_size_budget.py](tools/check_size_budget.py) using constants from [include/memory_map.h](include/memory_map.h).

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
