# Portenta X8 Internal-OpenOCD RTT Implementation — Findings & Architectural Block

**Date**: 2026-05-07
**Author**: Copilot (assistant)
**Source plan**: `2026-05-07_Portenta_X8_Max_Carrier_JTAG_Contention_Bypass_v1_Execution_and_v2_Plan_Copilot_v1_0.md` — Option A + Copilot Addendum on `SEGGER_RTT_SECTION` macro
**Status**: Option A attempted. Discovered a **fundamental architectural conflict** that prevents internal-openocd RTT from working on the Portenta X8 as-shipped. The `SEGGER_RTT_SECTION` macro change is **not the bottleneck**. The board now needs a **power cycle** to recover.

---

## 1. Headline conclusion

**Internal openocd (via i.MX `imx_gpio` driver) cannot read RTT from a running M4 sketch on the Portenta X8.** Every openocd `init` invocation halts the M7 cluster in System Bootloader ROM (PC = `0x1ff09abc`), and the M4 cluster only ever boots if the M7 firmware (`STM32H747AII6_CM7.bin` at `0x08000000`) is *running* and explicitly sets `RCC_GCR.BOOT_C2 = 1`. Openocd never lets M7 reach that firmware on a warm reset, so M4 stays in `external reset` state and cannot run the RTT-emitting sketch.

This is a chicken-and-egg constraint:

- To **read** RTT → must `openocd init` → halts M7 → kills M4 boot.
- For **M4 to write** RTT → M7 firmware must be running → openocd must NOT be attached.

These two states are mutually exclusive on the X8 with the current `imx_gpio` openocd config. The `SEGGER_RTT_SECTION` macro from the plan addendum would only help if M4 were actually executing — it isn't.

## 2. Empirical evidence that established the conclusion

### 2.1. M4 boot state probe (dual-core mode)

```
init
targets stm32h7x.cpu1
halt
```

Output:

```
Info : stm32h7x.cpu0: hardware has 8 breakpoints, 4 watchpoints
Info : stm32h7x.cpu1: hardware has 0 breakpoints, 2 watchpoints
Info : stm32h7x.cpu1: external reset detected         <-- M4 is held in reset
target halted due to debug-request, current mode: Thread
xPSR: 0x01000000 pc: 0x1ff09abc msp: 0x24003af8       <-- M7 in bootloader
Error: timed out while waiting for target halted
TARGET: stm32h7x.cpu1 - Not halted                    <-- M4 cannot be halted
```

`external reset detected` on cpu1 is openocd's way of saying the M4 core is currently asserted to RESET via the H747's intra-cluster reset signal, which is controlled by the M7 firmware writing `RCC_GCR.BOOT_C2`. Since M7 is halted in bootloader (never executing the firmware), M4 stays in reset.

### 2.2. Stale vector table proves M4 has not re-executed

After flashing a freshly-built ELF (Reset_Handler at `0x08107b48` per `arm-none-eabi-nm`), reading the M4 SRAM-relocated vector table at `0x30000000` (M7-side AHB alias of D2 SRAM) showed:

```
FLASH 0x08100000:  10048000 08107bc5 08107c11 0810425d ...   <-- new firmware
SRAM  0x30000000:  10048000 08107901 0810794d 08103f99 ...   <-- old firmware
SRAM  0x10000000:  10048000 08107901 0810794d 08103f99 ...   <-- same as 0x30000000 (alias confirmed)
```

The SRAM still holds vectors from a *previous* M4 boot (last time M7 firmware ran from cold start). M4 has not re-executed Reset_Handler since flashing.

### 2.3. `_SEGGER_RTT` region untouched

`arm-none-eabi-nm` reports `_SEGGER_RTT @ 0x10002488 (B)` (i.e. in `.bss`). `__bss_start__ = 0x100010c0`, `__bss_end__ = 0x1000ace0`, so this address IS in the bss region that Reset_Handler is supposed to zero.

Read from M7 view at the AHB alias `0x30002488`:

```
RTT_M7view = cccccccc cccccccc cccccccc cccccccc ...
RTT_bytes  = cc cc cc cc cc cc cc cc cc cc cc cc cc cc cc cc
```

`0xCC` is mbed's stack/uninitialized fill pattern. Bss should be `0x00` after Reset_Handler. The fact that this region is still `0xCC` is direct proof that **M4's Reset_Handler did not execute the bss-zeroing loop**, consistent with M4 being in external reset.

### 2.4. D2 SRAM dual-mapping verified (the addendum hypothesis was correct in principle)

For the record, the plan addendum's underlying memory-map claim is correct:

- M4 view `0x10000000` and M7 view `0x30000000` map to the **same physical D2 SRAM bytes**.
- Verified by reading both addresses from the M7-anchored openocd: identical content (`10048000 08107901 ...`).
- So `_SEGGER_RTT @ 0x10002488 (M4)` would correspond to physical address visible at `0x30002488 (M7)` — *if* M4 ever ran to populate it.
- Therefore the `SEGGER_RTT_SECTION` linker-attribute trick is **unnecessary** on this platform: the existing `RAM_D2 ORIGIN = 0x10000000` already places `_SEGGER_RTT` in shared D2 SRAM. M7 just reads it at the `0x3000XXXX` alias.
- The trick would only matter if a future sketch placed RTT in M4's CCM/TCM (e.g. `RAM_DTCM`), which is M4-private.

## 3. Why every openocd attach kills M4 boot

`/usr/arduino/extra/openocd_script-imx_gpio.cfg` ends with:

```tcl
init
targets
reset halt
```

The `reset halt` does:

1. Assert SRST via i.MX GPIO 10 (low).
2. Release SRST.
3. Halt M7 at first instruction.

On the X8, M7 boots into **System Bootloader ROM** at `0x1FF00000+` (boot option bytes route SRST-triggered cold boot to bootloader, not flash). The bootloader handles ST-Link / DFU / x8h7 SPI handshake. The actual `STM32H747AII6_CM7.bin` firmware at `0x08000000` is only entered if the bootloader hands off to it after the host driver acknowledges connection — and openocd halts before that handshake completes.

Subsequent openocd `exit` releases SRST GPIO 10 to input but does **not** trigger another reset. M7 just sits halted at the same PC after openocd lets go. M4 stays in reset because nothing wrote `RCC_GCR.BOOT_C2`.

Result: even after `program ... verify reset exit` succeeds, the system is left in a non-running state until a true power cycle.

## 4. Side effect: x8h7 SPI bridge stalls

Because M7 firmware was repeatedly halted across our experiments, the kernel's `x8h7_drv` SPI handshake to M7 firmware timed out:

```
$ cat /sys/kernel/x8h7_firmware/version
cat: /sys/kernel/x8h7_firmware/version: Connection timed out
```

`x8h7_*` modules are still loaded with stale handles to gpiochip5 and cannot be cleanly unloaded:

```
$ rmmod x8h7_gpio
rmmod: ERROR: Module x8h7_gpio is in use
$ rmmod x8h7_drv
rmmod: ERROR: Module x8h7_drv is in use by: x8h7_gpio
```

`systemctl restart stm32h7-program.service` fails because `load_modules_pre.sh` does `insmod` (not `modprobe`) and the modules are already loaded.

**Recovery requires a power cycle of the Portenta X8** (unplug both USB-C and the 12 V Max Carrier barrel). Software-only recovery is blocked until a fresh boot.

## 5. Implications for Method G LoRa bring-up

Two paths remain viable; neither uses the internal openocd RTT route:

### 5.1. UART telemetry via x8h7 SPI bridge — **CORRECTION (post-power-cycle empirical)**

**Initial recommendation was wrong.** Post-power-cycle probing (gpiochip enumeration + driver introspection) shows:

```
/sys/class/gpio/gpiochip0   label=30200000.gpio  base=0   ngpio=32   <-- i.MX SoC GPIO bank
/sys/class/gpio/gpiochip32  label=30210000.gpio  base=32  ngpio=32   <-- i.MX SoC GPIO bank
/sys/class/gpio/gpiochip64  label=30220000.gpio  base=64  ngpio=32   <-- i.MX SoC GPIO bank
/sys/class/gpio/gpiochip96  label=30230000.gpio  base=96  ngpio=32   <-- i.MX SoC GPIO bank
/sys/class/gpio/gpiochip128 label=30240000.gpio  base=128 ngpio=32   <-- i.MX SoC GPIO bank
/sys/class/gpio/gpiochip160 label=x8h7_gpio      base=160 ngpio=34   <-- H7-bridged GPIO

/sys/class/tty/ttymxc1/device/driver -> imx-uart
/sys/class/tty/ttymxc2/device/driver -> imx-uart
/sys/class/tty/ttymxc3/device/driver -> imx-uart   <-- DIRECT i.MX UART, not H7-bridged

dmesg: "Serial: X8H7 UART driver" + "x8h7uart: ttyX0 at I/O 0x0"  <-- separate ttyX0 device
```

**Therefore on the Portenta X8 + Max Carrier:**

- `/dev/ttymxc1`, `/dev/ttymxc2`, `/dev/ttymxc3` are **direct i.MX-internal UARTs** (`imx-uart` driver). They are NOT bridged through the H7. They reach physical UART pins on the carrier via the i.MX SoC's own pin mux.
- `/dev/ttymxc3` specifically is wired directly to the **Murata CMWX1ZZABZ-078 LoRa modem's L072 UART** at 19200 8N1 — this is documented in the `2026-05-04_Murata_UART_Firmware_Probe_Bench_Results` note and re-confirmed in this session.
- `/dev/ttyX0` (provided by the `x8h7_uart` kernel module) is the H7-bridged UART, but the H7 firmware does not by default forward arbitrary M4 `Serial1` writes to it. There is no generic `Serial1 → Linux` bridge.

**Implication for the route-probe sketch:** running `cat /dev/ttymxc3` will NOT show `LT_ROUTE_PROBE` output. The `Serial1` pin from M4 lands on a physical pin on the Max Carrier breakout — capture would require an external USB-UART or oscilloscope on those breakout pins. Use this only if external instrumentation is available.

### 5.1b. The unexpected upside — direct L072 access for Method G

Because `/dev/ttymxc3` is direct (no H7 in the path), Method G's L072-flashing workflow can be implemented **entirely on the Linux side** without ever touching the H7's `x8h7` bridge firmware:

- UART transport: `/dev/ttymxc3` at 19200 8N1 (or 8E1 for STM32 ROM bootloader).
- LoRa NRST: `gpio163` (x8h7_gpio chip base 160, line 3) — verified working this session: pulsing it from Linux did reset the modem.
- LoRa BOOT0: TBD (need to find which `x8h7_gpio` line. Likely also under base 160, see Max Carrier schematic / device tree).
- Linux toolchain: `stm32flash` (not currently installed on /usr/bin) OR a small Python script implementing STM32 ROM bootloader protocol.

**Direct Phase-0 verification this session:**
```
stty -F /dev/ttymxc3 19200 raw -echo cs8 -parenb -cstopb
echo 0 > /sys/class/gpio/gpio163/value; sleep 0.2; echo 1 > /sys/class/gpio/gpio163/value
printf 'AT+VER?\r\n' > /dev/ttymxc3
# response: "Error when receiving\n+ERR_RX\r" (repeated)
```
This is the exact stale-stock-firmware symptom documented in `2026-05-04_Murata_UART_Firmware_Probe_Bench_Results`. So the X8 + Max Carrier IS at the documented Phase-0 entry baseline, and the L072 IS reachable directly from Linux.

### 5.1c. Why the original `MKRWANFWUpdate_standalone` cannot just be uploaded

The sketch is designed for Portenta H7, where the H7 IS the application processor and `Serial` = USB-CDC to a host PC. On Portenta X8:

- The H7 does NOT own USB-C (i.MX does). So the sketch's `while (!Serial)` would block forever waiting for a serial monitor that doesn't exist on this hardware.
- Uploading `arduino:mbed_portenta:envie_m7` firmware to the X8 H7 would replace the `x8h7` bridge firmware → break Linux-side x8h7 modules. Recoverable via `systemctl restart stm32h7-program.service` but **destructive** to a working bench.
- Sketch compiled cleanly this session for `envie_m7` + `-DPORTENTA_CARRIER` (artifacts in `%TEMP%\mkrwan_fwupdate_build\`) but **was NOT uploaded** for the above reason.

The X8-native equivalent of the sketch is a Linux-side script that:
1. Drives `gpio163` (NRST) and the BOOT0 GPIO over `x8h7_gpio` chip 160.
2. Speaks STM32 ROM bootloader protocol over `/dev/ttymxc3` at 8E1.
3. Uses the embedded `mlm32l07x01.bin` from `MKRWAN/examples/MKRWANFWUpdate_standalone/fw.h` as the payload.

### 5.2. Move LoRa firmware off the X8 entirely

The Method G plan flashes the **Murata L072** SiP-internal MCU, not the H747. Once that firmware is loaded via the Phase-0 `MKRWANFWUpdate_standalone` bootloader pipeline (no SWD needed), the X8 H747 sketch only acts as a UART transport between Linux and the L072. RTT on the H747 becomes unnecessary; we just observe the L072's UART traffic on `/dev/ttymxc3`.

This is the cleanest path forward and bypasses the internal-openocd RTT impossibility entirely.

## 6. Action items (revised after post-power-cycle empirical pass)

1. **Power-cycle the X8** — DONE this session. Modules reload cleanly via `stm32h7-program.service`; `m4-proxy.service` and `monitor-m4-elf-file.path` come up active. Note: `/sys/kernel/x8h7_firmware/version` still times out, but other x8h7 modules (CAN/GPIO/RTC/UART) initialize per dmesg, so the bridge is functional — that one sysfs entry appears broken in the current OE image.
2. **Do not attach internal openocd to a running M4 sketch.** Any `openocd -f .../openocd_script-imx_gpio.cfg` invocation halts M7 and kills M4 boot.
3. **Cancel the `cat /dev/ttymxc3` telemetry plan.** That UART is wired to the Murata L072, not to H7 `Serial1`. To capture M4 sketch UART output on the X8, use external USB-UART probes on the Max Carrier breakout pins for the relevant `Serial*` interfaces.
4. **Skip the `SEGGER_RTT_SECTION` macro.** The default linker placement already puts `_SEGGER_RTT` in shared D2 SRAM — repositioning is unnecessary on the X8.
5. **Method G Phase 0 — pivot to Linux-native L072 flashing.** Do NOT upload `MKRWANFWUpdate_standalone` to the X8 H7 (would destructively replace `x8h7` bridge). Instead:
   - Identify Max Carrier BOOT0 GPIO line on `x8h7_gpio` chip (base 160) from device tree / schematic.
   - Either install `stm32flash` on the X8 Linux side or write a small Python script in `tools/` implementing STM32 ROM bootloader protocol over `/dev/ttymxc3` at 8E1.
   - Use `mlm32l07x01.bin` extracted from `MKRWAN/examples/MKRWANFWUpdate_standalone/fw.h` as the firmware payload (in user's Arduino libraries directory).
   - This entirely bypasses the H7 SWD/RTT/x8h7-vs-debug conflict family of problems.
6. **Phase-0 entry baseline confirmed this session:** `printf 'AT+VER?\r\n' > /dev/ttymxc3` returns the documented stale-firmware `+ERR_RX` symptom, proving the L072 is reachable and at the same starting state as the 2026-05-04 bench.

## 6b. BOOT0 sweep result — PG_7 is NOT exposed via x8h7_gpio (2026-05-07 empirical)

A collaborator hypothesis (`LORA_BOOT0 = PG_7`, `LORA_RESET = PC_7`, with linear port mapping suggesting `gpio227 = 163 + 64`) was tested and **disproven**:

- `x8h7_gpio` chip has `base=160 ngpio=34`, so the valid Linux GPIO range is 160–193. `gpio227` does not exist (export fails: `No such file or directory`).
- Exhaustive sweep of all 33 lines (161–193, skipping 160 which export-rejects and 163 which is NRST) using a discriminating control test:
  - For each candidate L: hold L=HIGH, pulse NRST, send `AT\r\n` at 19200 8N1, capture response.
  - Baseline (no BOOT0 manipulation): 29 bytes = `"Error when receiving\n+ERR_RX\r"`.
  - All 33 candidate lines: identical 29-byte AT-firmware response → modem booted into AT firmware in every case → BOOT0 was never asserted high.
- **Conclusion:** the H7 `x8h7` bridge firmware does NOT export PG_7 (or whichever physical net carries `LORA_BOOT0`) to Linux. There is no Linux-side GPIO that controls the Murata BOOT0 line through the `x8h7_gpio` driver.

### Implications and viable Phase-0 paths

Linux-side `stm32flash` on `/dev/ttymxc3` is **not viable as-is** because we cannot assert BOOT0 from Linux. Three remaining options:

**Option P0-A — M4 sketch as BOOT0 helper (recommended).** Write a minimal M4 sketch (`x8_lora_bootloader_helper.ino`) that runs on the H7 M4 cluster (which the X8 supports) and:
  1. On boot: drives `PG_7` HIGH via direct `GPIOG->BSRR` write (bypassing the M7 firmware's GPIO ownership) OR via the Arduino mbed `digitalWrite(PG_7, HIGH)` if the variant exposes it.
  2. Pulses `LORA_RESET` (PC_7) low for 50 ms then high.
  3. Releases `PG_7` LOW after a 200 ms hold (or keeps high until told to release via a UART command from Linux).
  4. Optionally proxies `/dev/ttymxc3` traffic by simply being passive once setup is done.
  
  This sketch is uploaded via the existing `monitor-m4-elf-file.path` pipeline (works flawlessly, used multiple times this session). Linux side then runs `stm32flash` against `/dev/ttymxc3`. **No H7 firmware modification needed.**

**Option P0-B — Update H7 bridge firmware to export PG_7.** Edit the open-source `x8h7-firmware` repo to add PG_7 to the GPIO export table, rebuild, flash via `stm32h7-program.service`. More invasive, requires understanding the bridge firmware's GPIO export mechanism, and risks bricking the bridge.

**Option P0-C — Use original `MKRWANFWUpdate_standalone` on a separate H7-only Portenta (not X8).** Not applicable: bench has only X8 hardware available.

**Recommendation:** pursue **Option P0-A** next. Verify the M4 sketch can drive `PG_7` (need to check if `arduino:mbed_portenta:portenta_x8` variant exposes it, or use raw register access).

## 7. Files touched during this investigation

- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_uart_route_probe/x8_uart_route_probe.ino` — temporarily added DBGMCU.CR debug-in-sleep write + busy-wait loop (both since reverted; see edit history). Final state matches pre-Option-A state (PC_0 + PA_4 deterministic init, `delay(1000)` loop).
- `LifeTrac-v25/AI NOTES/2026-05-07_Portenta_X8_Max_Carrier_JTAG_Contention_Bypass_v1_Execution_and_v2_Plan_Copilot_v1_0.md` — earlier execution note. The Copilot Addendum on `SEGGER_RTT_SECTION` is correct in mechanism but **does not address the root blocker**, which is M7-bootloader-vs-M4-boot conflict.

## 8. What an updated Option A would have to look like

For internal openocd RTT to ever work on the X8, openocd's session would need to:

1. Connect WITHOUT issuing `reset halt` (use `noinit` mode + manual `dap apreg` reads).
2. Read the RTT control block over SWD purely via DAP/AHB-AP without halting M7.
3. Never assert SRST via GPIO 10 (the standard openocd cfg unconditionally does).

This requires authoring a **custom openocd TCL script** that bypasses `init` / `reset halt` / `program` and only does AHB-AP memory reads on the M7 cluster (which is enough to reach D2 SRAM where the M4's RTT block lives). In that mode, M7 firmware continues running, M4 continues running, and we just snoop the SRAM.

That custom script is the **only viable extension of Option A** and is more complex than the simpler `/dev/ttymxc3` UART telemetry route. Recommend pursuing UART telemetry first, and only returning to a custom non-halting openocd RTT reader if UART throughput proves insufficient.
