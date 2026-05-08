# Portenta X8 + Max Carrier Debug Reframe - Internal OpenOCD Is the Real SWD Path

- **Date:** 2026-05-07
- **Author:** Copilot (Claude Opus 4.7)
- **Status:** Corrected root cause + actionable recovery path
- **Scope:** Supersedes the overly strong parts of `2026-05-07_Portenta_X8_SWD_RootCause_Wrong_DAP_Copilot_v1_0.md`.

## TL;DR

Yes, we were missing a board-architecture detail: **Portenta X8 on Max Carrier is not wired/debugged like Portenta H7 on Max Carrier**.

The Max Carrier onboard J-Link/Black Magic debug path is documented as supported for Portenta H7/C33, but **not for Portenta X8**. On X8, Arduino sketches are uploaded to the STM32H747 **M4** through ADB and an OpenOCD service running inside Linux on the i.MX8M. The X8's own Linux side bit-bangs SWD into the H747 through i.MX GPIOs.

The fix is therefore not another PA4/PC0 mux polarity sweep. The fix is to treat the X8 Linux side as the debug probe.

## Key evidence found today

### 1. Official upload service uses internal OpenOCD

`monitor-m4-elf-file.service` on the X8 programs user sketches with:

```ini
ExecStart=/bin/openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -c "program /tmp/arduino/m4-user-sketch.elf verify reset exit"
```

The OpenOCD config is explicit about the actual SWD wiring:

```tcl
adapter driver imx_gpio
imx_gpio_peripheral_base 0x30200000
imx_gpio_swd_nums 15 8
imx_gpio_srst_num 10
reset_config srst_only srst_push_pull
transport select swd
source [find target/stm32h7x_dual_bank.cfg]
init
targets
reset halt
```

So the supported X8 SWD path is:

```text
Windows/Arduino CLI -> ADB -> X8 Linux -> OpenOCD imx_gpio -> STM32H747 SWD
```

not:

```text
Windows -> Max Carrier onboard J-Link -> STM32H747 SWD
```

### 2. Internal OpenOCD reaches the real H747

This command succeeded:

```powershell
adb shell "echo fio | sudo -S openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -c shutdown"
```

Important output:

```text
Info : SWD DPIDR 0x6ba02477
Info : stm32h7x.cpu0: hardware has 8 breakpoints, 4 watchpoints
Info : starting gdb server for stm32h7x.cpu0 on 3333
target halted due to debug-request
pc: 0x1ff09abc
```

This is the smoking gun correction. The H747 path returns `0x6ba02477`. The Max Carrier J-Link path returned `0x5ba02477` / `0x5ba00477`, which means it was not reaching this same target path.

The prior note was directionally right that the J-Link path was wrong for this job, but too specific about the exact chip it likely reached. The stronger conclusion is architectural: **Max Carrier onboard JTAG debug is not the supported X8 H747 path**.

### 3. Dual-core OpenOCD can expose an M4 GDB port, but needs care

OpenOCD's installed `stm32h7x.cfg` supports `DUAL_CORE=1`. This command exposes both targets:

```powershell
adb shell "echo fio | sudo -S openocd -c 'set DUAL_CORE 1' -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -c shutdown"
```

Observed output:

```text
Info : stm32h7x.cpu0: hardware has 8 breakpoints, 4 watchpoints
Info : stm32h7x.cpu1: hardware has 0 breakpoints, 2 watchpoints
Info : starting gdb server for stm32h7x.cpu0 on 3333
Info : starting gdb server for stm32h7x.cpu1 on 3334
Error: timed out while waiting for target halted
TARGET: stm32h7x.cpu1 - Not halted
```

This says the M4/AP3 target is at least visible enough for OpenOCD to create `stm32h7x.cpu1` and a GDB server on port `3334`, but the stock script's `reset halt` sequence is not a clean M4 debug recipe yet.

## What failed and what it means

### PA4 and PC0 sweeps were valuable, but not the fix

PA4 and PC0 firmware controls did not change the external J-Link attach failure. That result now makes sense: the blocking issue was not a sketch-controlled carrier mux state. It was that the external J-Link route is the wrong route for X8 H747 debug.

### Linux ownership was not the external J-Link blocker

Stopping `m4-proxy.service` and unloading most `x8h7_*` modules did not change the external J-Link failure. That was also useful: Linux software ownership was not what made the Max Carrier J-Link path fail. Linux is actually the supported path when using its own internal OpenOCD adapter.

### SEGGER RTT is not recovered yet

OpenOCD on the X8 supports RTT commands:

```text
rtt
rtt server start <port> <channel>
```

The current `x8_uart_route_probe` ELF compiled with:

```text
08100558 T SEGGER_RTT_Init
10002488 B _SEGGER_RTT
```

After re-uploading the sketch, attach-only OpenOCD scans against `0x10002488` through both `cpu0` and experimental `cpu1` still reported:

```text
Info : rtt: Searching for control block 'SEGGER RTT'
Info : rtt: No control block found
rtt: Control block not available
```

So RTT remains a possible future path, but not the immediate recovery path. The likely reasons are one or more of:

- The M4 runtime RAM mapping is not being read through the target/AP we used.
- The stock X8 M4 boot/reset choreography keeps the user sketch out of the state expected by OpenOCD RTT scanning.
- The `_SEGGER_RTT` symbol address from the ELF is not the live address visible through this OpenOCD target view.

For Method G bring-up, do not block on RTT. Use UART sniffing or another Linux-visible telemetry channel first.

## Concrete fix path

### Fix 1 - Stop using Max Carrier J-Link as the X8 H747 debugger

Treat the Max Carrier onboard debugger as unsupported for Portenta X8 H747 debug. It may still be useful for debugger firmware, UART sniffing, or Portenta H7/C33 workflows, but it is not the path that Arduino uses to program X8 sketches.

### Fix 2 - Use X8 Linux OpenOCD as the debug probe

One-shot attach verification:

```powershell
adb shell "echo fio | sudo -S openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -c shutdown"
```

Long-running M7 GDB server:

```powershell
adb forward tcp:3333 tcp:3333
adb shell "echo fio | sudo -S openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg"
```

Experimental dual-core server:

```powershell
adb forward tcp:3333 tcp:3333
adb forward tcp:3334 tcp:3334
adb shell "echo fio | sudo -S openocd -c 'set DUAL_CORE 1' -f /usr/arduino/extra/openocd_script-imx_gpio.cfg"
```

Expected ports:

| Port | Target | Notes |
|---|---|---|
| `3333` | `stm32h7x.cpu0` / M7 | Stable attach verified. This is Arduino infrastructure on X8, not the user sketch core. |
| `3334` | `stm32h7x.cpu1` / M4 | Exposed in dual-core mode, but halt/debug behavior needs a custom attach recipe. |

### Fix 3 - Build a custom attach-only OpenOCD script for M4 debug

The stock Arduino script ends with `reset halt`, which is good for programming but disruptive for attaching to a running M4 sketch. A debug-specific script should reuse the same imx_gpio pin setup but omit `reset halt` until the GDB session asks for it.

Starting point:

```tcl
adapter driver imx_gpio
imx_gpio_peripheral_base 0x30200000
imx_gpio_speed_coeffs 500000 50
imx_gpio_swd_nums 15 8
imx_gpio_srst_num 10
reset_config srst_only srst_push_pull
transport select swd
set DUAL_CORE 1
source [find target/stm32h7x_dual_bank.cfg]
init
targets
```

Then connect GDB to `localhost:3334` through `adb forward`. Expect this to need a few iterations because `cpu1` currently reports `0` hardware breakpoints and did not halt cleanly in the first pass.

### Fix 4 - Use physical telemetry for Method G route discovery

For the immediate LoRa/Murata work, the fastest reliable observability path is still physical UART visibility:

- Use the Max Carrier `UART_SNIFF1..5` features if the onboard debugger firmware exposes them cleanly.
- Use an external USB-UART or logic analyzer on the candidate H7-to-Murata UART lines.
- Keep `Serial3` reserved for the cellular modem and test `Serial2`, `Serial4`, and `Serial5` candidates as previously planned.
- Do not expect Linux `/dev/ttymxc*` nodes to sniff the H747-to-Murata UART unless a schematic trace proves that endpoint is electrically shared.

### Fix 5 - For direct Murata L072 work, bypass H747 debug entirely

Method G ultimately targets the Murata CMWX1ZZABZ-078 internal STM32L072. If direct radio firmware programming/debug is required:

- Populate or tack-wire the CN2 LoRa SWD path if the carrier has it depopulated.
- Use the Murata L072 UART bootloader path with BOOT0/NRST control where possible.
- Keep the H747 as the host/loader/telemetry coordinator rather than forcing H747 J-Link debug to be the central path.

## Recommended next bench move

1. Lock in the conclusion: do not spend more time on PA4/PC0 for external J-Link attach.
2. Use the internal OpenOCD path as the official debug/programming path.
3. Build a custom `x8_openocd_m4_attach.cfg` from the script above and test GDB connection to M4 on port `3334`.
4. In parallel, move Method G route validation back to UART sniffing so LoRa progress is not blocked by M4 debug polish.

## Files touched or validated

- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_uart_route_probe/x8_uart_route_probe.ino` still compiles and uploads through the X8 ADB/OpenOCD route.
- Temporary build artifact used for symbol inspection: `%TEMP%/lifetrac_x8_route_probe_build/x8_uart_route_probe.ino.elf`.
- `_SEGGER_RTT` symbol in that ELF: `0x10002488`.
- X8 OpenOCD config validated on board: `/usr/arduino/extra/openocd_script-imx_gpio.cfg`.
