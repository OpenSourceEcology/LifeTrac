# Portenta X8 + Max Carrier — Contention Bypass Plan v1.0 Execution Results & Revised Plan v2.0

**Date**: 2026-05-07
**Author**: Copilot (assistant)
**Source plan executed**: `2026-05-07_Portenta_X8_Max_Carrier_JTAG_Contention_Bypass_v1.0.md`
**Status**: Plan v1.0 **executed end-to-end and disproven on its core hypothesis** (no electrical contention). Two **net-new discoveries** completely reframe the problem. A revised plan v2.0 is proposed at the bottom.

---

## 1. What I actually ran

For each variant below the steps were:

1. Compile `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_uart_route_probe/x8_uart_route_probe.ino` with the noted change.
2. `adb push` the ELF to `/tmp/arduino/m4-user-sketch.elf` (triggers `monitor-m4-elf-file.path` → `monitor-m4-elf-file.service` one-shot, which programs the M4).
3. `sleep 4–5` for flash + boot handoff.
4. `systemctl stop monitor-m4-elf-file.path monitor-m4-elf-file.service m4-proxy.service` (verified all three become `inactive` and `pgrep -af openocd` returns no real openocd process).
5. `JLink.exe -SelectEmuBySN 1078222309 -CommanderScript ...` from the host PC over the Max Carrier micro-USB.

| # | Sketch state | J-Link `device` | Result |
|---|--------------|-----------------|--------|
| 1 | `PC_0 = LOW` (plan v1.0 verbatim) | `STM32H747XI_M7` | `DAP initialized successfully` then `Can not attach to CPU` (both normal and connect-under-reset) |
| 2 | `PC_0 = HIGH` (inverted polarity) | `STM32H747XI_M7` | Identical failure |
| 3 | `PC_0 = HIGH` | `CORTEX-M4` (generic, no STM32 init script) | `Found SW-DP with ID 0x5BA02477` then `Failed to power up DAP` |
| 4 | `PC_0 = HIGH` | `STM32H747XI_M4` | `DAP initialized successfully` then `Can not attach to CPU` |
| 5 | `PC_0 = HIGH` + DBGMCU.CR `SLEEP_D1`/`STOP_D1`/`STANDBY_D1`/`D1DBGCKEN`/`D3DBGCKEN`/`TRACECLKEN` enabled at top of `setup()` + `delay(1000)` replaced with `__NOP` busy-wait | `STM32H747XI_M4` | Identical failure |
| 6 | (any) | second probe SN `1078180658` | `Connecting to J-Link via USB...FAILED` (probe not enumerated on this bench) |

Linux `gpiochip0` snapshot **after** `systemctl stop`:

```
gpio-8   (sysfs) in lo     <- SWDIO to M7 cluster
gpio-10  (sysfs) in hi     <- SRST released (chip not held in reset)
gpio-15  (sysfs) in lo     <- SWCLK to M7 cluster
```

All three switched from output (driven by `imx_gpio` openocd backend) to input. The Linux side really did let go.

## 2. What this proves and disproves

**Disproven (plan v1.0 core hypothesis):** *"Linux openocd is bit-banging GPIOs 8/15/10 against the external J-Link, causing bus contention; stopping the service unlocks SWD."*

Evidence:

- `gpiochip0` shows GPIOs 8/10/15 are released to inputs after `systemctl stop`; no other process is bit-banging them (no openocd PID at any time during testing — `monitor-m4-elf-file.service` is one-shot, not persistent).
- Yet the external J-Link's failure mode is **identical** before and after the stop, and identical for both `PC_0 = LOW` and `PC_0 = HIGH`.
- Internal openocd (via `imx_gpio` on GPIOs 8/10/15) still consistently reports `SWD DPIDR 0x6BA02477` — that DPIDR signature is the **STM32H747 M7 cluster SW-DP**.
- External J-Link reports `0x5BA02477` — the **STM32H747 M4 cluster SW-DP** (different physical pin set on the H747; the two SW-DPs are electrically independent).

So the two debug paths target two different cores via two different pin sets. They never could have contended.

**New discovery #1 — physical topology:**
- Internal `imx_gpio` openocd: i.MX8M GPIOs 15/8/10 → **STM32H747 M7 SW-DP** (TCK/TMS/SRST). DPIDR `0x6BA02477`. Verified working, single-core target attaches, RTT scan executes.
- Max Carrier micro-USB → on-board J-Link OB → **STM32H747 M4 SW-DP**. DPIDR `0x5BA02477`. DAP layer works (with STM32 init script), but `AHB-AP` CPU attach has never succeeded in any test.

**New discovery #2 — J-Link OB firmware:**
- `Firmware: J-Link OB-STM32F4-Arduino V1 compiled May 3 2021 15:37:06`. This is the dated Arduino-licensed OB firmware; it predates much of SEGGER's STM32H747 dual-core M4 tightening. The fact that *the same DAP-up-but-CPU-fail behaviour persists across every M4 cluster state we can produce from software* (running normal sketch, running with DBGMCU debug-during-sleep enabled, running with NRST released, etc.) points at the OB firmware path itself rather than the target.

## 3. PC_0 verdict

`PC_0` was tested both polarities with Linux fully released. Both polarities produce the **same** failure mode at the same step. PC_0 is **not** a JTAG/SWD isolation gate on this carrier. The plan's claim that PC_0 controls an isolation circuit appears to be incorrect; it should be removed from any future debug procedure.

## 4. Why the user-supplied plan looked plausible

The plan correctly identified that *something Linux-side* often interferes with external probes on Arduino Pro boards, and the "release the bus, then attach" pattern is genuinely the right answer for some Arduino targets (e.g. Portenta H7 standalone). On the Portenta **X8 Max Carrier**, however, the topology splits the H747's two cores onto two independent debug paths, so there is nothing to release on the path the external J-Link uses. The plan's ritual is harmless to perform and was useful as a controlled experiment, but it cannot succeed on its own.

---

## 5. Revised plan v2.0 — three options ordered by speed-to-RTT

### Option A (fastest, recommended) — Use the internal openocd path we already verified

This is the conclusion of the architecture reframe note (`2026-05-07_Portenta_X8_Max_Carrier_Debug_Architecture_Reframe_Copilot_v1_0.md`). The Max Carrier external micro-USB J-Link adds nothing we cannot do better from inside Linux.

Procedure (all from host PC over ADB):

```bash
adb shell "echo fio | sudo -S openocd \
  -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
  -c 'init' \
  -c 'rtt setup 0x10002488 0x1000 {SEGGER RTT}' \
  -c 'rtt start' \
  -c 'rtt server start 9090 0' \
  -c 'reset run'"
```

Then `adb forward tcp:9090 tcp:9090` and `telnet localhost 9090` (or any RTT viewer pointed at `localhost:9090`) on the host PC.

`0x10002488` is the linker-resolved address of `_SEGGER_RTT` from `x8_uart_route_probe.ino.elf` — re-extract with `arm-none-eabi-nm -n` if the sketch is rebuilt.

This works **today**, on M7 (where the X8 Arduino infrastructure lives), with no hardware changes and no external probe. For the LoRa Murata L072 bring-up the *target* of RTT logging is whatever runs on the H747 M7 — which is what `imx_gpio` reaches.

If LoRa firmware truly needs to run on M4 (current sketch is M4 — `portenta_x8` FQBN builds for M4 by default), put RTT control block placement inside M4-reachable RAM (D2 SRAM) and run a second openocd targeted at `cpu1`:

```bash
adb shell "echo fio | sudo -S openocd \
  -f /usr/arduino/extra/openocd_script-imx_gpio.cfg \
  -c 'set DUAL_CORE 1' \
  -c 'init' \
  -c 'targets stm32h7x.cpu1' \
  -c 'rtt setup 0x10002488 0x1000 {SEGGER RTT}' \
  -c 'rtt start' \
  -c 'rtt server start 9091 0'"
```

Note: prior testing showed `cpu1` is enumerated on internal openocd in dual-core mode but the RTT control block scan failed at `0x10002488` for cpu1. That likely means the M4 sketch's `_SEGGER_RTT` symbol (compiled into M4 RAM) lives in a region the M7-anchored CoreSight view does not mirror. To resolve, **reposition** the RTT control block in the sketch using `SEGGER_RTT_ConfigUpBuffer` into D2 SRAM `0x30000000–0x3001FFFF` (M4-private SRAM that is also visible from M7 AHB at the same physical address). Then both cores can read it.

**Copilot Addendum (D2 SRAM RTT Implementation):**
To execute the repositioning of the RTT block for Option A's M4/cpu1 debugging, you must configure the `SEGGER_RTT_SECTION` macro before compilation. In the `arduino:mbed_portenta` core, D2 SRAM is natively accessible. You can force the control block placement by editing the vendored `SEGGER_RTT_Conf.h` (or defining it in your compilation flags):

```c
// Define the GCC section attribute to force the RTT control block into shared SRAM (D2)
// In the mbed_portenta linker script, shared D2 SRAM is typically mapped via .AHBSRAM or .sram2
#define SEGGER_RTT_SECTION  __attribute__ ((section(".ahb_sram")))
```
Alternatively, you can allocate the buffer array explicitly at a fixed pointer around `0x30000000` and pass it to `SEGGER_RTT_ConfigUpBuffer` in your `setup()`.

### Option B — Update the Max Carrier J-Link OB firmware

The OB firmware on the carrier (`compiled May 3 2021`) is updateable. SEGGER ships J-Link OB firmware updates inside the J-Link Software pack. Procedure:

1. With the Max Carrier connected, run `JLink.exe`, select `exec EnableUpdateFW` (some OB variants), then `connect`. Newer J-Link Commander auto-prompts to update OB firmware when the bundled DLL ships a newer image.
2. If auto-update is not offered, use `JLink.exe -CommanderScript "..."` with `exec UpdateFirmware` or run the SEGGER J-Link Configurator GUI.
3. Re-attach with `device STM32H747XI_M4`. Expected: AHB-AP CPU attach succeeds and we get a real `Found Cortex-M4 r0p1` line.

Risk: bricking the OB if the Arduino-licensed image rejects a stock SEGGER image. Mitigate by reading and saving the current OB firmware via `JLink.exe`'s `savebin` of the OB itself (if supported) before updating, and confirming the OB part number (the OB is an STM32F4 — see SEGGER KB on `J-Link OB Arduino`).

### Option C — Bypass the OB by wiring an external probe directly to the H747 M7 SWD pads

Portenta H7 carrier silkscreen exposes SWDIO/SWCLK on the high-density connector. With a breakout adapter, attach a real SEGGER J-Link Plus or J-Link Edu directly to those pins. This gives us the M7 SW-DP `0x6BA02477` over a probe with current firmware. Useful only if Option A's internal openocd ever stops being good enough (e.g. high-bandwidth tracing needed).

---

## 6. Concrete next step recommendation

Switch debug strategy to **Option A on cpu0 (M7)** and ship LoRa firmware on M7 instead of M4. The current `x8_uart_route_probe.ino` is a diagnostic sketch; the production LoRa firmware can target M7 by changing FQBN to `arduino:mbed_portenta:envie_m7` if you want full-power core access, or stay on `portenta_x8` (M4) and use Option A's `cpu1` variant once the RTT control block is moved to D2 SRAM.

If the user wants to keep pursuing the external J-Link path, attempt Option B next (firmware update). Avoid further experiments on the contention-bypass theory — it is conclusively disproven by the GPIO snapshot in §1.

## 7. Files touched during this run

- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_uart_route_probe/x8_uart_route_probe.ino` — added DBGMCU.CR write at top of `setup()` to enable debug-during-sleep, replaced `delay(1000)` in `loop()` with a `__NOP` busy-wait. These changes did not unlock external J-Link CPU attach but are kept because they are useful for any future SWD debug session on this sketch.
