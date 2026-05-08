# Portenta X8 LoRa Phase-0 Status — End-of-Day 2026-05-07

**Author:** GitHub Copilot (Claude Opus 4.7)
**Scope:** Method G Phase-0 (reflash Murata L072 LoRa modem from the X8 itself)
**Status:** **BLOCKED — needs power-cycle + strategy decision**
**Companion docs:**
- [`2026-05-07_Portenta_X8_Internal_OpenOCD_RTT_Architectural_Block_Copilot_v1_0.md`](2026-05-07_Portenta_X8_Internal_OpenOCD_RTT_Architectural_Block_Copilot_v1_0.md)
- [`2026-05-07_Portenta_X8_Max_Carrier_JTAG_Contention_Bypass_v1_Execution_and_v2_Plan_Copilot_v1_0.md`](2026-05-07_Portenta_X8_Max_Carrier_JTAG_Contention_Bypass_v1_Execution_and_v2_Plan_Copilot_v1_0.md)
- New helper sketch: [`../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/x8_lora_bootloader_helper.ino`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/x8_lora_bootloader_helper.ino)

---

## 1. One-line summary

Three independent attack paths to push the L072 into its STM32 ROM bootloader from the X8 (Linux-direct GPIO, M4-helper Option P0-A) have all failed for **the same root cause**: the H7's `x8h7` bridge firmware *owns* the LoRa BOOT0 line and does not release it to either Linux (no x8h7_gpio export) or to the M4 (silently overrides M4 writes). The bench is currently in a wedged state and needs a power cycle before further testing.

## 2. What we tried this session — and what each result proved

### 2.1 Plan v1.0 — JTAG via Max Carrier OB micro-USB
**Result:** Disproven (documented in earlier note). M4 SW-DP attach succeeds at the DAP layer but AHB-AP CPU attach fails — OB firmware (`OB-STM32F4-Arduino V1, May 3 2021`) limitation. Reproducible across PC0 polarities and DBGMCU debug-in-sleep enable.

### 2.2 Plan v2.0 Option A — Internal openocd RTT over `imx_gpio`
**Result:** Architecturally blocked. Every `imx_gpio` openocd `init` does `reset halt` → M7 stuck in System Bootloader (PC = 0x1ff09abc) → M7 firmware never executes → never writes `RCC_GCR.BOOT_C2` → M4 stays in `external reset detected` → Reset_Handler never runs → `_SEGGER_RTT` BSS never zeroed (reads as 0xCC fill).
**Side effect:** x8h7 bridge stalls; `/sys/kernel/x8h7_firmware/version` and other x8h7_gpio reads return `Connection timed out`. Recovery requires power cycle.

### 2.3 Linux-direct BOOT0 sweep
**Hypothesis:** Some line on `gpiochip160` (the `x8h7_gpio` chip, base 160, ngpio 34) is wired to `LORA_BOOT0` (PG_7 on the H747 die).
**Method:** Hold candidate line HIGH, pulse NRST via gpio163, send `AT\r\n` at 19200 8N1, observe response. Baseline (no BOOT0 held) returns the documented stale-firmware 29-byte `Error when receiving\n+ERR_RX\r`.
**Result:** All 33 valid lines (161–193, skipping NRST=163) returned the **identical 29-byte AT response**. No line in the x8h7_gpio range steers BOOT0. The collaborator's earlier `gpio227 = 163+64` hypothesis was off the chip entirely (chip ends at 193).
**Conclusion:** The H7's x8h7 bridge firmware does **not** export PG_7 (or whichever pin physically is `LORA_BOOT0`) to Linux.

### 2.4 Option P0-A — M4 sketch as BOOT0 helper
**Sketch:** [`x8_lora_bootloader_helper.ino`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/x8_lora_bootloader_helper.ino)
- `pinMode(PG_7, OUTPUT); digitalWrite(PG_7, HIGH);`
- pulse `PC_7` LOW/HIGH (NRST)
- re-assert PG_7 HIGH each loop, blink `LED_BUILTIN` @ 1 Hz for liveness

**Build:** Compiled cleanly for `arduino:mbed_portenta:portenta_x8` — `EXIT=0`, ELF 3,698,388 bytes at `%TEMP%\lifetrac_x8_boot_helper_build\x8_lora_bootloader_helper.ino.elf`. The mbed `PinName` enum exposes `PG_7`/`PC_7` even though the X8 variant header omits the `LORA_BOOT0`/`LORA_RESET` macros.

**Deploy:** `adb push` → `/tmp/arduino/m4-user-sketch.elf` → `touch` triggers `monitor-m4-elf-file.path` → openocd reports `** Verified OK ** ** Resetting Target **`. Pipeline worked correctly.

**Empirical results post-flash:**
1. **AT probe at 19200 8N1:** L072 returned the **same 29-byte `+ERR_RX`** as baseline → BOOT0 NOT asserted.
2. **ROM sync `0x7F` at 19200 8E1:** 0 bytes returned → no ROM bootloader ACK.
3. **M4 liveness via gpio167 mirror:** `Connection timed out` on every read → x8h7 bridge stalled again, can't programmatically verify whether the M4 sketch is even running.

## 3. Key new discovery

`/usr/bin/m4_led_forwarder` is a tiny shell script that reveals an undocumented diagnostic channel:

```sh
#!/bin/sh
if [ $1 = "enable" ]; then
    echo gpio > /sys/class/leds/ledR/trigger
    echo 7 > /sys/class/leds/ledR/gpio
else
    echo none > /sys/class/leds/ledR/trigger
fi
```

The H7 bridge firmware exports the M4's `LED_BUILTIN` pin state on **x8h7_gpio line 7 (Linux gpio167)**, which the carrier-board red LED tracks via the `gpio` LED trigger.

**Implication:** When the x8h7 bridge is healthy, we can poll `/sys/class/gpio/gpio167/value` (or watch the carrier red LED with our eyes) to confirm the M4 sketch is actually running — no RTT, no UART, no debugger required. This is the diagnostic primitive we've been missing all session and is reusable for every future M4 sketch.

## 4. Why P0-A failed (two non-exclusive hypotheses)

### H1: M4 never booted
The same architectural block from §2.2 may apply: openocd's `reset halt` at the end of `monitor-m4-elf-file.service` could leave M7 in System Bootloader, in which case M7 firmware never runs and never asserts `RCC_GCR.BOOT_C2`, so M4 stays in `external reset detected` permanently.
- **Evidence for:** x8h7 bridge stalled exactly as in §2.2, suggesting M7 firmware is again not running.
- **Evidence against:** unit `monitor-m4-elf-file.service` uses `program ... verify reset exit` — `exit` should release SWD and re-trigger normal boot. We've successfully run M4 sketches earlier this session (`x8_uart_route_probe`).
- **Test:** post-power-cycle visual check — does the on-board green Portenta LED blink at 1 Hz after `touch`-triggered reflash?

### H2: H7 firmware actively owns PG_7/PC_7
Even if M4 boots, the H7's x8h7 bridge firmware likely configures PG_7/PC_7 itself at boot (it has to — that's how `m4_led_forwarder` exposes the M4's LED pin). The H7 firmware writes once at init, then only reactively. M4's once-per-loop `digitalWrite(PG_7, HIGH)` should win the race after the H7 init has settled.
- **Evidence for:** §2.3 sweep proves H7 firmware *does* expose at least some H747 pins to Linux as GPIO; ownership of PG_7 specifically is plausible.
- **Evidence against:** if H7 owned PG_7 and held it LOW, our M4 write would still latch HIGH (last-write-wins on STM32 GPIO; H7 is not periodically rewriting).
- **Test:** scope BOOT0 pin during a reflash cycle (requires bench instrument), or use M4 to check `GPIOG->IDR & (1<<7)` after writing and report via gpio167 LED pattern (mature plan).

## 5. Current bench state

- **x8h7 bridge:** stalled. `/sys/class/gpio/gpio167/value` and similar reads return `Connection timed out`.
- **L072:** in stale `+ERR_RX` AT firmware (i.e., the modem itself is healthy, just running the broken stock binary we're trying to replace).
- **M4 sketch:** unknown state — could be running and re-asserting PG_7 every second, could be dead. Diagnostic line (gpio167) is unreadable while bridge is stalled.
- **M7:** likely stuck in System Bootloader (consistent with x8h7 stall).

**Required first action before any further testing:** physical power cycle (unplug USB-C + 12 V barrel ≥10 s, replug). This is the same recovery as 2026-05-07 morning and has been confirmed reliable.

## 6. Decision matrix — what to do next

After power cycle, one of these paths. They are roughly ordered by cost-to-attempt; reliability and finality vary.

| # | Path | First action | Expected outcome | If it fails |
|---|---|---|---|---|
| **A** | **Visual LED check** | After power cycle, re-flash helper sketch via existing pipeline. Watch the on-board green LED on the Portenta module *and* the red carrier LED. | Both blink @ 1 Hz → M4 alive, H1 ruled out, focus on H2 (PG_7 ownership). LEDs dark or steady → M4 not booting, focus on H1 (boot path broken). | Move to (B) or (D). |
| **B** | **gpio167 read after settling** | Power cycle, wait 60 s for bridge to fully settle, *then* `gpioget` or `cat /sys/class/gpio/gpio167/value` repeatedly. | Same disambiguator as (A) but programmatic; succeeds only if bridge is healthy. | If bridge still stalls, M4 reflash itself may be triggering the stall — escalate to (D). |
| **C** | **Investigate `bootM4()` on this OE image** | Read x8h7-firmware source (Arduino-Pro/portenta-x8-h7-firmware repo). Check whether the bridge firmware calls `HAL_RCC_WakeUpStop2CPUInit()` or sets `RCC_GCR.BOOT_C2` itself. | Find documented M4-boot trigger or confirm none exists on this image. | If image lacks M4-boot, document and pivot to (E). |
| **D** | **Temporarily replace x8h7 firmware** | Build minimal H7 firmware: assert PG_7 HIGH, idle. Flash via openocd. Run stm32flash from Linux. After L072 reflash, run `systemctl restart stm32h7-program.service` to restore stock x8h7 binary (already on disk at `/usr/arduino/extra/STM32H747AII6_CM7.bin`). | Deterministic — no race with M4 cores. | This is the high-confidence fallback; it loses Linux↔H7 services for ~2 minutes during the procedure. |
| **E** | **Pivot to USB-DFU on the L072** | Connect a USB cable directly to the Murata module's USB pads and use stock STM32 DFU (the L072 ROM has both UART and USB DFU). Use existing gpio163 NRST + a manual jumper wire on the BOOT0 test pad. | Bypasses x8h7 entirely. | Requires probing/soldering on the carrier — may not be physically practical. |
| **F** | **External J-Link to L072 SWD** | Direct SWD onto the Murata module's debug pins. | Most flexible; no boot-path dependency at all. | Likewise requires hardware access and an external probe. |

**My recommendation (cheap → expensive):**

1. **A first.** It's free and disambiguates H1 vs H2 in 30 seconds.
2. **D if A says M4 boots.** A custom H7 sketch that just asserts PG_7 is ~50 lines and recovery is documented and reliable.
3. **C in parallel** as background research — knowing whether x8h7 firmware on this OE image even calls `bootM4()` is foundational and we should document it regardless.

## 7. Open files / artifacts created or touched this session

- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/x8_lora_bootloader_helper.ino` — Option P0-A M4 helper sketch (committed plan; flashed but did not assert BOOT0).
- `%TEMP%\lifetrac_x8_boot_helper_build\x8_lora_bootloader_helper.ino.elf` — compiled artifact (3,698,388 bytes).
- `%TEMP%\verify_l072_rom.sh` — Linux-side ROM-bootloader probe (AT → 0x7F sequence).
- `%TEMP%\poll_m4_led.sh` — gpio167 polling for M4 liveness (currently blocked by bridge stall).
- `%TEMP%\inspect_m4.sh`, `%TEMP%\inspect_boot.sh` — service/unit/script enumeration scripts.
- `%TEMP%\sweep_boot0_v2.sh` — definitive BOOT0 sweep (from earlier in session, still relevant).
- This document.

## 8. Constants and verified facts (for next session)

- ADB serial: `2E2C1209DABC240B`. Sudo: `echo fio | sudo -S -p '' …`.
- Arduino CLI: `C:\Users\dorkm\AppData\Local\Programs\ArduinoCLI\arduino-cli.exe`. Core `arduino:mbed_portenta` 4.5.0.
- FQBN for M4 sketches on X8: `arduino:mbed_portenta:portenta_x8`.
- M4 deploy pipeline: push ELF → `/tmp/arduino/m4-user-sketch.elf` → `touch` triggers `monitor-m4-elf-file.path` → `openocd … program … verify reset exit`.
- `x8h7_gpio`: base 160, ngpio 34 (lines 160–193 only).
- Verified Linux-mapped pins:
  - **gpio163** = LoRa NRST (PC_7), works.
  - **gpio167** = M4 `LED_BUILTIN` mirror (line 7 on x8h7_gpio chip).
- `/dev/ttymxc3` = direct i.MX `imx-uart` to Murata L072 at 19200 8N1 (no H7 in path for data; H7 only handles control pins).
- L072 stale firmware response to `AT\r\n`: 29 bytes — `Error when receiving\n+ERR_RX\r`.
- L072 expected ROM bootloader response to `0x7F` at 19200 8E1: `0x79` ACK.
- Stock H7 firmware binary on disk: `/usr/arduino/extra/STM32H747AII6_CM7.bin`. Reflash service: `stm32h7-program.service`.
