# Murata L072 LoRa Module J-Link Hardware Unbrick

**Date:** 2026-05-17
**Board:** Portenta Max Carrier (ABX00043) + Portenta X8 — Board 1 (adb serial `2D0A1209DABC240B`)
**Target:** Murata LoRa Module (STM32L072CZ)
**Context:** Module soft-bricked (post-`wunprot` OPT-byte lockout, ref [/memories/repo/lifetrac-l072-board2-opt-bytes-corrupted.md](file:memories/repo/lifetrac-l072-board2-opt-bytes-corrupted.md)) and the standard Portenta X8 flashing pipeline (`flash_unitB.ps1`) failed because the X8's H7 bridge SWD bus was unresponsive (`cannot read IDR`). Note: the i.MX8MM application processor remained fully functional via adb throughout — only the H7-DAP path was dead.

Related gates: closes the unbrick prerequisite for [LifeTrac-v25/TODO.md](../TODO.md) W1-7 (Method-G Phase 1, BOOT_URC capture).

This document details the exact Tier 4 hardware recovery steps used to wipe and recover the LoRa module.

## 1. Hardware Interface & Cable Modification

The Max Carrier exposes the LoRa SWD lines via a 10-pin 1.27mm pitch debug header (`CN2`). 

**CRITICAL WARNING:** 
A standard Segger Needle Adapter uses a physically incompatible "U-shaped" pin progression. The `CN2` header uses the standard ARM Cortex "Zig-Zag" pin progression. **Attempting to force a Needle Adapter onto this header will short pins and fail to connect.**

**Solution:** The user had to mechanically modify their J-Link breakout/cable to map the individual SWD pins directly to the correct layout for the ARM Cortex 10-pin header.

## 2. Power Requirements (VTref)

To communicate with the STM32L072, the J-Link requires a reference voltage (VTref) of ~3.3V. 

The LoRa module is powered by the X8's onboard buck converter, which in turn is powered by the Max Carrier.
*   **Requirement:** The 12V barrel jack **MUST** be plugged into the Max Carrier.
*   **Requirement:** The Portenta X8 **MUST** be seated on the carrier and powered on (often requiring the 5V USB-C to be connected as well).
*   If either is missing, the J-Link will report `VTref=0.000V. Target voltage too low.`

## 3. Mass Erase via J-Link

With the custom cable held against the header and VTref reading stably at ~3.3V, we connected using the J-Link Commander:

```powershell
JLink.exe -device STM32L072CZ -if SWD -speed 1000 -autoconnect 1
```

Once inside the J-Link prompt, we forcefully mass-erased the chip. This wipes the Flash memory and, most importantly, resets the Option Bytes (including the complement-byte pair) that caused the soft-brick lockout:

```text
J-Link> erase
```

*Note: J-Link Commander `erase` on STM32L0 performs a mass-erase of user flash and restores Option Bytes to factory defaults (RDP=0xAA level 0). RAM is volatile and not relevant here.*

### 3a. Verify Option Bytes BEFORE flashing

This is the diagnostic that would have caught the 2026-05-14 OPT corruption immediately. From the J-Link prompt:

```text
J-Link> mem32 0x1FF80000 4
```

Expected (factory-clean):
*   word 0 = `0x807800AA` (RDP=0xAA, nRDP=0x55, USER=0x78, nUSER=0x87)
*   word 1 = `0xFFFF0000` (WRPROT1)
*   words 2-3 = `0xFFFF0000` (WRPROT2 / unused)

If RDP ≠ 0xAA or any complement byte fails the `byte XOR ~byte == 0xFF` test, do NOT proceed to `loadbin` — re-issue `erase` first.

## 4. Bypassing the X8 H7-SWD Path to Flash Firmware

Normally, the `flash_unitB.ps1` script pushes the binary through the X8's internal SPI/I2C/SWD bridges. However, the X8's H7 bridge SWD bus was unresponsive (`cannot read IDR`) — a known Board-1 condition since 2026-05-12 that survived even a Tier 2 Cold Power Cycle. The i.MX8MM itself was fine; only the H7-DAP path was dead.

To safely bypass the H7-SWD wedge, we maintained physical pressure on the J-Link pins on `CN2` and pushed the custom firmware binary (`lora_ping_rx_active.bin`) directly to the base of the STM32L072 flash memory (`0x08000000`).

We used the following J-Link Commander script (`load_rx.jlink`, run from the **repo root** so the relative `loadbin` path resolves):

```text
r
h
loadbin LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/build_ping_rx_active/lora_ping_rx_active.bin, 0x08000000
r
go
qc
```

Executed via:
```powershell
JLink.exe -device STM32L072CZ -if SWD -speed 1000 -autoconnect 1 -CommanderScript load_rx.jlink
```

**Firmware provenance (TODO — fill in at flash time):**
*   Source: `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/build_ping_rx_active/lora_ping_rx_active.bin`
*   SHA256: `<run: certutil -hashfile lora_ping_rx_active.bin SHA256>`
*   Built from commit: `<git rev-parse HEAD at build time>`

## 5. Post-Flash Verification

After `go`, the LoRa module should be alive and the SWD pressure can be released. Confirm via one of:
*   **LED**: `lora_ping_rx_active` is expected to toggle the user LED on RX (TODO — confirm cadence with firmware author).
*   **UART**: with the J-Link removed, on the X8: `stty -F /dev/ttymxc3 19200 cs8 parenb -parodd -cstopb raw; xxd < /dev/ttymxc3` and look for periodic packet-RX prints from the firmware (TODO — confirm baud/format; current firmware may be silent until a peer transmits).
*   **Over-air**: power on the second Max Carrier with `lora_ping_tx_*` and watch for RX activity on this unit.

## 6. Re-entering ROM Bootloader After Flash — Falsified Path

**Finding (2026-05-17, post-erase, factory-clean OPT bytes):** the `swd_bypass_pa11_pf4_launcher.sh` SWD-bypass path **does NOT successfully drive the L072 into the ROM bootloader.** Probe result:

```
PA11 driven HIGH via /sys/class/pwm/pwmchip0/pwm4 (period=1ms, duty=99.9%)  ✓ (sysfs write OK)
NRST pulsed low 250ms via /sys/class/gpio/gpio163                            ✓ (sysfs write OK)
Probe /dev/ttymxc3 @ 19200 8E1, send 0x7F, 8 attempts                        → size=0 every time
ACK_HIT=0  →  RESULT: FAIL
```

This falsifies the prior assumption (recorded in `/memories/repo/lifetrac-x8-l072-bootloader.md`) that the bypass launcher provides a working alternate ROM-entry path. It does not — at least not on Board 1's hardware, and not while user firmware is resident. Candidate root causes (un-investigated):

1.  **PWM channel → PA11 mapping wrong**: x8h7_pwm channel 4 may not route to H7 PA11 on this firmware build.
2.  **GPIO offset → PF4 mapping wrong**: gpio163 (= base 160 + offset 3) may not route to H7 PF4.
3.  **H7 bridge GPIO override is ignored at L072 reset time**: even if PA11/PF4 are toggled on the H7, they may not be electrically tied to the L072 BOOT0/NRST nets via the H7 bridge — the schematic may require the H7's *own* firmware to forward them.
4.  **`lora_ping_rx_active` firmware claims the BOOT0 line back** as a GPIO post-reset (less likely — BOOT0 is sampled only at reset).

**Operational consequence:** Method-G ROM-based reflashing of this L072 over the X8 UART is **not currently possible on Board 1** without either (a) repairing the H7 SWD path so OpenOCD can drive BOOT0/NRST, or (b) using the J-Link hardware path documented in §1-§4 above for every reflash. Treat J-Link-on-`CN2` as the canonical Board-1 reflash mechanism until the bypass is debugged.

## Conclusion

The successful erase and flash returned the LoRa module to active duty, completely subverting the need to factory-reset (Tier 3 SDP) the Portenta X8 itself. However, the related assumption that the X8's x8h7-bridge sysfs path can drive the L072 into ROM as a software-only alternative is **falsified** as of this date — see §6.