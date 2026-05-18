# Hardware J-Link Unbrick (LoRa module)

## Purpose
An unbrick procedure using a physical Segger J-Link connected directly to the Portenta Max Carrier's 10-pin LoRa header. Use this if the Murata L072 module's option bytes or flash are corrupt to the point where it refuses the standard bootloader ROM pattern (`0x7F`) or becomes completely deaf.

Because the Max Carrier routes the L072 `SWDIO` and `SWCLK` to this 10-pin header, we can bypass the Portenta X8 entirely and re-flash the module using an external J-Link needle adapter or ribbon cable.

## When to use
* Option bytes have been erased or corrupted such that the ROM is in **RDP Level 1 or Level 2** (read-protected) — the soft UART path cannot lift Level 1 → Level 0 without a power cycle plus the RDP-1→0 mass-erase trigger, and Level 2 is permanent.
* The X8 ↔ L072 UART path is itself dead (USART2/UART4 hardware or pad failure) — confirmed by [routines/HC-04_stage1_standard_quant.md](../routines/HC-04_stage1_standard_quant.md) failing with `FAIL_SYNC` AND HC-02 also failing.
* Module is hard-faulting instantly out of reset preventing connection to the serial AT interface AND the page-by-page Ext-Erase fallback in the post-2026-05-18 `stm32_an3155_flasher.py` did not clear it.

## When NOT to use (try first instead)
* **2026-05-18 update — Tier 4 is now last-resort.** The X8-resident UART flasher handles the formerly-J-Link-only "mass-erase NACK" case via page-by-page Extended Erase. Run [HC-04](../routines/HC-04_stage1_standard_quant.md) first; if it PASSes, no J-Link work is needed.

## Prerequisites

1. **J-Link Debug Probe**: A Segger J-Link Base, Plus, or Edu.
2. **10-pin Adapter**: A **standard J-Link 9-Pin/10-Pin Cortex-M Adapter** (adapts the 20-pin 0.1" header to a 10-pin 0.05" / 1.27 mm pitch ribbon cable). 
   * *WARNING: Do NOT use the "Segger 10-pin Needle Adapter" (which has a proprietary U-shaped pinout and locating pins). The Max Carrier uses the standard ARM Cortex Debug 10-pin zig-zag pinout.*
3. **OpenOCD** or **J-Link software** installed on your host machine.

## Important Note: Powering the Board

The Portenta Max Carrier generates its 3.3V power rail (which powers the LoRa chip) by relying on the Portenta X8's onboard buck converter stepping down the carrier's 5V rail.

Therefore:
* **The 12V barrel jack MUST be attached** to power the high-current demands.
* **The Portenta X8 MUST be seated in the carrier** to generate the 3.3V reference.
* Do not rely on USB power or J-Link Target Power alone.

## Procedure

1. **Ensure Hardware is Powered**:
   * Plug in the 12V barrel jack to the Max Carrier.
   * Attach the Portenta X8 so it can supply the 3.3V rail.
   * Turn the setup on.

2. **Disable X8 Interference**:
   Log into the X8 via SSH or ADB and stop any Linux service that might be actively talking to or resetting the LoRa module. We need the X8 pins attached to SWDIO, SWCLK, and LORA_RST to remain in a high-impedance state.
   ```bash
   # Example: Stop the service talking to the LoRa module.
   systemctl stop m4-proxy.service
   ```

3. **Attach J-Link**:
   * Align the 10-pin needle adapter on the Max Carrier `CN2` (LoRa Debug Header).
   * Note the correct orientation (usually keyed by location pins on the needle adapter). Pin 1 provides the 3.3V Target Reference voltage.

4. **Flash / Erase the Module**:
   * Run OpenOCD or J-Link Commander using an `stm32l0` target profile.
   * Connect, halt the processor (`halt` or `reset halt`), and execute a mass erase or re-flash the correct "golden" binary back into the L072 flash.
   * Verify the option bytes reset to their defaults (Read/Write Protection).

5. **Re-Test via Bridge**:
   * Remove the J-Link probe.
   * Send a manual reset pulse via the bridge GPIO using `deafen_unbrick.sh` or standard AT test scripts.
   * Wait for the "BOOT" URC frame out of the Serial port.
