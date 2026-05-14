# 2026-05-13 — Board 1 (`2D0A`) post-T3a SDP reflash — SWD wedge persists

**Routine evidence:** [HC-01 ADB](../routines/HC-01_adb_enumeration.md), [HC-02 Linux health](../routines/HC-02_linux_health.md), [HC-03 SWD attach](../routines/HC-03_h7_swd_attach_diff.md)

## Procedure executed

T3a per [recovery/T3a_sdp_uuu_reflash.md](../recovery/T3a_sdp_uuu_reflash.md):

1. Power off, BOOT SEL DIP **ON**, USB-C only.
2. SDP HID enumerated as `VID_1FC9&PID_0134` (i.MX 8M Mini mask ROM).
3. `uuu 1.5.243` ran `full_image.uuu` from `mfgtool-files-portenta-x8/`.
4. Result: **12/12 stages, Success 1 / Failure 0**.
5. DIPs OFF, power-cycle (2 cycles needed; first attempt LEDs went red — likely 12 V missing or first-boot still running).

## Post-reflash verification

| Check | Result | Notes |
|---|---|---|
| HC-01 ADB | ✅ PASS | both boards `device` |
| HC-02 Linux | ✅ PASS | uptime 1 min, fresh LmP `4.0.11-934-91`, kernel `6.1.24-lmp-standard`, hostname `portenta-x8-2d0a1209dabc240b` |
| x8h7 module load | ✅ PASS | 10 modules, m4-proxy + stm32h7-program active |
| x8h7 SPI bridge liveness | ✅ PASS | dmesg shows `x8h7_can can0/can1 successfully initialized`, `x8h7_rtc registered`, `x8h7uart driver loaded` — **H7 firmware is alive and the SPI side works** |
| Manual gpio preflight | ✅ PASS | gpio8 in/0, gpio10 out/1, gpio15 in/0 |
| HC-03 SWD attach | ❌ **FAIL** | `bitbang_swd_read_reg(): JUNK DP read reg 0 = ffffffff` / `Error connecting DP: cannot read IDR` — identical signature to pre-reflash |

## Conclusion — IOMUXC hypothesis disproven

T3a definitively rules out the i.MX/Linux side as the cause:

- Bootloader, kernel, pinctrl driver, x8h7 modules, all userland — **all freshly written from a known-good factory image**.
- Carrier rails healthy (HC-02 PASS, x8h7 SPI bridge functional).
- H7 firmware running and responsive (SPI-side modules all initialized).
- gpio8/gpio10/gpio15 muxed correctly (preflight verified).
- Yet SWD attach still cannot read the H7 DP IDR.

**The fault is on the hardware path or H7 silicon state**, not on anything `uuu` can touch:

1. **Most likely:** physical damage to gpio8 (SWDIO) or gpio15 (SWCLK) traces / pads / vias on the X8 SoM, accumulated from the W2-01 ci_hdrc-imx wedge cycles + emergency unplugs. Damaged signal integrity could allow the line to read `0` (preflight reads in-direction `0`) yet not pass clean SWD signaling.
2. **Possible:** a latched state in the H7's debug power domain (DBGMCU) that survives even cold-boot — for instance an enabled WFI/WFE with debug clocks gated off in standby mode.
3. **Less likely:** Board 1's H7 firmware has SWD permanently disabled via option bytes (would need a deliberate change; both boards run the same fw).

## Recovery options remaining

- **T3b** — RAM-only rescue boot to capture forensic data on H7 (limited value if SWD is the only programming path).
- **T4 (new)** — H7 reflash via the **STM32 system bootloader** (BOOT0=high entry), assuming the BOOT0 pin is exposed. Would need schematic check on Portenta X8 SoM to identify the pin and a way to assert it.
- **T5 (new)** — Hardware inspection: scope the SWCLK/SWDIO lines during an attach attempt; check for shorts/opens with a meter.
- **T6 (worst case)** — Retire Board 1 from the bootloader pipeline, use it for higher-layer testing only (Board 1's L072 path is unaffected since the L072 lives on the Murata module and uses its own pins, but Stage 1 still needs the H7 SWD path to gate NRST during flash).

## Raw evidence

- uuu run: console capture (Success 1 / Failure 0, 12/12).
- HC-03 attempt: `Error: 3187 533 adi_v5_swd.c:144 swd_connect(): Error connecting DP: cannot read IDR`.
- dmesg liveness: `x8h7_can can@2 can0: X8H7 CAN successfully initialized.` etc.

## Recommendation

Stop trying soft-recovery paths on Board 1. Treat Board 1 as having a permanent
hardware-side SWD fault until proven otherwise via scope/meter inspection or
H7 system-bootloader reflash. **Board 2 (`2E2C`) remains the only flash-pipeline
asset.** Resume W1-10b 100-cycle campaign on Board 2 alone.
