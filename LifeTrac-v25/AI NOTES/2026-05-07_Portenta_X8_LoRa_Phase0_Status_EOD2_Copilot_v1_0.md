# 2026-05-07 Portenta X8 LoRa Phase 0 Status — End of Day #2

**Author:** Copilot v1.0
**Session focus:** Method G Phase 0 — flash custom firmware to onboard L072 LoRa modem from the Portenta X8 itself.
**Status:** Multiple attack paths conclusively disproven. Hardware routing assumption has fallen through. Hardware-level investigation now required.

---

## TL;DR

- We have a way to **electrically prove** any H747 GPIO state via OpenOCD (`imx_gpio` driver, M7 halted in System Bootloader at `PC=0x1ff09abc`, RMW writes verified by IDR readback).
- **PG_7 was driven HIGH** at the H747 pad — IDR reads back `1` — but the L072 stays in stale AT firmware. Therefore PG_7 (the Portenta H7 standalone variant's `LORA_BOOT0` pin) is **NOT** wired to the L072 BOOT0 net on this hardware.
- **PC_7 is NOT the L072 NRST** — it is mapped to PWM channel 0 in the x8h7 firmware (`src/gpio.c GPIO_pinmap[]`). Our earlier "PC_7 = NRST" assumption was wrong.
- **GPIOG** is dormant in the x8h7 firmware (clock enabled, no pinmap entries). We swept all GPIOG candidates for NRST while holding PG_7 HIGH — **no candidate produced 0x79 ACK** at `/dev/ttymxc3` 19200 8E1.
- L072 still returns its baseline 29-byte `Error when receiving / +ERR_RX` AT response after the sweep. Hardware undamaged.
- Bridge stalls after every OpenOCD cycle (recoverable with power cycle).

---

## Definitive evidence collected

### 1. OpenOCD electrical control proven
`01_halted_dump.cfg` confirms M7 halts at `PC=0x1ff09abc`, `msp=0x24003af8` (System Bootloader) with `RCC_AHB4ENR=0x00000000` — clean slate.

`02_assert_pg7_pc7.cfg` execution log:
```
Phase A: enable GPIOC + GPIOG clocks (RMW)
  RCC_AHB4ENR before = 0x00000000
  RCC_AHB4ENR after  = 0x00000044
Phase C: drive PG_7 HIGH
  GPIOG_MODER before = 0xffffffff
  GPIOG_MODER after  = 0xffff7fff
  GPIOG_IDR.PG_7 = 1   (expect 1)              ← HIGH confirmed at H747 pad
Phase D: pulse PC_7 LOW then HIGH (NRST)
  GPIOC_IDR.PC_7 (low pulse)  = 0
  GPIOC_IDR.PC_7 (after high) = 1
```
Verify probe at `/dev/ttymxc3` 19200 8E1: **0 bytes** (no ROM ACK).
Verify probe at `/dev/ttymxc3` 19200 8N1: **29 bytes `Error when receiving / +ERR_RX`** (stale AT firmware).

### 2. x8h7 firmware pinmap from upstream source
`github.com/arduino/portentax8-stm32h7-fw/blob/master/src/gpio.c` `GPIO_pinmap[]`:
- gpio160 = PF_8, gpio161 = PF_6, gpio162 = PF_3, gpio163 = PF_4 (called "LoRa NRST" in earlier session notes — but actually just an x8h7 generic GPIO with no Linux-side label binding it to LoRa)
- gpio164 = PF_12, gpio165 = PE_10, gpio166 = PE_11
- gpio167 = PF_11 (ADC channel 0)
- ... PWM channel 0 = PC_7 ...
- **No GPIOG entries anywhere in the pinmap** (clock enabled in `MX_GPIO_Init` but unused)

### 3. GPIOG NRST hunt (with PG_7 held HIGH as assumed BOOT0)
`auto_g_hunt.sh` swept PG_0..PG_15 (skip 7, 11/13/14 RMII):
```
PG_0..PG_15: ALL candidates returned 0 bytes from /dev/ttymxc3 8E1 probe
```
No L072 ROM bootloader ACK from any GPIOG-driven NRST candidate.

### 4. Linux-side has no LoRa awareness on this image
- `find /proc/device-tree`: zero hits for `lora`, `sx12`, `murata`, `stm32`
- `/sys/class/gpio`: only gpio8/10/15 exported (i.MX H7 SWDIO/NRST/SWCLK)
- `/usr/arduino/extra/`: only H7 reset.sh + program.sh + load_modules*.sh — nothing LoRa-related
- `fw_printenv` shows active overlay `ov_carrier_enuc_lora` but no DT node manifests it (probably a no-op on this build)

### 5. m4_led_forwarder corrected
The script does `echo 7 > /sys/class/leds/ledR/gpio` — this `7` is **i.MX gpiochip0 line 7** (GPIO1_IO7), NOT x8h7 gpio167. Earlier session note "M4 LED via gpio167" is wrong.

---

## Working hypothesis

**The active firmware/overlay assumes ENUC-carrier wiring; the user hardware is Max Carrier (per session memory).** The LoRa BOOT0 net on the Max Carrier may:
- (A) not be electrically routed to any H747 or i.MX GPIO at all (pad-level access required for ROM bootloader entry);
- (B) be routed to a pin that needs a pinctrl mux change in U-Boot/Linux to activate;
- (C) be on an H747 pin we haven't tried (banks A, B, D, E unswept — all heavily multiplexed for ADC/CAN/UART/SPI on the H7 module itself, hence why the sweep was ordered G-first and would be very risky to extend).

The L072 NRST behaviour observed in earlier sessions (gpio163 toggle "resets the LoRa") may have been coincidence — the AT firmware echoes `+ERR_RX` for any input including binary garbage, so it doesn't actually prove a reset occurred.

---

## What works (toolkit at `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/`)

| File | Purpose |
|---|---|
| `01_halted_dump.cfg` | Snapshot all GPIO bank clock + MODER + IDR state. Zero side-effects. |
| `02_assert_pg7_pc7.cfg` | Interactive RMW assert PG_7 HIGH + pulse PC_7. Holds halt until Enter. |
| `02b_assert_pg7_pc7_hold60.cfg` | Non-interactive 60s hold variant for parallel probing. |
| `03_boot0_hunt.cfg` | Original interactive hunt. Superseded by `auto_g_hunt.sh`. |
| `99_release_and_reset.cfg` | Resume + shutdown openocd. |
| `auto_boot0_hunt.sh` | Generic GPIO sweep (~60 candidates). Untested — superseded by `auto_g_hunt.sh`. |
| `auto_g_hunt.sh` | GPIOG-only hunt with PG_7 held HIGH. Run, no hits. |
| `verify_l072_rom.sh` | Two-phase AT(8N1) + ROM(8E1) probe. |
| `baseline_check.sh` | Quick L072 + bridge health probe. |
| `probe_dt.sh`, `probe_lora_userspace.sh`, `probe_x8h7_dt.sh` | Linux-side LoRa pin discovery. |
| `README_P0C_openocd_pokes.md` | Runbook + critical caveats. |

All scripts pushed to X8 at `/tmp/lifetrac_p0c/`.

---

## Recommended next steps

1. **Confirm carrier identity with the user.** Visual: Max Carrier is the large carrier with ETH, dual CAN, mPCIe, two USB-A; ENUC carrier is much smaller. The active overlay (`ov_carrier_enuc_lora`) may not match.
2. **Locate Max Carrier schematic** (Arduino docs PDF). Specifically need the L072 BOOT0 and NRST net assignments — which H747 (or i.MX) pin those pads route to. This is the authoritative answer.
3. **If Max Carrier doesn't expose BOOT0** to any GPIO (likely), evaluate options:
   - Hardware mod: solder a wire from L072 BOOT0 pad to a known testable GPIO.
   - Use a direct UART-to-USB adapter on the L072 UART pins (bypass X8 entirely) with manual BOOT0 jumper.
   - Re-flash the H7 firmware with a custom build that DOES wire LoRa control out via x8h7 GPIO (recoverable via `stm32h7-program.service`).
4. **If Max Carrier DOES expose BOOT0** on a known H747 GPIO bank (D or E most likely candidates remaining), repeat the hunt scoped to that bank.

---

## Bench restoration

X8 needs power cycle (unplug USB-C + 12V ≥ 10s) to recover the x8h7 bridge (`/sys/kernel/x8h7_firmware/version` returning Connection timed out). L072 itself is fine — still answers AT firmware.
