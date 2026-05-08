# Portenta X8 Murata L072 Flashing Plan — Option P0-C: OpenOCD Register-Poke
**Date**: 2026-05-07
**Author**: Copilot (assistant)
**Status**: Formulated / Pending Execution

## 1. Executive Summary

This document outlines a zero-footprint strategy to flash the Murata L072 LoRa modem on the Portenta X8 Max Carrier. It explicitly bypasses the limitations of the X8's internal `x8h7` bridge firmware and negates the need for custom hardware-level M4 or M7 boot helper sketches. 

Instead of treating the X8's internal OpenOCD as a debugger, we will weaponize it as a one-shot STM32H747 register writer. By halting the H7 core via OpenOCD, we physically freeze the opaque `x8h7` bridge firmware and take bare-metal control of the H7's memory bus. We will then directly write to the H7's GPIO registers over SWD to assert the Murata `BOOT0` strap and pulse `NRST`. 

Once the Murata module is forced into its ROM Bootloader, we exploit the direct i.MX-to-Murata UART (`/dev/ttymxc3`) to upload the payload. 

## 2. The Architectural Blocker

1. **Missing GPIO Export**: The `x8h7` Linux-to-H7 bridge firmware does not export `PG_7` (`LORA_BOOT0`) to the Linux `x8h7_gpio` driver. Thus, writing a pure Linux-level Python flashing script fails because we cannot drive `BOOT0` high.
2. **Firmware Dependencies**: Flashing a custom M7 helper sketch overwrites the `x8h7` bridge, breaking Linux-side commands and requiring invasive recovery. Flashing an M4 helper sketch is blocked by the M7 bootloader halting sequence when OpenOCD is normally attached.
3. **The Blessing**: Repeated probing confirms `/dev/ttymxc3` on the X8 Linux host provides direct, unproxied UART access to the Murata L072. We *only* need the H7 to hold the BOOT0 strap and toggle reset.

## 3. Option P0-C Theory: The "Register-Poke" Bypass

OpenOCD provides AHB-AP (Advanced High-performance Bus - Access Port) access directly to the STM32H747 peripherals. By issuing generic Memory Write Word (`mww`) commands via OpenOCD to the H7's physical register addresses, we can electrically manipulate `PG_7` and `PC_7`.

Because the H7 firmware is halted, it cannot fight our pin assertions or attempt to reclaim GPIO ownership. 

### Required STM32H747 Register Map:
- **RCC_AHB4ENR (`0x580244E0`)**: Clock enable for GPIO ports. Bit 2 = GPIOC, Bit 6 = GPIOG.
- **GPIOC_MODER (`0x58020800`)**: Mode register for `PC_7` (`LORA_RESET`). Bits 15:14 -> `01` (General purpose output mode).
- **GPIOC_BSRR (`0x58020818`)**: Bit Set/Reset Register. Bit 7 = Set (High), Bit 23 = Reset (Low).
- **GPIOG_MODER (`0x58021800`)**: Mode register for `PG_7` (`LORA_BOOT0`). Bits 15:14 -> `01` (General purpose output mode).
- **GPIOG_BSRR (`0x58021818`)**: Bit Set/Reset Register. Bit 7 = Set (High), Bit 23 = Reset (Low).

## 4. Execution Step-by-Step

### Phase 1: Clean Slate & OpenOCD Halt
1. Perform a hard power cycle of the Portenta X8 (unplug 12V barrel and USB-C). This clears any stale kernel module hangs or OpenOCD zombie states.
2. Connect exclusively via SSH to the Linux side.
3. Launch OpenOCD and halt the H7:
   ```bash
   openocd -f /usr/arduino/extra/openocd_script-imx_gpio.cfg -c "init" -c "halt"
   ```

### Phase 2: The Register Poke (via OpenOCD telnet or direct `-c` commands)
*Execute these `mww` writes to take control of the pins and drive the Murata module into Bootloader Mode:*

```tcl
# 1. Enable clocks for GPIOC and GPIOG (Read-Modify-Write is safer, but a forced OR is standard for this attack)
# Bit 2 (0x4) and Bit 6 (0x40)
mww 0x580244E0 0x00000044

# 2. Configure PG_7 (LORA_BOOT0) as Output (Write 01 to bits 15:14)
# (Assuming Reset State is ~Output, we overwrite. Better handled with TCL read/mask, simplified below)
mww 0x58021800 0x00004000 

# 3. Drive PG_7 HIGH (Assert BOOT0)
mww 0x58021818 0x00000080

# 4. Configure PC_7 (LORA_RESET) as Output (Write 01 to bits 15:14)
mww 0x58020800 0x00004000

# 5. Pulse PC_7 LOW (Assert Reset)
mww 0x58020818 0x00800000
sleep 50

# 6. Pulse PC_7 HIGH (Release Reset - L072 now boots and samples BOOT0 pin)
mww 0x58020818 0x00000080
```

### Phase 3: Flash the Murata L072
With the OpenOCD session still holding the H7 halted and `PG_7` driven high:
1. Open a new SSH terminal into the X8 Linux host.
2. Configure the UART for STM32 System Bootloader parity:
   ```bash
   stty -F /dev/ttymxc3 19200 raw -echo cs8 parenb -parodd -cstopb
   ```
3. Send the synchronization byte (`0x7F`) to `/dev/ttymxc3`.
4. Validate the ROM bootloader ACK (`0x79`).
5. Proceed with flashing custom firmware (Method G payload) via `stm32flash` or a custom Python transport script. 

### Phase 4: Teardown
1. Kill the OpenOCD session.
2. Power cycle the X8. The system will reboot functionally whole; no flash memory on the host SoC was modified.

## 5. Contingencies

- **No ROM Bootloader ACK (`0x79`)?** If the Murata fails to ACK after this register poke, the `PC_7` / `PG_7` assumptions from the Max Carrier schematic or Arduino Mbed core pinouts are factually incongruent with the physical traces on this hardware variant. The fallback is to meter `PG_7` vs physical BOOT0 straps (SW1 / R19) to confirm electrically. 
- **BSRR Mismatch?** If `PC_7` doesn't seem to reset the modem, verify the read state of `GPIOC_IDR`.
