# Murata L072 Bring-up on Max Carrier

Date: 2026-05-05
Scope: Bench flash and first-link validation for Method G firmware.
Reference schematic: Arduino ABX00043 (Portenta Max Carrier), revision V3.12, sheets 1-14.

## 1. Hardware checklist

- Portenta H7 (or H7 Lite/Connected) + Max Carrier (target board)
- Murata CMWX1ZZABZ-078 hosted on the Max Carrier (sheet 6 `LORA` block, U23)
- ST-Link V2 or J-Link (SWD) **— see §2 warning: CN2 is DNP by default**
- (Optional, often sufficient) on-carrier USB debugger (sheet 3 STM32F405) exposes `UART_SNIFF1..5` channels for passive UART snooping; covers most of what a separate USB-serial cable would do during bring-up
- Separate USB-serial adapter for host UART capture at 921600 8N1 (only required if `UART_SNIFF` channel is unavailable for the chosen `SERIALn`)
- 915 MHz antenna or 50 Ω load before any TX test
- 20-40 dB attenuators for two-board coax bench tests
- Multimeter or scope for verifying SWD pad continuity if the operator solders CN2

### 1.1 `LIFETRAC_MH_SERIAL` selection (schematic-driven)

The LoRa SiP exposes USART2 (PA2/PA3 + RTS/CTS) on its `SERIAL` net (sheet 6). On the Max Carrier this routes through the 80-pin HD connector to one of the H7-side `SERIAL{0,1,2}` instances (sheet 1, sheet 2 HD connector pinmap).

**Excluded outright:**

- `Serial1` — already consumed by the X8↔H747 link (`Serial1.begin(921600)` in `tractor_h7.ino` near line 1788).
- `Serial3` — reserved by the on-carrier Cellular Modem (sheet 5 `CELL_MODEM` → `MODEM_NOTATION` net → `SERIAL3`).

**Bench candidates:** `Serial2`, `Serial4`, `Serial5`. Final selection requires:

1. Zoom into ABX00043 sheet 1 (`TOP`) LoRa Module sub-sheet symbol; identify which `SERIAL{0,1,2}` net carries the LoRa UART up to the HD connector.
2. Cross-reference Arduino mbed `variants/PORTENTA_H7_M7/variant.cpp` (and `PeripheralPins.c`) to map that HD pin ↔ STM32H747 GPIO ↔ Arduino `SerialN` instance.
3. Confirm the chosen pins are NOT also claimed by `Wire`, `SPI`, ADC, or PWM in any default-on path on the H7 side (`SHARED PINS on H7` note, sheet 2).
4. Record the trace + chosen `SerialN` in `bench-evidence/T6_bringup_<date>/serial_routing.md` and `pinmap_audit.md`.
5. Build with `-DLIFETRAC_MH_SERIAL=Serial<N>` (build fails if the macro is undefined; intentional safety per T6 plan v1.2).

If the on-carrier USB debugger's `UART_SNIFF` channel matches the chosen `SerialN` HD pin (sheet 3), the same USB cable that flashes/debugs can also passively monitor the H7↔L072 traffic during bring-up.

## 2. SWD wiring

> **WARNING:** On stock ABX00043 boards the `CN2` (HPHD2-A-10-SGA) LoRa-SWD debug header is **DNP** (depopulated by default; see X marks on sheet 6). The `LORA_SWDIO` / `LORA_SWCLK` / `LORA_RST` / `LORA_GND` / `+3V3` nets are routed to the CN2 pads, but the connector itself is not soldered. Operators have two choices:
>
> 1. **Solder CN2 yourself** — 2x5 1.27 mm pitch header per the Max Carrier silkscreen, then attach ST-Link V2 / J-Link normally.
> 2. **Skip SWD entirely** — use the DFU path in §4C; the BOOT0 net is wired through R19 and viable on stock boards.

Wire SWDIO, SWCLK, GND, and VREF from debugger to the soldered CN2 header (or to the LoRa-SWD test pads if you trace them). Use NRST if available.

- Never back-power through ST-Link 3.3 V when the target has its own supply.
- Confirm target I/O voltage is 3.3 V before attaching debugger.

> **STSAFE warning (sheet 6, pin 34 `MCU_NRST/STSAFE_RST`):** the L072's NRST line is shared with the on-board STSAFE secure element (when populated). An OpenOCD `init; reset halt` cycle resets STSAFE as well. Chip-erase via `flash erase_sector ...` does NOT erase STSAFE provisioning, but a held NRST does interrupt any in-progress STSAFE session. T6 Method G firmware does not use STSAFE, so this is informational only — but if a future tranche provisions STSAFE, document a chip-erase recovery procedure first.

## 3. OpenOCD config files

Configs are provided in:

- `openocd/stlink.cfg`
- `openocd/stm32l0_swd.cfg`

Example command:

```bash
openocd -f openocd/stlink.cfg -f openocd/stm32l0_swd.cfg \
  -c "program build/firmware.elf verify reset exit"
```

## 4. Flash recipes

### A) ST-Link + OpenOCD

```bash
cd LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072
make all
openocd -f openocd/stlink.cfg -f openocd/stm32l0_swd.cfg \
  -c "program build/firmware.elf verify reset exit"
```

### B) J-Link CLI

```bash
JLinkExe -device STM32L072CZ -if SWD -speed 1000 <<'EOF'
r
loadfile build/firmware.bin, 0x08000000
r
g
q
EOF
```

### C) DFU path (board-specific)

Viable on stock ABX00043 boards — BOOT0 is wired via R19 from the `BOOT0` macro net (sheet 6) to the L072 BOOT0 pin (43). Confirm R19 is populated (0 Ω) before relying on this path. The BOOT0 entry sequence depends on how the carrier exposes the BOOT0 line to USB-DFU — if the Max Carrier does not directly expose USB-DFU to the L072 (it may not; verify against your specific board), fall back to the soldered-CN2 SWD path in §4A.

```bash
dfu-util -l                                             # confirm L072 enumerates
dfu-util -a 0 -s 0x08000000:leave -D build/firmware.bin
```

## 5. Three-stage bench procedure

### Stage 1: single-board boot and clock gate

Connect host UART and verify first BOOT_URC fields:

- `clock_source_id == 0` is required (HSE OK)
- `clock_source_id == 1` means HSI fallback: stop and inspect TCXO/clock path
- `radio_ok == 1` required before RF stages

Optional MCO check: only valid on debug firmware variants that explicitly export MCO. Production artifacts do not guarantee MCO output.

### Stage 2: known-good RX validation

RX is armed during normal boot after radio init. There is no RX_START host command.

- Transmit from a known-good LoRa peer on matching profile
- Confirm `HOST_TYPE_RX_FRAME_URC` arrival
- Confirm STATS counters increase (`radio_rx_ok`)

### Stage 3: two-board round-trip

- Board A sends `HOST_TYPE_TX_FRAME_REQ`
- Board A receives `HOST_TYPE_TX_DONE_URC`
- Board B receives matching `HOST_TYPE_RX_FRAME_URC`

Target pass criteria for 5-minute attenuated bench run:

- TX_DONE success rate >= 99%
- matching RX_FRAME payload rate >= 99%
- no persistent `radio_tx_abort_airtime` growth under nominal load

## 6. Failure-mode quick map

- `clock_source_id == 1`: HSE did not lock, investigate TCXO/clock path (sheet 6: TCXO is internal to U23 SiP; check `VDD_TCX0` rail and R12)
- `radio_ok == 0`: verify SX1276 wiring/power/reset path
- Frequent FAULT_URC `HOST_FAULT_CODE_CLOCK_HSE_FAILED (0x08)`: clock stability fault, stop RF tests
- No TX_DONE with valid TX requests: inspect RF switch path and IRQ wiring (sheet 6: `LORA_IRQ` via R16 from PA8/MCO; ANT/CRF1/CRF2/CRF3 RF-switch pins)
- H7 sees no L072 UART traffic with `LIFETRAC_USE_METHOD_G_HOST=1`: §1.1 mis-selection of `LIFETRAC_MH_SERIAL` is the most likely cause; passively snoop the chosen line via the on-carrier `UART_SNIFF` channel (sheet 3) to confirm bytes are arriving on the expected HD pin
- After OpenOCD `init; reset halt`, STSAFE state may be reset (sheet 6: shared NRST); informational only for T6 firmware which does not use STSAFE

## 7. Evidence capture

Save bench artifacts under:

- `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_<date>/`

Recommended artifacts:

- host UART log including BOOT_URC, FAULT_URC, STATS snapshots
- TX_DONE/RX_FRAME timestamp CSV for two-board run
- optional spectrum or MCO screenshots if captured
- `serial_routing.md` — sheet-1 trace + Arduino mbed cross-reference proving the chosen `LIFETRAC_MH_SERIAL` (per §1.1)
- `pinmap_audit.md` — confirmation that the chosen `SerialN` HD pins are not multiplexed with another default-on H7 peripheral (per T6 plan v1.2 R11)
- photo of the soldered CN2 header (if SWD path used) or `dfu-util -l` enumeration log (if DFU path used) — proves the chosen flash recipe is reproducible on this physical board
