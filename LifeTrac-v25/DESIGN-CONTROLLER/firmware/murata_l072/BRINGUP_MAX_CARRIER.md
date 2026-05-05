# Murata L072 Bring-up on Max Carrier

Date: 2026-05-05
Scope: Bench flash and first-link validation for Method G firmware.

## 1. Hardware checklist

- Portenta X8 + Max Carrier (target board)
- Murata CMWX1ZZABZ-078 target connection (direct board or Max Carrier routed pins)
- ST-Link V2 or J-Link (SWD)
- USB serial adapter for host UART capture at 921600 8N1
- 915 MHz antenna or 50 ohm load before any TX test
- 20-40 dB attenuators for two-board coax bench tests

## 2. SWD wiring

Wire SWDIO, SWCLK, GND, and VREF from debugger to the Murata-hosting board. Use NRST if available.

- Never back-power through ST-Link 3.3 V when the target has its own supply.
- Confirm target I/O voltage is 3.3 V before attaching debugger.

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

Use only if your board exposes DFU for the Murata L072 image path.

```bash
dfu-util -l
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

- `clock_source_id == 1`: HSE did not lock, investigate TCXO/clock path
- `radio_ok == 0`: verify SX1276 wiring/power/reset path
- Frequent FAULT_URC `HOST_FAULT_CODE_CLOCK_HSE_FAILED (0x08)`: clock stability fault, stop RF tests
- No TX_DONE with valid TX requests: inspect RF switch path and IRQ wiring

## 7. Evidence capture

Save bench artifacts under:

- `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/T6_bringup_<date>/`

Recommended artifacts:

- host UART log including BOOT_URC, FAULT_URC, STATS snapshots
- TX_DONE/RX_FRAME timestamp CSV for two-board run
- optional spectrum or MCO screenshots if captured
