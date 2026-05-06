# T6 Serial Routing Evidence

Date: 2026-05-05
Scope: Gate 3 serial-route proof for Method G bring-up

## Source references used

- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md` section 1.1
- `LifeTrac-v25/DESIGN-CONTROLLER/BUILD_CONFIG.md` section 10
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino`
- Installed Arduino core files:
  - `C:/Users/dorkm/AppData/Local/Arduino15/packages/arduino/hardware/mbed_portenta/4.5.0/variants/PORTENTA_H7_M7/pins_arduino.h`
  - `C:/Users/dorkm/AppData/Local/Arduino15/packages/arduino/hardware/mbed_portenta/4.5.0/variants/PORTENTA_X8/pins_arduino.h`

## Constraints confirmed

1. `Serial1` is excluded for Method G on target hardware.
   - `tractor_h7.ino` initializes `Serial1.begin(921600)` for the X8<->H747 link.
2. `Serial3` is excluded by Max Carrier design guidance.
   - Runbook marks `Serial3` reserved by the on-carrier cellular modem path.
3. Runbook bench candidates are `Serial2`, `Serial4`, `Serial5` pending board trace confirmation.

## Installed core audit result

### PORTENTA_H7_M7 variant

- `pins_arduino.h` defines:
  - `SERIAL2_TX PA_15`
  - `SERIAL2_RX PF_6`
  - `SERIAL2_RTS PF_8`
  - `SERIAL2_CTS PF_9`
- This confirms a complete 4-wire UART mapping exists for `Serial2` in the H7 variant.

### PORTENTA_X8 variant (current local core)

- `pins_arduino.h` defines `SERIAL_HOWMANY 1` and only `SERIAL1_TX`/`SERIAL1_RX`.
- `Serial2`/`Serial4`/`Serial5` are not exposed in this stock local X8 core package.

## Selected desk-audit route

Selected bench candidate: `Serial2`

Reason:
- It is the only runbook-allowed candidate that is concretely mapped in the locally installed variant data.
- It provides RTS/CTS pin definitions in the H7 variant header.
- It avoids known excluded routes (`Serial1`, `Serial3`).

## Remaining physical proof required

To fully close serial routing on physical hardware, capture one of:

1. ABX00043 sheet trace screenshot (LoRa `SERIAL` net to chosen H7 `SerialN` pins), and
2. UART_SNIFF or logic-analyzer capture at 921600 8N1 showing Method G traffic on the chosen route.

Without this hardware capture, this file is a desk-audit selection record, not final wiring proof.
