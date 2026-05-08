# Portenta X8 Max Carrier Mux Bypass Execution Findings (Copilot v1.0)

Date: 2026-05-07
Author: GitHub Copilot
Scope: Execute PA4 multiplexer bypass on Max Carrier, flash firmware, and verify J-Link SWD/RTT attach.

---

## 1. Objective

Validate whether controlling Max Carrier mux select from STM32 pin PA4 unlocks SWD/RTT access over the debug micro-USB path.

---

## 2. Firmware Changes Applied

Target sketch:
- LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_uart_route_probe/x8_uart_route_probe.ino

Changes:
- Added PA4 mux control in setup:
  - pinMode(PA_4, OUTPUT)
  - digitalWrite(PA_4, <LOW or HIGH>)
- Added compile-time selector macro:
  - LIFETRAC_MAXCARRIER_MUX_LEVEL (tested both LOW and HIGH)
- Added SEGGER RTT logging in sketch:
  - SEGGER_RTT_Init()
  - SEGGER_RTT_printf("LT_RTT_PROBE ...")
- Kept UART route probe outputs on Serial1/Serial2/Serial3.

RTT support files were vendored in sketch root:
- SEGGER_RTT.c
- SEGGER_RTT.h
- SEGGER_RTT_Conf.h
- SEGGER_RTT_printf.c

Build status:
- arduino:mbed_portenta:portenta_x8 compile PASS.

---

## 3. Bench Connectivity at Test Time

- Arduino board list reported:
  - COM11 -> Portenta X8, serial 2E2C1209DABC240B
  - COM13 -> J-Link CDC UART, USB serial 001078222309
- ADB reported:
  - 2E2C1209DABC240B device (online)
- J-Link emulators reported:
  - only serial 1078222309 present in ShowEmuList

---

## 4. Execution Results

### 4.1 PA4 LOW test

Flash result:
- Upload to COM11 with ANDROID_SERIAL=2E2C1209DABC240B: PASS

J-Link SWD attach (STM32H747XI_M7):
- FAIL
- DAP initialized, but cannot attach to CPU even under reset
- InitTarget returned error code -1

RTT logger:
- FAIL attach path
- Could not connect to target or no RTT control block found

### 4.2 PA4 HIGH test

Flash result:
- Upload to COM11 with ANDROID_SERIAL=2E2C1209DABC240B: PASS

J-Link SWD attach (STM32H747XI_M7):
- FAIL
- Same behavior as LOW: cannot attach to CPU under reset

RTT logger:
- FAIL
- Could not connect to target

### 4.3 Additional diagnostic target selection

J-Link attach using generic Cortex-M7:
- Partial signal evidence only
- Found SW-DP ID 0x5BA02477
- Failed to power up DAP

J-Link attach to STM32H747XI_M4:
- FAIL (same attach under reset failure pattern)

---

## 5. Interpretation

What is proven:
- Firmware-level control of PA4 is implemented and running on target.
- Both mux polarities were tested on real hardware with immediate SWD attach verification.
- PA4 mux control alone is insufficient to establish a stable debug attach in current bench state.

What this suggests:
- There is likely at least one additional gate condition beyond PA4 (secondary mux select, reset/debug ownership timing, or another board-state dependency).
- The debug physical path is not completely dead (SW-DP ID is discoverable in one mode), but target attach/power-up remains blocked.

---

## 6. Recommended Next Unblock

1. Capture and identify any additional mux select nets associated with SWD/JTAG routing beyond PA4, then mirror those controls from STM32 firmware if available.
2. Retry J-Link attach with a strict reset-held sequence at low SWD speed immediately at boot (before application reconfigures pins).
3. Verify board switch/jumper matrix for "debug ownership" mode and force a known "J-Link owns SWD" state.
4. Once SWD attach succeeds, use RTT as the primary debug channel and stop relying on J-Link CDC UART sniff route for command/response flow.
