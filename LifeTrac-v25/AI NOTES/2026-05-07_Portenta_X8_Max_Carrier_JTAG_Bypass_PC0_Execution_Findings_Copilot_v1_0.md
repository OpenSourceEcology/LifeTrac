# Portenta X8 Max Carrier JTAG Bypass via PC0 - Execution Findings (Copilot v1.0)

Date: 2026-05-07
Author: GitHub Copilot
Scope: Execute the updated PC0-based JTAG/SWD isolation-bypass hypothesis and verify SWD/RTT access.

---

## 1. Objective

Validate whether STM32 pin PC0 (JTAG/SWD isolation control net per updated schematic interpretation) unlocks J-Link attach and RTT visibility on Max Carrier debug micro-USB.

---

## 2. Firmware Under Test

Sketch:
- LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_uart_route_probe/x8_uart_route_probe.ino

Applied control logic:
- PA4 kept deterministic for UART sniff mux path:
  - pinMode(PA_4, OUTPUT)
  - digitalWrite(PA_4, LIFETRAC_UART_SNIFF_MUX_LEVEL) with default HIGH
- PC0 used as JTAG/SWD isolation control under test:
  - pinMode(PC_0, OUTPUT)
  - digitalWrite(PC_0, LIFETRAC_JTAG_SWD_ISO_LEVEL)
- RTT boot message updated to print both PA4 and PC0 states.

Tested variants:
- Variant A: PC0 LOW (default compile-time value)
- Variant B: PC0 HIGH (compiled with `-DLIFETRAC_JTAG_SWD_ISO_LEVEL=HIGH`)

---

## 3. Execution Sequence

1. Compile and upload PC0 LOW image to COM11 (ANDROID_SERIAL=2E2C1209DABC240B).
2. Run J-Link SWD attach test on probe 1078222309 (STM32H747XI_M7, SWD, 4000 kHz).
3. Run RTT logger channel 0 test on same probe.
4. Compile PC0 HIGH variant via extra build flag and upload to COM11.
5. Re-run SWD attach test.
6. Re-run RTT logger test.

---

## 4. Results

### 4.1 PC0 LOW

Upload:
- PASS (`.elf pushed`, COM11 re-enumerated)

SWD attach:
- FAIL
- `DAP initialized successfully`, but `Can not attach to CPU`
- `Connecting to CPU via connect under reset failed`
- `InitTarget() returned error code -1`

RTT:
- FAIL
- `ERROR: Could not connect to target.`

### 4.2 PC0 HIGH

Upload:
- PASS (high-define binary compiled and pushed successfully)

SWD attach:
- FAIL
- Same failure class and signature as LOW:
  - DAP init OK
  - CPU attach fails under reset
  - `InitTarget() returned error code -1`

RTT:
- FAIL
- `ERROR: Could not connect to target.`

---

## 5. Interpretation

What this run proves:
- PC0 isolation-control hypothesis was implemented and tested in both polarities.
- Firmware deployment path is healthy enough to iterate quickly.
- PC0 polarity alone does not clear the SWD attach blocker in current bench state.

Combined with previous PA4 low/high runs:
- Neither PA4-only nor PC0-only polarity sweeps have unlocked stable target attach.
- The debug path likely has additional gating/state conditions (secondary control net, ownership mode, or reset/debug timing dependency).

---

## 6. Suggested Next Unblock Path

1. Identify all remaining debug-path gates in schematic nets beyond PA4 and PC0, then drive them deterministically from firmware.
2. Attempt attach immediately after power-up or held reset, before sketch GPIO init side effects.
3. Validate board-level debug ownership switches/jumpers against expected J-Link-to-H747 mode.
4. If possible, compare attach behavior using external SWD probe directly at test pads/header to isolate carrier routing logic from OB probe path.
