# Code Implementation Readiness Review — LifeTrac v25 Controller
Date: 2026-04-26

This review evaluates the current state of the custom LoRa stack and Modbus slave implementation (`tractor_m7.ino`, `handheld.ino`, `opta_modbus_slave.ino`, and `lora_bridge.py`) for hardware testing readiness.

## Executive Summary
**Overall Status:** RED (Not Ready for End-to-End Hardware Test)
There are multiple logic flaws, unhandled edge cases, and missing cryptographic implementations that will immediately block system operation or result in catastrophic failure during a field test. 

## 1. End-to-End Cryptography Mismatch (Critical Blocker)
* **The Problem:** The C-based nodes (`handheld.ino`, `tractor_m7.ino`, `lora_proto.c`) rely on `crypto_stub.c`, which circumvents `lp_encrypt()` and `lp_decrypt()` by using a plaintext pass-through and a fake 16-byte zero tag. Conversely, the base station (`lora_bridge.py`) uses the Python `cryptography` library (`AESGCM`), which expects and enforces a mathematically sound 16-byte authentication tag over the cipher data.
* **Impact:** The Python base station will silently discard 100% of packets received from the C nodes due to an AEAD verification failure. The Tractor and Handheld nodes will also fail to decrypt base station messages properly.
* **Resolution:** Before bench testing, the team must either implement `MbedTLS/ArduinoBearSSL` in `crypto_stub.c` OR temporarily implement a zero-tag mock on the Python side to allow cleartext bench testing.

## 2. Radio Airtime Saturation / Loop Blocking (`handheld.ino`)
* **The Problem:** The handheld controller dictates a 50ms tick rate (20 Hz). In every loop, it calls `send_frame()` twice synchronously—once for the `ControlFrame` (16 bytes) and once for the `HeartbeatFrame` (10 bytes).
* **Impact:** `RadioLib.transmit()` is blocking. Two sequential LoRa transmits at SF7/BW500 will consume ~30-40 ms, occupying almost the entire 50ms loop allowance. This limits other processing and continuously saturates the channel. 
* **Resolution:** Under `LORA_PROTOCOL.md` and the tractor's arbitration logic, `HEARTBEAT_TIMEOUT_MS` allows for up to 500 ms between heartbeats. The Handheld should only transmit heartbeats every 250-500ms (2-4 Hz), heavily prioritizing the 20 Hz control frames. Furthermore, liveness is already tracked through the `cf.heartbeat_ctr` in the `ControlFrame` itself.

## 3. Opta E-Stop Latching Bug (`opta_modbus_slave.ino`) (Critical Safety Blocker)
* **The Problem:** In `on_holding_change()`, if `g_safety_state != SAFETY_NORMAL`, all subsequent holding register writes (except the watchdog) hit a `return` and short-circuit. If the master triggers an E-Stop (`REG_ARM_ESTOP = 1`), `enter_safe_state(SAFETY_ESTOP_LATCHED)` changes the safety state. If the master later tries to clear the E-Stop by sending `REG_ARM_ESTOP = 0`, the slave's early exit block prevents handling it!
* **Impact:** Once an E-Stop is triggered, the `PIN_ENGINE_KILL` relay stays HIGH indefinitely. The system enters an unrecoverable `SAFETY_ESTOP_LATCHED` state and cannot be reset without manually power-cycling the Opta slave.
* **Resolution:** Create a dedicated clear/reset mechanism. E.g., if `REG_ARM_ESTOP` transitions from `1 -> 0`, process it even when `g_safety_state == SAFETY_ESTOP_LATCHED`, conditionally restoring the `SAFETY_NORMAL` state if all other parameters are healthy.

## 4. Dangerous Joystick-to-Valve Mapping (`tractor_m7.ino`)
* **The Problem:** The `apply_control()` maps joysticks to eight directional binary valve coils blindly using simple thresholds (`cf.axis_lh_y > 20`). Flow setpoints (`REG_FLOW_SP_1` / `2`) are scaled instantly: `abs(cf.axis_lh_y) * 78`.
* **Impact:** If moved rapidly, the hydraulic pressures will violently spike because there is zero ramping or deadband logic in place. A structural shock to the chassis or boom dropping is highly probable.
* **Resolution:** As noted in the codebase TODO comment, port the deadband, ramp, and dual-flow split logic from the `RESEARCH-CONTROLLER/arduino_opta_controller/` scripts prior to full hydraulic testing. 

## 5. Missing / Incomplete Subsystems
Several features scoped for v25 are incomplete across the DRAFT firmware:
* **Base Station Command Forwarding:** The X8 UART polling (`poll_x8_uart` in `tractor_m7.ino`) reads incoming telemetry from Linux but lacks the transmit/bridge layer to send Air-received telemetry and Air-received replies back to the Linux side, effectively making it a one-way pipe right now. 
* **Cellular Fallback & MQTT-SN Dispatch:** Stubs only.
* **Engine/Battery Telemetry:** The Modbus mapping and extraction is there for analogs, but pure engine CAN-bus data is not mapped yet to the Air interface.