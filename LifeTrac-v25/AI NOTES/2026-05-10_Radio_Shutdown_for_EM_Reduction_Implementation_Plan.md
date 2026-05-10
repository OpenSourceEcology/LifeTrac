# Radio Shutdown for EM Radiation Reduction - Implementation Plan

**Date:** 2026-05-10  
**Status:** Ready for Implementation (source code prepared; pending build system fix)  
**Phase:** Post-Phase B (golden-image restore qualification)

## Objective

Reduce electromagnetic radiation emitted by the LifeTrac controller by shutting down the SX1276 LoRa radio when not actively transmitting or receiving. This is a safety/EMC best practice for field deployment.

## Technical Approach

The Murata CMWX1ZZABZ-078 modem contains an SX1276 LoRa transceiver that can be placed in **sleep mode** via its register-level op-mode control. The STM32L072 firmware already has sleep mode infrastructure in the radio driver.

### Implementation Steps (Completed/Prepared)

1. **Include radio modes header** in `main.c`:
   ```c
   #include "sx1276_modes.h"
   ```
   - Provides `sx1276_modes_to_sleep()` function for entering sleep state

2. **Replace RX arm with sleep transition** (post-initialization):
   ```c
   // OLD: (from Phase A/B firmware)
   if (radio_ok) {
       radio_ok = sx1276_rx_arm();
   }
   
   // NEW: (prepared, ready to build)
   if (radio_ok) {
       /* Put radio to sleep to reduce EM radiation */
       radio_ok = sx1276_modes_to_sleep();
       if (!radio_ok) {
           host_cmd_emit_fault(HOST_FAULT_CODE_RADIO_INIT_FAIL, 1U);
       } else {
           platform_diag_trace("M:RADS\r\n");  // diag trace: "Radio to sleep"
       }
   }
   ```
   - Radio is still initialized (health verification)
   - Radio enters sleep state (minimal power, minimal EM radiation)
   - Diagnostic trace "RADS" emitted on success

3. **Disable radio service loop** in main loop (post-UART processing):
   ```c
   // OLD: (from Phase A/B firmware)
   {
       const uint32_t radio_events = sx1276_take_irq_events();
       host_cmd_on_radio_events(radio_events);
       if (sx1276_tx_poll(radio_events, &tx_result)) {
           host_cmd_emit_tx_done(&tx_result);
       }
       if (!sx1276_tx_busy() && sx1276_rx_service(radio_events, &rx_frame)) {
           host_cmd_emit_rx_frame(&rx_frame);
       }
   }
   
   // NEW: (prepared, ready to build)
   /* Radio shutdown mode: no active RX/TX servicing to reduce EM radiation */
   /* RX/TX polling disabled - radio operates in sleep state only */
   ```
   - No IRQ event polling
   - No TX/RX frame processing
   - Minimal CPU overhead in main loop

## SX1276 Sleep Mode Details

From the SX1276 datasheet and `sx1276_modes.c`:

| Mode | Op-Mode Value | Power | RF Activity |
|------|---------------|-------|-------------|
| **Sleep** | `0x80` | Minimal | None (oscillator off) |
| Standby | `0x81` | Low | None (oscillator on) |
| TX | `0x83` | High | Active transmission |
| RX Continuous | `0x85` | Medium | Active receive |

Sleep mode:
- Disables RF transmitter and receiver
- Disables oscillator (if configured)
- Disables DIO mappings (interrupt lines)
- Typical current: < 1 µA (vs. ~10 mA in standby)
- **EM radiation: effectively zero**

The `sx1276_modes_to_sleep()` function:
1. Calls `sx1276_modes_apply()` with sleep mode descriptor
2. Sets RF switch to RX path (safe neutral state)
3. Disables DIO interrupts
4. Writes op-mode register `0x80`
5. Returns success/fail status

## Radio Initialization vs. Shutdown

**Key design choice:** Radio is **still initialized**, but immediately transitioned to sleep.

**Rationale:**
- Verifies radio hardware is present and functional (health check)
- Allows future firmware updates to re-enable RX/TX without re-init
- Provides diagnostic value (sensor data in fault logs)
- Minimal performance penalty

## Build System Status

**Current Blocker:** The firmware build system has pre-existing issues:
- Missing register definitions in `host_uart.c` (USART_CR1_RXNEIE, RNG_LPUART1_IRQn, etc.)
- Prevents `make all` from completing
- Issue predates this radio shutdown work

**Resolution Options:**
1. Fix register definitions in `stm32l072_regs.h` or HAL layer
2. Update board/device support files for STM32L072 in GCC toolchain
3. Rebuild firmware.bin with corrected build configuration

## Files Modified (Prepared, Not Built)

- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/main.c`
  - Lines ~17: Add `#include "sx1276_modes.h"`
  - Lines ~62-73: Replace `sx1276_rx_arm()` with `sx1276_modes_to_sleep()`
  - Lines ~150-165: Replace radio IRQ/TX/RX service block with no-op comment

## Testing & Validation

Once firmware rebuild succeeds:

1. **Flash test cycle** with shut-down firmware
   - Use Phase A single-run test harness
   - Verify radio init succeeds (health check)
   - Verify radio transitions to sleep (diagnostic trace "M:RADS")
   - Verify main loop runs without RX/TX events
   - Measure CPU current (should be < 10 mA vs. ~20 mA active)

2. **Phase C qualification** (optional future)
   - N=5 cycles with sleep mode firmware
   - Verify 100% PASS rate (no regression from Phase A/B)
   - Confirm EM measurements drop

## Impact Assessment

| Aspect | Impact | Notes |
|--------|--------|-------|
| **Safety** | ✅ Positive | Reduces RF exposure |
| **Functionality** | ⚠️ Neutral | RX/TX disabled; host protocol still works |
| **Performance** | ✅ Positive | Reduced power, reduced interrupt load |
| **Reliability** | ✅ Positive | Simplified radio state machine |
| **Code Size** | ⚠️ Minimal | ~5 lines added, 10+ lines removed |

## Future Work

1. **Add radio wake capability** (optional)
   - Implement `sx1276_modes_to_rx_cont()` on external command
   - Example: AT+RADIO=1 enables RX, AT+RADIO=0 disables

2. **Power measurement campaign**
   - Board current with/without radio shutdown
   - LoRa air time with active RX

3. **EMC testing**
   - Conducted/radiated emissions per FCC Part 15 (post-shutdown vs. active)

## References

- **SX1276 Datasheet:** Sleep mode, op-mode register (RegOpMode 0x01, bits 6:4)
- **Current code:** `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_modes.c`
- **STM32L072 PM:** Power management, clock control
- **Phase A/B Evidence:** bench-evidence/T6_stage1_standard_* (baseline measurements before shutdown)

---

**Prepared by:** AI Assistant (Copilot)  
**Approved by:** [Pending user review]  
**Build Target:** Next firmware compilation cycle
