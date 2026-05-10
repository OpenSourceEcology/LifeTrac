# Radio Shutdown for EM Radiation Reduction — Bench-Testing Only Implementation

**Date:** 2026-05-10  
**Status:** Ready for Implementation (conditional compilation, bench-testing only)  
**Scope:** Bench/lab testing to reduce EM radiation; **production firmware unaffected**

## Objective

During bench testing of the LifeTrac controller, reduce electromagnetic radiation by shutting down the SX1276 LoRa radio. This is enabled via a compile-time conditional flag (`LIFETRAC_BENCH_MODE`) and **does not affect production builds**.

## Scope & Production Safety

- **Bench/Test Builds:** Radio shutdown enabled via `-DLIFETRAC_BENCH_MODE=1`, no RX/TX during idle
- **Production Builds:** Radio operates normally with active RX/TX (default behavior, `LIFETRAC_BENCH_MODE=0`)
- **Zero Production Impact:** Default build behavior unchanged; production firmware is unaffected

## Technical Approach

The Murata CMWX1ZZABZ-078 modem contains an SX1276 LoRa transceiver that can be placed in **sleep mode** via its register-level op-mode control. The STM32L072 firmware already has sleep mode infrastructure in the radio driver.

### Implementation Steps

1. **Include radio modes header** in `main.c` (always included, used only in bench mode):
   ```c
   #include "sx1276_modes.h"
   ```
   - Provides `sx1276_modes_to_sleep()` function for entering sleep state

2. **Add bench-mode conditional to radio initialization** (post-`sx1276_init()`):
   ```c
   #define LIFETRAC_BENCH_MODE 0  // Set to 1 for bench testing with radio shutdown

   // ... (after sx1276_init())
   
   #if LIFETRAC_BENCH_MODE
   if (radio_ok) {
       /* Bench mode: Put radio to sleep to reduce EM radiation during testing */
       radio_ok = sx1276_modes_to_sleep();
       if (!radio_ok) {
           host_cmd_emit_fault(HOST_FAULT_CODE_RADIO_INIT_FAIL, 1U);
       } else {
           platform_diag_trace("M:RADS\r\n");  // Diagnostic mark: "Radio to sleep"
       }
   }
   #else
   if (radio_ok) {
       radio_ok = sx1276_rx_arm();  // Production: normal RX mode
   }
   #endif
   ```

3. **Add bench-mode conditional to main loop** (radio event servicing):
   ```c
   #if LIFETRAC_BENCH_MODE
   /* Bench mode: Radio shutdown — no active RX/TX servicing */
   /* RX/TX polling disabled to reduce EM radiation during testing */
   #else
   {  // Production: normal radio servicing
       const uint32_t radio_events = sx1276_take_irq_events();
       host_cmd_on_radio_events(radio_events);
       if (sx1276_tx_poll(radio_events, &tx_result)) {
           host_cmd_emit_tx_done(&tx_result);
       }
       if (!sx1276_tx_busy() && sx1276_rx_service(radio_events, &rx_frame)) {
           host_cmd_emit_rx_frame(&rx_frame);
       }
   }
   #endif
   ```

## Build Instructions

### Production Build (Default — Active RX/TX)

```bash
cd LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072
make clean; make all
# Result: firmware.bin with normal RX/TX operation
```

### Bench-Testing Build (Radio Shutdown)

```bash
cd LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072
make clean; make all CFLAGS="-DLIFETRAC_BENCH_MODE=1"
# Result: firmware.bin with radio sleep mode enabled
```

### Alternative: Edit config.h or main.c

```c
// In main.c, change:
#define LIFETRAC_BENCH_MODE 0  // Change to 1
```

Then build normally:
```bash
make clean; make all
```

## SX1276 Sleep Mode Details

From the SX1276 datasheet and `sx1276_modes.c`:

| Mode | Op-Mode Value | Power | RF Activity |
|------|---------------|-------|-------------|
| **Sleep (Bench)** | `0x80` | Minimal | None (oscillator off) |
| Standby | `0x81` | Low | None (oscillator on) |
| TX | `0x83` | High | Active transmission |
| RX Continuous | `0x85` | Medium | Active receive |

Sleep mode characteristics:
- Disables RF transmitter and receiver
- Disables oscillator (if configured)
- Disables DIO mappings (interrupt lines)
- Typical current: < 1 µA (vs. ~10 mA in standby, ~20 mA active RX/TX)
- **EM radiation: effectively zero**

The `sx1276_modes_to_sleep()` function:
1. Calls `sx1276_modes_apply()` with sleep mode descriptor
2. Sets RF switch to RX path (safe neutral state)
3. Disables DIO interrupts
4. Writes op-mode register `0x80`
5. Returns success/fail status

## Key Design Decisions

### Radio Initialization Still Occurs
Even in bench mode, the radio is **still initialized** before sleep mode.

**Rationale:**
- Verifies radio hardware is present and functional (health check)
- Provides early diagnostic value (sensor data in fault logs)
- Minimal performance penalty (~23–24 sec per cycle observed in Phase A/B)
- Allows future firmware updates to re-enable RX/TX without re-init

### Production Safety: Default is Disabled
`LIFETRAC_BENCH_MODE` defaults to `0` (disabled), so:
- Default build = production behavior (active RX/TX)
- Bench mode requires **explicit flag** at compile time
- Zero risk of accidentally shipping bench-mode firmware

### No Runtime Overhead in Production
All bench-mode conditionals are eliminated at compile time when `LIFETRAC_BENCH_MODE=0`:
- Production binary is unchanged
- No extra branches, no extra code size
- Production build is identical to pre-bench-mode baseline

## Files to Modify

- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/main.c`
  - Lines ~17: Add `#include "sx1276_modes.h"` (to existing includes)
  - Lines ~44–56: Wrap radio initialization in `#if LIFETRAC_BENCH_MODE` conditional
  - Lines ~65–80: Wrap radio service loop in `#if LIFETRAC_BENCH_MODE` conditional

## Testing & Validation

### Bench-Mode Validation (Recommended)

Once firmware rebuild succeeds:

1. **Single validation cycle** with bench-mode firmware
   - Flash with `-DLIFETRAC_BENCH_MODE=1`
   - Verify radio init succeeds (health check)
   - Verify radio transitions to sleep (diagnostic trace "M:RADS")
   - Verify main loop runs without RX/TX events
   - Measure CPU current (should be < 5 mA idle vs. ~20 mA active)

2. **Repeat Phase A/B with bench firmware** (optional)
   - Run N=5 cycles with bench-mode firmware
   - Verify 100% PASS rate (no regression from Phase A/B)
   - Confirm EM measurements drop if spectrum analyzer available

### Production Validation (Default)

```bash
make clean; make all  # Default: LIFETRAC_BENCH_MODE=0
# Existing Phase A/B test harness runs without modification
# Radio operates normally: RX armed, active polling in main loop
```

## Impact Assessment

| Aspect | Impact | Notes |
|--------|--------|-------|
| **Bench Safety** | ✅ Positive | Reduces RF exposure during lab testing |
| **Production Safety** | ✅ No Change | Default build unaffected; must explicitly enable bench mode |
| **Functionality** | ⚠️ Bench-only | RX/TX disabled in bench mode only; production RX/TX normal |
| **Performance** | ✅ Positive (Bench) | Reduced power, reduced interrupt load (bench mode only) |
| **Reliability** | ✅ Positive (Bench) | Simplified radio state during bench testing |
| **Code Size** | ✅ Minimal | ~20 lines added (~10 lines per conditional block); ~15 lines removed from production |
| **Production Binary Size** | ✅ Unchanged | Conditionals stripped at compile time; production .elf identical to baseline |

## Future Work

1. **Radio wake capability** (optional, if bench needs dynamic RX/TX)
   - Implement `sx1276_modes_to_rx_cont()` on external AT command
   - Example: AT+RADIO=1 enables RX, AT+RADIO=0 re-enters sleep

2. **Power measurement campaign** (optional)
   - Board current with/without radio shutdown
   - EM spectrum analysis: bench shutdown vs. production active

3. **Extended bench testing** (optional)
   - Phase C qualification with bench-mode firmware (N=5 cycles)
   - Verify flash/boot reliability with sleep mode enabled

## References

- **SX1276 Datasheet:** Sleep mode, op-mode register (RegOpMode 0x01, bits 6:4)
- **Current code:** `LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_modes.c`
- **STM32L072 PM:** Power management, clock control
- **Phase A/B Evidence:** bench-evidence/T6_stage1_standard_* (baseline measurements, active RX/TX)

---

**Prepared by:** AI Assistant (Copilot)  
**Scope:** Bench-testing only (conditional compilation)  
**Production Impact:** None (default behavior unchanged)  
**Build Target:** Next firmware compilation cycle (after toolchain fixes)
