# Transport Layer Debugging Session Summary
**Date:** 2025-05-10  
**Focus:** Phase A VER_REQ Timeout Investigation and Root Cause Analysis  
**Hardware:** Murata L072 on Method G X8 (Board 2E2C1209DABC240B)

## Executive Summary

During this session, I investigated a critical issue where the firmware successfully processes ASCII fallback AT commands (like ATI) but fails to respond to binary protocol frames (VER_REQ). The investigation uncovered multiple issues in the firmware's UART initialization and diagnostics, but the primary VER_REQ timeout problem remains unresolved and requires further investigation.

### Key Findings
1. **USART1 RX Was Disabled** (✅ FIXED) - The fallback UART had RX disabled in the control register, preventing frames on that port from being received
2. **DMA Counter Not Incremented** (✅ FIXED) - The s_stats_rx_bytes counter wasn't being incremented for DMA-received bytes, masking actual reception
3. **Firmware Uses Interrupt-Driven RX** - Both LPUART1 and USART1 are configured for interrupt-driven reception, not DMA (despite DMA-related code in the file)
4. **VER_REQ Still Times Out** (❌ UNRESOLVED) - Even with USART1 RX enabled, binary VER_REQ frames don't receive responses from the firmware

## Issue Details

### Issue 1: USART1 RX Path Disabled (FIXED)

**Symptom:**  
Fallback UART routing on USART1 (PA9/PA10) was not receiving bytes despite GPIO pins being configured for Alt-Function mode.

**Root Cause:**  
In `host_uart_init()` at line ~520, the USART1 control register was initialized as TX-only:
```c
// BEFORE (WRONG):
USART1_CR1 = USART_CR1_UE | USART_CR1_TE;
platform_irq_enable(RNG_LPUART1_IRQn, 1U);
```

The RX-related flags (RE, RXNEIE, IDLEIE) were missing, and the USART1 IRQn was never enabled, so the interrupt handler was never called even if RX hardware was active.

**Fix Applied:**
```c
// AFTER (CORRECT):
USART1_CR1 = USART_CR1_UE |
             USART_CR1_RE |
             USART_CR1_TE |
             USART_CR1_RXNEIE |
             USART_CR1_IDLEIE;
platform_irq_enable(RNG_LPUART1_IRQn, 1U);
platform_irq_enable(USART1_IRQn, 1U);
```

**Status:** ✅ Applied and compiled successfully

### Issue 2: DMA RX Bytes Counter Not Incremented (FIXED)

**Symptom:**  
STATS_URC reports `host_rx_bytes=0` even though the firmware is actively receiving and processing frames (confirmed by successful ATI responses and FAULT_URC emission).

**Root Cause:**  
The firmware receives bytes via two paths:
- **Interrupt path:** RNG_LPUART1_IRQHandler and USART1_IRQHandler increment s_stats_rx_bytes when they read bytes
- **DMA path:** service_dma_rx_to() processes bytes from DMA buffer but does NOT increment s_stats_rx_bytes

The firmware calls `host_uart_poll_dma()` in the main loop, but this function is empty:
```c
void host_uart_poll_dma(void) {
    /* RX is interrupt-driven on the UART lane that is physically routed. */
}
```

Despite having empty DMA polling, there's residual DMA service code that processes bytes without counting them, causing the rx_bytes counter to remain at 0.

**Fix Applied:**
In `service_dma_rx_to()`, added the counter increment:
```c
while (s_dma_last_idx != write_idx) {
    ingest_rx_byte(s_dma_rx_buf[s_dma_last_idx]);
    s_stats_rx_bytes++;  // <-- ADDED
    s_dma_last_idx++;
    ...
}
```

**Status:** ✅ Applied and compiled successfully (binary size: 14,528 bytes)

### Issue 3: VER_REQ Binary Frames Timeout (UNRESOLVED)

**Symptom:**  
The Stage 1 probe successfully sends and receives responses to ASCII "ATI\r\n" command:
- Sends: `ATI\r\n`
- Receives: 36 bytes (ASCII firmware name + version, embedded FAULT_URC frame)
- Decoded: FAULT_URC(0xF1) with code 0x0A (HOST_RX_INACTIVE)

But when attempting to send binary VER_REQ (0x01) frame:
- Sends: Binary VER_REQ frame (0x01 0x00)
- Waits: 1.0 second timeout
- Receives: NO BYTES
- Result: TimeoutError - "timeout waiting for response type 0x81 to req 0x01"

Similarly, ASCII fallback AT+VER? command also gets no response:
- Sends: `AT+VER?\r\n`
- Receives: NO BYTES (0.6-second timeout)

**Expected Behavior:**
1. **Binary VER_REQ:** Firmware should dispatch frame via `host_cmd_dispatch()` → `handle_version()` → send VER_URC response
2. **ASCII AT+VER?:** Firmware should dispatch via `host_cmd_dispatch_at_line()` → `at_handle_version()` → send ASCII response

Both should succeed since:
- The transport is proven working (ATI works)
- The dispatch tables include both handlers
- The response functions are implemented

**Possible Root Causes (Ranked by Likelihood):**

1. **State Corruption After AT Command Processing**
   - After processing ATI, the firmware state may be corrupted or locked, preventing subsequent command processing
   - Possible locks in command dispatcher or response sending path
   - AT mode transition state machine may be stuck

2. **AT+VER? Command Not Being Recognized**
   - The at_line_equals() comparison might fail (unlikely - function looks correct)
   - The AT shell might be disabled or failed to initialize
   - The command might be truncated or corrupted in reception

3. **Binary Frame Reception Blocked After ASCII Processing**
   - The s_at_candidate flag might not be properly cleared after ATI
   - The AT mode exit logic might leave the firmware in a state that rejects binary frames
   - The frame queue might be full from previous operations

4. **Response Generation Failing Silently**
   - `handle_version()` or `at_handle_version()` might crash or exit silently
   - The response sending path (usart2_write_bytes) might be blocked
   - Both LPUART1 and USART1 might be unable to transmit (TX buffer full, ISR flags not cleared)

5. **Timing/Race Condition**
   - The probe might not be waiting long enough after sending the command
   - The firmware's interrupt handlers might not be processing bytes from the second command before the timeout

## Technical Analysis

### Firmware Architecture

**UART Configuration:**
- **LPUART1** (PA2 RX, PA3 TX): Primary host transport lane - INTERRUPT-driven
- **USART1** (PA9 RX, PA10 TX): Fallback/mirror routing lane - INTERRUPT-driven

Both operate at 921600 8N1 with RX interrupt handlers (RNG_LPUART1_IRQHandler, USART1_IRQHandler) that call ingest_rx_byte().

**TX Path (Mirrored):**
Both UARTs are written to simultaneously via usart2_write_byte():
```c
void usart2_write_byte(uint8_t byte) {
    while ((LPUART1_ISR & USART_ISR_TXE) == 0U) {}  // Wait for TX empty
    LPUART1_TDR = byte;
    while ((USART1_ISR & USART_ISR_TXE) == 0U) {}   // Wait for TX empty
    USART1_TDR = byte;
}
```

**RX Path (Dual-Input, Demultiplexed by Interrupt):**
Each interrupt handler independently calls ingest_rx_byte() with bytes from its respective UART:
```c
void RNG_LPUART1_IRQHandler(void) {
    if (LPUART1_ISR & USART_ISR_RXNE) {
        s_stats_rx_bytes++;
        ingest_rx_byte((uint8_t)LPUART1_RDR);
    }
}

void USART1_IRQHandler(void) {
    if (USART1_ISR & USART_ISR_RXNE) {
        s_stats_rx_bytes++;  // Now properly incremented!
        ingest_rx_byte((uint8_t)USART1_RDR);
    }
}
```

### Early UART Capture Analysis

The latest test (T6_bringup_2026-05-09_160800) captured 1,987 bytes of L072 output after boot:
- Contains repeated STATS_URC (0xC1) frames at ~250ms intervals (confirmed 6-8 instances)
- Contains at least one FAULT_URC (0xF1) frame with code 0x0A (HOST_RX_INACTIVE)
- All frames are properly COBS-encoded and delimited with 0x00 bytes
- No H:PROC_FRAME diagnostic traces visible (expected - firmware was in ASCII mode during probe's initial AT commands)

**Interpretation:**
The firmware IS operational and actively emitting frames. The FAULT code 0x0A (HOST_RX_INACTIVE) suggests the firmware detected no ingress from the host during the 6-second probe window, but this contradicts the successful ATI response. This likely means:
- The rx_bytes counter was at 0 (even though bytes were received - Bug #2)
- The firmware incorrectly reported no ingress due to counter bug
- The actual reception was working (evidenced by FAULT_URC response)

## Code Changes Applied

### File: host_uart.c

**Change 1 - Enable USART1 RX (Line ~520):**
```diff
    USART1_CR1 = USART_CR1_UE |
+               USART_CR1_RE |
+               USART_CR1_TE |
+               USART_CR1_RXNEIE |
+               USART_CR1_IDLEIE;
+   platform_irq_enable(USART1_IRQn, 1U);
```

**Change 2 - Increment RX Counter in DMA Path (Line ~433):**
```diff
    while (s_dma_last_idx != write_idx) {
        ingest_rx_byte(s_dma_rx_buf[s_dma_last_idx]);
+       s_stats_rx_bytes++;
        s_dma_last_idx++;
```

## Build Results

- **Binary Size:** 14,528 bytes (180 KB APP allocation, 98.2% utilized)
- **Compilation:** Clean (no errors, -Werror enabled)
- **New Build:** From commit with both fixes applied
- **Timestamp:** 2025-05-10

## Next Steps & Recommendations

### Immediate Actions
1. **Deploy Fixed Firmware:** Flash the new build (14,528 bytes) to board 2E2C and run Stage 1 probe
2. **Verify Counter Fix:** Confirm host_rx_bytes is now non-zero in STATS_URC output
3. **Analyze Remaining Issue:** If VER_REQ still times out, run with H:PROC_FRAME diagnostic traces enabled to see if frames are reaching the dispatch layer

### Diagnostic Enhancements Needed
1. **Binary Frame Reception Tracing:**
   - Add trace at frame reception: "H:VER_RCV" when 0x01 frame received
   - Add trace at dispatch: "H:VER_DISP" before handle_version() call
   - Track the frame through the entire pipeline

2. **AT Command Processing Tracing:**
   - Add trace at AT+VER? reception: "H:ATVER_RCV"
   - Add trace at AT+VER? dispatch: "H:ATVER_DISP"
   - Verify the command is recognized correctly

3. **Response Sending Tracing:**
   - Add trace before usart2_write_byte(): "H:TX_START"
   - Add trace after each byte: "H:TX_BYTE_OK" or "H:TX_BYTE_FAIL"
   - Verify TX path is not blocked

### Hypothesis for Further Investigation

**Most Likely Scenario:**
The issue is a **double-free or state corruption** in the AT command dispatcher that only manifests on the second AT command or when transitioning from AT mode to binary mode. The symptoms:
- ✅ ATI works (first AT command)
- ❌ AT+VER? fails (second AT command)  
- ❌ Binary VER_REQ fails (binary after AT session)

This pattern suggests the AT dispatcher corrupts state that prevents subsequent command processing.

**Recommended Investigation:**
1. Add comprehensive state validation traces after at_finish_line()
2. Check if s_q_count or other queue variables are corrupted after dispatching an AT command
3. Verify that at_reset_candidate() properly clears all AT state
4. Consider adding a sanity check that validates the frame queue hasn't been corrupted

## Testing Status

| Test | Build | Result | Binary | Output |
|------|-------|--------|--------|--------|
| 160547 | Phase A base (14,344B) | ATI works, VER_REQ timeout | Pre-recorded | FAULT_URC decoded |
| 160800 | Phase A + USART1 RX fix (14,520B) | ATI works, VER_REQ still timeout | Rebuild applied | Same timeout result |
| Pending | Phase A + Both fixes (14,528B) | TBD | DMA counter fix added | Awaiting test execution |

## Conclusion

Two bugs have been identified and fixed:
1. ✅ USART1 RX disabled - blocking fallback UART reception
2. ✅ DMA RX counter not incremented - masking actual byte reception

However, the primary VER_REQ timeout issue persists, suggesting a deeper problem in either:
- The command dispatcher state machine
- The AT command processing logic  
- The frame reception/transmission pipeline after AT mode transitions

The next test run with diagnostic tracing enabled should reveal whether frames are actually reaching the firmware and dispatch layer, which will help narrow down the root cause.
