# LoRa Firmware Implementation Plan Review (Method G)

**Date:** 2026-05-05
**Author:** GitHub Copilot (Gemini 3.1 Pro)
**Target:** `LifeTrac-v25/DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/` Implementation Plan
**Scope:** High-level architectural decisions, capabilities analysis, roadmap, and proposed code structures for the Murata STM32L072CZ custom firmware.

---

## 1. Big Picture Decisions Review

### 1.x Commitment to Method G (Custom L072 Firmware)
**Verdict:** Outstanding pivot. Bypassing the AT firmware (which adds an unpredictable 10–20ms parser tax) fundamentally enables the sub-millisecond latencies necessary for your v25 control plane. Repurposing the Murata SiP’s internal SPI bus requires no BOM changes, preserving hardware uniformity while unlocking massive flexibility.

### 1.x Protocol Location Shift (H7 → L072)
**Verdict:** Sound architectural decoupling. Shifting the PHY-level radio management (SF ladder, FHSS, P0 preempt, listen-before-talk) into the L072 allows the H7 to focus strictly on payload prioritization and application state. The L072 becomes a highly specialized smart-transceiver. 

### 1.x No LoRaWAN MAC
**Verdict:** Necessary and correct. LoRaWAN duty-cycle regulations and Class-A/C receive window behaviors natively conflict with the immediate, high-priority, preemptable traffic needed for LifeTrac's P0 ControlFrames.

### 1.x Defense-in-Depth Brick Recovery
**Verdict:** Phenomenal. The 5-layer plan (NRST -> UART "SAFE" sequence -> BOOT0+NRST -> physical SW1 -> SWD) combined with the H7 hosting a "golden binary" ensures that field updates and dev iterations are effectively unbrickable. Implementing this in Phase 1 ensures no time is wasted later un-wedging modules.

---

## 2. Proposed Architecture & "Code" Structure Review

*(Note: While the `firmware/murata_l072/` directory and C/C++ files are not yet created, this reviews the explicit design patterns requested in the specification.)*

### 2.x COBS-Framed UART Transport
The decision to use Consistent Overhead Byte Stuffing (COBS) over a 921600 8N1 UART link is excellent.
- **Why it works:** COBS eliminates the need for complex escape-character state machines in the L072. Combined with length prefixes, the L072 knows exactly how many bytes to grab, enabling perfect pairing with STM32's `DMA-on-IDLE` interrupt.
- **Resilience:** The addition of a CCITT-FALSE CRC16 polynomial will catch wiring noise or UART framing drift that could otherwise manifest as mysterious downstream logic bugs.

### 2.x Bare-Metal C & Static Allocation
Operating within a strict 192KB Flash / 20KB RAM budget requires discipline. 
- **Proposed approach:** Zero `malloc`, static allocation, pre-sized circular buffers, and dropping exceptions/RTTI are standard embedded best practices that will easily keep this firmware under the ~8KB targeted RAM usage.
- **Crypto:** If the H7 handles AES-GCM (as planned/default), TinyCrypt is acceptable. If N-12 is realized and crypto shifts to the L072, you must leverage the hardware AES engine on the L072CZ rev Y+ rather than burning CPU cycles.

### 2.x Hardware Timer Triggers (N-02)
Using the `TIM2` 32-bit timer to drive the SX1276 TX trigger is a massive upgrade over software polling. Realizing microsecond-accurate time-of-air anchors facilitates complex TDMA or slotted communications later (Phase 7+).

---

## 3. Risks & Recommendations

1. **H7 Boot Glitching (BOOT0 / NRST):**
   - *Risk:* During power-on-reset of the H7, its GPIO pins will float before being configured. If the BOOT0 or NRST lines float to an undesirable state, the L072 could accidentally enter the ROM bootloader.
   - *Recommendation:* Ensure `LORA_BOOT0` and `LORA_NRST` have appropriate hardware pull-downs/pull-ups on the carrier board, or verify the Murata module's internal strapping prevents floating states.
2. **IWDG Configuration (N-20):**
   - *Risk:* Feeding the Independent Watchdog from a high-priority UART or SysTick interrupt. If the main L072 control loop hangs while interrupts continue firing, the IWDG will never reset the chip.
   - *Recommendation:* The `iwdg_kick()` should only occur at the end of the `main()` while loop, strictly confirming that all state machines correctly iterated.
3. **921600 Baud Tolerance:**
   - *Risk:* The clock configurations (HSI/MSI/PLL) on both the H7 and L072 must be highly accurate to support a reliable 921.6 kbps UART span. Even a 2-3% clock drift from temperature/voltage variations can cause framing errors.
   - *Recommendation:* Use the L072's HSI16 (High-Speed Internal) with clock autocalibration via the CRS (Clock Recovery System) leveraging the LSE (Low-Speed External 32.768 kHz) if equipped on the SiP, ensuring UART baud drift remains near 0%.
4. **Phase 6 "Field Update" Complexity:**
   - Jumping straight to A/B firmware bank swaps creates high linker-script and vector-table offset complexity. Ensure `SCB->VTOR` is correctly relocated immediately upon booting slot B, and that interrupts are completely disabled before the jump.

## Conclusion
The Method G documentation is mature, robust, and correctly prioritizes reliability over rushed delivery. The choice to fully decouple and assume ownership of the Murata SiP's internal SX1276 provides a permanent pathway to absolute RF control, removing the black-box variability of vendor firmware. Proceed to Phase 0.