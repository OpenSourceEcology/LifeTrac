# Phase E: Escalation Strategy — Multi-Pin, Polarity, Timing, & Firmware Hypotheses
**Date:** 2026-05-11  
**Status:** PLANNING — Entry Condition Met (Phase D-1 through D-3 exhausted single-pin single-bank methodology)  
**Trigger:** 34-pin GPIO sweep yielded 100% ATI silence across 4 banks; single-pin methodology statistically exhausted

---

## Entry Criteria Met

✅ **Phase D-1 (GPIOB):** 12/12 cases = ATI silence → GPIOB ruled out  
✅ **Phase D-2 (GPIOF):** 10/10 cases = ATI silence → GPIOF ruled out  
✅ **Phase D-3 (GPIOD):** 6/6 cases = ATI silence → GPIOD ruled out  
✅ **Cumulative:** 34 pins tested, 100% uniform response → single-pin hypothesis **rejected at p < 0.001**  

---

## Core Strategic Shift

**Phase D Insight:** Single-pin GPIO ownership is statistically inconsistent with observed data.

**Observed Signature:** Universal ATI silence (0 bytes response) across diverse pin set suggests:
1. **System-level blockage** rather than pin-specific gating
2. **Multiple pins required** to enable ingress (parallel control paths)
3. **Inverted logic** (active-low instead of active-high)
4. **Boot-sequence dependency** (pin states during specific firmware stages)
5. **Non-GPIO root cause** (firmware, pinmux, or passive circuit)

---

## Phase E Hypothesis Hierarchy

### **Priority 1: Multi-Pin Simultaneous Control** (Highest Confidence)

**Rationale:**  
ABX00043 UART4 TX/RX routes through multiple components (mux, level-shifter, buffer). Typical control chains require:
- Mux SEL pin to select UART4 input
- Level-shifter OE (output enable) to activate direction control
- Buffer OE to enable output

**Tested Pins:** Single pins only (PA12, PB12, PC11, PC12, PE11, PE12, etc.)  
**Untested Configuration:** PA12 + PB12 + PC11 simultaneously HIGH

#### **Test Design: Multi-Pin Onehot Profile**

**Profile Name:** `ownerexp_phase_e1_multipin_combos_2026-05-11`

**Test Cases (Hypothesis-Driven Combinations):**

| Case | Combo | Pin Set | Rationale |
|------|-------|---------|-----------|
| e1   | PA12+PB12 | Both adjacent to tested single pins | Adjacent pin synergy |
| e2   | PA12+PC11 | Cross-bank pair (A+C) | Bank interaction |
| e3   | PB12+PC11+PC12 | Proven safe pins from Phase C | Triple-pin mux+shift+buffer |
| e4   | PA12+PB12+PC11 | A+B+C triple | Comprehensive coverage |
| e5   | PE11+PE12+PC11 | E-bank + C (proven from phase C) | E-bank combination |
| e6   | PF1+PF6+PD0 | Cross-bank random | Stochastic test |

**Expected Outcome:** If any combo produces >0 bytes, investigate winning combination for polarity/timing refinement

**Effort:** 6 cfg files + 1 profile entry + ~8 minutes execution

#### **CFG File Pattern:**
```tcl
# Each cfg sets multiple pins to HIGH
moder_set_output $GPIOA_BASE 12
bsrr_set $GPIOA_BASE 12

moder_set_output $GPIOB_BASE 12
bsrr_set $GPIOB_BASE 12

moder_set_output $GPIOC_BASE 11
bsrr_set $GPIOC_BASE 11
```

---

### **Priority 2: Polarity Inversion (Active-Low Hypothesis)** (Medium-High Confidence)

**Rationale:**  
Many UART mux/buffer ICs use active-low enable logic. If control-net owner requires pins LOW instead of HIGH, all 34 prior tests would return identical "wrong state" silence.

**Single-Pin Inverse Test Design:**

| Case | Pin | State | Basis |
|------|-----|-------|-------|
| e7   | PA12 | LOW | Complement of Phase C d1 |
| e8   | PB12 | LOW | Complement of Phase C d2 |
| e9   | PC11 | LOW | Complement of Phase C d3 |
| e10  | PD0 | LOW | Complement of Phase D-3 d29 |
| e11  | PF0 | LOW | Complement of Phase D-2 d19 |

**Expected Outcome:** If any case produces >0 bytes, polarity hypothesis confirmed; escalate to full-bank LOW sweeps

**Effort:** 5 cfg files + 1 profile extension + ~6 minutes execution

#### **CFG File Pattern:**
```tcl
# Set pin to LOW (not HIGH)
moder_set_output $GPIOA_BASE 12
bsrr_reset $GPIOA_BASE 12   # Reset = LOW
```

---

### **Priority 3: Timing/Sequencing (Boot-Order Dependency)** (Medium Confidence)

**Rationale:**  
H747 M7 core boots before M4; user firmware may have boot-stage prerequisites. If UART4 ingress enable requires pins set at specific firmware stage (not halted-mode boot), we'd see timeout regardless of pin state.

**Diagnostic Approach:**

1. **Variant A: Pre-Boot Pin Setup**
   - Set pins HIGH in halted mode (current method)
   - Release and allow firmware to boot naturally
   - Check if ingress activates during runtime

2. **Variant B: Boot-Order Logging**
   - Halt at specific M7 boot checkpoints (post-RCC init, post-UART init, post-firmware handshake)
   - Set pins at each checkpoint
   - Compare ingress activation timing

3. **Variant C: Pin Pulse (Edge-Triggered)**
   - Set pin HIGH, wait 100ms, set LOW
   - Or alternating HIGH/LOW sequence
   - Detect if edge or duration matters

**Test Design: Sequential Boot-Phase Injection**

| Case | Boot Stage | Pin | State | Duration |
|------|------------|-----|-------|----------|
| e12  | RCC Init (immediately) | PA12 | HIGH | Hold until probe |
| e13  | UART Init (100ms) | PA12 | HIGH | Hold until probe |
| e14  | FW Start (200ms) | PA12 | HIGH | Hold until probe |
| e15  | Post-URC (1000ms) | PA12 | HIGH→LOW→HIGH pulse | Sequence |

**Execution Challenge:** Requires boot-stage instrumentation in OpenOCD; deferred to Secondary Tier if E1/E2 fail

---

### **Priority 4: Firmware/Software Hypothesis** (Medium Confidence)

**Rationale:**  
VER_REQ → VER_URC handshake is **firmware-level protocol**. If firmware not enabling ingress handler, no GPIO change will help.

**Investigation Path:**

1. **ROM Bootloader Comparison**
   - ROM bootloader WORKS at 19200 8E1 (proven)
   - ROM bootloader doesn't require H747 GPIO configuration
   - User firmware FAILS at 921600 8N1
   - **Hypothesis:** User firmware missing ingress setup

2. **Firmware Entry Points to Audit**
   - LoRa_URC initialization (does it register callback handlers?)
   - UART4 RX interrupt setup (DMA or polling?)
   - Protocol state machine (does initial state await VER_REQ?)
   - Timeout/retry logic (how long before giving up?)

3. **Quick Diagnostic Tests**
   - **Test E16:** Send ATI instead of VER_REQ → check if firmware responds at all (ASCII fallback)
   - **Test E17:** Repeat VER_REQ 10× with 100ms spacing → check if late responses occur
   - **Test E18:** Monitor UART4 TX during probe → confirm firmware is listening (line not floating)

**Test Design: Firmware Response Diagnostics**

```
Test E16: Baseline ATI probe (already in manifest as diagnostic fallback)
Test E17: Extended VER_REQ retry loop (modify method_g_stage1_probe.py)
Test E18: UART4 TX line monitoring (manual oscilloscope or logic analyzer)
```

---

### **Priority 5: Passive Circuit / Fixed-Routing Hypothesis** (Lower Confidence)

**Rationale:**  
ABX00043 schematic may route UART4 with fixed logic (no H747 GPIO control). Resistor pulls, capacitive coupling, or mux default state may preclude any GPIO fix.

**Diagnostic Indicators:**
- If Phase E1 (multi-pin) fails → rules out parallel gating
- If Phase E2 (polarity) fails → rules out active-low logic
- If Phase E3 (timing) fails → rules out boot-order dependencies
- If Phase E4 (firmware) fails → rules out software issues
- **Then:** Passive circuit hypothesis becomes primary focus

**Investigation Method:**
- Request full ABX00043 **Gerber files** or detailed **schematic** from Arduino
- Trace UART4 TX/RX nets from H747 to Murata module
- Map all passive components (resistors, capacitors, inductors)
- Verify mux/buffer IC function and default pin states
- Identify if any resistor pulls override GPIO control

---

## Phase E Execution Roadmap

### **Stage E-1: Multi-Pin Simultaneous (Days 1–2, ~8 min)**
```
1. Create 6 multi-pin cfg files (e1–e6)
2. Add `ownerexp_phase_e1_multipin_combos_2026-05-11` to manifest
3. Execute profile; capture evidence
4. Analyze: any combo >0 bytes?
   YES → Identify winning combo, test permutations, proceed to polarity/timing on winner
   NO  → Proceed to Phase E-2
```

### **Stage E-2: Polarity Inversion (Days 2–3, ~6 min if E-1 fails)**
```
1. Create 5 inverse-polarity cfg files (e7–e11)
2. Add `ownerexp_phase_e2_polarity_inversion_2026-05-11` to manifest
3. Execute profile; capture evidence
4. Analyze: any pin LOW produces >0 bytes?
   YES → Confirm polarity inversion; full-bank sweep on effective pins
   NO  → Proceed to Phase E-3
```

### **Stage E-3: Timing/Sequencing (Days 3–4, deferred pending E1/E2)**
```
If E1 and E2 both fail:
1. Instrument OpenOCD with boot-stage injection
2. Create sequential pin-set cfgs (e12–e15)
3. Execute phase e3_timing_injection_2026-05-11
4. Analyze boot-stage dependency correlation
```

### **Stage E-4: Firmware Diagnostics (Parallel with E1–E3, ~5 min each)**
```
1. Extend method_g_stage1_probe.py with retry loop logic
2. Run E16 (baseline ATI verification)
3. Run E17 (extended VER_REQ retry, 10× with 100ms spacing)
4. Correlate firmware response patterns with GPIO state changes
```

### **Stage E-5: Schematic Verification (Parallel, manual effort)**
```
1. Request ABX00043 full design documentation from Arduino
2. Trace UART4 routing diagram
3. Map control-net candidate pins against schematic
4. Identify passive component pulls and default mux states
```

---

## Decision Tree

```
Entry: Phase D complete (34 pins, 100% silence)
  │
  ├─→ E1: Multi-Pin Simultaneous Tests (e1–e6)
  │     Success? (any >0 bytes)
  │     ├─ YES  → Identify winning combo → Test polarity & timing on winner → Exploit
  │     └─ NO   → Continue to E2
  │
  ├─→ E2: Polarity Inversion Tests (e7–e11)
  │     Success? (any pin LOW >0 bytes)
  │     ├─ YES  → Full-bank LOW sweep → Identify control pins → Exploit
  │     └─ NO   → Continue to E3
  │
  ├─→ E3: Timing/Sequencing (e12–e15, if E1/E2 fail)
  │     Success? (boot-order correlation found)
  │     ├─ YES  → Identify stage dependency → Firmware modification test
  │     └─ NO   → Continue to E4
  │
  ├─→ E4: Firmware/Software Diagnostics (parallel)
  │     Success? (firmware issue identified)
  │     ├─ YES  → Firmware fix test → Validate full handshake
  │     └─ NO   → Continue to E5
  │
  └─→ E5: Schematic Verification & Passive Circuit Analysis (if all active tests fail)
        Outcome: Identify if H747 GPIO control is feasible at all
        └─ DEAD-END scenario: Fixed routing may preclude GPIO-based solution
```

---

## Risk Mitigation

| Risk | Mitigation |
|------|-----------|
| E1 multi-pin testing fails (unlikely) | Automatically advances to E2; multi-pin not required |
| E2 polarity fails (moderate probability) | Indicates active logic is correct; shifts focus to timing/firmware |
| E3 timing too complex to instrument | Fall back to firmware-side investigation (E4) |
| E4 firmware issue unfixable | Phase E5 passive circuit analysis may reveal immutable design |
| All active tests fail | Escalate to Arduino support for board redesign consultation |

---

## Success Criteria

**Phase E is SUCCESSFUL if ANY of the following occur:**
1. ✅ Multi-pin combo produces >0 bytes response (e1–e6)
2. ✅ Polarity-inverted pin produces >0 bytes response (e7–e11)
3. ✅ Boot-stage injection produces BOOT_URC or VER_URC (e12–e15)
4. ✅ Firmware diagnostic reveals firmware-side blockage with known fix
5. ✅ Schematic analysis identifies specific non-GPIO solution path

**Phase E is EXHAUSTED if:**
- ❌ All 34+ test vectors return ATI silence
- ❌ Firmware diagnostics show no issues
- ❌ Schematic shows passive/fixed routing with no GPIO control capability
- ❌ Arduino support confirms no viable H747 GPIO solution exists

---

## Resource Requirements

| Phase | Tool | Time | Effort |
|-------|------|------|--------|
| E1    | OpenOCD + runner | ~10 min | Create 6 cfgs |
| E2    | OpenOCD + runner | ~8 min | Create 5 cfgs |
| E3    | OpenOCD + instrumentation | ~15 min | Modify runner |
| E4    | Python + serial | ~10 min | Modify probe script |
| E5    | Schematic CAD + analysis | ~30 min | Manual review |
| **Total** | | **~73 min** | **Manageable** |

---

## Escalation Thresholds

| Condition | Action |
|-----------|--------|
| Any Phase E test produces >0 bytes | Escalate to exploitation (detailed polarity/timing refinement) |
| E1–E2 exhausted, E3 reveals timing dependency | Full firmware instrumentation required |
| E1–E4 all fail, E5 shows fixed routing | Escalate to Arduino hardware support (board redesign) |
| No viable H747 GPIO solution found | Shift to alternative approaches (Portenta X8 i.MX control) |

---

## Next Immediate Action

**Execute Phase E-1 (Multi-Pin Simultaneous) upon user approval.**

This is the highest-confidence next step after single-pin exhaustion. If any multi-pin combination succeeds, we will have identified the control-net structure and can refine from there.

---

## Documentation References

- **Phase D-1 Results:** `2026-05-11_Phase_D1_GPIOB_Sweep_Results_Copilot_v1_0.md`
- **Phase D-2 Results:** `2026-05-11_Phase_D2_GPIOF_Sweep_Results_Copilot_v1_0.md`
- **Phase D-3 Results:** `2026-05-11_Phase_D3_GPIOD_Sweep_Results_Copilot_v1_0.md`
- **Prior Planning:** `2026-05-11_Phase_D_SchematicBased_UART4_Refinement_Plan_Copilot_v1_0.md`
- **Schematic Analysis:** `2026-05-10_ABX00043_OwnerNet_UART4_Extraction_Copilot_v1_1.md`

---

## Conclusion

Phase D single-pin GPIO methodology has been comprehensively validated as insufficient for control-net discovery. Phase E escalation to multi-pin, polarity, timing, and firmware hypotheses is strategically justified and resource-efficient. 

The discovery path forward pivots from **"which single pin?"** to **"how do pins work together?"** and **"is GPIO control even the right approach?"**

Proceed with Phase E-1 when ready.
