# Phase D: Schematic-Based UART4 Control-Net Refinement Plan
**Date:** 2026-05-11  
**Status:** Planning document for next investigation cycle  
**Scope:** Design Phase D profiles targeting remaining GPIO candidates and strategic net-ownership hypotheses

---

## Executive Summary

Phase C definitively ruled out pins PA12, PB12, PC11, PC12, PE11, PE12 as UART4 control-net drivers via deterministic testing and signature extraction. Phase D will systematically probe remaining H747 GPIO banks and i.MX GPIO pathways to identify the actual UART4 control-net ownership.

**Key Decision Point:** Since the schematic PDF fetch is currently blocked (network restriction), Phase D will proceed via strategic systematic exploration of remaining GPIO banks in logical priority order, backed by the component topology analysis already completed.

---

## ABX00043 UART4 Route: What We Know

### From User Manual & Component Analysis

**TX Path (H747 M7 UART4_TXD):**
- i.MX UART4_TXD pin
- → Mux (U16/U17: 74LVC1G157 single-bit 2:1 multiplexer)
  - SEL pin controls route to either Murata L072 RX or other destination
- → Level Shifter (U8/U20: SN74LVC1T45 bi-directional)
  - OE (output enable) pin controls whether signal is driven
  - DIR (direction) pin may control TX vs. RX mode
- → Buffer (U10: SN74LVC1G125 tri-state)
  - OE (output enable) controls drive capability
- → Murata L072 RX pin

**RX Path (H747 M7 UART4_RXD):**
- Murata L072 TX pin
- → Level Shifter (U21/U22: SN74LVC1T45)
  - OE, DIR for return path
- → Mux (U18/U19: 74LVC1G157)
  - SEL controls routing
- → i.MX UART4_RXD pin

**Control Elements (Ownership Unknown — Phase D target):**
1. **Mux U16/U17 SEL pin** — selects Murata vs. other destination (likely GPIOX_n on H747 or i.MX)
2. **Mux U18/U19 SEL pin** — selects Murata vs. other source (likely GPIOX_n)
3. **Level Shifter U8/U20 OE pin** — enables TX path drive (likely GPIOX_n)
4. **Level Shifter U8/U20 DIR pin** — controls bi-directional mode (likely GPIOX_n or GND/VCCIO tie)
5. **Level Shifter U21/U22 OE pin** — enables RX path pull-up/return (likely GPIOX_n)
6. **Level Shifter U21/U22 DIR pin** — controls return path mode (likely GPIOX_n or fixed)
7. **Buffer U10 OE pin** — enables output drive to Murata (likely GPIOX_n)

**Three Ownership Scenarios:**
- **Scenario A:** All control pins driven by H747 GPIO (GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, etc.)
- **Scenario B:** Some pins driven by H747 GPIO, others by i.MX or fixed logic
- **Scenario C:** Majority driven by i.MX GPIO or firmware-controlled logic (less likely given user firmware in ROM bootloader doesn't work)

---

## H747 GPIO Bank Coverage: Tested vs. Untested

### Already Tested (Phase A–C)

✅ **GPIOA:** PA11 (discriminator), PA12 (test)  
✅ **GPIOB:** PB12 (test)  
✅ **GPIOC:** PC11, PC12 (tests)  
✅ **GPIOE:** PE11, PE12 (tests)  

### Untested H747 Banks (Phase D Candidates)

🔴 **GPIOB:** B0–B15 (excepting B12 already tested)
- Priority: HIGH (adjacent to tested PA band; likely continuation of differential/enable logic)
- Candidates: PB0–PB11, PB13–PB15
- Strategy: Onehot sweep (PB0 HIGH, PB1 HIGH, ... in sequence)

🔴 **GPIOD:** D0–D15
- Priority: MEDIUM (less common in mux/level-shifter contexts but possible)
- Candidates: PD0–PD15
- Strategy: Selective sweep of high-utilization pins (PD0, PD1, PD6, PD7, D14, D15)

🔴 **GPIOF:** F0–F15
- Priority: MEDIUM–HIGH (PF4 already used for NRST; suggests F-bank activity)
- Candidates: PF0–PF3, PF5–PF15
- Strategy: Focus on F0–F3 (adjacent to NRST PF4); full sweep if needed

🔴 **GPIOG:** G0–G15 (if H747 firmware brings this bank out)
- Priority: LOW (depends on board layout; least likely for UART4 control)
- Strategy: Defer unless Phase D B/D/F sweeps exhaust candidate pool

### i.MX GPIO (Alternative Hypothesis)

⚠️ **i.MX 8M Mini MX8MMP_GPIO (Port 1–5)**
- Priority: DEFERRED (would require modifying i.MX firmware; complex)
- Hypothesis: If H747 GPIO sweeps fail, suspect i.MX ownership
- Action: Consult i.MX device tree and GPIO control flow in i.MX Linux side

---

## Phase D Test Design

### Baseline (Carry Forward from Phase C)

Same halted-mode baseline as Phase C seeded profiles:
```
PA9  = ANALOG (unused)
PA10 = ANALOG (unused)
PA11 = OUTPUT LOW (user firmware boot mode)
PF4  = OUTPUT LOW initially, then HIGH after 250 ms delay (L072 release)
[Test pin HIGH or varied configuration]
```

### Phase D-1: GPIOB Onehot Sweep (Priority HIGH)

**Test Cases: d7–d17** (11 cases)

| Case | Pin   | State | Hypothesis |
|------|-------|-------|-----------|
| d7   | PB0   | HIGH  | UART4 TX mux or level-shifter control |
| d8   | PB1   | HIGH  | Continuation of differential logic |
| d9   | PB2   | HIGH  | Mux SEL or enable function |
| d10  | PB3   | HIGH  | Buffer OE or enable |
| d11  | PB4   | HIGH  | Level-shifter OE or DIR |
| d12  | PB5   | HIGH  | Secondary enable or polarity |
| d13  | PB6   | HIGH  | Cross-lane coupling or timing |
| d14  | PB7   | HIGH  | UART4 RX path control |
| d15  | PB8   | HIGH  | Mux routing or selection |
| d16  | PB9   | HIGH  | Level-shifter direction or mode |
| d17  | PB10  | HIGH  | Possible enable or tri-state control |
| d18  | PB11  | HIGH  | Secondary control or diagnostic |

**Execution:** Create d7–d18 cfg files, add to manifest as `ownerexp_phase_d1_gpiob_sweep_2026-05-11`, execute and extract signatures.

**Success Criteria:**
- Any case produces new signature class (byte count ≠ 0 and ≠ 144–145)
- BOOT_URC observed during probe
- Partial VER_URC handshake recovered

**Fallback:** If all GPIOB tests return ATI silence, advance to Phase D-2.

### Phase D-2: GPIOF Selective Sweep (Priority MEDIUM–HIGH)

**Test Cases: d19–d29** (11 cases, focused on F0–F10)

| Case | Pin   | State | Hypothesis |
|------|-------|-------|-----------|
| d19  | PF0   | HIGH  | Level-shifter OE (adjacent to NRST logic) |
| d20  | PF1   | HIGH  | Mux SEL or control |
| d21  | PF2   | HIGH  | Buffer OE |
| d22  | PF3   | HIGH  | DIR or polarity control |
| d23  | PF5   | HIGH  | Secondary enable |
| ... | ...   | ...   | [Additional candidates if needed] |

**Execution:** If Phase D-1 inconclusive, execute GPIOF sweep and assess results.

### Phase D-3: GPIOD Targeted Sweep (Priority MEDIUM)

**Test Cases: d30–d45** (up to 16 cases, selective pins)

**Execution:** If Phase D-1 and D-2 exhaust GPIOB/GPIOF without success, probe GPIOD high-probability pins (D0, D1, D6, D7, D14, D15).

---

## Hypothesis Refinement Strategy

### If Phase D-1 (GPIOB) Yields Success

**Expected Outcome:** New signature class or BOOT_URC observed for one or more GPIOB pins (e.g., PB5 HIGH).

**Next Action:**
1. Identify winning pin (e.g., PB5)
2. Test polarity (PB5 LOW, PB5 toggle, PB5 pulse) in d19–d21 Phase D-1b variant
3. Combine winning pin with other suspected enables (e.g., PB5 HIGH + PB0 HIGH) for multi-pin hypothesis refinement
4. Document control-net ownership matrix once pin function identified

### If Phase D-1 (GPIOB) Yields Silence

**Expected Outcome:** All GPIOB cases return ATI silence (same as seeded baseline).

**Next Action:**
1. Advance to Phase D-2 (GPIOF sweep)
2. If GPIOF also yields silence, prepare Phase D-3 (GPIOD)
3. If all three banks exhaust without success, suspect:
   - i.MX ownership (requires firmware-level investigation)
   - Fixed logic (resistor pull-ups, capacitive coupling, no GPIO needed)
   - Timing-dependent enable (not GPIO-state-dependent per se)

---

## Manifest Structure for Phase D

### Manifest Entry Template

```json
{
  "name": "ownerexp_phase_d1_gpiob_sweep_2026-05-11",
  "description": "GPIOB onehot sweep targeting untested H747 control-net pins. Expected outcome: identify at least one pin that enables UART4 TX/RX paths or produces new signature class.",
  "probeDevice": "/dev/ttymxc3",
  "baud": 921600,
  "bootTimeout": 2,
  "cases": [
    {
      "cfg": "d7_uart4_pb0_high_halted",
      "tag": "d7"
    },
    {
      "cfg": "d8_uart4_pb1_high_halted",
      "tag": "d8"
    },
    ... [continue for d9–d18]
  ]
}
```

### Configuration File Pattern

Each Phase D-1 cfg will follow:
```tcl
# d7_uart4_pb0_high_halted.cfg
init
halt
[RCC setup, moder/bsrr procs as before]
moder_set_analog $GPIOA_BASE 9
moder_set_analog $GPIOA_BASE 10
moder_set_output $GPIOA_BASE 11
bsrr_reset $GPIOA_BASE 11
moder_set_output $GPIOB_BASE 0  # <-- NEW: test pin
bsrr_set $GPIOB_BASE 0           # <-- NEW: toggle state
moder_set_output $GPIOF_BASE 4
bsrr_reset $GPIOF_BASE 4
sleep 250
bsrr_set $GPIOF_BASE 4
```

---

## Decision Tree: Phase D → Phase E

```
┌─ Phase D-1 (GPIOB sweep)
│  ├─ Success (new signature or BOOT_URC) ──→ Document pin, test polarity/combo, proceed to Phase E-1 (fine-tuning)
│  └─ Silence (all cases ATI silence) ──────→ Advance to Phase D-2 (GPIOF)
│
├─ Phase D-2 (GPIOF sweep)
│  ├─ Success ──→ Document, refine, proceed Phase E-1
│  └─ Silence ───→ Advance to Phase D-3 (GPIOD)
│
├─ Phase D-3 (GPIOD sweep)
│  ├─ Success ──→ Document, refine, Phase E-1
│  └─ Silence ───→ Phase E-2 (i.MX/firmware hypothesis or timing/sequence analysis)
│
└─ Phase E (Exploitation & Validation)
   ├─ E-1: Fine-tune identified control pins
   ├─ E-2: Multi-pin combinations (if single pin insufficient)
   └─ E-3: Full protocol handshake validation
```

---

## Schematic Access Contingency

**Note (2026-05-11):** Direct PDF access to ABX00043-schematics.pdf is currently blocked (network/browser restriction). Phase D proceeds via strategic GPIO exploration instead of schematic-guided hypothesis. Once schematics become accessible, a secondary review will validate/correct Phase D pin selections against actual net names.

**Future Enhancement:** Cross-reference Phase D empirical results against schematic once available.

---

## Timeline & Resource Notes

**Phase D-1 Execution (GPIOB):**
- cfg creation: ~30 min (12 files, pattern-based)
- manifest update: ~10 min
- profile execution: ~2 min per case × 12 = 24 min
- Total: ~1 hour wall-clock time

**Decision point:** Upon Phase D-1 completion, assess results:
- **Branch A (Success):** Immediate Phase E planning (high confidence)
- **Branch B (Silence):** Commit to Phase D-2/D-3 exploration (systematic fallback)

---

## References

- **Phase C Results:** `2026-05-10_OwnerNet_Derived_UART4_Profile_Results_Copilot_v1_0.md`
- **Topology Analysis:** `2026-05-10_ABX00043_OwnerNet_UART4_Extraction_Copilot_v1_1.md`
- **Framework:** `run_owner_net_profile.ps1` (manifest runner)
- **Blocker Status:** `DESIGN-CONTROLLER/TODO.md` (Phase D notes)

---

**Status:** Ready to execute Phase D-1 upon user approval. Manifest and cfg templates prepared; awaiting go-ahead to begin GPIOB sweep.
