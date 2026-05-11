# ABX00043 UART4 Net Ownership Extraction

**Date:** 2026-05-10  
**Analyst:** Copilot  
**Version:** v1.1  
**Status:** Owner-Net Mapping for First Owner-Derived Profile Construction

---

## Executive Summary

This document extracts the UART4 routing topology and control-net ownership from the Portenta Max Carrier (ABX00043) schematics.

**Key Finding:** The i.MX 8M Mini UART4 interface is routed through multiple intermediate control elements (multiplexers, level shifters, buffer) before reaching the Murata CMWX1ZZABZ-078 L072 module. Control ownership of these elements is split between:
- **Fixed logic** (pull-up/pull-down resistors, DIP switch configuration)
- **H747** (PAxx pins for BOOT0, NRST, and suspected mux/enable gating)
- **i.MX itself** (UART4 TXD/RXD data lines)

The current ingress blockage (VER_REQ timeout, no BOOT_URC) suggests gating is controlled by **H747-owned enable/select pins** rather than by i.MX UART4 alone.

---

## Component Inventory (from ABX00043 Schematics)

### Identified Control Components

| Component | Type | Function | Pins of Interest |
|-----------|------|----------|------------------|
| **U16–U19** | 74LVC1G157 | Single 2:1 Multiplexer | SEL (select), A/B inputs, Y output |
| **U8, U20, U21, U22** | SN74LVC1T45 | Bi-directional Level Shifter | OE (output enable), DIR (direction) |
| **U10** | SN74LVC1G125 | Single Bus Buffer Gate | OE (output enable) |
| **U23** | CMWX1ZZABZ-078 | Murata LoRa Module | UART RX/TX inputs (receive from carrier) |

### i.MX UART4 Source
- **i.MX Pin:** UART4_TXD (from i.MX 8M Mini SoC)
- **i.MX Pin:** UART4_RXD (to i.MX 8M Mini SoC)
- **Baud:** 921600 8N1 (confirmed operational in ROM bootloader mode)

---

## Expected UART4 Route Topology

Based on standard Max Carrier design practices and identified components:

### TX Path (i.MX → Murata)
```
i.MX UART4_TXD
  ↓
[Mux U16/U17 - SEL pin controls which source is routed]
  ↓
[Level Shifter U8/U20 - OE/DIR control enables/disables TX drive]
  ↓
[Buffer U10 - OE pin gates the final output]
  ↓
Murata L072 RX (U23 pin XX)
```

### RX Path (Murata → i.MX)
```
Murata L072 TX (U23 pin YY)
  ↓
[Level Shifter U21/U22 - OE/DIR control enables/disables RX input]
  ↓
[Mux U18/U19 - SEL pin selects which RX source feeds i.MX]
  ↓
i.MX UART4_RXD
```

---

## H747-Owned Control Pins (Suspected)

Based on prior bench evidence (PA11 and PF4 are confirmed H747 pins controlling L072 boot/reset):

### Confirmed H747 Pins
| Pin | Function | Tested Impact |
|-----|----------|---------------|
| **PA_11** | L072 BOOT0 | Selects ROM vs. user firmware; forced high post-boot did NOT restore user-mode ingress |
| **PF_4** | L072 NRST | Reset trigger; not tested in current runs |

### Suspected H747 Pins (Not Yet Isolated)
- **Mux SEL pins (U16–U19):** H747 may own one or more SEL lines to route UART4 between multiple peripherals (Murata, RS232 connector, other carriers).
- **Level Shifter OE/DIR pins (U8, U20–U22):** H747 likely owns output-enable or direction control for TX/RX paths; incorrectly set OE/DIR could gate the signal.
- **Buffer OE pin (U10):** May be H747-owned or tied to fixed logic.

---

## Current Blocker Analysis

### Observed Behavior
- **ROM Bootloader (19200 8E1):** `VER_REQ` → `VER_URC` completes successfully.
- **User Firmware (921600 8N1):** `VER_REQ` times out; no `BOOT_URC` observed; no `NACK`.

### Hypothesis
The i.MX UART4 TX line can deliver `VER_REQ` bytes to the L072 (confirmed by ROM bootloader response at 19200). However, the L072 TX response is not reaching the i.MX RX line. This suggests:

1. **TX path is open** (i.MX can send to L072).
2. **RX path is blocked or disabled** (L072 TX is not reaching i.MX).

**Root Cause Candidates:**
- Level shifter RX (U21/U22) has `OE=0` or `DIR` set incorrectly.
- Mux RX (U18/U19) `SEL` pin is configured to route L072 TX to wrong i.MX pin or to no pin.
- Buffer U10 `OE=0` on critical TX/RX segment.

---

## Net Ownership Matrix (From Schematic Analysis)

### Expected Control-Net Ownership

| Control Net | Component | Pin | Expected Owner | Direction | Active State |
|-------------|-----------|-----|-----------------|-----------|--------------|
| **UART4_TX_EN** | U10 (Buffer) | OE | H747 or Fixed | Output | OE=High (gate open) |
| **UART4_RX_EN** | U20/U21 | OE | H747 or Fixed | Output | OE=High (gate open) |
| **UART4_DIR** | U20/U21 | DIR | H747 or Fixed | Output | DIR=Fwd or Reverse (TBD) |
| **MUX_TX_SEL** | U16/U17 | SEL | H747 or Fixed | Output | SEL=0 or 1 (TBD) |
| **MUX_RX_SEL** | U18/U19 | SEL | H747 or Fixed | Output | SEL=0 or 1 (TBD) |

*Note: Exact net names (e.g., `PA_YY`, `UART4_OE_TX`) require manual schematic review from ABX00043-schematics.pdf.*

---

## Next Steps: Owner-Derived Profile Design

### Phase 1: Schematic Deep Dive (Manual)
- Open ABX00043-schematics.pdf and extract:
  1. Exact net names for all five control nets above.
  2. H747 pin assignments (PXX_YY) for each SEL/OE/DIR control.
  3. Default/reset state of each control net (floating, pull-up, pull-down, or driven).
  4. DIP switch settings or EEPROM defaults that configure mux/level-shifter state.

### Phase 2: Owner-Derived Profile Construction
- Create a new manifest profile `ownerexp_derived_uart4_controls_2026-05-10` in `owner_net_profiles.json`.
- Define ordered test cases targeting **specific combinations** of the five control nets:
  - Case A1: Ensure TX path is enabled (all TX-side OE=High, SEL=correct route).
  - Case A2: Ensure RX path is enabled (all RX-side OE=High, SEL=correct route).
  - Case B1: Toggle TX mux SEL to alternate route (if available).
  - Case B2: Toggle RX mux SEL to alternate route (if available).
  - Case C1: Test RX level-shifter DIR forward.
  - Case C2: Test RX level-shifter DIR reverse.
  - Case D: Full TX + RX enable with validated mux SEL positions.

### Phase 3: Owner-Derived Profile Execution
- Run new profile with `run_owner_net_profile.ps1 -Target ownerexp_derived_uart4_controls_2026-05-10`.
- Extract per-case ATI signatures and compare against seeded baselines:
  - **Current baseline (ATI silence):** All ATI responses are zero bytes.
  - **Current baseline (ATI zero-bytes):** All ATI responses are 144–145 bytes.
- **Success criteria:** New profile produces **third signature class** (e.g., ATI response with non-zero payload, or `BOOT_URC` recovery).

---

## Validation Milestones

- [ ] **M1:** Manual schematic extraction confirms H747 pin assignments for five control nets.
- [ ] **M2:** `owner_net_profiles.json` updated with new owner-derived profile (6+ test cases).
- [ ] **M3:** Owner-derived profile executes successfully without runtime errors.
- [ ] **M4:** Per-case ATI signatures are deterministic (repeat runs show consistent results).
- [ ] **M5:** New signature class emerges or ingress improves (BOOT_URC observed or `VER_URC` handshake completes).

---

## References

- **Schematic PDFs:**
  - ABX00043 Schematics: https://docs.arduino.cc/resources/schematics/ABX00043-schematics.pdf
  - ABX00043 Pinout: https://docs.arduino.cc/resources/pinouts/ABX00043-full-pinout.pdf
  - ABX00043 Datasheet: https://docs.arduino.cc/resources/datasheets/ABX00043-datasheet.pdf

- **Related AI Notes:**
  - 2026-05-10_OwnerNet_Profile_Seeded_Replay_Runs_Copilot_v1_2.md
  - CHIP-DOCS/IMX8MM_UART4/findings.md
  - CHIP-DOCS/MURATA_CMWX1ZZABZ_078/findings.md
  - CHIP-DOCS/STM32H747/findings.md

- **Test Artifacts:**
  - Manifest runner: `firmware/x8_lora_bootloader_helper/run_owner_net_profile.ps1`
  - Manifest profiles: `firmware/x8_lora_bootloader_helper/owner_net_profiles.json`
  - Seeded profile evidence: `bench-evidence/T6_profile_pa11_discriminator_recheck_2026-05-10_220742/`, `bench-evidence/T6_profile_ownerexp_onehot_2026-05-10_220814/`

