# 2026-05-10 Chip Research + Lane Isolation (Copilot v1.0)

## Scope

This note was produced to satisfy: "do more research about these chips, then proceed with your suggestion."

The immediate objective was to combine:
1. Chip-level constraints (i.MX8MM UART4, Murata CMWX1ZZABZ-078 / STM32L072, STM32H747 ownership), and
2. A board-level discriminating experiment that targets likely mux/OE/select control lanes.

## Sources used

### Official board sources already tracked
- ABX00043 schematics PDF:
  - https://docs.arduino.cc/resources/schematics/ABX00043-schematics.pdf
- ABX00043 datasheet PDF:
  - https://docs.arduino.cc/resources/datasheets/ABX00043-datasheet.pdf
- ABX00043 full pinout PDF:
  - https://docs.arduino.cc/resources/pinouts/ABX00043-full-pinout.pdf

### In-repo chip docs and bench logs
- `AI NOTES/CHIP-DOCS/*` findings + open questions for:
  - `IMX8MM_UART4`
  - `MURATA_CMWX1ZZABZ_078`
  - `STM32H747`
- New experiment artifacts:
  - `DESIGN-CONTROLLER/bench-evidence/T6_lane_isolation_2026-05-10_184524/`

## Chip-level synthesis (what matters to the blocker)

## 1) i.MX8MM UART4 host endpoint
- Working host endpoint in bench flows remains `/dev/ttymxc3`.
- ROM-mode AN3155 communication is known-good, so the physical path can carry host->target traffic in at least one boot state.
- Prior control-line test showed Linux cannot successfully hold `crtscts` enabled on `/dev/ttymxc3` in this image; software RTS/CTS toggles did not recover ingress.

Implication:
- The current failure mode is unlikely to be explained by user-space termios settings alone.
- A board-level state gate (route/enable/select) is still the strongest hypothesis.

## 2) Murata CMWX1ZZABZ-078 / STM32L072
- L072 user firmware emits outbound diagnostics/URCs to host, confirming user-mode TX path from module to X8 is alive.
- Host ingress remains absent at ISR/accounting level in user mode in prior runs.
- BOOT0 (`PA_11`) and NRST (`PF_4`) ownership/path has been proven sufficient to enter ROM bootloader and flash.
- Forcing `PA_11` high post-boot did not restore user-mode ingress.

Implication:
- BOOT0 strap alone is not the hidden gate for user-mode host RX.
- There is likely another control path between host UART and Murata RX in user mode.

## 3) STM32H747 control ownership
- Existing notes already classify H747 as the likely owner for route-control or modem-side glue logic on this carrier stack.
- OpenOCD halted-mode GPIO forcing is effective and readback-verifiable (IDR confirms driven state), making it a valid lever for discriminating mux/OE/select hypotheses.

Implication:
- The right next move is coherent lane forcing on candidate H747 GPIO clusters while probing `/dev/ttymxc3`.

## Board-level discriminating experiment run

## Experiment name
- `T6_lane_isolation_2026-05-10_184524`

## Method
For each lane-set cfg, run:
1. OpenOCD script to boot user app path, then force a coherent GPIO lane low/high.
2. Stage1 probe on `/dev/ttymxc3` at `921600`.
3. UART diag probe (`ATI`, `VER_REQ`, `PING_REQ`).

Lane sets tested:
- A low/high: `PA2 PA3 PA15 PB14 PB15`
- B low/high: `PA9 PA10 PB10 PB11 PC9`
- C low/high: `PC10 PC11 PC12 PC9 PE11 PE12`

## Primary observations

### A) OpenOCD forcing worked electrically
- IDR readback in openocd logs shows expected 0 values in low cfgs and 1 values in high cfgs across all forced pins.
- This confirms the experiment actually perturbed carrier control states.

### B) Probe behavior changed from prior "no bytes" signature
Across all six lane states:
- `reply to ATI` now returns non-empty payloads (about 141-146 bytes, mostly/all `0x00`), not plain silence.
- `AT+VER?` still returns no bytes.
- Binary `VER_REQ` still times out waiting for `VER_URC`.

Interpretation:
- Lane forcing clearly perturbs the observed serial behavior, so these control clusters are coupled to the path.
- But none of the tested low/high full-cluster states restored valid framed ingress (`VER_URC`) in user mode.

### C) No clear winning lane yet
- A/B/C each altered the `ATI` byte-count pattern slightly, but no state produced protocol recovery.
- This suggests either:
  1. The true gate is a different net not included in these three clusters, or
  2. The gate requires a specific mixed state (not full-cluster all-high/all-low), or
  3. Timing/order dependency exists (state must be asserted before/through reset differently).

## Updated blocker classification

Most likely current class:
- Carrier-level route/enable/select condition on the host->L072 direction that is boot-state or controller-state dependent.

Less likely now:
- Pure baud mismatch (already tested).
- Pure host termios flow-control toggle issue (`crtscts` not enabled and no effect).
- BOOT0-only gating.

## Next high-value experiments

1. Mixed-state matrix inside the most sensitive lane cluster
- Do not only test all-high/all-low.
- Test a small binary matrix with one-bit flips while holding others fixed.
- Start with lane B and C first (closest to canonical USART/LPUART candidate groupings in current hypotheses).

2. Reset-order sensitivity test
- Assert candidate gate state before NRST release and maintain through early user boot window.
- Compare against post-boot-only forcing.

3. ROM-vs-user under identical forced lane state
- Under one fixed forced state, run strict A/B:
  - ROM 19200 8E1 ACK check
  - user 921600 probe/diag
- Goal: prove whether forced state narrows the ROM-user asymmetry.

4. Schematic extraction completion (required to reduce search space)
- Extract exact net names from ABX00043 schematic for:
  - i.MX8 UART4_TXD/RXD
  - intermediary buffers/muxes/switches
  - OE/EN/SEL nets and owning controller pin
- Then collapse matrix to only evidence-backed control pins.

## Practical decision for immediate continuation

Proceed with a focused mixed-state lane test (not blanket high/low) plus reset-order control, while continuing schematic net extraction in parallel.

This gives the best chance of converting the current "path is perturbable" result into a concrete "this net gates ingress" finding.
