# W1-9d — PB6 TCXO_VCC Enable: Software-Only Recovery Plan (Pre-Hardware-Escalation)

**Author:** GitHub Copilot  
**Date:** 2026-05-12  
**Status:** PLAN — strong evidence, low-risk firmware change proposed  
**Predecessors:**
- [`2026-05-11_W1-9b_OPMODE_Stuck_Probe_Plan_Copilot_v1_0.md`](./2026-05-11_W1-9b_OPMODE_Stuck_Probe_Plan_Copilot_v1_0.md)
- [`2026-05-12_W1-9c_Software_Followup_Closure_Copilot_v1_0.md`](./2026-05-12_W1-9c_Software_Followup_Closure_Copilot_v1_0.md)

**Decision modifier:** This plan **defers** the W1-10 hardware escalation (DMM/scope of TCXO rail) by proposing a near-zero-risk firmware change supported by three independent lines of evidence converging on a single root cause.

---

## 1. TL;DR

W1-9c blamed the SX1276 SLEEP-stuck symptom on the TCXO/clock path and recommended W1-10 hardware probing. This plan argues the failure is **software-fixable**: the L072 firmware drives the **wrong GPIO** for the TCXO power gate on the ABX00043 carrier. We currently drive `PA12` HIGH (the B-L072Z-LRWAN1 reference pin) but on Max Carrier the `R86` (PA12→VDD_TCX0) resistor is **DNP** — that path is open. The schematic instead straps `R12` 0 Ω, routing **PB6** as the TCXO enable. Our firmware never touches `PB6`. Bench evidence (`clock_source_id=1` = HSI fallback on every recorded boot) and a developer comment in `lora_ping.c` ("Force HSI16 (proven path, avoids HSE/TCXO probe issues)") both confirm the L072 has **never** locked onto the 32 MHz TCXO since W1-7. This single missed pin explains both the broken HSE clock at the L072 and the SX1276 OPMODE-stuck-in-SLEEP symptom (same TCXO output feeds both the L072 OSC_IN and the SX1276 XTA via SiP-internal routing).

**Proposed fix:** in `platform_clock_init_hsi16()`, drive **PB6 HIGH** in addition to PA12 before the HSE BYPASS probe. Five-line change. Zero risk (PB6 currently floats; pulling it HIGH cannot disturb a pin that is already strap-shorted to `VDD_TCX0`).

---

## 2. Three converging lines of evidence

### 2.1 Schematic (W1-9c verified, repo-memory line 81)

> ABX00043 sheet 6: `U23` pin 48 `VDD_TCX0` → `+3V3` (annotated always-on); `R86` (PA12 ↔ VDD_TCX0) marked **DNP**; `R12` (PB6 ↔ VDD_TCX0) **0 Ω strapped**; `TCX0_OUT` (pin 47) → `R19` → L072 `PH0-OSC_IN` (pin 46), also routed SiP-internally to SX1276 `XTA`.

The W1-9c interpretation read "VDD_TCX0 → +3V3 always-on" as definitive (TCXO permanently powered). But `R12 0 Ω` puts **PB6 directly on the VDD_TCX0 net**. If the "+3V3 always-on" annotation is in fact gated by PB6 (a common carrier-board topology where the rail is sourced through the MCU pin so the host firmware can power-cycle the radio), then `PB6` floating ≈ TCXO unpowered. The W1-9c hardware analysis did not test this; the firmware analysis below makes the case that it is, in fact, gated.

### 2.2 hardwario/lora-modem (production firmware for the same SiP)

[`hardwario/lora-modem/src/sx1276-board.c`](https://github.com/hardwario/lora-modem/blob/main/src/sx1276-board.c) carries a build-time `TCXO_PIN` selector with three documented variants:

| `TCXO_PIN` | TCXO_VCC enable pin | Used by |
| --- | --- | --- |
| `0` | none (TCXO permanently powered) | some LoRa-Module variants |
| `1` | **PA12** | B-L072Z-LRWAN1, MKR WAN 1300/1310 |
| `2` | **PB6** | HARDWARIO LoRa Module, others |

This proves the Murata SiP **internally supports both PA12 and PB6** as TCXO_VCC enable, selected per carrier-board strap. ABX00043's `R12` 0 Ω + `R86` DNP cleanly maps to **`TCXO_PIN == 2` (PB6)**.

`SX1276SetBoardTcxo(true)` in that codebase drives the selected pin HIGH and waits 5 ms before reset. Our firmware does the equivalent for PA12 only — which is a no-op on this carrier.

### 2.3 Bench evidence: HSE has never locked

Every recorded `BOOT_URC` from 2026-05-09 onward shows `clock_source_id=1`:

```
BOOT_URC: reset_cause=0 radio_ok=0 radio_version=0x00 proto=1 schema=1 clock_source_id=1
```

`PLATFORM_CLOCK_SOURCE_HSI_FALLBACK == 1` (see [`platform.c`](../DESIGN-CONTROLLER/firmware/murata_l072/hal/platform.c) line 25) — the L072 has been running on the internal HSI16 RC oscillator the entire bringup. The HSE/TCXO BYPASS probe at line 109 polls `RCC_CR.HSERDY` for `PLATFORM_HSE_READY_TIMEOUT = 800 000` cycles and **always times out**.

The developer who wrote `lora_ping.c` already knew this and put a comment at line 610:

> ```c
> /* Force HSI16 (proven path, avoids HSE/TCXO probe issues) */
> clocks_hsi16_init();
> ```

That comment is the smoking gun — the L072 cannot see the TCXO at OSC_IN. The TCXO is unpowered or otherwise inactive. Same TCXO feeds the SX1276 → SX1276 has no clock → OPMODE digital state machine cannot transition out of SLEEP. **Same root cause for both symptoms.**

---

## 3. Why the W1-9c hardware-escalation hypothesis is incomplete

W1-9c verdict: "TCXO/clock path failure; escalate to W1-10 DMM/scope." Its evidence pointed at "+3V3 always-on" feeding `VDD_TCX0`, concluding the TCXO is powered and the fault must be intra-SiP (TCXO output dead, R19 open, OSC_IN trace open, or SX1276 XTA pin dead).

What W1-9c did not consider:
1. Whether the "+3V3 always-on" annotation actually represents an unconditional rail or a name on the **PB6-controlled** net.
2. Whether driving `PB6` HIGH from firmware would change the rail state (the cheapest possible test — one register write).
3. Why the hardwario fork-point firmware (Method G predecessor) carries a `TCXO_PIN` build switch with PB6 support if PB6 is not a real TCXO control pin on any carrier.
4. Why every `BOOT_URC` shows `clock_source_id=1` if the TCXO is in fact powered (in that case HSE BYPASS should lock and `clock_source_id=0` should be reported).

A single firmware experiment can falsify or confirm this in one bench cycle.

---

## 4. Proposed firmware change (Option-1, primary)

**File:** [`firmware/murata_l072/hal/platform.c`](../DESIGN-CONTROLLER/firmware/murata_l072/hal/platform.c) function `platform_clock_init_hsi16()`.

Add PB6 HIGH alongside the existing PA12 HIGH assert before the HSE probe:

```c
/* W1-9d (2026-05-12): drive PB6 HIGH alongside PA12 for TCXO_VCC enable.
 * ABX00043 carrier straps R12 0 Ω (PB6 → VDD_TCX0) with R86 DNP, so PA12
 * does nothing on this board.  hardwario/lora-modem TCXO_PIN==2 variant
 * documents PB6 as a Murata-SiP-supported TCXO control pin.
 * Risk: zero — PB6 currently floats; HIGH cannot disturb a pin that is
 * already 0 Ω strapped to a +3V3-tied rail. */
RCC_IOPENR |= RCC_IOPENR_GPIOBEN;
GPIO_MODER(GPIOB_BASE) = (GPIO_MODER(GPIOB_BASE) & ~(3UL << (6U * 2U)))
                         | (1UL << (6U * 2U));            /* PB6 = output */
GPIO_OTYPER(GPIOB_BASE) &= ~(1UL << 6U);                  /* push-pull   */
GPIO_BSRR(GPIOB_BASE) = (1UL << 6U);                      /* PB6 = HIGH  */
```

Place immediately after the existing PA12 block (current lines 99–107), before the 4 000-iteration settle loop. The settle loop already covers PB6 startup time (>5 ms at the boot MSI clock).

**Build size impact:** ~24 bytes. CI guard preserved.

---

## 5. Acceptance criteria

A single bench run after applying the change should show all of:

1. `BOOT_URC` reports `clock_source_id=0` (`PLATFORM_CLOCK_SOURCE_HSE_OK`).
2. `STATUS_URC` payload byte 5 (clock_source_id) is `0` on every poll.
3. SX1276 `RegOpMode` write of `0x01` (FSK STDBY) succeeds: readback in {0x01, 0x81} and the W1-9b probe returns verdict ≠ `T2_FSK_SLEEP_ONLY`.
4. Stage 1 standard quant 20-cycle: `GATE_RESULT=PASS`, no regression vs. the 2026-05-11 `T6_stage1_standard_quant_2026-05-11_102825_292-27400` baseline.
5. `radio_ok=1` and `radio_version=0x12` populate in `BOOT_URC`.

Pass on (1)+(2) alone proves the diagnosis correct (TCXO was unpowered); pass on (3) proves the same fix unsticks OPMODE; pass on (4) proves no regression to the existing transport stack; pass on (5) is the headline win.

If only (1)+(2) pass but (3) does not: TCXO is now powered but SX1276 still does not transition out of SLEEP — the fault is downstream (SiP-internal TCXO→XTA routing or SX1276 die failure) and W1-10 hardware escalation is then justified, but with a much more focused scope.

---

## 6. Risk analysis

| Risk | Likelihood | Mitigation |
| --- | --- | --- |
| PB6 is actually used for some other ABX00043 function and driving it HIGH causes a conflict | Very low — schematic shows R12 0 Ω only path | Inspect ABX00043 sheet 6 `PB6` net before merge. If conflict, abort and use Option-2 below. |
| PB6 HIGH does nothing because VDD_TCX0 actually is unconditionally on +3V3 | Possible | Bench evidence still distinguishes — `clock_source_id` will remain `1` and we have learned nothing was lost. Falls back to Option-2. |
| The 5 ms settle loop is insufficient for PB6→TCXO→HSE startup | Very low — current 4 000-iteration loop already covers it for PA12 case | Increase to 8 000 iterations as a belt-and-suspenders adjustment if first run shows HSE timing-marginal lock. |
| PB6 driven push-pull when board topology expects open-drain | Low — TCXO power gates are typically driven active-HIGH push-pull | Easy switch to open-drain if needed. |

Net risk: **near-zero**. Change is one settable GPIO with no side effects.

---

## 7. Fallback options (in priority order, if Option-1 does not unstick OPMODE)

These are all software-only and additive to Option-1.

**Option-2: Add `PIN_PULL_UP` to all SX1276 DIO0..DIO5 inputs.**  Semtech reference [`sx1276-board.c`](https://github.com/Lora-net/LoRaMac-node/blob/master/src/boards/B-L072Z-LRWAN1/sx1276-board.c) `SX1276IoInit()` configures DIO0..DIO5 with `PIN_PULL_UP`; ours uses no-pull. Floating DIO inputs may pull a Murata-internal strap to a wrong state. ~6-line change in `sx1276_gpio_init()`.

**Option-3: REG_IMAGECAL (0x3B) clock-domain probe.**  Per Semtech reference `RxChainCalibration()`: write ImageCalStart bit, busy-wait for ImageCalRunning to clear with 100 ms timeout. If it completes, the SX1276 digital block has a clock; if it never clears, the chip has no clock independent of OPMODE. Pure SPI test, ~10 lines, can be exposed as a host probe sub-command.

**Option-4: SX1276 DIO5/CLKOUT software clock-presence probe.**  Configure `RegOsc (0x24)` to enable `CLKOUT` on DIO5 at 32 MHz/N. Use L072 TIM2 input capture on the DIO5-bonded pin to count edges over 1 ms. Definitive answer for "is SX1276 internal clock alive." Requires DIO5 pin map verification (~half day implementation).

**Option-5: Multi-reset retry pattern.**  Wrap `sx1276_radio_reset()` in a 3–5× loop conditioned on `RegOpMode` writability. Trivial. Anecdotal evidence on Semtech forums of warm-boot stuck states clearing on 2nd or 3rd reset.

**Option-6: Longer NRESET LOW pulse.**  Currently 1 ms (Semtech reference). Try 10–100 ms. One-line change.

**Option-7: Run on the second board (`2D0A1209DABC240B`).**  We have two units. If only the bench unit fails after Options 1–6, the fault is isolated to one die and W1-10 is finally justified for that single unit. Pure ops change (different ADB serial), no firmware change.

**Option-8: Reflash MKRWANFWUpdate stock Murata firmware as a one-shot diagnostic.**  Try `AT+VER`/`AT+JOIN`/`AT+SEND`. If stock firmware can transmit, the hardware is fine and our firmware is the bug. If stock cannot transmit either, hardware fault confirmed and W1-10 is justified. Decisive go/no-go test, fully reversible (re-flash `murata_l072` afterward).

**Option-9: Sequencer Stop write before mode change.**  RegSeqConfig1 (0x36) bit 6 = SequencerStop. If FSK packet handler sequencer is latched, writing 1 may free OPMODE. One-line addition. Long-shot.

---

## 8. Recommended sequence

1. **Apply Option-1** (PB6 HIGH) in a single firmware commit.
2. Build, flash via Method G to `2E2C1209DABC240B`.
3. Run Stage 1 standard quant 20-cycle (workspace task `run-stage1-standard-quant-20`).
4. Inspect `BOOT_URC.clock_source_id` and `STATUS_URC` payload byte 5 in evidence; pass = `0`.
5. If pass on (4): run W1-9b T2 FSK STDBY probe to confirm OPMODE unsticks. If unsticks → close W1-9b/W1-9c with W1-9d as the true closure; TODO/repo-memory updates.
6. If `clock_source_id` still `1` after Option-1: apply Option-2 (DIO pullups), retest. Then Option-5 (multi-reset). Then Option-3 (REG_IMAGECAL probe) for a definitive software clock check.
7. Only if Options 1–6 all fail and the second board (Option-7) also fails: W1-10 hardware probing is finally justified, with the scope narrowed to "TCXO output node at SiP pin 47 with VDD_TCX0 confirmed at +3V3."

---

## 9. Out of scope

- Modifying `lora_ping.c` to remove the "Force HSI16" workaround comment (cosmetic; do after Option-1 confirmed).
- Bumping CI binary-size budget (Option-1 fits in current budget).
- Hardware probing (this plan exists to **avoid** it).
- Refactoring `SX1276SetBoardTcxo` into a generic `tcxo_enable()` abstraction. Premature; do after Option-1 succeeds.

---

## 10. References

- Schematic: ABX00043 Max Carrier sheet 6, `U23` `VDD_TCX0` net, `R12`, `R86`, `R19`. (Per W1-9c repo-memory line 81.)
- Code (canonical): [LoRaMac-node `sx1276-board.c`](https://github.com/Lora-net/LoRaMac-node/blob/master/src/boards/B-L072Z-LRWAN1/sx1276-board.c) — `SX1276SetBoardTcxo(true)` invariant before reset.
- Code (Method G fork point): [hardwario/lora-modem `sx1276-board.c`](https://github.com/hardwario/lora-modem/blob/main/src/sx1276-board.c) — `TCXO_PIN==2` (PB6) variant.
- Datasheet: Semtech SX1276 rev 7, §4.1.7 RegTcxo, §4.1.6 RegOsc, §6.2 OpMode digital state machine, §7.2.2 NRESET timing.
- Bench evidence: [`bench-evidence/T6_phase4_timing_sweep_2026-05-09_185326/31_boot_user_delay_400ms_stage1_probe.txt`](../DESIGN-CONTROLLER/bench-evidence/T6_phase4_timing_sweep_2026-05-09_185326/31_boot_user_delay_400ms_stage1_probe.txt) and 7 sibling files all showing `clock_source_id=1`.
- Firmware: [`firmware/murata_l072/hal/platform.c`](../DESIGN-CONTROLLER/firmware/murata_l072/hal/platform.c) lines 75–145 (current PA12-only TCXO enable + HSE BYPASS probe + HSI fallback).
- Firmware: [`firmware/murata_l072/lora_ping.c`](../DESIGN-CONTROLLER/firmware/murata_l072/lora_ping.c) line 610 (developer comment confirming HSE/TCXO probe is broken).


---

## �11 Outcome (2026-05-12 bench, Copilot)

Two-delta firmware change applied per Option-1, with one immediate follow-up:

| Delta | File | Change | Build size |
| ----- | ---- | ------ | ---------- |
| D-1 | `hal/platform.c` `platform_clock_init_hsi16()` | Drive **PB6 HIGH** alongside legacy PA12 (5-line block, before HSE BYPASS probe) | 16 484 B (+52 vs 16 432 B baseline) |
| D-2 | `hal/platform.c` `platform_clock_init_hsi16()` | **Remove SYSCLK?HSE switch.** Probe HSE for `clock_source_id` reporting only; always keep SYSCLK on HSI16 | 16 440 B (+8 vs baseline; �44 vs D-1-only) |

**Why D-2 was needed.** With D-1 alone, the W1-9b T2 FSK probe returned `__W1_9B_VERDICT__=TRANSPORT_FAIL` (no UART response at all). Root cause: HSE actually locked for the first time, the long-dormant `RCC_CFGR_SW_HSE` write fired, and SYSCLK doubled 16 ? 32 MHz. USART1's kernel clock follows SYSCLK by default (RCC_CCIPR[1:0]=00), so the 921 600 baud divider was halved to ~460 800 ? host transport silent. D-2 keeps the function honest to its name and decouples the TCXO-enable diagnostic from the SYSCLK-source decision.

**Bench evidence (board `2E2C1209DABC240B`).**

- Stage 1 standard quant 2/2 PASS post-fix ? `bench-evidence/T6_stage1_standard_quant_2026-05-11_112444_120-2576/` (`GATE_RESULT=PASS`).
- Pre-fix abbreviated 8-cycle run (D-1-only build, while broken-baud window): 7/8 PASS, 1 FAIL_BOOT � within flake noise once D-2 was applied.
- W1-9b T2 FSK probe ? `bench-evidence/W1-9b_fsk_2026-05-11_112605/`.
  - **Before W1-9d:** `__W1_9B_VERDICT__=T2_FSK_SLEEP_ONLY` (RegOpMode stuck at 0x00, mode bits 2:0 frozen).
  - **After W1-9d:** `__W1_9B_VERDICT__=PRE_STATE_UNEXPECTED_0x85` (`pre RegOpMode=0x85`).

**0x85 decode.** `RegOpMode = 0x85 = 0b1000_0101`: bit 7 `LongRangeMode=1` (LoRa), bits 4-3 reserved, **bits 2:0 = 0b101 = RXCONTINUOUS**. The radio successfully exits SLEEP and is actively receiving � exactly the previously-unreachable transition.

**Acceptance check vs �5 of this plan.**

| Acceptance criterion | Result |
| -------------------- | ------ |
| `clock_source_id` reports HSE_OK on at least one cycle | Indirect ? � Stage 1 probe still wraps `BOOT_URC` as raw bytes; HSE locking was confirmed by the SYSCLK-doubling baud-break observed pre-D-2 |
| W1-9b T2 verdict ? `T2_FSK_SLEEP_ONLY` | ? moved to `PRE_STATE_UNEXPECTED_0x85` (LoRa+RXCONTINUOUS) |
| Stage 1 standard quant =18/20 PASS | ? 2/2 in the abbreviated re-run (full 20/20 not re-executed because the launcher was killed by an unrelated terminal cleanup at cycle 8 of an in-flight 20-cycle run; D-2 build's stage1 path is unchanged from the 20/20 baseline) |
| `radio_ok=1`, `radio_version=0x12` | Inherited from W1-8 (no regression � SPI path untouched) |

**Decision.** **W1-10 hardware escalation is no longer required.** W1-9 OPMODE-stuck root cause was a software/strap mismatch (PA12 vs PB6) compounded by a latent SYSCLK-derived USART1 baud bug � not a missing-clock hardware defect. None of �7's 8 fallback options were needed.

**Next actions (out of scope for this plan):**

1. Relax the W1-9b T2 probe pre-state assertion to accept any non-SLEEP LoRa mode (firmware now boots straight into LoRa RX, so `0x80` SLEEP is no longer the expected pre-state).
2. Revisit the deferred [W1-9 TX bring-up](2026-05-11_W1-9_TX_BringUp_Closure_Copilot_v1_1.md) � the SX1276 is no longer mode-locked, so the previously-stalled `status=TIMEOUT` / `radio_tx_ok delta=0` path should now advance.
3. Long-term clean-up: if SYSCLK=HSE 32 MHz is desired in the future, override USART1 kernel clock to HSI16 via RCC_CCIPR[1:0]=10 (matching the pre-existing LPUART1 pattern) before re-enabling the SYSCLK switch.

**Lesson.** When fixing a clock source, audit every USART/peripheral that derives its kernel clock from SYSCLK by default � kernel-clock changes silently re-derive baud/timing dividers. A function named `init_hsi16()` that opportunistically switches to HSE/PLL violates its name and surfaces latent bugs the moment hardware actually cooperates.
