# W1-9c Software Follow-up Closure — Copilot v1.0

**Date:** 2026-05-12
**Author:** GitHub Copilot (autonomous agent run)
**Status:** CLOSED — fixes kept; W1-10 hardware escalation re-confirmed
**Predecessor:** [2026-05-11_W1-9b_OPMODE_Stuck_Probe_Closure_Copilot_v1_0.md](2026-05-11_W1-9b_OPMODE_Stuck_Probe_Closure_Copilot_v1_0.md)

---

## 1. Motivation

After W1-9b returned verdict `T2_FSK_SLEEP_ONLY` and recommended escalating to
W1-10 hardware investigation, the user asked for one more software pass:
*"please come up with some more software firmware ideas. search the internet
and do more research. then proceed if you find some good ideas that other
people have tried."*

This note documents the research, the two fixes that were applied, and the
bench result.

---

## 2. Reference research

Fetched canonical Semtech reference driver code from raw GitHub:

| File | Purpose |
| --- | --- |
| `Lora-net/LoRaMac-node/src/radio/sx1276/sx1276.c` | `SX1276Init`, `SX1276SetOpMode` |
| `Lora-net/LoRaMac-node/src/boards/B-L072Z-LRWAN1/sx1276-board.c` | `SX1276Reset`, `SX1276SetBoardTcxo` |
| `ABX00043-schematics.pdf`, rev V3.12, sheet 6 `LORA` | Max Carrier U23 Murata TCXO rail / clock routing |

Two concrete deltas vs our production driver were identified:

### Delta A — NRESET release pattern

**Semtech reference (`SX1276Reset()` in `sx1276-board.c`):**
```c
SX1276SetBoardTcxo( true );                                                    // TCXO ON before reset
GpioInit( &Reset, RADIO_RESET, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );    // drive LOW
DelayMs( 1 );
GpioInit( &Reset, RADIO_RESET, PIN_INPUT,  PIN_PUSH_PULL, PIN_NO_PULL, 1 );    // Hi-Z release
DelayMs( 6 );
```

**Our pre-W1-9c driver (`sx1276_radio_reset()` in `radio/sx1276.c`):**
```c
gpio_write(SX1276_RESET_PORT, SX1276_RESET_PIN, 0U);    // drive LOW
platform_delay_ms(5U);
gpio_write(SX1276_RESET_PORT, SX1276_RESET_PIN, 1U);    // ACTIVE PUSH-PULL HIGH (wrong)
platform_delay_ms(20U);
```

The MCU actively driving NRESET HIGH fights the SX1276's internal pull-up
during the brief sampling window. The Semtech-canonical Hi-Z release lets
the chip's own pull-up bring the line up cleanly.

### Delta B — RegTcxo (0x4B) write clobbers reserved bits

**Datasheet rev 7 §4.1.7:** RegTcxo bits [3:0] are *"Reserved — 1001 — Retain
default value."* Reset value = 0x09.

**Our pre-W1-9c W1-9 fix #2:** `sx1276_write_reg(0x4BU, 0x10U);`
This sets bit 4 (TcxoInputOn=1) but **clears the reserved 0x09 default**.

**Murata `mlm32l07x01` reference and LoRaMac-node both use RMW** to preserve
the reserved bits, producing the canonical value 0x19.

---

## 3. Fixes applied

### Fix C-1 — Hi-Z NRESET release

[radio/sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) `sx1276_radio_reset()`:

```c
gpio_mode_output(SX1276_RESET_PORT, SX1276_RESET_PIN);
gpio_write(SX1276_RESET_PORT, SX1276_RESET_PIN, 0U);
platform_delay_ms(1U);
gpio_mode_input(SX1276_RESET_PORT, SX1276_RESET_PIN);   // Hi-Z release
platform_delay_ms(6U);
```

(`gpio_mode_input` clears PUPDR → no MCU pull, relies on SX1276 internal pull-up.)

### Fix C-2 — RegTcxo read-modify-write

[radio/sx1276_modes.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_modes.c) `sx1276_modes_init()`:

```c
sx1276_write_reg(0x4BU, sx1276_read_reg(0x4BU) | 0x10U);   // 0x09 | 0x10 = 0x19
```

### Build artefact

| Metric | Value |
| --- | --- |
| `firmware.bin` size | 16432 B (unchanged from W1-9b) |
| Compiler warnings | 0 |
| `make` exit code | 0 |

---

## 4. Bench result

Test: `run_method_h_stage2_tx_end_to_end.ps1 -AdbSerial 2E2C1209DABC240B -Probe fsk`
Evidence dir: `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/W1-9b_fsk_2026-05-11_085842/`

```
T2 step2: pre RegOpMode=0x80 (raw=0180)                                  ← LoRa SLEEP, expected
T2 step3: REG_WRITE OPMODE=0x00 ack=0100                                 ← write FSK SLEEP, accepted
T2 step5: post-FSK-SLEEP RegOpMode=0x00 (raw=0100) elapsed=23.92ms       ← entered FSK SLEEP ✓
T2 step6: REG_WRITE OPMODE=0x01 ack=0101                                 ← write FSK STDBY, accepted
T2 step8: post-FSK-STDBY RegOpMode=0x00 (raw=0100) elapsed=23.65ms       ← STILL IN SLEEP ✗
T2 restore: RegOpMode=0x80 (raw=0180)                                    ← LoRa SLEEP restored OK
__T2_RADIO_VERDICT__=FSK_SLEEP_ONLY
__W1_9B_VERDICT__=T2_FSK_SLEEP_ONLY
```

**Verdict identical to W1-9b baseline.** Chip can ENTER any SLEEP state but
cannot LEAVE SLEEP regardless of which modem family (FSK/LoRa) we target.

---

## 5. Interpretation

The two fixes are correct per Semtech reference and are kept in the firmware,
but they **do not unblock TX**. This rules out two whole categories of
software root cause:

- **NOT** a reset-polarity / driver-fight glitch (Hi-Z release matches
  Semtech reference exactly; symptom unchanged).
- **NOT** a reserved-bit corruption in RegTcxo (now writes canonical 0x19;
  symptom unchanged).

Combined with W1-9b's evidence that even direct host `REG_WRITE_REQ` of
RegOpMode cannot move the chip out of SLEEP (firmware control path is not
the gate), this is now a textbook *"digital block has no clock"* symptom:

> The chip can enter SLEEP (no clock needed — purely combinational write).
> The chip cannot leave SLEEP (needs the 32 MHz TCXO clock to run the mode-
> transition state machine).

Follow-up schematic research against the official Arduino Max Carrier
ABX00043 schematic (rev V3.12, sheet 6 `LORA`) refines the hardware
hypothesis:

- U23 is the Murata `CMWX1ZZABZ-078` SiP.
- U23 pin 48 is `VDD_TCX0`. It is on the `LORA_VDD_TCX0` rail with C13/C14/C15
   decoupling and is tied to `+3V3` on the carrier. This rail is not switched
   by the L072 firmware in the B-L072Z-LRWAN1 sense.
- U23 pin 1 is `PA12/USB+`. The only PA12-to-`VDD_TCX0` path is optional
   resistor `R86` (0 ohm), and the schematic marks it as DNP/no-fit. Therefore
   `PA12` is **not** the TCXO enable on ABX00043.
- U23 pin 39 is `PB6/LPTIM1_ETR`. `PB6` connects to `VDD_TCX0` through `R12`
   (0 ohm). Because `VDD_TCX0` is tied to `+3V3`, `PB6` must be treated as a
   strapped/observed rail, not as a safe firmware-controlled TCXO power switch.
- U23 pin 47 `TCX0_OUT` is linked to U23 pin 46 `PH0-OSC_IN` through `R19`
   (0 ohm), providing the external 32 MHz clock path into the STM32L072 HSE
   input.

Net: the earlier PA12-enable hypothesis is now falsified. The Max Carrier
intends the TCXO rail to be always powered. If the SX1276 still cannot leave
SLEEP, W1-10 should look for an actual missing/failed 32 MHz clock despite the
rail being nominally always on: absent `VDD_TCX0`, open/failed `R19` clock path,
failed oscillator inside U23, or an internal SiP/SX1276 clock-path issue.

---

## 6. Decision

**RE-AFFIRM:** `ESCALATE_TO_W1-10_HARDWARE`

W1-10 must include — at minimum:

1. **DMM rail check**: verify `LORA_VDD_TCX0` / U23 pin 48 / C13-C15 is at
   approximately 3.3 V whenever the LoRa module is powered.
2. **No-drive safety check**: confirm `R86` is not populated / no continuity
   exists from `PA12` to `VDD_TCX0`, and do **not** drive `PB6` or `PA12` as a
   TCXO power-enable in firmware.
3. **Scope clock path**: probe U23 pin 47 `TCX0_OUT` and U23 pin 46
   `PH0-OSC_IN` / both sides of `R19`. A 32 MHz waveform must be present before
   firmware writes `OPMODE = STDBY`.
4. **Firmware stance**: keep `RegTcxo` RMW (`0x4B |= 0x10`) and Hi-Z reset
   fixes, but treat TCXO power control as board-fixed on ABX00043. Add only
   diagnostics around HSE/clock presence until the rail/scope checks are known.

---

## 7. Acceptance block

```
KEY=VALUE
W1-9C_FIX_C1=NRESET_HIZ_RELEASE_APPLIED
W1-9C_FIX_C2=REG_TCXO_RMW_APPLIED_VALUE_0x19
W1-9C_BUILD=PASS
W1-9C_BUILD_BIN_BYTES=16432
W1-9C_BENCH_PROBE=fsk
W1-9C_BENCH_VERDICT=T2_FSK_SLEEP_ONLY
W1-9C_BENCH_RESULT=NO_CHANGE_VS_W1-9B
W1-9C_DECISION=ESCALATE_TO_W1-10_HARDWARE
W1-9C_RESEARCH_SOURCES=Lora-net/LoRaMac-node@master,Semtech_SX1276_DS_rev7
W1-9C_SCHEMATIC_SOURCE=ABX00043-schematics.pdf_revV3.12_sheet6_LORA
W1-9C_TCXO_PA12_ENABLE=NO_R86_DNP
W1-9C_TCXO_POWER=VDD_TCX0_TIED_TO_LORA_3V3
W1-9C_TCXO_PB6=STRAPPED_TO_VDD_TCX0_VIA_R12_DO_NOT_DRIVE
W1-9C_TCXO_NEXT_CHECK=VDD_TCX0_RAIL_AND_TCX0_OUT_R19_SCOPE
W1-9C_FIXES_RETAINED=YES
```

---

## 8. Files changed

| File | Δ |
| --- | --- |
| [radio/sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) | `sx1276_radio_reset()` rewritten — Hi-Z release |
| [radio/sx1276_modes.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_modes.c) | RegTcxo write switched to RMW |

No CI guard violations. `Makefile` line 218–248 enforcement: only
`sx1276_modes.c` writes RegOpMode — preserved.
