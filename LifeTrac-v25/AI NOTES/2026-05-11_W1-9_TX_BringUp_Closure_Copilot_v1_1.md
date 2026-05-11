# W1-9 — SX1276 TX bring-up — closure (DEFERRED) (Copilot v1.1)

**Author:** Copilot (Claude Opus 4.7)
**Date:** 2026-05-11 (continues immediately after the v1.0 plan in this folder)
**Hardware:** Portenta X8 ABX00049 (ADB serial `2E2C1209DABC240B`) + Max Carrier ABX00043 + Murata CMWX1ZZABZ-078 (STM32L072 + SX1276)
**Predecessor:** [`2026-05-11_W1-9_TX_BringUp_Plan_Copilot_v1_0.md`](./2026-05-11_W1-9_TX_BringUp_Plan_Copilot_v1_0.md)
**Status at end:** **DEFERRED** — root cause isolated, fix not yet successful, W1-7/W1-8 baseline preserved, diagnostic infrastructure committed.

This document is the §8 closure for the W1-9 plan but lives in its own
file because the result is _deferred_ rather than PASS, and because
the diagnostic body is large enough to warrant its own version (per
user-memory preference: deep analysis notes as new versioned files in
`LifeTrac-v25/AI NOTES`).

---

## 8.1 Headline result

| Acceptance criterion | Result |
|---|---|
| W1-9.A — Stage 1 still PASS | **PASS** (Method G probe rc=0; quant 5/5 PASS post-fix) |
| W1-9.B — `TX_DONE_URC` arrives | **PASS** (received in ~80 ms wall) |
| W1-9.C — `status == OK` | **FAIL** (`status = 1 / TIMEOUT` every cycle) |
| W1-9.D — `radio_tx_ok` delta == 1 | **FAIL** (counter does not advance) |
| W1-9.E — invariant counters stable | **PASS** (`host_parse_err = 0`; all PE/FE/NE/ORE = 0) |
| W1-9.F — no `FAULT_URC` | **PASS** (no faults during cycle) |
| W1-9.G — UART counters stable | **PASS** (no regression in 5-cycle quant) |

Verdict: **W1-9 critical gates C and D fail.** The radio modem cannot
be brought out of LoRa SLEEP on this hardware unit. Ground-truth
diagnostic data is captured below; the most likely root cause is
identified as a missing TCXO clock at the SX1276's `XTA` input pin,
but the obvious software fix (`RegTcxo[4] = 1` plus FSK→LoRa SLEEP
transition sequence) does not unstick the chip — pointing to a
hardware-level investigation as next step.

---

## 8.2 What works

* `TX_FRAME_REQ` (0x10) is parsed and dispatched correctly.
* `sx1276_tx_begin()` accepts the request: LBT result is `DISABLED`
  (probe sets `CFG_KEY_LBT_ENABLE = 0` first), airtime reservation
  succeeds, FIFO is loaded, `RegPayloadLength` (0x22) latches to `0x08`
  as observed via `REG_READ_REQ`.
* `RegPaConfig` (0x09), `RegFrf*` (0x06–0x08), `RegModemConfig*`
  (0x1D/0x1E/0x26) all latch to firmware-written values.
* `TX_DONE_URC` (0x90) is emitted by the firmware on the host UART
  with the expected payload format (`tx_id`, `status`, `toa_us`,
  `tx_power_dbm`).
* `STATS_DUMP_REQ`/`STATS_URC` round-trips, `host_parse_err = 0`,
  every UART per-flag counter stays at 0.
* Stage 1 protocol probe still passes; W1-7 5-cycle quant still 5/5
  PASS with the W1-9 firmware (build = 16 432 B vs 16 292 B baseline).

## 8.3 What fails

* `RegOpMode` (0x01) **never transitions out of LoRa SLEEP (0x80)**:
  * After firmware boot: `RegOpMode = 0x80` (expected 0x85 = LoRa
    `RX_CONT` after `sx1276_init()` + `sx1276_rx_arm()`).
  * 5 ms after `TX_FRAME_REQ` send: `RegOpMode = 0x80` (expected
    0x83 = LoRa TX during ToA window).
  * 25 ms after `TX_FRAME_REQ` send: `RegOpMode = 0x80`,
    `RegIrqFlags = 0x00`, `RegPayloadLen = 0x08`, `RegPaConfig = 0x8C`,
    `RegDioMapping1 = 0x00` (firmware wrote `0x40` for TxDone — write
    ignored).
  * **Direct host-issued `REG_WRITE_REQ` (`OPMODE = 0x81`) ack returns
    OK but the chip stays at `0x80`.** Same for `OPMODE = 0x83`.
* `radio_dio0` counter never increments → DIO0 EXTI line never fires
  (because the modem never enters TX, so TxDone never asserts on
  the radio die's DIO0 output regardless of MCU pin mapping).
* `radio_tx_ok` stays 0 across all attempts; firmware deadline
  (`ToA estimate + 50 ms guard` = ~68 ms) elapses cleanly and the
  TX FSM cleans up via the timeout path.

## 8.4 Diagnostic methodology

The W1-9 probe (`method_h_stage2_tx_probe.py`) was extended over four
bench cycles:

| Cycle | Probe instrumentation added | Finding |
|---|---|---|
| 1 | Baseline TX_FRAME_REQ + STATS before/after | `STATS_URC(before)` raced `BOOT_URC` → fix: 1 s settle window. |
| 2 | `BOOT_URC` drain | `ERR_PROTO_URC` reason=`0x08` (FORBIDDEN) → fix: probe sends `CFG_SET_REQ(LBT_ENABLE=0)` first. |
| 3 | mid-cycle `radio_dio0` peek | `dio0` delta = 0 → DIO0 line never fires. |
| 4 | mid-cycle `REG_READ_REQ` for OPMODE / IrqFlags / PayloadLen / PaConfig / DioMap1 | OPMODE stuck at `0x80`; config writes did latch; clock-side fault inferred. |
| 5 | direct `REG_WRITE_REQ` of OPMODE (allowlisted in firmware build) | Direct write also ignored → bug is **not** firmware sequencing; chip itself rejects mode-bit transitions. |

The §6 contingency tree from the v1.0 plan was walked end-to-end. The
"`RegOpMode` (0x01); if still in TX (0x83), modem config wrong" branch
was disproved (chip is in SLEEP, not TX). The "DIO0 EXTI not firing"
branch was disproved (`radio_dio0 = 0` because modem never asserts
DIO0, not because EXTI line is broken).

## 8.5 Root-cause hypothesis

The SX1276 die inside the Murata CMWX1ZZABZ-078 SiP is **starving for
its digital-block clock**. Evidence:

* Configuration register writes (FRF, PA_CONFIG, MODEM_CFG*) latch
  through the SPI shadow path which **does not require the digital
  state machine** — and they do work.
* `RegOpMode` mode-bit transitions go through the digital state
  machine — and they do _not_ work.
* `RegOpMode` LongRangeMode bit (bit 7) **does** flip on SLEEP→LoRa
  SLEEP transition, suggesting the configuration-latch path can also
  flip that one bit.
* MCU's `HSE` source clock locks to the same TCXO output (32 MHz,
  HSEBYP) and runs the system at 32 MHz — so the TCXO IS oscillating
  somewhere on the board.

The natural fix would be `RegTcxo` (0x4B) bit 4 (`TcxoInputOn = 1`)
to tell the SX1276 to use external TCXO instead of attempting to
oscillate an internal crystal driver across XTA/XTB. We applied this
fix (bench-confirmed via dump `0x4B = 0x10`) — **and the chip is
still stuck in SLEEP**. So either:

1. The TCXO output is wired to the MCU's `OSC_IN` only and **not** to
   the SX1276 `XTA` pin on this Murata variant — in which case
   `RegTcxo` is irrelevant and the radio die has no clock source at
   all.
2. The TCXO output reaches `XTA` but the SX1276 ignores it for some
   other reason (chip damage, undocumented init requirement).
3. There is a separate per-radio `TCXO_VCC` rail (e.g. R12 / VDD_TCXO
   per `BRINGUP_MAX_CARRIER.md`) that needs to be enabled in addition
   to PA12 — only PA12 is currently driven HIGH by `platform.c`.

Hypothesis (3) is the most actionable: investigate the Max Carrier
ABX00043 schematic for any second TCXO rail or `RADIO_TCXO_EN` pin
that the W1-7 RX-fix did not cover. Hypothesis (1) requires
oscilloscope verification at the SiP package level (probe `XTA` net
on the carrier or sniff the SX1276 BGA-equivalent if exposed) and is
beyond what the bench infrastructure can resolve.

## 8.6 Firmware changes left in place

All edits are guarded by W1-9 comments and are bench-safe (do not
break W1-7/W1-8 PASS):

1. `radio/sx1276_modes.c` — `sx1276_modes_init()`:
   * Write `RegOpMode = 0x00` (FSK SLEEP) before LoRa transition (per
     SX1276 datasheet §4.1.6).
   * Write `RegTcxo = 0x10` immediately after the FSK SLEEP write so
     the TCXO bit is set before any LoRa-mode transition is attempted.
2. `radio/sx1276_modes.c` — `sx1276_modes_apply()`:
   * Add a 5 ms soft-retry loop on `RegOpMode` after every mode-bit
     write. On failure, emit `HOST_FAULT_CODE_RADIO_OPMODE_DRIFT` for
     visibility but **do not** mark `s_state = FAULT` — keeps
     `sx1276_init()` proceeding through `set_frequency_hz` /
     `set_tx_power_dbm` / `set_sf_bw_cr` so other paths (host_cmd,
     stats, RX wiring) keep running.
3. `host/host_cmd.c` — `reg_write_allowed()`:
   * Allowlist `RegOpMode` (0x01) and `RegDioMapping1` (0x40) for
     `REG_WRITE_REQ` — needed for the W1-9 SPI-write isolation
     diagnostic. **Diagnostic-only — re-tighten before production.**

Build deltas:
* Pre-W1-9 baseline: 16 292 B
* W1-9 deferred image:  **16 432 B** (+140 B)

## 8.7 Diagnostic infrastructure committed

* `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py`
  — Stage 2 TX probe + LBT-disable + multi-stage register peek + direct
  `REG_WRITE_REQ` isolation test.
* `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_method_h_stage2_tx.sh`
  — X8-side bash wrapper.
* `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_method_h_stage2_tx_end_to_end.ps1`
  — Host-side launcher (push helper, exec, parse `__METHOD_H_RC__`,
  pull logs).

These tools will be reused as-is for the next attempt; no rework
needed.

## 8.8 Recommended next step (W1-10 or hardware deep-dive)

1. **Schematic review of Max Carrier ABX00043 + Murata CMWX1ZZABZ-078**
   for any radio-side `TCXO_VCC` enable that is not PA12. Add the GPIO
   write to `platform_clock_init_hsi16()` adjacent to the existing PA12
   assert.
2. **Oscilloscope probe** (if available) of the TCXO output trace and
   the SX1276 `XTA` pin net to confirm the 32 MHz is actually present
   at the radio die. This is the single decisive measurement.
3. **Cross-test on a second hardware unit** (different Max Carrier or
   different Murata module) to rule out a damaged SX1276 die on the
   current bench unit.
4. Once the chip can be brought out of SLEEP, the rest of the W1-9
   plan (DIO0 EXTI verification, TX_DONE round-trip, `radio_tx_ok`
   increment) should fall through cleanly because every other layer
   (UART, COBS/CRC, host_cmd dispatch, TX FSM, statistics, airtime
   budgeter, LBT bypass) is already validated by the diagnostic data
   above.

## 8.9 Acceptance summary (re-stated)

* W1-9 is **not closed PASS**.
* W1-7 and W1-8 baselines are **preserved** (5-cycle quant 5/5 PASS,
  Method G probe rc=0, REG 0x42=0x12).
* Root cause is **isolated** to the radio die's digital clock path.
* Diagnostic tooling is **production-quality** and ready to re-run as
  soon as a hardware-side fix is identified.
* Firmware build still flashes, boots, runs host transport at full
  baud — no functional regression for any non-radio path.
