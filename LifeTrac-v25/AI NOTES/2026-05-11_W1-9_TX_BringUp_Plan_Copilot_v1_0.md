# W1-9 — SX1276 TX bring-up plan (Copilot v1.0)

**Author:** Copilot (Claude Opus 4.7)
**Date:** 2026-05-11 (late night, immediately following W1-8 closure)
**Hardware:** Portenta X8 ABX00049 (ADB serial `2E2C1209DABC240B`) + Max Carrier ABX00043 + Murata CMWX1ZZABZ-078 (STM32L072 + SX1276)
**Predecessor:** [`2026-05-11_W1-8_SX1276_SPI_BringUp_Plan_Copilot_v1_0.md`](./2026-05-11_W1-8_SX1276_SPI_BringUp_Plan_Copilot_v1_0.md) (CLOSED, PASSED)
**Status at start:** firmware 16 292 B already flashed, Method G probe rc=0, REG 0x42=0x12, W1-7 quant 20/20 PASS.

---

## 1. Goal

Validate the **transmit path** of the SX1276 LoRa radio end-to-end:

1. Compose a `TX_FRAME_REQ` (`HOST_TYPE_TX_FRAME_REQ` = `0x10`) over the host UART.
2. Firmware loads the FIFO, switches to TX mode (with `DIO_MAPPING1 = 0x40` ⇒ DIO0 = TxDone).
3. SX1276 transmits one LoRa packet on the default profile (915 MHz, SF7, BW 250 kHz, CR 4/5, TX power 14 dBm).
4. DIO0 rises on TxDone → STM32 EXTI fires → `sx1276_tx_poll()` reads `RegIrqFlags`, sees `TX_DONE` (0x08), emits `TX_DONE_URC` (`HOST_TYPE_TX_DONE_URC` = `0x90`).
5. Host probe receives `TX_DONE_URC` with `status == SX1276_TX_STATUS_OK` (0).

This is the minimum-viable proof that:

- The SPI-driven register configuration (frequency / modem / PA / DIO mapping) takes effect.
- The radio actually enters and exits TX mode without LBT or airtime abort.
- The DIO0 EXTI line fires and is observed by the foreground poll loop.
- The RF switch routing (`PA1` TX/RX select + `PC2` TX boost enable) is asserted correctly.

Out of scope for W1-9 (deferred to W1-10): receiving the packet on a second radio, RSSI/SNR validation, RX path bring-up, TX/RX ping-pong.

## 2. What is already in place (no firmware change needed)

| Component | File | Status |
|---|---|---|
| TX state machine | [`firmware/murata_l072/radio/sx1276_tx.c`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c) | implemented (LBT + airtime + FIFO load + TX → wait DIO0 → cleanup) |
| TX_FRAME_REQ handler | [`host/host_cmd.c`](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c) `handle_tx_frame()` | implemented (parses `{u8 tx_id, u8 length, u8 payload[length]}`) |
| TX_DONE_URC emitter | [`host/host_cmd.c`](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c) `host_cmd_emit_tx_done()` | emits `{u8 tx_id, u8 status, u32 toa_us, u8 tx_power_dbm}` |
| DIO0 → EXTI4_15 → `s_irq_events` | [`radio/sx1276.c`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) | implemented; SX1276_EVT_DIO0 set in handler |
| Mode → DIO mapping for TX | [`radio/sx1276_modes.c`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_modes.c) `k_mode_descs[]` | TX entry sets `DIO_MAPPING1 = 0x40` ⇒ DIO0 = TxDone |
| Default profile applied at init | [`radio/sx1276.c`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) `sx1276_init()` | 915 MHz, SF7, BW 250, CR 4/5, 14 dBm |
| Stats counter `radio_tx_ok` | [`host/host_stats.c`](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_stats.c) | incremented in `sx1276_tx_poll()` on success |

**Conclusion:** the firmware side is complete. W1-9 is purely a **host-side test harness + bench validation** task.

## 3. New artifacts to create

| File | Purpose |
|---|---|
| `firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py` | Python probe: open `/dev/ttymxc3`, snapshot `radio_tx_ok` from `STATS_URC`, send `TX_FRAME_REQ`, wait for `TX_DONE_URC`, snapshot `radio_tx_ok` again, validate. |
| `firmware/x8_lora_bootloader_helper/run_method_h_stage2_tx.sh` | X8-side bash wrapper: optionally reflash, boot user app, run the Stage 2 probe. |
| `firmware/x8_lora_bootloader_helper/run_method_h_stage2_tx_end_to_end.ps1` | Host PowerShell launcher: push probe + helper, optionally build, optionally flash, run end-to-end, collect logs. |

Pattern mirrors the existing Stage 1 harness (`method_g_stage1_probe.py` / `run_method_g_stage1.sh` / `run_method_g_stage1_end_to_end.ps1`).

## 4. Probe wire-protocol layout

```
TX_FRAME_REQ (0x10):
  payload = [u8 tx_id] [u8 length] [u8 payload[length]]
  example: tx_id=0x42, length=0x08, payload="LIFETRAC"
           = 42 08 4C 49 46 45 54 52 41 43

TX_DONE_URC (0x90):
  payload = [u8 tx_id] [u8 status] [u32 time_on_air_us LE] [u8 tx_power_dbm]
  PASS condition: tx_id matches request, status == 0 (SX1276_TX_STATUS_OK)
```

`time_on_air_us` for SF7 / BW 250 kHz / CR 4/5 / 8 B payload / preamble 8 sym / explicit header / CRC on / no LDRO ≈ **~33–35 ms** (8.704 ms preamble + 8 payload symbols + header). The probe will accept anything in `[10 000, 200 000]` µs as plausible — the precise value is firmware-computed and not the focus of this gate.

## 5. Acceptance criteria

| # | Criterion | How verified |
|---|---|---|
| W1-9.A | Method G Stage 1 still PASS after Stage 2 sequence | Run Stage 1 probe before and after — `probe_rc=0` both times. |
| W1-9.B | `TX_DONE_URC` arrives within 5 s of `TX_FRAME_REQ` | Probe blocks on `link.read_frames()` for the URC. |
| W1-9.C | Returned `tx_id` matches the one we sent | Byte-for-byte. |
| W1-9.D | Returned `status == 0` (SX1276_TX_STATUS_OK) | Field check. |
| W1-9.E | `radio_tx_ok` counter increments by exactly 1 | `STATS_URC` snapshot before vs after. |
| W1-9.F | No new `host_parse_err`, `radio_tx_abort_lbt`, `radio_tx_abort_airtime`, or FAULT_URC during the cycle | STATS_URC delta = 0 on these counters. |
| W1-9.G | UART error counters all stay 0 (no W1-7 regression) | Reuse the per-flag PE/FE/NE/ORE check. |

A pass on W1-9.A through W1-9.G means the SX1276 has produced exactly one over-the-air LoRa frame on 915 MHz @ 14 dBm and asserted TxDone. **This does not prove the RF chain is calibrated** (no second radio on the bench yet), only that the digital side of the transmitter is healthy.

## 6. Risk / contingency

| Symptom | Likely cause | First fix |
|---|---|---|
| `TX_DONE_URC` never arrives | DIO0 EXTI not firing (line stuck low or polarity wrong) | Read `RegIrqFlags` (0x12) via `REG_READ_REQ` — if bit 3 (TX_DONE) is set, the issue is host-side EXTI; if not, the radio never finished TX (modem config mismatch). |
| `status == TIMEOUT` (1) | Foreground saw deadline elapse before DIO0 ⇒ either ToA estimate too tight or DIO0 line dead | Inspect `RegOpMode` (0x01); if still in TX (0x83), modem config wrong. |
| `status == LBT_ABORT` (2) | Channel busy at 915 MHz from ambient noise | LBT check is currently disabled in default config — only fire if cfg changed. Set tx power lower or disable LBT. |
| `ERR_PROTO_QUEUE_FULL` | TX state machine is busy | A previous `TX_FRAME_REQ` is still pending; wait or reset. |
| `radio_tx_abort_airtime` increments | Airtime budget exhausted in this 1-hour window | Reset stats with `STATS_RESET_REQ` (0x40), retry. |

## 7. Bench plan

1. Confirm Stage 1 still passes (`run_method_g_stage1_end_to_end.ps1`) — sanity gate.
2. Push the new probe + shell + launcher to the X8.
3. Run `run_method_h_stage2_tx_end_to_end.ps1 -AdbSerial 2E2C1209DABC240B -SkipFlash` (image already loaded from W1-8).
4. If the first try fails, walk the contingency tree in §6.
5. On success, append §8 closure to this file with `tx_id`, `status`, `time_on_air_us`, `tx_power_dbm`, `radio_tx_ok` delta.
6. Update `LifeTrac-v25/DESIGN-CONTROLLER/TODO.md` with **Step 8 — TX bring-up — CLOSED**.
7. Update `/memories/repo/lifetrac-portenta-x8-lora.md`.

Awaiting the bench cycle to populate §8.


---


## 8. Closure pointer

This plan's closure was captured in a separate v1.1 file because the result was deferred (not PASS). See [2026-05-11_W1-9_TX_BringUp_Closure_Copilot_v1_1.md](./2026-05-11_W1-9_TX_BringUp_Closure_Copilot_v1_1.md) for the full bench-evidence summary, root-cause hypothesis, firmware deltas left in place, and recommended next steps.

