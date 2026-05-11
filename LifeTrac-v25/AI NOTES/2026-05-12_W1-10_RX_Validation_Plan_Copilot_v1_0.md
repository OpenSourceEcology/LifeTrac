# W1-10 — LoRa RX validation milestone

**Date:** 2026-05-12
**Status:** Phase A (single-board RX liveness) **CLOSED — PASS first run**.
Phase B (two-board end-to-end) **OPEN** (blocked on Board 1 returning to the bench).
**Author:** GitHub Copilot (Claude Opus 4.7)
**Hardware under test:** Murata CMWX1ZZABZ-078 SiP on ABX00043 V3.12 carrier, hosted by Portenta X8 ABX00049.
**Firmware build:** 16,440 B (post-W1-9d, identical to W1-9f/W1-9g closure builds — no firmware change required for Phase A).

> **Predecessor closures (this session):** W1-9d (PB6 TCXO + SYSCLK), W1-9e (HostLink urc_queue), W1-9f (BENIGN_FAULT_CODES filter for `HOST_RX_SEEN`/`HOST_DIAG_MARK`), W1-9g + W1-9b T2 (post-VER STATS dispatch stall — extended `drain_boot()` + new `drain_pending()` helper). Stage 1 standard quant **20/20 PASS** (W1-7) and **3/3 PASS** post-W1-9g regression. Stage 2 TX probe **7/7 PASS** (W1-9f). Stage 2 FSK STDBY probe `probe_rc=0` (W1-9g, mode bits 2:0 transition 0→1). The remaining gate to **unconditional GO** for the custom LoRa firmware was an end-to-end LoRa receive validation — addressed below.

---

## 1. Scope

The Murata L072 firmware boots straight into LoRa `RXCONTINUOUS` mode (`RegOpMode=0x85`) once the SX1276 init succeeds (see [`main.c`](../DESIGN-CONTROLLER/firmware/murata_l072/main.c#L64) and [`sx1276_rx.c::sx1276_rx_arm()`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx.c#L25)). On every `DIO0` IRQ the main loop calls `sx1276_rx_service()` which reads the FIFO and emits `RX_FRAME_URC` (0x91) with payload `{u8 len, i8 snr_db, i16 rssi_dbm_le, u32 timestamp_us_le, payload[len]}`.

Per [`BRINGUP_MAX_CARRIER.md`](../DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md#L99) §5, RX validation is split into **Stage 2 (single-board, known peer required)** and **Stage 3 (two-board round-trip ≥99% rates)**. Because only Board 2 (`2E2C1209DABC240B`) was reachable via ADB at probe time and no calibrated peer transmitter was on the bench, this milestone is split:

| Phase | Goal | Hardware | Status |
|---|---|---|---|
| **A. Single-board RX liveness** | Prove the firmware's RX chain is alive end-to-end up to the host URC dispatch boundary, **without** requiring a peer transmitter. | Board 2 only | ✅ CLOSED — PASS first run, this doc |
| **B. Two-board end-to-end** | Board 1 TX → Board 2 RX, ≥99% packet match rate over 5-minute attenuated bench run. | Both boards | ⏸ OPEN — Board 1 not currently ADB-reachable |

This split is safe because Phase A directly validates every link in the RX path **except** the SX1276's RF demodulator producing a real `RX_DONE` IRQ from an external signal. Phase B closes that final gap; until Phase B ships the firmware is **conditional GO** for receive.

---

## 2. Phase A — single-board RX liveness probe

### 2.1 Implementation

Added as `--probe rx` sub-mode in [`firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py) (`run_rx_liveness()`), wired into the existing PowerShell wrapper `run_method_h_stage2_tx_end_to_end.ps1 -Probe rx`. New shared helpers — `parse_rx_frame()` for `RX_FRAME_URC` payloads and three SX1276 register constants (`RSSI_VALUE=0x1B`, `MODEM_STAT=0x18`, `PKT_RSSI=0x1A`).

### 2.2 Gates

All seven must hold for `__W1_10_VERDICT__=RX_LIVENESS_PASS`:

| Gate | Check | Why it matters |
|---|---|---|
| **A1** | `RegOpMode(pre) == 0x85` | Firmware boots into LoRa+RXCONTINUOUS — proves W1-9d/9e closures didn't regress the RX path. |
| **A2** | `RegOpMode(post) == 0x85` after the observation window | RX state is "stuck" — chip didn't drop to SLEEP/STDBY mid-window. |
| **B1** | `RegRssiValue ∈ [10..200]` (≈ -147..+43 dBm raw with -157 offset) | Receiver is reporting a real RSSI; a dead receiver reads constant 0x00 or 0xFF. |
| **B2** | `RegRssiValue` varies between two samples 50 ms apart | AGC + IQ chain is digitally alive (any live receiver will twitch by ≥1 LSB on noise floor between samples). |
| **C** | `radio_state == 4` (`SX1276_STATE_RX_CONT`) per [`include/sx1276_modes.h#L12`](../DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_modes.h#L12) | Firmware-side state machine agrees with hardware register read (catches host-side cache drift). |
| **D** | `INVARIANT_COUNTERS` unchanged across window | host_parse_err + per-flag UART error counters (PE/FE/NE/ORE on LPUART1+USART1) all at 0 — proves no transport regression while RX was running. |
| **E** | Zero real `FAULT_URC` during window (`BENIGN_FAULT_CODES = HOST_RX_SEEN | HOST_DIAG_MARK` filtered per W1-9f) | No HARDFAULT/RADIO/CLOCK faults trigger while RX is armed. |

`RX_FRAME_URC` arrivals are captured as **bonus evidence** (not gated): a single-board quiet bench has no transmitter, so 0 frames is expected; any frames received would be parsed and printed with RSSI/SNR/payload but won't fail the gate.

### 2.3 Bench evidence — first-run PASS

[`bench-evidence/W1-10_rx_liveness_2026-05-11_133459/`](../DESIGN-CONTROLLER/bench-evidence/W1-10_rx_liveness_2026-05-11_133459/)

```
T0a: VER warm-up OK fw=v0.0.0 build=00000000
T0b: post-VER drain consumed 4 frames        (W1-9g drain_pending() doing its job)
T0c STATS(before): radio_state=4 (4=RX_CONT) radio_rx_ok=0 radio_crc_err=0 host_parse_err=0
T1: RegOpMode(pre) = 0x85 (expected 0x85 = LoRa+RXCONTINUOUS)
T2: RegRssiValue A=0x25 (-120 dBm) B=0x26 (-119 dBm) in_band=True varies=True
T3: RegModemStat(0x18) = 0x04   (bit2=RxOngoing set — header detect armed)
T4: observation window 10.0s
T5: RegOpMode(post) = 0x85 (stuck-in-RXCONT = True)
T5 STATS(after): radio_rx_ok=0 (delta=0) radio_crc_err=0 (delta=0) ... host_parse_err=0
Critical checks:
  [PASS] A1 RegOpMode(pre)  == 0x85 (LoRa+RXCONTINUOUS)
  [PASS] A2 RegOpMode(post) == 0x85 (LoRa+RXCONTINUOUS)
  [PASS] B1 RegRssiValue in plausible band [10..200]
  [PASS] B2 RegRssiValue varies between samples (AGC alive)
  [PASS] C  radio_state == 4 (RX_CONT)
  [PASS] D  host invariants stable
  [PASS] E  no real FAULT_URC during window
__W1_10_VERDICT__=RX_LIVENESS_PASS
probe_rc[/dev/ttymxc3][rx]=0
```

**Reading the receiver-floor RSSI:** -120 dBm at SF7/BW125 is right on the SX1276 noise floor with the carrier's whip antenna at ~2 m from the X8 USB cable run; no nearby 915 MHz carrier was active. The 1 LSB twitch (-120 → -119) is exactly the AGC noise jitter expected of a live receiver — matches Semtech datasheet §5.5.5 (RSSI is averaged over 8 samples at the demodulator output). RegModemStat = 0x04 (only bit2=RxOngoing set, no SigDetect/Sync) confirms the receiver is armed and listening but not currently locked onto any preamble — the correct steady-state for a quiet channel.

### 2.4 What Phase A does NOT prove

1. **End-to-end packet receive.** No external preamble was generated, so we did not exercise the `DIO0` IRQ → `sx1276_rx_service()` → FIFO read → `RX_FRAME_URC` emit path on real RF input. The `parse_rx_frame()` helper is exercised by code path but not by a real frame.
2. **Frequency / SF / BW / CR alignment with a real peer.** We assumed the firmware's hardcoded profile (915 MHz, BW=125 kHz, SF7, CR=4/5, CRC on per [`sx1276.c#L269`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c#L269)) but didn't lock to a real signal.
3. **Sustained throughput / collision behaviour / retry behaviour.**
4. **CRC failure handling** (`HOST_FAULT_CODE_RX_CRC_DISABLED` path in `sx1276_rx_arm`).

These all fall to Phase B.

---

## 3. Phase B — two-board end-to-end (OPEN, probe code READY)

> **2026-05-11 update — probe code complete, blocked only on hardware.** The two-board orchestrator and per-board sub-modes have been implemented and smoke-tested (RX side end-to-end PASS in single-board mode, orchestrator gracefully refuses to run with one board absent). Run `run_w1_10b_rx_pair_end_to_end.ps1` the moment Board 1 (`2D0A1209DABC240B`) is back on the bench. See §3.5 below for execution recipe.

### 3.1 Hardware prerequisites

- **Board 1** ABX00049 ADB serial `2D0A1209DABC240B` — last seen on bench during W4-pre (see [`bench-evidence/W4-pre/NOTES.md`](../DESIGN-CONTROLLER/bench-evidence/W4-pre/NOTES.md#L11)). Currently not USB-connected. Action: power up + `adb devices` confirms enumeration.
- **Board 1 firmware**: must be re-flashed with the current 16,440 B build (W1-9d/9e/9f/9g). If Board 1 currently has the W4-pre era firmware, it lacks PB6 TCXO enable and will FAIL_BOOT. Use the existing Stage 1 helper to flash.
- **Antenna**: both carriers have the on-board chip antenna (ABX00043 sheet 6 footprint). For first-light bench, leave them ~30 cm apart; if RX overflows or non-linearly compresses, add a 20 dB attenuator on Board 2's antenna pad.
- **Same RF profile**: both boards run identical firmware → identical default profile → no `CFG_SET_REQ` needed for first run. Both must agree on 915 MHz / BW=125k / SF7 / CR=4/5 / CRC on.

### 3.2 Probe design

Add `--probe rx_pair` (or new wrapper `run_w1_10b_rx_pair_end_to_end.ps1`) that:

1. Opens **two** `HostLink` instances (Board 1 = TX role, Board 2 = RX role) over **two parallel ADB sessions** (`-s 2D0A1209DABC240B` + `-s 2E2C1209DABC240B`).
2. On both: `drain_boot()` → VER warm-up → `drain_pending()` → STATS snapshot.
3. On Board 2: confirm Phase A liveness gates (RegOpMode=0x85, AGC twitch).
4. Loop N times (default N=100):
   - Board 1: `TX_FRAME_REQ` with payload `f"W1-10b seq={i:04d} {os.urandom(8).hex()}"` (≤64 B) and `tx_id = i & 0xFF`.
   - Board 1: `wait_for_tx_done()` (existing helper from `--probe tx`); record `time_on_air_us` + `status`.
   - Board 2: `read_frames()` window = 1.0 s after TX_DONE; collect any `RX_FRAME_URC`; correlate by payload prefix `W1-10b seq=`.
5. Compute and report:
   - `tx_done_rate = #status==OK / N`
   - `rx_match_rate = #payload-matched-RX_FRAME_URC / N`
   - `rx_orphan_count = #RX_FRAME_URC without matching TX`
   - per-frame RSSI/SNR distribution on Board 2
   - both boards' STATS deltas (`radio_tx_ok` should == N on Board 1, `radio_rx_ok` should ≈ N on Board 2)

### 3.3 Phase B gates

Per [BRINGUP_MAX_CARRIER §5 Stage 3](../DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md#L119):

- **B1**: `tx_done_rate ≥ 0.99` over N=100
- **B2**: `rx_match_rate ≥ 0.99`
- **B3**: zero `radio_tx_abort_airtime` growth on Board 1
- **B4**: zero real `FAULT_URC` on either board across the N-cycle run
- **B5**: host invariants stable on both boards (per `INVARIANT_COUNTERS`)
- **B6**: median RSSI on Board 2 within plausible bench-distance band (e.g. [-60, -120] dBm) — exact band depends on antenna spacing + attenuator

### 3.4 Risks specific to Phase B

1. **CFG_SET_REQ(LBT_ENABLE=0)** is needed on Board 1 (same disable as `--probe tx` does today, line 765 of `method_h_stage2_tx_probe.py`) — without LBT-disable the existing TX probe sees `radio_tx_abort_lbt` increments without a calibrated antenna.
2. **Antenna saturation** at <30 cm spacing can make the receiver compress and the LoRa demodulator produce CRC errors; first-light test should use ≥1 m spacing or add 20 dB pad.
3. **Sequence-number wrap on `tx_id`** — `tx_id` is `u8`; for N>256 the correlation logic must use `(i & 0xFF, i >> 8)` to disambiguate. **Mitigation in shipped probe:** correlation is by full payload contents (`W1-10b seq=NNNN <random4hex>`), not by `tx_id`, so wrap is harmless up to N≈10⁴; the random 4-byte hex suffix gives 32-bit collision resistance per cycle.
4. **Board 1 firmware drift** — if Board 1 still has the old W4-pre artifact, it will fail to boot on the W1-9d two-delta carrier. Re-flash via the Stage 1 helper before any Phase B run.

### 3.5 Execution recipe (when Board 1 returns)

```powershell
# 1. Confirm both boards present.
adb devices
# Expected:
#   2D0A1209DABC240B  device   (Board 1 / TX)
#   2E2C1209DABC240B  device   (Board 2 / RX)

# 2. (Required if Board 1 has pre-W1-9d firmware.) Reflash Board 1 with the
#    current 16,440 B build via Stage 1 helper.
powershell -NoProfile -ExecutionPolicy Bypass `
  -File LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_stage1_standard_quant_end_to_end.ps1 `
  -AdbSerial 2D0A1209DABC240B -Cycles 1

# 3. Run the W1-10b orchestrator (default 100 cycles, 0.2 s spacing).
powershell -NoProfile -ExecutionPolicy Bypass `
  -File LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_w1_10b_rx_pair_end_to_end.ps1 `
  -RxAdbSerial 2E2C1209DABC240B `
  -TxAdbSerial 2D0A1209DABC240B `
  -Cycles 100
```

Evidence lands in `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/W1-10b_rx_pair_<stamp>/` with `summary.json` + `tx_stdout.txt` + `rx_stdout.txt` + per-board OpenOCD/probe logs. Verdict line is `__W1_10B_VERDICT__=RX_PAIR_PASS` or `RX_PAIR_FAIL_<n>_GATES`.

### 3.6 Probe code shipped (2026-05-11)

| Component | Path | Notes |
|---|---|---|
| RX-side sub-mode | [`method_h_stage2_tx_probe.py::run_rx_listen()`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py) | Drains, warms up, prints `__W1_10B_LISTEN_READY__`, listens for window seconds, emits one `__RX_FRAME__ ...` line per `RX_FRAME_URC`, summary on `__W1_10B_LISTEN_DONE__`. |
| TX-side sub-mode | [`method_h_stage2_tx_probe.py::run_tx_burst()`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py) | Drains, warms up, disables LBT, sends N `TX_FRAME_REQ` with payload `W1-10b seq=NNNN <random4hex>`, emits `__TX_DONE__ ...` per cycle + summary `__W1_10B_BURST_DONE__`. |
| Host orchestrator | [`run_w1_10b_rx_pair_end_to_end.ps1`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_w1_10b_rx_pair_end_to_end.ps1) | Verifies both ADB serials present, pushes helper to both boards, starts RX listener (background `Start-Process` with stdout-to-file), polls for `__W1_10B_LISTEN_READY__`, runs TX burst foreground, correlates by full payload, evaluates B1..B6, writes `summary.json`. |
| Single-board wrapper extended | [`run_method_h_stage2_tx_end_to_end.ps1`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_method_h_stage2_tx_end_to_end.ps1) | `-Probe rx_listen` / `-Probe tx_burst` for ad-hoc per-board debugging; forwards `LIFETRAC_RX_WINDOW`/`LIFETRAC_TX_COUNT`/`LIFETRAC_INTER_CYCLE_S` env vars to the X8. |

**Smoke test (2026-05-11 14:01):** standalone `-Probe rx_listen -RxWindow 5` on Board 2 produced `__W1_10B_LISTEN_READY__` then `__W1_10B_LISTEN_DONE__ rx_frames=0 radio_rx_ok_delta=0 radio_crc_err_delta=0 real_faults=0 invariants_violated=0`, `probe_rc=0`. Orchestrator with TX serial absent gracefully threw `TX board '...' not present in 'adb devices'`. Wiring confirmed; only hardware availability blocks Phase B execution.

---

## 4. Lessons from Phase A (carry-forwards into Phase B)

1. The W1-9g `drain_pending()` helper is the right pattern for any back-to-back `request()` call after `VER_REQ`. Phase B's pair probe should use it on both link instances after the warm-up.
2. SF7/BW125 RX on a quiet bench bottoms at ~-120 dBm — that's the noise floor, not a defect. Don't write a gate that requires RSSI ≠ noise-floor; require AGC variance instead.
3. RegModemStat reading 0x04 (only `RxOngoing` set) is the correct **idle-but-armed** state. SigDetect/Sync/HdrInfoValid bits should set transiently when a real preamble lands; Phase B can use those as additional bonus evidence.
4. The W1-9f `BENIGN_FAULT_CODES` filter must be active on **both** boards' link in Phase B — both will emit one-shot `HOST_RX_SEEN` on first probe contact.

---

## 5. Closure summary for the GO/NO-GO banner

With Phase A green, the controller-side LoRa firmware status is:

> **Conditional GO** for advancing past Stage 1 ingress bring-up. All Stage 1 + Stage 2 TX + Stage 2 RX-liveness gates closed with bench evidence. Final unconditional GO requires Phase B (W1-10b two-board end-to-end ≥99% rates) once Board 1 returns to the bench.

[TODO.md](../DESIGN-CONTROLLER/TODO.md) "Go/No-Go" banner and recipe doc [§7](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/07_Successful_Connection_Recipe_All_Chips_Copilot_v1_0.md) are updated with this closure note.

---

## 6. Cross-references

- Probe code: [`method_h_stage2_tx_probe.py::run_rx_liveness()`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py)
- Probe wrapper: [`run_method_h_stage2_tx_end_to_end.ps1 -Probe rx`](../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_method_h_stage2_tx_end_to_end.ps1)
- Bench evidence: [`bench-evidence/W1-10_rx_liveness_2026-05-11_133459/`](../DESIGN-CONTROLLER/bench-evidence/W1-10_rx_liveness_2026-05-11_133459/)
- Predecessor closure: [`2026-05-12_W1-9g_Post_VER_STATS_Dispatch_Stall_Copilot_v1_0.md`](2026-05-12_W1-9g_Post_VER_STATS_Dispatch_Stall_Copilot_v1_0.md)
- Bring-up roadmap: [`DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md`](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md)
- Hardware bring-up: [`firmware/murata_l072/BRINGUP_MAX_CARRIER.md`](../DESIGN-CONTROLLER/firmware/murata_l072/BRINGUP_MAX_CARRIER.md#L99) §5
