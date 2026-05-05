# LoRa Firmware — Next Tranche Implementation Plan
### DMA half/full RX path · Full host protocol set · SX1276 transaction layer

**Author:** GitHub Copilot (Claude Opus 4.7)
**Version:** v1.0
**Date:** 2026-05-05
**Scope:** Plan only. No code in this document. Targets the firmware skeleton at [../../DESIGN-CONTROLLER/firmware/murata_l072/](../DESIGN-CONTROLLER/firmware/murata_l072/) following the Phase 1 hello-world increment that already landed (HSI16 platform, USART2 DMA-on-IDLE host transport, SX1276 register driver + DIO IRQ capture).
**Authoritative references:**
- [02 Firmware Architecture Plan](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md)
- [03 Bring-up Roadmap](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md) (W2-4 host command set, W2-* radio TX/RX)
- [04 Hardware Interface and Recovery](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) §2 (host protocol), §6 (CFG keys)
- [05 Method G Review Findings](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md) §1.5 / §5 / §6 / §7 (converged decisions)

---

## 0. Executive summary

Three work items, sequenced to land in the order **B → A → C** below, because:

- **B (Host protocol command set)** unblocks the H7 reference driver and HIL fixtures. It can land entirely without touching the radio path. **Land first.**
- **A (DMA half/full-transfer handling)** is a localized rework of `host_uart.c` that becomes valuable only when the protocol set is complete enough to actually saturate the link. **Land second**, after B exposes traffic that benefits from it.
- **C (SX1276 transaction layer)** is the largest piece of net-new code, has the most stable spec dependencies, and is the entry point to all Phase 2/3/4 radio features. **Land third** so the host transport is rock-solid before the radio starts emitting URCs at sustained rates.

Each work item below specifies: scope, file changes, design constraints, sequencing, risks, and acceptance gates that must hold before merge.

---

## 1. Work item A — DMA half-transfer / full-transfer host RX path

### 1.1 Why now (what the current path lacks)

The current [host_uart.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c) services the 512-byte circular DMA buffer **only on USART IDLE**. That is sufficient for sparse traffic but has two failure modes once `TX_FRAME` traffic ramps to 20 Hz with payloads near `HOST_INNER_MAX_LEN = 320` (encoded ≈ 325 + 2 framing = 327 B):

1. **Latency tail.** A continuous stream of frames with no inter-frame gap leaves IDLE deasserted; bytes only get drained on the IRQ tail (~8.7 µs at 921600 baud) at the *end* of the burst. For 50 ms slot scheduling that means up to 327 bytes (~3.0 ms) of buffered data sitting unprocessed inside an in-progress slot.
2. **Buffer wrap risk.** At 921600 baud and 512 B buffer, wrap time = 5.55 ms. A worst-case bursty host (Linux side scheduling jitter on `/dev/ttymxc3`) plus a busy main loop (radio IRQ servicing) can plausibly overrun half the buffer before `host_uart_poll_dma()` runs again. Today's code papers over this with `service_in_progress` reentrancy guard — that masks the symptom, not the cause.

### 1.2 Design

Replace "service on IDLE only" with "service on **HT | TC | IDLE**, whichever fires first":

| Trigger | Source | Action |
|---|---|---|
| `HTIF` (Half-Transfer) | `DMA1_Channel4_5_6_7_IRQHandler` | Drain bytes `[0 .. BUF_SIZE/2)` |
| `TCIF` (Transfer-Complete, circular wrap) | same | Drain bytes `[BUF_SIZE/2 .. BUF_SIZE)`, DMA continues at index 0 |
| `IDLE` (USART2_IDLE) | `USART2_IRQHandler` | Drain bytes `[s_dma_last_idx .. CNDTR_now)` — covers the partial-burst tail |
| `TEIF` (Transfer Error) | DMA channel | Restart DMA, bump `s_stats_errors` (already implemented) |

Worst-case service latency drops from "IDLE deassert + IRQ entry" to "256 bytes at 921600 baud = 2.78 ms" guaranteed — and IDLE still covers the small-burst case.

### 1.3 File touches

| Path | Change |
|---|---|
| [host/host_uart.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c) | Add `DMA_CCR_HTIE \| DMA_CCR_TCIE` to `dma_rx_start()`; split `service_dma_rx()` into `drain_range(begin, end)`; rewrite `DMA1_Channel4_5_6_7_IRQHandler` to dispatch HT/TC/TE clearing the right `CHTIF/CTCIF/CTEIF` bit; keep IDLE path unchanged in `USART2_IRQHandler` |
| [include/stm32l072_regs.h](../DESIGN-CONTROLLER/firmware/murata_l072/include/stm32l072_regs.h) | Add `DMA_CCR_HTIE`, `DMA_CCR_TCIE`, `DMA_ISR_HTIF()`, `DMA_ISR_TCIF()`, `DMA_IFCR_CHTIF()`, `DMA_IFCR_CTCIF()` macros if not already present |
| [include/host_uart.h](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_uart.h) | New stat accessors: `host_uart_stats_ht_irq()`, `host_uart_stats_tc_irq()`, `host_uart_stats_idle_irq()` for visibility in `STATS_URC` |
| [host/host_uart.c](../DESIGN-CONTROLLER/firmware/murata_l072/host/host_uart.c) | Add three counters incremented in their respective IRQ branches |

### 1.4 Constraints (must hold)

- **No new heap.** All buffers stay statically sized.
- **Reentrancy still impossible.** All three drain paths run inside an IRQ handler (HT/TC) or inside a path that briefly disables interrupts (the existing IDLE handler runs at IRQ priority 1; HT/TC run at priority 2). The `s_service_in_progress` guard stays — it now also protects against an HT and IDLE racing.
- **Copy minimization.** `drain_range()` calls `ingest_rx_byte()` directly; do not introduce an intermediate copy.
- **CRC failure semantics unchanged.** A frame split across the half boundary still resolves correctly because COBS state lives in `s_cobs_encoded[]` and survives across drain calls.
- **Static_assert that `HOST_DMA_RX_BUF_SIZE` is even.** Half-transfer math requires it.

### 1.5 Acceptance gates (HIL, no new test infra needed)

1. Loopback test: H7 sends 1000 frames of 320 B inner length back-to-back at 921600 8N1. L072 must echo all 1000 with zero `s_stats_dropped` and zero `s_stats_errors`.
2. Latency test: H7 timestamps a `PING` then waits for the URC. With HT/TC enabled the p99 round-trip must drop ≥30 % vs the IDLE-only baseline measured before the change.
3. Stats sanity: after a 10 s 20 Hz `PING` storm, `host_uart_stats_ht_irq() + host_uart_stats_tc_irq()` must be ≥ `host_uart_stats_idle_irq()` (i.e. the new triggers actually fire and are doing the work).

### 1.6 Risks

- **HT firing inside IDLE servicing.** Mitigated by `s_service_in_progress` (already in place); HT IRQ early-exits, and the IDLE drain naturally picks up the bytes the HT IRQ would have drained.
- **`DMA1_Channel4_5_6_7_IRQHandler` is shared.** USART2 RX uses channel 6; channels 4/5/7 are unused today. Verify the handler's `pending` mask is gated to `HOST_DMA_RX_CH` only — matters once SX1276 SPI gets DMA in work item C.

---

## 2. Work item B — Full host protocol command set & response taxonomy

### 2.1 What's implemented today vs the spec

| `type` | Name | Status |
|---|---|---|
| `0x00` | `PING` | ✓ implemented (echo) |
| `0x01` | `VER_REQ` / `0x81 VER_URC` | ✓ implemented (literal string; **lacks** capability bitmap, git SHA, protocol version) |
| `0x30` | `REG_READ` / `0xB0 REG_DATA_URC` | ✓ implemented (single register) |
| `0x31` | `REG_WRITE` | ✓ implemented; **lacks** allowlist gate |
| `0xF0` | `BOOT_URC` | ✓ implemented (radio_ok + version byte; **lacks** reset-cause field per [05 §C](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md), nonce-epoch random per option (c)) |
| `0xF1` | `RADIO_IRQ_URC` | ✓ implemented (raw DIO event mask) — **note:** [04 §2.3](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) reserves `0xF1` for `FAULT_URC`. **Conflict — must resolve.** |
| `0xFE` | `ERR_PROTO` | ✓ implemented |
| All other types in [04 §2.3](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) | not yet present |

### 2.2 Type-code reconciliation (must do first)

The current `main.c` uses `0xF1` for `RADIO_IRQ_URC`, but [04 §2.3](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) reserves `0xF1` for `FAULT_URC`. **Decision required before any further URCs land:** pick a free L→H code for `RADIO_IRQ_URC`. Recommendation: `0xC0` (in the unused L→H "C-block" alongside `0xC1 STATS_URC`). Update `04 §2.3` table in the same commit that fixes the firmware.

### 2.3 Full taxonomy to land

Authoritative source: [04 §2.3](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md). Implementation order matches the dependencies — each row's "Depends on" column tells you what must already be merged.

| Tranche | `type` | Name | Direction | Depends on | Notes |
|---|---|---|---|---|---|
| **B1** | `0x02` | `UID_REQ` | H→L | nothing | reads STM32 UID at `0x1FF80050` (3× u32) |
| B1 | `0x82` | `UID_URC` | L→H | B1 | 12-byte payload |
| B1 | `0x03` | `RESET` | H→L | nothing | calls `NVIC_SystemReset()`; emit no response (next `BOOT_URC` is the response) |
| B1 | `0x40` | `STATS_RESET` | H→L | nothing | zero all counters |
| B1 | `0x41` | `STATS_DUMP` | H→L | B1 | request → emits B2 URC |
| B1 | `0xC1` | `STATS_URC` | L→H | B1 | versioned struct: `{u32 host_dropped, u32 host_errors, u32 ht_irq, u32 tc_irq, u32 idle_irq, u32 radio_dio0..3, u32 radio_crc_err, u32 radio_rx_ok, u32 radio_tx_ok, u32 radio_tx_abort_lbt, ...}` — pad to fixed length so older H7 can index it |
| B1 | `0xFE` | `ERR_PROTO` | L→H | (already there) | extend payload to include length-mismatch sub-codes |
| **B2** | `0x20` | `CFG_SET` | H→L | persistent CFG region wiring (Phase 6 prep can stub: in-RAM only for now) | TLV payload: `{u8 key, u8 len, u8 val[len]}` |
| B2 | `0x21` | `CFG_GET` | H→L | B2 SET | new in [04 §6](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) future-tense ("A future `cfg get` command…") — propose adding to spec |
| B2 | `0xA0` | `CFG_OK_URC` | L→H | B2 | `{u8 key, u8 status}` |
| B2 | `0xA1` | `CFG_DATA_URC` | L→H | B2 GET | TLV |
| **B3** | `0x10` | `TX_FRAME` | H→L | work item C TX state machine | inner header: `{u8 channel_hint, u8 sf_hint, u8 cr_hint, u8 priority, u16 deadline_ms, u8 flags, u8 reserved}` + on-air bytes |
| B3 | `0x90` | `TX_DONE_URC` | L→H | C | `{u16 seq_echo, u32 t_air_us, u32 t_lbt_us, u8 channel_used, u8 sf_used, i8 power_dbm, u8 result_code}` |
| B3 | `0x91` | `RX_FRAME_URC` | L→H | C | `{u32 air_timestamp_us, i16 rssi_x16, i8 snr_x4, u16 freq_err_hz, u8 channel, u8 sf, u8 cr, u8 crc_ok, u16 len, u8 bytes[len]}` |
| **B4** | `0xF1` | `FAULT_URC` | L→H | **after the F1↔C0 swap in §2.2** | `{u8 fault_code, u8 subsystem, u32 detail}`; subsystems: 0=host, 1=radio, 2=clock, 3=watchdog, 4=memory |
| B4 | `0xF0` | `BOOT_URC` (v2) | L→H | nothing | extend existing payload to: `{u8 reset_cause, u8 radio_ok, u8 radio_version, u8 safe_mode_considered, u8 golden_jump_considered, u8 random[5]}` per [05 §C](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md) options (c) + reset-cause |

### 2.4 Architectural moves (do these alongside B1)

The current `handle_host_frame()` is a 70-line `switch` in `main.c`. Once B1 lands it will be 200+ lines. Move to the layout specified by [02 §2](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md):

| New file | Responsibility |
|---|---|
| `host/host_cmd.c` / `include/host_cmd.h` | `host_cmd_dispatch(const host_frame_t*)` plus one `static` handler per type; takes the switch out of `main.c` |
| `host/host_stats.c` / `include/host_stats.h` | All counters, `host_stats_reset()`, `host_stats_dump(uint8_t* out)` returning packed wire format |
| `host/host_cfg.c` / `include/host_cfg.h` | TLV codec, in-RAM key/value store + `cfg_apply(key, value, len)` callbacks per key (e.g. `tx_power_dbm` calls into radio); persistence is **deferred** — Phase 6 hooks the CFG Flash region |
| `version.h` | `OSE_LIFETRACLORA_FW_NAME`, `FW_VERSION_MAJOR/MINOR/PATCH`, `GIT_SHA[]`, `HOST_PROTO_VERSION`, `CAPABILITY_BITMAP` — generated by Makefile from `git describe` (existing pattern) |

Update `main.c` to a one-liner: `host_cmd_dispatch(&frame);` inside the existing pop loop.

### 2.5 Wire-format contract tests (must land same PR)

Per [05 §F](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md) — every URC payload gets a host-side parser test in `bench/host_proto/` so the H7 driver can never silently disagree with the L072 byte order. Minimum tests:

- COBS encode/decode round-trip on golden vectors (already needed; not yet present).
- CRC16 round-trip vs a Python reference.
- `STATS_URC` field-offset table — locked, additive-only.
- `RX_FRAME_URC` field-offset table — locked, additive-only.
- `BOOT_URC` reset-cause decode for each reset source the L072 RCC reports.

### 2.6 Acceptance gates

1. Every type-code in [04 §2.3](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) (with the F1/C0 reconciliation) is either implemented in firmware or has a code comment `// TODO(W3-x): implement` and an entry in the unimplemented-list section of the doc — no silent gaps.
2. `VER_URC` returns the capability bitmap; H7 driver can refuse to operate against an unknown bitmap.
3. `CFG_SET tx_power_dbm` round-trips through `CFG_GET` and is observable in `STATS_URC` (or a register read of `RegPaConfig`).
4. `RESET` + `BOOT_URC` round-trip succeeds in <1 s per [04 §5 H5](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md).
5. Send a malformed frame (bad CRC, bad len, unknown type, unknown ver) — each yields an `ERR_PROTO` with the right sub-code, never a hang or silent drop.

### 2.7 Risks

- **Type-code conflict on `0xF1`.** Already called out above; resolve in the spec doc first.
- **`STATS_URC` payload layout drift.** Lock the struct in `host_stats.h` with explicit field offsets and a `_Static_assert(sizeof(stats_wire_t) == EXPECTED)`. Reviewer must veto any non-additive change.
- **`REG_WRITE` allowlist** — production builds must reject writes to dangerous registers (FIFO ptrs, OpMode bits that bypass safe modes). Build-flag-gated, not runtime-toggled.

---

## 3. Work item C — SX1276 transaction layer (RX/TX state machine, DIO mapping, CAD/LBT hooks)

### 3.1 What's there today vs what we need

The current [sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) is a **register driver**: SPI transactions, single-register PHY profile programming, raw DIO IRQ capture into a u32 event word. There is no TX path, no RX path, no FIFO management, no mode-aware DIO mapping, and no CAD/LBT primitive. All the higher-level features in [01](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md) (R-01 FHSS, N-04 LBT, N-06 quality FHSS, N-09 deep-sleep RX, N-30 beacon) sit on top of a transaction layer that does not yet exist.

### 3.2 Design — the transaction layer in three sub-modules

Per [02 §2](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md):

| New file | Responsibility |
|---|---|
| `radio/sx1276_modes.c` / `include/sx1276_modes.h` | OpMode transitions + DIO mapping (`RegDioMapping1/2`) per current mode |
| `radio/sx1276_fsm.c` / `include/sx1276_fsm.h` | RX/TX state machine consuming DIO IRQ events; exposes `radio_tx_start()`, `radio_rx_start()`, `radio_cad_start()`, `radio_get_state()` |
| `radio/sx1276_lbt.c` / `include/sx1276_lbt.h` | RSSI-snapshot LBT (N-04) and CAD-based LBT scheduling hooks |

`sx1276.c` becomes the **HAL layer** under these modules — it keeps `sx1276_read_reg`/`write_reg`/`read_burst`/`write_burst` and the GPIO/SPI/EXTI init. Higher-level callers do **not** touch registers directly anymore.

### 3.3 OpMode + DIO mapping table (data, not code)

This table is the contract between the FSM and the HAL. Per SX1276 datasheet §4.1.6 (OpMode) and §6.4 (LoRa DIO mapping):

| Mode (`RegOpMode[2:0]`) | DIO0 | DIO1 | DIO2 | DIO3 | Used by |
|---|---|---|---|---|---|
| `SLEEP (000)` | — | — | — | — | deep-sleep idle |
| `STANDBY (001)` | — | — | — | — | between TX/RX, register fiddling |
| `TX (011)` | `TxDone` | — | — | — | TX path |
| `RXCONT (101)` | `RxDone` | `RxTimeout` | `FhssChange` | `CadDone`/`ValidHeader` | continuous RX (control plane) |
| `RXSINGLE (110)` | `RxDone` | `RxTimeout` | `FhssChange` | `ValidHeader` | duty-cycled RX (N-09) |
| `CAD (111)` | `CadDone` | `CadDetected` | `FhssChange` | — | LBT pre-TX, idle scan |

`sx1276_modes.c` programs `RegDioMapping1/2` **as part of** the mode-set call. Calling code never has to know the mapping.

### 3.4 RX state machine (SF-by-SF + FHSS-friendly)

States: `IDLE → STANDBY → CAD_PROBE? → RX_ARMED → RX_HEADER → RX_DONE → STANDBY → IDLE`.
Inputs: caller `radio_rx_start(profile, rx_mode)`, DIO0 (`RxDone`), DIO1 (`RxTimeout`), DIO3 (`ValidHeader`).
Outputs: `RX_FRAME_URC` payload (channel, SF, RSSI, SNR, freq err, len, bytes), error counter on CRC fail.

Sub-rules:
- On `ValidHeader` DIO3: arm CRC validation; if CRC bit clear in `RegHopChannel`, count and discard at `RxDone`.
- On `RxDone` DIO0: read `RegRxNbBytes`, `RegFifoRxCurrentAddr`; burst-read FIFO; read RSSI/SNR/FreqErr; clear `RegIrqFlags`; emit URC.
- On `RxTimeout` DIO1: re-arm if profile says continuous, else exit to `STANDBY`.
- On caller-requested abort: write `STANDBY` to OpMode, drain DIO IRQ flags before returning.

### 3.5 TX state machine (LBT-aware)

States: `IDLE → STANDBY → CAD_LBT → BACKOFF? → TX_LOAD → TX_FIRE → TX_DONE → STANDBY → IDLE`.
Inputs: caller `radio_tx_submit(tx_request_t*)`, DIO0 (`TxDone`), DIO0/DIO1 (`CadDone`/`CadDetected`).
Outputs: `TX_DONE_URC` payload (`t_air_us`, `t_lbt_us`, channel/SF/power used, result code), counters.

Sub-rules:
- If `lbt_enable` (default true per [04 §6](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md)): enter `CAD_LBT`, wait for `CadDone` (DIO0). If `CadDetected` (DIO1) was also set → measure RSSI snapshot (already-implemented `sx1276_read_reg`); if above `lbt_threshold_dbm` → backoff per [01 §N-04](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md) (target ≤300 µs decision time).
- Backoff policy: deterministic short backoff (e.g. 250 µs × `(1 + retry)` capped at 4 retries) for P0 traffic; abort with `result_code=LBT_BUSY` and counter bump if all retries fail (do not delay past `deadline_ms`).
- TX_LOAD: write FIFO via `sx1276_write_burst` (already exists), set `RegPayloadLength`, `RegFifoTxBaseAddr`, OpMode→TX.
- On `TxDone`: read `t_air_us` from a SysTick capture latched in the TX_FIRE→TxDone transition; emit URC.

### 3.6 CAD primitive (foundation for both LBT and idle scan)

Single function `radio_cad_once(channel, profile) -> {clear|busy}` blocks ≤300 µs. Used by:
- `sx1276_lbt.c` pre-TX gate (work item C now).
- N-05 spectrum scanning (Phase 4+).
- N-19 RSSI watchdog (Phase 4+).

Keep CAD result accessible without forcing the FSM through `IDLE` — the FSM exposes `radio_peek_cad_last()` returning the last result + timestamp.

### 3.7 DIO IRQ → FSM event lifting

Current [sx1276.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) packs raw DIO toggles into `s_irq_events`. The FSM needs **mode-tagged** events. Approach:

1. Keep raw bit capture in EXTI handlers (no logic in IRQ context).
2. Add `sx1276_fsm_poll()` called from the main loop (alongside `host_uart_poll_dma()`):
   - Reads `s_irq_events` once via `sx1276_take_irq_events()`.
   - Reads `RegIrqFlags` over SPI.
   - Combines DIO-mode mapping (per §3.3 table for current OpMode) + IRQ-flag bits into typed FSM events.
   - Drives the appropriate state transition.
3. Do **not** do SPI in IRQ context — the M0+ takes too long for an SPI burst at IRQ priority. Lift to main-loop poll. (This is the convergent recommendation from [05 §6](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md) Copilot §7.4 — keep IRQ handlers nano-sized.)

### 3.8 Sequencing inside work item C

| Step | Deliverable | Gate |
|---|---|---|
| C1 | `sx1276_modes.c` with mode/DIO table; replace direct `OpMode` writes in `sx1276.c::sx1276_init` and `apply_profile_full` | `make`, regression test: `RADIO_IRQ_URC` still arrives |
| C2 | `sx1276_fsm.c` skeleton: `IDLE`/`STANDBY` states only, polled from main loop | radio still passes existing init self-test |
| C3 | RX path (continuous mode); add `RX_FRAME_URC` per §3.4 | HIL: two boards on bench, transmitter is a stub Python script driving an SDR or another board's `REG_WRITE`; receiver emits `RX_FRAME_URC` with correct byte count |
| C4 | TX path (no LBT); add `TX_FRAME` (work item B3) and `TX_DONE_URC` | HIL: H7 sends `TX_FRAME` with 16-byte payload; second board receives via C3 path; round-trip OK |
| C5 | CAD primitive + LBT gate on TX path; expose `lbt_enable` CFG | LBT-aborted-TX counter increments when bench radio is busy; clear-channel TX completes |
| C6 | `radio_get_state()` and FSM-state sample in `STATS_URC` | observability for next phase |

### 3.9 Constraints (must hold)

- **No SPI in IRQ context.** All radio register access stays in main-loop poll.
- **Single owner of OpMode.** Only `sx1276_modes.c` writes `RegOpMode`. `sx1276_fsm.c` calls into `sx1276_modes.c`. No other file may touch `RegOpMode`.
- **No allocation.** TX request payload sits in a single `static` buffer of `HOST_PAYLOAD_MAX_LEN` bytes; one TX in flight at a time per [05 §B](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md).
- **Timing budget.** LBT gate target ≤300 µs per [01 §N-04](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md). Measure in C5 via SysTick deltas; record p99 in `STATS_URC`.
- **Don't anticipate FHSS.** Per-frame FHSS (R-01) lands in Phase 3 W3-1; the TX state machine accepts a `channel_hint` but the FSM treats it as "set frequency once before TX_LOAD" — no per-symbol hop logic yet.
- **Test in isolation first.** Before HIL, exercise the FSM with a mock radio (compiled with `-DRADIO_FAKE=1`) that returns scripted `RegIrqFlags` values. Lives in `bench/radio_fsm/` — uses host `cc` only, runs in `make check`.

### 3.10 Acceptance gates

1. Two-board HIL: `TX_FRAME` → over-air → `RX_FRAME_URC` round-trip succeeds at SF7/BW250/CR4-5 with payload integrity verified by CRC16 over the emitted bytes.
2. LBT busy-channel test: a third board transmits continuously on the LBT channel; the device-under-test's `TX_FRAME` returns `LBT_BUSY` and the counter bumps; when the third board stops, the next `TX_FRAME` succeeds.
3. RX timeout: `radio_rx_start` with a finite timeout returns to `STANDBY`, no URC emitted, no IRQ leak (verified by `RegIrqFlags == 0` after the timeout).
4. State observability: `STATS_URC` shows `radio_state`, `radio_dio0..3` event counts, `radio_crc_err`, `radio_rx_ok`, `radio_tx_ok`, `radio_tx_abort_lbt`.
5. No SPI in IRQ: instrument with a "longest IRQ-handler runtime" counter; cap < 20 µs.

### 3.11 Risks

- **CAD vs LBT semantics.** SX1276 `CAD` detects LoRa preambles, **not** arbitrary energy. For non-LoRa interferers we need an RSSI snapshot fallback. Plan: try CAD first (~3 symbols ≈ 1 ms at SF7/BW250 — too slow for the 300 µs LBT budget); fall back to RSSI register read (×4 samples ≈ 250 µs) for the launch path. CAD becomes a Phase 4 enhancement when N-06 quality-aware FHSS lands. **Decision needed before C5** — recommend RSSI-snapshot LBT for launch, CAD for idle-scan only.
- **DIO3 routing unconfirmed.** [02 §8 Q4](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) flags DIO3 as TBD on the Max Carrier. The FSM design uses DIO3 for `ValidHeader` — if DIO3 is not routed, fall back to polling `RegIrqFlags::ValidHeader` from `sx1276_fsm_poll()`. Either path works; the FSM API is unchanged.
- **Shared DMA IRQ vector.** Once SX1276 SPI gets DMA (planned for after C6 if bursts get long), it will share `DMA1_Channel4_5_6_7_IRQHandler` with the host UART RX. Already a constraint on work item A's IRQ refactor — call out the interaction in code comments.

---

## 4. Combined sequencing & branch strategy

| Order | Work item | Branch / PR | Why this order |
|---|---|---|---|
| 1 | **B1** (UID, RESET, STATS, ERR_PROTO sub-codes) + 2.4 file refactor + 2.2 type-code reconciliation in spec | `feat/host-proto-b1` | Lowest risk; unblocks H7 driver work; enables observability |
| 2 | **B2** (CFG_SET/GET, in-RAM only) | `feat/host-proto-b2-cfg` | Required by C5 for `lbt_enable` toggle |
| 3 | **A** (DMA HT/TC) | `perf/host-uart-dma-ht-tc` | Localized; benefits B1/B2 traffic immediately; benefits C onward |
| 4 | **C1–C2** (modes + FSM skeleton) | `feat/radio-fsm-skeleton` | Foundation for radio work |
| 5 | **C3** (RX path) + **B3** (`RX_FRAME_URC`) | `feat/radio-rx` | RX simpler than TX; unblocks listening tests |
| 6 | **C4** (TX path) + **B3** (`TX_FRAME`/`TX_DONE_URC`) | `feat/radio-tx` | First over-air |
| 7 | **C5** (LBT gate) | `feat/radio-lbt` | Prerequisite for Phase 4 launch features |
| 8 | **C6** (state observability) + **B4** (`FAULT_URC`, extended `BOOT_URC`) | `feat/radio-fsm-observability` | Closes Phase 2 |

Each PR carries its host-side wire-format test vectors. No PR may regress an earlier PR's acceptance gates.

---

## 5. Companion Analysis (Gemini 3.1 Pro Preview, 2026-05-05)

**Reviewer:** GitHub Copilot (Gemini 3.1 Pro)
**Purpose:** Technical validation of the next-tranche sequencing, DMA mechanics, and radio state-machine constraints.

### 5.1 DMA and IRQ Concurrency (Work Item A)
- **Feasibility:** At 921600 baud, a 512-byte buffer fills every ~5.55 ms. The Half-Transfer (HT) interrupt will fire every ~2.78 ms for continuous traffic. The STM32L072 (Cortex-M0+ @ 32MHz) can comfortably service this, provided the `drain_range()` function is strictly O(N) and does zero blocking processing.
- **Risk - Shared Vector Trap:** STM32L0x2 line devices merge DMA channels 4, 5, 6, and 7 into a single IRQ handler (`DMA1_Channel4_5_6_7_IRQHandler`). When Work Item C adds SPI DMA on these channels, the ISR must definitively check `DMA_ISR` flags *before* blindly clearing them with `DMA_IFCR`, otherwise a UART decode could accidentally clear and drop an SPI RX-complete signal. Your callout in §1.6 and §3.11 is critical.

### 5.2 Command Set & Sequencing (Work Item B)
- **F1 vs C0 Conflict:** Moving `RADIO_IRQ_URC` to `0xC0` to preserve `0xF1` for `FAULT_URC` is the correct immediate fix. Grouping telemetry/diagnostics in the `0xC*` block is semantically cleaner.
- **B1 → A → C Ordering:** Excellent strategic sequencing. Validating the COBS frame handling at high speed (B1 + A) *before* introducing SX1276 interrupts (C) guarantees that any protocol fragmentation observed in Phase 3 is definitely an RF/State issue, not a UART bottleneck.

### 5.3 SX1276 State Machine & LBT (Work Item C)
- **LBT Implementation Mechanism (§3.11):** You correctly identified the timing mismatch for CAD. A standard LoRa CAD at SF7 takes roughly ~1.5 to 2 symbols, which exceeds your strict 300 µs LBT gate. **Recommendation:** Hard-commit to the RSSI-snapshot method (FSK mode RSSI or continuous LoRa RSSI read) for the launch's LBT implementation, and relegate CAD exclusively for scheduled deep-sleep RX wakes (N-09). 
- **Mode-Tagged Event Lifting (§3.7):** Moving from in-IRQ SPI calls to a main-loop `sx1276_fsm_poll` is the only way to prevent dropping host-UART data. SPI transactions take real time; doing them in a high-priority EXTI line will stall the MCU and guarantee DMA buffer overwrites. This polling design correctly protects the architecture's deterministic latency.

## 6. Out of scope (do NOT do as part of this tranche)

These are tempting but belong to later phases and would balloon the PRs:

- **A/B firmware slots / N-26.** Phase 6. Do not touch the linker script regions.
- **Crypto in L072 / N-12.** Defer per [05 §A1 Profile A](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md). H7 owns AES-GCM at launch.
- **Per-frame FHSS / R-01.** Phase 3 W3-1. The TX state machine accepts a `channel_hint` — it does not implement hopping logic.
- **Quality-aware FHSS / N-06.** Phase 4+ per converged decision in [05 §5.4](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/05_Method_G_Review_Findings_and_Code_Suggestions_GPT-5.3-Codex_v1_0.md).
- **Deep sleep / N-09.** Phase 5. The mode table includes `SLEEP` for completeness but the FSM does not enter it autonomously.
- **PCAP / MFGTEST commands** from [02 §2 host_cmd.c comment](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md). Useful, but not on the launch critical path.
- **Persistent CFG in Flash.** Phase 6 lands the CFG region writer. Until then, `CFG_SET` is RAM-only and `RESET` clears it (documented behavior, not a bug).

---

## 7. One-line summary

> **Land the host protocol set first (refactor `main.c` switch into `host/host_cmd.c`, fix the 0xF1 conflict, add UID/RESET/STATS/CFG/FAULT/extended-BOOT URCs); then enable DMA HT/TC IRQs to drop host-RX latency under sustained traffic; then build the radio transaction layer as `sx1276_modes.c` (OpMode + DIO mapping) → `sx1276_fsm.c` (polled RX/TX state machine, no SPI in IRQ context) → `sx1276_lbt.c` (RSSI-snapshot LBT for launch, CAD reserved for idle-scan), with a wire-format-locked `RX_FRAME_URC`/`TX_DONE_URC`/`STATS_URC` and contract tests committed alongside the firmware.**

---

## 8. Companion Analysis (GitHub Copilot v1.0, 2026-05-05)

**Reviewer:** GitHub Copilot
**Purpose:** Validate the next-tranche order against the current `murata_l072` skeleton and call out implementation traps that should be resolved before code lands.

### 8.1 Overall Verdict

The tranche is pointed in the right direction. The highest-leverage next move is still to stabilize the host protocol and observability before the radio FSM starts producing high-rate URCs. The B -> A -> C shape is sound, but I would tighten the first half to:

1. **B0/B1 first:** central type-code definitions, `main.c` dispatch refactor, UID/RESET/STATS basics, version identity.
2. **A second:** DMA HT/TC service and RX race hardening, before adding more command surface.
3. **B2 third:** CFG_SET/CFG_GET once the transport is less fragile under burst traffic.
4. **C after that:** radio modes/FSM/LBT, with host wire tests already in place.

This changes the plan only slightly: move A ahead of B2. B2 is useful, but it is not needed to validate the core host protocol shape, and the current DMA path is a more foundational reliability risk.

### 8.2 Required B0 Corrections Before B1

The plan correctly catches the `0xF1` collision between current `RADIO_IRQ_URC` and spec `FAULT_URC`. There is a second live collision in the skeleton: current `main.c` uses `0xA0` as `REG_WRITE_RSP`, while [04 §2.3](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) reserves `0xA0` for `CFG_OK_URC`.

Add a tiny B0 step before B1:

- Create `include/host_types.h` as the single source for command/URC IDs.
- Move `RADIO_IRQ_URC` to `0xC0` and document it in [04](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md).
- Remove or move `REG_WRITE_RSP`; recommendation: use `REG_DATA_URC` only for register read data, and use a generic ACK/ERR path for diagnostic writes so `0xA0` remains `CFG_OK_URC`.
- Make the compiler fail if two host type constants share the same value. A simple generated table or static host-side test is enough.

B0 should also replace the current literal version string (`murata_l072 baremetal v0.1.0`) with `version.h` fields for `OSE-LifeTracLORA-MurataFW`, `v0.0.0`, git SHA, host protocol version, and capability bitmap.

### 8.3 Work Item A - DMA HT/TC Needs Stronger Concurrency Rules

The HT/TC direction is correct, and the needed DMA flag macros already exist in `include/stm32l072_regs.h`. The risky part is not the registers; it is ownership of parser and queue state between IRQ and main-loop paths.

Required amendments before implementing A:

- Make `s_service_in_progress` an atomic test-and-set guarded by `cpu_irq_save()`/`cpu_irq_restore()`, or ensure DMA draining occurs in only one context. The current byte flag is not enough if main-loop polling and HT/TC/IDLE IRQs can enter the same function.
- Protect `queue_push()` with the same interrupt discipline as `host_uart_pop_frame()`. Today `queue_push()` can run from `host_uart_poll_dma()` in main context or from IRQ context; once HT/TC lands, that race becomes easier to hit.
- Implement HT/TC/IDLE as `drain_to(write_idx)`, not fixed `drain_range(0, half)` / `drain_range(half, end)` calls. IDLE may already have drained part of a half-buffer, so fixed ranges can duplicate bytes unless every path respects `s_dma_last_idx`.
- Add counters for `ht_irq`, `tc_irq`, `idle_irq`, `rx_overrun`, and `queue_full`. STATS needs to distinguish malformed input from parser backpressure.

Revise A acceptance gate 1. With today’s blocking `host_uart_send_urc()`, an unpaced 1000-frame echo storm can fail for the wrong reason: the L072 is busy transmitting replies while DMA IRQs continue to enqueue new frames into a 4-deep queue. Use one of these instead:

- paced request/response loop: host sends next PING only after receiving the previous echo;
- RX-only stress command that does not emit one URC per input frame;
- or implement async host TX/credits before requiring unpaced full-duplex echo.

Without that adjustment, the test measures blocking TX and queue depth more than HT/TC RX correctness.

### 8.4 Work Item B - Protocol Payloads Need Binary Layouts Now

B1/B2 should not leave URC payloads as ad hoc strings. The H7 driver needs fixed binary layouts from the first implementation commit.

Recommendations:

- `VER_URC`: use `{u8 host_proto_ver, u8 fw_major, u8 fw_minor, u8 fw_patch, u32 capability_bitmap, u8 git_sha_len, bytes...}` plus optional product string, or define a TLV layout. Do not rely on parsing human-readable text.
- `ERR_PROTO`: include `{u8 offending_type, u8 offending_ver, u8 code, u16 detail}`. Codes should separate bad COBS, CRC mismatch, length mismatch, too large, unsupported version, unknown type, and queue full.
- `STATS_RESET`: clear counters under interrupt lock. Otherwise HT/TC IRQs can update while the reset is in progress.
- `CFG_SET host_baud`: RAM-only is fine for now, but safe-mode must continue to ignore runtime/persisted baud settings.
- `REG_WRITE`: gate behind a build flag and an allowlist immediately. Once `sx1276_modes.c` owns `RegOpMode`, diagnostic writes must not be able to bypass it.

The plan’s wire-format contract tests are essential. Put them in `bench/host_proto/` before B1 is considered done, even if they initially compile only on the host PC.

### 8.5 Work Item C - Radio FSM Corrections

The module split (`sx1276_modes`, `sx1276_fsm`, `sx1276_lbt`) is the right shape. A few details should be resolved before C1/C3:

- Resolve the CAD/LBT contradiction before coding. The state name should be `LBT_PROBE`, not `CAD_LBT`, if launch uses RSSI snapshots. CAD can still exist for idle scan or later deep-sleep wake work, but it should not sit on the launch TX critical path with a 300 us budget.
- RX-first sequencing is good only if the transmitter is known-good. The C3 gate should use an external known-good LoRa transmitter, SDR, or a second board that already has the TX FSM. A second skeleton board driven only by `REG_WRITE` is not a reliable transmitter fixture.
- Keep raw DIO capture in IRQ context, but make the FSM robust when DIO3 is absent. `ValidHeader` should be optional observability, not a hard dependency for RX success.
- Single-owner `RegOpMode` must be enforced in code review. After C1, only `sx1276_modes.c` may write `RegOpMode`; `REG_WRITE` allowlists and `sx1276_apply_profile_full()` must respect that boundary.
- Add FIFO/IRQ-flag order tests to C3/C4: RX should clear stale IRQ flags before arming, set FIFO base/current addresses deterministically, and verify `RegIrqFlags == 0` after timeout/abort.

### 8.6 Additional Acceptance Gates

Add these gates to the combined sequencing table:

1. **B0 gate:** `host_types.h` and [04 §2.3](../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) agree exactly; no duplicate type codes; `VER_URC` returns `OSE-LifeTracLORA-MurataFW v0.0.0` identity fields.
2. **A gate:** HT/TC/IDLE all drain through one `s_dma_last_idx` path; no duplicate-byte or skipped-byte cases when frame delimiters land at 255/256/257 and 511/512/0 boundaries.
3. **B1 gate:** malformed frames under load produce deterministic `ERR_PROTO` subcodes and do not increment only a generic error bucket.
4. **C1 gate:** existing register dump and raw DIO event capture still work after `sx1276_modes.c` takes over OpMode and DIO mapping.
5. **C3/C4 gate:** no SPI transaction occurs inside EXTI handlers; longest IRQ-handler runtime remains below the stated 20 us cap.

### 8.7 Out-of-Scope Clarification

Keeping deep sleep out of this tranche is reasonable. It should not be forgotten, though: if the MKR WAN handheld remains in launch scope, N-09 still needs a separate tracked tranche. The current `config.h` already anticipates deep-sleep build support, while this plan places deep sleep in Phase 5; that is acceptable only if handheld battery behavior is explicitly not a blocker for this tranche.

### 8.8 Bottom Line

Approve the next tranche with amendments. The main correction is to make **B0/B1 -> A -> B2 -> C** the practical landing order, resolve both type-code conflicts before new commands land, harden DMA parser concurrency before high-rate host tests, and treat RSSI-snapshot LBT as the launch path instead of CAD-based LBT.

---

## 9. Companion Analysis (GPT-5.3-Codex v1.0, 2026-05-05)

**Reviewer:** GitHub Copilot (GPT-5.3-Codex)
**Purpose:** Add execution-critical details not yet explicit in sections 1-8 so implementation can proceed with lower integration risk.

### 9.1 Verdict

The plan is technically sound and close to implementation-ready. The biggest remaining risk is not architecture; it is **measurement and contract drift** during rapid incremental landing. The next tranche should proceed, with four guardrails added before coding begins:

1. Freeze host wire contracts in one canonical header + host-side vectors before B1 merges.
2. Add a microsecond timing source before any gate that claims sub-millisecond latency (`t_lbt_us`, `t_air_us`, IRQ runtime cap).
3. Treat blocking host TX behavior as an explicit test constraint until async TX is added.
4. Include safe-mode startup window in RESET/BOOT round-trip acceptance numbers.

### 9.2 Missing Timing Primitive (Sub-ms Metrics)

Current platform timebase is SysTick at 1 ms. That cannot support credible measurements for:

- LBT decision budget <= 300 us
- `TX_DONE_URC.t_lbt_us`
- `TX_DONE_URC.t_air_us`
- "longest IRQ runtime < 20 us" gate

Add a `time_us` primitive in B0 or C1 (TIM21/TIM22 free-running counter) and reserve SysTick for coarse scheduling only. Without this, Phase C metrics will be numerically present but not trustworthy.

### 9.3 RESET Round-Trip Gate Needs Re-baseline

The current firmware executes safe-mode listener early with a 500 ms window. Any RESET acceptance gate claiming <1 s must include:

- reset command handling
- hardware reset and startup
- safe-mode scan window
- platform/radio init
- BOOT_URC emit

Recommendation: keep the <1 s target as a stretch metric, but set merge gate to <= 1.2 s until safe-mode window is shortened or made conditional for software-reset cause.

### 9.4 Host Transport Throughput vs Blocking TX

Work item A rightly strengthens RX drain, but acceptance criteria still risks false failures if each received frame immediately triggers blocking `host_uart_send_urc()`. Until async TX is implemented, define stress tests in two classes:

- RX integrity tests: no per-frame response, validate parser/queue/counters only.
- Request-response tests: paced host traffic (next request only after response).

This separates RX-path correctness from TX backpressure artifacts.

### 9.5 Contract Freeze Additions (B0)

In addition to type-code deconfliction, add two explicit version fields:

- `host_proto_version`: semantic protocol version.
- `wire_schema_version`: binary payload layout version.

Then require `VER_URC` to expose both. This prevents future ambiguity where protocol feature set and byte layout evolve at different times.

### 9.6 Radio FSM Entry Criteria Tightening

Before C3/C4 land, add these code-level constraints:

1. `RegOpMode` write ownership enforced by API boundary (`sx1276_modes.c`) and code review checklist.
2. `REG_WRITE` denylist/allowlist cannot bypass `RegOpMode`, `RegDioMapping1/2`, FIFO base pointers in production profile.
3. `RADIO_IRQ_URC` remains debug-only once typed URCs (`RX_FRAME_URC`, `TX_DONE_URC`, `FAULT_URC`) are active, to prevent URC flooding and host parser ambiguity.

### 9.7 Additional Acceptance Gates

Add these gates to the plan:

1. **Wire Freeze Gate:** host parser vectors for `VER_URC`, `ERR_PROTO`, `STATS_URC`, `BOOT_URC`, `RX_FRAME_URC`, `TX_DONE_URC` are committed before the corresponding firmware feature merges.
2. **Boundary Gate:** COBS/CRC framing passes vectors where delimiter and code bytes cross DMA boundaries at 255/256/257 and 511/0 indices.
3. **Reset Cause Gate:** `BOOT_URC.reset_cause` must distinguish at least POR, software reset, watchdog reset.
4. **Timing Trust Gate:** any `*_us` field is emitted only when microsecond source is active; otherwise send sentinel and increment `stats_timing_invalid`.

### 9.8 Recommended Landing Order (Refined)

Final recommended implementation order:

1. **B0:** host types freeze + wire/schema versioning + timing primitive + conflicts fixed in spec and code.
2. **B1:** UID/RESET/STATS/ERR subcodes + dispatch refactor.
3. **A:** DMA HT/TC drain hardening + concurrency protections + boundary tests.
4. **B2:** CFG_SET/CFG_GET (RAM-only) + apply hooks.
5. **C1-C2:** modes ownership + FSM skeleton.
6. **C3-C4:** RX then TX typed URCs.
7. **C5-C6:** RSSI-snapshot LBT + observability + FAULT/BOOT v2 payloads.

Bottom line: proceed now, but establish timing and wire-contract foundations first so the remaining radio work can be verified quantitatively instead of heuristically.
