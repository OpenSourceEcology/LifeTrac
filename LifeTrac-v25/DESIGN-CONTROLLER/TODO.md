# LifeTrac v25 Controller — Development TODO

> **📋 Pre-deployment summary:** for a single consolidated view of
> what still has to happen across all subsystems before field
> deployment (controller + structural + hydraulic + integration +
> field tests + regulatory), see the
> **[Pre-field-deployment checklist](../TODO.md#pre-field-deployment-checklist-open-items-as-of-2026-05-04)**
> section in the top-level [LifeTrac-v25/TODO.md](../TODO.md). This
> file remains the authoritative source for the controller-side phase
> plan; the top-level checklist links back to specific items here.

> **Scope (2026-04-26):** LoRa-only per [MASTER_PLAN.md](MASTER_PLAN.md). Cellular (Cat-M1 / SARA-R412M) line items below are **archived** — do not order, do not implement. Operator-browser ↔ base-station LAN/WiFi is retained. Legacy WiFi/BLE/MQTT-over-WiFi work is in [RESEARCH-CONTROLLER/](RESEARCH-CONTROLLER/).
>
> **Image-pipeline scope (2026-04-27):** see the canonical [IMAGE_PIPELINE.md](IMAGE_PIPELINE.md) implementation plan. The image-pipeline tasks in Phases 4–5 below are pulled from that plan; if they ever drift, IMAGE_PIPELINE.md is the source of truth.
>
> **LoRa system scope (2026-04-27):** see the canonical [LORA_IMPLEMENTATION.md](LORA_IMPLEMENTATION.md) implementation plan. The LoRa-stack tasks scattered across Phases 1–5 below are pulled from that plan; if they ever drift, LORA_IMPLEMENTATION.md is the source of truth for *what to build*, [LORA_PROTOCOL.md](LORA_PROTOCOL.md) for the wire bytes, and [MASTER_PLAN.md §8.17](MASTER_PLAN.md) for PHY policy pins.

Comprehensive task list for implementing the three-tier controller architecture (Portenta Max Carrier on tractor + Portenta Max Carrier with X8 base station + MKR WAN 1310 handheld). See [ARCHITECTURE.md](ARCHITECTURE.md) for the design.

Tasks are organized by phase. Hardware purchases come first because lead times dominate the schedule.

## Go/No-Go Blockers (Current)

> **Current gate state (refreshed 2026-05-12 evening):** **CONDITIONAL GO** — every original Stage 1 blocker (W1-7 LPUART1 RX, W1-9d PB6 TCXO + SYSCLK, W1-9e urc_queue, W1-9f benign FAULT_URC filter, W1-9b T2 + W1-9g post-VER STATS dispatch stall) is CLOSED with bench evidence. **W1-10 Phase A (single-board RX liveness) closed 2026-05-12 evening — 7/7 gates PASS first run** (`bench-evidence/W1-10_rx_liveness_2026-05-11_133459/`: `RegOpMode=0x85` stuck pre/post, `RegRssiValue` -120 → -119 dBm with AGC twitch, `radio_state=4` RX_CONT, host invariants stable, 0 real FAULT_URC across 10 s window). Stage 1 standard quant **20/20 PASS** (W1-7), then **3/3 PASS** post-W1-9g regression. Stage 2 **TX probe 7/7 PASS** (W1-9f, `tx_id=0x42 status=OK time_on_air_us=18048 tx_power_dbm=14`). Stage 2 **FSK STDBY probe `probe_rc=0`** (W1-9g, mode bits 2:0 transition 0→1 = SX1276 clock alive). Build artifact 16,440 B. The **single remaining gate before unconditional GO** is **W1-10 Phase B** (two-board end-to-end ≥99% match rate) — **probe code SHIPPED 2026-05-11 evening** ([`run_w1_10b_rx_pair_end_to_end.ps1`](firmware/x8_lora_bootloader_helper/run_w1_10b_rx_pair_end_to_end.ps1) + `--probe rx_listen` / `--probe tx_burst` sub-modes); RX-side smoke-tested PASS in single-board mode; orchestrator gracefully refuses to run with one board absent. **Execution blocked only on Board 1 (`2D0A1209DABC240B`) returning to the bench** — design + execution recipe captured in [`../AI NOTES/2026-05-12_W1-10_RX_Validation_Plan_Copilot_v1_0.md`](../AI%20NOTES/2026-05-12_W1-10_RX_Validation_Plan_Copilot_v1_0.md) §3.5.
>
> **Historical reframing (2026-05-11) — preserved for traceability:** the official Arduino BSP overlay [`ov_carrier_enuc_lora.dts`](https://raw.githubusercontent.com/arduino/meta-arduino/scarthgap/meta-arduino-nxp/recipes-bsp/device-tree/arduino-device-tree/portenta-x8/overlays/ov_carrier_enuc_lora.dts) shows the i.MX `UART4` ↔ Murata link is **direct TX/RX with no Linux-controlled mux/SEL/OE/EN** — only `H747.PF4` as `NRST`. The pad config is `0x140` (= `PE | PUE`, internal pull-up already enabled — see Step 1 below). This invalidated the Phase A–D GPIO-sweep premise and re-scoped the blockers around (a) i.MX pad pull-up, (b) L072 host UART AF mismatch (LPUART1/AF6 vs USART2/AF4), (c) NRST → first-byte timing alignment. Full synthesis: [`2026-05-11_Blocker_Research_Synthesis_Copilot_v1_0.md`](../AI%20NOTES/2026-05-11_Blocker_Research_Synthesis_Copilot_v1_0.md).

### Action plan (REVISED 2026-05-11 02:30 — major win: HSI16 root cause fixed)

> **2026-05-11 status update:** Step 1' (hello_world decisive test) **PASSED** — `"LIFETRAC L072 tick=0x0000006D"` etc captured cleanly at 19200 8N1, isolating the bug to the production firmware. Step 2' audit then found the root cause: `platform_clock_init_hsi16()` in [`hal/platform.c`](firmware/murata_l072/hal/platform.c) called `RCC_CR &= ~RCC_CR_HSION` after switching SYSCLK to HSE, but [`host_uart_init()`](firmware/murata_l072/host/host_uart.c) explicitly clocks LPUART1 from HSI16 via `CCIPR[11:10]=10`. Disabling HSION cut the LPUART1 baud-rate generator's clock → no TX, no RX, ever. **Fix applied** (don't disable HSI16 in platform.c; defensively re-enable in host_uart.c). After re-flash, the production firmware now emits **BOOT_URC (0xF0), READY_URC (0xF2), STATS/FAULT URCs (0xF1)**, plus the diagnostic ASCII traces `M:RADIO_BYPASS`, `C:SEND_BOOT`, `C:SEND_READY`, `C:INIT_DONE` — all visible in the wrapper's `early_921600_bytes=310` capture. **Primary blocker (LPUART1 silent on /dev/ttymxc3) is CLEARED.** A secondary, smaller bug remains: LPUART1 RX is not receiving the probe's `VER_REQ` (firmware emits `HOST_RX_INACTIVE` fault code 0x0A after 3 s) — see Step 6 below.
>
> **Earlier corrections (preserved):** the original Step 1 (pad pull-up) and Step 2-as-BRR-fix sub-hypothesis were invalidated by primary-source verification. The standard Stage 1 quant `BOOT_OK=1` was a false positive (loopback echo on `/dev/ttymxc0`, 0 bytes on the actual `/dev/ttymxc3`). The Step 0 bench regression was recovered without power-cycle via `gpio10` toggle (i.MX→H7 NRST) + `systemctl restart m4-proxy.service`. See [`../AI NOTES/2026-05-11_Blocker_Research_Synthesis_Copilot_v1_0.md`](../AI%20NOTES/2026-05-11_Blocker_Research_Synthesis_Copilot_v1_0.md).

- [x] **Step 0 — Bench recovery.** Resolved 2026-05-10 without power-cycle: pulse `/sys/class/gpio/gpio10` low/high (i.MX GPIO1_IO10 = H7 NRST per `/usr/arduino/extra/reset.sh`) then `systemctl restart m4-proxy.service`. SWD returned `DPIDR 0x6ba02477`.

- [x] ~~**Step 1 — i.MX `UART4_RXD` pull-up patch.**~~ **INVALIDATED 2026-05-10.** Authoritative bit encoding from u-boot `arch/arm/include/asm/mach-imx/iomux-v3.h` (IMX8MM branch): `PAD_CTL_PUE = 0x40`, `PAD_CTL_PE = 0x100`. Therefore `0x140 = PE | PUE` — the pad **already has the internal pull-up enabled**. Also: the correct conf_reg physical address is `0x303304B4` (UART4_RXD conf_reg offset 0x4B4 + IOMUXC base 0x30330000), not the previously stated `0x30330280`. No `devmem` patch is needed or warranted.

- [x] **Step 1' — Decisive bring-up test: flash `hello_world.bin`.** **PASSED 2026-05-10.** Captured `LIFETRAC L072 tick=0x0000006D` through `0x00000073` cleanly on `/dev/ttymxc3` @ 19200 8N1 → LPUART1/AF6/PA2 ↔ UART4 hardware path fully functional → bug isolated to production firmware logic.

- [x] **Step 2' — Production firmware bring-up audit + HSI16 fix.** **DONE 2026-05-11.** Root cause: `platform_clock_init_hsi16()` cleared `RCC_CR.HSION` after switching SYSCLK to HSE, but `host_uart_init()` selected HSI16 as the LPUART1 kernel clock via `CCIPR[11:10]=10`. With HSION off, LPUART1's baud-rate generator had no clock → continuous low (`0x00`) on TX. **Fix:** removed the `RCC_CR &= ~RCC_CR_HSION` line in [`hal/platform.c`](firmware/murata_l072/hal/platform.c) `platform_clock_init_hsi16()`, and added a defensive `RCC_CR |= RCC_CR_HSION; while ((RCC_CR & RCC_CR_HSIRDY) == 0U) {}` at the top of [`host/host_uart.c`](firmware/murata_l072/host/host_uart.c) `host_uart_init()`. Build artifact: `build/firmware.bin` = 15 288 bytes (was 15 276; +12 bytes). After flash, captured BOOT_URC (0xF0), READY_URC (0xF2), STATS/FAULT URCs (0xF1), plus ASCII traces `M:RADIO_BYPASS`, `C:SEND_BOOT`, `C:SEND_READY`, `C:INIT_DONE` on `/dev/ttymxc3` @ 921600 8N1.

- [x] **Step 6 — LPUART1 RX bug — CLOSED 2026-05-11 (evening). W1-7 PASSED.** Two fixes landed (IDLE-recovery + `host_uart_poll_dma` vs IRQ race on `RDR`); both ASCII (`ATI`/`AT+VER?`/`AT+STAT?`) and binary (`VER_REQ`/`VER_URC`, `STATS_DUMP_REQ`/`STATS_URC`) paths now roundtrip cleanly with `HOST_PARSE_ERR=0`, `HOST_UART_ERR_LPUART=0`, `HOST_UART_ERR_USART1=0`, all per-flag PE/FE/NE/ORE = 0. Method G probe `rc` dropped from `2` (transport timeout) to `1` (the remaining failure is `[FAIL] sx1276 version reg valid` — a separate radio-SPI issue, not a UART issue). **20-cycle standard quant: 20/20 PASS** (`FINAL_RESULT_PASS=20`, exceeds the ≥18/20 acceptance criterion). Diagnostic `C:NOMATCH` branch removed; build artifact: 16 240 bytes. Authoritative writeup: [`../AI NOTES/2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_3.md`](../AI%20NOTES/2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_3.md) §11–§12. Historical investigation kept below for posterity.

- [x] **W1-10 Phase A — Single-board RX-liveness probe — CLOSED 2026-05-12 evening.** Added `--probe rx` sub-mode to [`method_h_stage2_tx_probe.py::run_rx_liveness()`](firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py) and wired it through the existing wrapper [`run_method_h_stage2_tx_end_to_end.ps1 -Probe rx`](firmware/x8_lora_bootloader_helper/run_method_h_stage2_tx_end_to_end.ps1). Probe verifies the firmware's LoRa receive chain is alive end-to-end up to the host URC dispatch boundary, **without** requiring a paired transmitter. Seven gates (RegOpMode pre/post == 0x85, RegRssiValue in plausible band, RegRssiValue varies between samples = AGC alive, radio_state == 4 (RX_CONT), host invariants stable, no real FAULT_URC during window). **Bench evidence:** [`bench-evidence/W1-10_rx_liveness_2026-05-11_133459/`](bench-evidence/W1-10_rx_liveness_2026-05-11_133459/) — 7/7 gates PASS first run on Board 2. `RegOpMode(pre)=0x85`, `RegOpMode(post)=0x85`, `RegRssiValue A=-120 dBm B=-119 dBm` (1 LSB AGC twitch on quiet bench noise floor — exactly the Semtech §5.5.5 expectation), `RegModemStat=0x04` (only RxOngoing bit set = idle-but-armed for preamble), `radio_state=4` (RX_CONT), host_parse_err=0, all UART per-flag error counters unchanged, 0 real FAULT_URC across 10 s window (post-VER drain consumed 4 frames per W1-9g `drain_pending()`). Authoritative writeup: [`../AI NOTES/2026-05-12_W1-10_RX_Validation_Plan_Copilot_v1_0.md`](../AI%20NOTES/2026-05-12_W1-10_RX_Validation_Plan_Copilot_v1_0.md). **Next action:** Phase B (W1-10b) two-board end-to-end ≥99% match rate per [BRINGUP_MAX_CARRIER §5 Stage 3](firmware/murata_l072/BRINGUP_MAX_CARRIER.md) — **probe code shipped 2026-05-11 evening**: new `--probe rx_listen` (RX side) and `--probe tx_burst` (TX side) sub-modes in [`method_h_stage2_tx_probe.py`](firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py), orchestrator [`run_w1_10b_rx_pair_end_to_end.ps1`](firmware/x8_lora_bootloader_helper/run_w1_10b_rx_pair_end_to_end.ps1) opens two parallel ADB sessions, correlates by full payload (`W1-10b seq=NNNN <random4hex>`), evaluates B1..B6, writes `summary.json` to `bench-evidence/W1-10b_rx_pair_<stamp>/`. RX-side smoke test 2026-05-11 14:01 PASS (`__W1_10B_LISTEN_DONE__ rx_frames=0 ... real_faults=0 invariants_violated=0 probe_rc=0`); orchestrator pre-flight check gracefully fails when one board absent. Execution recipe in [`../AI NOTES/2026-05-12_W1-10_RX_Validation_Plan_Copilot_v1_0.md`](../AI%20NOTES/2026-05-12_W1-10_RX_Validation_Plan_Copilot_v1_0.md) §3.5. **Blocked on Board 1 (`2D0A1209DABC240B`) returning to the bench**; reflash with Stage 1 helper if it has pre-W1-9d firmware.

- [x] **W1-9g + W1-9b T2 — Post-VER STATS dispatch stall + FSK probe relaxation — CLOSED 2026-05-12.** Root cause was probe-side (no firmware change): after `VER_REQ` returned, the firmware emitted unsolicited `FAULT_URC(HOST_RX_SEEN)` + auto-`STATS_URC(seq=0)` snapshot from `main.c` lines 92/98 (one-shot first-RX-byte diagnostic), and these arrived AFTER `VER_URC` so they got parked in `HostLink.urc_queue`. The next `request(STATS_DUMP_REQ, STATS_URC)` then drained the queued `STATS_URC(seq=0)` snapshots and discarded them as stale type-matches, but the discard loop consumed the entire 1.0 s response window before the actual `STATS_URC(seq=N)` reply could land — verdict `TRANSPORT_FAIL`. Stage 1 didn't hit this because its boot-window `read_frames()` loop explicitly handles every URC type (BOOT/STATS/FAULT/READY/ERR_PROTO) and the per-call print/format work between requests gives a few hundred ms of natural settle. **Fix (probe-side, two functions in [`method_h_stage2_tx_probe.py`](firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py)):** (1) extended `drain_boot()` to handle every URC type explicitly so they don't get parked; (2) added `drain_pending(link, quiet_s=0.25, max_s=1.0)` helper that drains `urc_queue` AND wire interleaved until quiet for 250 ms, called between the `VER_REQ` warm-up and `fetch_stats()`. Plus the previously-landed W1-9b T2 changes in the same probe (pre-state relaxation to accept any LoRa-mode bit-7-set, `VER_REQ` warm-up moved before `fetch_stats`) and the W1-9b/W1-9g `HostLink.request()` rewrite in [`method_g_stage1_probe.py`](firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py) (drain `urc_queue` once up front, then always read wire; discard stale-seq type-matches with logged INFO). **Bench evidence:** [`bench-evidence/W1-9b_fsk_2026-05-11_125612/`](bench-evidence/W1-9b_fsk_2026-05-11_125612/) — `T0b: post-VER drain consumed 4 frames`, `STATS(before) host_parse_err=0`, FSK SLEEP/STDBY transitions execute (`step5 RegOpMode=0x80`, `step8 RegOpMode=0x81` → mode bits 2:0 transition 0→1 = clock alive), `probe_rc=0`. Stage 1 regression check: 3/3 PASS preserved (`bench-evidence/T6_stage1_standard_quant_2026-05-11_125646_*/`). Final FSK verdict `T2_INCONSISTENT` (rather than `FSK_STDBY_WORKS`) is a separate, minor classifier issue: the probe writes `0x00` (FSK SLEEP) but reads back `0x80` (LoRa SLEEP) because per SX1276 datasheet `LongRangeMode` (bit 7) can only toggle from SLEEP — chip silently keeps LoRa mode on the FSK write because it was in `0x85` (LoRa+RXCONTINUOUS) at probe entry. To capture `FSK_STDBY_WORKS` properly the probe would need to pre-walk LoRa→LoRa-SLEEP→FSK-SLEEP — out of scope for the W1-9b/W1-9g closure since the discriminator's actual invariant (mode bits 2:0 transition when commanded) is already proven. Authoritative writeup: [`../AI NOTES/2026-05-12_W1-9g_Post_VER_STATS_Dispatch_Stall_Copilot_v1_0.md`](../AI%20NOTES/2026-05-12_W1-9g_Post_VER_STATS_Dispatch_Stall_Copilot_v1_0.md). **Lessons:** any probe-side `drain_boot()` MUST handle every URC type the firmware can emit out-of-band — unhandled URCs are at best silently iterated past, at worst left in `urc_queue` to poison the next `request()`; tight back-to-back `request(A); request(B);` is fragile when A's response triggers additional unsolicited URCs (always drain between calls, or build a `request_drain()` that explicitly consumes type-mismatches before returning). **Next action:** RX validation milestone (firmware boots into RXCONTINUOUS but no end-to-end packet-receive test exists yet).

- [x] **W1-9f — TX-probe benign FAULT_URC filter — CLOSED 2026-05-12.** First end-to-end TX cycle (after W1-9d/W1-9e) reported gate-fail `[FAIL] no FAULT_URC during cycle` because of one `FAULT_URC code=0x09 sub=0x01 lanes=LPUART1`. Investigation showed fault code `0x09` is **`HOST_FAULT_CODE_HOST_RX_SEEN`** (per `include/host_types.h` and emit site at `main.c` line 91) — a one-shot diagnostic the L072 fires the first time it observes any host byte on a UART lane. NOT `HOST_PARSE_ERROR (0x0B)` as initially mis-labeled in the W1-9e closure note. End-of-cycle `host_parse_err=0` confirmed no parse errors. The probe's `format_fault_payload()` already classifies it correctly (`lanes=LPUART1`), but `wait_for_tx_done()` blindly added every FAULT_URC type to the fault list. **Fix:** added `BENIGN_FAULT_CODES = (HOST_FAULT_CODE_HOST_RX_SEEN, HOST_FAULT_CODE_HOST_DIAG_MARK)` filter in [`firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py`](firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe.py); benign codes now print as `INFO: diagnostic FAULT_URC during TX wait (benign)` and are excluded from the gate. Real faults (HARDFAULT, NMI, RADIO_*, HOST_PARSE_ERROR, HOST_DMA_OVERRUN, STACK_GUARD, CLOCK_HSE_FAILED, HOST_RX_INACTIVE) still fail the gate. **Result:** W1-9 TX probe **7/7 PASS** (`probe_rc=0`, `TX_DONE_URC tx_id=0x42 status=OK time_on_air_us=18048 tx_power_dbm=14`). Bench evidence: `bench-evidence/W1-9_stage2_tx_2026-05-11_115340/`. Recipe doc [`DESIGN-LORAFIRMWARE/07_Successful_Connection_Recipe_All_Chips_Copilot_v1_0.md`](DESIGN-LORAFIRMWARE/07_Successful_Connection_Recipe_All_Chips_Copilot_v1_0.md) §6 + §7 updated. **Lesson:** when a diagnostic emission shares the same URC type-byte (0xF1 FAULT_URC) as a real fault, host-side gate logic must distinguish by fault-code, not just URC type. **Next action:** relax W1-9b T2 probe pre-state assertion (accept any non-SLEEP LoRa mode); then move to RX validation milestone.

- [x] **W1-9d — PB6 TCXO_VCC enable — IMPLEMENTED & BENCH-CONFIRMED 2026-05-12.** Two-delta firmware change to `hal/platform.c::platform_clock_init_hsi16()`: (D-1) drive **PB6 HIGH** alongside legacy PA12 right after enabling GPIOB clock, before HSE BYPASS probe (powers the TCXO via the ABX00043 R12 0Ω strap); (D-2) **remove the SYSCLK→HSE switch** so the function honors its name (probe HSE for `clock_source_id` reporting only; always keep SYSCLK on HSI16). D-2 was a latent bug only exposed once HSE actually locked: switching SYSCLK 16→32 MHz silently doubled USART1 kernel clock (USART1 uses SYSCLK by default per `RCC_CCIPR[1:0]=00`), halving the 921 600 host transport baud — first probe attempt with D-1-only returned `TRANSPORT_FAIL`. Build: **16 440 B** (was 16 432 B baseline; D-1+SYSCLK-switch attempt was 16 484 B). Stage 1 standard quant **2/2 PASS** post-fix; pre-fix 8-cycle abbreviated run was **7/8 PASS** (one `FAIL_BOOT` cycle inside the broken-baud window). **W1-9b T2 FSK probe verdict moved from `T2_FSK_SLEEP_ONLY` → `PRE_STATE_UNEXPECTED_0x85`** — `RegOpMode=0x85` decodes as **LoRa + RXCONTINUOUS**, meaning the SX1276 successfully exits SLEEP and is actively receiving. The probe just needs its `expected pre-state=0x80 (SLEEP)` assertion relaxed to accept any non-SLEEP LoRa mode (firmware boots straight into LoRa RX now). **W1-10 hardware escalation no longer required — W1-9 OPMODE-stuck root cause was a software/strap mismatch (PA12 vs PB6) compounded by a latent SYSCLK-derived USART1 baud bug, not a missing-clock hardware defect.** Bench evidence: `bench-evidence/T6_stage1_standard_quant_2026-05-11_112444_120-2576/` (Stage 1 PASS) and `bench-evidence/W1-9b_fsk_2026-05-11_112605/` (T2 verdict). Authoritative plan: [`../AI NOTES/2026-05-12_W1-9d_PB6_TCXO_Enable_Plan_Copilot_v1_0.md`](../AI%20NOTES/2026-05-12_W1-9d_PB6_TCXO_Enable_Plan_Copilot_v1_0.md). **Next action:** relax W1-9b T2 probe pre-state assertion (accept any non-SLEEP LoRa mode), then proceed to W1-9 TX bring-up retry now that SX1276 is no longer mode-locked.

- [ ] **W1-9d original plan entry (superseded by ✓ above) — PLAN 2026-05-12.** New diagnostic plan that argues the SX1276 SLEEP-stuck and HSE/TCXO probe failures share a single software root cause: `platform_clock_init_hsi16()` drives **PA12** HIGH for TCXO enable (B-L072Z-LRWAN1 reference pin), but ABX00043 carrier straps **R12 0Ω (PB6)** with **R86 (PA12) DNP** — so PA12 is a no-op on this board and PB6 (the actual TCXO_VCC enable per hardwario/lora-modem `TCXO_PIN==2` variant) is left floating. Three converging evidence lines: (1) W1-9c-verified schematic; (2) `BOOT_URC.clock_source_id=1` (HSI_FALLBACK) on **every** recorded boot since W1-7 = HSE has never locked = TCXO never alive at L072 OSC_IN; (3) developer comment in [`firmware/murata_l072/lora_ping.c`](firmware/murata_l072/lora_ping.c) line 610 ("Force HSI16 (proven path, avoids HSE/TCXO probe issues)") confirms the workaround was knowingly applied. **Proposed fix:** 5-line addition to `platform_clock_init_hsi16()` driving PB6 HIGH alongside PA12 before the HSE BYPASS probe. **Risk: near-zero** (PB6 currently floats; cannot disturb a pin already 0Ω strapped to a +3V3-tied rail). Acceptance: `clock_source_id=0` (HSE_OK) post-fix and W1-9b T2 verdict no longer `T2_FSK_SLEEP_ONLY`. Plan also enumerates 8 software-only fallback options (DIO pullups, REG_IMAGECAL probe, DIO5/CLKOUT clock probe, multi-reset retry, longer NRESET, second-board test, MKRWANFWUpdate stock-firmware diagnostic, sequencer-stop write) before any W1-10 hardware escalation. Authoritative writeup: [`../AI NOTES/2026-05-12_W1-9d_PB6_TCXO_Enable_Plan_Copilot_v1_0.md`](../AI%20NOTES/2026-05-12_W1-9d_PB6_TCXO_Enable_Plan_Copilot_v1_0.md). **Next action:** apply Option-1 firmware change, build, flash, run Stage 1 quant 20 + W1-9b T2 probe.

- [x] **W1-9b plan doc cleanup — 2026-05-11.** Patched [`../AI NOTES/2026-05-11_W1-9b_OPMODE_Stuck_Probe_Plan_Copilot_v1_0.md`](../AI%20NOTES/2026-05-11_W1-9b_OPMODE_Stuck_Probe_Plan_Copilot_v1_0.md) with three documentation deltas (no firmware change, no bench rerun required): (1) §3 verdict matrix amended to forward-point at the §10.2/§11.2 narrowing and to label the `0x00 / 0x00` row as `T2_FSK_SLEEP_ONLY`; (2) restored all §-character encoding corruption in §11 (zero U+FFFD remaining); (3) added §12 Outcome stub recording the bench verdict (`T2_FSK_SLEEP_ONLY`), `W1_9B_DECISION=W1-10`, and forward links to the W1-9b closure and the W1-9c software follow-up closure. Regression check after the doc edits: **Stage 1 standard quant 20/20 PASS** (`FINAL_RESULT_PASS=20`, `LAUNCHER_FAIL_COUNT=0`, `TIMEOUT_COUNT=0`, `GATE_RESULT=PASS`); evidence `bench-evidence/T6_stage1_standard_quant_2026-05-11_102825_292-27400/`. W1-9c verdict (`T2_FSK_SLEEP_ONLY`) and W1-10 hardware-side TCXO escalation unchanged.

- [x] **Step 8c — W1-9c software follow-up (Semtech-reference fixes) — CLOSED 2026-05-12. Decision: ESCALATE_TO_W1-10_HARDWARE re-confirmed.** Two surgical fixes applied per Lora-net/LoRaMac-node canonical reference: (C-1) `sx1276_radio_reset()` switched from active push-pull HIGH release to **Hi-Z release** (drive PC0 LOW 1 ms, then `gpio_mode_input` → SX1276 internal pull-up brings line up, wait 6 ms) — matches `SX1276Reset()` in `boards/B-L072Z-LRWAN1/sx1276-board.c`; (C-2) RegTcxo (0x4B) write switched to **read-modify-write** preserving reserved bits 3:0 (POR default `0x09`) → final value `0x19` instead of clobbering reserved bits with `0x10` — matches Murata `mlm32l07x01` and datasheet rev 7 §4.1.7 ("Retain default value"). Build: 16 432 B (unchanged), 0 warnings, CI guard preserved. Bench: FSK STDBY discriminator probe verdict **`T2_FSK_SLEEP_ONLY` (no change vs W1-9b)** — chip still cannot leave SLEEP. This rules out two more software root-cause categories (reset-driver-fight glitch, RegTcxo reserved-bit corruption) and reinforces the "no 32 MHz clock at the SX1276 die" hypothesis even more strongly. Both fixes retained as canonical-reference improvements. Authoritative writeup: [`../AI NOTES/2026-05-12_W1-9c_Software_Followup_Closure_Copilot_v1_0.md`](../AI%20NOTES/2026-05-12_W1-9c_Software_Followup_Closure_Copilot_v1_0.md). Bench evidence: `bench-evidence/W1-9b_fsk_2026-05-11_085842/`.

- [x] **Step 8b — W1-9b OPMODE-stuck discriminator — CLOSED 2026-05-11 (overnight). Decision: ESCALATE_TO_W1-10_HARDWARE.** Three software bench probes (T1 RegVersion burst 1024/1024 PASS; T2 FSK STDBY datapath verdict `FSK_SLEEP_ONLY`; T3 skipped per matrix) executed against unchanged W1-9 firmware (16 432 B). The chip can clear `LongRangeMode` (bit 7 of `RegOpMode`) via the SPI shadow path but **cannot transition mode bits 2:0** in either FSK or LoRa family — falsifies the "LoRa-mode-specific firmware bug" hypothesis and strongly reinforces the "no digital clock at the SX1276 die" hypothesis. W1-7 5-cycle quant regression: **5/5 PASS**, gate `PASS`. W1-10 (TCXO-rail / SiP `XTA` scope investigation, hardware-side) remains the next action; out of agent scope. Authoritative writeup: [`../AI NOTES/2026-05-11_W1-9b_OPMODE_Stuck_Probe_Closure_Copilot_v1_0.md`](../AI%20NOTES/2026-05-11_W1-9b_OPMODE_Stuck_Probe_Closure_Copilot_v1_0.md). Probe + wrappers extended with `--probe {regversion,fsk,opmode_walk}` modes; default `tx` path unchanged.

- [ ] **Step 8 — SX1276 TX bring-up — DEFERRED 2026-05-11 (late night). W1-9 partial.** Diagnostic infrastructure (Stage 2 TX probe + LBT bypass + multi-stage register peek + direct `REG_WRITE_REQ` SPI-write isolation) is committed and bench-validated. Root cause **isolated**: `RegOpMode` (0x01) refuses to transition out of LoRa SLEEP (0x80) via firmware-issued or host-issued writes, while configuration-register writes (FRF, PA_CONFIG, MODEM_CFG*, FIFO/PAYLOAD_LEN) all latch correctly — strongly indicates the SX1276 die's digital state-machine clock is missing. Software fixes attempted (`RegTcxo[4] = TcxoInputOn`, FSK→LoRa SLEEP transition per datasheet §4.1.6, OPMODE-write retry loop with 5 ms budget) did not unstick the chip, pointing at hardware-level investigation (Max Carrier ABX00043 schematic for a second `TCXO_VCC` enable beyond PA12, oscilloscope probe of TCXO trace at SiP `XTA` pin, or test on a different module). **Acceptance gates A/B/E/F/G all PASS; gates C and D FAIL** because TX never starts (`status = TIMEOUT`, `radio_tx_ok` does not advance, `radio_dio0` stays at 0). **W1-7 regression: 5/5 PASS** with W1-9 firmware (16 432 B). **W1-8 still PASS** (Method G probe rc=0, REG 0x42=0x12). Authoritative writeup: [`../AI NOTES/2026-05-11_W1-9_TX_BringUp_Closure_Copilot_v1_1.md`](../AI%20NOTES/2026-05-11_W1-9_TX_BringUp_Closure_Copilot_v1_1.md) (closure §8.1–§8.9) plus the [v1.0 plan](../AI%20NOTES/2026-05-11_W1-9_TX_BringUp_Plan_Copilot_v1_0.md). Bench logs: `stage2_w1_9_final.log`, `quant5_w1_9.log`. Reopen as **W1-10** when a hardware-side TCXO investigation is on the bench.

- [x] **Step 7 — SX1276 SPI bring-up — CLOSED 2026-05-11 (late night). W1-8 PASSED.** Root cause: `radio/sx1276.c` `sx1276_gpio_init()` configured **PA5 as SPI1_SCK**, but on the Murata CMWX1ZZABZ-078 SiP **PA5 is wired to SX1276 DIO4** — the actual SCK net is **PB3**. Result: SX1276 received no clock, MISO floated, every register read returned `0x00`. The standalone [`firmware/murata_l072/lora_ping.c`](firmware/murata_l072/lora_ping.c) `gpio_spi_init()` already proved PB3=SCK works on this hardware. **Fix (3 edits in `radio/sx1276.c`):** (1) retarget SCK from PA5 to PB3, (2) add MISO PUPDR pull-up on PA6 (defensive against floating reads), (3) add `BSY`-wait before re-init in `sx1276_spi_init()`. Build: 16 292 bytes (+52 from W1-7 baseline). **Method G probe verdict: PASS** (`probe_rc=0`); `REG 0x42 VERSION = 0x12` (was `0x00`); full register dump shows real silicon contents (e.g., `0x00: 69 80 1A 0B 00 52 E4 C0 …`, RegFifoTxBaseAddr `0x0E = 0x80`, RegModemConfig1 `0x1D = 0x82`). **Regression check: W1-7 20-cycle quant rerun = 20/20 PASS** (`FINAL_RESULT_PASS=20`, no UART regression). Authoritative writeup: [`../AI NOTES/2026-05-11_W1-8_SX1276_SPI_BringUp_Plan_Copilot_v1_0.md`](../AI%20NOTES/2026-05-11_W1-8_SX1276_SPI_BringUp_Plan_Copilot_v1_0.md). Bench logs: `stage1_probe_w1_8.log`, `stage1_quant_w1_8.log`. Stage 1 protocol + register-access checks are now both green; ready for W1-9 (TX bring-up).

  - Stimulus test (reset L072, deliberately write `VER_REQ` and `AT\r\n` to `/dev/ttymxc3 @ 921600`) captured the diagnostic ASCII traces `H:RX_LPUART1` (ISR fired on first byte), `H:ERR_LPUART1` (`USART_ISR.PE|FE|NE|ORE` set on a subsequent byte), `H:PROC_FRAME` (parser reached), and `H:COBS_ERR` (decoder rejected corrupted frame), plus a FAULT_URC body with `HOST_DIAG_MARK_FRAME_PARSE_ERR` (0x02). RX hardware path is **proven functional**.
  - **Two fix attempts on 2026-05-11 both FAILED:**
    1. Switched LPUART1 kernel clock from HSI16 to SYSCLK + computed BRR from runtime `platform_core_hz()`. Diagnostic trace `C:CLK_HSI_16M` proves HSE never locked on this hardware (`platform_clock_init_hsi16()` falls back to HSI16). PA12-as-TCXO_EN attempt (push-pull HIGH + 5 ms settle before HSE probe) did NOT enable HSE — PA12 is not the TCXO enable on the Arduino Max Carrier wiring (or VDD_TCXO rail itself is missing per BRINGUP_MAX_CARRIER §R12).
    2. Dropped `HOST_BAUD_DEFAULT` from 921600 to 115200 (rebuilt firmware = 15 428 bytes, flashed, re-ran stimulus at 115200 baud). The four error traces (`H:RX_LPUART1`, `H:ERR_LPUART1`, `H:PROC_FRAME`, `H:COBS_ERR`) **still appear**. At 115 200 with HSI16 ±1%, the sample-clock drift over a 10-bit frame is ~1 sample at 16× oversampling — well within tolerance. **This falsifies the clock-tolerance hypothesis.** `HOST_BAUD_DEFAULT` was reverted to `921600UL`.
  - **2026-05-11 source-audit update:** the ISR self-stretch hypothesis is promoted to a source-level bug. `platform_diag_trace()` blocks on LPUART1 TXE/TC; `RNG_LPUART1_IRQHandler()` and `USART1_IRQHandler()` call it directly, then call `ingest_rx_byte()`, which can synchronously reach `ingest_binary_byte()` → `process_encoded_frame()` and emit `H:PROC_FRAME`, `H:COBS_ERR`, `H:PARSE_ERR`, `H:FRAME_OK`, etc. So parser traces are also IRQ-context blocking calls when the delimiter is consumed by RX IRQ. In addition, `host_uart_poll_dma()` currently holds `PRIMASK=1` across its RX/error polling loop and can call the same blocking trace/parser path before restoring interrupts.
  - **Revised next action (v1.3 plan, supersedes v1.2):** v1.2 patch landed and was bench-validated 2026-05-11. Build clean (`firmware.bin` 16 140 B, sha256 `324FC313F938CEEAFBC3CF25BF02D35ECEC2C353359E2E1A3F27D814BE452003`). Real-time invariant restored: `ATI` round-trips end-to-end (70-byte banner reply with `H:RX_LPUART1` + `H:AT_DISPATCH` deferred traces); BOOT/READY/FAULT/STATS URCs all emit as proper COBS frames; standard-quant `BOOT_OK=1` for every cycle (≥17/20 PASS at time of writing). Remaining failure is **narrower** than v1.2 hang: binary `VER_REQ → VER_URC` still times out, and a follow-up `AT+VER?` produces only one `H:RX_LPUART1` deferred trace and no `H:AT_DISPATCH`. v1.3 plan turns this into bench debugging instead of more architecture work — see [`../AI NOTES/2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_3.md`](../AI%20NOTES/2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_3.md). Order: **(D)** capture one populated 132-byte STATS payload from a failed probe + an i.MX TX byte-count snoop; **(E)** branch deterministically on the per-flag PE/FE/NE/ORE/ring_ovf counters using the v1.3 §2 decision tree; **(F)** re-run 20-cycle quant. Out of scope for v1.3: DMA conversion, HSE/TCXO, USART1 mirror disable (kept as branch E.usart1-mirror), function rename.
  - Detailed analysis (with all evidence + hex captures): [`../AI NOTES/2026-05-11_W1-7_HSI16_Root_Cause_And_Fix_Copilot_v1_0.md`](../AI%20NOTES/2026-05-11_W1-7_HSI16_Root_Cause_And_Fix_Copilot_v1_0.md) §3.5.3–3.5.6. Plan history: source-audit plan [`v1.1`](../AI%20NOTES/2026-05-11_W1-7_RX_Research_Plan_Copilot_v1_1.md) → implementation plan [`v1.2`](../AI%20NOTES/2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_2.md) (landed) → debug plan [`v1.3`](../AI%20NOTES/2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_3.md) (authoritative). Bench evidence: [`bench-evidence/T6_bringup_2026-05-11_003104/`](bench-evidence/T6_bringup_2026-05-11_003104) and [`bench-evidence/T6_stage1_standard_quant_2026-05-11_003335_280-32152/`](bench-evidence/T6_stage1_standard_quant_2026-05-11_003335_280-32152).
  - **Code state at end of session 2026-05-11 (post-v1.2):** [`host/host_uart.c`](firmware/murata_l072/host/host_uart.c) drain-only IRQ handlers + 512-byte SW RX ring + deferred `HOST_TRACE_*` bitmask + foreground `host_uart_service_rx` / `host_uart_flush_diag_traces` + non-blocking `host_uart_poll_dma`; [`include/host_uart.h`](firmware/murata_l072/include/host_uart.h) declares 13 trace flags + 9 new stat getters; [`include/host_types.h`](firmware/murata_l072/include/host_types.h) `HOST_STATS_PAYLOAD_LEN=132`; [`host/host_stats.c`](firmware/murata_l072/host/host_stats.c) extended `host_stats_wire_t` + serializer; [`main.c`](firmware/murata_l072/main.c) main loop calls `poll_dma → service_rx → flush_diag_traces`; [`firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py`](firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py) `parse_stats()` accepts new 132-byte payload (backward compatible with 96-byte). Build artifact: 16 140 bytes.
  - **2026-05-11 evening update (post-v1.3 §10/§11): TWO root causes identified and fixed; ASCII path fully working end-to-end; binary path now the only remaining blocker.**
    - **Fix #1 — IDLE-recovery (Residual transport blocker, v1.3 §9–§10):** boot-time line noise on `/dev/ttymxc3` left `s_cobs_encoded_len > 0` with no terminating `0x00`, permanently gating off AT-shell entry (`ingest_rx_byte` only enters AT mode when the COBS state is empty). Fix: USART_ISR_IDLE handler sets `s_rx_idle_pending`; foreground `host_uart_service_rx` clears stuck COBS state when no AT candidate is in flight. Build delta: 16 140 → 16 224 B.
    - **Fix #2 — `host_uart_poll_dma` vs IRQ race on RDR (Residual A, v1.3 §11):** foreground cached `lpuart_isr` (RXNE=1), the RX IRQ pre-empted and drained `LPUART1_RDR`, then foreground re-read `RDR` based on the stale cached flag and pushed a duplicate byte to `s_rx_ring` while also incrementing `s_stats_rx_lpuart_bytes`. Manifested as a positionally-inserted ghost byte (`AT+VER?` → `AT+VTER?`; `AT+STAT?` → `AT+STTAT?`). Diagnostic confirmed `RX_LP` delta of 11 for a 10-byte send. Fix: wrap each `ISR-snapshot + RDR-drain` step inside `host_uart_poll_dma` in `cpu_irq_save`/`cpu_irq_restore` so foreground is atomic w.r.t. the IRQ. Build delta: 16 300 → 16 344 B.
    - **Bench validation (16 344 B firmware via Method G, captured by [`d3_bash_ingress.sh`](firmware/x8_lora_bootloader_helper/d3_bash_ingress.sh)):** `ATI` → 70-byte banner + `OK`; `AT+VER?` → `FW=OSE-LifeTracLORA-MurataFW VERSION=0.0.0 GIT=dev PROTO=1 SCHEMA=1 CAP=0x0000007F` + `OK`; `AT+STAT?` → full ASCII stats dump including `HOST_PARSE_OK=115 HOST_PARSE_ERR=0 HOST_UART_ERR_LPUART=0 HOST_UART_ERR_USART1=0`; second `ATI` → idempotent. Method G probe progressed from `rc=2` → `rc=1`.
    - **Remaining: Residual B (binary `VER_REQ`/`STATS_DUMP_REQ` still timeout).** Next bench probe: capture `HOST_PARSE_ERR` via `AT+STAT?` before/after a Python binary attempt to discriminate "request never arrives at L072" vs "silent COBS/CRC parse error". Once Residual B closes, remove the temporary `C:NOMATCH` diagnostic in [`host/host_cmd.c`](firmware/murata_l072/host/host_cmd.c) (or gate it behind a build flag) and re-run the 20-cycle quant for the ≥18/20 PASS criterion. Authoritative writeup: [`../AI NOTES/2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_3.md`](../AI%20NOTES/2026-05-11_W1-7_RX_Implementation_Plan_Copilot_v1_3.md) §11.

- [ ] **Step 3 — Combined fixes with NRST → first-byte timing tightened.** *(Unchanged in intent; only relevant once Step 1' identifies whether the production firmware is the issue. The NRST timing tightening still applies regardless.)*
  - Ensure the active probe waits for `BOOT_URC` (with timeout) or at least `≥250 ms` after `gpioset gpiochip5 3=1` before sending `VER_REQ`.
  - In the L072 firmware, ensure no GPIO/UART activity on PA2 between `host_uart_init` and the first beacon (`≥50 ms` settle).
  - Run the full standard 20-cycle Stage 1 quant (`run_stage1_standard_quant_end_to_end.ps1 -Cycles 20`).
  - **Pass criterion (W1-7 cleared):** ≥18/20 cycles produce a valid `BOOT_URC` followed by a successful `VER_REQ` → `VER_URC` exchange.
  - **NOTE:** the standard quant's existing `BOOT_OK` definition must also be hardened — see Step 3a below.

- [ ] **Step 3a (NEW) — Harden the Stage 1 standard contract gate.**
  - Patch [`firmware/x8_lora_bootloader_helper/run_stage1_standard_contract.sh`](firmware/x8_lora_bootloader_helper/run_stage1_standard_contract.sh) line ~97 so the AT-response sweep does **not** accept matches from `/dev/ttymxc0` (the X8 console UART, which loops back our own writes since it has no termios layer). Restrict the sweep to `/dev/ttymxc3` only and require a non-echo byte sequence (e.g. response must contain bytes outside the request bytes), or replace the AT loop with a call to `method_g_stage1_probe.py` and gate on its `0x81` response.
  - **Pass criterion:** with the L072 deliberately held in reset, the contract reports `FINAL_RESULT_FAIL` (not PASS-by-loopback).

- [ ] **Step 4 — Logic-analyzer capture on `UART4_TXD` and `Murata.PA2` during boot + `VER_REQ`.**
  - Use a Saleae (or equivalent ≥10 MS/s) on the carrier test points or the Murata module pads. Capture: `PF4` (NRST), `UART4_TXD`, `UART4_RXD`/`Murata.PA2`, `Murata.PA3`.
  - Capture window: from `gpioset` release through 1 s after `VER_REQ`.
  - Determine which side is silent and at what bit time, and whether either line idles low (no pull) or shows framing errors.
  - **Outcome:** definitive root cause; update earlier steps' disposition and define the targeted fix (firmware bug, pad config, hardware net break).

- [ ] **Step 5 — ABX00043 schematic verification of the direct-route hypothesis.**
  - Manually trace the [Portenta Max Carrier schematic PDF](https://docs.arduino.cc/hardware/portenta-max-carrier/) for three nets:
    1. `Murata.PA2` and `Murata.PA3` → `i.MX UART4_TXD` / `UART4_RXD` (expect: only series resistors; no level shifter / no mux / no buffer).
    2. `Murata.NRST` → `H747.PF4` (expect: 1 MΩ external pull-up).
    3. `Murata.BOOT0` → `H747.PA11` (expect: direct, no buffer / no mux).
  - Record findings in [`CHIP-DOCS/IMX8MM_UART4/findings.md`](CHIP-DOCS/IMX8MM_UART4/findings.md) and [`CHIP-DOCS/MURATA_CMWX1ZZABZ_078/findings.md`](CHIP-DOCS/MURATA_CMWX1ZZABZ_078/findings.md).
  - **Pass criterion (closes Blocker C):** all three nets confirmed direct → file a one-page net map and explicitly disposition U16–U22 as belonging to other carrier subsystems (RS-232/RS-485/audio).
  - **On fail (unexpected mux found):** reopen Phase D with a real, schematic-derived target.

### Blocker disposition matrix

- [x] **Blocker A — W1-7 protocol bring-up gate (primary). CLOSED 2026-05-11 (evening).**
  **Go criterion (met):** on `/dev/ttymxc3` at `921600 8N1`, end-to-end Method G probe completes binary `VER_REQ`/`VER_URC` and `STATS_DUMP_REQ`/`STATS_URC` with `HOST_PARSE_ERR=0` and all UART error counters = 0; probe `rc` dropped from 2 → 1 (remaining `rc=1` is the separate SX1276 register-readback failure tracked under W1-8). The 20-cycle quant gate is the final acceptance check for the W1-7 milestone — see test result alongside [`bench-evidence/`](bench-evidence) once captured. **Cleared by Step 6.**

- [ ] **Blocker B — No deterministic board-level ingress gate condition.**
  **Revised go criterion:** a reproducible **pad-config or firmware-AF state** (not GPIO pin) that consistently changes ingress classification across immediate repeats.
  **Plan:** Steps 1 and 2 each independently satisfy this if they change classification. The 34-pin Phase D-1/D-2/D-3 GPIO sweep is preserved as the *negative corpus* that ruled out the "Linux-controlled mux SEL" hypothesis.

- [ ] **Blocker C — ABX00043 net-ownership extraction for UART route-control.**
  **Revised go criterion:** verified net map for the Murata UART4 link from the ABX00043 schematic, confirming or refuting the BSP-derived direct-route hypothesis.
  **Plan:** Step 5. The previously planned multi-pin / polarity / timing combinatorics in Phase E is **dropped** as unsupported by the BSP overlay.

- [ ] **Blocker D — Timed hold-window dependence.**
  **Disposition:** **closing as a root-cause dimension** (not reproducible in the 100/500/1500 ms sweep). Replaced by the NRST → first-byte timing alignment work in Step 3. Once Step 3 passes, this checkbox can be ticked with the "closed by re-scope" disposition.

**References (current):** synthesis note [`../AI NOTES/2026-05-11_Blocker_Research_Synthesis_Copilot_v1_0.md`](../AI%20NOTES/2026-05-11_Blocker_Research_Synthesis_Copilot_v1_0.md); Phase D negative corpus [`../AI NOTES/2026-05-11_Phase_D1_GPIOB_Sweep_Results_Copilot_v1_0.md`](../AI%20NOTES/2026-05-11_Phase_D1_GPIOB_Sweep_Results_Copilot_v1_0.md), [`../AI NOTES/2026-05-11_Phase_D2_GPIOF_Sweep_Results_Copilot_v1_0.md`](../AI%20NOTES/2026-05-11_Phase_D2_GPIOF_Sweep_Results_Copilot_v1_0.md), [`../AI NOTES/2026-05-11_Phase_D3_GPIOD_Sweep_Results_Copilot_v1_0.md`](../AI%20NOTES/2026-05-11_Phase_D3_GPIOD_Sweep_Results_Copilot_v1_0.md); upstream BSP [`ov_carrier_enuc_lora.dts`](https://raw.githubusercontent.com/arduino/meta-arduino/scarthgap/meta-arduino-nxp/recipes-bsp/device-tree/arduino-device-tree/portenta-x8/overlays/ov_carrier_enuc_lora.dts).

---

## Phase 0 — Hardware procurement & shop setup

### Tractor node hardware

- [ ] Order Arduino Portenta Max Carrier (ABX00043)
- [ ] **Order Arduino Portenta X8 (ABX00049)** — same SKU as base station. Canonical per [MASTER_PLAN.md §8.9](MASTER_PLAN.md); the X8's onboard STM32H747 co-MCU runs the realtime M7+M4 firmware bare-metal, and the i.MX 8M Mini side gives Linux/SSH/MIPI camera.
- [ ] Order LoRa SMA whip antenna (915 MHz, 3 dBi) — ANT-8/9-IPW1-SMA or equivalent
- [ ] Order cellular SMA antenna (700–2700 MHz)
- [ ] Order **SparkFun NEO-M9N Qwiic GPS — SMA variant (GPS-15733, ~$75)** + panel-mount SMA bulkhead (~$5) + a second 50 mm Qwiic cable (~$1). Daisy-chains off the BNO086 on the same MCP2221A bus, so no extra USB port or driver work. Onboard SMA jack means the cab-roof antenna feedline runs straight to the enclosure wall — no U.FL pigtail. See [HARDWARE_BOM.md Tier 1](HARDWARE_BOM.md#tier-1--tractor-node-portenta-max-carrier--portenta-x8--arduino-opta).
- [ ] Order cab-roof antenna mounting bracket + N-bulkhead pass-through
- [ ] Activate Cat-M1 IoT SIM card (Hologram, Soracom, or equivalent)
- [ ] Order industrial microSD 32 GB
- [ ] Order EMI filter / ferrite beads for 12 V input
- [ ] Order master battery cutoff switch (Blue Sea or equivalent)
- [ ] Order ATC blade fuse holder kit + assorted fuses (5/10/20 A)
- [ ] Order 18650 LiPo cell (3500 mAh, protected) + TP4056 charger module
- [ ] **Order Arduino Opta WiFi (AFX00002)** — industrial PLC, runs Modbus slave for valve I/O
- [ ] **Order Opta Ext D1608S** expansion (8× relay outputs + 8× digital inputs)
- [ ] **Order Opta Ext A0602** expansion (6× analog inputs + 2× 0–10 V analog outputs for Burkert 8605)
- [ ] Order DIN rail 35 mm × 300 mm + end stops for in-enclosure mounting
- [ ] Order RS-485 cable + 120 Ω termination resistors (×2) for Max Carrier J6 ↔ Opta link
- [ ] Order **engine-kill automotive relay** (30 A SPDT, e.g. Bosch 0332019150) — wired through an Opta D1608S relay channel
- [ ] Order **Phoenix Contact PSR safety relay** (PSR-MC38 or equivalent dual-channel monitored)
- [ ] Order signal-conditioning components for the analog inputs already covered by Opta A0602 (only needed if external transducers require scaling beyond A0602's 0–10 V / 4–20 mA ranges)
- [ ] Order status LED panel (3× 12 V panel-mount: POWER / LINK / FAULT)
- [ ] Order 12 V piezo buzzer (panel-mount)
- [ ] Order Deutsch DT connector kit + crimp tool (rent or buy)
- [ ] Order IP65 enclosure ~250×200×100 mm + rubber-bobbin vibration mounts
- [ ] Order conformal coating spray (MG Chemicals 419 acrylic or equivalent)
- [ ] Order cable glands (M16/M20 IP68, ×6)

### Tractor-side sensors and cameras

See the sensor/cabling discussion in [HARDWARE_BOM.md § Notes on substitutions](HARDWARE_BOM.md#notes-on-substitutions) and [TRACTOR_NODE.md § Telemetry sources](TRACTOR_NODE.md). USB UVC is the easy path — the Linux mainline `uvcvideo` driver enumerates any UVC webcam as `/dev/video0` with no custom driver work; MIPI CSI is faster/lower-latency but requires Yocto device-tree overlays so it lands in Phase 2 once the USB pipeline is proven.

- [ ] Order **USB UVC webcam** for first-light video — Logitech C920 (onboard H.264 hw-encode, ~$70) or ELP-USBFHD06H (M12, IP67, ~$90 if you need a sealed connector). UVC = no driver install on the X8 Yocto image.
- [ ] Order **panel-mount USB-A bulkhead** + cable gland to bring the webcam cable through the IP65 enclosure wall (consumer USB cables are not field-sealed).
- [ ] Order **Adafruit MCP2221A breakout (4471, ~$8)** — USB ↔ I²C bridge with Stemma QT/Qwiic. Mainline `hid-mcp2221` driver, no install on the X8 Yocto image. Hosts the IMU on the **Linux side**, not the M7.
- [ ] Order **SparkFun Qwiic 9DoF IMU — BNO086 (DEV-22857, ~$30)** with on-chip sensor fusion (quaternion straight out, no Madgwick/Mahony work). Used for tip-over warning, heading hold, vibration logging.
- [ ] Order **Qwiic / Stemma QT 50 mm cable** for MCP2221A → BNO086 link.
- [ ] *(Production-build hardening)* Order **USB galvanic isolator** (ADuM3160-based, ~$25) for inline use between the X8 USB port and the MCP2221A in the noisy tractor enclosure. Optional for bench bring-up.
- [ ] Order **2× hydraulic pressure sensors**, 0–250 bar 4–20 mA output with M12-A 4-pin cordsets (~$40 each). Wires into Opta A0602 analog inputs (`hyd_supply_psi` 0x0103 / `hyd_return_psi` 0x0104 per [TRACTOR_NODE.md Modbus map](TRACTOR_NODE.md#modbus-rtu-register-map-max-carrier--opta)).
- [ ] *(Phase 2, optional)* Order **MIPI CSI camera** — Arducam IMX219/IMX477 with 22-pin FFC ribbon (~$30–60). Requires building a Yocto image with the camera device-tree overlay enabled; track separately in `RESEARCH-CONTROLLER/VIDEO_COMPRESSION/`.
- [ ] *(Phase 2, optional)* Order **PoE IP camera** (Reolink RLC-510A or equivalent) if night vision / IR / single-cable PoE is wanted — pulled by FFmpeg/GStreamer from RTSP on the X8.

### Industrial cabling and connectors

- [ ] Order **Deutsch DT04-5P connector kit** + crimp tool for engine CAN harness (already covered above for the generic DT kit; verify 5-pin J1939 variant is in the kit).
- [ ] Order **M12-A 5-pin cordsets** (TURCK or Phoenix Contact, preassembled, 2 m) for RS-485 master ↔ Opta — preferred over field-terminated cable. Quantity 2.
- [ ] Order **M12-A 4-pin cordsets** for the 2× hydraulic pressure sensors (matching the sensor side).
- [ ] Order **M12 X-coded → RJ45 field plug** + Cat6a cable for IP camera or dev laptop Ethernet drop into the enclosure.
- [ ] Order **Hammond 1554/1555 polycarbonate IP66 enclosure** (or upgrade the existing IP65 spec) sized to fit Max Carrier + Opta + D1608S + A0602 + IMU breakout on a single DIN rail.
- [ ] Order **inline 5 A automotive blade fuse holder + TVS diode** (e.g. SMBJ33A) for the 12/24 V power feed — load-dump protection.

### Base station hardware

- [ ] Order second Arduino Portenta Max Carrier (ABX00043)
- [ ] Order Arduino Portenta X8 (ABX00049)
- [ ] Order 8 dBi 915 MHz omni mast antenna (L-com or equivalent)
- [ ] Order LMR-400 coax cable (3 m, SMA-M to N-M)
- [ ] Order N-female bulkhead lightning arrestor
- [ ] Order ~3 m galvanized mast + ground rod
- [ ] Order second cellular SMA antenna + activate second Cat-M1 SIM
- [ ] Order indoor 12 V / 5 A power supply
- [ ] Order mini UPS (12 V, 30 min runtime)
- [ ] Order indoor ventilated enclosure
- [ ] Order Cat6 Ethernet cable (5 m)
- [ ] **Order [Coral Mini PCIe Accelerator](https://coral.ai/products/pcie-accelerator/) (~$25)** — *strongly recommended optional* per [MASTER_PLAN.md §8.19](MASTER_PLAN.md). Unproven on the Max Carrier — the Phase 1 validation spike below decides whether this stays in the BOM, gets swapped for the Coral USB Accelerator, or is dropped entirely.
- [ ] *(Spike-conditional)* Order [Coral USB Accelerator](https://www.digikey.com/en/products/detail/coral/G950-01456-01/10256669) (~$60) **only if** the Mini PCIe spike fails
- [ ] *(If Coral fitted)* Order ~15×15×8 mm stick-on aluminium heatsink for the Edge TPU package
- [ ] **2-day validation spike (gate before BOM lock):** install Coral Mini PCIe in the Max Carrier; confirm PCIe enumeration in `lspci`, `gasket`/`apex` driver loads against the X8 Yocto kernel, sustained 30-min inference without thermal throttle, slot power budget adequate. Document outcome in [HARDWARE_BOM.md](HARDWARE_BOM.md). On failure, repeat with the USB Accelerator; on second failure, ship CPU-only and degrade the image pipeline per [MASTER_PLAN.md §8.19](MASTER_PLAN.md).

### Handheld hardware

- [ ] Order Arduino MKR WAN 1310 (ABX00029)
- [ ] Order 1/4-wave 915 MHz whip antenna with u.FL or pigtail
- [ ] Order dual-axis analog joysticks (×2)
- [ ] Order 22 mm latching mushroom E-stop
- [ ] Order momentary push buttons (×4, IP67 tactile)
- [ ] Order SSD1306 OLED 128×64 I²C
- [ ] Order 1S 2000 mAh LiPo battery
- [ ] Order USB-C panel-mount breakout
- [ ] Order IP54 handheld enclosure (Hammond/Polycase ~120×80×40)
- [ ] Design custom PCB for joystick interface to MKR (KiCad)
- [ ] Order PCB from OSHPark or JLCPCB (5-pack)

### Development gear

- [ ] Order RTL-SDR USB receiver (NooElec) for LoRa packet sniffing
- [ ] Order Saleae Logic 8 (or clone) for latency measurement
- [ ] Identify spectrum analyzer rental/borrow source for FCC EIRP verification
- [ ] Order bench power supply (0–30 V, 0–5 A) if not already in shop
- [ ] Order 50 Ω SMA dummy loads (×2, 5 W rated)

### Spare parts (highly recommended)

- [ ] Order ×2 of each: Max Carrier, MKR WAN 1310, Portenta X8
- [ ] Order ×2 spare LoRa antennas, cellular antennas, joysticks, OLEDs

---

## Phase 1 — Bench bring-up

**2026-05-04 bench status:** two Portenta X8 + Max Carrier stacks have
been flashed with the `tractor_h7` M7 image at `0x08040000`. Both M7
cores reached `loop()` and advanced SRAM4 liveness with CFSR/HFSR = 0
using the X8 no-USB, bit-banged-SPI, nonce-PRNG bring-up image. This is
partial W4-pre evidence only; USB-CDC, rail, blink/echo, stock M7<->M4
handshake, and W4-00 TX/RX burst evidence are still open. Newer radio
diagnostics enqueue frames but fail before TX start (`stageMode(TX)`
returns `-16`; direct SX127x register snapshots are zero), so the Max
Carrier LoRa interface must be revalidated before W4-00. Details:
[`../AI NOTES/2026-05-04_Portenta_X8_M7_W4_Pre_Bringup_Status.md`](../AI%20NOTES/2026-05-04_Portenta_X8_M7_W4_Pre_Bringup_Status.md).

- [ ] Set up shared Git repo with subfolders for `firmware/handheld_mkr`, `firmware/tractor_h7` (runs on the X8's onboard STM32H747 co-MCU), `firmware/tractor_opta`, `firmware/tractor_x8` (Linux services), `firmware/common`, `base_station/` (Docker compose root). Per [MASTER_PLAN.md §8.2](MASTER_PLAN.md), there is **no `firmware/base_*/` target**. 2026-05-04 interface update: revalidate the original "Linux drives SX1276 directly over SPI" assumption for the Max Carrier; Arduino's X8 gateway example drives the onboard Murata LPWAN module as an AT modem on `/dev/ttymxc3` with reset on `/dev/gpiochip5` line 3.
- [ ] Set up PlatformIO with Portenta X8 + MKR WAN 1310 + Opta board support
- [ ] Set up Arduino-CLI CI (extend existing [ARDUINO_CI.md](ARDUINO_CI.md) setup)
- [ ] **Tractor:** verify Max Carrier + X8 boots, M7 blink works on the X8's onboard H747 co-MCU, M4 blink works
- [ ] **Tractor:** verify LoRa TX/RX from M7 with RadioLib at 915 MHz, **SF7 / BW 250 kHz / CR 4-5** per [LORA_PROTOCOL.md](LORA_PROTOCOL.md) / [DECISIONS.md](DECISIONS.md) D-A2. 
  **2026-05-04 update:** Definitive confirmation received: The Max Carrier physically does not route the Murata module's SPI pins. It acts purely as a UART AT modem (19200 or 115200 baud). Raw `RadioLib` SPI control is impossible. The tractor M7 firmware must be refactored to use `Serial3` (or the equivalent Linux X8 serial endpoint) and AT commands (e.g. `AT`, `AT+VER?`, `AT+APPEUI`, etc).
  **2026-05-09 update:** This item is superseded by Method G custom firmware on the Murata L072. The qualified path is now X8/H7 → AN3155 UART bootloader → custom [`murata_l072/`](firmware/murata_l072/) firmware at `921600 8N1`, not RadioLib on exposed SPI and not production AT control. Phase 0 is green after a **20/20 PASS** flash stress run. The next hardware gate is Phase 1 Stage 1 bring-up: flash the custom binary, capture `BOOT_URC`, and prove live SX1276 register access using [`firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py`](firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py). X8-side one-shot runner: [`firmware/x8_lora_bootloader_helper/run_method_g_stage1.sh`](firmware/x8_lora_bootloader_helper/run_method_g_stage1.sh). Windows host launcher: [`firmware/x8_lora_bootloader_helper/run_method_g_stage1_end_to_end.ps1`](firmware/x8_lora_bootloader_helper/run_method_g_stage1_end_to_end.ps1). Windows build helper: [`firmware/murata_l072/build.ps1`](firmware/murata_l072/build.ps1). Current launcher also pulls Stage 1 logs into [`bench-evidence/`](bench-evidence/) for the run.
  **2026-05-09 Stage 1 run result:** Custom `murata_l072` firmware now builds on Windows and stages to the X8 automatically. The STM32 ROM bootloader still NACKs both global **Extended Erase** and global **Standard Erase** payloads, but the helper flasher now succeeds by erasing only the required flash pages. The flash/boot path itself is therefore validated: a direct `hello.bin` run through the same helper flow produced the expected hello-world output on `/dev/ttymxc3`. The remaining blocker is the custom firmware boot path, not AN3155 flashing: after a successful flash and user-app boot pulse, the custom image produces no `BOOT_URC`, no `VER_URC`, and no ASCII AT fallback response at `921600 8N1`. Until that silence is explained, the W1-7 gate cannot pass. Evidence captured under [`bench-evidence/`](bench-evidence/).
  **2026-05-09 Stage 1 delta update:** Unified flash-layout remediation has been applied (`memory_map.h`, linker, startup/static asserts), and an additional startup blocker was fixed: STM32L072 HSI16 `RCC_CR` bit definitions were corrected to `HSION=bit0` and `HSIRDY=bit2`. Rebuilt firmware now shows deterministic post-boot serial output at `19200 8N1` during the pre-probe window (instead of full silence), proving execution reaches startup diagnostics. W1-7 remains open because the host binary protocol at `921600 8N1` is still not observed (`BOOT_URC` timeout); current work is focused on startup-to-host-UART handoff timing/framing.
  **2026-05-09 Stage 1 rerun (post-beacon-timing patch):** startup beacon duration was reduced so diagnostics do not hold the line for tens of seconds before protocol bring-up. W1-7 still fails (`BOOT_URC` timeout), and this rerun captured no bytes during the pre-probe `19200 8N1` window. Latest evidence folder: [`bench-evidence/T6_bringup_2026-05-09_131716/`](bench-evidence/T6_bringup_2026-05-09_131716/). Immediate next action is explicit startup-to-`host_uart_init()` transition instrumentation.
  **2026-05-09 Stage 1 rerun (dual-UART host fallback):** `host_uart` now initializes both Murata-side UART candidates (`LPUART1` and `USART1`) concurrently, mirrors outgoing bytes on both, and ingests RX IRQ bytes from both to eliminate PA2/PA3 vs PA9/PA10 routing uncertainty. Bench rerun still fails W1-7 with unchanged probe behavior: no `BOOT_URC`, no `VER_URC`, no ASCII AT fallback bytes, and timeout waiting for response type `0x81` at `921600 8N1` after successful flash + boot pulse. Evidence: [`bench-evidence/T6_bringup_2026-05-09_132507/`](bench-evidence/T6_bringup_2026-05-09_132507/) (manual Stage 1 log and pulled OpenOCD/flash logs).
  **2026-05-09 Stage 1 rerun (ordered breadcrumbs + early split UART capture):** firmware instrumentation was added around `safe_mode_listen`, `platform_clock_init_hsi16`, `host_uart_init`, and `host_cmd_init`, and the X8 harness now captures the UART immediately after reset at both `19200` and `921600`. W1-7 still fails, but the failure signature is sharper: earliest `19200 8N1` capture shows non-text garbage, early `921600 8N1` capture shows a burst of `0x00` bytes only, and by the time the active probe opens there is no valid `BOOT_URC`, no `VER_URC`, and no ASCII fallback response. Evidence: [`bench-evidence/T6_bringup_2026-05-09_133825/`](bench-evidence/T6_bringup_2026-05-09_133825/). Immediate next action is to inspect the first transmit path after `host_uart_init` for a zero-byte flood / TX-low condition rather than continuing to treat the symptom as total silence.
  **2026-05-09 Stage 1 rerun (decoded startup frames, unresolved host->Murata RX):** Harness corrections are now confirmed with decoded wire evidence: early capture contains `FAULT_URC(0xF1)` + `BOOT_URC(0xF0)` + `FAULT_URC(0xF1)`, so startup and `host_cmd_init()` are executing and TX from Murata to X8 is proven. `BOOT_URC` payload indicates `radio_ok=0`, `radio_version=0x00`, protocol/schema version 1, and `clock_source_id=1` (HSI fallback). Remaining blocker is active request path only: `VER_REQ` still times out waiting for `VER_URC`; ASCII fallback also observes no bytes. A targeted firmware change was tested in `firmware/murata_l072/host/host_uart.c` to disable `USART1` RX/IRQ (retain TX mirror, LPUART1 RX only) to eliminate double-ingest possibility; behavior did not change on bench. Current priority is proving whether X8 TX bytes are physically/logically reaching LPUART1 RX after probe open/flush.
  **2026-05-10 Stage 1 (ROOT CAUSE IDENTIFIED & FIXED):** After detailed code audit against STM32L072 RM0367 vector table, identified definitive root cause of RX silence: **interrupt numbers in stm32l072_regs.h and vector table were off by one**. STM32L072 has IRQ19=RESERVED (no peripheral), but firmware defined USART1_IRQn=26U (should be 27U), USART2_IRQn=27U (should be 28U), RNG_LPUART1_IRQn=28U (should be 29U). This caused `platform_irq_enable(USART1_IRQn)` to enable NVIC bit 26 (SPI2) instead of bit 27 (USART1), so UART RX interrupts were masked. Additionally, `startup.c` vector table jumped directly from `TIM7_IRQHandler` to `TIM21_IRQHandler` with no reserved slot, shifting all subsequent handlers left by one position. **Fixes applied (all committed):**
    1. `stm32l072_regs.h` lines 21–22: Fixed RCC bits HSION→(1<<0) and HSIRDY→(1<<2)
    2. `stm32l072_regs.h` lines 232–234: Fixed USART1/2/LPUART1 IRQ numbers (+1 each)
    3. `startup.c` vector table: Added `0,` reserved entry between TIM7 and TIM21
    4. `host_uart.c` RNG_LPUART1_IRQHandler: Moved error check BEFORE RXNE loop (prevents corrupted bytes from reaching COBS decoder)
    5. `host_uart.c` HOST_UART_RX_ECHO_DIAG: Disabled echo to eliminate TX contention
  **Rebuild result:** Firmware 14492 bytes (clean compile, no errors). Flash verified PASS via STM32 AN3155 bootloader (11 s completion). Boot pulse confirmed via OpenOCD (PA11 LOW → NRST pulse → firmware runs).
  **Current status (2026-05-10 post-fix):** Flash + boot PASS; early `19200 8N1` capture confirms startup diagnostics execute (no longer all-zero/garbage); SPI/AT fallback testing confirms X8↔L072 UART path is electrically open and firmware responds to command bytes (ATI echoed, AT+VER? echoed back). **Binary VER_REQ probe still times out on VER_URC.** Investigation revealed root cause: **SX1276 SPI initialization returns 0x00 from version register read**, causing `sx1276_init()` to fail and emit FAULT_URC(0xF1). This is a separate issue from the UART RX interrupt fix. Secondary diagnostics added to track frame flow:
    - `C:SEND_BOOT` / `C:SEND_FAULT` / `C:SEND_READY` / `C:INIT_DONE` in host_cmd_init()
    - `C:DISPATCH` / `C:BAD_VER` / `C:VER_DISP` in host_cmd_dispatch()
    - `H:COBS_ERR` / `H:PARSE_ERR` / `H:VER_REQ_RX` / `H:FRAME_OK` / `H:Q_FULL` in process_encoded_frame()
    - `H:PROC_FRAME` when frame delimiter (0x00) triggers processing
  **Temporary radio init bypass applied:** Firmware modified to continue past radio_ok=false so binary protocol dispatcher can be tested independently. Firmware 14692 bytes with bypass and diagnostics, pushed to X8.
  **Next immediate action:** Determine root cause of SX1276 SPI read returning 0x00: (a) SPI clock not configured correctly for L072, (b) NSS/CS pin not asserted/deasserted properly, (c) SX1276 not powered or in sleep state, (d) GPIO AFI settings wrong for SPI1 PA5/PA6/PA7. Investigation blocked without access to oscilloscope/logic analyzer to probe SPI bus. Alternative: disable radio init fully and test VER_REQ probe to confirm binary protocol path works, then return to radio debugging with full context.
  **2026-05-09 Stage 1 rerun (RX observability instrumentation):** Added additive host stats counters for RX ingress (`HOST_RX_BYTES`, lane split, parse_ok/parse_err, per-UART error counters) and an unsolicited `STATS_URC` snapshot emitted when RX-byte count changes. Rebuilt and reran Stage 1 twice (`bench-evidence/T6_bringup_2026-05-09_144749/`, `bench-evidence/T6_bringup_2026-05-09_144935/`). Outcome is unchanged at probe level (`VER_REQ` timeout; no ASCII fallback bytes), and no unsolicited `STATS_URC` appears in early capture despite startup TX frames. This strongly points to **no host->Murata ingress at UART ISR level** during active probe traffic, rather than parser-only drop. Detailed writeup: [../AI NOTES/2026-05-09_Controller_Stage1_RX_Observability_Copilot_v1_1.md](../AI%20NOTES/2026-05-09_Controller_Stage1_RX_Observability_Copilot_v1_1.md).
  **2026-05-09 Stage 1 rerun (forced heartbeat stats beacon):** Main-loop diagnostics now emit unconditional `STATS_URC` beacons every 250 ms for the first 6 s after `host_cmd_init`, independent of RX-byte deltas. New bench run [`bench-evidence/T6_bringup_2026-05-09_145307/`](bench-evidence/T6_bringup_2026-05-09_145307/) captured `STATS_URC(init): host_rx_bytes=0 host_parse_ok=0 host_parse_err=0`, then the same active-probe failure (`VER_REQ` timeout, no ASCII fallback replies). This confirms the firmware is alive and transmitting diagnostic URCs while ingress counters stay at zero, further narrowing the blocker to **X8 TX path / routing into Murata RX**, not request-dispatch logic. Detailed writeup: [../AI NOTES/2026-05-09_Controller_Stage1_RX_Heartbeat_Classification_Copilot_v1_2.md](../AI%20NOTES/2026-05-09_Controller_Stage1_RX_Heartbeat_Classification_Copilot_v1_2.md).
  **2026-05-09 Stage 1 rerun (lane/error counter confirmation):** Probe output now prints lane-split RX and UART error counters from `STATS_URC(init)`. Latest run [`bench-evidence/T6_bringup_2026-05-09_145434/`](bench-evidence/T6_bringup_2026-05-09_145434/) shows `host_rx_lpuart=0`, `host_rx_usart1=0`, `uart_err_lpuart=0`, `uart_err_usart1=0` together with `host_rx_bytes=0` and unchanged `VER_REQ` timeout. That confirms both candidate ingress lanes remain idle at ISR accounting level during active probe traffic.
  **2026-05-09 Stage 1 rerun (ISR-latched RX-seen marker):** Added an ISR-near lane latch in `firmware/murata_l072/host/host_uart.c` and main-loop one-shot fault emission (`HOST_FAULT_CODE_HOST_RX_SEEN`) so first-byte ingress is signaled even if frame parsing fails. Probe FAULT logging was extended to decode this marker and print lane names. Fresh run [`bench-evidence/T6_bringup_2026-05-09_145716/`](bench-evidence/T6_bringup_2026-05-09_145716/) still reports `STATS_URC(init): host_rx_bytes=0 host_rx_lpuart=0 host_rx_usart1=0 host_parse_ok=0 host_parse_err=0 uart_err_lpuart=0 uart_err_usart1=0` followed by `VER_REQ` timeout; no `HOST_RX_SEEN` FAULT marker appears during the active probe window. This further supports the current blocker classification: no host->Murata ingress reaches UART RX ISR/accounting during active queries.
  **2026-05-09 Stage 1 rerun (strict ROM-vs-user A/B + IRQ echo diagnostic):** A strict A/B sync test with identical line settings (`19200 8E1`, send `0x7F`) now confirms boot-state-dependent path behavior: in ROM state (BOOT0 high) `/dev/ttymxc3` receives `0x79` ACK, while in user-app state (BOOT0 low + NRST) the same probe returns zero bytes. To remove parser ambiguity, a diagnostic build was flashed with IRQ-level raw RX echo enabled in `host_uart.c`; user-mode probe sent pattern `c35aa71e33cc` at `921600 8N1` and received 475 bytes of firmware output but **no echoed pattern** (`pattern_found=False`). This materially strengthens the current blocker classification: host TX bytes are still not reaching active L072 RX in user mode even with RX-echo-at-IRQ instrumentation. Reference notes: [../AI NOTES/2026-05-09_ROM_vs_User_AB_19200_8E1_and_RX_Echo_Diagnostic_Copilot_v1_0.md](../AI%20NOTES/2026-05-09_ROM_vs_User_AB_19200_8E1_and_RX_Echo_Diagnostic_Copilot_v1_0.md), [../AI NOTES/2026-05-09_RX_Echo_Diagnostic_Run_and_Chip_Document_Checklist_Copilot_v1_0.md](../AI%20NOTES/2026-05-09_RX_Echo_Diagnostic_Run_and_Chip_Document_Checklist_Copilot_v1_0.md).
  **2026-05-09 Stage 1 rerun (PA11 post-boot HIGH test):** To test the remaining OE hypothesis, user firmware was booted normally (`08_boot_user_app.cfg`), then `PA_11` was forced HIGH in a second OpenOCD invocation **without resetting L072**. OpenOCD confirmed write success (`PA11 forced HIGH, GPIOA_IDR=0x0000c800`). Re-running the IRQ-echo probe at `921600 8N1` still produced `pattern_found=False` (normal URC traffic present, no echoed injected bytes). Conclusion: `PA_11` alone is not sufficient to restore host->L072 ingress in user mode; the blocker likely involves another control net or board-level routing dependency. Reference note: [../AI NOTES/2026-05-09_PA11_PostBoot_High_Test_Copilot_v1_0.md](../AI%20NOTES/2026-05-09_PA11_PostBoot_High_Test_Copilot_v1_0.md).
  **2026-05-09 documentation organization update:** Per-chip document vault created at [../AI NOTES/CHIP-DOCS/README.md](../AI%20NOTES/CHIP-DOCS/README.md) with folders and templates for `STM32L072`, `STM32H747`, `IMX8MM_UART4`, `SX1276`, and `MURATA_CMWX1ZZABZ_078` to track authoritative datasheets/schematics, confirmed findings, and open hardware-path questions.
  **2026-05-09 documentation source update:** Official Arduino ABX00043 source links were logged in [../AI NOTES/CHIP-DOCS/source_log.md](../AI%20NOTES/CHIP-DOCS/source_log.md), including direct URLs for Max Carrier schematics, datasheet, and full pinout PDFs to support net-level UART route extraction.
  **2026-05-10 Stage 1 rerun (SPI/reset hardening baseline + USART1 ingress fix attempt):** Fresh full run [`bench-evidence/T6_bringup_2026-05-10_155326/`](bench-evidence/T6_bringup_2026-05-10_155326/) on the 14692-byte image (SPI prescaler slowed + reset pulse widened + radio bypass diagnostics) still fails W1-7: early `921600` capture shows `FAULT/BOOT/READY` and `C:SEND_*` traces, but active probe times out on `VER_REQ` with no ASCII fallback replies. A targeted follow-up fix was applied in [`firmware/murata_l072/host/host_uart.c`](firmware/murata_l072/host/host_uart.c): `USART1_IRQHandler` now mirrors `LPUART1` error-first behavior (clear PE/FE/NE/ORE + discard errored byte before decode) to prevent COBS state corruption on the mirrored lane. Post-fix launcher invocations created metadata folders [`bench-evidence/T6_bringup_2026-05-10_155445/`](bench-evidence/T6_bringup_2026-05-10_155445/) and [`bench-evidence/T6_bringup_2026-05-10_155534/`](bench-evidence/T6_bringup_2026-05-10_155534/) but did not pull stage logs (`stage1_stdout.txt` absent), so blocker closure is still pending another full captured run.
  **2026-05-10 Stage 1 rerun (one-shot ingress telemetry classification):** Added bounded one-shot diagnostic emits in [`firmware/murata_l072/main.c`](firmware/murata_l072/main.c) using existing ISR latches/counters: `HOST_RX_SEEN`, `HOST_RX_INACTIVE`, `HOST_PARSE_ERROR`, and `HOST_DIAG_MARK`, each paired with `host_cmd_emit_stats_snapshot()`. Full captured rerun [`bench-evidence/T6_bringup_2026-05-10_160014/`](bench-evidence/T6_bringup_2026-05-10_160014/) still fails W1-7 (`VER_REQ` timeout), but the early `921600` burst now classifies the failure mode explicitly: decoded frames include `FAULT_URC code=0x0A (HOST_RX_INACTIVE)` plus `STATS_URC host_rx_bytes=0 host_parse_err=0 host_errors=0`. No `HOST_RX_SEEN` and no parser/dispatch diag marks were observed in-window. This tightens the blocker to **no host->Murata ingress observed during active probe traffic** (pre-dispatch), not COBS/parser corruption on received frames.
  **2026-05-10 Stage 1 rerun (RX polling fallback in firmware):** `host_uart_poll_dma()` in [`firmware/murata_l072/host/host_uart.c`](firmware/murata_l072/host/host_uart.c) was upgraded from a no-op to an IRQ-masked RXNE/error polling path for both LPUART1 and USART1, reusing the ISR ingest logic as a fallback when IRQ delivery is suspect. Full captured rerun [`bench-evidence/T6_bringup_2026-05-10_160521/`](bench-evidence/T6_bringup_2026-05-10_160521/) still fails W1-7 with unchanged signature (`VER_REQ` timeout, no ASCII fallback replies). Early `921600` diagnostics remain consistent with `HOST_RX_INACTIVE` classification and no host ingress counters increasing.
  **2026-05-10 Stage 1 rerun (X8 probe device sweep):** The X8 helper runner [`firmware/x8_lora_bootloader_helper/run_method_g_stage1.sh`](firmware/x8_lora_bootloader_helper/run_method_g_stage1.sh) now sweeps `/dev/ttymxc3 /dev/ttymxc2 /dev/ttymxc1 /dev/ttymxc0` for active `method_g_stage1_probe.py` queries instead of hard-coding one device. Captured rerun [`bench-evidence/T6_bringup_2026-05-10_160705/`](bench-evidence/T6_bringup_2026-05-10_160705/) shows `/dev/ttymxc3`, `/dev/ttymxc2`, and `/dev/ttymxc1` all timing out on `VER_REQ` (`probe_rc=2`), while `/dev/ttymxc0` fails open/configure (`stty: Inappropriate ioctl for device`). Net: W1-7 remains blocked; no responding binary host endpoint found in this sweep.
  **2026-05-10 Stage 1 rerun (serial-usable filtering + expanded endpoint discovery):** [`firmware/x8_lora_bootloader_helper/run_method_g_stage1.sh`](firmware/x8_lora_bootloader_helper/run_method_g_stage1.sh) now pre-validates probe candidates with `stty -F <dev> 921600 ...` and skips non-serial nodes before launching active queries. Endpoint auto-discovery has been expanded to include live `/dev/ttymxc*`, `/dev/ttyX*`, `/dev/ttyUSB*`, and `/dev/ttyACM*` nodes. Full captured rerun [`bench-evidence/T6_bringup_2026-05-10_161139/`](bench-evidence/T6_bringup_2026-05-10_161139/) confirms: `/dev/ttymxc3`, `/dev/ttymxc2`, `/dev/ttymxc1`, and `/dev/ttyX0` each timeout on `VER_REQ` (`probe_rc=2`), `/dev/ttymxc0` is correctly skipped as `not usable serial tty`, and `probe_selected_dev=none`. Net: filtering + broader endpoint coverage removed false candidates and tested `ttyX0`, but W1-7 remains blocked with no responding binary host endpoint discovered.
  **2026-05-10 Stage 1 rerun (115200 baud discriminator):** A full discriminator run was executed with a temporary 115200 user-mode host baud variant (`HOST_BAUD_DEFAULT=115200`) and matching probe baud updates, then restored to the baseline 921600 configuration after the test. Captured evidence [`bench-evidence/T6_bringup_2026-05-10_171207/`](bench-evidence/T6_bringup_2026-05-10_171207/) shows unchanged failure behavior across `/dev/ttymxc3`, `/dev/ttymxc2`, `/dev/ttymxc1`, and `/dev/ttyX0`: `VER_REQ` timeout, no ASCII fallback bytes (`ATI`/`AT+VER?`), and `probe_selected_dev=none`. The early capture still contains outbound firmware traffic (`FAULT/BOOT/READY`, `C:SEND_*`), but no host ingress indicators. Net: lowering from 921600 to 115200 does **not** restore host->L072 ingress, so a simple high-baud bandwidth limit is unlikely to be the primary blocker.
  **2026-05-10 control-line experiment (RTS/CTS toggle on `/dev/ttymxc3`):** A fresh Stage 1 baseline rerun was captured first in [`bench-evidence/T6_bringup_2026-05-10_183644/`](bench-evidence/T6_bringup_2026-05-10_183644/), then a targeted UART control-line check was run in [`bench-evidence/T6_ctrlline_uart4_2026-05-10_183642/`](bench-evidence/T6_ctrlline_uart4_2026-05-10_183642/). Findings:
    1. `diag_uart_rxtx.py` at `921600` with `--hwflow off` still produced no replies to `ATI`, `VER_REQ`, or `PING_REQ` ([`diag_hwflow_off.txt`](bench-evidence/T6_ctrlline_uart4_2026-05-10_183642/diag_hwflow_off.txt)).
    2. Repeating with `--hwflow on` showed the same no-reply behavior and explicit `stty` warning (`unable to perform all requested operations`) ([`diag_hwflow_on.txt`](bench-evidence/T6_ctrlline_uart4_2026-05-10_183642/diag_hwflow_on.txt)).
    3. Direct `stty` probe confirms Linux cannot enable `crtscts` on `/dev/ttymxc3`; mode remains `-crtscts` before and after attempted toggle ([`stty_crtscts_probe.txt`](bench-evidence/T6_ctrlline_uart4_2026-05-10_183642/stty_crtscts_probe.txt)).
    4. Low-level UART4 register snapshots via `uart4_regs2.py` failed on this image (`OSError: [Errno 14] Bad address`), and `devmem` is unavailable, so hardware register-level CTS/RTS state could not be sampled in this run ([`uart4_regs_before.txt`](bench-evidence/T6_ctrlline_uart4_2026-05-10_183642/uart4_regs_before.txt)).
  **Conclusion:** This run did not show evidence of a host-side termios `crtscts` gating issue; the driver refuses CRTSCTS mode and ingress failure persists identically. Next step is board-level routing/control-net isolation (mux/OE/select path) rather than further termios-only toggles.
  **2026-05-10 board-level lane isolation sweep (H747 forced GPIO clusters):** Executed coherent low/high forcing across three candidate control clusters using OpenOCD lane cfgs (`34..39`) with per-case active probe + UART diagnostics. Evidence: [`bench-evidence/T6_lane_isolation_2026-05-10_184524/`](bench-evidence/T6_lane_isolation_2026-05-10_184524/). Key findings:
    1. OpenOCD IDR readbacks confirm forced states were actually applied electrically in every case (low cfgs read `0`, high cfgs read `1` on targeted pins).
    2. Across all six states, `method_g_stage1_probe.py` still times out on `VER_REQ` (`VER_URC` absent) and `AT+VER?` still shows no response.
    3. Unlike prior pure-silence signatures, `ATI` now returns non-empty zero-heavy payloads in all six forced states (~141-146 bytes), which indicates the route is perturbable by these control clusters but not yet restored to valid command ingress.
  **Conclusion:** The blocker remains pre-dispatch host->L072 ingress in user mode, but this sweep materially narrows the search: board-level control nets are coupled to behavior. Next action is a mixed-state (bit-flip) matrix with reset-order control on the most plausible lane clusters, not only all-high/all-low forcing.
  **2026-05-10 follow-up (lane-B mixed-state + reset-order matrix):** Executed reset-ordered mixed-state cases in [`bench-evidence/T6_laneB_mixed_resetorder_2026-05-10_185139/`](bench-evidence/T6_laneB_mixed_resetorder_2026-05-10_185139/) using new cfgs `40..49` (plus targeted repeats for `43/44`). Lane forcing integrity is confirmed by per-case IDR readbacks matching intended bit patterns. Across all cases, `VER_REQ` still times out (`VER_URC` absent) and `AT+VER?` remains no-response. Most cases continue to show zero-heavy ATI payloads (~141-144 bytes). Two single-high cases (`PB11`-only and `PC9`-only) briefly showed ATI silence in one pass but did **not** reproduce on immediate repeat (`r2` logs restored ATI bytes), so that discriminator is currently non-deterministic.
  **Conclusion:** Mixed-state + reset-order testing strengthens the board-level perturbation classification but does not isolate a deterministic single lane-B gate. Next step is lane-C mixed-state reset-order probing (with selective repeats for divergent signatures) while schematic net extraction continues to replace heuristic lane groups with exact ABX00043 control-net ownership.
  **2026-05-10 follow-up (lane-C mixed-state + reset-order matrix):** Executed one-hot lane-C cases in [`bench-evidence/T6_laneC_mixed_resetorder_2026-05-10_190458/`](bench-evidence/T6_laneC_mixed_resetorder_2026-05-10_190458/) using cfgs `50..55`. OpenOCD IDR readbacks confirm each intended one-hot forced state (`PC10`, `PC11`, `PC12`, `PC9`, `PE11`, `PE12`) was applied before reset release. Across all six cases, `VER_REQ` still times out (`VER_URC` absent) and `AT+VER?` remains no-response.
  **Observed split:** `PC10/PC11/PC12` one-hot cases produce non-empty zero-heavy ATI payloads (~141-143 bytes), while `PC9/PE11/PE12` one-hot cases produce ATI silence (`no bytes observed`).
  **Conclusion:** Lane-C provides a reproducible behavioral partition but still no protocol recovery. This supports a multi-net ingress gate hypothesis (likely cross-lane interaction around `PC9`) rather than a single selector bit. Next step is a reduced cross-lane interaction matrix (lane-B + lane-C pairings) with selective repeats only for divergent signatures.
  **2026-05-10 follow-up (cross-lane B+C interaction matrix + selective repeats):** Executed reduced cross-lane cases in [`bench-evidence/T6_crosslane_BC_2026-05-10_190847/`](bench-evidence/T6_crosslane_BC_2026-05-10_190847/) using cfgs `56..63` (PC10 with `PB11/PC9` pairings, and PE11 with `PA9/PA10` pairings). OpenOCD readbacks confirm intended tuple states in all runs. Across all matrix cases, `VER_REQ` still times out (`VER_URC` absent) and `AT+VER?` remains no-response.
  **Observed behavior:** first pass showed mixed ATI signatures (including ATI silence on some tuples), but selective repeats (`59.r2`, `60.r2`) reverted to ATI-present zero-heavy payloads (~141-144 bytes).
  **Conclusion:** cross-lane tuple effects are perturbative but non-deterministic under current method; no stable ingress-gate tuple was isolated. Next step is timed hold-window sweeps on best-perturbing tuples while schematic owner/net extraction continues.
  **2026-05-10 follow-up (timed hold-window sweep on candidate tuples):** Executed hold-duration matrix in [`bench-evidence/T6_holdwindow_2026-05-10_191631/`](bench-evidence/T6_holdwindow_2026-05-10_191631/) with tuple A (`PC10=1, PB11=1, PC9=1`) and tuple B (`PE11=1, PA9=0, PA10=0`) at `100/500/1500 ms` pre/post-reset holds (cfgs `64..69`). OpenOCD readbacks confirm intended tuple states in all runs. Across all cases, `VER_REQ` still times out (`VER_URC` absent) and `AT+VER?` remains no-response.
  **Observed behavior:** one first-pass outlier (`66`, tuple A long hold) showed ATI silence, but immediate repeat (`66.r2`) reverted to ATI-present zero-heavy bytes, so timing-only discriminator was not reproducible.
  **Conclusion:** hold-window sweep is complete and does not clear the Stage1 ingress gate; timing variation alone is insufficient under current controls. Next step is owner/net-specific control extraction and intervention.
  **2026-05-10 follow-up (Stage1 standard quant 20-cycle stability run):** Executed [`firmware/x8_lora_bootloader_helper/run_stage1_standard_quant_end_to_end.ps1`](firmware/x8_lora_bootloader_helper/run_stage1_standard_quant_end_to_end.ps1) with `-Cycles 20` against serial `2E2C1209DABC240B`; run artifacts are in [`bench-evidence/T6_stage1_standard_quant_2026-05-10_103651_078-25816/`](bench-evidence/T6_stage1_standard_quant_2026-05-10_103651_078-25816/).
  **Results:** `LAUNCHER_FAIL_COUNT=0`, `TIMEOUT_COUNT=0`, `FINAL_RESULT_PASS=20`; all per-cycle subchecks in `results.csv` are `1` (`sync/getid/erase/write/verify/boot`) with stable `88-89 s` cycle times.
  **Conclusion:** this is positive harness/infrastructure stability evidence for the Stage1 standard runner, but it does not replace Method G ingress-go criteria (`VER_REQ` -> `VER_URC`), so the W1-7 protocol blocker remains open.
  **2026-05-10 follow-up (ABX00043 owner-net extraction Stage0):** Added topology-driven extraction note [`../AI NOTES/2026-05-10_ABX00043_UART4_OwnerNet_Extraction_Stage0_Copilot_v1_0.md`](../AI%20NOTES/2026-05-10_ABX00043_UART4_OwnerNet_Extraction_Stage0_Copilot_v1_0.md) and updated CHIP-DOCS findings/questions for [`IMX8MM_UART4`](../AI%20NOTES/CHIP-DOCS/IMX8MM_UART4/findings.md). Using Arduino official user-manual topology evidence, candidate active route-control devices are now explicitly tracked (`U16-U19` muxes `74LVC1G157`, `U8/U20/U21/U22` level shifters `SN74LVC1T45`, `U10` bus buffer `SN74LVC1G125`, `U23` Murata).
  **Interpretation:** component-level evidence reinforces a control-ownership ingress gate hypothesis (OE/SEL/EN/DIR ownership) consistent with one-way user-mode behavior.
  **Next action:** complete direct schematic net-name extraction for those devices, map owner/polarity of each control net, then run a narrowed OpenOCD matrix using only those owner nets (with immediate repeat on every divergent signature).
  **2026-05-10 targeted halted follow-up (PA9=ANALOG + PA11 sweep):** Executed halted cfgs `70a` and `71a` with explicit probe-before-release sequencing and captured evidence in [`bench-evidence/T6_targeted_pa9analog_manual_2026-05-10_205045/`](bench-evidence/T6_targeted_pa9analog_manual_2026-05-10_205045/). OpenOCD readbacks confirm forced states were applied while H7 remained halted (`A9=ANALOG`, `A10=ANALOG`, `B11/C9/C10=HIGH`, PF4 reset released) with `PA11=LOW` in `70a` and `PA11=HIGH` in `71a`. Outcomes: both cases remain `VER_REQ` timeout (no `VER_URC`), no `BOOT_URC`; `70a` shows ATI zero-bytes (`144 x 0x00`), while `71a` shows ATI silence. Net: PA9 bus-contention and PA11-level flip are insufficient as standalone fixes; W1-7 remains blocked and owner-net expansion is required.
  **2026-05-10 targeted owner-net expansion (PA12/PB12 under halted PA9=ANALOG baseline):** Executed halted cfgs `72a` (`PA12=HIGH`) and `73a` (`PB12=HIGH`) with evidence in [`bench-evidence/T6_targeted_ownerexp_pa12_pb12_2026-05-10_210828/`](bench-evidence/T6_targeted_ownerexp_pa12_pb12_2026-05-10_210828/). Readbacks confirmed intended states during probe (`A9/A10=ANALOG`, `A11=LOW`, `B11/C9/C10=HIGH`, plus `A12=HIGH` or `B12=HIGH`, PF4 released). Both cases show identical failure signature: no `BOOT_URC`, ATI returns `143` zero bytes, `AT+VER?` no response, and `VER_REQ` timeout waiting for `VER_URC`. Net: PA12/PB12 one-bit toggles do not clear ingress; continue owner-net search beyond PA11/PA12/PB12.
  **2026-05-10 external similar-project expansion (with examples):** Added cross-project reference note [`../AI NOTES/2026-05-10_Similar_Projects_External_Examples_Copilot_v1_0.md`](../AI%20NOTES/2026-05-10_Similar_Projects_External_Examples_Copilot_v1_0.md), covering official Portenta X8 user-manual and multi-protocol-gateway examples (`/dev/ttymxc3`, `/dev/gpiochip5`), Arduino meta-arduino overlay reset examples (`gpioset gpiochip5 3`), and comparable gateway reset/stack projects. This note is now the external-example source for subsequent owner-net experiments and post-ingress integration planning.
  **2026-05-10 external similar-project expansion v1.1 (more examples):** Added deeper cross-project implementation examples in [`../AI NOTES/2026-05-10_Similar_Projects_External_Examples_Copilot_v1_1.md`](../AI%20NOTES/2026-05-10_Similar_Projects_External_Examples_Copilot_v1_1.md), including Semtech legacy/SX1302 reset-helper choreography (`reset_lgw.sh`), SX1302 HAL startup gating where tools call reset helper before chip access, ChirpStack concentratord pin/device-path override model (`com_dev_path`, `sx1302_reset_chip/pin`, power-en pins), and vendor profile defaults (RAK2287 / Seeed WM1302 with USB-vs-SPI path selection). This expands the evidence-backed template set for owner-net/endpoint separation and profile-style targeted experiments.
  **2026-05-10 owner-net profile framework follow-up (manifest runner + seeded profiles):** Added manifest-driven harness artifacts [`firmware/x8_lora_bootloader_helper/owner_net_profiles.json`](firmware/x8_lora_bootloader_helper/owner_net_profiles.json) and [`firmware/x8_lora_bootloader_helper/run_owner_net_profile.ps1`](firmware/x8_lora_bootloader_helper/run_owner_net_profile.ps1), plus implementation note [`../AI NOTES/2026-05-10_OwnerNet_Profile_Manifest_Runner_Copilot_v1_0.md`](../AI%20NOTES/2026-05-10_OwnerNet_Profile_Manifest_Runner_Copilot_v1_0.md). Seeded profiles currently cover `pa11_discriminator_recheck`, `pa11_inreset_toggle_recheck`, and `ownerexp_onehot_replay`, with explicit probe endpoint fields and ordered cfg case tags. Net: profile-style replay infrastructure is now in place for narrowed owner-net/endpoint branch execution; direct ABX00043 schematic net-name extraction remains pending before first owner-derived profile is added.
  **2026-05-10 owner-net profile framework validation/fix pass (v1.1):** Repaired runner execution blockers in [`firmware/x8_lora_bootloader_helper/run_owner_net_profile.ps1`](firmware/x8_lora_bootloader_helper/run_owner_net_profile.ps1) (repo-root auto-discovery, PowerShell 5.1 parent-walk compatibility, adb exec-out command path, fail-fast pull checks), then executed `pa11_inreset_toggle_recheck` successfully with evidence in [`bench-evidence/T6_profile_pa11_inreset_toggle_2026-05-10_220440/`](bench-evidence/T6_profile_pa11_inreset_toggle_2026-05-10_220440/). Result signatures are reproduced under manifest control: `85a/85a_r2` -> ATI zero-byte class (`143/144` zeros), `85b/85b_r2` -> ATI silence, all still `VER_REQ` timeout/no `VER_URC`. Detailed validation note: [`../AI NOTES/2026-05-10_OwnerNet_Profile_Manifest_Runner_Validation_Copilot_v1_1.md`](../AI%20NOTES/2026-05-10_OwnerNet_Profile_Manifest_Runner_Validation_Copilot_v1_1.md).
  **2026-05-10 seeded profile replay completion (v1.2):** Executed the two remaining seeded profiles using the validated runner: `pa11_discriminator_recheck` and `ownerexp_onehot_replay` with evidence in [`bench-evidence/T6_profile_pa11_discriminator_recheck_2026-05-10_220742/`](bench-evidence/T6_profile_pa11_discriminator_recheck_2026-05-10_220742/) and [`bench-evidence/T6_profile_ownerexp_onehot_2026-05-10_220814/`](bench-evidence/T6_profile_ownerexp_onehot_2026-05-10_220814/). Artifact completeness verified (`12` logs for 4-case profile, `18` logs for 6-case profile). Classification split is deterministic under manifest control: discriminator replay cases (`82a/82b/83a/83b`) stayed ATI-silence, while ownerexp one-hot replay cases (`72a..77a`) stayed ATI zero-byte class (`144/145` bytes). All cases still fail `VER_REQ` -> `VER_URC`, so W1-7 remains blocked. Detailed run note: [`../AI NOTES/2026-05-10_OwnerNet_Profile_Seeded_Replay_Runs_Copilot_v1_2.md`](../AI%20NOTES/2026-05-10_OwnerNet_Profile_Seeded_Replay_Runs_Copilot_v1_2.md).
  **2026-05-10 targeted owner-net expansion (PC11/PC12/PE11/PE12 under halted PA9=ANALOG baseline):** Executed halted cfgs `74a` (`PC11=HIGH`), `75a` (`PC12=HIGH`), `76a` (`PE11=HIGH`), and `77a` (`PE12=HIGH`) with evidence in [`bench-evidence/T6_targeted_ownerexp_pc11_pc12_pe11_pe12_2026-05-10_211610/`](bench-evidence/T6_targeted_ownerexp_pc11_pc12_pe11_pe12_2026-05-10_211610/). OpenOCD readbacks confirmed intended one-hot highs while H7 remained halted (`A9/A10=ANALOG`, `A11=LOW`, `B11/C9/C10=HIGH`, plus each discriminator high). All four probes show the same blocked signature: no `BOOT_URC`, ATI returns zero-only payload (`144/145` bytes), no `AT+VER?` response, and `VER_REQ` timeout waiting for `VER_URC`. Net: one-hot owner-net expansion through `PA11/PA12/PB12/PC11/PC12/PE11/PE12` has not cleared ingress; move to constrained multi-net tuples.
  **2026-05-10 constrained tuple follow-up (PA11-high + owner-net pairs under halted PA9=ANALOG baseline):** Executed tuple cfgs `78a` (`PA11=HIGH + PC11=HIGH`), `79a` (`PA11=HIGH + PC12=HIGH`), `80a` (`PA11=HIGH + PE11=HIGH`), and `81a` (`PA11=HIGH + PE12=HIGH`) with evidence in [`bench-evidence/T6_tuple_pa11high_ownerexp_2026-05-10_2026-05-10_212516/`](bench-evidence/T6_tuple_pa11high_ownerexp_2026-05-10_2026-05-10_212516/). OpenOCD readbacks confirmed each tuple while halted (`A9/A10=ANALOG`, `B11/C9/C10=HIGH`, tuple highs asserted). All four probe logs collapse to the same full-silence signature: no `BOOT_URC`, ATI no bytes, `AT+VER?` no bytes, and `VER_REQ` timeout waiting for `VER_URC`. Net: constrained 2-bit tuples around `PA11=HIGH` did not recover ingress; treat this as a stable hard-disable class and continue with mixed-domain tuple design.
  **2026-05-10 mixed-domain tuple PA11-pair discriminator (halted baseline):** Executed paired tuple families where only `PA11` flips while other tuple bits stay fixed: `82a` (`PA11=LOW, C11=HIGH, E11=HIGH`) vs `82b` (`PA11=HIGH, C11=HIGH, E11=HIGH`), and `83a` (`PA11=LOW, C12=HIGH, E12=HIGH`) vs `83b` (`PA11=HIGH, C12=HIGH, E12=HIGH`). Evidence: [`bench-evidence/T6_mixed_tuple_pa11_pair_2026-05-10_2026-05-10_212843/`](bench-evidence/T6_mixed_tuple_pa11_pair_2026-05-10_2026-05-10_212843/). Readbacks confirm intended tuple states in all cases. Reproducible classifier result: PA11 low cases (`82a`, `83a`) produce ATI zero-byte payloads (`144` zeros), while PA11 high cases (`82b`, `83b`) produce full ATI silence; all four still fail `VER_REQ` -> `VER_URC`. Net: PA11 is a dominant mode selector but not sufficient for ingress recovery.
  **2026-05-10 PA11 reset-edge timing discriminator (halted, repeated):** Executed timing cfgs `84a` (PA11 pre-high -> post-low) and `84b` (PA11 pre-low -> post-high), each with immediate repeat (`84a_r2`, `84b_r2`), under fixed tuple controls (`C11=HIGH`, `E11=HIGH`, baseline `A9/A10=ANALOG`, `B11/C9/C10=HIGH`). Evidence: [`bench-evidence/T6_pa11_timing_edge_2026-05-10_2026-05-10_214142/`](bench-evidence/T6_pa11_timing_edge_2026-05-10_2026-05-10_214142/). OpenOCD confirms intended post states (`A11=0` for 84a, `A11=1` for 84b). Repeated probe signatures are stable: 84a/84a_r2 -> full silence (ATI no bytes), 84b/84b_r2 -> ATI zero-byte class (`143/144` zeros); both still fail `VER_REQ` -> `VER_URC`. Net: PA11 post-reset state remains deterministic classifier; edge sequencing alone did not recover ingress.
  **2026-05-10 PA11 in-reset toggle discriminator (halted, repeated):** Executed cfgs `85a` (toggle PA11 HIGH->LOW while PF4 held LOW, post-low) and `85b` (toggle PA11 LOW->HIGH while PF4 held LOW, post-high), each with immediate repeat (`85a_r2`, `85b_r2`) under fixed tuple controls (`C11=HIGH`, `E11=HIGH`, `A9/A10=ANALOG`, `B11/C9/C10=HIGH`). Evidence: [`bench-evidence/T6_pa11_inreset_toggle_2026-05-10_2026-05-10_214318/`](bench-evidence/T6_pa11_inreset_toggle_2026-05-10_2026-05-10_214318/). OpenOCD confirms intended post states (`A11=0` for 85a, `A11=1` for 85b). Repeated probe signatures stayed deterministic and unchanged from prior PA11 classification: 85a/85a_r2 -> ATI zero-byte class (`143/144` zeros), 85b/85b_r2 -> full silence (ATI no bytes); both still fail `VER_REQ` -> `VER_URC` and show no `BOOT_URC`. Net: toggling PA11 during reset-low does not recover ingress; post-state remains the dominant discriminator.
- [ ] **Tractor:** verify cellular SARA-R412M registers and sends a test MQTT publish
- [ ] **Tractor:** wire D1608S SSR1–SSR4 to four directional valve coils (boom-up, boom-down, bucket-curl, bucket-dump) per [MASTER_PLAN.md §8.18](MASTER_PLAN.md); wire the remaining four directional coils (drive LH/RH fwd/rev) to D1608S SSR5–SSR8. The Opta base's onboard EMRs are reserved for engine-kill / horn / parking-brake / spare. Drive 8 LEDs as coil stand-ins from M4 core.
- [ ] **Base:** verify Max Carrier + X8 boots, get SSH access
- [ ] **Base:** install Docker on X8 Yocto image
- [ ] **Base:** verify Linux can drive the Max Carrier LoRa module from `base_station/lora_bridge.py` (no Arduino firmware on the base H747 per [MASTER_PLAN.md §8.2](MASTER_PLAN.md)). 2026-05-04 update: first prove whether this is `/dev/ttymxc3` AT commands to the Murata `CMWX1ZZABZ-078` module rather than raw SPI to an exposed SX1276.
- [ ] **Base:** verify Gigabit Ethernet works, get DHCP lease on office LAN
- [ ] **Handheld:** flash MKR WAN 1310 with `RadioLib` "hello world" sketch, verify LoRa TX/RX
- [ ] **Handheld:** verify joystick analog reads + button reads on breadboard
- [ ] **Handheld:** verify OLED display works
- [ ] **All three nodes:** verify they can hear each other's LoRa frames at bench distance with the same parameters (SF7, **BW 250 kHz**, CR 4-5, freq 915.0 MHz, sync 0x12) per [LORA_PROTOCOL.md](LORA_PROTOCOL.md) / [DECISIONS.md](DECISIONS.md) D-A2

### Tractor X8 USB peripherals — W2-01 wedge mitigation follow-ups

Implementation done in this round (camera bring-up, root-only USB guard,
unprivileged capture decoupling, orchestrator integration, udev rule + audio
blacklist files) is captured in [`firmware/x8_lora_bootloader_helper/w2_01_camera_usb_guard.sh`](firmware/x8_lora_bootloader_helper/w2_01_camera_usb_guard.sh),
[`firmware/x8_lora_bootloader_helper/w2_01_camera_first_light.sh`](firmware/x8_lora_bootloader_helper/w2_01_camera_first_light.sh),
[`firmware/x8_lora_bootloader_helper/run_w2_01_camera_first_light_end_to_end.ps1`](firmware/x8_lora_bootloader_helper/run_w2_01_camera_first_light_end_to_end.ps1),
[`firmware/x8_lora_bootloader_helper/99-w2-01-c2.rules`](firmware/x8_lora_bootloader_helper/99-w2-01-c2.rules), and
[`firmware/x8_lora_bootloader_helper/lifetrac-no-usb-audio.conf`](firmware/x8_lora_bootloader_helper/lifetrac-no-usb-audio.conf).
Full pros/cons + voltage-drop quantification is in [`../AI NOTES/2026-05-14_USB_Wedge_Software_Mitigations.md`](../AI%20NOTES/2026-05-14_USB_Wedge_Software_Mitigations.md) §7–§8.

The remaining items below are the deferred suggestions from that note's §8.
Each is independently shippable; none block the next bench run.

- [ ] **W2-01.D1 Bench-only `provision_x8.sh` install step** — one-shot
  helper that copies `99-w2-01-c2.rules` to `/etc/udev/rules.d/`, copies
  `lifetrac-no-usb-audio.conf` to `/etc/modprobe.d/`, runs `udevadm control
  --reload && udevadm trigger`, and `rmmod snd_usb_audio` (best-effort).
  Documented reversal: `rm` both files + `udevadm reload` + `modprobe
  snd_usb_audio`. Buys ~10–30 mV avoided VBUS sag at C2 attach plus
  protection against mid-session unplug/replug events the guard misses
  (per §8.3 / §8.4 of the mitigation note).
- [ ] **W2-01.D2 Stopgap systemd oneshot for runtime autosuspend** —
  install a unit that writes `-1` to `/sys/module/usbcore/parameters/autosuspend`
  early in boot, before USB hot-plug. Local-only; no Foundries change.
  Useful when the operator forgets to run the guard, or when the C2 is
  plugged at boot. Race-prone (may lose to fast camera enumeration);
  ship only if §8.1 is more than a sprint away. See §8.2 of the
  mitigation note.
- [ ] **W2-01.D3 Production `OSTREE_KERNEL_ARGS=usbcore.autosuspend=-1`** —
  bake the autosuspend disable into the next LmP factory image so it is
  active before any USB device enumerates. Bundle with the next planned
  image cycle, not a one-off rebuild. Closes the cold-plug-at-boot hole
  the runtime guard cannot. Coordinate with Foundries pipeline owner.
  See §8.1 of the mitigation note.
- [ ] **W2-01.D4 Bench measurement of voltage-drop estimates** — the
  ~110–310 mV software-only headroom number in §8.4 is order-of-magnitude
  only (assumes ~150–300 mΩ rail+trace+cable). Capture a USB VBUS scope
  trace at the C2 input during attach with and without the guard, and
  with and without a powered hub, to tighten the numbers. Adds bench
  evidence to [`../AI NOTES/2026-05-14_USB_Wedge_Software_Mitigations.md`](../AI%20NOTES/2026-05-14_USB_Wedge_Software_Mitigations.md) §8.4.
- [ ] **W2-01.D5 Powered USB hub on bench** — still the largest single
  voltage-drop fix (~300–800 mV vs ~110–310 mV from the entire software
  stack combined). Source one before W2-01 declares "wedge-free" status.
  Existing TODO on this is implicit; this entry makes it explicit.

### Tractor X8 USB peripherals — W2-02/W2-03 GPS + IMU coverage

Per [`../AI NOTES/2026-05-14_USB_Wedge_Software_Mitigations.md`](../AI%20NOTES/2026-05-14_USB_Wedge_Software_Mitigations.md) §7,
the MCP2221A bridge plus its Qwiic-attached NEO-M9N GPS and BNO086 IMU
appear to the X8 as **one HID device drawing ~50–80 mA total** — no
isochronous bandwidth, no audio interface, no UVC quirks. They do **not**
need the W2-01 camera guard.

- [ ] **W2-02/W2-03.U1 Inherit autosuspend disable for free** — once
  W2-01.D3 (production kernel arg) ships, the MCP2221A automatically
  benefits at no extra config cost. No per-device work required.
- [ ] **W2-02/W2-03.U2 Optional belt-and-braces udev rule** — drop a
  one-liner keyed on `04d8:00dd` that sets `power/control=on` and
  `power/autosuspend_delay_ms=-1` for the MCP2221A specifically. Cost
  is essentially zero; protects against the HID re-bind path that opens
  the device on demand from `gps_service.py` / `imu_service.py`. See
  the suggested rule body in §7 of the mitigation note.
- [ ] **W2-02/W2-03.U3 Confirm `hid-mcp2221` is mainline in the X8 LmP
  kernel** — driver has been in mainline since Linux 5.10, so this
  should be a no-op on the current image, but verify before W2-02
  bring-up so we do not chase a missing-module symptom on bench.
- [ ] **W2-02/W2-03.U4 ADuM3160 USB galvanic isolator integration** —
  already tracked under [Phase 0 → Tractor-side sensors](#tractor-side-sensors-and-cameras)
  ("USB galvanic isolator (ADuM3160-based, ~$25)"). When inserted
  inline between the X8 and the MCP2221A, the isolator presents as its
  own USB hub node in `lsusb -t` and the bus addresses for the C2 and
  MCP2221A will shift — re-verify the W2-01 udev VID:PID rule matches
  unchanged (it should; rules key on VID:PID, not bus path).

### Tractor X8 USB peripherals — Coral USB Accelerator (base-station only)

Per [`../AI NOTES/2026-05-14_USB_Wedge_Software_Mitigations.md`](../AI%20NOTES/2026-05-14_USB_Wedge_Software_Mitigations.md) §7,
the Coral USB Accelerator is the **only other peripheral in the BOM**
that shares the C2's wedge risk profile (multi-amp inrush at first
inference, isochronous-like bursty load). It currently lives only on the
base-station X8.

- [ ] **W?-Coral.U1 If Coral is co-located with the C2 on a single X8**
  (currently not planned, but possible if the Phase 1 Mini-PCIe spike
  fails and the USB variant is adopted), apply the W2-01 guard pattern
  to it as well: add `1a6e:089a` (unbound dev board) and `18d1:9302`
  (post-firmware) to the udev rule, and to the guard's authorization-gate
  list. No new script; reuse `w2_01_camera_usb_guard.sh` with an extra
  VID:PID.

---

## Phase 2 — Common firmware (shared by all three nodes)

Implement [`firmware/common/`](firmware/common/) — the shared protocol layer.

- [ ] Implement KISS framer (FEND/FESC byte stuffing) — ~100 lines
- [ ] Implement CRC-16/CCITT
- [ ] Implement frame structs (`ControlFrame`, `TelemetryFrame`, `HeartbeatFrame`) per [LORA_PROTOCOL.md § Frame format](LORA_PROTOCOL.md#frame-format)
- [ ] Implement `lora_proto_encode()` / `lora_proto_decode()`
- [ ] Implement AES-128-GCM wrapper using MbedTLS (built into Arduino core for both SAMD21 and STM32H7)
- [ ] Implement nonce generator (source_id + sequence + timestamp + random)
- [ ] Unit tests for framer (FEND-in-payload, max-length frame, zero-length frame)
- [ ] Unit tests for crypto (round-trip encrypt/decrypt, replay rejection, tamper rejection)
- [ ] Implement key provisioning utility `provision.py` (writes pre-shared key to flash via USB-CDC)
- [ ] Document key rotation procedure

### Phase 2.LoRa — LoRa-stack tasks consolidated from [LORA_IMPLEMENTATION.md](LORA_IMPLEMENTATION.md)

Major implementation-plan-level tasks not already enumerated in Phases 2–5 above. Cross-link each to the LORA_IMPLEMENTATION.md § that owns the spec.

**Phase 2.LoRa.0 — Week-1 bench measurements (block Phase 1–5 work that depends on the numbers):**

- [ ] **R-7 retune-cost bench** (per [LORA_IMPLEMENTATION.md §8 week 1](LORA_IMPLEMENTATION.md)): measure `setFrequency` + `setSpreadingFactor` + `setBandwidth` + `setCodingRate` cost on SX1276 via RadioLib. Record baseline. **If > 5 ms,** burst-batching code path becomes mandatory in `lora_proto.cpp`. *Sketch in place at [`firmware/bench/lora_retune_bench/`](firmware/bench/lora_retune_bench/); needs an actual run.*
- [ ] **TX→RX turnaround + CSMA backoff bench** (per [LORA_IMPLEMENTATION.md L8](LORA_IMPLEMENTATION.md)): record so the airtime ledger is accurate, not assumed.
- [ ] **Control cadence / PHY blocker** — resolve the 2026-04-27 airtime mismatch before field motion: encrypted `ControlFrame` is ~92 ms at SF7/BW125 while the current control cadence is 20 Hz (50 ms). Decide between BW250/BW500, lower cadence, reduced overhead, or a dedicated control radio/channel; bench-verify before hydraulic testing.
- [ ] **Image fragment cap blocker** — resolve the 2026-04-27 airtime mismatch before image-pipeline bring-up: 32 B at SF7/BW250 estimates at ~36 ms, so the 25 ms cap currently allows only ~15 B cleartext fragments unless PHY/cap changes.
- [x] ~~**Cross-doc cascade pass** — land the IMAGE_PIPELINE.md §3.2 reservations (`0x28`/`0x29`/`0x2A`/`0x63`/badge enum) into [LORA_PROTOCOL.md](LORA_PROTOCOL.md) proper.~~ **✅ Done** — see [LORA_PROTOCOL.md topic table](LORA_PROTOCOL.md#telemetryframe-variable-9128-bytes) (`0x28`/`0x29`/`0x2A`), [opcode table](LORA_PROTOCOL.md#command-frame-opcodes) (`0x63`), and the badge enum table at line 270.

**Phase 2.LoRa.1 — PHY policy implementation (per [LORA_IMPLEMENTATION.md §3](LORA_IMPLEMENTATION.md)):**

- [x] ~~**Three-profile PHY in `lora_proto.cpp`**~~ **✅ Done** — `LP_PHY_CONTROL_SF7/SF8/SF9`, `LP_PHY_TELEMETRY`, `LP_PHY_IMAGE` defined in [`firmware/common/lora_proto/lora_proto.c`](firmware/common/lora_proto/lora_proto.c) and mirrored in [`base_station/lora_proto.py`](base_station/lora_proto.py). Per-frame retune via `CMD_LINK_TUNE` mechanism still needs runtime wiring on the M7.
- [x] **Adaptive control-link SF ladder** with R-8 hysteresis (N=3 consecutive bad 5 s windows for any transition; 30 s clean for SF↑→SF↓). `CMD_LINK_TUNE` sent twice back-to-back at old-then-new SF; revert + fail-counter increment if no Heartbeat at new SF within 500 ms. *Implemented in [`firmware/tractor_h7/tractor_m7.ino`](firmware/tractor_h7/tractor_m7.ino) `poll_link_ladder()` + `try_step_ladder()` + `send_link_tune()`. Reciprocal handheld/base receiver still needs the matching retune handler (handheld.ino + lora_bridge.py both currently no-op on inbound `CMD_LINK_TUNE`).*
- [x] **Image-link auto-fallback ladder** wiring (per [IMAGE_PIPELINE.md §3.4](IMAGE_PIPELINE.md)): `RollingAirtimeLedger` + `EncodeModeController` from [`base_station/link_monitor.py`](base_station/link_monitor.py) are now wired into [`lora_bridge.py`](base_station/lora_bridge.py) — every TX and RX records airtime via `attribute_phy()`, a 1 Hz worker emits `CMD_ENCODE_MODE` on rung change, and the `(U_image, U_telemetry, U_total)` triple is published as retained JSON on `lifetrac/v25/control/source_active`. Alarms fire on `U_telemetry > 30 %` and `U_total > 60 %`.

**Phase 2.LoRa.2 — MAC: FHSS + CSMA (per [LORA_IMPLEMENTATION.md §3.2](LORA_IMPLEMENTATION.md), [MASTER_PLAN.md §8.17 FHSS bullet](MASTER_PLAN.md)):**

- [x] ~~**8-channel hop sequence** in `lora_proto.cpp`~~ **✅ Done** — `lp_fhss_channel_index` / `lp_fhss_channel_hz` in [`firmware/common/lora_proto/lora_proto.c`](firmware/common/lora_proto/lora_proto.c); Python mirror `fhss_channel_index` / `fhss_channel_hz` in [`base_station/lora_proto.py`](base_station/lora_proto.py). Deterministic Fisher-Yates seeded by `key_id`, 8 channels @ 3.25 MHz starting 902 MHz.
- [x] **CSMA skip-busy** — helper landed: `lp_csma_pick_hop()` in [`firmware/common/lora_proto/lora_proto.c`](firmware/common/lora_proto/lora_proto.c) and `pick_csma_hop()` in [`base_station/lora_proto.py`](base_station/lora_proto.py); default threshold –90 dBm, max 4 skips, falls through to last candidate so a control frame still goes out. Unit tests cover clean-channel, single-skip, all-busy, and threshold-boundary cases. *Caller wiring (RadioLib `scanChannel` on tractor/handheld TX path, base-SPI driver on bridge) and the audit-log skip-event hook still open.*
- [ ] **Spectrum-analyser FHSS verification** (week 6, per [LORA_IMPLEMENTATION.md §8](LORA_IMPLEMENTATION.md)) — confirm 8 channels active, ≤ 12.5 % per-channel dwell over 60 s, no out-of-band emissions, EIRP ≤ +36 dBm. Pre-Phase-9 FCC verification.

**Phase 2.LoRa.3 — Priority queue + airtime cap (per [LORA_IMPLEMENTATION.md §4](LORA_IMPLEMENTATION.md)):**

- [x] **P0/P1/P2/P3 priority queue** in the base-station bridge — [`base_station/lora_bridge.py`](base_station/lora_bridge.py) routes every TX through `_tx_queue` (heap by priority + FIFO tiebreaker) drained by a single `_tx_worker` thread; classification via `classify_priority()` in [`base_station/lora_proto.py`](base_station/lora_proto.py) covered by `test_classify_priority_buckets`. P0 = ControlFrame/Heartbeat/CMD_ESTOP/CMD_LINK_TUNE/CMD_PERSON_APPEARED; P1 = CMD_CLEAR_ESTOP/CMD_ROI_HINT/CMD_REQ_KEYFRAME/CMD_CAMERA_SELECT/CMD_ENCODE_MODE; P2 = telemetry; P3 = image fragments. *Firmware-side queue (handheld + tractor in `lora_proto.c`) still open — those nodes only TX a handful of frame types so an explicit queue is lower-priority there.*
- [ ] **L1/R-6 25 ms-per-fragment airtime cap** — enforced uniformly on P2 telemetry and P3 image fragments. No P2/P3 frame can begin TX if remaining airtime in current opportunity > 25 ms.
- [ ] **R-6 telemetry fragmentation** — oversized `TelemetryFrame` payloads fragment using the `TileDeltaFrame` scheme; base-station bridge reassembles before MQTT publish.
- [ ] **Burst-batching code path** (gated by R-7 measurement) — if retune > 5 ms, batch image fragments so radio retunes at most twice per refresh window (once into image PHY, once back).

**Phase 2.LoRa.4 — Security (per [LORA_IMPLEMENTATION.md §6](LORA_IMPLEMENTATION.md)):**

- [x] **Replay-defence sliding window** — 64-frame per-source `LpReplayWindow` lives in [`firmware/common/lora_proto/lora_proto.c`](firmware/common/lora_proto/lora_proto.c) (`lp_replay_init` / `lp_replay_check_and_update`); used per source in [`firmware/tractor_h7/tractor_m7.ino`](firmware/tractor_h7/tractor_m7.ino) `process_air_frame`. Bit-compatible Python mirror `ReplayWindow` in [`base_station/lora_proto.py`](base_station/lora_proto.py) covered by 5 unit tests in [`base_station/tests/test_lora_proto.py`](base_station/tests/test_lora_proto.py) (duplicate, in-order advance, out-of-order, too-old, 16-bit wrap).
- [x] ~~**Nonce generator**~~ **✅ Done** — `build_nonce` in [`base_station/lora_proto.py`](base_station/lora_proto.py); 12 B = `source_id (1) ‖ seq (2) ‖ epoch_s (4) ‖ random (5)`. C-side mirror now landed as `lp_build_nonce(...)` in [`firmware/common/lora_proto/lora_proto.c`](firmware/common/lora_proto/lora_proto.c) + declaration in [`firmware/common/lora_proto/lora_proto.h`](firmware/common/lora_proto/lora_proto.h).
- [x] ~~**`tools/provision.py`**~~ **✅ Done** — [`tools/provision.py`](tools/provision.py) writes pre-shared 16 B key + 4 B key ID via USB-CDC.
- [ ] **`KEY_ROTATION.md`** (NEW, week 2 per [LORA_IMPLEMENTATION.md §11](LORA_IMPLEMENTATION.md)) — operator-facing key rotation procedure. Companion to `provision.py`. **Decision L-O2 (rotation cadence) deadline: before week 2.**

**Phase 2.LoRa.5 — Multi-source arbitration (per [LORA_IMPLEMENTATION.md §5](LORA_IMPLEMENTATION.md)):**

- [x] ~~**`pick_active_source()`** runs every M7 loop iteration~~ **✅ Done** — [`firmware/tractor_h7/tractor_m7.ino`](firmware/tractor_h7/tractor_m7.ino) line 130, `HEARTBEAT_TIMEOUT_MS = 500` enforced. Failsafe to neutral on `SOURCE_NONE`.
- [ ] **TAKE CONTROL latch** — 30 s P0 priority after button release; persists across heartbeat misses but not across `SOURCE_NONE`.
- [ ] **Source-active publisher** — topic `0x10` carries `active_source` + per-source RSSI/SNR + current SF rung + airtime-% triple `(U_image, U_telemetry, U_total)` at 1 Hz + on every change. *Wire in [`base_station/lora_bridge.py`](base_station/lora_bridge.py) once airtime ledger is integrated.*
- [ ] **Audit log** — every source transition, SF transition, encode-mode transition, FHSS skip event, replay rejection, GCM-tag rejection, CSMA backoff event appended to the [§8.10 black-box logger](MASTER_PLAN.md).

**Phase 2.LoRa.6 — Observability (per [LORA_IMPLEMENTATION.md §7](LORA_IMPLEMENTATION.md)):**

- [x] **`link_monitor.py`** — rolling 10 s airtime ledger per profile; computes `U_image`, `U_telemetry`, `U_total`; alarms if `U_telemetry > 30 %` or `U_total > 60 %`; emits `CMD_ENCODE_MODE` per the image auto-fallback ladder. *Module at [`base_station/link_monitor.py`](base_station/link_monitor.py) is now wired into [`lora_bridge.py`](base_station/lora_bridge.py); alarm thresholds live as `U_TELEMETRY_ALARM` / `U_TOTAL_ALARM` in the bridge worker.*
- [ ] **`tools/lora_rtt.py`** — handheld→tractor→base RTT measurement harness via timestamp echo. Run nightly during build weeks 1–10; audit-log diff per night; flag regressions.
- [ ] **Operator UI surface** — airtime-% bar (green < 40 %, yellow 40–60 %, red > 60 %); current SF rung pill; FHSS hop indicator (channel 1–8); two-source-active banner if both handheld and base are within heartbeat timeout.

**Phase 2.LoRa.7 — Validation gates (per [LORA_IMPLEMENTATION.md §9](LORA_IMPLEMENTATION.md), all must pass before field test):**

- [ ] **L-V1.** L1 P0 starvation — 30-min mixed-mode stress, zero P0 TX-start delays > 25 ms (joint pass with image V1).
- [ ] **L-V2.** L3 failsafe — power-off active source, valve neutral within 500 ms p99 (repeat for HANDHELD, BASE, AUTONOMY).
- [ ] **L-V3.** Three-source arbitration — 30 s latch + 500 ms timeout handover.
- [ ] **L-V4.** TAKE CONTROL preemption — within next M7 tick (≤ 20 ms).
- [ ] **L-V5.** Replay rejection — captured-frame retransmit rejected + logged.
- [ ] **L-V6.** Tamper rejection — bit-flipped frame rejected via GCM tag + logged.
- [ ] **L-V7.** Adaptive SF ladder — step-attenuator sweep, no hunting, 30 s clean before step-up.
- [ ] **L-V8.** R-6 telemetry fragmentation round-trip — 120 B payload survives bit-identical via fragment+reassemble.
- [ ] **L-V9.** R-7 retune cost recorded; burst-batching code path validated under image load if > 5 ms.
- [ ] **L-V10.** L5 FHSS spectrum-analyser compliance — 8 channels, ≤ 12.5 % dwell, no OOB, EIRP ≤ +36 dBm.
- [ ] **L-V11.** Latency — handheld→valve ≤ 150 ms p99; base→valve ≤ 250 ms p99.
- [ ] **L-V12.** Field range — base mast→tractor ≥ 1 km LoS minimum; handheld→tractor ≥ 500 m minimum.

**Phase 2.LoRa.8 — Open scope decisions (need a human stakeholder):**

- [ ] **L-O1 — Region.** US 915 MHz only for v25, or also EU 868 MHz? Affects FHSS channel plan + per-region key-ID prefix. **Deadline: before week 1 (Phase 0 procurement).** Default if undecided: US 915 MHz only.
- [ ] **L-O2 — Key rotation cadence.** Annual / on-incident-only / never. Affects whether `provision.py` ships with operator-runnable docs or stays a workshop tool. **Deadline: before week 2.** Default if undecided: on-incident-only.
- [ ] **L-O3 — Two-radio split (Revisit-4) field-test escape hatch.** Adopt in v25 if L-V1 / image-V1 joint gate fails in field test, or hold the line and downshift the encoder more aggressively? **Deadline: only on documented gate failure.** Default if undecided: hold the line; revisit only on field failure.

---

## Phase 3 — Handheld firmware

- [ ] Implement `firmware/handheld_mkr/handheld.ino` skeleton with main loop at 50 Hz
- [ ] Wire joystick reads with deadband and calibration
- [ ] Wire button reads with debounce
- [ ] Implement TAKE CONTROL latch logic (30 s after button release)
- [ ] Implement E-STOP detection and signaling
- [ ] Implement OLED status screen (source state, RSSI, battery, take-control countdown)
- [ ] Build ControlFrame from inputs and TX at 50 Hz
- [ ] RX bench-test: send hand-crafted ControlFrame from PC SDR transmitter or another MKR, verify decode + display update
- [ ] LiPo battery + charging verification
- [ ] Move from breadboard to perfboard or custom PCB
- [ ] Enclosure assembly with cable strain relief

---

## Phase 4 — Tractor firmware

### M7 core (`firmware/tractor_h7/tractor.ino`)

- [ ] LoRa modem driver setup
- [ ] Cellular SARA-R412M setup with MKRNB library
- [ ] Implement `pick_active_source()` arbitration (per [LORA_PROTOCOL.md § Multi-source arbitration](LORA_PROTOCOL.md#multi-source-arbitration))
- [ ] Implement source-state tracking (last heartbeat, sequence #, RSSI)
- [ ] Implement IPC to M4 (push active ControlFrame every 50 ms)
- [ ] Implement **Modbus RTU master** to Opta valve controller per [TRACTOR_NODE.md § Modbus RTU register map](TRACTOR_NODE.md#modbus-rtu-register-map-max-carrier--opta)
  - [ ] 50 Hz writes of `valve_coils` + flow setpoints + watchdog_counter
  - [ ] 10 Hz reads of telemetry + safety_state
  - [ ] Detect Opta `WATCHDOG_TRIPPED` or `ESTOP_LATCHED` and propagate up to operator UI
- [ ] Implement telemetry publishers for each topic
- [ ] Implement MQTT-SN packet builder
- [ ] Implement microSD logging (rotated daily)
- [ ] Implement watchdog (M7 watchdog hits → reset; also stops Modbus master → Opta safe-mode within 200 ms)

### M4 core (`firmware/tractor_h7/tractor_m4.cpp`)

- [ ] 100 Hz deterministic loop
- [ ] Read ControlFrame from IPC shared memory
- [ ] Translate axes → valve-coil bitfield + flow setpoint values for Modbus
- [ ] Watchdog: ControlFrame age > 200 ms → zero coils + flow setpoint (Opta will independently catch this too via Modbus watchdog)
- [ ] Bench test: cycle valves with bench Opta + LED stand-ins at 100 Hz

### Hardware E-stop chain

- [ ] Wire latching safety relay (Phoenix Contact PSR series)
- [ ] Wire engine-kill solenoid path
- [ ] Wire 24 V valve coil rail through NC contact
- [ ] Test: M7 watchdog timeout drops valve power within 200 ms
- [ ] Test: handheld E-stop signal triggers safety relay within 200 ms

### Tractor-side telemetry sources

- [ ] Wire **NEO-M9N GPS via Qwiic** (chains off BNO086 on the MCP2221A bus); mount the SMA active patch antenna on cab roof with U.FL→SMA bulkhead pigtail; verify `i2cdetect` sees both 0x42 (GPS) and 0x4A/0x4B (IMU); add a second X8-side service that publishes NMEA at 1 Hz on `lifetrac/v25/telemetry/gps` (`topic_id=0x01`, already in `lora_bridge.py` `TOPIC_BY_ID`)
- [ ] Wire CAN-FD to engine ECU (if engine ECU available) — Deutsch DT04-5P J1939 harness
- [ ] Wire hydraulic pressure sensors (4–20 mA → Opta A0602 inputs, M12-A cordsets); confirm `hyd_supply_psi`/`hyd_return_psi` register values track a calibrated reference gauge
- [ ] Wire **BNO086 IMU via MCP2221A USB→Qwiic adapter** to an X8 USB host port; verify Yocto enumerates `/dev/i2c-N` (`i2cdetect -y N` shows BNO086 at 0x4A or 0x4B). Run a Python service on the X8 that reads the IMU at 50 Hz and publishes `lifetrac/v25/telemetry/imu` MQTT (roll/pitch/yaw + accel) at 5 Hz. The bridge ships it as `topic_id=0x07` over LoRa. (Switch to native I²C wired to the Max Carrier breakout only if a future use case needs sub-millisecond determinism.)
- [ ] Mount **USB UVC webcam** (Logitech C920 or ELP IP67) — route cable through panel-mount USB bulkhead with gland; verify Yocto enumerates as `/dev/video0` (`v4l2-ctl --list-devices`)
- [ ] Bring up GStreamer pipeline on X8: `v4l2src device=/dev/video0 ! image/jpeg ! jpegdec ! v4l2h264enc ! rtph264pay ! udpsink` for first-light WebRTC test
- [ ] Verify each source publishes to correct MQTT topic (per [LORA_PROTOCOL.md topic table](LORA_PROTOCOL.md#telemetryframe-variable-9128-bytes))

---

## Phase 4.5 — Opta valve-controller firmware (Modbus slave)

This is the *industrial I/O layer* that the Max Carrier H7 talks to over RS-485. Most logic ports from [RESEARCH-CONTROLLER/arduino_opta_controller/](RESEARCH-CONTROLLER/arduino_opta_controller/) — strip the MQTT/BLE control surfaces, replace with Modbus slave.

### Setup

- [ ] Install Arduino IDE + Opta board package (`Arduino Mbed OS Opta Boards`)
- [ ] Install `ArduinoRS485` and `ArduinoModbus` libraries
- [ ] Verify Opta WiFi boots, USB-C serial monitor works
- [ ] Verify Opta Ext D1608S enumerates over the expansion bus (`OptaController` API)
- [ ] Verify Opta Ext A0602 enumerates and 0–10 V output reaches full scale on a multimeter

### Modbus slave implementation (`firmware/tractor_opta/opta_valves.ino`)

- [ ] Initialize RS-485 at 115200 8N1 with `ArduinoRS485`
- [ ] Initialize `ArduinoModbus` as **slave at address 0x01**
- [ ] Allocate holding-register block 0x0000–0x0006 (7 registers) per [TRACTOR_NODE.md § Modbus RTU register map](TRACTOR_NODE.md#modbus-rtu-register-map-max-carrier--opta)
- [ ] Allocate input-register block 0x0100–0x010B (12 registers)
- [ ] On each `poll()`:
  - [ ] If `watchdog_counter` (0x0004) hasn't changed in 200 ms → set `safety_state = WATCHDOG_TRIPPED`, force all coils off, force flow setpoints to 0
  - [ ] Otherwise, copy `valve_coils` bitfield (0x0000) to onboard relays + D1608S relays
  - [ ] Copy `flow_setpoint_1/2` (0x0001/0x0002) to A0602 analog outputs (scale 0..10000 → 0..10 V)
  - [ ] If `arm_engine_kill` (0x0006) non-zero, energize engine-kill relay through PSR safety chain
  - [ ] Read all D1608S digital inputs into `digital_inputs` (0x0101)
  - [ ] Read all A0602 analog inputs, scale, write to 0x0102–0x0107
  - [ ] Update `safety_state` based on ignition-sense input + external E-stop loop monitor
- [ ] Implement onboard hardware watchdog (`IWatchdog`) at 500 ms; reset on every successful Modbus loop iteration
- [ ] Drive a GPIO output "Opta-alive" signal that feeds the PSR safety relay's monitored channel; goes low on `WATCHDOG_TRIPPED`
- [ ] Implement boot self-test: cycle each relay briefly, verify A0602 outputs hit 5 V midpoint, log to USB serial

### Carry-over from RESEARCH-CONTROLLER/arduino_opta_controller

- [ ] Port valve sequencing + deadband logic
- [ ] Port [`MICROTRAC_V17.10_OPTIMIZATION.md`](RESEARCH-CONTROLLER/MICROTRAC_V17.10_OPTIMIZATION.md) flow-valve scaling tables
- [ ] Port the four [code-review safety fixes from 2026-04-25](../AI%20NOTES/CODE%20REVIEWS/) (NaN clamp, non-blocking reconnect equivalent, stale-input zeroing, mode-switch polling) into the new Opta firmware from the start — they are already validated bugs

### Bench tests

- [ ] Master simulator (Python `pymodbus` on PC) writes valve_coils with random patterns at 50 Hz; verify Opta tracks within 50 ms
- [ ] Master simulator stops writing watchdog_counter; verify all coils drop within 200 ms
- [ ] Master simulator writes flow_setpoint = 5000; verify A0602 outputs 5.00 ± 0.05 V
- [ ] Pull RS-485 cable mid-operation; verify Opta drops to safe state, recovers cleanly when reconnected
- [ ] Power-cycle Opta while master is writing; verify coils stay off until Opta has completed boot self-test
- [ ] Run for 24 h with master simulator; verify zero missed cycles, zero false watchdog trips

---

## Phase 5 — Base station firmware (Linux side, runs in Docker on X8)

### Containers

- [ ] Write `docker-compose.yml` with services: nginx, web_ui, mosquitto, lora_bridge, timeseries
- [ ] **Mosquitto:** carry over [RESEARCH-CONTROLLER/config/mosquitto.conf](RESEARCH-CONTROLLER/config/mosquitto.conf), bind to LAN-only interface
- [ ] **lora_bridge** (Python):
  - [ ] Read serial from M7 over UART
  - [ ] Decode frames using shared protocol (port `lora_proto.cpp` to `lora_proto.py`)
  - [ ] Decrypt using shared AES-GCM (port `crypto.cpp` to `crypto.py`)
  - [ ] Publish telemetry to MQTT
  - [ ] Subscribe to control topics from web_ui, encrypt + frame, send over UART
  - [ ] Publish link-health topics (RSSI, SNR, loss)
- [ ] **web_ui** (FastAPI + Jinja2 + WebSockets):
  - [ ] HTTP routes: `/`, `/map`, `/telemetry`, `/log`, `/settings`, `/diagnostics`
  - [ ] WebSocket endpoint `/ws` for control + telemetry stream
  - [ ] Static assets: HTML, CSS, JS joystick widget
  - [ ] Authentication: **single shared PIN** (4–6 digits) configured at first boot, per [MASTER_PLAN.md §8.5](MASTER_PLAN.md)
- [ ] **nginx:** reverse proxy to web_ui + static asset caching. **Plain HTTP on port 80, LAN-only** per [MASTER_PLAN.md §8.5](MASTER_PLAN.md) (no TLS for v25; threat model is "physical LAN access = trusted operator").
- [ ] **timeseries:** InfluxDB or SQLite storing all `lifetrac/v25/telemetry/*` topics

### Web UI front-end (browser-side JavaScript)

- [ ] Touch-friendly virtual joystick widget (canvas-based; check `nipplejs` library)
- [ ] WebSocket client with auto-reconnect
- [ ] Live telemetry sidebar (RPM, oil T, battery, RSSI, source)
- [ ] E-STOP button (always-active; sends over both LoRa via lora_bridge AND cellular MQTT)
- [ ] Source-active banner ("HANDHELD HAS CONTROL", "BASE HAS CONTROL", etc.)
- [ ] REQUEST CONTROL button (visible when not active source)
- [ ] Map page with Leaflet + cached OpenStreetMap tiles + live GPS marker
- [ ] Telemetry graph page with Plotly or Chart.js
- [ ] Diagnostics page with link-health graphs

### Base station services (Linux only)

Per [MASTER_PLAN.md §8.2](MASTER_PLAN.md), the base station runs **no Arduino firmware**. There is no `firmware/base_h7/` target to build, flash, or CI. 2026-05-04 interface update: revalidate the original raw-SPI assumption for `base_station/lora_bridge.py`; Arduino's X8 Max Carrier example drives the onboard Murata LPWAN module through `/dev/ttymxc3` AT commands, not a `/dev/spidev*` SX1276 node.

### Image pipeline (per [IMAGE_PIPELINE.md](IMAGE_PIPELINE.md), [MASTER_PLAN.md §8.19](MASTER_PLAN.md), [BASE_STATION.md § Image pipeline](BASE_STATION.md#image-pipeline-portenta-x8-linux-side), [LORA_PROTOCOL.md § TileDeltaFrame](LORA_PROTOCOL.md#tiledeltaframe-image-pipeline-i--p-frames))

**Phase 5.0 — Protocol-level reservations (do FIRST, before any image-pipeline code lands):**

- [x] **Reserve topic ID `0x28`** `video/motion_vectors` in [LORA_PROTOCOL.md](LORA_PROTOCOL.md) (optical-flow microframes, degraded mode Q)
- [x] **Reserve topic ID `0x29`** `video/wireframe` in [LORA_PROTOCOL.md](LORA_PROTOCOL.md) (PiDiNet edges, extreme degraded mode P)
- [x] **Reserve topic ID `0x2A`** `video/semantic_map` as **v26-only** placeholder in [LORA_PROTOCOL.md](LORA_PROTOCOL.md) (do NOT implement in v25)
- [x] **Reserve opcode `0x63`** `CMD_ENCODE_MODE` (base → tractor, P2): `{full | y_only | motion_only | wireframe}` for the §3.4 auto-fallback ladder
- [x] **Add Badge enum table** to [LORA_PROTOCOL.md](LORA_PROTOCOL.md) per [IMAGE_PIPELINE.md §3.3](IMAGE_PIPELINE.md): `Raw`, `Cached`, `Enhanced`, `Recolourised`, `Predicted`, `Synthetic`, `Wireframe` — base attaches, browser must fail-closed if missing/malformed
- [ ] **LoRa PHY revisit — Revisit-3** (per [IMAGE_PIPELINE.md §13.1](IMAGE_PIPELINE.md)): **✅ Spec shipped** in [IMAGE_PIPELINE.md §3.4](IMAGE_PIPELINE.md). `link_monitor.py` implementation (week 5): use `RadioLib::getTimeOnAir`, apply 3-window hysteresis, surface `U` + ladder rung on telemetry topic `0x10`
- [ ] **LoRa PHY revisit — Revisit-4** (per [IMAGE_PIPELINE.md §13.2](IMAGE_PIPELINE.md)): **✅ Documented** in [MASTER_PLAN.md §8.17.1](MASTER_PLAN.md) as the v26 escape hatch. **Not adopted for v25.** No build action; revisit only if the C1 gate fails in field testing.
- [ ] **LoRa PHY revisit — Revisit-5** (per [IMAGE_PIPELINE.md §13.1](IMAGE_PIPELINE.md)): **✅ Policy shipped** in [MASTER_PLAN.md §8.17](MASTER_PLAN.md) — 8-channel FHSS across 902–928 MHz, 3.25 MHz spacing, ~12.5 % per-channel dwell. Implementation: deterministic hop sequence in `lora_proto.cpp` seeded by AES key ID; CSMA skip-busy-channel rule. Bench-verify duty calc with spectrum analyser before [Phase 9 FCC verification](#phase-9--documentation-regulatory-release).
- [ ] **LoRa PHY revisit — R-6** (per [IMAGE_PIPELINE.md §13.1](IMAGE_PIPELINE.md)): **✅ Policy shipped** in [MASTER_PLAN.md §8.17](MASTER_PLAN.md) — P2 telemetry frames inherit the 25 ms airtime cap; oversized topic payloads fragment using the `TileDeltaFrame` scheme. Implementation: extend the fragment scheme to `TelemetryFrame` in `lora_proto.cpp`; base-station bridge reassembles before MQTT publish.
- [ ] **LoRa PHY revisit — R-7** (per [IMAGE_PIPELINE.md §13.3](IMAGE_PIPELINE.md)): **Week 1 bench task** — measure actual `CMD_LINK_TUNE` retune cost (`setFrequency` + `setSpreadingFactor` + `setBandwidth` + `setCodingRate`) on the SX1276 via RadioLib. **If > 5 ms,** implement burst-batching of image fragments so the radio retunes at most twice per refresh window. Until measured, plan the burst-batching code path conservatively.
- [ ] **LoRa PHY revisit — R-8** (per [IMAGE_PIPELINE.md §13.1](IMAGE_PIPELINE.md)): **✅ Policy shipped** in [MASTER_PLAN.md §8.17](MASTER_PLAN.md) and [IMAGE_PIPELINE.md §3.4](IMAGE_PIPELINE.md) — require N=3 consecutive bad 5 s windows for any SF or `CMD_ENCODE_MODE` ladder transition. Implementation: hysteresis state machine in the SF-ladder logic in `lora_proto.cpp` and in `link_monitor.py`.

**Phase 5A — Foundation (CPU-only, no AI accelerator required, ships value alone):**

- [ ] `image_pipeline/reassemble.py` — collect `TileDeltaFrame` (topic `0x25`) fragments, time out missing fragments, mark stale tiles
- [ ] `image_pipeline/canvas.py` — persistent tile canvas; replace changed tiles in place; on `base_seq` mismatch send `CMD_REQ_KEYFRAME` (opcode `0x62`); **attaches the badge enum to every published tile**
- [ ] `image_pipeline/accel_select.py` — auto-detect Coral at startup; export `HAS_CORAL` for downstream stages; expose status to the UI
- [ ] **`firmware/tractor_x8/image_pipeline/register.py`** — phase-correlation pre-diff image registration (NEON-accelerated, ~5 % CPU). **Non-negotiable** per [IMAGE_PIPELINE.md week 2](IMAGE_PIPELINE.md): without this, byte-savings collapse the moment the tractor moves
- [ ] **`encode_tile_delta.py --y-only` + base-side `image_pipeline/recolourise.py`** — scheme Z (Y-only luma + 30 s colour reference + base recolouriser, `Recolourised` badge). Inserted at week 2.5 per [IMAGE_PIPELINE.md §8](IMAGE_PIPELINE.md)
- [ ] **`image_pipeline/link_monitor.py`** — rolling 10 s `bytes/refresh`; emits `CMD_ENCODE_MODE` per the auto-fallback ladder (`full ≥400 B`, `y_only 150–400 B`, `motion_only 50–150 B`, `wireframe <50 B`)
- [ ] **`image_pipeline/bg_cache.py`** — rolling per-tile median keyed on segmenter class output; fills missed tiles with `Cached` badge + age (the only hole-filler available without an AI inpainter)
- [ ] **`image_pipeline/state_publisher.py`** — WebSocket publisher: canvas tiles + per-tile age + per-tile badge enum + detection vectors + safety verdicts + accelerator status. **All authoritative state lives here, not in the browser**
- [ ] **`image_pipeline/fallback_render.py`** — server-side 1 fps render of the canvas, for HDMI console + headless QA (kept alive even when browser is the primary surface)

**Phase 5A.B — Browser-tier offload (mandatory in Phase 1, not deferred — per [IMAGE_PIPELINE.md §6](IMAGE_PIPELINE.md)):**

Lives in `base_station/web_ui/static/img/`. Capability floor = WebGL 2 + Canvas 2D; opportunistic WebGPU with WebGL fallback; defer WebNN to v26.

- [ ] `canvas_renderer.js` — WebSocket subscriber; per-tile blits to Canvas 2D in an OffscreenCanvas Worker
- [ ] `fade_shader.js` — WebGL 2 fragment shader for per-tile cross-fade (~3 display frames, target 60 fps local)
- [ ] `staleness_overlay.js` — yellow tint + age-in-seconds rendering (consumes `age_ms` from base, never computes locally)
- [ ] `badge_renderer.js` — reads badge enum per tile; **fail-closed if missing/malformed** (refusal-to-display logged to base health endpoint)
- [ ] `detection_overlay.js` — bounding-box rendering from `state_publisher` detection vectors
- [ ] `accel_status.js` — "AI accelerator: online / offline / degraded" pill, always visible
- [ ] `raw_mode_toggle.js` — one-click toggle to most-recent-bytes-only view; choice logged to base audit endpoint
- [ ] **Trust-boundary documentation in [BASE_STATION.md](BASE_STATION.md)** per [IMAGE_PIPELINE.md §6.1](IMAGE_PIPELINE.md): table of what stays browser-only (display polish) vs what never goes to the browser (safety detector, ROI generation, badge decisions, anything autopilot might consume)
- [ ] **Browser test matrix gate** (Phase-1 completion gate): Latest Chrome on a $200 Android tablet, latest Safari on a 2020 iPhone SE, latest Firefox on a Linux laptop, latest Chrome on a Windows laptop — all four must render canvas + per-tile fade + staleness + badges + raw-mode toggle correctly

**Phase 5B — Base-side AI (CPU first, Coral if available):**

- [ ] `image_pipeline/superres_cpu.py` — Real-ESRGAN-General-x4v3 via [ncnn](https://github.com/Tencent/ncnn) on the A53 cores at **0.5–1 fps** (not 30 fps); benchmark gate ≤300 ms/frame; sets `Enhanced` badge
- [ ] `image_pipeline/superres_coral.py` — Edge-TPU port of Real-ESRGAN; only used iff `HAS_CORAL` and the spike (Phase 0) passed; benchmark gate ≤30 ms/frame
- [ ] `image_pipeline/detect_yolo.py` — **independent base-side safety detector (R6, two-detector pattern)**; CPU path = YOLOv8-nano OR NanoDet-Plus per AGPL decision, Coral path = YOLOv8-medium. **Two-detector disagreement banner in UI** when tractor `0x26` and base detectors disagree on a high-confidence object; log to §8.10 logger for v26 retraining
  - [ ] **OPEN SCOPE DECISION O1 — AGPL stance** on Ultralytics YOLOv8 (AGPL-3.0) vs. NanoDet-Plus (Apache-2.0). **Deadline: before week 6.** Default if undecided: NanoDet-Plus, accept ~5 % accuracy reduction. See [IMAGE_PIPELINE.md §10](IMAGE_PIPELINE.md)
- [ ] **`image_pipeline/motion_replay.py`** — apply `0x28` motion vectors to existing canvas; sets `Predicted` badge (Q degraded mode)
- [ ] **`image_pipeline/wireframe_render.py`** — render `0x29` wireframe over canvas; sets `Wireframe` overlay (P extreme degraded mode)
- [ ] **Tractor-side `encode_motion.py`** — optical-flow microframe encoder for topic `0x28`
- [ ] **Tractor-side `encode_wireframe.py`** — PiDiNet edge encoder for topic `0x29`
- [ ] **OPEN SCOPE DECISION O2 — Coral on the v25 BOM** — order it for the spike, or skip entirely? **Deadline: before week 5** (Phase-0 spike must complete by then). Default if undecided: ship CPU-only Stack-NoCoral as primary; Coral added in v25.5 if a later spike succeeds. See [IMAGE_PIPELINE.md §10](IMAGE_PIPELINE.md)
- [ ] `image_pipeline/interp_rife.py` *(optional, Coral-only)* — RIFE frame interpolation between thumbnail arrivals; under `Enhanced` badge
- [ ] `image_pipeline/inpaint_lama.py` *(optional, Coral-only)* — LaMa-Fourier fill of stale tiles; under `Synthetic` badge; opt-in only

**Phase 5C — Operator UX safety rules (mandatory before live hydraulic test):**

- [ ] Staleness clock visible on every displayed canvas, always
- [ ] "Enhanced" / "Synthetic" badge on any non-1:1 pixel (super-resolved, RIFE-interpolated, LaMa-inpainted, AI-colorized)
- [ ] Never hide loss — partial / corrupt canvas shown with gaps visible, never extrapolated and hidden
- [ ] One-click "raw mode" toggle — most-recent received bytes only, no enhancement
- [ ] Audit log: which view-mode (raw / enhanced) the operator was using when each command was issued; persisted to the §8.10 black-box logger
- [ ] **"AI accelerator: online / offline / degraded"** indicator visible on the operator console at all times

### Tractor image pipeline (per [TRACTOR_NODE.md § Image pipeline](TRACTOR_NODE.md#image-pipeline-portenta-x8-linux-side)) — *no Coral on tractor*

- [ ] `firmware/tractor_x8/image_pipeline/capture.py` — V4L2 → 384×256 YCbCr buffer per camera
- [ ] `tile_diff.py` — pHash-based 32×32 tile-change detector, NEON-accelerated, ≤30 ms for 96 tiles
- [ ] `roi.py` — read valve activity from H747 over IPC, classify mode (loading / driving / idle), produce ROI mask; honour `CMD_ROI_HINT` (opcode `0x61`)
- [ ] `detect_nanodet.py` — [NanoDet-Plus](https://github.com/RangiLyu/nanodet) (Apache-2.0) at 320×320 INT8, six classes; benchmark gate ≤50 ms p99 on the X8 A53s
- [ ] `encode_tile_delta.py` — per-tile WebP at q15/q40/q60 by ROI/detection; assemble `TileDeltaFrame` body
- [ ] `fragment.py` — split into ≤25 ms airtime fragments
- [ ] `ipc_to_h747.py` — hand fragments to the M7 firmware ring buffer at P3
- [ ] **`CMD_PERSON_APPEARED` (opcode `0x60`)** — on first new high-confidence person/animal/vehicle in frame, emit P0 alert with normalised bbox centroid; promote next image transmission to P1 for one frame
- [ ] **Multi-camera attention multiplexing** — front=70 / bucket=25 / rear=5 default; reverse-stick-driven flip; person-detection-driven 90 % promotion
- [ ] **Logger requirement** — every captured canvas + detection set + operator command + active-view-mode appended to the §8.10 black-box logger (builds the v26 fine-tuning dataset)

**Phase 5D — Validation gates (must all pass before field test, per [IMAGE_PIPELINE.md §9](IMAGE_PIPELINE.md)):**

- [ ] **V1.** Image-pipeline P0 starvation gate: 30-min mixed-mode stress run, **zero P0 ControlFrame TX-start delays >25 ms attributable to image fragments**
- [ ] **V2.** End-to-end image latency: capture → base UI repaint, ≤500 ms p99 CPU-only, ≤300 ms p99 with Coral
- [ ] **V3.** `CMD_PERSON_APPEARED` end-to-end: walk a person across the FOV, alert reaches base UI in ≤250 ms p99
- [ ] **V4.** `CMD_REQ_KEYFRAME` recovery: induce I-frame loss, confirm base detects mismatch and tractor returns a fresh I within 1 refresh
- [ ] **V5.** Coral fallback: yank Coral mid-operation, confirm UI flips to "AI accelerator: offline" within 10 s and pipeline continues degraded
- [ ] **V6.** **Auto-fallback ladder validation** — attenuate the LoRa link in 50 B/s steps; verify the encoder downshifts cleanly through `full → y_only → motion_only → wireframe` without operator intervention and without losing the canvas
- [ ] **V7.** **Browser test matrix** — all four target browsers (Chrome/Android, Safari/iOS, Firefox/Linux, Chrome/Windows) render canvas + fade + badges + raw-mode toggle correctly
- [ ] **V8.** **Two-detector cross-check** — tractor `0x26` vs. base `detect_yolo.py` disagreements surface in UI within one refresh; logged to §8.10 black-box logger
- [ ] **V9.** Operator-UX safety rules (Phase 5C) all visible and functional — pre-condition for any live hydraulic test
- [ ] **V10.** **Trust-boundary fail-closed** — patch a tile in transit to remove its badge enum; browser **must refuse to display** the tile and log the refusal to the base health endpoint

---

## Phase 6 — Mast antenna installation (base station)

- [ ] Site survey: choose mast location (clear LoS to typical work area, away from buildings)
- [ ] Install ground rod, ≥2.5 m driven
- [ ] Erect mast (concrete base or guyed)
- [ ] Mount 8 dBi omni at top
- [ ] Run LMR-400 coax inside conduit
- [ ] Install lightning arrestor at mast base, ground to dedicated rod
- [ ] Verify SWR < 2:1 with VNA (or NanoVNA)
- [ ] Range-test by driving handheld+vehicle away from base, log RSSI vs distance

---

## Phase 7 — Integration & end-to-end testing

- [ ] All three nodes powered, in same room, exchange frames at 1 m bench distance
- [ ] Single-source test: only handheld active → tractor follows handheld
- [ ] Single-source test: only base UI active → tractor follows base
- [ ] Two-source test: both handheld and base active → tractor follows handheld (priority)
- [ ] Handover test: handheld releases control → tractor switches to base after 30 s latch + 500 ms timeout
- [ ] TAKE CONTROL test: base controlling → handheld grabs control with button → tractor switches immediately
- [ ] Failsafe test: power-off handheld while it's active source → tractor goes to neutral within 500 ms
- [ ] Failsafe test: power-off base while it's active source → tractor goes to neutral within 500 ms
- [ ] Replay attack test: capture a frame, retransmit later → tractor rejects
- [ ] Tamper test: flip a bit in a captured frame, retransmit → tractor rejects
- [ ] Latency measurement: handheld joystick → tractor valve, target ≤ 150 ms median
- [ ] Latency measurement: base UI joystick → tractor valve, target ≤ 250 ms median

---

## Phase 8 — Field testing

- [ ] Range test: base mast → tractor at 1 km, 5 km, 10 km, 15 km LoS
- [ ] Range test: base mast → tractor through light foliage at 1 km, 3 km
- [ ] Range test: handheld → tractor at 100 m, 500 m, 1 km, 2 km
- [ ] Vibration test: drive tractor over rough ground, verify no enclosure issues, no spurious failsafes
- [ ] Cellular fallback test: physically unplug LoRa antenna at tractor → verify cellular telemetry continues
- [ ] Engine-crank brown-out test: cold-start engine while tractor MCU is running → verify LiPo backup carries through
- [ ] Rain/IP rating test: spray-test enclosures with garden hose
- [ ] 24-hour soak test in shop: simulate normal operation pattern, log all events
- [ ] 7-day deployment test on real work site, log everything, fix any issues
- [ ] Document field-test results, update [HARDWARE_BOM.md](HARDWARE_BOM.md) with any part substitutions

---

## Phase 9 — Documentation, regulatory, release

- [ ] Write hookup guide (consolidated from [TRACTOR_NODE.md](TRACTOR_NODE.md), [BASE_STATION.md](BASE_STATION.md), [HANDHELD_REMOTE.md](HANDHELD_REMOTE.md))
- [ ] Write operator manual (how to use the handheld + web UI)
- [ ] Verify FCC §15.247 compliance with spectrum analyzer (handheld at +14 dBm, tractor at +20 dBm, base at +20 dBm + 8 dBi antenna = +26.3 dBm EIRP, all under +36 dBm limit)
- [ ] Open-source the firmware under GPLv3 (matching Meshtastic / RadioLib community norms)
- [ ] Open-source the web UI under AGPLv3 (so improvements stay open)
- [ ] Update top-level [LifeTrac-v25/README.md](../README.md) to reference this controller design
- [ ] Update [LifeTrac-v25/TODO.md](../TODO.md): mark "wireless control" item as in-progress, link here
- [ ] Tag a `controller-v1.0.0` release on GitHub
- [ ] Add controller hardware to the v25 main BOM

---

## Cross-cutting concerns (must address before Phase 7 integration)

These items cut across all three nodes and don't fit neatly into a single phase. Address each before end-to-end integration testing.

### Device pairing & key provisioning

- [ ] Define pairing bootstrap: tractor X8 generates AES-128 key + node ID on first boot, displays as **QR code** on its status OLED (or web UI for headless tractors)
- [ ] Handheld pairing flow: phone app or laptop scans tractor QR → writes key + node ID to MKR WAN 1310 over USB-C serial (one-time setup tool in `tools/pair_handheld.py`)
- [ ] Base station pairing: scan same QR from base X8 web UI `/settings/pair` page → key stored in encrypted config file
- [ ] Re-pair / key-rotation procedure documented in `OPERATIONS_MANUAL.md`
- [ ] Persistent AES-GCM nonce counter in tractor flash (survives reboot) to prevent replay-attack window after power cycle

### Firmware update strategy

- [ ] Document update path per node in `FIRMWARE_UPDATES.md`:
  - Tractor X8 / base X8: OTA over cellular (X8 pulls signed image, A/B partition rollback)
  - Tractor / base H7 co-MCU: flashed from X8 over internal SPI/UART
  - Opta: flashed from tractor X8 over RS-485 (Modbus file-transfer extension) OR USB in shop
  - Handheld MKR: USB-C only (too small a RAM for OTA over LoRa)
- [ ] Code-sign all binaries (Ed25519); X8 verifies before flashing co-MCUs

### Time synchronization

- [ ] Base X8 syncs to NTP over cellular, exposes time as LoRa beacon (1 Hz on a reserved frame type)
- [ ] Tractor X8 disciplines its RTC from base beacon when available, falls back to GPS PPS, falls back to free-running RTC
- [ ] All telemetry timestamps in UTC microseconds; document in `LORA_PROTOCOL.md`

### Safety case & cybersecurity

- [ ] Write `SAFETY_CASE.md`: hazard analysis (HAZOP-lite), claim ISO 13849 PL=c on the E-stop chain, document Phoenix PSR wiring as the safety function
- [ ] Document Modbus RS-485 link as a trust boundary (sealed enclosure, no external access)
- [ ] Base-station web UI: require WireGuard / Tailscale tunnel for any non-LAN access (no public HTTP exposure); document in `BASE_STATION.md`
- [ ] HTTPS + basic auth on LAN-only web UI as defence-in-depth

### Missing documentation

- [ ] Write `NON_ARDUINO_BOM.md` — consolidated DigiKey / Mouser / L-com / Phoenix Contact / Burkert / McMaster order list (counterpart to the Arduino-store list)
- [ ] Write `CALIBRATION.md` — joystick deadband, flow-valve 0–10 V → GPM curve, pressure-sensor zero, GPS antenna offset
- [ ] Write `FIELD_SERVICE.md` — diagnostic flowcharts, fuse map, common failure modes, spare-parts kit contents
- [ ] Write `OPERATIONS_MANUAL.md` — operator-facing (not engineer-facing): power-on, pairing, take-control, E-stop, charging the handheld

### Hardware-in-the-loop bench

- [ ] Add HIL bench rig to `HARDWARE_BOM.md` Dev Gear: 8× 12 V LEDs in place of valve coils, 8× 1 kΩ trimpots in place of pressure transducers, 2× DMM on the 0–10 V Burkert outputs, 12 V bench supply with current meter
- [ ] Document HIL bring-up procedure as part of Phase 1

### Radio vendor lock-in mitigation

- [ ] Abstract the radio HAL in `firmware/common/radio.h` so SX1276 (Murata SiP) and SX1262 (RFM modules) can be swapped firmware-only
- [ ] Keep an RFM95W + bare STM32 reference design sketch in `RESEARCH-CONTROLLER/` as Murata-EOL insurance

---

## Stretch goals (Phase 10+)

- [ ] Add MIPI camera on tractor (Portenta X8 only; needs to swap H7 for X8 on tractor side)
- [ ] WiFi video streaming when tractor + base are within WiFi range (see [VIDEO_OPTIONS.md](VIDEO_OPTIONS.md))
- [ ] LoRa thumbnail JPEGs (~5 KB every 2 seconds) when WiFi out of range
- [ ] **Image processing & transmission for the LoRa fallback link** — see [RESEARCH-CONTROLLER/VIDEO_COMPRESSION/README.md](RESEARCH-CONTROLLER/VIDEO_COMPRESSION/README.md) for full analysis. Build in this order:
  - [ ] **Phase A — SSDV slideshow (fallback floor):** ffmpeg/libjpeg on tractor X8 → 160×120 JPEG every 5–10 s, SSDV-chunked with FEC into ~250 B LoRa packets, reassembler + thumbnail tile in base-station web UI. Target: 2–5 kbps, 5–30 s latency. Days of work; ship first.
  - [ ] **Phase B — Situational overlay (don't compress video, compress the situation):** YOLO-nano + lane/edge + bucket-pose detection on tractor X8 (i.MX 8M Mini, NEON only — no NPU), send bounding boxes + classes + free-space mask + bucket angle (~50–500 B/frame at 5 fps = 2–20 kbps), base-station X8 renders synthetic top-down/first-person view from the structured data + tractor CAD. Annotate UI as "synthesized".
  - [ ] **Phase C — SVT-AV1 + ROI baseline:** 256×192 / 5 fps, GOP 150, CRF ~50, ROI map weighting lower-center (bucket) + top-center (horizon), film-grain table, dav1d on base. Reuse comma.ai grid-search CSV for tuning. Target: 8–15 kbps. Comparison baseline for Phase D.
  - [ ] **Phase D — Keyframe + neural temporal inflate:** real keyframe (256×192 JPEG/AVIF, ~3–5 kB) every 5–10 s + tiny encoder net producing ~64–128 B latent per frame at 5 fps, base-station decoder warps keyframe using latent + CAN ego-motion (already on LoRa control channel — free side signal). Adapt comma.ai `neural_inflate` (PR #49) or `mask2mask` (PR #53). Distill to ≤50 MFLOPs/frame to fit i.MX 8M Mini NEON budget. Target: 6–10 kbps, 200–600 ms latency.
  - [ ] **Dataset capture:** record ≥10 h of tractor footage at 256×192 / 5 fps with synchronized CAN ego-motion (yard, field, bucket loading, mud, dust, dawn/dusk, rain) for Phase D fine-tuning. Use the MIPI camera bullet above.
  - [ ] **Safety annotations:** any phase that synthesizes/hallucinates pixels (B, D, future mask2mask) must overlay a "SYNTHESIZED" badge in the base-station UI; tractor X8 keeps full local recording for incident review independent of what crosses LoRa.
  - [ ] **Bandwidth arbitration:** image stream MUST yield to control + telemetry packets — codec drops to next-lower phase (D → C → A) when LoRa link budget tightens; never starves control loop.
  - [ ] **Phase E — Multi-camera arbitration on the tractor X8:** support front + rear + implement cameras simultaneously (single USB root hub); only the *selected* camera produces LoRa thumbnails. Implement `CMD_CAMERA_SELECT` (opcode 0x03 in [LORA_PROTOCOL.md](LORA_PROTOCOL.md#command-frame-opcodes)); auto-flip to rear camera when reverse stick is held >50% for >1 s (decision made on tractor X8, not base, to avoid round-trip latency); echo active camera back on telemetry topic 0x22. Add the second camera (Kurokesu C2 or C1 PRO board + 3.6 mm M12) on the rear ROPS. Mount cabin cam in the cab through windshield, or add Wiegmann WA-series NEMA 4X enclosure for external mounts (see HARDWARE_BOM.md camera-path notes).
  - [ ] **Phase F — Crop-health onboard analysis (the killer feature):** RGB-only ExG / canopy-cover proxy on the *existing* front camera (zero hardware adder, ships with v25 first light); 30 B/min summary on topic 0x24. Phase F.1: add MAPIR Survey3W NDVI/OCN or dual NoIR for true NDVI; X8 NEON computes per-row NDVI + percent-canopy-cover, geotagged from `topic 0x01` GPS + IMU heading; raw frames cached to microSD for WiFi-when-parked retrieval. Base-station `/map` view overlays heatmap. See [VIDEO_OPTIONS.md § Crop-health analysis](VIDEO_OPTIONS.md#crop-health-analysis).
- [ ] Autonomy: GPS waypoint following on the M7 core
- [ ] ROS 2 bridge (port from [RESEARCH-CONTROLLER/ros2_bridge/](RESEARCH-CONTROLLER/ros2_bridge/))
- [ ] DroidPad mobile-app integration as fourth control source (port from [RESEARCH-CONTROLLER/DROIDPAD_INTEGRATION.md](RESEARCH-CONTROLLER/DROIDPAD_INTEGRATION.md))
- [ ] Multi-tractor base station: single base controls a fleet, each tractor has unique source ID

---

## Risk register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Custom firmware control-path bug causes runaway implement | Medium | **Critical** | Hardware E-stop independent of MCU; 200 ms M4 watchdog; bench test before every field session |
| FCC EIRP exceeded with high-gain mast antenna | Low | High (regulatory) | Software EIRP cap; spectrum analyzer verification before deployment |
| LoRa SiP firmware bug in Murata module | Low | Medium | Murata SiP is mature (in production since 2017); fall back to dedicated SX1276 module if discovered |
| Portenta product line discontinuation | Low | High | Use [RESEARCH-CONTROLLER/](RESEARCH-CONTROLLER/) SparkFun stack as alternative path |
| Cellular Cat-M1 not available at tractor work site | Medium | Low | Cellular is backup-only; LoRa is primary; degraded mode is well-tested |
| Mast lightning strike | Low | High | Lightning arrestor + dedicated ground rod; insurance |
| Operator confusion over source priority | High | Medium | Clear UI banner; physical TAKE CONTROL button is unambiguous; operator training |
| Murata CMWX1ZZABZ EOL / single-source | Low | High | Abstract radio HAL; keep RFM95W reference design in RESEARCH-CONTROLLER/ |
| Replay attack after tractor power cycle | Low | High | Persistent AES-GCM nonce counter in flash; session re-key on pairing |

---

## See also

- [ARCHITECTURE.md](ARCHITECTURE.md) — system design
- [HARDWARE_BOM.md](HARDWARE_BOM.md) — what to buy
- [LORA_PROTOCOL.md](LORA_PROTOCOL.md) — air-interface spec
- [TRACTOR_NODE.md](TRACTOR_NODE.md) · [BASE_STATION.md](BASE_STATION.md) · [HANDHELD_REMOTE.md](HANDHELD_REMOTE.md) — per-tier build guides
- [RESEARCH-CONTROLLER/WIRELESS_OPTIONS.md](RESEARCH-CONTROLLER/WIRELESS_OPTIONS.md) *(archived)* — historical comparison of wireless technologies considered before LoRa was selected
- [VIDEO_OPTIONS.md](VIDEO_OPTIONS.md) — video streaming options
- [RESEARCH-CONTROLLER/](RESEARCH-CONTROLLER/) — earlier prototypes and research docs
- [LifeTrac-v25/TODO.md](../TODO.md) — top-level v25 TODO list
