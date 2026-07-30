# LifeTrac v25 — LoRa Radio & Firmware Settings Reference

Bench lookup table for every configurable value in the v25 radio/firmware/daemon stack:
SX1276 registers, FCC profile machinery, compile-time flags, HostLink CFG keys and opcodes,
daemon environment variables, encoder knobs, safety timeouts, and bench-harness parameters.

**Reflects commit `d2630395`** (`docs(todo): consolidate v25.0.7+ radio-system work into the controller TODO`).
The working tree at capture time had modified `firmware/murata_l072/build/**` artifacts only; no source drift.

**All paths in the `Where` column are relative to `LifeTrac-v25/DESIGN-CONTROLLER/`.**

> **Defaults move.** Every row carries `file:line` precisely so you can re-read the source before you
> trust a number at the bench. Line numbers are the authority; this table is a convenience index.
> Anything that could not be resolved to a single source of truth is marked **unverified** rather than guessed.

**Status vocabulary**

| Status | Meaning |
| --- | --- |
| `live` | Read or written by production code on the shipped path. |
| `diagnostic` | Reachable only from bench harnesses, debug builds, or observability code. |
| `dead` | Defined but never read by anything that runs — changing it does nothing. |
| `deprecated` | Superseded; kept for wire/back-compat or archived subtrees. |
| `unknown` | Present in config but no consumer confirmed. |

Jump to: [PHY/radio](#1-phy--radio) · [Regulatory & FHSS](#2-regulatory--fhss) ·
[Firmware compile-time](#3-firmware-compile-time) · [Host protocol](#4-host-protocol--cfg-keys-and-opcodes) ·
[Tractor daemon](#5-tractor-daemon-environment) · [Base daemon](#6-base-daemon-environment) ·
[Encoder & image pipeline](#7-encoder--image-pipeline) · [Safety timing](#8-safety-timing) ·
[Bench harness](#9-bench-harness) · [**Dead or diagnostic-only**](#10-dead-or-diagnostic-only-settings) ·
[What we have tried on the bench](#11-what-we-have-actually-tried-on-the-bench) ·
[Not settable](#12-settings-that-are-not-settable)

---

## 1. PHY / radio

SX1276 registers, modem geometry, pin maps, LBT, RX scan/retune, and the Python-side PHY profile tables.

| Setting | Default | Range | Where | Status | Purpose |
| --- | --- | --- | --- | --- | --- |
| RegFrf 0x06/0x07/0x08 (carrier) | 915000000 Hz at boot; frf=(f<<19)/32e6 = 0xE4C000 | unconstrained — no bounds check; callers guard | `firmware/murata_l072/radio/sx1276.c:362` | live | 24-bit synthesizer word; rewritten on every FHSS hop and RX retune. |
| Boot frequency constant | `915000000UL` | hardcoded literal | `firmware/murata_l072/radio/sx1276.c:277` | live | The only frequency programmed at boot; under BENCH/DTS nothing retunes it. |
| boot default frequency (duplicate view) | `915000000UL` | n/a | `firmware/murata_l072/radio/sx1276.c:277` | live | p0 bench path deliberately leaves the synth here (TX retune skipped when `s_hop_freq_hz == 0`). |
| RegPaConfig 0x09 | `0x8C` = PA_BOOST, MaxPower 0, OutputPower 0x0C | dBm clamped 2..17; MaxPower hardcoded 0 | `firmware/murata_l072/radio/sx1276.c:376` | live | Only TX-power control the firmware has. |
| TX power boot value | `14U` dBm | 2..17 after clamp | `firmware/murata_l072/radio/sx1276.c:279` | live | What the radio runs at unless a CFG_SET arrives. |
| boot default TX power (clip site) | 14 dBm, written as `0x80 or (dbm-2)` | clipped to [2,17] | `firmware/murata_l072/radio/sx1276.c:369` | live | This hard clip — not the ERP clamp — is what actually bounds conducted power. |
| DEFAULT_TX_POWER_DBM | `14U` | tier ceilings bench 17, FHSS/DTS 30 dBm | `firmware/murata_l072/host/host_cfg.c:24` | live | Boot default for CFG key 0x01; the only CFG key with a real apply fn. |
| RegModemConfig1 0x1D | `0x82` = BW250, CR 4/5, explicit header | see field rows below | `firmware/murata_l072/radio/sx1276.c:450` | live | Whole-register write; no read-modify-write. |
| Spreading factor (SF) | 7 at boot and in all three profiles | clamped 6..12 | `firmware/murata_l072/radio/sx1276.c:452` | live | LoRa spreading factor; no CFG key, internal API or raw REG_WRITE 0x1E only. |
| Bandwidth (`bw_khz`) | 250 kHz boot; p0 250, p1 250, p2 500 | only 3 outcomes: >=500→500, >=250→250, else→125 kHz silently | `firmware/murata_l072/radio/sx1276.c:152` | live | Maps kHz onto register codes. Anything under 250 becomes 125 with no rejection. |
| Coding rate denominator | 5 (CR 4/5), encoded `(cr-4)<<1` | clamped 5..8 | `firmware/murata_l072/radio/sx1276.c:451` | live | FEC rate; no CFG key. |
| Header mode | 0 (explicit), hardcoded `or 0U` | hardcoded; no API sets implicit | `firmware/murata_l072/radio/sx1276.c:451` | live | Explicit LoRa PHY header always. |
| RegModemConfig2 0x1E | `0x74` = SF7, CRC on, SymbTimeout MSB 0 | n/a | `firmware/murata_l072/radio/sx1276.c:452` | live | Whole-register write; silently zeroes SymbTimeout high bits on every SF change. |
| RxPayloadCrcOn (0x1E bit 2) | 1, hardcoded | hardcoded 1 | `firmware/murata_l072/radio/sx1276.c:453` | live | `sx1276_rx_arm()` re-reads it and refuses to arm with `RX_CRC_DISABLED` if clear (`sx1276_rx.c:68`). |
| RegModemConfig3 0x26 | RMW: bit2 forced set, bit3 iff SF>=11 and BW<=125 → 0x04 at boot | n/a | `firmware/murata_l072/radio/sx1276.c:455` | live | AGC + low-data-rate optimize. |
| AgcAutoOn (0x26 bit 2) | 1, unconditional | never cleared by any path | `firmware/murata_l072/radio/sx1276.c:456` | live | Modem drives LNA gain; makes RegLna's manual gain field inert. |
| LowDataRateOptimize (0x26 bit 3) | 0 at boot (SF7/BW250) | derived, not a parameter | `firmware/murata_l072/radio/sx1276.c:457` | live | Auto-asserted only when SF>=11 and BW<=125; airtime estimator reads it back. |
| RegDetectOptimize 0x31 | RMW `(read & 0xF8) or 0x03`; 0x05 for SF6 | bits 2:0 only 0x03/0x05 | `firmware/murata_l072/radio/sx1276.c:469` | live | SF-dependent detection optimizer; upper 5 bits preserved from POR. |
| RegDetectionThreshold 0x37 | `0x0A` (SF!=6); `0x0C` for SF6 | two literals | `firmware/murata_l072/radio/sx1276.c:471` | live | SF-dependent detection threshold. |
| RegOpMode 0x01 | SLEEP 0x80, STANDBY 0x81, TX 0x83, RX_CONT 0x85, RX_SINGLE 0x86, CAD 0x87 | table-driven, 6 descriptors | `firmware/murata_l072/radio/sx1276_modes.c:91` | live | Boot writes 0x00 → 0x80 → 0x81, then `main.c:74` arms RX_CONT. |
| OpMode write-retry budget | 5000 µs deadline; 200 µs pre / 50 µs post per retry | hardcoded | `firmware/murata_l072/radio/sx1276_modes.c:108` | live | W1-9 workaround; on failure emits `RADIO_OPMODE_DRIFT` and accepts the mismatch. |
| RegTcxo 0x4B TcxoInputOn | RMW `read or 0x10` → 0x19 from POR 0x09 | only bit 4 set | `firmware/murata_l072/radio/sx1276_modes.c:236` | live | Enables the 32 MHz TCXO; without it OpMode transitions are silently ignored. |
| RegDioMapping1 0x40 | SLEEP/STANDBY/RX 0x00, TX 0x40, CAD 0xA0 | three literals | `firmware/murata_l072/radio/sx1276_modes.c:89` | live | DIO0..DIO3 mapping per mode. |
| RegDioMapping2 0x41 | 0x00 in all six mode rows | always 0x00 | `firmware/murata_l072/radio/sx1276_modes.c:90` | live | DIO4/DIO5 + MapPreambleDetect left at zero. |
| RegIrqFlags 0x12 | 0xFF clear-all before TX and on TX timeout; CAD writes 0xFF | n/a | `firmware/murata_l072/radio/sx1276_tx.c:414` | live | IRQ ack. Bits: TX_DONE 0x08, CAD_DET 0x01, CAD_DONE 0x04, VALID_HDR 0x10, CRC_ERR 0x20, RX_DONE 0x40. |
| RegFifoTxBaseAddr 0x0E | 0x00, written every `sx1276_tx_begin()` | hardcoded | `firmware/murata_l072/radio/sx1276_tx.c:386` | live | TX FIFO base pointer. |
| RegFifoAddrPtr 0x0D | 0x00 before TX fill; on RX set from RegFifoRxCurrentAddr | n/a | `firmware/murata_l072/radio/sx1276_tx.c:387` | live | FIFO read/write cursor (RX side `sx1276_rx.c:127`). |
| RegPayloadLength 0x22 | `req->length + 8` in routed builds | requests with len+8 > 255 refused before any radio action | `firmware/murata_l072/radio/sx1276_tx.c:388` | live | On-air PHY payload byte count. |
| TX FIFO length guard | `length + LORA_PKT_HDR_LEN <= 255` | payload <= 247 B routed | `firmware/murata_l072/radio/sx1276_tx.c:145` | live | Refuses oversized frames before consuming a hop slot; emits INTERNAL RFCO snapshot. |
| RegVersion 0x42 | read-only; expected 0x12 | init fails on {0x00, 0xFF} | `firmware/murata_l072/radio/sx1276.c:355` | live | Presence / SPI sanity check. |
| RegRssiValue 0x1B | read-only; `dBm = raw - 157` | n/a | `firmware/murata_l072/radio/sx1276_lbt.c:138` | live | Carrier-sense energy for LBT. |
| RSSI conversion offset | `-157` | n/a | `firmware/murata_l072/radio/sx1276_lbt.c:138` | live | HF-port constant, hardcoded twice (LBT and RX). |
| RegPktRssiValue 0x1A / RegPktSnrValue 0x19 | read-only; `rssi = raw-157`, `snr = q4/4` | n/a | `firmware/murata_l072/radio/sx1276_rx.c:124` | live | Per-frame quality reported in RX_FRAME URCs. |
| RegRxNbBytes 0x13 / RegFifoRxCurrentAddr 0x10 | read-only per frame | n/a | `firmware/murata_l072/radio/sx1276_rx.c:122` | live | Length and FIFO offset of the last received packet. |
| SPI1 clock divider | `SPI_CR1_BR_DIV8` → 2 MHz at 16 MHz PCLK2 | compile-time | `firmware/murata_l072/radio/sx1276.c:236` | live | Was DIV64 (~250 kHz) pre-RS-3.6; 255 B burst dropped ~8 ms → ~1 ms. |
| DIO EXTI config | DIO0=PB4, DIO1=PB1, DIO2=PB0, DIO3=PC13; rising edge; NVIC prio 3 | compile-time pin map | `firmware/murata_l072/radio/sx1276.c:240` | live | Captures modem IRQs into `s_irq_events`. |
| SPI / RF-switch / NSS pin map | NSS PA15, RESET PC0, SCK PB3, MISO PA6, MOSI PA7; RF_SW PA1/PC1/PC2 | compile-time | `firmware/murata_l072/radio/sx1276.c:12` | live | SCK on PB3 is load-bearing: PA5 is DIO4 inside the SiP and makes RegVersion read 0x00. |
| RF switch TX_BOOST assertion | TX: TXRX=1, RX=0, BOOST=1; RX/idle: 0/1/0 | binary | `firmware/murata_l072/radio/sx1276_modes.c:59` | live | Routes antenna to PA_BOOST during TX, matching RegPaConfig bit 7. |
| LBT enable | `LORA_FW_LBT_ENABLE = 1` | 0..1 | `firmware/murata_l072/radio/sx1276_lbt.c:83` | live | Master gate for carrier-sense-before-TX; 0 returns DISABLED immediately. |
| LBT RSSI threshold | `-90` dBm (CFG default byte 0xA6) | validator -120..0 dBm | `firmware/murata_l072/radio/sx1276_lbt.c:14` | live | Channel declared busy above this. |
| LBT max backoff | 500 ms; base 10 ms doubling, shift capped at 6 | validator 10..5000 ms | `firmware/murata_l072/radio/sx1276_lbt.c:15` | live | Exponential backoff ceiling after a busy verdict. |
| CAD symbols / LBT sample window | `4U` | validator 1..10; 0 would skip CAD but cfg cannot store 0 | `firmware/murata_l072/radio/sx1276_lbt.c:16` | live | Not programmed as symbols — reused verbatim as a millisecond RX dwell before the RSSI read. |
| SX1276_RX_RETUNE_PERIOD_MS | `380U` | compile-time; <= dwell cap/1000 | `firmware/murata_l072/include/sx1276_rx_retune_policy.h:46` | live | γ-1 RX silence-walk period; consumed at `sx1276_rx_retune_policy.c:50`, called from `sx1276_rx.c:362`. Inert whenever the slot clock is anchored. |
| SX1276_RX_SCAN_DWELL_MS | `500U` | compile-time | `firmware/murata_l072/include/sx1276_rx_scan_policy.h:63` | live | Per-channel listen time during cold-start acquisition. Comments at that line and `sx1276_rx.h:130` still say 100 ms — both stale. |
| SX1276_RX_SCAN_LOCK_LOSS_MS / GOAL_MS / REDESIGN_MS | 2000 / 5000 / 30000 ms | compile-time | `firmware/murata_l072/include/sx1276_rx_scan_policy.h:77` | live | Demote silent LOCKED at 2 s; abort SCANNING at 30 s. GOAL_MS is tracked-only (dead, see §10). |
| SX1276_RX_SCAN_MAX_RETRIES | `3U` | attempt count reported in low 4 bits of the fault sub-byte | `firmware/murata_l072/include/sx1276_rx_scan_fail.h:63` | live | Scan re-entries before FAILED becomes absorbing and the modem parks in standby. |
| Scan SM gating (fhss_init + tx_busy) | early-return if `!fhss_is_initialized()` or `tx_busy()` | n/a | `firmware/murata_l072/radio/sx1276_rx.c:604` | live | Two bench root-cause fixes: gate 1 stops the scan stealing the host-pinned 915 carrier; gate 2 stops ADVANCE yanking the PA mid-TX (was 100 % TX_TIMEOUT). |
| per-profile modem tuple (SF/BW/CR) | p0 7/250/5, p1 7/250/5, p2 7/500/5 | SF [6,12], BW {125,250,500}, CR [5,8] | `firmware/murata_l072/host/host_cfg_profile.c:196` | live | Modem config written at profile activation; boot default is 7/250/5. |
| LORA_FW_DEEP_SLEEP_BUILD | `1` | 0/1 | `firmware/murata_l072/config.h:37` | live | Seeds CFG key 0x08 only; no deep-sleep scheduler exists in the tree. |
| LORA_FW_BEACON_ENABLE | `1` | 0/1 | `firmware/murata_l072/config.h:40` | live | Seeds CFG key 0x09 only; grep finds no beacon implementation. |
| DEFAULT_BEACON_CHANNEL_IDX | `0U` | validator 0..7 | `firmware/murata_l072/host/host_cfg.c:30` | live | Stored and reportable; no beacon code consumes it. |
| HOST_TYPE_REG_READ_REQ | `0x30` | payload_len must be 1 | `firmware/murata_l072/include/host_types.h:36` | live | Raw SX1276 register read of any address — no allow-list on reads. |
| HOST_TYPE_REG_WRITE_REQ | `0x31` | allow-list {01,06,07,08,09,1D,1E,26,31,33,37,3B,40} | `firmware/murata_l072/include/host_types.h:37` | live | The backdoor the daemons use to pin RegFrf and re-arm RXCONT; 0x01 writes fold into the mode SM. |
| HOST_TYPE_REG_DATA_URC | `0xB0` | n/a | `firmware/murata_l072/include/host_types.h:54` | live | 2-byte {addr, value} answer to REG_READ_REQ. |
| HOST_TYPE_REG_WRITE_ACK_URC | `0xB1` | n/a | `firmware/murata_l072/include/host_types.h:55` | live | 2-byte echo of an accepted REG_WRITE_REQ. |
| LIFETRAC_FORCE_FRF_HZ | unset; auto-pins 915000000 for profile 0 or 2 | integer Hz; non-integer warned and ignored | `firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py:522` | live | Force-writes RegFrf after standby; works around the firmware not retuning on a same-id profile set. |
| RXCONT arm / re-arm (tractor) | `RegOpMode <- 0x85`, read/write timeout 0.5 s | n/a | `firmware/tractor_x8/image_tx_daemon.py:512` | live | Raw opmode write so the tractor can hear 0xFB frames; bypasses firmware, so every TX parks STANDBY and it must re-arm (`:636`). |
| read_reg/write_reg RegOpMode timeout (base) | 0.5 s | unconstrained | `base_station/image_rx_daemon.py:874` | live | Timeout on the raw RegOpMode read/write used to re-arm RXCONT. |
| RXCONT re-arm throttle (base) | 5.0 s | unconstrained | `base_station/image_rx_daemon.py:1070` | live | Idle-pass defensive re-arm to heal a silently dropped RX state. |
| PHY_CONTROL_SF7 | sf 7, bw 250, cr 5, preamble 8 | PhyProfile fields | `base_station/lora_proto.py:146` | live | Default control-plane PHY returned by `attribute_phy()`. |
| PHY_CONTROL_SF8 | sf 8, bw 125, cr 5, preamble 8 | LINK_PHY_NAMES index 3 | `base_station/lora_proto.py:147` | live | Second rung of the control SF ladder (no auto-fallback driver yet). |
| PHY_CONTROL_SF9 | sf 9, bw 125, cr 5, preamble 8 | LINK_PHY_NAMES index 2 | `base_station/lora_proto.py:148` | live | Third rung of the control SF ladder. |
| PHY_TELEMETRY | sf 9, bw 250, cr 8, preamble 12 | LINK_PHY_NAMES index 1 | `base_station/lora_proto.py:149` | live | Non-image telemetry PHY; only profile with cr 8 / preamble 12. |
| PHY_IMAGE_BW250 | sf 7, bw 250, cr 5, preamble 8 | LINK_PHY_NAMES index 5 | `base_station/lora_proto.py:150` | live | The image PHY under REG_PROFILE 0 and 1; default of `max_image_fragment_body`. |
| PHY_IMAGE_BW500 | sf 7, bw 500, cr 5, preamble 8 | LINK_PHY_NAMES index 6 | `base_station/lora_proto.py:151` | live | Image PHY for REG_PROFILE 2. |
| PHY_IMAGE | alias of BW250, distinct object and name | LINK_PHY_NAMES index 0 | `base_station/lora_proto.py:152` | live | Kept for `attribute_phy()`; PHY_BY_NAME holds both. |
| LINK_PHY_NAMES | image, telemetry, control_sf9, control_sf8, control_sf7, image_bw250, image_bw500 | index 0..6 | `base_station/lora_proto.py:164` | live | Wire index space for CMD_LINK_PROFILE; indices 0-4 frozen, new names append. |
| LINK_PHY_NAMES (tractor mirror) | same 7-tuple | `LinkBudget.update` rejects index outside range | `firmware/tractor_x8/camera_service.py:744` | live | Append-only mirror used by the link_budget topic. |
| LIFETRAC_FRAG_AIR_CAP_MS | `"170.0"` | unconstrained — bare `float()` at module scope | `base_station/lora_proto.py:837` | live | Per-fragment airtime cap feeding `max_image_fragment_body()`. A malformed value raises during import of both web_ui and image_rx_daemon. |
| _PROFILE_TO_PHY (tractor) | {0: BW250, 1: BW250, 2: BW500} | `.get(prof, BW250)` | `firmware/tractor_x8/image_tx_daemon.py:181` | live | Profile id → PHY for ToA estimation, fragment packing, modem contract check. |
| _PROFILE_TO_PHY (base) | {0: BW250, 1: BW250, 2: BW500} | keys {0,1,2} | `base_station/image_rx_daemon.py:188` | live | Profile id → PHY the modem is verified against. |
| _FRAG_BODY_BY_PROFILE | {0: 203, 1: 203, 2: 243}, fallback 203 | n/a | `firmware/tractor_x8/image_tx_daemon.py:232` | live | Usable fragment body bytes per profile; drives the batching efficiency guard. |
| fragment.py re-exports | magic = TELEMETRY_FRAGMENT_MAGIC; cap = IMAGE_FRAG_AIR_CAP_MS; default profile BW250 | frag_seq rolls 0..255 | `firmware/tractor_x8/image_pipeline/fragment.py:43` | live | Thin re-export so tractor callers do not reach into the base tree. |
| LADDER[3] rung table (H7) | {SF7,BW250,CR4/5}, {SF8,BW125}, {SF9,BW125} | 3 rungs | `firmware/tractor_h7/tractor_h7.ino:535` | live | The H7-side adaptive control PHY ladder actually programmed into RadioLib; rung 0's BW250 is not mirrored in the L072 PHY table. |
| LADDER hysteresis (window / step / HB thresholds / revert) | 5000 ms window; 3 bad to step down; 6 good to step up; 50 HB expected, bad <40, good >=48; revert 500 ms | unconstrained | `firmware/tractor_h7/tractor_h7.ino:540` | live | Hysteresis policy for the SF ladder and the CMD_LINK_TUNE revert window. |
| H7 `radio.begin()` frequency + sync word | 915.0 MHz, sync word `0x12` | n/a | `firmware/tractor_h7/tractor_h7.ino:1848` | live | RadioLib-side carrier and sync word; must match the L072's implicit POR 0x12. |
| LIFETRAC_BENCH_TX_DBM | `2` dBm (SX1276 minimum) | 2..20 dBm | `firmware/tractor_h7/tractor_h7.ino:1845` | live | TX power for the Arduino-side radios; comment says set to 20 for field use. A power surface entirely separate from CFG key 0x01. |
| TX_QUEUE_CAP / PRIO_SAFETY / PRIO_NORMAL / PAYLOAD_MAX (H7) | 4 / 0 / 1 / 256 | unconstrained | `firmware/tractor_h7/tractor_h7.ino:1310` | live | H7-side TX queue depth and two-level priority; separate from HOST_TXQ_DEPTH on the L072. |
| DurationS (RSSI sniff harness) | `25` s | unconstrained | `firmware/x8_lora_bootloader_helper/run_air_coupling_rssi_sniff.ps1:28` | diagnostic | RX sniff window for air-coupling characterisation. |
| TxCycles (RSSI sniff) | `60` | unconstrained | `firmware/x8_lora_bootloader_helper/run_air_coupling_rssi_sniff.ps1:29` | diagnostic | TX bursts fired at the sniffing peer. |
| rx_pair_nrst params | TxCycles 30, TxInterS 0.25, ExtraRxWindowS 15, RegProfile "0" | RegProfile 0/1/2 unvalidated | `firmware/x8_lora_bootloader_helper/run_rx_pair_nrst.ps1:32` | diagnostic | Two-board RX-pair NRST harness. |
| radio_state_dump params | WorkDir /tmp/lifetrac_strict, PostNrstSleep 1500 ms, ContainerTimeout 25 s, AlsoAfterS 0.0, RegProfile "0" | AlsoAfterS 0 disables second dump | `firmware/x8_lora_bootloader_helper/run_radio_state_dump.ps1:28` | diagnostic | Register/state dump harness. |
| rx_ver_warmup_sweep params | AttemptsPerCell 3, SettleSeconds {1.5, 3.0, 5.0}, PostNrstSleep 1500 ms | SettleSeconds is the sweep axis | `firmware/x8_lora_bootloader_helper/run_rx_ver_warmup_sweep.ps1:35` | diagnostic | Post-reset settle time vs VER-handshake success. |
| radio_sleep params | RemoteDir /tmp/lifetrac_p0c, Dev /dev/ttymxc3, Baud 921600 | AdbSerial is `[string[]]`, NOT mandatory — omitting it auto-discovers all devices | `firmware/x8_lora_bootloader_helper/run_radio_sleep.ps1:20` | diagnostic | Radio sleep-mode probe. |
| radio_wake_rxcont params | RemoteDir /tmp/lifetrac_p0c, Dev /dev/ttymxc3, Baud 921600 | same as above | `firmware/x8_lora_bootloader_helper/run_radio_wake_rxcont.ps1:15` | diagnostic | Wake-from-sleep into RXCONT probe. |
| w1_10b link harness params | Cycles 100, InterCycleS 0.2, Timeout 5.0, ExtraRxWindowS 30, Probe "tx_burst", RttTimeout 5.0, RegProfile "0" | Probe is `ValidateSet(tx_burst, ping_pong)` — genuinely validated | `firmware/x8_lora_bootloader_helper/run_w1_10b_rx_pair_end_to_end.ps1:54` | live | Two-peer link harness; forwards LIFETRAC_TX_COUNT / INTER_CYCLE_S / RX_WINDOW / RTT_TIMEOUT / PROBE_MODE / REG_PROFILE. |
| Probe (method_h stage2) | `"tx"` | `ValidateSet(tx, regversion, fsk, opmode_walk, rx, rx_listen, tx_burst)` | `firmware/x8_lora_bootloader_helper/run_method_h_stage2_tx_end_to_end.ps1:4` | live | Selects the stage-2 TX probe mode via LIFETRAC_PROBE_MODE. |

---

## 2. Regulatory & FHSS

### 2.1 Profile comparison — p0 / p1 / p2

`CFG_KEY_REG_PROFILE` (wire key `0x14`) selects one of three regimes. There is **no region abstraction** in
this tree — all three are US FCC Part 15.247, 902–928 MHz (`firmware/murata_l072/include/host_cfg_keys.h:43`).
The only region selector anywhere is `comm.lora_region` in the build config, and it does not reach the L072.

| Aspect | p0 `BENCH_ONLY_FIXED_915` | p1 `FCC_15_247_FHSS_50CH_BW250` | p2 `FCC_15_247_DTS_BW500` |
| --- | --- | --- | --- |
| Enum value | `0U` (`host_cfg_keys.h:63`) | `1U` (`host_cfg_keys.h:64`) | `2U` (`host_cfg_keys.h:65`) |
| Regulatory basis | out of scope (bench only) | 15.247(a)(1)(i) narrowband FHSS | 15.247(e) DTS, PSD-based |
| SF / BW / CR programmed | 7 / 250 kHz / 4-5 (`host_cfg_profile.c:226`) | 7 / 250 kHz / 4-5 (`host_cfg_profile.c:196`) | 7 / 500 kHz / 4-5 (`host_cfg_profile.c:219`) |
| BW the validator demands | 125 kHz declared (`host_cfg_profile.h:160`) — **not** what activation programs | 250 kHz exact (`host_cfg_profile.h:73`) | 500 kHz exact (`host_cfg_profile.h:74`) |
| Channel mask required | none | exactly `0x0003FFFFFFFFFFFF` (`host_cfg_profile.h:71`) | none |
| Hop scheduler | skipped (`sx1276_tx.c:187`) | armed — `sx1276_fhss_init(0,0,0)` (`host_cfg_profile.c:194`) | skipped |
| Slot clock | not anchored | 200 ms slots, 50-slot / 10 s epoch | not anchored |
| Legal-dwell accountant | skipped (`sx1276_tx.c:363`) | enforced, 400 ms per 10 s per channel | skipped |
| QoS airtime budget | 400000 µs/s (`host_cfg_profile.c:227`) | 400000 µs/s (`host_cfg_profile.c:196`) | 950000 µs/s (`host_cfg_profile.c:221`) |
| Tier power ceiling | 17 dBm (`host_cfg_profile.h:84`) | 30 dBm (`host_cfg_profile.h:85`) | 30 dBm (`host_cfg_profile.h:86`) |
| Host-side QoS mirror | 380000 µs (`image_tx_daemon.py:195`) | 860000 µs | 930000 µs |
| Fragment body bytes | 203 (`image_tx_daemon.py:232`) | 203 | 243 |
| Routing requirement | always accepted | rejected `UNROUTED` unless `LIFETRAC_FHSS_TX_ROUTED` | rejected `UNROUTED` unless `LIFETRAC_FHSS_TX_ROUTED` |
| Carrier behaviour | stays on whatever FRF was last written | hops 50 channels, 902.75–927.25 MHz | stays on whatever FRF was last written |

**Bench trap:** the shipped `CFG_KEY_FHSS_CHANNEL_MASK` default is `0xFF` (popcount 8), so activating p1
without first widening the mask fails with `MASK_POPCOUNT` (wire status 8). Both daemons set
`LIFETRAC_FHSS_WIDE_MASK=1` automatically when switching to p1.

### 2.2 Profile machinery, channel table, dwell, airtime

| Setting | Default | Range | Where | Status | Purpose |
| --- | --- | --- | --- | --- | --- |
| CFG_KEY_REG_PROFILE (0x14) | `REG_PROFILE_BENCH_ONLY_FIXED_915` = 0 | 0..2; >2 short-circuits OUT_OF_RANGE | `firmware/murata_l072/host/host_cfg.c:130` | live | Selects the regime; cfg_set collapses stage()+activate() into one wire transaction. |
| REG_PROFILE_MAX | `2U` | n/a | `firmware/murata_l072/include/host_cfg_keys.h:66` | live | Upper bound checked by both the wire validator (`host_cfg.c:277`) and `host_cfg_profile_validate` (`host_cfg_profile.c:93`). |
| HOST_CFG_PROFILE_FHSS_50CH_REQUIRED_MASK | `0x0003FFFFFFFFFFFF` | exact match for p1 | `firmware/murata_l072/include/host_cfg_profile.h:71` | live | Popcount<50 gives MASK_POPCOUNT; any bit >49 gives MASK_OUT_OF_TABLE. |
| HOST_CFG_PROFILE_FHSS_BW_HZ | `250000UL` | exact match (`host_cfg_profile.c:131`) | `firmware/murata_l072/include/host_cfg_profile.h:73` | live | BW p1 must declare; mismatch → BW_MISMATCH. |
| HOST_CFG_PROFILE_DTS_BW_HZ | `500000UL` | exact match (`host_cfg_profile.c:143`) | `firmware/murata_l072/include/host_cfg_profile.h:74` | live | BW p2 must declare. |
| HOST_CFG_PROFILE_BENCH_BW_HZ | `125000UL` | not enforced — p0 skips the BW check | `firmware/murata_l072/include/host_cfg_profile.h:160` | live | Synthesised for p0 by `host_cfg_profile_default_bw_hz()` (`host_cfg_profile.c:41`), reached from `host_cfg.c:284` and `:354`. **Mismatch: activation actually programs 250 kHz.** |
| CFG_KEY_FHSS_CHANNEL_MASK default | `0x00000000000000FFULL` (8 channels) | wire validator only requires != 0 | `firmware/murata_l072/host/host_cfg.c:28` | live | Consumed only by the profile validator; never read by the FHSS scheduler. Blocks p1 until widened. |
| host_cfg_profile power clamp | tier − max(0, gain−6), floored by hw ceiling | 0 or [2,30] dBm | `firmware/murata_l072/host/host_cfg_profile.c:64` | live | The ERP clamp. **FIXED 2026-07-29:** the result is now programmed into the PA at profile activation and also caps later `CFG_KEY_TX_POWER_DBM` writes (`host_cfg.c`, `cfg_active_erp_max_dbm()`). Previously it was only tested for ==0 at `:103` and discarded. |
| Power-clamp result → RegPaConfig | `min(configured, erp_max)` programmed on every activation | [2,17] dBm at the PA | `firmware/murata_l072/host/host_cfg.c` (`CFG_KEY_REG_PROFILE` case) | live | **FIXED 2026-07-29.** Activation programs the PA unconditionally, so hardware and cfg table always agree afterwards. Only ever lowers power — operator intent is preserved. Pinned by `cfg_profile_wire.c` `test_erp_*` (4 cases). |
| HOST_CFG_PROFILE_TIER_CEILING_BENCH_DBM | `17U` | n/a | `firmware/murata_l072/include/host_cfg_profile.h:84` | live | Tier ceiling for p0 (matches the SX1276 PA clamp). |
| HOST_CFG_PROFILE_TIER_CEILING_FHSS_DBM | `30U` | n/a | `firmware/murata_l072/include/host_cfg_profile.h:85` | live | Tier ceiling for p1; hw_ceiling (17) clamps far below it in practice. |
| HOST_CFG_PROFILE_TIER_CEILING_DTS_DBM | `30U` | n/a | `firmware/murata_l072/include/host_cfg_profile.h:86` | live | Tier ceiling for p2. |
| HOST_CFG_PROFILE_ANTENNA_GAIN_MAX_DBI | `30` | validator accepts [0,30]; outside → ANTENNA_OUT_OF_RANGE | `firmware/murata_l072/include/host_cfg_profile.h:88` | live | Upper bound on declared gain; negative gain also rejected. |
| HOST_CFG_PROFILE_ANTENNA_GAIN_FCC_THRESHOLD_DBI | `6` | n/a | `firmware/murata_l072/include/host_cfg_profile.h:89` | live | One-sided ERP knee: reduction = max(0, gain − 6) dB. |
| HOST_CFG_PROFILE_TX_POWER_MIN_DBM | `2U` | n/a | `firmware/murata_l072/include/host_cfg_profile.h:90` | live | Clamp floor; below it the clamp returns 0 and the profile is rejected NO_POWER_HEADROOM. |
| DEFAULT_ANTENNA_GAIN_DBI | `+2` dBi (int8) | no wire clamp; profile validator enforces [0,30] | `firmware/murata_l072/host/host_cfg.c:39` | live | Declared antenna gain feeding the ERP clamp at activation. |
| DEFAULT_HW_CEILING_DBM | `17U` | no wire clamp | `firmware/murata_l072/host/host_cfg.c:40` | live | Firmware-known PA ceiling for the headroom check. |
| HOST_CFG_PROFILE_REJECT_* enum | NONE 0 … NULL_ARG 9 | 0..9 | `firmware/murata_l072/include/host_cfg_profile.h:98` | live | Structured reject reasons from stage()/activate(). |
| host_cfg_profile_reject_t mapping | BAD_PROFILE→3, UNROUTED→7 | 0..9 → cfg_status 0/3/7..14 | `firmware/murata_l072/host/host_cfg_profile.c:48` | live | Internal reject codes → wire status byte. |
| sx1276_set_sf_bw_cr_checked() return | discarded — `(void)` cast, all 4 production callers use the void wrapper | n/a | `firmware/murata_l072/radio/sx1276.c:379` | live | FCC-A1a config invariant: rejects a tuple whose 255 B ToA exceeds the cap, emits the reject URC, skips register writes — but callers cannot observe the rejection. |
| airtime invariant max payload | `255U` | n/a | `firmware/murata_l072/radio/sx1276.c:419` | live | Worst-case payload each (SF,BW,CR) tuple is tested against. |
| airtime invariant framing assumptions | crc_on 1, implicit_header 0, preamble 8, low_dr_opt = (SF>=11 and BW<=125) | SF [6,12], CR [5,8], bw_hz != 0 | `firmware/murata_l072/radio/sx1276_airtime.c:147` | live | Fixed framing tuple; must match what `sx1276_set_sf_bw_cr()` programs. |
| SX1276_AIRTIME_DWELL_CAP_US | `380000UL` | compile-time | `firmware/murata_l072/include/sx1276_airtime.h:27` | live | Per-TX ToA ceiling, checked at config time (`sx1276.c:420`) and per frame before keying (`sx1276_tx.c:313`). |
| SX1276_AIRTIME_DWELL_WINDOW_MS | `400U` | n/a | `firmware/murata_l072/include/sx1276_airtime.h:25` | **dead** | Documents the window the 380 ms cap sits under; zero references anywhere. |
| SX1276_AIRTIME_DWELL_GUARD_US | `20000U` | n/a | `firmware/murata_l072/include/sx1276_airtime.h:26` | **dead** | Documents the 20 ms jitter guard; CAP_US is an independent literal, so editing this does not move the cap. |
| Airtime ToA preamble term | `((preamble+4)*t_sym) + t_sym/4`; preamble defaults to 8 when the register reads 0 | SF [6,12], CR [5,8], bw_hz 0 → returns 0 | `firmware/murata_l072/radio/sx1276_airtime.c:129` | live | Shared by the config invariant, per-TX gate, QoS reserve, RX clock anchor and TX slot-fit advisory. |
| SX1276_AIRTIME_WINDOW_MS | `1000U` | n/a | `firmware/murata_l072/radio/sx1276_airtime.c:12` | live | Rolling QoS fairness window per channel; deliberately separate from the legal 10 s window. |
| SX1276_AIRTIME_BUDGET_US | `400000U` | n/a | `firmware/murata_l072/radio/sx1276_airtime.c:13` | live | Static initialiser for `s_budget_cap_us` — the boot QoS budget before any activation. |
| s_budget_cap_us (setter) | 400000 initial; per-profile 400000 / 400000 / 950000 | clamped [10000, 950000] µs | `firmware/murata_l072/radio/sx1276_airtime.c:27` | live | Per-channel airtime allowed in the rolling 1 s window; a P0-latency policy gate, not an FCC rule. |
| SX1276_AIRTIME_BUDGET_DEFAULT_US | `400000UL` | clamped by the setter | `firmware/murata_l072/include/sx1276_airtime.h:44` | live | QoS duty installed by p0 and p1 activation. |
| SX1276_AIRTIME_BUDGET_DTS_US | `950000UL` | clamped [10000, 950000] | `firmware/murata_l072/include/sx1276_airtime.h:45` | live | 95 % duty for p2, leaving >=50 ms/s RX headroom. |
| airtime budget floor clamp | `10000U` | n/a | `firmware/murata_l072/radio/sx1276_airtime.c:30` | live | Raises any sub-10 ms request to 10 ms. |
| airtime budget ceiling clamp | `950000U` | n/a | `firmware/murata_l072/radio/sx1276_airtime.c:33` | live | Lowers any request above 950 ms so RX headroom survives. |
| SX1276_AIRTIME_CHANNEL_COUNT | `50U` | compile-time | `firmware/murata_l072/radio/sx1276_airtime.c:11` | live | Per-channel QoS array size; idx >= 50 is BAD_INPUT. |
| SX1276_FHSS_CHANNEL_COUNT | `50U` | `_Static_assert == 50` (`sx1276_fhss_chantab.c:11`) and `<= 64` (`sx1276_fhss.c:17`); idx >= 50 returns 0 and callers skip the RegFrf write | `firmware/murata_l072/include/sx1276_fhss_chantab.h:32` | live | Hop table size and one epoch (one channel per slot). |
| SX1276_FHSS_CHANNEL_SPACING_HZ | `500000UL` | `_Static_assert >= 25000` | `firmware/murata_l072/include/sx1276_fhss_chantab.h:33` | live | Uniform hop spacing; FCC 15.247(a)(1) requires >= 25 kHz. |
| SX1276_FHSS_FIRST_CENTER_HZ | `902750000UL` | `_Static_assert >= BAND_LOWER` | `firmware/murata_l072/include/sx1276_fhss_chantab.h:34` | live | Channel 0 centre; 750 kHz guard from the lower band edge. |
| SX1276_FHSS_LAST_CENTER_HZ | derived `927250000` | `_Static_assert <= BAND_UPPER` | `firmware/murata_l072/include/sx1276_fhss_chantab.h:35` | live | Channel 49 centre; 750 kHz upper guard. |
| SX1276_FHSS_BAND_LOWER_HZ | `902000000UL` | n/a | `firmware/murata_l072/include/sx1276_fhss_chantab.h:39` | live | ISM lower edge; used only by compile-time asserts. |
| SX1276_FHSS_BAND_UPPER_HZ | `928000000UL` | n/a | `firmware/murata_l072/include/sx1276_fhss_chantab.h:40` | live | ISM upper edge; asserts only. |
| 20 dB OBW static assert | none — commented-out placeholder | n/a | `firmware/murata_l072/radio/sx1276_fhss_chantab.c:23` | **dead** | Planned `SPACING_HZ >= MEASURED_20DB_OBW_HZ` (TODO FCC-EVID-D2). The 500 kHz spacing has never been checked against a measured OBW. |
| SX1276_FHSS_LEGAL_FLOOR | `50U` | `_Static_assert <= CHANNEL_COUNT` | `firmware/murata_l072/include/sx1276_fhss.h:48` | live | Minimum active hopset. Floor == table size, so `blacklist()` is **always** refused today. |
| SX1276_FHSS_WARMUP_HOPS | `50U` | n/a | `firmware/murata_l072/include/sx1276_fhss.h:52` | live | One full epoch of cold-start warm-up during which blacklist() returns BLACKLIST_WARMUP. |
| SX1276_FHSS_SNAP_MAX_EPOCH_DRIFT | `1U` | compile-time | `firmware/murata_l072/include/sx1276_fhss.h:141` | live | Max epoch delta `sx1276_fhss_consider_remote()` accepts before rejecting a peer header as spoof/replay. With no MIC in schema v1 this is the **only** spoof barrier. |
| FNV1A_OFFSET_BASIS_32 | `0x811C9DC5UL` | n/a | `firmware/murata_l072/radio/sx1276_fhss.c:21` | live | FNV-1a basis for seed = H(farm_id, node_id, epoch) over 20 LE bytes. |
| FNV1A_PRIME_32 | `0x01000193UL` | n/a | `firmware/murata_l072/radio/sx1276_fhss.c:22` | live | FNV-1a prime for the hop-sequence seed. |
| XORSHIFT32_NONZERO_FALLBACK | `0x9E3779B9UL` | n/a | `firmware/murata_l072/radio/sx1276_fhss.c:25` | live | Substitute RNG seed when the FNV hash is 0 (xorshift32 collapses on zero state). |
| FHSS seed inputs (farm_id, node_id, epoch) | `sx1276_fhss_init(0ULL, 0ULL, 0U)` — all zero at the only production call site | API accepts any u64/u64/u32 | `firmware/murata_l072/host/host_cfg_profile.c:194` | live | **Every node in the fleet derives the identical hop permutation.** No per-farm or per-node diversity, and no CFG key reaches these arguments. |
| SX1276_FHSS_SLOT_MS | `200U` | compile-time | `firmware/murata_l072/include/sx1276_fhss_clock.h:52` | live | Slot grid: 5 slots/s, one 50-slot epoch per 10 s. |
| FHSS epoch period (derived) | `200 × 50 = 10000 ms` | pinned == 10000 by the bench check | `firmware/murata_l072/bench/host_proto/fhss_clock_test.c:104` | live | Each channel revisited once per 10 s — the geometry that keeps occupancy inside 400 ms/10 s. |
| SX1276_FHSS_SLOT_TX_GUARD_US | `15000UL` | compile-time | `firmware/murata_l072/include/sx1276_fhss_clock.h:58` | live | TX may key only if ToA + guard fits the remaining slot (`sx1276_tx.c:530`); else the frame parks in the mailbox. |
| SX1276_FHSS_SLOT_TX_HEADSTART_MS | `12U` | compile-time | `firmware/murata_l072/include/sx1276_fhss_clock.h:68` | live | TX waits this far into a slot so the receiver's boundary retune finishes first (run-20 fix). |
| sx1276_fhss_clock anchor state | anchor_ms 0, anchor_abs 0, valid 0 after reset | u32 ms; anchor younger than ~49.7 d | `firmware/murata_l072/radio/sx1276_fhss_clock.c:16` | live | TX anchors lazily on its first FHSS TX; RX re-anchors on every accepted header as `now − toa_ms − slot_offset_ms`. |
| TCXO drift assumption | ±2 ppm (~0.12 ms/min vs a 200 ms slot) | n/a | `firmware/murata_l072/include/sx1276_fhss_clock.h:18` | diagnostic | Documentation only — not encoded as a constant anywhere. |
| abs_slot u32 wrap horizon | ~85.9 M epochs ≈ 27 years | n/a | `firmware/murata_l072/include/sx1276_fhss_clock.h:29` | diagnostic | Documented bound; no runtime check, unreachable in practice due to re-anchoring. |
| LORA_PKT_HDR_LEN | `8U` | n/a | `firmware/murata_l072/include/lora_pkt_hdr.h:47` | live | Hop-sync header on every routed TX frame; included in every length-bearing calculation. |
| LORA_PKT_HDR_SCHEMA_VER | `1U` | unpack refuses any other value with BAD_SCHEMA | `firmware/murata_l072/include/lora_pkt_hdr.h:46` | live | Header schema (schema_ver, profile_id, hop_idx, slot_offset_ms, epoch LE32). v2 will add a MIC; there is none today. |
| slot_offset_ms header field | saturated at 255 | 0..255 (u8) | `firmware/murata_l072/radio/sx1276_tx.c:222` | live | ms from the sender's slot boundary to key-up; lets a receiver phase-lock from one decode. Saturation unreachable at SLOT_MS 200. |
| SX1276_DWELL_CHANNEL_COUNT | `64U` | `_Static_assert <= 64` | `firmware/murata_l072/include/sx1276_legal_dwell.h:69` | live | Legal-dwell buckets. Asymmetry: dwell tracks 64 channels, hop table and SUMMARY URC only 50. |
| SX1276_DWELL_WINDOW_10S_MS | `10000UL` | window_ms must be in (0, 20000] | `firmware/murata_l072/include/sx1276_legal_dwell.h:70` | live | The enforced FCC narrowband window; the only one the production TX path passes (`sx1276_tx.c:370`). |
| SX1276_DWELL_WINDOW_20S_MS | `20000UL` | upper bound of accepted window_ms | `firmware/murata_l072/include/sx1276_legal_dwell.h:71` | live | Read on the production reserve path as the BAD_WINDOW gate (`sx1276_legal_dwell.c:139`) and the ring-slot reuse horizon (`:89`). No production caller passes it as an enforcement window. |
| SX1276_DWELL_DEFAULT_CAP_US | `400000UL` | cap must be non-zero and >= pessimistic_us | `firmware/murata_l072/include/sx1276_legal_dwell.h:72` | live | 400 ms per-channel budget in the 10 s window; passed verbatim at `sx1276_tx.c:371`. reserve() returns OVER_BUDGET and books nothing when exceeded. |
| SX1276_DWELL_RING_CAPACITY | `128U` | `_Static_assert <= 128` (7 handle bits) | `firmware/murata_l072/include/sx1276_legal_dwell.h:73` | live | Single shared event ring across all 64 channels; full ring → RING_FULL. |
| SX1276_DWELL_HANDLE_INVALID | `0xFFFFU` | n/a | `firmware/murata_l072/include/sx1276_legal_dwell.h:74` | live | Sentinel on any failed reserve; reconcile() drops it silently. |
| DWELL_HANDLE_SLOT_MASK / GEN_SHIFT / GEN_MASK | `0x007F` / `7U` / `0x01FF` | generation wraps mod 512 | `firmware/murata_l072/radio/sx1276_legal_dwell.c:29` | live | Handle encoding [9-bit generation, 7-bit slot] so a stale reconcile against a reused slot is dropped. |
| legal-dwell window inclusion rule | half-open: included iff `now − start < window` | tolerates one u32 wrap while window < 2^31 ms | `firmware/murata_l072/radio/sx1276_legal_dwell.c:53` | live | `start == now − W` excluded; `now − W + 1` included. Unsigned subtraction absorbs the ms wrap. |
| legal-dwell reconcile policy | monotonically downward only; no release() API | n/a | `firmware/murata_l072/radio/sx1276_legal_dwell.c:200` | live | Pessimistic at TX-start, shrunk only if actual < reserved. Never rolled back on NACK/TX_FAIL/TX_TIMEOUT. |
| legal-dwell reconcile source | `s_expected_toa_us` (predicted, not measured) | n/a | `firmware/murata_l072/radio/sx1276_tx.c:464` | live | The L072 wiring exposes no measured airtime, so FCC-A1b prediction is reconciled on TX_DONE. |
| legal-dwell reserve gating | skipped for p0 and p2; enforced only for p1 | n/a | `firmware/murata_l072/radio/sx1276_tx.c:363` | live | p0 is out of scope, p2 is PSD-based with no dwell limit. |
| Legal-dwell / FHSS bypass (BENCH, DTS) | both skip the hop scheduler (`:187`) and the dwell reserve (`:363`); channel_idx forced 0 | profile enum 0..2 | `firmware/murata_l072/radio/sx1276_tx.c:363` | live | Lets bench/DTS key the radio without burning budget; the radio never retunes off the last-written FRF. |
| sx1276_legal_dwell_reset() | n/a | n/a | `firmware/murata_l072/radio/sx1276_legal_dwell.c:97` | diagnostic | Zeroes the dwell ring and peaks. **No firmware caller** — booked dwell survives a profile change on-target. |
| per-channel dwell peak sidecar | high-water of (used_before + pessimistic), zeroed on snapshot | 64 entries | `firmware/murata_l072/radio/sx1276_legal_dwell.c:170` | live | Peak evidence per SUMMARY window; only successful reservations update it. |
| per-channel hop counter saturation | u16 in memory, `0xFF` on the wire | n/a | `firmware/murata_l072/radio/sx1276_fhss.c:291` | live | FCC-B1-SUMMARY equal-use evidence; snapshot saturates each entry to 0xFF (`:308`). |
| per-channel LBT block counter | saturates at UINT32_MAX | idx must be < 50 | `firmware/murata_l072/radio/sx1276_fhss.c:273` | live | Pure observability; explicitly decoupled from blacklist() so it can never affect scheduling. |
| RX scan walker order | linear 0..49 then wrap; reset to 0 on every BEGIN_SCAN | 0..49 | `firmware/murata_l072/radio/sx1276_rx_scan_walker.c:21` | live | Cold-start scan order, deliberately separate from the TX-side slot pointer. |
| SX1276_RX_SCAN_LOCK_LOSS_MS | `2000U` | n/a | `firmware/murata_l072/include/sx1276_rx_scan_policy.h:77` | live | A LOCKED receiver with no valid frame for 2 s is demoted to SCANNING and the slot clock is reset. |
| SX1276_RX_SCAN_GOAL_MS | `5000U` | n/a | `firmware/murata_l072/include/sx1276_rx_scan_policy.h:84` | **dead** | Acceptance target for cold-start reacquire; grep finds no reference — the A6c-3 overrun counter was never wired. |
| SX1276_RX_SCAN_REDESIGN_MS | `30000U` | n/a | `firmware/murata_l072/include/sx1276_rx_scan_policy.h:91` | live | Hard abort: 30 s in SCANNING forces FAILED, checked before channel-advance so it always wins. |
| scan SM gating on scheduler init | early return when `sx1276_fhss_is_initialized() == 0` | n/a | `firmware/murata_l072/radio/sx1276_rx.c:604` | live | The whole scan SM only runs under p1, so p0/p2 keep the host-pinned carrier. |
| slot-follow interlocks | requires clock_valid, LOCKED, `!tx_busy`; skips if `abs_now == last_followed_abs` | n/a | `firmware/murata_l072/radio/sx1276_rx.c:690` | live | Boundary follower retunes once per slot regardless of decode success — one air loss costs exactly one packet. |
| LBT_BASE_BACKOFF_MS | `10U` | shift capped at 6 → raw max 640 ms before the max clamp | `firmware/murata_l072/radio/sx1276_lbt.c:18` | live | `backoff = 10 << min(attempt, 6)`, then clamped to max_backoff_ms. |
| LBT backoff shift cap | `6U` | attempt counter saturates at 0xFF | `firmware/murata_l072/radio/sx1276_lbt.c:58` | live | Caps exponential growth of the LBT backoff. |
| DEFAULT_LBT_THRESHOLD_DBM | `0xA6` = −90 dBm | validator −120..0 | `firmware/murata_l072/host/host_cfg.c:25` | live | Boot default RSSI busy threshold; consumed at `sx1276_lbt.c:84`. |
| DEFAULT_LBT_MAX_BACKOFF_MS | `500U` | validator 10..5000 | `firmware/murata_l072/host/host_cfg.c:26` | live | Boot default max backoff; consumed at `sx1276_lbt.c:86`. |
| DEFAULT_CAD_SYMBOLS | `4U` | validator 1..10 | `firmware/murata_l072/host/host_cfg.c:27` | live | Boot default CAD count. Note `sx1276_lbt.c:88` uses its own `LBT_DEFAULT_CAD_SYMBOLS` fallback, not this macro. |
| DEFAULT_FHSS_DWELL_MS | `400U` | validator 50..2000 | `firmware/murata_l072/host/host_cfg.c:29` | live | Seeds CFG key 0x13. Nothing consumes the stored value (see §10). |
| HOST_RFCO_PERTX_SCHEMA_VER | `1U` | must be bumped on any field addition | `firmware/murata_l072/include/host_rfco.h:81` | live | Byte 0 of every RFCO_PERTX_URC; mirrored into the SUMMARY snapshot. |
| HOST_RFCO_PERTX_PAYLOAD_LEN | `21U` | fixed wire contract | `firmware/murata_l072/include/host_rfco.h:82` | live | Per-TX compliance URC length. |
| host_rfco_tx_status_t | OK 0, ABORT_AIRTIME_INVARIANT 1, ABORT_LBT 2, ABORT_LEGAL_DWELL 3, ABORT_QOS 4, TX_TIMEOUT 5, TX_FAIL 6, INTERNAL 0xFF | 0..6 plus 0xFF | `firmware/murata_l072/include/host_rfco.h:84` | live | Fail-closed gate attribution per TX; also the index into `blocked_attempts_by_reason[8]` (0xFF folds into slot 7). |
| HOST_RFCO_BLOCKED_ATTEMPTS_SLOTS | `8U` | must be >= the reason-code count | `firmware/murata_l072/include/host_rfco.h:163` | live | Per-reason blocked-attempt counters. |
| HOST_RFCO_SUMMARY_PERIOD_MS | `60000U` | n/a | `firmware/murata_l072/include/host_rfco_summary.h:89` | live | Cadence of the per-minute compliance URC, polled from `main.c:196`. |
| HOST_RFCO_SUMMARY_SCHEMA_VER | `1U` | must be bumped on field addition | `firmware/murata_l072/include/host_rfco_summary.h:85` | live | Pinned into byte 0 of the SUMMARY payload. |
| HOST_RFCO_SUMMARY_PAYLOAD_LEN | `191U` | `_Static_assert == 191` and `<= HOST_PAYLOAD_MAX_LEN` | `firmware/murata_l072/include/host_rfco_summary.h:86` | live | Fixed SUMMARY length. |
| HOST_RFCO_SUMMARY_CHANNEL_COUNT | `50U` | pinned == SX1276_FHSS_CHANNEL_COUNT | `firmware/murata_l072/include/host_rfco_summary.h:92` | live | Channel-table width for hop counts and dwell peaks; the 64-entry dwell array is truncated to 50. |
| HOST_RFCO_SUMMARY_REASON_SLOTS | `8U` | `_Static_assert >= 7U` | `firmware/murata_l072/include/host_rfco_summary.h:93` | live | Blocked-attempt reason buckets in the SUMMARY payload. |
| HOST_RFCO_SUMMARY_LAST_CLAMP_REASON_NONE | `0xFF` | compile-time | `firmware/murata_l072/include/host_rfco_summary.h:117` | live | "No non-OK TX this window" sentinel; duplicated as `HOST_RFCO_LAST_CLAMP_NONE` at `host_rfco.c:125`. |
| HOST_TYPE_AIRTIME_INVARIANT_REJECT_URC | `0xC2`, 15-byte payload | HOST_AIRTIME_REJECT_PAYLOAD_LEN = 15 | `firmware/murata_l072/include/host_types.h:59` | live | {SF, CR, BW, payload_len, computed ToA, cap} emitted from `sx1276.c:442`. |
| HOST_TYPE_RFCO_PERTX_URC | `0xC3`, 21-byte payload | schema_ver 1 | `firmware/murata_l072/include/host_types.h:93` | live | Per-TX compliance snapshot emitted from `sx1276_tx.c:100`. |
| HOST_TYPE_RFCO_SUMMARY_URC | `0xC4`, 191-byte payload | schema_ver 1 | `firmware/murata_l072/include/host_types.h:117` | live | Per-minute summary: 50-bucket hop histogram, 50 dwell peaks, 8 reason counters, trailing CRC-16. |
| per-region conditional | none | n/a | `firmware/murata_l072/include/host_cfg_keys.h:43` | **dead** | There is **no** region/locale abstraction in the L072 tree. Grep for region/ETSI/EU868/868/ARIB returns only flash-region hits. |
| comm.lora_region | `"us915"` | enum us915 / eu868 / au915 | `base_station/config/build.default.toml:60` | live | The fleet-wide regulatory region selector; codegens `LIFETRAC_COMM_LORA_REGION_*` into `firmware/common/lifetrac_build_config.h:69`. Does not reach the L072 profile machinery. |
| CMD_OP_RADIO_PROFILE | `0x65` | profile 0..2 | `base_station/lora_proto.py:958` | live | Strict-path 0xFB opcode; base commands the tractor to switch profile (phase A). |
| CMD_OP_RADIO_PROFILE_ACK | `0x66` | profile 0..2 | `base_station/lora_proto.py:959` | live | Tractor→base, sent on the OLD grid before the tractor switches. |
| CMD_OP_RADIO_PROFILE_CONF | `0x67` | profile 0..2 | `base_station/lora_proto.py:960` | live | Base→tractor proof-of-life on the NEW grid; absence triggers revert. |
| configure_regulatory_profile_if_needed | `profile_id=1` arg default; both daemons call it with no arg so the env decides | profile 0..2 | `firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py:425` | live | The one place any daemon issues configuration: CFG 0x07, 0x15, 0x16, **0x17, 0x18**, 0x14, then CFG_GET 0x14 readback and optional RegFrf pinning. Only keys 0x07/0x14/0x15/0x16/0x17/0x18 and 0x03 are ever issued in production. **Ordering is load-bearing as of 2026-07-29:** the seed keys 0x17/0x18 MUST precede 0x14, because the firmware consumes the seed only inside `host_cfg_profile_activate()` and there is no re-seed path short of re-activating the profile. |
| LIFETRAC_REG_PROFILE (base) | `0` | clamped 0..2 by `_env_int` | `base_station/image_rx_daemon.py:279` | live | Regulatory profile the RX daemon requests and sizes fragments against; also rewritten in-process by `_apply_profile` (`:859`). |
| LIFETRAC_REG_PROFILE (tractor) | `0` | clamped 0..2 at every read site | `firmware/tractor_x8/image_tx_daemon.py:372` | live | Re-read per frame in `_pack_for`/`_tx_one_frame`; rewritten at `:844` by `_apply_profile`. |
| LIFETRAC_FHSS_WIDE_MASK (helper) | unset (narrow single-channel mask) | compared `== "1"` | `firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py:460` | live | "1" sends the 50-channel mask `ff ff ff ff ff ff 03 00` required by p1. |
| LIFETRAC_FHSS_WIDE_MASK (base writer) | unset | "1" or absent | `base_station/image_rx_daemon.py:864` | live | Set when applying p1, popped otherwise. |
| LIFETRAC_FHSS_WIDE_MASK (tractor writer) | unset | "1" or absent | `firmware/tractor_x8/image_tx_daemon.py:846` | live | Same write-only role on the tractor side. |
| LIFETRAC_FHSS_CHANNEL | `0` | 0..49; silently coerced to 0 outside that range or on parse error | `firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py:465` | live | Single-channel index used to build the narrow mask `1 << ch`. |
| RADIO_PROFILE_TOPIC | `lifetrac/v25/control/radio_profile` | unconstrained | `base_station/image_rx_daemon.py:164` | live | Retained {profile, source, ts}; subscribed QoS 1 and converted to a 0xFB command. |
| RADIO_PROFILE_ACK_TOPIC (rx) | `lifetrac/v25/status/radio_profile/rx` | unconstrained | `base_station/image_rx_daemon.py:165` | live | Retained RX-side profile ack. |
| RADIO_PROFILE_TX_ACK_TOPIC | `lifetrac/v25/status/radio_profile/tx` | unconstrained | `base_station/image_rx_daemon.py:166` | live | Tractor-side ack synthesized from the over-air 0x66 (no LAN path to the tractor in the field). |
| _PROFILE_CHOICES (base) | `(0, 1, 2)` | {0,1,2} | `base_station/image_rx_daemon.py:170` | live | Anything else on the control topic is logged and dropped. |
| _PROFILE_CHOICES (tractor) | `(0, 1, 2)` | n/a | `firmware/tractor_x8/image_tx_daemon.py:304` | live | Whitelist of ids accepted from an inbound LoRa RADIO_PROFILE command. |
| Phase-A re-send interval | 1.5 s | inline literal | `base_station/image_rx_daemon.py:504` | live | Re-transmit the RADIO_PROFILE command while awaiting ACK — the tractor only listens in inter-fragment gaps. |
| drain_pending (profile apply) | quiet_s 0.2, max_s 1.0 | unconstrained | `base_station/image_rx_daemon.py:867` | live | Quiesce the HostLink before reconfiguring the profile mid-run. |
| RadioProfileBody.profile | required | `^(auto or 0 or 1 or 2)$` | `base_station/web_ui.py:130` | live | POST /api/settings/radio_profile selection. |
| LIFETRAC_RADIO_PROFILE_STORE | `<module dir>/.radio_profile_override` | contents validated against the UI choices | `base_station/web_ui.py:312` | live | Persisted UI selection; unreadable/unrecognised content falls back to "2". |
| _RADIO_PROFILE_UI_CHOICES | `("auto", "0", "1", "2")` | exactly those 4 | `base_station/web_ui.py:319` | live | Selectable radio-profile values in the settings UI. |
| _RADIO_PROFILE_LABELS | 0 Fixed 915 (bench), 1 FHSS 50ch BW250, 2 DTS BW500, auto Auto | n/a | `base_station/web_ui.py:320` | live | Human labels for GET /api/settings/radio_profile. |
| _radio_profile_mode | `"2"` | UI choices | `base_station/web_ui.py:327` | live | Module-default UI selection before the store is read (`:621`). |
| _radio_profile_active | `2` | {0,1,2} | `base_station/web_ui.py:328` | live | Concrete profile last commanded/observed; kept truthful across restarts by capturing the retained control topic (`:976`). |
| _load_radio_profile fallback | `"2"` | n/a | `base_station/web_ui.py:350` | live | Returned when the store is missing, unreadable, or unrecognised. |
| AutoRadioPolicy.STALE_LINK_S | `20.0` | unconstrained | `base_station/web_ui.py:402` | live | link_stats older than this counts as unhealthy. |
| AutoRadioPolicy.TIMEOUT_RATE_MAX | `2.5` timeouts / 10 s | unconstrained | `base_station/web_ui.py:403` | live | Reassembler-timeout rate above which p2 degrades to p1. |
| AutoRadioPolicy.PROMOTE_AFTER_S | `60.0` | unconstrained | `base_station/web_ui.py:404` | live | Continuous-health dwell before promoting p1 → p2. |
| AutoRadioPolicy.MIN_SWITCH_GAP_S | `60.0` | unconstrained | `base_station/web_ui.py:405` | live | Hysteresis between any two auto switches; each reconfig costs an outage. |
| AutoRadioPolicy.initial_profile | `2` | coerced to {1,2} | `base_station/web_ui.py:407` | live | p0 is never auto-selected. |
| _radio_auto_worker tick | `5.0` s | unconstrained | `base_station/web_ui.py:643` | live | Auto-policy evaluation period; defers seeding until the MQTT client is connected so the retained replay wins. |
| AirtimeBudget(budget_us, window_s) | 380000 µs, 1.0 s | n/a | `firmware/tractor_x8/image_tx_daemon.py:256` | live | Host mirror of the firmware QoS gate; `admit()` blocks until the estimated ToA fits. Constructed at `:352` then immediately replaced at `:373` by the profile-derived one. |
| LEGAL_DWELL_US (host mirror) | `400_000` | n/a | `firmware/tractor_x8/image_tx_daemon.py:176` | **dead** | FCC 400 ms dwell cloned from `w2_02_host_pipeline.py`; defined and never referenced. |
| MAX_FRAMES_PER_DWELL_CAP (host mirror) | `8` | n/a | `firmware/tractor_x8/image_tx_daemon.py:177` | diagnostic | Referenced only by the "TX worker ready" log line at `:522`; constrains nothing. |
| DWELL_HEADROOM_PCT (host mirror) | `85` | n/a | `firmware/tractor_x8/image_tx_daemon.py:178` | **dead** | Cloned constant, never referenced. |
| RegProfile (radio monitor script) | `0` | 0/1/2 documented, **not validated** | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:59` | live | Regulatory profile pushed to both daemons for a bench run. |
| LIFETRAC_FHSS_WIDE_MASK (script derive) | forced to 1 only when RegProfile == 1 | derived, not user-settable | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:92` | live | Auto-enables the wide mask because the p1 validator rejects popcount<50. Never recorded in params.txt. |
| RegProfile (RSSI sniff) | `"0"` (a string here, unlike the monitor's int) | 0/1/2 unvalidated | `firmware/x8_lora_bootloader_helper/run_air_coupling_rssi_sniff.ps1:31` | live | LIFETRAC_REG_PROFILE for the sniff run. |

---

## 3. Firmware compile-time

Build flags, memory maps, version strings, crypto build axis, and the H7/L072 toolchain settings.

| Setting | Default | Range | Where | Status | Purpose |
| --- | --- | --- | --- | --- | --- |
| LIFETRAC_FHSS_TX_ROUTED | `1` — hard-set in CFLAGS, not config.h | defined / undefined | `firmware/murata_l072/Makefile:112` | live | Master gate for the hop scheduler, slot clock, 8-byte hop header, legal-dwell and scan SM. Undefined degrades TX/RX to a single fixed channel with no header. |
| L072 CFLAGS `-DLIFETRAC_FHSS_TX_ROUTED=1` | present unconditionally | 0/1 | `firmware/murata_l072/Makefile:113` | live | The value is fixed at build-system level with no override hook — edit the Makefile or override CFLAGS. |
| LIFETRAC_BENCH_RADIO_IDLE_SLEEP | `0` | 0/1 | `firmware/murata_l072/config.h:100` | live | 1 parks the radio in LoRa SLEEP at boot (`main.c:69`) instead of arming RX-continuous. |
| HOST_DEBUG_OPMODE_GUARD | `0` | 0/1 | `firmware/murata_l072/config.h:76` | **dead** | 1 would force SX1276_STATE_FAULT on an OpMode readback mismatch (`sx1276_modes.c:129`). Compiled out. |
| HOST_ALLOW_REG_WRITE_DIAG | `1` | 0/1 | `firmware/murata_l072/config.h:61` | live | Enables host REG_WRITE_REQ — the out-of-band path that can change RegModemConfig behind the airtime invariant, which is exactly why the per-frame FCC-A1b check exists. |
| LORA_FW_LBT_ENABLE | `1` | 0/1 | `firmware/murata_l072/config.h:30` | live | Compile-time LBT default; seeds CFG key 0x03 and is the runtime fallback at `sx1276_lbt.c:83`. |
| LORA_FW_QUALITY_AWARE_FHSS | `0` | 0/1 | `firmware/murata_l072/config.h:27` | deprecated | Seeds CFG key 0x06, whose apply handler is `cfg_apply_unsupported` — the feature cannot be turned on. |
| LORA_FW_TX_POWER_ADAPT | `1` | 0/1 | `firmware/murata_l072/config.h:33` | live | Referenced once, as the default byte for CFG key 0x02 (`host_cfg.c:95`). The stored value is never read, so no adaptation happens. |
| LORA_FW_CRYPTO_IN_L072 | `0` | 0/1; validator rejects >1 | `firmware/murata_l072/config.h:24` | live | Selects crypto location (Profile A = H7 owns AES-GCM); seeds CFG key 0x0E, whose apply fn always fails. |
| LORA_FW_AB_SLOTS | `0` | documented 0=single-slot, 1=A/B; not enforced anywhere | `firmware/murata_l072/config.h:21` | **dead** | Intended A/B firmware slot Flash layout (N-26, Phase 6). |
| FINAL_RELEASE_BUILD | `0` | 0/1 | `firmware/murata_l072/include/dev_build_policy.h:10` | **dead** | Intended dev-vs-release master switch; referenced by zero lines of code. |
| MM_FLASH_SIZE | `192 * 1024` | chip capacity | `firmware/murata_l072/include/memory_map.h:30` | live | Drives MM_FLASH_END, MM_APP_SIZE, the linker MEMORY block and the ld ASSERTs. |
| MM_RAM_SIZE | `20 * 1024` | chip capacity | `firmware/murata_l072/include/memory_map.h:34` | live | Drives MM_RAM_END / stack placement and the stack-fits assert. |
| MM_FLASH_PAGE_SIZE | `128` | silicon constant | `firmware/murata_l072/include/memory_map.h:47` | live | Used only by the CFG page-alignment `_Static_assert`. |
| MM_REGION_ALIGN | `4 * 1024` | must be a multiple of the page size | `firmware/murata_l072/include/memory_map.h:48` | live | Region boundary alignment for the future A/B split. |
| MM_CFG_BASE | `MM_FLASH_BASE + 0x2E000` | must be 4 KB aligned and BASE+SIZE == FLASH_END | `firmware/murata_l072/include/memory_map.h:72` | live | Start of the runtime config/calibration Flash region; feeds the linker CFG region. |
| MM_CFG_SIZE | `8 * 1024` | whole number of Flash pages | `firmware/murata_l072/include/memory_map.h:73` | live | Config Flash region reserved in the linker script. |
| MM_STACK_SIZE | `2560` (2.5 KB) | >= 1024 and < MM_RAM_SIZE | `firmware/murata_l072/include/memory_map.h:91` | live | Exported to the linker as `_stack_size`. |
| MM_RAM_HEADROOM_MIN | `4 * 1024` | unconstrained | `firmware/murata_l072/include/memory_map.h:98` | **dead** | Intended "firmware grew too much" warning threshold; no C code, no static assert, no ld ASSERT evaluates it. |
| MM_SYSMEM_BASE | `0x1FF00000` | fixed silicon address (AN2606) | `firmware/murata_l072/include/memory_map.h:105` | live | ROM-bootloader system memory; safe-mode fetches SP/PC and sets VTOR from it. |
| BOOT_REGION_SIZE | `4 KiB` | legacy | `firmware/murata_l072/include/flash_map.h:16` | **dead** | Superseded A/B map. `flash_map.h` is included only by an orphaned linker script the Makefile never builds. |
| SLOT_A_SIZE | `176 KiB` | legacy | `firmware/murata_l072/include/flash_map.h:19` | **dead** | Contradicts the live map (MM_APP_SIZE 192 KB, MM_CFG_SIZE 8 KB). |
| CONFIG_REGION_SIZE | `12 KiB` | legacy | `firmware/murata_l072/include/flash_map.h:22` | **dead** | Disagrees with the live MM_CFG_SIZE of 8 KB. |
| L072_PAGE_SIZE | `128u` | legacy | `firmware/murata_l072/include/flash_map.h:12` | **dead** | Superseded by MM_FLASH_PAGE_SIZE; referenced by zero lines. |
| FW_VERSION_MAJOR / MINOR / PATCH | `0` / `0` / `0` | u8 on the wire | `firmware/murata_l072/include/version.h:6` | live | Reported over AT+VER? and the binary VER_URC. |
| FW_GIT_SHA | `"dev"` | string; no length assert against the VER_URC buffer | `firmware/murata_l072/include/version.h:11` | live | Build provenance. Hardcoded placeholder — the Makefile never overrides it, so **every image reports "dev"**. |
| OSE_LIFETRACLORA_FW_NAME | `"OSE-LifeTracLORA-MurataFW"` | string | `firmware/murata_l072/include/version.h:4` | live | Firmware identity string in ATI/AT+VER?/VER_URC. |
| CROSS | `arm-none-eabi-` | any valid prefix | `firmware/murata_l072/Makefile:18` | live | Toolchain prefix; `build.ps1` overrides with the resolved Arduino 7-2017q4 path. |
| OPT_FLAGS | `-Os -ffunction-sections -fdata-sections` | any gcc flags | `firmware/murata_l072/Makefile:104` | live | Paired with `--gc-sections` to keep the image inside 192 KB. |
| WARN_FLAGS | `-Wall -Wextra -Werror -Wno-unused-function -Wno-unused-variable` | any gcc flags | `firmware/murata_l072/Makefile:99` | live | The two `-Wno-unused-*` are what let several dead defines survive `-Werror` unnoticed. |
| MCU_FLAGS | `-mcpu=cortex-m0plus -mthumb -mfloat-abi=soft` | fixed for STM32L072CZ | `firmware/murata_l072/Makefile:94` | live | Applied to both CFLAGS and LDFLAGS. |
| L072 LDFLAGS | `-nostartfiles -Wl,--gc-sections -specs=nano.specs -specs=nosys.specs` | n/a | `firmware/murata_l072/Makefile:133` | live | newlib-nano selection and dead-section stripping the flash budget depends on. |
| LD_NO_WARN_RWX | `0` | 0/1 | `firmware/murata_l072/Makefile:128` | live | 1 adds `-Wl,--no-warn-rwx-segments`; off because the bundled 7-2017q4 ld lacks the flag. |
| unit_id / schema_version | `"lifetrac-001"` / `1` | unit_id `^[a-z0-9-]{3,32}$`; schema >= 1 | `base_station/config/build.default.toml:9` | live | Per-unit identity selecting the `build.<unit_id>.toml` override; baked into the firmware header. |
| LIFETRAC_CONFIG_SHA256_HEX (+ _SHORT) | `079dd573…4565` / `079dd573` | 64 hex chars | `firmware/common/lifetrac_build_config.h:12` | live | Digest of the validated build config so a running node can prove which config it was built from. |
| LIFETRAC_KEY_ID | `0xAEFBF982UL` | any uint32 | `firmware/common/lora_proto/key.h:9` | live | 32-bit fleet key identifier in the crypto envelope; generated by `tools/provision.py`. |
| LIFETRAC_FLEET_KEY_BYTES | 16-byte literal in `key.h`; all-zero fallback if absent | 16 bytes | `firmware/common/lora_proto/key.h:10` | live | The AES-128 key compiled into MCU firmware. Header is marked "do not commit" yet a real key is committed at that path. |
| LIFETRAC_USE_REAL_CRYPTO / LIFETRAC_ALLOW_STUB_CRYPTO | neither defined → `#error` at compile time | defined / undefined | `firmware/common/lora_proto/crypto_stub.c:28` | live | Selects real AES-GCM vs the no-op placeholder; the stub path requires explicit opt-in acknowledging a bench-only build. |
| LIFETRAC_ALLOW_UNCONFIGURED_KEY | unset — bridge refuses to start with an all-zero key | "1" enables | `base_station/lora_bridge.py:155` | live | Safety escape hatch for bench work; also an Arduino `-D` flag in CI. |
| LIFETRAC_FORCE_MBEDTLS / LIFETRAC_FORCE_RWEATHER_CRYPTO | undefined (auto-select) | defined / undefined | `firmware/bench/crypto_vectors/host_check.c:30` | diagnostic | Forces a specific AES-GCM backend for the known-answer vector harness. |
| LIFETRAC_METHOD_G_HOST_BUILD / LIFETRAC_USE_METHOD_G_HOST | `0` unless `=1` is passed | 0/1 | `firmware/tractor_h7/tractor_h7.ino:23` | live | Compiles the H7 murata_host (Method G) HostLink driver into the tractor sketch instead of driving the radio directly. |
| LIFETRAC_MH_SERIAL | none — `#error` if Method G is on and this is undefined | a HardwareSerial identifier | `firmware/tractor_h7/tractor_h7.ino:49` | live | Names the serial carrying the H7→L072 HostLink (CI uses Serial1). |
| LIFETRAC_MH_SOFT_BRIDGE / _HEARTBEAT_MS | `0` / `0` | 0/1; heartbeat ms (0 disables) | `firmware/tractor_h7/tractor_h7.ino:29` | diagnostic | Turns the H7 into a transparent X8↔L072 bridge with an optional periodic heartbeat. |
| LIFETRAC_MH_BENCH_LOG / _BRIDGE_DEBUG_SERIAL | BENCH_LOG 1 (0 under SOFT_BRIDGE); DEBUG_SERIAL = SerialRPC on X8 else Serial | 0/1; a Serial identifier | `firmware/tractor_h7/tractor_h7.ino:54` | diagnostic | Bench logging verbosity and debug stream selection. |
| SERIAL2_TX / RX / RTS / CTS | `PA_15` / `PF_6` / `PF_8` / `PF_9` | STM32 pin names | `firmware/tractor_h7/tractor_h7.ino:80` | live | Pin map for the second UART the stock PORTENTA_X8 core does not expose; required for Method G. |
| OPTA_SLAVE_ID | `0x01` | 1..247 | `firmware/tractor_h7/tractor_h7.ino:148` | live | Modbus RTU slave address of the Opta valve controller. |
| X8_CMD_TOPIC (H7) | `0xC0` | uint8 | `firmware/tractor_h7/tractor_h7.ino:1632` | live | Topic byte the H7 uses forwarding REQ_KEYFRAME / CAMERA_SELECT to the X8 over Serial1. |
| LIFETRAC_SHARED_ADDR / _VERSION / _ESTOP_MAGIC | `0x38000000` / `2` / `0xA5A5A5A5` | n/a | `firmware/common/shared_mem.h:20` | live | M7↔M4 shared-memory contract; the E-stop magic is chosen so SRAM noise or stale boot bytes cannot trip the safety chain. Must agree with RAM_D3 in the linker script. |
| LIFETRAC_SHARED_RADIO_MAGIC + counter slots | `0x52414430` ("RAD0"); 11 slot indices | slot idx 0..10 | `firmware/common/shared_mem.h:57` | diagnostic | Layout for the M4-visible bench radio counters written by `bench_radio_set()`. |
| tractor_h7 M7 linker MEMORY / CM4_BINARY_START | FLASH 0x8040000; RAM 0x24000000/512K; RAM_D2 0x30000000/288K; RAM_D3 0x38000000/64K | n/a | `firmware/tractor_h7/linker_script_x8_m7.ld:3` | live | H7 memory map, dual-core split point, and the OpenAMP reservation bounding where `shared_mem` at 0x38000000 can live. |
| openocd adapter speed / reset_config | `adapter speed 1000`; `reset_config srst_nogate`; CHIPNAME stm32l072cz | kHz | `firmware/murata_l072/openocd/stm32l0_swd.cfg:7` | live | SWD clock (conservative for long jumper harnesses) used by every flash/recover run. |
| ImageRemote / Cycles / RunPostListen / PostListenSec / RunRevive | `/tmp/lifetrac_p0c/mlm32l07x01.bin` / 20 / 1 / 10 / 0 | RunPostListen and RunRevive are int-typed booleans, not switches | `firmware/x8_lora_bootloader_helper/run_single_board_stress_end_to_end.ps1:3` | live | Single-board flash/stress loop. |
| LocalImage / Build (method G stage 1) | `""` / off | empty LocalImage uses the repo default artifact | `firmware/x8_lora_bootloader_helper/run_method_g_stage1_end_to_end.ps1:4` | live | Stage-1 orchestrator: optional image override and build-before-flash. |
| DelaysMs / BurstsPerDelay / AttemptsPerBurst / BurstPassMinAck / OpenOcdLifetimeS / PollSec / MaxAdbRecovery / MaxMonitorMinutes | `"2,5,10"` / 20 / 1 / 1 / 75 / 20 / 6 / 120 | DelaysMs is a comma-separated ms sweep | `firmware/x8_lora_bootloader_helper/run_rom_baseline_burst_matrix_hardened.ps1:4` | diagnostic | L072 ROM-bootloader burst-entry matrix and its watchdog limits. |
| LIFETRAC_BOARD_MKR_WAN1310 / _PORTENTA_MAX | none (one must be selected) | defined / undefined | `arduino_libraries.txt:32` | diagnostic | Board selector for the `lora_retune_bench` sketch. |
| LIFETRAC_L072_SERIAL / LIFETRAC_ENABLE_BOOT_PINS | `Serial2` / `0` | Serial identifier; 0/1 | `firmware/portenta_m7_l072_passthrough_ping/portenta_m7_l072_passthrough_ping.ino:43` | diagnostic | Which UART the passthrough ping probe uses and whether it drives L072 BOOT0/NRST. |
| LIFETRAC_JTAG_SWD_ISO_LEVEL | `LOW` | LOW / HIGH | `firmware/x8_uart_route_probe/x8_uart_route_probe.ino:16` | diagnostic | Level driven on the JTAG/SWD isolation net during the X8 UART route probe. |
| LIFETRAC_ENABLE_DEV_RADIO | documented as off; **no implementation** | n/a | `MASTER_PLAN.md:298` | **dead** | Documented as a hard safety policy gating tractor WiFi/BT and `diag_web.py`; grep finds no `#ifdef` or env read. The policy is unenforced. |
| LIFETRAC_RADIO_DISABLE | none | n/a | `HIL_RUNBOOK.md:159` | **dead** | Runbook tells operators to use this flag for RF-silent HIL; no such define exists, so the documented procedure cannot be followed. |
| LIFETRAC_REGION_EU | none | n/a | `KEY_ROTATION.md:94` | **dead** | Troubleshooting table blames this define for "no link forever"; it does not exist. The real region knob is `comm.lora_region`. |
| LIFETRAC_ROLE | none | n/a | `HIL_RUNBOOK.md:269` | **dead** | Runbook references `LIFETRAC_ROLE=base`; unimplemented. Role selection is really `PING_ROLE` plus separate sketch targets. |

---

## 4. Host protocol — CFG keys and opcodes

HostLink framing, the 25-key CFG table, request/URC opcodes, and the H7-side mirror of the same wire contract.
RFCO compliance-URC constants live in [§2](#2-regulatory--fhss).

### 4.1 CFG keys

| Setting | Default | Range | Where | Status | Purpose |
| --- | --- | --- | --- | --- | --- |
| CFG_KEY_COUNT | `25U` | `_Static_assert` against the table size (`host_cfg.c:155`) | `firmware/murata_l072/include/host_cfg_keys.h:73` | live | Descriptor table size. **Bumped 23 → 25 on 2026-07-29** by the FHSS seed keys 0x17/0x18. Keys 0x0F and 0x12 are intentionally unallocated. |
| CFG_KEY_MAX_VALUE_LEN | `8U` | must cover the widest key kind (U64) | `firmware/murata_l072/include/host_cfg_keys.h:39` | live | Max CFG value width; sizes the descriptor, value and set-path scratch buffers. |
| CFG_FLAG_READ_ONLY | `1U << 0` | bitmask bit 0 | `firmware/murata_l072/host/host_cfg.c:13` | live | Rejects CFG_SET with READ_ONLY; applied to PROTOCOL_VERSION, WIRE_SCHEMA_VERSION, CFG_DIRTY. |
| CFG_FLAG_DEFERRED | `1U << 1` | bitmask bit 1 | `firmware/murata_l072/host/host_cfg.c:14` | live | Returns DEFERRED so the value takes effect after reboot; only CFG_KEY_HOST_BAUD uses it. |
| CFG_KEY_TX_POWER_DBM (0x01) | `14U` | normalised (not rejected) into [2,17] at `host_cfg.c:233`, then capped at the active profile's ERP allowance, clipped again in the radio | `firmware/murata_l072/host/host_cfg.c:24` | live | **CORRECTED 2026-07-29 — the previous entry here stated the opposite.** This is no longer the only setting that programs RegPaConfig: profile activation does too. And it **is** now cross-checked against the profile ERP clamp via `cfg_active_erp_max_dbm()` (`host_cfg.c:218`), so a host can no longer set 17 dBm regardless of declared antenna gain. `cfg_init()` still seeds the table without calling the apply fn, so the boot power is the 14 dBm from `sx1276_init()` — but the first profile activation now reconciles hardware with the table. |
| CFG_KEY_TX_POWER_ADAPT_ENABLE (0x02) | `1` (LORA_FW_TX_POWER_ADAPT) | 0..1 | `firmware/murata_l072/host/host_cfg.c:94` | **dead** | Stored but no firmware module reads it. |
| CFG_KEY_LBT_ENABLE (0x03) | `1` (LORA_FW_LBT_ENABLE) | 0/1 | `firmware/murata_l072/host/host_cfg.c:96` | live | Read live by `sx1276_lbt_check_and_backoff` (`sx1276_lbt.c:83`); 0 makes TX proceed unconditionally. |
| CFG_KEY_LBT_THRESHOLD_DBM (0x04) | `0xA6` = −90 dBm | −120..0 signed | `firmware/murata_l072/host/host_cfg.c:98` | live | RSSI busy threshold; read at `sx1276_lbt.c:84`. |
| CFG_KEY_FHSS_ENABLE (0x05) | `1U` (literal in the table) | 0/1 | `firmware/murata_l072/host/host_cfg.c:100` | **dead** | Stored and validated but never read; hopping is decided purely by the active profile. |
| CFG_KEY_FHSS_QUALITY_AWARE (0x06) | `0` | 0/1 at the validator; apply rejects everything | `firmware/murata_l072/host/host_cfg.c:102` | deprecated | Bound to `cfg_apply_unsupported`; every set returns APPLY_FAILED and rolls back. |
| CFG_KEY_FHSS_CHANNEL_MASK (0x07) | `0x00000000000000FF` | wire validator only requires != 0; p1 demands `0x0003FFFFFFFFFFFF` | `firmware/murata_l072/host/host_cfg.c:28` | live | u64 hop mask read at `cfg_set(REG_PROFILE)` (`host_cfg.c:287`), 8 bytes LE. |
| CFG_KEY_DEEP_SLEEP_ENABLE (0x08) | `1` (LORA_FW_DEEP_SLEEP_BUILD) | 0..1 at the validator; apply always fails | `firmware/murata_l072/host/host_cfg.c:109` | **dead** | N-09 deep-sleep RX scheduler; unimplemented. |
| CFG_KEY_BEACON_ENABLE (0x09) | `1` (LORA_FW_BEACON_ENABLE) | 0..1 | `firmware/murata_l072/host/host_cfg.c:111` | **dead** | N-30 emergency beacon; stored only, no reader. |
| CFG_KEY_BEACON_CHANNEL_IDX (0x0A) | `0U` | 0..7 | `firmware/murata_l072/host/host_cfg.c:30` | **dead** | No consumer; the 0..7 range is a leftover from the 8-channel baseline, inconsistent with the 50-channel table. |
| CFG_KEY_HOST_BAUD (0x0B) | `921600` | exactly one of {9600, 115200, 921600} | `firmware/murata_l072/host/host_cfg.c:115` | **dead** | CFG_FLAG_DEFERRED: a successful set returns DEFERRED and **nothing ever re-inits the UART** — `main.c:52` always uses HOST_BAUD_DEFAULT. |
| CFG_KEY_REPLAY_WINDOW (0x0C) | `16` | 1..64 | `firmware/murata_l072/host/host_cfg.c:118` | **dead** | The L072 does no replay checking (crypto is Profile A on the H7). |
| CFG_KEY_IWDG_WINDOW_MS (0x0D) | `100` | 50..5000 | `firmware/murata_l072/host/host_cfg.c:120` | **dead** | No IWDG code exists in the firmware; the value is inert. |
| CFG_KEY_CRYPTO_IN_L072 (0x0E) | `0` | 0..1 at the validator; apply always fails | `firmware/murata_l072/host/host_cfg.c:122` | **dead** | Crypto Profile B (L072 owns AES-GCM); unimplemented. |
| CFG_KEY_LBT_MAX_BACKOFF_MS (0x10) | `500` | 10..5000 | `firmware/murata_l072/host/host_cfg.c:124` | live | Ceiling on the exponential LBT backoff; read at `sx1276_lbt.c:86`. |
| CFG_KEY_CAD_SYMBOLS (0x11) | `4` | 1..10 | `firmware/murata_l072/host/host_cfg.c:126` | live | CAD symbol count for LBT and (reused as ms) the RSSI settle dwell; read at `sx1276_lbt.c:88`. |
| CFG_KEY_FHSS_DWELL_MS (0x13) | `400U` | 50..2000 | `firmware/murata_l072/host/host_cfg.c:29` | **dead** | Range-checked and stored but no consumer; real dwell comes from SX1276_FHSS_SLOT_MS and SX1276_DWELL_DEFAULT_CAP_US. |
| CFG_KEY_REG_PROFILE (0x14) | `0` | 0..2 | `firmware/murata_l072/host/host_cfg.c:130` | live | Synthesises a profile request from the current mask/gain/ceiling, then stage()+activate() in one wire transaction. |
| CFG_KEY_ANTENNA_GAIN_DBI (0x15) | `2` (int8) | no wire clamp — full int8; profile validator enforces [0,30] | `firmware/murata_l072/host/host_cfg.c:39` | live | Declared antenna gain feeding the ERP clamp. |
| CFG_KEY_HW_CEILING_DBM (0x16) | `17U` | no wire clamp — full u8 | `firmware/murata_l072/host/host_cfg.c:40` | live | Firmware-known PA ceiling fed to the power clamp. |
| CFG_KEY_FHSS_FARM_ID (0x17) | `0` (u64 LE, len 8) | full u64 — any value is a legal seed | `firmware/murata_l072/host/host_cfg.c` | live | **NEW 2026-07-29.** Farm/fleet-scope half of the FHSS hop seed. Consumed only at profile activation — write it BEFORE `CFG_KEY_REG_PROFILE`. |
| CFG_KEY_FHSS_NODE_ID (0x18) | `0` (u64 LE, len 8) | full u64 | `firmware/murata_l072/host/host_cfg.c` | live | **NEW 2026-07-29.** **LINK-scoped despite the name** — both ends of one link must carry the same value or their permutations diverge and the follower never holds lock. Host env var is deliberately named `LIFETRAC_FHSS_LINK_ID` to prevent per-board provisioning. Consumed only at activation. |
| CFG_KEY_PROTOCOL_VERSION (0x80) | `1` | read-only; set returns READ_ONLY | `firmware/murata_l072/host/host_cfg.c:136` | live | Must equal `frame->ver` or the dispatcher answers ERR_PROTO BAD_VERSION. |
| CFG_KEY_WIRE_SCHEMA_VERSION (0x81) | `2` | read-only | `firmware/murata_l072/host/host_cfg.c:138` | live | Payload-schema version. **`tools/mh_wire_constants.py:54` still mirrors 1 — the two peers disagree.** |
| CFG_KEY_CFG_DIRTY (0x82) | `0`; cfg_get special-cases it to the live flag (`host_cfg.c:425`) | read-only | `firmware/murata_l072/host/host_cfg.c:140` | diagnostic | Latches 1 the first time any key is set to a non-default value; cleared only by `cfg_init()`. |
| cfg profile initial active state | profile 0, mask 0xFF, bw 125000, gain 2, ceiling 17 | n/a | `firmware/murata_l072/host/host_cfg.c:349` | live | Seed for `host_cfg_profile_reset()` at cfg_init. **Never run through the validator.** |
| cfg_status_t | 0 OK … 14 PROFILE_REJECT_NULL_ARG | 0..14 | `firmware/murata_l072/include/host_cfg.h:7` | live | Status byte in CFG_OK_URC byte 1. `host_types.h:201` only defines HOST_CFG_STATUS_* for 0..6; 7..14 exist solely here. |
| CFG_STATUS_PROFILE_* wire codes | UNROUTED 7 … NULL_ARG 14 | 0..14 | `firmware/murata_l072/include/host_cfg.h:46` | live | Additive extension returned by `cfg_set(0x14)`. |
| HOST_CFG_OK_PAYLOAD_LEN | `4U` | fixed wire contract | `firmware/murata_l072/include/host_cfg_wire.h:9` | live | CFG_OK_URC payload length. |
| HOST_CFG_DATA_HEADER_LEN | `2U` | fixed wire contract | `firmware/murata_l072/include/host_cfg_wire.h:10` | live | CFG_DATA_URC header preceding value bytes. |
| HOST_CFG_DATA_PAYLOAD_MAX_LEN | `10U` | must be >= header + CFG_KEY_MAX_VALUE_LEN | `firmware/murata_l072/include/host_cfg_wire.h:11` | live | Max CFG_DATA_URC payload (2 B header + up to 8 B value). |
| HOST_ALLOW_REG_WRITE_DIAG allowlist | `{0x01, 0x06, 0x07, 0x08, 0x09, 0x1D, 0x1E, 0x26, 0x31, 0x33, 0x37, 0x3B, 0x40}` | 13 addresses | `firmware/murata_l072/host/host_cmd.c:519` | live | The complete set of registers a host may write; everything else answers ERR_PROTO FORBIDDEN. |

### 4.2 Opcodes and URCs

| Setting | Default | Range | Where | Status | Purpose |
| --- | --- | --- | --- | --- | --- |
| HOST_PROTOCOL_VER | `1U` | fixed wire contract | `firmware/murata_l072/include/host_types.h:6` | live | Stamped into every outbound frame; enforced inbound at `host_cmd.c:771`. |
| HOST_WIRE_SCHEMA_VER | `2U` | fixed wire contract | `firmware/murata_l072/include/host_types.h:7` | live | Payload-schema version reported in BOOT/VER/READY URCs and CFG key 0x81. |
| HOST_CAPABILITY_BITMAP | `0x7F` (PING, VERSION, UID, REG_IO, STATS, RESET, TX_FRAME) | bits 0..6 | `firmware/murata_l072/include/host_types.h:17` | live | Advertised in VER/BOOT/READY. Static — does **not** track HOST_ALLOW_REG_WRITE_DIAG, so REG_IO is advertised even when writes are refused. |
| HOST_TYPE_PING_REQ | `0x00` | n/a | `firmware/murata_l072/include/host_types.h:26` | live | Echoes payload/seq/flags back under the same type code. |
| HOST_TYPE_VER_REQ | `0x01` | n/a | `firmware/murata_l072/include/host_types.h:27` | live | Triggers VER_URC 0x81. |
| HOST_TYPE_UID_REQ | `0x02` | n/a | `firmware/murata_l072/include/host_types.h:28` | live | Returns the 96-bit STM32 UID as UID_URC 0x82. |
| HOST_TYPE_RESET_REQ | `0x03` | n/a | `firmware/murata_l072/include/host_types.h:29` | live | Immediate `platform_system_reset()` with no ack; resets all CFG state via `cfg_init()`. Both daemons send it at link open unless LIFETRAC_SKIP_RESET_REQ is set. |
| HOST_TYPE_TX_FRAME_REQ | `0x10` | payload_len must equal 2 + length | `firmware/murata_l072/include/host_types.h:31` | live | {tx_id, length, body}; parks into the depth-4 ring when busy or the hop slot cannot fit, ERR_PROTO QUEUE_FULL when full. |
| HOST_TYPE_CFG_SET_REQ | `0x20` | payload_len must equal 2 + len | `firmware/murata_l072/include/host_types.h:33` | live | The only opcode that changes configuration; answered by CFG_OK_URC 0xA0. |
| HOST_TYPE_CFG_GET_REQ | `0x21` | payload_len must be 1 | `firmware/murata_l072/include/host_types.h:34` | live | Answers CFG_DATA_URC 0xA1 on success, CFG_OK_URC carrying the error otherwise. |
| HOST_TYPE_STATS_RESET_REQ | `0x40` | n/a | `firmware/murata_l072/include/host_types.h:39` | live | Zeroes every radio and UART counter; sends no reply. |
| HOST_TYPE_STATS_DUMP_REQ | `0x41` | n/a | `firmware/murata_l072/include/host_types.h:40` | live | Emits STATS_URC 0xC1 with the 132-byte counter block. |
| HOST_TYPE_VER_URC | `0x81` | name truncated at 40 B, git at 16 B | `firmware/murata_l072/include/host_types.h:43` | live | {proto, schema, maj, min, patch, name_len, git_len, 0, cap_u32le, name, git}. |
| HOST_TYPE_UID_URC | `0x82` | n/a | `firmware/murata_l072/include/host_types.h:44` | live | 12-byte STM32 unique id, three u32 LE. |
| HOST_TYPE_TX_DONE_URC | `0x90` | status 0 OK, 1 TIMEOUT, 2 LBT_ABORT, 3 BUSY | `firmware/murata_l072/include/host_types.h:46` | live | 7-byte {tx_id, status, time_on_air_us, tx_power_dbm}; seq always 0. |
| HOST_TYPE_RX_FRAME_URC | `0x91` | 8-byte header | `firmware/murata_l072/include/host_types.h:49` | live | {len, snr_db, rssi_dbm, timestamp_us, payload}. |
| HOST_TYPE_CFG_OK_URC | `0xA0` | status byte 0..14 | `firmware/murata_l072/include/host_types.h:51` | live | Fixed 4-byte {key, status, actual_len, 0}. **A rejection arrives on this same type — the host must inspect byte 1, not trust the type match.** |
| HOST_TYPE_CFG_DATA_URC | `0xA1` | value_len <= 8 | `firmware/murata_l072/include/host_types.h:52` | live | {key, value_len, value}. |
| HOST_TYPE_RADIO_IRQ_URC | `0xC0` | n/a | `firmware/murata_l072/include/host_types.h:57` | diagnostic | 4-byte radio-event bitmask; compile-gated behind HOST_EMIT_RADIO_IRQ_DEBUG_URC (0), so production never emits it. |
| HOST_TYPE_STATS_URC | `0xC1` | HOST_STATS_PAYLOAD_LEN = 132 | `firmware/murata_l072/include/host_types.h:58` | live | 33 u32 LE fields, additive-only. |
| HOST_TYPE_BOOT_URC | `0xF0` | n/a | `firmware/murata_l072/include/host_types.h:157` | live | 6-byte {reset_cause, radio_ok, radio_version, proto_ver, schema_ver, clock_source_id}, sent first in `host_cmd_init`. |
| HOST_TYPE_READY_URC | `0xF2` | n/a | `firmware/murata_l072/include/host_types.h:160` | live | 8-byte final boot handshake {0x01, proto, schema, at_shell_enabled, capability_bitmap}. |
| HOST_TYPE_ERR_PROTO_URC | `0xFE` | err 1 BAD_VERSION … 8 FORBIDDEN | `firmware/murata_l072/include/host_types.h:161` | live | 5-byte {offending_type, offending_ver, err_code, detail}. |
| HOST_STATS_PAYLOAD_LEN | `132U` | `_Static_assert` pins `sizeof(host_stats_wire_t)` | `firmware/murata_l072/include/host_types.h:155` | live | Serialized STATS_URC payload size. |
| HOST_FAULT_URC_PAYLOAD_LEN | `24U` | fixed wire contract | `firmware/murata_l072/include/host_types.h:163` | live | FAULT_URC payload size. |
| HOST_AIRTIME_REJECT_PAYLOAD_LEN | `15U` | fixed wire contract | `firmware/murata_l072/include/host_types.h:77` | live | AIRTIME_INVARIANT_REJECT_URC payload size. |
| AT shell command set | `AT, ATI, AT+VER?, AT+RADIO?, AT+STAT?, AT+HELP, AT+HELP?, AT+BIN` | exact-match strings | `firmware/murata_l072/host/host_cmd.c:839` | diagnostic | Case-insensitive read-only shell; none of these change configuration. Unrecognised lines answer `ERROR\r\n`. |
| STM32_UID_BASE | `0x1FF80050UL` | fixed silicon address | `firmware/murata_l072/host/host_cmd.c:18` | live | Base of the 96-bit unique device ID read for UID_URC. |
| TOPIC_* telemetry ids | `TOPIC_GPS 0x01` … `TOPIC_SEMANTIC_MAP 0x2A` | uint8; 0x2A reserved for v26 | `firmware/common/lora_proto/lora_proto.h:57` | live | Full over-air telemetry topic space. 0x2A must not be emitted in v25. |
| SRC_* / FT_* | SRC_HANDHELD 0x01, BASE 0x02, TRACTOR 0x03, AUTONOMY 0x04, NONE 0xFF; FT_CONTROL 0x10, TELEMETRY 0x20, COMMAND 0x30, HEARTBEAT 0x40 | uint8 | `firmware/common/lora_proto/lora_proto.h:30` | live | Source-arbitration identities and frame-type space underpinning the control-priority arbiter. |
| BTN_* / FLAG_* | BTN_BUCKET_CURL..BTN_TAKE_CONTROL bits 0-7; FLAG_TAKECTL_HELD / ESTOP_ARMED / CELLULAR_FALLBACK bits 0-2 | bit positions | `firmware/common/lora_proto/lora_proto.h:96` | live | ControlFrame button and flag wire layout. |
| LP_NONCE_LEN / LP_TAG_LEN | `12` / `16` | bytes | `firmware/common/lora_proto/lora_proto.h:242` | live | AES-GCM nonce and tag lengths; sets the per-frame overhead the airtime budget must absorb. |
| PROTO_VERSION (Python) | `0x01` | exact match | `base_station/lora_proto.py:18` | live | Byte 0 of every over-air frame packed by pack_control / pack_heartbeat / pack_command. |
| CMD_ESTOP / CLEAR_ESTOP / CAMERA_SELECT / CAMERA_QUALITY | `0x01` / `0x02` / `0x03` / `0x04` | args <= 8 bytes | `base_station/lora_proto.py:31` | live | FT_COMMAND opcodes on the AEAD bridge path; ESTOP is P0, the rest P1. |
| CMD_LINK_TUNE | `0x21` | u8 opcode | `base_station/lora_proto.py:37` | live | P0 opcode consumed at `base_station/lora_bridge.py:378`. |
| CMD_PLAN_COMMIT / CMD_LINK_HINT | `0x10` / `0x20` | u8 opcode | `base_station/lora_proto.py:35` | **dead** | Zero references outside their own assignment lines (and the equally unused C mirrors). |
| CMD_PERSON_APPEARED / ROI_HINT / REQ_KEYFRAME / ENCODE_MODE / LINK_PROFILE | `0x60`–`0x64` | u8 opcode | `base_station/lora_proto.py:38` | live | Back-channel FT_COMMAND opcodes; PERSON_APPEARED is P0, the rest P1. |
| COMMAND_FRAME_MAGIC | `0xFB` | exact match | `base_station/lora_proto.py:955` | live | Strict-path command frame {magic, opcode, args}, chosen below the 0xFC/0xFD/0xFE fragment magics. **These frames are plaintext**, unlike the AEAD `pack_command` path. |
| COMMAND_FRAME_MAX_ARGS | `200` | 0..200 | `base_station/lora_proto.py:989` | live | `pack_command_frame` raises above it. |
| CMD_OP_REQ_KEYFRAME | `0x60` | member of `_CMD_OPS`; unknown opcodes parse to None | `base_station/lora_proto.py:956` | live | Strict-path keyframe request, no args. |
| CRYPTO_GCM128_EXPLICIT | overhead 28 B (12 nonce + 16 tag), has_mac | n/a | `base_station/lora_proto.py:566` | live | The shipped crypto profile and CRYPTO_PROFILE_DEFAULT. |
| CRYPTO_GCM64_IMPLICIT | overhead 12 B (4 seq + 8 truncated tag) | SEQ 4, TAG 8, BOOT_CTR 4 | `base_station/lora_proto.py:575` | live | D13 proposal: implicit nonce src+boot_ctr+seq+pad. |
| TELEM_MAX_PAYLOAD / TELEM_HEADER_LEN / HEADER_LEN / CRC_LEN / CTRL_FRAME_LEN / HB_FRAME_LEN | 118 / 7 / 5 / 2 / 16 / 10 | module constants | `base_station/lora_proto.py:114` | live | Over-air frame geometry. TELEM_MAX_PAYLOAD was cut 120→118 (IP-306) because the C-side `payload[120]` stores the trailing CRC in the same buffer. |
| KISS_FEND / FESC / TFEND / TFESC | `0xC0` / `0xDB` / `0xDC` / `0xDD` | module constants | `base_station/lora_proto.py:109` | live | KISS framing for the over-air path. The L072 host link uses COBS+0x00 instead, not KISS. |
| crc16_ccitt (Python) | poly 0x1021, init 0xFFFF, no reflection, xorout 0 | 16-bit | `base_station/lora_proto.py:170` | live | Identical to the firmware's `host_uart.c:154` and `host_rfco_summary.c:46`; stored little-endian everywhere. |
| REPLAY_WINDOW_BITS | `64` | module constant | `base_station/lora_proto.py:743` | live | Python sliding-window width mirroring LpReplayWindow. **Note this is 64 while CFG_KEY_REPLAY_WINDOW defaults to 16.** |
| PRIO_P0..PRIO_P3 | 0 / 1 / 2 / 3 | 0..3 | `base_station/lora_proto.py:703` | live | Traffic classes for `enforce_class_tag_boundary` / `enforce_class_downgrade_only`; unknown FT_COMMAND opcodes default to P1, everything else P2. |
| HOST_BAUD_DEFAULT | `921600UL` | validator accepts only 9600, 115200, 921600 | `firmware/murata_l072/config.h:47` | live | HostLink baud passed to `host_uart_init()` at boot and seeded into CFG key 0x0B. |
| HOST_INNER_MAX_LEN | `320U` | unconstrained — no static assert against HOST_COBS_MAX_LEN | `firmware/murata_l072/config.h:48` | live | Max decoded inner frame; sizes the COBS decode buffer, TX assembly buffer and the RX length guard. |
| HOST_COBS_MAX_LEN | `325U` | unconstrained | `firmware/murata_l072/config.h:49` | live | Worst-case COBS-encoded frame length; sole consumer is HOST_COBS_ENCODED_MAX. |
| HOST_COBS_ENCODED_MAX | `HOST_COBS_MAX_LEN - 2` = 323 | derived | `firmware/murata_l072/host/host_uart.c:13` | live | Sizes the RX accumulation buffer, overflow trip and TX encode scratch. |
| HOST_HEADER_LEN | `7U` | unconstrained | `firmware/murata_l072/include/host_uart.h:10` | live | Inner-frame header (ver, type, flags, seq[2], payload_len[2]). |
| HOST_CRC_LEN | `2U` | unconstrained | `firmware/murata_l072/include/host_uart.h:11` | live | Trailing CRC-16/CCITT-FALSE (init 0xFFFF, poly 0x1021, both inline literals at `host_uart.c:154`), little-endian. |
| HOST_PAYLOAD_MAX_LEN | `320 - 7 - 2` = 311 | `_Static_assert` requires the RFCO summary to fit | `firmware/murata_l072/include/host_uart.h:12` | live | Max HostLink payload; bounds both RX and URC TX. |
| HOST_DMA_RX_CH | `6U` | unconstrained | `firmware/murata_l072/host/host_uart.c:10` | live | DMA1 channel index read for the DMA RX write index. |
| HOST_DMA_RX_BUF_SIZE | `512U` | must be even (`_Static_assert` for HT/TC servicing) | `firmware/murata_l072/host/host_uart.c:11` | live | Circular DMA RX buffer size. |
| HOST_RX_RING_LEN | `512U` | must be a power of two (`_Static_assert`) | `firmware/murata_l072/host/host_uart.c:80` | live | Software RX byte ring between the drain-only IRQ producers and the foreground parser. |
| HOST_FRAME_QUEUE_DEPTH | `4U` | unconstrained | `firmware/murata_l072/host/host_uart.c:12` | live | Parsed inbound frame queue; overflow increments `s_stats_queue_full`. |
| HOST_RX_SEEN_FLAG_LPUART / _USART1 | `0x01` / `0x02` | bitmask bits 0 and 1 | `firmware/murata_l072/host/host_uart.c:103` | live | First-byte-seen flags surfaced via the HOST_RX_SEEN fault URC. |
| USART_CR1_OVER8 | `1UL << 15` | register bit | `firmware/murata_l072/host/host_uart.c:15` | live | Oversampling-by-8, set on USART1 only for tighter baud error on the mirror lane. |
| LPUART1_CLOCK_HZ | `16000000UL` | unconstrained | `firmware/murata_l072/host/host_uart.c:14` | **dead** | Stale constant referenced by zero lines — both BRR calculators call `platform_core_hz()` at runtime. It also disagrees with the live 32 MHz HSE case. |
| HOST_UART_TX_MIRROR_USART1 | `0` | 0/1 | `firmware/murata_l072/config.h:74` | diagnostic | Mirrors all host TX onto USART1; doubles blocking busy-wait per URC. |
| HOST_UART_RX_ECHO_DIAG | `0U` | 0/1 | `firmware/murata_l072/host/host_uart.c:19` | diagnostic | Non-blockingly echoes every raw RX byte on both lanes. |
| HOST_EMIT_RADIO_IRQ_DEBUG_URC | `0` | 0/1 | `firmware/murata_l072/config.h:66` | diagnostic | 1 emits a ~16-byte RADIO_IRQ_URC per nonzero event batch through the blocking TX path. Turned off in RS-3.7 for throughput. |
| HOST_AT_SHELL_ENABLE | `1` | 0/1 | `firmware/murata_l072/config.h:78` | live | Compiles in the ASCII AT shell alongside the binary COBS protocol. |
| HOST_AT_LINE_MAX_LEN | `96U` | `_Static_assert >= 8U` | `firmware/murata_l072/config.h:81` | live | AT-shell line buffer; overlong lines are discarded. |
| HOST_TXQ_DEPTH | `4U` | overflow answers ERR_PROTO QUEUE_FULL | `firmware/murata_l072/config.h:57` | live | Firmware TX ring; frames failing the slot-fit advisory park here rather than burning a hop slot. |
| HOST_TXQ_P0_RESERVED | `1U` | n/a | `firmware/murata_l072/config.h:58` | **dead** | No reference outside config.h — the ring is pure FIFO and P0 priority is entirely host-side policy. |
| MH_STREAM_IO_CHUNK_MAX / RX_ENCODED_CAP / TX_ENCODED_CAP / WRITE_BUDGET_MS | 256 / 512 / 512 / 5 ms | unconstrained | `firmware/tractor_h7/murata_host/mh_stream.h:15` | live | H7-side HostLink stream buffers and per-call write time budget. |
| HOST_HEADER_LEN / CRC_LEN / MAX_PAYLOAD_LEN (H7 mirror) | 7 / 2 / 320 | must match the L072 side | `firmware/tractor_h7/murata_host/mh_wire.h:66` | live | Duplicate wire constants kept in sync by `tools/check_mh_wire_sync.py` (enforced in CI). A drift-prone second source of truth. |
| HOST_STATS_LEGACY_OFFSET_RADIO_STATE / _PAYLOAD_LEN | `60` / `64` | n/a | `firmware/tractor_h7/murata_host/mh_wire.h:63` | deprecated | Back-compat offsets for the pre-expansion 64-byte STATS payload. |
| HOST_FAULT_CODE_CLOCK_HSE_FAILED | `0x08` | uint8 code space | `firmware/tractor_h7/murata_host/mh_wire.h:41` | live | Fault code emitted when HSE startup fails. |
| HostLink (class) | n/a | n/a | `firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py:178` | live | The L072 host-link transport lives here, **not** in `lora_proto.py`; re-exported by `method_h_stage2_tx_probe_v2.py` and imported from there by both daemons. |
| HostLink framing | `\x00 + cobs(header+payload+crc16le) + \x00`; header `<BBBHH` | inner frame must be >= 9 bytes | `firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py:149` | live | Python mirror of `host_uart.c:595` and `:843`, byte-for-byte. |
| BAUD_DEFAULT (HostLink) | `"921600"` | any stty-accepted baud string | `firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py:30` | live | `configure_uart()` runs stty `cs8 -parenb -cstopb raw -echo -ixon -ixoff -ixany -crtscts`. |
| DEV_DEFAULT (HostLink) | `"/dev/ttymxc3"` | any tty path | `firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py:29` | live | Default L072 host UART node on the Portenta X8. |
| HOST_PROTOCOL_VER (Python) | `1` | exact match | `firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py:32` | live | Must equal the firmware's or the L072 answers ERR_PROTO BAD_VERSION. |
| HostLink.seq | starts at 1, wraps mod 0x10000, skips 0 | 1..0xFFFF | `firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py:183` | live | Request/response correlation; VER_URC is matched on type alone. |
| cfg_set_checked | n/a | raises RuntimeError on status != 0 | `firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py:370` | live | The only Python helper that inspects the CFG_OK status byte. `CFG_STATUS_NAMES` covers 0..13 but **is missing 14** (PROFILE_REJECT_NULL_ARG), which prints as `?`. |
| LIFETRAC_L072_UART / _BAUD (base) | `/dev/ttymxc3` / `"921600"` | free-form strings passed to stty | `base_station/image_rx_daemon.py:1515` | live | Host-link device and baud; `--uart` / `--baud` override. Baud is never int-parsed. |
| LIFETRAC_L072_UART / _BAUD (tractor) | `/dev/ttymxc3` / `"921600"` | unconstrained strings | `firmware/tractor_x8/image_tx_daemon.py:1329` | live | Same knobs on the tractor side. |
| LIFETRAC_SKIP_RESET_REQ (tractor) | `"0"` — reset IS sent | truthy unless in ("0", "", "false", "False") — **"FALSE"/"no"/"off" all read as TRUE** | `firmware/tractor_x8/image_tx_daemon.py:441` | live | Skips the UART RESET_REQ when the caller already pulsed gpio163 NRST, avoiding the boot race that breaks VER warm-up. |
| LIFETRAC_SKIP_PHY_CONTRACT (tractor) | `"0"` | compared `!= "1"` — only the exact string "1" disables | `firmware/tractor_x8/image_tx_daemon.py:488` | live | Skips `verify_modem_matches_profile` at open and on every profile switch. |
| LIFETRAC_SKIP_RESET_REQ (bench script) | hardcoded `1` on both containers | never varied across all 70 runs | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:219` | live | The script performs the L072 resets itself (gpio163 NRST on RX, OpenOCD SWD on TX). |
| VER warm-up retry policy (tractor) | 5 attempts, 2.0 s each; stray-frame drain 0.3 s | n/a | `firmware/tractor_x8/image_tx_daemon.py:463` | live | A single 1.0 s attempt produced spurious "cannot open HostLink" fatals on the 2026-07-24 saturation bench. |
| VER warm-up retry budget (base) | 5 attempts, 2.0 s each | `range(1, 6)` | `base_station/image_rx_daemon.py:397` | live | Same fix on the base side; VER_REQ is idempotent. |
| drain_boot settle_s (tractor) | 1.5 s after a UART RESET_REQ; 0.25 s on the SKIP path | n/a | `firmware/tractor_x8/image_tx_daemon.py:446` | live | How long to absorb L072 boot chatter before the VER handshake. |
| drain_boot settle_s (base, reset path) | 1.5 s | unconstrained | `base_station/image_rx_daemon.py:381` | live | Boot-chatter settle after the daemon's own RESET_REQ. |
| drain_boot settle_s (base, external NRST) | 0.25 s | unconstrained | `base_station/image_rx_daemon.py:389` | live | Shorter post-NRST drain when LIFETRAC_SKIP_RESET_REQ is set. |
| drain_pending (tractor) | quiet_s 0.25, max_s 1.0 at open; 0.2 / 1.0 before a profile reconfigure | n/a | `firmware/tractor_x8/image_tx_daemon.py:483` | live | Drains URCs to a quiet line so CFG replies are not confused with stale traffic. |
| drain_pending (base, link open) | quiet_s 0.25, max_s 1.0 | unconstrained | `base_station/image_rx_daemon.py:409` | live | Quiesce the HostLink after VER warm-up. |
| CFG_KEY_LBT_ENABLE = 0 (tractor) | `{0x03, 0x01, 0x00}`, timeout 1.0 s | n/a | `firmware/tractor_x8/image_tx_daemon.py:493` | live | Unconditionally disables LBT at link open (W1-10b rationale); failure is logged and ignored. |
| CFG_KEY_LBT_ENABLE = 0 (base) | forced `0x00`, timeout 1.0 s | hard-coded 0x00 | `base_station/image_rx_daemon.py:420` | live | run-32 root cause: firmware boots with LBT on, and at ~58 % tractor duty every command TX died ABORT_LBT/FORBIDDEN. |
| RX_POLL_TIMEOUT_S | `0.25` | unconstrained | `base_station/image_rx_daemon.py:199` | live | `read_frames()` poll timeout; doubles as the idle-drain / pending-command retry cadence. |
| DEFAULT_UART / DEFAULT_BAUD (base) | `/dev/ttymxc3` / `"921600"` | unconstrained | `base_station/image_rx_daemon.py:195` | live | Fallbacks for the L072 HostLink on the base X8. |
| wait_for_tx_done timeout (base) | 2.0 s (frame type 0xFD) | unconstrained | `base_station/image_rx_daemon.py:650` | live | A logged OK means the command frame really went on air. |
| X8 back-channel opcodes + KISS bytes | X8_CMD_TOPIC 0xC0; REQ_KEYFRAME 0x62, CAMERA_SELECT 0x03, CAMERA_QUALITY 0x04, ENCODE_MODE 0x63, ROI_HINT 0x61, LINK_PROFILE 0x64 | frames not starting with 0xC0 are dropped | `firmware/tractor_x8/camera_service.py:1170` | **dead** | The reader thread is not started under LIFETRAC_USE_LORA_BRIDGE=1, so on the strict path this whole opcode set — including CMD_LINK_PROFILE and CMD_ROI_HINT — is unreachable. |
| back-channel reader chunk size | `rfh.read(64)`, unbuffered | n/a | `firmware/tractor_x8/camera_service.py:1393` | **dead** | Read granularity of the KISS state machine; unreachable in bridge mode. |
| ipc_to_h747 IpcWriter | device `/dev/ttymxc1`; FRAME_MARKER 0xA5; FLAG_KEYFRAME 0x01, MOTION 0x02, WIREFRAME 0x04 | n/a | `firmware/tractor_x8/image_pipeline/ipc_to_h747.py:33` | live | Length-framed X8→H747 image transport with CRC8-SMBUS; skipped entirely in LoRa-bridge mode. |
| GPS/IMU wire packet formats | GPS v0x01 `<BB iii hH BBB`; IMU v0x01 `<BB hhhh hhh H` (18 B); TOPIC_GPS 0x01, TOPIC_IMU 0x07 | n/a | `firmware/tractor_x8/gps_service.py:94` | live | KISS-framed sensor telemetry to the H747; same KISS bytes as the camera back-channel, different topic bytes. |
| BLINKA_MCP2221 | `setdefault("1")` — set only if absent | n/a | `firmware/tractor_x8/gps_service.py:62` | live | Forces the Blinka backend to the MCP2221 I2C bridge before the sensor libraries import. The only non-`LIFETRAC_` env var in the tractor tree. |
| ComPort / DurationSec / Baud / Parity / DataBits / StopBits | COM8 / 18 s / 19200 / Even / 8 / One | standard serial enums, unvalidated | `firmware/x8_lora_bootloader_helper/sniff_jlink_cdc.ps1:2` | diagnostic | J-Link virtual COM sniffer; 19200/Even is the STM32 AN3155 ROM-bootloader line setting. |
| ComBaud / ComProbeTimeoutMs | 115200 / 1500 ms | unconstrained | `firmware/x8_lora_bootloader_helper/diagnose_x8_recovery.ps1:17` | diagnostic | X8 recovery diagnostic probe timings. AdbSerial is mandatory here. |
| Target / ManifestPath / ProbeDevice / Baud / BootTimeout / SkipDiagProbes | Target required; others `""` / 0 / 0 / off | Baud 0 and BootTimeout 0 are "inherit from manifest" sentinels | `firmware/x8_lora_bootloader_helper/run_owner_net_profile.ps1:2` | live | Applies a named net-owner profile from `owner_net_profiles.json`. |
| owner_net_profiles.json | probeDevice `/dev/ttymxc3`, baud 921600, bootTimeout 2.0 per profile | per-profile | `firmware/x8_lora_bootloader_helper/owner_net_profiles.json` | diagnostic | The actual source of the replay-profile values the PowerShell parameters merely override. |

---

## 5. Tractor daemon environment

`image_tx_daemon.py`, `logger_service.py`, `params_service.py`, and the tractor systemd units.
Every `_env_int` / `_env_float` read happens **once at import** unless noted.

| Setting | Default | Range | Where | Status | Purpose |
| --- | --- | --- | --- | --- | --- |
| LIFETRAC_TX_PIPELINE | `"v2"` | "v2" or "v3"; any other value behaves as v2 | `firmware/tractor_x8/image_tx_daemon.py:201` | live | v2 = serial send → TX_DONE → send. v3 = keep PIPELINE_DEPTH TX_FRAME_REQs in flight against the firmware ring. |
| LIFETRAC_TX_PIPELINE_DEPTH | `2` | `_env_int lo=1`, **no upper bound**; comment recommends <=3 at FHSS, 4 at DTS — not enforced | `firmware/tractor_x8/image_tx_daemon.py:207` | live | Outstanding TX_FRAME_REQs in v3 so UART turnaround hides inside the previous fragment's airtime. Firmware ring holds 1 transmitting + 4 parked. |
| LIFETRAC_TX_BATCH | `0` (off) | `_env_int` with no lo/hi; used only as `== 0` gate at `:580`, so any non-zero enables | `firmware/tractor_x8/image_tx_daemon.py:212` | live | Greedy batching of non-key frames into one FRAME_BATCH train to amortize the measured ~44 ms per-handoff overhead. |
| LIFETRAC_TX_PREPARE_AHEAD | `0` (off) | no lo/hi; `== 0` gate at `:563` | `firmware/tractor_x8/image_tx_daemon.py:217` | live | Builds train N+1 during train N's airtime so the inter-train gap becomes designed listen time. |
| LIFETRAC_BATCH_BUDGET_B | `760` | `_env_int lo=1`, no hi | `firmware/tractor_x8/image_tx_daemon.py:226` | live | Byte ceiling on a batched train; admits triples of ~246 B frames so the structural runt fragment amortizes. |
| LIFETRAC_BATCH_MAX_FRAMES | `4` | `_env_int lo=1`, no hi | `firmware/tractor_x8/image_tx_daemon.py:239` | live | Hard cap on frames merged into one batch regardless of remaining byte budget. |
| LIFETRAC_ACK_COPIES | `2` | `_env_int lo=1 hi=4` | `firmware/tractor_x8/image_tx_daemon.py:764` | live | On-air copies for queued replies (probe echo, non-profile acks). Profile ACKs bypass this and hardcode 2 at `:790`. Evaluated once at class-body import. |
| LIFETRAC_PROBE_ECHO | `1` (echo enabled) | `_env_int lo=0 hi=1`, compared `== 1` | `firmware/tractor_x8/image_tx_daemon.py:243` | live | 0 suppresses CMD_OP_PROBE_ECHO so a run measures the image-throughput cost of carrying acks; the tractor-side PROBE log stays the authoritative delivery counter. |
| LIFETRAC_IMAGE_TX_QUEUE_DEPTH | `4` | `_env_int lo=1` | `firmware/tractor_x8/image_tx_daemon.py:1339` | live | Host-side pending-frame queue; overflow increments `frames_dropped_queue_full`. Also `--max-queue-depth`. |
| LIFETRAC_MQTT_HOST (tractor) | `"127.0.0.1"` | unconstrained string | `firmware/tractor_x8/image_tx_daemon.py:1334` | live | Local broker for image frames. **camera_service reads the same var but defaults to `"localhost"`** (`camera_service.py:113`). |
| LIFETRAC_MQTT_PORT | `1883` | `_env_int lo=1 hi=65535` | `firmware/tractor_x8/image_tx_daemon.py:1335` | live | Broker port. camera_service has no equivalent knob — its port is hardcoded. |
| LIFETRAC_IMAGE_TX_LOG_LEVEL | `"INFO"` | unknown names silently degrade to INFO | `firmware/tractor_x8/image_tx_daemon.py:1342` | live | Root log level; also `--log-level`. |
| LIFETRAC_LORA_INTER_CYCLE_S | `0.05` (MIN_LORA_HOST_INTER_CYCLE_S) | `_env_float lo=0.0`, then floored at 0.05 by a bare `max()` at `:351` | `firmware/tractor_x8/image_tx_daemon.py:1337` | **dead** | Intended min spacing between TX_FRAME_REQs. `self.inter_cycle_s` is stored and printed in one LOG.info (`:521`) and nothing ever sleeps on it. `clamp_inter_cycle_s` is **not** called here, so no P3-CLAMP warning is ever printed. |
| MIN_LORA_HOST_INTER_CYCLE_S | `0.05` | module constant | `firmware/tractor_x8/image_tx_daemon.py:175` | **dead** | Documented 50 ms floor; only used as the default and floor for the dead value above. |
| --stats-interval-s (tractor) | `10.0` | unconstrained float | `firmware/tractor_x8/image_tx_daemon.py:1341` | live | Goodput/util/queue-depth stats line period. CLI-only — no env var. |
| max_qos_retries | `4` | n/a | `firmware/tractor_x8/image_tx_daemon.py:969` | live | Retries per fragment on ERR_PROTO FORBIDDEN / ABORT_QOS (zero RF spent). Backoff is `est_us/2e6` s. |
| max_rf_retries | `1` | n/a | `firmware/tractor_x8/image_tx_daemon.py:970` | live | Retries on TX_DONE non-OK or timeout (airtime **was** spent). Exhausting it aborts the whole frame — a frame missing any fragment can never reassemble. |
| _send_command_frame copies / timeout | copies 2 (signature default), TX_DONE timeout 2.0 s, tx_id byte 0xFE | copies coerced with `max(1, ...)` | `firmware/tractor_x8/image_tx_daemon.py:741` | live | Fire-and-forget command TX. Profile ACKs pass 2, other replies pass ACK_COPIES, the encode ack passes 1. |
| encode-ack payload truncation | `ack[:200]`, copies 1 | n/a | `firmware/tractor_x8/image_tx_daemon.py:776` | live | Caps the camera_service encode-mode ack JSON forwarded as CMD_OP_ENCODE_MODE_ACK. |
| _cmd_out queue maxsize | `8` | n/a | `firmware/tractor_x8/image_tx_daemon.py:384` | live | Outbound ack/echo queue; `queue.Full` is silently swallowed at both put sites (`:709`, `:738`). |
| TX worker idle poll timings | frame-queue get 0.25 s; idle RX drain 0.05 s; pipelined read poll 0.05 s | n/a | `firmware/tractor_x8/image_tx_daemon.py:545` | live | How often the TX worker services the control plane and how much listen time the command downlink gets. |
| MQTT keepalive (tractor) | `30` s | n/a | `firmware/tractor_x8/image_tx_daemon.py:1290` | live | paho keepalive for the local broker connection. |
| shutdown timings (tractor) | main loop sleep 0.5 s; `tx_thread.join(timeout=5.0)` | n/a | `firmware/tractor_x8/image_tx_daemon.py:1316` | live | SIGINT/SIGTERM graceful-shutdown latency and worker join deadline. |
| _stats_worker tick | `time.sleep(1.0)` inner tick, gated by stats_interval_s | n/a | `firmware/tractor_x8/image_tx_daemon.py:1228` | diagnostic | Stats thread wakes every second, emits only once the interval elapses. |
| batching log cadence | every 25th batched train (`% 25 == 1`) | n/a | `firmware/tractor_x8/image_tx_daemon.py:614` | diagnostic | Rate-limits the "batching: N frames in M trains" line. |
| MQTT_TOPIC_IN | `lifetrac/v25/cmd/image_frame` | n/a | `firmware/tractor_x8/image_tx_daemon.py:281` | live | Topic the TX daemon subscribes to for complete TileDeltaFrame payloads. |
| RADIO_PROFILE_ACK_TOPIC (tractor) | `lifetrac/v25/status/radio_profile/tx` | n/a | `firmware/tractor_x8/image_tx_daemon.py:290` | diagnostic | Retained local-only log of profile-switch outcome; not radiated. |
| TRACTOR_ENCODE_MODE_TOPIC | `lifetrac/v25/tractor/encode_mode_override` | n/a | `firmware/tractor_x8/image_tx_daemon.py:291` | live | Retained intra-board hop carrying {mode, quality} decoded from a 0xFB frame. Deliberately named apart from base cmd topics so a shared bench broker cannot bypass the radio. |
| TRACTOR_KEYFRAME_TOPIC | `lifetrac/v25/tractor/req_keyframe` | n/a | `firmware/tractor_x8/image_tx_daemon.py:292` | live | Intra-board hop for a LoRa REQ_KEYFRAME; payload `b"\x01"`, qos 0, receipt alone is the trigger. |
| TRACTOR_ENC_STATUS_TOPIC | `lifetrac/v25/status/encode_mode` | n/a | `firmware/tractor_x8/image_tx_daemon.py:293` | live | Retained ack published by camera_service; the TX daemon radiates it back as 0x68, suppressing the retained-replay echo. |
| LIFETRAC_TRACTOR_AUDIT_PATH | `/var/log/lifetrac/tractor_audit.jsonl` | unconstrained path | `firmware/tractor_x8/logger_service.py:47` | live | Rotating JSONL audit log (shared AuditLog imported from base_station). |
| LIFETRAC_TRACTOR_TELEM_DB | `/var/log/lifetrac/telemetry.sqlite` | parent dirs are created | `firmware/tractor_x8/logger_service.py:49` | live | SQLite sink for every message matching `lifetrac/v25/#`. |
| logger_service timings / broker | writer join 5 s; queue get 0.5 s; flush/idle 60.0 s; `connect("localhost", 1883)` hardcoded | n/a | `firmware/tractor_x8/logger_service.py:86` | live | Single-writer pacing. **The broker is not env-settable here — it ignores LIFETRAC_MQTT_HOST.** |
| LIFETRAC_PARAMS_PATH | `/var/lib/lifetrac/params.json` | unconstrained path | `firmware/tractor_x8/params_service.py:59` | live | Persisted overlay deep-merged over built-in defaults; changes announced on `lifetrac/v25/params/changed`. |
| params_service built-in defaults | joystick.deadband_pct 6, expo_pct 25; link.control_phy "SF7_BW250", link.image_phy "SF7_BW500"; key.fleet_key_id 0 | no validation on merge | `firmware/tractor_x8/params_service.py:46` | live | Production systemd unit; the retained snapshot is cached by the base console and served to the UI, and a SIL test pins `link.control_phy`. **`link.image_phy` is advisory metadata only** — the real image PHY comes from LIFETRAC_REG_PROFILE. |
| params_service broker / poll | `connect("localhost", 1883)`; main loop `sleep(3600)` | n/a | `firmware/tractor_x8/params_service.py:183` | live | Hardcoded broker (ignores LIFETRAC_MQTT_HOST) and idle re-check interval. |
| gps_service rates / serial timeouts | `--rate-hz 1.0`, `--idle-rate-hz 0.2`; serial timeout 0.1, write_timeout 0.1 | idle_rate <= 0 falls back to active period | `firmware/tractor_x8/gps_service.py:235` | live | Active vs idle GPS publish cadence. CLI-only, no env vars. |
| imu_service rate / serial timeouts | `--rate-hz 5.0`; serial timeout 0.1, write_timeout 0.1 | rate floored at 0.1 Hz | `firmware/tractor_x8/imu_service.py:214` | live | IMU publish cadence. CLI-only, no env vars. |
| PYTHONPATH (container + systemd) | `/app/base_station:/app`; `/opt/lifetrac/base_station` | path list | `Dockerfile:35` | live | Makes lora_proto / audit_log / settings_store importable without per-script sys.path hacks (IP-304). Systemd variant at `firmware/tractor_x8/systemd/lifetrac-camera.service:14`. |
| lifetrac-camera.service SupplementaryGroups | `video` | group names | `firmware/tractor_x8/systemd/lifetrac-camera.service:17` | live | Grants access to `/dev/video*` and `/dev/media*`. |
| systemd Restart / RestartSec | `on-failure` / `5` | systemd restart policies | `firmware/tractor_x8/systemd/lifetrac-camera.service:9` | live | Crash-restart policy for all four X8 Linux services (same in logger/params/time units). |
| lifetrac-time.service User | `root` | any user | `firmware/tractor_x8/systemd/lifetrac-time.service:16` | live | Needed for the PPS_FETCH ioctl, raw UART writes and future adjtimex slewing. |
| mosquitto.video-test.conf | `listener 1883 0.0.0.0`, `allow_anonymous true`, `persistence false` | n/a | `firmware/tractor_x8/mosquitto.video-test.conf:4` | live | Minimal anonymous broker seeded from the host; without it eclipse-mosquitto:2 rejects every connection. |
| tractor requirements.txt | adafruit-blinka>=8.40.0, hidapi>=0.14.0, bno08x>=1.2.5, gps>=3.10.0, **paho-mqtt>=1.6.1**, Pillow>=10.0.0; picamera2 commented out | n/a | `firmware/tractor_x8/requirements.txt:7` | live | Tractor dependency set. paho-mqtt >=1.6.1 here vs >=2.0 on the base is a cross-node callback-API split. |

---

## 6. Base daemon environment

`image_rx_daemon.py`, `web_ui.py`, `lora_bridge.py`, compose/systemd, and the browser-side client constants.

### 6.1 RX daemon and transport

| Setting | Default | Range | Where | Status | Purpose |
| --- | --- | --- | --- | --- | --- |
| _env_int() | helper | per-call lo/hi | `base_station/image_rx_daemon.py:114` | live | Defensive env-int parser: blank/garbage falls back to the caller default, logs, then clamps. |
| _env_float() | helper | per-call lo/hi | `base_station/image_rx_daemon.py:131` | **dead** | Float twin of `_env_int`; defined for the same contract but never called anywhere in the file. |
| LIFETRAC_MQTT_HOST / _PORT (base) | `127.0.0.1` / `1883` | port clamped 1..65535 | `base_station/image_rx_daemon.py:1519` | live | Primary broker for tile_delta/link_stats publishes and control subscriptions. Assumes `adb reverse tcp:1883` bridges the X8 to the Windows mosquitto. |
| LIFETRAC_CTRL_MQTT_HOST | unset (None) | hostname string; blank means None | `base_station/image_rx_daemon.py:361` | live | Optional second broker the RX daemon publishes control acks to, split from the data plane. |
| LIFETRAC_IMAGE_RX_LOG_LEVEL | `"INFO"` | unknown name silently → INFO | `base_station/image_rx_daemon.py:1526` | live | Root log level; also `--log-level`. |
| --stats-interval-s (base) | `10.0` | unconstrained | `base_station/image_rx_daemon.py:1525` | live | Stats line period (rx_frames, decode errors, parity recon, air_gap histogram, probe grid). CLI-only. |
| LIFETRAC_ALIGNED_PUMP | `"1"` (enabled) | "0" disables | `base_station/image_rx_daemon.py:204` | live | Completion-aligned command pump (RS-1.x); "0" falls back to idle-drain-only TX, the behaviour the 171/1 convergence soak measured. |
| LIFETRAC_KF_REQUEST_DISABLE | `"0"` | "1" disables requests | `base_station/image_rx_daemon.py:223` | diagnostic | RS-4.14: suppresses self-heal keyframe requests to test whether the requests feed the loss they react to. |
| MQTT_TOPIC_OUT | `lifetrac/v25/video/tile_delta` | unconstrained | `base_station/image_rx_daemon.py:151` | live | Reassembled TileDeltaFrame republish topic (mirrors bridge topic 0x25). |
| KEYFRAME_REQ_TOPIC | `lifetrac/v25/cmd/req_keyframe` | unconstrained | `base_station/image_rx_daemon.py:152` | live | Published by the self-heal poker, subscribed as the LoRa command trigger. |
| LINK_STATS_TOPIC | `lifetrac/v25/video/link_stats` | unconstrained | `base_station/image_rx_daemon.py:155` | live | Rolling RX speed/RSSI/SNR JSON consumed by the UI image panel and the auto-profile policy. |
| LINK_STATS_INTERVAL_S | `2.0` | unconstrained | `base_station/image_rx_daemon.py:156` | live | Sample period; also the window over which bps and frames_per_s are differenced. |
| KEYFRAME_CMD_MIN_GAP_S | `10.0` | unconstrained | `base_station/image_rx_daemon.py:182` | live | Min spacing between LoRa REQ_KEYFRAME commands taken off MQTT; keeps base TX duty tiny. |
| KeyframeRequester.min_interval_s | `5.0` | unconstrained | `base_station/image_rx_daemon.py:212` | live | Rate limit on self-heal request publishes after a reassembly error/timeout; constructed with the default at `:266`. |
| KeyframeRequester publish QoS | qos 0, payload `b"\x01"` | unconstrained | `base_station/image_rx_daemon.py:232` | live | Fire-and-forget; a lost request just means the reassembler heals on its own. |
| _ctrl_out queue maxsize | `16` | 16 entries | `base_station/image_rx_daemon.py:340` | live | Legacy one-shot outbound command queue; the ack-driven `_pending_cmds` dict is the primary path now. |
| _set_pending timeout_s | `10.0` | unconstrained | `base_station/image_rx_daemon.py:556` | live | Deadline for an ack-driven pending command; past it the entry is deleted and logged GAVE UP. All callers use the default. |
| _set_pending max_attempts | `20` | unconstrained | `base_station/image_rx_daemon.py:556` | live | Retry budget per pending command; reset when the same opcode arrives with a different body. |
| PENDING_RETRY_MIN_GAP_S | `0.4` | unconstrained | `base_station/image_rx_daemon.py:591` | live | Min spacing between retries of the same command so 20 attempts span the full 10 s instead of exhausting in ~5 s. |
| _send_command_frame copies (base) | `2` | `max(1, copies)` | `base_station/image_rx_daemon.py:634` | live | Default on-air copies. The aligned pump and idle drain pass 1; profile-switch and CONF take the default 2 — which is why raw PROBE_ECHO arrivals double-count without the pop-keying fix. |
| _probe_pending / _probe_bins pop-keying | first-arrival-only counting | n/a | `base_station/image_rx_daemon.py:728` | live | Echo counting pops the seq so duplicate echoes do not double-count — the fix for the observed 105.8 % delivery on run B2. |
| aligned-pump minimum spacing | `0.12` s | inline literal | `base_station/image_rx_daemon.py:1236` | live | Run-H forensics: without spacing, two enqueued copies went out ~37 ms apart into the same stale window and collapsed to one attempt. |
| MAX_RETAINED_CMD_AGE_S | `600.0` | unconstrained | `base_station/image_rx_daemon.py:906` | live | Retained radio_profile pins older than this are ignored on reconnect (run-V: a stale pin re-commanded the tractor mid-run). The **first** retained delivery after start is always honoured; a payload with no numeric `ts` is treated as live. |
| control-topic subscribe QoS | radio_profile 1, encode_mode_override 1, req_keyframe 0 | 0 or 1 | `base_station/image_rx_daemon.py:1441` | live | Retained control pins at QoS 1 so a restart re-applies the operator's choice; mirrored for the split-broker client at `:1467`. |
| MQTT keepalive (base) | `30` s | unconstrained | `base_station/image_rx_daemon.py:1450` | live | paho keepalive for the primary and split-broker clients. |
| LIFETRAC_LORA_DEVICE | `/dev/ttyACM0` (base compose); `/dev/ttymxc3` in the video-test compose | any host device path | `docker-compose.yml:41` | live | Host serial bind-mounted into the bridge/RX container and passed as the daemon's positional port arg. |
| LIFETRAC_FLEET_KEY_FILE | unset → falls back to the hex var | file path; contents 16 bytes / 32 hex | `docker-compose.yml:45` | live | Docker secret holding the AES-128 fleet key (read at `base_station/lora_bridge.py:104`). |
| LIFETRAC_FLEET_KEY_HEX | `""` | 32 hex chars | `base_station/lora_bridge.py:122` | live | Inline hex fleet key used when the file var is absent. |
| LIFETRAC_NONCE_STORE | `/var/lib/lifetrac/nonce_store.json` | file path | `base_station/lora_bridge.py:216` | live | Persisted SRC_BASE TX nonce-seq so a crash inside one wall-clock second cannot reuse a (key, nonce) pair (IP-101). |
| NonceStore DEFAULT_GAP / FLUSH_INTERVAL_S | `64` / `1.0` | unconstrained | `base_station/nonce_store.py:34` | live | How far ahead the nonce sequence is reserved and how often it is flushed. |
| LIFETRAC_BRIDGE_LOCKFILE | `/var/lib/lifetrac/lora_bridge.lock` | file path | `base_station/lora_bridge.py:766` | live | Advisory flock preventing two bridges from driving the radio serial port (IP-209). Also `--lockfile`. |
| LIFETRAC_SETTINGS_PATH | `/var/lib/lifetrac/settings.json` | file path | `base_station/settings_store.py:27` | live | Backing JSON for the persisted operator settings store. |
| LIFETRAC_BASE_SETTINGS | none | file path | `base_station/settings_store.py:28` | deprecated | Legacy alias for LIFETRAC_SETTINGS_PATH, still honoured as a fallback. |
| LIFETRAC_TRUSTED_PROXIES | `""` | comma-separated IPs/CIDRs | `docker-compose.yml:67` | unknown | Proxy allowlist for uvicorn `--proxy-headers` so audit-log client IPs are trustworthy. No consumer confirmed. |
| web_ui uvicorn bind + published port | `--host 0.0.0.0 --port 8080`; published `8080:8080` | any host:port | `docker-compose.yml:71` | live | Operator console listen address and LAN exposure. |
| mosquitto listener / anonymous / persistence | `listener 1883 0.0.0.0`, `allow_anonymous true`, `persistence true`, data in `/mosquitto/data/` | port 1..65535 | `base_station/mosquitto.conf:9` | live | Broker bind, auth policy and retained-message persistence. |
| mosquitto max_queued / max_inflight | `1000` / `50` | positive ints | `base_station/mosquitto.conf:26` | live | Backpressure limits sized for the ~80 msg/s tile-delta flood. |
| mosquitto host port publish | `127.0.0.1:1883:1883` (loopback only) | host:port mapping | `docker-compose.yml:29` | live | Whether the broker is reachable off-host; currently deliberately loopback-bound. |
| lifetrac-base.service | WorkingDirectory `/opt/lifetrac/DESIGN-CONTROLLER`; EnvironmentFile `.env`; TimeoutStartSec 300 | path / seconds | `base_station/systemd/lifetrac-base.service:10` | live | Where the base compose stack launches from and which `.env` supplies LIFETRAC_PIN / LIFETRAC_LORA_DEVICE. |
| .env.example | `LIFETRAC_PIN=1234`, `LIFETRAC_LORA_DEVICE=/dev/ttyACM0` | PIN digits; device path | `.env.example:2` | live | The only two documented `.env` knobs; the shipped example PIN is the weak default. |
| base requirements.txt | fastapi>=0.110, uvicorn[standard]>=0.27, pyserial>=3.5, **paho-mqtt>=2.0**, cryptography>=42, jinja2>=3.1 | unpinned (file says "DRAFT — not pinned") | `base_station/requirements.txt:2` | live | Dependency floors; the paho v2 API changed callback signatures relative to the tractor's >=1.6.1. |

### 6.2 Web UI — auth, sockets, limits

| Setting | Default | Range | Where | Status | Purpose |
| --- | --- | --- | --- | --- | --- |
| LIFETRAC_MQTT_HOST (web_ui) | `"localhost"` | unconstrained | `base_station/web_ui.py:569` | live | Broker host for the console's single paho client (client_id `web_ui` — a fixed id, so two instances evict each other). |
| web_ui MQTT port | `1883`, hard-coded | 1883 only | `base_station/web_ui.py:574` | live | **Asymmetry:** web_ui honours LIFETRAC_MQTT_HOST but has no port override, while image_rx_daemon honours LIFETRAC_MQTT_PORT and defaults its host to `127.0.0.1`. |
| MQTT connect retry budget | deadline 30.0 s; backoff 0.5 s doubling to a 5.0 s cap | backoff capped at 5.0 | `base_station/web_ui.py:575` | live | IP-201: bounded import-time retries so a transient DNS failure does not crash the console. |
| MQTT subscription set | telemetry/#, status/#, video/#, control/#, params/changed (+ cmd/image_frame when enabled) | n/a | `base_station/web_ui.py:1026` | live | Everything fanned out to /ws/telemetry plus the cached surfaces. |
| DEFAULT_OPERATOR_PIN | `"1234"` | unconstrained | `base_station/web_ui.py:161` | live | Last-resort PIN so first boot is never a lockout. Priority: PIN_STORE file > LIFETRAC_PIN_FILE > LIFETRAC_PIN > this. |
| LIFETRAC_PIN_STORE | `<module dir>/.operator_pin` | unconstrained | `base_station/web_ui.py:165` | live | Persisted UI-changed PIN (written 0600 via tmp + os.replace); highest priority source. |
| LIFETRAC_PIN_FILE | `""` | unconstrained | `base_station/web_ui.py:194` | live | Docker-secret mount path (IP-002/IP-008); second in priority. |
| LIFETRAC_PIN | `""` | unconstrained | `base_station/web_ui.py:199` | live | Dev/test PIN, third in priority. An empty resolved PIN makes /api/login return 503 rather than fail open. |
| SESSION_TTL_S | `1800` (30 min) | unconstrained | `base_station/web_ui.py:221` | live | Idle session timeout and the cookie max_age; every valid request refreshes last-used. |
| LOCKOUT_FAILS | `5` | unconstrained | `base_station/web_ui.py:222` | live | Wrong PINs from one IP before lockout; each failure and the lockout are audited. |
| LOCKOUT_S | `60` | unconstrained | `base_station/web_ui.py:223` | live | Cool-off after LOCKOUT_FAILS; /api/login returns 429 with the remaining time. |
| session cookie policy | httponly, samesite strict, path /, max_age SESSION_TTL_S; **Secure not set** | n/a | `base_station/web_ui.py:1989` | live | Per MASTER_PLAN §8.5; Secure is deliberately omitted because v25 is plain HTTP on LAN. Token is `secrets.token_urlsafe(32)`. A PIN change invalidates every session except the caller's. |
| LoginBody.pin length | required | 1-16 chars | `base_station/web_ui.py:77` | live | Submitted PIN bound on POST /api/login. |
| ChangePinBody.new_pin pattern | required | `^\d{4,6}$`; current_pin 1-16 chars | `base_station/web_ui.py:83` | live | New operator PIN must be 4-6 digits. |
| _ALLOWED_PARAM_KEYS | image.coral_enabled, image.encode_quality, image.tile_quality, ui.theme, ui.language | exactly those 5 | `base_station/web_ui.py:47` | live | Allow-list for POST /api/params; anything else is HTTP 400 so a compromised browser cannot poke internal params topics. |
| MAX_PARAM_VALUE_LEN | `256` | 256 bytes | `base_station/web_ui.py:55` | live | Longer string param values are rejected. |
| MAX_PARAMS_BODY_BYTES | `4096` | 4096 bytes | `base_station/web_ui.py:56` | live | Raw-body cap on POST /api/params; over it returns HTTP 413. |
| MAX_AUDIT_EVENT_BODY_BYTES | `2048` | 2048 bytes | `base_station/web_ui.py:1305` | live | Cap on the two audit-intent POSTs; oversized bodies become `{"truncated": true, "size": n}`. |
| MAX_TELEMETRY_SUBSCRIBERS | `8` | 8 | `base_station/web_ui.py:783` | live | Concurrent /ws/telemetry socket cap (IP-202/IP-208 OOM guard). |
| MAX_IMAGE_SUBSCRIBERS | `4` | 4 | `base_station/web_ui.py:784` | live | Concurrent /ws/image socket cap. |
| MAX_STATE_SUBSCRIBERS | `8` | 8 | `base_station/web_ui.py:785` | live | Concurrent /ws/state socket cap. |
| WS_OVER_CAPACITY_CODE | `4429` | 4429 | `base_station/web_ui.py:795` | live | Close code for an over-cap connection; also audited as `ws_over_cap`. |
| /ws/state keepalive period | `0.5` s (2 Hz) | unconstrained | `base_station/web_ui.py:1294` | live | Snapshot push cadence, plus one on connect and one after every canvas update. |
| /api/audit limit | `200` | clamped `max(1, min(limit, 5000))` | `base_station/web_ui.py:1202` | live | Trailing audit JSONL lines returned; tail read in 64 KiB chunks from EOF backwards. |
| LIFETRAC_AUDIT_PATH | `audit_log.DEFAULT_PATH` | unconstrained | `base_station/web_ui.py:1212` | live | File GET /api/audit tails. **The write path (`_get_audit_log` at `:508`) does not honour this var — reads and writes can diverge.** |
| /api/estop publish | qos 1, `wait_for_publish` timeout 1.0 s | 1.0 s bounded wait | `base_station/web_ui.py:1472` | live | E-stop latch to `lifetrac/v25/cmd/estop`. 503 if the broker is disconnected; "delivered" means the LOCAL broker confirmed, not that the tractor stopped. On paho <1.6 it optimistically reports True rather than blocking. |
| _CAMERA_IDS_FULL | auto 0x00, front 0x01, rear 0x02, implement 0x03, crop 0x04 | 0x00-0x04 | `base_station/web_ui.py:1619` | live | Camera name → CMD_CAMERA_SELECT byte, filtered against BuildConfig presence flags. With `cameras.count == 0` the whole table drops, even "auto". |
| CameraSelectBody.camera length | required | 1-16 chars | `base_station/web_ui.py:72` | live | Bound for POST /api/camera/select. |
| GeofencePlanRequest.swath_width_m | `2.5` | gt 0.5, lt 30.0 (exclusive) | `base_station/web_ui.py:2336` | live | Boustrophedon sweep spacing. **The polygon list itself has no length or coordinate-range bound.** |
| _base_quiescence() | parked 999.0 s, subscribers 0, m7_tx_queue_depth 0, engine idle True | n/a | `base_station/web_ui.py:1700` | live | Hard-coded trivially-true quiescence for the base ConfigWatcher (the base has no hydraulics), so reloads are never deferred. |
| LIFETRAC_UNIT_ID | None | unconstrained | `base_station/web_ui.py:724` | live | Selects the per-unit BuildConfig override; a load failure logs a warning and leaves BUILD=None (degraded-but-running console). |
| LIFETRAC_BUILD_CONFIG_PATH | unset (set and restored around validation) | unconstrained | `base_station/web_ui.py:1811` | live | Temporarily points build_config.load() at a candidate TOML; also the general loader override. |
| ui.max_control_subscribers / pin_required_for_control | `4` / `true` | max 1..32 | `base_station/config/build.default.toml:66` | live | Console control-socket cap and whether PIN auth gates control; codegen'd into firmware defines. |
| net.mqtt_host / net.mqtt_port | `localhost` / `1883` | port 1..65535 | `base_station/config/build.default.toml:94` | live | Build-config broker endpoint — a second source of truth alongside the env var. |
| config_bundle BUNDLE_VERSION / regexes | version 1; body sentinel `# --- body ---`; unit_id `^[a-z0-9-]{3,32}$`; sha `^[0-9a-f]{64}$` | see regexes | `base_station/config_bundle.py:42` | live | Signed config-bundle format and the accepted unit_id shape fleet-wide. |
| installer_daemon RESULT_* | `lifetrac-config-result.json`, version 1; statuses applied/rejected/noop/deferred | enumerated | `base_station/installer_daemon.py:77` | live | On-disk result contract the config installer writes for the UI. |
| config_watcher EVENT_* | noop, applied, deferred, restart_pending, firmware_required, rejected | enumerated | `base_station/config_watcher.py:59` | live | Outcome space for a live config change, including the states that decide whether a setting needs a reflash. |
| feedback LED_* / OLED_LINE_MAX | 5 LedPattern definitions; OLED_LINE_MAX 21 | n/a | `base_station/feedback.py:51` | live | Operator-visible LED patterns per config outcome and the OLED truncation width. |
| person_alert DEFAULT_MIN_CONFIDENCE / DEBOUNCE_S | `0.70` / `1.5` s | 0..1 / seconds | `base_station/person_alert.py:61` | live | Detection confidence floor and debounce before CMD_PERSON_APPEARED fires. |
| person_alert CLS_PERSON / ANIMAL / VEHICLE / OTHER | `0` / `1` / `2` / `0xFF` | uint8 | `base_station/person_alert.py:49` | live | Wire class-id space; must agree with the tractor detector's CLASS_TABLE. |
| app.js pollGamepad / LONG_PRESS_MS / refreshEncodeMode | 20 ms (50 Hz) / 800 ms / 5000 ms | unconstrained | `base_station/web/app.js:874` | live | Console input sample rate (which sets the real control-frame cadence to /ws/control), long-press threshold, status poll. |
| diagnostics.js WINDOW_S / TICK_MS | 60 s / 1000 ms | unconstrained | `base_station/web/diagnostics.js:23` | live | Diagnostics chart window and tick. |
| map.js HOME_LATLON / STALE_FIX_MS / NO_FIX_MS | `[37.7749, -122.4194]` / 5000 / 30000 | unconstrained | `base_station/web/map.js:19` | live | Map staleness thresholds. **HOME_LATLON is a hardcoded San Francisco placeholder, not the actual farm.** |
| tools/pair_handheld.py defaults | `/dev/ttyACM0`, 115200, 5.0 s, `config/keys`, `logs/audit_log.jsonl` | n/a | `tools/pair_handheld.py:40` | live | Handheld pairing transport and where pairing keys and the audit trail land. |
| tools/provision.py | baud 115200; KEY_LEN 16; output `firmware/common/lora_proto/key.h`; prologue `LIFETRAC-V25-PROV\n`; `--i-am-the-key-officer` gate | KEY_LEN 16 (AES-128) | `tools/provision.py:36` | live | Key-provisioning parameters and the explicit key-officer acknowledgement. |
| RESEARCH-CONTROLLER mqtt bridge credentials | host 192.168.1.100, port 1883, user `lifetrac`, pass `lifetrac_pass`, qos 1, keep_alive 60, protocol 5 | n/a | `RESEARCH-CONTROLLER/ros2_bridge/lifetrac_mqtt_bridge/config/mqtt_bridge_params.yaml:11` | deprecated | Archived research branch, not on the v25 path — but a **plaintext broker password and hardcoded LAN IP committed in the tree**. |
| RESEARCH web controller safety knobs | send_rate_hz 20, deadzone 0.1, max_value 1.0, timeout_ms 1000, emergency_stop_enabled true | five commented profiles, 10-30 Hz / 500-2000 ms | `RESEARCH-CONTROLLER/raspberry_pi_web_controller/config/config.example.yaml:32` | deprecated | Legacy control cadence and command-loss auto-stop; superseded, but still the only YAML-configurable safety timeout in the repo. |
| lifetrac-web-controller.service | n/a | n/a | `RESEARCH-CONTROLLER/raspberry_pi_web_controller/lifetrac-web-controller.service` | deprecated | A fifth systemd unit belonging to the archived research controller; confirm it is not deployed. |

---

## 7. Encoder & image pipeline

`camera_service.py` (capture, tiling, encode ladder, budgets), the fragmenter, the base-side
reassembler/canvas, and the browser render clients.

### 7.1 Capture and canvas geometry

| Setting | Default | Range | Where | Status | Purpose |
| --- | --- | --- | --- | --- | --- |
| LIFETRAC_GRID_W | `12` | `_env_int lo=1`, no hi | `firmware/tractor_x8/camera_service.py:97` | live | Tile columns; with TILE_PX gives CANVAS_W 384. |
| LIFETRAC_GRID_H | `8` | `_env_int lo=1`, no hi | `firmware/tractor_x8/camera_service.py:98` | live | Tile rows; gives CANVAS_H 256. |
| LIFETRAC_TILE_PX | `32` | `_env_int lo=1`, no hi | `firmware/tractor_x8/camera_service.py:99` | live | Tile edge in pixels; written into the frame header so the base decoder follows it. |
| CANVAS_W / CANVAS_H | `384 x 256` | derived | `firmware/tractor_x8/camera_service.py:100` | live | Logical canvas the capture backend produces and the slicer indexes. |
| TILE_BYTES_MAX | `256` | hard wire limit (`tile_size_minus1` is u8) | `firmware/tractor_x8/camera_service.py:102` | live | Per-tile blob ceiling; `_encode_tile` degrades quality until it fits and returns None rather than shipping a truncated RIFF (F14). |
| LIFETRAC_CAMERA_SOURCE | `"libcamera"` | `synthetic`; `v4l2`/`ffmpeg`/`usb`; **anything else falls into the libcamera branch** | `firmware/tractor_x8/camera_service.py:112` | live | Capture backend selector with a libcamera → v4l2 → synthetic fallback chain. Unknown values are not rejected. |
| LIFETRAC_CAMERA_DEVICE | `/dev/video1` | unconstrained | `firmware/tractor_x8/camera_service.py:119` | live | V4L2 node for the USB backend (bench-validated with a Kurokesu C2). |
| LIFETRAC_V4L2_INPUT_FORMAT | `"mjpeg"` | passed straight to `ffmpeg -input_format` | `firmware/tractor_x8/camera_service.py:120` | live | Stream format requested from the camera. |
| LIFETRAC_V4L2_INPUT_SIZE | `"1920x1080"` | passed straight to `-video_size` | `firmware/tractor_x8/camera_service.py:121` | live | Sensor resolution before the scale filter reduces it to the canvas. |
| LIFETRAC_V4L2_INPUT_FPS | `30` | `_env_int lo=1`, no hi | `firmware/tractor_x8/camera_service.py:122` | live | Sensor-side framerate (`-framerate`), independent of TARGET_FPS. |
| LIFETRAC_FFMPEG_PATH | `"ffmpeg"` | unconstrained | `firmware/tractor_x8/camera_service.py:123` | live | ffmpeg binary path (the bench used a static aarch64 build at `/tmp/ffmpeg`). |
| LIFETRAC_CAMERA_FPS | `2.0` | `_env_float lo=0.1`; loop period `1/max(fps, 0.1)` | `firmware/tractor_x8/camera_service.py:108` | live | Encoder loop rate; also injected as `fps=N` into the ffmpeg filter chain so stdout cannot accumulate stale frames. |
| LIFETRAC_CAMERA_DESHAKE | `"1"`; `"0"` when USE_LORA_BRIDGE=1 | raw string `== "1"` | `firmware/tractor_x8/camera_service.py:237` | live | Appends `deshake=open2=1:search=16`. Off in bridge mode because stabilisation jitter inflates the tile diff. |
| ffmpeg low-latency flags | `-thread_queue_size 2 -fflags nobuffer -flags low_delay -analyzeduration 0 -probesize 32` | n/a | `firmware/tractor_x8/camera_service.py:278` | live | Bounds capture latency so stdout never queues stale rawvideo. |
| V4l2FfmpegCamera restart policy | 2 grab attempts; `proc.wait(timeout=1.0)` then kill; stderr tail 200 chars | n/a | `firmware/tractor_x8/camera_service.py:354` | live | Restarts ffmpeg on EOF/short read; raises only after the retry also fails. |
| LIFETRAC_M7_UART | `/dev/ttymxc1` | unconstrained | `firmware/tractor_x8/camera_service.py:129` | live | X8→H747 UART for the IpcWriter and KISS back-channel; both skipped under USE_LORA_BRIDGE=1. |
| LIFETRAC_CAMERA_DEBUG_MQTT | `""` (disabled) | `.strip() == "1"`; forced True when USE_LORA_BRIDGE is set | `firmware/tractor_x8/camera_service.py:130` | live | Enables the MQTT frame mirror and the whole control-plane subscriber. Without it the service is UART-only and deaf to LoRa commands. |
| LIFETRAC_USE_LORA_BRIDGE | `""` (disabled) | `.strip() == "1"` | `firmware/tractor_x8/camera_service.py:136` | live | Strict-path master switch: skips the M7 writer and back-channel, forces DEBUG_MQTT, and flips the defaults of IMAGE_METHOD (A→C), TILE_MAGNITUDE_MIN (8000→4000), DESHAKE (1→0) and the fragment-budget fallback. |
| LIFETRAC_MQTT_HOST (camera_service) | `"localhost"` | unconstrained | `firmware/tractor_x8/camera_service.py:113` | live | Same env name as the TX daemon's, **different default**. |
| MQTT port (camera_service) | `1883`, hardcoded in `client.connect(...)` | n/a | `firmware/tractor_x8/camera_service.py:1490` | live | No env var on the camera side, unlike LIFETRAC_MQTT_PORT. |
| camera_service MQTT topics | PUBLISH `cmd/image_frame`; KEYFRAME_REQ `cmd/req_keyframe`; ENCODE_MODE_OVERRIDE, TRACTOR_KEYFRAME, ENCODE_MODE_STATUS, LINK_BUDGET | n/a | `firmware/tractor_x8/camera_service.py:140` | live | Control topics subscribe at qos 1, image frames publish at qos 0, encode-mode ack is retained. |
| LIFETRAC_CAMERA_HEALTH_LOG | `""` (disabled) | `.strip() == "1"` | `firmware/tractor_x8/camera_service.py:1506` | diagnostic | Frozen-sensor detector: reports consecutive frames hashing identically over the first 4096 canvas bytes. |
| LIFETRAC_CAMERA_HEALTH_EVERY_S | `2.0` | `_env_float lo=1.0` | `firmware/tractor_x8/camera_service.py:1507` | diagnostic | Min spacing between frame_health lines. |
| LIFETRAC_SYNTHETIC_MODE | `"scroll"` | only `"delta"` branches; every other value is the legacy scrolling gradient | `firmware/tractor_x8/camera_service.py:174` | diagnostic | "delta" freezes the background and toggles one tile so P-frames stay ~1 tile; "scroll" re-encodes every pixel each frame. |
| cameras.count / coral_tpu / imu_model / gps_model | 1 / `mini_pcie` / `bno086` / `neo_m9n` | count 0..4; enums per schema | `base_station/config/build.default.toml:44` | live | Fleet-shape leaves gating camera/accelerator/sensor code paths and HIL gate applicability. |
| usbcore autosuspend / udev power control | `power/control=on`, `autosuspend_delay_ms=-1` for VID:PID 16d0:0ed4 | n/a | `firmware/x8_lora_bootloader_helper/99-w2-01-c2.rules:25` | live | Disables USB runtime PM for the Kurokesu C2 — the mitigation for the ci_hdrc-imx host-controller wedge. |
| /dev/lifetrac-c2 symlink + SYSTEMD_WANTS | `SYMLINK+="lifetrac-c2"` on the index-0 capture node; restarts lifetrac-camera.service | n/a | `firmware/x8_lora_bootloader_helper/99-w2-01-c2.rules:34` | live | Stable camera name so LIFETRAC_CAMERA_DEVICE need not chase `/dev/videoN` drift. |
| snd_usb_audio blacklist | `install snd_usb_audio /bin/false` | n/a | `firmware/x8_lora_bootloader_helper/lifetrac-no-usb-audio.conf:26` | live | Removes isochronous bandwidth pressure during the enumeration window where the X8 host controller wedges. |

### 7.2 Encode ladder, quality, and byte budget

| Setting | Default | Range | Where | Status | Purpose |
| --- | --- | --- | --- | --- | --- |
| LIFETRAC_ENCODE_MODE | `0` (full) | `_env_int` with no lo/hi, then `_clamp_encode_mode`: only {0,1,2,3,6,8} implemented; **4, 5, 7 and anything else silently clamp to 1 (y_only)** | `firmware/tractor_x8/camera_service.py:515` | live | Selects the per-tile encode strategy and the wire codec byte. Mutable at runtime by `_apply_encode_mode`. |
| ENCODE_MODE ids + implemented set | 0 full, 1 y_only, 2 motion_only, 3 wireframe, 4 btc4_per_tile, 5 btc4_per_frame, 6 mono_g4, 7 adaptive, 8 rawstream; implemented {0,1,2,3,6,8} | unimplemented ids clamp to 1 | `firmware/tractor_x8/camera_service.py:467` | live | Modes 4, 5 and 7 have names and codec rows but no encoder — a stale base cannot brick tractor video. |
| EncodeMode / ENCODE_MODE_LADDER | FULL 0 … ADAPTIVE 7; ladder = (FULL, Y_ONLY, BTC4_PER_TILE, BTC4_PER_FRAME, MONO_G4) | 0..7 on the wire; ladder covers 0,1,4,5,6 | `base_station/lora_proto.py:80` | live | Stable wire values for CMD_ENCODE_MODE / 0x63. ADAPTIVE(7) and the two legacy modes are deliberately not in the auto-fallback ladder. |
| CMD_OP_ENCODE_MODE | `0x63` | mode 0..7; quality 1..100 | `base_station/lora_proto.py:957` | live | Strict-path opcode, args {mode id, optional quality}. |
| CMD_OP_ENCODE_MODE_ACK | `0x68` | <= 200 B | `base_station/lora_proto.py:961` | live | Tractor→base UTF-8 JSON ack. |
| LIFETRAC_WEBP_QUALITY | `55` | `_env_int lo=20 hi=100` (IP-208 clamp) | `firmware/tractor_x8/camera_service.py:111` | live | Baseline tile quality with no ROI planner. **Mutable at runtime** by `_apply_encode_mode` (`:1215`) and CMD_CAMERA_QUALITY (`:1287`). |
| LIFETRAC_MOTION_ONLY_QUALITY | `30` | `_env_int lo=5 hi=100` | `firmware/tractor_x8/camera_service.py:513` | live | Hard ceiling applied as `min(requested, this)` when mode == 2. |
| LIFETRAC_WIREFRAME_QUALITY | `20` | `_env_int lo=5 hi=100` | `firmware/tractor_x8/camera_service.py:514` | live | Hard ceiling when mode == 3. |
| per-call quality clamps | `_encode_tile` max(5, min(100, q)); `_apply_encode_mode` max(20, …); CMD_CAMERA_QUALITY max(20, …) | note the encoder floor is 5 while both command paths floor at 20 | `firmware/tractor_x8/camera_service.py:642` | live | Bounds any externally supplied quality; the 0x63 quality byte is pre-filtered 1..100 at `image_tx_daemon.py:688` and again at `camera_service.py:1484`. |
| WebP encoder method (effort) | `6` for keyframe tiles, `4` for delta tiles | PIL WEBP method 0-6 | `firmware/tractor_x8/camera_service.py:650` | live | Trades encode CPU for compression; the gray-q5 last-resort retry also uses 6. Not env-settable. |
| tile quality degrade ladder | `while quality >= 5: encode, else quality -= 10` | floor 5 | `firmware/tractor_x8/camera_service.py:651` | live | Shrinks a tile until it fits TILE_BYTES_MAX; exhaustion → grayscale q5 retry, then drop (F14). |
| MONO_G4 zlib level | `zlib.compress(level=9)`, falls back to raw packed bits if zlib grows it | output <= `1 + (TILE_PX²)/8` = 129 B | `firmware/tractor_x8/camera_service.py:609` | live | 1-bit tile for ENCODE_MODE_MONO_G4; the flag byte's bit0 tells the decoder whether the payload is compressed. |
| LIFETRAC_CONTAINER_STRIP | `"0"` (off) | raw string `== "1"` | `firmware/tractor_x8/camera_service.py:559` | live | Strips the 20-byte RIFF container from FULL-mode tiles without a CMD_ENCODE_MODE round trip; honoured by both `_encode_tile` and `_codec_for_mode` so bytes and codec byte cannot desync. |
| LIFETRAC_IMAGE_METHOD | `"C"` when USE_LORA_BRIDGE=1, else `"A"` | {A, B, C} after strip/upper; anything else warns and reverts | `firmware/tractor_x8/camera_service.py:704` | live | A = byte-diff bitmap + row-major; B = L1-magnitude ranked; C = B plus the rotating fair-share sweep. B/C silently fall back to A without numpy. |
| LIFETRAC_TILE_MAGNITUDE_MIN | `4000` under bridge mode, else `8000` | `_env_int lo=0`, no hi | `firmware/tractor_x8/camera_service.py:714` | live | Method B/C L1 noise floor over the 32x32x3 tile. Too high and slow real motion reads as static. |
| LIFETRAC_SWEEP_STEP | `2` | `_env_int lo=0`; 0 disables (explicit `> 0` guard at `:938`) | `firmware/tractor_x8/camera_service.py:721` | live | Method C: longest-unsent tiles forced into each P-frame so a static scene converges (~48 s on a 96-tile canvas at 1 fps). |
| LIFETRAC_TILE_AGE_ESCALATE_FRAMES | `40` | `_env_int lo=0`; **0 disables escalation** | `firmware/tractor_x8/camera_service.py:523` | live | Starvation guard: an over-age tile jumps to the front of pack order and gates the over-budget liveness valve. |
| LIFETRAC_OVERFLOW_SCAN_LIMIT | `6` | `_env_int lo=1`, no hi | `firmware/tractor_x8/camera_service.py:527` | live | After the first budget overflow, how many more tiles the greedy packer may try. A second break, `tile_cap - used < 6`, fires independently at `:1092`. |
| aged-tile over-budget liveness valve | one over-budget admission per frame | gated off entirely when TILE_AGE_ESCALATE_FRAMES <= 0 | `firmware/tractor_x8/camera_service.py:1041` | live | Guarantees a tile that can never fit still ships eventually; that frame spills to ~2 fragments, then packing stops. |
| byte-budget header charge | `tile_cap = max(0, cap - 6 - bitmap_bytes)` | 6 B header + `(n_tiles+7)//8` bitmap (12 B at 12x8) | `firmware/tractor_x8/camera_service.py:1029` | live | Charges header and bitmap against the budget. Before this a "243 B" budget produced 261 B — one runt fragment per budget-full frame. |
| dropped-tile carry splice | enabled whenever a cap is active and canvas sizes match | skipped on a canvas-size change | `firmware/tractor_x8/camera_service.py:1130` | live | Splices OLD pixels back into dropped tile regions so a changed-but-unsent tile re-flags next frame at full priority. |
| LIFETRAC_FRAGMENT_BUDGET | `""` (no cap) | **bare `int()`** in a local try/except; garbage or `<= 0` silently disables the budget | `firmware/tractor_x8/camera_service.py:778` | live | Fragment count converted to a per-frame wire byte cap. Not routed through `_env_int` — a typo silently removes the cap rather than warning. |
| LIFETRAC_FRAGMENT_BUDGET_BRIDGE_DEFAULT | `"12"` | same bare-parse path | `firmware/tractor_x8/camera_service.py:782` | live | Budget used when USE_LORA_BRIDGE=1 and the explicit budget is unset, so bridge mode is bandwidth-capped by default. |
| LIFETRAC_FRAGMENT_PROFILE | `"image"` | looked up in `PHY_BY_NAME`; an unknown name leaves the frame **uncapped** (deliberate) | `firmware/tractor_x8/camera_service.py:791` | live | PHY name used to convert a fragment count into a byte cap; superseded at runtime by the retained link_budget message. |
| _compute_link_bytes fallback_bytes | `n_fragments * 40` | n/a | `firmware/tractor_x8/camera_service.py:756` | live | Conservative guess used only when lora_proto / fragment cannot be imported. A known-but-unrecognised profile name returns None instead. |
| LIFETRAC_IMAGE_FRAGMENTS_PER_FRAME | unset — falls back to the caller-supplied count | published value is `max(1, n)` | `firmware/tractor_x8/image_tx_daemon.py:879` | live | Overrides the per-frame fragment budget published on the link_budget topic; parsed defensively because it runs inside the paho on_connect callback. |
| TRACTOR_LINK_BUDGET_TOPIC | `lifetrac/v25/tractor/link_budget` | n/a | `firmware/tractor_x8/image_tx_daemon.py:300` | live | Retained {n_fragments, profile_index, ts} so the encoder sizes frames to the real air quantum (243 B DTS / 203 B FHSS) instead of a boot-frozen env default. |
| _PROFILE_TO_LINK_PHY_IDX | `{0: 5, 1: 5, 2: 6}`, fallback 5 | indices into LINK_PHY_NAMES | `firmware/tractor_x8/image_tx_daemon.py:303` | live | Active profile → LINK_PHY_NAMES index published on link_budget. |
| LIFETRAC_ROI_ENABLE | `""` (disabled) | `.strip() == "1"` | `firmware/tractor_x8/camera_service.py:1347` | live | Without it every changed tile is rank 0 at plain WEBP_QUALITY and the ROI split is inert. |
| LIFETRAC_ROI_QUALITY_INSIDE | `WEBP_QUALITY + 10` → 65 | `_env_int lo=20 hi=100` | `firmware/tractor_x8/camera_service.py:730` | live | Quality inside the ROI mask. The default **snapshots WEBP_QUALITY at import**, so later runtime quality changes do not move it. |
| LIFETRAC_ROI_QUALITY_OUTSIDE | `30` | `_env_int lo=5 hi=100` | `firmware/tractor_x8/camera_service.py:732` | live | Quality outside the ROI; these tiles are also dropped first when the byte budget bites. |
| LIFETRAC_ROI_DEFAULT_MODE | `"idle"` | keys of DEFAULT_ROIS_BY_MODE: loading, driving, idle, reverse | `firmware/tractor_x8/camera_service.py:1352` | live | Initial ROI mode; "idle" is the whole frame. |
| roi.DEFAULT_ROIS_BY_MODE | loading (1,10,4,7); driving (3,8,0,4); idle (0,11,0,7); reverse (3,8,4,7) | inclusive col/row bounds on the 12x8 grid | `firmware/tractor_x8/image_pipeline/roi.py:21` | live | Per-mode default ROI rectangles. |
| RoiPlanner grid / hint_ttl_ms | grid 12x8, hint_ttl_ms 5000, mode "idle" | n/a | `firmware/tractor_x8/image_pipeline/roi.py:38` | live | An operator-painted CMD_ROI_HINT expires after 5 s. The hint path only arrives over the M7 back-channel, which is disabled in bridge mode. |
| LIFETRAC_TILE_CACHE_ENABLE | `""` (disabled) | `.strip() == "1"` | `firmware/tractor_x8/camera_service.py:1371` | live | Per-tile WebP encode cache (IP-W2-05). Pure CPU saver — the wire payload is byte-identical because the key folds in raw RGB plus quality and mode. |
| LIFETRAC_TILE_CACHE_HISTORY | `4` | `_env_int lo=1`, no hi | `firmware/tractor_x8/camera_service.py:1374` | live | Recent encodes retained per tile for hash lookup. |
| tile_cache.DEFAULT_HISTORY | `4` | n/a | `firmware/tractor_x8/image_pipeline/tile_cache.py:32` | live | Constructor default; camera_service always passes an explicit value. |
| encoder loop period / slip resync | `period = 1.0 / max(TARGET_FPS, 0.1)`; on slip `next_t = now` (no catch-up) | n/a | `firmware/tractor_x8/camera_service.py:1504` | live | Paces capture/encode/publish and prevents a CPU-bound catch-up spiral. |

### 7.3 Frame format, fragmentation, reassembly

| Setting | Default | Range | Where | Status | Purpose |
| --- | --- | --- | --- | --- | --- |
| frame header layout | `BBBBBB` = frame_kind, seq&0xFF, grid_w, grid_h, tile_px, codec; then bitmap | seq rolls 0..255 | `firmware/tractor_x8/camera_service.py:1110` | live | TileDeltaFrame header. `frame_kind == 0x01` marks a keyframe — image_tx_daemon reads exactly this byte for never-batch / copies / parity policy. |
| HEADER_FIXED_LEN | `6` | 6 bytes | `base_station/image_pipeline/frame_format.py:77` | live | Fixed header, followed by a `(n_tiles+7)//8` changed-bitmap. |
| CODEC_* ids / CODEC_RESERVED_MAX | WEBP 0, MONO_G4 1, BTC4_PER_TILE 2, BTC4_PER_FRAME 3, WEBP_LUMA 4, WEBP_RAWSTREAM 5; max 15 | 0..15 | `base_station/image_pipeline/frame_format.py:82` | live | Per-frame codec ids; the parser rejects codec > 15. |
| CODEC ids (tractor copy) | same six ids | n/a | `firmware/tractor_x8/camera_service.py:531` | live | Duplicated from `frame_format.py` to avoid importing the base tree from the tractor. |
| _CODEC_NAMES | {0 webp, 1 mono_g4, 2 btc4_tile, 3 btc4_frame, 4 webp_luma, 5 webp_rawstream} | 0-5 | `base_station/image_rx_daemon.py:185` | live | codec id → `link_stats.rx_codec_name`, the implicit encoder ACK the UI compares against the operator pin. |
| TileDeltaFrame parser bounds | grid 1-32 per axis; tile_px > 0; tile blob 1-256 B; no trailing bytes | as listed | `base_station/image_pipeline/frame_format.py:148` | live | Envelope checks turning a malformed frame into FrameDecodeError so the caller requests a keyframe. |
| FRAME_BATCH_MAGIC | `0xB5` | count 1-255; each segment 1-65535 B; trailing bytes rejected | `base_station/image_pipeline/frame_format.py:39` | live | RS-3.1 batched container so 2-4 frames ride one train. Chosen to avoid colliding with frame_kind (0/1) or the 0xFB-0xFE magics. |
| TELEMETRY_FRAGMENT_MAGIC / _V2 / _PARITY | `0xFE` (v1, 4 B hdr) / `0xFD` (v2 redundancy, 5 B) / `0xFC` (XOR parity, 4 B) | copies 1..15; at most 256 fragments per payload | `base_station/lora_proto.py:819` | live | Fragment header magics; v2 adds a `(total_copies<<4) or copy_idx` byte with first-copy-wins dedup. |
| FRAGMENT_MAGIC / HEADER_LEN | `0xFE` / `4` | n/a | `base_station/image_pipeline/reassemble.py:51` | live | v1 header: magic, frag_seq, frag_idx, total_minus1. |
| FRAGMENT_MAGIC_V2 / HEADER_LEN_V2 | `0xFD` / `5` | total_copies 1-15; copy_idx must be < total_copies | `base_station/image_pipeline/reassemble.py:53` | live | v2 redundancy header; a bad header counts as v2_bad_header + decode_error. |
| FRAGMENT_MAGIC_PARITY / HEADER_LEN_PARITY | `0xFC` / `4` | n/a | `base_station/image_pipeline/reassemble.py:55` | live | XOR-parity header: magic, frag_seq, group_start, group_len; body is the group XOR. |
| DEFAULT_TIMEOUT_MS (reassembler) | `1500` | unconstrained | `base_station/image_pipeline/reassemble.py:57` | live | Inactivity window before a partial set is dropped; GC runs in both `feed()` and `tick()` (F16) so idle RX loops still time out. |
| LIFETRAC_REASSEMBLER_TIMEOUT_MS | `1500` | **bare `int()`** — a malformed value crashes at startup | `base_station/image_rx_daemon.py:1523` | live | Base-side reassembler GC timeout; also `--reassembler-timeout-ms`. |
| _image_reassembler timeout (web_ui) | module default 1500 | unconstrained | `base_station/web_ui.py:1056` | live | web_ui's own reassembler takes the module default — **LIFETRAC_REASSEMBLER_TIMEOUT_MS only affects image_rx_daemon**, so the two stages can disagree. |
| _COMPLETED_LRU_CAP | `64` | 64 entries | `base_station/image_pipeline/reassemble.py:98` | live | Bounded LRU of completed frag_seqs so late v2 copies count as redundant instead of re-opening a slot and double-completing a frame. |
| parity last-fragment exclusion | skip reconstruction when `missing_idx == total-1` | exactly 1 missing per group, and not the last index | `base_station/image_pipeline/reassemble.py:130` | live | The last fragment may be shorter than the group XOR width; reconstructing it appends padding the parser rejects (2026-07-24: a 512 B payload reassembled to a corrupt 609 B). A lost last fragment falls back to GC + keyframe request. |
| TELEMETRY_FRAGMENT_MAX_AIRTIME_MS | `25.0` | float ms | `base_station/lora_proto.py:833` | live | Per-fragment airtime cap for the P2 telemetry path's binary search. |
| LORA_HOP_HDR_LEN | `8` | module constant | `base_station/lora_proto.py:835` | live | Mirrors LORA_PKT_HDR_LEN; added to every body length before the airtime estimate. |
| TX_FRAME_BODY_MAX | `247` (= 255 − 8) | module constant | `base_station/lora_proto.py:836` | live | Hard ceiling on a TX_FRAME_REQ body; the `body_ceiling` argument of `max_image_fragment_body`. |
| LIFETRAC_KEYFRAME_COPIES | `1` | `_env_int lo=1`; v2 packer rejects >15 | `firmware/tractor_x8/image_tx_daemon.py:919` | live | Redundant copies of keyframe fragments. At the default the daemon auto-promotes to 2 when recent fragment loss exceeds 0.5 %. |
| LIFETRAC_PARITY_GROUP | `0` (off) | `_env_int lo=0`; >0 enables | `firmware/tractor_x8/image_tx_daemon.py:934` | live | Group size for interleaved XOR parity fragments. RX-side reconstruction ships; emission is opt-in (documented value 8). |
| LIFETRAC_FRAME_MAX_AGE_MS | `10000` | `_env_int lo=0` | `firmware/tractor_x8/image_tx_daemon.py:947` | live | F16a stale-frame cancellation: a queued frame older than this is dropped if a fresher one is queued. |
| CRYPTO_IMAGE_PLAIN_CRC32 | overhead 6 B (4 seq + 2 CRC), **no MAC** | P3 only | `base_station/lora_proto.py:588` | live | D14 split-trust image profile; `enforce_class_tag_boundary` rejects it for P0/P1/P2. |
| IMAGE_TOPIC_IDS | `{0x25, 0x28, 0x29}` | subset of TOPIC_BY_ID | `base_station/lora_proto.py:681` | live | Telemetry topics that ride the image PHY and classify as P3 (tile_delta, motion_vectors, wireframe). |

### 7.4 Base-side canvas, transcode, and browser render

| Setting | Default | Range | Where | Status | Purpose |
| --- | --- | --- | --- | --- | --- |
| Canvas.grid_w / grid_h / tile_px | 12 / 8 / 32 | parser enforces 1..32 per axis, tile_px > 0 | `base_station/image_pipeline/canvas.py:70` | live | Default canvas geometry (96 tiles, 384x256). A differing frame is refused with `request_keyframe` + "grid mismatch". |
| _image_canvas default grid | `Canvas()` = 12x8 @ 32 px | as above | `base_station/web_ui.py:1055` | live | Auto-adopts the upstream grid on the first differing keyframe (`:1086`) — before that fix any non-12x8@32 camera produced empty snapshots. |
| base_seq gap policy | `expected = (last+1) & 0xFF`; on mismatch apply tiles **and** request a keyframe | base_seq 0..255 | `base_station/image_pipeline/canvas.py:128` | live | Gap-tolerant merge; the old refuse-to-apply behaviour starved the canvas to keyframe-only coverage on a lossy link. A delta before any keyframe is still refused. |
| _CODEC_BADGE | WEBP→RAW, WEBP_LUMA→RECOLOURISED, MONO_G4→WIREFRAME; unmapped → RAW | Badge 0..6 | `base_station/image_pipeline/canvas.py:35` | live | Per-frame codec decides the default badge. RAWSTREAM (5) is unmapped and therefore publishes as RAW. |
| Canvas.overlay() badge guard | raises ValueError on `Badge.RAW` | 0 <= index < n_tiles | `base_station/image_pipeline/canvas.py:207` | live | Trust boundary: synthesized or held-over pixels can never be published as RAW. |
| _TRANSCODE_CACHE_MAX | `256` | 256 entries | `base_station/image_pipeline/canvas.py:44` | live | LRU on the (codec, blob) → WebP transcode cache (~2.6 full keyframes). CODEC_WEBP short-circuits the cache. |
| _TRANSCODERS registry | {MONO_G4, WEBP_LUMA, WEBP_RAWSTREAM}; WEBP short-circuits | codec 1, 4, 5 (plus pass-through 0) | `base_station/image_pipeline/codec_decode.py:118` | live | **BTC4_PER_TILE (2) and BTC4_PER_FRAME (3) are deliberately absent** — a tile carrying them raises CodecDecodeError and the canvas drops it and requests a keyframe. |
| mono_g4 re-encode quality | WEBP quality 70, method 4 | 0-100 / 0-6 | `base_station/image_pipeline/codec_decode.py:83` | live | Settings when the 1-bit tile is expanded to 8bpp gray and re-encoded for the browser. |
| StatePublisher defaults | accel_status "offline", encode_mode "full", needs_keyframe False, link_power {None, None}, link_stats None | n/a | `base_station/image_pipeline/state_publisher.py:43` | live | Authoritative server-side display state on every /ws/state push; tile blobs go base64 with age_ms and badge int. |
| DEFAULT_WINDOW (bg_cache) | `8` | deque maxlen | `base_station/image_pipeline/bg_cache.py:26` | live | Rolling per-tile history; the byte-wise mode is painted into a stale slot with Badge.CACHED. |
| DEFAULT_STALE_AFTER_MS | `3000` | unconstrained | `base_station/image_pipeline/bg_cache.py:27` | live | Tile age past which `fill_misses()` overlays the cached median. Fresh tiles are recorded only when their badge is RAW; tiles with no history stay a real gap. |
| DEFAULT_REFERENCE_TTL_MS | `30_000` | refresh at >= TTL; hard reject at > 2x TTL | `base_station/image_pipeline/recolourise.py:26` | live | How often the per-tile colour reference is refreshed from a FULL tile (scheme Z); emits `0x59 luma 0xC0 colour_ref` with Badge.RECOLOURISED. |
| motion-vector wire bounds | per vector `<Hbb` = u16 tile_index, i8 dx, i8 dy (4 B) | dx/dy −128..127 by struct; **docstring says −64..63, not enforced** | `base_station/image_pipeline/motion_replay.py:59` | live | Topic 0x28 replay. Pixels are not transformed server-side; the slot is re-stamped Badge.PREDICTED and the browser applies a CSS transform. |
| wireframe geometry gate | `apply()` returns False unless width/height match the canvas exactly | bitmap length must be `ceil(w*h/8)` | `base_station/image_pipeline/wireframe_render.py:67` | live | Topic 0x29 overlay bookkeeping; canvas slots are never modified. |
| FallbackRenderer.HUD_HEIGHT | `16` px | 16 px | `base_station/image_pipeline/fallback_render.py:41` | live | HUD strip below the composited canvas; composite saved as WEBP q70. Without PIL it degrades to a text digest. |
| newest_frame_wins _SEQ_MOD / _SEQ_HALF | `1<<16` / `1<<15` | tied to the 16-bit base_seq | `base_station/image_pipeline/newest_frame_wins.py:38` | live | Wrap modulus and half-window deciding when a sequence gap is a wrap vs a reorder. |
| web_ui keyframe-request publish | qos 1, reason truncated to 64 B | reason[:64] | `base_station/web_ui.py:1101` | live | Published whenever `Canvas.apply()` reports request_keyframe. This is the **second, independent** keyframe-request source the RS-4.14 experiment had to gate (Run P: 109 requests came from here). |
| LIFETRAC_ALLOW_CMD_IMAGE_FRAME | `""` (disabled) | exactly "1" enables | `base_station/web_ui.py:964` | diagnostic | Bench-only: ingests legacy tile payloads on `cmd/image_frame`. Off by default so a local synthetic publisher cannot override the authoritative radio path. |
| _ENCODE_MODE_UI_CHOICES | `("full", "y_only", "motion_only", "mono_g4")` | exactly those 4 | `base_station/web_ui.py:99` | live | 2026-07-26: btc4 modes removed because the tractor has no BTC4 encoder and silently clamps both to y_only. "wireframe" is on-wire only. |
| _ENCODE_MODE_CYCLE_ORDER | same 4 names | out-of-list snaps back to head | `base_station/web_ui.py:2142` | live | Rotation for the gamepad BACK/SELECT button and the on-screen pill; cycling persists so a restart cannot roll back. |
| EncodeModeBody.mode / quality | both None; at least one required | mode `^(full or y_only or motion_only or mono_g4)$`; quality 1-100 | `base_station/web_ui.py:114` | live | Both optional so a quality-only change need not restate a mode that may have been cycled since page load. |
| _VALID_ENCODE_MODES | `frozenset(EncodeMode)` = {0..7} | 0..7 | `base_station/image_rx_daemon.py:958` | live | Out-of-set modes are rejected rather than masked onto the air. Accepts 4/5 which the UI no longer offers. |
| encode-mode quality wire range | none (optional second args byte) | 1..100 inclusive; non-int coerces to −1 and is rejected | `base_station/image_rx_daemon.py:996` | live | Quality is only added to the 0x63 frame when in range; otherwise mode is sent alone with a warning. |
| TRACTOR_QUALITY_MIN / MAX | `20` / `100` | as listed | `base_station/image_rx_daemon.py:795` | live | Mirror of camera_service's clamp so `_ack_matches_body` computes the same effective quality. |
| ack-match policy (quality absent) | return False (keep retrying) | n/a | `base_station/image_rx_daemon.py:838` | live | An old tractor whose 0x68 ack carries no quality cannot confirm the quality half, so the command stays pending rather than claiming an unobserved convergence. |
| ENCODE_MODE_OVERRIDE_TOPIC | `lifetrac/v25/control/encode_mode_override` | unconstrained | `base_station/image_rx_daemon.py:167` | live | Operator pin subscribed at QoS 1 and turned into a 0x63 frame. |
| ENCODE_MODE_STATUS_TOPIC | `lifetrac/v25/status/encode_mode` | unconstrained | `base_station/image_rx_daemon.py:168` | live | Tractor 0x68 ack republished retained so the UI confirmation loop works without LAN to the tractor. |
| LIFETRAC_ENCODE_MODE_STORE | `<module dir>/.encode_mode_override` | unconstrained | `base_station/web_ui.py:230` | live | Persisted {mode, quality} JSON surviving a web_ui restart; a legacy bare-mode line is accepted and a stored "auto" coerces to "full". |
| _encode_mode_runtime_override | `"full"` | one of the UI choices | `base_station/web_ui.py:238` | live | In-process mode used by the pill/gamepad cycle and /api/encode_mode/current. |
| _encode_mode_runtime_quality | None (keep tractor default) | 1-100 or None | `base_station/web_ui.py:239` | live | None means the command carries mode only. **Never cleared back to None once set.** |
| LIFETRAC_CORAL_ENABLED | `""`; effective enabled default True | truthy set {1, true, yes, on} | `base_station/image_pipeline/accel_select.py:269` | live | Seeds `image.coral_enabled` only when the store has no value; thereafter the persisted operator toggle wins. |
| _REFRESH_INTERVAL_S (accel) | `30.0` | unconstrained | `base_station/image_pipeline/accel_select.py:45` | live | Background Coral re-detection poll (no udev dependency). The operator toggle is re-read on every `state()`/`is_active()`. |
| _WARMUP_TIMEOUT_S | `2.0` | unconstrained | `base_station/image_pipeline/accel_select.py:46` | live | Watchdog on pycoral warm-up (it can hang on a dead USB device); on expiry `usable` stays False. |
| Coral PCI/USB ids | PCI 1ac1:089a; USB pre-init 1a6e:089a, post-init 18d1:9302 | n/a | `base_station/image_pipeline/accel_select.py:40` | live | Needles matched against lspci/lsusb (2.0 s subprocess timeouts); pycoral enumeration is the last-resort probe. |
| _AUDIT_INTERVAL_S (detect) | `1.0` | unconstrained | `base_station/image_pipeline/detect.py:25` | diagnostic | Debounce on the accel_path audit entry. Both detect paths are Phase-1 stubs returning []. |
| _AUDIT_INTERVAL_S (superres) | `1.0` | unconstrained | `base_station/image_pipeline/superres.py:25` | diagnostic | Same debounce for superres; both enhance paths are Phase-1 pass-throughs. |
| canvas_renderer SYNTHETIC_THRESHOLD / REAL_HOLD_MS | `0.72` / `10000` ms | unconstrained | `base_station/web/img/canvas_renderer.js:33` | live | Browser thresholds for treating a frame as synthetic and how long a real frame is held. |
| source_guard SAMPLE_STRIDE / MIN_SAMPLES / MATCH_THRESHOLD / CONSECUTIVE_FRAMES | 8 / 96 / 0.72 / 3 | unconstrained | `base_station/web/img/source_guard.js:13` | live | Client-side guard detecting when the displayed image no longer matches the claimed source — a safety-relevant display-integrity check. |
| staleness_overlay STALE_MS / VERY_STALE_MS | `1000` / `5000` ms; overlay enabled by URL param or `localStorage.lifetracDiag='1'` | unconstrained | `base_station/web/img/staleness_overlay.js:16` | live | When the operator sees a stale-image warning, plus the hidden diagnostic toggle. |
| raw_mode_toggle `lifetrac.raw_mode` | unset → off | '0'/'1' | `base_station/web/img/raw_mode_toggle.js:15` | live | Per-browser raw-vs-enhanced choice in localStorage; pings /api/audit/view_mode. Outside any server-side store. |
| badge_renderer VALID / LABEL | ids 0-6 = RAW, CACHED, ENHANCED, RECOLOURISED, PREDICTED, SYNTHETIC, WIREFRAME | 0..6 | `base_station/web/img/badge_renderer.js:20` | live | Client mirror of the provenance id space; must stay in sync with `_CODEC_BADGE`. |

---

## 8. Safety timing

Deadmen, watchdogs, settle delays, profile-switch windows, and the build-config safety leaves.

> **Authority note.** The base's 150 ms `/ws/control` deadman is a courtesy stop. The authoritative
> fail-safe is the tractor M7's `CONTROL_TIMEOUT_MS` / `HEARTBEAT_TIMEOUT_MS`, which drop the valves to
> neutral independently of the base. There is no Python constant mirroring the M7 thresholds.

| Setting | Default | Range | Where | Status | Purpose |
| --- | --- | --- | --- | --- | --- |
| CONTROL_TIMEOUT_MS (M7) | `200` ms | compile-time `const uint32_t` | `firmware/tractor_h7/tractor_h7.ino:513` | live | A ControlFrame staler than this drops the valves to neutral. Not settable from base-station Python. |
| HEARTBEAT_TIMEOUT_MS (M7) | `500` ms | compile-time `const uint32_t` | `firmware/tractor_h7/tractor_h7.ino:512` | live | `pick_active_source()` needs a heartbeat within 500 ms **and** a control frame within CONTROL_TIMEOUT_MS; failing both yields SOURCE_NONE and neutral valves within one tick. |
| TAKECTL_LATCH_MS (M7) | `30000` ms | unconstrained | `firmware/tractor_h7/tractor_h7.ino:514` | live | Take-control latch duration on the tractor. |
| safety.m4_watchdog_ms | `200` ms | 100..500 (schema) | `base_station/config/build.default.toml:42` | live | How long the M4 waits without an M7 liveness tick (or loop-counter change) before tripping the safety relay; consumed at `firmware/tractor_h7/tractor_m4.cpp:121`. |
| safety.estop_topology / estop_latency_ms_max | `psr_monitored_dual` / `100` ms | topology enum; latency 50..200 ms | `base_station/config/build.default.toml:39` | live | E-stop chain topology and the max permitted latency asserted by the safety case. |
| safety.modbus_fail_latch_count | `10` | 3..50 | `base_station/config/build.default.toml:41` | live | Consecutive Modbus failures before the safety chain latches. |
| hydraulic.track_ramp_seconds / arm_ramp_seconds | `2.0` / `1.0` | track 0.5..5.0, arm 0.25..3.0 | `base_station/config/build.default.toml:16` | live | Release-ramp durations per axis; codegen'd as `LIFETRAC_HYDRAULIC_*_RAMP_SECONDS`. |
| hydraulic.ramp_shape | `"linear"` | enum linear / scurve | `base_station/config/build.default.toml:23` | live | "scurve" substitutes a half-cosine smoothstep that halves P95 jerk (BC-26 / K-A3). |
| hydraulic.spool_type / load_holding / valve_settling_ms | `tandem` / `spool_inherent` / `100` ms | enums per schema; settling 0..250 ms | `base_station/config/build.default.toml:29` | live | Hydraulic build variant and the delay between EFC reaching zero and the solenoid de-energising (BC-19). |
| ui.stick_curve_exponent | `1.0` | enum [1.0, 1.5, 2.0] | `base_station/config/build.default.toml:71` | live | `effective = sign(x)*abs(x)^n` applied post-deadband, pre-mixing; consumed at `firmware/tractor_h7/tractor_h7.ino:859`. |
| ui.axis_deadband | `13` (~10 % of int8 full scale) | 0..32 | `base_station/config/build.default.toml:91` | live | Deadband on already-int8-clipped logical axes (BC-29 / K-D3); consumed at `firmware/tractor_h7/tractor_h7.ino:646`. |
| ui.operator_profile | `"normal"` | enum normal / gentle / sport | `base_station/config/build.default.toml:85` | live | Preset that overrides confined_space_mode, ramp_shape and stick_curve at config-load time so codegen, audit and firmware all see post-override state. |
| ui.confined_space_mode_enabled | `false` | boolean | `base_station/config/build.default.toml:76` | live | True multiplies the release-ramp / reversal-decay duration by 3/2 (BC-27 / K-D2). |
| handheld LINK_STALE_MS / TICK_PERIOD_MS / AXIS_DEADBAND / DEBOUNCE_MS | `3000` / `50` (20 Hz) / `16` / `15` ms | unconstrained | `firmware/handheld_mkr/handheld_mkr.ino:79` | live | Handheld link-stale threshold, control TX cadence, stick deadband, button debounce. **AXIS_DEADBAND 16 is hardcoded and does not track the configurable `ui.axis_deadband` = 13 on the tractor.** |
| handheld pin map + OLED | A0-A3 axes, button base pin 2, TAKECTL 10, ESTOP 11 (LOW = latched), ESTOP LED 12; OLED 128x64 @ 0x3C | board pins / I2C 7-bit addr | `firmware/handheld_mkr/handheld_mkr.ino:31` | live | Handheld hardware assignment and OLED geometry. |
| /ws/control deadman timeout | `0.15` s (150 ms) | inline literal | `base_station/web_ui.py:1384` | live | On timeout, if base controls are allowed, an all-zero ControlFrame is published at qos 0 and seq/hb keep advancing. The same zeroized publish happens in the finally block on disconnect (`:1437`). |
| /ws/control TX rate limit | `0.05` s (20 Hz) | inline literal | `base_station/web_ui.py:1408` | live | Server-side throttle matching the air cadence. **The rate check runs before `_base_controls_allowed()` and before validation, so rejected frames still consume the slot.** |
| /ws/control raw payload cap | `512` bytes | 512 | `base_station/web_ui.py:1405` | live | IP-203 size check before JSON parsing; oversize frames are dropped without closing the socket. |
| /ws/control unauthenticated close code | `4401` | 4401 | `base_station/web_ui.py:1370` | live | FastAPI does not run cookie Depends for WebSockets, so the session cookie is checked by hand — the browser cannot stream control before unlocking. |
| ControlFrame seq / hb wrap | `seq & 0xFFFF`, `hb & 0xFF` | 0-65535 / 0-255 | `base_station/web_ui.py:1430` | live | Per-socket counters feeding the tractor's monotonic-sequence and liveness checks. |
| ControlMsg axis/button ranges | all zero; `extra="forbid"` | axes −127..127; buttons 0..0xFFFF; flags 0..0xFF | `base_station/web_ui.py:62` | live | Pydantic validation tighter than the on-air protocol so malformed input never reaches the transmitter. |
| _base_controls_allowed() | blocked when active_source in {handheld, autonomy} | active_source in {none, handheld, base, autonomy} | `base_station/web_ui.py:874` | live | Source arbitration: while the handheld or autonomy owns the vehicle the base transmits neither control frames nor its own zeroized deadman frame. |
| MAX_CONTROL_SUBSCRIBERS | `BUILD.ui.max_control_subscribers`, else `4` | whatever BuildConfig validates | `base_station/web_ui.py:794` | live | The only WS cap driven from BuildConfig; stops a credentialled-but-misbehaving client starving the event loop. |
| CMD_OP_CTRL_DITTO | `0x6B` | 2 B args; a shorter args field parses to None and is treated as no command | `base_station/lora_proto.py:983` | live | Repeat-last-ControlFrame. `ditto_applies()` honours it only when ref_seq matches the last applied seq, so a desync degrades to the 200 ms deadman rather than replaying a stale command. |
| SX1276_TX_TIMEOUT_GUARD_US | `150000UL` | compile-time | `firmware/murata_l072/radio/sx1276_tx.c:30` | live | Added to expected ToA to form the TX_DONE watchdog deadline; expiry yields TX_STATUS_TIMEOUT. |
| SX1276_TX_PLL_SETTLE_US | `1000UL` | compile-time | `firmware/murata_l072/radio/sx1276_tx.c:45` | live | Busy-wait after a hop retune before LBT/CAD/TX may sample. **The comment above it claims a "200 µs budget"; the constant is 1000 µs, 5x what the comment states.** |
| SX1276_RX_PLL_SETTLE_US | `1000UL` (fallback `#define` duplicated at `sx1276_rx.c:51`) | compile-time | `firmware/murata_l072/radio/sx1276_rx.c:276` | live | Busy-wait after every RX retune before re-arming RX-cont. **The comment at `:264` says it mirrors the TX value "(200 µs)" — both the mirrored value and the parenthetical are wrong; both constants are 1000.** |
| sx1276_modes RF-switch settle delay | `10U` µs | compile-time | `firmware/murata_l072/radio/sx1276_modes.c:87` | live | Lets the antenna switch settle before the modem changes mode. |
| Radio reset pulse timing | NRESET low, 1 ms, release to Hi-Z (input), 6 ms | compile-time | `firmware/murata_l072/radio/sx1276.c:312` | live | Semtech-pattern reset; the Hi-Z release is deliberate — push-pull HIGH release caused the OPMODE-stuck fault. |
| LBT_CAD_TIMEOUT_US | `20000UL` | compile-time | `firmware/murata_l072/radio/sx1276_lbt.c:17` | live | CAD poll deadline; on expiry `sx1276_cad_abort()` clears the latch (RS-4.9) and LBT returns ERROR. |
| scan SM TX interlock | early return while `sx1276_tx_busy()` | n/a | `firmware/murata_l072/radio/sx1276_rx.c:619` | live | Freezes the scan SM during a keyed TX; added after run-16 where scan ADVANCE yanked the PA mid-frame (100 % TX_TIMEOUT). |
| MM_SAFE_MODE_WINDOW_MS | `500U` | unconstrained | `firmware/murata_l072/boot/safe_mode.c:17` | live | Brick-recovery window at boot watching USART2 for the magic sequence before normal init. |
| SAFE_MODE_BAUD_COUNT | `3U` | must equal the `kSafeModeBauds` initialiser count (**not asserted**) | `firmware/murata_l072/boot/safe_mode.c:18` | live | Sizes `kSafeModeBauds[] = {921600, 115200, 9600}`. |
| SAFE_MODE_SLOT_MS | derived `500/3` = 166 | derived | `firmware/murata_l072/boot/safe_mode.c:19` | live | Per-baud listen slot; the last baud gets the remainder. The magic is the fixed 8-byte literal `A5 5A A5 5A 5A A5 5A A5` at `:27` — not a `#define`. |
| PLATFORM_FAULT_MAGIC | `0xFA075DECUL` | arbitrary 32-bit sentinel | `firmware/murata_l072/hal/platform.c:6` | live | Validity sentinel in the retained fault record; checked before the record is reported. |
| PLATFORM_HSE_READY_TIMEOUT | `800000UL` | unitless busy-wait loop count, **not milliseconds** | `firmware/murata_l072/hal/platform.c:7` | live | HSERDY wait during clock bring-up; exhaustion drops to HSI16 and later raises `CLOCK_HSE_FAILED` from `main.c:89`. |
| IWDG_BOOT_WINDOW_MS | `500U` | unconstrained | `firmware/murata_l072/config.h:108` | **dead** | Intended boot watchdog window (N-20). Zero references anywhere; there is **no IWDG driver at all** — no IWDG_KR/PR/RLR writes exist outside the reset-cause read at `hal/platform.c:28`. |
| IWDG_RUN_WINDOW_MS | `100U` | CFG validator 50..5000 | `firmware/murata_l072/config.h:109` | live | Its only consumer is the CFG_KEY_IWDG_WINDOW_MS default bytes at `host_cfg.c:121`. Reported over CFG_GET, never programmed into hardware. |
| HOST_TYPE_FAULT_URC | `0xF1`, 24-byte payload | fault codes 0x01..0x0D (`host_types.h:165`); 0x09 and 0x0C are benign diagnostics | `firmware/murata_l072/include/host_types.h:159` | live | {code, sub, rsvd, pc, lr, psr, bfar, uptime_ms}. |
| _PROFILE_TO_BUDGET_US | `{0: 380000, 1: 860000, 2: 930000}` µs per 1 s | per-profile constant | `firmware/tractor_x8/image_tx_daemon.py:195` | live | Host mirror of the firmware QoS gate, deliberately under the firmware caps (400000/400000/950000) so a paced TX can never draw ABORT_QOS. |
| AirtimeBudget.admit poll clamp | `min(max(wait, 0.005), 0.25)` | n/a | `firmware/tractor_x8/image_tx_daemon.py:272` | live | Bounds how long `admit()` sleeps: at least 5 ms, at most 250 ms, so shutdown stays responsive. |
| PER_FRAGMENT_TX_TIMEOUT_S | `3.0` s | n/a | `firmware/tractor_x8/image_tx_daemon.py:179` | live | Per-fragment TX_DONE deadline on the serial path (`:1002`) and the in-flight watchdog on v3 (`:1132`); a miss is charged to the airtime budget as RF-spent. |
| PROFILE_SWITCH_ACK_FLUSH_S | `0.7` s | n/a | `firmware/tractor_x8/image_tx_daemon.py:310` | live | Phase-1 dwell after flushing a RADIO_PROFILE_ACK so the last copy finishes radiating on the OLD grid before the modem retunes. |
| PROFILE_CONFIRM_TIMEOUT_S | `45.0` s | n/a | `firmware/tractor_x8/image_tx_daemon.py:311` | live | Revert watchdog: no RADIO_PROFILE_CONF on the new profile within this window reverts the tractor, so a half-switched link heals. Sized for worst-case FHSS reacquisition (~25 s) plus CONF flight. |
| PROFILE_ACK_TIMEOUT_S | `12.0` s | unconstrained | `base_station/image_rx_daemon.py:177` | live | Phase-A window the base holds on the OLD profile waiting for the tractor's ACK. |
| PROFILE_REVERT_TIMEOUT_S | `45.0` s | unconstrained | `base_station/image_rx_daemon.py:178` | live | Phase-B window: after switching locally, how long to wait for proof-of-life before reverting. |
| TrainGapMs (bench script) | `40` ms | `_env_float lo=0.0`, no hi; divided by 1000 | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:50` | live | RS-3.10 designed inter-train gap spent draining RX — it doubles as the reverse-slot command window. |
| TxInterS (RSSI sniff) | `0.25` s | unconstrained | `firmware/x8_lora_bootloader_helper/run_air_coupling_rssi_sniff.ps1:30` | diagnostic | Inter-burst spacing during the RSSI sniff. |
| LIFETRAC_PPS_DEVICE | `/dev/pps0` | unconstrained path | `firmware/tractor_x8/time_service.py:46` | live | PPS character device polled with the PPS_FETCH ioctl for the 1 Hz tick. |
| LIFETRAC_TIME_UART | `/dev/ttymxc0` | a missing device degrades to tick-only with a warning | `firmware/tractor_x8/time_service.py:47` | live | UART the 6-byte `<IH` time tick is written to. |
| PPS_FETCH ioctl | `0xC038_7003` | n/a | `firmware/tractor_x8/time_service.py:56` | live | `_IOWR('p', 0xa4, struct pps_fdata)`; hardcoded because magic 'p' plus dir/size are stable across kernels. |

---

## 9. Bench harness

`run_live_radio_monitor.ps1` and friends, the `make check` host suite, dev-build policy flags, and the
probe instrumentation. The value matrix from 70 archived runs is in [§11](#11-what-we-have-actually-tried-on-the-bench).

| Setting | Default | Range | Where | Status | Purpose |
| --- | --- | --- | --- | --- | --- |
| TxAdbSerial | `2E2C1209DABC240B` | unconstrained; never overridden in 69 recorded runs | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:11` | live | adb serial of the tractor-side X8 hosting tx_smoke, synth_pub and (local feed) the bench broker. |
| RxAdbSerial | `2D0A1209DABC240B` | unconstrained; never overridden | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:12` | live | adb serial of the base-side X8 hosting rx_smoke and the gpio163 NRST path. |
| HostIp | `192.168.1.79` | unconstrained; never overridden | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:13` | live | Bench-PC broker IP: TCP-1883 preflight/ARP pin, LIFETRAC_CTRL_MQTT_HOST on RX, TX feed host when `-TxFeed host`. |
| DurationS | `30` | unconstrained | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:14` | live | Length of the 5 s-poll monitoring loop; also sets the synth publisher lifetime to DurationS + 10. **The coded default was never used in any archived run.** |
| SynthFps | `2` | unconstrained | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:20` | live | Offered frame rate of the synthetic publisher — the saturation lever that decides whether reported goodput is a measurement or a traffic report. **The comment above it argues for 6 fps; the coded default is 2.** |
| SynthBudgetB | `250` | unconstrained | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:21` | live | Per-frame encoded byte budget; sets on-air frame size and fragments per train. Only 250 and 3000 have ever been used, in lockstep with SynthFps. |
| TxFeed | `"local"` | only `local` and `host` are handled; any other string falls into the host branch | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:55` | live | RS-5.9: "local" runs a bench broker + publisher on the tractor's own loopback (required since tractor WiFi is off); "host" uses the bench PC over LAN. **The "host" path has never been exercised since the knob was added.** |
| Archive | switch, absent = off | boolean | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:84` | live | Writes tx/rx/synth logs and params.txt into `bench-evidence/radio_monitor_<stamp>_<sha>/` — the only reason any run history exists. |
| ReactiveFire | `0` (off) | 0/1 in practice | `base_station/image_rx_daemon.py:290` | diagnostic | RS-0.12: fires a CMD_OP_PROBE at each fragment-RX-complete using the same TX machinery a real ControlFrame would, so its delivery rate is the drive-command delivery rate. |
| LIFETRAC_PROBE_PHASE_SWEEP_MS | `[0]` (blank or bad parse → single offset-0 bin) | non-negative ints, each floored at 0 | `base_station/image_rx_daemon.py:298` | diagnostic | Comma-separated ms offsets the probe fires at, round-robin, to measure P(delivery given phase). |
| LIFETRAC_PROBE_MIN_GAP_S | `0.5` | float seconds; malformed falls back to 0.5 with a warning | `base_station/image_rx_daemon.py:311` | diagnostic | Minimum spacing between reactive-fire probes. |
| LIFETRAC_PROBE_SIZES_B | `[0]` (no padding) | values below 16 are dropped (8 B hop hdr + 2 B magic/opcode + 6 B args) | `base_station/image_rx_daemon.py:325` | diagnostic | Total on-air payload sizes probes are padded to, so the sweep measures P(delivery given phase, size). |
| _probe_sizes floor | `16` | >= 16 | `base_station/image_rx_daemon.py:333` | diagnostic | Hard minimum on-air probe size; smaller requests are silently filtered out. |
| probe pending-map cap | cap 512, trims oldest 256 | 512 / 256 | `base_station/image_rx_daemon.py:1276` | diagnostic | Bounds `_probe_pending` / `_probe_bins` growth over a long reactive-fire run. |
| air-gap sample discard threshold | `0 < delta < 5_000_000` µs | 5 s upper bound | `base_station/image_rx_daemon.py:1117` | diagnostic | RS-2.3: deltas above 5 s are stream pauses, not gaps, and are excluded from the median/p95/histogram. |
| air_gap histogram edges | `(2, 5, 10, 20, 40, 80, 120, 200, 400, 1000)` ms | 11 buckets incl. >=1000 | `base_station/image_rx_daemon.py:1328` | diagnostic | Fixed-edge buckets; median and p95 alone hid the bimodal train-boundary mode a control frame must land in. |
| CMD_OP_PROBE | `0x69` | 6 B args before optional size padding | `base_station/lora_proto.py:969` | diagnostic | Strict-path probe {probe_seq, phase_ms}; the tractor only logs and echoes it. Gated by LIFETRAC_REACTIVE_FIRE. |
| CMD_OP_PROBE_ECHO | `0x6A` | 4 B args | `base_station/lora_proto.py:970` | diagnostic | Tractor→base echo carrying the probe seq for RTT timing; suppressible via LIFETRAC_PROBE_ECHO=0. |
| LIFETRAC_SYNTH_PREBUILD | hardcoded `32` | local-feed branch **only** | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:247` | live | Replays a prebuilt 32-frame bank so the paced publisher is CPU-free; without it the i.MX builds only ~3.3 fps and silently de-saturates the measurement. The host-feed branch never sets it. |
| LIFETRAC_SYNTH_DURATION_S | `DurationS + 10` | derived; the +10 s constant never varied | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:245` | live | Publisher self-exit time, deliberately outliving the measurement window. |
| LIFETRAC_MQTT_HOST (script derive) | TX `127.0.0.1` when local else `$HostIp`; RX and synth hardcoded `127.0.0.1` | derived | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:210` | live | RX deliberately publishes to the base board's own mosquitto because the base→host Wi-Fi leg drops ARP within a minute. |
| LIFETRAC_CTRL_MQTT_HOST (script) | `$HostIp` | derived from `-HostIp`; never independently varied | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:232` | live | Separate control-plane broker for the base daemon, split from the data plane. |
| container images (tx/rx/synth/broker) | `arduino-ootb-python-devel:738bc44` / `lifetrac-v25:latest` / `lifetrac-tractor-x8:latest` / `eclipse-mosquitto:2` | hardcoded, never parameterised, **never recorded in params.txt** | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:214` | live | TX and RX deliberately run different images; synth_pub needs the tractor image for numpy/cv2. A real reproducibility hole — `latest` tags are mutable across the 70-run history. |
| L072 device / work dir / sudo password | `--device=/dev/ttymxc3`; `-v /tmp/lifetrac_strict:/work`; sudo password literal `fio` | hardcoded throughout | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:131` | live | UART to the L072, the staging dir on both boards, and the credential piped into every adb shell. |
| adbExe fallback path | `(Get-Command adb).Source`, else the per-user WinGet platform-tools path | machine-specific hardcode | `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:96` | live | Resolves adb dynamically because the winget package dir name varies per machine. |
| AdbSerial / TxAdbSerial / RxAdbSerial / RepoRoot | tractor `2E2C1209DABC240B`, base `2D0A1209DABC240B`, or `""`/mandatory; RepoRoot `""` auto-resolves from `$PSScriptRoot` | Mandatory in diagnose_x8_recovery, run_w1_10b_rx_pair, run_w2_01_bench_validation, run_w2_01_camera_first_light, run_w2_01_v3_stress. **Not** mandatory in run_radio_sleep / run_radio_wake_rxcont (omitting auto-discovers all devices) | `firmware/x8_lora_bootloader_helper/run_targeted_pa9analog_cfgs.ps1:2` | live | Board-selection plumbing repeated across ~20 helper scripts. |
| bench_mqtt.conf | `listener 1883 127.0.0.1`, `allow_anonymous true` | n/a | `firmware/x8_lora_bootloader_helper/bench_mqtt.conf:8` | diagnostic | Tractor-local bench broker on the host netns loopback for the RS-5.9 local feed. |
| deploy compose project names | `lifetrac-vtest` (base), `tractor-vtest`; TimeoutStartSec 180 | n/a | `deploy/lifetrac-base-compose.service:15` | live | Which compose project/file the deployed video stacks bring up at boot; both units deliberately omit `--build` (documented destructive-loop hazard). |
| hil/gate_applicability.json | schema_version 1; per-gate `applies_when` lists | ops eq / gt / gte / truthy over build-config paths | `hil/gate_applicability.json` | live | Declarative predicates deciding which HIL gates apply vs report N/A; read by `hil/dispatch.ps1` and the SIL test. |
| HOST_CC | `gcc` on Windows_NT, else `cc` | any host C11 compiler | `firmware/murata_l072/Makefile:216` | live | Compiler for the toolchain-free `make check` suite (memory-map preprocess, static asserts, ~25 host_proto unit tests). Some tests inject `-DLIFETRAC_FHSS_TX_ROUTED` to exercise routed branches. |
| EXTRA_CFLAGS | (empty) | any gcc flags | `firmware/murata_l072/Makefile:35` | live | Per-target CFLAGS injection appended in the object rule; used by the recursive `lora_ping_*` targets. |
| PING_ROLE | `0` (0 = TX pinger, 1 = RX ponger) | 0/1 | `firmware/murata_l072/lora_ping.c:25` | diagnostic | Selects behaviour in the bench ping image; injected by the `lora_ping_tx` / `lora_ping_rx` make targets. |
| lora_ping.c standalone PHY | FRF 915 MHz, PA 0x8C, MODEM_CFG1 0x72 (**BW 125 kHz**), CFG2 0x74, CFG3 0x04, preamble 8, SYNC_WORD 0x12, DETECT 0x03/0x0A, FIFO bases 0x00 | all literals | `firmware/murata_l072/lora_ping.c:315` | diagnostic | Separate bring-up binary (PING_SRCS, **not** linked into production). The only place in the tree that programs preamble length, sync word and RegFifoRxBaseAddr. Its BW is 125 kHz, unlike production 250. |
| RADIO_MINIMIZER_DEFAULT | `1` | 0/1 | `firmware/murata_l072/include/dev_build_policy.h:14` | diagnostic | Brings dev images up with the radio quiet. Only consumer is `lora_ping.c:649`; `dev_build_policy.h` is included solely by lora_ping.c, so it has no effect on the main image. |
| RADIO_ACTIVE_TEST_DEFAULT | `0` | 0/1 | `firmware/murata_l072/include/dev_build_policy.h:26` | diagnostic | Opt-in gate for active RF on bench images; injected by the `lora_ping_*_active` make targets. Affects the ping images only. |
| POWER_CYCLE_AVOIDANCE_DEFAULT | `1` | 0/1 | `firmware/murata_l072/include/dev_build_policy.h:18` | **dead** | Intended dev-policy flag to avoid power-cycling the board; referenced by zero lines. |
| LIFETRAC_BENCH_BOOT_HEARTBEAT_ENABLE | `0` | 0/1 | `firmware/murata_l072/config.h:86` | diagnostic | Emits ASCII `LT_BOOT_HEARTBEAT` markers at post_uart_init and radio_ready/radio_fault (`main.c:53`, `:78`). |
| sx1276_reg_dump() readback window | registers `0x00..0x42`; requires `out_len >= 0x43` | 0x00..0x42 | `firmware/murata_l072/radio/sx1276.c:497` | **dead** | Bulk register readback. **No caller anywhere** — only the definition and the declaration at `include/sx1276.h:43`. Note it starts at 0x00 (RegFifo), so a dump would pop a FIFO byte. |
| LORA_FPD_LEGAL_DWELL_US | `400000UL` | pinned == 400000 by `bench/host_proto/frames_per_dwell.c:56` | `firmware/murata_l072/include/lora_frames_per_dwell.h:45` | diagnostic | Dwell budget for the frames-per-dwell pacing hint — an advisory mirror of the X8 host pipeline, not an enforcement path. |
| LORA_FPD_DWELL_HEADROOM_PCT | `85U` | pinned == 85 by the bench check | `firmware/murata_l072/include/lora_frames_per_dwell.h:46` | diagnostic | Slack for retransmits: 400000 * 85 / 100 = 340000 µs. |
| LORA_FPD_MAX_FRAMES_PER_DWELL_CAP | `8U` | return value always in [1,8] | `firmware/murata_l072/include/lora_frames_per_dwell.h:47` | diagnostic | Hard cap on the pacing hint regardless of ToA. |
| LORA_FPD_MIN_INTER_CYCLE_MS | `50U` | n/a | `firmware/murata_l072/include/lora_frames_per_dwell.h:44` | **dead** | Mirror of the host constant; not referenced by `lora_frames_per_dwell.c` — only asserted for parity at `bench/host_proto/frames_per_dwell.c:70`. |
| lora_frames_per_dwell() | n/a | returns [1,8] | `firmware/murata_l072/radio/lora_frames_per_dwell.c:9` | diagnostic | The pacing helper has no firmware caller; only the bench TU invokes it, so the whole TU is advisory. |
| bench profile SF/BW/CR mirror | p0 7/250/5, p1 7/250/5, p2 7/500/5 | n/a | `firmware/murata_l072/bench/host_proto/airtime_invariant.c:56` | diagnostic | Host-side ToA prediction mirror; must stay identical to what activation programs (it currently does for all three). |
| pinned worst-case ToA values | SF7/BW250/CR45/255B = 199808 µs; /24B = 30848 µs; SF12/BW125/0B = 663552 µs | n/a | `firmware/murata_l072/bench/host_proto/airtime_invariant.c:168` | diagnostic | Golden vectors proving the active FHSS tuple's worst case sits well under the 380000 µs cap. |
| chantab golden vectors | idx0 902750000 … idx49 927250000 Hz | all centres within [902.75, 927.25] MHz | `firmware/murata_l072/bench/host_proto/fhss_chantab_vectors.c:37` | diagnostic | Cross-language parity contract with `fhss_chantab.py`; asserts uniform 500 kHz spacing and that idx >= 50 returns 0. |
| slot-clock geometry pins | SLOT_MS 200; SLOT_MS*COUNT 10000; TX_GUARD_US 15000; TX_HEADSTART_MS 12 | n/a | `firmware/murata_l072/bench/host_proto/fhss_clock_test.c:100` | diagnostic | Golden pins that fail the host check build if the grid geometry drifts. |
| CSMA_DEFAULT_BUSY_DBM / MAX_SKIPS | `-90` dBm / `4` | max_skips >= 0 | `base_station/lora_proto.py:646` | **dead** | `pick_csma_hop()` skip-busy thresholds; matches the firmware LBT default, but nothing in the daemons calls it. |
| LIFETRAC_FRAGMENT_BUDGET_DEMO | `"250"` | **truly bare `int()`** — no try/except, no clamp; non-numeric raises and aborts the harness | `firmware/tractor_x8/method_compare.py:156` | diagnostic | Byte budget for the A/B/C method-comparison demo. The one remaining unguarded `os.environ` parse in the tractor tree. |
| method_compare env writes | sets `LIFETRAC_IMAGE_METHOD`; `setdefault("LIFETRAC_FRAGMENT_BUDGET", "")` | n/a | `firmware/tractor_x8/method_compare.py:47` | diagnostic | Forces the plan revision and neutralizes the auto budget before re-importing camera_service. |
| dry_run_w2_05 gates | 4 gates: default keyframe payload, CMD_LINK_PROFILE budget, capped payload, tile-cache hits | n/a | `firmware/tractor_x8/dry_run_w2_05.py:144` | diagnostic | Import-time smoke test pinning grid/quality/LINK_PHY_NAMES and exercising the budget and cache seams without hardware. |
| LIFETRAC_ENABLE_BENCH_RADIO | `"1"` — **enabled by default** | case-insensitive {1, true, yes, on}; anything else → HTTP 403 | `base_station/web_ui.py:1581` | live | Gate on POST /api/bench/radio, which shells out to the L072 sleep/wake helpers. Default-on means a production deployment must explicitly disable it. |
| bench radio subprocess limits | timeout 180 s, output_tail 40 lines | n/a | `base_station/web_ui.py:1543` | diagnostic | Run off the event loop behind an asyncio single-flight lock because both scripts drive the same physical L072; concurrent calls get HTTP 409, a timeout returns rc 124. |
| BenchRadioBody.mode | required | `^(sleep or wake)$` | `base_station/web_ui.py:135` | diagnostic | Which L072 helper script POST /api/bench/radio invokes. |
| DurationS (concurrent smoke) | `30` | RegProfile is **not** parameterised here — profile 0 only | `firmware/x8_lora_bootloader_helper/run_concurrent_smoke.ps1:14` | live | Concurrent TX+RX smoke window; hardcodes LIFETRAC_REG_PROFILE=0 and LIFETRAC_SKIP_RESET_REQ=1. |
| VideoDev / InterFragS / Quality / ExtraRxWindowS | `/dev/video1` / 0.2 / 55 / 30 | unconstrained | `firmware/x8_lora_bootloader_helper/run_w2_02_image_over_lora_end_to_end.ps1:56` | live | W2-02 image-over-LoRa harness. |
| Redundancy (w2_02 v2) | `2` | **validated: throws if < 1** | `firmware/x8_lora_bootloader_helper/run_w2_02_image_over_lora_end_to_end_v2.ps1:60` | live | Fragment repeat factor; the only param differing from v1. Predates the RS-4.1 ParityGroup approach to the same loss problem. |
| N / InterRunSleepS / InterFragS / ExtraRxWindowS | 10 / 5 / 0.2 / 30 | unconstrained | `firmware/x8_lora_bootloader_helper/run_w2_02_stability_loop.ps1:24` | live | Repeats the W2-02 orchestrator N times with a cooldown. |
| ContainerName / Image / CameraDevice / V4l2InputFormat / M7Uart / FfmpegPath / CameraFps / deshake switches | `lifetrac-camera-x8` / ootb image / `/dev/video1` / mjpeg / `/dev/ttymxc1` / `/tmp/ffmpeg` / 2 / off | NoDeshake and UseDeshake are opposing switches resolved into one value | `firmware/x8_lora_bootloader_helper/run_camera_container_x8.ps1:3` | live | Launches the real camera container and sets the LIFETRAC_CAMERA_* env set. |
| Width / Height / PixFmt / capture + guard switches | 1280 / 720 / MJPG; FullCapture off (safe enumerate-only); PerAttemptTimeoutS 8; GuardSettleMs 800 | several switches default OFF into the safe path by design | `firmware/x8_lora_bootloader_helper/run_w2_01_camera_first_light_end_to_end.ps1:52` | live | Camera first-light bring-up plus USB-guard sub-harness. `NoGuard` explicitly overrides `RunGuard`. |
| Iterations / SleepMs / EvidenceRoot | 50 / 200 ms / (unset) | EvidenceRoot has no default and no Mandatory attribute — it can be empty at runtime | `firmware/x8_lora_bootloader_helper/run_w2_01_v3_stress.ps1:31` | live | Camera v3 USB-guard stress loop. |
| Cycles / Install / Interactive / SysfsOnly | 1 / off / off / off | AdbSerial is mandatory here | `firmware/x8_lora_bootloader_helper/run_w2_01_bench_validation.ps1:64` | live | W2-01 bench validation entry point. |
| Cycles / CycleTimeoutSec / RunGate | 20 / 180 s / `$true` | RunGate is `[bool]`, not `[switch]` — pass `-RunGate $false` to disable | `firmware/x8_lora_bootloader_helper/run_stage1_standard_quant_end_to_end.ps1:5` | live | Stage-1 quantitative contract run. |
| SummaryPath / ExpectedCycles | mandatory / `0` | ExpectedCycles 0 = do not enforce a cycle count | `firmware/x8_lora_bootloader_helper/run_stage1_quant_gate.ps1:3` | live | Pass/fail gate over a stage-1 summary file. |
| DeviceDir / KeepDeviceFiles | `/var/rootdirs/home/fio/lifetrac_w2_05` / off | unconstrained | `firmware/x8_lora_bootloader_helper/run_w2_05_dry_run.ps1:25` | live | W2-05 dry-run staging directory on the board. |
| Image / CameraDevices (UVC probe) | ootb image / `("/dev/video1", "/dev/video3")` | `[string[]]` | `firmware/x8_lora_bootloader_helper/probe_uvc_controls_x8.ps1:4` | diagnostic | Enumerates UVC controls for both camera nodes; the `_v2` script carries identical defaults. |
| FfmpegPath (V4L2 format probe) | `/tmp/ffmpeg` | unconstrained | `firmware/x8_lora_bootloader_helper/probe_v4l2_formats_x8.ps1:4` | diagnostic | ffmpeg binary staged on the board for format enumeration. |
| ContainerName (camera check) | `lifetrac-camera-x8` | unconstrained | `firmware/x8_lora_bootloader_helper/check_camera_container_x8.ps1:4` | diagnostic | Camera container health check. |
| RunDir / AiNotesRelPath | `""` / `LifeTrac-v25/AI NOTES` | empty RunDir = latest run | `firmware/x8_lora_bootloader_helper/pull_stress_report_to_ai_notes.ps1:3` | live | Pulls a stress report off the board into the notes tree. |
| ReportPath | `""` | unconstrained | `firmware/x8_lora_bootloader_helper/update_latest_stress_status.ps1:3` | live | Report folded into the latest-status summary; empty = auto-discover. |

---

## 10. Dead or diagnostic-only settings

**Read this section before changing anything at the bench.** Every row below is a knob that will not do
what its name suggests. "Dead" means changing it has *no runtime effect at all*.

### 10.1 Dead — changing these does nothing

| Setting | Where | Consequence of relying on it |
| --- | --- | --- |
| ~~Power-clamp result → RegPaConfig~~ | `firmware/murata_l072/host/host_cfg.c` | **FIXED 2026-07-29 — no longer dead.** Activation now programs `min(configured, erp_max)` into the PA, and `CFG_KEY_TX_POWER_DBM` writes are capped at the active allowance. Declaring 12 dBi now genuinely reduces conducted power to 11 dBm. |
| RegSymbTimeout 0x1F + ModemConfig2 bits 1:0 | `firmware/murata_l072/radio/sx1276.c:453` | Register 0x1F is never written (POR retained) and 0x1F is not in the REG_WRITE allowlist. Only matters for RX_SINGLE, which is never entered. |
| `sx1276_modes_to_rx_single(timeout_symbols)` | `firmware/murata_l072/radio/sx1276_modes.c:197` | The argument is discarded with `(void)` and no caller exists. RX_SINGLE is unreachable. |
| RegIrqFlagsMask 0x11 | `firmware/murata_l072/radio/sx1276.c:330` | Never written; all IRQs stay unmasked. You cannot mask an IRQ source — the firmware relies on DIO EXTI edges plus a full RegIrqFlags read. |
| RegFifoRxBaseAddr 0x0F | `firmware/murata_l072/radio/sx1276_rx.c:17` | Never written by the main firmware (only `lora_ping.c`). RX base overlaps TX base at 0, which is why `main.c:152` orders the mailbox drain after RX service. |
| RegPreambleMsb/Lsb 0x20/0x21 | `firmware/murata_l072/radio/sx1276_airtime.c:171` | **Preamble length is not settable.** Only read back by the airtime estimator; the chip POR of 8 symbols is what is on air, and the invariant assumes 8. |
| RegSyncWord 0x39 | `firmware/murata_l072/radio/sx1276.c:330` | Never written; the POR 0x12 is used implicitly. Not allowlisted, no CFG key. Private-vs-public network separation cannot be changed on the L072. |
| RegLna 0x0C | `firmware/murata_l072/radio/sx1276.c:330` | Never written. LNA boost is never enabled, and manual gain is moot because AgcAutoOn is forced set. |
| RegOcp 0x0B | `firmware/murata_l072/radio/sx1276.c:330` | PA current limit stays at POR while the firmware drives PA_BOOST. Not adjustable. |
| RegPaRamp 0x0A | `firmware/murata_l072/radio/sx1276.c:330` | PA ramp shaping stays at POR. |
| RegPaDac 0x4D | `firmware/murata_l072/radio/sx1276.c:330` | +20 dBm boost is never enabled — consistent with the 17 dBm clamp, but it also means 17 dBm is a hard ceiling. |
| RegPll 0x70 | `firmware/murata_l072/radio/sx1276_tx.c:38` | The comment asserts a "PllBandwidth default (75 kHz)" to justify the PLL settle budget, but **no code reads or writes 0x70** — the claim is unverified in-tree. |
| RegMaxPayloadLength 0x23 / RegHopPeriod 0x24 | `firmware/murata_l072/radio/sx1276.c:330` | Never written or read. FHSS hopping is done in software by retuning RegFrf; the SX1276 hardware hopper is unused. |
| `sx1276_apply_profile_full()` / `sx1276_profile_t` | `firmware/murata_l072/radio/sx1276.c:476` | The only place a full PHY tuple is expressed as one unit, and **nothing uses it** — only comments reference it. |
| `sx1276_reg_dump()` | `firmware/murata_l072/radio/sx1276.c:497` | No caller anywhere. You cannot get a bulk register dump from the running firmware; use REG_READ_REQ per address. |
| `sx1276_airtime_get_budget_us()` | `firmware/murata_l072/radio/sx1276_airtime.c:39` | Exported accessor with no firmware or bench caller — the live QoS budget is not observable. |
| SX1276_AIRTIME_DWELL_WINDOW_MS | `firmware/murata_l072/include/sx1276_airtime.h:25` | Zero references. Editing it does not move any window. |
| SX1276_AIRTIME_DWELL_GUARD_US | `firmware/murata_l072/include/sx1276_airtime.h:26` | Zero references. **CAP_US is an independent literal**, so editing the guard does not move the 380 ms cap. |
| SX1276_RX_SCAN_GOAL_MS | `firmware/murata_l072/include/sx1276_rx_scan_policy.h:84` | Documented as tracked-not-enforced; grep confirms zero references. The A6c-3 overrun counter was never wired, so cold-start reacquire time is unmeasured. |
| 20 dB OBW static assert | `firmware/murata_l072/radio/sx1276_fhss_chantab.c:23` | Commented-out placeholder. **The 500 kHz channel spacing has never been checked against a measured occupied bandwidth** (TODO FCC-EVID-D2). |
| per-region conditional | `firmware/murata_l072/include/host_cfg_keys.h:43` | There is no region abstraction. Anyone expecting an EU/AU build path from the L072 will not find one. |
| HOST_TXQ_P0_RESERVED | `firmware/murata_l072/config.h:58` | No reference outside config.h. The TX ring is pure FIFO — **P0 control traffic has no reserved slot in firmware**; priority is entirely host-side policy. |
| HOST_DEBUG_OPMODE_GUARD | `firmware/murata_l072/config.h:76` | Compiled out at 0, so an OpMode readback mismatch is accepted rather than faulted. |
| LORA_FW_AB_SLOTS | `firmware/murata_l072/config.h:21` | Documented 0/1 but not enforced anywhere; there is no A/B slot layout. |
| FINAL_RELEASE_BUILD | `firmware/murata_l072/include/dev_build_policy.h:10` | Zero references — there is no dev-vs-release build distinction. |
| POWER_CYCLE_AVOIDANCE_DEFAULT | `firmware/murata_l072/include/dev_build_policy.h:18` | Zero references. |
| IWDG_BOOT_WINDOW_MS | `firmware/murata_l072/config.h:108` | **There is no IWDG driver at all.** No watchdog is started or kicked; a wedged TX is not bounded by a watchdog. |
| CFG_KEY_IWDG_WINDOW_MS (0x0D) | `firmware/murata_l072/host/host_cfg.c:120` | Validated and stored, never applied — same consequence as above. |
| CFG_KEY_TX_POWER_ADAPT_ENABLE (0x02) | `firmware/murata_l072/host/host_cfg.c:94` | Stored; no module reads it. Per-frame TX-power adaptation does not exist. |
| CFG_KEY_FHSS_ENABLE (0x05) | `firmware/murata_l072/host/host_cfg.c:100` | Stored; never read. **Hopping is enabled purely by which profile is active** — setting this to 0 does not stop hopping under p1. |
| CFG_KEY_DEEP_SLEEP_ENABLE (0x08) | `firmware/murata_l072/host/host_cfg.c:109` | Apply fn always returns APPLY_FAILED; no deep-sleep scheduler exists. |
| CFG_KEY_BEACON_ENABLE (0x09) | `firmware/murata_l072/host/host_cfg.c:111` | Stored; no beacon implementation exists anywhere in radio/, host/, hal/, boot/ or main.c. |
| CFG_KEY_BEACON_CHANNEL_IDX (0x0A) | `firmware/murata_l072/host/host_cfg.c:30` | No consumer, and its 0..7 range is a leftover from the 8-channel baseline. |
| CFG_KEY_HOST_BAUD (0x0B) | `firmware/murata_l072/host/host_cfg.c:115` | Accepted and answered DEFERRED, but **nothing ever re-inits the UART** — `main.c:52` always uses HOST_BAUD_DEFAULT. Changing the baud over the wire silently does nothing, even after a reboot. |
| CFG_KEY_REPLAY_WINDOW (0x0C) | `firmware/murata_l072/host/host_cfg.c:118` | The L072 does no replay checking. Note the Python side uses 64 bits while this defaults to 16 — neither is enforced here. |
| CFG_KEY_CRYPTO_IN_L072 (0x0E) | `firmware/murata_l072/host/host_cfg.c:122` | Apply always fails; Crypto Profile B is unimplemented. |
| CFG_KEY_FHSS_DWELL_MS (0x13) | `firmware/murata_l072/host/host_cfg.c:29` | Range-checked and stored; no consumer. Real dwell comes from SX1276_FHSS_SLOT_MS and SX1276_DWELL_DEFAULT_CAP_US. |
| LPUART1_CLOCK_HZ | `firmware/murata_l072/host/host_uart.c:14` | Referenced by zero lines — both BRR calculators call `platform_core_hz()`. It also disagrees with the live 32 MHz HSE case, so it is actively misleading. |
| MM_RAM_HEADROOM_MIN | `firmware/murata_l072/include/memory_map.h:98` | No C code, `_Static_assert`, or ld ASSERT evaluates it. There is no "firmware grew too much" warning. |
| BOOT_REGION_SIZE / SLOT_A_SIZE / CONFIG_REGION_SIZE / L072_PAGE_SIZE | `firmware/murata_l072/include/flash_map.h:12-22` | `flash_map.h` is included only by an orphaned linker script the Makefile never builds, so even its own `_Static_assert` never compiles. All four values contradict the live `memory_map.h`. |
| LORA_FPD_MIN_INTER_CYCLE_MS | `firmware/murata_l072/include/lora_frames_per_dwell.h:44` | Not referenced by its own TU; asserted only for parity in a bench test. |
| CMD_PLAN_COMMIT (0x10) / CMD_LINK_HINT (0x20) | `base_station/lora_proto.py:35` | Zero references outside their assignment lines (and the equally unused C mirrors). Sending them does nothing. |
| CSMA_DEFAULT_BUSY_DBM / CSMA_DEFAULT_MAX_SKIPS | `base_station/lora_proto.py:646` | `pick_csma_hop()` has no caller in either daemon. There is no host-side CSMA. |
| LIFETRAC_LORA_INTER_CYCLE_S / MIN_LORA_HOST_INTER_CYCLE_S | `firmware/tractor_x8/image_tx_daemon.py:1337`, `:175` | Stored and printed in one log line; **nothing sleeps on it**. Actual pacing comes from AirtimeBudget and the firmware slot clock. Setting it will not slow the TX loop. |
| LEGAL_DWELL_US / DWELL_HEADROOM_PCT (host mirrors) | `firmware/tractor_x8/image_tx_daemon.py:176`, `:178` | Defined and never referenced. The host does not enforce a 400 ms dwell or an 85 % headroom. |
| _env_float() | `base_station/image_rx_daemon.py:131` | Defined but never called — every float env knob in the RX daemon is parsed by hand, so the defensive contract does not apply to them. |
| X8 back-channel opcodes + KISS bytes | `firmware/tractor_x8/camera_service.py:1170` | The reader thread is not started under `LIFETRAC_USE_LORA_BRIDGE=1`. **On the strict path the whole opcode set — including CMD_LINK_PROFILE and CMD_ROI_HINT — is unreachable**, so ROI hints cannot arrive. |
| back-channel reader chunk size | `firmware/tractor_x8/camera_service.py:1393` | Same reason: the KISS state machine never runs in bridge mode. |
| LIFETRAC_TRACTOR_DETECTOR_WEIGHTS | `firmware/tractor_x8/image_pipeline/detect_nanodet.py:64` | Empty means the stub; but `_NanoDetOnnx.infer()` returns `[]` unconditionally, so **the ONNX branch produces no detections either**. |
| _NanoDetOnnx input_size / score_threshold | `firmware/tractor_x8/image_pipeline/detect_nanodet.py:50` | Not env-settable and not reachable — camera_service never constructs a detector. |
| CLASS_TABLE (tractor) | `firmware/tractor_x8/image_pipeline/detect_nanodet.py:27` | No detections are ever produced, so the class table never reaches the wire. |
| LIFETRAC_DETECTOR / LIFETRAC_DETECTOR_WEIGHTS | `base_station/image_pipeline/detect_yolo.py:90`, `:91` | `make_detector()` has **zero callers**; `detect_yolo.py` is never imported anywhere. Setting a backend or weights path has no effect. |
| MAX_FPS_DEFAULT | `base_station/image_pipeline/detect_yolo.py:29` | `DetectorWorker` is never constructed; the 1.0 fps inference cap paces nothing. |
| NanoDet input_size / score_threshold (base) | `base_station/image_pipeline/detect_yolo.py:56` | Unreachable for the same reason. |
| cross_check thresholds | `base_station/image_pipeline/detect_yolo.py:111` | `cross_check()` has zero callers. **There is no two-detector safety cross-check running.** `state_publisher.py` defines its own SafetyVerdict and never imports detect_yolo. |
| LIFETRAC_SUPERRES_WEIGHTS | `base_station/image_pipeline/superres_cpu.py:52` | `superres_cpu` is imported by nothing; `make_engine()` has no callers. Supplying weights does not enable super-resolution. |
| LIFETRAC_TX_POWER_ADAPTER_V3 | `base_station/image_pipeline/state_publisher.py:55` | Referenced only inside a docstring. It is consumed by `lora_bridge/tx_power_adapter_v3`, not by anything in image_pipeline / web_ui / image_rx_daemon. |
| capture.py DEFAULT_WIDTH / HEIGHT / FPS | `firmware/tractor_x8/image_pipeline/capture.py:52` | camera_service does not use this module. Note DEFAULT_FPS 10 disagrees with the live LIFETRAC_CAMERA_FPS default of 2.0. |
| CaptureRing depth / join_timeout_s | `firmware/tractor_x8/image_pipeline/capture.py:224`, `:248` | Standalone capture pipeline only. (There is **no** identifier named `FrameRing` anywhere in the tree.) |
| tile_diff.DEFAULT_THRESHOLD / PHASH_SIDE / grid defaults | `firmware/tractor_x8/image_pipeline/tile_diff.py:38`, `:89` | camera_service implements its own diff inline (byte-diff or L1 magnitude) and never imports this module. |
| encode_motion.SEARCH_RANGE | `firmware/tractor_x8/image_pipeline/encode_motion.py:33` | Unreachable — `motion_only` is implemented as a WebP quality cap, not real motion estimation. |
| LIFETRAC_ENABLE_DEV_RADIO | `MASTER_PLAN.md:298` | Documented as a hard safety policy ("never enabled on a tractor leaving the bench") but **grep finds no `#ifdef` or env read anywhere — the policy is unenforced**. |
| LIFETRAC_RADIO_DISABLE | `HIL_RUNBOOK.md:159` | The documented RF-silent HIL procedure cannot be followed as written; no such define exists. |
| LIFETRAC_REGION_EU | `KEY_ROTATION.md:94` | Stale troubleshooting entry pointing at a nonexistent setting. The real region knob is `comm.lora_region`. |
| LIFETRAC_ROLE | `HIL_RUNBOOK.md:269` | Unimplemented. Role selection is `PING_ROLE` plus separate sketch targets. |

### 10.2 Deprecated

| Setting | Where | Note |
| --- | --- | --- |
| LORA_FW_QUALITY_AWARE_FHSS | `firmware/murata_l072/config.h:27` | Seeds CFG key 0x06, whose apply handler rejects every set — the feature cannot be turned on. |
| CFG_KEY_FHSS_QUALITY_AWARE (0x06) | `firmware/murata_l072/host/host_cfg.c:102` | Every set returns CFG_STATUS_APPLY_FAILED and the stored value rolls back. |
| HOST_STATS_LEGACY_OFFSET_RADIO_STATE / _PAYLOAD_LEN | `firmware/tractor_h7/murata_host/mh_wire.h:63` | Back-compat offsets for the pre-expansion 64-byte STATS payload; parsers must choose between 68 and 64. |
| LIFETRAC_BASE_SETTINGS | `base_station/settings_store.py:28` | Legacy alias for LIFETRAC_SETTINGS_PATH; still honoured as a fallback. |
| RESEARCH-CONTROLLER MQTT credentials | `RESEARCH-CONTROLLER/ros2_bridge/lifetrac_mqtt_bridge/config/mqtt_bridge_params.yaml:11` | Archived branch, not on the v25 path — but a plaintext broker password and hardcoded LAN IP are committed in the tree. |
| RESEARCH web controller safety knobs | `RESEARCH-CONTROLLER/raspberry_pi_web_controller/config/config.example.yaml:32` | Superseded control cadence and command-loss auto-stop. |
| lifetrac-web-controller.service | `RESEARCH-CONTROLLER/raspberry_pi_web_controller/lifetrac-web-controller.service` | A fifth systemd unit; confirm it is not deployed. |

### 10.3 Diagnostic-only — reachable, but not on the shipped path

| Setting | Where | Why it is not live |
| --- | --- | --- |
| RegInvertIQ 0x33 / RegInvertIQ2 0x3B | `firmware/murata_l072/radio/sx1276.c:281` | Never written by firmware; a 2026-05-25 change writing 0x26/0x1D was explicitly reverted. Both remain host-writable for diagnosis. |
| TCXO drift assumption (±2 ppm) | `firmware/murata_l072/include/sx1276_fhss_clock.h:18` | Narrative only — not encoded as a constant anywhere. |
| abs_slot u32 wrap horizon (~27 y) | `firmware/murata_l072/include/sx1276_fhss_clock.h:29` | Documented bound; no runtime check. |
| `sx1276_legal_dwell_reset()` | `firmware/murata_l072/radio/sx1276_legal_dwell.c:97` | Invoked only from bench TUs, so **booked dwell survives a profile change on-target**. |
| LORA_FPD_LEGAL_DWELL_US / DWELL_HEADROOM_PCT / MAX_FRAMES_PER_DWELL_CAP / `lora_frames_per_dwell()` | `firmware/murata_l072/include/lora_frames_per_dwell.h:45`, `radio/lora_frames_per_dwell.c:9` | Advisory pacing hint with no firmware caller; the whole TU is bench-only. |
| MAX_FRAMES_PER_DWELL_CAP (host mirror) | `firmware/tractor_x8/image_tx_daemon.py:177` | Referenced only by the "TX worker ready" log line; constrains nothing. |
| bench mirror tables (SF/BW/CR, ToA vectors, chantab vectors, slot-clock pins) | `firmware/murata_l072/bench/host_proto/airtime_invariant.c:56`, `:168`, `fhss_chantab_vectors.c:37`, `fhss_clock_test.c:100` | Golden pins that fail the host check build on drift; they do not run on target. |
| lora_ping.c standalone PHY | `firmware/murata_l072/lora_ping.c:315` | Separate bring-up binary, not linked into `firmware.bin`. Its BW is 125 kHz, unlike production 250. |
| RADIO_MINIMIZER_DEFAULT / RADIO_ACTIVE_TEST_DEFAULT / PING_ROLE | `firmware/murata_l072/include/dev_build_policy.h:14`, `:26`, `lora_ping.c:25` | Affect the ping images only; `dev_build_policy.h` is included solely by lora_ping.c. |
| HOST_EMIT_RADIO_IRQ_DEBUG_URC / HOST_TYPE_RADIO_IRQ_URC | `firmware/murata_l072/config.h:66`, `include/host_types.h:57` | Off by default since RS-3.7; production builds never emit the URC. |
| HOST_UART_TX_MIRROR_USART1 | `firmware/murata_l072/config.h:74` | Doubles blocking busy-wait per URC; off in production. |
| HOST_UART_RX_ECHO_DIAG | `firmware/murata_l072/host/host_uart.c:19` | Stage-1 ingress echo; off in production. |
| LIFETRAC_BENCH_BOOT_HEARTBEAT_ENABLE | `firmware/murata_l072/config.h:86` | ASCII boot markers; off in production. |
| CFG_KEY_CFG_DIRTY (0x82) | `firmware/murata_l072/host/host_cfg.c:140` | Observability latch only; never cleared except by `cfg_init()`. |
| AT shell command set | `firmware/murata_l072/host/host_cmd.c:839` | Read-only identity/version/radio/stats queries; **no AT command changes configuration**. |
| CMD_OP_PROBE / CMD_OP_PROBE_ECHO | `base_station/lora_proto.py:969`, `:970` | RS-0.12 delivery instrumentation, gated by LIFETRAC_REACTIVE_FIRE. |
| LIFETRAC_KF_REQUEST_DISABLE | `base_station/image_rx_daemon.py:223` | RS-4.14 experiment: suppresses self-heal keyframe requests. |
| LIFETRAC_REACTIVE_FIRE / PROBE_PHASE_SWEEP_MS / PROBE_MIN_GAP_S / PROBE_SIZES_B / probe floors and caps | `base_station/image_rx_daemon.py:290`, `:298`, `:311`, `:325`, `:333`, `:1276` | Probe sweep instrumentation only. |
| air-gap discard threshold / histogram edges | `base_station/image_rx_daemon.py:1117`, `:1328` | RS-2.3 forensics. |
| _stats_worker tick / batching log cadence | `firmware/tractor_x8/image_tx_daemon.py:1228`, `:614` | Observability pacing. |
| RADIO_PROFILE_ACK_TOPIC (tractor) | `firmware/tractor_x8/image_tx_daemon.py:290` | Retained local-only log; never radiated. |
| LIFETRAC_SYNTHETIC_MODE | `firmware/tractor_x8/camera_service.py:174` | Synthetic camera only. |
| LIFETRAC_CAMERA_HEALTH_LOG / _EVERY_S | `firmware/tractor_x8/camera_service.py:1506`, `:1507` | Frozen-sensor detector logging. |
| LIFETRAC_FRAGMENT_BUDGET_DEMO / method_compare env writes / dry_run_w2_05 gates | `firmware/tractor_x8/method_compare.py:156`, `:47`, `dry_run_w2_05.py:144` | Comparison and smoke harnesses. `LIFETRAC_FRAGMENT_BUDGET_DEMO` is the one **unguarded** `os.environ` parse left in the tractor tree — a non-numeric value aborts the harness. |
| _AUDIT_INTERVAL_S (detect, superres) | `base_station/image_pipeline/detect.py:25`, `superres.py:25` | Debounce on audit entries for Phase-1 stubs that return `[]` / pass through. |
| LIFETRAC_ALLOW_CMD_IMAGE_FRAME | `base_station/web_ui.py:964` | Bench-only legacy ingest; off so a local publisher cannot override the radio path. |
| BenchRadioBody.mode / bench radio subprocess limits | `base_station/web_ui.py:135`, `:1543` | Bench sleep/wake helper plumbing. Note the **gate itself (`LIFETRAC_ENABLE_BENCH_RADIO`) is live and defaults ON**. |
| LIFETRAC_MH_SOFT_BRIDGE / _HEARTBEAT_MS / MH_BENCH_LOG / MH_BRIDGE_DEBUG_SERIAL | `firmware/tractor_h7/tractor_h7.ino:29`, `:54` | H7 bridge and logging diagnostics. |
| LIFETRAC_FORCE_MBEDTLS / _FORCE_RWEATHER_CRYPTO | `firmware/bench/crypto_vectors/host_check.c:30` | Backend forcing for the crypto vector harness. |
| LIFETRAC_SHARED_RADIO_MAGIC + counter slots | `firmware/common/shared_mem.h:57` | M4-visible bench radio counters. |
| LIFETRAC_BOARD_MKR_WAN1310 / _PORTENTA_MAX | `arduino_libraries.txt:32` | Board selector for the retune bench sketch. |
| LIFETRAC_L072_SERIAL / _ENABLE_BOOT_PINS / _JTAG_SWD_ISO_LEVEL | `firmware/portenta_m7_l072_passthrough_ping/portenta_m7_l072_passthrough_ping.ino:43`, `firmware/x8_uart_route_probe/x8_uart_route_probe.ino:16` | Bring-up probe sketches. |
| bench_mqtt.conf / owner_net_profiles.json | `firmware/x8_lora_bootloader_helper/bench_mqtt.conf:8`, `owner_net_profiles.json` | Bench broker config and replay-profile manifest. |
| RSSI-sniff, rx_pair_nrst, radio_state_dump, rx_ver_warmup_sweep, radio_sleep/wake, rom-baseline, sniff_jlink_cdc, diagnose_x8_recovery, probe_uvc, probe_v4l2, check_camera params | see §1, §3, §4, §9 rows | Harness-only parameters; none reach a production code path. |

### 10.4 Unknown

| Setting | Where | Note |
| --- | --- | --- |
| LIFETRAC_TRUSTED_PROXIES | `docker-compose.yml:67` | Declared in compose for uvicorn `--proxy-headers`, but no consumer was confirmed. Treat audit-log client IPs behind a proxy as **unverified**. |

---

## 11. What we have actually tried on the bench

Source: **70 run directories** under `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/radio_monitor_*`,
each with a `params.txt` — 69 auto-written by `-Archive` plus one hand-written stub
(`radio_monitor_20260729_C1_unarchived_4d33b499`, whose NOTE records that `-Archive` was omitted and only
the raw console transcript survives; it has no tx_serial/rx_serial/host_ip/timestamp and omits
tx_batch, tx_prepare_ahead, train_gap_ms, parity_group, tx_feed and the probe sweeps).

Date range **2026-07-24 15:34:49 → 2026-07-29 10:53:14** (plus the undated C1 stub, same day).
Per day: 07-24 ×26, 07-25 ×14, 07-26 ×17, 07-29 ×13.
**Gap: zero runs on 07-27 and 07-28**, even though the RS-0.13b / RS-0.12 instrumentation
(AlignedPump, ReactiveFire, ProbePhaseSweepMs, ProbeSizesB) was committed on 07-27 — it was first flown on 07-29.
22 distinct git SHAs; top: `c522dcc3` ×13, `d2630395` ×12, `616ede2b` ×6, `6a6d2684` ×5, `e43d07cf` ×5, `4d33b499` ×4.

### 11.1 Value matrix — `run_live_radio_monitor.ps1`

| Knob | Coded default | Values actually flown | Chronology / notes |
| --- | --- | --- | --- |
| TxPipeline | `"v3"` (was `"v2"` from `c522dcc3` until `8cfb21f2`) | v3 ×47, v2 ×23 | v2 ×13 (07-24 15:34-17:06) → v3 ×1 → v2 ×6 → v3 ×2 → v2 ×1 → **v3 ×34** (07-24 21:55 - 07-26 17:46) → v2 ×2 → v3 ×1 → v2 ×1 → v3 ×9. **The only literal default flip in the whole param block's history.** |
| DurationS | `30` | 120 ×35, 60 ×14, 240 ×6, 90 ×3, 180 ×3, 150 ×2, 300 ×2, 1800 ×2, 420 ×1, 600 ×1, 900 ×1 | **The coded default 30 was never used in any archived run.** 60 (07-24 pm) → 90 → 120 (dominant) → 300 → two 1800 s soaks + 900 (07-26 pm) → 180/150/600/420 → 240 ×6 (07-29 ack experiments). |
| SynthFps | `2` | 6 ×30, 20 ×28, 2 ×11, 12 ×1 | 6 ×28 (07-24 15:34 - 07-25 03:19) → 12 ×1 → 6 ×2 → **20 ×25** (07-25 16:54 - 07-26 17:11) → 2 ×3 (paired with budget 3000) → 20 ×3 → 2 ×8. The comment above the param argues 6 fps is ~3× saturation; **comment and coded default disagree.** |
| SynthBudgetB | `250` | 250 ×59, 3000 ×11 | Only two values, moving in lockstep with SynthFps: (6/12/20 fps, 250 B) = small-frame saturation; (2 fps, 3000 B) = big-frame regime from 07-26 17:46, dominant on 07-29. |
| TxPipelineDepth | `2` | 2 ×38, 4 ×1, knob absent ×31 | The single depth-4 run is `20260725_202455_d2630395`, at reg_profile 2 (DTS) — consistent with the "<=3 at FHSS, 4 at DTS" comment. |
| TxBatch | `1` | 1 ×31, 0 ×1, absent ×38 | The single 0 run is `20260726_124732_c09fcfc4` at reg_profile 1, immediately after the 1 run at 12:44:11 — a deliberate back-to-back A/B. |
| TxPrepareAhead | `1` | 1 ×15, 0 ×7, absent ×48 | 1 ×12 (07-26 12:50 - 07-29 09:25) → 0 ×7 (the RS-0.13b bisection block, always paired with TrainGapMs 0) → 1 ×3 (from 07-29 10:38, paired with TrainGapMs 40). |
| TrainGapMs | `40` | 40 ×14, 0 ×7, absent ×49 | Moves in **perfect lockstep** with TxPrepareAhead (40↔1, 0↔0) in every run — the two were never varied independently. |
| ParityGroup | `0` | 4 ×7, 0 ×10, absent ×53 | Only ever 0 or 4 — never 2, 3 or 8. 4 ×7 (07-26 13:53 - 07-29 09:25, incl. both 1800 s soaks) → 0 ×10 (all of 07-29 from 09:30). |
| KfRequestDisable | `0` | 1 ×14, 0 ×8, absent ×48 | Most-toggled diagnostic: 1 ×2 → 0 ×3 → 1 ×1 → 0 ×3 → **1 ×11** (07-29 09:17-10:48) → 0 ×2. |
| RegProfile | `0` | 2 ×31, 1 ×8, not logged ×31 | 2 ×14 (07-25 16:54 - 07-26 12:41) → 1 ×2 → 2 ×1 → 1 ×6 → **2 ×16** (07-26 16:31 on). The default 0 was never explicitly recorded. |
| TxFeed | `"local"` | local ×38, absent ×32 | **The `host` path has never been exercised since the knob was added.** The 31 pre-knob runs used the then-hardcoded host wiring. |
| AlignedPump | `1` | 1 ×13, absent ×57 | **Never set to 0 in any archived run** — despite the comment describing it as a bisection toggle, the 0 leg was never flown. |
| ReactiveFire | `0` | 1 ×9, 0 ×4, absent ×57 | All 13 uses are 2026-07-29: 1 ×9 (09:17-10:16) then 0 ×4 (10:38 on, once the ack-cost experiment moved to ProbeEcho 0). |
| ProbePhaseSweepMs | `""` | empty ×11, `"0,20,40,60,80,100,120"` ×1, absent ×58 | Exercised on air exactly **once**: `20260729_094545_e43d07cf` (600 s, v3, DTS). |
| ProbeSizesB | `""` | empty ×11, `"23,38"` ×1, absent ×58 | Same single run as the phase sweep. |
| AckCopies | `2` | 2 ×5, 1 ×1, absent ×64 | Newest knob (2026-07-29, `4d33b499`). The single AckCopies=1 run is `20260729_101137_b319f0cb`. |
| ProbeEcho | `1` | 0 ×5, 1 ×1, absent ×64 | Added with AckCopies; default used once (10:11:37), 0 for the remaining 5 — the "→ 91 % delivery" experiment. |
| Archive | switch off | on ×69, off ×1 | The one off run is `20260729_C1_unarchived_4d33b499`. |
| TxAdbSerial / RxAdbSerial / HostIp | `2E2C1209DABC240B` / `2D0A1209DABC240B` / `192.168.1.79` | never overridden ×69 (1 run unrecorded) | Fixed bench identities across the whole history. |

### 11.2 params.txt schema drift — absent ≠ zero

The archive key list grew in lockstep with the param block (verified commit by commit), so **a missing key
means the knob did not exist in that script version, not that it was zero**
(`firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1:308-332`).

| Milestone commit | Keys added |
| --- | --- |
| `c522dcc3` (07-24) | 9 keys — the initial set |
| `346aa414` | + tx_pipeline_depth, tx_batch, tx_feed, reg_profile |
| `b5527e3e` | + tx_prepare_ahead |
| `9a633f1c` | + train_gap_ms, kf_request_disable |
| `54f11e86` | + parity_group |
| `64fe6ebc` | + aligned_pump, reactive_fire, probe_phase_sweep_ms |
| `1487b762` | + probe_sizes_b |
| `4d33b499` | + ack_copies, probe_echo (22 keys total) |

**One exception:** `RegProfile` was a live param from `212bc44e` (07-24) but was not archived until
`346aa414`, so **13 runs ran with an unlogged reg_profile** (SHAs `212bc44e` ×2, `616ede2b` ×6,
`6bb93912` ×3, `3b1fb347` ×2) whose value can only be assumed to be the then-default 0. The 18 runs before
that (`c522dcc3` ×13, `6a6d2684` ×5) had no profile knob at all.
Also note `git_sha` records repo HEAD, not the script version — the 07-25 `d2630395` runs already carry
`tx_feed` / `tx_pipeline_depth` from working-tree edits later committed as `346aa414`.

### 11.3 Default-flip audit

All 13 commits touching the param block were diffed (`24c9d78b`, `c522dcc3`, `6a6d2684`, `212bc44e`,
`faed3f02`, `346aa414`, `b5527e3e`, `9a633f1c`, `54f11e86`, `64fe6ebc`, `1487b762`, `8cfb21f2`, `4d33b499`).

- **Exactly one default value ever changed after introduction:** `TxPipeline` `"v2"` → `"v3"` at `8cfb21f2` (2026-07-29).
- Every other knob kept its introduction-time default for its whole life.
- **Two silent behaviour flips** that are not literal default changes but shifted the code path under test:
  1. `TxFeed` introduced at `346aa414` defaulting to `"local"`, moving the frame feed from the bench-PC LAN
     broker to a tractor-loopback broker for every subsequent run.
  2. `RegProfile` introduced at `212bc44e` defaulting to 0 — runs before it had no regulatory-profile control at all.

**Reproducibility gaps to close:** container image tags are hardcoded `latest` and never recorded in
params.txt; `LIFETRAC_SYNTH_PREBUILD`, `LIFETRAC_FHSS_WIDE_MASK` and the derived broker hosts are never
recorded either.

---

## 12. Settings that are NOT settable

Parameters that are hardcoded today and have no CFG key, no host command, and no API. Several are on the
roadmap to be exposed; all of them are things an engineer will reach for at the bench and not find.

| Parameter | Current fixed value | Where | Why it is stuck | Exposure path if needed |
| --- | --- | --- | --- | --- |
| **Preamble length** | chip POR = 8 symbols | `firmware/murata_l072/radio/sx1276_airtime.c:171` (read-back only) | 0x20/0x21 are never written, are not in the REG_WRITE allowlist, and no CFG key or API touches them. The airtime invariant *assumes* 8. | Add 0x20/0x21 to the allowlist **and** feed the value into `sx1276_airtime_compute` — changing one without the other breaks the ToA prediction the dwell accountant reconciles against. |
| Sync word | POR `0x12` | `firmware/murata_l072/radio/sx1276.c:330` | Not allowlisted, no CFG key. Only `lora_ping.c:341` writes it, and that binary is not shipped. | Allowlist 0x39, or write it in `sx1276_init()`. Must match the H7 peer at `tractor_h7.ino:1853`. |
| Spreading factor / bandwidth / coding rate | 7 / 250 kHz / 4-5 | `firmware/murata_l072/radio/sx1276.c:452` | No CFG key exists. Only `sx1276_set_sf_bw_cr()` (internal) or raw REG_WRITE 0x1D/0x1E. | `CFG_KEY_REG_PROFILE` indirectly selects 250 vs 500 kHz; anything else needs a new key routed through the airtime invariant. |
| Header mode (implicit) | explicit, hardcoded `or 0U` | `firmware/murata_l072/radio/sx1276.c:451` | No API can set implicit mode. | Raw REG_WRITE 0x1D only, which bypasses the invariant. |
| ~~FHSS seed (farm_id, node_id, epoch)~~ | farm/node settable; epoch stays 0 by design | `CFG_KEY_FHSS_FARM_ID` 0x17 / `CFG_KEY_FHSS_NODE_ID` 0x18 | **FIXED 2026-07-29.** Two new u64 keys ride `host_cfg_profile_req_t` exactly as `channel_mask` does; `activate()` forwards them. Only **two** keys, not three: epoch is a runtime time coordinate the first FHSS TX overwrites, so it is not identity. Defaults 0/0 reproduce the historical permutation bit-for-bit. | — |
| Hop table geometry (count, spacing, first centre) | 50, 500 kHz, 902.75 MHz | `firmware/murata_l072/include/sx1276_fhss_chantab.h:32-34` | Compile-time, pinned by `_Static_assert`, and must stay bit-identical to `bench/host_proto/fhss_chantab.py`. | Compile-time only, with a matching Python change and re-run of the parity vectors. |
| Slot-clock geometry (slot ms, guard, headstart) | 200 ms / 15000 µs / 12 ms | `firmware/murata_l072/include/sx1276_fhss_clock.h:52`, `:58`, `:68` | Compile-time; pinned by the bench geometry check. | Compile-time; changing SLOT_MS changes the epoch period and therefore the dwell arithmetic. |
| Legal-dwell cap and window | 400000 µs / 10000 ms | `firmware/murata_l072/include/sx1276_legal_dwell.h:70-72` | Passed as literals at the single call site `sx1276_tx.c:370`. | Compile-time. |
| ~~Conducted TX power derived from antenna gain~~ | **now applied** | `firmware/murata_l072/host/host_cfg.c` | **FIXED 2026-07-29.** Implemented in `host_cfg.c` rather than `host_cfg_profile.c` — the latter is a pure, heavily-unit-tested module and calling `cfg_get` from it would create mutual recursion between translation units. | — |
| LNA gain / LNA boost / OCP / PA ramp / PA_DAC | chip POR | `firmware/murata_l072/radio/sx1276.c:330` | None are allowlisted; AgcAutoOn is forced on so manual LNA gain is inert anyway. | Allowlist the addresses, or write them in `sx1276_init()`. |
| IRQ mask (RegIrqFlagsMask) | POR, all unmasked | `firmware/murata_l072/radio/sx1276.c:330` | Not allowlisted; the firmware relies on DIO EXTI edges. | Allowlist 0x11. |
| TCXO enable (RegTcxo 0x4B) | RMW `read or 0x10` | `firmware/murata_l072/radio/sx1276_modes.c:236` | Not in the REG_WRITE allowlist — deliberately, since clearing it makes all OpMode transitions silently fail. | Leave alone. |
| DIO4/DIO5 mapping (RegDioMapping2 0x41) | `0x00` in all six modes | `firmware/murata_l072/radio/sx1276_modes.c:90` | Not allowlisted; the mode table column is all-zero. | Add a column to `k_mode_descs`. |
| Region / locale | none — US FCC 15.247 only | `firmware/murata_l072/include/host_cfg_keys.h:43` | No region abstraction exists in the L072 tree; `comm.lora_region` in the build config never reaches it. | Would require a per-region channel table, dwell policy and tier ceilings. |
| Host UART baud at runtime | 921600 | `firmware/murata_l072/config.h:47` | CFG key 0x0B is accepted and answered DEFERRED, but nothing re-inits the UART even after reboot. | Persist the CFG value to Flash and read it in `main.c:52`. |
| Watchdog window | none — no IWDG driver | `firmware/murata_l072/config.h:109` | Both IWDG constants and CFG key 0x0D exist; no code starts or kicks a watchdog. | Implement the IWDG driver (N-20). |

