# LoRa Protocol Reliability (Phase 3) & Codec Efficiency (Phase 4) — Implementation & Verification Report

**Date:** 2026-07-23  
**Author:** GitHub Copilot (Gemini 3.6 Flash)  
**Version:** v1.0  
**Scope:** Implementation and verification of Phase 3 (Protocol Reliability, XOR Parity Reconstruction, Adaptive Sizing Signal) and Phase 4 (Codec Efficiency, Container Stripping `CODEC_WEBP_RAWSTREAM`, Keyframe Quality Tuning) across `LifeTrac-v25/DESIGN-CONTROLLER/`.  
**Git Branch:** `main`  

---

## 1. Executive Summary

This document reports the completion of **Phase 3 (Protocol Reliability)** and **Phase 4 (Codec Efficiency)** as specified in the master review ([`2026-07-23_Phase2_Review_and_Phase3_4_Plan_Suggestions_Copilot_v1_0.md`](./2026-07-23_Phase2_Review_and_Phase3_4_Plan_Suggestions_Copilot_v1_0.md)).

All features have been implemented, tested with 100% pass rates across Python unit test suites and C firmware verification targets, deployed, and verified on physical Portenta X8 MAX carrier hardware (`2E2C1209DABC240B` and `2D0A1209DABC240B`).

---

## 2. Implemented Features & Architecture Details

### 2.1 Phase 3 — Protocol Reliability & Parity Reconstruction (`0xFC`)
- **XOR Parity Headers & Wire Contract:**
  - Added `TELEMETRY_FRAGMENT_MAGIC_PARITY = 0xFC` and `TELEMETRY_FRAGMENT_HEADER_LEN_PARITY = 4`.
  - Header layout: `u8 magic (0xFC) | u8 frag_seq | u8 group_start | u8 group_len | parity_payload`.
- **RX-Side Parity Reconstruction (`FragmentReassembler`)**:
  - `FragmentReassembler.feed()` in [`reassemble.py`](../../base_station/image_pipeline/reassemble.py) parses `0xFC` parity fragments, storing them in `partial.parities[(group_start, group_len)]`.
  - Implemented `_try_parity_reconstruct(partial)`: when exactly one data fragment in a parity group `[group_start, group_start + group_len)` is missing on air, `FragmentReassembler` bitwise-XORs all received data fragment bodies in that group with the parity body, reconstructs the missing data fragment in place, and completes the frame without requiring a retransmission round trip.
  - Added `stats.parity_reconstructions` counter.
- **TX-Side Parity Interleaving**:
  - Implemented `add_parity_fragments(fragments, frag_seq, group_len)` in [`lora_proto.py`](../../base_station/lora_proto.py).
  - Integrated into [`image_tx_daemon.py`](../../firmware/tractor_x8/image_tx_daemon.py) gated behind env knob `LIFETRAC_PARITY_GROUP` (e.g. `LIFETRAC_PARITY_GROUP=8`).
- **Air PER & Adaptive Sizing Feedback**:
  - Updated `recent_frag_loss_rate()` in `image_tx_daemon.py` to calculate loss from abandoned fragments (where all RF retries failed) rather than local QoS refusals.

### 2.2 Phase 4 — Codec Efficiency & Container Stripping
- **Container-Stripped Bitstream Codec (`CODEC_WEBP_RAWSTREAM = 5`)**:
  - Defined `CODEC_WEBP_RAWSTREAM = 5` in [`frame_format.py`](../../base_station/image_pipeline/frame_format.py).
  - Added `ENCODE_MODE_RAWSTREAM = 8` in [`camera_service.py`](../../firmware/tractor_x8/camera_service.py). When `ENCODE_MODE_RAWSTREAM` or `LIFETRAC_CONTAINER_STRIP=1` is active, `_encode_tile` strips the fixed 12 B RIFF header and 8 B VP8/VP8L chunk header from the WebP container output (`blob[20:]`), saving 20 B per tile (~10–15% byte reduction per frame).
- **Base Station Re-wrapping Transcoder**:
  - Implemented `rewrap_webp(raw: bytes) -> bytes` and `_decode_webp_rawstream` in [`codec_decode.py`](../../base_station/image_pipeline/codec_decode.py).
  - Automatically reconstructs standard 20 B RIFF WebP headers from VP8/VP8L rawstreams, allowing standard browsers and PIL image decoders to render tiles pixel-exactly without client-side changes.
- **Keyframe Compression Optimization**:
  - Updated `_encode_tile` in `camera_service.py` to execute WebP `method=6` higher compression effort for keyframes (`is_key=True`).

---

## 3. Verification & Test Suite Results

1. **Python Unit Test Suite (`base_station/tests`)**:
   - `test_image_tx_rx_optimization.py`: **PASS** (includes `test_parity_fragments_reconstruction` proving 1-lost-fragment group reconstruction via `0xFC` parity, and `test_codec_webp_rawstream` proving round-trip bitstream stripping and re-wrapping).
   - `test_phy_golden_vectors.py`: **PASS**
   - `test_telemetry_fragmentation.py`: **PASS**
   - `test_telemetry_fragmentation_fuzz.py`: **PASS**
   - `test_lora_proto.py`: **PASS**
   - `test_x8_encode_mode.py`: **PASS**

2. **C Firmware Test Suite (`mingw32-make check`)**:
   - `check-tx-len-guard` — **PASS**
   - `check-cfg-profile` — **PASS** (27 cases)
   - `check-cfg-profile-wire` — **PASS** (9 cases)
   - `check-airtime-invariant` — **PASS** (12 cases)

3. **Hardware Deployment Verification**:
   - Both Portenta X8 MAX carrier boards (`2E2C1209DABC240B` and `2D0A1209DABC240B`) were deployed with updated daemons and tested live over the air link.

---

## 4. Summary of Files Modified / Created

- `base_station/image_pipeline/frame_format.py`
- `base_station/image_pipeline/codec_decode.py`
- `base_station/image_pipeline/reassemble.py`
- `base_station/lora_proto.py`
- `firmware/tractor_x8/camera_service.py`
- `firmware/tractor_x8/image_tx_daemon.py`
- `base_station/image_rx_daemon.py`
- `base_station/tests/test_image_tx_rx_optimization.py`
- `base_station/tests/test_telemetry_fragmentation.py`
- `base_station/tests/test_telemetry_fragmentation_fuzz.py`
- `base_station/tests/test_lora_proto.py`
- `firmware/murata_l072/Makefile`
- `firmware/murata_l072/host/host_cmd.c`
- `firmware/murata_l072/host/host_cfg_profile.c`
- `firmware/murata_l072/main.c`
- `firmware/murata_l072/radio/sx1276_tx.c`
- `firmware/x8_lora_bootloader_helper/run_live_radio_monitor.ps1`
