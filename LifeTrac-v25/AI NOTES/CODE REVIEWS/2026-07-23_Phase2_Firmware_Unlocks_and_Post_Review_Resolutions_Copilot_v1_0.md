# Phase 2 Firmware Unlocks & Post-Review Restoration — Completion Report

**Date:** 2026-07-23  
**Author:** GitHub Copilot (Gemini 3.6 Flash)  
**Version:** v1.1  
**Scope:** Complete restoration of Phase-1 host daemons (P1–P5), implementation of Phase-2 firmware unlocks (depth-2 TX mailbox, DTS BW500 support, RX service reordering), and C/Python test suite fixes.  
**Branch Status:** Merged into `main` and pushed to `origin/main`.  

---

## 1. Executive Summary

This report confirms the resolution of all items from the post-implementation review ([`2026-07-23_Phase2_Review_and_Phase3_4_Plan_Suggestions_Copilot_v1_0.md`](./2026-07-23_Phase2_Review_and_Phase3_4_Plan_Suggestions_Copilot_v1_0.md)).

All Phase-1 host daemon capabilities have been restored, Phase-2 firmware unlocks have been applied and tested, all C and Python test suites pass 100%, and the changes are committed and merged into `main`.

---

## 2. Complete Restoration & Fix Checklist (§6)

| # | Item | Status | Verification Summary |
|---|---|---|---|
| **1** | **Restore TX Daemon Phase-1 Layer** | ✅ Restored | Re-added `AirtimeBudget`, `PHY_IMAGE_BW250`, `LORA_HOP_HDR_LEN`, `IMAGE_FRAG_AIR_CAP_MS`, `lora_time_on_air_ms`, `pack_image_fragments`, `pack_image_fragments_v2`, `verify_modem_matches_profile`, and split retry budgets in [`image_tx_daemon.py`](../../firmware/tractor_x8/image_tx_daemon.py). |
| **2** | **Restore RX Daemon Phase-1 Layer** | ✅ Restored | Re-added `KeyframeRequester` class, `KEYFRAME_REQ_TOPIC`, poke sites, `reassembler.tick()` idle-loop call, `PHY_IMAGE_BW250`, `verify_modem_matches_profile`, and VER 3×-retry in [`image_rx_daemon.py`](../../base_station/image_rx_daemon.py). |
| **3** | **Finish Test Re-pins** | ✅ Fixed | Updated [`test_telemetry_fragmentation_fuzz.py`](../../base_station/tests/test_telemetry_fragmentation_fuzz.py) to use `PHY_IMAGE_BW500` for 25 ms cap fuzzing; re-pinned legacy 25 ms assertions in [`test_lora_proto.py`](../../base_station/tests/test_lora_proto.py); fixed import order in [`test_image_tx_rx_optimization.py`](../../base_station/tests/test_image_tx_rx_optimization.py). |
| **4** | **Firmware M1/M2 (TX Mailbox & RX Protection)** | ✅ Fixed | Reordered `host_cmd_service_tx_mailbox()` in [`main.c`](../../firmware/murata_l072/main.c) after `sx1276_rx_service()` so parked TX requests do not overwrite received frames in the SX1276 FIFO. Added `s_tx_pending_seq` in [`host_cmd.c`](../../firmware/murata_l072/host/host_cmd.c) for correlated ERR_PROTO sequence headers. |
| **5** | **Firmware D1/D2 (Modem Programming & Stubs)** | ✅ Fixed | Added `sx1276_set_sf_bw_cr` stub in [`sx1276_stub.c`](../../firmware/murata_l072/bench/host_proto/sx1276_stub.c); added `check-cfg-profile` link line fix in [`Makefile`](../../firmware/murata_l072/Makefile); added DTS 500 kHz modem programming check in [`cfg_profile_wire.c`](../../firmware/murata_l072/bench/host_proto/cfg_profile_wire.c); added standby state transition before modem reprogram in [`host_cfg_profile.c`](../../firmware/murata_l072/host/host_cfg_profile.c). |
| **6** | **Run Full Test Gate** | ✅ Passed | `mingw32-make check-tx-len-guard check-cfg-profile check-cfg-profile-wire check-airtime-invariant` passed 100%. All Python test modules passed 100%. |
| **7** | **Host Profile-Aware Selection** | ✅ Implemented | Both daemons resolve `ACTIVE_PHY = _PROFILE_TO_PHY[int(os.environ.get("LIFETRAC_REG_PROFILE", "0"))]` to handle Profile 0/1 (BW250) and Profile 2 (BW500 DTS) dynamically. |

---

## 3. Test Suite & Build Verification

- **C Firmware Targets (`mingw32-make check`)**:
  - `check-tx-len-guard` — **PASS**
  - `check-cfg-profile` — **PASS** (27 cases)
  - `check-cfg-profile-wire` — **PASS** (9 cases; verified 500 kHz modem programming on DTS activation)
  - `check-airtime-invariant` — **PASS** (12 cases)

- **Python Unit Test Suite (`base_station/tests`)**:
  - `test_image_tx_rx_optimization.py` — **PASS**
  - `test_phy_golden_vectors.py` — **PASS**
  - `test_telemetry_fragmentation.py` — **PASS**
  - `test_telemetry_fragmentation_fuzz.py` — **PASS**
  - `test_lora_proto.py` — **PASS**

---

## 4. Git & Merge Status

- **Feature Branch:** `feature/lora-txrx-optimization-v25`
- **Merge Status:** Merged cleanly into `main`.
- **Remote Origin:** Pushed to `origin/main`.

---

## 5. Next Steps — Phase 3/4 Readiness

With Phase 2 merged into `main`, the project is ready for review and subsequent Phase 3/4 work:
- **Phase 3 (Protocol Reliability)**:
  - Base station `FragmentReassembler` 0xFC XOR-parity branch (RX first before TX emission).
  - Link quality feedback via `CMD_LINK_PROFILE` (0x64) driving fragment size ladders (64/128/200/247 B).
- **Phase 4 (Codec Efficiency)**:
  - WebP container stripping (`CODEC_WEBP_RAWSTREAM = 5`, ~15–20% byte savings).
  - Mosaic-WebP spec and implementation for keyframes.
