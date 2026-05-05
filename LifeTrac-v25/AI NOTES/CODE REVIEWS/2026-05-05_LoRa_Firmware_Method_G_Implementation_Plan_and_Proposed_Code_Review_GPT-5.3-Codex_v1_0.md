# LoRa Firmware Method G Review - Implementation Plan and Proposed Code

Date: 2026-05-05
Reviewer: GitHub Copilot (GPT-5.3-Codex)
Scope: Method G planning docs in DESIGN-LORAFIRMWARE plus proposed LoRa protocol code in firmware/common/lora_proto and firmware/bench/crypto_vectors.

## Executive Verdict

Overall status: AMBER-RED.

The strategic Method G choice is strong, but there are multiple design-level contradictions and a few concrete code defects that should be resolved before treating the plan as execution-ready.

## Findings (Ordered by Severity)

### Critical 1: Flash memory map is internally inconsistent and can exceed STM32L072 flash limits

Evidence:
- Subtotal says Slot A is ~92 KB and "rounds to a 96 KB slot" (DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md:135).
- Slot B is listed as 96 KB (DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md:136).
- A separate 4 KB boot/recovery/config region is also listed (DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md:137).
- Memory map then describes Slot A and Slot B as 92 KB each from 0x08001000 to 0x08030000 (DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md:163,164,165,166), then adds a 4 KB config page starting at 0x08030000 and ending at 0x08030400 while claiming this is end of 192 KB flash (DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md:167,168).

Why this matters:
- 0x08030000 is already the 192 KB flash limit for STM32L072.
- The stated ranges conflict with each other and can lead to linker scripts that overwrite reserved space or run off-device flash.
- This is a direct brick risk during A/B rollout work.

Recommendation:
- Freeze one canonical memory layout with exact byte math and addresses, then derive linker script constants from it.
- Decide whether you want 92+92+4+4 KB or true 96+96 KB slots; both cannot coexist on 192 KB.

### Critical 2: FCC strategy is contradictory (8-channel mandatory vs >=50-channel compliance assumption)

Evidence:
- Scope commits to per-frame FHSS with 8 channels (DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md:37).
- Risks section states compliance via >=50 channels and says v25 currently targets 8 channels (DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md:81).
- Capabilities marks R-01 (8-channel FHSS) as mandatory (DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md:29).
- N-06 then describes >=50-channel weighted hopping to satisfy 15.247 distribution expectations (DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md:74).

Why this matters:
- This is a launch-blocking compliance ambiguity, not a tuning detail.
- Test plans, FHSS tables, and acceptance criteria cannot be finalized until channel-count policy is fixed.

Recommendation:
- Lock a single regulatory baseline per launch region now (for example: 8-channel under one legal path, or >=50-channel FHSS under another), and update all R-IDs and test gates to match.

### High 3: "Method G complete" gate does not match the mandatory capability set

Evidence:
- Completion criterion uses "All R-1..R-7" (DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md:87).
- Capabilities document defines mandatory launch set as R-01 through R-12 (DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md:211).

Why this matters:
- Program can be declared complete while mandatory requirements remain unverified.
- Creates audit and release-governance gaps.

Recommendation:
- Replace the completion gate with explicit IDs and test artifacts for all mandatory R-IDs.

### High 4: Crypto ownership and nonce authority are ambiguous between H7 and L072

Evidence:
- Decision doc says host-side encryption remains default and L072 transmits pre-formed frames (DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md:100).
- Capabilities doc keeps H7-encrypts default (DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md:107) but also makes L072 RNG-derived nonce generation mandatory (DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md:109,211).
- UART API allows TX_FRAME payload to be already encrypted by H7 (DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md:78).

Why this matters:
- Replay windows and AEAD nonce safety require one canonical nonce/crypto owner.
- Ambiguous ownership can create subtle interop or replay-rejection bugs.

Recommendation:
- Choose one launch profile and encode it as a hard contract:
  - Profile A: H7 owns encrypt+nonce+replay, L072 transports ciphertext only.
  - Profile B: L072 owns encrypt+nonce+replay, H7 sends plaintext only.
- Treat the other profile as post-launch, not dual-default.

### High 5: Host UART ABI has no backpressure despite pipelining and unresolved burst budget

Evidence:
- Transport explicitly uses no flow control (DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md:32).
- Host is allowed to pipeline TX without waiting (DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md:33).
- L072 budget currently allocates only 2x256 B RX DMA and 1 KB TX queue (DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md:146,147).
- Maximum safe DMA burst size is still an open question (DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md:221).

Why this matters:
- Under bursty command/load, frame drops and queue overflow can be silent and timing-dependent.
- P0 control determinism can degrade exactly during stress events.

Recommendation:
- Add explicit credit/ready signaling (or strict per-seq pacing) in protocol v1.
- Define and enforce per-type queue limits with drop policy telemetry.

### High 6 (Code): Crypto vector harness passes wrong length to lp_decrypt

Evidence:
- host_check passes ct_len as ciphertext length only (firmware/bench/crypto_vectors/host_check.c:116).
- API contract says ct_len must be ciphertext plus tag total length (firmware/common/lora_proto/lora_proto.h:229,230).

Why this matters:
- The harness can report false failures and does not correctly validate the real decrypt contract.

Recommendation:
- Pass clen + 16 (or clen + LP_TAG_LEN) to lp_decrypt in host_check.

### Medium 7 (Code): host_check prototypes do not match canonical bool signatures

Evidence:
- host_check declares lp_encrypt/lp_decrypt as int-returning (firmware/bench/crypto_vectors/host_check.c:37,39).
- Canonical API defines bool return values (firmware/common/lora_proto/lora_proto.h:237,241).

Why this matters:
- Signature drift increases warning noise and can hide real ABI regressions.

Recommendation:
- Include lora_proto.h directly in host_check and remove local prototypes.

### Medium 8 (Code): lp_csma_pick_hop dereferences callback without null guard

Evidence:
- sample callback is invoked directly with no validation (firmware/common/lora_proto/lora_proto.c:226).

Why this matters:
- Early bring-up or miswired call sites can crash hard instead of failing cleanly.

Recommendation:
- Add null callback guard and deterministic fallback behavior.

### Medium 9 (Process): Proposed code is still explicitly unvalidated

Evidence:
- Firmware tree says not yet compiled/flashed/tested (firmware/README.md:3).
- Core protocol sources are marked draft/not compiled or tested (firmware/common/lora_proto/lora_proto.c:3, firmware/common/lora_proto/lora_proto.h:3).

Why this matters:
- Design confidence currently exceeds implementation evidence.

Recommendation:
- Add minimal CI compile + host-unit sweep before Phase 2 closure.

## Positive Notes

- Method G itself is the right strategic direction for deterministic control latency and long-term BOM stability.
- Recovery architecture (NRST + BOOT0 + safe-mode + golden image) is unusually strong for an embedded radio path.
- The phased roadmap structure is clear and generally execution-friendly once the contradictions above are resolved.

## Suggested Priority Fix Sequence

1. Fix memory map math and linker-region contract.
2. Lock regulatory channel-count policy and cascade to all docs/tests.
3. Align completion gates to mandatory capability IDs.
4. Freeze crypto ownership model for launch profile.
5. Patch host_check length bug and signature drift.
6. Add UART backpressure/credit semantics before high-rate HIL.

## Verification Notes

- This review is static (document/code inspection).
- Runtime validation of host_check was attempted, but make was not available in the current Windows shell environment.
