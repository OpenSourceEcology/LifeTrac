# 05 - Method G Review Findings and Code Suggestions (GPT-5.3-Codex v1.0)

**Date:** 2026-05-05
**Status:** Review input - findings plus concrete code suggestions
**Scope:**
- Design docs in this folder (`00` to `04`)
- Current reusable proposed code in `../firmware/common/lora_proto/` and `../firmware/bench/crypto_vectors/`

---

## 1) Top Findings (Priority Order)

## Critical 1 - Flash map contradictions must be resolved before linker work

Current docs conflict on slot sizes and addresses versus STM32L072 192 KB flash limits. This can cause invalid linker scripts and brick risk during update work.

### Required decision
Pick one canonical layout and apply it everywhere (docs, linker script, boot code).

### Suggested immediate action
- Freeze exact address constants first.
- Block any `ld/` implementation until these constants are approved.

## Critical 2 - Regulatory FHSS baseline is inconsistent (8 channels vs >=50 channels)

Some sections require 8-channel FHSS while others assume >=50 channels for FCC strategy.

### Required decision
Declare one launch regulatory strategy per deployment region.

### Suggested immediate action
- If launch is 8-channel, mark >=50 as post-launch enhancement.
- If launch is >=50, update capability IDs, tests, and channel table implementation now.

## High 3 - "Method G complete" acceptance gate does not match mandatory capability list

Completion currently references `R-1..R-7`, while mandatory launch set elsewhere includes `R-01..R-12`.

### Suggested immediate action
- Replace completion criterion with explicit verification of all mandatory IDs.

## High 4 - Crypto ownership is ambiguous (H7-owned vs L072-owned)

The docs currently support both models at once. This creates nonce/replay ownership ambiguity.

### Required decision
Choose launch profile:
- **Profile A (recommended for launch):** H7 owns AES-GCM, nonce, replay. L072 transports authenticated ciphertext.
- **Profile B (hardening profile):** L072 owns AES-GCM, nonce, replay.

### Suggested immediate action
- Lock launch to Profile A.
- Move Profile B to post-launch with separate milestones.

## High 5 - Host UART transport has no backpressure contract

No HW flow control is fine, but current design allows pipelining without a defined credit/ready mechanism while burst-size limits are still open.

### Suggested immediate action
- Add host-level credits or strict per-sequence pacing in protocol v1.
- Define max inbound frame length and queue-overflow behavior (`ERR_BUSY`, drop policy counters).

---

## 2) Specific Code Suggestions

## A) Fix AES-GCM test harness length bug in host_check

File: `../firmware/bench/crypto_vectors/host_check.c`

### Problem
`lp_decrypt` is called with ciphertext length only, but API contract requires `ciphertext + tag` total length.

### Suggested patch
```c
/* before */
if (!lp_decrypt(key, nonce, buf, (size_t)clen, rec)) {

/* after */
if (!lp_decrypt(key, nonce, buf, (size_t)clen + 16u, rec)) {
```

### Also align function signatures
Use the canonical `bool` signatures from `lora_proto.h` instead of local `int` declarations.

```c
#include "lora_proto.h"

/* remove these local declarations:
int  lp_encrypt(...)
int  lp_decrypt(...)
*/
```

## B) Add null-callback guard in CSMA hop picker

File: `../firmware/common/lora_proto/lora_proto.c`

### Problem
`lp_csma_pick_hop` dereferences `sample` without checking for null.

### Suggested patch
```c
uint32_t lp_csma_pick_hop(uint32_t key_id,
                          uint32_t start_hop,
                          lp_rssi_sampler_fn sample,
                          void* ctx,
                          int16_t busy_threshold_dbm,
                          uint8_t max_skips,
                          uint8_t* skips_out) {
    if (sample == NULL) {
        if (skips_out) *skips_out = 0;
        return start_hop;
    }

    uint32_t hop = start_hop;
    for (uint8_t skips = 0; skips <= max_skips; skips++) {
        int16_t rssi = sample(lp_fhss_channel_hz(key_id, hop), ctx);
        if (rssi <= busy_threshold_dbm) {
            if (skips_out) *skips_out = skips;
            return hop;
        }
        hop++;
    }

    if (skips_out) *skips_out = max_skips;
    return hop - 1u;
}
```

## C) Define a hard host frame-size limit now

Files to add/modify in future `murata_l072` tree:
- `host/host_uart.h`
- `host/host_uart.c`

### Recommendation
Introduce one authoritative limit and enforce it before decode/write.

```c
#define HOST_INNER_MAX_LEN 320u

typedef enum {
    HOST_OK = 0,
    HOST_ERR_PROTO = 1,
    HOST_ERR_CRC = 2,
    HOST_ERR_TOO_LARGE = 3,
    HOST_ERR_BUSY = 4,
} host_status_t;
```

Behavior:
- Reject frames larger than `HOST_INNER_MAX_LEN` before COBS decode output write.
- Emit `ERR_PROTO` or `ERR_TOO_LARGE` with offending `type` and `payload_len`.

## D) Add explicit backpressure in host command flow

File target: future `host/host_cmd.c`

### Minimal v1 approach
- Allow at most one in-flight `TX_FRAME` per source queue.
- Return `ERR_BUSY` when queue is full.
- Include queue depth counters in `STATS_URC`.

```c
if (txq_is_full()) {
    send_err_busy(seq, CMD_TX_FRAME, txq_depth());
    return;
}
```

## E) Force one compile-time memory map contract

File target: future `ld/stm32l072cz_flash.ld` plus `boot/slot_select.c`

### Recommendation
Define constants once in a shared header used by both linker generation and boot code.

```c
#define L072_FLASH_BASE      0x08000000u
#define L072_FLASH_SIZE      (192u * 1024u)

#define BOOT_REGION_SIZE     (8u * 1024u)
#define SLOT_SIZE            (88u * 1024u)
#define CONFIG_REGION_SIZE   (8u * 1024u)
```

Then assert at build time:
```c
_Static_assert(BOOT_REGION_SIZE + (2u * SLOT_SIZE) + CONFIG_REGION_SIZE == L072_FLASH_SIZE,
               "Flash layout must exactly fit STM32L072 flash size");
```

Note: exact sizes above are placeholders. Final values must be chosen once and propagated to all docs.

---

## 3) Suggested Resolution Order

1. Freeze flash map constants and update docs.
2. Freeze regulatory channel-count strategy for launch.
3. Freeze launch crypto ownership model.
4. Patch immediate code defects (`host_check.c`, `lora_proto.c`).
5. Add host transport hard limits and backpressure semantics.
6. Start `murata_l072` implementation with these contracts pinned.

---

## 4) Definition of Ready for Phase 1 Coding

Phase 1 should not start until all of these are true:
- Single approved flash layout with exact addresses.
- Single approved launch regulatory channel strategy.
- Single approved launch crypto/replay ownership model.
- Host frame size limit and queue policy written in the host UART spec.
- Immediate code defects in shared helper/test code are patched.

Once those are locked, the firmware implementation can move quickly with much lower rework risk.

---

## 5) Companion Analysis (Claude Opus 4.7 v1.0, 2026-05-05)

**Reviewer:** GitHub Copilot (Claude Opus 4.7)
**Purpose:** Independent verification of the Codex findings above plus added analysis on items the Codex pass did not surface. Cross-references my full review at [../../AI NOTES/CODE REVIEWS/2026-05-05_LoRa_Firmware_Implementation_Plan_Review_ClaudeOpus4_7_v1_0.md](../../AI%20NOTES/CODE%20REVIEWS/2026-05-05_LoRa_Firmware_Implementation_Plan_Review_ClaudeOpus4_7_v1_0.md).

### 5.1 Verification of the Codex critical/high findings

**Critical 1 (flash map contradictions) — confirmed, more broken than stated.**
The arithmetic in [02 §4.3](02_Firmware_Architecture_Plan.md) does not close in two independent ways:

- **Address arithmetic:** Boot 4 KB at `0x08000000` → Slot A starts at `0x08001000`. Slot A = 92 KB → next address is `0x08001000 + 0x17000 = 0x0802F000`. The doc places Slot B at `0x08030000`, leaving a phantom 4 KB hole. Then "Reserved / config page 4 KB" at `0x08030000` would end at `0x08030400` — but `0x08030000` is itself the end of 192 KB Flash on the L072CZ, so the reserved page is entirely outside the chip.
- **Size arithmetic:** [02 §4.1](02_Firmware_Architecture_Plan.md) describes "rounds to a 96 KB slot" with two slots plus a 4 KB bootloader/recovery region. 96 + 96 + 4 = **196 KB > 192 KB**. Even ignoring the missing config page, the slot pair alone (192 KB) consumes the entire chip and leaves zero room for the boot/safe-mode region that [04 §3](04_Hardware_Interface_and_Recovery.md) Layer 2 depends on.

A self-consistent layout that fits 192 KB and preserves all stated regions:

```
0x08000000  Boot + safe-mode + slot-select  4 KB    (resident)
0x08001000  Slot A (active app)            88 KB
0x08017000  Slot B (inactive app)          88 KB
0x0802D000  Reserved / config / cal page   12 KB   (3× Flash pages × 4 KB; option-byte-friendly)
0x08030000  end of Flash (192 KB)
```

Or, if 88 KB feels tight for the launch capability set in [01 §10](01_Capabilities_Analysis_Custom_Firmware.md) plus mandatory growth headroom, drop to a single-slot launch and defer A/B (N-26) to Phase 6 as originally planned, recovering ~88 KB until then. **Strongly recommend the second option.** A/B-slot work in Phase 6 is the right time to design the linker layout against measured slot occupancy, not to commit it on day 1 from estimates.

Concur with Codex's recommendation to encode the layout once in a shared header with a `_Static_assert` covering total flash size. Add a second assert that each region is 4 KB-aligned (L0 Flash erase granularity).

**Critical 2 (regulatory FHSS, 8 vs ≥50 channels) — confirmed.**
[01 §1 R-01](01_Capabilities_Analysis_Custom_Firmware.md) commits to "8 channels" mandatory; [01 §2 N-06](01_Capabilities_Analysis_Custom_Firmware.md) and [03 Phase 4 W4-2](03_Bringup_Roadmap.md) then assume "weighted-but-non-zero probability over 50+ channels" for §15.247 compliance. These are not the same regulatory posture and you cannot have both at launch.

My companion review (§1.4 there) recommended **launching on the 8-channel R-01 scheme and deferring N-06 to Phase 5/post-launch.** That resolves Critical 2 in the conservative direction and removes the regulatory-analysis-before-code dependency. Concur with Codex that this decision must precede any FHSS-table code.

**High 3 (R-1..R-7 vs R-01..R-12 in completion criteria) — confirmed; minor.**
[00 §7](00_DECISION_Method_G_Commitment.md) point 1 says "All `R-1`..`R-7` LoRa requirements pass." [01 §1](01_Capabilities_Analysis_Custom_Firmware.md) defines R-01..R-12. Either (a) the gate intends *external* requirement IDs `R-1..R-7` from elsewhere in the v25 spec (in which case say so and link them) or (b) it should read R-01..R-12. Codex's reading (b) matches the rest of the docs; concur.

**High 4 (crypto ownership ambiguity) — partial confirm.**
The docs are actually unambiguous on the **default**: [00 §3](00_DECISION_Method_G_Commitment.md) lists "AES-128-GCM payload" in the L072 scope and [01 §4 N-12](01_Capabilities_Analysis_Custom_Firmware.md) explicitly says "**Default remains: H7 encrypts, L072 transports ciphertext.**" That is Profile A in Codex's framing, and it is already the committed launch profile.

What *is* genuinely ambiguous is the interaction between Profile A and N-13 ("per-frame nonce from L072 hardware RNG"). If H7 owns crypto, the H7 must source the nonce — it cannot ask the L072 for a nonce per frame without a UART round-trip that defeats N-13's latency premise. There are three coherent options:

- (a) Drop N-13 from the launch set; H7 sources nonces from its own TRNG/SE050.
- (b) Move nonce generation to L072 only and ship it back in `TX_DONE_URC` for replay-window bookkeeping (subtle: replay-window is per-source-id, not per-MCU, so the H7 still needs to know what nonce went out).
- (c) L072 derives nonces deterministically from `(source_id, sequence)` per the existing scheme in [lora_proto.h](../firmware/common/lora_proto/lora_proto.h), no TRNG needed; N-13 becomes "L072 contributes 5 random bytes at the *boot epoch*, not per frame."

(c) is the cleanest fit with the existing nonce layout `[source_id(1) | sequence(2) | timestamp_s(4) | random(5)]`. The 5 random bytes change once per boot and are advertised in `BOOT_URC`. Recommend committing to (c) and updating [01 §4 N-13](01_Capabilities_Analysis_Custom_Firmware.md) accordingly.

**High 5 (no host-UART backpressure contract) — confirmed; concur with Codex's `ERR_BUSY` proposal.**
Add to Codex's recommendation: the credit count should be **per-priority-class** (P0 always gets at least one reserved slot, P2/P3 share the rest), so a P3 image-fragment burst can never starve a P0 ControlFrame at the host-UART boundary. ~16 LoC and ~16 B RAM extra.

### 5.2 Verification of the Codex code suggestions

**(A) `host_check.c` `lp_decrypt` length bug — confirmed real bug.**
[lora_proto.h](../firmware/common/lora_proto/lora_proto.h) explicitly documents (IP-003 contract): `lp_decrypt`'s `ct_len` is the **total** ciphertext + tag length. [host_check.c](../firmware/bench/crypto_vectors/host_check.c) calls it with `(size_t)clen` (ciphertext only), so for any non-empty plaintext the decrypt would either fault on a short read or report tag-mismatch. The reason the test passes today on the `empty_pt` and `block_zero` vectors is that mbedTLS treats the `ct + tag` boundary using the supplied length and expects the tag to immediately follow; with `clen=0` the buffer happens to start with the tag and the bug accidentally produces the right answer. The `block_zero` case (clen=16) reads only the ciphertext and treats the tag as ciphertext-2 — this **should** be failing today. If it isn't, the test harness is suppressing the failure somehow; worth investigating before just patching.

Concur with Codex's patch. Also concur with switching to the canonical `bool` signatures via `#include "lora_proto.h"`; the local `int` shadow declarations are exactly the kind of thing that hides ABI drift.

**(B) Null-callback guard in `lp_csma_pick_hop` — accept with a different default.**
Codex proposes returning `start_hop` if `sample == NULL`. That's defensible but silently makes CSMA a no-op, which is the failure mode this code exists to prevent. Prefer either:

- (b1) Make the null-sample contract explicit: assert in debug builds, return `start_hop` in release. Document that calling with `sample == NULL` disables CSMA and is a configuration choice, not a fallback.
- (b2) Don't accept null at all; require callers to pass a `lp_rssi_sampler_always_clear()` stub that returns `INT16_MIN` if they explicitly want CSMA disabled.

(b2) is cleaner and avoids a class of bug where a null-pointer config silently degrades airtime safety. Recommend (b2).

**(C) `HOST_INNER_MAX_LEN` — concur.**
Strongly agree. Suggest an additional constant for the **decoded** max separate from the **encoded-on-wire** max, since worst-case COBS expansion is +1 byte per 254 input bytes plus 2 framing zeros. Without that, the RX buffer can be sized one byte too small for a maximum-length frame and reject perfectly valid traffic.

**(D) Backpressure in host command flow — concur with addition above (per-priority credits).**

**(E) Single compile-time memory-map header — concur.**
This is the clean fix for Critical 1. One addition: the same header should be the **only** place where boot/slot/config addresses live. The linker script should `#include` it (gcc-ld supports this via `-imacros` / `-include` on the preprocessor pass), not hand-mirror the constants. Hand-mirrored constants in `.ld` files are a perennial source of "everything links but the second slot boots into garbage" bugs.

### 5.3 Items Codex did not surface that I want flagged here

These are short summaries; full discussion is in my review at [2026-05-05_LoRa_Firmware_Implementation_Plan_Review_ClaudeOpus4_7_v1_0.md](../../AI%20NOTES/CODE%20REVIEWS/2026-05-05_LoRa_Firmware_Implementation_Plan_Review_ClaudeOpus4_7_v1_0.md).

1. **Effort estimate in [00 §5](00_DECISION_Method_G_Commitment.md) is ~2× optimistic.** 4–6 weeks for launch firmware is closer to 8–12 weeks once Phase 2 SX1276 driver bring-up and Phase 4 spectrum-bench validation are honestly costed. Schedule pressure is not a Method G concern, but downstream H7-team planning anchors on this number.
2. **Event-queue priority discipline is unspecified** in [02 §3](02_Firmware_Architecture_Plan.md). The pseudocode is FIFO; the requirements need a 3-level priority queue with a per-handler runtime cap. Otherwise P0 preempt at the L072 application layer can be blocked by an in-progress `EV_HOST_RX` of an image fragment.
3. **Safe-mode listener in [04 §2.4](04_Hardware_Interface_and_Recovery.md) has a single point of failure**: it depends on USART2 being configured at the right baud. If a future `CFG_COMMIT`-persisted `host_baud` value is honoured by the safe-mode path, Layer 2 brick recovery silently fails. Spec must say (a) safe-mode uses a fixed baud table independent of persisted config, and (b) auto-baud across `{921600, 115200, 9600}` from the `0xA5 0x5A` preamble.
4. **`BOOT_URC` should carry reset cause** (POR / NRST / IWDG / soft / brown-out) from the L072 RCC registers. Costs 1 byte and is the cheapest field-diagnostics improvement available.
5. **No DoS rate-limit on incoming LoRa frames** before AES-GCM verification. A flood at SF7 (~250 frames/s) crowds out real ControlFrame processing. Per-(SF × channel × claimed-source-id) token bucket, ~5 KB code, ~256 B RAM.

---

## 6) Companion Analysis (Gemini 3.1 Pro Preview, 2026-05-05)

**Reviewer:** GitHub Copilot (Gemini 3.1 Pro)
**Purpose:** Review of high-level architectural decisions, capabilities analysis, and hardware-specific risk factors not covered by the primary codebase queries.

### 6.1 Architectural Decisions Review
- **Commitment to Method G:** Outstanding pivot. Bypassing the AT firmware fundamentally enables the sub-millisecond latencies necessary for the v25 control plane.
- **Protocol Location Shift (H7 → L072):** Sound architectural decoupling. Shifting the PHY-level radio management into the L072 allows the H7 to focus strictly on payload prioritization and application state.
- **No LoRaWAN MAC:** Necessary and correct. LoRaWAN duty-cycle regulations natively conflict with the immediate, high-priority, preemptable traffic needed for P0 ControlFrames.
- **Defense-in-Depth Brick Recovery:** Phenomenal. The 5-layer plan combined with a "golden binary" ensures field updates and dev iterations are effectively unbrickable.

### 6.2 Proposed Architecture & Structure Review
- **COBS-Framed UART Transport:** COBS eliminates complex state machines and pairs perfectly with STM32's `DMA-on-IDLE` interrupt. The CRC16 addition prevents mysterious logic bugs from framing drift.
- **Bare-Metal C & Static Allocation:** Operating within 192KB Flash / 20KB RAM requires zero `malloc`, pre-sized circular buffers, and dropping exceptions/RTTI (standard embed best practices). 
- **Hardware Timer Triggers (N-02):** Using the `TIM2` 32-bit timer to drive the SX1276 TX trigger is a massive upgrade over software polling, giving microsecond-accurate time-of-air anchors.

### 6.3 Risks & Recommendations
1. **H7 Boot Glitching (BOOT0 / NRST):** During power-on-reset of the H7, its GPIO pins will float before being configured. **Recommendation:** Ensure `LORA_BOOT0` and `LORA_NRST` have appropriate hardware pull-downs/pull-ups on the carrier board, or verify the Murata strapping prevents floating.
2. **IWDG Configuration (N-20):** Feeding the IWDG from a high-priority UART or SysTick interrupt is dangerous. **Recommendation:** `iwdg_kick()` should only occur at the end of the `main()` while loop to strictly confirm that all state machines correctly iterated.
3. **921600 Baud Tolerance:** A 2-3% clock drift from temperature/voltage variations can cause UART framing errors. **Recommendation:** Use the L072's HSI16 with clock autocalibration via the CRS leveraging the LSE (if equipped on the SiP) to ensure UART baud drift remains near 0%.
4. **Phase 6 "Field Update" Complexity:** Jumping straight to A/B firmware bank swaps creates high linker-script and vector-table offset complexity. **Recommendation:** Ensure `SCB->VTOR` is correctly relocated immediately upon booting slot B, and that interrupts are completely disabled beforehand.
6. **P0-preempt path needs a single `apply_profile_full()`** that re-emits every profile-dependent SX1276 register from scratch. Incremental updates after a `RADIO_RESET` will produce intermittently-wrong PHY config.
7. **Failsafe-tripped behaviour is undefined** for handheld vs. tractor roles in [01 §1 R-11](01_Capabilities_Analysis_Custom_Firmware.md). Tractor likely needs to broadcast `FAILSAFE_TRIPPED` on every slot until link recovers; handheld likely needs to stop transmitting. Spec before code.

### 5.4 Updated Definition of Ready for Phase 1 coding

Concur with Codex's [§4 DoR](#4-definition-of-ready-for-phase-1-coding), with these additions:

- Single approved flash layout — **concur**, plus the layout must be encoded in a header consumed by both linker and boot code (Codex E).
- Single approved launch regulatory channel strategy — **concur**, recommend committing to 8-channel R-01 baseline and deferring N-06 to Phase 5+.
- Single approved launch crypto/replay ownership model — **concur**, recommend Profile A (H7 owns AES-GCM) with N-13 reframed as boot-epoch random-bytes (option (c) in §5.1 above).
- Host frame size limit + queue policy + **per-priority-class credit allocation** in the host UART spec.
- Immediate code defects patched — **concur**.
- **Add:** event-queue priority discipline specified in [02 §3](02_Firmware_Architecture_Plan.md).
- **Add:** safe-mode auto-baud + persisted-config exclusion specified in [04 §2.4](04_Hardware_Interface_and_Recovery.md).
- **Add:** `BOOT_URC` payload includes reset cause in [04 §2.3](04_Hardware_Interface_and_Recovery.md).

### 5.5 Verdict

The Codex review correctly identified two genuinely critical blockers (Critical 1 flash map, Critical 2 regulatory baseline) and one real shipped code bug (host_check.c length). All other Codex items are correct in direction; my notes above adjust two of them in the conservative direction (B → use a sentinel sampler rather than a null-tolerant pick; D → per-priority credits not flat queue).

**Combined with the items in §5.3, the launch firmware is not yet ready to start Phase 1 coding** — the flash map alone would invalidate the linker script on first commit. With the §5.4 DoR additions resolved, the path to Phase 1 is short and the architecture is sound.

> **Method G stays committed. Resolve the Codex criticals + the §5.3 additions before Phase 1; do not let "we'll fix the linker layout later" become a Phase 1 blocker.**

---

## 7) Companion Analysis (GitHub Copilot v1.0, 2026-05-05)

**Reviewer:** GitHub Copilot
**Purpose:** Add system-boundary and implementation-contract analysis that complements the Codex/Claude/Gemini passes above. Full standalone review is archived at [../../AI NOTES/CODE REVIEWS/2026-05-05_LoRa_Firmware_Implementation_Plan_Review_Copilot_v1_0.md](../../AI%20NOTES/CODE%20REVIEWS/2026-05-05_LoRa_Firmware_Implementation_Plan_Review_Copilot_v1_0.md).

### 7.1 Concurrence With Existing Findings

I concur with the existing review consensus:

- **Method G remains the right strategic direction.** Once the Max Carrier's host-inaccessible SX1276 SPI bus was confirmed, custom L072 firmware became the only no-new-BOM path that can preserve per-frame radio control and the original v25 protocol intent.
- **Flash map, regulatory channel strategy, crypto ownership, and host backpressure are pre-code contracts.** They should be resolved before the `murata_l072` tree starts accumulating implementation decisions around unresolved system questions.
- **The immediate proposed-code fixes are valid.** The `host_check.c` decrypt-length contract mismatch is a real test-harness defect, `lp_csma_pick_hop()` needs an explicit null/disabled-sampler policy, and host UART hard limits plus `ERR_BUSY` behavior belong in protocol v1.

### 7.2 Additional Blocker - Method G Scope Must Cover Every Radio Node

The current Method G folder reads primarily as a Portenta Max Carrier plan. That is not enough for the v25 network. The production air link includes at least tractor and base Portenta Max Carrier radios, and may include the MKR WAN 1310 handheld. If one node remains on vendor AT firmware, the network cannot preserve the Method G promises: custom P2P air bytes, per-frame FHSS, adaptive PHY, P0 preempt, and a shared no-LoRaWAN MAC.

Add a Phase 0 node matrix before coding:

| Node | Host | L072 access path | Recovery owner | Golden binary storage | Method G requirement |
|---|---|---|---|---|---|
| Tractor Max Carrier + X8 | H747/X8 | `Serial3` / `/dev/ttymxc3` | H7 or X8 | H7 flash or X8 eMMC | required |
| Base Max Carrier + X8 | X8 and/or H747 | `/dev/ttymxc3` likely | TBD | X8 eMMC likely | required |
| Handheld MKR WAN 1310 | SAMD21 | MKR modem UART | TBD through MKRWAN update path | TBD | required unless handheld is deferred |

Phase 0 should not be called green until each in-scope node class has a proven flash/recovery path or is explicitly deferred from v25 launch.

### 7.3 Additional High Finding - Define the L072 as Thin Modem or Protocol Endpoint

The docs currently mix two different L072 roles:

- **Thin deterministic radio modem:** H7/base/handheld host submits already authenticated on-air bytes plus metadata and deadlines; L072 owns channel access, retune, LBT, preempt, radio reset, RX/TX metadata, stats, and recovery.
- **Full LoRa protocol endpoint:** host submits semantic commands or telemetry; L072 owns frame construction, AES-GCM, replay, nonce policy, scheduling, failsafe radio behavior, and possibly telemetry/image fragmentation.

For v25 launch, I recommend the **thin modem** model. It matches the existing H7/M4 safety split, keeps replay/key state out of the 20 KB L072 unless deliberately promoted later, and makes `TX_FRAME`/`RX_FRAME_URC` a crisp ABI. The full-endpoint model is a valid later hardening path, but it should not be blended into Phase 2 by accident.

Concrete doc change: in [04 §2.3](04_Hardware_Interface_and_Recovery.md), define `TX_FRAME` launch semantics as "payload is a complete authenticated on-air frame, already encrypted by the host" unless/until `crypto_in_l072` negotiates a new protocol version.

### 7.4 Additional High Finding - COBS Host ABI and KISS Air Framing Need a Hard Boundary

COBS over UART is a good host transport choice. The existing reusable code, however, is KISS-oriented for the current air/serial helper path. That is not a conflict if the boundary is deliberate:

- `host/` owns COBS, host CRC, `seq`, command/URC types, queue credits, and max frame length.
- `proto/` owns the bytes that go on the radio and any KISS-or-successor air framing.
- H7/base host code sends COBS `TX_FRAME` commands, not raw KISS serial frames.
- `RX_FRAME_URC` carries radio metadata plus the decoded/validated on-air bytes in exactly one documented layout.

Add compatibility tests before firmware implementation: COBS encode/decode, oversized-frame rejection, CRC failure, `TX_FRAME` queue-full behavior, and `RX_FRAME_URC` parsing on the host side.

### 7.5 Additional High Finding - Method G Fixes AT Latency, Not Airtime Physics

Custom L072 firmware removes the AT parser tax but does not remove half-duplex LoRa airtime. The current SF7/BW250 control profile leaves only a few milliseconds of margin inside a 50 ms control period once LBT, CCA, retune, guard time, scheduler jitter, and any separate heartbeat are counted.

The launch scheduler should state these invariants explicitly:

- latest P0 replaces older P0; stale P0 is dropped, not queued;
- ControlFrame carries heartbeat/liveness when control is active;
- telemetry/image frames carry deadlines or are only accepted when they cannot delay the next P0 slot;
- SF8/SF9 fallback has a documented degraded cadence, or trips failsafe rather than pretending 20 Hz still holds;
- `TX_DONE_URC` exposes actual timing so HIL tests can compare scheduler intent to measured radio behavior.

This should be part of Phase 2/3 acceptance, not a later optimization.

### 7.6 Additional Medium Findings

1. **From-scratch bare-metal bring-up dropped the earlier fork-point mitigation.** The earlier Method G note recommended proving a known STM32L0/SX1276 firmware first. If the plan now intentionally goes from scratch, update effort/risk estimates and Phase 1 gates around RF switch, TCXO, DIO mapping, and SX1276 errata.
2. **Default TX power must be node/region-specific.** A universal `tx_power_dbm = 14` default conflicts with earlier tractor/base +20 dBm assumptions and with base antenna gain/EIRP calculations.
3. **Documentation drift around PHY profiles remains.** The current code pins control SF7/BW250 and image BW500, while some planning text still describes older BW125/BW250 blockers. The Method G docs should include the current canonical PHY table directly.
4. **BOOT0 and golden-binary ownership should be evidence-backed.** Capture reset/BOOT0 traces, ROM bootloader ACK transcripts, and one intentional bad-image recovery for every in-scope node class.

### 7.7 Updated Phase 1 Definition of Ready Additions

Add these to the existing DoR in §4 and §5.4:

- Per-node Method G scope table approved: tractor, base, handheld, and any deferred node explicitly marked.
- L072 launch role approved: thin modem vs full protocol endpoint. Recommendation: thin modem.
- COBS/KISS boundary and host `TX_FRAME`/`RX_FRAME_URC` wire layouts specified with tests.
- P0 scheduling invariants specified, including stale-frame replacement and degraded SF cadence policy.
- Node/region TX power and EIRP policy specified before spectrum testing.
- Known-reference firmware or equivalent bring-up step chosen before committing to a fully custom SX1276 driver path.

### 7.8 Verdict

The combined review set is converging: **Method G should proceed, but Phase 1 coding should wait until the system contracts are closed.** The most important additional point from my pass is node coverage. A perfect tractor-side L072 firmware does not complete Method G unless the base and handheld story is equally explicit.

