# LoRa Firmware Implementation Plan Review (Method G)

**Date:** 2026-05-05
**Reviewer:** GitHub Copilot (Claude Opus 4.7)
**Target:** [LifeTrac-v25/DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/) — docs 00–04
**Companion review:** [2026-05-05_LoRa_Firmware_Implementation_Plan_Review.md](../2026-05-05_LoRa_Firmware_Implementation_Plan_Review.md) (Gemini 3.1 Pro)
**Scope:** Big-picture decisions, capability scoping, architecture, bring-up roadmap, host-interface contract, and the code structures the plan commits to writing.

---

## 0. TL;DR

The Method G plan is **strategically sound and unusually well-scoped** for an MCU-firmware effort of this size. The decision record (00) is honest about reversibility, the capability list (01) is properly graded, and the host interface (04) takes brick-recovery seriously from day one — which is the single most common way custom-modem-firmware projects die.

The areas that deserve sharper thinking before Phase 1 starts:

1. **The 4–6 week effort estimate in [00 §5](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md) is optimistic by ~2× given the scope committed for launch (Phase 4 exit).** This isn't a reason not to do it — the project is explicitly not time-constrained — but the estimate is one of the few things in the docs that doesn't match the rest of the rigour, and downstream planning shouldn't anchor on it.
2. **The brick-recovery design has one quiet single-point-of-failure**: the safe-mode magic depends on USART2 actually coming up at the right baud before the main loop is corrupted. Worth adding a Layer-2.5 (see §4 below).
3. **The cooperative scheduler in [02 §3](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) hides a real-time hazard** around P0 preempt vs. host-UART RX-DMA completion — the event-queue approach is right but the queue contract needs to be specified, not just sketched.
4. **Phase 4's "launch milestone" is quietly aggressive.** N-04 (LBT) + N-06 (quality-aware FHSS) + N-07 (TX-power adapt) all interact non-trivially with the §15.247 regulatory story, and the docs commit to launching with all three before producing the validated channel list (Q5 in [02 §8](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md)). Recommend reordering.
5. **The proposed source layout is good but over-modularised in two places** that will produce friction during bring-up.
6. **No discussion of how bring-up tracks against [LORA_PROTOCOL.md](../../DESIGN-CONTROLLER/LORA_PROTOCOL.md) drift.** The protocol spec on the H7 side and the firmware on the L072 side will absolutely drift unless one is tagged as canonical and the other regenerated from it.

Detailed review follows.

---

## 1. Big-picture decisions

### 1.1 The G commitment itself — concur, with one caveat

[00 §1](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md) is the right call. The previous Gemini review correctly identified the strategic logic; I won't repeat it. One thing the decision record doesn't quite say out loud:

> Method G is also the only option that keeps the v25 over-the-air protocol useful as a **portable spec**. If we'd shipped on D (raw-P2P AT) the over-the-air format would have been silently constrained by what the AT firmware would consent to emit. Owning the L072 firmware means the LORA_PROTOCOL.md document is now actually authoritative.

Worth adding a line to [00 §8](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md) ("what this decision does *not* change"), because right now that section reads as if the protocol is unchanged by accident, when really the *ability to keep it stable* is one of G's biggest wins.

**Caveat on the effort estimate:** [00 §5](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md) says "~4–6 weeks of focused L072 firmware work (single contributor, FTE)." Counting just the launch scope (Phases 0–4 from [03](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md)):

- Phase 0: 0.5–1 week (toolchain, vendor flash pipeline qualification)
- Phase 1: 1–2 weeks (HAL bring-up, safe-mode, register dump — first time on real silicon, usually slower than expected)
- Phase 2: 2–3 weeks (SX1276 driver port + COBS + crypto + first end-to-end TX/RX is usually where projects burn a month they didn't budget)
- Phase 3: 2–3 weeks (R-01..R-12 — FHSS table tuning, three-profile swap, P0 preempt timing all need HIL iteration)
- Phase 4: 2–3 weeks (LBT/quality-aware FHSS need spectrum-bench validation, which is where weeks evaporate)

That's a realistic **8–12 week launch-firmware FTE budget**, not 4–6. Recommend the decision doc say "~10 weeks for launch (Phases 0–4); commercial-readiness (Phase 6) ~6–8 additional weeks; long-horizon (Phase 7) open-ended." The "no schedule pressure" stance still holds, but a credible estimate matters for downstream planning (e.g., when the H7 team can rely on the L072 binary being feature-complete enough to drop the AT-firmware fallback).

### 1.2 LoRaWAN MAC explicitly out — concur

[00 §3](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md) and [01 §9](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md) are unambiguous. Good. The one thing missing is an explicit note about **what we lose** by not being LoRaWAN-compatible: any future "join a public TTN/Helium gateway for telemetry-only fallback when the base station is dead" is foreclosed by this decision. That's almost certainly the right tradeoff for a control plane, but the docs don't address it. Worth a one-paragraph entry in [00 §2](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md) explaining that even option C/F as a "telemetry-only secondary" is rejected because of BOM/complexity.

### 1.3 Reversibility window through Phase 4 — concur, but tighten the contract

[00 §9](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md) and [02 §6](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) say the right thing — keep `radio/` and `proto/` separable so a Method-E fallback only touches `radio/` and `hal/spi.c`. But the architecture doesn't enforce this with a build target. Recommendation:

- Add a `tests/host_native/` build that compiles `proto/` against a **mock radio** on a host PC. If that build stays green throughout Phases 2–4, the reversibility property is verified continuously, not just claimed.
- Without that, the radio/proto split will silently rot the first time a P0-preempt path needs to call into a radio register directly from the scheduler.

### 1.4 Capability commit list ([01 §10](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md)) — mostly right, two changes recommended

Strongly agree with the R-XX mandatory set. Two adjustments to the launch N-XX set:

**Move N-09 (deep sleep) into the launch set, not Phase 5.**
The handheld controller's battery story is one of the few things end-users will notice immediately. Postponing deep-sleep to Phase 5 means the launch demo is on a tethered handheld or one with brick-of-the-week battery life. Deep-sleep STOP-mode entry on L072 is well-trodden ground (~1 week of firmware work if `pwr.c` is written cleanly the first time). Doing this in Phase 4 also forces the scheduler ([app/scheduler.c](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md)) to be sleep-aware from the start, rather than being retrofitted later — retrofits to a slot scheduler are notoriously prone to introducing latency outliers.

**Defer N-06 (quality-aware FHSS) to Phase 5 / fast-follow.**
This is the single capability in the launch set with the highest spec-vs-implementation gap. The "weighted-but-non-zero probability over 50+ channels with statistical uniformity preserved" claim in [01 §2 N-06](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md) is the kind of regulatory argument that should be reviewed by an FCC compliance specialist before code is written, not after. For launch, ship fixed 8-channel FHSS (R-01); add quality-awareness post-launch when the regulatory analysis is in hand and the production telemetry shows whether it's actually needed.

Net: launch set becomes `{R-01..R-12, N-04, N-07, N-09, N-13, N-15, N-19, N-20, N-21, N-22, N-24}`. Drops one item, adds one, reorders Phase 4↔5 work.

### 1.5 The five-layer brick recovery — agree it's the right shape; one gap

[04 §3](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) is excellent. The single gap: **Layer 2 (UART safe-mode magic) requires that USART2 comes up at exactly the baud the H7 is sending at.** If a future build accidentally changes the default baud (or the `host_baud` config in [04 §6](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) is persisted to Flash and a bad value is written), the safe-mode magic will be received as garbage and Layer 2 fails silently, forcing escalation to Layer 3.

Mitigations to specify in [04](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md):

- The first 500 ms of safe-mode listen should **auto-baud** by trying `{921600, 115200, 9600}` in sequence (75 ms each, leaving 275 ms for whichever matched). Auto-baud detection from the magic preamble's `0xA5 0x5A` is straightforward — both bytes have a known bit pattern.
- The persisted `host_baud` config must **never apply to the safe-mode listener**. Safe-mode runs from a hardcoded baud table, period. Document this as binding firmware behavior in [04 §2.4](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md).
- The magic byte sequence should not include the ASCII letters "SAFE" — they're 8-bit characters and at the wrong baud they'll alias differently than the `0xA5 0x5A` preamble. Use a sequence whose Hamming distance to common UART noise patterns is high at all the candidate baud rates (e.g. `0xA5 0x5A 0xA5 0x5A 0x5A 0xA5 0x5A 0xA5`).

### 1.6 The two-stable-interfaces contract ([02 §6](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md)) — concur, missing teeth

The contract lists over-the-air format and host UART protocol as the two stable surfaces. Both have version fields. Good.

What's missing: **a CI gate that fails any change to `proto/lora_proto.h` or `host/host_cmd.h` that doesn't bump the protocol version constant.** Without that, the version field is documentation, not enforcement. A simple check would be:

- A header file `proto/lora_proto_abi.h` containing only the on-air struct definitions and the `proto_version` constant.
- A test that hashes the file's normalized contents and compares against a checked-in golden hash.
- The hash file is updated only by the same commit that bumps the version.

Same idea for the host UART command table. This is a 1-day implementation that catches a class of bug that otherwise costs days each time it happens.

---

## 2. Architecture (doc 02)

### 2.1 Source layout — good with two trims

The [02 §2](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) layout is reasonable. Two changes I'd push back on:

**Merge `radio/sx1276_safety.c` into `radio/sx1276.c`.**
P0 preempt is "abort current TX, drop to standby, re-init, switch profile." That's three SX1276 register writes. Splitting it into a separate translation unit guarantees that the P0-preempt path will end up calling through a public API surface of `sx1276.c`, adding function-call overhead in exactly the path where you're measuring 100 µs latency. Keep it `static inline` in the same TU.

**Don't pre-create `boot/golden_jump.c` and `boot/slot_select.c` in Phase 1.**
Phase 1 only needs `boot/safe_mode.c`. Stubbing the others now will pull A/B-slot assumptions into the linker script before A/B is designed, and it tends to ossify decisions you haven't actually made. Add them in Phase 6 when N-26 is real.

### 2.2 Cooperative scheduler — right model, under-specified contract

[02 §3](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) commits to bare-metal, single-threaded, event-queue dispatch. That's the right call (an RTOS on M0+ for this workload would be wasteful and would burn RAM you don't have).

The hazard the doc doesn't address: **what is the priority discipline of the event queue?** The pseudocode shows FIFO pop. But:

- An `EV_HOST_RX` carrying a P0 ControlFrame must preempt a half-processed `EV_RADIO_RXD` (which is application data, never safety-critical) in flight in the main loop.
- An `EV_FAULT` must preempt everything else.
- An `EV_TICK` must run on schedule even if `EV_RADIO_RXD` events are flooding in.

Recommendations to add to [02 §3](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md):

1. Make the event queue a small **multi-level priority queue**, not a FIFO. Three levels (P0 = fault/safety/P0-TX, P1 = tick, P2 = everything else) is enough.
2. Explicitly cap **handler runtime per event** (e.g. 200 µs hard ceiling); long work splits into a continuation event that re-enters the queue at lower priority.
3. The IWDG kick belongs **after the priority-1 (tick) handler runs**, not at the bottom of the loop. If a P2 handler stalls, you want the watchdog to fire even though ticks are still being serviced. (The Gemini review touched on IWDG placement; this is a sharper version of the same point.)
4. Document that the loop **never blocks** except inside `wfi`. Any future bring-up code that does `HAL_Delay()` in a handler is a bug, not a temporary hack.

### 2.3 RAM/Flash budget — credible, with one underestimate

[02 §4.1](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md): the SX1276 driver at ~14 KB is realistic for a slimmed Semtech reference; don't assume 30% reduction without a measurement.
The "tinycrypt ~9 KB or HW-AES ~3 KB" line is correct for raw AES, but **GCM mode adds another ~3–4 KB on top of either** because GHASH multiplication is non-trivial to fit in code-size-optimized form. Realistic: tinycrypt-AES-GCM ~12 KB, HW-AES + software GHASH ~7 KB. Update the budget.

[02 §4.2](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) RAM looks right. The 2 KB stack budget is **tight** — ISR depth on a Cortex-M0+ with nested interrupts disabled isn't bad, but a HardFault handler that wants to dump registers into a backup buffer wants ~500 B of headroom. Suggest 2.5 KB.

### 2.4 Build system — two entry points is one too many

[02 §5](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) commits to shipping both Make and PlatformIO. Pick one as canonical (Make, given CI plans) and document the other as best-effort. Maintaining two build systems for the same source tree across A/B-slot linker variants and signed-image post-processing will eat hours. The PlatformIO entry can be a thin `extra_scripts` wrapper that invokes the Makefile if it's really wanted for VS Code one-click flash, but the "two source-of-truth build systems" framing in the doc invites bit-rot.

---

## 3. Bring-up roadmap (doc 03)

### 3.1 Phase ordering — sound, with the Phase 4↔5 swap from §1.4

Already covered. The roadmap is otherwise well-gated.

### 3.2 Phase 0 — concur

Re-running the vendor AT firmware to qualify the bootloader pipeline before any custom binary is excellent discipline. This is the kind of step that gets skipped under schedule pressure and then bites in Phase 1 when you can't tell if your binary is broken or your flashing path is broken.

### 3.3 Phase 1 — add one work item

Add **W1-9: write the host-side `stm32flash`-equivalent and version-pin it.** [03 §Phase 1 W1-4](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md) mentions reusing the `stm32flash` source as reference, but the firmware development cycle will involve hundreds of flashes; the H7-driven flashing tool is itself a small project (~500 LOC) and treating it as "we'll figure it out" risks the team flashing through the Arduino sketch for weeks because the helper script never quite got finished.

### 3.4 Phase 2 — add a Q3 measurement gate

[02 §8 Q3](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) (max DMA-friendly UART burst at 921600) is flagged as deferred to Phase 2. Make it an **explicit work item W2-X** with an exit criterion (e.g. "burst sizes from 16 B to 512 B all complete with zero overruns over a 1 M-frame stress test"), otherwise it'll get hand-waved in code review and surface as flaky CI in Phase 3.

### 3.5 Phase 3 exit criteria — needs PER over time, not just instantaneous

[03 Phase 3 exit](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md): "1 hour at 20 Hz cadence; PER < 1% on a clean bench." That's 72,000 frames. Statistically reasonable, but the failure modes Method G enables (FHSS table corruption, register-rot from a bad write, P0-preempt leaving the radio in a stuck state) often manifest as PER drift over **days**, not hours. Add a rolling-24-hour PER acceptance test before Phase 3 is closed.

### 3.6 Phase 4 — regulatory work item missing

Q5 from [02 §8](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) ("validated regulatory channel list") is correctly listed as Phase 4 work (W4-8). But it's listed alongside the LBT and quality-aware FHSS work items as if they're parallel. They're not — W4-8 is a **prerequisite** for W4-2 (N-06) being safe to merge. Add an explicit dependency arrow.

### 3.7 Cross-phase ongoing work — add one

[03 "Cross-phase ongoing work"](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md): add **regression-on-real-hardware**. Every Phase exit should re-run the previous Phase's exit criteria, not just the new ones. This is the only way to catch the kind of regression where Phase 4's LBT changes accidentally widen Phase 3's per-fragment airtime histogram.

---

## 4. Hardware interface & recovery (doc 04)

### 4.1 Pin map — concur

[04 §1](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) is appropriately humble about what's not yet confirmed (BOOT0 routing, optional wake/IRQ, optional SWD test points). These are the right things to confirm in Phase 0/1 with a multimeter and the carrier schematic.

### 4.2 Frame format — good, with two clarifications needed

[04 §2.2](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) frame format:

- **CRC scope:** "CRC16 ... over [ver..payload]" — good, but specify byte order. CCITT-FALSE is conventionally MSB-first transmitted but the L072 will compute it as a 16-bit value; pick one and write it in the spec to prevent endianness debates in code review.
- **`payload_len`** is little-endian; good, but the **CRC field's endianness is unspecified**. State that explicitly — typically little-endian to match `payload_len`, but writing it down avoids the inevitable bug.
- **COBS overhead:** worst-case COBS adds ~0.4% overhead to a 250 B inner frame; for a 16 B ControlFrame it's 1 byte. The framing budget should call this out when sizing the host-UART TX queue.

### 4.3 Command table — minor additions

[04 §2.3](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md):

- Add `0x42` `STATS_GET_KEY` / `0xC2` `STATS_KEY_URC` for fetching a specific counter rather than the whole dump. The whole-dump becomes painful as histograms grow.
- Add `0x21` `CFG_GET` / `0xA1` `CFG_VAL_URC` (the doc mentions "a future `cfg get`" in §6 but doesn't reserve type codes; reserve them now).
- `0xF0 BOOT_URC` should carry **reset cause** (POR / NRST / IWDG / soft / brown-out) — invaluable for diagnosing field issues. The L072 RCC registers preserve this across reset.

### 4.4 Brick recovery — see §1.5 above

Already covered the auto-baud and magic-sequence improvements.

One additional point: **[04 §3](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) Layer 1 (NRST) is listed as recovering "from a runaway main loop."** It actually doesn't — a runaway main loop that keeps stomping on UART/SPI but doesn't trip IWDG can survive an NRST cycle and come right back. Layer 1 only recovers from "I want to reset and the L072 is willing to be reset." The doc should say so. The truly runaway case is what Layers 2–4 exist for.

### 4.5 Configuration & feature flags — persistence model missing

[04 §6](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) lists the config keys but doesn't say whether `CFG_SET` writes are **persisted across reboot** or live only until the next reset. Both have failure modes:

- Persistent: you can brick yourself with a bad config (e.g. `host_baud = 0`) and even Layer 2 won't help if the safe-mode listener honors it (mitigated by §1.5 above).
- Volatile: every boot re-applies defaults, and every H7 boot has to re-configure the modem, slowing recovery.

The right answer is probably **volatile by default, with an explicit `CFG_COMMIT` command** that persists the current runtime config to a Flash page. Document this in [04 §6](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md).

### 4.6 H7-side responsibilities — concur, add one

[04 §5](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) H1–H6 are all good. Add:

- **H7:** the H7 must verify, on every boot, that the L072 firmware's `VER_URC` capability bitmap is a superset of what the H7 was built against. If the L072 is older than expected (e.g. someone re-flashed the golden binary and forgot to re-apply the latest), the H7 must enter degraded mode rather than calling features that don't exist.

---

## 5. Proposed code (the structures the docs commit to)

No source files exist yet, so this reviews the **design patterns and data structures** the plan commits to writing.

### 5.1 COBS + length-prefix + CRC layered transport

Solid, conventional choice. The combination is intentionally redundant (length tells you when to stop; COBS tells you where the boundary is; CRC tells you if it was right) and that redundancy is what catches the weird failure modes — a UART glitch that drops a byte mid-frame is caught by length mismatch *and* CRC mismatch *and* COBS-decode-failure, which makes diagnostics much easier than any one of the three.

One implementation note: **the COBS decoder must enforce a maximum decoded-frame size before it allocates the output buffer.** A maliciously- or accidentally-sized input can otherwise cause an unbounded write. The fixed-size ping-pong buffers in [02 §4.2](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) implicitly cap this, but the decoder should also explicitly refuse over-long frames and emit `ERR_PROTO`.

### 5.2 DMA-on-IDLE pattern for USART2 RX

This is the right pattern on STM32L0. Two pitfalls the firmware will hit and should be designed around now:

- **IDLE-line interrupt fires once per gap, not once per frame.** If the H7 sends two back-to-back frames with no gap (which it will, because COBS allows it), the L072 gets one IDLE for both. The driver must walk the DMA buffer looking for `0x00` framing bytes and dispatch each frame, not assume one IDLE = one frame.
- **DMA half-transfer + transfer-complete + IDLE is the canonical triple-interrupt pattern for ping-pong.** The driver in `host/host_uart.c` should be written for this from the start; retrofitting half-transfer handling later is painful.

### 5.3 P0 preempt via direct radio reset

[01 §1 R-04](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md) and [02 §2 `radio/sx1276_safety.c`](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md): "P0 preempt of in-flight P2/P3 fragments via direct radio reset (~100 µs)."

Direct radio reset is a sledgehammer. It works, but it discards the SX1276's internal state — which means whatever profile (FHSS channel, SF, CR, TX power) was loaded for the preempted frame is lost, and the P0 frame's profile must be re-applied from scratch. That's fine for latency math (the re-apply is part of the ≤5 ms request-to-air-quiet budget) but the driver design must ensure **every register write the radio depends on is re-emitted by the P0 path**, not assumed to still be there. This is a documentation discipline ("the canonical place to enumerate radio register state is `sx1276.c`") more than a code-correctness one, but skipping it leaves a class of bug where P0 frames have intermittently-wrong PHY config because the developer forgot one register.

Recommend: a single `sx1276_apply_profile_full()` function that writes **every** profile-dependent register, called by both normal-TX-prep and P0-preempt-TX-prep. No "incremental update" optimization in either path. The 100 µs cost is worth the correctness guarantee.

### 5.4 Per-frame nonce from hardware RNG (N-13)

Correct. One subtlety: the L072 TRNG peripheral has a known startup transient — early reads can be biased. The driver should discard the first ~16 bytes after enable and verify the health bits before using a value. The Semtech reference and the ST HAL both get this right; just make sure the wrapper in `hal/rng.c` doesn't optimize it out.

### 5.5 IWDG independent watchdog (N-20)

[02 §3](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) shows `iwdg_kick()` at the bottom of the main loop. Per §2.2 above, kick **after the tick handler**, not at the bottom. Otherwise a hung priority-2 handler can prevent the kick even though the rest of the system is fine.

Also: the IWDG window in [04 §6](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) defaults to 100 ms. That's reasonable in steady state but is too tight for the very first boot when SX1276 init can take 50–80 ms by itself. Either disable IWDG until after `radio_init()`, or set the initial window to 500 ms and tighten it post-init. Document which.

### 5.6 Failsafe state machine (R-11 `proto/failsafe.c`)

[01 §1 R-11](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md): "≤500 ms failsafe trip." The plan correctly puts this in `proto/` rather than `host/` — the L072 should trip failsafe on its own evidence (last RX ControlFrame age) rather than waiting for an H7 instruction, because the H7 might be the thing that's failed.

What's not specified: **what does the L072 do once it has tripped failsafe?** Options:
- Stop transmitting (handheld scenario).
- Transmit a special `FAILSAFE_TRIPPED` frame on every slot until link recovers (tractor scenario).
- Both, depending on a build-time `ROLE` flag or a runtime config.

The current docs imply but don't state. Specify before coding.

### 5.7 Three-profile per-frame PHY swap (R-03)

The plan calls for three profiles (control / telemetry / image). Worth adding a fourth: **emergency-beacon profile** (SF12/BW125/CR4/8/+20dBm). N-30 needs this and it's already implied; making it a fourth named profile rather than a special case keeps the profile-selection code regular.

---

## 6. Specific risks the docs don't yet address

### 6.1 Coexistence with the H7's `Serial3` driver

The H7-side serial driver (mbed OS or Arduino-mbed) has its own RX buffer and its own latency characteristics. At 921600 baud the H7 must service its UART at sustained ~92 kB/s without dropping bytes; whether the mbed `Serial3` driver does this reliably under load is **untested for this project**. Recommend a Phase 2 work item to characterize this with an artificial sender on the L072 side, before assuming the link can sustain rate.

### 6.2 SX1276 errata that bite custom drivers

The SX1276 datasheet has ~12 errata that the Semtech reference driver works around silently. A "slimmed" port risks dropping the workarounds. Specifically watch:
- Errata 2.1 (sensitivity optimization with 500 kHz BW)
- Errata 2.3 (receiver spurious reception of LoRa signal)
- The IQ-inversion register settings for downlink-style RX

Recommend the Phase 2 SX1276 driver port include a checklist comment listing each errata and how the driver addresses it. This is the kind of thing that disappears in a year-long firmware effort.

### 6.3 LoRa Carrier-board-vs-internal antenna path

The Murata SiP routes its RF output through the carrier; some carrier revisions have a u.FL connector and some have a chip antenna. The TX-power-vs-DAC calibration (N-28) will be different for each. Flag that the Phase 6 calibration table is per-(unit × carrier-revision), not just per-unit.

### 6.4 Upgrade path for the over-the-air protocol version

[04 §8](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md) discusses UART protocol versioning but not over-the-air protocol versioning. If a v25.1 release adds a field to the ControlFrame, how does a v25.0 base station handle it? The 4-bit `proto_version` in [00 §3](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md) implies versioning exists but the upgrade-mixed-fleet semantics are undefined. Worth a section in [LORA_PROTOCOL.md](../../DESIGN-CONTROLLER/LORA_PROTOCOL.md) (out of scope for this folder, but flag it from here).

### 6.5 No explicit threat model for malicious LoRa traffic

The plan mentions AES-GCM, replay window, signed firmware (N-14). It doesn't enumerate what the L072 does when it receives **a flood of garbage frames** intended to deny service:
- AES-GCM verification on every received frame costs CPU.
- A flood at SF7 can be ~250 frames/s.
- Without rate-limiting, this crowds out real ControlFrames in the L072's processing path.

Recommendation: a per-source-address rate limiter (token bucket, ~5 KB code, ~256 B RAM) that drops frames before AES-GCM verification once the budget is exceeded for that address. Address authenticity isn't established pre-decrypt, so the limiter is per-(SF × channel × source-address-claimed) — which is good enough to localize a flood.

### 6.6 No statement about what the L072 logs

The plan has stats counters (N-16) and packet capture (N-18) but no "log buffer." A small ring of textual one-line events (e.g. "FAULT: rx_dma_overrun at t=12345.6 ms"), accessible via a `LOG_DUMP` UART command, costs ~2 KB RAM and is invaluable for field diagnostics that don't fit the histogram model. Recommend adding to Phase 5.

---

## 7. Things the plan gets unusually right (worth keeping)

A few things should be highlighted because they're easy to lose track of as the implementation proceeds:

1. **The reversibility-through-Phase-4 escape hatch.** Most embedded projects pretend they can revert; this one designs `radio/` and `proto/` to make it actually possible. Don't let that erode.
2. **Phase 0 = qualify the bootloader pipeline, not "write code."** The discipline of proving the flashing path on vendor binary first is what makes Phase 1 brick-recovery tests meaningful.
3. **The W1-7 risk gate** (read SX1276 registers from custom binary, confirm non-zero defaults). Single best "go/no-go" gate in the whole roadmap.
4. **The two-stable-interfaces framing.** Naming exactly two contracts that can't break casually — over-air format + host UART — keeps everything else refactorable, which is the right tradeoff for a 6-month-plus effort.
5. **"Cooperative scheduler, no RTOS" for v25 launch.** This will be questioned by every reviewer who's used to FreeRTOS. Hold the line. RTOS pays for itself when you have ≥4 truly independent activities and ≥64 KB RAM; this firmware has neither.

---

## 8. Recommended changes summary

In rough priority order:

1. **Update [00 §5](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md)** effort estimate to 8–12 weeks for launch firmware.
2. **Specify the event-queue priority discipline** in [02 §3](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md) (multi-level, per-handler runtime cap, IWDG kick after tick).
3. **Add safe-mode auto-baud + persisted-config exclusion** in [04 §2.4](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md).
4. **Add `tests/host_native/` mock-radio build** ([02 §6](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md)) and run it in CI to enforce the radio/proto split.
5. **Move N-09 (deep sleep) into launch set; defer N-06 (quality-aware FHSS) to Phase 5** ([01 §10](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md), [03 Phase 4/5](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md)).
6. **Make W4-8 (regulatory channel list) a hard prerequisite for W4-2 (N-06)** in [03 Phase 4](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md), with an explicit dependency.
7. **Specify config persistence model + add `CFG_COMMIT`** in [04 §6](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md).
8. **Add reset-cause to `BOOT_URC`** ([04 §2.3](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md)).
9. **Add a 24-hour rolling-PER acceptance test** to [03 Phase 3 exit criteria](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md).
10. **Specify failsafe-tripped behavior** (handheld vs. tractor role) in `proto/failsafe.c` design.
11. **Single canonical build system** ([02 §5](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md)).
12. **Update Flash budget** to reflect realistic GCM size (~+3–4 KB).
13. **Add per-source rate limiter** (§6.5) to protect against LoRa flood DoS.

None of these change the strategic direction. Items 1–6 are the ones I'd want done before Phase 1 starts; the rest can land as the relevant phase begins.

---

## 9. Verdict

**Approve Method G as committed.** The decision record is honest, the architecture is properly scoped for the hardware, the brick-recovery design is the most thorough I've reviewed for an MCU-firmware project of this size, and the bring-up roadmap is gated on capability rather than calendar.

The plan's biggest risk is **not technical but motivational** — by Phase 4 the firmware will be usable enough to launch on, and it will be tempting to ship before the soak tests, the regulatory channel list, and the mixed-fleet protocol-version handling are buttoned up. The roadmap's Phase 4 exit criteria are strong; the discipline to actually wait for them is what determines whether v25 ships once or ships repeatedly.

> **Method G is the right call. The plan is ~85% there. Tighten the event-queue contract, harden the safe-mode listener, defer quality-aware FHSS, and treat the 4–6 week estimate as marketing copy rather than a planning input. Then go build it.**
