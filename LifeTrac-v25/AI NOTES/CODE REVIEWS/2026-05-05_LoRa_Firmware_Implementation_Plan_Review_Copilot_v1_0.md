# LifeTrac v25 LoRa Firmware Implementation Plan Review - GitHub Copilot v1.0

Date: 2026-05-05
Reviewer: GitHub Copilot
Scope: `LifeTrac-v25/DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/`, plus the existing shared LoRa protocol helpers that the plan proposes to reuse or migrate.

## Executive Summary

The Method G direction is strategically sound. Given the confirmed Max Carrier reality that the host cannot reach the SX1276 SPI bus directly, custom firmware on the Murata STM32L072 is the only path that preserves the original v25 goals without adding external LoRa hardware: per-frame radio control, low host-to-radio latency, P0 preempt, custom framing, and a single hardware BOM.

The plan is not yet implementation-ready, though. The biggest risks are not small C details. They are ownership boundaries and system-level assumptions: which nodes run custom L072 firmware, which MCU owns crypto and replay, whether the radio firmware is a thin modem or a protocol endpoint, and whether the regulatory channel plan is actually compatible with the existing 8-channel code. Resolve those before creating the `firmware/murata_l072/` source tree.

At review time I did not find an actual `DESIGN-CONTROLLER/firmware/murata_l072/` implementation. The proposed-code review below therefore focuses on the current reusable LoRa helpers in `firmware/common/lora_proto/` and `base_station/lora_proto.py`, because those are the concrete code paths the plan references as migration inputs.

## Findings

### Blocker 1 - Method G is specified for the Portenta Murata, but the system has three Murata node classes

The new design folder targets the STM32L072 inside the Murata SiP on the Portenta Max Carrier, and [04 Hardware Interface & Recovery](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md#L171) makes the H7 responsible for BOOT0, NRST, golden-binary flashing, and recovery. That works for the tractor only if the H7 owns those pins, and it may work for the base only if either the H7 is brought back into the base design or the X8 can directly control both BOOT0 and NRST. It does not yet cover the optional MKR WAN 1310 handheld, which also uses a Murata module and must speak the same custom air protocol if the stock AT firmware is removed from production.

This matters because the current canonical system is handheld <-> tractor <-> base, not one Portenta modem. A custom L072 binary on the tractor alone cannot preserve per-frame FHSS, adaptive PHY, and raw P2P semantics if the base or handheld remain on stock MKRWAN AT firmware. The plan should add an explicit per-node table before Phase 1:

| Node | Host MCU/OS | L072 host path | BOOT0/NRST recovery owner | Golden binary storage | Method G status |
|---|---|---|---|---|---|
| Tractor Portenta Max Carrier + X8 | H747/X8 | `Serial3` / `/dev/ttymxc3` | H7 or X8, proven | H7 flash or X8 eMMC | required |
| Base Portenta Max Carrier + X8 | X8, currently no base H7 sketch in old plan | `/dev/ttymxc3` likely | TBD | X8 eMMC likely | required |
| Handheld MKR WAN 1310 | SAMD21 | MKR modem UART | TBD via MKRWAN updater path | SAMD flash? external? | required or hardware changes |

Do not treat Phase 0 as green until all production node classes have a proven flash and recovery path, not just the two Portenta bench boards.

### Blocker 2 - Regulatory compliance is still unresolved, while the proposed FHSS code hardcodes 8 US channels

The Method G decision record says production FHSS compliance requires revisiting channel count: [00 Decision](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md#L81) notes that >=50 channels with <=0.4 s dwell satisfies FCC 15.247 and that v25 currently targets 8 channels. The capabilities analysis then proposes quality-aware FHSS over all 50+ channels as the compliance-preserving path in [01 Capabilities](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md#L74).

The actual reusable code is not ready for that. The C helper uses a fixed 8-entry permutation and 902 MHz + 3.25 MHz spacing in [lora_proto.c](../../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c#L196) and [lora_proto.c](../../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c#L210). The Python mirror does the same in [lora_proto.py](../../DESIGN-CONTROLLER/base_station/lora_proto.py#L302) and [lora_proto.py](../../DESIGN-CONTROLLER/base_station/lora_proto.py#L314). The new host config advertises `fhss_channel_mask` in [04 Hardware Interface](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md#L196), but no current code consumes a mask, region profile, variable channel count, dwell ledger, or weighted uniformity rule.

Before firmware coding starts, pin the regulatory strategy per region:

- If using FHSS under FCC 15.247, implement the required channel count and dwell accounting in both firmware and tests.
- If relying on 500 kHz digital modulation for some profiles, state which profiles qualify and what happens to SF/BW fallback profiles that do not.
- If 8-channel operation is only a bench or experimental mode, mark it as such and prevent it from being the default production profile.

### Blocker 3 - The A/B slot and bootloader memory map does not currently fit its own numbers

The architecture plan lists two roughly 96 KB slots plus a 4 KB boot/recovery/config area in [02 Firmware Architecture](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md#L122). Its map then places Slot B at `0x08018000` and the config page at `0x08030000` in [02 Firmware Architecture](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md#L164). On an STM32L072CZ with 192 KB flash, `0x08030000` is the end of flash, not the start of another 4 KB config page. The displayed `0x08030400` end marker is beyond 192 KB.

There is also a deeper design conflict: [01 Capabilities](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md#L163) describes two 80 KB slots plus a 32 KB bootloader/recovery region, while [02 Firmware Architecture](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md#L136) describes 96 KB mirrors and only about 4 KB of resident boot/recovery/config. A 4 KB resident region is not credible once Phase 6 adds A/B slot selection, image validation, rollback state, and ED25519 signature verification from [03 Roadmap](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md#L160).

Fix this before writing the linker script. Pick one of these models and update all docs:

- Simple launch image, no A/B, no signed boot: use nearly all flash, keep ROM bootloader recovery only.
- Small resident boot manager plus one app slot: easier, no rollback slot.
- True A/B signed update: reserve a larger boot region and reduce slot sizes to fit inside 192 KB with config pages inside the valid flash range.

Also document vector-table relocation and interrupt handoff explicitly for non-zero app slots.

### Blocker 4 - Crypto and replay ownership is contradictory

[01 Capabilities](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/01_Capabilities_Analysis_Custom_Firmware.md#L107) says the default remains H7 encrypts and the L072 transports ciphertext. [04 Hardware Interface](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md#L203) agrees by defaulting `crypto_in_l072` to false. But Phase 2 of the roadmap requires porting AES-GCM and replay to the L072 and deriving per-frame nonce material from the L072 RNG in [03 Roadmap](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md#L74).

Those are different security architectures:

- H7-owned crypto means the UART carries ciphertext, the L072 is a radio scheduler/modem, and the L072 does not know LoRa session keys or replay windows.
- L072-owned crypto means the UART carries plaintext inside the enclosure, the L072 owns keys, nonce construction, replay state, and tamper rejection, and the H7-to-L072 ABI changes meaningfully.

This decision affects key provisioning, field recovery, H7/base host code, tests, and safety analysis. Choose one launch default. If L072 crypto is optional post-launch hardening, move AES-GCM/replay out of Phase 2 exit criteria and make `TX_FRAME` semantics unambiguously "already authenticated ciphertext" for launch.

### High 5 - The new COBS host ABI is not connected to the current KISS-based codebase

The new H7-to-L072 host protocol is COBS-framed in [04 Hardware Interface](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md#L41), and the roadmap asks for `host/host_uart.c` plus `host_cobs.c` in [03 Roadmap](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md#L71). The existing code, however, is KISS-centric: the shared C helper exposes `lp_kiss_encode()` and `lp_kiss_feed()` in [lora_proto.c](../../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c#L40) and [lora_proto.c](../../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c#L68); the base bridge imports and writes `kiss_encode()` in [lora_bridge.py](../../DESIGN-CONTROLLER/base_station/lora_bridge.py#L469); and the Python helper defines `KissDecoder` in [lora_proto.py](../../DESIGN-CONTROLLER/base_station/lora_proto.py#L166).

COBS for host transport and KISS for over-the-air framing can coexist, but only if the boundary is explicit:

- `host/` owns COBS and never emits KISS onto the UART ABI.
- `proto/` owns over-the-air bytes and either keeps KISS internally or replaces it with a compact binary equivalent.
- H7/base host code sends `TX_FRAME` commands over COBS, not raw KISS frames.
- `RX_FRAME_URC` carries radio metadata plus decoded on-air bytes in one documented format.

Add host-side compatibility tests before firmware implementation: COBS encode/decode, CRC failure, max payload rejection, `TX_FRAME` round-trip, and `RX_FRAME_URC` parsing in the H7/base code.

### High 6 - Host frame sizing conflicts with the planned 256-byte UART DMA buffers

The host transport has a 16-bit `payload_len` in [04 Hardware Interface](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md#L48), while the architecture budget assumes two 256-byte host UART RX DMA buffers in [02 Firmware Architecture](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md#L144). The same plan allows LoRa payloads up to roughly 250 bytes in [00 Decision](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md#L34). A `TX_FRAME` command that carries a near-maximum radio payload plus channel/SF/profile/priority metadata, host header, CRC, and COBS overhead can exceed 256 bytes.

This is a parser and reliability issue, not just a memory-budget footnote. Define a launch `HOST_MAX_INNER_LEN` and size all buffers around it, or cap radio payloads below the host transport ceiling. The COBS decoder should reject oversized frames before writing past static storage, and the H7 should get a deterministic `ERR_TOO_LARGE` rather than a silent modem reset.

### High 7 - Method G removes AT latency but does not remove LoRa airtime or half-duplex scheduling pressure

The current control decision pins SF7/BW250 because a 44-byte encrypted ControlFrame is about 46 ms on air, as recorded in [LORA_PROTOCOL](../../DESIGN-CONTROLLER/LORA_PROTOCOL.md#L99) and implemented in [lora_proto.c](../../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c#L21). That leaves only a few milliseconds inside a 50 ms control period before LBT, CCA, retune, queueing, clock drift, and any separate heartbeat traffic. Fallback rungs at SF8/SF9 do not preserve 20 Hz control; they trade latency for range.

The L072 firmware should not simply implement a generic 50 ms scheduler and declare the old SPI-era timing recovered. It needs a strict policy for the control path:

- latest P0 wins;
- stale P0 frames are replaced or dropped, never queued behind older P0;
- heartbeat is merged into ControlFrame whenever control is active, or sent at a lower idle cadence;
- telemetry and image only transmit inside explicit uplink opportunities or only when their airtime cannot delay the next P0 window;
- SF8/SF9 fallback has a documented degraded cadence or trips failsafe instead of pretending 20 Hz still holds.

The roadmap should add this timing model before Phase 3, not after first over-air success.

### High 8 - The L072 role is split between "thin modem" and "application protocol endpoint"

[00 Decision](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md#L103) says the H7 keeps application logic, the priority queue, AES-GCM by default, failsafe state machine, image fragmenter, telemetry scheduler, and pre-formed frames. But [02 Firmware Architecture](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md#L58) puts `scheduler.c`, `beacon.c`, `deepsleep.c`, and `stats.c` under an L072 `app/` layer, and [03 Roadmap](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/03_Bringup_Roadmap.md#L96) asks the L072 to implement a priority queue and 50 ms slot scheduler.

Both models can work, but mixing them is how stale control frames and split-brain failsafes creep in. Define the L072 launch contract as one of these:

- Thin deterministic radio modem: H7 submits already authenticated frames with metadata and deadlines; L072 owns only channel access, radio timing, preempt, stats, and recovery.
- Full LoRa protocol endpoint: H7 submits semantic commands/telemetry; L072 owns encryption, replay, frame construction, scheduling, and some failsafe behavior.

For v25 launch I recommend the thin modem model. It best matches the existing H7/M4 safety split and minimizes new safety-critical logic on the 20 KB RAM modem MCU.

### High 9 - The plan drops the originally recommended fork point without updating risk or schedule

The earlier Method G note recommended starting from `hardwario/lora-modem` to inherit a working STM32L0/SX1276 bring-up path. The new architecture instead says no RTOS, hand-picked HAL, no vendor SDK in tree, and a custom bare-metal event loop in [02 Firmware Architecture](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/02_Firmware_Architecture_Plan.md#L83).

Bare-metal is a reasonable final architecture, but from-scratch bring-up increases risk around the Murata pin map, RF switch control, TCXO handling, DIO mapping, bootloader jump details, and SX1276 errata. If the plan intentionally changed from "fork and strip" to "from scratch," update the cost estimate and Phase 1 gates. A safer sequence is:

1. Flash known stock firmware to prove bootloader path.
2. Build or run a known open STM32L0/SX1276 reference on the module to prove custom binary, clock, UART, SPI, RF switch, and DIO wiring.
3. Then strip toward the final bare-metal design.

### Medium 10 - Recovery design is strong, but BOOT0 control and golden-binary policy are still assumptions

The five-layer recovery model in [04 Hardware Interface](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md#L108) is exactly the right instinct. The risk is that several layers are not yet proven on every node. BOOT0 is still marked TBD in [04 Hardware Interface](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md#L16), and H7 responsibilities are binding in [04 Hardware Interface](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md#L171) before H7 host firmware changes exist.

Add Phase 0 evidence requirements for each board class: BOOT0 level capture, NRST capture, ROM bootloader ACK transcript, golden-binary reflash transcript, and one intentionally bad image recovered without SWD.

### Medium 11 - Default TX power and EIRP policy need to be per-node and per-region

The host config defaults `tx_power_dbm` to 14 dBm in [04 Hardware Interface](../../DESIGN-CONTROLLER/DESIGN-LORAFIRMWARE/04_Hardware_Interface_and_Recovery.md#L190), while earlier protocol/system docs expect +20 dBm on tractor/base and +14 dBm on handheld. The base also has an 8 dBi mast antenna in the canonical hardware plan, so legal EIRP and conducted power limits are node-specific.

Make TX power a profile selected by node role and region, not one universal default. Record conducted power, antenna gain, cable loss, and allowed EIRP in the spectrum test plan.

### Medium 12 - Documentation drift remains around PHY profiles

The shared code now sets control SF7 to BW250 and image to BW500 in [lora_proto.c](../../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c#L21) and [lora_proto.c](../../DESIGN-CONTROLLER/firmware/common/lora_proto/lora_proto.c#L25). Some planning text still describes old BW125 control blockers or SF7/BW250 image fragments that miss the 25 ms cap. The Method G folder should state the current pinned profile table directly instead of relying on cross-document archaeology.

## Strengths Worth Preserving

- The Method G pivot is the right strategic choice for a no-rush platform that values radio performance and single-BOM repeatability.
- Phase 0 as bootloader-pipeline qualification before any custom binary is excellent and should stay non-negotiable.
- The five-layer recovery plan is appropriately paranoid for a hidden MCU inside an RF module.
- Splitting `hal/`, `radio/`, `proto/`, `host/`, and diagnostics is a good source layout. Keep `host/` as the public ABI and keep `radio/` independent of the over-air protocol where possible.
- Bare-metal cooperative scheduling can fit this MCU if ISRs only enqueue events and the L072's role is kept narrow.
- The plan correctly treats spectrum testing, HIL pair testing, and soak testing as required evidence, not nice-to-have polish.

## Recommended Next Actions

1. Add a Method G system-scope amendment covering tractor, base, and handheld nodes, including each host path and recovery owner.
2. Resolve the regulatory channel plan and replace the hardcoded 8-channel helpers with region/channel-mask-aware code before production defaults are chosen.
3. Fix the flash map and decide whether Phase 6 truly needs A/B signed update on the L072 or a simpler launch recovery story.
4. Decide crypto ownership for launch and make Phase 2, host ABI, key provisioning, and replay tests match that decision.
5. Define `HOST_MAX_INNER_LEN`, UART backpressure/credit rules, and COBS parser overflow behavior.
6. Write the H7/base host-side COBS `TX_FRAME`/`RX_FRAME_URC` tests before writing the L072 parser.
7. Create `firmware/murata_l072/` only after the above contracts are pinned; start it with host-PC unit tests for COBS, CRC, command parsing, replay if applicable, and mock-SX1276 scheduling.
8. Treat Phase 1 SX1276 register dump and non-zero default-register readback as the Method G viability gate. If SPI reads all zeroes again from a custom L072 binary, stop and re-investigate hardware before building protocol features.

## Bottom Line

Proceed with Method G, but do not start by writing the whole firmware tree. First close the contracts: node coverage, regulatory channel count, flash layout, crypto ownership, host COBS ABI, maximum frame sizes, and P0 scheduling semantics. Once those are pinned, the actual L072 implementation can be quite small and clean. Without those decisions, the firmware will accumulate clever local code around unresolved system questions.
