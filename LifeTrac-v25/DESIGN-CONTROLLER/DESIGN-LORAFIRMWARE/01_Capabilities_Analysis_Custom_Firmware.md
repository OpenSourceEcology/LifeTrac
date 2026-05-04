# 01 — Capabilities Analysis: What Custom L072 Firmware Unlocks

**Date:** 2026-05-04
**Status:** Analysis — feeds requirements into [02](02_Firmware_Architecture_Plan.md) and the bring-up roadmap [03](03_Bringup_Roadmap.md).
**Author:** GitHub Copilot (Claude Opus 4.7)
**Scope:** Enumerate everything that becomes possible *because* we own the L072 firmware, beyond simply re-implementing the SPI-era plan.

---

## 0. How to read this document

Capabilities are graded by:

- **Recovery:** "R" = recovers a feature that was already in the v25 plan but was lost when SPI access turned out to be impossible. "N" = new — was not previously planned.
- **Cost:** S = Small (≤1 week of firmware work), M = Medium (1–2 weeks), L = Large (2+ weeks).
- **Value:** ◆ = nice-to-have, ◆◆ = clearly worth doing, ◆◆◆ = changes what the platform can do safely / legally / commercially.
- **Phase:** which [bring-up roadmap](03_Bringup_Roadmap.md) phase it lands in.

A capability is *only* recommended if it (a) provides clear safety, performance, regulatory, or operational value to LifeTrac, and (b) doesn't crowd the L072 RAM/Flash budget defined in [02 §4](02_Firmware_Architecture_Plan.md).

---

## 1. Recovered SPI-era features (mandatory, not optional)

These are the features the v25 protocol design depends on. They must all land in Phase 2–4 of the roadmap.

| ID | Capability | Cost | Value | Phase |
|---|---|---|---|---|
| R-01 | Per-frame FHSS, 8 channels, register-level retune <1 ms | M | ◆◆◆ | 3 |
| R-02 | Adaptive SF ladder, per-frame SF selection (SF7..SF12) | S | ◆◆◆ | 3 |
| R-03 | Three-profile per-frame PHY swap (control / telemetry / image) | M | ◆◆◆ | 3 |
| R-04 | P0 preempt of in-flight P2/P3 fragments via direct radio reset (~100 µs) | S | ◆◆◆ | 3 |
| R-05 | Custom 16 B ControlFrame on the air | S | ◆◆◆ | 2 |
| R-06 | AES-128-GCM payload, replay window | S | ◆◆◆ | 2 |
| R-07 | No-ACK / no-retry semantics | S | ◆◆◆ | 2 |
| R-08 | ≤25 ms per-fragment airtime cap (L1) | S | ◆◆◆ | 3 |
| R-09 | ≤5 ms per-frame retune (R-7) | S | ◆◆◆ | 3 |
| R-10 | 50 ms / 20 Hz ControlFrame cadence with low jitter | S | ◆◆◆ | 3 |
| R-11 | Failsafe heartbeat (≤500 ms trip, L3) | S | ◆◆◆ | 2 |
| R-12 | KISS framing on the air (or a binary equivalent we choose) | S | ◆◆ | 2 |

**These do not require custom firmware to *exist* in the v25 design — they require it to *work on this hardware*. Method G is the only path that lands them.**

---

## 2. Performance improvements newly possible

Because the radio is one MCU away from the application instead of two, with a bus we control end-to-end, we can do things the H7-side host code never could.

### N-01 — Sub-millisecond per-frame radio control loop  ◆◆◆ (S, Phase 3)
The L072 talks to the SX1276 over a dedicated, idle internal SPI bus at 8 MHz. A typical "set frequency, set SF, set CR, set TX power, write FIFO, trigger TX" sequence completes in **~300–500 µs**. The H7's contribution is one UART burst (~0.5 ms at 921600 baud for a 50 B frame). Total command-to-air = ~1 ms, vs. ~10–20 ms for AT and ~5 ms for the original SPI plan (which would have gone over the H7's slower SPI bus shared with other devices).

This headroom is what allows N-04 (LBT) and N-06 (channel-quality FHSS) to fit inside the 50 ms slot.

### N-02 — Hardware-timed TX scheduling (TIMx + TX trigger)  ◆◆ (M, Phase 4)
The SX1276 can be armed in standby, then triggered on a hardware timer interrupt. Pairing this with the L072's `TIM2` 32-bit timer gives us a **TX time-of-air anchor accurate to the µs**. Useful for:

- TDMA-style slot alignment between handheld and tractor (cooperative collision avoidance with the base station).
- Synchronising LoRa TX to non-LoRa events (e.g. starting transmission on a hydraulic-state-change interrupt rather than waiting for the next 50 ms tick).
- Sub-slot fairness inside the 50 ms cadence (e.g. "start ControlFrame at t+5 ms, leave 0..5 ms free for an opportunistic preempt").

### N-03 — RX windows on µs precision, not ms  ◆◆ (S, Phase 3)
DIO0/DIO1/DIO2 interrupts wired to the L072's NVIC give per-symbol RX-done / preamble-detect / sync-word interrupts. We can open and close RX windows with sub-symbol precision — important for a duty-cycled receiver (N-09 below) that wants to wake only when there's actually a preamble on the channel.

### N-04 — Listen-before-talk (LBT) with µs-budget CCA  ◆◆◆ (M, Phase 4)
With register-level access we can run a **clear-channel assessment** in ~150–300 µs (read RSSI register over SPI a few times, compare against a noise-floor estimate the firmware maintains per channel). At ~500 µs per CCA we can afford to LBT before *every* P0 ControlFrame transmission and still hit the 50 ms slot. This is **only practical with custom firmware** — AT modems either don't expose CCA at all or expose it as a 10+ ms command.

Operational value: dramatically reduces self-collisions on busy channels and provides one of the cleanest paths to FCC §15.247 polite-spectrum-use compliance. Also helps when more than one LifeTrac is operating in the same physical area.

### N-05 — Real-time spectrum scan / waterfall  ◆◆ (M, Phase 4)
Same RSSI register, swept across the band during otherwise-idle slots. Builds a per-channel noise-floor estimate that feeds N-06 (channel-quality FHSS) and N-15 (interference flagging in telemetry). Maintenance teams get a cheap "is the bench radio environment clean today?" check.

### N-06 — Channel-quality-aware FHSS  ◆◆◆ (M, Phase 4)
Instead of cycling through 8 channels in fixed order, the firmware maintains a per-channel quality score (RSSI floor, recent CRC pass rate, recent CCA-busy rate) and **biases the hop sequence away from chronically dirty channels** while still satisfying §15.247 (uniform statistical distribution over ≥50 channels can be retained by using all 50+ channels with weighted-but-non-zero probability instead of the original 8-fixed scheme).

Outcomes:
- Lower PER in noisy environments (workshop with VFDs, nearby Wi-Fi, etc.).
- Same regulatory position as the 8-fixed scheme, arguably stronger.
- Per-link telemetry showing which channels are being avoided is genuinely useful field-debugging data.

### N-07 — Per-frame TX-power adaptation  ◆◆ (S, Phase 4)
Custom firmware can read RX RSSI on the *receive* side and feed back a "target TX power" hint in the ack-less ControlFrame's reverse direction. We then back off TX power when the link is strong, saving ~1.5 W of average draw on the tractor LoRa rail and reducing self-interference for nearby units. The handheld benefits even more because its battery life depends on it.

### N-08 — Symbol-level CR (coding rate) hopping  ◆ (S, Phase 4)
Same idea as adaptive SF but cheaper: switch CR4/5 ↔ CR4/8 per frame based on recent CRC error rate. Cheaper than SF changes (no airtime jump) and finer-grained robustness control.

---

## 3. Power & duty-cycle improvements

### N-09 — L072 deep-sleep RX window scheduling  ◆◆ (M, Phase 5)
The L072 can drop to **STOP mode @ ~1.5 µA** between scheduled RX windows, woken by RTC alarm or DIO0 (preamble-detect) interrupt. The SX1276 itself drops to ~200 nA in sleep. For the handheld controller — which does not need 20 Hz cadence in idle — this gives a 100×+ idle-power reduction over today's "always-on" plan. For the tractor it's less dramatic but still meaningful between operator commands.

This is the single biggest operational improvement: handheld battery life can credibly stretch from "a shift" to "a multi-day deployment without recharge" if the firmware is power-aware end-to-end.

### N-10 — Scheduled wake-on-LoRa for telemetry beacons  ◆ (S, Phase 5)
Tractor-side firmware emits an opportunistic beacon every Ts seconds and the handheld firmware wakes only for that window. Wraps N-09 into a usable application pattern.

### N-11 — Whole-modem deep sleep with H7-driven wake  ◆ (S, Phase 5)
H7 can park the modem when the tractor is parked + powered-off-by-operator, then wake it via NRST + a "are you there?" UART ping. Useful for unattended monitoring scenarios.

---

## 4. Security improvements

### N-12 — Move AES-GCM into the L072 (optional)  ◆ (M, Phase 4)
With hardware AES on L072CZ rev Y+ silicon, GCM throughput is comfortable. Moving encryption into the L072 means the H7 ↔ L072 UART carries plaintext only inside the box, so a compromised H7 application can't accidentally exfiltrate the LoRa key. Note: this is a marginal improvement in the LifeTrac threat model since the H7 is the trusted controller. **Default remains: H7 encrypts, L072 transports ciphertext.** This is an opt-in profile.

### N-13 — Per-frame AES-GCM nonce derived from L072 hardware RNG  ◆◆ (S, Phase 2)
Eliminates one of the classic v25 nonce-reuse risks (host clock skew at boot causing repeated nonces). The L072 has a true RNG peripheral; one read per frame is ~5 µs.

### N-14 — Signed firmware images & secure boot stub  ◆◆ (L, Phase 6)
ED25519 signature verification at boot using the L072's Flash-resident public key. Without it, a stolen Murata module can be re-flashed to anything. With it, only signed binaries run. ED25519 verify on M0+ is ~80 ms — acceptable at boot.

This is the prerequisite for shipping commercial / community-distributed binaries with confidence.

---

## 5. Diagnostics & observability improvements

### N-15 — Per-frame radio metadata in telemetry  ◆◆ (S, Phase 3)
RSSI, SNR, frequency error, CRC status, packet length, channel index, SF used, CR used, TX power used — all available from SX1276 status registers. The L072 attaches them to the URC frame the H7 reads. The H7 forwards a subset to the base station. Operations teams get a real picture of link health without external SDR equipment.

### N-16 — Histogram counters in firmware  ◆ (S, Phase 5)
Per-channel frame counts, per-SF frame counts, CRC error histogram, queue depth histogram, latency-bucket histograms — accumulated in the L072 and dumped on demand to the H7 via a `STATS` UART command. Costs ~1 KB RAM, negligible CPU. Invaluable in the field.

### N-17 — On-demand register dump  ◆ (S, Phase 1) (already a debug bring-up tool)
A `REG_DUMP` UART command that returns the full SX1276 register set. First diagnostic tool we'll write; never goes away.

### N-18 — In-firmware packet capture ring  ◆ (M, Phase 5)
Last 32 frames (raw bytes + radio metadata) kept in a circular buffer in L072 RAM. Dumped to H7 on `PCAP_DUMP`. Equivalent of a flight-data-recorder for the radio link. ~2 KB RAM.

### N-19 — RSSI watchdog → fault flag  ◆ (S, Phase 4)
Fault if noise floor on any channel exceeds threshold for >N seconds. Surfaces "someone is jamming us" or "a VFD just turned on" without the operator noticing slow degradation.

---

## 6. Reliability & failsafe improvements

### N-20 — Independent watchdog inside L072  ◆◆ (S, Phase 2)
L072 IWDG (independent watchdog) reset if the main loop misses a tick. Independent of the H7's watchdog. Combined with H7's "modem heartbeat" check, gives a two-level guarantee: either MCU detecting the other's hang triggers a recovery.

### N-21 — H7-driven hard reset path  ◆◆ (S, Phase 1)
H7 GPIO → Murata NRST. Already wired (we proved it on the bench via `gpio163`). Firmware just has to expect it and recover cleanly.

### N-22 — UART safe-mode command (brick-resistance)  ◆◆◆ (S, Phase 1)
A single-byte magic sequence on UART within the first 500 ms of L072 boot causes the L072 firmware to call `SystemMemoryBoot()` directly, jumping to the ROM bootloader without needing BOOT0 toggling. This is the cheapest known **brick-recovery insurance** and must be in place from Phase 1, before any custom logic is written. See [04_Hardware_Interface_and_Recovery.md](04_Hardware_Interface_and_Recovery.md) §3.

### N-23 — Brown-out detection + clean shutdown  ◆ (S, Phase 5)
L072 BOR (brown-out reset) at 2.7 V; firmware queues a "BROWNOUT" telemetry record into a small persistent buffer (backup register or option byte) so the next boot can report the fault.

### N-24 — Last-frame replay on link recovery  ◆ (S, Phase 4)
After a failsafe trip and link recovery, the L072 has the last-known-good ControlFrame in RAM and can re-transmit it once before the H7's normal cadence resumes. Smooths the recovery transient.

---

## 7. Operational / fleet improvements

### N-25 — Field-updatable firmware via H7 (without USB)  ◆◆◆ (L, Phase 6)
The H7 already drives BOOT0/NRST and can flash the L072 over UART using the STM32 ROM bootloader protocol. Build that into normal H7 firmware and the field-update story becomes "operator runs an update from the tablet UI" instead of "send the unit back, plug into a Windows PC, run Arduino IDE." This is a significant deployability win.

### N-26 — A/B firmware slots in L072 Flash  ◆◆ (M, Phase 6)
Split the 192 KB Flash into two 80 KB slots + a 32 KB bootloader/recovery region. Update flashes the inactive slot, sets a "try B" flag, reboots; on successful boot+heartbeat the slot is committed, on failure it reverts. Combined with N-25, this is a **safe** field-update story.

### N-27 — Manufacturing test mode  ◆ (S, Phase 6)
A factory `MFGTEST` UART command runs a fixed sequence of self-tests (TX at known power on each channel, expected RX on a paired test fixture, assert pass/fail). Reduces "did the modem make it through assembly?" check from 30 minutes of manual probing to 30 seconds.

### N-28 — Per-unit calibration storage  ◆ (S, Phase 6)
Store per-unit TX-power-vs-DAC calibration table in L072 option bytes (or a reserved Flash page). Compensates for the SiP-to-SiP variation in the SX1276's PA gain (typically ±1.5 dB). Squeezes the last bit of link-budget margin.

### N-29 — Stable serial number / hardware ID exposed via UART  ◆ (S, Phase 2)
L072's 96-bit unique device ID surfaced via a `UID` UART command. Used by the base station's pairing logic and for telemetry attribution.

---

## 8. Application-layer improvements newly possible

### N-30 — Dual-mode emergency beacon  ◆◆ (M, Phase 5)
On a failsafe-trip event, the L072 autonomously transmits a low-data-rate (SF12/BW125) emergency beacon at full TX power on a fixed beacon channel, **independently of any H7 instruction**. Range typically 5–10× the normal control link. Designed to find a stuck/disabled tractor in a pasture even when its main controller has crashed. The L072 has its own copy of the unit ID and last-known operating mode, so the beacon carries useful information.

This capability is **only possible because the L072 firmware can run when the H7 is hung or the H7 power rail has dropped** — an external SX1276 module would lose its host.

### N-31 — LoRa-only "wake-up tractor" command  ◆ (M, Phase 5)
With N-09 (deep sleep) and N-10 (scheduled wake), the handheld can send a wake command to a parked tractor. The L072 receives, validates, then asserts a GPIO that brings up the H7 / X8 power rail. Tractor goes from sleeping (~10 mA) to running (~3 A) without the operator walking over.

### N-32 — Mesh / repeater mode for multi-tractor sites  ◆ (L, Phase 7)
Multiple LifeTracs in radio range can opportunistically forward each other's traffic to the base station. Doesn't need to be perfect — even simple "I heard X's frame, retransmit if base didn't ack within Δt" gives substantial range extension. Out of scope for v25 launch but enabled by owning the firmware.

### N-33 — Autonomous "lost link, return to dock" coordination  ◆ (L, Phase 7)
On extended link loss, the L072 keeps trying to reach the base station independently of the H7 (which may itself have failed over to autonomous mode). This is the radio-side anchor of a multi-layer "tractor never gets stranded silent" guarantee. Out of scope for v25 launch.

### N-34 — Time synchronisation broadcast  ◆ (S, Phase 5)
Base station broadcasts periodic time beacons; tractors and handhelds discipline their RTCs from them. With N-02 (HW-timed TX), these beacons can be accurate to ~10 µs, good enough for log-correlation across the fleet without GPS on every unit.

---

## 9. Capabilities explicitly *not* pursued

- LoRaWAN MAC compliance — out of scope; we are not joining any public network.
- Sigfox — out of scope; the SX1276 supports it but we have no use case.
- FSK / OOK / GFSK modulations — supported by SX1276 but no v25 use case.
- Continuous-wave TX for testing — only as a temporary `MFGTEST` mode, not a runtime feature.
- BLE / Wi-Fi — the L072 doesn't have these radios; not on the table.

---

## 10. Recommended commit list for v25 launch

Sorting by value × cost ratio, the following capabilities are recommended for the v25 launch firmware. Anything tagged `(opt)` is a fast-follow if launch schedule pressure appears.

**Mandatory (bring-up roadmap Phase 2–4):** R-01 through R-12, N-13, N-15, N-20, N-21, N-22.

**Strongly recommended for launch (Phase 4):** N-04 (LBT), N-06 (quality-aware FHSS), N-07 (TX-power adapt), N-19 (RSSI watchdog), N-24 (last-frame replay).

**Recommended for launch if Phase 5 fits (otherwise fast-follow):** N-09 (deep sleep), N-10 (scheduled wake), N-30 (emergency beacon).

**Post-launch (Phase 6+):** N-02, N-05, N-08, N-11, N-12, N-14, N-16–N-18, N-23, N-25–N-29, N-31, N-34.

**Long-horizon platform features (Phase 7+):** N-32 (mesh), N-33 (autonomous).

The launch set fits comfortably inside the L072 Flash/RAM budget per [02 §4](02_Firmware_Architecture_Plan.md). Post-launch features are added incrementally as A/B updates (N-26).

---

## 11. One-line summary

> **Owning the L072 firmware doesn't only recover the SPI-era v25 protocol — it unlocks listen-before-talk, channel-quality-aware FHSS, sub-millisecond radio control loops, deep-sleep handheld operation, an autonomous emergency beacon, signed firmware, and field-updatable A/B slots. The launch firmware should aim at "all R-* + the strongly-recommended N-*" set; the rest is fast-follow on a no-deadline schedule.**
