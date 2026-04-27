# LifeTrac v25 MASTER_PLAN.md — In-Depth Review

**Reviewer:** GitHub Copilot (Claude Opus 4.7)
**Review version:** v1.0
**Date:** 2026-04-27
**Target:** [DESIGN-CONTROLLER/MASTER_PLAN.md](../DESIGN-CONTROLLER/MASTER_PLAN.md) (dated 2026-04-26)
**Cross-checked against:** [LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md), [TRACTOR_NODE.md](../DESIGN-CONTROLLER/TRACTOR_NODE.md), [HARDWARE_BOM.md](../DESIGN-CONTROLLER/HARDWARE_BOM.md), [TODO.md](../DESIGN-CONTROLLER/TODO.md), prior reviews [Copilot v1.0](2026-04-27_Controller_Master_Plan_Review_Copilot_v1_0.md), [in-depth v1.0](2026-04-27_Controller_Master_Plan_Review_v1.0.md), [Gemini 3.1 Pro](2026-04-27_MASTER_PLAN_Review_Gemini_3.1_Pro.md).

---

## Executive Verdict

**Status: AMBER — strong direction, several physics-violating numbers.**

The plan's *architecture* is good and should not be re-litigated: LoRa-only RF, Opta-as-isolated-I/O, X8 Linux as sidecar, hardware E-stop chain, USB-C-only flashing for v25. These deserve to stay pinned.

The plan's *numbers* are not yet self-consistent. Three independent reviewers (this one, Copilot v1.0, Gemini 3.1 Pro) converged on the same root cause: **§8.17 underestimates LoRa airtime by roughly 3×**, which then makes the §8.12 cadence table physically impossible. Several smaller issues compound it (replay-vulnerable E-stop, wear-out-vulnerable nonce, contradictory base-radio ownership, M7/M4 split that doesn't match the code).

This review intentionally focuses on **findings the prior three reviews under-weighted or missed**, then summarises the agreed-upon issues at the end so this single document is sufficient as the action list.

---

## 1. Findings the prior reviews under-weighted

### 1.1 LoRa airtime numbers in §8.17 are not just optimistic — they are mutually inconsistent with [LORA_PROTOCOL.md](../DESIGN-CONTROLLER/LORA_PROTOCOL.md)

`MASTER_PLAN.md §8.17` and `LORA_PROTOCOL.md` both quote ~30 ms / ~55 ms / ~100 ms for SF7/SF8/SF9 at BW 125 kHz on a 44-byte on-air frame. The Semtech LoRa airtime formula gives, for 44 payload bytes + 8-symbol preamble + explicit header + CR 4/5 + CRC + low-data-rate-optimise off:

| SF | BW | Symbol time $T_s$ | Approx. on-air time |
|---:|---:|---:|---:|
| 7 | 125 kHz | 1.024 ms | **~85–95 ms** |
| 8 | 125 kHz | 2.048 ms | **~155–175 ms** |
| 9 | 125 kHz | 4.096 ms | **~290–320 ms** |
| 7 | 500 kHz | 0.256 ms | ~22–25 ms |

In other words the *original* (pre-§8.17) BW 500 kHz figure of ~30 ms was roughly correct; when the plan was repinned to BW 125 kHz for range, the airtime numbers were not recomputed. The decision to switch bandwidths is fine; the consequences of the switch were never propagated into the cadence table.

**Implication.** At SF7/BW 125 kHz, a single 20 Hz ControlFrame stream alone consumes ~1.7–1.9 *seconds* of airtime per second — physically impossible. Adding a separate 20 Hz HeartbeatFrame stream doubles the deficit. This is *the* root finding; everything in §1.2–§1.5 below is downstream of it.

**Action.** Pick **one** coherent profile and pin it everywhere:

- **Profile A — long range, slow stick:** SF7 / BW 125 kHz, control cadence drops to **5 Hz** (200 ms tick). Heartbeat is *piggybacked* in the ControlFrame header (1 bit), no standalone heartbeat stream. ~45% control airtime, leaves headroom for telemetry. Watchdog and HEARTBEAT_TIMEOUT_MS rescaled accordingly.
- **Profile B — fast stick, shorter range:** SF7 / BW 500 kHz, control cadence stays at **20 Hz** (50 ms tick) as currently planned. ~50% control airtime. Range with the 8 dBi mast may be acceptable for OSE's typical farm-scale use. Document the range trade.

I recommend **Profile B for v25 first build** and reserve Profile A as a documented runtime fallback. The system's value proposition is a *responsive* tractor, not a long-range one — operators are typically within line of sight of their own equipment.

### 1.2 Telemetry SF is overlooked: the 64-byte TelemetryFrame at SF9/BW 250 ~250 ms means even 2 Hz telemetry costs 50% of *its own* link

`LORA_PROTOCOL.md` quotes ~250 ms for a 64-byte TelemetryFrame at SF9/BW 250 kHz. At the plan's nominal 2 Hz that is 500 ms/s of airtime on a half-duplex radio — consuming half of the channel that is *also* expected to carry control + heartbeats. The plan papers over this by saying "telemetry stays on its own SF", but a single SX1276 cannot RX two PHYs simultaneously: every retune costs settling time and creates a window where the *other* link's frames are missed.

**Action.** Add a section explicitly addressing single-radio TDD: define superframe slots (e.g. 50 ms control window / 50 ms telemetry window in a 100 ms cycle), or accept that telemetry is best-effort and may be starved during high-control-load periods. This is *not* covered by §8.17.

### 1.3 §8.18 SSR routing is correct, but the SSR's leakage current and snubber requirements are unaddressed

D1608S SSRs (typically MOSFET- or triac-based) have **off-state leakage** in the 100 µA–1 mA range. For low-impedance hydraulic-solenoid coils this is fine — the holding current is hundreds of mA. But:

- If any future revision daisy-chains a high-impedance pilot solenoid or a small relay coil, leakage may keep it partially energised.
- Some industrial DC-SSRs benefit from a parallel snubber (RC across the load) to suppress switching transients into the 1N4007 flyback. The §8.18 paragraph says "1N4007 across each coil — not optional" but says nothing about whether a snubber is needed for the SSR's own protection.

**Action.** One sentence in §8.18 stating measured/spec'd off-state leakage of the chosen D1608S part and that no snubber is required for the listed coils, or one is. This is the kind of detail that bites at field-test time, not bench time.

### 1.4 The §8.10 IPC contract is asserted but not specified

§8.10 says "X8 ↔ H747 IPC is non-blocking in the H747 direction" and "the H747 never waits on the X8 for anything". This is the right rule. But the *mechanism* is unspecified: shared SRAM region with versioned record + sequence counter? Mailbox + DMA? OpenAMP RPMsg? Each has different failure modes:

- Shared SRAM with naive read can give a torn read of a multi-field struct mid-write.
- RPMsg requires Linux-side liveness for the channel itself to stay healthy.
- A simple mailbox with single-word atomics avoids both but limits payload.

This matters because the correctness of "H747 never blocks on X8" depends entirely on the IPC pattern being chosen with that property in mind.

**Action.** Add a §8.10.1 sub-section pinning the IPC mechanism (recommendation: shared SRAM with **double-buffered, sequence-tagged** records, H747 always reads the most-recent fully-written buffer, never blocks). Add a "stale flag returns true after N ms without a fresh write" rule with N = 200 ms.

### 1.5 §8.6 key.h provisioning is a CI/dev-onboarding hazard

The flow as written:

1. Developer clones repo.
2. `key.h.example` is committed; `key.h` is gitignored.
3. Developer runs `openssl rand -hex 16`, edits `key.h`, compiles.

Failure modes the plan does not address:
- Two developers each generate a different key, fly to a remote farm, and the radios won't pair.
- The CI builder needs *some* `key.h` to compile-check `firmware/`. Either CI also generates a random key (different from production, fine) — but this must be explicit, or builds fail mysteriously.
- If `key.h` is ever accidentally committed (say, copied into another file that *is* tracked), it leaks. There is no pre-commit guard.

**Action.** Pin three things in §8.6: (a) CI generates an ephemeral random key per build, never published; (b) production key is generated **once per deployed network** and stored in a password manager owned by the operator; (c) add a `pre-commit` hook entry to grep for the production key sentinel. None of these are research; they're 30 minutes of devops.

### 1.6 §7 lists waypoint slot reservations but the wire-format slot for a `HeartbeatFrame` priority bit is unmentioned

The §1.1 fix above (piggybacking heartbeat liveness inside the ControlFrame header) requires a header bit that is not currently reserved. Today's 5-byte header has `version | source_id | frame_type | sequence_num`. Fixing this later is a hard wire-format break.

**Action.** Reserve a `flags` byte now (steal one from a free position or document the upgrade as a `version` bump to 0x02). Explicitly enumerate the bits, including a `liveness_only` flag that lets a ControlFrame double as a heartbeat when no stick movement is present.

### 1.7 §8.5 PIN-only auth gives no separation between operator and bystander

A 4–6 digit PIN on a LAN with multiple household devices is more or less equivalent to "anyone on the WiFi can drive the tractor once they observe one operator session." There is no per-user identity, so there is no audit log of *who* commanded which motion. For a piece of equipment that can kill people, this is below-bar even on a trusted LAN.

**Action.** Either (a) keep the shared PIN but require it to be **re-entered every 5 minutes of control activity** (idle-control timeout, not idle-session timeout); or (b) add a free, no-database one-step-up: WebAuthn / passkey enrolment on first connect from each device, PIN as fallback. The latter is well within FastAPI ecosystem and adds a meaningful per-device identity without running a user database.

### 1.8 §8.13 "WiFi/BT off in shipped firmware on the tractor" lacks an enforcement mechanism

The §8.13 policy is good but is currently a *firmware build flag* ( `LIFETRAC_ENABLE_DEV_RADIO`). Yocto/Linux on the X8 can still bring up `wlan0` independently of firmware. The Mbed OS Portenta sketch's BT/WiFi state and the Linux side's BT/WiFi state are separate kernels.

**Action.** Add an OS-level enforcement: a oneshot systemd unit on the tractor X8 that `rfkill block wifi bt` at boot when `/etc/lifetrac/role=tractor` and `LIFETRAC_DEV_RADIO != 1`. Belt and suspenders, but cheap.

### 1.9 §8.16 "USB-C flashing only" forecloses a useful safety property

USB-C flashing implies physical access for every update. That's good for *security* but means a tractor with a known firmware bug can't be patched until someone walks to it. For v25 that's fine. But the plan should explicitly note that the **black-box logger (§8.10 logger_service)** is the substitute mitigation — without OTA you must be able to forensically diagnose an in-field failure without re-flashing first. Make sure the logger is on the §5 critical path, not optional.

**Action.** Promote `logger_service.py` from "Pinned for v25" bullet to an explicit §5 step ("LED stand-in test does not pass until the logger has captured ≥1 hour of synthetic traffic and the post-mortem analyser can replay it"). The logger is a release gate, not a service.

### 1.10 §8.15 field-test gate doesn't measure E-stop *initiation* latency, only completion

The gate measures "E-stop from base reaches tractor in <500 ms p99, measured by oscilloscope on a dummy coil." That covers RF + decode + Modbus + coil de-energise. It does *not* measure how long it takes the operator's *click* in the browser to become an outgoing LoRa frame — which on a Python-FastAPI-asyncio stack with browser→WebSocket→MQTT→bridge→spidev can easily eat another 100–200 ms.

**Action.** Add a separate gate: **browser click → SX1276 TX-start ≤ 100 ms p99**, measured by injecting a known correlation marker on the click and capturing on the radio's DIO. If this fails, the §1.1 channel-saturation problem makes the missed-deadline failures invisible because the operator already perceives the system as laggy.

---

## 2. Smaller / editorial findings (new)

| # | Where | Issue | Fix |
|---|---|---|---|
| E1 | §1 (in-scope) | "operator-browser ↔ base-station link" is described as "Ethernet/WiFi" but §8.13 says base may be either. The two should reference each other. | Add `(see §8.13)` cross-link. |
| E2 | §2 system diagram | Still shows "Arduino Opta WiFi*" footnote; the asterisk explanation is below but easy to miss. | Move the "WiFi/BLE disabled" note immediately under the diagram, not after §3. |
| E3 | §3 hardware table | Lists "Adafruit BNO085" but §8.10 also says BNO085. The repo elsewhere references **BNO086** (newer revision, drop-in pinout). | Reconcile to BNO086 throughout. |
| E4 | §4 implementation tree | `firmware/tractor_h7/` is documented as "X8's onboard H747 co-MCU" — this is correct but the directory name still implies a separate H7 board. | Rename to `firmware/tractor_h747/` or `firmware/tractor_realtime/` before any production tag. |
| E5 | §5 step 3 | Lists 9 readiness blockers from the 2026-04-26 review. Several are now stale or ambiguous (e.g. "ControlFrame size mismatch (15 vs 16 bytes)" — has it been resolved?). | Each blocker should have a `[ ]`/`[x]` status next to it, updated in-place when fixed. |
| E6 | §7 reservations | Reserved opcodes 0x40–0x44 + 0x50 are listed. The reservation is *only* enforced by being documented here; the protocol parser doesn't reject unknown opcodes from unauthorized sources. | Add one sentence: "Receivers MUST silently drop reserved opcodes from any source other than future `SOURCE_AUTONOMY` (0x04) until §7 is reversed." |
| E7 | §8.2 fallback | "If SPI-from-Linux ever proves flaky" — fine, but no measurable trigger. | Add: "Trigger criterion: >0.5% missed IRQs over a 1-minute window in `lora_bridge.py` debug counters, or >5 ms p99 IRQ-to-handler latency. Either triggers the §8.2 fallback decision." |
| E8 | §8.4 | Says "browser does not speak MQTT". Good. But says nothing about **WebSocket reconnection** behavior — what happens to control state when the WS drops? | Pin: on WS drop, the bridge MUST send `CMD_ESTOP` from `SRC_BASE` (or a "base disconnected" command that the tractor treats as base-source no-op + re-arbitrate). |
| E9 | §8.7 | "Exactly one tractor and one base per LoRa network." Fine, but how is this *enforced* on RX? Currently AES-GCM keying is per-network, not per-pair; two tractors sharing a key would interleave. | Pin: each tractor has a `tractor_id` (1 byte) included in telemetry header and ControlFrame target field. Multi-tractor support is then a wire-format-compatible upgrade. |
| E10 | §8.12 | Cadence table says "Opta watchdog timeout 200 ms" but TRACTOR_NODE.md and the active Opta code may say something else. | Pure docs-alignment task; pin once, propagate. |
| E11 | §8.14 | "PPS where the breakout exposes it, NMEA-derived otherwise" — the SparkFun Qwiic NEO-M9N (GPS-15733) **does not break out PPS** on the Qwiic connector. So in practice the system uses NMEA-only timing. | Either (a) accept NMEA-only and remove the PPS aspiration, or (b) spec an additional jumper from the NEO-M9N PPS pad to a free X8 GPIO. |
| E12 | §8.17 | "Adaptive SF7→SF8→SF9 fallback" sends `CMD_LINK_TUNE` "at *both* the current SF and the new SF (back-to-back) for one cycle." Doubling airtime exactly when the link is bad is counter-productive. | Use a deterministic time-based handover: announce new SF in 3 consecutive frames, switch on a fixed sequence number. |

---

## 3. Findings prior reviews already covered (consolidated for completeness)

I confirm these and add no new analysis — see the cited prior reviews for detail. Listed here so this document alone is sufficient as the action list.

| # | Topic | Severity | Source |
|---|---|---|---|
| C1 | LoRa airtime budget oversubscribed at SF7/BW125 with current cadences | **Critical** | [Copilot v1.0 §1](2026-04-27_Controller_Master_Plan_Review_Copilot_v1_0.md), [v1.0 §Critical 1–2](2026-04-27_Controller_Master_Plan_Review_v1.0.md), [Gemini §1.1](2026-04-27_MASTER_PLAN_Review_Gemini_3.1_Pro.md) |
| C2 | Base-station radio ownership contradicted across docs and `lora_bridge.py` (Linux-SPI vs `base.ino`) | **Critical** | Copilot v1.0 §2, v1.0 §Addendum B |
| C3 | D1608S SSR routing in §8.18 not propagated to active Opta firmware | **Critical** | Copilot v1.0 §3, v1.0 §Addendum C |
| C4 | Base E-stop replay vulnerability (header sequence = 0) | **Critical** | Copilot v1.0 §4, v1.0 §Addendum A |
| C5 | M7/M4 responsibility split documented ≠ implemented | **High** | Copilot v1.0 §5 |
| C6 | E-stop clear/recovery flow incomplete | **High** | Copilot v1.0 §6 |
| C7 | AES-GCM nonce flash-wear at 20 Hz persistence (§8.14) | **High** | v1.0 §High 6, Gemini §1.3 |
| C8 | Single-radio multi-PHY behavior underspecified | **High** | v1.0 §Critical 3 |
| C9 | Linux-Python LoRa MAC jitter on base | **High** | Gemini §1.3 |
| C10 | FCC Part 15.247 duty-cycle / FHSS compliance question | **Medium** | Gemini §1.1 |
| C11 | Modbus FC23 consolidation opportunity | **Medium** | v1.0 §Medium 10, Gemini §2.2 |
| C12 | FT232H USB-I²C jitter vs native I²C on X8 SoC | **Medium** | Gemini §2.1 |
| C13 | "CONFIRM" preface tag not actually used in §8 | **Low** | v1.0 §Low 12 |

---

## 4. Recommended action order

The right order is *not* "fix all 25 issues in parallel". Some fixes invalidate others.

1. **Decide the airtime profile (this review §1.1, C1).** Profile A or B. Without this, every cadence number downstream is fiction.
2. **Decide the base-radio path (C2).** Linux-SPI or H747-modem. Without this, `lora_bridge.py` is rewriting against a moving target.
3. **Decide the M7/M4 split (C5).** Either edit the plan to match the code (M7 owns control, M4 is a watchdog) or refactor the code to match the plan.
4. **Fix the four hard safety bugs (C3, C4, C6, C7).** These four together must land before any hydraulic-pressurised test.
5. **Reserve the wire-format flag bits and tractor_id (this review §1.6, E9).** Cheap now, expensive later.
6. **Run the doc-alignment sweep (E1–E13).** Mostly mechanical once §1–§4 above are pinned.
7. **Add the new IPC, key-provisioning, OS-rfkill, and logger-as-gate pins (this review §1.4, §1.5, §1.8, §1.9).** Each is small individually; together they remove the most likely "we forgot to think about that" failures during bench bring-up.
8. **Add the click-to-TX latency gate (this review §1.10).** Before bench bring-up, not after.

When all of the above land, the plan becomes a buildable spec. Until then it is a strong direction document with arithmetic homework outstanding.

---

## 5. What *not* to change

To balance the critique: these choices are correct and should be defended against scope creep:

- LoRa-only RF to the tractor.
- Opta as Modbus-RTU isolated I/O.
- X8 Linux as non-realtime sidecar; H747 owns the hot path.
- USB-C-only flashing for v25.
- Plain HTTP + LAN-only + shared PIN for v25 (with the §1.7 caveat).
- Hardware E-stop chain via PSR safety relay.
- The "research is in `RESEARCH-CONTROLLER/`, canonical is in `DESIGN-CONTROLLER/`" partition.
- Field-test gate at 500 m before escalation.

These choices are the spine of the v25 system. None of the findings above suggest changing any of them.

---

**End of review v1.0.** Successor versions of this document should bump the filename version (`_v1_1`, `_v2_0`) and link back to this one for diff context.
