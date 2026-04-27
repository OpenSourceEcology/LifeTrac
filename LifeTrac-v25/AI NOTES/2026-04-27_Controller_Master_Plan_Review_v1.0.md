# Controller Master Plan In-Depth Review v1.0 - LifeTrac v25
Date: 2026-04-27
Reviewer: GitHub Copilot
Primary document reviewed: DESIGN-CONTROLLER/MASTER_PLAN.md (dated 2026-04-26)
Cross-check set: DESIGN-CONTROLLER/LORA_PROTOCOL.md, DESIGN-CONTROLLER/TRACTOR_NODE.md, DESIGN-CONTROLLER/HARDWARE_BOM.md, DESIGN-CONTROLLER/TODO.md

## Executive Verdict
Overall status: AMBER-RED.

The architecture direction is strong (clear scope, safety-first partitioning, good separation of canonical vs research), but the current RF timing assumptions in the pinned control-link policy are not internally consistent with LoRa PHY reality at BW 125 kHz. As written, the control cadence and packet budget are not physically achievable, and this will likely become the primary blocker before safe field operation.

Estimated readiness impact if uncorrected:
- Bench protocol realism risk: High
- End-to-end control-loop stability risk: Critical
- Schedule slip risk (rework after bench bring-up): High

## What Is Strong in the Plan
1. Scope discipline is excellent: LoRa-only tractor link is clearly pinned and alternatives are explicitly archived.
2. Safety architecture is strong: H747 real-time ownership + Opta watchdog + hardware PSR E-stop chain is the right safety posture.
3. Canonical tree intent is clear: MASTER_PLAN defines ownership boundaries well.
4. Bring-up path is practical: LED stand-ins before hydraulics is correct and should be kept.

## Findings (Ordered by Severity)

### Critical 1: Section 8.17 air-time figures are not feasible for SF7/SF8/SF9 at BW 125 kHz
Evidence:
- MASTER_PLAN 8.17 pins control PHY to SF7/BW125/CR4-5 and claims ~30 ms air time for the encrypted control frame.
- The same section claims ~55 ms at SF8 and ~100 ms at SF9.
- With typical LoRa timing for about 44-46 bytes on-air payload (as documented in protocol notes), SF7/BW125 is about 90-100 ms, SF8/BW125 about 160-180 ms, SF9/BW125 about 280-320 ms.

Impact:
- Core timing assumptions in the master plan are optimistic by about 3x.
- Control-loop and watchdog behavior will not match design expectations during bench tests.
- Adaptive fallback policy in 8.17 becomes unstable under real link conditions.

Recommendation:
- Recompute and repin air-time values using one canonical formula and one canonical payload size.
- Add an explicit "air-time budget" table in MASTER_PLAN and require linked docs to match it.

### Critical 2: 20 Hz ControlFrame plus 20 Hz HeartbeatFrame is channel-infeasible at BW 125 kHz
Evidence:
- MASTER_PLAN 8.12 pins 20 Hz control uplink and 20 Hz heartbeat uplink.
- MASTER_PLAN 8.17 pins BW125 for control path.
- Even conservative realistic packet durations at BW125 imply >100% channel occupancy for control alone, and far above 100% when standalone heartbeats are added.

Impact:
- Persistent frame drops, starvation, and false failsafe entries are likely.
- The system may oscillate between "link degraded" and "failsafe" rather than providing usable teleoperation.

Recommendation:
- Remove standalone heartbeat frames while control frames are flowing; heartbeat should be piggybacked in control traffic.
- Choose one of two coherent profiles:
  - Low-latency profile: keep 20 Hz control and move back to wider bandwidth strategy.
  - Long-range profile: keep BW125 but reduce control cadence and adjust watchdog/timeouts accordingly.

### Critical 3: Single-radio, multi-PHY behavior is underspecified and currently contradictory
Evidence:
- MASTER_PLAN 8.17 says control link adapts SF while telemetry keeps separate SF/BW.
- SX1276-class radios are single-channel/single-PHY at a time unless explicit retune scheduling is defined.
- No deterministic schedule is defined in MASTER_PLAN for when each side listens at which PHY.

Impact:
- Frames may be transmitted at PHY settings the peer is not currently listening to.
- Link-tune transitions can become lossy or deadlock-prone in edge cases.

Recommendation:
- Define an explicit TDD/superframe schedule in canonical docs (for example fixed 100 ms cycles with known RX/TX windows and retune points).
- If separate control and telemetry PHY profiles are retained, define exact switch timing and guard intervals.

### High 4: Canonical-drift risk remains high across protocol and implementation docs
Evidence:
- MASTER_PLAN positions itself as single source of truth.
- Cross-check docs still contain active-language references to archived cellular paths, older PHY assumptions, and older node responsibilities.
- Some examples still rely on historical paths and assumptions that differ from pinned v25 policy.

Impact:
- Engineers can implement against contradictory requirements.
- Review and test failures will be misdiagnosed as firmware bugs when they are doc-contract mismatches.

Recommendation:
- Run a canonical alignment pass across LORA_PROTOCOL, TRACTOR_NODE, HARDWARE_BOM, and TODO.
- Add a "Policy Conformance" checklist header in each canonical doc with version/date of MASTER_PLAN it is aligned to.

### High 5: AES-GCM nonce/counter uniqueness is not fully pinned for all transmitters
Evidence:
- MASTER_PLAN references nonce/sequence handling, and calls out handheld persistence.
- Requirements are not equally explicit for base-source control frames and tractor telemetry source counters.
- Any nonce reuse with the same key in GCM is a severe cryptographic failure mode.

Impact:
- Potential replay and integrity failures after reboots/power interruptions if counters are not globally consistent.

Recommendation:
- Pin a single nonce scheme in MASTER_PLAN and LORA_PROTOCOL that guarantees uniqueness for every source.
- Require persistent monotonic counters (or boot-epoch + counter strategy) for handheld, base, and tractor.

### High 6: Nonce persistence policy is ambiguous and can cause handheld flash-endurance failure
Evidence:
- MASTER_PLAN 8.14 states nonce uniqueness uses a monotonic per-source sequence persisted to handheld flash.
- The document does not pin a persistence cadence, wear-leveling strategy, or boot-time epoch policy.
- At 20 Hz, naively persisting each increment would imply about 72,000 writes/hour, which can exceed embedded flash endurance rapidly.

Impact:
- Premature handheld flash wear if implemented naively.
- Increased risk of nonce reuse if implementers reduce persistence frequency ad hoc without a pinned recovery scheme.

Recommendation:
- Pin a flash-safe nonce policy in canonical docs:
  - Keep the fast counter in RAM.
  - Persist only coarse epoch/high bits on a controlled cadence.
  - Advance epoch on boot before first transmit.
  - Use a wear-leveled ring (or FRAM/NVRAM when available).

### High 7: Base-station Linux-SPI radio path is reasonable but lacks explicit degradation gates
Evidence:
- MASTER_PLAN 8.2 intentionally chooses Linux SPI control without base co-MCU firmware.
- Fallback path is described, but trigger criteria are not defined.

Impact:
- Teams may debate fallback late, after substantial time spent chasing timing edge cases.

Recommendation:
- Add hard trigger criteria now (for example missed IRQ rate, frame-jitter threshold, sustained packet-loss threshold).
- Define a short decision checkpoint after loopback tests where fallback is accepted or rejected.

### High 8: Bring-up critical path includes too many non-safety services before hydraulics
Evidence:
- MASTER_PLAN 8.10 pins several Linux services before bench bring-up (gps, imu, logger, params, event recorder, time discipline, camera plumbing).

Impact:
- High schedule risk and integration complexity before validating core control safety path.

Recommendation:
- Split into strict P0 and P1:
  - P0 before hydraulics: control path, watchdogs, estop, modbus, minimal telemetry, logging minimal.
  - P1 after first stable hydraulic valve test: GPS/IMU/video and advanced service stack.

### Medium 9: Authentication model is intentionally simple but needs two additional controls
Evidence:
- MASTER_PLAN 8.5 uses shared PIN and plain HTTP on LAN-only model.

Impact:
- Accidental LAN exposure or weak WiFi segmentation can still allow nuisance control lockouts or unauthorized unlock attempts.

Recommendation:
- Add explicit network guardrails in plan text:
  - Bind control endpoints to trusted subnet allowlist.
  - Require control unlock only from same-origin session with explicit CSRF token for control actions.

### Medium 10: Modbus transaction strategy can be optimized for better timing headroom
Evidence:
- MASTER_PLAN 8.12 pins 50 Hz write and 10 Hz read rates on the tractor RS-485 link.
- The document does not pin whether these operations are benchmarked as separate transactions or consolidated read/write cycles.
- Separate request/response cycles increase turnaround overhead and reduce margin during transient bus errors.

Impact:
- Lower timing headroom and weaker recovery behavior under noisy or degraded bus conditions.
- Increased risk of jitter propagating into watchdog-sensitive control cycles.

Recommendation:
- Benchmark consolidated Modbus read/write strategy (for example FC23 Read/Write Multiple Registers) versus current split cycles.
- Choose and pin one canonical transaction model in TRACTOR_NODE and MASTER_PLAN based on measured worst-case timing.

### Medium 11: Source-arbitration transition behavior is not fully explicit
Evidence:
- MASTER_PLAN 8.3 defines priority and active-source behavior.
- Detailed semantics for passive-source liveness/claim timing are left to protocol details and can be interpreted differently.

Impact:
- Different implementations may handle handoff and control-request timing differently.

Recommendation:
- Add a short state-transition table in MASTER_PLAN for source claim, preemption, and release timing.

### Low 12: Editorial mismatch in Section 8 preface
Evidence:
- Section 8 preface says items marked "CONFIRM" need sign-off.
- In the current document snapshot, no section is actually marked with that tag.

Impact:
- Minor governance ambiguity.

Recommendation:
- Either add explicit CONFIRM tags where intended or remove the preface sentence.

## Secondary Analysis Addendum (Validated Against Active Repo)

The items below were extracted from the secondary review and then independently validated against current canonical docs and active implementation files.

### Addendum A: Base E-stop replay risk is currently real
Evidence:
- `base_station/lora_bridge.py` builds `/cmd/estop` with a header sequence of `0` (`struct.pack(... FT_COMMAND, 0, 0x01)`).
- The same file increments `self.tx_seq` in `_tx()` and uses that sequence for encryption nonce construction.
- `firmware/tractor_h7/tractor_m7.ino` rejects stale/replay frames based on the plaintext header sequence before command handling.

Impact:
- After normal base control traffic advances the tractor's last-seen base sequence, a later E-stop with header sequence `0` can be dropped as replay/stale.

Recommendation:
- Stamp command headers (including E-stop) with the same advancing base sequence used for nonce construction.
- Add a regression test that proves E-stop still latches after N prior base control frames.

### Addendum B: Base radio ownership is contradictory across canonical docs and bridge code
Evidence:
- `MASTER_PLAN.md` pins Linux-on-X8 direct SPI control and explicitly says there is no base H747 firmware target.
- `BASE_STATION.md` still references `firmware/base_h7/base.ino`.
- `LORA_PROTOCOL.md` still includes `base_h7/base.ino` in planned layout.
- `base_station/lora_bridge.py` still describes serial link behavior to an H747-side modem sketch.

Impact:
- Implementation and CI cannot converge on one base-station radio architecture.

Recommendation:
- Choose one canonical base radio path now and align all canonical docs plus bridge implementation to it.

### Addendum C: D1608S routing decision is not propagated into active Opta firmware
Evidence:
- `MASTER_PLAN.md` section 8.18 pins all 8 directional coils to D1608S SSR channels.
- `firmware/tractor_opta/opta_modbus_slave.ino` still maps bits 0..3 to onboard relays R1..R4 and bits 4..7 to D1608S placeholder calls.

Impact:
- Active control I/O path does not match the pinned low-latency architecture.

Recommendation:
- Update Opta mapping so all directional coils are driven through D1608S channels, and keep onboard EMRs for auxiliary/engine-kill roles only.

### Addendum D: Command opcode contracts are split between protocol and shared header
Evidence:
- `firmware/common/lora_proto/lora_proto.h` defines `CMD_REQ_CONTROL = 0x02` and `CMD_REKEY = 0x03`.
- `LORA_PROTOCOL.md` defines `0x02` as `CMD_CLEAR_ESTOP` and `0x03` as `CMD_CAMERA_SELECT`.

Impact:
- Components may compile while disagreeing on command semantics.

Recommendation:
- Freeze command opcodes in one canonical source and generate/mirror constants into all C and Python components.

### Addendum E: Key management policy in plan does not match active implementation defaults
Evidence:
- `MASTER_PLAN.md` requires `key.h` provisioning and forbids stub crypto in production.
- `firmware/handheld_mkr/handheld.ino` and `firmware/tractor_h7/tractor_m7.ino` still use `kFleetKey[16] = {0}`.
- `base_station/lora_bridge.py` uses `FLEET_KEY = bytes(16)`.

Impact:
- Runtime behavior can drift from intended production security model, and test outcomes may not represent provisioned-key operation.

Recommendation:
- Implement `key.h`/secret provisioning path across all nodes and remove zero-key defaults from active targets.

### Addendum F: CI and CI documentation still track archived paths
Evidence:
- `.github/workflows/arduino-ci.yml` compiles `DESIGN-CONTROLLER/arduino_opta_controller` and `DESIGN-CONTROLLER/esp32_remote_control` paths.
- `DESIGN-CONTROLLER/ARDUINO_CI.md` documents the same archived targets.

Impact:
- Canonical v25 firmware tree is not being validated by CI.

Recommendation:
- Retarget CI and CI docs to active firmware directories and add Python checks for base-station services.

### Addendum G: Sensor stack is inconsistent between MASTER_PLAN and procurement/implementation docs
Evidence:
- `MASTER_PLAN.md` hardware table lists Adafruit BNO085 + FT232H.
- `HARDWARE_BOM.md` and `TODO.md` specify MCP2221A + SparkFun BNO086 path.

Impact:
- Procurement and software-driver assumptions can diverge before bring-up.

Recommendation:
- Pin one canonical sensor stack and update all canonical docs accordingly.

### Addendum H: Cellular is archived in policy, but still present in active procurement/test lines
Evidence:
- `TODO.md` and `HARDWARE_BOM.md` contain archived warning banners, but still include SIM, cellular antenna, and fallback test tasks in active tables/lists.
- `LORA_PROTOCOL.md` still states E-stop is sent over both LoRa and cellular.

Impact:
- Procurement and test execution can still drift into archived scope.

Recommendation:
- Remove or isolate archived cellular items from active procurement and bring-up lists to prevent operational confusion.

## Key Pitfalls to Resolve Before Hardware Motion Tests
1. RF control profile must be internally coherent with real air-time math.
2. Channel occupancy must stay below practical limits with control + telemetry + command traffic.
3. PHY switching behavior must be deterministic on single-radio hardware.
4. Document contract drift must be corrected so all teams implement the same wire and safety behavior.
5. Nonce persistence must preserve cryptographic uniqueness without exhausting flash endurance.
6. E-stop command sequencing must be replay-safe across mixed control/command traffic.
7. Canonical docs, CI, and active firmware mapping must agree on base radio path and valve-output routing.

## Improvements and Optimizations

### Optimization A: Replace dual-frame control heartbeat with unified control snapshot
- Merge heartbeat fields into ControlFrame and transmit one P0 frame at the control cadence.
- Keep standalone heartbeat only when controls are idle.
- Expected benefit: major airtime reduction and lower collision probability.

### Optimization B: Add deterministic radio scheduling
- Define fixed TX/RX windows and ordering for control, command, and telemetry.
- For example: control window first, command micro-window second, telemetry opportunistic third.
- Expected benefit: predictable behavior during link degradation and easier reproducible bench testing.

### Optimization C: Define two compile-time link profiles instead of one overloaded profile
- Profile FAST: for close-range low-latency teleop.
- Profile RANGE: for extended-range slower-response operation.
- Expected benefit: avoids trying to force contradictory latency/range goals into one static profile.

### Optimization D: Tighten release gates with measurable criteria
- Add protocol-gate metrics before hydraulics:
  - sustained packet delivery under controlled attenuation,
  - no false failsafe during 30-minute bench control run,
  - deterministic estop latency under load.
- Expected benefit: catches RF contract errors before hydraulic risk is introduced.

### Optimization E: Add doc-conformance CI checks
- Add a lightweight check that flags contradictory policy markers (cellular references in canonical path, PHY/rate mismatches against MASTER_PLAN).
- Expected benefit: prevents future drift after this review.

### Optimization F: Make nonce persistence flash-safe by design
- Persist only coarse sequence epochs, keep per-frame increments in RAM, and advance epoch on boot.
- Add wear-leveling/NVRAM guidance in canonical docs.
- Expected benefit: preserves nonce uniqueness while preventing handheld flash wear-out.

### Optimization G: Consolidate Modbus cycles where beneficial
- Benchmark combined read/write transaction patterns against split cycles and pin one canonical approach.
- Expected benefit: increases RS-485 timing margin and reduces watchdog-jitter coupling.

## Concrete Master Plan Edit Set (Recommended)
1. In 8.17, replace air-time numbers with corrected values and include formula assumptions.
2. In 8.12, revise control/heartbeat cadence strategy to avoid impossible occupancy.
3. In 8.17, add explicit single-radio scheduling rules for control and telemetry PHY switching.
4. In 8.10, move non-critical Linux services to post-P0 milestones.
5. In 8.5, add minimum network hardening controls for shared-PIN LAN operation.
6. In 8.14 and LORA_PROTOCOL, pin a flash-safe nonce persistence policy with explicit boot behavior.
7. In 8.12/TRACTOR_NODE, pin the canonical Modbus transaction model after benchmark.
8. In Section 8 preface, resolve the "CONFIRM" marker mismatch.
9. Add one canonical command-opcode table and one canonical base-radio ownership statement, then enforce both in LORA_PROTOCOL/BASE_STATION and active bridge code.
10. Add a canonical E-stop state machine covering base UI command, replay checks, tractor latch, Opta latch, and clear/rearm rules.
11. Add a canonical sensor-stack statement (BNO085+FT232H vs BNO086+MCP2221A) and align BOM/TODO/MASTER_PLAN.

## Proposed Priority Plan

### P0 (immediate, before next firmware cycle)
1. Correct 8.17 air-time table and repin rates.
2. Decide unified control/heartbeat strategy.
3. Freeze a deterministic single-radio scheduling policy.
4. Pin flash-safe nonce persistence and boot-epoch handling.
5. Fix base E-stop sequence stamping and verify replay-safe behavior with regression tests.
6. Freeze one base radio ownership model and align canonical docs to it.

### P1 (before hydraulic bench test)
1. Align LORA_PROTOCOL, TRACTOR_NODE, HARDWARE_BOM, TODO to corrected master policy.
2. Add protocol conformance checks in CI.
3. Add pass/fail gate for false-failsafe soak test.
4. Benchmark and freeze one Modbus transaction strategy.
5. Align Opta directional-coil routing to D1608S across docs and firmware.
6. Retarget Arduino CI and CI docs to active v25 firmware paths.
7. Implement documented PIN/session gate in web UI before LAN multi-client control tests.

### P2 (before field test)
1. Validate fallback decision for base radio path using explicit thresholds.
2. Complete security counter/nonce pinning for all source nodes.
3. Finalize network hardening baseline for base UI.

## Final Assessment
The master plan is structurally strong and close to being an excellent canonical control document. The main blocker is not concept selection, but timing realism in the pinned LoRa control profile plus resulting cross-doc drift. Correcting those items first will substantially reduce downstream rework and improve safety confidence during bench and hydraulic bring-up.
