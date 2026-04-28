# LifeTrac v25 Safety Case (ISO 25119 sketch)

**Status:** DRAFT FOR REVIEW. Not an audited safety file — provides the
working hazard table and mitigation map the v25 design has been built
against. Final sign-off requires a third-party functional-safety review.

**Companions:** [MASTER_PLAN.md](MASTER_PLAN.md), [TRACTOR_NODE.md](TRACTOR_NODE.md),
[BASE_STATION.md](BASE_STATION.md), [LORA_PROTOCOL.md](LORA_PROTOCOL.md),
[CYBERSECURITY_CASE.md](CYBERSECURITY_CASE.md).

## 1. Scope

LifeTrac v25 is a **remote-controlled and tele-operated agricultural
tractor** (~3 t class, hydraulic implements). Operators are within
~500 m line-of-sight of the machine. The system has three command
sources: handheld (P0), base station (P1), autonomy (P2 / P3).

This document covers the **safety-related electrical and software**
controls. It does **not** cover mechanical/hydraulic/structural safety
(see DESIGN-STRUCTURAL/, DESIGN-HYDRAULIC/).

## 2. Standards anchor

ISO 25119 (Tractors and machinery for agriculture and forestry —
Safety-related parts of control systems). Target Performance Level **AgPL-c**
for the remote-control E-stop chain, **AgPL-b** for non-emergency
operator controls. Rationale: remote operation does not change the
mechanical hazard envelope, but loss-of-link must be a recognised
foreseeable failure mode and shall not lead to uncommanded motion.

## 3. Hazard table (HARA-style)

| ID | Hazard | Cause | Severity (S) | Exposure (E) | Controllability (C) | AgPL | Mitigation refs |
|----|--------|-------|--------------|--------------|--------------------|------|-----------------|
| H1 | Uncommanded motion after loss of radio link | Handheld dropped, base PC crash, antenna fault | S2 | E2 | C2 | c | LORA_PROTOCOL.md § Heartbeat watchdog (200 ms hard cutoff at H7); TRACTOR_NODE.md § Wiring overview (M4 watchdog + Opta watchdog, both independent) |
| H2 | Latched motion when operator releases joystick | Stuck input on handheld, frozen WS | S2 | E3 | C2 | c | Handheld and base both send neutralised ControlFrames at 20 Hz; tractor zeros all valves on missed-frame timeout per LORA_IMPLEMENTATION.md §4 |
| H3 | E-STOP press not actioned | Frame loss, GCM rejection, radio jam | S3 | E2 | C1 | c | E-STOP is FT_COMMAND/CMD_ESTOP, P0 priority, retransmitted 3× with QoS 1 over MQTT; physical E-STOP on base hard-wires the PSR safety relay (not radio-dependent) |
| H4 | Wrong source's commands accepted (handheld off but base controlling) | Handoff race | S2 | E2 | C2 | c | Strict source priority enforced in tractor_m7.ino apply_control(); only one source may be ACTIVE at a time, see DECISIONS.md D-A1 / D-A4 |
| H5 | Replay attack — captured ControlFrame replayed by attacker | Hostile RF capture + replay | S2 | E1 | C2 | c | AES-256-GCM with monotonically-increasing nonce + replay window; LORA_PROTOCOL.md § Cryptographic envelope; rejected frames audited per audit_log.py |
| H6 | Stale image misleads operator into commanding into an obstacle | Lossy radio, frozen camera | S2 | E2 | C2 | b | Tile-age ≥ N seconds → yellow tint overlay + visible age in seconds (BASE_STATION.md § Image pipeline); operator instructed to release controls when canvas freezes |
| H7 | M7 firmware hangs but Modbus link still healthy | M7 SW fault | S2 | E2 | C2 | c | M4 independent watchdog (firmware/tractor_h7/tractor_m4.cpp) drops PSR safety relay if M7 stops ticking >200 ms; Opta also expects REG_WATCHDOG_CTR change |
| H8 | Engine kill solenoid command mis-fires | Fault on REG_ARM_ESTOP write | S2 | E1 | C3 | b | REG_ARM_ESTOP only writable from M7; Opta refuses motion writes once latched (firmware/tractor_opta/opta_modbus_slave.ino) |
| H9 | Operator authenticates against a clone of the base UI on hostile WiFi | LAN / DNS spoof | S2 | E1 | C2 | b | LIFETRAC_PIN required, brute-force lockout 5/60 s; v25 deploys on dedicated SSID (per MASTER_PLAN.md §8.5) |
| H10 | New firmware bricks safety chain on push | OTA bug | S3 | E1 | C1 | c | OTA_STRATEGY.md: signed firmware only, watchdog-protected swap, factory rollback after N failed boot health-checks |

S/E/C scale per ISO 25119:2010 Annex A. AgPL letters per the same annex.

## 4. Architectural mitigations (defence in depth)

The v25 architecture relies on **three independent watchdog chains**
between the operator's stick and the hydraulic valves; **all three** must
agree the system is healthy for valves to energise.

1. **M7 control-loop watchdog** — apply_control() at 20 Hz; missed
   ControlFrame for >200 ms zeros all outputs in software.
2. **M4 hardware watchdog** — separate Cortex-M4 core on the H747 reads
   shared SRAM4; pulls PIN_PSR_ALIVE low if M7's `alive_tick_ms` or
   `loop_counter` stops advancing for >200 ms. Latches until reset.
   See [firmware/common/shared_mem.h](firmware/common/shared_mem.h).
3. **Opta Modbus watchdog** — Opta drops all coils + zeros analog
   outputs if REG_WATCHDOG_CTR is unchanged for >200 ms.

A single point of failure on any one chain is therefore non-hazardous:
the other two still de-energise the valves and the engine-kill solenoid.

## 5. E-STOP

The base station and handheld each carry a hardware E-STOP button:

- **Base hardware E-STOP** is wired in series with the PSR safety relay
  *and* publishes a CMD_ESTOP frame. Pressing it drops valve power
  even if the Linux side is hung — auth-independent.
- **Handheld hardware E-STOP** publishes CMD_ESTOP P0 with QoS 1; the
  M7 latches and the M4 mirrors via shared-memory `estop_request`.
- **Software E-STOP** (UI button, `/api/estop`) requires PIN auth.

Recovery from an E-STOP requires a *physical* operator action on the
handheld (key-cycle of the latched switch) AND a base-station UI clear.
This is intentional — software-only recovery would mask intermittent
electrical faults.

## 6. Open items / known limitations

- **Single LoRa channel** (no certified backup path): documented
  constraint, not a defect; fall-back is the multi-watchdog
  de-energise behaviour above.
- **No ASIL/PL formal proof**: AgPL claims here are working targets;
  a notified-body review is out of scope for v25.
- **Mechanical guarding** of the base station and tractor (operator
  exclusion zone) is documented in DESIGN-STRUCTURAL/.

## 7. Verification matrix

| Mitigation | Test | Where defined |
|------------|------|---------------|
| H1 watchdog | Bench: pull antenna mid-drive → valves de-energise within 250 ms | LORA_IMPLEMENTATION.md §10 |
| H3 E-STOP | Bench: press base E-STOP; press handheld E-STOP; trigger via UI | BASE_STATION.md § Bench validation |
| H5 replay | `base_station/tests/test_replay_window.py` | base_station/tests/ |
| H7 M4 watchdog | Bench: pause M7 in debugger → PSR drops | TRACTOR_NODE.md § M4 watchdog |
| H10 OTA | OTA_STRATEGY.md § Rollback test | OTA_STRATEGY.md |
