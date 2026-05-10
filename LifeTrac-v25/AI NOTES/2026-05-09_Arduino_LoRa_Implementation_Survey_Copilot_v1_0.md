# Arduino LoRa Implementation Survey for LifeTrac v25 (Copilot v1.0)

Date: 2026-05-09
Scope: Comparative research of mature Arduino LoRa/LoRaWAN implementations, with focus on architecture, reliability patterns, diagnostics, and applicability to current LifeTrac Murata host-link debugging.

## Why this matters to current blocker

Current Method G Stage 1 evidence shows no host RX ingress at Murata UART ISR level during active probe (`VER_REQ` -> expected `VER_URC`).

Research objective here: identify proven implementation patterns from other Arduino LoRa stacks that improve:
- host-modem serial robustness,
- timing/loop discipline,
- fault observability,
- deterministic recovery.

## Implementations reviewed

1. Arduino MKRWAN library + MKRWAN1300 firmware
- Library: https://github.com/arduino-libraries/MKRWAN
- Murata firmware source: https://github.com/arduino/mkrwan1300-fw

2. MCCI Arduino LMIC + Arduino LoRaWAN wrapper
- LMIC core: https://github.com/mcci-catena/arduino-lmic
- Wrapper: https://github.com/mcci-catena/arduino-lorawan

3. RadioLib LoRaWAN
- https://github.com/jgromes/RadioLib

4. Sandeep Mistry Arduino-LoRa (P2P)
- https://github.com/sandeepmistry/arduino-LoRa

5. SX126x-Arduino (beegee-tokyo)
- https://github.com/beegee-tokyo/SX126x-Arduino

6. Official/operational references
- Arduino MKR WAN docs: https://docs.arduino.cc/hardware/mkr-wan-1310/
- TTN LoRaWAN fundamentals: https://www.thethingsnetwork.org/docs/lorawan/

## Key architecture patterns

### A) Host-modem split with AT command contract (MKRWAN + mkrwan1300-fw)

Observed pattern:
- Host side (`MKRWAN.h`) uses strict command/response grammar via `sendAT(...)` + `waitResponse(...)`.
- Modem side (`mkrwan1300-fw` AT_Slave) has command parser (`CMD_Process`, `parse_cmd`) and explicit AT error taxonomy (`+ERR`, `+ERR_PARAM`, `+ERR_BUSY`, etc.).
- Downlinks/events are surfaced as asynchronous event strings (e.g., `+EVENT=...`, `+RECV=...`) and consumed by host parser.

What is strong:
- Clear textual protocol boundaries and explicit error tokens.
- Host gets graded error causes, not just timeout.
- Firmware includes startup event signal (`+EVENT=0,0`) for liveness sync.

What to borrow for LifeTrac:
- Add explicit transport-level heartbeat and startup-ready token in binary protocol equivalent.
- Preserve specific negative send outcomes (busy, no network, overflow, timeout) at host API boundary.
- Keep one parser owner and one command dispatcher owner (single authority per direction).

### B) Time-discipline runloop architecture (MCCI LMIC)

Observed pattern:
- Tight and explicit scheduler (`os_runloop_once`) with timed jobs and IRQ edge processing.
- Strong emphasis that application must call runloop frequently; helper API `os_queryTimeCriticalJobs(...)` for near-deadline awareness.
- Configurable interrupt vs polling model for DIO handling.
- Practical clock error compensation (`LMIC_setClockError`) and documented behavior.

What is strong:
- Precise execution model reduces hidden timing races.
- Deterministic event queue and callback dispatch.
- Good docs for real-time constraints and failure modes.

What to borrow for LifeTrac:
- Maintain explicit service cadence for host transport state machine (not ad-hoc polling).
- Track "time-critical jobs due soon" concept in probe/firmware loops.
- Keep ISR edge capture separated from heavy work, then process in controlled loop context.

### C) Structured event metadata and status decoding (RadioLib)

Observed pattern:
- `sendReceive(...)` returns both high-level result and optional event structs (`LoRaWANEvent_t`) with datarate/freq/fcnt/flags.
- Large status code surface with decode helpers in examples.
- Session persistence buffers are explicit and documented.
- Optional class switching and package manager hooks are explicit APIs.

What is strong:
- Visibility-first API design.
- Easier bench triage due to meaningful status labels.

What to borrow for LifeTrac:
- Extend probe logs to include structured fields for every transaction outcome.
- Maintain status decode table close to transport constants.
- Prefer typed event payloads over ad-hoc print fragments.

### D) P2P radio-first simplicity with strict pin/IRQ assumptions (arduino-LoRa)

Observed pattern:
- Very simple begin/send/receive/callback APIs.
- Hard requirement that `setPins(...)` and SPI setup are correct before `begin(...)`.
- DIO0 interrupt is central for callback-driven receive/tx done/CAD done.
- FAQ explicitly calls out wiring/current/SPI frequency as primary failure roots.

What is strong:
- Fast bring-up and clear minimal API.
- Strong practical debugging guidance around hardware realities.

What to borrow for LifeTrac:
- Preserve simple "hardware first" checklist before protocol debugging:
  - RX/TX routing,
  - level/baud/parity,
  - SPI/UART clocking,
  - current/voltage integrity.
- Keep callback/ISR assumptions explicit in docs and probes.

### E) Deep-sleep re-init and board-specific HW config discipline (SX126x-Arduino)

Observed pattern:
- Distinct init vs re-init paths (`lora_hardware_init` vs `lora_hardware_re_init`, `Radio.Init` vs `Radio.ReInit`).
- Hardware config struct captures all board pin mappings and antenna/TCXO control choices.
- Region and sub-band setup are explicit and documented.

What is strong:
- Avoids unnecessary full resets after wakeup.
- Centralized HW config reduces per-board drift.

What to borrow for LifeTrac:
- Keep explicit transport re-sync path separate from cold boot path.
- Maintain one canonical board transport map source (lanes, mux, inversion, parity).

## Comparative summary

| Stack | Best at | Typical risk if misused | Most relevant to current LifeTrac issue |
|---|---|---|---|
| MKRWAN + mkrwan1300-fw | Host-modem protocol contract and AT error grading | Parser sync loss if framing/newline assumptions are wrong | Very high (host<->Murata serial boundary) |
| MCCI LMIC | Timing discipline and scheduler correctness | Missed RX windows if loop is starved | Medium (transport state machine cadence) |
| RadioLib | Rich observability and status semantics | Complexity overhead if over-integrated | High (better probe telemetry model) |
| arduino-LoRa | Minimal bring-up and hardware sanity checks | Hardware issues can masquerade as software issues | Very high (ingress path verification mindset) |
| SX126x-Arduino | Init/re-init separation, board config hygiene | Region/subband/config mismatch | Medium (recovery and board abstraction patterns) |

## Direct recommendations for LifeTrac (next practical moves)

1. Introduce a transport-ready banner event at Murata boot
- Analog to `+EVENT=0,0` semantics from AT_Slave.
- Purpose: separate "firmware alive" from "command parser alive" and from "RX lane alive".

2. Add explicit host-link error taxonomy in binary probe path
- Preserve distinct outcomes equivalent to:
  - timeout,
  - parser error,
  - unsupported cmd,
  - RX framing/parity error,
  - lane inactive.
- Current single timeout is too coarse for rapid triage.

3. Add dual-phase sync command pair
- Phase A: tiny no-state ping (transport only).
- Phase B: parser/function ping (command decode path).
- If A fails -> electrical/routing likely.
- If A passes and B fails -> parser/dispatch likely.

4. Mirror MKRWAN-style bounded wait-response strategy
- Every request should have explicit timeout class and failure reason.
- Keep a rolling parser state reset strategy when response preamble is invalid.

5. Keep ISR capture and parser work separated
- Preserve current ISR-seen marker mechanism (already added) and extend with:
  - first-byte timestamp,
  - framing error counters by lane,
  - overrun counters by lane.

6. Add a hardware-first validation checklist gate in Method G
- Before probe retries, enforce a preflight similar to arduino-LoRa FAQ flow:
  - pin route asserted,
  - electrical level compatibility,
  - baud/parity/stop match,
  - known-good low baud fallback,
  - optional direct loopback or passthrough sanity test.

7. Keep cold-boot vs re-sync paths distinct
- Borrow from SX126x-Arduino init/re-init split.
- For host link specifically:
  - cold init = full UART/peripheral init,
  - soft re-sync = parser/queue reset without peripheral reset.

## What not to copy directly

1. Do not switch LifeTrac transport to AT text protocol just for familiarity.
- Current binary framing can remain; adopt AT stack lessons at design-pattern level.

2. Do not rely on loop-starved polling for critical timing.
- Use disciplined scheduler/service points for both host parser and command queue.

3. Do not hide detailed failure causes behind generic timeout.
- Observability debt slows every bench iteration.

## Suggested LifeTrac adoption roadmap

Phase 1 (fast, low risk)
- Add transport-ready event.
- Expand fault codes for transport categories.
- Add probe-side decode table for new categories.

Phase 2 (moderate)
- Implement dual-phase ping (transport ping + parser ping).
- Add per-lane UART hardware error counters and timestamped first-seen metadata.

Phase 3 (structural)
- Refactor host link service loop to explicit scheduler cadence and queue states.
- Add soft re-sync API path distinct from full reboot path.

## Evidence snippets consulted

- MKRWAN host parser/wait-response patterns and send result taxonomy in `src/MKRWAN.h`.
- mkrwan1300-fw AT command parser (`CMD_Process`/`parse_cmd`) and AT command/event model in AT_Slave sources.
- MCCI LMIC timing docs and examples showing strict `os_runloop_once()` cadence and `LMIC_setClockError` usage.
- RadioLib LoRaWAN `sendReceive`/event structures/status handling and class management APIs.
- arduino-LoRa API/examples/FAQ around `setPins`, DIO0 callbacks, SPI frequency, and hardware-first failure diagnosis.
- SX126x-Arduino examples and docs for init vs re-init, callback-centric architecture, and region/subband controls.

## Bottom line

The most useful external pattern for the current blocker is the MKRWAN/mkrwan1300-fw host-modem contract style: explicit startup readiness, explicit response grammar, and explicit error classes. Combined with LMIC-style service-loop discipline and RadioLib-style event/status telemetry, this should materially reduce ambiguity in the next Stage 1 debug cycle.
