# Controller Stage 1 Chip Research and Next Move Plan (Copilot v1.0)

Date: 2026-05-09  
Scope: Murata STM32L072 ROM entry reliability on Portenta X8/Max Carrier path  
Target board: 2E2C1209DABC240B

## Executive Decision

Yes, do more research, but only as a bounded blocker-removal pass.

- Do not run broad literature sweeps.
- Run one short chip-focused research tranche tied directly to bench decisions.
- Immediately execute a discriminating experiment set with stop rules.

Current evidence supports this: ROM-entry pass rate is flat at about 10 percent across 2 ms, 5 ms, and 10 ms, while OTHER/noise increases with delay.

## What Is Already Known (Locked)

- Independent-entry measurement method is correct: ATTEMPTS_PER_BURST=1.
- Reliability is achieved by independent resets, not by many sync attempts in one session.
- Delay tuning in 2..10 ms does not change pass rate in the latest focused runs.
- Noise composition changes with delay (OTHER rises), so delay is now a noise-control parameter, not a success-rate lever.

## Open Uncertainties (Research Targets)

### R1. L072 ROM State Transition and Session Semantics

Question: after one sync exchange, how does the ROM command parser/state machine behavior drive NACK/SILENT progression under our exact host timing?

Decision value: confirms whether any additional in-session probing has diagnostic value, or should be categorically avoided.

### R2. Host-Side Session Noise Injection (OpenOCD + Reset Ownership)

Question: which part of the control path contributes most to OTHER growth with longer delays?

- OpenOCD attach lifetime and transport side effects
- timing between control release and serial probe open
- potential serial line condition changes across delay windows

Decision value: determines whether to harden harness sequencing further or move directly to hardware route-control checks.

### R3. Carrier Route/Ownership Constraints on UART Path

Question: are there route-control or ownership windows where electrical integrity changes without changing ACK probability?

Decision value: if yes, treat as a separate integrity gate and isolate from ROM-entry probability work.

## Next-Move Hypotheses and Discriminating Tests

## H1. Delay is not a success lever; it is primarily a noise lever

Prediction:

- PASS_BURSTS stays near 10 percent across short delays.
- OTHER rises monotonically as delay increases.

Discriminating test:

- Repeat compact matrix: DELAYS_MS=2,5,10, BURSTS_PER_DELAY=20, ATTEMPTS_PER_BURST=1.
- Run N=2 replicate sets on same board and N=1 on second board if available.

Pass criteria for H1 acceptance:

- per-delay pass rates remain statistically overlapping
- OTHER trend remains non-decreasing with delay in most replicates

Action if accepted:

- lock operational delay to shortest practical value (2 ms or 5 ms)
- stop spending cycles on delay expansion

## H2. OTHER increase is mostly host/session artifact, not true ROM response class

Prediction:

- OTHER can be reduced by tighter sequencing and lifecycle controls without improving ACK rate.

Discriminating test:

- A/B harness sequencing only (same delay set, same attempts):
  - A: current sequencing
  - B: stricter lifecycle control (single attach policy, deterministic probe-open point, minimized idle interval)

Pass criteria for H2 acceptance:

- B reduces OTHER materially while ACK remains near baseline

Action if accepted:

- standardize harness to B
- re-baseline all future ROM metrics under B only

## H3. Route/ownership effects primarily change integrity classes, not entry probability

Prediction:

- controlled ownership perturbations shift OTHER/ZERO/NACK composition with limited impact on ACK frequency.

Discriminating test:

- Use the existing owner/route A/B strategy from Stage 1 notes with ATTEMPTS_PER_BURST=1 and short delay.
- Compare class distribution deltas against ACK delta.

Pass criteria for H3 acceptance:

- integrity classes move while ACK remains statistically similar

Action if accepted:

- split workstreams:
  - reliability workstream = independent resets
  - integrity workstream = route/ownership cleanup

## Stop Rules (To Prevent Endless Research)

Stop Rule S1: Delay research stop

- If two additional replicate matrices keep pass-rate flat across 2/5/10 ms, freeze delay research.

Stop Rule S2: Harness research stop

- If sequencing hardening lowers OTHER but not ACK, freeze harness changes to the best deterministic mode and move on.

Stop Rule S3: Chip-doc research stop

- If no new register- or topology-level discriminator appears after one targeted document pass, stop reading and execute bench decision tree only.

## Operational Baseline to Use Immediately

- Keep ATTEMPTS_PER_BURST=1.
- Use short settle delay only (2 ms or 5 ms).
- Scale success probability with independent resets.
- Track two metrics separately:
  - Entry reliability: PASS_BURSTS/TOTAL_BURSTS
  - Line integrity: OTHER and ZERO composition by delay/setup

## 48-Hour Execution Plan

1. Run two additional focused replicate matrices (2/5/10 ms, 20 bursts each, ATTEMPTS_PER_BURST=1).
2. Run one sequencing A/B set to evaluate host/session artifact contribution to OTHER.
3. Produce one short decision memo with:
   - accepted/rejected hypotheses
   - locked baseline settings
   - next implementation gate

## Required Artifacts Per Run

- summary.txt
- delay_summary.csv
- burst_summary.csv
- run.log
- one-line verdict with hypothesis tag (H1/H2/H3)

Store under DESIGN-CONTROLLER/bench-evidence/T6_* and cross-link in TODO.md.

## Recommended Next Gate After This Plan

If H1 is accepted and no ACK uplift appears, shift objective from "find better delay" to "automate independent retries with deterministic stop-on-success behavior" and treat noise cleanup as a parallel quality task, not a blocker for entry reliability.
