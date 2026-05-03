# HIL LoRa Test Execution Review v1.0
Date: 2026-04-30
Reviewer: GitHub Copilot (GPT-5.3-Codex)

## Scope reviewed
- `LifeTrac-v25/DESIGN-CONTROLLER/HIL_RUNBOOK.md`
- `LifeTrac-v25/MASTER_TEST_PROGRAM.md`
- `LifeTrac-v25/DESIGN-CONTROLLER/TODO.md` (Phase 2.LoRa + W4 gates)
- `LifeTrac-v25/DESIGN-CONTROLLER/LORA_IMPLEMENTATION.md`

## Executive verdict
Status for the requested session ("connect both Max Carriers to this laptop, verify LoRa link, triage issues, then optimize"): READY WITH GATES.

The plan in-repo already matches this intent if executed in this order:
1. W4-pre (board bring-up sanity, no RF)
2. W4-00 (dual-Portenta LoRa stack bench)
3. Triage if any W4-00 failure occurs
4. Optimization passes (ladder/capacity/durability)
5. Expand into additional hardware/control gates W4-01 and onward

## Key findings (review-first)

### Critical
1. Gate order is mandatory: W4-pre must be completed before W4-00.
- Why this matters: plugging in and testing LoRa immediately without W4-pre can hide board-level faults as radio faults.
- Source: HIL runbook and master matrix explicitly define W4-pre as the front gate before any RF.

### High
2. W4-00 pass criteria are strict enough for meaningful triage and should be treated as go/no-go.
- 1000-frame bursts at SF7/SF8/SF9 with 0 losses at 1 m.
- RTT distribution checks tied to model expectations.
- Payload-size parity checks against the Python mirror.
- This gives immediate branch points for radio, framing, crypto, or scheduler faults.

3. The test objective should be split into two durability windows.
- Initial connection bring-up (fast): prove link exists and decodes consistently.
- Stability soak (longer): verify no drift, replay-window anomalies, or queue starvation under sustained traffic.
- This aligns with your request for both "efficient" and "durable" link behavior.

### Medium
4. Post-connection optimization should focus on bounded, measurable knobs.
- Prioritize SF ladder behavior, packet-loss tails, and airtime utilization.
- Keep optimization tied to explicit metrics from W4-00/W4-02 artifacts to avoid subjective tuning.

## Operator sequence for this session

### Phase A: Plug in and board sanity (W4-pre)
- Plug both Portenta + Max Carrier stacks into this laptop over USB-C.
- Confirm both enumerate as distinct serial devices.
- Verify simple serial stability and board health checks.
- Keep LoRa antennas disconnected during W4-pre.

Exit criteria:
- Both boards stable on USB and booting cleanly.

### Phase B: LoRa bring-up (W4-00)
- Attach 915 MHz antennas and keep them separated by at least 30 cm.
- Run dual-console TX/RX checks at SF7, SF8, SF9.
- Execute round-trip latency sampling.
- Verify encrypted on-air lengths versus expected frame model.

Exit criteria:
- Link established and repeatable at bench distance.

### Phase C: Triage on failure
If failures appear, triage in this order:
1. Board/USB/boot instability
2. PHY mismatch (SF/BW/CR/sync)
3. Key/provisioning or decrypt failures
4. Frame-format/CRC drift
5. Timing/queue/IRQ tail spikes

### Phase D: Optimize after first success
- Tune for efficiency: reduce retransmission pressure and queue contention.
- Tune for durability: stabilize p99 latency and loss tails under repeated bursts.
- Record any parameter changes with before/after metrics.

### Phase E: Prepare additional hardware/control testing over LoRa
After W4-00 is stable, stage for W4-01+:
- E-stop latency chain
- watchdog trip behavior
- Modbus failure behavior
- mixed-mode/ramp gates

## Ready-to-plug signal
Ready when:
- Two USB-C cables are available
- Two 915 MHz antennas are available
- You can open two serial monitors on this laptop

At that point, start with W4-pre first, then W4-00.
