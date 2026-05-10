# Controller Stage 1 Phase 7 Live-Runtime Policy-Loop Result (Copilot v1.0)

Date: 2026-05-09
Board: 2E2C1209DABC240B

## Scope

Test live intervention directly by keeping runtime active and repeatedly re-applying candidate lane policy, instead of applying policy only after halting.

## Added Configs

- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/44_phase7_live_laneB_af_loop.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/45_phase7_live_laneA_policy_loop.cfg`
- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/46_phase7_live_laneC_af_loop.cfg`

## Intervention Design

Each config performs:

1. `BOOT0 low + NRST pulse` to enter user-app path
2. `resume` (runtime active)
3. 100-iteration loop, 20 ms cadence:
   - lane B: AF re-assert on `PA9/PA10/PB10/PB11/PC9`
   - lane A: AF re-assert on `PA2/PA3/PA15` + `PB14/PB15` input pull-up re-assert
   - lane C: AF re-assert on `PC10/PC11/PC12/PC9/PE11/PE12`
4. `shutdown` and immediate Stage 1 probe

## Evidence Folder

- `DESIGN-CONTROLLER/bench-evidence/T6_phase7_live_policy_2026-05-09_190504/`

## Integrity Checks

OpenOCD logs show loop markers in all three runs:

- lane B: `LIVE_AF_INJECT laneB iter=0/20/40/60/80`
- lane A: `LIVE_POLICY_INJECT laneA iter=0/20/40/60/80`
- lane C: `LIVE_AF_INJECT laneC iter=0/20/40/60/80`

This confirms live-loop intervention actually executed during runtime.

## Stage 1 Outcome

All three probe logs (`44..46`) show unchanged failure signature:

- `STATS_URC(init)` with `host_rx_bytes=0`
- no `BOOT_URC` in initial observation window
- `ATI` and `AT+VER?` both decode to same recurring `STATS_URC` frame
- `FATAL timeout waiting for response type 0x81 to req 0x01`

## Conclusion

Phase 7 is negative: live GPIO/AF policy overwrite at this layer still does not restore ingress.

Interpretation:

- root cause is likely above pin mux snapshots (runtime peripheral transaction/owner policy level),
- or in a different gating domain not affected by these GPIO+AF loop writes.

## Recommended Next Discriminator (Phase 8)

Switch to firmware-level runtime instrumentation around host RX ingest path:

1. add explicit counters/trace IDs at ingress branch points (before and after parser gate)
2. emit branch reason code in periodic status URC
3. run controlled A/B with one forced runtime owner policy branch inside firmware

Success criterion: first non-zero delta in `host_rx_bytes` and arrival of any non-`STATS_URC` response to active request.
