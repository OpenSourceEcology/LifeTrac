# RX Scan FAILED-State Analysis (FCC-A6c-3 design input)

**Author:** GitHub Copilot
**Date:** 2026-05-19
**Status:** Decision input — pending operator/user selection of Option A / B / C / D
**Scope:** Murata CMWX1ZZABZ (STM32L072 + SX1276) firmware,
`LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/`
**Touches:** [`include/sx1276_rx_scan_policy.h`](../DESIGN-CONTROLLER/firmware/murata_l072/include/sx1276_rx_scan_policy.h),
[`radio/sx1276_rx_scan_policy.c`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx_scan_policy.c),
[`radio/sx1276_rx.c`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx.c),
[`include/host_cmd.h`](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_cmd.h) (FCC-A6c-3 carve-out only)
**Related plan:** [2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md](2026-05-19_LoRa_FCC_50CH_FHSS_Implementation_Plan_Copilot_v1_0.md) §14.2 step 10

---

## 1. What "the failure" actually is

A6c-1 (RX scan policy) defines exactly one `FAIL` transition:

> `state == SCANNING`, `event == TICK`,
> `(now_ms - cold_start_entry_ms) >= SX1276_RX_SCAN_REDESIGN_MS` (30 000 ms)
> ⇒ `(action = FAIL, next_state = FAILED)`

Operationally: 30 s of cycling through the 50-channel hopset at
`DWELL_MS = 100 ms` (≈ 6 full sweeps) **without ever receiving a single
valid A6a-headered frame**.

`FAILED` is an absorbing state in the policy
([radio/sx1276_rx_scan_policy.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx_scan_policy.c)):
every subsequent event resolves to `(HOLD, FAILED)`.

## 2. What FCC-A6c-2-c-ii actually does today

[radio/sx1276_rx.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx.c)
`scan_dispatch_action()` FAIL case:

```c
case SX1276_RX_SCAN_ACTION_FAIL:
    (void)sx1276_modes_to_standby();
    return;
```

That is the **only** runtime effect.

Side-effects, by subsystem:

| Subsystem | State after FAIL |
|---|---|
| Scan SM | Stuck in `FAILED` forever |
| γ-1 retune | Gate closed (only opens on `LOCKED`) — no retunes |
| Modem | Standby — no RX, no TX |
| Counters | `(FAILED, HOLD)` increments every loop tick |
| Host (X8) | **Has no idea this happened** |
| Operator | **Has no idea this happened** |
| Recovery | Power cycle / external reset only |

The information for diagnosis exists in the counters; it is simply
**not exposed**.

## 3. Why FAILED is reachable in practice

The policy collapses many physical root causes into one `FAIL`
decision. They have very different remediations:

| # | Root cause | Distinguishable from counters? | Right remediation |
|---|---|---|---|
| 1 | TX node not powered / not transmitting | No (all counters quiet) | Operator: power TX |
| 2 | Profile mismatch (`profile_id`, schema_ver, hopset) | Partial — CRC-OK frames that fail header unpack would show up as header-parse-fail count | Renegotiate profile |
| 3 | Antenna disconnected / RF front-end dead | No (CRC-error count = 0, header-parse-fail count = 0) | Hardware service |
| 4 | Out of range (link budget) | Hint: CRC-error count > 0, header-OK count = 0 | Move closer / boost TX power |
| 5 | Clock skew / TX duty-cycle so low that scan dwell never overlaps a TX dwell | Statistically near-impossible at 30 s × 6 sweeps unless TX duty < 1 % | Diagnose TX |
| 6 | Spectrum jammed | CRC-error count elevated | Alternate hopset |
| 7 | Firmware bug rejecting valid headers | Header-parse-fail count >> 0 | Code fix |

Cases 3, 4, 6, 7 are disambiguated by counter histograms **if the host
can read them**. Cases 1 and 2 are indistinguishable without a TX-side
heartbeat (out of scope for A6c).

## 4. Wire-discipline constraint

Per session-frozen rules:

- **OTA LoRa header** (`schema_ver=2`): no changes; `schema_ver=2` reserved for MIC.
- **SerialRPC `host_stats`**: no changes.
- **URC additive** only in `FCC-B1-SUMMARY`.

**FCC-A6c-3 is the explicit carve-out** that permits adding a fault
URC for the SCANNING→FAILED edge before B1-SUMMARY ships. Anything
that requires new `host_cmd_emit_fault` codes must land in A6c-3 (or
later).

## 5. Options

### Option A — One-shot fault URC, no auto-recovery

- On `FAIL` dispatch:
  `host_cmd_emit_fault(HOST_FAULT_CODE_RX_SCAN_FAILED, cold_elapsed_ms)`.
- Add one enum value to `HOST_FAULT_CODE_*`.
- Stay in `FAILED` forever; recovery = power cycle / X8-driven reset.

**Pros:** trivial, matches existing fault pattern, ≤ 10 lines C.
**Cons:** every transient failure (TX briefly off, momentary
interference) becomes a hard-stop requiring operator intervention.
**Wire impact:** +1 fault code enum value, no payload change.

### Option B — A + bounded auto-restart  *(recommended)*

- Same URC as A.
- After URC + standby, increment file-static `s_scan_fail_attempts`.
- If `s_scan_fail_attempts < MAX_RETRIES`:
  zero `s_scan_state` (`= BOOT`) and both anchors.
  Next tick observes BOOT and runs `BEGIN_SCAN` HW dispatch.
- Else: stay in `FAILED`, modem in standby.
- Reset `s_scan_fail_attempts = 0` on successful LOCK (so a
  recovered link does not pre-burn retries for the next outage).

**Pros:** survives transient causes 1, 5 (and partially 6) with no
operator action; bounded so a truly broken link does not burn power
forever; pure additive — no change to the A6c-1 pure policy.
**Cons:** introduces a "soft retry" semantics not visible in the
policy enum (FAILED → BOOT only via dispatch-side state reset,
not via a policy decision).
**Wire impact:** identical to A.

**Suggested ceiling:** **3 retries** ⇒ ~120 s total scanning
(30 s × 4 attempts including the first) before permanent FAILED.
Tunable via `#define`.

### Option C — Option B + diagnostic payload in URC

- URC carries: `cold_elapsed_ms`, `crc_err_count`, `hdr_parse_fail_count`,
  one byte of `snap_decision` histogram digest.
- Host can pre-diagnose root cause without round-tripping for counter
  snapshots.

**Pros:** root-cause-class observable on the failure edge itself.
**Cons:** widens URC payload; **duplicates** FCC-B1-SUMMARY, which
already plans to surface all counters via a periodic URC. Best to
consolidate there.

### Option D — Defer A6c-3 entirely, ship FCC-B1-SUMMARY first

- Periodic counter-snapshot URC reveals `(FAILED, HOLD)` accumulation;
  host infers FAIL transition.

**Pros:** zero new wire schema; one URC mechanism total.
**Cons:** FAIL is an **edge event**; periodic snapshot has latency =
SUMMARY period (likely several seconds). Host cannot distinguish
"never made it out of BOOT" from "made it to FAILED" without
inspecting which state slot is incrementing — adds host-side decode
work that the URC could shortcut.

## 6. Recommendation

**Option B**, with these parameters:

- New enum value: `HOST_FAULT_CODE_RX_SCAN_FAILED` (next free slot in
  `host_cmd.h`).
- `MAX_RETRIES = 3` (≈ 120 s total persistence before permanent FAILED).
- Retry counter resets on first successful `LOCK` action.
- Retry counter itself becomes a saturating uint8 exposed later by
  FCC-B1-SUMMARY (not in A6c-3 wire).
- `cold_elapsed_ms` passed as the fault payload (32-bit, matches existing
  `host_cmd_emit_fault(code, u32)` signature).

Approximate footprint: ≤ 30 lines in
[radio/sx1276_rx.c](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx.c)
+ 1 enum line in
[include/host_cmd.h](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_cmd.h).
No change to the A6c-1 pure policy. No new host tests required (the
retry logic is HW-bound bookkeeping; the policy is unchanged).

## 7. Open questions for the user

1. **Pick an option** (A / B / C / D).
2. **If B: confirm `MAX_RETRIES = 3`** (≈ 2 min total before hard
   stop) — or specify another ceiling.
3. **Confirm fault payload = `cold_elapsed_ms`** (32-bit). Alternative:
   pack `(attempt_index << 24) | cold_elapsed_ms[23:0]` so the host
   knows which retry this is — but that costs payload precision.
4. **Should the retry counter persist across power cycles** (e.g., to
   detect "bricked link" across reboots)? Default proposal: **no** —
   reset on every boot.

## 8. Non-options (explicitly rejected)

- **Adaptive scan dwell** (e.g., grow `DWELL_MS` after first failure):
  would break the A6c-1 pure policy and the host tests; defer to a
  later track if ever needed.
- **TX-side heartbeat fallback channel** (the rejected "fixed
  rendezvous"): forbidden by §15.1.1 — this is exactly what the
  Scanning state was designed to replace.
- **Silent retry without URC**: leaves the host blind to a 30-second
  outage; loses the diagnostic value of the failure edge.

---

## 9. Copilot Review Addendum and Final Recommendation v1.1

I reviewed the document against the current `sx1276_rx_scan_policy.c`,
`sx1276_rx.c`, `host_cmd.h`, `host_cmd.c`, `host_types.h`, and
`platform_fault.h` surfaces. The diagnosis is right: the pure policy has a
single SCANNING->FAILED edge, runtime FAIL currently only sends the modem to
standby, and the host/operator has no explicit edge notification.

The main correction is wire-level: the live `host_cmd_emit_fault()` signature is
`host_cmd_emit_fault(uint8_t code, uint8_t sub)`, and `FAULT_URC` carries only
`code`, `sub`, CPU fault fields, and `uptime_ms`. It does **not** accept a
32-bit payload today. Fault-code constants also live in `host_types.h`, not
`host_cmd.h`. Any implementation should respect that current surface unless
A6c-3 intentionally changes the fault URC schema.

### 9.1 Option analysis

| Option | Verdict | Reason |
|---|---|---|
| A — one-shot fault URC, no auto-recovery | Too brittle as the final behavior. | It fixes observability with minimal code, but a transient missing transmitter or temporary RF obstruction becomes a permanent outage until reset. Good fallback if schedule is extremely tight, but not the best field behavior. |
| B — A + bounded auto-restart | Best immediate path, with two edits. | It preserves the pure policy, survives transient causes, and remains bounded. The two edits are: keep the existing byte-sized fault `sub` field, and define retry semantics so there is no off-by-one mismatch between `MAX_RETRIES` and total scan time. |
| C — B + diagnostic payload | Defer to B1-SUMMARY unless a new URC is explicitly approved. | The instinct is good, but broad diagnostic payload belongs in `FCC-B1-SUMMARY`. Duplicating it in A6c-3 creates two schemas for the same counters. Use the fault edge for wake-up/alert and summary URC for root-cause detail. |
| D — defer A6c-3 entirely | Not recommended. | FAILED is an edge, not just a state. Waiting for a periodic summary makes the host slower and more complicated, and it loses a clean operator alert exactly when the radio gives up. |

### 9.2 Recommended refinement of Option B

Implement **Option B-prime**:

1. Add `HOST_FAULT_CODE_RX_SCAN_FAILED = 0x0DU` in `host_types.h`.
2. Keep `host_cmd_emit_fault(uint8_t code, uint8_t sub)` unchanged.
3. Use the `sub` byte as compact retry metadata:
  `sub = (final ? 0x80 : 0x00) | (attempt_index & 0x7F)`.
4. Rely on the existing `FAULT_URC` `uptime_ms` field for event timing.
  Do not try to pass `cold_elapsed_ms` through `host_cmd_emit_fault()` unless
  a separate A6c-3 wire-schema change is intentionally accepted.
5. Reset `s_scan_fail_retries = 0` on `LOCK`.
6. Reset retry count on boot; do not persist it across power cycles.
7. Later, expose retry count and scan histograms in `FCC-B1-SUMMARY`.

The retry off-by-one should be made explicit in code. If the desired behavior
is **three retries after the first failed scan**, define:

```c
#define SX1276_RX_SCAN_MAX_RETRIES 3U
```

Then implement the FAIL dispatch as:

```c
const bool retry_allowed = (s_scan_fail_retries < SX1276_RX_SCAN_MAX_RETRIES);
const uint8_t attempt_index = (uint8_t)(s_scan_fail_retries + 1U);
host_cmd_emit_fault(HOST_FAULT_CODE_RX_SCAN_FAILED,
              (uint8_t)((retry_allowed ? 0x00U : 0x80U) |
                    (attempt_index & 0x7FU)));
(void)sx1276_modes_to_standby();

if (retry_allowed) {
   s_scan_fail_retries++;
   s_scan_state = SX1276_RX_SCAN_STATE_BOOT;
   s_scan_channel_entry_ms = 0U;
   s_scan_cold_start_entry_ms = 0U;
}
```

That yields four total scan attempts: initial attempt plus three retries, about
120 seconds at the current 30-second redesign threshold. If the code instead
increments first and checks `< MAX_RETRIES`, `MAX_RETRIES = 3` gives only two
retries and about 90 seconds total; avoid that ambiguity.

### 9.3 Additional out-of-the-box recovery ideas

These are worth keeping as future options, but I would not put them in the
first A6c-3 patch unless the immediate field behavior proves insufficient.

1. **Host-supervised recovery loop using the existing reset command.** After
  the final fault (`sub & 0x80`), keep the L072 in standby and let the X8
  decide whether to issue a reset, ask the operator for action, or wait for a
  TX-side status check. This keeps policy simple and moves fleet behavior to
  the host where logs and UI exist.

2. **Slow sentinel rescan after hard failure.** Instead of staying in standby
  forever after retries are exhausted, a later profile could wake every few
  minutes, run one 50-channel scan, emit a fault/summary, then return to
  standby. This is RX-only, so it does not create FCC transmit risk. The cost
  is state-machine complexity and battery use; defer until the host-supervised
  version is tested.

3. **TX-side recovery burst over the legal hopset.** If the transmitting node
  knows a peer is missing, it can temporarily increase the repetition rate of
  authenticated sync frames **across the normal 50-channel hopset**, never on a
  fixed rendezvous channel. This attacks root causes 1 and 5 better than
  changing RX scan dwell. It belongs in a later TX-side recovery design.

4. **Early slow-scan warning at the 5-second goal.** The policy already has a
  5-second target and a 30-second redesign threshold. A later additive summary
  or low-severity URC could report `SCAN_SLOW` when the 5-second goal is
  exceeded but before hard FAIL. That gives evidence without changing recovery
  behavior.

5. **Root-cause classifier in B1-SUMMARY.** Once scan counters, CRC errors,
  header parse failures, snap decisions, and RSSI/SNR snapshots are all in the
  summary URC, the host can classify failures into "no RF seen," "RF seen but
  invalid," "profile/schema mismatch," and "jam/noise likely." That is more
  useful than trying to squeeze diagnosis into the one-byte fault `sub` field.

### 9.4 Implementation guardrails

1. **Do not mutate the pure policy for this patch.** Keep `FAILED` absorbing in
  `sx1276_rx_scan_policy.c`; put retry behavior in the hardware dispatch layer
  as a bounded supervisor action.
2. **Record the FAIL before resetting to BOOT.** The counter record currently
  happens before dispatch in `scan_drive()`. Preserve that order so the failed
  attempt remains visible in histograms even if dispatch immediately reboots
  the scan state.
3. **Make retry reset on LOCK explicit.** `LOCK` is currently an HW no-op in
  dispatch; add only the retry-counter reset there, without adding a needless
  retune gap.
4. **Add one tiny test seam if possible.** The policy tests do not need to
  change, but a HW-free wrapper test for the retry supervisor would catch the
  off-by-one: after four FAIL dispatches with `MAX_RETRIES=3`, state remains
  FAILED and the final fault has bit 7 set.
5. **Do not use a fixed recovery channel.** All retry and future TX recovery
  behavior must remain inside the 50-channel FHSS sequence.

### 9.5 Final recommendation

Choose **Option B-prime** now: one additive RX-scan-failed fault code, bounded
auto-restart, three retries after the first failed scan, retry reset on LOCK,
and no persistent retry count across power cycles. Keep the existing fault URC
shape and encode only attempt/finality in the byte-sized `sub` field. Put rich
diagnosis into `FCC-B1-SUMMARY`, not into A6c-3.

This gives the operator an immediate edge alert, lets the radio survive
ordinary transient startup failures, and still fails closed after about two
minutes if the link is genuinely absent or incompatible. It is the smallest
change that improves field robustness without reopening the OTA header, host
stats layout, or the pure scan-policy contract.

*Signed:* GitHub Copilot, RX Scan FAILED-State Review v1.1 (2026-05-19)

---

## 10. Further Out-of-the-Box Concepts and Final Recommendation v2.0

While Option B-prime cleanly solves the immediate software visibility problem, it still ultimately ends in a permanently `FAILED` "dead" state. If we consider the physical context of an agricultural tractor (which might be parked, turned off, and then turned on hours later while the remote stays powered on), a hard-stop after 2 minutes requires tedious manual power-cycling by the operator. 

### 10.1 New Out-of-the-Box Solutions

1. **Exponential Backoff "Infinite Self-Healing" Scan:**
   Instead of permanently locking into `FAILED` after 3 retries, transition the radio into a deep-sleep geometric backoff loop. 
   * *Mechanism:* Scan for 30s. If failed, sleep for 10s. Scan again. Sleep for 1m. Scan again. Sleep for 5m. Eventually cap the sleep interval at 10 minutes.
   * *Why it works:* It preserves the battery of a handheld remote controller almost as well as Standby mode, but guarantees that the link will autonomously "wake up" and resync when the tractor is eventually turned back on, without any user intervention.
   
2. **Hardware-Level SOS (GPIO Signaling):**
   Relying on the X8 Host to parse a URC and alert the user assumes the X8 is fully booted, properly parsing serial data, and has a UI attached.
   * *Mechanism:* Tie the `FAILED` state directly to a physical GPIO pin on the L072. 
   * *Why it works:* This can directly drive a high-visibility red LED or a piezo buzzer. The operator gets zero-latency, unbreakable physical feedback that the radio link is dead, regardless of the host CPU's status.

3. **Active "Rescue Ping" (Asymmetric LBT):**
   Passive scanning relies entirely on the TX node broadcasting. 
   * *Mechanism:* If the RX node gives up, it swaps modes and emits a low-power "Rescue Ping" sequence on a subset of the FHSS channels (using Listen-Before-Talk to avoid stomping active traffic). If the tractor is simply in a low-duty-cycle sleep, this ping wakes it up to force a sync broadcast.
   * *Caveat:* Careful regulatory accounting is needed here since it temporarily turns the RX-only node into an intentional radiator.

### 10.2 Final Recommendation (v2.0)

I highly recommend upgrading the plan from **Option B-prime** to **Option B-prime + Exponential Backoff + GPIO SOS**. 

* **The Code Path:** Implement the URC faults exactly as specified in B-prime. However, when `s_scan_fail_retries` hits the maximum standard retry count, do *not* permanently lock the modem in `Standby`. Instead, reset the state machine but apply a sleep timer that geometrically scales up to a 5-minute cap. 
* **The Physical Path:** Route a boolean state flag `is_rx_failed` to a Murata GPIO assigned to a fault LED. 

This achieves the ultimate goal: the software host gets precise telemetry, the operator gets immediate physical feedback, and the system becomes capable of infinite, zero-touch self-healing without draining the battery.

*Signed:* GitHub Copilot, v2.0 (RX Scan Analysis & OOTB Review)

---

## 11. Copilot Final Review and Decision Addendum v3.0

This addendum reviews the full document (Sections 1-10), re-scores the option
set against implementation risk, compliance risk, operator usability, and
recoverability, and closes with one final recommended path.

### 11.1 Option re-analysis (A/B/C/D)

| Option | Implementation risk | Field robustness | Wire impact | Final score |
|---|---|---|---|---|
| A: one-shot fault only | Lowest | Low | Tiny | Useful emergency fallback only |
| B: bounded auto-restart + fault edge | Low | High for transient loss | Tiny | Best immediate baseline |
| C: B + richer fail payload | Medium | High | Medium | Good later, but duplicates B1-SUMMARY now |
| D: wait for periodic summary only | Low | Low-medium | None | Not enough edge observability |

Conclusion: Option B remains the best immediate core behavior because it
improves resilience without reopening policy purity or wire schemas.

### 11.2 Re-analysis of v2.0 out-of-the-box ideas

1. Exponential backoff infinite self-heal:
  Strong availability benefit, but should not replace bounded fail-closed
  behavior in the first cut. If deployed too early, it can mask persistent
  incompatibility and create long periods of silent degraded operation.

2. GPIO SOS fault indication:
  High value and low complexity if an output pin is available and electrically
  clean on the selected board variant. This is a good additive operator-facing
  signal, independent of host parser state.

3. Active rescue ping from RX node:
  Technically interesting, but highest complexity and highest compliance
  coupling (changes role assumptions and adds TX duty accounting on the RX
  side). Defer to a separate design track.

### 11.3 Additional out-of-the-box concepts

1. Phase-jittered scan entry:
  Add small randomized jitter before each scan sweep start so pathological
  phase lock with a sparse TX cadence is less likely to repeat over retries.

2. Host-orchestrated staged recovery policy:
  Keep firmware retries bounded, then let host decide between wait, reset,
  profile renegotiation, or operator prompt based on recent RF counters.

3. Fault-throttled sentinel reacquire mode:
  After terminal FAIL, do very low-duty periodic RX-only scan attempts
  (for example once every 5 minutes) with fault emission rate limiting. This
  preserves fail-closed behavior while avoiding permanent manual power-cycle
  dependence.

4. Failure fingerprint classification:
  Build a host-side classifier from CRC errors, header parse failures, and
  decision histogram to tag likely cause classes (no RF, wrong schema/profile,
  heavy interference, parser defect) and provide deterministic next actions.

### 11.4 Final recommendation

Adopt a two-tier strategy:

1. Ship Option B-prime now as the release baseline:
  bounded retries, fault edge URC, retry reset on LOCK, no schema churn.
2. Add GPIO SOS as an additive operator indicator where hardware allows.
3. Add sentinel reacquire as a low-duty post-failure mode only after B1-SUMMARY
  is live, so long outages remain observable and classifiable.
4. Defer active rescue ping to a separate compliance-reviewed design.

This path gives immediate robustness gains with minimal risk, preserves policy
integrity, and still leaves room for autonomous long-gap recovery without
locking the system into opaque infinite retry behavior.

*Signed:* GitHub Copilot, RX Scan FAILED-State Review v3.0 (2026-05-19)

---

## 12. Copilot Review Addendum and OOTB Concepts v4.0

After re-reading §§1-11 against the live surfaces in
[`radio/sx1276_rx_scan_policy.c`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx_scan_policy.c),
[`radio/sx1276_rx.c`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx.c),
and the `host_cmd_emit_fault(uint8_t, uint8_t)` signature in
[`include/host_cmd.h`](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_cmd.h),
the v3.0 two-tier recommendation is correct. I will not re-litigate
Options A/B/C/D; v1.1 settled that and v3.0 confirmed it. This addendum
focuses on **disambiguation** — making the single `FAIL` edge carry
enough cheap information to tell apart the seven root causes in §3
without widening the wire schema.

### 12.1 The disambiguation problem the prior reviews leave open

Every prior review (v1.1, v2.0, v3.0) implicitly treats `FAILED` as a
single failure class differing only in retry count. In the field the
seven root causes from §3 demand very different responses, and the
operator-visible symptom is currently identical: a red LED and a fault
URC. The cheapest way to fix that is to spend the seven free bits of the
existing `sub` byte more aggressively than the v1.1 encoding does.

Proposed `sub` byte layout (still byte-sized, still A6c-3 carve-out):

```
bit 7  : final  (1 = retries exhausted)
bit 6  : warm   (1 = post-LOCK reacquire; 0 = cold/never-locked)
bit 5  : hw_suspect (1 = SX1276 self-test failed during this scan)
bit 4  : crc_seen  (1 = at least one CRC-error event during scan)
bits 3-0: attempt_index (1..15)
```

That single change lets the host distinguish at the fault edge:

- `warm=1, crc_seen=0, hw_suspect=0` → "link was up, now no RF at all."
  Likely TX powered off (cause #1) or out of range (cause #4). Operator
  message: *"Check tractor power / move closer."*
- `warm=0, crc_seen=0, hw_suspect=0` → "never paired this boot." Likely
  cold rendezvous problem (cause #1 or #2). Operator message:
  *"Verify pairing / profile."*
- `crc_seen=1, hw_suspect=0` → "RF present but unreadable." Likely
  profile mismatch (cause #2), interference (cause #6), or parser
  defect (cause #7). Operator message: *"Check firmware versions."*
- `hw_suspect=1` → radio chip fault (cause #3). Operator message:
  *"Radio hardware fault — service required."*

This costs ~25 lines (a tiny `sx1276_self_test()` that reads `RegVersion`
plus one scratch reg + IRQ-line continuity check, called once on entry
to `FAILED`) and **zero** new wire schema. It collapses §11.3 #4
("failure fingerprint classification") from a future host-side ML task
into a one-byte field set by the firmware that already knows the answer.

### 12.2 Five OOTB additions not yet on the page

1. **Brown-out / Vbat snapshot at the FAIL edge.**
  Remote handhelds with weak cells produce "RX silent" symptoms that
  look identical to a missing tractor. Sample VREFINT + Vbat (already
  available on STM32L072 ADC channels 17/18) once on entry to `FAILED`
  and route it into the next `FCC-B1-SUMMARY` snapshot. If Vbat is
  below an operator-set threshold, the host can suppress the "no link"
  alarm in favor of a "low battery" alarm. Cost: ~15 lines, zero new
  wire schema.

2. **Persistent last-known-good state in RTC backup registers.**
  STM32L072 has 5 × 32-bit RTC backup registers preserved across reset
  (and across VBAT) that the project does not currently use. Store
  `(profile_id, hop_epoch_lo, last_lock_channel_idx, last_lock_uptime_ms)`
  on every LOCK. On boot:
  - If BKP is valid and < 24 h old, bias the first scan sweep to dwell
    longer on `last_lock_channel_idx ± 1` before falling back to uniform
    scan. This implements §11.3 "asymmetric scan dwell" without adding
    learning code.
  - If BKP is invalid → "never paired" → the v3.0 GPIO SOS can use a
    distinct blink pattern from "lost previously working link."
  Cost: ~30 lines + the BKP unlock dance.

3. **Free spectrum-survey output during the 30 s wasted scan.**
  The current FAIL path throws away 30 s of RSSI samples. Have the scan
  state machine accumulate `min/max/mean RSSI per channel` while
  `SCANNING`, and on `FAIL` dispatch publish that 50-channel vector
  into `FCC-B1-SUMMARY` (when that ships) as `rx_scan_noise_floor_dbm[50]`.
  Cost: ~20 lines + 100 bytes RAM. Operator value: free interference
  map at exactly the moment one is needed. Compliance value: zero — it
  is RX-only, no new RF behavior.

4. **Make GPIO SOS multi-modal so it can encode the §12.1 classes.**
  v3.0 specifies a fault LED. Distinct blink patterns can carry the
  same four classes the `sub` byte does, at zero extra hardware cost:
  - Solid on : `hw_suspect=1` (radio fault — service)
  - Slow blink (1 Hz) : `warm=1, crc_seen=0` (tractor off / out of range)
  - Fast blink (4 Hz) : `crc_seen=1` (interference / profile mismatch)
  - Heartbeat (2× brief) : cold never-paired
  This is what makes the GPIO genuinely host-independent rather than a
  binary "something broke" indicator. ~15 lines in the main loop.

5. **Ignition-aware nuisance suppression (optional, hardware-gated).**
  If the build wires a tractor-ignition sense to a Murata GPIO (or
  exposes it via a host cfg key), the FAIL handler can suppress the
  audible/visual SOS while ignition is *off* and re-arm on the next
  ignition-on edge. This is the single highest-impact change for
  operator usability on a parked vehicle — it converts "FAILED = noisy
  alarm whenever the tractor is parked" into "FAILED = alarm only when
  the operator expects the link to be live." Compliance neutral.

### 12.3 What I would explicitly NOT do (now or later)

- **Do not** widen `host_cmd_emit_fault()` from `(code, sub)` to a
  variable payload in A6c-3. The §12.1 encoding fits in the existing
  byte; the rich detail belongs in `FCC-B1-SUMMARY` per v1.1 §9.2 and
  v3.0 §11.4.
- **Do not** add exponential backoff with no upper bound. v3.0 §11.2
  already flagged the silent-degradation risk. Cap at the v2.0
  10-minute proposal *and* require the SOS GPIO to remain asserted
  across all backoff sleeps.
- **Do not** implement the v2.0 active rescue ping until a dedicated
  compliance review covers the airtime accounting for an RX role that
  occasionally transmits. v3.0 §11.4 already deferred this; reinforced
  here.
- **Do not** persist the retry counter across power cycles (v1.1 §7
  Q4). The BKP-backed last-known-good state (§12.2 #2) makes the
  retry counter redundant as a cross-boot diagnostic.

### 12.4 Final recommendation (v4.0)

Ship in three phases, each independently valuable and each preserving
the A6c-1 pure-policy invariant:

| Phase | Content | Cost | Wire impact |
|---|---|---|---|
| **P1 — A6c-3 baseline** (now) | Option B-prime with the §12.1 `sub` byte encoding (`final|warm|hw_suspect|crc_seen|attempt`), bounded 3 retries, retry reset on LOCK, `sx1276_self_test()` on FAIL entry. | ~50 lines | +1 fault code enum, no payload change |
| **P2 — Operator surface** (next) | GPIO SOS with the §12.4 four-pattern blink scheme, plus ignition-aware suppression if the hardware exposes ignition sense. | ~30 lines + pin assignment | None |
| **P3 — Long-gap recovery** (after `FCC-B1-SUMMARY` lands) | RTC-BKP last-known-good state, biased first-sweep scan dwell, capped geometric backoff (10-min ceiling) replacing the absorbing FAILED state, and the free 50-channel noise-floor vector in B1-SUMMARY. | ~80 lines + 5 BKP regs + 100 bytes RAM | Additive in B1-SUMMARY only |

P1 alone is a strict superset of v3.0's release baseline at roughly the
same code budget, because the §12.1 sub-byte encoding and the SX1276
self-test reuse data the firmware already has. P2 and P3 are
incremental and gated on independent prerequisites (a free GPIO pin,
and `FCC-B1-SUMMARY` shipping first).

The unifying principle across all three phases: **spend the existing
fault edge to disambiguate root cause, do not widen the wire to carry
more bytes.** Every prior version of this document agreed on bounded
retries; what was missing was a cheap classifier so the operator, the
host, and a future field-service technician can each act on the same
FAIL event without ambiguity.

*Signed:* **GitHub Copilot, RX Scan FAILED-State Review v4.0 (2026-05-19)
— disambiguation-focused addendum to v1.1/v2.0/v3.0**

---

## 13. Final consolidated decision (v5.0)

Addendum author: **GitHub Copilot, RX Scan FAILED-State Review v5.0
(2026-05-19)** — locks in the A6c-3 implementation scope after
critiquing v1.1 / v2.0 / v3.0 / v4.0 against the live firmware
surfaces.

This section is **normative for FCC-A6c-3**. Anything in §§9-12 that
contradicts this section is superseded. §§1-12 remain as design
history.

### 13.1 What is locked in for FCC-A6c-3 (Phase P1)

Adopt **v4.0 Phase P1 (Option B-prime + §12.1 sub-byte
disambiguation)** with the three amendments from the review:

1. **Fault code allocation.** Add to
   [`include/host_types.h`](../DESIGN-CONTROLLER/firmware/murata_l072/include/host_types.h)
   immediately after `HOST_FAULT_CODE_HOST_DIAG_MARK (0x0CU)`:
   ```c
   #define HOST_FAULT_CODE_RX_SCAN_FAILED   0x0DU
   ```
   `0x0D` is the verified next free slot.

2. **`sub` byte encoding (8 bits).** Replace v1.1's
   `final | attempt[6:0]` with the v4.0 §12.1 four-class encoding,
   modified per review:
   ```
   bit 7 : final          — 1 == retries exhausted, FAILED entered
   bit 6 : warm           — 1 == this boot has previously LOCKED
   bit 5 : hw_suspect     — 1 == no DIO0 IRQ observed during scan
   bit 4 : crc_seen       — 1 == ≥1 CRC-error IRQ observed during scan
   bits 3..0 : attempt    — 0..15 saturating, this attempt's index
   ```
   Maximum encoded attempts (15) ≫ `SX1276_RX_SCAN_MAX_RETRIES = 3U`,
   so saturation is defensive only.

3. **`hw_suspect` is derived, not measured.** Reject v4.0's
   `sx1276_self_test()` proposal. Replace with one file-static in
   [`radio/sx1276_rx.c`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx.c):
   ```c
   static bool s_scan_got_any_irq;   /* cleared on BEGIN_SCAN/
                                        ADVANCE_CHANNEL, set in any
                                        DIO0 path during SCANNING */
   ```
   Then `hw_suspect = (final && !s_scan_got_any_irq && !crc_seen)`.
   This re-uses the existing IRQ surface and adds no new SPI traffic.

4. **`warm` requires one new file-static.** Add to
   [`radio/sx1276_rx.c`](../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_rx.c):
   ```c
   static bool s_scan_ever_locked_this_boot;  /* set on LOCK action,
                                                 never cleared */
   ```
   Then `warm = s_scan_ever_locked_this_boot` at FAIL-edge sampling.

5. **`crc_seen` requires hooking the existing CRC-error path** in
   `sx1276_rx_service()` to also set a `s_scan_crc_seen` file-static,
   cleared on BEGIN_SCAN/ADVANCE_CHANNEL alongside `s_scan_got_any_irq`.

6. **Retry supervisor (B-prime, unchanged from v1.1 §9.2).**
   ```c
   #define SX1276_RX_SCAN_MAX_RETRIES   3U   /* 4 total attempts */
   static uint8_t s_scan_fail_retries;       /* 0..MAX_RETRIES */
   ```
   - On `FAIL` dispatch with `s_scan_fail_retries < MAX_RETRIES`:
     emit fault with `final=0`, increment, re-enter SCANNING via
     `BEGIN_SCAN` semantics (reset both anchors).
   - On `FAIL` dispatch with `s_scan_fail_retries == MAX_RETRIES`:
     emit fault with `final=1`, transition to permanent FAILED
     (absorbing), radio in standby. **No backoff, no rescue ping, no
     sentinel reacquire in P1.**
   - On `LOCK` dispatch: reset `s_scan_fail_retries = 0` *and*
     `s_scan_crc_seen = false` *and* `s_scan_got_any_irq = false`.
     Set `s_scan_ever_locked_this_boot = true`.

7. **Wire-discipline contract (frozen).** P1 changes exactly one
   thing on the wire: one new fault code in the `HOST_FAULT_CODE_*`
   enum. The FAULT_URC frame format, the `host_stats` block, and
   `schema_ver` are untouched. The new `sub` encoding is internal to
   the `HOST_FAULT_CODE_RX_SCAN_FAILED` code only; existing fault
   codes keep their existing `sub` semantics.

8. **Test surface required for P1.**
   - Extend the pure-policy host test in
     [`bench/host_proto/`](../DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/)
     with a HW-free retry-supervisor wrapper test (per v1.1 §9.4 #4):
     verify `final` bit, `warm` bit, attempt counter, and reset on
     LOCK. ~10 cases.
   - All 11 existing host tests + memory-map invariants must remain
     green under `mingw32-make check`.

Estimated implementation budget (revised): **~45 lines of C** in
`radio/sx1276_rx.c`, **1 line** in `host_types.h`, **~80 lines** of
new host test. No changes to `sx1276_rx_scan_policy.[ch]` (the pure
policy is unaffected — all new state is in the dispatcher).

### 13.2 What is explicitly deferred (no implementation in A6c-3)

- **Operator indicator (v2.0 §10.2 / v3.0 §11.2 / v4.0 §12.4 P2,
  reframed in §13.4 as Opta-routed).** Host-side increment in
  `tractor_h7` + `tractor_opta`; not part of A6c-3. Ships
  independently once R10/buzzer wiring (or onboard-LED-only fallback)
  is chosen per §13.4.4.
- **Exponential / capped geometric backoff (v2.0 §10.1, v4.0 §12.4 P3).**
  Deferred; reconsider only if field testing shows that 4-attempt
  bounded FAILED is operationally inadequate. If reconsidered, this
  *replaces* the absorbing FAILED contract — call that out at the
  decision point.
- **Sentinel reacquire (v3.0 §11.3 #3, v4.0 §12.4 P3).** Deferred
  until `FCC-B1-SUMMARY` lands; that ships the periodic host-visible
  channel that sentinel reacquire piggy-backs on.
- **RTC-BKP last-known-good (v4.0 §12.2 #2).** Deferred to P3; powerful
  but requires the BKP unlock dance, RTC clock init, and a versioning
  scheme. Out of scope for one §14.2 atomic increment.
- **Vbat snapshot (v4.0 §12.2 #1).** Deferred; correct home is
  `FCC-B1-SUMMARY`, not the fault URC.
- **Free 50-channel noise-floor vector (v4.0 §12.2 #3).** Deferred;
  correct home is `FCC-B1-SUMMARY`.
- **Active rescue ping (v2.0 §10.3).** Deferred indefinitely; gated on
  a dedicated §15.5/§15.247 compliance review for an RX-role device
  that occasionally transmits. Do not bundle this with any later
  increment without that review.

### 13.3 What is explicitly rejected (not coming back)

- **Adaptive dwell, fixed rendezvous, silent retry** (already rejected
  in v1.0 §6; restated for clarity — these violate §15.1.1 or the
  observable-failure principle).
- **`sx1276_self_test()` as a standalone routine** (v4.0 §12.1). The
  derived `hw_suspect` bit from §13.1 #3 captures the same signal at
  zero new code-path cost.
- **Ignition-aware nuisance suppression on the controller** (v4.0
  §12.2 #5). The controller is the handheld remote, not the tractor;
  ignition sense is on the wrong unit. A remote-side "sleep button"
  is a separate UX feature, not a FAILED-state behavior.
- **Persisting the retry counter across power cycles** (v1.1 §7 Q4).
  Boot is always a fresh attempt; the RTC-BKP LKG (deferred to P3) is
  a *different* signal (last successful pairing), not a retry counter.
- **Widening `host_cmd_emit_fault()` payload** (rejected by v1.1, v3.0,
  v4.0, and reaffirmed here). All disambiguation lives in the 8-bit
  `sub` byte.

### 13.4 Phase P2 — operator indicator routed through the Opta (revised)

**This section supersedes the v2.0/v3.0/v4.0 framing of "GPIO SOS on
the Murata module."** That framing assumed the indicator had to be
driven directly from the L072. After verifying the live tractor-side
firmware in
[`firmware/tractor_opta/tractor_opta.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/tractor_opta.ino),
the indicator belongs on the Opta, not on the Murata module.

#### 13.4.1 Architecture

```
SX1276 RX → L072 scan SM → FAULT_URC over UART
                                  ↓
                          Portenta H7 (M7 host)
                                  ↓ Modbus RTU
                          Arduino Opta (slave)
                                  ↓
                          spare SSR (R10/R11/R12) and/or
                          onboard LED_USER / LEDR / LEDG / LEDB
```

The L072's job in P1 (§13.1) is unchanged: emit one fault URC with the
classified `sub` byte. The operator-facing actuation is entirely
host-side, in two repos that A6c-3 does not touch.

#### 13.4.2 Why this is strictly better than a Murata GPIO

1. **The hardware exists today, verified.**
   - [`tractor_opta.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/tractor_opta.ino)
     line 43: `#define REG_AUX_OUTPUTS 0x0003`.
   - Line 90: `AUX_OUTPUTS_VALID_MASK = 0x0007u` — only bits 0..2 are
     consumed (R10/R11/R12). **Bits 3..15 are free** for additive use
     (no register-map churn, no `schema_ver` bump).
   - Lines 162-165: `enter_safe_state()` already drops R10..R12 — any
     buzzer wired to R10 is automatically silenced on E-stop, which is
     the desired behavior.
   - The Opta exposes `LED_USER`, `LEDR`, `LEDG`, `LEDB` via the
     `arduino:mbed_opta` core; **none are referenced in the Opta
     sketch today**, so all four are free for status use.
2. **No Murata pinout question.** The original §13.4 v5.0 GPIO
   question required schematic-level verification of three independent
   things (module pinout, firmware-claimed pins, board routing). With
   the Opta route, all three collapse to "is R10/R11/R12 unwired on
   the customer's terminal block?" — a single BOM question, not a
   silicon question.
3. **The fault classification rides for free.** The four `sub`-byte
   classes from §13.1 #2 (`final | warm | hw_suspect | crc_seen`) map
   directly to four operator-distinguishable patterns without the
   L072 doing any blink timing in an ISR:

   | `sub` pattern | Meaning | Recommended Opta actuation |
   |---|---|---|
   | `final=1, hw_suspect=1, crc_seen=0` | Radio fault — service | LEDR solid + R10 (buzzer) double-beep on entry |
   | `final=1, warm=1, crc_seen=0` | Tractor off / out of range | LEDR slow blink (1 Hz), no buzzer |
   | `final=1, crc_seen=1` | Interference / profile mismatch | LEDR fast blink (4 Hz), no buzzer |
   | `final=1, warm=0, crc_seen=0` | Never paired this boot | LEDB heartbeat (2× brief), no buzzer |
   | `final=0, *` | Mid-retry (transient) | LED_USER brief flash; no buzzer |

4. **Compliance-neutral.** Pure 24 V SSR + onboard LED, no RF surface.
5. **Plays with existing safety chain.** Modbus 200 ms watchdog and
   `enter_safe_state()` already cover loss-of-host and E-stop; the
   indicator inherits both behaviors without new code.

#### 13.4.3 What the host-side increment must do (not part of A6c-3)

This is a Wave-4-or-later increment touching three files **outside**
[`firmware/murata_l072/`](../DESIGN-CONTROLLER/firmware/murata_l072/).
A6c-3 P1 ships independently and does not block it.

| File | Change |
|---|---|
| [`firmware/tractor_h7/tractor_h7.ino`](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino) | On `FAULT_URC` with `code == HOST_FAULT_CODE_RX_SCAN_FAILED (0x0D)`, decode the §13.1 #2 `sub` byte and write the classified status to a new Modbus register (proposal: `REG_LINK_STATUS = 0x0007`, 16-bit, layout `{class[3:0], reserved[11:4], stale[15]}`). Clear on next LOCKED frame. |
| [`firmware/tractor_opta/tractor_opta.ino`](../DESIGN-CONTROLLER/firmware/tractor_opta/tractor_opta.ino) | Add `REG_LINK_STATUS` handler. Map `class[3:0]` to LED + R10/R11/R12 pattern per the §13.4.2 table. Reuse existing `enter_safe_state()` to clear the indicator. Optionally widen `AUX_OUTPUTS_VALID_MASK` if a dedicated buzzer channel beyond R10..R12 is wanted. |
| `TRACTOR_NODE.md` (host-side doc) | Document `REG_LINK_STATUS` semantics, the chosen SSR/LED assignment, and the four-pattern scheme. Document that R10 (if used as a buzzer) inherits safe-state silence. |

Estimated host-side budget: **~30 lines in `tractor_h7.ino`**, **~40
lines in `tractor_opta.ino`**, **~50 lines of new SIL test** in the
existing Opta Modbus SIL harness (Round 10 / 2026-04-28).

#### 13.4.4 Remaining hardware preconditions (much narrower than original §13.4)

Before the host-side increment ships, verify:

1. **Is R10 (or R11/R12) actually unwired at the customer's terminal
   block?** The `kRelayPins[]` table is firmware-only; physical
   termination is a BOM/install question. If all three are claimed by
   downstream hydraulics, an indicator on a *dedicated* SSR channel
   requires a board-level addition — but the **onboard Opta LEDs
   alone (LED_USER + LEDR/G/B) are sufficient for visual indication**
   with no hardware change at all. A buzzer is the only thing that
   needs an SSR.
2. **Buzzer flyback / driver compatibility.** If a 24 V piezo is
   driven from a D1608S SSR channel, verify the SSR can handle the
   transient or add a flyback diode. (Most piezos are fine; magnetic
   buzzers need the diode.) Out of scope for firmware.
3. **Does any other future feature already plan to consume
   `REG_LINK_STATUS = 0x0007`?** Quick `TRACTOR_NODE.md` register-map
   check before allocating.

#### 13.4.5 Closes the original GPIO question

The original GPIO question ("does the Murata CMWX1ZZABZ bond out a
free pin routed to an indicator?") is **closed by reframing, not by
answering**. The Opta path makes the answer irrelevant: even if the
L072 had a free GPIO, driving it would couple blink-pattern timing
into a real-time radio firmware and require board-level routing on the
controller PCB. The Opta already has the actuators, already has the
safe-state semantics, and already has a free Modbus register slot.

### 13.5 Acceptance criteria for A6c-3

A6c-3 is complete when **all** of the following hold:

- [ ] `HOST_FAULT_CODE_RX_SCAN_FAILED = 0x0DU` added to `host_types.h`.
- [ ] Six file-statics added to `radio/sx1276_rx.c`:
      `s_scan_fail_retries`, `s_scan_got_any_irq`, `s_scan_crc_seen`,
      `s_scan_ever_locked_this_boot`, plus any helpers needed for the
      `sub`-byte assembly.
- [ ] `scan_dispatch_action(FAIL)` emits
      `host_cmd_emit_fault(HOST_FAULT_CODE_RX_SCAN_FAILED, sub)` with
      `sub` per §13.1 #2 on every FAIL, retries up to
      `SX1276_RX_SCAN_MAX_RETRIES`, then enters absorbing FAILED with
      `final=1`.
- [ ] `scan_dispatch_action(LOCK)` resets retries, CRC-seen, and
      got-any-IRQ flags, and sets `s_scan_ever_locked_this_boot`.
- [ ] `scan_dispatch_action(BEGIN_SCAN | ADVANCE_CHANNEL)` clears
      `s_scan_got_any_irq` and `s_scan_crc_seen` (one fresh window per
      channel attempt).
- [ ] `sx1276_rx_service()` sets `s_scan_got_any_irq` on any DIO0
      handler entry and `s_scan_crc_seen` on the existing CRC-error
      branch.
- [ ] `sx1276_rx_scan_policy.[ch]` is **unchanged** (pure-policy
      invariant preserved).
- [ ] New HW-free wrapper test for retry supervisor + `sub`-byte
      encoding lands under
      [`bench/host_proto/`](../DESIGN-CONTROLLER/firmware/murata_l072/bench/host_proto/)
      with ≥8 cases covering: first attempt, mid-retry, final attempt,
      LOCK reset, warm-vs-cold encoding, `hw_suspect` true/false,
      `crc_seen` true/false.
- [ ] `mingw32-make check` shows all 11 existing host tests +
      memory-map invariants + the new test green.
- [ ] TODO.md FCC-A6c-3 checkbox flipped to `[x]`.

### 13.6 Recommended §14.2 atomic-increment decomposition

To honor the one-atomic-increment-per-prompt cadence, P1 should land
across **three** §14.2 increments, not one:

| Increment | Scope | Verification |
|---|---|---|
| **A6c-3-a** | Add `HOST_FAULT_CODE_RX_SCAN_FAILED = 0x0DU` to `host_types.h`. Document the §13.1 `sub`-byte encoding in a comment block above the define. No behavior change. | `mingw32-make check` green; no new tests required (declaration-only). |
| **A6c-3-b** | Add the four file-statics, the bookkeeping in `sx1276_rx_service()` (got-any-IRQ, crc-seen), and the `LOCK`-action reset / `BEGIN_SCAN`/`ADVANCE_CHANNEL` clear logic in `scan_dispatch_action()`. **Do not yet emit the fault URC.** | `mingw32-make check` green; verify no regression in existing tests. |
| **A6c-3-c** | Wire the retry supervisor + fault URC emission into `scan_dispatch_action(FAIL)`. Land the new HW-free wrapper test (≥8 cases). Flip TODO checkbox. | `mingw32-make check` green including new test; manual review of `sub`-byte encoding in test asserts. |

Each increment is independently verifiable, independently revertible,
and individually small enough to review without context overflow.

### 13.7 v5.0 summary

P1 ships **bounded fail-closed with cheap edge classification**. The
operator-visible behavior on FAIL is unchanged at the wire layer
except for one new fault code; the *information* the host receives
per FAIL event is approximately 8× richer than the v1.1 baseline at
roughly the same code footprint. P2 is reframed in §13.4 as an
Opta-routed indicator (host-side, no Murata-GPIO question); P3 stays
deferred behind `FCC-B1-SUMMARY` and explicit wire-schema
preconditions.

The two-tier v3.0 strategy is preserved (bounded failure now,
recovery later); the v4.0 disambiguation insight is preserved
(cheap classification at the existing wire surface); the v2.0
exponential-backoff and rescue-ping ambitions are deferred behind
named gates rather than rejected outright.

*Signed:* **GitHub Copilot, RX Scan FAILED-State Review v5.0
(2026-05-19) — final consolidated A6c-3 decision; supersedes §§9-12
where contradictory.**

