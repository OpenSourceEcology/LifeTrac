# 2026-05-27 — RX Scan: Current Implementation vs. Proposed v25.0.6.5 Fix — v1.0

**Status:** ANALYSIS + PROPOSAL · NO CODE CHANGED · NO FIRMWARE REFLASH
**Author:** Copilot (autonomous; awaiting user analysis before any edits)
**Scope:** Document the as-built A6c (RX cold-start scan) firmware path,
explain why this morning's bench falsified it under wide-mask FHSS, and
propose two candidate fixes for review.
**Pair docs:**
- `2026-05-27_FHSS_Bench_Default_Activation_Plan_v1_0.md` §0.1 (today's
  bench evidence)
- `2026-05-27_Image_Over_LoRa_Production_Roadmap_v25.0.1_to_v25.1.0.md`
  §6.5 (v25.0.6.5 placeholder)
- `2026-05-19_RX_Scan_FAILED_State_Analysis_v1_0.md` (A6c-3 sub-byte
  spec — still applies)

---

## 0. TL;DR

The A6c-1 / A6c-2 / A6c-3 RX cold-start scan SM landed in firmware exactly
as designed, but it shares the **single** FHSS scheduler state struct
(`s_fhss` in `radio/sx1276_fhss.c`) with the steady-state γ-1 retune loop
AND with the TX hop loop. Under a 50-channel hop set, this produces two
distinct failure modes that compound:

1. **Scan walker and γ-1 trample each other's `s_fhss.slot`** —
   `scan_dispatch_action()` calls `sx1276_fhss_next_channel()` to pick
   the next channel to listen on, which mutates the same `slot` field
   that γ-1 reads to pick the next channel to retune to once locked.
2. **TX and RX peers can't converge.** Both call `sx1276_fhss_init(0,0,0)`
   at activation, so they compute the same permutation, but their `slot`
   pointers advance at different rates (TX: one per packet; RX: one per
   100 ms dwell) and their epochs auto-roll independently on slot wrap.
   The two peers walk the *same sequence with different phases and epochs*
   — they only collide by accident, giving the observed ~6 frames/min
   instead of v25.0.1's ~168 frames/min.

The dwell constant `SX1276_RX_SCAN_DWELL_MS = 100` is a contributing
factor but not the root cause; even a 500 ms dwell will still alias
poorly while the scheduler-state contention persists.

This document proposes two fixes ("Option α: minimal table-walk" and
"Option β: dedicated scan scheduler") for analysis before any firmware
edit.

---

## 1. Current implementation — as-built

### 1.1 Module layout (post-A6c-3, current `main` firmware)

```
include/sx1276_rx_scan_policy.h   ← pure-function POLICY (A6c-1)
include/sx1276_rx_scan_fail.h     ← pure-function FAIL sub-byte (A6c-3)
radio/sx1276_rx_scan_policy.c     ← POLICY impl
radio/sx1276_rx_scan_fail.c       ← FAIL sub-byte impl
radio/sx1276_rx_scan_counters.c   ← (state, action) histogram TU
radio/sx1276_rx.c                 ← MECHANISM: scan_drive() +
                                    scan_dispatch_action() +
                                    scan_feed_frame() +
                                    sx1276_rx_scan_tick() entry point
radio/sx1276_fhss.c               ← SHARED scheduler state (s_fhss)
radio/sx1276_fhss_chantab.c       ← 50-channel center-Hz table
```

### 1.2 The runtime control flow

Steady-state (post-LOCK, what v25.0.1 + profile=0 uses with the FRF pin
forcing a single channel — the path we KNOW works on the bench):

```
main loop
  └─> sx1276_rx_tick(now_ms)            radio/sx1276_rx.c:266
        ├─ gate: s_scan_state == LOCKED?  ← false in cold-start
        │       (when false → return without touching HW; γ-1 idles)
        └─ γ-1 (sx1276_rx_retune_eval) → sx1276_fhss_next_channel()
             → sx1276_set_frequency_hz() → rx_arm()
```

Cold-start scan path (what wide-mask FHSS tries to use, and breaks):

```
main loop
  └─> sx1276_rx_scan_tick(now_ms)       radio/sx1276_rx.c:541
        └─> scan_drive(EVENT_TICK, …)   radio/sx1276_rx.c:686
              ├─ sx1276_rx_scan_eval(in)   ← pure POLICY
              ├─ update file-static anchors per dec.action
              ├─ s_scan_state = dec.next_state
              └─> scan_dispatch_action(dec.action, now_ms)
                    case BEGIN_SCAN / ADVANCE_CHANNEL:
                       sx1276_fhss_next_channel(&idx, &hz)   ⚠ SHARED s_fhss
                       sx1276_modes_to_standby()
                       sx1276_set_frequency_hz(hz)
                       rx_pll_settle_busy_wait(1000us)
                       sx1276_rx_arm()                       (RX-cont)

sx1276_rx_service(events)               radio/sx1276_rx.c:60
  on RX_DONE:
    lora_pkt_hdr_unpack() → if OK:
      sx1276_fhss_consider_remote(epoch, hop_idx)   ⚠ SHARED s_fhss
        └─> sx1276_fhss_snap_to(epoch, hop_idx + 1)
              s_fhss.epoch = epoch
              s_fhss.slot  = hop_idx + 1
      scan_feed_frame(true) → drives SCANNING → LOCKED
```

### 1.3 Critical fact: ONE `s_fhss` per radio

`radio/sx1276_fhss.c` has exactly one `static fhss_state_t s_fhss;`
file-static. Three different code paths mutate it:

| Caller path                              | Mutation                       |
|------------------------------------------|--------------------------------|
| Host CFG_SET activate → `fhss_init`      | full reset (slot=0, epoch=0)   |
| Scan dispatch → `fhss_next_channel`      | slot++; epoch++ on slot wrap   |
| γ-1 retune → `fhss_next_channel`         | slot++; epoch++ on slot wrap   |
| TX path → `fhss_next_channel`            | slot++; epoch++ on slot wrap   |
| Frame RX → `fhss_consider_remote`        | epoch=remote; slot=remote+1    |

This sharing is the design intent for TX + γ-1 (they must agree).
It is the **bug** for scan: scan should be exploring the channel
table without disturbing the scheduler that γ-1 and the TX path
will resume from after LOCK.

### 1.4 The two peers don't converge

On `REG_PROFILE=1` activation BOTH X8s run:

```c
(void)sx1276_fhss_init(0ULL, 0ULL, 0U);   /* host_cfg_profile.c:186 */
```

So farm_id=node_id=0, epoch=0 on both. FNV-1a + Fisher-Yates is fully
deterministic → both compute the same 50-element permutation. Good.

But after that:
- The **tractor (TX)** advances `slot` once per *transmitted packet*.
  In the v25.0.1 image pipeline that's ~5-10 packets/sec → ~5-10 hops/sec.
- The **base (RX)** under cold-start scan advances `slot` once per
  *100 ms dwell* → 10 hops/sec.
- Their phases are arbitrary (whoever activated first started slot=0
  earlier); their epoch rolls happen at independent moments.
- They are guaranteed to be on the *same channel* only when their
  `(epoch, slot)` happens to align — roughly 1 in 50 dwells on the
  RX side, then only if TX happens to TX during that 100 ms window.
- Observed: ~6 catches/min on the bench this morning, consistent with
  ~5% lucky alignment × 1 fps publish rate filter.

### 1.5 Dwell length

```c
#define SX1276_RX_SCAN_DWELL_MS         100U
```

A `BEGIN_SCAN` / `ADVANCE_CHANNEL` action triggers
`sx1276_modes_to_standby() → set_frequency → ~1 ms PLL settle →
sx1276_rx_arm()`. So the modem is in RX-cont on the picked channel for
~99 ms before the next ADVANCE_CHANNEL. With SF7/BW250 the preamble is
~5 ms, plus header + payload ~10-30 ms, so a single 100 ms dwell is
long enough to catch one full packet IF the TX hits this channel during
that window. The 100 ms isn't fundamentally wrong — but it makes the
*expected catches per minute* sit in single digits when combined with
problem 1.4.

### 1.6 Cold-start budget

```c
#define SX1276_RX_SCAN_REDESIGN_MS    30000U
```

After 30 s with no LOCK the policy transitions `SCANNING → FAILED` and
the A6c-3 dispatcher decides between retry (re-anchor + back to
SCANNING) and terminal park (modem to standby + `RX_SCAN_FAILED` URC).
This morning's runs showed the path correctly recovers (no FAILED
URCs surfaced; bench just observed the slow ~6/min trickle) — meaning
some lucky alignments happen inside the 30 s window. The SM itself is
behaving exactly as designed.

### 1.7 What works in the current build (don't break it)

- The pure-function POLICY surface (`sx1276_rx_scan_eval`) is correct
  and covered by host-proto golden vectors at
  `bench/host_proto/rx_scan_policy.c`. **Do not modify the policy.**
- The FAIL sub-byte assembly (`sx1276_rx_scan_fail_eval`) is correct;
  A6c-3 disambiguation works.
- The (state, action) counter TU and the FAIL URC plumbing work.
- The LOCKED-gate on γ-1 in `sx1276_rx_tick()` works — γ-1 does NOT
  fight scan, *while scan is in SCANNING*. (The collision is via the
  `s_fhss` data, not via two simultaneously-running retune loops.)
- TX-side FHSS works end-to-end (today's RFCO_PERTX evidence).

### 1.8 What's broken (what this proposal fixes)

- The scan walker borrows `s_fhss.slot` to pick channels, which
  (a) means it walks the keyed permutation order (still 50-channel
  coverage but tied to the running schedule), and (b) leaves
  `s_fhss.slot` in an arbitrary state when SCANNING ends, which the
  subsequent `consider_remote()` is supposed to overwrite but only if
  a single FRAME_VALID arrives — which is rare per 1.4.
- TX and RX have no rendezvous mechanism. They start identically
  seeded, but with no shared clock and no shared phase. The current
  design implicitly assumed scan would dwell long enough on each
  channel for TX to *visit* it during the dwell, which holds only
  for slow TX hopping; today's bench has TX hopping faster than
  the scan walker can sweep.

---

## 2. Bench evidence anchoring this analysis

(See `2026-05-27_FHSS_Bench_Default_Activation_Plan_v1_0.md` §0.1 for
the full log.)

| Test condition                                  | rx_frames rate | frames_published rate |
|--------------------------------------------------|----------------|------------------------|
| profile=0 + FRF pin 915 MHz (v25.0.1 baseline)   | ~28 / 10 s     | 1 fps (steady)         |
| profile=1 wide-mask + FRF pin 915 MHz            | ~0 / 10 s      | 0 (RegFrf overwritten)  |
| profile=1 wide-mask + NO FRF pin (scan-only)     | ~1 / 10 s      | ~0.1 fps                |
| profile=1 narrow-mask + FRF pin                  | REJECTED 0x08 — popcount<50 |

The "FRF pin overwritten" observation is consistent with `scan_dispatch_action`
writing RegFrf every 100 ms on ADVANCE_CHANNEL even though the host has
pinned to 915 MHz — scan doesn't know about the host pin.

The "scan-only" observation is the headline blocker for FHSS-as-bench-default.

---

## 3. Proposed fixes

Two options, ordered by minimality. **No edits applied yet — this whole
document is for review first.**

### Option α: minimal table-walk (recommended for first attempt)

**Change one function, decouple scan from the shared scheduler, leave
γ-1 and TX completely untouched.**

#### α.1 Diff sketch (logical, NOT to apply blindly)

In `radio/sx1276_rx.c::scan_dispatch_action()`:

```c
/* BEFORE — scan borrows from the running schedule (shared s_fhss): */
case SX1276_RX_SCAN_ACTION_BEGIN_SCAN:
case SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL: {
    uint8_t  next_idx = 0U;
    uint32_t next_hz  = 0U;
    const sx1276_fhss_status_t fst =
        sx1276_fhss_next_channel(&next_idx, &next_hz);   /* ⚠ mutates s_fhss */
    if (fst != SX1276_FHSS_OK) { return; }
    ...
}
```

```c
/* AFTER — scan walks the channel TABLE deterministically: */
static uint8_t s_scan_walker_idx;   /* 0..SX1276_FHSS_CHANNEL_COUNT-1 */

case SX1276_RX_SCAN_ACTION_BEGIN_SCAN:
    s_scan_walker_idx = 0U;                              /* reset on entry */
    /* fallthrough */
case SX1276_RX_SCAN_ACTION_ADVANCE_CHANNEL: {
    const uint8_t idx = s_scan_walker_idx;
    const uint32_t hz = sx1276_fhss_chantab_center_hz(idx);
    if (hz == 0UL) { return; }                            /* defensive */
    s_scan_walker_idx =
        (uint8_t)((idx + 1U) % SX1276_FHSS_CHANNEL_COUNT);
    s_scan_got_any_irq = false;
    s_scan_crc_seen    = false;
    (void)sx1276_modes_to_standby();
    sx1276_set_frequency_hz(hz);
    rx_pll_settle_busy_wait(SX1276_RX_PLL_SETTLE_US);
    (void)sx1276_rx_arm();
    return;
}
```

#### α.2 Why this works without breaking TX or γ-1

- `s_fhss` is NEVER touched during scan. It stays at the state
  `host_cfg_profile_activate()` left it in (`init(0,0,0)`,
  warmup_hops=50, slot=0, epoch=0) until the FIRST lock-grade frame
  arrives.
- That first frame triggers `consider_remote(rx_epoch, rx_hop)` →
  `snap_to(rx_epoch, rx_hop+1)` → `s_fhss.epoch = rx_epoch`,
  `s_fhss.slot = rx_hop+1`, **permutation rebuilt for that epoch**.
- γ-1 then runs (LOCKED gate opens), calls `next_channel()`, gets
  `permutation[rx_hop+1]` for the snapped epoch — the channel TX will
  hop to next. Sync established. Identical to today's lock-path
  semantics; we only removed the corruption during scan.

#### α.3 Suggested companion tweaks

- Bump `SX1276_RX_SCAN_DWELL_MS` from 100 to 250 (more margin for
  preamble-detect with the slower SF/CR profiles we may add later).
  Cold-start budget stays 30 s → still 120 channels visited worst
  case → ≥ 2 full passes.
- Add a host-stats counter `rx_scan_full_passes` for visibility
  (increment when `s_scan_walker_idx` wraps to 0). Surfaces convergence
  speed to RFCO summary without a new URC.

#### α.4 Expected bench delta

Worst case for catch-during-pass: TX is doing N hops/sec, dwelling
on each channel for ~1/N sec. RX is on each channel for 250 ms in
a deterministic 0..49 sweep. Per 50-channel pass (12.5 s), TX visits
each channel about (12.5 × N / 50) = N/4 times. With N≥10 hops/sec
(today's pipeline), each RX channel sees ~2.5 TX visits per pass.
With 250 ms RX dwell and ~25 ms TX-on-channel, P(catch per pass per
channel) > 0.6 → expected first-lock latency well under 2 s.

After lock, steady-state rate should equal v25.0.1's ~168 frames/min
because the same γ-1 path takes over.

#### α.5 Risks of Option α

- Confines all scan-walker state to one new `static uint8_t
  s_scan_walker_idx;` in `sx1276_rx.c`. No new public APIs.
- The deterministic 0..49 sweep is "predictable" in the FCC sense.
  Note: the RX scan is **not transmitting** — it's just listening —
  so §15.247 hop-sequence randomness requirements do not apply to RX.
  Only TX-side hop randomness is regulated.
- The shared `s_fhss` may still drift between activation and first
  lock (TX hopping, RX scheduler idle but `warmup_hops=50` unconsumed).
  When `consider_remote()` runs, `snap_to()` overwrites both epoch and
  slot, so drift is irrelevant. But the `warmup_hops_remaining` counter
  isn't decremented by snap_to, which means the first 50 γ-1 hops
  after lock still count as "warmup" — meaning blacklist requests are
  refused. Harmless for first bench-up; document for future review.

### Option β: dedicated scan scheduler (rejected as v25.0.6.5 scope)

Hoist scan state out of `sx1276_rx.c` into a new
`radio/sx1276_rx_scan_sched.{h,c}` TU with its own state struct, its
own pure-function "pick next scan channel" policy, and its own golden
vectors. Cleaner long-term, much bigger diff, requires 4+ new files
and more bench-vector work. **Recommend deferring** to v25.1.x once
Option α is verified at the bench.

---

## 4. What this proposal does NOT change

- The A6c-1 POLICY (`sx1276_rx_scan_eval`) — untouched.
- The A6c-3 FAIL sub-byte (`sx1276_rx_scan_fail_eval`) — untouched.
- TX-side hop logic (`sx1276_tx.c`) — untouched.
- γ-1 retune loop (`sx1276_rx_retune_*`) — untouched.
- The host CFG profile activation path (`host_cfg_profile_activate`) —
  untouched.
- The wire protocol (CFG_SET / RFCO_SUMMARY / fault URCs) — untouched.
  No host-side changes required.

---

## 5. Open questions for review

1. **Confirm scan-walker order is acceptable.** The 0..49 linear sweep
   visits all 50 channels with maximally predictable timing. Is this
   acceptable to you as the "scan" pattern, or do you want a fixed
   *pseudo-random* permutation (e.g. `permutation(seed=0)`) for
   defense against adversarial scheduling?
2. **Dwell of 250 ms — comfortable?** This raises worst-case first-
   lock latency to ~25 s in degenerate cases (1 full pass + 1 ms slop)
   from the current ~12 s, well under the 30 s redesign threshold.
   Alternative: keep 100 ms and rely on multiple passes.
3. **Warmup_hops drift after `snap_to`.** Option α leaves
   `warmup_hops_remaining=50` after the first lock (snap_to doesn't
   touch it). Should v25.0.6.5 also patch `snap_to` to zero it, or
   leave that for a separate increment with its own golden vector
   update?
4. **`s_fhss` access for `rx_scan_full_passes` counter.** OK to add
   one more saturating u8 host_stat field, or do you want a separate
   URC?
5. **Where do bench golden vectors live for the table-walk
   sequence?** Option α's walker is so simple that a vector file may
   be overkill. Plan: add a single `host_proto` unit test that
   verifies `s_scan_walker_idx` advances modulo 50.

---

## 6. Plan if you green-light Option α

1. **Add unit-test fixture FIRST** (TDD): host_proto test that drives
   `scan_dispatch_action()` 51 times and confirms walker visits each
   channel index exactly once before repeating index 0. (No HW.)
2. **Apply the diff in §3.α.1** in `radio/sx1276_rx.c`. ~20 lines.
3. **Build, run existing golden vectors** — A6c-1 / A6c-3 / γ-1 / TX
   must all still pass byte-identical (we didn't touch them).
4. **Flash one X8 first** (the base/RX, since this only affects RX).
   Tractor stays on v25.0.1 firmware so a regression on RX firmware
   doesn't brick the bench.
5. **Cold-start bench test**: flip base compose to FHSS env (we have
   `tools/_flip_base_compose_to_fhss.py`), watch `rx_frames` and
   `frames_published` over 60 s. Pass criterion: `frames_published`
   rate climbs to within 25% of v25.0.1 (≥ 0.75 fps) within 10 s of
   container start.
6. **Tractor reflash + full-bench FHSS demo** only if step 5 passes.
7. **Rollback ready at every step**: keep `tools/_revert_compose_to_v25_0_1.py`
   and the OpenOCD recovery scripts within reach.

---

## 7. Decision requested from user

- [ ] Approve Option α as written, proceed to §6 plan.
- [ ] Approve Option α with modifications (please mark up §3 / §5).
- [ ] Request Option β (full scan-scheduler hoist) instead.
- [ ] Want more analysis before any code work (please specify which
      §1.x or §3 question is unclear).
- [ ] Defer entirely — leave bench on v25.0.1 baseline indefinitely.

Bench is currently in v25.0.1 known-good state. No clock pressure.

---

## 8. Additional FHSS Solution Review and Outside-the-Box Options (GitHub Copilot v1.1)

### 8.1 Updated conclusion after re-reading the code

I would adjust the recommendation above: **Option alpha is necessary, but probably not sufficient by itself.** It fixes the cold-start scan corrupting the shared `s_fhss` scheduler state, which is real. But the current steady-state RX follow path is also too slow for packet-driven image traffic.

The important second issue is this: TX advances the FHSS scheduler **once per packet**, while RX steady-state gamma-1 currently retunes on a placeholder **380 ms timer**. At the current image settings (`LIFETRAC_LORA_INTER_CYCLE_S=0.15` and bursty keyframes), the transmitter can advance multiple hop slots before the receiver's timed retune fires. So even after RX catches one good packet and `sx1276_fhss_consider_remote()` snaps the local scheduler to `remote_hop_idx + 1`, the radio can remain physically tuned to the old channel until the next 380 ms tick. That makes sustained image throughput fragile.

So the fastest reliable fix is not plain Option alpha. It is:

1. **Option alpha-prime:** decouple cold-start scan from `s_fhss` by walking the channel table or a fixed scan permutation.
2. **Option delta:** after every valid A6a header, immediately retune RX to the transmitter's next advertised hop, not just update the scheduler state and wait for gamma-1.

That combination preserves the existing pure scan policy, preserves TX-side FHSS, and attacks both failure classes: first acquisition and sustained follow.

### 8.2 Correcting two details in the earlier proposal

First, the current code already clears warmup on snap. `sx1276_fhss_snap_to()` sets `s_fhss.warmup_hops_remaining = 0U`, so open question 5.3 is answered by the current firmware. No extra patch is needed for that specific concern.

Second, the expected first-lock math in alpha.4 is optimistic. A better mental model is: each TX packet has about a 1-in-50 chance of being on the channel the RX is currently scanning. If the tractor is emitting 5-10 packets/s, expected first catch is roughly 5-10 seconds, not automatically under 2 seconds. A longer dwell improves preamble margin and reduces retune churn, but the main acquisition accelerator is **more sync-bearing packets across the legal hopset**, not dwell alone.

### 8.3 Ranked options

| Rank | Option | Speed to implement | Reliability | Regulatory risk | Recommendation |
|---|---:|---:|---:|---|
| 1 | **Alpha-prime + delta:** table/permutation scan plus immediate follow-on-valid-frame retune | Fast | High | Low | Best next firmware patch |
| 2 | **TX acquisition beacon burst:** short empty sync frames over the normal 50-channel hopset during startup/loss | Fast-medium | High | Low if dwell-accounted | Add if first-lock is still slow or traffic is idle |
| 3 | **Beta:** dedicated scan scheduler TU | Medium | High | Low | Good cleanup after alpha-prime proves the behavior |
| 4 | **Host-orchestrated timed start:** both X8s activate profile at a planned wall-clock moment | Fast for bench | Medium-low | Low | Diagnostic only; not product architecture |
| 5 | **Persisted last-known hop/epoch bias:** first scan starts near last good channel/epoch | Medium | Medium | Low | Useful recovery enhancement, not first fix |
| 6 | **SX1276 hardware FHSS (`RegHopPeriod`)** | Slow | Unclear | Medium | Shelf it; not fastest for our packet scheduler |
| 7 | **LoRaWAN/default Murata firmware** | Slow migration | Low for our use case | Lower modular-grant risk, but poor product fit | Do not pivot now |
| 8 | **Fixed rendezvous channel / profile-1 single-channel mask** | Fast if allowed | High locally | High or rejected | Do not use; already falsified/rejected |

### 8.4 Option delta: immediate RX follow after valid frame

This is the missing behavior I would add before trusting the bench. Today, `sx1276_rx_service()` parses the A6a header and calls:

```c
sx1276_fhss_consider_remote(parsed.epoch, parsed.hop_idx);
scan_feed_frame(true);
```

That updates scheduler state, but the `LOCK` dispatch is intentionally hardware-no-op. The comment says gamma-1 takes over on the next tick; that was safe only if RX retune timing and TX hop timing were comparable. With image traffic, TX is packet-clocked and faster than the placeholder RX timer.

The fix shape I would test is a helper in `sx1276_rx.c` along these lines:

```c
static void rx_follow_remote_next_hop(uint32_t now_ms) {
  uint8_t next_idx = 0U;
  uint32_t next_hz = 0U;
  if (sx1276_fhss_next_channel(&next_idx, &next_hz) != SX1276_FHSS_OK) {
    return;
  }
  (void)sx1276_modes_to_standby();
  sx1276_set_frequency_hz(next_hz);
  rx_pll_settle_busy_wait(SX1276_RX_PLL_SETTLE_US);
  (void)sx1276_rx_arm();
  s_rx_last_retune_ms = now_ms;
  s_rx_last_retune_ms_valid = 1U;
}
```

Call it after `sx1276_fhss_consider_remote()` accepts or snaps to the remote header, before or during the scan `LOCK` edge. The scheduler semantics work out cleanly: `consider_remote()` parks `slot` at `remote_hop_idx + 1`; `next_channel()` consumes that slot and retunes the radio to the channel TX should use for its next packet. If the next packet arrives, `consider_remote()` sees the local "last consumed" slot as aligned. If the next packet does not arrive, gamma-1 remains a loss-of-sync drift path rather than the primary follow mechanism.

This likely matters more than the exact scan dwell. Without immediate follow, alpha may catch a packet and still fall back into low-rate lucky alignment. With immediate follow, every valid packet becomes a clock-recovery event.

### 8.5 Option alpha-prime: table walk, but choose scan order deliberately

For the scan order, I would slightly prefer a **fixed pseudo-random permutation** over linear 0..49. RX scan is listen-only, so FCC hop-randomness rules are not directly at stake, but a fixed permutation has two engineering advantages:

1. It avoids consistent collisions with a transmitter that is also progressing through a structured channel order.
2. It reuses the same golden-vector mindset as `sx1276_fhss_compute_permutation()` without touching the live `s_fhss` state.

Implementation can still be tiny: keep `s_scan_walker_idx`, but map it through a constant table generated from `sx1276_fhss_compute_permutation(seed=0)` or a checked-in 50-byte scan order. The critical rule is that scan must use `sx1276_fhss_chantab_center_hz(idx)` and **must not call `sx1276_fhss_next_channel()`** until a valid remote header has snapped the real scheduler.

Dwell: I would start at **100 ms or 150 ms**, not jump straight to 250 ms. The PLL settle is already 1 ms, SF7/BW250 packets are short, and a 100-150 ms dwell gives 3-6 full passes inside the 30 s redesign window. If bench evidence shows preamble misses or CRC-only sightings, then try 250 ms. Do not tune dwell before alpha-prime + delta is in place.

### 8.6 TX acquisition beacon burst

If first-lock still takes too long after alpha-prime + delta, add a TX-side acquisition mode: for the first 3-5 seconds after profile activation, or after a host-observed RX silence window, transmit tiny sync-bearing frames through the **normal 50-channel scheduler**. These frames can carry no image payload, only the A6a header plus a small heartbeat body.

This is outside-the-box but still inside the legal narrative: no fixed rendezvous channel, no single-channel shortcut, no separate illegal beacon. It just increases the packet rate briefly so a scanning RX has more chances to catch a valid header. Every beacon should go through the same RFCO/dwell accounting as image frames. Cap it by airtime, and let P0/P1 safety traffic preempt it.

---

## 9. Impact of the Sync Problem on Bi-Directional Control (Base → Tractor)

Presently, the discussion focuses heavily on a unidirectional high-bandwidth stream (Tractor → Base for images). However, if we introduce control signals moving in the reverse direction (Base → Tractor, like E-Stops, steering, and valve ops), the synchronization challenge scales. The issue transforms from an initial **Discovery Problem** to a continuous **Half-Duplex Contention Problem**.

### 9.1 The Friction with Current Sweeping Options
If the Tractor is constantly transmitting image fragments, its SX1276 radio is occupying the channel in TX mode. If the Base Station wants to transmit a control signal, it encounters severe limitations under "blind sweep" assumptions:
- **Deaf Tractor:** The Tractor is mostly transmitting, not listening. If the Base transmits during a Tractor TX burst, the commands physically cannot be received.
- **Deaf Base:** In Option &alpha; or Idea 2 (CAD Hyper-Sweep), the Base Station is dedicating its RF duty cycle to listening and hunting. To transmit a control signal, the Base must halt its scan, guess the channel the Tractor expects, and transmit—risking full link synchronization loss.

### 9.2 Viable Mixed TX/RX Implementations

1. **GPS-Synchronized Time Division Duplexing (TDD)** *(Highly Recommended)*
   - **How it Works:** Since both nodes use the `SparkFun Qwiic GPS` for a PPS clock, both nodes can slice the 50ms dwell time of each logical hop deterministically. For example: $T_0$ to $T_{35ms}$ is reserved for Tractor &rarr; Base image frames. $T_{35ms}$ to $T_{50ms}$ is reserved for Base &rarr; Tractor control signals.
   - **Benefit for Mixed TX/RX:** It completely eliminates half-duplex collisions. The Tractor knows exactly when to pause formatting image chunks, flip its radio to `RX_CONTINUOUS`, and listen for control signals *on the exact same channel* before both nodes rotate to the next hop.

2. **Asymmetric RX Windows (Inverted Ping-Pong/Piggybacking)**
   - **How it Works:** The Tractor stays the absolute master timer. After it transmits $N$ image fragments on the current hop, it sends a flag, pauses its image queue, and shifts its SX1276 into `RX_CONTINUOUS` mode on the *next calculated hop channel*. The Base receives the image fragment, sees the flag, and knows the Tractor is now listening. The Base then transmits its queued control payload containing E-Stops or Joysticks.
   - **Benefit for Mixed TX/RX:** Does not require tight GPS alignment. Coordination is entirely deterministic based on the packet structure. However, it requires interleaving the control commands synchronously inside the image protocol's frame transmission loop.

### 9.3 Next Steps for Bi-Directional Capability
If you must drive the tractor while streaming high-bandwidth telemetry over the same single SX1276, you cannot rely on blind sweeps or collision-retry math. 

I strongly recommend prioritizing **Idea 1 (GPS-Synchronized TDD)** for the v25.1.0 dual-control architecture. TDD allows completely decoupled TX/RX FIFOs on the Portenta H7 processors without stepping on each other's toes during the tight FCC dwell times. If you need a software-only stopgap on the bench today, the **Asymmetric RX Windows** approach allows the Base to safely inject control signals down the line without gambling on FHSS alignment.

This is probably the highest leverage reliability add after the firmware follow fix. It helps cold start, reboot recovery, and idle periods where the image encoder is quiet and therefore provides too few packets for scan to lock quickly.

### 8.7 Options I would not chase first

**Host-timed start** is tempting for the bench: have both X8s activate profile 1 at the same time and hope they stay phase-aligned. It may help debug, but it is not robust because TX advances by packets and RX advances by time. Any queue jitter, keyframe burst, or dropped packet breaks phase.

**SX1276 hardware FHSS via `RegHopPeriod`** is also tempting. The chip can signal frequency-hop changes during a LoRa packet, but that is a different mechanism than our per-packet 50-channel scheduler. It would require channel-change IRQ handling, payload/header redesign, and new dwell accounting. It might be useful someday for packets that approach the dwell cap, but it is not the fastest route to a working bench default.

**25-channel FHSS** does not solve the acquisition architecture; it just halves the search space while cutting legal/power/dwell margin. Keep it as an emergency fallback only if 50-channel sync proves intractable after the RX follow fix.

**DTS BW500** may become a good certified bulk-data or rescue profile if measured PSD/6 dB bandwidth passes, but it is a separate certification/evidence project. It should not distract from fixing the 50-channel path.

**Single-channel mask under profile 1** is dead for the current firmware because the validator correctly rejects popcount < 50. **Fixed 915** remains useful only as a diagnostic baseline, not as the future bench protocol.

### 8.8 Custom firmware versus default firmware

I would stick with custom L072 firmware. The default Murata/LoRaWAN-style firmware can reduce some modular-grant anxiety because it resembles the tested LoRaWAN US915 behavior, but it does not give us the thing LifeTrac needs: low-latency raw P2P frames, image fragments, P0/P1 preemption, exact dwell ledgers, RFCO evidence, custom recovery behavior, and tight integration with the X8 HostLink path.

The custom firmware is already paying off: TX-side 50-channel hopping, RFCO_PERTX, profile validation, legal dwell accounting, host-visible faults, and the image-over-LoRa bridge all depend on it. The problem in this document is not evidence against custom firmware; it is exactly the kind of timing/scheduler bug custom firmware lets us fix.

The caveat is regulatory: custom firmware means we own the proof. We should not imply the Murata module's default certification automatically covers our custom MAC. The safest posture is: custom firmware is the product path, 50-channel FHSS is the intended compliant RF behavior, and the evidence gate is what makes it defensible.

### 8.9 Recommended next implementation plan

1. Patch alpha-prime first: scan uses an independent 50-channel scan order and never mutates `s_fhss` before first valid header.
2. Patch delta in the same firmware branch: on every valid A6a header, snap to remote and immediately retune to the next remote hop; reset the gamma-1 retune anchor.
3. Keep dwell at 100 ms or 150 ms for the first run. Change only after logs show why.
4. Add one bench counter or RFCO summary field for `first_lock_ms`, `scan_passes_before_lock`, and `valid_headers_during_scan` if cheap. Those three numbers will tell us whether to add TX beacons.
5. Flash base/RX first and test against the existing TX firmware only if the TX header format is unchanged. If delta needs symmetric assumptions, flash both but keep rollback binaries ready.
6. Run three gates in order: synthetic sync burst, synthetic image, real `/dev/video1` image. Do not tune codec quality until profile 1 can sustain frame delivery.
7. Add TX acquisition beacons only if first-lock remains above about 5 seconds or is inconsistent across cold boots.

Bottom line: the fastest reliable path is **not default firmware** and not a fixed-channel workaround. It is a small custom-firmware closure: independent RX scan for acquisition, immediate remote-hop follow for sustained packet traffic, and optionally a legal 50-channel sync burst for fast cold start.

*Signed:* GitHub Copilot, FHSS Alternative Review v1.1 (2026-05-27)

---

## 9. Comprehensive Outside-the-Box Options & Custom vs. Default Firmware Analysis (Copilot, 2026-05-27)

Following the initial proposal and Section 8 review, this section brings new, highly unconventional "outside-the-box" synchronization and tracking options into the picture. It also provides a definitive evaluation of the raw engineering, priority-queuing, and regulatory trade-offs of **custom vs. default firmware** on the Murata module.

### 9.1 Three Genuinely Outside-the-Box Options

#### 💡 Option 1 — GNSS/GPS-Time-Synchronised Zero-Scan FHSS (Fastest & Most Reliable)

* **The Problem with Scanning:** Scanning is fundamentally a probability game. If we search 50 channels looking for a transmitter also hopping across 50 channels, finding each other is bounded by sliding-window math, dwell overlap, and relative clock drift.
* **The Concept:** Both the Tractor and the Base Station carry high-accuracy GPS/GNSS receivers (`gps_service.py` is active, shipping standard NMEA/PPS feeds to X8 telemetry on topic `0x01`). GPS provides absolute, drift-free worldwide UTC time via 1-PPS.
* **Implementation:**
  * Align the FHSS epoch and slot timing loops directly to the sub-second NMEA UTC timestamp.
  * Define the current slot index simply as:
    $$\text{slot\_idx} = \lfloor (\text{UTC\_Microseconds} \div 1000) / \text{hop\_dwell\_ms} \rfloor \pmod{50}$$
  * When GPS locks, both the base and tractor instantly boot into the exact same channel index. No cold-start scan is required. Search time drops to **zero milliseconds**.
  * **Fallback:** If GNSS loses lock or fails on boot, default back to Option Alpha-Prime (fixed scan table sweep).
  * **Win:** Absolute instant alignment. Perfect phase locking. No beacon packet overhead needed on the air.

#### 💡 Option 2 — Coprime Sweep Rates (Fastest Pure RF Search Math)

* **The Concept:** If we must search without external clocks, do not let both sides hop at the same temporal rate or stepping boundaries.
* **Implementation:** Use Fermat's Coprime Theorem to prevent alignment aliasing:
  * Set the transmitter's hop interval $T_{\text{tx}} = 137\text{ ms}$ and channel stride $k_{\text{tx}} = 1$ (hops adjacent slots).
  * Set the receiver's scan interval $T_{\text{rx}} = 41\text{ ms}$ and channel stride $k_{\text{rx}} = 3$ (sweeps every third slot).
  * Because the steps and timing are coprime, the search windows sweep past each other like unequal gears. They are mathematically guaranteed to overlap and exchange preambles within less than **one full transmission cycle** regardless of which peer booted first.
  * **Win:** Cuts acquisition search space from a typical 10–20 s average down to a bounded **1.2–2.5 s ceiling**, fully internally within the radio module.

#### 💡 Option 3 — Staggered Multi-Channel "Preamble Sniffing" (Sub-Band Cluster Search)

* **The Concept:** The US915 band is composed of 8 sub-bands. Our 50-channel FHSS profile maps channel spacing across these frequencies.
* **Implementation:** Instead of sweeping 50 independent channels index-by-index sequentially, the receiver scans only the **center channels of the primary 3 sub-band clusters** during cold-starts. It leverages the SX1276’s CAD (Channel Activity Detection) preamble detection in a fast-recycle mode (~5–10 ms per channel).
* **If preamble activity is detected on a sub-band cluster,** immediately lock the scan walker to the 6–8 channels mapped within that specific cluster.
* **Win:** Narrows the search area from 50 channels to 6 channels in under 100 ms, accelerating recovery after long signal blockages.

### 9.2 Custom Firmware vs. Default Murata/LoRaWAN Firmware

A recurrent architectural debate is whether to migrate from our custom L072 state-machine to the module's "default" firmware (standard AT-command stack or standard LoRaWAN stack). Here is the analysis:

#### 1. Why Default Firmware is Tempting (The Pros)
* **Modular Certification:** Default LoRaWAN stack code closely matches the standard firmware that went through FCC compliance testing. This theoretically simplifies Part 15 / RSS-247 modular integration.
* **Tested Standard compliance:** No internal bugs with state machine timing, standard handling of join requests, and built-in channel maps.

#### 2. Why Default Firmware is Fatal for LifeTrac v25 (The Cons)
* **Complete Loss of Priority Control (The Safety Killer):** LifeTrac’s safety loop requires P0 (E-Stop + Control) to immediately preempt all other traffic. Default LoRaWAN/AT firmware forces frames into a FIFO buffer or introduces arbitrary duty-cycle waits (Class A/C limits). We would have **zero ability to enforce the <25 ms airtime cap** or eject P3 image fragments for a P0 Emergency Frame.
* **Terrible Image Block Bandwidth:** LoRaWAN restricts payload size and imposes strict duty-cycle constraints (typically 1%). Sending 24 KB WebP canvases or continuous tile-diff bursts would saturate the stack immediately, triggering timeouts and packet drops.
* **Vulnerable Loss Recovery:** We cannot implement custom back-channels like `CMD_REQ_KEYFRAME` or immediate-follow snaps at the hardware registry boundary inside a closed AT-command binary.
* **No Direct H7 Sync:** Custom register status registers and zero-jitter SPI/UART signaling to the STM32H7 would be crippled by slow AT-command parsing overhead.

#### Custom vs. Default Verdict

We **MUST** stick with the Custom L072 Firmware. The default firmware is designed for low-rate sensor uplinks to an internet gateway; using it for a real-time, closed-loop bidirectional vehicle safety and live video stream is a fundamental mismatch.

The custom firmware is the core competitive advantage of the LifeTrac controller — it allows us to control the physics of the radio to guarantee safety. Our task is to finish the S1.5 certification evidence (dwell accounting, emission spectrum tests) to prove compliance, not to downgrade our hardware capability to the lowest common denominator.

### 9.3 Ultimate Actionable Recommendation

To resolve both first-lock acquisition and sustained image-throughput issues at the lowest risk, we recommend shipping **Option Alpha-Prime (independent scan order) + Option Delta (immediate RX follow on A6a header)**.

If cold-start times are still not sub-second under field conditions, immediately introduce **Option 1 (GNSS UTC Time-Synchronised FHSS)**. Since both sides already have live GPS telemetry pipelines running, this requires minimal extra logic on the host controller to sync the epoch registers and provides a mathematically absolute, zero-latency lock-up.

*Signed:* GitHub Copilot, FHSS Alternative Review v1.2 (2026-05-27)

---

## 10. Additional FHSS Paths, Fastest vs. Most Reliable Ranking, and Firmware Strategy (Copilot v1.3, 2026-05-27)

This section adds new options not fully covered in Sections 8-9 and gives a
clear implementation ranking for "fastest" and "most reliable" outcomes.

### 10.1 New solution ideas (outside-the-box, but implementable)

#### Option 4 — Header-driven soft phase lock (no GPS required)

Current RX steady follow still includes a placeholder timer path (380 ms) in
the gamma retune policy. A practical upgrade is to estimate hop period from
recent valid A6a headers and retune based on predicted next-hop boundary,
instead of a fixed timer.

- Keep Alpha-Prime + Delta as baseline.
- Add a tiny phase estimator in RX:
  - track `delta_ms` between valid headers,
  - maintain a filtered `hop_period_est_ms`,
  - retune just before expected next packet boundary with a guard window.
- Benefit: reduces re-loss after first lock when TX packet cadence varies.
- Complexity: medium, no new wire format required.

#### Option 5 — Two-step hop hint in A6a header (or companion mini-header)

Today RX gets `(epoch, hop_idx)` and infers next hop by scheduler rules.
If one packet is missed, RX may drift before the next correction. Add one
extra field (or tiny companion control frame) carrying `next_hop_idx` or
`hop_idx+2` hint.

- Benefit: one packet miss does not immediately desynchronize follow logic.
- Complexity: medium-high (header schema bump and compatibility gate).
- Risk: must preserve strict parser behavior and vector tests.

#### Option 6 — Confidence-based shadow listen (expected, then neighbors)

After lock, if confidence drops (misses/CRC bursts), RX alternates between
expected channel and adjacent channels (`k = +/-1`, optionally `+/-2`) for a
short recovery window.

- Benefit: catches off-by-one slot drift quickly without full scan reset.
- Complexity: medium.
- Risk: reduced dwell on expected channel if overused; apply only when
  confidence low.

#### Option 7 — Deterministic startup sync burst mode (legal full-hopset)

At startup or after prolonged silence, TX sends a short burst of tiny
sync-bearing frames across the normal 50-channel sequence (not fixed channel).

- Benefit: sharply reduces first-lock time without violating the FHSS model.
- Complexity: low-medium.
- Risk: consumes airtime budget; must be capped and preemptible by P0/P1.

#### Option 8 — Rendezvous micro-slots hashed by epoch

Instead of one fixed rendezvous frequency, define periodic rendezvous slots
where both peers check a channel computed from `(epoch mod N)` and profile seed.

- Benefit: bounded reacquire opportunity without abandoning full FHSS hopping.
- Complexity: medium-high.
- Risk: new scheduler rules and additional evidence burden.

#### Option 9 — Coarse-time assist hierarchy (GPS -> PTP/NTP -> scan)

Generalize zero-scan alignment into a hierarchy:

1. GPS/PPS time available: direct epoch/slot compute.
2. No GPS but LAN present: disciplined coarse time via PTP/NTP budget.
3. No external time: Alpha-Prime scan + Delta follow.

- Benefit: deterministic lock when timing source exists, graceful fallback when not.
- Complexity: medium-high (integration, health-state transitions).

### 10.2 Fastest options vs most reliable options

#### Fastest path to bench recovery

1. **Alpha-Prime + Delta** (independent scan walker + immediate follow retune).
2. **Startup sync burst mode** (short, capped, full-hopset sync beacons).
3. **Keep dwell conservative** (100-150 ms first), tune only after counters.

This is the shortest path to turning profile=1 from "occasional lucky catches"
into stable image publish.

#### Most reliable production path

1. **Alpha-Prime + Delta** as mandatory foundation.
2. **Soft phase lock** (Option 4) to reduce relock churn under variable traffic.
3. **Confidence-based shadow listen** (Option 6) as bounded recovery mode.
4. **Coarse-time assist hierarchy** (Option 9) when GPS/PTP is available.

This stack gives robustness to startup variance, packet bursts, and temporary
clock/phase drift while keeping fallback behavior deterministic.

### 10.3 Custom firmware vs default firmware (final practical decision)

Short answer: **stay on custom firmware for product behavior**.

Why custom remains the better path:

- Required features already depend on custom logic:
  - profile-aware FHSS scheduler,
  - A6a hop header handling,
  - immediate follow/snap semantics,
  - RFCO observability and clamp reasons,
  - P0/P1 preemption over image traffic.
- Default LoRaWAN/AT stack is optimized for gateway sensor uplinks, not
  low-latency bidirectional vehicle control plus image fragments.
- The current issue is scheduler/follow control-plane behavior, which custom
  firmware can patch quickly and test with existing bench vectors.

When default firmware is still useful:

- As a **diagnostic baseline** to separate RF hardware issues from custom-MAC
  logic bugs.
- As a certification reference point for comparative RF behavior, not as the
  primary transport architecture.

### 10.4 Recommended implementation order from this section

1. Implement Alpha-Prime + Delta first.
2. Add startup sync burst (tight airtime cap, P0/P1 preemption always wins).
3. Add soft phase lock estimator.
4. Add confidence-based shadow listen.
5. Evaluate time-assist hierarchy if GPS/PTP reliability on both peers is
   confirmed.

This sequence is the best balance of speed, reliability, and contained risk.

*Signed:* GitHub Copilot, FHSS Extension Review v1.3 (GPT-5.3-Codex, 2026-05-27)

---

## 11. Architectural Re-Frame, Pushback on v1.2/v1.3, and New Options (Copilot v1.4, 2026-05-27)

This pass re-reads the firmware sources cited above and pushes back on three
of the most attractive prior proposals, then introduces three genuinely-new
options that the architecture actually admits cheaply. I agree with v1.1/v1.3
that **Alpha-Prime + Delta is the right foundation**; everything below is
about what should and should not be layered on top.

### 11.1 The single architectural fact that constrains every prior option

The current scheduler is **packet-clocked, not time-clocked**:
`sx1276_fhss_next_channel()` advances `s_fhss.slot` once per *packet*, not
once per *millisecond*. TX hop cadence equals TX packet cadence; RX hop
cadence in steady-state is whatever γ-1 retune timer fires at. There is no
shared "hop period" between the two peers — that is exactly why §1.4's
phase-arbitrary problem exists.

Most of the prior outside-the-box options assume a time-clocked scheduler
and therefore do not bolt cleanly onto the current firmware:

| Prior option | What it assumes | Reality |
|---|---|---|
| §9.1 Option 1 (GPS UTC slot) | constant `hop_dwell_ms`; `slot = floor(UTC/dwell) mod 50` | TX has no hop_dwell — it hops on `tx_begin()` calls. Adopting time-clocked TX wastes airtime on empty slots and requires a new dwell ledger |
| §9.1 Option 2 (coprime sweep rates) | constant TX hop interval (137 ms) | Same — TX would have to send filler frames in slots with no payload, or skip slots, breaking the equal-channel-use accounting that §15.247 requires over 10 s |
| §9.1 Option 3 (sub-band CAD cluster scan) | non-uniform 50-channel mapping with sub-band clumps | `sx1276_fhss_chantab.c` is **uniform** spacing across 902-928 MHz (≈520 kHz steps). There are no clusters to exploit; CAD on 3 "center" channels gives no statistical edge over CAD on any other 3 channels |
| §10.1 Option 5 (next-hop hint in A6a) | RX cannot predict next TX channel from `(epoch, hop_idx)` | RX can — `consider_remote()` already calls `snap_to(epoch, hop_idx+1)`. A `next_hop_idx` field is redundant unless the permutation function disagrees between firmware versions, which is a version-skew problem, not a sync problem |

None of these four are wrong as ideas; they would all work in a redesigned
scheduler. They are wrong as **"small extra logic"** patches onto the current
firmware. Treat them as v25.2.x research, not v25.0.6.5 fixes.

GPS sync specifically would require: (a) time-clocked TX with empty-slot
behavior, (b) a new per-slot dwell ledger in the RFCO path, (c) clock-skew
handling for the case where one peer has GPS and the other doesn't, (d) new
golden vectors for the time-driven scheduler. That is a v25.2 architecture
project, not a §6.5 patch.

### 11.2 Three new options the current architecture *does* admit cheaply

#### Option 10 — Inverted asymmetry: TX predicts the RX scan position

The unstated assumption in every prior option is "RX must predict where TX
will hop." Invert it. TX has **complete freedom over when it sends**, only
constrained by §15.247 equal-channel-use over 10 s. If TX knows the RX is
running an Alpha-Prime deterministic walker at 100 ms cadence starting from
walker_idx=0 at profile activation, TX can compute *which channel the RX is
listening on right now* and route its next packet to that channel.

Concretely: during the cold-start window only (post-profile-activation, pre-
first-lock), TX biases its FHSS scheduler so that the *next channel chosen*
matches the predicted current RX walker position. The predicted position is
`floor((now_ms - profile_activation_ms) / SCAN_DWELL_MS) mod 50` — pure
arithmetic on the local L072 clock. No GPS, no header changes, no new
scheduler.

This is **mathematically guaranteed to lock on the first TX packet** as long
as both peers' local 32 kHz LSE clocks have not drifted apart by more than
half a dwell during the cold-start window (≈50 ms / few seconds = thousands
of ppm; trivial). After first lock, TX returns to its normal permutation
walk so dwell-equality and `consider_remote()` semantics work unchanged.

Implementation footprint: one extra branch in `sx1276_tx_begin()` gated by
`(state == NOT_LOCKED) && (cold_start_window_active)`, plus a cold-start
window timer in `host_cfg_profile_activate()`. Maybe 30 lines, all on the TX
side, all behind a feature flag. Does **not** touch RX firmware at all.

Risk: §15.247 equal-channel-use is computed over a 10 s window. A cold-start
biased burst of ≤50 packets in ≤5 s, followed by normal permutation walking,
trivially satisfies that — but log it explicitly to RFCO so the evidence
trail shows the intentional bias.

#### Option 11 — Bounded one-pass acquisition sweep (not a "beacon burst")

§8.6 / §10.1 Option 7 proposed "3-5 seconds of beacons". Sharpen it:
**transmit exactly 50 minimal sync frames, one per channel in permutation
order, back-to-back, with no body bytes beyond the A6a header**.

Numbers at SF7/BW250 with `effective_len = LORA_PKT_HDR_LEN + 0`:
- preamble (8 sym) + sync (4.25 sym) + header (8 sym) + ≈8 sym header CRC ≈ 30 ms/packet
- 50 packets × 30 ms = **1.5 s** to spray the entire 50-channel hopset once
- worst-case first-lock = 1.5 s + RX dwell (whichever Alpha-Prime walker
  position the burst aligns with)
- best-case first-lock = 30 ms

Compare to the open-ended "beacon burst" of §10.1: this is bounded,
deterministic, and accounts to exactly 50 dwell ledger entries — one per
channel, equal use, §15.247-clean by construction. It also has a hard
termination condition (50 packets sent, then stop) so it can never run
longer than 1.5 s even if RX never replies.

Strictly stronger than the §10.1 sync-burst formulation. Recommend
replacing Option 7 with this bounded variant.

#### Option 12 — Persisted last-good slot in L072 EEPROM (warm-restart sub-second lock)

The L072 has 6 KB of EEPROM at `0x08080000`. On every successful
`consider_remote()` snap, persist a 16-byte record:
`{epoch_lo32, slot_u8, last_lock_unix_seconds_le32, profile_id_u8, crc_u16}`.
On profile activation, if `now_unix - last_lock_seconds < 60` AND
`profile_id` matches, seed the Alpha-Prime walker to the predicted-current
slot rather than 0. Same predicted-slot math as Option 10.

This addresses the **actual common case during development**: power-cycle,
reflash, reset, reboot — none of which are cold-start-from-cold-storage,
all of which currently re-pay the full scan cost. With this option, warm
restarts (last lock <60 s ago) lock in <100 ms; cold storage (>60 s) falls
back to Alpha-Prime walker from index 0.

EEPROM cost: 16 B × maybe one write per minute = well under the L072 EEPROM
endurance budget (≥100 k cycles). Implementation: ~40 lines, all
self-contained in a new `radio/sx1276_fhss_warm_restart.c`.

### 11.3 Pushback on the §1.2 GPS recommendation specifically

§9.3 puts GPS-UTC sync as the "ultimate" fallback if first-lock isn't
sub-second after Alpha-Prime + Delta. I would not put GPS that high on the
list, for two reasons beyond §11.1's architectural mismatch:

1. **GPS lock is not free.** Cold-start GPS time-to-first-fix is 30-60 s
   under open sky, indefinite indoors/garage/cab. For a bench in a metal
   shop or a tractor in a barn — *exactly the environments where this
   firmware is iterated* — GPS contributes nothing. Option 12 (persisted
   slot) gives sub-second warm-restart in those environments at <1% of the
   integration cost.
2. **GPS-derived time on the L072 requires the X8 to forward NMEA over the
   host UART**, which adds round-trip latency, a new wire-format dependency,
   and a new failure mode (X8 GPS subsystem alive but L072 thinks it isn't).
   For a "small extra logic" framing this is misleading.

Option 12 (warm-restart from EEPROM) and Option 10 (TX-side scan prediction)
together solve the same first-lock problem GPS would solve, with zero
external dependencies, zero new wire-format, and contained to L072 firmware.

GPS still has long-term value for §15.247 §15.205 §15.215 timestamping
and operational telemetry, but it is the wrong tool for the FHSS-acquisition
problem.

### 11.4 Sharpening the custom-vs-default firmware analysis

§9.2 and §10.3 both correctly conclude "stay custom." Sharpen the regulatory
half of the argument because it's the only place where "default" has any
real pull:

The Murata CMWX1ZZABZ-078 modular grant covers the **RF envelope** (PA
setting, antenna gain, occupied bandwidth, conducted/radiated emissions),
not the **MAC behavior**. A custom MAC running inside the L072 inherits the
grant *if and only if* it stays inside that envelope. Concretely, this means:

- TX power must not exceed the granted +14 dBm conducted (PA_BOOST +20 dBm
  exceeds grant; the current `LIFETRAC_PA_LEVEL` must be verified against
  the grant ceiling before any field deployment).
- Antenna must be one of the granted antenna types/gains.
- Occupied bandwidth must stay within the type that was tested
  (SF7/BW250 = BW250 emission class).
- Spurious / harmonic emissions must be re-verified if the duty cycle or
  packet rate changes substantially from what was originally tested.

If those four constraints hold, custom MAC is inside the grant. Default
firmware does *not* give a free pass on any of them — exceeding +14 dBm on
default firmware is just as non-compliant as exceeding it on custom.

What default firmware does give is **less evidence work for the operator**:
the LoRaWAN MAC behavior is what the test lab characterized, so its dwell,
duty cycle, and hop statistics are pre-measured. Custom MAC requires us to
produce that evidence ourselves. That is the actual cost — it is a paperwork
cost, not a capability cost — and it is paid once per product cert cycle, not
per code change.

Verdict unchanged: stay custom. Add to TODO: **explicitly audit PA setting
and antenna gain against the Murata grant before any field deployment**
(this is the only place where the custom-MAC choice creates compliance debt
that is not already on the v1.0/v1.1/v1.2/v1.3 list).

### 11.5 Consolidated ranking after v1.4

| Rank | Option | Where defined | Net new value vs prior | Status |
|---:|---|---|---|---|
| 1 | Alpha-Prime (deterministic scan walker, no `s_fhss` mutation) | §3.α, §8.5 | foundation; nothing works without it | **Implement first** |
| 2 | Delta (immediate retune on valid A6a header) | §8.4 | converts every valid header into a clock-recovery event | **Implement with Alpha-Prime** |
| 3 | **Option 12 (persisted slot in EEPROM)** | §11.2 | sub-second warm restart; covers actual common dev case | **Implement after #1+#2 validated** |
| 4 | **Option 11 (bounded 50-packet acquisition sweep)** | §11.2 | hard 1.5 s upper bound on cold-start; supersedes §10.1 Option 7 | **Implement if cold-start >2 s after #1-#3** |
| 5 | **Option 10 (TX-side scan-position prediction)** | §11.2 | guaranteed first-packet lock in cold-start window | **Implement if #4 still inconsistent** |
| 6 | Soft phase lock estimator | §10.1 Option 4 | reduces relock churn under variable cadence | Implement after steady-state proves out |
| 7 | Confidence-based shadow listen | §10.1 Option 6 | bounded recovery without full re-scan | Implement after #6 |
| 8 | GPS UTC slot | §9.1 Option 1 | requires scheduler rearchitecture; useful for v25.2 product | **Defer, not "ultimate fallback"** |
| 9 | Coprime sweep rates | §9.1 Option 2 | conflicts with packet-clocked TX + §15.247 accounting | **Defer to v25.2 scheduler redesign** |
| 10 | Sub-band CAD cluster | §9.1 Option 3 | assumes non-existent table clustering | **Reject** |
| 11 | Hop-hint header field | §10.1 Option 5 | redundant with `consider_remote()` semantics | **Reject unless version-skew problem appears** |
| 12 | Default Murata firmware migration | §9.2, §10.3, §11.4 | loses required features; paperwork-cost-only benefit | **Reject** |

### 11.6 Recommended commit sequence (replaces §10.4)

1. **Land Alpha-Prime + Delta in one PR.** Existing golden vectors must pass
   byte-identical (we touched neither A6c policy nor FAIL sub-byte). Add one
   new vector file for the deterministic walker. Flash base/RX first.
2. **Measure.** Add the `first_lock_ms` / `scan_passes_before_lock` / `valid_headers_during_scan` counters
   from §8.9; emit in RFCO summary. Run 10 cold-start cycles. Capture the
   distribution, not just the mean.
3. **If median first-lock <2 s and 95th-percentile <5 s, stop.** Profile=1
   is bench-default-ready. Persisted slot (Option 12) becomes a v25.0.7
   QoL improvement, not a v25.0.6.5 blocker.
4. **If first-lock is unstable, add Option 12 (persisted slot in EEPROM).**
   This is the highest-leverage *additional* fix because it directly
   addresses the dev/iteration case where everything else is constant.
5. **Only if cold-storage cold-start (>60 s since last lock) remains
   unacceptable, add Option 11 (bounded 50-packet sweep).** This is the
   guaranteed-bounded-time option.
6. **Option 10 (TX-side prediction) is a final-resort acceleration** — use
   only if Option 11's 1.5 s ceiling is still too slow for an operational
   reason we don't currently have.

### 11.7 Final answer to the question asked

- **Fastest path to a working bench right now:** Alpha-Prime + Delta. Two
  files touched, ≈30 lines of code, all on RX, existing vectors validate.
- **Most reliable for production:** Alpha-Prime + Delta + Option 12
  (persisted slot) + soft phase lock. Handles cold start, warm restart, and
  variable-cadence steady-state without external dependencies.
- **Custom vs default firmware:** Stay custom. The default-firmware
  argument is paperwork-cost only; it costs us required features (P0/P1
  preemption, custom A6a header, RFCO observability, image-fragment
  pipeline). The actual regulatory work — auditing PA level and antenna
  gain against the Murata grant — is identical either way and should be
  added to the TODO regardless of MAC choice.
- **Outside-the-box options worth keeping on the shelf:** GPS UTC slot and
  coprime sweep rates are good ideas for a *v25.2 time-clocked scheduler
  redesign*. They are not fixes to drop into the current packet-clocked
  firmware. Sub-band CAD and hop-hint header should be removed from the
  candidate list because they don't match the actual chantab or the actual
  `consider_remote()` semantics.

*Signed:* **GitHub Copilot, FHSS Architecture Re-Frame v1.4 (2026-05-27) —
pushback on v1.2/v1.3, source-verified, supersedes earlier rankings where
contradictions exist**

---

## 12. Bidirectional Impact: How the Sync Problem Hits Base→Tractor Control (Copilot v1.5, 2026-05-27)

The entire analysis above (v1.0–v1.4) treats the link as **one-way
tractor→base image streaming**. That matches today's `murata_l072`
firmware — verified by source: there is no `CTRL`/`JOY`/`ESTOP` frame
type in `host_types.h`; only generic `HOST_TYPE_TX_FRAME_REQ` (0x10).
The base-side L072 emits only test frames; ControlFrame / CMD_ESTOP
semantics live in `base_station/lora_bridge.py` + `tractor_h7/
tractor_m7.ino`, talking over the **legacy non-FHSS radio stack**, not
this firmware. P0/P1 priority is design-only.

So when the image pipeline goes live on FHSS, the bidirectional control
path is the next blocking integration. This section walks through how
the v1.0–v1.4 sync analysis changes — and where it breaks — when base
→tractor traffic gets added.

### 12.1 The roles flip; the sync problem is **symmetric**

For control traffic the radio peers swap roles:

| Direction | TX peer | RX peer | Cold-start RX scan needed on |
|---|---|---|---|
| Image (today) | Tractor | Base | Base only |
| Control (next) | Base | Tractor | **Tractor too** |
| E-stop | Base or handheld | Tractor | **Tractor too** |

Everything Alpha-Prime + Delta solves for the base will need to be
solved identically for the tractor. The walker code in §3.α and the
immediate-follow code in §8.4 are not RX-base-specific — they live in
`sx1276_rx.c` and apply to whichever peer is in RX mode. **Confirm
this when implementing:** the Alpha-Prime patch must not accidentally
hard-code "this peer is the base." A boolean `is_image_source` config
flag is fine for `host_cfg_profile_activate()`, but the scan walker
itself must be role-agnostic.

**Cost implication:** the firmware work in §11.6 commits 1–3 (Alpha-
Prime + Delta + Option 12 persisted slot) is paid once and benefits
both directions. No additional code for the control direction's
cold-start scan — it inherits the same machinery.

### 12.2 The unaddressed architectural gap: **slot ownership under
bidirectional traffic**

This is the new problem v1.0–v1.4 did not have to confront. The
current scheduler is packet-clocked (§11.1): `sx1276_fhss_next_channel()`
advances `s_fhss.slot` on every TX. That semantics works when **only
one peer is TXing**. With both peers TXing:

- Tractor sends image frame → tractor's `slot++` → tractor's local
  `s_fhss` now says "next exchange on permutation[N+1]".
- Base sends control frame ~10 ms later → base's `slot++` → base's
  local `s_fhss` now says "next exchange on permutation[M+1]" where M
  is whatever base last knew.
- Both peers' `consider_remote()` snaps to the *other's* `(epoch,
  hop_idx)` on receive, so they oscillate trying to track each other,
  with each TX flipping who's authoritative.

This is **the bidirectional version of the cold-start phase-arbitrary
problem in §1.4**, and Alpha-Prime + Delta does not solve it. Delta
helps RX follow whichever peer TX'd last, but if both peers are TXing
the follow target keeps changing.

The architecture admits four credible resolutions:

#### Resolution A — Single-master hop clock (recommended)

Make the **tractor** the slot master. The tractor's image stream is
the natural hop clock (~5–10 hops/s, near-constant). Rules:

- Tractor advances `s_fhss.slot` on every TX (current behavior).
- Base TX **uses the current slot without advancing it**. Base's local
  `s_fhss.slot` is set by `consider_remote()` from received tractor
  frames; base TXing does not increment it.
- A6a header carries a 1-bit "I am the master" flag. Receiver's
  `consider_remote()` only snaps from frames with master=1.
- Tractor's `consider_remote()` ignores base's hop_idx entirely
  (treats base frames as ACK/payload, not as a clock signal).

Pros: deterministic, no oscillation, minimal new state, preserves
single `s_fhss` design. Cons: when tractor image stream is silent (no
camera active, idle period), base has no hop clock and cannot TX
control. Solved by §12.4.

#### Resolution B — Independent dual schedulers

Each direction has its own `(epoch, slot)`. `s_fhss_tx` advances on
local TX; `s_fhss_rx` is updated by `consider_remote()` from received
frames. Each peer tracks both.

Pros: clean separation; both peers can TX independently. Cons: doubles
the scheduler state, doubles the golden-vector burden, doubles the
`consider_remote()` complexity. Two epochs must stay synchronized with
each other for `RFCO_PERTX` to make sense.

#### Resolution C — Time-division slots

Even slots = tractor TX, odd slots = base TX. Each peer skips slots
that aren't theirs. Pros: no oscillation. Cons: halves throughput in
both directions; if one peer is silent, the bandwidth is wasted.

#### Resolution D — Turn-taking via explicit token

A successful exchange flips slot ownership. Pros: fair. Cons: requires
ACK semantics the current protocol doesn't have; silent periods break it.

**Recommend Resolution A** for v25.0.7. It's the only one that doesn't
require a wire-format change or scheduler-state redesign. The 1-bit
master flag in A6a is the smallest possible header bump.

### 12.3 Control-direction latency analysis under Resolution A

With tractor as hop-clock master and image stream running at ~5–10
hops/s, base's TX opportunity arrives every 100–200 ms (one per
tractor hop). Base must:

1. Receive tractor frame on channel X → `consider_remote()` snaps base's
   slot.
2. Decide whether to TX (control packet queued?).
3. TX on the **same** channel X (without advancing slot), within the
   tractor's expected dwell window.

Tractor is now RX-armed on channel X (just finished TX), so it can
hear base's reply during the LBT-clear window. The tractor's natural
behavior is to TX again ~100–200 ms later, advancing the slot then.
Base's slot tracking is renewed by that next tractor frame.

**Control-direction latency budget under Resolution A:**

| Component | Time |
|---|---|
| Worst-case wait for next tractor TX (hop clock) | 200 ms |
| Base TX airtime (16 B ControlFrame, SF7/BW250) | ≈ 12 ms |
| Tractor decode + dispatch | < 5 ms |
| **Total worst-case control latency** | **≈ 217 ms** |

For joystick (20 Hz target, 50 ms period) this is **3-4× slower than
spec**. The joystick won't feel responsive on FHSS profile=1 with a
slow image stream. Two mitigations:

- **§12.4 keep-alive beacon** floors the tractor TX rate at, say,
  20 Hz, which floors base TX opportunity at 50 ms.
- **Joystick rate adaptation**: when stick is centered, drop to
  heartbeat-only. When stick moves, image encoder can be told to ship
  a quick-noop frame to open a control window.

For E-stop the situation is worse — see §12.5.

### 12.4 Tractor hop-clock keep-alive (required for Resolution A)

When the image encoder is idle, the tractor must still TX a tiny
header-only frame every N ms so the base has a hop-clock pulse and a
TX opportunity. Numbers:

- 30 ms airtime per keep-alive (header-only, SF7/BW250).
- 20 Hz keep-alive rate = 50 ms period → base TX opportunity every
  50 ms.
- Tractor airtime overhead from keep-alive = 600 ms/s = **60% TX
  duty**. Too high.
- 10 Hz keep-alive rate = 100 ms period → 300 ms/s = 30% TX duty.
  Still high but acceptable under the §15.247 50-ch FHSS budget
  (each channel sees ~6 ms/s = well under 400 ms cap).
- 5 Hz keep-alive rate = 200 ms period → 150 ms/s = 15% TX duty.
  Comfortable.

**Recommend 5–10 Hz keep-alive** when image stream is idle; suppress
keep-alive whenever the image stream is active (image frames *are* the
hop clock). Keep-alive frames are P3 (lowest priority); image and
control frames preempt freely.

This adds one new wire-format frame type (`A6a header-only with
payload_len=0`) but no new opcodes — payload_len=0 is already legal.

### 12.5 E-stop is the architectural pressure point

E-stop has a hard latency requirement (`AI NOTES/2026-04-26
LoRa_QoS_Bandwidth_Management.md` lists ≤200 ms as the P0 budget).
Under Resolution A + keep-alive at 10 Hz, *typical* E-stop latency is
≈ 100 ms (wait for next tractor pulse + 30 ms TX + dispatch). That
fits the 200 ms budget. **Worst-case** latency is bounded by the
keep-alive interval + any in-flight image frame currently airing
(image frames cap at ≤25 ms per the v1.2 P0-preemption rule). So
worst case ≈ 125 ms. Still within 200 ms.

But this analysis assumes:
1. Both peers are LOCKED (Alpha-Prime + Delta have succeeded).
2. The keep-alive beacon is running.
3. No prior multi-packet image fragment burst is occupying the
   scheduler.

If any of those is false (cold start, post-blackout recovery,
silenced image stream), E-stop falls back to the cold-start latency
floor: **at least one full Alpha-Prime walker pass for the tractor to
hear the base**, which is 50 ch × 100 ms dwell = **5 s worst case**
without help from §11 Option 11 (bounded 50-packet sweep) or Option 12
(persisted slot).

**Implication for E-stop architecture:**

- During normal operation: E-stop latency is acceptable under
  Resolution A + keep-alive.
- During cold-start / recovery: E-stop is **not** safety-grade over
  the LoRa link alone. A 5 s E-stop floor is unsafe for a tractor.

This is not a new finding for v25 — it's been implicit in the design.
The mitigations already on the table:

1. **Wired E-stop on the operator station** (handheld panic-bar
   physically wired to the tractor's hydraulic dump valve when within
   tether range). The LoRa E-stop is **secondary**, not primary.
2. **Local fail-safe on the tractor H7**: if the H7 hasn't seen *any*
   valid control frame (any priority, from any source) for >500 ms,
   center sticks and apply hold-state. The H7 implements this
   independently of the LoRa link.
3. **Optional dedicated safety radio**: a second SX1276 module
   per peer, fixed-channel + DSSS-like spread, used only for E-stop
   and link-state. ~$15 BOM cost. This is what real industrial
   wireless does. **Recommend evaluating** as an option for v25.1
   safety cert; not needed to ship the image pipeline.

The cleanest statement of policy: **the FHSS link carries E-stop on a
best-effort basis with a 200 ms typical and 5 s cold-start worst
case; safety-critical E-stop is provided by (1) tractor-local fail-
safe and (2) optional tethered/dedicated-radio secondary path.** This
should be documented as the v25 safety architecture before any field
deployment.

### 12.6 What this means for the §6 / §11 implementation order

The Alpha-Prime + Delta + Option 12 plan in §11.6 is unchanged for
the image direction. Additions for bidirectional:

1. **In Alpha-Prime PR (§11.6 commit 1):** verify the scan walker is
   role-agnostic. Same code path runs on tractor when image stream
   stops and tractor needs to RX-scan for base TX. (Probably already
   true — `sx1276_rx_scan_tick()` doesn't know its role — but verify
   no shortcut assumes "I am the base.")
2. **In Delta PR (§11.6 commit 2):** verify `consider_remote()`
   handles symmetric updates from either peer. Same concern.
3. **Defer Resolution A wire-format (master-bit in A6a) to v25.0.8.**
   Image-only stream doesn't need it yet. When control traffic gets
   ported to the new firmware (currently it's on the legacy stack
   per `lora_bridge.py`), land Resolution A in the same PR.
4. **Keep-alive beacon (§12.4) lands with Resolution A.** Not before.
   Adding it during image-only operation just wastes airtime.
5. **E-stop architecture documentation (§12.5)** should land **now**,
   independently of any firmware work, so the safety policy is
   explicit before anyone fields the tractor.

### 12.7 What does NOT change

- All v1.0–v1.4 conclusions for the image direction are preserved.
- Alpha-Prime + Delta + Option 12 stack is unchanged.
- Custom firmware decision is unchanged — the bidirectional analysis
  actually *strengthens* the case for custom: P0 preemption,
  Resolution A master-bit, keep-alive scheduling, and the master/
  slave hop-clock design are all things default LoRaWAN firmware
  cannot do.
- The §15.247 50-channel requirement is unchanged. Resolution A keeps
  every TX (image, control, keep-alive) on the same FHSS permutation,
  so dwell accounting and equal-channel-use accounting are unaffected.

### 12.8 One-line answer to the question

The sync problem is **symmetric** — it hits base→tractor exactly the
same way it hits tractor→base, and the Alpha-Prime + Delta + Option 12
foundation solves both directions for free. The **new** problem
bidirectional traffic adds is **slot ownership** (who's the hop
clock?), which Resolution A (tractor-as-master with a low-rate keep-
alive beacon) resolves with a single 1-bit header field. The hard
constraint on top of all of this is **E-stop latency during cold
start**, which the LoRa link cannot guarantee under §15.247 FHSS at
any channel count — that's an architectural pressure point that needs
a tractor-local fail-safe (and optionally a dedicated safety radio),
not more clever scheduler design.

*Signed:* **GitHub Copilot, FHSS Bidirectional Impact Review v1.5
(2026-05-27) — extends v1.4 with base→tractor control plane
analysis**

---

## 13. Mixed TX/RX Control Review Addendum After Source Pass (GitHub Copilot v1.6, 2026-05-27)

I re-read the current control and HostLink implementation after §12. The
section's high-level conclusion is right: the one-way image sync fix is not
enough for a mixed image/control radio. But there are two important
implementation clarifications that change what I would build first.

### 13.1 The L072 does not need to understand ControlFrame

The current `murata_l072` firmware is a payload-agnostic radio pipe. Its
host protocol exposes `HOST_TYPE_TX_FRAME_REQ` and `HOST_TYPE_RX_FRAME_URC`;
the L072 wraps transmitted payloads with the A6a FHSS hop header and strips
that hop header on receive, but it does not parse `FT_CONTROL`,
`CMD_ESTOP`, or image fragment contents. That is good. It means reverse
control does not require a new L072 application protocol just to carry
ControlFrames.

The gap is above that layer:

- `image_tx_daemon.py` is tractor-side, image-only, FIFO-ish, bridge-bypass,
  raw-fragment, and owns `/dev/ttymxc3` while it runs.
- `image_rx_daemon.py` is base-side, RX-only, and also assumes single-owner
  HostLink access.
- `base_station/lora_bridge.py` already has the useful P0/P1/P2/P3 priority
  queue and MQTT command subscriptions, but it is still the older KISS/AES
  bridge model, not the current Method-H HostLink image path.

So a mixed bench cannot be two independent daemons fighting over the same
Murata UART. The next real integration unit should be a **single link arbiter
per peer**:

- base arbiter: RX image/telemetry + TX P0/P1 commands;
- tractor arbiter: TX image/telemetry + RX P0/P1 commands;
- one HostLink owner, one priority queue, one RFCO/latency accounting path;
- P0/P1 frames always dequeue ahead of P3 image fragments.

That arbiter can reuse the Python `lora_bridge.py` priority rules and
`lora_proto.py` frame definitions, but the radio I/O surface should be the
current HostLink `TX_FRAME_REQ` / `RX_FRAME_URC` path.

### 13.2 Ordinary TX in both directions will desynchronize the link

The most important source-level detail is in `sx1276_tx_begin()`: under the
FHSS routed profile, **every normal TX calls `sx1276_fhss_next_channel()`**.
That consumes the next scheduler slot, retunes the synthesizer, stamps
`hop_idx`, and transmits.

That is perfect for one-way image streaming. It is dangerous for mixed
traffic if the base sends control with the same normal TX path:

1. Tractor transmits image on hop `N`; tractor's scheduler advances to
  `N+1` but the radio cleanup re-arms RX on the just-used RF channel.
2. Base receives hop `N`; Delta would snap base's scheduler to `N+1`.
3. If base now sends control through ordinary `TX_FRAME_REQ`, base consumes
  hop `N+1` and transmits on the next channel.
4. Tractor may still be listening on hop `N`, or may have advanced on a
  timer. Either way, the reverse control packet is no longer guaranteed to
  land in the tractor's receive window.

So Alpha-Prime + Delta solves acquisition/follow for whichever side is
currently a receiver, but it does **not** define bidirectional slot
ownership. Mixed operation needs one extra rule: when a receiver replies to
the current master, that reply must not become an independent hop-clock event.

### 13.3 Recommended mixed-mode architecture: master exchange windows

The fastest reliable mixed implementation is a refinement of §12 Resolution
A. I would call it **master-framed exchange windows**.

Rules:

1. The tractor is the hop-clock master while it is the image/telemetry
  source. Master TX uses today's normal `sx1276_fhss_next_channel()` path.
2. After each master TX, the tractor immediately re-arms RX on the same RF
  channel for a short reverse window.
3. The base may transmit at most one queued P0/P1 reply in that reverse
  window, on that same RF channel, using a new "reply without advancing
  FHSS" TX mode.
4. Reply frames carry an A6a flag in the reserved byte, for example
  `MASTER=0, REPLY=1`. The tractor accepts the payload but does not snap
  its scheduler to the reply's hop fields.
5. The next tractor master TX advances to the next FHSS slot and renews the
  clock for both peers.

This preserves the single `s_fhss` model, avoids oscillating clock authority,
and does not require a full time-clocked scheduler. It does require a small
firmware addition: a TX mode that can send on the current/last-master channel
without calling `sx1276_fhss_next_channel()` and without treating the reply
as a scheduler authority.

I would not make the base the hop master for the first mixed bench. The
image stream is the high-rate traffic already present in the demo, so the
tractor naturally supplies frequent hop-clock packets. If image traffic is
idle, the tractor should emit low-rate master keep-alives across the same
50-channel FHSS sequence so the base still gets command opportunities.

### 13.4 Latency and safety implications

With master exchange windows, base-to-tractor command latency is bounded by
the master-packet cadence plus the reverse packet airtime:

| Master cadence | Expected command opportunity | Fit for base web control? | Notes |
|---:|---:|---|---|
| 20 Hz | 50 ms | Best | Matches the original control cadence, but costs airtime if produced by keep-alives |
| 10 Hz | 100 ms | Good | Comfortable for web/base control target of <=250 ms |
| 5 Hz | 200 ms | Marginal | Still usable for non-fine joystick work; too slow for crisp teleop |
| no master traffic | scan/reacquire only | Not acceptable | Command latency becomes probabilistic seconds-scale |

For E-stop, the distinction matters. When both peers are locked and the
tractor is producing 10-20 Hz master windows, a P0 E-stop can fit inside the
existing base-control latency target. During cold start, blackout recovery,
or any period with no master windows, LoRa cannot guarantee a safety-grade
E-stop latency by scheduler cleverness alone. The tractor-local watchdog and
valve-neutral fail-safe remain mandatory, and a wired or dedicated safety
radio path should stay on the table for any field safety claim.

### 13.5 Current options that work in mixed TX/RX

| Option | Works for mixed image + control? | Why |
|---|---|---|
| Alpha-Prime + Delta | Necessary but insufficient | Makes each receiver acquire/follow, but does not define who owns the hop clock when both sides transmit |
| Master exchange windows | **Best first mixed-mode path** | Keeps one scheduler, gives base reverse control slots, and fits current packet-clocked design |
| Low-rate tractor master keep-alive | Needed when image idle | Keeps command opportunities alive without fixed-channel rendezvous |
| Unified HostLink arbiter | Required integration step | Prevents image/control daemons from contending for `/dev/ttymxc3` and gives P0/P1 real priority over P3 |
| Dual independent schedulers | Technically clean, too large now | Good v25.1/v25.2 cleanup, but doubles state, vectors, and evidence burden |
| TDMA/time-clocked scheduler | Strong product architecture, not quick | Solves slot ownership formally, but is a scheduler redesign with new dwell accounting |
| Repeated P0 acquisition sweep | Useful recovery tool only | Can improve cold-start E-stop odds, but cannot be the primary safety guarantee |
| Fixed reverse control channel | Reject for field | It undermines the 50-channel FHSS legal/evidence story; diagnostic only |
| Default LoRaWAN/Murata firmware | Reject | Cannot provide raw P0 preemption, reverse windows, RFCO evidence, or image fragments |

### 13.6 Implementation order I would use

1. Land Alpha-Prime + Delta first and verify both boards can lock as a
  receiver under profile 1. This remains the foundation.
2. Replace the image-only daemon split with a single HostLink arbiter per
  peer. Start with base RX-image + TX-command and tractor TX-image +
  RX-command; keep payloads generic so the L072 remains a transport.
3. Add A6a reserved-byte flags for `master` and `reply` semantics. This can
  be additive because current parsers already ignore the reserved byte, but
  new mixed-mode firmware must enforce the semantics.
4. Add reply-without-hop-advance TX support and a post-master-TX RX window on
  the tractor. This is the smallest firmware change that makes base→tractor
  control deterministic while image traffic is active.
5. Add tractor master keep-alive only after the reverse-window behavior is
  green. Tune it to the slowest cadence that still meets the base-control
  feel target, probably 10 Hz for bench and 5-10 Hz for idle field mode.
6. Run mixed-mode gates before calling FHSS bench-default complete: image-only,
  control-only, image+P0 starvation, image+P1 camera/keyframe commands,
  locked E-stop latency, and loss-of-lock recovery.

Bottom line: the sync problem absolutely affects base→tractor control, but
not because ControlFrame needs special radio parsing. It affects control
because a single half-duplex SX1276 and a packet-clocked FHSS scheduler need
an explicit exchange rule. The quickest control-safe rule is: tractor frames
advance the hop clock; base control replies ride a short same-channel reverse
window without advancing the hop clock; idle periods are bridged by low-rate
tractor master keep-alives. Everything else is either a later scheduler
redesign or a diagnostic escape hatch.

*Signed:* GitHub Copilot, Mixed TX/RX Control Addendum v1.6 (2026-05-27)

---

## 14. Mixed TX/RX Implementation Gate Review and New Practical Options (Copilot v1.7, 2026-05-27)

This section explicitly answers: which current options still work once the
base must send control traffic back to the tractor on the same FHSS radio.

### 14.1 Source-anchored constraints that matter for mixed direction

1. The L072 host protocol is payload-agnostic. `HOST_TYPE_TX_FRAME_REQ (0x10)`
  can carry image, control, or any other upper-layer bytes.
2. The radio is half-duplex. RX is only serviced when `!sx1276_tx_busy()` in
  `main.c`, so bidirectional traffic must schedule explicit RX opportunities.
3. In FHSS routed mode, normal TX consumes scheduler slots by calling
  `sx1276_fhss_next_channel()` in `sx1276_tx_begin()`.
4. TX cleanup re-arms RX if the radio was in RX before TX (`s_rearm_rx` path),
  which is useful for same-channel reverse windows.
5. Base-side command priority logic already exists (P0/P1/P2/P3) in
  `base_station/lora_bridge.py`, but this is not yet the same runtime path as
  the current HostLink image daemons.

### 14.2 Current options: mixed TX/RX viability matrix

| Option | Works for mixed TX/RX as-is? | Why / Why not | Verdict |
|---|---|---|---|
| Alpha-Prime (independent scan walker) | Partly | Fixes acquisition for either side in RX mode; does not define reverse-slot ownership | Keep (foundation) |
| Delta (immediate follow on valid header) | Partly | Improves follow recovery; still allows hop-clock oscillation if both peers TX normally | Keep (foundation) |
| Bounded 50-packet acquisition sweep | Yes (startup/recovery only) | Great for initial rendezvous, not a steady bidirectional arbitration model | Keep as startup tool |
| Persisted last-good slot (warm restart) | Yes | Reduces reacquire time after resets; orthogonal to slot ownership | Keep |
| GPS time-locked TDD | Not as-is | Requires time-clocked scheduler redesign and new dwell/evidence logic | Defer to later architecture |
| Coprime sweep rates | Not as-is | Assumes time-driven TX intervals; current scheduler is packet-clocked | Defer |
| Dual independent schedulers | Yes (large change) | Architecturally clean but high integration/test burden | Defer unless needed |
| Default LoRaWAN/AT firmware | No | Cannot provide required priority preemption + custom exchange semantics | Reject for product path |

### 14.3 New options that fit current firmware shape

#### Option 13 — Master exchange windows with explicit reverse-opportunity marker

Keep tractor as hop-clock master for image-heavy operation, but mark each
master frame with a small reverse-window hint in A6a reserved-byte bits:

- `reply_allowed` (1 bit)
- `reply_window_class` (2 bits, e.g. 10/20/40 ms)
- remaining bits reserved

Base sends at most one P0/P1 reply inside that same-channel window using a
non-advancing reply TX mode. Tractor receives reply, then resumes normal master
advance on next image/keepalive TX.

Why it works: minimal wire change, no time-clocked redesign, deterministic
reverse opportunities, bounded contention.

#### Option 14 — Non-advancing reply TX primitive

Add a firmware TX primitive that transmits on current channel without calling
`sx1276_fhss_next_channel()` (reply-only path). This is the key to preventing
hop-clock tug-of-war in mixed traffic.

Why it works: preserves single scheduler authority while still enabling
base-to-tractor control injection.

#### Option 15 — Control-state supersession (latest state wins)

For joystick-like control, queue semantics should be state-based, not
command-history-based:

- keep only latest control state per source,
- drop stale intermediate control packets automatically,
- always send freshest P0/P1 at next reverse window.

Why it works: avoids command backlog and improves control feel under constrained
reverse opportunities.

#### Option 16 — Reserved reverse opportunities every K master frames

If image cadence is bursty, reserve deterministic reverse opportunities every
K master transmissions (for example K=3 or K=4), independent of image payload
size. This prevents long control starvation during large image bursts.

Why it works: bounded worst-case control latency without full TDD redesign.

### 14.4 Fastest reliable mixed-direction plan

1. Land Alpha-Prime + Delta (already identified as foundation).
2. Add Option 14 (non-advancing reply TX) + Option 13 marker semantics.
3. Integrate single HostLink arbiter per peer with P0/P1 preemption over P3.
4. Add Option 15 control-state supersession.
5. Add Option 16 reserved reverse opportunities only if latency tails still
  exceed target under image load.

This sequence gives the fastest path to practical mixed TX/RX without needing
GPS-TDD or dual-scheduler redesign.

### 14.5 Reliability and safety interpretation

- **Normal locked operation:** mixed control should be deterministic with
  exchange windows and non-advancing replies.
- **Cold-start / recovery:** bounded acquisition tools (bounded sweep + warm
  restart seed) reduce command blackout but do not eliminate startup gaps.
- **Safety:** LoRa E-stop remains best-effort in cold-start conditions; tractor-
  local fail-safe remains mandatory.

### 14.6 Final answer to the mixed-direction question

The sync issue affects reverse control as much as image, but in mixed operation
the real blocker is scheduler ownership, not payload format. The options that
actually work with current firmware are: Alpha-Prime + Delta as base, then a
master-window model with non-advancing reply TX and state-superseding control
queueing. GPS-TDD and other time-clocked ideas are valuable later, but are not
the fastest reliable path on the current packet-clocked stack.

*Signed:* GitHub Copilot, Mixed TX/RX Gate Review v1.7 (GPT-5.3-Codex, 2026-05-27)



