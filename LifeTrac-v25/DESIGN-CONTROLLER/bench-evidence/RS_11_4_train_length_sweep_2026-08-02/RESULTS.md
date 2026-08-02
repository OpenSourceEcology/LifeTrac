# RS-11.4 train-length sweep — the cumulative-loss question, re-baselined (2026-08-02)

**SHA under test:** main @ `bb21b053`. Six 300 s runs, synth local feed,
pinned operating point (v3 depth 2, DTS profile 2, smooth pacing,
TrainGapMs 40, SynthFps 2, KfRequestDisable 1, ProbeEcho 0), `-Archive`
bundles `radio_monitor_20260801_21{1446,2010,2534,3057,3622,4147}_bb21b053`.
SynthBudgetB 3000 / 1500 / 750 → 13 / 7 / 4 fragments per train, n=2 each.

## Verdict in three parts

1. **The D1 in-train gradient is GONE — the original RS-11.4 question is
   moot.** D1 (2026-07-29, token-bucket era) attributed 67 in-train losses
   in one run with indices 8–11 taking 37/67. Under smooth pacing (default
   since 2026-07-30) the in-train attributed loss collapsed to 2–8 per run
   (~0.1–0.4% of fragments) with NO index gradient at any train length.
   The cumulative mechanism was almost certainly the AirtimeBudget token
   bucket — the same family as the index-10 notch — and shipping smooth
   pacing fixed it before this sweep ran. None of the four RS-11.4
   candidate mechanisms (PA thermal, dwell, backpressure, clock drift)
   shows in-train.

2. **The dominant remaining loss is a PER-TRAIN-BOUNDARY event, and it is
   much bigger than the old floor.** Raw fragment shortfall
   (frags_tx − rx_frames) scales INVERSELY with train length — the
   opposite of cumulative-in-train:

   | budget | frags/train | trains | frags_tx | raw lost | raw loss% | reassembler timeouts | trains timed out | frames published |
   |-------:|------------:|-------:|---------:|---------:|----------:|---------:|--------:|--------:|
   | 3000 B | 13 | 190 | 2419 | 118 | 4.9% | 64 | 34% | 122 |
   | 3000 B | 13 | 191 | 2438 | 129 | 5.3% | (n=2 twin) | | 122 |
   | 1500 B |  7 | 382 | 2439 | 148 | 6.1% | 94 | 25% | 280 |
   | 1500 B |  7 | 382 | 2440 | 150 | 6.1% | (twin) | | 276 |
   |  750 B |  4 | 582 | 1891 | 179 | 9.5% | 133 | 23% | 436 |
   |  750 B |  4 | 583 | 1895 | 190 | 10.0% | (twin) | | 428 |

   `published + timeouts ≈ trains` in every run (122+64≈190, 280+94≈382,
   436+133≈569) — every train either completes or times out; there is no
   third bucket. Loss events per boundary are near-constant at ~0.20–0.25
   after subtracting the 17 periodic 0x63 radiations per run (see §4),
   while events per second double across the sweep — the event is tied to
   the boundary, not to wall clock. Each event costs ~1.3–1.8 fragments
   (raw lost / timeouts), consistent with the `post_loss` gap median of
   ~234 ms ≈ exactly two 116.9 ms fragment slots.

3. **Instrument gap (violates the standing reconcile rule):** the
   `lost_frag_idx` attribution counts 2–8 losses per run against a raw
   delta of 118–190 — it never attributes fragments belonging to trains
   that TIME OUT, which is now ~95% of all loss. Every per-index claim
   from it (including D1's, and the F4 gate answer) described only the
   completing-train subset. The fix is the next increment: attribute the
   missing indices of evicted/timed-out trains before eviction, then
   re-read the per-index histogram of FAILED trains — mass at index 0–1
   discriminates RX re-arm at the boundary from TX-side teardown at the
   tail.

## 4. Secondary observations

- **17 × 0x63 (ENCODE_MODE) radiations in every run** — the retained
  encode-mode override retries unacked for the whole session because the
  synth feed has no camera_service to ack it (pending-ack machinery never
  gives up; throttled to ~1/18 s). Each TX blinds the base's RX briefly:
  a real robustness point (bounded retries needed) and a bench artifact
  to subtract from loss accounting. Constant across tiers, so it does not
  drive the inverse scaling.
- Boundary gap median grows as trains shorten: 178 ms (13/7-frag) →
  274.7 ms (4-frag) — prepare-ahead has less in-train time to build the
  next frame at short trains. Not separately investigated.
- Frame-level throughput: 0.41 / 0.93 / 1.45 frames published per second
  across the tiers — short trains publish more frames but pay a higher
  per-fragment tax; with a ~23–34% train-failure rate, the boundary event
  is now the single largest throughput lever on the table.

## 5. What this changes

- RS-11.4 CLOSED (question answered by re-baseline: mechanism was the
  bucket; already fixed).
- New item RS-11.5 opened: timeout-loss attribution (host-side, small),
  then the boundary-event discrimination (index 0–1 vs tail) it enables.
- The ~3.5% "loss floor" number should stop being quoted — the current
  floor is ~5% at 13-frag trains and it lives at boundaries, not in-train.
