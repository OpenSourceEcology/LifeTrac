# RS-3.3 long-train attempt — bright scene + forced keyframes + strict hold (2026-09-07)

**Verdict: two findings, neither the one this leg was designed for.**
(1) **Long camera trains are not reachable by scene choice.** With a
bright, detailed, moving scene and 39 keyframe requests delivered, the
encoder never produced a frame over 600 B — keyframes at this
resolution/quality are ~500 B, i.e. 2–3 fragments, not the 2.4 KB /
10-fragment trains of 2026-07-31. Encode-to-fit is doing its job; the
long-train case needs a different *encoder configuration*, not a
different picture. (2) **Under a heavy command plane the strict hold is
no longer loss-free: 4.1 % loss, 14 timeouts, and the losses sit on the
penultimate-of-two in 12 of 14 cases — the RS-12 signature — while RF
corruption stayed low (9 dumps).** This is the first leg where the
NO_PARK_LAST command-plane interaction has shown up as loss, and it
argues against a default flip until the URC path is fixed in firmware.

## Setup

- Archive `radio_monitor_20260907_153624_a42f0f14` (git a42f0f14):
  `-TxFeed camera -KfRequestDisable 0 -ForceFrfHz 927500000
  -NoParkLast 1 -LogFragArrivals 1 -DurationS 300 -Archive`.
  Self-substantiating: `params.txt` `no_park_last=1`, TX daemon
  `no_park_last=1 gap_ms=80 pipeline_depth=2`.
- Camera zoomed onto the video player region of the bench laptop; the
  railroad cab-view video restarted from its (bright, high-detail)
  route-map intro at launch. YouTube autoplay had chained to an
  unrelated video between legs — restored before the stream went live;
  do not assume a browser tab is still on the intended content.
- Keyframe injector (`kf_inject.py 15 20`) started on the FIRST
  PUBLISHED FRAME rather than a fixed delay — **timing fix validated:
  39 `REQ_KEYFRAME` receptions at the tractor ≈ 20 × 2 copies**, all
  in-leg (08-22 got 11 of 24).
- Same-day channel check: 927.5 read **1 hot / max −72 dBm** in 60 s —
  its first non-zero reading since 2026-08-16 (previously clean 6/6).
  A single burst ~30 dB below the known emitters; recorded, not
  disqualifying. Radios parked to SLEEP (0x80 both) afterward.

## Numbers (base brackets)

| metric | value |
|---|---:|
| loss | **40/980 = 4.1 %** |
| timeouts | **14** |
| frames published | 583 |
| TX train lengths | **1 × 258, 2 × 359, 3 × 6 — nothing longer** |
| publish sizes | 294 < 300 B, 304 in 300–599 B, **none ≥ 600 B** |
| pair spacing (fw clock) | 206.6 ms (n=353) — hold active |
| lost-index profile (2-frag trains) | **idx 0 = 12, idx 1 = 2** — penultimate 86 % vs uniform 50 % |
| base command sends | **145**: 102 × `TILE_STALE`, 43 × `REQ_KEYFRAME` |
| `REQ_KEYFRAME` received at tractor | 39 |
| radio Δ (base) | dio0 1196, rx_ok 1010, crc_err 25, tx_ok 147 |
| crc closure | **Δcrc_err 25 vs crc_dumps 9 — 16 corrupt-capture URCs missing** |
| Δrx_ok − URCs | 70 (ack-contaminated; ~290 ack copies expected from 145 commands) |
| identity residue | 14 |

## Reading

**Finding 1 — keyframe size is an encoder property here.** Bright
scene, motion, 39 delivered keyframe requests, and the largest frame
was 497 B. The 07-31 run's 2381–2430 B keyframes came from a different
encoder operating point (its log recorded `byte_budget=2436`), not from
a brighter room. On the current camera path a keyframe is 2–3
fragments, full stop. The ≥10-fragment camera train exists only if the
encoder is configured to produce it (resolution, quality, or budget).
That reframes the residue from "bench conditions" to "a config axis to
decide on" — and it means the synth 13-fragment legs remain the only
long-train evidence for the hold.

**Finding 2 — the hold is not loss-free under a heavy command plane.**
Compare the two camera legs 20 hours apart, same channel, same hold:

| | motion leg (09-06) | this leg |
|---|---:|---:|
| base command sends | 104 | 145 |
| kf requests received | 2 | 39 |
| loss | 0.3 % | **4.1 %** |
| timeouts | 0 | **14** |
| crc dumps | 6 | 9 |

RF corruption barely moved (6 → 9 dumps); loss went up 13×. The lost
fragments are overwhelmingly the first of a two-fragment train (12/14)
— the penultimate — which is the RS-12 overwrite signature, appearing
*despite* the 206 ms pair spacing the hold enforces. And 16 of 25
corrupt-capture URCs never reached the host either. The economical
reading is **URC-path contention from command-plane traffic**: with
~290 ack copies arriving at the base in 300 s on top of ~1000 image
fragments, the L072's single pending-URC slot is being overwritten by
*command/ack* URCs, not by the short final fragment the hold was built
to separate. The hold fixes the ride; it cannot fix a second writer.
n=1; the direction is strongly supported by the index profile and the
missing dump URCs, the magnitude is not pinned.

**Consequence for the NO_PARK_LAST default decision:** do not flip. The
08-24 instrumentation legs proved the hold removes the ride-induced
lock; this leg shows a *different* writer reaches the same slot under
load. The firmware fix (double-buffered URC path / minimum inter-fire
spacing) addresses both; the host-side hold addresses one. The flash
session's `rx_urc_lost` counter would distinguish them directly.

**Also on the record:** the stale-scan flood recurred (102 `TILE_STALE`
in 300 s, matching 09-06's 101) — it is a stable property of the fresh
UI under motion, not a one-off; and this is the second consecutive
camera leg with a crc-closure miss, now large (16), consistent with the
contention reading above.

## Quality

Mid-motion canvas archived (`long_leg_canvas_midmotion_20260907.png`,
147 KB — the most detail of any capture so far). The zoomed aim frames
the video content edge to edge; the scene reads clearly.
