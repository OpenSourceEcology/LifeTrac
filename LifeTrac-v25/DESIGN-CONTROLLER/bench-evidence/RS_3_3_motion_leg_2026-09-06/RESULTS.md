# RS-3.3 motion leg — camera + moving scene + strict hold + live keyframe/stale machinery (2026-09-06)

**Verdict: the camera path streamed a moving scene to the operator
console over LoRa at 0.3 % loss with zero timeouts, while the base ran
103 live commands mid-stream under the strict hold. The composed image
was observed on the base-station website and the exact canvas is
archived (`motion_leg_canvas_20260906.png`). The specific residue this
leg was meant to close — LONG multi-fragment camera trains under the
hold — was NOT reached: even with sustained motion and keyframes
enabled, the darkened bench scene never produced a train longer than
2 fragments. What it did produce is 49 two-fragment trains, which are
on-mechanism (a short final riding a full first) and all completed.**

## Setup

- Tractor camera aimed at the bench laptop screen; a railroad cab-view
  video (continuous scene motion) played fullscreen for the whole leg.
- Archive `radio_monitor_20260906_181005_edebe543` (git edebe543):
  `-TxFeed camera -KfRequestDisable 0 -ForceFrfHz 927500000
  -NoParkLast 1 -LogFragArrivals 1 -DurationS 300 -Archive`.
  **Self-substantiating**: `params.txt` `no_park_last=1`, TX daemon
  startup `no_park_last=1 gap_ms=80 pipeline_depth=2`.
- Channel 927.5 spot-checked immediately before (60 s, 0 hot, max −96)
  — **clean 6/6** across every survey since 2026-08-16.
- Base station website: fresh-from-repo `web_ui` on port 8090 (not the
  stale deployed copy), retained control topics cleared on the base
  broker beforehand (dual-broker rule). PC-side broker was down;
  control rode the primary broker (daemon degrades gracefully).
- Radios had been in verified LoRa SLEEP for 13 days (boards up 14 d
  22 h, no reboot); `/tmp` had been age-cleaned by tmpfiles and was
  re-pushed. Radios parked back to SLEEP (0x80 both) after the leg.

## Numbers (base brackets; leg report + arrival analysis)

| metric | value |
|---|---:|
| loss | **2/650 = 0.3 %** |
| timeouts | **0** |
| frames published | 599 |
| TX train lengths | **560 × 1-frag, 49 × 2-frag, none longer** |
| publish sizes | 565 < 300 B, 47 in 300–999 B, none ≥ 1000 B |
| 2-frag pair spacing (fw clock) | **216.5 ms** (n=49) — the hold signature; the ride is 42 ms |
| base command sends | **104**: 101 × `CMD_OP_TILE_STALE`, 2 × `REQ_KEYFRAME` (both converged: keyframe received) |
| radio Δ (base) | dio0 784, rx_ok 669, crc_err 8, tx_ok 104 |
| crc closure | **Δcrc_err 8 vs crc_dumps 6 — NOT exact** (first non-closure of the campaign; 2 corrupt demods without a dump line; on the record, unexplained) |
| identity residue | 3 |

The two lost fragments were single-fragment trains (no partial ⇒ no
timeout), so **every one of the 49 two-fragment trains completed**.

## Quality, as observed

Screenshot and canvas capture mid-leg: the laptop screen is
recognizable — the video player, the orange locomotive hood and track,
the sidebar thumbnails; the "4 Hz · tile stream" badge and SNR +6 dB
in the status line. The image is soft with visible tile blockiness and
is dark overall (room lighting; the screen is the bright object).
Motion is rendered as a rolling refresh of changed tiles rather than
as whole-frame updates. Usable for "what is the machine looking at";
not yet a video feed. `motion_leg_canvas_20260906.png` is the exact
384×256 composed canvas pulled from the page's `<canvas>` element, not
a screenshot.

## What this does and does not close

**Closed:** the camera path runs under the strict hold with a moving
scene and a busy command plane. 49 short-final trains (the RS-12 ride
geometry) all delivered. 103 commands dispatched live mid-stream with
no loss penalty. Combined with the 08-24 instrumentation legs
(mechanism signature) and the 08-22 leg 3 (11 injected kf), the
NO_PARK_LAST hold now has live-traffic evidence at every train length
the camera path actually emits on this bench.

**Not closed, stated plainly:** long trains (≥10 fragments) under the
hold. The 07-31 run produced 2.4 KB keyframes in daylight; tonight's
dark room compressed keyframes into ≤2 fragments, so the sustained
long-train case remains unexercised on the camera path. Motion alone
is insufficient — it raises the tile *rate* (4 Hz), not the fragment
*count*. To reach it: a bright, detailed scene (lights on, or a
high-detail image on the screen) so keyframes hit the 3000 B budget,
plus keyframes forced on a cadence (`kf_inject.py` is in the 08-22
evidence dir). That is a 10-minute bench item, not a design change.

## Observations for the record

1. **Stale-scan flood under motion.** 101 `TILE_STALE` requests in
   300 s (one per ~3 s). The stale-scan horizon is tuned for a static
   scene (memory: horizon must exceed the sweep rotation); under motion
   tiles go stale faster than the sweep refreshes and the base chases
   them. Each request costs command airtime the image would otherwise
   have. Not a bug verdict — the requests did no visible harm at 0.3 %
   — but it is the RS-4.14 interaction reappearing from the *fresh*
   UI, and the horizon/rate deserves a motion-aware rule before field
   use.
2. **The harness SWD-resets the tractor L072 at launch**
   (`[RESET] TX L072 via OpenOCD SWD ... swd_reset_spawned` in the
   launch transcript). This is the mechanism behind the 08-22 leg-3
   tractor bracket invalidity (`tx_ok` 1229 → 620): the reset is
   spawned in the background and does not always complete, so tractor
   pre/post deltas are unreliable across launches. Base brackets remain
   the analysis basis; tractor brackets are archived but not used.
3. **crc closure missed by 2** for the first time in the campaign
   (8 corrupt demods, 6 dump lines). Unexplained; watch for recurrence.
4. **Long-quiesce behaviour**: 13 days with Linux up and radios asleep
   cost nothing but a `/tmp` re-push (tmpfiles age-cleaning, not a
   reboot). Quiesce-instead-of-halt is now validated at the two-week
   scale.
