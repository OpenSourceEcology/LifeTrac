# F11 on-air A/B — per-gap keyframe request demoted (2026-08-02)

**SHA under test:** branch `f11-kf-on-seq-gap` (gate commit `7a1dfb1c` on
main @ 02a84f2e). **Verdict: PASS — default flipped to OFF** in the
follow-up commit, per the measure-then-flip protocol (F9 precedent).

## 1. What was measured

web_ui's gap-tolerant canvas requested a keyframe on EVERY base_seq gap,
including a single lost delta frame. F10's 0x6C stale-tile path now detects
and repairs exactly that damage tile-by-tile, so F11 gates the per-gap
request behind `LIFETRAC_KF_ON_SEQ_GAP`. Suppression applies ONLY to
pure-gap requests (structured `seq_gap`/`tile_error` causes on
CanvasUpdate); cold start, grid mismatch, and tile-decode errors always
pass through.

Setup identical to the F10 acceptance (camera feed, DTS profile 2, v3
depth 2, smooth pacing, base-board web_ui with 30 s stale horizon / 3 s
period), gate OFF. All times host/base UTC.

## 2. Control (A side): F10 acceptance surgical window, gate ON

From `bench-evidence/F10_tile_stale_acceptance_2026-08-01/RESULTS.md` §5:
three seq-gap events (one 18-frame resume gap + two natural single-frame
air losses) → three req_keyframe publishes → two 0x60 radiations (10 s
throttle) → keyframe trains granted, each a multi-frame cost at the
243 B/frame budget.

## 3. Treatment (B side): gate OFF, this session

- **Cold start still keyframes (keep-path, on air):** f11_webui restarted
  mid-stream 01:44:30 with an empty canvas; "delta arrived before any
  keyframe" published 01:44:33, radiated (two 0x60 attempts, converged) —
  the ungated path works.
- **Six gap events, six suppressions, ZERO keyframe requests:**
  - Gaps 1–3 induced (3 s rx stops at 01:41:03 / 01:41:47 / 01:42:32):
    req_keyframe count in the broker tap stayed 0 (pre-log-fix, the
    suppression line was swallowed by the root logger's WARNING default —
    fixed to logging.warning and re-verified).
  - Gap 4 induced (01:45:27): `kf-on-gap suppressed (F11): base_seq gap:
    got 205, expected 194` (11 frames).
  - Two NATURAL single-frame air losses in the same window — the exact
    event class the control paid keyframes for: `got 234, expected 233`
    and `got 6, expected 5` — both suppressed.
- **Canvas health under suppression:** short-gap losses re-swept within
  the 24 s rotation (below the 30 s stale horizon — no report even
  needed); gap-4 stragglers crossed the horizon and F10 repaired them in
  1–2 report periods (1–2 tile bitmaps at 01:45:54–01:46:12, then
  silence). No lingering staleness, no keyframe.
- **Radiation audit:** all four 0x60 radiations in the session log are
  accounted for — two at stream acquisition (01:39:43, rx daemon's own
  requester at relaunch) and two for the mid-stream cold start. None
  gap-driven.

## 4. Decision

Every keyframe the old behaviour would have paid across six gap events
was avoided with zero canvas harm; the repair burden moved to F10's
existing tile-by-tile path exactly as designed. Default flipped:
`LIFETRAC_KF_ON_SEQ_GAP=0`; set `=1` to restore the old behaviour.

## 5. Raw artifacts

- `tap_base.log` (this directory) — broker tap: tile_stale + req_keyframe.
- Suppression lines quoted inline from `docker logs f11_webui`.
- Untracked working copy at repo root: `_f11_tap_base.log`.
