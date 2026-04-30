# DIFFERENTIAL_MIXING

**Status.** BC-21 + BC-23 LANDED (Rounds 44 + 46). Schema-driven
policy selector for BC-23 (``hydraulic.steering_priority`` enum) deferred —
the as-shipped policy is unconditionally **preserve-steering**
(proportional scale-down).

## Scope

Transformation from **stick axes** (`lhx`, `lhy`) to **logical track axes**
(`leftTrack`, `rightTrack`), including saturation handling and steering
priority.

## Base formula

```
leftTrack_raw  = lhy + lhx
rightTrack_raw = lhy - lhx
```

## Saturation handling (BC-23 — pending)

When `|lhy| + |lhx| > 1.0`, the naive `clip(±1.0)` consumes throttle budget at
the expense of steering authority. A forward-pinned stick + steering input
loses the turn under naive clipping.

Three policies (selectable via `hydraulic.steering_priority` schema leaf):

- `equal` (default, BC-23): proportional scale both inputs
  ```
  sum = |lhy| + |lhx|
  if sum > 1.0:
      scale = 1.0 / sum
      lhy_scaled = lhy * scale
      lhx_scaled = lhx * scale
  leftTrack  = lhy_scaled + lhx_scaled
  rightTrack = lhy_scaled - lhx_scaled
  ```
- `throttle_first`: clip after summing (current behaviour)
- `steering_first`: scale `lhy` only when `|lhx| + |lhy| > 1`, preserving
  full steering authority at cost of throttle

## Mix-then-ramp ordering (BC-21 — landed Round 44)

**Pre-BC-21 ordering (broken):** `stick_axis → ramp → mix → coil_select`.
Mid-stick activation stepped the corresponding track because the ramp's
`is_now=true` short-circuit fired on the stick channel that no track ramp
was watching.

**Post-BC-21 ordering (landed):** `stick_axis → mix → ramp → coil_select`.

Implemented in [tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino)
`apply_control()`. The four ramps now operate on logical axes
`g_ramp_left_track`, `g_ramp_right_track`, `g_ramp_arms`, `g_ramp_bucket`.
A `clip_to_int8()` helper saturates the post-mix sum (which can reach ±254)
back into int8 range before feeding the ramp engine.

A second BC-21 fix (incidental): coil mapping moved from `lhy`-only
(`if lhy > db: coils |= LF | RF`) to per-side
(`if leftTrack > db: coils |= LF`). Pre-BC-21 a pure spin-turn
(`lhy=0, lhx=full`) produced ZERO drive coil activation; post-BC-21 each
side energises in the correct direction.

Verification: [test_track_mix_ramp_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_track_mix_ramp_sil.py)
RT-A (mixing math), RT-B (spin-turn coil activation), RT-C (smooth-curve
turn no-step), RT-G (firmware source tripwire).

## Cross-references

- Reversal handling (cross-zero brake): [REVERSAL_HANDLING.md](REVERSAL_HANDLING.md)
- Flow setpoint computation under spin-turn: [FLOW_BUDGETING.md](FLOW_BUDGETING.md)
- Ramp shapes: [RAMP_PROFILES.md](RAMP_PROFILES.md)

## Saturation policy: preserve-steering (BC-23 — landed Round 46)

The naive ``clip_to_int8(lhy + lhx)`` saturates each track intent
independently. When the throttle+steering sum exceeds ±127 on one side,
that side is clipped while the other is untouched, destroying the
differential (steering) ratio. Concrete example: ``lhy=120, lhx=80``
produces raw intents ``(200, 40)`` which clips to ``(127, 40)`` —
steering authority drops from 80 to ``(127-40)/2 = 43``, nearly half.

BC-23 replaces the per-side clip with
``mix_tracks_preserve_steering(left_intent, right_intent)`` in
[tractor_h7.ino](../DESIGN-CONTROLLER/firmware/tractor_h7/tractor_h7.ino):

* If ``max(|left_intent|, |right_intent|) <= 127``: pass through unchanged.
* Otherwise: scale BOTH intents by ``127 / max_mag`` (int truncation
  toward zero). The differential RATIO is preserved at the cost of
  throttle authority.

For the example above, intents ``(200, 40)`` scale to ``(127, 25)`` —
differential = ``(127-25)/2 = 51``, materially closer to the operator's
commanded 80 than the pre-BC-23 43. The SP-B test class proves the
ratio-preservation invariant holds across a sweep of saturating turns.

Verification: [test_steering_priority_sil.py](../DESIGN-CONTROLLER/base_station/tests/test_steering_priority_sil.py)
SP-A (helper math: non-saturating identity, single-side saturation,
both-side saturation, sign preservation, mixed-sign saturation, just-
above-threshold scaling, exact-127 boundary, pure-throttle no-op),
SP-B (end-to-end ratio preservation through ``FourAxisArbiter``),
SP-E (firmware source tripwire pins the helper name and absence of
pre-BC-23 ``clip_to_int8(left_track_raw)`` pattern).
