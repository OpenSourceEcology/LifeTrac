# 2026-05-10 Hold-Window Timing Sweep (Copilot v1.0)

## Why this run

This run targeted the easiest remaining blocker experiment:
- test whether pre/post-reset hold duration alone can produce a reproducible ingress discriminator.

The sweep used two previously informative tuples and varied only hold timing.

## Evidence folder

- `DESIGN-CONTROLLER/bench-evidence/T6_holdwindow_2026-05-10_191631/`

## Config set executed

Tuple A (cross-lane): `PC10=1, PB11=1, PC9=1`
- `64_hold_tA_short_100ms_pre_post.cfg`
- `65_hold_tA_med_500ms_pre_post.cfg`
- `66_hold_tA_long_1500ms_pre_post.cfg`

Tuple B (cross-lane): `PE11=1, PA9=0, PA10=0`
- `67_hold_tB_short_100ms_pre_post.cfg`
- `68_hold_tB_med_500ms_pre_post.cfg`
- `69_hold_tB_long_1500ms_pre_post.cfg`

Selective repeats:
- `64_hold_tA_short_100ms_pre_post.r2.*`
- `66_hold_tA_long_1500ms_pre_post.r2.*`

## Validation of forcing

OpenOCD IDR readbacks match intended tuple states in all runs, including repeats.

## Probe outcome summary

Across all hold-window cases:
- `VER_REQ` still times out waiting for `VER_URC`.
- `AT+VER?` remains no-response.

ATI behavior:
- Most cases: ATI present, zero-heavy payload (~141-145 bytes).
- One first-pass outlier: `66` (tuple A long hold) showed ATI silence.
- Repro check: `66.r2` reverted to ATI-present (~142 bytes), so the silent signature did not hold.

## Interpretation

What this sweep resolved:
- Timing-window variation (100/500/1500 ms pre/post) was tested directly on top candidate tuples.
- No reproducible timing-only ingress discriminator was found.

What remains blocked:
- Stage1 ingress gate (`VER_REQ`/`VER_URC`) still fails in every tested timing state.

## Conclusion

This hold-window sweep is complete and does not clear the go/no-go gate.
The blocker classification remains: board-state coupled and non-deterministic under current tuple/time controls, requiring either owner/net-specific controls or a different intervention dimension.

## Recommended next step

1. Continue ABX00043 owner/net extraction to remove heuristic control groups.
2. Run owner-policy perturbation while holding a fixed proven tuple.
3. Keep selective repeats on any newly observed divergent signature before accepting it as deterministic.
