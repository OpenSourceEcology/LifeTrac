# RS-12.15 — FHSS clock authority (firmware, 2026-09-14)

**Status: fix implemented, unit-tested, flashed to both L072s, and healthy
on air with no regression. The BEHAVIORAL A/B (does it stop the
camera-motion lock losses?) is NOT demonstrated here — the reproducible
synthetic path does not exhibit the RS-12.15 break, and the camera path
that did cannot be re-flown on the old firmware. Both radios parked
(0x80). Boards now carry the RS-12.15 bench build; keep it for the next
validation session.**

## The fix

Root cause (from RS-12.12/14): every profile-1 follower lock loss began
1–2 s after a command the tractor RECEIVED. The streaming node
self-anchors its FHSS clock from its own TX but never sets
`s_grid_adopted`, so the RX health gate reads it as UNANCHORED — the tier
with no refusal authority. On a received command header it therefore
SNAPs its scheduler back to (or re-anchors its phase from) the base's
lagged FOLLOWER copy of its own grid, walking the whole shared grid a
slot off; the base's follower is then a slot out within the 2 s
LOCK_LOSS and re-acquires in 20–30 s.

Fix (commit 23ba5122, wire-compatible — no header change): a
self-anchored originator (valid clock + `!s_grid_adopted`) adopts a
remote grid ONLY when that grid genuinely LEADS its own (earlier slot
start). New pure helper `sx1276_fhss_clock_rx_leads()` (bench-pinned in
`check-fhss-clock`) does the earlier/later comparison with the same F7
half-up ToA math as `anchor_rx`. In `sx1276_rx.c` the originator's
ALIGNED echo from a lagging follower is left to run on its own
TX-maintained phase (no drag), and `consider_remote` is handed FRESH so
it REFUSES a follower snap (LOCKED_OUT) — while a genuinely leading peer
is still adopted. This is monotonic-earlier: globally convergent (two
self-anchored grids can never deadlock; they converge to the earliest)
and recovery-safe (a demoted node has an INVALID clock and still takes
the UNANCHORED adopt path). Followers and recovering nodes are unchanged.

## What was verified

- **Unit (SIL):** `check-fhss-clock` green including the new RS-12.15
  `rx_leads` golden vectors (equal / earlier / later, ToA+offset math,
  a different absolute slot, and the u32 ms-wrap). Full host `check`
  suite green.
- **Build:** bench binary cross-compiles clean under `-Werror`
  (`build/firmware_bench_diag.bin`, md5 `2ee69f9c50b7eba84edc9cdc1b49e8bb`,
  24328 B — 132 B larger than the RS-12.10 build `e8ad8424…`).
- **Flash:** both L072s, `Verify OK` + `flash_rc=0`
  (`legs/rs1215_flash_*.txt`), `REVIVE_MODE=reboot`, boards back in
  ~12–24 s.
- **Boot health (rs116):** both boards `radio_state=4` (RXCONT),
  `RS115-INSTRUMENTED-FIRMWARE=YES`, `RS12-URC-COUNTERS=YES`,
  `RS12-10-COUNTERS=YES`.

## On-air legs (profile 1, synth feed, two-opcode dual injector)

Identical setup across all three; the ONLY difference is the L072
firmware, so leg L vs O/P is a clean firmware A/B.

| leg | firmware | loss | tractor cmds rx | lock-loss gaps > 3 s | max frag gap |
|---|---|---:|---:|---:|---:|
| L (old) | RS-12.10 `e8ad8424` | 7.8 % | 49 | 0 | 0.8 s |
| O (new) | RS-12.15 `2ee69f9c` | 0.3 % | 28 | 0 | 0.6 s |
| P (new) | RS-12.15 `2ee69f9c` | 4.9 % | 20 | 0 | 0.6 s |

## Honest reading

1. **No regression.** Both new-firmware legs held the follower with the
   grid rock-solid (zero gaps over 3 s, max frag gap 0.6 s) and loss
   inside the normal profile-1 bench band. The flashed timing change is
   safe on air.
2. **Not a behavioral A/B of the fix.** The RS-12.15 lock-loss break was
   only ever seen on the camera legs (leg I 62.8 %/4 losses, leg J
   38.8 %/4 losses). ALL THREE synthetic legs here — including the
   old-firmware leg L — had ZERO lock-loss episodes, so the synthetic
   stream does not exercise the failure. With both boards now on the new
   firmware the old firmware cannot be re-flown for a camera A/B, and the
   bench camera scene is not drivable head-less.
3. **Loss is not a distinguishable improvement at this n.** New-firmware
   loss was 0.3 % and 4.9 % across two runs vs 7.8 % old in one; the
   profile-1 bench floor is interference-driven and varies this much run
   to run (see lifetrac-bench-network memory). Leg O's 0.3 % was a
   low-interference window, not proof — leg P at 4.9 % is the honest band.
4. **The clean feed-independent proof is a counter, not a loss number.**
   The fix's direct signature is the tractor recording
   `consider_remote` = REJECTED_LOCKED_OUT (refusing a follower's echo)
   where the old firmware would SNAP/re-anchor. Those counts live in
   `sx1276_rx_counters.c` but are surfaced only via the per-minute
   RFCO_SUMMARY URC, which the stats probe does not read. Capturing it
   needs a small RFCO_SUMMARY probe.

## Next validation step (recommended, needs a fresh GO)

Either (a) a **camera-motion leg** on the new firmware, driving the
bench camera scene so multi-fragment keyframe trains stress the follower
the way legs I/J did — a healthy link there is the behavioral proof; or
(b) an **RFCO_SUMMARY counter capture** showing the originator now logs
LOCKED_OUT refusals on received commands — a feed-independent signature.
Until one lands, RS-12.15 is code-complete and flashed but its on-air
benefit is asserted from the SIL model, not measured. The firmware is
safe to leave on the bench boards (healthy, no regression).
