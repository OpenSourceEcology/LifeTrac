# RS-12.15 — FHSS clock authority (firmware, 2026-09-14)

**Status — two versions, read them apart (PR #125 review):**

- **v1 (commit 23ba5122, md5 `2ee69f9c…`) — flashed to both L072s and
  flown (legs O/P below).** Healthy, no regression. It fixed only the
  secondary ms-level drag; its on-air legs are NOT a validation of the
  fix, because the synthetic path does not exhibit the break and the
  camera path cannot be re-flown on the old firmware. **Both boards still
  run v1.**
- **v2 (commit f7d98f8b + follow-ups, the version this PR ships) — built,
  unit-tested, staged on both boards, NOT flashed and NOT flown.** It
  addresses the dominant mechanism (the demotion resetting the streaming
  node's own clock) and adds the counters that let the next leg measure
  it. See the v2 section at the end; nothing above it is evidence for v2.

Both radios parked (0x80).**

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

---

## v2 re-analysis (2026-09-14, after the PR #125 review and legs O/P)

Copilot's review flagged that `clock_valid && !s_grid_adopted` is also
the documented post-demotion recovery state, so v1's originator test
could re-lock a recovering duplex node out. Re-reading the demotion edge
with that in mind exposed the DOMINANT mechanism, which v1 did not touch:

1. **A single accepted frame moves the scan machine SCANNING → LOCKED**
   (`sx1276_rx_scan_policy.c`), and **2 s without another demotes it**
   (`SX1276_RX_SCAN_LOCK_LOSS_MS`).
2. **The demotion edge reset the FHSS clock UNCONDITIONALLY.** On the
   streaming tractor that wiped its OWN TX grid; the next TX re-anchored
   "slot k+1 starts now" (`sx1276_tx.c` first-TX branch).
3. With dense synthetic trains the next TX lands before the old boundary,
   the base still decodes a fragment in the overlap and re-syncs inside the
   slot — leg L: 49 received commands, ZERO lock losses. With sparse camera
   trains the next TX lands past the boundary, the grid renumbers, and the
   base cannot decode anything until a 20–30 s rescan — legs I/J.
4. The "1–2 s after a received command" timing IS the 2 s lock-loss timer.

That explains every observation, including why the synthetic path never
reproduced the break. The v1 mechanism (ms-level drag from adopting the
follower's echo) is real but secondary.

### v2 (commit f7d98f8b; PR #125) — built, staged, NOT flashed
- `sx1276_fhss_authority.[ch]`: originator authority is earned only by
  **sustained own-TX streaming** (≥ 8 consecutive TXs each within 1 s). A
  command sender (≥ 1 s apart under the RS-12.14 gate, or two copies 37 ms
  apart) never earns it; a node fresh from a demotion starts at zero;
  adopting a remote clears it. `check-fhss-authority` pins it, including
  the demotion → TX → RX regression the review asked for.
- `sx1276_rx.c`: the adoption gate uses that authority (v1's lead-only
  rule kept for a true originator; followers/recovery unchanged);
  **the demotion edge resets the clock only when the grid was ADOPTED** —
  a self-anchored clock survives its owner's demotion.
- STATS additive tail 168 → 208 (mirrored in `mh_wire.h`,
  `check_mh_wire_sync` PASS, `check-stats-layout` 29 cases, probe labels,
  `rs12_leg_report.py` RS-12.15 block): `fhss_dec_*` decision histogram
  (the LOCKED_OUT refusals), `clk_demotion_reset` / `clk_demotion_kept`,
  `tx_first_anchor` (phase restarts), `tx_stream_streak_max`.
- Full host `check` green, H7 host vectors green, bench binary `-Werror`
  clean: md5 `5a160e4a8c9296c7d2e49727bdfb8880`, 24860 B.
- **Round 3 (PR #125 review):** (a) the streak's chain test is STRICT
  (`< 1000 ms`) so a sender spaced exactly at the RS-12.14 stream gate's
  1.0 s never chains, and **authority decays** one gap after the last own
  TX, so a post-demotion base that bursts a few queued commands and goes
  quiet cannot sit on a stale self-anchored grid refusing the tractor;
  (b) the RX adopt / demotion rules moved out of `sx1276_rx.c` into a
  HW-free TU, `sx1276_rx_grid_policy.[ch]`, and **`check-rx-grid-policy`
  drives the full adopt → demote → duplex-TX → remote-frame sequence
  against the real `consider_remote()`**: the post-demotion single-TX node
  is UNANCHORED and gets SNAPPED (never LOCKED_OUT); the streaming
  originator refuses a lagging echo (LOCKED_OUT, clock untouched on
  ALIGNED) and adopts a leading grid; its own demotion keeps its clock;
  authority decays after 1 s of silence; exactly-1 s commands never earn it.

**Staged on both boards (`/tmp/lifetrac_p0c`, flash tooling re-pushed
LF-clean after the post-flash reboots):** v2 as `firmware_bench_diag.bin`
and the pre-RS-12.15 RS-12.10 build as
`firmware_bench_rs1210_e8ad8424.bin` (rebuilt from main, byte-identical:
md5 `e8ad842489d5acfc09f204c7807e4661`, 24196 B). **Boards still run v1
(`2ee69f9c`).** Nothing is flashed: the L072 boot path arms RXCONT, so a
flash brings the receiver up, and the operator asked for no radio activity
without GO.

### Validation plan for the next GO (reproducible instrument, firmware A/B)
The synthetic path needs SPARSE trains to expose the demotion reset (the
next TX after the 2 s demotion must land past the old slot boundary):
`-SynthFps 1.5 -SynthBudgetB 400` (2-fragment trains ~667 ms apart — far
past the 200 ms slot boundary, yet the ~627 ms gap between pairs stays
under the 1 s authority chain limit so the tractor keeps its streak) with
the plain keyframe injector, whose unacked REQ_KEYFRAME retries at 0.4 /
0.8 s make the 2–3-command bursts that LOCK the tractor.

| leg | firmware | expect |
|---|---|---|
| Q | RS-12.10 `e8ad8424` (re-flash) | lock-loss gaps > 3 s appear (reproduces I/J on synth); no counters (168-byte STATS, fly from `main` so the probe labels match) |
| R | v2 `5a160e4a` (re-flash) | tractor `clk_demotion_reset` = 0, `clk_demotion_kept` > 0, `fhss_dec_rej_locked_out` > 0, `tx_first_anchor` = 1, `tx_stream_streak_max` ≥ 8; zero lock-loss gaps; fly from `rs12-15-clock-authority` |

Each flash: `REVIVE_MODE=reboot bash /home/fio/run_flash_bench.sh
/tmp/lifetrac_p0c/<bin>` (tractor: stop `lifetrac-camera.service` and the
`tractor-camera` container first), then re-push `/tmp/lifetrac_strict`
(the reboot wipes it) and `rs116` both boards. Park (0x80 both) at the end.
