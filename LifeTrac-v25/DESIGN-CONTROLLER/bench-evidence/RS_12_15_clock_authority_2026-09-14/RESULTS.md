# RS-12.15 — FHSS clock authority (firmware, 2026-09-14)

**Status — two versions, read them apart (PR #125 review). Current board
state: both L072s run v2 (`5a160e4a`), parked.**

- **v1 (commit 23ba5122, md5 `2ee69f9c…`) — the INTERIM result: flashed
  and flown earlier the same day (legs O/P below), then superseded.**
  Healthy, no regression, but it fixed only the secondary ms-level drag
  and its synthetic legs did not exercise the break, so they are NOT a
  validation of the fix. Nothing in the v1 sections describes the boards
  as they are now.
- **v2 (commit f7d98f8b + follow-ups, the version this PR ships) — flashed
  to both L072s and VALIDATED on air 2026-09-14.** It addresses the dominant
  mechanism (the demotion resetting the streaming node's own clock). The
  behavioral A/B on the real camera workload is on the record: the
  old-firmware break (75 s of lock-loss dead air) is GONE on v2 (0 gaps),
  same workload — see "Camera behavioral A/B" at the end, plus the leg-R
  counter proof. Boards left on v2, parked.

Both radios parked (0x80).

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

## Next validation step (recommended, needs a fresh GO) — DONE 2026-09-14, see "Camera behavioral A/B" at the end

**Superseded:** both halves below were completed the same day — the
camera-motion A/B (legs S/T) and the counter capture (leg R). This section
is kept for the reasoning; the result is at the end of this file.

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

### v2 (commit f7d98f8b + follow-ups; PR #125) — design (flashed and validated later the same day, see the two sections at the end)
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
- **Round 4 (PR #125 review, after the on-air legs):** a leading grid was
  handed UNANCHORED, which makes `consider_remote` skip the ±1 epoch-drift
  check — the unauthenticated header's only spoof/replay barrier — so a
  forged "leading" epoch+2 could have teleported a streaming originator.
  The originator now hands **STALE** for a leading grid (drift check kept)
  and FRESH for a lagging one; `check-rx-grid-policy` sequence 9 pins
  epoch+2 → REJECTED_EPOCH_DRIFT with the clock untouched, epoch+1 still
  adopted. `tools/check_mh_wire_sync.py` now enforces the ten new STATS
  offsets in the H7 mirror (57 constants). **Flown vs shipped:** legs R/S/T
  flew bench build `5a160e4a`; the PR head's bench build is `0c1bb0a9`
  (same 24860 B). The only behavioural delta is the tier handed to
  `consider_remote` when a streaming originator sees a *leading* grid — a
  case that never occurred on air (0 SNAPPED on the tractor across every
  v2 leg), so the on-air evidence stands, and the case is host-pinned. A
  short confirmation leg on `0c1bb0a9` at the next GO is recommended, not
  required. The tracked **production** image `build/firmware.bin` was
  rebuilt from the same sources (md5 `589c120323c2d5e7ef9f459d7a4ba42d`,
  24860 B, no diag flag) and is committed with this PR — before it, the
  committed production binary was still the RS-12.10 build.

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

---

## GO validation 2026-09-14 (late): legs Q, Q2, R -- the counter-level proof

Flashed the old RS-12.10 build (`e8ad8424`) to reproduce the break on the
reproducible synthetic instrument, then v2 (`5a160e4a`) to measure the fix.
All flashes Verify OK / flash_rc=0. Radios parked 0x80 both after leg R.

### Legs Q / Q2 -- the synthetic path cannot reproduce the break (old fw)

| leg | fw | rate / budget | injector | loss | tractor cmds rx | lock-loss gaps > 3 s |
|---|---|---|---|---:|---:|---:|
| Q  | e8ad8424 | 1.5 fps / 400 B | kf 15x20 | 0.6 % | 0 | 0 |
| Q2 | e8ad8424 | 2 fps / 400 B   | kf 15x20 | 6.5 % | 3 | 0 |

Neither reproduced the lock loss. The mechanism needs the tractor BOTH to
receive commands AND to pause its own TX for > 2 s so the LOCK_LOSS timer
fires and demotes it. The steady synth publisher delivers neither: the
tractor's near-continuous TX leaves almost no reverse-path window (0 and 3
commands landed), and it never pauses, so even when it does demote the next
TX re-syncs the base inside the slot. The break was only ever on the CAMERA
keyframe workload (legs I/J), whose large multi-fragment keyframe trains
create both the TX pauses and the reverse-path windows. The bench camera is
not usable this session: `/dev/video1` exists but the `/tmp/ffmpeg` capture
binary was wiped by the flash reboots, no moving content is playing, and the
physical camera aim cannot be verified head-less.

### Leg R -- v2, the fix measured directly (dual injector, 2 fps / 3000 B)

Same setup as the old-fw leg L (7.8 % / 0 gaps / 49 cmds received), so this
is a clean firmware A/B. The tractor's v2 STATS counters (pre -> post delta):

| tractor counter | delta | meaning |
|---|---:|---|
| `radio_tx_ok` | 1246 | fragments transmitted (it was the streaming originator) |
| `tx_stream_streak_max` | 1246 | held originator authority throughout |
| `fhss_dec_aligned` | 19 | received 19 base command headers, all in-slot ALIGNED |
| `fhss_dec_rej_locked_out` | 0 | none were disagreeing snap candidates, so none to refuse |
| **`clk_demotion_kept`** | **19** | **demoted 19x and KEPT its self-anchored clock every time** |
| **`clk_demotion_reset`** | **0** | **never reset its own clock (old fw would reset all 19)** |
| **`tx_first_anchor`** | **1** | **anchored the grid phase ONCE for the whole leg** |

This is the fix, feed-independent: on the streaming tractor every command it
received LOCKed its scan machine and 2 s later demoted it -- 19 times -- and
v2 KEPT the self-anchored clock across all 19, re-anchoring the grid phase
exactly once (the boot lazy anchor). On the old firmware each of those 19
demotions reset the clock and the next TX re-anchored "slot k+1 starts now"
-- 19 grid renumberings, the RS-12.12/14 lock-loss mechanism. v2 has zero.
The follower held (frag gaps: max 0.6 s, none > 3 s); no regression.

The base-bracket loss line read negative (a bracket artifact: the base
follower scan-reset once mid-leg, perturbing its radio_rx_ok pre/post
delta), so follower health here is the frag-gap metric, not that number.

### What remains

The full behavioral A/B (a real lock-loss GAP on old fw, gone on v2) still
cannot be shown on synth: the dense synth TX re-syncs the base within the
slot even when the old-fw clock resets, so the reset never surfaces as a
base gap -- it only surfaces as a gap under the sparse camera keyframe
workload. The counter proof above is the feed-independent substitute and is
unambiguous. A camera-motion leg (needs the ffmpeg binary re-pushed, moving
content, and a verified camera aim) would add the behavioral half.


---

## Camera behavioral A/B 2026-09-14 (late): legs S and T -- the break, and the fix

The operator confirmed the bench UVC camera (`/dev/video1`) is aimed at the
computer screen, so the RS-12.15 break could finally be reproduced on the
workload that caused it. The railroad reference video was played fullscreen
in Firefox (kiosk mode, autoplay muted); two frames grabbed from
`/dev/video1` 1.2 s apart differed and were ~210 KB each (a detailed, moving
image -- not a static/black screen). Same harness invocation as camera legs
I/J: `-TxFeed camera -RegProfile 1 -DurationS 300 -KfRequestDisable 0` plus
the keyframe injector (15 s x 20). The ONLY difference between S and T is the
L072 firmware -- a clean A/B.

| | leg S (old fw `e8ad8424`) | leg T (v2 `5a160e4a`) |
|---|---:|---:|
| **lock-loss gaps > 3 s** | **2 (53.2 s, 21.9 s)** | **0** |
| **total silent** | **75.2 s of 256 s** | **0 s** |
| image loss | 39.0 % | 20.7 % |
| max frag gap | 53.2 s | 0.8 s |
| published frames | 362 | 462 |
| tractor commands received | 4 | 2 |

**Leg S reproduced the break** exactly as legs I/J: the follower lost lock
twice, 53 s and 22 s, 75 s of dead air, 39 % loss. **Leg T eliminated it:**
zero gaps over 3 s, max frag gap 0.8 s, on the identical camera +
keyframe-storm workload; loss also nearly halved (39.0 -> 20.7 %).

The remaining 20.7 % on v2 is the profile-1 keyframe-storm floor (fragment
loss under the command load + the bench interference floor), NOT lock losses.
RS-12.15 targets the multi-second lock-loss episodes, and those are gone.
(Leg T's tractor happened to receive only 2 commands, so its demotion
counters stayed near 0 this run; the mechanism was measured directly in leg
R, where the tractor demoted 19x and KEPT its clock every time.
`tx_first_anchor` was 1 in both v2 legs -- the grid phase anchored once and
never reset.)

### Leg U 2026-09-15 — confirmation on the SHIPPED build (`0c1bb0a9`)

The legs above flew bench build `5a160e4a`; the PR head builds `0c1bb0a9`
(round-4 leading-grid tier + round-5 doc/tooling work). Leg U closes that
flown-vs-shipped gap: both L072s flashed to `0c1bb0a9` (Verify OK,
`flash_rc=0`), same camera workload as leg T — moving railroad scene on the
bench screen (two `/dev/video1` frames 1.2 s apart differed at ~215 KB),
profile 1, 300 s, keyframe injector 15 s × 20.

| | leg T (`5a160e4a`, flown) | **leg U (`0c1bb0a9`, shipped)** |
|---|---:|---:|
| frames published | 477 over 240 s (1.99 fps) | **537 over 268 s (2.00 fps)** |
| fragments arrived | 608 | 603 |
| **lock-loss gaps > 3 s** | **0** | **0** |
| max fragment gap | 0.8 s | 1.0 s |
| air loss (fragments sent → decoded) | — | 7.5 % (652 → 603) |

(The 7.5 % is the TX daemon's `frags_ok`=652 against 603 decoded, which is
the like-for-like fragment figure. The tractor's firmware counter
`radio_tx_ok`=678 is larger because it counts every transmission including
the command plane, so it must not be used as the fragment denominator.)

Tractor counter deltas — the fix signature, on the shipped build:

| counter | delta | reading |
|---|---:|---|
| `clk_demotion_kept` | 5 | demotions absorbed with the self-anchored clock KEPT |
| `clk_demotion_reset` | **0** | never reset its own clock |
| `tx_first_anchor` | 1 | grid phase anchored once for the whole leg |
| **`fhss_dec_rej_locked_out`** | **1** | **first LOCKED_OUT refusal ever captured ON AIR** — the originator refusing a follower echo, previously only ever seen in the host tests |
| `tx_stream_streak_max` / `radio_tx_ok` | 677 / 678 | sustained originator streaming |

**Correction — the leg report's headline was wrong, and why.** `rs12_leg_report`
first printed `loss 651/652 = 99.8% published=1` for this leg. That is an
artifact, not a result: the daemon's stats thread crashed ~1 frame in
(`_stats_worker`, `for smp in samples:` → `TypeError: 'NoneType' object is
not iterable`), so the last `stats:` line in the log — which is where the
report takes every number on that line — froze at the leg's first seconds.
The per-frame log events, which cannot go stale, show 537 frames published
and 603 fragments arrived. `base_station/` is **byte-identical** between the
leg T and leg U commits (`git diff a75089c0 aa74cde2 -- base_station/` is
empty), so this is a pre-existing latent bug that leg T happened not to hit,
not a regression from the shipped firmware. The crash is in `_stats_worker`
only; `_link_stats_worker` is a separate thread and kept publishing, so the
web UI's link stats and the auto-profile policy input were unaffected. Fix
tracked separately (RS-12.17); `rs12_leg_report` now detects the dead thread
and prints the log-derived figures instead of a false catastrophe.

**Reading:** the shipped build behaves like the flown build — zero lock
losses on the workload that broke the old firmware, counters exactly as the
host tests predict, and slightly better throughput (537 vs 477 published).
The optional confirmation is done; no further leg is required for this PR.

### Verdict
RS-12.15 v2 is validated on air, both mechanistically (leg R counters:
`clk_demotion_kept`=19 / `reset`=0) and behaviorally (legs S->T on the
camera: 75 s of lock-loss dead air -> 0). Boards left on v2 (`5a160e4a`),
parked (0x80). Recommend v2 as the production FHSS clock-authority fix.
