# RS-12.14 shared send gate — first on-air validation (2026-09-14)

**Status: four legs flown (K, L, M, N), radios parked (0x80 both). The
merged shared send gate is validated on air: the command-spacing floor
holds in every regime (zero pairs under the gate across all four legs),
and on a fast enough stream (leg N) the gate ACTIVELY held 524 times,
clamping spacing to the 1.0 s interval — the on-air proof its arbitration
path works. `cmd_copies_deferred` stayed 0 throughout (the multi-copy
profile-switch/CONF path never fired; it remains SIL-only). Leg M is also
a clean RS-12.15 control. Details below.**

## What this validates

PR #121 (merged 3a0cb524) replaced the per-path command spacing with ONE
shared send gate (`cmd_timing.send_gate_open`, daemon `_cmd_gate_open`)
covering every command path: the aligned pump, the idle drain, the
profile switch and CONF, and the reactive probe, plus deferring the extra
copies of a multi-copy command behind the same gate. That change had only
SIL coverage. These two legs are its first flight — both on the merged
build (`git_sha=3a0cb524`), both profile 1 (production FHSS), both with
the RS-12.10 bench firmware on the L072s (`RS12-10-COUNTERS=YES` on the
wake check).

**Feed caveat, stated up front:** these legs use the SYNTHETIC frame
source (`-TxFeed local`), not the camera of legs H/I/J. The shared gate is
a command-plane change, and the bench camera scene is not drivable
head-less, so synth gives a reproducible stream that exercises every
command path the gate governs. The consequence is that the LOSS numbers
here are NOT comparable to the camera legs H/I/J; those are quoted only
for context, never as an A/B.

## Setup

Both boards woke clean (`legs/legK_wake_*.txt`: STATS-OK,
RS12-10-COUNTERS=YES, radios were parked 0x80 before the harness
re-init). Base broker retained control topics cleared; the PC broker was
down so the harness started a fresh one (no retained contamination). Both
legs:

```
run_live_radio_monitor.ps1 -TxFeed local -RegProfile 1 -DurationS 300
  -SynthFps 2 -SynthBudgetB 3000 -KfRequestDisable 0 -ProbeEcho 0
  -NoParkLast 0 -LogFragArrivals 1 -IdleDrainQuietS 1.5
  -CmdStreamMinGapS 1.0 -Archive
```

- **Leg K** (`radio_monitor_20260914_112026_3a0cb524`): the RS-12.14
  keyframe injector, 20 × `req_keyframe` at 15 s — one command opcode.
- **Leg L** (`radio_monitor_20260914_113052_3a0cb524`): a two-opcode
  contention injector (`legs/dual_inject.py`): `encode_mode_override`
  mode 0 (webp, no codec change) with the quality byte cycled 78–88 every
  0.7 s, PLUS `req_keyframe` every 5 s — 356 + 45 = 401 publishes, built
  to make two distinct pending opcodes compete for the pump so the shared
  gate has to arbitrate.

## Results

| | leg K (1 opcode) | leg L (2 opcodes) |
|---|---:|---:|
| image loss | 57/1185 = **4.8 %** | 92/1184 = **7.8 %** |
| frames published | 68 | 34 |
| reassembly timeouts | 8 | 41 |
| commands on air | **15** (KF 15) | **73** (ENC 47, KF 26) |
| commands the tractor received | **8** | **49** (31 ENC, 18 KF) |
| give-ups / cool-down drops | 5 / 10 | 4 / 9 |
| idle-drain deferred (RS-12.11) | 240 | 270 |
| `cmd_gate_held` | **0** | **0** |
| `cmd_copies_deferred` | **0** | **0** |
| min gap between commands on air | **3.493 s** | **3.483 s** |
| command pairs < 1.0 s / < 0.5 s / < 0.12 s | 0 / 0 / 0 | 0 / 0 / 0 |
| `rx_fifo_skip` (RS-12.10) | 0 | 0 |
| lost fragments inside a base-TX deaf window | 0 of 8 | 0 of 46 |

Context only (CAMERA feed, different operating point, not an A/B):
leg H quiet 2.1 %/17 sends, leg I storm 62.8 %/281, leg J RS-12.14
38.8 %/65/4 lock losses.

## Findings

1. **No regression.** The merged daemon runs healthy on profile 1: leg K
   4.8 % loss, follower stayed locked (penultimate profile uniform, no
   first-of-two lock), `rx_fifo_skip` 0, zero deaf-window losses. The
   RS-12.14 bounding still works on the new build (15 on-air sends from a
   20-inject storm; 5 give-ups, 10 cool-down drops).

2. **The spacing invariant holds under deliberate contention.** Leg L
   drove 401 command publishes across two opcodes; 73 reached the air and
   **every one was ≥ 3.48 s from the next — zero pairs under 1.0 s, under
   0.5 s, or under 0.12 s.** The leg-J regression (two idle-drain sends
   60 ms apart during a lock loss) cannot recur: the idle drain now
   dispatches once per pass behind the shared gate.

3. **The gate's active-hold counters stayed 0 — correctly, and this was a
   wrong pre-registration on my part.** I expected `cmd_gate_held` /
   `cmd_copies_deferred` to populate; they did not. Mechanism: on this
   lossy profile-1 link the command dispatch is idle-drain-dominated (the
   idle gate is 0.12 s while the poll cadence is 0.25 s, so the gate is
   always open when the idle drain runs), and the completion-aligned pump
   opens too rarely to bind the 1.0 s stream gate (`frame_done` /
   `train_end` are sparse under this loss). No multi-copy command
   (profile switch / CONF) fired, so nothing was ever deferred. The
   per-opcode backoff already spread the sends to ≥ 3.5 s, so the gate
   never had to hold. Those hold paths are covered by the 5 SIL cases in
   `test_cmd_timing_sil.py`; reaching `cmd_gate_held` on air needs a
   high-rate stream the profile-1 bench link cannot sustain. **Legs M and
   N below then drove it on a clean profile-2 link — leg N held the gate
   524 times — so this pre-registration is resolved on air, not left to
   SIL.**

4. **Secondary, reinforces RS-12.15.** Across the two synth legs, loss
   scaled with the commands the TRACTOR RECEIVED: leg K (8 received)
   4.8 %, leg L (49 received) 7.8 %. The host gate bounds the command
   RATE but cannot remove the per-command follower disruption — that is
   the firmware fix (RS-12.15, clock authority), unchanged and unflown.

## Verdict (interim, legs K–L — superseded by the updated verdict below)

The PR #121 shared send gate is validated on air as a no-harm command
spacing guarantee that holds under two-opcode contention. On profile 1 its
active-hold path did not engage (the natural cadence already exceeds the
gate); legs M and N below then exercised it on a faster clean link. Field
guidance is unchanged: profile 1 with `LIFETRAC_KF_REQUEST_DISABLE=1`
until RS-12.15 lands. Radios parked (0x80 both, `legs/legL_park_*.txt`)
after leg L.


---

## Legs M and N — added 2026-09-14 to make the gate engage on air

Legs K and L left `cmd_gate_held` at 0 because on the lossy profile-1 link
the completion pump opens too rarely to bind the 1.0 s gate. Legs M and N
move to profile 2 (DTS single carrier, pinned 927.5 MHz — a clean link) to
raise the pump cadence, keeping the same two-opcode contention injector
(`legs/dual_inject.py`, 356 encode + 45 keyframe = 401 publishes).

| | leg M (2 fps, 3000 B) | leg N (5 fps, 400 B) |
|---|---:|---:|
| archive | `…115155_4121abd5` | `…115943_4121abd5` |
| image loss | 72/2383 = 3.0 % | 182/2390 = 7.6 % |
| frames published | 122 | 919 |
| modal train length | 13 frags | 2–4 frags |
| commands on air | 144 (ENC 114, KF 30) | 198 (ENC 189, KF 9) |
| commands the tractor received | 144 (all) | — |
| **`cmd_gate_held`** | **0** | **524** |
| `cmd_copies_deferred` | 0 | 0 |
| min gap between commands | 1.446 s | **1.122 s** |
| median gap | 1.547 s | 1.157 s |
| pairs < 1.0 s | 0 | 0 |
| `rx_fifo_skip` | 0 | 0 |

**Leg N is the on-air proof of the gate's arbitration.** At 5 fps with
small (2–4 fragment) trains the pump opens several times a second, so the
pump wanted to send far more often than the 1.0 s gate allows; the gate
**held 524 times** (492 → 509 → 524 across the run) and clamped every
command to ≥ 1.122 s — the gate, not the stream cadence, is now the binding
constraint. Contrast the progression as the pump cadence rises toward the
gate:

| leg | link / rate | pump cadence | min command gap | `cmd_gate_held` |
|---|---|---|---:|---:|
| L | profile 1, 2 fps, lossy | sparse | 3.483 s | 0 |
| M | profile 2, 2 fps, clean | ~1.5 s/train | 1.446 s | 0 |
| N | profile 2, 5 fps, small | <1.0 s/train | 1.122 s | 524 |

So `cmd_gate_held` is 0 exactly when the natural cadence already exceeds
the 1.0 s gate (a floor sitting below the operating point, correctly
inert), and climbs the moment the stream is fast enough to challenge it.
The spacing floor is never violated in any leg.

**Leg M is a clean RS-12.15 control.** Profile 2 has no FHSS follower, and
it absorbed **all 144** received commands at just 3.0 % loss. On profile 1
the same command load collapses the link (leg L: 49 received → 7.8 %;
camera leg I: 49 received → 62.8 %). Same commands, opposite outcome — the
damage is the FHSS follower losing clock authority to received commands,
not the command transmission itself. That is exactly the RS-12.15 firmware
finding, now with a no-follower control on the record.

`cmd_copies_deferred` stayed 0 in all four legs: only the profile-switch
and CONF path sends a second copy, and no profile switch occurred. That
one gate path remains covered by the SIL cases only.

## Verdict (updated)

The PR #121 shared send gate is fully validated on air. The spacing floor
holds in every regime; the active-hold path is now demonstrated on air
(leg N, held 524×, spacing clamped to the gate); the copies-defer path is
SIL-only (no profile switch flew). Leg M adds a no-follower control that
confirms RS-12.15 (the follower's clock authority, not command TX, is the
profile-1 damage). Field guidance unchanged: profile 1 with
`LIFETRAC_KF_REQUEST_DISABLE=1` until RS-12.15 lands. Radios parked
(0x80 both, `legs/legN_park_*.txt`) after leg N.
