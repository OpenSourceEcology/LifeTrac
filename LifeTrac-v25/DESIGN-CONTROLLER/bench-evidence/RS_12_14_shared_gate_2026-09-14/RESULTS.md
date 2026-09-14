# RS-12.14 shared send gate — first on-air validation (2026-09-14)

**Status: two legs flown (K, L), radios parked (0x80 both). The merged
shared send gate is validated on air as a no-harm spacing guarantee that
holds under deliberate command contention. Its active-hold counters
(`cmd_gate_held`, `cmd_copies_deferred`) stayed 0 — correctly, for the
reason in §4 — so those paths remain SIL-only at this operating point.**

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
   `test_cmd_timing_sil.py`; reaching them on air needs a high-rate
   stream the profile-1 bench link cannot sustain, which would change the
   operating point — not worth chasing for a counter tick.

4. **Secondary, reinforces RS-12.15.** Across the two synth legs, loss
   scaled with the commands the TRACTOR RECEIVED: leg K (8 received)
   4.8 %, leg L (49 received) 7.8 %. The host gate bounds the command
   RATE but cannot remove the per-command follower disruption — that is
   the firmware fix (RS-12.15, clock authority), unchanged and unflown.

## Verdict

The PR #121 shared send gate is validated on air as a no-harm command
spacing guarantee that holds under two-opcode contention. Its active-hold
path is exercised only in SIL at the profile-1 bench operating point.
Field guidance is unchanged: profile 1 with `LIFETRAC_KF_REQUEST_DISABLE=1`
until RS-12.15 lands. Radios parked (0x80 both, `legs/legL_park_*.txt`)
after leg L.
