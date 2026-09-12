# RS-12.14 — bounding the keyframe self-heal storm on FHSS (2026-09-12, late)

**Status: gate leg in progress — appended when it completes.**

## Why

`RS_12_12_fhss_validation_2026-09-12` leg I: on the production FHSS
profile, enabling keyframe requests collapsed a healthy 2.1 % link to
62.8 % loss. One perpetually re-triggered pending `REQ_KEYFRAME` retried
every 0.4 s on every pump window (222 sends in 300 s), each send a base TX
that skips the FHSS follower under tx-busy, the follower lost lock for
64 s. The acks that would have converged it mostly never came back
because base→tractor delivery on profile 1 is 6–17 % (RS-12.15: the
tractor's RX follows the slot clock only while scan-LOCKED; a node that
mostly transmits rarely receives, so between trains it listens on the
scan walker's channel).

## The mitigation under test (PR #121, `cmd_timing.py`)

| rule | default | control |
|---|---|---|
| `pending_retry_gap` — exponential backoff between retries of one pending command | 0.4 s × 2ⁿ, cap 8 s (`LIFETRAC_PENDING_RETRY_BACKOFF` 2.0, `_MAX_GAP_S` 8) | backoff 1.0 (fixed 0.4 s) |
| `giveup_cooldown_active` — a given-up opcode is refused for a cool-down | 30 s (`LIFETRAC_PENDING_GIVEUP_COOLDOWN_S`) | 0 |
| `pump_min_gap` — spacing between ANY two base commands while a stream is active | 1.0 s (`LIFETRAC_CMD_STREAM_MIN_GAP_S`) | 0.12 s |

## Setup

Same as leg I: camera on the railroad video, profile 1, `KfRequestDisable 0`,
`kf_inject.py 15 20` on the first published frame, hold off, RS-12.11 and
RS-4.15 defaults, harness on branch `rs12-14-keyframe-storm` @ 0a89d8aa
(it deploys the mitigated daemon). Retained control topics cleared
(incl. `radio_profile`). Boards on the RS-12.10 bench build.

Pre-registered expectation: forward loss back near 2 %, follower locked
all leg (no gap > a few s), base sends < 100, `cmd_cooldown_drops` > 0
if the storm tried to restart. Reverse delivery stays poor (RS-12.15) —
not the gate.

## Leg J — mitigated daemon, profile 1, keyframes on — **partial: storm bounded, lock losses remain**

Archive `radio_monitor_20260912_184432_0a89d8aa` (`legs/legJ_*`), harness
on `rs12-14-keyframe-storm` @ 0a89d8aa, injector 20/20, retained topics
cleared, video verified on the railroad route (an autoplay chain was
caught and re-navigated before launch).

| metric | leg H (quiet plane) | leg I (storm) | **leg J (mitigated)** |
|---|---:|---:|---:|
| loss | 2.1 % | 62.8 % | **38.8 %** (435/1122) |
| frames published | 572 | 226 | **365** |
| base command sends | 17 | 281 | **65** (42 `REQ_KEYFRAME`, 23 `TILE_STALE`) |
| median gap between base sends | — | ~1 s | 1.73 s |
| `idle_drain_deferred` / `cmd_cooldown_drops` / give-ups | — | 271 / — / — | **362 / 7 / 4** |
| tractor received | 1 | 49 | 15 (9 `REQ_KEYFRAME`, 6 `TILE_STALE`) |
| follower lock losses (> 3 s) | 0 (max 1.2 s) | 4 (max 64 s) | **4 (7.4, 28.2, 23.8, 28.9 s)** |
| lost fragments near a base TX | — | 2/17 | 2/10 |

The mitigation did what it was built to do: sends 281 → 65, the retry
storm capped (4 give-ups, 7 cool-down drops, backoff visible in the
1.7 s median spacing), loss 62.8 → 38.8 %. It did not restore leg H,
because the follower still lost lock four times, and the loss during
those ~90 s of silence is most of the 38.8 %.

**What precedes each lock loss.** Aligning the tractor's log to the
base's (on the first fragment's TX_DONE), every lock loss starts 1–2 s
after a command the tractor *received*:

| lock loss | tractor received (s before) | base sent (s before) |
|---|---|---|
| 7.4 s at +38.8 | `TILE_STALE` −1.79 | 0x6c −1.81, 0x6c +0.06 |
| 28.2 s at +49.2 | `TILE_STALE` −1.49 | 0x6c −1.55, 0x60 +0.05 |
| 23.8 s at +97.4 | `REQ_KEYFRAME` −0.95 | 0x60 −1.18, 0x60 +0.08 |
| 28.9 s at +146.0 | `REQ_KEYFRAME` −4.37/−3.14, `TILE_STALE` −1.91 | 0x60, 0x60, 0x6c −1.92 |

Across the three profile-1 legs the pattern holds: leg H, 1 command
received, no lock loss; leg I, 49 received, four losses up to 64 s;
leg J, 15 received, four losses up to 29 s. Base *sends* that the tractor
did not hear do not do this (leg J's other 50 sends caused nothing).

**Reading (firmware, RS-12.15).** The tractor is the clock anchor and
re-anchors its FHSS clock from any accepted header — including the
base's command headers, which carry the base's *follower* copy of the
grid (lagged by the anchor latency). F6 (2026-07-30) deliberately gave
a self-anchored grid "no authority to refuse a remote one". When a base
command lands, the tractor adopts the base's lagged grid, its transmit
slots shift, and 1–2 s later the base's follower is a slot off and
demotes after `SX1276_RX_SCAN_LOCK_LOSS_MS` (2 s); re-acquisition then
takes 20–30 s. The fix is clock authority: the streaming node must not
re-anchor from a follower's headers (mark follower-originated frames as
non-anchoring, or let the anchor refuse remote grids while it streams).
That is a small firmware change with a large payoff, and it also removes
the reverse-path problem's worst symptom. It needs a flash session.

A small host follow-up seen in the data: the idle drain fired two
commands 60 ms apart (0x60 then 0x6c at +50.9 s) during a lock loss,
because it drains the legacy queue in one pass — on FHSS "quiet" can
mean "the base is deaf", so the stream gap should apply to the idle
drain too (cheap; not the driver of this leg).

## Verdict

- **RS-12.14 lands as a strict improvement** (sends −77 %, loss 62.8 →
  38.8 % on the storm case, no effect on the quiet case) but does **not**
  meet its pre-registered gate. Keep the defaults; the storm cannot be
  bounded below the follower's tolerance while every received command
  re-anchors the tractor.
- **RS-12.15 is the real fix and is firmware:** clock authority for the
  streaming node. Until it lands, profile-1 field operation should keep
  `LIFETRAC_KF_REQUEST_DISABLE=1` (which also blocks injected keyframes)
  and accept that any base command the tractor hears may cost a
  20–30 s lock loss.
- Radios parked (0x80 both) after the leg.
