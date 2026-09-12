# RS-12.12 — RS-12.11 under the production FHSS profile (2026-09-12, late)

**Status: leg in progress — results appended when it completes.**

## Why

The two gate legs that established the RS-12.11 fix (idle drain gated on
1.5 s of quiet, aligned pump on any train end) and RS-4.15 (measured-
rotation stale horizon) ran on profile 2 (DTS, single carrier pinned at
927.5 MHz). Production is profile 1 (FHSS, 50 channels, slot clock), where
the aligned pump meets the slot follower: a base command near a slot
boundary can make the tractor's follower skip under tx-busy (RS-4.14).
This leg asks whether the fix holds there.

## Setup

Both boards on the RS-12.10 bench build (md5 `e8ad8424…`), tooling and
helpers from main 57566ce6 (harness pushes the daemons from the same
commit). Wake-up: `wake_base.txt` / `wake_tractor.txt` — both
`STATS-OK`, `RS12-10-COUNTERS=YES`, radios in SLEEP (parked) before the
harness re-initialised them. Camera on the railroad video (restarted at
1:00), keyframe injector 15 s × 20 started on the first published frame,
hold off, RS-12.11/RS-4.15 defaults:

```
run_live_radio_monitor.ps1 -TxFeed camera -RegProfile 1 -DurationS 300
  -SynthFps 2 -SynthBudgetB 3000 -KfRequestDisable 0 -ProbeEcho 0
  -NoParkLast 0 -LogFragArrivals 1 -IdleDrainQuietS 1.5 -Archive
```

No `-ForceFrfHz` (FHSS hops the 902.75–927.25 grid, which has no
emitter-free channel, RS-11.6/11.8 — so the loss floor here is the
profile-1 floor, not the 927.5 floor of the profile-2 legs).

## Expectation (pre-registered)

Loss near the profile-1 floor rather than the 3–5 % of the control legs;
no first-of-two lock in the lost-index profile; base command sends in
the tens (aligned pump only) with `idle_drain_deferred` in the hundreds;
`TILE_STALE` well under 100. Comparators: leg E (profile 2, same config,
1.0 % / 2 timeouts / 58 sends) and the 08-01 profile-1 synth legs.

## Leg G — first attempt, CONTAMINATED (retained profile pin + keyframe storm)

Archive `radio_monitor_20260912_164512_57566ce6` (`legs/legG_*`). The leg
did not measure what it was meant to; recorded here because the failure
is instructive.

| metric | value |
|---|---:|
| frames published | **0** |
| base fragments received | 294 of ~1030 sent (tractor `frags_ok=1030`, `ok=553`) |
| base command sends | **259** — 204 `REQ_KEYFRAME` (0x60), 39 `TILE_STALE`, 16 profile (0x65) |
| base `radio_tx_ok` | **262** (transmitting almost as often as receiving) |
| fragment arrival span | 208 s of 300, **max gap 69.3 s**, 4 gaps > 1 s |
| RF when received | RSSI −62, SNR 10 (good); `phase_telemetry valid=1`, 50 hop_idx seen |

Two contaminants, both now understood:

1. **A retained `lifetrac/v25/control/radio_profile` = `{"profile": 2}`
   pin** sat on the base broker from the day's profile-2 legs.
   `clear_retained.py` did not cover that topic, so at startup the rx
   daemon accepted the pin and spent 12 s commanding the tractor 1 → 2
   (two-phase), got no ACK, and stayed on profile 1. Both ends ended on
   profile 1, but the churn delayed proof-of-life and the confirm/revert
   machinery ran. **Fixed:** `radio_profile` added to
   `clear_retained.py`; pin cleared on the broker.
2. **The self-heal keyframe requester stormed** (`KfRequestDisable 0`):
   with reassembly failing on a marginal profile-1 link it fired 204
   REQ_KEYFRAME in 300 s. Each is a base TX that deafens the base and (on
   FHSS) can skip the follower under tx-busy — a feedback loop that feeds
   the very loss it reacts to (the RS-4.14 concern, now seen at full
   strength). `radio_tx_ok` 262 is that storm.

**A note I had to retract.** From leg G plus the 2026-08-01 profile-1
synth legs (which published only ~7 % of frames) I first concluded that
profile-1 FHSS is fundamentally marginal at this bench. Leg H below
disproves that: with the pin cleared and the keyframe plane quiet, the
follower stays locked and the link publishes ~97 %. The 08-01 figure was
six weeks of follower/slot-clock work ago; profile-1 is healthy on the
current stack. The marginality is command-plane-induced, not fundamental.

## Leg H — clean, light command plane (`KfRequestDisable 1`) — **profile-1 link is healthy**

Archive `radio_monitor_20260912_165805_57566ce6` (`legs/legH_*`). Pin
cleared (`clear_retained.py` now covers `radio_profile`); `KfRequestDisable 1`
suppressed both self-heal and the injected keyframes, so the command
plane was the stale scan alone.

| metric | value |
|---|---:|
| loss | **23/1093 = 2.1 %** |
| timeouts | 2 |
| frames published | **572** (tractor offered ~587) |
| follower lock | **held all leg** — 298 s span, 2 gaps > 1 s, max 1.2 s, median 0.214 s |
| base command sends | **17** (all `TILE_STALE`; tractor received 1) |
| base `radio_tx_ok` | **17** |
| lost-index profile | idx 0 = 2, else 0 (no first-of-two lock) |
| `rx_fifo_skip` | 0 | 
| RF | RSSI −62, SNR 10 |

Profile-1 with a quiet command plane runs at the same ~2 % as profile-2
leg E, with the follower locked throughout. RS-12.11 and RS-4.15 run
clean (no crash, `tx_deaf` booked at 35 ms mean, stale scan modest). The
one gap: `KfRequestDisable 1` also blocks the *injected* keyframes (the
relay is the same gate as self-heal), so this leg did not exercise the
heavy command plane. Leg I does.

## Leg I — heavy command plane (`KfRequestDisable 0`, injector live) — **the keyframe self-heal storm breaks profile-1**

Archive `radio_monitor_20260912_170541_57566ce6` (`legs/legI_*`). Same as
leg H but keyframe requests enabled.

| metric | leg H (quiet) | **leg I (keyframes on)** |
|---|---:|---:|
| loss | 2.1 % | **62.8 %** (706/1124) |
| frames published | 572 | **226** |
| base command sends | 17 | **281** — 222 `REQ_KEYFRAME`, 59 `TILE_STALE` |
| base `radio_tx_ok` | 17 | **281** |
| `idle_drain_deferred` | — | **271** (RS-12.11 gate active) |
| follower lock | held (max gap 1.2 s) | **lost — max gap 64.4 s**, 4 gaps > 1 s |
| lost within ±150 ms of a base TX | — | 2 of 17 (0.5×) |

The injector adds 20 keyframes; the other ~200 are self-heal, because a
few early misses trigger requests, each request is a base TX, each base
TX on FHSS can skip the follower under tx-busy (RS-4.14), the follower
loses lock, more frames miss, more requests fire — a 64 s collapse. This
is the RS-4.14 loop at full strength on the production profile.

**RS-12.11 is working and is not the lever here.** `idle_drain_deferred`
271 shows the idle-drain gate deferring commands all leg; the storm still
gets out because the keyframe **retries ride the pending-command pump**
(fired on frame-completion / train-end), a different path than the idle
drain RS-12.11 gates. And the losses are not base-TX-coincident (0.5×) —
this is follower lock loss, not the per-fragment deafness RS-12.11 fixed.

## Verdict

- **RS-12.11 + RS-4.15 hold under the production FHSS profile** (leg H:
  2.1 %, follower locked, stale scan modest). Validation passed for a
  normal command plane.
- **New issue RS-12.14: the keyframe self-heal storm destroys the FHSS
  follower.** With keyframe requests enabled, profile-1 collapses to
  62.8 % (leg I) because the self-heal retries flood the pending-command
  pump — the one command path RS-12.11 does not gate — and every base TX
  disrupts the follower. The fix belongs there: rate-limit / back off the
  keyframe requester on a marginal FHSS link, or gate the pending pump
  the way RS-12.11 gated the idle drain. On profile-2 (single carrier,
  leg E) this never surfaced because the link was good and few self-heal
  requests fired. **Field profile-1 operation should run with the
  keyframe self-heal disabled or heavily rate-limited until RS-12.14.**
- Every lost fragment in both legs was transmitted (tractor TX_DONE);
  `rx_fifo_skip` 0 throughout (RS-12.10 unchanged).
- Contamination fixed on the record: retained `radio_profile` pin cleared
  and added to `clear_retained.py`.

Radios parked to LoRa SLEEP (0x80 both) after the legs. NB: a probe's own
HostLink connect auto-wakes the L072 to RXCONT (0x85), so the park's own
readback is the authoritative parked state — do not read state after
parking.
