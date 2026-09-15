# RS-1.4 — AutoRadioPolicy live validation of the RS-12.16 inputs (2026-09-15)

**Leg V, PR #127 head `03a1144b`, both L072s on the shipped RS-12.15 v2
bench build `0c1bb0a9`.** First on-air exercise of the auto radio-profile
selector since it was written (2026-07-25); until today it had zero air
evidence (TODO RS-1.4).

## Verdict

- **Dead-air input: VALIDATED.** The tractor's frame source was paused at
  20:51:15 (base clock, UTC). The policy pinned FHSS at **20:51:28.4 — 13 s
  after silence began** (`DEAD_AIR_S` = 10 s plus the 5 s tick); the tractor
  ACKed the switch at 20:51:28.7. The pre-RS-12.16 inputs would have seen
  nothing: `link_stats` kept arriving every 2 s and the reassembler timeout
  count stayed at 0 for the whole leg.
- **Promote-back: VALIDATED.** Frames resumed at 20:52:14; the policy pinned
  DTS at **20:53:18.4**, exactly the 60 s healthy dwell later, with the 60 s
  min-switch gap satisfied.
- **Loss-rate input: NOT exercised — and structurally blind on this
  traffic** (see finding 3). Stays SIL-only.
- Three findings below; one is fixed in this PR (RS-12.18), two are design
  items for the TODO.

## Instrument

Camera workload on DTS (profile 2), 300 s, no injector. The **frame source**
(`camera_svc` on the tractor) was `docker pause`d for 30 s at first-frame
+90 s and unpaused at +120 s. The TX daemon stayed alive, so the tractor
could still ACK the switch the policy commanded — which is what a real lock
loss looks like from the base: dead air on the fragment counter with the
control plane intact. The tractor's `rx_frames_seen` at the base went flat
at 202 from 20:51:15.

Web UI ran on the base from this branch (`bench_webui`, port 8090, MQTT
127.0.0.1), selector set to **Auto**. web_ui configures no root logger, so
its `logging.info` policy lines never reach `docker logs`; the record is an
MQTT tap (`legs/legV_tap.jsonl`, 166 lines: every control pin, both status
acks, and every `link_stats` sample) plus the rx daemon's own log in the
archive `radio_monitor_20260915_155355_03a1144b`.

## Timeline (base clock, UTC)

| time | source | event |
|---|---|---|
| 20:46:58 | policy | pinned **1** (`auto`) — 60 s after Auto was selected with no daemon running: the pre-existing stale-link rule (link down → robust profile). By design; see operator note. |
| 20:48:42 | rx daemon | started on DTS, obeyed the stale retained pin, commanded 2→1; **no tractor ACK in 12 s → stayed on 2** (tractor not up yet) |
| 20:48:57 / 20:48:59 | operator / policy | re-pinned 2 concretely, re-selected Auto → policy seeded at 2 (`auto-start`) |
| 20:49:45 | harness | first published frame |
| **20:51:15** | watcher | **frame source paused**; `rx_frames_seen` flat at 202 from here |
| **20:51:28.4** | **policy** | **pinned 1 (`auto`) — dead air detected, 13 s after silence** |
| 20:51:28.5 / 28.7 | rx daemon / tractor | commanded 2→1; tractor ACK on the old grid; both switched locally (`link_stats.radio_profile` = 1 at 20:51:29) |
| 20:52:00 | watcher | frame source unpaused (tractor now transmitting on FHSS) |
| 20:52:14.4 | rx daemon | **"no frames on new profile within 45 s — reverting to 2"** |
| ~20:52:14 | tractor | "no CONF within 45 s — reverting to 2" (its own 45 s from the switch) — **both sides reverted within ~1 s of each other** |
| 20:52:14 | base | fragments resume (on DTS) — a 61 s silence at the base in total |
| **20:53:18.4** | **policy** | **pinned 2 (`auto`) — promote after the 60 s healthy dwell.** No-op for the daemon (already on 2). |
| 20:53:55 | harness | leg ends; daemons stop |
| 20:54:18.4 | policy | pinned 1 (`auto`) — stale-link rule again, daemons down. Disarmed to concrete 2 afterwards. |

Leg numbers (harness view): 508 fragments sent, 373 received, loss 26.6 %,
`crc_dumps` 14; two silences at the base — startup (47 s, the daemon's
failed obedience to the stale pin plus camera start) and the instrument
(61 s = 30 s pause + 14 s on FHSS unheard + revert). `frag_gap_report`:
n_frag 385, max gap 61.2 s.

## Findings

**1. The 45 s revert window is marginal against FHSS acquisition — by
construction of the situation Auto acts in.** Phase A of the two-phase
switch worked (ACK in 0.3 s). Phase B failed because my instrument left only
~14 s of FHSS traffic (20:52:00 → 20:52:14) inside the 45 s proof-of-life
window, and the base's follower needs 20–30 s of frames to lock. That is an
artifact of the pause overlapping the window — but it is also exactly the
shape of a real event: Auto degrades to FHSS *because* frames stopped, so
the switch's "frames within 45 s" proof is racing the very outage that
triggered it. Both sides reverted within ~1 s of each other (the tractor's
no-CONF timer and the base's no-frames timer both started at the switch), so
the link converged cleanly and there was no long profile mismatch. Design
item: the proof-of-life window should start from the first frame *attempt*
on the new profile, or be lengthened when the switch was auto-triggered by
dead air. **TODO RS-12.19.**

**2. The policy never learns the daemon reverted (FIXED HERE, RS-12.18).**
From 20:52:15 (daemon back on 2) to 20:53:18 (policy promoted to 2) the
policy believed it was on FHSS while the link was on DTS. Had the link gone
bad in that window, Auto would have done nothing — it thought it had already
degraded. It resolved by luck (healthy link → no-op promote). The daemon
publishes its actual profile in every `link_stats` sample; the worker now
feeds it to `AutoRadioPolicy.observe_active()`, which re-syncs after the
disagreement has persisted `RESYNC_AFTER_S` (20 s — longer than the 12 s
phase-A handshake, so a normal switch in flight is not mistaken for a
revert) and treats a revert as a switch for hysteresis. SIL-pinned.

**3. The loss-rate input is blind to whole-frame loss.** The reassembler
counted **0 missing of 385 expected** while the harness measured 26.6 %
fragment loss. Every fragment the tractor sent on FHSS while the base was
not locked vanished *whole*; a single-fragment frame that never arrives
never opens a partial, so nothing is booked missing. `frags_missing` only
sees loss *inside* frames the base partially received — which on this
camera scene at DTS (one fragment per frame) is structurally zero. The
dead-air input caught the outage; the loss-rate input could not. Document:
it measures unrecovered intra-frame loss on multi-fragment traffic; a
sequence-gap detector (missing `frag_seq` values) is the complementary
signal for whole-frame loss. **TODO RS-12.20.**

**Operator note.** Selecting Auto while no daemon is running pins FHSS
within 60 s (stale link → robust profile). That is the existing rule
working as documented, but it means the link starts on FHSS when the
daemons come up. Arm Auto after the daemons are up, or expect an FHSS start.

## Not established

The loss-rate threshold (0.25) is still a bench-calibrated placeholder;
nothing in this leg exercised it. The 10 s / 60 s dead-air constants worked
as designed on one event — the detection latency (13 s) and the dwell are
measured, not tuned.

Radios parked (`0x80` both, `legs/legV_park_*.txt`). Auto disarmed to
concrete 2 on the bench web UI.
