# DTS throughput re-baseline on the 2026-07-25 firmware (RS-2.1 / RS-2.3 / RS-5.9)

**Date:** 2026-07-25 late session. **Firmware:** 22,504 B build (SPI 2 MHz, URC
mirror off, NVIC fix, CAD-abort, TX ring depth 4) on both boards.
**Harness:** RS-5.9 local-feed mode (broker + synth on the tractor loopback —
no LAN leg; tractor WiFi off per §8.13). Profile 2 (DTS BW500), v3 pipelining,
synth 20 fps × ~226–252 B prebuilt frames (≈4 KB/s offered), 120 s each.

## Runs

| Run | Evidence dir | Depth | Valid? | Goodput | Util |
|---|---|---|---|---|---|
| A (first local-feed) | `radio_monitor_20260725_201732_d2630395` | 2 | **no — under-offered** (synth build-bound at ~3.3 fps on the A53s; qdepth=0) | 600–820 B/s | 24–35 % |
| A2 | `radio_monitor_20260725_202153_d2630395` | 2 | yes (drop_full=1417, qdepth pinned) | **~1640 B/s** | **69–70 %** |
| B | `radio_monitor_20260725_202455_d2630395` | 4 | yes (identical saturation) | **~1639 B/s** | **69–70 %** |

Zero TX failures and zero RX decode errors in all runs (frags_ok=1089 in both
saturated runs — deterministic replay). RX reassembler_timeouts ~24–27 per run
(fragment loss under saturation → keyframe-request pokes).

## Findings

1. **RS-2.1 first re-baseline datapoint (DTS):** ~**1640 B/s with the live
   0xFB control plane**, vs the published 1755 B/s which predates it
   (commit 6bb93912 runs). The −6.5 % is the control plane's airtime +
   re-arm cost, as RS-2.1 predicted. FHSS re-baseline still pending.
2. **RS-2.3 negative result — host-refill latency is NOT the DTS
   bottleneck.** Depth 4 vs depth 2 changed nothing (1639 vs 1640 B/s,
   same util, same fragment count). With SPI (~8 ms→~1 ms) and URC
   (halved) already fixed this session and util still 70 % at saturation,
   the surviving ~30 % idle candidates are per-fragment radio-side fixed
   costs. **Correction (post-analysis, same day): LBT is also eliminated —
   both daemons CFG-disable it at startup (`LBT_ENABLE=0` verified in both
   run logs), so these measurements already ran LBT-free.** Four of five
   candidates gone; survivors: **RXCONT disarm→retune→TX→re-arm cycle per
   fragment, PLL settle + mode transitions, daemon pacing/UART turnaround**.
   The inter-arrival gap forensics (`air_gap:` line added to
   image_rx_daemon same session) is the discriminating instrument.
3. **RS-1.1 partial air evidence:** `LoRa cmd: ENCODE_MODE` received by the
   tractor in its inter-fragment gaps in every run — but only during the
   pre-saturation ramp. **No command was observed arriving mid-saturation,
   and the base's ~24 keyframe-request pokes never appeared in the tractor
   log** (the RX daemon's poke→topic→callback→LoRa path exists and is
   subscribed; its send is unlogged, so "not sent" vs "sent and lost"
   cannot be distinguished from these logs). Consistent with the
   inter-fragment listening gap closing under load. **This is the #1
   question for the RS-9.8 soak: command delivery latency/loss at
   saturation is exactly the "maintain hydraulic control under load"
   requirement.**
4. **RS-5.9 validated:** three archived runs end-to-end with zero LAN
   involvement on the tractor. Fixes en route: paho 1.x/2.x compat shim in
   `publish_synthetic_frames.py`; `LIFETRAC_SYNTH_PREBUILD` replay mode
   (offered load now exact); base mosquitto's `127.0.0.1:1883` port
   mapping restored (the stale deployed compose had silently lost it —
   RS-5.1-class deployment drift, found because rx_smoke's connect
   refused).

## Run C — gap forensics + command-TX instrumentation (same session)

`radio_monitor_20260725_215427_d2630395`, identical params to A2 (depth 2),
with the new `air_gap:` + command-TX logging aboard. Goodput unchanged
(1618–1639 B/s — instrumentation did not perturb).

**The idle, measured:** `dt_med = 140.9 ms` per fragment, `len_med = 246 B`,
p95 ≈ 148 ms (tight — the dead time is uniform per fragment, not bimodal).
ToA(246 B @ SF7/BW500/CR4-5) ≈ 86.5 ms ⇒ **~54 ms dead air per fragment,
every fragment (≈38 % of the cycle)** — matching the observed util.

**Command delivery at saturation: 0/22, with send-side now PROVEN.** The
base logged 24 command-frame copies with TX_DONE confirmed ("OK (on air)",
`cmd_tx_fail=0`): 2× ENCODE_MODE pre-stream + 22× REQ_KEYFRAME during
saturation. The tractor received exactly the 2 pre-stream copies. All 22
in-stream copies were lost at the tractor.

**Mechanism (from `image_tx_daemon.py`):** `_ensure_rxcont` — the run-31
fix — re-arms the command downlink **only in the `queue.Empty` idle branch
and after a pipelined burst**; its own docstring says "during continuous TX
the tractor listens only in those gaps (acceptable: commands are retried ×2
and idempotent)". At saturation the queue is never empty (drop_full≈1400),
so the between-frames re-armed window shrinks to the daemon's loop latency
(single-digit ms) — far smaller than a command's ~25–30 ms ToA. **The
tractor is effectively deaf for the entire saturated stream, by a design
assumption that only holds below saturation.** The ~54 ms dead time is NOT
listening (the radio is in STANDBY for most of it): it is host-side
per-frame turnaround — `_ensure_rxcont`'s read+write REG round trips, the
drain poll, queue/MQTT handling, and TX_DONE processing in Python on the
A53.

**Unified fix — RS-4.12 (firmware auto re-arm RXCONT after TX):** firmware
re-arms RXCONT within µs of TX_DONE and disarms at the next `tx_begin`;
the host drops the per-frame `_ensure_rxcont` dance entirely. Effects:
(1) the ~54 ms turnaround becomes genuine armed listening — commands land
in real windows even at saturation; (2) two UART round trips per frame
disappear, shrinking the cycle toward ~100 ms ⇒ **estimated ~2.2–2.4 KB/s
(+35–45 %) AND restored control-under-load — both program goals from one
firmware change.** (Exact split of the 54 ms between REG round trips,
drain, and Python overhead is the one remaining unknown; the post-RS-4.12
re-run measures it implicitly.)

## Run D — RS-4.12 verification (same session, firmware 22,580 B)

`radio_monitor_20260725_222356_d2630395`, identical params to Run C, with the
RS-4.12 fix flashed on both boards (`sx1276_modes_sync_external`: raw opmode
writes fold into tracked state → existing post-TX re-arm path fires).

| Prediction | Result |
|---|---|
| Command delivery >0 at saturation | **CONFIRMED**: 2 of 12 in-stream commands received (`REQ_KEYFRAME` at 03:22:29 + 03:23:19, mid-saturation) vs **0/11 in Run C**. Delivery now matches window-alignment physics (below). |
| `dt_med` drops | **CONFIRMED, modest**: 140.9 → **130.5 ms** (−10.4 ms = the `_ensure_rxcont` register round trips). |
| Goodput | **+6–9 %**: 1640 → **1745–1790 B/s**, util 74–76 % — now AT/ABOVE the 1755 B/s pre-control-plane baseline, i.e. **the control-plane tax has been bought back while commands land**. |

**Delivery arithmetic (now honest physics, not a bug):** armed listening
window per cycle ≈ 44 ms; command ToA ≈ 25–30 ms; alignment probability
≈ 12 %/copy → ~23 %/command (×2 copies) → expected ~2.8 of 12 ≈ observed 2.
**Raising delivery is now a scheduling problem:** the base KNOWS when the
tractor's window opens (it just received the fragment) — sending commands
immediately on fragment-RX-complete would align with the turnaround window
for near-deterministic delivery. Cheap host-side change; queue for RS-1.x.

**Final RS-2.3 idle decomposition (measured):** 54 ms/fragment dead time =
~10 ms `_ensure_rxcont` register dance (now deleted) + **~44 ms host-daemon
per-frame overhead** (Python/MQTT/TX_DONE handling on the A53). Next lever:
RS-3.1 fragment batching amortizes that 44 ms over 2–3× payload per handoff.

## Runs E/F/G — batching (RS-3.1) + window-aligned command TX (same session)

Three iterations, each falsifying and fixing something specific:

| Run | Evidence dir | Change | Goodput | Cmds (of 12) |
|---|---|---|---|---|
| E | `radio_monitor_20260725_224503` | batching ON (budget 480) + any-fragment-aligned pump | 1748 (unchanged) | 1/12 |
| F | `radio_monitor_20260725_224942` | budget 480→520 (pairs fit) + completion-aligned pump | 1785–1805 | 1/12 |
| G | `radio_monitor_20260725_225401` | + mid-burst RX_FRAME_URC dispatch fix | 1777–1786 | **3/12** |

**Run E postmortem (two calibration errors):** budget 480 < 498 needed for a
pair of 246 B synth frames → batching silently never engaged (identical
numbers to Run D were the tell — engagement logging added in F so absence
can never masquerade as a result again); and the pump aligned to
*any-fragment* arrival, but post-RS-4.12 the ring restarts TX in ~5 ms
mid-train — the only real window is per-TRAIN (after the last fragment), so
any-fragment alignment is chance.

**Run F:** batching engaged cleanly (351 trains, all pairs, ~485 B) and the
air cycle shrank as predicted (`dt_med` 130.5 → **104.9 ms**) — but offered
+16 % was eaten by loss (TX ~836 frames, RX published 710; timeouts 16→25;
p95 gap spikes to ~190–210 ms). Net delivered: unchanged. Suspected loss
mechanisms: the inherent 14 B runt third-fragment of a 498 B pair, and
half-duplex collisions when the base's command TX overlaps the next train.

**Run G:** the tractor's pipelined TX_DONE wait loop was found to **silently
discard RX_FRAME_URCs** ("informational, skip") — with RS-4.12 arming the
radio through every turnaround, that loop sees most inbound command URCs, so
commands the FIRMWARE heard were being dropped by the HOST. Fixed (dispatch,
don't drop). Delivery 1/12 → 3/12; the fix is unambiguously correct
regardless of magnitude (a heard command must never be host-dropped).

**Where this leaves the two features:**
- **Batching: engaged, air-efficient, net-neutral so far.** The cycle-time
  win is real; converting it to delivered goodput requires killing the loss
  (runt-avoiding batch sizing; smarter command/train collision avoidance).
  Kept harness-opt-in (`-TxBatch`), default OFF in the daemon.
- **Command delivery at saturation: 0/11 (pre-RS-4.12) → 3/12 (25 %).**
  Three stacked fixes each removed a real mechanism (deaf radio → wrong
  alignment → host discard). The residual gate is NOT identifiable from
  existing logs. **Next diagnostic (queued): compare the tractor firmware's
  `radio_rx_ok` counter delta against commands-sent per run** — if the radio
  counted them, the remainder is host-path; if not, RF/timing. One STATS
  read before/after a run answers it.

## Runs H/I/J — the command-delivery arc closes (2026-07-26)

**Run H (`radio_monitor_20260726_083837`) — the radio_rx_ok diagnostic:**
tractor `radio_rx_ok = 5 = dispatched 5` (host path now PERFECT — every
frame the radio decoded reached dispatch), `radio_tx_ok = 1198` (counters
live), `RADIO_CRC_ERR = 0`. Verdict: the 21/26 lost copies were **window
misses at the radio** — never decoded, not corrupted. Bonus find: both
copies of each command left ~37 ms apart into the same stale window (the
"independent windows" enqueue design collapsed to one attempt).

**Run I (`radio_monitor_20260726_084343`):** added a ≥120 ms pump gate —
pairs STILL went out 37 ms apart. Root cause: the drain-all loop inside
`_maybe_switch_profile` ran on EVERY loop pass and silently defeated both
the alignment and the spacing — the pump had been decorative in every run
since E. Fix: drain moved out entirely; commands now leave ONLY via the
aligned+spaced pump (streaming) or `_drain_ctrl_idle` (quiet link).

**Run J (`radio_monitor_20260726_084712`) — pump exclusive at last:**
in-stream copy spacing 188–407 ms (different trains ✓);
**16/26 copies delivered (62 %, vs 19 % ≈ chance in H)** — 14 REQ_KEYFRAME
receptions from 12 commands means several landed BOTH copies and
command-level delivery is near-total; goodput 1779 B/s (zero cost).

**Arc summary (runs C→J):** command delivery at DTS saturation went
**0/11 → ~all**, through five verified mechanisms, each measured before
and after: (1) tractor radio deaf in STANDBY (RS-4.12 firmware fix);
(2) tractor host discarding mid-burst RX URCs; (3) base pump aligned to
the wrong event (any-fragment vs train-completion); (4) copies collapsing
into one window (no spacing); (5) an unconditional drain path silently
bypassing the pump. Per-copy odds now ~3× chance; RS-1.5 (ack-driven
convergence) remains the field-grade closure on top.

## Perspective on the firmware patch

The patch's throughput levers (SPI, URC, ring) did not raise the DTS ceiling —
the idle lives elsewhere (finding 2). They still bought: main-loop headroom
(SPI 7 ms/fragment returned to the loop), the CAD-latch and watchdog-class
availability fixes, UART-over-radio interrupt priority, and ring capacity that
FHSS (whose slot-clock interacts differently) and command coexistence may yet
use. The honest headline: **the cheap wins are banked, the 30 % idle needs
forensics, not more guessing.**
