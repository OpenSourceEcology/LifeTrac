# RS-12.11 / RS-4.15 gate — base command timing vs fragment arrivals (2026-09-12, evening)

**Status: legs in progress — sections are appended as they complete.**

## Firmware on the boards

Both boards: PR #117 RS-12.10 bench build (`make bench`,
`HOST_ALLOW_REG_WRITE_DIAG=1`), 24,180 B, md5
`8c112e6f8b42109aab38a69f2b0aa147`, flashed with `REVIVE_MODE=reboot`
(base 19:01 UTC, tractor 11:28 tractor-clock; both Verify OK, both
deliberate reboots, no oops, back in 24 s / 15 s;
`flash/flash_pipeline_*_rs1210.log`). Health after the flash:
`RS12-10-COUNTERS=YES` on both, all four new counters 0
(`flash/*_health_after_rs1210_flash.txt`). The tractor's production
`tractor-camera` container came back after its reboot and stole the
UART again (documented 08-08 symptom); `systemctl stop
lifetrac-camera.service` + `docker stop tractor-camera` cleared it, and
the harness stops it again at launch.

Two flash attempts on the tractor failed before that without touching
the L072: the helper scripts pushed to `/tmp/lifetrac_p0c` carried CRLF
line endings (git autocrlf had rewritten them in the working tree when
the branch was pulled), so `bash` saw `\r` in every path. LF-normalized
copies fixed it; recorded in the bench toolchain memory.

## What is being tested

Flash-session finding (`RS_12_urc_counters_flash_session_2026-09-12`):
on the camera path 21 of 25 lost fragments sat 30–200 ms after a base
command TX, and 82 of the 171 commands went out 250–300 ms after the
last fragment — the rx daemon's idle-link drain firing on its 0.25 s
poll timeout, exactly where the next 2-fragment train's first fragment
is due (train gaps 258–540 ms at 2 fps). The base's own TX makes it
deaf, and a 100 ms fragment overlapping the deaf window in any way is
not demodulated.

Host changes under test (PR #118):

- **RS-12.11** `LIFETRAC_IDLE_DRAIN_QUIET_S` (default 1.5 s): the idle
  drain waits for true quiet; in-stream commands ride the
  completion-aligned pump, which now also opens on a train's last-index
  fragment. `0` = old behaviour (control).
- **RS-4.15** motion-aware stale horizon in `web_ui.py`: horizon =
  max(20 s, 1.5 × slowest recent tile-refresh interval) capped at 120 s,
  identical report not repeated inside 10 s.
  `LIFETRAC_TILE_STALE_ROTATION_FACTOR=0` = F10 behaviour (control).

Firmware counters under test (PR #117): `tx_deaf_max_us`,
`tx_deaf_sum_us` (RX disarm → re-arm around each TX), `tx_done_to_rearm_max_us`,
`rx_fifo_skip`.

## Pre-registered expectations

| leg | config | deaf-join enrichment (lost vs base TX, ±150 ms) | loss | TILE_STALE sends |
|---|---|---:|---:|---:|
| D control | `-IdleDrainQuietS 0`, `ROTATION_FACTOR=0` | ≈ 10× (replicates leg C / 09-07) | ≈ 4–5 % | ≈ 100 / 300 s |
| E fix | defaults (1.5 s quiet, factor 1.5) | → ~1× | → ≈ 1.5 % floor | well under 100 |

Both legs: camera feed, railroad video, `kf_inject.py 15 20` started on
the first published frame, `-NoParkLast 1 -KfRequestDisable 0
-ForceFrfHz 927500000 -LogFragArrivals 1 -Archive`, base brackets with
`rs115_stats_probe.py`. Firmware expectations either leg:
`tx_done_to_rearm_max_us` in the low milliseconds; `tx_deaf_max_us` ≈
the longest command ToA plus that (the deaf window is the airtime, not
a slow re-arm); `rx_fifo_skip` ≈ 0 under the hold.

## Leg D — control (`-IdleDrainQuietS 0`, `ROTATION_FACTOR=0`) — **mechanism reproduced**

Archive `radio_monitor_20260912_142654_8961a69c` (`legs/legD_archive.txt`),
brackets `legs/legD_pre_base.txt` → `legs/legD_post_base.txt`, report
`legs/legD_report.txt`, join `legs/legD_deaf_join.txt`, injector 20/20.

| metric | leg C (flash session) | **leg D (control)** |
|---|---:|---:|
| loss | 42/868 = 4.8 % | **28/909 = 3.1 %** |
| timeouts | 25 | **15** |
| frames published | 565 | 578 |
| lost-index profile | 25/25 at idx 0 | **14/15 at idx 0** |
| base command sends | 171 | 145 |
| lost within ±150 ms after a base TX | 21/25 (10.7×) | **15/16 (11.3×)** |
| `idle_drain_deferred` | n/a | 0 (gate off, as configured) |
| `rx_urc_lost` / `rx_pretx_drained` | 6 / 0 | 1 / 0 |

Same signature as leg C and 09-07: every lost fragment was transmitted
(tractor TX_DONE), nearly every one lands 30–200 ms after a base
command, and the losses are first-of-two fragments.

## Leg E — fix (`-IdleDrainQuietS 1.5`, RS-4.15 defaults) — **gate PASSED**

Archive `radio_monitor_20260912_143437_8961a69c` (`legs/legE_archive.txt`),
brackets `legs/legE_pre_base.txt` → `legs/legE_post_base.txt`, report
`legs/legE_report.txt`, join `legs/legE_deaf_join.txt`, injector 20/20.

| metric | leg D control | **leg E fix** |
|---|---:|---:|
| loss | 28/909 = 3.1 % | **9/918 = 1.0 %** |
| timeouts | 15 | **2** |
| frames published | 578 | **609** |
| lost-index profile | 14/15 at idx 0 | **0 at idx 0** (1 at idx 1 of 3) |
| base command sends | 145 (102 TILE_STALE, 43 REQ_KEYFRAME) | **58** (13 TILE_STALE, 39 REQ_KEYFRAME, 4 ENCODE_MODE, 2 other) |
| `idle_drain_deferred` | 0 | **603** |
| lost within ±150 ms after a base TX | 15/16 | 1 of 2 placeable (n too small to score) |
| crc dumps / Δcrc_err | 9 / 10 | 5 / 5 |
| `rx_urc_lost` / `rx_pretx_drained`* | 1 / 0 | 4 / 0 |

\* the `rx_pretx_drained` slot on this build is really `rx_fifo_skip`; see
the wire note below.

Same camera scene, same injector cadence, same hold, 20 minutes apart.
The first-of-two losses are gone with the idle drain held, the command
plane shrank from 145 to 58 sends because the stale scan stopped
flooding (102 → 13 TILE_STALE per 300 s), and the delivered frame count
went up. The residual 1.0 % is at or under the RS-11.6 interference
floor (1.5 % on this bench), and none of the remaining lost fragments
is a first-of-two.

Pre-registered expectation: enrichment → ~1×, loss → floor, TILE_STALE
well under 100. Loss and TILE_STALE met outright; the enrichment
statistic has only 2 placeable losses left to score, which is the
point.

Where the 58 commands went (log time minus the last fragment arrival):
55 within 50 ms of a fragment — the completion-aligned pump, right after
a train; 2 after more than a second of quiet — the idle drain on a
genuinely idle link; 1 before the first fragment. In the control leg
82 of 145 sat 250–300 ms after the last fragment.

## Wire note — RS-12.10 counters on the build flown here

The bench build flashed for these legs (md5 `8c112e6f…`) has a
serializer slip: the four RS-12.10 fields were appended after the
`rx_pretx_drained` line, which — being last — had no index advance, so
`rx_fifo_skip` landed at offset 148 (over `rx_pretx_drained`) and the
rest shifted one slot down. The values read as `rx_pretx_drained` in
legs D/E are therefore `rx_fifo_skip` (0 in both, consistent with the
hold), the `tx_deaf_*` labels are off by one, and the last label reads
the zeroed tail. Fixed in PR #117 together with a producer-side layout
test (`check-stats-layout`) that pins every `HOST_STATS_OFFSET_*`
against `host_stats_serialize`; the `tx_deaf_*` reading is taken on the
corrected build below.

## Verdicts

- **RS-12.11 lands.** With the idle drain gated on 1.5 s of quiet and
  the pump opening on any train end, the camera-path loss under a heavy
  keyframe-request plane went 3.1 % → 1.0 %, timeouts 15 → 2, and the
  first-of-two loss signature that drove the RS-12 / 09-07 investigation
  is gone. Default `LIFETRAC_IDLE_DRAIN_QUIET_S=1.5`; `0` is the A/B
  control.
- **RS-4.15 lands.** The measured-rotation horizon cut TILE_STALE from
  102 to 13 sends per 300 s on the same moving scene with no visible
  canvas regression (609 frames delivered vs 578). Default factor 1.5,
  cap 120 s, repeat guard 10 s; `LIFETRAC_TILE_STALE_ROTATION_FACTOR=0`
  restores F10.
- **NO_PARK_LAST default: still not flipped.** Both legs ran under the
  hold; the hold was never the camera-path problem. The synth-train
  question (M1) stays with RS-12.10.

## Leg F — synth 13-frag, `-NoParkLast 0`, new host defaults — the RS-12.10 counters on the corrected build

Both boards re-flashed with the corrected bench build (md5
`e8ad842489d5acfc09f204c7807e4661`; `flash/*_rs1210b*`), `check-stats-layout`
green, `RS12-10-COUNTERS=YES` on both. Archive
`radio_monitor_20260912_144645_6d9681d9` (`legs/legF_archive.txt`),
brackets `legs/legF_pre_base.txt` → `legs/legF_post_base.txt`, report
`legs/legF_report.txt`, join `legs/legF_deaf_join.txt`.

| metric | leg A (flash session, old host) | **leg F** |
|---|---:|---:|
| loss | 76/2384 = 3.2 % | **72/2382 = 3.0 %** |
| timeouts | 57 | 65 |
| penultimate (idx 11 of 13) share | 27/78 = 35 % | **29/72 = 40 %** |
| base command sends | 113 | 83 (all `TILE_STALE`; idle drain held 33×) |
| lost within ±150 ms after a base TX | 12/70 | **2/75 (0.34×)** |
| crc dumps / Δcrc_err | 41 / 41 | 61 / 62 |
| `rx_urc_lost` / `rx_pretx_drained` | 5 / 0 | 1 / 0 |
| **`rx_fifo_skip`** | — | **0** over 2,366 received packets |
| **`tx_deaf_max_us` / `tx_deaf_sum_us` / `tx_done_to_rearm_max_us`** | — | **0 / 0 / 0** over 83 transmissions |

Two clean answers:

1. **The penultimate-of-13 lock (M1) is not FIFO coalescing at the
   base.** With the address tracker live on every serviced packet, no
   packet ever started anywhere but where the previous one ended, so no
   packet completed unserviced. The RS-12.10 prediction
   (`rx_fifo_skip ≈ penultimate losses`) fails outright: 0 vs 29. M1
   is also not base-TX-coincident (2 of 75 lost near a base TX, below
   the received baseline). It survives the RS-12.11 host change (it is a
   synth-train, no-hold phenomenon) and the hold removes it (leg B).
   Every lost fragment was transmitted (tractor TX_DONE). What is left:
   a modem-level effect of the final fragment keying up 42 ms behind the
   penultimate — the packet is on air and not demodulated — which needs
   an RF-side instrument (RSSI/SNR during the penultimate, or a third
   radio listening), not more host-side counters.

2. **The firmware never runs its own post-TX re-arm in daemon
   operation.** `tx_deaf_*` are booked only on the `s_rearm_rx` branch
   of `sx1276_tx_cleanup`, and they stayed at zero across 83
   transmissions while the tracked state read RX_CONT before and after
   the leg. So the radio is left in STANDBY after every TX_DONE and the
   rx daemon's `_ensure_rxcont` (read RegOpMode, write 0x85) is what
   re-arms it — the deaf window is the airtime plus a host round trip,
   not the airtime plus a few hundred microseconds. That is why the
   kill zone in the joins reaches 200 ms after the command's TX_DONE
   log line. A direct single-transmit probe on the base
   (`flash/tx_deaf_probe_base.txt`) pins which branch runs; the fix, if
   the tracked state is the culprit, is a firmware one-liner and
   worth the next flash.

