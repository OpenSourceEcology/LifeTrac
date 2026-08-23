# RS-3.3 — camera path first flight (2026-08-22, 3 legs)

**Verdict: PASS at the transport level across all three legs — losses
0.0 % / 0.3 % / 0.2 %, CRC closure exact every leg. Leg 3 additionally
put 11 injected REQ_KEYFRAME commands (×2 copies each, 22 receptions in
the tractor log) through the link DURING live image traffic — the first
live-traffic exercise of the command-plane interaction that kept
NO_PARK_LAST env-gated.** (⚠️ CORRECTION, PR #111 review: an earlier
revision claimed 24 requests — the injector was configured for 24 but
started 170 s after harness *launch*, so only 11 landed inside the leg;
the archive's 22 receptions and Δtx_ok=30 arithmetic confirm 11. Also
note the strict-hold caveat under "Evidence limitations" below.)
Bounding caveat: the static bench scene compresses so well that even
keyframes fit in a **single fragment** (the day's sole 2-fragment train
was leg 3's *startup batch* — 2 frames, 434 B, `seq=1`, before any
injected request — not a keyframe; review catch). Multi-fragment train
mechanics remain entirely unexercised on the camera path; that residue
now requires physical scene motion (operator's hand), not more software.

## Leg

- Archive: `radio_monitor_20260822_173616_132d23f6` (git SHA 132d23f6)
- Harness: `run_live_radio_monitor.ps1 -TxFeed camera -RegProfile 2
  -DurationS 300 -SynthFps 2 -SynthBudgetB 3000 -KfRequestDisable 1
  -ProbeEcho 0 -ForceFrfHz 927500000 -NoParkLast 1 -Archive`
- Channel: 927.5 MHz, same-day spot-check CLEAN (60 s, 949 samples,
  0 hot, max −99 dBm) ~40 min before the leg. Wake-up context: both
  boards fresh-booted this morning; both L072 health probes
  RS115-INSTRUMENTED-FIRMWARE=YES; counters near-zero at bracket time
  (power cycle cleared the base's historically-cumulative counters).
- Brackets: `rs33_pre/post_base.txt`, `rs33_pre/post_tractor.txt`
  (rs115_stats_probe, both boards, this directory).

## Numbers

| Metric | Value |
|---|---|
| Host loss | **0/587 = 0.0 %** (`timeouts=0`, `published=587`) |
| Radio symmetry | tractor Δtx_ok **609** ↔ base Δrx_ok **609** — every TX demodulated |
| CRC closure | Δcrc_err=2 vs crc_dumps=2 — exact |
| Δrx_ok − host URCs | 609 − 587 = 22, ≈ all **ack/handshake contamination** (7 base command TXs this leg; tractor acks ride ×2 copies; plus session handshake URCs). Consistent with 0 loss + 0 timeouts — NOT a firmware drop signal. |
| Identity residue | **6** (Δdio0 624 vs rx_ok+crc+tx 618) — small NEW anomaly; the identity closed exactly in all RS-12 legs. Unexplained; on the record. |
| Frame shape | ~200–240 B tile deltas → **1 fragment per frame** (static scene; encode-to-fit never approached the 3000 B budget) |

## Path features that flew for the first time

Camera capture (`/dev/video1`) → tile-delta encode-to-fit packer → carry
fix → age-escalation → liveness valve → LoRa → base publish
(`lifetrac/v25/video/tile_delta`). All ran for 300 s with zero transport
loss. On a link measured at 0.9 % the previous session, a clean leg
attributes the *path* as working; it does not stress it.

## Legs 2 and 3 (same day)

- **Leg 2** — same recipe with `-KfRequestDisable 0`:
  `radio_monitor_20260822_174805_132d23f6`. loss 2/613 = 0.3 %,
  timeouts=0, published=611, crc closure 5=5, identity residue 1.
  With a clean link nothing ever *triggers* a kf request, so the enabled
  machinery sat idle and trains stayed single-fragment. (The two lost
  fragments were sole-fragment trains: no partial ⇒ no timeout, the
  frame simply never published — consistent accounting.)
- **Leg 3** — kf-enabled + deliberate injection
  (`kf_inject.py`, this directory, run as `15 24` behind a 170 s
  post-launch delay; harness setup consumed most of that margin, so
  **11 requests landed inside the leg** — the injector was stopped at
  12/24 after harness teardown; count/cadence vs. leg duration must be
  aligned when reusing the script):
  `radio_monitor_20260822_175441_132d23f6`. loss 1/614 = 0.2 %,
  timeouts=1, crc closure 5=5, identity residue 3. Base Δtx_ok **30** =
  22 request copies + 8 session sends — the requests really flew;
  tractor log shows 22 `LoRa cmd: REQ_KEYFRAME` receptions (11 ×2-copy
  pairs on the 15 s cadence), all dispatched during active image TX.
  **Command-plane evidence for NO_PARK_LAST:** commands demonstrably
  dispatch under live traffic (bounded: trains were 1–2 fragments, so
  the specifically-mid-TRAIN arrival case is still thin; and see
  "Evidence limitations" for the strict-hold caveat). Train histogram:
  615×1-frag, **1×2-frag** — the sole 2-frag train is the **startup
  batch** (tx log: `batching: 2 frames … 434 B` immediately before
  `frame seq=1 … 2 fragments ok`, minutes before the first injected
  request), NOT a keyframe (⚠️ corrected attribution, review catch —
  an earlier revision called it "consistent with one keyframe"). Every
  keyframe the injection produced fit a single fragment: encode-to-fit
  is correct, but this run demonstrated NO multi-fragment keyframe.
- Brackets for both legs: `rs33kf_*` / `rs33l3_*` in this directory.
  **Leg-3 tractor bracket INVALID as a delta (review catch):** the
  tractor L072 reset between the pre and post snapshots (`radio_tx_ok`
  1229 → 620; the ~1.02 M `host_uart_err_lpuart` counters cleared) —
  cause of the mid-session reset unestablished. Post values are
  absolute-since-reset and *consistent with* leg-3's own traffic
  (620 ≈ 614 frags + session commands) but no tractor-side delta is
  claimed for leg 3. All leg-3 numbers above use the BASE brackets,
  which bracket cleanly. Leg-1/leg-2 tractor brackets are internally
  consistent (cumulative 1229 at leg-3 pre = legs 1+2 exactly).

## What these legs did NOT test

1. **Multi-fragment camera trains.** Static scene → every delta fit one
   fragment. No penultimate fragment exists in a 1-fragment train, so the
   RS-12 mechanics and the NoParkLast hold were idle. To exercise:
   physical motion in front of the camera during a leg, and/or a
   keyframe-enabled leg (`-KfRequestDisable 0` — keyframes are large) —
   noting kf-enable also re-opens the RS-4.14 stale-web_ui interaction.
2. **frame_id observation — RESOLVED same day:** every publish line
   reads `frame_id=0` because `TileDeltaFrame` has no `frame_id`
   attribute and the daemon falls back to 0 (`getattr(done, "frame_id",
   0) or 0`); this is universal across every leg ever flown, synth
   included — expected behavior, not a camera-path bug. The publish
   line now carries the train `seq` instead (PR #111), which is the
   useful identifier.

## Evidence limitations (PR #111 review)

- **The archives do not independently record `-NoParkLast 1`** for any
  of the three legs: `params.txt` lacked a `no_park_last` field and the
  TX daemon never logged the effective setting. The flag was passed on
  every harness command line (session transcript), but the archived
  artifacts alone cannot substantiate the strict-hold claim — treat
  "with the strict hold active" accordingly. FIXED FORWARD in this PR:
  the harness now writes `no_park_last=` into `params.txt` and the TX
  daemon logs `no_park_last=/gap_ms=/pipeline_depth=` at startup, so
  every future archive is self-substantiating. The motion leg should be
  flown with the instrumented build and will supersede this caveat.

## Anomalies for the record

- Identity residue 6 on the base (above).
- The tractor rebooted ~10 min before the leg (collateral of the base
  power cycle at the bench): /tmp re-pushed, tractor-camera container
  re-stopped, health re-probed green before the leg. Separately noted:
  a `docker run -v` against a missing host path auto-creates it
  root-owned — chown before `adb push` if the probe ran first.
- Base's first power-on this morning hung in early boot (USB descriptor
  enumerated, adbd + ethernet never came up); one power cycle cured it.
