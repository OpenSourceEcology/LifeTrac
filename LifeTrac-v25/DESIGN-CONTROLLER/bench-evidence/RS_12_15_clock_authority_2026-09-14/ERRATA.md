# RS-12.15 clock authority — errata (2026-10-18)

Added 2026-10-18, from the committed brackets and archives only (no board
access). Nothing in [RESULTS.md](RESULTS.md) or `legs/` is rewritten;
RESULTS.md carries a dated pointer next to each passage this touches.

**Bottom line: no published conclusion in this folder changes.** Leg Q2's
tractor L072 restarted at the start of the leg (so did leg P's, which has no
tractor post bracket), so Q2's tractor counter deltas are the post values, but
no figure anywhere quotes a Q2 tractor delta other than "3 commands
received", which is right either way. Leg R's negative loss line was not
caused by a reset (§2). The 09-14 loss figures depend on the counting method
(§3).

The reset rules are those of
[`deltas.py`](../RS_12_15_reverse_delivery_2026-10-18/scripts/deltas.py). An
L072 restart between the brackets is inferred from one of two signs:

- an event counter or high-water mark went down;
- d(`host_parse_ok`) < d(`radio_tx_ok`), which cannot happen without a
  restart because every radio TX starts from a parsed host frame.

After a restart the leg's delta is the post value. `deltas.py` itself refuses
the old-firmware legs (Q, Q2 and S: the RS-12.15 counters are `ABSENT`, so it
exits 2). Its `reset_evidence()` was therefore applied to every bracket pair
directly. The same check also covered the counters that only the probe's
`ALL:` line carries (`host_irq_idle`, `host_errors`, `host_uart_*`).

The `legs/leg*_archive.txt` pointers name absolute paths in another checkout.
The archives themselves are committed in `bench-evidence/` under the same
directory names.

## 1. Leg Q2: the tractor L072 restarted at the start of the leg

| tractor counter | Q2 pre | Q2 post | post − pre (wrong) | leg value (= post) |
|---|---:|---:|---:|---:|
| `radio_tx_ok` | 886 | 1136 | 250 | **1136** |
| `radio_dio0` | 887 | 1140 | 253 | 1140 |
| `tx_fifo_rb_ok` | 886 | 1136 | 250 | 1136 |
| `host_parse_ok` | 1505 | 1679 | 174 | 1679 |
| `rx_urc_lost` | 2 | 3 | 1 | 3 |
| `tx_deaf_sum_us` | 144 594 006 | 185 807 334 | 41 213 328 | 185 807 334 |
| `radio_rx_ok` | 0 | 3 | 3 | 3 |
| `host_irq_idle` (`ALL:` line) | 142 036 | 1 679 | falls | 1 679 |
| `host_errors` (`ALL:` line) | 140 530 | 0 | falls | 0 |

([legQ2_pre_tractor.txt](legs/legQ2_pre_tractor.txt),
[legQ2_post_tractor.txt](legs/legQ2_post_tractor.txt))

- **The restart.** d(`host_parse_ok`) 174 < d(`radio_tx_ok`) 250, and
  `host_irq_idle`, `host_errors`, `host_uart_err_lpuart` and
  `host_uart_fe_lpuart` all went down. The Q2 pre bracket is leg Q's post
  bracket value for value, so the restart came after it.
- **When it happened.** The Q2 TX log
  ([tx_daemon.log](../radio_monitor_20260914_154752_6feca2c0/tx_daemon.log))
  has seven FAULT URCs (code 0x0D), spread from 34 s to 270 s of uptime. A
  FAULT URC carries the L072's `platform_now_ms()` in payload bytes 20–23,
  little-endian (`host_send_fault_urc()` in `host/host_cmd.c`). For all
  seven, log time minus uptime gives the same boot time, 12:49:15.4–15.7 on
  the tractor clock. So the L072 booted 3.4 s before the TX daemon opened the
  HostLink (12:49:18.94) and stayed up to the end of the log. Nothing
  restarted mid-leg, and the post bracket covers the whole leg.
- **Q2 and P.** The same arithmetic puts the boot before the daemon opened
  by 120–165 s in each of Q, R, S, T and U, and by 631 s in O. That is the
  reboot after the leg's flash. Leg P was the other leg here flown without a
  flash before it: it came straight after O, and its pre brackets are O's end
  state (tractor `radio_tx_ok` 1250, `radio_rx_ok` 28, and 28 is O's
  "tractor decoded cmds"). P restarted the same way. Its three FAULT URCs
  (uptimes 34 308, 64 308 and 284 071 ms, logged at 10:12:54.221,
  10:13:24.198 and 10:17:03.816 in
  [P's tx_daemon.log](../radio_monitor_20260914_130945_23ba5122/tx_daemon.log))
  put the boot at 10:12:19.75–19.91, 3.6 s before the TX daemon opened the
  HostLink (10:12:23.47). P has no tractor post bracket, so no delta was ever
  computed across that restart. The reset therefore landed on both legs
  flown without a flash before them and on none of the six first legs after
  a flash. That fits the harness's background OpenOCD SWD reset of the tractor
  L072 (`run_live_radio_monitor.ps1` step 5) and the BENCH_RUNBOOK finding
  that this reset does not land on the first leg after a flash reboot.
- **Cross-checks.** The TX log has 1112 per-fragment TX completions
  (`txdone_arrival`; the "K fragments ok" train lines the leg report counts
  sum to 1108). After the TX log's last line (seq 66, idx 3) the base decoded
  18 more fragments (seq 68–77). 1136 fits that. 250 is under a quarter of
  the fragments the tractor itself logged as sent.
- **Command plane.** The counts agree with the logs (`frag_gap_report.py`):
  base d(`radio_tx_ok`) 33 matches "base sends on air=33", and the tractor's
  3 matches "tractor decoded cmds=3".
- **Max fields.** `tx_deaf_max_us` 181 198 and `tx_done_to_rearm_max_us`
  5 237 are Q2's own maxima, because the counters restarted at launch.
- **Base.** The base L072 did not restart: no counter fell, and
  d(`host_parse_ok`) 139 ≥ d(`radio_tx_ok`) 33.

**Does any published figure or conclusion depend on it? No.**

- RESULTS.md's Q/Q2 table quotes one tractor counter: 3 commands received.
  It is the same under either rule (the pre value was 0) and matches the
  tractor log.
- The "radio deltas" in [legQ2_report.txt](legs/legQ2_report.txt) come from
  the BASE brackets (33 TX, 1054 RX), which are unaffected.
- Q2 flew the old firmware (`e8ad8424`, 168-byte STATS). All four brackets
  read `ABSENT` for every `fhss_dec_*`, `clk_demotion_*`, `tx_first_anchor`
  and `tx_stream_streak_max` counter, so no RS-12.15 counter conclusion uses
  Q2. The counter proof is leg R, confirmed by U. Both have all-zero tractor
  pre brackets and no restart.
- Leg P's restart touches nothing either. P has no tractor post bracket, and
  the one tractor figure quoted for P (20 commands received, L/O/P table)
  comes from the tractor log.
- No other document quotes 250 or any other Q2 tractor delta. Searched:
  - DC/TODO.md. Its RS-12.15 v2 entry mentions Q/Q2 only qualitatively (no
    lock-loss gap, too few reverse-path commands), and both points still
    hold.
  - CONTROL_PLANE_DESIGN.md, SINGLE_RADIO_CONTROL_PLANE_OPTIONS.md and
    VECTOR_SCENE.md.
  - `AI NOTES/`.
  - The RS-13 firmware README, which only names the leg.
  - The 2026-10-18 reverse-delivery RESULTS.

Anyone re-deriving Q2 tractor figures from the brackets must use the post
values.

## 2. The other legs

| leg | base brackets | tractor brackets | base TX: brackets / log | tractor RX: brackets / log |
|---|---|---|---:|---:|
| O | no restart | not captured (FAULT-URC uptime: no restart) | 72 / 72 | — / 28 |
| P | no restart | pre only; **restart at leg start** (FAULT-URC uptime, §1) | 73 / 71 | — / 20 |
| Q | no restart | no restart | 44 / 41 | 0 / 0 |
| Q2 | no restart | **restart at leg start** (§1) | 33 / 33 | 3 / 3 |
| R | no restart | no restart | 72 / 70 | 19 / 19 |
| S | no restart | no restart | 36 / 36 | 4 / 4 |
| T | no restart | no restart | 43 / 42 | 2 / 2 |
| U | no restart | no restart | 49 / 49 | 7 / 7 |

"Log" means `frag_gap_report.py` run on each leg's archive ("base sends on
air", "tractor decoded cmds").

- **Q2 is the only restart inside a bracket pair.** P's tractor L072 also
  restarted at leg start (§1), but P has no tractor post bracket.
- **P, Q, R and T** show base bracket counts 1–3 above the log. That is not a
  restart: a restart can only make post − pre SMALLER than the true count. The
  log's tally misses a few base transmissions. `deltas.py` would mark those
  legs' zero-delta criteria UNVERIFIED, but nothing in this folder is scored
  that way.
- **R, S, T and U.** Every tractor counter RESULTS.md quotes for these legs is
  post − pre over a zero pre bracket with no restart, so all of them stand.
  They include:
  - R: `radio_tx_ok` 1246, `clk_demotion_kept` 19, `clk_demotion_reset` 0,
    `tx_first_anchor` 1;
  - U: `clk_demotion_kept` 5, `clk_demotion_reset` 0, `tx_first_anchor` 1,
    `fhss_dec_rej_locked_out` 1;
  - S and T: 4 and 2 commands received.

### Correction (2026-10-18): leg R's negative loss line was not a reset

RESULTS.md says R's `-17/1204 = -1.4%` came from the base follower's
scan-reset once mid-leg, which perturbed its `radio_rx_ok` pre/post delta.
None of that holds:

- **The base L072 counters show no restart.** No counter fell, and
  d(`host_parse_ok`) 216 ≥ d(`radio_tx_ok`) 72.
- **The base's one `clk_demotion_reset` was not mid-leg.** A demotion needs
  2 s without an accepted frame, and R's fragment timeline never gaps more
  than 0.6 s. The one `fhss_dec_snapped` and the one `clk_demotion_reset` are
  the adoption at the start of the leg and the demotion at its end. T and U
  show the same pair.
- **The loss line never read `radio_rx_ok`.** It was the TX daemon's last
  `frags_ok` (1204) minus the RX daemon's last `rx_frames` (1221).

The real cause is that the two logs cover different windows. The TX log ends
mid-train (seq 49, idx 11 of 15). The base goes on to decode the rest of seq
49 and two more trains (seq 55 and 62), 28 fragments the TX log never saw.
Over the window both logs cover, 1212 fragments were sent and 1207 decoded
(0.4 %). The 2026-10-18 reverse-delivery RESULTS reached the same "logs do
not cover the same window" conclusion. The paragraph's own conclusion stands:
follower health comes from the frag-gap metric (max 0.6 s, none over 3 s).

## 3. Loss figures: flagged, not corrected

This is separate from any restart. The 09-14 `rs12_leg_report.py` printed the
loss figures for L, O, P, Q, Q2 and R. It took both counts from the last
periodic `stats:` lines, the stale-counter problem round 6 corrected for S, T
and U on 09-15.

Re-deriving them depends on the method, because in 7 of the 8 legs here (all
but S) the TX log ends before the RX log does:

- the base decodes 2–29 fragments after the TX log's last line;
- each tractor's own `radio_tx_ok` exceeds its logged fragments by 4–34,
  which fits.

A total-against-total comparison counts those late fragments against a
denominator that has already stopped. That understates the loss and can make
it negative.

The table gives two re-derivations:

- **Current report:** today's `rs12_leg_report.py`, which compares train
  events with `frag_arrival` lines.
- **Shared window:** the per-fragment TX completions (`txdone_arrival`),
  matched train by train to the base's `frag_arrival` lines, over the window
  both logs cover.

| leg | published | current report | shared window |
|---|---:|---:|---:|
| L (RS-12.14 folder) | 7.8 % | 6.8 % (81/1200) | 7.8 % (95/1212) |
| O | 0.3 % | none (1218 received > 1215 sent) | 2.2 % (27/1220) |
| P | 4.9 % | 2.8 % (34/1215) | 5.7 % (69/1221) |
| Q | 0.6 % | 0.9 % (8/868) | 2.5 % (22/868) |
| Q2 | 6.5 % | 5.4 % (60/1108) | 7.4 % (82/1112) |
| R | −1.4 % | none (1235 received > 1200 sent) | 0.4 % (5/1212) |
| S | 40.2 % | 40.2 % (306/762) | 40.2 % (307/763) |
| T | 20.9 % | 20.9 % (161/769) | 22.1 % (170/770) |
| U | 10.5 % | 10.5 % (71/674) | 10.8 % (73/674) |

None of this file's conclusions moves:

- no gap count changes;
- the loss comparison of O and P against L was already called
  indistinguishable at this n (Honest reading, point 3);
- Q and Q2 still show no lock loss;
- the camera A/B still drops from 40.2 % to 22.1 % over the shared window.

The figures stay as published until the leg report scores the shared window
itself.

For comparison, the 2026-10-18 legs V, W, W2 and X barely move under the
shared-window method: 0.7, 10.8, 4.0 and 6.5 % against the reported 0.6,
10.6, 3.7 and 6.5 %. On those legs the base decodes at most 2 fragments
after the TX log's last line.

### How the shared-window figure is computed

1. Group each log's fragment lines by `seq`, in order: `txdone_arrival ...
   status=0` in `tx_daemon.log`, and `frag_arrival` in `rx_daemon.log`.
2. Match each TX train to the next RX train with the same `seq`, looking at
   most 4 trains ahead. A TX train with no match is lost.
3. Count the overlap of the two trains' `idx` sets as decoded fragments.
4. Leave out any RX trains after the last matched TX train: they are outside
   the window.
