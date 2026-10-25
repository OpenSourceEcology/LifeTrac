"""rs12_leg_report.py — one-command leg analysis for the RS-12 campaign.

Reports, for a radio-monitor archive: raw loss over the whole logs, the
fragment loss over the window both logs cover (``txdone_arrival`` trains
matched to ``frag_arrival`` trains, RS-12.15 09-14 ERRATA §3), the
per-index loss histogram with the penultimate share, the corrupt-capture
index distribution (read out of the crc_dump payload headers), and — when
given bracketing stats-probe snapshots — the radio-counter deltas and the
firmware-drop count (drx_ok - host URCs). With ``--capture`` (the base-side
``vector_dry_run.py capture`` JSONL of an RS-13 leg) it also prints the
frame loss read from the TileDeltaFrame sequence numbers, which no
periodic counter line can make stale.

2026-10-24 (the FHSS round's leg 6, where the base never locked): an
``acquisition:`` line gives the base's first image reception after its
profile-1 activation against the FHSS cold-start scan budget (4 attempts x
30 s = 120 s), and the line under it the acquisition verdict on positive
evidence only (review), read from the activation up to the base's next
REG_PROFILE activation (a profile switch stops the scan):
``!! ACQUISITION FAILED`` (also at the end of the report's first line, which
leg_post.sh's 8-line console excerpt always shows) on a cold or early final
RX_SCAN_FAILED, or on an instrument known to be on that shows no frame
within the budget (the always-on stats counter while its RX worker lives, or
per-fragment logging shown on by a frag_arrival line) when nothing
contradicts that silence (a lock shown, a FAULT_URC-logging daemon with no
RX_SCAN_FAILED line, a base bracket whose radio_rx_ok outruns the host);
``confirmed acquisition: ACQUIRED`` on a lock shown (a base-bracket
LOCKED -> SCANNING demotion with --pre/--post, or a share of the tractor's
fragments only a grid follower decodes); ``NOT DETERMINED`` otherwise, since
a frame received is not a lock. The base's RX_SCAN_FAILED FAULT lines follow
when its log holds them. On the RS-12.21 image (scan option (b), VER
capability bit 7: --scan-image, default auto) a final RX_SCAN_FAILED is a
30 s report and the scan goes on, so a lock shown after it is a late
acquisition, and silence counts over the whole window, not the budget
(2026-10-24 review). The capture's head (the frames before the first one
heard, RS-13.1 A7) is placed on the tractor's TX log (``txdone_arrival``) by
the frame's transport seq, read off rx_daemon.log's published line at its
capture time (the TileDeltaFrame seq is the camera's own counter), instead of
being assumed to sit in the first u8 seq cycle, and reads AMBIGUOUS when it
cannot be placed or the two counters differ, whatever its seq; the ``tractor
TX log`` line withholds its share, and counts trains, on a leg whose TX log
shows a batched train (a ``batching:`` line); a capture with no clean +1 seq
step times its gaps by the leg's synth_fps, so an outage of more than 128
frames is counted, not booked as a counter-restart jump.

Usage:
    py -3 rs12_leg_report.py <archive_dir> [--pre stats_pre.txt --post stats_post.txt]
                             [--capture legs/leg<X>_base.jsonl]
"""

from __future__ import annotations

import argparse
import collections
import pathlib
import re
import statistics
import sys
from datetime import datetime, timezone

# TileDeltaFrame header byte 1 (LORA_PROTOCOL.md § TileDeltaFrame) is a u8.
# The fragment seq both daemons log (txdone_arrival / frag_arrival) wraps at
# 256 too.
SEQ_MOD = 256

# Per-fragment events, both behind the harness's -LogFragArrivals 1: the TX
# daemon logs one txdone_arrival per TX_DONE URC (status 0 = on air), the RX
# daemon one frag_arrival per fragment the base decoded.
TXDONE_RE = re.compile(r"txdone_arrival: seq=(\d+) idx=(\d+) status=(\d+)")
# total = the header's fragment count (image_rx_daemon: raw[3] + 1). Parity
# frames (magic 0xFC) are not logged; a copies=2 key frame logs each copy
# with the same idx.
FRAG_RE = re.compile(r"frag_arrival: seq=(\d+) idx=(\d+)(?: total=(\d+))?")
LOG_TS_RE = re.compile(r"^\ufeff?(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d),(\d{3}) ")
# How many RX trains ahead a TX train may find its match (ERRATA §3 step 2).
SHARED_LOOKAHEAD = 4
# A train never spans this long, so two runs of one seq further apart than
# this are separate trains (the u8 seq came round again).
TRAIN_SPLIT_S = 10.0
# How far apart the matched trains' RX-minus-TX log times may spread before
# one of them is taken to be matched a u8 seq wrap off. The boards' clocks
# differ by days (the tractor has no time sync) but tick together: over the
# 55 committed archives with per-fragment events the spread is at most
# 2.1 s (a 15-fragment train whose first fragments the base lost), and one
# wrap is 256 frames, 25.6 s even at 10 frames/s.
ALIGN_TOL_S = 10.0

# 2026-10-24: placing a frame the base captured on the tractor's TX log (the
# A7 head). With the params.txt clock offsets (board minus PC, read after the
# leg, error at most rtt/2, about 0.03 s) the frame's txdone_arrival lands
# within a few tens of ms of it on the PC clock (19-33 ms on the 10-24 legs);
# the next frame of the same u8 seq is a wrap away, 25.6 s even at 10 fps.
PLACE_TOL_S = 2.0
# 2026-10-24 review (PR #166): a captured frame is placed by its TRANSPORT
# seq, never by its TileDeltaFrame seq. The TX daemon numbers each MQTT
# message it takes (image_tx_daemon.py _on_message, from 1) and logs that
# frame.seq in txdone_arrival; the fragment header carries it, and the base
# logs it in frag_arrival and "published frame_id=N seq=S" (train_seq_of).
# The TileDeltaFrame seq is the camera's own counter per encoder path
# (camera_service.py _VECTOR_SEQ / accum.seq): on the committed 2d switch
# legs it restarts at 1 under VECTOR while the transport seq runs on (63,
# 64, ...). The capture runs on the base, so its ts and the RX log share the
# base clock: on the 22 committed captures each frame's published line is
# 1-8 ms from it and the next one at least 0.18 s away.
PUB_RE = re.compile(r"published frame_id=\d+ seq=(-?\d+)")
PUB_MATCH_S = 0.05
# Without the offsets (every archive before 10-24): how much later the RX
# daemon's first log line came than the TX daemon's, PC time, for the
# candidate to be the frame. The harness starts the TX daemon first: 0.1 to
# 32.4 s later on the 61 committed archives with per-fragment events. Two
# candidates fit when a wrap is shorter than the window (above ~5.6 fps):
# the head is then ambiguous.
START_SKEW_S = (-5.0, 40.0)

# 2026-10-24 (leg 6): the FHSS cold-start scan budget. A scan attempt dwells
# SX1276_RX_SCAN_DWELL_MS per channel (500 ms; 2 s on the RS-12.21 image,
# option d2, a 50-channel sweep then taking 100 s) and fails after
# SX1276_RX_SCAN_REDESIGN_MS (30000 ms, sx1276_rx_scan_policy.h); the first
# attempt and SX1276_RX_SCAN_MAX_RETRIES (3, sx1276_rx_scan_fail.h) retries
# make 4, and the fourth FAIL is final. Up to the RS-12.21 image it entered
# scan FAILED, which is absorbing (sx1276_rx_scan_policy.c): only a reset or
# a profile re-activation leaves it, and a follower there never hops, so a
# base that decodes no frame within 4 x 30 s of its profile-1 activation ran
# out of scan. The RS-12.21 image (scan option (b), FHSS_SCAN_FAILED_OPTIONS.md;
# VER capability bit 7, HOST_CAP_RADIO_PARK) restarts the scan of a node
# without originator authority at once -- the base always -- and sends a
# final RX_SCAN_FAILED every 30 s until a LOCK: there a final is a report,
# not a stop (scan_image() tells the images apart).
SCAN_ATTEMPT_S = 30.0
SCAN_MAX_RETRIES = 3
SCAN_BUDGET_S = SCAN_ATTEMPT_S * (SCAN_MAX_RETRIES + 1)
# RS-12.21 (b): the restarted attempts' finals come SCAN_ATTEMPT_S apart (each
# attempt gets a fresh 30 s budget); a log that runs this much longer past
# the last one without another shows a LOCK within that attempt.
FINAL_REPEAT_SLACK_S = 5.0
# The capability bitmap image_rx_daemon.py logs with its VER warm-up line
# (2026-10-24 review), and its bit for the RS-12.21 image (host_types.h).
VER_CAP_RE = re.compile(r"L072 VER warm-up ok \(attempt \d+\) cap=0x([0-9A-Fa-f]{8})")
HOST_CAP_RADIO_PARK = 0x80
SCAN_IMAGES = ("auto", "restarts", "absorbing")
# "CFG_SET_REQ(REG_PROFILE=1) OK: 14000100", printed (no timestamp) when the
# daemon's profile activation is acknowledged; the next timestamped line (the
# LBT_ENABLE line on every committed archive since 07-25) dates it.
ACTIVATION_RE = re.compile(r"REG_PROFILE=(\d+)\) OK")
# A frame the base decoded: frag_arrival (-LogFragArrivals 1) or a publish.
DECODED_RE = re.compile(r"frag_arrival: seq=|published frame_id=")
# FAULT_URC lines as the daemons log them: the tractor's legacy "FAULT during
# pipelined TX: code=0x0D sub=0x20 raw=..." (format_fault_payload), and from
# 2026-10-24 one "FAULT_URC: code=0x0D sub=0xA3 name=RX_SCAN_FAILED ..." line
# per URC from both daemons (format_fault_urc_line, method_g_stage1_probe.py).
# The same URC can leave a legacy line beside its FAULT_URC line, so when a
# log holds FAULT_URC lines (with name=) only they are counted. Code 0x0D is
# HOST_FAULT_CODE_RX_SCAN_FAILED; it was also TX_FIFO_RB_MISMATCH on images
# before the 0x0D split (e083ae46 and older; 0x0F since 2026-10-24):
# the daemon's name= decides it when it names one of them, else (AMBIGUOUS,
# or a legacy line) scan_fault_sub() reads the sub-byte by the scan layout.
FAULT_RE = re.compile(r"\bFAULT\w*\b.*?\bcode=0x([0-9A-Fa-f]{2}) sub=0x([0-9A-Fa-f]{2})")
FAULT_URC_RE = re.compile(r"\bFAULT_URC: code=0x[0-9A-Fa-f]{2} sub=0x[0-9A-Fa-f]{2} "
                          r"name=(\w+)")
FAULT_CODE_SCAN = 0x0D
# sx1276_rx_scan_fail.h, the RX_SCAN_FAILED sub-byte (MSB to LSB)
SCAN_SUB_FINAL, SCAN_SUB_WARM, SCAN_SUB_HW_SUSPECT, SCAN_SUB_CRC_SEEN = 0x80, 0x40, 0x20, 0x10
SCAN_SUB_ATTEMPT = 0x0F
PROFILE_NAMES = {"0": "bench-only fixed 915 MHz", "1": "FHSS", "2": "DTS"}

# 2026-10-24 review (PR #166): the acquisition verdict rests on positive
# evidence only. A frame received is not a lock: sx1276_rx.c (the
# lora_pkt_hdr_unpack block of sx1276_rx_service) still delivers a frame whose
# FHSS header sx1276_fhss_consider_remote REJECTED, and feeds the scan only an
# accepted one (leg 6's base delivered 2 such frames, d(fhss_dec_rej_epoch_drift)
# 2). Silence is evidence only from an instrument known to be on.
ACQUIRED, FAILED, UNDETERMINED = "ACQUIRED", "FAILED", "NOT DETERMINED"
# SX1276_RX_SCAN_LOCK_LOSS_MS (sx1276_rx_scan_policy.h): a LOCKED scan drops
# back to SCANNING only once SX1276_RX_SCAN_LOCK_LOSS_MS has passed since its
# channel anchor, and a LOCK resets the attempt count. Up to the RS-12.21
# image a LOCK kept that anchor (sx1276_rx.c scan_drive: LOCK left the anchors
# unchanged), which the activation's BEGIN_SCAN set (sx1276_rx_scan_reset ->
# BOOT -> BEGIN_SCAN) or a later ADVANCE moved on; from it (RS-12.21 L) a LOCK
# anchors it at the locking frame. Either way a lock and its loss take at
# least this long from the activation, and a final RX_SCAN_FAILED less than
# SCAN_BUDGET_S + this after it ends 4 attempts with no lock. For this rule the activation
# is dated by the last timestamped line before it, never later than it, so
# the fault's time from it is never read short (the timestamped lines either
# side of it are 0.34-0.42 s apart on the 45 committed profile-1 archives).
SCAN_LOCK_LOSS_S = 2.0
# image_rx_daemon.py's periodic "stats: rx_frames=N" line: INFO, every 10 s,
# whatever -LogFragArrivals says. rx_frames counts every RX_FRAME_URC that is
# not a command frame (image fragments and parity), logged or not.
STATS_RX_RE = re.compile(r"\bstats: rx_frames=(\d+)")
# frag_arrival lines: image_rx_daemon.py reads LIFETRAC_LOG_FRAG_ARRIVALS once
# at start-up, so one such line shows per-fragment logging was on all run.
FRAG_LINE_RE = re.compile(r"\bfrag_arrival: seq=")
# The FHSS hop set the base activated: method_h_stage2_tx_probe_v2.py prints
# "FHSS mask: WIDE (50 channels)" or "FHSS mask: single channel N" before the
# REG_PROFILE line of the same configuration.
MASK_RE = re.compile(r"FHSS mask: (?:WIDE \((\d+) channels\)|single channel)")
# 2026-10-24 review (PR #166): Python's threading prints these untimestamped
# when a daemon thread dies. image-rx-worker takes every RX URC off the wire,
# counts it (rx_frames) and logs frag_arrival; image-rx-stats prints the
# stats line. A dead worker leaves the stats thread printing a frozen
# counter, so after it neither is an instrument that is on.
WORKER_CRASH = "Exception in thread image-rx-worker"
STATS_CRASH = "Exception in thread image-rx-stats"
# A base bracket's d(radio_rx_ok) above the host's count by more than
# rx_urc_lost + this shows a host that did not see what the L072 delivered
# (a deaf RX path, or command frames, which rx_frames does not count): the
# FIRMWARE DROP on the committed bracketed legs reads 0-6, the log-window
# artefact (10-24 leg 6: 5 - 5 = 0).
HOST_DROP_SLACK = 10
# A base that does not follow the hop grid is on the tractor's channel about
# one slot in N (N channels; 2 % on the wide mask); only a LOCKED scan follows
# the grid (sx1276_rx.c, rx_follow_now). Decoding this share of the tractor's
# fragments on air, on a hop set of at least GRID_MIN_CHANNELS, shows a lock
# (10-24: legs 1-5 77-97 %, leg 6 0.8 %).
GRID_SHARE = 0.25
GRID_MIN_CHANNELS = 10


def parse_stats(path: pathlib.Path) -> dict[str, int]:
    out = {}
    for m in re.finditer(r"(\w+)=(\d+)", path.read_text(encoding="utf-8",
                                                        errors="replace")):
        out[m.group(1)] = int(m.group(2))
    return out


def _last_int(pattern: str, text: str, default: int = 0) -> int:
    found = re.findall(pattern, text)
    return int(found[-1]) if found else default


def _log_time(line: str) -> float | None:
    """Seconds (epoch, board clock) of a daemon log line, or None."""
    m = LOG_TS_RE.match(line)
    if not m:
        return None
    return (datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S")
            .replace(tzinfo=timezone.utc).timestamp()
            + int(m.group(2)) / 1000.0)


def fragment_events(text: str, rx: bool) -> list[tuple]:
    """``(t, seq mod 256, idx, total)`` per per-fragment event, in log order:
    ``frag_arrival`` lines when ``rx``, else ``txdone_arrival ... status=0``
    lines (on air). ``t`` is None on a line without a timestamp, ``total``
    None on the TX side and on old RX lines without it."""
    out = []
    for line in text.splitlines():
        m = (FRAG_RE if rx else TXDONE_RE).search(line)
        if not m:
            continue
        if not rx and int(m.group(3)) != 0:
            continue
        total = int(m.group(3)) if rx and m.group(3) else None
        out.append((_log_time(line), int(m.group(1)) % SEQ_MOD,
                    int(m.group(2)), total))
    return out


def fragment_trains(events) -> list[dict]:
    """Trains: consecutive events with the same seq (mod 256), in log order,
    split where two of them are more than TRAIN_SPLIT_S apart (the seq came
    round again). Each train: ``seq``, ``idxs`` (set), ``idx_list`` (as
    logged, duplicates kept), ``totals`` (set), ``t`` (first event's time),
    ``t_end`` (last event's time: the base publishes a frame after its last
    fragment, 2026-10-24)."""
    trains: list[dict] = []
    last_t = None
    for t, seq, idx, total in events:
        same = (trains and trains[-1]["seq"] == seq
                and not (t is not None and last_t is not None
                         and t - last_t > TRAIN_SPLIT_S))
        if not same:
            trains.append({"seq": seq, "idxs": set(), "idx_list": [],
                           "totals": set(), "t": t, "t_end": t})
        tr = trains[-1]
        tr["idxs"].add(idx)
        tr["idx_list"].append(idx)
        if total is not None:
            tr["totals"].add(total)
        if t is not None:
            last_t = tr["t_end"] = t
    return trains


def _misaligned(pairs, tx_trains) -> str | None:
    """Why the matched trains are not one alignment, or None. The boards'
    clocks differ by days but tick together, so every correct match has
    the same RX-minus-TX log time to within a train's span; a train
    matched one u8 seq wrap off (after an outage of about 256 frames, or
    of a little less with the lookahead) is a wrap's time off the rest."""
    offs = [(rx["t"] - tx["t"], rx["seq"]) for tx, rx in pairs
            if tx["t"] is not None and rx["t"] is not None]
    if len(offs) < len(pairs):
        span = sum((b["seq"] - a["seq"]) % SEQ_MOD
                   for a, b in zip(tx_trains, tx_trains[1:]))
        if span >= SEQ_MOD // 2:
            return (f"matched trains without log timestamps over {span} "
                    f"seqs: the alignment cannot be checked")
        return None
    if not offs:
        return None
    lo, hi = min(offs), max(offs)
    if hi[0] - lo[0] > ALIGN_TOL_S:
        return (f"the matched trains' RX-minus-TX log times spread "
                f"{hi[0] - lo[0]:.0f} s (seq {lo[1]} vs seq {hi[1]}): a train "
                f"was matched a u8 seq wrap off after an outage, so the "
                f"trains cannot be aligned")
    return None


def _redundant(tx_train: dict, rx_train: dict) -> str | None:
    """Why a matched train cannot be scored idx by idx, or None. The TX
    daemon logs each transmission's place in its list; the RX daemon logs
    the header's fragment index. With key-frame copies (one idx received
    twice) or parity (a TX idx at or past the header's fragment count, the
    parity frame itself not logged) the two do not name the same thing."""
    rx_list = rx_train["idx_list"]
    if len(rx_list) != len(set(rx_list)):
        return (f"seq {rx_train['seq']}: the base logged idx "
                f"{sorted(rx_list)} (a fragment twice: key-frame copies, or a "
                f"fragment re-sent after a TX_DONE timeout)")
    if len(rx_train["totals"]) > 1:
        return (f"seq {rx_train['seq']}: fragment counts "
                f"{sorted(rx_train['totals'])} in one train")
    if rx_train["totals"]:
        total = next(iter(rx_train["totals"]))
        over = sorted(i for i in tx_train["idxs"] if i >= total)
        if over:
            return (f"seq {rx_train['seq']}: TX idx {over} at or past the "
                    f"header's {total} fragment(s) (copies or parity)")
    return None


def shared_window_loss(tx: str, rx: str,
                       lookahead: int = SHARED_LOOKAHEAD) -> dict | None:
    """Fragment loss over the window both logs cover (RS-12.15 09-14
    ERRATA.md §3, 2026-10-18), or None when either log lacks the events.

    Up to the 10-19 legs the harness read the TX log before the RX log, and
    both before it stopped the daemons, so the RX log runs on past the TX
    log's last line: the base goes on decoding fragments the TX log never
    saw (the TX daemon's TX_DONE lines can also stop short of the base's
    last fragment for other reasons). A whole-log comparison
    counts those against a denominator that has already stopped, which
    understates the loss and can make it negative (09-14 leg R: -1.4 %
    whole-log, 0.4 % here). Steps, as in ERRATA §3:

    1. group each log's fragment events by seq, in order: ``txdone_arrival
       ... status=0`` in tx_daemon.log, ``frag_arrival`` in rx_daemon.log;
    2. match each TX train to the next RX train with the same seq, looking
       at most ``lookahead`` trains ahead; a TX train with no match is lost;
    3. count the overlap of the two trains' idx sets as decoded;
    4. leave out the RX trains after the last matched one (outside the
       window).

    Fails closed: ``withheld`` carries the reason, and the report gives no
    figure, when a train was matched a u8 seq wrap off (its RX-minus-TX log
    time differs from the other matches' by more than ALIGN_TOL_S: an
    outage of about 256 frames) or when a matched train carries key-frame
    copies or parity (the two logs number its fragments differently).
    """
    tx_trains = fragment_trains(fragment_events(tx, rx=False))
    rx_trains = fragment_trains(fragment_events(rx, rx=True))
    if not tx_trains or not rx_trains:
        return None
    nxt, last, decoded, unheard, extra = 0, -1, 0, 0, 0
    withheld = None
    pairs = []
    for tr in tx_trains:
        for k in range(nxt, min(nxt + lookahead, len(rx_trains))):
            if rx_trains[k]["seq"] == tr["seq"]:
                withheld = withheld or _redundant(tr, rx_trains[k])
                decoded += len(tr["idxs"] & rx_trains[k]["idxs"])
                extra += len(rx_trains[k]["idxs"] - tr["idxs"])
                pairs.append((tr, rx_trains[k]))
                last, nxt = k, k + 1
                break
        else:
            unheard += 1
    withheld = _misaligned(pairs, tx_trains) or withheld
    sent = sum(len(tr["idxs"]) for tr in tx_trains)
    return {"sent": sent, "decoded": decoded, "lost": sent - decoded,
            "trains": len(tx_trains), "unheard": unheard,
            "late": sum(len(tr["idxs"]) for tr in rx_trains[last + 1:]),
            "extra": extra, "withheld": withheld}


def format_shared_loss(res: dict | None) -> list[str]:
    """Report lines for :func:`shared_window_loss`."""
    if res is None:
        return ["shared-window loss: n/a (needs txdone_arrival lines in "
                "tx_daemon.log and frag_arrival lines in rx_daemon.log: "
                "harness -LogFragArrivals 1)"]
    if res.get("withheld"):
        return [f"shared-window loss: n/a, withheld ({res['withheld']})"]
    sent, lost = res["sent"], res["lost"]
    out = [f"shared-window loss {lost}/{sent} = {100 * lost / sent:.1f}%   "
           f"[txdone_arrival vs frag_arrival, train by train over the window "
           f"both logs cover; {res['unheard']} of {res['trains']} trains "
           f"not heard]"]
    left = []
    if res["late"]:
        left.append(f"{res['late']} fragment(s) the base decoded after the "
                    f"TX log's last train (outside the window)")
    if res["extra"]:
        left.append(f"{res['extra']} decoded in a matched train with no "
                    f"TX_DONE status=0 line for their idx")
    if left:
        out.append("  left out: " + "; ".join(left))
    return out


def seq_gap_loss(frames, period_s: float | None = None) -> dict:
    """Frame loss from TileDeltaFrame sequence numbers (RS-13.1 A14).

    ``frames``: one ``(ts, codec, seq)`` per captured payload, in arrival
    order; ``codec`` and ``seq`` are None for a payload that did not parse.
    ``period_s``: the leg's nominal frame period (1 / params synth_fps),
    used only when the capture has no clean +1 step to time the cadence by
    (2026-10-24 leg 6: 5 frames, no two consecutive; its 102 -> 238 step,
    68 s, read as a counter-restart jump and the loss as 91/96, not 226/231).

    The header seq is a u8 and each encoder path numbers its own frames
    (camera_service: ``_VECTOR_SEQ`` for codec 6, the tile accumulator's
    ``accum.seq`` for the tile codecs), so a codec switch restarts or resumes
    a different counter. The stream is therefore cut into runs of one codec
    and nothing is counted across a cut — a frame lost right at a switch is
    not seen. Inside a run a step of d (mod 256) means d - 1 frames never
    arrived; when the arrival times say more than 256 frames elapsed (an
    outage longer than the seq space) the whole wraps are added back from
    the nominal period (median arrival gap over clean +1 steps). A step of
    more than half the seq space that the elapsed time cannot explain is a
    counter restart or a reorder: reported as a jump, not counted as loss.
    d = 0 is a duplicate.
    """
    frames = list(frames)
    parsed = [(float(t), c, s) for t, c, s in frames
              if c is not None and s is not None]
    steps = [b[0] - a[0] for a, b in zip(parsed, parsed[1:])
             if a[1] == b[1] and (b[2] - a[2]) % SEQ_MOD == 1]
    period = statistics.median(steps) if steps else None
    period_src = "steps" if steps else None
    if period is None and period_s:
        period, period_src = float(period_s), "nominal"
    runs: list[dict] = []
    for ts, codec, seq in parsed:
        run = runs[-1] if runs else None
        if run is None or run["codec"] != codec:
            runs.append({"codec": codec, "first_seq": seq, "last_seq": seq,
                         "last_ts": ts, "received": 1, "lost": 0, "gaps": 0,
                         "longest": 0, "duplicates": 0, "jumps": 0})
            continue
        d = (seq - run["last_seq"]) % SEQ_MOD
        if d == 0:
            run["duplicates"] += 1
            continue
        if period:
            elapsed = (ts - run["last_ts"]) / period
            jump = d > SEQ_MOD // 2 and elapsed < d / 2
            wraps = max(0, round((elapsed - d) / SEQ_MOD))
        else:   # no clean step to time the cadence by: no wrap correction
            jump, wraps = d > SEQ_MOD // 2, 0
        if jump:
            run["jumps"] += 1
        else:
            missing = d - 1 + SEQ_MOD * wraps
            if missing:
                run["lost"] += missing
                run["gaps"] += 1
                run["longest"] = max(run["longest"], missing)
        run["last_seq"], run["last_ts"] = seq, ts
        run["received"] += 1
    received = sum(r["received"] for r in runs)
    lost = sum(r["lost"] for r in runs)
    return {"runs": runs, "received": received, "lost": lost,
            "expected": received + lost, "period_s": period,
            "period_src": period_src,
            "gaps": sum(r["gaps"] for r in runs),
            "longest": max((r["longest"] for r in runs), default=0),
            "duplicates": sum(r["duplicates"] for r in runs),
            "jumps": sum(r["jumps"] for r in runs),
            "unparseable": len(frames) - len(parsed),
            "first_seq": runs[0]["first_seq"] if runs else None}


def format_seq_loss(res: dict, label: str, names: dict | None = None,
                    place: dict | None = None) -> list[str]:
    """Report lines for :func:`seq_gap_loss`. ``place``: the capture's first
    and last frames on the tractor's TX log (:func:`place_capture`); without
    it, or when the first frame could not be placed, a capture that starts
    after seq 1 gets no head figure (2026-10-24: the head was counted as if
    the first frame heard sat in the camera's first u8 seq cycle, and leg 6's
    seq 8 was the tractor's frame #264)."""
    names = names or {}
    if not res["runs"]:
        return [f"capture seq gaps ({label}): no parseable frames "
                f"({res['unparseable']} unparseable)"]
    exp, lost = res["expected"], res["lost"]
    n_runs = len(res["runs"])
    period = f"; period {res['period_s']:.2f} s" if res["period_s"] else ""
    if period and res.get("period_src") == "nominal":
        period += " (the leg's synth_fps: no clean +1 step to time it by)"
    out = [f"capture seq gaps ({label}): lost {lost}/{exp} = "
           f"{100 * lost / exp:.1f}%  (received {res['received']}, "
           f"{res['gaps']} gap{'' if res['gaps'] == 1 else 's'}, "
           f"longest {res['longest']}; "
           f"{n_runs} codec run{'s' if n_runs > 1 else ''}{period})"]
    seq0 = res["first_seq"]
    first = (place or {}).get("first") or {}
    placed = "frame" in first
    # 2026-10-24 review: the TX log counts the camera's frames before it only
    # when its TileDeltaFrame seq is its transport seq (one counter); else, or
    # unplaced, the head is AMBIGUOUS, however small its seq (seq 1 can be
    # frame #257, seq 0 is frame #256 at the earliest)
    same = placed and first.get("rx_seq", seq0) % SEQ_MOD == seq0 % SEQ_MOD
    if placed:
        # #N is a train on a leg whose TX log shows a batched train (place["batched"])
        on_air = (f"in the tractor's train #{first['frame']}" if place.get("batched")
                  else f"the tractor's frame #{first['frame']}")
    least = ((f"{(seq0 - 1) % SEQ_MOD} frame(s) numbered before it if seq {seq0} is in the "
              f"camera's first u8 cycle") if seq0 % SEQ_MOD else
             f"at least {SEQ_MOD - 1} frame(s) numbered before it (seq 0 is frame "
             f"#{SEQ_MOD} at the earliest)") + f", {SEQ_MOD} more per wrap"
    head = seq0 - 1 + (SEQ_MOD * first["wraps"] if same else 0)
    if placed and not same:
        out.append(
            f"  first frame heard at seq {seq0}: the head is AMBIGUOUS — it is {on_air} on "
            f"air by its transport seq {first['rx_seq']} (rx_daemon.log line "
            f"{first['rx_line']}), but its TileDeltaFrame seq is not that seq: the camera's "
            f"counter and the TX daemon's differ here (a codec switch, a camera restart or a "
            f"frame the TX daemon never took), so the camera's frames before it cannot be "
            f"counted from the TX log ({least}); no figure counting from seq 1 (RS-13.1 A7)")
    elif not placed:
        why = first.get("why") or (place or {}).get("why") or (
            "no tractor TX log was given to place it on")
        out.append(
            f"  first frame heard at seq {seq0}: the head is AMBIGUOUS — {least}, and it "
            f"cannot be placed on the tractor's TX log ({why}); no figure counting from seq 1 "
            f"(RS-13.1 A7)")
    elif head > 0 and first["wraps"] == 0:
        # word for word as before 2026-10-24: the TX log puts the first frame
        # in the first u8 cycle (every committed leg but 10-24 leg 6)
        out.append(
            f"  first frame heard at seq {seq0}: the {head} "
            f"frame(s) numbered before it are not in the figure above (the "
            f"camera numbers from seq 1 at start-up — an acquisition delay "
            f"or a late capture, RS-13.1 A7); counting from seq 1: "
            f"{lost + head}/{exp + head} = "
            f"{100 * (lost + head) / (exp + head):.1f}%")
    elif head > 0:
        w = first["wraps"]
        out.append(
            f"  first frame heard at seq {seq0} is {on_air} on air ({w} u8 seq "
            f"wrap{'s' if w > 1 else ''} before it, by its TX log): the {head} frame(s) "
            f"numbered before it are not in the figure above (the camera numbers from seq 1 "
            f"at start-up — an acquisition delay or a late capture, RS-13.1 "
            f"A7); counting from seq 1: {lost + head}/{exp + head} = "
            f"{100 * (lost + head) / (exp + head):.1f}%")
    out += format_tx_place(res, place)
    if n_runs > 1:
        out.append(
            "  codec runs: " + " | ".join(
                f"{names.get(r['codec'], r['codec'])} seq "
                f"{r['first_seq']}..{r['last_seq']} ({r['received']} rx, "
                f"{r['lost']} lost)" for r in res["runs"])
            + " — each codec path numbers its own frames, so nothing is "
              "counted across a switch")
    odd = {k: res[k] for k in ("duplicates", "jumps", "unparseable")
           if res[k]}
    if odd:
        out.append("  !! " + ", ".join(f"{k} {v}" for k, v in odd.items())
                   + " (jumps = a counter restart or a reorder; none of "
                     "these is counted as loss)")
    return out


def format_tx_place(res: dict, place: dict | None) -> list[str]:
    """The ``tractor TX log`` line under the capture seq-gap line
    (2026-10-24): the capture's first and last frames placed on the
    tractor's frames on air, and the share of those frames the capture never
    received, the head and the tail included. Nothing when the first frame
    was not placed (the head line says why). On a leg whose TX log shows a
    batched train (``place["batched"]``) it counts trains and gives no share."""
    first = (place or {}).get("first") or {}
    if "frame" not in first:
        return []
    last = place.get("last") or {}
    sent = place["sent"]
    if place["how"] == "offsets":
        dts = [abs(p["dt"]) for p in (first, last) if "dt" in p]
        basis = (f"placed by the params.txt clock offsets, within "
                 f"{max(dts):.2f} s")
    else:
        basis = (f"placed by the daemons' start times: the RX log starts "
                 f"{first['skew']:.1f} s after the TX log on the PC clock")
    batched = place.get("batched")
    if batched:
        # 2026-10-24 review: res["received"] counts TileDeltaFrames, the TX log
        # trains, and a batched train carries several frames
        line = (f"  tractor TX log: {sent} train(s) on air (txdone_arrival); the "
                f"capture's first frame is in its train #{first['frame']}")
    else:
        line = (f"  tractor TX log: {sent} frame(s) on air (txdone_arrival); the "
                f"capture's first frame is its #{first['frame']}")
    if "frame" not in last:
        return [line + f" ({basis}); its last could not be placed "
                       f"({last.get('why', 'no frame')})"]
    line += (f", its last in train #{last['frame']} ({basis})" if batched
             else f", its last #{last['frame']} ({basis})")
    rcv = res["received"]
    if batched:
        line += f": no share of them that never reached the capture ({batched})"
    elif rcv <= sent and first["frame"] <= last["frame"]:
        line += (f": {sent - rcv}/{sent} = {100 * (sent - rcv) / sent:.1f}% "
                 f"of them never reached the capture ({first['frame'] - 1} "
                 f"before its first, {sent - last['frame']} after its last)")
        if place.get("batch_note"):
            line += f"; {place['batch_note']}"
    return [line]


def params_dict(text: str) -> dict[str, str]:
    """params.txt (key=value lines, a BOM on the first) -> {key: value}."""
    out = {}
    for line in text.splitlines():
        key, sep, val = line.lstrip("\ufeff").partition("=")
        if sep:
            out[key.strip()] = val.strip()
    return out


def clock_offsets(params: dict) -> tuple[float, float] | None:
    """``(tractor, base)`` board-minus-PC clock offsets from params.txt
    (2026-10-24 harness on), or None."""
    try:
        return (float(params["tractor_clock_offset_s"]),
                float(params["base_clock_offset_s"]))
    except (KeyError, ValueError):
        return None


def nominal_period(params: dict) -> float | None:
    """1 / params synth_fps (the leg's frame rate), or None."""
    try:
        fps = float(params.get("synth_fps", ""))
    except ValueError:
        return None
    return 1.0 / fps if fps > 0 else None


def log_start(text: str) -> float | None:
    """Board time of a daemon log's first timestamped line, or None."""
    return next((t for t in map(_log_time, text.splitlines()) if t is not None),
                None)


def tx_frames(tx: str) -> list[dict]:
    """The tractor's frames on air, in log order: the ``txdone_arrival``
    trains (:func:`fragment_trains`), each with ``n`` (1-based) and ``u``,
    its seq unwrapped along the log (the first train's seq taken as in the
    camera's first u8 cycle)."""
    trains = fragment_trains(fragment_events(tx, rx=False))
    u = None
    for n, tr in enumerate(trains, 1):
        u = tr["seq"] if u is None else u + (tr["seq"] - u) % SEQ_MOD
        tr["n"], tr["u"] = n, u
    return trains


def rx_publications(rx: str | None) -> list[tuple[float, int, int]]:
    """``(board time, transport seq, line)`` per ``published frame_id=N
    seq=S`` line of rx_daemon.log (a seq of -1, none, left out)."""
    out = []
    for n, line in enumerate((rx or "").splitlines(), 1):
        m = PUB_RE.search(line)
        t = _log_time(line) if m else None
        if t is not None and int(m.group(1)) >= 0:
            out.append((t, int(m.group(1)), n))
    return out


def transport_seq(ts: float, pubs: list[tuple[float, int, int]]) -> dict:
    """The transport seq of a frame the base captured at ``ts`` (base board
    time): ``seq`` and ``line`` of the one published seq within PUB_MATCH_S of
    it (rx_publications; frames batched in one train share it), or ``why``."""
    near = [p for p in pubs if abs(p[0] - ts) <= PUB_MATCH_S]
    seqs = sorted({p[1] for p in near})
    if not near:
        return {"why": f"rx_daemon.log has no published line within {PUB_MATCH_S:g} s of its "
                       f"capture time, so its transport seq (the one txdone_arrival logs) is "
                       f"unknown, and its TileDeltaFrame seq is the camera's own counter"}
    if len(seqs) > 1:
        return {"why": f"rx_daemon.log published transport seqs {seqs} within "
                       f"{PUB_MATCH_S:g} s of its capture time"}
    best = min(near, key=lambda p: abs(p[0] - ts))
    return {"seq": best[1], "line": best[2]}


def place_frame(ts: float, seq: int, frames: list[dict],
                offsets: tuple[float, float] | None = None,
                starts: tuple[float, float] | None = None,
                basis: str = "the params.txt clock offsets") -> dict:
    """Which of the tractor's frames on air (:func:`tx_frames`) a frame the
    base captured at ``ts`` (base board time) with u8 transport ``seq``
    (:func:`transport_seq`) is.

    With ``offsets`` (``(tractor, base)`` board minus PC, as in params.txt):
    the frame of that seq whose last txdone_arrival (its train's ``t_end``:
    the base publishes after the last fragment) lands nearest it on the PC
    clock, within PLACE_TOL_S (``basis`` names the offsets in the reason).
    Else with ``starts`` (rx_daemon.log and tx_daemon.log first-line board
    times): the one frame of that seq for which the RX daemon started
    START_SKEW_S after the TX daemon on the PC clock; none or several is no
    placement. Returns ``frame`` (its n), ``wraps`` (u8 seq wraps before
    it), ``dt`` or ``skew``, or ``why`` when not placed."""
    cands = [f for f in frames
             if f["seq"] == seq % SEQ_MOD and f["t_end"] is not None]
    if not cands:
        return {"why": f"no txdone_arrival with seq {seq} in tx_daemon.log"}

    def hit(f, **kw):
        return dict(kw, frame=f["n"], wraps=(f["u"] - seq) // SEQ_MOD)
    if offsets is not None:
        tr_off, base_off = offsets
        dt, f = min((((ts - base_off) - (f["t_end"] - tr_off), f)
                     for f in cands), key=lambda c: abs(c[0]))
        if abs(dt) > PLACE_TOL_S:
            return {"why": f"the nearest txdone_arrival with seq {seq} is "
                           f"{dt:+.1f} s from it on {basis} (more than "
                           f"{PLACE_TOL_S:g} s)"}
        return hit(f, dt=dt)
    if starts is None:
        return {"why": "no clock offsets in params.txt and no daemon start "
                       "times to align the two logs by"}
    rx0, tx0 = starts
    lo, hi = START_SKEW_S
    fit = [(s, f) for s, f in (((rx0 - tx0) - (ts - f["t_end"]), f)
                               for f in cands) if lo <= s <= hi]
    if len(fit) == 1:
        return hit(fit[0][1], skew=fit[0][0])
    return {"why": (f"no params.txt clock offsets, and the daemons' start "
                    f"times fit {len(fit)} of the tractor's frames with seq "
                    f"{seq}" + (f" (#{', #'.join(str(f['n']) for _, f in fit)})"
                                if fit else ""))}


# image_tx_daemon.py (_batch_more) logs this INFO line on its first batched
# train and every 25th after, since batching came in (346aa414, 2026-07-26).
# It never batches a key frame or a VECTOR frame (RS-13 guard), so a default
# -TxBatch 1 VECTOR leg logs none. A TX log with txdone_arrival lines (INFO)
# and no such line carried one frame per train.
BATCH_LINE_RE = re.compile(r"\bbatching: \d+ frames in \d+ trains")


def _tx_batch_param(params: dict) -> str:
    val = params.get("tx_batch")
    return f"params.txt tx_batch={val}" if val is not None else "params.txt has no tx_batch"


def batching(params: dict, tx: str | None = None) -> str | None:
    """Why a train on this leg may carry several frames, or None. With
    LIFETRAC_TX_BATCH on, image_tx_daemon.py packs frames into one train under
    the first one's seq, so the TX log counts trains while a capture counts
    frames (2026-10-24 review). None when params.txt says tx_batch=0, or when
    ``tx`` (tx_daemon.log) is given and holds no ``batching:`` line
    (BATCH_LINE_RE); without ``tx`` any other tx_batch (or none) may batch."""
    if params.get("tx_batch") == "0":
        return None
    unit = ": a train can carry several frames, so the TX log's trains and the capture's " \
           "frames are not one unit"
    if tx is None:
        return _tx_batch_param(params) + unit
    n = next((i for i, ln in enumerate(tx.splitlines(), 1) if BATCH_LINE_RE.search(ln)), None)
    if n is None:
        return None
    return (f"tx_daemon.log line {n} logs a batched train ({_tx_batch_param(params)})" + unit)


def place_capture(frames, tx: str, rx: str | None, params: dict) -> dict | None:
    """The capture's first and last parsed frames on the tractor's TX log
    (:func:`place_frame`), each by its transport seq, read off rx_daemon.log's
    published line at its capture time (:func:`transport_seq`): ``sent`` (its
    frames on air), ``how`` ("offsets" or "starts"), ``first`` and ``last``
    (with ``rx_seq`` and ``rx_line`` once read, and ``tdf_seq``, its
    TileDeltaFrame seq), ``batched`` (:func:`batching` on the TX log) and
    ``batch_note`` (params.txt does not say tx_batch=0, but no train batched);
    ``why`` when the TX log has no txdone_arrival line. None for a capture
    with no parsed frame.
    A frame whose transport seq cannot be read is not placed (its ``why``
    says so). Without the params.txt offsets the last frame is placed on the
    first one's own match (the two clocks tick together), so the start-time
    window is not needed twice; a capture that runs past the TX log's last
    frame (a harness that read the logs while the daemons ran) says so."""
    parsed = [(float(t), s) for t, c, s in frames
              if c is not None and s is not None]
    if not parsed:
        return None
    trains = tx_frames(tx)
    if not trains:
        return {"sent": 0, "why": "tx_daemon.log has no txdone_arrival line "
                                  "(harness -LogFragArrivals 1)"}
    offsets = clock_offsets(params)
    starts = None
    if offsets is None and rx is not None:
        rx0, tx0 = log_start(rx), log_start(tx)
        if rx0 is not None and tx0 is not None:
            starts = (rx0, tx0)
    pubs = rx_publications(rx)

    def place(ts, tdf_seq, *args, **kw):
        tseq = transport_seq(ts, pubs)
        if "seq" not in tseq:
            return {"why": tseq["why"], "tdf_seq": tdf_seq}
        got = place_frame(ts, tseq["seq"], trains, *args, **kw)
        return dict(got, tdf_seq=tdf_seq, rx_seq=tseq["seq"], rx_line=tseq["line"])
    first = place(*parsed[0], offsets, starts)
    batched = batching(params, tx)
    out = {"sent": len(trains), "how": "offsets" if offsets else "starts",
           "first": first, "last": first if len(parsed) == 1 else None,
           "batched": batched,
           "batch_note": None if batched or params.get("tx_batch") == "0" else (
               f"{_tx_batch_param(params)}, but tx_daemon.log logs no 'batching:' line: no "
               f"train carried two frames")}
    if "frame" not in first:
        out["last"] = {"why": "the first frame was not placed"}
    elif out["last"] is None:
        ts, seq = parsed[-1]
        if offsets is not None:
            rel, basis = offsets, "the params.txt clock offsets"
        else:
            rel = (0.0, parsed[0][0] - trains[first["frame"] - 1]["t_end"])
            basis = "the first frame's alignment"
        last = place(ts, seq, rel, basis=basis)
        t_tractor = ts - rel[1] + rel[0]      # its time on the tractor clock
        if "frame" not in last and t_tractor > trains[-1]["t_end"]:
            last = {"why": f"it came after the TX log's last frame "
                           f"(#{len(trains)}): the TX log ends before the "
                           f"capture does"}
        if offsets is None:
            last.pop("dt", None)            # not a params-offset residual
        out["last"] = last
    return out


def scan_fault_sub(sub: int, strict: bool = True) -> dict | None:
    """An RX_SCAN_FAILED FAULT sub-byte (sx1276_rx_scan_fail.h, built by
    sx1276_rx_scan_fail_eval): ``attempt`` (bits 3..0), ``final`` (bit 7,
    set on the last attempt only: retries >= SX1276_RX_SCAN_MAX_RETRIES),
    ``warm`` (bit 6), ``hw_suspect`` (bit 5: no DIO0 IRQ in the attempt),
    ``crc_seen`` (bit 4, never with hw_suspect), ``marker``: 0x80, the
    final attempt 0 sx1276_rx_scan_fail_eval sends for a NULL input (its
    defensive path). None when the byte does not fit that layout: code 0x0D
    was also TX_FIFO_RB_MISMATCH on images before the 0x0D split (e083ae46
    and older), whose sub is the fragment's tx_id. The same 25 values as rx_scan_fail_sub_valid
    (method_g_stage1_probe.py, the daemons' decoder). ``strict`` False reads
    the bits whatever they hold (the daemon named the URC RX_SCAN_FAILED)."""
    attempt, final = sub & SCAN_SUB_ATTEMPT, bool(sub & SCAN_SUB_FINAL)
    hw, crc = bool(sub & SCAN_SUB_HW_SUSPECT), bool(sub & SCAN_SUB_CRC_SEEN)
    marker = sub == SCAN_SUB_FINAL
    if strict and not marker and (attempt > SCAN_MAX_RETRIES
                                  or final != (attempt == SCAN_MAX_RETRIES)
                                  or (hw and crc)):
        return None
    return {"attempt": attempt, "final": final,
            "warm": bool(sub & SCAN_SUB_WARM), "hw_suspect": hw,
            "crc_seen": crc, "marker": marker}


def _bracket_delta(pre: dict | None, post: dict | None, keys: tuple) -> dict | None:
    """``{key: d(key)}`` between the base's rs115 brackets (parse_stats), or
    None when a bracket lacks one of ``keys``. A counter that went down (an
    L072 reset between the brackets restarts them all at 0) gives the post
    values, as bracket_deltas.py reads a reset."""
    if pre is None or post is None or any(k not in pre or k not in post for k in keys):
        return None
    reset = any(post[k] < pre[k] for k in set(keys) | {
        "clk_demotion_reset", "clk_demotion_kept", "radio_dio0", "radio_rx_ok", "radio_tx_ok"}
        if k in pre and k in post)
    return {k: post[k] if reset else post[k] - pre[k] for k in keys}


def bracket_demotions(pre: dict | None, post: dict | None) -> tuple[int, int] | None:
    """``(d(clk_demotion_reset), d(clk_demotion_kept))`` between the base's
    rs115 brackets (parse_stats), or None when a bracket lacks them. sx1276_rx.c
    counts one only when a LOCKED scan drops back to SCANNING. A counter that
    went down (an L072 reset between the brackets restarts them all at 0)
    gives the post values, as bracket_deltas.py reads a reset."""
    d = _bracket_delta(pre, post, ("clk_demotion_reset", "clk_demotion_kept"))
    return None if d is None else (d["clk_demotion_reset"], d["clk_demotion_kept"])


def scan_image(lines: list[str], act: int, faults: list[dict],
               override: str = "auto") -> tuple[bool | None, str]:
    """``(restarts, source)``: whether the base's image restarts its scan
    after a final RX_SCAN_FAILED (the RS-12.21 image, scan option (b): True)
    or keeps an absorbing FAILED (every image before it: False), or None when
    nothing here says (then read as absorbing, the rule those images need),
    and what says so. In order: ``override`` (--scan-image restarts /
    absorbing; "auto" reads on); the capability bitmap on the last
    image_rx_daemon.py VER warm-up line before the activation (logged from the
    2026-10-24 review on; bit 7, HOST_CAP_RADIO_PARK, is the RS-12.21 image's);
    an RX_SCAN_FAILED after a final one in the window (``faults``, the scan
    faults), which an absorbing FAILED never sends."""
    if override not in SCAN_IMAGES:
        raise ValueError(f"scan image {override!r}: one of {', '.join(SCAN_IMAGES)}")
    if override != "auto":
        return override == "restarts", f"--scan-image {override} given"
    for n in range(act - 1, -1, -1):
        m = VER_CAP_RE.search(lines[n])
        if m:
            cap = int(m.group(1), 16)
            bit = "set" if cap & HOST_CAP_RADIO_PARK else "clear"
            return bool(cap & HOST_CAP_RADIO_PARK), (
                f"rx_daemon.log line {n + 1} logs the L072's VER capability bitmap "
                f"0x{cap:08X}, bit 7 (HOST_CAP_RADIO_PARK, the RS-12.21 image) {bit}")
    finals = [f for f in faults if f["scan"]["final"]]
    # (later by more than a second line of the same URC could be: the next
    # report comes a 30 s attempt later at the soonest)
    later = [f for f in faults if finals and f["line"] > finals[0]["line"]
             and f["t"] > finals[0]["t"] + FINAL_REPEAT_SLACK_S]
    if later:
        return True, (f"the base sent an RX_SCAN_FAILED at rx_daemon.log line "
                      f"{later[0]['line']} after its final one at line {finals[0]['line']}, "
                      f"which only an image that restarts its scan does")
    return None, ("rx_daemon.log logs no VER capability bitmap (image_rx_daemon.py "
                  "before the 2026-10-24 review) and no RX_SCAN_FAILED after a final one")


def acquisition(rx: str, tx: str | None = None, pre: dict | None = None,
                post: dict | None = None, image: str = "auto") -> dict:
    """The base's FHSS cold-start acquisition (2026-10-24, leg 6), one rule
    for rs12_leg_report.py and rs13_probe_score.py.

    The base daemon's first ``REG_PROFILE=N) OK`` line is its start-up
    activation; on profile 1 it starts the scan, dated by the next
    timestamped line. Everything below is read in the window from it to the
    daemon's next REG_PROFILE activation (``cut``: a profile switch calls
    sx1276_fhss_reset / sx1276_rx_scan_reset, host_cfg_profile.c, so the scan
    stops there; later activations are not checked), or to the log's end.

    First image reception (a fact, not a verdict): ``first``, the first
    ``frag_arrival`` or ``published frame_id`` line in the window, and
    ``heard`` / ``quiet``, the first stats line whose rx_frames counted a
    frame and the last one that had not (``(s, line)``; the counter is
    always on while the RX worker lives: ``crash``, when it died, ends both).

    ``status``, on positive evidence only (``basis`` says which; ``failed``
    True / False / None for FAILED / ACQUIRED / NOT DETERMINED). The rules
    depend on the base's image (``restarts`` and ``image``, scan_image():
    ``image`` "auto", or "restarts" / "absorbing" from --scan-image); an
    image not identified is read as absorbing, every image before RS-12.21:

    * FAILED on firmware evidence, absorbing image: a final RX_SCAN_FAILED
      that is cold (warm=0: never locked since the L072 booted), or that
      came less than SCAN_BUDGET_S + SCAN_LOCK_LOSS_S after the activation
      (4 attempts in a row with no lock; the activation dated by the last
      timestamped line before it). This wins over a lock shown: FAILED is
      absorbing, so nothing locks after it;
    * the same finals on the RS-12.21 image (scan option (b)) are 30 s
      reports: its scan restarts at once and sends a final every 30 s
      until a LOCK. From the first such final the chain of finals runs to a
      sign of a LOCK (a later RX_SCAN_FAILED that is not final, or warm
      after a cold one), else to the last final. A lock shown is placed
      after the chain by that sign, by a cold last final (no lock before
      it), or by a FAULT_URC-logging log that runs more than SCAN_ATTEMPT_S
      + FINAL_REPEAT_SLACK_S past it with no further final: then the lock
      counts, as a late acquisition (``basis`` says after which final).
      Without a placement, and with the log ending (or FAULT_URC logging
      off) within that span of the last final, it is FAILED with no lock
      shown, NOT DETERMINED with one (the bracket and the fragment share
      span the leg). A sign of a LOCK is itself a lock shown;
    * FAILED on silence: no reception shown within SCAN_BUDGET_S (on the
      RS-12.21 image: in the whole window -- its scan does not strand), and
      an instrument known to be on that shows none (a stats line after the
      budget, or the window's last, still at the activation's rx_frames, the
      RX worker alive; or per-fragment logging on, a frag_arrival line
      anywhere, with none in the budget or the window), and nothing that
      contradicts it: no lock shown (below), no FAULT_URC-logging daemon
      (``logged``) without an RX_SCAN_FAILED line, no base bracket whose
      d(radio_rx_ok) exceeds the host's count by more than d(rx_urc_lost) +
      HOST_DROP_SLACK (in a window no switch cut, the bracket spanning the
      leg). With a contradiction it is NOT DETERMINED, naming it: the
      silence is the host's, not the scan's;
    * ACQUIRED: no FAILED, and a lock shown in a window no switch cut: the
      base bracket (``pre`` / ``post``, parse_stats) counts a LOCKED ->
      SCANNING demotion, or the base decoded at least GRID_SHARE of the
      tractor's fragments on air on a hop set of GRID_MIN_CHANNELS or more
      (the mask the start-up configuration printed);
    * NOT DETERMINED otherwise (a frame received within the budget alone,
      silence that no instrument known to be on shows, a contradicted
      silence, or a window a profile switch cut).

    ``faults``: the timestamped FAULT code 0x0D lines in the window, the
    base's RX_SCAN_FAILED faults (logged by image_rx_daemon.py from
    2026-10-24 on as ``FAULT_URC: ... name=`` lines, ``logged`` True when the
    log holds any, and then only those count; dropped at DEBUG before), each
    with ``scan`` (scan_fault_sub, None for a TX_FIFO_RB_MISMATCH).
    ``tractor``: its frames on air (txdone_arrival trains, else "K fragments
    ok" lines) when ``tx`` is given. ``rx_frags``: the host's fragment count
    in the window, the stats counter floored by the frag_arrival and
    published lines (``rx_exact`` False when only those floors are left: the
    stats thread died and per-fragment logging was off). ``applies`` False,
    with ``why``, off profile 1 or without an activation line."""
    lines = rx.splitlines()
    acts = [i for i, ln in enumerate(lines) if ACTIVATION_RE.search(ln)]
    if not acts:
        return {"applies": False,
                "why": "rx_daemon.log has no REG_PROFILE activation line"}
    prof = ACTIVATION_RE.search(lines[acts[0]]).group(1)
    out = {"profile": prof, "later": len(acts) - 1}
    if prof != "1":
        return dict(out, applies=False,
                    why=f"the base daemon started on reg_profile {prof} "
                        f"({PROFILE_NAMES.get(prof, 'unknown')}): no FHSS scan")
    end = acts[1] if len(acts) > 1 else len(lines)
    at = next((n for n in range(acts[0] + 1, end)
               if _log_time(lines[n]) is not None), None)
    if at is None:
        return dict(out, applies=False, why="no timestamped line after its "
                                            "REG_PROFILE=1 activation")
    t_act = _log_time(lines[at])
    # the early-final rule dates the activation by the line before it
    lo = next((n for n in range(acts[0] - 1, -1, -1) if _log_time(lines[n]) is not None), None)
    base_rx = 0                      # rx_frames at the activation
    for ln in lines[:at]:
        m = STATS_RX_RE.search(ln)
        if m:
            base_rx = int(m.group(1))
    first = first_line = first_frag = heard = quiet = crash = None
    last_t, faults, rx_last, frag_n = t_act, [], base_rx, 0
    pubs, pub_any = [], 0
    for n in range(acts[0] + 1, end):
        line = lines[n]
        if crash is None and WORKER_CRASH in line:      # untimestamped
            crash = (last_t - t_act, n + 1)
        t = _log_time(line)
        if t is None or n < at:       # the untimestamped stdout FAULT lines too
            continue
        last_t = t
        if first is None and DECODED_RE.search(line):
            first, first_line = t - t_act, n + 1
        if FRAG_LINE_RE.search(line):
            frag_n += 1
            if first_frag is None:
                first_frag = t - t_act
        if "published frame_id" in line:
            pub_any += 1
            m = PUB_RE.search(line)
            if m and int(m.group(1)) >= 0:
                pubs.append(int(m.group(1)))
        m = STATS_RX_RE.search(line)
        if m and crash is None:
            rx_last = int(m.group(1))
            if heard is None and rx_last > base_rx:
                heard = (t - t_act, n + 1)
            elif heard is None:
                quiet = (t - t_act, n + 1)
        m = FAULT_RE.search(line)
        if m:
            name = FAULT_URC_RE.search(line)
            faults.append({"t": t - t_act, "line": n + 1,
                           "code": int(m.group(1), 16), "sub": int(m.group(2), 16),
                           "name": name.group(1) if name else None})
    logged = FAULT_URC_RE.search(rx) is not None    # the 2026-10-24 FAULT_URC lines
    faults = [f for f in faults if f["code"] == FAULT_CODE_SCAN
              and (f["name"] or not logged)]
    for f in faults:
        # the daemon's name= both ways: RX_SCAN_FAILED is a scan fault and
        # TX_FIFO_RB_MISMATCH is not, whatever the layout says; AMBIGUOUS and
        # a legacy line (no name=) go by the layout
        f["scan"] = (None if f["name"] == "TX_FIFO_RB_MISMATCH"
                     else scan_fault_sub(f["sub"],
                                         strict=f["name"] != "RX_SCAN_FAILED"))
    cut = None
    if len(acts) > 1:
        cut = {"line": acts[1] + 1, "t": last_t - t_act,
               "profile": ACTIVATION_RE.search(lines[acts[1]]).group(1)}
    tractor = tx_frags = None
    if tx is not None:
        tractor = (len(fragment_trains(fragment_events(tx, rx=False)))
                   or len(re.findall(r"\d+ fragments ok", tx)))
        tx_frags = (sum(int(k) for k in re.findall(r"(\d+) fragments ok", tx))
                    or len(fragment_events(tx, rx=False)))
    frag_on = FRAG_LINE_RE.search(rx) is not None
    stats_dead = STATS_CRASH in rx
    # floors on the fragments the base host received, as the loss block's: the
    # frag_arrival lines, and the published frames (each took a fragment at
    # least; a batched train publishes several under one seq, so on a leg
    # whose TX log may batch only the runs of one published seq count)
    one_per_train = tx is not None and not BATCH_LINE_RE.search(tx)
    pub_floor = pub_any if one_per_train else sum(
        1 for k, s in enumerate(pubs) if k == 0 or pubs[k - 1] != s)
    counter = 0 if stats_dead else rx_last - base_rx
    rx_frags = max(counter, frag_n, pub_floor)
    rx_ok = _bracket_delta(pre, post, ("radio_rx_ok",))
    urc = _bracket_delta(pre, post, ("rx_urc_lost",))
    mask = None
    for ln in lines[:acts[0]]:       # the start-up configuration's mask
        mask = MASK_RE.search(ln) or mask
    channels = (int(mask.group(1)) if mask.group(1) else 1) if mask else None
    restarts, image_why = scan_image(
        lines, acts[0], [f for f in faults if f["scan"] and not f["scan"]["marker"]
                         and f["name"] != "AMBIGUOUS"], image)
    out.update(applies=True, t_act=t_act, act_line=at + 1, restarts=restarts,
               image=image_why,
               lag=None if lo is None else t_act - _log_time(lines[lo]),
               lo_line=None if lo is None else lo + 1, cut=cut, crash=crash,
               first=first, first_line=first_line, first_frag=first_frag, heard=heard,
               quiet=quiet, base_rx=base_rx, frag_on=frag_on, span=last_t - t_act,
               faults=faults, logged=logged, tractor=tractor, rx_frags=rx_frags,
               rx_exact=not stats_dead or frag_on, stats_dead=stats_dead, tx_frags=tx_frags,
               channels=channels, demotions=bracket_demotions(pre, post),
               rx_ok=None if rx_ok is None else rx_ok["radio_rx_ok"],
               urc_lost=0 if urc is None else urc["rx_urc_lost"])
    out["status"], out["basis"] = _acquisition_verdict(out)
    out["failed"] = {FAILED: True, ACQUIRED: False}.get(out["status"])
    return out


def _lock_evidence(acq: dict) -> tuple[list[str], list[str]]:
    """``(locked, short)`` for :func:`_acquisition_verdict`: what shows a lock
    in the window (a bracket demotion, the grid share), and what falls short
    of showing one."""
    cut = acq["cut"]
    locked, short = [], []
    if cut:
        # the bracket and the tractor's fragments span the leg, not the scan
        short.append(f"the base switched to profile {cut['profile']} at rx_daemon.log line "
                     f"{cut['line']} (+{cut['t']:.1f} s), which stops the FHSS scan: what it "
                     f"decoded after that is not the scan's, so neither its share of the "
                     f"tractor's fragments nor a bracket demotion is weighed")
        return locked, short
    dem = acq["demotions"]
    if dem and sum(dem):
        locked.append(f"the base's bracket counts {sum(dem)} LOCKED -> SCANNING "
                      f"demotion(s) (d(clk_demotion_reset) {dem[0]}, d(clk_demotion_kept) "
                      f"{dem[1]}), which only a locked scan makes")
    elif dem:
        short.append("the base's bracket counts no LOCKED -> SCANNING demotion")
    rx_n, tx_n, ch = acq["rx_frags"], acq["tx_frags"], acq["channels"]
    if tx_n and ch and ch >= GRID_MIN_CHANNELS:
        least = "" if acq["rx_exact"] else "at least "
        share = (f"the base decoded {least}{rx_n} of the tractor's {tx_n} fragments on air "
                 f"({least}{100 * rx_n / tx_n:.1f} %)")
        if rx_n >= GRID_SHARE * tx_n:
            locked.append(f"{share}: on its {ch}-channel hop set a receiver that does not "
                          f"follow the hop grid is on the tractor's channel about one slot "
                          f"in {ch}, and only a locked scan follows the grid")
        elif not acq["rx_exact"]:
            short.append(f"the base's fragment count is unknown: its stats thread died "
                         f"(rx_daemon.log holds '{STATS_CRASH}') and per-fragment logging "
                         f"was off, so its published frames give only a floor ({share})")
        else:
            short.append(f"{share}, under the {100 * GRID_SHARE:g} % that shows a receiver "
                         f"following its {ch}-channel hop grid")
    elif not tx_n:
        short.append("no tractor fragment count (tx_daemon.log) to weigh the base's against")
    else:
        short.append(f"rx_daemon.log names no FHSS hop set of {GRID_MIN_CHANNELS} or more "
                     f"channels to weigh the base's fragment count against")
    return locked, short


def _acquisition_verdict(acq: dict) -> tuple[str, str]:
    """``(status, basis)`` for :func:`acquisition` (its docstring has the rule)."""
    budget = SCAN_BUDGET_S
    restarts = bool(acq["restarts"])
    scan = [f for f in acq["faults"] if f["scan"] and not f["scan"]["marker"]
            and f["name"] != "AMBIGUOUS"]
    final = [f for f in scan if f["scan"]["final"]]
    # 2026-10-24 review: RS-12.21 L anchors the loss timer at the LOCK itself
    anchor = ("on this image anchors the loss timer at the locking frame (RS-12.21 L)"
              if restarts else "keeps the channel anchor the activation's BEGIN_SCAN set")
    failed = []                  # (fault, why): the finals the absorbing rule reads as FAILED
    for f in final:
        where = (f"the base's final RX_SCAN_FAILED at +{f['t']:.1f} s (rx_daemon.log line "
                 f"{f['line']}, sub 0x{f['sub']:02X})")
        if not f["scan"]["warm"]:
            failed.append((f, f"{where} is cold (warm=0): the base's L072 had not locked "
                              f"since it booted"))
        elif acq["lag"] is not None and f["t"] + acq["lag"] < budget + SCAN_LOCK_LOSS_S:
            failed.append((f, f"{where} came less than {budget + SCAN_LOCK_LOSS_S:g} s after "
                              f"the activation (at most +{f['t'] + acq['lag']:.1f} s from "
                              f"rx_daemon.log line {acq['lo_line']}, the last timestamped line "
                              f"before it): 4 scan attempts failed in a row from it (a lock "
                              f"resets the attempt count but {anchor}, so a locked scan drops "
                              f"back to scanning no sooner than SX1276_RX_SCAN_LOCK_LOSS_MS "
                              f"{SCAN_LOCK_LOSS_S * 1000:.0f} after the activation)"))
    if failed and not restarts:
        return FAILED, "; ".join(why for _f, why in failed)
    cut, crash = acq["cut"], acq["crash"]
    locked, short = _lock_evidence(acq)
    late = None
    if failed:
        # RS-12.21 (b), 2026-10-24 review: on this image a final is a 30 s
        # report. The chain of finals from the first one the absorbing rule
        # would read as FAILED runs to a sign of a LOCK, else to the last.
        first = failed[0][0]
        last, sign, n = first, None, 1
        for f in scan:
            if f["line"] <= first["line"]:
                continue
            if f["sub"] == last["sub"] and f["t"] <= last["t"] + FINAL_REPEAT_SLACK_S:
                continue                    # a second line of the same URC
            if not f["scan"]["final"] or (f["scan"]["warm"] and not last["scan"]["warm"]):
                sign = f
                break
            last, n = f, n + 1
        reports = (f"{failed[0][1]}, but on this image a final is a {SCAN_ATTEMPT_S:g} s "
                   f"report, not a stop (RS-12.21 scan option (b): the scan restarts at once "
                   f"and reports again every {SCAN_ATTEMPT_S:g} s until a LOCK; "
                   f"{acq['image']})"
                   + (f"; {n} final reports in a row, the last at +{last['t']:.1f} s "
                      f"(rx_daemon.log line {last['line']})" if n > 1 else ""))
        gap = acq["span"] - last["t"]
        quiet = (sign is None and acq["logged"]
                 and gap > SCAN_ATTEMPT_S + FINAL_REPEAT_SLACK_S)
        if sign is not None:
            s = sign["scan"]
            locked.append(f"the base's RX_SCAN_FAILED at +{sign['t']:.1f} s (rx_daemon.log "
                          f"line {sign['line']}, sub 0x{sign['sub']:02X}) is "
                          + (f"attempt {s['attempt']}, not final: the attempt count restarts "
                             f"only at a LOCK" if not s["final"] else
                             "warm after a cold one: the L072 sets warm at a LOCK"))
            placed = (f"the scan locked between +{last['t']:.1f} s and +{sign['t']:.1f} s")
        elif locked and not last["scan"]["warm"]:
            placed = (f"the last final is cold (warm=0: no lock since the L072 booted), so the "
                      f"lock shown came after +{last['t']:.1f} s")
        elif locked and quiet:
            placed = (f"no RX_SCAN_FAILED followed the last final in the {gap:.1f} s the log "
                      f"runs past it, so the lock shown came within {SCAN_ATTEMPT_S:g} s of "
                      f"+{last['t']:.1f} s")
        else:
            placed = None
        if placed is not None:
            late = f"a late acquisition: {reports}; {placed}"
        elif not quiet:
            tail = (f"the log ends {gap:.1f} s after the last final, within the "
                    f"{SCAN_ATTEMPT_S + FINAL_REPEAT_SLACK_S:g} s a next report may take"
                    if acq["logged"] else
                    "this daemon does not log every FAULT_URC (no FAULT_URC line), so no "
                    "missing report shows a lock")
            if locked:
                return UNDETERMINED, (f"{reports}; nothing places the lock shown after the "
                                      f"last final ({tail}), and its evidence spans the leg: "
                                      + "; ".join(locked))
            return FAILED, f"{reports}; nothing shows a lock after the last final ({tail})"
        else:
            short.append(f"{reports}; no RX_SCAN_FAILED followed the last final in the "
                         f"{gap:.1f} s the log runs past it, which on this image means a LOCK "
                         f"within {SCAN_ATTEMPT_S:g} s of it unless its FAULT URCs were lost, "
                         f"but nothing else shows one")
    if crash:
        short.append(f"the base's image-rx-worker thread died at +{crash[0]:.1f} s "
                     f"(rx_daemon.log line {crash[1]}): its stats counter and frag_arrival "
                     f"lines stop counting there")
    first, heard, quiet = acq["first"], acq["heard"], acq["quiet"]
    # RS-12.21 (b): a scan that misses its budget does not strand, so silence
    # counts over the whole window, not the budget
    if restarts:
        within = first is not None or bool(heard)
        scope = f"in the {acq['span']:.1f} s window"
    else:
        within = (first is not None and first <= budget) or bool(heard and heard[0] <= budget)
        scope = f"within the {budget:g} s budget"
    silent = []
    if not within and restarts:
        if quiet and not crash and not acq["stats_dead"]:
            silent.append(f"the base's stats counter still read rx_frames={acq['base_rx']} at "
                          f"+{quiet[0]:.1f} s (rx_daemon.log line {quiet[1]}), its last stats "
                          f"line in the window (always on, it counts every image fragment, "
                          f"logged or not)")
        if acq["frag_on"] and acq["first_frag"] is None and not crash:
            silent.append("per-fragment logging was on (rx_daemon.log holds frag_arrival "
                          f"lines) and none came in the {acq['span']:.1f} s window")
    elif not within:
        if quiet and quiet[0] > budget:
            silent.append(f"the base's stats counter still read rx_frames={acq['base_rx']} at "
                          f"+{quiet[0]:.1f} s (rx_daemon.log line {quiet[1]}; always on, it "
                          f"counts every image fragment, logged or not)")
        # (a dead worker logs no frag_arrival, so a first one past the budget
        # shows it alive until then)
        frag_after = (acq["first_frag"] if acq["first_frag"] is not None
                      else acq["span"] if acq["span"] > budget and not crash else None)
        if acq["frag_on"] and frag_after is not None and frag_after > budget:
            silent.append("per-fragment logging was on (rx_daemon.log holds frag_arrival lines) "
                          "and " + (f"the first came at +{acq['first_frag']:.1f} s"
                                    if acq["first_frag"] is not None else
                                    f"none came in the {acq['span']:.1f} s window"))
    if silent:
        doubt = list(locked)
        if acq["logged"] and not any(f["scan"] for f in acq["faults"]):
            doubt.append("this daemon logs every FAULT_URC (rx_daemon.log holds FAULT_URC "
                         "lines) and a scan that runs out sends an RX_SCAN_FAILED one per "
                         "attempt, yet the window holds none")
        rx_ok, rx_n = acq["rx_ok"], acq["rx_frags"]
        # (the bracket spans the leg, so not past a switch: rx_n is the window's)
        if (not cut and rx_ok is not None
                and rx_ok - rx_n > acq["urc_lost"] + HOST_DROP_SLACK):
            doubt.append(f"the base's bracket counts d(radio_rx_ok) {rx_ok} against the host's "
                         f"{rx_n} (FIRMWARE DROP {rx_ok - rx_n}, d(rx_urc_lost) "
                         f"{acq['urc_lost']}): the L072 delivered frames the host never counted")
        text = "; ".join(silent) + f": no image fragment reached the base {scope}"
        if not doubt:
            return FAILED, text + ", and its scan locks only on a frame it decodes"
        why = ("A scan that runs out reports it and scans on (this image restarts it, "
               "RS-12.21 (b)), so this silence is the host's, not the scan's" if restarts else
               "The scan's FAILED is absorbing (only a re-activation leaves it), so this "
               "silence is the host's, not the scan's")
        return UNDETERMINED, (
            text + " by the host's count, but " + "; ".join(doubt) + f". {why}: its RX path "
            "missed what the base received (UART contention, a stalled RX worker, lost "
            "URCs), or the base locked on a command frame, which rx_frames does not count")
    if locked:
        return ACQUIRED, "; ".join(locked + ([late] if late else []))
    if within:
        lead = ("a frame received is not a lock (sx1276_rx.c still delivers a frame whose FHSS "
                "header it rejected), and nothing here shows one")
    elif first is None and not heard and acq["span"] <= budget and not restarts:
        lead = (f"the scan ran {acq['span']:.1f} s before the base's profile switch, no more "
                f"than the budget" if cut else
                f"the base's log covers {acq['span']:.1f} s, no more than the budget")
    else:
        lead = (("the base's first frame line came past the budget" if first is not None or heard
                 else "the base's log holds no frame line")
                + ", but no instrument known to be on shows that no frame reached it "
                + ("in the window" if restarts else "within the budget")
                + ("" if crash else
                   " (no live stats line in it, no frag_arrival line to show per-fragment "
                   "logging on)" if restarts else
                   f" (no stats line after it still at rx_frames={acq['base_rx']}, no "
                   f"frag_arrival line to show per-fragment logging on)"))
    return UNDETERMINED, "; ".join([lead] + short)


def reception_phrase(acq: dict) -> str:
    """The base's first image reception against the scan budget, in a few
    words (a fact, not the acquisition verdict): rs13_probe_score.py's
    per-leg line."""
    budget = f"the {SCAN_BUDGET_S:g} s FHSS scan budget"
    first, heard, quiet = acq["first"], acq["heard"], acq["quiet"]
    if first is not None:
        return (f"first image reception {first:.1f} s after the base's profile-1 activation, "
                f"{'past' if first > SCAN_BUDGET_S else 'within'} {budget}")
    if heard:
        return (f"first image reception by +{heard[0]:.1f} s after the base's profile-1 "
                f"activation (its stats counter; no decoded-frame line), "
                + _against(heard[0], quiet, budget))
    where = (f"from the base's profile-1 activation to its switch to profile "
             f"{acq['cut']['profile']}" if acq.get("cut") else
             "the base log covers after its profile-1 activation")
    return (f"no image reception in the {acq['span']:.1f} s {where}, "
            + ("past" if acq["span"] > SCAN_BUDGET_S else "no longer than") + f" {budget}")


def _against(heard_s: float, quiet, budget: str) -> str:
    """A reception the stats counter dates only to (quiet, heard] against the budget."""
    if heard_s <= SCAN_BUDGET_S:
        return f"within {budget}"
    if quiet and quiet[0] > SCAN_BUDGET_S:
        return f"past {budget}"
    return (f"not placed against {budget} (the counter read no frame at "
            + (f"+{quiet[0]:.1f} s)" if quiet else "its first line after the activation)"))


def format_acquisition(acq: dict) -> list[str]:
    """Report lines for :func:`acquisition`: the ``acquisition:`` line (the
    first image reception, a fact), the verdict line (``confirmed
    acquisition:`` ACQUIRED or NOT DETERMINED, or the ``!! ACQUISITION
    FAILED`` flag), each with the evidence, and the base's scan faults."""
    if not acq["applies"]:
        return [f"acquisition: n/a ({acq['why']})"]
    budget = (f"the FHSS cold-start scan budget of {SCAN_BUDGET_S:g} s "
              f"({SCAN_MAX_RETRIES + 1} attempts x {SCAN_ATTEMPT_S:g} s: "
              f"SX1276_RX_SCAN_MAX_RETRIES {SCAN_MAX_RETRIES} + 1, "
              f"SX1276_RX_SCAN_REDESIGN_MS {SCAN_ATTEMPT_S * 1000:.0f})")
    stamp = datetime.fromtimestamp(acq["t_act"], timezone.utc).strftime(
        "%H:%M:%S.%f")[:-3]
    act = f"rx_daemon.log line {acq['act_line']} at {stamp} board time"
    first, heard = acq["first"], acq["heard"]
    if first is not None:
        out = [f"acquisition: first image reception: the base's first decoded frame came "
               f"{first:.1f} s after its profile-1 activation ({act}, the frame at line "
               f"{acq['first_line']}), "
               f"{'past' if first > SCAN_BUDGET_S else 'within'} {budget}"]
        if heard and heard[0] < first:
            out[0] += (f"; its stats counter counted a frame by +{heard[0]:.1f} s (line "
                       f"{heard[1]})")
    elif heard:
        out = [f"acquisition: first image reception: no decoded-frame line, but the base's "
               f"stats counter counted a frame by +{heard[0]:.1f} s after its profile-1 "
               f"activation ({act}; the counter at line {heard[1]}), "
               + _against(heard[0], acq["quiet"], budget)]
    else:
        where = ("from its profile-1 activation ({act}) to its profile switch" if acq["cut"]
                 else "its log covers after its profile-1 activation ({act})").format(act=act)
        out = [f"acquisition: first image reception: none, the base decoded no frame in the "
               f"{acq['span']:.1f} s {where}, "
               + ("past" if acq["span"] > SCAN_BUDGET_S else "no longer than") + f" {budget}"]
    cut = acq["cut"]
    if cut:
        # 2026-10-24 review: a profile switch stops the scan; the window ends there
        out[0] += (f"; read up to its switch to profile {cut['profile']} (rx_daemon.log line "
                   f"{cut['line']}, +{cut['t']:.1f} s)"
                   + (f", {acq['later'] - 1} later REG_PROFILE activation(s) not checked"
                      if acq["later"] > 1 else ""))
    if acq["failed"]:
        sent = ("" if acq["tractor"] is None else
                f" The tractor logged {acq['tractor']} frame(s) on air."
                + (" It sent nothing, so the scan had nothing to find."
                   if not acq["tractor"] else ""))
        if acq["restarts"]:
            # 2026-10-24 review: the RS-12.21 image does not strand the scan
            out.append(f"  !! ACQUISITION FAILED: {acq['basis']}. This image restarts the "
                       "scan after each final RX_SCAN_FAILED (RS-12.21 scan option (b)), so "
                       "the base was not stranded on one channel, but it never locked onto "
                       "the tractor's hop grid in the window: this leg's loss and command "
                       "figures measure an unacquired base, not the link." + sent)
        else:
            # 2026-10-24 review: an image not identified is read by the old
            # rule; say so, and how to read it as the RS-12.21 image
            unknown = ("" if acq["restarts"] is not None else
                       f" The base's image was not identified ({acq['image']}), so the "
                       "rule of every image before RS-12.21 read it; the RS-12.21 image "
                       "(VER capability bit 7) restarts its scan after a final "
                       "RX_SCAN_FAILED and counts silence over the whole window -- if it "
                       "flew, run again with --scan-image restarts.")
            out.append(f"  !! ACQUISITION FAILED: {acq['basis']}. Its scan ran out into "
                       "FAILED, which is absorbing on every image before RS-12.21 "
                       "(sx1276_rx_scan_policy.c): a follower there never hops and hears "
                       "only what crosses the channel it is parked on, until a reset or a "
                       "profile re-activation. This leg's loss and command figures measure "
                       "that, not the link (2026-10-24 leg 6)." + unknown + sent)
    else:
        out.append(f"  confirmed acquisition: {acq['status']}: {acq['basis']}")
    scan = [f for f in acq["faults"] if f["scan"]]
    other = [f for f in acq["faults"] if not f["scan"]]
    if scan:
        def cell(f):
            s = f["scan"]
            flags = [k for k in ("final", "warm", "hw_suspect", "crc_seen") if s[k]]
            if s["marker"]:
                flags.append("the firmware's defensive NULL-input marker")
            if f["name"] == "AMBIGUOUS":
                flags.append("AMBIGUOUS with TX_FIFO_RB_MISMATCH")
            return (f"attempt {s['attempt']} at +{f['t']:.1f} s (line {f['line']}, "
                    f"sub 0x{f['sub']:02X}" + (f": {', '.join(flags)}" if flags else "")
                    + ")")
        how = ("the daemon's FAULT_URC name= and the scan sub-byte layout"
               if acq["logged"] else "the scan sub-byte layout")
        out.append(f"  base RX_SCAN_FAILED fault(s), FAULT code 0x0D read by {how}: "
                   + "; ".join(cell(f) for f in scan))
        final = [f for f in scan if f["scan"]["final"]]
        if final:
            f = final[-1]
            amb = f["name"] == "AMBIGUOUS"
            out.append(f"  !! the base's final RX_SCAN_FAILED at +{f['t']:.1f} s "
                       f"(rx_daemon.log line {f['line']}, sub 0x{f['sub']:02X}"
                       + (", AMBIGUOUS with TX_FIFO_RB_MISMATCH" if amb else "")
                       + ("): its scan reported FAILED there and scanned on (this image "
                          "restarts it at once and reports every 30 s until a LOCK, "
                          "RS-12.21 (b))" if acq["restarts"] else
                          "): its scan entered FAILED there")
                       + (" if this URC was the scan's (the daemon could not rule "
                          "out a TX FIFO readback mismatch on a TX with that tx_id)"
                          if amb else ""))
    if other:
        out.append("  base FAULT code 0x0D read as TX_FIFO_RB_MISMATCH (sub = the "
                   "fragment's tx_id; " + ("by name= or " if acq["logged"] else "")
                   + "not in the scan sub-byte layout): "
                   + ", ".join(f"sub 0x{f['sub']:02X} at +{f['t']:.1f} s "
                               f"(line {f['line']})" for f in other))
    if acq["failed"] and not scan and not acq["logged"]:
        # 2026-10-24 review: a FAILED with no RX_SCAN_FAILED line is the
        # silence rule's, which a FAULT_URC-logging daemon would contradict
        # (acquisition() reads those as NOT DETERMINED); this log holds no
        # FAULT_URC line at all, so it cannot tell the two daemons apart
        out.append("  (rx_daemon.log holds no FAULT_URC line, so the FAILED entry is inferred, "
                   "not seen: image_rx_daemon.py dropped FAULT URCs at DEBUG before 2026-10-24; "
                   "from 2026-10-24 on it logs every one, and a scan that runs out sends an "
                   "RX_SCAN_FAILED per attempt, so with that daemon this silence would "
                   "contradict the FAILED entry; check which daemon ran)")
    return out


def capture_frames(path) -> tuple[list, dict]:
    """``(ts, codec, seq)`` per payload of a ``vector_dry_run.py capture``
    JSONL, plus the codec-name map. Imported lazily, so the archive report
    keeps working where the base_station tree is not importable."""
    tools = str(pathlib.Path(__file__).resolve().parent)
    if tools not in sys.path:
        sys.path.insert(0, tools)
    import vector_dry_run as vdr                 # puts base_station/ on sys.path
    from image_pipeline.frame_format import parse_tile_delta_frame
    out = []
    for ts, _topic, payload in vdr.iter_capture(path):
        try:
            frame = parse_tile_delta_frame(payload)
        except Exception:                        # noqa: BLE001 -- counted
            out.append((ts, None, None))
        else:
            out.append((ts, frame.codec, frame.base_seq))
    return out, vdr.CODEC_NAMES


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("archive", type=pathlib.Path)
    ap.add_argument("--pre", type=pathlib.Path)
    ap.add_argument("--post", type=pathlib.Path)
    ap.add_argument("--capture", type=pathlib.Path,
                    help="base-side vector_dry_run capture JSONL: adds the "
                         "frame loss from TileDeltaFrame sequence gaps")
    ap.add_argument("--scan-image", choices=SCAN_IMAGES, default="auto",
                    help="how the base's L072 image treats a final RX_SCAN_FAILED: "
                         "'restarts' (the RS-12.21 image, scan option (b): a 30 s "
                         "report, the scan goes on) or 'absorbing' (every image "
                         "before it); 'auto' (default) reads the VER capability "
                         "bitmap image_rx_daemon.py logs, else the faults")
    args = ap.parse_args(argv)

    rx = (args.archive / "rx_daemon.log").read_text(encoding="utf-8",
                                                    errors="replace")
    tx = (args.archive / "tx_daemon.log").read_text(encoding="utf-8",
                                                    errors="replace")
    params = (args.archive / "params.txt").read_text(encoding="utf-8",
                                                     errors="replace")
    params_kv = params_dict(params)

    for key in ("train_gap_ms", "tx_pipeline_depth", "synth_budget_b",
                "force_frf_hz", "git_sha", "log_window"):
        m = re.search(rf"{key}=(\S+)", params)
        if m:
            print(f"{key}={m.group(1)}", end="  ")
    # 2026-10-24 (review): the acquisition lines come after the loss block,
    # past leg_post.sh's 8-line console excerpt whenever a stale-counter line
    # prints, so a failed acquisition also marks this first line. The base
    # brackets (leg_post.sh passes the base's) can show a lock. Read here only
    # if both are readable (review): a missing one must not cost the loss
    # lines; the radio-deltas block below still fails on it, as before.
    try:
        brackets = ((parse_stats(args.pre), parse_stats(args.post))
                    if args.pre and args.post else (None, None))
    except OSError:
        brackets = (None, None)
    acq = acquisition(rx, tx, *brackets, image=args.scan_image)
    print("!! ACQUISITION FAILED (see the acquisition line below)"
          if acq.get("failed") else "")
    # 2026-10-19: every harness up to the 10-19 legs read the daemon logs
    # BEFORE it stopped the daemons (TX log first, then RX), while the post
    # brackets are read after; a harness that reads them after the stop says
    # so with log_window=after_stop in params.txt.
    # 2026-10-22: the harness writes after_stop only when both daemon stops
    # worked and both logs hold their SIGTERM shutdown line; otherwise it
    # writes log_window=after_stop_unverified and the reason in
    # log_window_detail=. Only the exact after_stop drops the note; any other
    # value, or none (every archive before 10-22), keeps the old note.
    # The unverified note is printed after the loss block (whole-log,
    # shared-window and capture seq-gap lines), not here: leg_post.sh shows
    # only the report's first 8 lines on the console, and the capture line
    # (the loss of record) must stay among them (2026-10-22 review). Its
    # wording follows the reason: a docker stop the harness could not
    # confirm leaves the old artefact possible (a daemon still ran when its
    # log was read); a missing SIGTERM shutdown line after both stops
    # worked is the opposite case, a daemon that died (or a log that did
    # not come back whole) before the stop, so that log ends early.
    m = re.search(r"log_window=(\S+)", params)
    log_window = m.group(1) if m else None
    logs_after_stop = log_window == "after_stop"
    logs_unverified = log_window == "after_stop_unverified"
    unverified_note = None
    if logs_unverified:
        md = re.search(r"^log_window_detail=(.*?)\s*$", params, re.M)
        detail = (md.group(1) if md and md.group(1)
                  else "params.txt has no log_window_detail")
        stop_failed = re.search(r"\b(?:tx|rx)_smoke stop:", detail)
        no_shutdown = re.search(r"\b(?:tx|rx)_smoke log has no shutdown line",
                                detail)
        if stop_failed:
            why = ("a daemon stop was not confirmed, so that daemon may "
                   "still have run when its log was read and the logs may "
                   "not cover the post-stop window — a 'received > sent' "
                   "line above or a small positive FIRMWARE DROP below can "
                   "carry the old log-window artefact (logs read while a "
                   "daemon ran), not the link or the firmware")
            if no_shutdown:
                why += ("; a log without its shutdown line can also mean "
                        "that daemon died before the stop, so its log ends "
                        "early")
        elif no_shutdown:
            why = ("both daemon stops were confirmed, but a daemon log has "
                   "no SIGTERM shutdown line — that daemon may have died "
                   "before the stop (look for a traceback in its log) or "
                   "its log did not come back whole, so the log can end "
                   "before the leg did and the log-derived figures above "
                   "cover less than the leg (an RX log that ends early "
                   "inflates the whole-log loss and the FIRMWARE DROP "
                   "below); this is not the old read-while-running artefact")
        else:
            why = ("the harness could not confirm that each daemon ran until "
                   "its stop and had stopped before its log was read, so the "
                   "log-derived figures above and a FIRMWARE DROP below may "
                   "not cover the leg's window")
        unverified_note = (f"  !! LOG WINDOW UNVERIFIED (log_window_detail: "
                           f"{detail}): {why}")

    # PR #125 review round 6 (2026-09-15): `frags_ok=` is the TX daemon's LAST
    # periodic stats line and can be stale independently of the RX stats
    # thread -- in leg U it read 652 while the per-frame completion events
    # ("frame seq=N done ...: K fragments ok", one per train, logged as each
    # train finishes) summed to 674. Those events are the fragments the L072
    # reported on air (TX_DONE OK); aborted fragments never appear in them
    # and were never on air, so they are the right air-loss denominator.
    # Prefer the events; fall back to the counter only for a log without them.
    sent_counter = int(re.findall(r"frags_ok=(\d+)", tx)[-1])
    sent_events = sum(int(n) for n in re.findall(r"(\d+) fragments ok", tx))
    sent = sent_events if sent_events else sent_counter
    if sent_events and sent_events != sent_counter:
        print(f"  (tx frags_ok counter {sent_counter} is stale; using "
              f"{sent_events} fragments from per-frame TX events)")
    rcvd = _last_int(r"rx_frames=(\d+)", rx)
    crc = rx.count("crc_dump")
    timeouts = _last_int(r"reassembler_timeouts=(\d+)", rx)
    published = _last_int(r"frames_published=(\d+)", rx)

    # 2026-09-15 (leg U): EVERY number on the loss line comes from the LAST
    # "stats:" line in the rx log. If the daemon's stats thread dies mid-leg
    # that line is whatever was true when it died, and this report then
    # describes the first seconds as if they were the whole leg — silently,
    # and in the direction of a false alarm. Leg U crashed that thread one
    # frame in and this printed "loss 651/652 = 99.8% published=1" for a leg
    # that actually published 537 frames with zero lock losses. Cross-check
    # against events logged once per frame/fragment, which cannot go stale.
    pub_log = rx.count("published frame_id")
    frag_log = rx.count("frag_arrival")          # needs -LogFragArrivals 1
    stats_dead = "Exception in thread image-rx-stats" in rx
    counters_stale = stats_dead or (pub_log and published < pub_log * 0.9)
    if counters_stale:
        why = ("the stats thread CRASHED (traceback in rx_daemon.log)"
               if stats_dead else
               "the counter line disagrees with the per-frame log events")
        print(f"  !! STALE COUNTERS: {why} — the stats-line figures below "
              f"(timeouts) describe only the window before it stopped "
              f"updating; rx and published are floored by the log events "
              f"(A14). Trust these:")
        if frag_log:
            print(f"     log-derived: published={pub_log}  "
                  f"fragments_arrived={frag_log}  "
                  f"air_loss={100 * (sent - frag_log) / sent:.1f}% "
                  f"({sent} sent -> {frag_log} decoded)")
        else:
            print(f"     log-derived: published={pub_log}  "
                  f"(re-run with -LogFragArrivals 1 for fragment-level loss)")

    # RS-13.1 A14 (2026-10-03): even with a healthy stats thread the
    # counters above come from the LAST periodic "stats:" line (every 10 s),
    # so they miss whatever arrived between that line and the end of the
    # log. On every leg of RS-13.1 round 3 that inflated the loss line
    # (2a_yt: rx_frames=288 against 297 published -> 4.3 % printed, 1.3 %
    # from sequence gaps). Every published frame and every logged
    # frag_arrival is a fragment the base received, so both event counts
    # are floors on the true rx count: the counter is used only when it is
    # not below them. frag_arrival (-LogFragArrivals 1) is fragment-level,
    # the same unit as `sent`; the published count is the fallback and the
    # same number on one-fragment legs.
    # A published frame is one fragment only when every train carried one
    # (the RS-13 legs run -TxBatch 0): with batching a train of K fragments
    # can publish several frames, and a frame count is no floor on a
    # fragment count (it once printed a negative loss). frag_arrival is
    # fragment-level and always comparable.
    one_frag_per_frame = bool(sent_events) and not re.search(
        r"done(?: \(pipelined\))?: (?:[02-9]|[1-9][0-9]+) fragments ok", tx)
    candidates = [(frag_log, "fragments from frag_arrival events")]
    if one_frag_per_frame:
        candidates.append((pub_log, "frames from 'published frame_id' events"))
    floor, floor_src = max(candidates, key=lambda c: c[0])
    if floor > rcvd:
        print(f"  (rx rx_frames counter {rcvd} is stale; using {floor} "
              f"{floor_src})")
        rcvd = floor
    # 2026-10-19: the clamp below is for the whole-log loss line only. It
    # used to overwrite rcvd itself, so the FIRMWARE DROP line further down
    # subtracted the TX count, not the base's own (A2c: 617 - 611 = 6
    # instead of 617 - 612 = 5). Keep the base's count for that line.
    rcvd_host = rcvd
    shared = shared_window_loss(tx, rx)
    shared_ok = shared is not None and not shared.get("withheld")
    if rcvd > sent:
        # More received than sent: the logs or counters disagree (the RX log
        # runs on past the TX log's last line, a restarted daemon). Never
        # print a negative loss; say so instead. With --capture the capture
        # seq-gap line is the loss of record (RS-13), so point there first.
        if args.capture:
            hint = ("check the capture seq-gap line (the loss of record)"
                    + (" and the shared-window line" if shared_ok else ""))
        else:
            hint = ("read the shared-window line" if shared_ok else
                    "re-run with --capture <leg>_base.jsonl for the seq-gap "
                    "figure (no shared-window figure here)")
        if logs_unverified:
            # 2026-10-22 (review): the note is printed below the loss block.
            hint += ("; the log window is unverified, see the LOG WINDOW "
                     "UNVERIFIED note below")
        print(f"  !! received {rcvd} > sent {sent}: the TX and RX logs disagree "
              f"on the window; whole-log loss shown as 0 — {hint}")
        rcvd = sent
    published = max(published, pub_log)
    # The whole-log line is kept exactly as before (only the label is new),
    # so a report on old evidence reads the same; the shared-window line
    # below it is the fragment loss without the log-window artefact.
    print(f"loss {sent - rcvd}/{sent} = {100 * (sent - rcvd) / sent:.1f}%   "
          f"crc_dumps={crc}  timeouts={timeouts}  published={published}"
          + ("   [STALE — see above]" if counters_stale else "")
          + "   [whole logs]")
    for line in format_shared_loss(shared):
        print(line)
    if args.capture:
        frames, names = capture_frames(args.capture)
        # 2026-10-24: the head placed on the TX log, and the leg's synth_fps
        # for a capture with no clean +1 step (leg 6)
        res = seq_gap_loss(frames, nominal_period(params_kv))
        for line in format_seq_loss(res, args.capture.name, names,
                                    place_capture(frames, tx, rx, params_kv)):
            print(line)
    if unverified_note:
        print(unverified_note)
    # 2026-10-24: after the loss block, which leg_post.sh's 8-line excerpt
    # must keep (the capture line is the loss of record)
    for line in format_acquisition(acq):
        print(line)

    # per-index loss histogram (attribution instrument)
    idx: collections.Counter = collections.Counter()
    for m in re.finditer(r"lost_frag_idx: n=\d+.*?\| top ([0-9: ]+)", rx):
        for pair in m.group(1).split():
            i, c = pair.split(":")
            idx[int(i)] += int(c)
    tot = sum(idx.values())
    if tot:
        # Train length comes from the TX log's healthy "K fragments ok"
        # lines, and the penultimate metric is scoped to the LONG-train
        # population (>= MIN_LONG fragments).
        #
        # 2026-08-24, two bugs found in one leg:
        #  (a) this used to read the `total` byte out of CORRUPT capture
        #      headers — the one population whose bytes are unreliable —
        #      with most_common(1) and no sanity bound. A control leg
        #      with a single readable dump reported train length 208 off
        #      that one garbage byte, moving "penultimate" to idx 206 and
        #      printing 0 % while the real lock sat at 35 % on idx 11: a
        #      false NEGATIVE on the campaign's headline metric.
        #  (b) a plain modal over ALL trains is also wrong — these legs
        #      carry a MIXTURE (e.g. 143x1, 45x2, 56x12, 115x13), so the
        #      overall mode is 1 and "penultimate" collapses to idx -1.
        #      Short trains have no penultimate to lock, so the metric is
        #      only meaningful over long trains.
        # The mixture is printed so a reader can never mistake a
        # mixed-length leg for a uniform one.
        MIN_LONG = 3
        tx_lens = collections.Counter(
            int(n) for n in re.findall(r"(\d+) fragments ok", tx))
        long_lens = collections.Counter(
            {k: v for k, v in tx_lens.items() if k >= MIN_LONG})
        if long_lens:
            tlen = long_lens.most_common(1)[0][0]
            tlen_src = (f"tx log, modal of {sum(long_lens.values())} long "
                        f"trains; mixture " +
                        " ".join(f"{k}x{v}" for k, v in
                                 sorted(tx_lens.items())))
        else:
            tlen = 13
            tlen_src = "DEFAULT 13 -- no long trains in tx log, treat with care"
        pen = tlen - 2
        print(f"train length {tlen} [{tlen_src}]")
        print(f"attributed {tot}; penultimate idx {pen} = {idx.get(pen, 0)} "
              f"({100 * idx.get(pen, 0) / tot:.0f}%, uniform "
              f"{100 / tlen:.0f}%)")

        # MIXED-LENGTH AMBIGUITY (review catch, PR #112). The rx daemon's
        # lost_frag_idx instrument reports an index with no train length,
        # so this histogram aggregates every length present. When more
        # than one long length occurs, a single index means different
        # things per population: at lengths 12 and 13, idx 11 is the
        # FINAL fragment of the 12s and the PENULTIMATE of the 13s. The
        # share printed above is therefore mixture-weighted, not a clean
        # per-population figure. Say so rather than letting the label
        # imply a purity the instrument cannot deliver.
        other_long = sorted(k for k in long_lens if k != tlen)
        if other_long:
            print(f"  !! MIXED LENGTHS {sorted(long_lens)} -- idx {pen} is "
                  f"penultimate for len {tlen}, but also "
                  + ", ".join(
                      f"{'final' if pen == k - 1 else f'idx {pen}'} of "
                      f"len {k}" for k in other_long)
                  + f"; each length's own penultimate: "
                  + ", ".join(f"len {k}->idx {k - 2}={idx.get(k - 2, 0)}"
                              for k in sorted(long_lens))
                  + ". Shares are mixture-weighted; separating them needs "
                    "lost_frag_idx to carry the train length.")
        print("  " + " ".join(f"{i}:{idx.get(i, 0)}" for i in range(tlen)))

        # corrupt-capture indices
        cidx: collections.Counter = collections.Counter()
        for m in re.finditer(r"dump=([0-9a-f]{24,})", rx):
            d = bytes.fromhex(m.group(1)[:24])
            if d[8] == 0xFE and d[11] + 1 == tlen:
                cidx[d[10]] += 1
        ctot = sum(cidx.values())
        if ctot:
            print(f"corrupt-capture idx (readable {ctot}): penultimate "
                  f"{cidx.get(pen, 0)} ({100 * cidx.get(pen, 0) / ctot:.0f}% "
                  f"vs uniform {100 / tlen:.0f}%)")

    if args.pre and args.post:
        pre = parse_stats(args.pre)
        post = parse_stats(args.post)
        d = {k: post[k] - pre[k] for k in
             ("radio_dio0", "radio_rx_ok", "radio_crc_err", "radio_tx_ok")
             if k in pre and k in post}
        print(f"radio deltas: {d}")
        fw_drop = d.get("radio_rx_ok", 0) - rcvd_host
        print(f"FIRMWARE DROP (drx_ok - host URCs): "
              f"{d.get('radio_rx_ok', 0)} - {rcvd_host} = {fw_drop}")
        if logs_unverified:
            # 2026-10-22: the stop or a shutdown line was not confirmed (the
            # LOG WINDOW UNVERIFIED note above says which). Worded for both
            # causes: a daemon read while it ran, or an RX daemon that died
            # before its stop (its log ends early, the bracket does not).
            print("  (log window: after_stop_unverified, see the LOG WINDOW "
                  "UNVERIFIED note above — if a daemon still ran when its "
                  "log was read, or the RX daemon died before its stop, "
                  "fragments the base received after its log ended count in "
                  "drx_ok but not in the log: a positive drop can be that "
                  "window, not the firmware; rx_urc_lost on the URC-path "
                  "line counts the URCs the L072 really lost)")
        elif not logs_after_stop:
            # Every 10-18/10-19 leg read 4-5 here (4-6 with the old clamp):
            # mostly the fragments the base decoded between the RX log read
            # and the daemon stop, which are in the post bracket's
            # radio_rx_ok but not in the log (the harness reads and stops
            # the feed's container, camera_svc or synth_pub, in between);
            # rx_urc_lost (0-2 on those legs) is the rest.
            print("  (log window: params.txt has no log_window=after_stop — "
                  "this harness read the daemon logs BEFORE it stopped the "
                  "daemons, and the post bracket was read after, so "
                  "fragments the base received in between count in drx_ok "
                  "but not in the log: a small positive drop can be that "
                  "window, not the firmware; rx_urc_lost on the URC-path "
                  "line counts the URCs the L072 really lost)")
        resid = d.get("radio_dio0", 0) - sum(
            d.get(k, 0) for k in ("radio_rx_ok", "radio_crc_err",
                                  "radio_tx_ok"))
        print(f"identity residue: {resid}")
        print(f"crc closure: dcrc_err={d.get('radio_crc_err', 0)} "
              f"vs crc_dumps={crc}")
        # RS-12 (2026-09-12): URC-path loss counters, present only on the
        # instrumented build; absent keys print as n/a so old brackets
        # still analyse.
        urc = {k: (post[k] - pre[k]) if (k in pre and k in post) else "n/a"
               for k in ("rx_urc_lost", "rx_pretx_drained")}
        print(f"URC-path: rx_urc_lost={urc['rx_urc_lost']} "
              f"rx_pretx_drained={urc['rx_pretx_drained']}")
        # RS-12.10 (2026-09-12): FIFO-skip detector (the coalescing the edge
        # counter cannot see) and the TX deaf window. max fields are NOT
        # deltas — the bracket max is the leg max only when the counters
        # were reset at launch; sum is a true delta.
        # The probe dump lists only NON-ZERO counters, so a zero reads as
        # absent: treat any RS-12.10 key present in the post bracket as
        # proof of the instrumented build and default the rest to 0.
        keys = ("rx_fifo_skip", "tx_deaf_sum_us")
        maxk = ("tx_deaf_max_us", "tx_done_to_rearm_max_us")
        if any(k in post for k in keys + maxk):
            pre = {**{k: 0 for k in keys + maxk}, **pre}
            post = {**{k: 0 for k in keys + maxk}, **post}
            tx_n = max(d.get("radio_tx_ok", 0), 1)
            deaf_sum = post["tx_deaf_sum_us"] - pre["tx_deaf_sum_us"]
            print(f"RS-12.10: rx_fifo_skip={post['rx_fifo_skip'] - pre['rx_fifo_skip']} "
                  f"tx_deaf_sum={deaf_sum} us over {tx_n} TX "
                  f"(mean {deaf_sum / tx_n / 1000.0:.1f} ms) "
                  f"tx_deaf_max={post['tx_deaf_max_us'] / 1000.0:.1f} ms "
                  f"tx_done_to_rearm_max={post['tx_done_to_rearm_max_us']} us "
                  f"(max fields: post-bracket values)")
        else:
            print("RS-12.10: n/a (counters absent in a bracket)")
        # RS-12.15 v2 (2026-09-14): FHSS clock-authority counters. Deltas of
        # the decision histogram and the demotion outcome; streak_max and
        # first_anchor are post-bracket values (first_anchor is cumulative
        # since firmware reset -- report the delta AND the post value).
        rs15 = ("fhss_dec_aligned", "fhss_dec_snapped", "fhss_dec_rej_not_init",
                "fhss_dec_rej_bad_hop", "fhss_dec_rej_epoch_drift",
                "fhss_dec_rej_locked_out", "clk_demotion_reset",
                "clk_demotion_kept", "tx_first_anchor", "tx_stream_streak_max")
        if any(k in post for k in rs15):
            pre15 = {**{k: 0 for k in rs15}, **pre}
            post15 = {**{k: 0 for k in rs15}, **post}
            d15 = {k: post15[k] - pre15[k] for k in rs15}
            print(f"RS-12.15: consider_remote deltas aligned={d15['fhss_dec_aligned']} "
                  f"snapped={d15['fhss_dec_snapped']} locked_out={d15['fhss_dec_rej_locked_out']} "
                  f"epoch_drift={d15['fhss_dec_rej_epoch_drift']} bad_hop={d15['fhss_dec_rej_bad_hop']} "
                  f"not_init={d15['fhss_dec_rej_not_init']} | demotion reset={d15['clk_demotion_reset']} "
                  f"kept={d15['clk_demotion_kept']} | tx_first_anchor delta={d15['tx_first_anchor']} "
                  f"(post {post15['tx_first_anchor']}) | streak_max post={post15['tx_stream_streak_max']}")
        else:
            print("RS-12.15: n/a (counters absent in a bracket)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
