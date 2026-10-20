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
    logged, duplicates kept), ``totals`` (set), ``t`` (first event's time)."""
    trains: list[dict] = []
    last_t = None
    for t, seq, idx, total in events:
        same = (trains and trains[-1]["seq"] == seq
                and not (t is not None and last_t is not None
                         and t - last_t > TRAIN_SPLIT_S))
        if not same:
            trains.append({"seq": seq, "idxs": set(), "idx_list": [],
                           "totals": set(), "t": t})
        tr = trains[-1]
        tr["idxs"].add(idx)
        tr["idx_list"].append(idx)
        if total is not None:
            tr["totals"].add(total)
        if t is not None:
            last_t = t
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


def seq_gap_loss(frames) -> dict:
    """Frame loss from TileDeltaFrame sequence numbers (RS-13.1 A14).

    ``frames``: one ``(ts, codec, seq)`` per captured payload, in arrival
    order; ``codec`` and ``seq`` are None for a payload that did not parse.

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
            "gaps": sum(r["gaps"] for r in runs),
            "longest": max((r["longest"] for r in runs), default=0),
            "duplicates": sum(r["duplicates"] for r in runs),
            "jumps": sum(r["jumps"] for r in runs),
            "unparseable": len(frames) - len(parsed),
            "first_seq": runs[0]["first_seq"] if runs else None}


def format_seq_loss(res: dict, label: str,
                    names: dict | None = None) -> list[str]:
    """Report lines for :func:`seq_gap_loss`."""
    names = names or {}
    if not res["runs"]:
        return [f"capture seq gaps ({label}): no parseable frames "
                f"({res['unparseable']} unparseable)"]
    exp, lost = res["expected"], res["lost"]
    n_runs = len(res["runs"])
    period = f"; period {res['period_s']:.2f} s" if res["period_s"] else ""
    out = [f"capture seq gaps ({label}): lost {lost}/{exp} = "
           f"{100 * lost / exp:.1f}%  (received {res['received']}, "
           f"{res['gaps']} gap{'' if res['gaps'] == 1 else 's'}, "
           f"longest {res['longest']}; "
           f"{n_runs} codec run{'s' if n_runs > 1 else ''}{period})"]
    head = res["first_seq"] - 1
    if head > 0:
        out.append(
            f"  first frame heard at seq {res['first_seq']}: the {head} "
            f"frame(s) numbered before it are not in the figure above (the "
            f"camera numbers from seq 1 at start-up — an acquisition delay "
            f"or a late capture, RS-13.1 A7); counting from seq 1: "
            f"{lost + head}/{exp + head} = "
            f"{100 * (lost + head) / (exp + head):.1f}%")
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
    args = ap.parse_args(argv)

    rx = (args.archive / "rx_daemon.log").read_text(encoding="utf-8",
                                                    errors="replace")
    tx = (args.archive / "tx_daemon.log").read_text(encoding="utf-8",
                                                    errors="replace")
    params = (args.archive / "params.txt").read_text(encoding="utf-8",
                                                     errors="replace")

    for key in ("train_gap_ms", "tx_pipeline_depth", "synth_budget_b",
                "force_frf_hz", "git_sha", "log_window"):
        m = re.search(rf"{key}=(\S+)", params)
        if m:
            print(f"{key}={m.group(1)}", end="  ")
    print()
    # 2026-10-19: every harness up to the 10-19 legs read the daemon logs
    # BEFORE it stopped the daemons (TX log first, then RX), while the post
    # brackets are read after; a harness that reads them after the stop says
    # so with log_window=after_stop in params.txt.
    m = re.search(r"log_window=(\S+)", params)
    logs_after_stop = bool(m) and m.group(1) == "after_stop"

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
        for line in format_seq_loss(seq_gap_loss(frames), args.capture.name,
                                    names):
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
        if not logs_after_stop:
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
