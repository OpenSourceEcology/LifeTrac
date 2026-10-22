#!/usr/bin/env python3
"""rs13_digest_trace.py -- DIGEST trace and A16 confirmation of a VECTOR leg's base capture.

Replays a leg's base capture (the `vector_dry_run.py capture` JSONL,
legs/leg<X>_base.jsonl) through the base station's own parser and
VectorSceneStore -- the same replay as `vector_dry_run.py replay` -- and
traces, per capture row, the encoder's DIGEST n_live against the base's live
count at the check (RS13_RANGE_EDGE_LEG.md, Round A "A16 confirmation"):

* a seq gap between two received frames of one codec is a loss; "loss at
  row" is the first row received after the gap (leg_replay.py's count);
* a resync episode (the dry run's `resync:` episodes) is anchored on the
  DIGEST mismatch run that holds its first row (the run's first BAD DIGEST:
  the store enters resync only RESYNC_DIGEST_RUN mismatches into the run),
  or on its first row when no run holds it; it is opened by the last loss at
  or before that anchor, within --horizon rows -- the same measure as the
  mismatch runs' P1 check below. An episode anchored before the first epoch
  start the store accepts (a key frame, or the repeat-once copy of its anchor
  + LAYER_CLEAR when the key was lost inside the capture, i.e. the copy
  switched the store out of an epoch it already held; a mid-epoch anchor, or
  a copy that opens the capture, is not one: the base still lacks the shapes
  defined before it joined) is the base's join.
  A mismatch run ends at an OK DIGEST and at an epoch switch or epoch start,
  so a join's mismatches never run into a later loss's;
* the loss is isolated when it is one lost frame (a seq gap of one) and no
  other loss falls within --horizon rows after it; a gap of two or more
  frames is a burst, which the rule's single-loss recovery bound does not
  cover, so its episode is not scored;
* an isolated episode PASSES when it ends by DIGEST within --horizon rows of
  its loss, counted in capture rows from the seq gap to the episode's end,
  and its trace shows the base's live count back at the encoder's DIGEST
  n_live within those rows (back in step: the first equal count after the
  last row, up to the episode's end, where the two differ; the loss row
  itself when they never differed), with no later row (to --tail rows past
  the episode, or the next loss) where the encoder counts more live shapes
  than the base. Any other end -- an epoch start, still open, too late --
  FAILS;
* at a leg seq-gap loss above --max-loss (10 %) episodes are listed, not
  scored ("overlapping losses legitimately stretch episodes");
* a loss-opened episode that is not scored still has its whole trace
  printed (from the loss to --tail rows past max(its end, loss + --horizon),
  across the next loss), and it is flagged when it has the RS-13.1 A16
  shape: an epoch start ended it more than --horizon rows after the loss,
  or the encoder still counts more live shapes than the base after the last
  equal count before its end. Its back-in-step cell reads n/a when a burst,
  or the next loss inside the horizon, makes the count meaningless.

    py -3 rs13_digest_trace.py <leg>_base.jsonl [<leg>_base.jsonl ...]
                              [--profile image_bw500] [--horizon 25] [--tail 25]
                              [--rows LO:HI] [--strict]

Prints per capture: the `capture seq gaps` loss of record, the seq-gap rows,
the store line (ttl_dropped, orphans, digests), the final digest_ok with the
last DIGEST checked, the episodes, the DIGEST mismatch runs that start with
no loss in the --horizon rows before them (P1's defect signature), the
RESULTS row per episode (loss at row | episode | ended by | back in step |
isolated? | pass?), then the per-row trace of every episode, or of --rows
(keep it as legs/leg<X>_digest_trace.txt). Over all the captures given: the
loss-opened count (fewer than 3 = thin: pool the Round B VECTOR legs with
loss <= 10 %), the verdict, and a `!! A16 shape` line naming any not-scored
episode with that shape (the verdict covers isolated losses only).

The profile comes from the capture's meta line unless --profile is given;
the two must agree. The store replayed is this checkout's: run it at the
SHA of the base image that flew the leg. Exit 0; 1 when a capture holds no
VECTOR frame, or with --strict when an episode fails, or an episode or a
DIGEST mismatch run has no loss before it (an A16 shape in a not-scored
episode is flagged, not scored, so it does not set the exit code); 2 on
missing or garbled input (any capture line that is not JSON with a ts and
hex, unknown profile, a meta line that is not the capture's first line (two
capture sessions appended into one file), a payload the replay cannot parse, or a
VECTOR body the store rejects), which also withholds the verdict over the
captures.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import statistics
import sys
from pathlib import Path

DC = Path(__file__).resolve().parents[3]       # bench_tools -> helper -> firmware -> DC
if str(DC / "tools") not in sys.path:
    sys.path.insert(0, str(DC / "tools"))
import rs12_leg_report as rep  # noqa: E402
import vector_dry_run as vdr  # noqa: E402  (puts base_station/ on sys.path)
from image_pipeline.frame_format import FRAME_KIND_KEY  # noqa: E402

SEQ_MOD = rep.SEQ_MOD
HORIZON = 25           # rows: the a16_sil.py acceptance figure (recovery <= ~25 frames)
TAIL = 25              # rows checked past the episode for a base that falls short again
MAX_LOSS_PCT = 10.0


class TraceError(Exception):
    """Missing or garbled input (exit 2)."""


def load_capture(path: str) -> tuple[str | None, list[tuple[float, bytes]]]:
    """``(meta profile, [(ts, payload)])`` of a capture JSONL; every non-blank
    line must be the meta line or a payload line, or this raises."""
    if not os.path.isfile(path):
        raise TraceError(f"missing {path}")
    profile, frames, meta_at = None, [], 0
    with open(path, encoding="utf-8") as fh:
        for n, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except ValueError:
                raise TraceError(f"{path} line {n} is not JSON (truncated or garbled)") from None
            if isinstance(obj, dict) and isinstance(obj.get("meta"), dict):
                if meta_at or frames:
                    raise TraceError(
                        f"{path} line {n}: a meta line after "
                        + (f"the one at line {meta_at}" if meta_at else "payload lines")
                        + ": `vector_dry_run.py capture` appends one meta line per session, so "
                        "this file holds two captures (one capture per file)")
                meta_at = n
                profile = obj["meta"].get("profile")
                continue
            if not isinstance(obj, dict) or "hex" not in obj or "ts" not in obj:
                raise TraceError(f"{path} line {n} is neither the meta line nor a payload line")
            try:
                frames.append((float(obj["ts"]), bytes.fromhex(obj["hex"])))
            except (TypeError, ValueError):
                raise TraceError(f"{path} line {n} has a bad ts or hex") from None
    if not frames:
        raise TraceError(f"{path} holds no payload line")
    return profile, frames


def replay(frames: list, profile: str) -> tuple[object, list[dict]]:
    """Feed the capture to a DryRun with a hook on the store's DIGEST check.

    Per row: idx, t (s from the first row), codec, kind, seq, vector, wire,
    records, digest ("ok" / "BAD" / "-"), checks [(encoder n_live, base live
    count at the check)], orph_add, ttl, resync, the live ids the frame
    added / removed, epoch_start (an applied key frame, or the key's
    repeat-once copy -- anchor + LAYER_CLEAR -- when the key was lost inside
    the capture: it switched the store out of an epoch it already held) and
    boundary (epoch_start, or the frame switched the store's epoch)."""
    dr = vdr.DryRun(profile)
    store = dr.store
    cur: list = []
    orig = store._check_digest

    def hook(rec):
        cur.append((rec.n_live, len(store._live())))
        return orig(rec)

    store._check_digest = hook
    rows, prev_ids, po = [], set(), 0
    for ts, payload in frames:
        cur.clear()
        held = store._epoch                       # None until the store applied a VECTOR frame
        r = dr.feed(payload, ts)
        switched = bool(getattr(r, "epoch_switched", False))
        # A lost key's repeat-once copy is an epoch start only when it switched the
        # store out of an epoch it already held (the key's loss is then a seq gap);
        # a copy that opens the store's first epoch (the capture, or the VECTOR run
        # after a codec switch, begins on it) is still the join.
        start = bool(r.vector and getattr(r, "applied", False)
                     and (r.frame_kind == FRAME_KIND_KEY
                          or (switched and held is not None and store._epoch_clear is not None)))
        st = store.stats
        ids = {sh.id for sh in store._live()}
        rows.append({"idx": r.idx, "t": r.t_rel, "parsed": r.parsed, "codec": r.codec,
                     "kind": r.frame_kind, "seq": r.seq, "vector": r.vector, "wire": r.wire,
                     "records": r.records, "digest": r.digest, "checks": list(cur),
                     "orph_add": st["orphans"] - po, "ttl": st["ttl_dropped"],
                     "resync": bool(st["resync"]), "added": sorted(ids - prev_ids),
                     "removed": sorted(prev_ids - ids), "epoch_start": start,
                     "boundary": start or switched})
        po, prev_ids = st["orphans"], ids
    return dr, rows


def loss_rows(rows: list[dict]) -> tuple[list[tuple[int, int]], int]:
    """``[(row, frames lost)]`` per seq gap, and the jump count.

    The same rules as rs12_leg_report.seq_gap_loss (the loss of record):
    runs of one codec, d = 0 a duplicate, whole wraps added back from the
    arrival times, a step of more than half the seq space that the elapsed
    time cannot explain a jump (counter restart), not loss. The row is the
    first frame received after the gap."""
    parsed = [r for r in rows if r["codec"] is not None and r["seq"] is not None]
    steps = [b["t"] - a["t"] for a, b in zip(parsed, parsed[1:])
             if a["codec"] == b["codec"] and (b["seq"] - a["seq"]) % SEQ_MOD == 1]
    period = statistics.median(steps) if steps else None
    gaps, jumps, last = [], 0, None
    for r in parsed:
        if last is None or last["codec"] != r["codec"]:
            last = r
            continue
        d = (r["seq"] - last["seq"]) % SEQ_MOD
        if d == 0:
            continue
        if period:
            elapsed = (r["t"] - last["t"]) / period
            jump = d > SEQ_MOD // 2 and elapsed < d / 2
            wraps = max(0, round((elapsed - d) / SEQ_MOD))
        else:
            jump, wraps = d > SEQ_MOD // 2, 0
        if jump:
            jumps += 1
        else:
            missing = d - 1 + SEQ_MOD * wraps
            if missing:
                gaps.append((r["idx"], missing))
        last = r
    return gaps, jumps


def _check(row: dict):
    """The row's last DIGEST check (enc n_live, base count), or None."""
    return row["checks"][-1] if row["checks"] else None


def join_row(rows: list[dict]) -> int | None:
    """The first epoch start the store accepted: the base's join ends there
    (a key frame, or its repeat-once copy when the key was lost). Hand-built
    rows without the replay's ``epoch_start`` field fall back to the first
    key frame."""
    if rows and "epoch_start" in rows[0]:
        return next((r["idx"] for r in rows if r["epoch_start"]), None)
    return next((r["idx"] for r in rows if r["vector"] and r["kind"] == FRAME_KIND_KEY), None)


def mismatch_runs(rows: list[dict], gaps: list, join: int | None,
                  horizon: int = HORIZON) -> list[tuple[int, int, bool]]:
    """Runs of BAD DIGESTs: ``(first row, last row, explained)``. A run ends
    at an OK DIGEST and before a ``boundary`` row (an epoch switch or epoch
    start: its check follows the reset); rows
    with no DIGEST do not break it. explained = the run starts within
    ``horizon`` rows after a seq gap, or before ``join`` (join_row: the
    base's join)."""
    loss = [g[0] for g in gaps]
    runs, cur = [], None
    for r in rows:
        if r.get("boundary") and cur is not None:
            runs.append(cur)
            cur = None
        if r["digest"] == "BAD":
            cur = [r["idx"], r["idx"]] if cur is None else [cur[0], r["idx"]]
        elif r["digest"] == "ok" and cur is not None:
            runs.append(cur)
            cur = None
    if cur is not None:
        runs.append(cur)
    out = []
    for a, b in runs:
        joined = join is None or a < join
        out.append((a, b, joined or any(0 <= a - L <= horizon for L in loss)))
    return out


def _a16_shape(e: dict, L: int, by_idx: dict, last_row: int, horizon: int) -> str | None:
    """Why a loss-opened episode has the RS-13.1 A16 shape, or None: an epoch
    start ended it more than ``horizon`` rows after the loss, or the encoder
    counts more live shapes than the base on a DIGEST after the last equal
    count before the episode's end (a missing shape that the repair never
    restored). The end row itself is left out: an epoch start's check
    follows the reset."""
    why = []
    if e["end"] is not None and e["ended_by"] == "epoch start" and e["end"] - L > horizon:
        why.append(f"an epoch start ended it {e['end'] - L} rows after the loss")
    stop = e["end"] if e["end"] is not None else last_row + 1
    checks = [(i, _check(by_idx[i])) for i in range(L, stop) if i in by_idx and _check(by_idx[i])]
    last_eq = max((i for i, c in checks if c[0] == c[1]), default=L - 1)
    short = [i for i, c in checks if i > last_eq and c[0] > c[1]]
    if short:
        why.append(f"the encoder counts more live shapes than the base at {len(short)} DIGEST "
                   f"row(s) #{short[0]}-#{short[-1]} after the last equal count (row "
                   f"{last_eq if last_eq >= L else 'none'}), up to the episode's end")
    return "; ".join(why) or None


def score_episodes(rows: list[dict], episodes: list[dict], gaps: list, *,
                   horizon: int = HORIZON, tail: int = TAIL,
                   loss_pct: float = 0.0, max_loss_pct: float = MAX_LOSS_PCT,
                   runs: list | None = None) -> list[dict]:
    """The A16 rule per resync episode. Adds to a copy of each episode:
    anchor (the first row of the DIGEST mismatch run that holds the
    episode's first row, else that row: the loss is looked for before it,
    as mismatch_runs does for P1; ``runs`` defaults to mismatch_runs'),
    loss_row, isolated, next_loss, window (first, last row of the trace: to
    ``tail`` rows past max(end, loss + horizon), not cut at the next loss),
    back_row (isolated episodes only, searched up to the next loss),
    a16_shape (_a16_shape, for every loss-opened episode), status ("PASS",
    "FAIL", "not scored", "join", "no loss"), reasons, and letter_only: a
    FAIL whose one reason is an epoch start that ended the episode within
    the horizon (the rule's first sentence asks for a DIGEST end; its second
    fails only an epoch start past the horizon)."""
    loss = [g[0] for g in gaps]
    lost = dict(gaps)                                 # row -> frames lost in its gap
    last_row = rows[-1]["idx"]
    join = join_row(rows)
    if runs is None:
        runs = mismatch_runs(rows, gaps, join, horizon)
    by_idx = {r["idx"]: r for r in rows}
    out, prev_loss = [], None
    for e in episodes:
        res = dict(e, anchor=None, loss_row=None, isolated=None, next_loss=None, window=None,
                   back_row=None, a16_shape=None, reasons=[], reentry=False, letter_only=False,
                   burst=None, near_loss=False)
        out.append(res)
        anchor = next((a for a, b, _ in runs if a <= e["start"] <= b), e["start"])
        res["anchor"] = anchor
        if join is None or anchor < join:
            res["status"] = "join"
            res["reasons"].append(
                ("starts" if anchor == e["start"] else f"its DIGEST mismatch run starts (row "
                 f"{anchor})") + " before the first epoch start in the capture ("
                + (f"row {join}" if join is not None else "none") + ": the base joining "
                "mid-epoch)")
            continue
        prior = [L for L in loss if L <= anchor]
        if not prior or anchor - prior[-1] > horizon:
            res["status"] = "no loss"
            res["reasons"].append(
                f"no seq gap in the {horizon} rows before "
                + (f"row {anchor}, where its DIGEST mismatch run starts" if anchor != e["start"]
                   else "it") + ": a mismatch with no loss before it fails P1 "
                "(encoder/store defect)")
            continue
        L = prior[-1]
        nxt = next((x for x in loss if x > L), None)
        burst = lost.get(L, 1) if lost.get(L, 1) > 1 else None
        near = nxt is not None and nxt - L <= horizon
        isolated = burst is None and not near
        res.update(loss_row=L, next_loss=nxt, isolated=isolated, burst=burst, near_loss=near,
                   reentry=prev_loss == L)
        prev_loss = L
        end = e["end"] if e["end"] is not None else last_row
        trace_end = min(last_row, max(end, L + horizon) + tail)
        res["window"] = (L, trace_end)
        res["a16_shape"] = _a16_shape(e, L, by_idx, last_row, horizon)
        back, short_later, w_end = None, [], trace_end
        if isolated:
            # Scored up to the next loss only: a later loss may legitimately
            # put the base out of step again.
            if nxt is not None:
                w_end = min(trace_end, nxt - 1)
            win = [by_idx[i] for i in range(L, w_end + 1) if i in by_idx]
            # Back in step: the first equal count after the last row, up to the
            # episode's end, where the counts differ (a ghost or a missing shape);
            # L itself when they never differed (a lost UPD: only the crc moved).
            end_b = e["end"] if e["end"] is not None else w_end
            differ = [r["idx"] for r in win if r["idx"] <= end_b and _check(r)
                      and _check(r)[0] != _check(r)[1]]
            after = differ[-1] if differ else L - 1
            back = next((r["idx"] for r in win if r["idx"] > after and _check(r)
                         and _check(r)[0] == _check(r)[1]), None)
            short_later = [r["idx"] for r in win if back is not None and r["idx"] > back
                           and _check(r) and _check(r)[0] > _check(r)[1]]
            res["back_row"] = back
        if loss_pct > max_loss_pct:
            res["status"] = "not scored"
            res["reasons"].append(f"leg seq-gap loss {loss_pct:.1f} % > {max_loss_pct:g} %")
            continue
        if not res["isolated"]:
            res["status"] = "not scored"
            why = []
            if burst:
                why.append(f"{burst} frames lost in one gap (a burst; the rule bounds the "
                           f"recovery from one lost frame)")
            if near:
                why.append(f"another loss at row {nxt}, {nxt - L} rows after it")
            res["reasons"].append("not isolated: " + "; ".join(why))
            continue
        if e["end"] is None and last_row - L <= horizon:
            res["status"] = "not scored"
            res["reasons"].append(f"still open when the capture ends, {last_row - L} rows "
                                  f"after the loss (within the horizon)")
            continue
        if e["end"] is None:
            res["reasons"].append(f"still in resync at the capture end, {last_row - L} rows "
                                  f"after the loss")
        else:
            if e["ended_by"] != "digest":
                res["reasons"].append(f"ended by {e['ended_by']}, not by DIGEST")
                res["letter_only"] = e["end"] - L <= horizon
            if e["end"] - L > horizon:
                res["reasons"].append(f"ended {e['end'] - L} rows after the loss (> {horizon})")
        if back is None:
            res["reasons"].append(f"the base's live count does not get back to the encoder's "
                                  f"DIGEST n_live in trace rows {L}-{w_end}")
        elif back - L > horizon:
            res["reasons"].append(f"back in step {back - L} rows after the loss (> {horizon})")
        if short_later:
            res["reasons"].append(f"the encoder counts more live shapes than the base again "
                                  f"after it, at row(s) {short_later[:8]}")
        res["status"] = "FAIL" if res["reasons"] else "PASS"
        res["letter_only"] = res["letter_only"] and len(res["reasons"]) == 1
    return out


def leg_name(path: str) -> str:
    base = os.path.basename(path)
    m = re.match(r"leg(.+?)_base\.jsonl$", base)
    return m.group(1) if m else os.path.splitext(base)[0]


def format_row(r: dict, marks: dict) -> str:
    if not r["parsed"]:
        line = f"#{r['idx']:3d} UNPARSEABLE {r['wire']:3d}B"
    else:
        c = _check(r)
        line = (f"#{r['idx']:3d} seq={r['seq']:3d} kind={r['kind']} {r['wire']:3d}B "
                f"rec={r['records']:2d} dig={r['digest']:3s} enc/base n_live={c} "
                f"orph+{r['orph_add']} ttl={r['ttl']} rs={int(r['resync'])} "
                f"+{r['added']} -{r['removed']}")
        if not r["vector"]:
            line += f" (codec {vdr.CODEC_NAMES.get(r['codec'], r['codec'])})"
    note = marks.get(r["idx"])
    return line + (f"  <- {'; '.join(note)}" if note else "")


def _marks(rows, gaps, episodes) -> dict:
    marks: dict = {}
    for i, n in gaps:
        marks.setdefault(i, []).append(f"loss: {n} frame(s) lost before this row")
    for e in episodes:
        marks.setdefault(e["start"], []).append("resync in")
        if e["end"] is not None:
            marks.setdefault(e["end"], []).append(f"resync out ({e['ended_by']})")
    for r in rows:
        if r.get("epoch_start"):
            marks.setdefault(r["idx"], []).append(
                "epoch start" if r["kind"] == FRAME_KIND_KEY
                else "epoch start (repeat-once copy of a lost key)")
        elif r.get("boundary"):
            marks.setdefault(r["idx"], []).append("epoch switch")
        elif "epoch_start" not in r and r["vector"] and r["kind"] == FRAME_KIND_KEY:
            marks.setdefault(r["idx"], []).append("epoch start")
    return marks


def _episode_cell(e: dict) -> str:
    if e["end"] is None:
        return f"#{e['start']}-end ({e['frames']}, open)"
    return f"#{e['start']}-#{e['end']} ({e['frames']}, {e['t_end_s'] - e['t_start_s']:.1f} s)"


def _status_cell(e: dict) -> str:
    if e["status"] in ("PASS",):
        return "PASS"
    head = {"FAIL": "FAIL", "not scored": "not scored", "join": "join (not scored)",
            "no loss": "NO LOSS BEFORE IT"}[e["status"]]
    if e["letter_only"]:
        head = (f"FAIL by the letter (an epoch start {e['end'] - e['loss_row']} rows after the "
                f"loss ended it before a DIGEST end could show)")
    shape = (f"; !! A16 shape: {e['a16_shape']}" if e["status"] == "not scored"
             and e["a16_shape"] else "")
    return f"{head}: {'; '.join(e['reasons'])}{shape}"


def analyse(path: str, profile_arg: str | None, horizon: int, tail: int,
            max_loss: float) -> dict:
    meta_profile, frames = load_capture(path)
    if profile_arg and meta_profile and profile_arg != meta_profile:
        raise TraceError(f"--profile {profile_arg} but the capture was taken with "
                         f"--profile {meta_profile}")
    profile = profile_arg or meta_profile
    if profile is None:
        raise TraceError(f"{path} has no meta profile: give --profile image_bw500|image_bw250")
    if profile not in vdr.PROFILE_CHOICES:
        raise TraceError(f"unknown profile {profile!r} (one of {', '.join(vdr.PROFILE_CHOICES)})")
    dr, rows = replay(frames, profile)
    bad = dr.store.stats["frames_bad"]
    if dr.parse_errors or bad:
        why = dr.store.stats.get("bad_reasons")
        why = f" {why}" if why else ""
        raise TraceError(f"{path}: {dr.parse_errors} payload(s) the replay cannot parse, {bad} "
                         f"VECTOR body(ies) the store rejected{why}: a corrupt capture, or a "
                         f"store at another SHA than the base image that flew the leg; not scored")
    gaps, jumps = loss_rows(rows)
    seq = rep.seq_gap_loss([(r["t"], r["codec"], r["seq"]) for r in rows])
    if sum(n for _, n in gaps) != seq["lost"]:      # the two must count the same frames
        raise TraceError(f"internal: seq-gap rows sum to {sum(n for _, n in gaps)} but "
                         f"rs12_leg_report counts {seq['lost']}")
    loss_pct = 100.0 * seq["lost"] / seq["expected"] if seq["expected"] else 0.0
    runs = mismatch_runs(rows, gaps, join_row(rows), horizon)
    eps = score_episodes(rows, dr.resync_episodes(), gaps, horizon=horizon, tail=tail,
                         loss_pct=loss_pct, max_loss_pct=max_loss, runs=runs)
    return {"path": path, "leg": leg_name(path), "profile": profile, "dr": dr, "rows": rows,
            "gaps": gaps, "jumps": jumps, "seq": seq, "loss_pct": loss_pct, "episodes": eps,
            "runs": runs, "vector": sum(1 for r in rows if r["vector"])}


def report(res: dict, horizon: int, rows_range: tuple[int, int] | None) -> list[str]:
    rows, eps = res["rows"], res["episodes"]
    out = [f"== rs13_digest_trace: {os.path.basename(res['path'])} (leg {res['leg']}, profile "
           f"{res['profile']}, {len(rows)} rows, {res['vector']} vector; horizon {horizon} "
           f"rows) =="]
    out += rep.format_seq_loss(res["seq"], os.path.basename(res["path"]), vdr.CODEC_NAMES)
    out.append(f"seq gaps (row, frames lost): {res['gaps']}"
               + (f"; {res['jumps']} jump(s) not counted" if res["jumps"] else ""))
    s = res["dr"].summary()
    st, sc = s["store"], s["scene"]
    last_d = next(((r["idx"], r["digest"]) for r in reversed(rows) if r["digest"] in ("ok", "BAD")),
                  None)
    out.append(f"store: ttl_dropped {st['ttl_dropped']}, orphans {st['orphans']}, digest "
               f"{st['digest_checks']} checked / {st['digest_mismatch']} mismatched; final "
               f"digest_ok {sc.get('digest_ok') if sc else None}; last DIGEST checked "
               f"(row, result): {last_d}")
    out.append(vdr.format_episodes(s["resync_episodes"]))
    bad = [(a, b) for a, b, ok in res["runs"] if not ok]
    out.append(f"DIGEST mismatch runs: {len(res['runs'])}"
               + (f"; !! {len(bad)} start with no seq gap in the {horizon} rows before them and "
                  f"after the join: rows {bad} (P1: a mismatch with no loss before it)" if bad
                  else " (each starts within the horizon after a seq gap, or at the join)"))
    out.append("")
    out.append("| leg | loss at row | episode (start-end, frames, s) | ended by | back in step "
               "(digest_trace, frames after the loss) | isolated? | pass? |")
    out.append("|---|---|---|---|---|---|---|")
    for e in eps:
        L = e["loss_row"]
        burst = [f"burst of {e['burst']} frames"] if e["burst"] else []
        back = ("-" if e["window"] is None
                else "n/a (" + ", ".join(burst + ([f"next loss at row {e['next_loss']}"]
                                                  if e["near_loss"] else [])) + ")"
                if not e["isolated"]
                else "never" if e["back_row"] is None
                else f"#{e['back_row']} (+{e['back_row'] - L})")
        iso = ("-" if e["isolated"] is None else "yes" if e["isolated"]
               else "no (" + ", ".join(burst + ([f"row {e['next_loss']}"] if e["near_loss"]
                                                else [])) + ")")
        ended = e["ended_by"] or "open"
        out.append(f"| {res['leg']} | {L if L is not None else '-'} | {_episode_cell(e)}"
                   f"{' re-entry' if e['reentry'] else ''} | {ended} | {back} | {iso} | "
                   f"{_status_cell(e)} |")
    if not eps:
        out.append(f"| {res['leg']} | - | no resync episode | - | - | - | - |")
    out.append("")
    marks = _marks(rows, res["gaps"], res["dr"].resync_episodes())
    if rows_range:
        lo, hi = rows_range
        out.append(f"# trace rows {lo}-{hi}: enc/base n_live = the DIGEST record's n_live vs the "
                   f"base store's live count at the check; orph+ = orphans added by the frame; "
                   f"ttl = base ttl_dropped running total")
        out += [format_row(r, marks) for r in rows if lo <= r["idx"] <= hi]
    else:
        for e in eps:
            if e["window"] is None:
                continue
            a, b = e["window"]
            out.append(f"# episode {_episode_cell(e)} opened by the loss at row {e['loss_row']}: "
                       f"trace rows {a}-{b} (enc/base n_live = the DIGEST record's n_live vs the "
                       f"base store's live count at the check)")
            out += [format_row(r, marks) for r in rows if a <= r["idx"] <= b]
    return out


def _rows_arg(text: str) -> tuple[int, int]:
    m = re.fullmatch(r"(\d+):(\d+)", text)
    if not m or int(m.group(1)) > int(m.group(2)):
        raise argparse.ArgumentTypeError("LO:HI with LO <= HI, e.g. 520:595")
    return int(m.group(1)), int(m.group(2))


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("capture", nargs="+", help="base capture JSONL(s): legs/leg<X>_base.jsonl")
    ap.add_argument("--profile", choices=vdr.PROFILE_CHOICES,
                    help="default: the capture's meta line")
    ap.add_argument("--horizon", type=int, default=HORIZON,
                    help=f"rows from the loss: isolation, the DIGEST end and back in step "
                         f"(default {HORIZON})")
    ap.add_argument("--tail", type=int, default=TAIL,
                    help=f"rows traced past the episode for a base that falls short again "
                         f"(default {TAIL})")
    ap.add_argument("--max-loss", type=float, default=MAX_LOSS_PCT,
                    help=f"leg seq-gap loss %% above which episodes are not scored "
                         f"(default {MAX_LOSS_PCT:g})")
    ap.add_argument("--rows", type=_rows_arg,
                    help="trace these rows (LO:HI) instead of the episodes")
    ap.add_argument("--strict", action="store_true",
                    help="exit 1 when an episode fails, or an episode or a DIGEST mismatch "
                         "run has no loss before it")
    a = ap.parse_args(argv)
    if a.horizon < 1 or a.tail < 0:
        ap.error("--horizon must be >= 1 and --tail >= 0")
    results, rc = [], 0
    for path in a.capture:
        try:
            res = analyse(path, a.profile, a.horizon, a.tail, a.max_loss)
        except TraceError as exc:
            print(f"{os.path.basename(path)}: CANNOT TRACE: {exc}")
            rc = 2
            continue
        if res["vector"] == 0:
            print(f"{os.path.basename(path)}: no VECTOR frame in {len(res['rows'])} rows "
                  f"(a mono_g4 leg has no DIGEST to trace)")
            rc = max(rc, 1)
            continue
        print("\n".join(report(res, a.horizon, a.rows)))
        print()
        results.append(res)
    if rc == 2 and results:
        print("A16 confirmation withheld: a capture above could not be traced")
    if rc == 2 or not results:
        return rc
    eps = [e for r in results for e in r["episodes"]]
    opened = [e for e in eps if e["loss_row"] is not None]
    scored = [e for e in eps if e["status"] in ("PASS", "FAIL")]
    fails = [e for e in scored if e["status"] == "FAIL"]
    noloss = [e for e in eps if e["status"] == "no loss"]
    p1 = sum(1 for r in results for _, _, ok in r["runs"] if not ok)
    thin = (" -- THIN: fewer than 3; pool the Round B VECTOR legs with seq-gap loss <= 10 %"
            if len(opened) < 3 else "")
    letter = sum(1 for e in fails if e["letter_only"])
    verdict = ("FAIL (re-opens A16)" if len(fails) > letter else
               "FAIL BY THE LETTER ONLY (epoch starts within the horizon; see the rule's "
               "second sentence)" if fails else "NO SCORED EPISODE" if not scored else "PASS")
    shaped = [(r["leg"], e) for r in results for e in r["episodes"]
              if e["status"] == "not scored" and e["a16_shape"]]
    print(f"A16 confirmation over {len(results)} capture(s) "
          f"({', '.join(r['leg'] for r in results)}): {len(eps)} episode(s); loss-opened "
          f"{len(opened)}{thin}; isolated and scored {len(scored)}: "
          f"{len(scored) - len(fails)} PASS, {len(fails)} FAIL"
          + (f" ({letter} by the letter only)" if letter else "") + f" -> {verdict}"
          + (f" (!! A16 shape in {len(shaped)} not-scored episode(s), below)" if shaped else ""))
    if shaped:
        print("!! A16 shape in not-scored episode(s): "
              + "; ".join(f"{leg} {_episode_cell(e).split(' (')[0]} (loss at row "
                          f"{e['loss_row']}: {e['a16_shape']})" for leg, e in shaped)
              + ". The rule scores isolated losses only: read their traces before quoting "
                "the verdict")
    if noloss or p1:
        print(f"!! P1: {len(noloss)} episode(s) and {p1} DIGEST mismatch run(s) with no loss "
              f"before them (encoder/store defect signature)")
    if a.strict and (fails or noloss or p1):
        rc = max(rc, 1)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
