#!/usr/bin/env python3
"""tools/vector_replay_publish.py — RS-13.2 desk-check tool: replay a capture onto a broker.

Reads a ``vector_dry_run.py capture`` JSONL file (one payload per line, see
below) and publishes every payload to an MQTT topic at the capture's own
inter-frame timing, so a web_ui subscribed to that broker renders the leg
again as the operator would have seen it. Optional drop injection removes
chosen frames before they reach the broker; the receiver then sees a
``base_seq`` gap exactly as it would after a frame lost on air, which is how
the desk check reproduces A16 (a lost frame carrying a DEL, RS-13.1 RESULTS
A16) on screen.

Nothing here touches a radio. The tool talks only to the broker named by
``--host``/``--port`` and never retains a message (an image frame must not
outlive the broker session). It is stdlib + paho-mqtt only, so it also runs
inside the ``lifetrac-v25`` image, whose PYTHONPATH carries no ``tools/``.

Usage::

    python tools/vector_replay_publish.py legs/leg2a_r4_base.jsonl
    python tools/vector_replay_publish.py legs/leg2b_r4_base.jsonl \\
        --host 127.0.0.1 --port 1883 --speed 1 --loop
    python tools/vector_replay_publish.py legs/leg2b_r4_base.jsonl \\
        --drop-rows 200 --log desk/2b_drop200.jsonl
    python tools/vector_replay_publish.py legs/leg2a_r4_base.jsonl \\
        --start-row 500 --start-at-key --end-row 600 --dry-run

Capture line (written by ``vector_dry_run.py capture``)::

    {"ts": 1727345678.123, "topic": "...", "hex": "<payload>"}

The first line is usually ``{"meta": {...}}``; meta, blank and non-JSON lines
are skipped, as ``vector_dry_run.iter_capture`` skips them. Rows are numbered
from 0 over the payload lines only, the same ``#idx`` the dry-run report
prints and RS-13.1 RESULTS quotes ("rows 525-589"), so a row named in a
report can be dropped or windowed directly.

Timing: frame *i* is published ``(ts[i] - ts[first]) / speed`` seconds after
the pass starts, on an absolute schedule, so a slow publish never accumulates
drift. ``--max-gap`` caps any single inter-frame wait (before the speed
division) and a negative gap (a clock step in the capture) counts as 0. A
dropped frame keeps its slot: the next frame goes out at its original time,
as it would after an air loss.

Drops: ``--drop-rows`` names capture rows (``100,200-202``);
``--drop-every N`` drops the Nth, 2Nth, ... frame replayed in each pass,
counted from the first row of the window. Both may be combined.

Exit status: 0; 1 when the broker cannot be reached; 2 on a usage error.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import struct
import sys
import time
from dataclasses import dataclass, field
from typing import Callable, Iterable, Optional

BASE_TOPIC = "lifetrac/v25/video/tile_delta"
DEFAULT_LOOP_GAP_S = 4.0     # > the store's 3 s outage rule (VECTOR_SCENE.md §3.5 rule 4)

# TileDeltaFrame fixed header (base_station/image_pipeline/frame_format.py,
# HEADER_FIXED_LEN): frame_kind, base_seq, grid_w, grid_h, tile_px, codec.
_HDR = struct.Struct("BBBBBB")
FRAME_KIND_KEY = 1
CODEC_NAMES = {0: "webp", 1: "mono_g4", 2: "btc4_tile", 3: "btc4_frame",
               4: "webp_luma", 5: "rawstream", 6: "vector"}


@dataclass(frozen=True)
class Row:
    idx: int          # 0-based over payload lines, as vector_dry_run numbers them
    ts: float         # capture wall time (s)
    topic: str        # topic the payload was captured on (informational)
    payload: bytes


@dataclass(frozen=True)
class Step:
    row: Row
    offset_s: float   # seconds after the pass starts
    drop: bool


@dataclass
class ReplayStats:
    passes: int = 0
    published: int = 0
    dropped: int = 0
    dropped_rows: list = field(default_factory=list)   # (pass, row) per drop
    interrupted: bool = False


# -- capture -------------------------------------------------------------------
def read_capture(path) -> list:
    """Every payload line of a capture as a :class:`Row`, in file order."""
    rows: list = []
    with open(path, "r", encoding="utf-8") as fh:
        for n, line in enumerate(fh, 1):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except ValueError:
                print(f"warning: line {n} is not JSON, skipped", file=sys.stderr)
                continue
            if not isinstance(obj, dict) or "hex" not in obj:
                continue
            try:
                payload = bytes.fromhex(obj["hex"])
                ts = float(obj.get("ts", 0.0))
            except (TypeError, ValueError):
                print(f"warning: line {n} has a bad ts or hex, skipped", file=sys.stderr)
                continue
            rows.append(Row(len(rows), ts, str(obj.get("topic", "")), payload))
    return rows


def peek_header(payload: bytes) -> Optional[dict]:
    """The 6-byte TileDeltaFrame header, without the base_station parser."""
    if len(payload) < _HDR.size:
        return None
    kind, seq, gw, gh, px, codec = _HDR.unpack_from(payload, 0)
    return {"kind": kind, "seq": seq, "grid": (gw, gh, px), "codec": codec}


def parse_row_spec(spec: str) -> frozenset:
    """``"100,200-202"`` -> {100, 200, 201, 202}. Raises ValueError on junk."""
    out: set = set()
    for part in (spec or "").split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            lo, hi = int(a), int(b)
            if lo < 0 or hi < lo:
                raise ValueError(f"bad row range {part!r}")
            out.update(range(lo, hi + 1))
        else:
            v = int(part)
            if v < 0:
                raise ValueError(f"bad row {part!r}")
            out.add(v)
    return frozenset(out)


# -- plan ------------------------------------------------------------------------
def plan(rows: list, *, speed: float = 1.0, max_gap: float = 0.0, drop_every: int = 0,
         drop_rows: Iterable = (), start_row: Optional[int] = None,
         end_row: Optional[int] = None, start_at_key: bool = False) -> list:
    """One pass as a list of :class:`Step`: the window, each frame's offset from
    the pass start, and whether it is dropped. Pure; no clock, no broker."""
    if speed <= 0:
        raise ValueError("speed must be > 0")
    if max_gap < 0:
        raise ValueError("max_gap must be >= 0")
    if drop_every < 0:
        raise ValueError("drop_every must be >= 0")
    lo = 0 if start_row is None else int(start_row)
    hi = (len(rows) - 1) if end_row is None else int(end_row)
    if start_at_key and 0 < lo < len(rows):
        # back up to the last epoch start (K = 1) at or before the window start,
        # so the store gets an anchor before the rows of interest
        k = lo
        while k > 0:
            h = peek_header(rows[k].payload)
            if h is not None and h["kind"] == FRAME_KIND_KEY:
                break
            k -= 1
        lo = max(0, k)
    window = [r for r in rows if lo <= r.idx <= hi]
    drops = frozenset(drop_rows)
    steps: list = []
    offset = 0.0
    prev: Optional[Row] = None
    for n, r in enumerate(window, 1):
        if prev is not None:
            gap = max(0.0, r.ts - prev.ts)
            if max_gap > 0:
                gap = min(gap, max_gap)
            offset += gap / speed
        drop = (r.idx in drops) or (drop_every > 0 and n % drop_every == 0)
        steps.append(Step(r, offset, drop))
        prev = r
    return steps


# -- publishers --------------------------------------------------------------------
class PahoPublisher:
    """paho-mqtt 1.x or 2.x; QoS 0, never retained."""

    def __init__(self, host: str, port: int, client_id: str, qos: int = 0) -> None:
        import paho.mqtt.client as mqtt                    # type: ignore
        api = getattr(mqtt, "CallbackAPIVersion", None)    # paho 2.x
        self._client = (mqtt.Client(api.VERSION2, client_id=client_id) if api is not None
                        else mqtt.Client(client_id=client_id))
        self._qos = int(qos)
        self._last = None
        self._client.connect(host, int(port), keepalive=30)
        self._client.loop_start()

    def publish(self, topic: str, payload: bytes) -> None:
        self._last = self._client.publish(topic, payload, qos=self._qos, retain=False)

    def close(self) -> None:
        try:
            if self._last is not None:
                self._last.wait_for_publish(timeout=2.0)
        except Exception:                                    # noqa: BLE001
            pass
        self._client.loop_stop()
        try:
            self._client.disconnect()
        except Exception:                                    # noqa: BLE001
            pass


class NullPublisher:
    """``--dry-run``: publishes nothing."""

    def publish(self, topic: str, payload: bytes) -> None:
        pass

    def close(self) -> None:
        pass


# -- replay ------------------------------------------------------------------------
def replay(steps: list, publisher, topic: str, *, passes: int = 1, loop: bool = False,
           loop_gap_s: float = DEFAULT_LOOP_GAP_S,
           clock: Callable[[], float] = time.monotonic,
           sleep: Callable[[float], None] = time.sleep,
           on_step: Optional[Callable[[int, Step, float], None]] = None,
           should_stop: Callable[[], bool] = lambda: False) -> ReplayStats:
    """Publish ``steps`` on their schedule, ``passes`` times (forever with
    ``loop``), waiting ``loop_gap_s`` between passes. ``on_step(pass, step,
    t_rel)`` sees every step, dropped or not; ``t_rel`` is the clock offset from
    the pass start when it was handled. Ctrl-C ends the replay cleanly."""
    st = ReplayStats()
    if not steps:
        return st
    p = 0
    try:
        while loop or p < passes:
            if should_stop():
                break
            if p > 0 and loop_gap_s > 0:
                sleep(loop_gap_s)
            p += 1
            st.passes = p
            t0 = clock()
            for s in steps:
                if should_stop():
                    return st
                wait = (t0 + s.offset_s) - clock()
                if wait > 0:
                    sleep(wait)
                if s.drop:
                    st.dropped += 1
                    st.dropped_rows.append((p, s.row.idx))
                else:
                    publisher.publish(topic, s.row.payload)
                    st.published += 1
                if on_step is not None:
                    on_step(p, s, clock() - t0)
    except KeyboardInterrupt:
        st.interrupted = True
    return st


# -- formatting ----------------------------------------------------------------------
def _utc_now() -> str:
    return _dt.datetime.now(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def describe(s: Step) -> dict:
    h = peek_header(s.row.payload) or {}
    return {"row": s.row.idx, "offset_s": round(s.offset_s, 3),
            "seq": h.get("seq"), "kind": h.get("kind"),
            "codec": CODEC_NAMES.get(h.get("codec"), h.get("codec")),
            "bytes": len(s.row.payload), "action": "dropped" if s.drop else "published"}


def format_step(pass_no: int, s: Step, wall: str) -> str:
    d = describe(s)
    return (f"{wall} pass {pass_no} #{d['row']:4d} +{d['offset_s']:8.2f}s "
            f"seq {d['seq'] if d['seq'] is not None else '?':>3} K={d['kind']} "
            f"{d['codec']} {d['bytes']:3d}B "
            f"{'DROPPED' if s.drop else 'published'}")


# -- command line -----------------------------------------------------------------------
def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="vector_replay_publish.py",
        description="RS-13.2 desk check: replay a vector_dry_run capture onto an MQTT "
                    "broker at the capture's timing, with optional drop injection.")
    ap.add_argument("capture", help="vector_dry_run capture JSONL")
    ap.add_argument("--host", default=os.environ.get("LIFETRAC_MQTT_HOST", "127.0.0.1"))
    ap.add_argument("--port", type=int, default=int(os.environ.get("LIFETRAC_MQTT_PORT", "1883")))
    ap.add_argument("--topic", default=BASE_TOPIC,
                    help=f"publish topic (default {BASE_TOPIC}, what web_ui ingests); "
                         "the captured topic is ignored")
    ap.add_argument("--qos", type=int, default=0, choices=(0, 1))
    ap.add_argument("--speed", type=float, default=1.0,
                    help="time scale: 2 = twice as fast. Shape ages and age styling "
                         "are wall-clock at the base, so judge them at 1")
    ap.add_argument("--max-gap", type=float, default=0.0, dest="max_gap",
                    help="cap any single inter-frame wait at this many capture seconds (0 = no cap)")
    ap.add_argument("--loop", action="store_true", help="repeat the window until Ctrl-C")
    ap.add_argument("--passes", type=int, default=1, help="passes without --loop (default 1)")
    ap.add_argument("--loop-gap", type=float, default=DEFAULT_LOOP_GAP_S, dest="loop_gap",
                    help="seconds between passes (default %(default)s: more than the "
                         "store's 3 s outage rule, so each pass starts a fresh epoch)")
    ap.add_argument("--start-row", type=int, default=None, dest="start_row",
                    help="first capture row to replay (0-based, as the dry-run report numbers rows)")
    ap.add_argument("--end-row", type=int, default=None, dest="end_row",
                    help="last capture row to replay (inclusive)")
    ap.add_argument("--start-at-key", action="store_true", dest="start_at_key",
                    help="back the window start up to the last epoch start (K=1) at or before it")
    ap.add_argument("--drop-every", type=int, default=0, dest="drop_every",
                    help="drop the Nth, 2Nth, ... frame of each pass (0 = off)")
    ap.add_argument("--drop-rows", default="", dest="drop_rows",
                    help="capture rows to drop, e.g. 100,200-202")
    ap.add_argument("--dry-run", action="store_true", dest="dry_run",
                    help="print the plan without a broker and without waiting")
    ap.add_argument("--log", default=None,
                    help="also append one JSON line per frame (UTC wall time, row, seq, action) here")
    ap.add_argument("--quiet", action="store_true", help="summary only, no per-frame lines")
    return ap


def main(argv: Optional[list] = None) -> int:
    ap = build_parser()
    args = ap.parse_args(argv)
    try:
        drop_rows = parse_row_spec(args.drop_rows)
    except ValueError as exc:
        ap.error(f"--drop-rows: {exc}")
    if args.passes < 1:
        ap.error("--passes must be >= 1")
    try:
        rows = read_capture(args.capture)
    except OSError as exc:
        print(f"ERROR: cannot read {args.capture}: {exc}", file=sys.stderr)
        return 2
    if not rows:
        print(f"ERROR: no payload lines in {args.capture}", file=sys.stderr)
        return 2
    try:
        steps = plan(rows, speed=args.speed, max_gap=args.max_gap, drop_every=args.drop_every,
                     drop_rows=drop_rows, start_row=args.start_row, end_row=args.end_row,
                     start_at_key=args.start_at_key)
    except ValueError as exc:
        ap.error(str(exc))
    if not steps:
        print("ERROR: the row window is empty", file=sys.stderr)
        return 2
    n_drop = sum(1 for s in steps if s.drop)
    head = (f"{args.capture}: {len(rows)} row(s); window #{steps[0].row.idx}-#{steps[-1].row.idx} "
            f"= {len(steps)} frame(s), {steps[-1].offset_s:.1f} s per pass at speed {args.speed:g}; "
            f"{n_drop} drop(s) per pass; topic {args.topic}")

    log_fh = open(args.log, "a", encoding="utf-8") if args.log else None

    def _on_step(pass_no: int, s: Step, _t_rel: float) -> None:
        wall = _utc_now()
        if not args.quiet:
            print(format_step(pass_no, s, wall), flush=True)
        if log_fh is not None:
            log_fh.write(json.dumps({"utc": wall, "pass": pass_no, **describe(s)}) + "\n")
            log_fh.flush()

    try:
        if args.dry_run:
            print(head + " [dry run: no broker]")
            st = replay(steps, NullPublisher(), args.topic, passes=1,
                        clock=lambda: 0.0, sleep=lambda _s: None, on_step=_on_step)
        else:
            try:
                pub = PahoPublisher(args.host, args.port, f"vector_replay_{os.getpid()}", args.qos)
            except ImportError:
                print("ERROR: paho-mqtt not installed (pip install paho-mqtt)", file=sys.stderr)
                return 2
            except OSError as exc:
                print(f"ERROR: cannot reach the broker at {args.host}:{args.port}: {exc}",
                      file=sys.stderr)
                return 1
            print(f"{head}; broker {args.host}:{args.port}"
                  f"{'; looping until Ctrl-C' if args.loop else ''}", flush=True)
            try:
                st = replay(steps, pub, args.topic, passes=args.passes, loop=args.loop,
                            loop_gap_s=args.loop_gap, on_step=_on_step)
            finally:
                pub.close()
    finally:
        if log_fh is not None:
            log_fh.close()
    rows_txt = ", ".join(f"#{r}" if st.passes <= 1 else f"{p}:#{r}" for p, r in st.dropped_rows[:40])
    more = f" (+{len(st.dropped_rows) - 40} more)" if len(st.dropped_rows) > 40 else ""
    print(f"done: {st.passes} pass(es), {st.published} published, {st.dropped} dropped"
          f"{': ' + rows_txt + more if st.dropped_rows else ''}"
          f"{' (interrupted)' if st.interrupted else ''}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
