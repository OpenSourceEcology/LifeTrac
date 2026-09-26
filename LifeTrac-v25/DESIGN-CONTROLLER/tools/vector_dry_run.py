#!/usr/bin/env python3
"""tools/vector_dry_run.py — RS-13 bench tool for VECTOR (codec 6) frames.

Captures TileDeltaFrame payloads from a broker, replays them through the
base station's own parser and ``VectorSceneStore``, and reports what the
operator's browser would have shown: wire size against the one-fragment
limit, epoch starts, the store's verdict per frame (applied / bad /
behind), orphans, DIGEST agreement, arrival timing and the final scene.

Subcommands::

    capture      subscribe to a topic, save every payload as JSONL, analyse live
    replay       analyse a saved JSONL capture (no broker, no paho needed)
    tractor-log  summarise camera_service's ``vector_stats`` lines (encoder ms
                 per stage, wire bytes, level) from a ``docker logs`` dump

Where it runs (bench_tools/RS13_VECTOR_LEG.md):

* on the tractor, before any radio, against ``lifetrac/v25/cmd/image_frame``
  — what camera_service hands to image_tx_daemon;
* on the base, during a radio leg, against ``lifetrac/v25/video/tile_delta``
  — what image_rx_daemon reassembled, i.e. what web_ui would ingest.

Usage::

    python tools/vector_dry_run.py capture --topic lifetrac/v25/cmd/image_frame \\
        --duration 120 --out legs/step1_tractor.jsonl --strict
    python tools/vector_dry_run.py replay legs/leg2a_base.jsonl \\
        --profile image_bw500 --strict --json legs/leg2a_base.json
    python tools/vector_dry_run.py tractor-log legs/leg2a_camera_service.log

Capture line: ``{"ts": 1727345678.123, "topic": "...", "hex": "<payload>"}``.
Exit status: 0; 1 when ``--strict`` and a check fails; 2 on a usage error.

The one-fragment limits are derived from ``lora_proto`` (the same sizer the
tx daemon uses), never typed in: 203 B / 243 B for a delta at BW250 / BW500
and one byte less for an epoch start, whose 0xFD copies header is 5 B
instead of the 0xFE header's 4 B (VECTOR_SCENE.md §3.1).
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import queue
import re
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Optional

# Repo layout: tools/ sits beside base_station/. On a bench board the harness
# pushes base_station's modules flat into /work and runs with PYTHONPATH=/work,
# so the same imports resolve there without this insert.
_BS = Path(__file__).resolve().parent.parent / "base_station"
if _BS.is_dir() and str(_BS) not in sys.path:
    sys.path.insert(0, str(_BS))

from lora_proto import (  # noqa: E402
    LORA_HOP_HDR_LEN, PHY_BY_NAME, PhyProfile, lora_time_on_air_ms,
    max_image_fragment_body,
)
from image_pipeline import frame_format as _ff  # noqa: E402
from image_pipeline.frame_format import (  # noqa: E402
    CODEC_VECTOR, FRAME_KIND_KEY, parse_tile_delta_frame,
)
from image_pipeline.vector_scene import codec as vs  # noqa: E402
from image_pipeline.vector_scene_store import VectorSceneStore  # noqa: E402

# Per-fragment framing on the strict path (lora_proto.pack_image_fragments):
# the 0xFE header is magic, seq, idx, total = 4 B. A keyframe sent with
# copies > 1 takes the 0xFD header instead, one byte longer, which is why
# the encoder trims epoch starts by one byte (VECTOR_SCENE.md §3.1).
IMAGE_FRAG_V1_HDR_LEN = 4
KEY_COPIES_EXTRA = 1

PROFILE_CHOICES = ("image_bw250", "image_bw500", "image")
DEFAULT_TOPIC = "lifetrac/v25/cmd/image_frame"
BASE_TOPIC = "lifetrac/v25/video/tile_delta"

CODEC_NAMES = {v: k[len("CODEC_"):].lower() for k, v in vars(_ff).items()
               if k.startswith("CODEC_") and isinstance(v, int) and k != "CODEC_RESERVED_MAX"}


def one_fragment_limit(profile: PhyProfile) -> int:
    """Largest TileDeltaFrame payload that image_tx_daemon sends as ONE 0xFE
    fragment at ``profile`` under the 170 ms airtime cap."""
    return max_image_fragment_body(profile) - IMAGE_FRAG_V1_HDR_LEN


def peek_vs_header(body: bytes) -> Optional[dict]:
    """The 13-bit VS1 header (marker, V, K, AAAA, EEEE, LL) without decoding
    records — read with the codec's own reader so the bit order cannot drift."""
    if len(body) < 2:
        return None
    r = vs.BitReader(bytes(body))
    return {"marker": r.read(1), "version": r.read(1), "key": r.read(1),
            "age": r.read(4), "epoch": r.read(4), "level": r.read(2)}


@dataclass
class FrameRow:
    idx: int
    t_rel: float
    wire: int
    parsed: bool = False
    codec: Optional[int] = None
    frame_kind: Optional[int] = None
    seq: Optional[int] = None
    body: int = 0
    limit: int = 0
    fits: bool = False
    vector: bool = False
    applied: bool = False
    reason: Optional[str] = None
    records: int = 0
    epoch_switched: bool = False
    vs_epoch: Optional[int] = None
    vs_level: Optional[int] = None
    vs_age: Optional[int] = None
    digest: str = "-"          # "ok" | "BAD" | "-" (no DIGEST in this frame)


def _pct(xs: list, q: float) -> float:
    if not xs:
        return 0.0
    s = sorted(xs)
    return float(s[min(len(s) - 1, max(0, round((len(s) - 1) * q)))])


class DryRun:
    """Feed payloads in arrival order; ask for ``summary()`` at the end."""

    def __init__(self, profile_name: str = "image_bw250", *, min_frames: int = 1,
                 min_applied_ratio: float = 0.9) -> None:
        if profile_name not in PHY_BY_NAME:
            raise ValueError(f"unknown profile {profile_name!r}")
        self.profile_name = profile_name
        self.profile = PHY_BY_NAME[profile_name]
        self.limit = one_fragment_limit(self.profile)
        self.min_frames = int(min_frames)
        self.min_applied_ratio = float(min_applied_ratio)
        self.store = VectorSceneStore()
        self.rows: list[FrameRow] = []
        self.codec_counts: collections.Counter = collections.Counter()
        self.parse_errors = 0
        self.t0: Optional[float] = None
        self.last_ts: Optional[float] = None
        self._prev = self.store.stats

    # -- per frame ---------------------------------------------------------
    def feed(self, payload: bytes, ts: float) -> FrameRow:
        if self.t0 is None:
            self.t0 = ts
        row = FrameRow(idx=len(self.rows), t_rel=ts - self.t0, wire=len(payload))
        try:
            frame = parse_tile_delta_frame(bytes(payload))
        except Exception as exc:                                   # noqa: BLE001
            self.parse_errors += 1
            row.reason = f"parse:{type(exc).__name__}"
            self.rows.append(row)
            return row
        row.parsed = True
        row.codec, row.frame_kind, row.seq = frame.codec, frame.frame_kind, frame.base_seq
        self.codec_counts[frame.codec] += 1
        row.limit = self.limit - (KEY_COPIES_EXTRA if frame.frame_kind == FRAME_KIND_KEY else 0)
        row.fits = row.wire <= row.limit
        if frame.codec != CODEC_VECTOR:
            self.rows.append(row)
            return row
        row.vector = True
        row.body = len(frame.vector_body)
        hdr = peek_vs_header(frame.vector_body)
        if hdr is not None:
            row.vs_epoch, row.vs_level, row.vs_age = hdr["epoch"], hdr["level"], hdr["age"]
        airtime = lora_time_on_air_ms(min(row.wire, row.limit) + IMAGE_FRAG_V1_HDR_LEN
                                      + LORA_HOP_HDR_LEN, self.profile)
        res = self.store.ingest(frame.vector_body, frame.frame_kind, int(ts * 1000), airtime)
        row.applied, row.reason = res.applied, res.reason
        row.records, row.epoch_switched = res.records, res.epoch_switched
        st = self.store.stats
        if st["digest_mismatch"] > self._prev["digest_mismatch"]:
            row.digest = "BAD"
        elif st["digest_checks"] > self._prev["digest_checks"]:
            row.digest = "ok"
        self._prev = st
        self.last_ts = ts
        self.rows.append(row)
        return row

    # -- report --------------------------------------------------------------
    def summary(self) -> dict:
        vec = [r for r in self.rows if r.vector]
        parsed = [r for r in self.rows if r.parsed]
        wires = [r.wire for r in vec]
        gaps = [b.t_rel - a.t_rel for a, b in zip(vec, vec[1:])]
        first_applied = next((r for r in vec if r.applied), None)
        st = self.store.stats
        snap = self.store.snapshot(int(self.last_ts * 1000)) if self.last_ts is not None else None
        scene = None
        if snap is not None:
            scene = {
                "epoch": snap.get("epoch"), "level": snap.get("level"),
                "badge": snap.get("badge"), "digest_ok": snap.get("digest_ok"),
                "horizon_mode": (snap.get("horizon") or {}).get("mode"),
                "layers": {str(L.get("id")): len(L.get("shapes") or []) for L in snap.get("layers") or []},
                "cal_rev": snap.get("cal_rev"), "anchor_age_ms": snap.get("anchor_age_ms"),
                "handover": snap.get("handover"),
            }
        s = {
            "profile": self.profile_name,
            "one_fragment_limit": self.limit,
            "frames": {
                "total": len(self.rows), "vector": len(vec),
                "other": len(parsed) - len(vec), "parse_errors": self.parse_errors,
                "by_codec": {CODEC_NAMES.get(c, str(c)): n for c, n in sorted(self.codec_counts.items())},
            },
            "wire": {
                "min": min(wires) if wires else 0, "p50": _pct(wires, 0.5),
                "p95": _pct(wires, 0.95), "max": max(wires) if wires else 0,
                "over_limit": sum(1 for r in parsed if not r.fits),
            },
            "epoch_starts": sum(1 for r in vec if r.frame_kind == FRAME_KIND_KEY),
            "epoch_starts_applied": sum(1 for r in vec if r.frame_kind == FRAME_KIND_KEY and r.applied),
            "arrival": {
                "span_s": (vec[-1].t_rel - vec[0].t_rel) if len(vec) > 1 else 0.0,
                "fps_mean": ((len(vec) - 1) / (vec[-1].t_rel - vec[0].t_rel)
                             if len(vec) > 1 and vec[-1].t_rel > vec[0].t_rel else 0.0),
                "gap_p50_s": _pct(gaps, 0.5), "gap_p95_s": _pct(gaps, 0.95),
                "gap_max_s": max(gaps) if gaps else 0.0,
                "first_applied_t_rel_s": first_applied.t_rel if first_applied else None,
                "frames_before_first_apply": (vec.index(first_applied) if first_applied else len(vec)),
            },
            "store": st,
            "scene": scene,
        }
        s["checks"] = self.checks(s)
        s["pass"] = all(ok for _, ok, _ in s["checks"])
        return s

    def checks(self, s: dict) -> list:
        f, st, scene = s["frames"], s["store"], s["scene"]
        applied = st["frames_applied"]
        ratio = (applied / f["vector"]) if f["vector"] else 0.0
        ends_ok = scene is None or scene.get("digest_ok") is not False
        return [
            ("parse_ok", f["parse_errors"] == 0,
             f"{f['parse_errors']} unparseable payload(s)"),
            ("all_vector", f["other"] == 0,
             f"{f['other']} non-vector frame(s): {f['by_codec']}"),
            ("one_fragment", s["wire"]["over_limit"] == 0,
             f"{s['wire']['over_limit']} over {self.limit} B (epoch starts "
             f"{self.limit - KEY_COPIES_EXTRA} B); max {s['wire']['max']} B"),
            ("store_clean", st["frames_bad"] == 0,
             f"frames_bad={st['frames_bad']} {st.get('bad_reasons') or ''}"),
            ("no_orphans", st["orphans"] == 0, f"orphans={st['orphans']}"),
            ("digest", st["digest_mismatch"] == 0 and ends_ok,
             f"{st['digest_checks']} checked, {st['digest_mismatch']} mismatched, "
             f"scene digest_ok={scene.get('digest_ok') if scene else None}"),
            ("epoch_seen", s["epoch_starts_applied"] >= 1,
             f"{s['epoch_starts_applied']} epoch start(s) applied of {s['epoch_starts']} received"),
            ("first_apply", f["vector"] > 0 and s["arrival"]["frames_before_first_apply"] == 0,
             f"{s['arrival']['frames_before_first_apply']} frame(s) before the first apply"),
            ("applied_ratio", f["vector"] > 0 and ratio >= self.min_applied_ratio,
             f"{applied}/{f['vector']} applied = {ratio:.1%} (need >= {self.min_applied_ratio:.0%})"),
            ("min_frames", f["vector"] >= self.min_frames,
             f"{f['vector']} vector frame(s), need >= {self.min_frames}"),
        ]


# -- formatting ------------------------------------------------------------
def format_row(r: FrameRow) -> str:
    head = f"#{r.idx:4d} +{r.t_rel:8.2f}s {r.wire:3d}B"
    if not r.parsed:
        return f"{head}  UNPARSEABLE ({r.reason})"
    if not r.vector:
        return (f"{head}  codec={r.codec} ({CODEC_NAMES.get(r.codec, '?')}) K={r.frame_kind} "
                f"seq={r.seq}  NOT VECTOR{'' if r.fits else '  OVER ' + str(r.limit) + 'B'}")
    if r.applied:
        verdict = "applied"
    elif r.reason == "epoch_behind":
        verdict = "BEHIND"
    else:
        verdict = f"BAD:{r.reason}"
    return (f"{head}/{r.limit}{'' if r.fits else ' OVER'} K={r.frame_kind} seq={r.seq:3d} "
            f"body={r.body:3d} ep={r.vs_epoch} L{r.vs_level} age={r.vs_age} rec={r.records:2d} "
            f"{verdict} dig={r.digest}{' EPOCH-SWITCH' if r.epoch_switched else ''}")


def format_summary(s: dict) -> str:
    f, w, a, st = s["frames"], s["wire"], s["arrival"], s["store"]
    first = ("+%.2f s" % a["first_applied_t_rel_s"]
             if a["first_applied_t_rel_s"] is not None else "never")
    lines = [
        "== vector dry run summary ==",
        f"profile {s['profile']}: one-fragment limit {s['one_fragment_limit']} B "
        f"(epoch starts {s['one_fragment_limit'] - KEY_COPIES_EXTRA} B)",
        f"frames: {f['total']} total, {f['vector']} vector, {f['other']} other, "
        f"{f['parse_errors']} unparseable; by codec {f['by_codec']}",
        f"wire bytes (vector): min {w['min']} / p50 {w['p50']:.0f} / p95 {w['p95']:.0f} / "
        f"max {w['max']}; over limit: {w['over_limit']}",
        f"epoch starts: {s['epoch_starts']} received, {s['epoch_starts_applied']} applied",
        f"arrival: {a['fps_mean']:.2f} fps mean over {a['span_s']:.1f} s; gap p50 {a['gap_p50_s']:.2f} s, "
        f"p95 {a['gap_p95_s']:.2f} s, max {a['gap_max_s']:.2f} s; first applied at {first} "
        f"({a['frames_before_first_apply']} frame(s) before it)",
        f"store: applied {st['frames_applied']}, bad {st['frames_bad']} {st.get('bad_reasons') or ''}, "
        f"epoch_behind {st['epoch_behind']}, orphans {st['orphans']}, digest {st['digest_checks']} checked / "
        f"{st['digest_mismatch']} mismatched, epochs {st['epochs']}, handovers {st['handovers']}, "
        f"ttl_dropped {st['ttl_dropped']}, resync {st['resync_events']}, records {st['records_applied']}",
    ]
    sc = s["scene"]
    if sc is None:
        lines.append("scene: none (no frame applied)")
    else:
        layers = " ".join(f"{k}={v}" for k, v in sc["layers"].items()) or "no layers"
        lines.append(f"scene: epoch {sc['epoch']}, level {sc['level']}, badge {sc['badge']}, "
                     f"digest_ok {sc['digest_ok']}, horizon {sc['horizon_mode']}, {layers}, "
                     f"cal_rev {sc['cal_rev']}, anchor age {sc['anchor_age_ms']} ms"
                     f"{', HANDOVER PENDING' if sc.get('handover') else ''}")
    lines.append("checks:")
    for name, ok, detail in s["checks"]:
        lines.append(f"  [{'PASS' if ok else 'FAIL'}] {name:14s} {detail}")
    lines.append("RESULT: " + ("PASS" if s["pass"] else "FAIL"))
    return "\n".join(lines)


# -- capture file ------------------------------------------------------------
def write_capture_line(fh, ts: float, topic: str, payload: bytes) -> None:
    fh.write(json.dumps({"ts": round(float(ts), 3), "topic": topic,
                         "hex": bytes(payload).hex()}) + "\n")


def iter_capture(path) -> Iterator[tuple[float, str, bytes]]:
    """Yield ``(ts, topic, payload)`` per capture line; meta and blank lines are skipped."""
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
                yield float(obj.get("ts", 0.0)), str(obj.get("topic", "")), bytes.fromhex(obj["hex"])
            except ValueError:
                print(f"warning: line {n} has bad hex, skipped", file=sys.stderr)


# -- tractor log -------------------------------------------------------------
_VSTATS_RE = re.compile(r"vector_stats\s+(.*)$")
_KV_RE = re.compile(r"(\w+)=(\{[^}]*\}|\S+)")


def parse_vector_stats_line(line: str) -> Optional[dict]:
    """camera_service: ``vector_stats ms_total=12.3 ms={"resize":1.2,...} bytes=203
    level=0 detail=80 epoch=3 n_live=17 residual=0.031 epoch_pending=0``"""
    m = _VSTATS_RE.search(line)
    if not m:
        return None
    out: dict = {}
    for k, v in _KV_RE.findall(m.group(1)):
        if v.startswith("{"):
            try:
                out[k] = json.loads(v)
            except ValueError:
                out[k] = {}
        else:
            try:
                out[k] = int(v)
            except ValueError:
                try:
                    out[k] = float(v)
                except ValueError:
                    out[k] = v
    return out if "ms_total" in out else None


def summarise_tractor_log(lines: Iterable[str]) -> dict:
    rows = [r for r in (parse_vector_stats_line(ln) for ln in lines) if r]
    totals = [float(r["ms_total"]) for r in rows]
    stages: dict = collections.defaultdict(list)
    for r in rows:
        for k, v in (r.get("ms") or {}).items():
            if k != "total":
                stages[k].append(float(v))
    nbytes = [int(r["bytes"]) for r in rows if "bytes" in r]
    levels = collections.Counter(int(r.get("level", 0)) for r in rows)
    return {
        "n": len(rows),
        "ms_total": {"p50": _pct(totals, 0.5), "p95": _pct(totals, 0.95),
                     "max": max(totals) if totals else 0.0},
        "stages_p50_ms": {k: _pct(v, 0.5) for k, v in stages.items()},
        "bytes": {"p50": _pct(nbytes, 0.5), "max": max(nbytes) if nbytes else 0},
        "levels": {str(k): v for k, v in sorted(levels.items())},
        "epoch_pending_lines": sum(1 for r in rows if int(r.get("epoch_pending", 0))),
        "last": rows[-1] if rows else None,
    }


def format_tractor_summary(d: dict) -> str:
    if not d["n"]:
        return ("no vector_stats lines found (VECTOR mode logs one every "
                "LIFETRAC_CAMERA_HEALTH_EVERY_S, default 2 s)")
    st = " ".join(f"{k}={v:.1f}" for k, v in d["stages_p50_ms"].items())
    last = d["last"] or {}
    return "\n".join([
        f"vector_stats lines: {d['n']}",
        f"encoder ms_total: p50 {d['ms_total']['p50']:.1f} / p95 {d['ms_total']['p95']:.1f} / "
        f"max {d['ms_total']['max']:.1f}",
        f"stage p50 ms: {st}",
        f"frame bytes: p50 {d['bytes']['p50']:.0f} / max {d['bytes']['max']}",
        f"levels seen: {d['levels']}; lines with a pending epoch start: {d['epoch_pending_lines']}",
        f"last: epoch {last.get('epoch')} n_live {last.get('n_live')} residual {last.get('residual')} "
        f"detail {last.get('detail')}",
    ])


# -- commands ----------------------------------------------------------------
def _finish(dr: DryRun, args) -> int:
    s = dr.summary()
    print(format_summary(s))
    if getattr(args, "json", None):
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(s, fh, indent=2, default=str)
        print(f"json report -> {args.json}")
    return 1 if (getattr(args, "strict", False) and not s["pass"]) else 0


def _make_client(mqtt, client_id: str):
    api = getattr(mqtt, "CallbackAPIVersion", None)       # paho 2.x
    if api is not None:
        return mqtt.Client(api.VERSION2, client_id=client_id)
    return mqtt.Client(client_id=client_id)                # paho 1.x


def cmd_capture(args) -> int:
    try:
        import paho.mqtt.client as mqtt                    # type: ignore
    except ImportError:
        print("ERROR: paho-mqtt not installed (pip install paho-mqtt)", file=sys.stderr)
        return 2
    dr = DryRun(args.profile, min_frames=args.min_frames, min_applied_ratio=args.min_applied_ratio)
    inbox: queue.Queue = queue.Queue()

    def _on_connect(client, _userdata, _flags, *rest):
        rc = rest[0] if rest else 0
        print(f"connected to {args.mqtt_host}:{args.mqtt_port} (rc={rc}); "
              f"subscribing {args.topic}", flush=True)
        client.subscribe(args.topic, qos=0)

    def _on_message(_client, _userdata, msg):
        inbox.put((time.time(), msg.topic, bytes(msg.payload)))

    client = _make_client(mqtt, f"vector_dry_run_{os.getpid()}")
    client.on_connect = _on_connect
    client.on_message = _on_message
    client.connect(args.mqtt_host, args.mqtt_port, keepalive=30)
    client.loop_start()
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    deadline = (time.time() + args.duration) if args.duration > 0 else None
    n = 0
    print(f"capture -> {out_path} (profile {args.profile}, limit {dr.limit} B); "
          f"{'%.0f s' % args.duration if deadline else 'until Ctrl-C'}"
          f"{' or %d frames' % args.count if args.count else ''}", flush=True)
    with open(out_path, "a", encoding="utf-8") as out:
        out.write(json.dumps({"meta": {"tool": "vector_dry_run", "topic": args.topic,
                                       "profile": args.profile,
                                       "started": round(time.time(), 3)}}) + "\n")
        out.flush()
        try:
            while True:
                if deadline is not None and time.time() >= deadline:
                    break
                if args.count and n >= args.count:
                    break
                try:
                    ts, topic, payload = inbox.get(timeout=0.25)
                except queue.Empty:
                    continue
                write_capture_line(out, ts, topic, payload)
                out.flush()
                row = dr.feed(payload, ts)
                n += 1
                if not args.quiet:
                    print(format_row(row), flush=True)
        except KeyboardInterrupt:
            print("interrupted", flush=True)
        finally:
            client.loop_stop()
            try:
                client.disconnect()
            except Exception:                                  # noqa: BLE001
                pass
    print(f"captured {n} payload(s) to {out_path}")
    return _finish(dr, args)


def cmd_replay(args) -> int:
    dr = DryRun(args.profile, min_frames=args.min_frames, min_applied_ratio=args.min_applied_ratio)
    n = 0
    for ts, _topic, payload in iter_capture(args.capture):
        row = dr.feed(payload, ts)
        n += 1
        if not args.quiet:
            print(format_row(row))
    print(f"replayed {n} payload(s) from {args.capture}")
    return _finish(dr, args)


def cmd_tractor_log(args) -> int:
    with open(args.log, "r", encoding="utf-8", errors="replace") as fh:
        d = summarise_tractor_log(fh)
    print(format_tractor_summary(d))
    if getattr(args, "json", None):
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(d, fh, indent=2, default=str)
        print(f"json report -> {args.json}")
    return 0


def _add_analysis_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--profile", default="image_bw250", choices=PROFILE_CHOICES,
                   help="PHY the leg runs at: image_bw250 = FHSS profile 1 (203 B), "
                        "image_bw500 = DTS profile 2 (243 B)")
    p.add_argument("--strict", action="store_true", help="exit 1 when any check fails")
    p.add_argument("--min-frames", type=int, default=1, dest="min_frames")
    p.add_argument("--min-applied-ratio", type=float, default=0.9, dest="min_applied_ratio")
    p.add_argument("--quiet", action="store_true", help="summary only, no per-frame lines")
    p.add_argument("--json", default=None, help="also write the summary as JSON here")


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="vector_dry_run.py",
                                 description="RS-13 bench tool: capture/replay VECTOR frames "
                                             "through the base station's own store.")
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture", help="subscribe, save JSONL, analyse live")
    c.add_argument("--mqtt-host", default=os.environ.get("LIFETRAC_MQTT_HOST", "127.0.0.1"))
    c.add_argument("--mqtt-port", type=int,
                   default=int(os.environ.get("LIFETRAC_MQTT_PORT", "1883")))
    c.add_argument("--topic", default=DEFAULT_TOPIC,
                   help=f"{DEFAULT_TOPIC} on the tractor, {BASE_TOPIC} on the base")
    c.add_argument("--duration", type=float, default=0.0, help="seconds; 0 = until Ctrl-C")
    c.add_argument("--count", type=int, default=0, help="stop after N payloads; 0 = no limit")
    c.add_argument("--out", required=True, help="JSONL capture file (appended)")
    _add_analysis_args(c)
    c.set_defaults(func=cmd_capture)
    r = sub.add_parser("replay", help="analyse a saved capture")
    r.add_argument("capture")
    _add_analysis_args(r)
    r.set_defaults(func=cmd_replay)
    t = sub.add_parser("tractor-log", help="summarise camera_service vector_stats lines")
    t.add_argument("log")
    t.add_argument("--json", default=None)
    t.set_defaults(func=cmd_tractor_log)
    return ap


def main(argv: Optional[list] = None) -> int:
    args = build_parser().parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
