"""VS1 vector scene store: epochs, per-field last-writer-wins, honest ages and
the ``vector_scene`` snapshot (``VECTOR_SCENE.md`` §3.2–3.5, §4.3–4.4, §7.2).

The store is the base half of the VS1 codec. ``ingest`` takes one decoded
``TileDeltaFrame`` body per call and never raises; ``snapshot`` returns the
JSON-serialisable object that ``state_publisher`` hands to the browser. Nothing
else leaves the store: VS1 has no uplink, so loss is repaired only by what the
tractor sends next (§4.3). Pure stdlib because it runs inside ``web_ui``.

Times are milliseconds on the caller's clock. A frame's capture time is
``rx_ms − airtime_ms − 200·AAAA`` (§3.2). A saturated AAAA (15) is only a
bound, so its records fill absent state but never supersede state with a
known capture time. Every field carries its own capture-time clock (§3.4
rule 5), which is what makes duplicates and reordering harmless.

Geometry leaves the store in canvas pixels with the local offset, the group
shift and the ground zoom already applied (§3.4, §4.4); colours leave as
``#rrggbb`` with GAIN applied. The renderer only draws what it is given.
"""
from __future__ import annotations

import bisect
import copy
import math
from collections import deque
from dataclasses import dataclass, field
from typing import Any

from .vector_scene import codec as vs

CANVAS_W, CANVAS_H = 384, 256
SATURATED_AGE = 15                  # AAAA = 15: >= 3.0 s, a bound (§3.2)
SATURATED_AGE_MS = 3000
TTL_FRAMES = 20                     # applied frames without a verification (§4.3)
EPOCH_OUTAGE_MS = 3000              # §3.5 rule 4
BEHIND_SELF_HEAL = 3                # §3.5 rule 5
HANDOVER_DIGEST_TIMEOUT_FRAMES = 2  # only the DIGEST half of the hand-over times out (§3.5)
RESYNC_ORPHAN_WINDOW_MS = 10_000
RESYNC_ORPHAN_RATIO = 0.20
RESYNC_ORPHAN_MIN_ITEMS = 8         # a ratio over fewer records is noise, not desync
RESYNC_DIGEST_RUN = 3
ZOOM_DROP_RATIO = 2.0               # r'/r above this is dropped as PREDICTED (§4.4)
BADGE_CACHED, BADGE_PREDICTED, BADGE_VECTOR = 1, 4, 7
MIN_OBJECT_PLACEHOLDER = "n/a (no calibration)"

_GRP_FAR, _GRP_GROUND, _GRP_ALL, _GRP_L4 = 0, 1, 2, 3
_EDGE_CLS = ("rut", "structure", "fence", "contact", "overhead")
_DEFINES = (vs.Poly, vs.Tree, vs.Edge, vs.Blob, vs.Anom)
_HZN = (vs.HznAbs, vs.HznResid, vs.HznColours, vs.HznNoHorizon)
_CTRL = (vs.Gshift, vs.Status, vs.Confirm, vs.Digest, vs.LayerClear, vs.CalRev,
         vs.Pal, vs.Gzoom, vs.Gain)
_EDGE_FILL = vs.Fill(rgb444=0)      # EDGE carries no FILL (§3.3); its state-hash colour is 0


def _wins(new: tuple[int, bool], old: tuple[int, bool] | None) -> bool:
    """Per-field last-writer-wins (§3.4 rule 5). A known capture beats an
    older or equal one (a duplicate re-applies the same value); a saturated
    bound only fills absent state (§3.2)."""
    if old is None:
        return True
    if not new[1]:
        return False
    return (not old[1]) or new[0] >= old[0]


def _layer_of(id_: int) -> int:
    if id_ in vs.ID_PLANT:
        return 2
    if id_ in vs.ID_EDGE:
        return 3
    if id_ in vs.ID_L4:
        return 4
    return 1            # masses 1–31 and corridor anomalies 80–87 (§2.2)


def _rgb8(rgb444: int) -> tuple[int, int, int]:
    return ((rgb444 >> 8) & 0xF) * 17, ((rgb444 >> 4) & 0xF) * 17, (rgb444 & 0xF) * 17


def _clip8(v: float) -> int:
    return max(0, min(255, int(round(v))))


def _hex(rgb: tuple[int, int, int]) -> str:
    return "#%02x%02x%02x" % tuple(rgb)


def _gain(code: int) -> float:
    return 2.0 ** ((code - 16) / 32.0)


def _r2(v: float) -> float:
    return round(v, 2)


def _bucket(rec) -> str:
    """Which ``bits.per_layer`` bucket a record's bits belong to (§7.2)."""
    if isinstance(rec, _HZN) or isinstance(rec, vs.Skyline):
        return "L0"
    if isinstance(rec, vs.Insert):
        return "insert"
    if isinstance(rec, _CTRL):
        return "ctrl"
    if isinstance(rec, vs.Anom):
        return "L1"
    if isinstance(rec, vs.Hole):
        return "L%d" % _layer_of(rec.parent)
    return "L%d" % _layer_of(rec.id)


class _Horizon:
    """The horizon line in canvas px: y(x) = yc + (x − 192)·tan(ang) +
    sag·((x − 192)/192)², then shifted by (sx, sy) as a far-group member."""
    __slots__ = ("yc", "tan", "sag", "sx", "sy")

    def __init__(self, yc: float, ang_deg: float, sag: float, sx: float = 0.0, sy: float = 0.0):
        self.yc, self.sag, self.sx, self.sy = yc, sag, sx, sy
        self.tan = math.tan(math.radians(ang_deg))

    def y_at(self, x: float) -> float:
        d = (x - self.sx) - CANVAS_W / 2
        return self.yc + d * self.tan + self.sag * (d / (CANVAS_W / 2)) ** 2 + self.sy

    def pts(self) -> list[list[float]]:
        return [[float(x), _r2(self.y_at(x))] for x in (0, CANVAS_W // 2, CANVAS_W)]


class _History:
    """Absolute group-transform codes keyed by capture time, so a define is
    read against the transform current at *its* capture and the renderer
    applies now − at-define (§3.4 rule 5) whatever order frames arrived in."""
    __slots__ = ("caps", "vals", "known")

    def __init__(self) -> None:
        self.caps: list[int] = []
        self.vals: list = []
        self.known = True

    def put(self, clk: tuple[int, bool], val) -> None:
        if not clk[1]:
            if self.caps:
                return                      # a bound never supersedes known state
            self.known = False
        elif not self.known:
            self.caps, self.vals, self.known = [], [], True
        i = bisect.bisect_left(self.caps, clk[0])
        if i < len(self.caps) and self.caps[i] == clk[0]:
            self.vals[i] = val
            return
        self.caps.insert(i, clk[0])
        self.vals.insert(i, val)
        if len(self.caps) > 512:
            del self.caps[0], self.vals[0]

    def at(self, t: int, default):
        i = bisect.bisect_right(self.caps, t)
        return self.vals[i - 1] if i else default

    def now(self, default):
        return self.vals[-1] if self.vals else default


@dataclass(frozen=True)
class IngestResult:
    applied: bool
    reason: str | None          # VsDecodeError reason, "epoch_behind" or None
    epoch_switched: bool
    records: int


@dataclass(slots=True)
class _Shape:
    id: int
    define: Any                                 # Poly / Tree / Edge / Blob / Anom
    dhash: int
    define_clk: tuple[int, bool]                # first define with this hash (transform reference)
    geom_clk: tuple[int, bool]                  # latest define with this hash (geometry LWW)
    verified: tuple[int, bool]                  # last verified capture: define / CONFIRM / UPD
    fill: vs.Fill
    col_clk: tuple[int, bool]
    epoch: int
    off: tuple[int, int] = (0, 0)               # UPD codes, 4 px units, absolute from the define
    off_clk: tuple[int, bool] | None = None
    group: int | None = None                    # 0 far, 1 ground, 3 L4; None until the epoch has an anchor
    inserts: dict = field(default_factory=dict)             # (edge, k) -> (Insert, clock)
    holes: list = field(default_factory=lambda: [None] * 4)  # slot -> (Hole | None, clock) | None
    frames_since_verify: int = 0
    carry_px: tuple[float, float] = (0.0, 0.0)  # previous epochs' shift, baked at hand-over
    baked_rgb: tuple[int, int, int] | None = None  # colour with the previous epoch's GAIN baked

    @property
    def v0_px(self) -> tuple[float, float]:
        d = self.define
        if isinstance(d, vs.Poly):
            cell = 8 if d.grid == 0 else 4
            return d.vertices[0][0] * cell, d.vertices[0][1] * cell
        if isinstance(d, vs.Tree):
            return d.cx * 8, d.cy * 8
        if isinstance(d, vs.Edge):
            return d.points[0][0] * 4, d.points[0][1] * 4
        if isinstance(d, vs.Anom):
            return d.x * 8, d.y * 8
        return d.cx * 4, d.cy * 4

    def live_holes(self) -> list:
        return [slot[0] if slot else None for slot in self.holes]


class VectorSceneStore:
    """Epoch state machine, per-field LWW store and snapshot builder."""

    def __init__(self, canvas_w: int = CANVAS_W, canvas_h: int = CANVAS_H) -> None:
        self.canvas_w, self.canvas_h = canvas_w, canvas_h
        self.reset()

    def reset(self) -> None:
        self._epoch: int | None = None
        self._last_cap: int | None = None       # last applied known capture time (§3.5 rule 2)
        self._last_rx: int | None = None        # rx time of the last applied frame (§3.5 rule 4)
        self._behind: list[int] = []            # capture times of consecutive behind frames (rule 5)
        self._shapes: dict[int, _Shape] = {}
        self._cached: dict[int, _Shape] = {}    # previous epoch, badge 1, until hand-over
        self._cached_horizon: dict | None = None
        self._cached_epoch: int | None = None
        self._palette = list(vs.STATIC_PALETTE) + [0x888] * 8
        self._pal_clk: list = [None] * 16
        self._status: tuple | None = None       # (Status, clock)
        self._calrev: tuple | None = None
        self._skyline: tuple | None = None      # (Skyline, clock); L0 singleton, never cleared
        self._last_clear: tuple | None = None   # (epoch, range) last applied LAYER_CLEAR
        self._last_bits: dict | None = None
        self._last_level = 0                    # LL of the last applied frame (§3.2)
        self._resync = False
        self._bad_reasons: dict[str, int] = {}
        self._st = {k: 0 for k in ("frames_rx", "frames_bad", "frames_applied", "epoch_behind",
                                   "orphans", "digest_checks", "digest_mismatch", "bad_version",
                                   "epochs", "handovers", "ttl_dropped", "resync_events",
                                   "records_applied")}
        self._epoch_reset()

    def _epoch_reset(self) -> None:
        self._abs: tuple | None = None          # (HznAbs, clock)
        self._resid: tuple | None = None
        self._nohz: tuple | None = None
        self._cols: tuple | None = None         # ((top vfill, bottom vfill), clock)
        self._gs = [_History() for _ in range(4)]
        self._zoom = _History()
        self._gain: tuple | None = None         # ((r, g, b), clock)
        self._tomb: dict[int, tuple[int, bool]] = {}
        self._epoch_clear: int | None = None    # the epoch start's LAYER_CLEAR range
        self._digest_ok: bool | None = None
        self._digest_n_live: int | None = None
        self._digest_run = 0
        self._frames_in_epoch = 0
        self._orphan_win: deque = deque()       # (rx_ms, items, orphans) per applied frame

    # ------------------------------------------------------------ public API

    @property
    def stats(self) -> dict:
        st = dict(self._st)
        st["resync"] = self._resync
        st["bad_reasons"] = dict(self._bad_reasons)
        st["epoch"] = self._epoch
        st["shapes"] = len(self._shapes)
        st["cached_shapes"] = len(self._cached)
        return st

    def ingest(self, body: bytes, frame_kind: int, rx_ms: int, airtime_ms: float = 0.0) -> IngestResult:
        """Apply one VS body. Never raises: a bad frame is counted per reason."""
        self._st["frames_rx"] += 1
        if not isinstance(body, (bytes, bytearray, memoryview)):
            return self._bad("not_bytes")           # garbage types, not just garbage bytes
        body = bytes(body)
        try:
            frame = vs.decode_frame(body, frame_kind)
        except vs.VsDecodeError as e:
            return self._bad(e.reason)
        except Exception:
            return self._bad("exception")
        hdr = frame.header
        known = hdr.age < SATURATED_AGE
        age_ms = 200 * hdr.age if known else SATURATED_AGE_MS
        clk = (int(round(rx_ms - airtime_ms - age_ms)), known)
        decision = self._epoch_decision(hdr, clk[0], rx_ms)
        if decision == "behind":
            self._st["epoch_behind"] += 1
            return IngestResult(False, "epoch_behind", False, len(frame.records))
        switched = decision == "switch"
        if switched:
            self._switch_epoch(hdr.epoch)
        if hdr.key:
            self._resync = False                    # an accepted epoch start ends a resync (§4.3)
        self._apply_frame(frame, clk, rx_ms, len(body) * 8)
        return IngestResult(True, None, switched, len(frame.records))

    def _bad(self, reason: str) -> IngestResult:
        self._st["frames_bad"] += 1
        self._bad_reasons[reason] = self._bad_reasons.get(reason, 0) + 1
        if reason == "bad_version":
            self._st["bad_version"] += 1
        return IngestResult(False, reason, False, 0)

    # ------------------------------------------------------------ epochs (§3.5)

    def _epoch_decision(self, hdr: vs.Header, cap: int, rx_ms: int) -> str:
        if self._epoch is None:
            return "switch"
        if hdr.epoch == self._epoch:
            self._behind = []
            return "same"
        if hdr.key and (self._last_cap is None or cap > self._last_cap):
            return "switch"                         # rule 2: reboot or a normal epoch start
        if 1 <= (hdr.epoch - self._epoch) % 16 <= 7:
            return "switch"                         # rule 3
        if self._last_rx is not None and rx_ms - self._last_rx > EPOCH_OUTAGE_MS:
            return "switch"                         # rule 4: outage, any wrap
        if self._behind and cap > self._behind[-1]:
            self._behind.append(cap)
        else:
            self._behind = [cap]
        if len(self._behind) >= BEHIND_SELF_HEAL:
            return "switch"                         # rule 5: self-heal
        return "behind"

    def _switch_epoch(self, epoch: int) -> None:
        if self._epoch is not None:
            # The old picture stays visible as CACHED until hand-over. Bake the
            # ending epoch's transforms and GAIN into each shape so the cached
            # picture, and later the carried shapes, need no old render state.
            for sh in self._shapes.values():
                sx, sy = self._shift_px(sh)
                sh.carry_px = (sx, sy)
                sh.baked_rgb = self._base_rgb8(sh)
            merged = dict(self._cached)
            merged.update(self._shapes)
            self._cached = merged
            if self._anchor_clk() is not None:
                self._cached_horizon = self._horizon_json(cached=False)
                self._cached_epoch = self._epoch
        self._epoch = epoch
        self._shapes = {}
        self._behind = []
        self._epoch_reset()
        self._st["epochs"] += 1

    def _carriable(self, id_: int) -> bool:
        """Cached shapes outside the epoch start's LAYER_CLEAR range carry over (§3.3)."""
        return self._epoch_clear is None or _layer_of(id_) <= self._epoch_clear

    def _lookup(self, id_: int) -> _Shape | None:
        sh = self._shapes.get(id_)
        if sh is None and self._carriable(id_):
            sh = self._cached.get(id_)
        return sh

    def _live(self) -> list[_Shape]:
        live = list(self._shapes.values())
        live += [sh for id_, sh in self._cached.items()
                 if id_ not in self._shapes and self._carriable(id_)]
        return live

    def _check_handover(self) -> None:
        if not self._cached or self._anchor_clk() is None:
            return                                  # the anchor half never times out
        digest_half = (self._digest_n_live is not None
                       and 2 * len(self._live()) >= self._digest_n_live)
        if digest_half or self._frames_in_epoch >= HANDOVER_DIGEST_TIMEOUT_FRAMES:
            for id_, sh in self._cached.items():
                if self._carriable(id_) and id_ not in self._shapes:
                    if sh.group is None:
                        sh.group = self._membership(sh)
                    self._shapes[id_] = sh
            self._cached, self._cached_horizon, self._cached_epoch = {}, None, None
            self._st["handovers"] += 1

    # ------------------------------------------------------------ frames

    def _apply_frame(self, frame: vs.Frame, clk: tuple[int, bool], rx_ms: int, body_bits: int) -> None:
        self._verified_now: set[int] = set()
        self._items = 0
        self._frame_orphans = 0
        bits = {"hdr": vs.HEADER_BITS, "L0": 0, "L1": 0, "L2": 0, "L3": 0, "L4": 0, "ctrl": 0, "insert": 0}
        confirms: list[vs.Confirm] = []
        digest: vs.Digest | None = None
        for rec in frame.records:
            bits[_bucket(rec)] += vs.record_bits(rec)
            if isinstance(rec, vs.Confirm):
                confirms.append(rec)
            elif isinstance(rec, vs.Digest):
                digest = rec
            else:
                self._items += 1
                self._apply_record(rec, clk)
        # The DIGEST describes the mirror after this frame's changes, and it
        # gates the CONFIRMs (§4.3), so both are evaluated after the rest.
        if digest is not None:
            self._check_digest(digest)
        for c in confirms:
            self._apply_confirm(c, clk)
        bits["pad"] = body_bits - sum(bits.values())
        self._last_bits = {"last_frame": body_bits, "per_layer": bits}
        self._last_level = frame.header.level
        self._st["records_applied"] += len(frame.records)
        self._st["frames_applied"] += 1
        self._frames_in_epoch += 1
        self._last_rx = rx_ms
        if clk[1]:
            self._last_cap = clk[0] if self._last_cap is None else max(self._last_cap, clk[0])
        self._tick_ttl()
        self._orphan_win.append((rx_ms, self._items, self._frame_orphans))
        self._check_orphan_rate(rx_ms)
        self._check_handover()

    def _tick_ttl(self) -> None:
        for shapes in (self._shapes, self._cached):
            for id_ in list(shapes):
                sh = shapes[id_]
                if id_ in self._verified_now and sh.frames_since_verify == 0:
                    continue
                sh.frames_since_verify += 1
                if sh.frames_since_verify >= TTL_FRAMES:
                    del shapes[id_]
                    self._st["ttl_dropped"] += 1

    def _orphan(self, n: int = 1) -> None:
        self._frame_orphans += n
        self._st["orphans"] += n

    def _check_orphan_rate(self, rx_ms: int) -> None:
        win = self._orphan_win
        while win and win[0][0] < rx_ms - RESYNC_ORPHAN_WINDOW_MS:
            win.popleft()
        items = sum(w[1] for w in win)
        orphans = sum(w[2] for w in win)
        if items >= RESYNC_ORPHAN_MIN_ITEMS and orphans > RESYNC_ORPHAN_RATIO * items:
            self._enter_resync()

    def _enter_resync(self) -> None:
        if not self._resync:
            self._resync = True
            self._st["resync_events"] += 1

    def _verify(self, sh: _Shape, clk: tuple[int, bool]) -> None:
        if _wins(clk, sh.verified):
            sh.verified = clk
        sh.frames_since_verify = 0
        self._verified_now.add(sh.id)

    # ------------------------------------------------------------ records (§3.3, §3.4)

    def _apply_record(self, rec, clk: tuple[int, bool]) -> None:
        if isinstance(rec, _DEFINES):
            self._apply_define(rec, clk)
        elif isinstance(rec, vs.Upd):
            sh = self._lookup(rec.id)
            if sh is None:
                self._orphan()
                return
            if _wins(clk, sh.off_clk):
                sh.off, sh.off_clk = (rec.dx, rec.dy), clk
            self._verify(sh, clk)
        elif isinstance(rec, vs.Ucol):
            sh = self._lookup(rec.id)
            if sh is None:
                self._orphan()
                return
            if _wins(clk, sh.col_clk):
                sh.fill, sh.col_clk, sh.baked_rgb = rec.fill, clk, None
        elif isinstance(rec, vs.Del):
            sh = self._lookup(rec.id)
            if sh is None:
                self._orphan()
                return
            if clk[1] and clk[0] < sh.define_clk[0]:
                return                              # older than the live define (reordered)
            self._shapes.pop(rec.id, None)
            self._cached.pop(rec.id, None)
            self._tomb[rec.id] = clk
        elif isinstance(rec, vs.Insert):
            sh = self._lookup(rec.id)
            if (sh is None or not isinstance(sh.define, vs.Poly)
                    or rec.edge >= len(sh.define.vertices)):
                self._orphan()
                return
            if clk[1] and clk[0] < sh.define_clk[0]:
                return                              # belongs to a superseded define
            cur = sh.inserts.get((rec.edge, rec.k))
            if _wins(clk, cur[1] if cur else None):
                sh.inserts[(rec.edge, rec.k)] = (rec, clk)
        elif isinstance(rec, vs.Hole):
            sh = self._lookup(rec.parent)
            if sh is None or not isinstance(sh.define, vs.Poly):
                self._orphan()
                return
            cur = sh.holes[rec.slot]
            if _wins(clk, cur[1] if cur else None):
                sh.holes[rec.slot] = (None if rec.delete else rec, clk)
        elif isinstance(rec, vs.Gshift):
            self._gs[rec.grp].put(clk, (rec.dx, rec.dy))
        elif isinstance(rec, vs.Gzoom):
            self._zoom.put(clk, rec.code)
        elif isinstance(rec, vs.Gain):
            if _wins(clk, self._gain[1] if self._gain else None):
                self._gain = ((rec.r, rec.g, rec.b), clk)
        elif isinstance(rec, vs.Pal):
            slot = 8 + rec.slot
            if _wins(clk, self._pal_clk[slot]):
                self._palette[slot], self._pal_clk[slot] = rec.rgb444, clk
        elif isinstance(rec, vs.Status):
            if _wins(clk, self._status[1] if self._status else None):
                self._status = (rec, clk)
        elif isinstance(rec, vs.CalRev):
            if _wins(clk, self._calrev[1] if self._calrev else None):
                self._calrev = (rec, clk)
        elif isinstance(rec, vs.Skyline):
            if _wins(clk, self._skyline[1] if self._skyline else None):
                self._skyline = (rec, clk)
        elif isinstance(rec, vs.LayerClear):
            self._layer_clear(rec.range)
        elif isinstance(rec, _HZN):
            self._apply_hzn(rec, clk)

    def _apply_define(self, rec, clk: tuple[int, bool]) -> None:
        id_ = 80 + rec.slot if isinstance(rec, vs.Anom) else rec.id
        tomb = self._tomb.get(id_)
        if tomb is not None:
            if not (_wins(clk, tomb) and clk[0] > tomb[0]):
                return                              # deleted at a later capture
            del self._tomb[id_]
        dh = vs.define_hash(rec)
        if isinstance(rec, vs.Anom):
            fill = vs.Fill(rgb444=rec.rgb444)
        elif isinstance(rec, vs.Edge):
            fill = _EDGE_FILL                   # class-coloured by the renderer; hashes as colour 0
        else:
            fill = rec.fill
        sh = self._shapes.get(id_)
        if sh is None and self._carriable(id_) and id_ in self._cached:
            # The cached original keeps showing until hand-over; the new epoch
            # works on a copy that carries the offset, INSERTs, holes and age.
            sh = copy.copy(self._cached[id_])
            sh.inserts, sh.holes = dict(sh.inserts), list(sh.holes)
            self._shapes[id_] = sh
        if sh is None:
            sh = _Shape(id_, rec, dh, clk, clk, clk, fill, clk, self._epoch, off_clk=clk)
            sh.group = self._membership(sh)
            self._shapes[id_] = sh
            self._verified_now.add(id_)
            return
        if not _wins(clk, sh.geom_clk):
            return                                  # an older define (reordered)
        if dh == sh.dhash:
            sh.geom_clk = clk                       # same define-hash: only the geometry age moves
            self._verify(sh, clk)
            return
        # A new define-hash is a redefine at the current position (§3.4 rule 5):
        # the local offset restarts at this capture and the INSERTs go.
        sh.define, sh.dhash, sh.define_clk, sh.geom_clk = rec, dh, clk, clk
        sh.off, sh.off_clk = (0, 0), clk
        sh.inserts = {}
        sh.carry_px, sh.baked_rgb, sh.epoch = (0.0, 0.0), None, self._epoch
        if _wins(clk, sh.col_clk):
            sh.fill, sh.col_clk = fill, clk
        sh.group = self._membership(sh)
        self._verify(sh, clk)

    def _membership(self, sh: _Shape) -> int | None:
        """GSHIFT group, fixed at define time (§3.3): L4 ids are group 3, the
        rest ground when v0 is below the epoch's horizon line, else far."""
        if _layer_of(sh.id) == 4:
            return _GRP_L4
        line = self._abs_line()
        if line is None:
            # NO_HORIZON: nothing is above the horizon. None until an anchor exists.
            return _GRP_GROUND if self._nohz is not None else None
        x0, y0 = sh.v0_px
        return _GRP_GROUND if y0 > line.y_at(x0) else _GRP_FAR

    def _layer_clear(self, rng: int) -> None:
        pair = (self._epoch, rng)
        if pair == self._last_clear:
            return                                  # idempotent per (epoch, range): the repeat-once copy
        self._last_clear = pair
        if self._epoch_clear is None:
            self._epoch_clear = rng                 # the epoch start's declaration for the hand-over
            return
        for shapes in (self._shapes, self._cached):   # a different range mid-epoch clears at once
            for id_ in [i for i in shapes if _layer_of(i) > rng]:
                del shapes[id_]

    def _apply_hzn(self, rec, clk: tuple[int, bool]) -> None:
        first = self._abs is None and self._nohz is None
        if isinstance(rec, vs.HznAbs):
            if _wins(clk, self._abs[1] if self._abs else None):
                self._abs = (rec, clk)
            self._set_cols((rec.sky, rec.gnd), clk)
        elif isinstance(rec, vs.HznResid):
            if _wins(clk, self._resid[1] if self._resid else None):
                self._resid = (rec, clk)
        elif isinstance(rec, vs.HznColours):
            self._set_cols((rec.sky, rec.gnd), clk)
        else:
            if _wins(clk, self._nohz[1] if self._nohz else None):
                self._nohz = (rec, clk)
            self._set_cols((rec.top, rec.bottom), clk)
        if first and (self._abs is not None or self._nohz is not None):
            # The new epoch's anchor is the epoch start as far as the store can
            # see it (the key frame or its repeat-once copy): it ends a resync
            # and fixes the membership of shapes defined before it arrived.
            self._resync = False
            for sh in self._shapes.values():
                if sh.group is None:
                    sh.group = self._membership(sh)

    def _set_cols(self, pair, clk: tuple[int, bool]) -> None:
        if _wins(clk, self._cols[1] if self._cols else None):
            self._cols = (pair, clk)

    def _check_digest(self, rec: vs.Digest) -> None:
        self._st["digest_checks"] += 1
        self._digest_n_live = rec.n_live
        ok = self._digest() == rec.crc
        self._digest_ok = ok
        if ok:
            self._digest_run = 0
            return
        self._digest_run += 1
        self._st["digest_mismatch"] += 1
        if self._digest_run >= RESYNC_DIGEST_RUN:
            self._enter_resync()

    def _digest(self) -> int:
        shapes = [(sh.id, sh.dhash, self._state_hash(sh)) for sh in self._live()]
        gshifts = [self._gs[g].now((0, 0)) for g in range(4)]
        gain = self._gain[0] if self._gain else (16, 16, 16)
        return vs.digest_crc(shapes, gshifts, self._zoom.now(128), gain)

    def _state_hash(self, sh: _Shape) -> int:
        return vs.state_hash(sh.off[0], sh.off[1], self._rgb444(sh.fill),
                             [ins for ins, _ in sh.inserts.values()], sh.live_holes())

    def _apply_confirm(self, rec: vs.Confirm, clk: tuple[int, bool]) -> None:
        for i, tag in enumerate(rec.tags):
            if tag is None:
                continue
            self._items += 1
            sh = self._lookup(rec.base_id + i)
            if sh is None:
                self._orphan()
                continue
            # A 2-bit tag passes a stale state 25 % of the time; the DIGEST
            # bounds that, so a mismatching DIGEST blocks every reset (§4.3).
            # Only a tag that contradicts the stored state is an orphan: a
            # DIGEST mismatch has its own resync rule (3 consecutive), and
            # counting every blocked CONFIRM would trip the 20 % rule on one
            # lost UPD.
            if (self._state_hash(sh) & 3) != tag:
                self._orphan()
            elif self._digest_ok is not False and not self._resync:
                self._verify(sh, clk)

    # ------------------------------------------------------------ geometry and colour

    def _rgb444(self, fill: vs.Fill) -> int:
        return self._palette[fill.palette] if fill.palette is not None else fill.rgb444

    def _gain_codes(self) -> tuple[int, int, int]:
        return self._gain[0] if self._gain else (16, 16, 16)

    def _gained(self, rgb: tuple[int, int, int]) -> tuple[int, int, int]:
        return tuple(_clip8(c * _gain(g)) for c, g in zip(rgb, self._gain_codes()))

    def _base_rgb8(self, sh: _Shape) -> tuple[int, int, int]:
        return self._gained(sh.baked_rgb or _rgb8(self._rgb444(sh.fill)))

    def _abs_line(self) -> _Horizon | None:
        if self._abs is None:
            return None
        a = self._abs[0]
        return _Horizon(2 * a.y - 128, 0.5 * a.ang, 4 * a.curv)

    def _shown_line(self) -> _Horizon | None:
        """ABS, plus the newest RESID on top of it, shifted as a far-group member."""
        if self._abs is None:
            return None
        a, aclk = self._abs
        yc, ang = 2 * a.y - 128, 0.5 * a.ang
        if self._resid is not None and _wins(self._resid[1], aclk):
            yc += 2 * self._resid[0].dy
            ang += 0.5 * self._resid[0].dang
        sx, sy = self._group_shift_px(_GRP_FAR, aclk[0])
        return _Horizon(yc, ang, 4 * a.curv, sx, sy)

    def _anchor_mode(self) -> str:
        if self._nohz is not None and (self._abs is None or _wins(self._nohz[1], self._abs[1])):
            return "none"
        if self._abs is None:
            return "none"
        return "resid" if self._resid is not None and _wins(self._resid[1], self._abs[1]) else "abs"

    def _anchor_clk(self) -> tuple[int, bool] | None:
        """Latest L0 capture, or None until the epoch has an absolute anchor."""
        if self._abs is None and self._nohz is None:
            return None
        clocks = [f[1] for f in (self._abs, self._resid, self._cols, self._nohz) if f is not None]
        return max(clocks, key=lambda c: c[0])

    def _group_shift_px(self, grp: int | None, t: int) -> tuple[float, float]:
        """(S_all + S_grp) now − at capture t, in canvas px (2 px codes)."""
        ax, ay = self._gs[_GRP_ALL].now((0, 0))
        bx, by = self._gs[_GRP_ALL].at(t, (0, 0))
        dx, dy = ax - bx, ay - by
        if grp is not None and grp != _GRP_ALL:
            gx, gy = self._gs[grp].now((0, 0))
            hx, hy = self._gs[grp].at(t, (0, 0))
            dx, dy = dx + gx - hx, dy + gy - hy
        return 2.0 * dx, 2.0 * dy

    def _shift_px(self, sh: _Shape) -> tuple[float, float]:
        sx, sy = self._group_shift_px(sh.group, sh.define_clk[0])
        return sx + sh.carry_px[0], sy + sh.carry_px[1]

    def _zoom_u(self, sh: _Shape) -> float:
        if sh.group != _GRP_GROUND:
            return 0.0
        return (self._zoom.now(128) - self._zoom.at(sh.define_clk[0], 128)) / 4096.0

    def _place(self, sh: _Shape, pts, cached: bool):
        """vertices → local offset → group shift → group zoom about the
        vanishing point (§3.4 rule 5, §4.4: r' = r / (1 − u·r), x scaled by
        r'/r). Returns (points, scale at the first point, dropped)."""
        ox, oy = 4.0 * sh.off[0], 4.0 * sh.off[1]
        if cached:
            (sx, sy), u, line = sh.carry_px, 0.0, None
        else:
            sx, sy = self._shift_px(sh)
            u = self._zoom_u(sh)
            line = self._shown_line() if u else None
        out, scale, dropped = [], 1.0, False
        for i, (x, y) in enumerate(pts):
            x, y = x + ox + sx, y + oy + sy
            if line is not None:
                yh = line.y_at(x)
                r = y - yh
                if r > 0:
                    d = 1.0 - u * r
                    if d < 1.0 / ZOOM_DROP_RATIO:
                        dropped, d = True, 1.0 / ZOOM_DROP_RATIO
                    ratio = 1.0 / d
                    x, y = CANVAS_W / 2 + (x - CANVAS_W / 2) * ratio, yh + r * ratio
                    if i == 0:
                        scale = ratio
            out.append((x, y))
        if u and not dropped and all(x < 0 or x > self.canvas_w or y < 0 or y > self.canvas_h
                                     for x, y in out):
            dropped = True                          # projected off-frame (§4.4)
        return out, scale, dropped

    @staticmethod
    def _ring(poly: vs.Poly, inserts: dict) -> list[tuple[float, float]]:
        """Order-independent INSERT ring (§3.3): define vertices in order, and
        within each define edge e the received INSERTs sorted by k, each at the
        define-edge midpoint + (dx, dy). Any subset is a sub-ring."""
        n = len(poly.vertices)
        by_edge: dict[int, list] = {}
        for (e, k), (ins, _) in inserts.items():
            by_edge.setdefault(e, []).append((k, ins))
        ring: list[tuple[float, float]] = []
        for e in range(n):
            x0, y0 = poly.vertices[e]
            x1, y1 = poly.vertices[(e + 1) % n]
            ring.append((float(x0), float(y0)))
            for _, ins in sorted(by_edge.get(e, ()), key=lambda ki: ki[0]):
                ring.append(((x0 + x1) / 2 + ins.dx, (y0 + y1) / 2 + ins.dy))
        return ring

    def _fill_json(self, sh: _Shape, pts) -> dict:
        base = self._base_rgb8(sh)
        if not sh.fill.grad or len(pts) < 2:
            return {"c0": _hex(base)}
        dir3, dl = sh.fill.grad
        dL = 16 * dl                                # §3.3: ΔL = 16·c on every channel
        th = math.radians(dir3 * 22.5)
        dx, dy = math.cos(th), math.sin(th)
        proj = [x * dx + y * dy for x, y in pts]
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        cp = cx * dx + cy * dy
        lo, hi = min(proj) - cp, max(proj) - cp
        return {"c0": _hex(tuple(_clip8(c - dL) for c in base)),
                "c1": _hex(tuple(_clip8(c + dL) for c in base)),
                "g": [_r2(cx + dx * lo), _r2(cy + dy * lo), _r2(cx + dx * hi), _r2(cy + dy * hi)]}

    def _vfill_pair(self, vf: vs.VFill, above: bool) -> list[str]:
        """Band colours in screen order (top → bottom): base + ΔL at the band's
        top edge and base − ΔL at its bottom edge, for either band (§3.3 vfill:
        sky +ΔL at the frame top, −ΔL at the horizon; ground +ΔL at the
        horizon, −ΔL at the frame bottom) — the same reading the encoder fits."""
        base = self._gained(_rgb8(self._rgb444(vs.Fill(palette=vf.palette, rgb444=vf.rgb444))))
        dL = 8 * vf.dl
        top = _hex(tuple(_clip8(c + dL) for c in base))
        bottom = _hex(tuple(_clip8(c - dL) for c in base))
        return [top, bottom]

    def _shape_json(self, sh: _Shape, badge: int, now: int, cached: bool) -> tuple[dict, list[dict]]:
        d = sh.define
        out: dict[str, Any] = {"id": sh.id, "badge": badge, "age_ms": now - sh.verified[0]}
        dropped = False
        if isinstance(d, vs.Poly):
            cell = 8 if d.grid == 0 else 4
            pts, _, dropped = self._place(sh, [(x * cell, y * cell) for x, y in self._ring(d, sh.inserts)], cached)
            out["k"] = "poly"
            out["pts"] = [_r2(v) for p in pts for v in p]
            out["fill"] = self._fill_json(sh, pts)
            if sh.id in vs.ID_CORRIDOR:
                out["corridor"] = True
        elif isinstance(d, (vs.Tree, vs.Blob)):
            cell = 8 if isinstance(d, vs.Tree) else 4
            (c,), scale, dropped = self._place(sh, [(d.cx * cell, d.cy * cell)], cached)
            rx, ry = (d.rx + 1) * 4 * scale, (d.ry + 1) * 4 * scale
            bbox = [(c[0] - rx, c[1] - ry), (c[0] + rx, c[1] - ry), (c[0] + rx, c[1] + ry), (c[0] - rx, c[1] + ry)]
            out.update(k="tree" if isinstance(d, vs.Tree) else "blob", cx=_r2(c[0]), cy=_r2(c[1]),
                       rx=_r2(rx), ry=_r2(ry), fill=self._fill_json(sh, bbox))
            if isinstance(d, vs.Tree):
                out["trunk"] = None if d.trunk_h is None else {
                    "w": 4, "h": (d.trunk_h + 1) * 8, "c0": _hex(self._gained(_rgb8(self._palette[6])))}
            else:
                out["rot_deg"] = d.rot * 22.5
        elif isinstance(d, vs.Edge):
            pts, _, dropped = self._place(sh, [(x * 4, y * 4) for x, y in d.points], cached)
            out.update(k="edge", cls=_EDGE_CLS[d.cls], pts=[_r2(v) for p in pts for v in p])
        else:                                       # Anom box, refined by a later POLY on the same id
            (p0, p1), _, dropped = self._place(
                sh, [(d.x * 8, d.y * 8), ((d.x + d.w + 1) * 8, (d.y + d.h + 1) * 8)], cached)
            out.update(k="anom", corridor=True, fill={"c0": _hex(self._base_rgb8(sh))},
                       box=[_r2(p0[0]), _r2(p0[1]), _r2(p1[0] - p0[0]), _r2(p1[1] - p0[1])])
        if dropped:
            out["badge"] = BADGE_PREDICTED
        holes = []
        for slot, entry in enumerate(sh.holes):
            if entry and entry[0] is not None:
                h = entry[0]
                (c,), scale, _ = self._place(sh, [(h.cx * 4, h.cy * 4)], cached)
                holes.append({"id": sh.id, "k": "hole", "parent": sh.id, "slot": slot,
                              "cx": _r2(c[0]), "cy": _r2(c[1]), "r": _r2((h.r + 1) * 4 * scale),
                              "badge": out["badge"], "age_ms": out["age_ms"]})
        return out, holes

    def _horizon_json(self, cached: bool) -> dict:
        mode = self._anchor_mode()
        line = self._shown_line() if mode != "none" else None
        pts = line.pts() if line is not None else [[0.0, 128.0], [192.0, 128.0], [384.0, 128.0]]
        sky = ground = None
        if self._cols is not None:
            top, bottom = self._cols[0]
            sky, ground = self._vfill_pair(top, True), self._vfill_pair(bottom, False)
        skyline, skyline_fill = [], None
        if self._skyline is not None and line is not None:
            sk, sclk = self._skyline
            n, unit = len(sk.heights), 4 << sk.unit
            sx = self._group_shift_px(_GRP_FAR, sclk[0])[0]
            for i, h in enumerate(sk.heights):
                x = (i + 0.5) * CANVAS_W / n + sx
                skyline.append([_r2(x), _r2(line.y_at(x) - h * unit)])
            skyline_fill = {"c0": _hex(self._gained(_rgb8(self._rgb444(sk.fill))))}
        return {"mode": mode, "pts": pts, "sky": sky, "ground": ground, "skyline": skyline,
                "skyline_fill": skyline_fill, "anchor_clk": self._anchor_clk()}

    # ------------------------------------------------------------ snapshot (§7.2)

    def snapshot(self, now_ms: int) -> dict | None:
        """The ``vector_scene`` object, or None until a frame has been applied.
        During a pending hand-over the previous epoch is shown as CACHED (1)."""
        if self._epoch is None:
            return None
        cached = bool(self._cached)
        shapes = self._cached if cached else self._shapes
        badge = BADGE_CACHED if cached else BADGE_VECTOR
        per_layer: dict[int, list] = {1: [], 2: [], 3: [], 4: []}
        holes: list = []
        for sh in shapes.values():
            js, hs = self._shape_json(sh, badge, now_ms, cached)
            per_layer[_layer_of(sh.id)].append(js)
            holes += hs
        masses = [s for s in per_layer[1] if not s.get("corridor")]
        masses.sort(key=lambda s: -self._area(s["pts"]))     # L1 paints area-descending (§2.2)
        anoms = sorted((s for s in per_layer[1] if s.get("corridor")), key=lambda s: s["id"])
        per_layer[1] = masses + holes + anoms
        layers = []
        for n in (1, 2, 3, 4):
            items = per_layer[n] if n == 1 else sorted(per_layer[n], key=lambda s: s["id"])
            lb = badge
            if items and all(s["badge"] == BADGE_PREDICTED for s in items):
                lb = BADGE_PREDICTED
            layers.append({"id": "L%d" % n, "badge": lb, "shapes": items})
        horizon = dict(self._cached_horizon) if cached and self._cached_horizon else \
            (self._horizon_json(cached=False) if not cached else
             {"mode": "none", "pts": [[0.0, 128.0], [192.0, 128.0], [384.0, 128.0]], "sky": None,
              "ground": None, "skyline": [], "skyline_fill": None, "anchor_clk": None})
        anchor_clk = horizon.pop("anchor_clk")
        horizon["badge"] = badge
        horizon["age_ms"] = None if anchor_clk is None else now_ms - anchor_clk[0]
        status = None
        if self._status is not None:
            s, sclk = self._status
            status = {"arm_src": s.arm_src, "arm_deg": s.arm - 30, "bkt_src": s.bkt_src,
                      "bkt_deg": (s.bkt - 64) * 1.5, "conf": s.conf, "corr_n": s.corr_n,
                      "mask_anom": s.mask_anom, "moving": s.moving, "age_ms": now_ms - sclk[0]}
        cal = None if self._calrev is None else {"cal": self._calrev[0].cal, "mask": self._calrev[0].mask}
        return {
            "v": 1, "epoch": self._epoch, "badge": BADGE_VECTOR, "lab": False, "level": self._last_level,
            "handover": cached, "cached_epoch": self._cached_epoch if cached else None,
            "anchor_age_ms": horizon["age_ms"], "digest_ok": self._digest_ok, "resync": self._resync,
            "corr_detected": self._status[0].corr_n if self._status else 0,
            "corr_shown": len(anoms) + sum(1 for s in masses if s["id"] in vs.ID_CORRIDOR),
            "min_object": MIN_OBJECT_PLACEHOLDER,
            "bits": self._last_bits or {"last_frame": 0, "per_layer": {}},
            "horizon": horizon, "layers": layers, "masked": [],
            "warp": {"applied": False, "source": None, "since_capture_ms": 0},
            "status": status, "cal_rev": cal, "stats": self.stats,
        }

    @staticmethod
    def _area(flat: list[float]) -> float:
        n = len(flat) // 2
        if n < 3:
            return 0.0
        s = 0.0
        for i in range(n):
            x0, y0 = flat[2 * i], flat[2 * i + 1]
            x1, y1 = flat[(2 * i + 2) % (2 * n)], flat[(2 * i + 3) % (2 * n)]
            s += x0 * y1 - x1 * y0
        return abs(s) / 2.0
