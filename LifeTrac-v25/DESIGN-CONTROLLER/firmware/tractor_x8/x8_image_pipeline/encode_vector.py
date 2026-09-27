"""VS1 vector-scene encoder for the tractor X8 (``VECTOR_SCENE.md`` §2–§4).

``VectorEncoder.frame`` turns one camera frame into one complete
``TileDeltaFrame`` of codec 6: the 6 B header ``[frame_kind, seq, 12, 8, 32,
6]`` followed by a VS body that never exceeds ``budget_bytes − 6`` (one byte
less on an epoch-start frame, §3.1). ``camera_service`` publishes the bytes
on ``cmd/image_frame`` exactly like a tile frame (§7.1).

Pipeline per capture (§2.2): resize to the 96×64 working image → L0 horizon
and bands → L1 colour regions (L2 tree test inside) → **T**: match the
regions to the live shapes the base holds (a *mirror*, §4) by IoU on the
label maps and choose UPD / UCOL / redefine / DEL / CONFIRM → **P**: pack one
frame in the §4.2 priority order under the byte budget, then CONFIRM tags
and the DIGEST from the mirror.

What the mirror is. The encoder keeps, per live id, the define record as
sent, its define-hash, the cumulative UPD offset and the current fill —
the state the base holds if every frame arrived — and it applies only the
records that were actually packed, so a record that missed the budget is
simply a candidate again next frame. Every record describes the current
capture (principle 2): a repeated define is re-verified against the newest
regions first (IoU ≥ 0.7 and ΔE76 ≤ 6, §4.3), and a verification also passes
when the capture yields the identical define record, so a small shape whose
raster IoU is quantisation-limited still confirms deterministically.

Staying in step with the base (§3.4 rule 5, §4.3; the RS-13.1 bench found
the mirror drifting from the store on a loss-free path, anomalies A1/A2):

* the base drops a shape after ``TTL_FRAMES`` applied frames without a
  verifying record (a define, an UPD or a CONFIRM), so the mirror expires a
  shape on exactly that count, and every matched shape leaves the temporal
  pass with a record or a CONFIRM — kept, redefined or deleted, never silent;
* a same-hash define is a repeat that keeps the base's offset and colour,
  so an identical outline at a non-zero offset returns to 0 by an UPD and
  the mirror applies a same-hash define as a repeat;
* a repeat-once or carousel re-send carries the shape's current UPD and
  UCOL (once a shape has had either, always: a lost UPD back to 0 or a
  lost UCOL back to the define's fill would otherwise never heal), so a
  base that lost either, or dropped the shape, recovers at the next visit;
  after a range-1/2 epoch start every kept shape owes its re-statement —
  one slot-5 record carrying the define with the UPD and UCOL the shape
  has after that capture, in place of any bare UPD, UCOL or CONFIRM — and
  until it has gone out the shape is not counted by a DIGEST, CONFIRMs are
  reserved only for shapes half-way to their TTL and no DIGEST goes, so a
  base with stale state is repaired rather than tripped into resync and
  the repeats are never starved by the frame's own changes or its reserve;
* within a frame a shape's UPD / UCOL follow its define, so the repeat of
  a define the base missed lands before this frame's change to it;
* an id a DEL or a TTL drop freed cools down for TTL_FRAMES frames, and a
  fresh define that ever re-uses it states its offset and colour, because a
  base that missed the DEL still holds the old shape — for TTL_FRAMES of
  *applied* frames, which loss stretches beyond any window the encoder
  could count; after a range-1/2 start no DIGEST goes until such a ghost
  has certainly expired at the base, or the DIGEST itself would put a
  healthy base into resync;
* a same-id redefine competes with the live outline over the union of both
  rasters, not with itself over its own pixels: when the new outline is not
  worth its bits the live one is confirmed instead.

Epochs (§3.5, §4.3). A new epoch starts on the first frame, on
``force_epoch()`` / ``epoch_start=True``, when the horizon state flips
between found and NO_HORIZON, when more than 40 % of the valid area is
relabelled between captures, and every 60 s (safety refresh, LAYER_CLEAR
range 2, which keeps the masses). The id space is not a trigger: fresh
defines are scored first and get ids in score order, and a layer whose ids
run out keeps its best candidates while the rest wait for a DEL to free an
id (§4.3 lists ID exhaustion as a trigger; restarting the epoch on every
frame of a busy scene cost a key frame per frame on the bench and drew no
more shapes). A newcomer worth at least EVICT_VALUE_RATIO times the least
valuable live shape of its layer, on EVICT_STRIKES consecutive frames,
evicts that shape (a DEL, then its id) so the picture never freezes on
stale low-value shapes while a new object waits; a shape's value is the
error it removes over the region it describes after this capture, against
the mirror without it, the same measure a newcomer is scored by, and one
eviction runs at a time per layer. An epoch start is a
transaction: the frame is built as the new epoch's key frame, but the
counter, the mirror and the GAIN reset are committed only when the anchor
is actually packed; until then every frame is another attempt with the key
bit set, so a receiver is never left waiting for an epoch that never
anchored. The epoch-start records are repeated in the following frame(s)
per the level (§4.5.4), and a static scene converges to RESID + STATUS +
CONFIRM + DIGEST with a carousel frame every 1/κ frames.

Quality byte (§4.5.4): its band is the level (60–100 V0, 40–59 V1, 20–39 V2,
1–19 V3), which sets the frame body cap, κ and the repeat count; V3 sends
only the absolute anchor and STATUS. The position inside the band is the
detail scale, reported in ``last_stats`` (INSERT/L4 are not built yet).

Layers produced: L0 (HZN ABS / RESID / NO_HORIZON with measured vfills),
L1 POLY masses, L2 TREE/SHRUB ellipses, L3 EDGE polylines (rut / structure /
overhead classes, chamfer-matched), plus STATUS, GAIN, LAYER_CLEAR, UPD,
UCOL, DEL, CONFIRM and DIGEST.

Not implemented here (see the module report): INSERT/HOLE/L4, GSHIFT/GZOOM
ego-motion, ANOM, SKYLINE, PAL, CAL_REV, the fence/contact edge classes, the
sensed arm (STATUS sources are "none"), the weight map W and the self-mask
anomaly test.
"""
from __future__ import annotations

import os
import sys
import time
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

try:                                             # pragma: no cover
    import cv2                                   # type: ignore
except ImportError:                              # pragma: no cover
    cv2 = None                                   # type: ignore

# The codec: the tractor image is built from firmware/tractor_x8 alone
# (README-DEPLOY.md step 1), so it carries its own copy, vs1_codec.py, pinned
# byte-identical to base_station/image_pipeline/vector_scene/codec.py by
# tests/test_vs1_codec_parity_sil.py (the same arrangement as the codec-id
# table camera_service duplicates). The base tree is only a fallback for a
# source checkout whose mirror is missing.
try:
    from . import vs1_codec as vs
except ImportError:                              # loaded as a bare module, or no mirror beside us
    try:
        import vs1_codec as vs                   # type: ignore
    except ImportError:
        _BS_DIR = os.path.normpath(os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "base_station"))
        if _BS_DIR not in sys.path:
            sys.path.insert(0, _BS_DIR)
        from image_pipeline.vector_scene import codec as vs  # noqa: E402

try:
    from . import vector_extract as vx
except ImportError:                              # loaded as a bare module
    import vector_extract as vx                  # type: ignore

GRID_W, GRID_H, TILE_PX = 12, 8, 32              # TileDeltaFrame canvas geometry (§3)
HEADER_LEN = 6

# §4.5.4 ladder, indexed by level V0..V3.
LEVEL_BODY = (None, 100, 40, 20)                 # frame body cap F (None = the link budget)
LEVEL_KAPPA = (0.25, 0.50, 0.75, 0.0)            # carousel share
LEVEL_REPEATS = (1, 2, 3, 0)                     # epoch-start / define repeats
LEVEL_BANDS = ((60, 100), (40, 59), (20, 39), (1, 19))

SAFETY_REFRESH_S = 60.0                          # §4.3 new-epoch trigger
RELABEL_EPOCH_FRACTION = 0.40
MATCH_IOU = 0.5                                  # same id (§4 T)
VERIFY_IOU = 0.7                                 # re-verification (§4.3)
VERIFY_DE = 6.0
UPD_MIN_SHIFT_WORK = 1.5                         # working px before an UPD is worth its 17 bits
HZN_JUMP_DEG = 3.0                               # §2.4 temporal consistency
HZN_JUMP_PX = 32.0
MIN_DD_PER_PX = 48.0                             # squared 8-bit RGB error a define must remove per px
MAX_NEW_DEFINES = 24                             # per frame; the rest wait (disclosed omission)
EVICT_VALUE_RATIO = 2.0                          # a waiting newcomer must be worth this × the weakest holder
EVICT_STRIKES = 2                                # ... on this many consecutive frames before the holder goes
MAX_NEW_EDGES = 8
EDGE_MATCH_CELLS = 1.5                           # §2.7 item 5: keep the id up to 1.5 cells of chamfer
GAIN_NEUTRAL = (16, 16, 16)                      # §3.3 GAIN: gain = 2^((code − 16) / 32)
GAIN_MIN_AREA_SHARE = 0.25                       # of the labelled area, from regions matched to the last capture

STATUS_NONE = dict(arm_src=0, arm=30, bkt_src=0, bkt=64, conf=0, corr_n=0, mask_anom=False)


def level_of_quality(quality: int) -> int:
    """Quality byte band → level (§4.5.4): 60–100 V0, 40–59 V1, 20–39 V2, 1–19 V3."""
    q = max(1, min(100, int(quality)))
    for level, (lo, _) in enumerate(LEVEL_BANDS):
        if q >= lo:
            return level
    return 3


def detail_of_quality(quality: int) -> float:
    """Position inside the band, 0..1: scales the INSERT/L4 budget (§4.5.4)."""
    q = max(1, min(100, int(quality)))
    lo, hi = LEVEL_BANDS[level_of_quality(q)]
    return (q - lo) / float(max(hi - lo, 1))


@dataclass
class _Shape:
    """One live shape as the base holds it (the mirror)."""
    id: int
    define: object                  # vs.Poly | vs.Tree, absolute geometry as sent
    dhash: int
    fill: object                    # current vs.Fill (define fill or the last UCOL)
    layer: str                      # "mass" | "plant"
    raster: np.ndarray              # bool H×W at the current offset
    area: int
    cx: float                       # raster centroid, working px
    cy: float
    lab: np.ndarray                 # Lab of the *measured* mean behind the fill
    define_frame: int               # frame counter of the last define sent
    dx: int = 0                     # cumulative UPD, 4 px units (= working px)
    dy: int = 0
    repeat_left: int = 0
    frames_since_verify: int = 0    # the base's TTL clock, mirrored (§4.3)
    ever_upd: bool = False          # a re-send states the offset from now on, even at 0
    ever_ucol: bool = False         # a re-send states the colour from now on, even the define's

    @property
    def shown(self) -> np.ndarray:
        return vx.fill_shown_rgb8(self.fill)

    def state_hash(self) -> int:
        # An EDGE has no colour: its base colour field hashes as 0.
        base = vx.fill_base_rgb444(self.fill) if self.fill is not None else 0
        return vs.state_hash(self.dx, self.dy, base, [], [])


@dataclass
class _L0State:
    """The L0 the base holds; set only by the apply of the HZN record that carries it."""
    top_vf: object
    bot_vf: object
    top_lab: np.ndarray
    bot_lab: np.ndarray
    found: bool
    codes: Optional[tuple]          # (y8, ang6, curv4) of the epoch's ABS; None under NO_HORIZON
    shown_hz: object                # vx.Horizon the base draws


@dataclass
class _Cand:
    slot: int                       # §4.2 priority slot
    order: tuple                    # ascending within the slot
    record: object                  # a codec record, or the (anchor, LAYER_CLEAR) tuple of an epoch start
    bits: int
    apply: Callable[[], None]       # mutates the mirror when the record is packed
    mentions: tuple = ()            # ids the record names (no CONFIRM for those)
    needs: object = None            # a candidate that must be packed before this one


def _shift_mask(m: np.ndarray, sx: int, sy: int) -> np.ndarray:
    h, w = m.shape
    out = np.zeros_like(m)
    xs0, xs1 = max(0, -sx), min(w, w - sx)
    ys0, ys1 = max(0, -sy), min(h, h - sy)
    if xs1 > xs0 and ys1 > ys0:
        out[ys0 + sy:ys1 + sy, xs0 + sx:xs1 + sx] = m[ys0:ys1, xs0:xs1]
    return out


def _geometry_key(rec) -> tuple:
    if isinstance(rec, vs.Poly):
        return ("poly", rec.grid, rec.vertices)
    return ("tree", rec.cx, rec.cy, rec.rx, rec.ry, rec.trunk_h)


def _layer_of(rec) -> str:
    return "plant" if isinstance(rec, vs.Tree) else "mass"


def _bits_layer(rec) -> str:
    """Per-layer accounting key for ``last_stats['bits']``."""
    if isinstance(rec, (vs.HznAbs, vs.HznResid, vs.HznColours, vs.HznNoHorizon, vs.Skyline)):
        return "L0"
    if isinstance(rec, vs.Tree):
        return "L2"
    if isinstance(rec, vs.Edge):
        return "L3"
    if isinstance(rec, (vs.Poly, vs.Insert, vs.Hole)):
        return "L1"
    if isinstance(rec, (vs.Upd, vs.Ucol, vs.Del)):
        return "L2" if rec.id in vs.ID_PLANT else "L3" if rec.id in vs.ID_EDGE else "L1"
    if isinstance(rec, vs.Blob):
        return "L4"
    return "ctrl"


def _records_of(c: "_Cand") -> tuple:
    """A candidate's records: one, or the indivisible anchor + LAYER_CLEAR pair,
    or a define with the UPD / UCOL that re-state it."""
    return c.record if isinstance(c.record, tuple) else (c.record,)


def _defines_first(records: list) -> list:
    """The wire order of a frame with a shape's UPD / UCOL moved to just after
    its define when the define comes later (a slot-5 change to a shape whose
    define is this frame's slot-6 repeat of a lost frame): the base applies
    records in order and orphans an UPD naming an id it does not hold yet
    (§3.4 rule 4). Everything else keeps the packer's order."""
    define_at: dict = {}
    for i, rec in enumerate(records):
        if isinstance(rec, (vs.Poly, vs.Tree, vs.Edge)):
            define_at.setdefault(rec.id, i)
    held: dict = {}
    out: list = []
    for i, rec in enumerate(records):
        if isinstance(rec, (vs.Upd, vs.Ucol)) and define_at.get(rec.id, -1) > i:
            held.setdefault(rec.id, []).append(rec)
            continue
        out.append(rec)
        if isinstance(rec, (vs.Poly, vs.Tree, vs.Edge)) and rec.id in held:
            out.extend(held.pop(rec.id))
    return out


class VectorEncoder:
    """Camera frame → one VS1 ``TileDeltaFrame`` (codec 6). See the module doc."""

    def __init__(self, mask: "Optional[np.ndarray]" = None, canvas: tuple = (384, 256), *,
                 working: tuple = (96, 64), clock: Callable[[], float] = time.monotonic, k: int = 8):
        if cv2 is None:
            raise RuntimeError("encode_vector requires OpenCV on the tractor X8")
        if tuple(canvas) != (vx.CANVAS_W, vx.CANVAS_H) or tuple(working) != (vx.WORK_W, vx.WORK_H):
            raise ValueError("VS1 grids are defined for a 384×256 canvas at 96×64 working resolution")
        self.canvas = tuple(canvas)
        self.working = tuple(working)
        w, h = self.working
        if mask is None:
            self._mask = None
            self._valid = np.ones((h, w), bool)
        else:
            mask = np.asarray(mask).astype(bool)
            if mask.shape != (h, w):
                raise ValueError(f"mask must be {h}×{w} (working resolution), got {mask.shape}")
            self._mask = mask
            self._valid = ~mask
        self._clock = clock
        self._seg = vx.Segmenter(k=k)
        self._shapes: dict = {}
        self._epoch = -1
        self._epoch_t0 = clock()
        self._pending_epoch: Optional[int] = 0        # LAYER_CLEAR range of the next epoch start
        self._pending_reason: Optional[str] = "first"  # what asked for it (last_stats['epoch_trigger'])
        self._last_trigger: Optional[str] = None
        self._epochs_committed = 0
        self._clear_range = 0
        self._anchor_repeat_left = 0
        self._anchor_found: Optional[bool] = None     # the epoch anchor's state (ABS vs NO_HORIZON)
        self._anchor_codes: Optional[tuple] = None    # (y8, ang6, curv4) of the epoch's ABS
        self._shown_hz: Optional[vx.Horizon] = None   # geometry the base draws after the last frame
        self._top_vf = None
        self._bot_vf = None
        self._top_lab = None
        self._bot_lab = None
        self._hzn_last: Optional[vx.Horizon] = None
        self._hzn_pending: Optional[vx.Horizon] = None
        self._prev_region_map: Optional[np.ndarray] = None
        self._prev_region_lab = np.zeros((0, 3), np.float32)
        self._prev_region_raw = np.zeros((0, 3), np.float32)
        self._gain = GAIN_NEUTRAL                     # the epoch's GAIN codes as the base holds them
        self._gain_next = GAIN_NEUTRAL                # this capture's estimate
        self._frame_no = 0
        self._committed = False                       # this attempt's epoch start was packed
        self._carousel_acc = 0.0
        self._verified_now: set = set()               # ids this frame's packed records verify at the base
        self._id_freed: dict = {}                     # id -> (frame, dhash, offset, fill) at its DEL / expiry
        self._ttl_dropped = 0
        self._restate_owed: set = set()               # kept shapes not yet re-stated after a range-1/2 start
        self._restate_all = False                     # the key frame of a range-1/2 attempt: every kept shape owes
        self._evict_strikes: dict = {}                # layer -> consecutive frames a newcomer beat its weakest holder
        self._evict_hold: dict = {}                   # layer -> frame until which no eviction runs (an evictee took its id back)
        self._evicted: dict = {}                      # id -> raster of the shape evicted from it
        self._digest_quiet_until = -1                 # after a range-1/2 start: no DIGEST while a base may hold a missed-DEL ghost
        self._evict_next: set = set()                 # ids DEL'd next frame to free their id
        self._waiting = 0                             # fresh defines refused an id in the last frame
        self._last_stats: dict = {}

    # ------------------------------------------------------------ public API

    @property
    def epoch(self) -> int:
        return max(self._epoch, 0)

    @property
    def last_stats(self) -> dict:
        return self._last_stats

    def force_epoch(self) -> None:
        """The next frame starts a new epoch (LAYER_CLEAR 0)."""
        self._pending_epoch = 0
        self._pending_reason = "forced"

    def frame(self, rgb: np.ndarray, budget_bytes: int, *, epoch_start: bool = False, quality: int = 80,
              moving: bool = False, age_units: int = 0, seq: int = 0) -> bytes:
        """Encode one capture. ``rgb`` is H×W×3 uint8 at any size; ``budget_bytes``
        is the whole-payload budget (the retained ``tractor/link_budget``)."""
        t0 = time.perf_counter()
        ms: dict = {}
        self._frame_no += 1
        rgb = np.asarray(rgb)
        if rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError("rgb must be H×W×3")
        if rgb.dtype != np.uint8:
            rgb = np.clip(rgb, 0, 255).astype(np.uint8)
        work = vx.resize_working(np.ascontiguousarray(rgb), self.working)
        t1 = time.perf_counter()
        ms["resize"] = (t1 - t0) * 1000.0
        hz = self._temporal_horizon(vx.extract_horizon(work, self._valid))
        t2 = time.perf_counter()
        ms["l0"] = (t2 - t1) * 1000.0
        lab = vx.rgb_to_lab(work)
        regions, region_map = vx.extract_regions(work, lab, self._valid, self._mask, self._seg, hz)
        t3 = time.perf_counter()
        ms["l1"] = (t3 - t2) * 1000.0
        rgb192 = vx.resize_working(np.ascontiguousarray(rgb), (vx.EDGE_W, vx.EDGE_H))
        edges = vx.extract_edges(rgb192, self._mask, regions, self._valid.shape, hz)
        t3b = time.perf_counter()
        ms["l3"] = (t3b - t3) * 1000.0
        # Global gain (§2.1 item 3): the exposure step since the last capture,
        # accumulated into the epoch's absolute codes; colours are compared and
        # sent in the reference exposure so an AE step costs one GAIN record.
        self._gain_next = self._gain_codes(regions, region_map)
        self._normalise(regions, self._gain_next)
        level = level_of_quality(quality)
        now = self._clock()
        clear = self._epoch_trigger(epoch_start, hz, region_map, regions, now)
        body, records, stats = self._build(work, hz, regions, region_map, edges, clear, level, moving,
                                           budget_bytes, age_units, now)
        t4 = time.perf_counter()
        ms["temporal_pack"] = (t4 - t3b) * 1000.0
        self._prev_region_map = region_map
        self._prev_region_lab = np.stack([r.lab for r in regions]) if regions else np.zeros((0, 3), np.float32)
        self._prev_region_raw = np.stack([r.mean for r in regions]) if regions else np.zeros((0, 3), np.float32)
        key = stats["epoch_start"]
        header = bytes([1 if key else 0, seq & 0xFF, GRID_W, GRID_H, TILE_PX, vs.CODEC_VECTOR])
        stats["ms"] = ms
        stats["ms"]["total"] = (time.perf_counter() - t0) * 1000.0
        stats["level"] = level
        stats["detail"] = detail_of_quality(quality)
        stats["horizon"] = {"found": hz.found, "y0": hz.y0, "angle_deg": hz.angle_deg, "sag": hz.sag,
                            "support": hz.support}
        stats["n_regions"] = len(regions)
        stats["frame_bytes"] = HEADER_LEN + len(body)
        stats["epochs"] = self._epochs_committed
        stats["epoch_trigger"] = (self._last_trigger if stats["epoch_committed"]
                                  else (self._pending_reason if stats["epoch_start"] else None))
        stats["last_epoch_trigger"] = self._last_trigger       # the last committed start's cause
        stats["waiting"] = self._waiting
        self._last_stats = stats
        return header + body

    # ------------------------------------------------------------ L0 state

    def _temporal_horizon(self, hz: vx.Horizon) -> vx.Horizon:
        """§2.4 item 4: a jump (> 3°, > 32 px, or a found/NO_HORIZON flip) is
        accepted only when two consecutive captures agree; the previous
        geometry is kept meanwhile, with this capture's colours."""
        last = self._hzn_last
        if last is None:
            self._hzn_last = hz
            return hz

        def agrees(a: vx.Horizon, b: vx.Horizon) -> bool:
            if a.found != b.found:
                return False
            if not a.found:
                return True
            return abs(a.angle_deg - b.angle_deg) <= HZN_JUMP_DEG and abs(a.y0 - b.y0) <= HZN_JUMP_PX

        if agrees(hz, last):
            self._hzn_pending = None
            self._hzn_last = hz
            return hz
        if self._hzn_pending is not None and agrees(hz, self._hzn_pending):
            self._hzn_pending = None                    # seen in two consecutive captures
            self._hzn_last = hz
            return hz
        self._hzn_pending = hz
        kept = vx.Horizon(last.found, last.y0, last.slope, last.sag, last.support, last.columns,
                          hz.top, hz.bottom, hz.top_dl, hz.bottom_dl, hz.sky)
        return kept

    @staticmethod
    def _abs_codes(hz: vx.Horizon) -> tuple:
        y8 = max(0, min(255, int(round((hz.y0 + 128.0) / 2.0))))
        ang = max(-32, min(31, int(round(hz.angle_deg / 0.5))))
        curv = max(-8, min(7, int(round(hz.sag / 4.0))))
        return y8, ang, curv

    def _anchor_record(self, hz: vx.Horizon) -> tuple:
        """A fresh absolute anchor (ABS or NO_HORIZON) from this capture and the
        L0 state it re-anchors; the state is committed by the record's apply."""
        top_vf = vx.make_vfill(hz.top, int(round(hz.top_dl / 8.0)))
        bot_vf = vx.make_vfill(hz.bottom, int(round(hz.bottom_dl / 8.0)))
        top_lab, bot_lab = vx.rgb_to_lab(hz.top), vx.rgb_to_lab(hz.bottom)
        if hz.found:
            codes = self._abs_codes(hz)
            state = _L0State(top_vf, bot_vf, top_lab, bot_lab, True, codes, vx.horizon_from_codes(*codes))
            return vs.HznAbs(codes[0], codes[1], codes[2], top_vf, bot_vf), state
        state = _L0State(top_vf, bot_vf, top_lab, bot_lab, False, None, vx.Horizon(False))
        return vs.HznNoHorizon(top_vf, bot_vf), state

    def _l0_state(self) -> Optional[_L0State]:
        if self._top_vf is None:
            return None                                  # no anchor has gone out yet
        return _L0State(self._top_vf, self._bot_vf, self._top_lab, self._bot_lab, bool(self._anchor_found),
                        self._anchor_codes, self._shown_hz)

    def _set_l0(self, state: _L0State) -> None:
        self._top_vf, self._bot_vf = state.top_vf, state.bot_vf
        self._top_lab, self._bot_lab = state.top_lab, state.bot_lab
        self._anchor_found = state.found
        self._anchor_codes = state.codes
        self._shown_hz = state.shown_hz

    def _horizon_records(self, hz: vx.Horizon, key: bool, level: int, clear: Optional[int]) -> tuple:
        """(records, the L0 state they leave at the base). Slot 1 on an
        epoch-start attempt or its repeat (anchor + LAYER_CLEAR), else slot 3:
        RESID when the change fits its fields, ABS on a larger move or a colour
        stop change of ΔE > 6, nothing in a NO_HORIZON epoch. At V3 every frame
        is a beacon carrying the absolute anchor (§4.5.4). The state is
        committed only by the packed record's apply, so a dropped ABS never
        becomes the reference of later RESIDs and a dropped epoch start
        leaves the old anchor in place."""
        if key or self._anchor_repeat_left > 0 or self._anchor_found is None:
            rec, state = self._anchor_record(hz)
            return [rec, vs.LayerClear(clear if key else self._clear_range)], state
        if level >= 3:
            rec, state = self._anchor_record(hz)
            return [rec], state
        current = self._l0_state()
        colour_jump = (vx.delta_e76(vx.rgb_to_lab(hz.top), self._top_lab) > VERIFY_DE
                       or vx.delta_e76(vx.rgb_to_lab(hz.bottom), self._bot_lab) > VERIFY_DE)
        if not hz.found:
            if colour_jump:
                rec, state = self._anchor_record(hz)
                return [rec], state
            return [], current
        y8, ang, curv = self._abs_codes(hz)
        ay, aa, ac = self._anchor_codes
        dy, dang = y8 - ay, ang - aa
        if colour_jump or curv != ac or not (-16 <= dy <= 15) or not (-8 <= dang <= 7):
            rec, state = self._anchor_record(hz)
            return [rec], state
        state = _L0State(current.top_vf, current.bot_vf, current.top_lab, current.bot_lab, True, current.codes,
                         vx.horizon_from_codes(ay + dy, aa + dang, ac))
        return [vs.HznResid(dy, dang)], state

    # ------------------------------------------------------------ epochs

    def _epoch_trigger(self, epoch_start: bool, hz: vx.Horizon, region_map: np.ndarray, regions: list,
                       now: float) -> Optional[int]:
        """LAYER_CLEAR range of the epoch this frame starts, or None (§4.3). A
        trigger stays pending until an attempt's anchor is packed; a stronger
        trigger (a lower range) joins a pending weaker one. The trigger's
        name is reported as ``last_stats['epoch_trigger']``."""
        clear = reason = None
        if self._epoch < 0:
            clear, reason = 0, "first"
        elif epoch_start:
            clear, reason = 0, "forced"
        elif self._anchor_found is not None and hz.found != self._anchor_found:
            clear, reason = 0, "horizon"
        elif now - self._epoch_t0 >= SAFETY_REFRESH_S:
            clear, reason = 2, "safety"
        elif self._pending_epoch != 0:
            prev = self._prev_region_map
            if prev is not None and self._relabelled_fraction(prev, self._prev_region_lab, region_map,
                                                              regions) > RELABEL_EPOCH_FRACTION:
                clear, reason = 0, "relabel"
        if clear is not None:
            if self._pending_epoch is None or clear < self._pending_epoch:
                self._pending_reason = reason
            self._pending_epoch = clear if self._pending_epoch is None else min(self._pending_epoch, clear)
        return self._pending_epoch

    @staticmethod
    def _gain_lin(codes: tuple) -> np.ndarray:
        return np.array([2.0 ** ((c - 16) / 32.0) for c in codes], np.float32)

    def _gain_codes(self, regions: list, region_map: np.ndarray) -> tuple:
        """GAIN codes for this capture: the current codes plus the per-channel
        log-gain step measured as the area-weighted median ratio of raw means
        over regions that overlap a region of the last capture by IoU ≥ 0.5.
        Unchanged unless those regions cover GAIN_MIN_AREA_SHARE of the area."""
        prev = self._prev_region_map
        if prev is None or not regions or len(self._prev_region_raw) == 0:
            return self._gain
        np_, nc = int(prev.max()) + 2, int(region_map.max()) + 2
        pair = np.bincount((prev.ravel() + 1) * nc + (region_map.ravel() + 1), minlength=np_ * nc).reshape(np_, nc)
        area_c, area_p = pair.sum(axis=0), pair.sum(axis=1)
        steps, weights = [], []
        for r in regions:
            c = r.index + 1
            if c >= nc or area_c[c] == 0:
                continue
            p = int(np.argmax(pair[1:, c])) + 1
            inter = pair[p, c]
            union = area_c[c] + area_p[p] - inter
            old = self._prev_region_raw[p - 1]
            if union <= 0 or inter / union < MATCH_IOU or old.min() < 8 or r.mean.min() < 8 \
                    or old.max() > 245 or r.mean.max() > 245:                # clipping breaks the ratio
                continue
            steps.append(np.log2(r.mean / old))
            weights.append(float(area_c[c]))
        total = float(area_c[1:].sum())
        if not steps or total <= 0 or sum(weights) < GAIN_MIN_AREA_SHARE * total:
            return self._gain
        steps_a, w = np.array(steps), np.array(weights)
        order = np.argsort(steps_a, axis=0)
        med = []
        for ch in range(3):
            cw = np.cumsum(w[order[:, ch]])
            med.append(float(steps_a[order[:, ch], ch][int(np.searchsorted(cw, cw[-1] / 2.0))]))
        cur = np.log2(self._gain_lin(self._gain))
        return tuple(int(max(0, min(31, round(16 + 32.0 * (cur[ch] + med[ch]))))) for ch in range(3))

    def _normalise(self, regions: list, codes: tuple) -> None:
        """Express every region's colour in the epoch's reference exposure."""
        g = self._gain_lin(codes)
        for r in regions:
            norm = np.clip(r.mean / g, 0.0, 255.0)
            r.lab = vx.rgb_to_lab(norm)
            r.fill = vx.make_fill(norm, r.fill.grad if r.fill is not None else None)

    @staticmethod
    def _relabelled_fraction(prev: np.ndarray, prev_lab: np.ndarray, cur: np.ndarray, regions: list) -> float:
        """Share of the labelled area whose region has no partner in the
        previous capture with IoU ≥ 0.5 and ΔE76 ≤ 6 (the "40 % relabelled"
        trigger of §4.3)."""
        np_, nc = int(prev.max()) + 2, int(cur.max()) + 2
        pair = np.bincount((prev.ravel() + 1) * nc + (cur.ravel() + 1), minlength=np_ * nc).reshape(np_, nc)
        area_c = pair.sum(axis=0)
        area_p = pair.sum(axis=1)
        total = float(area_c[1:].sum())
        if total <= 0 or np_ < 2:
            return 1.0 if total > 0 else 0.0
        relabelled = 0.0
        for r in regions:
            c = r.index + 1
            if c >= nc or area_c[c] == 0:
                continue
            p = int(np.argmax(pair[1:, c])) + 1
            inter = pair[p, c]
            union = area_c[c] + area_p[p] - inter
            if union <= 0 or inter / union < MATCH_IOU \
                    or float(vx.delta_e76(r.lab, prev_lab[p - 1])) > VERIFY_DE:
                relabelled += float(area_c[c])
        return relabelled / total

    def _commit_epoch(self, clear: int, level: int, now: float) -> None:
        """The epoch start went out: advance the counter and start the epoch's
        clocks. The cleared layers were removed from the mirror when the
        attempt was built (``_build``), and a failed attempt puts the old
        mirror back, so only the counters live here."""
        self._epoch = (self._epoch + 1) & 15
        self._epoch_t0 = now
        self._clear_range = clear
        self._pending_epoch = None
        self._last_trigger, self._pending_reason = self._pending_reason, None
        self._epochs_committed += 1
        self._anchor_repeat_left = LEVEL_REPEATS[level]
        if clear == 0:                                # every mass is redefined: a fresh reference exposure
            self._gain = self._gain_next = GAIN_NEUTRAL
            self._restate_owed = set()
            self._digest_quiet_until = -1
        else:
            # The kept layers are re-stated in the following frames like the
            # epoch-start records (§3.5): every live shape owes a repeat-once
            # that carries its current UPD and UCOL, so a base that lost
            # state, or sat in resync, holds the epoch's mirror within the
            # safety period with no uplink (§4.3). Until a shape's repeat has
            # gone out it is neither CONFIRMed nor counted by a DIGEST: the
            # base may hold stale state from before the refresh, and a tag
            # or crc that contradicts it would count orphans and mismatches
            # against a base that is about to be repaired.
            for s in self._shapes.values():
                s.repeat_left = max(s.repeat_left, LEVEL_REPEATS[level])
            self._restate_owed = set(self._shapes)
            # A base that missed a DEL still holds that shape for TTL_FRAMES
            # of its own applied frames; a DIGEST naming the live set without
            # it mismatches there and, three in a row, puts a healthy base
            # into resync (§4.3), which only the next epoch start ends. So no
            # DIGEST goes until every id freed inside the last TTL_FRAMES
            # frames has certainly expired at the base.
            young = [fr[0] for i, fr in self._id_freed.items()
                     if i not in self._shapes and self._frame_no - fr[0] <= vs.TTL_FRAMES]
            self._digest_quiet_until = (max(young) + vs.TTL_FRAMES + 3) if young else -1
        self._committed = True

    def _alloc_id(self, layer: str, taken: set) -> Optional[int]:
        """The lowest free id of the layer, or None when every id is live so
        the region waits. An id a DEL or a TTL drop freed is passed over for
        TTL_FRAMES frames unless nothing else is free: a base that missed the
        DEL still holds the old shape until its own TTL, and a same-hash
        define for that id would keep the old shape's offset (§3.4 rule 5)."""
        rng = {"plant": vs.ID_PLANT, "edge": vs.ID_EDGE}.get(layer, vs.ID_MASS)
        cooling = None                                       # (frame freed, id): the oldest as last resort
        for id_ in rng:
            if id_ in self._shapes or id_ in taken:
                continue
            freed = self._id_freed.get(id_)
            if freed is not None and self._frame_no - freed[0] <= vs.TTL_FRAMES:
                if cooling is None or freed[0] < cooling[0]:
                    cooling = (freed[0], id_)
                continue
            return id_
        return cooling[1] if cooling is not None else None

    def _release_id(self, id_: int) -> None:
        """A DEL went out, or the shape expired: the id starts its reuse
        cooldown, remembering the state a base that missed the DEL still
        holds under it (``_fresh_define_records``)."""
        s = self._shapes.pop(id_, None)
        if s is not None:
            self._id_freed[id_] = (self._frame_no, s.dhash, (s.dx, s.dy), s.fill, s.ever_upd, s.ever_ucol)
        self._restate_owed.discard(id_)
        self._evict_next.discard(id_)

    def _fresh_define_records(self, rec) -> tuple:
        """(records, ever_upd, ever_ucol) for a fresh define. A base that
        missed the DEL that freed this id still holds the old shape, and a
        define it reads as same-hash would keep that shape's offset and
        colour (§3.4 rule 5) — whether this define's hash equals the
        mirror's last one or an earlier one the base never saw replaced. So
        a define that re-uses a freed id states the fresh shape's offset and
        colour whenever the old shape ever had an UPD or a UCOL. There is no
        time window: the base keeps the ghost for TTL_FRAMES of *applied*
        frames, which loss stretches past any count the encoder could keep,
        and the price is one UPD / UCOL per re-use."""
        freed = self._id_freed.get(rec.id)
        if freed is None:
            return (rec,), False, False
        _, _, off, fill, had_upd, had_ucol = freed
        recs, ever_upd, ever_ucol = [rec], False, False
        if off != (0, 0) or had_upd:
            recs.append(vs.Upd(rec.id, 0, 0))
            ever_upd = True
        if fill is not None and getattr(rec, "fill", None) is not None and (fill != rec.fill or had_ucol):
            recs.append(vs.Ucol(rec.id, rec.fill))
            ever_ucol = True
        return tuple(recs), ever_upd, ever_ucol

    # ------------------------------------------------------------ T + P

    def _masses(self) -> list:
        return [s for s in self._shapes.values() if s.layer != "edge"]

    def _build(self, work: np.ndarray, hz: vx.Horizon, regions: list, region_map: np.ndarray, edges: list,
               clear: Optional[int], level: int, moving: bool, budget_bytes: int, age_units: int,
               now: float) -> tuple:
        """One frame. An epoch start is a transaction (§3.5): the attempt is
        built on the mirror the new epoch would have (the cleared layers
        gone, their ids free) with the key bit set, the F − 1 cap, the anchor
        + LAYER_CLEAR at the head and every other record but STATUS needing
        the anchor; the epoch — counter, clocks, clear range, repeats, GAIN
        reset, mirror — is committed by the anchor candidate's apply, so an
        attempt whose anchor did not fit leaves nothing behind and is made
        again next frame, key bit still set, until it fits."""
        key = clear is not None
        saved_shapes = self._shapes
        self._committed = False
        if key:
            self._pending_epoch = clear if self._pending_epoch is None else min(self._pending_epoch, clear)
            dropped = {0: ("mass", "plant", "edge"), 1: ("plant", "edge"), 2: ("edge",), 3: ()}[clear]
            self._shapes = {i: s for i, s in saved_shapes.items() if s.layer not in dropped}
        try:
            body, records, stats = self._build_attempt(work, hz, regions, region_map, edges, clear, level,
                                                       moving, budget_bytes, age_units, now)
        finally:
            if key and not self._committed:
                self._shapes = saved_shapes                  # the attempt failed: the old epoch stands
        # The base ages every shape this frame did not verify, after checking
        # the frame's DIGEST, and drops it at TTL_FRAMES; so does the mirror.
        self._tick_ttl()
        # The residual is measured against what the base holds after this frame.
        h, w = self._valid.shape
        if self._top_vf is not None:
            mirror = vx.render_l0(self._shown_hz, self._top_vf, self._bot_vf, (h, w))
        else:
            mirror = np.zeros((h, w, 3), np.float32)         # no anchor has gone out yet
        gain = self._gain_lin(self._gain)
        for s in sorted(self._masses(), key=lambda s: -s.area):
            mirror[s.raster] = np.clip(s.shown * gain, 0.0, 255.0)
        err = ((work.astype(np.float32) - mirror) ** 2).sum(axis=2)
        stats["residual"] = float(err[self._valid].mean()) if self._valid.any() else 0.0
        stats["n_live"] = len(self._shapes)
        stats["ttl_dropped"] = self._ttl_dropped
        stats["epoch_committed"] = self._committed
        stats["epoch_pending"] = self._pending_epoch
        return body, records, stats

    def _build_attempt(self, work: np.ndarray, hz: vx.Horizon, regions: list, region_map: np.ndarray,
                       edges: list, clear: Optional[int], level: int, moving: bool, budget_bytes: int,
                       age_units: int, now: float) -> tuple:
        key = clear is not None
        if key and clear == 0:
            self._gain_next = GAIN_NEUTRAL               # the new epoch's reference exposure is this capture
            self._normalise(regions, GAIN_NEUTRAL)
        base_gain = GAIN_NEUTRAL if (key and clear == 0) else self._gain   # the base's GAIN once this frame applies
        f_bytes = max(0, int(budget_bytes) - HEADER_LEN)
        if LEVEL_BODY[level] is not None:
            f_bytes = min(f_bytes, LEVEL_BODY[level])
        if key:
            f_bytes -= 1                                     # §3.1: a copied epoch start stays one fragment
        f_bytes = max(f_bytes, 0)
        total_bits = max(f_bytes * 8 - vs.HEADER_BITS, 0)
        h, w = self._valid.shape
        epoch_no = ((self._epoch + 1) & 15) if key else self._epoch    # the number the header carries
        cands: list = []
        hzn, l0_state = self._horizon_records(hz, key, level, clear)
        anchor_cand = None
        if len(hzn) == 2:
            # An epoch start, or its repeat-once, is the anchor + LAYER_CLEAR as
            # ONE candidate: the base switches epochs on the anchor and drops
            # the cleared layers at hand-over by the clear range (§3.3, §3.5),
            # so a frame carrying one without the other would switch without
            # the clear. Both go, or neither and the attempt stays pending.
            if key:                                          # the epoch start commits the epoch
                apply = lambda: (self._commit_epoch(clear, level, now), self._set_l0(l0_state))
            else:                                            # its repeat
                apply = lambda: (self._spend_anchor_repeat(), self._set_l0(l0_state))
            cands.append(_Cand(1, (0,), tuple(hzn), sum(vs.record_bits(r) for r in hzn), apply))
            if key:
                anchor_cand = cands[-1]
        elif hzn:                                            # RESID / mid-epoch ABS, or the V3 beacon: slot 3
            cands.append(_Cand(3, (0,), hzn[0], vs.record_bits(hzn[0]), lambda: self._set_l0(l0_state)))
        if self._gain_next != base_gain or (key and self._gain_next != GAIN_NEUTRAL):
            gain = vs.Gain(*self._gain_next)             # absolute since the epoch start (§3.3)
            cands.append(_Cand(3, (9,), gain, vs.record_bits(gain), lambda: setattr(self, "_gain", self._gain_next)))
        status = vs.Status(moving=bool(moving), **STATUS_NONE)
        cands.append(_Cand(4, (0,), status, vs.record_bits(status), lambda: None))
        # ΔD is scored against the L0 this frame's HZN record leaves (it precedes every define).
        if l0_state is not None:
            l0 = vx.render_l0(l0_state.shown_hz, l0_state.top_vf, l0_state.bot_vf, (h, w))
        else:
            l0 = np.zeros((h, w, 3), np.float32)
        verified: set = set()
        self._waiting = 0
        # The key frame of a range-1/2 start: every kept shape owes its
        # re-statement from this frame on (the commit that records the owed
        # set happens inside _pack, after the candidates are built).
        self._restate_all = key and clear is not None and clear >= 1
        try:
            if level < 3:                                    # V3 is the anchor + STATUS beacon only
                self._temporal_candidates(work, l0, regions, region_map, level, key, cands, verified)
                self._edge_candidates(edges, level, cands, verified)
        finally:
            self._restate_all = False
        if anchor_cand is not None:
            # Nothing of the new epoch goes out ahead of its anchor: a define,
            # GAIN or CONFIRM in a frame whose anchor was dropped would name an
            # epoch the base cannot hand over to. STATUS is epoch-free and rides.
            for c in cands:
                if c is not anchor_cand and c.needs is None and not isinstance(c.record, vs.Status):
                    c.needs = anchor_cand
        carousel_bits = int(LEVEL_KAPPA[level] * total_bits) if self._carousel_due(level) else 0
        # While kept shapes owe their re-statement (the key frame of a
        # range-1/2 start and the frames after it until every repeat went
        # out) the budget goes to the repeats: CONFIRMs are reserved and sent
        # only for shapes half-way to their TTL, and no DIGEST is reserved
        # or sent, so at a small budget the reserve cannot starve the
        # repeats until the kept layer expires (§3.5, §4.3).
        restating = (bool(self._restate_owed) or (key and clear is not None and clear >= 1)
                     or self._frame_no <= self._digest_quiet_until)
        eligible = [i for i in sorted(verified)
                    if not restating or self._shapes[i].frames_since_verify >= vs.TTL_FRAMES // 2]
        reserve = self._confirm_bits(eligible) + (0 if restating else vs.record_bits(vs.Digest(0, 0)))
        packed, used = self._pack(cands, total_bits, reserve, carousel_bits)
        for c in packed:
            c.apply()
        mentioned = {id_ for c in packed for id_ in c.mentions}
        records = _defines_first([r for c in packed for r in _records_of(c)])
        for rec in records:
            if isinstance(rec, (vs.Poly, vs.Tree, vs.Edge)):
                self._restate_owed.discard(rec.id)           # re-stated: the base holds the mirror's shape
        if level < 3 and (not key or self._committed):       # CONFIRM and DIGEST describe the epoch's mirror
            unmentioned = [i for i in eligible if i not in mentioned and i in self._shapes
                           and i not in self._restate_owed]
            for rec in self._confirm_records(unmentioned):
                if used + vs.record_bits(rec) <= total_bits:
                    records.append(rec)
                    used += vs.record_bits(rec)
            if not self._restate_owed and self._frame_no > self._digest_quiet_until:   # see _commit_epoch
                digest = vs.Digest(len(self._shapes), vs.digest_crc(
                    [(s.id, s.dhash, s.state_hash()) for s in self._shapes.values()],
                    [(0, 0)] * 4, 128, self._gain))
                if used + vs.record_bits(digest) <= total_bits:
                    records.append(digest)
                    used += vs.record_bits(digest)
        # What this frame verifies at the base (§4.3): a define (new, repeat
        # or carousel), an UPD, or a CONFIRM tag that went out. A UCOL does
        # not, and a DEL removes. The mirror's TTL clock runs on this set.
        verified_now: set = set()
        for c in packed:
            for rec in _records_of(c):
                if isinstance(rec, (vs.Poly, vs.Tree, vs.Edge, vs.Upd)):
                    verified_now.add(rec.id)
        for rec in records:
            if isinstance(rec, vs.Confirm):
                verified_now.update(rec.base_id + i for i, t in enumerate(rec.tags) if t is not None)
        self._verified_now = verified_now
        header = vs.Header(key=key, age=max(0, min(15, int(age_units))), epoch=epoch_no, level=level)
        body = vs.encode_frame(header, records, f_bytes) if f_bytes >= 2 else b""
        bits = {"hdr": vs.HEADER_BITS, "L0": 0, "L1": 0, "L2": 0, "L3": 0, "L4": 0, "ctrl": 0}
        counts: dict = {}
        for rec in records:
            bits[_bits_layer(rec)] += vs.record_bits(rec)
            counts[type(rec).__name__] = counts.get(type(rec).__name__, 0) + 1
        bits["pad"] = len(body) * 8 - sum(bits.values()) if body else 0
        packed_ids = {id(c) for c in packed}
        stats = {"epoch": epoch_no, "epoch_start": key, "f_bytes": f_bytes, "body_bytes": len(body),
                 "record_bits": used + vs.HEADER_BITS, "bits": bits, "records": counts,
                 "n_candidates": len(cands), "n_packed": len(packed), "carousel_bits": carousel_bits,
                 "candidates": [(c.slot, "+".join(type(r).__name__ for r in _records_of(c)), c.bits, c.order,
                                 id(c) in packed_ids)
                                for c in cands]}
        return body, records, stats

    def _carousel_due(self, level: int) -> bool:
        """κ of §4.2 as a duty cycle: one carousel frame every 1/κ frames on
        average, each spending up to κ of the frame (the ≈ 40 B carousel frame
        of §4.2's static-accumulation note)."""
        self._carousel_acc += LEVEL_KAPPA[level]
        if self._carousel_acc >= 1.0:
            self._carousel_acc -= 1.0
            return True
        return False

    def _temporal_candidates(self, work: np.ndarray, l0: np.ndarray, regions: list, region_map: np.ndarray,
                             level: int, key: bool, cands: list, verified: set) -> None:
        """Slots 5–7: match regions to live shapes, then Del / Upd / Ucol /
        redefine / new define, repeat-once and carousel candidates. Every
        matched shape leaves with a record or a CONFIRM — kept, redefined or
        deleted, never silent (§4.3) — because the base drops a shape that
        goes TTL_FRAMES applied frames without one."""
        h, w = self._valid.shape
        n_reg = len(regions)
        masses = self._masses()
        live_lbl = np.zeros((h, w), np.int32)
        for s in sorted(masses, key=lambda s: -s.area):
            live_lbl[s.raster] = s.id
        pair = np.bincount(live_lbl.ravel() * (n_reg + 1) + (region_map.ravel() + 1),
                           minlength=128 * (n_reg + 1)).reshape(128, n_reg + 1)
        # Greedy IoU matching, best pairs first. A shape that moved locally by
        # up to the UPD range is matched on the IoU after shifting its raster
        # to the region's centroid (there is no ego-motion prediction yet).
        pairs = []
        region_masks: dict = {}
        for s in masses:
            row = pair[s.id]
            for r in regions:
                inter = int(row[r.index + 1])
                union = s.area + r.area - inter
                iou = inter / union if union > 0 else 0.0
                if iou >= MATCH_IOU:
                    pairs.append((iou, s.id, r.index))
                    continue
                ddx, ddy = int(round(r.centroid[0] - s.cx)), int(round(r.centroid[1] - s.cy))
                if max(abs(ddx), abs(ddy)) > 8 or max(abs(ddx), abs(ddy)) == 0 \
                        or float(vx.delta_e76(r.lab, s.lab)) > VERIFY_DE \
                        or (r.tree is None) != (s.layer == "mass"):
                    continue
                if r.index not in region_masks:
                    region_masks[r.index] = region_map == r.index
                shifted = _shift_mask(s.raster, ddx, ddy)
                inter = int((shifted & region_masks[r.index]).sum())
                union = int(shifted.sum()) + r.area - inter
                if union > 0 and inter / union >= MATCH_IOU:
                    pairs.append((inter / union * 0.99, s.id, r.index))   # behind a direct match
        pairs.sort(reverse=True)
        matched_shape: dict = {}
        matched_region: dict = {}
        self._evict_next &= set(self._shapes)                # an evictee already gone owes nothing
        for iou, id_, ri in pairs:
            if id_ in matched_shape or ri in matched_region or id_ in self._evict_next:
                continue                                     # an evictee leaves; its region is a newcomer
            matched_shape[id_] = (ri, iou)
            matched_region[ri] = id_
        taken: set = set()
        deleted: set = set()

        def add_del(s: _Shape) -> None:
            rec = vs.Del(s.id)
            cands.append(_Cand(5, (0, -s.area), rec, vs.record_bits(rec),
                               lambda id_=s.id: self._release_id(id_), (s.id,)))
            taken.add(s.id)
            deleted.add(s.id)

        # Pass 1: decide per shape and region. An unmatched live shape no longer
        # describes the capture (DEL); a matched one is kept (UPD / UCOL /
        # CONFIRM / repeat / carousel), redefined, or DEL'd when its layer changed.
        for s in masses:
            if s.id not in matched_shape:
                add_del(s)
        redefines: list = []                                 # (shape, region, ΔE): same layer, geometry failed
        fresh: list = []                                     # regions no live shape describes
        for r in regions:
            rec_geo = self._region_record(1, r)              # geometry probe; the id is assigned below
            if r.index in matched_region:
                s = self._shapes[matched_region[r.index]]
                same_layer = rec_geo is not None and _layer_of(rec_geo) == s.layer
                geometry_ok, upd = self._verify_geometry(s, r, rec_geo, matched_shape[s.id][1],
                                                         region_map, same_layer)
                de = float(vx.delta_e76(r.lab, s.lab))
                if geometry_ok and same_layer:
                    self._keep_candidates(s, r, upd, de, cands, verified)
                    continue
                if same_layer:
                    redefines.append((s, r, de))             # redefine under the same id, or keep (pass 2)
                    continue
                add_del(s)                                   # layer change: DEL + a fresh id
            if rec_geo is not None:
                fresh.append(r)
        # Pass 2: ΔD on the mirror with this frame's deletions already applied
        # (their pixels show L0), so no candidate is credited for painting
        # over a shape another record of this frame removes.
        mirror = l0.copy()
        gain = self._gain_lin(self._gain_next)
        under: dict = {}                                     # id -> the mirror under the shape, on its raster
        top = np.zeros((h, w), np.int32)                     # which shape the mirror shows per pixel
        for s in sorted(masses, key=lambda s: -s.area):
            if s.id not in deleted:
                under[s.id] = mirror[s.raster].copy()
                mirror[s.raster] = np.clip(s.shown * gain, 0.0, 255.0)
                top[s.raster] = s.id
        f = work.astype(np.float32)
        err_before = ((f - mirror) ** 2).sum(axis=2)
        for s, r, de in redefines:
            if not self._add_redefine(cands, s, r, level, err_before, f, l0, masses, deleted):
                # The live outline describes the region at least as well as
                # a new one would, so neither a redefine nor a DEL is
                # warranted: the shape is kept and confirmed like a verified one.
                self._keep_candidates(s, r, None, de, cands, verified)
        # Fresh defines are scored first and get ids in score order: a layer
        # whose ids run out keeps its best candidates and the rest wait for a
        # DEL to free an id (never an epoch trigger, see the module doc).
        scored = []
        for r in fresh:
            sc = self._score_define(r, err_before, f)
            if sc is not None:
                scored.append((sc, r))
        scored.sort(key=lambda t: -t[0][0])
        new_defines = 0
        refused: dict = {}                                   # layer -> best ΔD refused an id this frame
        self._waiting = 0
        for (per_bit, dd, bits, raster, area), r in scored:
            if new_defines >= MAX_NEW_DEFINES:
                break
            layer = "plant" if r.tree is not None else "mass"
            id_ = self._alloc_id(layer, taken)
            if id_ is None:
                refused[layer] = max(refused.get(layer, 0.0), dd)
                continue
            taken.add(id_)
            old = self._evicted.pop(id_, None)
            if old is not None:
                inter, union = int((old & raster).sum()), int((old | raster).sum())
                if union and inter / union >= MATCH_IOU:
                    # The evictee's own region took its id straight back: the
                    # newcomer that beat it is gone again. Hold this layer's
                    # evictions for a TTL so an on/off newcomer cannot rotate
                    # a wasted DEL + define through every holder.
                    self._evict_hold[layer] = self._frame_no + vs.TTL_FRAMES
            rec = self._region_record(id_, r)
            recs, ever_upd, ever_ucol = self._fresh_define_records(rec)
            cands.append(_Cand(5, (3, -per_bit), recs if len(recs) > 1 else rec,
                               sum(vs.record_bits(x) for x in recs),
                               lambda rec=rec, raster=raster, area=area, r=r, eu=ever_upd, ec=ever_ucol:
                               self._apply_define(rec, raster, area, r, level, eu, ec),
                               (id_,)))
            new_defines += 1
        self._waiting = len(scored) - new_defines
        # Eviction under id pressure: the best refused newcomer of a layer
        # against the least valuable live holder of that layer, both measured
        # the same way — the error a shape removes over the region it
        # describes after this capture, against the mirror without it.
        # Beating the weakest EVICT_VALUE_RATIO-fold on EVICT_STRIKES
        # consecutive frames DELs it next frame; its id cools down and
        # _alloc_id hands it to the newcomer as the last resort. One eviction
        # runs at a time per layer, and the evictee's own region then waits
        # like any other.
        by_index = {r.index: r for r in regions}
        strikes: dict = {}
        for layer, best_dd in refused.items():
            if any(o.layer == layer for o in self._shapes.values() if o.id in self._evict_next):
                continue
            if self._evict_hold.get(layer, -1) >= self._frame_no:
                continue
            holders = [o for o in masses if o.layer == layer and o.id not in deleted]
            if not holders:
                continue
            value = self._holder_values(holders, [o for o in masses if o.id not in deleted], matched_shape,
                                        by_index, region_map, mirror, under, top, f, gain, self._valid)
            weak = min(value, key=value.get)
            if best_dd >= EVICT_VALUE_RATIO * max(value[weak], 0.0):
                n = self._evict_strikes.get(layer, 0) + 1
                strikes[layer] = n
                if n >= EVICT_STRIKES:
                    self._evict_next.add(weak)
                    self._evicted[weak] = self._shapes[weak].raster
        self._evict_strikes = strikes

    @staticmethod
    def _holder_values(holders: list, shapes: list, matched_shape: dict, by_index: dict, region_map: np.ndarray,
                       mirror: np.ndarray, under: dict, top: np.ndarray, f: np.ndarray, gain,
                       valid: np.ndarray) -> dict:
        """Per live holder, the error it removes from the picture it will be in
        after this frame: over the raster its define would paint for the
        region it now describes (what ``_score_define`` scores a newcomer
        on; its own raster when unmatched), less the pixels a smaller live
        shape shows on top of it, the region's colour against the mirror
        with the holder taken out (what lies under it where it is on top,
        the mirror itself elsewhere) — so holder and newcomer compare."""
        area_of = np.zeros(128, np.int64)
        for o in shapes:
            area_of[o.id] = o.area
        top_area = area_of[top]
        values: dict = {}
        for o in holders:
            m = matched_shape.get(o.id)
            r = by_index.get(m[0]) if m is not None else None
            if r is not None:
                rast = (r.raster & valid) if r.raster is not None else (region_map == r.index)
            else:
                rast = o.raster
            rast = rast & ~((top != 0) & (top != o.id) & (top_area < o.area))
            colour = np.clip(vx.fill_shown_rgb8(r.fill if r is not None else o.fill) * gain, 0.0, 255.0)
            without = mirror.copy()
            if o.id in under:
                mine = (top == o.id)
                without[o.raster & mine] = under[o.id][mine[o.raster]]
            values[o.id] = float(((f[rast] - without[rast]) ** 2).sum() - ((f[rast] - colour) ** 2).sum())
        return values

    def _edge_candidates(self, edges: list, level: int, cands: list, verified: set) -> None:
        """L3 (§2.7 item 5): each new polyline is scored by its mean chamfer
        distance to the live edges' raster (one distance transform per frame);
        within EDGE_MATCH_CELLS it keeps that id (CONFIRM / repeat / carousel),
        otherwise it is a new define. Unmatched live edges are DEL'd. Edge ids
        that run out are a disclosed omission, not an epoch trigger."""
        live = [s for s in self._shapes.values() if s.layer == "edge"]
        h, w = self._valid.shape
        matched: dict = {}
        if live and edges:
            canvas = np.ones((h, w), np.uint8)
            for s in live:
                canvas[s.raster] = 0
            dist, labels = cv2.distanceTransformWithLabels(canvas, cv2.DIST_L2, 3, labelType=cv2.DIST_LABEL_CCOMP)
            lbl_to_id: dict = {}
            for s in live:
                ls = labels[s.raster]
                if len(ls):
                    lbl_to_id.setdefault(int(np.bincount(ls).argmax()), s.id)
            claimed: set = set()
            scored = sorted(((float(dist[e.raster].mean()) if e.raster.any() else 99.0, i) for i, e in enumerate(edges)))
            for d, i in scored:
                if d > EDGE_MATCH_CELLS:
                    break
                ls = labels[edges[i].raster]
                id_ = lbl_to_id.get(int(np.bincount(ls).argmax())) if len(ls) else None
                if id_ is not None and id_ not in claimed:
                    matched[i] = id_
                    claimed.add(id_)
        for s in live:
            if s.id not in matched.values():
                rec = vs.Del(s.id)
                cands.append(_Cand(5, (0, -s.area), rec, vs.record_bits(rec),
                                   lambda id_=s.id: self._release_id(id_), (s.id,)))
        new_edges = 0
        taken: set = set(matched.values())
        for i, e in enumerate(edges):
            if i in matched:
                s = self._shapes[matched[i]]
                verified.add(s.id)
                if s.repeat_left > 0:
                    cands.append(_Cand(6, (-e.score,), s.define, vs.record_bits(s.define),
                                       lambda s=s: self._apply_repeat(s, spend=True), (s.id,)))
                else:
                    cands.append(_Cand(7, (s.define_frame, s.id), s.define, vs.record_bits(s.define),
                                       lambda s=s: self._apply_repeat(s), (s.id,)))
                continue
            if new_edges >= MAX_NEW_EDGES:
                continue
            id_ = self._alloc_id("edge", taken)
            if id_ is None:
                break
            rec = vs.Edge(id_, e.cls, e.points)
            try:
                bits = vs.record_bits(rec)
            except ValueError:                               # a delta the EG2 field cannot carry
                continue
            taken.add(id_)
            new_edges += 1
            cands.append(_Cand(5, (4, -e.score), rec, bits,
                               lambda rec=rec, e=e: self._apply_edge(rec, e, level), (id_,)))

    def _apply_edge(self, rec, e, level: int) -> None:
        ys, xs = np.nonzero(e.raster)
        self._shapes[rec.id] = _Shape(rec.id, rec, vs.define_hash(rec), None, "edge", e.raster, int(e.raster.sum()),
                                      float(xs.mean()) if len(xs) else 0.0, float(ys.mean()) if len(ys) else 0.0,
                                      None, self._frame_no, repeat_left=LEVEL_REPEATS[level])

    def _verify_geometry(self, s: _Shape, r, rec_geo, iou: float, region_map: np.ndarray,
                         same_layer: bool) -> tuple:
        """(geometry_ok, upd): the live shape still describes the region when
        the capture yields the identical define, or the raster IoU is ≥ 0.7,
        possibly after a local shift within the UPD field (then ``upd`` is
        (dx, dy, shifted raster)). The identical define at a non-zero offset
        means the region is back at the define's own position: the offset
        returns to 0 by an UPD, never by re-sending the define, which the
        base reads as a repeat that keeps the offset (§3.4 rule 5)."""
        if same_layer and _geometry_key(rec_geo) == _geometry_key(s.define):
            if s.dx == 0 and s.dy == 0:
                return True, None
            return True, (0, 0, vx.raster_of(s.define, 0, 0, self._valid.shape) & self._valid)
        ddx, ddy = r.centroid[0] - s.cx, r.centroid[1] - s.cy
        if abs(ddx) >= UPD_MIN_SHIFT_WORK or abs(ddy) >= UPD_MIN_SHIFT_WORK:
            ndx, ndy = s.dx + int(round(ddx)), s.dy + int(round(ddy))
            if -8 <= ndx <= 7 and -8 <= ndy <= 7:
                shifted = _shift_mask(s.raster, ndx - s.dx, ndy - s.dy)
                inter = int((shifted & (region_map == r.index)).sum())
                union = int(shifted.sum()) + r.area - inter
                if union > 0 and inter / union >= VERIFY_IOU:
                    return True, (ndx, ndy, shifted)
            return False, None
        return iou >= VERIFY_IOU, None

    def _region_record(self, id_: int, region) -> object:
        if region.tree is not None:
            cx, cy, rx, ry = region.tree
            return vs.Tree(id_, cx, cy, rx, ry, region.fill)
        if region.poly is not None:
            return vs.Poly(id_, 0, region.poly, region.fill)
        return None

    def _define_raster(self, rec, region) -> np.ndarray:
        raster = region.raster if region.raster is not None else vx.raster_of(rec, 0, 0, self._valid.shape)
        return raster & self._valid

    def _score_define(self, region, err_before: np.ndarray, f: np.ndarray) -> Optional[tuple]:
        """(ΔD per bit, ΔD, bits, raster, area) of a fresh define, or None when
        painting it removes no more than MIN_DD_PER_PX per painted pixel.
        Scored with a placeholder id: neither the bits nor ΔD depend on the
        id, which is handed out afterwards in score order."""
        probe = self._region_record(vs.ID_PLANT[0] if region.tree is not None else vs.ID_MASS[0], region)
        if probe is None:
            return None
        try:
            bits = vs.record_bits(probe)
        except ValueError:                                   # a delta the EG2 field cannot carry
            return None
        raster = self._define_raster(probe, region)
        area = int(raster.sum())
        if area == 0:
            return None
        c = np.clip(vx.fill_shown_rgb8(probe.fill) * self._gain_lin(self._gain_next), 0.0, 255.0)
        dd = float(err_before[raster].sum() - ((f[raster] - c) ** 2).sum())
        if dd <= MIN_DD_PER_PX * area:
            return None
        return dd / bits, dd, bits, raster, area

    def _add_redefine(self, cands: list, s: _Shape, region, level: int, err_before: np.ndarray,
                      f: np.ndarray, l0: np.ndarray, masses: list, deleted: set) -> bool:
        """A same-id redefine when the new outline beats the live one. ΔD is
        measured over the union of the two rasters with the live shape gone
        from the "after" mirror, so the shape competes with the alternative
        and not with itself over its own pixels (scoring it against a mirror
        that still shows it rejected every redefine of a jittering outline
        and left the shape silent). A record identical to the live define is
        not a redefine (§3.4 rule 5) and is refused like one not worth its
        bits; the caller then keeps and confirms the live shape."""
        rec = self._region_record(s.id, region)
        if rec is None or vs.define_hash(rec) == s.dhash:
            return False
        try:
            bits = vs.record_bits(rec)
        except ValueError:                                   # a delta the EG2 field cannot carry
            return False
        raster = self._define_raster(rec, region)
        area = int(raster.sum())
        if area == 0:
            return False
        gain = self._gain_lin(self._gain_next)
        uy, ux = np.nonzero(s.raster | raster)               # the union of the two outlines, as pixel lists
        fu = f[uy, ux]
        under = l0[uy, ux].copy()                            # the mirror without this shape, on the union
        for o in sorted(masses, key=lambda o: -o.area):
            if o.id not in deleted and o.id != s.id:
                m = o.raster[uy, ux]
                if m.any():
                    under[m] = np.clip(o.shown * gain, 0.0, 255.0)
        c = np.clip(vx.fill_shown_rgb8(rec.fill) * gain, 0.0, 255.0)
        after = ((fu - under) ** 2).sum(axis=1)
        new = raster[uy, ux]
        after[new] = ((fu[new] - c) ** 2).sum(axis=1)
        dd = float(err_before[uy, ux].sum() - after.sum())
        if dd <= MIN_DD_PER_PX * area:
            return False
        cands.append(_Cand(5, (3, -dd / bits), rec, bits,
                           lambda: self._apply_define(rec, raster, area, region, level), (s.id,)))
        return True

    def _apply_define(self, rec, raster: np.ndarray, area: int, region, level: int,
                      ever_upd: bool = False, ever_ucol: bool = False) -> None:
        dh = vs.define_hash(rec)
        s = self._shapes.get(rec.id)
        if s is not None and s.dhash == dh:
            self._apply_repeat(s)                            # §3.4 rule 5: a same-hash define is a repeat
            return
        ys, xs = np.nonzero(raster)
        self._shapes[rec.id] = _Shape(rec.id, rec, dh, rec.fill, _layer_of(rec), raster, area,
                                      float(xs.mean()), float(ys.mean()), region.lab, self._frame_no,
                                      repeat_left=LEVEL_REPEATS[level], ever_upd=ever_upd, ever_ucol=ever_ucol)

    def _keep_candidates(self, s: _Shape, region, upd: Optional[tuple], de: float, cands: list,
                         verified: set) -> None:
        """Slots 5–7 for a live shape that still describes its region: UPD when
        it moved; UCOL when its colour moved by more than ΔE 6 *and* the FILL
        it would send differs (below the FILL's resolution the base already
        shows what this capture would send); CONFIRM when neither applies;
        and the repeat-once or carousel re-send, which carries the current
        UPD and UCOL (§4.3) unless this frame offers that record itself. A
        shape that owes its re-statement after a range-1/2 epoch start gets
        that instead (``_restate_candidate``)."""
        r = region
        if s.id in self._restate_owed or self._restate_all:
            self._restate_candidate(s, r, upd, de, cands)
            return
        if upd is not None:
            rec = vs.Upd(s.id, upd[0], upd[1])
            cands.append(_Cand(5, (1, -r.area), rec, vs.record_bits(rec),
                               lambda s=s, u=upd, r=r: self._apply_upd(s, u, r), (s.id,)))
        recolour = de > VERIFY_DE and r.fill != s.fill
        if recolour:
            rec = vs.Ucol(s.id, r.fill)
            cands.append(_Cand(5, (2, -r.area), rec, vs.record_bits(rec),
                               lambda s=s, r=r: self._apply_ucol(s, r), (s.id,)))
        if upd is None and not recolour:
            verified.add(s.id)
        if s.repeat_left > 0:                                # repeat-once, re-verified on this capture
            recs = self._resend_records(s, with_upd=upd is None, with_ucol=not recolour)
            cands.append(_Cand(6, (-r.area,), recs, sum(vs.record_bits(x) for x in recs),
                               lambda s=s: self._apply_repeat(s, spend=True), (s.id,)))
        elif upd is None and not recolour:
            recs = self._resend_records(s, with_upd=True, with_ucol=True)
            cands.append(_Cand(7, (s.define_frame, s.id), recs, sum(vs.record_bits(x) for x in recs),
                               lambda s=s: self._apply_repeat(s), (s.id,)))

    def _restate_candidate(self, s: _Shape, r, upd: Optional[tuple], de: float, cands: list) -> None:
        """The re-statement a kept shape owes after a range-1/2 epoch start
        (§3.5, §4.3): ONE slot-5 candidate, ahead of the frame's other
        changes (order 0.5, behind the DELs), carrying the define with the
        UPD and UCOL the shape has after this capture, applied as a repeat
        plus this frame's change. It replaces the bare UPD / UCOL / CONFIRM
        the shape would otherwise get: a base that dropped the shape would
        orphan the UPD, and a CONFIRM would contradict its stale state; and
        a repeat left to slot 6 is starved by those UPDs at a small budget.
        The shape's repeat-once is not spent by it, so a lost re-statement
        frame is covered by the slot-6 copy that follows (§4.3)."""
        recolour = de > VERIFY_DE and r.fill != s.fill
        recs = [s.define]
        if upd is not None:
            recs.append(vs.Upd(s.id, upd[0], upd[1]))
        elif (s.dx, s.dy) != (0, 0) or s.ever_upd:
            recs.append(vs.Upd(s.id, s.dx, s.dy))
        if recolour:
            recs.append(vs.Ucol(s.id, r.fill))
        elif s.fill is not None and (s.fill != s.define.fill or s.ever_ucol):
            recs.append(vs.Ucol(s.id, s.fill))

        def apply(s=s, u=upd, r=r, recolour=recolour):
            self._apply_repeat(s)                            # not spent: the repeat-once follows as a second copy
            if u is not None:
                self._apply_upd(s, u, r)
            if recolour:
                self._apply_ucol(s, r)
        cands.append(_Cand(5, (0.5, -r.area), tuple(recs), sum(vs.record_bits(x) for x in recs), apply, (s.id,)))

    @staticmethod
    def _resend_records(s: _Shape, with_upd: bool, with_ucol: bool) -> tuple:
        """The define as sent, plus the UPD and UCOL that bring a base which
        lost either, or dropped the shape, to the mirror's state (§4.3).
        Once a shape has had an UPD or a UCOL they are stated on every
        re-send, at the define's own value too: the base's copy is not the
        define's value just because the mirror's is (a lost UPD back to 0,
        a lost UCOL back to the define's fill)."""
        recs = [s.define]
        if with_upd and ((s.dx, s.dy) != (0, 0) or s.ever_upd):
            recs.append(vs.Upd(s.id, s.dx, s.dy))
        if with_ucol and s.fill is not None and (s.fill != s.define.fill or s.ever_ucol):
            recs.append(vs.Ucol(s.id, s.fill))
        return tuple(recs)

    def _tick_ttl(self) -> None:
        """The base's shape TTL, mirrored: a shape this frame did not verify
        ages by one applied frame and is dropped at TTL_FRAMES (§4.3)."""
        for id_ in list(self._shapes):
            s = self._shapes[id_]
            if id_ in self._verified_now:
                s.frames_since_verify = 0
                continue
            s.frames_since_verify += 1
            if s.frames_since_verify >= vs.TTL_FRAMES:
                self._release_id(id_)
                self._ttl_dropped += 1

    def _apply_repeat(self, s: _Shape, spend: bool = False) -> None:
        """A define went out again. ``spend`` charges one repeat-once (§4.3)
        only now, when the packer really selected it: a repeat that missed
        the budget behind higher slots and the CONFIRM/DIGEST reserve is
        still owed and is offered again next frame."""
        s.define_frame = self._frame_no
        if spend and s.repeat_left > 0:
            s.repeat_left -= 1

    def _spend_anchor_repeat(self) -> None:
        if self._anchor_repeat_left > 0:
            self._anchor_repeat_left -= 1

    def _apply_upd(self, s: _Shape, upd: tuple, region) -> None:
        ndx, ndy, shifted = upd
        s.cx += ndx - s.dx
        s.cy += ndy - s.dy
        s.dx, s.dy = ndx, ndy
        s.raster = shifted
        s.area = int(shifted.sum())
        s.ever_upd = True

    def _apply_ucol(self, s: _Shape, region) -> None:
        s.fill = region.fill
        s.lab = region.lab
        s.ever_ucol = True

    def _confirm_records(self, ids: list) -> list:
        """CONFIRM runs of up to 16 consecutive ids (§3.3); each tag is the low
        2 bits of the shape's state-hash in the mirror (§4.3)."""
        recs = []
        i = 0
        while i < len(ids):
            base = ids[i]
            run = [id_ for id_ in ids[i:] if id_ < base + 16]
            tags: list = [None] * (run[-1] - base + 1)
            for id_ in run:
                s = self._shapes.get(id_)
                tags[id_ - base] = (s.state_hash() & 3) if s is not None else 0
            recs.append(vs.Confirm(base, tuple(tags)))
            i += len(run)
        return recs

    def _confirm_bits(self, ids: list) -> int:
        return sum(vs.record_bits(rec) for rec in self._confirm_records(ids))

    @staticmethod
    def _pack(cands: list, total_bits: int, reserve: int, carousel_bits: int) -> tuple:
        """Greedy fill in §4.2 order; never splits a candidate (a record, or
        the indivisible anchor + LAYER_CLEAR pair of an epoch start), and
        never packs one whose ``needs`` (the epoch start, on a key attempt)
        was not packed. The CONFIRM + DIGEST reserve binds slots ≥ 5 only, so
        the anchor and STATUS always go first; the carousel (slot 7) has its
        own cap."""
        cands.sort(key=lambda c: (c.slot, c.order))
        used = 0
        car_used = 0
        packed = []
        packed_ids: set = set()
        for c in cands:
            limit = total_bits if c.slot < 5 else total_bits - reserve
            if used + c.bits > limit or (c.needs is not None and id(c.needs) not in packed_ids):
                continue
            if c.slot == 7:
                if car_used + c.bits > carousel_bits:
                    continue
                car_used += c.bits
            packed.append(c)
            packed_ids.add(id(c))
            used += c.bits
        return packed, used
