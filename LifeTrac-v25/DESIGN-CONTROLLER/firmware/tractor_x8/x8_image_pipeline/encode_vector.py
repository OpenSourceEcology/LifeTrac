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

Epochs (§3.5, §4.3). A new epoch starts on the first frame, on
``force_epoch()`` / ``epoch_start=True``, when the horizon state flips
between found and NO_HORIZON, on id exhaustion, when more than 40 % of the
valid area is relabelled between captures, and every 60 s (safety refresh,
LAYER_CLEAR range 2, which keeps the masses). The epoch-start records are
repeated in the following frame(s) per the level (§4.5.4), and a static
scene converges to RESID + STATUS + CONFIRM + DIGEST with a carousel frame
every 1/κ frames.

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


class _IdExhausted(Exception):
    """No free id in the layer: the frame is rebuilt as an epoch start (§4.3)."""


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

    @property
    def shown(self) -> np.ndarray:
        return vx.fill_shown_rgb8(self.fill)

    def state_hash(self) -> int:
        # An EDGE has no colour: its base colour field hashes as 0.
        base = vx.fill_base_rgb444(self.fill) if self.fill is not None else 0
        return vs.state_hash(self.dx, self.dy, base, [], [])


@dataclass
class _Cand:
    slot: int                       # §4.2 priority slot
    order: tuple                    # ascending within the slot
    record: object
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
        self._epoch_started_frame = -1
        self._carousel_acc = 0.0
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
        try:
            body, records, stats = self._build(work, hz, regions, region_map, edges, clear, level, moving,
                                               budget_bytes, age_units, now)
        except _IdExhausted:
            body, records, stats = self._build(work, hz, regions, region_map, edges, 0, level, moving,
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

    def _anchor_record(self, hz: vx.Horizon):
        """A fresh absolute anchor (ABS or NO_HORIZON) from this capture; it
        re-anchors the mirror's L0 state."""
        top_vf = vx.make_vfill(hz.top, int(round(hz.top_dl / 8.0)))
        bot_vf = vx.make_vfill(hz.bottom, int(round(hz.bottom_dl / 8.0)))
        self._top_vf, self._bot_vf = top_vf, bot_vf
        self._top_lab, self._bot_lab = vx.rgb_to_lab(hz.top), vx.rgb_to_lab(hz.bottom)
        self._anchor_found = hz.found
        if hz.found:
            codes = self._abs_codes(hz)
            self._anchor_codes = codes
            self._shown_hz = vx.horizon_from_codes(*codes)
            return vs.HznAbs(codes[0], codes[1], codes[2], top_vf, bot_vf)
        self._anchor_codes = None
        self._shown_hz = vx.Horizon(False)
        return vs.HznNoHorizon(top_vf, bot_vf)

    def _horizon_records(self, hz: vx.Horizon, key: bool, level: int) -> list:
        """Slot 1 on an epoch start or its repeat (anchor + LAYER_CLEAR), else
        slot 3: RESID when the change fits its fields, ABS on a larger move
        or a colour stop change of ΔE > 6, nothing in a NO_HORIZON epoch. At
        V3 every frame is a beacon carrying the absolute anchor (§4.5.4)."""
        if key or self._anchor_repeat_left > 0 or self._anchor_found is None:
            # The repeat is spent by the anchor candidate's apply (when it is
            # packed), never here: a repeat that missed the budget is still owed.
            return [self._anchor_record(hz), vs.LayerClear(self._clear_range)]
        if level >= 3:
            return [self._anchor_record(hz)]
        colour_jump = (vx.delta_e76(vx.rgb_to_lab(hz.top), self._top_lab) > VERIFY_DE
                       or vx.delta_e76(vx.rgb_to_lab(hz.bottom), self._bot_lab) > VERIFY_DE)
        if not hz.found:
            return [self._anchor_record(hz)] if colour_jump else []
        y8, ang, curv = self._abs_codes(hz)
        ay, aa, ac = self._anchor_codes
        dy, dang = y8 - ay, ang - aa
        if colour_jump or curv != ac or not (-16 <= dy <= 15) or not (-8 <= dang <= 7):
            return [self._anchor_record(hz)]
        self._shown_hz = vx.horizon_from_codes(ay + dy, aa + dang, ac)
        return [vs.HznResid(dy, dang)]

    # ------------------------------------------------------------ epochs

    def _epoch_trigger(self, epoch_start: bool, hz: vx.Horizon, region_map: np.ndarray, regions: list,
                       now: float) -> Optional[int]:
        """LAYER_CLEAR range of the epoch this frame starts, or None (§4.3)."""
        if self._epoch < 0 or epoch_start:
            return 0
        if self._pending_epoch is not None:
            return self._pending_epoch
        if self._anchor_found is not None and hz.found != self._anchor_found:
            return 0
        if now - self._epoch_t0 >= SAFETY_REFRESH_S:
            return 2
        prev = self._prev_region_map
        if prev is not None and self._relabelled_fraction(prev, self._prev_region_lab, region_map,
                                                          regions) > RELABEL_EPOCH_FRACTION:
            return 0
        return None

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

    def _start_epoch(self, clear_range: int, level: int, now: float) -> None:
        if self._epoch_started_frame != self._frame_no:   # an id-exhaustion rebuild keeps the number
            self._epoch = (self._epoch + 1) & 15
            self._epoch_started_frame = self._frame_no
        self._epoch_t0 = now
        self._clear_range = clear_range
        self._pending_epoch = None
        self._anchor_found = None
        self._anchor_repeat_left = LEVEL_REPEATS[level]
        if clear_range == 0:                          # every mass is redefined: a fresh reference exposure
            self._gain = self._gain_next = GAIN_NEUTRAL
        dropped = {0: ("mass", "plant", "edge"), 1: ("plant", "edge"), 2: ("edge",), 3: ()}[clear_range]
        for id_ in [i for i, s in self._shapes.items() if s.layer in dropped]:
            del self._shapes[id_]

    def _alloc_id(self, layer: str, taken: set) -> int:
        rng = {"plant": vs.ID_PLANT, "edge": vs.ID_EDGE}.get(layer, vs.ID_MASS)
        for id_ in rng:
            if id_ not in self._shapes and id_ not in taken:
                return id_
        raise _IdExhausted(layer)

    # ------------------------------------------------------------ T + P

    def _masses(self) -> list:
        return [s for s in self._shapes.values() if s.layer != "edge"]

    def _build(self, work: np.ndarray, hz: vx.Horizon, regions: list, region_map: np.ndarray, edges: list,
               clear: Optional[int], level: int, moving: bool, budget_bytes: int, age_units: int,
               now: float) -> tuple:
        key = clear is not None
        if key:
            self._start_epoch(clear, level, now)
            if clear == 0:
                self._normalise(regions, GAIN_NEUTRAL)
        f_bytes = max(0, int(budget_bytes) - HEADER_LEN)
        if LEVEL_BODY[level] is not None:
            f_bytes = min(f_bytes, LEVEL_BODY[level])
        if key:
            f_bytes -= 1                                     # §3.1: a copied epoch start stays one fragment
        f_bytes = max(f_bytes, 0)
        total_bits = max(f_bytes * 8 - vs.HEADER_BITS, 0)
        h, w = self._valid.shape
        cands: list = []
        hzn = self._horizon_records(hz, key, level)
        for i, rec in enumerate(hzn):                        # anchor + LAYER_CLEAR: slot 1; RESID/ABS: slot 3
            apply = self._spend_anchor_repeat if (len(hzn) == 2 and not key and i == 0) else (lambda: None)
            # LAYER_CLEAR rides with its anchor: if the anchor does not fit, neither goes.
            needs = cands[-1] if (len(hzn) == 2 and i == 1) else None
            cands.append(_Cand(1 if len(hzn) == 2 else 3, (i,), rec, vs.record_bits(rec), apply, needs=needs))
        if self._gain_next != self._gain or (key and self._gain_next != GAIN_NEUTRAL):
            gain = vs.Gain(*self._gain_next)             # absolute since the epoch start (§3.3)
            cands.append(_Cand(3, (9,), gain, vs.record_bits(gain), lambda: setattr(self, "_gain", self._gain_next)))
        status = vs.Status(moving=bool(moving), **STATUS_NONE)
        cands.append(_Cand(4, (0,), status, vs.record_bits(status), lambda: None))
        l0 = vx.render_l0(self._shown_hz, self._top_vf, self._bot_vf, (h, w))
        verified: set = set()
        if level < 3:                                        # V3 is the anchor + STATUS beacon only
            self._temporal_candidates(work, l0, regions, region_map, level, key, cands, verified)
            self._edge_candidates(edges, level, cands, verified)
        carousel_bits = int(LEVEL_KAPPA[level] * total_bits) if self._carousel_due(level) else 0
        reserve = self._confirm_bits(sorted(verified)) + vs.record_bits(vs.Digest(0, 0))
        packed, used = self._pack(cands, total_bits, reserve, carousel_bits)
        for c in packed:
            c.apply()
        mentioned = {id_ for c in packed for id_ in c.mentions}
        records = [c.record for c in packed]
        if level < 3:
            unmentioned = [i for i in sorted(verified) if i not in mentioned and i in self._shapes]
            for rec in self._confirm_records(unmentioned):
                if used + vs.record_bits(rec) <= total_bits:
                    records.append(rec)
                    used += vs.record_bits(rec)
            digest = vs.Digest(len(self._shapes), vs.digest_crc(
                [(s.id, s.dhash, s.state_hash()) for s in self._shapes.values()],
                [(0, 0)] * 4, 128, self._gain))
            if used + vs.record_bits(digest) <= total_bits:
                records.append(digest)
                used += vs.record_bits(digest)
        header = vs.Header(key=key, age=max(0, min(15, int(age_units))), epoch=self._epoch, level=level)
        body = vs.encode_frame(header, records, f_bytes) if f_bytes >= 2 else b""
        mirror = l0.copy()
        gain = self._gain_lin(self._gain)
        for s in sorted(self._masses(), key=lambda s: -s.area):
            mirror[s.raster] = np.clip(s.shown * gain, 0.0, 255.0)
        err = ((work.astype(np.float32) - mirror) ** 2).sum(axis=2)
        residual = float(err[self._valid].mean()) if self._valid.any() else 0.0
        bits = {"hdr": vs.HEADER_BITS, "L0": 0, "L1": 0, "L2": 0, "L3": 0, "L4": 0, "ctrl": 0}
        counts: dict = {}
        for rec in records:
            bits[_bits_layer(rec)] += vs.record_bits(rec)
            counts[type(rec).__name__] = counts.get(type(rec).__name__, 0) + 1
        bits["pad"] = len(body) * 8 - sum(bits.values()) if body else 0
        packed_ids = {id(c) for c in packed}
        stats = {"epoch": self._epoch, "epoch_start": key, "f_bytes": f_bytes, "body_bytes": len(body),
                 "record_bits": used + vs.HEADER_BITS, "bits": bits, "records": counts,
                 "n_live": len(self._shapes), "n_candidates": len(cands), "n_packed": len(packed),
                 "residual": residual, "carousel_bits": carousel_bits,
                 "candidates": [(c.slot, type(c.record).__name__, c.bits, c.order, id(c) in packed_ids)
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
        redefine / new define, repeat-once and carousel candidates."""
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
        for iou, id_, ri in pairs:
            if id_ in matched_shape or ri in matched_region:
                continue
            matched_shape[id_] = (ri, iou)
            matched_region[ri] = id_
        taken: set = set()
        deleted: set = set()

        def add_del(s: _Shape) -> None:
            rec = vs.Del(s.id)
            cands.append(_Cand(5, (0, -s.area), rec, vs.record_bits(rec),
                               lambda id_=s.id: self._shapes.pop(id_, None), (s.id,)))
            taken.add(s.id)
            deleted.add(s.id)

        # Pass 1: decide per shape and region. An unmatched live shape no longer
        # describes the capture (DEL); a matched one is kept (UPD / UCOL /
        # CONFIRM / repeat / carousel), redefined, or DEL'd when its layer changed.
        for s in masses:
            if s.id not in matched_shape:
                add_del(s)
        defines: list = []                                   # (region, id or None for a fresh one)
        for r in regions:
            rec_geo = self._region_record(1, r)              # geometry probe; the id is assigned below
            if r.index in matched_region:
                s = self._shapes[matched_region[r.index]]
                same_layer = rec_geo is not None and _layer_of(rec_geo) == s.layer
                geometry_ok, upd = self._verify_geometry(s, r, rec_geo, matched_shape[s.id][1],
                                                         region_map, same_layer)
                de = float(vx.delta_e76(r.lab, s.lab))
                if geometry_ok and same_layer:
                    if upd is not None:
                        rec = vs.Upd(s.id, upd[0], upd[1])
                        cands.append(_Cand(5, (1, -r.area), rec, vs.record_bits(rec),
                                           lambda s=s, u=upd, r=r: self._apply_upd(s, u, r), (s.id,)))
                    if de > VERIFY_DE and r.fill != s.fill:
                        rec = vs.Ucol(s.id, r.fill)
                        cands.append(_Cand(5, (2, -r.area), rec, vs.record_bits(rec),
                                           lambda s=s, r=r: self._apply_ucol(s, r), (s.id,)))
                    if upd is None and de <= VERIFY_DE:
                        verified.add(s.id)
                    if s.repeat_left > 0:                 # repeat-once, re-verified on this capture
                        cands.append(_Cand(6, (-r.area,), s.define, vs.record_bits(s.define),
                                           lambda s=s: self._apply_repeat(s, spend=True), (s.id,)))
                    elif de <= VERIFY_DE and upd is None:
                        cands.append(_Cand(7, (s.define_frame, s.id), s.define, vs.record_bits(s.define),
                                           lambda s=s: self._apply_repeat(s), (s.id,)))
                    continue
                if same_layer:
                    defines.append((r, s.id))                # redefine under the same id
                    continue
                add_del(s)                                   # layer change: DEL + a fresh id
            if rec_geo is not None:
                defines.append((r, None))
        # Pass 2: ΔD on the mirror with this frame's deletions already applied
        # (their pixels show L0), so no candidate is credited for painting
        # over a shape another record of this frame removes.
        mirror = l0.copy()
        gain = self._gain_lin(self._gain_next)
        for s in sorted(masses, key=lambda s: -s.area):
            if s.id not in deleted:
                mirror[s.raster] = np.clip(s.shown * gain, 0.0, 255.0)
        f = work.astype(np.float32)
        err_before = ((f - mirror) ** 2).sum(axis=2)
        new_defines = 0
        for r, id_ in defines:
            if id_ is None:
                if new_defines >= MAX_NEW_DEFINES:
                    continue
                try:
                    id_ = self._alloc_id("plant" if r.tree is not None else "mass", taken)
                except _IdExhausted:
                    if key:                                  # already an epoch start: the rest waits
                        break
                    raise
                taken.add(id_)
                if self._add_define(cands, id_, r, level, err_before, f):
                    new_defines += 1
            else:
                self._add_define(cands, id_, r, level, err_before, f)

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
                                   lambda id_=s.id: self._shapes.pop(id_, None), (s.id,)))
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
            try:
                id_ = self._alloc_id("edge", taken)
            except _IdExhausted:
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

    @staticmethod
    def _verify_geometry(s: _Shape, r, rec_geo, iou: float, region_map: np.ndarray,
                         same_layer: bool) -> tuple:
        """(geometry_ok, upd): the live shape still describes the region when
        the capture yields the identical define, or the raster IoU is ≥ 0.7,
        possibly after a local shift within the UPD field (then ``upd`` is
        (dx, dy, shifted raster))."""
        if same_layer and s.dx == 0 and s.dy == 0 and _geometry_key(rec_geo) == _geometry_key(s.define):
            return True, None
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

    def _add_define(self, cands: list, id_: int, region, level: int, err_before: np.ndarray,
                    f: np.ndarray) -> bool:
        """A define candidate when painting it reduces the mirror error by more
        than MIN_DD_PER_PX per painted pixel; ordered by ΔD per bit."""
        rec = self._region_record(id_, region)
        if rec is None:
            return False
        try:
            bits = vs.record_bits(rec)
        except ValueError:                                   # a delta the EG2 field cannot carry
            return False
        raster = (region.raster if region.raster is not None else vx.raster_of(rec, 0, 0, self._valid.shape)) \
            & self._valid
        area = int(raster.sum())
        if area == 0:
            return False
        c = np.clip(vx.fill_shown_rgb8(rec.fill) * self._gain_lin(self._gain_next), 0.0, 255.0)
        dd = float(err_before[raster].sum() - ((f[raster] - c) ** 2).sum())
        if dd <= MIN_DD_PER_PX * area:
            return False
        cands.append(_Cand(5, (3, -dd / bits), rec, bits,
                           lambda: self._apply_define(rec, raster, area, region, level), (id_,)))
        return True

    def _apply_define(self, rec, raster: np.ndarray, area: int, region, level: int) -> None:
        ys, xs = np.nonzero(raster)
        self._shapes[rec.id] = _Shape(rec.id, rec, vs.define_hash(rec), rec.fill, _layer_of(rec), raster, area,
                                      float(xs.mean()), float(ys.mean()), region.lab, self._frame_no,
                                      repeat_left=LEVEL_REPEATS[level])

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

    def _apply_ucol(self, s: _Shape, region) -> None:
        s.fill = region.fill
        s.lab = region.lab

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
        """Greedy fill in §4.2 order; never splits a record, and never packs a
        candidate whose ``needs`` (LAYER_CLEAR's anchor) was not packed. The
        CONFIRM + DIGEST reserve binds slots ≥ 5 only, so the anchor and
        STATUS always go first; the carousel (slot 7) has its own cap."""
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
