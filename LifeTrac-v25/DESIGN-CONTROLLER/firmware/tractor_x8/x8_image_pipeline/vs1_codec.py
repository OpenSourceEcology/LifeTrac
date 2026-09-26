# Mirror of base_station/image_pipeline/vector_scene/codec.py for the tractor image, which cannot import the base tree.
# tests/test_vs1_codec_parity_sil.py pins the two byte-identical below this header: re-copy, never edit here.
"""VS1 wire format: ``TileDeltaFrame`` codec 6 (``VECTOR_SCENE.md`` §3.2–3.4).

Pure stdlib, shared by the tractor encoder and the base store. Records are
frozen dataclasses holding *codes* (the integers that go on the wire, signed
fields as signed ints); the encoder and renderer convert to pixels and
degrees with the formulas in §3.3. ``encode_frame`` never emits a body over
the caller's byte budget, and ``decode_frame`` is all-or-nothing: any parser
rule of §3.4 that fails raises :class:`VsDecodeError` with a short ``reason``
the caller can count, and nothing is applied.

Bit order is MSB-first. Every signed fixed-width field is two's complement.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Union

VS_VERSION = 0
HEADER_BITS = 13
CODEC_VECTOR = 6            # TileDeltaFrame codec byte for a VS body

# Static palette slots 0–7 (RGB444), §3.3. Slots 8–15 are set by PAL.
STATIC_PALETTE = (0x6BE, 0xBBC, 0x252, 0x693, 0xDB6, 0x753, 0x223, 0xFFF)

# id ranges by layer (§2.2, §3.4). id 0 is illegal on every record.
ID_MASS = range(1, 32)
ID_PLANT = range(32, 56)
ID_EDGE = range(56, 80)
ID_CORRIDOR = range(80, 88)
ID_L4 = range(88, 128)
ID_ANY = range(1, 128)

SKYLINE_SAMPLES = (8, 12, 16, 24)
EG2_MAX_ZIGZAG = 123        # four leading zeros at most (§3.3)


class VsDecodeError(ValueError):
    """The frame violates a §3.4 parser rule; ``reason`` is a short counter key."""

    def __init__(self, reason: str, detail: str = ""):
        super().__init__(f"{reason}: {detail}" if detail else reason)
        self.reason = reason


class FrameTooLarge(ValueError):
    """The records do not fit the byte budget F."""


class _Eof(Exception):
    """Internal: a read ran past the end of the body."""


# ---------------------------------------------------------------- bit I/O

class BitWriter:
    """MSB-first bit writer."""

    def __init__(self) -> None:
        self._acc = 0
        self._n = 0

    @property
    def bit_count(self) -> int:
        return self._n

    def write(self, value: int, nbits: int) -> None:
        if nbits < 0 or value < 0 or value >= (1 << nbits):
            raise ValueError(f"value {value} does not fit {nbits} bits")
        self._acc = (self._acc << nbits) | value
        self._n += nbits

    def to_bytes(self) -> bytes:
        """Zero-pad to a byte boundary."""
        pad = (-self._n) % 8
        return ((self._acc << pad).to_bytes((self._n + pad) // 8, "big")
                if self._n else b"")


class BitReader:
    """MSB-first bit reader over a bytes object."""

    def __init__(self, data: bytes) -> None:
        self._data = data
        self._pos = 0
        self._total = len(data) * 8

    def remaining(self) -> int:
        return self._total - self._pos

    def read(self, nbits: int) -> int:
        if nbits == 0:
            return 0
        if self._pos + nbits > self._total:
            raise _Eof()
        value = 0
        for _ in range(nbits):
            byte = self._data[self._pos >> 3]
            value = (value << 1) | ((byte >> (7 - (self._pos & 7))) & 1)
            self._pos += 1
        return value

    def remaining_all_zero(self) -> bool:
        pos = self._pos
        while pos < self._total:
            if (self._data[pos >> 3] >> (7 - (pos & 7))) & 1:
                return False
            pos += 1
        return True


def _s(value: int, nbits: int) -> int:
    """Two's-complement code of a signed value."""
    lo, hi = -(1 << (nbits - 1)), (1 << (nbits - 1)) - 1
    if not lo <= value <= hi:
        raise ValueError(f"{value} outside signed {nbits}-bit range {lo}..{hi}")
    return value & ((1 << nbits) - 1)


def _u2s(code: int, nbits: int) -> int:
    return code - (1 << nbits) if code >= (1 << (nbits - 1)) else code


def _check(value: int, nbits: int, name: str) -> int:
    if not 0 <= value < (1 << nbits):
        raise ValueError(f"{name}={value} does not fit {nbits} bits")
    return value


# ---------------------------------------------------------------- EG2

def zigzag(d: int) -> int:
    return 2 * d if d >= 0 else -2 * d - 1


def unzigzag(v: int) -> int:
    return v // 2 if v % 2 == 0 else -(v + 1) // 2


def eg2_bits(d: int) -> int:
    """Length of the order-2 Exp-Golomb code of a zigzagged delta."""
    v = zigzag(d)
    if v > EG2_MAX_ZIGZAG:
        raise ValueError(f"delta {d} exceeds the EG2 field")
    return 2 * (v + 4).bit_length() - 3


def _write_eg2(w: BitWriter, d: int) -> None:
    v = zigzag(d)
    if v > EG2_MAX_ZIGZAG:
        raise ValueError(f"delta {d} exceeds the EG2 field")
    wv = v + 4
    length = wv.bit_length()
    w.write(0, length - 3)
    w.write(wv, length)


def _read_eg2(r: BitReader) -> int:
    zeros = 0
    while r.read(1) == 0:
        zeros += 1
        if zeros > 4:
            raise VsDecodeError("eg2_overflow")
    length = zeros + 3
    wv = (1 << (length - 1)) | r.read(length - 1)
    return unzigzag(wv - 4)


# ---------------------------------------------------------------- fills

@dataclass(frozen=True)
class Fill:
    """FILL = m1 | pal4 or rgb12 | g1 | [dir3 | dl3] (§3.3)."""
    palette: int | None = None      # slot 0..15 (m = 0)
    rgb444: int | None = None       # 12-bit colour (m = 1)
    grad: tuple[int, int] | None = None   # (dir3 0..7, dl3 -4..3)

    def __post_init__(self) -> None:
        if (self.palette is None) == (self.rgb444 is None):
            raise ValueError("Fill needs exactly one of palette / rgb444")
        if self.palette is not None:
            _check(self.palette, 4, "palette")
        else:
            _check(self.rgb444, 12, "rgb444")
        if self.grad is not None:
            _check(self.grad[0], 3, "dir")
            _s(self.grad[1], 3)

    @property
    def bits(self) -> int:
        return 1 + (4 if self.palette is not None else 12) + 1 + (6 if self.grad else 0)


def _write_fill(w: BitWriter, f: Fill) -> None:
    if f.palette is not None:
        w.write(0, 1)
        w.write(f.palette, 4)
    else:
        w.write(1, 1)
        w.write(f.rgb444, 12)
    if f.grad is None:
        w.write(0, 1)
    else:
        w.write(1, 1)
        w.write(f.grad[0], 3)
        w.write(_s(f.grad[1], 3), 3)


def _read_fill(r: BitReader) -> Fill:
    if r.read(1) == 0:
        pal, rgb = r.read(4), None
    else:
        pal, rgb = None, r.read(12)
    grad = None
    if r.read(1):
        grad = (r.read(3), _u2s(r.read(3), 3))
    return Fill(palette=pal, rgb444=rgb, grad=grad)


@dataclass(frozen=True)
class VFill:
    """vfill (HZN only) = m1 | pal4 or rgb12 | dl4 (§3.3)."""
    palette: int | None = None
    rgb444: int | None = None
    dl: int = 0                     # -8..7, ΔL = 8·dl

    def __post_init__(self) -> None:
        if (self.palette is None) == (self.rgb444 is None):
            raise ValueError("VFill needs exactly one of palette / rgb444")
        if self.palette is not None:
            _check(self.palette, 4, "palette")
        else:
            _check(self.rgb444, 12, "rgb444")
        _s(self.dl, 4)

    @property
    def bits(self) -> int:
        return 1 + (4 if self.palette is not None else 12) + 4


def _write_vfill(w: BitWriter, f: VFill) -> None:
    if f.palette is not None:
        w.write(0, 1)
        w.write(f.palette, 4)
    else:
        w.write(1, 1)
        w.write(f.rgb444, 12)
    w.write(_s(f.dl, 4), 4)


def _read_vfill(r: BitReader) -> VFill:
    if r.read(1) == 0:
        pal, rgb = r.read(4), None
    else:
        pal, rgb = None, r.read(12)
    return VFill(palette=pal, rgb444=rgb, dl=_u2s(r.read(4), 4))


# ---------------------------------------------------------------- records
# Coordinates are grid cells: grid 0 = 8 px (x 0..47, y 0..31); grid 1 = 4 px
# (x -16..111, y 0..63). "centre 13" fields use the 4 px grid.

def _check_grid_point(x: int, y: int, grid: int) -> None:
    if grid == 0:
        if not (0 <= x <= 47 and 0 <= y <= 31):
            raise ValueError(f"({x},{y}) outside the 8 px grid")
    else:
        if not (-16 <= x <= 111 and 0 <= y <= 63):
            raise ValueError(f"({x},{y}) outside the 4 px grid")


def _write_v0(w: BitWriter, x: int, y: int, grid: int) -> None:
    _check_grid_point(x, y, grid)
    if grid == 0:
        w.write(x, 6)
        w.write(y, 5)
    else:
        w.write(x + 16, 7)
        w.write(y, 6)


def _read_v0(r: BitReader, grid: int) -> tuple[int, int]:
    if grid == 0:
        return r.read(6), r.read(5)
    return r.read(7) - 16, r.read(6)


@dataclass(frozen=True)
class Upd:
    id: int
    dx: int         # 4 px units, -8..7
    dy: int


@dataclass(frozen=True)
class Poly:
    id: int
    grid: int                                   # 0 = 8 px, 1 = 4 px
    vertices: tuple[tuple[int, int], ...]       # 3..10 absolute cells, v0 first
    fill: Fill


@dataclass(frozen=True)
class Tree:
    id: int
    cx: int         # 8 px grid
    cy: int
    rx: int         # code 0..7, radius (code+1)·4 px
    ry: int
    fill: Fill
    trunk_h: int | None = None      # code 0..7 → (code+1)·8 px; None = no trunk


@dataclass(frozen=True)
class Edge:
    id: int
    cls: int                                    # 0..4 (5..7 reject)
    points: tuple[tuple[int, int], ...]         # 2..5 points, 4 px grid, absolute


@dataclass(frozen=True)
class HznAbs:
    y: int          # y8 code: y_px = 2·code − 128 at x = 192
    ang: int        # ang6, 0.5° steps, -32..31
    curv: int       # curv4, 4 px steps, -8..7
    sky: VFill
    gnd: VFill


@dataclass(frozen=True)
class HznResid:
    dy: int         # dy5, 2 px, -16..15
    dang: int       # dang4, 0.5°, -8..7


@dataclass(frozen=True)
class HznColours:
    sky: VFill
    gnd: VFill


@dataclass(frozen=True)
class HznNoHorizon:
    top: VFill
    bottom: VFill


@dataclass(frozen=True)
class Del:
    id: int


@dataclass(frozen=True)
class Ucol:
    id: int
    fill: Fill


@dataclass(frozen=True)
class Gshift:
    grp: int        # 0 far, 1 ground, 2 all, 3 L4
    dx: int         # 2 px units, -128..127
    dy: int         # 2 px units, -64..63


@dataclass(frozen=True)
class Status:
    arm_src: int    # 0 none, 1 static, 2 sensed, 3 estimated
    arm: int        # code 0..127, angle = code − 30°
    bkt_src: int
    bkt: int        # code 0..127, (code − 64)·1.5° relative to the arm
    conf: int       # 0..3
    corr_n: int     # 0..7
    mask_anom: bool
    moving: bool


@dataclass(frozen=True)
class Anom:
    slot: int       # 0..7 → id 80 + slot
    x: int          # top-left, 8 px grid
    y: int
    w: int          # code 0..7 → (code+1)·8 px
    h: int
    rgb444: int


@dataclass(frozen=True)
class Blob:
    id: int
    cx: int         # 4 px grid
    cy: int
    rx: int         # code 0..7 → (code+1)·4 px
    ry: int
    rot: int        # 0..7 → 22.5° steps
    fill: Fill


@dataclass(frozen=True)
class Skyline:
    unit: int                       # sc2: height unit ×4/×8/×16/×32 px
    heights: tuple[int, ...]        # 8/12/16/24 codes 0..7
    fill: Fill


@dataclass(frozen=True)
class Insert:
    id: int
    edge: int       # define edge 0..n−1
    k: int          # 0..3, order within the edge
    dx: int         # from the define-edge midpoint, define grid cells
    dy: int


@dataclass(frozen=True)
class Confirm:
    base_id: int
    tags: tuple[int | None, ...]    # 1..16 entries; None = mask bit clear, else tag2 0..3


@dataclass(frozen=True)
class Digest:
    n_live: int     # 0..127
    crc: int        # 0..255


@dataclass(frozen=True)
class LayerClear:
    range: int      # 0 = L1–L4, 1 = L2–L4, 2 = L3–L4, 3 = L4


@dataclass(frozen=True)
class CalRev:
    cal: int        # low 16 bits of the calibration SHA-256
    mask: int


@dataclass(frozen=True)
class Pal:
    slot: int       # 0..7 → palette slot 8 + slot
    rgb444: int


@dataclass(frozen=True)
class Gzoom:
    code: int       # u = (code − 128) / 4096 px⁻¹


@dataclass(frozen=True)
class Gain:
    r: int          # codes 0..31, gain = 2^((code − 16) / 32)
    g: int
    b: int


@dataclass(frozen=True)
class Hole:
    parent: int
    slot: int       # 0..3
    delete: bool
    cx: int         # 4 px grid
    cy: int
    r: int          # code 0..7 → (code+1)·4 px


Record = Union[Upd, Poly, Tree, Edge, HznAbs, HznResid, HznColours, HznNoHorizon,
               Del, Ucol, Gshift, Status, Anom, Blob, Skyline, Insert, Confirm,
               Digest, LayerClear, CalRev, Pal, Gzoom, Gain, Hole]

# Canonical prefix code (§3.3). Kraft sum is exactly 1: see kraft_sum().
_PREFIX = {
    Upd: "00", Poly: "010", Tree: "011", Edge: "100", "HZN": "1010", Del: "1011",
    Ucol: "1100", Gshift: "1101", Status: "11100", Anom: "11101", Blob: "11110",
    Skyline: "111110", "EXT": "111111",
}
_HZN_MODE = {HznAbs: 0, HznResid: 1, HznColours: 2, HznNoHorizon: 3}
_EXT_SUB = {Insert: 0, Confirm: 1, Digest: 2, LayerClear: 3, CalRev: 4, Pal: 5,
            Gzoom: 6, Gain: 7, Hole: 8}
_EXT_BY_SUB = {v: k for k, v in _EXT_SUB.items()}
_HZN_BY_MODE = {v: k for k, v in _HZN_MODE.items()}


def kraft_sum() -> float:
    """Σ 2^-len over the top-level prefixes; the code is complete when it is 1."""
    return sum(2.0 ** -len(code) for code in _PREFIX.values())


def _write_prefix(w: BitWriter, code: str) -> None:
    w.write(int(code, 2), len(code))


def _id_ok(id_: int, allowed: range | None = None) -> int:
    if id_ == 0 or not 1 <= id_ <= 127:
        raise ValueError(f"id {id_} outside 1..127")
    if allowed is not None and id_ not in allowed:
        raise ValueError(f"id {id_} outside {allowed}")
    return id_


def _poly_id_ok(id_: int) -> int:
    if id_ not in ID_MASS and id_ not in ID_CORRIDOR:
        raise ValueError(f"POLY id {id_} must be 1..31 or 80..87")
    return id_


# ---- packing -------------------------------------------------------------

def pack_record(rec: Record, w: BitWriter | None = None) -> BitWriter:
    """Append one record (prefix included) to ``w``; validates every field."""
    w = w if w is not None else BitWriter()
    t = type(rec)
    if t is Upd:
        _write_prefix(w, _PREFIX[Upd])
        w.write(_id_ok(rec.id), 7)
        w.write(_s(rec.dx, 4), 4)
        w.write(_s(rec.dy, 4), 4)
    elif t is Poly:
        n = len(rec.vertices)
        if not 3 <= n <= 10:
            raise ValueError("POLY needs 3..10 vertices")
        _write_prefix(w, _PREFIX[Poly])
        w.write(_poly_id_ok(rec.id), 7)
        w.write(_check(rec.grid, 1, "grid"), 1)
        w.write(n - 3, 3)
        x0, y0 = rec.vertices[0]
        _write_v0(w, x0, y0, rec.grid)
        px, py = x0, y0
        for (x, y) in rec.vertices[1:]:
            _write_eg2(w, x - px)
            _write_eg2(w, y - py)
            px, py = x, y
        _write_fill(w, rec.fill)
    elif t is Tree:
        _write_prefix(w, _PREFIX[Tree])
        w.write(_id_ok(rec.id, ID_PLANT), 7)
        _write_v0(w, rec.cx, rec.cy, 0)
        w.write(_check(rec.rx, 3, "rx"), 3)
        w.write(_check(rec.ry, 3, "ry"), 3)
        _write_fill(w, rec.fill)
        if rec.trunk_h is None:
            w.write(0, 1)
        else:
            w.write(1, 1)
            w.write(_check(rec.trunk_h, 3, "trunk_h"), 3)
    elif t is Edge:
        k = len(rec.points) - 1
        if not 1 <= k <= 4:
            raise ValueError("EDGE needs 2..5 points")
        if not 0 <= rec.cls <= 4:
            raise ValueError("EDGE cls must be 0..4")
        _write_prefix(w, _PREFIX[Edge])
        w.write(_id_ok(rec.id, ID_EDGE), 7)
        w.write(rec.cls, 3)
        x0, y0 = rec.points[0]
        _write_v0(w, x0, y0, 1)
        w.write(k - 1, 2)
        px, py = x0, y0
        for (x, y) in rec.points[1:]:
            _write_eg2(w, x - px)
            _write_eg2(w, y - py)
            px, py = x, y
    elif t in _HZN_MODE:
        _write_prefix(w, _PREFIX["HZN"])
        w.write(_HZN_MODE[t], 2)
        if t is HznAbs:
            w.write(_check(rec.y, 8, "y"), 8)
            w.write(_s(rec.ang, 6), 6)
            w.write(_s(rec.curv, 4), 4)
            _write_vfill(w, rec.sky)
            _write_vfill(w, rec.gnd)
        elif t is HznResid:
            w.write(_s(rec.dy, 5), 5)
            w.write(_s(rec.dang, 4), 4)
        elif t is HznColours:
            _write_vfill(w, rec.sky)
            _write_vfill(w, rec.gnd)
        else:
            _write_vfill(w, rec.top)
            _write_vfill(w, rec.bottom)
    elif t is Del:
        _write_prefix(w, _PREFIX[Del])
        w.write(_id_ok(rec.id), 7)
    elif t is Ucol:
        _write_prefix(w, _PREFIX[Ucol])
        w.write(_id_ok(rec.id), 7)
        _write_fill(w, rec.fill)
    elif t is Gshift:
        _write_prefix(w, _PREFIX[Gshift])
        w.write(_check(rec.grp, 2, "grp"), 2)
        w.write(_s(rec.dx, 8), 8)
        w.write(_s(rec.dy, 7), 7)
    elif t is Status:
        _write_prefix(w, _PREFIX[Status])
        w.write(_check(rec.arm_src, 2, "arm_src"), 2)
        w.write(_check(rec.arm, 7, "arm"), 7)
        w.write(_check(rec.bkt_src, 2, "bkt_src"), 2)
        w.write(_check(rec.bkt, 7, "bkt"), 7)
        w.write(_check(rec.conf, 2, "conf"), 2)
        w.write(_check(rec.corr_n, 3, "corr_n"), 3)
        w.write(1 if rec.mask_anom else 0, 1)
        w.write(1 if rec.moving else 0, 1)
    elif t is Anom:
        _write_prefix(w, _PREFIX[Anom])
        w.write(_check(rec.slot, 3, "slot"), 3)
        _write_v0(w, rec.x, rec.y, 0)
        w.write(_check(rec.w, 3, "w"), 3)
        w.write(_check(rec.h, 3, "h"), 3)
        w.write(_check(rec.rgb444, 12, "rgb444"), 12)
    elif t is Blob:
        _write_prefix(w, _PREFIX[Blob])
        w.write(_id_ok(rec.id, ID_L4), 7)
        _write_v0(w, rec.cx, rec.cy, 1)
        w.write(_check(rec.rx, 3, "rx"), 3)
        w.write(_check(rec.ry, 3, "ry"), 3)
        w.write(_check(rec.rot, 3, "rot"), 3)
        _write_fill(w, rec.fill)
    elif t is Skyline:
        n = len(rec.heights)
        if n not in SKYLINE_SAMPLES:
            raise ValueError("SKYLINE needs 8, 12, 16 or 24 samples")
        _write_prefix(w, _PREFIX[Skyline])
        w.write(SKYLINE_SAMPLES.index(n), 2)
        w.write(_check(rec.unit, 2, "unit"), 2)
        for h in rec.heights:
            w.write(_check(h, 3, "height"), 3)
        _write_fill(w, rec.fill)
    elif t in _EXT_SUB:
        _write_prefix(w, _PREFIX["EXT"])
        w.write(_EXT_SUB[t], 4)
        if t is Insert:
            w.write(_id_ok(rec.id), 7)
            w.write(_check(rec.edge, 4, "edge"), 4)
            w.write(_check(rec.k, 2, "k"), 2)
            _write_eg2(w, rec.dx)
            _write_eg2(w, rec.dy)
        elif t is Confirm:
            cnt = len(rec.tags)
            if not 1 <= cnt <= 16:
                raise ValueError("CONFIRM covers 1..16 ids")
            _id_ok(rec.base_id)
            if rec.base_id + cnt - 1 > 127:
                raise ValueError("CONFIRM range runs past id 127")
            w.write(rec.base_id, 7)
            w.write(cnt - 1, 4)
            for tag in rec.tags:
                w.write(0 if tag is None else 1, 1)
            for tag in rec.tags:
                if tag is not None:
                    w.write(_check(tag, 2, "tag"), 2)
        elif t is Digest:
            w.write(_check(rec.n_live, 7, "n_live"), 7)
            w.write(_check(rec.crc, 8, "crc"), 8)
        elif t is LayerClear:
            w.write(_check(rec.range, 2, "range"), 2)
        elif t is CalRev:
            w.write(_check(rec.cal, 16, "cal"), 16)
            w.write(_check(rec.mask, 16, "mask"), 16)
        elif t is Pal:
            w.write(_check(rec.slot, 3, "slot"), 3)
            w.write(_check(rec.rgb444, 12, "rgb444"), 12)
        elif t is Gzoom:
            w.write(_check(rec.code, 8, "code"), 8)
        elif t is Gain:
            w.write(_check(rec.r, 5, "r"), 5)
            w.write(_check(rec.g, 5, "g"), 5)
            w.write(_check(rec.b, 5, "b"), 5)
        else:  # Hole
            w.write(_id_ok(rec.parent), 7)
            w.write(_check(rec.slot, 2, "slot"), 2)
            w.write(1 if rec.delete else 0, 1)
            _write_v0(w, rec.cx, rec.cy, 1)
            w.write(_check(rec.r, 3, "r"), 3)
    else:
        raise TypeError(f"not a VS1 record: {rec!r}")
    return w


def record_bits(rec: Record) -> int:
    return pack_record(rec).bit_count


# ---- parsing -------------------------------------------------------------

_PREFIX_LOOKUP = {code: kind for kind, code in _PREFIX.items()}


def _read_kind(r: BitReader):
    code = ""
    while len(code) <= 6:
        code += "1" if r.read(1) else "0"
        kind = _PREFIX_LOOKUP.get(code)
        if kind is not None:
            return kind
    raise VsDecodeError("bad_prefix")     # unreachable for a complete code


def _read_id(r: BitReader, allowed: range | None = None) -> int:
    id_ = r.read(7)
    if id_ == 0:
        raise VsDecodeError("id_zero")
    if allowed is not None and id_ not in allowed:
        raise VsDecodeError("bad_id", f"{id_} not in {allowed}")
    return id_


def _read_record(r: BitReader) -> Record:
    kind = _read_kind(r)
    if kind is Upd:
        id_ = _read_id(r)
        return Upd(id_, _u2s(r.read(4), 4), _u2s(r.read(4), 4))
    if kind is Poly:
        id_ = r.read(7)
        if id_ == 0:
            raise VsDecodeError("id_zero")
        if id_ not in ID_MASS and id_ not in ID_CORRIDOR:
            raise VsDecodeError("bad_id", f"POLY id {id_}")
        grid = r.read(1)
        n = r.read(3) + 3
        x, y = _read_v0(r, grid)
        verts = [(x, y)]
        for _ in range(n - 1):
            x += _read_eg2(r)
            y += _read_eg2(r)
            verts.append((x, y))
        return Poly(id_, grid, tuple(verts), _read_fill(r))
    if kind is Tree:
        id_ = _read_id(r, ID_PLANT)
        cx, cy = _read_v0(r, 0)
        rx, ry = r.read(3), r.read(3)
        fill = _read_fill(r)
        trunk = r.read(3) if r.read(1) else None
        return Tree(id_, cx, cy, rx, ry, fill, trunk)
    if kind is Edge:
        id_ = _read_id(r, ID_EDGE)
        cls = r.read(3)
        if cls > 4:
            raise VsDecodeError("reserved_cls", str(cls))
        x, y = _read_v0(r, 1)
        k = r.read(2) + 1
        pts = [(x, y)]
        for _ in range(k):
            x += _read_eg2(r)
            y += _read_eg2(r)
            pts.append((x, y))
        return Edge(id_, cls, tuple(pts))
    if kind == "HZN":
        mode = _HZN_BY_MODE[r.read(2)]
        if mode is HznAbs:
            y = r.read(8)
            ang = _u2s(r.read(6), 6)
            curv = _u2s(r.read(4), 4)
            return HznAbs(y, ang, curv, _read_vfill(r), _read_vfill(r))
        if mode is HznResid:
            return HznResid(_u2s(r.read(5), 5), _u2s(r.read(4), 4))
        if mode is HznColours:
            return HznColours(_read_vfill(r), _read_vfill(r))
        return HznNoHorizon(_read_vfill(r), _read_vfill(r))
    if kind is Del:
        return Del(_read_id(r))
    if kind is Ucol:
        id_ = _read_id(r)
        return Ucol(id_, _read_fill(r))
    if kind is Gshift:
        return Gshift(r.read(2), _u2s(r.read(8), 8), _u2s(r.read(7), 7))
    if kind is Status:
        return Status(r.read(2), r.read(7), r.read(2), r.read(7), r.read(2), r.read(3),
                      bool(r.read(1)), bool(r.read(1)))
    if kind is Anom:
        slot = r.read(3)
        x, y = _read_v0(r, 0)
        return Anom(slot, x, y, r.read(3), r.read(3), r.read(12))
    if kind is Blob:
        id_ = _read_id(r, ID_L4)
        cx, cy = _read_v0(r, 1)
        rx, ry, rot = r.read(3), r.read(3), r.read(3)
        return Blob(id_, cx, cy, rx, ry, rot, _read_fill(r))
    if kind is Skyline:
        n = SKYLINE_SAMPLES[r.read(2)]
        unit = r.read(2)
        heights = tuple(r.read(3) for _ in range(n))
        return Skyline(unit, heights, _read_fill(r))
    # EXT
    sub = r.read(4)
    ext = _EXT_BY_SUB.get(sub)
    if ext is None:
        raise VsDecodeError("reserved_ext", str(sub))
    if ext is Insert:
        id_ = _read_id(r)
        edge, k = r.read(4), r.read(2)
        return Insert(id_, edge, k, _read_eg2(r), _read_eg2(r))
    if ext is Confirm:
        base = _read_id(r)
        cnt = r.read(4) + 1
        if base + cnt - 1 > 127:
            raise VsDecodeError("bad_id", "CONFIRM range past 127")
        mask = [r.read(1) for _ in range(cnt)]
        tags = tuple(r.read(2) if bit else None for bit in mask)
        return Confirm(base, tags)
    if ext is Digest:
        return Digest(r.read(7), r.read(8))
    if ext is LayerClear:
        return LayerClear(r.read(2))
    if ext is CalRev:
        return CalRev(r.read(16), r.read(16))
    if ext is Pal:
        return Pal(r.read(3), r.read(12))
    if ext is Gzoom:
        return Gzoom(r.read(8))
    if ext is Gain:
        return Gain(r.read(5), r.read(5), r.read(5))
    parent = _read_id(r)
    slot = r.read(2)
    delete = bool(r.read(1))
    cx, cy = _read_v0(r, 1)
    return Hole(parent, slot, delete, cx, cy, r.read(3))


# ---------------------------------------------------------------- frames

@dataclass(frozen=True)
class Header:
    key: bool           # K: epoch-start frame (mirrors frame_kind)
    age: int            # AAAA, 200 ms units, 15 = saturated
    epoch: int          # EEEE, mod 16
    level: int = 0      # LL, V0..V3

    def __post_init__(self) -> None:
        _check(self.age, 4, "age")
        _check(self.epoch, 4, "epoch")
        _check(self.level, 2, "level")


@dataclass(frozen=True)
class Frame:
    header: Header
    records: tuple[Record, ...]


def encode_frame(header: Header, records: "list[Record] | tuple[Record, ...]",
                 f_bytes: int) -> bytes:
    """Header + records, zero-padded to a byte; raises FrameTooLarge over F."""
    w = BitWriter()
    w.write(0, 1)                   # marker
    w.write(VS_VERSION, 1)
    w.write(1 if header.key else 0, 1)
    w.write(header.age, 4)
    w.write(header.epoch, 4)
    w.write(header.level, 2)
    if w.bit_count > f_bytes * 8:
        raise FrameTooLarge(f"header alone ({w.bit_count} bits) exceeds F = {f_bytes} B")
    for rec in records:
        pack_record(rec, w)
        if w.bit_count > f_bytes * 8:
            raise FrameTooLarge(f"{w.bit_count} bits exceed F = {f_bytes} B")
    return w.to_bytes()


def decode_frame(body: bytes, frame_kind: int | None = None) -> Frame:
    """All-or-nothing parse per §3.4. ``frame_kind`` (the TileDeltaFrame byte)
    is checked against K when given."""
    if len(body) < 2:
        raise VsDecodeError("short")
    r = BitReader(body)
    if r.read(1):
        raise VsDecodeError("bad_marker")
    if r.read(1) != VS_VERSION:
        raise VsDecodeError("bad_version")
    key = bool(r.read(1))
    header = Header(key, r.read(4), r.read(4), r.read(2))
    if frame_kind is not None and bool(frame_kind) != key:
        raise VsDecodeError("key_mismatch")
    records: list[Record] = []
    while r.remaining() and not r.remaining_all_zero():
        try:
            records.append(_read_record(r))
        except _Eof:
            raise VsDecodeError("truncated") from None
    return Frame(header, tuple(records))


# ---------------------------------------------------------------- CRC pins

def crc8(data: bytes) -> int:
    """CRC-8, poly 0x07, init 0x00, no reflection, xorout 0 ("123456789" → 0xF4)."""
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE, poly 0x1021, init 0xFFFF, no reflection ("123456789" → 0x29B1)."""
    c = 0xFFFF
    for b in data:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def define_hash(rec: Record) -> int:
    """CRC-16 over the define record's packed bytes, prefix through FILL, zero-padded."""
    return crc16_ccitt_false(pack_record(rec).to_bytes())


def _centre_code(cx: int, cy: int) -> int:
    return ((cx + 16) << 6) | cy


def state_hash(dx: int, dy: int, base_rgb444: int, inserts: "list[Insert]",
               holes: "list[Hole | None]") -> int:
    """Per-shape state-hash (§3.3): local offset, base colour, INSERTs in
    canonical (edge, k) order, then the four HOLE slots."""
    buf = bytearray()
    buf += _s(dx, 8).to_bytes(1, "big") + _s(dy, 8).to_bytes(1, "big")
    buf += base_rgb444.to_bytes(2, "little")
    buf.append(len(inserts) & 0xFF)
    for ins in sorted(inserts, key=lambda i: (i.edge, i.k)):
        buf += pack_record(ins).to_bytes()
    slots: list = list(holes) + [None] * (4 - len(holes))
    for hole in slots[:4]:
        if hole is None:
            buf += b"\x00" + b"\x00\x00" + b"\x00"
        else:
            buf += b"\x01" + _centre_code(hole.cx, hole.cy).to_bytes(2, "little") + bytes([hole.r])
    return crc16_ccitt_false(bytes(buf))


def digest_crc(shapes: "list[tuple[int, int, int]]", gshifts: "list[tuple[int, int]]",
               gzoom: int, gain: tuple[int, int, int]) -> int:
    """DIGEST crc8 (§3.3): shapes as (id, define_hash, state_hash) sorted by id,
    then the render state — four GSHIFT groups (dx, dy codes), GZOOM, GAIN."""
    buf = bytearray()
    for id_, dh, sh in sorted(shapes):
        buf.append(id_ & 0xFF)
        buf += (dh & 0xFFFF).to_bytes(2, "little")
        buf += (sh & 0xFFFF).to_bytes(2, "little")
    groups = list(gshifts) + [(0, 0)] * (4 - len(gshifts))
    for dx, dy in groups[:4]:
        buf += _s(dx, 8).to_bytes(1, "big") + _s(dy, 8).to_bytes(1, "big")
    buf.append(gzoom & 0xFF)
    buf += bytes(g & 0xFF for g in gain)
    return crc8(bytes(buf))
