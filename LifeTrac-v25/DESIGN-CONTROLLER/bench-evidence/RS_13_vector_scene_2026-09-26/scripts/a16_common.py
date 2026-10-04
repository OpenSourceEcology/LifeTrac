"""Shared helpers for the A16 SIL reproduction (kept with the RS-13.1 record).

Imports the real VectorEncoder and VectorSceneStore from the code tree the way
base_station/tests/test_vector_sync.py does (sys.path insert of base_station and
firmware/tractor_x8), and builds a busy synthetic moving scene at 96x64.
"""
from __future__ import annotations

import math
import os
import sys

# DESIGN-CONTROLLER root; this file lives in bench-evidence/RS_13_vector_scene_2026-09-26/scripts/
TREE = os.environ.get("LIFETRAC_DC_TREE") or os.path.abspath(
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", ".."))
BS_DIR = os.path.join(TREE, "base_station")
X8_DIR = os.path.join(TREE, "firmware", "tractor_x8")
TOOLS_DIR = os.path.join(TREE, "tools")
for _p in (BS_DIR, X8_DIR, TOOLS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)
os.environ.setdefault("LIFETRAC_FLEET_KEY_HEX", "0102030405060708090a0b0c0d0e0f10")

import numpy as np  # noqa: E402
import cv2  # noqa: E402

from image_pipeline.frame_format import CODEC_VECTOR, parse_tile_delta_frame  # noqa: E402
from image_pipeline.vector_scene import codec as vs  # noqa: E402
from image_pipeline.vector_scene_store import TTL_FRAMES, VectorSceneStore  # noqa: E402
from x8_image_pipeline import encode_vector as ev  # noqa: E402

W, H = 96, 64
CW, CH = 384, 256
SKY = (120, 170, 235)
GROUND = (120, 96, 60)
GREEN = (40, 140, 40)
RED = (200, 60, 60)
GREY = (150, 150, 150)
YELLOW = (200, 200, 40)
BLUE = (40, 60, 180)
PURPLE = (130, 50, 150)


def busy_scene(i: int, *, n_static: int = 14, n_move: int = 8, n_churn: int = 5, n_trees: int = 5,
               n_rot: int = 0, n_star: int = 0, n_static_trees: int = 0, noise: int = 8, seed_base: int = 0) -> np.ndarray:
    """Frame i of a busy synthetic scene (horizon at canvas row 8, so mostly
    ground): n_static static red squares (the CONFIRM-only shapes A16 is
    about), n_move grey/yellow rects on sinusoidal paths (UPDs and, past the
    +/-8 UPD field, redefines), n_churn blue rects that blink on/off out of
    phase (DEL + fresh define churn every frame), n_trees green blobs that
    drift (tree UPDs), plus +/-noise per canvas pixel."""
    img = np.zeros((H, W, 3), np.uint8)
    line = 8.0 / (CW / W)
    ys = np.arange(H)[:, None] + 0.5
    above = np.broadcast_to(ys < line, (H, W))
    img[above] = SKY
    img[~above] = GROUND
    yy, xx = np.mgrid[0:H, 0:W]
    # static grid: left/bottom part, 6x6 squares on a 10 px pitch
    k = 0
    for j in range(4):
        for c in range(5):
            if k >= n_static:
                break
            x0, y0 = 2 + 10 * c, 30 + 8 * j
            img[y0:y0 + 5, x0:x0 + 6] = RED
            k += 1
    # moving rects: upper band, sinusoidal paths of different phase/period
    for m in range(n_move):
        per = 9 + 3 * (m % 4)
        ax = 6 + 2 * (m % 3)
        cx = 8 + 11 * m + ax * math.sin(2 * math.pi * (i + 3 * m) / per)
        cy = 12 + 4 * (m % 3) + 2 * math.cos(2 * math.pi * (i + m) / (per + 2))
        colour = GREY if m % 2 == 0 else YELLOW
        x0, y0 = int(round(cx)) - 3, int(round(cy)) - 3
        x0 = max(0, min(W - 7, x0))
        img[y0:y0 + 6, x0:x0 + 7] = colour
    # churn: blue rects on the right that blink (on 5, off 4), staggered
    for b in range(n_churn):
        if (i + 2 * b) % 9 < 5:
            x0, y0 = 58 + 7 * (b % 5), 34 + 9 * (b // 5)
            size = 4 + (b % 2)
            img[y0:y0 + size, x0:x0 + size + 1] = BLUE
    # trees: green blobs drifting slowly right/left on the far right
    for t in range(n_trees):
        cx = 60 + 7 * t + 4 * math.sin(2 * math.pi * (i + 5 * t) / 17.0)
        cy = 54 + 3 * (t % 2)
        img[((xx - cx) ** 2 + (yy - cy) ** 2) <= 9] = GREEN
    # static trees along the bottom-left edge (plant ids; CONFIRM-only like the static squares)
    for t in range(n_static_trees):
        cx, cy = 5 + 9 * t, 60
        img[((xx - cx) ** 2 + (yy - cy) ** 2) <= 5] = (30, 110, 50)
    # rotating ellipses (outline changes every frame: Poly redefines), mid band
    for e in range(n_rot):
        cx, cy = 10 + 14 * (e % 6), 23 + 3 * (e // 6)
        ang = (i * (7 + 3 * e) + 30 * e) % 180
        colour = PURPLE if e % 2 == 0 else (90, 160, 160)
        cv2.ellipse(img, (int(cx), int(cy)), (5, 2), ang, 0, 360, colour, -1)
    # rotating, drifting 5-point stars (10-vertex outlines that change every
    # frame: the expensive Poly redefines that fill a frame like the video did)
    for e in range(n_star):
        cx = 12 + 18 * (e % 5) + 3 * math.sin(2 * math.pi * (i + 2 * e) / 11.0)
        cy = 22 + 14 * (e // 5) + 2 * math.cos(2 * math.pi * (i + e) / 13.0)
        rot = 2 * math.pi * ((i * (0.07 + 0.02 * e)) % 1.0)
        pts = []
        for v in range(10):
            r = 7.0 if v % 2 == 0 else 3.0
            a = rot + math.pi * v / 5
            pts.append((int(round(cx + r * math.cos(a))), int(round(cy + r * math.sin(a)))))
        colour = (210, 120, 40) if e % 2 == 0 else (60, 170, 200)
        cv2.fillPoly(img, [np.array(pts, np.int32)], colour)
    if noise:
        big = cv2.resize(img, (CW, CH), interpolation=cv2.INTER_NEAREST)
        rng = np.random.default_rng(seed_base + i)
        img = np.clip(big.astype(np.int16) + rng.integers(-noise, noise + 1, big.shape), 0, 255).astype(np.uint8)
    return img


def encode_stream(n_frames: int, budget: int, *, quality: int = 80, scene_kw: dict | None = None,
                  safety_s: float | None = None, patch=None, fps: float = 2.0):
    """Encode n_frames with the real encoder on a fake 2 fps clock. The VS1
    stream does not depend on what the base receives (no uplink), so it is
    encoded once and replayed into stores with any loss pattern.

    Returns a list of dicts: payload, frame_kind, vector_body, stats summary,
    the encoder's live set after the frame (id -> (dhash, state_hash,
    frames_since_verify)), carousel records packed, carousel bits."""
    scene_kw = scene_kw or {}
    cv2.setRNGSeed(0)
    old_safety = ev.SAFETY_REFRESH_S
    if safety_s is not None:
        ev.SAFETY_REFRESH_S = safety_s
    t = [1000.0]
    enc = ev.VectorEncoder(clock=lambda: t[0])
    if patch is not None:
        patch(enc)
    out = []
    try:
        for i in range(n_frames):
            img = busy_scene(i, **scene_kw)
            payload = enc.frame(img, budget, seq=(i + 1) & 0xFF, quality=quality)
            t[0] += 1.0 / fps
            st = enc.last_stats
            frame = parse_tile_delta_frame(payload)
            assert frame.codec == CODEC_VECTOR
            decoded = vs.decode_frame(frame.vector_body, frame.frame_kind)
            car_packed = [c for c in st["candidates"] if c[0] == 7 and c[4]]
            car_offered = [c for c in st["candidates"] if c[0] == 7]
            slot6_packed = sum(1 for c in st["candidates"] if c[0] == 6 and c[4])
            slot6_offered = sum(1 for c in st["candidates"] if c[0] == 6)
            defines = {r.id for r in decoded.records if isinstance(r, (vs.Poly, vs.Tree, vs.Edge))}
            dels = {r.id for r in decoded.records if isinstance(r, vs.Del)}
            digest = next((r for r in decoded.records if isinstance(r, vs.Digest)), None)
            out.append(dict(
                i=i, payload=payload, frame_kind=frame.frame_kind, body=frame.vector_body,
                bytes=len(payload), key=bool(st["epoch_start"]), committed=bool(st["epoch_committed"]),
                trigger=st.get("epoch_trigger"), n_live=st["n_live"],
                live={sid: (s.dhash, s.state_hash(), s.frames_since_verify, s.layer, s.repeat_left)
                      for sid, s in enc._shapes.items()},
                car_bits=st["carousel_bits"], car_packed=len(car_packed), car_offered=len(car_offered),
                car_packed_bits=sum(c[2] for c in car_packed),
                slot6_packed=slot6_packed, slot6_offered=slot6_offered,
                defines=defines, dels=dels, digest_n=None if digest is None else digest.n_live,
                n_records=len(decoded.records), records=dict(st["records"]),
                residual=st["residual"],
                slot5_unpacked=sum(1 for c in st["candidates"] if c[0] == 5 and not c[4]),
                slot5_offered=sum(1 for c in st["candidates"] if c[0] == 5),
                header=decoded.header,
                car_offered_ids={c[3][1] for c in car_offered},
                car_packed_ids={c[3][1] for c in car_packed},
                upd_ids={r.id for r in decoded.records if isinstance(r, (vs.Upd, vs.Ucol))},
                confirm_ids={r.base_id + k for r in decoded.records if isinstance(r, vs.Confirm)
                             for k, t in enumerate(r.tags) if t is not None},
                ttl_dropped_enc=st["ttl_dropped"], ms=st["ms"]["total"],
            ))
    finally:
        ev.SAFETY_REFRESH_S = old_safety
    return out


def replay(stream, drops: set, *, period_ms: int = 500):
    """Replay an encoded stream into a fresh real store, losing the frames
    whose index is in ``drops``. Per frame row: applied, digest_ok, resync,
    orphans added, base live count, ids the encoder counts live but the
    base lacks (missing), ids the base holds but the encoder does not
    (ghosts), ttl drops."""
    store = VectorSceneStore(CW, CH)
    rx = 10_000
    rows = []
    prev_orph = 0
    for fr in stream:
        rx += period_ms
        lost = fr["i"] in drops
        if not lost:
            res = store.ingest(fr["body"], fr["frame_kind"], rx, 0.0)
            assert res.applied, (fr["i"], res)
        st = store.stats
        base_ids = {sh.id for sh in store._live()}
        enc_ids = set(fr["live"])
        rows.append(dict(
            i=fr["i"], lost=lost, key=fr["key"],
            digest_ok=store._digest_ok if not lost else None,
            resync=st["resync"], orph_add=st["orphans"] - prev_orph,
            base_n=len(base_ids), enc_n=len(enc_ids),
            missing=sorted(enc_ids - base_ids), ghosts=sorted(base_ids - enc_ids),
            ttl=st["ttl_dropped"], resync_events=st["resync_events"],
            resync_digest_ends=st["resync_digest_ends"],
        ))
        prev_orph = st["orphans"]
    return rows
