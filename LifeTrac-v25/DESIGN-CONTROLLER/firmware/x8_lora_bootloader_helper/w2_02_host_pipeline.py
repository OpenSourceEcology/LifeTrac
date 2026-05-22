"""W2-02 host-side image pipeline.

Runs on the Windows orchestrator host (no PIL/numpy installs needed on
the X8). Two subcommands:

    encode --raw RAW_RGB_FILE --out FRAGMENTS_HEX [--orig-png ORIG.png]
        Slice a 384x256 RGB raw frame into 12x8 = 96 tiles, WebP-encode
        each, build a keyframe TileDeltaFrame, fragment via
        pack_telemetry_fragments (capped to fit the L072 firmware's 64 B
        TX_FRAME_REQ payload limit), and write one hex-encoded fragment
        body per line.

    decode --rx-log RX_LOG --out-png RECON.png [--summary SUMMARY.json]
        Parse `__RX_FRAME__ ... payload_hex=...` lines from the RX-side
        log, feed each into a FragmentReassembler, decode the first
        completed TileDeltaFrame, paint the 384x256 canvas, and write
        a PNG.

The wire path under test:

    RGB capture
      -> WebP per-tile encode
      -> encode_tile_delta_frame                          (frame_format.py)
      -> pack_telemetry_fragments(profile=PHY_IMAGE,
                                  max_air_ms=tuned)       (lora_proto.py)
      -> [send each fragment body as a raw LoRa TX over the L072 HostLink]
      -> [L072 RX_FRAME_URC delivers each body intact]
      -> FragmentReassembler.feed                         (reassemble.py)
      -> Canvas paint + PIL composite                     (canvas.py manual)

This bypasses the M7 (no TelemetryFrame envelope, topic_id, or CRC) so
we can prove the encode -> fragment -> air -> reassemble -> decode loop
without the H7 firmware being involved. When the M7 firmware ships, the
same fragment bodies will be wrapped in a TelemetryFrame on topic 0x25
and the base station will strip that envelope before feeding bodies into
the same FragmentReassembler.
"""
from __future__ import annotations

import argparse
import io
import json
import os
import re
import struct
import sys
from pathlib import Path

_REPO = Path(__file__).resolve().parents[3]   # .../LifeTrac-v25
_BS = _REPO / "DESIGN-CONTROLLER" / "base_station"
for p in (str(_BS),):
    if p not in sys.path:
        sys.path.insert(0, p)

from image_pipeline.frame_format import (  # noqa: E402
    FRAME_KIND_KEY,
    TileBlob,
    TileDeltaFrame,
    encode_tile_delta_frame,
)
from image_pipeline.reassemble import FragmentReassembler  # noqa: E402
# S1.3: newest-frame-wins counter. Observability primitive that surfaces
# stale / duplicate completed frames so the W2-02 stability gate can assert
# `stale_dropped == 0` once the S3 firmware coalescing fix lands.
from newest_frame_wins import NewestFrameWinsCounter  # noqa: E402
from lora_proto import (  # noqa: E402
    PHY_IMAGE,
    pack_telemetry_fragments,
)

GRID_W = 12
GRID_H = 8
TILE_PX = 32
CANVAS_W = GRID_W * TILE_PX   # 384
CANVAS_H = GRID_H * TILE_PX   # 256

# L072 firmware caps HostLink TX_FRAME_REQ payload at 64 B.
# Fragment body = 4-byte 0xFE header + data (v1), so usable data per fragment
# = 60 B. The W2-02 P0c redundancy header (0xFD) prepends one extra byte
# encoding `(total_copies<<4)|copy_idx`, so v2 fragments lose 1 B of payload.
L072_TX_MAX = 64
FRAG_DATA_MAX = L072_TX_MAX - 4        # 60 B (v1)
FRAG_DATA_MAX_V2 = L072_TX_MAX - 5     # 59 B (v2)
FRAGMENT_MAGIC_V2 = 0xFD
MAX_REDUNDANCY_V2 = 15                 # one nibble

# ---------------------------------------------------------------------------
# P3 (2026-05-21): cadence + dwell guardrails.
#
# Two independent constraints, often conflated:
#
#   1. Host-loop inter-cycle pacing (MIN_LORA_HOST_INTER_CYCLE_S).
#      The 2026-05-21 walk_power_falsification_matrix verdict found a
#      +13.38pp PER cliff at 0.02s vs 0.05s inter-cycle pacing on the
#      host orchestrator (TX board cadence). Anything below 50 ms drives
#      the L072 HostLink ring into back-pressure. LBT was FALSIFIED as
#      the cause; the cliff is host-side. Callers requesting <50 ms must
#      be clamped + warned.
#
#   2. FCC §15.247(a)(1) airtime cap (LEGAL_DWELL_US).
#      400 ms PER CHANNEL of airtime within any 10 s window. This is
#      AIRTIME, not wall-clock — long inter-frame gaps don't help. At
#      SF7/BW250 a fragment's ToA is ~25-30 us… err, ~25 ms, so 8 frames
#      ≈ 200-240 ms airtime, well under cap. At SF9/BW250 ToA jumps and
#      the safe per-dwell count falls. `frames_per_dwell()` recomputes
#      from the current profile's ToA and applies an 85% headroom factor
#      to leave room for retransmits and dwell-boundary slop.
#
# v4.1 review correction (2026-05-21): keep MAX_FRAMES_PER_DWELL_CAP=8
# (NOT 6 as v4.0 §2 suggested). v4.0's recommendation assumed worst-case
# ToA; at the deployed SF7/BW250 8 frames is legal. The profile-aware
# guard below is what protects SF9 etc.
# ---------------------------------------------------------------------------
MIN_LORA_HOST_INTER_CYCLE_S = 0.05      # walk_power matrix verdict (2026-05-21)
LEGAL_DWELL_US              = 400_000   # FCC §15.247(a)(1) per-channel airtime
DWELL_HEADROOM_PCT          = 85        # leave 15% for retransmits / slop
MAX_FRAMES_PER_DWELL_CAP    = 8         # v4.1: keep 8 (v4.0's "6" was overcautious)


def frames_per_dwell(toa_us: int) -> int:
    """Return the legal number of fragments per dwell for a given ToA.

    `toa_us` is the on-air time of a single fragment in microseconds. The
    result is the integer floor of (LEGAL_DWELL_US * headroom / toa_us),
    clamped to [1, MAX_FRAMES_PER_DWELL_CAP].

    Unit-test anchor (must hold for L072 firmware mirror too):
      - SF7  / BW250 / 64 B  (~28_000 us)  -> 8   (cap binds)
      - SF9  / BW250 / 64 B  (~115_000 us) -> 2-3 (ToA binds)
    """
    if toa_us <= 0:
        return 1
    n = (LEGAL_DWELL_US * DWELL_HEADROOM_PCT // 100) // toa_us
    if n < 1:
        return 1
    if n > MAX_FRAMES_PER_DWELL_CAP:
        return MAX_FRAMES_PER_DWELL_CAP
    return int(n)


def clamp_inter_cycle_s(requested_s: float, *, logger=None) -> float:
    """Enforce MIN_LORA_HOST_INTER_CYCLE_S with a one-shot warning.

    Returns the clamped value. Pass a callable `logger(str)` (e.g.
    `print` or `logging.warning`) to surface the clamp event; default is
    silent so it's safe to call from tight inner loops.
    """
    if requested_s < MIN_LORA_HOST_INTER_CYCLE_S:
        if logger is not None:
            logger(
                "P3-CLAMP: requested inter_cycle_s={:.4f} < min {:.4f}; "
                "raising (walk_power matrix 2026-05-21).".format(
                    requested_s, MIN_LORA_HOST_INTER_CYCLE_S
                )
            )
        return MIN_LORA_HOST_INTER_CYCLE_S
    return requested_s


def _encode_tile_webp(rgb: bytes, quality: int = 55) -> bytes:
    from PIL import Image
    img = Image.frombytes("RGB", (TILE_PX, TILE_PX), rgb)
    buf = io.BytesIO()
    img.save(buf, format="WEBP", quality=quality, method=4)
    return buf.getvalue()


def _slice_tile(frame_rgb: bytes, tx: int, ty: int) -> bytes:
    """Extract tile (tx, ty) from row-major RGB24 of size CANVAS_W x CANVAS_H."""
    out = bytearray(TILE_PX * TILE_PX * 3)
    row_stride = CANVAS_W * 3
    src_x = tx * TILE_PX * 3
    for row in range(TILE_PX):
        src = (ty * TILE_PX + row) * row_stride + src_x
        dst = row * TILE_PX * 3
        out[dst:dst + TILE_PX * 3] = frame_rgb[src:src + TILE_PX * 3]
    return bytes(out)


def _build_keyframe(frame_rgb: bytes, base_seq: int, quality: int) -> TileDeltaFrame:
    tiles: list[TileBlob] = []
    for idx in range(GRID_W * GRID_H):
        ty, tx = divmod(idx, GRID_W)
        rgb = _slice_tile(frame_rgb, tx, ty)
        blob = _encode_tile_webp(rgb, quality=quality)
        # WebP blob length cap is 256 B in the wire format (u8 size_minus1).
        # If quality 55 produces something bigger, step down.
        q = quality
        while len(blob) > 256 and q > 10:
            q -= 10
            blob = _encode_tile_webp(rgb, quality=q)
        if len(blob) > 256:
            raise RuntimeError(
                f"tile {idx} WebP {len(blob)}>256 B even at q={q}; raise tile_px?")
        tiles.append(TileBlob(idx, tx, ty, blob))
    return TileDeltaFrame(
        frame_kind=FRAME_KIND_KEY,
        base_seq=base_seq,
        grid_w=GRID_W,
        grid_h=GRID_H,
        tile_px=TILE_PX,
        changed_indices=list(range(GRID_W * GRID_H)),
        tiles=tiles,
    )


def _frag_capped(payload: bytes, frag_seq: int, redundancy: int = 1) -> list[bytes]:
    """Hand-chunk ``payload`` into L072-cappable fragment bodies.

    When ``redundancy == 1`` (default), emits the legacy v1 wire format
    (4-byte 0xFE header, body <= 60 B). When ``redundancy > 1``, emits
    the W2-02 P0c v2 wire format: 5-byte 0xFD header carrying an extra
    ``redundancy = (total_copies<<4)|copy_idx`` byte, body <= 59 B. Each
    unique fragment idx is emitted ``redundancy`` times back-to-back
    (copy_idx-major), so the receiver can dedup via the copy_idx
    bitmask in :class:`FragmentReassembler`.

    We deliberately do not interleave fragments across the redundancy
    dimension (Step 0 lock 2026-05-20: burstiness verdict found no
    benefit from interleaving on the bench link and the simpler
    "copy 0 then copy 1" ordering minimises reassembly window).

    Backwards-compat note: ``pack_telemetry_fragments`` in lora_proto.py
    sizes by airtime (PHY_IMAGE = SF7/BW500) and would produce fragments
    much larger than the L072 HostLink 64 B cap, so we don't reuse it.
    """
    from lora_proto import TELEMETRY_FRAGMENT_MAGIC
    if redundancy < 1:
        raise ValueError(f"redundancy must be >= 1, got {redundancy}")
    if redundancy > MAX_REDUNDANCY_V2:
        raise ValueError(f"redundancy {redundancy} > MAX_REDUNDANCY_V2 ({MAX_REDUNDANCY_V2})")
    chunk = FRAG_DATA_MAX if redundancy == 1 else FRAG_DATA_MAX_V2
    total = max(1, (len(payload) + chunk - 1) // chunk)
    if total > 256:
        raise RuntimeError(f"payload requires {total} fragments; max 256")
    out: list[bytes] = []
    for idx in range(total):
        data = payload[idx * chunk:(idx + 1) * chunk]
        if redundancy == 1:
            header = bytes([TELEMETRY_FRAGMENT_MAGIC,
                            frag_seq & 0xFF, idx & 0xFF,
                            (total - 1) & 0xFF])
            out.append(header + data)
        else:
            total_copies_nibble = (redundancy & 0x0F) << 4
            for copy_idx in range(redundancy):
                red_byte = total_copies_nibble | (copy_idx & 0x0F)
                header = bytes([FRAGMENT_MAGIC_V2,
                                frag_seq & 0xFF, idx & 0xFF,
                                (total - 1) & 0xFF,
                                red_byte])
                out.append(header + data)
    return out


def cmd_encode(args) -> int:
    raw_path = Path(args.raw)
    if not raw_path.exists():
        print(f"FATAL: raw input not found: {raw_path}", file=sys.stderr)
        return 2
    raw = raw_path.read_bytes()
    expected = CANVAS_W * CANVAS_H * 3
    if len(raw) != expected:
        print(f"FATAL: raw size {len(raw)} != expected {expected} (RGB24 384x256)",
              file=sys.stderr)
        return 2

    # Optional: dump the captured frame as PNG for visual diff.
    if args.orig_png:
        from PIL import Image
        Image.frombytes("RGB", (CANVAS_W, CANVAS_H), raw).save(args.orig_png)
        print(f"ENCODE: wrote original PNG -> {args.orig_png}")

    frame = _build_keyframe(raw, base_seq=args.base_seq, quality=args.quality)
    body = encode_tile_delta_frame(frame)
    fragments = _frag_capped(body, frag_seq=args.frag_seq, redundancy=args.redundancy)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="ascii", newline="\n") as fh:
        for frag in fragments:
            fh.write(frag.hex() + "\n")
    tile_sizes = [len(t.blob) for t in frame.tiles]
    print(f"ENCODE: tile count={len(frame.tiles)} "
          f"tile_bytes min={min(tile_sizes)} max={max(tile_sizes)} "
          f"sum={sum(tile_sizes)}")
    print(f"ENCODE: TileDeltaFrame body={len(body)} B "
          f"fragments={len(fragments)} (cap {FRAG_DATA_MAX} B/data)")
    print(f"ENCODE: wrote {len(fragments)} fragments -> {out_path}")
    return 0


_RX_FRAME_RE = re.compile(
    r"^__RX_FRAME__\s+.*?payload_hex=([0-9a-fA-F]+)", re.MULTILINE)


def cmd_decode(args) -> int:
    log_path = Path(args.rx_log)
    if not log_path.exists():
        print(f"FATAL: RX log not found: {log_path}", file=sys.stderr)
        return 2
    text = log_path.read_text(encoding="utf-8", errors="replace")
    hex_payloads = _RX_FRAME_RE.findall(text)
    print(f"DECODE: parsed {len(hex_payloads)} __RX_FRAME__ lines")
    reasm = FragmentReassembler()
    # S1.3: track every completed-frame base_seq in arrival order. The
    # counter is pure observability — it does not change which frame the
    # harness paints (that stays `completed[0]` for backward-compat). The
    # counter surfaces what the S3-gated behavioural change must produce:
    # `stale_dropped == 0` and `duplicate_dropped == 0` across a 10-minute
    # mixed-load run.
    newest = NewestFrameWinsCounter()
    completed: list[TileDeltaFrame] = []
    for h in hex_payloads:
        try:
            body = bytes.fromhex(h)
        except ValueError as exc:
            print(f"WARN: bad hex: {exc}")
            continue
        frame = reasm.feed(body)
        if frame is not None:
            completed.append(frame)
            newest.observe(frame.base_seq)
    print(f"DECODE: reassembler stats={reasm.stats} pending={reasm.pending_frag_seqs()}")
    print(f"DECODE: completed frames={len(completed)}")
    print(f"DECODE: newest_frame_wins={newest.stats_dict()}")
    if not completed:
        print("FATAL: no frame completed")
        if args.summary:
            Path(args.summary).write_text(json.dumps({
                "n_rx_frames": len(hex_payloads),
                "completed_frames": 0,
                "decode_errors": reasm.stats.decode_errors,
                "pending_seqs": reasm.pending_frag_seqs(),
                "newest_frame_wins": newest.stats_dict(),
            }, indent=2))
        return 3

    frame = completed[0]
    # Paint into a CANVAS_W x CANVAS_H PIL image.
    from PIL import Image
    canvas = Image.new("RGB", (CANVAS_W, CANVAS_H), (40, 40, 40))
    tiles_decoded = 0
    tiles_failed = 0
    for tile in frame.tiles:
        try:
            tile_img = Image.open(io.BytesIO(tile.blob)).convert("RGB")
            canvas.paste(tile_img, (tile.tx * TILE_PX, tile.ty * TILE_PX))
            tiles_decoded += 1
        except Exception as exc:
            tiles_failed += 1
            print(f"WARN: tile {tile.index} decode failed: {exc}")
    out_png = Path(args.out_png)
    out_png.parent.mkdir(parents=True, exist_ok=True)
    canvas.save(out_png)
    print(f"DECODE: tiles_decoded={tiles_decoded} tiles_failed={tiles_failed}")
    print(f"DECODE: wrote reconstructed canvas -> {out_png}")
    if args.summary:
        Path(args.summary).write_text(json.dumps({
            "n_rx_frames": len(hex_payloads),
            "completed_frames": len(completed),
            "first_frame": {
                "kind": "key" if frame.is_keyframe else "delta",
                "base_seq": frame.base_seq,
                "grid_w": frame.grid_w,
                "grid_h": frame.grid_h,
                "tile_px": frame.tile_px,
                "tiles_changed": len(frame.tiles),
                "tiles_decoded": tiles_decoded,
                "tiles_failed": tiles_failed,
            },
            "reassembler": {
                "completed_frames": reasm.stats.completed_frames,
                "decode_errors": reasm.stats.decode_errors,
                "duplicate_fragments": reasm.stats.duplicate_fragments,
                "bad_magic_passthroughs": reasm.stats.bad_magic_passthroughs,
                "timeouts": reasm.stats.timeouts,
            },
            "newest_frame_wins": newest.stats_dict(),
            "pending_seqs": reasm.pending_frag_seqs(),
        }, indent=2))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd", required=True)

    enc = sub.add_parser("encode")
    enc.add_argument("--raw", required=True, help="raw RGB24 384x256 input")
    enc.add_argument("--out", required=True, help="output fragments hex file")
    enc.add_argument("--orig-png", default=None, help="optional original PNG dump")
    enc.add_argument("--quality", type=int, default=55, help="WebP quality")
    enc.add_argument("--base-seq", type=int, default=0)
    enc.add_argument("--frag-seq", type=int, default=0)
    enc.add_argument("--redundancy", type=int, default=1,
                     help="W2-02 P0c redundancy factor N (1=v1 0xFE wire format "
                          "no redundancy; >=2 emits v2 0xFD header with "
                          "(total_copies<<4)|copy_idx). 2026-05-20 plan default "
                          "for stability runs: 2. Max 15 (one nibble).")
    enc.set_defaults(func=cmd_encode)

    dec = sub.add_parser("decode")
    dec.add_argument("--rx-log", required=True)
    dec.add_argument("--out-png", required=True)
    dec.add_argument("--summary", default=None)
    dec.set_defaults(func=cmd_decode)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
