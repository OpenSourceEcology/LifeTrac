"""Tap /ws/state and report unique tile positions over a sampling window.

Used to validate the canvas.py gap-tolerant merge fix.
"""
from __future__ import annotations
import json
import sys
import time
import urllib.request
from websocket import create_connection  # type: ignore

URL = "ws://192.168.1.79:8080/ws/state"
WINDOW_S = int(sys.argv[1]) if len(sys.argv) > 1 else 30

def main() -> int:
    ws = create_connection(URL, timeout=5)
    ws.settimeout(2)
    deadline = time.monotonic() + WINDOW_S
    snapshots = 0
    tile_msgs = 0
    uniq_positions: set[int] = set()
    last_grid = None
    keyframe_count = 0
    while time.monotonic() < deadline:
        try:
            raw = ws.recv()
        except Exception:
            continue
        if not raw:
            continue
        snapshots += 1
        try:
            m = json.loads(raw)
        except Exception:
            continue
        tiles = m.get("tiles") or []
        if tiles:
            tile_msgs += len(tiles)
            for t in tiles:
                pos = t.get("i", t.get("index"))
                if pos is not None:
                    uniq_positions.add(pos)
        if m.get("last_keyframe_reason"):
            keyframe_count += 1
        last_grid = m.get("grid")
    ws.close()
    print(f"snapshots={snapshots} tile_msgs={tile_msgs} uniq_tiles={len(uniq_positions)}/96")
    # Render grid
    positions = sorted(uniq_positions)
    grid_w = 12
    grid_h = 8
    for r in range(grid_h):
        row = []
        for c in range(grid_w):
            idx = r * grid_w + c
            row.append('#' if idx in uniq_positions else '.')
        print(' '.join(row))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
