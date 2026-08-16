"""ROI mask generator from valve activity + CMD_ROI_HINT.

The tractor's H747 publishes valve commanded states + the active loader
mode (loading / driving / idle) over IPC. We translate that into a
per-tile importance mask used by :mod:`encode_tile_delta` to assign a
higher WebP quality to tiles inside the ROI than to tiles outside it.

When the base sends ``CMD_ROI_HINT`` (opcode ``0x61``) the operator-defined
focus rectangle takes precedence for ``hint_ttl_ms`` after receipt.

The grid is the same 12×8 used by camera_service.py.
"""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable

# Default per-mode ROIs as (col_lo, col_hi, row_lo, row_hi) inclusive on the
# 12×8 tile grid. col is left→right, row is top→bottom.
DEFAULT_ROIS_BY_MODE = {
    "loading": (1, 10, 4, 7),     # bottom-centre — bucket
    "driving": (3, 8, 0, 4),      # upper-centre — horizon
    "idle":    (0, 11, 0, 7),     # whole frame
    "reverse": (3, 8, 4, 7),      # rear thumbnail centre
}


@dataclass
class RoiHint:
    col_lo: int
    col_hi: int
    row_lo: int
    row_hi: int
    received_ms: int


@dataclass
class RoiPlanner:
    grid_w: int = 12
    grid_h: int = 8
    hint_ttl_ms: int = 5000
    clock_ms: Callable[[], int] = field(default=lambda: int(time.monotonic() * 1000))
    _hint: RoiHint | None = None
    mode: str = "idle"

    def update_mode(self, new_mode: str) -> None:
        if new_mode in DEFAULT_ROIS_BY_MODE:
            self.mode = new_mode

    def apply_hint(self, col_lo: int, col_hi: int, row_lo: int, row_hi: int) -> None:
        col_lo = max(0, min(self.grid_w - 1, col_lo))
        col_hi = max(col_lo, min(self.grid_w - 1, col_hi))
        row_lo = max(0, min(self.grid_h - 1, row_lo))
        row_hi = max(row_lo, min(self.grid_h - 1, row_hi))
        self._hint = RoiHint(col_lo, col_hi, row_lo, row_hi, self.clock_ms())

    def current_roi(self) -> tuple[int, int, int, int]:
        if self._hint is not None:
            age = self.clock_ms() - self._hint.received_ms
            if age <= self.hint_ttl_ms:
                return (self._hint.col_lo, self._hint.col_hi,
                        self._hint.row_lo, self._hint.row_hi)
            self._hint = None
        return DEFAULT_ROIS_BY_MODE.get(self.mode, DEFAULT_ROIS_BY_MODE["idle"])

    def mask(self) -> list[bool]:
        """Per-tile boolean mask in row-major order; True = inside ROI."""
        col_lo, col_hi, row_lo, row_hi = self.current_roi()
        out = [False] * (self.grid_w * self.grid_h)
        for r in range(row_lo, row_hi + 1):
            for c in range(col_lo, col_hi + 1):
                out[r * self.grid_w + c] = True
        return out

    def quality_for_tile(self, tile_index: int,
                         q_in: int = 60, q_out: int = 25) -> int:
        """Used by encode_tile_delta to pick a per-tile WebP quality."""
        return q_in if self.mask()[tile_index] else q_out
