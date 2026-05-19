"""Authoritative state publisher.

Per IMAGE_PIPELINE.md §6.1 the trust boundary requires that *all* operator
display state — tile blobs, age, badge enum, detections, safety verdicts,
accelerator status — be produced server-side in this module. The browser
renders only what we hand it.

This module is dependency-free: it produces JSON-serialisable dicts. The
WebSocket transport lives in ``web_ui.py`` which calls
``StatePublisher.snapshot()`` whenever a canvas update arrives.
"""
from __future__ import annotations

import base64
import time
from dataclasses import dataclass, field
from typing import Any, Callable

from .canvas import Canvas


@dataclass
class Detection:
    """One bounding box from the base-side `detect_yolo.py` module."""
    cls: str
    conf: float
    x: float       # 0..1, normalised by canvas width
    y: float       # 0..1, normalised by canvas height
    w: float
    h: float


@dataclass
class SafetyVerdict:
    """Cross-detector agreement verdict, per IMAGE_PIPELINE.md V8."""
    agree: bool
    note: str = ""


@dataclass
class StatePublisher:
    """Mutable bag of state that the WS endpoint serialises on demand."""
    canvas: Canvas
    accel_status: str = "offline"
    encode_mode: str = "full"
    detections: list[Detection] = field(default_factory=list)
    safety_verdict: SafetyVerdict | None = None
    needs_keyframe: bool = False
    last_keyframe_reason: str = ""
    # S7.1 LINK-pill telemetry. Mirrors the per-direction adapter
    # decisions published on `lifetrac/v25/control/link_power/{direction}`
    # by `lora_bridge._airtime_worker` (S6.2 host half / S7.2 schema).
    # Schema per direction: ``{state, action, value, reason, power_dbm,
    # sf_rung, snr_ewma, per, per_sample_count}``. ``None`` when the
    # bench flag ``LIFETRAC_TX_POWER_ADAPTER_V3`` is OFF or the bridge
    # has not yet emitted a sample (cold-start before first tick).
    link_power: dict[str, dict[str, Any] | None] = field(
        default_factory=lambda: {"uplink": None, "downlink": None})
    clock_ms: Callable[[], int] = field(default=lambda: int(time.monotonic() * 1000))

    def snapshot(self) -> dict[str, Any]:
        """Build the full JSON payload that the browser expects."""
        canvas = self.canvas
        tiles_payload = []
        for index, slot in canvas.snapshot():
            tiles_payload.append({
                "i": index,
                "tx": index % canvas.grid_w,
                "ty": index // canvas.grid_w,
                "age_ms": slot.age_ms_at_publish,
                "badge": int(slot.badge),
                "blob_b64": base64.b64encode(slot.blob).decode("ascii"),
            })
        return {
            "ts_ms": self.clock_ms(),
            "grid": {
                "w": canvas.grid_w,
                "h": canvas.grid_h,
                "tile_px": canvas.tile_px,
            },
            "tiles": tiles_payload,
            "accel_status": self.accel_status,
            "encode_mode": self.encode_mode,
            "detections": [d.__dict__ for d in self.detections],
            "safety_verdict": (
                self.safety_verdict.__dict__ if self.safety_verdict else None
            ),
            "needs_keyframe": self.needs_keyframe,
            "last_keyframe_reason": self.last_keyframe_reason,
            "link_power": {
                "uplink": self.link_power.get("uplink"),
                "downlink": self.link_power.get("downlink"),
            },
        }
