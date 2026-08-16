"""Tractor-side NanoDet-Plus 320×320 INT8 detector scaffold.

Mirror of :mod:`base_station.image_pipeline.detect_yolo` for the tractor
side, but locked to NanoDet-Plus (Apache-2.0) — no AGPL surface on the
machine itself. The base side still does its independent cross-check;
this module produces the per-frame detection vector the M7 ships on
LoRa topic ``0x26``.
"""
from __future__ import annotations

import logging
import os
import struct
from dataclasses import dataclass

LOG = logging.getLogger("detect_nanodet")

# Topic 0x26 wire format — one detection vector per call:
#   u8  count
#   repeated count times:
#       u8  cls_id          ; index into CLASS_TABLE below
#       u8  conf_x100       ; 0..100
#       i8  cx_q7           ; centre x normalised to int8 (q7) (-1.0..1.0 mapped to -128..127)
#       i8  cy_q7
#       u8  w_q8            ; 0..255 → 0..1
#       u8  h_q8
CLASS_TABLE = ["person", "vehicle", "animal", "obstacle", "implement", "other"]


@dataclass
class TractorDetection:
    cls: str
    conf: float
    cx: float        # 0..1 normalised
    cy: float
    w: float
    h: float


class _NanoDetStub:
    name = "stub"

    def infer(self, frame_bytes: bytes, width: int, height: int) -> list[TractorDetection]:
        return []


class _NanoDetOnnx:                               # pragma: no cover
    name = "nanodet_onnx_int8"

    def __init__(self, weights_path: str, input_size: int = 320,
                 score_threshold: float = 0.4) -> None:
        import onnxruntime                       # type: ignore
        self._sess = onnxruntime.InferenceSession(
            weights_path, providers=["CPUExecutionProvider"])
        self._input_size = input_size
        self._score_threshold = score_threshold

    def infer(self, frame_bytes: bytes, width: int, height: int) -> list[TractorDetection]:
        del frame_bytes, width, height
        return []


def make_detector() -> object:
    weights = os.environ.get("LIFETRAC_TRACTOR_DETECTOR_WEIGHTS", "")
    if not weights:
        LOG.warning("detect_nanodet: no weights path; using stub")
        return _NanoDetStub()
    try:
        return _NanoDetOnnx(weights)
    except ImportError as exc:                    # pragma: no cover
        LOG.warning("detect_nanodet: onnxruntime unavailable (%s); using stub", exc)
        return _NanoDetStub()


def pack_detection_frame(detections: list[TractorDetection]) -> bytes:
    """Serialise into the topic-0x26 wire format."""
    if len(detections) > 255:
        detections = detections[:255]
    out = bytearray([len(detections)])
    for d in detections:
        cls_id = CLASS_TABLE.index(d.cls) if d.cls in CLASS_TABLE \
                 else CLASS_TABLE.index("other")
        conf_b = max(0, min(100, int(round(d.conf * 100))))
        cx = max(-128, min(127, int(round((d.cx * 2 - 1) * 127))))
        cy = max(-128, min(127, int(round((d.cy * 2 - 1) * 127))))
        w = max(0, min(255, int(round(d.w * 255))))
        h = max(0, min(255, int(round(d.h * 255))))
        out += struct.pack("<BBbbBB", cls_id, conf_b, cx, cy, w, h)
    return bytes(out)
