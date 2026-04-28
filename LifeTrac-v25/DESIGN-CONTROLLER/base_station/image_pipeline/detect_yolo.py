"""Base-side independent safety detector scaffold.

Per IMAGE_PIPELINE.md §5B (R6, two-detector pattern). The base station runs
its own object detector against the reassembled canvas and cross-checks
results against the tractor-side ``detect_nanodet.py``; a high-confidence
disagreement raises a banner via ``state_publisher.safety_verdict``.

Backend selection order:
  1. ``LIFETRAC_DETECTOR=yolov8`` env var → Ultralytics YOLOv8-nano (AGPL-3.0,
     opt-in only — see open scope decision O1).
  2. Default → NanoDet-Plus through onnxruntime (Apache-2.0).
  3. If neither runtime is importable, falls back to ``NullDetector`` which
     produces no detections so the rest of the pipeline keeps working.

Model weight files are not bundled; ``LIFETRAC_DETECTOR_WEIGHTS`` selects
the path. Inference runs in a worker thread, capped at ``MAX_FPS`` (1 fps
default — base CPU is shared with the rest of the operator console).
"""
from __future__ import annotations

import logging
import os
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Iterable

LOG = logging.getLogger("detect_yolo")
MAX_FPS_DEFAULT = 1.0


@dataclass
class DetectorResult:
    cls: str
    conf: float
    x: float           # 0..1 normalised
    y: float
    w: float
    h: float


class NullDetector:
    """No-op detector used when no runtime/weights are available."""

    name = "null"

    def detect(self, frame_bytes: bytes, width: int, height: int) -> list[DetectorResult]:
        return []


class NanoDetOnnxDetector:
    """NanoDet-Plus via onnxruntime."""

    name = "nanodet_onnx"

    def __init__(self, weights_path: str, input_size: int = 320,
                 score_threshold: float = 0.4) -> None:
        import onnxruntime                       # type: ignore
        self._sess = onnxruntime.InferenceSession(
            weights_path, providers=["CPUExecutionProvider"])
        self._input_name = self._sess.get_inputs()[0].name
        self._input_size = input_size
        self._score_threshold = score_threshold

    def detect(self, frame_bytes: bytes, width: int, height: int) -> list[DetectorResult]:
        # Concrete pre/post-processing depends on the chosen NanoDet variant
        # and is wired in once the weights ship; the scaffold returns empty.
        # The signature is stable so the rest of the pipeline can be tested.
        del frame_bytes, width, height
        return []


class YoloV8Detector:
    """Ultralytics YOLOv8-nano. AGPL-3.0; opt-in only."""

    name = "yolov8"

    def __init__(self, weights_path: str, score_threshold: float = 0.4) -> None:
        from ultralytics import YOLO              # type: ignore
        self._model = YOLO(weights_path)
        self._score_threshold = score_threshold

    def detect(self, frame_bytes: bytes, width: int, height: int) -> list[DetectorResult]:
        del frame_bytes, width, height
        return []


def make_detector(weights_path: str | None = None,
                  override: str | None = None) -> object:
    backend = (override or os.environ.get("LIFETRAC_DETECTOR", "nanodet")).lower()
    weights = weights_path or os.environ.get("LIFETRAC_DETECTOR_WEIGHTS", "")
    if not weights:
        LOG.warning("detect_yolo: no weights path; using NullDetector")
        return NullDetector()
    try:
        if backend == "yolov8":
            return YoloV8Detector(weights)
        return NanoDetOnnxDetector(weights)
    except ImportError as exc:                    # pragma: no cover
        LOG.warning("detect_yolo: backend %s unavailable (%s); using NullDetector",
                    backend, exc)
        return NullDetector()


@dataclass
class CrossCheckVerdict:
    agree: bool
    note: str = ""


def cross_check(base: Iterable[DetectorResult],
                 tractor: Iterable[DetectorResult],
                 iou_threshold: float = 0.3) -> CrossCheckVerdict:
    """High-confidence disagreement check used by state_publisher.

    Two detections "agree" if their classes match and IoU >= ``iou_threshold``.
    Any base-side high-conf object with no matching tractor detection (and
    vice versa) drops ``agree`` to False and the operator UI surfaces a
    banner per IMAGE_PIPELINE.md V8.
    """
    base_l = list(base)
    tract_l = list(tractor)
    notes: list[str] = []
    for b in base_l:
        if b.conf < 0.7:
            continue
        if not any(_iou(b, t) >= iou_threshold and b.cls == t.cls for t in tract_l):
            notes.append(f"base-only {b.cls}@{b.conf:.2f}")
    for t in tract_l:
        if t.conf < 0.7:
            continue
        if not any(_iou(t, b) >= iou_threshold and t.cls == b.cls for b in base_l):
            notes.append(f"tractor-only {t.cls}@{t.conf:.2f}")
    if notes:
        return CrossCheckVerdict(agree=False, note=", ".join(notes[:4]))
    return CrossCheckVerdict(agree=True)


def _iou(a: DetectorResult, b: DetectorResult) -> float:
    ax2, ay2 = a.x + a.w, a.y + a.h
    bx2, by2 = b.x + b.w, b.y + b.h
    ix1 = max(a.x, b.x); iy1 = max(a.y, b.y)
    ix2 = min(ax2, bx2); iy2 = min(ay2, by2)
    iw = max(0.0, ix2 - ix1); ih = max(0.0, iy2 - iy1)
    inter = iw * ih
    union = a.w * a.h + b.w * b.h - inter
    if union <= 0:
        return 0.0
    return inter / union


@dataclass
class DetectorWorker:
    """Drive ``make_detector()`` in a thread; exposes latest results."""
    detector: object
    max_fps: float = MAX_FPS_DEFAULT
    on_result: Callable[[list[DetectorResult]], None] | None = None
    _stop: threading.Event = field(default_factory=threading.Event)
    _thread: threading.Thread | None = None
    _latest: list[DetectorResult] = field(default_factory=list)
    _frame: tuple[bytes, int, int] | None = None
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="detect_yolo", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def submit_frame(self, frame_bytes: bytes, width: int, height: int) -> None:
        with self._lock:
            self._frame = (frame_bytes, width, height)

    def latest(self) -> list[DetectorResult]:
        with self._lock:
            return list(self._latest)

    def _loop(self) -> None:
        period = 1.0 / max(0.1, self.max_fps)
        while not self._stop.is_set():
            with self._lock:
                pending = self._frame
                self._frame = None
            if pending is None:
                time.sleep(period)
                continue
            frame_bytes, width, height = pending
            try:
                results = self.detector.detect(frame_bytes, width, height)  # type: ignore[attr-defined]
            except Exception:                     # pragma: no cover
                LOG.exception("detect_yolo: backend raised")
                results = []
            with self._lock:
                self._latest = results
            if self.on_result is not None:
                try:
                    self.on_result(results)
                except Exception:                 # pragma: no cover
                    LOG.exception("detect_yolo: on_result callback raised")
            time.sleep(period)
