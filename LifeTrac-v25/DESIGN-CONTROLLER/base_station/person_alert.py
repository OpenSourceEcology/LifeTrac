"""CMD_PERSON_APPEARED (opcode 0x60) emitter.

When the on-tractor NanoDet (or the base-side YOLO/NanoDet) reports a
high-confidence ``person`` / ``animal`` / ``vehicle`` detection, the safety
loop emits a P0 alert frame so the bridge / operator UI / autonomy stack
can:

  - immediately bring the next image frame to P1 (priority promotion),
  - paint a banner on the operator console,
  - log the event to the audit JSONL,
  - and let the tractor's local autonomy yield control.

Wire format (CMD_PERSON_APPEARED args, 6 bytes):

    u8  cls_id           ; 0=person, 1=animal, 2=vehicle, 0xFF=other
    u8  conf_x100        ; clamp((conf * 100), 0, 100)
    i8  cx_q7            ; centroid X, signed Q1.7 around image center [-128..127]
    i8  cy_q7            ; centroid Y, signed Q1.7 around image center
    u8  age_ms_div10     ; how stale the detection was when emitted (10 ms ticks, 0..255)
    u8  reserved         ; 0

The cls_id table mirrors :mod:`firmware.tractor_x8.image_pipeline.detect_nanodet`
so the base side can decode tractor-emitted alerts without translation.

Public API
----------

- :data:`CLS_PERSON` / :data:`CLS_ANIMAL` / :data:`CLS_VEHICLE` / :data:`CLS_OTHER`
- :func:`pack_person_alert_args(cls_id, conf, cx, cy, age_ms=0)` — bytes
- :func:`build_person_alert_frame(seq, cls_id, conf, cx, cy, *, age_ms=0,
  source_id=SRC_BASE)` — wraps with :func:`pack_command`
- :class:`PersonAlertEmitter` — debouncer + audit hook + publish callback
  for use in :class:`base_station.image_pipeline.detect_yolo.DetectorWorker`
  or the tractor-side ``camera_service.py``.
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Optional

from lora_proto import (   # type: ignore  # noqa: E402
    CMD_PERSON_APPEARED,
    SRC_BASE,
    pack_command,
)

CLS_PERSON = 0
CLS_ANIMAL = 1
CLS_VEHICLE = 2
CLS_OTHER = 0xFF

_NAME_TO_CLS = {
    "person": CLS_PERSON,
    "animal": CLS_ANIMAL,
    "vehicle": CLS_VEHICLE,
}

# Per IMAGE_PIPELINE.md V8: only act on detections with conf ≥ 0.70.
DEFAULT_MIN_CONFIDENCE = 0.70

# Don't spam the air: collapse repeated detections of the same class within
# this window into one P0 emission. Tracks the last emit timestamp per class.
DEFAULT_DEBOUNCE_S = 1.5


def cls_id_for(name: str) -> int:
    """Map a textual class label to the wire ``cls_id`` byte."""
    return _NAME_TO_CLS.get(name.lower(), CLS_OTHER)


def _clamp_q7(v: float) -> int:
    """Map normalised [0..1] coord into signed Q1.7 around image center."""
    if v != v:                       # NaN guard
        return 0
    centered = (v - 0.5) * 2.0       # → [-1..1]
    scaled = max(-1.0, min(1.0, centered)) * 127.0
    return max(-128, min(127, int(round(scaled))))


def pack_person_alert_args(cls_id: int, conf: float,
                           cx: float, cy: float,
                           age_ms: int = 0) -> bytes:
    """Pack the 6-byte argument payload for CMD_PERSON_APPEARED."""
    if not (0 <= cls_id <= 0xFF):
        raise ValueError(f"cls_id {cls_id} out of range")
    conf_x100 = max(0, min(100, int(round(conf * 100.0))))
    age_div10 = max(0, min(255, int(age_ms // 10)))
    return struct.pack("<BBbbBB",
                       cls_id & 0xFF,
                       conf_x100,
                       _clamp_q7(cx),
                       _clamp_q7(cy),
                       age_div10,
                       0)


def build_person_alert_frame(seq: int, cls_id: int, conf: float,
                             cx: float, cy: float,
                             *,
                             age_ms: int = 0,
                             source_id: int = SRC_BASE) -> bytes:
    """Build the full ``CommandFrame`` cleartext for CMD_PERSON_APPEARED."""
    args = pack_person_alert_args(cls_id, conf, cx, cy, age_ms=age_ms)
    return pack_command(seq, CMD_PERSON_APPEARED, args, source_id=source_id)


# ---------------------------------------------------------------------------
# Emitter
# ---------------------------------------------------------------------------

@dataclass
class _DetectionLike:
    """Minimal duck-type view of a detector result.

    Accepts both :class:`detect_yolo.DetectorResult` (base side) and
    :class:`detect_nanodet.TractorDetection` (tractor side) without making
    this module import either one.
    """
    cls: str
    conf: float
    x: float          # 0..1
    y: float
    w: float
    h: float


def _to_detection_like(obj: object) -> Optional[_DetectionLike]:
    cls = getattr(obj, "cls", None)
    conf = getattr(obj, "conf", None)
    x = getattr(obj, "x", None); y = getattr(obj, "y", None)
    w = getattr(obj, "w", None); h = getattr(obj, "h", None)
    if cls is None or conf is None or None in (x, y, w, h):
        return None
    return _DetectionLike(cls=str(cls), conf=float(conf),
                          x=float(x), y=float(y),
                          w=float(w), h=float(h))


class PersonAlertEmitter:
    """Convert detector results into CMD_PERSON_APPEARED frames.

    Wire it into a :class:`DetectorWorker` by passing
    ``on_result=emitter.feed`` at construction time::

        emitter = PersonAlertEmitter(
            seq_provider=lambda: bridge.next_command_seq(),
            publish=lambda frame: bridge.tx_command_p0(frame),
            audit=audit_log,
            source="base",
        )
        worker = DetectorWorker(detector=det, on_result=emitter.feed)

    The emitter:
      - filters by ``min_confidence`` and the ``cls_id_for`` map,
      - debounces repeats of the same class within ``debounce_s``,
      - calls ``publish(frame_bytes)`` for the P0 TX queue,
      - calls ``audit.log_person_appeared(...)`` if an AuditLog was passed.
    """

    def __init__(self, *,
                 seq_provider: Callable[[], int],
                 publish: Callable[[bytes], None],
                 audit: Optional[object] = None,
                 source: str = "base",
                 source_id: int = SRC_BASE,
                 min_confidence: float = DEFAULT_MIN_CONFIDENCE,
                 debounce_s: float = DEFAULT_DEBOUNCE_S,
                 monotonic: Callable[[], float] = time.monotonic) -> None:
        self._seq_provider = seq_provider
        self._publish = publish
        self._audit = audit
        self._source = source
        self._source_id = source_id
        self._min_confidence = float(min_confidence)
        self._debounce_s = float(debounce_s)
        self._monotonic = monotonic
        self._last_emit: dict[int, float] = {}
        # Stats surfaced on the diagnostics page.
        self.emitted = 0
        self.suppressed_low_conf = 0
        self.suppressed_debounce = 0

    def feed(self, results: Iterable[object], *,
             age_ms: int = 0) -> list[bytes]:
        """Process a batch of detector results; return the frames emitted.

        Returns the (possibly empty) list of full ``CommandFrame`` cleartexts
        for callers that want to inspect what went out (tests, replay).
        """
        emitted: list[bytes] = []
        now = self._monotonic()
        for raw in results:
            det = _to_detection_like(raw)
            if det is None:
                continue
            if det.conf < self._min_confidence:
                self.suppressed_low_conf += 1
                continue
            cls_id = cls_id_for(det.cls)
            if cls_id == CLS_OTHER:
                # Only safety-relevant classes are P0; ignore the rest.
                continue
            last = self._last_emit.get(cls_id, -1e9)
            if (now - last) < self._debounce_s:
                self.suppressed_debounce += 1
                continue
            cx = det.x + det.w / 2.0
            cy = det.y + det.h / 2.0
            seq = int(self._seq_provider()) & 0xFFFF
            frame = build_person_alert_frame(
                seq, cls_id, det.conf, cx, cy,
                age_ms=age_ms, source_id=self._source_id)
            try:
                self._publish(frame)
            except Exception:
                # Surface in stats but never crash the detector loop.
                continue
            self._last_emit[cls_id] = now
            self.emitted += 1
            emitted.append(frame)
            if self._audit is not None:
                try:
                    self._audit.log_person_appeared(
                        source=self._source,
                        cls=det.cls,
                        confidence=det.conf,
                        cx=cx, cy=cy)
                except Exception:
                    pass
        return emitted


__all__ = [
    "CLS_PERSON", "CLS_ANIMAL", "CLS_VEHICLE", "CLS_OTHER",
    "cls_id_for",
    "pack_person_alert_args",
    "build_person_alert_frame",
    "PersonAlertEmitter",
]
