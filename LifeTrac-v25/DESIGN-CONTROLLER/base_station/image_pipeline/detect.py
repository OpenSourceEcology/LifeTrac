"""detect.py — base-side safety detector dispatcher (Coral-aware).

Phase-1 scaffold. Real impls land per IMAGE_PIPELINE.md §5.1 Phase 2:
  * CPU = YOLOv8-nano OR NanoDet-Plus (gated on AGPL stance §10 O1).
  * Coral = quantized YOLOv8-nano on Edge TPU.

This is the **base-side independent safety cross-check** required by
MASTER_PLAN.md R6 — it must produce results regardless of accelerator
state, only the *speed* changes.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

from .accel_select import get_accelerator

LOG = logging.getLogger("detect")

_audit_lock = threading.Lock()
_last_audit: dict[str, float] = {"coral": 0.0, "cpu": 0.0}
_AUDIT_INTERVAL_S = 1.0


def _audit(path: str, audit_writer: Any | None) -> None:
    now = time.monotonic()
    with _audit_lock:
        if now - _last_audit.get(path, 0.0) < _AUDIT_INTERVAL_S:
            return
        _last_audit[path] = now
    if audit_writer is not None:
        try:
            audit_writer.record("accel_path", subsystem="detect", path=path)
        except Exception:  # pragma: no cover
            LOG.exception("detect: audit write failed")


def detect(frame: Any, *, audit_writer: Any | None = None) -> list[dict]:
    """Return a list of detection dicts (`{class, conf, bbox}`).

    Phase-1 stubs return [] either way; dispatch shape is what counts.
    """
    if get_accelerator().is_active():
        _audit("coral", audit_writer)
        return _coral_detect(frame)
    _audit("cpu", audit_writer)
    return _cpu_detect(frame)


def _coral_detect(frame: Any) -> list[dict]:
    return []


def _cpu_detect(frame: Any) -> list[dict]:
    return []
