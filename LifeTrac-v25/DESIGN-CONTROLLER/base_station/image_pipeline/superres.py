"""superres.py — super-resolution dispatcher (Coral-aware).

Phase-1 scaffold. Real impls land per IMAGE_PIPELINE.md §5.1 Phase 2:
  * `superres_cpu.py` — Real-ESRGAN-General-x4v3 via ncnn on the X8 A53s.
  * `superres_coral.py` — Edge-TPU port (gated on the Phase-0 Coral spike).

This file is the public entry point that picks at call time. Per
DECISIONS.md D-CORAL-1 the choice is per-call, not per-startup, so the
operator toggle and hot-unplug both take effect within one frame.
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any

from .accel_select import get_accelerator

LOG = logging.getLogger("superres")

_audit_lock = threading.Lock()
_last_audit: dict[str, float] = {"coral": 0.0, "cpu": 0.0}
_AUDIT_INTERVAL_S = 1.0


def _audit(path: str, audit_writer: Any | None) -> None:
    """Debounced audit-log of which path ran (one entry per second per path)."""
    now = time.monotonic()
    with _audit_lock:
        if now - _last_audit.get(path, 0.0) < _AUDIT_INTERVAL_S:
            return
        _last_audit[path] = now
    if audit_writer is not None:
        try:
            audit_writer.record("accel_path", subsystem="superres", path=path)
        except Exception:  # pragma: no cover
            LOG.exception("superres: audit write failed")


def enhance(frame: Any, *, audit_writer: Any | None = None) -> Any:
    """Return a super-resolved frame, picking Coral vs CPU per call.

    Phase-1 stubs return the input unchanged; the dispatch shape is what
    matters here so consumers can call `superres.enhance(frame)` today and
    benefit automatically when the real impls land.
    """
    if get_accelerator().is_active():
        _audit("coral", audit_writer)
        return _coral_enhance(frame)
    _audit("cpu", audit_writer)
    return _cpu_enhance(frame)


def _coral_enhance(frame: Any) -> Any:
    # TODO: wire to superres_coral.py once Phase-0 spike clears.
    return frame


def _cpu_enhance(frame: Any) -> Any:
    # TODO: wire to superres_cpu.py (Real-ESRGAN ncnn) per IMAGE_PIPELINE Phase 2.
    return frame
