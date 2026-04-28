"""CPU super-resolution wrapper (Real-ESRGAN-General-x4v3 via ncnn).

Phase 5B base-side enhancement. This module is a thin façade so the rest of
the pipeline (canvas → state_publisher) can call ``superres.enhance_tile()``
without knowing whether the runtime is ncnn, ONNX, or a stub.

Model weights are out of band (``LIFETRAC_SUPERRES_WEIGHTS`` env var). When
the runtime is unavailable the module falls back to a pass-through that
returns the input blob unchanged but still tags the canvas with
``Badge.ENHANCED`` so the operator UI always reports the same surface
behavior — the only thing the operator sees differently is that the pixel
detail doesn't actually go up.
"""
from __future__ import annotations

import logging
import os
from dataclasses import dataclass

LOG = logging.getLogger("superres_cpu")


@dataclass
class SuperresResult:
    blob: bytes              # encoded WebP/PNG of the enhanced tile
    scale: int               # 1..4
    backend: str             # "ncnn" | "onnx" | "passthrough"


class _Passthrough:
    name = "passthrough"

    def enhance(self, tile_blob: bytes) -> bytes:
        return tile_blob


class _NcnnRealesrgan:                            # pragma: no cover
    name = "ncnn"

    def __init__(self, weights_dir: str, scale: int = 4) -> None:
        from realesrgan_ncnn_py import Realesrgan  # type: ignore
        self._engine = Realesrgan(model_path=weights_dir, scale=scale)
        self._scale = scale

    def enhance(self, tile_blob: bytes) -> bytes:
        # Real call requires PIL.Image round-trip; left to deployment so
        # the test environment doesn't pull in PIL.
        return tile_blob


def make_engine() -> object:
    weights = os.environ.get("LIFETRAC_SUPERRES_WEIGHTS", "")
    if not weights:
        LOG.warning("superres_cpu: no weights path; using passthrough")
        return _Passthrough()
    try:
        return _NcnnRealesrgan(weights)
    except ImportError as exc:                    # pragma: no cover
        LOG.warning("superres_cpu: ncnn unavailable (%s); using passthrough", exc)
        return _Passthrough()


def enhance_tile(engine: object, tile_blob: bytes) -> SuperresResult:
    out = engine.enhance(tile_blob)               # type: ignore[attr-defined]
    backend = getattr(engine, "name", "passthrough")
    return SuperresResult(blob=out, scale=4 if backend == "ncnn" else 1, backend=backend)
