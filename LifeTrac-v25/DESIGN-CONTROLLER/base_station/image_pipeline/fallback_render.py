"""Server-side 1 fps composite render of the canvas.

Used as the HDMI console + headless QA surface (IMAGE_PIPELINE.md Phase 5A).
Produces a single PNG/WebP framebuffer combining all current tiles, plus a
small HUD strip showing the encode mode, accel status, and "needs keyframe"
flag.

Falls back to a text-only digest if PIL is missing so the module is always
importable inside the unit test container.
"""
from __future__ import annotations

import io
from dataclasses import dataclass
from typing import Optional

from .canvas import Canvas
from .state_publisher import StatePublisher

try:                                     # pragma: no cover - import guard
    from PIL import Image, ImageDraw     # type: ignore
    _HAVE_PIL = True
except Exception:                        # pragma: no cover
    Image = None                          # type: ignore
    ImageDraw = None                      # type: ignore
    _HAVE_PIL = False


@dataclass
class FallbackRenderResult:
    """What :meth:`FallbackRenderer.render` produces."""
    image_bytes: bytes
    mime: str
    width: int
    height: int


class FallbackRenderer:
    """Composite the current canvas + a HUD strip into a single image."""

    HUD_HEIGHT = 16

    def __init__(self, canvas: Canvas, publisher: StatePublisher) -> None:
        self.canvas = canvas
        self.publisher = publisher

    def render(self) -> FallbackRenderResult:
        if not _HAVE_PIL:
            return self._text_digest()
        composite = Image.new("RGB", (self.canvas.grid_w * self.canvas.tile_px,
                                      self.canvas.grid_h * self.canvas.tile_px + self.HUD_HEIGHT),
                              (0, 0, 0))
        for idx, slot in self.canvas.snapshot():
            try:
                tile_img = Image.open(io.BytesIO(slot.blob))
                tile_img.load()
            except Exception:
                continue
            tx = idx % self.canvas.grid_w
            ty = idx // self.canvas.grid_w
            composite.paste(tile_img, (tx * self.canvas.tile_px, ty * self.canvas.tile_px))
        draw = ImageDraw.Draw(composite)
        hud = (f"mode={self.publisher.encode_mode} accel={self.publisher.accel_status}"
               f" kf={'1' if self.publisher.needs_keyframe else '0'}")
        draw.text((2, composite.height - self.HUD_HEIGHT + 2), hud, fill=(255, 255, 255))
        buf = io.BytesIO()
        composite.save(buf, format="WEBP", quality=70)
        return FallbackRenderResult(
            image_bytes=buf.getvalue(),
            mime="image/webp",
            width=composite.width,
            height=composite.height,
        )

    def _text_digest(self) -> FallbackRenderResult:
        # No PIL available — used in unit tests / minimal containers.
        snap = self.publisher.snapshot()
        body = (f"canvas {snap['grid']['w']}x{snap['grid']['h']}@{snap['grid']['tile_px']}px "
                f"tiles_populated={len(snap['tiles'])} mode={snap['encode_mode']} "
                f"accel={snap['accel_status']} kf={snap['needs_keyframe']}").encode()
        return FallbackRenderResult(
            image_bytes=body,
            mime="text/plain",
            width=self.canvas.grid_w * self.canvas.tile_px,
            height=self.canvas.grid_h * self.canvas.tile_px,
        )
