// staleness_overlay.js — yellow tint + age-in-seconds on stale tiles.
//
// Reads `age_ms` per tile *from the server* (state_publisher.snapshot()).
// Never computes age locally — that would let a clock-skewed browser hide
// real loss.
//
// Tints any tile whose age >= STALE_MS with a yellow alpha overlay scaled
// by how stale it is, and writes the age in seconds in the corner.

(function () {
  'use strict';

  const STALE_MS = 1000;
  const VERY_STALE_MS = 5000;

  function init() {
    const base = document.getElementById('image-canvas');
    if (!base) return;
    let overlay = document.getElementById('image-staleness');
    if (!overlay) {
      overlay = document.createElement('canvas');
      overlay.id = 'image-staleness';
      overlay.style.position = 'absolute';
      overlay.style.pointerEvents = 'none';
      overlay.style.left = base.offsetLeft + 'px';
      overlay.style.top = base.offsetTop + 'px';
      base.parentNode.appendChild(overlay);
    }
    overlay.width = base.width;
    overlay.height = base.height;
    const ctx = overlay.getContext('2d');

    window.addEventListener('lifetrac-state', (ev) => {
      const snap = ev.detail;
      if (!snap) return;
      const tilePx = snap.grid.tile_px;
      if (overlay.width !== base.width || overlay.height !== base.height) {
        overlay.width = base.width;
        overlay.height = base.height;
      }
      ctx.clearRect(0, 0, overlay.width, overlay.height);
      ctx.font = `${Math.max(10, tilePx / 3)}px monospace`;
      ctx.textBaseline = 'top';
      for (const t of snap.tiles) {
        if (t.age_ms < STALE_MS) continue;
        const ratio = Math.min(1, (t.age_ms - STALE_MS) / (VERY_STALE_MS - STALE_MS));
        const alpha = 0.15 + ratio * 0.45;
        ctx.fillStyle = `rgba(255, 215, 0, ${alpha.toFixed(3)})`;
        ctx.fillRect(t.tx * tilePx, t.ty * tilePx, tilePx, tilePx);
        ctx.fillStyle = 'rgba(0,0,0,0.85)';
        ctx.fillText((t.age_ms / 1000).toFixed(1) + 's',
                     t.tx * tilePx + 1, t.ty * tilePx + 1);
      }
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
