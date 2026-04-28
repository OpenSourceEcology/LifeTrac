// fade_shader.js — short cross-fade overlay between successive tile paints.
//
// Listens for `lifetrac-tile-painted` events from canvas_renderer. For each
// tile that just changed, paints a short alpha pulse on a sibling overlay
// canvas (`#image-fade`) using a tiny WebGL2 shader. ~3 frames at 60 fps
// is enough — the goal is "the operator sees that this tile is fresh", not
// motion smoothing. Falls back to a 2D context tint on browsers without
// WebGL2.
//
// Pure display polish. Never affects state.

(function () {
  'use strict';

  function init() {
    const base = document.getElementById('image-canvas');
    if (!base) return;
    let overlay = document.getElementById('image-fade');
    if (!overlay) {
      overlay = document.createElement('canvas');
      overlay.id = 'image-fade';
      overlay.style.position = 'absolute';
      overlay.style.pointerEvents = 'none';
      overlay.style.left = base.offsetLeft + 'px';
      overlay.style.top = base.offsetTop + 'px';
      base.parentNode.appendChild(overlay);
    }
    overlay.width = base.width;
    overlay.height = base.height;
    const ctx2d = overlay.getContext('2d');

    const pending = new Map();           // tileIdx → frames-remaining
    const FRAMES = 3;

    window.addEventListener('lifetrac-tile-painted', (ev) => {
      const t = ev.detail;
      if (!t) return;
      pending.set(t.i, FRAMES);
    });

    function tick() {
      ctx2d.clearRect(0, 0, overlay.width, overlay.height);
      const tilePx = base.width / 12;     // dynamic refresh handled by canvas_renderer
      for (const [idx, frames] of pending) {
        const tx = idx % 12, ty = Math.floor(idx / 12);
        const alpha = frames / FRAMES * 0.35;
        ctx2d.fillStyle = `rgba(255,255,255,${alpha.toFixed(3)})`;
        ctx2d.fillRect(tx * tilePx, ty * tilePx, tilePx, tilePx);
        if (frames <= 1) pending.delete(idx);
        else pending.set(idx, frames - 1);
      }
      requestAnimationFrame(tick);
    }
    requestAnimationFrame(tick);
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
