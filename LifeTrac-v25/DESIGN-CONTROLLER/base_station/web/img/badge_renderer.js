// badge_renderer.js — fail-closed badge enforcement.
//
// Per IMAGE_PIPELINE.md §6 / §6.1: every tile must carry a Badge enum from
// the state publisher. If a tile arrives with a missing or malformed badge
// the browser MUST refuse to display it and log the refusal back to the
// base /api/health endpoint. This module enforces that contract.
//
// Badge enum (mirrors lora_proto.Badge):
//   0 RAW            — pixels from the wire, no enhancement
//   1 CACHED         — bg_cache held-over
//   2 ENHANCED       — base-side super-res
//   3 RECOLOURISED   — Y-only luma + 30s colour reference fused
//   4 PREDICTED      — motion-vector replay
//   5 SYNTHETIC      — LaMa-inpainted (Coral-only, off in v25)
//   6 WIREFRAME      — PiDiNet edge overlay only

(function () {
  'use strict';

  const VALID = new Set([0, 1, 2, 3, 4, 5, 6]);
  const LABEL = ['RAW', 'CACHED', 'ENHANCED', 'RECOLOURISED', 'PREDICTED', 'SYNTHETIC', 'WIREFRAME'];
  const TINT = [
    null,                          // RAW: no overlay
    'rgba(120, 120, 120, 0.18)',   // CACHED — grey
    'rgba(0, 200, 255, 0.10)',     // ENHANCED — cyan
    'rgba(255, 100, 200, 0.18)',   // RECOLOURISED — magenta
    'rgba(255, 165, 0,  0.20)',    // PREDICTED — orange
    'rgba(200, 0, 0,    0.30)',    // SYNTHETIC — red (visible operator warning)
    'rgba(0, 200, 100,  0.20)',    // WIREFRAME — green
  ];

  function init() {
    const base = document.getElementById('image-canvas');
    if (!base) return;
    let overlay = document.getElementById('image-badges');
    if (!overlay) {
      overlay = document.createElement('canvas');
      overlay.id = 'image-badges';
      overlay.style.position = 'absolute';
      overlay.style.pointerEvents = 'none';
      overlay.style.left = base.offsetLeft + 'px';
      overlay.style.top = base.offsetTop + 'px';
      base.parentNode.appendChild(overlay);
    }
    overlay.width = base.width;
    overlay.height = base.height;
    const ctx = overlay.getContext('2d');

    function reportRefusal(reason, tileIdx) {
      try {
        fetch('/api/health/refusal', {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: JSON.stringify({ reason: reason, tile_index: tileIdx, ts_ms: Date.now() }),
          credentials: 'same-origin',
        });
      } catch (_) { /* health endpoint optional in tests */ }
    }

    window.addEventListener('lifetrac-state', (ev) => {
      const snap = ev.detail;
      if (!snap) return;
      const tilePx = snap.grid.tile_px;
      if (overlay.width !== base.width || overlay.height !== base.height) {
        overlay.width = base.width;
        overlay.height = base.height;
      }
      ctx.clearRect(0, 0, overlay.width, overlay.height);
      ctx.font = `${Math.max(8, tilePx / 4)}px monospace`;
      ctx.textBaseline = 'bottom';
      for (const t of snap.tiles) {
        if (!VALID.has(t.badge)) {
          reportRefusal('invalid_badge', t.i);
          // Black-out the offending tile so the operator never sees pixels
          // from a tile we can't classify.
          ctx.fillStyle = 'rgba(0, 0, 0, 1.0)';
          ctx.fillRect(t.tx * tilePx, t.ty * tilePx, tilePx, tilePx);
          ctx.fillStyle = 'rgba(255, 0, 0, 1.0)';
          ctx.fillText('BADGE?', t.tx * tilePx + 2, (t.ty + 1) * tilePx - 2);
          continue;
        }
        const tint = TINT[t.badge];
        if (tint) {
          ctx.fillStyle = tint;
          ctx.fillRect(t.tx * tilePx, t.ty * tilePx, tilePx, tilePx);
          ctx.fillStyle = 'rgba(0,0,0,0.85)';
          ctx.fillText(LABEL[t.badge].slice(0, 4), t.tx * tilePx + 2, (t.ty + 1) * tilePx - 2);
        }
      }
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
