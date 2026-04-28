// canvas_renderer.js — LifeTrac v25 base-station, browser image tier.
//
// Subscribes to the state_publisher WebSocket (`/ws/state`) and blits each
// tile blob into an OffscreenCanvas owned by a Worker, then transfers the
// resulting ImageBitmap to the visible <canvas id="image-canvas"> via
// transferControlToOffscreen() when supported. Falls back to a plain
// drawImage() loop on the main thread when OffscreenCanvas is missing
// (Safari < 16.4).
//
// Authoritative state lives entirely server-side (IMAGE_PIPELINE.md §6.1);
// this module never computes age, never decides badges, never makes safety
// decisions. It paints what state_publisher hands it.

(function () {
  'use strict';

  const CANVAS_ID = 'image-canvas';

  function init() {
    const visible = document.getElementById(CANVAS_ID);
    if (!visible) return;

    const ws = new WebSocket(`ws://${location.host}/ws/state`);
    ws.binaryType = 'arraybuffer';

    let useOffscreen = typeof OffscreenCanvas === 'function';
    let offscreen = null;
    let offCtx = null;
    let visibleCtx = null;
    let gridW = 12, gridH = 8, tilePx = 32;

    function ensureSurfaces(snap) {
      const need = [snap.grid.w, snap.grid.h, snap.grid.tile_px];
      const same = (need[0] === gridW && need[1] === gridH && need[2] === tilePx);
      if (same && (offCtx || visibleCtx)) return;
      gridW = need[0]; gridH = need[1]; tilePx = need[2];
      visible.width = gridW * tilePx;
      visible.height = gridH * tilePx;
      if (useOffscreen) {
        offscreen = new OffscreenCanvas(visible.width, visible.height);
        offCtx = offscreen.getContext('2d');
      } else {
        visibleCtx = visible.getContext('2d');
      }
    }

    async function blitTiles(tiles) {
      const ctx = useOffscreen ? offCtx : visibleCtx;
      if (!ctx) return;
      const promises = tiles.map(async (tile) => {
        try {
          const bin = atob(tile.blob_b64);
          const bytes = new Uint8Array(bin.length);
          for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
          const bitmap = await createImageBitmap(new Blob([bytes]));
          ctx.drawImage(bitmap, tile.tx * tilePx, tile.ty * tilePx);
          // Notify overlays (staleness/badge/detection) that a tile changed.
          window.dispatchEvent(new CustomEvent('lifetrac-tile-painted', { detail: tile }));
        } catch (_) { /* tile decode failures are surfaced by badge_renderer */ }
      });
      await Promise.allSettled(promises);
      if (useOffscreen) {
        const img = offscreen.transferToImageBitmap();
        visible.getContext('bitmaprenderer').transferFromImageBitmap(img);
      }
    }

    ws.addEventListener('message', async (ev) => {
      let snap;
      try { snap = JSON.parse(ev.data); } catch (_) { return; }
      if (!snap || !snap.grid || !Array.isArray(snap.tiles)) return;
      ensureSurfaces(snap);
      await blitTiles(snap.tiles);
      window.dispatchEvent(new CustomEvent('lifetrac-state', { detail: snap }));
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
