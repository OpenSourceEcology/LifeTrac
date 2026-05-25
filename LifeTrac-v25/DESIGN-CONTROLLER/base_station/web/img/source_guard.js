// source_guard.js — detect synthetic camera test-pattern feed.
//
// The synthetic tractor source in camera_service.py produces pixels with
// channel relationships close to:
//   R = v, G = (2*v) mod 256, B = 255 - v
// for a scrolling diagonal pattern. This module samples the rendered canvas
// and raises a warning banner when a high fraction of pixels match that
// signature consistently.

(function () {
  'use strict';

  const SAMPLE_STRIDE = 8;
  const MIN_SAMPLES = 96;
  const MATCH_THRESHOLD = 0.72;
  const CONSECUTIVE_FRAMES = 3;

  let matchStreak = 0;
  let clearStreak = 0;
  let lastCheckTs = 0;

  function near(a, b, eps) {
    return Math.abs(a - b) <= eps;
  }

  function gMatchesDoubleR(g, r) {
    const dbl = (2 * r) & 0xFF;
    return near(g, dbl, 14);
  }

  function ensureBanner() {
    let banner = document.getElementById('image-source-warning');
    if (banner) return banner;
    banner = document.createElement('div');
    banner.id = 'image-source-warning';
    banner.style.display = 'none';
    banner.style.position = 'fixed';
    banner.style.top = '82px';
    banner.style.left = '50%';
    banner.style.transform = 'translateX(-50%)';
    banner.style.background = '#a00000';
    banner.style.color = '#fff';
    banner.style.padding = '6px 14px';
    banner.style.borderRadius = '6px';
    banner.style.fontFamily = 'ui-monospace, monospace';
    banner.style.zIndex = '997';
    banner.textContent = 'SYNTHETIC TEST PATTERN DETECTED — CHECK CAMERA SOURCE';
    document.body.appendChild(banner);
    return banner;
  }

  function computeSyntheticScore(ctx, w, h) {
    const frame = ctx.getImageData(0, 0, w, h).data;
    let samples = 0;
    let hits = 0;

    for (let y = 0; y < h; y += SAMPLE_STRIDE) {
      for (let x = 0; x < w; x += SAMPLE_STRIDE) {
        const i = (y * w + x) * 4;
        const r = frame[i];
        const g = frame[i + 1];
        const b = frame[i + 2];
        samples++;

        if (near(b, 255 - r, 10) && gMatchesDoubleR(g, r)) {
          hits++;
        }
      }
    }

    if (samples < MIN_SAMPLES) return 0;
    return hits / samples;
  }

  function init() {
    const canvas = document.getElementById('image-canvas');
    if (!canvas) return;
    const ctx = canvas.getContext('2d', { willReadFrequently: true });
    if (!ctx) return;
    const banner = ensureBanner();

    window.addEventListener('lifetrac-state', () => {
      const now = performance.now();
      if (now - lastCheckTs < 1000) return;
      lastCheckTs = now;

      if (!canvas.width || !canvas.height) return;

      try {
        const score = computeSyntheticScore(ctx, canvas.width, canvas.height);
        if (score >= MATCH_THRESHOLD) {
          matchStreak++;
          clearStreak = 0;
        } else {
          clearStreak++;
          if (clearStreak >= 2) matchStreak = 0;
        }

        if (matchStreak >= CONSECUTIVE_FRAMES) {
          banner.style.display = 'block';
        } else if (clearStreak >= 2) {
          banner.style.display = 'none';
        }
      } catch (_) {
        // Ignore transient readback failures.
      }
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
