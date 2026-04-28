// accel_status.js — always-visible AI accelerator status pill.
//
// Reads `accel_status` from state_publisher snapshots:
//   "online"   — Coral detected and healthy
//   "offline"  — no Coral; CPU-only path
//   "degraded" — Coral present but failing health checks (>3 consecutive
//                inference timeouts per accel_select.py)
//
// Per IMAGE_PIPELINE.md V5 the operator must see this transition within
// 10 s; the WebSocket cadence (~1 Hz) covers that comfortably.

(function () {
  'use strict';

  const COLOURS = {
    online:   { bg: '#093', fg: '#fff' },
    offline:  { bg: '#666', fg: '#fff' },
    degraded: { bg: '#c50', fg: '#fff' },
    unknown:  { bg: '#333', fg: '#fff' },
  };

  function init() {
    let pill = document.getElementById('accel-status');
    if (!pill) {
      pill = document.createElement('div');
      pill.id = 'accel-status';
      pill.style.position = 'fixed';
      pill.style.right = '8px';
      pill.style.bottom = '8px';
      pill.style.padding = '4px 8px';
      pill.style.borderRadius = '12px';
      pill.style.fontFamily = 'monospace';
      pill.style.fontSize = '12px';
      pill.style.zIndex = '999';
      document.body.appendChild(pill);
    }

    function paint(status) {
      const palette = COLOURS[status] || COLOURS.unknown;
      pill.style.background = palette.bg;
      pill.style.color = palette.fg;
      pill.textContent = `AI accel: ${status}`;
    }

    paint('unknown');
    window.addEventListener('lifetrac-state', (ev) => {
      const snap = ev.detail;
      if (!snap) return;
      paint(snap.accel_status || 'unknown');
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
