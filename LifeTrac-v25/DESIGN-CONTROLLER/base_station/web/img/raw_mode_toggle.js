// raw_mode_toggle.js — operator one-click "show me the raw bytes only" toggle.
//
// Sets a body class `raw-mode` which CSS uses to hide all overlay canvases
// (#image-fade, #image-staleness, #image-badges, #image-detections). The
// underlying #image-canvas keeps painting as normal, so the operator sees
// only the most-recent received pixels with no enhancement.
//
// Persists the choice in localStorage and pings /api/audit/view_mode so the
// black-box logger knows which mode the operator was in when commands were
// issued (per IMAGE_PIPELINE.md §5C audit log requirement).

(function () {
  'use strict';

  const KEY = 'lifetrac.raw_mode';

  function init() {
    let btn = document.getElementById('raw-mode-toggle');
    if (!btn) {
      btn = document.createElement('button');
      btn.id = 'raw-mode-toggle';
      btn.type = 'button';
      btn.style.position = 'fixed';
      btn.style.right = '8px';
      btn.style.bottom = '40px';
      btn.style.padding = '6px 10px';
      btn.style.borderRadius = '6px';
      btn.style.border = '1px solid #444';
      btn.style.background = '#222';
      btn.style.color = '#eee';
      btn.style.fontFamily = 'monospace';
      btn.style.zIndex = '999';
      document.body.appendChild(btn);
    }

    function paint(rawOn) {
      btn.textContent = rawOn ? 'RAW MODE: ON' : 'RAW MODE: off';
      btn.style.background = rawOn ? '#a00' : '#222';
      document.body.classList.toggle('raw-mode', rawOn);
    }

    function logChoice(rawOn) {
      try {
        fetch('/api/audit/view_mode', {
          method: 'POST',
          headers: { 'content-type': 'application/json' },
          body: JSON.stringify({ raw_mode: rawOn, ts_ms: Date.now() }),
          credentials: 'same-origin',
        });
      } catch (_) { /* audit endpoint optional in tests */ }
    }

    let on = false;
    try { on = localStorage.getItem(KEY) === '1'; } catch (_) {}
    paint(on);

    btn.addEventListener('click', () => {
      on = !on;
      try { localStorage.setItem(KEY, on ? '1' : '0'); } catch (_) {}
      paint(on);
      logChoice(on);
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }
})();
