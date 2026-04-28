// detection_overlay.js — bounding-box rendering from state_publisher.
//
// Detections are produced server-side by `image_pipeline/detect_yolo.py` and
// the tractor-side `detect_nanodet.py`; the cross-detector verdict comes
// from `state_publisher.safety_verdict`. Browser only renders.

(function () {
  'use strict';

  const COLOUR_BY_CLASS = {
    person: '#ff3333',
    vehicle: '#ffaa00',
    animal: '#cc66ff',
  };

  function init() {
    const base = document.getElementById('image-canvas');
    if (!base) return;
    let overlay = document.getElementById('image-detections');
    if (!overlay) {
      overlay = document.createElement('canvas');
      overlay.id = 'image-detections';
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
      if (overlay.width !== base.width || overlay.height !== base.height) {
        overlay.width = base.width;
        overlay.height = base.height;
      }
      ctx.clearRect(0, 0, overlay.width, overlay.height);
      ctx.lineWidth = 2;
      ctx.font = '10px monospace';
      ctx.textBaseline = 'top';
      const dets = snap.detections || [];
      for (const d of dets) {
        const colour = COLOUR_BY_CLASS[d.cls] || '#33ddff';
        const x = d.x * overlay.width;
        const y = d.y * overlay.height;
        const w = d.w * overlay.width;
        const h = d.h * overlay.height;
        ctx.strokeStyle = colour;
        ctx.strokeRect(x, y, w, h);
        ctx.fillStyle = colour;
        ctx.fillText(`${d.cls} ${(d.conf * 100).toFixed(0)}%`, x + 2, y + 2);
      }
      const verdict = snap.safety_verdict;
      const banner = document.getElementById('detector-disagree');
      if (banner) {
        if (verdict && verdict.agree === false) {
          banner.style.display = 'block';
          banner.textContent = 'DETECTOR DISAGREEMENT — ' + (verdict.note || '');
        } else {
          banner.style.display = 'none';
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
