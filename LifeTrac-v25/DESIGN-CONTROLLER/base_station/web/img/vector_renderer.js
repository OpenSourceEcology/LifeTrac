// vector_renderer.js — draws the VS1 vector scene (VECTOR_SCENE.md §7.5).
//
// Paints on its own overlay canvas `#image-vector`, never on `#image-canvas`
// (source_guard.js samples that one). Subscribes to the `lifetrac-state`
// event that canvas_renderer.js dispatches with every /ws/state snapshot and
// draws `snap.vector_scene` whenever `snap.encode_mode === 'vector'`.
//
// Fail-closed rules (§7.5): the banner "VECTOR — NOT CAMERA PIXELS" is
// inside this canvas, so raw mode cannot hide it (the overlay is deliberately
// NOT in the raw-mode hide list); "NO VECTOR DATA" when the mode is vector
// but no scene has arrived; a shape with an unknown kind or badge is not
// painted, its bounding box is blacked out with "BADGE?"; age styling per
// §4.3 (tint > 1.5 s, desaturate + age > 5 s, outline only > 10 s); previous-
// epoch shapes (badge 1 CACHED) are desaturated and outline-weighted.
//
// Snapshot geometry is already in canvas pixels (the store applies offsets,
// group shifts and GZOOM), so this file only draws what it is given.

(function () {
  'use strict';

  const BADGE_VECTOR = 7, BADGE_CACHED = 1, BADGE_PREDICTED = 4;
  const SHAPE_BADGES = new Set([BADGE_VECTOR, BADGE_CACHED, BADGE_PREDICTED]);
  const EDGE_COLOUR = { rut: '#c8b090', structure: '#e0e0e0', fence: '#d8c060',
                        contact: '#ff6060', overhead: '#ffe040' };
  const EDGE_CLS = ['rut', 'structure', 'fence', 'contact', 'overhead'];

  let overlay = null, ctx = null;

  function init() {
    const base = document.getElementById('image-canvas');
    if (!base) return false;
    overlay = document.getElementById('image-vector');
    if (!overlay) {
      overlay = document.createElement('canvas');
      overlay.id = 'image-vector';
      overlay.style.position = 'absolute';
      overlay.style.pointerEvents = 'none';
      overlay.style.left = base.offsetLeft + 'px';
      overlay.style.top = base.offsetTop + 'px';
      base.parentNode.appendChild(overlay);
    }
    overlay.width = base.width;
    overlay.height = base.height;
    ctx = overlay.getContext('2d');
    return true;
  }

  function sizeTo(snap) {
    const w = snap.grid.w * snap.grid.tile_px, h = snap.grid.h * snap.grid.tile_px;
    if (overlay.width !== w || overlay.height !== h) { overlay.width = w; overlay.height = h; }
  }

  function ageStyle(age) {
    // §4.3 age styling; returns [alpha multiplier, desaturate, outlineOnly, showAge]
    if (age > 10000) return [1.0, true, true, true];
    if (age > 5000) return [0.85, true, false, true];
    if (age > 1500) return [0.75, false, false, false];
    return [1.0, false, false, false];
  }

  function desat(hex) {
    // cheap desaturation: blend the colour halfway to its luma grey
    if (!/^#[0-9a-f]{6}$/i.test(hex)) return hex;
    const r = parseInt(hex.slice(1, 3), 16), g = parseInt(hex.slice(3, 5), 16), b = parseInt(hex.slice(5, 7), 16);
    const l = Math.round(0.299 * r + 0.587 * g + 0.114 * b);
    const mix = (c) => Math.round((c + l) / 2).toString(16).padStart(2, '0');
    return '#' + mix(r) + mix(g) + mix(b);
  }

  function fillStyleFor(fill, opts) {
    if (!fill || !fill.c0) return '#808080';
    let c0 = fill.c0, c1 = fill.c1 || fill.c0;
    if (opts.desaturate) { c0 = desat(c0); c1 = desat(c1); }
    if (fill.g && fill.g.length === 4 && c1 !== c0) {
      const grad = ctx.createLinearGradient(fill.g[0], fill.g[1], fill.g[2], fill.g[3]);
      grad.addColorStop(0, c0);
      grad.addColorStop(1, c1);
      return grad;
    }
    return c0;
  }

  function pathFromPts(pts) {
    const p = new Path2D();
    if (!pts || pts.length < 4) return p;
    p.moveTo(pts[0], pts[1]);
    for (let i = 2; i + 1 < pts.length; i += 2) p.lineTo(pts[i], pts[i + 1]);
    p.closePath();
    return p;
  }

  function drawHorizon(h, W, H) {
    if (!h) return;
    const pts = h.pts || [];
    const sky = h.sky || ['#8fb3d9', '#cfe0ef'], ground = h.ground || ['#6b8f3a', '#4f6b2b'];
    if (h.mode === 'none' || pts.length < 2) {
      // NO_HORIZON: top and bottom bands only (§2.4)
      const g1 = ctx.createLinearGradient(0, 0, 0, H / 2); g1.addColorStop(0, sky[0]); g1.addColorStop(1, sky[1]);
      ctx.fillStyle = g1; ctx.fillRect(0, 0, W, H / 2);
      const g2 = ctx.createLinearGradient(0, H / 2, 0, H); g2.addColorStop(0, ground[0]); g2.addColorStop(1, ground[1]);
      ctx.fillStyle = g2; ctx.fillRect(0, H / 2, W, H / 2);
      return;
    }
    // sky: everything above the horizon polyline; ground: everything below
    const skyPath = new Path2D();
    skyPath.moveTo(0, 0);
    for (const [x, y] of pts) skyPath.lineTo(x, y);
    skyPath.lineTo(W, 0); skyPath.closePath();
    const yMid = pts[Math.floor(pts.length / 2)][1];
    const gs = ctx.createLinearGradient(0, 0, 0, Math.max(1, yMid));
    gs.addColorStop(0, sky[0]); gs.addColorStop(1, sky[1]);
    ctx.fillStyle = gs; ctx.fill(skyPath);
    const gndPath = new Path2D();
    gndPath.moveTo(0, H);
    for (const [x, y] of pts) gndPath.lineTo(x, y);
    gndPath.lineTo(W, H); gndPath.closePath();
    const gg = ctx.createLinearGradient(0, Math.min(H - 1, yMid), 0, H);
    gg.addColorStop(0, ground[0]); gg.addColorStop(1, ground[1]);
    ctx.fillStyle = gg; ctx.fill(gndPath);
    // skyline band (treeline profile above the horizon), if any
    if (Array.isArray(h.skyline) && h.skyline.length > 1) {
      const sp = new Path2D();
      sp.moveTo(h.skyline[0][0], pts[0][1]);
      for (const [x, y] of h.skyline) sp.lineTo(x, y);
      sp.lineTo(h.skyline[h.skyline.length - 1][0], pts[pts.length - 1][1]);
      sp.closePath();
      ctx.fillStyle = (h.skyline_fill && h.skyline_fill.c0) || '#2f4a26';
      ctx.fill(sp);
    }
    ctx.strokeStyle = 'rgba(255,255,255,0.35)'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(pts[0][0], pts[0][1]);
    for (const [x, y] of pts) ctx.lineTo(x, y);
    ctx.stroke();
  }

  function bboxOf(s) {
    if (Array.isArray(s.pts) && s.pts.length >= 2) {
      let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
      for (let i = 0; i + 1 < s.pts.length; i += 2) {
        x0 = Math.min(x0, s.pts[i]); x1 = Math.max(x1, s.pts[i]);
        y0 = Math.min(y0, s.pts[i + 1]); y1 = Math.max(y1, s.pts[i + 1]);
      }
      return [x0, y0, x1 - x0, y1 - y0];
    }
    if (Array.isArray(s.box)) return s.box;
    if (typeof s.cx === 'number') return [s.cx - (s.rx || s.r || 4), s.cy - (s.ry || s.r || 4), 2 * (s.rx || s.r || 4), 2 * (s.ry || s.r || 4)];
    return [0, 0, 8, 8];
  }

  function refuse(s, label) {
    const [x, y, w, h] = bboxOf(s);
    ctx.save();
    ctx.fillStyle = '#000'; ctx.fillRect(x, y, Math.max(w, 24), Math.max(h, 12));
    ctx.fillStyle = '#ff4040'; ctx.font = '10px ui-monospace, monospace';
    ctx.fillText(label, x + 2, y + 10);
    ctx.restore();
  }

  function drawShape(s) {
    if (!s || !SHAPE_BADGES.has(s.badge)) { refuse(s || {}, 'BADGE?'); return; }
    const age = typeof s.age_ms === 'number' ? s.age_ms : 0;
    let [alpha, desaturate, outlineOnly, showAge] = ageStyle(age);
    if (s.badge === BADGE_CACHED) { desaturate = true; alpha = Math.min(alpha, 0.8); }
    ctx.save();
    ctx.globalAlpha = alpha;
    const opts = { desaturate };
    const strokeCol = s.badge === BADGE_CACHED ? 'rgba(255,255,255,0.7)' : 'rgba(0,0,0,0.45)';
    switch (s.k) {
      case 'poly': {
        const p = pathFromPts(s.pts);
        if (!outlineOnly) { ctx.fillStyle = fillStyleFor(s.fill, opts); ctx.fill(p, 'nonzero'); }
        ctx.strokeStyle = strokeCol; ctx.lineWidth = s.badge === BADGE_CACHED ? 1.5 : 0.75; ctx.stroke(p);
        break;
      }
      case 'tree':
      case 'blob': {
        ctx.beginPath();
        ctx.ellipse(s.cx, s.cy, Math.max(1, s.rx), Math.max(1, s.ry), ((s.rot_deg || 0) * Math.PI) / 180, 0, 2 * Math.PI);
        if (!outlineOnly) { ctx.fillStyle = fillStyleFor(s.fill, opts); ctx.fill(); }
        ctx.strokeStyle = strokeCol; ctx.lineWidth = 0.75; ctx.stroke();
        if (s.k === 'tree' && s.trunk && typeof s.trunk.h === 'number' && s.trunk.h > 0) {
          // store emits {w, h, c0}: a bar of w px below the ellipse in the shadow slot;
          // it follows the same age rule as the ellipse (outline only past 10 s)
          const tw = s.trunk.w || 4;
          const tc = desaturate ? desat(s.trunk.c0 || '#223') : (s.trunk.c0 || '#223');
          if (outlineOnly) {
            ctx.strokeStyle = strokeCol; ctx.lineWidth = 0.75;
            ctx.strokeRect(s.cx - tw / 2, s.cy + s.ry, tw, s.trunk.h);
          } else {
            ctx.fillStyle = tc;
            ctx.fillRect(s.cx - tw / 2, s.cy + s.ry, tw, s.trunk.h);
          }
        }
        break;
      }
      case 'edge': {
        const cls = typeof s.cls === 'number' ? EDGE_CLS[s.cls] : s.cls;
        const col = EDGE_COLOUR[cls] || '#e0e0e0';
        ctx.lineCap = 'round'; ctx.lineJoin = 'round';
        ctx.beginPath();
        for (let i = 0; i + 1 < s.pts.length; i += 2) {
          if (i === 0) ctx.moveTo(s.pts[i], s.pts[i + 1]); else ctx.lineTo(s.pts[i], s.pts[i + 1]);
        }
        ctx.strokeStyle = 'rgba(0,0,0,0.6)'; ctx.lineWidth = cls === 'overhead' ? 4 : 3.5; ctx.stroke();   // dark halo
        ctx.strokeStyle = desaturate ? desat(col) : col; ctx.lineWidth = cls === 'overhead' ? 2.5 : 1.75; ctx.stroke();
        break;
      }
      case 'anom': {
        const [x, y, w, h] = s.box || bboxOf(s);
        if (!outlineOnly) { ctx.fillStyle = fillStyleFor(s.fill, opts); ctx.fillRect(x, y, w, h); }
        ctx.setLineDash([4, 3]); ctx.strokeStyle = '#ffffff'; ctx.lineWidth = 1.5; ctx.strokeRect(x, y, w, h);
        ctx.setLineDash([]);
        ctx.fillStyle = '#ffffff'; ctx.font = 'bold 9px ui-monospace, monospace';
        ctx.fillText('UNCLASSIFIED', x + 2, y - 2 < 8 ? y + h + 9 : y - 2);
        break;
      }
      case 'hole': {
        // §2.3 disclosed omission: hatched "UNKNOWN", never the parent colour
        ctx.beginPath(); ctx.arc(s.cx, s.cy, Math.max(2, s.r), 0, 2 * Math.PI);
        ctx.save(); ctx.clip();
        ctx.strokeStyle = 'rgba(255,255,255,0.55)'; ctx.lineWidth = 1;
        for (let d = -2 * s.r; d <= 2 * s.r; d += 4) {
          ctx.beginPath(); ctx.moveTo(s.cx + d - s.r, s.cy - s.r); ctx.lineTo(s.cx + d + s.r, s.cy + s.r); ctx.stroke();
        }
        ctx.restore();
        ctx.beginPath(); ctx.arc(s.cx, s.cy, Math.max(2, s.r), 0, 2 * Math.PI);
        ctx.strokeStyle = 'rgba(255,255,255,0.8)'; ctx.lineWidth = 1; ctx.stroke();
        if (s.r >= 10) { ctx.fillStyle = '#fff'; ctx.font = '8px ui-monospace, monospace'; ctx.fillText('UNKNOWN', s.cx - 16, s.cy + 3); }
        break;
      }
      default:
        ctx.restore();
        refuse(s, 'BADGE?');
        return;
    }
    if (showAge) {
      const [x, y] = bboxOf(s);
      ctx.globalAlpha = 1;
      ctx.fillStyle = 'rgba(0,0,0,0.6)'; ctx.fillRect(x, y, 30, 11);
      ctx.fillStyle = '#ffd050'; ctx.font = '9px ui-monospace, monospace';
      ctx.fillText((age / 1000).toFixed(1) + 's', x + 2, y + 9);
    }
    ctx.restore();
  }

  function chip(text, x, y, colour) {
    ctx.save();
    ctx.font = 'bold 11px ui-monospace, monospace';
    const w = ctx.measureText(text).width + 10;
    ctx.fillStyle = 'rgba(0,0,0,0.65)'; ctx.fillRect(x, y, w, 16);
    ctx.fillStyle = colour; ctx.fillText(text, x + 5, y + 12);
    ctx.restore();
    return w;
  }

  function render(snap) {
    if (!ctx && !init()) return;
    sizeTo(snap);
    const W = overlay.width, H = overlay.height;
    ctx.clearRect(0, 0, W, H);
    if (snap.encode_mode !== 'vector') return;      // photo modes: overlay stays clear
    const scene = snap.vector_scene;
    ctx.fillStyle = '#000'; ctx.fillRect(0, 0, W, H);   // never a silent fallback to the old photo
    if (!scene) {
      ctx.fillStyle = '#ffd050'; ctx.font = 'bold 18px ui-monospace, monospace';
      ctx.textAlign = 'center'; ctx.fillText('NO VECTOR DATA', W / 2, H / 2); ctx.textAlign = 'start';
      chip('VECTOR — NOT CAMERA PIXELS', 6, 6, '#9fd0ff');
      return;
    }
    drawHorizon(scene.horizon, W, H);
    for (const layer of scene.layers || []) {
      for (const s of layer.shapes || []) drawShape(s);
    }
    // chips: banner, epoch/anchor age, resync/digest state
    let x = 6;
    x += chip('VECTOR — NOT CAMERA PIXELS', x, 6, '#9fd0ff') + 4;
    const aa = typeof scene.anchor_age_ms === 'number' ? (scene.anchor_age_ms / 1000).toFixed(1) + 's' : '?';
    x += chip('epoch ' + scene.epoch + ' · anchor ' + aa, x, 6, '#dddddd') + 4;
    if (scene.resync) x += chip('RESYNC', x, 6, '#ff8040') + 4;
    else if (scene.digest_ok === false) x += chip('DIGEST MISMATCH', x, 6, '#ffb040') + 4;
    if (typeof scene.corr_detected === 'number') {
      chip(scene.corr_shown + ' / ' + scene.corr_detected + ' anomalies shown', x, 6, scene.corr_shown < scene.corr_detected ? '#ff8040' : '#a0e0a0');
    }
  }

  window.addEventListener('lifetrac-state', (ev) => {
    try { render(ev.detail); } catch (e) { /* surfaced by badge_renderer's health log path */ }
  });
  window.addEventListener('lifetrac-tile-painted', () => { /* photo tiles: nothing to do here */ });
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init); else init();

  window.renderVectorScene = function (context, scene, opts) {
    // exported for the Vector Lab page (§8), which renders a Lab store on its own canvas
    const saved = ctx, savedOverlay = overlay;
    ctx = context; overlay = context.canvas;
    try { render({ encode_mode: 'vector', vector_scene: scene, grid: (opts && opts.grid) || { w: 12, h: 8, tile_px: 32 } }); }
    finally { ctx = saved; overlay = savedOverlay; }
  };
})();
