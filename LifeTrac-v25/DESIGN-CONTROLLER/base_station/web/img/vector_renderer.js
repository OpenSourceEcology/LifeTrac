// vector_renderer.js — draws the VS1 vector scene (VECTOR_SCENE.md §7.5).
//
// Paints on its own overlay canvas `#image-vector`, never on `#image-canvas`
// (source_guard.js samples that one). Subscribes to the `lifetrac-state`
// event that canvas_renderer.js dispatches with every /ws/state snapshot and
// draws `snap.vector_scene` whenever `snap.encode_mode === 'vector'`.
//
// Fail-closed rules (§7.5): the banner "VECTOR — NOT CAMERA PIXELS · age ·
// loss" is inside this canvas, so raw mode cannot hide it (the overlay is
// deliberately NOT in the raw-mode hide list); a scene fault (rule 1: v ≠ 1,
// scene badge ≠ 7, no anchor_age_ms, or a scene this file cannot draw) gives
// a black "VECTOR?" panel and one POST to /api/health/refusal per fault
// episode; "NO VECTOR DATA" when the mode is vector but no scene has arrived;
// a shape with an unknown kind or badge is not painted, its bounding box is
// blacked out with "BADGE?"; age styling per §4.3 (tint > 1.5 s, desaturate
// + age > 5 s, outline only > 10 s); previous-epoch shapes (badge 1 CACHED)
// are desaturated and outline-weighted; "SAFETY DETECTOR: NO PIXELS" whenever
// the snapshot says the base detector has no pixels.
//
// Layout (RS-13.2 desk check, F1/F2): the backing store follows the element's
// CSS size. The scene is drawn in its own grid pixels through a transform,
// stretched with the panel as before; the banner and the chips are drawn in
// CSS pixels below the console's top bar and beside the settings drawer and
// the telemetry panel while those are open (`data-vector-avoid` in
// index.html), and wrap onto another row rather than run off the edge, so
// every chip stays whole and readable at any window size.
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
  const BANNER = 'VECTOR — NOT CAMERA PIXELS';
  const SAFETY_CHIP = ['SAFETY DETECTOR: NO PIXELS', '#ff6a50'];
  const REFUSAL_MIN_MS = 10000;   // a fault that flaps still posts at most once per 10 s

  let overlay = null, ctx = null, lastSnap = null;
  let faultOpen = false, faultPosted = false, lastRefusalMs = -Infinity;

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
    ctx = overlay.getContext('2d');
    return true;
  }

  // Sizes the backing store to the element (own overlay only; the Lab's canvas
  // keeps its size) and returns the two scales: scene grid px (W×H) to backing
  // px, and CSS px to backing px. The console's chrome is marked in index.html
  // (F1): `top`, where the chips may start, is below a `data-vector-avoid=
  // "below"` bar over the panel's top half; `blocks` are the `"beside"` panels
  // that are open, for the chips to step around. Both in CSS px. An element
  // with no layout box is treated as W×H CSS px and left as it is.
  function fit(W, H, own) {
    const r = overlay.getBoundingClientRect ? overlay.getBoundingClientRect() : { width: 0, height: 0, top: 0, left: 0 };
    const laidOut = r.width > 0 && r.height > 0;
    const cssW = laidOut ? r.width : W, cssH = laidOut ? r.height : H;
    if (own && laidOut) {
      const dpr = Math.min(2, window.devicePixelRatio || 1);
      const bw = Math.max(1, Math.round(cssW * dpr)), bh = Math.max(1, Math.round(cssH * dpr));
      if (overlay.width !== bw || overlay.height !== bh) { overlay.width = bw; overlay.height = bh; }
    }
    let top = 6;
    const blocks = [];
    if (own && laidOut) {
      for (const el of document.querySelectorAll('[data-vector-avoid]')) {
        const e = el.getBoundingClientRect();
        if (!(e.width > 0 && e.height > 0)) continue;                    // a closed panel
        if (el.dataset.vectorAvoid === 'beside') {
          blocks.push([e.left - r.left, e.top - r.top, e.right - r.left, e.bottom - r.top]);
        } else if (e.bottom > r.top && e.top < r.top + cssH / 2) {
          top = Math.max(top, e.bottom - r.top + 6);
        }
      }
    }
    return { sx: overlay.width / W, sy: overlay.height / H, k: overlay.width / cssW, w: cssW, h: cssH, top, blocks };
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
    const sky = h.sky, ground = h.ground;
    // No measured colours yet (an epoch joined before its anchor): the panel
    // stays black rather than show an invented sky (principle 4).
    if (!Array.isArray(sky) || !Array.isArray(ground)) return;
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

  function chipFont(px) { return 'bold ' + px + 'px ui-monospace, monospace'; }

  // Chips in CSS px, left to right from `L.top`, stepping past the `blocks`
  // and wrapping to a new row before one would cross the right edge (F1, F2);
  // a chip wider than the panel alone shrinks its font to fit. The size
  // follows the panel width, 12–18 px. Returns [text, colour, font px, x, y,
  // w] per chip, the chip height and the bottom of the last row.
  function layChips(L, chips, blocks) {
    const pad = 6, gap = 6, fs = Math.round(Math.max(12, Math.min(18, L.w / 90))), ch = fs + 8;
    const out = [];
    let x = pad, y = L.top;
    for (const [text, colour] of chips) {
      let f = fs;
      ctx.font = chipFont(f);
      let w = ctx.measureText(text).width + 12;
      while (w > L.w - 2 * pad && f > 9) { f--; ctx.font = chipFont(f); w = ctx.measureText(text).width + 12; }
      for (let n = 0; n < 64; n++) {                 // bounded: rows move down past every block
        const b = blocks.find((b) => y < b[3] && y + ch > b[1] && x < b[2] + gap && x + w > b[0] - gap);
        if (b) x = b[2] + gap;
        else if (x > pad && x + w > L.w - pad) { x = pad; y += ch + 4; }
        else break;
      }
      out.push([text, colour, f, x, y, w]);
      x += w + gap;
    }
    return { out, ch, bottom: y + ch };
  }

  // Beside the open side panels while that keeps the chips in the panel's top
  // half (clear of the joysticks); with no room there, laid out as if the
  // panels were closed, which then cover part of the row as they cover the
  // scene.
  function drawChips(L, chips) {
    ctx.save();
    ctx.setTransform(L.k, 0, 0, L.k, 0, 0);
    ctx.textBaseline = 'middle';
    let lay = layChips(L, chips, L.blocks);
    if (L.blocks.length && lay.bottom > L.h / 2) lay = layChips(L, chips, []);
    for (const [text, colour, f, x, y, w] of lay.out) {
      ctx.font = chipFont(f);
      ctx.fillStyle = 'rgba(0,0,0,0.72)'; ctx.fillRect(x, y, w, lay.ch);
      ctx.fillStyle = colour; ctx.fillText(text, x + 6, y + lay.ch / 2 + 1);
    }
    ctx.restore();
  }

  function panelText(L, text, colour) {
    ctx.save();
    ctx.setTransform(L.k, 0, 0, L.k, 0, 0);
    ctx.font = chipFont(Math.round(Math.max(18, Math.min(40, L.w / 30))));
    ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
    ctx.fillStyle = colour; ctx.fillText(text, L.w / 2, L.h / 2);
    ctx.restore();
  }

  // "VECTOR — NOT CAMERA PIXELS · age · loss" (§0.2, §7.5): the capture age of
  // the newest frame the store applied and the share of frames that seq gaps
  // show lost in the last 10 s, both from the store (F5). "?" when unknown.
  function bannerText(scene) {
    const age = scene && typeof scene.frame_age_ms === 'number'
      ? 'age ' + (scene.frame_age_ms / 1000).toFixed(1) + ' s' : 'age ?';
    const loss = scene && scene.loss && typeof scene.loss.ratio === 'number'
      ? 'loss ' + Math.round(100 * scene.loss.ratio) + ' %' : 'loss ?';
    return BANNER + ' · ' + age + ' · ' + loss;
  }

  // Banner first, then the safety chip (F6), the store's state, epoch/anchor and
  // anomalies. A scene that is null or refused contributes nothing it states.
  function chipsFor(snap, scene) {
    const chips = [[bannerText(scene), '#9fd0ff']];
    if (snap.safety_detector === 'no_pixels') chips.push(SAFETY_CHIP);
    if (!scene) return chips;
    if (scene.resync) chips.push(['RESYNC', '#ff8040']);
    else if (scene.digest_ok === false) chips.push(['DIGEST MISMATCH', '#ffb040']);
    const aa = typeof scene.anchor_age_ms === 'number' ? (scene.anchor_age_ms / 1000).toFixed(1) + 's' : 'none yet';
    chips.push(['epoch ' + scene.epoch + ' · anchor ' + aa, '#dddddd']);
    if (typeof scene.corr_detected === 'number') {
      chips.push([scene.corr_shown + ' / ' + scene.corr_detected + ' anomalies shown',
                  scene.corr_shown < scene.corr_detected ? '#ff8040' : '#a0e0a0']);
    }
    return chips;
  }

  // §7.5 rule 1. The store sends anchor_age_ms = null until the epoch has an
  // absolute anchor (a mid-epoch join): that is drawn ("anchor none yet"), a
  // missing or non-numeric one is refused.
  function sceneFault(scene) {
    if (typeof scene !== 'object' || Array.isArray(scene)) return 'not_object';
    if (scene.v !== 1) return 'v';
    if (scene.badge !== BADGE_VECTOR) return 'badge';
    const a = scene.anchor_age_ms;
    if (a === undefined || (a !== null && typeof a !== 'number')) return 'anchor_age_ms';
    return null;
  }

  // One POST per fault episode (the episode ends with the next scene drawn, or
  // a snapshot outside VECTOR), and at most one per REFUSAL_MIN_MS: an episode
  // that opens sooner posts when the interval has passed, if it is still open.
  // A fault that persists or flaps never turns the 2 Hz snapshots into a POST
  // storm.
  function reportFault(fault, scene, err) {
    if (!faultOpen) { faultOpen = true; faultPosted = false; }
    const now = Date.now();
    if (faultPosted || now - lastRefusalMs < REFUSAL_MIN_MS) return;
    faultPosted = true;
    lastRefusalMs = now;
    const s = (scene && typeof scene === 'object') ? scene : {};
    const num = (v) => (typeof v === 'number' ? v : null);
    const body = { reason: 'vector_scene_fault', fault: fault, v: num(s.v), badge: num(s.badge),
                   epoch: num(s.epoch), ts_ms: now };
    if (err) body.error = String((err && err.message) || err).slice(0, 120);
    try {
      fetch('/api/health/refusal', {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify(body),
        credentials: 'same-origin',
      }).catch(() => {});
    } catch (_) { /* health endpoint optional in tests */ }
  }

  function refuseScene(L, snap, fault, own, err) {
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.fillStyle = '#000'; ctx.fillRect(0, 0, overlay.width, overlay.height);
    panelText(L, 'VECTOR?', '#ff4040');
    drawChips(L, chipsFor(snap, null));
    if (own) reportFault(fault, snap.vector_scene, err);
  }

  function gridOf(snap) {
    const g = snap.grid || {};
    return [(g.w * g.tile_px) || 384, (g.h * g.tile_px) || 256];
  }

  // `own`: the console overlay. The Lab export draws on its canvas as sized
  // and never posts refusals.
  function render(snap, own) {
    if (!ctx && !init()) return;
    const [W, H] = gridOf(snap);
    const L = fit(W, H, own);
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.clearRect(0, 0, overlay.width, overlay.height);
    if (snap.encode_mode !== 'vector') {            // photo modes: overlay stays clear
      if (own) faultOpen = false;
      if (snap.safety_detector === 'no_pixels') drawChips(L, [SAFETY_CHIP]);
      return;
    }
    const scene = snap.vector_scene;
    ctx.fillStyle = '#000'; ctx.fillRect(0, 0, overlay.width, overlay.height);   // never a silent fallback to the old photo
    if (!scene) {
      if (own) faultOpen = false;
      panelText(L, 'NO VECTOR DATA', '#ffd050');
      drawChips(L, chipsFor(snap, null));
      return;
    }
    const fault = sceneFault(scene);
    if (fault) { refuseScene(L, snap, fault, own); return; }
    ctx.setTransform(L.sx, 0, 0, L.sy, 0, 0);       // the scene in its grid px, stretched with the panel
    drawHorizon(scene.horizon, W, H);
    for (const layer of scene.layers || []) {
      for (const s of layer.shapes || []) drawShape(s);
    }
    drawChips(L, chipsFor(snap, scene));
    if (own) faultOpen = false;
  }

  function paint(snap) {
    lastSnap = snap;
    try {
      render(snap, true);
    } catch (e) {
      // A scene this file cannot draw is refused like rule 1 (§7.5 rule 5: a
      // renderer that cannot parse a newer snapshot shows "VECTOR?").
      try {
        if (ctx && snap && snap.encode_mode === 'vector') {
          overlay.width = overlay.width;            // drops a save() the throw left open (alpha, clip)
          const [W, H] = gridOf(snap);
          refuseScene(fit(W, H, true), snap, 'render_error', true, e);
        }
      } catch (_) { /* nothing more to draw with */ }
    }
  }

  window.addEventListener('lifetrac-state', (ev) => { if (ev.detail) paint(ev.detail); });
  window.addEventListener('resize', () => { if (lastSnap) paint(lastSnap); });   // re-lay the chips at once
  window.addEventListener('lifetrac-tile-painted', () => { /* photo tiles: nothing to do here */ });
  if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init); else init();

  window.renderVectorScene = function (context, scene, opts) {
    // exported for the Vector Lab page (§8), which renders a Lab store on its own canvas
    const saved = ctx, savedOverlay = overlay;
    ctx = context; overlay = context.canvas;
    try { render({ encode_mode: 'vector', vector_scene: scene, grid: (opts && opts.grid) || { w: 12, h: 8, tile_px: 32 } }, false); }
    finally { ctx = saved; overlay = savedOverlay; }
  };
})();
