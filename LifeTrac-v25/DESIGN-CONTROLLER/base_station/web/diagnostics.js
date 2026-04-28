// diagnostics.js — Link-health renderer for diagnostics.html.
//
// Pure browser-side; consumes the LinkSnapshot JSON pushed on /ws/state.
// Server is authoritative — we only display fields we recognise; anything
// missing renders as "—" rather than fabricating a value (fail-closed
// per IMAGE_PIPELINE.md §6.1).
//
// Inputs (each `state` message):
//   {
//     link: {
//       image_util: 0..1, telemetry_util: 0..1, total_util: 0..1,
//       encode_mode: "FULL"|"Y_ONLY"|"MOTION_ONLY"|"WIREFRAME",
//       candidate_mode: same enum, candidate_count: int,
//       sf_rung: 0|1|2,                  // optional
//       sf_history: [{ts_ms, rung, sf}], // optional
//       fhss: {channel:int, last_skip_ms:int}, // optional
//       link_loss: {longest_ms, last_ms, current_ms} // optional
//     }
//   }
(() => {
  "use strict";

  const WINDOW_S = 60;
  const TICK_MS  = 1000;

  // Ring buffers indexed by epoch second.
  const airtime = []; // {t_s, image, telem, total}
  const sfHist  = []; // {t_s, rung, sf}
  const fhss    = []; // {t_s, channel}
  const linkLog = []; // {t_s, current_ms}

  function pushBounded(arr, item, maxLen) {
    arr.push(item);
    if (arr.length > maxLen) arr.shift();
  }

  function fitCanvas(cv) {
    const dpr = window.devicePixelRatio || 1;
    const r = cv.getBoundingClientRect();
    cv.width = Math.max(1, Math.round(r.width * dpr));
    cv.height = Math.max(1, Math.round(r.height * dpr));
    const ctx = cv.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { ctx, w: r.width, h: r.height };
  }

  // ----- Airtime -----
  function drawAirtime() {
    const cv = document.getElementById("cv-airtime");
    if (!cv) return;
    const { ctx, w, h } = fitCanvas(cv);
    ctx.fillStyle = "#1a1d22"; ctx.fillRect(0, 0, w, h);
    // Grid + thresholds (30 %, 60 %)
    ctx.strokeStyle = "#333"; ctx.lineWidth = 1;
    ctx.beginPath();
    for (let p = 0; p <= 100; p += 25) {
      const y = h - (p / 100) * (h - 4) - 2;
      ctx.moveTo(0, y); ctx.lineTo(w, y);
    }
    ctx.stroke();
    function drawLine(p, color) {
      const y = h - (p / 100) * (h - 4) - 2;
      ctx.strokeStyle = color; ctx.setLineDash([4, 4]);
      ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke();
      ctx.setLineDash([]);
    }
    drawLine(30, "#ff7a00"); drawLine(60, "#d62828");
    if (airtime.length === 0) return;
    const t0 = airtime[0].t_s;
    const tMax = airtime[airtime.length - 1].t_s;
    const span = Math.max(1, tMax - t0);
    function plot(field, color) {
      ctx.strokeStyle = color; ctx.lineWidth = 2; ctx.beginPath();
      for (let i = 0; i < airtime.length; i++) {
        const x = ((airtime[i].t_s - t0) / span) * (w - 2) + 1;
        const v = airtime[i][field] || 0;
        const y = h - Math.min(1, v) * (h - 4) - 2;
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
    plot("image", "#4ea1ff");
    plot("telem", "#ffc857");
    plot("total", "#e6e8eb");
  }

  // ----- SF rung -----
  function drawSf() {
    const cv = document.getElementById("cv-sf");
    if (!cv) return;
    const { ctx, w, h } = fitCanvas(cv);
    ctx.fillStyle = "#1a1d22"; ctx.fillRect(0, 0, w, h);
    if (sfHist.length === 0) return;
    const t0 = sfHist[0].t_s;
    const tMax = sfHist[sfHist.length - 1].t_s;
    const span = Math.max(1, tMax - t0);
    const rungColor = ["#4caf50", "#ff7a00", "#d62828"];
    for (let i = 0; i < sfHist.length; i++) {
      const x = ((sfHist[i].t_s - t0) / span) * (w - 2) + 1;
      const r = Math.max(0, Math.min(2, sfHist[i].rung || 0));
      ctx.fillStyle = rungColor[r];
      const barH = ((r + 1) / 3) * (h - 4);
      ctx.fillRect(x - 1, h - barH - 2, 3, barH);
    }
  }

  // ----- FHSS heatmap -----
  function drawFhss() {
    const cv = document.getElementById("cv-fhss");
    if (!cv) return;
    const { ctx, w, h } = fitCanvas(cv);
    ctx.fillStyle = "#1a1d22"; ctx.fillRect(0, 0, w, h);
    const channels = 8;
    const cellW = w / WINDOW_S;
    const cellH = h / channels;
    if (fhss.length === 0) return;
    // Bucket by channel + recent second.
    const tMax = Math.floor(Date.now() / 1000);
    const buckets = Array.from({ length: channels }, () => new Array(WINDOW_S).fill(0));
    for (const ev of fhss) {
      const dt = tMax - ev.t_s;
      if (dt < 0 || dt >= WINDOW_S) continue;
      const ch = Math.max(0, Math.min(channels - 1, ev.channel || 0));
      buckets[ch][WINDOW_S - 1 - dt] += 1;
    }
    let maxCount = 1;
    for (const row of buckets) for (const v of row) if (v > maxCount) maxCount = v;
    for (let ch = 0; ch < channels; ch++) {
      for (let s = 0; s < WINDOW_S; s++) {
        const v = buckets[ch][s] / maxCount;
        if (v <= 0) continue;
        const r = Math.round(40 + 200 * v);
        ctx.fillStyle = `rgb(${r},${Math.round(60 + 60 * v)},${Math.round(80 - 60 * v)})`;
        ctx.fillRect(s * cellW, ch * cellH, cellW + 0.5, cellH + 0.5);
      }
    }
    ctx.strokeStyle = "#333";
    for (let ch = 0; ch <= channels; ch++) {
      ctx.beginPath(); ctx.moveTo(0, ch * cellH); ctx.lineTo(w, ch * cellH); ctx.stroke();
    }
  }

  // ----- Link loss timeline -----
  function drawLink() {
    const cv = document.getElementById("cv-link");
    if (!cv) return;
    const { ctx, w, h } = fitCanvas(cv);
    ctx.fillStyle = "#1a1d22"; ctx.fillRect(0, 0, w, h);
    if (linkLog.length === 0) return;
    const t0 = linkLog[0].t_s;
    const tMax = linkLog[linkLog.length - 1].t_s;
    const span = Math.max(1, tMax - t0);
    const maxMs = Math.max(1000, ...linkLog.map(p => p.current_ms || 0));
    ctx.strokeStyle = "#4ea1ff"; ctx.lineWidth = 2; ctx.beginPath();
    for (let i = 0; i < linkLog.length; i++) {
      const x = ((linkLog[i].t_s - t0) / span) * (w - 2) + 1;
      const y = h - ((linkLog[i].current_ms || 0) / maxMs) * (h - 4) - 2;
      if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }

  function setText(id, val) {
    const el = document.getElementById(id);
    if (el) el.textContent = val;
  }

  function pct(v) {
    if (v == null || isNaN(v)) return "—";
    return (v * 100).toFixed(1) + " %";
  }

  function ms(v) {
    if (v == null || isNaN(v)) return "—";
    return v >= 1000 ? (v / 1000).toFixed(1) + " s" : Math.round(v) + " ms";
  }

  function applyState(state) {
    if (!state || typeof state !== "object") return;
    const link = state.link || {};
    const t_s = Math.floor((state.ts_ms || Date.now()) / 1000);
    if (link.image_util != null || link.total_util != null) {
      pushBounded(airtime, {
        t_s,
        image: link.image_util || 0,
        telem: link.telemetry_util || 0,
        total: link.total_util || 0,
      }, WINDOW_S * 4);
      setText("now-image", pct(link.image_util));
      setText("now-telem", pct(link.telemetry_util));
      setText("now-total", pct(link.total_util));
    }
    if (link.sf_rung != null) {
      pushBounded(sfHist, {
        t_s, rung: link.sf_rung, sf: link.sf || null,
      }, 256);
      setText("now-rung", String(link.sf_rung));
    }
    if (link.candidate_mode != null) {
      setText("now-cand",
        `${link.candidate_mode} (${link.candidate_count || 0}/3)`);
    }
    if (link.encode_mode != null) {
      setText("now-mode", link.encode_mode);
    }
    if (link.fhss && link.fhss.channel != null) {
      pushBounded(fhss, { t_s, channel: link.fhss.channel }, 1024);
    }
    if (link.link_loss) {
      pushBounded(linkLog, {
        t_s, current_ms: link.link_loss.current_ms || 0,
      }, WINDOW_S * 4);
      setText("link-longest", ms(link.link_loss.longest_ms));
      setText("link-last", ms(link.link_loss.last_ms));
      setText("link-now", ms(link.link_loss.current_ms));
    }
  }

  function redrawAll() {
    drawAirtime(); drawSf(); drawFhss(); drawLink();
  }

  // ----- WebSocket -----
  function connect() {
    const proto = location.protocol === "https:" ? "wss:" : "ws:";
    const ws = new WebSocket(`${proto}//${location.host}/ws/state`);
    ws.onmessage = (ev) => {
      try {
        applyState(JSON.parse(ev.data));
      } catch (_) {
        // Refusal-on-bad-payload — drop silently.
      }
    };
    ws.onclose = () => setTimeout(connect, 1500);
    ws.onerror = () => ws.close();
  }

  window.addEventListener("resize", redrawAll);
  setInterval(redrawAll, TICK_MS);
  connect();
})();
