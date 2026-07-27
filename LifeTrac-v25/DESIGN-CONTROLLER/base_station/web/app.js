// app.js — LifeTrac v25 base-station operator console.
// DRAFT FOR REVIEW. Not run yet.
//
// - Two virtual joysticks (touch + mouse).
// - USB Gamepad API support: when a controller is detected, its sticks/buttons
//   preempt the on-screen ones. Mapping (Standard Gamepad layout):
//     axes[0,1]  → LH joystick (drive)
//     axes[2,3]  → RH joystick (boom/aux)
//     buttons[0] (A) → bucket curl
//     buttons[1] (B) → bucket dump
//     buttons[2] (X) → aux1
//     buttons[3] (Y) → aux2
//     buttons[5] (RB)→ TAKE CONTROL hold (latch is server-side via flags bit 0)
//     buttons[9] (START) → E-stop
//
// - 50 Hz local read loop, 20 Hz WebSocket transmit (server enforces too).
// - Telemetry WebSocket fans MQTT topics into the sidebar.

(() => {
  'use strict';

  // ----- ControlFrame state shared across producers -----
  const state = {
    lhx: 0, lhy: 0, rhx: 0, rhy: 0,
    buttons: 0,    // bitmap: bit0 curl, bit1 dump, bit2 aux1, bit3 aux2, bit6 mode, bit7 takectl
    flags: 0,      // bit0 takectl_held
  };
  let gamepadActive = false;
  let gamepadButtonsMask = 0;

  // ----- WebSockets -----
  const linkEl = document.getElementById('link-status');
  let controlsLocked = false;

  // ----- WebSockets -----
  // wsCtrl auto-reconnects on close so a transient server restart or
  // cap-rejection doesn't permanently break the control link.
  let wsCtrl = null;
  let wsCtrlReconnectTimer = null;
  let wsCtrlReconnectDelayMs = 1000;

  function connectCtrl() {
    if (wsCtrlReconnectTimer) { clearTimeout(wsCtrlReconnectTimer); wsCtrlReconnectTimer = null; }
    wsCtrl = new WebSocket(`ws://${location.host}/ws/control`);
    wsCtrl.addEventListener('open', () => {
      linkEl.textContent = 'link: connected';
      wsCtrlReconnectDelayMs = 1000;
    });
    wsCtrl.addEventListener('close', () => {
      linkEl.textContent = 'link: DISCONNECTED';
      if (!wsCtrlReconnectTimer) {
        wsCtrlReconnectTimer = setTimeout(() => { wsCtrlReconnectTimer = null; connectCtrl(); }, wsCtrlReconnectDelayMs);
        wsCtrlReconnectDelayMs = Math.min(10000, wsCtrlReconnectDelayMs * 2);
      }
    });
    wsCtrl.addEventListener('error', () => {
      linkEl.textContent = 'link: error';
      try { wsCtrl.close(); } catch(_) {}
    });
    attachCtrlListeners(wsCtrl);
  }

  function attachCtrlListeners(ws) {
    ws.addEventListener('message', (ev) => {
      // Control socket receives no messages currently; reserved for ACK/telemetry.
    });
  }

  // Telemetry socket auto-reconnects like the control socket. Without
  // this a single drop (e.g. a web_ui restart) froze the source banner,
  // the controls-lock state, and the camera echo forever — and, since
  // 2026-07-26, latched the control-radio pill at a permanent false
  // "STALE" because its only data source is this socket.
  let wsTele = null;
  let wsTeleReconnectTimer = null;
  let wsTeleReconnectDelayMs = 1000;

  function connectTele() {
    if (wsTeleReconnectTimer) { clearTimeout(wsTeleReconnectTimer); wsTeleReconnectTimer = null; }
    wsTele = new WebSocket(`ws://${location.host}/ws/telemetry`);
    wsTele.addEventListener('open', () => {
      wsTeleReconnectDelayMs = 1000;
      // Re-seed cached server state the live stream can't replay.
      refreshEncodeMode();
    });
    wsTele.addEventListener('close', () => {
      if (!wsTeleReconnectTimer) {
        wsTeleReconnectTimer = setTimeout(() => {
          wsTeleReconnectTimer = null; connectTele();
        }, wsTeleReconnectDelayMs);
        wsTeleReconnectDelayMs = Math.min(10000, wsTeleReconnectDelayMs * 2);
      }
    });
    wsTele.addEventListener('error', () => {
      try { wsTele.close(); } catch (_) {}
    });
    wsTele.addEventListener('message', onTeleMessage);
  }

  function normalizeSource(value) {
    if (typeof value === 'string') return value.toLowerCase();
    if (typeof value === 'number') {
      return ({ 0:'none', 1:'handheld', 2:'base', 3:'autonomy', 4:'autonomy', 255:'none' })[value] || 'unknown';
    }
    if (value && typeof value === 'object') {
      return normalizeSource(value.active_source ?? value.source);
    }
    return 'unknown';
  }

  let lastSource = null;
  function playHandoffAudioCue(newSource) {
    try {
      const AudioCtx = window.AudioContext || window.webkitAudioContext;
      if (!AudioCtx) return;
      const ctx = new AudioCtx();
      
      const osc = ctx.createOscillator();
      const gain = ctx.createGain();
      osc.connect(gain);
      gain.connect(ctx.destination);
      
      const now = ctx.currentTime;
      if (newSource === 'base') {
        osc.type = 'triangle';
        osc.frequency.setValueAtTime(440, now);
        osc.frequency.linearRampToValueAtTime(880, now + 0.15);
        osc.frequency.setValueAtTime(880, now + 0.15);
        osc.frequency.linearRampToValueAtTime(1320, now + 0.3);
        
        gain.gain.setValueAtTime(0.15, now);
        gain.gain.exponentialRampToValueAtTime(0.01, now + 0.4);
        osc.start(now);
        osc.stop(now + 0.4);
      } else {
        osc.type = 'sawtooth';
        osc.frequency.setValueAtTime(660, now);
        osc.frequency.linearRampToValueAtTime(330, now + 0.2);
        
        gain.gain.setValueAtTime(0.12, now);
        gain.gain.exponentialRampToValueAtTime(0.01, now + 0.3);
        osc.start(now);
        osc.stop(now + 0.3);
      }

      if ('speechSynthesis' in window) {
        let msgStr = "";
        if (newSource === 'base') {
          msgStr = "Operator control active";
        } else if (newSource === 'handheld') {
          msgStr = "Handheld controller active";
        } else if (newSource === 'autonomy') {
          msgStr = "Autonomous steering active";
        } else {
          msgStr = "Control authority " + newSource;
        }
        window.speechSynthesis.cancel();
        const utterance = new SpeechSynthesisUtterance(msgStr);
        utterance.rate = 1.15;
        utterance.pitch = 1.0;
        utterance.volume = 0.6;
        window.speechSynthesis.speak(utterance);
      }
    } catch (e) {
      console.warn("Audio feedback play failed:", e);
    }
  }

  function setSource(value) {
    const source = normalizeSource(value);
    document.getElementById('t-src').textContent = source;
    const banner = document.getElementById('source-banner');
    controlsLocked = source === 'handheld' || source === 'autonomy';
    banner.textContent = controlsLocked
      ? `${source.toUpperCase()} ACTIVE — CONTROLS DISABLED`
      : `SOURCE: ${source.toUpperCase()}`;
    banner.className = 'source-banner ' + source;
    document.body.classList.toggle('controls-locked', controlsLocked);
    if (controlsLocked) {
      state.lhx = state.lhy = state.rhx = state.rhy = 0;
      state.buttons = 0;
      state.flags = 0;
      // Also clear the on-screen latches: a pointerup that arrives while
      // locked is swallowed by set(), so without this a held button
      // (worst case REQ CTRL, buttons bit7 + flags bit0) would re-emerge
      // latched on the first frame after unlock.
      screenButtons = 0;
      screenFlags = 0;
      syncPadVisuals(false);
      syncButtonVisuals(0);
    } else {
      syncButtonVisuals(state.buttons);
    }
    if (lastSource !== null && lastSource !== source) {
      playHandoffAudioCue(source);
    }
    lastSource = source;
  }

  function pct(value) {
    return (Number(value || 0) * 100).toFixed(1) + '%';
  }

  function setLinkStatus(data) {
    if (!data || typeof data !== 'object') return;
    if ('u_total' in data) document.getElementById('t-link-use').textContent = pct(data.u_total);
    if ('u_image' in data) document.getElementById('t-img-use').textContent = pct(data.u_image);
    if ('u_telemetry' in data) document.getElementById('t-loss').textContent = pct(data.u_telemetry);
    if ('encode_mode' in data) document.getElementById('t-encode').textContent = String(data.encode_mode).toLowerCase();
  }

  // Kick off the initial control WebSocket (auto-reconnects on close).
  connectCtrl();

  // 20 Hz tx (server also rate-limits)
  setInterval(() => {
    if (wsCtrl.readyState !== WebSocket.OPEN) return;
    if (controlsLocked) return;
    wsCtrl.send(JSON.stringify(state));
  }, 50);

  // ----- Telemetry sidebar -----
  function onTeleMessage(ev) {
    try {
      const msg = JSON.parse(ev.data);
      // Decoders are placeholders — real schema lives in lora_proto/MQTT-SN topic table.
      if (msg.topic.endsWith('/engine'))     document.getElementById('t-rpm').textContent = msg.data;
      if (msg.topic.endsWith('/battery'))    document.getElementById('t-bat').textContent = msg.data;
      if (msg.topic.endsWith('/hydraulics')) document.getElementById('t-oil').textContent = msg.data;
      if (msg.topic.endsWith('/gps'))        document.getElementById('t-gps').textContent = msg.data;
      if (msg.topic.endsWith('/active_camera')) {
        // 1B enum on topic 0x22: 0=auto,1=front,2=rear,3=implement,4=crop
        const map = { 0:'auto', 1:'front', 2:'rear', 3:'implement', 4:'crop' };
        const id = (typeof msg.data === 'number') ? msg.data : parseInt(msg.data, 16);
        const name = map[id] || '?';
        document.getElementById('t-cam').textContent = name;
        // Highlight the matching button so the UI reflects the *tractor's*
        // actual selection, not the last button click. Lets the operator
        // see immediately whether their request landed.
        document.querySelectorAll('#camera-switch button').forEach(b => {
          b.classList.toggle('active', b.dataset.cam === name);
        });
      }
      if (msg.topic.endsWith('/crop_health')) {
        // 30 B summary; here we just show mean NDVI if the bridge has
        // already JSON-decoded it. Real decoder lives elsewhere.
        const d = msg.data;
        const ndvi = (d && typeof d.ndvi_mean === 'number') ? d.ndvi_mean.toFixed(2) : d;
        document.getElementById('t-ndvi').textContent = ndvi;
      }
      // control/link_airtime is the topic the bridge actually publishes
      // ({u_image, u_telemetry, u_total, encode_mode}); /status/link kept
      // only for legacy payloads. Its arrival doubles as the control-radio
      // heartbeat for the header pill (retained replay fakes one beat on
      // reconnect; the staleness window absorbs that).
      if (msg.topic.endsWith('/control/link_airtime')) {
        lastAirtimeMs = performance.now();
        setLinkStatus(msg.data);
      }
      if (msg.topic.endsWith('/status/link') || (msg.topic.endsWith('/source_active') && msg.data && typeof msg.data === 'object')) {
        setLinkStatus(msg.data);
      }
      if (msg.topic.endsWith('/source_active')) {
        const hasSourceField = msg.data && typeof msg.data === 'object'
          && ('active_source' in msg.data || 'source' in msg.data);
        if (typeof msg.data !== 'object' || hasSourceField) setSource(msg.data);
      }
      if (msg.topic.endsWith('/control/safe_mode_active')) {
        // Bridge payload: {"active": bool, "since_ms": ...}; tolerate
        // bare booleans/ints from older publishers.
        const d = msg.data;
        const active = (d && typeof d === 'object') ? !!d.active
                     : (d === true || d === 1 || d === '1' || d === 'true');
        setSafeMode(active);
      }
    } catch (e) { /* shrug */ }
  }

  // Start the telemetry socket (auto-reconnecting).
  connectTele();
  setInterval(() => {
    if (wsTele && wsTele.readyState === WebSocket.OPEN) wsTele.send('ping');
  }, 5000);

  // ----- Control-radio health pill -----
  // Fed by control/link_airtime arrival (the bridge's periodic publish).
  // Before this pill, a dead lora_bridge / control radio was invisible:
  // the footer "link" pill only covers the browser-to-server WebSocket.
  const ctrlLinkPill = document.getElementById('ctrl-link-pill');
  setInterval(() => {
    if (!ctrlLinkPill) return;
    // Distinguish "the control radio is quiet" from "MY telemetry socket
    // is down" — otherwise any web_ui restart latches a red alarm about
    // hardware that is perfectly healthy.
    if (!wsTele || wsTele.readyState !== WebSocket.OPEN) {
      ctrlLinkPill.textContent = 'ctrl radio: no telemetry link';
      ctrlLinkPill.classList.remove('on');
      return;
    }
    if (!lastAirtimeMs) {
      ctrlLinkPill.textContent = 'ctrl radio: no data';
      ctrlLinkPill.classList.remove('on');
      return;
    }
    const ageS = (performance.now() - lastAirtimeMs) / 1000;
    if (ageS < 15) {
      ctrlLinkPill.textContent = 'ctrl radio: OK';
      ctrlLinkPill.classList.add('on');
    } else {
      ctrlLinkPill.textContent = `ctrl radio: STALE ${Math.round(ageS)}s`;
      ctrlLinkPill.classList.remove('on');
    }
  }, 2000);

  // ----- Image metadata (authoritative /ws/state event stream) -----
  // Image pixels are rendered by img/canvas_renderer.js from `/ws/state`.
  // Keep app.js limited to UI metadata so no secondary renderer can
  // overwrite tile-delta output.
  (() => {
    const cvs  = document.getElementById('image-canvas');
    const meta = document.getElementById('image-meta');
    const speed = document.getElementById('image-link-speed');
    if (!cvs) return;
    let stateFrames = 0;
    let stateLastTs = performance.now();
    window.addEventListener('lifetrac-state', (ev) => {
      stateFrames++;
      const now = performance.now();
      if (now - stateLastTs > 1000) {
        meta.textContent = `${stateFrames} Hz · tile stream`;
        stateFrames = 0;
        stateLastTs = now;
      }
      // 2026-07-24 link-speed panel: RX-side rolling counters published
      // by image_rx_daemon ride /ws/state as snapshot.link_stats.
      const ls = ev.detail && ev.detail.link_stats;
      if (speed && ls && typeof ls.bps === 'number') {
        const bps = ls.bps >= 1024
          ? `${(ls.bps / 1024).toFixed(2)} KB/s`
          : `${ls.bps.toFixed(0)} B/s`;
        const parts = [
          `▲ ${bps}`,
          `${(ls.frames_per_s ?? 0).toFixed(2)} frames/s`,
        ];
        if (typeof ls.rssi_dbm === 'number') {
          parts.push(`RSSI ${ls.rssi_dbm} dBm`);
          // Feed the telemetry-sidebar RSSI row too (it had no writer).
          const rssiRow = document.getElementById('t-rssi');
          if (rssiRow) rssiRow.textContent = `${ls.rssi_dbm} dBm`;
        }
        if (typeof ls.snr_db === 'number') parts.push(`SNR ${ls.snr_db > 0 ? '+' : ''}${ls.snr_db} dB`);
        if (ls.parity_reconstructions > 0) parts.push(`parity saves ${ls.parity_reconstructions}`);
        if (ls.timeouts > 0) parts.push(`⚠ ${ls.timeouts} timeouts`);
        speed.textContent = parts.join(' · ');
        speed.classList.toggle('stale', (Date.now() / 1000 - (ls.ts || 0)) > 10);
      }
    });
  })();

  // ----- Virtual joystick pads (touch + mouse) -----
  const virtualPads = {};
  function syncPadVisuals(fromGamepad) {
    if (virtualPads.l) virtualPads.l.setFromAxes(state.lhx, state.lhy, fromGamepad);
    if (virtualPads.r) virtualPads.r.setFromAxes(state.rhx, state.rhy, fromGamepad);
  }
  function createPad(id, axisX, axisY) {
    const pad = document.getElementById(id);
    const canvas = pad.querySelector('canvas');
    const ctx = canvas.getContext('2d');
    let active = false, cx = 0, cy = 0, hx = 0, hy = 0, radius = 80;

    function resize() {
      const r = pad.getBoundingClientRect();
      canvas.width = r.width; canvas.height = r.height;
      cx = r.width / 2; cy = r.height / 2;
      radius = Math.min(r.width, r.height) * 0.4;
      draw();
    }
    new ResizeObserver(resize).observe(pad);

    function draw() {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.strokeStyle = '#555'; ctx.lineWidth = 2;
      ctx.beginPath(); ctx.arc(cx, cy, radius, 0, Math.PI * 2); ctx.stroke();
      ctx.fillStyle = active ? '#4caf50' : '#888';
      ctx.beginPath(); ctx.arc(cx + hx, cy + hy, 28, 0, Math.PI * 2); ctx.fill();
    }
    function setFromAxes(ax, ay, isActive) {
      if (!isActive) {
        hx = 0;
        hy = 0;
        active = false;
        draw();
        return;
      }
      const nx = Math.max(-1, Math.min(1, (ax || 0) / 127));
      const ny = Math.max(-1, Math.min(1, (ay || 0) / 127));
      hx = nx * radius;
      hy = -ny * radius;
      active = (hx !== 0 || hy !== 0);
      draw();
    }

    function setFromEvent(ev) {
      const r = canvas.getBoundingClientRect();
      const x = (ev.touches ? ev.touches[0].clientX : ev.clientX) - r.left - cx;
      const y = (ev.touches ? ev.touches[0].clientY : ev.clientY) - r.top  - cy;
      const m = Math.min(Math.hypot(x, y), radius);
      const a = Math.atan2(y, x);
      hx = Math.cos(a) * m;
      hy = Math.sin(a) * m;
      // Only emit from on-screen joystick when no gamepad is driving.
      if (!gamepadActive && !controlsLocked) {
        state[axisX] =  Math.round((hx / radius) * 127);
        state[axisY] = -Math.round((hy / radius) * 127);  // y-axis: up = positive
      }
      draw();
    }

    pad.addEventListener('pointerdown', (e) => { active = true; pad.setPointerCapture(e.pointerId); setFromEvent(e); });
    pad.addEventListener('pointermove', (e) => { if (active) setFromEvent(e); });
    pad.addEventListener('pointerup',   () => {
      active = false; hx = hy = 0;
      if (!gamepadActive) { state[axisX] = 0; state[axisY] = 0; }
      draw();
    });
    pad.addEventListener('pointercancel', () => {
      active = false; hx = hy = 0;
      if (!gamepadActive) { state[axisX] = 0; state[axisY] = 0; }
      draw();
    });
    resize();
    return { setFromAxes };
  }
  virtualPads.l = createPad('pad-left',  'lhx', 'lhy');
  virtualPads.r = createPad('pad-right', 'rhx', 'rhy');

  // ----- Button strip -----
  let screenButtons = 0;
  let screenFlags = 0;
  const buttonVisuals = [];
  function syncButtonVisuals(mask) {
    buttonVisuals.forEach(({ el, maskBit }) => {
      el.classList.toggle('active', !!(mask & maskBit));
    });
  }
  document.querySelectorAll('button[data-btn]').forEach(btn => {
    const idx = parseInt(btn.dataset.btn, 10);
    const bit = 1 << idx;
    buttonVisuals.push({ el: btn, maskBit: bit });
    const set = (down) => {
      // Presses are ignored while another source holds control, but
      // releases must ALWAYS clear their bit — swallowing a pointerup
      // during a locked interval latches the bit until the next press.
      if (down && controlsLocked) return;
      if (down) screenButtons |= bit; else screenButtons &= ~bit;
      // Take-control button → also set flags bit0 while held.
      if (parseInt(btn.dataset.btn, 10) === 7) {
        if (down) screenFlags |= 0x01; else screenFlags &= ~0x01;
      }
      if (!gamepadActive) {
        state.buttons = screenButtons;
        state.flags = screenFlags;
        syncButtonVisuals(state.buttons);
      }
    };
    btn.addEventListener('pointerdown', () => set(true));
    btn.addEventListener('pointerup',   () => set(false));
    btn.addEventListener('pointerleave',() => set(false));
  });

  // ----- E-stop dispatch with visible delivery feedback -----
  // /api/estop only proves the request reached web_ui and its local MQTT
  // publish succeeded — the radio hop has no ack path — so report exactly
  // that, and scream when even that much fails. A web_ui restart wipes the
  // in-memory session store, and the resulting 401 used to be totally
  // silent: the operator pressed E-STOP and nothing happened, visibly or
  // otherwise.
  const estopBanner = (() => {
    const el = document.createElement('div');
    el.id = 'estop-banner';
    el.style.position = 'fixed';
    el.style.top = '50px';
    el.style.left = '50%';
    el.style.transform = 'translateX(-50%)';
    el.style.padding = '10px 18px';
    el.style.borderRadius = '8px';
    el.style.font = '700 16px/1.3 monospace';
    el.style.letterSpacing = '0.03em';
    el.style.color = '#fff';
    el.style.zIndex = '999';
    el.style.display = 'none';
    el.style.boxShadow = '0 4px 18px rgba(0,0,0,0.6)';
    document.body.appendChild(el);
    return el;
  })();
  let estopBannerTimer = null;
  function showEstopBanner(text, background, holdMs) {
    estopBanner.textContent = text;
    estopBanner.style.background = background;
    estopBanner.style.display = 'block';
    estopBanner.style.pointerEvents = 'none';
    estopBanner.style.cursor = 'default';
    estopBanner.onclick = null;
    if (estopBannerTimer) { clearTimeout(estopBannerTimer); estopBannerTimer = null; }
    if (holdMs) {
      estopBannerTimer = setTimeout(() => { estopBanner.style.display = 'none'; }, holdMs);
    }
  }
  function fireEstop() {
    showEstopBanner('E-STOP: sending…', '#b36b00', 0);
    fetch('/api/estop', { method: 'POST' })
      .then(async (resp) => {
        const data = await resp.json().catch(() => ({}));
        if (resp.status === 401) {
          showEstopBanner('E-STOP NOT SENT — SESSION EXPIRED — TAP HERE TO LOG IN', '#d62828', 0);
          estopBanner.style.pointerEvents = 'auto';
          estopBanner.style.cursor = 'pointer';
          estopBanner.onclick = () => { window.location = '/login'; };
          return;
        }
        if (resp.ok && data && data.ok !== false && data.delivered !== false) {
          showEstopBanner('E-STOP SENT (radio delivery unconfirmed)', '#2e7d32', 4000);
        } else {
          const why = (data && (data.detail || data.error)) || `HTTP ${resp.status}`;
          showEstopBanner(`E-STOP FAILED — ${why} — USE HARDWARE STOP`, '#d62828', 0);
        }
      })
      .catch(() => {
        showEstopBanner('E-STOP FAILED — NO SERVER LINK — USE HARDWARE STOP', '#d62828', 0);
      });
  }
  document.getElementById('estop').addEventListener('click', fireEstop);

  // ----- Camera switcher -----
  // Sends CMD_CAMERA_SELECT via the web_ui /api/camera/select endpoint, which
  // hands off to lora_bridge.py for the actual LoRa TX. The active-camera
  // echo on topic 0x22 updates the .active class above, so we don't optimistically
  // toggle here — we wait for the tractor to confirm.
  document.querySelectorAll('#camera-switch button').forEach(btn => {
    btn.addEventListener('click', () => {
      if (controlsLocked) return;
      const cam = btn.dataset.cam;
      fetch('/api/camera/select', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ camera: cam }),
      });
    });
  });

  // ----- Bench radio toggles (temporary diagnostics control) -----
  const radioStatus = document.getElementById('radio-bench-status');
  document.querySelectorAll('#radio-bench-switch button[data-radio]').forEach(btn => {
    btn.addEventListener('click', async () => {
      const mode = btn.dataset.radio;
      if (!mode) return;
      if (radioStatus) radioStatus.textContent = `Radio bench: ${mode}...`;
      try {
        const resp = await fetch('/api/bench/radio', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ mode }),
        });
        const data = await resp.json().catch(() => ({}));
        const ok = !!(resp.ok && data && data.ok);
        if (radioStatus) {
          if (ok) {
            radioStatus.textContent = `Radio bench: ${mode} OK`;
          } else {
            // Surface the real reason instead of an opaque "FAIL".
            // 401 → not logged in; 403 → bench disabled via env flag;
            // otherwise show returncode + last line of helper output
            // (or `error` / `detail` if the server attached one).
            let why;
            if (resp.status === 401) {
              why = 'login required (PIN)';
            } else if (resp.status === 403) {
              why = data.detail || 'bench control disabled';
            } else if (data && data.error) {
              why = data.error;
            } else if (data && data.detail) {
              why = data.detail;
            } else if (data && Array.isArray(data.output_tail) && data.output_tail.length) {
              const last = data.output_tail[data.output_tail.length - 1];
              const rc = (data.returncode !== undefined) ? `rc=${data.returncode} ` : '';
              why = `${rc}${last}`;
            } else if (data && data.returncode !== undefined) {
              why = `rc=${data.returncode}`;
            } else {
              why = `HTTP ${resp.status}`;
            }
            radioStatus.textContent = `Radio bench: ${mode} FAIL — ${why}`;
            // Also drop the full payload to the console for deeper triage.
            console.warn('bench radio FAIL', { status: resp.status, body: data });
          }
        }
      } catch (e) {
        if (radioStatus) radioStatus.textContent = `Radio bench: request failed — ${e && e.message ? e.message : e}`;
      }
    });
  });

  // ----- Encoder-mode pill -----
  // Short click cycles operator-selected encode modes by POSTing
  // /api/encode_mode/cycle. Long-press (>=800 ms) jumps to settings.
  // On mode switch we clear the canvas and show a temporary mode label
  // until the first tile of the new stream is painted.
  const encPill = document.getElementById('encode-mode-pill');
  // btc4_* removed 2026-07-26 — no tractor encoder exists; the server
  // catalogue and cycle order no longer offer them.
  const encMeta = {
    full: { label: 'Full color', desc: 'baseline RGB WebP' },
    y_only: { label: 'Y-only WebP', desc: 'luma only, recolor at base' },
    motion_only: { label: 'Motion-only gray', desc: 'pure grayscale with quality cap for bandwidth' },
    mono_g4: { label: 'Mono / G4', desc: '1-bit dither + Group-4 fax' },
  };
  const modeOverlay = (() => {
    const canvas = document.getElementById('image-canvas');
    if (!canvas || !canvas.parentElement) return null;
    const el = document.createElement('div');
    el.id = 'encode-switch-overlay';
    el.style.position = 'absolute';
    el.style.left = '50%';
    el.style.top = '50%';
    el.style.transform = 'translate(-50%, -50%)';
    el.style.padding = '8px 12px';
    el.style.borderRadius = '10px';
    el.style.background = 'rgba(0, 0, 0, 0.55)';
    el.style.border = '1px solid rgba(255, 255, 255, 0.35)';
    el.style.color = '#d5ffd5';
    el.style.font = '700 14px/1.2 monospace';
    el.style.letterSpacing = '0.04em';
    el.style.textTransform = 'uppercase';
    el.style.pointerEvents = 'none';
    el.style.display = 'none';
    canvas.parentElement.appendChild(el);
    return el;
  })();
  let waitingForModeFrame = false;
  let encCurrentMode = 'full';
  // 2026-07-25 confirmation loop: the tractor acks every applied mode on
  // a retained status topic (requested vs effective — catches silent
  // clamps), and reassembled frames self-describe their codec. Both
  // surface through /api/encode_mode/current; render a mismatch instead
  // of trusting the optimistic POST response forever.
  let encTractorAck = null;   // {effective_name, clamped, quality?, ...} | null
  let encQuality = null;      // operator-pinned quality (1-100) | null
  let safeModeActive = false; // link_monitor safe-mode override (bridge plane)
  let lastAirtimeMs = 0;      // performance.now() of last control/link_airtime
  function clearImageForModeSwitch(mode) {
    const canvas = document.getElementById('image-canvas');
    if (canvas) {
      const ctx = canvas.getContext('2d');
      if (ctx) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = '#000';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
      }
    }
    const meta = encMeta[mode] || { label: mode };
    if (modeOverlay) {
      modeOverlay.textContent = `loading: ${meta.label}`;
      modeOverlay.style.display = 'block';
    }
    waitingForModeFrame = true;
  }
  window.addEventListener('lifetrac-tile-painted', () => {
    if (!waitingForModeFrame) return;
    waitingForModeFrame = false;
    if (modeOverlay) modeOverlay.style.display = 'none';
  });
  function _renderEncPill() {
    if (!encPill) return;
    const meta = encMeta[encCurrentMode] || { label: encCurrentMode, desc: '' };
    let text = `enc: ${meta.label} \u25b8`;
    let mismatch = false;
    if (encTractorAck && encTractorAck.effective_name) {
      const eff = encTractorAck.effective_name;
      if (encTractorAck.clamped || (eff !== encCurrentMode)) {
        const effMeta = encMeta[eff] || { label: eff };
        text = `enc: ${meta.label} \u2192 ${effMeta.label} \u25b8`;
        mismatch = true;
      }
    }
    if (safeModeActive) text = 'SAFE MODE \u00b7 ' + text;
    encPill.textContent = text;
    encPill.title = (encTractorAck && encTractorAck.effective_name)
      ? `pinned: ${encCurrentMode} \u00b7 tractor runs: ${encTractorAck.effective_name}`
        + (typeof encTractorAck.quality === 'number'
            ? ` @ q${encTractorAck.quality}` : '')
        + (encTractorAck.clamped ? ' (clamped \u2014 encoder not implemented)' : '')
      : (meta.desc ? `${meta.label} - ${meta.desc}`
                   : `Current encode mode: ${encCurrentMode}`)
        + (encQuality != null ? ` \u00b7 q${encQuality} pinned` : '');
    if (safeModeActive) {
      encPill.title = 'Link monitor engaged SAFE MODE \u2014 the bridge floor-clamps '
        + 'the encode mode regardless of the operator pin. ' + encPill.title;
    }
    encPill.classList.toggle('safe', safeModeActive);
    encPill.classList.toggle('mismatch', mismatch && !safeModeActive);
  }
  function setSafeMode(active) {
    active = !!active;
    if (active === safeModeActive) return;
    safeModeActive = active;
    _renderEncPill();
  }
  function _setEncMode(mode) {
    encCurrentMode = (typeof mode === 'string' && mode) ? mode : 'full';
    _renderEncPill();
  }
  // Seed from the runtime endpoint so pill cycling is session-only, then
  // keep polling: the tractor ack and rx codec arrive asynchronously, and
  // another tab / the gamepad / safe-mode can change the pin under us.
  function refreshEncodeMode() {
    fetch('/api/encode_mode/current')
      .then(r => r.ok ? r.json() : null)
      .then(j => {
        if (!j) return;
        if (j.current) encCurrentMode = j.current;
        encQuality = (typeof j.quality === 'number') ? j.quality : null;
        encTractorAck = j.tractor || null;
        // Server-cached safe mode: the bridge publishes that topic
        // edge-only, so a page loaded mid-safe-mode would never see it
        // from the telemetry socket alone.
        if (typeof j.safe_mode === 'boolean') safeModeActive = j.safe_mode;
        _renderEncPill();
      })
      .catch(() => { /* keep last known state */ });
  }
  refreshEncodeMode();
  setInterval(refreshEncodeMode, 5000);
  function cycleEncodeMode() {
    if (modeOverlay) {
      modeOverlay.textContent = 'switching encode mode...';
      modeOverlay.style.display = 'block';
    }
    waitingForModeFrame = true;
    fetch('/api/encode_mode/cycle', { method: 'POST' })
      .then(async (r) => {
        if (r.status === 401) {
          // Same silent-failure class as the old E-stop handler: a
          // web_ui restart wipes sessions and the pill would just sit
          // there looking like the tractor ignored the command.
          waitingForModeFrame = false;
          if (modeOverlay) {
            modeOverlay.textContent = 'session expired — log in again';
            modeOverlay.style.display = 'block';
            setTimeout(() => { modeOverlay.style.display = 'none'; }, 4000);
          }
          return null;
        }
        return r.ok ? r.json() : null;
      })
      .then(j => {
        if (j && j.mode) {
          _setEncMode(j.mode);
          clearImageForModeSwitch(j.mode);
        } else if (waitingForModeFrame) {
          waitingForModeFrame = false;
          if (modeOverlay) modeOverlay.style.display = 'none';
        }
      })
      .catch(() => {
        waitingForModeFrame = false;
        if (modeOverlay) modeOverlay.style.display = 'none';
      });
  }
  if (encPill) {
    let encPressStart = 0;
    let encLongFired = false;
    const LONG_PRESS_MS = 800;
    let encLongTimer = null;
    const startPress = (ev) => {
      ev.preventDefault();
      encPressStart = performance.now();
      encLongFired = false;
      encLongTimer = setTimeout(() => {
        encLongFired = true;
        window.location = '/settings#encmode';
      }, LONG_PRESS_MS);
    };
    const endPress = (ev) => {
      ev.preventDefault();
      if (encLongTimer) { clearTimeout(encLongTimer); encLongTimer = null; }
      if (encLongFired) return;
      cycleEncodeMode();
    };
    encPill.addEventListener('mousedown',  startPress);
    encPill.addEventListener('mouseup',    endPress);
    encPill.addEventListener('mouseleave', () => {
      if (encLongTimer) { clearTimeout(encLongTimer); encLongTimer = null; }
    });
    encPill.addEventListener('touchstart', startPress, { passive: false });
    encPill.addEventListener('touchend',   endPress,   { passive: false });
  }

  // ----- USB Gamepad -----
  // Edge-detection memory for momentary actions (E-stop, etc.). Without this
  // the 50 Hz poll would re-fire fetch('/api/estop') every 20 ms while the
  // operator holds START — DoSing the server and re-latching the fault.
  const prevButtons = {};
  function pressed(gp, idx) {
    const now = !!gp.buttons[idx]?.pressed;
    const was = !!prevButtons[idx];
    prevButtons[idx] = now;
    return now && !was;   // rising edge only
  }
  const pill = document.getElementById('gamepad-pill');
  function pollGamepad() {
    const gps = navigator.getGamepads ? navigator.getGamepads() : [];
    const gp = Array.from(gps).find(g => g && g.connected);
    if (!gp) {
      if (gamepadActive) {
        gamepadActive = false;
        gamepadButtonsMask = 0;
        pill.textContent = 'no gamepad';
        pill.classList.remove('on');
        for (const k in prevButtons) prevButtons[k] = false;
        state.lhx = state.lhy = state.rhx = state.rhy = 0;
        state.buttons = screenButtons;
        state.flags = screenFlags;
        syncPadVisuals(false);
        syncButtonVisuals(state.buttons);
      }
      return;
    }
    if (!gamepadActive) {
      gamepadActive = true;
      pill.textContent = `gamepad: ${gp.id.split('(')[0].trim()}`;
      pill.classList.add('on');
    }
    const dz = (v) => Math.abs(v) < 0.07 ? 0 : v;
    const clip = (v) => Math.max(-127, Math.min(127, v));
    state.lhx = clip( Math.round(dz(gp.axes[0]) * 127));
    state.lhy = clip(-Math.round(dz(gp.axes[1]) * 127));
    state.rhx = clip( Math.round(dz(gp.axes[2] || 0) * 127));
    state.rhy = clip(-Math.round(dz(gp.axes[3] || 0) * 127));
    syncPadVisuals(true);
    let buttons = 0, flags = 0;
    if (gp.buttons[0]?.pressed) buttons |= (1 << 0);   // A → curl
    if (gp.buttons[1]?.pressed) buttons |= (1 << 1);   // B → dump
    if (gp.buttons[2]?.pressed) buttons |= (1 << 2);   // X → aux1
    if (gp.buttons[3]?.pressed) buttons |= (1 << 3);   // Y → aux2
    if (gp.buttons[5]?.pressed) { buttons |= (1 << 7); flags |= 0x01; } // RB → take-control
    gamepadButtonsMask = buttons;
    // E-stop: rising edge only. Shares the feedback banner with the
    // on-screen button so gamepad E-stops also surface failures.
    if (pressed(gp, 9)) {
      fireEstop();
    }
    // Encoder method cycle: BACK/SELECT (button 8) on standard XInput.
    // Edge-triggered so a long press still cycles exactly once. Mirrors
    // the on-screen pill so a bench operator can sweep codecs without
    // looking at the laptop. (Encoder-cycle plan, step 10.)
    if (pressed(gp, 8)) {
      cycleEncodeMode();
    }
    if (controlsLocked) {
      state.lhx = state.lhy = state.rhx = state.rhy = 0;
      state.buttons = 0;
      state.flags = 0;
      syncPadVisuals(false);
      syncButtonVisuals(0);
      return;
    }
    // Refresh hold-to-track edge memory for the buttons we report by level.
    [0, 1, 2, 3, 5].forEach(i => { prevButtons[i] = !!gp.buttons[i]?.pressed; });
    state.buttons = (buttons | screenButtons) & 0xFFFF;
    state.flags   = (flags | screenFlags) & 0xFF;
    syncButtonVisuals(state.buttons);
  }
  setInterval(pollGamepad, 20);   // 50 Hz
  window.addEventListener('gamepadconnected',    pollGamepad);
  window.addEventListener('gamepaddisconnected', pollGamepad);
})();
