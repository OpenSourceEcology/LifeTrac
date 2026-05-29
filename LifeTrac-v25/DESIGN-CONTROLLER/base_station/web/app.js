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
  const wsCtrl = new WebSocket(`ws://${location.host}/ws/control`);
  const wsTele = new WebSocket(`ws://${location.host}/ws/telemetry`);
  const linkEl = document.getElementById('link-status');
  let controlsLocked = false;

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
    if ('u_telemetry' in data) document.getElementById('t-loss').textContent = `telem ${pct(data.u_telemetry)}`;
    if ('encode_mode' in data) document.getElementById('t-encode').textContent = String(data.encode_mode).toLowerCase();
  }

  wsCtrl.addEventListener('open',  () => linkEl.textContent = 'link: connected');
  wsCtrl.addEventListener('close', () => linkEl.textContent = 'link: DISCONNECTED');
  wsCtrl.addEventListener('error', () => linkEl.textContent = 'link: error');

  // 20 Hz tx (server also rate-limits)
  setInterval(() => {
    if (wsCtrl.readyState !== WebSocket.OPEN) return;
    if (controlsLocked) return;
    wsCtrl.send(JSON.stringify(state));
  }, 50);

  // ----- Telemetry sidebar -----
  wsTele.addEventListener('message', (ev) => {
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
      if (msg.topic.endsWith('/status/link') || (msg.topic.endsWith('/source_active') && msg.data && typeof msg.data === 'object')) {
        setLinkStatus(msg.data);
      }
      if (msg.topic.endsWith('/source_active')) {
        const hasSourceField = msg.data && typeof msg.data === 'object'
          && ('active_source' in msg.data || 'source' in msg.data);
        if (typeof msg.data !== 'object' || hasSourceField) setSource(msg.data);
      }
      // Encoder-cycle plan step 11: safe-mode floor banner. lora_bridge
      // publishes {"active": bool, "since_ms": int} retained on this
      // topic; we surface as a red flash on the encoder pill so the
      // operator sees the override the moment the link degrades.
      if (msg.topic.endsWith('/control/safe_mode_active')) {
        const active = !!(msg.data && msg.data.active);
        setSafeMode(active);
      }
    } catch (e) { /* shrug */ }
  });
  setInterval(() => {
    if (wsTele.readyState === WebSocket.OPEN) wsTele.send('ping');
  }, 5000);

  // ----- Image metadata (authoritative /ws/state event stream) -----
  // Image pixels are rendered by img/canvas_renderer.js from `/ws/state`.
  // Keep app.js limited to UI metadata so no secondary renderer can
  // overwrite tile-delta output.
  (() => {
    const cvs  = document.getElementById('image-canvas');
    const meta = document.getElementById('image-meta');
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
      if (controlsLocked) return;
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

  document.getElementById('estop').addEventListener('click', () => {
    fetch('/api/estop', { method: 'POST' });
  });

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

  // ----- Encoder-mode pill (encoder-cycle plan, steps 9 + 10 + 11) -----
  // Short click cycles through the narrow on-bench codec subset by
  // POSTing /api/encode_mode/cycle. Long-press (>=800 ms) jumps to the
  // full settings page. The pill text mirrors the persisted operator
  // ceiling, and a safe-mode flag (published retained on
  // lifetrac/v25/control/safe_mode_active) replaces the label with a
  // red flashing "SAFE: mono_g4" until the link recovers.
  const encPill = document.getElementById('encode-mode-pill');
  let encSafeMode = false;
  let encCurrentMode = 'auto';
  function _renderEncPill() {
    if (!encPill) return;
    if (encSafeMode) {
      encPill.textContent = 'SAFE: mono_g4';
      encPill.classList.add('safe');
    } else {
      encPill.textContent = `enc: ${encCurrentMode} \u25B8`;
      encPill.classList.remove('safe');
    }
  }
  function setSafeMode(active) {
    if (encSafeMode === active) return;
    encSafeMode = active;
    _renderEncPill();
  }
  function _setEncMode(mode) {
    encCurrentMode = (typeof mode === 'string' && mode) ? mode : 'auto';
    _renderEncPill();
  }
  // Seed from the persisted-override endpoint on page load.
  fetch('/api/settings/encode_mode')
    .then(r => r.ok ? r.json() : null)
    .then(j => { if (j && j.current) _setEncMode(j.current); })
    .catch(() => { /* leave default */ });
  function cycleEncodeMode() {
    fetch('/api/encode_mode/cycle', { method: 'POST' })
      .then(r => r.ok ? r.json() : null)
      .then(j => { if (j && j.mode) _setEncMode(j.mode); })
      .catch(() => { /* shrug */ });
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
    // E-stop: rising edge only.
    if (pressed(gp, 9)) {
      fetch('/api/estop', { method: 'POST' });
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
