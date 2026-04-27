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
    }
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
    } catch (e) { /* shrug */ }
  });
  setInterval(() => {
    if (wsTele.readyState === WebSocket.OPEN) wsTele.send('ping');
  }, 5000);

  // ----- Virtual joystick pads (touch + mouse) -----
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
  }
  createPad('pad-left',  'lhx', 'lhy');
  createPad('pad-right', 'rhx', 'rhy');

  // ----- Button strip -----
  document.querySelectorAll('.buttons button[data-btn]').forEach(btn => {
    const bit = 1 << parseInt(btn.dataset.btn, 10);
    const set = (down) => {
      if (controlsLocked) return;
      if (down) state.buttons |= bit; else state.buttons &= ~bit;
      // Take-control button → also set flags bit0 while held.
      if (parseInt(btn.dataset.btn, 10) === 7) {
        if (down) state.flags |= 0x01; else state.flags &= ~0x01;
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

  // ----- USB Gamepad -----
  let gamepadActive = false;
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
        pill.textContent = 'no gamepad';
        pill.classList.remove('on');
        for (const k in prevButtons) prevButtons[k] = false;
        state.lhx = state.lhy = state.rhx = state.rhy = 0;
        state.buttons = 0;
        state.flags = 0;
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
    let buttons = 0, flags = 0;
    if (gp.buttons[0]?.pressed) buttons |= (1 << 0);   // A → curl
    if (gp.buttons[1]?.pressed) buttons |= (1 << 1);   // B → dump
    if (gp.buttons[2]?.pressed) buttons |= (1 << 2);   // X → aux1
    if (gp.buttons[3]?.pressed) buttons |= (1 << 3);   // Y → aux2
    if (gp.buttons[5]?.pressed) { buttons |= (1 << 7); flags |= 0x01; } // RB → take-control
    // E-stop: rising edge only.
    if (pressed(gp, 9)) {
      fetch('/api/estop', { method: 'POST' });
    }
    if (controlsLocked) {
      state.lhx = state.lhy = state.rhx = state.rhy = 0;
      state.buttons = 0;
      state.flags = 0;
      return;
    }
    // Refresh hold-to-track edge memory for the buttons we report by level.
    [0, 1, 2, 3, 5].forEach(i => { prevButtons[i] = !!gp.buttons[i]?.pressed; });
    state.buttons = buttons & 0xFFFF;
    state.flags   = flags   & 0xFF;
  }
  setInterval(pollGamepad, 20);   // 50 Hz
  window.addEventListener('gamepadconnected',    pollGamepad);
  window.addEventListener('gamepaddisconnected', pollGamepad);
})();
