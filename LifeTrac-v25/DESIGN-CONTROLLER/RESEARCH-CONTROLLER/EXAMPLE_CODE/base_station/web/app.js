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
  wsCtrl.addEventListener('open',  () => linkEl.textContent = 'link: connected');
  wsCtrl.addEventListener('close', () => linkEl.textContent = 'link: DISCONNECTED');
  wsCtrl.addEventListener('error', () => linkEl.textContent = 'link: error');

  // 20 Hz tx (server also rate-limits)
  setInterval(() => {
    if (wsCtrl.readyState !== WebSocket.OPEN) return;
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
      if (msg.topic.endsWith('/source_active')) {
        const txt = msg.data || 'NONE';
        document.getElementById('t-src').textContent = txt;
        const banner = document.getElementById('source-banner');
        banner.textContent = `SOURCE: ${txt.toUpperCase()}`;
        banner.className = 'source-banner ' + txt.toLowerCase();
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
      if (!gamepadActive) {
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

  // ----- USB Gamepad -----
  let gamepadActive = false;
  const pill = document.getElementById('gamepad-pill');
  function pollGamepad() {
    const gps = navigator.getGamepads ? navigator.getGamepads() : [];
    const gp = Array.from(gps).find(g => g && g.connected);
    if (!gp) {
      if (gamepadActive) {
        gamepadActive = false;
        pill.textContent = 'no gamepad';
        pill.classList.remove('on');
      }
      return;
    }
    if (!gamepadActive) {
      gamepadActive = true;
      pill.textContent = `gamepad: ${gp.id.split('(')[0].trim()}`;
      pill.classList.add('on');
    }
    const dz = (v) => Math.abs(v) < 0.07 ? 0 : v;
    state.lhx =  Math.round(dz(gp.axes[0]) * 127);
    state.lhy = -Math.round(dz(gp.axes[1]) * 127);
    state.rhx =  Math.round(dz(gp.axes[2] || 0) * 127);
    state.rhy = -Math.round(dz(gp.axes[3] || 0) * 127);
    let buttons = 0, flags = 0;
    if (gp.buttons[0]?.pressed) buttons |= (1 << 0);   // A → curl
    if (gp.buttons[1]?.pressed) buttons |= (1 << 1);   // B → dump
    if (gp.buttons[2]?.pressed) buttons |= (1 << 2);   // X → aux1
    if (gp.buttons[3]?.pressed) buttons |= (1 << 3);   // Y → aux2
    if (gp.buttons[5]?.pressed) { buttons |= (1 << 7); flags |= 0x01; } // RB → take-control
    if (gp.buttons[9]?.pressed) {                                       // START → E-stop
      fetch('/api/estop', { method: 'POST' });
    }
    state.buttons = buttons;
    state.flags   = flags;
  }
  setInterval(pollGamepad, 20);   // 50 Hz
  window.addEventListener('gamepadconnected',    pollGamepad);
  window.addEventListener('gamepaddisconnected', pollGamepad);
})();
