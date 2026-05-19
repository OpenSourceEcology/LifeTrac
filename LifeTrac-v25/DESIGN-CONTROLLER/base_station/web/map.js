// map.js — Live GPS marker + offline-first tiles for LifeTrac v25.
//
// Reads GPS + link telemetry from /ws/state. Tile source priority:
//
//   1. Offline pyramid at /tiles/{z}/{x}/{y}.png (served by nginx from
//      /opt/lifetrac/tiles/) — preferred so a base-station with no
//      internet still has a map.
//   2. OpenStreetMap public tile server — fallback only when offline
//      tiles 404 *and* the page detects connectivity.
//
// Server is authoritative: we display state.gps verbatim and never
// invent a position when the server hasn't published one (fail-closed
// per IMAGE_PIPELINE.md §6.1).
(() => {
  "use strict";

  // Default to the OFAS workshop coordinates so an empty map still has
  // a sensible center; replaced as soon as the first GPS fix arrives.
  const HOME_LATLON = [37.7749, -122.4194];
  const STALE_FIX_MS = 5_000;
  const NO_FIX_MS = 30_000;

  const map = L.map("map", {
    zoomControl: true,
    attributionControl: false,
  }).setView(HOME_LATLON, 15);

  const offlineLayer = L.tileLayer("/tiles/{z}/{x}/{y}.png", {
    minZoom: 5, maxZoom: 18,
    errorTileUrl: "",  // blank when missing → caller may swap in OSM
  });
  const onlineLayer = L.tileLayer(
    "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    { minZoom: 5, maxZoom: 19 });

  let usingOnline = false;
  offlineLayer.on("tileerror", () => {
    if (usingOnline) return;
    if (!navigator.onLine) return;
    usingOnline = true;
    map.removeLayer(offlineLayer);
    onlineLayer.addTo(map);
  });
  offlineLayer.addTo(map);

  const tractorIcon = L.divIcon({
    className: "tractor-marker",
    html: '<div style="width:14px;height:14px;border-radius:50%;'
        + 'background:#4caf50;border:2px solid #fff;box-shadow:0 0 4px #000;"></div>',
    iconSize: [18, 18], iconAnchor: [9, 9],
  });
  let marker = null;
  let track = L.polyline([], { color: "#4ea1ff", weight: 2, opacity: 0.7 }).addTo(map);
  let lastFixTs = 0;
  let lastFixLatLon = null;
  let firstFix = true;

  function setText(id, text, cls) {
    const el = document.getElementById(id);
    if (!el) return;
    el.textContent = text;
    if (cls !== undefined) {
      el.classList.remove("ok", "warn", "bad", "stale", "nofix");
      if (cls) el.classList.add(cls);
    }
  }

  function fmtLatLon(v) {
    if (v == null || isNaN(v)) return "—";
    return v.toFixed(6) + "°";
  }

  function fmtRssi(dbm) {
    if (dbm == null || isNaN(dbm)) return "—";
    return Math.round(dbm) + " dBm";
  }

  // Coarse range estimate from RSSI using free-space path loss for the
  // 915 MHz ISM band, plus a 6 dB tractor-loss + 3 dB rain margin
  // (IMAGE_PIPELINE.md Appendix B). This is intentionally conservative;
  // operators must NOT rely on it for safety decisions.
  function estimateRangeMeters(rssiDbm) {
    if (rssiDbm == null || isNaN(rssiDbm)) return null;
    const txDbm = 14;             // legal CONUS ISM
    const txGain = 2.15;          // tractor whip
    const rxGain = 6.0;           // base mast
    const margin = 9.0;           // tractor + rain
    const fMHz = 915;
    const pathLoss = txDbm + txGain + rxGain - margin - rssiDbm;
    if (pathLoss <= 0) return null;
    // FSPL: PL(dB) = 20 log10(d_km) + 20 log10(f_MHz) + 32.44
    const dKm = Math.pow(10, (pathLoss - 20 * Math.log10(fMHz) - 32.44) / 20);
    return Math.max(0, dKm * 1000);
  }

  // S7.1 LINK pill helpers. Pure formatters — no derived safety state is
  // computed here (per IMAGE_PIPELINE.md §6.1, browser only renders what
  // the server vouches for). Inputs are the per-direction dicts from
  // `state.link_power.{uplink,downlink}` (schema per S7.2:
  // {state, action, value, reason, power_dbm, sf_rung, snr_ewma, per,
  //  per_sample_count}).
  //
  // Color map matches the operator-console IMG/AI pill convention:
  //   NORMAL          → ok    (green)
  //   RECOVERY        → warn  (orange)
  //   MARGIN_LIMITED  → bad   (red) — power-saturated or below SNR floor
  //   AIRTIME_LIMITED → bad   (red) — adapter is shedding P3/P2/encode
  //   unknown / null  → grey (pill default)
  function pillClassForState(adapterState) {
    if (adapterState === "NORMAL") return "ok";
    if (adapterState === "RECOVERY") return "warn";
    if (adapterState === "MARGIN_LIMITED") return "bad";
    if (adapterState === "AIRTIME_LIMITED") return "bad";
    return "";
  }
  function fmtDbm(v) {
    if (v == null || isNaN(v)) return "—";
    return (v >= 0 ? "+" : "") + Math.round(v) + " dBm";
  }
  function fmtSuccessPct(per) {
    // PER is fraction in [0,1]; UX spec shows *success* % (1 - PER).
    if (per == null || isNaN(per)) return "—";
    return (100 * (1 - per)).toFixed(1) + "%";
  }
  function fmtSnr(s) {
    if (s == null || isNaN(s)) return "—";
    return "SNR " + (s >= 0 ? "+" : "") + s.toFixed(1) + " dB";
  }
  function fmtSf(rung) {
    // SF rung 0 == SF7 (fastest), each rung up == +1 SF (slower / longer
    // range). LoRa caps at SF12 per LORA_PROTOCOL.md.
    if (rung == null || isNaN(rung)) return "SF?";
    return "SF" + (7 + Math.max(0, Math.min(5, rung | 0)));
  }
  function renderLinkPillOne(pillId, snap) {
    const el = document.getElementById(pillId);
    if (!el) return;
    el.classList.remove("ok", "warn", "bad");
    if (!snap || typeof snap !== "object") {
      el.textContent = "—";
      return;
    }
    const cls = pillClassForState(snap.state);
    if (cls) el.classList.add(cls);
    el.textContent = "LINK: "
      + fmtSf(snap.sf_rung) + " / "
      + fmtDbm(snap.power_dbm) + " / "
      + fmtSuccessPct(snap.per) + " / "
      + fmtSnr(snap.snr_ewma);
  }
  function renderLinkPill(linkPower) {
    renderLinkPillOne("link-pill-up", linkPower.uplink);
    renderLinkPillOne("link-pill-dn", linkPower.downlink);
    // Worst-side reason wins: prefer uplink unless it's NORMAL/null and
    // downlink has something to say. Mirrors the IMG-pill convention of
    // surfacing the most-degraded subsystem's reason string.
    const up = linkPower.uplink || {};
    const dn = linkPower.downlink || {};
    const upActive = up.state && up.state !== "NORMAL";
    const reason = upActive ? (up.reason || "") : (dn.reason || up.reason || "");
    setText("link-reason", reason || "—");
  }

  function applyState(state) {
    if (!state || typeof state !== "object") return;

    const gps = state.gps || null;
    const now = state.ts_ms || Date.now();

    if (gps && typeof gps.lat === "number" && typeof gps.lon === "number" &&
        gps.fix !== false) {
      lastFixTs = now;
      lastFixLatLon = [gps.lat, gps.lon];
      if (marker == null) {
        marker = L.marker(lastFixLatLon, { icon: tractorIcon }).addTo(map);
      } else {
        marker.setLatLng(lastFixLatLon);
      }
      track.addLatLng(lastFixLatLon);
      // Cap the trail so it doesn't grow unbounded.
      const latlngs = track.getLatLngs();
      if (latlngs.length > 5000) track.setLatLngs(latlngs.slice(-5000));
      if (firstFix) { map.setView(lastFixLatLon, 17); firstFix = false; }
      setText("gps-fix", "fix", "ok");
      setText("gps-lat", fmtLatLon(gps.lat));
      setText("gps-lon", fmtLatLon(gps.lon));
      setText("gps-hdop", gps.hdop != null ? gps.hdop.toFixed(2) : "—");
      setText("gps-sats", gps.sats != null ? String(gps.sats) : "—");
      setText("gps-speed",
        gps.speed_mps != null ? (gps.speed_mps * 3.6).toFixed(1) + " km/h" : "—");
    }

    const link = state.link || {};
    setText("link-rssi", fmtRssi(link.rssi_dbm));
    setText("link-snr", link.snr_db != null ? link.snr_db.toFixed(1) + " dB" : "—");
    const r = estimateRangeMeters(link.rssi_dbm);
    if (r == null) {
      setText("link-range", "—");
    } else if (r >= 1000) {
      setText("link-range", (r / 1000).toFixed(2) + " km");
    } else {
      setText("link-range", Math.round(r) + " m");
    }
    setText("link-mode", link.encode_mode || "—",
      link.encode_mode === "FULL" ? "ok" :
      link.encode_mode === "WIREFRAME" ? "bad" :
      link.encode_mode ? "warn" : "");

    // S7.1 LINK pill renderer. `state.link_power` is the per-direction
    // snapshot published by lora_bridge._airtime_worker (S6.2 host half)
    // and forwarded by StatePublisher (S7.2 schema:
    // {state, action, value, reason, power_dbm, sf_rung, snr_ewma, per,
    //  per_sample_count}). All formatting is local and pure — no derived
    // safety state is computed in the browser per IMAGE_PIPELINE.md §6.1.
    renderLinkPill(state.link_power || {});
  }

  function ageSweep() {
    if (lastFixTs === 0) return;
    const age = Date.now() - lastFixTs;
    let cls = "ok";
    if (age > NO_FIX_MS) cls = "nofix";
    else if (age > STALE_FIX_MS) cls = "stale";
    setText("gps-age", age < 1000 ? Math.round(age) + " ms"
                                  : (age / 1000).toFixed(1) + " s", cls);
    if (cls === "nofix") setText("gps-fix", "no fix", "nofix");
    else if (cls === "stale") setText("gps-fix", "stale", "stale");
  }

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

  setInterval(ageSweep, 500);
  connect();
})();
