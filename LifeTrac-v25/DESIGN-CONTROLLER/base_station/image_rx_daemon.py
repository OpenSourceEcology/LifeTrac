"""Base-side image-over-LoRa RX daemon (Phase C of strict-path migration).

Runs on the base Portenta X8 (board serial 2D0A1209DABC240B on the
2026-05-24 bench). Opens the Murata CMWX1ZZABZ-078 L072 ``HostLink`` on
``/dev/ttymxc3``, drains ``RX_FRAME_URC`` (0x91) frames in a tight loop,
feeds each radio payload into the shared
``image_pipeline.reassemble.FragmentReassembler``, and on every
completed frame publishes the reassembled (re-encoded) TileDeltaFrame
bytes to the Windows-host MQTT broker on
``lifetrac/v25/video/tile_delta``.

This is the pure RX-side complement of ``image_tx_daemon.py`` and shares
its bridge-bypass design (no LoRa-frame envelope, no AES-GCM, no
audit/nonce). The RX daemon's MQTT publish replicates exactly what
``lora_bridge.py`` would have published when receiving topic id 0x25
(``TOPIC_BY_ID[0x25] = 'lifetrac/v25/video/tile_delta'``) — so
``web_ui.py`` can consume the strict path without any changes once
Phase D cutover lands.

Topology (Phase C end state):

    L072 RX (SX1276 air)
        → HostLink RX_FRAME_URC (this daemon)
        → FragmentReassembler.feed(raw)
        → encode_tile_delta_frame(frame)
        → mosquitto.publish('lifetrac/v25/video/tile_delta', bytes)
        → web_ui.py /ws/state (Windows host, reached via adb reverse
                                tcp:1883 tcp:1883)

Operational notes:
  * `adb reverse tcp:1883 tcp:1883` must be active on the host PC
    pointing at this device serial so the X8 can reach the Windows
    mosquitto. Without it the daemon will publish into the void.
  * On a malformed fragment / decode error the reassembler increments
    its own ``stats.decode_errors`` and we DO NOT crash. Per
    ``image_pipeline/reassemble.py`` semantics, a decode error implies
    the producer should send a keyframe — that's handled out-of-band by
    web_ui's existing keyframe-request path (``cmd/req_keyframe``); we
    just log it.
  * The L072 HostLink is single-owner. Nothing else on this device
    should be talking to /dev/ttymxc3 while this daemon runs (no
    parallel method_h probes, no stage1 quant runs targeting 2D0A).
"""
from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import threading
import time
from dataclasses import dataclass
from typing import Optional

# ---- repo imports ----------------------------------------------------
_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT_BASE = _HERE                                      # base_station/
_REPO_ROOT_FIRMWARE = os.path.abspath(
    os.path.join(_HERE, "..", "firmware"))
_X8_HELPER = os.path.join(_REPO_ROOT_FIRMWARE, "x8_lora_bootloader_helper")
for _p in (_REPO_ROOT_BASE, _X8_HELPER, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from lora_proto import PHY_IMAGE_BW250, PHY_IMAGE_BW500  # noqa: E402
from image_pipeline.frame_format import encode_tile_delta_frame  # noqa: E402
from image_pipeline.reassemble import FragmentReassembler         # noqa: E402
from method_h_stage2_tx_probe_v2 import (  # noqa: E402
    HostLink,
    HOST_TYPE_VER_REQ,
    HOST_TYPE_VER_URC,
    HOST_TYPE_RX_FRAME_URC,
    parse_rx_frame,
    drain_boot,
    drain_pending,
    configure_regulatory_profile_if_needed,
    verify_modem_matches_profile,
    read_reg,
    write_reg,
    SX1276_REG_OP_MODE,
    SX1276_OPMODE_LORA_RXCONT,
)

LOG = logging.getLogger("image_rx_daemon")

# Same topic ID 0x25 → "lifetrac/v25/video/tile_delta" as the production
# bridge would have used. We publish into the same MQTT slot so web_ui
# does not have to change.
MQTT_TOPIC_OUT = "lifetrac/v25/video/tile_delta"
KEYFRAME_REQ_TOPIC = "lifetrac/v25/cmd/req_keyframe"   # camera_service listens
# 2026-07-24: rolling RX-side link-speed sample for the web UI's image
# panel (web_ui forwards it into /ws/state as snapshot.link_stats).
LINK_STATS_TOPIC = "lifetrac/v25/video/link_stats"
LINK_STATS_INTERVAL_S = 2.0

_PROFILE_TO_PHY = {0: PHY_IMAGE_BW250, 1: PHY_IMAGE_BW250, 2: PHY_IMAGE_BW500}

# MQTT defaults; assume `adb reverse tcp:1883 tcp:1883` is wired so the
# X8 can reach the Windows host mosquitto via localhost.
DEFAULT_MQTT_HOST = "127.0.0.1"
DEFAULT_MQTT_PORT = 1883

DEFAULT_UART = "/dev/ttymxc3"
DEFAULT_BAUD = "921600"

# How long to wait per HostLink poll cycle when no frame is present.
RX_POLL_TIMEOUT_S = 0.25


class KeyframeRequester:
    """Rate-limited req_keyframe on reassembly failure. Publishes to the
    same topic web_ui's manual button uses, so it inherits whatever
    broker relay the deployment already has for that path."""

    def __init__(self, client_supplier, min_interval_s: float = 5.0):
        self._client_supplier = client_supplier
        self._min = min_interval_s
        self._last = 0.0

    def poke(self, reason: str) -> None:
        now = time.monotonic()
        client = self._client_supplier()
        if client is None or now - self._last < self._min:
            return
        self._last = now
        LOG.info("requesting keyframe (%s)", reason)
        try:
            client.publish(KEYFRAME_REQ_TOPIC, b"\x01", qos=0)
        except Exception as exc:
            LOG.warning("failed to publish keyframe request: %s", exc)


@dataclass
class _Stats:
    rx_frames_seen: int = 0
    rx_decode_errors: int = 0
    reassembled_frames_published: int = 0
    publish_errors: int = 0
    reassembler_decode_errors: int = 0
    reassembler_timeouts: int = 0


class ImageRxDaemon:
    """L072 RX → reassemble → MQTT publish.

    Single thread owns the L072 HostLink and the reassembler; an MQTT
    background loop handles publish I/O.
    """

    def __init__(self, *, uart: str, baud: str,
                 mqtt_host: str, mqtt_port: int,
                 reassembler_timeout_ms: int) -> None:
        self.uart = uart
        self.baud = baud
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.reassembler = FragmentReassembler(timeout_ms=reassembler_timeout_ms)
        self._stop = threading.Event()
        self.stats = _Stats()
        self._lock = threading.Lock()
        self._client = None
        self._kf_req = KeyframeRequester(lambda: self._client)
        # 2026-07-24 link-speed telemetry (read by _link_stats_worker).
        self._air_bytes = 0                  # on-air payload bytes received
        self._last_rssi_dbm = None
        self._last_snr_db = None

    def _open_link(self) -> HostLink:
        LOG.info("opening L072 HostLink on %s @ %s", self.uart, self.baud)
        link = HostLink(self.uart, self.baud)
        # 2026-05-25 W2-02 v3 fix: when an external NRST (gpio163 pulse)
        # has already reset the L072, sending another UART RESET_REQ on
        # top of the just-booted firmware races the boot sequence and
        # causes VER warm-up to time out (1/3 success at 1.5s settle vs
        # 3/3 when this is skipped — see rx_ver_warmup_diag.py sweep
        # 2026-05-25_rx_ver_sweep_2D0A1209DABC240B_064033.log). Opt-in
        # via env so default behaviour is unchanged for callers that
        # rely on the daemon to reset the MCU itself.
        import os as _os
        skip_reset = _os.environ.get(
            "LIFETRAC_SKIP_RESET_REQ", "0") not in ("0", "", "false", "False")
        if not skip_reset:
            try:
                link.send(0x03)  # HOST_TYPE_RESET_REQ
                drain_boot(link, settle_s=1.5)
            except Exception as exc:                          # pragma: no cover
                LOG.warning("L072 reset failed: %s (continuing)", exc)
        else:
            LOG.info("LIFETRAC_SKIP_RESET_REQ=1 — relying on external NRST; "
                     "draining boot chatter only")
            try:
                drain_boot(link, settle_s=0.25)
            except Exception as exc:                          # pragma: no cover
                LOG.warning("post-NRST drain failed: %s (continuing)", exc)
        try:
            link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
            LOG.info("L072 VER warm-up ok")
        except Exception as exc:
            LOG.error("VER warm-up failed: %s", exc)
            raise
        drain_pending(link, quiet_s=0.25, max_s=1.0)
        try:
            configure_regulatory_profile_if_needed(link)
        except Exception as exc:                              # pragma: no cover
            LOG.warning("regulatory profile config failed: %s", exc)
        if _os.environ.get("LIFETRAC_SKIP_PHY_CONTRACT", "0") != "1":
            prof_id = int(_os.environ.get("LIFETRAC_REG_PROFILE", "0"))
            active_phy = _PROFILE_TO_PHY.get(prof_id, PHY_IMAGE_BW250)
            verify_modem_matches_profile(link, active_phy)
        # 2026-05-25 fix: explicitly wake the SX1276 into LORA_RXCONTINUOUS
        # (RegOpMode=0x85). The firmware does NOT auto-enter RXCONT after
        # boot — prior probes (and any prior TX-side cleanup) leave the
        # radio in LORA_SLEEP (0x80). This mirrors the autowake folded
        # into method_h_stage2_tx_probe_v2.rx_listen and the standalone
        # w2_02_radio_wake_rxcont.py helper. Without this, rx_frames stays
        # at 0 even with a working TX peer on air.
        try:
            opm_pre, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
            if opm_pre != SX1276_OPMODE_LORA_RXCONT:
                write_reg(link, SX1276_REG_OP_MODE,
                          SX1276_OPMODE_LORA_RXCONT, timeout=0.5)
                opm_post, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
                LOG.info("SX1276 RXCONT autowake: opmode 0x%02X -> 0x%02X",
                         opm_pre, opm_post)
            else:
                LOG.info("SX1276 already in RXCONT (opmode=0x%02X)", opm_pre)
        except Exception as exc:                              # pragma: no cover
            LOG.warning("SX1276 RXCONT autowake failed: %s (continuing)", exc)
        return link

    def _publish(self, payload: bytes, frame_id: int) -> None:
        if self._client is None:
            with self._lock:
                self.stats.publish_errors += 1
            LOG.warning("publish skipped: MQTT client not initialised "
                        "(frame_id=%d, %d B)", frame_id, len(payload))
            return
        try:
            info = self._client.publish(MQTT_TOPIC_OUT, payload, qos=0,
                                        retain=False)
            # Don't block on info.wait_for_publish() — keep the L072
            # drain loop hot. paho-mqtt loops MQTT I/O on its own thread.
            with self._lock:
                self.stats.reassembled_frames_published += 1
            LOG.info("published frame_id=%d %d B → %s",
                     frame_id, len(payload), MQTT_TOPIC_OUT)
        except Exception as exc:
            with self._lock:
                self.stats.publish_errors += 1
            LOG.warning("publish failed for frame_id=%d: %s", frame_id, exc)

    def _rx_worker(self) -> None:
        try:
            link = self._open_link()
        except Exception as exc:
            LOG.error("fatal: cannot open L072 HostLink: %s", exc)
            self._stop.set()
            return
        LOG.info("RX worker ready; draining RX_FRAME_URC...")

        # Track reassembler stats so we can detect decode errors / timeouts
        # that happen mid-flight (the public ReassemblyStats counters
        # monotonically increase).
        last_decode_errors = self.reassembler.stats.decode_errors
        last_timeouts = self.reassembler.stats.timeouts

        while not self._stop.is_set():
            # HostLink.read_frames(timeout) returns a list of frame dicts
            # (possibly empty). Each dict has 'type', 'flags', 'seq',
            # 'payload' keys; see method_g_stage1_probe.parse_frame().
            try:
                frames = link.read_frames(timeout=RX_POLL_TIMEOUT_S)
            except Exception as exc:                          # pragma: no cover
                LOG.warning("HostLink read_frames error: %s", exc)
                time.sleep(0.1)
                continue
            if not frames:
                self.reassembler.tick()
                cur_timeout = self.reassembler.stats.timeouts
                if cur_timeout != last_timeouts:
                    with self._lock:
                        self.stats.reassembler_timeouts += (cur_timeout - last_timeouts)
                    last_timeouts = cur_timeout
                    self._kf_req.poke(f"reassembly timeout #{cur_timeout}")
                continue

            for frame in frames:
                ftype = frame.get("type")
                if ftype != HOST_TYPE_RX_FRAME_URC:
                    # FAULT_URC / STATS / TX_DONE etc — not our concern
                    # in the RX-only daemon. Log at DEBUG.
                    LOG.debug("ignoring host frame type=0x%02x", ftype or 0)
                    continue

                try:
                    parsed = parse_rx_frame(frame["payload"])
                except Exception as exc:
                    with self._lock:
                        self.stats.rx_decode_errors += 1
                    LOG.warning("RX_FRAME_URC parse error: %s raw=%s",
                                exc, frame["payload"].hex())
                    continue

                data: bytes = parsed.get("payload", b"")
                with self._lock:
                    self.stats.rx_frames_seen += 1
                    # 2026-07-24 link-speed telemetry: rolling counters
                    # consumed by _link_stats_worker → /video/link_stats.
                    self._air_bytes += len(data)
                    if parsed.get("rssi_dbm") is not None:
                        self._last_rssi_dbm = parsed["rssi_dbm"]
                    if parsed.get("snr_db") is not None:
                        self._last_snr_db = parsed["snr_db"]
                LOG.debug(
                    "RX_FRAME_URC #%d len=%d snr=%s rssi=%s payload_head=%s",
                    self.stats.rx_frames_seen, len(data),
                    parsed.get("snr_db"), parsed.get("rssi_dbm"),
                    data[:8].hex())

                # Feed into reassembler. Returns a TileDeltaFrame on
                # completion, None while in flight or on decode error.
                try:
                    completed = self.reassembler.feed(data)
                except Exception as exc:                      # pragma: no cover
                    LOG.warning("reassembler.feed crashed: %s data=%s",
                                exc, data.hex())
                    completed = None

                # Mirror reassembler counters into our stats.
                with self._lock:
                    cur_decode = self.reassembler.stats.decode_errors
                    cur_timeout = self.reassembler.stats.timeouts
                    if cur_decode != last_decode_errors:
                        self.stats.reassembler_decode_errors += (cur_decode - last_decode_errors)
                        last_decode_errors = cur_decode
                        self._kf_req.poke(f"decode error #{cur_decode}")
                    if cur_timeout != last_timeouts:
                        self.stats.reassembler_timeouts += (cur_timeout - last_timeouts)
                        last_timeouts = cur_timeout
                        self._kf_req.poke(f"reassembly timeout #{cur_timeout}")

                if completed is not None:
                    try:
                        payload_out = encode_tile_delta_frame(completed)
                    except Exception as exc:
                        LOG.warning("encode_tile_delta_frame failed: %s", exc)
                        continue
                    frame_id = getattr(completed, "frame_id", 0) or 0
                    self._publish(payload_out, frame_id)

        LOG.info("RX worker exit")

    def _stats_worker(self, interval_s: float) -> None:
        last = 0.0
        while not self._stop.is_set():
            time.sleep(1.0)
            now = time.monotonic()
            if now - last < interval_s:
                continue
            last = now
            with self._lock:
                s = self.stats
                LOG.info(
                    "stats: rx_frames=%d rx_decode_err=%d "
                    "frames_published=%d publish_err=%d "
                    "reassembler_decode_err=%d reassembler_timeouts=%d",
                    s.rx_frames_seen, s.rx_decode_errors,
                    s.reassembled_frames_published, s.publish_errors,
                    s.reassembler_decode_errors, s.reassembler_timeouts)

    def _link_stats_worker(self) -> None:
        """Publish a rolling link-speed JSON sample every ~2 s.

        ``bps`` counts ON-AIR payload bytes received (fragment headers
        included) — the honest received-throughput number; the web UI
        renders it under the image canvas. QoS 0, best-effort: a missed
        sample just leaves the previous one on screen (the UI greys the
        line once samples stop for >10 s).
        """
        import json as _json
        last_bytes = 0
        last_frames = 0
        last_t = time.monotonic()
        while not self._stop.is_set():
            time.sleep(LINK_STATS_INTERVAL_S)
            now = time.monotonic()
            elapsed = max(now - last_t, 1e-3)
            last_t = now
            with self._lock:
                cur_bytes = self._air_bytes
                cur_frames = self.stats.reassembled_frames_published
                sample = {
                    "bps": round((cur_bytes - last_bytes) / elapsed, 1),
                    "frames_per_s": round((cur_frames - last_frames) / elapsed, 2),
                    "rx_frames_seen": self.stats.rx_frames_seen,
                    "frames_published": cur_frames,
                    "rssi_dbm": self._last_rssi_dbm,
                    "snr_db": self._last_snr_db,
                    "parity_reconstructions":
                        self.reassembler.stats.parity_reconstructions,
                    "timeouts": self.stats.reassembler_timeouts,
                    "decode_errors": self.stats.reassembler_decode_errors,
                    "ts": round(time.time(), 1),
                }
            last_bytes = cur_bytes
            last_frames = cur_frames
            client = self._client
            if client is None:
                continue
            try:
                client.publish(LINK_STATS_TOPIC,
                               _json.dumps(sample).encode("utf-8"), qos=0)
            except Exception as exc:                          # pragma: no cover
                LOG.debug("link_stats publish failed: %s", exc)

    def run(self, *, stats_interval_s: float) -> int:
        import paho.mqtt.client as mqtt

        self._client = mqtt.Client(
            client_id=f"lifetrac-image-rx-{os.getpid()}")

        def _on_connect(_c, _u, _f, rc):
            if rc == 0:
                LOG.info("MQTT connected; ready to publish %s", MQTT_TOPIC_OUT)
            else:
                LOG.error("MQTT connect rc=%s", rc)
        self._client.on_connect = _on_connect

        try:
            self._client.connect(self.mqtt_host, self.mqtt_port, keepalive=30)
        except Exception as exc:
            LOG.error("MQTT connect to %s:%d failed: %s",
                      self.mqtt_host, self.mqtt_port, exc)
            return 2

        rx_thread = threading.Thread(target=self._rx_worker,
                                     name="image-rx-worker", daemon=True)
        rx_thread.start()
        stats_thread = threading.Thread(target=self._stats_worker,
                                        args=(stats_interval_s,),
                                        name="image-rx-stats", daemon=True)
        stats_thread.start()
        link_stats_thread = threading.Thread(target=self._link_stats_worker,
                                             name="image-rx-link-stats",
                                             daemon=True)
        link_stats_thread.start()

        def _handle_signal(_signum, _frame):
            LOG.info("signal received; shutting down")
            self._stop.set()
        signal.signal(signal.SIGINT, _handle_signal)
        signal.signal(signal.SIGTERM, _handle_signal)

        LOG.info("image_rx_daemon started; mqtt=%s:%d uart=%s",
                 self.mqtt_host, self.mqtt_port, self.uart)
        self._client.loop_start()
        try:
            while not self._stop.is_set():
                time.sleep(0.5)
        finally:
            self._client.loop_stop()
            self._client.disconnect()
            rx_thread.join(timeout=5.0)
        LOG.info("image_rx_daemon exit")
        return 0


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--uart", default=os.environ.get(
        "LIFETRAC_L072_UART", DEFAULT_UART))
    ap.add_argument("--baud", default=os.environ.get(
        "LIFETRAC_L072_BAUD", DEFAULT_BAUD))
    ap.add_argument("--mqtt-host", default=os.environ.get(
        "LIFETRAC_MQTT_HOST", DEFAULT_MQTT_HOST))
    ap.add_argument("--mqtt-port", type=int, default=int(os.environ.get(
        "LIFETRAC_MQTT_PORT", str(DEFAULT_MQTT_PORT))))
    ap.add_argument("--reassembler-timeout-ms", type=int, default=int(
        os.environ.get("LIFETRAC_REASSEMBLER_TIMEOUT_MS", "1500")))
    ap.add_argument("--stats-interval-s", type=float, default=10.0)
    ap.add_argument("--log-level", default=os.environ.get(
        "LIFETRAC_IMAGE_RX_LOG_LEVEL", "INFO"))
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    daemon = ImageRxDaemon(
        uart=args.uart,
        baud=args.baud,
        mqtt_host=args.mqtt_host,
        mqtt_port=args.mqtt_port,
        reassembler_timeout_ms=args.reassembler_timeout_ms,
    )
    return daemon.run(stats_interval_s=args.stats_interval_s)


if __name__ == "__main__":
    sys.exit(main())
