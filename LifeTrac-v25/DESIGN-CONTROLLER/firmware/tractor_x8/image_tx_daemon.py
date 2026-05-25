"""Tractor-side image-over-LoRa TX daemon (Phase B of strict-path migration).

Runs on the tractor Portenta X8 (board serial 2E2C1209DABC240B on the
2026-05-24 bench). Subscribes to the local MQTT broker
(``lifetrac/v25/cmd/image_frame``), fragments each TileDeltaFrame
payload via ``lora_proto.pack_telemetry_fragments`` using the
``PHY_IMAGE`` (SF7/BW500/CR5) profile, and pumps each fragment to the
on-board Murata CMWX1ZZABZ-078 SiP via the L072 ``HostLink``
``TX_FRAME_REQ`` (0x10) host protocol command on ``/dev/ttymxc3``.

This daemon is a long-running evolution of ``w2_02_tx_fragments.py`` —
that one-shot bench probe already proves the radio link end-to-end on
this exact board pair (2E2C → 2D0A). This file replaces the static
fragment-file source with a live MQTT subscriber so the production
camera_service can drive it continuously.

Strict-path migration context — see
``LifeTrac-v25/AI NOTES/2026-05-24_Camera_Fallback_Recovery_Runbook_v1_0.md``
(``Architectural Root Cause: Strict Path Needs Base-Side Linux Host``):

    camera_service.py  →  mosquitto (local)  →  THIS DAEMON  →  L072 HostLink
                                                              →  SX1276 air
                                                              →  base L072
                                                              →  image_rx_daemon.py
                                                              →  mosquitto (host, via adb reverse)
                                                              →  web_ui.py /ws/state

Wire format: each MQTT message body is a complete TileDeltaFrame (the
binary produced by ``image_pipeline.frame_format.encode_tile_delta_frame``).
That payload is fragmented with the v1 TELEMETRY_FRAGMENT_MAGIC (0xFE)
header, identical to what ``w2_02_host_pipeline.py`` emits, so the
existing ``FragmentReassembler`` (which understands both 0xFE and 0xFD)
on the RX side reassembles transparently.

Notes/limitations:
  * No LoRa-frame envelope, no AES-GCM, no audit/nonce/replay
    (bridge-bypass — same simplification as W2-02). The TX daemon's
    output on-air is raw fragment bodies, not full TelemetryFrames.
    That matches the RX daemon's expectations (Phase C). FIXME for
    production: wrap each fragment in a TelemetryFrame and let
    lora_bridge.py forward properly via cmd/image_frame subscription
    (Phase E).
  * Pacing honors MIN_LORA_HOST_INTER_CYCLE_S (50 ms) and the legal
    dwell cap (8 fragments per 400 ms dwell, 85% headroom) cloned from
    w2_02_host_pipeline.py.
  * On L072 transport failure the daemon logs and continues — does NOT
    kill the systemd unit, since the camera fallback path remains
    operational and we want zero blast radius during migration.
"""
from __future__ import annotations

import argparse
import logging
import os
import queue
import signal
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

# ---- repo imports ----------------------------------------------------
#
# The daemon expects to run from the tractor_x8 directory layout. We add
# the base_station directory to sys.path so lora_proto imports cleanly,
# matching how camera_service.py reaches into base_station for its own
# helpers. The x8_lora_bootloader_helper directory is also added so we
# can reuse the proven HostLink class from method_h_stage2_tx_probe_v2.

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT_FIRMWARE = os.path.dirname(_HERE)                  # firmware/
_BASE_STATION = os.path.abspath(
    os.path.join(_REPO_ROOT_FIRMWARE, "..", "base_station"))
_X8_HELPER = os.path.join(_REPO_ROOT_FIRMWARE, "x8_lora_bootloader_helper")
for _p in (_BASE_STATION, _X8_HELPER, _HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from lora_proto import (  # noqa: E402
    PHY_IMAGE,
    pack_telemetry_fragments,
)
from method_h_stage2_tx_probe_v2 import (  # noqa: E402
    HostLink,
    HOST_TYPE_VER_REQ,
    HOST_TYPE_VER_URC,
    HOST_TYPE_CFG_SET_REQ,
    HOST_TYPE_CFG_OK_URC,
    HOST_TYPE_TX_FRAME_REQ,
    CFG_KEY_LBT_ENABLE,
    drain_boot,
    drain_pending,
    wait_for_tx_done,
    configure_regulatory_profile_if_needed,
)

LOG = logging.getLogger("image_tx_daemon")

# ---- pacing constants (cloned from w2_02_host_pipeline.py) -----------
MIN_LORA_HOST_INTER_CYCLE_S = 0.05        # 50 ms between consecutive TX_FRAME_REQ
LEGAL_DWELL_US = 400_000                  # FCC §15.247(a)(1) 400 ms dwell
MAX_FRAMES_PER_DWELL_CAP = 8
DWELL_HEADROOM_PCT = 85
PER_FRAGMENT_TX_TIMEOUT_S = 3.0

# Topic the daemon subscribes to. camera_service.py already publishes
# here as part of the fallback path; the new daemon piggybacks so a
# single camera_service run feeds both paths until Phase D cutover.
MQTT_TOPIC_IN = "lifetrac/v25/cmd/image_frame"

# Local MQTT broker (camera_service publishes here, daemon subscribes).
DEFAULT_MQTT_HOST = "127.0.0.1"
DEFAULT_MQTT_PORT = 1883

# Default L072 host-link UART on the Portenta X8.
DEFAULT_UART = "/dev/ttymxc3"
DEFAULT_BAUD = "921600"


# ---- per-frame state -------------------------------------------------

@dataclass
class _PendingFrame:
    seq: int                     # frag_seq nibble (mod 256)
    payload: bytes
    enqueued_ms: int
    source_topic: str = MQTT_TOPIC_IN


# ---- daemon ----------------------------------------------------------

class ImageTxDaemon:
    """MQTT → L072 fragment pump.

    One worker thread owns the L072 ``HostLink`` and serializes all
    TX_FRAME_REQ calls; the MQTT callback thread only enqueues completed
    frame payloads. That separation keeps the radio fully busy without
    blocking the MQTT loop.
    """

    def __init__(self, *, uart: str, baud: str,
                 mqtt_host: str, mqtt_port: int,
                 inter_cycle_s: float,
                 max_queue_depth: int) -> None:
        self.uart = uart
        self.baud = baud
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.inter_cycle_s = max(MIN_LORA_HOST_INTER_CYCLE_S, inter_cycle_s)
        # Bounded queue: under sustained overload we drop OLDEST so the
        # newest video stays current (visual systems prefer freshness over
        # completeness).
        self._q: "queue.Queue[_PendingFrame]" = queue.Queue(maxsize=max_queue_depth)
        self._stop = threading.Event()
        self._next_seq = 0
        # Counters (read for periodic stats prints).
        self.lock = threading.Lock()
        self.frames_in = 0
        self.frames_dropped_queue_full = 0
        self.frames_tx_ok = 0
        self.frames_tx_fail = 0
        self.fragments_tx_ok = 0
        self.fragments_tx_fail = 0

    # ---- MQTT side ----
    def _on_message(self, _client, _userdata, msg) -> None:
        try:
            payload = msg.payload
            if not isinstance(payload, (bytes, bytearray)):
                LOG.debug("ignoring non-bytes payload (%s)", type(payload).__name__)
                return
            with self.lock:
                self.frames_in += 1
                self._next_seq = (self._next_seq + 1) & 0xFF
                seq = self._next_seq
            frame = _PendingFrame(
                seq=seq,
                payload=bytes(payload),
                enqueued_ms=int(time.monotonic() * 1000),
                source_topic=msg.topic,
            )
            try:
                self._q.put_nowait(frame)
            except queue.Full:
                # Drop the OLDEST queued frame, push the new one — prefer
                # freshness for video.
                try:
                    self._q.get_nowait()
                    with self.lock:
                        self.frames_dropped_queue_full += 1
                except queue.Empty:
                    pass
                try:
                    self._q.put_nowait(frame)
                except queue.Full:
                    with self.lock:
                        self.frames_dropped_queue_full += 1
        except Exception as exc:                              # pragma: no cover
            LOG.exception("MQTT on_message: unexpected %s", exc)

    # ---- L072 side ----
    def _open_link(self) -> HostLink:
        LOG.info("opening L072 HostLink on %s @ %s", self.uart, self.baud)
        link = HostLink(self.uart, self.baud)
        # Match w2_02_tx_fragments.py priming sequence so the bench-proven
        # boot/configure/LBT-off setup applies identically.
        # 2026-05-25 W2-02 v3 fix: skip UART RESET_REQ when caller has
        # already pulsed gpio163 NRST externally (see rx_ver_warmup_diag.py
        # sweep — UART RESET_REQ on top of just-booted firmware races boot
        # and intermittently breaks VER warm-up). Opt-in via env to keep
        # default behaviour for callers that rely on the daemon self-reset.
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
        except Exception as exc:
            LOG.error("VER warm-up failed: %s", exc)
            raise
        drain_pending(link, quiet_s=0.25, max_s=1.0)
        try:
            configure_regulatory_profile_if_needed(link)
        except Exception as exc:                              # pragma: no cover
            LOG.warning("regulatory profile config failed: %s", exc)
        try:
            link.request(HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
                         bytes([CFG_KEY_LBT_ENABLE, 0x01, 0x00]), timeout=1.0)
            LOG.info("LBT_ENABLE=0 (matches W1-10b TX_BURST rationale)")
        except Exception as exc:                              # pragma: no cover
            LOG.warning("CFG_SET(LBT_ENABLE=0) failed: %s", exc)
        return link

    def _tx_worker(self) -> None:
        try:
            link = self._open_link()
        except Exception as exc:
            LOG.error("fatal: cannot open L072 HostLink: %s", exc)
            self._stop.set()
            return
        LOG.info("TX worker ready (inter_cycle_s=%.3f, max %d frags/dwell)",
                 self.inter_cycle_s, MAX_FRAMES_PER_DWELL_CAP)
        while not self._stop.is_set():
            try:
                frame = self._q.get(timeout=0.25)
            except queue.Empty:
                continue
            self._tx_one_frame(link, frame)

    def _tx_one_frame(self, link: HostLink, frame: _PendingFrame) -> None:
        try:
            fragments = pack_telemetry_fragments(
                frame.payload, frame.seq, PHY_IMAGE)
        except Exception as exc:
            LOG.error("fragment pack failed (seq=%d, %d B): %s",
                      frame.seq, len(frame.payload), exc)
            with self.lock:
                self.frames_tx_fail += 1
            return

        n = len(fragments)
        if n > MAX_FRAMES_PER_DWELL_CAP:
            LOG.warning(
                "frame seq=%d requires %d fragments (>%d dwell cap); "
                "transmitting anyway with extra cadence padding",
                frame.seq, n, MAX_FRAMES_PER_DWELL_CAP)
        LOG.debug("TX frame seq=%d %d B → %d fragments", frame.seq,
                  len(frame.payload), n)

        ok = fail = 0
        for idx, body in enumerate(fragments):
            if self._stop.is_set():
                break
            # L072 firmware caps TX_FRAME_REQ bodies at 64 B (matches
            # W1-10b run_tx_burst clamp). The image-profile fragmenter
            # respects this via TELEMETRY_FRAGMENT_MAX_AIRTIME_MS, but
            # guard anyway since a clamp surprise here would silently
            # truncate fragments and break reassembly.
            if len(body) > 64:
                fail += 1
                LOG.error("fragment %d body %d B > 64 B L072 cap; dropping",
                          idx, len(body))
                continue
            tx_id = idx & 0xFF
            # TX_FRAME_REQ payload format (matches w2_02_tx_fragments.py):
            #   u8 tx_id, u8 len, u8 body[len]
            tx_frame = bytes([tx_id, len(body)]) + body
            try:
                link.send(HOST_TYPE_TX_FRAME_REQ, tx_frame)
            except Exception as exc:
                fail += 1
                LOG.warning("TX send exception: seq=%d idx=%d: %s",
                            frame.seq, idx, exc)
                continue
            try:
                done, faults = wait_for_tx_done(
                    link, tx_id, timeout=PER_FRAGMENT_TX_TIMEOUT_S)
            except TimeoutError as exc:
                fail += 1
                LOG.warning("TX_DONE timeout: seq=%d idx=%d tx_id=0x%02x %s",
                            frame.seq, idx, tx_id, exc)
                if idx + 1 < n:
                    time.sleep(self.inter_cycle_s)
                continue
            except Exception as exc:
                fail += 1
                LOG.warning("TX_DONE exception: seq=%d idx=%d: %s",
                            frame.seq, idx, exc)
                if idx + 1 < n:
                    time.sleep(self.inter_cycle_s)
                continue
            if done["status"] == 0:  # SX1276_TX_STATUS_OK
                ok += 1
            else:
                fail += 1
                LOG.warning("TX_DONE non-OK: seq=%d idx=%d tx_id=0x%02x "
                            "status=%d(%s) toa_us=%d",
                            frame.seq, idx, tx_id, done["status"],
                            done.get("status_name", "?"),
                            done.get("time_on_air_us", 0))
            for f in faults:
                LOG.warning("TX fault: seq=%d idx=%d %s",
                            frame.seq, idx, f)
            # Cadence pacing — sleep between fragments.
            if idx + 1 < n:
                time.sleep(self.inter_cycle_s)

        with self.lock:
            self.fragments_tx_ok += ok
            self.fragments_tx_fail += fail
            if fail == 0 and ok == n:
                self.frames_tx_ok += 1
            else:
                self.frames_tx_fail += 1
        LOG.info("frame seq=%d done: %d/%d fragments ok (%d fail)",
                 frame.seq, ok, n, fail)

    # ---- stats printer ----
    def _stats_worker(self, interval_s: float) -> None:
        last = 0.0
        while not self._stop.is_set():
            time.sleep(1.0)
            now = time.monotonic()
            if now - last < interval_s:
                continue
            last = now
            with self.lock:
                LOG.info(
                    "stats: frames_in=%d ok=%d fail=%d drop_full=%d "
                    "frags_ok=%d frags_fail=%d qdepth=%d",
                    self.frames_in, self.frames_tx_ok, self.frames_tx_fail,
                    self.frames_dropped_queue_full, self.fragments_tx_ok,
                    self.fragments_tx_fail, self._q.qsize())

    # ---- lifecycle ----
    def run(self, *, stats_interval_s: float) -> int:
        # paho-mqtt is the same dep camera_service.py uses; import inside
        # run() so synthetic-test imports of the module don't require it.
        import paho.mqtt.client as mqtt

        client = mqtt.Client(client_id=f"lifetrac-image-tx-{os.getpid()}")
        client.on_message = self._on_message

        def _on_connect(_c, _u, _f, rc):
            if rc == 0:
                LOG.info("MQTT connected; subscribing to %s", MQTT_TOPIC_IN)
                client.subscribe(MQTT_TOPIC_IN, qos=0)
            else:
                LOG.error("MQTT connect rc=%s", rc)
        client.on_connect = _on_connect

        try:
            client.connect(self.mqtt_host, self.mqtt_port, keepalive=30)
        except Exception as exc:
            LOG.error("MQTT connect to %s:%d failed: %s",
                      self.mqtt_host, self.mqtt_port, exc)
            return 2

        tx_thread = threading.Thread(target=self._tx_worker,
                                     name="image-tx-worker", daemon=True)
        tx_thread.start()
        stats_thread = threading.Thread(target=self._stats_worker,
                                        args=(stats_interval_s,),
                                        name="image-tx-stats", daemon=True)
        stats_thread.start()

        # SIGTERM/SIGINT → graceful shutdown.
        def _handle_signal(_signum, _frame):
            LOG.info("signal received; shutting down")
            self._stop.set()
        signal.signal(signal.SIGINT, _handle_signal)
        signal.signal(signal.SIGTERM, _handle_signal)

        LOG.info("image_tx_daemon started; mqtt=%s:%d uart=%s",
                 self.mqtt_host, self.mqtt_port, self.uart)
        client.loop_start()
        try:
            while not self._stop.is_set():
                time.sleep(0.5)
        finally:
            client.loop_stop()
            client.disconnect()
            tx_thread.join(timeout=5.0)
        LOG.info("image_tx_daemon exit")
        return 0


# ---- entry point -----------------------------------------------------

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
    ap.add_argument("--inter-cycle-s", type=float, default=float(os.environ.get(
        "LIFETRAC_LORA_INTER_CYCLE_S", str(MIN_LORA_HOST_INTER_CYCLE_S))))
    ap.add_argument("--max-queue-depth", type=int, default=int(os.environ.get(
        "LIFETRAC_IMAGE_TX_QUEUE_DEPTH", "4")))
    ap.add_argument("--stats-interval-s", type=float, default=10.0)
    ap.add_argument("--log-level", default=os.environ.get(
        "LIFETRAC_IMAGE_TX_LOG_LEVEL", "INFO"))
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(levelname)s %(name)s: %(message)s")

    daemon = ImageTxDaemon(
        uart=args.uart,
        baud=args.baud,
        mqtt_host=args.mqtt_host,
        mqtt_port=args.mqtt_port,
        inter_cycle_s=args.inter_cycle_s,
        max_queue_depth=args.max_queue_depth,
    )
    return daemon.run(stats_interval_s=args.stats_interval_s)


if __name__ == "__main__":
    sys.exit(main())
