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
import collections
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
    PHY_IMAGE_BW250,
    PHY_IMAGE_BW500,
    LORA_HOP_HDR_LEN,
    IMAGE_FRAG_AIR_CAP_MS,
    add_parity_fragments,
    lora_time_on_air_ms,
    pack_image_fragments,
    pack_image_fragments_v2,
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
    verify_modem_matches_profile,
)

LOG = logging.getLogger("image_tx_daemon")

# ---- pacing constants (cloned from w2_02_host_pipeline.py) -----------
MIN_LORA_HOST_INTER_CYCLE_S = 0.05        # 50 ms between consecutive TX_FRAME_REQ
LEGAL_DWELL_US = 400_000                  # FCC §15.247(a)(1) 400 ms dwell
MAX_FRAMES_PER_DWELL_CAP = 8
DWELL_HEADROOM_PCT = 85
PER_FRAGMENT_TX_TIMEOUT_S = 3.0

_PROFILE_TO_PHY = {0: PHY_IMAGE_BW250, 1: PHY_IMAGE_BW250, 2: PHY_IMAGE_BW500}

class AirtimeBudget:
    """Host mirror of the firmware per-channel QoS gate
    (sx1276_airtime.c: 400 ms ToA per 1 s window). Rolling window =
    strictly more conservative than the firmware's fixed-anchor window,
    so a paced TX can never draw ABORT_QOS. 380 ms budget leaves
    a 20 ms guard for estimator rounding."""

    def __init__(self, budget_us: int = 380_000, window_s: float = 1.0):
        self.budget_us, self.window_s = budget_us, window_s
        self._events: collections.deque[tuple[float, int]] = collections.deque()

    def _used(self, now: float) -> int:
        while self._events and now - self._events[0][0] >= self.window_s:
            self._events.popleft()
        return sum(toa for _, toa in self._events)

    def admit(self, est_toa_us: int, stop: threading.Event) -> bool:
        """Block until est_toa_us fits the window. Returns False on stop."""
        while not stop.is_set():
            now = time.monotonic()
            if self._used(now) + est_toa_us <= self.budget_us:
                return True
            wait = self._events[0][0] + self.window_s - now  # oldest expiry
            stop.wait(min(max(wait, 0.005), 0.25))
        return False

    def record(self, toa_us: int) -> None:
        self._events.append((time.monotonic(), toa_us))

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
        self.budget = AirtimeBudget()
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
        self.frames_dropped_stale = 0
        self.frames_tx_ok = 0
        self.frames_tx_fail = 0
        self.fragments_tx_ok = 0
        self.fragments_tx_fail = 0
        self.bytes_tx_ok = 0

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
        if _os.environ.get("LIFETRAC_SKIP_PHY_CONTRACT", "0") != "1":
            prof_id = int(_os.environ.get("LIFETRAC_REG_PROFILE", "0"))
            active_phy = _PROFILE_TO_PHY.get(prof_id, PHY_IMAGE_BW250)
            verify_modem_matches_profile(link, active_phy)
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

    def recent_frag_loss_rate(self) -> float:
        with self.lock:
            total = self.fragments_tx_ok + self.fragments_tx_fail
            return (self.fragments_tx_fail / total) if total > 0 else 0.0

    def _pack_for(self, frame: _PendingFrame) -> list[bytes]:
        is_key = frame.payload[:1] == b"\x01"          # frame_kind byte
        copies = int(os.environ.get("LIFETRAC_KEYFRAME_COPIES", "1"))
        if copies <= 1 and is_key and self.recent_frag_loss_rate() > 0.005:
            copies = 2                                  # auto: only when PER says so
        prof_id = int(os.environ.get("LIFETRAC_REG_PROFILE", "0"))
        active_phy = _PROFILE_TO_PHY.get(prof_id, PHY_IMAGE_BW250)
        if is_key and copies > 1:
            return pack_image_fragments_v2(frame.payload, frame.seq,
                                           active_phy, IMAGE_FRAG_AIR_CAP_MS,
                                           copies=copies)
        frags = pack_image_fragments(frame.payload, frame.seq,
                                     active_phy, IMAGE_FRAG_AIR_CAP_MS)
        # Phase 3: optional XOR parity (0xFC). RX-side reconstruction
        # shipped first (reassemble.py); enable emission per deployment
        # via LIFETRAC_PARITY_GROUP=8. v1 path only — the v2 copies path
        # above already carries its own redundancy.
        parity_group = int(os.environ.get("LIFETRAC_PARITY_GROUP", "0"))
        if parity_group > 0:
            frags = add_parity_fragments(frags, frame.seq, parity_group)
        return frags

    def _tx_one_frame(self, link: HostLink, frame: _PendingFrame) -> None:
        # F16a: Stale-frame cancellation
        frame_max_age_ms = int(os.environ.get("LIFETRAC_FRAME_MAX_AGE_MS", "10000"))
        age_ms = int(time.monotonic() * 1000) - frame.enqueued_ms
        if age_ms > frame_max_age_ms and not self._q.empty():
            with self.lock:
                self.frames_dropped_stale += 1
            LOG.info("dropping stale frame seq=%d (age %d ms, fresher queued)",
                     frame.seq, age_ms)
            return

        try:
            fragments = self._pack_for(frame)
        except Exception as exc:
            LOG.error("fragment pack failed (seq=%d, %d B): %s",
                      frame.seq, len(frame.payload), exc)
            with self.lock:
                self.frames_tx_fail += 1
            return

        n = len(fragments)
        LOG.debug("TX frame seq=%d %d B → %d fragments", frame.seq,
                  len(frame.payload), n)

        max_qos_retries = 4   # FORBIDDEN/ABORT_QOS: not admitted, ZERO RF spent
        max_rf_retries  = 1   # TX_DONE non-OK / timeout: airtime was spent
        prof_id = int(os.environ.get("LIFETRAC_REG_PROFILE", "0"))
        active_phy = _PROFILE_TO_PHY.get(prof_id, PHY_IMAGE_BW250)

        for idx, body in enumerate(fragments):
            if self._stop.is_set():
                break
            est_us = int(lora_time_on_air_ms(len(body) + LORA_HOP_HDR_LEN,
                                             active_phy) * 1000)
            sent = False
            qos_retries_left = max_qos_retries
            rf_retries_left = max_rf_retries

            while True:
                if not self.budget.admit(est_us, self._stop):
                    return                                     # shutting down
                try:
                    tx_id = idx & 0xFF
                    tx_frame = bytes([tx_id, len(body)]) + body
                    link.send(HOST_TYPE_TX_FRAME_REQ, tx_frame)
                    done, faults = wait_for_tx_done(
                        link, tx_id, timeout=PER_FRAGMENT_TX_TIMEOUT_S)
                except RuntimeError as exc:      # ERR_PROTO FORBIDDEN == QoS refusal:
                    qos_retries_left -= 1
                    if qos_retries_left < 0:
                        LOG.warning("QoS refusal cap reached for frag %d", idx)
                        break
                    time.sleep(est_us / 2e6)                   # cheap, no RF spent
                    continue
                except (TimeoutError, Exception) as exc:
                    self.budget.record(est_us)                 # assume RF spent
                    rf_retries_left -= 1
                    if rf_retries_left < 0:
                        LOG.warning("RF send error for frag %d: %s", idx, exc)
                        break
                    continue                                   # retry

                actual_toa = done.get("time_on_air_us") or est_us
                self.budget.record(actual_toa)
                if done["status"] == 0:  # SX1276_TX_STATUS_OK
                    sent = True
                    with self.lock:
                        self.fragments_tx_ok += 1
                        self.bytes_tx_ok += len(body)
                    break
                else:
                    rf_retries_left -= 1
                    LOG.warning("TX_DONE non-OK: seq=%d idx=%d tx_id=0x%02x "
                                "status=%d(%s) toa_us=%d",
                                frame.seq, idx, tx_id, done["status"],
                                done.get("status_name", "?"),
                                done.get("time_on_air_us", 0))
                    if rf_retries_left < 0:
                        break

            if not sent:
                # F4: a frame missing any fragment can never reassemble --
                # stop burning airtime on it and let RX request a keyframe.
                with self.lock:
                    self.fragments_tx_fail += 1
                    self.frames_tx_fail += 1
                LOG.warning("frame seq=%d ABORTED at fragment %d/%d",
                            frame.seq, idx, len(fragments))
                return

        with self.lock:
            self.frames_tx_ok += 1
        LOG.info("frame seq=%d done: %d fragments ok", frame.seq, n)

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
