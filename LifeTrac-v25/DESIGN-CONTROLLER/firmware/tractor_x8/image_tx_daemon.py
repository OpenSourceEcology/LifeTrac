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
    pack_command_frame,
    parse_command_frame,
    CMD_OP_REQ_KEYFRAME,
    CMD_OP_ENCODE_MODE,
    CMD_OP_RADIO_PROFILE,
    CMD_OP_RADIO_PROFILE_ACK,
    CMD_OP_RADIO_PROFILE_CONF,
    CMD_OP_ENCODE_MODE_ACK,
    CMD_OP_PROBE,
    CMD_OP_PROBE_ECHO,
)
from method_h_stage2_tx_probe_v2 import (  # noqa: E402
    HostLink,
    HOST_TYPE_VER_REQ,
    HOST_TYPE_VER_URC,
    HOST_TYPE_CFG_SET_REQ,
    HOST_TYPE_CFG_OK_URC,
    HOST_TYPE_TX_FRAME_REQ,
    HOST_TYPE_TX_DONE_URC,
    HOST_TYPE_ERR_PROTO_URC,
    HOST_TYPE_FAULT_URC,
    CFG_KEY_LBT_ENABLE,
    BENIGN_FAULT_CODES,
    parse_tx_done,
    format_err_proto_payload,
    format_fault_payload,
    drain_boot,
    drain_pending,
    wait_for_tx_done,
    configure_regulatory_profile_if_needed,
    verify_modem_matches_profile,
    HOST_TYPE_RX_FRAME_URC,
    parse_rx_frame,
    read_reg,
    write_reg,
    SX1276_REG_OP_MODE,
    SX1276_OPMODE_LORA_RXCONT,
)

LOG = logging.getLogger("image_tx_daemon")

# ---- pacing constants (cloned from w2_02_host_pipeline.py) -----------
MIN_LORA_HOST_INTER_CYCLE_S = 0.05        # 50 ms between consecutive TX_FRAME_REQ
LEGAL_DWELL_US = 400_000                  # FCC §15.247(a)(1) 400 ms dwell
MAX_FRAMES_PER_DWELL_CAP = 8
DWELL_HEADROOM_PCT = 85
PER_FRAGMENT_TX_TIMEOUT_S = 3.0

_PROFILE_TO_PHY = {0: PHY_IMAGE_BW250, 1: PHY_IMAGE_BW250, 2: PHY_IMAGE_BW500}

# D4 host mirror: the firmware widens its per-channel QoS budget to
# 950 ms/s under REG_PROFILE_FCC_15_247_DTS_BW500 (PSD-based compliance,
# no P0 on the strict path). The host bucket stays 20 ms under the
# firmware cap so a paced TX can never draw ABORT_QOS.
#
# v25.0.7 slot-clock: profile 1's real throttle is now the 200 ms hop
# grid (5 slots/s x <=173 ms ToA ~= 865 ms air/s ceiling), and the
# firmware QoS gate is per-CHANNEL (each channel sees <=2 packets per
# 10 s under hopping, nowhere near its 400 ms/s bucket). The old
# 380 ms/s HOST bucket was a single-carrier assumption that throttled
# FHSS to ~2.3 pkt/s; raise it to 860 ms/s so the slot grid, not the
# host, is the binding constraint.
_PROFILE_TO_BUDGET_US = {0: 380_000, 1: 860_000, 2: 930_000}

# LIFETRAC_TX_PIPELINE: 'v2' (default) = serial send->TX_DONE->send;
# 'v3' = keep 2 TX_FRAME_REQs in flight against the firmware's depth-2
# mailbox (host_cmd.c s_tx_pending) so the UART turnaround rides inside
# the previous fragment's time-on-air.
TX_PIPELINE = os.environ.get("LIFETRAC_TX_PIPELINE", "v2").strip().lower()
# RS-2.3 (2026-07-25): outstanding TX_FRAME_REQs kept in flight in v3 mode.
# Was a hard 2, matched to the old single-slot firmware mailbox. The
# 2026-07-25 firmware carries a depth-4 park ring (1 transmitting + 4
# parked), so up to 4 never NAKs. Latency guard (RS-9.7): keep <=3 at
# FHSS; 4 is for DTS. Default stays 2 for old-firmware compatibility.
PIPELINE_DEPTH = max(1, int(os.environ.get("LIFETRAC_TX_PIPELINE_DEPTH", "2")))
# RS-3.1 (2026-07-25): batch multiple non-key frames into one fragment
# train so the measured ~44 ms host-side per-handoff overhead (Run D
# forensics) amortizes across 2-4 frames. Default OFF until air-verified;
# the bench harness enables it explicitly.
TX_BATCH = int(os.environ.get("LIFETRAC_TX_BATCH", "0"))
# RS-3.10 (2026-07-26): build train N+1 while train N is on air (the
# pipelined wait loop is ~mostly idle polling for TX_DONEs), then hold a
# DESIGNED inter-train gap spent actively listening — the reverse-slot
# command window — instead of an accidental gap of Python overhead.
TX_PREPARE_AHEAD = int(os.environ.get("LIFETRAC_TX_PREPARE_AHEAD", "0"))
TRAIN_GAP_S = max(0.0, float(os.environ.get("LIFETRAC_TRAIN_GAP_MS", "40")) / 1000.0)
# 760 (2026-07-26 runt analysis): admits TRIPLES of ~246 B frames (748 B →
# 4 fragments at ~111 ms/frame vs pairs' ~120). The runt fragment is
# structural for frames >243 B (one fragment body) — batching amortizes it:
# 1 runt per N frames instead of per frame. The efficiency guard in
# _batch_more rejects adds that worsen fragments-per-frame, so variable
# real-camera sizes can't produce pathological fits. History: 480 silently
# rejected every pair (Run E); 520 allowed pairs (runs F–K).
BATCH_BUDGET_B = int(os.environ.get("LIFETRAC_BATCH_BUDGET_B", "760"))
# Fragment body capacity per radio profile: TX_FRAME_BODY_MAX − 4 B
# fragment header at BW500 (profile 2) = 243; at BW250 (profiles 0/1) the
# 170 ms air cap limits bodies to 207 → 203 usable. 2026-07-27: was a
# hardcoded 243, which mis-sized the batching efficiency guard whenever
# the radio ran FHSS.
_FRAG_BODY_BY_PROFILE = {0: 203, 1: 203, 2: 243}


def _frag_body_b(profile: int) -> int:
    return _FRAG_BODY_BY_PROFILE.get(int(profile), 203)


BATCH_MAX_FRAMES = max(1, int(os.environ.get("LIFETRAC_BATCH_MAX_FRAMES", "4")))
try:
    from image_pipeline.frame_format import pack_frame_batch
except Exception:                                             # pragma: no cover
    pack_frame_batch = None                                   # old deployments

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

# 2026-07-25 "LoRa-only tractor comms": control traffic reaches this
# daemon EXCLUSIVELY as 0xFB command frames over the LoRa link (base
# image_rx_daemon transmits them). The daemon then rebroadcasts on
# TRACTOR-scoped MQTT topics as the intra-board hop to camera_service —
# deliberately DIFFERENT topic names from the base-side control/cmd
# topics so a shared bench broker cannot silently bypass the radio.
# Acks travel back over LoRa the same way (this daemon is the TX side).
RADIO_PROFILE_ACK_TOPIC   = "lifetrac/v25/status/radio_profile/tx"   # local log only
TRACTOR_ENCODE_MODE_TOPIC = "lifetrac/v25/tractor/encode_mode_override"
TRACTOR_KEYFRAME_TOPIC    = "lifetrac/v25/tractor/req_keyframe"
TRACTOR_ENC_STATUS_TOPIC  = "lifetrac/v25/status/encode_mode"        # camera_service ack (local)
# 2026-07-27 encode-to-fit: this daemon owns the radio profile, so it tells
# camera_service the live per-frame byte budget (one fragment body at the
# active profile) via LinkBudget.update. Replaces the CMD_LINK_PROFILE M7
# back-channel that USE_LORA_BRIDGE mode never starts, so the encoder
# finally sizes frames to the real air quantum (243 DTS / 203 FHSS) instead
# of a boot-frozen env default.
TRACTOR_LINK_BUDGET_TOPIC = "lifetrac/v25/tractor/link_budget"
# LINK_PHY_NAMES index (camera_service / lora_proto) for each radio profile:
# profile 0/1 (BW250) -> "image_bw250" (idx 5); profile 2 (BW500) -> idx 6.
_PROFILE_TO_LINK_PHY_IDX = {0: 5, 1: 5, 2: 6}
_PROFILE_CHOICES = (0, 1, 2)
# Two-phase profile switch (tractor side): ack on the OLD profile, dwell
# so the ack flushes, switch, then REVERT unless the base confirms on the
# new profile within the window (fail-safe against a half-switched link).
# Window covers worst-case FHSS re-acquisition on the base (~25 s scan)
# plus its proof-of-life wait and the CONF flight time.
PROFILE_SWITCH_ACK_FLUSH_S   = 0.7
PROFILE_CONFIRM_TIMEOUT_S    = 45.0

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
        self.toa_us_sum = 0
        # D4 host mirror: budget follows the active regulatory profile
        # (DTS BW500 -> 930 ms/s, 20 ms under the firmware's 950 ms cap).
        _prof = int(os.environ.get("LIFETRAC_REG_PROFILE", "0"))
        self.budget = AirtimeBudget(
            budget_us=_PROFILE_TO_BUDGET_US.get(_prof, 380_000))
        # 2026-07-25 radio-profile selector state.
        self._pending_profile: int | None = None
        self._active_profile = _prof
        self._mqtt_client = None            # set in run(); used for acks
        self._carry: "_PendingFrame | None" = None   # RS-3.1 batch look-ahead
        self._next_train: "_PendingFrame | None" = None  # RS-3.10 prepared
        self._last_train_done_t = 0.0                    # RS-3.10 gap timer
        # LoRa-only control plane: pending outbound command frames
        # (acks) + the revert deadline armed after a profile switch.
        self._cmd_out: "queue.Queue[bytes]" = queue.Queue(maxsize=8)
        self._confirm_deadline: float | None = None
        self._revert_profile: int | None = None
        # camera_service's local encode ack, forwarded over LoRa.
        self._enc_ack_out: bytes | None = None
        # Last encode ack actually radiated as 0x68 — used to suppress the
        # retained-replay echo paho re-delivers on every MQTT reconnect.
        self._enc_ack_last_fwd: bytes | None = None

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
        # VER warm-up with retry. A freshly hard-reset L072 (SWD/NRST path)
        # can miss or answer the first request slowly while it drains its
        # boot burst + periodic STATS URCs; a single 1.0 s attempt produced
        # spurious "cannot open HostLink" fatals (2026-07-24 saturation
        # bench). VER_REQ is idempotent, so retrying is safe. On failure we
        # dump whatever frames DID arrive (wrong-type-match forensics).
        last_exc: Exception | None = None
        for attempt in range(1, 6):
            try:
                link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=2.0)
                last_exc = None
                break
            except Exception as exc:
                last_exc = exc
                LOG.warning("VER warm-up attempt %d/5 failed: %s", attempt, exc)
                try:
                    stray = list(link.urc_queue)
                    link.urc_queue.clear()
                    stray += link.read_frames(0.3)
                    for f in stray:
                        LOG.warning("  stray frame type=0x%02x len=%d",
                                    f.get("type", 0), len(f.get("payload", b"")))
                except Exception:
                    pass
        if last_exc is not None:
            LOG.error("VER warm-up failed: %s", last_exc)
            raise last_exc
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
        # LoRa-only control plane: arm RXCONT so the tractor can HEAR the
        # base's 0xFB command frames between its own transmissions (the
        # firmware re-arms RX after each TX when it was armed before —
        # sx1276_tx.c s_rearm_rx).
        try:
            opm, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
            if opm != SX1276_OPMODE_LORA_RXCONT:
                write_reg(link, SX1276_REG_OP_MODE,
                          SX1276_OPMODE_LORA_RXCONT, timeout=0.5)
                LOG.info("RXCONT armed for command downlink (opmode "
                         "0x%02x -> 0x85)", opm)
        except Exception as exc:                              # pragma: no cover
            LOG.warning("RXCONT arm failed: %s — command downlink deaf "
                        "until next TX re-arm", exc)
        LOG.info("TX worker ready (inter_cycle_s=%.3f, max %d frags/dwell)",
                 self.inter_cycle_s, MAX_FRAMES_PER_DWELL_CAP)
        while not self._stop.is_set():
            # ORDER MATTERS: flush the control plane FIRST so a profile
            # ACK radiates on the OLD grid before _maybe_switch_profile
            # retunes us (two-phase contract with the base).
            self._service_control_plane(link)
            self._maybe_switch_profile(link)
            # RS-3.10: a train prepared during the previous burst goes out
            # after the DESIGNED reverse-slot gap (spent listening, so the
            # command window survives prepare-ahead by construction).
            prepared = None
            if self._next_train is not None:
                prepared, self._next_train = self._next_train, None
                gap = TRAIN_GAP_S - (time.monotonic()
                                     - self._last_train_done_t)
                if gap > 0:
                    for rx in self._drain_rx_frames(link, gap):
                        self._dispatch_command(rx)
            if prepared is None:
                try:
                    if self._carry is not None:
                        frame, self._carry = self._carry, None
                    else:
                        frame = self._q.get(timeout=0.25)
                except queue.Empty:
                    # Idle: re-arm the command downlink (our own TXes park
                    # the modem in STANDBY — see _ensure_rxcont), then poll
                    # for inbound command frames.
                    self._ensure_rxcont(link)
                    for rx in self._drain_rx_frames(link, 0.05):
                        self._dispatch_command(rx)
                    continue
                prepared = self._batch_more(frame)   # RS-3.1
            self._tx_one_frame(link, prepared)
            self._last_train_done_t = time.monotonic()

    def _prep_next_train(self) -> None:
        """RS-3.10: called from the pipelined wait loop — build the next
        train from queued frames while the current one is on air. Pure
        host-side work (dequeue + batch); fragmentation stays at TX time
        so a profile switch between trains re-fragments correctly."""
        if TX_PREPARE_AHEAD == 0 or self._next_train is not None:
            return
        try:
            if self._carry is not None:
                frame, self._carry = self._carry, None
            else:
                frame = self._q.get_nowait()
        except queue.Empty:
            return
        self._next_train = self._batch_more(frame)

    def _batch_more(self, first: "_PendingFrame") -> "_PendingFrame":
        """RS-3.1: greedily pull queued non-key frames into one batched
        payload (length-prefixed container, FRAME_BATCH_MAGIC). A frame
        that does not fit is CARRIED to the next loop pass — never pushed
        back to the queue tail, so air order == submit order. Keyframes
        are never batched (they keep the copies/parity paths)."""
        if TX_BATCH == 0 or pack_frame_batch is None:
            return first
        if first.payload[:1] == b"\x01":       # keyframe: never batch
            return first
        batch = [first]
        total = 2 + 2 + len(first.payload)     # magic+count + len+seg
        while len(batch) < BATCH_MAX_FRAMES:
            try:
                nxt = self._q.get_nowait()
            except queue.Empty:
                break
            cand = total + 2 + len(nxt.payload)
            # Runt-aware efficiency guard (2026-07-26): accept the frame
            # only if fragments-per-frame does not get WORSE — e.g. a
            # 246 B pair (3 frags / 2 frames = 1.5) beats singles (2/1),
            # and a triple (4/3 = 1.33) beats the pair; but a fit that
            # would spill a near-empty extra fragment for a tiny frame
            # is rejected and carried to the next train.
            body_b = _frag_body_b(self._active_profile)
            frags_with = -(-cand // body_b)
            frags_without = -(-total // body_b)
            eff_ok = (frags_with * len(batch)
                      <= frags_without * (len(batch) + 1))
            if (nxt.payload[:1] == b"\x01"
                    or cand > BATCH_BUDGET_B
                    or not eff_ok):
                self._carry = nxt
                break
            batch.append(nxt)
            total = cand
        if len(batch) == 1:
            return first
        self._batched_frames = getattr(self, "_batched_frames", 0) + len(batch)
        self._batched_trains = getattr(self, "_batched_trains", 0) + 1
        if self._batched_trains % 25 == 1:
            LOG.info("batching: %d frames in %d trains so far (this train: "
                     "%d frames, %d B)", self._batched_frames,
                     self._batched_trains, len(batch),
                     sum(len(f.payload) for f in batch))
        return _PendingFrame(
            seq=first.seq,
            payload=pack_frame_batch([f.payload for f in batch]),
            enqueued_ms=first.enqueued_ms,     # oldest member: stale check
            source_topic=first.source_topic)

    def _ensure_rxcont(self, link: HostLink) -> None:
        """Re-arm RXCONT for the command downlink (run-31 root cause).

        RXCONT is armed via a RAW opmode write, so the firmware's TX
        path doesn't know RX was armed and leaves the modem in STANDBY
        after every image fragment — the downlink was deaf from the
        first TX in run 31. Re-arm in the idle gaps between frames;
        during continuous TX the tractor listens only in those gaps
        (acceptable: commands are retried ×2 and idempotent).
        """
        try:
            opm, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
            if opm != SX1276_OPMODE_LORA_RXCONT:
                write_reg(link, SX1276_REG_OP_MODE,
                          SX1276_OPMODE_LORA_RXCONT, timeout=0.5)
                LOG.debug("RXCONT re-armed (opmode 0x%02x -> 0x85)", opm)
        except Exception as exc:                              # pragma: no cover
            LOG.warning("RXCONT re-arm failed: %s", exc)

    # ---- LoRa control plane (2026-07-25) ----
    def _drain_rx_frames(self, link: HostLink, timeout_s: float) -> list:
        """Return RX_FRAME payloads seen within timeout (never raises)."""
        out = []
        try:
            for f in link.read_frames(timeout_s):
                if f.get("type") != HOST_TYPE_RX_FRAME_URC:
                    continue
                try:
                    parsed = parse_rx_frame(f["payload"])
                except Exception:
                    continue
                out.append(parsed.get("payload", b""))
        except Exception as exc:                              # pragma: no cover
            LOG.debug("command drain failed: %s", exc)
        return out

    def _dispatch_rx_pending(self, frames: list) -> None:
        """Dispatch RX frames harvested by a wait_for_tx_done rx_sink."""
        for f in frames:
            try:
                parsed = parse_rx_frame(f["payload"])
                self._dispatch_command(parsed.get("payload", b""))
            except Exception as exc:
                LOG.debug("rx_sink dispatch failed: %s", exc)
        frames.clear()

    def _dispatch_command(self, data: bytes) -> None:
        cmd = parse_command_frame(data)
        if cmd is None:
            return
        opcode, args = cmd
        client = self._mqtt_client
        if opcode == CMD_OP_REQ_KEYFRAME:
            LOG.info("LoRa cmd: REQ_KEYFRAME")
            if client is not None:
                client.publish(TRACTOR_KEYFRAME_TOPIC, b"\x01", qos=0)
        elif opcode == CMD_OP_ENCODE_MODE:
            if len(args) >= 1:
                mode = args[0]
                body = {"mode": int(mode)}
                # Optional second byte (2026-07-26): codec quality 1-100,
                # same frame. Old bases send 1-byte args; out-of-range
                # bytes are dropped rather than guessed at.
                if len(args) >= 2 and 1 <= args[1] <= 100:
                    body["quality"] = int(args[1])
                LOG.info("LoRa cmd: ENCODE_MODE %d%s", mode,
                         (" q=%d" % body["quality"]) if "quality" in body
                         else "")
                if client is not None:
                    import json as _json
                    client.publish(TRACTOR_ENCODE_MODE_TOPIC,
                                   _json.dumps(body),
                                   qos=0, retain=True)
        elif opcode == CMD_OP_RADIO_PROFILE:
            if len(args) >= 1 and args[0] in _PROFILE_CHOICES:
                profile = args[0]
                LOG.info("LoRa cmd: RADIO_PROFILE %d (two-phase)", profile)
                # Phase 1: ack on the CURRENT profile so the base knows we
                # heard it; the switch itself happens in _service_control
                # _plane after the ack flushes.
                try:
                    self._cmd_out.put_nowait(pack_command_frame(
                        CMD_OP_RADIO_PROFILE_ACK, bytes([profile])))
                except queue.Full:
                    pass
                with self.lock:
                    self._pending_profile = int(profile)
        elif opcode == CMD_OP_RADIO_PROFILE_CONF:
            LOG.info("LoRa cmd: RADIO_PROFILE_CONF — switch confirmed")
            self._confirm_deadline = None
            self._revert_profile = None
        elif opcode == CMD_OP_PROBE:
            # 2026-07-27 RS-0.12 reactive-fire instrumentation. A probe is a
            # no-op the tractor logs (with the base's commanded phase offset)
            # and echoes back so the base can measure in-stream delivery and
            # round-trip. THIS log line is the ground-truth delivery counter
            # — unlike REQ_KEYFRAME, nothing else can spuriously satisfy it.
            seq = int.from_bytes(args[0:4], "little") if len(args) >= 4 else 0
            phase = int.from_bytes(args[4:6], "little") if len(args) >= 6 else 0
            self._probe_rx = getattr(self, "_probe_rx", 0) + 1
            LOG.info("LoRa cmd: PROBE seq=%d phase_ms=%d (rx#%d)",
                     seq, phase, self._probe_rx)
            try:
                self._cmd_out.put_nowait(pack_command_frame(
                    CMD_OP_PROBE_ECHO, seq.to_bytes(4, "little")))
            except queue.Full:
                pass

    def _send_command_frame(self, link: HostLink, body: bytes,
                            copies: int = 2) -> None:
        """Fire-and-forget TX of one command frame (idempotent, tiny)."""
        rx_pending: list = []
        try:
            for _ in range(max(1, copies)):
                try:
                    tx_frame = bytes([0xFE, len(body)]) + body
                    link.send(HOST_TYPE_TX_FRAME_REQ, tx_frame)
                    wait_for_tx_done(link, 0xFE, timeout=2.0,
                                     rx_sink=rx_pending)
                except Exception as exc:
                    LOG.warning("command TX failed: %s", exc)
                    return
        finally:
            self._ensure_rxcont(link)
            # Frames captured during our own command TX (e.g. the base's
            # CONF landing while an ack radiates) are dispatched, not lost.
            self._dispatch_rx_pending(rx_pending)

    def _service_control_plane(self, link: HostLink) -> None:
        # Forward camera_service's encode ack over LoRa (latest wins).
        # Atomic swap under the lock: the paho thread writes _enc_ack_out,
        # and an unlocked read-then-clear here could drop an ack that
        # arrived between the two statements (2026-07-26).
        with self.lock:
            ack = self._enc_ack_out
            self._enc_ack_out = None
        if ack is not None:
            self._send_command_frame(link, pack_command_frame(
                CMD_OP_ENCODE_MODE_ACK, ack[:200]), copies=1)
            self._enc_ack_last_fwd = ack
        # Flush queued command frames (profile acks).
        flushed_profile_ack = False
        while True:
            try:
                body = self._cmd_out.get_nowait()
            except queue.Empty:
                break
            self._send_command_frame(link, body)
            if len(body) >= 2 and body[1] == CMD_OP_RADIO_PROFILE_ACK:
                flushed_profile_ack = True
        if flushed_profile_ack:
            # Phase 2: dwell so the last ack copy finishes radiating on
            # the old grid, then _maybe_switch_profile (caller) applies
            # the pending switch and arms the revert window.
            time.sleep(PROFILE_SWITCH_ACK_FLUSH_S)
        # Revert watchdog: no CONF on the new profile in time → the base
        # never made it over; return to the previous profile so the link
        # heals on the old grid.
        if (self._confirm_deadline is not None
                and time.monotonic() > self._confirm_deadline):
            revert_to = self._revert_profile
            self._confirm_deadline = None
            self._revert_profile = None
            if revert_to is not None and revert_to != self._active_profile:
                LOG.warning("radio_profile: no CONF within %.0f s — "
                            "reverting to %d", PROFILE_CONFIRM_TIMEOUT_S,
                            revert_to)
                self._apply_profile(link, revert_to)
                self._ack_profile(False, self._active_profile)

    def _maybe_switch_profile(self, link: HostLink) -> None:
        """Apply a pending profile change between frames (TX-worker thread).

        LoRa-only two-phase: the ACK was already flushed on the old grid
        (_service_control_plane); here we switch and arm the revert
        window — if the base's CONF frame doesn't arrive on the NEW
        profile within PROFILE_CONFIRM_TIMEOUT_S the watchdog reverts.
        """
        with self.lock:
            target = self._pending_profile
            self._pending_profile = None
        if target is None or target == self._active_profile:
            return
        previous = self._active_profile
        LOG.info("radio_profile: switching %d -> %d", previous, target)
        ok = self._apply_profile(link, target)
        if not ok:
            LOG.error("radio_profile: switch to %d FAILED; reverting to %d",
                      target, previous)
            if not self._apply_profile(link, previous):
                LOG.critical("radio_profile: revert to %d ALSO failed — "
                             "link needs operator attention", previous)
            self._ack_profile(False, self._active_profile)
            return
        # Armed: revert unless the base confirms on the new profile.
        self._revert_profile = previous
        self._confirm_deadline = time.monotonic() + PROFILE_CONFIRM_TIMEOUT_S
        self._ack_profile(True, self._active_profile)

    def _apply_profile(self, link: HostLink, profile: int) -> bool:
        try:
            os.environ["LIFETRAC_REG_PROFILE"] = str(profile)
            if profile == 1:
                os.environ["LIFETRAC_FHSS_WIDE_MASK"] = "1"
            else:
                os.environ.pop("LIFETRAC_FHSS_WIDE_MASK", None)
            drain_pending(link, quiet_s=0.2, max_s=1.0)
            configure_regulatory_profile_if_needed(link, profile)
            if os.environ.get("LIFETRAC_SKIP_PHY_CONTRACT", "0") != "1":
                verify_modem_matches_profile(
                    link, _PROFILE_TO_PHY.get(profile, PHY_IMAGE_BW250))
            self.budget = AirtimeBudget(
                budget_us=_PROFILE_TO_BUDGET_US.get(profile, 380_000))
            self._active_profile = profile
            self._publish_link_budget(profile)
            return True
        except Exception as exc:
            LOG.error("radio_profile: apply %d failed: %s", profile, exc)
            return False

    def _publish_link_budget(self, profile: int,
                             n_fragments: int = 1) -> None:
        """Tell camera_service the per-frame byte budget for this profile.

        Retained so a (re)starting camera_service picks it up immediately.
        n_fragments=1 makes every frame a single fragment — the natural unit
        under control-first TDMA; a unit file may widen it via
        LIFETRAC_IMAGE_FRAGMENTS_PER_FRAME for the parked/full-image mode.
        """
        client = self._mqtt_client
        if client is None:
            return
        # Parse defensively: this runs inside the paho on_connect callback
        # and inside _apply_profile, so a malformed env value must not raise
        # (it would abort connect — losing every subscription — or make a
        # profile switch report failure).
        _raw = os.environ.get("LIFETRAC_IMAGE_FRAGMENTS_PER_FRAME", "").strip()
        try:
            n = int(_raw) if _raw else int(n_fragments)
        except ValueError:
            LOG.warning("bad LIFETRAC_IMAGE_FRAGMENTS_PER_FRAME=%r — using %d",
                        _raw, n_fragments)
            n = int(n_fragments)
        idx = _PROFILE_TO_LINK_PHY_IDX.get(int(profile), 5)
        try:
            import json as _json
            client.publish(TRACTOR_LINK_BUDGET_TOPIC,
                           _json.dumps({"n_fragments": max(1, n),
                                        "profile_index": idx,
                                        "ts": round(time.time(), 1)}),
                           qos=0, retain=True)
            LOG.info("link_budget: published n_frag=%d profile_idx=%d "
                     "(profile %d)", max(1, n), idx, profile)
        except Exception as exc:                              # pragma: no cover
            LOG.debug("link_budget publish failed: %s", exc)

    def _ack_profile(self, ok: bool, active: int) -> None:
        client = self._mqtt_client
        if client is None:
            return
        try:
            import json as _json
            client.publish(RADIO_PROFILE_ACK_TOPIC,
                           _json.dumps({"ok": ok, "profile": active,
                                        "ts": round(time.time(), 1)}),
                           qos=0, retain=True)
        except Exception:                                     # pragma: no cover
            pass

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
            n_data = len(frags)
            frags = add_parity_fragments(frags, frame.seq, parity_group)
            # RS-4.1 observability (2026-07-26, run-T lesson): count what we
            # actually emit — "no reconstructions logged" must never again be
            # ambiguous between zero-function and zero-instrument.
            self._parity_frags_tx = (getattr(self, "_parity_frags_tx", 0)
                                     + len(frags) - n_data)
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

        if TX_PIPELINE == "v3":
            self._tx_fragments_pipelined(link, frame, fragments, active_phy,
                                         max_qos_retries, max_rf_retries)
            return

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
                rx_pending: list = []
                try:
                    tx_id = idx & 0xFF
                    tx_frame = bytes([tx_id, len(body)]) + body
                    link.send(HOST_TYPE_TX_FRAME_REQ, tx_frame)
                    # rx_sink (2026-07-26): commands arriving during the
                    # TX-verdict wait used to be consumed-and-dropped
                    # inside wait_for_tx_done — the v2-path half of the
                    # mid-burst command-discard bug (v3 got its fix on
                    # 2026-07-25; this is the DEFAULT path's).
                    done, faults = wait_for_tx_done(
                        link, tx_id, timeout=PER_FRAGMENT_TX_TIMEOUT_S,
                        rx_sink=rx_pending)
                except RuntimeError as exc:      # ERR_PROTO FORBIDDEN == QoS refusal:
                    self._dispatch_rx_pending(rx_pending)
                    qos_retries_left -= 1
                    if qos_retries_left < 0:
                        LOG.warning("QoS refusal cap reached for frag %d", idx)
                        break
                    time.sleep(est_us / 2e6)                   # cheap, no RF spent
                    continue
                except (TimeoutError, Exception) as exc:
                    # Commands captured before the failure are still valid
                    # — a refused/timed-out TX must not swallow them.
                    self._dispatch_rx_pending(rx_pending)
                    self.budget.record(est_us)                 # assume RF spent
                    rf_retries_left -= 1
                    if rf_retries_left < 0:
                        LOG.warning("RF send error for frag %d: %s", idx, exc)
                        break
                    continue                                   # retry

                actual_toa = done.get("time_on_air_us") or est_us
                self.budget.record(actual_toa)
                with self.lock:
                    self.toa_us_sum += actual_toa
                self._dispatch_rx_pending(rx_pending)
                if done["status"] == 0:  # SX1276_TX_STATUS_OK
                    sent = True
                    with self.lock:
                        self.fragments_tx_ok += 1
                        self.bytes_tx_ok += len(body)
                    # Command downlink: every TX parks the modem in
                    # STANDBY (raw-armed RXCONT is invisible to the
                    # firmware) — re-arm so the inter-fragment gap is
                    # listen time, and dispatch anything already heard.
                    self._ensure_rxcont(link)
                    for rx in self._drain_rx_frames(link, 0.0):
                        self._dispatch_command(rx)
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

    def _tx_fragments_pipelined(self, link: HostLink, frame: _PendingFrame,
                                fragments: list, active_phy,
                                max_qos_retries: int,
                                max_rf_retries: int) -> None:
        """v3 pipelined TX against the firmware depth-2 mailbox.

        Keeps up to PIPELINE_DEPTH TX_FRAME_REQs outstanding: while
        fragment N radiates, fragment N+1 is already parked in the
        L072's s_tx_pending mailbox (host_cmd.c), so the UART round
        trip + host turnaround ride inside N's time-on-air instead of
        adding dead air between fragments. Correlation:
          * TX_DONE_URC  -> in-flight fragment by payload tx_id
          * ERR_PROTO    -> in-flight fragment by request seq (QoS
                            refusal of the PARKED frame: no RF spent)
        Budget admission is per fragment, identical to the serial path,
        so the QoS gate still can never fire under nominal pacing.
        """
        n = len(fragments)
        inflight: dict = {}          # tx_id -> state dict
        seq_to_txid: dict = {}       # host-link seq -> tx_id
        next_i = 0
        aborted = False
        ok_count = 0

        def _submit(i: int, rf_left: int, qos_left: int) -> bool:
            body = fragments[i]
            est_us = int(lora_time_on_air_ms(len(body) + LORA_HOP_HDR_LEN,
                                             active_phy) * 1000)
            if not self.budget.admit(est_us, self._stop):
                return False                       # shutting down
            tx_id = i & 0xFF
            send_seq = link.send(HOST_TYPE_TX_FRAME_REQ,
                                 bytes([tx_id, len(body)]) + body)
            inflight[tx_id] = {
                "idx": i, "body": body, "est_us": est_us,
                "rf_left": rf_left, "qos_left": qos_left,
                "sent_at": time.monotonic(),
            }
            seq_to_txid[send_seq] = tx_id
            return True

        def _fail_fragment(tx_id: int, st: dict, why: str) -> None:
            nonlocal aborted
            with self.lock:
                self.fragments_tx_fail += 1
            LOG.warning("frag %d failed (%s); aborting frame seq=%d",
                        st["idx"], why, frame.seq)
            aborted = True

        while not self._stop.is_set() and (inflight or (next_i < n and not aborted)):
            while (not aborted and next_i < n
                   and len(inflight) < PIPELINE_DEPTH):
                if not _submit(next_i, max_rf_retries, max_qos_retries):
                    return                          # stop requested mid-frame
                next_i += 1

            if not inflight:
                break

            events = link.read_frames(0.05)
            now = time.monotonic()
            # RS-3.10: use the wait-loop's idle time to build train N+1.
            self._prep_next_train()

            # Watchdog: a fragment with no verdict inside the timeout is
            # treated as RF-spent (conservative for the budget mirror).
            for tx_id, st in list(inflight.items()):
                if now - st["sent_at"] > PER_FRAGMENT_TX_TIMEOUT_S:
                    self.budget.record(st["est_us"])
                    st["rf_left"] -= 1
                    del inflight[tx_id]
                    if st["rf_left"] >= 0 and not aborted:
                        if not _submit(st["idx"], st["rf_left"], st["qos_left"]):
                            return
                    else:
                        _fail_fragment(tx_id, st, "TX_DONE timeout")

            for fr in events:
                ftype = fr.get("type")
                if ftype == HOST_TYPE_TX_DONE_URC:
                    try:
                        done = parse_tx_done(fr["payload"])
                    except Exception:
                        continue
                    st = inflight.pop(done["tx_id"], None)
                    if st is None:
                        continue                    # stale URC from a past frame
                    self.budget.record(done.get("time_on_air_us") or st["est_us"])
                    with self.lock:
                        self.toa_us_sum += done.get("time_on_air_us") or st["est_us"]
                    if done["status"] == 0:
                        ok_count += 1
                        with self.lock:
                            self.fragments_tx_ok += 1
                            self.bytes_tx_ok += len(st["body"])
                    else:
                        st["rf_left"] -= 1
                        LOG.warning("TX_DONE non-OK: seq=%d idx=%d status=%d(%s)",
                                    frame.seq, st["idx"], done["status"],
                                    done.get("status_name", "?"))
                        if st["rf_left"] >= 0 and not aborted:
                            if not _submit(st["idx"], st["rf_left"], st["qos_left"]):
                                return
                        else:
                            _fail_fragment(done["tx_id"], st, "RF retries exhausted")
                elif ftype == HOST_TYPE_ERR_PROTO_URC:
                    tx_id = seq_to_txid.get(fr.get("seq"))
                    st = inflight.pop(tx_id, None) if tx_id is not None else None
                    if st is None:
                        LOG.warning("ERR_PROTO (uncorrelated): %s",
                                    format_err_proto_payload(fr["payload"]))
                        continue
                    st["qos_left"] -= 1           # refusal: zero RF was spent
                    if st["qos_left"] >= 0 and not aborted:
                        time.sleep(st["est_us"] / 2e6)
                        if not _submit(st["idx"], st["rf_left"], st["qos_left"]):
                            return
                    else:
                        _fail_fragment(tx_id, st, "QoS refusal cap reached")
                elif ftype == HOST_TYPE_FAULT_URC:
                    payload = fr.get("payload", b"")
                    code = payload[0] if payload else None
                    if code not in BENIGN_FAULT_CODES:
                        LOG.warning("FAULT during pipelined TX: %s",
                                    format_fault_payload(payload))
                elif ftype == HOST_TYPE_RX_FRAME_URC:
                    # RS-1.x (2026-07-25): inbound frames arriving MID-BURST
                    # were silently discarded here — with RS-4.12 arming the
                    # radio through every turnaround, THIS loop sees most
                    # inbound 0xFB command traffic (measured: 1/12 delivery
                    # in runs D and F, both explained by this drop). Dispatch,
                    # don't discard.
                    try:
                        parsed = parse_rx_frame(fr["payload"])
                        self._dispatch_command(parsed.get("payload", b""))
                    except Exception as exc:
                        LOG.debug("mid-burst RX dispatch failed: %s", exc)
                # RFCO_PERTX / STATS etc: informational, skip.

        with self.lock:
            if not aborted and ok_count == n:
                self.frames_tx_ok += 1
            else:
                self.frames_tx_fail += 1
        # Command downlink: restore RXCONT after the pipelined burst and
        # dispatch any command frames the drains queued up (see
        # _ensure_rxcont — raw-armed RXCONT drops on every TX).
        self._ensure_rxcont(link)
        for rx in self._drain_rx_frames(link, 0.0):
            self._dispatch_command(rx)
        if aborted:
            LOG.warning("frame seq=%d ABORTED (pipelined): %d/%d fragments ok",
                        frame.seq, ok_count, n)
        else:
            LOG.info("frame seq=%d done (pipelined): %d fragments ok",
                     frame.seq, ok_count)

    # ---- stats printer ----
    def _stats_worker(self, interval_s: float) -> None:
        last = 0.0
        last_bytes = 0
        last_toa_us = 0
        while not self._stop.is_set():
            time.sleep(1.0)
            now = time.monotonic()
            if now - last < interval_s:
                continue
            elapsed = (now - last) if last else interval_s
            last = now
            with self.lock:
                cur_bytes = self.bytes_tx_ok
                cur_toa = self.toa_us_sum
                bps = (cur_bytes - last_bytes) / elapsed
                util = ((cur_toa - last_toa_us) / 1e6) / elapsed
                LOG.info(
                    "stats: goodput=%.1f B/s util=%.0f%% pipeline=%s "
                    "frames_in=%d ok=%d fail=%d drop_full=%d drop_stale=%d "
                    "frags_ok=%d frags_fail=%d qdepth=%d parity_tx=%d",
                    bps, util * 100.0, TX_PIPELINE,
                    self.frames_in, self.frames_tx_ok, self.frames_tx_fail,
                    self.frames_dropped_queue_full, self.frames_dropped_stale,
                    self.fragments_tx_ok, self.fragments_tx_fail,
                    self._q.qsize(),
                    getattr(self, "_parity_frags_tx", 0))
                last_bytes = cur_bytes
                last_toa_us = cur_toa

    # ---- lifecycle ----
    def run(self, *, stats_interval_s: float) -> int:
        # paho-mqtt is the same dep camera_service.py uses; import inside
        # run() so synthetic-test imports of the module don't require it.
        import paho.mqtt.client as mqtt

        client = mqtt.Client(client_id=f"lifetrac-image-tx-{os.getpid()}")
        client.on_message = self._on_message
        # LoRa-only control plane: forward camera_service's LOCAL encode
        # ack over the radio (latest-wins snapshot; worker flushes it).

        def _on_enc_ack(_c, _u, msg):
            payload = bytes(msg.payload)[:200]
            # Retained replay identical to the last ack we radiated is the
            # MQTT reconnect echo, not a new ack from camera_service —
            # re-radiating it as 0x68 falsely "confirmed" whatever encode
            # command the base happened to be retrying (2026-07-26).
            if getattr(msg, "retain", False) and payload == self._enc_ack_last_fwd:
                return
            with self.lock:
                self._enc_ack_out = payload
        client.message_callback_add(TRACTOR_ENC_STATUS_TOPIC, _on_enc_ack)
        self._mqtt_client = client

        def _on_connect(_c, _u, _f, rc):
            if rc == 0:
                LOG.info("MQTT connected; subscribing to %s", MQTT_TOPIC_IN)
                client.subscribe(MQTT_TOPIC_IN, qos=0)
                client.subscribe(TRACTOR_ENC_STATUS_TOPIC, qos=0)
                # Seed the encoder's budget on connect (retained), so a
                # camera_service that started first still sizes to the air
                # quantum without waiting for a profile switch.
                self._publish_link_budget(self._active_profile)
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
