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
import queue
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

from lora_proto import (  # noqa: E402
    PHY_IMAGE_BW250,
    PHY_IMAGE_BW500,
    EncodeMode,
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
from image_pipeline.frame_format import encode_tile_delta_frame  # noqa: E402
from image_pipeline.reassemble import FragmentReassembler         # noqa: E402
from method_h_stage2_tx_probe_v2 import (  # noqa: E402
    HostLink,
    HOST_TYPE_VER_REQ,
    HOST_TYPE_VER_URC,
    HOST_TYPE_RX_FRAME_URC,
    HOST_TYPE_TX_FRAME_REQ,
    HOST_TYPE_CFG_SET_REQ,
    HOST_TYPE_CFG_OK_URC,
    CFG_KEY_LBT_ENABLE,
    parse_rx_frame,
    drain_boot,
    drain_pending,
    wait_for_tx_done,
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

# 2026-07-25 "LoRa-only tractor comms": the base->tractor control plane
# rides tiny 0xFB command frames transmitted by THIS daemon over its own
# L072 (the tractor has no LAN/IP reachability in the field). This daemon
# subscribes to the base-side control topics (web_ui publishes them) and
# converts each into a LoRa command; tractor acks come back as 0xFB
# frames on the image link and are republished retained for web_ui.
RADIO_PROFILE_TOPIC        = "lifetrac/v25/control/radio_profile"
RADIO_PROFILE_ACK_TOPIC    = "lifetrac/v25/status/radio_profile/rx"
RADIO_PROFILE_TX_ACK_TOPIC = "lifetrac/v25/status/radio_profile/tx"
ENCODE_MODE_OVERRIDE_TOPIC = "lifetrac/v25/control/encode_mode_override"
ENCODE_MODE_STATUS_TOPIC   = "lifetrac/v25/status/encode_mode"

_PROFILE_CHOICES = (0, 1, 2)
# Two-phase profile switch (base side): command the tractor, hold on the
# OLD profile until its ACK arrives (or give up), then switch locally;
# confirm back on the NEW profile at first proof-of-life, else revert.
# The ACK window covers several re-send volleys (tractor listens only in
# its inter-fragment gaps); the revert window covers worst-case FHSS
# re-acquisition after a switch INTO profile 1 (50-ch scan → ~25 s).
PROFILE_ACK_TIMEOUT_S     = 12.0
PROFILE_REVERT_TIMEOUT_S  = 45.0
# Keyframe-request commands are a convenience, not a control loop — a
# sparse cadence keeps the base's TX duty (and the run-33 bidirectional
# UART-stress window) tiny. The reassembler self-heals regardless.
KEYFRAME_CMD_MIN_GAP_S    = 10.0

# codec-id -> short name for link_stats (mirrors frame_format CODEC_*).
_CODEC_NAMES = {0: "webp", 1: "mono_g4", 2: "btc4_tile", 3: "btc4_frame",
                4: "webp_luma", 5: "webp_rawstream"}

_PROFILE_TO_PHY = {0: PHY_IMAGE_BW250, 1: PHY_IMAGE_BW250, 2: PHY_IMAGE_BW500}

# MQTT defaults; assume `adb reverse tcp:1883 tcp:1883` is wired so the
# X8 can reach the Windows host mosquitto via localhost.
DEFAULT_MQTT_HOST = "127.0.0.1"
DEFAULT_MQTT_PORT = 1883

DEFAULT_UART = "/dev/ttymxc3"
DEFAULT_BAUD = "921600"

# How long to wait per HostLink poll cycle when no frame is present.
RX_POLL_TIMEOUT_S = 0.25

# Completion-aligned command pump (RS-1.x). Set LIFETRAC_ALIGNED_PUMP=0 to
# fall back to idle-drain-only command TX — the behavior the 171/1
# convergence soak actually measured, kept as a bench A/B control.
ALIGNED_PUMP_ENABLE = os.environ.get("LIFETRAC_ALIGNED_PUMP", "1") != "0"


class KeyframeRequester:
    """Rate-limited req_keyframe on reassembly failure. Publishes to the
    same topic web_ui's manual button uses, so it inherits whatever
    broker relay the deployment already has for that path."""

    def __init__(self, client_supplier, min_interval_s: float = 5.0):
        self._client_supplier = client_supplier
        self._min = min_interval_s
        self._last = 0.0

    def poke(self, reason: str) -> None:
        # RS-4.14 diagnostic knob (2026-07-26): suppress self-heal keyframe
        # requests entirely. At FHSS, requests may FEED the loss they react
        # to (base command TX near a slot boundary -> follower
        # skip-under-tx-busy -> missed slot -> eviction -> another request).
        # A clean run with this =1 vs the normal run discriminates.
        if os.environ.get("LIFETRAC_KF_REQUEST_DISABLE", "0") == "1":
            return
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
        # 2026-07-25 encoder-selector implicit ACK: codec of the most
        # recently completed TileDeltaFrame (surfaced via link_stats).
        self._last_rx_codec: int | None = None
        self._last_rx_frame_kind: int | None = None
        # 2026-07-25 radio-profile selector: latest requested profile
        # (applied by the RX worker between poll cycles — the worker owns
        # the HostLink, MQTT callbacks must never touch it directly).
        self._pending_profile: int | None = None
        self._active_profile = int(os.environ.get("LIFETRAC_REG_PROFILE", "0"))
        # True once ANY retained radio_profile pin has been delivered to
        # this process — gates the age check so the boot replay (restart
        # convergence) is always honored and only reconnect replays age out.
        self._retained_profile_seen = False
        # 2026-07-27 RS-0.12 reactive-fire probe instrumentation (bench only).
        # LIFETRAC_REACTIVE_FIRE=1 fires a no-op PROBE at each fragment-RX-
        # complete (the moment the tractor's RXCONT re-arms), sweeping the
        # commanded delay so P(delivery | phase) can be measured. It uses the
        # SAME TX machinery a real ControlFrame would, so its delivery rate
        # IS the drive-command delivery rate. Off in production.
        self._reactive_fire = os.environ.get("LIFETRAC_REACTIVE_FIRE", "0") == "1"
        self._probe_seq = 0
        self._probe_tx = 0
        self._probe_echo_rx = 0
        self._probe_pending: "dict[int, float]" = {}   # seq -> monotonic send t
        self._probe_rtts: "list[float]" = []
        # Phase sweep: 0 = fire immediately at completion; else step through
        # the list (ms) round-robin so each offset bin gets samples.
        _sweep = os.environ.get("LIFETRAC_PROBE_PHASE_SWEEP_MS", "").strip()
        try:
            parsed_sweep = [max(0, int(x)) for x in _sweep.split(",")
                            if x.strip()]
        except ValueError:
            LOG.warning("bad LIFETRAC_PROBE_PHASE_SWEEP_MS %r — using [0]",
                        _sweep)
            parsed_sweep = []
        # `or [0]`: an empty/blank env (or a bad parse) falls back to a
        # single offset-0 bin (fire immediately at completion).
        self._probe_sweep = parsed_sweep or [0]
        self._probe_min_gap_s = float(os.environ.get(
            "LIFETRAC_PROBE_MIN_GAP_S", "0.5"))
        self._last_probe_t = 0.0
        # LoRa-only control plane: outbound command frames queued by MQTT
        # callbacks, drained by the RX worker (single link owner). Plus
        # the two-phase switch state machine fields.
        self._ctrl_out: "queue.Queue[bytes]" = queue.Queue(maxsize=16)
        # RS-1.5 (2026-07-26): ack-driven command convergence. Keyed by
        # opcode; the pump re-sends one copy per aligned window until the
        # command's ACK OBSERVABLE fires (keyframe arrival / ENCODE ACK)
        # or the caps trip. Replaces fire-and-forget ×2 copies for the
        # opcodes wired to it (generalizes the profile-switch Phase-A
        # retry pattern). Guarded by self._lock.
        self._pending_cmds: "dict[int, dict]" = {}
        self._await_ack_profile: int | None = None
        self._await_ack_deadline: float = 0.0
        self._await_ack_last_send: float = 0.0
        self._confirm_pending = False
        self._revert_deadline: float = 0.0
        self._revert_to: int | None = None
        self._last_kf_cmd_t = 0.0
        # Optional split-broker control plane: the bench base board
        # publishes video on its LOCAL mosquitto while web_ui (host)
        # publishes control on the HOST broker. When set, a second MQTT
        # client subscribes to control topics there and mirrors
        # link_stats/acks so the web UI sees them.
        self._ctrl_client = None
        self.ctrl_mqtt_host = os.environ.get(
            "LIFETRAC_CTRL_MQTT_HOST", "").strip() or None

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
        # VER warm-up with retry (mirrors image_tx_daemon 2026-07-24): a
        # freshly hard-reset L072 can miss the first request while it
        # drains its boot burst; a single 1.0 s attempt produced spurious
        # "cannot open HostLink" fatals during the saturation bench.
        # VER_REQ is idempotent, so retrying is safe.
        last_exc = None
        for attempt in range(1, 6):
            try:
                link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=2.0)
                LOG.info("L072 VER warm-up ok (attempt %d)", attempt)
                last_exc = None
                break
            except Exception as exc:
                last_exc = exc
                LOG.warning("VER warm-up attempt %d/5 failed: %s", attempt, exc)
        if last_exc is not None:
            LOG.error("VER warm-up failed: %s", last_exc)
            raise last_exc
        drain_pending(link, quiet_s=0.25, max_s=1.0)
        try:
            configure_regulatory_profile_if_needed(link)
        except Exception as exc:                              # pragma: no cover
            LOG.warning("regulatory profile config failed: %s", exc)
        # 2026-07-25 run-32 root cause: this daemon now TRANSMITS (0xFB
        # command uplink) and the firmware boots with LBT enabled — with
        # the tractor streaming at ~58 % duty the channel always sounds
        # busy, so every command TX died ABORT_LBT/FORBIDDEN. Disable LBT
        # exactly like image_tx_daemon does (W1-10b TX_BURST rationale).
        try:
            link.request(HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
                         bytes([CFG_KEY_LBT_ENABLE, 0x01, 0x00]), timeout=1.0)
            LOG.info("LBT_ENABLE=0 (command uplink shares the image channel)")
        except Exception as exc:                              # pragma: no cover
            LOG.warning("CFG_SET(LBT_ENABLE=0) failed: %s — command TX may "
                        "abort under load", exc)
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

    def _maybe_switch_profile(self, link: HostLink) -> None:
        """Drive the LoRa-only two-phase profile switch (RX worker thread).

        Phase A — a request is pending: transmit the RADIO_PROFILE command
        over the air and hold on the CURRENT profile awaiting the
        tractor's ACK frame (it acks before switching, on the old grid).
        Phase B — ACK seen (_handle_command): switch locally, then expect
        proof-of-life on the new grid; first RX frame triggers the CONF
        command back to the tractor. No proof within the window → revert
        (the tractor's own CONF watchdog reverts it independently, so
        both ends heal on the old grid without further coordination).
        """
        now = time.monotonic()
        # RS-1.x (2026-07-26 Run I forensics): the queue drain that lived
        # here ran on EVERY loop pass and silently defeated the aligned
        # pump — the second enqueued copy drained ~37 ms behind the pump's
        # send, into the same stale window. Command draining now happens
        # ONLY via the aligned+spaced pump (busy link) or
        # _drain_ctrl_idle (quiet link). This method keeps its actual
        # job: the two-phase profile-switch state machine.
        # Phase-A: while awaiting the tractor's ACK, re-send the command
        # every ~1.5 s — the tractor listens only in its inter-fragment
        # gaps, so a single volley has a real chance of landing while
        # its modem is keyed. Idempotent, tiny, worth the airtime.
        if self._await_ack_profile is not None:
            if now > self._await_ack_deadline:
                LOG.warning("radio_profile: no tractor ACK within %.0f s — "
                            "staying on profile %d", PROFILE_ACK_TIMEOUT_S,
                            self._active_profile)
                self._await_ack_profile = None
                self._ack_profile(False, self._active_profile,
                                  error="no tractor ack")
            elif now - self._await_ack_last_send > 1.5:
                self._await_ack_last_send = now
                self._send_command_frame(link, pack_command_frame(
                    CMD_OP_RADIO_PROFILE,
                    bytes([self._await_ack_profile])))
        # Revert watchdog: switched locally but never heard the tractor
        # on the new grid.
        if self._confirm_pending and now > self._revert_deadline:
            revert_to = self._revert_to
            self._confirm_pending = False
            self._revert_to = None
            if revert_to is not None and revert_to != self._active_profile:
                LOG.warning("radio_profile: no frames on new profile within "
                            "%.0f s — reverting to %d",
                            PROFILE_REVERT_TIMEOUT_S, revert_to)
                self._apply_profile(link, revert_to)
                self._ack_profile(False, self._active_profile,
                                  error="reverted: link silent")
        # New operator/auto request → Phase A.
        with self._lock:
            target = self._pending_profile
            self._pending_profile = None
        if target is None or target == self._active_profile:
            return
        LOG.info("radio_profile: commanding tractor %d -> %d (two-phase)",
                 self._active_profile, target)
        self._send_command_frame(
            link, pack_command_frame(CMD_OP_RADIO_PROFILE, bytes([target])))
        self._await_ack_profile = target
        self._await_ack_deadline = now + PROFILE_ACK_TIMEOUT_S
        self._await_ack_last_send = now

    def _ensure_rxcont(self, link: HostLink) -> None:
        """Re-arm RXCONT if our own TX dropped it (run-31 root cause).

        The daemons arm RXCONT via RAW opmode register writes, so the
        L072 firmware's tracked state never records RX-armed. Its TX
        path therefore takes the standby branch and leaves the radio in
        STANDBY after TX_DONE — one command transmission deafened the
        base for the whole of run 31 (rx_frames=1/300 s). After ANY
        local TX, put the modem back into RXCONT ourselves.
        """
        try:
            opm, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
            if opm != SX1276_OPMODE_LORA_RXCONT:
                write_reg(link, SX1276_REG_OP_MODE,
                          SX1276_OPMODE_LORA_RXCONT, timeout=0.5)
                LOG.debug("RXCONT re-armed (opmode 0x%02x -> 0x85)", opm)
        except Exception as exc:                              # pragma: no cover
            LOG.warning("RXCONT re-arm failed: %s", exc)

    def _set_pending(self, opcode: int, body: bytes,
                     timeout_s: float = 10.0, max_attempts: int = 20) -> None:
        """Register (or refresh) an ack-driven command.

        Same opcode + same body: just extend the deadline (idempotent
        re-trigger). Same opcode + DIFFERENT body (e.g. a rapid encode
        A→B change): replace the payload and restart the retry budget —
        before 2026-07-26 the new body was silently discarded and the
        pump kept retrying the stale command.
        """
        now = time.monotonic()
        with self._lock:
            cur = self._pending_cmds.get(opcode)
            if cur is not None:
                if cur["body"] == body:
                    cur["deadline"] = now + timeout_s
                    return
                LOG.info("cmd 0x%02x superseded while pending (after %d "
                         "attempts) — body replaced, retry budget reset",
                         opcode, cur["attempts"])
            self._pending_cmds[opcode] = {
                "body": body, "attempts": 0, "t0": now, "last_send": 0.0,
                "deadline": now + timeout_s, "max_attempts": max_attempts,
            }

    def _clear_pending(self, opcode: int, reason: str) -> None:
        with self._lock:
            cur = self._pending_cmds.pop(opcode, None)
        if cur is not None:
            LOG.info("cmd 0x%02x CONVERGED after %d attempts (%.1f s): %s",
                     opcode, cur["attempts"],
                     time.monotonic() - cur["t0"], reason)

    # Minimum spacing between retries of the SAME pending command: keeps
    # the 20-attempt budget spread across the whole 10 s deadline instead
    # of exhausting in ~5 s when completions (or idle polls) come fast.
    PENDING_RETRY_MIN_GAP_S = 0.4

    def _next_ctrl_body(self, now: float) -> "bytes | None":
        """Pump source: pending (ack-driven) commands first, oldest first;
        legacy one-shot queue entries second. Enforces give-up caps."""
        with self._lock:
            for opcode in sorted(self._pending_cmds,
                                 key=lambda k: self._pending_cmds[k]["t0"]):
                cur = self._pending_cmds[opcode]
                if cur["attempts"] >= cur["max_attempts"] or now > cur["deadline"]:
                    del self._pending_cmds[opcode]
                    LOG.warning("cmd 0x%02x GAVE UP after %d attempts (%.1f s)",
                                opcode, cur["attempts"], now - cur["t0"])
                    continue
                if now - cur.get("last_send", 0.0) < self.PENDING_RETRY_MIN_GAP_S:
                    continue
                cur["attempts"] += 1
                cur["last_send"] = now
                return cur["body"]
        try:
            return self._ctrl_out.get_nowait()
        except queue.Empty:
            return None

    def _drain_ctrl_idle(self, link: HostLink) -> None:
        """Idle-link fallback: drain queued commands when no image stream
        is flowing (the tractor is listening continuously then, so timing
        does not matter). During a stream the ONLY drain is the aligned,
        ≥120 ms-spaced pump in the RX loop."""
        # Pending (ack-driven) commands: one retry per idle pass — the
        # loop's ~0.25 s read timeout is the natural retry cadence, and
        # an idle tractor listens continuously so timing is free.
        body = self._next_ctrl_body(time.monotonic())
        while body is not None:
            self._send_command_frame(link, body, copies=1)
            # Only continue draining the LEGACY queue in one go; pending
            # entries retry across passes, not in a tight loop.
            try:
                body = self._ctrl_out.get_nowait()
            except queue.Empty:
                return

    def _send_command_frame(self, link: HostLink, body: bytes,
                            copies: int = 2) -> None:
        """TX one 0xFB command frame via this daemon's L072 (idempotent).

        2026-07-25 RS-1.1 instrumentation: every attempt is logged with the
        opcode and outcome. A logged OK means the L072 reported TX_DONE —
        the frame went ON AIR — so "never sent" vs "sent and lost at the
        tractor" is now distinguishable from this log alone (the 2026-07-25
        re-baseline runs could not tell the two apart).
        """
        op = body[1] if len(body) > 1 else 0
        n = max(1, copies)
        try:
            for i in range(n):
                try:
                    tx_frame = bytes([0xFD, len(body)]) + body
                    link.send(HOST_TYPE_TX_FRAME_REQ, tx_frame)
                    wait_for_tx_done(link, 0xFD, timeout=2.0)
                    self._cmd_tx_ok = getattr(self, "_cmd_tx_ok", 0) + 1
                    LOG.info("command TX opcode=0x%02x copy=%d/%d OK (on air)",
                             op, i + 1, n)
                except Exception as exc:
                    self._cmd_tx_fail = getattr(self, "_cmd_tx_fail", 0) + 1
                    LOG.warning("command TX opcode=0x%02x copy=%d/%d FAILED: %s",
                                op, i + 1, n, exc)
                    return
        finally:
            # CRITICAL: our TX parks the modem in STANDBY (see
            # _ensure_rxcont docstring) — re-arm or go deaf.
            self._ensure_rxcont(link)

    def _handle_command(self, link: HostLink, opcode: int,
                        args: bytes) -> None:
        """Inbound 0xFB frames from the tractor (acks/status)."""
        if opcode == CMD_OP_RADIO_PROFILE_ACK:
            # Synthesize the tractor-side ack for web_ui from the LoRa
            # frame — in the field there is NO LAN path for the tractor
            # to publish its own status topic to the base broker.
            if len(args) >= 1:
                import json as _json
                tx_ack = _json.dumps({"ok": True, "profile": int(args[0]),
                                      "via": "lora",
                                      "ts": round(time.time(), 1)})
                for c in (self._client, self._ctrl_client):
                    if c is None:
                        continue
                    try:
                        c.publish(RADIO_PROFILE_TX_ACK_TOPIC, tx_ack,
                                  qos=0, retain=True)
                    except Exception:                         # pragma: no cover
                        pass
            if (self._await_ack_profile is not None and len(args) >= 1
                    and args[0] == self._await_ack_profile):
                target = self._await_ack_profile
                self._await_ack_profile = None
                previous = self._active_profile
                LOG.info("radio_profile: tractor ACK for %d — switching "
                         "locally", target)
                if self._apply_profile(link, target):
                    self._confirm_pending = True
                    self._revert_to = previous
                    self._revert_deadline = (time.monotonic()
                                             + PROFILE_REVERT_TIMEOUT_S)
                else:
                    self._apply_profile(link, previous)
                    self._ack_profile(False, self._active_profile,
                                      error="local apply failed")
            elif (self._await_ack_profile is None and len(args) >= 1
                    and args[0] in _PROFILE_CHOICES
                    and args[0] != self._active_profile):
                # LATE ACK (run-33 split-link freeze): the tractor heard a
                # command copy AFTER our ACK window expired and committed
                # to the new profile while we stayed behind — the link is
                # split and the stream is dead until someone moves. The
                # tractor is already there; follow it (cheaper and faster
                # than waiting out its 45 s CONF-revert), with the same
                # proof-of-life/revert protection as the normal path.
                target = int(args[0])
                previous = self._active_profile
                LOG.warning("radio_profile: LATE tractor ACK for %d — "
                            "following to heal the split", target)
                if self._apply_profile(link, target):
                    self._confirm_pending = True
                    self._revert_to = previous
                    self._revert_deadline = (time.monotonic()
                                             + PROFILE_REVERT_TIMEOUT_S)
        elif opcode == CMD_OP_PROBE_ECHO:
            # 2026-07-27 RS-0.12: the tractor received our probe and echoed
            # it. This is a CONFIRMED in-stream round trip — the honest
            # delivery signal the contaminated keyframe-ack never was.
            seq = int.from_bytes(args[0:4], "little") if len(args) >= 4 else 0
            t0 = self._probe_pending.pop(seq, None)
            self._probe_echo_rx += 1
            if t0 is not None:
                rtt_ms = (time.monotonic() - t0) * 1000.0
                self._probe_rtts.append(rtt_ms)
                LOG.info("PROBE ECHO seq=%d rtt=%.0f ms (echo#%d, delivered "
                         "%d/%d)", seq, rtt_ms, self._probe_echo_rx,
                         self._probe_echo_rx, self._probe_tx)
            else:
                LOG.info("PROBE ECHO seq=%d (unmatched; echo#%d)", seq,
                         self._probe_echo_rx)
        elif opcode == CMD_OP_ENCODE_MODE_ACK:
            # RS-1.5: explicit ack — stop retrying ENCODE_MODE, but ONLY
            # if the ack matches the mode we are retrying. Before
            # 2026-07-26 ANY 0x68 cleared the retry — including the stale
            # retained ack the tractor re-radiates on every MQTT
            # reconnect — falsely "confirming" a command that never
            # landed. An ack for a different mode leaves the retry alive.
            # Compare-and-pop under ONE lock acquisition: the ack is parsed
            # outside the lock, but the match is re-checked against the
            # entry we actually remove. Dropping the lock between "read
            # body" and "pop" let a paho-thread _set_pending slip a NEW
            # command into the slot that then got popped unacked.
            ack = self._parse_encode_ack(args)
            if ack is None:
                LOG.warning("ENCODE_MODE_ACK unparseable (%r) — keeping "
                            "retry pending", args[:64])
            else:
                with self._lock:
                    cur = self._pending_cmds.get(CMD_OP_ENCODE_MODE)
                    matched = (cur is not None
                               and self._ack_matches_body(ack, cur["body"]))
                    if matched:
                        del self._pending_cmds[CMD_OP_ENCODE_MODE]
                if cur is not None:
                    if matched:
                        LOG.info("cmd 0x%02x CONVERGED after %d attempts "
                                 "(%.1f s): ENCODE_MODE_ACK %r",
                                 CMD_OP_ENCODE_MODE, cur["attempts"],
                                 time.monotonic() - cur["t0"], ack)
                    else:
                        LOG.info("ENCODE_MODE_ACK %r does not match pending "
                                 "%s — still retrying", ack,
                                 cur["body"][2:].hex())
            # Tractor's encode ack, forwarded over the air — republish
            # retained so web_ui's cache works with zero LAN to the
            # tractor. Payload is the camera_service JSON as-is.
            for c in (self._client, self._ctrl_client):
                if c is None:
                    continue
                try:
                    c.publish(ENCODE_MODE_STATUS_TOPIC, args,
                              qos=0, retain=True)
                except Exception:                             # pragma: no cover
                    pass

    # camera_service clamps an operator quality request into this range
    # before applying it, and echoes the CLAMPED value in its ack — so the
    # base must clamp identically to know what a matching ack looks like.
    TRACTOR_QUALITY_MIN = 20
    TRACTOR_QUALITY_MAX = 100

    @staticmethod
    def _parse_encode_ack(args: bytes) -> "dict | None":
        """Extract {mode, quality} from a 0x68 ack JSON body (None = unparseable)."""
        try:
            import json as _json
            body = _json.loads(args.decode("utf-8"))
        except Exception:
            return None
        if not isinstance(body, dict):
            return None
        mode = None
        for key in ("requested", "requested_mode", "mode", "effective"):
            val = body.get(key)
            if isinstance(val, int):
                mode = val
                break
        if mode is None:
            return None
        quality = body.get("quality")
        return {"mode": mode,
                "quality": quality if isinstance(quality, int) else None}

    @classmethod
    def _ack_matches_body(cls, ack: dict, body: bytes) -> bool:
        """Does this 0x68 ack confirm the pending 0x63 frame ``body``?

        Body layout: [0xFB, 0x63, mode] or [0xFB, 0x63, mode, quality].
        The mode must always match. When we commanded a QUALITY too, the
        ack's effective quality must equal our request clamped the way the
        tractor clamps it — otherwise an in-flight ack from the PREVIOUS
        (same mode, old quality) command would falsely confirm the new one
        and the quality change would be silently dropped.
        """
        if len(body) < 3 or ack.get("mode") != body[2]:
            return False
        if len(body) < 4:
            return True                    # mode-only command: no q to check
        want = max(cls.TRACTOR_QUALITY_MIN,
                   min(cls.TRACTOR_QUALITY_MAX, body[3]))
        got = ack.get("quality")
        if got is None:
            # Old tractor build with no quality in its ack: it cannot
            # confirm the quality half, so keep retrying rather than
            # claim a convergence we did not observe.
            return False
        return got == want

    def _note_proof_of_life(self, link: HostLink) -> None:
        """First frame on a freshly switched profile → confirm to tractor."""
        if not self._confirm_pending:
            return
        self._confirm_pending = False
        self._revert_to = None
        LOG.info("radio_profile: proof-of-life on profile %d — sending CONF",
                 self._active_profile)
        self._send_command_frame(link, pack_command_frame(
            CMD_OP_RADIO_PROFILE_CONF, bytes([self._active_profile])))
        self._ack_profile(True, self._active_profile)

    def _apply_profile(self, link: HostLink, profile: int) -> bool:
        try:
            os.environ["LIFETRAC_REG_PROFILE"] = str(profile)
            # Firmware profile-1 validator rejects popcount<50, so the
            # wide mask must accompany FHSS; single-carrier profiles
            # narrow the mask (configure's default) for determinism.
            if profile == 1:
                os.environ["LIFETRAC_FHSS_WIDE_MASK"] = "1"
            else:
                os.environ.pop("LIFETRAC_FHSS_WIDE_MASK", None)
            drain_pending(link, quiet_s=0.2, max_s=1.0)
            configure_regulatory_profile_if_needed(link, profile)
            if os.environ.get("LIFETRAC_SKIP_PHY_CONTRACT", "0") != "1":
                verify_modem_matches_profile(
                    link, _PROFILE_TO_PHY.get(profile, PHY_IMAGE_BW250))
            # Re-arm RXCONT: profile activation sequences through
            # standby; a receiver must never be left parked there.
            opm, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
            if opm != SX1276_OPMODE_LORA_RXCONT:
                write_reg(link, SX1276_REG_OP_MODE,
                          SX1276_OPMODE_LORA_RXCONT, timeout=0.5)
                LOG.info("radio_profile: RXCONT re-armed (opmode 0x%02x -> 0x85)",
                         opm)
            self._active_profile = profile
            return True
        except Exception as exc:
            LOG.error("radio_profile: apply %d failed: %s", profile, exc)
            return False

    def _ack_profile(self, ok: bool, active: int,
                     error: str | None = None) -> None:
        import json as _json
        body = {"ok": ok, "profile": active, "ts": round(time.time(), 1)}
        if error:
            body["error"] = error
        ack = _json.dumps(body)
        for c in (self._client, self._ctrl_client):
            if c is None:
                continue
            try:
                c.publish(RADIO_PROFILE_ACK_TOPIC, ack, qos=0, retain=True)
            except Exception:                                 # pragma: no cover
                pass

    # Retained control pins older than this are ignored on a RECONNECT
    # (run-V incident: a stale retained {"profile": N} re-commanded the
    # tractor mid-run when the daemon's MQTT session bounced). Publishers
    # stamp "ts"; a payload without one is treated as live for backward
    # compatibility.
    MAX_RETAINED_CMD_AGE_S = 600.0

    def _retained_and_stale(self, msg, body: dict) -> bool:
        """Should this retained pin be ignored?

        The FIRST retained delivery after process start is always honored
        regardless of age — that replay is how a restarting daemon
        converges to the operator's standing selection, and it is the
        pre-2026-07-26 behavior the age gate must not break. Only LATER
        replays (i.e. a mid-run broker reconnect, the run-V case) are
        age-gated, because by then we already hold a live profile that a
        stale pin must not override.
        """
        if not getattr(msg, "retain", False):
            return False
        if not self._retained_profile_seen:
            LOG.info("accepting first retained %s pin after start "
                     "(restart convergence)", msg.topic)
            return False
        ts = body.get("ts")
        if not isinstance(ts, (int, float)):
            return False
        age = time.time() - ts
        if age > self.MAX_RETAINED_CMD_AGE_S:
            LOG.warning("ignoring retained %s pin aged %.0f s (> %.0f s) on "
                        "reconnect", msg.topic, age,
                        self.MAX_RETAINED_CMD_AGE_S)
            return True
        return False

    def _on_radio_profile_msg(self, _c, _u, msg) -> None:
        try:
            import json as _json
            body = _json.loads(msg.payload.decode("utf-8") or "{}")
            profile = int(body.get("profile"))
        except (ValueError, TypeError, UnicodeDecodeError) as exc:
            LOG.warning("radio_profile: bad payload %r (%s)",
                        msg.payload[:64], exc)
            return
        if profile not in _PROFILE_CHOICES:
            LOG.warning("radio_profile: unknown profile %d", profile)
            return
        stale = isinstance(body, dict) and self._retained_and_stale(msg, body)
        if getattr(msg, "retain", False):
            self._retained_profile_seen = True
        if stale:
            return
        with self._lock:
            self._pending_profile = profile

    # Wire-valid encode modes (values only — the tractor clamps
    # unimplemented ones itself and says so in the 0x68 ack).
    _VALID_ENCODE_MODES = frozenset(int(m.value) for m in EncodeMode)

    def _on_encode_mode_msg(self, _c, _u, msg) -> None:
        """control/encode_mode_override → LoRa CMD_OP_ENCODE_MODE.

        Payload {"mode": <name>|<int>, "quality": 1-100?, "ts": ...}.
        Quality rides as an optional second args byte on the same 0x63
        frame (old tractors ignore it; omitting it keeps the tractor's
        current quality).
        """
        try:
            import json as _json
            body = _json.loads(msg.payload.decode("utf-8") or "{}")
            requested = body.get("mode")
            if requested in (None, "", "auto"):
                return
            if isinstance(requested, str):
                mode = int(EncodeMode[requested.upper()].value)
            else:
                mode = int(requested)
        except (ValueError, KeyError, TypeError, UnicodeDecodeError) as exc:
            LOG.warning("encode_mode_override: bad payload %r (%s)",
                        msg.payload[:64], exc)
            return
        if mode not in self._VALID_ENCODE_MODES:
            # Reject instead of masking &0xFF onto the air — an arbitrary
            # byte here would be retried for 10 s against a tractor that
            # can only clamp what it can parse.
            LOG.warning("encode_mode_override: mode %d outside EncodeMode — "
                        "rejected", mode)
            return
        quality = body.get("quality") if isinstance(body, dict) else None
        args = bytes([mode])
        if quality is not None:
            try:
                q = int(quality)
            except (TypeError, ValueError):
                q = -1
            if 1 <= q <= 100:
                args = bytes([mode, q])
            else:
                LOG.warning("encode_mode_override: quality %r out of range — "
                            "sending mode only", quality)
        # RS-1.5: retry until the tractor's ENCODE_MODE_ACK arrives.
        self._set_pending(
            CMD_OP_ENCODE_MODE,
            pack_command_frame(CMD_OP_ENCODE_MODE, args))

    def _on_req_keyframe_msg(self, _c, _u, _msg) -> None:
        """cmd/req_keyframe (self-heal poke or web button) → LoRa cmd."""
        # RS-4.14 diagnostic (2026-07-26): gate at the SUBSCRIPTION — the
        # choke point ALL request sources pass through. Run P proved the
        # poke-only gate insufficient: the (22-commit-stale) web_ui's own
        # I-frame recovery logic published 109 requests from outside the
        # daemon, contaminating the commands-off experiment. That web_ui
        # is an unaccounted control-plane actor is itself an RS-9 finding.
        if os.environ.get("LIFETRAC_KF_REQUEST_DISABLE", "0") == "1":
            return
        now = time.monotonic()
        if now - self._last_kf_cmd_t < KEYFRAME_CMD_MIN_GAP_S:
            return
        self._last_kf_cmd_t = now
        # RS-1.5: retry until a keyframe actually arrives — the keyframe
        # IS the ack (cleared in the completed-frame handler).
        self._set_pending(CMD_OP_REQ_KEYFRAME,
                          pack_command_frame(CMD_OP_REQ_KEYFRAME))

    def _subscribe_control(self, client) -> None:
        client.message_callback_add(
            RADIO_PROFILE_TOPIC, self._on_radio_profile_msg)
        client.message_callback_add(
            ENCODE_MODE_OVERRIDE_TOPIC, self._on_encode_mode_msg)
        client.message_callback_add(
            KEYFRAME_REQ_TOPIC, self._on_req_keyframe_msg)

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
                self._drain_ctrl_idle(link)
                self._maybe_switch_profile(link)
                # Defensive: heal a silently dropped RXCONT (throttled).
                now_arm = time.monotonic()
                if now_arm - getattr(self, "_last_arm_check", 0.0) > 5.0:
                    self._last_arm_check = now_arm
                    self._ensure_rxcont(link)
                continue

            saw_rx = False       # (kept for logging/diagnostics)
            frame_done = False   # RS-1.x: did a frame COMPLETE this pass?
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
                # 2026-07-25 LoRa control plane: command/ack frames are
                # dispatched BEFORE the image pipeline (the reassembler
                # never sees the 0xFB magic).
                cmd = parse_command_frame(data)
                if cmd is not None:
                    self._handle_command(link, cmd[0], cmd[1])
                    continue
                # Any valid frame on a freshly switched profile is proof
                # the tractor made it over — confirm + disarm the revert.
                self._note_proof_of_life(link)
                # 2026-07-25 RS-2.3 forensics: inter-arrival gap between raw
                # fragments from the firmware RX timestamp (u32 µs, wraps
                # ~71 min — wrap-safe subtract). At saturation the median
                # delta minus the median fragment's ToA IS the per-fragment
                # dead time we are hunting; deltas >5 s are stream pauses,
                # not gaps, and are discarded.
                ts = parsed.get("timestamp_us")
                if ts is not None:
                    prev = getattr(self, "_gap_prev_ts", None)
                    self._gap_prev_ts = ts
                    if prev is not None:
                        d = (ts - prev) & 0xFFFFFFFF
                        if 0 < d < 5_000_000:
                            if not hasattr(self, "_gap_samples"):
                                self._gap_samples = []
                            self._gap_samples.append((d, len(data)))

                saw_rx = True
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
                    # RS-1.x: a COMPLETED frame means that was the train's
                    # LAST fragment — the tractor's ~44 ms host-turnaround
                    # window opens NOW (mid-train the RS-4.12 ring restarts
                    # TX within ~5 ms, so any-fragment alignment — Run E's
                    # mistake — is no better than chance).
                    frame_done = True
                    # RS-3.1: a batched payload completes into a LIST of
                    # frames; bare payloads stay a single frame. Publish
                    # each — stats and web_ui see individual frames.
                    done_list = (completed if isinstance(completed, list)
                                 else [completed])
                    for done in done_list:
                        try:
                            payload_out = encode_tile_delta_frame(done)
                        except Exception as exc:
                            LOG.warning("encode_tile_delta_frame failed: %s",
                                        exc)
                            continue
                        frame_id = getattr(done, "frame_id", 0) or 0
                        with self._lock:
                            # Implicit encoder-mode ACK: frames self-describe
                            # their codec; the base UI compares this against
                            # the operator's pin (keyframe-forced on every
                            # mode change, so it converges within one frame).
                            self._last_rx_codec = getattr(done, "codec", 0)
                            self._last_rx_frame_kind = getattr(
                                done, "frame_kind", 0)
                        # RS-1.5: a keyframe arriving clears a pending
                        # req_keyframe. 2026-07-27 CAUTION: this is a
                        # CONTAMINATED delivery signal — the synth/encoder
                        # emits keyframes on its own cadence, so a keyframe
                        # arriving does NOT prove our request was delivered
                        # (this produced the retracted 171/1 result). The
                        # clear is logged as UNVERIFIED, and _clear_pending
                        # says so, so no future analysis mistakes it for a
                        # confirmed round trip. Use CMD_OP_PROBE for honest
                        # delivery measurement.
                        if getattr(done, "frame_kind", 0) == 1:
                            self._clear_pending(
                                CMD_OP_REQ_KEYFRAME,
                                "keyframe received (UNVERIFIED delivery — "
                                "keyframe may be encoder-initiated)")
                        self._publish(payload_out, frame_id)

            # RS-1.x window-aligned command TX (2026-07-25): a fragment in
            # this pass means the tractor JUST finished a TX — its armed
            # RXCONT turnaround window (~44 ms measured, Run D) is open
            # right now. Send ONE queued command copy per window; the ×2
            # redundancy is spread across independent windows at enqueue.
            # Run D showed unaligned delivery ≈ window-alignment odds
            # (2/12); aligned TX should approach deterministic.
            # 2026-07-26 pump-gate fix: the gate used to check only the
            # legacy _ctrl_out queue, but every subscribed command now
            # registers via _set_pending — so the aligned pump NEVER
            # fired and delivery leaned entirely on the idle drain
            # between trains. Gate on either source.
            #
            # NOTE (air-test pending, RS-0.9): the 171/1 convergence soak
            # was measured with this pump dead, so the aligned path is
            # UNPROVEN on air. Our command TX makes the base deaf for its
            # ToA plus the host round trip; if that overruns the tractor's
            # ~40 ms reverse-slot gap it can clip the head of the next
            # train. LIFETRAC_ALIGNED_PUMP=0 restores the pre-fix
            # (idle-drain-only) behavior for a clean A/B at the bench.
            with self._lock:
                have_pending = bool(self._pending_cmds)
            if (ALIGNED_PUMP_ENABLE and frame_done
                    and (have_pending or not self._ctrl_out.empty())):
                # Run H forensics (2026-07-26): without spacing, the two
                # enqueued copies go out ~37 ms apart — both into the SAME
                # (already stale) window, collapsing to one attempt. A
                # ≥120 ms gap forces copy 2 onto a later train boundary,
                # making the copies independent shots (expected per-command
                # delivery 1-(1-p)^2 instead of p).
                now_pump = time.monotonic()
                if now_pump - getattr(self, "_last_pump_t", 0.0) >= 0.12:
                    body = self._next_ctrl_body(now_pump)
                    if body is not None:
                        self._send_command_frame(link, body, copies=1)
                        self._last_pump_t = now_pump

            # 2026-07-27 RS-0.12 reactive-fire: a fragment completed the tile
            # of the last train, so the tractor's RXCONT just re-armed. Fire
            # a no-op PROBE now (optionally after a commanded phase delay) —
            # this is the honest measurement of in-stream base->tractor
            # delivery, using the exact TX path a drive command would.
            if self._reactive_fire and frame_done:
                self._maybe_fire_probe(link)

            self._maybe_switch_profile(link)

        LOG.info("RX worker exit")

    def _maybe_fire_probe(self, link: HostLink) -> None:
        now = time.monotonic()
        if now - self._last_probe_t < self._probe_min_gap_s:
            return
        self._last_probe_t = now
        # Round-robin the sweep offsets so each phase bin accumulates samples.
        phase_ms = self._probe_sweep[self._probe_seq % len(self._probe_sweep)]
        if phase_ms > 0:
            time.sleep(phase_ms / 1000.0)
        self._probe_seq = (self._probe_seq + 1) & 0xFFFFFFFF
        seq = self._probe_seq
        args = seq.to_bytes(4, "little") + (phase_ms & 0xFFFF).to_bytes(2, "little")
        self._probe_pending[seq] = time.monotonic()
        # Cap the pending map so a long run does not grow unbounded.
        if len(self._probe_pending) > 512:
            for k in sorted(self._probe_pending)[:256]:
                self._probe_pending.pop(k, None)
        self._probe_tx += 1
        LOG.info("PROBE TX seq=%d phase_ms=%d (tx#%d)", seq, phase_ms,
                 self._probe_tx)
        self._send_command_frame(
            link, pack_command_frame(CMD_OP_PROBE, args), copies=1)

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
                    "reassembler_decode_err=%d reassembler_timeouts=%d "
                    "parity_recon=%d",
                    s.rx_frames_seen, s.rx_decode_errors,
                    s.reassembled_frames_published, s.publish_errors,
                    s.reassembler_decode_errors, s.reassembler_timeouts,
                    self.reassembler.stats.parity_reconstructions)
            # RS-2.3 forensics + RS-1.1 command counters (outside the lock;
            # the sample list is only touched from the ingest thread and a
            # briefly stale read here is fine for a log line).
            samples = getattr(self, "_gap_samples", None)
            if samples:
                self._gap_samples = []
                ds = sorted(x[0] for x in samples)
                ls = sorted(x[1] for x in samples)
                n = len(ds)
                LOG.info(
                    "air_gap: n=%d dt_med=%dus dt_p95=%dus len_med=%dB "
                    "cmd_tx_ok=%d cmd_tx_fail=%d",
                    n, ds[n // 2], ds[min(n - 1, (n * 95) // 100)],
                    ls[n // 2],
                    getattr(self, "_cmd_tx_ok", 0),
                    getattr(self, "_cmd_tx_fail", 0))
                # 2026-07-27 RS-0.12: RAW gap histogram, not just med/p95 —
                # the distribution is bimodal (intra-fragment ~5 ms vs train
                # boundary), and med/p95 hide the boundary mode a control
                # frame must land in. Log fixed-edge buckets in ms.
                edges = (2, 5, 10, 20, 40, 80, 120, 200, 400, 1000)
                buckets = [0] * (len(edges) + 1)
                for d_us in ds:
                    d_ms = d_us / 1000.0
                    placed = False
                    for bi, e in enumerate(edges):
                        if d_ms < e:
                            buckets[bi] += 1
                            placed = True
                            break
                    if not placed:
                        buckets[-1] += 1
                labels = ["<2", "<5", "<10", "<20", "<40", "<80", "<120",
                          "<200", "<400", "<1000", ">=1000"]
                hist = " ".join(f"{lab}:{cnt}" for lab, cnt in
                                zip(labels, buckets) if cnt)
                LOG.info("air_gap_hist(ms): %s", hist)
            # RS-0.12 probe delivery summary (reactive-fire runs only).
            if self._reactive_fire and self._probe_tx:
                rtts = sorted(self._probe_rtts)
                self._probe_rtts = []
                deliv = (100.0 * self._probe_echo_rx / self._probe_tx)
                if rtts:
                    m = len(rtts)
                    LOG.info("probe: tx=%d echo=%d (%.1f%% delivered) "
                             "rtt_med=%.0fms rtt_p95=%.0fms",
                             self._probe_tx, self._probe_echo_rx, deliv,
                             rtts[m // 2], rtts[min(m - 1, (m * 95) // 100)])
                else:
                    LOG.info("probe: tx=%d echo=%d (%.1f%% delivered) "
                             "no echoes this window", self._probe_tx,
                             self._probe_echo_rx, deliv)

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
                    # 2026-07-25 implicit encoder ACK + radio-profile state
                    # for the web UI selectors / auto-profile policy.
                    "rx_codec": self._last_rx_codec,
                    "rx_codec_name": _CODEC_NAMES.get(
                        self._last_rx_codec, None)
                        if self._last_rx_codec is not None else None,
                    "radio_profile": self._active_profile,
                    "ts": round(time.time(), 1),
                }
            last_bytes = cur_bytes
            last_frames = cur_frames
            client = self._client
            if client is None:
                continue
            try:
                payload = _json.dumps(sample).encode("utf-8")
                client.publish(LINK_STATS_TOPIC, payload, qos=0)
                # Split-broker bench topology: mirror onto the control
                # broker so web_ui (host) can render link speed + drive
                # the auto-profile policy without an MQTT bridge.
                ctrl = self._ctrl_client
                if ctrl is not None:
                    ctrl.publish(LINK_STATS_TOPIC, payload, qos=0)
            except Exception as exc:                          # pragma: no cover
                LOG.debug("link_stats publish failed: %s", exc)

    def run(self, *, stats_interval_s: float) -> int:
        import paho.mqtt.client as mqtt

        self._client = mqtt.Client(
            client_id=f"lifetrac-image-rx-{os.getpid()}")

        def _on_connect(_c, _u, _f, rc):
            if rc == 0:
                LOG.info("MQTT connected; ready to publish %s", MQTT_TOPIC_OUT)
                # Retained: a restart re-applies the operator's last choice.
                self._client.subscribe(RADIO_PROFILE_TOPIC, qos=1)
                self._client.subscribe(ENCODE_MODE_OVERRIDE_TOPIC, qos=1)
                self._client.subscribe(KEYFRAME_REQ_TOPIC, qos=0)
            else:
                LOG.error("MQTT connect rc=%s", rc)
        self._client.on_connect = _on_connect
        self._subscribe_control(self._client)

        try:
            self._client.connect(self.mqtt_host, self.mqtt_port, keepalive=30)
        except Exception as exc:
            LOG.error("MQTT connect to %s:%d failed: %s",
                      self.mqtt_host, self.mqtt_port, exc)
            return 2

        # Optional split-broker control plane (bench: video on the base
        # board's local mosquitto, control on the host broker web_ui uses).
        if self.ctrl_mqtt_host and self.ctrl_mqtt_host != self.mqtt_host:
            try:
                self._ctrl_client = mqtt.Client(
                    client_id=f"lifetrac-image-rx-ctrl-{os.getpid()}")
                self._subscribe_control(self._ctrl_client)

                def _ctrl_on_connect(_c, _u, _f, rc):
                    if rc == 0:
                        LOG.info("ctrl MQTT connected (%s)", self.ctrl_mqtt_host)
                        self._ctrl_client.subscribe(RADIO_PROFILE_TOPIC, qos=1)
                        self._ctrl_client.subscribe(
                            ENCODE_MODE_OVERRIDE_TOPIC, qos=1)
                        self._ctrl_client.subscribe(KEYFRAME_REQ_TOPIC, qos=0)
                self._ctrl_client.on_connect = _ctrl_on_connect
                self._ctrl_client.connect(self.ctrl_mqtt_host,
                                          self.mqtt_port, keepalive=30)
                self._ctrl_client.loop_start()
            except Exception as exc:                          # pragma: no cover
                LOG.warning("ctrl MQTT connect to %s failed: %s — control "
                            "plane rides the primary broker only",
                            self.ctrl_mqtt_host, exc)
                self._ctrl_client = None

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
