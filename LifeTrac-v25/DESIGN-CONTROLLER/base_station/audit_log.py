"""audit_log.py — append-only JSONL audit log for the LoRa bridge.

Per LORA_IMPLEMENTATION.md §5 + §7 every operationally-interesting event must
be persisted so we can post-mortem bench failures and field incidents:

  - frame TX / RX (with source, type, seq, RSSI, CRC/GCM result)
  - source transitions (HANDHELD ↔ BASE ↔ AUTONOMY ↔ NONE)
  - SF ladder transitions (rung 0/1/2 + reason)
  - encode-mode transitions (full / y_only / motion_only / wireframe)
  - replay rejections, GCM tag rejections, CSMA skip events

The writer is intentionally minimal: line-buffered append to a rotating file,
JSON per line, no external dependencies. A single ``threading.Lock`` keeps the
worker threads from interleaving partial lines. If the disk is full or the
file cannot be opened we degrade to logging.warning() and keep the bridge
running — losing audit lines is preferable to dropping a control frame.
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from typing import Any

DEFAULT_PATH = "/var/log/lifetrac/audit.jsonl"
ROTATE_BYTES = 10 * 1024 * 1024   # 10 MB per file
KEEP_ROTATIONS = 5


class AuditLog:
    """Thread-safe JSONL writer with crude size-based rotation."""

    def __init__(self, path: str = DEFAULT_PATH,
                 rotate_bytes: int = ROTATE_BYTES,
                 keep_rotations: int = KEEP_ROTATIONS) -> None:
        self.path = path
        self.rotate_bytes = rotate_bytes
        self.keep_rotations = keep_rotations
        self._lock = threading.Lock()
        self._fp = None
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            # Line-buffered so a crash still flushes the previous record.
            self._fp = open(path, "a", buffering=1, encoding="utf-8")
        except OSError as exc:
            logging.warning("audit_log: open(%s) failed: %s", path, exc)
            self._fp = None

    def close(self) -> None:
        with self._lock:
            if self._fp is not None:
                try:
                    self._fp.close()
                finally:
                    self._fp = None

    def record(self, event: str, **fields: Any) -> None:
        """Write one event line. Never raises."""
        line = {"ts": time.time(), "event": event}
        line.update(fields)
        try:
            payload = json.dumps(line, separators=(",", ":"), default=_json_default)
        except (TypeError, ValueError) as exc:
            logging.debug("audit_log: json encode of %s failed: %s", event, exc)
            return
        with self._lock:
            if self._fp is None:
                return
            try:
                self._fp.write(payload + "\n")
                if self._fp.tell() >= self.rotate_bytes:
                    self._rotate_locked()
            except OSError as exc:
                logging.warning("audit_log: write failed: %s", exc)

    # ----- named event helpers ----------------------------------------
    #
    # Thin wrappers around :meth:`record` that bake in the canonical event
    # name and document the expected fields. Call-sites should prefer these
    # over :meth:`record` directly so the JSONL stays queryable (one event
    # name per concern, fields stable over time).

    def log_source_transition(self, *, prev: str, new: str,
                              reason: str | None = None,
                              **extra: Any) -> None:
        """Active-source arbitration changed (HANDHELD/BASE/AUTONOMY/NONE)."""
        self.record("source_transition", prev=prev, new=new,
                    reason=reason, **extra)

    def log_sf_step(self, *, prev_rung: int, new_rung: int,
                    prev_sf: int, new_sf: int,
                    reason: str,
                    rssi_dbm: float | None = None,
                    snr_db: float | None = None,
                    **extra: Any) -> None:
        """SF ladder transition (3-window hysteresis in :class:`LinkMonitor`)."""
        self.record("sf_step",
                    prev_rung=prev_rung, new_rung=new_rung,
                    prev_sf=prev_sf, new_sf=new_sf,
                    reason=reason, rssi_dbm=rssi_dbm, snr_db=snr_db, **extra)

    def log_encode_mode_change(self, *, prev: str, new: str,
                               reason: str,
                               airtime_pct: float | None = None,
                               **extra: Any) -> None:
        """Image-tier encode-mode transition (full/y_only/motion_only/wireframe)."""
        self.record("encode_mode_change", prev=prev, new=new, reason=reason,
                    airtime_pct=airtime_pct, **extra)

    def log_fhss_skip(self, *, channel: int, reason: str,
                      airtime_pct: float | None = None,
                      **extra: Any) -> None:
        """CSMA / FHSS hop skipped (channel busy or airtime cap)."""
        self.record("fhss_skip", channel=channel, reason=reason,
                    airtime_pct=airtime_pct, **extra)

    def log_replay_reject(self, *, source_id: int, seq: int,
                          window_low: int | None = None,
                          window_high: int | None = None,
                          **extra: Any) -> None:
        """Per-source ReplayWindow rejected an inbound frame."""
        self.record("replay_reject", source_id=source_id, seq=seq,
                    window_low=window_low, window_high=window_high, **extra)

    def log_gcm_reject(self, *, onair_len: int,
                       reason: str = "tag_mismatch",
                       **extra: Any) -> None:
        """AES-GCM tag verification failed on an inbound frame."""
        self.record("gcm_tag_reject", onair_len=onair_len, reason=reason,
                    **extra)

    def log_person_appeared(self, *, source: str, cls: str,
                            confidence: float,
                            cx: float, cy: float,
                            **extra: Any) -> None:
        """Tractor or base detector emitted CMD_PERSON_APPEARED (opcode 0x60).

        Fields:
          - ``source``     : 'tractor' (NanoDet) or 'base' (YOLO/NanoDet)
          - ``cls``        : 'person' / 'animal' / 'vehicle'
          - ``confidence`` : 0.0..1.0
          - ``cx, cy``     : normalized [0..1] centroid
        """
        self.record("person_appeared", source=source, cls=cls,
                    confidence=confidence, cx=cx, cy=cy, **extra)

    # ---- helpers ----
    def _rotate_locked(self) -> None:
        try:
            self._fp.close()
        except OSError:
            pass
        self._fp = None
        try:
            for i in range(self.keep_rotations - 1, 0, -1):
                src = f"{self.path}.{i}"
                dst = f"{self.path}.{i + 1}"
                if os.path.exists(src):
                    os.replace(src, dst)
            if os.path.exists(self.path):
                os.replace(self.path, f"{self.path}.1")
        except OSError as exc:
            logging.warning("audit_log: rotate failed: %s", exc)
        try:
            self._fp = open(self.path, "a", buffering=1, encoding="utf-8")
        except OSError as exc:
            logging.warning("audit_log: reopen failed: %s", exc)
            self._fp = None


def _json_default(obj: Any) -> Any:
    # Bytes are common in our event fields (raw payload snippets); render as hex
    # so the log stays grep-friendly without ballooning size.
    if isinstance(obj, (bytes, bytearray)):
        return obj.hex()
    raise TypeError(f"unsupported type for audit log: {type(obj).__name__}")


__all__ = ["AuditLog", "DEFAULT_PATH"]


# ---- CLI: --tail-mqtt mode (used by docker-compose audit_tail service) ----
#
# Subscribes to the broker's MQTT firehose and appends every message to the
# audit JSONL file. Lets us bring up an audit-only sidecar in docker-compose
# without coupling audit capture to the lora_bridge process lifecycle.

def _main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description="LifeTrac v25 audit log CLI")
    parser.add_argument("--tail-mqtt", action="store_true",
                        help="subscribe to lifetrac/v25/# and append every "
                             "message as an audit event")
    parser.add_argument("--mqtt-host",
                        default=os.environ.get("LIFETRAC_MQTT_HOST", "localhost"))
    parser.add_argument("--path", default=os.environ.get("LIFETRAC_AUDIT_PATH",
                                                          DEFAULT_PATH))
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")

    log = AuditLog(path=args.path)

    if not args.tail_mqtt:
        parser.print_help()
        return 0

    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        logging.error("audit_log --tail-mqtt: paho-mqtt not installed")
        return 1

    def _on_msg(_c, _u, msg):
        # Try to keep the JSON small: hex for binary, str for text.
        try:
            data: Any = msg.payload.decode("utf-8")
        except UnicodeDecodeError:
            data = msg.payload  # AuditLog renders bytes as hex via _json_default
        log.record("mqtt", topic=msg.topic, qos=msg.qos, payload=data)

    client = mqtt.Client(client_id="audit_tail")
    client.on_message = _on_msg
    client.connect(args.mqtt_host, 1883)
    client.subscribe("lifetrac/v25/#", qos=0)
    client.loop_forever()
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())
