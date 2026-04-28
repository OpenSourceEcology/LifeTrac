#!/usr/bin/env python3
"""
LifeTrac v25 — tractor X8 logger service.

Local SQLite + JSONL black-box recorder running on the tractor X8. Mirrors
the base-station audit log (base_station/audit_log.py) but on the tractor
side so a base-station outage doesn't lose tractor-local events.

Per MASTER_PLAN.md §8.10:
  * Subscribes to MQTT lifetrac/v25/{telemetry,status,event,control}/# on
    the tractor's local Mosquitto loopback.
  * Appends every event to /var/log/lifetrac/tractor_audit.jsonl with
    size-based rotation (10 MB, 5 keep) — same shape as base AuditLog.
  * Mirrors high-rate telemetry into /var/log/lifetrac/telemetry.sqlite
    for later download by the operator console.

Design notes:
  * The SQLite ingest uses a single-writer thread fed by a queue so the
    paho callback never blocks on disk. WAL mode + 1-row-per-message keeps
    the schema simple; queries are by (topic, ts) range.
  * If paho is missing (dev box), we fall back to no-op heartbeat so the
    service still boots — useful for systemd units installed before the
    MQTT broker is provisioned.
"""

from __future__ import annotations

import json
import logging
import os
import queue
import sqlite3
import sys
import threading
import time
from pathlib import Path

# Reuse the base-station rotating JSONL writer so we have one implementation.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "base_station"))
try:
    from audit_log import AuditLog
except ImportError as exc:
    raise SystemExit(f"logger_service: cannot import AuditLog: {exc}") from exc

LOG = logging.getLogger("logger_service")

TRACTOR_AUDIT_PATH = os.environ.get("LIFETRAC_TRACTOR_AUDIT_PATH",
                                    "/var/log/lifetrac/tractor_audit.jsonl")
TELEMETRY_DB_PATH  = os.environ.get("LIFETRAC_TRACTOR_TELEM_DB",
                                    "/var/log/lifetrac/telemetry.sqlite")
SUBSCRIBE_FILTER   = "lifetrac/v25/#"


# ---- SQLite ingest ----------------------------------------------------

_SCHEMA = """
CREATE TABLE IF NOT EXISTS messages (
    ts_ms   INTEGER NOT NULL,
    topic   TEXT    NOT NULL,
    payload BLOB    NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_messages_topic_ts
    ON messages (topic, ts_ms);
"""


class TelemetryWriter:
    """Single-writer thread fed by a queue. paho callbacks enqueue and
    return immediately; disk I/O happens off the network thread."""

    def __init__(self, db_path: str, audit: AuditLog):
        Path(db_path).parent.mkdir(parents=True, exist_ok=True)
        self._db_path = db_path
        self._audit = audit
        self._q: queue.Queue[tuple[int, str, bytes] | None] = queue.Queue(maxsize=10000)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="telem-writer",
                                         daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._q.put(None)
        self._thread.join(timeout=5)

    def enqueue(self, topic: str, payload: bytes) -> None:
        ts_ms = int(time.time() * 1000)
        try:
            self._q.put_nowait((ts_ms, topic, payload))
        except queue.Full:
            # Backpressure: drop the message and audit the drop. Better than
            # blocking the MQTT thread and falling further behind.
            self._audit.record("telemetry_dropped",
                               topic=topic, q_full=True)

    def _run(self) -> None:
        conn = sqlite3.connect(self._db_path)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.executescript(_SCHEMA)
        conn.commit()
        try:
            batch: list[tuple[int, str, bytes]] = []
            while not self._stop.is_set():
                try:
                    item = self._q.get(timeout=0.5)
                except queue.Empty:
                    if batch:
                        self._flush(conn, batch)
                        batch = []
                    continue
                if item is None:
                    break
                batch.append(item)
                if len(batch) >= 200:
                    self._flush(conn, batch)
                    batch = []
            if batch:
                self._flush(conn, batch)
        finally:
            conn.close()

    @staticmethod
    def _flush(conn: sqlite3.Connection, batch: list) -> None:
        try:
            conn.executemany(
                "INSERT INTO messages (ts_ms, topic, payload) VALUES (?, ?, ?)",
                batch,
            )
            conn.commit()
        except sqlite3.Error as exc:
            LOG.warning("logger: sqlite flush failed: %s", exc)


# ---- entry point ------------------------------------------------------

def main() -> int:
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s")
    audit = AuditLog(TRACTOR_AUDIT_PATH)
    audit.record("logger_start")

    writer = TelemetryWriter(TELEMETRY_DB_PATH, audit)
    writer.start()

    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        LOG.warning("logger_service: paho-mqtt missing — heartbeat-only mode")
        try:
            while True:
                time.sleep(60.0)
                audit.record("heartbeat")
        except KeyboardInterrupt:
            pass
        finally:
            writer.stop()
            audit.record("logger_stop")
            audit.close()
        return 0

    def on_connect(client, _userdata, _flags, rc):
        if rc == 0:
            client.subscribe(SUBSCRIBE_FILTER, qos=0)
            audit.record("mqtt_connected")
        else:
            audit.record("mqtt_connect_failed", rc=rc)

    def on_message(_client, _userdata, msg):
        # Audit every event-class message (low rate); telemetry goes only
        # to sqlite so we don't bloat the JSONL log.
        if "/event/" in msg.topic or "/control/" in msg.topic:
            try:
                audit.record("mqtt_event",
                             topic=msg.topic,
                             payload=msg.payload.decode("utf-8", "replace"))
            except Exception as exc:                    # never let a logger crash on bad UTF-8
                audit.record("mqtt_event_decode_fail",
                             topic=msg.topic, err=str(exc))
        writer.enqueue(msg.topic, bytes(msg.payload))

    client = mqtt.Client(client_id="logger_service")
    client.on_connect = on_connect
    client.on_message = on_message
    try:
        client.connect("localhost", 1883)
    except OSError as exc:
        LOG.warning("logger_service: MQTT connect failed: %s", exc)
        audit.record("mqtt_connect_oserror", err=str(exc))

    LOG.info("logger_service: running")
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        pass
    finally:
        writer.stop()
        audit.record("logger_stop")
        audit.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
