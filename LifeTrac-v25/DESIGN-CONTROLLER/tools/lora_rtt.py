#!/usr/bin/env python3
"""tools/lora_rtt.py — RTT measurement harness for the LoRa link.

Subscribes to the broker, publishes a tagged heartbeat onto a request
topic at a fixed cadence, listens for the matching echo on a reply topic
(usually ``lifetrac/v25/diag/rtt/echo`` published by an on-tractor or
on-handheld responder), and records each round-trip into a JSONL file.

This harness only depends on the MQTT broker — the LoRa link rides
underneath via the existing bridge. No physical radios are required to
run the tool against a loopback echo; CI uses
``--echo-loopback`` to bounce the ping inside the same broker so the
nightly regression can detect MQTT/queue regressions even without a
tractor on the bench.

Usage::

    python -m tools.lora_rtt --target tractor --interval 1.0 --count 60
    python -m tools.lora_rtt --target handheld --duration 600 \\
        --out /var/log/lifetrac/rtt-$(date +%F).jsonl

Each output line:

    {"ts": 1714234567.123, "target": "tractor", "tag": 42,
     "rtt_ms": 142.7, "ok": true}

Failed pings (timeout, malformed echo) emit ``{"ok": false, "reason": "..."}``.
A summary block is appended at the end with min/p50/p95/p99/max + loss %.
"""
from __future__ import annotations

import argparse
import json
import os
import secrets
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Optional

DEFAULT_REQ_TOPIC_FMT = "lifetrac/v25/diag/rtt/req/{target}"
DEFAULT_REPLY_TOPIC_FMT = "lifetrac/v25/diag/rtt/echo/{target}"
DEFAULT_TIMEOUT_S = 5.0
DEFAULT_INTERVAL_S = 1.0


@dataclass
class _Pending:
    tag: int
    sent_at: float


@dataclass
class _Stats:
    target: str
    samples: list[float] = field(default_factory=list)
    sent: int = 0
    received: int = 0
    timed_out: int = 0
    malformed: int = 0

    def record_ok(self, rtt_ms: float) -> None:
        self.samples.append(rtt_ms)
        self.received += 1

    def record_timeout(self) -> None:
        self.timed_out += 1

    def record_malformed(self) -> None:
        self.malformed += 1

    @property
    def loss_pct(self) -> float:
        return 0.0 if self.sent == 0 else 100.0 * (self.sent - self.received) / self.sent

    def summary(self) -> dict:
        out: dict = {
            "ts": time.time(),
            "event": "summary",
            "target": self.target,
            "sent": self.sent,
            "received": self.received,
            "timed_out": self.timed_out,
            "malformed": self.malformed,
            "loss_pct": round(self.loss_pct, 3),
        }
        if self.samples:
            s = sorted(self.samples)
            out["min_ms"] = round(s[0], 2)
            out["p50_ms"] = round(_percentile(s, 50), 2)
            out["p95_ms"] = round(_percentile(s, 95), 2)
            out["p99_ms"] = round(_percentile(s, 99), 2)
            out["max_ms"] = round(s[-1], 2)
            out["mean_ms"] = round(statistics.fmean(s), 2)
        return out


def _percentile(sorted_samples: list[float], p: float) -> float:
    if not sorted_samples:
        return 0.0
    if len(sorted_samples) == 1:
        return sorted_samples[0]
    k = (len(sorted_samples) - 1) * (p / 100.0)
    lo = int(k)
    hi = min(lo + 1, len(sorted_samples) - 1)
    frac = k - lo
    return sorted_samples[lo] * (1.0 - frac) + sorted_samples[hi] * frac


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

def run(*, target: str, mqtt_host: str, mqtt_port: int,
        interval_s: float, count: int, duration_s: Optional[float],
        timeout_s: float, out_path: Optional[str],
        req_topic: str, reply_topic: str,
        echo_loopback: bool) -> int:
    try:
        import paho.mqtt.client as mqtt          # type: ignore
    except ImportError:
        print("ERROR: paho-mqtt not installed (pip install paho-mqtt)",
              file=sys.stderr)
        return 2

    pending: dict[int, _Pending] = {}
    pending_lock = threading.Lock()
    stats = _Stats(target=target)
    out_fp = open(out_path, "a", buffering=1, encoding="utf-8") if out_path else None
    stop_event = threading.Event()

    def _emit(line: dict) -> None:
        text = json.dumps(line, separators=(",", ":"))
        print(text, flush=True)
        if out_fp is not None:
            out_fp.write(text + "\n")

    def _on_connect(client, _u, _f, rc):
        if rc != 0:
            print(f"ERROR: MQTT connect rc={rc}", file=sys.stderr)
            stop_event.set()
            return
        client.subscribe(reply_topic, qos=0)
        if echo_loopback:
            client.subscribe(req_topic, qos=0)

    def _on_message(client, _u, msg):
        now = time.monotonic()
        if echo_loopback and msg.topic == req_topic:
            # Bounce request → reply with the same body.
            client.publish(reply_topic, msg.payload, qos=0, retain=False)
            return
        if msg.topic != reply_topic:
            return
        try:
            body = json.loads(msg.payload)
            tag = int(body["tag"])
        except (ValueError, KeyError, TypeError):
            stats.record_malformed()
            _emit({"ts": time.time(), "event": "rtt", "target": target,
                   "ok": False, "reason": "malformed"})
            return
        with pending_lock:
            entry = pending.pop(tag, None)
        if entry is None:
            return
        rtt_ms = (now - entry.sent_at) * 1000.0
        stats.record_ok(rtt_ms)
        _emit({"ts": time.time(), "event": "rtt", "target": target,
               "tag": tag, "rtt_ms": round(rtt_ms, 2), "ok": True})

    client = mqtt.Client(client_id=f"lora_rtt_{secrets.token_hex(3)}")
    client.on_connect = _on_connect
    client.on_message = _on_message
    client.connect(mqtt_host, mqtt_port, keepalive=30)
    client.loop_start()

    try:
        deadline = time.monotonic() + duration_s if duration_s else None
        sent = 0
        while not stop_event.is_set():
            if count and sent >= count:
                break
            if deadline is not None and time.monotonic() >= deadline:
                break
            tag = sent & 0xFFFFFFFF
            sent_at = time.monotonic()
            payload = json.dumps({"tag": tag, "ts": time.time()}).encode("utf-8")
            with pending_lock:
                pending[tag] = _Pending(tag=tag, sent_at=sent_at)
            client.publish(req_topic, payload, qos=0, retain=False)
            stats.sent += 1
            sent += 1
            # Sweep timeouts.
            cutoff = time.monotonic() - timeout_s
            with pending_lock:
                expired = [t for t, p in pending.items() if p.sent_at < cutoff]
                for t in expired:
                    pending.pop(t, None)
            for t in expired:
                stats.record_timeout()
                _emit({"ts": time.time(), "event": "rtt", "target": target,
                       "tag": t, "ok": False, "reason": "timeout"})
            time.sleep(max(0.0, interval_s - (time.monotonic() - sent_at)))
    except KeyboardInterrupt:
        pass
    finally:
        # Final timeout sweep before emitting summary.
        cutoff = time.monotonic() - timeout_s
        with pending_lock:
            expired = list(pending.keys())
            pending.clear()
        for t in expired:
            stats.record_timeout()
        _emit(stats.summary())
        client.loop_stop()
        client.disconnect()
        if out_fp is not None:
            out_fp.close()
    return 0


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--target", default="tractor",
                   help="Logical target name; used in the topic and the JSONL")
    p.add_argument("--mqtt-host",
                   default=os.environ.get("LIFETRAC_MQTT_HOST", "localhost"))
    p.add_argument("--mqtt-port", type=int, default=1883)
    p.add_argument("--interval", type=float, default=DEFAULT_INTERVAL_S,
                   dest="interval_s",
                   help="seconds between consecutive pings (default 1.0)")
    p.add_argument("--count", type=int, default=0,
                   help="ping count; 0 = unlimited (use --duration)")
    p.add_argument("--duration", type=float, default=None, dest="duration_s",
                   help="run for at most this many seconds")
    p.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S,
                   dest="timeout_s")
    p.add_argument("--out", default=None, dest="out_path",
                   help="append every event as JSONL to this file")
    p.add_argument("--req-topic", default=None,
                   help="override request topic (default lifetrac/v25/diag/rtt/req/<target>)")
    p.add_argument("--reply-topic", default=None,
                   help="override reply topic (default lifetrac/v25/diag/rtt/echo/<target>)")
    p.add_argument("--echo-loopback", action="store_true",
                   help="when set, this process also acts as the echo "
                        "responder — useful for CI without a real tractor")
    args = p.parse_args(argv)
    if not args.req_topic:
        args.req_topic = DEFAULT_REQ_TOPIC_FMT.format(target=args.target)
    if not args.reply_topic:
        args.reply_topic = DEFAULT_REPLY_TOPIC_FMT.format(target=args.target)
    if args.count == 0 and args.duration_s is None:
        # Don't accidentally run forever.
        args.duration_s = 60.0
    return args


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(argv)
    return run(target=args.target,
               mqtt_host=args.mqtt_host, mqtt_port=args.mqtt_port,
               interval_s=args.interval_s,
               count=args.count, duration_s=args.duration_s,
               timeout_s=args.timeout_s, out_path=args.out_path,
               req_topic=args.req_topic, reply_topic=args.reply_topic,
               echo_loopback=args.echo_loopback)


if __name__ == "__main__":
    raise SystemExit(main())
