"""mqtt_tap.py <outfile> [host] -- timestamped record of the auto-policy leg.

Subscribes on the broker to the profile control topic (the policy's retained
pin is delivered on subscribe, so the seed state is captured), the two
status acks (tractor/base confirm each switch), and link_stats (every ~2 s:
rx_frames_seen, radio_profile, frags_expected/missing -- the dead-air and
loss signatures the policy consumes). Appends one JSON line per message,
fsync'd, so the record survives whatever happens to this container.
"""
import json
import os
import sys
import time

import paho.mqtt.client as mqtt

OUT = sys.argv[1]
HOST = sys.argv[2] if len(sys.argv) > 2 else "127.0.0.1"
TOPICS = [
    ("lifetrac/v25/control/radio_profile", 1),
    ("lifetrac/v25/status/radio_profile/tx", 1),
    ("lifetrac/v25/status/radio_profile/rx", 1),
    ("lifetrac/v25/video/link_stats", 0),
]
fh = open(OUT, "a")


def _on_message(_c, _u, msg):
    try:
        body = json.loads(msg.payload.decode("utf-8"))
    except Exception:
        body = msg.payload[:80].hex()
    rec = {"t": round(time.time(), 3), "topic": msg.topic.split("/", 2)[-1],
           "retain": bool(msg.retain), "body": body}
    fh.write(json.dumps(rec, separators=(",", ":")) + "\n")
    fh.flush()
    os.fsync(fh.fileno())


c = mqtt.Client(client_id="rs14-tap")
c.on_message = _on_message
c.connect(HOST, 1883, 10)
for t, q in TOPICS:
    c.subscribe(t, qos=q)
fh.write(json.dumps({"t": round(time.time(), 3), "topic": "TAP-START", "host": HOST}) + "\n")
fh.flush()
c.loop_forever()
