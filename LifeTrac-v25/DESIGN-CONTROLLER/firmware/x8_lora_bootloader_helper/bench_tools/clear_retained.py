"""Clear stale retained control topics on the local broker (bench rule:
a stale retained encode_mode_override twice flipped the camera
mid-session — clear on every broker the daemon can see before a leg)."""
import time

import paho.mqtt.client as mqtt

TOPICS = [
    "lifetrac/v25/control/encode_mode_override",
    "lifetrac/v25/cmd/req_keyframe",
    # 2026-09-12: a retained radio_profile pin (left by a profile-switch
    # leg) makes the rx daemon command the tractor to that profile at
    # startup, fighting the harness and storming keyframe requests.
    "lifetrac/v25/control/radio_profile",
]

c = mqtt.Client(client_id="bench-retain-clear")
c.connect("127.0.0.1", 1883, 10)
c.loop_start()
for t in TOPICS:
    c.publish(t, None, qos=1, retain=True)
time.sleep(1.0)
c.loop_stop()
print("RETAINED-CLEARED " + " ".join(TOPICS))
