import sys, time
import paho.mqtt.client as mqtt
HOST = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
TOPICS = [
    "lifetrac/v25/control/encode_mode_override",
    "lifetrac/v25/cmd/req_keyframe",
    "lifetrac/v25/control/radio_profile",
]
try:
    c = mqtt.Client(client_id="bench-retain-clear-%s" % HOST)
    c.connect(HOST, 1883, 5)
    c.loop_start()
    for t in TOPICS:
        c.publish(t, None, qos=1, retain=True)
    time.sleep(1.0)
    c.loop_stop()
    print("RETAINED-CLEARED host=%s : %s" % (HOST, " ".join(TOPICS)))
except Exception as exc:
    print("NO-BROKER host=%s (%s) -- harness will start a fresh one" % (HOST, exc))
