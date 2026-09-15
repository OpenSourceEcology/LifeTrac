"""Bench keyframe-request injector (RS-3.3 leg 3).

Publishes to the base broker's cmd/req_keyframe topic on a fixed cadence,
driving the rx daemon's real LoRa command path (CMD_OP_REQ_KEYFRAME ->
tractor encoder keyframe -> multi-fragment train). Run on the BASE inside
lifetrac-v25:latest with --network=host while a harness leg is live.
"""
import sys
import time

import paho.mqtt.client as mqtt

TOPIC = "lifetrac/v25/cmd/req_keyframe"
PERIOD_S = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
COUNT = int(sys.argv[2]) if len(sys.argv) > 2 else 24

c = mqtt.Client(client_id="bench-kf-injector")
c.connect("127.0.0.1", 1883, 30)
c.loop_start()
for i in range(COUNT):
    c.publish(TOPIC, b"bench-rs33-leg3-inject", qos=0)
    print("KF_INJECT %d/%d" % (i + 1, COUNT), flush=True)
    time.sleep(PERIOD_S)
c.loop_stop()
print("KF_INJECT_DONE")
