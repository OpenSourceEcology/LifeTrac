import sys
import os
import time

sys.path.append(os.path.abspath("LifeTrac-v25/DESIGN-CONTROLLER/base_station"))

import paho.mqtt.client as mqtt
from image_pipeline.reassemble import FragmentReassembler

r = FragmentReassembler()

def on_message(c, u, msg):
    print("Received MQTT message:", msg.topic, len(msg.payload))
    try:
        frame = r.feed(msg.payload)
        print("Reassembler returned:", frame)
        print("Reassembler stats:", r.stats)
    except Exception as e:
        print("Error feeding payload:", e)

c = mqtt.Client()
c.on_message = on_message
c.connect("127.0.0.1", 1883)
c.subscribe("lifetrac/v25/cmd/image_frame")
c.loop_start()

time.sleep(4)
c.loop_stop()
print("Done")
