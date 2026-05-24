import sys
import os
import time

sys.path.append(os.path.abspath("LifeTrac-v25/DESIGN-CONTROLLER/base_station"))

import paho.mqtt.client as mqtt
from image_pipeline.frame_format import parse_tile_delta_frame

def on_message(c, u, msg):
    if "req_keyframe" in msg.topic:
        print(">>> Received req_keyframe: msg =", msg.payload)
    elif "image_frame" in msg.topic:
        try:
            frame = parse_tile_delta_frame(msg.payload)
            print("Received base_seq:", frame.base_seq, "is_keyframe:", frame.is_keyframe, "tiles:", len(frame.tiles))
        except Exception as e:
            print("Failed to decode image_frame:", e)

c = mqtt.Client()
c.on_message = on_message
c.connect("127.0.0.1", 1883)
c.subscribe("lifetrac/v25/cmd/+")
print("Listening...")
c.loop_start()

time.sleep(6)
c.loop_stop()
print("Done")
