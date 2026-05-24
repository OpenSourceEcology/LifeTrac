import sys
import os
import time

sys.path.append(os.path.abspath("LifeTrac-v25/DESIGN-CONTROLLER/base_station"))

import paho.mqtt.client as mqtt
from image_pipeline.frame_format import parse_tile_delta_frame

def on_message(c, u, msg):
    print("Received raw payload on:", msg.topic, "length:", len(msg.payload))
    try:
        frame = parse_tile_delta_frame(msg.payload)
        print("Success! Decoded TileDeltaFrame:")
        print("  frame_kind:", frame.frame_kind)
        print("  base_seq:", frame.base_seq)
        print("  grid_w:", frame.grid_w)
        print("  grid_h:", frame.grid_h)
        print("  tile_px:", frame.tile_px)
        print("  changed_indices count:", len(frame.changed_indices))
        print("  tiles count:", len(frame.tiles))
    except Exception as e:
        print("Failed to decode frame:", type(e), e)
    os._exit(0)

c = mqtt.Client()
c.on_message = on_message
c.connect("127.0.0.1", 1883)
c.subscribe("lifetrac/v25/cmd/image_frame")
print("Listening...")
c.loop_forever()
