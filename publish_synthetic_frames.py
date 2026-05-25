import sys
import os
import time
import struct
import paho.mqtt.client as mqtt

# Add the paths to import from camera_service and image_pipeline
sys.path.insert(0, r"c:\Users\dorkm\Documents\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\firmware\tractor_x8")
sys.path.insert(0, r"c:\Users\dorkm\Documents\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\base_station")

from camera_service import SyntheticCamera, FrameAccum, _build_frame

def main():
    broker = os.environ.get("LIFETRAC_MQTT_HOST", "192.168.1.117")
    print(f"Starting synthetic camera publisher on Host (broker={broker})...")
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id="synthetic_camera_host")
    client.connect(broker, 1883)
    client.loop_start()

    cam = SyntheticCamera()
    accum = FrameAccum()

    try:
        # Run for 150 seconds to cover the smoke test completely
        # We send a frame every 500 ms (2 fps)
        for i in range(300):
            # Build and pack a synthetic frame
            # Force keyframe every 10 frames
            force_key = (i % 10 == 0)
            payload = _build_frame(cam, accum, force_keyframe=force_key, byte_budget=250)
            print(f"[{i:02d}] Publishing synthetic frame of size {len(payload)} bytes...")
            client.publish("lifetrac/v25/cmd/image_frame", payload, qos=0)
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        print("Done.")

if __name__ == "__main__":
    main()
