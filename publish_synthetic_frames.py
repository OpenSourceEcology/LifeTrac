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
    broker = os.environ.get("LIFETRAC_MQTT_HOST", "192.168.1.79")
    # Saturation-mode knobs (2026-07-24): a throughput MEASUREMENT is only
    # valid when offered load exceeds capacity (post-impl review §7.3) —
    # otherwise the number reported is just the generator rate.
    fps = float(os.environ.get("LIFETRAC_SYNTH_FPS", "2"))
    duration_s = float(os.environ.get("LIFETRAC_SYNTH_DURATION_S", "150"))
    byte_budget = int(os.environ.get("LIFETRAC_SYNTH_BYTE_BUDGET", "250"))
    keyframe_every = int(os.environ.get("LIFETRAC_SYNTH_KEYFRAME_EVERY", "10"))
    print(f"Starting synthetic camera publisher (broker={broker}, "
          f"fps={fps}, duration={duration_s}s, budget={byte_budget}B)...")
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id="synthetic_camera_host")
    client.connect(broker, 1883)
    client.loop_start()

    cam = SyntheticCamera()
    accum = FrameAccum()

    period = 1.0 / max(fps, 0.01)
    total = int(duration_s * fps)
    sent_bytes = 0
    t0 = time.monotonic()
    try:
        for i in range(total):
            force_key = (keyframe_every > 0 and i % keyframe_every == 0)
            payload = _build_frame(cam, accum, force_keyframe=force_key,
                                   byte_budget=byte_budget)
            sent_bytes += len(payload)
            print(f"[{i:03d}] Publishing synthetic frame of size {len(payload)} bytes...")
            client.publish("lifetrac/v25/cmd/image_frame", payload, qos=0)
            # Fixed-schedule pacing (not sleep-after-work) so the offered
            # rate stays honest even when _build_frame is slow.
            next_t = t0 + (i + 1) * period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
        elapsed = time.monotonic() - t0
        print(f"OFFERED: {total} frames, {sent_bytes} B in {elapsed:.1f}s "
              f"= {sent_bytes/elapsed:.1f} B/s offered load")
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        print("Done.")

if __name__ == "__main__":
    main()
