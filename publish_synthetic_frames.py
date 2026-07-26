import sys
import os
import time
import struct
import paho.mqtt.client as mqtt

# Add the paths to import from camera_service and image_pipeline. On the
# bench PC these are the repo checkouts; when run on-board (RS-5.9 local
# feed, /work inside the synth_pub container) camera_service.py sits next
# to this script and the inserts are harmless no-ops.
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
    # RS-5.9 follow-up (2026-07-25): on the tractor's A53 cores _build_frame
    # manages only ~3.3 fps, silently capping offered load below the link
    # ceiling (first local-feed run: util 30%, qdepth 0 — a traffic report,
    # not a measurement). PREBUILD>0 builds that many frames up front and
    # replays them round-robin, so the paced publish loop is CPU-free and
    # the offered rate is exactly fps. Payload shape matches live mode.
    prebuild = int(os.environ.get("LIFETRAC_SYNTH_PREBUILD", "0"))
    print(f"Starting synthetic camera publisher (broker={broker}, "
          f"fps={fps}, duration={duration_s}s, budget={byte_budget}B, "
          f"prebuild={prebuild})...")
    # RS-5.9 (2026-07-25): paho 1.x/2.x compatibility. The bench PC ships
    # paho 2.x (VERSION2 API); the on-board images ship 1.x, which has no
    # CallbackAPIVersion and crashed the first local-feed run.
    try:
        client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                             client_id="synthetic_camera_host")
    except AttributeError:
        client = mqtt.Client(client_id="synthetic_camera_host")
    client.connect(broker, 1883)
    client.loop_start()

    cam = SyntheticCamera()
    accum = FrameAccum()

    bank = []
    if prebuild > 0:
        for i in range(prebuild):
            force_key = (keyframe_every > 0 and i % keyframe_every == 0)
            bank.append(_build_frame(cam, accum, force_keyframe=force_key,
                                     byte_budget=byte_budget))
        sizes = sorted(len(p) for p in bank)
        print(f"PREBUILT: {len(bank)} frames, sizes {sizes[0]}..{sizes[-1]} B")

    period = 1.0 / max(fps, 0.01)
    total = int(duration_s * fps)
    sent_bytes = 0
    t0 = time.monotonic()
    try:
        for i in range(total):
            if bank:
                payload = bank[i % len(bank)]
            else:
                force_key = (keyframe_every > 0 and i % keyframe_every == 0)
                payload = _build_frame(cam, accum, force_keyframe=force_key,
                                       byte_budget=byte_budget)
            sent_bytes += len(payload)
            if not bank or i % 50 == 0:
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
