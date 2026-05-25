#!/usr/bin/env python3
"""Real-camera publisher for the LifeTrac v25 LoRa video pipeline.

This is the live-camera analogue of `publish_synthetic_frames.py`. It is
intended to run **inside the foundries python-devel docker container on
the TX board** (the same image used by `image_tx_daemon`), with
`/dev/video1` (Kurokesu C2) mapped via `docker run --device=/dev/video1`
and a static aarch64 `ffmpeg` binary mounted under `/work/ffmpeg`.

It mirrors the encode contract used by `camera_service._build_frame()`
so the wire payload published to `lifetrac/v25/cmd/image_frame` is
byte-equivalent to what the camera service produces. From there, the
existing `image_tx_daemon` → LoRa → `image_rx_daemon` → MQTT
`tile_delta` → `web_ui` → browser chain renders the frames unchanged.

Env vars:
  LIFETRAC_MQTT_HOST           broker IP (default 192.168.1.79)
  LIFETRAC_CAMERA_DEVICE       v4l2 device (default /dev/video1)
  LIFETRAC_V4L2_INPUT_FORMAT   mjpeg | yuyv422 (default mjpeg)
  LIFETRAC_V4L2_INPUT_SIZE     e.g. 1920x1080 (default 1920x1080)
  LIFETRAC_V4L2_INPUT_FPS      capture fps   (default 30)
  LIFETRAC_FFMPEG_PATH         ffmpeg binary (default ffmpeg in $PATH)
  LIFETRAC_CAMERA_FPS          publish fps   (default 2)
  LIFETRAC_DURATION_S          run for N seconds, then exit (default 0 = forever)
  LIFETRAC_KEYFRAME_PERIOD_S   keyframe cadence (default 10)
  LIFETRAC_FRAGMENT_BUDGET     soft per-frame byte cap (default 250 like synth)
  LIFETRAC_CAMERA_PUB_TOPIC    MQTT topic (default lifetrac/v25/cmd/image_frame)
"""

import os
import sys
import time
import logging

# Make the existing camera_service + image_pipeline importable both on the
# host (developer running it from the workspace) and inside the docker
# container on the TX board (everything mounted under /work).
def _ensure_imports():
    candidates = []
    here = os.path.dirname(os.path.abspath(__file__))
    # Host layout (repo root).
    candidates.append(os.path.join(here, "LifeTrac-v25", "DESIGN-CONTROLLER",
                                   "firmware", "tractor_x8"))
    candidates.append(os.path.join(here, "LifeTrac-v25", "DESIGN-CONTROLLER",
                                   "base_station"))
    # Container layout (smoke pushes camera_service.py + image_pipeline +
    # lora_proto.py into /work directly).
    candidates.append("/work")
    candidates.append("/work/paho")
    for p in candidates:
        if p and os.path.isdir(p) and p not in sys.path:
            sys.path.insert(0, p)


_ensure_imports()

import paho.mqtt.client as mqtt  # noqa: E402
from camera_service import (      # noqa: E402
    V4l2FfmpegCamera, SyntheticCamera, FrameAccum, _build_frame,
)


LOG = logging.getLogger("publish_camera_frames")


def _make_camera():
    """Try the real USB camera first; fall back to synthetic so the
    runner never silently produces zero frames on a sensor hiccup."""
    try:
        cam = V4l2FfmpegCamera()
        LOG.info("camera: V4l2FfmpegCamera ready")
        return cam
    except Exception as exc:
        LOG.warning("camera: V4l2FfmpegCamera failed (%s); using SyntheticCamera",
                    exc)
        return SyntheticCamera()


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    broker = os.environ.get("LIFETRAC_MQTT_HOST", "192.168.1.79")
    topic  = os.environ.get("LIFETRAC_CAMERA_PUB_TOPIC",
                            "lifetrac/v25/cmd/image_frame")
    pub_fps = float(os.environ.get("LIFETRAC_CAMERA_FPS", "2"))
    duration_s = float(os.environ.get("LIFETRAC_DURATION_S", "0"))
    budget = int(os.environ.get("LIFETRAC_FRAGMENT_BUDGET", "250"))
    keyframe_every = max(1, int(float(os.environ.get(
        "LIFETRAC_KEYFRAME_PERIOD_S", "10")) * pub_fps))

    LOG.info("connecting to broker %s:1883 topic=%s pub_fps=%.2f duration_s=%.0f budget=%dB",
             broker, topic, pub_fps, duration_s, budget)

    try:
        client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            client_id="camera_publisher_tx",
        )
    except AttributeError:
        # paho v1.x fallback (Portenta docker may still ship 1.6.x).
        client = mqtt.Client(client_id="camera_publisher_tx")
    client.connect(broker, 1883)
    client.loop_start()

    cam = _make_camera()
    accum = FrameAccum()
    period = 1.0 / max(pub_fps, 0.1)

    t0 = time.monotonic()
    next_t = t0
    i = 0
    try:
        while True:
            if duration_s > 0 and (time.monotonic() - t0) >= duration_s:
                LOG.info("duration_s=%.1f elapsed; exiting cleanly", duration_s)
                break
            force_key = (i % keyframe_every == 0)
            try:
                payload = _build_frame(cam, accum,
                                       force_keyframe=force_key,
                                       byte_budget=budget)
            except Exception as exc:
                LOG.exception("_build_frame failed: %s", exc)
                time.sleep(period)
                i += 1
                continue
            client.publish(topic, payload, qos=0)
            LOG.info("[%04d] published %d B (key=%s)", i, len(payload), force_key)
            i += 1
            next_t += period
            sleep_s = next_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                next_t = time.monotonic()
    except KeyboardInterrupt:
        LOG.info("KeyboardInterrupt; exiting")
    finally:
        try:
            client.loop_stop()
            client.disconnect()
        except Exception:
            pass
        try:
            close = getattr(cam, "close", None)
            if callable(close):
                close()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
