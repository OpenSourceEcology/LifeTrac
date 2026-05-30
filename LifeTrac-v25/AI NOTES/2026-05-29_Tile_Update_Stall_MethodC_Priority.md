# 2026-05-29 Tile Update Stall: Method C Priority Under Tight Budget

## Symptom
- Base station page showed the keyframe image but appeared static between keyframes.
- Status text stayed live (about 2-4 Hz), so websocket transport looked healthy.

## Falsification Path
1. Verified RF and base ingest still active:
   - `design-controller-image_rx-1` continued publishing non-empty tile_delta payloads.
2. Verified browser websocket feed itself was live:
   - `/ws/state` `ts_ms` advanced continuously at expected rate.
3. Compared tile blob signatures across consecutive `/ws/state` snapshots:
   - Most consecutive samples had 0 changed tile blobs.
   - Periodic near-full updates aligned with keyframe cadence.
4. This localized the issue to encoder tile-selection behavior (not websocket rendering).

## Root Cause
Under low per-frame byte budget (`LIFETRAC_FRAGMENT_BUDGET=200`) and Method C sweep enabled (`SWEEP_STEP>0`), the non-key priority sorter placed sweep-injected tiles ahead of measured-motion tiles.

Effect:
- Sweep tiles (often stale/static) consumed the very small payload capacity first.
- Real motion tiles were dropped when budget was hit.
- Operator view looked "keyframe-only" despite continuous frame flow.

## Code Changes
File:
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py`

Changes made:
- Method C default in LoRa bridge remains enabled.
- LoRa-bridge default `TILE_MAGNITUDE_MIN` set to `0` unless overridden.
- In Method B/C non-key priority sort, changed ordering from sweep-first to motion-first:
  - before: sweep tiles always won first slots
  - after: measured motion wins first, sweep fills remaining capacity
- Added optional frame-health logging hooks (`LIFETRAC_CAMERA_HEALTH_LOG`, `LIFETRAC_CAMERA_HEALTH_EVERY_S`) for future capture-source verification.

## Runtime Deploy Steps Performed
- Pushed updated `camera_service.py` to tractor bind mount:
  - `/opt/lifetrac/video-test/src/camera_service.py`
- Restarted container:
  - `tractor-camera-v2`

## Validation Snapshot (post-patch)
- `/ws/state` stream remained live.
- In short sampling windows, keyframe updates still appeared; non-key updates depend on scene motion and budget pressure.
- The sweep-starvation mechanism is now removed; true motion tiles have priority under the same budget.

## Follow-up Checks
- Re-test while introducing explicit scene motion in front of camera.
- If motion is still weak, increase budget (e.g. 200 -> 260) or reduce sweep step.
- If camera feed may be repeating, enable `LIFETRAC_CAMERA_HEALTH_LOG=1` in tractor camera service env and inspect `same_run` progression.

## Docker topology and runtime commands used

### Tractor / Unit A
- `tractor-camera-v2` — camera capture, diff, encode, and frame publish loop.
  - bind mount source: `/opt/lifetrac/video-test/src/camera_service.py`
  - container target: `/app/camera_service.py`
  - key devices / env observed during diagnosis:
    - `/dev/video1`
    - `/dev/ttymxc3`
    - `LIFETRAC_USE_LORA_BRIDGE=1`
    - `LIFETRAC_CAMERA_SOURCE=v4l2`
    - `LIFETRAC_CAMERA_DEVICE=/dev/video1`
    - `LIFETRAC_V4L2_INPUT_FORMAT=mjpeg`
    - `LIFETRAC_V4L2_INPUT_SIZE=640x480`
    - `LIFETRAC_CAMERA_FPS=1.0`
    - `LIFETRAC_TILE_PX=32`
    - `LIFETRAC_GRID_W=6`
    - `LIFETRAC_GRID_H=4`
    - `LIFETRAC_KEYFRAME_PERIOD_S=30`
    - `LIFETRAC_WEBP_QUALITY=30`
    - `LIFETRAC_FRAGMENT_BUDGET=200` (later reduced in the live low-latency patch)
- `tractor-image-tx-v2` — LoRa fragmenter / publish queue; source of queue saturation and fragment-size diagnostics.
- `tractor-mosquitto-v2` — local MQTT broker for tractor-side command ingress.

### Base station / Unit B
- `design-controller-web_ui-1` — FastAPI + websocket operator UI.
  - container target file: `/app/base_station/web_ui.py`
  - web assets: `/app/base_station/web/app.js`, `/app/base_station/web/index.html`
  - no `/dev/ttymxc3` inside this container; it cannot directly own the Murata radio UART.
- `design-controller-image_rx-1` — HostLink RX daemon and reassembly path.
  - owns `/dev/ttymxc3` and publishes reassembled `lifetrac/v25/video/tile_delta` payloads.
- `design-controller-mosquitto-1` — base-side MQTT broker.

### Docker commands used repeatedly during this debugging pass
- `docker ps --format 'table {{.Names}}\t{{.Status}}'`
- `docker logs --tail <N> <container>`
- `docker inspect <container> --format '{{json .Mounts}}'`
- `docker cp /tmp/<file> <container>:/path/in/container`
- `docker restart <container>`
- `docker stop <container>` / `docker start <container>` when a file was mounted read-only

### ADB wrapper used from the Windows host
- All Docker commands were issued on the Portenta X8s via:
  - `adb -s <serial> shell "echo fio | sudo -S docker ..."`

## Code paths used and why they matter
- [`LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py`](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py) — captures camera frames, computes changed tiles, encodes TileDeltaFrame payloads, and publishes them to MQTT / LoRa bridge.
- [`LifeTrac-v25/DESIGN-CONTROLLER/base_station/image_rx_daemon.py`](../DESIGN-CONTROLLER/base_station/image_rx_daemon.py) — reassembles LoRa fragments and republishes `tile_delta` payloads to base MQTT.
- [`LifeTrac-v25/DESIGN-CONTROLLER/base_station/web_ui.py`](../DESIGN-CONTROLLER/base_station/web_ui.py) — websocket publisher + ingest path for the live browser canvas, plus bench radio controls.
- [`LifeTrac-v25/DESIGN-CONTROLLER/base_station/web/app.js`](../DESIGN-CONTROLLER/base_station/web/app.js) — operator UI, gamepad feedback, encode-mode pill label, and control state transmission.
- [`LifeTrac-v25/DESIGN-CONTROLLER/base_station/web/index.html`](../DESIGN-CONTROLLER/base_station/web/index.html) — button layout and CSS for visible control-state feedback.

## Recovery recipe after a hard reset
1. Re-check the tractor camera runtime env and confirm `tractor-camera-v2` is still using `/opt/lifetrac/video-test/src/camera_service.py` as the source of truth.
2. Re-deploy the latest `camera_service.py`, `web_ui.py`, `app.js`, and `index.html` into the corresponding container paths with `docker cp`.
3. Restart `tractor-camera-v2` and `design-controller-web_ui-1`.
4. Verify `design-controller-image_rx-1` is publishing non-empty `tile_delta` payloads.
5. Verify the browser page shows `6x4` grid state and the tile stream is advancing.
6. If RADIO OFF fails with HTTP 500, check for missing helper executables inside the container and prefer the Python helper fallback in `web_ui.py`.
