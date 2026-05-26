# 2026-05-26 — Black Image Pane Root Cause & Remediation Options

## TL;DR

The base-station web UI (both the old Windows one on
`http://192.168.1.79:8080/` and the new X8 one on
`http://192.168.1.117:8080/`) shows a **black image pane** because **no
component anywhere in the system is currently emitting image frames**.

- Base `lora_bridge` reports `U_image=0.0%` on every encoder cycle.
- Tractor X8 (`2E2C1209DABC240B`) has the **systemd `lifetrac-camera.service`
  installed but it is a placeholder** — its ExecStart literally falls through
  to `echo "lifetrac-camera: compose app not installed; holding C2 device
  sentinel"` because the expected compose file at
  `/opt/lifetrac/compose-apps/lifetrac-camera/docker-compose.yml` does not
  exist on the device, and the source for that compose-app does not exist in
  this repository either.

So the LoRa-side ladder works end-to-end, but there is nothing upstream of
the M7 producing TileDeltaFrame payloads. The base side has been ready for
weeks; the tractor encoder side has never been packaged.

## Evidence collected (this session)

### Base X8 (`2D0A1209DABC240B`, eth0 192.168.1.117)
- `docker ps` shows the design-controller stack Up:
  `design-controller-{mosquitto,lora_bridge,web_ui,audit_tail}-1`.
- `lora_bridge` log every cycle:
  ```
  CMD_ENCODE_MODE → FULL (U_image=0.0%)
  CMD_ENCODE_MODE → Y_ONLY (U_image=0.0%)
  CMD_ENCODE_MODE → MONO_G4 (U_image=0.0%)
  ```
  `U_image` is the moving average of image-frame bytes received from the
  L072 over `/dev/ttymxc3`. Zero across all three encoder modes proves the
  base has received no image fragments since boot.
- LAN smoke `Invoke-WebRequest http://192.168.1.117:8080/` returns 200 with
  the encoder cycle + safe-mode pill HTML.

### Tractor X8 (`2E2C1209DABC240B`, wlan0 192.168.1.114)
Survey via `survey_tractor_image.sh`:

| Check | Result |
|---|---|
| Kurokesu C2 camera | Present, `/dev/video1..4` (`C2: C2 KTM-QGFST`) |
| Onboard MIPI-CSI | `/dev/video0` (`mx6s-csi`) |
| LoRa serial (M7 host) | `/dev/ttymxc3` (207,19) present, root:dialout |
| `lifetrac-camera.service` | active (running) for 22h, **but is the placeholder branch** |
| `lifetrac-params/-logger/-time` | NOT loaded |
| Docker containers | only foundries `x8-{devel,provisioning,webapp}` |
| `/opt/lifetrac` directory | DOES NOT EXIST |
| `/tmp/ffmpeg` | not present |
| Ping → base 192.168.1.117 | 12–28 ms over wlan0 (works) |
| Disk free | 12 GB |

The systemd unit's relevant line:
```
ExecStart=/bin/sh -lc 'if [ -f "$LIFETRAC_CAMERA_COMPOSE" ] && command -v docker >/dev/null 2>&1; then exec docker compose -f "$LIFETRAC_CAMERA_COMPOSE" up --remove-orphans; fi; echo "lifetrac-camera: compose app not installed; holding C2 device sentinel"; while [ -e /dev/lifetrac-c2 ]; do sleep 5; done; exit 1'
```
`journalctl -u lifetrac-camera` confirms the placeholder branch was taken.

### What exists in the repo that we'd need
- `firmware/tractor_x8/camera_service.py` — full TileDeltaFrame encoder, ready.
- `firmware/tractor_x8/image_tx_daemon.py` — MQTT/UART forwarder, ready.
- `firmware/tractor_x8/image_pipeline/` — supporting code, ready.
- `firmware/tractor_x8/requirements.txt` — picamera2 / pillow / paho / etc.
- **MISSING**: any `Dockerfile` or `docker-compose.yml` for the tractor side.
  All planning notes (see `2026-05-14_USB_Wedge_Software_Mitigations.md`
  §D) refer to a compose app at
  `/opt/lifetrac/compose-apps/lifetrac-camera/` that was scaffolded into
  the systemd unit but never authored.

## Three remediation paths

### A. Production-fidelity LoRa path
Build the missing tractor compose-app:
- `Dockerfile`: python:3.11-slim + apt(mosquitto-clients libusb-1.0-0
  ffmpeg) + pip install requirements.txt, COPY firmware/tractor_x8/.
- `docker-compose.yml`: one service running `camera_service.py` (and
  optionally `image_tx_daemon.py`), device-maps `/dev/video1` and
  `/dev/ttymxc3`, sets `LIFETRAC_CAMERA_DEVICE=/dev/video1`,
  `LIFETRAC_MQTT_HOST=mosquitto`, and runs an internal mosquitto so the
  M7 firmware can subscribe to `lifetrac/v25/cmd/image_frame`.
- Place at `/opt/lifetrac/compose-apps/lifetrac-camera/docker-compose.yml`
  on the tractor; restart `lifetrac-camera.service`.

Pros: exercises the real LoRa path the field unit will use.
Cons: requires the M7 firmware to be flashed, healthy and subscribed to
the local MQTT topic, AND the L072 ↔ L072 LoRa link must be operational
end-to-end. Both unverified at the moment. ~384×256 @ ~2 fps ceiling.

### B. Ethernet shortcut (recommended for first "pixels on screen" test)
Skip the M7 / LoRa hops entirely. Have the tractor publish completed
TileDeltaFrames straight to the BASE mosquitto over LAN.

Steps:
1. **Base side**: open a non-conflicting host port on the design-controller
   mosquitto so the tractor can reach it. The foundries `mosq` container
   already owns host:1883, so map e.g. `["192.168.1.117:11883:1883"]`.
   Update `LIFETRAC_MQTT_HOST` / `LIFETRAC_MQTT_PORT` env, restart stack.
2. **Bridge layer**: confirm that the base `image_rx_daemon.py` (or
   whatever subscribes to the assembled-frame topic) will accept a frame
   that *bypasses* `lora_bridge`'s defragmenter. If the web_ui consumes
   only `lifetrac/v25/img/canvas` (post-reassembly), the tractor needs to
   publish there directly. (Code read required — out of scope of this
   note.)
3. **Tractor side**: minimal docker image with just `camera_service.py`,
   `LIFETRAC_MQTT_HOST=192.168.1.117`, `LIFETRAC_MQTT_PORT=11883`,
   `LIFETRAC_CAMERA_DEVICE=/dev/video1`, mount the same.

Pros: fast feedback, no M7/LoRa dependency, full-quality frames.
Cons: doesn't validate the production path; needs a small wiring tweak on
both sides.

### C. Static placeholder
Patch the web UI to serve a "Camera offline — tractor encoder not deployed"
banner instead of a black canvas, so the bench state is visually obvious
until A or B lands.

Cons: hides the real diagnostic signal (`U_image=0.0%`); not really a fix.

## Recommendation

Implement **B first** to validate the UI canvas / safe-mode / encoder-cycle
end-to-end with real pixels, then implement **A** before shipping. Building
the tractor compose-app once is enough — it can target both A (publish to
local mosquitto, M7 picks up) and B (publish to remote mosquitto) via env
config.

## Decisions needed from the user (will not be made autonomously)

1. **Path**: A, B, or both?
2. **Image scope**: do we want the full 12×8 tile pipeline, or a simpler
   bench MJPEG → single-image canvas for first light? The repo's
   image_pipeline tests (`test_image_pipeline.py`, `test_e2e_image_pipeline.py`)
   are written around tiles, so leaving the tile pipeline in place is the
   safer choice — but it costs more bring-up effort.
3. **Tractor MQTT broker**: run a sidecar `mosquitto` inside the tractor
   compose (production-faithful), or skip it for B and point straight at
   the base broker?
4. **Network for option B**: open base `mosquitto` on
   `192.168.1.117:11883` (TLS-less, bench only)? Or VPN-style mTLS now?
5. **Touching prod compose**: am I allowed to add a `ports:` block to the
   workspace `docker-compose.yml`, or should I leave it bench-only via a
   compose override file (`docker-compose.bench.yml`)?

## Repo memory updated
- `/memories/repo/lifetrac-tractor-image-pipeline-status.md` (NEW)
  captures the placeholder-service finding so future sessions don't re-
  diagnose "image is black" from scratch.

## Files referenced
- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py](../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py)
- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/image_tx_daemon.py](../DESIGN-CONTROLLER/firmware/tractor_x8/image_tx_daemon.py)
- [LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/systemd/lifetrac-camera.service](../DESIGN-CONTROLLER/firmware/tractor_x8/systemd/lifetrac-camera.service)
- [LifeTrac-v25/DESIGN-CONTROLLER/base_station/lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py)
- [LifeTrac-v25/AI NOTES/2026-05-14_USB_Wedge_Software_Mitigations.md](2026-05-14_USB_Wedge_Software_Mitigations.md) §D
- [LifeTrac-v25/DESIGN-CONTROLLER/base_station/.scratch/survey_tractor_image.sh](../DESIGN-CONTROLLER/base_station/.scratch/survey_tractor_image.sh)
