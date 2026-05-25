# 2026-05-24 Camera Fallback Recovery Runbook v1.0

## Purpose

Restore live camera video on the base-station UI when the canvas is black due to no tile ingress.

This runbook captures the bench-proven fallback path used on 2026-05-24:

- Real camera capture on board `2E2C1209DABC240B`
- `camera_service.py` debug MQTT mirror on `lifetrac/v25/cmd/image_frame`
- Base station `web_ui.py` temporary compatibility ingest enabled

## Verified Root Cause (2026-05-24)

Black image occurred because `camera_service.py` repeatedly failed frame acquisition:

- ffmpeg stderr showed `Option not found` and filtergraph init failures
- service raised `V4l2FfmpegCamera: ffmpeg refused to produce a frame after restart`
- no valid image payloads reached MQTT ingest

Primary trigger: deshake filter chain unsupported by the static ffmpeg build in `/tmp/ffmpeg`.

## Working Configuration

Bench-stable settings:

- `LIFETRAC_V4L2_INPUT_FORMAT=mjpeg`
- `LIFETRAC_CAMERA_DESHAKE=0`
- `LIFETRAC_CAMERA_DEBUG_MQTT=1` (fallback mode only)
- `LIFETRAC_MQTT_HOST=127.0.0.1` with `adb reverse tcp:1883 tcp:1883`

Base station temporary compatibility mode:

- `LIFETRAC_ALLOW_CMD_IMAGE_FRAME=1`

## Recovery Steps

### 1. Ensure base-station web UI is running in fallback compatibility mode

```powershell
Stop-Process -Name python -Force
Set-Location "C:\Users\dorkm\Documents\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\base_station"
$env:LIFETRAC_PIN='1234'
$env:LIFETRAC_ALLOW_CMD_IMAGE_FRAME='1'
C:/Users/dorkm/AppData/Local/Python/pythoncore-3.14-64/python.exe -m uvicorn web_ui:app --host 0.0.0.0 --port 8080
```

Notes:

- If you do not want to kill all Python processes, stop only the active `uvicorn web_ui:app` PID.
- The `web_ui.py` startup race was patched on this date to avoid early MQTT callback `NameError`.

### 2. Start camera container in fallback-host-MQTT mode

Use the hardened launcher defaults:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "C:\Users\dorkm\Documents\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper\run_camera_container_x8.ps1" -AdbSerial 2E2C1209DABC240B -FallbackHostMqtt
```

Launcher now defaults to:

- `V4l2InputFormat=mjpeg`
- deshake OFF unless explicitly enabled

### 3. Confirm container health on board

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File "C:\Users\dorkm\Documents\GitHub\LifeTrac\LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper\check_camera_container_x8.ps1" -AdbSerial 2E2C1209DABC240B
```

Expected:

- container status `Up`
- `python3 /opt/lifetrac_camera/camera_service.py` present
- ffmpeg process present
- no repeating `Option not found` filtergraph errors

### 4. Validate broker ingress

```python
import time, paho.mqtt.client as mqtt
counts={}
def on_connect(c,u,f,rc,p=None):
    c.subscribe('lifetrac/v25/cmd/image_frame')
def on_message(c,u,m):
    counts[m.topic]=counts.get(m.topic,0)+1
cli=mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
cli.on_connect=on_connect; cli.on_message=on_message
cli.connect('127.0.0.1',1883,30)
cli.loop_start(); time.sleep(15); cli.loop_stop(); cli.disconnect()
print(counts)
```

Expected: topic `lifetrac/v25/cmd/image_frame` count increases.

### 5. Validate UI state stream

```python
import asyncio, json, websockets, time
async def main():
    counts=[]
    t0=time.time()
    async with websockets.connect('ws://127.0.0.1:8080/ws/state', ping_interval=None) as ws:
        while time.time()-t0 < 20:
            d=json.loads(await ws.recv())
            counts.append(len(d.get('tiles',[])))
    print('samples', len(counts), 'min', min(counts), 'max', max(counts))
asyncio.run(main())
```

Expected stable output:

- `min=96`, `max=96` for full 12x8 tile grid

## Known-Good Success Indicators

- Browser visibly shows real camera scene
- `/ws/state` tile count pinned at `96`
- `needs_keyframe=false`
- no synthetic color bars

## Exit Criteria for Leaving Fallback Mode

Before removing fallback compatibility:

1. Verified strict path producer on `lifetrac/v25/video/tile_delta`
2. Stable tile updates under strict path (same `96` tile stability checks)
3. No dependency on `/cmd/image_frame`

Then:

- restart base station without `LIFETRAC_ALLOW_CMD_IMAGE_FRAME`
- relaunch camera container without fallback debug mirror

## Strict-Path Blocker Status (2026-05-24)

Strict-path cutover is currently blocked by missing transport ingress on host COM links.

Observed evidence:

- `lora_bridge.py` starts cleanly on both `COM11` and `COM12` after nonce-store init fix.
- Broker traffic shows only:
    - `lifetrac/v25/control/link_airtime`
    - fallback `lifetrac/v25/cmd/image_frame`
- No `lifetrac/v25/video/tile_delta` observed.
- Raw serial sampling at 115200 on both COM ports showed `0 bytes` even during an active
    `run_stage1_standard_quant_end_to_end.ps1` run (3/3 PASS), so strict transport did
    not emit host-visible bytes in that window.

Operational rule:

- Do NOT disable fallback (`LIFETRAC_ALLOW_CMD_IMAGE_FRAME=1`) until tile_delta is measured live.

Strict-path gate to proceed:

1. Confirm nonzero `lifetrac/v25/video/tile_delta` message count over a 20s sample window.
2. Confirm `/ws/state` remains stable at full tile count after disabling fallback ingest.
3. Keep rollback command sequence ready (this runbook) during first strict-only validation.

## Architectural Root Cause: Strict Path Needs Base-Side Linux Host

Per [BASE_STATION.md](../DESIGN-CONTROLLER/BASE_STATION.md) and the
[lora_bridge.py](../DESIGN-CONTROLLER/base_station/lora_bridge.py) docstring,
the strict transport chain is designed to run on a Linux base-station Portenta X8
reading from `/dev/ttymxc3` (Murata L072 over LPUART1 at 921600 8N1, eventually
direct SX1276 over SPI). Tile frames flow:

```
Tractor X8 camera_service → (encode to tile_delta) → tractor L072 → LoRa air →
  base-side L072 → /dev/ttymxc3 → base-side lora_bridge.py →
    MQTT lifetrac/v25/video/tile_delta → base web_ui
```

In the current bench topology this chain is broken at the base-side ingress:

- `web_ui.py` and Mosquitto run on the Windows dev host `192.168.1.79`.
- Camera container runs on tractor X8 (`2E2C1209DABC240B`) and publishes
    `cmd/image_frame` to the Windows broker via `adb reverse tcp:1883`.
- There is no Linux base-station X8 with a connected, flashed L072/SX1276
    reachable from this host running `lora_bridge.py` against `/dev/ttymxc3`.
- `COM11` / `COM12` on Windows are not the LoRa radio receiver — both show
    `0 bytes` over 74s during an active stage1 radio cycle.

Therefore strict-only cutover is not achievable in this topology. Until a
base-side Linux host with an attached L072 is online, the fallback ingest
path documented in this runbook is the supported operational mode and must
not be disabled.

Required to lift this blocker (any one is sufficient):

1. Bring a second Portenta X8 online as the base station, flash its L072,
     run `lora_bridge.py /dev/ttymxc3` against the host broker, and verify
     the strict-path gate on its broker.
2. Run `lora_bridge.py` on the tractor X8 itself (`2E2C`) against its own
     L072 and bridge to the Windows broker — usable only for loopback /
     bench validation, not for true two-radio operation.
3. Replace fallback ingest with a base-side software shim that synthesizes
     `video/tile_delta` from `cmd/image_frame` (effectively making the
     fallback path the canonical path); requires a design decision to
     abandon the LoRa transport for video tiles on this bench.

## Rollback

If strict-path migration regresses:

1. Re-enable `LIFETRAC_ALLOW_CMD_IMAGE_FRAME=1`
2. Relaunch camera container with `-FallbackHostMqtt`
3. Re-run validation gates above
