# Base Station visual pipeline debugging & recovery guide (2026-05-28)

## 1. Overview and Objectives
During this visual tiles-ingest session, we validated and debugged the visual pipeline routing tile-delta frames representing dithered WebP elements. The pipeline flows from the Tractor (Unit A) camera service down over the LoRa air link to the operator screen of the Base Station (Unit B).

We successfully aligned co-processors to the standard 915.0 MHz sub-gig band (regulatory RF Profile 0), confirmed error-free fragmentation assembly, and debugged the grid adoption deadlock. The operator dashboard's dynamic adoption patch is documented below to prevent regression in the event of a hard reset.

---

## 2. Network Topology & Port Mappings
* **Base Station physical IP**: `192.168.1.117`
* **Operator Console UI URL**: `http://192.168.1.117:8080/`
* **Web UI WebSocket State Endpoint**: `ws://192.168.1.117:8080/ws/state`
* **Tractor MQTT Broker Host (Internal docker bridge)**: `tractor-mosquitto-v2:1883`
* **Base Station MQTT Broker Host (Internal docker bridge)**: `mosquitto:1883`

---

## 3. Active Containers & Services
Both Portenta X8 boards run lightweight multi-container environments. In case of a hard wipe, ensure the following containers are mapped and active:

### Tractor (Unit A) — ADB Serial: `2E2C1209DABC240B`
* `tractor-camera-v2`: Captures frame buffers from V4L2 device `/dev/video0` or `/dev/video1`, divides them into 32px grids, computes temporal changes, encodes active segments, and produces payloads.
* `tractor-image-tx-v2`: Hooks into `tractor-camera-v2` over local MQTT, compresses chunks as standard LoRa radio fragments, and pipes them out of the L072 RF co-processor port `/dev/ttymxc3` (921600 baud).
* `tractor-mosquitto-v2`: Standard local MQTT broker interfacing the sensor services.

### Base Station (Unit B) — ADB Serial: `2D0A1209DABC240B`
* `design-controller-web_ui-1`: Hosts the FastAPI web-app acting as the operator dashboard state gateway. Serves content, tracks dynamic canvas buffers, publishes state snapshots on websocket `/ws/state`, and processes telemetry arrays.
* `design-controller-image-rx-daemon-1` / `lora_bridge`: Manages physical Base L072 packet reassembly, monitors incoming sequence ranges, and forwards assembled frame structures back onto local broker sub-topics.
* `design-controller-mosquitto-1` (mapped locally as `mosquitto` on standard port `1883` for container networking).

---

## 4. Problem Pinpointed: The Grid Adoption Mismatch
The backend of [LifeTrac-v25/DESIGN-CONTROLLER/base_station/web_ui.py](LifeTrac-v25/DESIGN-CONTROLLER/base_station/web_ui.py) originally initialized a rigid `12x8` frame template for 1920x1080 screens or general standard feeds. 

Under test configurations (e.g. video benchmarks), the tractor outputs low-bandwidth streams composed of `6x4` tiles (size 192x128 pixels). Because automated back-channel lora feedback routing is disconnected in visual-evaluation modes, the web dashboard could not negotiate layout dimensions back to the camera. The incoming `6x4` keyframes were rejected inside [LifeTrac-v25/DESIGN-CONTROLLER/base_station/image_pipeline/canvas.py](LifeTrac-v25/DESIGN-CONTROLLER/base_station/image_pipeline/canvas.py) due to a geometry mismatch, leaving the operator view blank.

### The Fix
We patched `_ingest_tile_delta` in [LifeTrac-v25/DESIGN-CONTROLLER/base_station/web_ui.py](LifeTrac-v25/DESIGN-CONTROLLER/base_station/web_ui.py) to automatically adjust base station canvas parameters upon receiving a keyframe whose design dimensions differ from the active state:

```python
def _ingest_tile_delta(payload: bytes) -> None:
    global _image_canvas
    with _image_lock:
        frame = _image_reassembler.feed(payload)
        if frame is None:
            return
        # 2026-05-27: auto-adopt the upstream camera's grid on the first
        # keyframe whose layout differs from our current Canvas. Before
        # this, _image_canvas was hardcoded 12x8@32px and any camera with
        # a different grid (e.g. the LoRa-bridge synthetic 6x4@32px) was
        # rejected by Canvas.apply() with "grid mismatch", leaving every
        # /ws/state snapshot with tiles=[] and the operator canvas black.
        if frame.is_keyframe and (
            (frame.grid_w, frame.grid_h, frame.tile_px)
            != (_image_canvas.grid_w, _image_canvas.grid_h, _image_canvas.tile_px)
        ):
            _image_canvas = Canvas(
                grid_w=frame.grid_w,
                grid_h=frame.grid_h,
                tile_px=frame.tile_px,
            )
            _image_publisher.canvas = _image_canvas
        update = _image_canvas.apply(frame)
```

---

## 5. Scripts and Commands Used

### Triggering Keyframes from Tractor Command Line
Create a manual override script `/tmp/req_keyframe.py` on the tractor co-processor inside `tractor-camera-v2`:

```python
import paho.mqtt.publish as p
print("Publishing keyframe request to tractor-mosquitto-v2 on lifetrac/v25/cmd/req_keyframe...")
p.single('lifetrac/v25/cmd/req_keyframe', b'trigger', hostname='tractor-mosquitto-v2')
print("Published!")
```

Run via PowerShell Host to issue the trigger over ADB:
```powershell
$adb = "C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe"
& $adb -s 2E2C1209DABC240B shell "echo fio | sudo -S docker exec tractor-camera-v2 python /tmp/req_keyframe.py"
```

### Base Station Deep Pipeline Sniffer & Validator
Create a pipeline sniffer `/tmp/sniff_mqtt.py` inside modern base station `design-controller-web_ui-1` container to verify structural ingestion, parse sequences, and test canvas allocation changes:

```python
import paho.mqtt.subscribe as s
import time
import sys
from image_pipeline.frame_format import parse_tile_delta_frame

print("Loading web_ui...")
try:
    from web_ui import _ingest_tile_delta, _image_publisher
except Exception as exc:
    print("Failed to import from web_ui:", exc)
    sys.exit(1)

print("Sniffer started. Waiting for keyframe...")
start_time = time.time()
while time.time() - start_time < 30:
    p = s.simple('lifetrac/v25/video/tile_delta', hostname='mosquitto')
    kind = p.payload[0]
    print(f"Message: kind={kind}, len={len(p.payload)}")
    if kind == 1:
        print("Received Keyframe payload. Parsing first...")
        try:
            frame = parse_tile_delta_frame(p.payload)
            print(f"PARSED FRAME: grid={frame.grid_w}x{frame.grid_h}, tile_px={frame.tile_px}, codec={frame.codec}, is_keyframe={frame.is_keyframe}")
        except Exception as parse_exc:
            print("Failed to parse frame:", parse_exc)
            break

        print("Running web_ui._ingest_tile_delta...")
        print(f"Before ingest, publisher canvas grid: {_image_publisher.canvas.grid_w}x{_image_publisher.canvas.grid_h}")
        try:
            _ingest_tile_delta(p.payload)
            print("INGEST SUCCESSFUL!")
            print(f"After ingest, publisher canvas grid: {_image_publisher.canvas.grid_w}x{_image_publisher.canvas.grid_h}")
        except Exception as exc:
            print("INGEST FAILED with exception:", exc, type(exc))
            import traceback
            traceback.print_exc()
        break
```

Run via PowerShell ADB:
```powershell
$adb = "C:\Users\dorkm\AppData\Local\Microsoft\WinGet\Packages\Google.PlatformTools_Microsoft.Winget.Source_8wekyb3d8bbwe\platform-tools\adb.exe"
& $adb -s 2D0A1209DABC240B shell "echo fio | sudo -S docker exec -w /app design-controller-web_ui-1 python3 /tmp/sniff_mqtt.py"
```

---

## 6. Full Recovery Run-Book (If Containers Reset)
In the event of a total wipe or hardware replacement of the Portenta X8 co-processors, perform the following steps sequentially:

1. **Mount & Copy Web UI Patch**:
   Copy the patched [LifeTrac-v25/DESIGN-CONTROLLER/base_station/web_ui.py](LifeTrac-v25/DESIGN-CONTROLLER/base_station/web_ui.py) file back into the docker runtime:
   ```powershell
   & $adb -s 2D0A1209DABC240B push LifeTrac-v25/DESIGN-CONTROLLER/base_station/web_ui.py /tmp/web_ui.py
   & $adb -s 2D0A1209DABC240B shell "echo fio | sudo -S docker cp /tmp/web_ui.py design-controller-web_ui-1:/app/base_station/web_ui.py"
   & $adb -s 2D0A1209DABC240B shell "echo fio | sudo -S docker restart design-controller-web_ui-1"
   ```
2. **Setup Tractor Keyframe Trigger script**:
   If the Tractor container was wiped, copy the keyframe trigger script back to `tractor-camera-v2`:
   ```powershell
   & $adb -s 2E2C1209DABC240B shell "echo 'import paho.mqtt.publish as p; p.single(\"lifetrac/v25/cmd/req_keyframe\", b\"trigger\", hostname=\"tractor-mosquitto-v2\")' > /tmp/req_keyframe.py"
   & $adb -s 2E2C1209DABC240B shell "echo fio | sudo -S docker cp /tmp/req_keyframe.py tractor-camera-v2:/tmp/req_keyframe.py"
   ```
3. **Trigger Visual Frame Synced State**:
   Execute the manual keyframe script to align the active grids immediately:
   ```powershell
   & $adb -s 2E2C1209DABC240B shell "echo fio | sudo -S docker exec tractor-camera-v2 python /tmp/req_keyframe.py"
   ```
4. **Inspect State Snapshot**:
   Connect a browser or inspect state over the dashboard API to confirm the `grid` fields read `{"w": 6, "h": 4, "tile_px": 32}` and the tile updates show zero delays.
