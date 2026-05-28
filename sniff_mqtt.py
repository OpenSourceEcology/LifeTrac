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
