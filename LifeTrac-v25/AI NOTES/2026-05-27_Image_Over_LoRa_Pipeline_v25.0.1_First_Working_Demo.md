# Image-Over-LoRa Pipeline — v25.0.1 (First Working End-to-End Demo)

Date: 2026-05-27
Author: Copilot (Claude Opus 4.7)
Status: **MILESTONE — first end-to-end success.** Real USB camera frames captured on the tractor X8, encoded as per-tile WebP, fragmented over the Murata SX1276 LoRa link, reassembled on the base-station X8, and rendered on the operator web UI at `http://192.168.1.117:8080/`. Synthetic test pattern retired. Bandwidth thrift verified.

This note pins the demonstrated pipeline as **Image-Over-LoRa Pipeline v25.0.1** so we have a stable label to improve against. It complements (does not replace) the broader v25.0.1 release packaging plan in [2026-05-26_v25.0.1_Release_Packaging_Plan_Copilot_v1_0.md](2026-05-26_v25.0.1_Release_Packaging_Plan_Copilot_v1_0.md).

---

## 1. What "v25.0.1" means for the image pipeline

| Aspect | v25.0.1 baseline |
|---|---|
| Camera source | Real USB UVC camera (`C2 KTM-QGFST`) on `/dev/video1`, MJPEG ingest |
| Capture chain | static arm64 ffmpeg 7.0.2 → raw RGB → tiler |
| Capture geometry | 640×480 @ 5 fps → downscaled to 192×128 RGB24 |
| Tile grid | 6 cols × 4 rows × 32 px (24 tiles, full canvas 192×128) |
| Encoder | Per-tile WebP, `WEBP_QUALITY=30`, keyframe period 30 s |
| Encode mode | `full` (every tile every frame at 1 fps when changed; tile-delta otherwise) |
| Frame rate on air | ~1.0 fps tile snapshot, P-frames suppressed when no tile changes |
| Transport | Murata Type-1SC SX1276 @ 921600 UART, SF7/BW500, ~48 ms TX-on-air/fragment, `FRAGMENT_BUDGET=200` B |
| Bridge | `LIFETRAC_USE_LORA_BRIDGE=1` (radio path active; back-channel ladder bypassed by design for v25.0.1) |
| Idle bandwidth | ~260 B/s sustained (1× 2547 B keyframe + 9× 9 B P-frames per 10-frame sample) |
| Base UI | `lifetrac-vtest-web_ui-1` rendering `#image-canvas` via [canvas_renderer.js](../base_station/web/img/canvas_renderer.js) + [fade_shader.js](../base_station/web/img/fade_shader.js) |

---

## 2. Components frozen at v25.0.1

Tractor X8 (`adb 2E2C1209DABC240B`, host `/opt/lifetrac/video-test/`):

- `camera_service.py` — `V4l2FfmpegCamera` selected via `LIFETRAC_CAMERA_SOURCE=v4l2`.
- `docker-compose.yml` — camera env block locked to v4l2/MJPEG/640×480@5/6×4 grid/tile_px=32/WEBP_QUALITY=30/FRAGMENT_BUDGET=200/keyframe 30 s/cam fps 1.0; binds `/dev/video1` and `/opt/lifetrac/bin/ffmpeg`.
- `/opt/lifetrac/bin/ffmpeg` — static arm64 build 7.0.2 (51 MB), installed mode 0755.
- Containers: `tractor-camera-v2`, `tractor-image-tx-v2`, `tractor-mosquitto-v2` on `tractor-vtest_default`.

Base X8 (`adb 2D0A1209DABC240B`, LAN 192.168.1.117:8080, PIN 1234):

- Containers: `lifetrac-vtest-web_ui-1`, `lifetrac-vtest-image_rx-1`, `lifetrac-vtest-mosquitto-1`.
- [fade_shader.js](../base_station/web/img/fade_shader.js) — patched this session to read live `gridW`/`gridH`/`tilePx` from the `lifetrac-state` event and to use server-supplied `tile.tx`/`tile.ty`, fixing the hard-coded 12-column assumption that was clustering every freshness pulse into the top-left 192×32 strip on the 6×4 canvas.

Wire format unchanged: `TileDeltaFrame` = `struct.pack("BBBBBB", frame_kind, seq, grid_w, grid_h, tile_px, codec)` + ceil(W·H/8) presence bitmap + per-tile `(size-1:u8, blob)`; codecs 0=WEBP, 1=MONO_G4, 4=WEBP_LUMA.

---

## 3. What is explicitly NOT in v25.0.1 (deferred to v25.0.2+)

- Adaptive encoder ladder driven by airtime pressure (`CMD_ENCODE_MODE` 0x63, `CMD_LINK_PROFILE`, `CMD_ROI_HINT` 0x61). Code paths exist in `camera_service.py` / `web_ui.py` but are bypassed while `USE_LORA_BRIDGE=1`.
- `y_only` / `btc4_per_tile` / `btc4_per_frame` / `mono_g4` / `motion_only` / `wireframe` modes — present but not exercised over the link in this demo.
- Recolouriser wiring on the base UI.
- Removal of orphan `video-test_default` docker network.
- Increasing keyframe period beyond 30 s, lowering `WEBP_QUALITY` below 30, or raising tile grid resolution.
- Persisting workspace edits (camera_service.py, docker-compose.yml, fade_shader.js, web_ui.py, canvas.py, frame_format.py, codec_decode.py) back to git — currently hot-patched in the live containers only.
- Field-legal regulatory profile. v25.0.1 remains **bench-only** (`BENCH_ONLY_FIXED_915`, profile 0), matching [2026-05-26_v25.0.1_Release_Packaging_Plan_Copilot_v1_0.md](2026-05-26_v25.0.1_Release_Packaging_Plan_Copilot_v1_0.md).

---

## 4. Reproduction check (smoke)

1. Tractor host: `sudo docker ps` shows `tractor-camera-v2`, `tractor-image-tx-v2`, `tractor-mosquitto-v2` all Up.
2. Tractor host: `sudo docker logs tractor-camera-v2 --tail 5` shows `v4l2/ffmpeg pid=<n> device=/dev/video1 640x480@5fps -> 192x128 rgb24`.
3. Base host: `mosquitto_sub -t 'lifetrac/image/tiles' -F '%l B'` shows a 2.5 KB keyframe at startup followed by sparse 9 B P-frames.
4. Browser at `http://192.168.1.117:8080/` (PIN 1234) renders the live camera scene on `#image-canvas`; the header shows `enc: full ▸` and `3 Hz · tile stream`; the synthetic-pattern banner is absent.

---

## 5. Improvement roadmap (build forward from v25.0.1)

In rough priority order, each ticketable as v25.0.x:

1. Persist the live config to git (camera_service env, docker-compose, fade_shader.js fix) so the demo survives a reboot of either X8.
2. Wire the back-channel so `POST /api/encode_mode/cycle` actually changes the radio-side encoder mode under `USE_LORA_BRIDGE=1`. Validate `motion_only` and `mono_g4` reduce idle airtime further.
3. Lower `WEBP_QUALITY` to ~20 and raise `LIFETRAC_KEYFRAME_PERIOD_S` to 60 s; measure keyframe size.
4. Add tile-change-rate telemetry to the web UI footer (B/s, fragments/s, fragment loss %).
5. Recolouriser pass on the base UI for low-quality WebP banding.
6. Re-tile to 8×6 @ 24 px once airtime budget is characterised under motion.
7. Field-legal regulatory profile gate before any outdoor demo.

---

## 6. Cross-references

- Plan that led here: [2026-05-18_W2-02_Image_Over_LoRa_Bench_Plan_Copilot_v1_0.md](2026-05-18_W2-02_Image_Over_LoRa_Bench_Plan_Copilot_v1_0.md).
- Prior air-link proof: [2026-05-26_Image_Over_LoRa_AirLink_Proven_Status_Copilot_v1_0.md](2026-05-26_Image_Over_LoRa_AirLink_Proven_Status_Copilot_v1_0.md).
- Release packaging plan (governs how "v25.0.1" is cut): [2026-05-26_v25.0.1_Release_Packaging_Plan_Copilot_v1_0.md](2026-05-26_v25.0.1_Release_Packaging_Plan_Copilot_v1_0.md).
- Pipeline-visual verification (synthetic era): [2026-05-25_End_to_End_Pipeline_Visual_Verified.md](2026-05-25_End_to_End_Pipeline_Visual_Verified.md).
- Black-pane diagnosis that preceded the v4l2 cutover: [2026-05-26_Black_Image_Pane_Diagnosis.md](2026-05-26_Black_Image_Pane_Diagnosis.md).
