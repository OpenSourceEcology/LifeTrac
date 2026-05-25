# 2026-05-25 — Live Camera Pipeline End-to-End

## Status

✅ End-to-end live-camera pipeline plumbed and verified.
✅ Kurokesu C2 (`/dev/video1`) → `publish_camera_frames` container → MQTT
   `lifetrac/v25/cmd/image_frame` → `image_tx_daemon` → LoRa SF7/BW250 →
   `image_rx_daemon` → MQTT `lifetrac/v25/video/tile_delta` → `web_ui`
   `_ingest_tile_delta` → `Canvas.apply` → `/ws/state` →
   `canvas_renderer.blitTiles` → painted on `#image-canvas`.

Best run (budget=250, fps=1, 60 s): publisher=60, tx_in=58,
rx_frames=310, frames_published=55, reassembler_timeouts=0
→ 92 % single-fragment delivery success, 3 tiles painted on the canvas.

## Three independent issues found while debugging "I only see black"

### 1. Pillow missing from the Foundries devel image (TX side)

`hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44` does not
ship Pillow. `camera_service._encode_tile` does `from PIL import Image`,
so without it the encoder raises and the publisher emits nothing.

**Fix** (one-shot bootstrap, must be run before the first pipeline run):

```powershell
adb -s 2E2C1209DABC240B shell "echo fio | sudo -S docker run --rm -v /tmp/lifetrac_strict:/work --entrypoint pip hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 install --target=/work/site Pillow"
```

Then `run_camera_pipeline.ps1` must put `/work/site` on `PYTHONPATH`:

```text
-e PYTHONPATH=/work:/work/paho:/work/site
```

### 2. Login was returning 503 ("PIN not configured")

`LifeTrac-v25/DESIGN-CONTROLLER/base_station/web_ui.py` `_load_operator_pin`
is fail-closed: empty / unset → `/api/login` returns 503.
There is **no coded default**.

Workaround for bench: restart uvicorn with
`$env:LIFETRAC_PIN = "1234"` before launching `uvicorn web_ui:app …`.
This is a runtime-only setting; do **not** hard-code "1234" in the source.

### 3. Canvas is sized 0 × 0 in a small-viewport browser

CSS in `index.html`:

```css
main { grid-template-rows: 1.2fr auto auto 1fr; height: calc(100% - 80px); }
#image-panel { display: flex; min-height: 0; }
.image-wrapper { height: 100%; aspect-ratio: 1.5; }
```

`#image-panel` has no explicit grid-row and inherits whatever the auto
algorithm leaves over. In the integrated VS Code browser (≈ 458 px main
height), the panel collapses to 72 px high and the wrapper computes to
`0 × 0`. The buffer canvas (`width=384 height=256`) is still drawn into
but it has zero CSS area → invisible.

Confirmed via Playwright `getComputedStyle`. After force-setting
`#image-panel.style.height = '300px'` the wrapper becomes 450 × 300 and
the painted tiles become visible.

Action item: give `#image-panel` an explicit `min-height` (e.g. `220px`)
or move it into a fixed grid-row so it cannot collapse below ~ 200 px.

## Why the canvas still shows only a top-left strip of tiles (by design)

Three protocol facts compound:

1. **Fixed-budget encoder.** `camera_service._build_frame` encodes
   changed tiles in `(rank, idx)` priority order, capped by
   `LIFETRAC_FRAGMENT_BUDGET` (default 250 B). Header alone is 17 B
   (5 + 12 bitmap), so a 250 B budget fits 1–2 tiles per frame. The
   priority key is `(rank, idx)` so the **lowest-index** changed tiles
   always win and the rest are dropped. Top-left tiles always paint
   first; far-right / bottom tiles never get through at this budget.

2. **`KEYFRAME_PERIOD_S` = 10 s.** A keyframe sets every bit in the
   changed bitmap, but the budget cap then drops 94 of the 96. A "full"
   keyframe that actually covers the whole canvas would need
   `budget ≈ 96 × ~80 B + 17 = ~7.7 kB`, ≈ 30 LoRa fragments at SF7/BW250
   — well over the link's airtime budget.

3. **`base_seq` gap rejection.** `Canvas.apply` requires
   `frame.base_seq == (last + 1) & 0xFF`; any miss → request keyframe,
   drop subsequent deltas until next keyframe. With ~ 8 % single-fragment
   loss, each gap erases ~ 5 deltas (until the next keyframe at + ≤ 10 s).

**Net effect:** at the SF7/BW250 / 250 B-budget LoRa profile, the canvas
converges to a small top-left rectangle of "currently-changing" tiles.
This is the link bandwidth showing through the protocol, not a bug.

### Knobs exposed in `run_camera_pipeline.ps1`

```text
-FragmentBudget   [int]  default 250    bytes per LoRa frame
-KeyframePeriodS  [dbl]  default 10.0   seconds between keyframes
-PublishFps       [int]  default 2      camera publish rate
```

For a denser bench paint (still slow), try
`-FragmentBudget 1500 -KeyframePeriodS 2.0 -PublishFps 1 -DurationS 180`.
Note budget≥ 1500 makes keyframes need ≥ 6 LoRa fragments, so reassembly
success drops sharply (run3: budget=1500 → 20 / 120 frames reassembled,
all 12 keyframes lost, canvas stayed empty with
`last_keyframe_reason = "delta arrived before any keyframe"`).

## Falsification tests run (per methodology.md)

* Black canvas hypothesis "wrapper hidden by SOURCE=NONE arbitration":
  **falsified.** The wrapper geometry is determined purely by CSS grid
  + flexbox parent height, independent of any source-arbitration state.
  Force-resizing `#image-panel` to 300 px made painted tiles visible
  even with `SOURCE: NONE`.
* Hypothesis "FRAGMENT_BUDGET = 250 is too small, just bump it":
  **falsified by run3.** Budget = 1500 broke the keyframe path
  (multi-fragment frames lost) so the Canvas saw zero keyframes and
  `tiles: []` in `/ws/state`. Higher budget made the visible result
  *worse*, not better.
* Hypothesis "camera hardware broken": **falsified.** `probe_camera.sh`
  pulled a clean 70 kB MJPEG kitchen-scene frame from `/dev/video1`.
* Hypothesis "publisher dies silently on `from PIL import Image`":
  **confirmed and fixed.** Pre-fix the publisher exited at first encode;
  post Pillow-bootstrap the publisher emits 60 frames in 60 s.

## TODO

* Add a one-shot Pillow bootstrap guard at the top of
  `run_camera_pipeline.ps1` (test for `/tmp/lifetrac_strict/site/PIL` →
  run the `--entrypoint pip … install --target=/work/site Pillow` if
  missing).
* Give `#image-panel` a `min-height: 220px` so the canvas never
  collapses in small viewports.
* Investigate a tile-age-weighted priority (so tiles that have not
  shipped in N keyframes climb to the front of the queue) — would let
  the canvas converge to full coverage over a few keyframes even at
  budget = 250.
