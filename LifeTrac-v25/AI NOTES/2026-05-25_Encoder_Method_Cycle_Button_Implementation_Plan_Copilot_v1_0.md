# Encoder Method Cycle Button — Implementation Plan v1.0

**Date:** 2026-05-25
**Author:** Copilot (assistant)
**Status:** BENCH-READY (steps 1-4, 8-11 complete; steps 5-7, 12 deferred)
**Scope:** Ship 5 selected encoder methods + cycle button (web + gamepad) +
auto-demote-on-signal-loss. Operator pin stays sticky; controller only
overrides it when the link goes critical.

## Completion log (this push, 2026-05-25)
- ✅ Step 1 — wire-format `codec` byte (HEADER_FIXED_LEN=6, ids 0..15 with
  0-3 locked). 3 new tests in `test_image_pipeline.py`.
- ✅ Step 2 — base-side codec dispatch via `image_pipeline/codec_decode.py`
  + LRU transcode cache in `image_pipeline/canvas.py`.
- ✅ Step 3 — safe-mode hysteresis on `EncodeModeController`
  (SAFE_MODE_CRITICAL=0.95 × 3 windows engages; SAFE_MODE_CLEAR=0.60 × 5
  windows releases). Edge callback fires once per transition;
  `lora_bridge` publishes retained
  `lifetrac/v25/control/safe_mode_active` JSON. 3 new tests in
  `test_link_monitor.py`.
- ✅ Step 4 — MONO_G4 vertical slice (tractor encoder
  `_encode_tile_mono_g4` + base decoder `_decode_mono_g4`). 2 new tests.
- ⏳ Steps 5-7 — BTC4_PER_TILE / BTC4_PER_FRAME / ADAPTIVE codecs
  deferred; tractor still demotes to MONO_G4 if those modes are pinned.
- ✅ Step 8 — `POST /api/encode_mode/cycle` endpoint + shared
  `_set_encode_mode_override` helper, narrow cycle order
  `auto → full → y_only → mono_g4 → auto`.
- ✅ Step 9 — `#encode-mode-pill` in index.html with short-click cycle,
  long-press → settings.
- ✅ Step 10 — gamepad BACK/SELECT (button idx 8) cycles via the same
  endpoint, edge-triggered.
- ✅ Step 11 — safe-mode banner reuses the encoder pill (red flash via
  `.encmode-pill.safe`); driven off the retained telemetry topic.
- ⏳ Step 12 — bench rotation PS script deferred (manual cycle via pill
  or gamepad is sufficient for first bench run).

40/40 tests green:
`py -m pytest tests/test_image_pipeline.py tests/test_link_monitor.py tests/test_e2e_image_pipeline.py -q`.

---

Companion docs:
- `LifeTrac-v25/AI NOTES/2026-05-25_Grayscale_Quantization_Encoding_Research_Copilot_v1_0.md` (research catalogue)
- `LifeTrac-v25/DESIGN-CONTROLLER/LORA_PROTOCOL.md` § TileDeltaFrame + § 0x63 CMD_ENCODE_MODE
- `LifeTrac-v25/DESIGN-CONTROLLER/base_station/link_monitor.py` (EncodeModeController)
- Session memory: `/memories/session/phase2-encoder-scope-pending.md` (decision matrix)

---

## 0. What already shipped (Phase 1, this session)

| Layer | Change | File |
| --- | --- | --- |
| Wire enum | `EncodeMode` extended 0..7, `ENCODE_MODE_LADDER` tuple | [lora_proto.py](LifeTrac-v25/DESIGN-CONTROLLER/base_station/lora_proto.py) |
| Controller | `set_operator_ceiling(mode\|None)` + `_apply_operator_ceiling` | [link_monitor.py](LifeTrac-v25/DESIGN-CONTROLLER/base_station/link_monitor.py) |
| Bridge | Subscribe `lifetrac/v25/control/encode_mode_override`, call controller | [lora_bridge.py](LifeTrac-v25/DESIGN-CONTROLLER/base_station/lora_bridge.py) |
| HTTP | `GET/POST /api/settings/encode_mode` + file persist + retained MQTT | [web_ui.py](LifeTrac-v25/DESIGN-CONTROLLER/base_station/web_ui.py) |
| UI | Segmented control on `/settings`, 7 buttons | [web/settings.html](LifeTrac-v25/DESIGN-CONTROLLER/base_station/web/settings.html) |
| Tractor RX | `_clamp_encode_mode` demotes unimplemented modes → Y_ONLY + WARN | [camera_service.py](LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py) |

End-to-end works for **FULL** and **Y_ONLY**; modes 4–7 silently demote.
This plan finishes the picture.

---

## 1. Selected methods (operator-visible)

Order = rotation order for the cycle button. Auto is the "no operator pin"
state; pressing the button from Auto → Full enters operator-pinned mode.

| # | Code | Wire id | Label (UI) | Bytes/tile (typical) | Notes |
| - | ---- | ------- | ---------- | -------------------- | ----- |
| 0 | `auto` | — | Auto | controller picks | clears the operator pin |
| 1 | `full` | 0 FULL | Full color | 26 | baseline WebP RGB |
| 2 | `y_only` | 1 Y_ONLY | Y-only | 19 | greyscale WebP |
| 3 | `btc4_per_tile` | 4 BTC4_PER_TILE | BTC4 / tile | 8 | per-tile 4-level palette + DEFLATE |
| 4 | `btc4_per_frame` | 5 BTC4_PER_FRAME | BTC4 / frame | 5 | per-frame palette + DEFLATE |
| 5 | `mono_g4` | 6 MONO_G4 | Mono 1-bit | 4 | Floyd-Steinberg + DEFLATE of packed bits |
| 6 | `adaptive` | 7 ADAPTIVE | Adaptive | varies | per-frame heuristic picks among 1..5 |

Legacy slots `motion_only` (2) and `wireframe` (3) stay in the wire enum
for backward compatibility but are **excluded** from rotation. They still
work if a CMD_ENCODE_MODE arrives with id=2 or 3.

---

## 2. Wire format choice — DECISION REQUIRED

The current `TileDeltaFrame` header is 5 B (no codec discriminator); the
body is `(size_minus1, blob)*` and every blob is assumed WebP.

### Recommendation: **Option A — per-frame codec byte**
Add 1 B `codec` field after `tile_px`. `HEADER_FIXED_LEN: 5 → 6`. All
tiles in one frame share the codec. Overhead = +1 B/frame total.

Rationale:
- `EncodeMode` is already per-frame (the operator pins one mode that lives
  across many frames). Per-tile granularity buys nothing unless ADAPTIVE
  wants it — and ADAPTIVE can pick a different codec **per frame** based on
  whole-scene stats (which is actually how the research doc §4.10 frames
  the heuristic anyway).
- Smallest decoder change; one switch statement in `parse_tile_delta_frame`
  + one in the tractor encoder.
- Per-tile (Option B) costs +96 B/frame at full canvas — non-trivial when
  the FRAGMENT_BUDGET is 250 B.
- New opcode (Option C) doubles the test surface and migration friction
  for marginal benefit since both ends ship atomically anyway.

### Codec values
| codec | meaning | implemented in this plan |
| ----- | ------- | ------------------------ |
| 0 | WEBP (FULL or Y_ONLY — chroma drop signalled out-of-band by mode) | yes (already) |
| 1 | MONO_G4 (1-bit Floyd-Steinberg, DEFLATE) | yes, step 5 |
| 2 | BTC4_PER_TILE (4-level palette in blob header) | yes, step 6 |
| 3 | BTC4_PER_FRAME (palette in frame header extension) | yes, step 7 |
| 4..15 | reserved | — |

Codec 3 (BTC4_PER_FRAME) needs a tiny extension because the palette is
shared. Either:
- (a) put 12 B palette (4 × RGB) at the start of the body (before the
  first tile), gated by `codec == 3`; or
- (b) make the palette part of the per-tile blob anyway (degrades to
  Option-B-but-only-for-codec-3 territory).

Recommend (a) — keeps the body uniform once we're past the palette.

---

## 3. Cycle-button UX

### 3.1 Rotation logic (shared by web + gamepad)

```
order = ["auto", "full", "y_only", "btc4_per_tile",
        "btc4_per_frame", "mono_g4", "adaptive"]
next  = order[(order.indexOf(current) + 1) % order.length]
POST /api/settings/encode_mode  { mode: next }
```

### 3.2 Web button
- New **floating pill** on the main `/` page (next to existing gamepad
  pill / camera pill), labelled with current mode (e.g. `Y-only ▸`).
- Click rotates one step. Long-press (>800 ms) → opens `/settings`
  directly at the encode-mode section (anchor `#encmode`).
- Settings page keeps the full segmented control for direct jumps.
- Both controls call the same `POST /api/settings/encode_mode` and update
  optimistically from the response.

### 3.3 Gamepad button
Standard Gamepad layout — current free buttons are 4 (LB), 6 (LT), 7
(RT), 8 (BACK/SELECT), 10/11 (stick clicks), 12–15 (D-pad). Picking:

- **BACK / SELECT (button 8)** — primary choice. Single-purpose, rarely
  bumped during driving, paired well with START=E-stop on the same row.
- Rising edge only via existing `pressed(gp, 8)` helper.
- Same `fetch('/api/encode_mode/cycle', {method:'POST'})` as the web pill.

### 3.4 New endpoint: `POST /api/encode_mode/cycle`
Server-side cycle keeps web + gamepad consistent even when a third
client (e.g. another browser tab) changed the pin.

Body: empty.
Returns: `{mode: "<next>", index: N, delivered: bool}` — same shape as
the existing `POST /api/settings/encode_mode` plus the index for UI
state.

Implementation: read current via `_load_encode_mode_override()`, compute
next, call the same persist + publish path as the existing POST handler.
Internally reuse one helper so both POST endpoints stay in lockstep.

### 3.5 Visible state
Pill colour:
- grey "Auto" — no pin, controller has free hand
- green "Full" / "Y-only" / "BTC4 …" — operator pinned
- orange "Mono" — operator pinned to floor (resilience mode)
- yellow "Adaptive" — operator pinned to per-frame heuristic
- **red flash** for 1.5 s when the controller forced an auto-demote
  *below* the operator's ceiling because the link went critical
  (operator can see their pin was "overridden temporarily")

---

## 4. "Signal fails → safe mode" semantics

Already-implemented critical threshold:
[link_monitor.py L202](LifeTrac-v25/DESIGN-CONTROLLER/base_station/link_monitor.py#L202)
uses `image_utilization >= 0.80` (fraction of the 250-B fragment budget
consumed in the rolling 10-s window).

### 4.1 Trigger
`_apply_operator_ceiling(auto_target, utilization)` already:
1. Honours `ADAPTIVE` operator pin until `utilization >= 0.80`, then
   jumps to `MONO_G4` (floor).
2. For non-ADAPTIVE pins, clamps `auto_target` to `max(auto_idx, ceiling_idx)`
   via `ENCODE_MODE_LADDER.index()`.

For this plan we extend the contract:
- When `utilization >= 0.95` (new **safe-mode** threshold) for ≥ 3
  consecutive controller windows, force `MONO_G4` regardless of operator
  pin (even FULL → MONO_G4). Publish a retained MQTT message on
  `lifetrac/v25/control/safe_mode_active` so the UI can render the red
  flash + a banner.
- When `utilization <= 0.60` (new **clear** threshold) for ≥ 5 windows,
  clear safe-mode and re-apply the operator's pin.
- Hysteresis between 0.80 (gentle demote inside ceiling) and 0.95 (hard
  force to floor) prevents flapping.

### 4.2 What about link-loss (no telemetry at all)?
`link_monitor` has a separate `RollingAirtimeLedger`; if no inbound
TileDeltaFrames are decoded for `LINK_TIMEOUT_S` (default 8 s) the bridge
already publishes `lifetrac/v25/state/link = "down"`. New behavior:
when link comes back up, the persisted operator pin is re-published
retained-MQTT (already on `_initial_override` startup hook in
[web_ui.py](LifeTrac-v25/DESIGN-CONTROLLER/base_station/web_ui.py)) so the
controller picks it back up. No code change needed beyond ensuring the
bridge subscriber handles a retained re-publish idempotently (it does).

### 4.3 What about a **stuck** ceiling that prevents recovery?
If the operator pinned FULL and the link drops to 1% PRR, the existing
controller can't promote past MONO_G4. After the link recovers, the
controller's hysteresis (3 windows below threshold) will let it walk back
up the ladder until it hits the ceiling. **Tested by:** unit test
`test_controller_recovers_to_ceiling_after_critical`.

---

## 5. Implementation steps (ordered)

Each step is a single PR-sized commit. Steps 1-3 are wire-format prep;
steps 4-8 are per-codec; step 9 is the cycle button.

### Step 1: Wire format — add `codec` byte
- [frame_format.py](LifeTrac-v25/DESIGN-CONTROLLER/base_station/image_pipeline/frame_format.py):
  - `HEADER_FIXED_LEN = 6`
  - `TileDeltaFrame.codec: int = 0`
  - `parse_tile_delta_frame` reads codec after `tile_px`
  - `encode_tile_delta_frame` writes it
  - Reject `codec > 15` until we use those bits
- [camera_service.py](LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py)
  `_build_frame` header pack:
  - `struct.pack("BBBBBB", frame_kind, base_seq, GRID_W, GRID_H, TILE_PX, codec)`
- New module: `firmware/tractor_x8/image_pipeline/encoder_dispatch.py`
  with `pick_codec(mode: EncodeMode) -> int` mapping table.
- Update `tests/test_frame_format.py`: round-trip with each codec value.
- Update `tests/test_data_saving_measures.py`: bump header length.
- LORA_PROTOCOL.md § TileDeltaFrame: document new field + codec table.

### Step 2: Base-side codec dispatch + transcode-to-WebP
- New module: `base_station/image_pipeline/codec_decode.py`:
  - `decode_tile(codec: int, blob: bytes) -> bytes` → 32×32 RGB raw
  - Fallback to WebP when codec==0
  - Other codecs implemented in steps 5-8
- [canvas.py](LifeTrac-v25/DESIGN-CONTROLLER/base_station/image_pipeline/canvas.py)
  L124 (`slot.blob = tile.blob`):
  - If `frame.codec == 0`: keep raw WebP for browser (current path)
  - Else: decode via `codec_decode.decode_tile`, re-encode as WebP grey,
    set `slot.blob` to the transcoded blob
  - Browser stays unchanged.
- Cache the transcoded blob keyed on `(codec, blob_hash)` so identical
  unchanging tiles don't re-encode each frame.

### Step 3: Operator-ceiling safe-mode hysteresis (link_monitor)
- Add `SAFE_MODE_CRITICAL = 0.95`, `SAFE_MODE_CLEAR = 0.60`
- `EncodeModeController._safe_mode_active: bool` + entered/cleared
  counters
- New method `set_safe_mode_callback(cb)` so the bridge can publish
  `lifetrac/v25/control/safe_mode_active` on transitions.
- Unit tests:
  - `test_safe_mode_engages_at_95_three_windows`
  - `test_safe_mode_clears_at_60_five_windows`
  - `test_safe_mode_overrides_full_pin`
  - `test_operator_ceiling_restored_after_clear`
  - `test_controller_recovers_to_ceiling_after_critical`

### Step 4: Floor verification — MONO_G4 vertical slice
Smallest end-to-end change that proves the wire-format pipe works.
- `firmware/tractor_x8/image_pipeline/encode_mono_g4.py`:
  - `encode(rgb32x32: bytes) -> bytes`:
    1. Floyd-Steinberg dither to 1 bpp (numpy if available, pure-Python
       fallback; PIL has `Image.convert("1")` for the easy path)
    2. Pack bits MSB-first → 128 B
    3. `zlib.compress(level=9)` — typical 30-60 B
- `base_station/image_pipeline/decode_mono_g4.py`:
  - Inverse: zlib decompress → unpack bits → 32×32 grey → ×3 → RGB
- Wire in `_encode_tile`: when `mode == ENCODE_MODE_MONO_G4`, call this
  encoder instead of WebP; frame-level `codec=1`.
- Remove `MONO_G4` from `_ENCODE_MODE_IMPLEMENTED` fallback set.
- Test: `tests/test_encode_mono_g4.py` round-trips and verifies byte
  count stays under TILE_BYTES_MAX=256.

### Step 5: BTC4_PER_TILE (codec=2)
- Encoder: per-tile k-means or median-cut to 4 levels of luma, store
  4 × u8 palette (4 B) + 32×32×2 bpp (256 B / 4 = 256 nibbles = 256 B)
  ... wait, 32×32 = 1024 pixels × 2 bpp = 2048 bits = **256 B** + 4 B
  palette = 260 B. **Over budget.** zlib after packing typically
  shrinks to 60-120 B for natural images. Acceptable.
- Implement in `encode_btc4.py` shared by per-tile and per-frame variants.
- Decoder unpacks palette + indices → grey → RGB.
- Test: round-trip + budget guard.

### Step 6: BTC4_PER_FRAME (codec=3)
- Build one global 4-level palette from the whole canvas using
  histogram-based median-cut (fast: numpy or pure-Python histogram).
- Wire: 4 B palette at start of body (BEFORE first tile's
  `size_minus1`), gated on `codec == 3`. Update `parse_tile_delta_frame`
  to read the palette into `TileDeltaFrame.palette`.
- Per-tile body: just the 2-bpp index plane + zlib (no per-tile palette)
  → ~50-80 B/tile typical.
- Test: ensures `parse` rejects `codec == 3` frames without the palette
  prefix.

### Step 7: ADAPTIVE (codec picked per frame)
- Heuristic in `_build_frame`:
  - Compute mean tile entropy across the changed bitmap
  - If `entropy < T_low`: pick MONO_G4 (cheap, scene is flat)
  - Elif `entropy < T_mid`: pick BTC4_PER_FRAME
  - Else: pick Y_ONLY (preserve detail)
  - Safe-mode override (from §4.1) still applies on top.
- Tuning T_low / T_mid is bench work — initial values from research
  doc §4.10 then iterate.

### Step 8: Cycle endpoint + helper
- [web_ui.py](LifeTrac-v25/DESIGN-CONTROLLER/base_station/web_ui.py):
  - Extract `_set_encode_mode_override(name) -> dict` from the existing
    POST handler.
  - New `POST /api/encode_mode/cycle`:
    ```python
    @app.post("/api/encode_mode/cycle")
    async def api_encode_mode_cycle(_session = Depends(_require_session)):
        cur = _load_encode_mode_override()
        idx = _ENCODE_MODE_UI_CHOICES.index(cur)
        nxt = _ENCODE_MODE_UI_CHOICES[(idx + 1) % len(_ENCODE_MODE_UI_CHOICES)]
        return _set_encode_mode_override(nxt)
    ```
  - Refactor existing POST `/api/settings/encode_mode` to call
    `_set_encode_mode_override(body.mode)` for DRY.

### Step 9: Web cycle pill on main page
- [web/index.html](LifeTrac-v25/DESIGN-CONTROLLER/base_station/web/index.html):
  add `<span id="encode-mode-pill" class="pill grey">Auto ▸</span>`
  near `gamepad-pill`.
- [web/app.js](LifeTrac-v25/DESIGN-CONTROLLER/base_station/web/app.js):
  - On click: `fetch('/api/encode_mode/cycle', {method:'POST'})` → update
    pill label + colour from response
  - Long-press detection (800 ms): `location = '/settings#encmode'`
  - Subscribe via SSE or poll `/api/settings/encode_mode` every 5 s to
    catch external changes (gamepad, other tab, safe-mode override)
- CSS already has `.pill.green/.yellow/.orange/.grey`; add `.pill.red`
  for the safe-mode-active flash.

### Step 10: Gamepad cycle
- [web/app.js](LifeTrac-v25/DESIGN-CONTROLLER/base_station/web/app.js)
  inside `pollGamepad`:
  ```javascript
  if (pressed(gp, 8)) {   // BACK/SELECT → cycle encode mode
      fetch('/api/encode_mode/cycle', {method:'POST'})
        .then(r => r.json())
        .then(d => updateEncodeModePill(d));
  }
  ```
- `pressed()` rising-edge helper already exists.
- README the mapping in the app.js header comment block (lines 5-14).

### Step 11: Safe-mode banner + red flash
- [web/app.js](LifeTrac-v25/DESIGN-CONTROLLER/base_station/web/app.js):
  subscribe (or poll) the retained `lifetrac/v25/control/safe_mode_active`
  via a new GET endpoint or WebSocket. When `true`:
  - Pill goes red, label `SAFE (link)`
  - Top-of-page banner appears: "Link saturated — encoder forced to Mono
    for resilience. Will restore your pin when link recovers."
  - When `false` again: pill returns to operator's actual pin colour.

### Step 12: Bench rotation script
- New: `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/bench_rotate_encoders.ps1`
  - Holds the tractor at each mode for `-DurationS 60` (default)
  - Records per-mode airtime, decode-error rate, perceived FPS, bytes/tile
  - Outputs CSV for direct comparison
- Uses the existing `/api/encode_mode/cycle` so it's the same path the
  operator uses; verifies no admin-only side door is needed.

---

## 6. Test plan

### Unit (`base_station/tests/`)
- `test_frame_format.py` — codec round-trip, oversized codec rejected
- `test_encode_mono_g4.py` — round-trip pixel error bound, byte budget
- `test_encode_btc4.py` — palette correctness, indices in range, budget
- `test_link_monitor.py` — safe-mode hysteresis (5 new cases above)
- `test_web_ui_encode_mode.py` — POST cycle wraps correctly, MQTT publish
  args, file persistence

### Integration (`base_station/tests/integration/`)
- `test_e2e_cycle_via_http.py` — POST cycle 8 times, verify wrap to start
- `test_e2e_codec_dispatch.py` — synthesize TileDeltaFrames with each
  codec, verify canvas.py transcodes to displayable WebP

### Bench (manual; `MASTER_TEST_PROGRAM.md` entries)
- B-21 Encoder rotation airtime test (script from step 12)
- B-22 Operator cycle button latency (web): time from click to first
  tractor-emitted frame in new mode; target <2 s including bridge tick
- B-23 Gamepad cycle button latency: same metric, gamepad path
- B-24 Safe-mode demote latency under synthetic 95% link saturation
- B-25 Safe-mode recovery: drop saturation, measure time to restore pin

---

## 7. Risk register

| Risk | Likelihood | Impact | Mitigation |
| ---- | ---------- | ------ | ---------- |
| Wire-format break corrupts retained MQTT TileDeltaFrames | Medium | High | Bump retained-message version key; consumer drops mismatch. Restart both ends together; safe in pre-prod. |
| New encoder over TILE_BYTES_MAX | Medium | Med | Each encoder must guard + degrade (already pattern in WebP path); test enforces. |
| Safe-mode flapping at threshold | Low | Med | 3-window engage + 5-window clear hysteresis (§3) |
| Gamepad button 8 conflict on non-standard pads | Low | Low | Add settings page setting to remap; default 8 |
| ADAPTIVE picks codec that overshoots fragment budget | Med | Med | Heuristic includes byte-budget guard; falls back one step on overshoot |
| Cycle button double-fires (browser+gamepad on shared user) | Low | Low | `_set_encode_mode_override` is idempotent; pill updates from response |

---

## 8. Open questions (for operator)

1. **§2 wire format** — confirm per-frame codec byte (Option A) vs
   per-tile (Option B) vs new opcode (Option C). I recommend **A**.
2. **§3.3 gamepad button** — confirm BACK/SELECT (button 8), or pick
   another (LB=4, LT=6, RT=7 also free).
3. **§4.1 thresholds** — 0.95 critical / 0.60 clear OK? Or tighter
   (e.g. 0.90 / 0.50)?
4. **§5 step order** — ship MONO_G4 first as floor verification, or BTC4
   first (most useful operator pin)? Recommend MONO_G4 first (proves
   pipe, smallest code).
5. **§6 bench** — accept the 5-test bench plan, or add link-stress
   sweep (PRR vs codec) as B-26?

---

## 9. Estimated commit sequence

(Not estimating hours per implementation-discipline rules; just commit
count and dependency chain.)

```
1 wire format + tests
2 codec decode + canvas transcode + tests
3 safe-mode hysteresis + controller tests
4 MONO_G4 encoder+decoder vertical slice  ◀── PROVES THE PIPE
5 BTC4_PER_TILE
6 BTC4_PER_FRAME
7 ADAPTIVE
8 cycle endpoint refactor + tests
9 web cycle pill
10 gamepad cycle binding
11 safe-mode banner + red flash
12 bench rotation script
```

After step 4 the operator can pin MONO_G4 and actually see 4 B/tile
output on the bench. After step 8 the cycle button works (web+gamepad)
even if only steps 4 is in for new codecs (others still demote).
Steps 5/6/7 add real options to the rotation.

---

## 10. Out of scope (deferred to v1.1+)

- §4.11 post-processing stack (temporal median, bilateral, CLAHE, BTC
  deblock, Reinhard recolor, guided upscale) — separate plan; doesn't
  touch wire format.
- Multi-camera per-channel encoder pinning (i.e. front=FULL,
  implement=MONO).
- Per-tile codec (Option B) — only revisit if ADAPTIVE benchmarks show
  per-frame heuristic underperforms.
- On-tractor codec measurement telemetry (encoder CPU%, encode latency
  histogram). Useful but doesn't block bench rotation.
