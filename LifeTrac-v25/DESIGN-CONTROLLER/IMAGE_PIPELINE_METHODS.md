# Image Pipeline — Method letter tracker

Parallel to [`DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md`](DESIGN-LORAFIRMWARE/00_DECISION_Method_G_Commitment.md): a rolling lettered version index for the **tile-selection / priority-ordering** policy inside
[`firmware/tractor_x8/camera_service.py`](firmware/tractor_x8/camera_service.py)`._build_frame`.

> The wire format ([`LORA_PROTOCOL.md` § TileDeltaFrame](LORA_PROTOCOL.md)) and the canonical plan ([`IMAGE_PIPELINE.md`](IMAGE_PIPELINE.md)) are unchanged across these methods. Letters here only track **which-tile-to-ship-this-frame** policy revisions.

Active selector: `LIFETRAC_IMAGE_METHOD` env var (`A` / `B` / `C`).
Default: **`A`** (no behaviour change on existing deployments).

| Letter | Status | Bitmap policy (P-frame) | Priority order |
|:---:|---|---|---|
| **A** | Shipping (default) | "any byte differs" between current & previous canvas | `(rank, idx)` row-major |
| **B** | Experiment | L1 magnitude > `LIFETRAC_TILE_MAGNITUDE_MIN` (default 8000) | `(rank, -magnitude, idx)` most-changed first |
| **C** | Experiment | Method B + `LIFETRAC_SWEEP_STEP` longest-unsent tiles forced in | `(rank, -magnitude, idx)` |

## Why this exists

The [2026-05-25 live-camera pipeline run](../AI%20NOTES/2026-05-25_Live_Camera_Pipeline.md#L67-L84) reproduced the well-known "only the top strip paints" symptom and traced it to two compounding choices in `_build_frame`:

1. The bitmap uses an *any-byte-differs* check, so sensor noise marks ~100% of tiles changed every frame and the byte budget never has a real signal to prioritise on.
2. Priority order is `(rank, idx)` row-major, so when `LIFETRAC_FRAGMENT_BUDGET ≈ 250` only fits 1–2 tiles per P-frame, the lowest-index tiles always win and the rest of the canvas is starved.

Method **B** is the minimal, magnitude-aware fix. Method **C** adds a rotating fair-share sweep so a still scene still converges to full coverage (~96 / `SWEEP_STEP` P-frames at full coverage with zero motion).

## Method A — shipping baseline

**Code path:** the `IMAGE_METHOD == "A"` branch in `_build_frame`.

**Pros:**
- Byte-for-byte stable on the existing W2-03/04/05 tests.
- No NumPy dependency required.

**Cons:**
- Top-left starvation under any tight `LIFETRAC_FRAGMENT_BUDGET`.
- Sensor noise pins the bitmap to ~100% changed every frame, so the byte budget has no signal to prioritise on — keyframe ↔ P-frame distinction collapses on real video.

## Method B — magnitude-ranked priority

**Activation:** `LIFETRAC_IMAGE_METHOD=B`.

**Bitmap policy.** Per tile, compute `L1 = Σ |cur_rgb − prev_rgb|` over the 32×32×3 byte block (vectorised with NumPy reshape — same shape the legacy any-byte-differs branch already uses). A tile is considered changed iff `L1 > LIFETRAC_TILE_MAGNITUDE_MIN` (default 8000). Floor chosen as ≈ 2.5 levels × 3072 bytes; lower flips through into noise, higher misses real motion of a single small object.

**Priority order.** Sort `(rank, -magnitude, idx)`. Inside-ROI tiles still come before outside-ROI tiles; within a rank the highest-magnitude tile is encoded first. Under a tight `LIFETRAC_FRAGMENT_BUDGET` this means the bytes get spent where the motion is.

**Determinism.** `idx` is the final tie-breaker so two equal-magnitude tiles always order the same; tests can pin kept-tile lists without flake.

**Falls back to A** when NumPy is not available or when this is the very first frame (no previous canvas to diff against).

## Method C — Method B + rotating fair-share sweep

**Activation:** `LIFETRAC_IMAGE_METHOD=C`.

**P-frame extra step.** After the magnitude-based bitmap is built, look up the `SWEEP_STEP` (default 2) tile indices with the smallest `tile_last_seq` value (i.e. longest-unsent) that are *not* already changed, and force them into the changed bitmap. Their magnitude stays at 0 so they sort to the tail of the priority list — they only ship if the byte budget has room after the real motion has been accounted for.

**Keyframe rotation.** When a tight byte budget would clip an I-frame to a row-major strip (the user-reported "startup never paints the whole canvas" symptom), Method C reorders the keyframe by `tile_last_seq` ascending — so each successive keyframe rotates through the canvas covering whichever tiles are most overdue. After ~24 keyframes at 10 s cadence with `budget=250` (4 tiles per keyframe), the full 96-tile canvas is guaranteed to have shipped at least once even with zero motion.

`tile_last_seq` is a 96-entry per-tile counter set to `accum.sweep_seq` every time a tile is actually shipped. It is updated under every method so that a runtime switch from A → C immediately has useful data, no warm-up required.

**Convergence.** With `SWEEP_STEP=2` and a 1 fps loop, a static 96-tile canvas reaches full coverage in ≈ 48 s with zero motion. The keyframe rotation acts as an additional, slower convergence path that catches up tiles the P-frame sweep missed (e.g. tiles in regions with constant low-amplitude noise that aren't above `TILE_MAGNITUDE_MIN` but aren't stale either). Tune `SWEEP_STEP` up for faster fill (at the cost of less budget for real motion); tune down for the opposite.

## How to switch methods at runtime

```powershell
adb -s 2E2C1209DABC240B shell "echo fio | sudo -S systemctl set-environment LIFETRAC_IMAGE_METHOD=B"
adb -s 2E2C1209DABC240B shell "echo fio | sudo -S systemctl restart camera-service"
```

Or pass through `run_camera_pipeline.ps1`:

```powershell
$env:LIFETRAC_IMAGE_METHOD = "C"
$env:LIFETRAC_SWEEP_STEP   = "3"
.\run_camera_pipeline.ps1 -Cycles 60
```

## Bench-validating without hardware

```powershell
python LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/method_compare.py
```

Runs all three methods over the same synthetic canvas sequence under a tight 250 B budget and prints the kept-tile coverage map. Use this to sanity-check a method change before flashing the X8.

## Promotion criteria

A method graduates from "experiment" to "shipping" when:

1. `python method_compare.py` shows ≥ 4× the tile-coverage area of Method A under the production budget.
2. A 60 s `run_camera_pipeline.ps1` run on the real Kurokesu C2 shows the same.
3. The W2-03/04/05 cache + budget tests still pass under the new default (the cache key already includes encode mode + quality, so it should be unaffected — verify regardless).

When promoted, flip the `IMAGE_METHOD` default in `camera_service.py` and update the "Status" column above. Older methods stay available behind the env flag so a regression can be A/B'd in the field.
