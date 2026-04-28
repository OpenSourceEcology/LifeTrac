# LifeTrac v25 — Coral Edge TPU operator guide

**Audience:** the operator running the base station.
**Companion to:** [IMAGE_PIPELINE.md §5.1](IMAGE_PIPELINE.md), [MASTER_PLAN.md §8.19](MASTER_PLAN.md), [HARDWARE_BOM.md Tier 2](HARDWARE_BOM.md).

The Coral Edge TPU on the base-station Portenta X8 is a **strict additive upgrade**: the v25 image pipeline ships and validates CPU-only. If the Coral is present and working, it accelerates super-resolution and the safety-cross-check detector; if it's absent, broken, or you turn it off, the same code paths run on the X8 A53 cores at lower fps. **No safety feature depends on the Coral.**

---

## 1. Quick check

1. Open the base-station UI at `http://<base-ip>:8080/static/settings.html`.
2. Look at the "Status" pill:

| Pill | Meaning | What to do |
|---|---|---|
| **Coral active** (green) | Hardware present, runtime OK, operator switch ON | Nothing — you're getting the upgrade. |
| **present, disabled** (yellow) | Hardware present, operator switch OFF | Tick the "Use Coral when available" box if you want it on. |
| **present, driver missing** (orange) | Hardware visible to the OS but `pycoral` runtime can't open it | See § Driver troubleshooting. |
| **no accelerator** (grey) | No Coral M.2 or USB device detected | Either none is installed, or it's unplugged. Click "Re-scan hardware" after plugging in. |

3. If you've just plugged in or unplugged the Coral, click **Re-scan hardware** to skip the 30 s background poll.

---

## 2. Switching it off

Sometimes you want to force the CPU path:

- The Coral is overheating (no heatsink fitted).
- You want a deterministic A/B comparison of latency.
- You suspect a buggy `libedgetpu` build.

Untick the operator toggle. The change persists across reboots (stored in `/var/lib/lifetrac/base_settings.json`) and takes effect within **one inference call** — both `superres.py` and `detect.py` re-read the operator switch on every frame.

You can also set the boot-time default with the `LIFETRAC_CORAL_ENABLED` env var:

```ini
# /etc/systemd/system/lifetrac-web-ui.service
[Service]
Environment=LIFETRAC_CORAL_ENABLED=false
```

The env var is consulted only the first time the settings file is created; after that the file wins.

---

## 3. Driver troubleshooting (orange pill)

Detection probes in this order (see [`accel_select.py`](base_station/image_pipeline/accel_select.py)):

1. `lspci -nn` for vendor/device `1ac1:089a` (Coral M.2 / Mini-PCIe).
2. `lsusb` for `1a6e:089a` (uninitialised Apex USB) or `18d1:9302` (initialised Google Coral USB).
3. `pycoral.utils.edgetpu.list_edge_tpus()` as a fallback.

**Step 1 — confirm the kernel sees it.**

```bash
# On the base X8 Linux side
lspci -nn | grep -i 1ac1     # M.2 / Mini-PCIe variant
lsusb    | grep -iE '1a6e|18d1'   # USB variant
dmesg | grep -i apex         # Coral M.2 driver loads
```

If `lspci`/`lsusb` show nothing, this is a hardware/cable problem, not a driver one. Re-seat the M.2 card or try a different USB port (the USB Accelerator wants a USB 3 host port; the X8's USB-C is fine).

**Step 2 — confirm the kernel module is loaded** (M.2 only).

```bash
lsmod | grep -E 'apex|gasket'
# If empty:
sudo modprobe apex
```

**Step 3 — confirm the userspace runtime is installed.**

```bash
python3 -c "from pycoral.utils import edgetpu; print(edgetpu.list_edge_tpus())"
```

Expected: a non-empty list. If you get `ImportError: libedgetpu.so.1: cannot open shared object file`, install the runtime:

```bash
sudo apt install libedgetpu1-std python3-pycoral
```

(Or `libedgetpu1-max` for the higher-clock build — runs hotter, needs a heatsink.)

**Step 4 — re-scan from the UI.** Click "Re-scan hardware". The orange pill should turn green.

---

## 4. Hot-unplug behaviour

If you yank the Coral while the base is running:

- Within **30 s** (the background poll period, or immediately if you click "Re-scan"), the pill flips from green to grey.
- All subsequent `superres.enhance(frame)` and `detect.detect(frame)` calls automatically run on CPU.
- The audit log records the transition (`accel_path subsystem=superres path=cpu`).

This is the V5 gate from [IMAGE_PIPELINE.md §5.2](IMAGE_PIPELINE.md): *"yank Coral mid-operation, UI flips to AI accelerator: offline within 10 s, pipeline continues degraded."* The 10 s figure assumes the operator clicks Re-scan; the 30 s background poll is the worst case.

---

## 5. What it actually accelerates (today)

In v25 the dispatchers (`superres.py`, `detect.py`) are **scaffolds** — they pick the Coral vs. CPU path correctly but the underlying impls are stubs that pass the frame through unchanged. The real impls land per [IMAGE_PIPELINE.md §5.1 Phase 2](IMAGE_PIPELINE.md):

- `superres_cpu.py` — Real-ESRGAN-General-x4v3 via ncnn.
- `superres_coral.py` — Edge-TPU port (gated on the Phase-0 spike from [HARDWARE_BOM.md](HARDWARE_BOM.md)).
- `detect_yolo.py` — YOLOv8-nano (CPU and Coral builds), independent safety cross-check (R6).

You can wire your own model in by editing the `_coral_*` / `_cpu_*` functions in those two files; the dispatcher and the operator toggle are already in place.

---

## 6. Future hardening (out of v25 scope)

Per [MASTER_PLAN.md §8.19](MASTER_PLAN.md):

- Per-feature toggles (Coral for super-res only, not for detect, etc.). Today's single master switch is the v25 design.
- Automatic temperature throttle: read the apex driver's thermal sysfs and auto-disable above 85 °C.
- Per-Coral-model performance counters in the audit log.

Track these in `RESEARCH-CONTROLLER/`, not v25.
