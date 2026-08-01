# RS-3.3 — first attempt to run the real camera on air (2026-07-30)

`camera_service` owns the encode-to-fit budget packer, the carry fix,
age-escalation and the liveness valve. The synthetic bench feed publishes
PRE-BUILT tile-delta frames straight to `image_tx_daemon`, so none of that code
had ever executed on hardware. This is the first attempt to run it.

**Outcome: not yet delivering frames.** Three real defects were found and fixed
along the way; a fourth is open. Every one of them was invisible to the
synthetic path.

---

## Confirmed working

- **The camera is attached and functional.** A UVC 1.00 device "C2"
  (`16d0:0ed4`) on `/dev/video1` — `dmesg` shows
  `uvcvideo: Found UVC 1.00 device C2`. `/dev/video0` is the separate built-in
  `mx6s-csi` MIPI bridge, NOT the USB camera; pointing at it would silently
  capture nothing.
- **Capture works end to end when invoked correctly.** Verified by hand:
  `ffmpeg -f v4l2 -input_format mjpeg -video_size 1920x1080 -framerate 30
  -i /dev/video1 -vf fps=2,scale=384:256 -pix_fmt rgb24 -f rawvideo`
  produced **589,824 bytes for 2 frames — exactly 2 x 384 x 256 x 3.**
- Supported formats: `mjpeg` and `yuyv422`, both up to 1920x1080.

## Defect 1 — the `image_pipeline` package collision breaks PRODUCTION, not just tests

`camera_service` died immediately on:

    ModuleNotFoundError: No module named 'image_pipeline.ipc_to_h747'

This is RS-5.7's two-packages-one-name problem, and it had been filed as *test*
debt. It is not: it stops the tractor's camera service from starting at all. The
harness deployed only `base_station/image_pipeline`, while `camera_service`
needs the tractor's.

They merge safely — of 16 + 11 modules the **only** overlapping filename is
`__init__.py`, and both are docstring-only. The tractor genuinely needs both
copies: `image_tx_daemon` imports `image_pipeline.frame_format` from the base
station's, `camera_service` imports `ipc_to_h747` / `tile_cache` from its own.
Fixed by deploying both.

## Defect 2 — the camera feed never started the local broker

`-TxFeed camera` (added here) skipped the bench-broker startup that `-TxFeed
local` performs, leaving the TX daemon pointed at `$HostIp` with nothing
listening:

    image_tx_daemon: MQTT connect to 192.168.1.79:1883 failed: [Errno 101] Network unreachable

Mine, introduced with the camera branch. Fixed by treating `camera` as a
local-broker feed.

## Defect 3 — camera_service's DEFAULT config cannot capture a single frame

`_default_deshake = "0" if USE_LORA_BRIDGE else "1"`, and `USE_LORA_BRIDGE`
defaults false — so on the LoRa path **deshake defaults ON**, adding
`deshake=open2=1:search=16` to a 1920x1080@30 pipeline.

Measured on the i.MX8, identical pipelines, 30 s budget each:

| filter chain | output |
|---|---:|
| `fps=2,scale=384:256` | **589,824 bytes** (2 frames, exact) |
| `fps=2,scale=384:256,deshake=open2=1:search=16` | **0 bytes** |

The board cannot produce one frame through deshake in 30 seconds. Left at its
default, `camera_service` restarts ffmpeg forever and delivers nothing —
`grab_rgb` raises "ffmpeg refused to produce a frame after restart" on a loop.

The harness now sets `LIFETRAC_CAMERA_DESHAKE=0` explicitly rather than setting
`LIFETRAC_USE_LORA_BRIDGE=1`, which would also flip the image method A->C and
confound the encode-to-fit measurement.

**This deserves a decision, not just a bench override:** the shipped default is
non-functional on the actual tractor hardware. Either the gate is wrong (deshake
should be off unless explicitly enabled) or the tractor is expected to run with
`USE_LORA_BRIDGE=1` — in which case the LoRa bench path should set it.

## Open — camera_service starts, then publishes nothing

With all three fixed, `camera_svc` stays **Up** but emits exactly **one** log
line (the ffmpeg startup) and zero publish/encode activity. `image_tx_daemon`
reports `frames_in=0`, so nothing reaches the broker.

Alive but stuck after spawning ffmpeg. Next steps, in order:
1. Run `camera_service` in the foreground with `--log-level DEBUG` to see where
   it blocks — most likely inside `_read_exact` waiting for a full 294,912-byte
   rgb24 frame, or on the MQTT connect.
2. Confirm the broker is actually reachable from inside the camera container
   (`--network=host` should make 127.0.0.1 work, but this is untested for this
   container specifically).
3. Check whether the `fps=2` filter interacts badly with the read loop — the
   hand-verified capture used the same filter and worked, so the difference is
   in camera_service's consumption, not the pipeline.

## Why this matters

Four independent defects sat in the one code path that had never been exercised
on hardware, and the synthetic harness could not have surfaced any of them —
it bypasses `camera_service` entirely. The encode-to-fit logic *still* has not
executed on air. Everything RS-3.3 was meant to verify remains unverified.
