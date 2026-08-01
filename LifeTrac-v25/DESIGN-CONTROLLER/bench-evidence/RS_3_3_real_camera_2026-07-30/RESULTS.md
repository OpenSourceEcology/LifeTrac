# RS-3.3 — real camera on air: VERIFIED (2026-07-30)

`camera_service` owns the encode-to-fit budget packer, the carry fix,
age-escalation and the liveness valve. The synthetic bench feed publishes
PRE-BUILT tile-delta frames straight to `image_tx_daemon`, so none of that code
had ever executed on hardware. **It now has.**

## Result

Run `radio_monitor_20260731_213152_9db14d4d`, 240 s, real USB camera through
encode-to-fit, over LoRa (DTS profile 2, v3 pipeline, smooth pacing), to the
base station:

| | |
|---|---|
| camera frames encoded | `frames_in=492 ok=491 fail=0 drop_full=0 drop_stale=0` |
| fragments transmitted | `frags_ok=556` |
| base received | `rx_frames=529 rx_decode_err=0` |
| **complete frames delivered** | **472** |

**Encode-to-fit is confirmed working against the budget.** `camera_service`
logged `byte_budget=2436 B/frame`, and the keyframes it produced measured
**2381, 2389, 2426 and 2430 B — every one under budget, the largest within 6 B
of it.** That is the packer filling the transport quantum without exceeding it,
which is precisely what encode-to-fit exists to do, now demonstrated on real
camera data rather than on synthetic frames built to order.

Steady-state frames are 243 B because the bench scene is static — tile-delta
correctly sends almost nothing when nothing moves. Goodput 368 B/s at 15%
utilisation reflects that, and is not a defect.

## CORRECTION: the earlier "three defects" were largely one misconfiguration

An earlier revision of this file reported three independent product defects.
**That was wrong and is retracted.** All three collapse to a single missing
environment flag: `LIFETRAC_USE_LORA_BRIDGE=1`.

`camera_service` branches on that flag three separate ways
(`camera_service.py:1354`, `:1446`, `:240`):

1. **Skips the M7 `IpcWriter`** — which otherwise imports
   `image_pipeline.ipc_to_h747` *and* opens `/dev/ttymxc3`, a device
   `image_tx_daemon` already owns on this path. My "ModuleNotFoundError" defect
   was this import, and with the flag set the import never happens.
2. **Creates the MQTT client.** Without it `client is None` and the publish at
   `:1563` is silently skipped — frames are captured and thrown away with no log
   line at all. That is what made the service look hung.
3. **Turns deshake off.** The default really is unusable on this hardware (the
   i.MX8 produced 0 bytes in 30 s through `deshake=open2=1:search=16`, against
   589,824 B = 2 frames exactly without it) — but the default is *correct*,
   because it is off whenever the LoRa bridge owns the frames.

Omitting one flag made a working service look broken in three unrelated ways at
once. The gate is coherent: it declares which transport owns the frames, and
everything else follows.

**What survives as a genuine finding:** the harness's `-TxFeed camera` mode did
not start the bench broker (mine, introduced with the branch, fixed), and
deploying only `base_station/image_pipeline` is still wrong for the tractor —
harmless once the flag is set, but it would bite any non-bridge path.

## Also corrected: the "camera_service is blocked" diagnosis

An earlier revision reported `camera_service` blocked in `_read_exact`, based on
a `faulthandler` stack dump. **Also wrong.** At 2 fps the process legitimately
spends nearly all its time waiting in `_read_exact` for the next frame, so
catching it there proves nothing. Tracing added under `LIFETRAC_CAMERA_TRACE=1`
showed complete 294,912-byte frames arriving every ~0.25 s the whole time. It was
capturing correctly and discarding the result for want of an MQTT client.

The elimination work around it was still sound and is worth keeping: ffmpeg, the
real argv (including `-probesize 32`, `-thread_queue_size 2`, `-fflags
nobuffer`), the pipe, `stdin=DEVNULL` and device contention were each ruled out
by measurement. They were simply all innocent.

## Instrumentation left behind

`LIFETRAC_CAMERA_TRACE=1` logs one line per `read()` with elapsed time and the
spawn-to-first-read gap. Off by default. It is what turned "appears hung" into
"capturing fine, discarding output" in one run.

## Method note

Two wrong diagnoses in one investigation, both from reasoning about a system
instead of instrumenting it. The stack dump was over-read (a thread parked where
it belongs), and three symptoms were attributed to three causes when a single
config gate explained all of them. The trace flag settled it in one run and cost
less than any of the guesses.
