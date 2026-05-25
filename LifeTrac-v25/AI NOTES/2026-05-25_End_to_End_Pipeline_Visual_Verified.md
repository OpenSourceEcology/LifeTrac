# 2026-05-25 — End-to-end LoRa video pipeline visually verified in browser

## Headline

The full LifeTrac v25 video tile pipeline rendered live frames in the
operator browser console for the first time:

  publish_synthetic_frames.py        (host)
    -> MQTT lifetrac/v25/cmd/image_frame  (amqtt broker @ 192.168.1.79:1883)
       -> image_tx_daemon              (TX board 2E2C, docker)
          -> LoRa air                  (SF7/BW250/CR4-5, FRF 915 MHz, BENCH profile)
             -> image_rx_daemon         (RX board 2D0A, docker)
                -> MQTT lifetrac/v25/video/tile_delta
                   -> web_ui (uvicorn @ 192.168.1.79:8080)
                      -> /ws/state WebSocket -> browser
                         -> canvas detected "SYNTHETIC TEST PATTERN" banner

Latest numbers (30 s window):
- RX: `rx_frames=199 rx_decode_err=0 frames_published=14 publish_err=0`
- TX: `frames_in=36 ok=6 fail=24 drop_full=2 frags_ok=145 frags_fail=65`
- Browser: green "SYNTHETIC TEST PATTERN DETECTED" banner = canvas pixels OK.

## Two additional cascading harness bugs surfaced and fixed this turn

After last turn's "smoke green" milestone, restarting the smoke from
inside the helper directory broke it again. Two issues:

1. **Publisher path was relative.** `run_concurrent_smoke.ps1` used
   `-ArgumentList "publish_synthetic_frames.py"` for `Start-Process`. When
   PowerShell launches the smoke with CWD != workspace root, python
   gets a relative path that resolves under the helper dir and fails with
   `[Errno 2] No such file or directory`. Publisher dies silently;
   `frames_in=0` is reported on the TX side, looking identical to a
   firmware fault. Fix: resolve the publisher script under `$repoRoot`
   and pass it as an absolute, quoted argument, plus
   `-WorkingDirectory $repoRoot`.

2. **`web_ui.py` was never running.** The browser tab was loaded from a
   prior session and kept retrying `/ws/state` against a port nobody was
   listening on (ERR_CONNECTION_REFUSED every 7-10 s for 35+ minutes).
   The HTML had survived in the browser cache, but the FastAPI server
   process was gone. Fix: start it explicitly with
   `uvicorn web_ui:app --host 0.0.0.0 --port 8080` from the
   `base_station/` directory. Browser auto-reconnected and the canvas
   populated within one frame.

Both issues continue the "harness deployment trap" pattern documented in
the 2026-05-25 smoke-green note: a `frames_in=0` / `rx_frames=0` /
`ERR_CONNECTION_REFUSED` symptom is identical to a real firmware/RF
fault. The status-fields-hide-cause rule fires here too — neither
PowerShell nor the daemon stats reported the underlying ENOENT or the
missing UI process; they just zeroed out the counter.

## What was NOT touched

- Firmware unchanged.
- LoRa stack unchanged.
- ERR_PROTO FORBIDDEN (`1001080000`) drops still occur at ~10% of frame
  bursts. Decoded: TX_FRAME_REQ rejected by `sx1276_tx_begin()` with
  `HOST_ERR_PROTO_FORBIDDEN` and detail=0. Root cause analysis from
  `firmware/murata_l072/radio/sx1276_tx.c` line 158: when
  `s_tx_state != SX1276_TX_STATE_IDLE` at begin() entry, FORBIDDEN is
  returned. The earlier `sx1276_tx_busy()` gate in
  `host_cmd.c:449` returns `HOST_ERR_PROTO_QUEUE_FULL` instead — so
  FORBIDDEN means the busy()→begin() race window caught a TX that had
  just been signalled DONE but had not yet returned to IDLE in its
  cleanup. Not blocking the pipeline. Future: extend the daemon's
  retry-on-FORBIDDEN to wait ~1 ms before retry (the cleanup window).
- 0xC0/0xC3 "unrelated frame during TX wait" spam is still present in
  `method_h_stage2_tx_probe_v2.wait_for_tx_done()`. Not blocking;
  documented as low-priority.
- `reassembler_timeouts` 4→30 growth not investigated.

## Open follow-ups

- Auto-start broker, web_ui, and publisher from a single
  `run_full_stack_smoke.ps1` so the manual-orchestration trap closes.
- Verdict logic in `run_concurrent_smoke.ps1` should distinguish:
  - "publisher never ran"     (publisher_err.log non-empty, ENOENT)
  - "TX never received frames"  (frames_in=0 with no publisher error)
  - "RX never demodulated"      (rx_frames=0 with TX ok>0)
- Add `frames_published` to the SUCCESS line so the smoke can fail when
  RX decoded LoRa packets but the reassembler/MQTT chain dropped them
  all.
- Investigate publisher's hardcoded default `LIFETRAC_MQTT_HOST=
  192.168.1.117` (RX board IP). It overrides correctly via env, but the
  default is misleading and should change to localhost or be removed.

## Repro

```powershell
# 1. Start broker (Terminal 1)
$py = "C:\Users\dorkm\AppData\Local\Python\pythoncore-3.14-64\python.exe"
& $py .\run_amqtt_broker.py

# 2. Start web_ui  (Terminal 2)
$env:LIFETRAC_MQTT_HOST = "192.168.1.79"
cd LifeTrac-v25\DESIGN-CONTROLLER\base_station
& "$py" -m uvicorn web_ui:app --host 0.0.0.0 --port 8080

# 3. Run smoke from workspace root (Terminal 3)
cd C:\Users\dorkm\Documents\GitHub\LifeTrac
powershell -NoProfile -File LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper\run_concurrent_smoke.ps1

# 4. Open http://192.168.1.79:8080/ in a browser - the synthetic test
#    pattern banner should appear within 5 seconds of the first
#    frames_published log line.
```

## Lessons (additions to misdiagnosis.md candidates)

1. **Browser shows the UI != UI server is running.** A WebSocket
   `ERR_CONNECTION_REFUSED` loop in the console means the HTML was
   served-and-cached but the live socket isn't. Always check
   `Get-NetTCPConnection -LocalPort <port> -State Listen` before
   blaming the data pipeline for a stale-looking canvas.
2. **PowerShell `Start-Process` with a relative path is CWD-dependent.**
   `-WorkingDirectory` is mandatory for any script that may be invoked
   from a sibling/child directory. Always resolve under `$PSScriptRoot`
   or `$repoRoot` and pass an absolute path.
