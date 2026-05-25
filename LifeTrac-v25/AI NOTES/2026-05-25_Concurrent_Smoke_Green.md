# 2026-05-25 - Concurrent Smoke Green: rx_frames=201, frames_published=14

## TL;DR

The `run_concurrent_smoke.ps1` end-to-end smoke is now **GREEN**.

- TX board (2E2C) `image_tx_daemon`: `frames_in=37 ok=6 frags_ok=146`
- RX board (2D0A) `image_rx_daemon`: `rx_frames=201 frames_published=14 publish_err=0`
- Zero CRC errors, zero decode errors, zero MQTT publish errors.

The "rx_frames=0" baseline that motivated weeks of IQ / FHSS-header / RX_DROP_URC speculation was caused entirely by **harness deployment + host infrastructure gaps**, not by firmware or RF behaviour.

## What was actually wrong (in order of discovery and fix)

1. **TX daemon not deployed.** `image_tx_daemon.py` was missing from `/tmp/lifetrac_strict/` on 2E2C; the smoke script assumed it was pre-staged. TX container died on file-open every run. Fix: smoke now `adb push`es daemons + `lora_proto.py` + `image_pipeline/` before launch.
2. **`paho` missing on TX.** Only the RX board had `paho/` under `/tmp/lifetrac_strict/`. Fix: pulled `paho/` from RX (`adb pull /tmp/lifetrac_strict/paho .\_paho_pull\`) and the smoke now unconditionally pushes it to both boards.
3. **No MQTT broker on host.** Both daemons returned 2 on `connect timed out`. Fix: stood up a local `amqtt` broker via `run_amqtt_broker.py` bound to `0.0.0.0:1883`. Host IP `192.168.1.79` is the broker.
4. **Synthetic publisher pointed at the wrong broker.** `publish_synthetic_frames.py` defaulted to `192.168.1.117` (the RX board itself). Fix: smoke now sets `$env:LIFETRAC_MQTT_HOST = $HostIp` before launching the publisher.
5. **PowerShell `$1` regex placeholder bug** (cosmetic). Replaced with `$($matches[1])` so the SUCCESS banner prints cleanly without "variable not set" noise.

## Pre-existing falsified hypotheses (closed)

- ❌ IQ polarity flip — falsified by `run_rx_pair_nrst.ps1` showing 30/30 RX with default IQ.
- ❌ FHSS schema-ver header silent-drop — both peers share `SCHEMA_VER=1`; drop path never triggers.
- ❌ RX_DROP_URC instrumentation requirement — not needed; problem was deployment-layer.
- ❌ Sniff/listen race in `air_coupling_rssi_sniff.py` — orthogonal; still a real bug but unrelated to rx_frames=0.

## Reproduce

```powershell
# 1. Start the local MQTT broker (keep running)
& "C:\Users\dorkm\AppData\Local\Python\pythoncore-3.14-64\python.exe" run_amqtt_broker.py

# 2. In another shell, run the smoke
powershell -NoProfile -ExecutionPolicy Bypass -File `
  LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper\run_concurrent_smoke.ps1
```

Expect: `SUCCESS: Demodulated frame(s) seen by RX daemon! Match group: 201` (or similar non-zero count).

## Lessons (added to user memory / repo memory)

- **Harness deployment trap**: a test script that ASSUMES files are pre-staged on remote devices will mimic a firmware failure exactly. Always make harnesses deployment-hermetic.
- **Status field == 0 hides root cause**: 3rd recurrence in this codebase of "stage_N=0 led to false upstream diagnosis". Always cross-check raw stage logs before pivoting.
- **Run a hermetic reference test on identical hardware before blaming firmware/RF**: `method_h_stage2_tx_probe_v2.py --probe rx_listen`/`--probe tx_burst` is the canonical reference (30/30 frames).
- **Host-side infra is part of the test surface**: missing MQTT broker, wrong default broker IP in helper script, and missing Python package on remote all looked like "no frames demodulated" until the deploy + infra path was uniform.

## Files changed this session

- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_concurrent_smoke.ps1` — deployment + env + cosmetic fix.
- `run_amqtt_broker.py` (new, workspace root) — minimal local MQTT broker for smoke.
- `amqtt_broker_config.yaml` (new, workspace root) — broker config (currently unused; Python script holds config inline).
- `_paho_pull/paho/` (new, workspace root) — paho-mqtt pulled from RX board for re-pushing to TX.
- `LifeTrac-v25/AI NOTES/2026-05-25_Smoke_RxFrames_Was_Missing_TX_Daemon.md` — earlier root-cause note (still accurate).
- `LifeTrac-v25/AI NOTES/2026-05-25_Concurrent_Smoke_Green.md` — this note.

## Open items (deferred)

- MQTT broker is a Windows-host one-shot Python process; future work could bundle it into the smoke as an auto-start.
- TX daemon shows `ERR_PROTO during TX wait: 1001080000` occasionally (frame seq=48 lost 3/4 fragments). Worth investigating separately — likely a queue-depth or schedule edge case, not RF.
- `reassembler_timeouts` grew from 4 → 30 during the 30 s run. RX side reassembler is missing some fragments — separate tile-delta reliability question.
