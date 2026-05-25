# Strict-Path Image-over-LoRa Daemons — Phase B + C Implementation Plan & Runbook

**Date:** 2026-05-24  
**Author:** GitHub Copilot (Claude Opus 4.7)  
**Status:** Code written, **NOT YET DEPLOYED**. Awaits user review before hardware run.  
**Companion docs:**
- `2026-05-24_Camera_Fallback_Recovery_Runbook_v1_0.md` (architectural root cause)
- `2026-05-18_W2-02_Image_Over_LoRa_Bench_Plan_Copilot_v1_0.md` (proven one-shot precedent)

---

## Why this exists

The current live-video path is the **fallback path**: camera_service → local MQTT → ADB-tunneled to Windows host → web_ui. **Images do not touch LoRa today.**

The *designed* path requires a base-side Linux host that can sink topic id 0x25 (`lifetrac/v25/video/tile_delta`). On the 2026-05-24 bench we have two Portenta X8 boards that can serve as tractor + base, so the strict path is physically buildable.

The W2-02 one-shot probe (`w2_02_host_pipeline.py` + `run_w2_02_image_over_lora_end_to_end_v2.ps1`) already proves the full RF chain works on **this exact board pair**: camera → tile → WebP → fragment → L072 TX_FRAME_REQ → SX1276 air → L072 RX_FRAME_URC → reassemble → PNG. The missing piece was always: **make the loop continuous**.

This deliverable extracts the long-running daemons.

---

## New files

| Path | Role | Board | ~LOC |
|---|---|---|---|
| `LifeTrac-v25/DESIGN-CONTROLLER/firmware/tractor_x8/image_tx_daemon.py` | MQTT `cmd/image_frame` → fragment → L072 TX_FRAME_REQ | Tractor X8 (`2E2C1209DABC240B`) | ~340 |
| `LifeTrac-v25/DESIGN-CONTROLLER/base_station/image_rx_daemon.py` | L072 RX_FRAME_URC → reassemble → MQTT `video/tile_delta` | Base X8 (`2D0A1209DABC240B`) | ~290 |

Both daemons reuse existing primitives — no new wire format, no new dependencies:
- TX daemon uses `lora_proto.pack_telemetry_fragments` + `PHY_IMAGE` (SF7/BW500) and `method_h_stage2_tx_probe_v2.HostLink` (same class that drives W2-02).
- RX daemon uses `image_pipeline.reassemble.FragmentReassembler` (v1 0xFE + v2 0xFD magics) and the same HostLink class.

---

## Wire format (recap)

Same as W2-02 bench probe:

```
[L072 TX_FRAME_REQ body]
  u8 tx_id
  u8 len
  u8 body[len]   ←  this is a TELEMETRY_FRAGMENT (4-byte 0xFE header + payload chunk)

[L072 RX_FRAME_URC body]
  u8 rx_len
  i8 snr_db
  i16 rssi_dbm_le
  u32 timestamp_us_le
  u8 payload[rx_len]   ←  the same TELEMETRY_FRAGMENT, recovered intact

[Reassembled output → MQTT topic lifetrac/v25/video/tile_delta]
  encode_tile_delta_frame(TileDeltaFrame)   ←  identical to what camera_service publishes locally
```

The RX daemon's MQTT publish is byte-for-byte the format `web_ui.py /ws/state` already consumes, so **no UI changes needed**.

---

## Bridge-bypass: explicit, intentional, FIXME

This implementation **bypasses `lora_bridge.py`** on the RX side and **does not use `TelemetryFrame` envelopes** on the TX side. Consequences:

- No AES-GCM encryption on-air
- No nonce / replay protection
- No audit-log integration
- No topic-ID routing (the daemons only handle image traffic — control/telemetry still flows through the existing bridge on its own L072 channel… which on a single-X8 deployment is a conflict; see Phase E below)

This is **identical to W2-02 simplifications**. The reasoning is to keep the strict-path migration small and reviewable; harden in Phase E (extend `lora_bridge.py` to subscribe `cmd/image_frame` and wrap properly).

---

## Phase A — Verification (DONE)

```
adb devices
  2D0A1209DABC240B  device   ← base candidate
  2E2C1209DABC240B  device   ← tractor (camera)

adb -s 2D0A1209DABC240B shell ls /dev/ttymxc3
  /dev/ttymxc3                              ← present

adb -s 2D0A1209DABC240B shell uname -a
  Linux portenta-x8-2d0a1209dabc240b 6.1.24-lmp-standard …
```

W1-10b RX firmware validated on this same 2D0A board within the last two weeks (see `2026-05-12_W1-10_RX_Validation_Plan_Copilot_v1_0.md`), so no re-flash is required.

---

## Phase B — TX daemon (CODE WRITTEN, not yet deployed)

### Deployment

```powershell
# 1. Verify camera container is running (fallback stays live).
adb -s 2E2C1209DABC240B shell docker ps | sls camera

# 2. Push daemon and its dependencies to /tmp/lifetrac_p0c/ (same dir
#    pattern as w2_02 orchestrator).
$tractor = "2E2C1209DABC240B"
adb -s $tractor shell mkdir -p /tmp/lifetrac_strict
adb -s $tractor push LifeTrac-v25\DESIGN-CONTROLLER\firmware\tractor_x8\image_tx_daemon.py /tmp/lifetrac_strict/
adb -s $tractor push LifeTrac-v25\DESIGN-CONTROLLER\base_station\lora_proto.py /tmp/lifetrac_strict/
adb -s $tractor push LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper\method_h_stage2_tx_probe_v2.py /tmp/lifetrac_strict/
adb -s $tractor push LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper\method_g_stage1_probe.py /tmp/lifetrac_strict/

# 3. Run daemon (foreground, sudo for /dev/ttymxc3). Camera container's
#    paho-mqtt is reachable via host pip; verify with `pip3 show paho-mqtt`
#    inside the container.
adb -s $tractor shell "cd /tmp/lifetrac_strict && echo fio | sudo -S python3 -u image_tx_daemon.py --log-level INFO"
```

### Validation

Expected log output within 10 seconds of camera publishing:

```
INFO image_tx_daemon: opening L072 HostLink on /dev/ttymxc3 @ 921600
INFO image_tx_daemon: LBT_ENABLE=0 (matches W1-10b TX_BURST rationale)
INFO image_tx_daemon: MQTT connected; subscribing to lifetrac/v25/cmd/image_frame
INFO image_tx_daemon: TX worker ready (inter_cycle_s=0.050, max 8 frags/dwell)
INFO image_tx_daemon: image_tx_daemon started; mqtt=127.0.0.1:1883 uart=/dev/ttymxc3
INFO image_tx_daemon: frame seq=1 done: 5/5 fragments ok (0 fail)
INFO image_tx_daemon: frame seq=2 done: 5/5 fragments ok (0 fail)
INFO image_tx_daemon: stats: frames_in=12 ok=11 fail=1 drop_full=0 frags_ok=54 frags_fail=2 qdepth=0
```

Validation gate: **15 s of `frames_in > 0` with `ok/in > 0.9` ratio**. Bring up the RX daemon next.

---

## Phase C — RX daemon (CODE WRITTEN, not yet deployed)

### Deployment

```powershell
$base = "2D0A1209DABC240B"

# 1. Ensure adb reverse is wired so X8 can reach Windows host mosquitto.
adb -s $base reverse tcp:1883 tcp:1883

# 2. Push daemon + dependencies.
adb -s $base shell mkdir -p /tmp/lifetrac_strict
adb -s $base push LifeTrac-v25\DESIGN-CONTROLLER\base_station\image_rx_daemon.py /tmp/lifetrac_strict/
adb -s $base push LifeTrac-v25\DESIGN-CONTROLLER\base_station\lora_proto.py /tmp/lifetrac_strict/
adb -s $base push LifeTrac-v25\DESIGN-CONTROLLER\base_station\image_pipeline /tmp/lifetrac_strict/image_pipeline
adb -s $base push LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper\method_h_stage2_tx_probe_v2.py /tmp/lifetrac_strict/
adb -s $base push LifeTrac-v25\DESIGN-CONTROLLER\firmware\x8_lora_bootloader_helper\method_g_stage1_probe.py /tmp/lifetrac_strict/

# 3. Verify paho-mqtt is available on the base X8 (may need install if no
#    camera container was ever run on this board).
adb -s $base shell "python3 -c 'import paho.mqtt.client; print(paho.mqtt.client.__file__)'"
# If missing:
# adb -s $base shell "echo fio | sudo -S pip3 install paho-mqtt"

# 4. Run daemon.
adb -s $base shell "cd /tmp/lifetrac_strict && echo fio | sudo -S python3 -u image_rx_daemon.py --log-level INFO"
```

### Validation

Expected log within 5 s of TX daemon transmitting:

```
INFO image_rx_daemon: opening L072 HostLink on /dev/ttymxc3 @ 921600
INFO image_rx_daemon: L072 VER warm-up ok
INFO image_rx_daemon: MQTT connected; ready to publish lifetrac/v25/video/tile_delta
INFO image_rx_daemon: RX worker ready; draining RX_FRAME_URC...
INFO image_rx_daemon: published frame_id=1 412 B → lifetrac/v25/video/tile_delta
INFO image_rx_daemon: stats: rx_frames=12 rx_decode_err=0 frames_published=2 publish_err=0 reassembler_decode_err=0 reassembler_timeouts=1
```

Validation gate: **20 s of nonzero `frames_published` and `rx_decode_err=0`**.

Sanity check on host:

```powershell
mosquitto_sub -h 127.0.0.1 -t 'lifetrac/v25/video/tile_delta' -v | Select-Object -First 3
```

---

## Phase D — Cutover (requires user review of D-tier risk)

Once Phases B + C pass their validation gates:

1. **Verify** `/ws/state` is ingesting tile_delta (look for `tiles=96/96` in web_ui logs).
2. **Drop fallback env** from web_ui launch: remove `LIFETRAC_ALLOW_CMD_IMAGE_FRAME=1`, restart.
3. **Relaunch camera container** without `-FallbackHostMqtt` (so it stops publishing `cmd/image_frame` to host MQTT directly, while still publishing locally so the TX daemon can fragment & ship).
4. **Gate:** 20 s of nonzero tile_delta updates, 96/96 tiles per frame, no UI black-screen.

**Risk:** If the strict path drops more than ~10% of frames the UI will visibly stutter. Keep fallback env-var ready to re-enable.

---

## Phase E — Cleanup (deferred)

- Extend `lora_bridge.py` to subscribe `cmd/image_frame`, fragment with `pack_telemetry_fragments`, and wrap each fragment in a proper TelemetryFrame (with AES-GCM + nonce). This converges the strict path with the bridge architecture and removes the bridge-bypass FIXME.
- Add systemd unit files for both daemons (`lifetrac-image-tx.service`, `lifetrac-image-rx.service`).
- Add tcpdump-based audit hooks for compliance.
- Update `MASTER_TEST_PROGRAM.md` with strict-path soak test.

---

## Files modified or created in this session

| File | Action |
|---|---|
| `firmware/tractor_x8/image_tx_daemon.py` | created |
| `base_station/image_rx_daemon.py` | created |
| `AI NOTES/2026-05-24_Strict_Path_Image_Daemons_Phase_B_C_Plan_v1_0.md` | this file |

No other repo files were touched. The fallback path (camera_service → adb-reverse-mqtt → web_ui) remains untouched and operational.

---

## Outstanding concerns / open questions for user

1. **L072 contention on tractor X8**: stage1 quant runs use `/dev/ttymxc3` on 2E2C. Running `image_tx_daemon.py` will conflict. Need to decide whether to (a) suspend stage1 testing during video runs, or (b) put the TX daemon on a different board.
2. **paho-mqtt on base X8**: probably not pre-installed on 2D0A since no camera container has run there. Deployment script attempts install, but may need offline wheel if base has no network.
3. **No security on-air**: bridge-bypass means no encryption. Acceptable for bench testing, NOT for any deployment outside the lab.
4. **Single-board collapse**: if base and tractor end up co-located (e.g., one X8 doing both), the architecture above needs collapse into the bridge — Phase E.
