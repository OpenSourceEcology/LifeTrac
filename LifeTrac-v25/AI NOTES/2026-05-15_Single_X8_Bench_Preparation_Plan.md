# Single-X8 Bench Preparation Plan

**Date:** 2026-05-15
**Author:** Copilot (deep-analysis note, single pass)
**Scope:** Activities that move the LifeTrac-v25 controller program forward
while exactly one Portenta X8 is functioning. Captures the triaged list
discussed in chat so it can be sequenced into the W2/W3 plan without
re-deriving it later.

---

## 1. Why this matters

The dual-X8 architecture (handheld + tractor) is the cleanest test
target, but every week we wait on the second board is a week of
schedule risk. Most of the gates in
[`MASTER_TEST_PROGRAM.md`](../MASTER_TEST_PROGRAM.md) only need *one*
real X8 plus a laptop surrogate, and almost all of the integration
bugs you'd find on bench day can be flushed out now if we stage the
work right.

Today's baseline (verified 2026-05-15):

- `LifeTrac-v25/DESIGN-CONTROLLER/base_station/run_tests.ps1` → **69/69
  files green** under process isolation.
- IP-W2-09 end-to-end image-pipeline contract + V2 latency gate green
  (p99 well under the 80 ms Python-portion budget; <500 ms total budget
  has >420 ms of headroom for camera + LoRa airtime).
- IP-W2-07 X8 honours `CMD_ENCODE_MODE`; W2-08 offline IPC capture
  replay tool shipped.
- One X8 is bringing up the Kurokesu C2 USB cam over `/dev/video0` via
  `ffmpeg` inside an Arduino OOTB Foundries.io container; second X8
  status: out of service / awaiting replacement.

Everything below assumes that single X8 is reachable over `adb`, has
its LoRa SX1276 attached, and can talk to a laptop running the base
station services from
[`base_station/`](../DESIGN-CONTROLLER/base_station).

---

## 2. Capture corpora to file (do now, replay forever)

Cheapest, highest leverage: every bench session loses some real-world
data we can never get back. Persist it.

### 2.1 Live IPC capture → replay corpus

- **What:** 30–60 s of `cat /dev/ttymxc1 > capture.bin` on the X8 with
  the camera framing varied scenes (static room, hand-wave motion,
  low-light shutter, all-white reset frame, tile-edge motion).
- **How:** `adb exec-out timeout 60 cat /dev/ttymxc1 > corpus_<scene>.bin`
  for each scene tag. Pull to host, drop into a fixtures dir
  (recommend `LifeTrac-v25/DESIGN-CONTROLLER/base_station/tests/fixtures/ipc/`).
- **Why:** [`replay_ipc_capture.py`](../DESIGN-CONTROLLER/base_station/replay_ipc_capture.py)
  already turns these into per-frame PNGs and exercises the full
  `image_pipeline` decode path. With committed corpora, every laptop
  in CI can regression-test the encoder + reassembly + canvas without
  any hardware.
- **Cost:** ~1 hour of bench time, ~5 MB per scene tag at typical
  WebP-tile bitrates. Use Git LFS if it grows past 50 MB total.
- **Maps to W-item:** prerequisite for promoting IP-W2-08 from
  "offline-capable" to "regression-gated".

### 2.2 Encrypted on-air pcap-equivalent

- **What:** Stream the KISS-framed bytes leaving the X8's LoRa modem
  to a file while the X8 transmits its own captures from §2.1.
- **How:** Either (a) probe TX with a logic analyzer / SDR and dump
  bytes, or (b) cheaper: have the X8 also log every frame it tries to
  enqueue (already half-wired in `lora_proto.py` audit hooks; just
  need the file sink).
- **Why:** Brings up [`lora_bridge.py`](../DESIGN-CONTROLLER/base_station/lora_bridge.py)
  decode + dedup + replay-window audit *without* a second radio. Lets
  you fuzz the join layer at line speed against real airtime
  jitter/ordering.
- **Cost:** ~2 hours including the file-sink wiring.

### 2.3 Camera negative-case captures

- **What:** Captures of the camera under failure modes, captured at
  the IPC layer (so we can replay them through the fallback ladder):
    - USB cam unplugged mid-stream (triggers re-enumerate path).
    - CPU pegged at 95 % via `stress-ng` (triggers `accel_select`
      backoff to lower-cost encode modes).
    - Frame drop induced by `nice -20` competing process.
    - Low-light + saturated-light + all-black frames.
    - Rapid scene change (paper waved at lens).
- **Why:** Drives the `EncodeMode` auto-fallback ladder (W2-07) and
  the `accel_select` / `fallback_render` paths through real entry
  conditions, not just SIL fixtures. Any divergence between the
  ladder spec and what the encoder actually does gets caught here.
- **Cost:** ~2 hours.

---

## 3. Single-board HITL gates (one X8, no peer)

These promote existing SIL tests to real-hardware variants. Same
assertions, same fixtures, but the SUT is the actual board.

### 3.1 LoRa TX→RX self-loopback

- **What:** Jumper the SX1276 TX path back into the same RX (or use a
  short-range attenuator + a second SX1276 wired to the same X8 if
  you have one on a HAT).
- **Exercises:** `TelemetryReassembler`, `pack_telemetry_fragments`,
  audit ledger, replay-window invariant — under *real* RF airtime
  jitter, not synthetic timestamps. Catches any assumption in the
  bridge that doesn't survive ms-scale noise.
- **Promotes:** [`test_telemetry_fragmentation.py`](../DESIGN-CONTROLLER/base_station/tests/test_telemetry_fragmentation.py),
  [`test_replay_window_invariant.py`](../DESIGN-CONTROLLER/base_station/tests/test_replay_window_invariant.py).
- **Cost:** ~3 hours (RF wiring + fixture).

### 3.2 Power / brownout sequencing

- **What:** Variac or programmable bench supply on the 5 V rail,
  trip down to 3.3 V for 50 ms, snap back, observe boot self-test.
- **Exercises:** Watchdog, M4 safety MCU latch, `boot_self_test.py`
  hash gate.
- **Promotes:** [`test_boot_self_test_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_boot_self_test_sil.py),
  [`test_boot_phy_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_boot_phy_sil.py).
- **Cost:** ~4 hours including jig.

### 3.3 Watchdog + M4 safety MCU dead-man test

- **What:** Kill the M7 firmware process (`kill -9` over `adb`),
  confirm M4 cuts hydraulic enables within the budget defined in
  the safety spec.
- **Promotes:** [`test_m4_safety_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_m4_safety_sil.py).
- **Cost:** ~2 hours; needs the hydraulic-enable line broken out to
  a logic probe with timestamp.

### 3.4 Fleet-key provisioning rehearsal

- **What:** Run the documented provisioning SOP end-to-end on the one
  X8: install OOTB image, drop fleet key, verify `lora_proto`
  refuses to load without it (we already test the env override in
  CI), confirm encrypted frames appear on TX.
- **Why:** Shakes out the SOP before bench day. Any "wait, where does
  the key file actually go" surprises happen now, not when you've
  got two boards on the bench and a deadline.
- **Cost:** ~1 hour.

---

## 4. Surrogate harnesses (laptop pretends to be the missing X8)

The biggest unlock. With a small Python peer, the working X8 can do
full integration testing against a software stand-in for the absent
board.

### 4.1 Python LoRa peer (highest priority)

- **What:** A thin script that opens the host's USB-LoRa dongle,
  reuses [`lora_proto.py`](../DESIGN-CONTROLLER/base_station/lora_proto.py)
  for KISS / AES-GCM / header / CRC, and plays the missing role:
    - If the working X8 is the **tractor**, the peer pretends to be
      the **handheld** (sources: SRC_HANDHELD=0x01, sinks: telemetry,
      tile_delta, status).
    - If the working X8 is the **handheld**, the peer pretends to be
      the **tractor** (sources: tile_delta, telemetry; sinks:
      control, estop, encode_mode, req_keyframe).
- **Why:** Unblocks the entire base-station integration path with one
  real radio. Bridge join, MQTT fan-out, browser canvas + overlays,
  3-tier control source priority — all testable today.
- **Notes:**
    - Reuse `audit_log.py` and `nonce_store.py` so the peer's frames
      pass replay-window validation in the bridge.
    - Add `--inject-malformed`, `--drop-rate`, `--reorder` flags so
      this same harness doubles as a fuzzing rig for `lora_bridge.py`.
    - Keep it under 400 lines; this is a bench tool, not a product.
- **Cost:** ~1 day of focused work.
- **Recommend:** start here.

### 4.2 USB-485 modbus slave bench

- **What:** Cheap USB-RS485 dongle on the laptop, run a Python modbus
  slave that mimics the hydraulic controller. The one X8 talks to it
  via the same serial path as the real hydraulic side.
- **Promotes:** [`test_modbus_slave_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_modbus_slave_sil.py).
- **Cost:** ~4 hours including dongle setup.

### 4.3 Browser USB-gamepad → X8 → motor mock

- **What:** Run the base-station web UI on the laptop, connect a USB
  gamepad in the browser, route through MQTT → bridge → real LoRa →
  the one X8, and have the X8 drive a motor mock (LED PWM array or
  scope channels) instead of real hydraulic valves.
- **Why:** Catches deadband, ramp, and reversal-brake bugs against
  *real* timing, not SIL clocks. Also exercises the new 3-tier
  priority policy (handheld > browser > autonomy) end-to-end with
  the surrogate peer from §4.1 driving handheld traffic.
- **Promotes:** [`test_axis_deadband_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_axis_deadband_sil.py),
  [`test_axis_ramp_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_axis_ramp_sil.py),
  [`test_reversal_brake_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_reversal_brake_sil.py),
  [`test_steering_priority_sil.py`](../DESIGN-CONTROLLER/base_station/tests/test_steering_priority_sil.py).
- **Cost:** ~1 day, mostly the motor-mock fixture.

---

## 5. Process / docs prep

Cheap, no hardware contention with bench work.

### 5.1 Bench-wiring harness BOM + photo doc

- **What:** Connector list, USB hub, 12 V splitter, RF wiring, ESD
  precautions for the eventual dual-X8 rig. Photo every step as you
  build the single-X8 jig so the second-X8 jig is a copy job.
- **Why:** When the second X8 lands, you want plug-and-go in <30 min,
  not "where did I put that DC barrel adapter".

### 5.2 GitHub Actions CI matrix

- **What:** GitHub workflow that runs `run_tests.ps1` on every push
  and replays the §2.1 corpora through `replay_ipc_capture.py`.
- **Cost:** ~1 hour. Existing workflows under `.github/workflows/`
  give you the runner setup pattern.
- **Why:** No more silent regressions in `image_pipeline` or
  `lora_proto` because someone forgot to run the suite locally.

### 5.3 MASTER_TEST_PROGRAM cross-check

- **What:** Walk every gate listed in
  [`MASTER_TEST_PROGRAM.md`](../MASTER_TEST_PROGRAM.md) and tag each
  as `SIL-only`, `HIL-1board`, or `HIL-2board`. Anything currently
  living in `HIL-2board` that *could* be `HIL-1board` (with a
  surrogate from §4) gets re-tagged.
- **Why:** Surfaces any gate that secretly needs two X8s before bench
  day, when finding out is expensive.
- **Cost:** ~3 hours.

---

## 6. Recommended sequencing

Two-week prep window with one engineer:

| Week | Day | Task | Output |
|------|-----|------|--------|
| 1 | 1   | §2.1 IPC capture corpus + commit | regression fixture |
| 1 | 1   | §5.2 CI workflow                 | green badge on `main` |
| 1 | 2-3 | §4.1 Python LoRa peer            | surrogate handheld/tractor |
| 1 | 4   | §2.3 negative-case captures      | encode-fallback corpus |
| 1 | 5   | §5.3 MASTER_TEST_PROGRAM xref    | tagged catalog |
| 2 | 1-2 | §4.3 web → X8 → motor-mock loop  | full control HIL |
| 2 | 3   | §3.1 LoRa self-loopback          | airtime regression |
| 2 | 4   | §3.4 fleet-key SOP rehearsal     | shaken-out provisioning |
| 2 | 5   | §3.2/§3.3 power + watchdog HIL   | safety gates promoted |

§4.2 modbus and §5.1 bench-photo doc fit in any quiet hour.

---

## 7. What this plan deliberately defers

- **W2-10 `image_pipeline` package rename**: no longer on the
  critical path now that
  [`run_tests.ps1`](../DESIGN-CONTROLLER/base_station/run_tests.ps1)
  isolates each test file in its own process. Address when next
  touching that area.
- **Real two-X8 OTA bring-up**: needs the second X8. The work above
  ensures the *first* time both boards are on the bench is the
  *only* time we have to debug both-board interactions.
- **Field RF range testing**: needs two X8s and outdoor space.
  Schedule for after §3.1 and §4.1 close.

---

## 8. Open questions for the user

1. Do you want IPC corpora in-tree (`tests/fixtures/ipc/*.bin`,
   ~50 MB upper bound) or in Git LFS / a separate release artifact?
2. Which surrogate role is more useful first — handheld or tractor?
   (i.e. which X8 do you currently have working?) Default assumption
   above: working X8 is the **tractor**, surrogate is the
   **handheld**.
3. Is there a target date for when the second X8 is expected back?
   Drives whether to invest the full 2-week plan or compress.
