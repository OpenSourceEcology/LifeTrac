# W2-02 Image-Over-LoRa Bench — Single-Frame Proof of Life

**Author:** Copilot v1.0
**Date:** 2026-05-18
**Status:** ✅ PASS (4/4 gates green)
**Evidence dir:** `LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/W2-02_image_over_lora_2026-05-18_175638/`

## 1. Goal

Demonstrate the **end-to-end image transmit/receive path** between
two LifeTrac X8 controllers, glued by the Murata L072 + SX1276
HostLink-validated firmware (W1-10b, 100/100 SF7/BW125):

```
Kurokesu C2 (TX X8 /dev/video1)
  → ffmpeg single RGB frame
  → host TileDeltaFrame KEY encoder (96 tiles × 32 px, ≤60 B data fragments)
  → adb push to TX X8
  → method_h_stage2_tx_probe.py burst over HostLink (TX_FRAME_REQ)
  → SX1276 LoRa SF7/BW125 868.1 MHz
  → SX1276 (RX X8) RXCONT
  → method_h_stage2_tx_probe.py rx_listen via HostLink
  → adb pull rx_stdout.txt
  → host TileDeltaFrame reassembler + decoder
  → reconstructed.png on host
```

Success contract (encoded into `run_w2_02_image_over_lora_end_to_end.ps1`):

| Gate | Threshold |
|---|---|
| V1 `tx_ok_rate` | ≥ 0.99 |
| V2 `rx_match_rate` | ≥ 0.95 |
| V3 `frame_complete` | True |
| V4 `tiles_decoded` | == 96 |

## 2. Bench Topology

| Role | ADB serial | Kernel | Camera | Radio |
|---|---|---|---|---|
| TX (image source) | `2E2C1209DABC240B` | 5.10.93 | Kurokesu C2 `/dev/video1` MJPEG | Murata L072 + SX1276 |
| RX (image sink) | `2D0A1209DABC240B` | 6.1.24 | (unused) | Murata L072 + SX1276 |

Both L072s flashed with W1-10b firmware (HostLink @921600 on
`/dev/ttymxc3`, LBT disable, TX_FRAME_REQ payload format
`[tx_id, len, …]`, REG_OP_MODE bypass via `read_reg`/`write_reg`).

## 3. Result

```json
{
  "n_rx_frames": 190,
  "completed_frames": 1,
  "first_frame": {
    "kind": "key",
    "base_seq": 0,
    "grid_w": 12,
    "grid_h": 8,
    "tile_px": 32,
    "tiles_changed": 96,
    "tiles_decoded": 96,
    "tiles_failed": 0
  },
  "reassembler": {
    "completed_frames": 1,
    "decode_errors": 0,
    "duplicate_fragments": 0,
    "bad_magic_passthroughs": 0,
    "timeouts": 0
  },
  "pending_seqs": []
}
```

- **TX:** 190 / 190 OK, 0 timeouts, 0 ERR_PROTO, elapsed_ms≈73 per
  frag, total burst ~54 s.
- **RX:** 190 / 190 received (no gap), `radio_rx_ok_delta=190`,
  `radio_crc_err_delta=0`, RSSI median **−113 dBm**, SNR median
  **+3 dB**.
- **Decode:** 1 complete frame, 96 / 96 tiles decoded, 0 errors,
  0 duplicates, 0 bad-magic passthroughs.
- **Verdict:** **PASS** on all 4 gates.

## 4. Iterative Bench History

The single-shot PASS at 17:56:38 was the 7th orchestrator run of the
day. Earlier runs surfaced two distinct real-bench bugs that the
synthetic round-trip (encoder→reassembler in-process) had completely
missed.

| Run | Time | Outcome | Lesson |
|---|---|---|---|
| 174146 | 17:41 | FAIL — RX 0 frames | First end-to-end attempt. Discovered RX SX1276 not in RXCONT. |
| 174338 | 17:43 | FAIL — RX listener never READY | openocd warm-boot path tried, prep_bridge succeeded but `imx_gpio` SWD failed `cannot read IDR` on Board 1. |
| 174523 | 17:45 | FAIL — same SWD failure | Confirmed openocd unreliable on RX Board 1; pivot to HostLink-only approach. |
| 174704 | 17:47 | FAIL — TX_ok=104/189, RX=0 | First "TX completes but big losses" run. **Two bugs visible:** (a) `__W2_02_TX_ERR__ ERR_PROTO 1001080000` in bursts of 5 after every ~6 OK frags (queue overflow), (b) `RegOpMode(pre)=0x80 (expected 0x85)` (radio in SLEEP). |
| 175139 | 17:51 | FAIL — RX never READY | Re-enabled openocd warm-boot path via `run_method_h_stage2_tx.sh`. prep_bridge OK but SWD still failed. Orchestrator threw before TX. |
| 175419 | 17:54 | PARTIAL — 173/173 received, orchestrator dropped before completion | Introduced `w2_02_radio_wake_rxcont.py` + `inter_s=0.2`. TX/RX behaved perfectly for the first ~50 s but Tee/adb buffer dropped before TX_DONE. |
| **175638** | **17:56** | **PASS** | Same fixes, clean run. 190/190 TX, 190/190 RX, 96/96 tiles, all gates green. |

## 5. Root Causes (Falsified, Not Plausibility-Argued)

### Bug A — TX queue overflow → `HOST_ERR_PROTO_FORBIDDEN` (0x08)

**Symptom:** `__W2_02_TX_ERR__ idx=N ERR_PROTO during TX wait: 1001080000`
in bursts of 5 starting at frag idx=6 (then idx=17, 28, …).

**Mechanism:** Each TX_FRAME_REQ took `elapsed_ms ≈ 73` (LoRa SF7 ToA
59 ms + L072 framing). The orchestrator default `--inter-s 0.05`
queued the next request 50 ms after the previous response was even
returned — far faster than the L072 could drain. The L072 host queue
saturates at ~6 in-flight frames, after which any further enqueue
returns `HOST_ERR_PROTO_FORBIDDEN` (decoded constant
`HOST_ERR_PROTO = 0x08` in the W1 firmware).

**Falsification used:** The numbers fit ("every 6 OK then 5 fails" is
exactly queue-depth-6 with no drain) AND raising `inter_s` from 0.05
to 0.2 collapsed `TX_ERR` count from 85/189 → 0/190 across two
back-to-back runs with no other change.

**Fix:** `run_w2_02_image_over_lora_end_to_end.ps1` param
`InterFragS` default raised `0.05 → 0.2`. The TX wrap-script
forwards it as the probe's `--inter-s`. (At 0.2 s + 0.073 s real
time ≈ 0.275 s/frag × 190 = 52 s — matches observed `TX done in
54 s` exactly.)

### Bug B — RX SX1276 stuck in `LORA_SLEEP` (RegOpMode=0x80)

**Symptom:** `RX_LISTEN: RegOpMode(pre)=0x80 (expected 0x85)` followed
by 0 received frames despite TX transmitting normally.

**Mechanism:** Every probe in `method_h_stage2_tx_probe.py` ends
with a `__RADIO_SLEEP_ON_EXIT__` block that writes
`REG_OP_MODE=0x80` (LORA_SLEEP) as cleanup. The previous probe was
a `rx_listen`, so the radio was left in SLEEP. When a subsequent
`rx_listen` started, it queried RegOpMode (got 0x80), logged the
mismatch, but did **not** re-enter RXCONT — the original path
relied on an openocd-driven board warm-boot to re-initialize the
radio to RXCONT before each `rx_listen` invocation.

The "warm-boot via openocd" path was implemented in
`run_method_h_stage2_tx.sh` (calls `prep_bridge.sh` then
`openocd … -c reset run`) but **the openocd SWD bitbang via imx_gpio
is unreliable on Board 1** ("Error: Error connecting DP: cannot
read IDR"), so the bypass path stopped using it — which silently
removed the re-init step.

**Falsification used:** Two-bug isolation. Run 175139 reproduced the
SWD failure cleanly even when the radio was already known to be in
SLEEP — proved the openocd path is not viable. Then run 175419
introduced `w2_02_radio_wake_rxcont.py` (HostLink-only); the wake
helper printed `__W2_02_WAKE_OK__ opmode_pre=0x80 opmode_post=0x85
action=write`, and the very next `rx_listen` received frames 1…173
matching TX exactly. The 175638 re-run reproduced full 190/190
match, confirming the wake step is necessary AND sufficient.

**Fix:** New helper `w2_02_radio_wake_rxcont.py`:

```python
from method_g_stage1_probe import HostLink
from method_h_stage2_tx_probe import (
    drain_boot, drain_pending, read_reg, write_reg,
    HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC,
    SX1276_REG_OP_MODE, SX1276_OPMODE_LORA_RXCONT,
)
# … main(): HostLink(dev,baud) → drain_boot(0.5) → VER warm-up →
#   drain_pending(0.25,1.0) → read_reg(REG_OP_MODE) →
#   if != 0x85: write_reg(REG_OP_MODE, 0x85) → verify
```

Output contract: `__W2_02_WAKE_OK__ opmode_pre=0xNN
opmode_post=0xNN action=write|none`, RC 0/2/3.

Orchestrator Section 8 now invokes the wake helper before the
`rx_listen` probe; if `__W2_02_WAKE_OK__` is absent in `rx_wake.log`,
the orchestrator throws.

## 6. Methodology Notes (Apply to Future Iterations)

- **Cleanup-on-exit is not free.** Any cleanup write that puts a
  peripheral into a deep state (`LORA_SLEEP`, `STANDBY`,
  power-gate) MUST be paired with a documented re-init step in
  every subsequent test path that uses that peripheral. Otherwise
  the next test silently observes the cleanup, not the operational
  state.
- **Status fields lose causal information.** Run 174704 reported
  `TX_DONE n=189 ok=104 fail=0 timeout=85` — the "0 fail" field
  was misleading: all 85 lost frames went down the `ERR_PROTO`
  branch, not the `timeout` branch. Aggregate counters can hide a
  whole bug class. Always grep the raw stage log for `ERR_`
  patterns even when `fail=0`.
- **Bench openocd availability is per-board.** Board 1's imx_gpio
  SWD has historically been unreliable. Do not assume an openocd-
  dependent path works on both boards; design HostLink-only
  alternatives wherever the operation can be expressed in register
  reads/writes.
- **Synthetic round-trips don't catch radio-state bugs.** The
  `w2_02_host_pipeline.py` synthetic harness ran encoder →
  reassembler in-process and reported 178 frags → 96/96 tiles all
  along — perfectly green while the bench was 0/189. Synthetic
  passes are necessary but never sufficient.
- **adb exec-out can drop output mid-stream.** Run 175419 captured
  173/173 cleanly then dropped the connection before the TX-DONE
  marker, faking a "stall" that was actually a successful run with
  truncated logs. Re-running clean and logging via `Tee-Object`
  reproduced 190/190 the very next attempt. Don't conclude
  failure from missing terminal output alone — check on-device
  process state and file timestamps.

## 7. Artifacts (this session)

### Created
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/w2_02_radio_wake_rxcont.py`
- `LifeTrac-v25/AI NOTES/2026-05-18_W2-02_Image_Over_LoRa_Bench_Plan_Copilot_v1_0.md` (this file)
- `/memories/repo/lifetrac-w2-02-rx-wake.md`

### Edited
- `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_w2_02_image_over_lora_end_to_end.ps1`
  - `$InterFragS` default `0.05 → 0.2`
  - Section 8 (RX) now: (a) push + run `w2_02_radio_wake_rxcont.py`
    on RX (logs to `rx_wake.log`, throws if not `__W2_02_WAKE_OK__`),
    (b) invoke `rx_listen` directly via `_rx_wrap.sh` (no openocd
    warm-boot).
- `LifeTrac-v25/TODO.md` §K — promoted W2-02 single-frame
  proof-of-life from 🟥 pending to 🟩 PASS with evidence path and
  gate-by-gate verdict.

### Untouched (W1-10b proven; do **not** modify)
- `method_g_stage1_probe.py`
- `method_h_stage2_tx_probe.py`
- `run_method_h_stage2_tx.sh`

## 8. Next Steps (out of scope for this PoL)

1. Repeat-shot stability test (20-30 back-to-back KEY frames) to
   characterize variance of `tx_ok_rate` and `rx_match_rate` and
   verify no thermal/queue drift.
2. Replace the bench HostLink TX path with the real `tractor_h7`
   M7 `TileDeltaFrame` scheduler (the V1 P0-starvation gate in
   Phase 5D is the proof obligation).
3. Wire the `register.py` + `tile_diff.py` pipeline so subsequent
   frames send DELTA, not KEY (validate the 96-tiles-changed=
   full-canvas claim drops to a handful of tiles between adjacent
   captures).
4. Investigate moving the cleanup-on-exit policy: either (a) leave
   the radio in RXCONT instead of SLEEP after `rx_listen`, or
   (b) make the wake helper an unconditional preamble in
   `method_h_stage2_tx_probe.py::run_rx_listen` itself.

## 9. Final Counts (reproduce these for any future PASS claim)

```
TX_FRAG OK    : 190 / 190   (100.00 %)
TX_ERR        :   0
TX_TIMEOUT    :   0
RX_FRAME      : 190 / 190   (100.00 %, payload-matched)
radio_rx_ok_delta   : 190
radio_crc_err_delta :   0
real_faults         :   0
invariants_violated :   0
RSSI median         : −113 dBm
SNR median          :   +3 dB
tiles_decoded       :  96 / 96
decode_errors       :   0
duplicate_fragments :   0
bad_magic_passthroughs : 0
reassembler timeouts:   0
```
