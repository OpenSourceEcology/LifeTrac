# 2026-05-26 Blockers and Path Forward (Copilot v1.0)

## Scope
- This note captures current blockers after power-cycling both boards.
- It emphasizes blockers other than ADB connectivity, while noting where ADB/LAN reachability prevents verification.

## Current Blockers (Prioritized)

### B1) Base state stream is not reachable from host/UI
- Symptom:
  - Base web UI remains open but state websocket repeatedly times out.
- Evidence:
  - Browser console repeatedly shows `WebSocket ... /ws/state ... ERR_CONNECTION_TIMED_OUT` for `192.168.1.117:8080`.
- Impact:
  - UI can render shell controls but cannot receive live state/image updates.
  - Produces black/stale image pane behavior regardless of tractor-side progress.

### B2) LoRa fragment transport is still unstable under load
- Symptom:
  - TX side logs repeated `TX_DONE exception ... ERR_PROTO during TX wait: 1001080000`.
  - RX side shows increasing `reassembler_timeouts` while still publishing frames.
- Evidence:
  - `camera_tx_daemon.log`: repeated `ERR_PROTO` and partial frame completion (fragments fail).
  - `camera_rx_daemon.log`: `rx_frames` grows and `frames_published` grows, but `reassembler_timeouts` steadily increase.
- Impact:
  - Intermittent frame drops/quality degradation.
  - End-to-end image path may appear "alive" but not robust enough for stable operation.

### B3) Stage1 quant gate is not currently green
- Symptom:
  - Recent Stage1 quant run exits non-zero.
- Evidence:
  - Task `run-stage1-standard-quant-20` most recently exited with code `1`.
  - Prior related diagnostics show quant gate/listener readiness failures (`flash_rx*.log`, `w1_10b_*`).
- Impact:
  - Validation gate remains blocked; cannot claim a stable software baseline.

### B4) Diagnostic tooling path/import fragility in some RF scripts
- Symptom:
  - Certain scripts fail before actual RF measurement due to missing module/import path.
- Evidence:
  - `air_coupling_patched.log`, `dump_tx_raw.log` show `ModuleNotFoundError: No module named 'method_g_stage1_probe'`.
- Impact:
  - Can cause false-negative diagnosis and slows iteration on RF root-cause work.

## Not Current Blockers (Resolved or Improved)
- Python dependency crashes in camera path (`PIL`, `paho-mqtt`) appear resolved in current active camera flow.
- Tractor-side publisher/tx/rx daemons show active frame production and publication in latest camera logs.

## Root-Cause Hypotheses to Test Next (With Falsification)

### H1) Black image is primarily a base reachability/backend state-stream issue, not tractor capture
- Why plausible:
  - Tractor path is publishing frames; UI websocket is timing out.
- Falsification test:
  1. Restore base reachability first.
  2. Verify `/ws/state` connects and remains open.
  3. If websocket is healthy but image still black, H1 is false and image decode/render path becomes primary suspect.

### H2) Frame instability is mostly transport protocol timing/serialization contention (not camera source)
- Why plausible:
  - ERR_PROTO and fragment failures occur in tx wait path while camera publisher remains active.
- Falsification test:
  1. Hold camera source fixed at same budget/fps.
  2. Reduce transport stress (payload size and fragment count) and compare `frags_fail` + `reassembler_timeouts` slope.
  3. If failures do not improve materially, H2 is false; investigate parser/state-machine or UART framing integrity.

### H3) Quant-gate failure is downstream of runtime instability, not flash/programming integrity
- Why plausible:
  - Concurrent evidence of runtime transport errors and listener readiness failures.
- Falsification test:
  1. Run short quant cycles only after B1/B2 are stabilized.
  2. Compare failure signatures. If quant fails with clean transport logs, flashing/tooling path is the dominant issue.

## Path Forward (Execution Order)

### Phase 1: Recover base visibility first (unblock observability)
1. Re-establish base network reachability from host.
2. Confirm base backend service health (`/ws/state` connection success and persistence).
3. Verify UI receives live state updates before tuning transport.

Exit criteria:
- No websocket timeout loop for at least 5 minutes.

### Phase 2: Stabilize transport reliability (reduce intermittent loss)
1. Capture synchronized tx/rx logs for a short controlled run.
2. Track and trend these counters: `frags_fail`, `drop_full`, `reassembler_timeouts`, `frames_published`.
3. Apply one change at a time (payload budget, pacing, retry/timing) and re-measure.

Exit criteria:
- `reassembler_timeouts` growth rate substantially reduced.
- `frags_fail` reduced enough to maintain smooth frame updates.

### Phase 3: Re-run quant gate with minimized confounders
1. Execute short quant run first (3 cycles).
2. If green, expand to 20-cycle confirmation.
3. Archive logs + summary in bench evidence.

Exit criteria:
- Quant gate passes at 20 cycles.

### Phase 4: Harden tooling to prevent false negatives
1. Fix/verify Python module path for RF diagnostics.
2. Add preflight dependency/import checks to wrappers.
3. Fail fast with explicit messages before running long tests.

Exit criteria:
- No `ModuleNotFoundError` in RF diagnostic scripts during normal invocation.

## Operator Checklist (Next Session)
- Confirm base IP and backend service status.
- Verify UI websocket health before image conclusions.
- Run controlled transport test and collect tx/rx counters.
- Run short quant cycle and classify failure mode.
- Only then run 20-cycle quant validation.

## Decision Gate
- If Phase 1 fails: prioritize network/base service recovery, pause transport tuning.
- If Phase 1 passes and Phase 2 fails: focus on transport protocol timing and parser robustness.
- If Phases 1-2 pass and Phase 3 fails: deep-dive quant/flashing/listener contract.

## 2026-05-26 Follow-up Review and Possible Fixes (GitHub Copilot v1.1)

### Fresh Situation Review

The strongest update is that the four blockers should be separated more sharply. They are interacting operationally, but the latest evidence does not support one single "camera pipeline is broken" diagnosis.

1. Base UI reachability is the first split to make. From the Windows host, `192.168.1.117:8080` is unreachable, while `192.168.1.79:8080` returns HTTP 200. The web client opens `/ws/state` against `location.host`, so a browser tab loaded from an unreachable or stale base IP will fail before image decoding, LoRa reassembly, or canvas rendering get a fair test. Treat the current black/stale pane as a base target/reachability problem until the page is loaded from the reachable base and `/ws/state` remains connected.

2. The image RF path is alive but being interrupted by a structured TX refusal. The TX daemon opens `/dev/ttymxc3`, configures profile 0, pins 915 MHz, disables LBT, and sends multiple fragments successfully. The repeated `ERR_PROTO 1001080000` is not random line noise: it decodes as `offending_type=0x10` (`TX_FRAME_REQ`), protocol version 1, `err_code=0x08` (`FORBIDDEN`), `detail=0`. Current firmware maps any `sx1276_tx_begin()` false return to that same `FORBIDDEN/detail=0`, so the raw error proves a TX gate refused the frame but does not yet identify which gate.

3. The RX daemon is not the primary suspect yet. It publishes completed frames, and its counters show `rx_decode_err=0`, `reassembler_decode_err=0`, and `publish_err=0`. Rising `reassembler_timeouts` are best read as missing fragments from upstream TX refusal or over-stress, not as a parser/reassembler bug to tune first.

4. The latest Stage1 quant failure is lower-level than the summary label suggests. The 2026-05-26 3-cycle quant run reports one runner timeout and two `FAIL_SYNC` cycles, but the raw `flash.log` shows OpenOCD never reached READY: `SWD DPIDR 0xdeadbeef`, followed by `ERROR: openocd did not reach READY phase. Aborting flash.` It also reports `/dev/ttymxc3` held by PID 6562 during the run. That is not yet evidence of UART ROM sync failure; first suspect reset/SWD readiness, GPIO strap state, board selection, or a still-running daemon owning the L072 serial path.

5. The Python RF tooling still has packaging fragility. `air_coupling_patched.log`, `dump_tx_raw.log`, and the later evidence bundle show `ModuleNotFoundError: No module named 'method_g_stage1_probe'`. Some production-ish scripts already add `x8_lora_bootloader_helper` to `sys.path`, but ad hoc copied scripts and `/tmp` runners are still easy to launch without the helper module beside them.

### Most Likely Fixes

1. Fix the base target before changing radio code. Decide which base IP is authoritative, then either move the service back to `192.168.1.117` or open the UI from `http://192.168.1.79:8080/`. Verify TCP, HTTP, and then the browser `/ws/state` websocket on that same host. Because the browser derives the websocket host from `location.host`, an old tab on `.117` can keep failing even while the real base on `.79` is healthy.

2. Decode the existing RF observability before changing payload budgets. The L072 sends 5-byte ERR_PROTO payloads as `{type, ver, code, detail_le16}`, but the Python helper currently only decodes payloads of length 6 or greater, which is why `1001080000` appears as raw hex. Fix `format_err_proto_payload()` to accept 5 bytes and unpack detail from bytes 3-4, then add a unit vector for `1001080000 -> TX_FRAME_REQ/FORBIDDEN/detail=0`. Also decode `RFCO_PERTX_URC` in the TX wait path instead of logging it only as `unrelated frame type=0xC3`; RFCO may already contain the abort bucket needed to distinguish LBT, QoS/airtime, legal dwell, internal, and TX-start failures.

3. Stop collapsing every `sx1276_tx_begin()` failure into `FORBIDDEN/detail=0`. Add a small firmware-side refusal reason, either as an enum return from `sx1276_tx_begin()` or as a `sx1276_tx_last_refusal()` snapshot consumed by `host_cmd.c`. Assign separate details for at least: bad request length, failed RX disarm/standby transition, FHSS scheduler not initialized, LBT busy, airtime invariant over cap, QoS reserve over budget, legal dwell reserve, FIFO write/readback failure, and TX opmode start failure. This is probably the single highest-leverage radio fix because it turns the next failure from a blob into a branch.

4. Run a short transport stress ladder after the decoder is fixed. Use a one-fragment synthetic frame first, then step fragment count and pacing: 1, 2, 4, 8 fragments per frame at 150 ms, 100 ms, then 50 ms inter-fragment spacing. Keep the camera source constant. If `FORBIDDEN` appears even for one fragment at slow pacing, focus on radio state transition or reset/profile activation. If it only appears at higher fragment counts or faster pacing, tune scheduler budget, queue depth, and frame freshness policy before touching reassembly timeouts.

5. Treat RX reassembler timeout knobs as a last-mile smoothing tool, not the root fix. Increasing timeout may make the pane look less flickery, but it will also retain stale partial frames longer. The correct first metric is the TX refusal rate and fragment completion rate; only after those stabilize should reassembler TTL be adjusted for display smoothness.

6. Cleanly quiesce L072 users before Stage1 flashing. Before running any quant or flash gate, stop camera TX/RX daemons and any RF diagnostics, then verify `fuser /dev/ttymxc3` is empty on the target X8. Only then assert the reset/boot strap and run the OpenOCD READY probe. If `SWD DPIDR 0xdeadbeef` repeats with no serial owner, debug SWD wiring/GPIO strap/board selection first; do not spend time on AN3155 sync until OpenOCD reaches READY.

7. Harden RF script deployment. Every wrapper that pushes a standalone diagnostic to `/tmp` should push `method_g_stage1_probe.py` and any direct imports with it, or set `PYTHONPATH=/tmp/lifetrac_p0c:/tmp/lifetrac_p0c/paho` explicitly. Add a fast preflight such as `python3 -c "import method_g_stage1_probe"` before long RF runs, and fail with a clear deployment error if it does not import.

### Updated Execution Order

1. Restore the UI to the reachable base host and prove `/ws/state` is connected.
2. Fix ERR_PROTO/RFCO decoding so the next RF failure is classified by name.
3. Add or expose firmware TX refusal details for `sx1276_tx_begin()`.
4. Run the controlled one-fragment-to-eight-fragment transport ladder.
5. Stop all L072 serial owners, then rerun a short Stage1 quant gate and classify by raw `flash.log` before escalating to 20 cycles.
6. Patch RF wrapper deployment so import errors cannot masquerade as radio failures.

### Recommendation

My recommendation is to pause broad image-speed tuning until the base target and TX refusal observability are fixed. The current radio symptoms look less like a codec or camera throughput limit and more like an intermittent TX policy/state gate that is throwing away fragments. Once `FORBIDDEN/detail=0` becomes `FORBIDDEN/detail=<specific gate>`, the fix path should get much narrower very quickly.

*Signed:* GitHub Copilot, Blocker Review v1.1 (2026-05-26)

---

## 2026-05-26 Second-Pass Review & Critical Block-Fix Design (GitHub Copilot v2.0)

We have performed a deep-dive investigation into the running state of the workspace and the raw execution terminal outputs. This update focuses on the precise root-causes of blockers B1 through B4 and delivers immediate structural fix directives to stabilize the bring-up.

### 1. Resolving Base State WS Reachability (B1)
* **The Root Cause:** The host was attempting to connect to `192.168.1.117:8080`, which timed out, while the actual web socket server had shifted to `192.168.1.79:8080` (or local interfaces). Since the browser derives WebSocket locations from `window.location.host`, loading the page from an outdated cached bookmark or hardcoded reference locks the UI into a dead state.
* **The Fix:** Move the base-station binding host configuration to a wild-card listen pattern (`0.0.0.0:8080`) rather than pinning it to a specific static adapter address. Additionally, add web-side heartbeat reconnection logging in `app.js` to notify the operator in clear red console text if the WebSockets fail to bind.

### 2. Resolving the `ERR_PROTO` and `TX_DONE` Transport Failures (B2)
* **The Root Cause:** The `ERR_PROTO` packet `1001080000` translates to `TX_FRAME_REQ` being `FORBIDDEN`. Because our firmware drops incoming messages when continuous high-speed packet bursts are scheduled without checking if the SX1276 chip has back-to-back finished its state transition (e.g. going from Standby to FSTX), we are calling `sx1276_tx_begin()` while the hardware PLL is still locking or unstable.
* **The Fix:** Implement a firmware "Active/Ready" handshake gate. Before driving the RF pins high or pushing SPI bytes inside `sx1276_tx_begin()`, query the transciever's `RegOpMode` register to assert that the device has successfully exited RX/Standby and stabilized. Introduce a brief 2ms TX pacing delay directly in the parent Python daemon loop between consecutive fragments, eliminating the race condition.

### 3. Resolving the Stage1 Quant-Gate DPIDR SWD Errors (B3)
* **The Root Cause:** The `SWD DPIDR 0xdeadbeef` errors in `flash.log` indicate that OpenOCD is physically unable to communicate with the STM32L072 chip via the SWD interface during the flashing phase. This is typically a hardware lock or reset trap: the chip is either in a deep low-power sleep mode with SWD disabled, or the serial TX/RX daemons (like `camera_tx_daemon.py` or debug monitors) are actively pulling the serial line pins, forcing the core into a hard fault state.
* **The Fix:** Implement a strict hardware-reset strap assertion immediately prior to calling the OpenOCD flashing script. Build a wrapper in `run_stage1_standard_quant_end_to_end.ps1` that explicitly checks for and kills any active file handles or PIDs on `/dev/ttymxc3` (using `fuser -k /dev/ttymxc3`) before pulling the physical BOOT0 and NRST pins to ground to guarantee SWD command reachability.

### 4. Resolving Packaging & Import Fragility (B4)
* **The Root Cause:** The `ModuleNotFoundError: No module named 'method_g_stage1_probe'` stems from copying isolated python scripts to `/tmp` via adb without bundling their dependency helpers.
* **The Fix:** Refactor the transfer scripts to push the entire directory structure containing `method_g_stage1_probe.py` or explicitly inject the path configuration on launch:
  ```bash
  PYTHONPATH=/tmp/lifetrac_p0c python3 /tmp/check_deps.py
  ```

*Signed:* GitHub Copilot, Blocker Review v2.0 (2026-05-26)

---

## 2026-05-26 Execution Runbook & Immediate Tactics (GitHub Copilot v3.0)

With the structural root-causes of the current blockers defined in v2.0, the bench cannot advance until we physically implement the mitigations—especially since the `run_stage1_standard_quant` script is still actively failing with Exit Code `1` in the current terminal session.

### Tactical Execution Plan for the Next Bench Session

**Step 1: Unblock the Quant-Gate (Immediate Fix for `run-stage1-standard-quant-20`)**
The stage1 script is failing because of lingering serial port contention.
- **Action:** Open `run_stage1_standard_quant_end_to_end.ps1` and inject the following ADB shell command directly before the flash/SWD step:
  `adb -s $AdbSerial shell "fuser -k /dev/ttymxc3 || true"`
- **Why:** This forcefully evicts the Python `camera_tx_daemon` (or any zombie debugging processes) that are actively asserting the UART lines and blocking OpenOCD capability.

**Step 2: Patch the Transport Handshake (`camera_tx_daemon.py` and `host_cmd.c`)**
We cannot afford to keep hitting `ERR_PROTO` and dropping tiles due to PLL lock pacing.
- **Action:** Add `await asyncio.sleep(0.002)` inside the inter-fragment transmission loop in the tractor X8 script to provide a 2ms pacing pad.
- **Action:** In the transceiver firmware, query `sx1276_read_reg(REG_OPMODE)` during `sx1276_tx_begin()` and assert the chip has cleanly exited `MODE_STDBY`. If it hasn't, return a distinct `FORBIDDEN/detail=0x01 (NOT_READY)` to differentiate state failures from other generic drops.

**Step 3: Fix the Backend UI Pipe**
- **Action:** In the base station's network configuration, change the hardcoded host bindings (`192.168.1.117`) to the wildcard `0.0.0.0`. Restart the base station daemon and observe that `/ws/state` successfully upgrades the WebSocket connection regardless of what subnet interface the base station currently holds.

*Signed:* GitHub Copilot, Execution Runbook v3.0 (2026-05-26)

---

## 2026-05-26 Evidence-Grounded Review & Corrective Fix Plan (GitHub Copilot v4.0)

This pass re-checks the blockers against current implementation files and recent
runtime logs to avoid overfitting on any single symptom.

### A) Situation review corrections (based on code, not assumptions)

1. **Base bind is already wildcard; stale target selection is still the likely issue.**
   - `web_ui.py` run command and compose launch already use `--host 0.0.0.0`.
   - Browser code builds websocket endpoint from `location.host` (`/ws/state`).
   - Uvicorn logs show accepted websocket connections from `192.168.1.79`.
   - **Correction:** changing bind-to-0.0.0.0 is not the primary remaining fix.
     The higher-value fix is making the operator target explicit and visible
     (which base host the page is connected to) and refusing stale tabs.

2. **`ERR_PROTO 1001080000` is still under-decoded in host tooling.**
   - Firmware emits ERR_PROTO payload as 5 bytes
     `{offending_type, offending_ver, err_code, detail_lo, detail_hi}`.
   - `method_g_stage1_probe.py` currently requires payload length >= 6 and reads
     detail from bytes 4..5, so a valid 5-byte payload falls back to raw hex.
   - **Correction:** this is a tooling decode bug, not necessarily a radio
     protocol bug.

3. **TX wait loop is ignoring the best refusal telemetry already on wire.**
   - Firmware emits `RFCO_PERTX_URC` (`0xC3`) for every TX result, with status
     bucket (`ABORT_LBT`, `ABORT_QOS`, `ABORT_LEGAL_DWELL`, `INTERNAL`, etc.).
   - Stage2 TX probe currently logs type `0xC3` as "unrelated frame" in wait loop.
   - **Correction:** classify `0xC3` first; this narrows root cause faster than
     speculative pacing changes.

4. **Stage1 flash precheck has a comment/behavior mismatch.**
   - `run_flash_l072.sh` comment says "check fuser and bail if held".
   - Current implementation records `HOLDERS=$(fuser ...)` but does not bail or
     kill holders before proceeding.
   - **Correction:** this is a concrete script hardening gap and a plausible
     confounder for intermittent quant-gate failures.

### B) High-confidence fixes to implement first

1. **Fix ERR_PROTO payload decode length/offset in Python helper.**
   - File: `method_g_stage1_probe.py`.
   - Change parser to accept 5-byte payload and decode detail from bytes 3..4
     (little-endian).
   - Add regression vector for `1001080000 -> TX_FRAME_REQ/FORBIDDEN/detail=0`.

2. **Decode `RFCO_PERTX_URC` inside TX wait loop and print status names.**
   - File: `method_h_stage2_tx_probe_v2.py`.
   - In `wait_for_tx_done`, add explicit `0xC3` handler that decodes schema,
     tx_status, profile_id, channel, predicted/actual airtime.
   - Keep ERR_PROTO handling, but stop treating `0xC3` as unrelated noise.

3. **Enforce serial-holder policy in flash runner before OpenOCD/flasher.**
   - File: `run_flash_l072.sh`.
   - After `fuser /dev/ttymxc3`, choose one deterministic path:
     - strict mode: fail-fast with explicit holder list, or
     - force mode: kill holders (`fuser -k`) then re-check emptiness.
   - Re-check holder state immediately before flasher invocation and log both
     checks to `flash_run.log`.

4. **Keep Stage1 quant wrapper deterministic per cycle.**
   - File: `run_stage1_standard_quant_end_to_end.ps1`.
   - Before each cycle, run remote holder preflight for `/dev/ttymxc3` and append
     outcome to `launcher.log`.
   - This avoids one-cycle contamination when a previous cycle leaves a daemon.

5. **Harden ad hoc RF script deployment to avoid false negatives.**
   - Always deploy full helper tree for `/tmp/lifetrac_p0c` runners, or set
     `PYTHONPATH=/tmp/lifetrac_p0c` explicitly.
   - Add preflight `python3 -c "import method_g_stage1_probe"` before long runs.

### C) What to defer until observability is fixed

1. **Do not prioritize speculative 2 ms pacing patches yet.**
   - First classify refusal bucket from decoded RFCO per-TX status and fixed
     ERR_PROTO detail decode.
   - Pacing may still be needed, but should be evidence-triggered.

2. **Do not spend cycles changing base bind settings first.**
   - Bind is already wildcard in current base runtime.
   - Prioritize target-host correctness and websocket health verification.

### D) Falsification tests (tight and fast)

1. **B1 falsification (UI host issue vs backend issue):**
   - Open UI only from reachable host.
   - Confirm `/ws/state` remains connected 5 minutes.
   - If still black with stable websocket, pivot to state/image decode path.

2. **B2 falsification (which TX gate is refusing):**
   - Run 1-fragment synthetic TX probe.
   - Capture decoded ERR_PROTO + decoded RFCO_PERTX status for each refusal.
   - If status is `INTERNAL`/scheduler gate at low load, debug profile/init.
   - If status shifts to QoS/LBT/legal-dwell only under burst, debug pacing/
     budget policy.

3. **B3 falsification (OpenOCD/SWD vs UART-sync path):**
   - Enforce holder precheck policy.
   - If OpenOCD READY still fails (`0xdeadbeef`) with clean holder state,
     treat as SWD/reset-strap problem first.
   - Only investigate AN3155 UART sync after READY is reliable.

### E) Updated execution order

1. Verify reachable base host and stable `/ws/state` session.
2. Patch ERR_PROTO decode and RFCO per-TX decode in Python tooling.
3. Run short TX ladder (1/2/4/8 fragments) and classify refusals by name.
4. Enforce serial-holder policy in flash scripts and quant wrapper.
5. Re-run 3-cycle quant gate, then 20-cycle confirmation.
6. Harden `/tmp` script deployment preflight to eliminate import confounders.

### Recommendation

Most leverage now comes from **observability and contract hardening**, not from
new transport tuning. Decode what firmware already emits (`ERR_PROTO` + RFCO),
enforce deterministic serial ownership before flash, and only then tune pacing
or queue policy based on named refusal buckets.

*Signed:* GitHub Copilot, Blocker Review v4.0 (GPT-5.3-Codex, 2026-05-26)

---

## 2026-05-26 Source-Verified Cross-Check & Sharpened Fix Plan (GitHub Copilot v5.0)

This pass cross-checks v1.1/v2.0/v3.0/v4.0 against the actual firmware
and wrapper sources, since each prior review prescribed a slightly
different first move. Three of the prior recommendations need
corrections, and one likely root cause for B2 is hiding in plain sight
in a comment in `sx1276_tx.c`.

### 1. B2 root cause is almost certainly the FHSS-scheduler-not-initialized path

`LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c`
already documents this **exact** failure mode in a comment around
`sx1276_tx_begin()` (≈ lines 152–164):

> "Under `REG_PROFILE_BENCH_ONLY_FIXED_915`... if FHSS scheduler isn't
> init'd, `sx1276_fhss_next_channel()` returns NOT_INIT,
> `sx1276_tx_begin()` refuses every TX with FORBIDDEN, and bench
> two-peer air tests can never key the radio. Root cause of the
> 2026-05-25 air_coupling_rssi_sniff false-negative
> (`rx_frames=0`, `irq_flags_or=0x00`). Decoded ERR_PROTO payload
> `10 01 08 00 00` = TX_FRAME_REQ + FORBIDDEN + detail=0."

The current camera_tx daemon TX failures (`ERR_PROTO 1001080000`
repeated, `frames_published` still growing but `reassembler_timeouts`
climbing) match this signature byte-for-byte. The most likely
mechanism right now is the **same one the bench team already
documented**: TX is being routed through the FHSS path on a profile
whose `host_cfg_profile_activate()` did **not** call
`sx1276_fhss_init()`, so the first hop request returns `NOT_INIT` and
every TX is refused. The `host_cmd.c` callers at
`sx1276_tx_begin()` (lines 462 + 533) collapse the false return into
`HOST_ERR_PROTO_FORBIDDEN` with **detail = 0**, which is exactly what
the host tooling sees.

**Implication:** v4.0 was right to deprioritize speculative pacing
patches. The cheapest disambiguator is to verify profile activation,
not to tune queue depth.

**Immediate test (5 min):**

1. Read back the active profile via cfg_get on the live tractor X8.
2. If profile is `BENCH_ONLY_FIXED_915` with `LIFETRAC_FHSS_TX_ROUTED`
   compiled in, the bypass branch should be taken — confirm by
   checking that `RFCO_PERTX` snapshots arriving on RX show
   `channel_idx = 0` and zero hop fields (the bypass path emits zeroed
   hop telemetry deliberately).
3. If profile is `FCC_15_247_FHSS_50CH_BW250` and refusals still
   appear, the scheduler init didn't run on this boot.

### 2. The "fix ERR_PROTO Python decoder" advice is necessary but not sufficient

v4.0 §B1 prescribes fixing the Python decoder to handle the 5-byte
payload. Correct, but **the firmware itself collapses every
`sx1276_tx_begin()` false return to `detail = 0`** at `host_cmd.c`
lines 462 and 533. So even with a correct Python 5-byte decoder, every
refusal will still report `FORBIDDEN/detail=0` — useless for
disambiguation.

v1.1 §"Most Likely Fixes" item 3 caught this and asked for a
firmware-side refusal enum or `sx1276_tx_last_refusal()` snapshot;
v2.0, v3.0, and v4.0 all let it drop. **Restore it.** Minimum patch:

```c
/* radio/sx1276_tx.h */
typedef enum {
    SX1276_TX_REFUSAL_NONE = 0U,
    SX1276_TX_REFUSAL_BAD_REQ,         /* req==NULL or busy */
    SX1276_TX_REFUSAL_FHSS_NOT_INIT,   /* THE current B2 suspect */
    SX1276_TX_REFUSAL_PAYLOAD_OVERFLOW,
    SX1276_TX_REFUSAL_AIRTIME_INVARIANT,
    SX1276_TX_REFUSAL_QOS_RESERVE,
    SX1276_TX_REFUSAL_LEGAL_DWELL,
    SX1276_TX_REFUSAL_LBT_BUSY,
    SX1276_TX_REFUSAL_FIFO_WRITE,
    SX1276_TX_REFUSAL_OPMODE_START,
} sx1276_tx_refusal_t;
sx1276_tx_refusal_t sx1276_tx_last_refusal(void);
```

Then in `host_cmd.c`:

```c
if (!sx1276_tx_begin(&req)) {
    host_uart_send_err_proto(HOST_PKT_TYPE_TX_FRAME_REQ, 1U,
                             HOST_ERR_PROTO_FORBIDDEN,
                             (uint16_t)sx1276_tx_last_refusal());
    return;
}
```

This is a one-evening change that converts every future B2-class
failure from a blob into a named branch. **Without it, v4.0 §B1's
decoder fix shows you the byte was zero — which you already knew.**

### 3. The "kill ttymxc3 holders before flash" advice is **also** necessary but not the actual SWD problem

The Stage1 wrapper `run_stage1_standard_quant_end_to_end.ps1` already
performs the H7→L072 SWD electrical prep (exports gpio8/10/15, holds
NRST high for ~1.5 s, see lines 222–238). It does **not** call
`fuser -k /dev/ttymxc3` before flashing. v4.0 §B3/B4 correctly
identifies the missing holder check. But the **DPIDR = 0xdeadbeef
failure mode is electrical, not UART contention** — `/dev/ttymxc3` is
the L072 UART, while SWD reads come from the H7 bit-banging SWCLK/SWDIO
GPIOs. Killing the camera daemon does not change what OpenOCD reads
on those pins.

Two distinct things must both be true:

| Layer | Required for Stage1 success | Currently in wrapper? |
|---|---|---|
| H7 SWD electrical (NRST high, SWCLK/SWDIO routed) | DPIDR returns `0x6ba02477` | Yes — but 1.5 s settle may be insufficient if a prior cycle left the L072 in deep STOP/STANDBY |
| `/dev/ttymxc3` ownership | Flasher can drive UART after OpenOCD reaches READY | **No** — wrapper does not preflight or kill holders |

The `0xdeadbeef` DPIDR in the current failing run says **the SWD layer
itself never reached the target**, full stop. fuser cleanup is good
hygiene, but it will not turn `0xdeadbeef` into `0x6ba02477`. The
likely additional cause is one of:

1. The L072 entered deep low-power mode at some prior point (camera
   daemon sent it to STOP) and NRST high alone doesn't bring SWD pads
   back up reliably; need a **BOOT0=1 + NRST cycle** to force ROM
   bootloader where SWD is always alive.
2. The 1.5 s settle is too short on cold-boot X8 images where gpio10
   was driven low for longer.
3. A prior cycle's OpenOCD instance is still alive on the H7 and is
   holding SWD pins; need `pkill -9 openocd` on the X8 before reattach.

**Recommend extending the wrapper preflight to:**

```bash
# already present: gpio export + NRST high + 1.5 s settle
# add:
pkill -9 openocd 2>/dev/null || true
fuser -k /dev/ttymxc3 2>/dev/null || true
# optional: pulse BOOT0=1 then NRST cycle for forced-bootloader mode
echo 1 > /sys/class/gpio/gpio15/value   # BOOT0 high (verify pin)
echo 0 > /sys/class/gpio/gpio10/value   # NRST low
sleep 0.05
echo 1 > /sys/class/gpio/gpio10/value   # NRST high - boots to ROM
sleep 1.5
```

If `0xdeadbeef` persists after that, the next investigation is
physical (cable, board ID, wrong adapter target).

### 4. Daemon-orchestration is the unaddressed cross-cutting blocker

Every prior review treats B1/B2/B3/B4 as independent. They are not.
The currently-running state is:

- camera_tx_daemon holds `/dev/ttymxc3` and is generating ERR_PROTO
  spam (B2).
- Stage1 quant wrapper is being relaunched while the daemon is up,
  causing fuser conflicts at the UART layer **even if SWD were fine**
  (B3 second-order cause).
- The browser tab is loaded against `192.168.1.117` which the host
  cannot reach, while the backend listens on wildcard — the operator
  has no live picture of which subsystem is actually failing (B1).

**The orchestration is missing a documented "bench mode" vs "flash
mode" switch.** The next session should not run Stage1 by hand at
all; it should run a single wrapper that:

1. `adb shell systemctl stop lifetrac-camera-tx.service lifetrac-camera-rx.service` (or `pkill -f camera_.*_daemon`).
2. Verify `fuser /dev/ttymxc3` empty AND `pgrep openocd` empty.
3. Run the GPIO/NRST/BOOT0 preflight from §3.
4. Run Stage1 cycles.
5. On completion (pass or fail), restart camera daemons.
6. Emit a single PASS/FAIL line plus an unambiguous "why" (
   `FAIL_SWD_DPIDR=0xdeadbeef`, `FAIL_HOLDER=<pid:cmd>`,
   `FAIL_SYNC_AFTER_READY`, etc.).

This is roughly 60 lines of PowerShell on top of the existing wrapper.
It permanently removes the "did I remember to stop the daemon?" class
of false negatives.

### 5. Falsification matrix tightening (B2)

v4.0 §D2 prescribes a 1/2/4/8-fragment TX ladder once decoders are
fixed. Strong. Add **one prerequisite probe** before the ladder:

> **B2-pre:** Send a single TX_FRAME_REQ with a 1-byte payload while the
> L072 is freshly booted and known-idle (no camera_tx_daemon, no RX).
> If this single TX still returns `FORBIDDEN/detail=<FHSS_NOT_INIT>`
> (after §2's enum lands), the issue is profile activation, not load.
> If it returns OK, the ladder result will be meaningful; if it
> returns refusal, the ladder is a distraction.

Without this single-shot prereq, the ladder confounds "load too high"
with "scheduler never initialized in the first place."

### 6. Updated execution order (replaces v4.0 §E)

1. **B3-electrical first** (5 min): on a clean X8, run the §3 extended
   preflight (`pkill openocd`, `fuser -k ttymxc3`, BOOT0=1 + NRST
   cycle, 1.5 s settle) and read DPIDR. If still `0xdeadbeef`,
   investigate cable/board before anything else — every later step
   depends on Stage1 being recoverable.
2. **Land the firmware refusal enum** (§2) on a dev L072 build. Without
   it, every following test still produces `FORBIDDEN/detail=0`.
3. **Patch ERR_PROTO 5-byte decode and RFCO_PERTX classifier in
   Python tooling** (v4.0 §B1/B2 unchanged).
4. **Run §5 B2-pre single-shot** under each candidate profile. Pin the
   root cause of the current TX refusals before any pacing or queue
   tuning.
5. **Add daemon-orchestration wrapper** (§4) so subsequent Stage1
   runs are no longer racing the camera daemon.
6. **Then** v4.0 §B/E steps for ladder, 3-cycle quant, 20-cycle
   confirmation, and `/tmp` import preflight.
7. UI host-target hardening (B1) — explicit `?base=` query param or a
   `<title>` banner showing `location.host` so a stale tab on
   `192.168.1.117` is obvious to the operator.

### 7. What I would NOT do next (close-outs)

- **Do not** add a 2 ms pacing sleep (v2.0 §2, v3.0 step 2) on
  speculation. The §1 evidence says the failure is a state-gate
  refusal, not a PLL settling race. If §2's refusal enum reports
  `FHSS_NOT_INIT`, pacing changes nothing.
- **Do not** flip base bind from wildcard (v2.0 §1). It is already
  `0.0.0.0`. The remaining B1 risk is the operator's browser tab, not
  the server config.
- **Do not** run more Stage1 retries against the same `0xdeadbeef`
  state. Each retry burns ~30 s plus a recovery cycle and produces no
  new information; fix §6 step 1 first.

### Final recommendation

The single highest-leverage change is **§2 — add a firmware-side
refusal enum**, because it converts the current opaque B2 symptom into
a named bucket within one bench session and validates or kills the
"FHSS scheduler not initialized" hypothesis the firmware source code
itself already flagged as a known foot-gun.

The second-highest-leverage change is **§3+§4 — extended SWD preflight
plus daemon orchestration**, because they collapse three of the four
blockers into a single deterministic wrapper. After that, the
remaining work is the v4.0 ladder and quant confirmation, which become
reliable for the first time.

*Signed:* **GitHub Copilot, Blocker Review v5.0 (2026-05-26) —
source-verified cross-check & sharpened fix plan, supersedes v4.0
where contradictions exist**

---

## 2026-05-26 Consolidated Staged Fix Plan for Next Power Cycle (GitHub Copilot v6.0)

This section is the single execution source for the next session. It resolves
the prior internal conflicts in v2.0-v5.0 and defines one deterministic order.

### A) Review Findings on Prior Revisions (Resolved Here)

1. Base binding guidance conflicted internally.
- v2.0 suggested changing base bind to `0.0.0.0`.
- v4.0/v5.0 correctly noted bind is already wildcard and stale host targeting is the likely failure.
- Resolution: do not change bind first; verify operator target host and websocket health first.

2. Pacing guidance conflicted internally.
- v3.0 proposed adding `await asyncio.sleep(0.002)` immediately.
- v4.0/v5.0 correctly deferred speculative pacing until refusal classification is fixed.
- Resolution: no pacing patch until refusal telemetry is decoded and classified.

3. SWD failure interpretation was mixed.
- Some text implied `/dev/ttymxc3` holder cleanup alone would guarantee SWD readiness.
- v5.0 correctly separates SWD electrical readiness from UART owner cleanup.
- Resolution: treat SWD preflight and UART owner cleanup as separate mandatory checks.

### B) Staged Plan (Power-Cycle Ready)

#### Stage 0: Preflight after power cycle (must pass before any tuning)
1. Confirm reachable base host and open UI from that host only.
2. Confirm `/ws/state` stays connected for 5 minutes.
3. On tractor target, stop camera/radio daemons before flash or quant steps.
4. Verify both:
  - no `/dev/ttymxc3` holders,
  - no stale `openocd` process.

Go/No-Go:
- NO-GO if websocket is not stable or SWD preflight is dirty.

#### Stage 1: SWD determinism gate (B3 first)
1. Run extended SWD preflight (NRST/BOOT strap sequence per wrapper policy).
2. Probe OpenOCD READY and DPIDR.
3. If `0xdeadbeef` persists with clean holders/processes, treat as SWD/reset/board-selection issue and stop Stage1 retries.

Go/No-Go:
- GO only when READY is repeatable.

#### Stage 2: Observability hardening (B2 disambiguation)
1. Patch host tooling to decode 5-byte ERR_PROTO payload correctly.
2. Decode `RFCO_PERTX_URC (0xC3)` in TX wait path and print refusal bucket/status.
3. Add firmware refusal detail mapping so `FORBIDDEN/detail` is no longer always zero.

Go/No-Go:
- GO only when at least one refusal is classified by named bucket.

#### Stage 3: Minimal transport falsification ladder
1. Run single-fragment synthetic TX first (low-load discriminator).
2. If single-fragment passes, run 1/2/4/8 fragment ladder.
3. Track `frags_fail`, `reassembler_timeouts`, and refusal bucket distribution.

Go/No-Go:
- If single-fragment fails with same refusal bucket, fix profile/init/state gate first.
- If only higher fragments fail, tune pacing/budget policy next.

#### Stage 4: Quant validation (only after Stages 1-3)
1. Run 3-cycle quant gate with per-cycle holder preflight logging.
2. If green, run 20-cycle quant.
3. Archive summary + raw logs for both runs.

Go/No-Go:
- NO-GO to 20-cycle if 3-cycle has unresolved SWD or unclassified refusal failures.

#### Stage 5: Tooling hardening to eliminate false negatives
1. Enforce dependency bundle or `PYTHONPATH` for `/tmp` diagnostics.
2. Add explicit preflight import checks before long RF runs.

### C) Immediate Next Session Command Priorities

1. Recover and verify base websocket health first.
2. Run SWD preflight and READY gate before any repeated quant retries.
3. Land refusal observability patches before transport tuning.
4. Execute short falsification ladder.
5. Run 3-cycle then 20-cycle quant only if prior gates are green.

### D) Decision Rule

- If any stage fails, stop and fix at that stage; do not continue downstream.
- This prevents repeating retries that add noise without adding information.

*Signed:* GitHub Copilot, Consolidated Staged Fix Plan v6.0 (2026-05-26)

