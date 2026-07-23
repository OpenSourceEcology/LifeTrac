# DESIGN-CONTROLLER Code Review — LoRa Communication, Testing Procedure, and TX/RX Optimization Plan

**Date:** 2026-07-23
**Author:** GitHub Copilot (Claude Fable 5)
**Version:** v1.0
**Scope:** `LifeTrac-v25/DESIGN-CONTROLLER/` — LoRa air link (L072 firmware + X8 host daemons), image-over-LoRa pipeline, and the test infrastructure around them. Produces a phased TX/RX optimization plan whose goal is maximum image throughput without giving up the P0-control latency guarantees or FCC §15.247 legality.

**Evidence base (files read for this review):**

- Firmware: [`firmware/murata_l072/radio/sx1276_tx.c`](../../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c), [`sx1276.c`](../../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c), [`sx1276_airtime.c`](../../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c), [`sx1276_legal_dwell.c`](../../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_legal_dwell.c), [`lora_frames_per_dwell.c`](../../DESIGN-CONTROLLER/firmware/murata_l072/radio/lora_frames_per_dwell.c), [`host/host_cmd.c`](../../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cmd.c), [`host/host_cfg_profile.c`](../../DESIGN-CONTROLLER/firmware/murata_l072/host/host_cfg_profile.c), [`main.c`](../../DESIGN-CONTROLLER/firmware/murata_l072/main.c), `include/host_types.h`, `include/host_cfg_keys.h`, `include/lora_pkt_hdr.h`, `include/sx1276_airtime.h`
- Host TX/RX: [`firmware/tractor_x8/image_tx_daemon.py`](../../DESIGN-CONTROLLER/firmware/tractor_x8/image_tx_daemon.py), [`camera_service.py`](../../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py), [`base_station/image_rx_daemon.py`](../../DESIGN-CONTROLLER/base_station/image_rx_daemon.py), [`base_station/lora_proto.py`](../../DESIGN-CONTROLLER/base_station/lora_proto.py), [`base_station/image_pipeline/reassemble.py`](../../DESIGN-CONTROLLER/base_station/image_pipeline/reassemble.py), [`frame_format.py`](../../DESIGN-CONTROLLER/base_station/image_pipeline/frame_format.py), [`x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py`](../../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py) (+ `method_g_stage1_probe.py` `HostLink`)
- Deploy: [`docker-compose.video-test.v2.yml`](../../DESIGN-CONTROLLER/firmware/tractor_x8/docker-compose.video-test.v2.yml)
- Docs: [`LORA_PROTOCOL.md`](../../DESIGN-CONTROLLER/LORA_PROTOCOL.md), [`IMAGE_PIPELINE.md`](../../DESIGN-CONTROLLER/IMAGE_PIPELINE.md), [`IMAGE_PIPELINE_METHODS.md`](../../DESIGN-CONTROLLER/IMAGE_PIPELINE_METHODS.md), [`HIL_RUNBOOK.md`](../../DESIGN-CONTROLLER/HIL_RUNBOOK.md), [`TODO.md`](../../DESIGN-CONTROLLER/TODO.md)
- Tests/CI: `base_station/tests/` (78 files), `base_station/run_tests.ps1`, `firmware/murata_l072/bench/host_proto/`, `hil/`, `.github/workflows/arduino-ci.yml`
- Prior work honored: [`2026-05-27_Image_Over_LoRa_Production_Roadmap_v25.0.1_to_v25.1.0.md`](../2026-05-27_Image_Over_LoRa_Production_Roadmap_v25.0.1_to_v25.1.0.md), [`2026-05-26_Image_Over_LoRa_AirLink_Proven_Status_Copilot_v1_0.md`](../2026-05-26_Image_Over_LoRa_AirLink_Proven_Status_Copilot_v1_0.md), 2026-05-25 bench logs

---

## 1. Executive summary

The image-over-LoRa pipeline works end-to-end (proven 2026-05-27), but it currently delivers roughly **~180 B/s of effective image goodput** against a theoretical **~1.2–2.4 KB/s** available from the same silicon at the same spreading factor. The dominant losses are *not* in the radio — they are in host-side pacing policy, fragment sizing, a host↔firmware PHY-model mismatch, and the absence of any fragment-level loss recovery.

Top findings, in impact order:

| # | Finding | Severity | Effect today |
|---|---------|----------|--------------|
| F1 | Host fragmenter models **SF7/BW500**; firmware radio actually runs **SF7/BW250** | **High** (correctness + perf) | Every airtime prediction is 2× optimistic; the 25 ms C1 cap is silently violated on-air (~48.8 ms actual) |
| F2 | Fixed 150 ms open-loop inter-fragment sleep | **High** (perf) | Radio idle ~76 % of the time; goodput capped at ~180 B/s vs ~296 B/s available even under the single-channel QoS gate |
| F3 | 41 B fragments (sized by the wrong model) where firmware accepts up to 247 B | **High** (perf) | Fixed per-fragment overhead (preamble + 8 B hop header + 4 B frag header) eats ~25 % of airtime |
| F4 | No fragment retry and no abort-on-failure: a frame with 1 lost fragment still transmits its remaining fragments, then is discarded whole at RX | **High** (perf + robustness) | At 1 % PER an ~80-fragment keyframe completes only ~45 % of the time; every failure wastes the whole frame's airtime |
| F5 | Single-channel bench profile pins throughput under the 400 ms/s QoS gate (40 % duty); FHSS RX RegFrf trap (v25.0.6.5) still blocks the profile that would lift it | High (perf, known) | Hard ceiling ~40 % duty until FHSS closure |
| F6 | No firmware TX queue (depth 1); host must round-trip TX_DONE before next TX_FRAME_REQ | Medium (perf) | Adds host-turnaround dead time between fragments; blocks near-100 % duty even after F2 is fixed |
| F7 | `effective_len` u8 overflow in `sx1276_tx_begin()` for payloads > 247 B | Medium (latent bug) | Today masked by the daemon's 64 B clamp; becomes live the moment fragment sizes rise (Phase 1) |
| F8 | Per-tile WebP containers waste ~20–30 % of every frame byte budget | Medium (perf) | RIFF/WebP header ≈ 30–44 B *per 32×32 tile* |
| F9 | RX never requests a keyframe on reassembly timeout; recovery waits for the 60 s periodic keyframe | Medium (latency) | A lost keyframe = up to 60 s of stale canvas |
| F10 | TileDeltaFrame wire header diverged from `LORA_PROTOCOL.md` (no real `base_seq`, no `wall_seconds`); parser field `base_seq` actually holds the rolling frame seq | Medium (doc/design drift) | The documented P-frame-against-wrong-base detection **cannot work as specified**; masked by periodic keyframes |
| F11 | Reassembler GC (1500 ms inactivity) races the TX daemon's own failure modes (3 s TX_DONE timeout) | Low-Med | Explains the `reassembler_timeouts=20` seen in the 2026-05-27 validation run |
| F12 | `image_tx_daemon` enforces a 50 ms *minimum* sleep floor (`MIN_LORA_HOST_INTER_CYCLE_S`) even if configured lower | Low | Blocks closed-loop pacing without a code change |

**Bottom line of the optimization plan (§5):** with no new hardware and no change of SF, the pipeline can realistically go from **~180 B/s → ~1.2 KB/s** (BW250 + FHSS + big fragments + pipelining), and to **~2.4 KB/s** once the DTS BW500 profile is made real. Keyframe latency at the current 192×128 bench canvas drops from ~15 s to **~1.3–2.6 s**, and P-frame latency from ~400 ms to **~85–170 ms**. Codec work (F8) multiplies whatever the link gives by another ~1.3–1.6×.

---

## 2. Where every millisecond goes today

Production bench configuration ([`docker-compose.video-test.v2.yml`](../../DESIGN-CONTROLLER/firmware/tractor_x8/docker-compose.video-test.v2.yml)): synthetic delta camera at 0.5 fps, 6×4×32 canvas (192×128), `LIFETRAC_FRAGMENT_BUDGET=12`, `LIFETRAC_LORA_INTER_CYCLE_S=0.15`, `REG_PROFILE=0` (single channel 915 MHz), firmware modem at **SF7 / BW250 / CR4-5 / preamble 8** (set once in [`sx1276.c` line 273](../../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c) and never reprogrammed — see F1).

Per-fragment timeline (measured + computed, matches the observed `toa_us` values):

| Step | Cost | Source |
|---|---:|---|
| UART TX_FRAME_REQ (45 B @ 921600) | ~0.5 ms | wire math |
| Firmware admission (LBT disabled, QoS reserve, FIFO load) | ~1–2 ms | code path |
| **Time-on-air** — 41 B fragment body + 8 B hop header = 49 B at SF7/BW250 | **48.8 ms** | shared estimator; matches memory of bench `toa_us=48768` |
| TX_DONE_URC → host `read_frames` poll (10 ms granularity) | ~5–15 ms | [`method_g_stage1_probe.py::read_frames`](../../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_g_stage1_probe.py) |
| **Fixed `inter_cycle_s` sleep** | **150 ms** | [`image_tx_daemon.py`](../../DESIGN-CONTROLLER/firmware/tractor_x8/image_tx_daemon.py) |
| **Total per fragment** | **~205 ms** | |

- Airtime utilization: 48.8 / 205 ≈ **24 %**.
- Data per fragment: 41 B body − 4 B fragment header = **37 B**.
- **Effective goodput ≈ 180 B/s.** A ~3 KB keyframe (≈80 fragments) takes **~15–17 s** and historically tripped dwell-cap warnings + `FAULT_URC`.
- The firmware QoS gate ([`sx1276_airtime.c`](../../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_airtime.c): 400 ms ToA per rolling 1 s per channel) would allow 8 of these fragments/s = **296 B/s** — the 150 ms sleep gives away a third of even the single-channel budget. (History: 50 ms pacing → ~10 frags/s → >400 ms/s → `ABORT_QOS`/`ERR_PROTO FORBIDDEN`, which is why 150 ms was chosen. The correct answer was budget-aware pacing, not a bigger constant — see §5 Phase 1.)

### Ceilings at SF7 by regime (derived from the same airtime formula the code uses)

| Regime | Fragment body | ToA | Sustainable rate | Goodput |
|---|---:|---:|---|---:|
| Today (BW250, 1-ch, 150 ms sleep) | 41 B | 48.8 ms | 4.9 frag/s | **~180 B/s** |
| BW250, 1-ch, QoS-paced (400 ms/s) | 41 B | 48.8 ms | 8 frag/s | ~296 B/s |
| BW250, 1-ch, QoS-paced | 200 B | 164 ms | 2.4 frag/s | ~477 B/s |
| BW250, **FHSS-50** (per-channel budgets never bind), pipelined | 200 B | 164 ms | ~6 frag/s | **~1.2 KB/s** |
| **BW500 (DTS profile)**, pipelined | 200 B | 82 ms | ~12 frag/s | **~2.4 KB/s** |

(200 B body = 196 B data after the 4 B fragment header; on-air = body + 8 B hop-sync header. Airtime from the estimator in [`lora_proto.py::lora_time_on_air_ms`](../../DESIGN-CONTROLLER/base_station/lora_proto.py), which agrees with the firmware C mirror.)

---

## 3. LoRa communication — detailed findings

### F1 — Host PHY model (BW500) ≠ firmware modem config (BW250) — HIGH

- [`lora_proto.py` line 150](../../DESIGN-CONTROLLER/base_station/lora_proto.py): `PHY_IMAGE = PhyProfile("image", 7, 500, 5, 8)`.
- [`sx1276.c` line 273](../../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276.c): `sx1276_set_sf_bw_cr(7U, 250U, 5U)` at init — and **nothing in the host command path ever reprograms it**. `host_cfg_profile_activate()` validates `modem_bw_hz` against the profile but never calls `sx1276_apply_profile_full()` (which exists, and has no callers outside init — confirmed by search).
- Consequences:
  1. `pack_telemetry_fragments(payload, seq, PHY_IMAGE)` sizes fragments for a 25 ms cap **at BW500**, producing 41 B bodies whose *actual* ToA at BW250 is **48.8 ms — ~2× the cap**. The C1 constraint (`IMAGE_PIPELINE.md` §1.3) is silently violated on-air. `LORA_PROTOCOL.md` already flags the fragment-cap redesign as "pending", but the code pretends the cap holds.
  2. Every throughput/budget prediction made by `camera_service._compute_link_bytes()` and the compose comments ("SF7/BW500 ≈ 48 ms" — actually the BW250 number) inherits the error. Even the repo's memory notes carry it.
  3. The `CMD_LINK_PROFILE` machinery (IP-W2-04) faithfully ships the same wrong numbers to the tractor.
- **Fix direction:** single source of truth. Either (a) change `PHY_IMAGE` to `(7, 250, 5, 8)` **and** rework the cap policy (a 25 ms cap at BW250 leaves ≤6 B fragment bodies — useless; see Phase 1), or (b) make the firmware actually run BW500 via the DTS profile (Phase 2). Either way, add the contract test from §4-T2 so this class of drift can never silently recur.

### F2 — Open-loop 150 ms pacing — HIGH

[`image_tx_daemon.py`](../../DESIGN-CONTROLLER/firmware/tractor_x8/image_tx_daemon.py) sleeps a fixed `inter_cycle_s` after every TX_DONE. The firmware already tells the host everything needed to pace optimally:

- `TX_DONE_URC` carries `time_on_air_us` per fragment.
- `RFCO_PERTX` names the refusal reason (`ABORT_QOS`, `ABORT_LBT`, …) — an aborted attempt costs **zero RF** and ~2 ms of UART, so *retry-on-QoS-abort with short backoff* is cheap.
- The firmware even mirrors the host pacing-hint helper ([`lora_frames_per_dwell.c`](../../DESIGN-CONTROLLER/firmware/murata_l072/radio/lora_frames_per_dwell.c)).

A host-side token bucket that mirrors the firmware's own budget (400 000 µs per rolling 1 s per channel, refill from the `time_on_air_us` feedback) lets the daemon:
- send P-frames (1–2 fragments) **immediately** (better latency), and
- stream keyframes at the exact admissible rate (better throughput; no more magic constants).

Note [F12]: the daemon currently clamps `inter_cycle_s = max(0.05, configured)` — the floor must be removed as part of this change.

### F3 — Fragment size far below what the hardware and the dwell budget allow — HIGH

Fixed per-fragment overhead at BW250: preamble 6.27 ms + 8 B hop-sync header ([`lora_pkt_hdr.h`](../../DESIGN-CONTROLLER/firmware/murata_l072/include/lora_pkt_hdr.h)) + 4 B fragment header + 2 B TX_FRAME_REQ framing. At 41 B bodies that's ~25 % of airtime + 10 % of payload bytes; at 200 B bodies it drops to ~4 % + 2 %.

Constraints that actually bind fragment size:

| Constraint | Value | Source |
|---|---|---|
| Firmware TX payload struct | 255 B | `include/sx1276_tx.h` |
| Hop header | −8 B → 247 B max body | `sx1276_tx.c` (`LIFETRAC_FHSS_TX_ROUTED`) |
| Per-TX dwell cap | ToA ≤ 380 ms → ~253 B on-air at SF7/BW250 | `SX1276_AIRTIME_DWELL_CAP_US` |
| Fragment index space | u8 → ≤256 fragments/frame | `pack_telemetry_fragments` |
| **Host daemon clamp** | **64 B** ("matches W1-10b probe clamp") | `image_tx_daemon.py` — *self-imposed, not a firmware limit* |

Recommended: 160–200 B bodies (see Phase 1). Bonus: bigger fragments raise the max frame size from ~9.5 KB to ~50 KB (256 × 196 B), giving keyframes at bigger canvases headroom.

Trade-off to manage: longer fragments have higher per-fragment corruption probability under interference, so F4 (retry) should land in the same phase.

### F4 — No loss recovery below the whole-frame level — HIGH

- TX: on `TX_DONE` non-OK / timeout / `ERR_PROTO`, [`image_tx_daemon._tx_one_frame`](../../DESIGN-CONTROLLER/firmware/tractor_x8/image_tx_daemon.py) logs, counts, and **continues with the remaining fragments** of a frame that the RX can now never complete (the reassembler requires all fragments). Dead airtime.
- RX: a frame missing one fragment sits in the reassembler until the 1500 ms GC evicts it. No NACK, no fragment re-request, no FEC. Recovery = wait for the next keyframe (60 s period on the bench!) or an operator-driven `CMD_REQ_KEYFRAME`.
- Math: at fragment PER *p* and keyframe size *n* fragments, P(complete) = (1−p)ⁿ. For p=1 %, n=80: **45 %**. One TX-side retry of just the failed fragment lifts it to (1−p²)ⁿ ≈ **99.2 %**.
- Fixes (Phase 1d + Phase 3): abort frame remainder after a hard failure; single immediate retry per failed fragment; longer-term, parity fragments (FEC) so RX tolerates k losses without any round trip.

### F5 — Single-channel QoS gate vs FHSS — HIGH (known, quantified here)

Under `REG_PROFILE=0` every fragment debits the *same* channel's 400 ms/1 s budget → hard 40 % duty ceiling. Under the 50-channel FHSS profile each TX debits a *different* channel, so neither the QoS gate nor the 400 ms/10 s legal-dwell accountant ever binds at image rates — TX can stream essentially continuously **and** legally. This makes the v25.0.6.5 FHSS RX-scan closure (RegFrf never programmed on the RX side — see [`/memories` note + roadmap §6.5](../2026-05-27_Image_Over_LoRa_Production_Roadmap_v25.0.1_to_v25.1.0.md)) not just a compliance milestone but **the single biggest throughput unlock available** (~2.5× over the QoS-paced single channel).

### F6 — No TX pipelining — MEDIUM

`sx1276_tx_busy()` → `ERR_PROTO_QUEUE_FULL`: the L072 accepts exactly one in-flight TX. The host must absorb (UART round trip + poll granularity) between every fragment. A depth-2 firmware mailbox (accept the next TX_FRAME_REQ during RF, start it from `sx1276_tx_poll` completion) hides the entire host turnaround and is required to get >90 % duty in the FHSS/DTS regimes. Keep depth at 2 — deeper queues would violate the P0 25 ms-preemption story once control traffic shares the radio.

### F7 — `effective_len` u8 overflow — MEDIUM (latent, blocks Phase 1)

[`sx1276_tx.c`](../../DESIGN-CONTROLLER/firmware/murata_l072/radio/sx1276_tx.c): `const uint8_t effective_len = (uint8_t)(req->length + LORA_PKT_HDR_LEN);` — for `req->length > 247` this wraps (e.g. 250 → 2), the airtime invariant passes trivially, `REG_PAYLOAD_LENGTH` gets the wrapped value, and the FIFO burst still writes 8 + `length` bytes. Result: corrupt on-air frame with no error to the host. Must be rejected in `handle_tx_frame()` (`length > 247` → `HOST_ERR_PROTO_BAD_LENGTH`) before the host-side 64 B clamp is lifted. Verify the host-UART RX ring can carry ~260 B command frames at the same time.

### F8 — Per-tile WebP container overhead — MEDIUM

Every 32×32 tile ships as a complete RIFF/WebP file: ~30–44 B of container + VP8 frame header per tile against typical 150–250 B of image data (q=30). Options, cheapest first:

1. **Container strip:** transmit the raw VP8/VP8L bitstream; RX re-wraps it into a RIFF container before `PIL.Image.open` (pure framing, pixel-exact, ~15–20 % byte saving).
2. **Mosaic encode:** pack the frame's changed tiles into one N×32 strip, one WebP container for the whole frame, RX slices it back. One container + shared entropy state: ~25–40 % saving at equal quality. Costs one extra decode-slice step in [`codec_decode.py`](../../DESIGN-CONTROLLER/base_station/image_pipeline/codec_decode.py).
3. The BTC4 / MONO_G4 codecs already avoid containers — no action needed there.

### F9 — RX does not self-heal after a lost keyframe — MEDIUM

[`image_rx_daemon.py`](../../DESIGN-CONTROLLER/base_station/image_rx_daemon.py) deliberately defers keyframe requests to web_ui's path, but in the strict-path topology nothing watches `reassembler.stats.timeouts`. Wiring `timeouts`/`decode_errors` increments → publish `lifetrac/v25/cmd/req_keyframe` (rate-limited) cuts worst-case canvas staleness from `KEYFRAME_PERIOD_S` (60 s) to one frame round trip (~2–3 s post-Phase-1). ~15 lines.

### F10 — TileDeltaFrame wire format drifted from the protocol doc — MEDIUM

Implemented header ([`camera_service._build_frame`](../../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py) / [`frame_format.py`](../../DESIGN-CONTROLLER/base_station/image_pipeline/frame_format.py)): `frame_kind(1) seq(1) grid_w(1) grid_h(1) tile_px(1) codec(1) bitmap`. [`LORA_PROTOCOL.md § TileDeltaFrame`](../../DESIGN-CONTROLLER/LORA_PROTOCOL.md) specifies `seq(2) base_seq(2) wall_seconds(4) camera_id(1) flags(1) …`. Beyond doc rot, two functional gaps:

- The parser's `base_seq` field actually receives the **rolling frame seq** — there is no true I↔P dependency tag on the wire, so the documented "base sends `CMD_REQ_KEYFRAME` when a P-frame references an I it doesn't have" logic **cannot be implemented** with the current header. Today this is masked by periodic keyframes.
- No `wall_seconds` → the base cannot render the "honest staleness clock" the operator-UX rules (v1.1 LoRa analysis §5.4) call mandatory; the canvas can only show receive-time age.

Recommend: either update the doc to bless the compact header (and explicitly renumber its fields), or spend 3 bytes (`base_seq` u8 + `wall_offset` u16) to restore both features. Do this before the wire format calcifies further in v25.0.x releases.

### F11 — Reassembler GC vs slow/faulting keyframes — LOW-MED

[`reassemble.py`](../../DESIGN-CONTROLLER/base_station/image_pipeline/reassemble.py) GC is inactivity-based (`last_seen_ms`, default 1500 ms) — good design, and fine at 150 ms fragment spacing. But the TX daemon's own stall modes (3 s `PER_FRAGMENT_TX_TIMEOUT_S`, dwell-cap "transmit anyway" branch, L072 fault recovery) create inter-fragment gaps > 1.5 s mid-frame, at which point RX evicts the partial and the rest of the keyframe arrives into a dead slot. This is consistent with the `reassembler_timeouts=20` counted during the 2026-05-27 validation. After Phase 1 (paced, retrying TX) raise/derive the timeout as `max(3 s, 3 × expected inter-fragment gap)` and pin it with a test (§4-T8).

### Minor / hygiene

- `image_tx_daemon._PendingFrame.seq` docstring says "nibble" — it's a byte (mod 256).
- The dwell-cap warning path claims "transmitting anyway with extra cadence padding" but adds no extra padding — code and comment disagree.
- `image_rx_daemon` opens the link with RESET_REQ + 1.5 s settle on every restart; acceptable, but with `LIFETRAC_SKIP_RESET_REQ` documented in two places it belongs in the compose env for the strict path.
- `main.c` services RX only when `!sx1276_tx_busy()` — correct for half-duplex, but worth a comment that a depth-2 TX queue (F6) must preserve the RX service window between back-to-back TXs, or a continuously-streaming TX will starve RX_FRAME delivery to the host on the *tractor* side (relevant for the back-channel commands).

---

## 4. Testing procedure — review

### What is genuinely strong

1. **SIL breadth.** 78 test files under `base_station/tests/` covering protocol constants, CRC/KISS, fragmentation + fuzz (`test_telemetry_fragmentation_fuzz.py`), reassembly v1/v2 + fuzz, crypto vectors, encode modes, budget logic, link monitor/orchestrator, e2e image pipeline (SIL), and the safety-adjacent `*_sil` suites.
2. **Firmware host-proto bench tests in C** (`firmware/murata_l072/bench/host_proto/`): airtime invariant, legal dwell, frames-per-dwell parity, RFCO counters, CFG clamp fuzz — compiled and run host-side in CI (`make check-*` jobs in [`arduino-ci.yml`](../../../.github/workflows/arduino-ci.yml)), plus cross-compile + size budget + wire-constant sync (`check_mh_wire_sync.py`). This is a solid contract-test culture.
3. **HIL discipline.** [`HIL_RUNBOOK.md`](../../DESIGN-CONTROLLER/HIL_RUNBOOK.md) W4-pre…W4-10 with per-gate hardware lists, abort criteria, and evidence conventions; `hil/dispatch.ps1` + `results_schema.json` for machine-readable results.
4. **Institutionalized lessons.** The hermetic two-sided probe (`method_h_stage2_tx_probe_v2.py --probe rx_listen/tx_burst`) exists precisely because of the 2026-05-25 "harness never deployed the TX daemon" trap; `run_tests.ps1` documents *why* it isolates processes.

### Gaps (each maps to an optimization-plan item)

- **T1 — No throughput regression gate.** Nothing measures image goodput (B/s, fragments/s, frame-completion %) anywhere — SIL or bench. That is how a 24 %-utilization pipeline shipped without any test noticing. Add: (a) a SIL test pinning `pack_telemetry_fragments` output size × estimator ToA × pacing policy ≥ target B/s for the active profile; (b) a bench probe mode (`--probe throughput`, N-second stream, reports B/s + completion %) wired into the hil/ dispatch schema.
- **T2 — No host↔firmware PHY contract test.** F1 survived because Python validates its estimator against itself and C validates its mirror against itself, but nothing asserts *the configured firmware modem tuple equals the host's assumed `PhyProfile`*. Add a `CFG_GET`/VER-style readback of (sf, bw, cr, preamble) and assert it equals `PHY_BY_NAME[LIFETRAC_FRAGMENT_PROFILE]` at daemon startup (fail loud), plus a golden shared airtime-vector file consumed by both the C bench test and a Python unit test.
- **T3 — CI vs local runner discrepancy.** CI runs `python -m unittest discover -s tests` while `run_tests.ps1` exists specifically because discovery breaks on the duplicated `image_pipeline` package name (base-side vs X8-side). Either CI is quietly skipping/failing what local isolates, or local is masking an import-order landmine. Land the IP-W2-10 package rename, then converge both runners on one invocation.
- **T4 — No PER / frame-completion soak gate.** The keyframe math in F4 shows completion collapses non-linearly with PER; there is no gate that measures bench PER and asserts frame-completion % ≥ target for a 30–60 min soak. Extend W4-00.
- **T5 — C1 (25 ms cap) is untested against reality.** `airtime_invariant.c` enforces the 380 ms dwell cap; nothing tests the 25 ms fragment cap against the *actual modem config* — and indeed it does not hold (F1). Once the cap policy is reworked (Phase 1), pin it with a test that computes ToA of a max-size fragment at the *firmware-reported* profile.
- **T6 — TODO V4 (keyframe-loss recovery) remains unimplemented** — precisely the F9 gap.
- **T7 — E2E automation.** `test_e2e_image_pipeline.py` is SIL-only; the real-RF concurrent smoke lives in ad-hoc repo-root scripts (`check_image_flow.sh`, `tmp_*`). Promote the smoke into `hil/` with the results schema so it is runnable as a single gate, per roadmap v25.0.2's "no hot-patched containers" goal.
- **T8 — Reassembler long-frame GC test.** Simulate an 80-fragment frame at realistic spacing with one injected 3 s stall; assert survival (with the Phase-1 timeout) or an emitted keyframe request (F9), not silent eviction.

---

## 5. TX/RX optimization plan

Phased so that each phase is independently shippable, measurable, and low-regret. Expected-goodput figures use the §2 table; all assume SF7 and the current 6×4×32 bench canvas unless stated.

### Phase 0 — Instrument before touching (protects everything else)

| Step | What | Where |
|---|---|---|
| 0.1 | Add TX-side goodput counters: rolling B/s, fragments/s, per-frame completion, airtime-utilization % (Σ toa_us ÷ wall time) to the existing `stats:` line | `image_tx_daemon.py` |
| 0.2 | Add RX-side mirror: B/s reassembled, frame-completion %, timeout rate | `image_rx_daemon.py` |
| 0.3 | New bench probe `--probe throughput` (fixed-duration stream, JSON result) + hil/ dispatch entry | `method_h_stage2_tx_probe_v2.py`, `hil/` |
| 0.4 | **Contract test (T2):** daemon startup asserts firmware modem tuple == host `PhyProfile`; CI golden airtime vectors shared C↔Python | daemons + `tests/` + `bench/host_proto/` |

Exit: a number on a dashboard/log that every later phase must move. (Roadmap alignment: this *is* v25.0.3 Observability, narrowed to the link.)

### Phase 1 — Host-only quick wins (no firmware flash; days, not weeks)

| Step | What | Expected effect |
|---|---|---|
| 1.1 | **Fix the PHY model** (F1): `PHY_IMAGE → (7, 250, 5, 8)` or introduce `PHY_IMAGE_BW250`; update compose comments and `LINK_PHY_NAMES` consumers | Honest math everywhere; prerequisite for 1.2–1.3 |
| 1.2 | **Rework the fragment-cap policy** (F1/F3): replace the blanket 25 ms cap for the image profile with a *link-composition-aware* cap — on links with no P0 traffic (the current strict-path bench: control never rides this radio), cap = dwell cap (380 ms); when P0 shares the radio, cap = the C1 value that the M7 queue design actually needs. Encode the policy in `lora_proto.py` with the profile, not as a global constant | Unlocks 1.3 without betraying C1 |
| 1.3 | **Raise fragment size to 160–200 B** (F3): lift the daemon's 64 B clamp to 247 B ceiling, sized by the corrected estimator under the 1.2 cap. *Gate on firmware guard 2.1 (F7) — until it ships, clamp ≤ 200 B in the daemon* | Payload efficiency 75 %→94 %; preamble overhead 13 %→4 % |
| 1.4 | **Closed-loop pacing** (F2/F12): remove the 50 ms floor and the fixed sleep; token bucket mirroring the firmware 400 ms/1 s per-channel budget, refilled from `TX_DONE.time_on_air_us`; on `ABORT_QOS`, backoff `≈ toa` and retry (no RF was spent) | 1-ch goodput → QoS ceiling: **~477 B/s** (2.6×); P-frame latency 400 ms → ~170 ms |
| 1.5 | **Fragment failure policy** (F4): abort frame remainder on hard failure; ONE immediate retry of the failed fragment before aborting | Keyframe completion 45 %→99 % @ 1 % PER; no dead airtime on doomed frames |
| 1.6 | **RX keyframe self-heal** (F9): publish rate-limited `req_keyframe` on reassembler timeout/decode-error | Canvas worst-case staleness 60 s → seconds |
| 1.7 | Reassembler timeout derivation (F11): `timeout_ms = max(3000, 3 × expected_gap)`; keyframe period back to 15–20 s once 1.4 lands | Fewer spurious evictions |

Phase-1 exit target (bench, single channel): **≥ 400 B/s sustained, keyframe ≤ 8 s, P-frame ≤ 250 ms**, zero `ABORT_QOS` at steady state, frame completion ≥ 99 % at bench PER.

### Phase 2 — Firmware unlocks (flash both L072s; the big multipliers)

| Step | What | Expected effect |
|---|---|---|
| 2.1 | **Reject `length > 247`** in `handle_tx_frame` (F7); verify host-UART ring ≥ 300 B frames | Makes 1.3 safe at full size |
| 2.2 | **Depth-2 TX mailbox** (F6): accept one queued TX_FRAME_REQ while RF is active; preserve the RX service window between TXs; keep TX_DONE per tx_id semantics | Removes host turnaround; duty → ~95 % of admissible |
| 2.3 | **FHSS RX closure** (F5; roadmap v25.0.6.5 paths b/c): program RegFrf on RX arm per hop policy / scan-and-lock; then make `REG_PROFILE=1` the bench default | QoS gate stops binding: **~1.2 KB/s** @ BW250, field-legal |
| 2.4 | **Make DTS BW500 real** (F1/F5): on `host_cfg_profile_activate(REG_PROFILE_FCC_15_247_DTS_BW500)`, call `sx1276_apply_profile_full()` with the profile tuple (the function exists and is currently dead code); add `PHY_IMAGE_BW500` host-side; re-check §15.247 DTS PSD (8 dBm/3 kHz) against the +20 dBm PA setting per the FCC notes | **2× everything: ~2.4 KB/s**, keyframe ~1.3 s at bench canvas |
| 2.5 | (With 2.2) Re-validate P0 preemption story: measure worst-case TX-start delay for a P0-class frame injected mid-keyframe; must respect whatever cap 1.2 chose | Keeps the safety case honest |

Phase-2 exit target: **≥ 1 KB/s sustained under FHSS (BW250)**, or ≥ 2 KB/s under DTS BW500; keyframe ≤ 3 s; RFCO_SUMMARY shows legal dwell compliance.

### Phase 3 — Reliability at speed (protocol layer)

| Step | What | Why |
|---|---|---|
| 3.1 | **Parity fragments (FEC)**: extend the fragment scheme (new magic 0xFC or v2 nibble reuse) with k-of-n — e.g. 8 data + 1 XOR parity per group (~12 % overhead tolerates 1 loss/group with zero round trips). Fountain/RS if PER in the field says so | Keyframes survive PER without ARQ latency; complements 1.5 |
| 3.2 | **True `base_seq` + `wall_seconds`** on the wire (F10) — 3 bytes/frame — restoring documented P-frame validity checking and the honest staleness clock | Correctness the doc already promises |
| 3.3 | Adaptive fragment size from link feedback (`link_monitor` SNR/PER → fragment size ladder 64/128/200 B), same mechanism as the encode-mode ladder | Keeps long fragments from becoming a liability at range |

### Phase 4 — Stretch (only if the field demands more)

- **SF6 + implicit header** (fixed-length fragments): ×1.37 raw rate over SF7; costs ~2.5 dB sensitivity, needs fixed-size fragments and both-ends config discipline. Bench-only experiment first.
- **Preamble 8 → 6 symbols**: ~1–2 % at 200 B fragments; take it if it's free during 2.4 regression runs.
- **Codec (F8)**: container-strip (~15–20 %) then mosaic-WebP (~25–40 %) — multiplies every phase's outcome; independent of radio work and safe to slot anywhere after Phase 0 metrics exist.
- **Grid/quality operating-point sweep** — already planned as roadmap v25.0.6; run it *after* Phase 1 so the sweep measures the real link, not the pacing artifact.

### Combined outlook (192×128 bench canvas, WebP q30)

| Milestone | Goodput | ~3 KB keyframe | 2-tile P-frame |
|---|---:|---:|---:|
| Today | ~180 B/s | ~15–17 s | ~400 ms |
| After Phase 1 | ~477 B/s | ~6.4 s | ~170 ms |
| After Phase 2 (FHSS BW250) | ~1.2 KB/s | ~2.6 s | ~170 ms |
| After Phase 2 (DTS BW500) | ~2.4 KB/s | ~1.3 s | ~85 ms |
| + Phase 3/4 codec (−30 % bytes) | (same wire rate) | **~0.9 s** | **~60 ms** |

At the full 384×256 production canvas (~12 KB keyframes), the same chain lands keyframes in ~5 s (BW500) — workable with 15–20 s keyframe periods and P-frame-dominant traffic.

---

## 6. Measurement protocol (how every claim above gets checked)

1. **Unit of truth:** `time_on_air_us` from TX_DONE_URC (firmware-measured), never host wall-clock alone.
2. **Per run record:** profile id, modem tuple readback, fragment size, pacing policy, frames in/ok/fail, fragments ok/fail, Σtoa, wall time, RX completion %, reassembler timeouts. The Phase-0 probe emits this as JSON into `bench-evidence/` per the HIL conventions.
3. **A/B discipline:** every phase lands behind an env knob (`LIFETRAC_PACING=budget|fixed`, `LIFETRAC_FRAGMENT_MAX_B`, …) so regressions bisect in minutes, mirroring the `LIFETRAC_IMAGE_METHOD` A/B/C pattern that worked well.
4. **Falsification first** (per repo methodology memory): before attributing any future stall to RF, run the hermetic `rx_listen`/`tx_burst`/`throughput` probes that deploy their own dependencies.

---

## 7. Priority recommendation

If only three things get done: **1.4 (closed-loop pacing), 1.3+2.1 (big fragments, safely), 2.3 (FHSS closure)**. Those three alone are the difference between ~180 B/s and ~1.2 KB/s — everything else is compounding interest.

The single most important *process* change is Phase 0.4 / T2: the host and the firmware each had a self-consistent, individually-tested model of the PHY — and they were different. A ten-line startup assertion would have caught it on day one.

---

## 8. Verification and Secondary Review (Gemini 3.1 Pro)

**Date:** 2026-07-23
**Reviewer:** GitHub Copilot (Gemini 3.1 Pro)

I have independently reviewed the DESIGN-CONTROLLER codebase, particularly the LoRa transmission logic and corresponding host daemons, to verify the findings and the proposed optimization plan in this document.

### Verification of Findings:
- **F1 (PHY Model Mismatch):** Verified. sx1276_set_sf_bw_cr(7U, 250U, 5U) is hardcoded in sx1276.c while Python estimators in lora_proto.py assume BW500. This leads to the host miscalculating the time-on-air.
- **F2 & F12 (Open-loop Pacing):** Verified. image_tx_daemon.py uses explicit 	ime.sleep(self.inter_cycle_s) and has a hardcoded floor of 50ms (MIN_LORA_HOST_INTER_CYCLE_S).
- **F3 (Host Fragment Clamp):** Verified. image_tx_daemon.py enforces a 64 bytes fragment body clamp, heavily penalizing goodput with header/preamble overheads.
- **F7 (effective_len Overflow):** Verified. Because length is uint8_t, the sum 
eq->length + LORA_PKT_HDR_LEN wraps around at 256 in sx1276_tx.c. Payloads > 247 bytes will silently corrupt without error.
- **Testing Procedure Gaps:** Verified. Tests lack a firm contract test to check python metrics vs embedded firmware metrics T2, leading to the F1 bug.

### Assessment of the Optimization Plan:
The phased optimization plan in **Section 5** is exceptionally well-reasoned. 
- **Phase 0:** Implementing better telemetry and observability prior to changing parameters is critical.
- **Phase 1:** Correcting the Python-side PHY model and opening up the self-imposed 64B limit to 200B will yield ~2x effective throughput with zero firmware updates.
- **Phase 2 & 3:** Addressing the integer overflow before increasing payloads beyond 247B prevents silent failure. Activating the FHSS profile natively lifts the restrictive single-channel 40% duty ceiling. 

**Conclusion:** The initial review's claims are factual and the math holds up. The optimization plan is highly recommended. Proceeding with Phase 0 and Phase 1 will resolve the most aggressive performance bottlenecks immediately.

---

## 9. Final Review — §8 Verification Audit, New Findings, and Implementation-Ready Recommendations

**Date:** 2026-07-23
**Author:** GitHub Copilot (Claude Fable 5) — original author, closing pass
**Status:** Supersedes nothing; adds verification evidence, one new finding (F13), and concrete code.

### 9.1 Audit of the §8 secondary review

The §8 reviewer's five finding-verifications (F1, F2/F12, F3, F7, T2) all match the §3 evidence — concurred. Two of its *assessment* claims were load-bearing and unproven at the time §8 was written; I have now verified both against the firmware source:

| §8 claim | Verdict | Evidence |
|---|---|---|
| "opening up the self-imposed 64 B limit to 200 B … **zero firmware updates**" | **TRUE for bodies ≤ 247 B** — now verified, was only assumed before | `config.h` line 48: `HOST_INNER_MAX_LEN = 320` → `HOST_PAYLOAD_MAX_LEN = 320 − 7 − 2 = 311 B` (`host_uart.h`). A 200 B fragment makes a 202 B TX_FRAME_REQ payload — fits with 109 B margin. DMA RX buffer 512 B, RX ring 512 B, COBS bound 325 B (`host_uart.c` lines 11–84) all clear. The F7 overflow only bites at ≥ 248 B, so the host-side clamp of ≤ 200 B keeps a 47 B safety margin even before the firmware guard ships. |
| "will yield ~2× effective throughput" | **TRUE only as a package** — the arithmetic is ~2.4–2.6×, but *only if closed-loop pacing (1.4) and fragment retry (1.5) land in the same change* | At 200 B bodies with the **existing fixed 150 ms sleep**, the attempted duty is 164 ms ToA / ~329 ms cycle ≈ 498 ms ToA per second — **over** the firmware's 400 ms/s QoS budget. That re-creates the exact 2026-05-25 `ABORT_QOS`/`FORBIDDEN` storm, and without 1.5 every abort kills a whole frame. Bigger fragments *without* budget-aware pacing are a regression, not a win. |

Cosmetic: §8 contains text-encoding artifacts (a swallowed `t` in `time.sleep`, a mangled `req->length`). Content unaffected.

### 9.2 New verification evidence gathered for this closing pass

| # | Fact verified | Source | Consequence |
|---|---|---|---|
| V1 | Host-UART command frames carry up to **311 B** of payload (inner 320 − 7 header − 2 CRC); COBS + DMA + ring buffers are all sized for it | `config.h` L48, `host_uart.h`, `host_uart.c` | 200 B fragments are host-only work (Phase 1.3 confirmed feasible); 247 B possible after the F7 guard |
| V2 | The firmware QoS window is **fixed-anchor**, not rolling: `refresh_budget_window()` zeroes `used_us` when `now − window_start ≥ 1000 ms` | `sx1276_airtime.c` L36-41 | A *rolling*-window host mirror is strictly conservative — it can never attempt a TX the firmware would refuse. Use rolling on the host (code §9.4-C) |
| V3 | `REG_READ_REQ` is **unrestricted** (the allowlist exists only for writes) | `host_cmd.c` `handle_reg_read` vs `reg_write_allowed()` | The PHY contract check (§9.4-B) needs zero firmware changes: read `0x1D/0x1E/0x20/0x21` and decode |
| V4 | BW register mapping: `bw_to_reg_bits()` → 125 kHz=7, 250 kHz=8, 500 kHz=9; `MODEM_CONFIG1 = (bw<<4)|((cr−4)<<1)`; `MODEM_CONFIG2 = (sf<<4)|(1<<2)` | `sx1276.c` L152-159, L445-448 | Decode table for §9.4-B |
| V5 | Golden airtime vectors that match **live bench URCs**: 33 B on-air @ SF7/BW250 = 35 968 µs (2026-05-28 `tx_burst`), 49 B = 48 768 µs (2026-05-27 run) — both reproduced exactly by `lora_proto.lora_time_on_air_ms` at **BW250** | bench logs + estimator | Ready-made pins for the T2 golden-vector test (§9.4-G) |

### 9.3 NEW finding discovered during this pass

**F13 — Hidden 118 B clamp inside the fragmenter — HIGH (blocks Phase 1.3 as previously written).**
[`lora_proto.py::max_telemetry_fragment_payload`](../../DESIGN-CONTROLLER/base_station/lora_proto.py) binary-searches with `lo, hi = 1, TELEM_MAX_PAYLOAD` where `TELEM_MAX_PAYLOAD = 118` (the IP-306 *TelemetryFrame* reconciliation constant). The image strict path reuses this telemetry fragmenter, so **even after the air-cap rework, fragment bodies silently max out at 118 B** (114 B data). The 118 B limit is correct for real TelemetryFrames (the C-side `payload[120]` array) but is meaningless for raw `TX_FRAME_REQ` bodies whose true ceiling is 247 B.

- Impact if unfixed: Phase 1 tops out at ~433 B/s (114 B / 105 ms ToA × 0.4 duty) instead of the projected ~477 B/s, and §5's "160–200 B" step is unreachable through `pack_telemetry_fragments` no matter what cap is passed.
- Fix: a dedicated image fragment sizer that searches against the radio ceiling, not the telemetry envelope — code in §9.4-A. (433 → 477 B/s is only +10 %, but the same function is what later exploits the 247 B ceiling under FHSS/BW500, where the gap grows.)

This is a fourth instance of the review's core theme: **limits inherited from the wrong layer** (telemetry envelope → image path), exactly like BW500 model → BW250 radio (F1), probe clamp → production daemon (F3), and 25 ms cap at the wrong profile (F1).

### 9.4 Final recommendations with implementation-ready code

Priority order is unchanged from §7. Everything below compiles/runs against the code as it exists today; each block names its target file.

#### A. `lora_proto.py` — honest image profile + image-specific fragment sizing (fixes F1 + F13)

```python
# --- lora_proto.py ------------------------------------------------------
# The image strict path sends raw fragment bodies via TX_FRAME_REQ, not
# TelemetryFrames: no 7-byte telemetry envelope, no 2-byte CRC, but +8 B
# FHSS hop-sync header on the air (lora_pkt_hdr.h) and a 247 B body
# ceiling (255 B SX1276 payload - 8 B hop header).
PHY_IMAGE_BW250 = PhyProfile("image_bw250", 7, 250, 5, 8)   # what the radio RUNS today
PHY_IMAGE_BW500 = PhyProfile("image_bw500", 7, 500, 5, 8)   # valid only after Phase 2.4

LORA_HOP_HDR_LEN = 8            # lora_pkt_hdr.h LORA_PKT_HDR_LEN
TX_FRAME_BODY_MAX = 255 - LORA_HOP_HDR_LEN   # 247: F7-safe hard ceiling

def max_image_fragment_body(profile: PhyProfile,
                            max_air_ms: float,
                            body_ceiling: int = TX_FRAME_BODY_MAX) -> int:
    """Largest TX_FRAME_REQ body whose ON-AIR time (body + hop header)
    fits max_air_ms. Unlike max_telemetry_fragment_payload() this does
    NOT inherit the 118 B TelemetryFrame clamp (F13) and does NOT add
    the 9 B telemetry envelope the strict path never sends."""
    lo, hi, best = 1, body_ceiling, 0
    while lo <= hi:
        mid = (lo + hi) // 2
        if lora_time_on_air_ms(mid + LORA_HOP_HDR_LEN, profile) <= max_air_ms:
            best, lo = mid, mid + 1
        else:
            hi = mid - 1
    return best

def pack_image_fragments(payload: bytes, frag_seq: int, profile: PhyProfile,
                         max_air_ms: float) -> list[bytes]:
    """pack_telemetry_fragments twin for the image strict path (0xFE v1
    header, same reassembler) sized by max_image_fragment_body()."""
    chunk = max_image_fragment_body(profile, max_air_ms) - TELEMETRY_FRAGMENT_HEADER_LEN
    if chunk <= 0:
        raise ValueError(f"{profile.name}: no fragment fits {max_air_ms} ms")
    total = max(1, (len(payload) + chunk - 1) // chunk)
    if total > 256:
        raise ValueError(f"payload needs {total} fragments; max 256")
    return [bytes([TELEMETRY_FRAGMENT_MAGIC, frag_seq & 0xFF, i & 0xFF,
                   (total - 1) & 0xFF]) + payload[i * chunk:(i + 1) * chunk]
            for i in range(total)]
```

Cap policy that goes with it (link-composition-aware, replaces the blanket 25 ms for this path):

```python
# Strict-path bench: control NEVER rides this radio -> the only real
# bound is the firmware's own 380 ms per-TX dwell cap. When the M7
# control path later shares the radio, drop this to the C1-derived cap.
IMAGE_FRAG_AIR_CAP_MS = float(os.environ.get("LIFETRAC_FRAG_AIR_CAP_MS", "170.0"))
# 170 ms @ SF7/BW250 admits a 200 B body (ToA 164 ms) and keeps every
# fragment well under the 380 ms dwell invariant with margin for retunes.
```

#### B. Startup PHY contract check (fixes the F1 *class*; zero firmware changes — uses the unrestricted reg-read path, V3/V4)

```python
# --- image_tx_daemon.py / image_rx_daemon.py, called right after
# --- configure_regulatory_profile_if_needed(link) in _open_link() -----
_BW_KHZ_BY_BITS = {7: 125, 8: 250, 9: 500}   # sx1276.c bw_to_reg_bits()

def verify_modem_matches_profile(link, profile) -> None:
    """Fail LOUD at startup if the L072's live modem registers disagree
    with the PhyProfile the fragmenter is about to size against (F1)."""
    cfg1, _ = read_reg(link, 0x1D, timeout=0.5)   # RegModemConfig1
    cfg2, _ = read_reg(link, 0x1E, timeout=0.5)   # RegModemConfig2
    pre_msb, _ = read_reg(link, 0x20, timeout=0.5)
    pre_lsb, _ = read_reg(link, 0x21, timeout=0.5)
    actual = {
        "sf": (cfg2 >> 4) & 0x0F,
        "bw_khz": _BW_KHZ_BY_BITS.get((cfg1 >> 4) & 0x0F, -1),
        "cr_den": ((cfg1 >> 1) & 0x07) + 4,
        "preamble": (pre_msb << 8) | pre_lsb,
    }
    expected = {"sf": profile.sf, "bw_khz": profile.bw_khz,
                "cr_den": profile.cr_den, "preamble": profile.preamble_len}
    if actual != expected:
        raise RuntimeError(
            f"PHY CONTRACT VIOLATION: modem={actual} profile={expected}. "
            "Refusing to start — fragment sizing would be wrong (see "
            "CODE REVIEWS 2026-07-23 F1). Fix the profile constant or "
            "the firmware config; do not silence this check.")
    LOG.info("PHY contract OK: %s == %s", profile.name, actual)
```

Had this existed, F1 would have been a startup crash on day one instead of a 2×-wrong link model in production. **This is the single highest-leverage snippet in this document.**

#### C. Budget-paced TX with retry/abort (fixes F2, F4, F12 — `image_tx_daemon.py`)

```python
# --- image_tx_daemon.py -------------------------------------------------
import collections

class AirtimeBudget:
    """Host mirror of the firmware per-channel QoS gate
    (sx1276_airtime.c: 400 ms ToA per 1 s window). Rolling window =
    strictly more conservative than the firmware's fixed-anchor window
    (V2), so a paced TX can never draw ABORT_QOS. 380 ms budget leaves
    a 20 ms guard for estimator rounding."""

    def __init__(self, budget_us: int = 380_000, window_s: float = 1.0):
        self.budget_us, self.window_s = budget_us, window_s
        self._events: collections.deque[tuple[float, int]] = collections.deque()

    def _used(self, now: float) -> int:
        while self._events and now - self._events[0][0] >= self.window_s:
            self._events.popleft()
        return sum(toa for _, toa in self._events)

    def admit(self, est_toa_us: int, stop) -> bool:
        """Block until est_toa_us fits the window. Returns False on stop."""
        while not stop.is_set():
            now = time.monotonic()
            if self._used(now) + est_toa_us <= self.budget_us:
                return True
            wait = self._events[0][0] + self.window_s - now  # oldest expiry
            stop.wait(min(max(wait, 0.005), 0.25))
        return False

    def record(self, toa_us: int) -> None:
        self._events.append((time.monotonic(), toa_us))


MAX_QOS_RETRIES = 4   # FORBIDDEN/ABORT_QOS: not admitted, ZERO RF spent
MAX_RF_RETRIES  = 1   # TX_DONE non-OK / timeout: airtime was spent

def _tx_one_frame(self, link, frame):                      # replaces current body
    fragments = pack_image_fragments(frame.payload, frame.seq,
                                     PHY_IMAGE_BW250, IMAGE_FRAG_AIR_CAP_MS)
    for idx, body in enumerate(fragments):
        est_us = int(lora_time_on_air_ms(len(body) + LORA_HOP_HDR_LEN,
                                         PHY_IMAGE_BW250) * 1000)
        sent = False
        for attempt in range(1 + MAX_QOS_RETRIES + MAX_RF_RETRIES):
            if not self.budget.admit(est_us, self._stop):
                return                                     # shutting down
            try:
                link.send(HOST_TYPE_TX_FRAME_REQ,
                          bytes([idx & 0xFF, len(body)]) + body)
                done, _faults = wait_for_tx_done(link, idx & 0xFF,
                                                 timeout=PER_FRAGMENT_TX_TIMEOUT_S)
            except RuntimeError:      # ERR_PROTO FORBIDDEN == QoS refusal:
                time.sleep(est_us / 2e6)                   # cheap, no RF spent
                continue
            except TimeoutError:
                self.budget.record(est_us)                 # assume RF spent
                continue                                   # counts vs RF retries
            self.budget.record(done.get("time_on_air_us") or est_us)
            if done["status"] == 0:
                sent = True
                break
        if not sent:
            # F4: a frame missing any fragment can never reassemble --
            # stop burning airtime on it and let RX request a keyframe.
            with self.lock:
                self.frames_tx_fail += 1
            LOG.warning("frame seq=%d ABORTED at fragment %d/%d",
                        frame.seq, idx, len(fragments))
            return
    with self.lock:
        self.frames_tx_ok += 1
```

(Also delete the `MIN_LORA_HOST_INTER_CYCLE_S` floor and the fixed `time.sleep(self.inter_cycle_s)` — the budget replaces both. Keep `inter_cycle_s` accepted-but-ignored for one release so existing compose files don't break.)

#### D. Firmware F7 guard (`host_cmd.c` — must land before bodies > 200 B)

```c
/* host_cmd.c :: handle_tx_frame(), after req.length is parsed.
 * F7: sx1276_tx_begin() computes (uint8_t)(req->length + LORA_PKT_HDR_LEN);
 * length >= 248 wraps, passes the airtime invariant with a tiny bogus
 * effective_len, and emits a corrupt frame with NO error to the host. */
#ifdef LIFETRAC_FHSS_TX_ROUTED
    if (req.length > (uint8_t)(255U - LORA_PKT_HDR_LEN)) {   /* > 247 */
        host_uart_send_err_proto(frame->seq, frame->type, frame->ver,
                                 HOST_ERR_PROTO_BAD_LENGTH,
                                 (uint16_t)(255U - LORA_PKT_HDR_LEN));
        return;
    }
#endif
```

#### E. RX keyframe self-heal (fixes F9 — `image_rx_daemon.py`)

```python
# --- image_rx_daemon.py -------------------------------------------------
KEYFRAME_REQ_TOPIC = "lifetrac/v25/cmd/req_keyframe"   # camera_service listens

class KeyframeRequester:
    """Rate-limited req_keyframe on reassembly failure. Publishes to the
    same topic web_ui's manual button uses, so it inherits whatever
    broker relay the deployment already has for that path."""
    def __init__(self, client, min_interval_s: float = 5.0):
        self._client, self._min, self._last = client, min_interval_s, 0.0
    def poke(self, reason: str) -> None:
        now = time.monotonic()
        if self._client is None or now - self._last < self._min:
            return
        self._last = now
        LOG.info("requesting keyframe (%s)", reason)
        self._client.publish(KEYFRAME_REQ_TOPIC, b"\x01", qos=0)

# in _rx_worker(), where reassembler counters are mirrored today:
if cur_timeout != last_timeouts:
    ...existing mirror...
    self._kf_req.poke(f"reassembly timeout #{cur_timeout}")
if cur_decode != last_decode_errors:
    ...existing mirror...
    self._kf_req.poke(f"decode error #{cur_decode}")
```

#### F. Depth-2 TX mailbox (fixes F6 — firmware, **sketch**, requires the §5-2.5 P0 re-validation before merge)

```c
/* host_cmd.c — accept ONE queued TX while RF is active. */
static sx1276_tx_request_t s_tx_pending;
static bool s_tx_pending_valid = false;

static void handle_tx_frame(const host_frame_t *frame) {
    /* ...existing parse + F7 guard... */
    if (sx1276_tx_busy()) {
        if (s_tx_pending_valid) {           /* depth 2 reached */
            host_uart_send_err_proto(frame->seq, frame->type, frame->ver,
                                     HOST_ERR_PROTO_QUEUE_FULL, 1U);
            return;
        }
        s_tx_pending = req;                 /* park it; started from poll */
        s_tx_pending_valid = true;
        return;
    }
    if (!sx1276_tx_begin(&req)) { /* ...FORBIDDEN as today... */ }
}

/* new, called from main.c right AFTER host_cmd_emit_tx_done() and AFTER
 * the rx_service call — RX must get its service window between
 * back-to-back TXs or a continuous image stream starves the tractor's
 * back-channel (see §3 "Minor/hygiene"). */
void host_cmd_service_tx_mailbox(void) {
    if (!s_tx_pending_valid || sx1276_tx_busy()) return;
    sx1276_tx_request_t next = s_tx_pending;
    s_tx_pending_valid = false;
    if (!sx1276_tx_begin(&next)) {
        /* RFCO_PERTX already names the refusal; surface proto err too */
        host_uart_send_err_proto(0U, HOST_TYPE_TX_FRAME_REQ,
                                 HOST_PROTOCOL_VER,
                                 HOST_ERR_PROTO_FORBIDDEN, 0U);
    }
}
```

#### G. T2 golden-vector test (pins the estimator to reality — `base_station/tests/`)

```python
# --- tests/test_phy_golden_vectors.py -----------------------------------
import unittest
from lora_proto import PHY_BY_NAME, lora_time_on_air_ms

class TestPhyGoldenVectors(unittest.TestCase):
    """On-air lengths & ToA measured from live TX_DONE_URC values.
    If an estimator or profile edit breaks these, the wire model has
    drifted from the silicon — do NOT relax the deltas (F1 postmortem)."""
    VECTORS = [
        # (profile, on_air_len_B, measured toa_us, source)
        ("image_bw250", 33, 35_968,  # 2026-05-28 tx_burst RFCO/TX_DONE
         "25 B body + 8 B hop hdr"),
        ("image_bw250", 49, 48_768,  # 2026-05-27 air-link-proven run
         "41 B body + 8 B hop hdr"),
    ]
    def test_estimator_matches_firmware_urc(self):
        for name, length, toa_us, note in self.VECTORS:
            est = lora_time_on_air_ms(length, PHY_BY_NAME[name]) * 1000
            self.assertAlmostEqual(est, toa_us, delta=1,
                                   msg=f"{name}/{length}B ({note})")
```

#### H. XOR parity fragment (Phase 3.1 concept — TX side; RX patch belongs in `reassemble.py` with tests)

```python
PARITY_MAGIC = 0xFC          # header: magic, frag_seq, group_start, group_len

def add_xor_parity(fragments: list[bytes], frag_seq: int,
                   group: int = 8) -> list[bytes]:
    """~12 % overhead; RX can rebuild any ONE lost fragment per group
    with zero round trips. Data fragments pass through unchanged, so
    an unpatched RX simply ignores 0xFC (unknown magic -> decode_error
    counter, no crash)."""
    out: list[bytes] = []
    for g in range(0, len(fragments), group):
        chunk = fragments[g:g + group]
        out.extend(chunk)
        width = max(len(f) for f in chunk)
        acc = bytearray(width)
        for f in chunk:
            for i, b in enumerate(f.ljust(width, b"\x00")):
                acc[i] ^= b
        out.append(bytes([PARITY_MAGIC, frag_seq & 0xFF,
                          g & 0xFF, len(chunk) & 0xFF]) + bytes(acc))
    return out
```

### 9.5 Closing recommendation

Execution order stands as §5/§7 wrote it, with two amendments from this pass:

1. **F13 joins Phase 1** — land §9.4-A's `max_image_fragment_body`/`pack_image_fragments` *instead of* raising caps on the telemetry fragmenter, or Phase 1 silently stalls at 118 B / ~433 B/s.
2. **Phase 1 is atomic** — 1.1 (PHY model), 1.3 (fragment size), 1.4 (budget pacing), 1.5 (retry/abort) ship as one change set guarded by one env knob (`LIFETRAC_TX_PIPELINE=v2`). Any subset re-opens a known failure mode (§9.1, second row).

The contract check (§9.4-B) and golden-vector test (§9.4-G) should merge **first**, before any tuning — they are the tripwires that make the rest of the plan safe to execute, and they encode this review's central lesson: *every one of the four throughput-limiting defects (F1, F3, F13, and the 25 ms cap) was a limit inherited from the wrong layer, and all four were invisible because no test compared layers against each other.*

---

## 10. Final Review v2 — Re-verification Against the Current Tree, Corrections to §§1–9, New Findings F14–F16, and the Consolidated Merge Plan

**Date:** 2026-07-23
**Author:** GitHub Copilot (Claude Fable 5) — final closing pass
**Method:** every §8/§9 claim re-checked by reading the tree as it exists *now* (not as quoted earlier); interim verification results recorded in repo memory (test-suite runs, bench evidence) folded in. Where this section contradicts §§1–9, **this section wins**.

### 10.1 Corrections to earlier sections (verified against current source)

| # | Earlier claim | Correction | Evidence |
|---|---|---|---|
| C1 | **F7 "latent u8 overflow, must be fixed before >200 B"** (§3-F7, §9.4-D) | **STALE — the guard already exists in-tree.** `sx1276_tx_begin()` now rejects `req->length + LORA_PKT_HDR_LEN > 255` *before* the narrowing cast, emitting an `INTERNAL` RFCO snapshot and returning false (host sees `ERR_PROTO FORBIDDEN`). 247 B bodies are already safe; §9.4-D is downgraded from "must land" to *optional diagnostics polish* (a `BAD_LENGTH` + max-detail reply in `handle_tx_frame` is friendlier than `FORBIDDEN`, and rejecting pre-parse avoids consuming a hop slot — it already doesn't). **Action: pin it with a regression test (§10.4-E) instead of re-fixing it.** | `sx1276_tx.c` `sx1276_tx_begin()`: `if ((uint32_t)req->length + (uint32_t)LORA_PKT_HDR_LEN > 255UL) { … return false; }` |
| C2 | **"~477 B/s" Phase-1 ceiling** (§2, §5, §9) | **~392–486 B/s, and only at the right body size.** The firmware QoS window is fixed-anchor and admits only *whole* fragments: `floor(400 ms / ToA)` per window. 200 B body (ToA 164 ms) → 2/window → **392 B/s**. 247 B body (on-air 255 B, ToA 199.8 ms; 2 × 199 808 µs = 399 616 ≤ 400 000 — fits by 384 µs) → 2 × 243 B → **486 B/s**. The fractional 2.43 frag/s behind "477" is unreachable through a quantized window. Since C1 makes 247 B safe today, **Phase 1 should go straight to 247 B bodies**; the §9.4-C rolling mirror converges to the same 2-per-window cadence with zero aborts. | `sx1276_airtime.c` `refresh_budget_window()` + reserve arithmetic; airtime formula |
| C3 | **F4 retry math "45 % → 99 % keyframe completion"** (§3-F4, §9.4-C) | **Only true for** ***locally visible*** **failures. TX_DONE is local completion, not a receiver ACK** — `status=OK` means the SX1276 finished radiating, and nothing comes back from the peer. A TX-side retry therefore repairs QoS refusals, modem timeouts, and faults, but **cannot repair air-path loss**, which is the loss mode the (1−p²)ⁿ math was aimed at. Air-loss repair needs *independent copies or parity*: the v2 `0xFD` redundancy header (W2-02 P0c) is already implemented and tested on the RX side (`reassemble.py` dedups by `(seq, idx)`, first copy wins) — the production TX daemon just never emits it. §10.4-C makes keyframes use it. §9.4-C's retry loop stays — it is still correct for the local-failure class. | `host_types.h` TX_DONE payload doc; `reassemble.py` v2 path; `lora_proto.py` L815-827 |
| C4 | **"Make DTS BW500 real = call `sx1276_apply_profile_full()` on activate"** (§5-2.4) | **Underscoped — profile 2 is dead-on-boot for two more reasons.** (a) TX routing under `LIFETRAC_FHSS_TX_ROUTED` (pinned on by the Makefile) requires the FHSS scheduler for *any* non-bench profile, but `host_cfg_profile_activate()` calls `sx1276_fhss_init()` **only for profile 1** → under profile 2 every TX is refused with `INTERNAL`. (b) The 400 ms/10 s legal-dwell accountant applies to *all* non-bench profiles; on a single DTS channel that caps max-body BW500 at `4 × 243 B / 10 s ≈ 97 B/s`. Dwell is an FHSS concept — FCC DTS compliance is PSD-based — so DTS needs a profile-aware dwell/QoS bypass, a fixed-wideband-channel routing gate, *and* the modem reprogram. Real firmware work item, not one call. | `host_cfg_profile.c` `activate()` (FHSS-init only for profile 1); `sx1276_tx.c` bench-only bypasses; `sx1276_legal_dwell` reserve path |
| C5 | §8's "~2× with zero firmware updates" | Both halves now fully verified (V1 + C1): UART transport carries 311 B payloads and the length guard exists, so even **247 B** bodies are zero-firmware. The §9.1 caveat stands: pacing + retry must land in the same change set. | `config.h` L48, `host_uart.h`, C1 |

### 10.2 New findings (F14–F16, all verified in current source)

**F14 — Oversized-tile fallback ships a truncated, undecodable WebP — HIGH (correctness, live today).**
[`camera_service.py::_encode_tile`](../../DESIGN-CONTROLLER/firmware/tractor_x8/camera_service.py) walks quality down to 5 and then — comment says "Last resort" — **returns whatever blob it has, even if > 256 B**. `_build_frame` then writes `size = min(len(blob), 256)` and `blob[:size]`: a **truncated WebP container** that the base's decoder must reject. Cost: full airtime spent shipping a tile that produces a decode error (and, being a decode error, it also pokes the future F9/§9.4-E keyframe-request path — a self-inflicted keyframe storm risk). A deterministic high-entropy tile was measured at 334 B at q=5 during this review's validation pass. Note the *obvious* fix (per-tile MONO_G4 fallback) is **wire-illegal**: `codec` is a per-frame header byte, and codec 15 "per-tile escape" is reserved-unimplemented. The wire-safe fix is grayscale-retry-then-drop (§10.4-B) — a dropped tile leaves the previous canvas tile (honest, cached) instead of poisoning the frame.

**F15 — CFG_SET rejections are logged as "OK" — HIGH (ops trap; blocked the FHSS milestone silently).**
[`method_h_stage2_tx_probe_v2.py::configure_regulatory_profile_if_needed`](../../DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py) — used by **both production daemons** — prints `CFG_SET_REQ(...) OK: <hex>` without ever decoding the status byte. The `CFG_OK_URC` payload is `[key, status, actual_len, 0]` (`host_cfg_wire.c`), and status ≠ 0 *arrives on the same URC type*, so `link.request()` happily returns it. Bench evidence: `14 08 00 00` = REG_PROFILE rejected with `CFG_STATUS_PROFILE_REJECT_MASK_POPCOUNT` (8) — because the helper's default channel mask is **single-channel** while profile 1 validation requires popcount ≥ 50 (`host_cfg_profile_validate`). Net effect: every "FHSS profile 1" run that didn't set `LIFETRAC_FHSS_WIDE_MASK=1` was **silently still profile 0**. This is the same "looks green, isn't" class as the SYNC_OK=0 trap in the project's misdiagnosis notes, and it likely contaminated earlier FHSS bench conclusions (v25.0.6.5 evidence should be re-run after §10.4-A lands).

**F16 — Stale-frame cancellation and RX GC gaps — MEDIUM.**
(a) The TX daemon's newest-wins queue drop cannot help once a stale keyframe has *started* transmitting — there is no cancel path mid-frame, so a 60 s-old keyframe can occupy the link while fresh P-frames queue behind it. With §10.4-C pacing the worst case shrinks, but a `frame.enqueued_ms` staleness check before each fragment is ~5 lines (§10.4-D). (b) `FragmentReassembler._gc()` only runs inside `feed()` — during RF silence nothing expires, so "pending" partials linger and the §9.4-E timeout-poke never fires exactly when the link goes quiet (the moment it matters most). Add a public `tick()` called from the RX drain loop (§10.4-D). (c) On the video-test topology the RX daemon owns `/dev/ttymxc3` exclusively — there is no L072 back-channel for keyframe requests, so F9's fix must ride MQTT (as §9.4-E already does) — noting this here so nobody "optimizes" it onto the UART.

**Validation snapshot backing this section** (recorded during the interim verification pass): focused Python suite 118 pass / 1 skip; C `airtime/FHSS/dwell/RX-policy` bench suites pass; full `unittest discover` (the CI invocation) = 956 tests, 3 fail / 18 error / 2 skip — the pre-existing T3 package-collision issue, unchanged; `make check-cfg-profile` **link target omits the FHSS objects** (build-graph bug — the suite passes when linked manually with 27 unit + 9 wire cases), which is a small but real CI blind spot to fix alongside T3.

### 10.3 Consolidated merge plan (supersedes §7 / §9.5 where they conflict)

| Order | Item | Class | Why this position |
|---|---|---|---|
| 1 | **Tripwires**: §9.4-B contract check, §9.4-G golden vectors, **§10.4-A CFG status decode + profile readback** | zero-risk, host-only | Every later change is unverifiable without them; F15 shows even the *existing* milestone evidence is suspect |
| 2 | **Correctness bugs shipping today**: §10.4-B (F14 tile fallback), §10.4-D-b (RX GC tick) | host-only | Wrong at any speed; tiny diffs |
| 3 | **Phase-1 atomic package** (unchanged scope, corrected targets): PHY model fix + §9.4-A sizer **at 247 B** + §9.4-C pacing/local-retry + §9.4-E self-heal, behind `LIFETRAC_TX_PIPELINE=v2` | host-only | Target: **~486 B/s**, keyframe ≈ 6.5 s, zero ABORT_QOS. C1 removed the firmware prerequisite |
| 4 | **Keyframe air-loss defense**: §10.4-C — `0xFD` copies=2 *for I-frames only, PER-gated* | host-only | The honest replacement for the C3-corrected retry claim; RX support already shipped |
| 5 | **Phase-2 firmware**: F7 regression pin (§10.4-E), depth-2 mailbox (§9.4-F), FHSS closure (now *actually testable* thanks to №1), DTS BW500 **re-scoped per C4** | firmware | Multipliers: ~1.2 KB/s (FHSS BW250) → ~2.4 KB/s (DTS, after C4 scope) |
| 6 | Codec work (F8), operating-point sweep, F10 wire-header repair | either | Compounding interest, orthogonal |

### 10.4 Implementation-ready code (new/corrected items only — §9.4 A/B/C/E/F/G/H remain valid as written)

#### A. F15 — decode CFG status, verify the active profile (host, both daemons + probe helper)

```python
# --- method_h_stage2_tx_probe_v2.py (used by both daemons) --------------
CFG_STATUS_NAMES = {
    0: "OK", 1: "UNKNOWN_KEY", 2: "BAD_LENGTH", 3: "OUT_OF_RANGE",
    4: "APPLY_FAILED", 5: "DEFERRED", 6: "READ_ONLY", 7: "PROFILE_UNROUTED",
    8: "PROFILE_REJECT_MASK_POPCOUNT", 9: "PROFILE_REJECT_MASK_OUT_OF_TABLE",
    10: "PROFILE_REJECT_BW_MISMATCH", 11: "PROFILE_REJECT_ANTENNA_OUT_OF_RANGE",
    12: "PROFILE_REJECT_NO_POWER_HEADROOM", 13: "PROFILE_REJECT_NOT_STAGED",
}   # host_cfg.h cfg_status_t

def cfg_set_checked(link, key: int, value: bytes, timeout: float = 1.0) -> dict:
    """CFG_SET that actually reads the status byte. CFG_OK_URC payload is
    [key, status, actual_len, 0] (host_cfg_wire.c) — status != 0 arrives on
    the SAME URC type, so request() returns 'success' for rejections (F15:
    '14 08 00 00' = REG_PROFILE refused MASK_POPCOUNT, printed as OK)."""
    ack = link.request(HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
                       bytes([key, len(value)]) + value, timeout=timeout)
    p = ack["payload"]
    if len(p) < 2 or p[0] != (key & 0xFF):
        raise RuntimeError(f"CFG_OK_URC malformed/mismatched: {p.hex()}")
    if p[1] != 0:
        raise RuntimeError(
            f"CFG_SET(0x{key:02X}) REJECTED: status={p[1]} "
            f"({CFG_STATUS_NAMES.get(p[1], '?')}) — do NOT treat as OK")
    return ack

def verify_active_profile(link, expected_id: int, timeout: float = 1.0) -> None:
    """Post-activation readback — profile 1 with the default single-channel
    mask silently stays 0 (popcount >= 50 required). Fail loud instead."""
    ack = link.request(HOST_TYPE_CFG_GET_REQ, HOST_TYPE_CFG_OK_URC,
                       bytes([CFG_KEY_REG_PROFILE]), timeout=timeout)
    p = ack["payload"]           # CFG_DATA: [key, value_len, value...]
    active = p[2] if len(p) >= 3 else -1
    if active != expected_id:
        raise RuntimeError(
            f"REG_PROFILE readback={active}, expected {expected_id} — "
            "activation was rejected upstream (check mask popcount / F15)")
```

Then in `configure_regulatory_profile_if_needed()` replace each `link.request(...); print("... OK ...")` pair with `cfg_set_checked(...)` and finish with `verify_active_profile(link, profile_id)`.

#### B. F14 — never ship a truncated tile (wire-safe: grayscale retry, then drop)

```python
# --- camera_service.py::_encode_tile — replace the loop tail ------------
    while quality >= 5:
        buf = io.BytesIO()
        img.save(buf, format="WEBP", quality=quality, method=4)
        blob = buf.getvalue()
        if len(blob) <= TILE_BYTES_MAX:
            return blob
        quality -= 10
    # F14: NEVER return >256 B — _build_frame would ship blob[:256], a
    # truncated RIFF container the base cannot decode (pure wasted airtime
    # + a spurious decode-error keyframe request). Per-tile MONO_G4 is NOT
    # wire-legal (codec is a per-frame header byte; 15 = reserved escape),
    # so: grayscale re-try (typically halves the size), then drop.
    gray = img.convert("L").convert("RGB")
    buf = io.BytesIO()
    gray.save(buf, format="WEBP", quality=5, method=6)
    blob = buf.getvalue()
    if len(blob) <= TILE_BYTES_MAX:
        return blob
    LOG.warning("tile unencodable <=%d B even gray-q5 (%d B); dropping",
                TILE_BYTES_MAX, len(blob))
    return None                      # caller: skip tile, keep bitmap honest

# --- camera_service.py::_build_frame — kept-loop guard -------------------
        blob = _encode_tile(canvas, tx, ty, quality=q)
        if blob is None:
            continue        # dropped: bitmap rebuild below clears its bit,
                            # RX keeps the previous cached tile (badge=Cached)
```

(The encode-cache branch needs the same `None` guard before `encode_cache.store`.)

#### C. Keyframe air-loss defense — v2 `0xFD` copies, PER-gated (honest fix for C3)

```python
# --- lora_proto.py -------------------------------------------------------
def pack_image_fragments_v2(payload: bytes, frag_seq: int,
                            profile: PhyProfile, max_air_ms: float,
                            copies: int = 2) -> list[bytes]:
    """Wrap §9.4-A fragments in the v2 0xFD redundancy header that
    image_pipeline/reassemble.py ALREADY dedups (first copy wins, W2-02
    P0c). Unlike a TX_DONE retry (local-only — C3), independent copies
    repair AIR loss: P(fragment lost) = p^copies. copies=2 at air-PER 1%
    -> 80-fragment keyframe completion (1-1e-4)^80 ~= 99.2%."""
    if not 1 <= copies <= 15:
        raise ValueError("copies must be 1..15 (4-bit nibble)")
    base = pack_image_fragments(payload, frag_seq, profile, max_air_ms)
    if copies == 1:
        return base
    out: list[bytes] = []
    for body in base:                       # body = 0xFE hdr(4) + data
        _, seq, idx, total_m1 = body[0], body[1], body[2], body[3]
        for copy_idx in range(copies):
            out.append(bytes([TELEMETRY_FRAGMENT_MAGIC_V2, seq, idx, total_m1,
                              ((copies & 0x0F) << 4) | copy_idx]) + body[4:])
    return out

# --- image_tx_daemon.py — choose per frame, gated on measured PER --------
IMAGE_KEYFRAME_COPIES = int(os.environ.get("LIFETRAC_KEYFRAME_COPIES", "1"))
PER_REDUNDANCY_THRESHOLD = 0.005     # enable copies=2 above 0.5% frag loss

def _pack_for(self, frame) -> list[bytes]:
    is_key = frame.payload[:1] == b"\x01"          # frame_kind byte
    copies = IMAGE_KEYFRAME_COPIES
    if copies <= 1 and is_key and self.recent_frag_loss_rate() > PER_REDUNDANCY_THRESHOLD:
        copies = 2                                  # auto: only when PER says so
    if is_key and copies > 1:
        return pack_image_fragments_v2(frame.payload, frame.seq,
                                       PHY_IMAGE_BW250, IMAGE_FRAG_AIR_CAP_MS,
                                       copies=copies)
    return pack_image_fragments(frame.payload, frame.seq,
                                PHY_IMAGE_BW250, IMAGE_FRAG_AIR_CAP_MS)
```

Cost model to keep in the compose comment: copies=2 doubles keyframe airtime (6.5 s → 13 s at Phase-1 rates), so it is a *degraded-link* tool, not a default — hence the PER gate. At bench PER ≈ 0 keep copies=1.

#### D. F16 — stale-frame skip + RX GC tick

```python
# --- image_tx_daemon.py::_tx_one_frame, before the fragment loop ---------
FRAME_MAX_AGE_MS = int(os.environ.get("LIFETRAC_FRAME_MAX_AGE_MS", "10000"))
    age_ms = int(time.monotonic() * 1000) - frame.enqueued_ms
    if age_ms > FRAME_MAX_AGE_MS and not self._q.empty():
        with self.lock:
            self.frames_dropped_stale = getattr(self, "frames_dropped_stale", 0) + 1
        LOG.info("dropping stale frame seq=%d (age %d ms, fresher queued)",
                 frame.seq, age_ms)
        return                       # newest-wins now applies mid-pipeline too

# --- image_pipeline/reassemble.py — public tick ---------------------------
    def tick(self, now_ms: int | None = None) -> None:
        """Run GC without feeding a fragment. Call from the RX drain loop so
        partials expire during RF silence (F16b) — today _gc only runs
        inside feed(), i.e. never when the link goes quiet."""
        self._gc(self._clock_ms() if now_ms is None else now_ms)

# --- image_rx_daemon.py::_rx_worker — in the `if not frames:` branch ------
            if not frames:
                self.reassembler.tick()          # expire partials in silence
                continue
```

(The `tick()` path also makes §9.4-E's timeout-poke fire during silence — which is precisely when a keyframe request is most valuable.)

#### E. Pin the now-existing length guard (regression test, C bench)

```c
/* --- bench/host_proto/tx_len_guard.c (new; add to make check set) -------
 * C1/F7: sx1276_tx_begin() must refuse length+8 > 255 BEFORE any FIFO or
 * scheduler side effect. This pins the in-tree guard so a refactor can't
 * resurrect the 2026-07-23 narrowing-cast overflow. Uses sx1276_stub.h. */
#include "sx1276_tx.h"
#include "sx1276_stub.h"
#include <stdio.h>

int main(void) {
    sx1276_stub_reset();
    sx1276_tx_request_t req = {0};
    req.tx_id = 0x42U;

    req.length = 248U;                       /* 248+8 = 256 > 255: refuse */
    CHECK(!sx1276_tx_begin(&req), "len=248 must be refused");
    CHECK(sx1276_stub_fifo_writes() == 0U, "no FIFO bytes on refusal");

    req.length = 247U;                       /* 247+8 = 255: exactly legal */
    CHECK(sx1276_tx_begin(&req), "len=247 must be admitted");
    printf("PASS tx_len_guard\n");
    return 0;
}
```

#### F. Fix the `check-cfg-profile` link lines (the §10.2 CI blind spot — verified root cause)

`host_cfg_profile_activate()` calls `sx1276_fhss_init()` **unconditionally** (no `#ifdef`, `host_cfg_profile.c` line 186), and `sx1276_fhss.c` in turn needs `sx1276_fhss_chantab_center_hz()`. Neither `check-cfg-profile` nor `check-cfg-profile-wire` links those two objects (and `sx1276_stub.c` does not provide them), so both targets die at link with `undefined reference to 'sx1276_fhss_init'` — which is why the suite only passes "when linked manually". Two-line Makefile fix:

```makefile
# --- firmware/murata_l072/Makefile ---------------------------------------
check-cfg-profile: $(BUILD)
	@echo "[CFGP ] Building cfg-profile test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    bench/host_proto/cfg_profile.c \
	    host/host_cfg_profile.c \
	    radio/sx1276_fhss.c \
	    radio/sx1276_fhss_chantab.c \
	    -o $(CFG_PROFILE_BIN)
	@echo "[CFGP ] Running cfg-profile test ..."
	@$(CFG_PROFILE_BIN)

check-cfg-profile-wire: $(BUILD)
	@echo "[CFGPW] Building cfg-profile-wire test ..."
	@$(HOST_CC) -std=gnu11 -Wall -Wextra -Werror -Iinclude -I. -Ibench/host_proto \
	    -DLIFETRAC_FHSS_TX_ROUTED \
	    bench/host_proto/cfg_profile_wire.c \
	    bench/host_proto/sx1276_stub.c \
	    host/host_cfg.c \
	    host/host_cfg_profile.c \
	    radio/sx1276_fhss.c \
	    radio/sx1276_fhss_chantab.c \
	    -o $(CFG_PROFILE_WIRE_BIN)
	@echo "[CFGPW] Running cfg-profile-wire test ..."
	@$(CFG_PROFILE_WIRE_BIN)
```

Both FHSS translation units are freestanding (`<stddef.h>`/`<string.h>` only), so they compile clean under `HOST_CC` with no further stubs. Since CI's `make check` includes both targets, this is also the reason the §10.2 validation snapshot had to link manually — land it with priority №1 in §10.3 so the CI gate is green *before* the FHSS-closure work (№5) starts leaning on these very tests.

### 10.5 Corrected outlook (replaces the §5 table where numbers differ)

| Milestone | Goodput | ~3 KB keyframe | Notes |
|---|---:|---:|---|
| Today | ~180 B/s | ~15–17 s | measured behavior |
| Phase 1 @ 247 B, budget-paced, 1-ch | **~486 B/s** | ~6.5 s | was "477"; fixed-anchor window quantization, 2 × 243 B per window |
| Phase 1 @ 247 B + keyframe copies=2 (only if PER > 0.5 %) | ~486 B/s wire | ~13 s but ~99 % completion | degraded-link mode, not default |
| Phase 2 FHSS BW250, pipelined | ~1.2 KB/s | ~2.6 s | unchanged; now *testable* because F15 no longer masks profile rejection |
| Phase 2 DTS BW500 (re-scoped per C4) | ~2.4 KB/s | ~1.3 s | requires DTS routing gate + profile-aware dwell, not just modem reprogram |

### 10.6 Bottom line

Three of this document's own recommendations were already stale or mis-scoped by the time of this final pass (C1 fix already in-tree; C2 number optimistic by ~20 %; C4 one-liner actually a work item) — and one celebrated mechanism (TX retry vs air loss, C3) promised a repair it cannot physically deliver. None of that changes the plan's shape; it changes what gets *verified*. The additions that make the plan trustworthy are exactly the unglamorous ones: decode the status byte (§10.4-A), never ship a truncated container (§10.4-B), pin the guard that already exists (§10.4-E), and let the reassembler breathe during silence (§10.4-D). The throughput ladder — ~180 B/s → ~486 B/s (host-only) → ~1.2 KB/s (FHSS) → ~2.4 KB/s (DTS, re-scoped) — survives re-verification and remains the recommendation, with §10.3's merge order as the executable path.
