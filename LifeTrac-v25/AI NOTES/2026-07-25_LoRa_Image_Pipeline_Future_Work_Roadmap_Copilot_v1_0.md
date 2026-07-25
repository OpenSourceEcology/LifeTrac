# LoRa Image Pipeline — Future Work Roadmap
## Speed, Efficiency, and Reliability Improvements

**Document version:** v1.0
**Date:** 2026-07-25
**Author:** GitHub Copilot (assistant-owned document)
**Baseline commit:** 6bb93912 (+ DTS BW500 host wiring, this session)
**Companion docs:**
- `CODE REVIEWS/2026-07-23_LoRa_Comm_and_Image_TX_RX_Optimization_Review_Copilot_v1_0.md` (Phase 1–4 plan + exit targets)
- `CODE REVIEWS/2026-07-23_LoRa_Phase3_Phase4_Optimization_Implementation_and_Hardware_Verification_Copilot_v1_0.md`
- `2026-05-19_FCC_Part15_902-928_Compliance_Notes_Copilot_v1_0.md` (regulatory constraints)

---

## 1. Where the system stands (measured, 2026-07-24/25 bench)

All numbers from 120 s live-air runs, two Portenta X8 boards ~1 m apart,
`run_live_radio_monitor.ps1`, synthetic tile-delta frames (~150–275 B),
evidence archived under `DESIGN-CONTROLLER/bench-evidence/`.

| Run | Config | TX goodput | rx_frames | published | timeouts | Evidence |
|---|---|---|---|---|---|---|
| 24 | v2, profile 0 (fixed 915, BW250) | ~307 B/s | 223 | 157 | 0 | `radio_monitor_20260724_214400` |
| 25 | v3, profile 0 | ~280 B/s | 250 | 178 | 0 | `radio_monitor_20260724_215521` |
| 26 | v3, profile 1 (FHSS 50 ch, slot clock) | **699 B/s** | 424 | 268 | 75 | `radio_monitor_20260724_220402` |
| 27 | v3, profile 0 after profile 1 (teardown gate) | ~280 B/s | 249 | 175 | 0 | `radio_monitor_20260724_220655` |
| 29 | v3, profile 2 (DTS BW500) — **first BW500 air test** | **1158 B/s median, 1434 peak @ 49 % util** | **893** | **689** | 13 | `radio_monitor_20260725_031922` |
| 30 | v3, profile 2, 12 fps saturation | **1755 B/s median, 1776 peak @ 75 % util** | **1209** | **968** | 31 | `radio_monitor_20260725_032313` |

Key architectural wins already landed:
- **v25.0.7 slot-clock FHSS** — time-grid TX + phase-locked RX follower;
  delivery survives packet loss (one loss costs one packet, not a 5–40 s
  re-acquisition). FHSS now *beats* fixed-channel 2×.
- **Depth-2 TX mailbox + v3 pipelining** — verified better under FHSS
  (268 vs 175 published) after the mailbox garbage-payload fix.
- **Profile activation fully defines radio state** — non-FHSS activation
  tears down scheduler/scan SM/slot clock (run-23 stale-state deafness).
- **DTS BW500 live** — profile 2 wired end-to-end (FRF pin, PHY contract
  `image_bw500`, host budget 930 ms/s); first air test decoded 893 frames
  with zero decode errors.

Plan exit-target scorecard:
- Phase 1 (≥ 400 B/s single-channel): **not met** (~307 B/s) — but obsoleted
  by profile 2 for bulk data; see §2.4 for whether to keep chasing it.
- Phase 2 (≥ 1 KB/s FHSS *or* ≥ 2 KB/s DTS): **DTS at 1.76 KB/s sustained
  under saturation (88 % of target)**; FHSS at 699 B/s (70 %). The
  remaining DTS gap is fragment underfill + turnaround (util plateaued
  at 75 %) — §2.2 batching is projected to close it.

---

## 2. Boost speed

### 2.1 Characterize DTS BW500 beyond the first saturation point
Run 30 (12 fps offered ≈ 2.9 KB/s) measured **1.76 KB/s sustained at 75 %
util** with zero decode errors — the radio saturated with ~25 % of the
930 ms/s budget still idle (v3 keeps only 2 frames in flight; small tail
fragments + UART turnaround eat the rest). Remaining work:
- Latency histogram at saturation (P-frame age TX→publish).
- 30-minute soak: watch `reassembler_timeouts` (31/120 s at saturation),
  QUEUE_FULL, UART overruns, thermal drift.
- Explain the 25 % idle: instrument inter-TX gap on the wire (RFCO
  timestamps) to split fragment-underfill vs turnaround loss before
  buying either fix.

### 2.2 Fill the fragments (both profiles; biggest FHSS lever)
Synthetic frames average ~215 B → mostly 2 fragments with a small tail
fragment; per-fragment overhead (8 B hop header + 5 B fragment header +
preamble 6.5–4.1 ms) is paid twice for ~1.2× the bytes.
- **Batch small frames**: pack multiple tile-delta frames into one
  fragment train up to the 247 B body ceiling (needs a length-prefixed
  concatenation in `image_tx_daemon` + reassembler passthrough — the
  0xFE fragment format already carries total length).
- **Validate with real camera output**: real keyframes chunk at max body
  size; synth underfill exaggerates overhead. Rerun the FHSS benchmark
  with `camera_service.py` live before optimizing further.
- Projected effect at FHSS: 699 → ~850–950 B/s (util 61 % → ~80 %).

### 2.3 Trim FHSS slot overhead (fine tuning, firmware)
Every 200 ms slot pays 12 ms TX head-start + 15 ms guard = **13.5 %**.
- Measure actual RX boundary-retune latency (standby→set_freq→settle→arm)
  with a scope or cycle counter; head-start only needs
  `retune_latency + anchor_skew + margin`. 12 → 8 ms likely safe.
- Guard covers PLL settle + loop jitter; 15 → 10 ms after measurement.
- Combined: +3.5 % duty ≈ +35 B/s at FHSS. Low priority vs 2.2.
- Do NOT raise `SLOT_MS`: 200 ms is load-bearing for the FCC per-channel
  dwell math (≤ 2 visits × ToA per 20 s window).

### 2.4 Decide the fate of the Phase-1 single-channel target
Profile 0 is bench-only. Its 400 B/s target predates working FHSS/DTS.
Options: (a) declare it superseded (recommended — profile 2 is the
single-channel production path and already 3.7× faster); (b) if kept:
raise `LIFETRAC_FRAG_AIR_CAP_MS` 170 → 200 (247 B bodies, 486 B/s
ceiling, only 384 µs of dwell margin — needs one RFCO sweep) + remove
the 50 ms `inter_cycle_s` floor in favor of token-bucket pacing.

### 2.5 Adaptive profile switching (roadmap, design first)
The FCC notes' production concept: FHSS BW250 for control/telemetry
(robust, hop-diverse) + DTS BW500 burst mode for image bulk. Requires:
- Clean profile handoff (activation teardown landed — half the work);
- A policy owner on the X8 (who decides when to burst);
- Link-quality feedback (RSSI/SNR per RFCO already on the wire).

---

## 3. Reduce wasted time

### 3.1 Wasted airtime
- **Keyframe period 60 s** (review P1/P6): a lost keyframe stalls the
  canvas up to 60 s — the largest *perceived* speed loss in the system.
  Restore 15–20 s period now that pacing exists; pair with the RX-side
  `req_keyframe` self-heal (already implemented, verified firing).
- **Duplicate-frame suppression**: at saturation the TX queue drops
  76 % of offered frames (`drop_full`) *after* MQTT delivery burned LAN +
  CPU. Move the drop decision earlier (publisher-side rate feedback) or
  make `camera_service` adaptive to the published `link_stats` topic.
- **Tail fragments**: see §2.2 batching.

### 3.2 Wasted bench/engineering time (dev-loop)
- **Harness NRST verification**: the inter-run reset is fire-and-forget
  (`gpio163` pulse piped to `Out-Null`; tractor SWD reset backgrounded).
  A failed reset produced run 23's 120 s false-negative and a multi-hour
  investigation. Add a post-reset gate: VER round-trip + uptime/boot-count
  check on both L072s; abort the run loudly if either board didn't reset.
- **PS 5.1 `*>` redirects write UTF-16**: every log filter needs
  NUL-stripping; standardize the harness on `| Out-File -Encoding utf8`
  or have Python readers handle it once (recurring per-session tax).
- **Deploy verification**: the harness pushes 7 files per run with
  `| Out-Null` — a failed push = daemons crash-loop on ImportError (cost
  one full run this session, twice historically: the 2026-05-25
  "rx_frames=0" harness trap). Compare `md5sum` board-side vs local after
  push; fail fast.
- **Faster verdicts**: 120 s runs are right for evidence, but a
  `-DurationS 30 -NoArchive` smoke tier for iteration would cut the
  edit→verdict loop from ~9 min to ~4.
- **Container start latency**: `docker run` cold-start ~5 s/board/run;
  a long-lived container with a restart-loop entrypoint could shave
  ~1 min per bench session (weigh against state-leak risk — the same
  class of bug the teardown fix just killed).

---

## 4. Increase reliability

### 4.1 Radio link
- **Parity fragments for keyframes** (`add_parity_fragments` exists,
  is tested, and is wired nowhere): k-of-n XOR parity on keyframes only
  repairs single-fragment loss without retransmission. FHSS runs show
  75 timeouts/120 s — most are single-fragment losses. Highest
  reliability-per-byte item on the list.
- **Reassembler timeout tuning**: current timeout vs the 200 ms slot
  cadence causes premature evictions under FHSS (13 timeouts even in the
  clean DTS run). Derive from live PHY: `max(3 s, 3 × expected_gap)`
  (review F11) using the active profile's ToA.
- **LOCK_LOSS_MS=2000 under slot clock**: with the boundary follower,
  lock-loss demotion is the *only* remaining full-reacquisition path.
  Consider raising to 4–6 s under a valid slot clock (grid coasts on the
  TCXO at ~0.12 ms/min drift) so brief fades don't trigger a 5–40 s scan.
- **TX-side clock re-anchor on silence**: TX re-derives its grid from its
  own clock — fine; but after a multi-minute host pause the RX side has
  aged out. Consider a lightweight beacon (1 header-only frame/s when
  idle) so the RX follower never starves. Cost: ~7 ms/s airtime.

### 4.2 Boards / harness
- **NRST + deploy verification** (§3.2 — reliability twins).
- **Watchdog for daemons**: tx/rx daemons have no supervision; a UART
  wedge or MQTT drop kills the pipeline silently. systemd-in-container or
  docker `--restart on-failure` + a health topic.
- **Tractor gpio163 dead** — L072 reset only via the fragile
  OpenOCD/SWD path (7–10 s, occasionally hangs). Root-cause the GPIO
  (pinmux? blown pad?) or add a hardware NRST jumper on the carrier.
- **USB gadget hang class** (repo memory `lifetrac-x8-reboot-wedges-usb`,
  2026-05-25 RX 2D0A): aggressive container kills + sysfs writes can wedge
  the X8's gadget controller until physical power-cycle. The harness
  should use graceful `docker stop` before `rm -f`, and detection
  (descriptor-read probe) belongs in a preflight.

### 4.3 Firmware invariants
- **RFCO under bench/DTS zeroes freq_hz** — an off-channel synth is
  invisible in logs (made run 23 undiagnosable from evidence alone). Add
  a bench-only FRF readback to the per-TX snapshot, or a low-rate
  FRF-echo URC.
- **Golden-vector test for the slot follower**: `sx1276_rx_slot_follow`
  has no host-linked TU (the clock TU covers arithmetic only). Add a
  scripted sequence test: anchor → boundary crossings → follows, skips
  under tx_busy, resets on demotion.
- **Boot-count/uptime in VER_URC**: cheap firmware change that makes
  reset verification (§3.2) and "did it reboot?" forensics one request.

---

## 5. Compliance runway (before any field deployment)
- **DTS PSD audit**: §15.247(a)(2) caps conducted PSD at 8 dBm/3 kHz.
  At +17 dBm into 500 kHz the margin is comfortable (~-8 dBm/3 kHz), but
  it must be *measured*, and the 6 dB occupied-bandwidth ≥ 500 kHz claim
  needs a spectrum capture on the actual Murata module (chirp BW ≠
  occupied BW by assumption).
- **FHSS channel-count rule**: 50 channels at BW250 satisfies
  §15.247(a)(1)(i); keep the wide-mask popcount validator as the gate.
- **EIRP bookkeeping**: antenna-gain CFG key exists; wire the 1-for-1
  conducted-power reduction rule (>6 dBi antennas) into the HW ceiling
  clamp before anyone screws on a mast antenna.

---

## 6. Prioritized queue (effort × payoff)

| # | Item | Effort | Payoff |
|---|---|---|---|
| 1 | DTS soak + latency histogram (§2.1) | none (runs only) | production confidence at 1.76 KB/s |
| 2 | Harness NRST + deploy verification (§3.2) | low (PS + VER probe) | kills two recurring false-negative classes |
| 3 | Parity fragments on keyframes (§4.1) | low (host-only, code exists) | biggest reliability win per byte |
| 4 | Keyframe period 60→15 s + self-heal validation (§3.1) | low | biggest *perceived* speed win |
| 5 | Frame batching to 247 B bodies (§2.2) | medium (host-only) | FHSS +20–35 %, DTS +10–15 % |
| 6 | Real-camera benchmark rerun (§2.2) | none (runs only) | validates all synth-derived numbers |
| 7 | v3 default for profile 1/2 (config) | trivial | locks in verified best pipeline |
| 8 | Reassembler timeout from live PHY (§4.1) | low | fewer spurious evictions |
| 9 | Slot head-start/guard trim (§2.3) | medium (needs measurement) | +3–4 % FHSS duty |
| 10 | Adaptive FHSS↔DTS switching (§2.5) | high (design doc first) | production architecture |
| 11 | DTS PSD/OBW lab measurement (§5) | external | field-legal DTS |
| 12 | gpio163 root-cause / NRST jumper (§4.2) | hardware | removes the most fragile bench dependency |

Items 1–4 are a single bench session; 5–8 one code+bench session each.
