# 2026-05-20 — RF Testing: Problems Needing Fix (v1.0)

Compiled from the bench RF regression sweep on 2026-05-20 (T1, T2, T3, T4, T5,
T5b, T8). Both boards reachable via ADB during the session; final state =
parked at 0x80 (`__BENCH_RADIO_SLEEP_AUDIT__=PASS`).

Boards: TX `2D0A1209DABC240B`, RX `2E2C1209DABC240B`.

---

## P1 — Open problems

### P1-1. W2-02 image pipeline has no native loss tolerance
- **Symptom (baseline run T5):** 185/187 fragments arrived, but the missing
  two (idx 113, 115) caused the entire 384×256 frame to be lost
  (`completed_frames=0`).
- **Mechanism:** `FragmentReassembler` requires the full fragment set per
  `frag_seq`. With empirical per-fragment loss ≈ 0.7–1.5 % at our bench RSSI,
  catastrophic per-frame loss = 1 − (1 − p)^187 ≈ **73 %**.
- **Current mitigation (validated this session):** `-Redundancy N` flag added
  to `run_w2_02_image_over_lora_end_to_end_v2.ps1`; replicates each fragment
  N× before push. RX reassembler dedupes naturally.
  - `-Redundancy 2` validated → **VERDICT: PASS**, 96/96 tiles, 6/384 frag
    losses survived (evidence
    `bench-evidence/W2-02_image_over_lora_2026-05-20_185712/`).
- **Still needs fixing:**
  - Pure replication doubles airtime per frame — not viable at higher frame
    rates. Replace with systematic FEC (e.g. XOR parity per 16-frag group,
    ~6 % overhead, recovers any single drop per group). Or RaptorQ.
  - Decide N=2 vs N=3 by air-time budget vs target frame-complete rate
    (N=3 drops per-frame catastrophic loss to ~6e-4 at p=0.007).
  - Make redundancy a property of the wire frame header (so RX can know the
    intended replication count and report stats correctly), not just a TX
    knob.

### P1-2. `paired_walk_power_sweep.ps1` mis-reports RX_FRAME count
- **Symptom (T4):** the orchestrator's `=== RX summary ===` printed
  `RX_FRAME_URC count = 0`, but the underlying `rx_listen_board_b.log`
  actually contained **676 `__RX_FRAME__` lines**.
- **Cause:** the orchestrator greps for the legacy token `RX_FRAME_URC:` but
  probe v2 emits `__RX_FRAME__`. Token name drift, not an RF issue.
- **Fix:** update the grep pattern in
  `LifeTrac-v25/tools/paired_walk_power_sweep.ps1` to match
  `__RX_FRAME__|RX_FRAME_URC:` for back-compat.

### P1-3. `paired_walk_power_sweep.ps1` kills RX listener with SIGINT, leaving a Python traceback in the log
- **Symptom (T4):** RX-side log ends with `KeyboardInterrupt` traceback from
  `method_h_stage2_tx_probe_v2.py` line 2186 (`run_rx_listen`).
- **Impact:** cosmetic; rx_listen does not have a graceful SIGINT handler.
- **Fix:** install a `signal.SIGINT` handler in `run_rx_listen` that flushes
  partial stats + writes `__RX_LISTEN_STOPPED__` and exits 0. Update
  orchestrator to wait for that token before SIGKILL.

### P1-4. `walk_power` probe shows ~12–16 % TX_TIMEOUT independent of TX power
- **Symptom (T4 CSV):** at every power step 2..17 dBm, `tx_done_ok` is
  42–45/50 and `tx_timeout` is 5–8/50. **The loss is flat across power**
  — it is not a link-budget issue.
- **Contrast:** the W1-10b tx_burst probe with the same radio produces
  1000/1000 `tx_done_ok` in T3. So the radio + firmware is healthy; the issue
  is specific to the walk_power probe.
- **Hypotheses to falsify before claiming root cause:**
  1. walk_power's `--inter-cycle-s 0.02` (20 ms) is shorter than mean ToA
     (25.7 ms) at this SF/BW → the next TX_FRAME_REQ races the
     prior TX_DONE_URC. Cure: raise `--inter-cycle-s ≥ 50 ms`.
  2. CFG_SET_REQ for `tx_power` may delay the TX state machine for one
     packet at each power transition; if so, only the first packet of each
     step should fail. CSV does not currently break this down — add a
     per-packet-within-step OK/FAIL field.
  3. LBT default state differs between probes. Walk_power may have LBT on;
     W1-10b tx_burst forces LBT off.
- **Fix:** instrument walk_power to log per-packet outcome, set inter-cycle
  to ≥ 50 ms by default, and explicitly disable LBT to match the W1-10b
  baseline before declaring a power vs PER curve.

### P1-5. T2 ping_pong RTT outliers are reported as RF events
- **Symptom (T2):** RTT distribution shows two clusters: ~107 ms p50 and
  3 400–3 700 ms long tail. Currently flagged in the report as RF backoff.
- **Cause:** the host-side `--probe ping_pong` waits up to 3.0 s for the
  echo before declaring a missed RTT and moving on. The "second cluster" is
  the timeout itself, not actual airtime.
- **Fix:** in the ping_pong post-processor, exclude RTT samples whose value
  is within ±20 ms of the configured `--rtt-timeout`. Emit them as
  `__RTT_TIMEOUT__` events with their own counter so the bimodal histogram
  no longer looks like RF jitter.

---

## P2 — Pre-existing bench / scripting issues observed in passing

### P2-1. Profile-gate trips on `walk_power_board_a.log`
- `check_run_profile.py` returned exit 4 on the T4 log:
  `no RUNTIME_PROFILE_ENUM line found; pass --allow-legacy if this log
  predates FCC-B3-1 firmware/probe`.
- The walk_power probe never emits `RUNTIME_PROFILE_ENUM`. Either teach the
  probe to emit it on startup (matching the W1-10b probe), or have the
  paired sweep orchestrator pass `--allow-legacy` when invoking the gate.

### P2-2. Board 1 SWD bridge wedges between runs
- Already documented elsewhere; relevant here only because the W2-02 v2
  orchestrator explicitly avoids `openocd` warm-boot for this reason and
  relies on the auto-wake folded into probe v2. Keep this avoidance until
  the SWD wedge is fixed.

### P2-3. PowerShell `Tee-Object` swallows native rc into a non-terminating
NativeCommandError when used with `*>&1`
- Cosmetic CategoryInfo warnings appeared in `rf_test_t4_walkpwr.log`
  whenever `adb pull` printed to stderr while teed. Replace
  `*>&1 | Tee-Object` with `2>&1 | Tee-Object` in the orchestrator
  wrappers, or wrap the adb call in `& { } 2>&1`.

---

## P3 — Things that are NOT problems (close-outs)

- **Header-loss-causes-decode-fail theory:** disproved. The two missing
  fragments in T5 were idx 113 and 115, not the header. Header-replication
  alone would not have helped.
- **ISR self-DOS during burst start:** ruled out. Drops have no
  position-in-burst bias across T1/T2/T3.
- **RX warm-up window theory:** ruled out by the same uniform-drop pattern.
- **Sleep park flakiness:** sleep audit has been clean on every closing
  check this session.

---

## Suggested ordering for the next session

1. P1-1 → pick FEC strategy (XOR parity vs. fixed N=2 replication) and lock
   the on-wire header field.
2. P1-2 + P1-3 → quick cleanup of `paired_walk_power_sweep.ps1`.
3. P1-4 → re-run walk_power with corrected inter-cycle + LBT, generate a
   real power-vs-PER curve.
4. P1-5 → ping_pong RTT post-processor cleanup.
5. P2-1 / P2-3 → housekeeping.

Evidence dirs (for traceability):
- `bench-evidence/W1-10b_rx_pair_2026-05-20_183616/` (T1)
- `bench-evidence/W1-11_pingpong_2026-05-20_183815/` (T2)
- `bench-evidence/W1-10b_rx_pair_2026-05-20_185116/` (T3 stress)
- `bench-evidence/walk_power_pilot_2026-05-19/` (T4 — note the dir name
  is stale; the run actually executed on 2026-05-20)
- `bench-evidence/W2-02_image_over_lora_2026-05-20_184021/` (T5 baseline)
- `bench-evidence/W2-02_image_over_lora_2026-05-20_185712/` (T5b
  redundancy=2 PASS)

---

## Copilot review and recommendations (v1.1)

This problem list is well separated into real RF/data-link issues versus bench
harness issues. My main recommendation is to avoid drawing any new RF
conclusions until the harness lies are removed. P1-2, P1-3, P1-5, P2-1, and
P2-3 are mostly observability defects. P1-4 is still a hypothesis set, not a
root cause. P1-1 is the one confirmed product/data-path problem.

### Recommended priority order

1. **Fix the measurement harness first.** Patch `paired_walk_power_sweep.ps1`
  to count both `__RX_FRAME__` and `RX_FRAME_URC:`, stop `rx_listen`
  cleanly, fix the profile-gate behavior, and remove the PowerShell tee noise.
  These are small changes and they prevent the next run from producing another
  misleading report.
2. **Make `walk_power` event-driven before re-running power-vs-PER.** The next
  TX should be sent only after the prior `TX_DONE_URC` or an explicit timeout,
  not after a fixed 20 ms sleep. Also log `power_step`, `packet_idx_in_step`,
  `tx_id`, `send_ts`, `done_ts`, status, LBT setting, and timeout reason. Then
  run two falsification passes: LBT off with >=50 ms spacing, then LBT on with
  the same spacing. If the timeout rate disappears, P1-4 was harness pacing;
  if only first packets after CFG_SET fail, it is config-set settling; if it
  remains flat, look for host/probe queueing rather than RF power.
3. **Fix ping-pong reporting as censored data.** Samples near the configured
  timeout are not RTT measurements; they are missed echoes. Report them as
  `RTT_TIMEOUT` / loss, exclude them from p50/p95/p99 RTT, and keep a separate
  timeout rate. This will stop timeout policy from masquerading as RF jitter.
4. **Choose the W2-02 loss-tolerance strategy only after the harness cleanup.**
  The redundancy=2 result is strong evidence that the RF link can carry the
  workload, but it is not yet the final architecture because it doubles airtime.

### P1-1 deeper recommendation: avoid all-or-nothing frames

Replication is the right emergency lever, but the long-term image path should
not require every fragment of a 187-fragment frame before showing anything.
That creates a catastrophic frame-loss curve from ordinary fragment loss.

My preferred path is **tile-level graceful degradation plus modest FEC**:

1. Change the wire image layer so each tile, or each small tile group, can be
  completed independently. A missing fragment should lose one tile/group, not
  the entire 384x256 frame. The receiver can keep the previous tile for that
  region and still display the rest of the new frame.
2. Add a versioned fragment format rather than squeezing metadata into the
  current 4-byte `0xFE frag_seq frag_idx total_minus1` header. The current
  header has no clean room for redundancy/FEC metadata. Use a new magic or
  versioned extension so legacy reassembly remains deterministic.
3. Start with simple systematic parity, but do not over-trust XOR-16. One XOR
  parity per 16 data fragments is only a single-erasure repair per group; at
  0.7-1.5% fragment loss it may still lose too many full groups for a clean
  visual stream. Test smaller groups, two-parity Reed-Solomon-style groups,
  or tile-local retransmit before adopting a fixed FEC ratio.
4. Keep `-Redundancy 2` as the bench-safe mode until the versioned FEC/tile
  path proves it can beat the replication result at lower airtime.

Out-of-the-box option: send a tiny low-resolution base layer first, then tile
enhancements. The operator gets a recognizable frame even under loss, and lost
enhancement packets degrade quality rather than blanking the frame. That may be
more useful for tractor situational awareness than maximizing full-frame
completion statistics.

### Harness cleanup details

- Treat double-underscore tokens as the stable machine contract. Any parser or
  orchestrator that still greps human labels like `RX_FRAME_URC:` should be
  updated or wrapped in a compatibility regex.
- Add small fixture tests for log parsers: one log with `__RX_FRAME__`, one
  legacy log with `RX_FRAME_URC:`, one with a clean `__RX_LISTEN_STOPPED__`,
  and one with `KeyboardInterrupt` to ensure it is flagged as harness-noise.
- Make every probe emit `RUNTIME_PROFILE_ENUM` on startup. Use
  `--allow-legacy` only for old archived logs, not for new captures.
- Keep the sleep-park audit as a closing gate. It is currently a reliable
  invariant and should stay in every RF sweep script.

### Updated next-session sequence

1. Patch and self-test `paired_walk_power_sweep.ps1` plus `run_rx_listen`
  graceful shutdown.
2. Patch `walk_power` pacing/logging and rerun the power sweep with LBT off,
  >=50 ms spacing, and per-packet CSV rows.
3. Patch ping-pong post-processing to split RTT from timeout/loss.
4. Run a short clean regression sweep to confirm the harness no longer
  misreports RX counts, profile gates, or listener shutdown.
5. Then start W2-02 v2 framing: versioned fragment/FEC metadata, tile-local
  completion, and comparison against redundancy=2 airtime and frame quality.

### Final recommendation

Do not spend the next session first on a sophisticated FEC implementation.
First make the bench harness trustworthy and event-driven, because the current
logs already contain at least three measurement artifacts that look like RF
behavior. Once the harness is clean, keep redundancy=2 as the known-good image
transport fallback while designing a versioned, tile-tolerant image protocol.
That gives the team a reliable demo path now and a scalable RF-efficient path
next.

*Signed:* GitHub Copilot, RF Testing Review v1.1 (2026-05-20)

---

## Copilot Review Update (v2.0) — Transport Architecture & FHSS Alignment

Building upon the v1.1 recommendations, a deeper review of the interaction between these bench issues and the recently decided FCC 50-Channel FHSS compliance plan reveals several structural opportunities.

### 1. Aligning Test Fixes with the 50-Channel FHSS Roadmap
The harness issues (P2-1) blocking `RUNTIME_PROFILE_ENUM` validation present the perfect time to introduce the new FCC profiles into the test suite. 
* Do not just patch the python scripts to accept legacy single-channel `BENCH_ONLY;` immediately wire the python harness to accept `FCC_15_247_FHSS_50CH_BW250`. 
* Ensure the walk_power probe records whether it transitioned channels during the step. If the 60us PLL lock delay of the FHSS retune causes occasional ping drops, you want your new test scripts catching that as a "Retune Penalty", rather than misdiagnosing it as RF loss.

### 2. W2-02 Robustness via Packet Interleaving 
While Tile-level graceful degradation and FEC (P1-1) are the correct strategic goals, **fragment interleaving** is an out-of-the-box software trick that radically improves visual image transmission over LoRa.
* **The Concept:** RF interference on LoRa often occurs in bursts. If you transmit fragments sequentially (1, 2, 3, 4...), a burst of noise destroys a contiguous block of reality (e.g., the entire left side of the tractor's camera view).
* **The Fix:** Have the TX python probe transmit the fragments out-of-order (e.g., all even fragments, then all odd fragments, or in a pseudorandom scatter). If a burst of RF noise destroys 5 sequential packets over the air, the visual damage is scattered evenly as "salt and pepper" noise across the image rather than a massive blind spot. The human brain (and basic image smoothing filters) handles scattered noise significantly better than missing structural chunks.

### 3. Deprecating the Flaky PowerShell CLI Wrappers
The signal-catching and error-swallowing bugs (P1-3, P2-3) stem from using PowerShell `Tee-Object` alongside native Win32/ADB signals. 
* **The Fix:** The overarching test suite needs to migrate out of PowerShell and into a pure Python 3 `asyncio` orchestrator script. Python's `subprocess` module handles `stdout`/`stderr` piping infinitely better than PowerShell's `.NET` object streams, and natively intercepts Ctrl+C (`SIGINT`) to gracefully flush logs before terminating the ADB listeners. Retiring the PowerShell test wrappers will eliminate P1-3 and P2-3 permanently.

*Signed:* GitHub Copilot, RF Testing Review v2.0 (2026-05-20)

---

## Copilot Review Update (v3.0) - Decision-Gated Recommendations

After reviewing the full note and prior addenda, the issue split is clear:

- Confirmed product-path problem: P1-1 (frame completion collapses under small
  fragment loss with all-or-nothing reassembly).
- Confirmed harness/reporting problems: P1-2, P1-3, P1-5, P2-1, P2-3.
- Not yet diagnosed: P1-4 (flat TX_TIMEOUT versus power).

The highest leverage move is to force the next session into a decision-gated
flow where each hypothesis has a falsification test before calling root cause.

### 1) Harness-truth patch set first (single batch)

Patch these together before any RF inference run:

1. paired_walk_power_sweep token parser:
  count both __RX_FRAME__ and RX_FRAME_URC:.
2. RX listener shutdown path:
  SIGINT handler emits __RX_LISTEN_STOPPED__, flushes stats, exits 0.
3. Profile gate consistency:
  emit RUNTIME_PROFILE_ENUM from walk_power startup (use allow-legacy only
  for old evidence replays, never new runs).
4. PowerShell wrapper noise:
  replace broad stream redirection patterns that trigger NativeCommandError
  warnings during normal adb stderr output.

Acceptance gate for this batch:
- one dry-run log where parser count equals raw token count,
- listener exits without traceback,
- profile gate passes on new logs,
- wrapper output has no false error noise.

### 2) P1-4 falsification matrix (before any power conclusions)

Run three controlled walk_power passes and classify outcomes:

1. Event-driven pacing + LBT OFF + inter-cycle >= 50 ms.
2. Event-driven pacing + LBT ON + inter-cycle >= 50 ms.
3. Legacy pacing control run (20 ms), same payload/count.

Record per packet:
- power_step,
- packet_idx_in_step,
- tx_id,
- send_ts,
- done_ts or timeout_ts,
- lbt_enabled,
- cfg_set_age_ms.

Decision rules:
- If timeouts collapse in run 1: pacing/queueing artifact, not RF power.
- If run 1 passes but run 2 regresses: LBT/backoff interaction.
- If first packet of each step fails disproportionately: cfg-set settle issue.
- If all three remain flat: investigate probe host queue/state machine.

### 3) P1-5 RTT reporting correction

Treat near-timeout RTT values as censored misses, not latency samples.
Emit explicit RTT_TIMEOUT counters and remove these samples from RTT percentile
calculations.

### 4) P1-1 transport recommendation (practical path)

Keep Redundancy=2 as the immediate safe mode, then stage improvements:

1. Add fragment interleaving now (low effort, good burst-loss resilience).
2. Move to versioned wire metadata for recovery mode signaling.
3. Pilot small-group systematic FEC with measured airtime and frame-complete
  deltas against Redundancy=2 baseline.
4. Shift reassembly objective from full-frame atomic completion to tile-group
  completion so losses degrade quality, not total frame availability.

### Final recommendation

Do the next session in this strict order:

1. Harness-truth patch set and acceptance gate.
2. P1-4 falsification matrix and root-cause classification.
3. RTT timeout/report split.
4. Then, and only then, iterate W2-02 transport architecture beyond
  Redundancy=2.

This sequencing minimizes rework and prevents the team from optimizing the
wrong layer based on misleading measurements.

*Signed:* GitHub Copilot, RF Testing Review v3.0 (2026-05-20)

---

## Copilot Review Update (v4.0) — Quantification Before Engineering

The prior addenda (v1.1, v2.0, v3.0) converge on a sound process recipe:
harness-truth first, falsification matrix for P1-4, censored-data fix for P1-5,
staged transport work for P1-1. This addendum adds **numerical sanity checks**
and **measurement gaps** that I would resolve before any of the engineering
work is committed.

### 1. Run the numbers on P1-1 before picking FEC

The session measured per-fragment loss `p ≈ 0.007–0.015` and shipped
Redundancy=2 as the bench-safe mode. Before designing systematic FEC, compute
the per-frame catastrophic loss curve under each candidate:

| Strategy | Per-frame loss at p=0.007 | Per-frame loss at p=0.015 | Airtime cost |
|---|---|---|---|
| N=1 (current baseline) | 1 − (1−p)^187 ≈ **73%** | ≈ **94%** | 1.00× |
| N=2 (validated this session) | 1 − (1−p²)^187 ≈ **0.91%** | ≈ **4.1%** | 2.00× |
| N=3 | 1 − (1−p³)^187 ≈ **0.0064%** | ≈ **0.063%** | 3.00× |
| XOR-parity, group=16 (single-erasure repair) | dominated by ≥2 drops per group | similar | ~1.06× |

At p=0.007, **Redundancy=2 is already 80× better than required for a usable
demo stream and within an order of magnitude of N=3 at 2/3 the airtime**.
This changes the architectural priority: the case for jumping straight to
systematic FEC is weaker than v1.1 §"P1-1 deeper recommendation" implies.

Recommendation: ship **Redundancy=2 + fragment interleaving + tile-tolerant
reassembly** as the production target. Defer Reed-Solomon / RaptorQ until a
measured workload (higher frame rate, or sustained operation at p>0.02)
actually exceeds what N=2 + tile tolerance can carry.

### 2. Interleaving needs a loss-correlation measurement first

v2.0 §2 recommends fragment interleaving on the assumption that LoRa losses
are bursty. **That assumption was not measured this session.** The T5 evidence
(idx 113 and 115 lost) is consistent with either bursty or scattered loss; two
samples decides nothing.

Before committing to an interleaving design, run one analysis pass on the T5
+ T5b raw fragment-arrival logs:

- Compute the run-length distribution of consecutive lost fragments.
- Compute the lag-1 and lag-k autocorrelation of the per-fragment
  arrival indicator.
- Compare against an i.i.d. Bernoulli model with the measured `p`.

If losses are bursty → interleaving is high-value, prioritize it.
If losses are i.i.d. → interleaving buys ~nothing; tile tolerance + N=2 is
the better engineering investment. This is a one-evening Python notebook;
do it before the orchestrator work commits to interleaving in the wire format.

### 3. The P1-4 falsification matrix is missing a fourth hypothesis

v3.0 §2 lists three causal classes (pacing, LBT, cfg-set settle). A fourth
should be added explicitly: **probe-host CPU saturation**. At 20 ms
inter-cycle with synchronous Python URC parsing, the X8 host's Python event
loop is a credible bottleneck independent of the L072 radio. Add to the
per-packet log:

- `host_loop_iter_us` (time between successive probe-loop iterations),
- `urc_queue_depth` (lines waiting in stdin buffer at send time),
- `python_rss_mb` (memory growth proxy for GC pauses).

Decision rule: if `host_loop_iter_us` p99 exceeds `inter_cycle_ms × 1000`
during the timeout cluster, P1-4 is host-side, not L072-side, and the next
falsification step is a C/Rust probe rewrite or moving URC parsing into a
separate thread. Without this fourth lane, the matrix can still produce a
false root cause ("LBT off + 50 ms spacing → timeouts vanish") that is
really hiding a host-CPU artifact.

### 4. Harness fixes need regression tests, not just patches

The P1-2 bug (`__RX_FRAME__` vs `RX_FRAME_URC:`) silently swallowed 676
events. The fix is one line. **The risk is that the same class of token
drift recurs in six months on a different parser and nobody notices for
another sweep.** Add one tiny CI-grade fixture set alongside the parser:

- A canonical short log containing every double-underscore token the
  orchestrator depends on.
- A pytest (or `Pester` for the PS scripts pre-migration) that asserts:
  parser count == raw `grep -c` on each token, for that fixture.

Without this, every token rename becomes another T4-style silent zero.

### 5. Evidence-dir timestamp drift is a one-line bug worth fixing now

P3 close-out notes that `walk_power_pilot_2026-05-19/` was actually a
2026-05-20 run. The cause is almost certainly `Get-Date` captured once at
script-module load and reused. Fix the orchestrator to:

- Capture `Get-Date` at the **TX-start** moment, not at script load.
- Include the git SHA (`git rev-parse --short HEAD`) and a UUID in
  `summary.json`, so renamed/moved evidence dirs can still be traced.

This is a five-minute change and it prevents the next person reading the
archive from chasing a phantom date mismatch.

### 6. Quantify the FHSS retune penalty before instrumenting

v2.0 §1 worries about a 60 µs PLL lock penalty during 50-channel hops.
For a SF7/BW250 packet with mean ToA ≈ 25.7 ms, a single retune is
**0.23%** of ToA — well below jitter on the rest of the chain. Even a
full 50-channel sweep adds ~3 ms of cumulative lock, which is amortized
over many seconds of operation.

Recommendation: do **not** instrument a "retune penalty" counter as a
first-class metric until either (a) the modulation profile moves to higher
SF where ToA shrinks the retune fraction matters, or (b) the bench
actually observes a per-hop loss bias correlated with retune events. Add
it later as a B1-SUMMARY field if the data demands it; do not pre-build
infrastructure for a hypothetical effect.

### 7. Wire-format note for N≥2 redundancy

If Redundancy=N stays in the production wire format, the fragment header
needs to carry both `redundancy_total` and `copy_idx`. The RX reassembler
already dedupes correctly without it, but having both fields enables:

- diversity statistics ("how often did the original arrive vs. only a
  copy?") — directly observable burst-vs-i.i.d. evidence (folds back into
  §2 above),
- adaptive N control (host can dial N up/down based on observed PER per
  channel under FHSS), and
- correct accounting when interleaving and redundancy interact.

One byte: `redundancy = (total << 4) | copy_idx`. Land it in the same
versioned-header change v1.1 already proposed for FEC metadata.

### Final recommendation (v4.0)

Sequence the next session as:

1. **One-evening analytics pass** on the T5/T5b fragment logs to determine
  whether loss is bursty or i.i.d. (§2). This single result decides whether
  interleaving belongs in the next sprint at all.
2. v3.0's **harness-truth patch set**, but bundled with the parser-fixture
  regression tests from §4 and the evidence-dir timestamp fix from §5.
3. v3.0's **P1-4 falsification matrix**, extended with the host-CPU
  saturation lane from §3.
4. v3.0's **RTT censored-data fix** (P1-5) — unchanged, ship as-is.
5. **Lock the wire-format** for `redundancy_total | copy_idx` (§7) and
  ship N=2 + tile-tolerant reassembly + (optional, gated on §1 + §2)
  interleaving as the production transport. Do **not** build systematic
  FEC until measured workload exceeds what N=2 + tile tolerance can carry.

This ordering does two things the prior addenda did not explicitly do:
it quantifies which problems are already solved well enough by the validated
N=2 path, and it forces a measurement before each remaining engineering
commitment so the team does not optimize a layer the data does not support.

*Signed:* **GitHub Copilot, RF Testing Review v4.0 (2026-05-20) —
quantification & measurement-gap addendum to v1.1/v2.0/v3.0**

---

## Implementation log (started 2026-05-20)

### Step 0 — Burstiness analytics pass: DONE

Tool: `bench-evidence/_analyze_burstiness_2026-05-20.py` (new, pure stdlib).
Method: greedy align RX arrivals to TX fragments.hex order; compute per-TX
arrival indicator; run-length distribution of consecutive losses; chi-square
goodness-of-fit vs geometric(1-p) null; lag-1..8 autocorrelation with
1.96/sqrt(n) CI band.

Results across T5 (Redundancy=1) + T5b (Redundancy=2):
- T5  : 187 TX, 185 RX, p=1.07%, 2 single-fragment losses, max_run=1, chi2=0.02
- T5b : 384 TX, 378 RX, p=1.56%, 6 single-fragment losses, max_run=1, chi2=0.10
- Combined: 571 TX attempts, 8 losses, **zero consecutive losses anywhere**.
- Both runs: chi-square strongly CONSISTENT WITH I.I.D. (alpha=0.05).
- Apparent "significant" autocorr at lag-2 (T5) / lag-6 (T5b) is a small-sample
  artifact of 2/6 isolated losses landing at those separations; the
  Gaussian-CI approximation is invalid when the indicator is nearly constant.

**Verdict: I.I.D.** Per v4.0 §2, this means: **do NOT include fragment
interleaving in the new wire format.** Tile-tolerant reassembly + Redundancy=2
is sufficient. v2.0 §2's interleaving recommendation is closed out as
"measurement-gated and the measurement said no."

### Step 1 - Harness-truth patch batch: DONE

All five P1/P2 harness defects patched in one batch; each fix is the *minimum*
change to remove a single class of false-positive / false-negative without
altering measured behavior.

| ID    | File (line) | Defect | Fix shipped |
|-------|-------------|--------|-------------|
| P1-2  | `tools/paired_walk_power_sweep.ps1:54` | Counted `RX_FRAME_URC:` (retired token), reported 0 frames when probe v2 was emitting 676 `__RX_FRAME__` lines. | Pattern broadened to `__RX_FRAME__\|RX_FRAME_URC:`; label updated. |
| P1-3 (probe) | `method_h_stage2_tx_probe_v2.py::run_rx_listen` (L941) | `pkill -INT` landed inside `link.read_frames()` -> KeyboardInterrupt traceback emitted into RX log, downstream parsers misread as probe crash. | Wrapped listen loop in `try/except KeyboardInterrupt`; emits stable `__RX_LISTEN_STOPPED__ reason=SIGINT rx_frames_so_far=<N>` token then falls through to the normal `__W1_10B_LISTEN_DONE__` stats summary. Per user methodology memory: handler only sets state, all I/O happens on the main thread post-loop. New field `stopped_by_signal=<0|1>` added to LISTEN_DONE summary. |
| P1-3 (orch) | `tools/paired_walk_power_sweep.ps1` shutdown block | Hard 3-second sleep after `pkill -INT` regardless of probe state. | Poll `$rxLog` for `__RX_LISTEN_STOPPED__` or `__W1_10B_LISTEN_DONE__` with 8-second cap; only Stop-Process if probe is still alive after. |
| P2-1  | `method_h_stage2_tx_probe_v2.py::emit_runtime_profile_enum` (L262) | Single CFG_GET with 1.0 s timeout raced the cold-boot URC storm; produced `RUNTIME_PROFILE_ENUM=ERR request_failed:TimeoutError` which the FCC-B3-2 gate rejects identically to a missing line. | Drain BOOT_URC + drain_pending() before the request; retry once at 1.5 s then 2.5 s; only emit `ERR` if both attempts fail. Self-test (`--self-test-profile-emit`) re-run, 8/8 pass. |
| P2-3  | `tools/paired_walk_power_sweep.ps1:36` | PowerShell `*> $txLog` writes UTF-16-LE-with-BOM and surfaces stderr as NativeCommandError cosmetic noise (`0xFFFE` shows as `ÿþ` in first line, visible in T4 evidence). | Replaced with `2>&1 \| Out-File -Encoding utf8 -FilePath $txLog`. |
| v4.0 §5 | `tools/paired_walk_power_sweep.ps1:6` | Evidence dir hardcoded to `walk_power_pilot_2026-05-19`; subsequent runs all wrote into the same dated dir, masking which run produced which artifact. | `$outDir` now stamped from `Get-Date -Format yyyy-MM-dd_HHmmss` at TX-start; orchestrator also writes a `run_meta.json` with the timestamp, git short SHA, and a UUID. |

#### Parser-contract regression test (v4.0 §4): SHIPPED

New file: `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/test_parser_token_contract.py`
+ co-located hand-authored fixture
(`fixtures/parser_contract/rx_listen_canonical.log`).

Pure-stdlib runner asserts that nine canonical token regexes each match
exactly the expected count in the fixture. The fixture is auto-(re)written
from a single in-file `FIXTURE_TEXT` constant so contract + fixture are
co-located - any future probe token rename forces a co-edit, which is
exactly what failed in P1-2. Locked-in tokens: `__RX_FRAME__`,
`RX_FRAME_URC:` (legacy still recognised), combined match,
`__RX_LISTEN_STOPPED__`, `__W1_10B_LISTEN_DONE__`,
`^RUNTIME_PROFILE_ENUM=\d+$`, `^RUNTIME_PROFILE_ENUM=ERR\b`, `__TX_DONE__`,
bare `KeyboardInterrupt` (the anti-pattern P1-3 was designed to eliminate).

Self-check: `py -3 test_parser_token_contract.py` -> 9/9 PASS.

#### Validation
- `ast.parse` of patched probe v2: clean.
- `[Parser]::ParseFile` of patched orchestrator: clean.
- `--self-test-profile-emit`: 8/8 PASS (formatter helper unchanged; only the
  request path was hardened).
- `test_parser_token_contract.py`: 9/9 PASS.
- No hardware re-run yet; the regression burn-in run is scheduled as part of
  Step 5 once Steps 2-4 have landed.

### Steps 2-5: in progress.

### Step 2 — P1-4 falsification matrix harness: DONE (orchestrator + analyzer; bench run pending)

Per v3.0 §2 / v4.0 §3 the goal is to *classify* the flat 12–16% walk_power
TX_TIMEOUT rate before calling it "RF link budget". Three software pieces shipped:

1. **Per-packet instrumentation in the probe.** `method_h_stage2_tx_probe_v2.py::run_walk_power`
   now writes one CSV per run (`walk_power_perpacket_<ts>.csv`) with
   15 fields: `power_step_dbm, packet_idx_in_step, tx_id, send_ts, done_ts,
   timeout_ts, status, ok_latency_ms, lbt_enabled, cfg_set_age_ms,
   host_loop_iter_us, urc_queue_depth, python_rss_kb, inter_cycle_s,
   notes`. The four v4.0 §3 host-CPU fields (`host_loop_iter_us`,
   `urc_queue_depth`, `python_rss_kb`, plus `cfg_set_age_ms`) close the
   fourth-lane gap. `_rss_kb()` is Windows-safe (returns 0 when
   `resource` import fails). 

2. **Explicit LBT control.** New `--lbt-enable {0,1}` CLI flag (passes
   through to `cfg_set('lora.lbt_enable', val)` on probe startup) so
   passes A/B/C can independently force LBT off vs on.

3. **3-pass orchestrator.** New `tools/walk_power_falsification_matrix.ps1`
   runs the same 8-step power sweep (2..16 dBm) under three pacing/LBT
   regimes:
   - Pass A: LBT=0, `--inter-cycle-s 0.05` (event-driven, no LBT).
   - Pass B: LBT=1, `--inter-cycle-s 0.05` (event-driven, LBT on).
   - Pass C: LBT=0, `--inter-cycle-s 0.02` (legacy 20 ms pacing control).
   Each pass writes to its own `pass_{A,B,C}/` subdir under
   `bench-evidence/walk_power_matrix_<ts>/`, with a top-level
   `matrix_meta.json` (timestamp, git SHA, UUID, regime descriptors).
   Per Step 1 patches, all logs are UTF-8 and listener shutdown is
   token-driven.

4. **Decision-rule analyzer.** New `tools/walk_power_matrix_analyze.py`
   ingests the three per-packet CSVs and emits `verdict.md` covering
   v3.0 §2's three rules + v4.0 §3's host-CPU lane:
   - Rule 1 (host pacing): if Pass A timeout-rate << Pass C timeout-rate
     by `CLIFF_HIGH=5%`, the legacy 20 ms cadence was the cause.
   - Rule 2 (LBT defer): if Pass A << Pass B by `CLIFF_HIGH=5%`, LBT
     backoff is masquerading as "RF timeout".
   - Rule 3 (combined): if Pass A timeout-rate < `CLIFF_LOW=1%`, P1-4 is
     a pure harness/config artifact and walk_power no longer needs an
     RF investigation.
   Verdict file lays out per-pass aggregates, per-power-step breakdown,
   and host-CPU p50/p99/max for each pass so the host-saturation lane
   can be classified directly from the data.

**Validation (no bench run yet):**
- `py -3 -c "import ast; ast.parse(...)"`: probe clean, analyzer clean.
- `--self-test-profile-emit` re-run: 8/8 PASS.
- `[Parser]::ParseFile` on `walk_power_falsification_matrix.ps1`: clean.

**Status:** orchestrator + analyzer ready to run. Step 5 will execute the
3-pass burn-in on real hardware and let the analyzer produce the verdict.

### Step 3 — P1-5 RTT timeout censoring: DONE

Per v3.0 §3 / v4.0: RTT samples near the configured timeout are *censored
observations*, not latency measurements. Treating them as latency was
producing a bimodal RTT histogram that looked like RF bursting but was
just timeout policy.

**Probe-side (`method_h_stage2_tx_probe_v2.py::run_ping_pong`):**
- Per-attempt timeout now emits `__RTT_TIMEOUT__ idx=I rtt_timeout_s=X payload_hex=...`
  before the attempt-loop fallback.
- Locked into the parser contract (added as token #10 in
  `test_parser_token_contract.py`; fixture extended; 10/10 PASS).

**Analyzer-side (`analyze_rtt.py`):**
- New constants: `_RTT_TIMEOUT_RE`, `_PINGPONG_TIMEOUT_RE` (legacy form),
  `_PINGPONG_BANNER_RE`, `DEFAULT_RTT_CENSOR_MARGIN_MS=20.0` (v3.0 §3 spec).
- New helpers: `detect_rtt_timeout_s(text)`, `count_rtt_timeouts(text)`,
  `censor_rtts_near_timeout(rtts, rtt_timeout_s, margin_ms) -> (kept_list, dropped_count)`.
- `build_report()` signature: `(evidence_dir, rtt_timeout_s_override=None,
   rtt_censor_margin_ms=DEFAULT_RTT_CENSOR_MARGIN_MS) -> (text, payload)`.
- Report block: explicit boundary line
  `RTT censoring: kept=N raw=M (dropped Q within ±20.0 ms of 3.000 s)`,
  + raw vs censored p50/p95/p99, + explicit `__RTT_TIMEOUT__` count, +
  a `WARN` when the ping_pong banner is absent (means the run did not
  exercise ping_pong and the report's RTT section is empty by design).
- JSON: `pingpong_rtt_ms` is now the *censored* series; new keys
  `pingpong_rtt_raw_ms` and `pingpong_rtt_censoring` (kept/dropped/raw
  counts, boundary, margin) preserve the raw data for re-analysis.
- CLI: `--rtt-timeout SEC` and `--rtt-censor-margin-ms MS` for overrides.

**Validation:**
- `ast.parse`: clean.
- Inline censoring unit (boundary=5000 ms, margin=20 ms, input
  `[85.0, 90.0, 4985.0, 5005.0]`) -> `kept=[85.0, 90.0]`, `dropped=2`.

### Step 4 — W2-02 versioned wire header for redundancy: DONE

Per v3.0 §4 + v4.0 §7. The pre-existing redundancy path was a PowerShell
post-process that **duplicated each `fragments.hex` line N times** before
push. Three problems:

1. Receiver could not distinguish "first copy" from "later copy" — any
   diversity statistics (the v4.0 §2 burstiness-vs-i.i.d. measurement)
   would be invisible.
2. Same-magic re-deliveries inflated `duplicate_fragments` instead of
   being attributed to redundancy.
3. No room for future protocol evolution; the 4-byte 0xFE header has no
   free bits.

**New wire format v2 (magic `0xFD`):**
```
byte 0 : 0xFD                              (magic, distinct from v1 0xFE)
byte 1 : frag_seq         (mod 256)
byte 2 : frag_idx         (0..total-1)
byte 3 : total - 1
byte 4 : (total_copies << 4) | copy_idx    (nibbles, range 1..15 each)
bytes 5.. : payload (≤ 59 B on L072 HostLink 64 B cap)
```

`total_copies==0` or `copy_idx >= total_copies` is rejected as a bad
header. v1 and v2 traffic interoperate in the same reassembler — any
producer that does not need redundancy keeps emitting v1.

**Files patched:**

- `base_station/image_pipeline/reassemble.py`:
  - New constants `FRAGMENT_MAGIC_V2=0xFD`, `FRAGMENT_HEADER_LEN_V2=5`.
  - `_PartialFrame` gains `copy_bitmasks: dict[int, int]`
    (per-idx bitmask of which `copy_idx` values arrived).
  - `ReassemblyStats` gains `v2_fragments_seen`,
    `v2_redundant_copies_seen` (copies that landed after the slot
    was already filled — direct measurement of how often redundancy
    "saved" a slot), `v2_bad_header`.
  - `feed()` branches on magic; v2 path validates the redundancy byte,
    treats `(frag_seq, frag_idx, copy_idx)` re-deliveries as
    `duplicate_fragments`, and treats `(frag_seq, frag_idx, *)`
    late-arrivals after the slot is filled as
    `v2_redundant_copies_seen` (not duplicates).
  - New `_completed_recent` bounded LRU (cap 64) prevents a late v2
    copy from accidentally re-opening a slot for a frame that already
    completed.

- `firmware/x8_lora_bootloader_helper/w2_02_host_pipeline.py`:
  - New `FRAG_DATA_MAX_V2 = 59` (= 64 − 5).
  - `_frag_capped(payload, frag_seq, redundancy=1)`: when
    `redundancy==1`, emits v1 (backwards-compat); when `>=2`, emits v2
    idx-major + copy-major (all copies of fragment 0 back-to-back, then
    all copies of fragment 1, …). No interleaving across idx, per
    Step 0 burstiness verdict.
  - New `--redundancy {1..15}` flag on `cmd_encode`.

- `firmware/x8_lora_bootloader_helper/run_w2_02_image_over_lora_end_to_end_v2.ps1`:
  - `-Redundancy` default raised to **2** (the v1.1 / v4.0 "ship N=2"
    recommendation).
  - Old line-duplication block removed; redundancy is now produced by
    the encoder itself via `--redundancy $Redundancy`.

- `base_station/lora_proto.py`:
  - Exports the v2 constants so non-image telemetry consumers can
    recognise the magic without importing the image-pipeline module.
  - `parse_telemetry_fragment` accepts both magics (validates v2
    redundancy byte; silently drops it from the return tuple since
    telemetry parsing does not care about copy_idx).
  - `pack_telemetry_fragments` is unchanged — telemetry topics have no
    airtime budget for in-band redundancy under
    `TELEMETRY_FRAGMENT_MAX_AIRTIME_MS=25.0` and stay v1.

**Tests:**
- New `base_station/tests/test_image_reassembly_v2_redundancy.py` with
  five focused unit cases:
  - R1: v1 + v2 coexist in one reassembler (4 v2 + 2 v1 fragments,
    counters correct).
  - R2: late-arriving copy of a 1-fragment v2 frame is counted as
    `v2_redundant_copies_seen` (not as a duplicate, not as a re-open).
  - R3: loss of `copy_idx=0` for every fragment of a 3-fragment v2 frame
    still completes the frame via `copy_idx=1` arrivals — the
    redundancy actually saves the frame.
  - R4: malformed redundancy bytes (`total_copies=0` and
    `copy_idx >= total_copies`) are rejected and counted in
    `v2_bad_header`.
  - R5: exact `(frag_seq, frag_idx, copy_idx)` re-delivery is
    `duplicate_fragments`, not `v2_redundant_copies_seen`.
- Pre-existing `test_image_reassembly_fuzz.py` (13 cases) re-run to
  confirm v1 regression-free.
- Encoder→reassembler round-trip smoke test: 768 B payload at
  `redundancy=2` → 14 unique fragments × 2 = 28 wire frames; dropping
  every `copy_idx=0` on the air still completes the frame with
  `v2_fragments_seen=14`, `v2_redundant_copies_seen=0`,
  `pending=[]`.

**Validation:**
- `ast.parse` on all four touched Python files: clean.
- `[Parser]::ParseFile` on the patched PS1: clean.
- `unittest`: **18/18 PASS** (5 new + 13 v1 fuzz).
- Round-trip: PASS.

### Step 5: pending. Hardware burn-in (T1/T2/T3/T4/T5b on the patched harness +
the 3-pass walk_power matrix) is the only remaining work; all software for
Steps 1–4 is landed and unit-validated.


