# LifeTrac v25 — Open Problems To Fix

**Date:** 2026-05-21
**Author:** assistant
**Scope:** carry-over work after the 2026-05-20 RF testing plan closed at
the end of Step 5 (matrix + T5b PASS, both opening and closing radio-sleep
audits PASS).

This document is the running backlog. Items are ordered by *what we should
do next on the bench*, not by severity. Each entry lists the symptom, the
evidence, the current best hypothesis, and the proposed next action.

---

## Exit criteria ("DONE WHEN") — landed 2026-05-21 v1.0.1

Per v4.0 §6, every P-item must have a machine-checkable closure rule.

| Item | DONE WHEN |
|---|---|
| **P1** | 20 consecutive cold-boot `rx_listen` launches produce zero `RUNTIME_PROFILE_ENUM=ERR` lines, and `check_run_profile.py` accepts every resulting log without `--allow-legacy`. |
| **P2** | `run_w2_02_image_over_lora_end_to_end_v2.ps1` log contains zero `NativeCommandError` blocks across a 384+ fragment run **AND** the Pester fixture `tools/tests/adb_wrapper.Tests.ps1` passes. |
| **P3** | `MIN_LORA_HOST_INTER_CYCLE_S=0.05` is a named host constant referenced by every TX loop; firmware mirrors a profile-aware `frames_per_dwell` recompute capped at 8; comment cites both the 2026-05-21 matrix and §15.247(a)(1) airtime-vs-wall-clock distinction. |
| **P4/T1** | `__W1_10B_LISTEN_READY__` and `__W1_10B_LISTEN_DONE__` both fire and parser-contract test stays 10/10. |
| **P4/T2** | One `ping_pong` run produces a censored-vs-raw RTT JSON pair where `len(censored.timeouts)` matches the live `__RTT_TIMEOUT__` token count. |
| **P4/T3** | A 2000-fragment burst completes with `duplicate_fragments` counted, `v2_redundant_copies_seen` matching configured N, and zero cross-frame fragment leaks under deliberate `frag_seq` wraparound. |
| **P5** | A 10-minute orchestrator run never goes more than 5 s without a console progress line (measured by timestamp gap on the tee log). |
| **P6** | Continuous: new PS scripts use the scoped-EAP adb wrapper; new analyzers reading PS-written JSON use `utf-8-sig`; new frame builders cite `FRAG_DATA_MAX_V2=59`. |
| **P7** | **Not opened — BOM audit run 2026-05-21, only 2 hits, both `matrix_meta.json` already covered by the Step-5 `utf-8-sig` analyzer patch. No latent victims.** |

---

## P1 — `RUNTIME_PROFILE_ENUM=ERR request_failed:TimeoutError` at rx_listen startup

- **Symptom.** On some runs (most recently the Step-5 matrix and the T5b
  burn-in), the rx_listen probe emits
  `RUNTIME_PROFILE_ENUM=ERR request_failed:TimeoutError` during boot before
  going on to operate normally.
- **Status.** Non-blocking: the listener proceeds, all downstream tokens
  (`__RX_FRAME__`, `__RX_LISTEN_STOPPED__`, etc.) fire, and verdicts are
  unaffected.
- **Why it matters.** The Step-1 patch was supposed to harden this exact
  path (drain + retry on the initial `RUNTIME_PROFILE_ENUM` query). The
  error proves the drain window is still too short under a heavy boot-URC
  storm — i.e. the harness still races the firmware on a cold start.
- **Best hypothesis.** Single retry isn't enough when the URC queue is
  long; we need either (a) a longer initial settle window, (b) a
  pre-drain pass that consumes URCs for *N* ms before issuing the first
  request, or (c) bounded retries with exponential backoff.
- **Next action.** Reproduce deterministically by power-cycling a board
  and immediately launching rx_listen. Capture the raw UART trace,
  measure URC depth at the failing instant, then pick between (a)/(b)/(c).

## P2 — Cosmetic adb-stderr `NativeCommandError` in T5b orchestrator

- **Symptom.** `run_w2_02_image_over_lora_end_to_end_v2.ps1` emits a
  PowerShell `NativeCommandError` block on every `adb push` because
  adb writes its transfer summary (`299 files pushed … MB/s`) to stderr
  even on success; `$ErrorActionPreference="Stop"` plus a `2>&1` pipe
  turns that into a fatal-looking error record.
- **Status.** Cosmetic only on this script (the burst still completes
  and T5b verdicts PASS) but it noises up the tee log and could mask a
  real adb failure.
- **Best hypothesis / known-good fix.** Same pattern that was landed in
  `tools/walk_power_falsification_matrix.ps1` on 2026-05-21:
  ```powershell
  $prevEAP = $ErrorActionPreference
  $ErrorActionPreference = 'Continue'
  try {
      adb -s $serial push ...
      adb -s $serial pull ...
  } finally {
      $ErrorActionPreference = $prevEAP
  }
  ```
  Scope it tightly around the adb invocations only so genuine errors
  elsewhere in the script still abort.
- **Next action.** Wrap every adb `push` / `pull` call in
  `run_w2_02_image_over_lora_end_to_end_v2.ps1`; verify with a dry T5b
  re-run that no `NativeCommandError` line appears and that a forced
  bad-serial value still aborts cleanly.

## P3 — Promote the host-cadence finding into the TX loop

- **Symptom.** Matrix Pass C (`inter_cycle_s=0.02`) showed 13.38 % PER
  vs 0.00 % at 0.05; LBT had no effect at 0.05. The 20 ms tick is the
  failure mode, not RF/PA power.
- **Status.** Diagnosis confirmed (Step 5), but the production TX loop
  is still free to schedule sub-20 ms cadences.
- **Next action.** Two options, pick one:
  1. Hard-floor the host TX loop at `inter_cycle_s >= 0.05` (or whatever
     the calibration says) and log a warning if a caller requests less.
  2. Run a calibration sweep that records `host_loop_iter_us` (already
     in the probe CSV from Step 2) against PER, and pick the
     defensible minimum cadence from the curve. Land it as a named
     constant in the host pipeline with a comment that cites the
     2026-05-21 matrix verdict.
- **Recommendation.** (2) first, (1) as the landing patch.

## P4 — T1 / T2 / T3 regression sweeps (carry-over from the Step-5 plan)

The Step-5 plan listed five hardware regressions (T1–T5). Only T4 (matrix)
and T5b (image-over-lora) were executed in this pass. The remaining three
are optional regressions that we should still close before declaring the
2026-05-20 plan fully retired.

- **T1 — `W1-10b rx_pair` sanity sweep on the patched harness.** Confirms
  the Step-1 parser-contract tokens (`__W1_10B_LISTEN_READY__`,
  `__W1_10B_LISTEN_DONE__`) fire on real hardware and that the harness
  patches didn't regress the legacy listener path.
- **T2 — `ping_pong` RTT run.** Exercises the Step-3 RTT censoring on
  real radios: emit `__RTT_TIMEOUT__` lines, feed through
  `tools/analyze_rtt.py`, confirm censored vs raw JSON split is sane
  (`--rtt-censor-margin-ms 20`).
- **T3 — `rx_pair` long-burst stress.** Pushes the URC queue and the
  v2-format reassembler well past T5b's 206-fragment volume; specifically
  watch `v2_redundant_copies_seen` and `duplicate_fragments` for
  drift, and verify `_completed_recent` LRU(64) is sized correctly.

**Next action.** Only run on user request — these are guard rails, not
blockers. T2 is the highest-value one because it's the only path that
actually exercises Step-3 code on hardware.

## P5 — Orchestrator output buffering hides live progress

- **Symptom.** `walk_power_falsification_matrix.ps1` and
  `run_w2_02_image_over_lora_end_to_end_v2.ps1` both buffer child-process
  stdout until each pass / TX burst completes; the console looks frozen
  for tens of seconds at a time even though the per-pass log file is
  updating in real time.
- **Status.** Cosmetic, but it makes long runs feel like hangs and led
  to redundant polling cycles during Step 5.
- **Best hypothesis.** `Tee-Object` and the PowerShell pipeline are
  flushing on line boundaries only after the child exits. Likely fixes:
  add a `[Console]::Out.Flush()` in the Python probes after each major
  event, or pipe through `Out-Host` instead of (or in addition to)
  `Tee-Object`.
- **Next action.** Add per-event flush in the probe Python first
  (cheapest, no PS plumbing changes); re-evaluate.

## P6 — Tooling/install hygiene (low priority)

Not a bug, but worth noting before the next contributor touches the bench:

- PS 5.1 `Out-File -Encoding utf8` writes a BOM. Any new analyzer that
  reads PS-written JSON must use `utf-8-sig`. The Step-5 analyzer fix
  is the canonical example.
- adb's transfer summary on stderr is a recurring trap. New PS scripts
  that invoke adb under `$ErrorActionPreference="Stop"` must use the
  scoped-EAP pattern from P2.
- L072 `TX_FRAME_REQ` is capped at 64 B; v2 wire header is 5 B, so
  payload cap is 59 B. Anything new that builds frames must respect
  this — `FRAG_DATA_MAX_V2 = 59` is the single source of truth.

---

## Suggested order if we keep going

1. P2 (cheap, removes future log noise).
2. P1 (diagnostic-first, then patch — actual hardware issue we keep dodging).
3. P3 step (2) — calibration sweep — followed by P3 step (1) landing.
4. P5 (cosmetic but improves every future run).
5. P4 — T2 on real radios, then T1/T3 if anything looks off.

---

## Copilot review and recommendations (v1.1)

This backlog is in good shape, but the repo has already moved past a few of
the older symptoms. Treat this as a triage overlay: some items still require
code changes, while others are now mostly verification/closure work.

### Current read of each problem

| Item | My status read | Recommendation |
|---|---|---|
| P1 runtime-profile timeout | Still open. The emitter currently drains only 0.5 s + 0.5 s and then tries 1.5/2.5 s requests before printing a single canonical line. The failure mode is plausible under a cold boot URC storm. | Fix next after P2. Make this path more patient and measurable, not merely longer by guesswork. |
| P2 adb stderr noise | Still open in the W2-02 v2 orchestrator. The matrix script has the known-good scoped-EAP pattern, but W2-02 still has raw `adb push/pull` calls. | Do first. It is cheap and prevents fake red text from hiding real failures. |
| P3 host cadence | Diagnosis is strong enough to land a guard. The calibration sweep is useful, but 20 ms is already proven bad and 50 ms is proven clean in the matrix. | Land a conservative 50 ms minimum now, then refine downward only if a later calibration justifies it. |
| P4 T1/T2/T3 | Mostly verification. `analyze_rtt.py` already contains timeout-censoring support, and `rx_listen` already emits `__RX_LISTEN_STOPPED__`. | Run T2 first as the only one that exercises the RTT censoring on real radios. T1/T3 are confidence runs. |
| P5 output buffering | Still annoying, but not a correctness blocker. Some Python paths already flush and W2-02 uses `python3 -u`; the frozen console is more about PowerShell process redirection than Python stdout alone. | Add a PowerShell-side live-tail helper or periodic progress echo rather than relying only on more Python flushes. |
| P6 hygiene | Keep as contributor guidance. | No immediate code needed except using these rules in new scripts. |

### P1 recommendation: make runtime-profile readout a robust handshake

Do not just add one more retry and call it done. The recurring
`RUNTIME_PROFILE_ENUM=ERR request_failed:TimeoutError` is a sign that the probe
is asking for CFG data while boot/diagnostic URCs are still draining. The
profile readout is a gate, so it should behave like a handshake:

1. Add a named helper such as `drain_startup_until_quiet(link,
  min_settle_s=1.5, quiet_s=0.25, max_s=4.0)` and use it before the profile
  CFG_GET.
2. Attempt profile CFG_GET with bounded backoff, for example 1.5 s, 2.5 s,
  4.0 s. Log per-attempt timing and drained-frame counts in human-readable
  lines before the single `RUNTIME_PROFILE_ENUM=...` machine line.
3. Preserve the single-line contract. `check_run_profile.py` expects exactly
  one canonical readout, so do not emit an early `ERR` and a later success in
  the same process.
4. If all attempts fail, include enough context in the error reason to make the
  next run actionable, e.g. `RUNTIME_PROFILE_ENUM=ERR timeout attempts=3
  drained=17 quiet_ms=250`.
5. Add a self-test around the pure formatting/classification pieces, but still
  verify on hardware with immediate cold-start `rx_listen` because this is a
  queue-timing problem, not a parser problem.

Out-of-the-box option: make `READY_URC` or `BOOT_URC` carry the runtime profile
enum in a future wire revision. That would remove the startup CFG_GET race
entirely. I would not do that now because it changes firmware wire behavior,
but it is the cleaner long-term design if profile identity becomes mandatory
for every artifact.

### P2 recommendation: make adb calls boring

Apply the scoped `$ErrorActionPreference = 'Continue'` wrapper around every
successful-noisy `adb push` and `adb pull` in W2-02 v2. Keep explicit
`$LASTEXITCODE` checks immediately after each call. Also add one forced-failure
smoke test with a bad serial or bogus remote path so the wrapper proves it does
not suppress real adb failures.

This is worth doing before any more image testing because the current warning
shape trains the operator to ignore red output. That is exactly how real bench
failures get missed.

### P3 recommendation: land the floor, then calibrate if needed

The matrix already tells a clear story: 20 ms cadence is bad, 50 ms is clean,
and LBT was not the culprit at 50 ms. I would land a named constant now:

```text
MIN_LORA_HOST_INTER_CYCLE_S = 0.05
```

Use it anywhere the host schedules repeated `TX_FRAME_REQ` calls unless the
loop is fully event-driven on `TX_DONE_URC`. If a caller requests less, clamp
up and print a warning that cites the 2026-05-21 matrix result. A later
calibration sweep can try 30/35/40/45/50 ms, but it should be an optimization,
not a prerequisite to preventing the known-bad 20 ms mode from recurring.

### P4 recommendation: use regressions to close specific questions

Run only the regressions that answer a question:

1. **T2 first:** proves the RTT timeout censoring works on real RF logs.
2. **T1 second:** proves the listener token contract stayed intact after the
  shutdown/profile-readout fixes.
3. **T3 only if T1/T2 or future image runs show queue pressure symptoms.** The
  T5b pass already exercised the v2 redundancy path enough for basic
  confidence; T3 is mainly for stress margin.

### P5 recommendation: live progress belongs in the orchestrator

Python flushes are good and should stay, but `Start-Process` with redirected
stdout will still make the console feel frozen. Add a small PowerShell helper
that tails the active log every second and prints only new machine-summary
lines such as `__WALK_POWER_STEP__`, `__RX_FRAME__` counts, `__W2_02_*`, and
`__METHOD_H_RC__`. That keeps the console alive without interleaving every raw
line from the child process.

### Updated order

1. P2: patch W2-02 adb push/pull wrappers and forced-failure smoke test.
2. P1: harden runtime-profile readout with startup quiet-drain, bounded
  backoff, and better failure context.
3. P3: land the 50 ms host-cadence floor for repeated TX loops.
4. P4/T2: run ping-pong RTT to verify timeout censoring on real radios.
5. P5: add live-tail progress helpers if the next long run still feels frozen.
6. P4/T1 and P4/T3 only if the above changes leave uncertainty.

### Final recommendation

Keep going, but resist the urge to run more broad sweeps before P2 and P1 are
fixed. Right now the remaining risk is mostly harness trust: fake adb errors,
startup profile readout races, and a known-bad 20 ms TX cadence. Fix those
three, then use T2 as the first hardware validation because it exercises a real
post-processing change. After that, the backlog should shrink from "open RF
problems" to a much cleaner set of optional confidence regressions.

*Signed:* GitHub Copilot, Open Problems Review v1.1 (2026-05-21)

---

## Copilot Review Update (v2.0) — Systemic Telemetry & Orchestration Roadmap

Building upon the v1.1 review, analyzing the entirety of P1-P6 reveals a shared theme: the test-harness is hitting the upper bounds of what a basic PowerShell wrapper + stateless UART scraper can do. To align with our ongoing push toward 50-Channel FHSS and visual camera deployments, consider these structural shifts:

### 1. P2 and P5: The Case for a Python `asyncio` Rewrite
Problems 2 (adb stderr noise) and 5 (PowerShell stdout buffer blocking) are not strictly LifeTrac bugs—they are fundamental limitations of PowerShell's `.NET` object pipeline fighting against native streaming.
* **Recommendation:** Stop applying band-aids (`$ErrorActionPreference`, `Out-Host` live-tailing) to PowerShell. Over the next development cycle, sunset the PS test wrappers and rewrite the orchestrators entirely in Python using `asyncio` and `subprocess`. Python manages `stderr` multiplexing and non-blocking stream captures infinitely better, easily resolving P2 and P5 forever.

### 2. P1: True URC Multiplexing on the Transport Layer
The `RUNTIME_PROFILE_ENUM=ERR` timeout during boot is currently being blamed on "queue timing". If polling `CFG_GET` fails because the listener is busy digesting unsolicited `READY_URC` or `BOOT_URC` messages, it suggests a transport-layer flaw: RPC responses and URCs are sharing the exact same parse channel without clean delimiters.
* **Recommendation:** Out of the box, introduce a dedicated prefix byte to the transport layer. For example, all URCs start with `!` while expected RPC responses start with `=`. This allows the python `rx_listen` transport parser to asynchronously dump URCs into an observation queue while strictly returning the RPC response to the blocking `CFG_GET` call, instantly eliminating "Cold Boot URC Storm" race conditions.

### 3. P3: Synchronize the 50ms Floor with the FHSS Hopper
Setting the TX host loop at 50ms (`inter_cycle_s = 0.05`) shouldn't just be an empirical stopgap—it is a strategic alignment. 
* **Recommendation:** The FCC 50-Channel FHSS design limits maximum dwell time on any one channel to under 400ms. A 50ms host transmission cadence gives exactly 8 full transmissions per legal channel dwell before a hop is mandatory. Document this 50ms floor specifically as the *"FHSS 8-Frame Hop Quantum"* so the firmware team understands the regulatory math behind the limit.

*Signed:* GitHub Copilot, Open Problems Review v2.0 (2026-05-21)

---

## Copilot Review Update (v3.0) - Decision Gates and Low-Risk Landing Path

This backlog is already high quality. The main improvement now is execution
discipline: separate immediate low-risk fixes from architectural rewrites, and
require one falsification check before any new root-cause claim.

### 1) What should land immediately

1. P2 first, unchanged:
  apply scoped ErrorActionPreference handling to all adb push/pull in the
  W2-02 v2 orchestrator, with explicit LASTEXITCODE checks and one forced
  bad-serial smoke test.

2. P3 guardrail now:
  enforce a named 50 ms minimum host cadence for repeated TX loops unless the
  path is truly TX_DONE-driven.

3. P1 hardening with observability:
  implement startup quiet-drain + bounded retries + single canonical
  RUNTIME_PROFILE_ENUM line, and include attempt/drain context in the final
  ERR reason when it fails.

### 2) What should be staged (not immediate blockers)

1. Full PowerShell-to-asyncio rewrite:
  good direction, but high migration risk for active bench workflows.
  Recommend a staged bridge:
  - phase A: extract shared parser/orchestrator logic into Python helpers,
  - phase B: keep PS entrypoints but call Python subprocess helpers,
  - phase C: retire PS wrappers only after parity tests pass.

2. Transport-level wire delimiter redesign:
  potentially strong long-term improvement, but it changes wire behavior and
  should be treated as a versioned protocol project, not a quick P1 fix.
  For current scope, keep startup handshake hardening in user-space tooling.

### 3) Falsification gates before declaring causes

For P1 and P3, run one explicit counter-test each before closing:

1. P1 counter-test:
  cold-boot storm replay with increased quiet-drain/backoff disabled vs
  enabled. If timeout rate collapses only in enabled mode, closure is valid.

2. P3 counter-test:
  compare 20/35/50 ms on same payload and RF setup. If 50 ms is stable and
  20 ms fails while 35 ms is transitional, keep 50 ms production floor and
  document the matrix in the constant comment.

### 4) Recommended next bench sequence

1. P2 patch + forced-failure smoke test.
2. P1 startup handshake hardening + cold-boot replay evidence.
3. P3 cadence floor landing + 20/35/50 ms confirmation run.
4. P4/T2 RTT censoring validation on hardware.
5. P5 live progress helper only if operator visibility is still poor after 1-4.

### Final recommendation

Keep the near-term path conservative and evidence-driven: land P2/P1/P3 with
minimal surface-area changes, validate with explicit falsification runs, then
decide whether architectural rewrites are still necessary based on measured
residual pain. This preserves momentum and avoids reopening transport and
orchestration layers before the current blockers are truly closed.

*Signed:* GitHub Copilot, Open Problems Review v3.0 (2026-05-21)

---

## Copilot Review Update (v4.0) — Sharpening the Backlog

The v1.1/v2.0/v3.0 reviews converge on a sensible execution order
(P2 → P1 → P3 → T2 → P5). This addendum pushes on five specific places
where the prior reviews accepted a hypothesis or framing that doesn't
fully hold up.

### 1. P1: distinguish host-side drain race from firmware-not-ready

v1.1 and v3.0 both prescribe a host-side quiet-drain + bounded backoff
handshake. That fixes the case where URCs are clogging the parse channel.
It does **not** fix the case where the L072 firmware itself is not yet
in a state that can answer `CFG_GET RUNTIME_PROFILE_ENUM` (radio init,
SX1276 reset sequence, profile-table load all still in flight).

Cheap discriminator, run before committing to the v3.0 P1 counter-test:

1. Cold-boot 10–20 times.
2. From the L072 banner / `__BOOT_DONE__` token, log `t_banner_ms`.
3. From every CFG_GET attempt, log `t_attempt_ms` and `t_response_ms`.
4. If first successful CFG_GET consistently lands at `t_banner_ms + Δ`
   with Δ tightly clustered (e.g., 800–1200 ms), it is **firmware not
   ready** — host drain cannot beat it. Fix is a firmware-side
   `__PROFILE_READY__` URC (or, cleaner long-term, embed the profile in
   the existing `READY_URC` per v2.0 §2's "out-of-the-box" note).
5. If response time varies with URC depth at the moment of request, it
   really is a host-side queue race, and the v1.1 handshake is the
   right fix.

This is one extra evening of evidence that prevents shipping a
host-side drain that the firmware-not-ready case will silently defeat.

### 2. P3: the "FHSS 8-Frame Hop Quantum" framing is off by ~50%

v2.0 §3 documents the 50 ms cadence as "8 full transmissions per legal
channel dwell" (8 × 50 ms = 400 ms = the §15.247(a)(1) per-channel cap).
Mathematically correct, but **it consumes the entire legal dwell window
with zero margin for ToA jitter, retries, LBT backoff, or hop-retune
latency.** A frame whose ToA stretches even 5 ms over budget would push
the 8th frame past the 400 ms cap.

Two cleaner options:

- **A. 67 ms floor → 6 frames/dwell, ~100 ms margin** (≈ 25% headroom).
  Pure-cadence-only enforcement; simplest for the host loop.
- **B. Keep 50 ms floor but cap `frames_per_dwell = 6`** (force a hop
  after 6 frames or 300 ms, whichever first). This preserves the v3.0
  P3 landing but adds a counter that prevents the 8th frame from
  existing.

I recommend **B**, because the matrix-proven safe cadence stays 50 ms
and the FCC compliance gate becomes an additional counter the firmware
already needs for FHSS dwell accounting anyway. Either way, the
constant comment should cite **both** the 2026-05-21 matrix result and
the §15.247(a)(1) headroom calculation, not just one.

### 3. P2 forced-failure smoke test belongs in CI, not as a one-shot

v1.1 and v3.0 both prescribe "one forced-failure smoke test" after
applying the scoped-EAP wrapper. That validates the patch today; it
does **not** prevent the next refactor from silently regressing the
"real errors still abort" guarantee.

Convert the smoke test into a permanent fixture:

```powershell
# tools/tests/adb_wrapper_aborts_on_bad_serial.Tests.ps1
Describe 'adb wrapper' {
  It 'aborts when given a non-existent serial' {
    { Invoke-AdbScoped -Serial 'NOPE' -Args @('shell','echo','ok') } |
      Should -Throw
  }
  It 'survives benign stderr on successful push' {
    Invoke-AdbScoped -Serial $env:LIFETRAC_TX_SERIAL -Args @('push',...) |
      Should -Not -Throw
  }
}
```

Pester runs in CI/PowerShell-Gallery image; this is a ~30-line addition
that closes the regression door P2 keeps creaking open.

### 4. P5 is the same root cause as P2; the cheap fix is a sidecar file

v1.1 prescribes a PS live-tail helper; v2.0 prescribes a full Python
asyncio rewrite; v3.0 stages the rewrite. All three are correct, but
the **cheapest immediate fix** is neither:

Have each long-running child process write a tiny `progress.txt`
sidecar (overwrite-truncate, one line per major event with
ISO-8601 timestamp + event tag). The orchestrator polls it every 1 s
and prints only the diff. This bypasses both `Tee-Object` buffering and
PowerShell `.NET` stream multiplexing entirely — the sidecar is a file,
not a stream — and it costs ~10 lines in the probe + ~20 lines in the
orchestrator.

If/when the asyncio rewrite lands, the sidecar contract becomes the
input to the new orchestrator's progress UI. Nothing wasted.

### 5. T3 is higher-value than T1; promote it

v1.1 ranks T3 (long-burst stress) last; v3.0 keeps that order. The risk
profile argues the opposite:

- T1 confirms a token-contract change that the harness patches already
  exercise on every run that touched it. Low novel-information yield.
- T2 confirms one specific post-processing change (RTT censoring) on
  real RF. High novel-information yield.
- **T3** exercises `_completed_recent` LRU(64) under sustained load.
  The LRU is the most likely **silent-corruption** site: a `frag_seq`
  that rolls past the window can let stale fragments of a long-finished
  frame leak into a new frame's reassembly. T5b's 206-fragment run is
  too short to cycle the LRU. T3 is the only test that touches this.

New ordering: **T2 (validation) → T3 (silent-corruption hunt) → T1
(token-contract confidence)**. T1 stays last because its failure mode
is loud, while T3's failure mode is quiet and image-degrading.

### 6. Every backlog item needs a "DONE WHEN" line

The document gives each P-item a symptom, hypothesis, and next action,
but **no exit criterion**. The next session reads "P1 is open" and has
no machine-checkable way to know when to close it. Add a single line
per item, for example:

| Item | DONE WHEN |
|---|---|
| P1 | 20 consecutive cold-boot rx_listen launches produce zero `RUNTIME_PROFILE_ENUM=ERR` lines, and `check_run_profile.py` accepts every resulting log without `--allow-legacy`. |
| P2 | `run_w2_02_image_over_lora_end_to_end_v2.ps1` log contains zero `NativeCommandError` blocks across a 384-fragment run, AND the §3 Pester fixture passes. |
| P3 | The 50 ms floor + (recommended) `frames_per_dwell=6` cap is a named constant referenced by every TX loop, with a cite-comment linking the matrix result and the §15.247 headroom calc. |
| P5 | A 10-minute orchestrator run never goes more than 5 s without a console progress line (measured by a sidecar tail). |
| P4/T2 | One ping_pong run produces a censored-vs-raw RTT JSON pair where `len(censored.timeouts)` matches the live `__RTT_TIMEOUT__` token count. |
| P4/T3 | A 2000-fragment burst completes with `duplicate_fragments` counted, `v2_redundant_copies_seen` matching the configured N, and zero cross-frame fragment leaks under deliberate `frag_seq` wraparound. |

This makes the backlog self-closing: an item without measurable exit
criteria is an item that will still be "open" next quarter for unclear
reasons.

### 7. One unaddressed lurking trap: UTF-8 BOM audit

P6 lists the `Out-File -Encoding utf8` BOM trap as contributor guidance.
That framing implies it is forward-only advice. It is almost certainly
**already** present in some existing analyzer/summary file that nobody
has stumbled on yet. Run once:

```powershell
Get-ChildItem -Recurse -Include *.json,*.csv,*.txt -Path bench-evidence,tools |
  Where-Object { (Get-Content -Encoding Byte -TotalCount 3 $_) -eq @(0xEF,0xBB,0xBF) } |
  Select-Object FullName
```

Any hit is a latent failure waiting for an analyzer that uses plain
`utf-8` rather than `utf-8-sig`. Add to the v4.0 backlog as **P7 —
audit existing files for stale UTF-8 BOMs** if any are found.

### Final recommendation (v4.0)

Adopt v3.0's near-term order **with these specific tightenings**:

1. P2 patch **plus** the Pester regression fixture (§3) — this turn.
2. P1 cold-boot discriminator (§1) **before** writing the v1.1
   handshake, so the fix lands in the right layer.
3. P3 with the `frames_per_dwell=6` cap (§2), not just the 50 ms floor
   alone.
4. P5 sidecar `progress.txt` (§4) as the cheap immediate fix; the
   asyncio rewrite stays staged per v3.0.
5. P4/T2 → **T3 promoted** → T1 (§5).
6. Add "DONE WHEN" exit criteria to every P-item (§6).
7. Run the UTF-8 BOM audit one-liner (§7); promote to P7 if hits.

The unifying point across all seven: **every prior addendum prescribed
the right actions; this one tightens what "correct" and "done" mean for
each, so the backlog can actually close rather than slowly accreting
addenda.**

*Signed:* **GitHub Copilot, Open Problems Review v4.0 (2026-05-21) —
exit-criteria & hypothesis-sharpening addendum to v1.1/v2.0/v3.0**

---

## Copilot Review Update (v4.1) — Correction to §2: stick with 8 frames/dwell

v4.0 §2 recommended reducing `frames_per_dwell` from 8 to 6 on the grounds
that 8 × 50 ms = 400 ms "consumes the entire legal dwell window with zero
margin." That framing was **wrong**: it conflated host wall-clock cadence
with FCC airtime accounting. This addendum corrects the recommendation.

### What §15.247(a)(1) actually caps

The 400 ms / 10 s rule bounds **on-air time per channel** (transmitter
energized on that center frequency), not the wall-clock window in which
those transmissions occur. At the current SF7/BW250/≤59 B profile:

| Quantity | Value |
|---|---|
| Host cadence (frame-to-frame wall clock) | 50 ms |
| Per-frame ToA (typical 59 B payload) | ~25–30 ms |
| 8 frames × ToA = **airtime per channel** | **~200–240 ms** |
| 8 × 50 ms = wall-clock window | 400 ms |
| §15.247(a)(1) cap | 400 ms airtime / 10 s |

8 frames/dwell at 50 ms cadence uses **~50–60 % of the legal airtime cap**,
with ~160–200 ms of airtime headroom remaining. There is no compliance
issue at the current profile.

### 6 vs 8 tradeoff (corrected)

| Concern | 6 frames | 8 frames | Verdict at SF7/BW250 |
|---|---|---|---|
| §15.247 hard cap | ~62 % headroom | ~50 % headroom | Both legal |
| Throughput per channel | baseline | +33 % | 8 wins |
| Retune amortization (PLL lock + cfg overhead) | lower | better | 8 wins |
| Robustness if SF/payload grows later | safer fallback | could exceed cap at SF9+ | Profile-dependent |

### Corrected recommendation: keep 8, add a profile-aware guard

Keep `frames_per_dwell = 8` as the production value at the current profile.
The only real footgun is that 8 stops being safe if the modulation profile
shifts (e.g., SF9/BW250 pushes per-frame ToA to ~90 ms; 5 frames already
exceeds 400 ms). Add a small runtime guard that recomputes
`frames_per_dwell` whenever `(sf, bw, payload_cap)` changes:

```c
/* Recompute on every CFG_SET that touches sf/bw/payload_cap. */
uint8_t frames_per_dwell =
    (uint8_t)((LEGAL_DWELL_US * 85U / 100U) / toa_us_estimate);
if (frames_per_dwell > 8U) frames_per_dwell = 8U;  /* throughput cap */
if (frames_per_dwell < 1U) frames_per_dwell = 1U;  /* sanity */
```

The `85 %` factor preserves ~15 % airtime headroom for ToA jitter, LBT
backoff slack, and retune latency. At SF7/BW250 this evaluates to 8 (the
throughput cap), so production behavior is unchanged today. At SF9/BW250
it would automatically clamp to 3 instead of silently breaking the cap.

### Updated v4.0 §2 ruling

Supersede v4.0 §2 with:

> **P3 lands as the 50 ms cadence floor + `frames_per_dwell = 8` cap +
> profile-aware recompute guard.** The constant comment should cite both
> the 2026-05-21 matrix result (50 ms is the empirically clean cadence)
> and the airtime-vs-wall-clock distinction (8 × ToA, not 8 × cadence,
> is what §15.247 measures).

All other v4.0 recommendations (§§1, 3–7) stand unchanged.

*Signed:* **GitHub Copilot, Open Problems Review v4.1 (2026-05-21) —
correction to v4.0 §2: keep 8 frames/dwell, add profile-aware guard**




---

## Execution log — 2026-05-21 (Phases 0, 1, 3.1 landed)

User directive in force: "yes please proceed with all" (from the 6-phase
Implementation Plan). This log records what landed in the desk-work
phases (Phases 0, 1, 3.1) that do **not** require live radios. Phases 2,
3.2, 3.3, 4, 5 still require a wake-radios block and remain pending.

### Phase 0 — Pre-flight (DONE 2026-05-21)

- **0.1 BOM audit (P7).** Repo-wide grep for unguarded
  `json.load(open(...))` / `Out-File -Encoding utf8` analyzer patterns
  returned 321 scanned, 2 hits, both `matrix_meta.json` files that the
  Step-5 `utf-8-sig` analyzer patch already covers. **P7 not opened.**
- **0.2 DONE WHEN table.** Added a 9-row "Exit criteria" section just
  below the doc header (lines 15-33). Every P-item now has a
  machine-checkable closure rule.

### Phase 1 — P2 scoped-EAP adb wrapper (DONE 2026-05-21)

- **1.1 Wrapper landed.** `Invoke-AdbScoped` added to
  `run_w2_02_image_over_lora_end_to_end_v2.ps1` (just after the
  `Get-Command adb` check). All 13 noisy `& adb push/pull/exec-out`
  call sites converted; redundant per-call `$LASTEXITCODE` throws
  removed (the wrapper now owns that check). Parser-validates clean
  (`[Parser]::ParseFile` errors=0). Only remaining `& adb` references
  in the file are inside the wrapper itself.
- **1.2 Pester fixture landed.** `LifeTrac-v25/tools/tests/adb_wrapper.Tests.ps1`
  created with 5 `It` blocks: bad-serial-throws, bad-serial-with-NoThrow,
  pscustomobject contract, EAP-restoration, and an env-gated smoke push
  (`-Skip:(-not $env:LIFETRAC_TX_SERIAL)`). Pester 5.7.1 installed in
  CurrentUser scope; fixture runs **4 passed, 1 skipped, 0 failed**.
- **1.3 Live T5b re-run.** Pending — needs radios woken. Acceptance
  criterion (per the DONE WHEN row for P2): zero `NativeCommandError`
  blocks across a 384+ fragment run in the tee log.

### Phase 3.1 — P3 host constants (DONE 2026-05-21)

`w2_02_host_pipeline.py` now exports:

```
MIN_LORA_HOST_INTER_CYCLE_S = 0.05       # walk_power matrix verdict
LEGAL_DWELL_US              = 400_000    # FCC 15.247(a)(1)
DWELL_HEADROOM_PCT          = 85
MAX_FRAMES_PER_DWELL_CAP    = 8          # v4.1: keep 8, not 6
frames_per_dwell(toa_us)    -> int       # profile-aware
clamp_inter_cycle_s(s, *, logger=None) -> float
```

Comment cites BOTH the 2026-05-21 walk_power matrix verdict AND the
FCC 15.247(a)(1) airtime-vs-wall-clock distinction (per v4.1 §2).

Unit-test anchors verified at the REPL:
- `frames_per_dwell(28_000)`  -> 8   (SF7/BW250, cap binds)
- `frames_per_dwell(115_000)` -> 2   (SF9/BW250, ToA binds)
- `frames_per_dwell(500_000)` -> 1   (huge ToA, floor binds)
- `clamp_inter_cycle_s(0.02)` -> 0.05 (with `P3-CLAMP:` warning)
- `clamp_inter_cycle_s(0.05)` -> 0.05 (no warning)
- `clamp_inter_cycle_s(0.10)` -> 0.10 (passthrough)

### Still pending (require wake-radios block)

- **Phase 1.3** — T5b live re-run for P2 closure (zero NativeCommandError).
- **Phase 2.1** — `tools/p1_cold_boot_discriminator.ps1`, N=20 cold boots.
- **Phase 2.2** — Branch on 2.1: either `__PROFILE_READY__` URC firmware
  patch (firmware-not-ready limb) or v1.1 `drain_startup_until_quiet`
  + bounded backoff in `method_h_stage2_tx_probe_v2.py` (host-race limb).
- **Phase 3.2** — Firmware mirror of the cadence/dwell constants on L072
  (CFG_SET path recomputes `frames_per_dwell` using identical formula).
- **Phase 3.3** — walk_power matrix re-run at
  `inter_cycle_s in {0.02, 0.035, 0.05}` to chart the cliff.
- **Phase 4** — Sidecar `progress.txt` for long-running probes + tail
  in the two orchestrators (P5 closure).
- **Phase 5** — T2 (ping_pong RTT censoring) -> T3 (2000-frag wraparound
  burst) -> T1 (W1-10b sanity).

### Closure status against the DONE WHEN table

| Item | Status | Note |
|---|---|---|
| P1 | OPEN | Pending Phase 2.1 cold-boot discriminator. |
| P2 | PARTIAL | Wrapper + fixture landed; live re-run pending (Phase 1.3). |
| P3 | PARTIAL | Host half landed; firmware mirror pending (Phase 3.2). |
| P4/T1 | OPEN | Phase 5. |
| P4/T2 | OPEN | Phase 5. |
| P4/T3 | OPEN | Phase 5. |
| P5 | OPEN | Phase 4. |
| P6 | LIVE | Patterns now in repo; new code must follow. |
| P7 | CLOSED | 2026-05-21 BOM audit: no latent victims. |

*Signed:* **GitHub Copilot, execution log 2026-05-21 (Phases 0, 1, 3.1)**
