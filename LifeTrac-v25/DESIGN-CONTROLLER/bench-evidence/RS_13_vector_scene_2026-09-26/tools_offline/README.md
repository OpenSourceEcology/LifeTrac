# Offline (no-radio) harnesses from the RS-13.1 encoder work

The improvement and review rounds of 2026-10-03/04 built these in their
scratch space. They are kept so the next encoder or store change (the A16
fix, A8 relabel tuning) does not have to rebuild them. None of them touches a
board or a radio.

The bench session's leg scripts live in `../scripts/`. That includes the A16
reproduction `a16_sil.py`, which is the acceptance sweep for the A16 fix, and
`fold_emitter.py`.

**Paths:** several of these files hardcode the bench PC layout
(`C:/Users/dorkm/Documents/GitHub/LifeTrac…`, the scratchpad, and the Windows
wallpapers under `C:/Windows/Web/` used as photo input). Edit the constant at
the top of the file, or pass the tree as the documented argument or
environment variable, before running elsewhere. Run them with `py -3` from
`DESIGN-CONTROLLER` unless a file says otherwise.

| dir | files | purpose |
|---|---|---|
| `encoder_perf/` | `bench.py`, `ab.py`, `compare.py`, `x8.py`, `sync_new.py` | Encoder timing and **byte-identity A/B**: `bench.py` encodes fixed photo sequences, `ab.py` runs an old and a new encoder package side by side, and `compare.py ref.json new.json` reports the first differing frame and the timing. `x8.py` is a CPU-only helper for the tractor X8 (no devices, no radio, `/tmp/lifetrac_bench`). This harness proved `c7f3926e` byte-identical while halving the encode time. |
| `encoder_perf/` | `exact_check.py`, `mean_check.py`, `order_check.py`, `micro.py`, `micro2.py`, `miss_count.py` | Bit-exactness checks of the vectorised numpy formulations, micro-timings, and memo-miss counts behind the perf rewrite. |
| `relabel_eval/` | `rl_common.py`, `rl_scenarios.py`, `rl_features.py`, `rl_screen.py`, `rl_e2e.py`, `rl_loss.py`, `rl_report.py`, `run_e2e.sh`, `run_loss.sh`, `bench_fn.py`, `old_rule_check.py` | The A8 relabel-trigger evaluation that chose `2d800ef7`'s per-pixel rule. Deterministic scenarios (no-change runs, pans, cuts) and the real encoder end to end. Reports churn on no-change scenarios against cut detection, plus the loss trade-off. `old_rule_check.py` shows the new A8 tests fail on the old rule. Start here for the "relabel 12–14 per leg on air" follow-up. |
| `store_resync/` | `episodes.py`, `digest_pattern.py`, `ttl_rows.py`, `exp_lostupd.py`, `exp_handover.py` | Replays of RS-13.1 captures through `tools/vector_dry_run.DryRun` (resync episodes, per-row DIGEST pattern, rows where `ttl_dropped` steps), and the two store experiments behind `c9361044` / `c922efbb`: a lost UPD → carousel repair → resync exit, and a lost safety-refresh key frame during hand-over. |
| `fhss_authority/` | `bindiff.sh`, `mutate.sh` | The binary diff and mutation checks run on the rejected `imp/fhss-authority` firmware change (see `../firmware/README.md`). |
| `legtool/` | `seqpeek.py`, `tablecheck.py` | Peek a capture's sequence numbers; check that every markdown table row has its header's column count (used on RESULTS.md). |
| `review_2026-10-04/` | `acq_sim.py`, `power.py`, `power_legs.py`, `switch_gaps.py`, `txgaps.py`, `drop_sweep.py` | Tools from the "remaining radio tests" review (RESULTS). `acq_sim.py` is a Monte-Carlo of FHSS cold acquisition (A7), a model only. `power.py` / `power_legs.py` give the statistical power of VECTOR-vs-control loss comparisons for a given number of legs. `txgaps.py` / `switch_gaps.py` give TX-gap histograms from `tx_daemon.log` (the 1.038 s switch gap). `drop_sweep.py` replays a capture with one extra frame dropped at a time. |
