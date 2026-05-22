# Phase 2.2 attempt — HOST-RACE hypothesis FALSIFIED (2026-05-22 T2 ~16:08)

## TL;DR

The Phase 2.1 verdict ("HOST-RACE limb, fix is drain + backoff in
`method_h_stage2_tx_probe_v2.py`") **does not survive direct test**.
Implementing the prescribed fix — drain_boot(2.0 s), drain_pending,
and three CFG_GET attempts with re-drain between each — produced
**5/5 identical TimeoutError** at PreProbeSleepS=0.05 s, with the
new in-band telemetry confirming the drains were effective.

## What the fix did

`emit_runtime_profile_enum` was rewritten (commit in working tree) to:

1. `drain_boot(link, 2.0)` — full 2 s read of the wire.
2. `drain_pending(link, quiet_s=0.30, max_s=2.0)` — bounded drain to quiet.
3. Three CFG_GET attempts at gaps {0, 100 ms, 350 ms} with timeouts
   {1.5, 2.0, 2.5} s; each retry re-runs `drain_pending(0.20, 0.8)`.
4. Emits `__PROFILE_DRAINED__=<n>` and `__PROFILE_ATTEMPTS__=<k>`.

Self-test (`--self-test-profile-emit`): 8/8 PASS, no regression in
parser.

## Cycle stdout (representative — all 5 cycles identical)

```
PULSE_DONE_AT=1779482410.229531375
PRE_PROBE_SLEEP=0.05
MODE: rx
dev=/dev/ttymxc3 baud=921600
INFO: post-VER drain queued type=0xA0 seq=1
INFO: post-VER drain queued type=0xA0 seq=2
RUNTIME_PROFILE_ENUM=ERR request_failed:TimeoutError
__PROFILE_DRAINED__=2
__PROFILE_ATTEMPTS__=3
=== W1-10 probe RX: single-board RX-liveness (window=0.5s) ===
INFO: drained type=0xA0 seq=3 during boot settle
BOOT_URC not observed during 1.0s settle (firmware likely already past boot — continuing).
T0a: VER warm-up OK fw=v0.0.0 build=00000000     <-- VER_REQ succeeds same boot
T0b: post-VER drain consumed 0 frames
T0c STATS(before): radio_state=4 (4=RX_CONT) ...
```

## Why this falsifies HOST-RACE

The Phase 2.1 verdict pinned the failure on STATS_URC frames poisoning
the link.request() correlator. The new telemetry shows:

- `__PROFILE_DRAINED__=2` — both STATS_URC frames were consumed by the
  drains **before** the first CFG_GET attempt. The queue was provably
  empty entering the request loop.
- `__PROFILE_ATTEMPTS__=3` — all three attempts were exercised. With
  re-drain between each, no URC could have re-entered the queue
  silently.
- Total in-budget time for the emitter: ~13 s (drain 2 + drain 2 +
  req 1.5 + sleep+drain ≈ 1 + req 2 + sleep+drain ≈ 1 + req 2.5).
- Failure rate: **5/5 = 100%, byte-identical** (probe stdout = 1935 B
  in every cycle).

There is no remaining "race" to fix on the host side — the host is
demonstrably ready, draining, and retrying. The CFG_GET_REQ is being
sent into a void.

## What still must be explained

- **AT+VER (HOST_TYPE_VER_REQ) succeeds on the same boot, just seconds
  later** (line `T0a: VER warm-up OK fw=v0.0.0 build=00000000`). So
  the firmware's request/response engine *is* functional in this
  boot window.
- **W1-10 then issues many REG_READ/REG_WRITE requests** (T1..T5) and
  they all succeed (the failure verdict comes from RF state, not from
  request timeouts).
- Therefore, the bug is **specific to CFG_GET_REQ for CFG_KEY_REG_PROFILE**,
  not to the host-side framing or to the firmware's general request
  pipeline.

## Candidate hypotheses (Phase 2.3)

H1. **Firmware doesn't implement CFG_GET for CFG_KEY_REG_PROFILE on
    this build (v0.0.0 / build 00000000).** The handler may be a
    stub that swallows the request without responding. Falsifier:
    grep the firmware for the CFG_GET dispatch table; check whether
    `CFG_KEY_REG_PROFILE = 0x14` is mapped.

H2. **CFG handler is registered but the value source is uninitialized
    until later in boot.** Falsifier: call `emit_runtime_profile_enum`
    a second time AFTER T0a in the same boot (post-VER, post-drain).
    If it succeeds, it's a boot-order issue inside firmware. If it
    still fails, it's not.

H3. **CFG_GET payload encoding mismatch.** The host sends
    `bytes([CFG_KEY_REG_PROFILE])` but the firmware may expect a
    different framing (e.g. `[key_len, key, ...]`). Falsifier: cross-
    reference `host_cfg_wire.c` decode path against the host emit
    site.

H4. **CFG response routing bug.** The firmware emits CFG_DATA_URC but
    with a sequence/seq mismatch the host's link.request() correlator
    discards. Falsifier: enable a raw-wire dump (already present in
    `HostLink`?) and see whether ANY CFG_DATA_URC frame appears
    during the request window.

## Recommended next step

Cheapest discriminator first: **H2 (re-emit after VER)**. Add a
one-shot second call to `emit_runtime_profile_enum` (or a parallel
helper) after `T0a` and re-run the discriminator. ~10 minute fix +
one cycle. This single test divides the hypothesis space cleanly
between firmware-side issues (H1, H3, H4 — same failure result) and
firmware-side boot-order issues (H2 — succeeds late in boot).

Failing H2 → escalate to firmware source inspection
(`firmware/murata_l072/host/host_cfg_wire.c` plus dispatch
table) and a wire-trace dump.

## Status updates

- Open Problems P1: **REOPENED** (was DIAGNOSED 16:05). Phase 2.1
  verdict's host-race attribution superseded by direct falsification.
- `method_h_stage2_tx_probe_v2.py` patch is **kept** — the longer
  drains and retries are defensible regardless, and the new
  `__PROFILE_DRAINED__` / `__PROFILE_ATTEMPTS__` tokens are useful
  instrumentation for the next round.
- Evidence: `bench-evidence/p1_cold_boot_2026-05-22_160757/` (5
  cycles, 0.05 s dwell, post-fix, all ERR TimeoutError).
