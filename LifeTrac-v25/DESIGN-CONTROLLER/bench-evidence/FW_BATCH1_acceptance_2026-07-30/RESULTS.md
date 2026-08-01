# Firmware Batch 1 — flash + on-air acceptance (2026-07-30)

Both L072s flashed with the PR #87 build (23,036 B load image of the 38,554 B
link, commit `adc04a0f`) via the established AN3155-over-ttymxc3 path
(`run_flash_l072.sh`, flasher RC=0 on both boards), then booted into the app
via `08_boot_user_app.cfg`. The H7-first ordering constraint does not bind on
this bench: the X8 daemons parse RX_FRAME_URC directly and both Python parsers
were updated tolerant-first; the H7 Method-G runtime is not in the bench loop.

Acceptance run: `radio_monitor_20260801_150631`, FHSS profile 1 (the profile
where hop telemetry is meaningful), 240 s, smooth pacing, echo off.

## F8 — PASS. Slot alignment is now measurable from the host.

New `phase_telemetry` line in the RX daemon, per stats interval:

    phase_telemetry: valid=42 invalid=0 pre_f8=0 last_epoch=14 last_hop=45 slot_off=[14..14]
    phase_telemetry: valid=41 invalid=0 pre_f8=0 last_epoch=15 last_hop=45 slot_off=[14..18]
    phase_telemetry: valid=43 invalid=0 pre_f8=0 last_epoch=16 last_hop=46 slot_off=[14..16]
    phase_telemetry: valid=42 invalid=0 pre_f8=0 last_epoch=17 last_hop=46 slot_off=[14..14]
    phase_telemetry: valid=41 invalid=0 pre_f8=0 last_epoch=18 last_hop=46 slot_off=[14..18]

- **Every** delivered frame carries the tail with flags=1 (`invalid=0`,
  `pre_f8=0` — both boards on new firmware, no legacy frames).
- **The epoch advances by exactly one per ~10 s stats interval** (14 → 18
  across five intervals) — 50 slots × 200 ms = 10 s per epoch, the first
  host-visible proof the slot clock runs at its designed rate.
- `rx_decode_err=0` across 660 frames; wire change fully backward-compatible
  (the daemons' `air_gap` len fields unchanged — the len byte excludes the
  tail, as designed).

## F7 — PASS. The phase byte now tells the truth.

`slot_off=[14..18]` under FHSS. The designed TX head-start is 12 ms; pre-F7
firmware sampled the byte at admission and would have reported ~12. The
observed 14–16 ms typical (18 max) is precisely 12 + the ~2.3–3.6 ms of
pre-key-up delays (standby transition, 1 ms PLL settle, estimates, FIFO
burst) that F7 now includes by sampling at header-pack time — with LBT off,
which is the daemons' steady state. Values sit far inside the 185 ms slot
budget; no straddles observed at this duty.

## F9 — PASS (flag=0 build, direct wire probe)

Both boards were flashed with an `EXTRA_CFLAGS=-DHOST_ALLOW_REG_WRITE_DIAG=0`
build (23,040 B) and probed over the wire (`f9_gate_probe.py`):

    PASS rxcont-arm:        reg 0x01 <- 0x85 ACKed
    PASS standby:           reg 0x01 <- 0x81 ACKed
    PASS modemconfig1-diag: refused (ERR_PROTO)      <- diag surface CLOSED
    PASS raw-tx-value-gate: refused (ERR_PROTO)      <- value gate holds
    PASS re-arm:            reg 0x01 <- 0x85 ACKed

Production arming and the profile machinery then ran a full 360 s FHSS bench
run on the same build (the F6 test below) — daemons connected, profile
activated, RXCONT armed, 1318 frames received, `rx_decode_err=0`. The run-31
failure class is structurally closed: a flag=0 build now works.

## F6 — PASS (forced demotion + Δepoch=2 recovery)

Run `radio_monitor_20260801_153738`, 360 s FHSS, with `synth_pub` frozen for
30 s at ~T+90 (injected mid-run), silencing the tractor for ~3 epochs:

    epoch  1..9    steady, 41-43 valid frames per ~10 s interval
    epoch 10       partial interval (valid=27) — silence begins
    epoch 12       delivery RESUMES (valid=32), epoch 11 skipped in silence
    epoch 13..31   steady 41-43/interval, nineteen clean epochs

The recovery frame arrived with the tractor at epoch 12 against a base
scheduler frozen at ~10 — **Δepoch = 2, beyond the ±1 drift tolerance**. On
pre-F6 firmware that frame is `REJECTED_EPOCH_DRIFT`, yet the scan SM still
re-LOCKs (unconditional `scan_feed_frame(true)`), leaving the base locked on
its stale grid delivering nothing, forever — the exact Defect A signature.
On F6 firmware: the >2 s silence demoted the lock, demotion cleared
`s_grid_adopted`, the first heard frame hit the UNANCHORED tier, was SNAPPED
at Δ=2, and full-rate delivery resumed within the same stats interval.

`slot_off` stayed [14..18] throughout — F7's truthful phase byte, unchanged
across demotion and recovery.

## Wrap-up

All four Batch 1 items are now verified on air. Both boards were re-flashed
with the DEFAULT (flag=1) build afterwards (RC=0 both) so the bench probes
(T2 TCXO, SPI-isolation) keep working; the F9 default flip remains its own
future commit per the soak rule.
- FHSS goodput under smooth pacing (812 B/s at 67% util here) has no
  pre-Batch-1 FHSS/3000 B baseline to compare against — collect one if FHSS
  throughput ever matters; DTS remains the image-path profile.

Passive regression check: nothing in the run log shows misbehaviour
attributable to F6's gate (steady lock, epoch monotonic, no re-scan churn) or
F9's gate (profile activation and RXCONT arming worked — the daemons' writes
are in the always-allowed value set by construction).
