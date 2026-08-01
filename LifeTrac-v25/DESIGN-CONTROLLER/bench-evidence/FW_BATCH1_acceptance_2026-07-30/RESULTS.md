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

## Still open from the acceptance checklist

- **F6 demotion/re-adoption test** — needs a forced demotion mid-run (e.g.
  one-sided traffic hold >2 s) while both nodes transmit; the
  `s_grid_adopted` flag is air-testable only.
- **F9 flag=0 build** — one build with
  `EXTRA_CFLAGS=-DHOST_ALLOW_REG_WRITE_DIAG=0`, then: arming works, profile
  switch completes, 0x1D write FORBIDDEN. After that soak, the default flip
  is its own commit.
- FHSS goodput under smooth pacing (812 B/s at 67% util here) has no
  pre-Batch-1 FHSS/3000 B baseline to compare against — collect one if FHSS
  throughput ever matters; DTS remains the image-path profile.

Passive regression check: nothing in the run log shows misbehaviour
attributable to F6's gate (steady lock, epoch monotonic, no re-scan churn) or
F9's gate (profile activation and RXCONT arming worked — the daemons' writes
are in the always-allowed value set by construction).
