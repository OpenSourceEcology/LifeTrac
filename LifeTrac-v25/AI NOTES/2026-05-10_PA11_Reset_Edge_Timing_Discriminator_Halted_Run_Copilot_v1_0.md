# 2026-05-10 PA11 Reset-Edge Timing Discriminator Halted Run (Copilot v1.0)

## Objective

Test whether PA11 transition timing relative to PF4 reset edge affects the ingress
classifier, while holding a fixed tuple family and halted baseline.

Fixed controls:

- PA9=ANALOG, PA10=ANALOG
- B11=HIGH, C9=HIGH, C10=HIGH
- C11=HIGH, E11=HIGH
- H7 halted during probe window

Timing cases:

- 84a: PA11 pre-reset HIGH -> post-reset LOW
- 84b: PA11 pre-reset LOW  -> post-reset HIGH

Each case executed twice for immediate reproducibility.

## Method

Per run:

1. Apply halted timing cfg with PF4 reset pulse and PA11 timing sequence
2. Probe: method_g_stage1_probe.py on /dev/ttymxc3 @ 921600
3. Release with 99_release_and_reset.cfg

## Evidence

Run folder:

- DESIGN-CONTROLLER/bench-evidence/T6_pa11_timing_edge_2026-05-10_2026-05-10_214142/

Files:

- 84a.openocd.log, 84a.probe.log, 84a.release.log
- 84a_r2.openocd.log, 84a_r2.probe.log, 84a_r2.release.log
- 84b.openocd.log, 84b.probe.log, 84b.release.log
- 84b_r2.openocd.log, 84b_r2.probe.log, 84b_r2.release.log

## Results

OpenOCD readback confirms post-edge PA11 state as designed:

- 84a post state: A11=0 (POST-LOW)
- 84b post state: A11=1 (POST-HIGH)

Probe signatures are stable across repeats:

- 84a / 84a_r2 (post-low):
  - ATI: no bytes
  - AT+VER?: no bytes
  - VER_REQ timeout (no VER_URC)

- 84b / 84b_r2 (post-high):
  - ATI: 143/144 zero bytes
  - AT+VER?: no bytes
  - VER_REQ timeout (no VER_URC)

No BOOT_URC observed in any run.

## Key Finding

With all other tuple controls fixed, the post-reset PA11 state remains the dominant
classifier:

- Post-low path -> full silence
- Post-high path -> zero-byte ATI class

This discriminator is reproducible under immediate repeats.

## Interpretation

1. Timing/edge sequencing does not produce VER_URC recovery in this tuple family.
2. The classifier tracks resulting PA11 state after reset edge, not random noise.
3. Remaining unlock likely requires additional owner net(s), alternate timing window,
   or non-static behavior not captured by current halted snapshots.

## Conclusion

W1-7 remains red. This run strengthens confidence that PA11-mode behavior is deterministic,
but not sufficient for protocol ingress recovery.

## Next Suggested Experiment

Run one family where PA11 is pulsed during reset-low window (toggle while PF4 held low),
then settle to the opposite post state, to test whether in-reset transitions matter beyond
simple pre/post level sequencing.
