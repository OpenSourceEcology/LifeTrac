# 2026-05-10 Mixed-Domain Tuple PA11 Pair Discriminator Halted Run (Copilot v1.0)

## Objective

Test paired mixed-domain 3-bit tuples where only PA11 toggles between low and high
within each tuple family, to determine whether PA11 is the dominant classifier.

Tuple families:

- Family A:
  - 82a: PA11=LOW,  PC11=HIGH, E11=HIGH
  - 82b: PA11=HIGH, PC11=HIGH, E11=HIGH
- Family B:
  - 83a: PA11=LOW,  PC12=HIGH, E12=HIGH
  - 83b: PA11=HIGH, PC12=HIGH, E12=HIGH

Baseline held:

- H7 halted
- PA9=ANALOG, PA10=ANALOG
- PB11=HIGH, C9=HIGH, C10=HIGH
- PF4 reset pulse before probe

## Method

Per case:

1. Apply halted cfg with tuple
2. Probe on /dev/ttymxc3 at 921600
3. Release with 99_release_and_reset.cfg

## Evidence

Run folder:

- DESIGN-CONTROLLER/bench-evidence/T6_mixed_tuple_pa11_pair_2026-05-10_2026-05-10_212843/

Files:

- 82a.openocd.log, 82a.probe.log, 82a.release.log
- 82b.openocd.log, 82b.probe.log, 82b.release.log
- 83a.openocd.log, 83a.probe.log, 83a.release.log
- 83b.openocd.log, 83b.probe.log, 83b.release.log

## Results

OpenOCD readbacks confirm tuple bits exactly as intended in all four cases.

Probe outcomes:

- 82a (PA11 low): ATI = 144 zero bytes, AT+VER? no bytes, VER_REQ timeout
- 82b (PA11 high): ATI no bytes, AT+VER? no bytes, VER_REQ timeout
- 83a (PA11 low): ATI = 144 zero bytes, AT+VER? no bytes, VER_REQ timeout
- 83b (PA11 high): ATI no bytes, AT+VER? no bytes, VER_REQ timeout

No BOOT_URC observed in any case.

## Key Finding

Within identical tuple families, toggling PA11 alone flips the signature class:

- PA11 low  -> zero-byte ATI class
- PA11 high -> full-silence class

This is now the strongest reproducible directional-gate indicator seen so far.

## Interpretation

1. PA11 is a dominant mode selector, but neither mode restores protocol ingress.
2. The missing unlock likely requires additional owner control not yet modeled,
   or timing/sequence coupling beyond static halted pin states.

## Conclusion

W1-7 remains red. This run does not recover VER_URC, but it materially tightens
root-cause classification by proving PA11-state dominance across mixed-domain tuples.

## Suggested Next Step

Run a sequence-sensitive test where PA11 changes relative to reset edge timing
for one family (pre-reset high, post-reset low and inverse), while keeping all
other tuple bits fixed and H7 halted.
