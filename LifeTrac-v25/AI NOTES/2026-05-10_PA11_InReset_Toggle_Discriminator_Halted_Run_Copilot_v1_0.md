# 2026-05-10 PA11 In-Reset Toggle Discriminator (Halted Run)

## Objective

Test whether PA11 transitions during the PF4 reset-low window (not just pre/post static state) can alter ingress classification or recover valid `VER_REQ -> VER_URC` behavior.

## Method

- Baseline held constant from prior deterministic tuple branch:
  - `A9/A10=ANALOG`
  - `B11/C9/C10=HIGH`
  - `C11/E11=HIGH`
- Two new halted cfgs were executed with immediate repeat:
  - `85a`: PA11 toggled HIGH then LOW while PF4 held LOW, final post state LOW.
  - `85b`: PA11 toggled LOW then HIGH while PF4 held LOW, final post state HIGH.
- Execution flow per case:
  1. OpenOCD apply cfg with H7 halted
  2. Active probe on `/dev/ttymxc3` at `921600 8N1`
  3. OpenOCD release/reset (`99_release_and_reset.cfg`)

## Evidence

- Folder: `DESIGN-CONTROLLER/bench-evidence/T6_pa11_inreset_toggle_2026-05-10_2026-05-10_214318/`
- Key files:
  - `85a.openocd.log`, `85a.probe.log`, `85a.release.log`
  - `85a_r2.openocd.log`, `85a_r2.probe.log`, `85a_r2.release.log`
  - `85b.openocd.log`, `85b.probe.log`, `85b.release.log`
  - `85b_r2.openocd.log`, `85b_r2.probe.log`, `85b_r2.release.log`

## Results

### OpenOCD state confirmation

- `85a`: IDR confirms `A11=0 (POST-LOW)` with PF4 released.
- `85b`: IDR confirms `A11=1 (POST-HIGH)` with PF4 released.

### Probe behavior (all four runs)

- `BOOT_URC`: not observed.
- `VER_REQ -> VER_URC`: still timeout in every case.
- `AT+VER?`: no bytes observed.

Classifier split remained stable and repeatable:

- `85a` / `85a_r2` (post-low): ATI returns zero-only payload (`143` and `144` bytes).
- `85b` / `85b_r2` (post-high): ATI returns full silence (no bytes).

## Interpretation

PA11 transitions during the reset-low window did not break or override the previously observed PA11-dominant discriminator. The final post-reset PA11 state still determines the class (zero-byte ATI vs full silence), and neither class recovers protocol ingress (`VER_URC`).

## Conclusion

The in-reset-toggle hypothesis is closed for this branch: edge activity during PF4-low is not sufficient to recover Stage 1 ingress. Current evidence reinforces that PA11 is a strong mode selector but not the missing recovery gate.

## Next Move

Proceed to owner-net extraction and targeted control-net intervention around ABX00043 route-control ownership signals (OE/SEL/EN/DIR), then rerun a narrowed, evidence-first matrix with immediate repeats on each divergent signature.
