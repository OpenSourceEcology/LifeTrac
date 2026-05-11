# 2026-05-10 Constrained Tuple PA11-High Owner-Net Expansion Halted Run (Copilot v1.0)

## Objective

Evaluate constrained 2-bit tuples with `PA11=HIGH` under the halted `PA9=ANALOG`
baseline to determine whether combining PA11 with adjacent owner nets unlocks
`VER_REQ -> VER_URC`.

Tuples tested:

- 78a: `PA11=HIGH + PC11=HIGH`
- 79a: `PA11=HIGH + PC12=HIGH`
- 80a: `PA11=HIGH + PE11=HIGH`
- 81a: `PA11=HIGH + PE12=HIGH`

Baseline held in all runs:

- H7 halted during probe window
- `PA9=ANALOG`, `PA10=ANALOG`
- `PB11=HIGH`, `PC9=HIGH`, `PC10=HIGH`
- PF4 reset pulse before probe

## Method

Per case:

1. Apply halted tuple cfg (no resume)
2. Probe: `method_g_stage1_probe.py --dev /dev/ttymxc3 --baud 921600 --boot-timeout 2.0`
3. Release H7 with `99_release_and_reset.cfg`

## Evidence

Run folder:

- `DESIGN-CONTROLLER/bench-evidence/T6_tuple_pa11high_ownerexp_2026-05-10_2026-05-10_212516/`

Files:

- `78a.openocd.log`, `78a.probe.log`, `78a.release.log`
- `79a.openocd.log`, `79a.probe.log`, `79a.release.log`
- `80a.openocd.log`, `80a.probe.log`, `80a.release.log`
- `81a.openocd.log`, `81a.probe.log`, `81a.release.log`

## Results

All OpenOCD readbacks confirmed intended tuple highs with H7 halted.

Probe signature was identical in all four tuple cases:

- `BOOT_URC`: not observed
- `ATI`: no bytes observed
- `AT+VER?`: no bytes observed
- `VER_REQ`: timeout waiting for `VER_URC (0x81)`

## Interpretation

1. PA11-high constrained tuples produce a stable **full-silence** class
   (stronger than zero-byte ATI cases), but still no protocol ingress.
2. This suggests PA11-high likely pushes the path into a mode that suppresses
   observed UART response entirely, without restoring valid host command handling.
3. `VER_URC` remains unrecovered, so these tuples are not the missing unlock state.

## Conclusion

W1-7 remains red. Constrained 2-bit tuple expansion around PA11 did not recover
`VER_REQ -> VER_URC`.

## Recommended Next Step

Move to mixed-domain 3-bit tuples that include one PA11-high case and one PA11-low case
for each candidate family, with immediate repeats, to separate directional gating effects
from hard-disable states.
