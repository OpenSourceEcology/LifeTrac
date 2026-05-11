# 2026-05-10 Targeted Owner-Net Expansion (PA12/PB12) Halted Run (Copilot v1.0)

## Objective

Extend the halted PA9=ANALOG discriminator matrix to adjacent owner-net candidates:

- Case 72a: force `PA12=HIGH`
- Case 73a: force `PB12=HIGH`

while preserving the known probe baseline:

- H7 halted throughout probe window
- `PA9=ANALOG`, `PA10=ANALOG`
- `PA11=LOW`
- `PB11=HIGH`, `PC9=HIGH`, `PC10=HIGH`
- PF4 reset pulse before probe

Goal: determine whether either added control net enables `VER_REQ -> VER_URC`.

## Method

Sequence per case:

1. Apply halted OpenOCD cfg (force GPIOs + PF4 reset, no resume)
2. Run probe on `/dev/ttymxc3` at `921600 8N1`
3. Release H7 with `99_release_and_reset.cfg`

Probe command:

- `python3 /tmp/lifetrac_p0c/method_g_stage1_probe.py --dev /dev/ttymxc3 --baud 921600 --boot-timeout 2.0`

## Evidence

Run folder:

- `DESIGN-CONTROLLER/bench-evidence/T6_targeted_ownerexp_pa12_pb12_2026-05-10_210828/`

Files:

- `72a.openocd.log`
- `72a.probe.log`
- `72a.release.log`
- `73a.openocd.log`
- `73a.probe.log`
- `73a.release.log`

## Results

### 72a (`PA12=HIGH`)

OpenOCD readback:

- `A9=0(ANALOG) A10=0(ANALOG) A11=0 A12=1(HIGH)`
- `B11=1 C9=1 C10=1`
- `F4=1`
- H7 remained halted

Probe result:

- `BOOT_URC`: not observed
- `ATI`: `143` bytes, all `0x00`
- `AT+VER?`: no bytes
- `VER_REQ`: timeout waiting for `VER_URC (0x81)`

### 73a (`PB12=HIGH`)

OpenOCD readback:

- `A9=0(ANALOG) A10=0(ANALOG) A11=0`
- `B11=1 B12=1(HIGH) C9=1 C10=1`
- `F4=1`
- H7 remained halted

Probe result:

- `BOOT_URC`: not observed
- `ATI`: `143` bytes, all `0x00`
- `AT+VER?`: no bytes
- `VER_REQ`: timeout waiting for `VER_URC (0x81)`

## Interpretation

1. Neither `PA12=HIGH` nor `PB12=HIGH` restores host-protocol ingress under
   the halted PA9=ANALOG baseline.
2. Both cases collapse to the same signature as prior PA11-low halted runs:
   ATI returns zero-only bytes, active binary request times out.
3. This reduces probability that PA12/PB12 alone are the missing owner gate.

## Conclusion

Stage1 ingress gate remains red. Owner-net search must expand beyond PA11/PA12/PB12
single-bit toggles, with continued halted enforcement to prevent H7 runtime reconfiguration.

## Recommended Next Actions

1. Add next owner-net candidates from ABX00043 extraction in halted one-hot form
   (for example PC11, PC12, PE11, PE12 in combinations not yet tested under
   PA9=ANALOG+halted constraints).
2. Include immediate repeat runs for any non-matching ATI signature.
3. Keep release step isolated after probe capture only.
