# 2026-05-10 Targeted Owner-Net Expansion (PC11/PC12/PE11/PE12) Halted Run (Copilot v1.0)

## Objective

Continue one-hot owner-net expansion under the locked halted baseline to test whether
any of these controls unlock `VER_REQ -> VER_URC`:

- `PC11=HIGH` (74a)
- `PC12=HIGH` (75a)
- `PE11=HIGH` (76a)
- `PE12=HIGH` (77a)

Baseline held constant for all cases:

- H7 halted during probe window
- `PA9=ANALOG`, `PA10=ANALOG`
- `PA11=LOW`
- `PB11=HIGH`, `PC9=HIGH`, `PC10=HIGH`
- PF4 reset pulse before probe

## Method

Per case:

1. Apply halted OpenOCD cfg (force GPIOs + PF4 reset, no resume)
2. Run probe: `method_g_stage1_probe.py --dev /dev/ttymxc3 --baud 921600 --boot-timeout 2.0`
3. Release H7 via `99_release_and_reset.cfg`

## Evidence

Run folder:

- `DESIGN-CONTROLLER/bench-evidence/T6_targeted_ownerexp_pc11_pc12_pe11_pe12_2026-05-10_211610/`

Files:

- `74a.openocd.log`, `74a.probe.log`, `74a.release.log`
- `75a.openocd.log`, `75a.probe.log`, `75a.release.log`
- `76a.openocd.log`, `76a.probe.log`, `76a.release.log`
- `77a.openocd.log`, `77a.probe.log`, `77a.release.log`

## Results

### 74a (`PC11=HIGH`)

Readback confirms intended state and halted lock.

- `C11=1(HIGH)` confirmed in OpenOCD
- Probe: no `BOOT_URC`, ATI `144` zero bytes, no `AT+VER?`, `VER_REQ` timeout

### 75a (`PC12=HIGH`)

Readback confirms intended state and halted lock.

- `C12=1(HIGH)` confirmed in OpenOCD
- Probe: no `BOOT_URC`, ATI `145` zero bytes, no `AT+VER?`, `VER_REQ` timeout

### 76a (`PE11=HIGH`)

Readback confirms intended state and halted lock.

- `E11=1(HIGH)` confirmed in OpenOCD
- Probe: no `BOOT_URC`, ATI `144` zero bytes, no `AT+VER?`, `VER_REQ` timeout

### 77a (`PE12=HIGH`)

Readback confirms intended state and halted lock.

- `E12=1(HIGH)` confirmed in OpenOCD
- Probe: no `BOOT_URC`, ATI `144` zero bytes, no `AT+VER?`, `VER_REQ` timeout

## Interpretation

1. None of `PC11/PC12/PE11/PE12` one-hot highs unlock host-protocol ingress.
2. All four cases remain in the same failure family as prior halted runs:
   zero-only ATI payload with no protocol response.
3. The owner gate likely requires either:
   - a different control net not yet toggled, or
   - a multi-net tuple (not one-hot) across control domains.

## Conclusion

W1-7 remains red. One-hot expansion through PA11, PA12, PB12, PC11, PC12, PE11, PE12
has not yielded a `VER_URC` recovery state under the halted PA9=ANALOG baseline.

## Recommended Next Step

Move from one-hot to constrained multi-net tuples informed by ABX00043 owner-net extraction,
while preserving the same halted/probe/release sequencing discipline.
