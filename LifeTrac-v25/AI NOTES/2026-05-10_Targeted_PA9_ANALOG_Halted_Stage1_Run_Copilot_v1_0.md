# 2026-05-10 Targeted PA9=ANALOG Halted Stage1 Run (Copilot v1.0)

## Objective

Run the targeted halted experiments that isolate two hypotheses for the missing
`VER_REQ -> VER_URC` path on `/dev/ttymxc3` at `921600 8N1`:

1. H7 PA9 bus-contention hypothesis: set `PA9=ANALOG` while H7 is halted.
2. PA11 DIR/SEL hypothesis: compare `PA11=LOW` vs `PA11=HIGH` with all other
   known-good RX-path conditions held constant.

## Method

Hardware state for both tests:

- H7 CPU halted during probe window (no firmware reconfiguration race)
- `PA9=ANALOG`, `PA10=ANALOG`
- `PB11=HIGH`, `PC9=HIGH`, `PC10=HIGH`
- PF4 reset pulse applied and released (`F4=1` observed)

Executed sequence per case:

1. OpenOCD cfg apply (halt + force + reset, no resume)
2. Probe: `method_g_stage1_probe.py --dev /dev/ttymxc3 --baud 921600 --boot-timeout 2.0`
3. Release: `99_release_and_reset.cfg`

## Evidence

Run folder:

- `DESIGN-CONTROLLER/bench-evidence/T6_targeted_pa9analog_manual_2026-05-10_205045/`

Files:

- `70a.openocd.log`
- `70a.probe.log`
- `70a.release.log`
- `71a.openocd.log`
- `71a.probe.log`
- `71a.release.log`

## Results

### Case 70a (`PA11=LOW`)

OpenOCD readback confirms intended state:

- `IDR A9=0(ANALOG) A10=0(ANALOG) A11=0`
- `IDR B11=1 C9=1 C10=1`
- `IDR F4=1 (1=released)`
- `H7 CPU REMAINS HALTED`

Probe result:

- `BOOT_URC`: not observed
- `ATI`: `144 byte(s)` all `0x00`
- `AT+VER?`: no bytes
- `VER_REQ`: timeout waiting for `0x81 (VER_URC)`

### Case 71a (`PA11=HIGH`)

OpenOCD readback confirms intended state:

- `IDR A9=0(ANALOG) A10=0(ANALOG) A11=1(HIGH)`
- `IDR B11=1 C9=1 C10=1`
- `IDR F4=1 (1=released)`
- `H7 CPU REMAINS HALTED`

Probe result:

- `BOOT_URC`: not observed
- `ATI`: no bytes
- `AT+VER?`: no bytes
- `VER_REQ`: timeout waiting for `0x81 (VER_URC)`

## Interpretation

1. PA9 bus contention is not sufficient to explain TX ingress failure:
   with `PA9=ANALOG` and H7 halted, `VER_REQ -> VER_URC` still fails.
2. PA11 state changes behavior signature (`ATI` zeros vs ATI silence), but
   neither state enables valid host-protocol ingress.
3. The primary gate remains red: no valid `VER_URC` on `/dev/ttymxc3`.

## Updated Classification

- Confirmed: these controls perturb observable UART behavior.
- Not confirmed: a deterministic control state that restores protocol ingress.
- Remaining likely cause class: additional owner-net(s) not yet controlled in
  this matrix (beyond PA9/PA11/PB11/PC9/PC10).

## Recommended Next Actions

1. Expand halted matrix to adjacent owner candidates (e.g. PA12/PB12 and
   neighboring mux/translator controls from ABX00043 owner-net extraction).
2. Add immediate repeat pairs for every divergent signature (`ATI` zeros vs
   silence) to classify stability.
3. Keep H7 halted for all discriminators where PA9 mode matters; release only
   after probe capture.
