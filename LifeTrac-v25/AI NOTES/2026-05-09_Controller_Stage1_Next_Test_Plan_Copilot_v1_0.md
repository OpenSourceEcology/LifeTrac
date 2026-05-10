# Controller Stage 1 Next Test Plan (Copilot v1.0)

Date: 2026-05-09
Target board: 2E2C1209DABC240B
Objective: isolate the carrier-side ingress gate that blocks host->L072 in user mode.

## What Is Already Proven

- ROM bootloader ingress works (`0x7F` -> `0x79`) at `19200 8E1`.
- User firmware emits outbound traffic but still does not receive host ingress.
- `PA_11` (BOOT0) and `PF_4` (NRST) mapping is valid, but `PA_11` high post-boot does not restore ingress.
- RTS/CTS A/B diagnostic outcome is Case A (silent both modes), with `hwflow=on` reporting incomplete `stty` support on `/dev/ttymxc3`.

## Plan Structure

- Phase 1: run one high-discrimination ownership test (H7 halted vs H7 running).
- Phase 2: capture route-control state evidence from H7 GPIO snapshots.
- Phase 3: execute one focused candidate-net perturbation pass.
- Phase 4: only then decide between firmware workaround and hardware control-net fix.

---

## Phase 1 — H7 Route Ownership A/B (Highest Priority)

Purpose: determine whether ingress depends on H7 runtime ownership of route-control pins.

### Test 1A: User boot with H7 explicitly held halted

- Use existing helper:
  - `boot_and_listen_hold.sh` (uses `09_boot_user_app_hold.cfg`)
- Collect:
  - captured UART bytes
  - `openocd_boot.log`

Expected discriminator:
- If still silent, this confirms current failure signature under forced-halted H7.

### Test 1B: User boot then force H7 resume/reset-run before probing

- Sequence:
  1. Boot user app with `08_boot_user_app.cfg`
  2. Immediately run `99_release_and_reset.cfg` (explicit `resume`)
  3. Run `diag_uart_rxtx.py --hwflow off`
- Compare against Test 1A.

Decision gate:
- If ingress appears only when H7 is running: root cause is route/control ownership by H7 firmware state.
- If ingress remains absent: move to Phase 2 (static carrier net/control path).

---

## Phase 2 — Route-Control State Capture (No Guessing)

Purpose: identify which control pins change between working ROM path and failing user path.

### Test 2A: GPIO snapshot in ROM-reachable state

- Run `01_halted_dump.cfg` after entering ROM-hold setup (`07_assert_pa11_pf4_long.cfg` path).

### Test 2B: GPIO snapshot in user-mode failure state

- Boot user mode (`08` or `09` path), then run `01_halted_dump.cfg` immediately.

### Test 2C: Diff snapshots

- Diff MODER/IDR for banks/pins likely tied to mux/OE/SEL control.
- Prioritize pins that flip state between ROM-working and user-failing contexts.

Decision gate:
- At least one stable candidate control pin/net is identified for perturbation.

---

## Phase 3 — Candidate Control-Net Perturbation

Purpose: validate one candidate at a time with reversible changes.

### Test 3A: Single-pin force test (reversible)

- For each candidate, while maintaining user-mode boot:
  - force pin state with OpenOCD write
  - run `diag_uart_rxtx.py --hwflow off`
  - record whether `ATI`, `VER_REQ`, or `PING_REQ` receives bytes

Rules:
- One pin/state change per run.
- No multi-pin changes in the same run.

Decision gate:
- First candidate that restores ingress becomes provisional gate net.

---

## Phase 4 — Fix Path Selection

If a gate net is found:

- Firmware workaround path:
  - set required control state from H7 side before Stage 1 probe/host traffic.
- Hardware/control path:
  - update route-control logic ownership and boot-time defaults.

If no gate net is found:

- Return to ABX00043 schematic extraction as the authoritative route map task.

---

## Command Anchors (Existing Helpers)

- ROM hold/flash path:
  - `run_flash_l072.sh`
  - `07_assert_pa11_pf4_long.cfg`
- User boot path:
  - `08_boot_user_app.cfg`
  - `09_boot_user_app_hold.cfg`
- H7 release:
  - `99_release_and_reset.cfg`
- Diagnostics:
  - `verify_l072_rom.sh`
  - `diag_uart_rxtx.py --hwflow off|on`
  - `method_g_stage1_probe.py`
- Snapshot/dump:
  - `01_halted_dump.cfg`

## Evidence Requirements Per Test

- openocd log
- UART capture / diagnostic stdout
- one-line verdict (`ingress=yes/no`)
- board serial + timestamp

Store all artifacts under `DESIGN-CONTROLLER/bench-evidence/T6_*` and link the final result note in `TODO.md`.
