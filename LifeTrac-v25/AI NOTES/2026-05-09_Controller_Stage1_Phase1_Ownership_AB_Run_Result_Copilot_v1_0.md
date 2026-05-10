# Controller Stage 1 Phase 1 Ownership A/B Run Result (Copilot v1.0)

Date: 2026-05-09
Board: 2E2C1209DABC240B
Evidence folder: `DESIGN-CONTROLLER/bench-evidence/T6_phase1_owner_ab_2026-05-09_183718/`

## Scope

- Execute Phase 1 from the next-test plan:
  - Test 1A: user boot while H7 held halted
  - Test 1B: user boot then explicit H7 resume/reset-run before probing

## Commands Used

- Test 1A
  - `boot_and_listen_hold.sh 8`
- Test 1B
  - `openocd ... -f 08_boot_user_app.cfg`
  - `openocd ... -f 99_release_and_reset.cfg`
  - `diag_uart_rxtx.py --dev /dev/ttymxc3 --baud 921600 --hwflow off`
  - `method_g_stage1_probe.py --dev /dev/ttymxc3 --baud 921600`

## Observed Results

- Test 1A (H7 halted)
  - `phase1a_rx.bin` captured 78 bytes.
  - Content includes early startup breadcrumbs (`RST:START`, `RST:MAIN`, `M:ENTER`, `M:SAFE0`) then short binary burst.

- Test 1B (H7 resumed)
  - `phase1b_diag_hwflow_off.txt` shows substantially more outbound binary traffic than Test 1A.
  - Despite changed outbound behavior, `phase1b_stage1_probe_after_resume.txt` still fails active query:
    - no `BOOT_URC` in initial window
    - no ASCII fallback bytes
    - timeout waiting for `VER_URC` (`0x81`) to `VER_REQ` (`0x01`)

## Interpretation

- H7 ownership/runtime state influences observed outbound traffic shape.
- Host->L072 ingress remains blocked even after explicit H7 resume.
- Therefore Phase 1 does not clear ingress; move to Phase 2 route-control capture/diff.

## Next Step (From Master Plan)

- Run Phase 2 GPIO snapshot/diff:
  1. snapshot in ROM-working state
  2. snapshot in user-failing state
  3. diff MODER/IDR to produce candidate control nets for single-pin perturbation
