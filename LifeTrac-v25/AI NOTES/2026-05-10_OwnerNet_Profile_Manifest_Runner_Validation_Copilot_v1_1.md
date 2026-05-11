# 2026-05-10 Owner-Net Profile Manifest Runner Validation (Copilot v1.1)

## Scope

Validate the manifest runner end-to-end on a known profile branch and fix the runtime blockers that prevented evidence capture.

Profile executed:

- `pa11_inreset_toggle_recheck`

Run command:

```powershell
./run_owner_net_profile.ps1 -Target pa11_inreset_toggle_recheck -SkipDiagProbes
```

Evidence produced:

- `DESIGN-CONTROLLER/bench-evidence/T6_profile_pa11_inreset_toggle_2026-05-10_220440/`

## Fixes Applied To Runner

File:

- `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_owner_net_profile.ps1`

Changes:

1. Replaced fragile fixed-depth repo-root resolution with upward walk that finds nearest ancestor containing `DESIGN-CONTROLLER`.
2. Corrected PowerShell 5.1 compatibility issue (`Split-Path -LiteralPath ... -Parent`) by switching to `[System.IO.Directory]::GetParent(...)`.
3. Aligned adb command execution with known-good local pattern (`adb exec-out <command>`), which restored remote command/log behavior.
4. Added hard-fail checks on each pulled log file so missing evidence fails fast.

## Validation Outcome

The runner now:

1. Loads manifest profile.
2. Creates timestamped evidence directory.
3. Executes all case steps (`halted apply -> probe -> release -> pull logs`).
4. Produces all expected per-case artifacts (`.openocd.log`, `.probe.log`, `.release.log`).

Produced files (12 total):

- `85a.{openocd,probe,release}.log`
- `85a_r2.{openocd,probe,release}.log`
- `85b.{openocd,probe,release}.log`
- `85b_r2.{openocd,probe,release}.log`

## Case Results (from runtime output)

- `85a`: ATI zero-byte class (`143` bytes), no `VER_URC`.
- `85a_r2`: ATI zero-byte class (`144` bytes), no `VER_URC`.
- `85b`: ATI silence, no `VER_URC`.
- `85b_r2`: ATI silence, no `VER_URC`.

These signatures match prior manual/ad-hoc runs and confirm the profile runner reproduces the expected discriminator classes.

## Net

Runner infrastructure is now operational and evidence-complete for seeded profiles. Stage1 ingress gate remains blocked (`VER_REQ -> VER_URC` still failing), but profile replay is no longer a tooling blocker.
