# 2026-05-10 Owner-Net Profile Manifest Runner (Copilot v1.0)

## Objective

Proceed with the profile-style experiment framework recommended in the external similar-project review, so owner-net and endpoint-assumption branches can be executed reproducibly without hardcoded case lists.

## What Was Added

### 1) Manifest file

- Path: `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/owner_net_profiles.json`
- Role:
  - Declares named run profiles
  - Declares per-profile probe endpoint assumptions (`probeDevice`, `baud`, `bootTimeout`)
  - Declares ordered cfg case lists with explicit case tags

Initial profiles seeded from already-validated branches:

1. `pa11_discriminator_recheck`
2. `pa11_inreset_toggle_recheck`
3. `ownerexp_onehot_replay`

### 2) Manifest-driven runner

- Path: `DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/run_owner_net_profile.ps1`
- Behavior:
  - Pushes helper toolkit to `/tmp/lifetrac_p0c/`
  - Optionally runs `probe_lora_dt.sh` and `probe_lora_userspace.sh`
  - For each case in profile:
    1. Apply halted cfg (OpenOCD)
    2. Wait 800 ms
    3. Run Stage1 probe with profile endpoint params
    4. Release/reset (`99_release_and_reset.cfg`)
    5. Pull per-case openocd/probe/release logs
  - Writes evidence under `bench-evidence/<profile-prefix>_<timestamp>/`

## Usage Examples

From `LifeTrac-v25/DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/`:

```powershell
./run_owner_net_profile.ps1 -Target pa11_inreset_toggle_recheck
```

```powershell
./run_owner_net_profile.ps1 -Target ownerexp_onehot_replay -SkipDiagProbes
```

```powershell
./run_owner_net_profile.ps1 -Target pa11_discriminator_recheck -ProbeDevice /dev/ttymxc3 -Baud 921600 -BootTimeout 2.0
```

## Why This Helps Current Blocker Work

1. Separates endpoint assumptions from control-net case selection.
2. Makes repeated branch replays deterministic and auditable.
3. Provides a clean path to add future owner-net candidate sets as new profiles while preserving identical execution flow.

## Current Limitation

Direct schematic net-name extraction for ABX00043 control pins remains pending (PDF net-text extraction is not yet complete in this pass). The new framework is prepared to consume those owner-net mappings as additional profile entries once annotated.

## Next Suggested Step

Add a fourth profile that encodes the first net-owner-derived interventions immediately after schematic control-net annotations are completed.
