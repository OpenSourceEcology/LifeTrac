# 2026-05-10 Owner-Net Profile Seeded Replay Runs (Copilot v1.2)

## Scope

Run the two remaining seeded manifest profiles using the validated runner and capture classification deltas.

Profiles executed:

- `pa11_discriminator_recheck`
- `ownerexp_onehot_replay`

Run commands:

```powershell
./run_owner_net_profile.ps1 -Target pa11_discriminator_recheck -SkipDiagProbes
./run_owner_net_profile.ps1 -Target ownerexp_onehot_replay -SkipDiagProbes
```

## Evidence

- `DESIGN-CONTROLLER/bench-evidence/T6_profile_pa11_discriminator_recheck_2026-05-10_220742/`
- `DESIGN-CONTROLLER/bench-evidence/T6_profile_ownerexp_onehot_2026-05-10_220814/`

Artifact completeness checks:

- `pa11_discriminator_recheck`: 12 logs present (`4 cases x 3 logs`)
- `ownerexp_onehot_replay`: 18 logs present (`6 cases x 3 logs`)

## Results

### Profile: `pa11_discriminator_recheck`

Cases:

- `82a`: ATI silence (`no bytes observed`), no `VER_URC`
- `82b`: ATI silence (`no bytes observed`), no `VER_URC`
- `83a`: ATI silence (`no bytes observed`), no `VER_URC`
- `83b`: ATI silence (`no bytes observed`), no `VER_URC`

Net classification:

- Profile is fully in the silence class under this replay (`4/4` silence).

### Profile: `ownerexp_onehot_replay`

Cases:

- `72a`: ATI zero-byte class (`144` bytes), no `VER_URC`
- `73a`: ATI zero-byte class (`144` bytes), no `VER_URC`
- `74a`: ATI zero-byte class (`144` bytes), no `VER_URC`
- `75a`: ATI zero-byte class (`144` bytes), no `VER_URC`
- `76a`: ATI zero-byte class (`144` bytes), no `VER_URC`
- `77a`: ATI zero-byte class (`145` bytes), no `VER_URC`

Net classification:

- Profile is fully in the zero-byte ATI class (`6/6` non-empty zero payload).

## Interpretation

1. The manifest runner remains stable and reproducible across both additional seeded profiles.
2. Seeded branches are now separated into two deterministic classes:
   - silence class (`pa11_discriminator_recheck`)
   - zero-byte ATI class (`ownerexp_onehot_replay`)
3. Neither class recovers ingress; all cases still fail `VER_REQ -> VER_URC`.

## Next Step

Proceed to first schematic-owner-derived profile (ABX00043 owner-net extraction output encoded into manifest), then rerun with immediate repeats per case and the same evidence structure.
