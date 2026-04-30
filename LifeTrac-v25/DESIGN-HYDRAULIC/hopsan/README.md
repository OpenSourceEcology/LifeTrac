# Hopsan models & notes

Multi-domain system simulation models for the LifeTrac v25 hydraulic
system, authored against [Hopsan](https://github.com/Hopsan/hopsan)
(Apache-2.0 simulation core + CLI; GPL-3.0 GUI). Hopsan is a free
open-source tool from Linköping University's Division of Fluid and
Mechatronic Systems.

This folder holds:

- **Reference models** (`*.hmf`) — hand-authored or GUI-exported Hopsan
  schematics of the canonical v25 hydraulic stack (track motors, arm
  cylinders, aux ports, pump, relief valve, accumulator).

**Parameter source:** all `.hmf` models in this folder pull their part
numbers, displacements, pressures, and flow ratings from
[../HYDRAULIC_BOM.md](../HYDRAULIC_BOM.md). When the BOM changes, the
.hmf templates regenerate (BC-16). Do not hand-edit parameters in
.hmf files without updating the BOM first.
- **Tutorials & notes** — links to upstream Hopsan documentation plus
  any LifeTrac-specific naming conventions or component-library notes
  that future BC-16 codegen will rely on.
- **Simulation runbook** — once BC-16 lands, instructions for running
  `lifetrac-config export-hopsan <toml> --out model.hmf` and feeding
  the result into `HopsanCLI` for a headless simulation.

## File index

| File | What it is | Source |
|---|---|---|
| [Hopsan-test.hmf](Hopsan-test.hmf) | Minimal smoke-test model; empty `<system name="test">` with default simulation settings (10 s @ 1 ms, 2048 samples). Used to confirm a local Hopsan install can open + simulate v25 repo files. | OSE wiki ([HopsanTest.hmf](https://wiki.opensourceecology.org/images/4/46/HopsanTest.hmf)), Hopsan GUI v2.23.1. |

## Hopsan version pin

The reference models in this folder target **Hopsan 2.23.1**
(`hopsancoreversion="2.23.1.20251001.1413"`). Future BC-16 codegen
will emit the same version stamps so a CI gate can refuse to load a
model authored against an older or newer Hopsan if the schema breaks.

## Installing Hopsan locally

| Platform | Install |
|---|---|
| Windows | `choco install hopsan` (Chocolatey) or zip from [hopsan releases](https://github.com/Hopsan/hopsan/releases) |
| Ubuntu / Debian | `apt install ./hopsan-*.deb` from [releases](https://github.com/Hopsan/hopsan/releases) |
| Snap | `snap install hopsan` (CLI is `hopsan.cli`) |
| Flatpak | `flatpak install flathub com.github.hopsan.Hopsan` (CLI is `hopsancli` inside the sandbox) |

Verify the headless CLI is on PATH:

```powershell
HopsanCLI --help    # Windows
hopsancli --help    # Linux package managers
```

## Roadmap (BC-16 / BC-16B / BC-17)

1. **BC-16** — `lifetrac-config export-hopsan <toml> --out model.hmf`
   subcommand. Hand-written `.hmf` template per
   `(track_axis_count, arm_axis_count, aux.port_count)` topology
   tuple, deterministic parameter substitution from the validated
   `BuildConfig`, structural SIL gate (no Hopsan install required for
   CI).
2. **BC-16B** — `--simulate` flag that locates `HopsanCLI` (env var
   `LIFETRAC_HOPSAN_CLI` or `--hopsan-cli=<path>`), runs it against
   the emitted model, parses the CSV result, asserts sanity invariants
   (no NaN, pressures bounded, settling-time bounded). Skipped in CI
   unless the env var is set.
3. **BC-17** — FMI / FMU export pinned per topology; drive from
   PyFMI / FMPy in-process for per-push fluid-dynamics SIL gates.

## Why both directions are *not* worth it

Hopsan → BuildConfig (parsing a `.hmf` to fill a `build.<unit>.toml`)
was considered and rejected: our schema is much narrower than what
Hopsan can express (~15 hydraulic leaves vs. hundreds of Hopsan
component parameters), and any reverse mapping would require operators
to follow strict schematic-naming conventions we cannot enforce.
Authoring stays in the BuildConfig TOML; Hopsan is a *consumer*.

## Upstream references

- Repo: https://github.com/Hopsan/hopsan
- Documentation: https://hopsan.github.io/documentation
- HCOM scripting language (used inside `.hmf` for parameter sweeps):
  see Hopsan docs.
- Tutorials kept one level up: [tutorial_gettingstarted.pdf](../tutorial_gettingstarted.pdf),
  [tutorial_advanced_usage.pdf](../tutorial_advanced_usage.pdf).
