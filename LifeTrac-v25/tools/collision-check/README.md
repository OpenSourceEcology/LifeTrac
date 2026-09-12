# Collision check for the LifeTrac v25 assembly

Automated interference check of `DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad` over the
loader's pose envelope. Background, research and measurements are in
[issue #119](https://github.com/OpenSourceEcology/LifeTrac/issues/119). The GitHub
workflow `.github/workflows/openscad-collision-check.yml` runs it on every change under
`DESIGN-STRUCTURAL/openscad/` and fails (red check) on interference; the report lands in
the job summary and as an artifact.

## How it works

1. **Envelope.** An echo-only OpenSCAD run reads the model's `COLLISION_ENVELOPE` line
   (arm min/max lift angle, bucket dump angle at full lift, curl angle at ground) and a
   grid of poses is built in the machine's joint space: arm angle × bucket angle
   *relative to the arm* (the bucket cylinder is mounted between arm and bucket, so the
   relative angle is what it controls; absolute angle = arm + relative, 0° = level).
2. **Reachability.** For each pose a second echo-only run (about 0.4 s) reports the four
   hydraulic cylinder extensions (`COLLISION_CYL` lines). Poses where a cylinder would be
   over-extended or bottomed are marked unreachable and not judged.
3. **Exports.** Every rigid group is exported as STL with the model's own `show_*`
   toggles: `frame`, `wheels`, `platform` once (pose independent) and `arms`, `bucket`,
   `hydraulics` per pose, in parallel. The pose is set with
   `-D ARM_LIFT_ANGLE=… -D BUCKET_TILT_ANGLE=…` (both are top-level assignments).
4. **Measurement.** The exact intersection volume of every pair of groups is computed with
   the Manifold boolean engine (`manifold3d`) and compared with the per-pair budget in
   `collision_rules.json`. Clearance rules use the FCL minimum distance (`python-fcl` via
   `trimesh`); the ground rule uses the mesh bounds.
5. **Report.** Markdown report (`--report`, `--summary` for `$GITHUB_STEP_SUMMARY`) with
   the envelope grid, failures, per-pair overlap range, the curl hard-stop probe, ground-
   limited poses, clearance rules and cylinder stroke usage, plus a JSON file (`--json`).
   Exit code 1 when any check fails.

## Usage

```bash
pip install -r requirements.txt
cd LifeTrac-v25/tools/collision-check

# default 7x5 grid over the envelope (what CI runs)
python3 collision_check.py --report out/collision_report.md --json out/collision_results.json

# explicit poses: arm angle : bucket angle relative to the arm, degrees
# (use '=' when the list starts with '-'; absolute bucket angle = arm + relative)
python3 collision_check.py --poses=-27.7:27.7,10:-30,49.4:-94.4

# the animation path (what the GIF shows), 36 frames
python3 collision_check.py --animation-frames 36

python3 collision_check.py --help
```

`OPENSCAD=/path/to/openscad-nightly` (or `--openscad`) selects another binary; a nightly
build with the Manifold backend exports much faster than 2021.01. `--jobs` sets the number
of parallel OpenSCAD processes, `--force` re-exports cached meshes. Meshes, logs and
reports go to `out/` (git-ignored, `--out` or `COLLISION_OUT` to move it).

Cost with OpenSCAD 2021.01 on 4 cores: about 85 s of export time per reachable pose
(arms 67 s, hydraulics 12 s, bucket 6 s), the static groups once (frame 126 s, wheels
20 s, platform 20 s), and well under a second per pose for the volume booleans.

## Rules (`collision_rules.json`)

- `default_allowed_overlap_mm3` (50 mm³): overlap tolerated for any pair, enough to absorb
  polygon slivers where a pin and its hole share a nominal diameter.
- `allowed_overlap_mm3`: per-pair budgets for the joint simplifications the model still
  has: pins and clevises drawn straight through their mating parts, the UWU hubs through
  the side panels. They were calibrated from the measured overlap across the envelope plus
  a margin and should shrink to the default as those joints get real clearance. A motion
  collision adds volume on top of the joint overlap, which is what the budget catches.
- `min_clearance_mm`: `arms/wheels` 25.4 mm, rule 6 of `DESIGN-STRUCTURAL/DESIGN_RULES.md`
  (main arm at least 1" from the front wheels through the whole range of motion).
- `ground_plane`: `groups` (the arms) may not dip below ground (rule 2, crash prevention);
  `informational_groups` (the bucket) are only reported, because with the arms at ground
  level the ground itself limits how far the bucket can dump (the lip would be up to
  40 cm below grade at the dump limit). Such poses are marked ⛰ in the envelope grid.
- `bucket_curl_inset_deg` (10°): the curl limit is a plate-on-plate hard stop by definition
  (rule 5, back plate parallel to the drop leg), so the grid stops short of it and
  `hard_stop_probe` reports the stop pose itself informationally. Measured with the arms
  down, the bucket/arm overlap is at sliver level up to +40° relative (0.6 cm³), then
  12 cm³ at +44°, 18 cm³ at +46°, 22 cm³ at +48° and 26 cm³ at the defined +50° stop,
  with 63 mm of bucket-cylinder retraction still available: the plates meet about 8°
  before the angle the model calls full curl, and the cylinder can pull the bucket past
  the contact. Reduce the inset once the curl definition or the cylinder placement is
  corrected (rule 5 asks for the cylinder to bottom out at full curl).
- `cylinder_extension_tolerance_mm`: slack on the stroke limits for the reachability test.

To recalibrate after a joint changes, run the check with `--json` and then
`python3 suggest_budgets.py out/collision_results.json`: it prints the measured maximum per
pair plus a margin, rounded up, ready to paste into `allowed_overlap_mm3`.

## Diagnostics

`export_pair.sh <t> <groupA> <groupB>` renders the OpenSCAD `intersection()` of two groups
at animation time `t` (empty result = exit 1 and no file), and `analyze_pairs.py <t>`
measures and locates the overlap solids. Useful to cross-check a reported overlap against
OpenSCAD's own geometry.

## Known limitations

- OpenSCAD 2021.01 writes STL with six significant digits; trimesh reports the meshes as
  not watertight but Manifold accepts them (`Manifold.status() == NoError`) and its volumes
  agree with OpenSCAD-rendered intersections to within a few percent. If Manifold rejects
  a mesh, the affected pairs are reported as unmeasurable and the run fails.
- FCL contact depth is not used: it is a per-triangle number that reads 25.4 mm on flush
  coplanar faces (frame/platform) that do not overlap at all.
- `platform_fold_angle` is not swept (platform is treated as static, deployed).
- Hydraulics are one group, so rod-in-barrel overlap is internal and not checked; splitting
  barrel and rod per cylinder is a follow-up.
