# Collision check prototype (issue #119)

Prototype tooling for detecting part interference in the LifeTrac v25 OpenSCAD assembly
across the loader-arm animation. Nothing here modifies the model; the scripts work from
the existing `show_*` toggles in `DESIGN-STRUCTURAL/openscad/lifetrac_v25.scad` and a
sanitized copy of that file generated into the output directory.

See the discussion on [issue #119](https://github.com/OpenSourceEcology/LifeTrac/issues/119)
for the research behind this and the proposed CI design.

## What it does

1. `export_groups.sh <t> <group>` exports one rigid group (`frame`, `wheels`, `arms`,
   `bucket`, `hydraulics`, `platform`) as STL at animation time `t` (0 = arms down,
   0.5 = arms fully raised, 1 = back down).
2. `check_collisions.py <t>` loads every group exported for that `t` (static groups fall
   back to `t=0`) and runs an FCL collision query over all pairs. It prints contacts and the
   maximum FCL contact depth per pair, and the nearest approach for pairs that are clear.
   Exit code 1 when a non-whitelisted pair overlaps by more than the tolerance.
3. `export_pair.sh <t> <groupA> <groupB>` renders the OpenSCAD `intersection()` of two
   groups. With OpenSCAD 2021.01 an empty intersection exits 1 and writes no file; a
   non-empty one is the exact overlap solid.
4. `analyze_pairs.py <t>` measures the overlap solids from step 3 (volume, number of
   bodies, centroid of the largest bodies) so the overlap can be located.

## Usage

```bash
pip install -r requirements.txt          # trimesh, python-fcl, manifold3d, ...
cd LifeTrac-v25/tools/collision-check
for g in frame wheels platform; do ./export_groups.sh 0 $g; done
for t in 0 0.25 0.5; do for g in arms bucket hydraulics; do ./export_groups.sh $t $g; done; done
python3 check_collisions.py 0.5
./export_pair.sh 0.5 arms frame && python3 analyze_pairs.py 0.5
```

`OPENSCAD=/path/to/openscad-nightly` selects a different binary; `COLLISION_OUT` moves the
output directory (default `./out`, git-ignored).

## Caveats (measured with OpenSCAD 2021.01, CGAL backend)

- Export time per pose on a 4-core box: arms about 67 s, hydraulics 12 s, bucket 6 s;
  static groups once: frame 126 s, wheels 20 s, platform 20 s. A Manifold-backed nightly
  build is much faster.
- 2021.01 writes STL with six significant digits, which breaks watertightness of small
  features (weld beads, nuts), so the Manifold boolean volume inside `check_collisions.py`
  is only attempted for watertight meshes. Use `export_pair.sh` for exact volumes, or a
  nightly OpenSCAD which exports full precision.
- FCL contact depth is a per-triangle number: a good zero/non-zero filter, not the size of
  the overlap.
- Joint contacts that are intentional (pins in lugs, cylinder clevises on their mounts,
  wheel hubs in the side panels, platform brackets on the frame) currently show up as
  overlaps and need either real clearance in the model or an entry in `WHITELIST`.
