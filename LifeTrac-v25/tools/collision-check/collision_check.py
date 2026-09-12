#!/usr/bin/env python3
"""Interference check for the LifeTrac v25 OpenSCAD assembly over its pose envelope.

Background and design: https://github.com/OpenSourceEcology/LifeTrac/issues/119

What it does
------------
1. Asks the model for its pose envelope (the ``COLLISION_ENVELOPE`` echo line) and builds a
   grid of poses in the machine's joint space: arm lift angle x bucket angle relative to
   the arm (the bucket cylinder is mounted between arm and bucket, so the relative angle is
   what the cylinder controls; absolute angle = arm + relative). Explicit poses or the
   animation path can be used instead.
2. Runs a fast echo-only evaluation per pose and keeps only the poses whose hydraulic
   cylinders stay within their stroke (``COLLISION_CYL`` echo lines). Poses the cylinders
   cannot reach are reported but not judged.
3. Exports every rigid group (frame, wheels, platform once; arms, bucket, hydraulics per
   pose) as STL using the model's ``show_*`` toggles, in parallel.
4. Computes the exact intersection volume of every pair of groups with the Manifold
   boolean engine and compares it with the per-pair budget in ``collision_rules.json``
   (the budgets encode the joint simplifications the model still has: pins and clevises
   drawn through their mating parts). Clearance rules use the FCL minimum distance and the
   ground rule uses the mesh bounds.
5. Writes a Markdown report (optionally appended to ``$GITHUB_STEP_SUMMARY``), a JSON
   results file, and exits 1 when any check fails.

Requires OpenSCAD on PATH (2021.01 works; a nightly build with the Manifold backend is
faster) and the Python packages in requirements.txt.
"""
from __future__ import annotations

import argparse
import concurrent.futures
import itertools
import json
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass, field

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_MODEL = os.path.normpath(
    os.path.join(HERE, "..", "..", "DESIGN-STRUCTURAL", "openscad", "lifetrac_v25.scad"))
DEFAULT_CONFIG = os.path.join(HERE, "collision_rules.json")

# group name -> (show_* toggle in lifetrac_v25.scad, pose independent?)
GROUPS = {
    "frame": ("show_frame", True),
    "wheels": ("show_wheels", True),
    "platform": ("show_folding_platform", True),
    "arms": ("show_loader_arms", False),
    "bucket": ("show_bucket", False),
    "hydraulics": ("show_hydraulics", False),
}
ALL_TOGGLES = [flag for flag, _ in GROUPS.values()] + ["show_cog"]
STATIC_GROUPS = [g for g, (_, static) in GROUPS.items() if static]
MOVING_GROUPS = [g for g, (_, static) in GROUPS.items() if not static]

ECHO_RE = re.compile(r'^ECHO:\s*"COLLISION_(\w+)",\s*(.*)$')
# Fallback lines for a model that predates the COLLISION_* echoes.
FALLBACK_ARM_MIN_RE = re.compile(r'^ECHO:\s*"ARM_MIN_ANGLE:",\s*(-?[\d.]+)')
FALLBACK_ARM_MAX_RE = re.compile(r'^ECHO:\s*"_arm_max_for_animation:",\s*(-?[\d.]+)')
FALLBACK_CYL_RE = re.compile(
    r'^ECHO:\s*"oriented_cylinder: len=",\s*(-?[\d.eE+-]+),\s*"extension=",\s*(-?[\d.eE+-]+),\s*"stroke=",\s*(-?[\d.eE+-]+)')

VOLUME_NOISE_MM3 = 1e-3   # below this the boolean result is numerical noise, not geometry


# --------------------------------------------------------------------------- data types
@dataclass(frozen=True)
class Pose:
    arm: float          # ARM_LIFT_ANGLE in degrees
    bucket_rel: float   # BUCKET_TILT_ANGLE in degrees, relative to the arm (positive = curl)

    @property
    def bucket_abs(self) -> float:
        """Absolute bucket angle (0 = level, negative = dumping)."""
        return self.arm + self.bucket_rel

    @property
    def key(self) -> str:
        return f"arm{self.arm:+07.2f}_rel{self.bucket_rel:+07.2f}"

    def label(self) -> str:
        return f"arm {self.arm:+.1f}°, bucket {self.bucket_rel:+.1f}° rel ({self.bucket_abs:+.1f}° abs)"


@dataclass
class Envelope:
    arm_min: float
    arm_max: float
    rel_min: float     # dump limit: cylinder fully extended (dump angle reached with the arms at max lift)
    rel_max: float     # curl limit sampled (hard stop minus inset)
    curl_stop: float   # curl limit as the model defines it (back plate parallel to the drop leg)


@dataclass
class Check:
    pose: str
    kind: str        # overlap | clearance | ground | export | mesh
    subject: str     # pair or group
    value: float | None
    limit: float | None
    ok: bool
    note: str = ""


@dataclass
class PoseResult:
    pose: Pose
    reachable: bool
    informational: bool = False                       # probe poses: reported, never judged
    cylinders: list = field(default_factory=list)     # (name, extension, stroke)
    checks: list = field(default_factory=list)        # list[Check]
    overlaps: dict = field(default_factory=dict)      # pair -> volume mm^3 (None = unmeasurable)
    clearances: dict = field(default_factory=dict)    # pair -> mm
    ground_limited: dict = field(default_factory=dict)  # group -> lowest z (informational)

    @property
    def failed(self) -> bool:
        return any(not c.ok for c in self.checks)


# --------------------------------------------------------------------------- pure helpers
def pair_key(a: str, b: str) -> str:
    return "/".join(sorted((a, b)))


def snap_volume(v: float) -> float:
    return 0.0 if abs(v) < VOLUME_NOISE_MM3 else float(abs(v))


def parse_echo(text: str) -> dict:
    """Extract the COLLISION_* lines (with fallbacks) from OpenSCAD echo output."""
    out: dict = {"CYL": []}
    fallback_cyl = []
    for raw in text.splitlines():
        line = raw.strip()
        m = ECHO_RE.match(line)
        if m:
            kind, rest = m.group(1), m.group(2)
            vals = [v.strip().strip('"') for v in rest.split(",")]
            try:
                if kind == "CYL":
                    out["CYL"].append((vals[0], float(vals[1]), float(vals[2])))
                else:
                    out[kind] = [float(v) for v in vals]
            except (ValueError, IndexError):
                out.setdefault("errors", []).append(line)
            continue
        m = FALLBACK_ARM_MIN_RE.match(line)
        if m:
            out["fallback_arm_min"] = float(m.group(1))
            continue
        m = FALLBACK_ARM_MAX_RE.match(line)
        if m:
            out["fallback_arm_max"] = float(m.group(1))
            continue
        m = FALLBACK_CYL_RE.match(line)
        if m:
            fallback_cyl.append((float(m.group(2)), float(m.group(3))))
    if not out["CYL"] and fallback_cyl:
        # lift_cylinders() is instantiated before bucket_cylinders(): two lift, two bucket.
        names = ["lift", "lift", "bucket", "bucket"]
        out["CYL"] = [(names[i] if i < 4 else f"cyl{i}", ext, stroke)
                      for i, (ext, stroke) in enumerate(fallback_cyl)]
    return out


def envelope_from_echo(parsed: dict, rel_default=(-95.0, 50.0)) -> Envelope:
    """COLLISION_ENVELOPE carries arm min/max and the absolute bucket dump/curl angles at the
    arm positions where the design defines them (dump at max lift, curl at ground level).
    The relative limits follow: rel_min = dump - arm_max, rel_max = curl - arm_min."""
    env = parsed.get("ENVELOPE")
    if env and len(env) >= 4:
        arm_min, arm_max = env[0], env[1]
        abs_dump, abs_curl = min(env[2], env[3]), max(env[2], env[3])
        rel_min, rel_max = abs_dump - arm_max, abs_curl - arm_min
        return Envelope(arm_min, arm_max, rel_min, rel_max, rel_max)
    if "fallback_arm_min" in parsed and "fallback_arm_max" in parsed:
        return Envelope(parsed["fallback_arm_min"], parsed["fallback_arm_max"],
                        rel_default[0], rel_default[1], rel_default[1])
    raise RuntimeError("could not read the pose envelope from the model's echo output "
                       "(expected a COLLISION_ENVELOPE line)")


def apply_curl_inset(env: Envelope, inset_deg: float) -> Envelope:
    """The bucket curl limit is a steel-on-steel hard stop by definition (rule 5 of
    DESIGN_RULES.md: back plate parallel to the drop leg), so the envelope is sampled a
    little inside it; the stop pose itself is probed separately."""
    if not inset_deg:
        return env
    new_max = max(env.rel_min, env.rel_max - inset_deg)
    return Envelope(env.arm_min, env.arm_max, env.rel_min, new_max, env.curl_stop)


def grid_poses(env: Envelope, arm_steps: int, bucket_steps: int) -> list[Pose]:
    arms = np.linspace(env.arm_min, env.arm_max, max(1, arm_steps))
    rels = np.linspace(env.rel_min, env.rel_max, max(1, bucket_steps))
    return [Pose(round(float(a), 4), round(float(r), 4)) for a in arms for r in rels]


def animation_poses(env: Envelope, frames: int, dump_angle=-45.0) -> list[Pose]:
    """Poses along the animation path in lifetrac_v25.scad: arms rise from min to max while
    the absolute bucket angle goes from level (0°) to the dump angle."""
    out = []
    for i in range(max(1, frames)):
        phase = i / max(1, frames - 1)
        arm = env.arm_min + phase * (env.arm_max - env.arm_min)
        abs_angle = phase * dump_angle
        out.append(Pose(round(arm, 4), round(abs_angle - arm, 4)))
    return out


def hard_stop_poses(env: Envelope, arm_angles=None) -> list[Pose]:
    """The defined curl stop itself, probed informationally."""
    arms = arm_angles if arm_angles is not None else [env.arm_min]
    return [Pose(round(float(a), 4), round(env.curl_stop, 4)) for a in arms]


def parse_pose_list(spec: str) -> list[Pose]:
    """'arm:bucket_rel,arm:bucket_rel,...' in degrees (bucket angle relative to the arm)."""
    poses = []
    for item in spec.split(","):
        item = item.strip()
        if not item:
            continue
        a, r = item.split(":")
        poses.append(Pose(float(a), float(r)))
    return poses


def cylinders_reachable(cyls: list, tol_mm: float) -> bool:
    if not cyls:
        return False
    return all(-tol_mm <= ext <= stroke + tol_mm for _, ext, stroke in cyls)


def overlap_verdict(volume: float | None, allowed: float) -> tuple[bool, str]:
    if volume is None:
        return False, "unmeasurable (mesh rejected by Manifold)"
    if volume <= allowed:
        return True, "clear" if volume <= 0.0 else "within budget"
    return False, "interference"


# --------------------------------------------------------------------------- OpenSCAD
class OpenSCAD:
    def __init__(self, binary: str, model: str, out_dir: str, timeout: float):
        self.binary, self.model, self.out_dir, self.timeout = binary, model, out_dir, timeout
        os.makedirs(out_dir, exist_ok=True)

    def version(self) -> str:
        try:
            r = subprocess.run([self.binary, "--version"], capture_output=True, text=True, timeout=60)
            return (r.stdout + r.stderr).strip().splitlines()[0]
        except Exception as e:  # noqa: BLE001
            return f"unknown ({e})"

    @staticmethod
    def pose_args(pose: Pose | None) -> list[str]:
        if pose is None:
            return []
        return ["-D", f"ARM_LIFT_ANGLE={pose.arm:.4f}", "-D", f"BUCKET_TILT_ANGLE={pose.bucket_rel:.4f}"]

    def run(self, out_path: str, extra: list[str], export_format: str | None = None) -> tuple[int, float, str]:
        cmd = [self.binary, "-o", out_path]
        if export_format:
            cmd += ["--export-format", export_format]
        cmd += extra + [self.model]
        log_path = os.path.splitext(out_path)[0] + ".log"
        t0 = time.time()
        with open(log_path, "w") as log:
            try:
                rc = subprocess.run(cmd, stdout=log, stderr=subprocess.STDOUT, timeout=self.timeout).returncode
            except subprocess.TimeoutExpired:
                rc = 124
        return rc, time.time() - t0, log_path

    def echo(self, name: str, pose: Pose | None) -> dict:
        """Echo-only evaluation (no geometry); returns the parsed COLLISION_* lines."""
        out_path = os.path.join(self.out_dir, f"{name}.echo")
        rc, seconds, log_path = self.run(out_path, self.pose_args(pose), export_format="echo")
        text = ""
        if os.path.exists(out_path):
            with open(out_path, errors="replace") as f:
                text = f.read()
        if rc != 0 and not text:
            with open(log_path, errors="replace") as f:
                text = f.read()
        parsed = parse_echo(text)
        parsed["rc"], parsed["seconds"] = rc, seconds
        return parsed

    def export_group(self, group: str, pose: Pose | None, force: bool) -> tuple[str, int, float, bool]:
        """Export one rigid group as STL. Returns (path, rc, seconds, cached)."""
        name = f"{group}_static" if pose is None else f"{group}_{pose.key}"
        out_path = os.path.join(self.out_dir, name + ".stl")
        if not force and os.path.exists(out_path) and os.path.getsize(out_path) > 0 \
                and os.path.getmtime(out_path) >= os.path.getmtime(self.model):
            return out_path, 0, 0.0, True
        wanted = GROUPS[group][0]
        toggles = []
        for flag in ALL_TOGGLES:
            toggles += ["-D", f"{flag}={'true' if flag == wanted else 'false'}"]
        rc, seconds, _ = self.run(out_path, toggles + self.pose_args(pose))
        return out_path, rc, seconds, False


# --------------------------------------------------------------------------- meshes
def load_mesh(path: str):
    import trimesh
    m = trimesh.load(path, force="mesh", process=True)
    if m.is_empty or len(m.faces) == 0:
        return None
    return m


def to_manifold(mesh):
    """Manifold accepts the CGAL exports of OpenSCAD 2021.01 even where trimesh's stricter
    watertight test fails; returns None when Manifold rejects the mesh."""
    import manifold3d
    try:
        mm = manifold3d.Manifold(manifold3d.Mesh(
            np.ascontiguousarray(mesh.vertices, dtype=np.float32),
            np.ascontiguousarray(mesh.faces, dtype=np.uint32)))
    except Exception:  # noqa: BLE001
        return None
    status = mm.status()
    if getattr(status, "name", str(status)) != "NoError":
        return None
    return mm


def intersection_volume(ma, mb) -> float:
    return snap_volume((ma ^ mb).volume())


def min_distance(mesh_a, mesh_b) -> float:
    import trimesh
    cm = trimesh.collision.CollisionManager()
    cm.add_object("a", mesh_a)
    return float(cm.min_distance_single(mesh_b))


# --------------------------------------------------------------------------- driver
class Runner:
    def __init__(self, args, config):
        self.args, self.config = args, config
        self.scad = OpenSCAD(args.openscad, args.model, args.out, args.timeout)
        self.budgets = {pair_key(*k.split("/")): float(v)
                        for k, v in config.get("allowed_overlap_mm3", {}).items()}
        self.default_budget = float(config.get("default_allowed_overlap_mm3", 50.0))
        self.clearance_rules = {pair_key(*k.split("/")): float(v)
                                for k, v in config.get("min_clearance_mm", {}).items()}
        ground = config.get("ground_plane", {})
        self.ground_groups = list(ground.get("groups", []))
        self.ground_info_groups = list(ground.get("informational_groups", []))
        self.ground_min_z = float(ground.get("min_z_mm", -1.0))
        self.curl_inset = float(config.get("bucket_curl_inset_deg", 0.0))
        self.hard_stop_probe = bool(config.get("hard_stop_probe", True))
        self.cyl_tol = float(config.get("cylinder_extension_tolerance_mm", 1.0))
        self.timings: dict[str, float] = {}
        self.notes: list[str] = []
        self.static_checks: list[Check] = []   # pose-independent pairs, judged once

    def log(self, msg: str):
        print(msg, flush=True)

    # ---- poses
    def poses(self) -> tuple[Envelope, list[Pose], list[Pose]]:
        """Returns the sampled envelope, the poses to judge and the informational probe poses."""
        parsed = self.scad.echo("envelope", None)
        env = envelope_from_echo(parsed, tuple(self.config.get("bucket_rel_default_range", (-95.0, 50.0))))
        env = apply_curl_inset(env, self.curl_inset)
        probes: list[Pose] = []
        if self.args.poses:
            poses = parse_pose_list(self.args.poses)
        elif self.args.animation_frames:
            poses = animation_poses(env, self.args.animation_frames)
        else:
            poses = grid_poses(env, self.args.arm_steps, self.args.bucket_steps)
            if self.hard_stop_probe and self.curl_inset:
                probes = hard_stop_poses(env)
        return env, poses, probes

    def classify(self, poses: list[Pose], probes: list[Pose]) -> dict[Pose, PoseResult]:
        results: dict[Pose, PoseResult] = {}
        t0 = time.time()
        probe_set = set(probes)

        def one(pose: Pose):
            parsed = self.scad.echo(f"pose_{pose.key}", pose)
            cyls = parsed.get("CYL", [])
            return pose, cyls, parsed.get("rc", 1)

        with concurrent.futures.ThreadPoolExecutor(max_workers=self.args.jobs) as ex:
            for pose, cyls, rc in ex.map(one, poses + [p for p in probes if p not in set(poses)]):
                reachable = rc == 0 and cylinders_reachable(cyls, self.cyl_tol)
                results[pose] = PoseResult(pose=pose, reachable=reachable, cylinders=cyls,
                                           informational=pose in probe_set)
                if rc != 0:
                    results[pose].checks.append(Check(pose.key, "export", "echo", None, None, False,
                                                      f"OpenSCAD exited {rc} on the echo run"))
        self.timings["reachability_s"] = time.time() - t0
        return results

    # ---- exports
    def export_all(self, results: dict[Pose, PoseResult]) -> dict[tuple, str]:
        tasks: list[tuple[str, Pose | None]] = [(g, None) for g in STATIC_GROUPS]
        for pose, res in results.items():
            if res.reachable:
                tasks += [(g, pose) for g in MOVING_GROUPS]
        paths: dict[tuple, str] = {}
        t0 = time.time()
        self.log(f"exporting {len(tasks)} group meshes with {self.args.jobs} parallel OpenSCAD jobs ...")

        def one(task):
            group, pose = task
            path, rc, seconds, cached = self.scad.export_group(group, pose, self.args.force)
            return task, path, rc, seconds, cached

        with concurrent.futures.ThreadPoolExecutor(max_workers=self.args.jobs) as ex:
            for task, path, rc, seconds, cached in ex.map(one, tasks):
                group, pose = task
                label = "static" if pose is None else pose.label()
                if rc != 0:
                    self.log(f"  FAILED {group:<10} {label}  (exit {rc}, {seconds:.0f}s)")
                    if pose is not None:
                        results[pose].checks.append(Check(pose.key, "export", group, None, None, False,
                                                          f"OpenSCAD exited {rc}"))
                    else:
                        self.notes.append(f"export of static group '{group}' failed (exit {rc})")
                    continue
                self.log(f"  {'cached' if cached else 'done  '} {group:<10} {label}  ({seconds:.0f}s)")
                paths[task] = path
        self.timings["export_s"] = time.time() - t0
        return paths

    # ---- analysis
    def analyse(self, results: dict[Pose, PoseResult], paths: dict[tuple, str]):
        t0 = time.time()
        static_meshes, static_manifolds = {}, {}
        for g in STATIC_GROUPS:
            path = paths.get((g, None))
            mesh = load_mesh(path) if path else None
            if mesh is None:
                self.notes.append(f"static group '{g}' has no geometry; skipped")
                continue
            static_meshes[g] = mesh
            static_manifolds[g] = to_manifold(mesh)
            if static_manifolds[g] is None:
                self.notes.append(f"static group '{g}': Manifold rejected the mesh, its overlaps are unmeasurable")

        # static/static pairs are pose independent: judge once, report once, reuse the values
        static_pairs: dict[str, tuple] = {}
        for a, b in itertools.combinations(sorted(static_meshes), 2):
            volume, clearance, checks = self.judge_pair(a, b, static_meshes, static_manifolds, "static")
            static_pairs[pair_key(a, b)] = (volume, clearance)
            self.static_checks.extend(checks)

        for pose, res in results.items():
            if not res.reachable:
                continue
            meshes, manifolds = dict(static_meshes), dict(static_manifolds)
            for g in MOVING_GROUPS:
                path = paths.get((g, pose))
                mesh = load_mesh(path) if path else None
                if mesh is None:
                    res.checks.append(Check(pose.key, "mesh", g, None, None, False, "no geometry exported"))
                    continue
                meshes[g] = mesh
                manifolds[g] = to_manifold(mesh)
            for a, b in itertools.combinations(sorted(meshes), 2):
                key = pair_key(a, b)
                if key in static_pairs:
                    volume, clearance = static_pairs[key]
                else:
                    volume, clearance, checks = self.judge_pair(a, b, meshes, manifolds, pose.key)
                    res.checks.extend(checks)
                res.overlaps[key] = volume
                if clearance is not None:
                    res.clearances[key] = clearance
            for g in self.ground_groups:
                if g in meshes:
                    min_z = float(meshes[g].bounds[0][2])
                    res.checks.append(Check(pose.key, "ground", g, min_z, self.ground_min_z,
                                            min_z >= self.ground_min_z,
                                            "lowest point of the group above ground" if min_z >= self.ground_min_z
                                            else "group dips below ground level"))
            # Informational: the ground, not the machine, limits these poses (e.g. the bucket
            # lip when dumping with the arms at ground level). Reported, not failed.
            for g in self.ground_info_groups:
                if g in meshes:
                    min_z = float(meshes[g].bounds[0][2])
                    if min_z < self.ground_min_z:
                        res.ground_limited[g] = min_z
        self.timings["analysis_s"] = time.time() - t0

    def judge_pair(self, a: str, b: str, meshes: dict, manifolds: dict, pose_key: str):
        key = pair_key(a, b)
        checks: list[Check] = []
        ma, mb = manifolds.get(a), manifolds.get(b)
        volume = intersection_volume(ma, mb) if (ma is not None and mb is not None) else None
        allowed = self.budgets.get(key, self.default_budget)
        ok, note = overlap_verdict(volume, allowed)
        checks.append(Check(pose_key, "overlap", key, volume, allowed, ok, note))
        clearance = None
        if key in self.clearance_rules:
            limit = self.clearance_rules[key]
            if volume is not None and volume > 0.0:
                checks.append(Check(pose_key, "clearance", key, 0.0, limit, False, "groups overlap"))
                clearance = 0.0
            else:
                clearance = min_distance(meshes[a], meshes[b])
                checks.append(Check(pose_key, "clearance", key, clearance, limit, clearance >= limit,
                                    "nearest approach"))
        return volume, clearance, checks


# --------------------------------------------------------------------------- report
def fmt_mm3(v: float | None) -> str:
    if v is None:
        return "n/a"
    if v >= 1000.0:
        return f"{v / 1000.0:,.1f} cm³"
    return f"{v:,.0f} mm³"


def build_report(runner: Runner, env: Envelope, results: dict[Pose, PoseResult], version: str,
                 elapsed: float) -> tuple[str, bool]:
    judged = [p for p in results if not results[p].informational]
    probes = [p for p in results if results[p].informational]
    reachable = [p for p in judged if results[p].reachable]
    failed = [p for p in reachable if results[p].failed]
    static_failed_checks = [c for c in runner.static_checks if not c.ok]
    static_failed = [n for n in runner.notes if "failed" in n]
    passed = not failed and not static_failed_checks and not static_failed and bool(reachable)
    lines = []
    lines.append("# LifeTrac v25 collision check")
    lines.append("")
    lines.append(f"**Result: {'PASS' if passed else 'FAIL'}**  ")
    lines.append(f"OpenSCAD: `{version}` · poses: {len(judged)} in grid, {len(reachable)} reachable, "
                 f"{len(failed)} failing, {len(static_failed_checks)} static failures · runtime {elapsed / 60:.1f} min "
                 f"(reachability {runner.timings.get('reachability_s', 0):.0f}s, "
                 f"exports {runner.timings.get('export_s', 0):.0f}s, "
                 f"analysis {runner.timings.get('analysis_s', 0):.0f}s)")
    lines.append("")
    inset_note = (f" The curl limit ({env.curl_stop:+.1f}° rel) is a plate-on-plate hard stop by definition "
                  f"(rule 5), so the grid stops {runner.curl_inset:g}° short of it; the stop itself is probed "
                  "below." if runner.curl_inset else "")
    lines.append(f"Envelope sampled: arm {env.arm_min:+.1f}° to {env.arm_max:+.1f}°, bucket {env.rel_min:+.1f}° "
                 f"to {env.rel_max:+.1f}° relative to the arm (absolute angle = arm + relative; "
                 f"{env.rel_min:+.1f}° rel is the dump angle at full lift, the cylinder's extension limit)."
                 f"{inset_note} Poses whose cylinders would leave their stroke are marked unreachable and not judged.")
    lines.append("")

    # envelope grid
    arms = sorted({p.arm for p in judged}, reverse=True)
    rels = sorted({p.bucket_rel for p in judged})
    if len(arms) > 1 or len(rels) > 1:
        lines.append("## Envelope")
        lines.append("")
        lines.append("Rows: arm lift angle. Columns: bucket angle relative to the arm (positive = curl, "
                     "negative = dump). ✅ pass · ❌ fail · ⬜ unreachable · ➖ not sampled · ⛰ bucket below ground "
                     "(the ground limits this pose, informational)")
        lines.append("")
        lines.append("| arm \\ bucket rel | " + " | ".join(f"{r:+.1f}°" for r in rels) + " |")
        lines.append("|---|" + "---|" * len(rels))
        lookup = {(p.arm, p.bucket_rel): results[p] for p in judged}
        for a in arms:
            cells = []
            for r in rels:
                res = lookup.get((a, r))
                cell = "➖" if res is None else "⬜" if not res.reachable else "❌" if res.failed else "✅"
                if res is not None and res.ground_limited:
                    cell += " ⛰"
                cells.append(cell)
            lines.append(f"| {a:+.1f}° | " + " | ".join(cells) + " |")
        lines.append("")

    # failures
    lines.append("## Failures")
    lines.append("")
    fail_rows = [("static (pose independent)", c) for c in static_failed_checks]
    fail_rows += [(results[p].pose.label(), c) for p in reachable for c in results[p].checks if not c.ok]
    if not reachable:
        lines.append("No reachable pose was evaluated, so nothing was checked (this counts as a failure).")
    if not fail_rows and not static_failed:
        lines.append("None.")
    else:
        lines.append("| Pose | Check | Subject | Value | Limit | Note |")
        lines.append("|---|---|---|---|---|---|")
        for label, c in fail_rows:
            value = fmt_mm3(c.value) if c.kind == "overlap" else ("n/a" if c.value is None else f"{c.value:.1f} mm")
            limit = fmt_mm3(c.limit) if c.kind == "overlap" else ("n/a" if c.limit is None else f"{c.limit:.1f} mm")
            lines.append(f"| {label} | {c.kind} | {c.subject} | {value} | {limit} | {c.note} |")
        for n in static_failed:
            lines.append(f"| static | export | | | | {n} |")
    lines.append("")

    # overlap summary across poses
    lines.append("## Overlap per pair across reachable poses")
    lines.append("")
    lines.append("Budgets come from `collision_rules.json` and encode the joint simplifications still in the "
                 "model (pins and clevises drawn through their mating parts). They should shrink to the "
                 "default as the joints get real clearance.")
    lines.append("")
    lines.append("| Pair | Min | Max | Budget | Status |")
    lines.append("|---|---|---|---|---|")
    pairs = sorted({k for p in reachable for k in results[p].overlaps})
    for key in pairs:
        vols = [results[p].overlaps[key] for p in reachable if key in results[p].overlaps]
        measurable = [v for v in vols if v is not None]
        budget = runner.budgets.get(key, runner.default_budget)
        if not measurable:
            status = "unmeasurable"
        elif max(measurable) > budget:
            status = "❌ over budget"
        elif max(measurable) > 0:
            status = "within budget"
        else:
            status = "clear"
        lines.append(f"| {key} | {fmt_mm3(min(measurable)) if measurable else 'n/a'} | "
                     f"{fmt_mm3(max(measurable)) if measurable else 'n/a'} | {fmt_mm3(budget)} | {status} |")
    lines.append("")

    # hard stop probe (informational)
    probe_rows = [p for p in probes if results[p].reachable]
    if probe_rows:
        lines.append("## Curl hard stop (informational)")
        lines.append("")
        lines.append("Overlap at the curl limit as the model defines it (back plate parallel to the drop leg). "
                     "The stop is a contact by definition, so this is reported, not judged; a volume well above "
                     "the sliver level means the defined stop angle lies past the point where the plates meet.")
        lines.append("")
        lines.append("| Pose | arms/bucket overlap | Cylinders (extension / stroke) |")
        lines.append("|---|---|---|")
        for p in probe_rows:
            r = results[p]
            cyl = ", ".join(f"{n} {e:.0f}/{s:.0f}" for n, e, s in r.cylinders) or "n/a"
            lines.append(f"| {p.label()} | {fmt_mm3(r.overlaps.get('arms/bucket'))} | {cyl} |")
        lines.append("")

    # ground-limited poses (informational)
    limited = [(p, results[p].ground_limited) for p in reachable if results[p].ground_limited]
    if limited:
        lines.append("## Ground-limited poses (informational)")
        lines.append("")
        lines.append("At these poses a group would be below ground level, so the ground, not the machine, "
                     "limits the motion (typically the bucket lip when dumping with the arms down). "
                     "They are still checked for interference but the depth is not a failure.")
        lines.append("")
        lines.append("| Pose | Group | Lowest point |")
        lines.append("|---|---|---|")
        for p, groups in limited:
            for g, z in sorted(groups.items()):
                lines.append(f"| {p.label()} | {g} | {z:.0f} mm |")
        lines.append("")

    # clearance and cylinder usage
    if runner.clearance_rules:
        lines.append("## Clearance rules")
        lines.append("")
        lines.append("| Pair | Minimum over poses | Required | Status |")
        lines.append("|---|---|---|---|")
        for key, limit in sorted(runner.clearance_rules.items()):
            vals = [results[p].clearances[key] for p in reachable if key in results[p].clearances]
            if vals:
                worst = min(vals)
                lines.append(f"| {key} | {worst:.1f} mm | {limit:.1f} mm | {'✅' if worst >= limit else '❌'} |")
            else:
                lines.append(f"| {key} | n/a | {limit:.1f} mm | not evaluated |")
        lines.append("")
    cyl_names = sorted({c[0] for p in reachable for c in results[p].cylinders})
    if cyl_names:
        lines.append("## Cylinder stroke usage over reachable poses")
        lines.append("")
        lines.append("| Cylinder | Stroke | Min extension | Max extension | Used |")
        lines.append("|---|---|---|---|---|")
        for name in cyl_names:
            ext = [c[1] for p in reachable for c in results[p].cylinders if c[0] == name]
            stroke = max(c[2] for p in reachable for c in results[p].cylinders if c[0] == name)
            used = (max(ext) - min(ext)) / stroke * 100.0 if stroke else 0.0
            lines.append(f"| {name} | {stroke:.0f} mm | {min(ext):.0f} mm | {max(ext):.0f} mm | {used:.0f}% |")
        lines.append("")

    # per pose details
    lines.append("<details><summary>Per-pose details</summary>")
    lines.append("")
    lines.append("| Pose | Reachable | Cylinders (extension / stroke) | Overlaps above zero | Clearances |")
    lines.append("|---|---|---|---|---|")
    for p in judged + probes:
        r = results[p]
        cyl = ", ".join(f"{n} {e:.0f}/{s:.0f}" for n, e, s in r.cylinders) or "n/a"
        ov = ", ".join(f"{k} {fmt_mm3(v)}" for k, v in sorted(r.overlaps.items()) if v is None or v > 0) or "none"
        cl = ", ".join(f"{k} {v:.0f} mm" for k, v in sorted(r.clearances.items())) or ""
        tag = " (probe)" if r.informational else ""
        lines.append(f"| {p.label()}{tag} | {'yes' if r.reachable else 'no'} | {cyl} | {ov} | {cl} |")
    lines.append("")
    lines.append("</details>")
    lines.append("")
    if runner.notes:
        lines.append("## Notes")
        lines.append("")
        lines += [f"- {n}" for n in runner.notes]
        lines.append("")
    return "\n".join(lines), passed


def results_json(runner: Runner, env: Envelope, results: dict[Pose, PoseResult], version: str,
                 passed: bool) -> dict:
    return {
        "openscad": version,
        "passed": passed,
        "envelope": env.__dict__,
        "static_checks": [c.__dict__ for c in runner.static_checks],
        "notes": runner.notes,
        "poses": [
            {
                "arm": p.arm, "bucket_rel": p.bucket_rel, "bucket_abs": p.bucket_abs,
                "reachable": r.reachable, "informational": r.informational,
                "cylinders": [{"name": n, "extension_mm": e, "stroke_mm": s} for n, e, s in r.cylinders],
                "overlaps_mm3": r.overlaps, "clearances_mm": r.clearances,
                "ground_limited_mm": r.ground_limited,
                "checks": [c.__dict__ for c in r.checks],
            }
            for p, r in results.items()
        ],
    }


# --------------------------------------------------------------------------- main
def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model", default=DEFAULT_MODEL, help="assembly file (default: lifetrac_v25.scad)")
    ap.add_argument("--openscad", default=os.environ.get("OPENSCAD", "openscad"), help="OpenSCAD binary")
    ap.add_argument("--config", default=DEFAULT_CONFIG, help="rules file (default: collision_rules.json)")
    ap.add_argument("--out", default=os.environ.get("COLLISION_OUT", os.path.join(HERE, "out")),
                    help="working directory for meshes, logs and reports")
    ap.add_argument("--jobs", type=int, default=max(1, os.cpu_count() or 1), help="parallel OpenSCAD processes")
    ap.add_argument("--arm-steps", type=int, default=7, help="arm angles across the envelope (grid mode)")
    ap.add_argument("--bucket-steps", type=int, default=5, help="bucket angles across the envelope (grid mode)")
    ap.add_argument("--poses", help="explicit poses 'arm:bucket_rel,...' in degrees (bucket relative to the "
                                    "arm) instead of the grid; use --poses=... when the list starts with '-'")
    ap.add_argument("--animation-frames", type=int, help="sample the animation path with N frames instead of the grid")
    ap.add_argument("--force", action="store_true", help="re-export meshes even when a fresh STL exists")
    ap.add_argument("--timeout", type=float, default=1800.0, help="seconds per OpenSCAD run")
    ap.add_argument("--report", help="write the Markdown report here")
    ap.add_argument("--json", help="write the JSON results here")
    ap.add_argument("--summary", help="append the Markdown report to this file (e.g. $GITHUB_STEP_SUMMARY)")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    with open(args.config) as f:
        config = json.load(f)
    runner = Runner(args, config)
    t_start = time.time()
    version = runner.scad.version()
    runner.log(f"OpenSCAD: {version}")
    runner.log(f"model: {args.model}")

    env, poses, probes = runner.poses()
    runner.log(f"envelope: arm {env.arm_min:+.1f}..{env.arm_max:+.1f}°, bucket {env.rel_min:+.1f}.."
               f"{env.rel_max:+.1f}° relative (curl stop {env.curl_stop:+.1f}°); {len(poses)} poses, "
               f"{len(probes)} probe(s)")
    results = runner.classify(poses, probes)
    reachable = sum(1 for r in results.values() if r.reachable and not r.informational)
    runner.log(f"reachable poses: {reachable}/{len(poses)} ({runner.timings['reachability_s']:.0f}s)")

    paths = runner.export_all(results)
    runner.analyse(results, paths)

    report, passed = build_report(runner, env, results, version, time.time() - t_start)
    if args.report:
        os.makedirs(os.path.dirname(os.path.abspath(args.report)), exist_ok=True)
        with open(args.report, "w") as f:
            f.write(report)
    if args.json:
        os.makedirs(os.path.dirname(os.path.abspath(args.json)), exist_ok=True)
        with open(args.json, "w") as f:
            json.dump(results_json(runner, env, results, version, passed), f, indent=1)
    if args.summary:
        with open(args.summary, "a") as f:
            f.write(report + "\n")
    print(report)
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
