"""Unit tests for the pure helpers in collision_check.py (no OpenSCAD needed)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import collision_check as cc  # noqa: E402

SAMPLE_ECHO = """\
ECHO: "DEBUG: Loading lifetrac_v25_params.scad - Version with Fixes"
ECHO: "COLLISION_ENVELOPE", -27.7092, 49.4496, -45, 22.2908
ECHO: "COLLISION_POSE", 30, -20, 200, 1100, 1583.2, 10.5, -12
ECHO: "COLLISION_CYL", "lift", 26.9497, 650
ECHO: "COLLISION_CYL", "bucket", 136.252, 508
"""

OLD_MODEL_ECHO = """\
ECHO: "ARM_MIN_ANGLE:", -27.7092
ECHO: "_arm_max_for_animation:", 49.4496
ECHO: "oriented_cylinder: len=", 830.45, "extension=", 26.9497, "stroke=", 650
ECHO: "oriented_cylinder: len=", 830.45, "extension=", 26.9497, "stroke=", 650
ECHO: "oriented_cylinder: len=", 848.562, "extension=", 136.252, "stroke=", 508
ECHO: "oriented_cylinder: len=", 848.562, "extension=", -3, "stroke=", 508
"""


def test_parse_echo_reads_collision_lines():
    parsed = cc.parse_echo(SAMPLE_ECHO)
    assert parsed["ENVELOPE"] == [-27.7092, 49.4496, -45.0, 22.2908]
    assert parsed["POSE"][0] == 30.0 and parsed["POSE"][1] == -20.0
    assert parsed["CYL"] == [("lift", 26.9497, 650.0), ("bucket", 136.252, 508.0)]


def test_envelope_is_converted_to_relative_bucket_angles():
    env = cc.envelope_from_echo(cc.parse_echo(SAMPLE_ECHO))
    assert env.arm_min == -27.7092 and env.arm_max == 49.4496
    assert abs(env.rel_min - (-45.0 - 49.4496)) < 1e-9      # dump at full lift: cylinder fully extended
    assert abs(env.rel_max - (22.2908 + 27.7092)) < 1e-9    # curl at ground: the plate-on-plate stop
    assert env.curl_stop == env.rel_max
    # order of the two absolute angles in the echo does not matter
    swapped = cc.envelope_from_echo({"ENVELOPE": [-27.7092, 49.4496, 22.2908, -45.0]})
    assert swapped == env


def test_parse_echo_falls_back_to_old_lines():
    parsed = cc.parse_echo(OLD_MODEL_ECHO)
    env = cc.envelope_from_echo(parsed, rel_default=(-95.0, 50.0))
    assert env.arm_min == -27.7092 and env.arm_max == 49.4496
    assert env.rel_min == -95.0 and env.rel_max == 50.0 == env.curl_stop
    assert [c[0] for c in parsed["CYL"]] == ["lift", "lift", "bucket", "bucket"]
    assert not cc.cylinders_reachable(parsed["CYL"], tol_mm=1.0)  # one cylinder at -3 mm


def test_reachability_tolerance():
    cyls = [("lift", -0.5, 650.0), ("bucket", 508.5, 508.0)]
    assert cc.cylinders_reachable(cyls, tol_mm=1.0)
    assert not cc.cylinders_reachable(cyls, tol_mm=0.1)
    assert not cc.cylinders_reachable([], tol_mm=1.0)


def test_curl_inset_only_moves_the_sampled_curl_limit():
    env = cc.Envelope(-27.7, 49.4, -94.4, 50.0, 50.0)
    inset = cc.apply_curl_inset(env, 3.0)
    assert (inset.arm_min, inset.arm_max, inset.rel_min, inset.curl_stop) == (-27.7, 49.4, -94.4, 50.0)
    assert abs(inset.rel_max - 47.0) < 1e-9
    assert cc.apply_curl_inset(env, 0.0) == env
    assert cc.apply_curl_inset(cc.Envelope(0, 1, -5.0, -4.5, -4.5), 2.0).rel_max == -5.0  # never below the dump limit


def test_grid_animation_and_hard_stop_poses():
    env = cc.Envelope(-27.0, 49.0, -94.0, 47.0, 50.0)
    grid = cc.grid_poses(env, 3, 2)
    assert len(grid) == 6
    assert grid[0] == cc.Pose(-27.0, -94.0) and grid[-1] == cc.Pose(49.0, 47.0)
    anim = cc.animation_poses(env, 3)
    assert anim[0] == cc.Pose(-27.0, 27.0)            # level bucket (0° abs) with the arms down
    assert anim[-1] == cc.Pose(49.0, -94.0)           # -45° abs at full lift
    assert anim[1].arm == 11.0 and abs(anim[1].bucket_abs - (-22.5)) < 1e-9
    assert cc.hard_stop_poses(env) == [cc.Pose(-27.0, 50.0)]


def test_pose_absolute_angle_and_key():
    pose = cc.Pose(30.0, -50.0)
    assert pose.bucket_abs == -20.0
    assert pose.key == "arm+030.0000_rel-050.0000"
    assert "-20.0° abs" in pose.label()
    assert cc.parse_pose_list("30:-50, -27.7:27.7") == [cc.Pose(30.0, -50.0), cc.Pose(-27.7, 27.7)]
    # the key carries the same precision as the -D values, so close poses never share files
    assert cc.Pose(10.001, 0.0).key != cc.Pose(10.002, 0.0).key
    assert cc.OpenSCAD.pose_args(cc.Pose(10.001, 0.0)) != cc.OpenSCAD.pose_args(cc.Pose(10.002, 0.0))


def test_model_dependencies_follow_includes_and_uses(tmp_path):
    (tmp_path / "modules").mkdir()
    (tmp_path / "parts").mkdir()
    main = tmp_path / "main.scad"
    main.write_text('include <params.scad>\nuse <modules/arm.scad>\nuse <MCAD/polyholes.scad>\ncube(1);\n')
    (tmp_path / "params.scad").write_text("X = 1;\n")
    (tmp_path / "modules" / "arm.scad").write_text("include <../params.scad>\nuse <../parts/plate.scad>\n")
    (tmp_path / "parts" / "plate.scad").write_text("module plate() {}\n")
    deps = cc.model_dependencies(str(main))
    names = sorted(os.path.relpath(d, tmp_path) for d in deps)
    assert names == ["main.scad", "modules/arm.scad", "params.scad", "parts/plate.scad"]
    os.utime(tmp_path / "parts" / "plate.scad", (2_000_000_000, 2_000_000_000))
    assert cc.newest_mtime(deps) == 2_000_000_000


def test_verdict_counts_failures_on_unreachable_poses_too():
    ok = cc.Pose(0.0, 0.0)
    broken = cc.Pose(10.0, 0.0)
    unreachable = cc.Pose(20.0, 0.0)
    results = {
        ok: cc.PoseResult(pose=ok, reachable=True,
                          checks=[cc.Check(ok.key, "overlap", "arms/frame", 10.0, 50.0, True)]),
        broken: cc.PoseResult(pose=broken, reachable=False,
                              checks=[cc.Check(broken.key, "echo", "openscad", None, None, False, "exited 1")]),
        unreachable: cc.PoseResult(pose=unreachable, reachable=False),
    }
    judged, reachable, failed, passed = cc.verdict(results, [])
    assert judged == [ok, broken, unreachable] and reachable == [ok]
    assert failed == [broken] and passed is False
    del results[broken]
    assert cc.verdict(results, [])[3] is True
    # a failed static check (missing or empty static group, failed export) fails the run
    static = [cc.Check("static", "mesh", "frame", None, None, False, "static group exported no geometry")]
    assert cc.verdict(results, static)[3] is False
    # nothing reachable at all is a failure, never a pass by vacuity
    assert cc.verdict({unreachable: results[unreachable]}, [])[3] is False


def test_probe_interference_is_ignored_but_probe_errors_fail_the_run():
    ok = cc.Pose(0.0, 0.0)
    stop = cc.Pose(-27.7, 50.0)
    results = {
        ok: cc.PoseResult(pose=ok, reachable=True,
                          checks=[cc.Check(ok.key, "overlap", "arms/frame", 10.0, 50.0, True)]),
        stop: cc.PoseResult(pose=stop, reachable=True, informational=True,
                            checks=[cc.Check(stop.key, "overlap", "arms/bucket", 26500.0, 860.0, False, "interference")]),
    }
    judged, reachable, failed, passed = cc.verdict(results, [])
    assert judged == [ok] and reachable == [ok] and failed == [] and passed is True
    # an export failure, an empty mesh or an unmeasurable overlap on the probe does fail the run
    for check in (cc.Check(stop.key, "export", "arms", None, None, False, "OpenSCAD exited 124"),
                  cc.Check(stop.key, "mesh", "bucket", None, None, False, "no geometry exported"),
                  cc.Check(stop.key, "overlap", "arms/frame", None, 70.0, False, "unmeasurable")):
        results[stop].checks = [check]
        assert results[stop].failures == [check]
        assert cc.verdict(results, [])[2] == [stop]


def test_cylinder_problems_flag_missing_or_malformed_data():
    good = cc.parse_echo(SAMPLE_ECHO)
    assert cc.cylinder_problems(good, ["lift", "bucket"]) == []
    only_lift = cc.parse_echo('ECHO: "COLLISION_CYL", "lift", 26.9, 650\n')
    assert cc.cylinder_problems(only_lift, ["lift", "bucket"]) == ["no COLLISION_CYL data for: bucket"]
    malformed = cc.parse_echo('ECHO: "COLLISION_CYL", "lift", undef, 650\n')
    problems = cc.cylinder_problems(malformed, ["lift", "bucket"])
    assert any(p.startswith("malformed echo line") for p in problems)
    assert "no COLLISION_CYL data for: bucket, lift" in problems
    assert cc.cylinder_problems(cc.parse_echo(""), ["lift", "bucket"]) == ["no COLLISION_CYL data for: bucket, lift"]


def test_cache_stamp_invalidates_on_other_binary_or_model(tmp_path):
    stamp_path = str(tmp_path / "cache_stamp.json")
    stamp = {"openscad": "OpenSCAD version 2021.01", "binary": "/usr/bin/openscad", "model": "/m/a.scad"}
    assert cc.cache_stamp_matches(stamp_path, stamp) is False      # first run: nothing to reuse
    assert cc.cache_stamp_matches(stamp_path, stamp) is True       # same build, same model
    other = dict(stamp, binary="/opt/openscad-nightly", openscad="OpenSCAD version 2026.09.01")
    assert cc.cache_stamp_matches(stamp_path, other) is False      # another build invalidates
    assert cc.cache_stamp_matches(stamp_path, dict(other, model="/m/b.scad")) is False  # another model too


def test_stamp_mismatch_purges_cached_group_meshes(tmp_path):
    out = tmp_path / "out"
    out.mkdir()
    cached = ["frame_static.stl", "frame_static.log", "arms_arm+030.0000_rel-050.0000.stl",
              "hydraulics_arm-027.7092_rel+027.7092.log"]
    kept = ["pair_arms_frame_t0.stl", "pose_arm+030.0000_rel-050.0000.echo", "collision_report.md"]
    for name in cached + kept:
        (out / name).write_text("x")
    stamp_path = str(out / "cache_stamp.json")
    stamp = {"openscad": "OpenSCAD version 2021.01", "binary": "/usr/bin/openscad", "model": "/m/a.scad"}
    # first stamp: mismatch (no stamp yet) purges the group meshes before the stamp is written
    assert cc.cache_stamp_matches(stamp_path, stamp, purge_dir=str(out)) is False
    assert sorted(p.name for p in out.iterdir()) == sorted(kept + ["cache_stamp.json"])
    # a matching stamp leaves everything alone
    (out / "frame_static.stl").write_text("x")
    assert cc.cache_stamp_matches(stamp_path, stamp, purge_dir=str(out)) is True
    assert (out / "frame_static.stl").exists()
    # another producer purges again, even when the mesh is newer than the model
    assert cc.cache_stamp_matches(stamp_path, dict(stamp, binary="/opt/openscad-nightly"), purge_dir=str(out)) is False
    assert not (out / "frame_static.stl").exists()


def test_overlap_verdict():
    assert cc.overlap_verdict(0.0, 50.0) == (True, "clear")
    assert cc.overlap_verdict(20.0, 50.0) == (True, "within budget")
    assert cc.overlap_verdict(51.0, 50.0)[0] is False
    assert cc.overlap_verdict(None, 50.0)[0] is False


def test_pair_key_is_order_independent():
    assert cc.pair_key("frame", "arms") == "arms/frame" == cc.pair_key("arms", "frame")


def test_suggest_budgets_from_results():
    import suggest_budgets as sb
    results = {"poses": [
        {"reachable": True, "overlaps_mm3": {"arms/frame": 58.4, "arms/wheels": 0.0, "frame/hydraulics": 200267.9}},
        {"reachable": True, "overlaps_mm3": {"arms/frame": 62.0, "arms/wheels": 0.0, "frame/hydraulics": 206600.0}},
        {"reachable": False, "overlaps_mm3": {"arms/frame": 9999.0}},   # unreachable poses are ignored
        {"reachable": True, "informational": True, "overlaps_mm3": {"arms/bucket": 26500.0}},  # probes too
    ]}
    assert sb.round_up_2sig(649.0) == 650.0 and sb.round_up_2sig(309882.0) == 310000.0
    budgets = sb.suggest(results, margin=0.15, floor=50.0)
    assert budgets == {"arms/frame": 72.0, "frame/hydraulics": 240000.0}


def test_snap_volume_drops_boolean_noise():
    assert cc.snap_volume(1e-7) == 0.0
    assert cc.snap_volume(-1e-7) == 0.0
    assert cc.snap_volume(-58.4) == 58.4
    assert cc.overlap_verdict(cc.snap_volume(1e-7), 50.0) == (True, "clear")
