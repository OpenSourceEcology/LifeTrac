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
ECHO: "COLLISION_CYL", "lift", 26.9497, 650
ECHO: "COLLISION_CYL", "bucket", 136.252, 508
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
    assert parsed["CYL"] == [("lift", 26.9497, 650.0)] * 2 + [("bucket", 136.252, 508.0)] * 2


def test_parse_echo_falls_back_to_old_lines():
    parsed = cc.parse_echo(OLD_MODEL_ECHO)
    env = cc.envelope_from_echo(parsed, bucket_default=(-45.0, 30.0))
    assert env.arm_min == -27.7092 and env.arm_max == 49.4496
    assert env.bucket_abs_min == -45.0 and env.bucket_abs_max == 30.0
    assert [c[0] for c in parsed["CYL"]] == ["lift", "lift", "bucket", "bucket"]
    assert not cc.cylinders_reachable(parsed["CYL"], tol_mm=1.0)  # one cylinder at -3 mm


def test_envelope_orders_bucket_limits():
    env = cc.envelope_from_echo({"ENVELOPE": [-27.7, 49.4, 22.3, -45.0]})
    assert env.bucket_abs_min == -45.0 and env.bucket_abs_max == 22.3


def test_reachability_tolerance():
    cyls = [("lift", -0.5, 650.0), ("bucket", 508.5, 508.0)]
    assert cc.cylinders_reachable(cyls, tol_mm=1.0)
    assert not cc.cylinders_reachable(cyls, tol_mm=0.1)
    assert not cc.cylinders_reachable([], tol_mm=1.0)


def test_curl_inset_only_moves_the_curl_limit():
    env = cc.Envelope(-27.7, 49.4, -45.0, 22.3)
    inset = cc.apply_curl_inset(env, 1.0)
    assert (inset.arm_min, inset.arm_max, inset.bucket_abs_min) == (-27.7, 49.4, -45.0)
    assert abs(inset.bucket_abs_max - 21.3) < 1e-9
    assert cc.apply_curl_inset(env, 0.0) == env
    assert cc.apply_curl_inset(cc.Envelope(0, 1, -5.0, -4.5), 2.0).bucket_abs_max == -5.0  # never below the dump limit


def test_grid_and_animation_poses():
    env = cc.Envelope(-27.0, 49.0, -45.0, 22.0)
    grid = cc.grid_poses(env, 3, 2)
    assert len(grid) == 6
    assert grid[0] == cc.Pose(-27.0, -45.0) and grid[-1] == cc.Pose(49.0, 22.0)
    anim = cc.animation_poses(env, 3)
    assert anim[0] == cc.Pose(-27.0, 0.0)
    assert anim[-1] == cc.Pose(49.0, -45.0)
    assert anim[1].arm == 11.0 and anim[1].bucket_abs == -22.5


def test_pose_relative_angle_and_key():
    pose = cc.Pose(30.0, -20.0)
    assert pose.bucket_rel == -50.0
    assert pose.key == "arm+030.00_bucket-020.00"
    assert cc.parse_pose_list("30:-20, -27.7:0") == [cc.Pose(30.0, -20.0), cc.Pose(-27.7, 0.0)]


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
    ]}
    assert sb.round_up_2sig(649.0) == 650.0 and sb.round_up_2sig(309882.0) == 310000.0
    budgets = sb.suggest(results, margin=0.15, floor=50.0)
    assert budgets == {"arms/frame": 72.0, "frame/hydraulics": 240000.0}


def test_snap_volume_drops_boolean_noise():
    assert cc.snap_volume(1e-7) == 0.0
    assert cc.snap_volume(-1e-7) == 0.0
    assert cc.snap_volume(-58.4) == 58.4
    assert cc.overlap_verdict(cc.snap_volume(1e-7), 50.0) == (True, "clear")
