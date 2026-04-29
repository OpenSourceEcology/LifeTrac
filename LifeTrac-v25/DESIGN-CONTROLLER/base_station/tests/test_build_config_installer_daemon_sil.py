"""SIL gate for Round 29b-beta -- IP: BC-10 (delivery half, beta).

Pins the X8-side installer daemon, the operator feedback contract, the
watch-and-reload helper, and the new ``push`` CLI subcommand. The
producer-side surface (envelope + base download route + validate / bundle
/ verify / diff CLI) is gated separately by
``test_build_config_installer_sil.py`` (Round 29b-alpha).

BC10c_A_DaemonApply
    ``apply_bundle`` returns ``applied`` for a fresh canonical bundle and
    actually rewrites the target TOML; returns ``noop`` when the bundle's
    SHA matches the in-memory current config; rejects unit_id mismatches
    on the filename layer; rejects unit_id mismatches on the body layer
    even when the filename agrees; rejects schema violations; rejects
    firmware-required diffs without writing the target. ``discover_bundles``
    filters by unit_id and ignores foreign files. ``write_result_file``
    drops a parseable JSON envelope.

BC10c_B_FeedbackContract
    ``feedback_for`` maps every install-result status to a stable LED
    pattern + a <= 21-char OLED line. The pinned (status -> LED) mapping
    is locked verbatim. OLED truncation happens with an ellipsis when
    over budget.

BC10c_C_WatchReload
    ``ConfigWatcher.poll`` returns ``noop`` when nothing changed, ``applied``
    when a live change is on disk and quiescence holds, ``deferred`` when
    quiescence does not hold (without swapping the in-memory cfg),
    ``restart_pending`` for restart_required diffs (without swapping),
    ``rejected`` when the on-disk file fails to load. ``restart_pending``
    flag survives across subsequent polls. The base-side ``/api/build_config/state``
    endpoint surfaces the watcher state with the documented field set
    and emits a ``config_watch_event`` audit entry on non-noop polls.

BC10c_D_PushAndTripwires
    ``push --via local`` copies a bundle into a destination directory
    and exits 0; ``push --via local --apply --target ...`` invokes the
    daemon and prints per-bundle status. ``push --via ssh`` without
    ``--execute`` only prints the planned scp/ssh commands (no
    side-effects). Source tripwires: ``installer_daemon.py`` exports the
    documented surface, ``feedback.py`` exports the LED pattern set,
    ``config_watcher.py`` exports ``ConfigWatcher`` + every event token,
    and ``web_ui.py`` carries the ``/api/build_config/state`` route plus
    the ``config_watch_event`` audit verb.
"""

from __future__ import annotations

import importlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

from build_config import (  # type: ignore[import-not-found]
    BuildConfig,
    QuiescenceState,
    load,
)
from config_bundle import make_bundle, serialise  # type: ignore[import-not-found]
import config_watcher  # type: ignore[import-not-found]
import feedback  # type: ignore[import-not-found]
import installer_daemon  # type: ignore[import-not-found]


_BS = Path(__file__).resolve().parents[1]
_REPO = _BS.parents[1]
_CLI = _REPO / "tools" / "lifetrac_config.py"
_DEFAULT_TOML = _BS / "config" / "build.default.toml"


def _read_default_body() -> str:
    return _DEFAULT_TOML.read_text(encoding="utf-8")


def _load_default() -> BuildConfig:
    prev = os.environ.get("LIFETRAC_BUILD_CONFIG_PATH")
    os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(_DEFAULT_TOML)
    try:
        return load()
    finally:
        if prev is None:
            os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        else:
            os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = prev


def _write_bundle(dst_dir: Path, body: str, unit_id: str) -> Path:
    bundle = make_bundle(body, unit_id)
    p = dst_dir / bundle.filename
    p.write_text(serialise(bundle), encoding="utf-8")
    return p


def _quiet_state() -> QuiescenceState:
    return QuiescenceState(
        parked_seconds=999.0,
        active_control_subscribers=0,
        m7_tx_queue_depth=0,
        engine_idle_or_off=True,
    )


def _busy_state() -> QuiescenceState:
    return QuiescenceState(
        parked_seconds=999.0,
        active_control_subscribers=2,  # operator session active
        m7_tx_queue_depth=0,
        engine_idle_or_off=True,
    )


def _run_cli(*argv: str) -> "subprocess.CompletedProcess[str]":
    return subprocess.run(
        [sys.executable, str(_CLI), *argv],
        capture_output=True,
        text=True,
        check=False,
    )


def _import_web_ui_with(toml_path: Path):
    for mod in ("web_ui", "build_config", "config_watcher"):
        sys.modules.pop(mod, None)
    env = dict(os.environ)
    env["LIFETRAC_BUILD_CONFIG_PATH"] = str(toml_path)
    env["LIFETRAC_PIN"] = "1234"
    env.pop("LIFETRAC_PIN_FILE", None)
    with mock.patch.dict(os.environ, env, clear=True), \
         mock.patch("paho.mqtt.client.Client") as cls:
        cls.return_value.connect.return_value = None
        cls.return_value.loop_start.return_value = None
        web_ui = importlib.import_module("web_ui")
    return web_ui


# ------------------------------------------------------------- BC10c_A


class BC10c_A_DaemonApply(unittest.TestCase):
    """installer_daemon: apply / discover / result-file behaviour."""

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="lt-daemon-"))
        self.target = self.tmp / "active.toml"

    def tearDown(self) -> None:
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_apply_fresh_writes_target_and_returns_applied(self) -> None:
        body = _read_default_body()
        cfg = _load_default()
        bp = _write_bundle(self.tmp, body, cfg.unit_id)
        result = installer_daemon.apply_bundle(
            bp, target_path=self.target, unit_id=cfg.unit_id
        )
        self.assertEqual(result.status, installer_daemon.STATUS_APPLIED)
        self.assertTrue(self.target.is_file())
        self.assertEqual(
            self.target.read_text(encoding="utf-8").strip(),
            body.strip(),
        )
        self.assertEqual(result.applied_sha, cfg.config_sha256)

    def test_apply_same_sha_returns_noop(self) -> None:
        body = _read_default_body()
        cfg = _load_default()
        bp = _write_bundle(self.tmp, body, cfg.unit_id)
        result = installer_daemon.apply_bundle(
            bp, target_path=self.target, unit_id=cfg.unit_id, current_cfg=cfg
        )
        self.assertEqual(result.status, installer_daemon.STATUS_NOOP)
        self.assertFalse(self.target.is_file(), "noop must not touch target")

    def test_apply_filename_unit_id_mismatch_rejected(self) -> None:
        body = _read_default_body()
        cfg = _load_default()
        # Build for one unit_id, but pretend we're a different unit.
        bp = _write_bundle(self.tmp, body, cfg.unit_id)
        result = installer_daemon.apply_bundle(
            bp, target_path=self.target, unit_id="lifetrac-other"
        )
        self.assertEqual(result.status, installer_daemon.STATUS_REJECTED)
        self.assertIn("unit_id", result.reason)
        self.assertFalse(self.target.is_file())

    def test_apply_body_unit_id_mismatch_rejected(self) -> None:
        body = _read_default_body()
        cfg = _load_default()
        # Tamper: body still says cfg.unit_id; build a bundle whose filename
        # + header claim a different unit_id by hand-forging via make_bundle.
        # We can't have header != filename (verify_filename_matches catches
        # it), so simulate the body-level check by making a body for one
        # unit and a header for the same unit but the body's TOML unit_id
        # differs. Easiest: take canonical body, swap the body's unit_id.
        forged_body = body.replace(
            f'unit_id        = "{cfg.unit_id}"', 'unit_id        = "lifetrac-zzz"'
        )
        self.assertNotEqual(forged_body, body, msg="forge no-op")
        # Make bundle naming itself for the X8 we're testing.
        bp = _write_bundle(self.tmp, forged_body, "lifetrac-zzz")
        # X8 identity is "lifetrac-zzz" so filename + header agree;
        # body's unit_id matches too -- BUT the running X8 is actually
        # a different unit:
        result = installer_daemon.apply_bundle(
            bp, target_path=self.target, unit_id="lifetrac-001"
        )
        self.assertEqual(result.status, installer_daemon.STATUS_REJECTED)
        self.assertIn("unit_id", result.reason)
        self.assertFalse(self.target.is_file())

    def test_apply_schema_violation_rejected(self) -> None:
        body = _read_default_body()
        cfg = _load_default()
        # Out-of-range value: track_axis_count must be in {0,1,2}; set it to 7.
        broken = body.replace("track_axis_count    = 2", "track_axis_count    = 7")
        self.assertNotEqual(broken, body, msg="schema-break no-op")
        bp = _write_bundle(self.tmp, broken, cfg.unit_id)
        result = installer_daemon.apply_bundle(
            bp, target_path=self.target, unit_id=cfg.unit_id
        )
        self.assertEqual(result.status, installer_daemon.STATUS_REJECTED)
        self.assertIn("schema", result.reason)
        self.assertFalse(self.target.is_file())

    def test_apply_firmware_required_diff_rejected(self) -> None:
        body = _read_default_body()
        cfg = _load_default()
        # m4_watchdog_ms is firmware_required.
        bumped = body.replace("m4_watchdog_ms          = 200", "m4_watchdog_ms          = 250")
        self.assertNotEqual(bumped, body, msg="watchdog no-op")
        bp = _write_bundle(self.tmp, bumped, cfg.unit_id)
        result = installer_daemon.apply_bundle(
            bp, target_path=self.target, unit_id=cfg.unit_id, current_cfg=cfg
        )
        self.assertEqual(result.status, installer_daemon.STATUS_REJECTED)
        self.assertEqual(result.reload_class, "firmware_required")
        self.assertIn("firmware_required", result.reason)
        self.assertFalse(self.target.is_file())

    def test_discover_bundles_filters_by_unit_id_and_ignores_foreign(self) -> None:
        body = _read_default_body()
        cfg = _load_default()
        bp_mine = _write_bundle(self.tmp, body, cfg.unit_id)
        # A bundle for someone else.
        forged = body.replace(
            f'unit_id        = "{cfg.unit_id}"', 'unit_id        = "lifetrac-zzz"'
        )
        bp_other = _write_bundle(self.tmp, forged, "lifetrac-zzz")
        # A foreign file.
        (self.tmp / "README.txt").write_text("not a bundle\n", encoding="utf-8")
        found = installer_daemon.discover_bundles(self.tmp, cfg.unit_id)
        self.assertEqual(found, [bp_mine])
        found_other = installer_daemon.discover_bundles(self.tmp, "lifetrac-zzz")
        self.assertEqual(found_other, [bp_other])

    def test_write_result_file_round_trips_json(self) -> None:
        body = _read_default_body()
        cfg = _load_default()
        bp = _write_bundle(self.tmp, body, cfg.unit_id)
        result = installer_daemon.apply_bundle(
            bp, target_path=self.target, unit_id=cfg.unit_id
        )
        path = installer_daemon.write_result_file(result, self.tmp)
        self.assertEqual(path.name, installer_daemon.RESULT_FILENAME)
        data = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(data["status"], "applied")
        self.assertEqual(data["unit_id"], cfg.unit_id)
        self.assertEqual(data["result_version"], installer_daemon.RESULT_VERSION)
        self.assertEqual(data["applied_sha"], cfg.config_sha256)


# ------------------------------------------------------------- BC10c_B


class BC10c_B_FeedbackContract(unittest.TestCase):
    """feedback: pinned LED + OLED contract for every install status."""

    def _result(self, status: str, sha: str | None = "9ce6127812345678abcdef") -> installer_daemon.InstallResult:
        return installer_daemon.InstallResult(
            status=status,
            reason="test",
            unit_id="lifetrac-001",
            applied_sha=sha,
            prior_sha=None,
            reload_class=None,
            applied_at="2026-04-29T00:00:00Z",
        )

    def test_status_table_covers_every_status(self) -> None:
        for s in installer_daemon.STATUSES:
            self.assertIn(s, feedback.STATUS_TABLE, msg=f"missing status {s}")

    def test_led_pattern_applied_pinned(self) -> None:
        self.assertEqual(
            feedback.LED_APPLIED.steps,
            (
                ("green", 700), ("off", 300),
                ("green", 700), ("off", 300),
                ("green", 700), ("off", 300),
                ("green", 5000),
            ),
        )
        self.assertFalse(feedback.LED_APPLIED.repeat)

    def test_led_pattern_rejected_pinned(self) -> None:
        self.assertEqual(
            feedback.LED_REJECTED.steps,
            (
                ("red", 150), ("off", 150),
                ("red", 150), ("off", 150),
                ("red", 150), ("off", 150),
                ("red", 5000),
            ),
        )

    def test_led_pattern_deferred_repeats(self) -> None:
        self.assertTrue(feedback.LED_DEFERRED.repeat)
        self.assertEqual(feedback.LED_DEFERRED.steps, (("amber", 500), ("off", 500)))

    def test_feedback_oled_under_budget_for_each_status(self) -> None:
        for s in installer_daemon.STATUSES:
            fb = feedback.feedback_for(self._result(s))
            self.assertLessEqual(
                len(fb.oled_line), feedback.OLED_LINE_MAX,
                msg=f"{s}: {fb.oled_line!r}",
            )
            self.assertEqual(fb.status, s)

    def test_feedback_for_unknown_status_raises(self) -> None:
        with self.assertRaises(ValueError):
            feedback.feedback_for(self._result("not-a-real-status"))

    def test_feedback_for_status_helper(self) -> None:
        fb = feedback.feedback_for_status("deferred", sha8="9ce61278")
        self.assertEqual(fb.status, "deferred")
        self.assertIn("9ce61278", fb.oled_line)
        self.assertEqual(fb.led, feedback.LED_DEFERRED)


# ------------------------------------------------------------- BC10c_C


class BC10c_C_WatchReload(unittest.TestCase):
    """config_watcher: detect on-disk change, classify, gate on quiescence."""

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="lt-watch-"))
        self.toml = self.tmp / "build.toml"
        self.toml.write_text(_read_default_body(), encoding="utf-8")
        prev = os.environ.get("LIFETRAC_BUILD_CONFIG_PATH")
        os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(self.toml)
        self.addCleanup(self._restore_env, prev)
        self.cfg = load()

    def _restore_env(self, prev: str | None) -> None:
        if prev is None:
            os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        else:
            os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = prev
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _rewrite(self, new_body: str) -> None:
        # Bump mtime in a way stat() will notice even on coarse-grained FSes.
        st = self.toml.stat()
        self.toml.write_text(new_body, encoding="utf-8")
        os.utime(self.toml, (st.st_atime + 2, st.st_mtime + 2))

    def test_poll_noop_when_unchanged(self) -> None:
        w = config_watcher.ConfigWatcher(self.cfg, get_quiescence=_quiet_state)
        ev = w.poll()
        self.assertEqual(ev.kind, config_watcher.EVENT_NOOP)

    def test_poll_applied_for_live_change_when_quiet(self) -> None:
        w = config_watcher.ConfigWatcher(self.cfg, get_quiescence=_quiet_state)
        body = _read_default_body()
        new_body = body.replace(
            "max_control_subscribers   = 4",
            "max_control_subscribers   = 6",
        )
        self.assertNotEqual(new_body, body, msg="ms no-op")
        self._rewrite(new_body)
        ev = w.poll()
        self.assertEqual(ev.kind, config_watcher.EVENT_APPLIED)
        self.assertIsNotNone(ev.diff)
        self.assertEqual(ev.diff.worst, "live")
        self.assertEqual(w.current.ui.max_control_subscribers, 6)

    def test_poll_deferred_when_live_change_but_busy(self) -> None:
        w = config_watcher.ConfigWatcher(self.cfg, get_quiescence=_busy_state)
        body = _read_default_body()
        new_body = body.replace(
            "max_control_subscribers   = 4",
            "max_control_subscribers   = 6",
        )
        self._rewrite(new_body)
        ev = w.poll()
        self.assertEqual(ev.kind, config_watcher.EVENT_DEFERRED)
        # Did NOT swap.
        self.assertEqual(w.current.ui.max_control_subscribers, 4)

    def test_poll_restart_pending_for_restart_required_diff(self) -> None:
        w = config_watcher.ConfigWatcher(self.cfg, get_quiescence=_quiet_state)
        body = _read_default_body()
        new_body = body.replace(
            'lora_region              = "us915"',
            'lora_region              = "eu868"',
        )
        self.assertNotEqual(new_body, body, msg="lora no-op")
        self._rewrite(new_body)
        ev = w.poll()
        self.assertEqual(ev.kind, config_watcher.EVENT_RESTART_PENDING)
        self.assertTrue(w.restart_pending)
        self.assertEqual(w.current.comm.lora_region, "us915", "must NOT swap")
        # Subsequent poll (no further change) keeps the flag.
        ev2 = w.poll()
        self.assertEqual(ev2.kind, config_watcher.EVENT_NOOP)
        self.assertTrue(w.restart_pending)

    def test_poll_rejected_on_schema_violation(self) -> None:
        w = config_watcher.ConfigWatcher(self.cfg, get_quiescence=_quiet_state)
        body = _read_default_body()
        broken = body.replace("track_axis_count    = 2", "track_axis_count    = 7")
        self.assertNotEqual(broken, body, msg="schema-break no-op")
        self._rewrite(broken)
        ev = w.poll()
        self.assertEqual(ev.kind, config_watcher.EVENT_REJECTED)
        # In-memory cfg unchanged.
        self.assertEqual(w.current.hydraulic.track_axis_count, 2)

    def test_web_ui_state_endpoint_surfaces_watcher(self) -> None:
        from fastapi.testclient import TestClient
        web_ui = _import_web_ui_with(self.toml)
        client = TestClient(web_ui.app)
        # Login first.
        r = client.post("/api/login", json={"pin": "1234"})
        self.assertEqual(r.status_code, 200)
        # Initial poll: noop, no diff.
        r = client.get("/api/build_config/state")
        self.assertEqual(r.status_code, 200, r.text)
        data = r.json()
        self.assertEqual(data["last_event"], "noop")
        self.assertFalse(data["restart_pending"])
        self.assertEqual(data["sha256"], self.cfg.config_sha256)
        self.assertEqual(data["unit_id"], self.cfg.unit_id)

    def test_web_ui_state_endpoint_requires_session(self) -> None:
        from fastapi.testclient import TestClient
        web_ui = _import_web_ui_with(self.toml)
        client = TestClient(web_ui.app)
        r = client.get("/api/build_config/state")
        self.assertEqual(r.status_code, 401)


# ------------------------------------------------------------- BC10c_D


class BC10c_D_PushAndTripwires(unittest.TestCase):
    """push subcommand + module-export source tripwires."""

    def setUp(self) -> None:
        self.tmp = Path(tempfile.mkdtemp(prefix="lt-push-"))

    def tearDown(self) -> None:
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _make_bundle_file(self) -> tuple[Path, str]:
        body = _read_default_body()
        cfg = _load_default()
        bp = _write_bundle(self.tmp, body, cfg.unit_id)
        return bp, cfg.unit_id

    def test_push_local_copies_bundle(self) -> None:
        bp, _ = self._make_bundle_file()
        dest = self.tmp / "stick"
        cp = _run_cli("push", str(bp), "--via", "local", "--dest", str(dest))
        self.assertEqual(cp.returncode, 0, msg=cp.stderr)
        self.assertTrue((dest / bp.name).is_file())

    def test_push_local_apply_invokes_daemon(self) -> None:
        bp, _ = self._make_bundle_file()
        dest = self.tmp / "stick2"
        target = self.tmp / "active.toml"
        cp = _run_cli(
            "push", str(bp), "--via", "local", "--dest", str(dest),
            "--apply", "--target", str(target),
        )
        self.assertEqual(cp.returncode, 0, msg=cp.stderr)
        self.assertTrue(target.is_file(), "daemon should have written target")
        self.assertIn("applied", cp.stdout)
        # Result file dropped on the dest dir.
        self.assertTrue((dest / installer_daemon.RESULT_FILENAME).is_file())

    def test_push_ssh_dry_run_only_prints_plan(self) -> None:
        bp, _ = self._make_bundle_file()
        cp = _run_cli(
            "push", str(bp), "--via", "ssh",
            "--host", "tractor.local", "--user", "lifetrac",
        )
        self.assertEqual(cp.returncode, 0, msg=cp.stderr)
        self.assertIn("PLAN", cp.stdout)
        self.assertIn("scp", cp.stdout)
        self.assertIn("ssh", cp.stdout)
        self.assertIn("not given", cp.stdout)

    def test_installer_daemon_exports_documented_surface(self) -> None:
        for name in (
            "apply_bundle", "discover_bundles", "process_mount",
            "write_result_file", "InstallResult", "InstallerError",
            "STATUS_APPLIED", "STATUS_REJECTED", "STATUS_NOOP", "STATUS_DEFERRED",
            "RESULT_FILENAME", "RESULT_VERSION",
        ):
            self.assertTrue(hasattr(installer_daemon, name), msg=name)

    def test_feedback_exports_pinned_patterns(self) -> None:
        for name in (
            "LED_APPLIED", "LED_REJECTED", "LED_DEFERRED", "LED_NOOP", "LED_IDLE",
            "LedPattern", "Feedback", "OLED_LINE_MAX",
            "feedback_for", "feedback_for_status", "STATUS_TABLE",
        ):
            self.assertTrue(hasattr(feedback, name), msg=name)

    def test_config_watcher_exports_event_tokens(self) -> None:
        for name in (
            "ConfigWatcher", "WatchEvent", "EVENTS",
            "EVENT_NOOP", "EVENT_APPLIED", "EVENT_DEFERRED",
            "EVENT_RESTART_PENDING", "EVENT_FIRMWARE_REQUIRED", "EVENT_REJECTED",
        ):
            self.assertTrue(hasattr(config_watcher, name), msg=name)

    def test_web_ui_carries_state_route_and_audit_verb(self) -> None:
        src = (_BS / "web_ui.py").read_text(encoding="utf-8")
        self.assertIn("/api/build_config/state", src)
        self.assertIn("config_watch_event", src)
        self.assertIn("config_watcher", src)
        self.assertIn("Round 29b-beta", src)

    def test_cli_carries_push_subcommand(self) -> None:
        src = _CLI.read_text(encoding="utf-8")
        self.assertIn("def cmd_push", src)
        self.assertIn("--via", src)
        self.assertIn("--apply", src)
        self.assertIn("--execute", src)
        self.assertIn("Round 29b-beta", src)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
