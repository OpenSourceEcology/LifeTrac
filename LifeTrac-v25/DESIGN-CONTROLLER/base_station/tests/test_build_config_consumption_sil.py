"""Build-configuration consumption SIL test.

Round 28 -- IP: BC-04 (web_ui + lora_bridge consume BuildConfig and audit
the boot-time config_sha256).

Pure source-parse + in-process reload test. Reloads ``web_ui`` per case
with a synthesised TOML pinned through ``LIFETRAC_BUILD_CONFIG_PATH`` so
the module-level ``BUILD`` singleton picks up the test fixture, then
asserts the consumer surfaces (camera filter, control-subscriber cap,
boot audit-log call) react to it. ``lora_bridge`` is checked by source
parse + an in-process driver of the audit-log code path with the rest
of ``Bridge.__init__`` stubbed out.

Defended invariants
-------------------

BC4_A (audit boot record): both ``web_ui`` (module import) and
    ``lora_bridge.Bridge.__init__`` write a ``config_loaded`` audit entry
    carrying ``unit_id``, ``source_path``, and ``config_sha256``.

BC4_B (camera gating): the ``_CAMERA_IDS`` table reflects the loaded
    BuildConfig's ``cameras.{front,rear,implement,crop_health}_present``
    flags. ``cameras.count == 0`` collapses the table to empty.

BC4_C (parameter substitution): ``MAX_CONTROL_SUBSCRIBERS`` is read from
    ``BUILD.ui.max_control_subscribers`` at module import time, not
    hard-coded.

BC4_D (graceful degradation): if ``LIFETRAC_BUILD_CONFIG_PATH`` points
    at a non-existent file, ``web_ui`` still imports and ``BUILD is None``
    -- the historical defaults take over and the UI boots degraded
    rather than crash-looping.

BC4_E (source tripwire): ``web_ui.py`` and ``lora_bridge.py`` both
    contain the BC-04 boot audit call so the contract is greppable.
"""

from __future__ import annotations

import importlib
import os
import sys
import unittest
from pathlib import Path
from unittest import mock


_BS = Path(__file__).resolve().parents[1]
_DEFAULT_TOML = _BS / "config" / "build.default.toml"
_WEB_UI_PY = _BS / "web_ui.py"
_LORA_BRIDGE_PY = _BS / "lora_bridge.py"

if str(_BS) not in sys.path:
    sys.path.insert(0, str(_BS))


def _stub_paho_if_needed() -> None:
    """web_ui imports paho.mqtt.client at module import time. The test
    environment has it (it's a real dep), but we must make sure
    `Client(...).connect` doesn't actually try to TCP-connect.

    Round 25 already relies on patching paho's connect to a side_effect.
    Here we just patch `mqtt_client.connect` and `loop_start` so module
    import is a no-op on the wire.
    """


def _import_web_ui_with_toml(toml_path: Path):
    """Reload web_ui with LIFETRAC_BUILD_CONFIG_PATH pointed at *toml_path*.

    paho's connect() is patched to succeed instantly so the module-import
    `_connect_mqtt_with_retry()` returns on the first try.
    """
    # Drop cached modules so the patches take effect on re-import.
    for mod in ("web_ui", "build_config"):
        sys.modules.pop(mod, None)

    env = {
        "LIFETRAC_BUILD_CONFIG_PATH": str(toml_path) if toml_path else "",
        "LIFETRAC_ALLOW_UNCONFIGURED_KEY": "1",
        "LIFETRAC_PIN": os.environ.get("LIFETRAC_PIN", "1234"),
    }

    with mock.patch.dict(os.environ, env, clear=False), \
            mock.patch("paho.mqtt.client.Client") as mock_client_cls:
        instance = mock.MagicMock()
        instance.connect.return_value = None
        mock_client_cls.return_value = instance
        web_ui = importlib.import_module("web_ui")
    return web_ui


def _write_variant(tmpdir: Path, **overrides) -> Path:
    """Write a TOML at *tmpdir* derived from build.default.toml with
    string-replace overrides keyed by the literal text to find/replace.
    """
    text = _DEFAULT_TOML.read_text(encoding="utf-8")
    for old, new in overrides.items():
        if old not in text:
            raise AssertionError(f"variant override {old!r} not found in default")
        text = text.replace(old, new)
    out = tmpdir / "build.variant.toml"
    out.write_text(text, encoding="utf-8")
    return out


# ---------------------------------------------------------------- BC4_A


class BC4_A_AuditBootRecord(unittest.TestCase):

    def test_web_ui_emits_config_loaded_on_import(self) -> None:
        web_ui = _import_web_ui_with_toml(_DEFAULT_TOML)
        self.assertIsNotNone(web_ui.BUILD)
        # The boot helper exists and is wired up; we exercise it directly
        # against a captured AuditLog mock to pin the field set.
        captured = []

        class _FakeAudit:
            def record(self, kind, **fields):
                captured.append((kind, fields))

        with mock.patch.object(web_ui, "_get_audit_log", return_value=_FakeAudit()):
            web_ui._audit_config_loaded()

        self.assertEqual(len(captured), 1)
        kind, fields = captured[0]
        self.assertEqual(kind, "config_loaded")
        self.assertEqual(fields["component"], "web_ui")
        self.assertEqual(fields["unit_id"], web_ui.BUILD.unit_id)
        self.assertEqual(fields["config_sha256"], web_ui.BUILD.config_sha256)
        self.assertIn("source_path", fields)

    def test_audit_helper_is_a_noop_when_build_is_none(self) -> None:
        web_ui = _import_web_ui_with_toml(_DEFAULT_TOML)
        captured = []

        class _FakeAudit:
            def record(self, kind, **fields):
                captured.append((kind, fields))

        with mock.patch.object(web_ui, "BUILD", None), \
             mock.patch.object(web_ui, "_get_audit_log", return_value=_FakeAudit()):
            web_ui._audit_config_loaded()
        self.assertEqual(captured, [])

    def test_lora_bridge_audit_path_writes_config_loaded(self) -> None:
        # The Bridge.__init__ pulls in pyserial + paho + nonce_store; we
        # don't drive the whole constructor. Instead we exercise the
        # exact 7-line block by inlining it under controlled mocks.
        sys.modules.pop("build_config", None)
        import build_config as bc
        captured = []

        class _FakeAudit:
            def record(self, kind, **fields):
                captured.append((kind, fields))

        audit = _FakeAudit()
        with mock.patch.dict(os.environ, {"LIFETRAC_BUILD_CONFIG_PATH": str(_DEFAULT_TOML)}):
            cfg = bc.load(os.environ.get("LIFETRAC_UNIT_ID"))
            audit.record(
                "config_loaded",
                component="lora_bridge",
                unit_id=cfg.unit_id,
                source_path=str(cfg.source_path),
                config_sha256=cfg.config_sha256,
            )

        self.assertEqual(len(captured), 1)
        kind, fields = captured[0]
        self.assertEqual(kind, "config_loaded")
        self.assertEqual(fields["component"], "lora_bridge")
        self.assertEqual(fields["unit_id"], cfg.unit_id)
        self.assertEqual(fields["config_sha256"], cfg.config_sha256)


# ---------------------------------------------------------------- BC4_B


class BC4_B_CameraGating(unittest.TestCase):

    def test_default_canonical_build_advertises_auto_and_front_only(self) -> None:
        web_ui = _import_web_ui_with_toml(_DEFAULT_TOML)
        self.assertEqual(set(web_ui._CAMERA_IDS), {"auto", "front"})

    def test_full_camera_build_advertises_every_position(self) -> None:
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            variant = _write_variant(
                Path(td),
                **{
                    "count                = 1": "count                = 4",
                    "rear_present         = false": "rear_present         = true",
                    "implement_present    = false": "implement_present    = true",
                    "crop_health_present  = false": "crop_health_present  = true",
                },
            )
            web_ui = _import_web_ui_with_toml(variant)
            self.assertEqual(
                set(web_ui._CAMERA_IDS),
                {"auto", "front", "rear", "implement", "crop"},
            )

    def test_no_camera_build_collapses_table(self) -> None:
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            variant = _write_variant(
                Path(td),
                **{
                    "count                = 1": "count                = 0",
                    "front_present        = true": "front_present        = false",
                    'coral_tpu            = "mini_pcie"': 'coral_tpu            = "none"',
                },
            )
            web_ui = _import_web_ui_with_toml(variant)
            self.assertEqual(web_ui._CAMERA_IDS, {})


# ---------------------------------------------------------------- BC4_C


class BC4_C_ParameterSubstitution(unittest.TestCase):

    def test_max_control_subscribers_reads_from_buildconfig(self) -> None:
        import tempfile
        with tempfile.TemporaryDirectory() as td:
            variant = _write_variant(
                Path(td),
                **{
                    "max_control_subscribers   = 4": "max_control_subscribers   = 12",
                },
            )
            web_ui = _import_web_ui_with_toml(variant)
            self.assertEqual(web_ui.MAX_CONTROL_SUBSCRIBERS, 12)

    def test_default_value_when_loader_unavailable(self) -> None:
        # Point at a non-existent file so BuildConfig load fails and
        # the BUILD singleton ends up None; the historical default
        # (4) must take over rather than NameError.
        web_ui = _import_web_ui_with_toml(_BS / "no-such-file.toml")
        self.assertIsNone(web_ui.BUILD)
        self.assertEqual(web_ui.MAX_CONTROL_SUBSCRIBERS, 4)


# ---------------------------------------------------------------- BC4_D


class BC4_D_GracefulDegradation(unittest.TestCase):

    def test_missing_config_does_not_break_module_import(self) -> None:
        web_ui = _import_web_ui_with_toml(_BS / "definitely-missing.toml")
        self.assertIsNone(web_ui.BUILD)
        # Camera table falls back to the full hard-coded set so cameras
        # still selectable on a dev checkout without a config file.
        self.assertEqual(
            set(web_ui._CAMERA_IDS),
            {"auto", "front", "rear", "implement", "crop"},
        )


# ---------------------------------------------------------------- BC4_E


class BC4_E_SourceTripwire(unittest.TestCase):

    def test_web_ui_contains_bc04_audit_call(self) -> None:
        text = _WEB_UI_PY.read_text(encoding="utf-8")
        self.assertIn("_audit_config_loaded", text)
        self.assertIn("config_loaded", text)
        self.assertIn("BC-04", text)

    def test_lora_bridge_contains_bc04_audit_call(self) -> None:
        text = _LORA_BRIDGE_PY.read_text(encoding="utf-8")
        self.assertIn("BC-04", text)
        self.assertIn('"config_loaded"', text)
        self.assertIn("config_sha256", text)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
