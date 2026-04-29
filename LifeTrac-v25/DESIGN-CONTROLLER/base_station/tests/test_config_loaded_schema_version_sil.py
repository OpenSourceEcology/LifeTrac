"""SIL gate for ``config_loaded`` schema_version field (Round 40 / BC-14B).

Round 37 / BC-14 introduced ``lifetrac-config inventory`` with a
``schema_version`` column in the aggregated table, but the actual
emitters in ``web_ui._audit_config_loaded`` and ``Bridge.__init__``
did NOT include the field \u2014 the column rendered empty.

Round 40 closes the gap: both emitters now write
``schema_version=BUILD.schema_version``. This gate pins the contract:

* BC14B_A: ``web_ui._audit_config_loaded`` writes ``schema_version``
  pulled from the active ``BuildConfig``.
* BC14B_B: the ``lora_bridge`` boot block writes the same field
  (we exercise the inline 8-line block under controlled mocks the
  same way ``test_build_config_consumption_sil.py`` does, because
  ``Bridge.__init__`` drags in pyserial + paho).
* BC14B_C: end-to-end \u2014 fixture ``audit.jsonl`` lines that
  include ``schema_version`` flow through the inventory aggregator
  and surface in the rendered CSV; lines without it still aggregate
  cleanly (back-compat with rotated logs from before this change).
* BC14B_D: source greps \u2014 both emitter call-sites contain
  ``schema_version=`` so a future refactor that drops the field
  fails the gate even if the test happens to mock around it.
"""

from __future__ import annotations

import csv
import importlib
import io
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

_BS = Path(__file__).resolve().parents[1]
_REPO = _BS.parents[1]
_TOOLS = _REPO / "tools"

if str(_BS) not in sys.path:
    sys.path.insert(0, str(_BS))
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

import build_config as _bc  # noqa: E402
import lifetrac_config as _lc  # noqa: E402

_DEFAULT_TOML = _BS / "config" / "build.default.toml"


def _import_web_ui_with_toml(toml_path: Path):
    """Reload web_ui with a known config so module-level BUILD is set.

    paho's Client is mocked because module import calls connect()."""
    for mod in ("web_ui", "build_config"):
        sys.modules.pop(mod, None)
    env = {
        "LIFETRAC_BUILD_CONFIG_PATH": str(toml_path),
        "LIFETRAC_ALLOW_UNCONFIGURED_KEY": "1",
        "LIFETRAC_PIN": os.environ.get("LIFETRAC_PIN", "1234"),
    }
    with mock.patch.dict(os.environ, env, clear=False), \
            mock.patch("paho.mqtt.client.Client") as mock_client_cls:
        instance = mock.MagicMock()
        instance.connect.return_value = None
        mock_client_cls.return_value = instance
        return importlib.import_module("web_ui")


class BC14B_A_WebUIEmitsSchemaVersion(unittest.TestCase):
    def test_web_ui_config_loaded_includes_schema_version(self) -> None:
        web_ui = _import_web_ui_with_toml(_DEFAULT_TOML)
        self.assertIsNotNone(web_ui.BUILD)
        captured: list = []

        class _FakeAudit:
            def record(self, kind, **fields):
                captured.append((kind, fields))

        with mock.patch.object(web_ui, "_get_audit_log",
                               return_value=_FakeAudit()):
            web_ui._audit_config_loaded()

        self.assertEqual(len(captured), 1)
        kind, fields = captured[0]
        self.assertEqual(kind, "config_loaded")
        self.assertIn("schema_version", fields)
        self.assertEqual(fields["schema_version"], web_ui.BUILD.schema_version)
        # Round 40 contract: schema_version is an int, not a stringified one.
        self.assertIsInstance(fields["schema_version"], int)


class BC14B_B_LoraBridgeBlockEmitsSchemaVersion(unittest.TestCase):
    def test_lora_bridge_inline_block_writes_schema_version(self) -> None:
        sys.modules.pop("build_config", None)
        import build_config as bc
        captured: list = []

        class _FakeAudit:
            def record(self, kind, **fields):
                captured.append((kind, fields))

        audit = _FakeAudit()
        with mock.patch.dict(os.environ,
                             {"LIFETRAC_BUILD_CONFIG_PATH": str(_DEFAULT_TOML)}):
            cfg = bc.load(os.environ.get("LIFETRAC_UNIT_ID"))
            # Mirror the exact lora_bridge.Bridge.__init__ block:
            audit.record(
                "config_loaded",
                component="lora_bridge",
                unit_id=cfg.unit_id,
                source_path=str(cfg.source_path),
                config_sha256=cfg.config_sha256,
                schema_version=cfg.schema_version,
            )
        self.assertEqual(len(captured), 1)
        _, fields = captured[0]
        self.assertEqual(fields["schema_version"], cfg.schema_version)


class BC14B_C_InventorySurfacesSchemaVersion(unittest.TestCase):
    def test_inventory_csv_populates_schema_version_column(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            log = Path(td) / "audit.jsonl"
            with open(log, "w", encoding="utf-8", newline="\n") as fp:
                # Two records with schema_version=1, one without (rotated
                # log from before Round 40). Aggregator must still produce
                # one row per (unit_id, sha) and surface schema_version
                # whenever ANY record carries it.
                fp.write(json.dumps({
                    "ts": 1700000000.0, "event": "config_loaded",
                    "component": "web_ui",
                    "unit_id": "lifetrac-001", "source_path": "/etc/x.toml",
                    "config_sha256": "a" * 64, "schema_version": 1,
                }) + "\n")
                fp.write(json.dumps({
                    "ts": 1700000005.0, "event": "config_loaded",
                    "component": "lora_bridge",
                    "unit_id": "lifetrac-001", "source_path": "/etc/x.toml",
                    "config_sha256": "a" * 64, "schema_version": 1,
                }) + "\n")
                fp.write(json.dumps({
                    "ts": 1700000010.0, "event": "config_loaded",
                    "component": "web_ui",
                    "unit_id": "lifetrac-002", "source_path": "/etc/y.toml",
                    "config_sha256": "b" * 64,  # no schema_version (legacy)
                }) + "\n")
            rows = _lc.aggregate_inventory(_lc.iter_config_loaded_events([log]))
            text = _lc.render_inventory_csv(rows)
        reader = csv.reader(io.StringIO(text))
        header = next(reader)
        sv_idx = header.index("schema_version")
        unit_idx = header.index("unit_id")
        body = list(reader)
        rows_by_unit = {r[unit_idx]: r for r in body}
        self.assertEqual(rows_by_unit["lifetrac-001"][sv_idx], "1")
        self.assertEqual(rows_by_unit["lifetrac-002"][sv_idx], "")


class BC14B_D_EmitterSourceContainsSchemaVersion(unittest.TestCase):
    """Source-level grep so a future refactor that silently drops the
    field fails this gate even if the test happens to mock around it."""

    def test_web_ui_source_contains_schema_version_assignment(self) -> None:
        text = (_BS / "web_ui.py").read_text(encoding="utf-8")
        self.assertIn("schema_version=BUILD.schema_version", text)

    def test_lora_bridge_source_contains_schema_version_assignment(self) -> None:
        text = (_BS / "lora_bridge.py").read_text(encoding="utf-8")
        self.assertIn("schema_version=cfg.schema_version", text)


if __name__ == "__main__":
    unittest.main()
