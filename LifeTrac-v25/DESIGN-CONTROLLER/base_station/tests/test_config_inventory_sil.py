"""SIL gate for ``lifetrac-config inventory`` (Round 37 / BC-14).

Pins the fleet-wide config-inventory subcommand: aggregates
``config_loaded`` audit-log events across one or more ``audit.jsonl``
files into a single CSV / Markdown table, deduplicated by
``(unit_id, config_sha256)`` so a unit that has been reflashed shows
one row per build.

The tests work entirely against synthetic ``audit.jsonl`` fixtures
written into ``tempfile`` directories \u2014 no subprocess, no real
fleet logs, no clock dependence.

Test classes use the BC14_* prefix so the IP traceability matrix in
``MASTER_TEST_PROGRAM.md`` $5 maps cleanly back to BC-14.
"""

from __future__ import annotations

import csv
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
_REPO = _BS.parents[1]
_TOOLS = _REPO / "tools"

if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

import lifetrac_config as lc  # noqa: E402  -- after sys.path tweak


def _write_jsonl(path: Path, records: list[dict]) -> None:
    with open(path, "w", encoding="utf-8", newline="\n") as fp:
        for rec in records:
            fp.write(json.dumps(rec, separators=(",", ":")) + "\n")


def _config_loaded(ts: float, *, unit_id: str, sha: str,
                   schema_version: int = 1,
                   component: str = "web_ui",
                   source_path: str = "/etc/lifetrac/build.toml") -> dict:
    return {
        "ts": ts,
        "event": "config_loaded",
        "component": component,
        "unit_id": unit_id,
        "source_path": source_path,
        "config_sha256": sha,
        "schema_version": schema_version,
    }


_SHA_A = "a" * 64
_SHA_B = "b" * 64
_SHA_C = "c" * 64


class BC14_A_ParserSkipsNonInventoryAndMalformed(unittest.TestCase):
    def test_only_config_loaded_records_pass(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            log = Path(td) / "audit.jsonl"
            _write_jsonl(log, [
                _config_loaded(1000.0, unit_id="lifetrac-001", sha=_SHA_A),
                {"ts": 1001.0, "event": "bridge_start", "port": "/dev/ttyACM0"},
                _config_loaded(1002.0, unit_id="lifetrac-002", sha=_SHA_B),
                {"ts": 1003.0, "event": "source_transition",
                 "prev": "NONE", "new": "BASE"},
            ])
            recs = list(lc.iter_config_loaded_events([log]))
            self.assertEqual(len(recs), 2)
            self.assertEqual({r["unit_id"] for r in recs},
                             {"lifetrac-001", "lifetrac-002"})

    def test_malformed_lines_are_skipped(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            log = Path(td) / "audit.jsonl"
            with open(log, "w", encoding="utf-8", newline="\n") as fp:
                fp.write(json.dumps(_config_loaded(
                    1000.0, unit_id="lifetrac-001", sha=_SHA_A,
                )) + "\n")
                fp.write("this is not json\n")
                fp.write("\n")  # empty line
                fp.write("[1, 2, 3]\n")  # not a dict
                fp.write(json.dumps(_config_loaded(
                    1001.0, unit_id="lifetrac-002", sha=_SHA_B,
                )) + "\n")
            recs = list(lc.iter_config_loaded_events([log]))
            self.assertEqual(len(recs), 2)

    def test_records_missing_required_fields_are_skipped(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            log = Path(td) / "audit.jsonl"
            with open(log, "w", encoding="utf-8", newline="\n") as fp:
                # Missing config_sha256.
                fp.write(json.dumps({
                    "ts": 1000.0,
                    "event": "config_loaded",
                    "unit_id": "lifetrac-001",
                }) + "\n")
                # Valid.
                fp.write(json.dumps(_config_loaded(
                    1001.0, unit_id="lifetrac-002", sha=_SHA_B,
                )) + "\n")
            recs = list(lc.iter_config_loaded_events([log]))
            self.assertEqual(len(recs), 1)
            self.assertEqual(recs[0]["unit_id"], "lifetrac-002")

    def test_missing_files_skipped_silently(self) -> None:
        recs = list(lc.iter_config_loaded_events([Path("/no/such/file.jsonl")]))
        self.assertEqual(recs, [])


class BC14_B_AggregationDedupesByUnitAndSha(unittest.TestCase):
    def test_one_unit_one_sha_collapses_to_one_row(self) -> None:
        records = [
            _config_loaded(1000.0, unit_id="lifetrac-001", sha=_SHA_A,
                           component="web_ui"),
            _config_loaded(1100.0, unit_id="lifetrac-001", sha=_SHA_A,
                           component="lora_bridge"),
            _config_loaded(1200.0, unit_id="lifetrac-001", sha=_SHA_A,
                           component="web_ui"),
        ]
        rows = lc.aggregate_inventory(records)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["boot_count"], 3)
        self.assertEqual(rows[0]["first_seen"], 1000.0)
        self.assertEqual(rows[0]["last_seen"], 1200.0)
        self.assertEqual(rows[0]["components"], {"web_ui", "lora_bridge"})

    def test_one_unit_two_shas_yields_two_rows(self) -> None:
        records = [
            _config_loaded(1000.0, unit_id="lifetrac-001", sha=_SHA_A),
            _config_loaded(2000.0, unit_id="lifetrac-001", sha=_SHA_B),
            _config_loaded(2100.0, unit_id="lifetrac-001", sha=_SHA_B),
        ]
        rows = lc.aggregate_inventory(records)
        self.assertEqual(len(rows), 2)
        # Sorted: same unit_id, then last_seen descending \u2014 SHA_B first.
        self.assertEqual(rows[0]["config_sha256"], _SHA_B)
        self.assertEqual(rows[0]["boot_count"], 2)
        self.assertEqual(rows[1]["config_sha256"], _SHA_A)
        self.assertEqual(rows[1]["boot_count"], 1)

    def test_two_units_yields_two_rows_sorted_by_unit_id(self) -> None:
        records = [
            _config_loaded(1000.0, unit_id="lifetrac-002", sha=_SHA_B),
            _config_loaded(1100.0, unit_id="lifetrac-001", sha=_SHA_A),
        ]
        rows = lc.aggregate_inventory(records)
        self.assertEqual([r["unit_id"] for r in rows],
                         ["lifetrac-001", "lifetrac-002"])


class BC14_C_RenderingIsStable(unittest.TestCase):
    def test_csv_header_matches_inventory_fields(self) -> None:
        rows = lc.aggregate_inventory([
            _config_loaded(1000.0, unit_id="lifetrac-001", sha=_SHA_A),
        ])
        text = lc.render_inventory_csv(rows)
        reader = csv.reader(io.StringIO(text))
        header = next(reader)
        self.assertEqual(tuple(header), lc.INVENTORY_FIELDS)
        body = next(reader)
        self.assertEqual(body[0], "lifetrac-001")
        self.assertEqual(body[1], _SHA_A[:16])
        self.assertEqual(body[5], "1")  # boot_count

    def test_csv_is_byte_identical_across_two_runs(self) -> None:
        records = [
            _config_loaded(1000.0, unit_id="lifetrac-001", sha=_SHA_A),
            _config_loaded(2000.0, unit_id="lifetrac-002", sha=_SHA_B),
            _config_loaded(2100.0, unit_id="lifetrac-001", sha=_SHA_C),
        ]
        a = lc.render_inventory_csv(lc.aggregate_inventory(records))
        b = lc.render_inventory_csv(lc.aggregate_inventory(list(records)))
        self.assertEqual(a, b)

    def test_markdown_table_has_header_and_separator(self) -> None:
        rows = lc.aggregate_inventory([
            _config_loaded(1000.0, unit_id="lifetrac-001", sha=_SHA_A),
        ])
        text = lc.render_inventory_markdown(rows)
        lines = text.splitlines()
        self.assertGreaterEqual(len(lines), 3)
        for field in lc.INVENTORY_FIELDS:
            self.assertIn(field, lines[0])
        self.assertTrue(set(lines[1].replace("|", "")) <= {"-"})


class BC14_D_DirectoryGlobAndCommandIntegration(unittest.TestCase):
    def test_directory_argument_picks_up_rotated_siblings(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            d = Path(td)
            _write_jsonl(d / "audit.jsonl", [
                _config_loaded(1000.0, unit_id="lifetrac-001", sha=_SHA_A),
            ])
            _write_jsonl(d / "audit.jsonl.1", [
                _config_loaded(900.0, unit_id="lifetrac-002", sha=_SHA_B),
            ])
            # Stray non-audit file in the same directory must be ignored.
            (d / "boot.log").write_text("not for us\n", encoding="utf-8")
            # Run via the public command function with an argparse
            # Namespace shim, capturing stdout.
            import argparse
            ns = argparse.Namespace(paths=[str(d)], format="csv")
            buf = io.StringIO()
            old, sys.stdout = sys.stdout, buf
            try:
                rc = lc.cmd_inventory(ns)
            finally:
                sys.stdout = old
            self.assertEqual(rc, lc.EXIT_OK)
            text = buf.getvalue()
            self.assertIn("lifetrac-001", text)
            self.assertIn("lifetrac-002", text)
            # Header is exactly one line; both unit rows present \u2192 3 lines.
            self.assertEqual(len(text.strip().splitlines()), 3)

    def test_format_markdown_switches_renderer(self) -> None:
        with tempfile.TemporaryDirectory() as td:
            log = Path(td) / "audit.jsonl"
            _write_jsonl(log, [
                _config_loaded(1000.0, unit_id="lifetrac-001", sha=_SHA_A),
            ])
            import argparse
            ns = argparse.Namespace(paths=[str(log)], format="markdown")
            buf = io.StringIO()
            old, sys.stdout = sys.stdout, buf
            try:
                rc = lc.cmd_inventory(ns)
            finally:
                sys.stdout = old
            self.assertEqual(rc, lc.EXIT_OK)
            text = buf.getvalue()
            # Markdown signal: pipe-delimited header row.
            self.assertTrue(text.startswith("| unit_id |"))


class BC14_E_TimestampFormatIsISO8601UTC(unittest.TestCase):
    def test_known_timestamp_renders_to_known_iso(self) -> None:
        # 1700000000 = 2023-11-14T22:13:20Z.
        text = lc._format_ts(1700000000.0)
        self.assertEqual(text, "2023-11-14T22:13:20Z")

    def test_zero_or_negative_renders_empty(self) -> None:
        self.assertEqual(lc._format_ts(0.0), "")
        self.assertEqual(lc._format_ts(-1.0), "")


if __name__ == "__main__":
    unittest.main()
