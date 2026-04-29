"""SIL gate for HARDWARE_BOM.md capability cross-reference (Round 36 / BC-09).

Pins the ``## Capability cross-reference`` appendix in HARDWARE_BOM.md
to ground truth in two directions:

* Every capability ID listed in the cross-ref appendix is a real
  property in ``build_config.schema.json`` (no orphan IDs in the
  appendix).
* Every capability ID in the curated ``_PHYSICAL_CAPABILITIES`` set
  $em the leaves whose value determines whether a particular part
  appears in the BOM for a given build $em is present in the
  cross-ref appendix (no physical capability missing a BOM anchor).

Plus per-row evidence checks:

* Every cross-ref row carries at least one ``Tier 1`` / ``Tier 2`` /
  ``Tier 3`` token OR an explicit ``planned`` marker (so an aux-style
  unrealised capability stays self-documenting).
* Every appendix link target resolves on disk.

Test classes use the BC09_* prefix so the IP traceability matrix in
``MASTER_TEST_PROGRAM.md`` $5 maps cleanly back to BC-09.
"""

from __future__ import annotations

import json
import re
import unittest
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
_DC = _BS.parent
_BOM = _DC / "HARDWARE_BOM.md"
_SCHEMA = _BS / "config" / "build_config.schema.json"

# Curated set of *physical-shape* capability leaves: those whose value
# materially changes which parts are ordered for a build. Pure software
# / tunable parameters (ramps, thresholds, UI flags, MQTT host) are
# deliberately excluded $em they do not need BOM rows.
#
# Adding a new capability with a physical realisation: append it here,
# then add a matching row in HARDWARE_BOM.md's ``Capability
# cross-reference`` appendix. The BC09_B test will fail loudly until
# both edits land.
_PHYSICAL_CAPABILITIES: frozenset[str] = frozenset(
    {
        "hydraulic.proportional_flow",
        "safety.estop_topology",
        "cameras.count",
        "cameras.coral_tpu",
        "sensors.imu_present",
        "sensors.imu_model",
        "sensors.gps_present",
        "sensors.gps_model",
        "sensors.hyd_pressure_sensor_count",
        "comm.lora_region",
        "comm.cellular_backup_present",
        "comm.handheld_present",
        "aux.port_count",
        "aux.coupler_type",
        "aux.case_drain_present",
    }
)

_XREF_HEADING = "## Capability cross-reference"


def _read_bom() -> str:
    return _BOM.read_text(encoding="utf-8")


def _xref_block(bom_text: str) -> str:
    """Return the appendix block from the ``Capability cross-reference``
    heading to either the next ``## `` heading or end-of-file."""
    idx = bom_text.find(_XREF_HEADING)
    if idx < 0:
        return ""
    rest = bom_text[idx + len(_XREF_HEADING):]
    next_h = re.search(r"\n## ", rest)
    if next_h:
        return rest[: next_h.start()]
    return rest


_ROW_RE = re.compile(r"^\|\s*``([a-z_.]+)``\s*\|(.+?)\|(.+?)\|\s*$", re.MULTILINE)


def _xref_rows(block: str) -> list[tuple[str, str, str]]:
    """Return [(capability_id, evidence_cell, notes_cell)] from the table."""
    return [(m.group(1), m.group(2), m.group(3)) for m in _ROW_RE.finditer(block)]


def _schema_property_paths() -> set[str]:
    """Return every dotted leaf path that exists in the schema's ``properties``."""
    schema = json.loads(_SCHEMA.read_text(encoding="utf-8"))
    paths: set[str] = set()

    def walk(node: dict, prefix: str) -> None:
        props = node.get("properties", {})
        if not isinstance(props, dict):
            return
        for name, child in props.items():
            full = f"{prefix}.{name}" if prefix else name
            paths.add(full)
            if isinstance(child, dict) and child.get("type") == "object":
                walk(child, full)

    walk(schema, "")
    return paths


class BC09_A_NoOrphanIDsInAppendix(unittest.TestCase):
    def test_every_appendix_id_exists_in_schema(self) -> None:
        block = _xref_block(_read_bom())
        rows = _xref_rows(block)
        self.assertGreater(len(rows), 0, "cross-ref appendix is empty")
        schema_paths = _schema_property_paths()
        for cap_id, _evidence, _notes in rows:
            with self.subTest(capability_id=cap_id):
                self.assertIn(
                    cap_id,
                    schema_paths,
                    f"appendix lists unknown capability id {cap_id!r}",
                )


class BC09_B_EveryPhysicalCapabilityHasBOMRow(unittest.TestCase):
    def test_every_physical_capability_appears(self) -> None:
        block = _xref_block(_read_bom())
        rows = _xref_rows(block)
        listed = {cap_id for cap_id, _e, _n in rows}
        missing = _PHYSICAL_CAPABILITIES - listed
        self.assertEqual(
            missing,
            set(),
            f"physical capabilities missing from HARDWARE_BOM.md xref: {sorted(missing)}",
        )

    def test_curated_set_matches_schema(self) -> None:
        # Every entry in the curated _PHYSICAL_CAPABILITIES set must exist
        # in the schema $em prevents the curated list from drifting away
        # from reality if a capability is renamed.
        schema_paths = _schema_property_paths()
        for cap in sorted(_PHYSICAL_CAPABILITIES):
            with self.subTest(capability=cap):
                self.assertIn(cap, schema_paths, f"curated cap {cap!r} not in schema")


class BC09_C_RowEvidenceIsAnchored(unittest.TestCase):
    def test_every_row_carries_tier_token_or_planned_marker(self) -> None:
        block = _xref_block(_read_bom())
        rows = _xref_rows(block)
        token_re = re.compile(r"Tier [123]|planned", re.IGNORECASE)
        for cap_id, evidence, _notes in rows:
            with self.subTest(capability_id=cap_id):
                self.assertRegex(
                    evidence,
                    token_re,
                    f"row {cap_id!r} evidence cell lacks Tier/planned anchor: {evidence!r}",
                )


class BC09_D_AppendixLinksResolve(unittest.TestCase):
    def test_every_relative_link_in_appendix_resolves(self) -> None:
        block = _xref_block(_read_bom())
        for label, target in re.findall(r"\[([^\]]+)\]\(([^)]+)\)", block):
            if target.startswith(("http://", "https://", "#", "mailto:")):
                continue
            path_part = target.split("#", 1)[0]
            if not path_part:
                continue
            resolved = (_BOM.parent / path_part).resolve()
            with self.subTest(label=label, target=target):
                self.assertTrue(
                    resolved.exists(),
                    f"appendix link {label!r} -> {target!r} resolves to "
                    f"non-existent path {resolved}",
                )


class BC09_E_AppendixIsDiscoverable(unittest.TestCase):
    def test_heading_present_and_test_filename_referenced(self) -> None:
        bom = _read_bom()
        self.assertIn(_XREF_HEADING, bom, "Capability cross-reference heading missing")
        # Self-reference: appendix points at this very test file so the
        # next reader can find the contract that pins it.
        self.assertIn(
            "test_hardware_bom_xref_sil.py",
            bom,
            "appendix does not name its enforcing SIL test",
        )


if __name__ == "__main__":
    unittest.main()
