"""SIL gate for ``DESIGN-CONTROLLER/BUILD_CONFIG.md`` (Round 35 / BC-08).

Pins that the operator-onboarding doc stays in sync with the rest of
the build-config system. Mechanical, no-bench-hardware checks only:

* Every required schema section is named in the doc.
* Every CLI subcommand that ``lifetrac_config.py`` registers is named
  in the doc, and vice-versa (no orphans either way).
* Every ``reload_class`` value the schema uses is named in the doc.
* Every relative-path link the doc declares resolves to an existing
  file inside this repository.

The point of this gate is the same as every other BC-* doc gate: the
doc cannot quietly drift from the contract it documents.

Test classes use the BC08_* prefix so the IP traceability matrix in
``MASTER_TEST_PROGRAM.md`` $5 maps cleanly back to BC-08.
"""

from __future__ import annotations

import json
import re
import unittest
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
_DC = _BS.parent
_REPO = _DC.parent
_DOC = _DC / "BUILD_CONFIG.md"
_SCHEMA = _BS / "config" / "build_config.schema.json"
_CLI = _REPO / "tools" / "lifetrac_config.py"


def _read_doc() -> str:
    return _DOC.read_text(encoding="utf-8")


def _read_schema() -> dict:
    return json.loads(_SCHEMA.read_text(encoding="utf-8"))


def _cli_subcommands() -> set[str]:
    """Scrape ``sub.add_parser("<name>"...)`` calls from the CLI module."""
    text = _CLI.read_text(encoding="utf-8")
    return set(re.findall(r'sub\.add_parser\(\s*"([a-z\-]+)"', text))


def _doc_links(doc_text: str) -> list[tuple[str, str]]:
    """Return [(label, target)] for every Markdown link in the doc."""
    return re.findall(r"\[([^\]]+)\]\(([^)]+)\)", doc_text)


class BC08_A_DocCoversEverySchemaSection(unittest.TestCase):
    def test_every_required_section_is_named(self) -> None:
        doc = _read_doc()
        schema = _read_schema()
        # The required list contains scalar leaves (unit_id, schema_version)
        # AND object sections; we only assert the object sections appear,
        # because the doc names scalars in the prose anyway.
        section_names = [
            name
            for name in schema["required"]
            if schema["properties"].get(name, {}).get("type") == "object"
        ]
        self.assertGreaterEqual(len(section_names), 8, section_names)
        for name in section_names:
            with self.subTest(section=name):
                # Doc must reference the section by name in backticks
                # somewhere (e.g. ``hydraulic`` or ``[hydraulic]``).
                self.assertRegex(
                    doc,
                    rf"`{re.escape(name)}`|\[{re.escape(name)}\]",
                    f"BUILD_CONFIG.md does not name schema section {name!r}",
                )


class BC08_B_DocCoversEveryCLISubcommand(unittest.TestCase):
    def test_doc_lists_every_subcommand_and_no_orphans(self) -> None:
        doc = _read_doc()
        registered = _cli_subcommands()
        # Sanity: we know there are at least seven today.
        self.assertGreaterEqual(
            len(registered), 7, f"unexpected subcommand count: {registered}"
        )
        # Doc must name each registered subcommand in backticks.
        for name in sorted(registered):
            with self.subTest(subcommand=name):
                self.assertIn(
                    f"`{name}`",
                    doc,
                    f"BUILD_CONFIG.md does not mention CLI subcommand {name!r}",
                )
        # And the doc must not invent subcommands the CLI does not register.
        # Scrape every backticked token in the CLI section that looks like a
        # subcommand reference; we use the synopsis table block as the scope.
        synopsis_match = re.search(
            r"\| Subcommand \|.*?\n\n", doc, flags=re.DOTALL
        )
        self.assertIsNotNone(synopsis_match, "CLI synopsis table missing")
        synopsis = synopsis_match.group(0)
        synopsis_tokens = set(re.findall(r"`([a-z\-]+)`", synopsis))
        # Filter out flag tokens (e.g. ``--check``) and lone hyphens.
        # Subcommand names never start with ``-``.
        synopsis_tokens = {t for t in synopsis_tokens if not t.startswith("-")}
        for name in synopsis_tokens:
            with self.subTest(token=name):
                self.assertIn(
                    name,
                    registered,
                    f"BUILD_CONFIG.md mentions unknown subcommand {name!r}",
                )


class BC08_C_DocCoversEveryReloadClass(unittest.TestCase):
    def test_every_reload_class_value_is_named(self) -> None:
        doc = _read_doc()
        schema = _read_schema()

        def walk(node: object) -> set[str]:
            classes: set[str] = set()
            if isinstance(node, dict):
                if isinstance(node.get("reload_class"), str):
                    classes.add(node["reload_class"])
                for value in node.values():
                    classes |= walk(value)
            elif isinstance(node, list):
                for value in node:
                    classes |= walk(value)
            return classes

        classes = walk(schema)
        self.assertEqual(
            classes,
            {"live", "restart_required", "firmware_required"},
            f"unexpected reload_class vocabulary: {classes}",
        )
        for name in classes:
            with self.subTest(reload_class=name):
                self.assertIn(
                    f"`{name}`",
                    doc,
                    f"BUILD_CONFIG.md does not name reload_class {name!r}",
                )


class BC08_D_DocLinksResolve(unittest.TestCase):
    def test_every_relative_link_target_exists(self) -> None:
        doc = _read_doc()
        for label, target in _doc_links(doc):
            # Skip absolute URLs and intra-page anchors.
            if target.startswith(("http://", "https://", "#", "mailto:")):
                continue
            # Strip any trailing #anchor before resolving.
            path_part = target.split("#", 1)[0]
            if not path_part:
                continue
            resolved = (_DOC.parent / path_part).resolve()
            with self.subTest(label=label, target=target):
                self.assertTrue(
                    resolved.exists(),
                    f"BUILD_CONFIG.md link {label!r} -> {target!r} "
                    f"resolves to non-existent path {resolved}",
                )


class BC08_E_DocNamesEveryEstopTopology(unittest.TestCase):
    """Spot-check: every safety-critical enum value appears in the doc.

    This is a representative drift gate — the doc explicitly calls out
    that ``estop_topology`` has multiple values, so a new topology
    (e.g. a triplicated PSR option) must be reflected here too.
    """

    def test_every_estop_topology_enum_value(self) -> None:
        doc = _read_doc()
        schema = _read_schema()
        topology = schema["properties"]["safety"]["properties"][
            "estop_topology"
        ]
        values = topology["enum"]
        self.assertGreaterEqual(len(values), 3, values)
        # Doc need only mention the *canonical* value verbatim because
        # the prose in $3 names it. The remaining values must appear at
        # least somewhere (we accept either a backtick token or a plain
        # mention within the safety prose).
        for value in values:
            with self.subTest(estop_topology=value):
                self.assertIn(
                    value,
                    doc,
                    f"BUILD_CONFIG.md does not mention estop_topology={value!r}",
                )


if __name__ == "__main__":
    unittest.main()
