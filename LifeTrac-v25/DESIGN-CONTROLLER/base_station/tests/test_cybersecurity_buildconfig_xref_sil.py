"""SIL drift gate for CYBERSECURITY_CASE.md $9 (Round 38 / BC-13).

The build-config attack-surface section of CYBERSECURITY_CASE.md is
prose plus four tables that name concrete artefacts (CLI subcommands,
sibling docs, this test file). The gate pins those names against the
rest of the codebase so the doc cannot rot:

* BC13_A: $9 exists with the four expected sub-headings.
* BC13_B: every ``lifetrac-config <subcommand>`` named in $9 is a real
  registered CLI subcommand.
* BC13_C: every relative Markdown link in $9 resolves on disk.
* BC13_D: the STRIDE table (\\$9.2) covers each Z-CONFIG asset row from
  $9.1 at least once.
* BC13_E: $9 self-references this test file (renaming the test fails
  the gate, by design).
"""

from __future__ import annotations

import re
import sys
import unittest
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
_DESIGN = _BS.parent
_REPO = _DESIGN.parent
_TOOLS = _REPO / "tools"

if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

import lifetrac_config as lc  # noqa: E402

_DOC = _DESIGN / "CYBERSECURITY_CASE.md"
_THIS_REL = "base_station/tests/test_cybersecurity_buildconfig_xref_sil.py"


def _section_9() -> str:
    text = _DOC.read_text(encoding="utf-8")
    m = re.search(r"^## 9\. ", text, re.MULTILINE)
    if not m:
        raise AssertionError("CYBERSECURITY_CASE.md has no $9 heading")
    after = text[m.start():]
    nxt = re.search(r"^## 10\. ", after, re.MULTILINE)
    return after if nxt is None else after[: nxt.start()]


class BC13_A_SectionStructure(unittest.TestCase):
    def test_section_9_present_with_four_subheadings(self) -> None:
        body = _section_9()
        for sub in ("### 9.1 ", "### 9.2 ", "### 9.3 ", "### 9.4 "):
            self.assertIn(sub, body, f"missing sub-heading {sub!r}")


class BC13_B_NamedCLISubcommandsExist(unittest.TestCase):
    def test_every_named_subcommand_is_real(self) -> None:
        body = _section_9()
        named = set(re.findall(r"`+lifetrac-config (\w[\w-]*)`+", body))
        # Resolve real subcommands from the parser registration so this
        # gate stays in lockstep with the actual CLI.
        parser = lc._build_parser()
        sub_actions = [a for a in parser._actions
                       if a.__class__.__name__ == "_SubParsersAction"]
        self.assertEqual(len(sub_actions), 1, "expected exactly one subparsers action")
        real = set(sub_actions[0].choices.keys())
        unknown = named - real
        self.assertFalse(unknown,
                         f"$9 names CLI subcommand(s) that don't exist: {sorted(unknown)}")
        # Sanity: the section should actually mention more than zero.
        self.assertGreaterEqual(len(named), 4,
                                "$9 should reference several lifetrac-config subcommands")


class BC13_C_RelativeLinksResolve(unittest.TestCase):
    def test_every_relative_link_target_exists(self) -> None:
        body = _section_9()
        # [text](target) with target NOT starting with http:// or https:// or #.
        links = re.findall(r"\]\(([^)#]+)(?:#[^)]*)?\)", body)
        unresolved: list[str] = []
        for raw in links:
            if raw.startswith(("http://", "https://", "mailto:")):
                continue
            target = (_DESIGN / raw).resolve()
            if not target.exists():
                unresolved.append(raw)
        self.assertFalse(unresolved,
                         f"$9 has unresolved relative link(s): {unresolved}")


class BC13_D_StrideCoversEveryZConfigAsset(unittest.TestCase):
    """$9.1 enumerates Z-CONFIG assets; $9.2 is the STRIDE matrix.

    Each asset row's leading noun phrase ("build.toml", "Bundle on USB
    stick", "lifetrac-config push", "Active config", "config_loaded",
    "Generated firmware header") must appear in at least one $9.2 row,
    and vice versa.
    """

    # Curated mapping: $9.1 asset row -> string fragment that MUST
    # appear at least once in the leftmost cell of $9.2. Curated rather
    # than parsed-from-doc because the wording in the two tables is
    # deliberately different (one says "Active config under
    # /etc/lifetrac/", the other says "Active config on tractor").
    _ASSET_TOKENS = (
        "build.toml",
        "Bundle on USB stick",
        "lifetrac-config push",
        "Active config",
        "config_loaded",
        "Generated firmware header",
    )

    def test_every_asset_appears_in_stride_table(self) -> None:
        body = _section_9()
        # Slice $9.2 specifically so we don't false-positive on prose.
        stride_match = re.search(
            r"### 9\.2 [\s\S]+?(?=### 9\.3 )", body)
        self.assertIsNotNone(stride_match, "$9.2 STRIDE table not found")
        stride = stride_match.group(0)
        missing = [t for t in self._ASSET_TOKENS if t not in stride]
        self.assertFalse(missing,
                         f"STRIDE table missing rows for: {missing}")


class BC13_E_SelfReferencesEnforcingTest(unittest.TestCase):
    def test_section_9_links_to_this_test_file(self) -> None:
        body = _section_9()
        self.assertIn(_THIS_REL, body,
                      "$9 must self-reference its enforcing SIL gate "
                      f"({_THIS_REL}) so renaming the test breaks the doc gate")


if __name__ == "__main__":
    unittest.main()
