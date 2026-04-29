"""W4-XX HIL harness completeness SIL test.

Round 26 -- IP: HIL-01 (HIL harness completeness).

Pins the contract that DESIGN-CONTROLLER/hil/ provides exactly one harness
script per Wave-4 gate enumerated in MASTER_TEST_PROGRAM.md section 4, plus
the supporting infrastructure (README, schema, common helper, dispatcher).
This is a pure source-parse SIL test -- it does not invoke PowerShell.

Defended invariants
-------------------

HC-A (file existence): every W4-XX row in MASTER_TEST_PROGRAM.md section 4
    has a matching ``DESIGN-CONTROLLER/hil/w4-NN_*.ps1`` harness file, and
    the supporting files (README.md, results_schema.json, _common.ps1,
    dispatch.ps1) are all present.

HC-B (harness contract): every harness script
    * dot-sources ``_common.ps1``
    * declares ``[CmdletBinding()]`` + a mandatory ``-Operator`` parameter
    * calls ``Write-GateHeader`` and ``Assert-Section0-Ready``
    * calls ``Write-HilResult`` to log a JSONL line
    * mentions its own gate id (W4-NN) in the source
    * carries a reference back to ``HIL_RUNBOOK.md``

HC-C (schema invariants): ``results_schema.json`` is well-formed JSON
    Schema, restricts ``gate_id`` to ``W4-01..W4-10``, and ``result`` to the
    four-value enum the harnesses + dispatcher rely on.

HC-D (dispatcher coverage): ``dispatch.ps1`` enumerates exactly the same
    set of W4-XX gate ids as the harness directory (no orphans either way)
    and pairs each with a positive integer ``Target`` row count.
"""

from __future__ import annotations

import json
import re
import unittest
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
REPO_ROOT = _BS.parents[1]            # LifeTrac-v25/
HIL_DIR = REPO_ROOT / "DESIGN-CONTROLLER" / "hil"
RUNBOOK = REPO_ROOT / "DESIGN-CONTROLLER" / "HIL_RUNBOOK.md"
MASTER = REPO_ROOT / "MASTER_TEST_PROGRAM.md"

EXPECTED_GATES = [f"W4-{n:02d}" for n in range(1, 11)]


def _gate_to_filename_prefix(gate_id: str) -> str:
    # 'W4-01' -> 'w4-01_'
    return gate_id.lower() + "_"


def _harness_path(gate_id: str) -> Path | None:
    pattern = _gate_to_filename_prefix(gate_id) + "*.ps1"
    matches = sorted(HIL_DIR.glob(pattern))
    return matches[0] if matches else None


class HC_A_FileExistence(unittest.TestCase):
    """Every gate has a matching harness; supporting files exist."""

    def test_hil_directory_exists(self) -> None:
        self.assertTrue(HIL_DIR.is_dir(), f"missing HIL directory: {HIL_DIR}")

    def test_supporting_files_present(self) -> None:
        for name in ("README.md", "results_schema.json", "_common.ps1", "dispatch.ps1"):
            with self.subTest(file=name):
                self.assertTrue((HIL_DIR / name).is_file(), f"missing {name}")

    def test_one_harness_per_gate(self) -> None:
        for gid in EXPECTED_GATES:
            with self.subTest(gate=gid):
                path = _harness_path(gid)
                self.assertIsNotNone(path, f"no harness file matching {gid.lower()}_*.ps1")

    def test_no_orphan_harnesses(self) -> None:
        # Every w4-NN_*.ps1 in the directory must map back to an EXPECTED_GATE.
        actual = set()
        for p in HIL_DIR.glob("w4-*_*.ps1"):
            m = re.match(r"^(w4-\d{2})_", p.name)
            self.assertIsNotNone(m, f"bad harness filename: {p.name}")
            actual.add(m.group(1).upper())
        expected = set(EXPECTED_GATES)
        self.assertEqual(
            actual, expected,
            f"orphan or missing harnesses; only={actual ^ expected}",
        )

    def test_master_test_program_lists_all_expected_gates(self) -> None:
        # Defends against drift: if MASTER_TEST_PROGRAM.md grows or shrinks
        # the W4-XX matrix, this test must be updated in lockstep with
        # EXPECTED_GATES above.
        src = MASTER.read_text(encoding="utf-8")
        for gid in EXPECTED_GATES:
            with self.subTest(gate=gid):
                self.assertIn(f"**{gid}**", src, f"{gid} not bold-listed in MASTER_TEST_PROGRAM.md")


class HC_B_HarnessContract(unittest.TestCase):
    """Every harness script honours the documented contract."""

    REQUIRED_FRAGMENTS = (
        ('. "$PSScriptRoot/_common.ps1"', "must dot-source _common.ps1"),
        ("[CmdletBinding()]",            "must declare [CmdletBinding()]"),
        ("[Parameter(Mandatory)][string]$Operator", "must take a mandatory -Operator string"),
        ("Write-GateHeader",             "must call Write-GateHeader"),
        ("Assert-Section0-Ready",        "must call Assert-Section0-Ready"),
        ("Write-HilResult",              "must call Write-HilResult"),
        ("HIL_RUNBOOK.md",               "must reference HIL_RUNBOOK.md"),
    )

    def test_each_harness_honours_contract(self) -> None:
        for gid in EXPECTED_GATES:
            path = _harness_path(gid)
            self.assertIsNotNone(path)
            src = path.read_text(encoding="utf-8")
            for fragment, why in self.REQUIRED_FRAGMENTS:
                with self.subTest(gate=gid, fragment=fragment):
                    self.assertIn(fragment, src, f"{path.name}: {why}")
            with self.subTest(gate=gid, check="self-id"):
                self.assertIn(gid, src, f"{path.name}: must mention own gate id {gid}")


class HC_C_SchemaInvariants(unittest.TestCase):
    """results_schema.json pins the JSONL contract the harnesses + dispatcher rely on."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.schema = json.loads((HIL_DIR / "results_schema.json").read_text(encoding="utf-8"))

    def test_schema_is_object_with_required_fields(self) -> None:
        self.assertEqual(self.schema.get("type"), "object")
        required = set(self.schema.get("required", []))
        self.assertTrue(
            {"gate_id", "run_id", "timestamp", "operator", "firmware_sha", "result"}.issubset(required),
            f"missing required fields: {required}",
        )

    def test_gate_id_pattern_matches_only_W4_01_through_W4_10(self) -> None:
        pat = self.schema["properties"]["gate_id"]["pattern"]
        regex = re.compile(pat)
        for gid in EXPECTED_GATES:
            with self.subTest(gate=gid):
                self.assertIsNotNone(regex.match(gid))
        for bad in ("W4-00", "W4-11", "W3-01", "w4-01", ""):
            with self.subTest(bad=bad):
                self.assertIsNone(regex.match(bad))

    def test_result_enum_is_pass_fail_skip_abort(self) -> None:
        self.assertEqual(
            sorted(self.schema["properties"]["result"]["enum"]),
            ["ABORT", "FAIL", "PASS", "SKIP"],
        )

    def test_firmware_sha_bundle_required_keys(self) -> None:
        fw = self.schema["properties"]["firmware_sha"]
        self.assertEqual(
            sorted(fw["required"]),
            ["base", "handheld", "opta", "tractor_h7", "tractor_m4"],
        )


class HC_D_DispatcherCoverage(unittest.TestCase):
    """dispatch.ps1 enumerates exactly the W4-XX gate set with positive targets."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.src = (HIL_DIR / "dispatch.ps1").read_text(encoding="utf-8")

    def test_each_gate_has_a_target_entry(self) -> None:
        # Match `'W4-NN' = @{ Target = N; Title = '...' }` per gate.
        for gid in EXPECTED_GATES:
            pat = re.compile(
                r"'" + re.escape(gid) + r"'\s*=\s*@\{\s*Target\s*=\s*(\d+)\s*;",
                re.MULTILINE,
            )
            with self.subTest(gate=gid):
                m = pat.search(self.src)
                self.assertIsNotNone(m, f"dispatch.ps1 missing target entry for {gid}")
                target = int(m.group(1))
                self.assertGreater(target, 0, f"{gid} target must be positive")

    def test_no_orphan_dispatcher_entries(self) -> None:
        found = set(re.findall(r"'(W4-\d{2})'\s*=\s*@\{\s*Target", self.src))
        self.assertEqual(found, set(EXPECTED_GATES),
                         f"dispatcher gate set does not match harness set: {found ^ set(EXPECTED_GATES)}")

    def test_dispatcher_dot_sources_common(self) -> None:
        self.assertIn('. "$PSScriptRoot/_common.ps1"', self.src)


if __name__ == "__main__":
    unittest.main()
