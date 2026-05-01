"""Round 34 / BC-06 SIL gate: hil/dispatch.ps1 applicability rules.

Pins the contract that the JSON rules file shared between
``DESIGN-CONTROLLER/hil/dispatch.ps1`` and this test produces correct
N/A flags for representative fleet shapes, so the dispatcher never
recommends a Wave-4 gate the active build cannot exercise.

The SIL gate validates the *rules*, not the PowerShell that consumes
them: the dispatcher's PowerShell predicate evaluator is a thin
mechanical translation of the same conjunction/op vocabulary the
rules file declares (``eq`` / ``gt`` / ``gte`` / ``truthy``), so
covering the rules in Python and the dispatcher's vocabulary in
isolation gives full applicability coverage without subprocessing
``powershell.exe`` from CI.

Test classes:

* BC06_A_RulesFileShapeAndCoverage: every gate that ``dispatch.ps1``
  declares in ``$Script:GateTargets`` has a row in
  ``gate_applicability.json``, and vice versa; every predicate uses a
  known op; every dotted path resolves in the canonical default config
  (catches typos like ``hyrdaulic.proportional_flow``).
* BC06_B_CanonicalDefaultIsFullyApplicable: with the canonical default
  TOML every Wave-4 gate is applicable (the canonical fleet exercises
  the full Wave-4 suite by definition).
* BC06_C_NoHandheldMarksRadioGatesNA: with ``handheld_present = false``
  W4-01 and W4-02 are N/A and the rest stay applicable.
* BC06_D_NoCamerasMarksW408NA: with ``cameras.count = 0`` and every
  per-camera ``*_present = false`` W4-08 is N/A.
* BC06_E_BangBangMarksW405NA: with ``proportional_flow = false`` W4-05
  is N/A.
* BC06_F_NoArmAxisMarksW406NA: with ``arm_axis_count = 0`` W4-06 is
  N/A (the mixed-mode gate has nothing to mix into).
* BC06_G_DumpJsonSubcommandStable: ``lifetrac-config dump-json`` round
  trips through ``json.loads`` and contains the four documented top
  level keys; the JSON is byte-stable across two invocations against
  the same TOML.
"""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
_REPO = _BS.parents[1]
if str(_BS) not in sys.path:
    sys.path.insert(0, str(_BS))

import build_config as _bc  # type: ignore[import-not-found]

_DEFAULT_TOML = _BS / "config" / "build.default.toml"
_RULES_PATH = _REPO / "DESIGN-CONTROLLER" / "hil" / "gate_applicability.json"
_DISPATCH_PS1 = _REPO / "DESIGN-CONTROLLER" / "hil" / "dispatch.ps1"
_LIFETRAC_TOOL = _REPO / "tools" / "lifetrac_config.py"

_KNOWN_OPS = {"eq", "gt", "gte", "truthy"}


def _load_rules() -> dict:
    return json.loads(_RULES_PATH.read_text(encoding="utf-8"))


def _dispatch_gate_ids() -> set[str]:
    """Scrape the Wave-4 gate IDs declared in dispatch.ps1's GateTargets table."""
    text = _DISPATCH_PS1.read_text(encoding="utf-8")
    import re
    # Match lines like:    'W4-pre' = @{ Target = ... or 'W4-01' = @{ Target = ...
    return set(re.findall(r"'(W4-(?:pre|\d{2}))'\s*=\s*@\{", text))


def _resolve_dotted(payload: dict, path: str):
    node = payload
    for seg in path.split("."):
        if not isinstance(node, dict) or seg not in node:
            return _Sentinel
        node = node[seg]
    return node


class _SentinelType:
    def __repr__(self) -> str:  # pragma: no cover -- debug only
        return "<UNRESOLVED>"


_Sentinel = _SentinelType()


def _evaluate_predicate(value, op: str, ref) -> bool:
    if op == "eq":
        return value == ref
    if op == "gt":
        return value is not _Sentinel and value > ref
    if op == "gte":
        return value is not _Sentinel and value >= ref
    if op == "truthy":
        return bool(value) and value is not _Sentinel
    raise AssertionError(f"unknown op {op!r}")


def _gate_applies(rules: dict, gate_id: str, raw: dict) -> bool:
    gate = rules["gates"].get(gate_id)
    if gate is None:
        return True
    preds = gate.get("applies_when") or []
    for pred in preds:
        value = _resolve_dotted(raw, pred["path"])
        if not _evaluate_predicate(value, pred["op"], pred.get("value")):
            return False
    return True


def _materialise(tmp: Path, replacements: list[tuple[str, str]]) -> Path:
    """Write a TOML fixture under ``tmp`` from the canonical default with
    each (anchor, replacement) substitution applied. Each substitution
    is guarded by a no-op tripwire (recurring lesson from prior rounds)."""
    body = _DEFAULT_TOML.read_text(encoding="utf-8")
    for old, new in replacements:
        if old not in body:
            raise AssertionError(f"anchor not found in canonical default: {old!r}")
        new_body = body.replace(old, new)
        if new_body == body:
            raise AssertionError(f"no-op replacement (alignment drift?): {old!r}")
        body = new_body
    out = tmp / "build.fixture.toml"
    out.write_text(body, encoding="utf-8")
    return out


def _load_fixture(replacements: list[tuple[str, str]]):
    """Load a synthetic TOML through the validator and return ``cfg.raw``."""
    with tempfile.TemporaryDirectory() as td:
        path = _materialise(Path(td), replacements)
        import os
        os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(path)
        try:
            cfg = _bc.load()
        finally:
            os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        return cfg.raw


# ============================================================ BC06_A


class BC06_A_RulesFileShapeAndCoverage(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        cls.rules = _load_rules()
        cls.dispatch_ids = _dispatch_gate_ids()
        cls.canonical_raw = _bc.load().raw

    def test_dispatcher_and_rules_cover_the_same_gates(self) -> None:
        self.assertEqual(
            set(self.rules["gates"].keys()),
            self.dispatch_ids,
            msg="gate_applicability.json must cover exactly the gates dispatch.ps1 declares",
        )

    def test_every_predicate_uses_known_op(self) -> None:
        for gate_id, gate in self.rules["gates"].items():
            for pred in gate.get("applies_when") or []:
                with self.subTest(gate=gate_id, op=pred.get("op")):
                    self.assertIn(pred.get("op"), _KNOWN_OPS)

    def test_every_dotted_path_resolves_against_canonical_default(self) -> None:
        for gate_id, gate in self.rules["gates"].items():
            for pred in gate.get("applies_when") or []:
                with self.subTest(gate=gate_id, path=pred["path"]):
                    value = _resolve_dotted(self.canonical_raw, pred["path"])
                    self.assertIsNot(
                        value, _Sentinel,
                        msg=f"{gate_id}: predicate path {pred['path']!r} does not exist in build_config",
                    )

    def test_every_gate_carries_a_rationale_string(self) -> None:
        # Catches PRs that add a gate rule without a comment for the next reader.
        for gate_id, gate in self.rules["gates"].items():
            with self.subTest(gate=gate_id):
                self.assertIn("rationale", gate, msg=f"{gate_id}: missing rationale")
                self.assertIsInstance(gate["rationale"], str)
                self.assertGreater(len(gate["rationale"]), 10)


# ============================================================ BC06_B


class BC06_B_CanonicalDefaultIsFullyApplicable(unittest.TestCase):

    def test_every_gate_applies_to_canonical_fleet_shape(self) -> None:
        rules = _load_rules()
        raw = _bc.load().raw
        for gate_id in rules["gates"].keys():
            with self.subTest(gate=gate_id):
                self.assertTrue(
                    _gate_applies(rules, gate_id, raw),
                    msg=f"{gate_id}: must apply to the canonical default fleet shape",
                )


# ============================================================ BC06_C


class BC06_C_NoHandheldMarksRadioGatesNA(unittest.TestCase):

    def setUp(self) -> None:
        self.rules = _load_rules()
        self.raw = _load_fixture([("handheld_present         = true", "handheld_present         = false")])

    def test_w401_handheld_estop_is_na(self) -> None:
        self.assertFalse(_gate_applies(self.rules, "W4-01", self.raw))

    def test_w402_link_tune_is_na(self) -> None:
        self.assertFalse(_gate_applies(self.rules, "W4-02", self.raw))

    def test_remaining_gates_still_apply(self) -> None:
        for gate_id in ("W4-pre", "W4-00", "W4-03", "W4-04", "W4-05", "W4-06", "W4-07", "W4-08", "W4-09", "W4-10"):
            with self.subTest(gate=gate_id):
                self.assertTrue(_gate_applies(self.rules, gate_id, self.raw))


# ============================================================ BC06_D


class BC06_D_NoCamerasMarksW408NA(unittest.TestCase):

    def test_zero_cameras_marks_w408_na(self) -> None:
        rules = _load_rules()
        raw = _load_fixture([
            ("count                = 1", "count                = 0"),
            ("front_present        = true", "front_present        = false"),
        ])
        self.assertFalse(_gate_applies(rules, "W4-08", raw))
        # Spot-check: unrelated gates remain applicable.
        self.assertTrue(_gate_applies(rules, "W4-01", raw))
        self.assertTrue(_gate_applies(rules, "W4-05", raw))


# ============================================================ BC06_E


class BC06_E_BangBangMarksW405NA(unittest.TestCase):

    def test_bang_bang_marks_w405_na(self) -> None:
        rules = _load_rules()
        raw = _load_fixture([("proportional_flow   = true", "proportional_flow   = false")])
        self.assertFalse(_gate_applies(rules, "W4-05", raw))
        # Mixed-mode still applies because arm_axis_count stays at 2.
        self.assertTrue(_gate_applies(rules, "W4-06", raw))


# ============================================================ BC06_F


class BC06_F_NoArmAxisMarksW406NA(unittest.TestCase):

    def test_zero_arm_axes_marks_w406_na(self) -> None:
        rules = _load_rules()
        raw = _load_fixture([("arm_axis_count      = 2", "arm_axis_count      = 0")])
        self.assertFalse(_gate_applies(rules, "W4-06", raw))
        # W4-05 still applies (proportional_flow stays true).
        self.assertTrue(_gate_applies(rules, "W4-05", raw))


# ============================================================ BC06_G


class BC06_G_DumpJsonSubcommandStable(unittest.TestCase):

    def _run_dump(self) -> str:
        env = {}
        import os
        env.update(os.environ)
        proc = subprocess.run(
            [sys.executable, str(_LIFETRAC_TOOL), "dump-json", str(_DEFAULT_TOML)],
            capture_output=True, text=True, env=env, check=False,
        )
        if proc.returncode != 0:
            self.fail(f"dump-json failed (exit {proc.returncode}): {proc.stderr}")
        return proc.stdout.strip()

    def test_dump_json_is_byte_identical_across_two_runs(self) -> None:
        a = self._run_dump()
        b = self._run_dump()
        self.assertEqual(a, b, msg="dump-json must be deterministic")

    def test_dump_json_payload_carries_documented_top_level_keys(self) -> None:
        payload = json.loads(self._run_dump())
        self.assertEqual(
            sorted(payload.keys()),
            sorted(["unit_id", "schema_version", "config_sha256", "config"]),
        )
        self.assertEqual(payload["unit_id"], "lifetrac-001")
        self.assertEqual(payload["schema_version"], 1)
        self.assertEqual(len(payload["config_sha256"]), 64)
        # Sanity: the nested config has every top-level section.
        for section in ("hydraulic", "safety", "cameras", "sensors", "comm", "ui", "net", "aux"):
            self.assertIn(section, payload["config"])


if __name__ == "__main__":
    unittest.main()
