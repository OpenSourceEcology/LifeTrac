"""Encode-mode parity SIL: tractor ``ENCODE_MODE_*`` vs base ``EncodeMode``.

``firmware/tractor_x8/camera_service.py`` and ``base_station/lora_proto.py``
each carry their own copy of the ``CMD_ENCODE_MODE`` (0x63) value table —
the tractor deliberately does not import the base-station tree. The two
drifted once already: the tractor grew ``ENCODE_MODE_RAWSTREAM = 8`` while
``EncodeMode`` stopped at ``ADAPTIVE = 7``, so ``image_rx_daemon`` rejected
the mode before it ever reached the air and the operator could only get
RAWSTREAM through tractor environment variables (VECTOR_SCENE.md B2).

This SIL parses ``camera_service.py`` as source (importing it drags in the
camera / MQTT stack) and pins, for every mode in the tractor's
``_ENCODE_MODE_IMPLEMENTED`` set:

* the same NAME exists in ``lora_proto.EncodeMode``;
* it carries the same wire VALUE;
* ``ENCODE_MODE_NAMES[value]`` — the ``effective_name`` the tractor echoes
  in its 0x68 ack — is the lower-cased ``EncodeMode`` member name, which is
  the string web_ui compares the operator's pin against.

A fourth check reads ``web_ui.py`` the same way so the operator menu can
only ever offer modes the tractor really encodes (the 2026-07-26 btc4_*
placebo lesson) and accepts every mode it offers.
"""

from __future__ import annotations

import ast
import re
import sys
import unittest
from pathlib import Path

# Repo-relative paths.
REPO_ROOT = Path(__file__).resolve().parents[3]
BASE_STATION_DIR = REPO_ROOT / "DESIGN-CONTROLLER" / "base_station"
CAMERA_SERVICE_PY = (REPO_ROOT / "DESIGN-CONTROLLER" / "firmware"
                     / "tractor_x8" / "camera_service.py")
WEB_UI_PY = BASE_STATION_DIR / "web_ui.py"

if str(BASE_STATION_DIR) not in sys.path:
    sys.path.insert(0, str(BASE_STATION_DIR))

from lora_proto import EncodeMode  # noqa: E402

_MODE_CONST_RE = re.compile(r"^ENCODE_MODE_([A-Z0-9_]+)$")


def _module_assignments(path: Path) -> dict[str, ast.expr]:
    """Map module-level ``NAME = <expr>`` targets to their value nodes."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    out: dict[str, ast.expr] = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name):
            out[node.targets[0].id] = node.value
        elif isinstance(node, ast.AnnAssign) and node.value is not None \
                and isinstance(node.target, ast.Name):
            out[node.target.id] = node.value
    return out


def _names_in(container: ast.expr) -> list[str]:
    """Name ids inside ``frozenset({A, B})`` / ``{A, B}`` / ``(A, B)``."""
    if isinstance(container, ast.Call) and container.args:   # frozenset({...})
        container = container.args[0]
    return [elt.id for elt in getattr(container, "elts", [])
            if isinstance(elt, ast.Name)]


def _tractor_mode_table() -> tuple[dict[str, int], list[str], tuple[str, ...]]:
    """Return (constants, implemented, names) parsed from camera_service.py.

    ``constants`` maps every integer ``ENCODE_MODE_<X> = n`` at module
    scope; ``implemented`` lists the constant names inside
    ``_ENCODE_MODE_IMPLEMENTED``; ``names`` is ``ENCODE_MODE_NAMES``.
    """
    assigns = _module_assignments(CAMERA_SERVICE_PY)
    constants = {
        name: value.value for name, value in assigns.items()
        if _MODE_CONST_RE.match(name)
        and isinstance(value, ast.Constant)
        and isinstance(value.value, int) and not isinstance(value.value, bool)
    }
    implemented = _names_in(assigns.get("_ENCODE_MODE_IMPLEMENTED", ast.Tuple(elts=[])))
    names_node = assigns.get("ENCODE_MODE_NAMES")
    names = tuple(ast.literal_eval(names_node)) if names_node is not None else ()
    return constants, implemented, names


class EncodeModeParitySIL(unittest.TestCase):
    """Every mode the tractor implements must exist in ``EncodeMode``
    under the same name, with the same wire value and the same ack name."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.constants, cls.implemented, cls.names = _tractor_mode_table()

    def test_parser_found_the_tractor_table(self) -> None:
        # Guard the guard: a rename of the tractor symbols must fail here
        # loudly rather than let the parity tests pass over an empty set.
        self.assertGreaterEqual(
            len(self.implemented), 2,
            "camera_service._ENCODE_MODE_IMPLEMENTED not found or empty")
        self.assertTrue(self.names, "camera_service.ENCODE_MODE_NAMES not found")
        for const in self.implemented:
            self.assertIn(
                const, self.constants,
                f"{const} is listed in _ENCODE_MODE_IMPLEMENTED but has no "
                "integer ENCODE_MODE_* assignment at module scope")

    def test_implemented_modes_exist_in_base_enum_with_same_value(self) -> None:
        members = EncodeMode.__members__
        for const in self.implemented:
            name = _MODE_CONST_RE.match(const).group(1)
            value = self.constants[const]
            with self.subTest(mode=const):
                self.assertIn(
                    name, members,
                    f"camera_service.{const} = {value} is implemented by the "
                    f"tractor but lora_proto.EncodeMode has no member {name!r}: "
                    f"image_rx_daemon rejects mode {value} and the operator "
                    "cannot select it from the base")
                self.assertEqual(
                    int(members[name]), value,
                    f"camera_service.{const} = {value} but "
                    f"lora_proto.EncodeMode.{name} = {int(members[name])}: the "
                    "two ends of CMD_ENCODE_MODE (0x63) have drifted")

    def test_shared_names_share_values(self) -> None:
        # Placeholders the tractor only clamps (BTC4_*, ADAPTIVE) still ride
        # the wire by value from the base ladder, so any name both sides
        # know must mean the same byte on both sides.
        members = EncodeMode.__members__
        for const, value in sorted(self.constants.items()):
            name = _MODE_CONST_RE.match(const).group(1)
            if name in members:
                with self.subTest(mode=const):
                    self.assertEqual(
                        int(members[name]), value,
                        f"camera_service.{const} = {value} but "
                        f"lora_proto.EncodeMode.{name} = {int(members[name])}")

    def test_ack_names_match_base_enum_names(self) -> None:
        by_value = {int(m): m for m in EncodeMode}
        for const in self.implemented:
            value = self.constants[const]
            with self.subTest(mode=const):
                self.assertIn(value, by_value,
                              f"{const} = {value} has no EncodeMode value")
                self.assertLess(
                    value, len(self.names),
                    f"{const} = {value} is past the end of ENCODE_MODE_NAMES")
                self.assertEqual(
                    self.names[value], by_value[value].name.lower(),
                    f"tractor acks {const} as {self.names[value]!r} but the "
                    f"base calls value {value} {by_value[value].name.lower()!r}; "
                    "web_ui would render every pin of it as a mismatch")


class EncodeModeUiChoicesSIL(unittest.TestCase):
    """The operator menu offers only tractor-implemented modes, the cycle
    button walks a subset of the menu, and the POST validator accepts
    every entry the menu offers."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.constants, cls.implemented, cls.names = _tractor_mode_table()
        cls.implemented_names = {
            cls.names[cls.constants[c]] for c in cls.implemented
            if c in cls.constants and cls.constants[c] < len(cls.names)}
        assigns = _module_assignments(WEB_UI_PY)
        cls.choices = tuple(ast.literal_eval(assigns["_ENCODE_MODE_UI_CHOICES"]))
        cls.cycle = tuple(ast.literal_eval(assigns["_ENCODE_MODE_CYCLE_ORDER"]))
        cls.pattern = None
        tree = ast.parse(WEB_UI_PY.read_text(encoding="utf-8"))
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == "EncodeModeBody":
                for stmt in node.body:
                    if isinstance(stmt, ast.AnnAssign) \
                            and isinstance(stmt.target, ast.Name) \
                            and stmt.target.id == "mode" \
                            and isinstance(stmt.value, ast.Call):
                        for kw in stmt.value.keywords:
                            if kw.arg == "pattern" and isinstance(kw.value, ast.Constant):
                                cls.pattern = kw.value.value

    def test_menu_offers_only_implemented_modes(self) -> None:
        self.assertTrue(self.choices, "web_ui._ENCODE_MODE_UI_CHOICES not found")
        for choice in self.choices:
            with self.subTest(choice=choice):
                self.assertIn(
                    choice, self.implemented_names,
                    f"web_ui offers {choice!r} but the tractor does not "
                    "implement it — a placebo menu entry that clamps to y_only")

    def test_cycle_order_is_a_subset_of_the_menu(self) -> None:
        for mode in self.cycle:
            with self.subTest(mode=mode):
                self.assertIn(mode, self.choices)

    def test_post_validator_accepts_every_menu_entry(self) -> None:
        self.assertIsNotNone(self.pattern, "EncodeModeBody.mode pattern not found")
        for choice in self.choices:
            with self.subTest(choice=choice):
                self.assertIsNotNone(
                    re.fullmatch(self.pattern, choice),
                    f"web_ui offers {choice!r} but EncodeModeBody.mode's "
                    f"pattern {self.pattern!r} would 422 it")


if __name__ == "__main__":
    unittest.main()
