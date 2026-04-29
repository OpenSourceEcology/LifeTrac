"""Round 20: Boot-PHY first-frame decode SIL (W4-07 / IP-006).

W4-07 / IP-006 verification surface:

* All LoRa nodes on the LifeTrac control link MUST boot with their
  radios tuned to the SAME PHY parameters (SF, BW, CR, sync word) so
  the very first frame transmitted by any node after power-up decodes
  on every peer that has not yet received a ``CMD_LINK_TUNE``.

* The canonical boot PHY is the rung-0 entry of the adaptive-SF
  ladder, ``LADDER[0] = { sf=7, bw_khz=250, cr_den=5, bw_code=1 }``,
  per DECISIONS.md D-A2 (SF7 / BW250 / CR4-5). Any drift between

    1. ``LADDER[0]`` in ``firmware/tractor_h7/tractor_h7.ino``,
    2. ``LADDER[0]`` in ``firmware/handheld_mkr/handheld_mkr.ino``,
    3. the ``radio.begin(...)`` boot args inside each firmware's
       ``setup()``,
    4. ``DEFAULT_PARAMS["link"]["control_phy"]`` in
       ``firmware/tractor_x8/params_service.py``,

  is the failure mode this test catches at CI time without any radio.

* Additionally, ``CMD_LINK_TUNE`` rung-mapping breaks the moment the
  three-row ``LADDER[]`` tables on tractor and handheld disagree even
  on a non-rung-0 entry — e.g. tractor steps to its rung 1 (SF8/BW125)
  and broadcasts (sf=8, bw_code=0); handheld looks up (8, 0) in its
  ``LADDER[]`` and finds the wrong rung index. So the full table is
  required to be byte-identical between the two source nodes that
  carry one.

This module is pure-stdlib parsing of the firmware source files. It
runs in CI alongside the other base_station tests.
"""

from __future__ import annotations

import re
import unittest
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

# Repo-relative paths — base_station/tests/ → DESIGN-CONTROLLER/firmware/...
REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_DIR = REPO_ROOT / "DESIGN-CONTROLLER" / "firmware"
TRACTOR_INO = FIRMWARE_DIR / "tractor_h7" / "tractor_h7.ino"
HANDHELD_INO = FIRMWARE_DIR / "handheld_mkr" / "handheld_mkr.ino"
PARAMS_PY = FIRMWARE_DIR / "tractor_x8" / "params_service.py"


# ---------------------------------------------------------------------------
# DECISIONS.md D-A2 canonical boot PHY.
# ---------------------------------------------------------------------------

CANONICAL_BOOT_SF = 7
CANONICAL_BOOT_BW_KHZ = 250
CANONICAL_BOOT_CR_DEN = 5
CANONICAL_BOOT_BW_CODE = 1     # RadioLib bw_code 1 = 250 kHz
CANONICAL_FREQ_MHZ = 915.0
CANONICAL_SYNC_WORD = 0x12     # private LoRa sync (not LoRaWAN's 0x34)


@dataclass(frozen=True)
class LadderEntry:
    sf: int
    bw_khz: int
    cr_den: int
    bw_code: int


# ---------------------------------------------------------------------------
# Parsers — kept narrow on purpose (we WANT them to be brittle so a
# refactor of the C source forces a re-check of this test).
# ---------------------------------------------------------------------------


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


_LADDER_TABLE_RE = re.compile(
    r"static\s+const\s+LadderRung\s+LADDER\s*\[\s*3\s*\]\s*="
    r"\s*\{(?P<body>.*?)\};",
    re.DOTALL,
)
_LADDER_ROW_RE = re.compile(
    r"\{\s*(?P<sf>\d+)\s*,"
    r"\s*(?P<bw>\d+)\s*,"
    r"\s*(?P<cr>\d+)\s*,"
    r"\s*(?P<code>\d+)\s*\}"
)


def parse_ladder(source: str, where: str) -> Tuple[LadderEntry, ...]:
    """Extract the three-row LADDER[] table from a .ino source.

    Raises ``AssertionError`` with a descriptive message if the table
    is missing or doesn't have exactly three rows.
    """
    m = _LADDER_TABLE_RE.search(source)
    if m is None:
        raise AssertionError(
            f"{where}: could not locate `static const LadderRung LADDER[3] = {{...}};` "
            "— either the rung-table declaration was renamed, or the rung "
            "count differs from 3. Both situations require a deliberate "
            "review of IP-006 / CMD_LINK_TUNE rung mapping."
        )
    rows = _LADDER_ROW_RE.findall(m.group("body"))
    if len(rows) != 3:
        raise AssertionError(
            f"{where}: LADDER[] must have exactly 3 rows (found {len(rows)})"
        )
    return tuple(
        LadderEntry(int(sf), int(bw), int(cr), int(code))
        for (sf, bw, cr, code) in rows
    )


# RadioLib radio.begin signature used by both nodes:
#   radio.begin(freq_mhz, bw_khz, sf, cr_den, sync_word, tx_power_dbm)
# Two of the args may be wrapped in `(float)...` casts.
_RADIO_BEGIN_RE = re.compile(
    r"radio\.begin\s*\(\s*"
    r"(?P<freq>[^,]+)\s*,\s*"
    r"(?P<bw>[^,]+)\s*,\s*"
    r"(?P<sf>[^,]+)\s*,\s*"
    r"(?P<cr>[^,]+)\s*,\s*"
    r"(?P<sync>[^,]+)\s*,\s*"
    r"(?P<txpwr>[^,)]+)\s*\)",
    re.DOTALL,
)


@dataclass(frozen=True)
class RadioBeginCall:
    freq_text: str
    bw_text: str
    sf_text: str
    cr_text: str
    sync_text: str
    txpwr_text: str

    def normalize(self, expr: str) -> str:
        # Strip whitespace, `// ...` line comments, and `(float)` casts
        # so we can compare the *meaning* of each arg, not its formatting.
        s = expr
        s = re.sub(r"//[^\n]*", "", s)
        s = re.sub(r"\(\s*float\s*\)", "", s)
        s = re.sub(r"\s+", "", s)
        return s


def find_radio_begins(source: str) -> List[RadioBeginCall]:
    return [
        RadioBeginCall(
            freq_text=m.group("freq"),
            bw_text=m.group("bw"),
            sf_text=m.group("sf"),
            cr_text=m.group("cr"),
            sync_text=m.group("sync"),
            txpwr_text=m.group("txpwr"),
        )
        for m in _RADIO_BEGIN_RE.finditer(source)
    ]


# ---------------------------------------------------------------------------
# Tests.
# ---------------------------------------------------------------------------


class W4_07_LadderConsistencyTests(unittest.TestCase):
    """BP-A: the LADDER[] table is byte-identical across all source nodes
    that carry one (handheld + tractor M7).

    Drift here would break CMD_LINK_TUNE rung-mapping immediately on the
    first non-rung-0 step.
    """

    def setUp(self) -> None:
        self.handheld = parse_ladder(_read(HANDHELD_INO), "handheld_mkr.ino")
        self.tractor = parse_ladder(_read(TRACTOR_INO), "tractor_h7.ino")

    def test_BP_A_ladders_byte_identical(self) -> None:
        self.assertEqual(
            self.handheld, self.tractor,
            "handheld and tractor LADDER[] tables MUST agree row-for-row "
            "or CMD_LINK_TUNE rung mapping silently corrupts. Update both "
            "tables in lockstep and re-run the link-tune SIL "
            "(test_link_tune_sil.py) to confirm.",
        )

    def test_BP_A2_rung0_is_canonical_boot_phy(self) -> None:
        """BP-A2: LADDER[0] must be the DECISIONS.md D-A2 boot PHY."""
        for label, ladder in (("handheld", self.handheld), ("tractor", self.tractor)):
            with self.subTest(node=label):
                rung0 = ladder[0]
                self.assertEqual(rung0.sf, CANONICAL_BOOT_SF,
                                 f"{label} LADDER[0].sf must be SF7 (D-A2)")
                self.assertEqual(rung0.bw_khz, CANONICAL_BOOT_BW_KHZ,
                                 f"{label} LADDER[0].bw_khz must be 250 (D-A2)")
                self.assertEqual(rung0.cr_den, CANONICAL_BOOT_CR_DEN,
                                 f"{label} LADDER[0].cr_den must be CR4-5 (D-A2)")
                self.assertEqual(rung0.bw_code, CANONICAL_BOOT_BW_CODE,
                                 f"{label} LADDER[0].bw_code must be 1 (BW250)")

    def test_BP_A3_rungs_are_monotonically_widening(self) -> None:
        """BP-A3: SF strictly increases down the ladder (each rung wider).

        This is a structural invariant — the link-tune logic in
        try_step_ladder() assumes higher rung index = wider/slower SF.
        """
        for label, ladder in (("handheld", self.handheld), ("tractor", self.tractor)):
            with self.subTest(node=label):
                for i in range(len(ladder) - 1):
                    self.assertLess(
                        ladder[i].sf, ladder[i + 1].sf,
                        f"{label} LADDER[{i}].sf < LADDER[{i+1}].sf",
                    )

    def test_BP_A4_rung_phy_tuples_are_unique(self) -> None:
        """BP-A4: handheld's reverse-lookup (sf, bw_khz, cr_den) → rung
        relies on each rung being uniquely identifiable. A duplicate
        would map an inbound CMD_LINK_TUNE to an ambiguous rung.
        """
        for label, ladder in (("handheld", self.handheld), ("tractor", self.tractor)):
            with self.subTest(node=label):
                tuples = [(e.sf, e.bw_khz, e.cr_den) for e in ladder]
                self.assertEqual(len(set(tuples)), len(tuples),
                                 f"{label} LADDER[] rungs must be unique")


class W4_07_RadioBeginUsesLadderRung0Tests(unittest.TestCase):
    """BP-B: every radio.begin(...) call in the boot path references
    ``LADDER[0]`` symbolically (not numeric literals that could drift).
    """

    def _check_one(self, label: str, ino_path: Path) -> RadioBeginCall:
        source = _read(ino_path)
        calls = find_radio_begins(source)
        self.assertEqual(
            len(calls), 1,
            f"{label}: expected exactly one radio.begin() call in setup() "
            f"(found {len(calls)}). A second begin would mean either a "
            "boot-PHY override path slipped in or a debug-only sketch was "
            "merged — both warrant explicit review.",
        )
        return calls[0]

    def test_BP_B_handheld_radio_begin_uses_ladder_rung0(self) -> None:
        call = self._check_one("handheld_mkr.ino", HANDHELD_INO)
        self.assertEqual(call.normalize(call.freq_text), str(CANONICAL_FREQ_MHZ),
                         "handheld boot-PHY frequency must be 915.0 MHz")
        self.assertEqual(call.normalize(call.bw_text), "LADDER[0].bw_khz",
                         "handheld must boot with LADDER[0].bw_khz, not a literal")
        self.assertEqual(call.normalize(call.sf_text), "LADDER[0].sf",
                         "handheld must boot with LADDER[0].sf, not a literal")
        self.assertEqual(call.normalize(call.cr_text), "LADDER[0].cr_den",
                         "handheld must boot with LADDER[0].cr_den, not a literal")
        self.assertEqual(call.normalize(call.sync_text),
                         f"0x{CANONICAL_SYNC_WORD:02x}",
                         "handheld sync word must be 0x12 (private LoRa)")

    def test_BP_B2_tractor_radio_begin_uses_ladder_rung0(self) -> None:
        call = self._check_one("tractor_h7.ino", TRACTOR_INO)
        self.assertEqual(call.normalize(call.freq_text), str(CANONICAL_FREQ_MHZ))
        self.assertEqual(call.normalize(call.bw_text), "LADDER[0].bw_khz")
        self.assertEqual(call.normalize(call.sf_text), "LADDER[0].sf")
        self.assertEqual(call.normalize(call.cr_text), "LADDER[0].cr_den")
        self.assertEqual(call.normalize(call.sync_text),
                         f"0x{CANONICAL_SYNC_WORD:02x}")

    def test_BP_B3_no_node_uses_a_literal_sf_in_radio_begin(self) -> None:
        """BP-B3: regression-trap — if anyone replaces ``LADDER[0].sf``
        with a numeric literal (even the correct value 7) the symbolic
        coupling between radio.begin() and the ladder is lost.
        """
        for label, path in (("handheld_mkr.ino", HANDHELD_INO),
                            ("tractor_h7.ino", TRACTOR_INO)):
            with self.subTest(node=label):
                call = find_radio_begins(_read(path))[0]
                self.assertNotRegex(
                    call.normalize(call.sf_text),
                    r"^\d+$",
                    f"{label} radio.begin() SF arg must reference LADDER[0].sf, "
                    "not a numeric literal — the literal will silently drift "
                    "if someone re-orders the ladder rungs.",
                )
                self.assertNotRegex(
                    call.normalize(call.bw_text),
                    r"^\d+(?:\.\d+)?$",
                    f"{label} radio.begin() BW arg must reference LADDER[0].bw_khz",
                )
                self.assertNotRegex(
                    call.normalize(call.cr_text),
                    r"^\d+$",
                    f"{label} radio.begin() CR arg must reference LADDER[0].cr_den",
                )


class W4_07_PiSideAgreesTests(unittest.TestCase):
    """BP-C: the Pi-side ``params_service.py`` advertises the same boot
    PHY string. If the Pi says ``SF7_BW125`` but the radios boot at
    BW250 the operator UI shows the wrong PHY, and any field-rebuild of
    a node from Pi-pushed params would brick the link.
    """

    def test_BP_C_params_service_control_phy_matches_canonical(self) -> None:
        # Avoid importing tractor_x8.params_service (it has its own
        # path-resolution side-effects). Just match the canonical string.
        text = _read(PARAMS_PY)
        m = re.search(
            r'"control_phy"\s*:\s*"(?P<phy>[A-Za-z0-9_]+)"',
            text,
        )
        self.assertIsNotNone(
            m, "DEFAULT_PARAMS['link']['control_phy'] entry not found in "
               "params_service.py — refactor must keep this key.",
        )
        expected = f"SF{CANONICAL_BOOT_SF}_BW{CANONICAL_BOOT_BW_KHZ}"
        self.assertEqual(
            m.group("phy"), expected,
            f"params_service.DEFAULT_PARAMS['link']['control_phy'] must be "
            f"'{expected}' to match LADDER[0] (SF{CANONICAL_BOOT_SF}/"
            f"BW{CANONICAL_BOOT_BW_KHZ}). Drift here will mismatch the "
            "operator UI and any field-pushed params bundle.",
        )


class W4_07_ParserSelfTests(unittest.TestCase):
    """BP-D: parser self-tests — make sure the regex-based extractor
    actually catches the mis-formats it's supposed to catch. Without
    these the test file could silently pass on a refactored source
    where the table simply moved.
    """

    def test_BP_D_parser_rejects_2_row_ladder(self) -> None:
        bad = (
            "static const LadderRung LADDER[3] = {\n"
            "    { 7, 250, 5, 1 },\n"
            "    { 8, 125, 5, 0 }\n"
            "};\n"
        )
        with self.assertRaisesRegex(AssertionError, "exactly 3 rows"):
            parse_ladder(bad, "synthetic")

    def test_BP_D2_parser_rejects_missing_table(self) -> None:
        with self.assertRaisesRegex(AssertionError, "could not locate"):
            parse_ladder("// no ladder here", "synthetic")

    def test_BP_D3_radio_begin_finder_skips_unrelated_calls(self) -> None:
        src = (
            "// not a real begin: foo.begin(1,2,3,4,5,6);\n"
            "void setup() {\n"
            "    radio.begin(915.0, (float)LADDER[0].bw_khz, LADDER[0].sf,\n"
            "                LADDER[0].cr_den, 0x12, 14);\n"
            "}\n"
        )
        calls = find_radio_begins(src)
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0].normalize(calls[0].sf_text), "LADDER[0].sf")


if __name__ == "__main__":   # pragma: no cover
    unittest.main()
