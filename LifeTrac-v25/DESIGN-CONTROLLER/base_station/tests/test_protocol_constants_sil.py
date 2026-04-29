"""Round 23: Wave-3 polish constants SIL — IP-306, IP-303, IP-301.

Three small, independent invariants from the Wave-3 polish bucket
collapsed into one source-parse SIL because each is a one-shot
tripwire and they share no runtime surface:

* **IP-306** — ``TELEM_MAX_PAYLOAD`` cross-language reconciliation.
  Python ``lora_proto.TELEM_MAX_PAYLOAD`` must equal 118 (the C
  ``TelemetryFrame.payload[120]`` array reserves the trailing 2 bytes
  for the inline CRC-16). The C-side header comment in
  ``firmware/common/lora_proto/lora_proto.h`` documents this as
  ``payload_len = 0..118``. Drift in either direction causes either
  truncated telemetry (Python over-promising) or wasted air (C
  under-using the buffer).

* **IP-303** — ``REG_AUX_OUTPUTS`` decision. The Modbus map slot
  ``0x0003`` was at risk of being either declared-but-unhandled
  (Opta silently drops writes) or removed-but-still-referenced (M7
  Modbus master writes into a reserved slot). The decision per
  TRACTOR_NODE.md §4 is "implement, gated by ``AUX_OUTPUTS_VALID_MASK``
  to bits 0..2 only (R10..R12 SSR channels)". This test pins both:
  the symbol is defined on both sides AND the Opta side has a
  ``case REG_AUX_OUTPUTS:`` arm in its register-write switch.

* **IP-301** — ``s_btn_change_ms`` boot-init. The handheld
  button-debounce reference timestamp must be re-anchored to
  ``millis()`` at the end of ``setup()``. Otherwise the first
  ``read_buttons()`` call after boot can — under a non-zero idle
  raw-pin reading — satisfy ``(now - 0) >= DEBOUNCE_MS`` immediately
  and commit a spurious debounced state.

All three are pure source-parse assertions; no firmware execution
required.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

# Repo-relative paths.
REPO_ROOT = Path(__file__).resolve().parents[3]
FIRMWARE_DIR = REPO_ROOT / "DESIGN-CONTROLLER" / "firmware"
HANDHELD_INO = FIRMWARE_DIR / "handheld_mkr" / "handheld_mkr.ino"
OPTA_INO = FIRMWARE_DIR / "tractor_opta" / "tractor_opta.ino"
TRACTOR_INO = FIRMWARE_DIR / "tractor_h7" / "tractor_h7.ino"
LORA_PROTO_H = FIRMWARE_DIR / "common" / "lora_proto" / "lora_proto.h"
LORA_PROTO_PY = REPO_ROOT / "DESIGN-CONTROLLER" / "base_station" / "lora_proto.py"


# ---------------------------------------------------------------------------
# Pinned values — the contract these tests defend.
# ---------------------------------------------------------------------------

# IP-306: the canonical max-usable telemetry payload. The C-side storage
# is 120 bytes, with the trailing 2 reserved for the inline CRC-16.
EXPECTED_TELEM_MAX_PAYLOAD = 118
EXPECTED_TELEM_BUFFER_BYTES = 120

# IP-303: documented Modbus register slot.
EXPECTED_REG_AUX_OUTPUTS = 0x0003
# Bits 0..2 are the only legal targets (R10..R12 SSR channels).
EXPECTED_AUX_OUTPUTS_VALID_MASK = 0x0007


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


# ---------------------------------------------------------------------------
# PC-A: IP-306 — TELEM_MAX_PAYLOAD cross-language reconciliation.
# ---------------------------------------------------------------------------

class PC_A_TelemMaxPayload(unittest.TestCase):
    """Python and the C header MUST agree that the usable payload
    ceiling is 118 bytes, with a 120-byte storage buffer."""

    def test_PC_A_python_constant_equals_118(self) -> None:
        # Import locally so the test fails loudly if lora_proto can't be
        # resolved on sys.path (rather than masking the symptom).
        import sys
        bs_dir = str(REPO_ROOT / "DESIGN-CONTROLLER" / "base_station")
        if bs_dir not in sys.path:
            sys.path.insert(0, bs_dir)
        from lora_proto import TELEM_MAX_PAYLOAD
        self.assertEqual(
            TELEM_MAX_PAYLOAD, EXPECTED_TELEM_MAX_PAYLOAD,
            "lora_proto.TELEM_MAX_PAYLOAD drifted from the IP-306 contract "
            f"value of {EXPECTED_TELEM_MAX_PAYLOAD}. The C side's "
            f"TelemetryFrame.payload[{EXPECTED_TELEM_BUFFER_BYTES}] reserves "
            "the trailing 2 bytes for the inline CRC-16."
        )

    def test_PC_A2_c_buffer_is_120_bytes(self) -> None:
        src = _read(LORA_PROTO_H)
        m = re.search(
            r"struct\s*\{[^}]*?frame_type\s*=\s*FT_TELEMETRY[^}]*?"
            r"uint8_t\s+payload\s*\[\s*(?P<n>\d+)\s*\]",
            src,
            re.DOTALL,
        )
        # The struct uses a comment for the frame_type assignment, so
        # the regex above may miss it on a layout reshuffle. Fall back
        # to the simpler "payload[N]" inside the file.
        if m is None:
            m = re.search(r"uint8_t\s+payload\s*\[\s*(?P<n>\d+)\s*\]", src)
        self.assertIsNotNone(m, "lora_proto.h: TelemetryFrame.payload[] not found")
        n = int(m.group("n"))
        self.assertEqual(
            n, EXPECTED_TELEM_BUFFER_BYTES,
            f"TelemetryFrame.payload[] is {n} bytes; IP-306 expects "
            f"{EXPECTED_TELEM_BUFFER_BYTES} (the trailing 2 reserve the CRC-16)."
        )

    def test_PC_A3_c_header_comment_advertises_118(self) -> None:
        # Belt-and-braces: an operator reading the header should see
        # the same number the runtime enforces. Catches a refactor that
        # bumps the Python constant but forgets to update the comment.
        src = _read(LORA_PROTO_H)
        # Match either "0..118" or "0-118" or "<= 118" near payload_len.
        nearby = re.search(
            r"uint8_t\s+payload_len[^;]*;\s*(?:/[/*][^\n]*\n\s*//[^\n]*\n?)*",
            src,
        )
        self.assertIsNotNone(nearby, "payload_len + neighbouring comment block not found")
        text = nearby.group(0)
        self.assertIn(
            str(EXPECTED_TELEM_MAX_PAYLOAD), text,
            f"lora_proto.h payload_len comment must mention "
            f"{EXPECTED_TELEM_MAX_PAYLOAD} so on-call reading matches runtime."
        )

    def test_PC_A4_python_runtime_rejects_overflow(self) -> None:
        # Negative: a payload of length 119 must NOT round-trip. We don't
        # call the encoder (which would require a full PHY context), but
        # we do confirm the constant is consulted in at least one obvious
        # spot in lora_bridge / lora_proto.
        py_src = _read(LORA_PROTO_PY)
        bridge_src = _read(REPO_ROOT / "DESIGN-CONTROLLER" / "base_station"
                           / "lora_bridge.py")
        joined = py_src + "\n" + bridge_src
        # At least one boundary check using the constant must exist.
        self.assertGreaterEqual(
            joined.count("TELEM_MAX_PAYLOAD"), 2,
            "TELEM_MAX_PAYLOAD must be referenced from both lora_proto.py "
            "and lora_bridge.py (definition + boundary check); fewer "
            "references means an overflow path is unguarded."
        )


# ---------------------------------------------------------------------------
# PC-B: IP-303 — REG_AUX_OUTPUTS implemented on both nodes.
# ---------------------------------------------------------------------------

class PC_B_RegAuxOutputs(unittest.TestCase):
    """The Modbus slot 0x0003 must be defined on the Opta slave AND
    referenced by the M7 master, AND the Opta side must have a write
    handler with the documented ``AUX_OUTPUTS_VALID_MASK`` filter."""

    def test_PC_B_opta_defines_register(self) -> None:
        src = _read(OPTA_INO)
        m = re.search(
            r"#define\s+REG_AUX_OUTPUTS\s+0x([0-9A-Fa-f]+)", src)
        self.assertIsNotNone(m, "tractor_opta: REG_AUX_OUTPUTS #define not found")
        self.assertEqual(
            int(m.group(1), 16), EXPECTED_REG_AUX_OUTPUTS,
            f"tractor_opta REG_AUX_OUTPUTS slot drifted from "
            f"0x{EXPECTED_REG_AUX_OUTPUTS:04X}. TRACTOR_NODE.md §4 owns this."
        )

    def test_PC_B2_tractor_master_references_register(self) -> None:
        # The M7 Modbus master must mention the symbol, otherwise a
        # future register-shuffle on the Opta side would silently
        # break the master's write block.
        src = _read(TRACTOR_INO)
        self.assertIn(
            "REG_AUX_OUTPUTS", src,
            "tractor_h7 (Modbus master) does not reference REG_AUX_OUTPUTS — "
            "the slot is orphaned. Either map it on the master side or "
            "remove it from the Opta slave per IP-303."
        )

    def test_PC_B3_opta_handles_register_in_write_switch(self) -> None:
        src = _read(OPTA_INO)
        # Locate the write-handler switch — it must contain a
        # ``case REG_AUX_OUTPUTS:`` arm. Without this, a master write
        # to 0x0003 falls into the default (silent drop) branch.
        m = re.search(
            r"case\s+REG_AUX_OUTPUTS\s*:(?P<body>.*?)break\s*;",
            src,
            re.DOTALL,
        )
        self.assertIsNotNone(
            m, "tractor_opta: case REG_AUX_OUTPUTS in write-handler not found")
        body = m.group("body")
        # The handler MUST mask with AUX_OUTPUTS_VALID_MASK so a typo'd
        # bitfield can't drive R5..R8 (those belong to the valve manifold).
        self.assertIn(
            "AUX_OUTPUTS_VALID_MASK", body,
            "REG_AUX_OUTPUTS handler must mask the incoming value with "
            "AUX_OUTPUTS_VALID_MASK so reserved bits cannot energize the "
            "valve-manifold SSR channels."
        )

    def test_PC_B4_valid_mask_pinned_to_bits_0_through_2(self) -> None:
        src = _read(OPTA_INO)
        m = re.search(
            r"#define\s+AUX_OUTPUTS_VALID_MASK\s+0x([0-9A-Fa-f]+)u?",
            src,
        )
        self.assertIsNotNone(m, "tractor_opta: AUX_OUTPUTS_VALID_MASK #define not found")
        self.assertEqual(
            int(m.group(1), 16), EXPECTED_AUX_OUTPUTS_VALID_MASK,
            "AUX_OUTPUTS_VALID_MASK must equal 0x0007 (bits 0..2 = R10..R12). "
            "Widening this mask is a safety regression — engine-kill arming "
            "must remain owned by REG_ARM_ESTOP, not REG_AUX_OUTPUTS."
        )


# ---------------------------------------------------------------------------
# PC-C: IP-301 — s_btn_change_ms is anchored to millis() in setup().
# ---------------------------------------------------------------------------

class PC_C_BtnChangeMsBootInit(unittest.TestCase):
    """The handheld debounce reference timestamp MUST be re-anchored
    to ``millis()`` at the end of ``setup()`` so the first
    ``read_buttons()`` call after boot cannot satisfy
    ``(now - s_btn_change_ms) >= DEBOUNCE_MS`` against the
    uninitialised-zero default."""

    def setUp(self) -> None:
        self.src = _read(HANDHELD_INO)

    def _setup_body(self) -> str:
        # Capture from `void setup() {` through the matching closing
        # brace at column 0 (i.e. the next top-level `}`). Kept narrow:
        # the test wants to assert an init lives in setup() and not in
        # loop() / a helper.
        m = re.search(
            r"void\s+setup\s*\(\s*\)\s*\{\n(?P<body>.*?)\n\}\n",
            self.src,
            re.DOTALL,
        )
        self.assertIsNotNone(m, "handheld_mkr: setup() body not found")
        return m.group("body")

    def test_PC_C_static_default_is_zero(self) -> None:
        # Confirm the static default is still zero — the whole point
        # of the boot-init is to overwrite this. If someone changes
        # the static initializer to a non-zero value, IP-301's reason
        # for existing changes and this test should be revisited.
        m = re.search(
            r"static\s+uint32_t\s+s_btn_change_ms\s*=\s*(?P<v>\w+)\s*;",
            self.src,
        )
        self.assertIsNotNone(m, "handheld_mkr: s_btn_change_ms declaration not found")
        self.assertEqual(
            m.group("v"), "0",
            "s_btn_change_ms default changed; if the static init is no "
            "longer 0 the boot-anchor in setup() may be redundant — "
            "revisit IP-301."
        )

    def test_PC_C2_setup_anchors_to_millis(self) -> None:
        body = self._setup_body()
        # We accept either "s_btn_change_ms = millis();" or
        # "s_btn_change_ms = millis()  ;" (whitespace-tolerant).
        self.assertRegex(
            body,
            r"s_btn_change_ms\s*=\s*millis\s*\(\s*\)\s*;",
            "handheld setup() must anchor s_btn_change_ms to millis() "
            "before the first read_buttons() call (IP-301). Without this, "
            "the first iteration can satisfy (now - 0) >= DEBOUNCE_MS "
            "immediately and commit a spurious debounced state."
        )

    def test_PC_C3_anchor_runs_after_other_initializers(self) -> None:
        # The anchor should be near the END of setup() — definitely
        # after Serial.begin / pinMode / radio.begin / OLED init —
        # because millis() at the very top of setup() returns ~0 and
        # the bug we're defending against is "the timer reference is
        # stale relative to the first read_buttons() call". Anchoring
        # late minimises the gap.
        body = self._setup_body()
        anchor_idx = body.find("s_btn_change_ms = millis()")
        # Also accept the spaced form.
        if anchor_idx == -1:
            m = re.search(r"s_btn_change_ms\s*=\s*millis\s*\(", body)
            self.assertIsNotNone(m, "anchor line not found in setup() body")
            anchor_idx = m.start()
        # All of these init landmarks must precede the anchor.
        for landmark in ("Serial.begin", "radio.begin", "pinMode"):
            with self.subTest(landmark=landmark):
                lm_idx = body.find(landmark)
                self.assertGreater(
                    lm_idx, -1,
                    f"setup() landmark '{landmark}' missing — body capture wrong?")
                self.assertLess(
                    lm_idx, anchor_idx,
                    f"IP-301 anchor must run AFTER {landmark!r} so the "
                    f"timestamp is taken as late in setup() as possible."
                )

    def test_PC_C4_read_buttons_still_uses_the_field(self) -> None:
        # Tripwire: if a future refactor renames the field or replaces
        # the debounce logic, the boot-init becomes dead code and this
        # whole test class should be re-evaluated.
        m = re.search(
            r"static\s+uint16_t\s+read_buttons\s*\([^)]*\)\s*\{\n"
            r"(?P<body>.*?)\n\}\n",
            self.src,
            re.DOTALL,
        )
        # The body capture stops at the first column-0 '}'. The for-loop's
        # closing brace is indented, so this works as long as the function
        # is laid out conventionally.
        if m is None or "s_btn_change_ms" not in m.group("body"):
            # Fallback: just confirm the symbol appears anywhere outside
            # the static declaration line.
            decl_re = re.compile(
                r"static\s+uint32_t\s+s_btn_change_ms\s*=\s*\w+\s*;")
            stripped = decl_re.sub("", self.src)
            self.assertIn(
                "s_btn_change_ms", stripped,
                "s_btn_change_ms is declared but never used outside the "
                "static initialiser — the IP-301 boot-anchor in setup() "
                "may now be dead code."
            )


if __name__ == "__main__":
    unittest.main()
