"""Round 9: cross-language ReplayWindow invariant test.

Gemini 3.1 Pro (second-pass §4.2.3) flagged that ``LP_REPLAY_WINDOW_BITS``
(C side) and ``REPLAY_WINDOW_BITS`` (Python side) are open-coded in two
languages, so silent drift is possible. Both must equal 64 — the bitmap
field is a single ``uint64_t`` on the firmware and an unbounded Python
int that's masked to ``(1 << REPLAY_WINDOW_BITS) - 1``.

This test parses ``firmware/common/lora_proto/lora_proto.h`` for the
``#define`` and asserts equality with the Python constant. If either
side is bumped without the other, this test fails immediately.
"""

from __future__ import annotations

import re
import unittest
from pathlib import Path

from lora_proto import REPLAY_WINDOW_BITS


_HEADER = (
    Path(__file__).resolve().parent.parent.parent
    / "firmware" / "common" / "lora_proto" / "lora_proto.h"
)


def _parse_c_define(name: str) -> int:
    text = _HEADER.read_text(encoding="utf-8")
    m = re.search(rf"^\s*#\s*define\s+{re.escape(name)}\s+(\d+)\s*$",
                  text, re.MULTILINE)
    if m is None:
        raise AssertionError(f"#define {name} not found in {_HEADER}")
    return int(m.group(1))


class TestReplayWindowInvariant(unittest.TestCase):

    def test_python_matches_c_header(self) -> None:
        c_value = _parse_c_define("LP_REPLAY_WINDOW_BITS")
        self.assertEqual(
            REPLAY_WINDOW_BITS, c_value,
            f"REPLAY_WINDOW_BITS drift: Python={REPLAY_WINDOW_BITS} "
            f"vs C LP_REPLAY_WINDOW_BITS={c_value}. Both sides must agree "
            "or replay-protection breaks asymmetrically.")

    def test_window_is_64_as_documented(self) -> None:
        # LORA_PROTOCOL.md and the LpReplayWindow uint64_t bitmap field
        # both depend on this. Catch unrelated changes that "happen to
        # match" between Python and C but break the documented spec.
        self.assertEqual(REPLAY_WINDOW_BITS, 64)


if __name__ == "__main__":
    unittest.main()
