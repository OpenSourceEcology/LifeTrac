"""VS1 codec mirror parity SIL: tractor ``x8_image_pipeline/vs1_codec.py``
vs base ``image_pipeline/vector_scene/codec.py``.

The tractor image is built from ``firmware/tractor_x8`` alone
(``README-DEPLOY.md`` step 1: that directory is the Docker context and the
Dockerfile copies nothing else), so ``encode_vector.py`` cannot import the
base-station tree. Like the codec-id table ``camera_service.py`` duplicates
"to avoid importing the base-station tree from the tractor", it carries a
copy of the codec. This SIL pins that copy byte-identical to the base file
below its two-line mirror header, so the wire format cannot drift between
the two ends without this test failing (the ``ENCODE_MODE_RAWSTREAM``
lesson, VECTOR_SCENE.md B2). To fix a failure, re-copy the base file over
the mirror and put the header back; never edit the mirror by hand.

Pure stdlib: no numpy, no cv2, no import of either module.
"""

from __future__ import annotations

import difflib
import unittest
from pathlib import Path

# Repo-relative paths.
REPO_ROOT = Path(__file__).resolve().parents[3]
BASE_CODEC_PY = (REPO_ROOT / "DESIGN-CONTROLLER" / "base_station" / "image_pipeline"
                 / "vector_scene" / "codec.py")
TRACTOR_MIRROR_PY = (REPO_ROOT / "DESIGN-CONTROLLER" / "firmware" / "tractor_x8"
                     / "x8_image_pipeline" / "vs1_codec.py")
MIRROR_HEADER_LINES = 2

RECOPY_HINT = (
    "the tractor codec mirror has drifted from the base codec. Re-copy\n"
    "  base_station/image_pipeline/vector_scene/codec.py\n"
    "over\n"
    "  firmware/tractor_x8/x8_image_pipeline/vs1_codec.py\n"
    "and restore its two-line '# Mirror of ...' header; never edit the mirror by hand."
)


def _lines(path: Path) -> list:
    """Lines with their endings normalised, so a CRLF checkout compares equal."""
    return path.read_text(encoding="utf-8").splitlines(keepends=True)


class Vs1CodecMirrorParityTests(unittest.TestCase):
    def test_both_files_exist(self) -> None:
        self.assertTrue(BASE_CODEC_PY.is_file(), BASE_CODEC_PY)
        self.assertTrue(TRACTOR_MIRROR_PY.is_file(), TRACTOR_MIRROR_PY)

    def test_mirror_header_is_two_comment_lines_naming_the_base_and_this_test(self) -> None:
        lines = _lines(TRACTOR_MIRROR_PY)
        self.assertGreater(len(lines), MIRROR_HEADER_LINES)
        header = lines[:MIRROR_HEADER_LINES]
        for line in header:
            self.assertTrue(line.startswith("# "), f"mirror header line is not a comment: {line!r}")
        self.assertIn("base_station/image_pipeline/vector_scene/codec.py", header[0])
        self.assertIn(Path(__file__).name, header[1])
        # Exactly two lines: the base file starts with its docstring, never a
        # comment, so a third comment line would be code the base does not have.
        self.assertFalse(lines[MIRROR_HEADER_LINES].startswith("#"),
                         "the mirror header must be exactly two lines")
        self.assertFalse(_lines(BASE_CODEC_PY)[0].startswith("#"),
                         "the base codec must not start with a comment (the header strip assumes it)")

    def test_mirror_is_identical_to_the_base_below_the_header(self) -> None:
        base = _lines(BASE_CODEC_PY)
        mirror = _lines(TRACTOR_MIRROR_PY)[MIRROR_HEADER_LINES:]
        if mirror != base:
            diff = "".join(difflib.unified_diff(base, mirror, "base codec.py", "tractor vs1_codec.py", n=1))
            self.fail(RECOPY_HINT + "\n\nFirst differences:\n" + "".join(diff.splitlines(keepends=True)[:40]))


if __name__ == "__main__":
    unittest.main()
