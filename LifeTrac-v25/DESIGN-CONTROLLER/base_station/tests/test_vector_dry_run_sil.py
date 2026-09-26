"""tools/vector_dry_run.py — SIL tests for the RS-13 bench tool.

Pins: the one-fragment limits match the real packers (203 / 243 B for a
0xFE delta, one byte less for a 0xFD epoch start); a clean capture passes
every check; oversize, foreign and unparseable payloads fail the right
checks; a DIGEST mismatch fails the digest check; the VS header peek reads
the codec's own bits; camera_service's ``vector_stats`` line round-trips
through the log parser.
"""
from __future__ import annotations

import importlib.util
import io
import json
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
_REPO = _BS.parent
_TOOLS = _REPO / "tools"
_X8 = _REPO / "firmware" / "tractor_x8"
for _p in (_BS, _TOOLS):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import vector_dry_run as vdr  # noqa: E402
from lora_proto import (  # noqa: E402
    PHY_IMAGE_BW250, PHY_IMAGE_BW500, pack_image_fragments, pack_image_fragments_v2,
)
from image_pipeline.frame_format import (  # noqa: E402
    CODEC_VECTOR, CODEC_WEBP, FRAME_KIND_DELTA, FRAME_KIND_KEY, TileDeltaFrame,
    encode_tile_delta_frame,
)
from image_pipeline.vector_scene import codec as vs  # noqa: E402
from tests.test_vector_codec import worked_scene  # noqa: E402


def _wire(body: bytes, key: bool, seq: int) -> bytes:
    return encode_tile_delta_frame(TileDeltaFrame(
        frame_kind=FRAME_KIND_KEY if key else FRAME_KIND_DELTA, base_seq=seq,
        grid_w=12, grid_h=8, tile_px=32, codec=CODEC_VECTOR, vector_body=body))


def _scene() -> list:
    """The codec's worked scene minus its DIGEST, whose CRC is the design's
    worked value rather than what this store computes."""
    return [r for r in worked_scene() if not isinstance(r, vs.Digest)]


def _clean_frames() -> list:
    recs = _scene()
    return [
        (1000.0, _wire(vs.encode_frame(vs.Header(True, 0, 1), recs, 196), True, 1)),
        (1000.5, _wire(vs.encode_frame(vs.Header(False, 0, 1), recs[:3], 197), False, 2)),
        (1001.0, _wire(vs.encode_frame(vs.Header(False, 1, 1), recs[3:6], 197), False, 3)),
    ]


def _checks(dr: vdr.DryRun) -> dict:
    return {name: ok for name, ok, _ in dr.summary()["checks"]}


class LimitTests(unittest.TestCase):
    def test_limits_match_the_packers(self) -> None:
        for prof, expect in ((PHY_IMAGE_BW250, 203), (PHY_IMAGE_BW500, 243)):
            lim = vdr.one_fragment_limit(prof)
            self.assertEqual(lim, expect)
            self.assertEqual(len(pack_image_fragments(b"\0" * lim, 0, prof)), 1)
            self.assertEqual(len(pack_image_fragments(b"\0" * (lim + 1), 0, prof)), 2)
            # a keyframe sent with copies takes the 5 B 0xFD header: one chunk
            # x 2 copies at lim - 1, two chunks x 2 copies at lim (§3.1 F - 1)
            self.assertEqual(len(pack_image_fragments_v2(b"\0" * (lim - 1), 0, prof, copies=2)), 2)
            self.assertEqual(len(pack_image_fragments_v2(b"\0" * lim, 0, prof, copies=2)), 4)

    def test_row_limits_follow_the_frame_kind(self) -> None:
        dr = vdr.DryRun("image_bw500")
        rows = [dr.feed(p, ts) for ts, p in _clean_frames()]
        self.assertEqual([r.limit for r in rows], [242, 243, 243])
        self.assertTrue(all(r.fits for r in rows))

    def test_unknown_profile_is_refused(self) -> None:
        with self.assertRaises(ValueError):
            vdr.DryRun("telemetry_bw999")


class PeekTests(unittest.TestCase):
    def test_header_bits_come_from_the_codec_reader(self) -> None:
        body = vs.encode_frame(vs.Header(True, 3, 5, 2), _scene()[:1], 197)
        self.assertEqual(vdr.peek_vs_header(body),
                         {"marker": 0, "version": 0, "key": 1, "age": 3, "epoch": 5, "level": 2})
        self.assertIsNone(vdr.peek_vs_header(b"\x00"))


class ReplayTests(unittest.TestCase):
    def _capture(self, frames: list) -> str:
        path = os.path.join(tempfile.mkdtemp(), "cap.jsonl")
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(json.dumps({"meta": {"tool": "test"}}) + "\n\n")
            for ts, p in frames:
                vdr.write_capture_line(fh, ts, "t", p)
        return path

    def test_clean_capture_passes_every_check(self) -> None:
        path = self._capture(_clean_frames())
        out = io.StringIO()
        with redirect_stdout(out):
            rc = vdr.main(["replay", path, "--strict", "--profile", "image_bw250",
                           "--json", path + ".json"])
        self.assertEqual(rc, 0, out.getvalue())
        with open(path + ".json", encoding="utf-8") as fh:
            s = json.load(fh)
        self.assertTrue(s["pass"])
        self.assertEqual(s["frames"], {"total": 3, "vector": 3, "other": 0,
                                       "parse_errors": 0, "by_codec": {"vector": 3}})
        self.assertEqual(s["store"]["frames_applied"], 3)
        self.assertEqual((s["store"]["orphans"], s["store"]["frames_bad"]), (0, 0))
        self.assertEqual((s["epoch_starts"], s["epoch_starts_applied"]), (1, 1))
        self.assertEqual(s["arrival"]["frames_before_first_apply"], 0)
        self.assertAlmostEqual(s["arrival"]["fps_mean"], 2.0)
        self.assertEqual(s["scene"]["horizon_mode"], "abs")
        self.assertEqual(s["one_fragment_limit"], 203)
        text = out.getvalue()
        self.assertIn("RESULT: PASS", text)
        self.assertIn("EPOCH-SWITCH", text)
        self.assertIn("one-fragment limit 203 B (epoch starts 202 B)", text)

    def test_strict_fails_on_oversize_foreign_and_unparseable(self) -> None:
        dr = vdr.DryRun("image_bw250")
        for ts, p in _clean_frames():
            dr.feed(p, ts)
        big = vs.encode_frame(vs.Header(False, 0, 1), _scene() * 3, 600)   # > 203 B on the wire
        r_big = dr.feed(_wire(big, False, 4), 1001.5)
        self.assertFalse(r_big.fits)
        self.assertTrue(r_big.applied)                    # the store still takes it
        webp = bytes([FRAME_KIND_DELTA, 5, 12, 8, 32, CODEC_WEBP]) + bytes(12)   # empty WebP delta
        r_webp = dr.feed(webp, 1002.0)
        self.assertTrue(r_webp.parsed)
        self.assertFalse(r_webp.vector)
        r_bad = dr.feed(b"\x01\x02", 1002.5)
        self.assertFalse(r_bad.parsed)
        s = dr.summary()
        self.assertFalse(s["pass"])
        failed = {name for name, ok, _ in s["checks"] if not ok}
        self.assertEqual(failed, {"parse_ok", "all_vector", "one_fragment"})
        self.assertIn("OVER", vdr.format_row(r_big))
        self.assertIn("NOT VECTOR", vdr.format_row(r_webp))
        self.assertIn("UNPARSEABLE", vdr.format_row(r_bad))
        out = io.StringIO()
        with redirect_stdout(out):
            rc = vdr.main(["replay", self._capture([(1000.0, webp)]), "--strict", "--quiet"])
        self.assertEqual(rc, 1)
        self.assertIn("RESULT: FAIL", out.getvalue())

    def test_digest_mismatch_fails_the_digest_check(self) -> None:
        dr = vdr.DryRun()
        row = dr.feed(_wire(vs.encode_frame(vs.Header(True, 0, 1), worked_scene(), 196), True, 1), 1000.0)
        self.assertTrue(row.applied)
        self.assertEqual(row.digest, "BAD")
        self.assertFalse(_checks(dr)["digest"])

    def test_rejected_first_frame_fails_first_apply_and_store_clean(self) -> None:
        dr = vdr.DryRun()
        frames = _clean_frames()
        r0 = dr.feed(_wire(b"\xff\xff", False, 9), 999.5)    # marker bit set: bad_marker
        self.assertFalse(r0.applied)
        self.assertEqual(r0.reason, "bad_marker")
        self.assertIn("BAD:bad_marker", vdr.format_row(r0))
        for ts, p in frames:
            dr.feed(p, ts)
        s = dr.summary()
        checks = {name: ok for name, ok, _ in s["checks"]}
        self.assertFalse(checks["first_apply"])
        self.assertFalse(checks["store_clean"])
        self.assertTrue(checks["epoch_seen"])
        self.assertEqual(s["arrival"]["frames_before_first_apply"], 1)
        self.assertEqual(s["store"]["bad_reasons"], {"bad_marker": 1})

    def test_delta_before_any_epoch_start_is_the_stores_call(self) -> None:
        """The tool reports what the store does; it does not second-guess it.
        A fresh store adopts the epoch of a delta that arrives first."""
        dr = vdr.DryRun()
        frames = _clean_frames()
        r = dr.feed(frames[1][1], frames[1][0])
        self.assertTrue(r.applied)
        self.assertEqual(r.vs_epoch, 1)

    def test_capture_file_round_trip_skips_meta_and_junk(self) -> None:
        path = self._capture(_clean_frames())
        with open(path, "a", encoding="utf-8") as fh:
            fh.write("not json\n")
            fh.write(json.dumps({"ts": 1, "topic": "t", "hex": "zz"}) + "\n")
        err = io.StringIO()
        import contextlib
        with contextlib.redirect_stderr(err):
            rows = list(vdr.iter_capture(path))
        self.assertEqual([ts for ts, _, _ in rows], [1000.0, 1000.5, 1001.0])
        self.assertEqual(rows[0][2], _clean_frames()[0][1])
        self.assertIn("skipped", err.getvalue())


class TractorLogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        if str(_X8) not in sys.path:
            sys.path.insert(0, str(_X8))
        spec = importlib.util.spec_from_file_location(
            "x8_camera_service_rs13_dry_run", str(_X8 / "camera_service.py"))
        cls.cs = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = cls.cs
        spec.loader.exec_module(cls.cs)  # type: ignore[union-attr]

    def test_camera_service_line_round_trips(self) -> None:
        st = {"ms": {"resize": 3.14159, "l0": 8.0, "l1": 20.49, "total": 41.26},
              "frame_bytes": 203, "level": 0, "detail": 80, "epoch": 3, "n_live": 17,
              "residual": 0.0312, "epoch_pending": False}
        with self.assertLogs(self.cs.LOG, level="INFO") as cm:
            self.cs._log_vector_stats(st)
        line = cm.output[-1]
        d = vdr.parse_vector_stats_line(line)
        self.assertIsNotNone(d)
        self.assertEqual((d["bytes"], d["level"], d["detail"], d["epoch"], d["n_live"]),
                         (203, 0, 80, 3, 17))
        self.assertEqual(d["ms"]["resize"], 3.1)
        self.assertEqual(d["ms_total"], 41.3)
        self.assertEqual(d["residual"], 0.031)
        self.assertEqual(d["epoch_pending"], 0)
        summ = vdr.summarise_tractor_log([line, "unrelated noise", line.replace("41.3", "55.0")])
        self.assertEqual(summ["n"], 2)
        self.assertEqual(summ["ms_total"], {"p50": 41.3, "p95": 55.0, "max": 55.0})
        self.assertEqual(summ["stages_p50_ms"]["resize"], 3.1)
        self.assertEqual(summ["levels"], {"0": 2})
        self.assertEqual(summ["epoch_pending_lines"], 0)
        self.assertIn("ms_total: p50 41.3 / p95 55.0", vdr.format_tractor_summary(summ))
        self.assertIn("no vector_stats lines", vdr.format_tractor_summary(vdr.summarise_tractor_log([])))

    def test_empty_stats_log_nothing(self) -> None:
        with self.assertNoLogs(self.cs.LOG, level="INFO"):
            self.cs._log_vector_stats({})

    def test_parser_ignores_other_lines(self) -> None:
        self.assertIsNone(vdr.parse_vector_stats_line("camera_service: frame_health same_run=0"))
        self.assertIsNone(vdr.parse_vector_stats_line("vector_stats level=0"))   # no ms_total


if __name__ == "__main__":
    unittest.main()
