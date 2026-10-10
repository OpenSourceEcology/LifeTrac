"""tools/vector_replay_publish.py — the RS-13.2 desk-check replay tool, no broker.

Pins: the capture reader numbers rows exactly as ``vector_dry_run`` does (meta,
blank, junk and bad-hex lines skipped); the schedule is the capture's own
inter-frame timing divided by ``--speed``, with ``--max-gap`` capping a wait
before the division and a backwards clock step counting as zero; the window
and ``--start-at-key`` pick the right rows; drops (``--drop-rows`` by capture
row, ``--drop-every`` by position in the pass) never publish but keep their
time slot; the replay loop publishes in capture order on an absolute schedule
(a slow publisher does not drift), repeats passes after the loop gap, stops on
request and on Ctrl-C; frames go out at QoS 0 and are never retained; the CLI
dry run prints and logs the plan.
"""
from __future__ import annotations

import io
import json
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stderr, redirect_stdout
from pathlib import Path
from unittest import mock

_BS = Path(__file__).resolve().parents[1]
_REPO = _BS.parent
_TOOLS = _REPO / "tools"
for _p in (_BS, _TOOLS):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import vector_replay_publish as vrp  # noqa: E402
from image_pipeline.frame_format import (  # noqa: E402
    CODEC_MONO_G4, CODEC_VECTOR, FRAME_KIND_DELTA, FRAME_KIND_KEY, TileBlob,
    TileDeltaFrame, encode_tile_delta_frame, parse_tile_delta_frame,
)

_R4 = _REPO / "bench-evidence" / "RS_13_vector_scene_2026-09-26" / "legs" / "leg2a_r4_base.jsonl"


def _vec(seq: int, key: bool = False, body: bytes = b"\x80\x01\x02") -> bytes:
    return encode_tile_delta_frame(TileDeltaFrame(
        frame_kind=FRAME_KIND_KEY if key else FRAME_KIND_DELTA, base_seq=seq,
        grid_w=12, grid_h=8, tile_px=32, codec=CODEC_VECTOR, vector_body=body))


def _rows(ts: list, keys: tuple = ()) -> list:
    return [vrp.Row(i, t, vrp.BASE_TOPIC, _vec(i + 1, key=(i in keys))) for i, t in enumerate(ts)]


class FakeClock:
    """A monotonic clock that only moves when the replay sleeps (or a fake
    publisher spends time)."""

    def __init__(self, t: float = 1000.0) -> None:
        self.t = t
        self.sleeps: list = []

    def __call__(self) -> float:
        return self.t

    def sleep(self, s: float) -> None:
        self.sleeps.append(s)
        self.t += s


class FakePublisher:
    def __init__(self, clock: FakeClock, cost_s: float = 0.0, interrupt_after: int = 0) -> None:
        self.clock = clock
        self.cost_s = cost_s
        self.interrupt_after = interrupt_after
        self.sent: list = []          # (clock time, topic, payload)

    def publish(self, topic: str, payload: bytes) -> None:
        if self.interrupt_after and len(self.sent) >= self.interrupt_after:
            raise KeyboardInterrupt
        self.sent.append((self.clock(), topic, payload))
        self.clock.t += self.cost_s

    def close(self) -> None:
        pass


def _write_capture(path: Path, rows: list, meta: bool = True) -> None:
    with open(path, "w", encoding="utf-8") as fh:
        if meta:
            fh.write(json.dumps({"meta": {"tool": "vector_dry_run", "topic": vrp.BASE_TOPIC}}) + "\n")
        for ts, payload in rows:
            fh.write(json.dumps({"ts": ts, "topic": vrp.BASE_TOPIC, "hex": payload.hex()}) + "\n")


class CaptureReaderTests(unittest.TestCase):
    def test_rows_skip_meta_blank_and_junk_like_the_dry_run(self) -> None:
        import vector_dry_run as vdr
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "cap.jsonl"
            _write_capture(p, [(10.0, _vec(1, key=True)), (10.5, _vec(2))])
            with open(p, "a", encoding="utf-8") as fh:
                fh.write("\n")
                fh.write("not json\n")
                fh.write(json.dumps({"ts": 11.0, "topic": "t", "hex": "zz"}) + "\n")
                fh.write(json.dumps({"ts": 11.5, "topic": "t"}) + "\n")
                fh.write(json.dumps({"ts": 12.0, "topic": "t", "hex": _vec(3).hex()}) + "\n")
            with redirect_stderr(io.StringIO()):
                rows = vrp.read_capture(p)
                ref = list(vdr.iter_capture(p))
        self.assertEqual([r.idx for r in rows], [0, 1, 2])
        self.assertEqual([(r.ts, r.topic, r.payload) for r in rows], ref)

    @unittest.skipUnless(_R4.is_file(), "round-4 capture not in this tree")
    def test_round4_capture_reads_as_the_dry_run_numbers_it(self) -> None:
        import vector_dry_run as vdr
        rows = vrp.read_capture(_R4)
        ref = list(vdr.iter_capture(_R4))
        self.assertEqual(len(rows), len(ref))
        self.assertEqual([r.payload for r in rows[:50]], [p for _, _, p in ref[:50]])
        steps = vrp.plan(rows)
        self.assertAlmostEqual(steps[-1].offset_s, rows[-1].ts - rows[0].ts, places=6)

    def test_header_peek_agrees_with_the_parser(self) -> None:
        tile = encode_tile_delta_frame(TileDeltaFrame(
            frame_kind=FRAME_KIND_KEY, base_seq=200, grid_w=12, grid_h=8, tile_px=32,
            changed_indices=[0], tiles=[TileBlob(0, 0, 0, b"\x01\x02")], codec=CODEC_MONO_G4))
        for wire in (_vec(77, key=False), _vec(5, key=True), tile):
            ref = parse_tile_delta_frame(wire)
            h = vrp.peek_header(wire)
            self.assertEqual((h["kind"], h["seq"], h["grid"], h["codec"]),
                             (ref.frame_kind, ref.base_seq, (ref.grid_w, ref.grid_h, ref.tile_px),
                              ref.codec))
        self.assertIsNone(vrp.peek_header(b"\x00\x01"))

    def test_row_spec(self) -> None:
        self.assertEqual(vrp.parse_row_spec("100,200-202"), {100, 200, 201, 202})
        self.assertEqual(vrp.parse_row_spec(" 3 , 5-5 ,"), {3, 5})
        self.assertEqual(vrp.parse_row_spec(""), frozenset())
        for bad in ("5-3", "-1", "x", "1-x"):
            with self.assertRaises(ValueError, msg=bad):
                vrp.parse_row_spec(bad)


class PlanTests(unittest.TestCase):
    TS = [100.0, 100.5, 101.0, 104.0, 103.9, 104.4]

    def offsets(self, **kw) -> list:
        return [round(s.offset_s, 6) for s in vrp.plan(_rows(self.TS), **kw)]

    def test_offsets_follow_the_capture_and_a_backwards_step_counts_zero(self) -> None:
        self.assertEqual(self.offsets(), [0.0, 0.5, 1.0, 4.0, 4.0, 4.5])

    def test_speed_divides_every_gap(self) -> None:
        self.assertEqual(self.offsets(speed=2.0), [0.0, 0.25, 0.5, 2.0, 2.0, 2.25])
        self.assertEqual(self.offsets(speed=0.5), [0.0, 1.0, 2.0, 8.0, 8.0, 9.0])

    def test_max_gap_caps_a_wait_before_the_speed_division(self) -> None:
        self.assertEqual(self.offsets(max_gap=1.0), [0.0, 0.5, 1.0, 2.0, 2.0, 2.5])
        self.assertEqual(self.offsets(max_gap=1.0, speed=2.0), [0.0, 0.25, 0.5, 1.0, 1.0, 1.25])

    def test_bad_arguments_are_refused(self) -> None:
        for kw in ({"speed": 0}, {"speed": -1}, {"max_gap": -0.1}, {"drop_every": -1}):
            with self.assertRaises(ValueError, msg=kw):
                vrp.plan(_rows(self.TS), **kw)

    def test_window_is_inclusive_and_starts_at_offset_zero(self) -> None:
        steps = vrp.plan(_rows(self.TS), start_row=2, end_row=4)
        self.assertEqual([s.row.idx for s in steps], [2, 3, 4])
        self.assertEqual([round(s.offset_s, 6) for s in steps], [0.0, 3.0, 3.0])

    def test_start_at_key_backs_up_to_the_last_epoch_start(self) -> None:
        rows = _rows([float(i) for i in range(8)], keys=(0, 3))
        self.assertEqual(vrp.plan(rows, start_row=5, start_at_key=True)[0].row.idx, 3)
        self.assertEqual(vrp.plan(rows, start_row=3, start_at_key=True)[0].row.idx, 3)
        self.assertEqual(vrp.plan(rows, start_row=2, start_at_key=True)[0].row.idx, 0)
        self.assertEqual(vrp.plan(rows, start_row=5)[0].row.idx, 5)
        self.assertEqual(vrp.plan(rows, start_row=20, start_at_key=True), [])
        no_keys = _rows([float(i) for i in range(4)])
        self.assertEqual(vrp.plan(no_keys, start_row=2, start_at_key=True)[0].row.idx, 0)

    def test_drop_rows_name_capture_rows(self) -> None:
        steps = vrp.plan(_rows([float(i) for i in range(8)]), drop_rows={1, 5, 99})
        self.assertEqual([s.row.idx for s in steps if s.drop], [1, 5])

    def test_drop_every_counts_frames_from_the_window_start(self) -> None:
        rows = _rows([float(i) for i in range(10)])
        steps = vrp.plan(rows, drop_every=3)
        self.assertEqual([s.row.idx for s in steps if s.drop], [2, 5, 8])
        steps = vrp.plan(rows, drop_every=3, start_row=2)
        self.assertEqual([s.row.idx for s in steps if s.drop], [4, 7])
        steps = vrp.plan(rows, drop_every=4, drop_rows={0})
        self.assertEqual([s.row.idx for s in steps if s.drop], [0, 3, 7])

    def test_a_dropped_frame_keeps_its_time_slot(self) -> None:
        kept = vrp.plan(_rows(self.TS))
        dropped = vrp.plan(_rows(self.TS), drop_rows={2})
        self.assertEqual([s.offset_s for s in kept], [s.offset_s for s in dropped])


class ReplayTests(unittest.TestCase):
    def _run(self, steps, **kw):
        clk = FakeClock()
        pub = kw.pop("publisher", None) or FakePublisher(clk)
        if isinstance(pub, FakePublisher):
            pub.clock = clk
        seen: list = []
        st = vrp.replay(steps, pub, "x/topic", clock=clk, sleep=clk.sleep,
                        on_step=lambda p, s, t: seen.append((p, s.row.idx, s.drop, round(t, 6))),
                        **kw)
        return st, pub, clk, seen

    def test_publishes_in_order_at_the_planned_times_and_skips_drops(self) -> None:
        rows = _rows([100.0, 100.5, 101.0, 104.0, 104.5])
        steps = vrp.plan(rows, drop_rows={2})
        st, pub, clk, seen = self._run(steps)
        self.assertEqual([p for _, _, p in pub.sent], [rows[i].payload for i in (0, 1, 3, 4)])
        self.assertEqual({t for _, t, _ in pub.sent}, {"x/topic"})
        self.assertEqual([round(t - 1000.0, 6) for t, _, _ in pub.sent], [0.0, 0.5, 4.0, 4.5])
        self.assertEqual((st.passes, st.published, st.dropped, st.dropped_rows), (1, 4, 1, [(1, 2)]))
        self.assertEqual(seen, [(1, 0, False, 0.0), (1, 1, False, 0.5), (1, 2, True, 1.0),
                                (1, 3, False, 4.0), (1, 4, False, 4.5)])
        self.assertFalse(st.interrupted)

    def test_a_slow_publisher_does_not_drift(self) -> None:
        rows = _rows([0.0, 0.5, 1.0, 1.5, 1.6, 2.0, 3.0])
        clk = FakeClock()
        pub = FakePublisher(clk, cost_s=0.3)
        vrp.replay(vrp.plan(rows), pub, "t", clock=clk, sleep=clk.sleep)
        # each send costs 0.3 s; rows 0-3 still start on schedule. Rows 4 and 5
        # (0.1 s and 0.4 s gaps) go out as soon as the previous send ends, and
        # the schedule is absolute, so row 6 is back on time instead of 0.4 s late
        self.assertEqual([round(t - 1000.0, 6) for t, _, _ in pub.sent],
                         [0.0, 0.5, 1.0, 1.5, 1.8, 2.1, 3.0])
        self.assertTrue(all(s > 0 for s in clk.sleeps))

    def test_passes_repeat_after_the_loop_gap(self) -> None:
        rows = _rows([10.0, 10.5, 11.0])
        steps = vrp.plan(rows, drop_rows={1})
        st, pub, clk, _ = self._run(steps, passes=2, loop_gap_s=4.0)
        self.assertEqual([round(t - 1000.0, 6) for t, _, _ in pub.sent], [0.0, 1.0, 5.0, 6.0])
        self.assertEqual([p for _, _, p in pub.sent], [rows[i].payload for i in (0, 2, 0, 2)])
        self.assertEqual((st.passes, st.published, st.dropped, st.dropped_rows),
                         (2, 4, 2, [(1, 1), (2, 1)]))

    def test_loop_runs_until_asked_to_stop(self) -> None:
        rows = _rows([0.0, 0.5])
        clk = FakeClock()
        pub = FakePublisher(clk)
        st = vrp.replay(vrp.plan(rows), pub, "t", loop=True, loop_gap_s=1.0, clock=clk,
                        sleep=clk.sleep, should_stop=lambda: len(pub.sent) >= 5)
        self.assertEqual(len(pub.sent), 5)
        self.assertEqual(st.passes, 3)

    def test_ctrl_c_ends_the_replay_cleanly(self) -> None:
        rows = _rows([0.0, 0.5, 1.0, 1.5])
        clk = FakeClock()
        pub = FakePublisher(clk, interrupt_after=2)
        st = vrp.replay(vrp.plan(rows), pub, "t", clock=clk, sleep=clk.sleep)
        self.assertTrue(st.interrupted)
        self.assertEqual(st.published, 2)

    def test_empty_plan_is_a_no_op(self) -> None:
        clk = FakeClock()
        st = vrp.replay([], FakePublisher(clk), "t", clock=clk, sleep=clk.sleep)
        self.assertEqual((st.passes, st.published), (0, 0))


class PahoPublisherTests(unittest.TestCase):
    def test_frames_go_out_at_qos0_and_are_never_retained(self) -> None:
        try:
            import paho.mqtt.client  # noqa: F401
        except ImportError:  # pragma: no cover
            self.skipTest("paho-mqtt not installed")
        with mock.patch("paho.mqtt.client.Client") as cls:
            inst = cls.return_value
            pub = vrp.PahoPublisher("10.0.0.9", 18830, "cid")
            pub.publish(vrp.BASE_TOPIC, b"\x01\x02")
            pub.close()
        inst.connect.assert_called_once_with("10.0.0.9", 18830, keepalive=30)
        inst.loop_start.assert_called_once()
        inst.publish.assert_called_once_with(vrp.BASE_TOPIC, b"\x01\x02", qos=0, retain=False)
        inst.loop_stop.assert_called_once()
        inst.disconnect.assert_called_once()


class CliTests(unittest.TestCase):
    def _capture(self, d: str) -> Path:
        p = Path(d) / "cap.jsonl"
        _write_capture(p, [(50.0 + 0.5 * i, _vec(i + 1, key=(i == 0))) for i in range(6)])
        return p

    def test_dry_run_prints_and_logs_the_plan(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            cap = self._capture(d)
            log = Path(d) / "replay_log.jsonl"
            out = io.StringIO()
            with redirect_stdout(out):
                rc = vrp.main([str(cap), "--dry-run", "--drop-rows", "2", "--drop-every", "5",
                               "--speed", "2", "--log", str(log)])
            lines = [json.loads(x) for x in log.read_text(encoding="utf-8").splitlines()]
        text = out.getvalue()
        self.assertEqual(rc, 0)
        self.assertIn("6 row(s); window #0-#5 = 6 frame(s), 1.2 s per pass at speed 2", text)
        self.assertIn("2 drop(s) per pass", text)
        self.assertIn("done: 1 pass(es), 4 published, 2 dropped: #2, #4", text)
        self.assertEqual([x["action"] for x in lines],
                         ["published", "published", "dropped", "published", "dropped", "published"])
        self.assertEqual([x["seq"] for x in lines], [1, 2, 3, 4, 5, 6])
        self.assertEqual([x["offset_s"] for x in lines], [0.0, 0.25, 0.5, 0.75, 1.0, 1.25])
        self.assertEqual((lines[0]["kind"], lines[0]["codec"]), (1, "vector"))
        self.assertTrue(all(x["utc"].endswith("Z") for x in lines))

    def test_usage_errors(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            cap = self._capture(d)
            with redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as cm:
                    vrp.main([str(cap), "--dry-run", "--drop-rows", "9-3"])
                self.assertEqual(cm.exception.code, 2)
                with self.assertRaises(SystemExit) as cm:
                    vrp.main([str(cap), "--dry-run", "--speed", "0"])
                self.assertEqual(cm.exception.code, 2)
                self.assertEqual(vrp.main([str(cap), "--dry-run", "--start-row", "40"]), 2)
                self.assertEqual(vrp.main([os.path.join(d, "missing.jsonl"), "--dry-run"]), 2)

    def test_unreachable_broker_exits_1(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            cap = self._capture(d)
            with mock.patch.object(vrp, "PahoPublisher", side_effect=OSError("refused")):
                with redirect_stderr(io.StringIO()) as err, redirect_stdout(io.StringIO()):
                    rc = vrp.main([str(cap), "--host", "127.0.0.1", "--port", "1"])
        self.assertEqual(rc, 1)
        self.assertIn("cannot reach the broker at 127.0.0.1:1", err.getvalue())


if __name__ == "__main__":
    unittest.main()
