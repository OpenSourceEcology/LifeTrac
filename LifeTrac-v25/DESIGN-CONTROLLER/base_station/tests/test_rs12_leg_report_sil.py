"""tools/rs12_leg_report.py — SIL tests for the RS-13.1 A14 fixes.

Pins: (1) a stale ``rx_frames=`` counter (the base's last periodic
``stats:`` line, written seconds before the log ends) is floored by the
per-frame ``published frame_id`` / ``frag_arrival`` events, with a note,
and a fresh counter is left alone; (2) ``--capture`` prints the frame loss
from TileDeltaFrame sequence gaps: the u8 wrap is not loss, a codec switch
(each encoder path numbers its own frames) is not loss, a counter restart
is a jump rather than loss, an outage longer than the seq space is counted
whole from the arrival times, and a capture that starts after seq 1 (the
FHSS acquisition of A7) reports that head separately.
"""
from __future__ import annotations

import io
import json
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path

_BS = Path(__file__).resolve().parents[1]
_TOOLS = _BS.parent / "tools"
for _p in (_BS, _TOOLS):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import rs12_leg_report as rep  # noqa: E402
from image_pipeline.frame_format import (  # noqa: E402
    CODEC_MONO_G4, CODEC_VECTOR, FRAME_KIND_DELTA, FRAME_KIND_KEY,
    TileDeltaFrame, encode_tile_delta_frame,
)

V, M = CODEC_VECTOR, CODEC_MONO_G4


def _frames(codec, seqs, t0=0.0, period=1.0):
    return [(t0 + i * period, codec, s) for i, s in enumerate(seqs)]


class SeqGapLoss(unittest.TestCase):
    def test_clean_run_has_no_loss(self) -> None:
        res = rep.seq_gap_loss(_frames(V, range(1, 11)))
        self.assertEqual((res["received"], res["lost"], res["expected"]),
                         (10, 0, 10))
        self.assertEqual(len(res["runs"]), 1)
        self.assertAlmostEqual(res["period_s"], 1.0)

    def test_missing_seqs_are_counted(self) -> None:
        seqs = [1, 2, 3, 5, 6, 9, 10]          # 4, 7 and 8 never arrived
        frames = [(float(s), V, s) for s in seqs]
        res = rep.seq_gap_loss(frames)
        self.assertEqual((res["received"], res["lost"], res["expected"]),
                         (7, 3, 10))
        self.assertEqual((res["gaps"], res["longest"]), (2, 2))

    def test_u8_wrap_is_not_loss(self) -> None:
        seqs = [250, 251, 252, 253, 254, 255, 0, 1, 2]
        res = rep.seq_gap_loss(_frames(V, seqs))
        self.assertEqual(res["lost"], 0)
        self.assertEqual(res["jumps"], 0)

    def test_loss_across_the_wrap(self) -> None:
        frames = [(0.0, V, 253), (1.0, V, 254), (4.0, V, 1), (5.0, V, 2)]
        res = rep.seq_gap_loss(frames)                 # 255 and 0 lost
        self.assertEqual((res["lost"], res["expected"]), (2, 6))

    def test_codec_switch_is_not_loss(self) -> None:
        # 2d_yt shape: mono_g4 1..63, VECTOR restarts at 1, mono_g4 resumes
        # its own counter at 64. A plain seq diff would read 194 and 184.
        frames = (_frames(M, range(1, 64))
                  + _frames(V, range(1, 137), t0=63.0)
                  + _frames(M, range(64, 100), t0=199.0))
        res = rep.seq_gap_loss(frames)
        self.assertEqual(res["lost"], 0)
        self.assertEqual(res["jumps"], 0)
        self.assertEqual([(r["codec"], r["first_seq"], r["last_seq"])
                          for r in res["runs"]],
                         [(M, 1, 63), (V, 1, 136), (M, 64, 99)])

    def test_loss_inside_each_codec_run_still_counts(self) -> None:
        frames = (_frames(M, [1, 2, 4, 5])
                  + _frames(V, [1, 2, 3, 5], t0=4.0)
                  + _frames(M, [6, 7, 9], t0=8.0))
        res = rep.seq_gap_loss(frames)
        self.assertEqual([r["lost"] for r in res["runs"]], [1, 1, 1])
        self.assertEqual(res["lost"], 3)

    def test_counter_restart_is_a_jump_not_loss(self) -> None:
        # camera_service restarted mid-leg: seq 100 -> 1 one period later
        # (a plain mod-256 step would claim 156 lost frames).
        frames = _frames(V, range(1, 101)) + _frames(V, range(1, 11), t0=100.0)
        res = rep.seq_gap_loss(frames)
        self.assertEqual((res["lost"], res["jumps"]), (0, 1))
        self.assertEqual(res["received"], 110)

    def test_outage_longer_than_the_seq_space_is_counted_whole(self) -> None:
        # 1 fps, seq 1..10, then 300 s of silence (a lock loss): the next
        # frame is seq 310 & 0xFF = 54, i.e. 299 frames never arrived.
        frames = _frames(V, range(1, 11), t0=1.0) + [(310.0, V, 310 & 0xFF)]
        res = rep.seq_gap_loss(frames)
        self.assertEqual(res["lost"], 299)
        self.assertEqual(res["jumps"], 0)

    def test_backlog_burst_after_a_loss_still_counts(self) -> None:
        # The next frame can arrive early (a queued frame flushed behind a
        # stall); a small step is loss whatever the arrival time says.
        frames = _frames(V, range(1, 6)) + [(4.3, V, 7)]
        res = rep.seq_gap_loss(frames)
        self.assertEqual((res["lost"], res["jumps"]), (1, 0))

    def test_duplicate_and_unparseable_are_not_loss(self) -> None:
        frames = [(0.0, V, 1), (1.0, V, 2), (1.1, V, 2), (1.5, None, None),
                  (2.0, V, 3)]
        res = rep.seq_gap_loss(frames)
        self.assertEqual((res["received"], res["lost"]), (3, 0))
        self.assertEqual((res["duplicates"], res["unparseable"]), (1, 1))

    def test_head_before_the_first_frame_is_reported_apart(self) -> None:
        # 2b_yt shape: FHSS acquisition, first frame heard at seq 58, then
        # one loss in 248 (RS-13.1 round 3: 0.4 % after lock).
        seqs = list(range(58, 163)) + list(range(164, 306))
        res = rep.seq_gap_loss(_frames(V, [s & 0xFF for s in seqs]))
        self.assertEqual((res["first_seq"], res["lost"], res["expected"]),
                         (58, 1, 248))
        lines = rep.format_seq_loss(res, "leg2b.jsonl", {V: "vector"})
        self.assertIn("lost 1/248 = 0.4%", lines[0])
        self.assertIn("first frame heard at seq 58", lines[1])
        self.assertIn("counting from seq 1: 58/305 = 19.0%", lines[1])

    def test_format_names_the_codec_runs(self) -> None:
        frames = _frames(M, [1, 2, 4]) + _frames(V, [1, 2], t0=3.0)
        lines = rep.format_seq_loss(rep.seq_gap_loss(frames), "x.jsonl",
                                    {M: "mono_g4", V: "vector"})
        self.assertIn("lost 1/6 = 16.7%", lines[0])
        self.assertIn("2 codec runs", lines[0])
        self.assertIn("mono_g4 seq 1..4 (3 rx, 1 lost) | vector seq 1..2",
                      lines[1])

    def test_empty_capture(self) -> None:
        lines = rep.format_seq_loss(rep.seq_gap_loss([]), "empty.jsonl")
        self.assertEqual(len(lines), 1)
        self.assertIn("no parseable frames", lines[0])


def _payload(codec: int, seq: int, key: bool = False) -> bytes:
    return encode_tile_delta_frame(TileDeltaFrame(
        frame_kind=FRAME_KIND_KEY if key else FRAME_KIND_DELTA, base_seq=seq,
        grid_w=12, grid_h=8, tile_px=32, codec=codec,
        vector_body=b"\x00" * 8 if codec == CODEC_VECTOR else b""))


def _write_capture(path: Path, rows) -> None:
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(json.dumps({"meta": {"tool": "vector_dry_run"}}) + "\n")
        for ts, payload in rows:
            fh.write(json.dumps({"ts": ts, "topic": "lifetrac/v25/video/tile_delta",
                                 "hex": payload.hex()}) + "\n")


class CaptureFile(unittest.TestCase):
    def test_capture_frames_reads_codec_and_seq(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            p = Path(d) / "cap.jsonl"
            _write_capture(p, [(10.0, _payload(V, 1, key=True)),
                               (11.0, _payload(V, 2)),
                               (12.0, b"\x07\x00"),       # does not parse
                               (13.0, _payload(M, 64))])
            frames, names = rep.capture_frames(p)
        self.assertEqual(frames, [(10.0, V, 1), (11.0, V, 2),
                                  (12.0, None, None), (13.0, M, 64)])
        self.assertEqual((names[V], names[M]), ("vector", "mono_g4"))


def _archive(d: Path, *, sent: int, rx_counter: int, published_lines: int,
             frag_lines: int = 0, frags_per_train: int = 1) -> Path:
    a = d / "radio_monitor_test"
    a.mkdir()
    (a / "params.txt").write_text("git_sha=0000000\nsynth_fps=1\n",
                                  encoding="utf-8")
    trains = sent // frags_per_train
    tx = [f"2026-10-03 21:40:{i % 60:02d},000 INFO image_tx_daemon: frame "
          f"seq={i + 1} done (pipelined): {frags_per_train} fragments ok" for i in range(trains)]
    tx.append("2026-10-03 21:45:00,000 INFO image_tx_daemon: stats: "
              f"frags_ok={sent} frags_fail=0")
    (a / "tx_daemon.log").write_text("\n".join(tx) + "\n", encoding="utf-8")
    rx = ["2026-10-03 21:45:00,000 INFO image_rx_daemon: stats: "
          f"rx_frames={rx_counter} rx_decode_err=0 "
          f"frames_published={rx_counter} publish_err=0 "
          "reassembler_decode_err=0 reassembler_timeouts=0"]
    rx += [f"2026-10-03 21:45:0{i % 10},000 INFO image_rx_daemon: frag_arrival: "
           f"seq={i + 1} idx=0 total=1 fw_us=1 len=245" for i in range(frag_lines)]
    rx += [f"2026-10-03 21:45:0{i % 10},000 INFO image_rx_daemon: published "
           f"frame_id=0 seq={i + 1} 241 B -> lifetrac/v25/video/tile_delta"
           for i in range(published_lines)]
    (a / "rx_daemon.log").write_text("\n".join(rx) + "\n", encoding="utf-8")
    return a


def _run(argv) -> str:
    buf = io.StringIO()
    with redirect_stdout(buf):
        rc = rep.main([str(x) for x in argv])
    assert rc == 0
    return buf.getvalue()


class StaleRxCounter(unittest.TestCase):
    def test_stale_counter_is_floored_by_published_events(self) -> None:
        # Round-3 shape: the last stats: line said 8, the log published 10.
        with tempfile.TemporaryDirectory() as d:
            out = _run([_archive(Path(d), sent=10, rx_counter=8,
                                 published_lines=10)])
        self.assertIn("(rx rx_frames counter 8 is stale; using 10 frames "
                      "from 'published frame_id' events)", out)
        self.assertIn("loss 0/10 = 0.0%", out)
        self.assertIn("published=10", out)

    def test_batched_trains_do_not_floor_with_frame_counts(self) -> None:
        # -TxBatch 1: 5 trains of 2 fragments = 10 sent; 12 frames published
        # (several per train) must not be read as 12 fragments received.
        with tempfile.TemporaryDirectory() as d:
            out = _run([_archive(Path(d), sent=10, rx_counter=10,
                                 published_lines=12, frags_per_train=2)])
        self.assertNotIn("rx_frames counter", out)
        self.assertIn("loss 0/10 = 0.0%", out)

    def test_received_above_sent_is_clamped_with_a_warning(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            out = _run([_archive(Path(d), sent=10, rx_counter=12,
                                 published_lines=12)])
        self.assertIn("received 12 > sent 10", out)
        self.assertIn("loss 0/10 = 0.0%", out)
        self.assertNotIn("loss -", out)

    def test_fresh_counter_is_kept(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            out = _run([_archive(Path(d), sent=10, rx_counter=9,
                                 published_lines=9)])
        self.assertNotIn("rx_frames counter", out)
        self.assertIn("loss 1/10 = 10.0%", out)

    def test_frag_arrival_events_win_when_larger(self) -> None:
        # A fragment that arrived but never completed a frame is still a
        # received fragment: the fragment-level event count is the floor.
        with tempfile.TemporaryDirectory() as d:
            out = _run([_archive(Path(d), sent=10, rx_counter=7,
                                 published_lines=8, frag_lines=9)])
        self.assertIn("(rx rx_frames counter 7 is stale; using 9 fragments "
                      "from frag_arrival events)", out)
        self.assertIn("loss 1/10 = 10.0%", out)

    def test_capture_option_prints_the_seq_gap_loss(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            arch = _archive(Path(d), sent=6, rx_counter=5, published_lines=5)
            cap = Path(d) / "leg2x_base.jsonl"
            _write_capture(cap, [(float(s), _payload(V, s, key=(s == 1)))
                                 for s in (1, 2, 3, 5, 6)])
            out = _run([arch, "--capture", cap])
        self.assertIn("capture seq gaps (leg2x_base.jsonl): lost 1/6 = 16.7%",
                      out)

    def test_without_capture_no_seq_line(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            out = _run([_archive(Path(d), sent=4, rx_counter=4,
                                 published_lines=4)])
        self.assertNotIn("capture seq gaps", out)


if __name__ == "__main__":
    unittest.main()
