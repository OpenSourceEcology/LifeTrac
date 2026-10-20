"""tools/rs12_leg_report.py — SIL tests for the RS-13.1 A14 fixes.

Pins: (1) a stale ``rx_frames=`` counter (the base's last periodic
``stats:`` line, written seconds before the log ends) is floored by the
per-frame ``published frame_id`` / ``frag_arrival`` events, with a note,
and a fresh counter is left alone; (2) ``--capture`` prints the frame loss
from TileDeltaFrame sequence gaps: the u8 wrap is not loss, a codec switch
(each encoder path numbers its own frames) is not loss, a counter restart
is a jump rather than loss, an outage longer than the seq space is counted
whole from the arrival times, and a capture that starts after seq 1 (the
FHSS acquisition of A7) reports that head separately; (3) 2026-10-19: the
loss over the window both logs cover (``txdone_arrival`` trains matched to
``frag_arrival`` trains, RS-12.15 09-14 ERRATA §3) is printed beside the
whole-log line, which is kept and labelled; the FIRMWARE DROP line uses the
base's own count, not the one clamped to the TX count, and says when the
logs were read before the daemons stopped (no ``log_window=after_stop``);
on the committed archives the shared-window figures are ERRATA §3's.
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
        # no per-fragment events: the old hint, unchanged
        self.assertIn("whole-log loss shown as 0 — check the seq-gap figure", out)

    def test_received_above_sent_with_capture_names_the_capture_line(self) -> None:
        # no per-fragment events (no shared window), but a capture: the
        # warning names the capture seq-gap line alone
        with tempfile.TemporaryDirectory() as d:
            arch = _archive(Path(d), sent=10, rx_counter=12, published_lines=12)
            cap = Path(d) / "leg2x_base.jsonl"
            _write_capture(cap, [(float(s), _payload(V, s, key=(s == 1)))
                                 for s in range(1, 11)])
            out = _run([arch, "--capture", cap])
        warn = next(ln for ln in out.splitlines() if "received 12 > sent 10" in ln)
        self.assertTrue(warn.endswith("— check the capture seq-gap line "
                                      "(the loss of record)"), warn)

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

    def test_whole_log_line_is_labelled_and_unchanged(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            out = _run([_archive(Path(d), sent=10, rx_counter=9,
                                 published_lines=9)])
        line = next(ln for ln in out.splitlines() if ln.startswith("loss "))
        self.assertEqual(line, "loss 1/10 = 10.0%   crc_dumps=0  timeouts=0  "
                               "published=9   [whole logs]")
        # this archive logs no per-fragment events: no shared window
        self.assertIn("shared-window loss: n/a", out)


# -- shared-window loss (RS-12.15 09-14 ERRATA §3) -----------------------------
def _tx_log(trains) -> str:
    """``trains``: ``(seq, [idx, ...])`` or ``(seq, [idx, ...], done)``; a
    train with ``done`` False has no "K fragments ok" line (the TX log was
    read mid-train)."""
    out = []
    for t in trains:
        seq, idxs, done = (t + (True,))[:3]
        out += [f"2026-09-01 13:10:57,839 INFO image_tx_daemon: txdone_arrival: "
                f"seq={seq} idx={i} status=0 toa_us=169088" for i in idxs]
        if done:
            out.append(f"2026-09-01 13:10:58,000 INFO image_tx_daemon: frame "
                       f"seq={seq} done (pipelined): {len(idxs)} fragments ok")
    sent = sum(len(t[1]) for t in trains if (t + (True,))[2])
    out.append("2026-09-01 13:11:00,000 INFO image_tx_daemon: stats: "
               f"frags_ok={sent} frags_fail=0")
    return "\n".join(out) + "\n"


def _rx_log(trains) -> str:
    out = [f"2026-09-14 21:05:12,075 INFO image_rx_daemon: frag_arrival: "
           f"seq={seq} idx={i} total=15 fw_us=1 len=207"
           for seq, idxs in trains for i in idxs]
    n = sum(len(i) for _, i in trains)
    out.append("2026-09-14 21:05:20,000 INFO image_rx_daemon: stats: "
               f"rx_frames={n} rx_decode_err=0 frames_published={len(trains)} "
               "publish_err=0 reassembler_decode_err=0 reassembler_timeouts=0")
    return "\n".join(out) + "\n"


def _frag_archive(d: Path, tx_trains, rx_trains, params: str = "") -> Path:
    a = d / "radio_monitor_frag"
    a.mkdir()
    # the harness writes params.txt with Out-File -Encoding utf8: a BOM
    (a / "params.txt").write_text("﻿git_sha=0000000\n" + params,
                                  encoding="utf-8")
    (a / "tx_daemon.log").write_text(_tx_log(tx_trains), encoding="utf-8")
    (a / "rx_daemon.log").write_text(_rx_log(rx_trains), encoding="utf-8")
    return a


def _brackets(d: Path, rx_ok: int) -> list:
    pre, post = d / "pre.txt", d / "post.txt"
    pre.write_text("radio_dio0=0 radio_rx_ok=0 radio_crc_err=0 radio_tx_ok=0\n",
                   encoding="utf-8")
    post.write_text(f"radio_dio0={rx_ok + 3} radio_rx_ok={rx_ok} "
                    "radio_crc_err=0 radio_tx_ok=3\n", encoding="utf-8")
    return ["--pre", pre, "--post", post]


ALL = list(range(15))


class SharedWindowLoss(unittest.TestCase):
    def _res(self, tx_trains, rx_trains, **kw):
        return rep.shared_window_loss(_tx_log(tx_trains), _rx_log(rx_trains),
                                      **kw)

    def test_clean_logs_have_no_loss(self) -> None:
        res = self._res([(1, ALL), (2, ALL)], [(1, ALL), (2, ALL)])
        self.assertEqual((res["sent"], res["decoded"], res["lost"]),
                         (30, 30, 0))
        self.assertEqual((res["trains"], res["unheard"], res["late"],
                          res["extra"]), (2, 0, 0, 0))

    def test_rx_past_the_tx_logs_last_line_is_left_out(self) -> None:
        # 09-14 leg R shape: the TX log ends at seq 3 idx 11 of 15; the base
        # decodes the rest of seq 3 and two more trains. Whole-log totals
        # read 15 + 15 + 12 = 42 sent against 15 + 14 + 15 + 30 = 74 decoded.
        tx = [(1, ALL), (2, ALL), (3, list(range(12)), False)]
        rx = [(1, ALL), (2, [i for i in ALL if i != 7]), (3, ALL), (4, ALL),
              (5, ALL)]
        res = self._res(tx, rx)
        self.assertEqual((res["sent"], res["lost"]), (42, 1))
        self.assertEqual((res["late"], res["extra"]), (30, 3))

    def test_a_train_never_heard_is_lost_whole(self) -> None:
        res = self._res([(1, [0]), (2, [0]), (3, [0]), (4, [0])],
                        [(1, [0]), (3, [0]), (4, [0])])
        self.assertEqual((res["sent"], res["lost"], res["unheard"]), (4, 1, 1))

    def test_the_lookahead_bounds_the_match(self) -> None:
        # A seq 9 sits five RX trains on: outside the 4-train window, so the
        # TX train is lost and the trains in between still match. A wider
        # window would match across and lose the four trains it skips.
        tx = [(9, [0]), (1, [0]), (2, [0]), (3, [0]), (4, [0]), (5, [0])]
        rx = [(1, [0]), (2, [0]), (3, [0]), (4, [0]), (9, [0]), (5, [0])]
        res = self._res(tx, rx)
        self.assertEqual((res["lost"], res["unheard"]), (1, 1))
        self.assertEqual(self._res(tx, rx, lookahead=5)["lost"], 4)

    def test_the_u8_wrap_matches(self) -> None:
        # the TX side numbering past 255 still matches the base's u8 seq
        res = self._res([(255, [0, 1]), (256, [0, 1]), (257, [0, 1])],
                        [(255, [0, 1]), (0, [0, 1]), (1, [1])])
        self.assertEqual((res["sent"], res["lost"]), (6, 1))

    def test_only_status_0_completions_count_as_sent(self) -> None:
        tx = ("x txdone_arrival: seq=1 idx=0 status=0 toa_us=1\n"
              "x txdone_arrival: seq=1 idx=1 status=1 toa_us=1\n"   # retried
              "x txdone_arrival: seq=1 idx=1 status=0 toa_us=1\n"
              "x txdone_arrival: seq=2 idx=0 status=4 toa_us=1\n")  # given up
        res = rep.shared_window_loss(tx, _rx_log([(1, [0, 1])]))
        self.assertEqual((res["sent"], res["lost"], res["trains"]), (2, 0, 1))

    def test_missing_events_give_none_and_an_na_line(self) -> None:
        self.assertIsNone(rep.shared_window_loss("", _rx_log([(1, [0])])))
        self.assertIsNone(rep.shared_window_loss(_tx_log([(1, [0])]), ""))
        lines = rep.format_shared_loss(None)
        self.assertEqual(len(lines), 1)
        self.assertIn("shared-window loss: n/a", lines[0])

    def test_format(self) -> None:
        res = {"sent": 1212, "decoded": 1207, "lost": 5, "trains": 81,
               "unheard": 0, "late": 25, "extra": 3}
        lines = rep.format_shared_loss(res)
        self.assertTrue(lines[0].startswith("shared-window loss 5/1212 = 0.4%"))
        self.assertIn("0 of 81 trains not heard", lines[0])
        self.assertIn("25 fragment(s) the base decoded after the TX log's last "
                      "train", lines[1])
        self.assertIn("3 decoded in a matched train", lines[1])
        res.update(late=0, extra=0)
        self.assertEqual(len(rep.format_shared_loss(res)), 1)


class SharedWindowReport(unittest.TestCase):
    """The archive report: R/O shape, where the RX log outruns the TX log."""

    def _archive(self, d: Path, params: str = "") -> Path:
        # 3 trains of 15 on the TX log, one fragment of seq 2 lost, and the
        # base decodes one more train after the TX log's last line: 45 sent,
        # 44 + 15 = 59 decoded on the whole logs.
        tx = [(1, ALL), (2, ALL), (3, ALL)]
        rx = [(1, ALL), (2, ALL[1:]), (3, ALL), (4, ALL)]
        return _frag_archive(d, tx, rx, params)

    def test_shared_window_is_printed_beside_the_clamped_whole_log_line(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            out = _run([self._archive(Path(d))])
        lines = out.splitlines()
        self.assertIn("received 59 > sent 45", out)
        self.assertIn("read the shared-window line", out)
        i = next(n for n, ln in enumerate(lines) if ln.startswith("loss "))
        self.assertEqual(lines[i], "loss 0/45 = 0.0%   crc_dumps=0  timeouts=0  "
                                   "published=4   [whole logs]")
        self.assertTrue(lines[i + 1].startswith("shared-window loss 1/45 = 2.2%"),
                        lines[i + 1])
        self.assertIn("left out: 15 fragment(s)", lines[i + 2])

    def test_with_capture_the_warning_points_at_the_loss_of_record(self) -> None:
        # RS-13: with --capture the capture seq-gap line is the loss of
        # record; the received > sent warning must name it, not only the
        # shared-window line.
        with tempfile.TemporaryDirectory() as d:
            arch = self._archive(Path(d))
            cap = Path(d) / "leg2x_base.jsonl"
            _write_capture(cap, [(float(s), _payload(V, s, key=(s == 1)))
                                 for s in (1, 2, 3, 4)])
            out = _run([arch, "--capture", cap])
        warn = next(ln for ln in out.splitlines() if "received 59 > sent 45" in ln)
        self.assertIn("check the capture seq-gap line (the loss of record) "
                      "and the shared-window line", warn)
        self.assertIn("capture seq gaps (leg2x_base.jsonl): lost 0/4", out)

    def test_firmware_drop_uses_the_unclamped_count(self) -> None:
        # drx_ok 61: the base log holds 59, so the drop is 2 -- not 61 - 45.
        with tempfile.TemporaryDirectory() as d:
            arch = self._archive(Path(d))
            out = _run([arch] + _brackets(Path(d), 61))
        self.assertIn("FIRMWARE DROP (drx_ok - host URCs): 61 - 59 = 2", out)
        self.assertIn("(log window: params.txt has no log_window=after_stop", out)

    def test_no_log_window_note_when_the_logs_were_read_after_the_stop(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            arch = self._archive(Path(d), params="log_window=after_stop\n")
            out = _run([arch] + _brackets(Path(d), 59))
        self.assertIn("FIRMWARE DROP (drx_ok - host URCs): 59 - 59 = 0", out)
        self.assertNotIn("log window:", out)
        self.assertIn("log_window=after_stop", out.splitlines()[0])

    def test_another_log_window_value_still_gets_the_note(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            arch = self._archive(Path(d), params="log_window=before_stop\n")
            out = _run([arch] + _brackets(Path(d), 61))
        self.assertIn("(log window: params.txt has no log_window=after_stop", out)


# -- committed evidence ---------------------------------------------------------
_EV = _BS.parent / "bench-evidence"

# RS_12_15_clock_authority_2026-09-14/ERRATA.md §3: (lost, sent) over the
# shared window, and the whole-log "current report" figure (None where the
# report printed "none" because received > sent). The 10-18 legs V, W, W2,
# X are §3's comparison row; the 10-19 legs are RS-13 range-edge Round A.
ERRATA_S3 = {
    "radio_monitor_20260914_113052_3a0cb524": ("L", (95, 1212), (81, 1200)),
    "radio_monitor_20260914_130207_23ba5122": ("O", (27, 1220), None),
    "radio_monitor_20260914_130945_23ba5122": ("P", (69, 1221), (34, 1215)),
    "radio_monitor_20260914_153752_6feca2c0": ("Q", (22, 868), (8, 868)),
    "radio_monitor_20260914_154752_6feca2c0": ("Q2", (82, 1112), (60, 1108)),
    "radio_monitor_20260914_161016_6feca2c0": ("R", (5, 1212), None),
    "radio_monitor_20260914_170031_a75089c0": ("S", (307, 763), (306, 762)),
    "radio_monitor_20260914_171517_a75089c0": ("T", (170, 770), (161, 769)),
    "radio_monitor_20260915_130259_aa74cde2": ("U", (73, 674), (71, 674)),
    "radio_monitor_20261018_110651_d484a92d": ("V", (7, 949), (6, 949)),
    "radio_monitor_20261018_111558_d484a92d": ("W", (131, 1216), (129, 1215)),
    "radio_monitor_20261018_112639_d484a92d": ("W2", (49, 1232), (45, 1230)),
    "radio_monitor_20261018_113831_d484a92d": ("X", (131, 2010), (130, 2010)),
    "radio_monitor_20261019_134805_1d10f57f": ("A2c", (0, 611), None),
    "radio_monitor_20261019_135659_1d10f57f": ("A2a", (0, 607), None),
    "radio_monitor_20261019_140618_1d10f57f": ("A2c2", (1, 609), (0, 609)),
    "radio_monitor_20261019_141603_1d10f57f": ("A2a2", (1, 612), (0, 612)),
}


class CommittedArchives(unittest.TestCase):
    def test_errata_section_3_figures(self) -> None:
        seen = 0
        for name, (leg, (lost, sent), whole) in ERRATA_S3.items():
            arch = _EV / name
            if not (arch / "tx_daemon.log").is_file():
                continue
            seen += 1
            with self.subTest(leg=leg):
                out = _run([arch])
                self.assertIn(f"shared-window loss {lost}/{sent} = "
                              f"{100 * lost / sent:.1f}%", out)
                line = next(ln for ln in out.splitlines()
                            if ln.startswith("loss "))
                if whole is None:
                    self.assertIn(" > sent ", out)
                    self.assertTrue(line.startswith("loss 0/"), line)
                else:
                    self.assertTrue(line.startswith(
                        f"loss {whole[0]}/{whole[1]} = "), line)
                self.assertTrue(line.endswith("[whole logs]"), line)
        if not seen:
            self.skipTest("no committed archives in this tree")

    def test_a2c_firmware_drop_is_the_log_window(self) -> None:
        arch = _EV / "radio_monitor_20261019_134805_1d10f57f"
        legs = _EV / "RS_13_range_edge_2026-10-19" / "legs"
        if not (arch / "tx_daemon.log").is_file() or not (legs / "legA2c_post_base.txt").is_file():
            self.skipTest("A2c archive or brackets not in this tree")
        out = _run([arch, "--pre", legs / "legA2c_pre_base.txt",
                    "--post", legs / "legA2c_post_base.txt"])
        # was 617 - 611 = 6 (the base's 612 clamped to the TX log's 611)
        self.assertIn("FIRMWARE DROP (drx_ok - host URCs): 617 - 612 = 5", out)
        self.assertIn("(log window: params.txt has no log_window=after_stop", out)


if __name__ == "__main__":
    unittest.main()
