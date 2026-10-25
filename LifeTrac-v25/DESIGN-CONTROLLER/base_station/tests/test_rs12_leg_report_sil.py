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
on the committed archives the shared-window figures are ERRATA §3's;
(4) 2026-10-22: only the exact ``log_window=after_stop`` drops that note,
and ``log_window=after_stop_unverified`` (a daemon stop or a SIGTERM
shutdown line the harness could not confirm) gets its own note quoting
``log_window_detail=``, worded by reason and printed below the loss block
(leg_post.sh shows only the report's first 8 lines); (5) 2026-10-24 (the
FHSS round's leg 6): the capture's head is placed on the tractor's TX log
(params.txt clock offsets, else the daemons' start times) and reads
AMBIGUOUS when it cannot be, a head after a u8 wrap counts the wrap, the
``tractor TX log`` line counts every frame sent; a capture with no clean +1
step times its gaps by the leg's synth_fps (226/231, not 91/96 with a
jump); the ``acquisition:`` line against the 120 s scan budget (a first
frame at exactly 120 s is within it), its ``!! ACQUISITION FAILED`` flag
(also on the first line, inside leg_post.sh's 8-line excerpt), the
RX_SCAN_FAILED sub-byte read as the daemons' decoder reads it (the 0x80
marker, the daemon's name= both ways, AMBIGUOUS on the final line), base
FAULT lines in the tractor's format and archives without them; on the committed
10-24 legs every old line still prints and leg 6 reads frame #264, 601/606.
PR #166 review: the head is placed by the frame's transport seq (the
published line at its capture time), AMBIGUOUS when it cannot be read or
differs from the TileDeltaFrame seq, and for every unplaced first frame
(seq 1, seq 0 too); the ``tractor TX log`` share is withheld on a leg that
may batch; the ``acquisition:`` line is the first image reception and the
verdict under it rests on positive evidence (a cold or early final
RX_SCAN_FAILED, the always-on stats counter or per-fragment logging shown on
for FAILED; a bracket demotion or the share decoded for ACQUIRED; NOT
DETERMINED otherwise); legs 1-5 of 10-24 read ACQUIRED, leg 6 FAILED.
"""
from __future__ import annotations

import io
import json
import re
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from datetime import datetime, timedelta, timezone
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
        # one loss in 248 (RS-13.1 round 3: 0.4 % after lock). 2026-10-24:
        # the figure needs the TX log to place seq 58 in the first u8 cycle.
        seqs = list(range(58, 163)) + list(range(164, 306))
        res = rep.seq_gap_loss(_frames(V, [s & 0xFF for s in seqs]))
        self.assertEqual((res["first_seq"], res["lost"], res["expected"]),
                         (58, 1, 248))
        place = {"sent": 305, "how": "offsets",
                 "first": {"frame": 58, "wraps": 0, "dt": 0.03},
                 "last": {"frame": 305, "wraps": 1, "dt": 0.02}}
        lines = rep.format_seq_loss(res, "leg2b.jsonl", {V: "vector"}, place)
        self.assertIn("lost 1/248 = 0.4%", lines[0])
        self.assertEqual(
            lines[1], "  first frame heard at seq 58: the 57 frame(s) numbered "
            "before it are not in the figure above (the camera numbers from seq "
            "1 at start-up — an acquisition delay or a late capture, RS-13.1 "
            "A7); counting from seq 1: 58/305 = 19.0%")
        self.assertEqual(
            lines[2], "  tractor TX log: 305 frame(s) on air (txdone_arrival); "
            "the capture's first frame is its #58, its last #305 (placed by the "
            "params.txt clock offsets, within 0.03 s): 58/305 = 19.0% of them "
            "never reached the capture (57 before its first, 0 after its last)")

    def test_an_unplaced_head_is_ambiguous_not_a_figure(self) -> None:
        # 2026-10-24 leg 6: the first frame heard, seq 8, was the tractor's
        # frame #264; without the TX log no head figure is printed
        res = rep.seq_gap_loss(_frames(V, [8, 9, 10]))
        for place in (None, {"why": "no leg2x_params.txt beside the capture"},
                      {"sent": 606, "how": "starts",
                       "first": {"why": "the daemons' start times fit 2"},
                       "last": {"why": "the first frame was not placed"}}):
            lines = rep.format_seq_loss(res, "x.jsonl", {V: "vector"}, place)
            self.assertEqual(len(lines), 2, lines)
            self.assertTrue(lines[1].startswith(
                "  first frame heard at seq 8: the head is AMBIGUOUS — 7 frame(s) "
                "numbered before it if seq 8 is in the camera's first u8 cycle, "
                "256 more per wrap"), lines[1])
            self.assertNotIn("counting from seq 1:", lines[1])
        self.assertIn("(no tractor TX log was given to place it on)",
                      rep.format_seq_loss(res, "x.jsonl", None, None)[1])
        self.assertIn("(the daemons' start times fit 2)", lines[1])
        # review 2026-10-24: every unplaced first frame is AMBIGUOUS, however small
        # its seq: seq 1 can be frame #257, seq 0 is frame #256 at the earliest
        for s0, least in ((1, "0 frame(s) numbered before it if seq 1 is in the camera's "
                                "first u8 cycle"),
                          (0, "at least 255 frame(s) numbered before it (seq 0 is frame #256 "
                              "at the earliest)"),
                          (2, "1 frame(s) numbered before it if seq 2 is in the camera's "
                              "first u8 cycle")):
            lines = rep.format_seq_loss(rep.seq_gap_loss(_frames(V, [s0, s0 + 1])), "x.jsonl")
            self.assertEqual(len(lines), 2, lines)
            self.assertEqual(
                lines[1], f"  first frame heard at seq {s0}: the head is AMBIGUOUS — {least}, "
                          f"256 more per wrap, and it cannot be placed on the tractor's TX log "
                          f"(no tractor TX log was given to place it on); no figure counting "
                          f"from seq 1 (RS-13.1 A7)")
        # placed in the first cycle, seq 1 has nothing before it: no line; a wrap
        # before it is counted
        res = rep.seq_gap_loss(_frames(V, [1, 2]))
        first = {"frame": 1, "wraps": 0, "dt": 0.01, "rx_seq": 1, "rx_line": 9, "tdf_seq": 1}
        place = {"sent": 2, "how": "offsets", "first": first, "batched": None,
                 "last": dict(first, frame=2, rx_seq=2, tdf_seq=2)}
        lines = rep.format_seq_loss(res, "x.jsonl", None, place)
        self.assertFalse(any("first frame heard" in ln for ln in lines), lines)
        place["first"] = dict(first, frame=257, wraps=1)
        place["last"] = dict(first, frame=258, wraps=1, rx_seq=2, tdf_seq=2)
        place["sent"] = 258
        lines = rep.format_seq_loss(res, "x.jsonl", None, place)
        self.assertTrue(lines[1].startswith("  first frame heard at seq 1 is the tractor's frame "
                                            "#257 on air (1 u8 seq wrap before it"), lines[1])

    def test_a_head_after_a_wrap_counts_the_wrap(self) -> None:
        # leg 6's shape: seq 8 is frame #264 (one wrap), 226 lost after it
        res = rep.seq_gap_loss([(0.0, V, 8), (14.6, V, 37), (40.9, V, 90),
                                (46.9, V, 102), (115.0, V, 238)], period_s=0.5)
        place = {"sent": 606, "how": "offsets",
                 "first": {"frame": 264, "wraps": 1, "dt": 0.033},
                 "last": {"frame": 494, "wraps": 1, "dt": -0.004}}
        lines = rep.format_seq_loss(res, "leg2b_f3_base.jsonl", {V: "vector"}, place)
        self.assertIn("lost 226/231 = 97.8%", lines[0])
        self.assertIn("period 0.50 s (the leg's synth_fps: no clean +1 step to "
                      "time it by)", lines[0])
        self.assertEqual(
            lines[1], "  first frame heard at seq 8 is the tractor's frame #264 "
            "on air (1 u8 seq wrap before it, by its TX log): the 263 frame(s) "
            "numbered before it are not in the figure above (the camera numbers "
            "from seq 1 at start-up — an acquisition delay or a late capture, "
            "RS-13.1 A7); counting from seq 1: 489/494 = 99.0%")
        self.assertIn("the capture's first frame is its #264, its last #494 "
                      "(placed by the params.txt clock offsets, within 0.03 s): "
                      "601/606 = 99.2% of them never reached the capture (263 "
                      "before its first, 112 after its last)", lines[2])
        self.assertEqual(len(lines), 3, lines)        # no jump line any more
        # two wraps, and a last frame the TX log cannot place
        place = dict(place, first={"frame": 520, "wraps": 2, "skew": 0.6}, how="starts",
                     last={"why": "it came after the TX log's last frame (#606)"})
        lines = rep.format_seq_loss(res, "x.jsonl", None, place)
        self.assertIn("is the tractor's frame #520 on air (2 u8 seq wraps before "
                      "it, by its TX log): the 519 frame(s)", lines[1])
        self.assertEqual(
            lines[2], "  tractor TX log: 606 frame(s) on air (txdone_arrival); "
            "the capture's first frame is its #520 (placed by the daemons' start "
            "times: the RX log starts 0.6 s after the TX log on the PC clock); its "
            "last could not be placed (it came after the TX log's last frame "
            "(#606))")

    def test_nominal_period_counts_a_long_step_without_clean_steps(self) -> None:
        # leg 6: 5 frames, no two consecutive; 102 -> 238 in 68 s is 136
        # frames at 2 fps, not a counter restart
        frames = [(0.0, V, 8), (14.565, V, 37), (40.895, V, 90),
                  (46.914, V, 102), (114.96, V, 238)]
        old = rep.seq_gap_loss(frames)
        self.assertEqual((old["lost"], old["jumps"], old["period_s"]), (91, 1, None))
        res = rep.seq_gap_loss(frames, period_s=0.5)
        self.assertEqual((res["lost"], res["expected"], res["jumps"]), (226, 231, 0))
        self.assertEqual((res["period_s"], res["period_src"]), (0.5, "nominal"))
        # a real restart one period later is still a jump with the nominal period
        res = rep.seq_gap_loss([(0.0, V, 8), (14.5, V, 37), (15.0, V, 1)], period_s=0.5)
        self.assertEqual((res["lost"], res["jumps"]), (28, 1))
        # clean +1 steps win over the nominal period: output as before
        frames = _frames(V, range(1, 11))
        self.assertEqual(rep.seq_gap_loss(frames, period_s=0.25),
                         dict(rep.seq_gap_loss(frames)))
        self.assertEqual(rep.seq_gap_loss(frames, period_s=0.25)["period_src"], "steps")
        self.assertEqual(rep.nominal_period({"synth_fps": "2"}), 0.5)
        for bad in ({}, {"synth_fps": ""}, {"synth_fps": "0"}, {"synth_fps": "x"}):
            self.assertIsNone(rep.nominal_period(bad), bad)

    def test_format_names_the_codec_runs(self) -> None:
        frames = _frames(M, [1, 2, 4]) + _frames(V, [1, 2], t0=3.0)
        lines = rep.format_seq_loss(rep.seq_gap_loss(frames), "x.jsonl",
                                    {M: "mono_g4", V: "vector"})
        self.assertIn("lost 1/6 = 16.7%", lines[0])
        self.assertIn("2 codec runs", lines[0])
        # (lines[1]: no TX log to place the head on, so it reads AMBIGUOUS)
        self.assertIn("mono_g4 seq 1..4 (3 rx, 1 lost) | vector seq 1..2",
                      lines[2])

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
        self.assertIn("whole-log loss shown as 0 — re-run with --capture "
                      "<leg>_base.jsonl for the seq-gap figure", out)

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


def _timed_log(daemon: str, event: str, trains, total: int | None = None,
               period: float = 0.5) -> str:
    """One line per fragment, train n at n * period s: ``trains`` holds
    ``(n, [idx, ...])``; the logged seq is n mod 256 (the u8 wire seq)."""
    t0 = datetime(2026, 10, 19, 19, 0, 0)
    out = []
    for n, idxs in trains:
        ts = (t0 + timedelta(seconds=n * period)).strftime("%Y-%m-%d %H:%M:%S,%f")[:-3]
        for i in idxs:
            tail = (" status=0 toa_us=98624" if event == "txdone_arrival"
                    else f" total={total if total is not None else 1} fw_us=1 len=244")
            out.append(f"{ts} INFO {daemon}: {event}: seq={n % 256} idx={i}{tail}")
    return "\n".join(out) + "\n"


def _tx_t(trains) -> str:
    return _timed_log("image_tx_daemon", "txdone_arrival", trains)


def _rx_t(trains, total: int | None = None) -> str:
    return _timed_log("image_rx_daemon", "frag_arrival", trains, total)


class SharedWindowGuards(unittest.TestCase):
    """PR #161 review: fail closed where the two logs cannot be aligned."""

    def test_an_outage_across_a_full_wrap_is_withheld(self) -> None:
        # 258 one-fragment trains; the base hears only trains 1, 257 and 258,
        # whose wire seqs read 1, 1, 2. Grouping by seq alone merged the first
        # two and reported 256/258 lost (true: 255/258).
        tx = _tx_t([(n, [0]) for n in range(1, 259)])
        rx = _rx_t([(1, [0]), (257, [0]), (258, [0])])
        res = rep.shared_window_loss(tx, rx)
        self.assertIn("wrap off", res["withheld"])
        line = rep.format_shared_loss(res)
        self.assertEqual(len(line), 1)
        self.assertTrue(line[0].startswith("shared-window loss: n/a, withheld ("))
        # the same on a longer leg: the base hears trains 1-5 and 261-265, so
        # wire seq 5 comes round again 128 s after the first seq 5
        tx = _tx_t([(n, [0]) for n in range(1, 266)])
        rx = _rx_t([(n, [0]) for n in list(range(1, 6)) + list(range(261, 266))])
        res = rep.shared_window_loss(tx, rx)
        self.assertIn("wrap off", res["withheld"])

    def test_a_long_outage_inside_one_wrap_is_still_scored(self) -> None:
        # 129 trains (64.5 s) unheard between seq 10 and seq 140, inside one
        # wrap: every match has the same log offset, so every miss is counted.
        tx = _tx_t([(n, [0]) for n in range(1, 151)])
        rx = _rx_t([(n, [0]) for n in list(range(1, 11)) + list(range(140, 151))])
        res = rep.shared_window_loss(tx, rx)
        self.assertIsNone(res["withheld"])
        self.assertEqual((res["sent"], res["lost"], res["unheard"]), (150, 129, 129))

    def test_two_copies_received_are_withheld_not_half_lost(self) -> None:
        # a one-fragment key frame sent with copies=2: the TX log numbers the
        # transmissions 0 and 1, the base logs the header's idx 0 twice. The
        # idx overlap read 1/2 lost on a lossless link.
        tx = _tx_t([(1, [0]), (2, [0, 1]), (3, [0])])
        rx = _rx_t([(1, [0]), (2, [0, 0]), (3, [0])], total=1)
        res = rep.shared_window_loss(tx, rx)
        self.assertIn("key-frame copies", res["withheld"])
        self.assertIn("n/a, withheld", rep.format_shared_loss(res)[0])

    def test_a_parity_fragment_is_withheld(self) -> None:
        # two data fragments plus an XOR parity frame (0xFC, not logged by the
        # base): TX idx 2 is past the header's 2 fragments
        tx = _tx_t([(1, [0, 1, 2]), (2, [0, 1, 2])])
        rx = _rx_t([(1, [0, 1]), (2, [0, 1])], total=2)
        res = rep.shared_window_loss(tx, rx)
        self.assertIn("TX idx [2] at or past the header's 2 fragment(s)", res["withheld"])

    def test_a_plain_lossless_leg_is_not_withheld(self) -> None:
        tx = _tx_t([(n, [0]) for n in range(1, 400)])
        rx = _rx_t([(n, [0]) for n in range(1, 400)])
        res = rep.shared_window_loss(tx, rx)
        self.assertIsNone(res["withheld"])
        self.assertEqual((res["sent"], res["lost"]), (399, 0))


def _two_clock_logs(trains, heard, frags: int = 1,
                    rx_t0: datetime | None = None) -> tuple[str, str]:
    """TX and RX logs on two board clocks days apart (as on the bench):
    ``trains`` holds ``(u, t)`` (u = the TX seq before the u8 wrap, t = s
    from the start), ``heard(n)`` says whether the base decoded train n."""
    t0 = datetime(2026, 10, 19, 19, 0, 0)
    rx_t0 = rx_t0 or t0 + timedelta(days=13, hours=8)

    def f(base, t):
        return (base + timedelta(seconds=t)).strftime("%Y-%m-%d %H:%M:%S,%f")[:-3]
    tx, rx = [], []
    for n, (u, t) in enumerate(trains):
        for i in range(frags):
            tx.append(f"{f(t0, t + 0.21 * i)} INFO image_tx_daemon: txdone_arrival: "
                      f"seq={u % 256} idx={i} status=0 toa_us=1")
            if heard(n):
                rx.append(f"{f(rx_t0, t + 0.21 * i + 0.25)} INFO image_rx_daemon: "
                          f"frag_arrival: seq={u % 256} idx={i} total={frags} "
                          f"fw_us=1 len=1")
    return "\n".join(tx) + "\n", "\n".join(rx) + "\n"


class SharedWindowAlignment(unittest.TestCase):
    """The wrap guard checks the matches the scorer made: the boards' clocks
    differ by days but tick together, so every correct match has the same
    RX-minus-TX log time; a match one u8 wrap off does not."""

    def test_the_seq_running_ahead_of_the_trains(self) -> None:
        # 15-fragment trains: once the queue saturates the seq moves 8 per
        # train (frames dropped or batched), so 32 unheard trains are a whole
        # wrap; the RX log runs 10 trains past the TX log (truth 480/1500)
        trains, u, t = [], 1, 0.0
        for n in range(110):
            trains.append((u, t))
            u += 1 if n < 12 else 8
            t += 3.6
        tx, _ = _two_clock_logs(trains[:100], lambda n: True, 15)
        _, rx = _two_clock_logs(trains, lambda n: not 40 <= n < 72, 15)
        res = rep.shared_window_loss(tx, rx)
        self.assertIn("wrap off", res["withheld"])

    def test_an_outage_just_short_of_a_wrap(self) -> None:
        # one seq per train, 254 trains unheard: no wrap inside the RX gap,
        # yet the lookahead finds the seq after it (truth 254/600)
        trains = [(n + 1, n * 0.5) for n in range(600)]
        tx, rx = _two_clock_logs(trains, lambda n: not 121 <= n < 375)
        res = rep.shared_window_loss(tx, rx)
        self.assertIn("wrap off", res["withheld"])

    def test_an_idle_feed_is_not_an_outage(self) -> None:
        # the feed stops for 100 s (TX idle, the seq does not move)
        trains = ([(n + 1, n * 0.5) for n in range(200)]
                  + [(201 + n, 200 + n * 0.5) for n in range(200)])
        tx, rx = _two_clock_logs(trains, lambda n: n % 37 != 5)
        res = rep.shared_window_loss(tx, rx)
        self.assertIsNone(res["withheld"])
        self.assertEqual((res["lost"], res["sent"]), (11, 400))

    def test_board_times_are_not_read_as_local_time(self) -> None:
        # the base clock crosses 2026-11-01 02:00 (a US DST change if read as
        # the PC's local time): no 3600 s jump, no split train
        trains = [(n + 1, n * 0.5) for n in range(400)]
        tx, rx = _two_clock_logs(trains, lambda n: True, 2,
                                 rx_t0=datetime(2026, 11, 1, 1, 58, 0))
        res = rep.shared_window_loss(tx, rx)
        self.assertIsNone(res["withheld"])
        self.assertEqual(res["lost"], 0)


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
        self.assertNotIn("LOG WINDOW UNVERIFIED", out)
        self.assertIn("log_window=after_stop", out.splitlines()[0])

    def test_another_log_window_value_still_gets_the_note(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            arch = self._archive(Path(d), params="log_window=before_stop\n")
            out = _run([arch] + _brackets(Path(d), 61))
        self.assertIn("(log window: params.txt has no log_window=after_stop", out)
        self.assertNotIn("LOG WINDOW UNVERIFIED", out)

    # 2026-10-22: the harness writes log_window=after_stop only when both
    # daemon stops worked and both logs hold their SIGTERM shutdown line;
    # otherwise after_stop_unverified plus log_window_detail=<reason>.
    UNVERIFIED = ("log_window=after_stop_unverified\n"
                  "log_window_detail=tx_smoke stop: docker stop exit 1: Error "
                  "response from daemon: No such container: tx_smoke\n"
                  "stop_drain_s=2\n")

    def test_unverified_window_gets_its_own_note_quoting_the_detail(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            arch = self._archive(Path(d), params=self.UNVERIFIED)
            out = _run([arch] + _brackets(Path(d), 61))
        lines = out.splitlines()
        self.assertIn("log_window=after_stop_unverified", lines[0])
        note = next(n for n, ln in enumerate(lines)
                    if "LOG WINDOW UNVERIFIED (" in ln)
        # a failed stop: the old read-while-running artefact is possible
        self.assertEqual(
            lines[note],
            "  !! LOG WINDOW UNVERIFIED (log_window_detail: tx_smoke stop: "
            "docker stop exit 1: Error response from daemon: No such "
            "container: tx_smoke): a daemon stop was not confirmed, so that "
            "daemon may still have run when its log was read and the logs "
            "may not cover the post-stop window — a 'received > sent' line "
            "above or a small positive FIRMWARE DROP below can carry the old "
            "log-window artefact (logs read while a daemon ran), not the "
            "link or the firmware")
        # 2026-10-22 review: below the loss block (leg_post.sh shows only the
        # first 8 lines), above the FIRMWARE DROP it qualifies, and the
        # 'received > sent' line points down at it
        warn = next(n for n, ln in enumerate(lines) if "received 59 > sent 45" in ln)
        self.assertTrue(lines[warn].endswith(
            "read the shared-window line; the log window is unverified, see "
            "the LOG WINDOW UNVERIFIED note below"), lines[warn])
        shared = next(n for n, ln in enumerate(lines)
                      if ln.startswith("shared-window loss "))
        drop = next(n for n, ln in enumerate(lines)
                    if ln.startswith("FIRMWARE DROP"))
        self.assertLess(warn, note)
        self.assertLess(shared + 1, note)          # after the 'left out' line
        self.assertLess(note, drop)
        self.assertIn("FIRMWARE DROP (drx_ok - host URCs): 61 - 59 = 2", out)
        self.assertIn("(log window: after_stop_unverified, see the LOG WINDOW "
                      "UNVERIFIED note above — if a daemon still ran when its "
                      "log was read, or the RX daemon died before its stop,", out)
        # not the pre-10-22 note, which says the logs were read BEFORE the stop
        self.assertNotIn("params.txt has no log_window=after_stop", out)

    def test_unverified_note_leaves_leg_posts_8_line_excerpt_alone(self) -> None:
        # legs/leg_post.sh prints `head -n 8` of the report: with --capture
        # the capture seq-gap line (the loss of record) is among those lines
        # on every leg, the unverified ones included; only the header and
        # the pointer at the end of the 'received > sent' line differ.
        def run(d: Path, params: str) -> list:
            arch = self._archive(d, params=params)
            cap = d / "leg2x_base.jsonl"
            _write_capture(cap, [(float(s), _payload(V, s, key=(s == 1)))
                                 for s in (1, 2, 3, 4)])
            return _run([arch, "--capture", cap] + _brackets(d, 61)).splitlines()
        with tempfile.TemporaryDirectory() as d1, \
                tempfile.TemporaryDirectory() as d2:
            old = run(Path(d1), "")
            unv = run(Path(d2), self.UNVERIFIED)
        cap_old = next(n for n, ln in enumerate(old)
                       if ln.startswith("capture seq gaps (leg2x_base.jsonl)"))
        cap_unv = next(n for n, ln in enumerate(unv)
                       if ln.startswith("capture seq gaps (leg2x_base.jsonl)"))
        self.assertEqual(cap_unv, cap_old)
        self.assertLess(cap_unv, 8)
        # the capture's head line (these logs hold no published line to read the
        # first frame's transport seq from), then the note
        self.assertTrue(unv[cap_unv + 1].startswith("  first frame heard at seq 1: the head "
                                                    "is AMBIGUOUS"), unv[cap_unv + 1])
        self.assertTrue(unv[cap_unv + 2].startswith("  !! LOG WINDOW UNVERIFIED ("),
                        unv[cap_unv + 2])
        warn = next(n for n, ln in enumerate(unv) if "received 59 > sent 45" in ln)
        for n in range(1, cap_unv + 1):
            if n == warn:
                self.assertEqual(unv[n], old[n] + "; the log window is "
                                 "unverified, see the LOG WINDOW UNVERIFIED "
                                 "note below")
            else:
                self.assertEqual(unv[n], old[n])

    def test_unverified_note_is_printed_without_brackets_too(self) -> None:
        # 'received > sent' needs no brackets, so neither does the note.
        with tempfile.TemporaryDirectory() as d:
            arch = self._archive(Path(d), params=self.UNVERIFIED)
            out = _run([arch])
        self.assertIn("!! LOG WINDOW UNVERIFIED (log_window_detail: tx_smoke "
                      "stop: docker stop exit 1:", out)
        self.assertNotIn("FIRMWARE DROP (drx_ok", out)
        self.assertNotIn("(log window:", out)

    def test_missing_shutdown_line_after_good_stops_says_the_daemon_died_first(
            self) -> None:
        # Out-File -Encoding utf8 on Windows: a BOM and CRLF line ends; the
        # quoted detail must not carry the CR. Both stops worked (no 'stop:'
        # reason), so the log ends EARLY: not the read-while-running artefact.
        with tempfile.TemporaryDirectory() as d:
            arch = self._archive(Path(d))
            (arch / "params.txt").write_bytes(
                ("﻿git_sha=0000000\r\nlog_window=after_stop_unverified\r\n"
                 "log_window_detail=rx_smoke log has no shutdown line\r\n"
                 "feed_stop=synth_pub stopped\r\n").encode("utf-8"))
            out = _run([arch])
        self.assertIn("!! LOG WINDOW UNVERIFIED (log_window_detail: rx_smoke "
                      "log has no shutdown line): both daemon stops were "
                      "confirmed, but a daemon log has no SIGTERM shutdown "
                      "line — that daemon may have died before the stop", out)
        self.assertIn("so the log can end before the leg did and the "
                      "log-derived figures above cover less than the leg", out)
        self.assertIn("this is not the old read-while-running artefact", out)
        self.assertNotIn("may still have run when its log was read", out)
        self.assertNotIn("\r", out)

    def test_a_failed_stop_and_a_missing_line_name_both_causes(self) -> None:
        params = ("log_window=after_stop_unverified\n"
                  "log_window_detail=tx_smoke stop: docker stop exit 1: no "
                  "output; rx_smoke stop: no exit status came back (adb "
                  "failed?): no output; tx_smoke log has no shutdown line\n")
        with tempfile.TemporaryDirectory() as d:
            out = _run([self._archive(Path(d), params=params)])
        note = next(ln for ln in out.splitlines()
                    if "LOG WINDOW UNVERIFIED (" in ln)
        self.assertIn("a daemon stop was not confirmed, so that daemon may "
                      "still have run when its log was read", note)
        self.assertTrue(note.endswith(
            "; a log without its shutdown line can also mean that daemon "
            "died before the stop, so its log ends early"), note)

    def test_unverified_without_a_detail_line_says_so(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            arch = self._archive(
                Path(d), params="log_window=after_stop_unverified\n")
            out = _run([arch])
        # no reason to choose by: one neutral sentence
        self.assertIn("!! LOG WINDOW UNVERIFIED (log_window_detail: params.txt "
                      "has no log_window_detail): the harness could not "
                      "confirm that each daemon ran until its stop and had "
                      "stopped before its log was read", out)

    def test_no_key_keeps_the_old_note_and_no_unverified_line(self) -> None:
        # every archive before 10-22 (none has the key): unchanged output
        with tempfile.TemporaryDirectory() as d:
            arch = self._archive(Path(d))
            out = _run([arch] + _brackets(Path(d), 61))
        self.assertIn("(log window: params.txt has no log_window=after_stop — "
                      "this harness read the daemon logs BEFORE it stopped "
                      "the daemons", out)
        self.assertNotIn("LOG WINDOW UNVERIFIED", out)
        self.assertNotIn("after_stop_unverified", out)


# -- 2026-10-24: acquisition, the head on the TX log -------------------------------
# A leg on two board clocks (the tractor 19 days behind, as on the bench), on a PC
# clock in seconds from _PC0; the params.txt offsets are board minus PC.
_PC0 = datetime(2026, 10, 24, 15, 35, 36)
_TR_OFF, _BASE_OFF = -1667316.851, -0.869


def _stamp(pc_s: float, off: float) -> str:
    return (_PC0 + timedelta(seconds=pc_s + off)).strftime("%Y-%m-%d %H:%M:%S,%f")[:-3]


def _epoch(pc_s: float, off: float) -> float:
    return (_PC0 + timedelta(seconds=pc_s + off)).replace(tzinfo=timezone.utc).timestamp()


LEG6 = (264, 293, 346, 358, 494)          # the tractor frames 10-24 leg 6's base heard


def _two_board_leg(d: Path, *, sent: int = 606, heard=LEG6, period: float = 0.5,
                   offsets: bool = True, base_extra=(), rx_skew: float = 0.6,
                   profile: int = 1, capture: bool = True, mask: bool = False,
                   params_extra: str = "", cap_seq=None,
                   published: bool = True) -> tuple[Path, Path | None]:
    """The TX daemon starts at PC 0 and puts frame n on air at 8 + (n - 1) *
    period; the RX daemon starts ``rx_skew`` later, activates ``profile`` at
    PC 1.4 (its LBT line), decodes the frames in ``heard`` 30 ms after their
    txdone, publishes each 1 ms later (``published`` False: no published
    line), and stops 3 s after the tractor's last frame + 4 s. ``base_extra``:
    ``(PC s, text)`` lines for the base log; ``mask``: the base log names its
    50-channel FHSS hop set (untimestamped, as the activation prints it, 2 log
    lines before the activation's); ``cap_seq``: n -> the TileDeltaFrame seq
    of the tractor's frame n (default: its transport seq, n mod 256). params:
    tx_batch=0 (as the 10-24 harness writes it), then ``params_extra``.
    Returns (archive, capture)."""
    a = d / "radio_monitor_leg"
    a.mkdir()
    params = f"﻿git_sha=0000000\nreg_profile={profile}\nsynth_fps={1 / period:g}\n" \
             "log_window=after_stop\ntx_batch=0\n" + params_extra
    if offsets:
        params += f"tractor_clock_offset_s={_TR_OFF}\nbase_clock_offset_s={_BASE_OFF}\n"
    (a / "params.txt").write_text(params, encoding="utf-8")
    tx = [f"{_stamp(0, _TR_OFF)} INFO image_tx_daemon: opening L072 HostLink"]
    for n in range(1, sent + 1):
        s = _stamp(8 + (n - 1) * period, _TR_OFF)
        tx += [f"{s} INFO image_tx_daemon: txdone_arrival: seq={n % 256} idx=0 status=0 toa_us=1",
               f"{s} INFO image_tx_daemon: frame seq={n % 256} done (pipelined): 1 fragments ok"]
    end = 8 + sent * period + 4
    tx += [f"{_stamp(end, _TR_OFF)} INFO image_tx_daemon: stats: frames_in={sent} "
           f"frags_ok={sent} frags_fail=0",
           f"{_stamp(end + 1, _TR_OFF)} INFO image_tx_daemon: signal received; shutting down"]
    (a / "tx_daemon.log").write_text("\n\n".join(tx) + "\n", encoding="utf-8")
    rx = [(rx_skew, "INFO image_rx_daemon: opening L072 HostLink on /dev/ttymxc3"),
          (1.39, f"CFG_SET_REQ(REG_PROFILE={profile}) OK: 14000100"),     # no timestamp
          (1.4, "INFO image_rx_daemon: LBT_ENABLE=0 (command uplink shares the image channel)"),
          (1.45, "INFO image_rx_daemon: RX worker ready; draining RX_FRAME_URC...")]
    if mask:
        rx.append((1.38, "  FHSS mask: WIDE (50 channels) [LIFETRAC_FHSS_WIDE_MASK=1]"))
    rows = []
    for n in heard:
        pc = 8 + (n - 1) * period + 0.03
        rx.append((pc, f"INFO image_rx_daemon: frag_arrival: seq={n % 256} idx=0 total=1 "
                       f"fw_us=1 len=202"))
        if published:
            rx.append((pc + 0.001, f"INFO image_rx_daemon: published frame_id=0 seq={n % 256} "
                                   f"198 B"))
        rows.append((_epoch(pc + 0.002, _BASE_OFF),
                     _payload(V, n % 256 if cap_seq is None else cap_seq(n))))
    rx += list(base_extra)
    rx += [(end + 2, f"INFO image_rx_daemon: stats: rx_frames={len(heard)} "
                     f"frames_published={len(heard)} reassembler_timeouts=0"),
           (end + 3, "INFO image_rx_daemon: signal received; shutting down")]
    text = [line if pc in (1.38, 1.39) else f"{_stamp(pc, _BASE_OFF)} {line}"
            for pc, line in sorted(rx, key=lambda r: r[0])]
    (a / "rx_daemon.log").write_text("\n\n".join(text) + "\n", encoding="utf-8")
    cap = None
    if capture:
        cap = d / "leg2x_base.jsonl"
        _write_capture(cap, rows)
    return a, cap


def _read(p: Path) -> str:
    return p.read_text(encoding="utf-8")


class HeadOnTheTxLog(unittest.TestCase):
    """2026-10-24: the capture's first frame placed on the tractor's TX log."""

    def _place(self, d: Path, **kw) -> dict:
        arch, cap = _two_board_leg(d, **kw)
        frames, _ = rep.capture_frames(cap)
        return rep.place_capture(frames, _read(arch / "tx_daemon.log"),
                                 _read(arch / "rx_daemon.log"),
                                 rep.params_dict(_read(arch / "params.txt")))

    def test_params_offsets_place_leg_6s_first_frame_after_a_wrap(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            p = self._place(Path(d))
        self.assertEqual((p["sent"], p["how"]), (606, "offsets"))
        self.assertEqual((p["first"]["frame"], p["first"]["wraps"]), (264, 1))
        self.assertEqual((p["last"]["frame"], p["last"]["wraps"]), (494, 1))
        self.assertAlmostEqual(p["first"]["dt"], 0.032, places=2)

    def test_daemon_start_times_place_it_without_offsets(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            p = self._place(Path(d), offsets=False)
        self.assertEqual(p["how"], "starts")
        self.assertEqual(p["first"]["frame"], 264)
        self.assertAlmostEqual(p["first"]["skew"], 0.568, places=2)
        self.assertEqual(p["last"]["frame"], 494)            # on the first frame's match
        self.assertNotIn("dt", p["last"])
        # the RX daemon 32 s after the TX daemon (the widest committed): still one fit
        with tempfile.TemporaryDirectory() as d:
            p = self._place(Path(d), offsets=False, rx_skew=32.4)
        self.assertEqual(p["first"]["frame"], 264)

    def test_two_fits_or_none_leave_the_head_ambiguous(self) -> None:
        # 10 fps: a wrap is 25.6 s, so the start-time window fits #44 and #300
        with tempfile.TemporaryDirectory() as d:
            p = self._place(Path(d), offsets=False, period=0.1, sent=400, heard=(44, 54))
        self.assertNotIn("frame", p["first"])
        self.assertIn("the daemons' start times fit 2 of the tractor's frames with seq 44 "
                      "(#44, #300)", p["first"]["why"])
        self.assertEqual(p["last"], {"why": "the first frame was not placed"})
        # with the offsets the same leg places
        with tempfile.TemporaryDirectory() as d:
            p = self._place(Path(d), period=0.1, sent=400, heard=(44, 54))
        self.assertEqual((p["first"]["frame"], p["first"]["wraps"]), (44, 0))
        # an RX log that starts a minute before the TX log fits nothing
        with tempfile.TemporaryDirectory() as d:
            p = self._place(Path(d), offsets=False, rx_skew=-60.0)
        self.assertIn("fit 0 of the tractor's frames with seq 8", p["first"]["why"])

    def test_unplaceable_frames_say_why(self) -> None:
        trains = rep.tx_frames(_tx_t([(n, [0]) for n in range(1, 11)]))
        self.assertEqual([t["u"] for t in trains], list(range(1, 11)))
        self.assertIn("no txdone_arrival with seq 77",
                      rep.place_frame(0.0, 77, trains, (0.0, 0.0))["why"])
        far = rep.place_frame(trains[2]["t"] + 9.0, 3, trains, (0.0, 0.0))
        self.assertIn("is +9.0 s from it on the params.txt clock offsets (more than 2 s)",
                      far["why"])
        self.assertIn("no clock offsets in params.txt",
                      rep.place_frame(0.0, 3, trains)["why"])
        # no txdone_arrival lines at all, and no parsed frame
        p = rep.place_capture([(1.0, V, 3)], "x frame seq=3 done: 1 fragments ok\n", "",
                              {})
        self.assertEqual(p["sent"], 0)
        self.assertIn("no txdone_arrival line", p["why"])
        self.assertIsNone(rep.place_capture([(1.0, None, None)], "", "", {}))
        # unwrapping: 254, 255, 0, 1 count on past the wrap
        trains = rep.tx_frames(_tx_t([(n, [0]) for n in range(254, 260)]))
        self.assertEqual([t["u"] for t in trains], list(range(254, 260)))
        # a 13-fragment train spans 2.5 s: the base publishes after its last
        # fragment, so the frame is placed on the train's last txdone
        tx, _ = _two_clock_logs([(n, 3.6 * n) for n in range(1, 6)], lambda n: True, 13)
        trains = rep.tx_frames(tx)
        self.assertAlmostEqual(trains[2]["t_end"] - trains[2]["t"], 2.52, places=2)
        got = rep.place_frame(trains[2]["t_end"] + 0.05, 3, trains, (0.0, 0.0))
        self.assertEqual((got["frame"], round(got["dt"], 2)), (3, 0.05))

    def test_a_capture_that_outruns_the_tx_log(self) -> None:
        # an older harness read the TX log while the daemons ran: the base
        # published two frames after it
        with tempfile.TemporaryDirectory() as d:
            arch, cap = _two_board_leg(Path(d), sent=300, heard=range(40, 303), offsets=False)
            frames, _ = rep.capture_frames(cap)
            p = rep.place_capture(frames, _read(arch / "tx_daemon.log"),
                                  _read(arch / "rx_daemon.log"), {})
        self.assertEqual(p["first"]["frame"], 40)
        self.assertEqual(p["last"], {"why": "it came after the TX log's last frame (#300): "
                                            "the TX log ends before the capture does"})

    def test_frames_are_placed_by_their_transport_seq(self) -> None:
        # review 2026-10-24 (2): txdone_arrival logs the TX daemon's own count of the
        # messages it took, the seq the base logs in frag_arrival and its published
        # line; the TileDeltaFrame seq is the camera's counter. Here the camera
        # numbers one behind (the TX daemon took one more message): its seq 138 is
        # frame #139 on air, and txdone_arrival seq 138 (#138) went 0.5 s earlier,
        # well inside PLACE_TOL_S, so placing by the TileDeltaFrame seq would put it
        # one frame off without a word
        with tempfile.TemporaryDirectory() as d:
            p = self._place(Path(d), heard=range(139, 607), cap_seq=lambda n: (n - 1) % 256)
        self.assertEqual((p["first"]["frame"], p["first"]["tdf_seq"], p["first"]["rx_seq"]),
                         (139, 138, 139))
        self.assertEqual((p["last"]["frame"], p["last"]["tdf_seq"], p["last"]["rx_seq"]),
                         (606, 605 % 256, 606 % 256))
        self.assertIsNone(p["batched"])
        # no published line at the capture time: no transport seq, not placed
        with tempfile.TemporaryDirectory() as d:
            p = self._place(Path(d), heard=range(139, 607), published=False)
        self.assertNotIn("frame", p["first"])
        self.assertEqual(p["first"]["why"], "rx_daemon.log has no published line within 0.05 s "
                                            "of its capture time, so its transport seq (the "
                                            "one txdone_arrival logs) is unknown, and its "
                                            "TileDeltaFrame seq is the camera's own counter")
        # two published seqs at once: ambiguous, not placed
        pubs = [(10.0, 7, 3), (10.02, 8, 5)]
        self.assertEqual(rep.transport_seq(10.01, pubs),
                         {"why": "rx_daemon.log published transport seqs [7, 8] within 0.05 s "
                                 "of its capture time"})
        # frames batched in one train share its seq: one seq, placed on that train
        self.assertEqual(rep.transport_seq(10.01, [(10.0, 7, 3), (10.0, 7, 4)]),
                         {"seq": 7, "line": 3})
        self.assertEqual(rep.rx_publications("2026-10-24 15:35:00,000 INFO x: published "
                                             "frame_id=0 seq=-1 9 B\n"), [])

    def test_the_head_line_when_the_two_counters_differ(self) -> None:
        # the capture's first frame is placed on the TX log by its transport seq, but
        # the camera's frames before it cannot be counted from there
        with tempfile.TemporaryDirectory() as d:
            arch, cap = _two_board_leg(Path(d), heard=range(139, 607), mask=True,
                                       cap_seq=lambda n: (n - 1) % 256)
            out = _run([arch, "--capture", cap])
        # placed (the TX-log line below places it too), but its head not counted
        self.assertIn("  first frame heard at seq 138: the head is AMBIGUOUS — it is the "
                      "tractor's frame #139 on air by its transport seq 139 (rx_daemon.log line "
                      "13), but its TileDeltaFrame seq is not that seq: the camera's counter and "
                      "the TX daemon's differ here (a codec switch, a camera restart or a frame "
                      "the TX daemon never took), so the camera's frames before it cannot be "
                      "counted from the TX log (137 frame(s) numbered before it if seq 138 is in "
                      "the camera's first u8 cycle, 256 more per wrap); no figure counting from "
                      "seq 1 (RS-13.1 A7)\n", out)
        self.assertNotIn("cannot be placed", out)
        self.assertNotIn("counting from seq 1:", out)
        self.assertIn("the capture's first frame is its #139, its last #606 (placed by the "
                      "params.txt clock offsets, within 0.03 s): 138/606 = 22.8% of them never "
                      "reached the capture", out)

    def test_a_leg_that_batched_withholds_the_tx_log_share(self) -> None:
        # review 2026-10-24 (1): with LIFETRAC_TX_BATCH on, a train carries several
        # frames under the first one's seq, so the TX log's trains and the capture's
        # frames are not one unit. image_tx_daemon.py logs "batching:" on its first
        # batched train (never for a VECTOR or key frame): withheld on that line,
        # and the lines count trains
        batch = (f"{_stamp(9, _TR_OFF)} INFO image_tx_daemon: batching: 2 frames in 1 trains "
                 f"so far (this train: 2 frames, 400 B)")

        def leg(d, extra, batched, heard=range(139, 607)):
            arch, cap = _two_board_leg(Path(d), heard=heard, params_extra=extra)
            if batched:
                p = arch / "tx_daemon.log"
                head, rest = _read(p).split("\n\n", 1)
                p.write_text(head + "\n\n" + batch + "\n\n" + rest, encoding="utf-8")
            return _run([arch, "--capture", cap])
        for extra in ("tx_batch=1\n", "tx_batch=4\n", "tx_batch=\n"):
            with tempfile.TemporaryDirectory() as d:
                out = leg(d, extra, True)
            want = "params.txt " + extra.strip()
            self.assertIn("  tractor TX log: 606 train(s) on air (txdone_arrival); the capture's "
                          "first frame is in its train #139, its last in train #606 (placed by "
                          "the params.txt clock offsets, within 0.03 s): no share of them that "
                          "never reached the capture (tx_daemon.log line 3 logs a batched train "
                          f"({want}): a train can carry several frames, so the TX log's trains "
                          "and the capture's frames are not one unit)", out)
            self.assertNotIn("of them never reached the capture", out)
        # ... and the head line names the train it is in
        with tempfile.TemporaryDirectory() as d:
            out = leg(d, "tx_batch=1\n", True, heard=range(300, 607))
        self.assertIn("  first frame heard at seq 44 is in the tractor's train #300 on air (1 "
                      "u8 seq wrap before it, by its TX log): the 299 frame(s)", out)
        # params that do not say tx_batch=0, but no train batched (the harness default
        # -TxBatch 1 on a VECTOR leg): the share, and why it holds
        with tempfile.TemporaryDirectory() as d:
            out = leg(d, "tx_batch=1\n", False)
        self.assertIn("  tractor TX log: 606 frame(s) on air (txdone_arrival); the capture's "
                      "first frame is its #139, its last #606 (placed by the params.txt clock "
                      "offsets, within 0.03 s): 138/606 = 22.8% of them never reached the "
                      "capture (138 before its first, 0 after its last); params.txt tx_batch=1, "
                      "but tx_daemon.log logs no 'batching:' line: no train carried two "
                      "frames\n", out)
        self.assertEqual(rep.batching({}), "params.txt has no tx_batch: a train can carry "
                                           "several frames, so the TX log's trains and the "
                                           "capture's frames are not one unit")
        self.assertIsNone(rep.batching({"tx_batch": "0"}))
        self.assertIsNone(rep.batching({"tx_batch": "0"}, batch))
        self.assertIsNone(rep.batching({"tx_batch": "1"}, "x: frame seq=1 done\n"))
        self.assertTrue(rep.batching({"tx_batch": "1"}, "x\n" + batch).startswith(
            "tx_daemon.log line 2 logs a batched train (params.txt tx_batch=1)"))


class Acquisition(unittest.TestCase):
    """2026-10-24: the base's first decoded frame against the 120 s scan budget."""

    def _acq(self, d: Path, **kw) -> dict:
        arch, _ = _two_board_leg(d, capture=False, **kw)
        return rep.acquisition(_read(arch / "rx_daemon.log"), _read(arch / "tx_daemon.log"))

    def test_budget_is_four_attempts_of_thirty_seconds(self) -> None:
        self.assertEqual(rep.SCAN_BUDGET_S, 120.0)
        self.assertEqual((rep.SCAN_ATTEMPT_S, rep.SCAN_MAX_RETRIES), (30.0, 3))

    def test_leg_6_fails_and_a_locked_leg_does_not(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d))
        self.assertTrue(acq["applies"])
        self.assertAlmostEqual(acq["first"], 138.13, places=2)
        self.assertEqual((acq["status"], acq["failed"]), (rep.FAILED, True))
        self.assertEqual((acq["act_line"], acq["tractor"]), (5, 606))
        lines = rep.format_acquisition(acq)
        self.assertTrue(lines[0].startswith(
            "acquisition: first image reception: the base's first decoded frame came 138.1 s "
            "after its profile-1 activation (rx_daemon.log line 5 at 15:35:36.531 board time, "
            "the frame at line 9), past the FHSS cold-start scan budget of 120 s (4 attempts x "
            "30 s: SX1276_RX_SCAN_MAX_RETRIES 3 + 1, SX1276_RX_SCAN_REDESIGN_MS 30000)"),
            lines[0])
        # review 2026-10-24: FAILED on positive evidence. These logs hold frag_arrival
        # lines, so per-fragment logging was on all run (image_rx_daemon.py reads the
        # flag once), and the first came past the budget
        self.assertTrue(lines[1].startswith(
            "  !! ACQUISITION FAILED: per-fragment logging was on (rx_daemon.log holds "
            "frag_arrival lines) and the first came at +138.1 s: no image fragment reached the "
            "base within the 120 s budget, and its scan locks only on a frame it decodes. Its "
            "scan ran out into FAILED, which is absorbing"), lines[1])
        self.assertTrue(lines[1].endswith("The tractor logged 606 frame(s) on air."))
        # an archive without FAULT_URC lines (every one before 2026-10-24) says so, and
        # what a newer daemon's silence would mean (review 2026-10-24)
        self.assertEqual(lines[2], "  (rx_daemon.log holds no FAULT_URC line, so the FAILED "
                                   "entry is inferred, not seen: image_rx_daemon.py dropped "
                                   "FAULT URCs at DEBUG before 2026-10-24; from 2026-10-24 on it "
                                   "logs every one, and a scan that runs out sends an "
                                   "RX_SCAN_FAILED per attempt, so with that daemon this "
                                   "silence would contradict the FAILED entry; check which "
                                   "daemon ran)")
        self.assertEqual(len(lines), 3)
        # legs 1-5 of 10-24 caught their first frame 15-75 s in and decoded most of the
        # tractor's fragments on the 50-channel hop set: ACQUIRED, a lock shown
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), heard=range(139, 607), mask=True)
        self.assertEqual((acq["status"], acq["failed"]), (rep.ACQUIRED, False))
        lines = rep.format_acquisition(acq)
        self.assertEqual(len(lines), 2)
        self.assertIn("came 75.6 s after its profile-1 activation", lines[0])
        self.assertIn(", within the FHSS cold-start scan budget", lines[0])
        self.assertEqual(lines[1], "  confirmed acquisition: ACQUIRED: the base decoded 468 of "
                                   "the tractor's 606 fragments on air (77.2 %): on its "
                                   "50-channel hop set a receiver that does not follow the hop "
                                   "grid is on the tractor's channel about one slot in 50, and "
                                   "only a locked scan follows the grid")

    def test_a_frame_received_in_the_budget_is_not_a_lock(self) -> None:
        # review 2026-10-24 (3): sx1276_rx.c delivers a frame whose FHSS header it
        # rejected (10-24 leg 6's base: d(fhss_dec_rej_epoch_drift) 2 of its 5), so a
        # first image reception within the budget confirms no acquisition
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), heard=(40, 300, 560), mask=True)
        self.assertEqual((acq["status"], acq["failed"]), (rep.UNDETERMINED, None))
        lines = rep.format_acquisition(acq)
        self.assertIn("came 26.1 s after its profile-1 activation", lines[0])
        self.assertIn(", within the FHSS cold-start scan budget", lines[0])
        self.assertEqual(lines[1], "  confirmed acquisition: NOT DETERMINED: a frame received "
                                   "is not a lock (sx1276_rx.c still delivers a frame whose FHSS "
                                   "header it rejected), and nothing here shows one; the base "
                                   "decoded 3 of the tractor's 606 fragments on air (0.5 %), "
                                   "under the 25 % that shows a receiver following its "
                                   "50-channel hop grid")
        self.assertEqual(len(lines), 2)

        def urc(pc, sub):
            return (pc, f"WARNING image_rx_daemon: FAULT_URC: code=0x0D sub=0x{sub:02X} "
                        f"name=RX_SCAN_FAILED attempt=3 final=1 uptime_ms=1 raw=0d")
        # a cold final RX_SCAN_FAILED (warm=0: the L072 never locked since it booted)
        # after that reception: FAILED, never reported as acquired
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), heard=(40, 300, 560), mask=True,
                            base_extra=[urc(121.6, 0x83)])
        self.assertEqual(acq["status"], rep.FAILED)
        lines = rep.format_acquisition(acq)
        self.assertTrue(lines[1].startswith(
            "  !! ACQUISITION FAILED: the base's final RX_SCAN_FAILED at +120.2 s (rx_daemon.log "
            "line 15, sub 0x83) is cold (warm=0): the base's L072 had not locked since it "
            "booted. Its scan ran out into FAILED"), lines[1])
        # it wins over a share that would otherwise show a lock
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), heard=range(139, 607), mask=True,
                            base_extra=[urc(300.0, 0x83)])
        self.assertEqual(acq["status"], rep.FAILED)
        # a warm final less than 122 s after the activation: 4 attempts in a row
        # from it with no lock (a lock resets the count but keeps the channel anchor
        # the activation's BEGIN_SCAN set, so its loss comes 2 s after that at the
        # earliest). The activation is dated by the line before it (PC 0.6 here,
        # 0.8 s before its LBT line), so the final's time from it is not read short
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), heard=(40,), mask=True, base_extra=[urc(122.0, 0xC3)])
        self.assertEqual(acq["status"], rep.FAILED)
        self.assertIn("the base's final RX_SCAN_FAILED at +120.6 s (rx_daemon.log line 15, sub "
                      "0xC3) came less than 122 s after the activation (at most +121.4 s from "
                      "rx_daemon.log line 1, the last timestamped line before it): 4 scan "
                      "attempts failed in a row from it (a lock resets the attempt count but "
                      "keeps the channel anchor the activation's BEGIN_SCAN set, so a locked "
                      "scan drops back to scanning no sooner than SX1276_RX_SCAN_LOCK_LOSS_MS "
                      "2000 after the activation)", acq["basis"])
        # review 2026-10-24: 121.3 s after the LBT line can be 122.1 s after the
        # activation, so a lock lost at once is not ruled out: no evidence
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), heard=(40,), mask=True, base_extra=[urc(122.7, 0xC3)])
        self.assertEqual(acq["status"], rep.UNDETERMINED)
        self.assertAlmostEqual(acq["faults"][0]["t"], 121.3, places=3)
        # a log with no timestamped line before the activation cannot date it: no
        # early-final evidence (a cold final still is)
        rx = ("CFG_SET_REQ(REG_PROFILE=1) OK: 14000100\n"
              "2026-10-24 15:35:00,000 INFO x: LBT_ENABLE=0\n"
              "2026-10-24 15:35:10,000 INFO x: frag_arrival: seq=8 idx=0 total=1\n"
              "2026-10-24 15:37:00,500 WARNING x: FAULT_URC: code=0x0D sub=0x{:02X} "
              "name=RX_SCAN_FAILED uptime_ms=1 raw=0d\n")
        self.assertEqual(rep.acquisition(rx.format(0xC3))["status"], rep.UNDETERMINED)
        self.assertEqual(rep.acquisition(rx.format(0x83))["status"], rep.FAILED)
        # a warm final later (a lock lost mid-leg) does not undo the lock shown; its
        # !! line still says the scan entered FAILED there
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), heard=range(139, 300), mask=True,
                            base_extra=[urc(280.0, 0xC3)])
        self.assertEqual(acq["status"], rep.ACQUIRED)
        self.assertIn("  !! the base's final RX_SCAN_FAILED at +278.6 s",
                      "\n".join(rep.format_acquisition(acq)))
        # an AMBIGUOUS final (maybe a TX FIFO readback mismatch) is no evidence
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), heard=(40, 300, 560), mask=True, base_extra=[
                (121.6, "WARNING image_rx_daemon: FAULT_URC: code=0x0D sub=0x83 "
                        "name=AMBIGUOUS uptime_ms=1 raw=0d83")])
        self.assertEqual(acq["status"], rep.UNDETERMINED)

    def test_silence_counts_only_from_an_instrument_known_to_be_on(self) -> None:
        # review 2026-10-24 (4): -LogFragArrivals defaults off, and a frame can take
        # several fragments, so a base can log no frame line for over 120 s while it
        # hears the tractor. The stats counter is always on.
        def rx(*rows):
            # the start-up configuration prints the mask before its REG_PROFILE line
            return ("  FHSS mask: WIDE (50 channels) [LIFETRAC_FHSS_WIDE_MASK=1]\n"
                    "CFG_SET_REQ(REG_PROFILE=1) OK: 14000100\n"
                    "2026-10-24 15:35:00,000 INFO x: LBT_ENABLE=0\n"
                    + "".join(f"2026-10-24 {15 + (35 * 60 + s) // 3600:02d}:"
                              f"{(35 * 60 + s) // 60 % 60:02d}:{s % 60:02d},000 INFO x: {t}\n"
                              for s, t in rows)
                    + "2026-10-24 15:40:00,000 INFO x: signal received; shutting down\n")
        tx = "x frame seq=1 done (pipelined): 600 fragments ok\n"
        counting = [(s, f"stats: rx_frames={0 if s < 20 else 2 * s}") for s in range(10, 300, 10)]
        # no frame line in 300 s, but the counter counts from +20 s: received, and the
        # share shows the lock (the a3dc5d01 rule read this silence as FAILED)
        acq = rep.acquisition(rx(*counting), tx)
        self.assertEqual((acq["first"], acq["heard"][0], acq["status"]),
                         (None, 20.0, rep.ACQUIRED))
        self.assertTrue(rep.format_acquisition(acq)[0].startswith(
            "acquisition: first image reception: no decoded-frame line, but the base's stats "
            "counter counted a frame by +20.0 s after its profile-1 activation"))
        self.assertIn("the base decoded 580 of the tractor's 600 fragments on air (96.7 %)",
                      acq["basis"])
        # no frame line, no stats line, no frag_arrival line: not determined
        acq = rep.acquisition(rx(), tx)
        self.assertEqual(acq["status"], rep.UNDETERMINED)
        self.assertTrue(acq["basis"].startswith(
            "the base's log holds no frame line, but no instrument known to be on shows that no "
            "frame reached it within the budget (no stats line after it still at rx_frames=0, "
            "no frag_arrival line to show per-fragment logging on)"), acq["basis"])
        # a published line past the budget, alone: not determined either
        late = (130, "published frame_id=0 seq=8 198 B")
        self.assertEqual(rep.acquisition(rx(late), tx)["status"], rep.UNDETERMINED)
        # ... FAILED once the counter shows nothing by +125 s, or per-fragment logging
        # shows itself on
        acq = rep.acquisition(rx((125, "stats: rx_frames=0"), late), tx)
        self.assertEqual(acq["status"], rep.FAILED)
        self.assertTrue(acq["basis"].startswith(
            "the base's stats counter still read rx_frames=0 at +125.0 s (rx_daemon.log line 4; "
            "always on, it counts every image fragment, logged or not): no image fragment "
            "reached the base within the 120 s budget"), acq["basis"])
        acq = rep.acquisition(rx((130, "frag_arrival: seq=8 idx=0 total=1"), late), tx)
        self.assertEqual(acq["status"], rep.FAILED)
        # the counter's baseline is its value at the activation
        acq = rep.acquisition("2026-10-24 15:34:59,000 INFO x: stats: rx_frames=7\n"
                              + rx((125, "stats: rx_frames=7")), tx)
        self.assertEqual((acq["base_rx"], acq["status"], acq["rx_frags"]), (7, rep.FAILED, 0))
        # ... and the fragments the base decoded count from it
        acq = rep.acquisition("2026-10-24 15:34:59,000 INFO x: stats: rx_frames=7\n"
                              + rx((20, "stats: rx_frames=307")), tx)
        self.assertEqual((acq["rx_frags"], acq["status"]), (300, rep.ACQUIRED))

    def test_a_bracket_demotion_shows_the_lock(self) -> None:
        # rs12_leg_report.py --pre/--post (leg_post.sh passes the base's): a
        # LOCKED -> SCANNING demotion, which only a locked scan makes
        pre = {"clk_demotion_reset": 4, "clk_demotion_kept": 0, "radio_rx_ok": 10}
        post = {"clk_demotion_reset": 5, "clk_demotion_kept": 0, "radio_rx_ok": 400}
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), heard=(40, 300, 560), capture=False)
            rx, tx = _read(arch / "rx_daemon.log"), _read(arch / "tx_daemon.log")
        acq = rep.acquisition(rx, tx, pre, post)
        self.assertEqual(acq["status"], rep.ACQUIRED)
        self.assertEqual(acq["basis"], "the base's bracket counts 1 LOCKED -> SCANNING "
                                       "demotion(s) (d(clk_demotion_reset) 1, "
                                       "d(clk_demotion_kept) 0), which only a locked scan makes")
        acq = rep.acquisition(rx, tx, pre, dict(post, clk_demotion_reset=4))
        self.assertEqual(acq["status"], rep.UNDETERMINED)
        self.assertIn("; the base's bracket counts no LOCKED -> SCANNING demotion; ",
                      acq["basis"])
        # an L072 reset between the brackets restarts the counters: the post values
        self.assertEqual(rep.bracket_demotions(
            {"clk_demotion_reset": 9, "clk_demotion_kept": 0, "radio_rx_ok": 500},
            {"clk_demotion_reset": 1, "clk_demotion_kept": 0, "radio_rx_ok": 3}), (1, 0))
        self.assertIsNone(rep.bracket_demotions({"clk_demotion_reset": 1}, post))
        self.assertIsNone(rep.bracket_demotions(None, None))
        # a demotion that kept a self-anchored clock is a lock lost too (sx1276_rx.c
        # counts it as kept, not reset)
        acq = rep.acquisition(rx, tx, pre, dict(post, clk_demotion_reset=4, clk_demotion_kept=1))
        self.assertEqual(acq["status"], rep.ACQUIRED)
        self.assertIn("(d(clk_demotion_reset) 0, d(clk_demotion_kept) 1)", acq["basis"])

    def test_silence_that_a_lock_or_the_firmware_contradicts_is_not_a_failure(self) -> None:
        # review 2026-10-24: the host's silence (its stats counter flat past the budget,
        # per-fragment logging on with the first frag_arrival past it) is FAILED evidence
        # only while nothing contradicts it. The scan's FAILED is absorbing, so a lock
        # shown anywhere in the leg means the host missed what the base received (UART
        # contention, a stalled RX worker, lost URCs, a lock on a command frame)
        stats = [(s, f"INFO x: stats: rx_frames={0 if s < 138 else 2 * (s - 130)}")
                 for s in range(10, 320, 10)]

        def acq(mask=False, pre=None, post=None):
            with tempfile.TemporaryDirectory() as d:
                arch, _ = _two_board_leg(Path(d), capture=False, mask=mask,
                                         heard=range(261, 607), base_extra=stats)
                return rep.acquisition(_read(arch / "rx_daemon.log"),
                                       _read(arch / "tx_daemon.log"), pre, post)
        alone = acq()
        self.assertEqual(alone["status"], rep.FAILED)
        self.assertTrue(alone["basis"].startswith(
            "the base's stats counter still read rx_frames=0 at +128.6 s (rx_daemon.log line "),
            alone["basis"])
        # the share decoded on the 50-channel hop set: 346/606 = 57 %
        got = acq(mask=True)
        self.assertEqual((got["status"], got["failed"]), (rep.UNDETERMINED, None))
        self.assertIn("first came at +136.6 s: no image fragment reached the base within the "
                      "120 s budget by the host's count, but the base decoded 346 of the "
                      "tractor's 606 fragments on air (57.1 %): on its 50-channel hop set",
                      got["basis"])
        self.assertTrue(got["basis"].endswith(
            ". The scan's FAILED is absorbing (only a re-activation leaves it), so this silence "
            "is the host's, not the scan's: its RX path missed what the base received (UART "
            "contention, a stalled RX worker, lost URCs), or the base locked on a command "
            "frame, which rx_frames does not count"), got["basis"])
        line = rep.format_acquisition(got)[1]
        self.assertTrue(line.startswith("  confirmed acquisition: NOT DETERMINED: "), line)
        # a base-bracket demotion alone
        pre = {"clk_demotion_reset": 0, "clk_demotion_kept": 0, "radio_rx_ok": 0}
        got = acq(pre=pre, post=dict(pre, clk_demotion_reset=1, radio_rx_ok=346))
        self.assertEqual(got["status"], rep.UNDETERMINED)
        self.assertIn("by the host's count, but the base's bracket counts 1 LOCKED -> SCANNING "
                      "demotion(s)", got["basis"])
        # the bracket's radio_rx_ok well above the host's count (a deaf host)
        got = acq(pre=dict(pre, rx_urc_lost=0),
                  post=dict(pre, radio_rx_ok=550, rx_urc_lost=2))
        self.assertEqual(got["status"], rep.UNDETERMINED)
        self.assertIn("but the base's bracket counts d(radio_rx_ok) 550 against the host's 346 "
                      "(FIRMWARE DROP 204, d(rx_urc_lost) 2): the L072 delivered frames the host "
                      "never counted", got["basis"])
        # ... but not the log-window drop every bracketed leg shows (0-6)
        got = acq(pre=dict(pre, rx_urc_lost=0), post=dict(pre, radio_rx_ok=352, rx_urc_lost=0))
        self.assertEqual(got["status"], rep.FAILED)
        # firmware evidence still wins over a lock shown: a cold final
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), capture=False, mask=True, heard=range(261, 607),
                                     base_extra=stats + [(121.6, "WARNING x: FAULT_URC: "
                                                         "code=0x0D sub=0x83 name=RX_SCAN_"
                                                         "FAILED uptime_ms=1 raw=0d83")])
            got = rep.acquisition(_read(arch / "rx_daemon.log"), _read(arch / "tx_daemon.log"))
        self.assertEqual(got["status"], rep.FAILED)

    def test_a_dead_rx_worker_is_no_instrument(self) -> None:
        # review 2026-10-24: bench-evidence radio_monitor_20260724_154301_c522dcc3's shape.
        # Its image-rx-worker thread died (an AttributeError) right after it started,
        # and its stats thread kept printing rx_frames=0 every 10 s: that counter is
        # frozen, not silence the scan made
        crash = ["Exception in thread image-rx-worker:", "Traceback (most recent call last):",
                 "AttributeError: 'FragmentReassembler' object has no attribute 'tick'"]

        def rx(crash_after_s, until_s=210):
            out = ["2026-10-24 15:34:59,600 INFO x: opening L072 HostLink",
                   "  FHSS mask: WIDE (50 channels) [LIFETRAC_FHSS_WIDE_MASK=1]",
                   "CFG_SET_REQ(REG_PROFILE=1) OK: 14000100",
                   "2026-10-24 15:35:00,000 INFO x: LBT_ENABLE=0"]
            for s in range(10, until_s + 1, 10):
                if crash_after_s is not None and s - 10 < crash_after_s <= s:
                    out += crash
                out.append(f"2026-10-24 {15 + (35 * 60 + s) // 3600:02d}:"
                           f"{(35 * 60 + s) // 60 % 60:02d}:{s % 60:02d},000 INFO x: stats: "
                           f"rx_frames=0 frames_published=0")
            return "\n".join(out) + "\n"
        tx = "x frame seq=1 done (pipelined): 600 fragments ok\n"
        self.assertEqual(rep.acquisition(rx(None), tx)["status"], rep.FAILED)
        got = rep.acquisition(rx(5), tx)
        self.assertEqual((got["status"], got["crash"]), (rep.UNDETERMINED, (0.0, 5)))
        self.assertIsNone(got["quiet"])
        self.assertEqual(got["basis"], "the base's log holds no frame line, but no instrument "
                                       "known to be on shows that no frame reached it within the "
                                       "budget; the base decoded 0 of the tractor's 600 fragments "
                                       "on air (0.0 %), under the 25 % that shows a receiver "
                                       "following its 50-channel hop grid; the base's "
                                       "image-rx-worker thread died at +0.0 s (rx_daemon.log "
                                       "line 5): its stats counter and frag_arrival lines stop "
                                       "counting there")
        # a worker that died after the budget: its counter read 0 past it first
        got = rep.acquisition(rx(135), tx)
        self.assertEqual((got["status"], got["quiet"][0]), (rep.FAILED, 130.0))

    def test_a_dead_stats_thread_leaves_a_floor(self) -> None:
        # review 2026-10-24: bench-evidence radio_monitor_20260915_130259_aa74cde2 (leg U)
        # shape: the stats thread died one frame in, so its last rx_frames=1 is no
        # count. With per-fragment logging off, the published frames are the floor
        def acq(heard):
            with tempfile.TemporaryDirectory() as d:
                arch, _ = _two_board_leg(Path(d), capture=False, mask=True, heard=heard)
                rx, tx = _read(arch / "rx_daemon.log"), _read(arch / "tx_daemon.log")
            rx = "\n\n".join(ln for ln in rx.split("\n\n") if "frag_arrival" not in ln)
            rx = re.sub(r"stats: rx_frames=\d+", "stats: rx_frames=1", rx).replace(
                "RX worker ready", "RX worker ready\n\nException in thread image-rx-stats:\n\n"
                                   "Traceback (most recent call last):\n\nx")
            return rep.acquisition(rx, tx)
        got = acq(range(139, 607))
        self.assertEqual((got["status"], got["rx_frags"], got["rx_exact"]),
                         (rep.ACQUIRED, 468, False))
        self.assertTrue(got["basis"].startswith(
            "the base decoded at least 468 of the tractor's 606 fragments on air (at least "
            "77.2 %): on its 50-channel hop set"), got["basis"])
        got = acq((40, 300, 560))
        self.assertEqual(got["status"], rep.UNDETERMINED)
        self.assertIn("; the base's fragment count is unknown: its stats thread died "
                      "(rx_daemon.log holds 'Exception in thread image-rx-stats') and "
                      "per-fragment logging was off, so its published frames give only a floor "
                      "(the base decoded at least 3 of the tractor's 606 fragments on air (at "
                      "least 0.5 %))", got["basis"])
        # on a leg whose TX log shows a batched train, the published frames of one
        # train share its seq: one fragment at least per run of a seq
        rx = ("  FHSS mask: WIDE (50 channels)\nCFG_SET_REQ(REG_PROFILE=1) OK: 1\n"
              "2026-10-24 15:35:00,000 INFO x: LBT_ENABLE=0\n"
              + "".join(f"2026-10-24 15:35:0{k},000 INFO x: published frame_id=0 seq={s} 9 B\n"
                        for k, s in enumerate((5, 5, 6, 7, 7))))
        self.assertEqual(rep.acquisition(rx, "x: frame seq=5 done: 9 fragments ok\n")["rx_frags"],
                         5)
        self.assertEqual(rep.acquisition(rx, "x: batching: 2 frames in 1 trains so far\nx: frame "
                                             "seq=5 done: 9 fragments ok\n")["rx_frags"], 3)

    def test_rules_that_need_their_own_case(self) -> None:
        # review 2026-10-24 (mutation run): each of these survived every other test.
        # The firmware's defensive 0x80 marker (a final attempt 0 with warm=0) is no
        # evidence: a locked leg that logs one stays ACQUIRED, a leg with no lock
        # shown NOT DETERMINED
        marker = (31.4, "WARNING x: FAULT_URC: code=0x0D sub=0x80 name=RX_SCAN_FAILED "
                        "uptime_ms=1 raw=0d80")
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(self._acq(Path(d), heard=range(139, 607), mask=True,
                                       base_extra=[marker])["status"], rep.ACQUIRED)
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(self._acq(Path(d), heard=(40, 300, 560),
                                       base_extra=[marker])["status"], rep.UNDETERMINED)
        # a single-channel hop set: a high share shows no grid following
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), capture=False, mask=True, heard=range(139, 607))
            rx = _read(arch / "rx_daemon.log").replace(
                "FHSS mask: WIDE (50 channels) [LIFETRAC_FHSS_WIDE_MASK=1]",
                "FHSS mask: single channel 0 (0100000000000000)")
            got = rep.acquisition(rx, _read(arch / "tx_daemon.log"))
        self.assertEqual((got["status"], got["channels"]), (rep.UNDETERMINED, 1))
        self.assertIn("rx_daemon.log names no FHSS hop set of 10 or more channels",
                      got["basis"])
        # the hop set is the start-up configuration's (printed before its REG_PROFILE
        # line), not a mask printed later
        streamed = "x frame seq=1 done (pipelined): 600 fragments ok\n"
        got = rep.acquisition(rx + "\n\n  FHSS mask: WIDE (50 channels)\n", streamed)
        self.assertEqual(got["channels"], 1)
        got = rep.acquisition(rx.replace("single channel 0 (0100000000000000)",
                                         "WIDE (50 channels)")
                              + "\n\n  FHSS mask: single channel 0 (0100000000000000)\n", streamed)
        self.assertEqual(got["channels"], 50)
        # a reception the stats counter dates within the budget is within it, even
        # with the first frag_arrival line past it (a frame of several fragments)
        rx = ("  FHSS mask: WIDE (50 channels)\nCFG_SET_REQ(REG_PROFILE=1) OK: 1\n"
              "2026-10-24 15:35:00,000 INFO x: LBT_ENABLE=0\n"
              "2026-10-24 15:35:20,000 INFO x: stats: rx_frames=4\n"
              "2026-10-24 15:37:10,000 INFO x: frag_arrival: seq=8 idx=0 total=1\n"
              "2026-10-24 15:40:00,000 INFO x: stats: rx_frames=9\n")
        got = rep.acquisition(rx, "x frame seq=1 done (pipelined): 600 fragments ok\n")
        self.assertEqual((got["heard"][0], got["first_frag"], got["status"]),
                         (20.0, 130.0, rep.UNDETERMINED))

    def test_no_frame_at_all(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), heard=())
        # no frame line, and the always-on counter still read rx_frames=0 at the end
        self.assertEqual(acq["status"], rep.FAILED)
        self.assertIsNone(acq["first"])
        lines = rep.format_acquisition(acq)
        self.assertIn("acquisition: first image reception: none, the base decoded no frame in "
                      "the 316.6 s its log covers after its profile-1 activation", lines[0])
        self.assertIn(", past the FHSS cold-start scan budget", lines[0])
        self.assertTrue(lines[1].startswith("  !! ACQUISITION FAILED: the base's stats counter "
                                            "still read rx_frames=0 at +315.6 s"), lines[1])
        # a log too short to tell
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), heard=(), sent=100)
        self.assertEqual((acq["status"], acq["failed"]), (rep.UNDETERMINED, None))
        lines = rep.format_acquisition(acq)
        self.assertIn(", no longer than the FHSS cold-start scan budget", lines[0])
        self.assertTrue(lines[1].startswith("  confirmed acquisition: NOT DETERMINED: the base's "
                                            "log covers 63.6 s, no more than the budget"),
                        lines[1])
        self.assertEqual(len(lines), 2)
        # the tractor sent nothing: the flag says so
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), heard=(), capture=False)
            acq = rep.acquisition(_read(arch / "rx_daemon.log"), "")
        self.assertIn("The tractor logged 0 frame(s) on air. It sent nothing",
                      rep.format_acquisition(acq)[1])

    def test_not_fhss_or_no_activation(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), profile=2)
        self.assertFalse(acq["applies"])
        self.assertEqual(rep.format_acquisition(acq),
                         ["acquisition: n/a (the base daemon started on reg_profile 2 (DTS): "
                          "no FHSS scan)"])
        self.assertEqual(rep.format_acquisition(rep.acquisition("x: started\n")),
                         ["acquisition: n/a (rx_daemon.log has no REG_PROFILE activation "
                          "line)"])
        # review 2026-10-24: a later activation is a profile switch, which stops the
        # FHSS scan (host_cfg_profile.c), so the window ends there: what the base
        # decoded on the other profile is no lock, and the bracket spans both
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), heard=range(30, 606), mask=True,
                            base_extra=[(200.0, "INFO x: radio_profile: switching"),
                                        (200.5, "CFG_SET_REQ(REG_PROFILE=2) OK: 1")])
        self.assertEqual((acq["later"], acq["status"]), (1, rep.UNDETERMINED))
        lines = rep.format_acquisition(acq)
        self.assertTrue(lines[0].endswith(
            ", within the FHSS cold-start scan budget of 120 s (4 attempts x 30 s: "
            "SX1276_RX_SCAN_MAX_RETRIES 3 + 1, SX1276_RX_SCAN_REDESIGN_MS 30000); read up to its "
            "switch to profile 2 (rx_daemon.log line 1437, +198.6 s)"), lines[0])
        self.assertEqual(lines[1], "  confirmed acquisition: NOT DETERMINED: a frame received is "
                                   "not a lock (sx1276_rx.c still delivers a frame whose FHSS "
                                   "header it rejected), and nothing here shows one; the base "
                                   "switched to profile 2 at rx_daemon.log line 1437 (+198.6 s), "
                                   "which stops the FHSS scan: what it decoded after that is not "
                                   "the scan's, so neither its share of the tractor's fragments "
                                   "nor a bracket demotion is weighed")
        self.assertEqual(len(lines), 2)
        # the same leg without the switch reads ACQUIRED on the share
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(self._acq(Path(d), heard=range(30, 606), mask=True)["status"],
                             rep.ACQUIRED)

    def test_the_window_ends_at_a_profile_switch(self) -> None:
        # review 2026-10-24: bench-evidence radio_monitor_20260726_133257_9a633f1c's
        # shape. Scan attempts fail at +30 / +60 s, the base switches to DTS at +70 s
        # and decodes every frame after that: not ACQUIRED (the frames are DTS's), and
        # its silence on FHSS is no run-out either (the switch stopped the scan)
        dts = [(1.4 + 30.0, "WARNING x: FAULT_URC: code=0x0D sub=0x50 name=RX_SCAN_FAILED "
                            "uptime_ms=1 raw=0d50"),
               (1.4 + 60.0, "WARNING x: FAULT_URC: code=0x0D sub=0x51 name=RX_SCAN_FAILED "
                            "uptime_ms=1 raw=0d51"),
               (71.0, "INFO x: radio_profile: tractor ACK for 2 — switching locally"),
               (71.3, "  FHSS mask: single channel 0 (0100000000000000)"),
               (71.39, "CFG_SET_REQ(REG_PROFILE=2) OK: 14000100")]
        stats = [(s, f"INFO x: stats: rx_frames={0 if s < 80 else s}") for s in range(10, 300, 10)]
        rx_tx = None
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), capture=False, mask=True, heard=range(150, 606),
                                     base_extra=dts + stats)
            rx_tx = (_read(arch / "rx_daemon.log"), _read(arch / "tx_daemon.log"))
            pre = {"clk_demotion_reset": 0, "clk_demotion_kept": 0, "radio_rx_ok": 0}
            acq = rep.acquisition(*rx_tx, pre, dict(pre, clk_demotion_reset=1, radio_rx_ok=456))
        self.assertEqual((acq["status"], acq["first"]), (rep.UNDETERMINED, None))
        self.assertEqual((acq["cut"]["profile"], round(acq["span"], 1)), ("2", 69.9))
        lines = rep.format_acquisition(acq)
        self.assertIn("acquisition: first image reception: none, the base decoded no frame in "
                      "the 69.9 s from its profile-1 activation (rx_daemon.log line 7 at "
                      "15:35:36.531 board time) to its profile switch, no longer than the FHSS "
                      "cold-start scan budget", lines[0])
        self.assertTrue(lines[1].startswith(
            "  confirmed acquisition: NOT DETERMINED: the scan ran 69.9 s before the base's "
            "profile switch, no more than the budget; the base switched to profile 2"),
            lines[1])
        self.assertIn("attempt 0 at +30.0 s", lines[2])
        # the scorer's phrase names the switch too
        self.assertTrue(rep.reception_phrase(acq).startswith(
            "no image reception in the 69.9 s from the base's profile-1 activation to its switch "
            "to profile 2, no longer than"), rep.reception_phrase(acq))
        # a cold final inside the window still fails it: the switch came at +200 s
        late = [(121.4, "WARNING x: FAULT_URC: code=0x0D sub=0x83 name=RX_SCAN_FAILED "
                        "uptime_ms=1 raw=0d83"),
                (200.0, "CFG_SET_REQ(REG_PROFILE=2) OK: 14000100")]
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), mask=True, heard=range(400, 606), base_extra=late)
        self.assertEqual(acq["status"], rep.FAILED)
        # so does the host's silence past the budget: the bracket's radio_rx_ok, which
        # counts the frames decoded after the switch too, does not contradict it
        quiet = [(s, "INFO x: stats: rx_frames=0") for s in range(10, 200, 10)]
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), capture=False, mask=True, heard=range(400, 606),
                                     base_extra=quiet + late[1:])
            pre = {"clk_demotion_reset": 0, "clk_demotion_kept": 0, "radio_rx_ok": 0}
            acq = rep.acquisition(_read(arch / "rx_daemon.log"), _read(arch / "tx_daemon.log"),
                                  pre, dict(pre, clk_demotion_reset=1, radio_rx_ok=206))
        self.assertEqual((acq["status"], acq["rx_frags"]), (rep.FAILED, 0))
        self.assertTrue(acq["basis"].startswith("the base's stats counter still read "
                                                "rx_frames=0 at +188.6 s"), acq["basis"])
        # and the committed archive itself, when this tree has it
        arch = _BS.parent / "bench-evidence" / "radio_monitor_20260726_133257_9a633f1c"
        if (arch / "rx_daemon.log").is_file():
            out = _run([arch])
            self.assertIn("; read up to its switch to profile 2 (rx_daemon.log line 177, ", out)
            self.assertIn("  confirmed acquisition: NOT DETERMINED: the scan ran ", out)
            self.assertNotIn("ACQUIRED", out)

    def test_scan_fault_sub_byte(self) -> None:
        # sx1276_rx_scan_fail.h: final 7, warm 6, hw_suspect 5, crc_seen 4, attempt 3..0
        cases = {0x20: (0, False, False, True, False), 0x01: (1, False, False, False, False),
                 0x42: (2, False, True, False, False), 0x12: (2, False, False, False, True),
                 0x83: (3, True, False, False, False), 0xA3: (3, True, False, True, False),
                 0xC3: (3, True, True, False, False), 0xD3: (3, True, True, False, True)}
        for sub, want in cases.items():
            got = rep.scan_fault_sub(sub)
            self.assertEqual((got["attempt"], got["final"], got["warm"], got["hw_suspect"],
                              got["crc_seen"], got["marker"]), want + (False,), hex(sub))
        # 0x80: the final attempt 0 sx1276_rx_scan_fail_eval sends for a NULL input
        # (review 2026-10-24: the daemons' decoder names it RX_SCAN_FAILED)
        got = rep.scan_fault_sub(0x80)
        self.assertEqual((got["attempt"], got["final"], got["hw_suspect"], got["marker"]),
                         (0, True, False, True))
        # not the layout: TX_FIFO_RB_MISMATCH (sub = tx_id)
        for sub in (0x03, 0x81, 0x30, 0x05, 0xFF, 0x84):
            self.assertIsNone(rep.scan_fault_sub(sub), hex(sub))
        # strict=False (the daemon named it RX_SCAN_FAILED) reads the bits as they are
        got = rep.scan_fault_sub(0x84, strict=False)
        self.assertEqual((got["attempt"], got["final"], got["marker"]), (4, True, False))

    def test_scan_sub_agrees_with_the_daemons_decoder(self) -> None:
        # the same 25 of 256 values as rx_scan_fail_sub_valid (method_g_stage1_probe.py,
        # which names each 0x0D FAULT_URC in the daemons' logs)
        helper = str(_BS.parent / "firmware" / "x8_lora_bootloader_helper")
        if helper not in sys.path:
            sys.path.insert(0, helper)
        import method_g_stage1_probe as mg
        valid = [s for s in range(256) if mg.rx_scan_fail_sub_valid(s)]
        self.assertEqual(len(valid), 25)
        self.assertEqual([s for s in range(256) if rep.scan_fault_sub(s) is not None], valid)

    def test_the_first_frame_at_the_budget_boundary(self) -> None:
        # whole-second stamps, per-fragment logging on: the first frame exactly 120.0 s
        # after the activation is within the budget (no evidence of a failure, nor of a
        # lock), 1 ms later past it (FAILED); the always-on counter likewise
        def acq(ms, line="frag_arrival: seq=8 idx=0 total=1 fw_us=1 len=202"):
            rx = ("CFG_SET_REQ(REG_PROFILE=1) OK: 14000100\n"
                  "2026-10-24 15:35:00,000 INFO image_rx_daemon: LBT_ENABLE=0\n"
                  f"2026-10-24 15:37:00,{ms:03d} INFO image_rx_daemon: {line}\n")
            return rep.acquisition(rx)
        got = acq(0)
        self.assertEqual((got["first"], got["status"], got["failed"]),
                         (120.0, rep.UNDETERMINED, None))
        self.assertEqual((acq(1)["status"], acq(1)["failed"]), (rep.FAILED, True))
        # a published line alone shows no instrument on (a frame can take several
        # fragments): not determined either side of the boundary
        self.assertEqual(acq(1, "published frame_id=0 seq=8 198 B")["status"], rep.UNDETERMINED)
        got = acq(0, "stats: rx_frames=0")
        self.assertEqual((got["span"], got["first"], got["quiet"], got["status"]),
                         (120.0, None, (120.0, 3), rep.UNDETERMINED))
        self.assertEqual(acq(1, "stats: rx_frames=0")["status"], rep.FAILED)

    def test_base_fault_lines_in_the_legacy_format(self) -> None:
        # the tractor's legacy "FAULT during pipelined TX:" shape
        # (format_fault_payload), and other FAULT lines with code= sub= and no name=
        raw = "raw=0d20000000000000000000000000000000000000dc840000"
        extra = [(31.4, f"WARNING image_rx_daemon: FAULT during pipelined TX: code=0x0D "
                        f"sub=0x20 {raw}"),
                 (61.4, "WARNING image_rx_daemon: FAULT_URC: code=0x0D sub=0x21 raw=0d21"),
                 (91.4, "WARNING image_rx_daemon: FAULT: code=0x0d sub=0x22 raw=0d22"),
                 (100.0, "WARNING image_rx_daemon: FAULT: code=0x0B sub=0x01 "
                         "parse_err_count_hint=1 raw=0b01"),
                 (121.4, "WARNING image_rx_daemon: FAULT: code=0x0D sub=0xA3 raw=0da3"),
                 (130.0, "WARNING image_rx_daemon: FAULT: code=0x0D sub=0x07 raw=0d07")]
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), capture=False, base_extra=extra)
            # the untimestamped stdout line a URC can also leave (wait_for_tx_done) is
            # not counted, in a log without FAULT_URC name= lines too
            rx = _read(arch / "rx_daemon.log") + \
                "FAULT_URC during TX wait: code=0x0D sub=0xA3 raw=0da3\n"
            acq = rep.acquisition(rx, _read(arch / "tx_daemon.log"))
        self.assertFalse(acq["logged"])
        self.assertEqual([(round(f["t"], 1), f["sub"]) for f in acq["faults"]],
                         [(30.0, 0x20), (60.0, 0x21), (90.0, 0x22), (120.0, 0xA3),
                          (128.6, 0x07)])
        lines = rep.format_acquisition(acq)
        self.assertTrue(lines[1].startswith("  !! ACQUISITION FAILED"))
        self.assertEqual(lines[2], "  base RX_SCAN_FAILED fault(s), FAULT code 0x0D read by "
                                   "the scan sub-byte layout: attempt 0 at +30.0 s (line 9, "
                                   "sub 0x20: hw_suspect); attempt 1 at +60.0 s (line 11, sub "
                                   "0x21: hw_suspect); attempt 2 at +90.0 s (line 13, sub "
                                   "0x22: hw_suspect); attempt 3 at +120.0 s (line 17, sub "
                                   "0xA3: final, hw_suspect)")
        self.assertEqual(lines[3], "  !! the base's final RX_SCAN_FAILED at +120.0 s "
                                   "(rx_daemon.log line 17, sub 0xA3): its scan entered "
                                   "FAILED there")
        self.assertEqual(lines[4], "  base FAULT code 0x0D read as TX_FIFO_RB_MISMATCH (sub = "
                                   "the fragment's tx_id; not in the scan sub-byte layout): "
                                   "sub 0x07 at +128.6 s (line 19)")
        self.assertEqual(len(lines), 5)                 # no "inferred" note
        # the cold final (0xA3, warm=0) is itself evidence of the failure
        self.assertTrue(acq["basis"].startswith(
            "the base's final RX_SCAN_FAILED at +120.0 s (rx_daemon.log line 17, sub 0xA3) is "
            "cold (warm=0)"), acq["basis"])
        # a locked leg's earlier attempts print without a flag
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), heard=range(139, 607), base_extra=extra[:2], mask=True)
        lines = rep.format_acquisition(acq)
        self.assertEqual(len(lines), 3)
        self.assertEqual(acq["status"], rep.ACQUIRED)
        self.assertNotIn("!!", "\n".join(lines))
        self.assertIn("attempt 0 at +30.0 s", lines[2])

    def test_base_fault_urc_lines_of_2026_10_24(self) -> None:
        # image_rx_daemon.py from 2026-10-24: one "FAULT_URC: ... name=" line per
        # URC (format_fault_urc_line); a legacy line for the same URC is not
        # counted twice, nor are the untimestamped stdout lines
        def urc(sub, name, extra=""):
            return (f"WARNING image_rx_daemon: FAULT_URC: code=0x0D sub=0x{sub:02X} "
                    f"name={name}{extra} basis=x tx_inflight=- uptime_ms=1 raw=0d")
        extra = [(31.4, urc(0x20, "RX_SCAN_FAILED", " attempt=0 final=0 warm=0 "
                                                    "hw_suspect=1 crc_seen=0")),
                 (31.41, "WARNING image_rx_daemon: FAULT during pipelined TX: code=0x0D "
                         "sub=0x20 raw=0d20"),
                 (45.0, "INFO image_rx_daemon: FAULT_URC: code=0x09 sub=0x01 name="
                        "HOST_RX_SEEN lanes=LPUART1 uptime_ms=1 raw=0901"),
                 (61.4, urc(0x21, "AMBIGUOUS")),
                 (70.0, urc(0x02, "TX_FIFO_RB_MISMATCH", " tx_id=0x02")),
                 (121.4, urc(0xA3, "RX_SCAN_FAILED"))]
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), capture=False, base_extra=extra)
            rx = _read(arch / "rx_daemon.log") + \
                "FAULT_URC during TX wait: code=0x0D sub=0x22 raw=0d22\n"
            acq = rep.acquisition(rx, _read(arch / "tx_daemon.log"))
        self.assertTrue(acq["logged"])
        self.assertEqual([(round(f["t"], 1), f["sub"], f["name"]) for f in acq["faults"]],
                         [(30.0, 0x20, "RX_SCAN_FAILED"), (60.0, 0x21, "AMBIGUOUS"),
                          (68.6, 0x02, "TX_FIFO_RB_MISMATCH"), (120.0, 0xA3, "RX_SCAN_FAILED")])
        lines = rep.format_acquisition(acq)
        self.assertEqual(lines[2], "  base RX_SCAN_FAILED fault(s), FAULT code 0x0D read by the "
                                   "daemon's FAULT_URC name= and the scan sub-byte layout: "
                                   "attempt 0 at +30.0 s (line 9, sub 0x20: hw_suspect); "
                                   "attempt 1 at +60.0 s (line 15, sub 0x21: hw_suspect, "
                                   "AMBIGUOUS with TX_FIFO_RB_MISMATCH); attempt 3 at +120.0 s "
                                   "(line 19, sub 0xA3: final, hw_suspect)")
        self.assertIn("  !! the base's final RX_SCAN_FAILED at +120.0 s (rx_daemon.log line 19",
                      lines[3])
        self.assertEqual(lines[4], "  base FAULT code 0x0D read as TX_FIFO_RB_MISMATCH (sub = "
                                   "the fragment's tx_id; by name= or not in the scan sub-byte "
                                   "layout): sub 0x02 at +68.6 s (line 17)")
        # review 2026-10-24: the daemon's name= both ways, on the lines its own
        # format_fault_urc_line writes. The defensive 0x80 marker it names
        # RX_SCAN_FAILED is a final scan fault; a final it names AMBIGUOUS says so
        # on the !! line
        helper = str(_BS.parent / "firmware" / "x8_lora_bootloader_helper")
        if helper not in sys.path:
            sys.path.insert(0, helper)
        import method_g_stage1_probe as mg

        def named(pc, sub, in_flight=frozenset()):
            payload = bytes([0x0D, sub, 0, 0]) + bytes(16) + int(pc * 1000).to_bytes(4, "little")
            return (pc, "WARNING image_rx_daemon: " + mg.format_fault_urc_line(payload, in_flight))
        with tempfile.TemporaryDirectory() as d:
            lines = rep.format_acquisition(self._acq(Path(d), base_extra=[named(31.4, 0x80)]))
        self.assertIn("name=RX_SCAN_FAILED", named(31.4, 0x80)[1])
        self.assertEqual(lines[2], "  base RX_SCAN_FAILED fault(s), FAULT code 0x0D read by the "
                                   "daemon's FAULT_URC name= and the scan sub-byte layout: "
                                   "attempt 0 at +30.0 s (line 9, sub 0x80: final, the "
                                   "firmware's defensive NULL-input marker)")
        self.assertEqual(lines[3], "  !! the base's final RX_SCAN_FAILED at +30.0 s (rx_daemon.log "
                                   "line 9, sub 0x80): its scan entered FAILED there")
        self.assertEqual(len(lines), 4)                 # nothing read as TX_FIFO_RB_MISMATCH
        # a final named AMBIGUOUS, as PR #166's daemons wrote it (before the 0x0D
        # split) when its sub was an in-flight tx_id
        amb = (121.4, urc(0xA3, "AMBIGUOUS", " attempt=3 final=1 warm=0 hw_suspect=1 "
                                             "crc_seen=0 tx_id=0xA3"))
        with tempfile.TemporaryDirectory() as d:
            lines = rep.format_acquisition(self._acq(Path(d), base_extra=[
                named(31.4, 0x20), amb]))
        self.assertIn("attempt 3 at +120.0 s (line 11, sub 0xA3: final, hw_suspect, AMBIGUOUS "
                      "with TX_FIFO_RB_MISMATCH)", lines[2])
        self.assertEqual(lines[3], "  !! the base's final RX_SCAN_FAILED at +120.0 s "
                                   "(rx_daemon.log line 11, sub 0xA3, AMBIGUOUS with "
                                   "TX_FIFO_RB_MISMATCH): its scan entered FAILED there if this "
                                   "URC was the scan's (the daemon could not rule out a TX FIFO "
                                   "readback mismatch on a TX with that tx_id)")
        # 2026-10-24, the 0x0D split: the decoder now names that URC RX_SCAN_FAILED
        # (the in-flight tx_id stays in basis=), so the report reads a plain final
        self.assertIn("name=RX_SCAN_FAILED attempt=3 final=1 warm=0 hw_suspect=1 "
                      "crc_seen=0 basis=tx_id_in_flight tx_inflight=0xA3 ",
                      named(121.4, 0xA3, frozenset({0xA3}))[1])
        with tempfile.TemporaryDirectory() as d:
            lines = rep.format_acquisition(self._acq(Path(d), base_extra=[
                named(31.4, 0x20), named(121.4, 0xA3, frozenset({0xA3}))]))
        self.assertIn("attempt 3 at +120.0 s (line 11, sub 0xA3: final, hw_suspect)", lines[2])
        self.assertEqual(lines[3], "  !! the base's final RX_SCAN_FAILED at +120.0 s "
                                   "(rx_daemon.log line 11, sub 0xA3): its scan entered "
                                   "FAILED there")
        # a split image's FIFO fault is code 0x0F: never one of the base's scan faults
        with tempfile.TemporaryDirectory() as d:
            payload = bytes([0x0F, 0xFD, 0, 0]) + bytes(16) + (61400).to_bytes(4, "little")
            acq = self._acq(Path(d), base_extra=[
                named(31.4, 0x20),
                (61.4, "WARNING image_rx_daemon: " + mg.format_fault_urc_line(
                    payload, frozenset({0xFD})))])
        self.assertEqual([(f["code"], f["sub"]) for f in acq["faults"]], [(0x0D, 0x20)])
        self.assertNotIn("TX_FIFO_RB_MISMATCH", "\n".join(rep.format_acquisition(acq)))
        # a sub outside today's layout that the daemon still names RX_SCAN_FAILED (a
        # firmware that raised its retry budget) reads as a scan fault, not a FIFO one
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), base_extra=[(31.4, urc(0x84, "RX_SCAN_FAILED"))])
        self.assertEqual([(f["sub"], f["scan"]["attempt"], f["scan"]["final"])
                          for f in acq["faults"]], [(0x84, 4, True)])
        # review 2026-10-24: FAULT_URC lines but no 0x0D one. This daemon logs every
        # FAULT_URC and a scan that runs out sends one per attempt, so the silence
        # the host shows contradicts a run-out: not determined, not FAILED
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), base_extra=[extra[2]])
        self.assertEqual((acq["logged"], acq["status"]), (True, rep.UNDETERMINED))
        lines = rep.format_acquisition(acq)
        self.assertEqual(len(lines), 2)
        self.assertTrue(lines[1].startswith(
            "  confirmed acquisition: NOT DETERMINED: per-fragment logging was on (rx_daemon.log "
            "holds frag_arrival lines) and the first came at +138.1 s"), lines[1])
        self.assertIn("no image fragment reached the base within the 120 s budget by the host's "
                      "count, but this daemon logs every FAULT_URC (rx_daemon.log holds "
                      "FAULT_URC lines) and a scan that runs out sends an RX_SCAN_FAILED one "
                      "per attempt, yet the window holds none. The scan's FAILED is absorbing",
                      lines[1])
        # the same with its four attempts logged: FAILED, the cold final seen
        with tempfile.TemporaryDirectory() as d:
            acq = self._acq(Path(d), base_extra=[extra[2]] + [
                (1.4 + 30.0 * (k + 1), urc(k | (0x80 if k == 3 else 0), "RX_SCAN_FAILED"))
                for k in range(4)])
        self.assertEqual(acq["status"], rep.FAILED)
        self.assertTrue(acq["basis"].startswith("the base's final RX_SCAN_FAILED at +120.0 s"),
                        acq["basis"])


class AcquisitionOnTheRestartImage(unittest.TestCase):
    """2026-10-24 review: on the RS-12.21 image (scan option (b), VER capability
    bit 7) a final RX_SCAN_FAILED is a 30 s report and the scan goes on, so the
    absorbing-FAILED rule of every older image must not decide the leg."""

    VER_FIX = (1.0, "INFO image_rx_daemon: L072 VER warm-up ok (attempt 1) cap=0x000000FF")
    VER_OLD = (1.0, "INFO image_rx_daemon: L072 VER warm-up ok (attempt 1) cap=0x0000007F")

    @staticmethod
    def urc(pc, sub):
        return (pc, f"WARNING image_rx_daemon: FAULT_URC: code=0x0D sub=0x{sub:02X} "
                    f"name=RX_SCAN_FAILED basis=x tx_inflight=- uptime_ms=1 raw=0d")

    def _acq(self, image="auto", **kw) -> dict:
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), capture=False, mask=True, **kw)
            return rep.acquisition(_read(arch / "rx_daemon.log"),
                                   _read(arch / "tx_daemon.log"), image=image)

    def test_a_cold_final_then_a_lock_is_a_late_acquisition(self) -> None:
        # memo section 6 T3: the base silent for 125 s sends a cold final at
        # +120 s (sub 0xA3) and then LOCKs on the stream (here from +138 s on,
        # 343 of the tractor's 606 fragments): ACQUIRED, late
        late = dict(heard=range(264, 607), base_extra=[self.urc(121.4, 0xA3)])
        acq = self._acq(**dict(late, base_extra=[self.VER_FIX] + late["base_extra"]))
        self.assertIs(acq["restarts"], True)
        self.assertIn("logs the L072's VER capability bitmap 0x000000FF, bit 7 "
                      "(HOST_CAP_RADIO_PARK, the RS-12.21 image) set", acq["image"])
        self.assertEqual((acq["status"], acq["failed"]), (rep.ACQUIRED, False))
        self.assertTrue(acq["basis"].startswith(
            "the base decoded 343 of the tractor's 606 fragments on air (56.6 %)"),
            acq["basis"])
        self.assertIn("; a late acquisition: the base's final RX_SCAN_FAILED at +120.0 s "
                      "(rx_daemon.log line ", acq["basis"])
        self.assertIn("is cold (warm=0): the base's L072 had not locked since it booted, "
                      "but on this image a final is a 30 s report, not a stop (RS-12.21 "
                      "scan option (b): the scan restarts at once and reports again every "
                      "30 s until a LOCK; ", acq["basis"])
        self.assertTrue(acq["basis"].endswith(
            "; the last final is cold (warm=0: no lock since the L072 booted), so the lock "
            "shown came after +120.0 s"), acq["basis"])
        lines = rep.format_acquisition(acq)
        self.assertTrue(lines[1].startswith("  confirmed acquisition: ACQUIRED: "), lines[1])
        self.assertIn("its scan reported FAILED there and scanned on (this image restarts "
                      "it at once and reports every 30 s until a LOCK, RS-12.21 (b))",
                      lines[-1])
        self.assertNotIn("!! ACQUISITION FAILED", "\n".join(lines))
        # the same log read by the absorbing rule: FAILED, as before -- with the
        # image not identified (no VER line) the report says so and how to change it
        acq = self._acq(**late)
        self.assertIsNone(acq["restarts"])
        self.assertEqual(acq["status"], rep.FAILED)
        lines = rep.format_acquisition(acq)
        self.assertIn("The base's image was not identified (rx_daemon.log logs no VER "
                      "capability bitmap", lines[1])
        self.assertIn("run again with --scan-image restarts.", lines[1])
        self.assertIn("its scan entered FAILED there", lines[-1])
        # --scan-image restarts on that log, and an older image's bitmap (bit 7 clear)
        self.assertEqual(self._acq(image="restarts", **late)["status"], rep.ACQUIRED)
        acq = self._acq(**dict(late, base_extra=[self.VER_OLD] + late["base_extra"]))
        self.assertIs(acq["restarts"], False)
        self.assertEqual(acq["status"], rep.FAILED)
        self.assertNotIn("not identified", "\n".join(rep.format_acquisition(acq)))
        self.assertEqual(self._acq(image="absorbing", **dict(
            late, base_extra=[self.VER_FIX] + late["base_extra"]))["status"], rep.FAILED)
        with self.assertRaises(ValueError):
            self._acq(image="b", **late)

    def test_finals_to_the_end_of_the_log_are_a_failed_acquisition(self) -> None:
        # a final every 30 s up to 16.6 s before the log ends, and no lock shown
        finals = [self.urc(121.4 + 30.0 * k, 0xA3) for k in range(7)]
        acq = self._acq(heard=(40, 300, 560), base_extra=[self.VER_FIX] + finals)
        self.assertEqual(acq["status"], rep.FAILED)
        self.assertIn("; 7 final reports in a row, the last at +300.0 s (rx_daemon.log line ",
                      acq["basis"])
        self.assertTrue(acq["basis"].endswith(
            "; nothing shows a lock after the last final (the log ends 16.6 s after the last "
            "final, within the 35 s a next report may take)"), acq["basis"])
        lines = rep.format_acquisition(acq)
        self.assertIn(". This image restarts the scan after each final RX_SCAN_FAILED "
                      "(RS-12.21 scan option (b)), so the base was not stranded on one "
                      "channel, but it never locked onto the tractor's hop grid in the "
                      "window: this leg's loss and command figures measure an unacquired "
                      "base, not the link. The tractor logged 606 frame(s) on air.", lines[1])
        self.assertNotIn("absorbing", lines[1])
        # warm finals from an early one on, with a lock shown that nothing places
        # after the last: not determined
        warm = [self.urc(122.0 + 30.0 * k, 0xC3) for k in range(7)]
        acq = self._acq(heard=range(139, 607), base_extra=[self.VER_FIX] + warm)
        self.assertEqual(acq["status"], rep.UNDETERMINED)
        self.assertIn("came less than 122 s after the activation", acq["basis"])
        self.assertIn("(a lock resets the attempt count but on this image anchors the loss "
                      "timer at the locking frame (RS-12.21 L), so a locked scan drops back",
                      acq["basis"])
        self.assertIn("; nothing places the lock shown after the last final (the log ends "
                      "16.0 s after the last final, within the 35 s a next report may take), "
                      "and its evidence spans the leg: the base decoded 468 of",
                      acq["basis"])

    def test_a_sign_of_a_lock_after_the_finals(self) -> None:
        # a non-final attempt after the final: the attempt count restarts only at a
        # LOCK, so the scan locked in between -- itself a lock shown
        acq = self._acq(heard=(40, 300, 560), base_extra=[
            self.VER_FIX, self.urc(121.4, 0xA3), self.urc(251.4, 0x60)])
        self.assertEqual(acq["status"], rep.ACQUIRED)
        self.assertTrue(acq["basis"].startswith(
            "the base's RX_SCAN_FAILED at +250.0 s (rx_daemon.log line "), acq["basis"])
        self.assertIn(", sub 0x60) is attempt 0, not final: the attempt count restarts only "
                      "at a LOCK; a late acquisition: ", acq["basis"])
        self.assertTrue(acq["basis"].endswith(
            "; the scan locked between +120.0 s and +250.0 s"), acq["basis"])
        # a warm early final, a lock shown, and no fault in the 196 s after it:
        # this image would have reported again within 30 s without a LOCK
        acq = self._acq(heard=range(264, 607), base_extra=[self.VER_FIX,
                                                            self.urc(122.0, 0xC3)])
        self.assertEqual(acq["status"], rep.ACQUIRED)
        self.assertTrue(acq["basis"].endswith(
            "; no RX_SCAN_FAILED followed the last final in the 196.0 s the log runs past "
            "it, so the lock shown came within 30 s of +120.6 s"), acq["basis"])
        # two finals and no VER line: the second shows the image restarts its scan.
        # The log runs 166.6 s past it with no report and no lock shown: a LOCK is
        # likely (or the URCs were lost), so not determined, never FAILED
        acq = self._acq(heard=(40, 300, 560), base_extra=[self.urc(121.4, 0xA3),
                                                         self.urc(151.4, 0xA3)])
        self.assertIs(acq["restarts"], True)
        self.assertIn("after its final one at line ", acq["image"])
        self.assertIn("which only an image that restarts its scan does", acq["image"])
        self.assertEqual(acq["status"], rep.UNDETERMINED)
        self.assertIn("; no RX_SCAN_FAILED followed the last final in the 166.6 s the log "
                      "runs past it, which on this image means a LOCK within 30 s of it "
                      "unless its FAULT URCs were lost, but nothing else shows one",
                      acq["basis"])
        # the same two at the log's end: FAILED
        acq = self._acq(heard=(40, 300, 560), base_extra=[self.urc(271.4, 0xA3),
                                                         self.urc(301.4, 0xA3)])
        self.assertEqual((acq["restarts"], acq["status"]), (True, rep.FAILED))
        self.assertIn("This image restarts the scan", rep.format_acquisition(acq)[1])
        # two legacy lines of one URC (no FAULT_URC line, so both count) are not a
        # second report: the image stays unidentified, and on the restart image
        # they are one report
        twin = [(121.4, "WARNING image_rx_daemon: FAULT during pipelined TX: code=0x0D "
                        "sub=0xA3 raw=0da3"),
                (121.41, "WARNING image_rx_daemon: FAULT: code=0x0D sub=0xA3 raw=0da3")]
        acq = self._acq(heard=(40, 300, 560), base_extra=twin)
        self.assertEqual((len(acq["faults"]), acq["restarts"]), (2, None))
        acq = self._acq(image="restarts", heard=(40, 300, 560),
                        base_extra=[(271.4, twin[0][1]), (271.41, twin[1][1])])
        self.assertEqual(acq["status"], rep.FAILED)
        self.assertNotIn("final reports in a row", acq["basis"])

    def test_silence_counts_over_the_whole_window(self) -> None:
        # leg 6's first frame at +138.1 s is past the 120 s budget: FAILED on an
        # absorbing image, but on this image the scan does not strand
        acq = self._acq(heard=LEG6, base_extra=[self.VER_FIX])
        self.assertEqual(acq["status"], rep.UNDETERMINED)
        self.assertTrue(acq["basis"].startswith("a frame received is not a lock"),
                        acq["basis"])
        self.assertEqual(self._acq(heard=LEG6)["status"], rep.FAILED)
        # nothing in the whole window: FAILED
        acq = self._acq(heard=(), base_extra=[self.VER_FIX])
        self.assertEqual(acq["status"], rep.FAILED)
        self.assertEqual(acq["basis"], "the base's stats counter still read rx_frames=0 at "
                                       "+315.6 s (rx_daemon.log line 13), its last stats line "
                                       "in the window (always on, it counts every image "
                                       "fragment, logged or not): no image fragment reached "
                                       "the base in the 316.6 s window, and its scan locks "
                                       "only on a frame it decodes")

    def test_the_report_takes_scan_image(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), capture=False, mask=True, heard=range(264, 607),
                                     base_extra=[self.urc(121.4, 0xA3)])
            auto = _run([arch])
            fix = _run([arch, "--scan-image", "restarts"])
        self.assertIn("!! ACQUISITION FAILED (see the acquisition line below)", auto)
        self.assertIn("The base's image was not identified", auto)
        self.assertNotIn("ACQUISITION FAILED", fix)
        self.assertIn("  confirmed acquisition: ACQUIRED: the base decoded 343 of", fix)
        self.assertIn("; a late acquisition: ", fix)
        self.assertIn("--scan-image restarts given", fix)


class LegReportEndToEnd(unittest.TestCase):
    """2026-10-24: the archive report on a leg-6 shaped leg."""

    def test_leg_6_shape(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            arch, cap = _two_board_leg(Path(d))
            out = _run([arch, "--capture", cap])
        lines = out.splitlines()
        i = next(n for n, ln in enumerate(lines) if ln.startswith("capture seq gaps"))
        self.assertLess(i, 8)                         # leg_post.sh's 8-line excerpt
        self.assertIn("lost 226/231 = 97.8%", lines[i])
        self.assertIn("first frame heard at seq 8 is the tractor's frame #264 on air",
                      lines[i + 1])
        self.assertIn("the capture's first frame is its #264, its last #494 (placed by the "
                      "params.txt clock offsets, within 0.03 s): 601/606 = 99.2%", lines[i + 2])
        self.assertTrue(lines[i + 3].startswith("acquisition: first image reception: the base's "
                                                "first decoded frame came 138.1 s after"),
                        lines[i + 3])
        self.assertTrue(lines[i + 4].startswith("  !! ACQUISITION FAILED"), lines[i + 4])
        self.assertLess(i + 4, 8)
        self.assertNotIn("jumps", out)
        self.assertIn("shared-window loss 601/606 = 99.2%", out)
        self.assertTrue(lines[0].endswith("log_window=after_stop  !! ACQUISITION FAILED (see "
                                          "the acquisition line below)"), lines[0])

    def test_the_flag_stays_on_the_console_with_stale_counter_lines(self) -> None:
        # review 2026-10-24: leg_post.sh shows the report's first 8 lines; with the
        # stale-counter notes (legs 1-5 of 10-24 print one or two) the acquisition
        # lines fall past them, so the first line carries the flag too
        with tempfile.TemporaryDirectory() as d:
            arch, cap = _two_board_leg(Path(d))
            for name, old, new in (("tx_daemon.log", "frags_ok=606", "frags_ok=600"),
                                   ("rx_daemon.log", "rx_frames=5", "rx_frames=1")):
                p = arch / name
                p.write_text(_read(p).replace(old, new), encoding="utf-8")
            lines = _run([arch, "--capture", cap]).splitlines()
        self.assertTrue(lines[1].startswith("  (tx frags_ok counter 600 is stale"), lines[1])
        self.assertTrue(lines[2].startswith("  (rx rx_frames counter 1 is stale"), lines[2])
        i = next(n for n, ln in enumerate(lines) if ln.startswith("  !! ACQUISITION FAILED"))
        self.assertGreaterEqual(i, 8)                 # past the console excerpt
        self.assertIn("!! ACQUISITION FAILED (see the acquisition line below)", lines[0])
        # only a failed acquisition: not one that is not determined (a log shorter than
        # the budget) or n/a (off FHSS)
        for kw, acq in ((dict(heard=(), sent=100), "  confirmed acquisition: NOT DETERMINED: "),
                        (dict(profile=2), "acquisition: n/a (")):
            with tempfile.TemporaryDirectory() as d:
                arch, _ = _two_board_leg(Path(d), capture=False, **kw)
                out = _run([arch])
            self.assertIn(acq, out)
            self.assertTrue(out.splitlines()[0].endswith("log_window=after_stop  "), out)

    def test_a_missing_bracket_still_prints_the_loss_lines_first(self) -> None:
        # review 2026-10-24: the acquisition reads the brackets before the first line
        # prints; a missing one must not cost the loss lines (leg_post.sh keeps the
        # report's first 8 on the console). The radio-deltas block fails on it as
        # before, after them
        with tempfile.TemporaryDirectory() as d:
            arch, cap = _two_board_leg(Path(d))
            buf = io.StringIO()
            with redirect_stdout(buf), self.assertRaises(OSError):
                rep.main([str(arch), "--pre", str(Path(d) / "missing_pre.txt"),
                          "--post", str(Path(d) / "missing_post.txt"), "--capture", str(cap)])
        out = buf.getvalue()
        self.assertIn("!! ACQUISITION FAILED (see the acquisition line below)",
                      out.splitlines()[0])
        for want in ("\nloss 601/606 = 99.2%", "\nshared-window loss 601/606 = 99.2%",
                     "\ncapture seq gaps (leg2x_base.jsonl): lost 226/231 = 97.8%",
                     "\nacquisition: first image reception: ", "\n  !! ACQUISITION FAILED: "):
            self.assertIn(want, out)
        self.assertNotIn("radio deltas", out)

    def test_a_locked_leg_keeps_its_old_head_line(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            arch, cap = _two_board_leg(Path(d), heard=range(139, 607), mask=True)
            out = _run([arch, "--capture", cap])
        self.assertIn("  first frame heard at seq 139: the 138 frame(s) numbered before it "
                      "are not in the figure above (the camera numbers from seq 1 at "
                      "start-up — an acquisition delay or a late capture, RS-13.1 A7); "
                      "counting from seq 1: 138/606 = 22.8%", out)
        self.assertIn("the capture's first frame is its #139, its last #606", out)
        self.assertNotIn("!!", out)
        # without a capture: the acquisition lines, no head
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), heard=range(139, 607), capture=False, mask=True)
            out = _run([arch])
        self.assertTrue(out.splitlines()[-2].startswith(
            "acquisition: first image reception: the base's first decoded frame came 75.6 s "
            "after"), out)
        self.assertTrue(out.splitlines()[-1].startswith(
            "  confirmed acquisition: ACQUIRED: the base decoded 468 of the tractor's 606 "
            "fragments on air (77.2 %)"), out)


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

    def test_2026_10_24_round(self) -> None:
        # The FHSS round: every line of each committed legs/leg<X>_report.txt
        # still prints, plus the new ones; leg 6 (2b_f3) only loses its three
        # wrong lines (RESULTS.md, "Tool errors on leg 6").
        legs = _EV / "bench_session_2026-10-24" / "legs"
        wrong = ("capture seq gaps (leg2b_f3_base.jsonl): lost 91/96 = 94.8%",
                 "  first frame heard at seq 8: the 7 frame(s) numbered before it",
                 "  !! jumps 1 (jumps = a counter restart or a reorder")
        seen = 0
        for tag, stamp, first in (("2c_f1", "095315", 75.0), ("2b_f1", "100249", 34.5),
                                  ("2c_f2", "101341", 51.5), ("2b_f2", "102247", 34.5),
                                  ("2c_f3", "103156", 15.3), ("2b_f3", "104056", 138.1)):
            arch = _EV / f"radio_monitor_20261024_{stamp}_d28bc2ca"
            if not (arch / "tx_daemon.log").is_file() or not (legs / f"leg{tag}_report.txt").is_file():
                continue
            seen += 1
            with self.subTest(leg=tag):
                out = _run([arch, "--pre", legs / f"leg{tag}_pre_base.txt",
                            "--post", legs / f"leg{tag}_post_base.txt",
                            "--capture", legs / f"leg{tag}_base.jsonl"])
                new = out.splitlines()
                old = (legs / f"leg{tag}_report.txt").read_text(encoding="utf-8").splitlines()
                if tag == "2b_f3":
                    # its first line (params) gains the flag
                    self.assertEqual(new[0], old[0] + "!! ACQUISITION FAILED (see the "
                                                      "acquisition line below)")
                    old = old[1:]
                for ln in old:
                    if tag == "2b_f3" and ln.startswith(wrong):
                        self.assertNotIn(ln, new)
                    else:
                        self.assertIn(ln, new)
                acq = next(ln for ln in new if ln.startswith("acquisition:"))
                self.assertIn(f"came {first:.1f} s after its profile-1 activation", acq)
                if tag != "2b_f3":
                    self.assertIn(", within the FHSS cold-start scan budget", acq)
                    self.assertNotIn("!!", out)
                    # review 2026-10-24: a lock shown, by the base bracket's
                    # end-of-leg demotion and by the share decoded
                    self.assertIn("  confirmed acquisition: ACQUIRED: the base's bracket counts "
                                  "1 LOCKED -> SCANNING demotion(s) (d(clk_demotion_reset) 1, "
                                  "d(clk_demotion_kept) 0), which only a locked scan makes; the "
                                  "base decoded ", out)
                    continue
                # leg 6: FAILED on the evidence it has (no FAULT_URC line: its daemon
                # predates them): the always-on counter and per-fragment logging
                self.assertIn("  !! ACQUISITION FAILED: the base's stats counter still read "
                              "rx_frames=0 at +130.2 s (rx_daemon.log line 65; always on, it "
                              "counts every image fragment, logged or not); per-fragment "
                              "logging was on (rx_daemon.log holds frag_arrival lines) and the "
                              "first came at +138.1 s: no image fragment reached the base within "
                              "the 120 s budget", out)
                self.assertIn("capture seq gaps (leg2b_f3_base.jsonl): lost 226/231 = 97.8%", out)
                self.assertIn("  first frame heard at seq 8 is the tractor's frame #264 on air (1 "
                              "u8 seq wrap before it, by its TX log): the 263 frame(s)", out)
                self.assertIn("the capture's first frame is its #264, its last #494 (placed by "
                              "the params.txt clock offsets, within 0.03 s): 601/606 = 99.2% of "
                              "them never reached the capture (263 before its first, 112 after "
                              "its last)", out)
                self.assertIn(", past the FHSS cold-start scan budget", acq)
                self.assertIn("  !! ACQUISITION FAILED: ", out)
                self.assertNotIn("jumps", out)
        if not seen:
            self.skipTest("the 2026-10-24 archives are not in this tree")

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
