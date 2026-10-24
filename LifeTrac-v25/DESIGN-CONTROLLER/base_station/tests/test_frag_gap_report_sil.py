"""bench_tools/frag_gap_report.py -- SIL tests for the head and tail lines (2026-10-24).

The gaps run from the base's first fragment to its last, so on the FHSS round's
leg 6 (the base never locked) they left out the 138.1 s before its first fragment
and about 56 s after its last. Pins: the ``head:`` line (the base's profile
activation, rs12_leg_report.py's anchor, to the first fragment; with the
params.txt clock offsets also the tractor's first txdone) and the ``tail:``
line (the last fragment to the tractor's last txdone with the offsets, and to
the base daemon's stop, or its log's last line from an older harness); a leg
with one fragment or none still prints them and still exits 1, without the
command-plane tally that bracket_deltas.py cross-checks, so a base that heard
nothing stays UNVERIFIED there (end to end), while an archive without
per-fragment events prints its one line as before, and so does a copy that
cannot import rs12_leg_report (the one stage_boards.sh puts on the boards);
on the committed 10-24 legs every old line still prints.
"""
from __future__ import annotations

import io
import shutil
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

_BS = Path(__file__).resolve().parents[1]
_BT = _BS.parent / "firmware" / "x8_lora_bootloader_helper" / "bench_tools"
for _p in (_BS, _BT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import frag_gap_report as fg  # noqa: E402
from tests import test_bracket_deltas_sil as bdt  # noqa: E402
from tests.test_rs12_leg_report_sil import _two_board_leg  # noqa: E402

_EV = _BS.parent / "bench-evidence"


def _run(archive) -> tuple[int, str]:
    buf = io.StringIO()
    with mock.patch.object(sys, "argv", ["frag_gap_report.py", str(archive)]), \
            redirect_stdout(buf):
        rc = fg.main()
    return rc, buf.getvalue()


class HeadAndTail(unittest.TestCase):
    def test_leg_6_shape(self) -> None:
        # the tractor sends 606 frames from PC 8 s at 2 fps; the base activates at
        # 1.4 s, hears #264, #293, #346, #358 and #494, and stops at 318 s
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), capture=False)
            rc, out = _run(arch)
        self.assertEqual(rc, 0, out)
        lines = out.splitlines()
        self.assertTrue(lines[0].startswith("n_frag=5 span=115s max_gap=68.0s gaps>3s=4 "),
                        lines[0])
        self.assertEqual(lines[2], "head: the first fragment came 138.1 s after the base's "
                                   "profile activation (15:35:36.531 base clock, the first line "
                                   "after REG_PROFILE OK); 131.5 s after the tractor's first "
                                   "txdone (params.txt clock offsets)")
        self.assertEqual(lines[3], "tail: the last fragment came 56.0 s before the tractor's "
                                   "last txdone (params.txt clock offsets); 63.5 s before the "
                                   "base daemon's stop (its 'signal received' line)")
        self.assertEqual(lines[4], "base sends on air=0 | tractor decoded cmds=0")

    def test_without_offsets_and_without_a_stop_line(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), capture=False, offsets=False)
            rc, out = _run(arch)
            self.assertIn("head: the first fragment came 138.1 s after the base's profile "
                          "activation (15:35:36.531 base clock, the first line after "
                          "REG_PROFILE OK)\n", out)
            self.assertIn("tail: the last fragment came 63.5 s before the base daemon's stop "
                          "(its 'signal received' line)\n", out)
            self.assertNotIn("txdone", out)
            # an older harness read the log while the daemon ran: no stop line
            rx = arch / "rx_daemon.log"
            rx.write_text("\n".join(ln for ln in rx.read_text(encoding="utf-8").splitlines()
                                    if "signal received" not in ln) + "\n", encoding="utf-8")
            rc, out = _run(arch)
        self.assertIn("tail: the last fragment came 62.5 s before the base log's last line (no "
                      "stop line: an older harness read the log while the daemon ran)", out)

    def test_a_base_that_caught_one_fragment_or_none(self) -> None:
        for heard, head in (((264,), "head: the first fragment came 138.1 s after"),
                            ((), "head: no fragment in the 316.6 s the base log covers after "
                                 "its profile activation (15:35:36.531 base clock)")):
            with tempfile.TemporaryDirectory() as d:
                arch, _ = _two_board_leg(Path(d), capture=False, heard=heard)
                rc, out = _run(arch)
            lines = out.splitlines()
            self.assertEqual(rc, 1, out)
            self.assertEqual(lines[0], f"frag_arrival lines: {len(heard)} (need "
                                       f"-LogFragArrivals 1)")
            self.assertTrue(lines[1].startswith(head), lines[1])
            # no command-plane tally, as before (see the bracket_deltas test below)
            self.assertNotIn("base sends on air", out)
            self.assertEqual(len(lines), 3 if heard else 2, lines)

    def test_a_base_that_heard_nothing_stays_unverified_in_bracket_deltas(self) -> None:
        # review 2026-10-24: bracket_deltas.py reads the gaps file's command-plane tally
        # as its log cross-check. A tally of 0/0 from a base that heard nothing scored
        # its FHSS leg as a clean image-only pass (N/A delivery, exit 0); without it the
        # leg stays UNVERIFIED, STOP RULE UNDETERMINED, exit 1, as before 2026-10-24
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), capture=False, heard=())
            rc, gaps = _run(arch)
        self.assertEqual(rc, 1, gaps)
        # the base sent nothing (23 -> 23) and the tractor heard nothing (5 -> 5) while
        # it streamed 50, radio_state 4 on both boards at post: the never-locked leg
        files = dict(bdt.GOOD, gaps=gaps,
                     post_base=bdt.bracket(radio_tx_ok=23, host_parse_ok=200),
                     post_tractor=bdt.post_tr(radio_rx_ok=5, radio_state=4))
        rc, out = bdt.run(**files)
        self.assertEqual(rc, 1, out)
        self.assertIn('cross-check: brackets 0/0 vs no log cross-check (no "base sends on air" '
                      'line in legT_gaps.txt) -> DISAGREE or unavailable', out)
        self.assertIn("  [UNVERIFIED] command delivery: no command counted (brackets 0/0), not "
                      "confirmed by the logs", out)
        self.assertIn("!! BASE NEVER LOCKED: ", out)
        self.assertEqual(out.splitlines()[-1], "STOP RULE: UNDETERMINED -- deltas not verified "
                                               "against the logs (!! BASE NEVER LOCKED above: "
                                               "this leg did not test FHSS reverse delivery)")
        self.assertNotIn("[PASS]", out)
        self.assertNotIn("[N/A]", out)

    def test_an_archive_without_per_fragment_events_is_unchanged(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), capture=False, heard=())
            tx = arch / "tx_daemon.log"
            tx.write_text("\n".join(ln for ln in tx.read_text(encoding="utf-8").splitlines()
                                    if "txdone_arrival" not in ln) + "\n", encoding="utf-8")
            rc, out = _run(arch)
        self.assertEqual((rc, out), (1, "frag_arrival lines: 0 (need -LogFragArrivals 1)\n"))

    def test_a_copy_without_rs12_leg_report_prints_as_before(self) -> None:
        # stage_boards.sh also stages this file to the boards, where there is no
        # DC/tools: there it runs isolated from the repo and prints the old lines
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), capture=False)
            stage = Path(d) / "work"
            stage.mkdir()
            shutil.copy(_BT / "frag_gap_report.py", stage)
            p = subprocess.run([sys.executable, "-I", str(stage / "frag_gap_report.py"),
                                str(arch)], capture_output=True, text=True, encoding="utf-8")
            rc, full = _run(arch)
        self.assertEqual(p.returncode, 0, p.stderr)
        self.assertEqual(p.stdout.splitlines(),
                         [ln for ln in full.splitlines()
                          if not ln.startswith(("head: ", "tail: "))])

    def test_no_activation_line_and_the_rounding(self) -> None:
        with tempfile.TemporaryDirectory() as d:
            arch, _ = _two_board_leg(Path(d), capture=False)
            rx = arch / "rx_daemon.log"
            rx.write_text(rx.read_text(encoding="utf-8").replace("REG_PROFILE=1) OK", "x"),
                          encoding="utf-8")
            rc, out = _run(arch)
        self.assertIn("head: n/a (no REG_PROFILE activation line in rx_daemon.log)\n", out)
        self.assertIn("tail: the last fragment came 56.0 s before", out)
        # a last fragment logged a few ms after the tractor's last txdone
        self.assertEqual((fg._s(-0.019), fg._s(0.04), fg._s(-0.3)), ("0.0", "0.0", "-0.3"))


@unittest.skipUnless((_EV / "radio_monitor_20261024_104056_d28bc2ca" / "rx_daemon.log").is_file(),
                     "the 2026-10-24 archives are not in this tree")
class Round20261024(unittest.TestCase):
    def test_every_old_line_prints_and_leg_6_gets_its_head_and_tail(self) -> None:
        legs = _EV / "bench_session_2026-10-24" / "legs"
        for tag, stamp in (("2c_f1", "095315"), ("2b_f1", "100249"), ("2c_f2", "101341"),
                           ("2b_f2", "102247"), ("2c_f3", "103156"), ("2b_f3", "104056")):
            rc, out = _run(_EV / f"radio_monitor_20261024_{stamp}_d28bc2ca")
            self.assertEqual(rc, 0, out)
            new = out.splitlines()
            old = (legs / f"leg{tag}_gaps.txt").read_text(encoding="utf-8").splitlines()
            self.assertEqual([ln for ln in new if not ln.startswith(("head: ", "tail: "))], old)
            self.assertEqual(len(new), len(old) + 2)
        self.assertIn("head: the first fragment came 138.1 s after the base's profile activation "
                      "(15:35:37.624 base clock, the first line after REG_PROFILE OK); 131.6 s "
                      "after the tractor's first txdone (params.txt clock offsets)", out)
        self.assertIn("tail: the last fragment came 55.9 s before the tractor's last txdone "
                      "(params.txt clock offsets); 62.7 s before the base daemon's stop", out)


if __name__ == "__main__":
    unittest.main()
