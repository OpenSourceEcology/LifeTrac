"""bench_tools/bracket_deltas.py -- SIL tests for the leg counter-bracket scoring.

The cases of the 2026-10-18 scorer's --selftest
(bench-evidence/RS_12_15_reverse_delivery_2026-10-18/scripts/deltas.py), moved here
so CI runs them (``bracket_deltas.py --selftest`` runs this file), plus the kit
changes. Pins:

* parsing: ``ABSENT`` and missing counters are reported, never read as 0; a
  failed probe, a missing bracket and a negative value refuse to score (exit 2);
* reset detection: a lower event counter or high-water mark, or fewer parsed
  host frames than TXs, means the delta is the post value; ``radio_state``
  falling is no reset;
* the log cross-check: disagreeing or missing counts make every criterion
  that would pass UNVERIFIED and the STOP rule UNDETERMINED, while a delta
  above its limit still fails;
* tx_first_anchor by the leg's delta: the second FHSS leg on a tractor that
  was not reset (post 2, delta 1) passes; delta 2 trips STOP, with or without
  a detected reset; delta 0 fails once verified;
* the base's tx_stream_streak_max as a since-boot high-water mark: below 8
  passes, a rise to 8 or more fails, 8 or more held from before the leg is
  UNVERIFIED;
* an image-only FHSS leg (the RS-13 FHSS walk, no base commands): delivery
  0/0 with the logs agreeing is N/A and the leg can exit 0; 0/0 the logs do
  not confirm is UNVERIFIED; the base sending nothing while the tractor
  counts commands still fails;
* the profile: reg_profile from leg<X>_params.txt or --params (file or
  archive dir), --profile when there are none, a contradiction or an unknown
  profile refused; a DTS (or fixed-915) leg reports deltas, reset evidence and
  the cross-check with no criterion and exit 0, and notes FHSS-only counters
  that moved;
* on the committed legs: 2026-10-18 V, W, W2 and X (FHSS) give the numbers
  of their legs/leg<X>_deltas.txt and pass; 2026-10-19 A2c, A2a, A2c2, A2a2
  (DTS) report with no false FAIL;
* 2026-10-24 (the FHSS round's leg 6): base d(clk_demotion_reset) 0 and
  d(fhss_dec_aligned) 0 while the tractor streamed, radio_state 4 on both
  boards at post (or not in a bracket), prints ``!! BASE NEVER LOCKED`` and
  the STOP RULE line points at it, with the criteria, the STOP verdict and
  the exit code unchanged; a base that demoted or decoded aligned, a quiet
  tractor, another radio_state or a DTS leg is not flagged; of the committed
  10-24 legs only leg 6 is, and legs 1-5 print their deltas line for line.
"""
from __future__ import annotations

import io
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest import mock

_DC = Path(__file__).resolve().parents[2]
_BT = _DC / "firmware" / "x8_lora_bootloader_helper" / "bench_tools"
if str(_BT) not in sys.path:
    sys.path.insert(0, str(_BT))

import bracket_deltas as bd  # noqa: E402

_RS1215 = _DC / "bench-evidence" / "RS_12_15_reverse_delivery_2026-10-18" / "legs"
_RS13 = _DC / "bench-evidence" / "RS_13_range_edge_2026-10-19" / "legs"


def bracket(**kv) -> str:
    vals = dict.fromkeys(bd.NON_DECREASING, 5)
    vals.update(radio_state=4, tx_first_anchor=0, clk_demotion_reset=0,
                fhss_dec_snapped=0, fhss_dec_rej_epoch_drift=0, tx_stream_streak_max=1)
    vals.update(kv)
    return "\n".join(f"{k}={x}" for k, x in vals.items() if x is not None)


def pre_tr(**kv) -> str:
    return bracket(**dict(dict(radio_tx_ok=900, radio_rx_ok=5, host_parse_ok=1000), **kv))


def post_tr(**kv) -> str:
    # one FHSS activation: the first TX anchors the slot clock once
    return bracket(**dict(dict(radio_tx_ok=950, radio_rx_ok=27, host_parse_ok=1100,
                               radio_state=1, tx_first_anchor=1), **kv))


# a clean FHSS leg: base sent 27 (23 -> 50), the tractor heard 22 (5 -> 27); radio_state 4 -> 1
GOOD = {
    "pre_base": bracket(radio_tx_ok=23, host_parse_ok=100),
    "post_base": bracket(radio_tx_ok=50, host_parse_ok=200),
    "pre_tractor": pre_tr(),
    "post_tractor": post_tr(),
    "gaps": "n_frag=10 max_gap=0.5s gaps>3s=0\nbase sends on air=27 | tractor decoded cmds=22\n",
    "params": "﻿git_sha=0\nduration_s=300\nreg_profile=1\nforce_frf_hz=0\n",
}


def run(argv=None, **change) -> tuple[int, str]:
    """Score leg T built from GOOD with ``change`` (a part set to None is not written)."""
    files = dict(GOOD, **change)
    with tempfile.TemporaryDirectory() as d:
        for part, text in files.items():
            if text is not None:
                fn = f"legT_{part}.txt"
                with open(os.path.join(d, fn), "w", encoding="utf-8") as f:
                    f.write(text)
        out = io.StringIO()
        with redirect_stdout(out):
            rc = bd.main([d, "T"] + list(argv or []))
    return rc, out.getvalue()


class BracketDeltasTests(unittest.TestCase):
    def check(self, want_rc, *want_text, argv=None, absent=(), **change) -> str:
        rc, out = run(argv, **change)
        self.assertEqual(rc, want_rc, out)
        for s in want_text:
            self.assertIn(s, out)
        for s in absent:
            self.assertNotIn(s, out)
        return out

    def test_parsing(self) -> None:
        # ABSENT and missing fields are reported, never read as 0
        vals, absent = bd.parse(bracket(clk_demotion_reset="ABSENT"))
        self.assertEqual(bd.missing(vals, absent), ["clk_demotion_reset"])
        vals, absent = bd.parse("STATS_FAIL timeout waiting for response")
        self.assertEqual(vals, {})
        self.assertEqual(set(bd.missing(vals, absent)), set(bd.REQUIRED))

    def test_clean_leg(self) -> None:
        self.check(0, "leg T: reg_profile=1 (FHSS, from legT_params.txt) -> the RS-12.15 bracket "
                      "criteria apply",
                   "base: L072 counters not reset", "tractor: L072 counters not reset",
                   "= 22/27 = 81.5 %", "-> AGREE",
                   "[PASS] tractor tx_first_anchor delta = 1 (post 1;",
                   "[PASS] base tx_stream_streak_max post = 1 (d=0;", "STOP RULE: not tripped",
                   absent=("NEVER LOCKED",))

    def test_reset_detection(self) -> None:
        for mark in ("tx_deaf_max_us", "tx_done_to_rearm_max_us", "tx_stream_streak_max"):
            self.check(0, f"RESET during the leg (evidence: {mark})",
                       pre_tractor=pre_tr(**{mark: 900}), post_tractor=post_tr(**{mark: 100}),
                       gaps="base sends on air=27 | tractor decoded cmds=27")
        self.check(0, f"RESET during the leg (evidence: {bd.INVARIANT})",
                   post_tractor=post_tr(host_parse_ok=1010),
                   gaps="base sends on air=27 | tractor decoded cmds=27")
        self.check(1, "RESET during the leg (evidence: host_parse_ok",
                   post_tractor=post_tr(host_parse_ok=90))

    def test_cross_check(self) -> None:
        self.check(1, "[UNVERIFIED] command delivery", "[UNVERIFIED] tractor clk_demotion_reset",
                   "[UNVERIFIED] tractor tx_first_anchor delta = 1",
                   "[UNVERIFIED] base tx_stream_streak_max post = 1", "STOP RULE: UNDETERMINED",
                   gaps="base sends on air=27 | tractor decoded cmds=20")
        self.check(1, "legT_gaps.txt missing", "[UNVERIFIED] command delivery", gaps=None)
        self.check(1, 'no "base sends on air" line', gaps="frag_arrival lines: 1\n")
        self.check(1, "[FAIL] tractor fhss_dec_snapped",
                   post_tractor=post_tr(fhss_dec_snapped=7), gaps=None)

    def test_tx_first_anchor_by_delta(self) -> None:
        # The second FHSS leg on a tractor whose leg-start reset did not land:
        # its counters carry over (pre 1) and this leg's activation adds one.
        self.check(0, "[PASS] tractor tx_first_anchor delta = 1 (post 2;", "STOP RULE: not tripped",
                   pre_tractor=pre_tr(tx_first_anchor=1), post_tractor=post_tr(tx_first_anchor=2))
        # A re-anchor inside the leg (an adopted echo, a phase restart) trips STOP.
        self.check(1, "[FAIL] tractor tx_first_anchor delta = 2 (post 3;", "STOP RULE: TRIPPED",
                   pre_tractor=pre_tr(tx_first_anchor=1), post_tractor=post_tr(tx_first_anchor=3))
        self.check(1, "[FAIL] tractor tx_first_anchor delta = 2", "STOP RULE: TRIPPED",
                   post_tractor=post_tr(tx_first_anchor=2), gaps=None)
        # After a detected reset the delta is the post value.
        self.check(1, "RESET during the leg", "[FAIL] tractor tx_first_anchor delta = 2 (post 2;",
                   "STOP RULE: TRIPPED",
                   pre_tractor=pre_tr(tx_first_anchor=1, tx_deaf_max_us=900),
                   post_tractor=post_tr(tx_first_anchor=2, tx_deaf_max_us=100),
                   gaps="base sends on air=27 | tractor decoded cmds=27")
        self.check(0, "RESET during the leg", "[PASS] tractor tx_first_anchor delta = 1 (post 1;",
                   pre_tractor=pre_tr(tx_first_anchor=1, tx_deaf_max_us=900),
                   post_tractor=post_tr(tx_first_anchor=1, tx_deaf_max_us=100),
                   gaps="base sends on air=27 | tractor decoded cmds=27")
        # No anchor at all: fails once verified; unverified, an undetected reset could hide it.
        self.check(1, "[FAIL] tractor tx_first_anchor delta = 0 (post 1;", "STOP RULE: not tripped",
                   pre_tractor=pre_tr(tx_first_anchor=1), post_tractor=post_tr(tx_first_anchor=1))
        self.check(1, "[UNVERIFIED] tractor tx_first_anchor delta = 0",
                   pre_tractor=pre_tr(tx_first_anchor=1), post_tractor=post_tr(tx_first_anchor=1),
                   gaps=None)

    def test_base_streak_is_a_since_boot_mark(self) -> None:
        self.check(1, "[FAIL] base tx_stream_streak_max post = 8 (d=7;",
                   post_base=bracket(radio_tx_ok=50, host_parse_ok=200, tx_stream_streak_max=8))
        self.check(0, "[PASS] base tx_stream_streak_max post = 7 (d=4;",
                   pre_base=bracket(radio_tx_ok=23, host_parse_ok=100, tx_stream_streak_max=3),
                   post_base=bracket(radio_tx_ok=50, host_parse_ok=200, tx_stream_streak_max=7))
        # An earlier leg left it at 9; this leg did not raise it: not this leg's FAIL.
        out = self.check(1, "[UNVERIFIED] base tx_stream_streak_max post = 9 (d=0;",
                         "it stood at 9 before the leg", "STOP RULE: not tripped",
                         absent=("[FAIL]",),
                         pre_base=bracket(radio_tx_ok=23, host_parse_ok=100, tx_stream_streak_max=9),
                         post_base=bracket(radio_tx_ok=50, host_parse_ok=200,
                                           tx_stream_streak_max=9))
        self.assertEqual(out.count("[PASS]"), 5)
        self.check(1, "[FAIL] base tx_stream_streak_max post = 10 (d=1;",
                   pre_base=bracket(radio_tx_ok=23, host_parse_ok=100, tx_stream_streak_max=9),
                   post_base=bracket(radio_tx_ok=50, host_parse_ok=200, tx_stream_streak_max=10))

    def test_other_criteria(self) -> None:
        self.check(1, "[FAIL] tractor fhss_dec_rej_epoch_drift",
                   post_tractor=post_tr(fhss_dec_rej_epoch_drift=1))
        self.check(1, "= 16/27 = 59.3 %", "[FAIL] command delivery",
                   post_tractor=post_tr(radio_rx_ok=21),
                   gaps="base sends on air=27 | tractor decoded cmds=16")
        self.check(1, "[FAIL] tractor clk_demotion_reset", "STOP RULE: TRIPPED",
                   post_tractor=post_tr(clk_demotion_reset=1))
        self.check(1, "= 22/0 = n/a", "[FAIL] command delivery",
                   post_base=bracket(radio_tx_ok=23, host_parse_ok=200))
        self.check(0, f"  {'radio_crc_err':28s} {'5->5 (d=0)':>18s} {'n/a':>18s}",
                   pre_tractor=pre_tr(radio_crc_err=None))

    def test_image_only_fhss_leg(self) -> None:
        # The RS-13 FHSS walk (-ReactiveFire 0, no -InjectCmd): the base sends nothing,
        # frag_gap_report.py still prints its 0/0 line. Delivery is N/A, not a FAIL.
        image_only = dict(post_base=bracket(radio_tx_ok=23, host_parse_ok=200),
                          post_tractor=post_tr(radio_rx_ok=5))
        out = self.check(0, "brackets 0/0 vs logs: base sends on air=0, tractor decoded cmds=0 "
                            "-> AGREE",
                         "[N/A] command delivery: no command sent", "STOP RULE: not tripped",
                         absent=("[FAIL]", "[UNVERIFIED]", "= 0/0 = n/a"),
                         gaps="base sends on air=0 | tractor decoded cmds=0", **image_only)
        self.assertEqual(out.count("[PASS]"), 5)
        # the other criteria still apply on an image-only leg
        self.check(1, "[N/A] command delivery", "[FAIL] tractor clk_demotion_reset",
                   "STOP RULE: TRIPPED", gaps="base sends on air=0 | tractor decoded cmds=0",
                   post_base=image_only["post_base"],
                   post_tractor=post_tr(radio_rx_ok=5, clk_demotion_reset=1))
        # 0/0 the logs do not confirm (missing, or they counted sends): not verified
        for gaps in (None, "base sends on air=27 | tractor decoded cmds=22"):
            self.check(1, "[UNVERIFIED] command delivery: no command counted (brackets 0/0)",
                       "STOP RULE: UNDETERMINED", absent=("[N/A]", "[FAIL]"), gaps=gaps,
                       **image_only)

    def test_base_never_locked(self) -> None:
        # 2026-10-24 leg 6: the base's demotion and aligned counters stood still while
        # the tractor streamed, radio_state 4 (RX_CONT) on both boards at post. GOOD's
        # base already has d(aligned) 0 and d(demotion) 0; its tractor reads radio_state 1.
        unlocked = dict(post_tractor=post_tr(radio_state=4))
        out = self.check(0, "!! BASE NEVER LOCKED: base d(clk_demotion_reset)=0 and "
                            "d(fhss_dec_aligned)=0 while the tractor sent 50 (d(radio_tx_ok)); "
                            "post radio_state 4 (RX_CONT) on both boards. The fingerprint of a "
                            "base FHSS scan that ran out into FAILED",
                         "command delivery counts only the probes the base fired after a frame "
                         "it happened to catch (22/27). Re-fly the leg, or void it only under a "
                         "rule written before leg 1",
                         **unlocked)
        lines = out.splitlines()
        # the criteria and the exit code as before; the flag sits above the STOP RULE line,
        # which points at it (leg_post.sh shows that line on the console)
        self.assertEqual(out.count("[PASS]"), 6)
        self.assertTrue(lines[-2].startswith("!! BASE NEVER LOCKED"), lines[-2])
        self.assertEqual(lines[-1], "STOP RULE: not tripped (!! BASE NEVER LOCKED above: this "
                                    "leg did not test FHSS reverse delivery)")
        # the STOP rule keeps its verdict, tripped or undetermined
        self.check(1, "STOP RULE: TRIPPED -- stop the round (!! BASE NEVER LOCKED above",
                   post_tractor=post_tr(radio_state=4, clk_demotion_reset=1))
        self.check(1, "STOP RULE: UNDETERMINED -- deltas not verified against the logs (!! "
                      "BASE NEVER LOCKED above", gaps=None, **unlocked)
        # radio_state missing from a post bracket does not rule it out; the line says so
        self.check(0, "; post radio_state not in both brackets. ",
                   post_tractor=post_tr(radio_state=None),
                   post_base=bracket(radio_tx_ok=50, host_parse_ok=200, radio_state=None))
        self.check(0, "; post radio_state not in the tractor bracket. ",
                   post_tractor=post_tr(radio_state=None))

    def test_a_base_that_locked_is_not_flagged(self) -> None:
        for name, change in (
                ("a demotion", dict(post_base=bracket(radio_tx_ok=50, host_parse_ok=200,
                                                      clk_demotion_reset=1))),
                ("aligned decodes", dict(post_base=bracket(radio_tx_ok=50, host_parse_ok=200,
                                                           fhss_dec_aligned=30))),
                ("a quiet tractor", dict(post_tractor=post_tr(radio_tx_ok=910,
                                                              radio_state=4))),
                ("the tractor's radio_state", dict(post_tractor=post_tr(radio_state=2))),
                ("the base's radio_state", dict(post_base=bracket(radio_tx_ok=50,
                                                                  host_parse_ok=200,
                                                                  radio_state=2))),
                # one bracket without radio_state, the other not RX_CONT: ruled out
                ("a mixed post state", dict(post_base=bracket(radio_tx_ok=50, host_parse_ok=200,
                                                              radio_state=None),
                                            post_tractor=post_tr(radio_state=2)))):
            change.setdefault("post_tractor", post_tr(radio_state=4))
            rc, out = run(**change)
            self.assertNotIn("NEVER LOCKED", out, name)
            self.assertIn("STOP RULE: not tripped\n", out, name)
        # a counter the fingerprint reads is missing: no flag
        rc, out = run(post_tractor=post_tr(radio_state=4),
                      post_base=bracket(radio_tx_ok=50, host_parse_ok=200,
                                        fhss_dec_aligned=None))
        self.assertNotIn("NEVER LOCKED", out)
        # off FHSS there is no hop grid to lock: never flagged
        self.check(0, absent=("NEVER LOCKED",), params="reg_profile=2\n",
                   post_tractor=post_tr(radio_state=4, tx_first_anchor=0))

    def test_incomplete_input(self) -> None:
        self.check(2, "missing/ABSENT: fhss_dec_snapped", post_tractor=post_tr(fhss_dec_snapped=-1))
        self.check(2, "INCOMPLETE BRACKETS", "missing/ABSENT: clk_demotion_reset",
                   post_tractor=post_tr(clk_demotion_reset="ABSENT"))
        self.check(2, "base pre: empty bracket", pre_base="STATS_FAIL timeout\n")
        self.check(2, "tractor pre: file missing", pre_tractor=None)

    def test_dts_reports_without_criteria(self) -> None:
        dts = "﻿reg_profile=2\nforce_frf_hz=927500000\n"
        out = self.check(0, "reg_profile=2 (DTS, from legT_params.txt) -> deltas, reset evidence "
                            "and cross-check only",
                         "bracket criteria: none on a DTS leg (reg_profile=2)",
                         "DTS command delivery: rs13_probe_score.py", "-> DISAGREE or unavailable",
                         "STOP RULE: not applied (FHSS only)",
                         absent=("[PASS]", "[FAIL]", "[UNVERIFIED]", "note: FHSS-only"),
                         params=dts, gaps="base sends on air=26 | tractor decoded cmds=21",
                         post_tractor=post_tr(tx_first_anchor=0, radio_rx_ok=21))
        self.assertIn("tractor: L072 counters not reset", out)
        # a STOP-shaped tractor and a base streak of 9 are no criteria on DTS
        self.check(0, "note: FHSS-only counter(s) moved on a non-FHSS leg (tractor "
                      "clk_demotion_reset d=1, tractor tx_first_anchor d=2)",
                   absent=("[FAIL]", "TRIPPED"), params=dts,
                   post_tractor=post_tr(clk_demotion_reset=1, tx_first_anchor=2),
                   post_base=bracket(radio_tx_ok=50, host_parse_ok=200, tx_stream_streak_max=9))
        self.check(0, "reg_profile=0 (bench-only fixed 915 MHz, from legT_params.txt)",
                   "bracket criteria: none on a bench-only fixed 915 MHz leg (reg_profile=0)",
                   absent=("[PASS]", "DTS command delivery"), params="reg_profile=0\n")
        # incomplete brackets still refuse, whatever the profile
        self.check(2, "INCOMPLETE BRACKETS", params=dts, pre_tractor=None)

    def test_profile_sources(self) -> None:
        # no params: auto refuses, a flag names the profile
        self.check(2, "PROFILE UNKNOWN -- not scored", "legT_params.txt missing",
                   "--profile fhss|dts", params=None)
        self.check(0, "reg_profile=1 (FHSS, from --profile fhss)", argv=["--profile", "fhss"],
                   params=None)
        self.check(0, "reg_profile=2 (DTS, from --profile dts)", "bracket criteria: none",
                   argv=["--profile", "dts"], params=None)
        self.check(0, "(FHSS, from --profile fhss, as legT_params.txt says)",
                   argv=["--profile", "fhss"])
        self.check(2, "--profile dts (reg_profile=2) contradicts reg_profile=1 in legT_params.txt",
                   argv=["--profile", "dts"])
        for text, why in (("reg_profile=3\n", "reg_profile='3' is not 0, 1 or 2"),
                          ("reg_profile=\n", "no reg_profile value"),
                          ("duration_s=300\n", "no reg_profile value")):
            self.check(2, "PROFILE UNKNOWN", why, params=text)
        # a flag with params that say nothing about the profile
        self.check(0, "(DTS, from --profile dts)", argv=["--profile", "dts"],
                   params="duration_s=300\n")
        # --params: a file or an archive dir; it replaces legT_params.txt
        with tempfile.TemporaryDirectory() as arch:
            with open(os.path.join(arch, "params.txt"), "w", encoding="utf-8") as fh:
                fh.write("﻿git_sha=1\nreg_profile=2\n")
            self.check(0, "(DTS, from ", "bracket criteria: none", argv=["--params", arch])
            self.check(0, "(DTS, from ", argv=["--params", os.path.join(arch, "params.txt")],
                       params=None)
            self.check(2, "--params ", "no params file at",
                       argv=["--params", os.path.join(arch, "nope")])

    def test_cli(self) -> None:
        with redirect_stdout(io.StringIO()), mock.patch("sys.stderr", io.StringIO()):
            for bad in ([], ["only_one"], ["d", "T", "--profile", "dfs"]):
                with self.assertRaises(SystemExit) as cm:
                    bd.main(bad)
                self.assertEqual(cm.exception.code, 2)
        with mock.patch.object(bd, "selftest_path", return_value=os.path.join(
                tempfile.gettempdir(), "no_such_dir", "test_bracket_deltas_sil.py")), \
                mock.patch("sys.stderr", io.StringIO()) as err:
            self.assertEqual(bd.main(["--selftest"]), 2)
        self.assertIn("not found", err.getvalue())
        self.assertEqual(os.path.normcase(bd.selftest_path()),
                         os.path.normcase(os.path.realpath(__file__)))


def _lines(text: str) -> list[str]:
    return text.splitlines()


@unittest.skipUnless((_RS1215 / "legV_deltas.txt").is_file() and (_RS13 / "legA2c2_params.txt")
                     .is_file(), "the 2026-10-18 / 10-19 legs are not in this tree")
class CommittedLegsTests(unittest.TestCase):
    def _score(self, legs: Path, leg: str) -> tuple[int, str]:
        out = io.StringIO()
        with redirect_stdout(out):
            rc = bd.main([str(legs), leg])
        return rc, out.getvalue()

    def test_rs12_15_fhss_legs_keep_their_numbers(self) -> None:
        for leg in ("V", "W", "W2", "X"):
            rc, out = self._score(_RS1215, leg)
            self.assertEqual(rc, 0, out)
            self.assertIn(f"leg {leg}: reg_profile=1 (FHSS, from leg{leg}_params.txt)", out)
            old = _lines((_RS1215 / f"leg{leg}_deltas.txt").read_text(encoding="utf-8"))
            new = _lines(out)
            # the reset lines, the counter table and the cross-check, line for line
            same = [ln for ln in old if ln.startswith(("base: ", "tractor: ", "cross-check: "))
                    or ln[2:].split(" ")[0] in bd.SHOWN]
            self.assertEqual(len(same), 3 + len(bd.SHOWN), leg)
            for ln in same:
                self.assertIn(ln, new, leg)
            self.assertEqual(sum(ln.lstrip().startswith("[PASS]") for ln in old), 6, leg)
            self.assertEqual(sum(ln.lstrip().startswith("[PASS]") for ln in new), 6, out)
            self.assertIn("[PASS] tractor tx_first_anchor delta = 1 (post 1;", out)
            self.assertIn("STOP RULE: not tripped", out)

    def test_rs13_dts_legs_report_without_a_false_fail(self) -> None:
        for leg in ("A2c", "A2a", "A2c2", "A2a2"):
            rc, out = self._score(_RS13, leg)
            self.assertEqual(rc, 0, out)
            self.assertIn(f"leg {leg}: reg_profile=2 (DTS, from leg{leg}_params.txt)", out)
            self.assertIn("bracket criteria: none on a DTS leg", out)
            for bad in ("[FAIL]", "[PASS]", "[UNVERIFIED]", "TRIPPED", "note: FHSS-only"):
                self.assertNotIn(bad, out, leg)
        rc, out = self._score(_RS13, "A2c2")
        # its reset did not land (params tractor_l072_reset=failed): post - pre, as committed
        old = _lines((_RS13 / "legA2c2_deltas.txt").read_text(encoding="utf-8"))
        for ln in old:
            if ln.startswith(("base: ", "tractor: ", "cross-check: ")) or \
                    ln[2:].split(" ")[0] in bd.SHOWN:
                self.assertIn(ln, _lines(out))
        self.assertIn("cross-check: brackets 211/211 vs logs: base sends on air=210, tractor "
                      "decoded cmds=209 -> DISAGREE or unavailable", out)


_R1024 = _DC / "bench-evidence" / "bench_session_2026-10-24" / "legs"


@unittest.skipUnless((_R1024 / "leg2b_f3_deltas.txt").is_file(),
                     "the 2026-10-24 legs are not in this tree")
class Round20261024Tests(unittest.TestCase):
    """The FHSS round: legs 1-5 print their committed deltas line for line; leg 6
    (2b_f3), whose base never locked, adds the flag and the STOP RULE pointer only."""

    def test_only_leg_6_is_flagged(self) -> None:
        for leg in ("2c_f1", "2b_f1", "2c_f2", "2b_f2", "2c_f3", "2b_f3"):
            out = io.StringIO()
            with redirect_stdout(out):
                rc = bd.main([str(_R1024), leg])
            new = _lines(out.getvalue())
            old = _lines((_R1024 / f"leg{leg}_deltas.txt").read_text(encoding="utf-8"))
            self.assertEqual(rc, 0, leg)
            if leg != "2b_f3":
                self.assertEqual(new, old, leg)
                continue
            self.assertEqual(new[:-2], old[:-1])
            self.assertEqual(old[-1], "STOP RULE: not tripped")
            self.assertTrue(new[-2].startswith(
                "!! BASE NEVER LOCKED: base d(clk_demotion_reset)=0 and d(fhss_dec_aligned)=0 "
                "while the tractor sent 607 (d(radio_tx_ok)); post radio_state 4 (RX_CONT) on "
                "both boards."), new[-2])
            self.assertIn("(4/5)", new[-2])
            self.assertEqual(new[-1], "STOP RULE: not tripped (!! BASE NEVER LOCKED above: this "
                                      "leg did not test FHSS reverse delivery)")


if __name__ == "__main__":
    unittest.main()
