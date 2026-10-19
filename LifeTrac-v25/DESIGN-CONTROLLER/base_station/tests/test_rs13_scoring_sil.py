"""bench_tools/rs13_*.py -- SIL tests for the RS-13 range-edge scoring helpers.

Pins, for the three helpers RS13_RANGE_EDGE_LEG.md scores legs with:

* rs13_probe_score.py: delivery = distinct tractor ``LoRa cmd: PROBE`` seqs
  that the base sent, over the base's ``PROBE TX`` count; a skipped tx# / rx#
  counter, a repeated base seq, a tractor seq the base never sent, a
  garbled PROBE line and an empty or error-only daemon log (a failed
  `docker logs`) fail closed (exit 2, pooled verdict withheld); a leg with
  no probe exits 1; a dead reverse link scores 0 and says so; the pooled
  VECTOR / mono_g4 verdict and the two-sided Fisher exact p (checked against
  the textbook tea-tasting table).
* rs13_rf_extract.py: the median of the 10-s ``rx_rf:`` window medians, the
  minimums and n; ``None`` axes; garbled or self-contradicting lines fail
  closed; a crashed or stopped stats thread and frames with no window give
  no level (exit 1); R0 over legs with its range check and the mixed-profile
  refusal; the FRF readback at the pick (a 915 MHz read-back under a 927.5
  pick, one board off, MISMATCH, a failed force-write); the leakage check;
  the RSSI tracking rule in both regions and the closing check.
* rs13_digest_trace.py: strict capture loading (meta profile, garbled
  lines); seq-gap rows that sum to rs12_leg_report's loss of record (wrap,
  codec switch, duplicate); the A16 rule on hand-built rows (PASS, late or
  epoch-start ends, letter-only fails, isolation, join, no loss, heavy loss,
  open episodes, a base that falls short again); P1's unexplained mismatch
  runs, measured from the same anchor for runs and episodes; a not-isolated
  episode with the A16 shape traced past the next loss and flagged beside
  the verdict; and end to end on a codec-built VS stream from the store
  tests, where the lost DEL's ghost holds the base out of step for
  TTL_FRAMES.
"""
from __future__ import annotations

import io
import json
import os
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from datetime import datetime, timedelta
from pathlib import Path
from unittest import mock

_BS = Path(__file__).resolve().parents[1]
_BT = _BS.parent / "firmware" / "x8_lora_bootloader_helper" / "bench_tools"
for _p in (_BS, _BT):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import rs13_digest_trace as dt  # noqa: E402
import rs13_probe_score as ps  # noqa: E402
import rs13_rf_extract as rf  # noqa: E402
import rs12_leg_report as rep  # noqa: E402  (on sys.path through rs13_digest_trace)
from image_pipeline.frame_format import (  # noqa: E402
    CODEC_MONO_G4, CODEC_VECTOR, FRAME_KIND_DELTA, FRAME_KIND_KEY, TileDeltaFrame,
    encode_tile_delta_frame,
)
from tests.test_vector_scene_store import body as vs_body, scenario  # noqa: E402

T0 = datetime(2026, 10, 18, 16, 0, 0)


def _ts(sec: float) -> str:
    t = T0 + timedelta(seconds=sec)
    return t.strftime("%Y-%m-%d %H:%M:%S") + ",%03d" % (t.microsecond // 1000)


def _run(main, argv) -> tuple[int, str]:
    buf = io.StringIO()
    with redirect_stdout(buf):
        rc = main(argv)
    return rc, buf.getvalue()


def _archive(root: str, name: str, params: dict, rx: str, tx: str | None = "") -> str:
    d = os.path.join(root, name)
    os.makedirs(d)
    with open(os.path.join(d, "params.txt"), "w", encoding="utf-8") as fh:
        fh.write("﻿" + "\n".join(f"{k}={v}" for k, v in params.items()) + "\n")
    with open(os.path.join(d, "rx_daemon.log"), "w", encoding="utf-8") as fh:
        fh.write(rx)
    if tx is not None:
        with open(os.path.join(d, "tx_daemon.log"), "w", encoding="utf-8") as fh:
            fh.write(tx)
    return d


# ---------------------------------------------------------------- probe score

def _base_log(seqs, failed=()) -> str:
    out = [f"{_ts(0)} INFO image_rx_daemon: image_rx_daemon started"]
    for k, s in enumerate(seqs, 1):
        out.append(f"{_ts(k)} INFO image_rx_daemon: PROBE TX seq={s} phase_ms=0 size_b=16 (tx#{k})")
        res = "FAILED: timeout" if s in failed else "OK (on air)"
        out.append(f"{_ts(k)} INFO image_rx_daemon: command TX opcode=0x69 copy=1/1 {res}")
        out.append(f"{_ts(k)} INFO image_rx_daemon: probe: tx={k} echo=0 (0.0% delivered)")
    return "\n".join(out) + "\n"


def _tractor_log(seqs) -> str:
    out = [f"{_ts(0)} INFO image_tx_daemon: frame seq=1 done (pipelined): 1 fragments ok"]
    out += [f"{_ts(k)} INFO image_tx_daemon: LoRa cmd: PROBE seq={s} phase_ms=0 (rx#{k})"
            for k, s in enumerate(seqs, 1)]
    return "\n".join(out) + "\n"


class ProbeScoreTests(unittest.TestCase):
    def test_delivered_over_sent(self) -> None:
        res = ps.score_leg(_base_log(range(1, 11)), _tractor_log([1, 2, 4, 7, 10]))
        self.assertEqual((res["sent"], res["delivered"], res["duplicates"]), (10, 5, 0))
        self.assertEqual((res["on_air"], res["failed"]), (10, 0))

    def test_a_probe_that_never_went_on_air_is_undelivered(self) -> None:
        res = ps.score_leg(_base_log(range(1, 5), failed={3}), _tractor_log([1, 2, 4]))
        self.assertEqual((res["sent"], res["delivered"], res["on_air"], res["failed"]),
                         (4, 3, 3, 1))

    def test_dead_reverse_link_scores_zero_loudly(self) -> None:
        res = ps.score_leg(_base_log(range(1, 87)), _tractor_log([]))
        self.assertEqual((res["sent"], res["delivered"]), (86, 0))
        text = "\n".join(ps.format_leg("x", {}, res))
        self.assertIn("no PROBE line at all", text)

    def test_tractor_duplicates_count_once(self) -> None:
        tx = _tractor_log([1, 2, 2, 3])
        res = ps.score_leg(_base_log(range(1, 4)), tx)
        self.assertEqual((res["delivered"], res["duplicates"]), (3, 1))

    def test_fail_closed(self) -> None:
        good_tx = _tractor_log([1])
        skipped = _base_log(range(1, 4)).replace("(tx#2)", "(tx#5)")
        with self.assertRaisesRegex(ps.ScoreError, "tx# counter"):
            ps.score_leg(skipped, good_tx)
        with self.assertRaisesRegex(ps.ScoreError, "rx# counter"):
            ps.score_leg(_base_log(range(1, 4)), _tractor_log([1, 2]).replace("(rx#2)", "(rx#3)"))
        with self.assertRaisesRegex(ps.ScoreError, "repeat"):
            ps.score_leg(_base_log([1, 2, 2]), good_tx)
        with self.assertRaisesRegex(ps.ScoreError, "never sent"):
            ps.score_leg(_base_log(range(1, 4)), _tractor_log([1, 9]))
        garbled = _base_log(range(1, 3)).replace("PROBE TX seq=2 phase_ms=0", "PROBE TX seq=2 phas")
        with self.assertRaisesRegex(ps.ScoreError, "does not parse"):
            ps.score_leg(garbled, good_tx)

    def test_probe_echo_lines_are_not_probes(self) -> None:
        tx = _tractor_log([1]) + f"{_ts(9)} INFO image_tx_daemon: LoRa cmd: PROBE_ECHO 4 B\n"
        self.assertEqual(ps.score_leg(_base_log([1]), tx)["delivered"], 1)

    def test_fisher_exact_two_sided(self) -> None:
        # Fisher's tea-tasting table: 3 of 4 right, two-sided p = 34/70.
        self.assertAlmostEqual(ps.fisher_two_sided(3, 1, 1, 3), 34 / 70, places=12)
        self.assertAlmostEqual(ps.fisher_two_sided(10, 0, 0, 10), 2 / 184756, places=15)
        self.assertEqual(ps.fisher_two_sided(5, 5, 5, 5), 1.0)
        self.assertEqual(ps.fisher_two_sided(0, 0, 0, 0), 1.0)

    def test_pooled_verdicts(self) -> None:
        self.assertIn("NOT WORSE (VECTOR >= mono_g4)", ps.pooled_verdict((90, 100), (85, 100))[-1])
        self.assertIn("NOT WORSE (VECTOR below", ps.pooled_verdict((88, 100), (90, 100))[-1])
        self.assertIn("WORSE (VECTOR below", ps.pooled_verdict((50, 100), (90, 100))[-1])
        self.assertNotIn("NOT", ps.pooled_verdict((50, 100), (90, 100))[-1])

    def test_main_pools_by_codec_from_params(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            vec = {"reg_profile": 2, "reactive_fire": 1, "probe_echo": 0,
                   "cam_extra_env": "-e LIFETRAC_ENCODE_MODE=9 -e LIFETRAC_VECTOR_DETAIL=80"}
            mono = dict(vec, cam_extra_env="-e LIFETRAC_ENCODE_MODE=6")
            a = _archive(root, "A2a", vec, _base_log(range(1, 11)), _tractor_log(range(1, 10)))
            b = _archive(root, "A2c", mono, _base_log(range(1, 11)), _tractor_log(range(1, 9)))
            rc, out = _run(ps.main, [a, b])
        self.assertEqual(rc, 0)
        self.assertIn("pooled VECTOR 9/10 = 90.0 %; pooled mono_g4 8/10 = 80.0 %", out)
        self.assertIn("NOT WORSE (VECTOR >= mono_g4)", out)

    def test_a_tractor_that_was_not_receiving_is_not_scored(self) -> None:
        vec = {"reg_profile": 2, "reactive_fire": 1, "probe_echo": 0,
               "cam_extra_env": "-e LIFETRAC_ENCODE_MODE=9"}
        mono = dict(vec, cam_extra_env="-e LIFETRAC_ENCODE_MODE=6")
        for bad in ("Exception in thread image-tx-worker:\n",
                    f"{_ts(1)} ERROR image_tx_daemon: fatal: cannot open L072 HostLink: timeout\n",
                    f"{_ts(0)} WARNING image_tx_daemon: RS-11.5 diagnostic: RXCONT arming DISABLED "
                    "— command downlink deaf for this run\n"):
            with tempfile.TemporaryDirectory() as root:
                a = _archive(root, "A2a", vec, _base_log(range(1, 11)),
                             _tractor_log(range(1, 4)) + bad)
                b = _archive(root, "A2c", mono, _base_log(range(1, 11)), _tractor_log(range(1, 10)))
                rc, out = _run(ps.main, [a, b])
            self.assertEqual(rc, 2, out)
            self.assertIn("A2a: CANNOT SCORE: tx_daemon.log:", out)
            self.assertIn("pooled verdict withheld", out)
            self.assertNotIn("NOT WORSE", out)
        # A leg that fired no probe is not scored on the tractor's log at all.
        with tempfile.TemporaryDirectory() as root:
            quiet = _archive(root, "C9v", {"reactive_fire": 0}, f"{_ts(0)} INFO x: started\n",
                             f"{_ts(1)} ERROR image_tx_daemon: fatal: cannot open L072 HostLink\n")
            rc, out = _run(ps.main, [quiet])
        self.assertEqual(rc, 1, out)
        self.assertIn("fired no probe", out)

    def test_pool_only_dts_legs_without_echo(self) -> None:
        vec = {"reg_profile": 2, "reactive_fire": 1, "probe_echo": 0,
               "cam_extra_env": "-e LIFETRAC_ENCODE_MODE=9"}
        for name, mono, why in (
                ("echo", dict(vec, probe_echo=1), "A2c (reg_profile=2, probe_echo=1)"),
                ("fhss", dict(vec, reg_profile=1), "A2c (reg_profile=1, probe_echo=0)"),
                ("unset", {"reactive_fire": 1, "cam_extra_env": "-e LIFETRAC_ENCODE_MODE=6"},
                 "A2c (reg_profile=?, probe_echo=?)")):
            mono = dict(mono, cam_extra_env="-e LIFETRAC_ENCODE_MODE=6")
            with tempfile.TemporaryDirectory() as root:
                a = _archive(root, "A2a", vec, _base_log(range(1, 11)), _tractor_log(range(1, 10)))
                b = _archive(root, "A2c", mono, _base_log(range(1, 11)), _tractor_log(range(1, 3)))
                rc, out = _run(ps.main, [a, b])
            self.assertEqual(rc, 1, (name, out))
            self.assertIn("pooled verdict withheld", out)
            self.assertIn(why, out)
            self.assertNotIn("NOT WORSE", out)
            self.assertIn("delivered 2/10 = 20.0 %", out)       # the per-leg figures still print
        # A leg that is neither VECTOR nor mono_g4 does not slip out of the pool unseen.
        mono = dict(vec, cam_extra_env="-e LIFETRAC_ENCODE_MODE=6")
        for extra in ("", "-e LIFETRAC_ENCODE_MODE=7"):
            with tempfile.TemporaryDirectory() as root:
                legs = [_archive(root, "A2a", vec, _base_log(range(1, 11)), _tractor_log(range(1, 10))),
                        _archive(root, "A2c", mono, _base_log(range(1, 11)), _tractor_log(range(1, 10))),
                        _archive(root, "C", dict(vec, cam_extra_env=extra), _base_log(range(1, 11)),
                                 _tractor_log([1]))]
                rc, out = _run(ps.main, legs)
            self.assertEqual(rc, 1, out)
            self.assertIn("these legs are neither: C (", out)
            self.assertNotIn("NOT WORSE", out)

    def test_main_exit_codes(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            noprobe = _archive(root, "B9v", {"reactive_fire": 0}, f"{_ts(0)} INFO x: started\n")
            rc, out = _run(ps.main, [noprobe])
            self.assertEqual(rc, 1)
            self.assertIn("fired no probe", out)
            rc, out = _run(ps.main, [os.path.join(root, "missing")])
            self.assertEqual(rc, 2)
            self.assertIn("CANNOT SCORE", out)
            notx = _archive(root, "notx", {}, _base_log([1]), tx=None)
            good = _archive(root, "good", {"cam_extra_env": "-e LIFETRAC_ENCODE_MODE=9"},
                            _base_log([1]), _tractor_log([1]))
            rc, out = _run(ps.main, [good, notx])
            self.assertEqual(rc, 2)
            self.assertIn("pooled verdict withheld", out)

    def test_an_empty_or_error_only_tractor_log_is_no_log(self) -> None:
        # The harness writes `docker logs tx_smoke`: a gone container or a failed
        # adb call leaves an empty or error-only file, which must not score 0
        # delivered into the pooled verdict.
        vec = {"reg_profile": 2, "reactive_fire": 1, "probe_echo": 0,
               "cam_extra_env": "-e LIFETRAC_ENCODE_MODE=9"}
        mono = dict(vec, cam_extra_env="-e LIFETRAC_ENCODE_MODE=6")
        for tx in ("", "Error response from daemon: No such container: tx_smoke\n"):
            with self.assertRaisesRegex(ps.ScoreError, "no timestamped daemon line"):
                ps.score_leg(_base_log(range(1, 11)), tx)
            with tempfile.TemporaryDirectory() as root:
                a = _archive(root, "A2a", vec, _base_log(range(1, 11)), _tractor_log(range(1, 10)))
                b = _archive(root, "A2c", mono, _base_log(range(1, 11)), tx)
                rc, out = _run(ps.main, [a, b])
            self.assertEqual(rc, 2, out)
            self.assertIn("A2c: CANNOT SCORE: tx_daemon.log holds no timestamped daemon line", out)
            self.assertIn("pooled verdict withheld", out)
            self.assertNotIn("NOT WORSE", out)
        with self.assertRaisesRegex(ps.ScoreError, "rx_daemon.log holds no timestamped"):
            ps.score_leg("", _tractor_log([1]))
        # A leg that fired no probe needs no tractor log: still exit 1, not 2.
        res = ps.score_leg(f"{_ts(0)} INFO x: started\n", "")
        self.assertEqual(res["sent"], 0)


# ---------------------------------------------------------------- rf extract

def _rxrf(t, rssi, snr, n=20) -> str:
    r = "None" if rssi is None else "/".join(str(x) for x in rssi)
    s = "None" if snr is None else "/".join(f"{x:.1f}" for x in snr)
    nr = 0 if rssi is None else n
    nq = 0 if snr is None else n
    return (f"{_ts(t)} INFO image_rx_daemon: rx_rf: n_rssi={nr} n_snr={nq} "
            f"rssi[min/med/max]={r} snr[min/med/max]={s}")


def _rf_log(meds, *, pub_until=None, extra=()) -> str:
    out = [f"{_ts(0)} INFO image_rx_daemon: opening L072 HostLink on /dev/ttymxc3 @ 921600",
           "  FRF readback: 0xE7E000 = 927.500 MHz (OK)"]
    t = 0.0
    for i, m in enumerate(meds, 1):
        t = 10.0 * i
        out.append(_rxrf(t, (m - 3, m, m + 2), (6.0, 8.0, 9.0)))
        out.append(f"{_ts(t)} INFO image_rx_daemon: stats: rx_frames={20 * i} "
                   f"frames_published={20 * i}")
    end = pub_until if pub_until is not None else t + 5.0
    k = 1.0
    while k <= end:
        out.append(f"{_ts(k)} INFO image_rx_daemon: published frame_id=0 seq=1 240 B")
        k += 1.0
    out += list(extra)
    return "\n".join(out) + "\n"


class RfExtractTests(unittest.TestCase):
    def test_median_of_window_medians(self) -> None:
        log = rf.parse_rx_log(_rf_log([-64, -66, -63, -70]))
        s = rf.summarize(log)
        self.assertEqual(s["rssi"]["windows"], 4)
        self.assertEqual(s["rssi"]["samples"], 80)
        self.assertEqual(s["rssi"]["med"], -65.0)           # median of -64, -66, -63, -70
        self.assertEqual(s["rssi"]["min_med"], -70.0)
        self.assertEqual(s["rssi"]["min"], -73.0)
        self.assertEqual(s["rssi"]["max"], -61.0)
        self.assertEqual(s["snr"]["med"], 8.0)
        self.assertIsNone(rf.unusable(log))

    def test_none_axis_and_crc_dumps(self) -> None:
        text = "\n".join([
            _rxrf(10, (-70, -66, -60), None),
            _rxrf(20, None, (2.0, 4.0, 5.5)),
            f"{_ts(21)} WARNING image_rx_daemon: crc_dump: irq=0x70 rx_len=251 snr=-10.5 "
            f"rssi=-46 dump=0102",
            f"{_ts(22)} WARNING image_rx_daemon: crc_dump: irq=0x70 rx_len=12 snr=6.2 "
            f"rssi=-65 dump=",
        ]) + "\n"
        s = rf.summarize(rf.parse_rx_log(text))
        self.assertEqual((s["rssi"]["windows"], s["snr"]["windows"]), (1, 1))
        self.assertEqual((s["rssi"]["med"], s["snr"]["med"]), (-66.0, 4.0))
        self.assertEqual(s["crc"], 2)
        self.assertEqual(s["crc_rssi"], (-55.5, -65, -46))

    def test_garbled_and_contradicting_lines_fail_closed(self) -> None:
        bad = _rxrf(10, (-70, -66, -60), (1.0, 2.0, 3.0)).replace("rssi[min/med/max]",
                                                                  "rssi[min/med")
        with self.assertRaisesRegex(rf.ExtractError, "rx_rf line does not parse"):
            rf.parse_rx_log(bad)
        disorder = _rxrf(10, (-60, -66, -70), (1.0, 2.0, 3.0))
        with self.assertRaisesRegex(rf.ExtractError, "contradict"):
            rf.parse_rx_log(disorder)
        no_count = _rxrf(10, (-70, -66, -60), (1.0, 2.0, 3.0)).replace("n_rssi=20", "n_rssi=0")
        with self.assertRaisesRegex(rf.ExtractError, "contradict"):
            rf.parse_rx_log(no_count)
        crc = f"{_ts(1)} WARNING image_rx_daemon: crc_dump: irq=0x70 rx_len=2 snr=bad rssi=-4 dump="
        with self.assertRaisesRegex(rf.ExtractError, "crc_dump line does not parse"):
            rf.parse_rx_log(crc)
        with self.assertRaisesRegex(rf.ExtractError, "no timestamped line"):
            rf.parse_rx_log("\n\n")

    def test_stale_windows_give_no_level(self) -> None:
        crashed = _rf_log([-64], extra=["Exception in thread image-rx-stats:"])
        self.assertIn("crashed", rf.unusable(rf.parse_rx_log(crashed)))
        stopped = _rf_log([-64, -64], pub_until=200.0)       # windows end at +20 s
        self.assertIn("stop", rf.unusable(rf.parse_rx_log(stopped)))
        tail_ok = _rf_log([-64, -64], pub_until=34.0)        # within one period + slack
        self.assertIsNone(rf.unusable(rf.parse_rx_log(tail_ok)))
        nowin = _rf_log([], pub_until=30.0)
        self.assertIn("no rx_rf window", rf.unusable(rf.parse_rx_log(nowin)))

    def test_main_r0_range_and_profiles(self) -> None:
        dts = {"reg_profile": 2, "force_frf_hz": 927500000}
        with tempfile.TemporaryDirectory() as root:
            ok = "  FRF readback: 0xE7E000 = 927.500 MHz (OK)\n"
            a = _archive(root, "A2c", dts, _rf_log([-67, -68, -67]), ok)
            b = _archive(root, "A2a", dts, _rf_log([-69, -70, -69]), ok)
            rc, out = _run(rf.main, [a, b])
            self.assertEqual(rc, 0, out)
            self.assertIn("R0 over 2 legs (median of the legs' medians): RSSI -68.0 dBm", out)
            self.assertIn("in range", out)
            self.assertIn("row: RSSI med / min -67.0 / -71 dBm", out)
            self.assertNotIn("without `FRF readback", out)
            hot = _archive(root, "hot", dts, _rf_log([-60, -61]), ok)
            rc, out = _run(rf.main, [a, hot])
            self.assertIn("OUT OF RANGE: above -65 dBm", out)
            fhss = _archive(root, "C0v", {"reg_profile": 1}, _rf_log([-67]), "")
            rc, out = _run(rf.main, [a, fhss])
            self.assertEqual(rc, 2)
            self.assertIn("mix reg_profile", out)
            stale = _archive(root, "U", dts, _rf_log([-64], pub_until=200.0), ok)
            rc, out = _run(rf.main, [a, stale])
            self.assertEqual(rc, 1)
            self.assertIn("NO LEVEL FOR THIS LEG", out)
            self.assertIn("R0 withheld", out)
            self.assertEqual(out.count("row: "), 1)          # the stale leg prints no row
            notx = _archive(root, "notx", dts, _rf_log([-67]), tx=None)
            rc, out = _run(rf.main, [notx])
            self.assertIn("without `FRF readback: ... (OK)` at the pick 927.500 MHz on both "
                          "boards (tractor none)", out)

    def test_frf_readback_at_the_pick(self) -> None:
        ok927 = ([(927.5, "OK")], False)
        ok915 = ([(915.0, "OK")], False)
        dts = {"reg_profile": "2", "force_frf_hz": "927500000"}
        self.assertEqual(rf.frf_check(dts, ok927, ok927), [])
        self.assertEqual(rf.frf_check({"reg_profile": "1"}, ([], False), None), [])
        # A19: a stale daemon that ignores LIFETRAC_FORCE_FRF_HZ reads back the
        # helper's 915 MHz default, (OK), while params name the pick.
        text = "\n".join(rf.frf_check(dts, ok915, ok915))
        self.assertIn("at the pick 927.500 MHz on both boards (base 915.000 MHz (OK); tractor "
                      "915.000 MHz (OK))", text)
        self.assertIn("on 915.000 MHz (by the base read-back and the tractor read-back)", text)
        text = "\n".join(rf.frf_check(dts, ok927, ok915))      # one board off the pick
        self.assertIn("(tractor 915.000 MHz (OK))", text)
        # -ForceFrfHz 0 (or no force_frf_hz line): the pick is the helper's 915 MHz.
        for p0 in ({"reg_profile": "2", "force_frf_hz": "0"}, {"reg_profile": "2"}):
            text = "\n".join(rf.frf_check(p0, ok915, ok915))
            self.assertNotIn("without", text)
            self.assertIn("by params (force_frf_hz 0 or absent", text)
        self.assertIn("without", "\n".join(rf.frf_check(dts, ok927, ([(927.5, "OK")], True))))
        self.assertIn("not an integer", "\n".join(
            rf.frf_check({"reg_profile": "2", "force_frf_hz": "9x"}, ok927, ok927)))
        # The helper's mismatch status parses, is shown, and fails the check.
        rbs = rf.frf_readbacks("  FRF readback: 0xE7E000 = 927.500 MHz (**MISMATCH**)\n"
                               "  WARN: FRF force-write failed: timeout\n")
        self.assertEqual(rbs, ([(927.5, "**MISMATCH**")], True))
        self.assertEqual(rf._frf_text(*rbs), "927.500 MHz (**MISMATCH**), FRF force-write FAILED")
        text = "\n".join(rf.frf_check(dts, ([(927.5, "**MISMATCH**")], False), ok927))
        self.assertIn("(base 927.500 MHz (**MISMATCH**))", text)
        with tempfile.TemporaryDirectory() as root:
            mm = _rf_log([-67]).replace("927.500 MHz (OK)", "927.500 MHz (**MISMATCH**)")
            a = _archive(root, "mm", {"reg_profile": 2, "force_frf_hz": 927500000}, mm,
                         "  FRF readback: 0xE7E000 = 927.500 MHz (OK)\n")
            rc, out = _run(rf.main, [a])
        self.assertIn("FRF readback: base 927.500 MHz (**MISMATCH**); tractor 927.500 MHz (OK)",
                      out)
        self.assertIn("an abort condition", out)

    def test_leak_check(self) -> None:
        frf = "  FRF readback: 0xE4C000 = 915.000 MHz (OK)\n"     # the DTS pick (915 by default)
        with tempfile.TemporaryDirectory() as root:
            quiet = (f"{_ts(0)} INFO image_rx_daemon: started\n" + frf
                     + f"{_ts(10)} INFO image_rx_daemon: stats: rx_frames=0 frames_published=0\n"
                     f"{_ts(12)} WARNING image_rx_daemon: crc_dump: irq=0x70 rx_len=9 snr=-14.0 "
                     f"rssi=-121 dump=00\n")
            a = _archive(root, "leak_dts", {"reg_profile": 2, "duration_s": 20}, quiet, frf)
            rc, out = _run(rf.main, ["--leak", a])
            self.assertEqual(rc, 0, out)
            self.assertIn("leakage check: PASS", out)
            self.assertIn("med/max -121/-121 dBm", out)
            heard = _archive(root, "leak_fhss", {"reg_profile": 1}, _rf_log([-110]), "")
            rc, out = _run(rf.main, ["--leak", heard])
            self.assertEqual(rc, 1)
            self.assertIn("leakage check: FAIL", out)
            # Nothing decoded is evidence only while the receiver ran, its stats thread
            # reported to the end and across the leg, and a DTS leg sat on the pick.
            long_quiet = (f"{_ts(0)} INFO image_rx_daemon: started\n" + frf + "".join(
                f"{_ts(10 * i)} INFO image_rx_daemon: stats: rx_frames=0 frames_published=0\n"
                for i in range(1, 13)))
            for name, params, rx, tx, why in (
                    ("no_stats", {"reg_profile": 2},
                     f"{_ts(0)} INFO image_rx_daemon: started\n"
                     f"{_ts(1)} ERROR image_rx_daemon: HostLink init failed\n", frf,
                     "no `rx_frames=` stats line"),
                    ("crashed", {"reg_profile": 2}, quiet + "Exception in thread image-rx-stats:\n",
                     frf, "crashed"),
                    ("stopped", {"reg_profile": 2},
                     quiet + f"{_ts(60)} INFO image_rx_daemon: still running\n", frf,
                     "the stats thread stopped"),
                    # radio_monitor_20260724_170132's shape: one stats line, then the daemon dies.
                    ("fatal", {"reg_profile": 1, "duration_s": 60},
                     f"{_ts(0)} INFO image_rx_daemon: started\n"
                     f"{_ts(1)} INFO image_rx_daemon: stats: rx_frames=0 frames_published=0\n"
                     f"{_ts(1.6)} ERROR image_rx_daemon: fatal: cannot open L072 HostLink: timeout\n"
                     f"{_ts(2.1)} INFO image_rx_daemon: image_rx_daemon exit\n", "",
                     "never opened its L072 HostLink"),
                    ("short", {"reg_profile": 1, "duration_s": 300}, quiet, "",
                     "cover 10 s of a 300 s leg"),
                    ("off_pick", {"reg_profile": 2, "force_frf_hz": 927500000, "duration_s": 120},
                     long_quiet.replace("915.000 MHz (OK)", "915.000 MHz (**MISMATCH**)"),
                     "  FRF readback: 0xE7E000 = 927.500 MHz (OK)\n",
                     "did not read back (OK) at the pick")):
                rc, out = _run(rf.main, ["--leak", _archive(root, name, params, rx, tx)])
                self.assertEqual(rc, 1, (name, out))
                self.assertIn("leakage check: NOT SCORED", out)
                self.assertIn(why, out)
                self.assertNotIn("leakage check: PASS", out)
            # The receive worker died; the stats thread printed zeros to the end.
            rc, out = _run(rf.main, ["--leak", _archive(
                root, "worker", {"reg_profile": 1, "duration_s": 120},
                long_quiet.replace(f"{_ts(20)} INFO", "Exception in thread image-rx-worker:\n"
                                   f"{_ts(20)} INFO", 1), "")])
            self.assertEqual(rc, 1, out)
            self.assertIn("leakage check: NOT SCORED", out)
            self.assertIn("the receive worker crashed", out)
            # Coverage needs the leg's length: a missing or malformed duration_s is no evidence.
            for dur in (None, "120s", "nan", "inf", "0", "-120"):
                params = {"reg_profile": 1} if dur is None else {"reg_profile": 1, "duration_s": dur}
                rc, out = _run(rf.main, ["--leak", _archive(root, f"dur_{dur}".replace("-", "m"),
                                                            params, long_quiet, "")])
                self.assertEqual(rc, 1, (dur, out))
                self.assertIn("leakage check: NOT SCORED", out)
                self.assertIn("is not a positive number of seconds", out)
            ok = {"reg_profile": 1, "duration_s": 120}
            rc, out = _run(rf.main, ["--leak", _archive(root, "full_leg", ok, long_quiet, "")])
            self.assertEqual(rc, 0, out)                         # 120 s of stats for a 120 s leg
            self.assertIn("leakage check: PASS", out)

    def test_tracking_rule(self) -> None:
        line = rf.tracking(-77.5, 5.0, -67.0, 9.0, -114.0, None, None)[0]
        self.assertIn("expected R0 - step = -76.0 dBm", line)
        self.assertIn("HOLDS", line)
        self.assertIn("BROKEN", rf.tracking(-80.0, 5.0, -67.0, 9.0, -114.0, None, None)[0])
        self.assertIn("assumed the RSSI region",
                      rf.tracking(-76.0, 5.0, -67.0, 9.0, None, None, None)[0])
        snr = rf.tracking(-108.0, 2.0, -67.0, 48.0, -114.0, 5.0, 45.0)[0]
        self.assertIn("SNR region", snr)
        self.assertIn("falls 3.0 dB per 3 dB step", snr)
        self.assertIn("HOLDS", snr)
        self.assertIn("BROKEN", rf.tracking(-108.0, 4.5, -67.0, 48.0, -114.0, 5.0, 45.0)[0])
        self.assertIn("NO VERDICT", rf.tracking(-108.0, 2.0, -67.0, 48.0, -114.0, None, None)[0])

    def test_closing_and_usage(self) -> None:
        with tempfile.TemporaryDirectory() as root:
            a = _archive(root, "B0close", {"reg_profile": 2}, _rf_log([-66, -66]), "")
            rc, out = _run(rf.main, [a, "--r0", "-67.5", "--closing"])
            self.assertIn("closing check: median RSSI -66.0 dBm vs R0 -67.5 dBm "
                          "(delta +1.5 dB", out)
            self.assertIn("-> OK", out)
            rc, out = _run(rf.main, [a, "--r0", "-69", "--closing"])
            self.assertIn("SUSPECT", out)
            for bad in (["--r0", "-67"], ["--step", "9"], ["--r0", "-67", "--step", "9", a],
                        ["--r0", "-67", "--step", "9", "--prev-snr", "3"],
                        ["--r0", "-67", "--step", "9", "--prev-snr", "3", "--prev-step", "9"]):
                with self.assertRaises(SystemExit), redirect_stdout(io.StringIO()):
                    with open(os.devnull, "w") as err:
                        old, sys.stderr = sys.stderr, err
                        try:
                            rf.main([a] + bad)
                        finally:
                            sys.stderr = old


# ---------------------------------------------------------------- digest trace

def _row(idx, *, seq=None, codec=CODEC_VECTOR, kind=0, check=None, digest="-", t=None):
    return {"idx": idx, "t": 0.5 * idx if t is None else t, "parsed": True, "codec": codec,
            "kind": kind, "seq": idx + 1 if seq is None else seq, "vector": codec == CODEC_VECTOR,
            "wire": 240, "records": 40, "digest": digest,
            "checks": [check] if check else [], "orph_add": 0, "ttl": 0, "resync": False,
            "added": [], "removed": []}


def _rows(n, *, checks=None, key_rows=(0,)):
    """n rows, each with a DIGEST; ``checks`` maps row -> (enc, base) (default equal)."""
    checks = checks or {}
    rows = []
    for i in range(n):
        c = checks.get(i, (20, 20))
        rows.append(_row(i, kind=1 if i in key_rows else 0, check=c,
                         digest="ok" if c[0] == c[1] else "BAD"))
    return rows


def _ep(start, end, ended_by="digest"):
    return {"start": start, "end": end, "frames": (end if end is not None else 999) - start,
            "t_start_s": 0.5 * start, "t_end_s": None if end is None else 0.5 * end,
            "ended_by": ended_by if end is not None else None}


class DigestTraceLoadTests(unittest.TestCase):
    def _write(self, lines) -> str:
        fd, path = tempfile.mkstemp(suffix="_base.jsonl")
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")
        self.addCleanup(os.remove, path)
        return path

    def test_meta_profile_and_payloads(self) -> None:
        p = self._write([json.dumps({"meta": {"tool": "vector_dry_run", "profile": "image_bw500"}}),
                         json.dumps({"ts": 1.5, "topic": "t", "hex": "0102"}), ""])
        prof, frames = dt.load_capture(p)
        self.assertEqual((prof, frames), ("image_bw500", [(1.5, b"\x01\x02")]))

    def test_garbled_lines_fail_closed(self) -> None:
        meta = json.dumps({"meta": {"profile": "image_bw500"}})
        ok = json.dumps({"ts": 1.0, "hex": "01"})
        for bad, why in (('{"ts": 1.0, "hex": "01', "not JSON"),
                         (json.dumps({"ts": 1.0}), "neither"),
                         (json.dumps({"ts": 1.0, "hex": "0g"}), "bad ts or hex"),
                         (json.dumps([1, 2]), "neither")):
            with self.assertRaisesRegex(dt.TraceError, why):
                dt.load_capture(self._write([meta, ok, bad]))
        with self.assertRaisesRegex(dt.TraceError, "no payload"):
            dt.load_capture(self._write([meta]))
        bw250 = json.dumps({"meta": {"profile": "image_bw250"}})
        for lines in ([meta, ok, bw250, ok], [meta, ok, meta, ok], [ok, meta]):
            with self.assertRaisesRegex(dt.TraceError, "holds two captures"):
                dt.load_capture(self._write(lines))
        with self.assertRaisesRegex(dt.TraceError, "missing"):
            dt.load_capture(os.path.join(tempfile.gettempdir(), "no_such_capture.jsonl"))

    def test_replay_errors_fail_closed(self) -> None:
        meta = json.dumps({"meta": {"profile": "image_bw500"}})
        # A valid JSON row whose payload is truncated: the replay cannot parse it.
        p = self._write([meta, json.dumps({"ts": 1.0, "hex": "01"})])
        with self.assertRaisesRegex(dt.TraceError, "1 payload\\(s\\) the replay cannot parse"):
            dt.analyse(p, None, 25, 25, 10.0)
        rc, out = _run(dt.main, [p])
        self.assertEqual(rc, 2, out)
        self.assertIn("CANNOT TRACE", out)
        # A VECTOR body the store rejects.
        store = mock.Mock(stats={"frames_bad": 1, "bad_reasons": {"bad body": 1}})
        with mock.patch.object(dt, "replay", return_value=(mock.Mock(parse_errors=0, store=store),
                                                           [])):
            with self.assertRaisesRegex(dt.TraceError, "1 VECTOR body\\(ies\\) the store rejected"):
                dt.analyse(p, None, 25, 25, 10.0)


class LossRowTests(unittest.TestCase):
    def _check(self, rows) -> list:
        gaps, _ = dt.loss_rows(rows)
        seq = rep.seq_gap_loss([(r["t"], r["codec"], r["seq"]) for r in rows])
        self.assertEqual(sum(n for _, n in gaps), seq["lost"])
        return gaps

    def test_rows_after_each_gap(self) -> None:
        rows = [_row(i, seq=s) for i, s in enumerate([1, 2, 4, 5, 8, 9])]
        self.assertEqual(self._check(rows), [(2, 1), (4, 2)])

    def test_wrap_switch_and_duplicate(self) -> None:
        seqs = [(254, CODEC_VECTOR), (255, CODEC_VECTOR), (1, CODEC_VECTOR), (1, CODEC_VECTOR),
                (2, CODEC_VECTOR), (64, CODEC_MONO_G4), (66, CODEC_MONO_G4)]
        rows = [_row(i, seq=s, codec=c) for i, (s, c) in enumerate(seqs)]
        # 0 lost across the wrap, the duplicate is no loss, the codec switch is not compared
        self.assertEqual(self._check(rows), [(2, 1), (6, 1)])

    def test_outage_longer_than_the_seq_space(self) -> None:
        rows = [_row(0, seq=10, t=0.0), _row(1, seq=11, t=0.5), _row(2, seq=12, t=1.0),
                _row(3, seq=13, t=1.0 + 0.5 * 257)]
        self.assertEqual(self._check(rows), [(3, 256)])


class A16RuleTests(unittest.TestCase):
    def test_lost_del_ghost_within_horizon_passes(self) -> None:
        # Lost DEL at row 10: the base holds a ghost (one shape more) to row 29,
        # the counts agree from row 30, and 3 matching DIGESTs end it at 32.
        checks = {i: (20, 21) for i in range(10, 30)}
        rows = _rows(80, checks=checks)
        res = dt.score_episodes(rows, [_ep(12, 32)], [(10, 1)])[0]
        self.assertEqual((res["status"], res["loss_row"], res["isolated"], res["back_row"]),
                         ("PASS", 10, True, 30))
        self.assertEqual(res["window"], (10, 60))           # max(32, 10 + 25) + 25
        res = dt.score_episodes(rows, [_ep(12, 32)], [(10, 1)], horizon=20)[0]
        self.assertEqual(res["status"], "FAIL")
        self.assertIn("ended 22 rows after the loss (> 20)", res["reasons"])

    def test_pass_and_letter_only(self) -> None:
        checks = {i: (20, 21) for i in range(10, 20)}
        rows = _rows(80, checks=checks)
        res = dt.score_episodes(rows, [_ep(12, 24)], [(10, 1)])[0]
        self.assertEqual((res["status"], res["back_row"], res["reasons"]), ("PASS", 20, []))
        res = dt.score_episodes(rows, [_ep(12, 24, "epoch start")], [(10, 1)])[0]
        self.assertEqual(res["status"], "FAIL")
        self.assertTrue(res["letter_only"])
        res = dt.score_episodes(rows, [_ep(12, 40, "epoch start")], [(10, 1)])[0]
        self.assertEqual(res["status"], "FAIL")
        self.assertFalse(res["letter_only"])
        self.assertIn("ended 30 rows after the loss (> 25)", res["reasons"])

    def test_a16_shape_fails(self) -> None:
        # ghost, then the base one SHORT until the epoch start at row 60 (RS-13.1 A16)
        checks = {**{i: (20, 21) for i in range(10, 30)}, **{i: (21, 20) for i in range(30, 60)}}
        rows = _rows(90, checks=checks, key_rows=(0, 60))
        res = dt.score_episodes(rows, [_ep(12, 60, "epoch start")], [(10, 1)])[0]
        self.assertEqual(res["status"], "FAIL")
        self.assertEqual(res["back_row"], 60)
        text = " ".join(res["reasons"])
        self.assertIn("ended by epoch start, not by DIGEST", text)
        self.assertIn("back in step 50 rows after the loss", text)

    def test_falls_short_again_after_the_episode(self) -> None:
        checks = {**{i: (20, 21) for i in range(10, 14)}, 40: (21, 20)}
        rows = _rows(90, checks=checks)
        res = dt.score_episodes(rows, [_ep(12, 18)], [(10, 1)])[0]
        self.assertEqual(res["status"], "FAIL")
        self.assertIn("again after it, at row(s) [40]", " ".join(res["reasons"]))
        self.assertEqual(res["window"], (10, 60))           # max(18, 10 + 25) + 25

    def test_crc_only_loss_is_back_at_the_loss_row(self) -> None:
        rows = _rows(60)
        for i in range(10, 16):
            rows[i]["digest"] = "BAD"                       # counts equal, crc off
        res = dt.score_episodes(rows, [_ep(12, 16)], [(10, 1)])[0]
        self.assertEqual((res["status"], res["back_row"]), ("PASS", 10))

    def test_not_scored_and_classes(self) -> None:
        rows = _rows(80, key_rows=(5,))
        eps = dt.score_episodes(rows, [_ep(1, 4), _ep(12, 20), _ep(50, 55)], [(10, 1), (70, 1)])
        self.assertEqual([e["status"] for e in eps], ["join", "PASS", "no loss"])
        self.assertEqual([e["loss_row"] for e in eps], [None, 10, None])
        eps = dt.score_episodes(rows, [_ep(12, 20)], [(10, 1), (20, 1)])
        self.assertEqual(eps[0]["status"], "not scored")
        self.assertFalse(eps[0]["isolated"])
        self.assertEqual(eps[0]["window"], (10, 60))         # traced across the next loss
        self.assertIsNone(eps[0]["back_row"])                # not counted: n/a
        self.assertIsNone(eps[0]["a16_shape"])
        eps = dt.score_episodes(rows, [_ep(12, 20)], [(10, 1)], loss_pct=12.0)
        self.assertIn("12.0 % > 10 %", eps[0]["reasons"][0])
        eps = dt.score_episodes(rows, [_ep(72, None)], [(70, 1)])
        self.assertEqual(eps[0]["status"], "not scored")    # open, capture ends 9 rows on
        eps = dt.score_episodes(rows, [_ep(42, None)], [(40, 1)])
        self.assertEqual(eps[0]["status"], "FAIL")          # open 39 rows after the loss
        self.assertIn("still in resync", eps[0]["reasons"][0])

    def test_a_burst_is_not_isolated(self) -> None:
        # Two frames lost in one gap: the rule bounds the recovery from ONE lost frame.
        rows = _rows(80, key_rows=(5,))
        res = dt.score_episodes(rows, [_ep(12, 20)], [(10, 2)])[0]
        self.assertEqual((res["status"], res["isolated"], res["burst"], res["back_row"]),
                         ("not scored", False, 2, None))
        self.assertIn("2 frames lost in one gap", res["reasons"][0])
        self.assertIsNotNone(res["window"])                  # still traced
        res = dt.score_episodes(rows, [_ep(12, 20)], [(10, 3), (20, 1)])[0]
        self.assertIn("3 frames lost in one gap", res["reasons"][0])
        self.assertIn("another loss at row 20", res["reasons"][0])
        self.assertEqual(dt.score_episodes(rows, [_ep(12, 20)], [(10, 1)])[0]["status"], "PASS")
        # The RESULTS cells RS13_RANGE_EDGE_LEG.md quotes.
        for gaps, back, iso in (([(10, 2)], "n/a (burst of 2 frames)", "no (burst of 2 frames)"),
                                ([(10, 3), (20, 1)], "n/a (burst of 3 frames, next loss at row 20)",
                                 "no (burst of 3 frames, row 20)")):
            eps = dt.score_episodes(rows, [_ep(12, 20)], gaps)
            res = {"path": "legX_base.jsonl", "leg": "X", "profile": "image_bw500", "rows": rows,
                   "dr": _StubDryRun([_ep(12, 20)]), "gaps": gaps, "jumps": 0,
                   "seq": rep.seq_gap_loss([(r["t"], r["codec"], r["seq"]) for r in rows]),
                   "loss_pct": 0.0, "episodes": eps, "runs": [], "vector": len(rows)}
            row = next(ln for ln in dt.report(res, 25, None) if ln.startswith("| X | 10 |"))
            self.assertIn(f"| {back} | {iso} |", row)

    def test_reentry_from_the_same_loss(self) -> None:
        rows = _rows(80)
        eps = dt.score_episodes(rows, [_ep(12, 15), _ep(30, 33)], [(10, 1)])
        self.assertEqual([e["reentry"] for e in eps], [False, True])
        self.assertEqual(eps[1]["status"], "PASS")          # 33 - 10 = 23 <= 25

    def test_mismatch_runs(self) -> None:
        rows = _rows(60, key_rows=(3,))
        for i in (1, 2, 12, 13, 40, 41):
            rows[i]["digest"] = "BAD"
        rows[14]["digest"] = "-"                            # no DIGEST: does not break the run
        rows[15]["digest"] = "BAD"
        runs = dt.mismatch_runs(rows, [(11, 1)], join=3)
        self.assertEqual(runs, [(1, 2, True), (12, 15, True), (40, 41, False)])

    def test_runs_split_at_a_digest_free_epoch_start(self) -> None:
        # The base joined mid-epoch (rows 0-9, BAD DIGESTs); the key frame at
        # row 10 carries no DIGEST and starts the epoch; a loss at row
        # 12 opens new mismatches on 13-16. The join's run must not swallow them.
        rows = _rows(60, key_rows=(10,))
        for r in rows:
            r["epoch_start"] = r["boundary"] = r["idx"] == 10
        for i in range(0, 10):
            rows[i]["digest"], rows[i]["checks"] = "BAD", [(20, 19)]
        for i in (10, 11, 12):
            rows[i]["digest"], rows[i]["checks"] = "-", []
        for i in range(13, 17):
            rows[i]["digest"], rows[i]["checks"] = "BAD", [(20, 19)]
        self.assertEqual(dt.join_row(rows), 10)
        self.assertEqual(dt.mismatch_runs(rows, [(12, 1)], dt.join_row(rows)),
                         [(0, 9, True), (13, 16, True)])
        res = dt.score_episodes(rows, [_ep(15, 20)], [(12, 1)])[0]
        self.assertEqual((res["anchor"], res["loss_row"], res["status"]), (13, 12, "PASS"))
        for r in rows:                                       # the old rule: one run from row 0
            r["boundary"] = False
        self.assertEqual(dt.score_episodes(rows, [_ep(15, 20)], [(12, 1)])[0]["status"], "join")

    def test_join_ends_at_the_repeat_once_anchor(self) -> None:
        # The key frame was lost: the epoch start the store accepts is its
        # repeat-once copy (row 3, a delta frame carrying the anchor and
        # LAYER_CLEAR); the next key frame is row 50.
        rows = _rows(80, key_rows=(50,))
        for r in rows:
            r["epoch_start"] = r["boundary"] = r["idx"] in (3, 50)
        self.assertEqual(dt.join_row(rows), 3)
        res = dt.score_episodes(rows, [_ep(22, 30)], [(20, 1)])[0]
        self.assertEqual((res["status"], res["loss_row"]), ("PASS", 20))
        for r in rows:                                       # the old first-key rule
            del r["epoch_start"]
        self.assertEqual(dt.score_episodes(rows, [_ep(22, 30)], [(20, 1)])[0]["status"], "join")

    def test_episode_and_its_mismatch_run_share_the_p1_anchor(self) -> None:
        # Loss at row 10, a DIGEST-quiet stretch (rows 10-33), BAD DIGESTs on
        # 34-37: the run starts 24 rows after the loss (explained), and the
        # store enters resync on its third BAD (row 36), 26 rows after it. The
        # episode is measured from the run's first BAD, so it is not P1 either.
        rows = _rows(90)
        for i in range(10, 34):
            rows[i]["digest"], rows[i]["checks"] = "-", []
        for i in range(34, 38):
            rows[i]["digest"], rows[i]["checks"] = "BAD", [(21, 20)]
        runs = dt.mismatch_runs(rows, [(10, 1)], join=0)
        self.assertEqual(runs, [(34, 37, True)])
        res = dt.score_episodes(rows, [_ep(36, 60)], [(10, 1)])[0]
        self.assertEqual((res["anchor"], res["loss_row"]), (34, 10))
        self.assertNotEqual(res["status"], "no loss")
        self.assertEqual(res, dt.score_episodes(rows, [_ep(36, 60)], [(10, 1)], runs=runs)[0])
        # A run with no loss before it: both are P1, and the reason names the run.
        res = dt.score_episodes(rows, [_ep(36, 60)], [])[0]
        self.assertEqual(res["status"], "no loss")
        self.assertIn("before row 34, where its DIGEST mismatch run starts", res["reasons"][0])
        self.assertEqual(dt.mismatch_runs(rows, [], join=0), [(34, 37, False)])
        # A second loss between the run's start and the resync entry is not
        # what opened it: the episode keeps the loss before the run.
        res = dt.score_episodes(rows, [_ep(36, 60)], [(10, 1), (35, 1)])[0]
        self.assertEqual((res["loss_row"], res["isolated"], res["next_loss"]), (10, False, 35))

    def test_not_isolated_a16_shape_is_traced_and_flagged(self) -> None:
        # The RS-13.1 2d_r4 shape: a loss at row 10, a second one 11 rows later
        # (not isolated), the base one short of the encoder from row 30 until
        # the epoch start at row 70 (its own check follows the reset).
        checks = {**{i: (20, 21) for i in range(10, 22)}, **{i: (21, 20) for i in range(30, 70)}}
        rows = _rows(110, checks=checks, key_rows=(0, 70))
        gaps, eps = [(10, 1), (21, 1)], [_ep(12, 70, "epoch start")]
        e = dt.score_episodes(rows, eps, gaps)[0]
        self.assertEqual((e["status"], e["isolated"], e["back_row"]), ("not scored", False, None))
        self.assertEqual(e["window"], (10, 95))              # max(70, 10 + 25) + 25
        self.assertIn("an epoch start ended it 60 rows after the loss", e["a16_shape"])
        self.assertIn("40 DIGEST row(s) #30-#69", e["a16_shape"])
        seq = rep.seq_gap_loss([(r["t"], r["codec"], r["seq"]) for r in rows])
        res = {"path": "legX_base.jsonl", "leg": "X", "profile": "image_bw500",
               "dr": _StubDryRun(eps), "rows": rows, "gaps": gaps, "jumps": 0, "seq": seq,
               "loss_pct": 0.0, "episodes": [e], "runs": dt.mismatch_runs(rows, gaps, 0),
               "vector": len(rows)}
        with mock.patch.object(dt, "analyse", return_value=res):
            rc, out = _run(dt.main, ["legX_base.jsonl"])
        self.assertEqual(rc, 0, out)
        trace = [ln[:4] for ln in out.splitlines() if ln.startswith("#") and ln[1:4].strip()
                 .isdigit()]
        self.assertIn("# 45", trace)                         # past the next loss: printed
        self.assertEqual((trace[0], trace[-1]), ("# 10", "# 95"))
        row = next(ln for ln in out.splitlines() if ln.startswith("| X | 10 |"))
        self.assertIn("| n/a (next loss at row 21) | no (row 21) |", row)
        self.assertIn("!! A16 shape: an epoch start ended it 60 rows", row)
        self.assertIn("-> NO SCORED EPISODE (!! A16 shape in 1 not-scored episode(s), below)", out)
        self.assertIn("!! A16 shape in not-scored episode(s): X #12-#70 (loss at row 10:", out)
        # A digest end within the horizon has no A16 shape, isolated or not.
        e = dt.score_episodes(_rows(80, checks={i: (20, 21) for i in range(10, 20)}),
                              [_ep(12, 24)], [(10, 1), (21, 1)])[0]
        self.assertIsNone(e["a16_shape"])


class _StubDryRun:
    """The two DryRun methods report() reads, for hand-built rows."""

    def __init__(self, eps):
        self._eps = eps

    def resync_episodes(self):
        return self._eps

    def summary(self):
        return {"store": {"ttl_dropped": 0, "orphans": 0, "digest_checks": 0,
                          "digest_mismatch": 0}, "scene": None, "resync_episodes": self._eps}


def _wire(body: bytes, key: bool, seq: int) -> bytes:
    return encode_tile_delta_frame(TileDeltaFrame(
        frame_kind=FRAME_KIND_KEY if key else FRAME_KIND_DELTA, base_seq=seq,
        grid_w=12, grid_h=8, tile_px=32, codec=CODEC_VECTOR, vector_body=body))


class DigestTraceEndToEndTests(unittest.TestCase):
    """A codec-built VS stream (the store tests' simulated tractor: CONFIRM +
    DIGEST in every frame, a DEL at frame 3, an epoch start at frame 44)
    replayed through the real store from a capture file."""

    def _capture(self, drops=(), seq0=1, profile="image_bw500") -> str:
        _tr, frames = scenario(1, n_frames=80)
        fd, path = tempfile.mkstemp(prefix="legE2E_", suffix="_base.jsonl")
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            if profile:
                meta = {"meta": {"tool": "vector_dry_run", "profile": profile}}
                fh.write(json.dumps(meta) + "\n")
            for f, (key, epoch, recs) in enumerate(frames):
                if f in drops:
                    continue
                fh.write(json.dumps({"ts": 20.25 + 0.5 * f, "topic": "t", "hex": _wire(
                    vs_body(key, epoch, recs), key, (f + seq0) % 256).hex()}) + "\n")
        self.addCleanup(os.remove, path)
        return path

    def test_clean_stream_has_no_episode(self) -> None:
        rc, out = _run(dt.main, [self._capture(), "--strict"])
        self.assertEqual(rc, 0, out)
        self.assertIn("lost 0/80", out)
        self.assertIn("no resync episode", out)
        self.assertIn("-> NO SCORED EPISODE", out)

    def test_lost_del_ghost_recovers_by_digest(self) -> None:
        # Frame 3 (the DEL) lost, numbered across the u8 wrap: row 3 is the
        # first frame after the gap; the ghost lives TTL_FRAMES applied frames.
        path = self._capture(drops={3}, seq0=250)
        res = dt.analyse(path, None, 25, 25, 10.0)
        self.assertEqual(res["gaps"], [(3, 1)])
        self.assertEqual(res["profile"], "image_bw500")
        self.assertEqual(len(res["episodes"]), 1)
        e = res["episodes"][0]
        self.assertEqual((e["loss_row"], e["ended_by"], e["status"]), (3, "digest", "PASS"))
        self.assertEqual(e["back_row"] - 3, 20)              # TTL_FRAMES
        self.assertLessEqual(e["end"] - 3, 25)
        rc, out = _run(dt.main, [path])
        self.assertEqual(rc, 0)
        self.assertIn("<- loss: 1 frame(s) lost before this row", out)
        self.assertIn(f"| {dt.leg_name(path)} | 3 | #{e['start']}-#{e['end']} ", out)
        self.assertIn("-> PASS", out)
        # A tighter horizon than the ghost's TTL fails it, and --strict says so.
        rc, out = _run(dt.main, [path, "--horizon", "20", "--strict"])
        self.assertEqual(rc, 1)
        self.assertIn("FAIL (re-opens A16)", out)

    def test_epoch_starts_from_the_replay(self) -> None:
        # The epoch-start key (frame 44) lost: its repeat-once copy starts the epoch.
        _dr, rows = dt.replay(dt.load_capture(self._capture(drops={44}))[1], "image_bw500")
        self.assertEqual([r["idx"] for r in rows if r["epoch_start"]], [0, 44])
        # The capture opens on a repeat copy (frame 0 lost): still the join.
        _dr, rows = dt.replay(dt.load_capture(self._capture(drops={0}))[1], "image_bw500")
        self.assertEqual(dt.join_row(rows), 43)

    def test_a_body_the_store_rejects_withholds_the_trace(self) -> None:
        # Frame 5 with a bad version bit (the store tests' trick): a real store rejection.
        path = self._capture()
        lines = open(path, encoding="utf-8").read().splitlines()
        row = json.loads(lines[1 + 5])
        tdf = bytes.fromhex(row["hex"])
        _tr, frames = scenario(1, n_frames=80)
        key, epoch, recs = frames[5]
        b = vs_body(key, epoch, recs)
        row["hex"] = _wire(bytes([b[0] | 0x40]) + b[1:], key, 6).hex()
        self.assertNotEqual(row["hex"], tdf.hex())
        lines[1 + 5] = json.dumps(row)
        with open(path, "w", encoding="utf-8") as fh:
            fh.write("\n".join(lines) + "\n")
        rc, out = _run(dt.main, [path])
        self.assertEqual(rc, 2, out)
        self.assertIn("CANNOT TRACE", out)
        self.assertIn("1 VECTOR body(ies) the store rejected {'bad_version': 1}", out)

    def test_second_loss_inside_the_horizon_is_not_scored(self) -> None:
        res = dt.analyse(self._capture(drops={3, 10}), None, 25, 25, 10.0)
        self.assertEqual(res["gaps"], [(3, 1), (9, 1)])
        self.assertEqual(res["episodes"][0]["status"], "not scored")

    def test_rows_mode_and_profile_rules(self) -> None:
        path = self._capture(drops={3})
        rc, out = _run(dt.main, [path, "--rows", "2:4"])
        self.assertEqual(rc, 0)
        trace = [ln for ln in out.splitlines() if ln.startswith("#  ")]
        self.assertEqual([ln[:4] for ln in trace], ["#  2", "#  3", "#  4"])
        rc, out = _run(dt.main, [path, "--profile", "image_bw250"])
        self.assertEqual(rc, 2)
        self.assertIn("capture was taken with --profile image_bw500", out)
        rc, out = _run(dt.main, [self._capture(profile=None)])
        self.assertEqual(rc, 2)
        self.assertIn("no meta profile", out)
        rc, out = _run(dt.main, [self._capture(profile=None), "--profile", "image_bw500"])
        self.assertEqual(rc, 0, out)

    def test_a_garbled_capture_withholds_the_verdict(self) -> None:
        good = self._capture(drops={3})
        bad = self._capture()
        with open(bad, "a", encoding="utf-8") as fh:
            fh.write('{"ts": 99.0, "hex": "01')                # a truncated last line
        rc, out = _run(dt.main, [good, bad])
        self.assertEqual(rc, 2)
        self.assertIn("CANNOT TRACE", out)
        self.assertIn("not JSON", out)
        self.assertIn("A16 confirmation withheld", out)
        self.assertNotIn("-> PASS", out)

    def test_mono_capture_has_nothing_to_trace(self) -> None:
        fd, path = tempfile.mkstemp(suffix="_base.jsonl")
        mono = encode_tile_delta_frame(TileDeltaFrame(
            frame_kind=FRAME_KIND_KEY, base_seq=1, grid_w=12, grid_h=8, tile_px=32,
            codec=CODEC_MONO_G4))
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            fh.write(json.dumps({"ts": 1.0, "hex": mono.hex()}) + "\n")
        self.addCleanup(os.remove, path)
        rc, out = _run(dt.main, [path, "--profile", "image_bw500"])
        self.assertEqual(rc, 1)
        self.assertIn("no VECTOR frame", out)


if __name__ == "__main__":
    unittest.main()
