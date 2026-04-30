"""Round 53 / K-E2: SIL coverage for ramp-state heatmap data model.

Pins the public state vocabulary, classification precedence,
threshold validation, JSON-serializable shape, and identity behaviour
of [ramp_heatmap.py](../ramp_heatmap.py).

Test classes:

* ``RH_A_StateVocabulary`` \u2014 the public ``STATES`` tuple is
  exactly the five names the web UI painter expects.
* ``RH_B_LagFunction`` \u2014 ``lag()`` is signed ``raw - effective``
  with int coercion.
* ``RH_C_ClassifyState`` \u2014 each state is reachable via at least
  one canonical input pair, precedence is correct, and threshold
  validation rejects negatives.
* ``RH_D_RowShape`` \u2014 :func:`build_heatmap_row` produces the
  five-key dict the painter needs.
* ``RH_E_BuildHeatmap`` \u2014 :func:`build_heatmap` preserves stream
  order, applies thresholds uniformly, and emits the documented
  top-level keys.
* ``RH_F_FirmwareIdentity`` \u2014 ``DEFAULT_DEADBAND`` mirrors the
  BC-29 ``ui.axis_deadband`` default.

Pure stdlib.
"""

from __future__ import annotations

import pathlib
import sys
import unittest

_THIS = pathlib.Path(__file__).resolve()
_BS = _THIS.parents[1]
sys.path.insert(0, str(_BS))

import build_config  # noqa: E402
import ramp_heatmap  # noqa: E402
from ramp_heatmap import (  # noqa: E402
    DEFAULT_DEADBAND,
    DEFAULT_MUSHY_THRESHOLD,
    HeatmapSample,
    STATES,
    build_heatmap,
    build_heatmap_row,
    classify_state,
    lag,
)


class RH_A_StateVocabulary(unittest.TestCase):

    def test_states_tuple_is_pinned(self):
        self.assertEqual(STATES,
                         ("idle", "matched", "reversal", "decay", "mushy"))

    def test_states_is_immutable_tuple(self):
        self.assertIsInstance(STATES, tuple)


class RH_B_LagFunction(unittest.TestCase):

    def test_zero_zero_yields_zero(self):
        self.assertEqual(lag(0, 0), 0)

    def test_positive_pursuit_lag(self):
        self.assertEqual(lag(100, 40), 60)

    def test_negative_release_lag(self):
        # Operator released (raw=0) but effective still asserting.
        self.assertEqual(lag(0, 80), -80)

    def test_sign_mismatch(self):
        self.assertEqual(lag(50, -50), 100)
        self.assertEqual(lag(-50, 50), -100)

    def test_int_coercion(self):
        # Defensive: callers may pass numpy ints, etc.
        self.assertEqual(lag(True, False), 1)


class RH_C_ClassifyState(unittest.TestCase):

    def test_idle_when_both_within_deadband(self):
        for raw, eff in ((0, 0), (5, -3), (13, 13), (-13, 12)):
            with self.subTest(raw=raw, eff=eff):
                self.assertEqual(classify_state(raw, eff), "idle")

    def test_matched_when_steady_pass_through(self):
        # Same value, well above deadband \u2192 lag is zero.
        self.assertEqual(classify_state(100, 100), "matched")
        self.assertEqual(classify_state(-100, -100), "matched")
        # Small same-sign lag below mushy_threshold (= 40) but both
        # active.
        self.assertEqual(classify_state(100, 80), "matched")

    def test_reversal_when_opposite_signs_both_active(self):
        self.assertEqual(classify_state(100, -100), "reversal")
        self.assertEqual(classify_state(-80, 60), "reversal")

    def test_reversal_takes_precedence_over_mushy(self):
        # |100 - (-100)| = 200 > 40 (mushy threshold), but reversal
        # wins because reversal-brake lag is *expected*.
        self.assertEqual(classify_state(100, -100), "reversal")

    def test_decay_when_raw_idle_but_effective_active(self):
        # Operator released; release ramp in flight.
        self.assertEqual(classify_state(0, 80), "decay")
        self.assertEqual(classify_state(5, -100), "decay")  # raw within deadband

    def test_mushy_when_same_sign_lag_exceeds_threshold(self):
        # 127 - 80 = 47 > 40 \u2192 mushy.
        self.assertEqual(classify_state(127, 80), "mushy")
        # Negative direction.
        self.assertEqual(classify_state(-127, -80), "mushy")

    def test_mushy_boundary_inclusive_below_exclusive_above(self):
        # |raw - effective| == mushy_threshold (40) is NOT mushy.
        self.assertEqual(classify_state(100, 60), "matched")
        # |raw - effective| == 41 IS mushy.
        self.assertEqual(classify_state(100, 59), "mushy")

    def test_custom_thresholds_widen_mushy_band(self):
        # With mushy_threshold=80, the 47-delta case becomes matched.
        self.assertEqual(
            classify_state(127, 80, mushy_threshold=80), "matched")

    def test_custom_deadband_extends_idle_zone(self):
        # With deadband=30, raw=20/eff=10 is now idle (was matched).
        self.assertEqual(classify_state(20, 10, deadband=30), "idle")

    def test_negative_deadband_rejected(self):
        with self.assertRaises(ValueError):
            classify_state(0, 0, deadband=-1)

    def test_negative_mushy_threshold_rejected(self):
        with self.assertRaises(ValueError):
            classify_state(0, 0, mushy_threshold=-1)


class RH_D_RowShape(unittest.TestCase):

    def test_row_keys(self):
        sample = HeatmapSample(now_ms=1000, raw=100, effective=80)
        row = build_heatmap_row(sample)
        self.assertEqual(set(row.keys()),
                         {"now_ms", "raw", "effective", "lag", "state"})

    def test_row_values_match_inputs(self):
        sample = HeatmapSample(now_ms=12345, raw=100, effective=80)
        row = build_heatmap_row(sample)
        self.assertEqual(row["now_ms"], 12345)
        self.assertEqual(row["raw"], 100)
        self.assertEqual(row["effective"], 80)
        self.assertEqual(row["lag"], 20)
        self.assertEqual(row["state"], "matched")

    def test_row_state_uses_supplied_thresholds(self):
        sample = HeatmapSample(now_ms=0, raw=127, effective=80)
        # default mushy_threshold=40 \u2192 mushy
        self.assertEqual(build_heatmap_row(sample)["state"], "mushy")
        # widened \u2192 matched
        widened = build_heatmap_row(sample, mushy_threshold=80)
        self.assertEqual(widened["state"], "matched")

    def test_row_values_are_ints_not_dataclass_internals(self):
        sample = HeatmapSample(now_ms=1, raw=2, effective=3)
        row = build_heatmap_row(sample)
        for key in ("now_ms", "raw", "effective", "lag"):
            with self.subTest(key=key):
                self.assertIsInstance(row[key], int)
        self.assertIsInstance(row["state"], str)


class RH_E_BuildHeatmap(unittest.TestCase):

    def test_top_level_shape(self):
        out = build_heatmap({})
        self.assertEqual(set(out.keys()),
                         {"deadband", "mushy_threshold", "axes"})
        self.assertEqual(out["deadband"], DEFAULT_DEADBAND)
        self.assertEqual(out["mushy_threshold"], DEFAULT_MUSHY_THRESHOLD)
        self.assertEqual(out["axes"], {})

    def test_axis_keys_preserved(self):
        streams = {
            "left_track": [HeatmapSample(0, 0, 0)],
            "right_track": [HeatmapSample(0, 0, 0)],
            "arms": [HeatmapSample(0, 0, 0)],
            "bucket": [HeatmapSample(0, 0, 0)],
        }
        out = build_heatmap(streams)
        self.assertEqual(set(out["axes"].keys()),
                         {"left_track", "right_track", "arms", "bucket"})

    def test_stream_order_preserved(self):
        samples = [
            HeatmapSample(0, 100, 100),
            HeatmapSample(50, 0, 80),    # decay
            HeatmapSample(100, 0, 40),   # decay
            HeatmapSample(150, 0, 0),    # idle
        ]
        out = build_heatmap({"arms": samples})
        rows = out["axes"]["arms"]
        self.assertEqual([r["now_ms"] for r in rows], [0, 50, 100, 150])
        self.assertEqual([r["state"] for r in rows],
                         ["matched", "decay", "decay", "idle"])

    def test_threshold_overrides_propagate(self):
        sample = HeatmapSample(0, 127, 80)
        out = build_heatmap({"arms": [sample]}, mushy_threshold=80)
        self.assertEqual(out["mushy_threshold"], 80)
        self.assertEqual(out["axes"]["arms"][0]["state"], "matched")

    def test_mushy_lag_classifies_canonically(self):
        # A short pursuit ramp where the operator floors the stick
        # while the firmware is still ramping up: 127 raw, effective
        # climbing 0 \u2192 40 \u2192 80 \u2192 127.
        samples = [
            HeatmapSample(0, 127, 0),     # mushy (lag=127 > 40)
            HeatmapSample(50, 127, 40),   # mushy (lag=87 > 40)
            HeatmapSample(100, 127, 80),  # mushy (lag=47 > 40)
            HeatmapSample(150, 127, 127),  # matched (lag=0)
        ]
        out = build_heatmap({"left_track": samples})
        states = [r["state"] for r in out["axes"]["left_track"]]
        self.assertEqual(states, ["mushy", "mushy", "mushy", "matched"])


class RH_F_FirmwareIdentity(unittest.TestCase):

    def test_default_deadband_mirrors_ui_axis_deadband(self):
        cfg = build_config.load()
        self.assertEqual(DEFAULT_DEADBAND, cfg.ui.axis_deadband)

    def test_default_mushy_threshold_is_documented_starting_point(self):
        # Pin the starting value so a regression that silently moves
        # the threshold trips here. Documented as ~31% of int8 full
        # scale in the module docstring.
        self.assertEqual(DEFAULT_MUSHY_THRESHOLD, 40)


if __name__ == "__main__":
    unittest.main()
