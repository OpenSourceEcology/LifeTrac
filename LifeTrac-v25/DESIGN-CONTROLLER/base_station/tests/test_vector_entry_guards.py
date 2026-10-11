"""RS-13 guards (2026-10-10): camera_service around VECTOR entry.

Three findings of the 2026-10-04 reviews, each verified against the 2d_r4
leg (bench-evidence/radio_monitor_20261004_110423_4ab58b8d):

(b) Cold-start budget. In bridge mode the boot budget is the 12-fragment
    LIFETRAC_FRAGMENT_BUDGET_BRIDGE_DEFAULT (``byte_budget=2436`` in the log)
    until image_tx_daemon's retained ``tractor/link_budget`` arrives. VECTOR is
    a one-fragment codec, so until then its budget is one fragment of the
    active profile (LIFETRAC_REG_PROFILE) or of the most conservative image
    profile.
(c) Lazy encoder build. The first VECTOR frame imported OpenCV and the
    encoder and paid the process's first-call costs (l0 79.6 ms vs ~15 ms):
    a 1.038 s TX gap at the switch, past the strict < 1000 ms FHSS authority
    streak. main() now builds (and warms) the encoder before the first capture.
(d) Double epoch start. Every VECTOR entry logged ``trigger=first`` then
    ``trigger=forced``: the loop sampled the keyframe request BEFORE the
    blocking capture and the mode AFTER it, so a switch landing in between
    was built in the new mode without its force and forced again next frame
    (and a RE-entry's first frame was a delta against the stale scene).
    The request is now consumed after the capture, atomically with the mode.
"""

from __future__ import annotations

import os
import sys
import threading
import unittest
from unittest import mock

_HERE = os.path.dirname(os.path.abspath(__file__))
_X8_DIR = os.path.normpath(os.path.join(_HERE, "..", "..", "firmware", "tractor_x8"))
_BS_DIR = os.path.normpath(os.path.join(_HERE, ".."))
for _p in (_BS_DIR, _X8_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import camera_service as cs  # noqa: E402
from x8_image_pipeline.fragment import max_payload_for_n_fragments  # noqa: E402
from lora_proto import PHY_BY_NAME  # noqa: E402

HAVE_VECTOR = cs._vector_encoder_available()
ONE_FRAG_BW500 = max_payload_for_n_fragments(1, profile=PHY_BY_NAME["image_bw500"])
ONE_FRAG_BW250 = max_payload_for_n_fragments(1, profile=PHY_BY_NAME["image_bw250"])

if HAVE_VECTOR:
    import numpy as np


def _scene(n_blocks: int = 12, seed: int = 3, shift: int = 0) -> bytes:
    """A synthetic camera canvas (sky, ground, coloured blocks) as RGB bytes."""
    rng = np.random.default_rng(seed)
    img = np.zeros((cs.CANVAS_H, cs.CANVAS_W, 3), np.uint8)
    img[:96] = (120, 160, 220)
    img[96:] = (60, 120, 40)
    for _ in range(n_blocks):
        x, y = int(rng.integers(0, cs.CANVAS_W - 32)), int(rng.integers(100, cs.CANVAS_H - 26))
        w, h = int(rng.integers(12, 40)), int(rng.integers(10, 30))
        img[y:y + h, x:x + w] = rng.integers(0, 255, 3)
    return np.roll(img, shift, axis=1).tobytes()


class _Cam:
    """grab_rgb() returns a moving scene; ``on_grab`` runs INSIDE the grab,
    i.e. while the encode loop is blocked on the capture — where the 2d_r4
    ENCODE_MODE command landed."""

    def __init__(self, n_blocks: int = 12):
        self.n_blocks = n_blocks
        self.k = 0
        self.on_grab = None

    def grab_rgb(self) -> bytes:
        self.k += 1
        hook, self.on_grab = self.on_grab, None
        if hook is not None:
            hook()
        return _scene(self.n_blocks, shift=3 * self.k)


def _fake_tile(rgb_canvas, tx, ty, quality=None, encode_mode=None, is_key=False):
    """_encode_tile stand-in (no PIL): a small fixed blob."""
    return bytes([tx & 0xFF, ty & 0xFF]) + b"\xAA" * 6


class _CameraState(unittest.TestCase):
    """Saves/restores the camera_service globals these tests mutate."""

    def setUp(self):
        self._saved = (cs.ENCODE_MODE, cs.VECTOR_DETAIL, cs.WEBP_QUALITY,
                       cs._VECTOR_ENCODER, cs._VECTOR_SEQ, cs._MQTT_CLIENT)
        cs._MQTT_CLIENT = None            # no ack publish
        cs._VECTOR_ENCODER = None
        cs._VECTOR_SEQ = 0
        cs.VECTOR_DETAIL = 80

    def tearDown(self):
        (cs.ENCODE_MODE, cs.VECTOR_DETAIL, cs.WEBP_QUALITY,
         cs._VECTOR_ENCODER, cs._VECTOR_SEQ, cs._MQTT_CLIENT) = self._saved


# ---------------------------------------------------------------- (b)

class VectorColdStartBudget(_CameraState):

    def _env(self, profile):
        env = {k: v for k, v in os.environ.items() if k != "LIFETRAC_REG_PROFILE"}
        if profile is not None:
            env["LIFETRAC_REG_PROFILE"] = str(profile)
        return mock.patch.dict(os.environ, env, clear=True)

    def test_bridge_cold_start_default_is_twelve_fragments(self):
        # The value seen on air ("byte_budget=2436"): what VECTOR must not use.
        env = {k: v for k, v in os.environ.items()
               if k not in ("LIFETRAC_FRAGMENT_BUDGET", "LIFETRAC_FRAGMENT_BUDGET_BRIDGE_DEFAULT",
                            "LIFETRAC_FRAGMENT_PROFILE")}
        with mock.patch.dict(os.environ, env, clear=True), \
                mock.patch.object(cs, "USE_LORA_BRIDGE", True):
            self.assertEqual(cs._resolve_byte_budget(), 2436)

    def test_unknown_link_budget_clamps_to_one_fragment_of_active_profile(self):
        lb = cs.LinkBudget(bytes_=2436, n_fragments=None, profile_name="image")
        with self._env(2):
            self.assertEqual(cs._vector_byte_budget(lb), ONE_FRAG_BW500)
        with self._env(1):
            self.assertEqual(cs._vector_byte_budget(lb), ONE_FRAG_BW250)
        with self._env(0):
            self.assertEqual(cs._vector_byte_budget(lb), ONE_FRAG_BW250)

    def test_unknown_profile_uses_most_conservative_fragment(self):
        lb = cs.LinkBudget(bytes_=2436)
        for profile in (None, "x", 7):
            with self._env(profile):
                self.assertEqual(cs._vector_byte_budget(lb),
                                 min(ONE_FRAG_BW250, ONE_FRAG_BW500), f"profile={profile!r}")

    def test_no_budget_at_all_is_one_fragment(self):
        with self._env(2):
            self.assertEqual(cs._vector_byte_budget(None), ONE_FRAG_BW500)
            self.assertEqual(cs._vector_byte_budget(cs.LinkBudget()), ONE_FRAG_BW500)

    def test_smaller_provisional_budget_is_kept(self):
        with self._env(2):
            self.assertEqual(cs._vector_byte_budget(cs.LinkBudget(bytes_=120)), 120)

    def test_known_link_budget_is_authoritative(self):
        # Once the radio owner speaks it wins, even over a stale env profile,
        # and the clamp no longer applies (an explicit n_fragments=2 stands).
        lb = cs.LinkBudget(bytes_=2436)
        self.assertTrue(lb.update(1, 6))                       # image_bw500
        with self._env(0):
            self.assertEqual(cs._vector_byte_budget(lb), ONE_FRAG_BW500)
        self.assertTrue(lb.update(2, 6))
        with self._env(0):
            self.assertEqual(cs._vector_byte_budget(lb),
                             max_payload_for_n_fragments(2, profile=PHY_BY_NAME["image_bw500"]))

    def test_tile_modes_keep_the_provisional_budget(self):
        # _build_frame only routes vector_byte_budget into VECTOR frames.
        cs.ENCODE_MODE = cs.ENCODE_MODE_Y_ONLY
        with mock.patch.object(cs, "_encode_tile", _fake_tile), \
                mock.patch.object(cs, "_build_vector_frame", side_effect=AssertionError):
            payload = cs._build_frame(cs.SyntheticCamera(), cs.FrameAccum(), True,
                                      byte_budget=2436, vector_byte_budget=243)
        self.assertGreater(len(payload), 243)

    @unittest.skipUnless(HAVE_VECTOR, "numpy + OpenCV needed for the VS1 encoder")
    def test_cold_start_vector_frame_fits_one_fragment(self):
        cs.ENCODE_MODE = cs.ENCODE_MODE_VECTOR
        lb = cs.LinkBudget(bytes_=2436)          # bridge cold start, no link_budget yet
        # Premise: a busy scene at the 12-fragment budget spills past one fragment.
        unclamped = cs._build_frame(_Cam(n_blocks=80), cs.FrameAccum(), True, byte_budget=lb.bytes)
        self.assertGreater(len(unclamped), ONE_FRAG_BW500)
        cs._VECTOR_ENCODER = None
        with self._env(2):
            budget = cs._vector_byte_budget(lb)
        clamped = cs._build_frame(_Cam(n_blocks=80), cs.FrameAccum(), True,
                                  byte_budget=lb.bytes, vector_byte_budget=budget)
        self.assertLessEqual(len(clamped), ONE_FRAG_BW500)
        self.assertEqual(clamped[5], cs.CODEC_VECTOR)


# ---------------------------------------------------------------- (c)

class _StopMain(BaseException):
    """Escapes main()'s `except Exception` frame guard at the first capture."""


@unittest.skipUnless(HAVE_VECTOR, "numpy + OpenCV needed for the VS1 encoder")
class VectorEncoderPrebuild(_CameraState):

    def test_prebuild_leaves_a_fresh_encoder(self):
        self.assertTrue(cs._prebuild_vector_encoder())
        enc = cs._VECTOR_ENCODER
        self.assertIsNotNone(enc)
        self.assertEqual(enc.last_stats, {}, "warm-up must run on a throwaway encoder")
        self.assertTrue(cs._prebuild_vector_encoder())         # idempotent
        self.assertIs(cs._VECTOR_ENCODER, enc)

    def test_first_vector_frame_reuses_the_prebuilt_encoder(self):
        cs._prebuild_vector_encoder()
        enc = cs._VECTOR_ENCODER
        import x8_image_pipeline.encode_vector as ev
        with mock.patch.object(ev, "VectorEncoder", side_effect=AssertionError("built lazily")):
            frame = cs._build_vector_frame(_scene(), True, 243)
        self.assertIs(cs._VECTOR_ENCODER, enc)
        self.assertEqual((frame[0], frame[5]), (1, cs.CODEC_VECTOR))
        self.assertEqual(enc.last_stats["epochs"], 1)
        self.assertEqual(enc.last_stats["last_epoch_trigger"], "first")

    def test_prebuild_skipped_without_the_encoder_stack(self):
        with mock.patch.object(cs, "_vector_encoder_available", return_value=False):
            self.assertFalse(cs._prebuild_vector_encoder())
        self.assertIsNone(cs._VECTOR_ENCODER)

    def _run_main_to_first_capture(self, prebuild: bool):
        seen = {}

        class _Cam1:
            def grab_rgb(self_inner):
                seen["encoder"] = cs._VECTOR_ENCODER
                raise _StopMain

        with mock.patch.object(cs, "_make_camera", return_value=_Cam1()), \
                mock.patch.object(cs, "USE_LORA_BRIDGE", True), \
                mock.patch.object(cs, "DEBUG_MQTT", False), \
                mock.patch.object(cs, "VECTOR_PREBUILD", prebuild):
            # Boot in a tile mode and switch to VECTOR later, as on 2d_r4.
            cs.ENCODE_MODE = cs.ENCODE_MODE_MONO_G4
            with self.assertRaises(_StopMain):
                cs.main()
        return seen

    def test_main_builds_the_encoder_before_the_first_capture(self):
        seen = self._run_main_to_first_capture(prebuild=True)
        self.assertIsNotNone(seen.get("encoder"),
                             "VectorEncoder must exist before the first capture")

    def test_prebuild_knob_off_restores_lazy_build(self):
        seen = self._run_main_to_first_capture(prebuild=False)
        self.assertIsNone(seen.get("encoder"))


# ---------------------------------------------------------------- (d)

@unittest.skipUnless(HAVE_VECTOR, "numpy + OpenCV needed for the VS1 encoder")
class VectorEntrySingleEpochStart(_CameraState):

    def setUp(self):
        super().setUp()
        self.evt = threading.Event()
        self.cam = _Cam()
        self.accum = cs.FrameAccum()
        p = mock.patch.object(cs, "_encode_tile", _fake_tile)
        p.start()
        self.addCleanup(p.stop)

    def frame(self) -> bytes:
        """One encode-loop iteration exactly as main() runs it."""
        return cs._build_frame(self.cam, self.accum, False, force_evt=self.evt,
                               byte_budget=243, vector_byte_budget=243)

    def switch_during_capture(self, mode: int) -> None:
        self.cam.on_grab = lambda: cs._apply_encode_mode(mode, "lora_cmd", self.evt, quality=80)

    def stats(self) -> tuple:
        st = cs._VECTOR_ENCODER.last_stats
        return st["epochs"], st["last_epoch_trigger"]

    def test_switch_landing_in_the_capture_starts_one_epoch(self):
        cs.ENCODE_MODE = cs.ENCODE_MODE_MONO_G4
        self.assertEqual(self.frame()[5], cs.CODEC_MONO_G4)
        self.switch_during_capture(cs.ENCODE_MODE_VECTOR)
        entry = self.frame()
        self.assertEqual((entry[0], entry[5]), (1, cs.CODEC_VECTOR))
        self.assertTrue(self.accum.last_forced)
        self.assertFalse(self.evt.is_set(), "the switch's keyframe request was consumed")
        self.assertEqual(self.stats(), (1, "first"))
        for _ in range(3):
            f = self.frame()
            self.assertEqual((f[0], f[5]), (0, cs.CODEC_VECTOR))
            self.assertFalse(self.accum.last_forced)
            self.assertEqual(self.stats(), (1, "first"), "redundant epoch start after entry")

    def test_reentry_forces_exactly_one_new_epoch(self):
        cs.ENCODE_MODE = cs.ENCODE_MODE_VECTOR
        self.evt.set()                                   # boot keyframe
        for _ in range(3):
            self.frame()
        self.assertEqual(self.stats(), (1, "first"))
        self.switch_during_capture(cs.ENCODE_MODE_MONO_G4)
        leave = self.frame()
        self.assertEqual((leave[0], leave[5]), (1, cs.CODEC_MONO_G4))   # forced repaint
        self.frame()
        self.switch_during_capture(cs.ENCODE_MODE_VECTOR)
        entry = self.frame()
        # Before the fix this frame was a delta against the scene the encoder
        # mirrored minutes ago, and the start came one frame later.
        self.assertEqual((entry[0], entry[5]), (1, cs.CODEC_VECTOR))
        self.assertEqual(self.stats(), (2, "forced"))
        for _ in range(2):
            self.assertEqual(self.frame()[0], 0)
            self.assertEqual(self.stats(), (2, "forced"))

    def test_pre_fix_sampling_order_double_starts(self):
        # The 2d_r4 mechanism, reproduced through the legacy direct-call path
        # (force sampled by the caller BEFORE the blocking capture).
        cs.ENCODE_MODE = cs.ENCODE_MODE_MONO_G4
        self.frame()
        self.switch_during_capture(cs.ENCODE_MODE_VECTOR)
        triggers = []
        for _ in range(3):
            force = self.evt.is_set()
            self.evt.clear()
            cs._build_frame(self.cam, self.accum, force, byte_budget=243)
            triggers.append(self.stats())
        self.assertEqual(triggers, [(1, "first"), (2, "forced"), (2, "forced")])

    def test_switch_mid_tile_build_never_stamps_codec_6_on_tiles(self):
        cs.ENCODE_MODE = cs.ENCODE_MODE_MONO_G4
        self.frame()
        fired = []

        def _switching_tile(rgb, tx, ty, quality=None, encode_mode=None, is_key=False):
            if not fired:
                fired.append(1)
                cs._apply_encode_mode(cs.ENCODE_MODE_VECTOR, "lora_cmd", self.evt, quality=80)
            self.assertEqual(encode_mode, cs.ENCODE_MODE_MONO_G4)
            return _fake_tile(rgb, tx, ty)

        self.evt.set()                                   # make the frame encode tiles
        with mock.patch.object(cs, "_encode_tile", _switching_tile):
            tiles = self.frame()
        self.assertTrue(fired)
        self.assertEqual(tiles[5], cs.CODEC_MONO_G4, "tile payload stamped with the new codec")
        entry = self.frame()
        self.assertEqual((entry[0], entry[5]), (1, cs.CODEC_VECTOR))
        self.assertEqual(self.stats(), (1, "first"))


class ModeCommitIsAtomic(_CameraState):
    """Every keyframe a mode command raises is raised under _MODE_LOCK and
    after the new mode is committed, so the sampler sees both or neither."""

    class _ProbeEvent(threading.Event):
        def __init__(self, expect_mode):
            super().__init__()
            self.expect_mode = expect_mode
            self.calls = []

        def set(self):
            got = []
            t = threading.Thread(target=lambda: got.append(cs._MODE_LOCK.acquire(blocking=False)))
            t.start()
            t.join()
            if got[0]:
                cs._MODE_LOCK.release()
            self.calls.append((not got[0], cs.ENCODE_MODE == self.expect_mode))
            super().set()

    def test_apply_encode_mode(self):
        cs.ENCODE_MODE = cs.ENCODE_MODE_MONO_G4
        evt = self._ProbeEvent(cs.ENCODE_MODE_Y_ONLY)
        cs._apply_encode_mode(cs.ENCODE_MODE_Y_ONLY, "lora_cmd", evt)
        self.assertEqual(evt.calls, [(True, True)])

    def test_back_channel_encode_mode(self):
        cs.ENCODE_MODE = cs.ENCODE_MODE_MONO_G4
        evt = self._ProbeEvent(cs.ENCODE_MODE_Y_ONLY)
        cs.dispatch_back_channel(bytes([cs.X8_CMD_TOPIC, cs.CMD_ENCODE_MODE,
                                        cs.ENCODE_MODE_Y_ONLY]), evt)
        self.assertTrue(evt.calls)
        self.assertTrue(all(c == (True, True) for c in evt.calls), evt.calls)

    def test_sampler_consumes_request_with_the_mode(self):
        evt = threading.Event()
        cs.ENCODE_MODE = cs.ENCODE_MODE_Y_ONLY
        self.assertEqual(cs._sample_mode_and_force(evt), (cs.ENCODE_MODE_Y_ONLY, False))
        evt.set()
        self.assertEqual(cs._sample_mode_and_force(evt), (cs.ENCODE_MODE_Y_ONLY, True))
        self.assertFalse(evt.is_set())
        self.assertEqual(cs._sample_mode_and_force(None, True), (cs.ENCODE_MODE_Y_ONLY, True))


if __name__ == "__main__":
    unittest.main()
