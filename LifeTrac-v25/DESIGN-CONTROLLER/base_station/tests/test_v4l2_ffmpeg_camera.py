"""W2-02: V4l2FfmpegCamera unit tests.

Pins the bench-validated ffmpeg invocation from 2026-05-15 (Kurokesu C2 on
``/dev/video1``, MJPEG 1920x1080@30fps, scaled to 384x256 rgb24) and the
``_make_camera()`` selector behavior. Does NOT spawn ffmpeg \u2014 a fake
``Popen`` provides exactly one canvas-sized frame so the test runs on any
host (CI, Windows dev box, the X8 itself).
"""

from __future__ import annotations

import io
import os
import sys
import unittest
from unittest import mock


_X8_DIR = os.path.normpath(os.path.join(
    os.path.dirname(__file__), "..", "..", "firmware", "tractor_x8"))
if _X8_DIR not in sys.path:
    sys.path.insert(0, _X8_DIR)

import camera_service  # noqa: E402  (sys.path tweak above)
from camera_service import (  # noqa: E402
    CANVAS_H,
    CANVAS_W,
    V4l2FfmpegCamera,
    build_ffmpeg_argv,
)


class BuildFfmpegArgvTests(unittest.TestCase):

    def test_argv_matches_bench_validated_invocation(self) -> None:
        argv = build_ffmpeg_argv(
            device="/dev/video1",
            input_format="mjpeg",
            input_size="1920x1080",
            input_fps=30,
            ffmpeg_path="/tmp/ffmpeg",
        )
        self.assertEqual(argv[0], "/tmp/ffmpeg")
        # The exact token sequence the bench session ran.
        for token in (
            "-f", "v4l2",
            "-input_format", "mjpeg",
            "-video_size", "1920x1080",
            "-framerate", "30",
            "-i", "/dev/video1",
            "-pix_fmt", "rgb24",
            "-f", "rawvideo",
        ):
            self.assertIn(token, argv, f"missing token {token!r} in argv {argv}")
        # Scale filter must request the canvas dims (384x256).
        self.assertIn(f"scale={CANVAS_W}:{CANVAS_H}", argv)
        # Output sink is stdout.
        self.assertEqual(argv[-1], "-")

    def test_argv_defaults_pull_from_module_env(self) -> None:
        argv = build_ffmpeg_argv()
        self.assertIn(camera_service.V4L2_DEVICE, argv)
        self.assertIn(camera_service.V4L2_INPUT_FORMAT, argv)
        self.assertIn(camera_service.V4L2_INPUT_SIZE, argv)


class _FakeProc:
    """Minimal subprocess.Popen stand-in.

    Serves a single canvas-sized RGB frame, then returns b'' (EOF) on
    subsequent reads so the camera object's restart path is exercised.
    """

    def __init__(self, frame_bytes: int) -> None:
        self.pid = 12345
        self.stdout = io.BufferedReader(io.BytesIO(b"\xab" * frame_bytes), buffer_size=4096)
        self.stderr = io.BytesIO(b"")
        self._terminated = False
        self._returncode = None

    def poll(self):  # noqa: D401 - matches subprocess.Popen API
        return self._returncode

    def terminate(self) -> None:
        self._terminated = True
        self._returncode = 0

    def wait(self, timeout: float | None = None) -> int:  # noqa: ARG002
        self._returncode = 0
        return 0

    def kill(self) -> None:
        self._returncode = -9


class V4l2FfmpegCameraTests(unittest.TestCase):

    def test_grab_rgb_returns_canvas_sized_frame(self) -> None:
        frame_bytes = CANVAS_W * CANVAS_H * 3
        spawned: list[list[str]] = []

        def fake_popen(argv, **_kwargs):
            spawned.append(list(argv))
            return _FakeProc(frame_bytes=frame_bytes)

        with mock.patch("subprocess.Popen", side_effect=fake_popen):
            cam = V4l2FfmpegCamera()
            buf = cam.grab_rgb()
            cam.close()

        self.assertEqual(len(buf), frame_bytes)
        self.assertEqual(len(spawned), 1)
        self.assertIn("-f", spawned[0])
        self.assertIn("v4l2", spawned[0])

    def test_grab_rgb_restarts_ffmpeg_on_eof(self) -> None:
        frame_bytes = CANVAS_W * CANVAS_H * 3
        spawn_count = {"n": 0}

        def fake_popen(_argv, **_kwargs):
            spawn_count["n"] += 1
            return _FakeProc(frame_bytes=frame_bytes)

        with mock.patch("subprocess.Popen", side_effect=fake_popen):
            cam = V4l2FfmpegCamera()
            self.assertEqual(len(cam.grab_rgb()), frame_bytes)
            # Second call: previous _FakeProc.stdout is now exhausted; the
            # camera must restart ffmpeg and return another full frame.
            self.assertEqual(len(cam.grab_rgb()), frame_bytes)
            cam.close()

        self.assertGreaterEqual(spawn_count["n"], 2)

    def test_constructor_raises_when_ffmpeg_missing(self) -> None:
        with mock.patch("subprocess.Popen",
                        side_effect=FileNotFoundError("no ffmpeg")):
            with self.assertRaises(RuntimeError):
                V4l2FfmpegCamera(ffmpeg_path="/nonexistent/ffmpeg")


class MakeCameraSelectorTests(unittest.TestCase):

    def setUp(self) -> None:
        self._saved_source = camera_service.SOURCE

    def tearDown(self) -> None:
        camera_service.SOURCE = self._saved_source

    def test_synthetic_source_returns_synthetic_camera(self) -> None:
        camera_service.SOURCE = "synthetic"
        cam = camera_service._make_camera()
        self.assertIsInstance(cam, camera_service.SyntheticCamera)

    def test_v4l2_source_returns_v4l2_ffmpeg_camera(self) -> None:
        camera_service.SOURCE = "v4l2"
        frame_bytes = CANVAS_W * CANVAS_H * 3
        with mock.patch("subprocess.Popen",
                        return_value=_FakeProc(frame_bytes=frame_bytes)):
            cam = camera_service._make_camera()
            try:
                self.assertIsInstance(cam, V4l2FfmpegCamera)
            finally:
                cam.close()

    def test_v4l2_falls_back_to_synthetic_when_ffmpeg_missing(self) -> None:
        camera_service.SOURCE = "ffmpeg"
        with mock.patch("subprocess.Popen",
                        side_effect=FileNotFoundError("no ffmpeg")):
            cam = camera_service._make_camera()
        self.assertIsInstance(cam, camera_service.SyntheticCamera)


if __name__ == "__main__":
    unittest.main()
