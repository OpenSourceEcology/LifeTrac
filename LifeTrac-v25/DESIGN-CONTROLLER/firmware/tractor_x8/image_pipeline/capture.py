"""V4L2 capture ring buffer for the tractor X8 image pipeline.

The X8 normally pulls 384×256 YCbCr 4:2:0 frames at 10 fps off the front
camera through V4L2. To keep ``camera_service.py`` testable without a real
sensor we expose two backends:

- :class:`V4l2CaptureBackend` — the production path; opens ``/dev/videoN``
  with the ``v4l2`` Python module if it is importable. Otherwise raises at
  ``open()`` so the caller can fall back to mock.
- :class:`MockCaptureBackend` — pluggable frame source for unit tests +
  ``camera_service.py --replay``. Accepts an iterable of ``(y, cb, cr)``
  tuples *or* a callable that returns the next ``Frame``.

Both backends present the same :class:`CaptureRing` API:

    ring = CaptureRing(backend, depth=4)
    ring.start()
    frame = ring.latest()                # newest frame, or None
    frame = ring.next(timeout_s=0.2)     # blocks until the next fresh frame
    ring.stop()

The ring stores at most ``depth`` frames; producer overwrites the oldest
slot when the consumer is slow (newest-wins, no back-pressure on the
camera driver).

Public dataclass :class:`Frame` carries:
- ``ts_monotonic_ms`` (int)         — capture timestamp
- ``seq`` (int)                     — monotonic 0..4 294 967 295 wrap
- ``width`` / ``height`` (int)
- ``y`` (memoryview / numpy / bytes) — luma plane row-major
- ``cb`` / ``cr`` (optional)         — chroma planes (4:2:0 dims)

Only the Y plane is required by ``register.py`` / ``tile_diff.py`` /
``encode_motion.py``; chroma is optional and may be ``None`` for the
mock path.
"""
from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Callable, Iterable, Iterator, Optional, Sequence

try:                                             # pragma: no cover
    import numpy as _np                          # type: ignore
    _HAVE_NUMPY = True
except ImportError:                              # pragma: no cover
    _np = None                                    # type: ignore
    _HAVE_NUMPY = False

DEFAULT_WIDTH = 384
DEFAULT_HEIGHT = 256
DEFAULT_FPS = 10


@dataclass
class Frame:
    ts_monotonic_ms: int
    seq: int
    width: int
    height: int
    y: Any                          # row-major luma; numpy.ndarray (H, W) preferred
    cb: Any = None                  # optional chroma U
    cr: Any = None                  # optional chroma V


# ---------------------------------------------------------------------------
# Backends
# ---------------------------------------------------------------------------

class CaptureBackend:
    """Producer interface used by :class:`CaptureRing`."""

    width: int
    height: int

    def open(self) -> None:        # pragma: no cover (interface)
        raise NotImplementedError

    def read(self) -> Optional[Frame]:    # pragma: no cover
        raise NotImplementedError

    def close(self) -> None:       # pragma: no cover
        raise NotImplementedError


class MockCaptureBackend(CaptureBackend):
    """Test/replay backend: produces frames from an iterable or factory.

    ``source`` may be:
      - an iterable of ``Frame`` (full frames, taken as-is),
      - an iterable of ``ndarray`` / bytes / memoryview luma planes
        (a Frame is constructed for each),
      - a zero-arg callable that returns the next luma plane / Frame /
        None when the stream is exhausted.
    """

    def __init__(self,
                 source: Iterable[Any] | Callable[[], Any],
                 *,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT,
                 fps: float = DEFAULT_FPS):
        self.width = width
        self.height = height
        self._period_s = 1.0 / max(0.001, fps)
        self._iter: Optional[Iterator[Any]] = None
        self._factory: Optional[Callable[[], Any]] = None
        if callable(source):
            self._factory = source
        else:
            self._iter = iter(source)
        self._seq = 0
        self._next_due = 0.0
        self._opened = False

    def open(self) -> None:
        self._next_due = time.monotonic()
        self._opened = True

    def close(self) -> None:
        self._opened = False
        self._iter = None
        self._factory = None

    def read(self) -> Optional[Frame]:
        if not self._opened:
            return None
        # Honour fps so loops behave naturally in real time replay.
        now = time.monotonic()
        wait = self._next_due - now
        if wait > 0.0:
            time.sleep(min(wait, self._period_s))
        self._next_due += self._period_s

        try:
            item = self._factory() if self._factory is not None else next(self._iter)  # type: ignore[arg-type]
        except StopIteration:
            return None
        if item is None:
            return None
        if isinstance(item, Frame):
            self._seq = (item.seq + 1) & 0xFFFFFFFF
            return item

        # Treat the item as a luma plane.
        y = item
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        return Frame(
            ts_monotonic_ms=int(time.monotonic() * 1000.0),
            seq=seq,
            width=self.width,
            height=self.height,
            y=y,
        )


class V4l2CaptureBackend(CaptureBackend):
    """Production V4L2 backend — opens ``/dev/videoN`` lazily.

    The ``v4l2`` and ``fcntl`` modules are imported inside :meth:`open`
    so this file can be imported on Windows / CI for syntax + lint checks.
    """

    def __init__(self,
                 device: str = "/dev/video0",
                 *,
                 width: int = DEFAULT_WIDTH,
                 height: int = DEFAULT_HEIGHT,
                 fps: int = DEFAULT_FPS):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self._fd: Any = None
        self._buffers: Sequence[Any] = ()
        self._seq = 0

    def open(self) -> None:                      # pragma: no cover (hardware)
        try:
            import fcntl                         # noqa: F401
            import mmap                          # noqa: F401
            import v4l2                          # type: ignore  # noqa: F401
        except ImportError as exc:
            raise RuntimeError(
                f"V4L2 backend unavailable on this host: {exc}; "
                "fall back to MockCaptureBackend") from exc
        # Implementation lives on the embedded target; the production
        # build pulls in a vendor patch that wires V4L2 MMAP + DQBUF here.
        # For host-side runs this branch is never taken — the unit tests
        # use MockCaptureBackend exclusively.
        raise NotImplementedError(
            "V4l2CaptureBackend.open() must be provided by the embedded "
            "build (vendor patch); host-side tests use MockCaptureBackend")

    def read(self) -> Optional[Frame]:           # pragma: no cover
        raise NotImplementedError

    def close(self) -> None:                     # pragma: no cover
        if self._fd is not None:
            try:
                self._fd.close()
            except Exception:
                pass
            self._fd = None


# ---------------------------------------------------------------------------
# Ring buffer
# ---------------------------------------------------------------------------

class CaptureRing:
    """Newest-wins frame ring with a producer thread.

    The producer pulls frames from ``backend`` as fast as it can; the
    consumer reads :meth:`latest` (non-blocking) or :meth:`next` (blocks
    until a fresh frame appears or the timeout elapses). When the ring
    is full the oldest frame is discarded — the camera never blocks on
    a slow consumer.
    """

    def __init__(self, backend: CaptureBackend, *, depth: int = 4):
        if depth <= 0:
            raise ValueError("depth must be > 0")
        self.backend = backend
        self._depth = depth
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._frames: deque[Frame] = deque(maxlen=depth)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._dropped = 0
        self._produced = 0

    # ----- lifecycle -----

    def start(self) -> None:
        if self._thread is not None:
            return
        self.backend.open()
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="CaptureRing", daemon=True)
        self._thread.start()

    def stop(self, *, join_timeout_s: float = 1.0) -> None:
        self._stop.set()
        with self._cv:
            self._cv.notify_all()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=join_timeout_s)
        self._thread = None
        try:
            self.backend.close()
        except Exception:
            pass

    # ----- consumer API -----

    def latest(self) -> Optional[Frame]:
        with self._lock:
            if not self._frames:
                return None
            return self._frames[-1]

    def next(self, *, timeout_s: Optional[float] = None) -> Optional[Frame]:
        """Block until a frame newer than the last-returned arrives.

        Returns ``None`` on timeout or after :meth:`stop`.
        """
        deadline = None if timeout_s is None else time.monotonic() + timeout_s
        with self._cv:
            last_seq = self._frames[-1].seq if self._frames else None
            while not self._stop.is_set():
                fresh = self._frames[-1] if self._frames else None
                if fresh is not None and fresh.seq != last_seq:
                    return fresh
                if deadline is None:
                    self._cv.wait()
                else:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return None
                    self._cv.wait(timeout=remaining)
        return None

    # ----- introspection -----

    @property
    def produced(self) -> int:
        return self._produced

    @property
    def dropped(self) -> int:
        return self._dropped

    def __len__(self) -> int:
        with self._lock:
            return len(self._frames)

    # ----- producer thread -----

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self.backend.read()
            except Exception:
                # Production code logs; the ring keeps spinning so the
                # next successful read recovers the stream.
                time.sleep(0.05)
                continue
            if frame is None:
                # End-of-stream from the mock backend or a recoverable
                # camera hiccup — back off briefly and retry.
                if self._stop.wait(timeout=0.05):
                    return
                continue
            with self._cv:
                if len(self._frames) == self._depth:
                    self._dropped += 1
                self._frames.append(frame)
                self._produced += 1
                self._cv.notify_all()
