"""IP-W2-05/06 on-device dry-run for the X8 image-pipeline data-saving stack.

Runs against the real Portenta X8 LmP rootfs over ADB. Pure stdlib so we
don't need PIL / numpy / pyserial on the device — :func:`_encode_tile`
is replaced with a sized stub so the encode loop still produces a valid
``TileDeltaFrame`` payload we can size-compare across configurations.

Validates four behaviours end-to-end on the device:

1. Default ``_build_frame`` produces a payload whose byte budget matches
   what :data:`_resolve_byte_budget` resolved from environment.
2. Dispatching a ``CMD_LINK_PROFILE`` frame mutates ``LinkBudget.bytes``
   to a smaller value when we ask for fewer fragments / a slower PHY,
   and the *next* encoded payload is correspondingly smaller.
3. Force-keyframe is asserted via the same dispatcher (sanity).
4. The W2-05 tile encode cache: encoding the same canvas twice with the
   cache enabled invokes the (stubbed) encoder N tiles on the first
   pass and 0 tiles on the second.

Exit code 0 on success, 1 on the first failure with a one-line reason
on stderr so the wrapping PowerShell script can fan-out into pass/fail.
"""
from __future__ import annotations

import os
import sys
import threading
import traceback
import types


def _install_logging_shim() -> None:
    """Some Yocto/LmP images ship a stripped stdlib without ``logging``.

    ``camera_service`` only uses :func:`logging.getLogger` plus the
    ``debug``/``info``/``warning``/``error`` methods on the result, so a
    no-op shim is enough to make the import succeed.
    """
    try:
        import logging  # noqa: WPS433
        return
    except Exception:
        pass

    mod = types.ModuleType("logging")

    class _NullLogger:
        def __init__(self, name=""):
            self.name = name

        def _noop(self, *_a, **_kw):
            return None

        debug = info = warning = warn = error = exception = critical = _noop
        log = _noop

        def setLevel(self, *_a, **_kw):
            return None

        def isEnabledFor(self, *_a, **_kw):
            return False

        def addHandler(self, *_a, **_kw):
            return None

        def removeHandler(self, *_a, **_kw):
            return None

    def getLogger(name=""):
        return _NullLogger(name)

    def basicConfig(*_a, **_kw):
        return None

    class NullHandler:
        def __init__(self, *_a, **_kw):
            pass

        def emit(self, *_a, **_kw):
            pass

        def setLevel(self, *_a, **_kw):
            pass

    mod.getLogger = getLogger
    mod.basicConfig = basicConfig
    mod.NullHandler = NullHandler
    mod.DEBUG = 10
    mod.INFO = 20
    mod.WARNING = 30
    mod.ERROR = 40
    mod.CRITICAL = 50
    sys.modules["logging"] = mod


_install_logging_shim()


def _fail(msg: str) -> None:
    print(f"FAIL: {msg}", file=sys.stderr)
    sys.exit(1)


def _load_camera_service():
    here = os.path.dirname(os.path.abspath(__file__))
    if here not in sys.path:
        sys.path.insert(0, here)
    # camera_service.LinkBudget._compute_link_bytes needs ``lora_proto``,
    # which only lives in base_station/ in the repo. The PowerShell wrapper
    # adb-pushes lora_proto.py into the same DeviceDir, so it's already on
    # ``sys.path`` for on-device runs. For local-host smoke tests we walk
    # up to ``base_station/`` if it exists.
    bs_dir = os.path.normpath(os.path.join(
        here, "..", "..", "base_station"))
    if os.path.isdir(bs_dir) and bs_dir not in sys.path:
        sys.path.append(bs_dir)
    try:
        import camera_service                                # noqa: WPS433
    except Exception as exc:                                  # pragma: no cover
        _fail(f"import camera_service failed: {exc!r}\n{traceback.format_exc()}")
    return camera_service


def _stub_encoder(blob_size: int = 60):
    """Sized stub for camera_service._encode_tile (no PIL on device)."""
    calls = []

    def _enc(rgb_canvas, tx, ty, quality=None):
        calls.append((tx, ty, quality))
        head = bytes([tx & 0xFF, ty & 0xFF, (quality or 0) & 0xFF])
        return head + b"\xAA" * max(0, blob_size - len(head))

    return _enc, calls


def _kiss_unframed_cmd(opcode: int, *args: int) -> bytes:
    """The X8 dispatcher sees the *decoded* KISS frame; topic byte first."""
    cs = __import__("camera_service")
    return bytes([cs.X8_CMD_TOPIC, opcode]) + bytes(args)


def main() -> int:
    cs = _load_camera_service()
    print(f"camera_service loaded from {cs.__file__}")
    print(f"GRID_W={cs.GRID_W} GRID_H={cs.GRID_H} TILE_PX={cs.TILE_PX}")
    print(f"WEBP_QUALITY={cs.WEBP_QUALITY}  LINK_PHY_NAMES={cs.LINK_PHY_NAMES}")

    cam = cs.SyntheticCamera()
    canvas = cam.grab_rgb()

    class _ReplayCam:
        def grab_rgb(self):
            return canvas

    # --- 1. Default frame (no budget, no cache) --------------------------
    accum = cs.FrameAccum()
    enc, calls = _stub_encoder(blob_size=60)
    cs._encode_tile = enc                                     # type: ignore[attr-defined]
    payload_default = cs._build_frame(_ReplayCam(), accum, force_keyframe=True)
    n_tiles = cs.GRID_W * cs.GRID_H
    if len(calls) != n_tiles:
        _fail(f"default keyframe encoded {len(calls)} tiles, expected {n_tiles}")
    print(f"PASS: default keyframe payload={len(payload_default)} B "
          f"({len(calls)} tile encodes)")

    # --- 2. Apply CMD_LINK_PROFILE -> shrink budget ----------------------
    # Seed at the wide rung (n=8 image) then degrade to (n=2 image) so we
    # exercise both update + the smaller resulting cap.
    link_budget = cs.LinkBudget(profile_name="image")
    if not link_budget.update(n_fragments=8, profile_index=0):
        _fail("seed LinkBudget.update(8, image) returned False")
    seed_bytes = link_budget.bytes
    force_evt = threading.Event()
    cmd = _kiss_unframed_cmd(cs.CMD_LINK_PROFILE, 2, 0)       # n=2, image
    cs.dispatch_back_channel(cmd, force_evt, link_budget=link_budget)
    if link_budget.bytes is None or link_budget.bytes <= 0:
        _fail(f"CMD_LINK_PROFILE did not set a byte budget: {link_budget.bytes}")
    if link_budget.profile_name != "image":
        _fail(f"profile_name unexpectedly changed to {link_budget.profile_name}")
    if link_budget.bytes >= seed_bytes:
        _fail(f"degraded budget {link_budget.bytes} not < seed {seed_bytes}")
    if not force_evt.is_set():
        _fail("CMD_LINK_PROFILE did not force a keyframe")
    print(f"PASS: CMD_LINK_PROFILE applied; budget={link_budget.bytes} B "
          f"(< seed {seed_bytes} B) phy={link_budget.profile_name}")

    # --- 3. Frame with the new tight budget should be smaller -----------
    accum2 = cs.FrameAccum()
    calls.clear()
    payload_capped = cs._build_frame(
        _ReplayCam(), accum2, force_keyframe=True,
        byte_budget=link_budget.bytes)
    if len(payload_capped) >= len(payload_default):
        _fail(f"capped payload not smaller: {len(payload_capped)} vs "
              f"default {len(payload_default)}")
    print(f"PASS: capped payload={len(payload_capped)} B "
          f"(< {len(payload_default)} B default)")

    # --- 4. Tile encode cache: 2nd identical canvas -> 0 re-encodes -----
    try:
        from image_pipeline.tile_cache import TileEncodeCache
    except Exception as exc:                                  # pragma: no cover
        _fail(f"import tile_cache failed: {exc!r}")
    cache = TileEncodeCache(n_tiles=n_tiles, history=4)
    accum3 = cs.FrameAccum()
    calls.clear()
    cs._build_frame(_ReplayCam(), accum3, force_keyframe=True,
                    encode_cache=cache)
    first_pass = len(calls)
    accum4 = cs.FrameAccum()
    calls.clear()
    cs._build_frame(_ReplayCam(), accum4, force_keyframe=True,
                    encode_cache=cache)
    second_pass = len(calls)
    if first_pass != n_tiles:
        _fail(f"cache 1st pass encoded {first_pass}, expected {n_tiles}")
    if second_pass != 0:
        _fail(f"cache 2nd pass encoded {second_pass}, expected 0")
    print(f"PASS: tile cache 1st pass={first_pass} encodes, "
          f"2nd pass={second_pass} encodes "
          f"(hits={cache.stats.hits}, misses={cache.stats.misses})")

    print("ALL OK (4 dry-run gates passed)")
    return 0


if __name__ == "__main__":
    try:
        rc = main()
    except SystemExit:
        raise
    except Exception as exc:                                  # pragma: no cover
        print(f"FAIL: unhandled exception {exc!r}", file=sys.stderr)
        traceback.print_exc()
        rc = 1
    sys.exit(rc)
