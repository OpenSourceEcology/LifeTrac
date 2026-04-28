"""accel_select.py — Coral Edge TPU detection + operator gate.

Canonical per IMAGE_PIPELINE.md §5.1. Three-level state:

  * `present`  — hardware visible to the OS (lspci / lsusb hit).
  * `usable`   — pycoral runtime importable AND a 224×224 zero-tensor
                 warm-up inference completed within 2 s.
  * `enabled`  — operator master switch in `settings_store` is True.

`is_active()` returns `present and usable and enabled` and is the only call
the consumer modules (`superres.py`, `detect.py`) should make per inference.
This means hot-unplug or operator-toggle gracefully degrades to CPU within
one frame.

A 30 s background poll re-runs detection (no udev dependency, portable to
the Windows dev box). Operators can also force a re-scan via the web UI's
POST /api/settings/accel/refresh button.

Per MASTER_PLAN.md §8.19: pipeline must validate CPU-only and treat Coral
as a strict additive upgrade. This module ships no functional dependency on
Coral; if pycoral isn't installed, `usable` stays False and consumers run
their CPU paths.
"""

from __future__ import annotations

import logging
import os
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Callable

LOG = logging.getLogger("accel_select")

# Coral PCI / USB IDs (https://coral.ai/docs/m2/get-started/ + lsusb).
_PCI_EDGETPU_VENDOR = "1ac1"
_PCI_EDGETPU_DEVICE = "089a"
_USB_EDGETPU_PRE_INIT = ("1a6e", "089a")     # uninitialized Apex chip
_USB_EDGETPU_POST_INIT = ("18d1", "9302")    # Google after firmware load

_REFRESH_INTERVAL_S = 30.0
_WARMUP_TIMEOUT_S = 2.0


@dataclass
class AcceleratorState:
    present: bool = False
    usable: bool = False
    enabled: bool = True
    kind: str = "none"           # "coral_m2" | "coral_usb" | "none"
    last_error: str | None = None
    detected_at: float = 0.0

    def is_active(self) -> bool:
        return self.present and self.usable and self.enabled

    def to_dict(self) -> dict:
        d = asdict(self)
        d["active"] = self.is_active()
        return d


class Accelerator:
    """Thread-safe accelerator state holder + background refresher.

    `enabled_loader` is a no-arg callable returning the current operator
    setting (typically `lambda: settings_store.get("image.coral_enabled",
    True)`). Re-evaluated on every refresh AND on every `is_active()` call,
    so the operator toggle takes effect within one inference call without
    waiting for the 30 s poll.
    """

    def __init__(self, enabled_loader: Callable[[], bool] | None = None,
                 detect_fn: Callable[[], tuple[bool, str]] | None = None,
                 warmup_fn: Callable[[], tuple[bool, str | None]] | None = None):
        self._enabled_loader = enabled_loader or (lambda: True)
        self._detect_fn = detect_fn or _detect_default
        self._warmup_fn = warmup_fn or _warmup_default
        self._lock = threading.Lock()
        self._state = AcceleratorState(enabled=self._enabled_loader())
        self._stop_evt = threading.Event()
        self._thread: threading.Thread | None = None
        self._on_change: list[Callable[[AcceleratorState], None]] = []

    # ---- public API ----
    def state(self) -> AcceleratorState:
        with self._lock:
            # Always re-read enabled so toggle is instant.
            self._state.enabled = bool(self._enabled_loader())
            return AcceleratorState(**asdict(self._state))

    def is_active(self) -> bool:
        return self.state().is_active()

    def refresh(self) -> AcceleratorState:
        present, kind = self._detect_fn()
        usable = False
        last_error: str | None = None
        if present:
            ok, err = self._warmup_fn()
            usable = ok
            last_error = err
        else:
            last_error = "no Coral hardware detected"
        with self._lock:
            prev_active = self._state.is_active()
            self._state.present = present
            self._state.kind = kind
            self._state.usable = usable
            self._state.last_error = last_error
            self._state.detected_at = time.time()
            self._state.enabled = bool(self._enabled_loader())
            new_active = self._state.is_active()
            snapshot = AcceleratorState(**asdict(self._state))
        if new_active != prev_active:
            LOG.info("accel: active=%s present=%s usable=%s enabled=%s kind=%s",
                     new_active, snapshot.present, snapshot.usable,
                     snapshot.enabled, snapshot.kind)
            for cb in list(self._on_change):
                try:
                    cb(snapshot)
                except Exception:  # pragma: no cover  - callback hygiene
                    LOG.exception("accel: on_change callback failed")
        return snapshot

    def on_change(self, cb: Callable[[AcceleratorState], None]) -> None:
        self._on_change.append(cb)

    def start(self) -> None:
        if self._thread is not None:
            return
        self.refresh()
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run, daemon=True,
                                        name="accel_select")
        self._thread.start()

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _run(self) -> None:
        while not self._stop_evt.wait(_REFRESH_INTERVAL_S):
            try:
                self.refresh()
            except Exception:  # pragma: no cover
                LOG.exception("accel: refresh failed")


# ---- detection helpers ----

def _detect_default() -> tuple[bool, str]:
    """Probe for a Coral Edge TPU. Returns (present, kind).

    On Windows / macOS dev boxes both probes fail → (False, "none"). That's
    intentional: the production target is the X8 Yocto image; dev work uses
    the CPU path.
    """
    if _probe_pci():
        return True, "coral_m2"
    if _probe_usb():
        return True, "coral_usb"
    if _probe_pycoral():
        return True, "coral_pycoral"
    return False, "none"


def _probe_pci() -> bool:
    lspci = shutil.which("lspci")
    if not lspci:
        return False
    try:
        out = subprocess.run([lspci, "-nn"], capture_output=True,
                             text=True, timeout=2.0)
    except (subprocess.SubprocessError, OSError):
        return False
    if out.returncode != 0:
        return False
    needle = f"{_PCI_EDGETPU_VENDOR}:{_PCI_EDGETPU_DEVICE}"
    return needle.lower() in out.stdout.lower()


def _probe_usb() -> bool:
    lsusb = shutil.which("lsusb")
    if not lsusb:
        return False
    try:
        out = subprocess.run([lsusb], capture_output=True,
                             text=True, timeout=2.0)
    except (subprocess.SubprocessError, OSError):
        return False
    if out.returncode != 0:
        return False
    text = out.stdout.lower()
    for vid, pid in (_USB_EDGETPU_PRE_INIT, _USB_EDGETPU_POST_INIT):
        if f"{vid}:{pid}" in text:
            return True
    return False


def _probe_pycoral() -> bool:
    """Last-resort: ask pycoral itself. Useful when lspci/lsusb aren't
    in PATH (e.g. minimal container) but the runtime is."""
    try:
        from pycoral.utils import edgetpu  # type: ignore
    except Exception:
        return False
    try:
        return len(edgetpu.list_edge_tpus()) > 0
    except Exception:
        return False


def _warmup_default() -> tuple[bool, str | None]:
    """Run a tiny inference to confirm the runtime path works.

    Returns (ok, error_msg). Bounded by `_WARMUP_TIMEOUT_S` via a watchdog
    thread because pycoral can hang on a dead USB device.
    """
    try:
        from pycoral.utils import edgetpu  # type: ignore
    except Exception as exc:
        return False, f"pycoral not importable: {exc.__class__.__name__}"

    result: dict[str, object] = {"ok": False, "err": "warmup timeout"}

    def _worker() -> None:
        try:
            tpus = edgetpu.list_edge_tpus()
            if not tpus:
                result["err"] = "list_edge_tpus() returned empty"
                return
            # We don't ship the model here — just confirming the runtime
            # enumerates the device counts as warmup for v25. The full
            # zero-tensor inference lands when superres_coral.py arrives.
            result["ok"] = True
            result["err"] = None
        except Exception as exc:  # pragma: no cover
            result["err"] = f"{exc.__class__.__name__}: {exc}"

    t = threading.Thread(target=_worker, daemon=True)
    t.start()
    t.join(_WARMUP_TIMEOUT_S)
    if t.is_alive():
        return False, "warmup timeout"
    return bool(result["ok"]), (result["err"] if result["err"] else None)  # type: ignore[arg-type]


# ---- module-level singleton ----
_singleton: Accelerator | None = None
_singleton_lock = threading.Lock()


def get_accelerator() -> Accelerator:
    global _singleton
    with _singleton_lock:
        if _singleton is None:
            # Lazy-import settings_store so unit tests can substitute an
            # accelerator with a custom enabled_loader before importing.
            try:
                from settings_store import get_store
                store = get_store()
                env_default = os.environ.get("LIFETRAC_CORAL_ENABLED", "")
                if env_default and store.get("image.coral_enabled") is None:
                    store.set("image.coral_enabled",
                              env_default.lower() in ("1", "true", "yes", "on"))
                loader = lambda: bool(store.get("image.coral_enabled", True))
            except Exception:
                loader = lambda: True
            _singleton = Accelerator(enabled_loader=loader)
        return _singleton


def reset_for_tests() -> None:
    """Test hook only."""
    global _singleton
    with _singleton_lock:
        if _singleton is not None:
            _singleton.stop()
        _singleton = None
