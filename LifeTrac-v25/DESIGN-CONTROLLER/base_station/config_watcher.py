"""Watch-and-reload helper for build-config hot-reload (Round 29b-beta / BC-10).

Both ``web_ui`` (base side) and ``lora_bridge`` (tractor side) need to
detect when the on-disk ``build.<unit>.toml`` has changed under them --
typically because the X8-side installer (:mod:`installer_daemon`) just
atomically replaced it -- and decide what to do about it. The decision
isn't "always reload immediately": a ``restart_required`` change has to
defer to a daemon restart, and a ``live`` change still has to wait for
the unit to be quiescent (no active control session, M7 TX queue empty,
engine idle, parked > 30 s; see :func:`build_config.evaluate_quiescence`).

This module is the shared decision helper. It owns no I/O and no
threading -- the caller (web_ui's lifespan task, lora_bridge's main
loop) drives it with explicit ``poll(now=...)`` calls. That keeps it
trivially testable: feed it a fake clock, fake source-path mtimes, and
fake quiescence states, and assert what events come out.

Event vocabulary
----------------

:data:`EVENT_NOOP`
    Source unchanged since last poll. Caller does nothing.

:data:`EVENT_APPLIED`
    A ``live`` change loaded successfully and quiescence held; caller
    should swap its in-memory ``BuildConfig`` reference. Carries the
    new config + the :class:`build_config.ReloadDiff`.

:data:`EVENT_DEFERRED`
    A ``live`` change is on disk but quiescence does not hold. Caller
    surfaces the deferred state on the UI banner. The watcher will
    re-attempt on the next poll.

:data:`EVENT_RESTART_PENDING`
    A ``restart_required`` change is on disk. Caller does **not** swap
    -- it sets a "pending reload, restart to apply" flag visible on
    ``/api/status`` so an operator knows to bounce the daemon.

:data:`EVENT_FIREMWARE_REQUIRED`
    A ``firmware_required`` change is on disk. The X8 installer should
    have rejected this upstream, but if it leaks through (manual edit,
    pre-Round-29b daemon) we surface it so the operator can roll back.

:data:`EVENT_REJECTED`
    The on-disk file failed to load (schema error, missing file). The
    caller keeps the previous in-memory config and surfaces the reason.
"""

from __future__ import annotations

import logging
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Optional

import build_config

EVENT_NOOP = "noop"
EVENT_APPLIED = "applied"
EVENT_DEFERRED = "deferred"
EVENT_RESTART_PENDING = "restart_pending"
EVENT_FIRMWARE_REQUIRED = "firmware_required"
EVENT_REJECTED = "rejected"

EVENTS: tuple[str, ...] = (
    EVENT_NOOP,
    EVENT_APPLIED,
    EVENT_DEFERRED,
    EVENT_RESTART_PENDING,
    EVENT_FIRMWARE_REQUIRED,
    EVENT_REJECTED,
)

log = logging.getLogger(__name__)


@dataclass(frozen=True)
class WatchEvent:
    """One ``poll()`` outcome.

    For ``noop`` / ``deferred`` both ``new_cfg`` and ``diff`` are None.
    For ``applied`` both are populated. For ``restart_pending`` /
    ``firmware_required`` ``diff`` is populated but ``new_cfg`` is
    None (the caller intentionally does **not** swap).
    For ``rejected`` ``reason`` carries the loader error.
    """

    kind: str
    reason: str | None = None
    new_cfg: build_config.BuildConfig | None = None
    diff: build_config.ReloadDiff | None = None


@dataclass
class _SourceFingerprint:
    mtime_ns: int
    size: int

    @classmethod
    def from_path(cls, path: Path) -> "_SourceFingerprint":
        st = path.stat()
        return cls(mtime_ns=st.st_mtime_ns, size=st.st_size)


class ConfigWatcher:
    """Polling watcher around a ``BuildConfig.source_path``.

    Stateful (holds the current cfg and last-seen fingerprint) so the
    caller can drive a simple ``while True: poll(); sleep(N)`` loop
    without thinking about diffing or quiescence themselves.

    The ``get_quiescence`` callable is invoked *only* when a live
    candidate is on disk. It returns a :class:`build_config.QuiescenceState`;
    the watcher does the ``evaluate_quiescence`` itself so callers
    don't have to import the predicate.

    Parameters
    ----------
    initial_cfg : BuildConfig
        Currently-active config. The watcher uses ``initial_cfg.source_path``
        as the file to watch and ``initial_cfg.config_sha256`` as the
        diff baseline.
    get_quiescence : callable
        Returns the current :class:`build_config.QuiescenceState`. Only
        called when needed (i.e. a live change is on disk). On the
        base side (``web_ui``) this is mostly trivial since the base
        UI doesn't run hydraulics; on the tractor side it reads the
        in-memory M4/M7 telemetry.
    parked_seconds_min : float
        Threshold passed through to
        :func:`build_config.evaluate_quiescence`.
    """

    def __init__(
        self,
        initial_cfg: build_config.BuildConfig,
        *,
        get_quiescence: Callable[[], build_config.QuiescenceState],
        parked_seconds_min: float = 30.0,
    ) -> None:
        self._cfg = initial_cfg
        self._get_quiescence = get_quiescence
        self._parked_seconds_min = float(parked_seconds_min)
        self._fingerprint: Optional[_SourceFingerprint] = None
        self._restart_pending: bool = False
        self._restart_pending_diff: Optional[build_config.ReloadDiff] = None
        try:
            self._fingerprint = _SourceFingerprint.from_path(initial_cfg.source_path)
        except OSError as exc:
            log.warning("ConfigWatcher: initial stat failed: %s", exc)

    @property
    def current(self) -> build_config.BuildConfig:
        """Currently-active config (post any successful live applies)."""
        return self._cfg

    @property
    def restart_pending(self) -> bool:
        """True iff a ``restart_required`` change is on disk and we
        deliberately did not apply it. Surface on ``/api/status``."""
        return self._restart_pending

    @property
    def restart_pending_diff(self) -> build_config.ReloadDiff | None:
        return self._restart_pending_diff

    def poll(self) -> WatchEvent:
        """Check the source file once. Idempotent if nothing changed.

        Never raises for transient I/O errors; emits ``EVENT_REJECTED``
        with the reason and keeps the existing in-memory config in
        place. The caller is expected to call ``poll()`` periodically
        from a single thread / task; not safe for concurrent calls.
        """
        path = self._cfg.source_path
        try:
            fp = _SourceFingerprint.from_path(path)
        except OSError as exc:
            return WatchEvent(kind=EVENT_REJECTED, reason=f"stat {path}: {exc}")

        if self._fingerprint is not None and (
            fp.mtime_ns == self._fingerprint.mtime_ns
            and fp.size == self._fingerprint.size
        ):
            return WatchEvent(kind=EVENT_NOOP)

        # File changed -- attempt a fresh load.
        prev = os.environ.get("LIFETRAC_BUILD_CONFIG_PATH")
        os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(path)
        try:
            try:
                candidate = build_config.load()
            except build_config.BuildConfigError as exc:
                # Don't update the fingerprint -- we want to retry on the
                # next poll in case the file was caught mid-write. (The
                # X8 installer writes atomically so this should be rare;
                # a manual edit-in-place is the realistic case.)
                return WatchEvent(kind=EVENT_REJECTED, reason=str(exc))
        finally:
            if prev is None:
                os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
            else:
                os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = prev

        # Loaded OK -- commit the new fingerprint regardless of what we
        # decide to do with the contents (otherwise we'd re-load every
        # poll for a deferred change).
        self._fingerprint = fp

        if candidate.config_sha256 == self._cfg.config_sha256:
            # Same SHA, just a touched mtime (e.g. atomic-rename of an
            # identical body). Treat as noop.
            return WatchEvent(kind=EVENT_NOOP)

        try:
            diff = build_config.diff_reload_classes(self._cfg, candidate)
        except build_config.BuildConfigError as exc:
            return WatchEvent(kind=EVENT_REJECTED, reason=str(exc))

        if diff.firmware_required:
            self._restart_pending = True
            self._restart_pending_diff = diff
            return WatchEvent(
                kind=EVENT_FIRMWARE_REQUIRED,
                reason=f"firmware re-flash required: {', '.join(diff.changed)}",
                diff=diff,
            )
        if diff.restart_required:
            self._restart_pending = True
            self._restart_pending_diff = diff
            return WatchEvent(
                kind=EVENT_RESTART_PENDING,
                reason=f"restart required: {', '.join(diff.changed)}",
                diff=diff,
            )

        # Live class. Gate on quiescence.
        try:
            state = self._get_quiescence()
        except Exception as exc:  # noqa: BLE001 - quiescence callable is caller-supplied
            log.warning("ConfigWatcher: get_quiescence raised: %s", exc)
            return WatchEvent(
                kind=EVENT_REJECTED, reason=f"quiescence probe failed: {exc}"
            )
        q = build_config.evaluate_quiescence(
            state, parked_seconds_min=self._parked_seconds_min
        )
        if not q.ok:
            return WatchEvent(
                kind=EVENT_DEFERRED, reason=q.reason, diff=diff
            )

        # Safe -- swap in the new config.
        self._cfg = candidate
        return WatchEvent(kind=EVENT_APPLIED, new_cfg=candidate, diff=diff)


__all__ = [
    "ConfigWatcher",
    "EVENTS",
    "EVENT_APPLIED",
    "EVENT_DEFERRED",
    "EVENT_FIRMWARE_REQUIRED",
    "EVENT_NOOP",
    "EVENT_REJECTED",
    "EVENT_RESTART_PENDING",
    "WatchEvent",
]
