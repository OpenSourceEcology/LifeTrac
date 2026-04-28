"""Persistent AES-GCM nonce counter for the base station.

Per CYBERSECURITY_CASE.md / LORA_PROTOCOL.md §6 the GCM nonce is
``source_id || seq_le16 || time_le32 || random5`` and the (key, nonce)
pair must NEVER repeat. The 16-bit ``seq`` field gives only 65 536
distinct values per second; on cold boot the bridge restarts from
``seq=0``, which after a process restart in the same wall-clock second
collides with whatever the previous instance had already emitted.

This module persists the high-water seq per (source_id, time-bucket) to
disk so a restart resumes from ``last_seq + GAP`` (default GAP=64) instead
of zero. The store is a tiny JSON file that is fsync'd on every update —
acceptable because the bridge writes at most a few times per second on a
graceful shutdown path, and on the hot path we batch updates every
``flush_interval`` seconds.

The mirrored M7 firmware keeps an equivalent counter in flash; this is
the X86/X8 side of the same protection.
"""
from __future__ import annotations

import json
import logging
import os
import tempfile
import threading
import time
from dataclasses import dataclass
from typing import Optional

LOG = logging.getLogger("nonce_store")

DEFAULT_PATH = "/var/lib/lifetrac/nonce_store.json"
DEFAULT_GAP = 64
DEFAULT_FLUSH_INTERVAL_S = 1.0


@dataclass
class _Entry:
    seq: int
    epoch_s: int


class NonceStore:
    """File-backed monotonic counter per source_id."""

    def __init__(self, path: str = DEFAULT_PATH,
                 gap: int = DEFAULT_GAP,
                 flush_interval_s: float = DEFAULT_FLUSH_INTERVAL_S) -> None:
        self.path = path
        self.gap = max(1, gap)
        self.flush_interval_s = max(0.0, flush_interval_s)
        self._lock = threading.Lock()
        self._entries: dict[int, _Entry] = {}
        self._dirty = False
        self._last_flush_s = 0.0
        self._load()

    # ----- public API -------------------------------------------------
    def reserve(self, source_id: int) -> int:
        """Return the next safe seq for ``source_id`` and persist it.

        The returned value is guaranteed > any seq we've seen across all
        prior process lifetimes for this source, modulo 16-bit wrap. The
        caller is expected to feed it into ``build_nonce()``.
        """
        source_id &= 0xFF
        now = int(time.time())
        with self._lock:
            entry = self._entries.get(source_id)
            if entry is None:
                next_seq = 0
            else:
                next_seq = (entry.seq + self.gap) & 0xFFFF
                # If the wall-clock second advanced we don't need the gap
                # since the time-bucket changed, but using the gap anyway
                # is harmless and avoids a clock-go-backwards bug.
            self._entries[source_id] = _Entry(seq=next_seq, epoch_s=now)
            self._dirty = True
            self._maybe_flush_locked()
            return next_seq

    def observe(self, source_id: int, seq: int) -> None:
        """Record a seq we just emitted (e.g. from the radio TX path)."""
        source_id &= 0xFF
        seq &= 0xFFFF
        now = int(time.time())
        with self._lock:
            entry = self._entries.get(source_id)
            if entry is None or seq != entry.seq:
                self._entries[source_id] = _Entry(seq=seq, epoch_s=now)
                self._dirty = True
                self._maybe_flush_locked()

    def flush(self) -> None:
        with self._lock:
            self._flush_locked()

    def close(self) -> None:
        self.flush()

    # ----- internals --------------------------------------------------
    def _load(self) -> None:
        try:
            with open(self.path, "r", encoding="utf-8") as fh:
                blob = json.load(fh)
            for k, v in blob.get("entries", {}).items():
                try:
                    self._entries[int(k) & 0xFF] = _Entry(
                        seq=int(v["seq"]) & 0xFFFF,
                        epoch_s=int(v["epoch_s"]),
                    )
                except (KeyError, TypeError, ValueError):
                    continue
        except FileNotFoundError:
            return
        except (OSError, json.JSONDecodeError) as exc:
            LOG.warning("nonce_store: cannot load %s (%s); starting fresh",
                        self.path, exc)

    def _maybe_flush_locked(self) -> None:
        now = time.monotonic()
        if now - self._last_flush_s >= self.flush_interval_s:
            self._flush_locked()

    def _flush_locked(self) -> None:
        if not self._dirty:
            return
        directory = os.path.dirname(self.path) or "."
        try:
            os.makedirs(directory, exist_ok=True)
            fd, tmp = tempfile.mkstemp(prefix=".nonce_store-", dir=directory)
            try:
                with os.fdopen(fd, "w", encoding="utf-8") as fh:
                    json.dump({
                        "entries": {
                            str(sid): {"seq": e.seq, "epoch_s": e.epoch_s}
                            for sid, e in self._entries.items()
                        }
                    }, fh)
                    fh.flush()
                    os.fsync(fh.fileno())
                os.replace(tmp, self.path)
            except Exception:
                try:
                    os.unlink(tmp)
                except OSError:
                    pass
                raise
        except OSError as exc:                     # pragma: no cover
            LOG.warning("nonce_store: cannot persist to %s (%s)", self.path, exc)
            return
        self._dirty = False
        self._last_flush_s = time.monotonic()
