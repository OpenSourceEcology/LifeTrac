"""settings_store.py — base-station persistent operator settings.

Tiny JSON key/value store for base-local settings (Coral toggle, future
camera defaults, future operator preferences). Distinct from the *tractor*
`firmware/tractor_x8/params_service.py` because base-only settings (Coral
hardware lives on the base X8) have no business round-tripping through MQTT
to the tractor.

Thread-safe; atomic write via temp-file + rename.
"""

from __future__ import annotations

import json
import logging
import os
import threading
from pathlib import Path
from typing import Any

LOG = logging.getLogger("settings_store")

DEFAULT_PATH = Path(os.environ.get(
    "LIFETRAC_BASE_SETTINGS",
    "/var/lib/lifetrac/base_settings.json"))

DEFAULTS: dict[str, Any] = {
    "image": {
        # Operator master switch for the optional Coral Edge TPU.
        # Default True so a freshly-installed-and-detected Coral is
        # immediately used per MASTER_PLAN.md §8.19.
        "coral_enabled": True,
    },
}


class SettingsStore:
    def __init__(self, path: Path = DEFAULT_PATH):
        self._path = Path(path)
        self._lock = threading.Lock()
        self._data: dict[str, Any] = self._load()

    def _load(self) -> dict[str, Any]:
        if self._path.exists():
            try:
                disk = json.loads(self._path.read_text())
                return _merge(DEFAULTS, disk)
            except (OSError, json.JSONDecodeError) as exc:
                LOG.warning("settings: load failed (%s) — defaults", exc)
        return _deepcopy(DEFAULTS)

    def get(self, dotted_key: str, default: Any = None) -> Any:
        with self._lock:
            cur: Any = self._data
            for part in dotted_key.split("."):
                if not isinstance(cur, dict) or part not in cur:
                    return default
                cur = cur[part]
            return cur

    def set(self, dotted_key: str, value: Any) -> None:
        with self._lock:
            parts = dotted_key.split(".")
            cur = self._data
            for part in parts[:-1]:
                if not isinstance(cur.get(part), dict):
                    cur[part] = {}
                cur = cur[part]
            cur[parts[-1]] = value
            self._save_locked()

    def _save_locked(self) -> None:
        try:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            tmp = self._path.with_suffix(".json.tmp")
            tmp.write_text(json.dumps(self._data, indent=2, sort_keys=True))
            tmp.replace(self._path)
        except OSError as exc:
            LOG.warning("settings: save failed: %s", exc)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return _deepcopy(self._data)


def _deepcopy(d: Any) -> Any:
    if isinstance(d, dict):
        return {k: _deepcopy(v) for k, v in d.items()}
    if isinstance(d, list):
        return [_deepcopy(v) for v in d]
    return d


def _merge(base: dict, overlay: dict) -> dict:
    out = _deepcopy(base)
    for k, v in (overlay or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = _merge(out[k], v)
        else:
            out[k] = v
    return out


_default_store: SettingsStore | None = None
_default_lock = threading.Lock()


def get_store() -> SettingsStore:
    global _default_store
    with _default_lock:
        if _default_store is None:
            _default_store = SettingsStore()
        return _default_store
