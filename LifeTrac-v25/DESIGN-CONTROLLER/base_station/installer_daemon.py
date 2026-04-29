"""X8-side installer daemon for build-config bundles (Round 29b-beta / BC-10).

Round 29b-alpha shipped the *producer* side: a base-UI download route, a
laptop-side ``lifetrac-config`` CLI that builds bundles, and a bundle
envelope module (:mod:`config_bundle`) with anti-tamper checks. This
module ships the *consumer* side: the daemon code that runs on the X8
compute module, watches a USB stick (or LAN drop directory), verifies
each candidate against the unit's identity, atomically replaces the
active TOML, and writes a small ``lifetrac-config-result.json`` next to
the bundle so an operator unplugging the stick can read what happened.

The module is pure-Python and stdlib-only on purpose: a SIL gate can
exercise every code path without mocking out a real udev mount, and the
production wrapper script is a thin shim that calls
:func:`apply_bundle` + :func:`write_result_file` in a polling loop.

Design contract
---------------

* The daemon **never overwrites** the active TOML in place. It writes
  to ``<target>.new`` and then ``os.replace``-s it onto ``<target>``.
  POSIX guarantees the rename is atomic on the same filesystem; a
  partial write therefore cannot leave a half-applied config behind.
* The daemon **refuses** any bundle whose filename ``unit_id`` (or
  embedded header ``unit_id``, or body ``unit_id``) does not match
  this X8's own identity. This is the primary anti-foot-gun: walking
  the wrong stick to the wrong tractor must not apply silently.
* The daemon **classifies** the change against the in-memory current
  config using :func:`build_config.diff_reload_classes`. ``live`` and
  ``restart_required`` writes succeed (the watcher loop in
  :mod:`config_watcher` decides when to re-load / restart).
  ``firmware_required`` writes are **rejected** -- a re-flash is a
  bench operation, not a USB-stick operation, and we refuse to write
  a TOML the running firmware can't honour.
* Every outcome (applied / rejected / noop / deferred) produces an
  :class:`InstallResult` with a stable ``status`` token and a short
  human-readable ``reason``. The result is what gets serialised to
  ``lifetrac-config-result.json`` *and* what drives the OLED + LED
  feedback in :mod:`feedback`.

This module owns the format of ``lifetrac-config-result.json``. The
schema is intentionally tiny so an operator can ``cat`` it on a field
laptop:

.. code-block:: json

    {
      "status": "applied",
      "reason": "live reload (3 leaves changed)",
      "unit_id": "lifetrac-001",
      "applied_sha": "9ce6...",
      "prior_sha":   "abcd...",
      "reload_class": "live",
      "applied_at":  "2026-04-29T17:23:01Z",
      "schema_version": 1,
      "result_version": 1
    }

``result_version`` is bumped if the schema ever grows; an old base-side
log-collector reading new result files just ignores extra fields.
"""

from __future__ import annotations

import json
import logging
import os
import tempfile
from dataclasses import asdict, dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import build_config
import config_bundle

RESULT_FILENAME = "lifetrac-config-result.json"
RESULT_VERSION = 1

# Status tokens. Stable: feedback table, log queries, and result files
# all key off these strings.
STATUS_APPLIED = "applied"
STATUS_REJECTED = "rejected"
STATUS_NOOP = "noop"
STATUS_DEFERRED = "deferred"

STATUSES: tuple[str, ...] = (STATUS_APPLIED, STATUS_REJECTED, STATUS_NOOP, STATUS_DEFERRED)

log = logging.getLogger(__name__)


class InstallerError(Exception):
    """Raised by :func:`discover_bundles` for fatal mount-side problems.

    Per-bundle verification failures do **not** raise -- they return an
    :class:`InstallResult` with ``status == "rejected"`` so the daemon
    can write a result file and keep going.
    """


@dataclass(frozen=True)
class InstallResult:
    """Outcome of a single ``apply_bundle()`` call.

    Always serialisable: every field is a primitive or ``None``.
    """

    status: str
    reason: str
    unit_id: str
    applied_sha: str | None
    prior_sha: str | None
    reload_class: str | None
    applied_at: str
    schema_version: int | None = None
    result_version: int = RESULT_VERSION

    def to_json_dict(self) -> dict[str, object]:
        return asdict(self)

    @property
    def ok(self) -> bool:
        return self.status == STATUS_APPLIED


def _utcnow_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def discover_bundles(mount_dir: Path, unit_id: str) -> list[Path]:
    """Glob for ``lifetrac-config-<unit_id>-*.toml`` under ``mount_dir``.

    Only filenames matching :data:`config_bundle.FILENAME_RE` and whose
    parsed ``unit_id`` equals this X8's are returned. A stick with one
    bundle per unit (the common case) returns a single-element list; a
    stick with bundles for several units is fine -- ours is selected.

    Raises :class:`InstallerError` if ``mount_dir`` doesn't exist or
    isn't a directory; an empty match list is **not** an error
    (that's "stick has no bundle for me", a normal idle case).
    """
    if not mount_dir.is_dir():
        raise InstallerError(f"mount_dir {mount_dir} is not a directory")
    out: list[Path] = []
    for entry in sorted(mount_dir.iterdir()):
        if not entry.is_file():
            continue
        try:
            uid, _sha8 = config_bundle.parse_filename(entry.name)
        except config_bundle.BundleError:
            continue  # foreign file on the stick -- ignore quietly
        if uid == unit_id:
            out.append(entry)
    return out


def _load_body_via_temp(body: str) -> build_config.BuildConfig:
    """Schema-validate a body string by writing it to a temp file and
    invoking the canonical :func:`build_config.load`.

    The loader is tied to a file path (TOML reader + per-unit fallback);
    this helper is the single place we bridge "in-memory body" to "what
    the loader expects." Mirrors the same temp-file pattern used by the
    laptop CLI (:mod:`tools.lifetrac_config`).
    """
    with tempfile.NamedTemporaryFile(
        "w", suffix=".toml", delete=False, encoding="utf-8"
    ) as tf:
        tf.write(body)
        tmp_path = Path(tf.name)
    prev = os.environ.get("LIFETRAC_BUILD_CONFIG_PATH")
    os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = str(tmp_path)
    try:
        return build_config.load()
    finally:
        if prev is None:
            os.environ.pop("LIFETRAC_BUILD_CONFIG_PATH", None)
        else:
            os.environ["LIFETRAC_BUILD_CONFIG_PATH"] = prev
        tmp_path.unlink(missing_ok=True)


def _atomic_write(target: Path, body: str) -> None:
    """``os.replace``-based atomic write into ``target``.

    The temp file is created in the same directory as ``target`` so
    ``os.replace`` can rename across no boundaries (it has to be the
    same filesystem for atomicity).
    """
    target.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_str = tempfile.mkstemp(
        prefix=target.name + ".", suffix=".new", dir=str(target.parent)
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(body)
        os.replace(tmp_str, target)
    except Exception:
        try:
            os.unlink(tmp_str)
        except OSError:
            pass
        raise


def apply_bundle(
    bundle_path: Path,
    *,
    target_path: Path,
    unit_id: str,
    current_cfg: build_config.BuildConfig | None = None,
) -> InstallResult:
    """Verify, classify, and (if safe) atomically install a bundle.

    Parameters
    ----------
    bundle_path : Path
        Bundle file as it sits on the USB stick or LAN drop directory.
    target_path : Path
        Active TOML path -- the file the running daemon reads on next
        ``build_config.load()``. Atomically replaced on success.
    unit_id : str
        This X8's identity. The bundle's filename, header, and body all
        have to agree with this; mismatch on any layer is a rejection.
    current_cfg : BuildConfig, optional
        In-memory current config, used to compute the
        :class:`build_config.ReloadDiff`. When omitted (cold-boot, no
        prior config to diff against), the result's ``reload_class`` is
        ``None`` and the install is treated as a fresh apply.

    Returns
    -------
    InstallResult
        Always returned; never raises for verification failures (those
        become ``status == "rejected"``). Programming errors (e.g.
        ``target_path`` parent doesn't exist *and* can't be created)
        still propagate.
    """
    now = _utcnow_iso()
    prior_sha = current_cfg.config_sha256 if current_cfg is not None else None

    try:
        text = bundle_path.read_text(encoding="utf-8")
    except OSError as exc:
        return InstallResult(
            status=STATUS_REJECTED,
            reason=f"unreadable bundle: {exc}",
            unit_id=unit_id,
            applied_sha=None,
            prior_sha=prior_sha,
            reload_class=None,
            applied_at=now,
        )

    # Envelope + filename verification.
    try:
        bundle = config_bundle.parse(text)
        config_bundle.verify_filename_matches(bundle, bundle_path.name)
    except config_bundle.BundleError as exc:
        return InstallResult(
            status=STATUS_REJECTED,
            reason=f"bundle envelope: {exc}",
            unit_id=unit_id,
            applied_sha=None,
            prior_sha=prior_sha,
            reload_class=None,
            applied_at=now,
        )

    if bundle.unit_id != unit_id:
        return InstallResult(
            status=STATUS_REJECTED,
            reason=(
                f"unit_id mismatch: bundle is for {bundle.unit_id!r}, "
                f"this unit is {unit_id!r}"
            ),
            unit_id=unit_id,
            applied_sha=None,
            prior_sha=prior_sha,
            reload_class=None,
            applied_at=now,
        )

    # Schema-validate the body. This also surfaces a body-vs-header
    # unit_id mismatch (the loader strictly requires the body's unit_id).
    try:
        candidate = _load_body_via_temp(bundle.body)
    except build_config.BuildConfigError as exc:
        return InstallResult(
            status=STATUS_REJECTED,
            reason=f"schema: {exc}",
            unit_id=unit_id,
            applied_sha=None,
            prior_sha=prior_sha,
            reload_class=None,
            applied_at=now,
        )

    if candidate.unit_id != unit_id:
        return InstallResult(
            status=STATUS_REJECTED,
            reason=(
                f"body unit_id {candidate.unit_id!r} != this unit {unit_id!r}"
            ),
            unit_id=unit_id,
            applied_sha=None,
            prior_sha=prior_sha,
            reload_class=None,
            applied_at=now,
        )

    # Already on the right config? No-op rather than rewriting.
    if current_cfg is not None and candidate.config_sha256 == prior_sha:
        return InstallResult(
            status=STATUS_NOOP,
            reason="already on this config (no-op)",
            unit_id=unit_id,
            applied_sha=candidate.config_sha256,
            prior_sha=prior_sha,
            reload_class=None,
            applied_at=now,
            schema_version=candidate.schema_version,
        )

    # Classify and reject firmware-required changes outright -- those
    # need a re-flash, not a USB-stick swap.
    reload_class: str | None = None
    leaf_count = 0
    if current_cfg is not None:
        diff = build_config.diff_reload_classes(current_cfg, candidate)
        reload_class = diff.worst
        leaf_count = len(diff.changed)
        if diff.firmware_required:
            return InstallResult(
                status=STATUS_REJECTED,
                reason=(
                    f"firmware_required change rejected (leaves: "
                    f"{', '.join(diff.changed)})"
                ),
                unit_id=unit_id,
                applied_sha=None,
                prior_sha=prior_sha,
                reload_class="firmware_required",
                applied_at=now,
                schema_version=candidate.schema_version,
            )

    # Safe to land. Atomic replace.
    try:
        _atomic_write(target_path, bundle.body)
    except OSError as exc:
        return InstallResult(
            status=STATUS_REJECTED,
            reason=f"write failed: {exc}",
            unit_id=unit_id,
            applied_sha=None,
            prior_sha=prior_sha,
            reload_class=reload_class,
            applied_at=now,
            schema_version=candidate.schema_version,
        )

    if reload_class is None:
        reason = f"fresh apply (no prior config; {leaf_count} leaves)"
    elif reload_class == "live":
        reason = f"live reload ({leaf_count} leaves changed)"
    else:
        reason = f"restart required ({leaf_count} leaves changed)"
    return InstallResult(
        status=STATUS_APPLIED,
        reason=reason,
        unit_id=unit_id,
        applied_sha=candidate.config_sha256,
        prior_sha=prior_sha,
        reload_class=reload_class,
        applied_at=now,
        schema_version=candidate.schema_version,
    )


def write_result_file(result: InstallResult, dst_dir: Path) -> Path:
    """Write ``lifetrac-config-result.json`` into ``dst_dir``.

    The dst is typically the same USB-stick mount the bundle came from,
    so the operator unplugging the stick has a record of what happened.
    Atomic via temp + replace so a yanked stick can't leave a truncated
    file behind.
    """
    dst_dir.mkdir(parents=True, exist_ok=True)
    target = dst_dir / RESULT_FILENAME
    fd, tmp = tempfile.mkstemp(
        prefix=RESULT_FILENAME + ".", suffix=".new", dir=str(dst_dir)
    )
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(result.to_json_dict(), f, indent=2, sort_keys=True)
            f.write("\n")
        os.replace(tmp, target)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise
    return target


def process_mount(
    mount_dir: Path,
    *,
    target_path: Path,
    unit_id: str,
    current_cfg: build_config.BuildConfig | None = None,
) -> list[InstallResult]:
    """Apply every bundle on a mount that targets this unit.

    Convenience wrapper for the daemon's polling loop:
    :func:`discover_bundles` + :func:`apply_bundle` per match +
    :func:`write_result_file` next to the last applied bundle. The
    ``current_cfg`` is **not** updated between bundles -- if the stick
    carries two bundles for the same unit, both diff against the same
    in-memory baseline; the daemon's caller is responsible for
    triggering a watcher reload after the call returns.
    """
    candidates = discover_bundles(mount_dir, unit_id)
    results: list[InstallResult] = []
    for bundle_path in candidates:
        result = apply_bundle(
            bundle_path,
            target_path=target_path,
            unit_id=unit_id,
            current_cfg=current_cfg,
        )
        results.append(result)
        try:
            write_result_file(result, mount_dir)
        except OSError as exc:
            log.warning("installer_daemon: result file write failed: %s", exc)
    return results


__all__ = [
    "InstallResult",
    "InstallerError",
    "RESULT_FILENAME",
    "RESULT_VERSION",
    "STATUSES",
    "STATUS_APPLIED",
    "STATUS_DEFERRED",
    "STATUS_NOOP",
    "STATUS_REJECTED",
    "apply_bundle",
    "discover_bundles",
    "process_mount",
    "write_result_file",
]
