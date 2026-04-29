"""Installer-bundle format for build-config delivery (Round 29b / BC-10).

A bundle is a plain UTF-8 text file with a header block (one comment line per
field) followed by a sentinel line and the validated TOML body verbatim::

    # LifeTrac v25 build-config installer bundle
    # bundle_version: 1
    # unit_id: lifetrac-001
    # sha256: <64 hex chars of body bytes>
    # generator: <tool> <version>
    # created: 2026-04-29T12:34:56Z
    # --- body ---
    <validated TOML, byte-for-byte>

Why this shape:

* Plain text, no archive container -- an operator can ``cat`` it on a
  field laptop and see what's about to be applied. Anti-corruption SHA
  is also ``sha256sum`` / ``Get-FileHash`` -compatible if you strip the
  header by hand.
* Filename carries ``unit_id`` and the first 8 hex chars of the body
  SHA so a USB stick with several bundles is sortable / disambiguatable
  without opening anything.
* Filename ``unit_id`` is cross-checked against the body's parsed
  ``unit_id`` (BC-10 contract). Two layers of "is this for this
  tractor?" defence cost essentially nothing.
* SHA covers the body bytes only, not the header. That way the header
  can carry the SHA itself without a chicken-and-egg problem, and a
  base UI re-issuing the same TOML always produces the same body SHA.

Pure stdlib. No third-party deps. Used by ``tools/lifetrac-config`` and
by the base ``/config/download`` route.
"""

from __future__ import annotations

import hashlib
import re
from dataclasses import dataclass
from datetime import datetime, timezone

BUNDLE_VERSION = 1
BODY_SENTINEL = "# --- body ---"
FILENAME_RE = re.compile(
    r"^lifetrac-config-(?P<unit_id>[a-z0-9-]{3,32})-(?P<sha8>[0-9a-f]{8})\.toml$"
)
_HEADER_LINE_RE = re.compile(r"^#\s*([A-Za-z_][A-Za-z0-9_]*)\s*:\s*(.*)$")
_SHA_RE = re.compile(r"^[0-9a-f]{64}$")
_UNIT_ID_RE = re.compile(r"^[a-z0-9-]{3,32}$")


class BundleError(ValueError):
    """Raised when a bundle is malformed, mis-named, or fails verification."""


@dataclass(frozen=True)
class Bundle:
    """Parsed installer bundle.

    ``body`` is the validated TOML text (the bytes the SHA covers). The
    caller is responsible for handing ``body`` to ``build_config`` for
    schema validation -- this module only enforces the bundle envelope.
    """

    bundle_version: int
    unit_id: str
    sha256: str
    generator: str
    created: str
    body: str

    @property
    def sha8(self) -> str:
        return self.sha256[:8]

    @property
    def filename(self) -> str:
        return f"lifetrac-config-{self.unit_id}-{self.sha8}.toml"


def body_sha256(body: str) -> str:
    """SHA-256 over the body bytes (UTF-8). Stable across platforms."""
    return hashlib.sha256(body.encode("utf-8")).hexdigest()


def make_bundle(
    body: str,
    unit_id: str,
    *,
    generator: str = "lifetrac-config",
    created: str | None = None,
) -> Bundle:
    """Construct a bundle from a TOML body and the unit_id it targets.

    Caller has already validated the body against the schema; this module
    does not re-validate (orthogonality: bundle envelope vs schema).
    The ``unit_id`` argument must match the ``unit_id`` parsed out of the
    body when the body is loaded -- that match is the X8-side installer's
    job to enforce, not ours.
    """
    if not _UNIT_ID_RE.match(unit_id):
        raise BundleError(f"unit_id {unit_id!r} fails pattern {_UNIT_ID_RE.pattern}")
    sha = body_sha256(body)
    if created is None:
        created = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    return Bundle(
        bundle_version=BUNDLE_VERSION,
        unit_id=unit_id,
        sha256=sha,
        generator=generator,
        created=created,
        body=body,
    )


def serialise(bundle: Bundle) -> str:
    """Serialise a bundle to its on-disk text form. Round-trips with parse()."""
    header = [
        "# LifeTrac v25 build-config installer bundle",
        f"# bundle_version: {bundle.bundle_version}",
        f"# unit_id: {bundle.unit_id}",
        f"# sha256: {bundle.sha256}",
        f"# generator: {bundle.generator}",
        f"# created: {bundle.created}",
        BODY_SENTINEL,
    ]
    # Body kept verbatim; ensure newline terminator so `cat` is clean.
    body = bundle.body if bundle.body.endswith("\n") else bundle.body + "\n"
    return "\n".join(header) + "\n" + body


def parse(text: str) -> Bundle:
    """Parse + verify a serialised bundle.

    Raises :class:`BundleError` for any malformed envelope: unknown
    ``bundle_version``, missing required header field, malformed SHA,
    body SHA mismatch, missing ``# --- body ---`` sentinel.
    """
    if BODY_SENTINEL not in text:
        raise BundleError("missing body sentinel line")
    header_text, _, body = text.partition(BODY_SENTINEL + "\n")
    if not body and (BODY_SENTINEL + "\r\n") in text:
        # Tolerate Windows line endings on the sentinel only; the body
        # itself is preserved byte-for-byte in whatever form the bundler
        # wrote it in (operators editing TOML on Windows is real life).
        header_text, _, body = text.partition(BODY_SENTINEL + "\r\n")
    fields: dict[str, str] = {}
    for line in header_text.splitlines():
        if not line.strip():
            continue
        if line.startswith("# ") and ":" not in line:
            continue  # banner line
        m = _HEADER_LINE_RE.match(line)
        if not m:
            continue
        fields[m.group(1)] = m.group(2).strip()

    for required in ("bundle_version", "unit_id", "sha256", "generator", "created"):
        if required not in fields:
            raise BundleError(f"header missing required field {required!r}")

    try:
        bundle_version = int(fields["bundle_version"])
    except ValueError as exc:
        raise BundleError(f"bundle_version not an integer: {fields['bundle_version']!r}") from exc
    if bundle_version != BUNDLE_VERSION:
        raise BundleError(
            f"unsupported bundle_version {bundle_version}; this tool speaks {BUNDLE_VERSION}"
        )

    unit_id = fields["unit_id"]
    if not _UNIT_ID_RE.match(unit_id):
        raise BundleError(f"unit_id {unit_id!r} fails pattern {_UNIT_ID_RE.pattern}")

    sha = fields["sha256"].lower()
    if not _SHA_RE.match(sha):
        raise BundleError(f"sha256 {sha!r} not 64 hex chars")

    actual = body_sha256(body)
    if actual != sha:
        raise BundleError(
            f"body SHA mismatch: header says {sha[:8]}..., body hashes to {actual[:8]}..."
        )

    return Bundle(
        bundle_version=bundle_version,
        unit_id=unit_id,
        sha256=sha,
        generator=fields["generator"],
        created=fields["created"],
        body=body,
    )


def parse_filename(name: str) -> tuple[str, str]:
    """Return ``(unit_id, sha8)`` parsed from a bundle filename.

    Raises :class:`BundleError` if the filename doesn't match the
    documented pattern. The X8 installer uses this to glob for
    ``lifetrac-config-*.toml`` candidates and then cross-check the
    parsed ``unit_id`` against the unit it's running on before reading
    the file at all.
    """
    m = FILENAME_RE.match(name)
    if not m:
        raise BundleError(f"filename {name!r} does not match {FILENAME_RE.pattern}")
    return m.group("unit_id"), m.group("sha8")


def verify_filename_matches(bundle: Bundle, filename: str) -> None:
    """Cross-check a parsed bundle against its on-disk filename.

    Both ``unit_id`` and the first 8 hex chars of the SHA must agree.
    A mismatch means the bundle was renamed (deliberately or by a
    careless copy), which is suspicious enough to refuse.
    """
    unit_id, sha8 = parse_filename(filename)
    if unit_id != bundle.unit_id:
        raise BundleError(
            f"filename unit_id {unit_id!r} != header unit_id {bundle.unit_id!r}"
        )
    if sha8 != bundle.sha8:
        raise BundleError(
            f"filename sha8 {sha8!r} != header sha8 {bundle.sha8!r}"
        )


__all__ = [
    "BUNDLE_VERSION",
    "BODY_SENTINEL",
    "FILENAME_RE",
    "Bundle",
    "BundleError",
    "body_sha256",
    "make_bundle",
    "parse",
    "parse_filename",
    "serialise",
    "verify_filename_matches",
]
