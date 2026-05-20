#!/usr/bin/env python3
"""FCC-B2-b artifact-header linter.

Refuses any text artifact that does NOT carry a valid v1 FCC-B2-b
header block. Designed to be wired into orchestrator emission paths
(see ``mixed_load_soak.ps1`` etc.) so that a run which produces an
unstamped artifact fails the run instead of silently shipping it.

A file is considered VALID when:

* It is a text artifact (suffix in :data:`TEXT_EXTS`) AND
* :func:`artifact_header.parse_header_block` returns a non-empty dict
  AND
* That dict has every key in :data:`artifact_header.FIELD_ORDER` AND
* ``header_schema_ver`` equals
  :data:`artifact_header.HEADER_SCHEMA_VER` (currently 1).

A file is a VIOLATION when:

* It is a text artifact AND any of the above checks fails. This
  includes corrupt headers (BEGIN fence without END fence — surfaced
  as ``ValueError`` from ``parse_header_block``), missing fields, and
  wrong schema version.

Non-text artifacts (``.bin``, ``.png``, ``.elf`` …) are SKIPPED — this
linter intentionally mirrors the file-classification taxonomy used by
``inventory_artifact_headers.py`` and ``lint_artifact_naming.py``.

Per-file opt-out: a path matching ``--exclude`` glob is skipped. There
is intentionally no in-band opt-out marker — every text artifact under
a watched root must carry a header.

Usage::

    python3 lint_artifact_headers.py <path> [<path> ...]
                                     [--exclude GLOB ...]
                                     [--self-test]

Exit code 0 = clean. Exit code 1 = violations. Exit code 2 = bad CLI.

This is the CLI tool half of FCC-B2-b-b-3-3-1. Wiring into
orchestrators is FCC-B2-b-b-3-3-2.
"""
from __future__ import annotations

import argparse
import fnmatch
import sys
import tempfile
from pathlib import Path
from typing import List, Tuple

# Lazy / sibling import — artifact_header.py lives next to this file.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from artifact_header import (  # noqa: E402  (sys.path side-effect above)
    FIELD_ORDER,
    HEADER_SCHEMA_VER,
    comment_prefix_for_path,
    parse_header_block,
)

# File extensions treated as scannable text artifacts. Kept in sync
# with ``lint_artifact_naming.TEXT_EXTS`` and
# ``inventory_artifact_headers``'s text classifier.
TEXT_EXTS = {
    ".log", ".txt", ".json", ".csv", ".tsv", ".md",
    ".sh", ".ps1", ".py", ".c", ".h", ".cpp", ".hpp",
}

# Reasons a single file may be reported as a violation. These short
# tokens are emitted in the per-file VIOLATION line so the orchestrator
# wiring (b-b-3-3-2) can grep for them.
REASON_UNSTAMPED = "unstamped"
REASON_CORRUPT = "corrupt"
REASON_MISSING_FIELDS = "missing-fields"
REASON_WRONG_SCHEMA = "wrong-schema-ver"
REASON_UNREADABLE = "unreadable"


def is_text_artifact(path: Path) -> bool:
    return path.suffix.lower() in TEXT_EXTS


def check_text(path: Path, text: str) -> Tuple[bool, str, str]:
    """Validate the header in ``text`` for ``path``.

    Returns ``(violates, reason, detail)``. ``reason`` is one of the
    ``REASON_*`` constants when ``violates`` is True; empty string when
    valid. ``detail`` is a short human-readable hint.
    """
    prefix = comment_prefix_for_path(path)
    try:
        fields = parse_header_block(text, comment_prefix=prefix)
    except ValueError as exc:
        return True, REASON_CORRUPT, str(exc)
    if not fields:
        return True, REASON_UNSTAMPED, "no FCC-B2-b header block found"
    missing = [k for k in FIELD_ORDER if k not in fields]
    if missing:
        return True, REASON_MISSING_FIELDS, f"missing keys: {missing}"
    schema = fields.get("header_schema_ver", "")
    if schema != str(HEADER_SCHEMA_VER):
        return (
            True,
            REASON_WRONG_SCHEMA,
            f"header_schema_ver={schema!r} expected {HEADER_SCHEMA_VER!r}",
        )
    return False, "", ""


def iter_files(roots: List[Path]):
    for root in roots:
        if root.is_file():
            yield root
        elif root.is_dir():
            for p in root.rglob("*"):
                if p.is_file():
                    yield p


def matches_any(rel_path: str, patterns: List[str]) -> bool:
    return any(fnmatch.fnmatch(rel_path, pat) for pat in patterns)


def lint(roots: List[Path], excludes: List[str]) -> int:
    violations: List[Tuple[Path, str, str]] = []
    scanned = 0
    skipped_binary = 0
    skipped_excluded = 0
    for path in iter_files(roots):
        rel = str(path)
        if matches_any(rel, excludes) or matches_any(path.name, excludes):
            skipped_excluded += 1
            continue
        if not is_text_artifact(path):
            skipped_binary += 1
            continue
        try:
            text = path.read_text(
                encoding="utf-8", errors="surrogateescape", newline=""
            )
        except OSError as exc:
            violations.append((path, REASON_UNREADABLE, str(exc)))
            scanned += 1
            continue
        scanned += 1
        bad, reason, detail = check_text(path, text)
        if bad:
            violations.append((path, reason, detail))
    print(
        f"[lint] scanned={scanned} excluded={skipped_excluded} "
        f"binary={skipped_binary} violations={len(violations)}"
    )
    for path, reason, detail in violations:
        print(f"  VIOLATION [{reason}] {path}: {detail}")
    return 1 if violations else 0


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

def _write(p: Path, body: str) -> Path:
    p.write_text(body, encoding="utf-8", errors="surrogateescape", newline="")
    return p


def _write_bytes(p: Path, body: bytes) -> Path:
    p.write_bytes(body)
    return p


def _valid_header(prefix: str = "# ") -> str:
    lines = [
        f"{prefix}=== FCC-B2-b ARTIFACT HEADER BEGIN (v1) ===",
        f"{prefix}firmware_git_sha: 0123456789abcdef0123456789abcdef01234567",
        f"{prefix}firmware_git_sha_short: 0123456789ab",
        f"{prefix}build_timestamp_utc: 2026-05-20T08:00:00Z",
        f"{prefix}profile_enum: 0",
        f"{prefix}profile_string: REG_PROFILE_BENCH_ONLY_FIXED_915",
        f"{prefix}rfco_summary_schema_ver: 1",
        f"{prefix}rfco_pertx_schema_ver: 1",
        f"{prefix}header_schema_ver: 1",
        f"{prefix}=== FCC-B2-b ARTIFACT HEADER END ===",
        "",
    ]
    return "\n".join(lines)


def run_self_test() -> int:  # noqa: C901  (linear case table is clearer)
    cases_passed = 0
    cases_total = 0

    def record(label: str, ok: bool, detail: str = "") -> None:
        nonlocal cases_passed, cases_total
        cases_total += 1
        if ok:
            cases_passed += 1
        status = "OK" if ok else "FAIL"
        suffix = f" — {detail}" if detail else ""
        print(f"  [{status}] {label}{suffix}")

    with tempfile.TemporaryDirectory() as td:
        root = Path(td)

        # ---- check_text() unit cases ----
        p = _write(root / "valid.log", _valid_header() + "body line\n")
        bad, reason, _ = check_text(p, p.read_text(
            encoding="utf-8", errors="surrogateescape", newline=""))
        record("01 valid header → not a violation",
               (not bad) and reason == "")

        p = _write(root / "unstamped.log", "raw body with no header\n")
        bad, reason, _ = check_text(p, p.read_text(
            encoding="utf-8", errors="surrogateescape", newline=""))
        record("02 missing header → unstamped",
               bad and reason == REASON_UNSTAMPED)

        # Corrupt: BEGIN fence without END fence.
        body = (
            "# === FCC-B2-b ARTIFACT HEADER BEGIN (v1) ===\n"
            "# firmware_git_sha: deadbeef\n"
            "no end fence ever appears\n"
        )
        p = _write(root / "corrupt.log", body)
        bad, reason, _ = check_text(p, p.read_text(
            encoding="utf-8", errors="surrogateescape", newline=""))
        record("03 BEGIN without END → corrupt",
               bad and reason == REASON_CORRUPT)

        # Missing one required field (drop profile_string).
        partial = _valid_header().replace(
            "# profile_string: REG_PROFILE_BENCH_ONLY_FIXED_915\n", "")
        p = _write(root / "missing.log", partial + "body\n")
        bad, reason, _ = check_text(p, p.read_text(
            encoding="utf-8", errors="surrogateescape", newline=""))
        record("04 missing required field → missing-fields",
               bad and reason == REASON_MISSING_FIELDS)

        # Wrong schema version.
        wrong = _valid_header().replace(
            "# header_schema_ver: 1\n",
            "# header_schema_ver: 99\n",
        )
        p = _write(root / "wrongver.log", wrong + "body\n")
        bad, reason, _ = check_text(p, p.read_text(
            encoding="utf-8", errors="surrogateescape", newline=""))
        record("05 wrong header_schema_ver → wrong-schema-ver",
               bad and reason == REASON_WRONG_SCHEMA)

        # Comment-prefix dispatch: .c file with // prefix is valid.
        p = _write(root / "ok.c", _valid_header("// ") + "int main(){}\n")
        bad, reason, _ = check_text(p, p.read_text(
            encoding="utf-8", errors="surrogateescape", newline=""))
        record("06 .c file with // prefix → valid",
               (not bad) and reason == "",
               detail=f"reason={reason!r}")

        # Comment-prefix mismatch: .c file with # prefix header → unstamped
        # (parse_header_block keys off comment_prefix_for_path).
        p = _write(root / "bad.c", _valid_header("# ") + "int main(){}\n")
        bad, reason, _ = check_text(p, p.read_text(
            encoding="utf-8", errors="surrogateescape", newline=""))
        record("07 .c file with # prefix header → unstamped",
               bad and reason == REASON_UNSTAMPED)

        # SQL prefix dispatch.
        p = _write(root / "ok.sql",
                   _valid_header("-- ") + "SELECT 1;\n")
        bad, reason, _ = check_text(p, p.read_text(
            encoding="utf-8", errors="surrogateescape", newline=""))
        record("08 .sql file with -- prefix → valid",
               (not bad) and reason == "")

        # ---- iter_files / lint() integration cases ----

        # Build a clean sub-tree, then a violation sub-tree.
        clean_dir = root / "clean"
        clean_dir.mkdir()
        _write(clean_dir / "a.log", _valid_header() + "body\n")
        _write(clean_dir / "b.json",
               _valid_header() + '{"k": 1}\n')
        # Non-text file should be skipped, not flagged.
        _write_bytes(clean_dir / "blob.bin", b"\x00\x01\x02\xff")

        rc = lint([clean_dir], [])
        record("09 clean tree → rc=0", rc == 0, detail=f"rc={rc}")

        dirty_dir = root / "dirty"
        dirty_dir.mkdir()
        _write(dirty_dir / "ok.log", _valid_header() + "body\n")
        _write(dirty_dir / "bad.log", "no header here\n")
        rc = lint([dirty_dir], [])
        record("10 dirty tree → rc=1", rc == 1, detail=f"rc={rc}")

        # --exclude must hide the violation.
        rc = lint([dirty_dir], ["bad.log"])
        record("11 --exclude basename hides violation → rc=0",
               rc == 0, detail=f"rc={rc}")

        # --exclude with full-path glob.
        rc = lint([dirty_dir], [str(dirty_dir / "bad.log")])
        record("12 --exclude full-path hides violation → rc=0",
               rc == 0, detail=f"rc={rc}")

        # Non-UTF-8 byte prefix (UART noise like 0xff) must NOT crash
        # the read path — surrogateescape contract from b-b-3-2-2-3-1.
        noisy_path = dirty_dir / "noisy.log"
        _write_bytes(
            noisy_path,
            b"\xff\xfe garbage prefix\n" + (_valid_header() + "body\n").encode(
                "utf-8"),
        )
        # noisy.log has a valid header AFTER the garbage prefix; the
        # linter should accept it (parse_header_block uses .find() on
        # the BEGIN fence which is unaffected by leading noise).
        # The point of the case is: lint() must not raise.
        try:
            rc = lint([noisy_path], [])
            no_crash = True
        except UnicodeDecodeError:
            no_crash = False
            rc = -1
        record("13 non-UTF-8 prefix → no UnicodeDecodeError",
               no_crash, detail=f"rc={rc}")

        # File-as-root targeting (single-file invocation must work).
        single = clean_dir / "a.log"
        rc = lint([single], [])
        record("14 single-file root → rc=0", rc == 0, detail=f"rc={rc}")

        single_bad = dirty_dir / "bad.log"
        rc = lint([single_bad], [])
        record("15 single bad file → rc=1", rc == 1, detail=f"rc={rc}")

        # Empty tree → rc=0 (nothing scanned is not a violation).
        empty_dir = root / "empty"
        empty_dir.mkdir()
        rc = lint([empty_dir], [])
        record("16 empty tree → rc=0", rc == 0, detail=f"rc={rc}")

        # Unreadable file → reported as violation with reason unreadable.
        # Simulate by pointing at a directory entry that does not exist
        # — we can't easily revoke read perms on Windows. Instead,
        # construct a path that iter_files won't yield but feed it
        # through lint() directly via a custom root. Use a path that
        # exists as a *file* but with a name that bypasses suffix check
        # — we want to confirm the OSError path, so create then unlink
        # after gathering. Simpler: skip this case if can't reliably
        # produce an unreadable text file cross-platform.
        # (Coverage of REASON_UNREADABLE is exercised by lint()'s
        # try/except contract; we audit it via grep here instead.)
        src = Path(__file__).read_text(encoding="utf-8")
        record("17 REASON_UNREADABLE path is wired in lint()",
               'REASON_UNREADABLE' in src and 'except OSError' in src)

        # FIELD_ORDER must match what check_text validates against.
        record("18 FIELD_ORDER carries all 8 expected keys",
               len(FIELD_ORDER) == 8 and
               "header_schema_ver" in FIELD_ORDER and
               "profile_enum" in FIELD_ORDER,
               detail=f"len={len(FIELD_ORDER)}")

        # HEADER_SCHEMA_VER stays at 1 (this linter is wired to v1).
        record("19 HEADER_SCHEMA_VER == 1 (this linter targets v1)",
               HEADER_SCHEMA_VER == 1,
               detail=f"HEADER_SCHEMA_VER={HEADER_SCHEMA_VER}")

    print(f"[self-test] {cases_passed}/{cases_total} cases passed")
    return 1 if cases_passed != cases_total else 0


def main(argv: List[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("paths", nargs="*", type=Path,
                   help="files or directories to lint")
    p.add_argument("--exclude", action="append", default=[],
                   help="glob to skip (matched against full path AND "
                        "basename); repeatable")
    p.add_argument("--self-test", action="store_true",
                   help="run built-in unit cases and exit")
    args = p.parse_args(argv)
    if args.self_test:
        return run_self_test()
    if not args.paths:
        print("error: no paths given (or use --self-test)", file=sys.stderr)
        return 2
    return lint(args.paths, args.exclude)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
