#!/usr/bin/env python3
"""FCC-B2 naming linter for bench-evidence and orchestrator artifacts.

Refuses any artifact text file that contains the imprecise tokens
`airtime_us` or `dwell_us` without ALSO containing at least one of the
canonical replacements:

    qos_used_us_1s
    legal_dwell_used_us_10s
    legal_dwell_used_us_20s

Rationale: `airtime_us` collapses the QoS-window vs legal-dwell-window
distinction that the FCC §15.247 RFCO surface needs. Same for
`dwell_us`. Any artifact that mentions the old token MUST disambiguate
in the same file. See LifeTrac-v25/TODO.md item FCC-B2.

Per-file opt-out (for legitimate sources-of-truth that name the rule
itself): include the marker `LINTER_ALLOW_RAW_AIRTIME_DWELL_US` in the
file body, OR pass the file/path under --exclude.

Usage:
    python3 lint_artifact_naming.py <path> [<path> ...]
                                   [--exclude GLOB ...]
                                   [--self-test]

Exit code 0 = clean. Exit code 1 = violations. Exit code 2 = bad CLI.
"""
from __future__ import annotations

import argparse
import fnmatch
import sys
from pathlib import Path

OLD_TOKENS = ("airtime_us", "dwell_us")
CANONICAL_TOKENS = (
    "qos_used_us_1s",
    "legal_dwell_used_us_10s",
    "legal_dwell_used_us_20s",
)
OPT_OUT_MARKER = "LINTER_ALLOW_RAW_AIRTIME_DWELL_US"

# File extensions treated as scannable text artifacts.
TEXT_EXTS = {
    ".log", ".txt", ".json", ".csv", ".tsv", ".md",
    ".sh", ".ps1", ".py", ".c", ".h", ".cpp", ".hpp",
}


def is_text_artifact(path: Path) -> bool:
    return path.suffix.lower() in TEXT_EXTS


def file_violates(text: str) -> tuple[bool, list[str]]:
    """Return (violates, found_old_tokens). True if old token present
    without any canonical replacement and without the opt-out marker."""
    if OPT_OUT_MARKER in text:
        return False, []
    found_old = [t for t in OLD_TOKENS if t in text]
    if not found_old:
        return False, []
    has_canonical = any(t in text for t in CANONICAL_TOKENS)
    return (not has_canonical), found_old


def iter_files(roots: list[Path]):
    for root in roots:
        if root.is_file():
            yield root
        elif root.is_dir():
            for p in root.rglob("*"):
                if p.is_file():
                    yield p


def matches_any(rel_path: str, patterns: list[str]) -> bool:
    return any(fnmatch.fnmatch(rel_path, pat) for pat in patterns)


def lint(roots: list[Path], excludes: list[str]) -> int:
    violations: list[tuple[Path, list[str]]] = []
    scanned = 0
    skipped_binary = 0
    skipped_excluded = 0
    skipped_optout = 0
    for path in iter_files(roots):
        rel = str(path)
        if matches_any(rel, excludes) or matches_any(path.name, excludes):
            skipped_excluded += 1
            continue
        if not is_text_artifact(path):
            skipped_binary += 1
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="replace")
        except OSError as exc:
            print(f"[WARN] cannot read {path}: {exc}", file=sys.stderr)
            continue
        scanned += 1
        if OPT_OUT_MARKER in text:
            skipped_optout += 1
            continue
        bad, found = file_violates(text)
        if bad:
            violations.append((path, found))
    print(
        f"[lint] scanned={scanned} excluded={skipped_excluded} "
        f"binary={skipped_binary} optout={skipped_optout} "
        f"violations={len(violations)}"
    )
    for path, found in violations:
        print(
            f"  VIOLATION {path}: contains {found} without any of "
            f"{list(CANONICAL_TOKENS)} and no {OPT_OUT_MARKER} marker"
        )
    return 1 if violations else 0


def run_self_test() -> int:
    cases = [
        # (label, text, expected_violates)
        ("empty", "", False),
        ("only_canonical", "qos_used_us_1s = 42", False),
        ("only_old_airtime_no_canonical", "airtime_us=123", True),
        ("only_old_dwell_no_canonical", "dwell_us=456", True),
        (
            "old_airtime_with_qos_canonical",
            "airtime_us=123 qos_used_us_1s=99",
            False,
        ),
        (
            "old_dwell_with_legal_10s_canonical",
            "dwell_us=456 legal_dwell_used_us_10s=400",
            False,
        ),
        (
            "old_dwell_with_legal_20s_canonical",
            "dwell_us=456 legal_dwell_used_us_20s=800",
            False,
        ),
        ("optout_marker_overrides", "airtime_us=1 " + OPT_OUT_MARKER, False),
        (
            "both_old_tokens_no_canonical",
            "airtime_us=1 dwell_us=2",
            True,
        ),
        (
            "both_old_tokens_one_canonical_ok",
            "airtime_us=1 dwell_us=2 qos_used_us_1s=3",
            False,
        ),
        (
            "substring_only_does_not_count_as_canonical_for_old_present",
            # 'airtime_used_us_1s' is NOT a canonical token; bare 'airtime_us'
            # appears, so this still violates.
            "airtime_us=1 airtime_used_us_1s=2",
            True,
        ),
    ]
    fails = 0
    for label, text, expected in cases:
        bad, _found = file_violates(text)
        status = "OK" if bad == expected else "FAIL"
        if bad != expected:
            fails += 1
        print(f"  [{status}] {label}: violates={bad} expected={expected}")
    print(f"[self-test] {len(cases) - fails}/{len(cases)} cases passed")
    return 1 if fails else 0


def main(argv: list[str]) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("paths", nargs="*", type=Path,
                   help="files or directories to lint")
    p.add_argument("--exclude", action="append", default=[],
                   help="glob to skip (matched against full path AND basename)")
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
