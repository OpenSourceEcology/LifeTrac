#!/usr/bin/env python3
"""FCC-B2-b artifact-header stamping.

Single source of truth for the metadata block prepended to every
bench-evidence / orchestrator artifact emitted by the LifeTrac-v25
DESIGN-CONTROLLER pipeline. Required fields per TODO.md item FCC-B2:

    firmware_git_sha          full 40-char SHA of HEAD
    firmware_git_sha_short    first 12 chars (matches `git log --abbrev=12`)
    build_timestamp_utc       ISO-8601 with trailing 'Z' (no offset / no µs)
    profile_enum              numeric REG_PROFILE_* id (0..255)
    profile_string            symbolic name harvested from host_cfg_keys.h
    rfco_summary_schema_ver   harvested from HOST_RFCO_SUMMARY_SCHEMA_VER
    rfco_pertx_schema_ver     harvested from HOST_RFCO_PERTX_SCHEMA_VER
    header_schema_ver         this module's own format version

Format: a fenced comment block that is safe to prepend to any text
artifact whose line-comment prefix is configurable (default '# ' for
shell / Python / Make / log / md; '// ' for C / C++; '-- ' for SQL).
JSON / CSV are NOT supported in-band by this module — those callers
must write a sidecar `<artifact>.header.yaml` (left to FCC-B2-b-b-2).

Example block (comment_prefix='# '):

    # === FCC-B2-b ARTIFACT HEADER BEGIN (v1) ===
    # firmware_git_sha: abcdef0123456789abcdef0123456789abcdef01
    # firmware_git_sha_short: abcdef012345
    # build_timestamp_utc: 2026-05-19T18:42:07Z
    # profile_enum: 1
    # profile_string: REG_PROFILE_FCC_15_247_FHSS_50CH_BW250
    # rfco_summary_schema_ver: 1
    # rfco_pertx_schema_ver: 1
    # header_schema_ver: 1
    # === FCC-B2-b ARTIFACT HEADER END ===

Naming-linter compatibility: this header NEVER contains the imprecise
tokens `airtime_us` or `dwell_us`, so it does not need the opt-out
marker from `lint_artifact_naming.py`. (This module itself names those
tokens while documenting the rule, so it carries the marker
``LINTER_ALLOW_RAW_AIRTIME_DWELL_US`` for the FCC-B2 linter.)

Pipeline integration is out of scope for this increment (FCC-B2-b-b-1
is module + self-test only). The CLI exposes `--self-test`,
`--harvest`, `--stamp`, and `--parse` so downstream callers can drive
it once b-b-2 lands.

Exit codes: 0 = ok, 1 = self-test or stamp/parse failure, 2 = bad CLI.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import io
import re
import subprocess
import sys
from pathlib import Path
from typing import Dict, Mapping, Optional, Tuple

HEADER_SCHEMA_VER = 1
BEGIN_FENCE = f"=== FCC-B2-b ARTIFACT HEADER BEGIN (v{HEADER_SCHEMA_VER}) ==="
END_FENCE = "=== FCC-B2-b ARTIFACT HEADER END ==="

# Field order is significant for round-trip + visual diff stability.
FIELD_ORDER: Tuple[str, ...] = (
    "firmware_git_sha",
    "firmware_git_sha_short",
    "build_timestamp_utc",
    "profile_enum",
    "profile_string",
    "rfco_summary_schema_ver",
    "rfco_pertx_schema_ver",
    "header_schema_ver",
)

# Canonical comment-prefix table. Caller may override.
COMMENT_PREFIX_BY_SUFFIX: Dict[str, str] = {
    ".c": "// ",
    ".h": "// ",
    ".cpp": "// ",
    ".hpp": "// ",
    ".sql": "-- ",
}
DEFAULT_COMMENT_PREFIX = "# "

# Regexes used by the C-header harvester. Anchored to '#define' to
# avoid matching commented-out lines / struct members.
_SCHEMA_RE = re.compile(
    r"^\s*#\s*define\s+(HOST_RFCO_(?:SUMMARY|PERTX)_SCHEMA_VER)\s+(\d+)U?\b",
    re.MULTILINE,
)
_PROFILE_RE = re.compile(
    r"^\s*#\s*define\s+(REG_PROFILE_[A-Z0-9_]+)\s+(\d+)U?\b",
    re.MULTILINE,
)


# ---------------------------------------------------------------------------
# Harvesters
# ---------------------------------------------------------------------------

def harvest_firmware_schema_vers(firmware_root: Path) -> Dict[str, int]:
    """Return a dict like {'HOST_RFCO_SUMMARY_SCHEMA_VER': 1, ...}.

    Drift gate: raises ``RuntimeError`` if either macro is missing —
    the stamper refuses to fabricate a schema version, so a renamed
    macro fails loudly instead of stamping ``unknown``.
    """
    out: Dict[str, int] = {}
    inc = firmware_root / "include"
    if not inc.is_dir():
        raise RuntimeError(f"include/ not found under {firmware_root}")
    for header in inc.glob("*.h"):
        try:
            text = header.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        for m in _SCHEMA_RE.finditer(text):
            out[m.group(1)] = int(m.group(2))
    required = ("HOST_RFCO_SUMMARY_SCHEMA_VER", "HOST_RFCO_PERTX_SCHEMA_VER")
    missing = [k for k in required if k not in out]
    if missing:
        raise RuntimeError(
            f"required schema macro(s) not found in {inc}: {missing}"
        )
    return out


def harvest_profile_table(firmware_root: Path) -> Dict[int, str]:
    """Return {enum_value: symbolic_name} for every REG_PROFILE_* macro.

    REG_PROFILE_MAX is excluded (it aliases another profile and would
    otherwise overwrite the real symbolic name at the same enum value).
    """
    keys = firmware_root / "include" / "host_cfg_keys.h"
    if not keys.is_file():
        raise RuntimeError(f"host_cfg_keys.h not found at {keys}")
    text = keys.read_text(encoding="utf-8", errors="replace")
    out: Dict[int, str] = {}
    for m in _PROFILE_RE.finditer(text):
        name = m.group(1)
        if name == "REG_PROFILE_MAX":
            continue
        out[int(m.group(2))] = name
    if not out:
        raise RuntimeError("no REG_PROFILE_* macros found")
    return out


def current_git_sha(repo_root: Path) -> Tuple[str, str]:
    """Return (full_sha, short_sha). Falls back to ('unknown','unknown')
    when git is not available or the directory is not a repo."""
    try:
        full = subprocess.run(
            ["git", "rev-parse", "HEAD"],
            cwd=str(repo_root),
            check=True,
            capture_output=True,
            text=True,
            timeout=10,
        ).stdout.strip()
        if not re.fullmatch(r"[0-9a-f]{40}", full):
            return ("unknown", "unknown")
        return (full, full[:12])
    except (subprocess.SubprocessError, FileNotFoundError, OSError):
        return ("unknown", "unknown")


def current_utc_timestamp(now: Optional[_dt.datetime] = None) -> str:
    """ISO-8601 second-precision UTC timestamp with trailing 'Z'."""
    if now is None:
        now = _dt.datetime.now(_dt.timezone.utc)
    if now.tzinfo is None:
        now = now.replace(tzinfo=_dt.timezone.utc)
    return now.astimezone(_dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


# ---------------------------------------------------------------------------
# Field assembly
# ---------------------------------------------------------------------------

def build_header_fields(
    *,
    firmware_root: Path,
    repo_root: Path,
    profile_enum: int,
    now: Optional[_dt.datetime] = None,
) -> Dict[str, str]:
    """Assemble the canonical header fields dict.

    Values are all strings (the wire format is comment lines, and YAML
    round-trips them as strings anyway). Numeric fields are stringified
    via ``str(int)`` so callers comparing against the wire form do not
    have to second-guess formatting.
    """
    profiles = harvest_profile_table(firmware_root)
    schema = harvest_firmware_schema_vers(firmware_root)
    if profile_enum not in profiles:
        raise ValueError(
            f"profile_enum={profile_enum} not in harvested table "
            f"{sorted(profiles)}"
        )
    full_sha, short_sha = current_git_sha(repo_root)
    fields: Dict[str, str] = {
        "firmware_git_sha": full_sha,
        "firmware_git_sha_short": short_sha,
        "build_timestamp_utc": current_utc_timestamp(now),
        "profile_enum": str(int(profile_enum)),
        "profile_string": profiles[profile_enum],
        "rfco_summary_schema_ver": str(schema["HOST_RFCO_SUMMARY_SCHEMA_VER"]),
        "rfco_pertx_schema_ver": str(schema["HOST_RFCO_PERTX_SCHEMA_VER"]),
        "header_schema_ver": str(HEADER_SCHEMA_VER),
    }
    # Defensive: order is enforced at format time, but assert all keys
    # are accounted for so a future field addition can't silently drop.
    missing = set(FIELD_ORDER) - set(fields)
    extra = set(fields) - set(FIELD_ORDER)
    if missing or extra:
        raise RuntimeError(
            f"FIELD_ORDER drift: missing={missing} extra={extra}"
        )
    return fields


# ---------------------------------------------------------------------------
# Format / parse / stamp
# ---------------------------------------------------------------------------

def _validate_value(key: str, value: str) -> None:
    if "\n" in value or "\r" in value:
        raise ValueError(f"field {key!r} contains a newline: {value!r}")


def format_header_block(
    fields: Mapping[str, str],
    *,
    comment_prefix: str = DEFAULT_COMMENT_PREFIX,
) -> str:
    """Render the fenced header block as a single string ending in '\\n'.

    Field order follows ``FIELD_ORDER``; unknown keys raise so the
    canonical schema cannot drift sideways via stray fields.
    """
    if not comment_prefix.endswith(" "):
        raise ValueError("comment_prefix must end with a space")
    extra = set(fields) - set(FIELD_ORDER)
    missing = set(FIELD_ORDER) - set(fields)
    if extra or missing:
        raise ValueError(
            f"field set mismatch: missing={sorted(missing)} "
            f"extra={sorted(extra)}"
        )
    buf = io.StringIO()
    buf.write(f"{comment_prefix}{BEGIN_FENCE}\n")
    for key in FIELD_ORDER:
        val = fields[key]
        _validate_value(key, val)
        buf.write(f"{comment_prefix}{key}: {val}\n")
    buf.write(f"{comment_prefix}{END_FENCE}\n")
    return buf.getvalue()


def parse_header_block(
    text: str,
    *,
    comment_prefix: str = DEFAULT_COMMENT_PREFIX,
) -> Dict[str, str]:
    """Extract the first FCC-B2-b header from ``text``.

    Returns ``{}`` when no block is found. Raises ``ValueError`` when a
    BEGIN fence is found without a matching END (corrupt artifact).
    """
    begin_line = f"{comment_prefix}{BEGIN_FENCE}"
    end_line = f"{comment_prefix}{END_FENCE}"
    begin_idx = text.find(begin_line)
    if begin_idx < 0:
        return {}
    end_idx = text.find(end_line, begin_idx + len(begin_line))
    if end_idx < 0:
        raise ValueError("BEGIN fence found without matching END fence")
    body = text[begin_idx + len(begin_line):end_idx]
    fields: Dict[str, str] = {}
    for raw in body.splitlines():
        line = raw.strip()
        if not line:
            continue
        if not line.startswith(comment_prefix.strip()):
            continue
        line = line[len(comment_prefix.strip()):].lstrip()
        if ":" not in line:
            continue
        key, _, val = line.partition(":")
        fields[key.strip()] = val.strip()
    return fields


def stamp_text(
    text: str,
    fields: Mapping[str, str],
    *,
    comment_prefix: str = DEFAULT_COMMENT_PREFIX,
) -> str:
    """Return ``text`` with the header block prepended.

    Idempotency rule: if ``text`` already contains a header block whose
    field set is byte-equal to ``fields``, the input is returned
    unchanged. If the existing block disagrees (any field differs),
    ``ValueError`` is raised — re-stamping with new metadata MUST be an
    explicit unstamp + restamp by the caller, never silent.
    """
    existing = parse_header_block(text, comment_prefix=comment_prefix)
    if existing:
        if existing == dict(fields):
            return text
        diff = {
            k: (existing.get(k), fields.get(k))
            for k in set(existing) | set(fields)
            if existing.get(k) != fields.get(k)
        }
        raise ValueError(
            f"text already stamped with different fields; diff={diff}"
        )
    block = format_header_block(fields, comment_prefix=comment_prefix)
    # Insert a blank separator line between the header and the original
    # body when the body is non-empty.
    if text and not text.startswith("\n"):
        return block + "\n" + text
    return block + text


def comment_prefix_for_path(path: Path) -> str:
    return COMMENT_PREFIX_BY_SUFFIX.get(path.suffix.lower(), DEFAULT_COMMENT_PREFIX)


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

def _self_test_repo_paths() -> Tuple[Path, Path]:
    """Return (firmware_root, repo_root) for the self-test's harvester
    cases. Computed relative to this file so the self-test works from
    any working directory."""
    here = Path(__file__).resolve().parent  # .../LifeTrac-v25/tools
    firmware_root = (here.parent /
                     "DESIGN-CONTROLLER" / "firmware" / "murata_l072")
    repo_root = here.parent.parent  # repo root (parent of LifeTrac-v25)
    return firmware_root, repo_root


def run_self_test() -> int:
    fails = 0
    cases = 0

    def check(label: str, cond: bool, detail: str = "") -> None:
        nonlocal fails, cases
        cases += 1
        status = "OK" if cond else "FAIL"
        if not cond:
            fails += 1
        msg = f"  [{status}] {label}"
        if detail and not cond:
            msg += f" :: {detail}"
        print(msg)

    firmware_root, repo_root = _self_test_repo_paths()

    # ---- Case 1: schema harvester finds both macros ----
    schema = harvest_firmware_schema_vers(firmware_root)
    check(
        "harvest_firmware_schema_vers finds SUMMARY + PERTX",
        schema.get("HOST_RFCO_SUMMARY_SCHEMA_VER") == 1
        and schema.get("HOST_RFCO_PERTX_SCHEMA_VER") == 1,
        f"got={schema}",
    )

    # ---- Case 2: profile harvester finds all three real profiles ----
    profiles = harvest_profile_table(firmware_root)
    check(
        "harvest_profile_table finds 0/1/2 (excludes REG_PROFILE_MAX)",
        profiles.get(0) == "REG_PROFILE_BENCH_ONLY_FIXED_915"
        and profiles.get(1) == "REG_PROFILE_FCC_15_247_FHSS_50CH_BW250"
        and profiles.get(2) == "REG_PROFILE_FCC_15_247_DTS_BW500",
        f"got={profiles}",
    )

    # ---- Case 3: UTC timestamp format ----
    ts = current_utc_timestamp(
        _dt.datetime(2026, 5, 19, 18, 42, 7, tzinfo=_dt.timezone.utc))
    check(
        "current_utc_timestamp formats ISO-8601 Z",
        ts == "2026-05-19T18:42:07Z",
        f"got={ts!r}",
    )
    check(
        "current_utc_timestamp matches strict regex",
        re.fullmatch(r"\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}Z",
                     current_utc_timestamp()) is not None,
    )

    # ---- Case 4: build_header_fields end-to-end ----
    fields = build_header_fields(
        firmware_root=firmware_root,
        repo_root=repo_root,
        profile_enum=1,
        now=_dt.datetime(2026, 5, 19, 18, 42, 7, tzinfo=_dt.timezone.utc),
    )
    check(
        "build_header_fields populates every FIELD_ORDER key",
        set(fields) == set(FIELD_ORDER),
        f"missing={set(FIELD_ORDER) - set(fields)} "
        f"extra={set(fields) - set(FIELD_ORDER)}",
    )
    check(
        "build_header_fields propagates profile lookup",
        fields["profile_string"] == "REG_PROFILE_FCC_15_247_FHSS_50CH_BW250"
        and fields["profile_enum"] == "1",
    )
    check(
        "build_header_fields stamps schema_ver from harvest",
        fields["rfco_summary_schema_ver"] == "1"
        and fields["rfco_pertx_schema_ver"] == "1",
    )

    # ---- Case 5: invalid profile_enum rejected ----
    try:
        build_header_fields(
            firmware_root=firmware_root,
            repo_root=repo_root,
            profile_enum=99,
        )
        check("build_header_fields rejects unknown profile_enum", False,
              "no exception raised")
    except ValueError:
        check("build_header_fields rejects unknown profile_enum", True)

    # ---- Case 6: format / parse round-trip (default '# ' prefix) ----
    block = format_header_block(fields)
    parsed = parse_header_block(block)
    check(
        "format -> parse round-trip preserves every field",
        parsed == dict(fields),
        f"got={parsed}",
    )

    # ---- Case 7: round-trip under '// ' prefix (C/C++ artifacts) ----
    c_block = format_header_block(fields, comment_prefix="// ")
    c_parsed = parse_header_block(c_block, comment_prefix="// ")
    check(
        "format -> parse round-trip under '// ' prefix",
        c_parsed == dict(fields),
        f"got={c_parsed}",
    )

    # ---- Case 8: stamp empty text ----
    stamped = stamp_text("", fields)
    check(
        "stamp_text on empty input produces the header block alone",
        stamped == block,
    )

    # ---- Case 9: stamp non-empty text inserts separator ----
    body = "line1\nline2\n"
    stamped_body = stamp_text(body, fields)
    check(
        "stamp_text on non-empty body prepends header + blank line",
        stamped_body == block + "\n" + body,
    )

    # ---- Case 10: idempotent re-stamp with identical fields ----
    twice = stamp_text(stamped_body, fields)
    check(
        "stamp_text is idempotent when fields match",
        twice == stamped_body,
    )

    # ---- Case 11: re-stamp with different fields is rejected ----
    different = dict(fields)
    different["profile_enum"] = "2"
    different["profile_string"] = "REG_PROFILE_FCC_15_247_DTS_BW500"
    try:
        stamp_text(stamped_body, different)
        check("stamp_text rejects re-stamp with different fields", False,
              "no exception raised")
    except ValueError:
        check("stamp_text rejects re-stamp with different fields", True)

    # ---- Case 12: parse on un-stamped text returns {} ----
    check(
        "parse_header_block returns {} when no header is present",
        parse_header_block("nothing to see here\n") == {},
    )

    # ---- Case 13: parse on corrupt (BEGIN without END) raises ----
    corrupt = "# " + BEGIN_FENCE + "\n# firmware_git_sha: abc\n"
    try:
        parse_header_block(corrupt)
        check("parse_header_block raises on BEGIN without END", False,
              "no exception raised")
    except ValueError:
        check("parse_header_block raises on BEGIN without END", True)

    # ---- Case 14: header does NOT trip the FCC-B2 naming linter ----
    # The block must not contain the forbidden raw tokens.
    bad_tokens = ("airtime_us", "dwell_us")
    check(
        "header block avoids forbidden raw FCC-B2 tokens",
        not any(t in block for t in bad_tokens),
        f"block={block!r}",
    )

    # ---- Case 15: validation rejects newline in a field value ----
    bad_fields = dict(fields)
    bad_fields["profile_string"] = "BAD\nVALUE"
    try:
        format_header_block(bad_fields)
        check("format_header_block rejects embedded newlines", False,
              "no exception raised")
    except ValueError:
        check("format_header_block rejects embedded newlines", True)

    # ---- Case 16: format rejects unknown extra fields ----
    extra_fields = dict(fields)
    extra_fields["bogus_key"] = "x"
    try:
        format_header_block(extra_fields)
        check("format_header_block rejects unknown keys", False,
              "no exception raised")
    except ValueError:
        check("format_header_block rejects unknown keys", True)

    # ---- Case 17: comment_prefix_for_path table ----
    check(
        "comment_prefix_for_path picks // for .c / .h",
        comment_prefix_for_path(Path("x.c")) == "// "
        and comment_prefix_for_path(Path("x.h")) == "// ",
    )
    check(
        "comment_prefix_for_path defaults to '# ' for .log / .md / .py",
        comment_prefix_for_path(Path("x.log")) == "# "
        and comment_prefix_for_path(Path("x.md")) == "# "
        and comment_prefix_for_path(Path("x.py")) == "# ",
    )

    # ---- Case 18-20: --if-unstamped CLI mode (b-b-3-1) -----------------
    # Exercise the CLI directly because the new behaviour lives in
    # _cmd_stamp, not in stamp_text. Use a tempdir to avoid touching
    # the workspace.
    import tempfile
    with tempfile.TemporaryDirectory() as td:
        tdp = Path(td)
        unstamped = tdp / "unstamped.log"
        unstamped.write_text("body line\n", encoding="utf-8")
        rc = main([
            "stamp", "--profile-enum", "0",
            "--input", str(unstamped),
            "--output", str(unstamped),
            "--if-unstamped",
        ])
        text_after = unstamped.read_text(encoding="utf-8")
        check(
            "--if-unstamped on unstamped file: stamps + exit 0",
            rc == 0 and BEGIN_FENCE in text_after,
            f"rc={rc} has_header={BEGIN_FENCE in text_after}",
        )

        # Re-run --if-unstamped: must be a no-op AND must preserve the
        # original build_timestamp_utc byte-for-byte.
        snapshot = unstamped.read_text(encoding="utf-8")
        rc2 = main([
            "stamp", "--profile-enum", "0",
            "--input", str(unstamped),
            "--output", str(unstamped),
            "--if-unstamped",
        ])
        snapshot_after = unstamped.read_text(encoding="utf-8")
        check(
            "--if-unstamped on pre-stamped file: no-op + exit 0",
            rc2 == 0 and snapshot_after == snapshot,
            f"rc={rc2} bytes_changed={snapshot_after != snapshot}",
        )

        # Same file, --if-unstamped, but with a DIFFERENT profile-enum.
        # Must still be a no-op (the flag means "skip if header exists"
        # regardless of field values) and must NOT raise the strict
        # stamp_text field-diff ValueError that bit us in b-b-2 verify.
        rc3 = main([
            "stamp", "--profile-enum", "2",
            "--input", str(unstamped),
            "--output", str(unstamped),
            "--if-unstamped",
        ])
        snapshot_after2 = unstamped.read_text(encoding="utf-8")
        check(
            "--if-unstamped tolerates stale fields without raising",
            rc3 == 0 and snapshot_after2 == snapshot,
            f"rc={rc3} bytes_changed={snapshot_after2 != snapshot}",
        )

        # Without --if-unstamped, the same back-to-back re-stamp must
        # still fail (regression guard for b-b-1 strict idempotency).
        rc4 = -1
        raised = False
        try:
            rc4 = main([
                "stamp", "--profile-enum", "2",
                "--input", str(unstamped),
                "--output", str(unstamped),
            ])
        except ValueError:
            raised = True
        check(
            "stamp without --if-unstamped still rejects field diff",
            raised,
            f"rc={rc4} raised={raised}",
        )

    print(f"[self-test] {cases - fails}/{cases} cases passed")
    return 1 if fails else 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _cmd_harvest(args: argparse.Namespace) -> int:
    firmware_root = args.firmware_root
    schema = harvest_firmware_schema_vers(firmware_root)
    profiles = harvest_profile_table(firmware_root)
    for k in sorted(schema):
        print(f"{k}: {schema[k]}")
    for v in sorted(profiles):
        print(f"profile[{v}]: {profiles[v]}")
    return 0


def _cmd_stamp(args: argparse.Namespace) -> int:
    text = args.input.read_text(encoding="utf-8") if args.input else ""
    prefix = (args.comment_prefix
              if args.comment_prefix is not None
              else (comment_prefix_for_path(args.input) if args.input
                    else DEFAULT_COMMENT_PREFIX))
    # --if-unstamped (b-b-3-1 pre-work for the retro-stamp sweep): if a
    # v1 header block is already present, exit 0 + no-op regardless of
    # field values. Skips harvesters AND git-sha collection so the
    # call is cheap to re-run over a large tree, AND preserves the
    # original `build_timestamp_utc` (which the strict stamp_text
    # idempotency contract from b-b-1 would otherwise reject as a
    # field diff). Output file (if any) is left untouched.
    if args.if_unstamped:
        existing = parse_header_block(text, comment_prefix=prefix)
        if existing:
            return 0
    fields = build_header_fields(
        firmware_root=args.firmware_root,
        repo_root=args.repo_root,
        profile_enum=args.profile_enum,
    )
    stamped = stamp_text(text, fields, comment_prefix=prefix)
    if args.output is None:
        sys.stdout.write(stamped)
    else:
        args.output.write_text(stamped, encoding="utf-8")
    return 0


def _cmd_parse(args: argparse.Namespace) -> int:
    text = args.input.read_text(encoding="utf-8")
    prefix = (args.comment_prefix
              if args.comment_prefix is not None
              else comment_prefix_for_path(args.input))
    parsed = parse_header_block(text, comment_prefix=prefix)
    if not parsed:
        print("(no header found)", file=sys.stderr)
        return 1
    for k in FIELD_ORDER:
        if k in parsed:
            print(f"{k}: {parsed[k]}")
    return 0


def main(argv: list) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest="cmd")

    p.add_argument("--self-test", action="store_true",
                   help="run built-in unit cases and exit")

    here = Path(__file__).resolve().parent
    default_firmware = (here.parent /
                        "DESIGN-CONTROLLER" / "firmware" / "murata_l072")
    default_repo = here.parent.parent

    sp = sub.add_parser("harvest",
                        help="print harvested schema_ver + profile table")
    sp.add_argument("--firmware-root", type=Path, default=default_firmware)
    sp.set_defaults(func=_cmd_harvest)

    sp = sub.add_parser("stamp",
                        help="prepend a header block to a text artifact")
    sp.add_argument("--firmware-root", type=Path, default=default_firmware)
    sp.add_argument("--repo-root", type=Path, default=default_repo)
    sp.add_argument("--profile-enum", type=int, required=True)
    sp.add_argument("--input", type=Path,
                    help="input file (default: stdin / empty)")
    sp.add_argument("--output", type=Path,
                    help="output file (default: stdout)")
    sp.add_argument("--comment-prefix", default=None,
                    help="override comment prefix (default: derive from --input)")
    sp.add_argument("--if-unstamped", action="store_true",
                    help="no-op + exit 0 if a v1 header is already present")
    sp.set_defaults(func=_cmd_stamp)

    sp = sub.add_parser("parse",
                        help="extract header fields from a stamped artifact")
    sp.add_argument("--input", type=Path, required=True)
    sp.add_argument("--comment-prefix", default=None)
    sp.set_defaults(func=_cmd_parse)

    args = p.parse_args(argv)
    if args.self_test:
        return run_self_test()
    if not args.cmd:
        p.print_help(sys.stderr)
        return 2
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
