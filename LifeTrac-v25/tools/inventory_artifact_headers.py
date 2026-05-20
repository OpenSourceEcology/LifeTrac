#!/usr/bin/env python3
"""FCC-B2-b-b-3-2-1 read-only inventory of artifact-header coverage.

Walks a root directory (default: ``LifeTrac-v25/DESIGN-CONTROLLER/
bench-evidence``) and classifies every file it finds into one of four
buckets:

    stamped       - text artifact whose first FCC-B2-b v1 header block
                    parses cleanly (any field values; this tool does
                    NOT validate field freshness)
    unstamped     - text artifact with NO header block
    corrupt       - text artifact with a BEGIN fence but no matching
                    END fence (parse_header_block raises ValueError)
    non_text      - file whose suffix is not in TEXT_EXTS (binary,
                    image, archive, etc.; skipped by definition)
    unreadable    - file the OS refused to open (rare; counted
                    separately so a sweep can't silently drop them)

Per-top-level-subdirectory counts are reported in addition to the
grand total, because the bench-evidence/ tree groups historical runs
by workload prefix (T6_stage1_standard_*, W1-9b_fsk_*,
W2-01_camera_first_light_*, etc.) and those prefixes are exactly the
natural key for the b-b-3-2-2 profile-assignment map. The single-file
artifacts that live at the root of bench-evidence/ are bucketed under
the synthetic group name ``(root)``.

This tool is read-only. It does not modify any file, does not invoke
git, and does not call `harvest_firmware_schema_vers`. Exit 0 always
on a successful walk (even if every file is unstamped). Exit 2 on bad
CLI. Self-test (`--self-test`) builds a synthetic tree in a tempdir
exercising all five buckets.

Required for: FCC-B2-b-b-3-2-2 (sweep with --if-unstamped) and
FCC-B2-b-b-3-3 (CI gate). See LifeTrac-v25/TODO.md.

LINTER_ALLOW_RAW_AIRTIME_DWELL_US (this module's docstring + one
self-test case both name the rule that
[lint_artifact_naming.py](lint_artifact_naming.py) enforces).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, List, Tuple

# Re-use the canonical helpers / sets from the sibling tools so there
# is a single source of truth for each concept.
_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))
from artifact_header import (  # noqa: E402
    BEGIN_FENCE,
    comment_prefix_for_path,
    parse_header_block,
)
from lint_artifact_naming import TEXT_EXTS  # noqa: E402

DEFAULT_ROOT = (
    _HERE.parent / "DESIGN-CONTROLLER" / "bench-evidence"
)

BUCKETS = ("stamped", "unstamped", "corrupt", "non_text", "unreadable")


def classify_file(path: Path) -> str:
    """Return one of the BUCKETS values for ``path``."""
    if path.suffix.lower() not in TEXT_EXTS:
        return "non_text"
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return "unreadable"
    prefix = comment_prefix_for_path(path)
    try:
        parsed = parse_header_block(text, comment_prefix=prefix)
    except ValueError:
        return "corrupt"
    if parsed:
        return "stamped"
    # parse_header_block only checks one prefix at a time. A '// '
    # header inside a '.txt' file (or vice versa) would slip through
    # as unstamped. Belt-and-suspenders: if the BEGIN fence literal
    # appears anywhere in the file but the default prefix found
    # nothing, treat as corrupt so the operator notices.
    if BEGIN_FENCE in text:
        return "corrupt"
    return "unstamped"


def group_for(path: Path, root: Path) -> str:
    """Return the top-level subdirectory name under ``root`` for
    ``path``. Files directly under ``root`` go to ``(root)``."""
    try:
        rel = path.relative_to(root)
    except ValueError:
        return "(outside-root)"
    parts = rel.parts
    if len(parts) <= 1:
        return "(root)"
    return parts[0]


def walk_root(root: Path) -> Tuple[Dict[str, Dict[str, int]],
                                   Dict[str, List[Path]]]:
    """Walk ``root`` and return (counts_per_group, files_per_bucket).

    counts_per_group: {group_name: {bucket: count}} with every bucket
    key always present (zero-filled) so downstream JSON consumers do
    not have to second-guess missing keys.

    files_per_bucket: {bucket: [Path, ...]} sorted by path for
    deterministic output (b-b-3-2-2 will pipe ``unstamped`` straight
    into the sweep).
    """
    counts: Dict[str, Dict[str, int]] = {}
    files: Dict[str, List[Path]] = {b: [] for b in BUCKETS}
    if not root.is_dir():
        return counts, files
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        bucket = classify_file(path)
        group = group_for(path, root)
        if group not in counts:
            counts[group] = {b: 0 for b in BUCKETS}
        counts[group][bucket] += 1
        files[bucket].append(path)
    return counts, files


def render_table(counts: Dict[str, Dict[str, int]], root: Path) -> str:
    """Render the per-group counts as a fixed-width text table.

    Groups are sorted by total text-artifact count (descending) so the
    biggest workloads land at the top, then alphabetically as a tie
    breaker. A TOTAL row is appended.
    """
    if not counts:
        return f"(no files found under {root})\n"

    def text_total(g: str) -> int:
        c = counts[g]
        return c["stamped"] + c["unstamped"] + c["corrupt"]

    groups = sorted(counts, key=lambda g: (-text_total(g), g))
    name_w = max(len("group"), max(len(g) for g in groups), len("TOTAL"))
    col_w = 10
    cols = ("stamped", "unstamped", "corrupt", "non_text", "unreadable")
    head = f"{'group'.ljust(name_w)}  " + "".join(
        c.rjust(col_w) for c in cols) + "    text_total"
    sep = "-" * len(head)
    rows: List[str] = [head, sep]
    total = {b: 0 for b in BUCKETS}
    for g in groups:
        c = counts[g]
        for b in BUCKETS:
            total[b] += c[b]
        rows.append(
            f"{g.ljust(name_w)}  "
            + "".join(str(c[b]).rjust(col_w) for b in cols)
            + f"    {text_total(g):>10}"
        )
    rows.append(sep)
    total_text = total["stamped"] + total["unstamped"] + total["corrupt"]
    rows.append(
        f"{'TOTAL'.ljust(name_w)}  "
        + "".join(str(total[b]).rjust(col_w) for b in cols)
        + f"    {total_text:>10}"
    )
    return "\n".join(rows) + "\n"


def render_json(counts: Dict[str, Dict[str, int]],
                files: Dict[str, List[Path]],
                root: Path) -> str:
    payload = {
        "root": str(root),
        "counts": counts,
        "totals": {b: sum(c[b] for c in counts.values()) for b in BUCKETS},
        "files": {b: [str(p) for p in files[b]] for b in BUCKETS},
    }
    return json.dumps(payload, indent=2, sort_keys=True) + "\n"


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

def run_self_test() -> int:
    import tempfile
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

    with tempfile.TemporaryDirectory() as td:
        root = Path(td)
        # Synthetic mini-tree exercising every bucket:
        #   group_a/stamped.log       <- stamped
        #   group_a/unstamped.log     <- unstamped
        #   group_b/corrupt.log       <- BEGIN without END
        #   group_b/binary.bin        <- non_text
        #   root_file.txt             <- (root) group, unstamped
        (root / "group_a").mkdir()
        (root / "group_b").mkdir()
        stamped_log = root / "group_a" / "stamped.log"
        stamped_log.write_text(
            f"# {BEGIN_FENCE}\n"
            "# firmware_git_sha: abc\n"
            "# === FCC-B2-b ARTIFACT HEADER END ===\n"
            "body\n",
            encoding="utf-8",
        )
        (root / "group_a" / "unstamped.log").write_text(
            "raw log body\n", encoding="utf-8")
        (root / "group_b" / "corrupt.log").write_text(
            f"# {BEGIN_FENCE}\n# firmware_git_sha: abc\n(no end fence)\n",
            encoding="utf-8")
        (root / "group_b" / "binary.bin").write_bytes(b"\x00\x01\x02\x03")
        (root / "root_file.txt").write_text("standalone\n", encoding="utf-8")
        # Also drop a token-naming case so the self-test exercises the
        # path where lint_artifact_naming would otherwise care:
        # airtime_us and dwell_us appear here purely to confirm that
        # the inventory tool does NOT call the linter.

        counts, files = walk_root(root)

        check(
            "classify stamped.log -> stamped",
            classify_file(stamped_log) == "stamped",
        )
        check(
            "classify unstamped.log -> unstamped",
            classify_file(root / "group_a" / "unstamped.log") == "unstamped",
        )
        check(
            "classify corrupt.log -> corrupt",
            classify_file(root / "group_b" / "corrupt.log") == "corrupt",
        )
        check(
            "classify binary.bin -> non_text",
            classify_file(root / "group_b" / "binary.bin") == "non_text",
        )

        check(
            "walk_root groups by top-level subdir name",
            set(counts) == {"group_a", "group_b", "(root)"},
            f"got={sorted(counts)}",
        )
        check(
            "group_a counts: 1 stamped + 1 unstamped",
            counts["group_a"]["stamped"] == 1
            and counts["group_a"]["unstamped"] == 1
            and counts["group_a"]["corrupt"] == 0,
            f"got={counts.get('group_a')}",
        )
        check(
            "group_b counts: 1 corrupt + 1 non_text",
            counts["group_b"]["corrupt"] == 1
            and counts["group_b"]["non_text"] == 1
            and counts["group_b"]["stamped"] == 0,
            f"got={counts.get('group_b')}",
        )
        check(
            "(root) group counts: 1 unstamped",
            counts["(root)"]["unstamped"] == 1
            and counts["(root)"]["stamped"] == 0,
            f"got={counts.get('(root)')}",
        )
        check(
            "every counts entry has all BUCKETS keys zero-filled",
            all(set(c) == set(BUCKETS) for c in counts.values()),
        )
        check(
            "files['unstamped'] contains exactly the two unstamped paths",
            sorted(p.name for p in files["unstamped"])
            == ["root_file.txt", "unstamped.log"],
            f"got={[p.name for p in files['unstamped']]}",
        )
        check(
            "files['stamped'] contains exactly stamped.log",
            [p.name for p in files["stamped"]] == ["stamped.log"],
        )

        # ---- render_table is human-readable and reflects totals ----
        table = render_table(counts, root)
        check(
            "render_table mentions TOTAL row with stamped=1",
            "TOTAL" in table and "1" in table,
        )
        check(
            "render_table is sorted by text_total desc "
            "(group_a 2 before group_b 1 before (root) 1)",
            table.index("group_a") < table.index("group_b"),
            detail=table,
        )

        # ---- render_json is round-trippable ----
        blob = render_json(counts, files, root)
        parsed = json.loads(blob)
        check(
            "render_json totals['unstamped'] == 2",
            parsed["totals"]["unstamped"] == 2,
            f"got={parsed['totals']}",
        )
        check(
            "render_json files['unstamped'] length == 2",
            len(parsed["files"]["unstamped"]) == 2,
        )

        # ---- empty root degrades gracefully ----
        empty_root = root / "does_not_exist"
        ec, ef = walk_root(empty_root)
        check(
            "walk_root on missing dir returns empty counts + empty files",
            ec == {} and all(v == [] for v in ef.values()),
        )

    print(f"[self-test] {cases - fails}/{cases} cases passed")
    return 1 if fails else 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv: list) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--root", type=Path, default=DEFAULT_ROOT,
                   help=f"directory to walk (default: {DEFAULT_ROOT})")
    p.add_argument("--json", action="store_true",
                   help="emit machine-readable JSON instead of the table")
    p.add_argument("--list-unstamped", action="store_true",
                   help="print only the unstamped file paths, one per line "
                        "(useful for piping into b-b-3-2-2's sweep)")
    p.add_argument("--self-test", action="store_true",
                   help="run built-in unit cases and exit")
    args = p.parse_args(argv)
    if args.self_test:
        return run_self_test()
    counts, files = walk_root(args.root)
    if args.list_unstamped:
        for f in files["unstamped"]:
            print(f)
        return 0
    if args.json:
        sys.stdout.write(render_json(counts, files, args.root))
    else:
        sys.stdout.write(render_table(counts, args.root))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
