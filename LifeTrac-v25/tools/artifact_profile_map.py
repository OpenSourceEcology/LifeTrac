#!/usr/bin/env python3
"""FCC-B2-b-b-3-2-2-2 loader + validator for the workload-prefix ->
profile_enum map.

The companion data file
[artifact_profile_map.json](artifact_profile_map.json) records the
profile_enum that every collapsed workload prefix observed under
``LifeTrac-v25/DESIGN-CONTROLLER/bench-evidence/`` should be stamped
with. The forcing-function design (see that file's ``doc`` field):

  * Every prefix that exists in the corpus MUST have an explicit
    entry.
  * ``lookup_profile_for_group(group, prefix_map)`` collapses the
    group via ``collapse_to_prefix`` and then does a strict dict
    lookup. Unknown prefixes raise ``KeyError`` — there is no
    silent default to bench-only. Future FHSS / DTS captures
    therefore CANNOT be mis-stamped by accident; the operator is
    forced to author a map entry before the b-b-3-2-2-3 sweep will
    touch the new workload.
  * Profile-enum values are constrained to the three wire constants
    documented in
    [LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072/](LifeTrac-v25/DESIGN-CONTROLLER/firmware/murata_l072):
        0 = REG_PROFILE_BENCH_ONLY_FIXED_915
        1 = REG_PROFILE_FCC_15_247_FHSS_50CH_BW250
        2 = REG_PROFILE_FCC_15_247_DTS_BW500

This module is read-only. It does not stamp anything; the actual
``stamp --profile-from-map FILE`` CLI lives downstream in
b-b-3-2-2-3, which will import ``load_profile_map`` +
``lookup_profile_for_group`` from here.

Exit codes (CLI):
    0  success (self-test green, validation clean, lookup hit)
    1  validation found problems, or lookup KeyError
    2  bad CLI / file unreadable / JSON parse error
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Dict, Iterable, List, Optional

_HERE = Path(__file__).resolve().parent
if str(_HERE) not in sys.path:
    sys.path.insert(0, str(_HERE))
from inventory_artifact_headers import collapse_to_prefix  # noqa: E402

MAP_SCHEMA_VERSION = 1
VALID_PROFILE_ENUMS = (0, 1, 2)
DEFAULT_MAP_PATH = _HERE / "artifact_profile_map.json"


def load_profile_map(path: Path) -> Dict[str, int]:
    """Read the JSON map file and return the validated prefixes dict.

    Raises ``ValueError`` on schema mismatch, malformed JSON, or
    invalid profile_enum values. Raises ``OSError`` if the file
    cannot be read.
    """
    raw = path.read_text(encoding="utf-8")
    try:
        envelope = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise ValueError(f"{path}: invalid JSON: {exc}") from exc
    if not isinstance(envelope, dict):
        raise ValueError(f"{path}: top level must be a JSON object")
    ver = envelope.get("schema_version")
    if ver != MAP_SCHEMA_VERSION:
        raise ValueError(
            f"{path}: schema_version={ver!r} but loader expects "
            f"{MAP_SCHEMA_VERSION}"
        )
    prefixes = envelope.get("prefixes")
    if not isinstance(prefixes, dict):
        raise ValueError(f"{path}: 'prefixes' must be a JSON object")
    out: Dict[str, int] = {}
    for k, v in prefixes.items():
        if not isinstance(k, str) or not k:
            raise ValueError(
                f"{path}: prefix keys must be non-empty strings, "
                f"got {k!r}"
            )
        if not isinstance(v, int) or isinstance(v, bool):
            raise ValueError(
                f"{path}: prefix {k!r} maps to non-int {v!r}"
            )
        if v not in VALID_PROFILE_ENUMS:
            raise ValueError(
                f"{path}: prefix {k!r} maps to profile_enum={v} "
                f"which is not one of {VALID_PROFILE_ENUMS}"
            )
        out[k] = v
    return out


def lookup_profile_for_group(group: str,
                             prefix_map: Dict[str, int]) -> int:
    """Collapse ``group`` to its workload prefix and return the
    mapped profile_enum.

    Raises ``KeyError`` with a clear message if the collapsed
    prefix is not in the map — by design, there is no default.
    """
    prefix = collapse_to_prefix(group)
    if prefix not in prefix_map:
        raise KeyError(
            f"no profile_enum entry for group={group!r} "
            f"(collapsed prefix={prefix!r}); add it to the map "
            f"before stamping"
        )
    return prefix_map[prefix]


def validate_map(prefix_map: Dict[str, int],
                 required_prefixes: Optional[Iterable[str]] = None
                 ) -> List[str]:
    """Return a list of human-readable error strings, empty on clean.

    Re-checks invariants that ``load_profile_map`` already enforced
    (defensive — in case the dict was constructed in-memory and not
    via the loader), and optionally cross-checks against a list of
    prefixes the caller knows must be covered (e.g. the output of
    ``inventory_artifact_headers.py --list-prefixes`` against the
    real corpus).
    """
    errors: List[str] = []
    for k, v in prefix_map.items():
        if not isinstance(k, str) or not k:
            errors.append(f"non-empty-string key required, got {k!r}")
            continue
        if not isinstance(v, int) or isinstance(v, bool):
            errors.append(f"prefix {k!r}: value must be int, got {v!r}")
            continue
        if v not in VALID_PROFILE_ENUMS:
            errors.append(
                f"prefix {k!r}: profile_enum={v} not in "
                f"{VALID_PROFILE_ENUMS}"
            )
    if required_prefixes is not None:
        missing = sorted(set(required_prefixes) - set(prefix_map))
        for m in missing:
            errors.append(f"required prefix missing from map: {m!r}")
        extra = sorted(set(prefix_map) - set(required_prefixes))
        # Extras are NOT errors (the map may legitimately list
        # prefixes that don't exist on disk yet — e.g. an upcoming
        # FHSS workload pre-registered before the first capture).
        # Surface them as informational so the operator notices
        # stale entries.
        for e in extra:
            errors.append(
                f"INFO: map prefix not present in required set: {e!r}"
            )
    return errors


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

    # ---- load_profile_map: round-trip a minimal good map ----
    with tempfile.TemporaryDirectory() as td:
        good = Path(td) / "good.json"
        good.write_text(json.dumps({
            "schema_version": 1,
            "doc": "tiny test map",
            "prefixes": {
                "T6_bringup": 0,
                "future_fhss_workload": 1,
                "future_dts_workload": 2,
            },
        }), encoding="utf-8")
        loaded = load_profile_map(good)
        check(
            "load_profile_map round-trips a valid 3-entry map",
            loaded == {"T6_bringup": 0,
                       "future_fhss_workload": 1,
                       "future_dts_workload": 2},
            f"got={loaded}",
        )

        # ---- schema_version mismatch -> ValueError ----
        bad_ver = Path(td) / "bad_ver.json"
        bad_ver.write_text(json.dumps({
            "schema_version": 99, "prefixes": {}}), encoding="utf-8")
        try:
            load_profile_map(bad_ver)
            check("schema_version mismatch raises ValueError", False)
        except ValueError as exc:
            check("schema_version mismatch raises ValueError",
                  "schema_version" in str(exc))

        # ---- invalid enum value -> ValueError ----
        bad_enum = Path(td) / "bad_enum.json"
        bad_enum.write_text(json.dumps({
            "schema_version": 1,
            "prefixes": {"x": 7}}), encoding="utf-8")
        try:
            load_profile_map(bad_enum)
            check("invalid profile_enum raises ValueError", False)
        except ValueError as exc:
            check("invalid profile_enum raises ValueError",
                  "profile_enum=7" in str(exc))

        # ---- bool sneaking through as int -> ValueError ----
        # (in Python isinstance(True, int) is True; the loader
        # explicitly rejects bools so True doesn't silently become
        # profile_enum=1)
        bad_bool = Path(td) / "bad_bool.json"
        bad_bool.write_text(
            '{"schema_version":1,"prefixes":{"x":true}}',
            encoding="utf-8")
        try:
            load_profile_map(bad_bool)
            check("bool value rejected (not silently == 1)", False)
        except ValueError:
            check("bool value rejected (not silently == 1)", True)

        # ---- malformed JSON -> ValueError ----
        bad_json = Path(td) / "bad.json"
        bad_json.write_text("{not json", encoding="utf-8")
        try:
            load_profile_map(bad_json)
            check("malformed JSON raises ValueError", False)
        except ValueError:
            check("malformed JSON raises ValueError", True)

    # ---- lookup_profile_for_group: collapses then looks up ----
    pmap = {"T6_bringup": 0, "W2-02_image_over_lora": 0,
            "future_fhss": 1}
    check(
        "lookup collapses dated suffix before lookup",
        lookup_profile_for_group(
            "T6_bringup_2026-05-09_132110", pmap) == 0,
    )
    check(
        "lookup preserves non-collapsing names",
        lookup_profile_for_group("future_fhss", pmap) == 1,
    )
    try:
        lookup_profile_for_group("never_seen_workload", pmap)
        check("unknown prefix raises KeyError (no silent default)",
              False)
    except KeyError as exc:
        check(
            "unknown prefix raises KeyError (no silent default)",
            "never_seen_workload" in str(exc),
        )

    # ---- validate_map: in-memory checks ----
    errs = validate_map({"T6_bringup": 0, "x": 1, "y": 2})
    check("validate_map clean on a good in-memory dict",
          errs == [], f"got={errs}")
    errs = validate_map({"T6_bringup": 5})
    check("validate_map flags out-of-range enum",
          any("not in" in e for e in errs), f"got={errs}")
    errs = validate_map({"T6_bringup": True})
    check("validate_map flags bool value",
          any("must be int" in e for e in errs), f"got={errs}")
    errs = validate_map(
        {"T6_bringup": 0, "stale": 0},
        required_prefixes=["T6_bringup", "missing_one"],
    )
    check(
        "validate_map reports missing required prefix as error",
        any("required prefix missing" in e and "missing_one" in e
            for e in errs),
        f"got={errs}",
    )
    check(
        "validate_map reports extra map entry as INFO (not error)",
        any(e.startswith("INFO:") and "stale" in e for e in errs),
        f"got={errs}",
    )

    # ---- the real shipped map ----
    if DEFAULT_MAP_PATH.exists():
        shipped = load_profile_map(DEFAULT_MAP_PATH)
        check(
            "shipped map loads cleanly with schema_version=1",
            len(shipped) >= 79,
            f"got len={len(shipped)}",
        )
        check(
            "shipped map: every value in VALID_PROFILE_ENUMS",
            all(v in VALID_PROFILE_ENUMS for v in shipped.values()),
        )
        check(
            "shipped map: validate_map returns no errors",
            validate_map(shipped) == [],
        )
        # Spot-check a representative subset (large workloads
        # observed in the b-b-3-2-1 inventory).
        for prefix in ("T6_stage1_standard", "stage1_standard_runs",
                       "T6_rom_baseline_burst", "T6_bringup",
                       "w2_01_production", "(root)",
                       "W2-02_image_over_lora", "mixed_load"):
            check(
                f"shipped map contains {prefix!r}",
                prefix in shipped,
            )

    print(f"[self-test] {cases - fails}/{cases} cases passed")
    return 1 if fails else 0


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv: list) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--map", type=Path, default=DEFAULT_MAP_PATH,
                   help=f"map file (default: {DEFAULT_MAP_PATH})")
    p.add_argument("--validate", action="store_true",
                   help="load the map and run validate_map; if "
                        "--require-prefixes-from is given, cross-check "
                        "the keyset")
    p.add_argument("--require-prefixes-from", type=Path, default=None,
                   help="path to a newline-delimited list of prefixes "
                        "the map must cover (typically the output of "
                        "`inventory_artifact_headers.py "
                        "--list-prefixes` against the live corpus)")
    p.add_argument("--lookup", metavar="GROUP", default=None,
                   help="print the profile_enum for GROUP (after "
                        "collapsing); exit 1 on KeyError")
    p.add_argument("--self-test", action="store_true",
                   help="run built-in unit cases and exit")
    args = p.parse_args(argv)
    if args.self_test:
        return run_self_test()
    try:
        prefix_map = load_profile_map(args.map)
    except (OSError, ValueError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    if args.lookup is not None:
        try:
            print(lookup_profile_for_group(args.lookup, prefix_map))
            return 0
        except KeyError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 1
    if args.validate:
        required = None
        if args.require_prefixes_from is not None:
            # utf-8-sig so a BOM emitted by PowerShell's `Out-File
            # -Encoding utf8` (PS 5.1) does not corrupt the first
            # prefix. Strict utf-8 would silently mis-key the first
            # line as '\ufeffT6_...'.
            required = [
                line.strip()
                for line in args.require_prefixes_from.read_text(
                    encoding="utf-8-sig").splitlines()
                if line.strip()
            ]
        errs = validate_map(prefix_map, required_prefixes=required)
        only_info = errs and all(e.startswith("INFO:") for e in errs)
        for e in errs:
            print(e)
        if not errs:
            print(f"[map] OK  entries={len(prefix_map)}")
            return 0
        if only_info:
            print(f"[map] OK with info  entries={len(prefix_map)}")
            return 0
        print(f"[map] FAIL  entries={len(prefix_map)}  errors="
              f"{sum(1 for e in errs if not e.startswith('INFO:'))}")
        return 1
    # Default action: print a summary.
    print(f"[map] loaded {len(prefix_map)} prefixes from {args.map}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
