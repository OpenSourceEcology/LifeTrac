#!/usr/bin/env python3
"""FCC-B3-2 runtime profile gate.

Compares the runtime profile enum reported by an on-device probe (a
single canonical ``RUNTIME_PROFILE_ENUM=<N>`` line written to stdout
by ``emit_runtime_profile_enum`` in
``DESIGN-CONTROLLER/firmware/x8_lora_bootloader_helper/method_h_stage2_tx_probe_v2.py``,
FCC-B3-1) against the orchestrator's declared ``--expected-enum``.

Purpose: catch the failure mode where an orchestrator believes it is
running profile X but the firmware on the board is actually compiled
for profile Y (the FCC-B3 problem statement: artifact-header
``profile_enum`` documents intent; this gate proves runtime identity).

Exit codes (locked by FCC-B3-0, must not collide with the FCC-B2-b
header-lint exit 3):

    0 = match: log has exactly one ``RUNTIME_PROFILE_ENUM=<N>`` line
        and N equals ``--expected-enum`` (or ``--allow-legacy`` was
        passed AND the log contains zero readout lines).
    1 = mismatch: log has exactly one ``RUNTIME_PROFILE_ENUM=<N>``
        line and N != ``--expected-enum`` — i.e. the firmware on the
        board is the wrong profile for this run.
    2 = parse / CLI misuse: bad arguments, missing log file, or the
        expected-enum value is not a valid u8.
    4 = probe regression: zero readouts found (and ``--allow-legacy``
        was not passed) OR more than one readout found (FCC-B3-1
        idempotency violation) OR the single readout is an
        ``RUNTIME_PROFILE_ENUM=ERR <reason>`` line (probe could not
        reach the firmware to determine the runtime profile).

Encoding contract: reads the log with ``encoding="utf-8-sig",
errors="surrogateescape", newline=""``. ``utf-8-sig`` transparently
strips the BOM that PowerShell ``Out-File -Encoding utf8`` prepends to
captured logs while behaving identically to ``utf-8`` for non-BOM
files. ``surrogateescape`` matches the FCC-B2-b artifact-header reader
contract so a binary byte in the log never crashes the gate.

Usage::

    python3 check_run_profile.py --log path/to/probe.log --expected-enum 0
    python3 check_run_profile.py --log path/to/probe.log --expected-enum 0 --allow-legacy
    python3 check_run_profile.py --self-test

This is the CLI tool half of FCC-B3-2. Wiring into the three
orchestrators (``mixed_load_soak.ps1``, ``full_walk_power_sweep.ps1``,
``paired_walk_power_sweep.ps1``) is FCC-B3-3.
"""
from __future__ import annotations

import argparse
import re
import sys
import tempfile
from pathlib import Path
from typing import List, Tuple

EXIT_MATCH = 0
EXIT_MISMATCH = 1
EXIT_CLI = 2
EXIT_PROBE_REGRESSION = 4

# Anchored at line start (re.MULTILINE) so stray ``RUNTIME_PROFILE_ENUM=``
# substrings in narrative text — e.g. a TODO entry quoting the spec —
# never count as a readout. The value capture is greedy on non-whitespace
# so both numeric ``RUNTIME_PROFILE_ENUM=0`` and error
# ``RUNTIME_PROFILE_ENUM=ERR <reason>`` forms match: the trailing
# ``<reason>`` after a space is NOT captured here on purpose so that the
# discriminator stays cheap; downstream logic re-inspects the original
# line if it needs the reason.
_READOUT_RE = re.compile(r"(?m)^RUNTIME_PROFILE_ENUM=(\S+)")


def _read_log(path: Path) -> str:
    """Read ``path`` with the FCC-B3-2 encoding contract.

    Raises ``FileNotFoundError`` if the path does not exist; caller
    converts that into exit 2.
    """
    return path.read_text(
        encoding="utf-8-sig", errors="surrogateescape", newline="")


def find_readouts(text: str) -> List[str]:
    """Return every ``RUNTIME_PROFILE_ENUM=<token>`` value (the
    token after the ``=``, no trailing reason) in document order.

    Pure function — exposed for the self-test harness.
    """
    return _READOUT_RE.findall(text)


def classify(readouts: List[str], expected_enum: int,
             allow_legacy: bool) -> Tuple[int, str]:
    """Pure classifier.

    Returns ``(exit_code, human_message)``. No I/O. Exposed so the
    self-test can drive every branch without writing temp files.
    """
    n = len(readouts)
    if n == 0:
        if allow_legacy:
            return (EXIT_MATCH,
                    "no RUNTIME_PROFILE_ENUM line found; --allow-legacy "
                    "accepted (pre-FCC-B3-1 capture)")
        return (EXIT_PROBE_REGRESSION,
                "no RUNTIME_PROFILE_ENUM line found; pass --allow-legacy "
                "if this log predates FCC-B3-1 firmware/probe")
    if n > 1:
        return (EXIT_PROBE_REGRESSION,
                f"found {n} RUNTIME_PROFILE_ENUM lines, expected exactly 1 "
                f"(FCC-B3-1 idempotency violation in producer); "
                f"values={readouts!r}")
    token = readouts[0]
    if token == "ERR":
        return (EXIT_PROBE_REGRESSION,
                "RUNTIME_PROFILE_ENUM=ERR readout (probe could not reach "
                "firmware to determine runtime profile)")
    try:
        got = int(token, 10)
    except ValueError:
        return (EXIT_PROBE_REGRESSION,
                f"RUNTIME_PROFILE_ENUM token {token!r} is not a decimal "
                f"integer or 'ERR'")
    if got != expected_enum:
        return (EXIT_MISMATCH,
                f"runtime profile mismatch: log reports "
                f"RUNTIME_PROFILE_ENUM={got}, orchestrator expected "
                f"--expected-enum={expected_enum} (wrong firmware on board?)")
    return (EXIT_MATCH,
            f"runtime profile match: RUNTIME_PROFILE_ENUM={got} == "
            f"--expected-enum={expected_enum}")


def check(log_path: Path, expected_enum: int,
          allow_legacy: bool = False) -> Tuple[int, str]:
    """Read ``log_path`` and classify. Returns ``(exit_code, message)``.

    Caller is responsible for printing the message and exiting with
    the returned code.
    """
    try:
        text = _read_log(log_path)
    except FileNotFoundError:
        return (EXIT_CLI, f"log file not found: {log_path}")
    except OSError as exc:
        return (EXIT_CLI, f"failed to read log {log_path}: {exc}")
    return classify(find_readouts(text), expected_enum, allow_legacy)


# ---------------------------------------------------------------------------
# Self-test
# ---------------------------------------------------------------------------

def _write(p: Path, body: str) -> Path:
    p.write_text(body, encoding="utf-8", errors="surrogateescape", newline="")
    return p


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

    # ---- find_readouts() unit cases ----
    record("01 single readout enum=0",
           find_readouts("MODE: tx_burst\nRUNTIME_PROFILE_ENUM=0\nbody\n")
           == ["0"])
    record("02 single readout enum=2",
           find_readouts("RUNTIME_PROFILE_ENUM=2\n") == ["2"])
    record("03 ERR readout",
           find_readouts(
               "RUNTIME_PROFILE_ENUM=ERR request_failed:TimeoutError\n")
           == ["ERR"])
    record("04 zero readouts (empty)",
           find_readouts("MODE: tx\nbody\n") == [])
    record("05 zero readouts (substring in narrative does NOT count)",
           find_readouts("# notes: the RUNTIME_PROFILE_ENUM= line is the "
                         "B3-1 contract.\n") == [])
    record("06 two readouts (idempotency violation)",
           find_readouts(
               "RUNTIME_PROFILE_ENUM=0\nMODE: tx\nRUNTIME_PROFILE_ENUM=0\n")
           == ["0", "0"])
    # BOM handling is the reader's responsibility (utf-8-sig in _read_log),
    # not find_readouts'. Confirm the bare regex correctly does NOT match a
    # BOM-prefixed line so we'd catch a future refactor that dropped
    # utf-8-sig from the reader. The end-to-end BOM case below (35)
    # exercises the full reader + classifier path.
    record("07 BOM-prefixed line at function level does NOT match "
           "(reader is responsible for BOM strip)",
           find_readouts("\ufeffRUNTIME_PROFILE_ENUM=1\n") == [])

    # ---- classify() branch cases ----
    code, _ = classify(["0"], 0, allow_legacy=False)
    record("10 match enum=0 → EXIT_MATCH", code == EXIT_MATCH)

    code, _ = classify(["2"], 2, allow_legacy=False)
    record("11 match enum=2 → EXIT_MATCH", code == EXIT_MATCH)

    code, _ = classify(["1"], 0, allow_legacy=False)
    record("12 mismatch (got 1, expected 0) → EXIT_MISMATCH",
           code == EXIT_MISMATCH)

    code, _ = classify(["0"], 1, allow_legacy=False)
    record("13 mismatch (got 0, expected 1) → EXIT_MISMATCH",
           code == EXIT_MISMATCH)

    code, _ = classify([], 0, allow_legacy=False)
    record("14 zero readouts, no --allow-legacy → EXIT_PROBE_REGRESSION",
           code == EXIT_PROBE_REGRESSION)

    code, _ = classify([], 0, allow_legacy=True)
    record("15 zero readouts, --allow-legacy → EXIT_MATCH",
           code == EXIT_MATCH)

    code, _ = classify(["0", "0"], 0, allow_legacy=False)
    record("16 two matching readouts → EXIT_PROBE_REGRESSION "
           "(idempotency violation, NOT pass)",
           code == EXIT_PROBE_REGRESSION)

    code, _ = classify(["0", "1"], 0, allow_legacy=False)
    record("17 two conflicting readouts → EXIT_PROBE_REGRESSION",
           code == EXIT_PROBE_REGRESSION)

    code, _ = classify(["ERR"], 0, allow_legacy=False)
    record("18 ERR readout → EXIT_PROBE_REGRESSION (not mismatch)",
           code == EXIT_PROBE_REGRESSION)

    code, _ = classify(["ERR"], 0, allow_legacy=True)
    record("19 ERR readout under --allow-legacy still "
           "→ EXIT_PROBE_REGRESSION (legacy opt-in covers MISSING "
           "lines, not ERR lines from a B3-1 probe that reached the "
           "board but failed CFG_GET)",
           code == EXIT_PROBE_REGRESSION)

    code, _ = classify(["garbage"], 0, allow_legacy=False)
    record("20 non-integer non-ERR token → EXIT_PROBE_REGRESSION",
           code == EXIT_PROBE_REGRESSION)

    # ---- check() end-to-end with temp files ----
    with tempfile.TemporaryDirectory() as td:
        root = Path(td)

        good = _write(root / "good.log",
                      "MODE: walk_power\ndev=/dev/ttymxc3\n"
                      "RUNTIME_PROFILE_ENUM=0\nbody\n")
        code, _ = check(good, 0)
        record("30 end-to-end match", code == EXIT_MATCH)

        wrong = _write(root / "wrong.log",
                       "RUNTIME_PROFILE_ENUM=1\nbody\n")
        code, _ = check(wrong, 0)
        record("31 end-to-end mismatch", code == EXIT_MISMATCH)

        legacy = _write(root / "legacy.log",
                        "MODE: tx\nold capture, no profile line\n")
        code, _ = check(legacy, 0)
        record("32 end-to-end legacy (no line) without --allow-legacy "
               "→ EXIT_PROBE_REGRESSION",
               code == EXIT_PROBE_REGRESSION)
        code, _ = check(legacy, 0, allow_legacy=True)
        record("33 end-to-end legacy (no line) with --allow-legacy "
               "→ EXIT_MATCH",
               code == EXIT_MATCH)

        code, _ = check(root / "does-not-exist.log", 0)
        record("34 missing log file → EXIT_CLI", code == EXIT_CLI)

        # PowerShell-style BOM-prefixed file (Out-File -Encoding utf8).
        # Write a single UTF-8 BOM (0xEF 0xBB 0xBF) followed by plain
        # UTF-8 bytes — this matches what PS `Out-File -Encoding utf8`
        # actually produces. utf-8-sig in _read_log should transparently
        # strip the BOM so find_readouts sees a clean ASCII line.
        bom = root / "bom.log"
        bom.write_bytes(b"\xef\xbb\xbfRUNTIME_PROFILE_ENUM=2\n")
        code, _ = check(bom, 2)
        record("35 end-to-end BOM-prefixed log → EXIT_MATCH",
               code == EXIT_MATCH)

    print(f"[check_run_profile self-test] cases={cases_total} "
          f"passed={cases_passed} failed={cases_total - cases_passed}")
    return EXIT_MATCH if cases_passed == cases_total else 1


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="FCC-B3-2: gate a probe log against an expected "
                    "RUNTIME_PROFILE_ENUM value.")
    parser.add_argument("--log", type=Path,
                        help="Path to the probe log to inspect.")
    parser.add_argument("--expected-enum", type=int,
                        help="Expected RUNTIME_PROFILE_ENUM value (0,1,2).")
    parser.add_argument("--allow-legacy", action="store_true",
                        help="Treat a log with zero RUNTIME_PROFILE_ENUM "
                             "lines as a pass (pre-FCC-B3-1 captures). "
                             "Does NOT cover ERR readouts.")
    parser.add_argument("--self-test", action="store_true",
                        help="Run built-in self-test and exit 0/1.")
    args = parser.parse_args(argv)

    if args.self_test:
        return run_self_test()

    if args.log is None or args.expected_enum is None:
        parser.print_usage(sys.stderr)
        print("error: --log and --expected-enum are required "
              "(unless --self-test)", file=sys.stderr)
        return EXIT_CLI

    if not (0 <= args.expected_enum <= 255):
        print(f"error: --expected-enum must fit in a u8 "
              f"(0..255); got {args.expected_enum}", file=sys.stderr)
        return EXIT_CLI

    code, message = check(args.log, args.expected_enum,
                          allow_legacy=args.allow_legacy)
    print(message)
    return code


if __name__ == "__main__":
    sys.exit(main())
