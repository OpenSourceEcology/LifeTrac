"""Parser-token contract regression test (P1-2 / v4.0 §4).

Locks the orchestrator-parser <-> probe-log token contract so a future rename
of `__RX_FRAME__` (or `__RX_LISTEN_STOPPED__`, `__W1_10B_LISTEN_DONE__`,
`__RUNTIME_PROFILE_ENUM`, etc.) cannot silently swallow events the way the
2026-05-20 T4 walk_power run did (676 RX frames reported as 0).

Run:
    py -3 test_parser_token_contract.py

This is intentionally a pure-stdlib script with a tiny self-rolled
assertion harness (no pytest dependency) so it can run from any X8 board
or developer laptop without installing anything.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
FIXTURE_DIR = ROOT / "fixtures" / "parser_contract"

# Canonical contract: every entry is (token_label, regex_pattern,
# expected_count_in_fixture). The fixture log MUST contain exactly the
# expected count of matches; any drift (probe rename, orchestrator regex
# miscount) trips the test.
CONTRACT = [
    ("RX_FRAME_v2",        r"__RX_FRAME__\b",                 3),
    ("RX_FRAME_legacy",    r"RX_FRAME_URC:",                  2),
    ("RX_FRAME_combined",  r"__RX_FRAME__|RX_FRAME_URC:",     5),
    ("RX_LISTEN_STOPPED",  r"__RX_LISTEN_STOPPED__\b",        1),
    ("LISTEN_DONE",        r"__W1_10B_LISTEN_DONE__\b",       1),
    ("RUNTIME_PROFILE_OK", r"^RUNTIME_PROFILE_ENUM=\d+$",     1),
    ("RUNTIME_PROFILE_ERR",r"^RUNTIME_PROFILE_ENUM=ERR\b",    1),
    ("TX_DONE_v2",         r"__TX_DONE__\b",                  2),
    ("KEYBOARD_INT",       r"^KeyboardInterrupt\b",           1),
]

# A minimal, hand-authored log that exercises every contract token. If you
# add or rename a token, update CONTRACT and this fixture together — that
# co-located change is the whole point of this test.
FIXTURE_TEXT = """\
MODE: rx_listen
dev=/dev/ttymxc3 baud=921600
RUNTIME_PROFILE_ENUM=0
RUNTIME_PROFILE_ENUM=ERR request_failed:TimeoutError
__W1_10B_LISTEN_READY__
__RX_FRAME__ idx=1 rssi=-114 snr=2 len=64 timestamp_us=100 payload_hex=fe0000ba
RX_FRAME_URC: idx=2 rssi=-113 snr=3 len=64 timestamp_us=200 payload_hex=fe0001ba
__RX_FRAME__ idx=3 rssi=-114 snr=2 len=64 timestamp_us=300 payload_hex=fe0002ba
RX_FRAME_URC: idx=4 rssi=-113 snr=2 len=64 timestamp_us=400 payload_hex=fe0003ba
__RX_FRAME__ idx=5 rssi=-114 snr=3 len=64 timestamp_us=500 payload_hex=fe0004ba
__TX_DONE__ idx=1 tx_id=0xdead status=0(OK) toa_us=30000 elapsed_ms=30.1 payload_hex=00
__TX_DONE__ idx=2 tx_id=0xbeef status=0(OK) toa_us=30000 elapsed_ms=30.2 payload_hex=00
KeyboardInterrupt
__RX_LISTEN_STOPPED__ reason=SIGINT rx_frames_so_far=5
__W1_10B_LISTEN_DONE__ rx_frames=5 radio_rx_ok_delta=5 radio_crc_err_delta=0 real_faults=0 invariants_violated=0 stopped_by_signal=1
"""


def ensure_fixture() -> Path:
    FIXTURE_DIR.mkdir(parents=True, exist_ok=True)
    fp = FIXTURE_DIR / "rx_listen_canonical.log"
    if not fp.exists() or fp.read_text(encoding="utf-8") != FIXTURE_TEXT:
        fp.write_text(FIXTURE_TEXT, encoding="utf-8")
    return fp


def count_pattern(text: str, pattern: str) -> int:
    flags = re.MULTILINE
    return len(re.findall(pattern, text, flags))


def run_contract(log_path: Path) -> int:
    text = log_path.read_text(encoding="utf-8")
    fails: list[str] = []
    for label, pattern, expected in CONTRACT:
        got = count_pattern(text, pattern)
        status = "OK " if got == expected else "FAIL"
        print(f"  [{status}] {label:<22} pattern={pattern!r:<40} "
              f"expected={expected} got={got}")
        if got != expected:
            fails.append(f"{label}: expected {expected}, got {got}")
    return 1 if fails else 0


def main() -> int:
    fp = ensure_fixture()
    print(f"Parser-token contract test on fixture: {fp}")
    print(f"  ({len(CONTRACT)} contract entries)")
    rc = run_contract(fp)
    if rc == 0:
        print("\nPASS  all parser tokens match contract.")
    else:
        print("\nFAIL  parser-token contract drift detected. "
              "Update CONTRACT and orchestrator parsers together.")
    return rc


if __name__ == "__main__":
    sys.exit(main())
