#!/usr/bin/env python3
"""tools/replay_pcap.py — offline replay of a captured LoRa frame log.

Reads a simple capture file (one frame per line, hex-encoded raw on-air
bytes — the same format `audit_log.py` writes for `rx_raw` events when
`LIFETRAC_LOG_RAW=1` is set on the bridge) and re-runs each frame through
`decrypt_frame` + `ReplayWindow.check_and_update`. Prints a one-line
summary per frame so an analyst can verify offline that:

  * GCM tags validate (key was correct at capture time),
  * sequence numbers were monotonic (no replay during the live session),
  * a deliberate replay attempt by appending a duplicate line is caught.

Useful for:
  * post-incident analysis of a flagged audit window,
  * validating a freshly-captured pcap against a candidate fleet key
    before fielding the key,
  * regression-testing changes to the replay window logic.

Capture file format (hex, one frame per line, lines starting with `#`
are comments):

    # capture from bench session 2026-04-27
    01 00 12 34 56 78 ...
    01 00 12 34 56 79 ...

Whitespace inside a line is ignored.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Make `base_station` importable so this script works from anywhere in
# the repo without a pip install.
_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / "base_station"))

from lora_proto import ReplayWindow, decrypt_frame  # noqa: E402


def _parse_key(s: str) -> bytes:
    cleaned = s.replace(" ", "").replace(":", "").replace("-", "").strip()
    key = bytes.fromhex(cleaned)
    if len(key) not in (16, 32):
        raise ValueError(f"key must be 16 or 32 bytes, got {len(key)}")
    return key


def _iter_frames(path: Path):
    for n, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        try:
            yield n, bytes.fromhex(line.replace(" ", ""))
        except ValueError as exc:
            print(f"# line {n}: skip ({exc})", file=sys.stderr)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("capture", type=Path,
                   help="hex-per-line capture file (see module docstring)")
    p.add_argument("--key", required=True,
                   help="fleet key as hex (16 B AES-128 or 32 B AES-256)")
    p.add_argument("--key-file", type=Path,
                   help="read key from file (overrides --key); strips whitespace")
    args = p.parse_args()

    if args.key_file is not None:
        key = _parse_key(args.key_file.read_text(encoding="ascii"))
    else:
        key = _parse_key(args.key)

    if not args.capture.exists():
        print(f"error: capture file not found: {args.capture}", file=sys.stderr)
        return 2

    rw = ReplayWindow()
    n_total = n_decrypt_ok = n_replay = n_decrypt_fail = 0

    for line_no, frame in _iter_frames(args.capture):
        n_total += 1
        plain = decrypt_frame(key, frame)
        if plain is None:
            n_decrypt_fail += 1
            print(f"line {line_no:5d}  GCM_REJECT          ({len(frame)} B)")
            continue
        # The first 8 bytes of the nonce we built are: source_id (1) + seq (2)
        # + epoch (4) + tail. We reuse that here for replay window indexing.
        nonce = frame[:12]
        source_id = nonce[0]
        seq = int.from_bytes(nonce[1:3], "little")
        accepted = rw.check_and_update(seq)
        n_decrypt_ok += 1
        if not accepted:
            n_replay += 1
            tag = "REPLAY_REJECT"
        else:
            tag = "OK           "
        print(f"line {line_no:5d}  {tag}  src=0x{source_id:02x} "
              f"seq={seq:5d}  pt={len(plain)} B")

    print(f"\nsummary: total={n_total}  ok={n_decrypt_ok}  "
          f"replay={n_replay}  gcm_fail={n_decrypt_fail}")
    return 0 if (n_decrypt_fail == 0) else 1


if __name__ == "__main__":
    raise SystemExit(main())
