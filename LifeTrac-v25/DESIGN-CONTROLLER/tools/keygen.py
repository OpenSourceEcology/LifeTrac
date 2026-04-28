#!/usr/bin/env python3
"""tools/keygen.py — emit a fresh LifeTrac v25 fleet key.

A thin standalone wrapper around `secrets.token_bytes(16)` so that key
generation is a separately-auditable step from key *deployment*. The
existing `tools/provision.py` can both generate and write a key, but
operators who follow KEY_ROTATION.md are expected to:

  1. Generate the new key on an offline laptop:    python tools/keygen.py > keys/fleet.hex
  2. Carry it on a USB stick to the bench host.
  3. Provision every node:                         python tools/provision.py --key keys/fleet.hex --write-port /dev/ttyACM0

Splitting the two steps means the laptop that generates the key never
needs to touch a flashed device, and the bench host never needs to hold
generation entropy beyond the lifetime of the file on the USB stick.

Output is **uppercase hex, no whitespace, single line, trailing newline**
so it round-trips cleanly through `bytes.fromhex` in `provision.py`.
"""

from __future__ import annotations

import argparse
import secrets
import sys
from pathlib import Path

KEY_LEN = 16   # AES-128 fleet key per LORA_PROTOCOL.md § Cryptographic envelope


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--out", type=Path, default=None,
                   help="write to file instead of stdout (mode 0600)")
    p.add_argument("--length", type=int, default=KEY_LEN,
                   help=f"override key length in bytes (default {KEY_LEN}); "
                        "v25 production must be 16")
    args = p.parse_args()

    if args.length not in (16, 32):
        print("error: --length must be 16 (AES-128) or 32 (AES-256 future)",
              file=sys.stderr)
        return 2

    key = secrets.token_bytes(args.length)
    line = key.hex().upper() + "\n"

    if args.out is not None:
        args.out.write_text(line, encoding="ascii")
        try:
            args.out.chmod(0o600)
        except OSError:
            # Windows lacks chmod semantics; relying on filesystem ACLs.
            pass
        print(f"wrote {args.length}-byte key to {args.out}", file=sys.stderr)
    else:
        sys.stdout.write(line)
        sys.stdout.flush()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
