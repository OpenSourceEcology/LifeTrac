#!/usr/bin/env python3
"""w2_02_radio_wake_rxcont.py

W2-02 helper: open HostLink to /dev/ttymxc3, drain any pending URCs, then
write SX1276 RegOpMode=0x85 (LoRa + RXCONTINUOUS) so the radio is in the
correct receive state before run_rx_listen() enters its passive loop.

Background: every method_h_stage2_tx_probe.py probe call (TX or RX) ends
with __RADIO_SLEEP_ON_EXIT__ which writes RegOpMode=0x80 (LoRa SLEEP) so
debug probes don't leave the SX1276 transmitting at boot-time defaults.
That parking is fine for bring-up probes, but for W2-02 image-over-LoRa
the RX side needs to be in RXCONT *before* the listen window starts.

This script is intentionally tiny and import-clean: it reuses
method_h_stage2_tx_probe.py's HostLink + write_reg helpers via PYTHONPATH.

Usage (on the X8, as root for /dev/ttymxc3):
    python3 w2_02_radio_wake_rxcont.py --dev /dev/ttymxc3 --baud 921600

Prints exactly one machine-readable line on stdout:
    __W2_02_WAKE_OK__ opmode_pre=0xNN opmode_post=0xNN
or:
    __W2_02_WAKE_FAIL__ reason="..."
"""

from __future__ import annotations

import argparse
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from method_g_stage1_probe import HostLink  # noqa: E402
from method_h_stage2_tx_probe_v2 import (  # noqa: E402
    drain_boot,
    drain_pending,
    read_reg,
    write_reg,
    HOST_TYPE_VER_REQ,
    HOST_TYPE_VER_URC,
    SX1276_REG_OP_MODE,
    SX1276_OPMODE_LORA_RXCONT,
)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/ttymxc3")
    ap.add_argument("--baud", default="921600")
    args = ap.parse_args()

    try:
        link = HostLink(args.dev, args.baud)
    except Exception as exc:
        print(f'__W2_02_WAKE_FAIL__ reason="open: {exc}"')
        return 2

    try:
        drain_boot(link, 0.5)
        try:
            link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
        except Exception as exc:
            print(f'__W2_02_WAKE_FAIL__ reason="VER warm-up: {exc}"')
            return 2
        drain_pending(link, quiet_s=0.25, max_s=1.0)

        try:
            opm_pre, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
        except Exception as exc:
            print(f'__W2_02_WAKE_FAIL__ reason="read_reg(pre): {exc}"')
            return 2

        if opm_pre == SX1276_OPMODE_LORA_RXCONT:
            print(f"__W2_02_WAKE_OK__ opmode_pre=0x{opm_pre:02X} "
                  f"opmode_post=0x{opm_pre:02X} action=none")
            return 0

        try:
            write_reg(link, SX1276_REG_OP_MODE,
                      SX1276_OPMODE_LORA_RXCONT, timeout=0.5)
        except Exception as exc:
            print(f'__W2_02_WAKE_FAIL__ reason="write_reg: {exc}" '
                  f'opmode_pre=0x{opm_pre:02X}')
            return 2

        try:
            opm_post, _ = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
        except Exception as exc:
            print(f'__W2_02_WAKE_FAIL__ reason="read_reg(post): {exc}" '
                  f'opmode_pre=0x{opm_pre:02X}')
            return 2

        if opm_post != SX1276_OPMODE_LORA_RXCONT:
            print(f"__W2_02_WAKE_FAIL__ reason=\"opmode mismatch\" "
                  f"opmode_pre=0x{opm_pre:02X} opmode_post=0x{opm_post:02X} "
                  f"target=0x{SX1276_OPMODE_LORA_RXCONT:02X}")
            return 3

        print(f"__W2_02_WAKE_OK__ opmode_pre=0x{opm_pre:02X} "
              f"opmode_post=0x{opm_post:02X} action=write")
        return 0
    finally:
        try:
            link.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
