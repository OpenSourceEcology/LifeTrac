#!/usr/bin/env python3
"""radio_park.py — put the L072's SX1276 into LoRa SLEEP and verify.

Mirrors channel_survey_sniff.py's link bring-up exactly (HostLink, boot
drain, VER handshake, drain), then walks RegOpMode STANDBY -> SLEEP and
reads it back. Prints PARK_OK/PARK_FAIL. Run in the daemon-shaped
container on either board.
"""
from __future__ import annotations

import json
import sys
import time

sys.path.insert(0, "/work")

from method_h_stage2_tx_probe_v2 import (  # type: ignore
    HostLink,
    HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC,
    SX1276_REG_OP_MODE,
    read_reg, write_reg,
    drain_boot, drain_pending,
)

OPMODE_LORA_STANDBY = 0x81
OPMODE_LORA_SLEEP = 0x80


def main() -> int:
    uart = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttymxc3"
    link = HostLink(uart, "921600")
    drain_boot(link, settle_s=0.25)
    try:
        link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
    except Exception as exc:
        print("PARK_FAIL " + json.dumps({"stage": "ver", "err": str(exc)}),
              flush=True)
        return 2
    drain_pending(link, quiet_s=0.25, max_s=1.0)
    # PR #125 review (2026-09-15): the immediate readback proves only the
    # instant of the write. The firmware's own RX owners -- the scan walker
    # (ADVANCE_CHANNEL), the gamma-1 retune tick and, when LOCKED, the slot
    # follower -- all call sx1276_rx_arm() and will put the modem back into
    # RXCONT (receive-only) if any of them still has work. The park only
    # holds once the scan SM has exhausted into FAILED (nothing left to arm),
    # which is the normal state some tens of seconds after the daemons stop.
    # So: write, wait longer than a scan channel dwell (500 ms) and a slot
    # (200 ms), read AGAIN, and only call it parked if it stayed asleep.
    # A firmware park command that gates those owners is TODO RS-12.21;
    # until then PARK_OK means "asleep now and nothing re-armed it".
    SETTLE_S = 1.5
    try:
        write_reg(link, SX1276_REG_OP_MODE, OPMODE_LORA_STANDBY, timeout=0.5)
        time.sleep(0.05)
        write_reg(link, SX1276_REG_OP_MODE, OPMODE_LORA_SLEEP, timeout=0.5)
        time.sleep(0.05)
        first, _raw = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
        time.sleep(SETTLE_S)
        drain_pending(link, quiet_s=0.1, max_s=0.5)
        second, _raw = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
    except Exception as exc:
        print("PARK_FAIL " + json.dumps({"stage": "opmode", "err": str(exc)}),
              flush=True)
        return 3
    asleep = lambda v: (int(v) & 0x87) == (OPMODE_LORA_SLEEP & 0x87)
    info = {"opmode_readback": f"0x{int(first):02X}",
            "opmode_after_settle": f"0x{int(second):02X}",
            "settle_s": SETTLE_S}
    if asleep(first) and asleep(second):
        print("PARK_OK " + json.dumps(info), flush=True)
        return 0
    if asleep(first):
        # The write took, then a firmware RX owner re-armed the modem:
        # scan SM still walking or LOCKED. Not parked. Stop the daemons,
        # wait for the scan to fail, run again -- and verify later with
        # the read-only radio_state.py.
        print("PARK_TRANSIENT " + json.dumps(info), flush=True)
        return 5
    print("PARK_FAIL " + json.dumps(info), flush=True)
    return 4


if __name__ == "__main__":
    raise SystemExit(main())
