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
    try:
        write_reg(link, SX1276_REG_OP_MODE, OPMODE_LORA_STANDBY, timeout=0.5)
        time.sleep(0.05)
        write_reg(link, SX1276_REG_OP_MODE, OPMODE_LORA_SLEEP, timeout=0.5)
        time.sleep(0.05)
        got, _raw = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
    except Exception as exc:
        print("PARK_FAIL " + json.dumps({"stage": "opmode", "err": str(exc)}),
              flush=True)
        return 3
    ok = (int(got) & 0x87) == OPMODE_LORA_SLEEP & 0x87
    print(("PARK_OK " if ok else "PARK_FAIL ")
          + json.dumps({"opmode_readback": f"0x{int(got):02X}"}), flush=True)
    return 0 if ok else 4


if __name__ == "__main__":
    raise SystemExit(main())
