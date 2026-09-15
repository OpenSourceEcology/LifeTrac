#!/usr/bin/env python3
"""radio_state.py — READ-ONLY report of the SX1276's current mode.

Deliberately writes nothing: answering "is the radio off?" by setting it
off would prove nothing. Reads RegOpMode (0x01) and decodes it.
"""
from __future__ import annotations

import json
import sys

sys.path.insert(0, "/work")

from method_h_stage2_tx_probe_v2 import (  # type: ignore
    HostLink,
    HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC,
    SX1276_REG_OP_MODE,
    read_reg,
    drain_boot, drain_pending,
)

MODES = {0: "SLEEP", 1: "STANDBY", 2: "FSTX", 3: "TX",
         4: "FSRX", 5: "RXCONT", 6: "RXSINGLE", 7: "CAD"}


def main() -> int:
    uart = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttymxc3"
    link = HostLink(uart, "921600")
    drain_boot(link, settle_s=0.25)
    try:
        link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
    except Exception as exc:
        print("STATE_FAIL " + json.dumps({"err": str(exc)}), flush=True)
        return 2
    drain_pending(link, quiet_s=0.25, max_s=1.0)
    val, _raw = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
    val = int(val)
    mode = MODES.get(val & 0x07, "?")
    print("RADIO_STATE " + json.dumps({
        "reg_op_mode": f"0x{val:02X}",
        "mode": mode,
        "lora": bool(val & 0x80),
        "transmitting_or_listening": mode in ("TX", "RXCONT", "RXSINGLE",
                                              "FSTX", "FSRX", "CAD"),
    }), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
