"""FHSS RX acquisition probe (run inside base-board container).

Activates REG_PROFILE=1 (wide mask) on the base L072, then listens for
`duration_s` printing EVERY host frame that arrives: RX_FRAME_URCs,
FAULT_URCs (with the RX_SCAN_FAILED sub-byte decoded — got_any_irq /
crc_seen / ever_locked bits), STATS, everything. Run while the tractor
TX daemon is hopping under profile 1.

Usage: python3 -u fhss_rx_probe.py [duration_s]
"""
import os
import sys
import time

sys.path.insert(0, "/work")

from method_h_stage2_tx_probe_v2 import (  # noqa: E402
    HostLink,
    drain_boot,
    configure_regulatory_profile_if_needed,
    HOST_TYPE_VER_REQ,
    HOST_TYPE_VER_URC,
    HOST_TYPE_RX_FRAME_URC,
)
from method_g_stage1_probe import (  # noqa: E402
    format_fault_payload,
    HOST_TYPE_FAULT_URC,
)


def main() -> int:
    duration_s = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    os.environ.setdefault("LIFETRAC_REG_PROFILE", "1")
    os.environ.setdefault("LIFETRAC_FHSS_WIDE_MASK", "1")

    link = HostLink("/dev/ttymxc3", "921600")
    drain_boot(link, settle_s=0.3)
    for attempt in range(5):
        try:
            link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=2.0)
            print("VER ok (attempt %d)" % (attempt + 1))
            break
        except Exception as exc:
            print("VER retry:", exc)
    configure_regulatory_profile_if_needed(link)

    print("=== listening %.0fs; dumping every URC ===" % duration_s)
    t0 = time.monotonic()
    counts = {}
    while time.monotonic() - t0 < duration_s:
        for f in link.read_frames(0.5):
            t = f.get("type", 0)
            counts[t] = counts.get(t, 0) + 1
            if t == HOST_TYPE_FAULT_URC:
                print("[%6.1fs] FAULT %s" % (time.monotonic() - t0,
                                             format_fault_payload(f.get("payload", b""))))
            elif t == HOST_TYPE_RX_FRAME_URC:
                pl = f.get("payload", b"")
                print("[%6.1fs] RX_FRAME len=%d" % (time.monotonic() - t0, len(pl)))
            else:
                print("[%6.1fs] urc type=0x%02X len=%d" % (
                    time.monotonic() - t0, t, len(f.get("payload", b""))))
    print("=== counts by type:", {hex(k): v for k, v in sorted(counts.items())})
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
