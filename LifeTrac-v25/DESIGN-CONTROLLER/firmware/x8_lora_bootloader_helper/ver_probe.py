"""One-shot L072 VER probe (run inside board container).

Pulses nothing itself — assumes caller already NRST'd if desired.
Prints VER-OK <payload hex> or VER-FAIL <reason> and exits 0/1.
"""
import sys

sys.path.insert(0, "/work")

from method_h_stage2_tx_probe_v2 import (  # noqa: E402
    HOST_TYPE_VER_REQ,
    HOST_TYPE_VER_URC,
    HostLink,
    drain_boot,
)


def main() -> int:
    link = HostLink("/dev/ttymxc3", "921600")
    if "--reset" in sys.argv:
        print("sending HOST_TYPE_RESET_REQ (0x03)...")
        link.send(0x03)
        drain_boot(link, settle_s=3.0)
    try:
        drain_boot(link, settle_s=0.25)
    except Exception as exc:
        print("drain warn:", exc)
    try:
        resp = link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=2.0)
    except Exception as exc:
        print("VER-FAIL", exc)
        # Forensic dump: whatever DID arrive (methodology note 2026-05-22).
        try:
            for f in link.urc_queue:
                print("  urc type=0x%02x len=%d" % (f.get("type", 0), len(f.get("payload", b""))))
            link.send(HOST_TYPE_VER_REQ)
            raw = link.read_raw(timeout=2.0, max_bytes=512)
            print("  raw after 2nd VER_REQ: %d bytes: %s" % (len(raw), raw.hex()))
        except Exception as exc2:
            print("  forensic dump failed:", exc2)
        return 1
    payload = resp.get("payload", b"") if isinstance(resp, dict) else bytes(resp or b"")
    print("VER-OK", payload.hex())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
