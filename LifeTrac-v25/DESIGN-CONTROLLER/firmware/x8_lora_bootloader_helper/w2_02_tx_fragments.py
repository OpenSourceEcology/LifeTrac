"""W2-02 TX-side fragment sender.

Reads /tmp/w2_02_fragments.hex (one fragment-body per line, hex encoded)
and TXes each via the L072 HostLink TX_FRAME_REQ protocol, reusing the
same HostLink class + wait_for_tx_done helper that drive the W1-10b
two-board burst probe.

Output contract (stable, machine-parseable):
    __W2_02_TX_READY__ n_fragments=N
    __W2_02_TX_FRAG__ idx=K tx_id=0xHH status=N(NAME) toa_us=U
                       elapsed_ms=M payload_hex=...
    __W2_02_TX_TIMEOUT__ idx=K tx_id=0xHH
    __W2_02_TX_DONE__ n=N ok=K fail=F timeout=T

Pure dependency on method_h_stage2_tx_probe.py — keeps the host-orchestrator
and the bench tooling on one toolkit. Single board, no MQTT.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

# Same directory layout as the W1-10b orchestrator pushes: helper toolkit
# lives at /tmp/lifetrac_p0c/ with method_h_stage2_tx_probe.py alongside.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from method_h_stage2_tx_probe import (  # noqa: E402
    HostLink,
    HOST_TYPE_VER_REQ,
    HOST_TYPE_VER_URC,
    HOST_TYPE_CFG_SET_REQ,
    HOST_TYPE_CFG_OK_URC,
    HOST_TYPE_TX_FRAME_REQ,
    CFG_KEY_LBT_ENABLE,
    SX1276_TX_STATUS_OK,
    drain_boot,
    drain_pending,
    wait_for_tx_done,
)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--uart", default="/dev/ttymxc3")
    ap.add_argument("--baud", default="921600")  # str: configure_uart calls stty
    ap.add_argument("--fragments", default="/tmp/w2_02_fragments.hex")
    ap.add_argument("--inter-s", type=float, default=0.05,
                    help="inter-fragment delay (seconds)")
    ap.add_argument("--timeout", type=float, default=3.0,
                    help="per-fragment TX_DONE_URC timeout (seconds)")
    args = ap.parse_args(argv)

    # Load fragment hex blobs.
    if not os.path.exists(args.fragments):
        print(f"FATAL: fragments file not found: {args.fragments}")
        print("__W2_02_TX_VERDICT__=NO_INPUT")
        return 2
    fragments: list[bytes] = []
    with open(args.fragments, "r", encoding="ascii") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            try:
                fragments.append(bytes.fromhex(line))
            except ValueError as exc:
                print(f"FATAL: bad hex line: {exc}")
                print("__W2_02_TX_VERDICT__=BAD_INPUT")
                return 2
    n = len(fragments)
    if n == 0:
        print("FATAL: zero fragments to send")
        print("__W2_02_TX_VERDICT__=NO_INPUT")
        return 2

    print(f"=== W2-02 TX: {n} fragments, uart={args.uart} ===")
    link = HostLink(args.uart, args.baud)
    drain_boot(link, 1.0)
    try:
        link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
    except Exception as exc:
        print(f"FATAL: VER warm-up failed: {exc}")
        print("__W2_02_TX_VERDICT__=TRANSPORT_FAIL")
        return 2
    drain_pending(link, quiet_s=0.25, max_s=1.0)

    # LBT off (matches W1-10b TX_BURST rationale).
    try:
        link.request(HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
                     bytes([CFG_KEY_LBT_ENABLE, 0x01, 0x00]), timeout=1.0)
        print("CFG_OK_URC: LBT_ENABLE=0")
    except Exception as exc:
        print(f"WARN: CFG_SET_REQ(LBT_ENABLE=0) failed: {exc}")

    print(f"__W2_02_TX_READY__ n_fragments={n}")
    sys.stdout.flush()

    ok = 0
    fail = 0
    timeout_n = 0
    for idx, body in enumerate(fragments):
        tx_id = idx & 0xFF
        if len(body) > 64:
            # L072 firmware caps TX payloads at 64 B (matches W1-10b
            # run_tx_burst clamp). Encoder should already respect this
            # via the host-side fragmenter; this is a guard rail.
            print(f"FATAL: fragment {idx} length {len(body)} > 64 B cap")
            print("__W2_02_TX_VERDICT__=OVERSIZE")
            return 2
        tx_frame = bytes([tx_id, len(body)]) + body
        t_send = time.time()
        try:
            link.send(HOST_TYPE_TX_FRAME_REQ, tx_frame)
        except Exception as exc:
            print(f"__W2_02_TX_SEND_ERR__ idx={idx} {exc}")
            timeout_n += 1
            continue
        try:
            done, faults = wait_for_tx_done(link, tx_id, timeout=args.timeout)
        except TimeoutError as exc:
            print(f"__W2_02_TX_TIMEOUT__ idx={idx} tx_id=0x{tx_id:02X} {exc}")
            timeout_n += 1
            if args.inter_s > 0:
                time.sleep(args.inter_s)
            continue
        except Exception as exc:
            print(f"__W2_02_TX_ERR__ idx={idx} {exc}")
            timeout_n += 1
            if args.inter_s > 0:
                time.sleep(args.inter_s)
            continue
        elapsed_ms = (time.time() - t_send) * 1000.0
        if done["status"] == SX1276_TX_STATUS_OK:
            ok += 1
        else:
            fail += 1
        print(f"__W2_02_TX_FRAG__ idx={idx} tx_id=0x{tx_id:02X} "
              f"status={done['status']}({done['status_name']}) "
              f"toa_us={done['time_on_air_us']} "
              f"elapsed_ms={elapsed_ms:.1f} "
              f"payload_hex={body.hex()}")
        sys.stdout.flush()
        for f in faults:
            print(f"__W2_02_TX_FAULT__ idx={idx} {f}")
        if args.inter_s > 0:
            time.sleep(args.inter_s)

    print(f"__W2_02_TX_DONE__ n={n} ok={ok} fail={fail} timeout={timeout_n}")
    return 0 if (ok == n and fail == 0 and timeout_n == 0) else 1


if __name__ == "__main__":
    sys.exit(main())
