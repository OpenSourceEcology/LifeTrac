#!/usr/bin/env python3
"""
2026-05-25 — RX-side VER warm-up isolation diagnostic.

Purpose: falsify hypotheses about why image_rx_daemon's VER warm-up times
out on 2D0A after gpio163 NRST + 1.5 s settle, without dragging MQTT, the
TX peer, or RXCONT autowake into the picture.

This script does NOT pulse gpio163 — the caller is expected to have done
that (or to skip NRST entirely). It only opens the HostLink, optionally
issues the daemon's 0x03 RESET_REQ, drains, and requests VER, recording
how each attempt behaves.

Sweep matrix executed per invocation:
  - For each attempt N (1..--attempts):
      open link, optionally send RESET_REQ, drain for settle_s, VER req
      record result (ok / urc count / latency / fault payloads / partial
      COBS warning count) and close link.

Output: one JSON line per attempt to stdout for machine parsing, plus a
final summary table.

Run inside the daemon container:
    docker run --rm --device=/dev/ttymxc3 -v /tmp/lifetrac_strict:/work \\
        -w /work --network=host \\
        -e PYTHONPATH=/work:/work/paho --entrypoint timeout \\
        hub.foundries.io/arduino/arduino-ootb-python-devel:738bc44 30 \\
        python3 -u /work/rx_ver_warmup_diag.py \\
            --attempts 3 --settle-s 1.5 --skip-reset-req
"""
from __future__ import annotations

import argparse
import json
import logging
import sys
import time
from typing import Any, Dict, List

# Mirror image_rx_daemon's import paths — caller must set PYTHONPATH so
# both the daemon dir and method_h_stage2_tx_probe_v2 are importable.
from method_h_stage2_tx_probe_v2 import (  # type: ignore
    HostLink,
    HOST_TYPE_FAULT_URC,
    HOST_TYPE_STATS_URC,
    HOST_TYPE_VER_REQ,
    HOST_TYPE_VER_URC,
    format_fault_payload,
)

LOG = logging.getLogger("rx_ver_diag")


def _drain_and_classify(link: HostLink, settle_s: float) -> Dict[str, Any]:
    """Read every frame for `settle_s` seconds; classify and count."""
    deadline = time.time() + settle_s
    counts: Dict[str, int] = {
        "boot_urc": 0,
        "fault_urc": 0,
        "stats_urc": 0,
        "other": 0,
    }
    fault_payloads: List[str] = []
    while time.time() < deadline:
        for frame in link.read_frames(0.2):
            ftype = frame["type"]
            if ftype == 0xF0:
                counts["boot_urc"] += 1
            elif ftype == HOST_TYPE_FAULT_URC:
                counts["fault_urc"] += 1
                fault_payloads.append(format_fault_payload(frame["payload"]))
            elif ftype == HOST_TYPE_STATS_URC:
                counts["stats_urc"] += 1
            else:
                counts["other"] += 1
    return {"counts": counts, "fault_payloads": fault_payloads}


def attempt_ver_warmup(
    uart: str,
    baud: str,
    *,
    send_reset_req: bool,
    settle_s: float,
    ver_timeout_s: float,
) -> Dict[str, Any]:
    """One open-link/(reset)/drain/VER cycle. Returns a result dict."""
    result: Dict[str, Any] = {
        "uart": uart,
        "baud": baud,
        "send_reset_req": send_reset_req,
        "settle_s": settle_s,
        "ver_timeout_s": ver_timeout_s,
    }
    t_open = time.time()
    link = HostLink(uart, baud)
    result["open_latency_s"] = round(time.time() - t_open, 4)
    try:
        if send_reset_req:
            try:
                link.send(0x03)  # HOST_TYPE_RESET_REQ
                result["reset_req_sent"] = True
            except Exception as exc:
                result["reset_req_sent"] = False
                result["reset_req_error"] = str(exc)
        else:
            result["reset_req_sent"] = False

        drain = _drain_and_classify(link, settle_s)
        result["drain"] = drain

        t_ver = time.time()
        try:
            link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC,
                         timeout=ver_timeout_s)
            result["ver_ok"] = True
            result["ver_latency_s"] = round(time.time() - t_ver, 4)
        except Exception as exc:
            result["ver_ok"] = False
            result["ver_latency_s"] = round(time.time() - t_ver, 4)
            result["ver_error"] = str(exc)
    finally:
        try:
            link.close()
        except Exception:
            pass
    return result


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--uart", default="/dev/ttymxc3")
    # HostLink.__init__ declares baud as str (it forwards to stty), so keep
    # this as a string to match image_rx_daemon's argparse default.
    parser.add_argument("--baud", default="921600")
    parser.add_argument("--attempts", type=int, default=3,
                        help="Number of independent open/VER cycles per run.")
    parser.add_argument("--settle-s", type=float, default=1.5,
                        help="Drain duration before VER_REQ.")
    parser.add_argument("--ver-timeout-s", type=float, default=1.0)
    parser.add_argument("--skip-reset-req", action="store_true",
                        help="Do NOT send 0x03 RESET_REQ — rely on external "
                             "gpio163 NRST pulse done by caller. Default is "
                             "to mimic image_rx_daemon._open_link().")
    parser.add_argument("--inter-attempt-s", type=float, default=0.5,
                        help="Sleep between attempts.")
    parser.add_argument("--log-level", default="INFO")
    args = parser.parse_args(argv)

    logging.basicConfig(level=args.log_level,
                        format="%(asctime)s %(levelname)s %(message)s")

    results: List[Dict[str, Any]] = []
    for i in range(1, args.attempts + 1):
        LOG.info("attempt %d/%d (send_reset_req=%s settle_s=%.2f)",
                 i, args.attempts,
                 (not args.skip_reset_req), args.settle_s)
        try:
            r = attempt_ver_warmup(
                args.uart,
                args.baud,
                send_reset_req=(not args.skip_reset_req),
                settle_s=args.settle_s,
                ver_timeout_s=args.ver_timeout_s,
            )
        except Exception as exc:
            r = {"attempt": i, "fatal": str(exc)}
        r["attempt"] = i
        results.append(r)
        print("RESULT_JSON " + json.dumps(r, sort_keys=True))
        sys.stdout.flush()
        if i < args.attempts:
            time.sleep(args.inter_attempt_s)

    ok_count = sum(1 for r in results if r.get("ver_ok"))
    print("\n=== SUMMARY ===")
    print(f"ver_ok: {ok_count}/{len(results)}")
    print(f"settle_s={args.settle_s} send_reset_req={not args.skip_reset_req}"
          f" ver_timeout_s={args.ver_timeout_s}")
    for r in results:
        if "fatal" in r:
            print(f"  attempt {r['attempt']}: FATAL {r['fatal']}")
            continue
        drain = r.get("drain", {}).get("counts", {})
        faults = r.get("drain", {}).get("fault_payloads", [])
        print(f"  attempt {r['attempt']}: "
              f"ver_ok={r.get('ver_ok')} "
              f"ver_latency_s={r.get('ver_latency_s')} "
              f"drain={drain} "
              f"faults={faults[:2]}{'…' if len(faults) > 2 else ''}")
    return 0 if ok_count == len(results) else 1


if __name__ == "__main__":
    sys.exit(main())
