#!/usr/bin/env python3
"""Park the SX1276 in LoRa SLEEP (RegOpMode = 0x80) and verify by readback.

Standalone, idempotent bench EMC utility. Unlike the existing
`method_h_stage2_tx_probe_v2.py --probe rx_listen --rx-window 0` trick (which
auto-wakes the radio to RXCONT to fetch stats before sleeping it again), this
tool:

  1. Opens the host_link UART on /dev/ttymxc3.
  2. Drains boot URCs (if the L072 just rebooted).
  3. Reads RegOpMode once (no wake).
  4. If already 0x80 → reports SLEEP_ALREADY and exits 0.
     Otherwise → writes 0x80, reads back, reports SLEEP_OK (exit 0) or
     SLEEP_FAIL_READBACK (exit 1).
  5. Closes the link.

Markers emitted (stable, grep-friendly for orchestrators):
  __RADIO_SLEEP_PRE__   opmode=0xNN
  __RADIO_SLEEP_RESULT__ status=<SLEEP_ALREADY|SLEEP_OK|SLEEP_FAIL_READBACK|
                                  SLEEP_FAIL_WRITE|LINK_FAIL>
                         opmode_pre=0xNN opmode_post=0xNN (or omitted)
                         dev=<path> serial=<x8-adb-serial-or-unknown>

Exit codes:
  0 = radio confirmed at 0x80 after run.
  1 = link opened but write or readback failed (firmware/SX1276 issue).
  2 = link did not open (UART permission, /dev path wrong, no L072 firmware).
"""
import argparse
import os
import sys
import time

# Import HostLink + helpers from the v2 probe (already deployed to
# /tmp/lifetrac_p0c/ on every bench X8). Keeps a single source of truth for
# host_link wire format and SX1276 register helpers.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import method_h_stage2_tx_probe_v2 as m  # noqa: E402


def _x8_serial_hint() -> str:
    """Best-effort identify which X8 we're running on (for the audit line).
    Returns empty string if no serial can be read."""
    candidates = (
        "/sys/devices/soc0/serial_number",
        "/proc/device-tree/serial-number",
    )
    for path in candidates:
        try:
            with open(path, "rb") as fh:
                raw = fh.read().strip().rstrip(b"\x00").decode("ascii", "replace")
                if raw:
                    return raw
        except OSError:
            continue
    return ""


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Park SX1276 in LoRa SLEEP (RegOpMode=0x80).")
    parser.add_argument("--dev", default="/dev/ttymxc3",
                        help="L072 host_link UART (default /dev/ttymxc3).")
    parser.add_argument("--baud", default="921600",
                        help="host_link baud (default 921600).")
    parser.add_argument("--boot-settle-s", type=float, default=1.0,
                        help="seconds to drain boot URCs before probing "
                             "(default 1.0; set 0 to skip).")
    parser.add_argument("--reg-timeout-s", type=float, default=0.5,
                        help="REG_READ/REG_WRITE per-call timeout "
                             "(default 0.5).")
    args = parser.parse_args(argv)

    serial = _x8_serial_hint() or "unknown"
    base_suffix = f"dev={args.dev} serial={serial}"

    # Step 1: open link.
    try:
        link = m.HostLink(args.dev, args.baud)
    except Exception as exc:
        print(f"__RADIO_SLEEP_RESULT__ status=LINK_FAIL reason={exc!s} "
              f"{base_suffix}")
        return 2

    try:
        # Step 2: drain boot URCs if requested.
        if args.boot_settle_s > 0:
            try:
                m.drain_boot(link, args.boot_settle_s)
            except Exception:
                # Drain failures are non-fatal — we still try the register op.
                pass
            # Drain any URCs that arrived during settle so REG_READ_REQ is
            # matched against a fresh URC stream.
            try:
                m.drain_pending(link, quiet_s=0.2, max_s=0.8)
            except Exception:
                pass

        # Step 3: read RegOpMode (no wake).
        try:
            opm_pre, _ = m.read_reg(link, m.SX1276_REG_OP_MODE,
                                    timeout=args.reg_timeout_s)
        except Exception as exc:
            print(f"__RADIO_SLEEP_RESULT__ status=SLEEP_FAIL_WRITE "
                  f"reason=read_pre_failed:{exc!s} {base_suffix}")
            return 1
        print(f"__RADIO_SLEEP_PRE__ opmode=0x{opm_pre:02X}")

        target = m.SX1276_OPMODE_LORA_SLEEP_VAL  # 0x80
        if opm_pre == target:
            print(f"__RADIO_SLEEP_RESULT__ status=SLEEP_ALREADY "
                  f"opmode_pre=0x{opm_pre:02X} opmode_post=0x{opm_pre:02X} "
                  f"{base_suffix}")
            return 0

        # Step 4: write target, then verify.
        try:
            ack = m.write_reg(link, m.SX1276_REG_OP_MODE, target,
                              timeout=args.reg_timeout_s)
        except Exception as exc:
            print(f"__RADIO_SLEEP_RESULT__ status=SLEEP_FAIL_WRITE "
                  f"reason=write_failed:{exc!s} opmode_pre=0x{opm_pre:02X} "
                  f"{base_suffix}")
            return 1

        # Small settle so SX1276 latches the mode transition before readback.
        time.sleep(0.01)

        try:
            opm_post, _ = m.read_reg(link, m.SX1276_REG_OP_MODE,
                                     timeout=args.reg_timeout_s)
        except Exception as exc:
            print(f"__RADIO_SLEEP_RESULT__ status=SLEEP_FAIL_READBACK "
                  f"reason=read_post_failed:{exc!s} "
                  f"opmode_pre=0x{opm_pre:02X} ack={ack.hex()} {base_suffix}")
            return 1

        if opm_post != target:
            print(f"__RADIO_SLEEP_RESULT__ status=SLEEP_FAIL_READBACK "
                  f"opmode_pre=0x{opm_pre:02X} opmode_post=0x{opm_post:02X} "
                  f"target=0x{target:02X} ack={ack.hex()} {base_suffix}")
            return 1

        print(f"__RADIO_SLEEP_RESULT__ status=SLEEP_OK "
              f"opmode_pre=0x{opm_pre:02X} opmode_post=0x{opm_post:02X} "
              f"ack={ack.hex()} {base_suffix}")
        return 0
    finally:
        try:
            link.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
