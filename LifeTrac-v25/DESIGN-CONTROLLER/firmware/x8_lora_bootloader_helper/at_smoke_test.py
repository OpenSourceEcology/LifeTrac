#!/usr/bin/env python3
"""
at_smoke_test.py — MKRWAN AT command smoke test for Murata CMWX1ZZABZ (L072).
Runs after flashing mlm32l07x01.bin via Method G pipeline.

Usage (on X8 via adb exec-out):
  python3 /tmp/lifetrac_p0c/at_smoke_test.py

Assumes:
  - L072 is already flashed and booted (firmware running, not in ROM bootloader)
  - /dev/ttymxc3 pre-configured to 115200 8N1 by the caller (or by this script via stty)
  - openocd NOT running (UART mux released)

Exit codes:
  0 = all mandatory checks pass
  1 = one or more checks fail
  2 = fatal: cannot open port or firmware unresponsive
"""
import os
import sys
import time
import subprocess

DEV = "/dev/ttymxc3"
# mlm32l07x01.bin debug UART runs at 19200 8N1 (same as ROM bootloader config).
# Standard MKRWAN AT modem firmware runs at 115200; update here if using that binary.
AT_BAUD = "19200"


def configure_uart():
    """Set UART to configured baud/8N1 raw for AT command communication."""
    ret = subprocess.run(
        ["stty", "-F", DEV, AT_BAUD, "cs8", "-parenb", "-cstopb", "raw", "-echo"],
        capture_output=True
    )
    if ret.returncode != 0:
        print(f"WARNING: stty returned {ret.returncode}: {ret.stderr.decode()}")


def open_port():
    configure_uart()
    fd = os.open(DEV, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    return fd


def drain(fd, timeout=0.3):
    """Discard any pending bytes."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            os.read(fd, 256)
        except BlockingIOError:
            time.sleep(0.05)


def send_cmd(fd, cmd: str):
    os.write(fd, (cmd + "\r\n").encode())


def read_response(fd, timeout=3.0, terminator=None):
    """
    Read until timeout or until `terminator` string appears in the accumulated response.
    Returns decoded string.
    """
    buf = b""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            chunk = os.read(fd, 256)
            if chunk:
                buf += chunk
                if terminator and terminator.encode() in buf:
                    break
        except BlockingIOError:
            time.sleep(0.02)
    return buf.decode(errors="replace")


def at(fd, cmd, timeout=3.0, terminator="OK"):
    """Send an AT command and return (raw_response, passed)."""
    drain(fd)
    send_cmd(fd, cmd)
    resp = read_response(fd, timeout=timeout, terminator=terminator)
    passed = "OK" in resp
    return resp.strip(), passed


# ---------------------------------------------------------------------------
# Test cases
# ---------------------------------------------------------------------------

def check_liveness(fd):
    """
    Read UART for 4s and check for any known firmware output.
    Passes with pinger firmware (TX PING, SX1276) OR AT modem (OK, +AT: OK).
    """
    resp = read_response(fd, timeout=4.0)
    ok = bool(resp.strip()) and (
        "TX PING" in resp or
        "SX1276" in resp or
        "PINGER" in resp or
        "OK" in resp or
        "+AT:" in resp
    )
    return ok, resp.strip()


def check_at(fd):
    resp, ok = at(fd, "AT")
    return ok, resp


def check_version(fd):
    resp, ok = at(fd, "AT+VERSION")
    return ok, resp


def check_deveui(fd):
    resp, ok = at(fd, "AT+DEVEUI")
    return ok, resp


def check_band(fd):
    """Query band — just check it returns OK, not a specific value."""
    resp, ok = at(fd, "AT+BAND")
    return ok, resp


def check_class(fd):
    resp, ok = at(fd, "AT+CLASS")
    return ok, resp


def check_dr(fd):
    resp, ok = at(fd, "AT+DR")
    return ok, resp


def check_join_otaa(fd):
    """
    Attempt OTAA join (will fail without a real network, but the firmware must
    respond with +ERR=... or JOIN_FAILED rather than hang, proving AT I/O works.
    """
    drain(fd)
    send_cmd(fd, "AT+JOIN")
    # Allow 12s for firmware to attempt + time out
    resp = read_response(fd, timeout=12.0, terminator="+ERR")
    # Acceptable outcomes: "+ERR=...", "Join failed", "no network", timeout with partial output
    ok = ("+ERR" in resp) or ("JOIN" in resp.upper()) or ("failed" in resp.lower())
    return ok, resp.strip()


def check_loopback(fd):
    """
    AT+TEST=TXLORA sends a raw LoRa packet (no network needed).
    Some MKRWAN firmware supports this; skip gracefully if not.
    """
    resp, ok = at(fd, "AT+TEST=TXLORA", timeout=5.0)
    # Acceptable: OK, +ERR (unsupported), or any non-empty response
    ok = ok or ("+ERR" in resp) or len(resp) > 0
    return ok, resp


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

TESTS = [
    # --- Liveness check (works with pinger firmware AND AT modem) ---
    # Any UART output including pinger banner proves firmware is running.
    ("Firmware liveness",  check_liveness,    True),   # mandatory
    # --- AT modem checks (only pass with proper MKRWAN AT modem firmware) ---
    # These will FAIL with mlm32l07x01.bin pinger firmware — that is expected.
    ("AT ping",        check_at,          False),  # optional for pinger fw
    ("AT+VERSION",     check_version,     False),  # optional
    ("AT+DEVEUI",      check_deveui,      False),  # optional
    ("AT+BAND",        check_band,        False),  # optional
    ("AT+CLASS",       check_class,       False),  # optional
    ("AT+DR",          check_dr,          False),  # optional
    ("AT+JOIN (OTAA)", check_join_otaa,   False),  # optional (network absent)
    ("AT+TEST=TXLORA", check_loopback,    False),  # optional
]

PASS_EMOJI = "PASS"
FAIL_EMOJI = "FAIL"
SKIP_EMOJI = "SKIP"

def main():
    print("=" * 60)
    print("MKRWAN AT Command Smoke Test")
    print(f"Device: {DEV} @ {AT_BAUD} 8N1")
    print("NOTE: mlm32l07x01.bin enters pinger mode on boot and may not respond")
    print("  to AT commands. FAIL on 'AT ping' is expected with pinger firmware.")
    print("  Use MKRWAN AT modem firmware (different binary) for full AT coverage.")
    print("=" * 60)

    try:
        fd = open_port()
    except Exception as e:
        print(f"FATAL: cannot open {DEV}: {e}")
        sys.exit(2)

    # Brief boot settle — firmware may still be printing startup banner
    time.sleep(1.0)
    drain(fd)

    results = []
    mandatory_failures = 0

    for name, fn, mandatory in TESTS:
        try:
            ok, resp = fn(fd)
        except Exception as e:
            ok = False
            resp = f"exception: {e}"

        status = PASS_EMOJI if ok else FAIL_EMOJI
        flag = " [mandatory]" if mandatory else ""
        print(f"  [{status}] {name}{flag}")
        # Show first two lines of response (keep output readable)
        for line in resp.splitlines()[:3]:
            print(f"          {line}")

        results.append((name, ok, mandatory))
        if not ok and mandatory:
            mandatory_failures += 1

    os.close(fd)

    print()
    print("=" * 60)
    total = len(results)
    passed = sum(1 for _, ok, _ in results if ok)
    print(f"Results: {passed}/{total} checks passed")
    if mandatory_failures:
        print(f"VERDICT: FAIL ({mandatory_failures} mandatory check(s) failed)")
        sys.exit(1)
    else:
        print("VERDICT: PASS (all mandatory checks passed)")
        sys.exit(0)


if __name__ == "__main__":
    main()
