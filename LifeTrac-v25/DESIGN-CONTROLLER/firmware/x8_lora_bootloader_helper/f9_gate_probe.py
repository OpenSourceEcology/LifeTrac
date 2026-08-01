"""F9 on-air acceptance probe. Run on an X8 against a flag=0 L072 build.

Asserts the host_reg_gate policy over the wire:
  1. write_reg(0x01, 0x85) RXCONT      -> ACK   (production value, always)
  2. write_reg(0x01, 0x81) STANDBY     -> ACK
  3. write_reg(0x1D, 0x72) ModemConfig -> ERR_PROTO FORBIDDEN (diag surface closed)
  4. write_reg(0x01, 0x83) raw TX      -> ERR_PROTO FORBIDDEN (value gate)
Exit 0 iff all four behave.
"""
import sys

sys.path.insert(0, "/work")
sys.path.insert(0, "/work/paho")

from method_h_stage2_tx_probe_v2 import HostLink, write_reg, drain_boot  # noqa: E402

DEV = "/dev/ttymxc3"
BAUD = "921600"

link = HostLink(DEV, BAUD)
drain_boot(link)

failures = []

def expect_ack(addr, val, label):
    try:
        write_reg(link, addr, val)
        print(f"PASS {label}: reg 0x{addr:02X} <- 0x{val:02X} ACKed")
    except Exception as exc:
        failures.append(label)
        print(f"FAIL {label}: expected ACK, got {exc!r}")

def expect_forbidden(addr, val, label):
    try:
        write_reg(link, addr, val)
        failures.append(label)
        print(f"FAIL {label}: reg 0x{addr:02X} <- 0x{val:02X} was ACCEPTED "
              f"(diag surface open?)")
    except RuntimeError as exc:
        msg = str(exc)
        if "ERR_PROTO" in msg:
            print(f"PASS {label}: refused as expected ({msg[:60]})")
        else:
            failures.append(label)
            print(f"FAIL {label}: wrong error {msg[:80]}")

expect_ack(0x01, 0x85, "rxcont-arm")
expect_ack(0x01, 0x81, "standby")
expect_forbidden(0x1D, 0x72, "modemconfig1-diag")
expect_forbidden(0x01, 0x83, "raw-tx-value-gate")
# leave the radio armed the way the daemons expect
expect_ack(0x01, 0x85, "re-arm")

print("F9_PROBE_RESULT:", "PASS" if not failures else f"FAIL {failures}")
sys.exit(0 if not failures else 1)
