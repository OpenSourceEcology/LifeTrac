#!/usr/bin/env python3
"""radio_park.py — put the L072's SX1276 into LoRa SLEEP and verify.

Mirrors channel_survey_sniff.py's link bring-up exactly (HostLink, boot
drain, VER handshake, drain), parks the radio by one of two paths, then
reads RegOpMode back twice, SETTLE_S apart. Run in the daemon-shaped
container on either board.

Paths (2026-10-24, RS-12.21):
  firmware  the park command (HOST_TYPE_RADIO_PARK_REQ, answered by
            RADIO_PARK_URC). The L072 sets its host_parked flag, which gates
            every RX arm path (scan walker, gamma-1 retune, slot follower,
            the post-TX re-arm, the scan restarts), and puts the modem to
            SLEEP. PERSISTENT: it holds until a host raw RXCONT write, a
            profile activation (a daemon start) or a reset.
  register  an image from before the command: it answers ERR_PROTO
            UNKNOWN_TYPE, or nothing within RADIO_PARK_TIMEOUT_S (any other
            ERR_PROTO for the command lands here too, with its text). The
            old STANDBY -> SLEEP RegOpMode writes. TRANSIENT: a firmware RX
            owner that still has work re-arms RXCONT (receive-only), so on
            such an image it holds only once the scan has failed out into
            FAILED, which parked it, up to ~2 min after the daemons stop
            (PR #125 review).
The VER reply picks the path (2026-10-24 review): its capability bitmap's
bit 7 (HOST_CAP_RADIO_PARK) marks the image with the command. That image
also restarts its scan after a final RX_SCAN_FAILED (scan option (b)): once
the daemons stop it scans on, re-arming RXCONT every 2 s dwell, so a
register SLEEP write cannot hold on it. It gets the firmware path only: the
command up to PARK_CMD_TRIES times, a late answer to an earlier try counting
as the answer, and PARK_FAIL (no register write) if none comes. An image
without the bit, or a VER reply too short to say, gets the command once and,
unanswered or refused as unknown, the register path.

Output: the LAST line (callers read `tail -1`) is PARK_OK, PARK_TRANSIENT
or PARK_FAIL and a JSON object: "path" (firmware / register), "persistent"
(true only for a firmware park that held through the settle), "holds"
(until what), "opmode_readback" (the park URC's readback, or the register
path's first read), "opmode_after_settle", "settle_s", "cap_bitmap" (the VER
reply's, "0x000000FF"; null when it carried none), "park_tries" (park
commands sent), and "park_status" (firmware path: PARKED / ALREADY_PARKED /
REFUSED) or "park_cmd" (the command's answer when it did not park:
unknown_type / no_reply / err_proto, with "park_cmd_err"); "late": true when
the answer came to an earlier try; "reason" on a firmware-path PARK_FAIL
without a park status.

Exit codes (unchanged): 0 PARK_OK (asleep at both reads); 2 the VER
handshake failed; 3 a stage raised (the park command's send, a register
write or read); 4 PARK_FAIL (not asleep at the first read, the firmware
refused the park, or an image that advertises the command did not answer
it); 5 PARK_TRANSIENT (asleep, then re-armed within the settle. Register
path, an image from before the command: the scan was still walking -- wait
~60 s and park again, as leg_post.sh does. Firmware path: the gate missed an
arm path, a firmware defect).
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
from method_g_stage1_probe import (  # type: ignore
    HOST_CAP_RADIO_PARK,
    RADIO_PARK_STATUS_PARKED, RADIO_PARK_STATUS_ALREADY_PARKED,
    RADIO_PARK_TIMEOUT_S,
    parse_version,
)

OPMODE_LORA_STANDBY = 0x81
OPMODE_LORA_SLEEP = 0x80

# Longer than the longest a firmware RX owner that still has work takes to
# re-arm RXCONT on any image: a scan channel dwell (2 s since RS-12.21 d2,
# 500 ms before) or a 200 ms slot, with margin. (1.5 s until the 2026-10-24
# review: shorter than the 2 s dwell, so a register park could read PARK_OK
# before the walker's next step re-armed it.)
SETTLE_S = 2.5
PARK_CMD_TIMEOUT_S = RADIO_PARK_TIMEOUT_S
# An image that advertises the command: tries before PARK_FAIL.
PARK_CMD_TRIES = 3

HOLDS_FIRMWARE = "until a host RXCONT write, a profile activation or a reset"
HOLDS_REGISTER = ("until a firmware RX owner re-arms it: the scan walker "
                  "until the scan fails out into FAILED, or a host RXCONT write")


def asleep(value) -> bool:
    return (int(value) & 0x87) == (OPMODE_LORA_SLEEP & 0x87)


def _hex(value) -> "str | None":
    return None if value is None else f"0x{int(value):02X}"


def _cap_bitmap(ver_frame) -> "int | None":
    """The VER reply's capability bitmap, or None when it carries none."""
    try:
        return parse_version(bytes(ver_frame["payload"]))["cap_bitmap"]
    except (KeyError, TypeError, ValueError):
        return None


def _settle_and_report(link, first: int, info: dict) -> int:
    """Wait SETTLE_S, read RegOpMode again and print the verdict line."""
    try:
        time.sleep(SETTLE_S)
        drain_pending(link, quiet_s=0.1, max_s=0.5)
        second, _raw = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
    except Exception as exc:
        print("PARK_FAIL " + json.dumps(dict(info, stage="opmode",
                                             err=str(exc))), flush=True)
        return 3
    info.update(opmode_readback=_hex(first), opmode_after_settle=_hex(second),
                settle_s=SETTLE_S)
    if asleep(first) and asleep(second):
        info["persistent"] = info["path"] == "firmware"
        print("PARK_OK " + json.dumps(info), flush=True)
        return 0
    info["persistent"] = False
    if asleep(first):
        # Asleep, then re-armed within the settle. Register path (an image
        # from before the command): the scan SM was still walking (or
        # LOCKED) -- stop the daemons, wait for the scan to fail out, run
        # again, verify later with the read-only radio_state.py. Firmware
        # path: the park did not gate that arm.
        print("PARK_TRANSIENT " + json.dumps(info), flush=True)
        return 5
    print("PARK_FAIL " + json.dumps(info), flush=True)
    return 4


def _firmware_park(link, park: dict, info: dict) -> int:
    info.update(path="firmware", persistent=False, holds=HOLDS_FIRMWARE,
                park_status=park["status_name"])
    if park["answer_seq"] != park["seq"]:
        info["late"] = True
    if park["status"] not in (RADIO_PARK_STATUS_PARKED,
                              RADIO_PARK_STATUS_ALREADY_PARKED):
        # REFUSED (or a status this tool does not know): the firmware kept
        # the radio as it was. Not parked; run again once it is idle.
        info.update(opmode_readback=_hex(park["opmode"]))
        print("PARK_FAIL " + json.dumps(info), flush=True)
        return 4
    first = park["opmode"]
    if first is None:                     # a short URC: read it ourselves
        try:
            first, _raw = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
        except Exception as exc:
            print("PARK_FAIL " + json.dumps(dict(info, stage="opmode",
                                                 err=str(exc))), flush=True)
            return 3
    return _settle_and_report(link, first, info)


def _register_park(link, park: dict, info: dict) -> int:
    # PR #125 review (2026-09-15): the immediate readback proves only the
    # instant of the write. The firmware's own RX owners -- the scan walker
    # (ADVANCE_CHANNEL), the gamma-1 retune tick and, when LOCKED, the slot
    # follower -- all call sx1276_rx_arm() and will put the modem back into
    # RXCONT (receive-only) if any of them still has work. On an image
    # without the park command the park only holds once the scan SM has
    # exhausted into FAILED (nothing left to arm), which is the normal
    # state some tens of seconds after the daemons stop. So: write, wait
    # SETTLE_S -- longer than a scan channel dwell (2 s since RS-12.21 d2,
    # 500 ms before) and a slot (200 ms) -- read AGAIN, and only call it
    # parked if it stayed asleep: PARK_OK on this path means "asleep now
    # and nothing re-armed it".
    info.update(path="register", persistent=False, holds=HOLDS_REGISTER,
                park_cmd=park["answer"])
    if park["detail"]:
        info["park_cmd_err"] = park["detail"]
    try:
        write_reg(link, SX1276_REG_OP_MODE, OPMODE_LORA_STANDBY, timeout=0.5)
        time.sleep(0.05)
        write_reg(link, SX1276_REG_OP_MODE, OPMODE_LORA_SLEEP, timeout=0.5)
        time.sleep(0.05)
        first, _raw = read_reg(link, SX1276_REG_OP_MODE, timeout=0.5)
    except Exception as exc:
        print("PARK_FAIL " + json.dumps(dict(info, stage="opmode",
                                             err=str(exc))), flush=True)
        return 3
    return _settle_and_report(link, first, info)


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    uart = argv[0] if argv else "/dev/ttymxc3"
    link = HostLink(uart, "921600")
    drain_boot(link, settle_s=0.25)
    try:
        ver = link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
    except Exception as exc:
        print("PARK_FAIL " + json.dumps({"stage": "ver", "err": str(exc)}),
              flush=True)
        return 2
    cap = _cap_bitmap(ver)
    has_cmd = cap is not None and bool(cap & HOST_CAP_RADIO_PARK)
    drain_pending(link, quiet_s=0.25, max_s=1.0)
    # RS-12.21 (2026-10-24): the firmware park command first. An image that
    # answers it parks persistently; one from before it says UNKNOWN_TYPE
    # (or nothing) and gets the register writes it has always had. An image
    # that advertises it (VER cap bit 7) is asked again on silence, and an
    # earlier try's late answer counts (2026-10-24 review).
    seqs = []
    try:
        for _try in range(PARK_CMD_TRIES if has_cmd else 1):
            park = link.radio_park(timeout=PARK_CMD_TIMEOUT_S, earlier_seqs=seqs)
            seqs.append(park["seq"])
            if park["answer"] != "no_reply":
                break
    except Exception as exc:
        print("PARK_FAIL " + json.dumps({"stage": "park_cmd", "err": str(exc)}),
              flush=True)
        return 3
    info = {"cap_bitmap": None if cap is None else f"0x{cap:08X}",
            "park_tries": len(seqs)}
    if park["answer"] == "urc":
        return _firmware_park(link, park, info)
    if has_cmd:
        # 2026-10-24 review: a register SLEEP write cannot hold on this image
        # (its scan never fails out), so none is made: PARK_FAIL, and the
        # caller runs again (leg_post.sh: 3 attempts, 60 s apart).
        what = ("did not answer it" if park["answer"] == "no_reply"
                else "refused it")
        tries = "1 try" if len(seqs) == 1 else f"{len(seqs)} tries"
        info.update(path="firmware", persistent=False, holds=None,
                    park_cmd=park["answer"],
                    reason=(f"the L072 advertises the park command (VER cap bit 7) "
                            f"but {what} in {tries}; a register SLEEP write cannot "
                            f"hold on this image (its scan restarts after a final "
                            f"RX_SCAN_FAILED and re-arms RX every dwell), so none "
                            f"was made"))
        if park["detail"]:
            info["park_cmd_err"] = park["detail"]
        print("PARK_FAIL " + json.dumps(info), flush=True)
        return 4
    return _register_park(link, park, info)


if __name__ == "__main__":
    raise SystemExit(main())
