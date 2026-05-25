#!/usr/bin/env python3
"""radio_state_dump.py

Dump SX1276 modem/RF registers from the L072 co-processor after running
the same configure_regulatory_profile_if_needed + role-specific tail as
image_{rx,tx}_daemon._open_link(). Output is a single JSON line prefixed
with `RADIO_DUMP ` so the host-side orchestrator can grep + parse it.

Falsification harness for the 2026-05-25 "TX 4/4 ok but rx_frames=0"
breakthrough — checks whether TX and RX peers are actually programmed
with matching RF parameters (FRF, SF, BW, CR, SyncWord, HopPeriod).

Assumes the caller already pulsed gpio163 NRST externally; mirrors the
LIFETRAC_SKIP_RESET_REQ=1 behaviour added to the daemons (no extra UART
RESET_REQ, just drain boot chatter briefly).

Usage in container:
    python3 -u /work/radio_state_dump.py --role rx
    python3 -u /work/radio_state_dump.py --role tx
    python3 -u /work/radio_state_dump.py --role raw   # just boot, no CFG
"""
from __future__ import annotations

import argparse
import json
import sys
import time

sys.path.insert(0, "/work")

from method_h_stage2_tx_probe_v2 import (  # type: ignore
    HostLink,
    HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC,
    HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
    CFG_KEY_LBT_ENABLE,
    SX1276_REG_OP_MODE,
    SX1276_OPMODE_LORA_RXCONT,
    read_reg, write_reg,
    drain_boot, drain_pending,
    configure_regulatory_profile_if_needed,
)

# SX1276 LoRa register map (subset relevant to "is air config sane?").
REGS = [
    (0x01, "RegOpMode"),
    (0x06, "RegFrfMsb"),
    (0x07, "RegFrfMid"),
    (0x08, "RegFrfLsb"),
    (0x09, "RegPaConfig"),
    (0x0B, "RegOcp"),
    (0x0C, "RegLna"),
    (0x12, "RegIrqFlags"),
    (0x18, "RegModemStat"),
    (0x1B, "RegRssiValue"),
    (0x1D, "RegModemConfig1"),
    (0x1E, "RegModemConfig2"),
    (0x20, "RegPreambleMsb"),
    (0x21, "RegPreambleLsb"),
    (0x22, "RegPayloadLength"),
    (0x24, "RegHopPeriod"),
    (0x26, "RegModemConfig3"),
    (0x33, "RegInvertIQ"),
    (0x39, "RegSyncWord"),
    (0x40, "RegDioMapping1"),
    (0x41, "RegDioMapping2"),
]

BW_TABLE = {
    0: 7800, 1: 10400, 2: 15600, 3: 20800, 4: 31250,
    5: 41700, 6: 62500, 7: 125000, 8: 250000, 9: 500000,
}


def dump_regs(link: HostLink) -> dict:
    out = {}
    for addr, name in REGS:
        try:
            val, _raw = read_reg(link, addr, timeout=0.5)
            out[name] = {"addr": addr, "val": int(val), "hex": "0x%02X" % int(val)}
        except Exception as exc:
            out[name] = {"addr": addr, "error": str(exc)}
    return out


def _v(regs: dict, name: str):
    """Return regs[name]['val'] or None."""
    e = regs.get(name)
    if not e or "val" not in e:
        return None
    return e["val"]


def decode_modem(regs: dict) -> dict:
    d = {}
    msb = _v(regs, "RegFrfMsb")
    mid = _v(regs, "RegFrfMid")
    lsb = _v(regs, "RegFrfLsb")
    if None not in (msb, mid, lsb):
        frf = (msb << 16) | (mid << 8) | lsb
        # SX1276 datasheet: Fstep = Fxosc / 2^19; Fxosc = 32 MHz.
        freq_hz = int(round(frf * 32_000_000 / (1 << 19)))
        d["frf_raw"] = frf
        d["frf_hex"] = "0x%06X" % frf
        d["freq_hz"] = freq_hz
        d["freq_mhz"] = round(freq_hz / 1e6, 6)

    mc1 = _v(regs, "RegModemConfig1")
    if mc1 is not None:
        bw_code = (mc1 >> 4) & 0x0F
        cr_code = (mc1 >> 1) & 0x07
        ih = mc1 & 0x01
        d["bw_code"] = bw_code
        d["bw_hz"] = BW_TABLE.get(bw_code, "unknown(%d)" % bw_code)
        d["cr_code"] = cr_code
        d["cr"] = ("4/%d" % (4 + cr_code)) if 1 <= cr_code <= 4 else "unknown(%d)" % cr_code
        d["implicit_header"] = bool(ih)

    mc2 = _v(regs, "RegModemConfig2")
    if mc2 is not None:
        d["sf"] = (mc2 >> 4) & 0x0F
        d["tx_continuous"] = bool((mc2 >> 3) & 1)
        d["rx_payload_crc_on"] = bool((mc2 >> 2) & 1)
        d["symb_timeout_msb"] = mc2 & 0x03

    mc3 = _v(regs, "RegModemConfig3")
    if mc3 is not None:
        d["low_data_rate_optimize"] = bool((mc3 >> 3) & 1)
        d["agc_auto_on"] = bool((mc3 >> 2) & 1)

    pre_msb = _v(regs, "RegPreambleMsb")
    pre_lsb = _v(regs, "RegPreambleLsb")
    if None not in (pre_msb, pre_lsb):
        d["preamble_len"] = (pre_msb << 8) | pre_lsb

    sw = _v(regs, "RegSyncWord")
    if sw is not None:
        d["sync_word"] = "0x%02X" % sw

    hp = _v(regs, "RegHopPeriod")
    if hp is not None:
        d["hop_period_symbols"] = hp
        d["fhss_enabled"] = (hp != 0)

    inv = _v(regs, "RegInvertIQ")
    if inv is not None:
        d["invert_iq_rx"] = bool((inv >> 6) & 1)
        d["invert_iq_tx"] = bool((inv >> 0) & 1)

    opm = _v(regs, "RegOpMode")
    if opm is not None:
        d["opmode_hex"] = "0x%02X" % opm
        d["long_range_mode"] = bool((opm >> 7) & 1)
        d["low_freq_mode_on"] = bool((opm >> 3) & 1)
        d["mode_bits"] = opm & 0x07
        d["mode_name"] = {
            0: "SLEEP", 1: "STANDBY", 2: "FSTX", 3: "TX",
            4: "FSRX", 5: "RXCONTINUOUS", 6: "RXSINGLE", 7: "CAD",
        }.get(opm & 0x07, "?")

    return d


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--uart", default="/dev/ttymxc3")
    ap.add_argument("--baud", default="921600")
    ap.add_argument("--role", choices=["rx", "tx", "raw"], default="raw",
                    help="rx = post-CFG + RXCONT autowake (mirrors image_rx_daemon); "
                         "tx = post-CFG + LBT=0 (mirrors image_tx_daemon); "
                         "raw = boot only, no CFG_SET")
    ap.add_argument("--also-after-s", type=float, default=0.0,
                    help="if >0, dump again after this many seconds (catch FHSS drift / opmode change)")
    args = ap.parse_args()

    out: dict = {"role": args.role, "uart": args.uart, "baud": args.baud,
                 "ts_start": time.time()}

    link = HostLink(args.uart, args.baud)
    try:
        # External NRST already pulsed by caller — just drain boot chatter.
        drain_boot(link, settle_s=0.25)
        try:
            link.request(HOST_TYPE_VER_REQ, HOST_TYPE_VER_URC, timeout=1.0)
            out["ver_ok"] = True
        except Exception as exc:
            out["ver_ok"] = False
            out["ver_error"] = str(exc)
            print("RADIO_DUMP " + json.dumps(out, sort_keys=False))
            return 2
        drain_pending(link, quiet_s=0.25, max_s=1.0)

        out["regs_boot"] = dump_regs(link)
        out["decoded_boot"] = decode_modem(out["regs_boot"])

        if args.role in ("rx", "tx"):
            try:
                configure_regulatory_profile_if_needed(link)
                out["regprofile_ok"] = True
            except Exception as exc:
                out["regprofile_ok"] = False
                out["regprofile_error"] = str(exc)

            if args.role == "rx":
                try:
                    write_reg(link, SX1276_REG_OP_MODE,
                              SX1276_OPMODE_LORA_RXCONT, timeout=0.5)
                    out["rxcont_ok"] = True
                except Exception as exc:
                    out["rxcont_ok"] = False
                    out["rxcont_error"] = str(exc)
            elif args.role == "tx":
                try:
                    link.request(HOST_TYPE_CFG_SET_REQ, HOST_TYPE_CFG_OK_URC,
                                 bytes([CFG_KEY_LBT_ENABLE, 0x01, 0x00]),
                                 timeout=1.0)
                    out["lbt_off_ok"] = True
                except Exception as exc:
                    out["lbt_off_ok"] = False
                    out["lbt_off_error"] = str(exc)

            time.sleep(0.3)
            out["regs_post_config"] = dump_regs(link)
            out["decoded_post_config"] = decode_modem(out["regs_post_config"])

        if args.also_after_s > 0:
            time.sleep(args.also_after_s)
            out["regs_after_wait"] = dump_regs(link)
            out["decoded_after_wait"] = decode_modem(out["regs_after_wait"])
            out["wait_s"] = args.also_after_s

        out["ts_end"] = time.time()
        print("RADIO_DUMP " + json.dumps(out, sort_keys=False))
        return 0
    finally:
        try:
            link.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
