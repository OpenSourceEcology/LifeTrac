#!/usr/bin/env python3
"""Ensure tractor_h7/murata_host/mh_wire.h mirrors key host_types constants."""

from __future__ import annotations

import pathlib
import re
import sys
from typing import Dict, Iterable, List

ROOT = pathlib.Path(__file__).resolve().parents[1]
HOST_TYPES = ROOT / "firmware" / "murata_l072" / "include" / "host_types.h"
MH_WIRE = ROOT / "firmware" / "tractor_h7" / "murata_host" / "mh_wire.h"

DEFINE_RE = re.compile(r"^\s*#define\s+([A-Za-z0-9_]+)\s+(.+?)\s*(?:/\*.*\*/)?$")

SYNC_KEYS: List[str] = [
    "HOST_PROTOCOL_VER",
    "HOST_WIRE_SCHEMA_VER",
    "HOST_TYPE_PING_REQ",
    "HOST_TYPE_VER_REQ",
    "HOST_TYPE_UID_REQ",
    "HOST_TYPE_RESET_REQ",
    "HOST_TYPE_TX_FRAME_REQ",
    "HOST_TYPE_CFG_SET_REQ",
    "HOST_TYPE_CFG_GET_REQ",
    "HOST_TYPE_REG_READ_REQ",
    "HOST_TYPE_REG_WRITE_REQ",
    "HOST_TYPE_STATS_RESET_REQ",
    "HOST_TYPE_STATS_DUMP_REQ",
    "HOST_TYPE_VER_URC",
    "HOST_TYPE_UID_URC",
    "HOST_TYPE_TX_DONE_URC",
    "HOST_TYPE_RX_FRAME_URC",
    "HOST_TYPE_CFG_OK_URC",
    "HOST_TYPE_CFG_DATA_URC",
    "HOST_TYPE_REG_DATA_URC",
    "HOST_TYPE_REG_WRITE_ACK_URC",
    "HOST_TYPE_RADIO_IRQ_URC",
    "HOST_TYPE_STATS_URC",
    "HOST_TYPE_BOOT_URC",
    "HOST_TYPE_FAULT_URC",
    "HOST_TYPE_ERR_PROTO_URC",
    "HOST_FAULT_CODE_CLOCK_HSE_FAILED",
    "HOST_STATS_OFFSET_HOST_DROPPED",
    "HOST_STATS_OFFSET_HOST_ERRORS",
    "HOST_STATS_OFFSET_HOST_QUEUE_FULL",
    "HOST_STATS_OFFSET_HOST_IRQ_IDLE",
    "HOST_STATS_OFFSET_HOST_IRQ_HT",
    "HOST_STATS_OFFSET_HOST_IRQ_TC",
    "HOST_STATS_OFFSET_HOST_IRQ_TE",
    "HOST_STATS_OFFSET_RADIO_DIO0",
    "HOST_STATS_OFFSET_RADIO_DIO1",
    "HOST_STATS_OFFSET_RADIO_DIO2",
    "HOST_STATS_OFFSET_RADIO_DIO3",
    "HOST_STATS_OFFSET_RADIO_CRC_ERR",
    "HOST_STATS_OFFSET_RADIO_RX_OK",
    "HOST_STATS_OFFSET_RADIO_TX_OK",
    "HOST_STATS_OFFSET_RADIO_TX_ABORT_LBT",
    "HOST_STATS_OFFSET_RADIO_TX_ABORT_AIRTIME",
    "HOST_STATS_OFFSET_RADIO_STATE",
    "HOST_STATS_PAYLOAD_LEN",
]


def _load_defines(path: pathlib.Path) -> Dict[str, str]:
    defines: Dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        match = DEFINE_RE.match(raw_line)
        if not match:
            continue
        key = match.group(1)
        value = match.group(2).strip()
        defines[key] = value
    return defines


def _missing(keys: Iterable[str], table: Dict[str, str]) -> List[str]:
    return [key for key in keys if key not in table]


def main() -> int:
    if not HOST_TYPES.exists() or not MH_WIRE.exists():
        print("[FAIL] required files not found")
        print(f"  host_types: {HOST_TYPES}")
        print(f"  mh_wire:    {MH_WIRE}")
        return 1

    host_defines = _load_defines(HOST_TYPES)
    mh_defines = _load_defines(MH_WIRE)

    missing_host = _missing(SYNC_KEYS, host_defines)
    missing_mh = _missing(SYNC_KEYS, mh_defines)

    if missing_host:
        print("[FAIL] host_types missing expected keys:")
        for key in missing_host:
            print(f"  - {key}")
        return 1

    if missing_mh:
        print("[FAIL] mh_wire missing expected keys:")
        for key in missing_mh:
            print(f"  - {key}")
        return 1

    mismatches = []
    for key in SYNC_KEYS:
        if host_defines[key] != mh_defines[key]:
            mismatches.append((key, host_defines[key], mh_defines[key]))

    if mismatches:
        print("[FAIL] mh_wire constant drift detected:")
        for key, host_value, mh_value in mismatches:
            print(f"  - {key}: host_types={host_value} mh_wire={mh_value}")
        return 1

    print(f"[PASS] mh_wire sync: {len(SYNC_KEYS)} constants match")
    return 0


if __name__ == "__main__":
    sys.exit(main())
