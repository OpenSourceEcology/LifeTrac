#!/usr/bin/env python3
"""
LifeTrac v25 — tractor X8 GPS service (draft, study-only)
=========================================================

Runs as a systemd service (or Docker container) on the *tractor* Portenta X8's
Linux side. Reads a u-blox **NEO-M9N** GPS module (SparkFun GPS-15733, SMA
Qwiic variant) at I²C address 0x42, daisy-chained on the **same Qwiic bus as
the BNO086 IMU** through a single Adafruit MCP2221A USB→I²C bridge
(Adafruit 4471, see [`HARDWARE_BOM.md`](../../../HARDWARE_BOM.md) Tier 1).

Why a periodic 1 Hz message is enough:
  * **Tractor top speed ~5 mph (8 km/h ≈ 2.2 m/s).** At 1 Hz we miss at most
    ~2.2 m of travel between fixes — well inside the NEO-M9N's ~1.5 m typical
    standalone accuracy. Bumping to 5 Hz costs LoRa airtime and doesn't make
    a tracked tractor visibly smoother on the base-station map.
  * **Hydraulic loop is unrelated.** Tip-over warning + heading hold come from
    the IMU at 5 Hz (see [`imu_service.py`](imu_service.py)) — the GPS only
    feeds the operator-console map and the autonomy waypoint follower.
  * **Battery / cellular budget.** A 1 Hz NMEA update at SF7/BW500 is ~30 ms
    of airtime — negligible. We can also drop to 0.2 Hz when the tractor is
    parked (speed < 0.1 m/s for 10 s) to free the channel for telemetry.

The service publishes 20-byte fixed-point GPS samples to the M7 over the
X8↔H747 UART (`/dev/ttymxc0`); the M7 wraps each one in a `TelemetryFrame`
with `topic_id = 0x01` (already mapped to `lifetrac/v25/telemetry/gps` in
[`base_station/lora_bridge.py`](../base_station/lora_bridge.py) `TOPIC_BY_ID`).

Architecture (same Qwiic bus as IMU — chained, not parallel):

    [BNO086 Qwiic]──Stemma QT──[NEO-M9N SMA Qwiic]──Stemma QT──[Adafruit 4471 MCP2221A]──USB-A──[Portenta X8 Linux]
        I²C 0x4A/0x4B                    I²C 0x42                                                      │
                                                                                                       │  /dev/ttymxc0
                                                                                                       │  KISS-framed `GPSSample` @ 1 Hz
                                                                                                       │  KISS-framed `IMUSample`  @ 5 Hz
                                                                                                       ▼
                                                                                              [tractor_m7.ino]
                                                                                                       │
                                                                                                       ▼
                                                                                          LoRa topic_id=0x01 (gps)
                                                                                          LoRa topic_id=0x07 (imu)

Status: **draft for review**. Not run, not flashed. Companion to
[`imu_service.py`](imu_service.py); the two services share the wire format
contract with the M7 (KISS framing + per-topic payload schema).
"""

from __future__ import annotations

import argparse
import logging
import os
import struct
import sys
import time
from dataclasses import dataclass
from typing import Optional

# Reuse the same MCP2221A discipline as imu_service.py — set BEFORE importing
# `board` so Blinka picks the USB bridge over the X8's onboard I²C.
os.environ.setdefault("BLINKA_MCP2221", "1")


def _import_hardware():
    import board                                # type: ignore  # noqa: F401
    import busio                                # type: ignore
    from adafruit_gps import GPS_GtopI2C        # type: ignore
    return board, busio, GPS_GtopI2C


# ---------------------------------------------------------------------------
# Wire format from X8 → M7 for `topic_id = 0x01` (GPS).
# ---------------------------------------------------------------------------
# Designed to (a) fit comfortably inside the 51-byte SF7/BW500 telemetry
# budget from LORA_PROTOCOL.md § Frame format, (b) keep a fixed schema so
# the bridge can deserialise without per-version code paths, (c) cover the
# common UI needs (map marker, heading rose, fix quality bar).
#
#     uint8_t   version       (= 0x01)
#     uint8_t   flags         (bit 0: pos_valid, bit 1: vel_valid,
#                              bit 2: heading_valid, bit 3: time_valid)
#     int32_t   lat_e7        (degrees × 1e7, signed)
#     int32_t   lon_e7        (degrees × 1e7, signed)
#     int32_t   alt_cm        (centimetres above MSL, signed)
#     int16_t   speed_cmps    (centimetres/second, ground speed)
#     uint16_t  heading_deg10 (course over ground × 10, 0..3599)
#     uint8_t   fix_type      (0=no fix, 2=2D, 3=3D, 4=DGPS, 5=RTK fix, 6=RTK float)
#     uint8_t   num_sats      (count of satellites used)
#     uint8_t   hdop_x10      (horizontal DoP × 10; 255 = unknown)
#
# Total: 1 + 1 + 4 + 4 + 4 + 2 + 2 + 1 + 1 + 1 = 21 bytes payload, plus the
# 2-byte CRC the M7 appends → 23 bytes. Comfortably inside budget.
GPS_PACKET_VERSION = 0x01
GPS_PACKET_FMT = "<BB iii hH BBB"
assert struct.calcsize(GPS_PACKET_FMT) == 21


KISS_FEND = 0xC0
KISS_FESC = 0xDB
KISS_TFEND = 0xDC
KISS_TFESC = 0xDD


def kiss_encode(data: bytes) -> bytes:
    """Standard KISS framing — same as `lp_kiss_encode` in
    [`lora_proto/lora_proto.c`](../lora_proto/lora_proto.c) and the IMU
    service. One framer in the codebase, used here, on the air, *and* on
    the X8↔H747 UART."""
    out = bytearray([KISS_FEND])
    for b in data:
        if b == KISS_FEND:
            out.extend([KISS_FESC, KISS_TFEND])
        elif b == KISS_FESC:
            out.extend([KISS_FESC, KISS_TFESC])
        else:
            out.append(b)
    out.append(KISS_FEND)
    return bytes(out)


# ---------------------------------------------------------------------------
# We tag GPS frames with a topic byte the M7 expects, so a single UART
# receiver can demux gps/imu/etc. (Both services prefix their KISS payload
# with this byte. The M7 strips it before stuffing the rest into the
# TelemetryFrame.payload.) Matches TOPIC_BY_ID in base_station/lora_bridge.py.
TOPIC_GPS = 0x01
TOPIC_IMU = 0x07


@dataclass
class GPSSample:
    lat_deg: float
    lon_deg: float
    alt_m: float
    speed_mps: float
    heading_deg: float          # course over ground, 0..360
    fix_type: int               # 0..6 (see fmt comment)
    num_sats: int
    hdop: float                 # 0..99.9 typical
    pos_valid: bool
    vel_valid: bool
    heading_valid: bool
    time_valid: bool

    def pack(self) -> bytes:
        flags = (
            (0x01 if self.pos_valid else 0)
            | (0x02 if self.vel_valid else 0)
            | (0x04 if self.heading_valid else 0)
            | (0x08 if self.time_valid else 0)
        )
        lat_e7 = max(-2_147_483_648, min(2_147_483_647, int(round(self.lat_deg * 1e7))))
        lon_e7 = max(-2_147_483_648, min(2_147_483_647, int(round(self.lon_deg * 1e7))))
        alt_cm = max(-2_147_483_648, min(2_147_483_647, int(round(self.alt_m * 100))))
        spd_cmps = max(-32768, min(32767, int(round(self.speed_mps * 100))))
        head_deg10 = int(round(self.heading_deg * 10)) % 3600
        hdop_x10 = max(0, min(255, int(round(self.hdop * 10)))) if self.hdop is not None else 255
        return struct.pack(
            GPS_PACKET_FMT,
            GPS_PACKET_VERSION, flags,
            lat_e7, lon_e7, alt_cm,
            spd_cmps, head_deg10,
            min(self.fix_type, 6),
            min(self.num_sats, 255),
            hdop_x10,
        )


# ---------------------------------------------------------------------------
# Sensor read loop
# ---------------------------------------------------------------------------
def open_gps():
    """Open the NEO-M9N over the MCP2221A bridge. Returns a configured
    `adafruit_gps.GPS_GtopI2C` instance."""
    board, busio, GPS_GtopI2C = _import_hardware()
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
    # u-blox NEO-M9N defaults to 7-bit address 0x42 over I²C (DDC).
    gps = GPS_GtopI2C(i2c, address=0x42, debug=False)
    # Ask the receiver for the bare minimum we use:
    #   GGA — fix, sats, alt, HDOP
    #   RMC — lat, lon, ground speed, course, UTC time, valid flag
    # Output rate 1 Hz (1000 ms). The NEO-M9N supports up to 25 Hz, but per
    # the module docstring 1 Hz is plenty for a slow-moving tractor.
    gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")  # GLL/RMC/VTG/GGA/GSA/GSV
    gps.send_command(b"PMTK220,1000")
    return gps


def read_sample(gps) -> GPSSample:
    """Pull the latest fix from the parser. `adafruit_gps` accumulates a
    single most-recent fix internally; we just have to pump `update()` until
    it returns False (no more bytes pending)."""
    # Drain everything in the I²C buffer; `update()` returns True per parsed
    # sentence. We bound the loop so a runaway sensor can't starve the rest
    # of the service.
    for _ in range(32):
        if not gps.update():
            break

    if gps.has_fix:
        return GPSSample(
            lat_deg=float(gps.latitude or 0.0),
            lon_deg=float(gps.longitude or 0.0),
            alt_m=float(gps.altitude_m or 0.0),
            # adafruit_gps reports speed in knots; 1 knot = 0.514444 m/s.
            speed_mps=float((gps.speed_knots or 0.0) * 0.514444),
            heading_deg=float(gps.track_angle_deg or 0.0),
            fix_type=int(gps.fix_quality or 0),
            num_sats=int(gps.satellites or 0),
            hdop=float(gps.horizontal_dilution or 99.9),
            pos_valid=True,
            vel_valid=gps.speed_knots is not None,
            heading_valid=gps.track_angle_deg is not None,
            time_valid=gps.timestamp_utc is not None,
        )
    # No fix — emit an explicit "invalid" sample so the bridge can still
    # publish a heartbeat-style "no fix yet" message and the UI can show
    # "acquiring satellites".
    return GPSSample(
        lat_deg=0.0, lon_deg=0.0, alt_m=0.0,
        speed_mps=0.0, heading_deg=0.0,
        fix_type=0,
        num_sats=int(gps.satellites or 0),
        hdop=99.9,
        pos_valid=False, vel_valid=False, heading_valid=False, time_valid=False,
    )


# ---------------------------------------------------------------------------
# UART link to the M7 (/dev/ttymxc0 on the Portenta X8 by default)
# ---------------------------------------------------------------------------
def open_uart(port: str, baud: int):
    import serial  # type: ignore
    return serial.Serial(port, baud, timeout=0.1, write_timeout=0.1)


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def main(argv: Optional[list[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--uart", default="/dev/ttymxc0",
                   help="X8↔H747 UART device (default: /dev/ttymxc0)")
    p.add_argument("--baud", type=int, default=921_600,
                   help="UART baud (must match tractor_m7.ino, default 921600)")
    p.add_argument("--rate-hz", type=float, default=1.0,
                   help="GPS emit rate in Hz (default 1; module supports up to 25)")
    p.add_argument("--idle-rate-hz", type=float, default=0.2,
                   help="Reduced rate when the tractor has been parked "
                        "(speed < 0.1 m/s for 10 s). 0 disables idle backoff.")
    p.add_argument("--dry-run", action="store_true",
                   help="Read the GPS and log to stdout; do not open the UART")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
    )

    gps = open_gps()
    uart = None if args.dry_run else open_uart(args.uart, args.baud)

    active_period = 1.0 / max(args.rate_hz, 0.05)
    idle_period = 1.0 / args.idle_rate_hz if args.idle_rate_hz > 0 else active_period
    next_tick = time.monotonic()
    parked_since: Optional[float] = None
    n_sent = 0
    n_no_fix = 0
    while True:
        try:
            sample = read_sample(gps)
        except Exception as exc:                                  # noqa: BLE001
            logging.warning("GPS read failed: %s", exc)
            sample = GPSSample(
                0.0, 0.0, 0.0, 0.0, 0.0, 0, 0, 99.9,
                False, False, False, False,
            )

        # Idle-rate backoff: if we've been parked for >10 s, drop to idle rate.
        now = time.monotonic()
        if sample.vel_valid and sample.speed_mps < 0.1:
            if parked_since is None:
                parked_since = now
        else:
            parked_since = None
        is_parked = (
            args.idle_rate_hz > 0
            and parked_since is not None
            and (now - parked_since) > 10.0
        )
        period = idle_period if is_parked else active_period

        # Topic-tagged payload: the M7 reads <topic><payload> off the UART
        # and stuffs the payload into TelemetryFrame.payload after setting
        # tf.topic_id from the leading byte.
        framed = kiss_encode(bytes([TOPIC_GPS]) + sample.pack())
        if uart is not None:
            try:
                uart.write(framed)
            except Exception as exc:                              # noqa: BLE001
                logging.error("UART write failed: %s", exc)
                time.sleep(0.1)
                continue
        else:
            logging.info("gps lat=%.6f lon=%.6f alt=%.1fm spd=%.2fm/s "
                         "hdg=%.1f° fix=%d sats=%d hdop=%.1f valid=%s parked=%s",
                         sample.lat_deg, sample.lon_deg, sample.alt_m,
                         sample.speed_mps, sample.heading_deg,
                         sample.fix_type, sample.num_sats, sample.hdop,
                         sample.pos_valid, is_parked)

        n_sent += 1
        if not sample.pos_valid:
            n_no_fix += 1
        if n_sent % 60 == 0:
            logging.info("gps_service: %d samples, %d without fix", n_sent, n_no_fix)

        # Steady-rate scheduler — same pattern as imu_service.py.
        next_tick += period
        sleep_for = next_tick - time.monotonic()
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_tick = time.monotonic()


if __name__ == "__main__":
    try:
        sys.exit(main() or 0)
    except KeyboardInterrupt:
        sys.exit(0)
