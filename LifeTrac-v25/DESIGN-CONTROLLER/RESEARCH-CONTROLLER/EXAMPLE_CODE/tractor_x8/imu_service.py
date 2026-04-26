#!/usr/bin/env python3
"""
LifeTrac v25 — tractor X8 IMU service (draft, study-only)
=========================================================

Runs as a systemd service (or Docker container) on the *tractor* Portenta X8's
Linux side. Reads a Bosch BNO086 9-DoF IMU connected over Qwiic / Stemma QT
through an Adafruit MCP2221A USB→I²C bridge (Adafruit 4471, see
[`HARDWARE_BOM.md`](../../../HARDWARE_BOM.md) Tier 1), then forwards 5 Hz
quaternion + accel samples to the M7 firmware over the X8↔M7 UART so the M7
can re-emit them as `topic_id=0x07` on the LoRa link.

Why this lives on the X8 Linux side and not on the M7:
  * **No new driver work.** `hid-mcp2221` has been mainline since Linux 5.10,
    so a stock Yocto X8 image enumerates the bridge as `/dev/i2c-N` with no
    install. Adafruit's `adafruit-blinka` + `adafruit-circuitpython-bno08x`
    libraries open it like any other I²C bus.
  * **Frees the M7.** The M7 is already running the 50 Hz Modbus master + LoRa
    arbitration loop (see [`tractor_h7/tractor_m7.ino`](../tractor_h7/tractor_m7.ino)).
    IMU readings are not sub-millisecond critical (tip-over warning, heading
    hold, vibration logging), so handing them to Linux is free latency we
    can spend.
  * **Hot-pluggable.** USB-Qwiic means the IMU can be unplugged for bench
    work without re-flashing the M7.

Architecture:

    [BNO086 Qwiic]──Stemma QT──[Adafruit 4471 MCP2221A]──USB-A──[Portenta X8 Linux]
                                                                       │
                                                                       │  /dev/ttymxc0  (UART to H747 M7)
                                                                       │  KISS-framed `IMUSample` packets at 5 Hz
                                                                       ▼
                                                            [tractor_m7.ino]
                                                                       │
                                                                       ▼
                                                          LoRa topic_id=0x07
                                                          (see lora_bridge.py
                                                          TOPIC_BY_ID)

To switch to native I²C (Phase 2 / shipping product) wire BNO086 SDA/SCL/3V3/GND
directly to the Max Carrier I²C breakout and move the read loop into
`tractor_m7.ino`. Drops the MCP2221A + USB cable (~$9) but moves the work to
the M7. Tradeoff is documented in
[`HARDWARE_BOM.md` § Notes on substitutions — IMU path](../../../HARDWARE_BOM.md#notes-on-substitutions).

Status: **draft for review**. Not run, not flashed.
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

# ---------------------------------------------------------------------------
# Tell Adafruit Blinka to use the MCP2221A (4471) as the I²C provider, *not*
# the X8's built-in I²C bus. Setting this env var before importing `board`
# is the canonical pattern documented by Adafruit.
#   https://learn.adafruit.com/circuitpython-libraries-on-any-computer-with-mcp2221
# ---------------------------------------------------------------------------
os.environ.setdefault("BLINKA_MCP2221", "1")

# Imports below are deferred so `--help` works on a host without the libs
# installed. They are all available from `pip install -r requirements.txt`.
def _import_hardware():
    import board                             # type: ignore  # noqa: F401
    import busio                             # type: ignore
    import adafruit_bno08x                   # type: ignore
    from adafruit_bno08x.i2c import BNO08X_I2C  # type: ignore
    return board, busio, adafruit_bno08x, BNO08X_I2C


# ---------------------------------------------------------------------------
# Wire format from X8 → M7
# ---------------------------------------------------------------------------
# We send a KISS-framed binary packet that the M7 unpacks and stuffs straight
# into the `payload` field of a TelemetryFrame with `topic_id = 0x07`. Format:
#
#     uint8_t   version       (= 0x01)
#     uint8_t   sample_flags  (bit 0: quat_valid, bit 1: accel_valid, bit 2: cal_ok)
#     int16_t   qw, qx, qy, qz   (Q14 fixed-point, range [-2.0, +2.0))
#     int16_t   ax, ay, az       (mg, signed; ±32 g range)
#     uint16_t  yaw_deg10        (heading × 10, 0..3599)  — convenience for UI
#
# Total: 1 + 1 + 8 + 6 + 2 = 18 bytes; fits the 51-byte SF7/BW500 telemetry budget
# from LORA_PROTOCOL.md § Frame format with room for the 2-byte CRC and topic
# header. Layout matches what the bridge will publish under
# `lifetrac/v25/telemetry/imu` (see TOPIC_BY_ID 0x07 in
# [`base_station/lora_bridge.py`](../base_station/lora_bridge.py)).
IMU_PACKET_VERSION = 0x01
IMU_PACKET_FMT = "<BB hhhh hhh H"   # 1+1+8+6+2 = 18 bytes
assert struct.calcsize(IMU_PACKET_FMT) == 18

KISS_FEND = 0xC0
KISS_FESC = 0xDB
KISS_TFEND = 0xDC
KISS_TFESC = 0xDD


def kiss_encode(data: bytes) -> bytes:
    """Standard KISS (RFC 1055-style) framing — same as `lp_kiss_encode` in
    [`lora_proto/lora_proto.c`](../lora_proto/lora_proto.c). One framer in the
    codebase, used here, on the air, *and* on the X8↔H747 UART."""
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


@dataclass
class IMUSample:
    qw: float; qx: float; qy: float; qz: float
    ax_mg: int; ay_mg: int; az_mg: int
    yaw_deg10: int          # 0..3599
    quat_valid: bool
    accel_valid: bool
    cal_ok: bool

    def pack(self) -> bytes:
        flags = (
            (0x01 if self.quat_valid else 0)
            | (0x02 if self.accel_valid else 0)
            | (0x04 if self.cal_ok else 0)
        )
        # Quaternion → Q14 fixed-point (1.0 -> 16384). Clamp to int16 range.
        def q14(v: float) -> int:
            return max(-32768, min(32767, int(round(v * 16384.0))))
        return struct.pack(
            IMU_PACKET_FMT,
            IMU_PACKET_VERSION,
            flags,
            q14(self.qw), q14(self.qx), q14(self.qy), q14(self.qz),
            int(self.ax_mg), int(self.ay_mg), int(self.az_mg),
            int(self.yaw_deg10) % 3600,
        )


# ---------------------------------------------------------------------------
# Sensor read loop
# ---------------------------------------------------------------------------
def open_imu():
    """Open the BNO086 over the MCP2221A bridge. Returns a configured sensor
    object with the rotation-vector and linear-accel reports enabled."""
    board, busio, adafruit_bno08x, BNO08X_I2C = _import_hardware()
    # On the MCP2221A, board.I2C() returns a busio.I2C wrapping `/dev/i2c-N`.
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400_000)
    # SparkFun DEV-22857 ships at 0x4B (ADR pulled high). The Adafruit
    # BNO08x dev board defaults to 0x4A. We try 0x4B first, fall back to 0x4A.
    last_err: Optional[Exception] = None
    for addr in (0x4B, 0x4A):
        try:
            sensor = BNO08X_I2C(i2c, address=addr)
            sensor.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
            sensor.enable_feature(adafruit_bno08x.BNO_REPORT_LINEAR_ACCELERATION)
            logging.info("BNO086 opened at I²C 0x%02X", addr)
            return sensor
        except Exception as exc:                                  # noqa: BLE001
            last_err = exc
            logging.debug("BNO086 not at 0x%02X: %s", addr, exc)
    raise RuntimeError(f"BNO086 not found on Qwiic bus: {last_err}")


def read_sample(sensor) -> IMUSample:
    """Pull the latest fused quaternion + linear acceleration from the sensor.

    BNO086 returns the rotation vector as (i, j, k, real). We map it to the
    aerospace convention (w, x, y, z). Linear acceleration is gravity-removed
    in m/s², which we convert to milligees for compact serialisation."""
    qx, qy, qz, qw = sensor.quaternion
    ax, ay, az = sensor.linear_acceleration
    # Heading from the quaternion's z-axis (yaw). atan2 in deg×10:
    import math
    siny_cosp = 2 * (qw * qz + qx * qy)
    cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
    yaw_deg = math.degrees(math.atan2(siny_cosp, cosy_cosp)) % 360.0
    # The BNO086 has a self-reported calibration status (0..3 for each subsystem);
    # we treat ≥2 across the board as "OK".
    cal_status = getattr(sensor, "calibration_status", 3)  # 3 = fully calibrated
    return IMUSample(
        qw=qw, qx=qx, qy=qy, qz=qz,
        ax_mg=int(ax * 101.971621),  # 1 m/s² = 101.97 mg
        ay_mg=int(ay * 101.971621),
        az_mg=int(az * 101.971621),
        yaw_deg10=int(yaw_deg * 10) % 3600,
        quat_valid=True,
        accel_valid=True,
        cal_ok=cal_status >= 2,
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
    p.add_argument("--rate-hz", type=float, default=5.0,
                   help="IMU emit rate in Hz (default 5; LoRa budget allows ≤10)")
    p.add_argument("--dry-run", action="store_true",
                   help="Read the IMU and log to stdout; do not open the UART")
    p.add_argument("-v", "--verbose", action="store_true")
    args = p.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)-7s %(message)s",
    )

    sensor = open_imu()
    uart = None if args.dry_run else open_uart(args.uart, args.baud)

    period = 1.0 / max(args.rate_hz, 0.1)
    next_tick = time.monotonic()
    n_sent = 0
    n_errs = 0
    while True:
        next_tick += period
        try:
            sample = read_sample(sensor)
        except Exception as exc:                                  # noqa: BLE001
            n_errs += 1
            logging.warning("IMU read failed (#%d): %s", n_errs, exc)
            # If we lose the sensor we still want to emit a fault so the M7
            # can publish on topic 0x08 (sensor_faults). Cheap one-byte path:
            # send a zero-payload v1 packet with all flag bits cleared.
            sample = IMUSample(0, 0, 0, 0, 0, 0, 0, 0, False, False, False)

        frame = kiss_encode(sample.pack())
        if uart is not None:
            try:
                uart.write(frame)
            except Exception as exc:                              # noqa: BLE001
                logging.error("UART write failed: %s", exc)
                # Don't die — the M7 has its own watchdog; just back off briefly.
                time.sleep(0.1)
                continue
        else:
            logging.info("imu sample qw=%.3f qx=%.3f qy=%.3f qz=%.3f "
                         "a=(%d,%d,%d)mg yaw=%.1f° cal_ok=%s",
                         sample.qw, sample.qx, sample.qy, sample.qz,
                         sample.ax_mg, sample.ay_mg, sample.az_mg,
                         sample.yaw_deg10 / 10.0, sample.cal_ok)

        n_sent += 1
        if n_sent % 100 == 0:
            logging.info("imu_service: %d samples, %d errors", n_sent, n_errs)

        # Steady-rate scheduler — never busy-wait, and re-sync if we fall behind.
        now = time.monotonic()
        sleep_for = next_tick - now
        if sleep_for > 0:
            time.sleep(sleep_for)
        else:
            next_tick = now  # we're behind — reset the schedule


if __name__ == "__main__":
    try:
        sys.exit(main() or 0)
    except KeyboardInterrupt:
        sys.exit(0)
