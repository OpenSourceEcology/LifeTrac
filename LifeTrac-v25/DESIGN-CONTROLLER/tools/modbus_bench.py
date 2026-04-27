"""Bench-test the Opta Modbus-RTU slave from a PC or Portenta X8.

Use this with a USB-RS485 adapter before connecting hydraulic outputs. The
script cycles the writable register map, keeps REG_WATCHDOG_CTR alive, and can
optionally prove the Opta watchdog drops to safe state when writes stop.
"""

from __future__ import annotations

import argparse
import time

try:
    from pymodbus.client import ModbusSerialClient
except ImportError as exc:  # pragma: no cover - friendly hardware-bench error
    raise SystemExit("Install pymodbus first: python -m pip install pymodbus") from exc


SLAVE_ID = 0x01
REG_VALVE_COILS = 0x0000
REG_FLOW_SP_1 = 0x0001
REG_FLOW_SP_2 = 0x0002
REG_AUX_OUTPUTS = 0x0003
REG_WATCHDOG_CTR = 0x0004
REG_CMD_SOURCE = 0x0005
REG_ARM_ESTOP = 0x0006
REG_SAFETY_STATE = 0x0100
REG_DIGITAL_INPUTS = 0x0101
REG_BATTERY_MV = 0x0102


def write_register(client: ModbusSerialClient, address: int, value: int) -> None:
    result = client.write_register(address, value & 0xFFFF, slave=SLAVE_ID)
    if result.isError():
        raise RuntimeError(f"write 0x{address:04X} failed: {result}")


def read_inputs(client: ModbusSerialClient, count: int = 12) -> list[int]:
    result = client.read_input_registers(REG_SAFETY_STATE, count=count, slave=SLAVE_ID)
    if result.isError():
        raise RuntimeError(f"input read failed: {result}")
    return list(result.registers)


def keepalive(client: ModbusSerialClient, ticks: int, period_s: float) -> int:
    for _ in range(ticks):
        keepalive.counter = (keepalive.counter + 1) & 0xFFFF
        write_register(client, REG_WATCHDOG_CTR, keepalive.counter)
        time.sleep(period_s)
    return keepalive.counter


keepalive.counter = 0


def run_bench(args: argparse.Namespace) -> None:
    client = ModbusSerialClient(
        port=args.port,
        baudrate=args.baud,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=args.timeout,
    )
    if not client.connect():
        raise SystemExit(f"could not open {args.port}")
    try:
        print("zeroing outputs")
        write_register(client, REG_VALVE_COILS, 0)
        write_register(client, REG_FLOW_SP_1, 0)
        write_register(client, REG_FLOW_SP_2, 0)
        write_register(client, REG_AUX_OUTPUTS, 0)
        write_register(client, REG_ARM_ESTOP, 0)

        print("starting watchdog keepalive")
        keepalive(client, 5, 0.05)

        print("cycling valve bitfield with keepalive")
        for bit in range(8):
            write_register(client, REG_VALVE_COILS, 1 << bit)
            write_register(client, REG_CMD_SOURCE, 0x02)  # base
            keepalive(client, args.hold_ticks, 0.05)
        write_register(client, REG_VALVE_COILS, 0)

        print("sweeping analog outputs")
        for mv in (0, 2500, 5000, 7500, 10000, 0):
            write_register(client, REG_FLOW_SP_1, mv)
            write_register(client, REG_FLOW_SP_2, 10000 - mv if mv else 0)
            keepalive(client, args.hold_ticks, 0.05)

        inputs = read_inputs(client)
        print("inputs:", " ".join(f"0x{value:04X}" for value in inputs))
        print("safety_state=", inputs[REG_SAFETY_STATE - REG_SAFETY_STATE])
        print("digital_inputs=", f"0x{inputs[REG_DIGITAL_INPUTS - REG_SAFETY_STATE]:04X}")
        print("battery_mv=", inputs[REG_BATTERY_MV - REG_SAFETY_STATE])

        if args.watchdog_trip:
            print("proving watchdog trip: stopping writes for 350 ms")
            time.sleep(0.35)
            print("post-timeout inputs:", read_inputs(client)[:3])
    finally:
        try:
            write_register(client, REG_VALVE_COILS, 0)
            write_register(client, REG_FLOW_SP_1, 0)
            write_register(client, REG_FLOW_SP_2, 0)
        finally:
            client.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="serial device, e.g. COM7 or /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--timeout", type=float, default=0.5)
    parser.add_argument("--hold-ticks", type=int, default=3, help="50 ms watchdog writes per output step")
    parser.add_argument("--watchdog-trip", action="store_true", help="intentionally stop writes and read safe state")
    run_bench(parser.parse_args())


if __name__ == "__main__":
    main()