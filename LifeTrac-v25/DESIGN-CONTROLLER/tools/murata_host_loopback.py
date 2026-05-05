#!/usr/bin/env python3
"""PTY/TCP loopback harness for tractor_h7 host stream driver."""

from __future__ import annotations

import argparse
import os
import pathlib
import select
import socket
import subprocess
import sys
import time

try:
    import pty
except ImportError:  # pragma: no cover - platform-specific
    pty = None

ROOT = pathlib.Path(__file__).resolve().parents[1]
TOOLS_DIR = ROOT / "tools"
if str(TOOLS_DIR) not in sys.path:
    sys.path.insert(0, str(TOOLS_DIR))


def _run_generator() -> None:
    gen = TOOLS_DIR / "gen_mh_wire_py.py"
    subprocess.run([sys.executable, str(gen)], cwd=ROOT, check=True)


def _load_mock_helpers():
    from murata_host_l072_mock import MurataL072Mock, cobs_decode  # noqa: WPS433

    return MurataL072Mock, cobs_decode


def _default_driver_path() -> pathlib.Path:
    return ROOT / "build" / "loopback_driver"


def _drain_driver_output(proc: subprocess.Popen[str]) -> str:
    stdout, stderr = proc.communicate(timeout=2)
    text = (stdout or "") + (stderr or "")
    return text.strip()


def _run_frame_pump(
    proc: subprocess.Popen[str],
    read_fd: int,
    read_chunk,
    write_chunk,
    mock_cls,
    cobs_decode_fn,
    timeout_s: float,
) -> None:
    mock = mock_cls()
    for frame in mock.startup_fuzz_frames():
        mock.write_chunked(write_chunk, frame)

    rx = bytearray()
    deadline = time.monotonic() + timeout_s

    while proc.poll() is None:
        if time.monotonic() > deadline:
            proc.kill()
            print("loopback timeout exceeded", file=sys.stderr)
            break

        readable, _, _ = select.select([read_fd], [], [], 0.02)
        if not readable:
            continue

        try:
            chunk = read_chunk()
        except (BlockingIOError, OSError):
            break

        if not chunk:
            break

        rx.extend(chunk)
        while True:
            try:
                delim = rx.index(0)
            except ValueError:
                break

            encoded = bytes(rx[:delim])
            del rx[: delim + 1]

            if not encoded:
                continue

            inner = cobs_decode_fn(encoded)
            if inner is None:
                continue

            responses = mock.handle_inner(inner)
            for out_frame in responses:
                mock.write_chunked(write_chunk, out_frame)


def _launch_driver(driver_path: pathlib.Path, endpoint: str, iterations: int) -> subprocess.Popen[str]:
    return subprocess.Popen(
        [str(driver_path), endpoint, str(iterations)],
        cwd=ROOT,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
    )


def run_loopback_pty(driver_path: pathlib.Path, iterations: int, timeout_s: float) -> int:
    if pty is None:
        print("PTY loopback requires a POSIX platform (Linux/macOS/WSL).", file=sys.stderr)
        return 2

    _run_generator()
    mock_cls, cobs_decode_fn = _load_mock_helpers()

    master_fd, slave_fd = pty.openpty()
    slave_name = os.ttyname(slave_fd)

    proc = _launch_driver(driver_path, slave_name, iterations)
    os.close(slave_fd)

    try:
        _run_frame_pump(
            proc,
            master_fd,
            lambda: os.read(master_fd, 4096),
            lambda part: os.write(master_fd, part),
            mock_cls,
            cobs_decode_fn,
            timeout_s,
        )
    finally:
        try:
            os.close(master_fd)
        except OSError:
            pass

    output = _drain_driver_output(proc)
    if output:
        print(output)

    return proc.returncode or 0


def run_loopback_tcp(
    driver_path: pathlib.Path,
    iterations: int,
    timeout_s: float,
    host: str,
    port: int,
) -> int:
    _run_generator()
    mock_cls, cobs_decode_fn = _load_mock_helpers()

    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((host, port))
    listener.listen(1)
    bound_host, bound_port = listener.getsockname()

    proc = _launch_driver(driver_path, f"tcp://{bound_host}:{bound_port}", iterations)
    conn: socket.socket | None = None
    deadline = time.monotonic() + timeout_s

    try:
        while conn is None and proc.poll() is None:
            if time.monotonic() > deadline:
                proc.kill()
                print("loopback timeout exceeded", file=sys.stderr)
                break

            readable, _, _ = select.select([listener], [], [], 0.05)
            if not readable:
                continue
            conn, _ = listener.accept()
            conn.setblocking(False)

        if conn is not None:
            _run_frame_pump(
                proc,
                conn.fileno(),
                lambda: conn.recv(4096),
                conn.sendall,
                mock_cls,
                cobs_decode_fn,
                timeout_s,
            )
    finally:
        if conn is not None:
            conn.close()
        listener.close()

    output = _drain_driver_output(proc)
    if output:
        print(output)

    return proc.returncode or 0


def _select_transport(requested: str) -> str:
    if requested == "auto":
        if pty is not None:
            return "pty"
        return "tcp"

    if requested == "pty" and pty is None:
        raise ValueError("PTY transport is unavailable on this platform; use --transport tcp")

    return requested


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--driver",
        type=pathlib.Path,
        default=_default_driver_path(),
        help="Path to compiled loopback_driver binary",
    )
    parser.add_argument(
        "--transport",
        choices=["auto", "pty", "tcp"],
        default="auto",
        help="Harness transport mode (default: auto)",
    )
    parser.add_argument(
        "--tcp-host",
        default="127.0.0.1",
        help="TCP loopback listen host (used with --transport tcp)",
    )
    parser.add_argument(
        "--tcp-port",
        type=int,
        default=0,
        help="TCP loopback listen port (0 selects an ephemeral port)",
    )
    parser.add_argument("--iterations", type=int, default=120)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    if not args.driver.exists():
        print(f"driver not found: {args.driver}", file=sys.stderr)
        return 2

    if args.iterations <= 0:
        print("iterations must be > 0", file=sys.stderr)
        return 2

    if args.tcp_port < 0 or args.tcp_port > 65535:
        print("tcp-port must be in range 0..65535", file=sys.stderr)
        return 2

    try:
        transport = _select_transport(args.transport)
    except ValueError as exc:
        print(str(exc), file=sys.stderr)
        return 2

    if transport == "tcp":
        return run_loopback_tcp(args.driver, args.iterations, args.timeout, args.tcp_host, args.tcp_port)

    return run_loopback_pty(args.driver, args.iterations, args.timeout)


if __name__ == "__main__":
    raise SystemExit(main())
