"""Bench script B: AES-GCM-64 + GCM-128 throughput microbench on Portenta X8.

Reports:
  * cryptography library version + AES-NI / hwaccel hint
  * encrypt+decrypt throughput for the three crypto profiles at the three
    relevant payload sizes:
        - 16 B (ControlFrame, P0, 20 Hz)
        - 32 B (image fragment, P3, ~10 Hz per design refresh)
        - 100 B (telemetry, P2, ~5 Hz)
  * estimated CPU cost per second for the design cadences

Run via:
    adb push bench_crypto_perf.py /tmp/bench_crypto_perf.py
    adb push lora_proto.py        /tmp/lora_proto.py
    adb shell "cd /tmp && python3 bench_crypto_perf.py"
"""
from __future__ import annotations

import os
import platform
import sys
import time

sys.path.insert(0, "/tmp")
import lora_proto as L  # noqa: E402


def bench(fn, iters: int) -> float:
    """Return seconds per call (mean)."""
    t0 = time.perf_counter()
    for _ in range(iters):
        fn()
    return (time.perf_counter() - t0) / iters


def main() -> int:
    print(f"== Portenta X8 crypto microbench ==")
    print(f"python  : {sys.version.split()[0]}")
    print(f"platform: {platform.machine()} {platform.system()} {platform.release()}")
    try:
        import cryptography
        print(f"crypto  : cryptography {cryptography.__version__}")
    except ImportError:
        print("crypto  : cryptography MISSING — abort")
        return 2

    key = os.urandom(16)
    src = 0x42
    boot_ctr = 7
    payloads = {
        "16 B ControlFrame (P0, 20 Hz)": (bytes(range(16)), 20.0),
        "32 B image frag   (P3, 10 Hz)": (bytes(range(32)), 10.0),
        "100 B telemetry   (P2,  5 Hz)": (bytes(range(100)), 5.0),
    }
    iters = 2000

    def hdr():
        print()
        print(f"{'case':40s} {'enc ms':>8s} {'dec ms':>8s} "
              f"{'rt ms':>8s} {'CPU% @cadence':>14s}")
        print("-" * 82)

    hdr()
    for label, (pt, hz) in payloads.items():
        # GCM-128 explicit (shipped)
        seq_box = [0]
        def enc_shipped():
            seq_box[0] += 1
            L.encrypt_frame(key, src, seq_box[0], pt)
        ct = L.encrypt_frame(key, src, 1, pt)
        def dec_shipped():
            L.decrypt_frame(key, ct)
        e1 = bench(enc_shipped, iters)
        d1 = bench(dec_shipped, iters)
        rt1 = e1 + d1
        cpu1 = rt1 * hz * 100.0
        print(f"{label + '  GCM-128 explicit':40s} "
              f"{e1*1e3:8.3f} {d1*1e3:8.3f} {rt1*1e3:8.3f} {cpu1:13.3f}%")

        # GCM-64 implicit (D13)
        seq_box[0] = 0
        def enc_d13():
            seq_box[0] += 1
            L.encrypt_frame_gcm64_implicit(key, src, boot_ctr, seq_box[0], pt)
        ct13 = L.encrypt_frame_gcm64_implicit(key, src, boot_ctr, 1, pt)
        def dec_d13():
            L.decrypt_frame_gcm64_implicit(key, src, boot_ctr, ct13)
        e2 = bench(enc_d13, iters)
        d2 = bench(dec_d13, iters)
        rt2 = e2 + d2
        cpu2 = rt2 * hz * 100.0
        print(f"{label + '  GCM-64  implicit':40s} "
              f"{e2*1e3:8.3f} {d2*1e3:8.3f} {rt2*1e3:8.3f} {cpu2:13.3f}%")

    # D14 image plain (no crypto, sanity baseline)
    print()
    print("D14 image plain+CRC32 (no crypto):")
    pt32 = bytes(range(32))
    seq_box = [0]
    def pack14():
        seq_box[0] += 1
        L.pack_image_fragment_plain(seq_box[0], pt32)
    wire14 = L.pack_image_fragment_plain(1, pt32)
    def unpack14():
        L.unpack_image_fragment_plain(wire14)
    e3 = bench(pack14, iters)
    d3 = bench(unpack14, iters)
    print(f"  pack 32 B  : {e3*1e6:8.2f} us")
    print(f"  unpack 32 B: {d3*1e6:8.2f} us")

    # Aggregate at the design cadence (sum CPU% across all classes)
    print()
    print("== Aggregate CPU%% at full design cadence ==")
    # Worst-case running tally if we run all 3 classes simultaneously
    # under D13 on TX and decrypt on RX.
    total = 0.0
    detail = []
    for label, (pt, hz) in payloads.items():
        seq_box[0] = 0
        ct = L.encrypt_frame_gcm64_implicit(key, src, boot_ctr, 1, pt)
        e = bench(lambda: L.encrypt_frame_gcm64_implicit(key, src, boot_ctr,
                                                          1, pt), iters)
        d = bench(lambda: L.decrypt_frame_gcm64_implicit(key, src, boot_ctr,
                                                          ct), iters)
        cpu = (e + d) * hz * 100.0
        total += cpu
        detail.append((label, cpu))
    for label, cpu in detail:
        print(f"  {label:40s} {cpu:6.3f}%%")
    print(f"  {'TOTAL (single-core)':40s} {total:6.3f}%%")
    if total > 25.0:
        print("  WARN: > 25%% of a single core — risk to audio/video pipeline")
        return 1
    print("  OK: well under 25%% single-core budget")
    return 0


if __name__ == "__main__":
    sys.exit(main())
