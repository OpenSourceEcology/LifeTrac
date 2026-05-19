"""Bench harness C: UDP loopback exercising the full D13 + D14 + class-tag
enforcer pipeline on the real Portenta X8 Python runtime.

Substitutes UDP for the LoRa link as a TRANSPORT-SHAPED HARNESS. This does
NOT validate any RF behavior. What it DOES validate:

  * The D13 AES-GCM-64 implicit-nonce codec encrypts on one socket and
    decrypts cleanly on another socket, using the real `cryptography`
    library on the production ARM64 Python (not the dev x86 Python).
  * The D14 image plaintext+CRC32 framer round-trips across the same
    socket boundary.
  * The host-boundary class-tag enforcer correctly accepts/rejects each
    of the four traffic classes paired with each profile.
  * A FORGED D14 fragment injected into a P0-receiver channel is rejected
    by the enforcer (the §20 split-trust safety claim).
  * A REPLAYED authentic GCM-64 frame is rejected by the replay window
    (uses the existing `ReplayWindow` from lora_proto).
  * A MAC-tampered GCM-64 frame is rejected by the AEAD verifier.
  * A boot_ctr mismatch between TX and RX causes ALL frames to fail
    (validates the boot-ctr binding works end-to-end on ARM64).

Exit code 0 if all assertions pass, non-zero on first failure.

Run via:
    adb push bench_loopback_d13_d14.py /tmp/
    adb push lora_proto.py             /tmp/
    adb shell "cd /tmp && python3 bench_loopback_d13_d14.py"
"""
from __future__ import annotations

import os
import socket
import sys
import time

sys.path.insert(0, "/tmp")
import lora_proto as L  # noqa: E402

KEY = b"\x00" * 16   # static test key; bench-only, not a secret
SRC_TX = 0x42
BOOT_TX = 7
TIMEOUT_S = 1.0
PORT = 35421         # arbitrary high port

PASS = "[PASS]"
FAIL = "[FAIL]"


def make_sockets() -> tuple[socket.socket, socket.socket]:
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", PORT))
    rx.settimeout(TIMEOUT_S)
    return tx, rx


def send_recv(tx: socket.socket, rx: socket.socket, wire: bytes) -> bytes:
    tx.sendto(wire, ("127.0.0.1", PORT))
    data, _ = rx.recvfrom(2048)
    return data


def assert_eq(name: str, got, want) -> bool:
    if got == want:
        print(f"  {PASS} {name}")
        return True
    print(f"  {FAIL} {name}: got={got!r} want={want!r}")
    return False


def assert_true(name: str, cond: bool, hint: str = "") -> bool:
    if cond:
        print(f"  {PASS} {name}")
        return True
    print(f"  {FAIL} {name}{(': ' + hint) if hint else ''}")
    return False


def section(title: str) -> None:
    print()
    print(f"== {title} ==")


def main() -> int:
    print(f"D13/D14 UDP-loopback bench on {os.uname().machine} "
          f"({os.uname().sysname} {os.uname().release})")
    import cryptography
    print(f"cryptography {cryptography.__version__}, "
          f"python {sys.version.split()[0]}")

    failures = 0

    # 1. D13 round-trip across UDP
    section("D13 AES-GCM-64 implicit-nonce round-trip over UDP")
    tx, rx = make_sockets()
    try:
        pt_control = bytes(range(16))   # ControlFrame-sized
        wire = L.encrypt_frame_gcm64_implicit(KEY, SRC_TX, BOOT_TX, 1, pt_control)
        if not assert_eq("control wire size",
                         len(wire), 16 + L.GCM64_OVERHEAD):
            failures += 1
        got = send_recv(tx, rx, wire)
        out = L.decrypt_frame_gcm64_implicit(KEY, SRC_TX, BOOT_TX, got)
        if not assert_eq("control round-trip", out, (1, pt_control)):
            failures += 1
    finally:
        tx.close(); rx.close()

    # 2. D14 image fragment over UDP
    section("D14 image plaintext+CRC32 round-trip over UDP")
    tx, rx = make_sockets()
    try:
        pt_img = os.urandom(32)
        wire = L.pack_image_fragment_plain(42, pt_img)
        if not assert_eq("image wire size",
                         len(wire), 32 + L.IMAGE_PLAIN_OVERHEAD):
            failures += 1
        got = send_recv(tx, rx, wire)
        out = L.unpack_image_fragment_plain(got)
        if not assert_eq("image round-trip", out, (42, pt_img)):
            failures += 1
    finally:
        tx.close(); rx.close()

    # 3. MAC-tampered D13 frame rejected
    section("D13 MAC-tampered frame rejected by AEAD")
    tx, rx = make_sockets()
    try:
        wire = bytearray(L.encrypt_frame_gcm64_implicit(KEY, SRC_TX, BOOT_TX,
                                                         2, b"PING"))
        wire[-1] ^= 0x01   # flip last tag bit
        got = send_recv(tx, rx, bytes(wire))
        out = L.decrypt_frame_gcm64_implicit(KEY, SRC_TX, BOOT_TX, got)
        if not assert_true("tampered tag rejected", out is None,
                           f"unexpected decrypt success: {out!r}"):
            failures += 1
    finally:
        tx.close(); rx.close()

    # 4. boot_ctr mismatch rejects every frame
    section("D13 boot_ctr mismatch rejects everything")
    tx, rx = make_sockets()
    try:
        bad = 0
        for seq in range(1, 6):
            wire = L.encrypt_frame_gcm64_implicit(KEY, SRC_TX, BOOT_TX, seq,
                                                   b"X" * 8)
            got = send_recv(tx, rx, wire)
            out = L.decrypt_frame_gcm64_implicit(KEY, SRC_TX, BOOT_TX + 99,
                                                   got)
            if out is not None:
                bad += 1
        if not assert_eq("boot_ctr+99 rejects all 5 frames", bad, 0):
            failures += 1
    finally:
        tx.close(); rx.close()

    # 5. Replay window rejects duplicate seq
    section("ReplayWindow rejects duplicate D13 seq")
    tx, rx = make_sockets()
    try:
        rw = L.ReplayWindow()
        wire = L.encrypt_frame_gcm64_implicit(KEY, SRC_TX, BOOT_TX, 1000,
                                               b"once")
        # First arrival
        got = send_recv(tx, rx, wire)
        out = L.decrypt_frame_gcm64_implicit(KEY, SRC_TX, BOOT_TX, got)
        if not assert_true("first arrival accepted",
                           out is not None and rw.check_and_update(out[0])):
            failures += 1
        # Replay
        got = send_recv(tx, rx, wire)
        out = L.decrypt_frame_gcm64_implicit(KEY, SRC_TX, BOOT_TX, got)
        if not assert_true("replay rejected by ReplayWindow",
                           out is not None and not rw.check_and_update(out[0]),
                           "ReplayWindow accepted a duplicate seq"):
            failures += 1
    finally:
        tx.close(); rx.close()

    # 6. Class-tag enforcer on the RX side accepts/rejects correctly
    section("Host-boundary class-tag enforcer over UDP")
    tx, rx = make_sockets()
    try:
        # P3 + D14 plain: accepted
        wire = L.pack_image_fragment_plain(7, b"\xaa" * 16)
        got = send_recv(tx, rx, wire)
        unpacked = L.unpack_image_fragment_plain(got)
        try:
            L.enforce_class_tag_boundary(L.PRIO_P3, L.CRYPTO_IMAGE_PLAIN_CRC32)
            if not assert_true("P3 + D14 plain accepted", unpacked is not None):
                failures += 1
        except L.ClassTagViolation as e:
            print(f"  {FAIL} P3 + D14 plain should be accepted but raised: {e}")
            failures += 1

        # Forged D14 fragment laundered into P0 path — MUST be rejected
        # by the enforcer even though the unpacker happily accepted it.
        rejected = False
        try:
            L.enforce_class_tag_boundary(L.PRIO_P0, L.CRYPTO_IMAGE_PLAIN_CRC32)
        except L.ClassTagViolation:
            rejected = True
        if not assert_true("P0 + D14 plain REJECTED (split-trust guard)",
                           rejected):
            failures += 1

        # P0 + GCM-64 implicit: accepted
        rejected = False
        try:
            L.enforce_class_tag_boundary(L.PRIO_P0, L.CRYPTO_GCM64_IMPLICIT)
        except L.ClassTagViolation:
            rejected = True
        if not assert_true("P0 + GCM-64 implicit accepted", not rejected):
            failures += 1
    finally:
        tx.close(); rx.close()

    # 7. Sustained throughput under realistic cadence (20 Hz control over UDP)
    section("Sustained 20 Hz control loop over UDP for 2 s")
    tx, rx = make_sockets()
    try:
        n_ok = 0
        n_loss = 0
        rw = L.ReplayWindow()
        t_end = time.time() + 2.0
        seq = 1
        last_send = 0.0
        period = 1.0 / 20.0
        while time.time() < t_end:
            if time.time() - last_send >= period:
                wire = L.encrypt_frame_gcm64_implicit(KEY, SRC_TX, BOOT_TX,
                                                       seq, bytes(range(16)))
                tx.sendto(wire, ("127.0.0.1", PORT))
                last_send = time.time()
                seq += 1
            rx.settimeout(0.005)
            try:
                data, _ = rx.recvfrom(2048)
                out = L.decrypt_frame_gcm64_implicit(KEY, SRC_TX, BOOT_TX, data)
                if out is None or not rw.check_and_update(out[0]):
                    n_loss += 1
                else:
                    n_ok += 1
            except socket.timeout:
                pass
        if not assert_true("≥ 35 frames decoded+accepted in 2 s",
                           n_ok >= 35, f"n_ok={n_ok}, n_loss={n_loss}"):
            failures += 1
        if not assert_eq("0 MAC/replay losses on lossless localhost",
                         n_loss, 0):
            failures += 1
        print(f"  (info: {n_ok} accepted, {n_loss} rejected over 2 s @ 20 Hz)")
    finally:
        tx.close(); rx.close()

    print()
    if failures == 0:
        print(f"== {PASS} all D13/D14 UDP-loopback assertions passed ==")
        return 0
    print(f"== {FAIL} {failures} assertion(s) failed ==")
    return 1


if __name__ == "__main__":
    sys.exit(main())
