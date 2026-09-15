"""Clear stale retained control topics on another host's broker.

    py -3 clear_retained_host.py [host]

Same preflight as clear_retained.py, for a broker that may simply not be
running (the PC-side broker is often down, and the harness then starts a
fresh one — that case is fine and exits 0).

PR #125 review (2026-09-15): the old version wrapped everything in one
try/except that printed NO-BROKER and exited 0 for ANY exception —
authentication failures, protocol errors and rejected publishes included.
A reachable broker that refused the deletes therefore kept the stale
control pin while the preflight looked non-fatal. Now only an absent
broker is tolerated:

    exit 0  deletes confirmed, OR the broker is genuinely not reachable
    exit 1  broker answered but the retained deletes were not confirmed
"""
import errno
import socket
import sys
import time

import paho.mqtt.client as mqtt

TOPICS = [
    "lifetrac/v25/control/encode_mode_override",
    "lifetrac/v25/cmd/req_keyframe",
    "lifetrac/v25/control/radio_profile",
]

PORT = 1883
ACK_TIMEOUT_S = 5.0

# "Nothing is listening / cannot get there" — the only tolerated case.
_ABSENT_ERRNOS = {errno.ECONNREFUSED, errno.EHOSTUNREACH, errno.ENETUNREACH,
                  errno.ETIMEDOUT, errno.EHOSTDOWN}


def _is_absent_broker(exc: BaseException) -> bool:
    if isinstance(exc, socket.gaierror):          # name does not resolve
        return True
    if isinstance(exc, (ConnectionRefusedError, socket.timeout)):
        return True
    if isinstance(exc, OSError) and exc.errno in _ABSENT_ERRNOS:
        return True
    return False


def _wait_published(info, timeout_s: float) -> bool:
    try:
        info.wait_for_publish(timeout=timeout_s)
    except TypeError:                              # paho < 1.6
        deadline = time.monotonic() + timeout_s
        while not info.is_published() and time.monotonic() < deadline:
            time.sleep(0.05)
    except (ValueError, RuntimeError):
        return False
    return bool(info.is_published())


def main() -> int:
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    c = mqtt.Client(client_id="bench-retain-clear-%s" % host)
    try:
        c.connect(host, PORT, 5)
    except Exception as exc:
        if _is_absent_broker(exc):
            print("NO-BROKER host=%s (%s) -- nothing retained there; "
                  "harness will start a fresh one" % (host, exc))
            return 0
        # Reached something that refused us (auth, protocol, TLS): the
        # stale pin may still be live and we must not look successful.
        print("RETAINED-CLEAR-FAILED host=%s connect (%s: %s)"
              % (host, type(exc).__name__, exc))
        return 1
    c.loop_start()
    failed = []
    try:
        for t in TOPICS:
            info = c.publish(t, None, qos=1, retain=True)
            if info.rc != mqtt.MQTT_ERR_SUCCESS:
                failed.append("%s (publish rc=%d)" % (t, info.rc))
                continue
            if not _wait_published(info, ACK_TIMEOUT_S):
                failed.append("%s (no broker ack in %.0f s)" % (t, ACK_TIMEOUT_S))
    finally:
        c.loop_stop()
        try:
            c.disconnect()
        except Exception:
            pass
    if failed:
        print("RETAINED-CLEAR-FAILED host=%s : %s" % (host, "; ".join(failed)))
        return 1
    print("RETAINED-CLEARED host=%s : %s" % (host, " ".join(TOPICS)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
