"""Clear stale retained control topics on the local broker.

Bench rule: a stale retained encode_mode_override twice flipped the camera
mid-session, and a stale retained radio_profile pin made the rx daemon
re-command the tractor's profile at startup — clear on every broker the
daemon can see before a leg.

PR #125 review (2026-09-15): this used to publish the QoS-1 retained
deletes, sleep 1 s and print RETAINED-CLEARED unconditionally. A slow or
wedged broker could therefore leave the very pin this preflight exists to
remove while the operator read a success line. Each delete is now waited on
and the exit status reflects the truth:

    exit 0  every retained delete CONFIRMED by the broker
    exit 1  connect failed, a publish was rejected, or an ack timed out
"""
import sys
import time

import paho.mqtt.client as mqtt

TOPICS = [
    "lifetrac/v25/control/encode_mode_override",
    "lifetrac/v25/cmd/req_keyframe",
    # 2026-09-12: a retained radio_profile pin (left by a profile-switch
    # leg) makes the rx daemon command the tractor to that profile at
    # startup, fighting the harness and storming keyframe requests.
    "lifetrac/v25/control/radio_profile",
]

HOST = "127.0.0.1"
PORT = 1883
ACK_TIMEOUT_S = 5.0


def _wait_published(info, timeout_s: float) -> bool:
    """True once the broker has acked this QoS-1 publish.

    paho >= 1.6 takes a timeout; older builds block forever, so fall back
    to polling is_published() against our own deadline.
    """
    try:
        info.wait_for_publish(timeout=timeout_s)
    except TypeError:                      # paho < 1.6
        deadline = time.monotonic() + timeout_s
        while not info.is_published() and time.monotonic() < deadline:
            time.sleep(0.05)
    except (ValueError, RuntimeError):     # already failed/disconnected
        return False
    return bool(info.is_published())


def main() -> int:
    try:
        c = mqtt.Client(client_id="bench-retain-clear")
        c.connect(HOST, PORT, 10)
    except Exception as exc:
        print("RETAINED-CLEAR-FAILED connect %s:%d (%s)" % (HOST, PORT, exc))
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
        print("RETAINED-CLEAR-FAILED " + "; ".join(failed))
        return 1
    print("RETAINED-CLEARED " + " ".join(TOPICS))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
