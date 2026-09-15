"""Two-opcode command-plane contention injector (PR #121 shared-gate leg L).

GOAL: drive TWO distinct pending opcodes at the base rx daemon so the pump wants to send them faster than the 1.0 s stream gate allows. Whether the gate actually has to hold is an OUTCOME read from the counters after the leg, not a guarantee of running this: on the profile-1 legs the per-opcode backoff and the idle-drain cadence already separated the candidates and cmd_gate_held stayed 0. What the leg measures is command spacing under the injected load:

  * REQ_KEYFRAME  every 5 s  (lifetrac/v25/cmd/req_keyframe)
  * ENCODE_MODE   every 0.7 s, mode 0 (webp, the current default -> NO codec
    change) with the quality byte cycled 78..88 so each publish is a NEW
    body and stays actively pending instead of being de-duplicated.

Both publish to the BASE broker (127.0.0.1), qos=0, non-retained. Run on the
base inside lifetrac-v25:latest with --network=host while a harness leg runs.
"""
import json
import sys
import time

import paho.mqtt.client as mqtt

KF_TOPIC = "lifetrac/v25/cmd/req_keyframe"
EM_TOPIC = "lifetrac/v25/control/encode_mode_override"
DUR_S = float(sys.argv[1]) if len(sys.argv) > 1 else 250.0

c = mqtt.Client(client_id="bench-dual-injector")
c.connect("127.0.0.1", 1883, 30)
c.loop_start()

t0 = time.time()
i = 0
n_em = 0
n_kf = 0
last_kf = 0.0
q = 78
while time.time() - t0 < DUR_S:
    c.publish(EM_TOPIC, json.dumps({"mode": 0, "quality": q}), qos=0)
    n_em += 1
    q = 78 + ((q - 78 + 1) % 11)          # 78..88, webp only
    now = time.time()
    if now - last_kf >= 5.0:
        c.publish(KF_TOPIC, b"bench-contention", qos=0)
        n_kf += 1
        last_kf = now
    if i % 14 == 0:
        print("DUAL t=%.0f em=%d kf=%d" % (now - t0, n_em, n_kf), flush=True)
    i += 1
    time.sleep(0.7)

c.loop_stop()
print("DUAL_DONE em=%d kf=%d dur=%.0f" % (n_em, n_kf, time.time() - t0))
