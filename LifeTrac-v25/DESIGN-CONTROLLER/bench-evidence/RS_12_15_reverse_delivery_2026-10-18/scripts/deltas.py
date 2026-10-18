"""deltas.py <evidence legs dir> <leg>: RS-12.15 reverse-delivery scoring from the
rs115 pre/post brackets of both boards (key=value lines)."""
import re, sys, os

E, LEG = sys.argv[1], sys.argv[2]

def read(path):
    d = {}
    for line in open(path, encoding="utf-8", errors="replace"):
        m = re.match(r"\s*([a-z][a-z0-9_]*)=(-?\d+)\s*$", line.strip())
        if m:
            d[m.group(1)] = int(m.group(2))
    return d

v = {}
reset = {}
for w in ("base", "tractor"):
    pre = read(os.path.join(E, f"leg{LEG}_pre_{w}.txt"))
    post = read(os.path.join(E, f"leg{LEG}_post_{w}.txt"))
    # The harness resets an L072 at leg start (tractor: OpenOCD SWD NRST); its RAM
    # counters then restart at 0 and post-minus-pre is meaningless. Counters only
    # grow otherwise, so any decrease marks a reset: use the post value as the delta.
    dec = [k for k in post if k in pre and post[k] < pre[k]]
    reset[w] = dec
    v[w] = {k: (pre.get(k, 0), post[k], post[k] if dec else post[k] - pre.get(k, 0)) for k in post}
for w in ("base", "tractor"):
    print(f"{w}: L072 counters {'RESET during the leg (decreased: ' + ', '.join(reset[w][:4]) + ') -> delta = post' if reset[w] else 'not reset -> delta = post - pre'}")

keys = ["radio_tx_ok", "radio_rx_ok", "radio_crc_err", "fhss_dec_aligned", "fhss_dec_snapped",
        "fhss_dec_rej_locked_out", "fhss_dec_rej_epoch_drift", "clk_demotion_reset",
        "clk_demotion_kept", "tx_first_anchor", "tx_stream_streak_max"]
print(f"leg {LEG}: counter deltas (pre -> post)")
print(f"  {'counter':28s} {'base':>18s} {'tractor':>18s}")
for k in keys:
    row = []
    for w in ("base", "tractor"):
        if k in v[w]:
            a, b, d = v[w][k]
            row.append(f"{a}->{b} (d={d})")
        else:
            row.append("n/a")
    print(f"  {k:28s} {row[0]:>18s} {row[1]:>18s}")

bt = v["base"].get("radio_tx_ok", (0, 0, 0))[2]
tr = v["tractor"].get("radio_rx_ok", (0, 0, 0))[2]
ratio = 100.0 * tr / bt if bt else float("nan")
t = {k: v["tractor"].get(k, (0, 0, 0)) for k in keys}
b = {k: v["base"].get(k, (0, 0, 0)) for k in keys}
print()
print("criteria (DC/TODO.md RS-12.15 reverse-delivery PASS / STOP):")
chk = [
    (f"command delivery tractor d(radio_rx_ok)/base d(radio_tx_ok) = {tr}/{bt} = {ratio:.1f} % >= 60 %", ratio >= 60),
    (f"tractor clk_demotion_reset delta = {t['clk_demotion_reset'][2]} (must be 0; STOP if > 0)", t["clk_demotion_reset"][2] == 0),
    (f"tractor tx_first_anchor post = {t['tx_first_anchor'][1]} (must be 1; STOP if > 1)", t["tx_first_anchor"][1] == 1),
    (f"tractor fhss_dec_snapped delta = {t['fhss_dec_snapped'][2]} (must be 0)", t["fhss_dec_snapped"][2] == 0),
    (f"tractor fhss_dec_rej_epoch_drift delta = {t['fhss_dec_rej_epoch_drift'][2]} (must be 0)", t["fhss_dec_rej_epoch_drift"][2] == 0),
    (f"base tx_stream_streak_max post = {b['tx_stream_streak_max'][1]} (must be < 8)", b["tx_stream_streak_max"][1] < 8),
]
for text, ok in chk:
    print(f"  [{'PASS' if ok else 'FAIL'}] {text}")
stop = t["clk_demotion_reset"][2] > 0 or t["tx_first_anchor"][1] > 1
print(f"STOP RULE: {'TRIPPED -- stop the round' if stop else 'not tripped'}")
