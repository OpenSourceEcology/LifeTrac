"""deltas.py <evidence legs dir> <leg> | --selftest

RS-12.15 reverse-delivery scoring from the rs115 pre/post brackets of both boards
(leg<leg>_{pre,post}_{base,tractor}.txt, key=value lines written by rs115_stats_probe.py).
It scores the bracket criteria only; the gap and loss criteria come from
frag_gap_report.py (leg<leg>_gaps.txt) and rs12_leg_report.py.

Fails closed:
  * a missing bracket file, an empty bracket (a failed probe), or a counter the criteria
    read (REQUIRED) that is missing or `ABSENT` (the probe prints key=ABSENT when the
    firmware does not report it) stops the scoring with exit 2;
  * an L072 reset between the brackets (the harness's tractor SWD reset, or a
    STATS_RESET_REQ) restarts the counters at 0, so post minus pre would be meaningless.
    A reset is inferred only from (a) a decrease of an event counter or a high-water
    mark (NON_DECREASING; radio_state is a mode, not a counter, and never counts), or
    (b) d(host_parse_ok) < d(radio_tx_ok): every radio TX starts from a parsed host
    frame, and the brackets are taken with the daemons stopped;
  * the bracket delivery is cross-checked against the log counts leg_post.sh records in
    leg<leg>_gaps.txt ("base sends on air=N | tractor decoded cmds=M"). If they disagree
    or the counts are missing, an undetected reset cannot be ruled out: delivery and each
    zero-delta criterion that would pass are UNVERIFIED, and the STOP rule, unless it
    tripped, UNDETERMINED. A delta above 0 still fails: post minus pre never exceeds the
    true count.
Exit codes: 0 every bracket criterion PASS, 1 a criterion FAILED or is UNVERIFIED,
2 incomplete input or bad arguments.
"""
import os
import re
import sys

# Values that never go down while an L072 stays up: event counters, and the high-water
# marks host_stats.c only ever raises. All of them are cleared together by
# host_stats_reset() (boot or STATS_RESET_REQ), so a decrease means a reset.
NON_DECREASING = (
    "radio_dio0", "radio_rx_ok", "radio_tx_ok", "radio_crc_err",
    "host_parse_ok", "host_parse_err", "host_rx_ring_ovf", "host_dropped", "host_queue_full",
    "tx_fifo_rb_ok", "tx_fifo_rb_bad", "tx_done_early",
    "rx_urc_lost", "rx_pretx_drained", "rx_fifo_skip", "tx_deaf_sum_us",
    "fhss_dec_aligned", "fhss_dec_snapped", "fhss_dec_rej_not_init", "fhss_dec_rej_bad_hop",
    "fhss_dec_rej_epoch_drift", "fhss_dec_rej_locked_out",
    "clk_demotion_reset", "clk_demotion_kept", "tx_first_anchor",
    "tx_deaf_max_us", "tx_done_to_rearm_max_us", "tx_stream_streak_max",
)
# What the criteria read (both boards, both brackets).
REQUIRED = ("radio_tx_ok", "radio_rx_ok", "fhss_dec_snapped", "fhss_dec_rej_epoch_drift",
            "clk_demotion_reset", "tx_first_anchor", "tx_stream_streak_max")
SHOWN = ("radio_tx_ok", "radio_rx_ok", "radio_crc_err", "fhss_dec_aligned", "fhss_dec_snapped",
         "fhss_dec_rej_locked_out", "fhss_dec_rej_epoch_drift", "clk_demotion_reset",
         "clk_demotion_kept", "tx_first_anchor", "tx_stream_streak_max")

# the firmware's counters are unsigned (the probe unpacks '<I'): a signed or other
# malformed value does not match, so the key counts as missing
LINE = re.compile(r"([a-z][a-z0-9_]*)=(\d+|ABSENT)$")
INVARIANT = "d(host_parse_ok) < d(radio_tx_ok)"


def parse(text):
    """-> (values {key: int}, absent {key})"""
    vals, absent = {}, set()
    for raw in text.splitlines():
        m = LINE.match(raw.strip())
        if not m:
            continue
        if m.group(2) == "ABSENT":
            absent.add(m.group(1))
        else:
            vals[m.group(1)] = int(m.group(2))
    return vals, absent


def missing(vals, absent):
    return [k for k in REQUIRED if k not in vals or k in absent]


def reset_evidence(pre, post):
    ev = [k for k in NON_DECREASING if k in pre and k in post and post[k] < pre[k]]
    # sx1276_tx_begin() is only called for a parsed TX_FRAME_REQ (host_cmd.c), so
    # without a reset the brackets see at least as many parsed frames as TXs
    if all(k in pre and k in post for k in ("host_parse_ok", "radio_tx_ok")):
        if post["host_parse_ok"] - pre["host_parse_ok"] < post["radio_tx_ok"] - pre["radio_tx_ok"]:
            ev.append(INVARIANT)
    return ev


def deltas(pre, post):
    """-> (delta {key: (pre, post, delta)} for keys numeric in both brackets, reset evidence)"""
    ev = reset_evidence(pre, post)
    return {k: (pre[k], post[k], post[k] if ev else post[k] - pre[k])
            for k in post if k in pre}, ev


def log_counts(path):
    """leg<leg>_gaps.txt -> ((base sends on air, tractor decoded cmds) or None, why not)"""
    name = os.path.basename(path)
    if not os.path.exists(path):
        return None, f"{name} missing"
    m = re.search(r"base sends on air=(\d+)\s*\|\s*tractor decoded cmds=(\d+)",
                  open(path, encoding="utf-8", errors="replace").read())
    if not m:
        return None, f'no "base sends on air" line in {name}'
    return (int(m.group(1)), int(m.group(2))), ""


def score(E, LEG):
    brackets, bad = {}, []
    for w in ("base", "tractor"):
        for b in ("pre", "post"):
            p = os.path.join(E, f"leg{LEG}_{b}_{w}.txt")
            if not os.path.exists(p):
                bad.append(f"{w} {b}: file missing ({os.path.basename(p)})")
                continue
            vals, absent = parse(open(p, encoding="utf-8", errors="replace").read())
            miss = missing(vals, absent)
            if miss:
                why = "empty bracket (failed probe?)" if not vals else "missing/ABSENT: " + ", ".join(miss)
                bad.append(f"{w} {b}: {why}")
            brackets[(w, b)] = vals
    if bad:
        print(f"leg {LEG}: INCOMPLETE BRACKETS -- not scored")
        for x in bad:
            print(f"  {x}")
        return 2

    v, ev = {}, {}
    for w in ("base", "tractor"):
        v[w], ev[w] = deltas(brackets[(w, "pre")], brackets[(w, "post")])
        print(f"{w}: L072 counters " + (
            f"RESET during the leg (evidence: {', '.join(ev[w])}) -> delta = post"
            if ev[w] else "not reset -> delta = post - pre"))

    print(f"leg {LEG}: counter deltas (pre -> post)")
    print(f"  {'counter':28s} {'base':>18s} {'tractor':>18s}")
    for k in SHOWN:
        row = []
        for w in ("base", "tractor"):
            if k in v[w]:
                a, b, d = v[w][k]
                row.append(f"{a}->{b} (d={d})")
            else:
                row.append("n/a")
        print(f"  {k:28s} {row[0]:>18s} {row[1]:>18s}")

    bt = v["base"]["radio_tx_ok"][2]
    tr = v["tractor"]["radio_rx_ok"][2]
    ratio = f"{100.0 * tr / bt:.1f} %" if bt else "n/a"
    t = v["tractor"]
    b = v["base"]

    lc, why = log_counts(os.path.join(E, f"leg{LEG}_gaps.txt"))
    if lc is None:
        cross, cross_ok = f"no log cross-check ({why})", False
    else:
        cross = f"logs: base sends on air={lc[0]}, tractor decoded cmds={lc[1]}"
        cross_ok = lc == (bt, tr)
    print()
    print(f"cross-check: brackets {tr}/{bt} vs {cross} -> {'AGREE' if cross_ok else 'DISAGREE or unavailable'}")
    print()

    def zero(d):
        """a delta that must be 0: > 0 always fails; 0 passes only when the deltas are verified"""
        return "FAIL" if d > 0 else ("PASS" if cross_ok else "UNVERIFIED")

    print("bracket criteria (DC/TODO.md RS-12.15 reverse-delivery PASS / STOP):")
    deliver_ok = bt > 0 and tr * 100 >= 60 * bt
    rows = [
        (f"command delivery tractor d(radio_rx_ok)/base d(radio_tx_ok) = {tr}/{bt} = {ratio} >= 60 %",
         "PASS" if deliver_ok and cross_ok else ("UNVERIFIED" if deliver_ok else "FAIL")),
        (f"tractor clk_demotion_reset delta = {t['clk_demotion_reset'][2]} (must be 0; STOP if > 0)",
         zero(t["clk_demotion_reset"][2])),
        (f"tractor tx_first_anchor post = {t['tx_first_anchor'][1]} (must be 1; STOP if > 1)",
         "PASS" if t["tx_first_anchor"][1] == 1 else "FAIL"),
        (f"tractor fhss_dec_snapped delta = {t['fhss_dec_snapped'][2]} (must be 0)",
         zero(t["fhss_dec_snapped"][2])),
        (f"tractor fhss_dec_rej_epoch_drift delta = {t['fhss_dec_rej_epoch_drift'][2]} (must be 0)",
         zero(t["fhss_dec_rej_epoch_drift"][2])),
        (f"base tx_stream_streak_max post = {b['tx_stream_streak_max'][1]} (must be < 8)",
         "PASS" if b["tx_stream_streak_max"][1] < 8 else "FAIL"),
    ]
    for text, res in rows:
        print(f"  [{res}] {text}")
    if t["clk_demotion_reset"][2] > 0 or t["tx_first_anchor"][1] > 1:
        stop = "TRIPPED -- stop the round"
    elif not cross_ok:
        stop = "UNDETERMINED -- deltas not verified against the logs"
    else:
        stop = "not tripped"
    print(f"STOP RULE: {stop}")
    return 0 if all(res == "PASS" for _, res in rows) else 1


def selftest():
    import contextlib
    import io
    import tempfile

    def bracket(**kv):
        vals = dict.fromkeys(NON_DECREASING, 5)
        vals.update(radio_state=4, tx_first_anchor=1, clk_demotion_reset=0,
                    fhss_dec_snapped=0, fhss_dec_rej_epoch_drift=0, tx_stream_streak_max=1)
        vals.update(kv)
        return "\n".join(f"{k}={x}" for k, x in vals.items() if x is not None)

    # a clean leg: base sent 27 (23 -> 50), the tractor heard 22 (5 -> 27); radio_state 4 -> 1
    good = {
        "pre_base": bracket(radio_tx_ok=23, host_parse_ok=100),
        "post_base": bracket(radio_tx_ok=50, host_parse_ok=200),
        "pre_tractor": bracket(radio_tx_ok=900, radio_rx_ok=5, host_parse_ok=1000),
        "post_tractor": bracket(radio_tx_ok=950, radio_rx_ok=27, host_parse_ok=1100, radio_state=1),
        "gaps": "n_frag=10 max_gap=0.5s gaps>3s=0\nbase sends on air=27 | tractor decoded cmds=22\n",
    }

    def run(name, **change):
        files = dict(good, **change)
        with tempfile.TemporaryDirectory() as d:
            for part, text in files.items():
                if text is not None:
                    fn = f"legT_{part}.txt" if part != "gaps" else "legT_gaps.txt"
                    with open(os.path.join(d, fn), "w", encoding="utf-8") as f:
                        f.write(text)
            out = io.StringIO()
            with contextlib.redirect_stdout(out):
                rc = score(d, "T")
        return rc, out.getvalue()

    def check(name, want_rc, *want_text, **change):
        rc, out = run(name, **change)
        assert rc == want_rc and all(s in out for s in want_text), f"{name}: rc={rc}\n{out}"

    # parsing: ABSENT and missing fields are reported, never read as 0
    vals, absent = parse(bracket(clk_demotion_reset="ABSENT"))
    assert missing(vals, absent) == ["clk_demotion_reset"]
    vals, absent = parse("STATS_FAIL timeout waiting for response")
    assert vals == {} and set(missing(vals, absent)) == set(REQUIRED)

    check("clean leg, radio_state 4 -> 1 is not a reset", 0,
          "base: L072 counters not reset", "tractor: L072 counters not reset",
          "= 22/27 = 81.5 %", "-> AGREE", "STOP RULE: not tripped")
    for mark in ("tx_deaf_max_us", "tx_done_to_rearm_max_us", "tx_stream_streak_max"):
        check(f"lower {mark} = reset", 0, f"RESET during the leg (evidence: {mark})",
              pre_tractor=bracket(radio_tx_ok=900, radio_rx_ok=5, host_parse_ok=1000, **{mark: 900}),
              post_tractor=bracket(radio_tx_ok=950, radio_rx_ok=27, host_parse_ok=1100, **{mark: 100}),
              gaps="base sends on air=27 | tractor decoded cmds=27")
    check("fewer parsed frames than TXs = reset", 0, f"RESET during the leg (evidence: {INVARIANT})",
          post_tractor=bracket(radio_tx_ok=950, radio_rx_ok=27, host_parse_ok=1010),
          gaps="base sends on air=27 | tractor decoded cmds=27")
    check("lower event counter = reset", 1, "RESET during the leg (evidence: host_parse_ok",
          post_tractor=bracket(radio_tx_ok=950, radio_rx_ok=27, host_parse_ok=90))
    check("logs disagree", 1, "[UNVERIFIED] command delivery", "[UNVERIFIED] tractor clk_demotion_reset",
          "STOP RULE: UNDETERMINED", gaps="base sends on air=27 | tractor decoded cmds=20")
    check("no gaps file", 1, "legT_gaps.txt missing", "[UNVERIFIED] command delivery", gaps=None)
    check("no counts line", 1, 'no "base sends on air" line', gaps="frag_arrival lines: 1\n")
    check("delta > 0 fails even unverified", 1, "[FAIL] tractor fhss_dec_snapped",
          post_tractor=bracket(radio_tx_ok=950, radio_rx_ok=27, host_parse_ok=1100, fhss_dec_snapped=7),
          gaps=None)
    check("re-anchored tractor trips STOP", 1, "[FAIL] tractor tx_first_anchor post = 2", "STOP RULE: TRIPPED",
          post_tractor=bracket(radio_tx_ok=950, radio_rx_ok=27, host_parse_ok=1100, tx_first_anchor=2))
    check("epoch drift fails", 1, "[FAIL] tractor fhss_dec_rej_epoch_drift",
          post_tractor=bracket(radio_tx_ok=950, radio_rx_ok=27, host_parse_ok=1100, fhss_dec_rej_epoch_drift=1))
    check("base streak of 8 fails", 1, "[FAIL] base tx_stream_streak_max post = 8",
          post_base=bracket(radio_tx_ok=50, host_parse_ok=200, tx_stream_streak_max=8))
    check("delivery under 60 % fails", 1, "= 16/27 = 59.3 %", "[FAIL] command delivery",
          post_tractor=bracket(radio_tx_ok=950, radio_rx_ok=21, host_parse_ok=1100),
          gaps="base sends on air=27 | tractor decoded cmds=16")
    check("negative value refused", 2, "missing/ABSENT: fhss_dec_snapped",
          post_tractor=bracket(radio_tx_ok=950, radio_rx_ok=27, host_parse_ok=1100, fhss_dec_snapped=-1))
    check("STOP trips", 1, "[FAIL] tractor clk_demotion_reset", "STOP RULE: TRIPPED",
          post_tractor=bracket(radio_tx_ok=950, radio_rx_ok=27, host_parse_ok=1100, clk_demotion_reset=1))
    check("base sent nothing", 1, "= 22/0 = n/a", "[FAIL] command delivery",
          post_base=bracket(radio_tx_ok=23, host_parse_ok=200))
    check("other counter missing in pre shows n/a", 0,
          f"  {'radio_crc_err':28s} {'5->5 (d=0)':>18s} {'n/a':>18s}",
          pre_tractor=bracket(radio_tx_ok=900, radio_rx_ok=5, host_parse_ok=1000, radio_crc_err=None))
    check("ABSENT required counter", 2, "INCOMPLETE BRACKETS", "missing/ABSENT: clk_demotion_reset",
          post_tractor=bracket(radio_tx_ok=950, radio_rx_ok=27, host_parse_ok=1100, clk_demotion_reset="ABSENT"))
    check("failed probe", 2, "base pre: empty bracket", pre_base="STATS_FAIL timeout\n")
    check("missing bracket", 2, "tractor pre: file missing", pre_tractor=None)
    print("deltas.py selftest: OK")
    return 0


if __name__ == "__main__":
    if sys.argv[1:] == ["--selftest"]:
        sys.exit(selftest())
    if len(sys.argv) != 3:
        print(__doc__, file=sys.stderr)
        sys.exit(2)
    sys.exit(score(sys.argv[1], sys.argv[2]))
