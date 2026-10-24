#!/usr/bin/env python3
"""bracket_deltas.py -- one leg's L072 counter deltas, and the RS-12.15 bracket criteria on FHSS.

    py -3 bracket_deltas.py <legs dir> <leg> [--profile auto|fhss|dts] [--params <params.txt | archive dir>]
    py -3 bracket_deltas.py --selftest

Reads the rs115 pre/post brackets of both boards (leg<leg>_{pre,post}_{base,tractor}.txt,
key=value lines written by rs115_stats_probe.py through leg_post.sh) and the log counts in
leg<leg>_gaps.txt (frag_gap_report.py). It scores the bracket criteria only; the gap and
loss criteria come from frag_gap_report.py and rs12_leg_report.py, and DTS command delivery
from rs13_probe_score.py. The kit copy of the 2026-10-18 scorer
(bench-evidence/RS_12_15_reverse_delivery_2026-10-18/scripts/deltas.py, which stays as
flown); it differs in three places: tx_first_anchor is judged by the leg's delta, the base's
tx_stream_streak_max is read as the since-boot high-water mark it is, and the profile picks
the criteria.

Profile (--profile auto, the default): reg_profile from leg<leg>_params.txt (leg_post.sh
copies the archive's params.txt there) or from --params. FHSS (reg_profile=1) applies the
RS-12.15 criteria and the STOP rule. DTS (2) and the bench-only fixed 915 MHz profile (0)
apply none of them, since every one is FHSS-only (FHSS decode, slot-clock authority, the
base's send streak, the 60 % delivery bar): the deltas, the reset evidence and the
cross-check print, exit 0. --profile fhss|dts names the profile when there are no params;
it must agree with params that give one.

Fails closed:
  * a missing bracket file, an empty bracket (a failed probe), or a counter the criteria
    read (REQUIRED) that is missing or `ABSENT` (the probe prints key=ABSENT when the
    firmware does not report it) stops the scoring with exit 2; so does a profile that
    cannot be told (auto with no params or no reg_profile in them, a value other than
    0, 1 or 2, or --profile against the params);
  * an L072 reset between the brackets (the harness's tractor SWD reset, or a
    STATS_RESET_REQ) restarts the counters at 0, so post minus pre would be meaningless.
    A reset is inferred only from (a) a decrease of an event counter or a high-water
    mark (NON_DECREASING; radio_state is a mode, not a counter, and never counts), or
    (b) d(host_parse_ok) < d(radio_tx_ok): every radio TX starts from a parsed host
    frame, and the brackets are taken with the daemons stopped. After a reset the delta
    is the post value;
  * the bracket delivery is cross-checked against the log counts leg_post.sh records in
    leg<leg>_gaps.txt ("base sends on air=N | tractor decoded cmds=M"). If they disagree
    or the counts are missing, an undetected reset cannot be ruled out: on FHSS, delivery
    and each criterion that would pass are UNVERIFIED, and the STOP rule, unless it
    tripped, UNDETERMINED. A delta above its limit still fails: post minus pre never
    exceeds the true count.
tx_first_anchor: each profile-1 activation resets the slot clock and the first TX
re-anchors it (host_cfg_profile.c, sx1276_rx_scan_reset; sx1276_tx.c), so an FHSS leg
adds exactly one. Judged by the delta: a tractor that was not reset before the leg reads
post 2 on its second FHSS leg and is healthy; delta 2 or more trips STOP.
Base tx_stream_streak_max: a high-water mark since the base L072 booted (the harness pulses
gpio163 but does not reset it). Below 8 it passes (once verified, like every criterion);
at 8 or more it fails when it rose during the leg (or the counters restarted), and is
UNVERIFIED when it stood there before the leg (this leg's own longest streak cannot be
read from it).
Command delivery on an FHSS leg that sent no command (brackets and logs 0/0, an image-only
leg such as the RS-13 FHSS walk) is N/A, not FAIL; brackets at 0/0 that the logs do not
confirm are UNVERIFIED. The base sending nothing while the tractor counts commands fails.
A base that never locked (2026-10-24, the FHSS round's leg 6): on an FHSS leg, base
d(clk_demotion_reset) 0 and d(fhss_dec_aligned) 0 while the tractor streamed (its
d(radio_tx_ok) at least STREAMED_TX), with post radio_state 4 (RX_CONT) on both boards
(or no radio_state in a post bracket), is the fingerprint of a base FHSS scan that ran out
into FAILED: no LOCKED -> SCANNING edge and no aligned decode all leg. It prints
`!! BASE NEVER LOCKED` above the STOP RULE line, and the STOP RULE line points at it. The
flag changes no verdict: the criteria, the STOP rule and the exit code are judged as
before. They read the tractor's counters and the probe counts, which a base parked on one
channel leaves clean, so with the log cross-check they can pass without testing FHSS
reverse delivery (see rs12_leg_report.py's acquisition line); a base that caught fewer
than two fragments gets no command-plane tally in leg<leg>_gaps.txt (frag_gap_report.py),
so there each criterion that would pass is UNVERIFIED, the STOP rule (unless it tripped)
UNDETERMINED, exit 1.
Exit codes: 0 every bracket criterion PASS or N/A (FHSS) or the non-FHSS report printed,
1 a criterion FAILED or is UNVERIFIED, 2 incomplete input or bad arguments.
--selftest runs base_station/tests/test_bracket_deltas_sil.py (the cases CI runs).
"""
from __future__ import annotations

import argparse
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
# Counters only the FHSS decoder and slot clock move; a non-FHSS leg should leave them
# alone. fhss_dec_rej_not_init is not one: every DTS receive counts there.
FHSS_ONLY = ("fhss_dec_aligned", "fhss_dec_snapped", "fhss_dec_rej_bad_hop",
             "fhss_dec_rej_epoch_drift", "fhss_dec_rej_locked_out", "clk_demotion_reset",
             "clk_demotion_kept", "tx_first_anchor")
STREAK_LIMIT = 8          # the base must never stream into clock authority (>= 8 own TX)
# 2026-10-24: "the tractor streamed" for the never-locked fingerprint: 10 s of frames at
# 2 fps (leg 6's tractor sent 607 while its base never locked)
STREAMED_TX = 20
RADIO_STATE_RX_CONT = 4   # sx1276_modes.h SX1276_STATE_RX_CONT
# host_cfg_keys.h REG_PROFILE_*
PROFILE_NAMES = {"0": "bench-only fixed 915 MHz", "1": "FHSS", "2": "DTS"}
FLAG_PROFILE = {"fhss": "1", "dts": "2"}

# the firmware's counters are unsigned (the probe unpacks '<I'): a signed or other
# malformed value does not match, so the key counts as missing
LINE = re.compile(r"([a-z][a-z0-9_]*)=(\d+|ABSENT)$")
INVARIANT = "d(host_parse_ok) < d(radio_tx_ok)"


class ProfileError(Exception):
    """The leg's profile cannot be told (exit 2)."""


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
    with open(path, encoding="utf-8", errors="replace") as fh:
        m = re.search(r"base sends on air=(\d+)\s*\|\s*tractor decoded cmds=(\d+)", fh.read())
    if not m:
        return None, f'no "base sends on air" line in {name}'
    return (int(m.group(1)), int(m.group(2))), ""


def parse_params(text):
    """params.txt (key=value lines, a BOM on the first) -> {key: value}"""
    out = {}
    for line in text.splitlines():
        key, sep, val = line.lstrip("﻿").partition("=")
        if sep:
            out[key.strip()] = val.strip()
    return out


def leg_profile(E, LEG, flag="auto", params=None):
    """-> (reg_profile "0" / "1" / "2", where it came from). Raises ProfileError.

    ``params``: a params.txt or a harness archive dir holding one; default
    <E>/leg<LEG>_params.txt. ``flag``: auto, fhss or dts."""
    path = params if params is not None else os.path.join(E, f"leg{LEG}_params.txt")
    if params is not None and os.path.isdir(path):
        path = os.path.join(path, "params.txt")
    name = path if params is not None else os.path.basename(path)
    reg = None
    if os.path.isfile(path):
        with open(path, encoding="utf-8-sig", errors="replace") as fh:
            reg = parse_params(fh.read()).get("reg_profile", "")
    elif params is not None:
        raise ProfileError(f"--params {params}: no params file at {path}")
    if flag != "auto":
        want = FLAG_PROFILE[flag]
        if reg and reg != want:
            raise ProfileError(f"--profile {flag} (reg_profile={want}) contradicts "
                               f"reg_profile={reg} in {name}")
        return want, f"--profile {flag}" + (f", as {name} says" if reg == want else "")
    if reg is None:
        raise ProfileError(f"{name} missing (leg_post.sh copies the archive's params.txt "
                           f"there): FHSS and DTS legs are scored differently. Give --params "
                           f"<archive dir> or --profile fhss|dts")
    if reg not in PROFILE_NAMES:
        raise ProfileError(f"{name}: " + (f"reg_profile={reg!r} is not 0, 1 or 2" if reg else
                                          "no reg_profile value")
                           + ". Give --profile fhss|dts")
    return reg, name


def score(E, LEG, profile="auto", params=None):
    try:
        reg, src = leg_profile(E, LEG, profile, params)
    except ProfileError as exc:
        print(f"leg {LEG}: PROFILE UNKNOWN -- not scored")
        print(f"  {exc}")
        return 2
    fhss = reg == "1"

    brackets, bad = {}, []
    for w in ("base", "tractor"):
        for b in ("pre", "post"):
            p = os.path.join(E, f"leg{LEG}_{b}_{w}.txt")
            if not os.path.exists(p):
                bad.append(f"{w} {b}: file missing ({os.path.basename(p)})")
                continue
            with open(p, encoding="utf-8", errors="replace") as fh:
                vals, absent = parse(fh.read())
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

    print(f"leg {LEG}: reg_profile={reg} ({PROFILE_NAMES[reg]}, from {src}) -> " + (
        "the RS-12.15 bracket criteria apply" if fhss else
        "deltas, reset evidence and cross-check only (the RS-12.15 criteria are FHSS-only)"))
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

    if not fhss:
        print(f"bracket criteria: none on a {PROFILE_NAMES[reg]} leg (reg_profile={reg}); every "
              f"RS-12.15 criterion and the STOP rule is FHSS-only")
        if reg == "2":
            print("  DTS command delivery: rs13_probe_score.py (the PROBE seqs); the cross-check "
                  "above is the bracket / log agreement only")
        moved = [f"{w} {k} d={v[w][k][2]}" for w in ("base", "tractor") for k in FHSS_ONLY
                 if k in v[w] and v[w][k][2] > 0]
        if moved:
            print(f"  note: FHSS-only counter(s) moved on a non-FHSS leg ({', '.join(moved)}): "
                  f"check the leg's profile")
        print("STOP RULE: not applied (FHSS only)")
        return 0

    def zero(d):
        """a delta that must be 0: > 0 always fails; 0 passes only when the deltas are verified"""
        return "FAIL" if d > 0 else ("PASS" if cross_ok else "UNVERIFIED")

    def one(d):
        """the FHSS leg's one slot-clock anchor: > 1 always fails; 1 passes, and 0 fails,
        only when the deltas are verified (an undetected reset hides counts)"""
        if d > 1:
            return "FAIL"
        return ("PASS" if d == 1 else "FAIL") if cross_ok else "UNVERIFIED"

    anchors = t["tx_first_anchor"][2]
    s_pre, s_post, s_d = b["tx_stream_streak_max"]
    if s_post < STREAK_LIMIT:
        streak, s_why = ("PASS" if cross_ok else "UNVERIFIED"), ""
    elif ev["base"] or s_post > s_pre:
        streak, s_why = "FAIL", ""
    else:
        streak = "UNVERIFIED"
        s_why = (f"; it stood at {s_post} before the leg, so this leg's own longest streak "
                 f"is not measurable from it")

    print("bracket criteria (DC/TODO.md RS-12.15 reverse-delivery PASS / STOP):")
    deliver_ok = bt > 0 and tr * 100 >= 60 * bt
    if bt == 0 and tr == 0:
        # an image-only leg (the RS-13 FHSS walk: -ReactiveFire 0, no -InjectCmd) has no
        # command to deliver: N/A when the logs agree (0/0); otherwise an undetected reset
        # could hide sends, so unverified, like every criterion the logs do not confirm
        delivery = (("command delivery: no command sent (brackets and logs 0/0, an image-only "
                     "leg); not scored", "N/A") if cross_ok else
                    ("command delivery: no command counted (brackets 0/0), not confirmed by "
                     "the logs", "UNVERIFIED"))
    else:
        delivery = (f"command delivery tractor d(radio_rx_ok)/base d(radio_tx_ok) = {tr}/{bt} = "
                    f"{ratio} >= 60 %",
                    "PASS" if deliver_ok and cross_ok else ("UNVERIFIED" if deliver_ok else "FAIL"))
    rows = [
        delivery,
        (f"tractor clk_demotion_reset delta = {t['clk_demotion_reset'][2]} (must be 0; STOP if > 0)",
         zero(t["clk_demotion_reset"][2])),
        (f"tractor tx_first_anchor delta = {anchors} (post {t['tx_first_anchor'][1]}; must be 1, "
         f"the activation's anchor; STOP if > 1)",
         one(anchors)),
        (f"tractor fhss_dec_snapped delta = {t['fhss_dec_snapped'][2]} (must be 0)",
         zero(t["fhss_dec_snapped"][2])),
        (f"tractor fhss_dec_rej_epoch_drift delta = {t['fhss_dec_rej_epoch_drift'][2]} (must be 0)",
         zero(t["fhss_dec_rej_epoch_drift"][2])),
        (f"base tx_stream_streak_max post = {s_post} (d={s_d}; a high-water mark since the base "
         f"L072 booted; must be < {STREAK_LIMIT}{s_why})",
         streak),
    ]
    for text, res in rows:
        print(f"  [{res}] {text}")
    unlocked = never_locked(v, brackets)
    if unlocked:
        print(unlocked)
    if t["clk_demotion_reset"][2] > 0 or anchors > 1:
        stop = "TRIPPED -- stop the round"
    elif not cross_ok:
        stop = "UNDETERMINED -- deltas not verified against the logs"
    else:
        stop = "not tripped"
    # 2026-10-24: the STOP rule itself is unchanged; the line points at the flag,
    # since leg_post.sh shows the STOP RULE line on the console and not the !! line
    print(f"STOP RULE: {stop}" + (" (!! BASE NEVER LOCKED above: this leg did not test FHSS "
                                  "reverse delivery)" if unlocked else ""))
    return 0 if all(res in ("PASS", "N/A") for _, res in rows) else 1


def never_locked(v, brackets):
    """The ``!! BASE NEVER LOCKED`` line on an FHSS leg, or None (2026-10-24, leg 6).

    ``v``: deltas() per board; ``brackets``: the parsed brackets by (board, pre/post).
    Fingerprint: base d(clk_demotion_reset) 0 (no LOCKED -> SCANNING edge, which every
    locked leg ends with) and d(fhss_dec_aligned) 0 (no decode on a held grid) while the
    tractor streamed (d(radio_tx_ok) >= STREAMED_TX), and post radio_state 4 (RX_CONT) on
    both boards; a post bracket without radio_state does not rule it out. None when a
    counter it reads is missing."""
    b, t = v["base"], v["tractor"]
    if any(k not in b for k in ("clk_demotion_reset", "fhss_dec_aligned", "radio_tx_ok")):
        return None
    streamed = t["radio_tx_ok"][2]            # radio_tx_ok / radio_rx_ok are REQUIRED
    if b["clk_demotion_reset"][2] or b["fhss_dec_aligned"][2] or streamed < STREAMED_TX:
        return None
    states = [brackets[(w, "post")].get("radio_state") for w in ("base", "tractor")]
    # any board that reports a post radio_state other than RX_CONT rules it out; only
    # a missing radio_state leaves the question open
    if any(s is not None and s != RADIO_STATE_RX_CONT for s in states):
        return None
    state = (f"post radio_state {RADIO_STATE_RX_CONT} (RX_CONT) on both boards"
             if None not in states else
             "post radio_state not in " + ("both brackets" if states == [None, None] else
                                           f"the {('base', 'tractor')[states.index(None)]} "
                                           f"bracket"))
    bt, tr = b["radio_tx_ok"][2], t["radio_rx_ok"][2]
    return (f"!! BASE NEVER LOCKED: base d(clk_demotion_reset)=0 and d(fhss_dec_aligned)=0 "
            f"while the tractor sent {streamed} (d(radio_tx_ok)); {state}. The fingerprint of a "
            f"base FHSS scan that ran out into FAILED (2026-10-24 leg 6; rs12_leg_report.py's "
            f"acquisition line): the base never followed the hop grid and heard only what "
            f"crossed the channel it was parked on. The criteria above are scored as before and "
            f"do not test FHSS reverse delivery on this leg: the tractor's clock counters stay "
            f"clean when its peer never hops, and command delivery counts only the probes the "
            f"base fired after a frame it happened to catch ({tr}/{bt}). Re-fly the leg, or void "
            f"it only under a rule written before leg 1")


def selftest_path():
    """The SIL cases, in the repo's base_station/tests (CI runs them there)."""
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.normpath(os.path.join(here, "..", "..", "..", "base_station", "tests",
                                         "test_bracket_deltas_sil.py"))


def selftest():
    import importlib.util
    import unittest

    path = selftest_path()
    if not os.path.isfile(path):
        print(f"bracket_deltas.py selftest: {path} not found (the cases live in the repo's "
              f"base_station/tests)", file=sys.stderr)
        return 2
    spec = importlib.util.spec_from_file_location("test_bracket_deltas_sil", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    res = unittest.TextTestRunner(verbosity=1).run(
        unittest.defaultTestLoader.loadTestsFromModule(mod))
    if not res.wasSuccessful():
        return 1
    print("bracket_deltas.py selftest: OK")
    return 0


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if argv == ["--selftest"]:
        return selftest()
    ap = argparse.ArgumentParser(prog="bracket_deltas.py", description=__doc__.splitlines()[0])
    ap.add_argument("legs_dir", help="the evidence legs/ folder holding leg<leg>_*.txt")
    ap.add_argument("leg", help="the leg tag, e.g. V or A2c2")
    ap.add_argument("--profile", choices=("auto", "fhss", "dts"), default="auto",
                    help="auto (default): reg_profile from leg<leg>_params.txt or --params; "
                         "fhss / dts: name it (must agree with params that give one)")
    ap.add_argument("--params", help="the leg's params.txt, or the harness archive dir holding it "
                                     "(default: <legs dir>/leg<leg>_params.txt)")
    a = ap.parse_args(argv)
    return score(a.legs_dir, a.leg, a.profile, a.params)


if __name__ == "__main__":
    sys.exit(main())
